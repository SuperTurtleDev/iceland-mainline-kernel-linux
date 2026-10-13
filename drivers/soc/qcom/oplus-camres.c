// SPDX-License-Identifier: GPL-2.0-only
/*
 * oplus-camres — camera shared I2C bus (QUP wrap2 SE1) between AP and SEE
 *
 * gpio4/5 are physically shared between the AP and the SEE (sensors
 * execution environment on the ADSP), which uses the bus for AON and
 * virtual camera sensors.  This driver exposes the bus as its own I2C
 * adapter ("oplus,camres"): camera sensor/VCM/EEPROM children hang off
 * it in DT.  Transfers are forwarded to a geni_i2c platform device that
 * is instantiated from the (otherwise disabled) i2c9 DT node only while
 * the AP holds the bus — geni_i2c itself stays unmodified.
 *
 * Ownership:
 *
 *   - SEE (default): nothing in DT occupies gpio4/5, i2c9 stays
 *     disabled, no geni device exists.  Transfers return -EBUSY.
 *
 *   - AP: a resident SNS client negotiates with SEE, then the pins are
 *     muxed to qup2_se1 and the geni device is instantiated:
 *
 *       registration (once, at service discovery):
 *           SUID lookup (msg 512, magic uid) -> 768
 *       takeover:  INIT 0x240 {name, 0, slot}  -> ACK 0x340 (error==0)
 *                  REVOKE 0x243 {}             -> ACK 0x343   [optional]
 *       release:   RELEASE 0x241 {0}           -> ACK 0x341
 *
 *     The sequence is ACK-driven: each request blocks until its ACK
 *     indication arrives (or times out); there are no sleeps.
 *
 * All SNS/QRTR/QMI wire layouts are fixed on this firmware and are
 * described by packed structs that are validated strictly — a datagram
 * that does not match is dropped with a reason logged.
 *
 * sysfs:
 *   force_start_i2c        — A: kernel negotiates with SEE, then takes
 *                            the bus.  Write 0 to drop the hold.
 *   force_start_i2c_direct — B: take the bus with no SEE calls at all
 *                            (userspace is responsible for them).
 *   ap_holds               — read current ownership (1 = AP)
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/kstrtox.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/net.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/qrtr.h>
#include <linux/uio.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>
#include <linux/oplus-camres.h>

/* ---- protocol constants (validated against libqmi/libqrtr + SEE) ---- */

#define SNS_SERVICE_ID		400
#define SNS_QMI_CONTROL		0x0020	/* QMI SSC "Control" request */
#define SNS_QMI_REPORT_SMALL	0x0021
#define SNS_QMI_REPORT_LARGE	0x0022

#define SNS_MSG_SUID_LOOKUP	512
#define SNS_MSG_SUID_RESULT	768
#define SNS_SUID_MAGIC		0xababababababababULL

/* SNS std sensor enable flow on camera_oplus_aon (captured vendor
 * sequence): attributes request (516, empty), then config (513) with
 * {320x240 @ 5 fps} — this brings the camera power domain up. */
#define SNS_MSG_SENSOR_INFO	516
#define SNS_MSG_SENSOR_CONFIG	513
#define SNS_AON_UID_LO		0xf8447cc60209c100ULL
#define SNS_AON_UID_HI		0x3e9145ace8717480ULL

#define QSH_INIT		0x240	/* attach {name, retry, slot, otp} */
#define QSH_INIT_ACK		0x340
#define QSH_RELEASE		0x241	/* hand back to SEE {power} */
#define QSH_RELEASE_ACK		0x341
#define QSH_REVOKE		0x243	/* take from SEE {} */
#define QSH_REVOKE_ACK		0x343

#define ACK_TIMEOUT_MS		2000
#define GENI_TIMEOUT_MS		2000
#define SUSPEND_TIMEOUT_MS	1000
#define LOOKUP_RETRY_MS		3000
#define SESSION_RETRY_MS	1000

#define CAMRES_ATTACH_NAME	"icelandmain"
#define CAMRES_ATTACH_SLOT	0
#define CAMRES_SSC_MAX		1024

/* protobuf tags: (field << 3) | wire type */
#define PB_BYTES(f)	(((f) << 3) | 2)
#define PB_F64(f)	(((f) << 3) | 1)
#define PB_F32(f)	(((f) << 3) | 5)
#define PB_VARINT(f)	(((f) << 3) | 0)

/* ---- SNS wire structs (fixed layouts, little-endian) ---- */

/* QMI SSC Control request SDU */
struct camres_qmi_req {
	u8 flags;			/* 0x00 = request */
	__le16 txn;
	__le16 msg_id;			/* SNS_QMI_CONTROL */
	__le16 msg_len;
	u8 report_type;			/* TLV 0x10 */
	__le16 report_len;		/* 1 */
	u8 report;			/* 1 = large reports */
	u8 data_type;			/* TLV 0x01 */
	__le16 data_len;		/* 2 + pb_len */
	__le16 pb_len;
	u8 pb[];			/* SscClientRequest protobuf */
} __packed;

/* QMI SSC Report indication SDU */
struct camres_qmi_ind {
	u8 flags;			/* 0x04 = indication */
	__le16 txn;
	__le16 msg_id;			/* SNS_QMI_REPORT_LARGE */
	__le16 msg_len;
	u8 client_type;			/* TLV 0x01 */
	__le16 client_len;		/* 8 */
	u8 client_id[8];
	u8 data_type;			/* TLV 0x02 */
	__le16 data_len;		/* 2 + pb_len */
	__le16 pb_len;
	u8 pb[];			/* SscClientResponse protobuf */
} __packed;

/* SscClientRequest{1: uid, 2: msg_id, 3: config, 4: body{2: msg}} */
struct camres_sns_req {
	u8 uid_tag;			/* PB_BYTES(1) */
	u8 uid_len;			/* 18 */
	u8 lo_tag;			/* PB_F64(1) */
	__le64 uid_lo;
	u8 hi_tag;			/* PB_F64(2) */
	__le64 uid_hi;
	u8 msg_id_tag;			/* PB_F32(2) */
	__le32 msg_id;
	u8 cfg_tag;			/* PB_BYTES(3) */
	u8 cfg_len;			/* 4 */
	u8 cfg[4];			/* {1:1, 2:0} */
	u8 body_tag;			/* PB_BYTES(4) */
	u8 body_len;			/* 2 + msg_len */
	u8 msg_tag;			/* PB_BYTES(2) */
	u8 msg_len;
	u8 msg[];
} __packed;

/*
 * SscClientResponse{1: uid, 2: body{1: msg_id, 2: ts, 3: {1: error,
 * 2: {1: state}}}} — handshake ACK shape.  The SUID registry reply
 * shares the envelope and body.1 but has a different tail, so the tail
 * is validated only when body_len says it is a handshake ACK.
 */
struct camres_sns_resp {
	u8 uid_tag;			/* PB_BYTES(1) */
	u8 uid_len;			/* 18 */
	u8 lo_tag;			/* PB_F64(1) */
	u8 uid_lo[8];
	u8 hi_tag;			/* PB_F64(2) */
	u8 uid_hi[8];
	u8 body_tag;			/* PB_BYTES(2) */
	u8 body_len;
	u8 msg_id_tag;			/* PB_F32(1) */
	__le32 msg_id;
	/* handshake tail follows (see camres_sns_ack_tail) */
} __packed;

struct camres_sns_ack_tail {
	u8 ts_tag;			/* PB_F64(2) */
	u8 ts[8];
	u8 err_tag;			/* PB_BYTES(3) */
	u8 err_len;			/* 6 */
	u8 error_tag;			/* PB_VARINT(1) */
	u8 error;
	u8 state_tag;			/* PB_BYTES(2) */
	u8 state_len;			/* 2 */
	u8 state_val_tag;		/* PB_VARINT(1) */
	u8 state;
} __packed;

#define ACK_TAIL_LEN		(5 + sizeof(struct camres_sns_ack_tail))

/* ---- driver state ---- */

struct oplus_camres {
	struct device *dev;

	struct mutex lock;
	atomic_t consumers;
	bool force;		/* sysfs hold active */
	bool sns_quiet;		/* mode B: skip SEE calls */
	bool ap_active;		/* pins muxed + geni instantiated */

	struct pinctrl *pctl;
	struct pinctrl_state *p_active;
	struct pinctrl_state *p_released;

	struct device_node *geni_np;	/* the disabled i2c9 DT node */
	struct platform_device *geni_pdev;
	struct i2c_adapter *geni_adap;
	struct device *geni_dev;

	struct i2c_adapter adapter;	/* the oplus,camres bus itself */
	struct delayed_work release_work;

	/* resident SNS client */
	struct socket *sock;
	struct sockaddr_qrtr sns_sq;
	bool sns_found;
	bool registered;	/* SNS client registered (ADSP up) */
	bool session_ready;	/* SEE camera handshake answered */
	u16 txn;
	u64 suid_hi;
	u64 suid_lo;
	char attach_name[24];
	u32 attach_slot;
	u8 ack_error;		/* from the last ACK */
	u8 ack_state;
	struct delayed_work lookup_work;
	struct delayed_work session_work;
};

static struct oplus_camres *g_camres;

/* experiment knobs (defaults come from DT) */
static bool revoke = true;
module_param(revoke, bool, 0644);
static u64 suid_hi, suid_lo;
module_param(suid_hi, ullong, 0444);
module_param(suid_lo, ullong, 0444);
static char *attach_name;
module_param(attach_name, charp, 0444);
static int attach_slot = -1;
module_param(attach_slot, int, 0444);

/* ---- protobuf emitters (only for the variable-length name field) ---- */

static size_t pb_put_bytes(u8 *b, size_t pos, unsigned int field,
			   const void *data, size_t len)
{
	b[pos++] = PB_BYTES(field);
	b[pos++] = len;			/* names are short (< 128) */
	if (len)
		memcpy(b + pos, data, len);
	return pos + len;
}

static size_t pb_put_varint(u8 *b, size_t pos, unsigned int field, u64 val)
{
	b[pos++] = PB_VARINT(field);
	b[pos++] = val;			/* values used here are < 128 */
	return pos;
}

/*
 * Message bodies:
 *   SUID lookup: {1: "camera_handshake", 2: 0, 3: 1}
 *   INIT:        {1: name, 2: 0, 3: slot, 4: ""}
 *   RELEASE:     {1: 0}
 *   REVOKE:      {} (empty)
 */
static size_t sns_body_lookup(u8 *b)
{
	size_t pos = 0;

	pos = pb_put_bytes(b, pos, 1, "camera_handshake", 16);
	pos = pb_put_varint(b, pos, 2, 0);
	pos = pb_put_varint(b, pos, 3, 1);
	return pos;
}

static size_t sns_body_init(u8 *b, const char *name, u32 slot)
{
	size_t pos = 0;

	pos = pb_put_bytes(b, pos, 1, name, strlen(name));
	pos = pb_put_varint(b, pos, 2, 0);
	pos = pb_put_varint(b, pos, 3, slot);
	pos = pb_put_bytes(b, pos, 4, NULL, 0);
	return pos;
}

/* RELEASE_CONTROL body {1: 0} */
struct camres_sns_body_release {
	u8 power_tag;		/* PB_VARINT(1) */
	u8 power;		/* 0 = hand back to SEE */
} __packed;

/* ---- SNS transport ---- */

static int sns_socket_open(struct oplus_camres *cr)
{
	int ret;

	if (cr->sock)
		return 0;

	ret = sock_create_kern(&init_net, AF_QIPCRTR, SOCK_DGRAM, 0,
			       &cr->sock);
	if (ret)
		dev_err(cr->dev, "qrtr socket: %d\n", ret);
	return ret;
}

static int sns_send(struct oplus_camres *cr, u64 uid_lo, u64 uid_hi,
		    u32 msg_id, const u8 *msg, size_t msg_len)
{
	u8 pb_buf[sizeof(struct camres_sns_req) + 32];
	u8 pkt_buf[sizeof(pb_buf) + sizeof(struct camres_qmi_req)];
	struct camres_sns_req *pb = (void *)pb_buf;
	struct camres_qmi_req *sdu = (void *)pkt_buf;
	struct sockaddr_qrtr to = cr->sns_sq;
	struct msghdr m = { .msg_name = &to, .msg_namelen = sizeof(to) };
	struct kvec iov;
	size_t pkt_len;
	int ret;

	if (!cr->sns_found)
		return -ENOTCONN;
	if (msg_len > sizeof(pb_buf) - sizeof(*pb))
		return -EMSGSIZE;

	ret = sns_socket_open(cr);
	if (ret)
		return ret;

	*pb = (struct camres_sns_req){
		.uid_tag = PB_BYTES(1),	.uid_len = 18,
		.lo_tag = PB_F64(1),	.uid_lo = cpu_to_le64(uid_lo),
		.hi_tag = PB_F64(2),	.uid_hi = cpu_to_le64(uid_hi),
		.msg_id_tag = PB_F32(2), .msg_id = cpu_to_le32(msg_id),
		.cfg_tag = PB_BYTES(3), .cfg_len = 4,
		.cfg = { PB_VARINT(1), 1, PB_VARINT(2), 0 },
		.body_tag = PB_BYTES(4), .body_len = 2 + msg_len,
		.msg_tag = PB_BYTES(2), .msg_len = msg_len,
	};
	if (msg_len)
		memcpy(pb->msg, msg, msg_len);

	sdu->flags = 0x00;
	sdu->txn = cpu_to_le16(cr->txn++);
	sdu->msg_id = cpu_to_le16(SNS_QMI_CONTROL);
	sdu->report_type = 0x10;
	sdu->report_len = cpu_to_le16(1);
	sdu->report = 1;				/* large */
	sdu->data_type = 0x01;
	sdu->pb_len = cpu_to_le16(sizeof(*pb) + msg_len);
	sdu->data_len = cpu_to_le16(sizeof(*pb) + msg_len + 2);
	sdu->msg_len = cpu_to_le16(4 + 3 + 2 + sizeof(*pb) + msg_len);
	pkt_len = sizeof(*sdu) + sizeof(*pb) + msg_len;
	memcpy(sdu->pb, pb_buf, sizeof(*pb) + msg_len);

	iov.iov_base = pkt_buf;
	iov.iov_len = pkt_len;

	ret = kernel_sendmsg(cr->sock, &m, &iov, 1, pkt_len);
	if (ret < 0)
		dev_err(cr->dev, "sns send 0x%x: %d\n", msg_id, ret);
	return ret < 0 ? ret : 0;
}

/* control-plane packets are handled by the discovery parser instead */
static bool camres_parse_ctrl(struct oplus_camres *cr, const u8 *buf,
			      size_t len);

/*
 * Validate one report indication as a response for @uid carrying
 * @expect.  Handshake ACKs also yield error/power-state; registry
 * replies only match on msg_id.  Returns a negative reason code when
 * the datagram does not match (for logging), 0 on match.
 */
static int sns_parse_ack(const u8 *buf, size_t len, u64 uid_lo, u64 uid_hi,
			 u32 expect, struct camres_sns_ack_tail *tail)
{
	const struct camres_qmi_ind *ind = (const void *)buf;
	const struct camres_sns_resp *resp;
	size_t pb_len;

	if (len < sizeof(*ind) + sizeof(*resp))
		return -1;
	if (ind->flags != 0x04)
		return -2;
	if (ind->msg_id != cpu_to_le16(SNS_QMI_REPORT_LARGE) &&
	    ind->msg_id != cpu_to_le16(SNS_QMI_REPORT_SMALL))
		return -3;
	if (ind->client_type != 0x01 || ind->client_len != cpu_to_le16(8))
		return -4;
	if (ind->data_type != 0x02)
		return -5;
	pb_len = le16_to_cpu(ind->pb_len);
	if (ind->data_len != cpu_to_le16(pb_len + 2))
		return -6;
	if (len != sizeof(*ind) + pb_len)
		return -7;
	if (pb_len < sizeof(*resp))
		return -8;

	resp = (const void *)ind->pb;
	if (resp->uid_tag != PB_BYTES(1) || resp->uid_len != 18 ||
	    resp->lo_tag != PB_F64(1) || resp->hi_tag != PB_F64(2))
		return -9;
	if (get_unaligned_le64(resp->uid_lo) != uid_lo ||
	    get_unaligned_le64(resp->uid_hi) != uid_hi)
		return -10;
	if (resp->body_tag != PB_BYTES(2) || resp->msg_id_tag != PB_F32(1))
		return -11;
	if (le32_to_cpu(resp->msg_id) != expect)
		return -12;

	if (resp->body_len == ACK_TAIL_LEN) {
		const struct camres_sns_ack_tail *t;

		if (pb_len != sizeof(*resp) + sizeof(*t))
			return -13;
		t = (const void *)(resp + 1);
		if (t->ts_tag != PB_F64(2) || t->err_tag != PB_BYTES(3) ||
		    t->err_len != 6 || t->error_tag != PB_VARINT(1) ||
		    t->state_tag != PB_BYTES(2) || t->state_len != 2 ||
		    t->state_val_tag != PB_VARINT(1))
			return -14;
		*tail = *t;
	} else {
		/* registry reply shape: no error/state tail */
		memset(tail, 0, sizeof(*tail));
	}
	return 0;
}

/*
 * Block until the ACK indication for @expect arrives.  Control-plane
 * datagrams seen while waiting are fed to the discovery parser.
 */
static int sns_wait_ack(struct oplus_camres *cr, u32 expect,
			u64 uid_lo, u64 uid_hi)
{
	u8 buf[CAMRES_SSC_MAX + 32];
	unsigned long deadline = jiffies + msecs_to_jiffies(ACK_TIMEOUT_MS);
	struct camres_sns_ack_tail tail;
	int ret;

	while (1) {
		struct sockaddr_qrtr from = { };
		struct msghdr m = { .msg_name = &from,
				    .msg_namelen = sizeof(from) };
		struct kvec iov = { buf, sizeof(buf) - 1 };

		ret = kernel_recvmsg(cr->sock, &m, &iov, 1,
				     sizeof(buf) - 1, MSG_DONTWAIT);
		if (ret > 0 && from.sq_port == QRTR_PORT_CTRL) {
			camres_parse_ctrl(cr, buf, ret);
			continue;
		}
		if (ret > 0) {
			ret = sns_parse_ack(buf, ret, uid_lo, uid_hi,
					    expect, &tail);
			if (ret == 0) {
				cr->ack_error = tail.error;
				cr->ack_state = tail.state;
				dev_info(cr->dev,
					 "sns ack 0x%x: error=%u state=%u\n",
					 expect, tail.error, tail.state);
				return 0;
			}
			dev_dbg(cr->dev, "sns datagram dropped (reason %d)\n",
				ret);
		}
		if (time_after(jiffies, deadline))
			return -ETIMEDOUT;
		msleep(20);
	}
}

/* ---- resident client: discovery + registration ---- */

/*
 * QRTR service discovery, libqrtr semantics: a QRTR_TYPE_NEW_LOOKUP
 * with an all-zero filter on our own socket; the in-kernel name service
 * replays every registered server and announces later ones.  We select
 * the SNS service ourselves.
 */
static void lookup_work_fn(struct work_struct *work)
{
	struct oplus_camres *cr = container_of(to_delayed_work(work),
					       struct oplus_camres,
					       lookup_work);
	struct qrtr_ctrl_pkt pkt = { };
	struct sockaddr_qrtr to = { };
	struct msghdr m = { .msg_name = &to, .msg_namelen = sizeof(to) };
	struct kvec iov = { &pkt, sizeof(pkt) };
	u8 buf[64];
	unsigned long deadline = jiffies + msecs_to_jiffies(300);
	int ret;

	if (cr->sns_found)
		return;

	ret = sns_socket_open(cr);
	if (ret)
		goto retry;

	ret = kernel_getsockname(cr->sock, (struct sockaddr *)&to);
	if (ret < 0)
		goto retry;
	to.sq_family = AF_QIPCRTR;
	to.sq_port = QRTR_PORT_CTRL;
	pkt.cmd = cpu_to_le32(QRTR_TYPE_NEW_LOOKUP);
	ret = kernel_sendmsg(cr->sock, &m, &iov, 1, sizeof(pkt));
	if (ret < 0)
		goto retry;

	/* drain the replay looking for the SNS service */
	while (!cr->sns_found) {
		struct sockaddr_qrtr from = { };
		struct msghdr rm = { .msg_name = &from,
				     .msg_namelen = sizeof(from) };
		struct kvec riov = { buf, sizeof(buf) - 1 };

		ret = kernel_recvmsg(cr->sock, &rm, &riov, 1,
				     sizeof(buf) - 1, MSG_DONTWAIT);
		if (ret > 0 && from.sq_port == QRTR_PORT_CTRL)
			camres_parse_ctrl(cr, buf, ret);
		if (ret <= 0) {
			if (time_after(jiffies, deadline))
				break;
			msleep(10);
		}
	}

	if (!cr->sns_found)
		goto retry;

	/* register the resident client with the SUID registry */
	{
		u8 body[32];
		size_t len = sns_body_lookup(body);

		ret = sns_send(cr, SNS_SUID_MAGIC, SNS_SUID_MAGIC,
			       SNS_MSG_SUID_LOOKUP, body, len);
		if (ret)
			goto retry;
		ret = sns_wait_ack(cr, SNS_MSG_SUID_RESULT,
				   SNS_SUID_MAGIC, SNS_SUID_MAGIC);
		if (ret) {
			dev_info(cr->dev, "sns registration: no ack\n");
			goto retry;
		}
		cr->registered = true;
		dev_info(cr->dev, "sns client registered\n");
		schedule_delayed_work(&cr->session_work, 0);
	}
	return;

retry:
	schedule_delayed_work(&cr->lookup_work,
			      msecs_to_jiffies(LOOKUP_RETRY_MS));
}

static bool camres_parse_ctrl(struct oplus_camres *cr, const u8 *buf,
			      size_t len)
{
	u32 cmd, node, port, service;

	if (len < 20)
		return false;
	cmd = get_unaligned_le32(buf);
	if (cmd != QRTR_TYPE_NEW_SERVER && cmd != QRTR_TYPE_DEL_SERVER)
		return false;

	service = get_unaligned_le32(buf + 4);
	/* instance (buf+8) ignored: wildcard, as in libqrtr */
	node = get_unaligned_le32(buf + 12);
	port = get_unaligned_le32(buf + 16);

	if (cmd == QRTR_TYPE_NEW_SERVER) {
		if (!node && !port && !service)
			return true;		/* end of replay */
		if (service == SNS_SERVICE_ID && !cr->sns_found) {
			cr->sns_sq.sq_family = AF_QIPCRTR;
			cr->sns_sq.sq_node = node;
			cr->sns_sq.sq_port = port;
			cr->sns_found = true;
			dev_info(cr->dev, "sns service at %x:%x\n", node, port);
		}
	} else if (service == SNS_SERVICE_ID && cr->sns_found &&
		   cr->sns_sq.sq_node == node && cr->sns_sq.sq_port == port) {
		bool was_ready = cr->session_ready;

		cr->sns_found = false;
		cr->registered = false;		/* SEE restart drops it */
		cr->session_ready = false;
		if (was_ready)
			kobject_uevent_env(&cr->dev->kobj, KOBJ_CHANGE,
					   (char *[]){ "CAMRES_READY=0", NULL });
		schedule_delayed_work(&cr->lookup_work, 0);
	}
	return true;
}

/*
 * Attach to SEE's camera session.  Returns 0 only when SEE acked with
 * error == 0 (the session exists and the camera power domain is up).
 */
static int sns_attach(struct oplus_camres *cr)
{
	u8 body[32];
	size_t len;
	int ret;

	if (!cr->registered)
		return -ENOTCONN;

	len = sns_body_init(body, cr->attach_name, cr->attach_slot);
	ret = sns_send(cr, cr->suid_lo, cr->suid_hi, QSH_INIT, body, len);
	if (ret)
		return ret;

	ret = sns_wait_ack(cr, QSH_INIT_ACK, cr->suid_lo, cr->suid_hi);
	if (ret)
		return ret;
	if (cr->ack_error) {
		dev_info(cr->dev, "sns attach rejected: error=%u\n",
			 cr->ack_error);
		return -EPROTO;
	}
	dev_info(cr->dev, "sns attach ok (state=%u)\n", cr->ack_state);
	return 0;
}

static int sns_simple(struct oplus_camres *cr, u32 msg, u32 ack,
		      const u8 *body, size_t len)
{
	int ret;

	if (!cr->registered)
		return -ENOTCONN;

	ret = sns_send(cr, cr->suid_lo, cr->suid_hi, msg, body, len);
	if (ret)
		return ret;

	ret = sns_wait_ack(cr, ack, cr->suid_lo, cr->suid_hi);
	if (ret)
		return ret;
	if (cr->ack_error) {
		dev_info(cr->dev, "sns 0x%x rejected: error=%u\n",
			 msg, cr->ack_error);
		return -EPROTO;
	}
	return 0;
}

/*
 * Enable the AON camera sensor on SEE (vendor-captured sequence): this
 * is what powers the camera module.  Fire-and-forget — the sensor
 * streams asynchronously via report indications we do not consume.
 */
/*
 * AON camera config body {1: res_w=320, 2: res_h=240, 3: fps=5,
 * 4: period_ms=1000, 5..7: 0,0,5} — the varint encodings of the fixed
 * values are part of the fixed wire layout.
 */
struct camres_sns_aon_cfg {
	u8 res_w_tag;	u8 res_w[2];
	u8 res_h_tag;	u8 res_h[2];
	u8 fps_tag;	u8 fps;
	u8 period_tag;	u8 period[2];
	u8 f5_tag;	u8 f5;
	u8 f6_tag;	u8 f6;
	u8 f7_tag;	u8 f7;
} __packed;

static void sns_aon_power_on(struct oplus_camres *cr)
{
	static const struct camres_sns_aon_cfg cfg = {
		.res_w_tag = PB_VARINT(1),	.res_w = { 0xc0, 0x02 },
		.res_h_tag = PB_VARINT(2),	.res_h = { 0xf0, 0x01 },
		.fps_tag = PB_VARINT(3),	.fps = 5,
		.period_tag = PB_VARINT(4),	.period = { 0xe8, 0x07 },
		.f5_tag = PB_VARINT(5),		.f5 = 0,
		.f6_tag = PB_VARINT(6),		.f6 = 0,
		.f7_tag = PB_VARINT(7),		.f7 = 5,
	};
	int ret;

	ret = sns_send(cr, SNS_AON_UID_LO, SNS_AON_UID_HI,
		       SNS_MSG_SENSOR_INFO, NULL, 0);
	if (ret)
		return;
	sns_send(cr, SNS_AON_UID_LO, SNS_AON_UID_HI,
		 SNS_MSG_SENSOR_CONFIG, (const u8 *)&cfg, sizeof(cfg));
}

/* tell SEE the AP is done with the camera (power off) */
static void sns_release_notify(struct oplus_camres *cr)
{
	static const struct camres_sns_body_release body = {
		.power_tag = PB_VARINT(1),
		.power = 0,
	};

	if (sns_simple(cr, QSH_RELEASE, QSH_RELEASE_ACK,
		       (const u8 *)&body, sizeof(body)))
		dev_warn(cr->dev, "SEE release notify failed\n");
}

/* ---- SEE camera session readiness ---- */

static int camres_reprobe_child(struct device *dev, void *data)
{
	int ret;

	if (dev->driver)
		return 0;

	ret = device_attach(dev);
	dev_dbg(dev, "camres reprobe: %d\n", ret);
	return 0;
}

/*
 * Called without cr->lock: the children's probes acquire the bus and
 * take the lock themselves.
 */
static void camres_session_up(struct oplus_camres *cr)
{
	dev_info(cr->dev, "SEE camera session up\n");
	kobject_uevent_env(&cr->dev->kobj, KOBJ_CHANGE,
			   (char *[]){ "CAMRES_READY=1", NULL });
	/* consumers that deferred on !is_ready() get no other trigger */
	device_for_each_child(&cr->adapter.dev, NULL, camres_reprobe_child);
}

/*
 * SNS registration only proves the ADSP is up.  SEE does not answer the
 * camera handshake until adsprpcd has served its camera config, so poll
 * the attach here and declare readiness only once it is acked (or once
 * a takeover already holds the bus, which proves the same thing).
 */
static void session_work_fn(struct work_struct *work)
{
	struct oplus_camres *cr = container_of(to_delayed_work(work),
					       struct oplus_camres,
					       session_work);
	int ret;

	mutex_lock(&cr->lock);
	if (cr->session_ready)
		goto out_unlock;

	if (!cr->ap_active) {
		if (!cr->registered)
			goto out_unlock;	/* lookup_work reschedules us */

		ret = sns_attach(cr);
		if (ret) {
			mutex_unlock(&cr->lock);
			dev_info(cr->dev,
				 "SEE camera session not up yet (%d)\n", ret);
			schedule_delayed_work(&cr->session_work,
					      msecs_to_jiffies(SESSION_RETRY_MS));
			return;
		}
		sns_release_notify(cr);
	}
	cr->session_ready = true;
	mutex_unlock(&cr->lock);

	camres_session_up(cr);
	return;

out_unlock:
	mutex_unlock(&cr->lock);
}

/* ---- dynamic geni_i2c instantiation ---- */

static int geni_adap_match(struct device *dev, void *data)
{
	struct oplus_camres *cr = data;

	if (dev->of_node == cr->geni_np && dev->type == &i2c_adapter_type) {
		cr->geni_adap = to_i2c_adapter(dev);
		return 1;
	}
	return 0;
}

static int geni_wait_adapter(struct oplus_camres *cr)
{
	unsigned long deadline = jiffies + msecs_to_jiffies(GENI_TIMEOUT_MS);

	do {
		cr->geni_adap = NULL;
		if (bus_for_each_dev(&i2c_bus_type, NULL, cr, geni_adap_match))
			return 0;
		msleep(20);
	} while (time_before(jiffies, deadline));

	return -ETIMEDOUT;
}

static int geni_create(struct oplus_camres *cr)
{
	struct device_node *np = cr->geni_np;
	struct platform_device *wrapper, *pdev;
	struct resource res[2];
	int ret;

	if (cr->geni_pdev)
		goto wait;

	wrapper = of_find_device_by_node(np->parent);
	if (!wrapper)
		return -ENODEV;

	pdev = platform_device_alloc("camres-i2c", PLATFORM_DEVID_AUTO);
	if (!pdev) {
		platform_device_put(wrapper);
		return -ENOMEM;
	}

	memset(res, 0, sizeof(res));
	ret = of_address_to_resource(np, 0, &res[0]);
	if (ret)
		goto err;
	of_irq_to_resource(np, 0, &res[1]);

	pdev->dev.parent = &wrapper->dev;
	pdev->dev.of_node = of_node_get(np);
	platform_device_add_resources(pdev, res, ARRAY_SIZE(res));

	ret = platform_device_add(pdev);
	platform_device_put(wrapper);
	if (ret)
		goto err;
	cr->geni_pdev = pdev;

wait:
	ret = geni_wait_adapter(cr);
	if (ret)
		return ret;
	cr->geni_dev = &cr->geni_pdev->dev;
	return 0;

err:
	/* platform_device_put releases dev.of_node (the get above) */
	platform_device_put(pdev);
	return ret;
}

/*
 * pm_runtime_disable() before unregister is load-bearing: the driver
 * core resumes the device while unbinding, and its trailing put only
 * queues an autosuspend that can never run on a dying device — the SE
 * clock prepare count would leak.  With pm disabled the core's resume
 * fails without touching hardware, so the device is freed suspended.
 */
static void geni_destroy(struct oplus_camres *cr)
{
	if (!cr->geni_pdev)
		return;

	pm_runtime_disable(&cr->geni_pdev->dev);
	/* unregistering puts dev.of_node (the ref taken in geni_create) */
	platform_device_unregister(cr->geni_pdev);
	cr->geni_pdev = NULL;
	cr->geni_adap = NULL;
	cr->geni_dev = NULL;
}

/* ---- bus ownership ---- */

static void camres_uevent(struct oplus_camres *cr, const char *owner)
{
	char env[16];
	char *envp[] = { env, NULL };

	snprintf(env, sizeof(env), "OWNERSHIP=%s", owner);
	kobject_uevent_env(&cr->dev->kobj, KOBJ_CHANGE, envp);
}

/* called with cr->lock held; @negotiate: run the SEE handshake first */
static int takeover_locked(struct oplus_camres *cr, bool negotiate)
{
	int ret;

	if (cr->ap_active)
		return 0;

	if (negotiate) {
		ret = sns_attach(cr);
		if (ret) {
			dev_warn(cr->dev, "SEE attach failed (%d)\n", ret);
			return ret;
		}
		if (revoke) {
			ret = sns_simple(cr, QSH_REVOKE, QSH_REVOKE_ACK,
					 NULL, 0);
			if (ret)
				dev_warn(cr->dev, "SEE revoke failed (%d)\n",
					 ret);
		}
		/* the AON camera powers the module; enable it on SEE */
		sns_aon_power_on(cr);
	}

	ret = pinctrl_select_state(cr->pctl, cr->p_active);
	if (ret)
		return ret;

	ret = geni_create(cr);
	if (ret)
		goto err_pins;

	ret = pm_runtime_resume_and_get(cr->geni_dev);
	if (ret)
		goto err_geni;

	cr->ap_active = true;
	dev_info(cr->dev, "camera bus acquired by AP\n");
	camres_uevent(cr, "AP");
	if (!cr->session_ready)
		schedule_delayed_work(&cr->session_work, 0);
	return 0;

err_geni:
	geni_destroy(cr);
err_pins:
	pinctrl_select_state(cr->pctl, cr->p_released);
	return ret;
}

/* called with cr->lock held; -EAGAIN asks the caller to retry */
static int release_locked(struct oplus_camres *cr)
{
	unsigned long deadline;

	if (!cr->ap_active)
		return 0;

	/* block new transfers on our adapter, then drain in-flight ones */
	WRITE_ONCE(cr->ap_active, false);

	i2c_lock_bus(&cr->adapter, I2C_LOCK_ROOT_ADAPTER);

	pm_runtime_put_sync_suspend(cr->geni_dev);
	deadline = jiffies + msecs_to_jiffies(SUSPEND_TIMEOUT_MS);
	while (!pm_runtime_suspended(cr->geni_dev)) {
		if (time_after(jiffies, deadline)) {
			pm_runtime_resume_and_get(cr->geni_dev);
			i2c_unlock_bus(&cr->adapter, I2C_LOCK_ROOT_ADAPTER);
			WRITE_ONCE(cr->ap_active, true);
			return -EAGAIN;
		}
		msleep(20);
	}

	pinctrl_select_state(cr->pctl, cr->p_released);

	i2c_unlock_bus(&cr->adapter, I2C_LOCK_ROOT_ADAPTER);

	geni_destroy(cr);

	dev_info(cr->dev, "camera bus released to SEE\n");
	if (!cr->sns_quiet)
		sns_release_notify(cr);
	camres_uevent(cr, "SEE");
	return 0;
}

static void release_work_fn(struct work_struct *work)
{
	struct oplus_camres *cr = container_of(to_delayed_work(work),
					       struct oplus_camres,
					       release_work);

	mutex_lock(&cr->lock);
	if (!atomic_read(&cr->consumers) && !cr->force && cr->ap_active) {
		if (release_locked(cr))
			schedule_delayed_work(&cr->release_work,
					      msecs_to_jiffies(200));
	}
	mutex_unlock(&cr->lock);
}

/* ---- the camres i2c adapter ---- */

static int camres_i2c_xfer(struct i2c_adapter *adap,
			   struct i2c_msg *msgs, int num)
{
	struct oplus_camres *cr = i2c_get_adapdata(adap);

	if (!READ_ONCE(cr->ap_active))
		return -EBUSY;

	return i2c_transfer(cr->geni_adap, msgs, num);
}

static u32 camres_i2c_func(struct i2c_adapter *adap)
{
	struct oplus_camres *cr = i2c_get_adapdata(adap);

	if (cr->geni_adap && cr->geni_adap->algo)
		return cr->geni_adap->algo->functionality(cr->geni_adap);

	/* match geni_i2c's set so client capability checks stay truthful */
	return I2C_FUNC_I2C | (I2C_FUNC_SMBUS_EMUL & ~I2C_FUNC_SMBUS_QUICK);
}

static const struct i2c_algorithm camres_i2c_algo = {
	.xfer = camres_i2c_xfer,
	.functionality = camres_i2c_func,
};

/* ---- public API ---- */

int oplus_camres_acquire(struct device *consumer)
{
	struct oplus_camres *cr = g_camres;
	int ret;

	if (!cr)
		return -EPROBE_DEFER;

	mutex_lock(&cr->lock);
	ret = takeover_locked(cr, true);
	if (!ret)
		atomic_inc(&cr->consumers);
	mutex_unlock(&cr->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(oplus_camres_acquire);

int oplus_camres_release(struct device *consumer)
{
	struct oplus_camres *cr = g_camres;

	if (!cr)
		return -ENODEV;

	if (atomic_dec_if_positive(&cr->consumers) == 0 && !cr->force)
		schedule_delayed_work(&cr->release_work, 0);
	return 0;
}
EXPORT_SYMBOL_GPL(oplus_camres_release);

bool oplus_camres_ap_holds(void)
{
	return g_camres ? READ_ONCE(g_camres->ap_active) : false;
}
EXPORT_SYMBOL_GPL(oplus_camres_ap_holds);

/*
 * True once SEE has answered the camera handshake (which needs the ADSP
 * up *and* adsprpcd serving).  Consumers defer their probe while this
 * is false; when it turns true camres reprobes its unbound children
 * and emits a KOBJ_CHANGE uevent (CAMRES_READY=1).
 */
bool oplus_camres_is_ready(void)
{
	struct oplus_camres *cr = g_camres;

	return cr && READ_ONCE(cr->session_ready);
}
EXPORT_SYMBOL_GPL(oplus_camres_is_ready);

/* ---- sysfs ---- */

static int force_set(struct oplus_camres *cr, bool val, bool direct)
{
	int ret = 0;

	if (val && !cr->force) {
		cr->force = true;
		cr->sns_quiet = direct;
		ret = takeover_locked(cr, !direct);
		if (ret)
			cr->force = false;
	} else if (!val && cr->force) {
		cr->force = false;
		if (!atomic_read(&cr->consumers))
			schedule_delayed_work(&cr->release_work, 0);
	}
	return ret;
}

static ssize_t force_start_i2c_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct oplus_camres *cr = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", cr->force && !cr->sns_quiet);
}

static ssize_t force_start_i2c_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct oplus_camres *cr = dev_get_drvdata(dev);
	bool val;
	int ret;

	ret = kstrtobool(buf, &val);
	if (ret)
		return ret;

	mutex_lock(&cr->lock);
	ret = force_set(cr, val, false);
	mutex_unlock(&cr->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(force_start_i2c);

static ssize_t force_start_i2c_direct_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct oplus_camres *cr = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", cr->force && cr->sns_quiet);
}

static ssize_t force_start_i2c_direct_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct oplus_camres *cr = dev_get_drvdata(dev);
	bool val;
	int ret;

	ret = kstrtobool(buf, &val);
	if (ret)
		return ret;

	mutex_lock(&cr->lock);
	ret = force_set(cr, val, true);
	mutex_unlock(&cr->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(force_start_i2c_direct);

static ssize_t ap_holds_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%d\n", oplus_camres_ap_holds());
}
static DEVICE_ATTR_RO(ap_holds);

static struct attribute *camres_attrs[] = {
	&dev_attr_force_start_i2c.attr,
	&dev_attr_force_start_i2c_direct.attr,
	&dev_attr_ap_holds.attr,
	NULL,
};
ATTRIBUTE_GROUPS(camres);

/* ---- platform driver ---- */

static int oplus_camres_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct oplus_camres *cr;
	const char *name = NULL;
	int ret;

	cr = devm_kzalloc(dev, sizeof(*cr), GFP_KERNEL);
	if (!cr)
		return -ENOMEM;

	cr->dev = dev;
	cr->suid_hi = 0xa7167528f835f4eeULL;	/* camera_handshake */
	cr->suid_lo = 0xe0e3f77b504b96a1ULL;
	strscpy(cr->attach_name, CAMRES_ATTACH_NAME, sizeof(cr->attach_name));
	cr->attach_slot = CAMRES_ATTACH_SLOT;
	mutex_init(&cr->lock);
	atomic_set(&cr->consumers, 0);
	INIT_DELAYED_WORK(&cr->release_work, release_work_fn);
	INIT_DELAYED_WORK(&cr->lookup_work, lookup_work_fn);
	INIT_DELAYED_WORK(&cr->session_work, session_work_fn);
	platform_set_drvdata(pdev, cr);

	cr->geni_np = of_parse_phandle(dev->of_node, "i2c-bus", 0);
	if (!cr->geni_np)
		return dev_err_probe(dev, -EINVAL, "missing i2c-bus phandle\n");

	of_property_read_u64(dev->of_node, "sns,suid-hi", &cr->suid_hi);
	of_property_read_u64(dev->of_node, "sns,suid-lo", &cr->suid_lo);
	of_property_read_string(dev->of_node, "sns,attach-name", &name);
	if (name)
		strscpy(cr->attach_name, name, sizeof(cr->attach_name));
	of_property_read_u32(dev->of_node, "sns,attach-slot", &cr->attach_slot);

	/* experiment overrides */
	if (suid_hi)
		cr->suid_hi = suid_hi;
	if (suid_lo)
		cr->suid_lo = suid_lo;
	if (attach_name && attach_name[0])
		strscpy(cr->attach_name, attach_name,
			sizeof(cr->attach_name));
	if (attach_slot >= 0)
		cr->attach_slot = attach_slot;

	cr->pctl = devm_pinctrl_get(dev);
	if (IS_ERR(cr->pctl))
		return dev_err_probe(dev, PTR_ERR(cr->pctl),
				     "failed to get pinctrl\n");

	cr->p_active = pinctrl_lookup_state(cr->pctl, "active");
	if (IS_ERR(cr->p_active))
		return dev_err_probe(dev, PTR_ERR(cr->p_active),
				     "missing active pinctrl state\n");

	cr->p_released = pinctrl_lookup_state(cr->pctl, "released");
	if (IS_ERR(cr->p_released))
		return dev_err_probe(dev, PTR_ERR(cr->p_released),
				     "missing released pinctrl state\n");

	/*
	 * Boot state: the bus belongs to SEE.  Nothing in DT occupies
	 * gpio4/5, so the bootloader's high-Z simply persists — no
	 * pinctrl action here.  States are only selected around
	 * takeovers.
	 */

	cr->adapter.owner = THIS_MODULE;
	cr->adapter.algo = &camres_i2c_algo;
	cr->adapter.dev.parent = dev;
	cr->adapter.dev.of_node = dev->of_node;
	cr->adapter.nr = -1;
	strscpy(cr->adapter.name, "oplus-camres-i2c",
		sizeof(cr->adapter.name));
	i2c_set_adapdata(&cr->adapter, cr);

	ret = i2c_add_adapter(&cr->adapter);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add i2c adapter\n");

	/* resident SNS client: discover + register in the background */
	schedule_delayed_work(&cr->lookup_work, 0);

	g_camres = cr;
	dev_info(dev, "camera bus ready, released to SEE (bus %d)\n",
		 i2c_adapter_id(&cr->adapter));
	return 0;
}

static void oplus_camres_remove(struct platform_device *pdev)
{
	struct oplus_camres *cr = platform_get_drvdata(pdev);

	g_camres = NULL;
	cancel_delayed_work_sync(&cr->release_work);
	cancel_delayed_work_sync(&cr->lookup_work);
	cancel_delayed_work_sync(&cr->session_work);

	mutex_lock(&cr->lock);
	if (cr->ap_active) {
		i2c_lock_bus(&cr->adapter, I2C_LOCK_ROOT_ADAPTER);
		pm_runtime_put_sync_suspend(cr->geni_dev);
		pinctrl_select_state(cr->pctl, cr->p_released);
		i2c_unlock_bus(&cr->adapter, I2C_LOCK_ROOT_ADAPTER);
		geni_destroy(cr);
		cr->ap_active = false;
	}
	mutex_unlock(&cr->lock);

	i2c_del_adapter(&cr->adapter);
	if (cr->sock)
		sock_release(cr->sock);
	of_node_put(cr->geni_np);
}

static const struct of_device_id oplus_camres_of_match[] = {
	{ .compatible = "oplus,camres" },
	{ }
};
MODULE_DEVICE_TABLE(of, oplus_camres_of_match);

static struct platform_driver oplus_camres_driver = {
	.probe = oplus_camres_probe,
	.remove = oplus_camres_remove,
	.driver = {
		.name = "oplus-camres",
		.of_match_table = oplus_camres_of_match,
		.dev_groups = camres_groups,
	},
};
module_platform_driver(oplus_camres_driver);

MODULE_AUTHOR("Iceland mainline");
MODULE_DESCRIPTION("OPlus camera shared I2C bus between AP and SEE");
MODULE_LICENSE("GPL");
