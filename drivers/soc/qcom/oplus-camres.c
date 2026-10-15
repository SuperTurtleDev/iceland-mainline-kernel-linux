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
 *   - Boot (UNCONFIGURED): the AP takes the bus right away and holds
 *     it — the camera works with zero userspace.  The AON subscribe
 *     that powers the camera module is run best-effort in the
 *     background.
 *
 *   - Userspace that wants proper arbitration opens /dev/camres and
 *     choreographs the one protocol step that is genuinely its
 *     business (the INIT handshake, which carries project camera
 *     parameters):
 *
 *         ioctl(DEINIT)  — blocks until every AP user is done, then
 *                          unsubscribes AON; this fd becomes the owner
 *         ... INIT handshake with SEE, from userspace ...
 *         ioctl(INIT, ack_rc)
 *             ack == 0   — CONFIGURED: per-use REVOKE/RELEASE
 *                          arbitration via runtime PM
 *             ack != 0   — back to UNCONFIGURED (vendor tolerance)
 *
 *     If the owner fd dies before INIT the bus enters ERR: acquires
 *     fail until a fresh DEINIT/INIT cycle recovers it.  The kernel
 *     never assumes anything about userspace liveness.
 *
 *   - AP users never call into this driver: children hanging below
 *     the adapter get a runtime-PM device link (supplier ordering),
 *     and probe-time I2C — which bypasses runtime PM, as i2c sensor
 *     drivers power on directly in probe — is covered by votes taken
 *     at the exact bus-notifier boundaries BIND_DRIVER (probe is
 *     about to run) and dropped at BOUND_DRIVER / BIND_FAILED.  A
 *     child with no driver in the system never takes a vote, so the
 *     bus is free for the SEE whenever the AP has nothing to do.
 *
 *     The public acquire/release API stays exported for drivers with
 *     exotic probe strategies of their own.
 *
 * All SNS/QRTR/QMI wire layouts are fixed on this firmware and are
 * described by packed structs that are validated strictly — a datagram
 * that does not match is dropped with a reason and hex logged.
 *
 * sysfs:
 *   ap_hold (0600, DAC only) — manual ownership vote; write 1 to hold,
 *                              0 to drop the manual vote (the bus then
 *                              returns to the SEE once idle, unless
 *                              UNCONFIGURED, where it is not
 *                              supported)
 */

#include <linux/delay.h>
#include <linux/cleanup.h>
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/kstrtox.h>
#include <linux/miscdevice.h>
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
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <net/sock.h>
#include <linux/uaccess.h>
#include <linux/uio.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>
#include <linux/oplus-camres.h>
#include <uapi/linux/oplus-camres.h>

/* ---- protocol constants (validated against libqmi/libqrtr + SEE) ---- */

#define SNS_SERVICE_ID		400
#define SNS_QMI_CONTROL		0x0020	/* QMI SSC "Control" request */
#define SNS_QMI_REPORT_SMALL	0x0021
#define SNS_QMI_REPORT_LARGE	0x0022

#define SNS_MSG_SUID_LOOKUP	512
#define SNS_MSG_SUID_RESULT	768
#define SNS_SUID_MAGIC		0xababababababababULL

/* camera_oplus_aon subscribe (captured vendor sequence; on the wire
 * these are the qsh_camera_oplus 0x204 algo query and 0x201 sensor
 * subscribe): attributes request (516, empty), then config (513) with
 * {320x240 @ 5 fps} — this is what powers the camera module, and its
 * ACK 0x20D (error==0) is the precondition for REVOKE. */
#define SNS_MSG_SENSOR_INFO	516
#define SNS_MSG_SENSOR_CONFIG	513
#define SNS_MSG_SENSOR_CONFIG_ACK 0x20D
#define SNS_MSG_SENSOR_DISABLE	10
#define SNS_AON_UID_LO		0xf8447cc60209c100ULL
#define SNS_AON_UID_HI		0x3e9145ace8717480ULL

#define QSH_RELEASE		0x241	/* hand back to SEE {power} */
#define QSH_RELEASE_ACK		0x341
#define QSH_REVOKE		0x243	/* take from SEE {} */
#define QSH_REVOKE_ACK		0x343

#define ACK_TIMEOUT_MS		2000
#define GENI_TIMEOUT_MS		2000
#define SUSPEND_TIMEOUT_MS	1000
#define LOOKUP_RETRY_MS		3000
#define LOOKUP_REPLAY_MS	300
#define AON_RETRY_MS		1000
#define AON_BOOT_TRIES		10
#define LOOKUP_BOOT_TRIES	30	/* ~90s at LOOKUP_RETRY_MS */
#define RELEASE_IDLE_MS		2000

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

/*
 * REVOKE ack tail (0x343): same {2: ts, 3: {...}} envelope as the
 * other handshake acks, but field 3 carries {1: error, 5..8:
 * optional AON stats blobs} instead of a nested power/state
 * submessage (captured: body 22, err_len 6 = "08 00" + "3a 02 08 00").
 * Only the fixed prefix is validated; the stats blobs are ignored.
 */
struct camres_sns_revoke_tail {
	u8 ts_tag;			/* PB_F64(2) */
	u8 ts[8];
	u8 err_tag;			/* PB_BYTES(3) */
	u8 err_len;			/* >= 2, grows with stats blobs */
	u8 error_tag;			/* PB_VARINT(1) */
	u8 error;
} __packed;

/* ---- driver state ---- */

/*
 * Bus lifecycle.  The kernel never binds itself to a userspace
 * process: state transitions are driven by file objects and ioctls.
 *
 *   UNCONFIGURED  boot state: AP holds the bus, camera works with no
 *                 userspace; AON subscribed best-effort
 *   CONFIGURING   a DEINIT is (or was) in flight: acquires queue up
 *   CONFIGURED    INIT succeeded: per-use REVOKE/RELEASE arbitration
 *   ERR           the DEINIT owner died before INIT: acquires fail
 *                 until a fresh DEINIT/INIT cycle recovers the bus
 */
enum camres_state {
	CAMRES_UNCONFIGURED,
	CAMRES_CONFIGURING,
	CAMRES_CONFIGURED,
	CAMRES_ERR,
};

/* per-fd state of the misc device */
struct camres_file {
	struct oplus_camres *cr;
	bool vote;		/* this fd holds an ACQUIRE vote */
	bool owner;		/* this fd is the deinit owner */
};

/* per-child probe/link tracking */
struct camres_child {
	struct list_head node;
	struct device *dev;
	struct device_link *link;
	bool probe_vote;	/* vote held across an ongoing probe */
};

struct oplus_camres {
	struct device *dev;

	struct mutex lock;
	struct mutex manual_lock;	/* serializes the sysfs manual vote */
	atomic_t consumers;	/* runtime/ACQUIRE/probe/manual votes */
	bool manual_ap;		/* sysfs manual vote active */
	enum camres_state state;
	struct camres_file *owner;	/* CONFIGURING deinit owner */
	bool deinit_done;	/* CONFIGURING: AON unsubscribed, wait done */
	bool ap_active;		/* pins muxed + geni instantiated */
	wait_queue_head_t state_wq;	/* CONFIGURING waiters (acquires) */

	struct pinctrl *pctl;
	struct pinctrl_state *p_active;
	struct pinctrl_state *p_released;

	struct device_node *geni_np;	/* the disabled i2c9 DT node */
	struct platform_device *geni_pdev;
	struct i2c_adapter *geni_adap;
	struct device *geni_dev;

	struct i2c_adapter adapter;	/* the oplus,camres bus itself */
	struct delayed_work release_work;
	struct notifier_block nb;	/* i2c bus notifier (children) */
	struct list_head children;	/* camres_child list */

	struct miscdevice misc;

	/* resident SNS client */
	struct socket *sock;
	void (*saved_data_ready)(struct sock *sk);
	struct work_struct rx_work;	/* drains the socket on data-ready */
	wait_queue_head_t rx_wq;	/* discovery: sns_found changes */
	struct mutex sns_lock;		/* one request/ack transaction at a time */
	spinlock_t ack_lock;		/* the armed-ack state below */
	struct completion ack_done;
	u32 ack_expect;			/* msg id awaited, 0 = none */
	u64 ack_uid_lo;
	u64 ack_uid_hi;
	struct sockaddr_qrtr sns_sq;
	bool sns_found;
	u8 ack_error;		/* from the last ACK */
	u8 ack_state;
	bool boot_done;		/* adapter registered, children visible */
	int aon_tries;
	int lookup_tries;
	bool registered;	/* SNS client registered (ADSP up) */
	bool aon_armed;		/* AON subscribe acked — REVOKE precondition */
	u16 txn;
	u64 suid_hi;
	u64 suid_lo;
	struct delayed_work lookup_work;
	struct delayed_work aon_work;
};

static struct oplus_camres *g_camres;

/* experiment knobs */
static bool revoke = true;
module_param(revoke, bool, 0644);
static u64 suid_hi, suid_lo;
module_param(suid_hi, ullong, 0444);
module_param(suid_lo, ullong, 0444);

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

/* RELEASE_CONTROL body {1: 0} */
struct camres_sns_body_release {
	u8 power_tag;		/* PB_VARINT(1) */
	u8 power;		/* 0 = hand back to SEE */
} __packed;

/* ---- SNS transport ---- */

static void camres_data_ready(struct sock *sk);

static int sns_socket_open(struct oplus_camres *cr)
{
	struct sock *sk;
	int ret;

	if (cr->sock)
		return 0;

	ret = sock_create_kern(&init_net, AF_QIPCRTR, SOCK_DGRAM, 0,
			       &cr->sock);
	if (ret) {
		dev_err(cr->dev, "qrtr socket: %d\n", ret);
		return ret;
	}

	/* event-driven receive, as net/qrtr/ns.c does for its socket */
	sk = cr->sock->sk;
	write_lock_bh(&sk->sk_callback_lock);
	sk->sk_user_data = cr;
	cr->saved_data_ready = sk->sk_data_ready;
	sk->sk_data_ready = camres_data_ready;
	write_unlock_bh(&sk->sk_callback_lock);
	return 0;
}

static void sns_socket_close(struct oplus_camres *cr)
{
	struct sock *sk;

	if (!cr->sock)
		return;

	sk = cr->sock->sk;
	write_lock_bh(&sk->sk_callback_lock);
	sk->sk_data_ready = cr->saved_data_ready;
	sk->sk_user_data = NULL;
	write_unlock_bh(&sk->sk_callback_lock);

	cancel_work_sync(&cr->rx_work);
	sock_release(cr->sock);
	cr->sock = NULL;
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
		if (t->ts_tag == PB_F64(2) && t->err_tag == PB_BYTES(3) &&
		    t->err_len == 6 && t->error_tag == PB_VARINT(1) &&
		    t->state_tag == PB_BYTES(2) && t->state_len == 2 &&
		    t->state_val_tag == PB_VARINT(1)) {
			*tail = *t;
			return 0;
		}
		if (expect != QSH_REVOKE_ACK)
			return -14;
	}

	if (expect == QSH_REVOKE_ACK) {
		const struct camres_sns_revoke_tail *t;

		/* {2: ts, 3: {1: error, 5..8: optional stats}} */
		if (resp->body_len < 5 + sizeof(*t) ||
		    pb_len < sizeof(*resp) + sizeof(*t))
			return -15;
		t = (const void *)(resp + 1);
		if (t->ts_tag != PB_F64(2) || t->err_tag != PB_BYTES(3) ||
		    t->err_len < 2 || t->error_tag != PB_VARINT(1))
			return -16;
		memset(tail, 0, sizeof(*tail));
		tail->error = t->error;
		return 0;
	}

	/* registry reply shape: no error/state tail */
	memset(tail, 0, sizeof(*tail));
	return 0;
}

/* softirq context: just kick the drain work */
static void camres_data_ready(struct sock *sk)
{
	struct oplus_camres *cr;

	read_lock_bh(&sk->sk_callback_lock);
	cr = sk->sk_user_data;
	if (cr)
		schedule_work(&cr->rx_work);
	read_unlock_bh(&sk->sk_callback_lock);
}

/* a data-plane datagram: complete the armed transaction if it matches */
static void camres_rx_ack(struct oplus_camres *cr, const u8 *buf, size_t len)
{
	struct camres_sns_ack_tail tail;
	int ret;

	spin_lock(&cr->ack_lock);
	if (!cr->ack_expect) {
		ret = -ENOENT;
		goto out;
	}
	ret = sns_parse_ack(buf, len, cr->ack_uid_lo, cr->ack_uid_hi,
			    cr->ack_expect, &tail);
	if (ret)
		goto out;

	cr->ack_error = tail.error;
	cr->ack_state = tail.state;
	dev_info(cr->dev, "sns ack 0x%x: error=%u state=%u\n",
		 cr->ack_expect, tail.error, tail.state);
	cr->ack_expect = 0;
	complete(&cr->ack_done);
out:
	spin_unlock(&cr->ack_lock);
	if (ret) {
		dev_dbg(cr->dev, "sns datagram dropped (reason %d)\n", ret);
		print_hex_dump_debug("sns drop: ", DUMP_PREFIX_OFFSET, 16, 1,
				     buf, min_t(size_t, len, 128), true);
	}
}

/*
 * Drain everything queued on the socket.  Control-plane datagrams go
 * to the discovery parser, the rest to the ack matcher.  Runs whenever
 * the socket signals data-ready, so nobody has to poll.
 */
static void rx_work_fn(struct work_struct *work)
{
	struct oplus_camres *cr = container_of(work, struct oplus_camres,
					       rx_work);
	u8 *buf __free(kfree) = kmalloc(CAMRES_SSC_MAX + 32, GFP_KERNEL);
	int ret;

	if (!buf)
		return;

	while (1) {
		struct sockaddr_qrtr from = { };
		struct msghdr m = { .msg_name = &from,
				    .msg_namelen = sizeof(from) };
		struct kvec iov = { buf, CAMRES_SSC_MAX + 31 };

		ret = kernel_recvmsg(cr->sock, &m, &iov, 1, iov.iov_len,
				     MSG_DONTWAIT);
		if (ret <= 0)
			break;

		if (from.sq_port == QRTR_PORT_CTRL) {
			camres_parse_ctrl(cr, buf, ret);
			wake_up(&cr->rx_wq);
		} else {
			camres_rx_ack(cr, buf, ret);
		}
	}
}

/*
 * One request/ack transaction: arm the matcher, send, wait.  Arming
 * before the send closes the window where a fast ack could arrive
 * before anyone expects it.
 */
static int sns_xfer(struct oplus_camres *cr, u64 uid_lo, u64 uid_hi,
		    u32 msg_id, const u8 *msg, size_t msg_len, u32 expect)
{
	int ret;

	mutex_lock(&cr->sns_lock);

	spin_lock(&cr->ack_lock);
	reinit_completion(&cr->ack_done);
	cr->ack_expect = expect;
	cr->ack_uid_lo = uid_lo;
	cr->ack_uid_hi = uid_hi;
	spin_unlock(&cr->ack_lock);

	ret = sns_send(cr, uid_lo, uid_hi, msg_id, msg, msg_len);
	if (!ret && !wait_for_completion_timeout(&cr->ack_done,
					msecs_to_jiffies(ACK_TIMEOUT_MS)))
		ret = -ETIMEDOUT;

	spin_lock(&cr->ack_lock);
	cr->ack_expect = 0;
	spin_unlock(&cr->ack_lock);

	mutex_unlock(&cr->sns_lock);
	return ret;
}

/* ---- resident client: discovery + registration ---- */

/*
 * QRTR service discovery, libqrtr semantics: a QRTR_TYPE_NEW_LOOKUP
 * with an all-zero filter on our own socket; the in-kernel name service
 * replays every registered server and announces later ones.  We select
 * the SNS service ourselves.
 */
static void camres_boot_done(struct oplus_camres *cr);

static void lookup_work_fn(struct work_struct *work)
{
	struct oplus_camres *cr = container_of(to_delayed_work(work),
					       struct oplus_camres,
					       lookup_work);
	struct qrtr_ctrl_pkt pkt = { };
	struct sockaddr_qrtr to = { };
	struct msghdr m = { .msg_name = &to, .msg_namelen = sizeof(to) };
	struct kvec iov = { &pkt, sizeof(pkt) };
	int ret;

	/*
	 * 'registered', not 'sns_found': after a SEE restart the service
	 * may reappear before this work runs — sns_found alone would
	 * silently skip the SUID re-registration and kill arbitration.
	 */
	if (cr->registered)
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

	/* rx_work parses the replay; wait for it to spot the SNS service */
	if (!wait_event_timeout(cr->rx_wq, cr->sns_found,
				msecs_to_jiffies(LOOKUP_REPLAY_MS)))
		goto retry;

	/* register the resident client with the SUID registry */
	{
		u8 body[32];
		size_t len = sns_body_lookup(body);

		ret = sns_xfer(cr, SNS_SUID_MAGIC, SNS_SUID_MAGIC,
			       SNS_MSG_SUID_LOOKUP, body, len,
			       SNS_MSG_SUID_RESULT);
		if (ret) {
			dev_info(cr->dev, "sns registration: no ack\n");
			goto retry;
		}
		cr->registered = true;
		dev_info(cr->dev, "sns client registered\n");
		schedule_delayed_work(&cr->aon_work, 0);
	}
	return;

retry:
	/*
	 * A SEE that never appears (ADSP down, firmware absent) must not
	 * keep the whole camera subsystem unregistered forever: after a
	 * generous window settle the boot without it — the bus comes up
	 * unpowered, which is all this hardware can do in that state.
	 */
	if (!cr->boot_done && ++cr->lookup_tries >= LOOKUP_BOOT_TRIES) {
		dev_warn(cr->dev,
			 "SEE service never appeared: bringing the bus up unpowered\n");
		camres_boot_done(cr);
		return;
	}
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
		dev_info(cr->dev, "SEE service restart\n");
		cr->sns_found = false;
		cr->registered = false;		/* SEE restart drops it */
		cr->aon_armed = false;
		/*
		 * The arbitration session died with the SEE.  Fall back to
		 * the unconfigured (AP-held) mode; userspace re-runs the
		 * DEINIT/INIT cycle on the uevent if it wants arbitration.
		 */
		mutex_lock(&cr->lock);
		if (cr->state == CAMRES_CONFIGURED)
			cr->state = CAMRES_UNCONFIGURED;
		mutex_unlock(&cr->lock);
		kobject_uevent_env(&cr->dev->kobj, KOBJ_CHANGE,
				   (char *[]){ "CAMRES_SEE_RESTART=1", NULL });
		schedule_delayed_work(&cr->lookup_work, 0);
	}
	return true;
}

static int sns_simple(struct oplus_camres *cr, u32 msg, u32 ack,
		      const u8 *body, size_t len)
{
	int ret;

	if (!cr->registered)
		return -ENOTCONN;

	ret = sns_xfer(cr, cr->suid_lo, cr->suid_hi, msg, body, len, ack);
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
 * is what powers the camera module, once, at session bring-up.  The
 * subscribe is kept standing for the whole session — its ACK (0x20D,
 * error==0) is also the precondition for REVOKE, and after a successful
 * REVOKE the SEE side no longer accepts AON requests.  The sensor
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

static int sns_aon_power_on(struct oplus_camres *cr)
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

	/* attributes request first; reply is an asynchronous event */
	ret = sns_send(cr, SNS_AON_UID_LO, SNS_AON_UID_HI,
		       SNS_MSG_SENSOR_INFO, NULL, 0);
	if (ret)
		return ret;

	return sns_xfer(cr, SNS_AON_UID_LO, SNS_AON_UID_HI,
			SNS_MSG_SENSOR_CONFIG, (const u8 *)&cfg, sizeof(cfg),
			SNS_MSG_SENSOR_CONFIG_ACK);
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

/* ---- AON subscribe (camera module power) ---- */

static int takeover_locked(struct oplus_camres *cr, bool negotiate);

/*
 * Unsubscribe the AON camera report (disable message 10, the wire form
 * libssc's close uses).  Fire-and-forget like the userspace client.
 */
static void sns_aon_power_off(struct oplus_camres *cr)
{
	if (!cr->registered || !cr->aon_armed)
		return;

	if (sns_send(cr, SNS_AON_UID_LO, SNS_AON_UID_HI,
		     SNS_MSG_SENSOR_DISABLE, NULL, 0))
		dev_warn(cr->dev, "SEE AON unsubscribe failed\n");
	cr->aon_armed = false;
}

/*
 * AON subscribe (camera module power), retried in the background.
 * Runs at exactly three points: the boot chain (before the bus is
 * ever taken), a SEE restart (deferred until active users hand the
 * bus back), and right after a userspace INIT cycle.  The subscribe
 * needs the bus free — the SEE rejects it while the camera is
 * "acquired" — so while users hold the bus this just retries cheaply
 * until they are done.
 *
 * During boot the bus is not registered yet: children cannot vote,
 * so the retry window is naturally conflict-free, and the adapter is
 * only added once the module power question is settled (or given up
 * on, when the SEE never answers at all).
 */
static void aon_work_fn(struct work_struct *work)
{
	struct oplus_camres *cr = container_of(to_delayed_work(work),
					       struct oplus_camres, aon_work);
	bool boot_phase = !cr->boot_done;
	int ret;

	if (!cr->registered)
		return;

	/*
	 * Already armed but the boot settlement has not happened (a
	 * previous camres_boot_done attempt failed) — finish the boot
	 * instead of dropping the retry on the floor.
	 */
	if (cr->aon_armed) {
		if (!cr->boot_done)
			camres_boot_done(cr);
		return;
	}

	if (cr->state == CAMRES_CONFIGURING) {
		/* DEINIT in flight: resubscribe happens at INIT */
		if (boot_phase)
			schedule_delayed_work(&cr->aon_work,
					      msecs_to_jiffies(AON_RETRY_MS));
		return;
	}

	if (cr->ap_active) {
		/*
		 * SEE would reject the subscribe while the camera is
		 * acquired.  In CONFIGURED the bus returns to the SEE on
		 * idle, so keep retrying; in UNCONFIGURED it never does
		 * until a DEINIT — stop instead of spinning (the next
		 * DEINIT/INIT cycle re-subscribes).
		 */
		if (READ_ONCE(cr->state) == CAMRES_CONFIGURED)
			schedule_delayed_work(&cr->aon_work,
					      msecs_to_jiffies(AON_RETRY_MS));
		else
			dev_info_ratelimited(cr->dev,
					     "AON subscribe deferred: bus held (unconfigured)\n");
		return;
	}

	ret = sns_aon_power_on(cr);
	if (ret) {
		dev_info(cr->dev, "SEE AON subscribe pending (%d)\n", ret);
		if (boot_phase && ++cr->aon_tries >= AON_BOOT_TRIES) {
			/* the SEE never answers (ADSP down, no userspace
			 * serving): settle the boot question without it —
			 * the module stays unpowered, which is the best
			 * the hardware allows in this situation anyway */
			dev_warn(cr->dev,
				 "SEE silent: bringing the bus up unpowered\n");
			camres_boot_done(cr);
			return;
		}
		schedule_delayed_work(&cr->aon_work,
				      msecs_to_jiffies(AON_RETRY_MS));
		return;
	}
	cr->aon_armed = true;
	dev_info(cr->dev, "SEE AON subscribed (camera module powered)\n");

	if (boot_phase)
		camres_boot_done(cr);
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

/* called with cr->lock held; @negotiate: run the SEE takeover sequence */
static int takeover_locked(struct oplus_camres *cr, bool negotiate)
{
	int ret;

	if (cr->ap_active)
		return 0;

	if (negotiate && revoke && cr->state == CAMRES_CONFIGURED &&
	    cr->aon_armed) {
		/*
		 * REVOKE is only answerable while the AON subscribe
		 * stands, and only from a valid handshake session (which
		 * userspace established via INIT) — an out-of-session
		 * REVOKE takes the SEE service down with it.  AON requests
		 * die after a successful REVOKE, so there is nothing to
		 * (re)enable.
		 */
		ret = sns_simple(cr, QSH_REVOKE, QSH_REVOKE_ACK, NULL, 0);
		if (ret)
			dev_warn(cr->dev, "SEE revoke failed (%d)\n", ret);
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
	if (cr->state == CAMRES_CONFIGURED)
		sns_release_notify(cr);
	camres_uevent(cr, "SEE");
	/*
	 * After a SEE restart the AON subscribe (and with it the module
	 * power) is gone; re-arm it now that SEE owns the bus again.
	 */
	if (cr->registered && !cr->aon_armed)
		schedule_delayed_work(&cr->aon_work, 0);
	return 0;
}

/*
 * The single natural handback path: only CONFIGURED gives the bus back
 * (UNCONFIGURED and ERR hold it for good), and only once every vote is
 * gone.
 */
static void release_work_fn(struct work_struct *work)
{
	struct oplus_camres *cr = container_of(to_delayed_work(work),
					       struct oplus_camres,
					       release_work);

	mutex_lock(&cr->lock);
	if (cr->state == CAMRES_CONFIGURED && cr->ap_active &&
	    !atomic_read(&cr->consumers)) {
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

/* ---- votes (the single ownership oracle) ---- */

/*
 * One vote through the common path: state gate, then takeover.  Every
 * AP user funnels through here — the public kernel API, the misc
 * device ioctls, the sysfs manual vote and the probe-window votes.
 * While CONFIGURING the caller sleeps until userspace resolves the
 * state (INIT or the owner dying); ERR is a hard error until a fresh
 * DEINIT/INIT cycle recovers the bus.
 */
static int camres_acquire_vote(struct oplus_camres *cr)
{
	int ret;

	if (!READ_ONCE(cr->boot_done))
		return -EAGAIN;

	mutex_lock(&cr->lock);
	for (;;) {
		enum camres_state state = READ_ONCE(cr->state);

		if (state == CAMRES_ERR) {
			mutex_unlock(&cr->lock);
			return -EIO;
		}
		if (state != CAMRES_CONFIGURING)
			break;
		mutex_unlock(&cr->lock);
		if (wait_event_interruptible(cr->state_wq,
					     READ_ONCE(cr->state) !=
					     CAMRES_CONFIGURING))
			return -EINTR;
		mutex_lock(&cr->lock);
	}

	ret = takeover_locked(cr, READ_ONCE(cr->state) == CAMRES_CONFIGURED);
	if (ret) {
		mutex_unlock(&cr->lock);
		return ret;
	}
	atomic_inc(&cr->consumers);
	mutex_unlock(&cr->lock);
	return 0;
}

static void camres_release_vote(struct oplus_camres *cr)
{
	if (atomic_dec_if_positive(&cr->consumers) == 0) {
		/* a waiting DEINIT learns the last vote is gone */
		wake_up_interruptible_all(&cr->state_wq);
		mod_delayed_work(system_wq, &cr->release_work,
				 msecs_to_jiffies(RELEASE_IDLE_MS));
	}
}

/* ---- public API (kept for drivers with exotic probe strategies) ---- */

int oplus_camres_acquire(struct device *consumer)
{
	if (!g_camres)
		return -EPROBE_DEFER;

	return camres_acquire_vote(g_camres);
}
EXPORT_SYMBOL_GPL(oplus_camres_acquire);

int oplus_camres_release(struct device *consumer)
{
	if (!g_camres)
		return -ENODEV;

	camres_release_vote(g_camres);
	return 0;
}
EXPORT_SYMBOL_GPL(oplus_camres_release);

bool oplus_camres_ap_holds(void)
{
	return g_camres ? READ_ONCE(g_camres->ap_active) : false;
}
EXPORT_SYMBOL_GPL(oplus_camres_ap_holds);

/* the bus is usable (not in the ERR state) */
bool oplus_camres_is_ready(void)
{
	struct oplus_camres *cr = g_camres;

	return cr && READ_ONCE(cr->state) != CAMRES_ERR;
}
EXPORT_SYMBOL_GPL(oplus_camres_is_ready);

/* ---- child hooks: runtime PM links + exact probe-boundary votes ---- */

static struct camres_child *camres_child_find(struct oplus_camres *cr,
					      struct device *dev)
{
	struct camres_child *child;

	list_for_each_entry(child, &cr->children, node)
		if (child->dev == dev)
			return child;
	return NULL;
}

/*
 * I2C sensor drivers power their devices on directly inside probe(),
 * before runtime PM is in play — a PM_RUNTIME device link alone cannot
 * cover that window.  The bus notifier gives the exact boundaries:
 * BIND_DRIVER runs inline on the probing thread right before ->probe()
 * (blocking here queues the probe), BOUND_DRIVER/BIND_FAILED run right
 * after it.  A child with no driver in the system never reaches
 * BIND_DRIVER, so it never takes a vote and never pins the bus.
 */
static int camres_bus_notify(struct notifier_block *nb, unsigned long action,
			     void *data)
{
	struct oplus_camres *cr = container_of(nb, struct oplus_camres, nb);
	struct device *dev = data;
	struct camres_child *child;

	if (dev->parent != &cr->adapter.dev)
		return NOTIFY_DONE;

	switch (action) {
	case BUS_NOTIFY_ADD_DEVICE:
		child = kzalloc(sizeof(*child), GFP_KERNEL);
		if (!child)
			return NOTIFY_DONE;
		child->dev = dev;
		/*
		 * consumer first: the child's runtime-resume must run
		 * the supplier (camres) resume first, and the link's
		 * lifetime follows the child binding.
		 */
		child->link = device_link_add(dev, cr->dev,
					 DL_FLAG_PM_RUNTIME |
				 DL_FLAG_AUTOREMOVE_CONSUMER);
		list_add(&child->node, &cr->children);
		break;
	case BUS_NOTIFY_BIND_DRIVER:
		child = camres_child_find(cr, dev);
		if (child && !child->probe_vote) {
			/*
			 * Vote first, book only on success: a failed
			 * acquire (ERR, interrupted wait) must not be
			 * released at the BOUND boundary — that would
			 * steal another user's vote.
			 */
			if (!camres_acquire_vote(cr))
				child->probe_vote = true;
		}
		break;
	case BUS_NOTIFY_BOUND_DRIVER:
	case BUS_NOTIFY_DRIVER_NOT_BOUND:
		child = camres_child_find(cr, dev);
		if (child && child->probe_vote) {
			child->probe_vote = false;
			camres_release_vote(cr);
		}
		break;
	case BUS_NOTIFY_DEL_DEVICE:
		child = camres_child_find(cr, dev);
		if (child) {
			if (child->probe_vote)
				camres_release_vote(cr);
			if (child->link)
				device_link_del(child->link);
			list_del(&child->node);
			kfree(child);
		}
		break;
	}
	return NOTIFY_DONE;
}

/* ---- /dev/camres ---- */

static int camres_open(struct inode *inode, struct file *file)
{
	struct camres_file *priv;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->cr = g_camres;
	if (!priv->cr) {
		kfree(priv);
		return -ENODEV;
	}
	file->private_data = priv;
	return 0;
}

/*
 * fd teardown: an ACQUIRE vote dies with its fd, and so does an
 * unfinished DEINIT — the bus enters ERR until a fresh DEINIT/INIT
 * cycle recovers it.
 */
static int camres_release(struct inode *inode, struct file *file)
{
	struct camres_file *priv = file->private_data;
	struct oplus_camres *cr = priv->cr;

	mutex_lock(&cr->lock);
	if (priv->owner) {
		if (cr->state == CAMRES_CONFIGURING) {
			cr->state = CAMRES_ERR;
			cr->owner = NULL;
			dev_err(cr->dev,
				"deinit owner died before init: bus in ERR\n");
			/* queued acquires wake up and fail */
			wake_up_interruptible_all(&cr->state_wq);
		}
		priv->owner = false;
	}
	mutex_unlock(&cr->lock);

	if (priv->vote)
		camres_release_vote(cr);

	kfree(priv);
	return 0;
}

static long camres_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct camres_file *priv = file->private_data;
	struct oplus_camres *cr = priv->cr;
	long ret;

	/* node permissions are the only authority */
	if (!(file->f_mode & FMODE_WRITE))
		return -EACCES;

	switch (cmd) {
	case CAMRES_IOC_ACQUIRE:
		if (priv->vote)
			return 1;
		ret = camres_acquire_vote(cr);
		if (!ret)
			priv->vote = true;
		return ret;
	case CAMRES_IOC_RELEASE:
		if (!priv->vote)
			return -EINVAL;
		priv->vote = false;
		camres_release_vote(cr);
		return 0;
	case CAMRES_IOC_DEINIT:
		/*
		 * A DEINIT cycle overrides manual holds: they cannot be
		 * dropped from sysfs once the state is UNCONFIGURED, so
		 * leaving one standing would block this ioctl forever.
		 */
		mutex_lock(&cr->manual_lock);
		if (cr->manual_ap) {
			cr->manual_ap = false;
			camres_release_vote(cr);
		}
		mutex_unlock(&cr->manual_lock);

		mutex_lock(&cr->lock);
		if (cr->state == CAMRES_CONFIGURING) {
			ret = priv->owner && cr->deinit_done ? 1 : -EBUSY;
			mutex_unlock(&cr->lock);
			return ret;
		}
		if (cr->state == CAMRES_ERR)
			dev_info(cr->dev, "ERR recovery: new deinit cycle\n");
		cr->state = CAMRES_CONFIGURING;
		cr->deinit_done = false;
		cr->owner = priv;
		priv->owner = true;
		mutex_unlock(&cr->lock);

		/*
		 * Block until the last AP user is done: explicit votes
		 * (ioctls, kernel API, sysfs, probe windows) are counted
		 * in ->consumers, device-link users in the runtime PM
		 * usage of this device.  New acquires queue on the state
		 * change instead of taking the bus.  Wakes come from the
		 * vote-release path and the runtime-suspend completion.
		 */
		ret = wait_event_interruptible(cr->state_wq,
				atomic_read(&cr->consumers) == 0 &&
				pm_runtime_suspended(cr->dev));
		if (ret) {
			mutex_lock(&cr->lock);
			if (cr->owner == priv &&
			    cr->state == CAMRES_CONFIGURING && !cr->deinit_done) {
				cr->state = CAMRES_UNCONFIGURED;
				cr->owner = NULL;
				priv->owner = false;
				wake_up_interruptible_all(&cr->state_wq);
			}
			mutex_unlock(&cr->lock);
			return -EINTR;
		}

		cancel_delayed_work_sync(&cr->release_work);
		mutex_lock(&cr->lock);
		if (cr->owner != priv || cr->state != CAMRES_CONFIGURING) {
			mutex_unlock(&cr->lock);
			return -EBUSY;
		}
		/*
		 * Every user is gone; hand the bus to the SEE now (the
		 * normal idle-release paths stay out of CONFIGURING) and
		 * unsubscribe the AON camera report.
		 */
		if (cr->ap_active && release_locked(cr))
			dev_warn(cr->dev, "deinit: handback deferred\n");
		sns_aon_power_off(cr);
		cr->deinit_done = true;
		mutex_unlock(&cr->lock);
		return 0;
	case CAMRES_IOC_INIT: {
		__s32 ack;
		enum camres_state state;

		if (copy_from_user(&ack, (void __user *)arg, sizeof(ack)))
			return -EFAULT;

		mutex_lock(&cr->lock);
		state = cr->state;
		if (!priv->owner || state != CAMRES_CONFIGURING) {
			mutex_unlock(&cr->lock);
			return state == CAMRES_UNCONFIGURED ||
			       state == CAMRES_CONFIGURED ? 1 : -EACCES;
		}
		cr->owner = NULL;
		priv->owner = false;

		if (ack == 0) {
			cr->state = CAMRES_CONFIGURED;
			dev_info(cr->dev, "configured: SEE arbitration active\n");
		} else {
			/* vendor tolerance: keep the camera, drop arbitration */
			cr->state = CAMRES_UNCONFIGURED;
			dev_info(cr->dev,
				 "INIT rejected (ack %d): staying unconfigured\n",
				 ack);
		}
		mutex_unlock(&cr->lock);

		/*
		 * Re-subscribe AON *before* waking the queued acquirers:
		 * the subscribe needs the bus free, and once it stands the
		 * REVOKE precondition of this session's first takeover is
		 * met.  One synchronous attempt — a failure degrades to a
		 * takeover without REVOKE, logged.
		 */
		if (!cr->aon_armed && !sns_aon_power_on(cr)) {
			cr->aon_armed = true;
			dev_info(cr->dev, "SEE AON resubscribed\n");
		} else if (!cr->aon_armed) {
			dev_warn(cr->dev, "SEE AON resubscribe failed\n");
			schedule_delayed_work(&cr->aon_work, 0);
		}
		wake_up_interruptible_all(&cr->state_wq);
		return 0;
	}
	default:
		return -ENOTTY;
	}
}

static const struct file_operations camres_fops = {
	.owner = THIS_MODULE,
	.open = camres_open,
	.release = camres_release,
	.unlocked_ioctl = camres_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
};

/* ---- sysfs ---- */

static ssize_t ap_hold_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%d\n", oplus_camres_ap_holds());
}

static ssize_t ap_hold_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct oplus_camres *cr = dev_get_drvdata(dev);
	bool val;
	int ret;

	ret = kstrtobool(buf, &val);
	if (ret)
		return ret;

	/* UNCONFIGURED never takes or drops manual votes */
	if (READ_ONCE(cr->state) == CAMRES_UNCONFIGURED)
		return -EOPNOTSUPP;

	/*
	 * manual_lock serializes the whole manual-vote lifecycle: the
	 * acquire may park in a CONFIGURING wait, so cr->lock alone
	 * cannot span it — without this a concurrent write-1 would
	 * pass the manual_ap check before the parked one books it, and
	 * a later write-0 could only release one of the two votes.
	 */
	mutex_lock(&cr->manual_lock);

	if (val) {
		if (!cr->manual_ap) {
			ret = camres_acquire_vote(cr);
			if (!ret) {
				/*
				 * A parked acquire may complete after the
				 * state fell back to UNCONFIGURED (INIT
				 * with an error); a manual vote is
				 * unreleasable there, so drop it at once —
				 * the next DEINIT cycle force-clears any
				 * that slip through regardless.
				 */
				if (READ_ONCE(cr->state) == CAMRES_UNCONFIGURED) {
					camres_release_vote(cr);
					ret = -EOPNOTSUPP;
				} else {
					cr->manual_ap = true;
				}
			}
		}
	} else {
		if (cr->manual_ap) {
			cr->manual_ap = false;
			camres_release_vote(cr);
		}
	}

	mutex_unlock(&cr->manual_lock);
	return ret ? ret : count;
}
static struct device_attribute dev_attr_ap_hold =
	__ATTR(ap_hold, 0600, ap_hold_show, ap_hold_store);

static struct attribute *camres_attrs[] = {
	&dev_attr_ap_hold.attr,
	NULL,
};
ATTRIBUTE_GROUPS(camres);

/* ---- runtime PM (the supplier side of the child links) ---- */

static int camres_runtime_resume(struct device *dev)
{
	struct oplus_camres *cr = dev_get_drvdata(dev);
	enum camres_state state = READ_ONCE(cr->state);
	int ret;

	/*
	 * The runtime PM core increments the usage count of this device
	 * BEFORE calling here (it must, to keep the supplier active
	 * across the consumer's resume).  Queueing inside CONFIGURING
	 * would therefore hold a usage reference while parked — and a
	 * DEINIT waiting for pm_runtime_suspended() could never
	 * complete, deadlocking against the userspace INIT that only
	 * the DEINIT caller can issue.  So the runtime path fails fast
	 * instead of queueing: the child's resume errors out, its
	 * driver retries, and the ioctl/kernel-API acquire paths (which
	 * queue first and count only afterwards) carry the queueing
	 * semantics.
	 */
	if (state == CAMRES_CONFIGURING)
		return -EAGAIN;
	if (state == CAMRES_ERR)
		return -EIO;

	mutex_lock(&cr->lock);
	ret = takeover_locked(cr, READ_ONCE(cr->state) == CAMRES_CONFIGURED);
	mutex_unlock(&cr->lock);
	return ret;
}

static int camres_runtime_suspend(struct device *dev)
{
	struct oplus_camres *cr = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&cr->lock);
	/* only the configured state hands an idle bus back to the SEE;
	 * a DEINIT in progress forces its own handback after the wait */
	if (cr->state == CAMRES_CONFIGURED && cr->ap_active)
		ret = release_locked(cr);
	mutex_unlock(&cr->lock);

	/* the DEINIT waiter counts on this transition */
	wake_up_interruptible_all(&cr->state_wq);
	return ret;
}

static const struct dev_pm_ops camres_pm_ops = {
	RUNTIME_PM_OPS(camres_runtime_suspend, camres_runtime_resume, NULL)
};

/* ---- platform driver ---- */

/*
 * Boot settlement: the module-power question is resolved (AON armed)
 * or given up on (SEE silent), so the bus is taken for good and the
 * adapter — with its children — becomes visible.  Children probe
 * against a held bus with the module already powered.
 */
static void camres_boot_done(struct oplus_camres *cr)
{
	int ret;

	if (cr->boot_done)
		return;

	cr->state = CAMRES_UNCONFIGURED;
	mutex_lock(&cr->lock);
	ret = takeover_locked(cr, false);
	mutex_unlock(&cr->lock);
	if (ret) {
		/*
		 * Fatal, by design: a takeover/registration failure here
		 * is not something SEE rediscovery would cure — the only
		 * sanctioned retry is a SEE restart, which re-runs this
		 * path from the lookup chain.
		 */
		dev_err(cr->dev, "boot takeover failed (%d)\n", ret);
		return;
	}

	ret = bus_register_notifier(&i2c_bus_type, &cr->nb);
	if (ret)
		goto err_release;

	ret = i2c_add_adapter(&cr->adapter);
	if (ret)
		goto err_notifier;

	cr->misc.minor = MISC_DYNAMIC_MINOR;
	cr->misc.name = "camres";
	cr->misc.fops = &camres_fops;
	cr->misc.parent = cr->dev;
	ret = misc_register(&cr->misc);
	if (ret)
		goto err_adapter;

	/*
	 * Runtime PM: autosuspend hands an idle CONFIGURED bus to the
	 * SEE; in the other states the suspend callback is a no-op, so
	 * no reference is pinned here — UNCONFIGURED and ERR keep the
	 * bus through ->ap_active and the state gates alone, and the
	 * DEINIT wait can observe "suspended" uniformly.
	 */
	pm_runtime_set_active(cr->dev);
	pm_runtime_enable(cr->dev);
	pm_runtime_set_autosuspend_delay(cr->dev, RELEASE_IDLE_MS);
	pm_runtime_use_autosuspend(cr->dev);

	cr->boot_done = true;
	g_camres = cr;
	kobject_uevent_env(&cr->dev->kobj, KOBJ_CHANGE,
			   (char *[]){ "CAMRES_READY=1", NULL });
	dev_info(cr->dev, "camera bus ready, held to AP (bus %d)\n",
		 i2c_adapter_id(&cr->adapter));
	return;

err_adapter:
	i2c_del_adapter(&cr->adapter);
err_notifier:
	bus_unregister_notifier(&i2c_bus_type, &cr->nb);
err_release:
	mutex_lock(&cr->lock);
	if (cr->ap_active)
		release_locked(cr);
	mutex_unlock(&cr->lock);
	dev_err(cr->dev, "boot bring-up failed (%d)\n", ret);
}

static int oplus_camres_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct oplus_camres *cr;

	cr = devm_kzalloc(dev, sizeof(*cr), GFP_KERNEL);
	if (!cr)
		return -ENOMEM;

	cr->dev = dev;
	cr->suid_hi = 0xa7167528f835f4eeULL;	/* camera_handshake */
	cr->suid_lo = 0xe0e3f77b504b96a1ULL;
	mutex_init(&cr->lock);
	mutex_init(&cr->manual_lock);
	atomic_set(&cr->consumers, 0);
	INIT_DELAYED_WORK(&cr->release_work, release_work_fn);
	INIT_DELAYED_WORK(&cr->lookup_work, lookup_work_fn);
	INIT_DELAYED_WORK(&cr->aon_work, aon_work_fn);
	INIT_WORK(&cr->rx_work, rx_work_fn);
	init_waitqueue_head(&cr->rx_wq);
	init_waitqueue_head(&cr->state_wq);
	INIT_LIST_HEAD(&cr->children);
	mutex_init(&cr->sns_lock);
	spin_lock_init(&cr->ack_lock);
	init_completion(&cr->ack_done);
	platform_set_drvdata(pdev, cr);

	cr->geni_np = of_parse_phandle(dev->of_node, "i2c-bus", 0);
	if (!cr->geni_np)
		return dev_err_probe(dev, -EINVAL, "missing i2c-bus phandle\n");

	of_property_read_u64(dev->of_node, "sns,suid-hi", &cr->suid_hi);
	of_property_read_u64(dev->of_node, "sns,suid-lo", &cr->suid_lo);
	if (suid_hi)
		cr->suid_hi = suid_hi;
	if (suid_lo)
		cr->suid_lo = suid_lo;

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

	cr->adapter.owner = THIS_MODULE;
	cr->adapter.algo = &camres_i2c_algo;
	cr->adapter.dev.parent = dev;
	cr->adapter.dev.of_node = dev->of_node;
	cr->adapter.nr = -1;
	strscpy(cr->adapter.name, "oplus-camres-i2c",
		sizeof(cr->adapter.name));
	i2c_set_adapdata(&cr->adapter, cr);

	cr->nb.notifier_call = camres_bus_notify;

	/*
	 * Boot: discover the SNS service, subscribe AON (which needs the
	 * bus free), and only then take the bus and expose the adapter —
	 * camres_boot_done() from aon_work_fn.  Nothing here blocks
	 * probe; the camera subsystem appears asynchronously.
	 */
	schedule_delayed_work(&cr->lookup_work, 0);

	dev_info(dev, "camera bus manager starting (SEE boot chain)\n");
	return 0;
}

static void oplus_camres_remove(struct platform_device *pdev)
{
	struct oplus_camres *cr = platform_get_drvdata(pdev);
	struct camres_child *child, *tmp;

	g_camres = NULL;
	cancel_delayed_work_sync(&cr->release_work);
	cancel_delayed_work_sync(&cr->lookup_work);
	cancel_delayed_work_sync(&cr->aon_work);

	if (!cr->boot_done) {
		/* boot chain never settled: nothing was registered */
		sns_socket_close(cr);
		of_node_put(cr->geni_np);
		return;
	}

	pm_runtime_disable(cr->dev);

	misc_deregister(&cr->misc);
	i2c_del_adapter(&cr->adapter);
	bus_unregister_notifier(&i2c_bus_type, &cr->nb);

	list_for_each_entry_safe(child, tmp, &cr->children, node) {
		if (child->probe_vote)
			camres_release_vote(cr);
		if (child->link)
			device_link_del(child->link);
		list_del(&child->node);
		kfree(child);
	}

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

	sns_socket_close(cr);
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
		.pm = pm_ptr(&camres_pm_ops),
	},
};
module_platform_driver(oplus_camres_driver);

MODULE_AUTHOR("Iceland mainline");
MODULE_DESCRIPTION("OPlus camera shared I2C bus between AP and SEE");
MODULE_LICENSE("GPL");
