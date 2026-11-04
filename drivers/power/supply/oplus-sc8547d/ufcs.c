// SPDX-License-Identifier: GPL-2.0-only
/*
 * UFCS sink protocol over the SC8547D UFCS PHY.
 *
 * Message formats, timers and the handshake sequence follow the Oplus UFCS
 * class (vendor/oplus/kernel/charger/v2/ufcs/),
 * Copyright (C) 2023 Oplus. All rights reserved.
 *
 * The PHY computes and checks the CRC and answers every received message
 * with ACK on its own. The driver therefore sees, for each sent message, a
 * "sent" flag once the adapter acknowledged it, and for each received
 * message a "data ready" flag. The PHY has one transmit buffer, so every
 * message goes out under one lock and waits for its own flag. One
 * transaction runs at a time; messages the adapter starts on its own are
 * recorded by the interrupt thread and answered from a work item.
 */
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#include "sc8547d.h"
#include "ufcs.h"

#define UFCS_VERSION		1
#define UFCS_ADDR_SOURCE	1
#define UFCS_ADDR_SINK		2
#define UFCS_TYPE_CTRL		0
#define UFCS_TYPE_DATA		1
#define UFCS_TYPE_VENDOR	2

#define CTRL_PING			0x00
#define CTRL_ACK			0x01
#define CTRL_NCK			0x02
#define CTRL_ACCEPT			0x03
#define CTRL_SOFT_RESET			0x04
#define CTRL_POWER_READY		0x05
#define CTRL_GET_OUTPUT_CAPABILITIES	0x06
#define CTRL_GET_SOURCE_INFO		0x07
#define CTRL_GET_SINK_INFO		0x08
#define CTRL_GET_CABLE_INFO		0x09
#define CTRL_GET_DEVICE_INFO		0x0a
#define CTRL_GET_ERROR_INFO		0x0b
#define CTRL_EXIT_UFCS_MODE		0x0f

#define DATA_OUTPUT_CAPABILITIES	0x01
#define DATA_REQUEST			0x02
#define DATA_SOURCE_INFO		0x03
#define DATA_SINK_INFO			0x04
#define DATA_DEVICE_INFO		0x06
#define DATA_ERROR_INFO			0x07
#define DATA_CONFIG_WATCHDOG		0x08
#define DATA_REFUSE			0x09
#define DATA_POWER_CHANGE		0x0c

#define REFUSE_UNKNOWN_CMD		1
#define REFUSE_NOT_SUPPORT_CMD		2
#define REFUSE_DEVICE_BUSY		3
#define REFUSE_OUT_OF_RANGE		4

/* Device info the stock firmware reports, so adapters see the same sink. */
#define SINK_DEV_VENDOR		0x22d9
#define SINK_IC_VENDOR		0x0006
#define SINK_HW_VERSION		0x0001
#define SINK_SW_VERSION		0x0001

#define UFCS_RX_DAMAGED		(SC8547D_UFCS2_CRC_ERROR | SC8547D_UFCS2_STOP_ERROR | \
				 SC8547D_UFCS2_START_FAIL | SC8547D_UFCS2_LENGTH_ERROR | \
				 SC8547D_UFCS2_BYTE_TIMEOUT | SC8547D_UFCS2_TRAINING_ERROR | \
				 SC8547D_UFCS2_BAUD_ERROR)

#define T_HANDSHAKE_MS		300
#define T_SENT_MS		100
#define T_SENDER_RESPONSE_MS	100
#define T_POWER_SUPPLY_MS	550
#define T_MSG_TRANS_DELAY_US	10000
#define HANDSHAKE_RETRIES	3
#define BAUD_COUNT		4

/*
 * Write one message and wait for the PHY to report it acknowledged. The
 * message number is taken, the buffer written and the flag awaited under
 * one lock, so numbers go out in order. Every attempt owns a sequence
 * number: a flag that arrives for an attempt that already timed out is
 * not credited to the next one.
 */
static int ufcs_write(struct ufcs *u, u8 type, const u8 *body, int body_len)
{
	u8 buf[64];
	u16 head;
	unsigned int seq;
	long left;
	int ret;

	if (body_len < 1 || body_len > sizeof(buf) - 2)
		return -EMSGSIZE;
	mutex_lock(&u->tx_lock);
	head = (UFCS_ADDR_SOURCE << 13) | ((u->msg_index & 0xf) << 9) | (UFCS_VERSION << 3) | type;
	u->msg_index = (u->msg_index + 1) & 0xf;
	put_unaligned_be16(head, buf);
	memcpy(&buf[2], body, body_len);
	/* The adapter needs a gap between consecutive messages. */
	usleep_range(T_MSG_TRANS_DELAY_US, T_MSG_TRANS_DELAY_US + 1000);
	seq = ++u->tx_seq;
	reinit_completion(&u->sent);
	u->send_result = -ETIMEDOUT;
	ret = sc8547d_ufcs_write_msg(u->cp, buf, 2 + body_len);
	left = msecs_to_jiffies(T_SENT_MS);
	while (!ret) {
		left = wait_for_completion_timeout(&u->sent, left);
		if (!left) {
			ret = -ETIMEDOUT;
			break;
		}
		if (READ_ONCE(u->sent_seq) == seq) {
			ret = u->send_result;
			break;
		}
		reinit_completion(&u->sent);	/* a stale flag: keep waiting */
	}
	mutex_unlock(&u->tx_lock);
	return ret;
}

static int ufcs_send_ctrl(struct ufcs *u, u8 cmd)
{
	return ufcs_write(u, UFCS_TYPE_CTRL, &cmd, 1);
}

static int ufcs_send_data(struct ufcs *u, u8 cmd, const u8 *data, u8 len)
{
	u8 body[62];

	if (len > sizeof(body) - 2)
		return -EMSGSIZE;
	body[0] = cmd;
	body[1] = len;
	memcpy(&body[2], data, len);
	return ufcs_write(u, UFCS_TYPE_DATA, body, 2 + len);
}

static int ufcs_send_u64(struct ufcs *u, u8 cmd, u64 value)
{
	u8 data[8];

	put_unaligned_be64(value, data);
	return ufcs_send_data(u, cmd, data, sizeof(data));
}

static int ufcs_send_refuse(struct ufcs *u, u8 index, u8 type, u8 cmd, u8 reason)
{
	u8 data[4];
	u32 v = ((u32)(index & 0xf) << 24) | ((u32)(type & 0x7) << 16) |
		((u32)(cmd & 0xf) << 8) | reason;

	put_unaligned_be32(v, data);
	return ufcs_send_data(u, DATA_REFUSE, data, sizeof(data));
}

/* Wait for the response named by ufcs_expect(); the ref flags cancel it. */
static int ufcs_wait_response(struct ufcs *u, unsigned int ms)
{
	if (!wait_for_completion_timeout(&u->response, msecs_to_jiffies(ms)))
		return -ETIMEDOUT;
	if (u->hard_reset || !u->linked)
		return -ECONNRESET;
	return 0;
}

static void ufcs_expect(struct ufcs *u, u8 type, u8 cmd, u8 cmd2)
{
	unsigned long flags;

	spin_lock_irqsave(&u->rx_lock, flags);
	u->want_type = type;
	u->want_cmd = cmd;
	u->want_cmd2 = cmd2;
	u->rx_valid = false;
	reinit_completion(&u->response);
	spin_unlock_irqrestore(&u->rx_lock, flags);
}

static void ufcs_expect_none(struct ufcs *u)
{
	ufcs_expect(u, 0xff, 0xff, 0xff);
}

/* Interrupt side */

struct ufcs_msg {
	u8 type;
	u8 index;
	u8 cmd;
	u8 len;
	const u8 *data;
};

static int ufcs_parse(const u8 *buf, int len, struct ufcs_msg *m)
{
	u16 head;

	if (len < 3)
		return -EBADMSG;
	head = get_unaligned_be16(buf);
	m->type = head & 0x7;
	m->index = (head >> 9) & 0xf;
	switch (m->type) {
	case UFCS_TYPE_CTRL:
		m->cmd = buf[2];
		m->len = 0;
		m->data = NULL;
		return 0;
	case UFCS_TYPE_DATA:
		if (len < 4 || buf[3] != len - 4)
			return -EBADMSG;
		m->cmd = buf[2];
		m->len = buf[3];
		m->data = &buf[4];
		return 0;
	case UFCS_TYPE_VENDOR:
		if (len < 5 || buf[4] != len - 5)
			return -EBADMSG;
		m->cmd = 0;
		m->len = buf[4];
		m->data = &buf[5];
		return 0;
	default:
		return -EBADMSG;
	}
}

static void ufcs_reply_sink_info(struct ufcs *u)
{
	struct ufcs_sink_info info = { .batt_temp_c = 25, .usb_temp_c = 25 };
	u64 v;

	if (u->get_sink_info)
		u->get_sink_info(u->sink_data, &info);
	v = ((u64)((info.batt_temp_c + 50) & 0xff) << 40) |
	    ((u64)((info.usb_temp_c + 50) & 0xff) << 32) |
	    ((u64)((info.vbat_mv / 10) & 0xffff) << 16) |
	    ((u64)(abs(info.ibat_ma) / 10) & 0xffff);
	ufcs_send_u64(u, DATA_SINK_INFO, v);
}

static void ufcs_reply_device_info(struct ufcs *u)
{
	u64 v = ((u64)SINK_DEV_VENDOR << 48) | ((u64)SINK_IC_VENDOR << 32) |
		((u64)SINK_HW_VERSION << 16) | SINK_SW_VERSION;

	ufcs_send_u64(u, DATA_DEVICE_INFO, v);
}

/* A malformed power change is ignored; a valid one caps later requests. */
static void ufcs_power_change_record(struct ufcs *u, const struct ufcs_msg *m)
{
	int i, n = m->len / 3;

	if (!n || m->len % 3 || n > UFCS_PDO_MAX)
		return;
	for (i = 0; i < n; i++)
		u->power_change[i] = ((u32)m->data[3 * i] << 16) |
				     ((u32)m->data[3 * i + 1] << 8) | m->data[3 * i + 2];
	for (; i < UFCS_PDO_MAX; i++)
		u->power_change[i] = 0;
	u->power_changed = true;
}

/* Messages the adapter sends on its own, answered from the work item. */
static void ufcs_reply(struct ufcs *u, const struct ufcs_msg *m)
{
	if (m->type == UFCS_TYPE_CTRL) {
		switch (m->cmd) {
		case CTRL_GET_SINK_INFO:
			ufcs_reply_sink_info(u);
			return;
		case CTRL_GET_DEVICE_INFO:
			ufcs_reply_device_info(u);
			return;
		case CTRL_GET_ERROR_INFO:
			ufcs_send_data(u, DATA_ERROR_INFO, (u8[4]){ 0, 0, 0, 0 }, 4);
			return;
		default:
			break;
		}
	}
	ufcs_send_refuse(u, m->index, m->type, m->cmd, REFUSE_NOT_SUPPORT_CMD);
}

static void ufcs_reply_work(struct work_struct *work)
{
	struct ufcs *u = container_of(work, struct ufcs, reply_work);
	struct ufcs_msg m;
	unsigned long flags;
	u8 raw[sizeof(u->pending)];
	int len;

	spin_lock_irqsave(&u->rx_lock, flags);
	len = u->pending_len;
	memcpy(raw, u->pending, sizeof(raw));
	u->pending_len = 0;
	spin_unlock_irqrestore(&u->rx_lock, flags);
	if (!len || !READ_ONCE(u->linked))
		return;
	if (ufcs_parse(raw, len, &m))
		return;
	ufcs_reply(u, &m);
}

/*
 * Record one adapter-initiated message for the reply work. The adapter waits
 * for each answer before asking again; should a second request arrive first
 * it replaces the earlier one, which then goes unanswered and is re-asked.
 */
static void ufcs_unsolicited(struct ufcs *u, const struct ufcs_msg *m, const u8 *buf, int len)
{
	unsigned long flags;

	if (m->type == UFCS_TYPE_CTRL) {
		switch (m->cmd) {
		case CTRL_PING:
			return;
		case CTRL_SOFT_RESET:
			u->msg_index = 0;
			return;
		case CTRL_EXIT_UFCS_MODE:
			u->linked = false;
			return;
		default:
			break;
		}
	} else if (m->type == UFCS_TYPE_DATA && m->cmd == DATA_POWER_CHANGE) {
		ufcs_power_change_record(u, m);
		return;
	}
	if (len > sizeof(u->pending))
		len = sizeof(u->pending);
	spin_lock_irqsave(&u->rx_lock, flags);
	memcpy(u->pending, buf, len);
	u->pending_len = len;
	spin_unlock_irqrestore(&u->rx_lock, flags);
	schedule_work(&u->reply_work);
}

static void ufcs_receive(struct ufcs *u, const u8 *buf, int len)
{
	struct ufcs_msg m;
	unsigned long flags;
	bool wanted;

	if (ufcs_parse(buf, len, &m)) {
		dev_dbg(u->dev, "UFCS: bad message %*ph\n", len, buf);
		return;
	}
	/* The PHY answers these itself; a stray one carries nothing. */
	if (m.type == UFCS_TYPE_CTRL && (m.cmd == CTRL_ACK || m.cmd == CTRL_NCK))
		return;

	spin_lock_irqsave(&u->rx_lock, flags);
	wanted = m.type == u->want_type && (m.cmd == u->want_cmd || m.cmd == u->want_cmd2);
	if (!wanted && m.type == UFCS_TYPE_DATA && m.cmd == DATA_REFUSE && u->want_type != 0xff)
		wanted = true;
	if (!wanted && m.type == UFCS_TYPE_CTRL && m.cmd == CTRL_SOFT_RESET && u->want_type != 0xff)
		wanted = true;
	if (wanted) {
		memcpy(u->rx, buf, len);
		u->rx_len = len;
		u->rx_valid = true;
		if (m.type == u->want_type && m.cmd == u->want_cmd && u->want_cmd2 != u->want_cmd) {
			/* The first of two expected replies: keep waiting for the second. */
			u->want_cmd = u->want_cmd2;
		} else {
			u->want_type = 0xff;
			complete(&u->response);
		}
	}
	spin_unlock_irqrestore(&u->rx_lock, flags);
	if (!wanted)
		ufcs_unsolicited(u, &m, buf, len);
}

/*
 * ufcs_irq() - the SC8547D interrupt while UFCS owns it
 *
 * Reading the flag registers clears them, so everything they report is
 * handled here: handshake outcome, the fate of the message being sent and
 * any message waiting in the receive buffer.
 */
void ufcs_irq(void *data)
{
	struct ufcs *u = data;
	u8 flags[3];
	u8 buf[64];
	int ret;

	ret = sc8547d_ufcs_read_flags(u->cp, flags);
	if (ret)
		return;
	dev_dbg(u->dev, "UFCS flags %02x %02x %02x\n", flags[0], flags[1], flags[2]);

	if (flags[0] & (SC8547D_UFCS_HANDSHAKE_OK | SC8547D_UFCS_HANDSHAKE_FAIL)) {
		u->handshake_result = (flags[0] & SC8547D_UFCS_HANDSHAKE_FAIL) ? -EIO : 0;
		complete(&u->handshake);
	}
	if (flags[1] & SC8547D_UFCS2_HARD_RESET) {
		u->hard_reset = true;
		u->linked = false;
		u->send_result = -ECONNRESET;
		WRITE_ONCE(u->sent_seq, u->tx_seq);
		complete(&u->sent);
		complete(&u->response);
		return;
	}
	if (flags[2] & SC8547D_UFCS3_BUS_CONFLICT) {
		u->send_result = -EBUSY;
		WRITE_ONCE(u->sent_seq, u->tx_seq);
		complete(&u->sent);
	} else if (flags[0] & (SC8547D_UFCS_SENT | SC8547D_UFCS_TRANS_FAIL |
			       SC8547D_UFCS_ACK_TIMEOUT)) {
		/* Any outcome of the message in the transmit buffer ends its wait. */
		if (flags[0] & SC8547D_UFCS_TRANS_FAIL)
			u->send_result = -EIO;
		else if (flags[0] & SC8547D_UFCS_ACK_TIMEOUT)
			u->send_result = -ETIMEDOUT;
		else
			u->send_result = 0;
		WRITE_ONCE(u->sent_seq, u->tx_seq);
		complete(&u->sent);
	}
	if (flags[0] & (SC8547D_UFCS_DATA_READY | SC8547D_UFCS_RX_BUSY)) {
		ret = sc8547d_ufcs_read_msg(u->cp, buf, sizeof(buf));
		/* A frame the PHY flagged as damaged is read to free the buffer and dropped. */
		if (ret > 0 && (flags[1] & UFCS_RX_DAMAGED))
			dev_dbg(u->dev, "UFCS: dropping damaged frame, flags %02x\n", flags[1]);
		else if (ret > 0 && (flags[0] & SC8547D_UFCS_DATA_READY))
			ufcs_receive(u, buf, ret);
	}
	if (flags[0] & SC8547D_UFCS_RX_OVERFLOW)
		sc8547d_ufcs_clear_rx(u->cp);
}

/* Process side */

void ufcs_init(struct ufcs *u, struct device *dev, struct sc8547d *cp)
{
	u->dev = dev;
	u->cp = cp;
	mutex_init(&u->lock);
	mutex_init(&u->tx_lock);
	INIT_WORK(&u->reply_work, ufcs_reply_work);
	spin_lock_init(&u->rx_lock);
	init_completion(&u->handshake);
	init_completion(&u->sent);
	init_completion(&u->response);
	ufcs_expect_none(u);
}

bool ufcs_linked(struct ufcs *u)
{
	return READ_ONCE(u->linked);
}

/*
 * The PHY retries the handshake itself; a fail flag means it gave up. After
 * the handshake the baud rate is probed from 115200 down with a ping.
 */
int ufcs_handshake(struct ufcs *u)
{
	int ret, attempt, baud;

	mutex_lock(&u->lock);
	u->linked = false;
	u->hard_reset = false;
	u->exited = false;
	u->msg_index = 0;
	u->power_changed = false;
	u->pdo_count = 0;
	u->pending_len = 0;
	ufcs_expect_none(u);
	ret = sc8547d_ufcs_enable(u->cp);
	if (ret)
		goto out;
	for (attempt = 0; attempt < HANDSHAKE_RETRIES; attempt++) {
		reinit_completion(&u->handshake);
		u->handshake_result = -ETIMEDOUT;
		ret = sc8547d_ufcs_handshake(u->cp);
		if (ret)
			goto fail;
		if (wait_for_completion_timeout(&u->handshake, msecs_to_jiffies(T_HANDSHAKE_MS)) &&
		    !u->handshake_result)
			break;
	}
	if (attempt == HANDSHAKE_RETRIES) {
		ret = -ENODEV;
		goto fail;
	}
	for (baud = 0; baud < BAUD_COUNT; baud++) {
		ret = sc8547d_ufcs_set_baud(u->cp, baud);
		if (ret)
			goto fail;
		ret = ufcs_send_ctrl(u, CTRL_PING);
		if (!ret)
			break;
	}
	if (ret)
		goto fail;
	u->linked = true;
	dev_dbg(u->dev, "UFCS link up at baud index %d\n", baud);
out:
	mutex_unlock(&u->lock);
	return ret;
fail:
	sc8547d_ufcs_disable(u->cp);
	mutex_unlock(&u->lock);
	return ret;
}

/* A refuse to our request; a "busy" refusal gets one soft reset and retry. */
static int ufcs_check_refuse(struct ufcs *u, u8 type, u8 cmd)
{
	u32 v;

	if (u->rx_len < 8)
		return -EBADMSG;
	v = get_unaligned_be32(&u->rx[4]);
	if (((v >> 16) & 0x7) != type || ((v >> 8) & 0xf) != cmd)
		return 0;	/* a refusal of something else: ignore */
	dev_dbg(u->dev, "UFCS refuse reason %u\n", v & 0xf);
	return (v & 0xf) == REFUSE_DEVICE_BUSY ? -EAGAIN : -EOPNOTSUPP;
}

/*
 * Send a message and wait for one reply. A refusal or a soft reset counts as
 * a reply. Returns 0 with the reply in u->rx.
 */
static int ufcs_transact(struct ufcs *u, bool data, u8 cmd, const u8 *payload, u8 len,
			 u8 reply_type, u8 reply_cmd, u8 reply_cmd2, unsigned int ms)
{
	bool soft_reset_done = false;
	int ret;

	if (!u->linked)
		return -ENOTCONN;
retry:
	ufcs_expect(u, reply_type, reply_cmd, reply_cmd2);
	ret = data ? ufcs_send_data(u, cmd, payload, len) : ufcs_send_ctrl(u, cmd);
	if (ret)
		goto out;
	ret = ufcs_wait_response(u, ms);
	if (ret)
		goto out;
	if (u->rx[2] == CTRL_SOFT_RESET && (u->rx[0] & 0x7) == UFCS_TYPE_CTRL &&
	    u->rx_len == 3) {
		u->msg_index = 0;
		ret = -EAGAIN;
		goto out;
	}
	if ((get_unaligned_be16(u->rx) & 0x7) == UFCS_TYPE_DATA && u->rx[2] == DATA_REFUSE) {
		ret = ufcs_check_refuse(u, data ? UFCS_TYPE_DATA : UFCS_TYPE_CTRL, cmd);
		if (ret == -EAGAIN && !soft_reset_done) {
			soft_reset_done = true;
			ufcs_expect_none(u);
			ufcs_send_ctrl(u, CTRL_SOFT_RESET);
			u->msg_index = 0;
			goto retry;
		}
		if (!ret)
			ret = -EAGAIN;
	}
out:
	ufcs_expect_none(u);
	return ret;
}

int ufcs_get_device_info(struct ufcs *u, u64 *info)
{
	int ret;

	mutex_lock(&u->lock);
	ret = ufcs_transact(u, false, CTRL_GET_DEVICE_INFO, NULL, 0, UFCS_TYPE_DATA,
			    DATA_DEVICE_INFO, DATA_DEVICE_INFO, T_SENDER_RESPONSE_MS);
	if (!ret && u->rx_len < 12)
		ret = -EBADMSG;
	if (!ret)
		*info = get_unaligned_be64(&u->rx[4]);
	mutex_unlock(&u->lock);
	return ret;
}

int ufcs_get_output_caps(struct ufcs *u, u64 *pdo, int *count)
{
	int ret, i, n;

	mutex_lock(&u->lock);
	ret = ufcs_transact(u, false, CTRL_GET_OUTPUT_CAPABILITIES, NULL, 0, UFCS_TYPE_DATA,
			    DATA_OUTPUT_CAPABILITIES, DATA_OUTPUT_CAPABILITIES,
			    T_SENDER_RESPONSE_MS);
	if (ret)
		goto out;
	n = (u->rx_len - 4) / 8;
	if (n < 1 || n > UFCS_PDO_MAX) {
		ret = -EBADMSG;
		goto out;
	}
	for (i = 0; i < n; i++) {
		pdo[i] = get_unaligned_be64(&u->rx[4 + 8 * i]);
		if (!pdo[i])
			break;
	}
	u->pdo_count = i;
	memcpy(u->pdo, pdo, sizeof(u->pdo[0]) * i);
	*count = i;
	if (!i)
		ret = -ENODATA;
out:
	mutex_unlock(&u->lock);
	return ret;
}

int ufcs_get_source_info(struct ufcs *u, u64 *info)
{
	int ret;

	mutex_lock(&u->lock);
	ret = ufcs_transact(u, false, CTRL_GET_SOURCE_INFO, NULL, 0, UFCS_TYPE_DATA,
			    DATA_SOURCE_INFO, DATA_SOURCE_INFO, T_SENDER_RESPONSE_MS);
	if (!ret && u->rx_len < 12)
		ret = -EBADMSG;
	if (!ret)
		*info = get_unaligned_be64(&u->rx[4]);
	mutex_unlock(&u->lock);
	return ret;
}

int ufcs_config_watchdog(struct ufcs *u, u16 ms)
{
	u8 data[2];
	int ret;

	put_unaligned_be16(ms, data);
	mutex_lock(&u->lock);
	ret = ufcs_transact(u, true, DATA_CONFIG_WATCHDOG, data, sizeof(data), UFCS_TYPE_CTRL,
			    CTRL_ACCEPT, CTRL_ACCEPT, T_SENDER_RESPONSE_MS);
	mutex_unlock(&u->lock);
	return ret;
}

static int ufcs_select_pdo(struct ufcs *u, int mv, int ma)
{
	int i;

	for (i = 0; i < u->pdo_count; i++) {
		u64 p = u->pdo[i];

		if (mv >= UFCS_CAP_VOL_MIN(p) && mv <= UFCS_CAP_VOL_MAX(p) &&
		    ma >= UFCS_CAP_CURR_MIN(p) && ma <= UFCS_CAP_CURR_MAX(p))
			return i + 1;
	}
	return -ENODATA;
}

/*
 * ufcs_request() - ask the adapter for a voltage and current
 *
 * The adapter first accepts, then reports power ready once the output has
 * settled. An adapter-initiated power change caps the current.
 */
int ufcs_request(struct ufcs *u, int mv, int ma)
{
	u8 data[8];
	int ret, index;
	u64 req;

	mutex_lock(&u->lock);
	index = ufcs_select_pdo(u, mv, ma);
	if (index < 0) {
		ret = index;
		goto out;
	}
	if (u->power_changed) {
		int cap = (u->power_change[index - 1] & 0xffff) * 10;

		if (cap > 0 && ma > cap)
			ma = cap;
	}
	req = ((u64)(index & 0xf) << 60) | ((u64)((mv / 10) & 0xffff) << 16) |
	      ((u64)(ma / 10) & 0xffff);
	put_unaligned_be64(req, data);
	/* Accept within the response time, then power ready once settled. */
	ret = ufcs_transact(u, true, DATA_REQUEST, data, sizeof(data), UFCS_TYPE_CTRL,
			    CTRL_ACCEPT, CTRL_POWER_READY, T_SENDER_RESPONSE_MS + T_POWER_SUPPLY_MS);
out:
	mutex_unlock(&u->lock);
	return ret;
}

/* Leave UFCS mode politely, falling back to a hard reset, and drop the PHY. */
int ufcs_exit(struct ufcs *u)
{
	int ret = 0;

	mutex_lock(&u->lock);
	if (u->exited)
		goto out;
	u->exited = true;
	if (u->linked && !u->hard_reset) {
		ret = ufcs_send_ctrl(u, CTRL_EXIT_UFCS_MODE);
		if (ret)
			sc8547d_ufcs_hard_reset(u->cp);
	}
	u->linked = false;
	ufcs_expect_none(u);
	/* The reply work checks linked before it sends; let one in flight finish first. */
	flush_work(&u->reply_work);
	sc8547d_ufcs_disable(u->cp);
	mutex_unlock(&u->lock);
	return ret;
out:
	mutex_unlock(&u->lock);
	return ret;
}
