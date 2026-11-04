/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Oplus VOOC/SuperVOOC adapter frame protocol.
 *
 * The adapter asks, the phone answers within a few milliseconds. Frames are
 * ten bits on D+/D-, most significant first: a three-bit head and a
 * seven-bit payload. This library decodes requests and produces the replies
 * of the identification exchanges; the session answers power commands.
 */
#ifndef _OPLUS_FC_SVOOC_H
#define _OPLUS_FC_SVOOC_H

#include <linux/types.h>

#define SVOOC_HEAD_SVOOC	4
#define SVOOC_HEAD_VOOC3	3
#define SVOOC_HEAD_VOOC2	5

#define SVOOC_CMD_NULL		0x00
#define SVOOC_CMD_GET_VBAT	0x01
#define SVOOC_CMD_VBUS_OK	0x02
#define SVOOC_CMD_GET_CURRENT	0x03
#define SVOOC_CMD_FASTCHG	0x04
#define SVOOC_CMD_TELL_MODEL	0x05
#define SVOOC_CMD_BAD_CABLE	0x09
#define SVOOC_CMD_AP_STATUS	0x0a
#define SVOOC_CMD_IDENTIFY	0x0c
#define SVOOC_CMD_BAT_MODEL	0x0d

#define SVOOC_VBUS_LOW		0x10
#define SVOOC_VBUS_HIGH		0x20
#define SVOOC_VBUS_NORMAL	0x30
#define SVOOC_CHANGE_CURRENT	0x7f
#define SVOOC_ALLOW		0x40
/* VOOC 2.0 fixed current replies and the "ask again" battery reply */
#define SVOOC_VOOC2_CURR_3A	0x00
#define SVOOC_VOOC2_CURR_3P75A	0x04
#define SVOOC_VOOC2_VBAT_AGAIN	0x3a
#define SVOOC_CURVE_LEN		6

enum svooc_adapter {
	SVOOC_ADAPTER_NONE,
	SVOOC_ADAPTER_SVOOC,
	SVOOC_ADAPTER_VOOC3,
	SVOOC_ADAPTER_VOOC2,
};

struct svooc_request {
	u8 command;
	u8 head;
	bool shifted;	/* the PHY latched the frame one bit late */
};

enum svooc_exchange {
	SVOOC_EXCHANGE_COMMAND,
	SVOOC_EXCHANGE_IDENTIFY,
	SVOOC_EXCHANGE_MODEL,
	SVOOC_EXCHANGE_CURVE,
};

/**
 * struct svooc - adapter identification state
 * @circuit: FASTCHG reply payload of the charging circuit resistance, per
 *	adapter family: SuperVOOC, VOOC 3.0, VOOC 2.0
 * @curve: the six battery curve payloads in transmission order
 * @permit: the phone is ready to let the adapter raise VBUS
 * @adapter: the adapter family seen in the frame heads
 * @identified: the challenge was answered correctly
 * @model: the adapter model, or 0
 * @curve_sent: the adapter read the battery curve
 * @granted: a FASTCHG reply carried the allow bit
 */
struct svooc {
	u8 circuit[3];
	u8 curve[SVOOC_CURVE_LEN];
	u8 bypass_curve[SVOOC_CURVE_LEN];
	bool permit;
	enum svooc_adapter adapter;
	bool identified;
	u8 model;
	bool curve_sent;
	bool granted;
	/* private */
	enum svooc_exchange exchange;
	bool offered;
	u8 remaining;
	u8 attempts;
	u8 second;
	u16 challenge;
	u16 response;
	u8 vooc2_offers;
};

int svooc_decode(u8 rx, struct svooc_request *req);
u16 svooc_frame(u8 head, u8 payload, bool shifted);
void svooc_reset(struct svooc *sv);
int svooc_handle(struct svooc *sv, const struct svooc_request *req, u16 random, u8 *payload);
u8 svooc_reply_head(const struct svooc *sv);
int svooc_current_payload(int ma, u8 *payload);
u8 svooc_vbat_payload(int mv);
u8 svooc_vooc2_vbat_payload(int mv, bool allow);
u8 svooc_vbus_payload(int vbus_mv, int vbat_mv, int ratio, int low_mv, int high_mv);
static inline bool svooc_bypass(const struct svooc *sv)
{
	return sv->adapter == SVOOC_ADAPTER_VOOC3 || sv->adapter == SVOOC_ADAPTER_VOOC2;
}

#endif
