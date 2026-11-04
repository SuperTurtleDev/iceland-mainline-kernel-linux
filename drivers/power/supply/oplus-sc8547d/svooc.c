// SPDX-License-Identifier: GPL-2.0-only
/*
 * Oplus VOOC/SuperVOOC adapter frame protocol.
 *
 * Frame formats and exchanges follow the Oplus AP VOOC PHY
 * (vendor/oplus/kernel/charger/v2/voocphy/oplus_ap_voocphy.c),
 * Copyright (C) 2018-2024 Oplus. All rights reserved.
 */
#include <linux/errno.h>
#include <linux/module.h>

#include "svooc.h"

#define SVOOC_IDENTIFY_FRAMES	5
#define SVOOC_MODEL_FRAMES	2
#define SVOOC_IDENTIFY_MAX	3
#define SVOOC_VOOC2_OFFERS	3

/*
 * svooc_decode() - decode the last eight bits the PHY received
 *
 * The head is in bits 7:5 and the command in bits 4:1. A PHY that latched
 * one bit late shows the head in bits 6:4 and the command in bits 3:0.
 *
 * Return: 0, or -EPROTO for a frame of no known family.
 */
int svooc_decode(u8 rx, struct svooc_request *req)
{
	u8 head = rx >> 5, moved = (rx >> 4) & 7;

	req->shifted = false;
	/* A VOOC2 frame with its top data bit set looks like a moved VOOC3. */
	if (head == SVOOC_HEAD_VOOC2 && moved == SVOOC_HEAD_VOOC3)
		moved = 0;
	if (head != SVOOC_HEAD_SVOOC && head != SVOOC_HEAD_VOOC3 && head != SVOOC_HEAD_VOOC2) {
		head = moved;
		req->shifted = true;
	}
	if (head != SVOOC_HEAD_SVOOC && head != SVOOC_HEAD_VOOC3 && head != SVOOC_HEAD_VOOC2)
		return -EPROTO;
	req->head = head;
	req->command = req->shifted ? rx & 0x0f : (rx >> 1) & 0x0f;
	return 0;
}

/** svooc_frame() - the ten-bit reply for a payload, as the PHY takes it */
u16 svooc_frame(u8 head, u8 payload, bool shifted)
{
	u16 frame = ((u16)(head & 7) << 7) | (payload & 0x7f);

	return shifted ? frame >> 1 : frame;
}

/** svooc_reply_head() - the head of replies to the adapter family seen */
u8 svooc_reply_head(const struct svooc *sv)
{
	switch (sv->adapter) {
	case SVOOC_ADAPTER_VOOC3:
		return SVOOC_HEAD_VOOC3;
	case SVOOC_ADAPTER_VOOC2:
		return SVOOC_HEAD_VOOC2;
	default:
		return SVOOC_HEAD_SVOOC;
	}
}

/** svooc_reset() - forget the adapter, keeping the board data */
void svooc_reset(struct svooc *sv)
{
	sv->permit = false;
	sv->adapter = SVOOC_ADAPTER_NONE;
	sv->identified = false;
	sv->model = 0;
	sv->curve_sent = false;
	sv->granted = false;
	sv->exchange = SVOOC_EXCHANGE_COMMAND;
	sv->offered = false;
	sv->attempts = 0;
	sv->vooc2_offers = 0;
}

/*
 * The adapter must echo both random halves in four nibbles; the two replies
 * scramble the halves bit by bit. Bits 0 and 8 of the echo are don't-care.
 */
static u8 svooc_challenge(struct svooc *sv, u16 random)
{
	u8 lo = random & 0x7f, hi = (random >> 8) & 0x7f;
	u8 first, second;

	sv->challenge = ((u16)hi << 9) | ((u16)lo << 1);
	sv->response = 0;
	first = ((lo >> 2) & 1) | (((hi >> 4) & 1) << 1) | ((hi & 1) << 2) |
		((lo & 1) << 3) | (((hi >> 6) & 1) << 4) | (((lo >> 4) & 1) << 5) |
		(((lo >> 6) & 1) << 6);
	second = ((hi >> 3) & 1) | (((lo >> 1) & 1) << 1) | (((hi >> 2) & 1) << 2) |
		 (((hi >> 1) & 1) << 3) | (((lo >> 5) & 1) << 4) | (((hi >> 5) & 1) << 5) |
		 (((lo >> 3) & 1) << 6);
	sv->second = second;
	sv->exchange = SVOOC_EXCHANGE_IDENTIFY;
	sv->remaining = SVOOC_IDENTIFY_FRAMES;
	return first;
}

/*
 * svooc_handle() - answer an adapter frame
 * @random: random bits for a new challenge
 * @payload: the reply payload
 *
 * Return: 1 with @payload set when answered here; 0 for a power command
 * (GET_VBAT, VBUS_OK, GET_CURRENT, AP_STATUS) the session answers; -EACCES
 * for a power command before the grant; -ENOLINK when the adapter reports a
 * bad cable; -EKEYREJECTED when it fails identification; -EPROTO otherwise.
 */
int svooc_handle(struct svooc *sv, const struct svooc_request *req, u16 random, u8 *payload)
{
	u8 cmd = req->command;
	int index;

	if (req->head == SVOOC_HEAD_SVOOC) {
		sv->adapter = SVOOC_ADAPTER_SVOOC;
	} else if (req->head == SVOOC_HEAD_VOOC3) {
		sv->adapter = SVOOC_ADAPTER_VOOC3;
	} else {
		/*
		 * A VOOC 2.0 head carries only FASTCHG offers and power commands.
		 * A newer adapter opens the same way, so the offer is answered as
		 * VOOC 3.0 three times before the adapter is taken for VOOC 2.0,
		 * which has no identification and is granted on permit alone.
		 */
		if (sv->adapter != SVOOC_ADAPTER_VOOC2) {
			if (cmd != SVOOC_CMD_FASTCHG || sv->granted ||
			    sv->exchange != SVOOC_EXCHANGE_COMMAND)
				return -EPROTO;
			if (sv->adapter == SVOOC_ADAPTER_NONE)
				sv->adapter = SVOOC_ADAPTER_VOOC3;
			if (++sv->vooc2_offers <= SVOOC_VOOC2_OFFERS) {
				sv->offered = true;
				*payload = sv->circuit[1];
				return 1;
			}
			sv->adapter = SVOOC_ADAPTER_VOOC2;
			sv->identified = true;
			sv->model = 0x7f;
			sv->curve_sent = true;
		}
		switch (cmd) {
		case SVOOC_CMD_FASTCHG:
			sv->offered = true;
			if (sv->permit)
				sv->granted = true;
			*payload = sv->circuit[2] | (sv->granted ? SVOOC_ALLOW : 0);
			return 1;
		case SVOOC_CMD_GET_VBAT:
		case SVOOC_CMD_VBUS_OK:
		case SVOOC_CMD_GET_CURRENT:
		case SVOOC_CMD_AP_STATUS:
			return sv->granted ? 0 : -EACCES;
		case SVOOC_CMD_BAD_CABLE:
			return -ENOLINK;
		default:
			return -EPROTO;
		}
	}

	switch (sv->exchange) {
	case SVOOC_EXCHANGE_IDENTIFY:
		if (sv->remaining == SVOOC_IDENTIFY_FRAMES) {
			*payload = sv->second;
		} else {
			sv->response = (sv->response << 4) | cmd;
			*payload = 0x10;
		}
		if (!--sv->remaining) {
			sv->exchange = SVOOC_EXCHANGE_COMMAND;
			if ((sv->response & 0xfefe) != sv->challenge)
				return -EKEYREJECTED;
			sv->identified = true;
		}
		return 1;
	case SVOOC_EXCHANGE_MODEL:
		sv->model = (sv->model << 4) | cmd;
		*payload = 0x10;
		if (!--sv->remaining) {
			sv->model &= 0x7f;
			sv->exchange = SVOOC_EXCHANGE_COMMAND;
			if (!sv->model)
				return -EPROTO;
		}
		return 1;
	case SVOOC_EXCHANGE_CURVE:
		/* Three pairs, each asked twice, in the order of the curve. */
		index = SVOOC_CURVE_LEN - sv->remaining;
		if (cmd != index / 2 + 1)
			return -EPROTO;
		*payload = svooc_bypass(sv) ? sv->bypass_curve[index] : sv->curve[index];
		if (!--sv->remaining) {
			sv->curve_sent = true;
			sv->exchange = SVOOC_EXCHANGE_COMMAND;
		}
		return 1;
	case SVOOC_EXCHANGE_COMMAND:
		break;
	}

	switch (cmd) {
	case SVOOC_CMD_FASTCHG:
		sv->offered = true;
		if (sv->permit && sv->identified && sv->model && sv->curve_sent)
			sv->granted = true;
		*payload = sv->circuit[sv->adapter == SVOOC_ADAPTER_VOOC3] |
			   (sv->granted ? SVOOC_ALLOW : 0);
		return 1;
	case SVOOC_CMD_IDENTIFY:
		if (!sv->offered || sv->granted || ++sv->attempts > SVOOC_IDENTIFY_MAX)
			return -EPROTO;
		sv->identified = false;
		sv->curve_sent = false;
		*payload = svooc_challenge(sv, random);
		return 1;
	case SVOOC_CMD_TELL_MODEL:
		if (!sv->offered || sv->granted)
			return -EPROTO;
		sv->model = 0;
		sv->curve_sent = false;
		sv->exchange = SVOOC_EXCHANGE_MODEL;
		sv->remaining = SVOOC_MODEL_FRAMES;
		*payload = 0x28;
		return 1;
	case SVOOC_CMD_BAT_MODEL:
		if (!sv->identified || !sv->model || sv->granted)
			return -EPROTO;
		sv->curve_sent = false;
		sv->exchange = SVOOC_EXCHANGE_CURVE;
		sv->remaining = SVOOC_CURVE_LEN;
		*payload = 0x18;
		return 1;
	case SVOOC_CMD_GET_VBAT:
	case SVOOC_CMD_VBUS_OK:
	case SVOOC_CMD_GET_CURRENT:
	case SVOOC_CMD_AP_STATUS:
		return sv->granted ? 0 : -EACCES;
	case SVOOC_CMD_BAD_CABLE:
		return -ENOLINK;
	default:
		return -EPROTO;
	}
}

/** svooc_current_payload() - the GET_CURRENT reply, input current in 100 mA */
int svooc_current_payload(int ma, u8 *payload)
{
	if (ma < 100 || ma > 12700 || ma % 100)
		return -ERANGE;
	*payload = ma / 100;
	return 0;
}

/** svooc_vbat_payload() - the GET_VBAT reply, 10 mV steps from 3.404 V */
u8 svooc_vbat_payload(int mv)
{
	if (mv < 3404)
		return 0;
	if (mv > 4654)
		return 125;
	return (mv - 3404) / 10;
}

/*
 * svooc_vooc2_vbat_payload() - the VOOC 2.0 GET_VBAT reply
 *
 * Six bits of 16 mV from 3.392 V, capped at 61, with the allow bit on top;
 * zero below 3 V.
 */
u8 svooc_vooc2_vbat_payload(int mv, bool allow)
{
	u8 v;

	if (mv < 3000)
		return 0;
	if (mv < 3392)
		return allow ? SVOOC_ALLOW : 0;
	v = min((mv - 3392) / 16, 61);
	return v | (allow ? SVOOC_ALLOW : 0);
}

/** svooc_vbus_payload() - the VBUS_OK reply for VBUS against ratio × VBAT */
u8 svooc_vbus_payload(int vbus_mv, int vbat_mv, int ratio, int low_mv, int high_mv)
{
	int delta = vbus_mv - ratio * vbat_mv;

	if (delta < low_mv)
		return SVOOC_VBUS_LOW;
	if (delta > high_mv)
		return SVOOC_VBUS_HIGH;
	return SVOOC_VBUS_NORMAL;
}
