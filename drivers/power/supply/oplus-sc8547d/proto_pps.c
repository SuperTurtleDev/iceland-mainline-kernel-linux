// SPDX-License-Identifier: GPL-2.0-only
/*
 * USB PD PPS through the PMIC: the ADSP negotiates with the adapter, the
 * session steers its requests from the charge pump readings.
 *
 * The third-party voltage loop follows oplus_third_pps_target_voltage_check()
 * and the current steps oplus_pps_current_work() of oplus_chg_pps.c,
 * Copyright (C) 2020-2024 Oplus. All rights reserved.
 */
#include <linux/minmax.h>
#include <linux/power/qcom-battmgr-pps.h>
#include <linux/usb/pd.h>

#include "core.h"

#define PPS_TYPE_PD_PPS		8
#define PPS_VBUS_MAX_MV		10500	/* PMIC path validated range, and the pump OVP */
#define PPS_START_VOL_MV	5500

struct pps_state {
	int count_over;
	int count_abnormal;
	int count_output_low;
	bool request_too_low;
};

static struct pps_state pps_priv;

static int pps_probe(struct fc_instance *inst)
{
	struct fastchg *fc = inst->fc;
	const struct fc_profile *p = &inst->prof;
	int target = min(p->target_vbus_mv, PPS_VBUS_MAX_MV);
	u32 type, pdos[7];
	int i, ret, max_ma = 0;

	ret = fc_pmic_pps(fc, QCOM_BATTMGR_PPS_TYPE, &type);
	if (ret)
		return ret;
	if (type != PPS_TYPE_PD_PPS)
		return -ENODEV;
	ret = fc_pmic_pps(fc, QCOM_BATTMGR_PPS_CAPS, pdos);
	if (ret)
		return ret;
	for (i = 0; i < ARRAY_SIZE(pdos); i++) {
		u32 v = pdos[i];

		if (pdo_type(v) != PDO_TYPE_APDO || pdo_apdo_type(v) != APDO_TYPE_PPS)
			continue;
		/* The adapter must start at 5 V and reach the pump target. */
		if (pdo_pps_apdo_min_voltage(v) > PPS_START_VOL_MV ||
		    pdo_pps_apdo_max_voltage(v) < target)
			continue;
		max_ma = max_t(int, max_ma, pdo_pps_apdo_max_current(v));
	}
	if (max_ma < p->start_curr_ma)
		return -ENODEV;
	inst->adapter_max_ma = max_ma;
	memset(&pps_priv, 0, sizeof(pps_priv));
	inst->priv = &pps_priv;
	return 0;
}

static int pps_request(struct fc_instance *inst, int mv, int ma)
{
	u32 target[2];

	target[0] = clamp(mv, PPS_START_VOL_MV - 500, PPS_VBUS_MAX_MV);
	target[1] = clamp(ma, 100, min(inst->adapter_max_ma, 6500));
	return fc_pmic_pps(inst->fc, QCOM_BATTMGR_PPS_TARGET, target);
}

enum pps_curr { PPS_CURR_LOW, PPS_CURR_OK, PPS_CURR_HIGH, PPS_CURR_OVER, PPS_CURR_ABNORMAL };

static enum pps_curr pps_current_check(struct fc_instance *inst, struct pps_state *st,
				       const struct fc_sample *s)
{
	const struct fc_tuning *t = &inst->prof.tune;
	int ibus = s->pump[0].ibus_ma + (s->pump_valid[1] && s->pump[1].switching ?
					 s->pump[1].ibus_ma : 0);
	int delta_ok = st->request_too_low ? t->ibus_ok_low_ma : t->ibus_ok_ma;
	int target = inst->target_ma;

	if (ibus > target + t->ibus_over_ma)
		return PPS_CURR_OVER;
	if (ibus > target + delta_ok)
		return PPS_CURR_HIGH;
	if (ibus > target - delta_ok)
		return PPS_CURR_OK;
	if (ibus < t->ibus_disconnect_ma)
		return PPS_CURR_ABNORMAL;
	return PPS_CURR_LOW;
}

static int pps_current_step(int from, int to)
{
	int d = abs(from - to);
	int step = d >= 500 ? 500 : d >= 300 ? 300 : d >= 100 ? 100 : 50;

	return from + clamp(to - from, -step, step);
}

/*
 * Walk the adapter voltage toward the pump's upper window while the input
 * current tracks the target, and step the requested current by at most
 * 500 mA per tick. Sustained over-current (ten ticks) is fatal, as in the
 * stock driver; the stock driver only counts it without a pump current
 * reading, which this board has, so the count is kept but never grows.
 */
static int pps_tick(struct fc_instance *inst, const struct fc_sample *s)
{
	struct pps_state *st = inst->priv;
	const struct fc_profile *p = &inst->prof;
	const struct fc_tuning *t = &p->tune;
	int target_max = min(p->target_vbus_mv, PPS_VBUS_MAX_MV);
	int vol = inst->request_mv;
	int vbus_min = s->vbat_mv * 2 + t->cp_min_delta_mv;
	int vtarget, next = vol;
	int wanted = inst->target_ma;
	bool check = false;

	if (inst->vbus_cap_mv) {
		/* A user ceiling below the pump window cannot be honoured. */
		if (inst->vbus_cap_mv < vbus_min)
			return -ERANGE;
		target_max = min(inst->vbus_cap_mv, target_max);
	}
	vtarget = target_max - t->cp_max_delta_mv;
	inst->target_ma = pps_current_step(inst->request_ma, wanted);
	if (inst->request_ma != wanted) {
		/* Let the current settle before touching the voltage. */
		inst->target_mv = vol;
		return 0;
	}
	if (vol < vtarget - t->vbus_debounce_mv) {
		check = true;
		next = vol + t->vbus_step_mv;
	} else if (vol > vtarget + t->vbus_debounce_mv) {
		next = vol - t->vbus_step_mv;
	} else {
		check = true;
	}
	if (check) {
		switch (pps_current_check(inst, st, s)) {
		case PPS_CURR_OVER:
			st->count_abnormal = 0;
			next = vol - t->vbus_step_over_mv;
			break;
		case PPS_CURR_HIGH:
			st->count_over = st->count_abnormal = 0;
			next = vol - t->vbus_step_mv;
			break;
		case PPS_CURR_OK:
			st->count_over = st->count_abnormal = 0;
			next = vol;
			break;
		case PPS_CURR_ABNORMAL:
			if (++st->count_abnormal >= t->abnormal_counts)
				return -ENOLINK;
			break;
		case PPS_CURR_LOW:
			st->count_over = st->count_abnormal = 0;
			break;
		}
	}
	if (next < vbus_min) {
		if (next < vol) {
			next = vol;
			if (st->count_output_low > t->req_low_counts)
				return -EOVERFLOW;
			st->count_output_low++;
			st->request_too_low = true;
		} else {
			st->count_output_low = 0;
		}
	} else if (next > target_max) {
		next = target_max;
		st->count_output_low = 0;
	} else {
		st->count_output_low = 0;
	}
	inst->target_mv = next;
	return 0;
}

static void pps_stop(struct fc_instance *inst)
{
	/* The PMIC returns to 5 V when the session restores its input. */
}

const struct fc_protocol_ops fc_pps_ops = {
	.name = "PPS",
	.id = FC_PROTO_PPS,
	.probe = pps_probe,
	.request = pps_request,
	.tick = pps_tick,
	.stop = pps_stop,
};
