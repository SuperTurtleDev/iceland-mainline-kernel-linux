// SPDX-License-Identifier: GPL-2.0-only
/*
 * UFCS through the main charge pump's PHY.
 *
 * Session steps follow oplus_chg_ufcs.c (oplus_ufcs_switch_check_work,
 * oplus_ufcs_volt_update_check, oplus_ufcs_check_current_low),
 * Copyright (C) 2023-2024 Oplus. All rights reserved.
 * Every adapter is treated as a third-party one: no authentication, no
 * vendor messages.
 */
#include <linux/limits.h>
#include <linux/minmax.h>
#include <linux/slab.h>

#include "core.h"
#include "ufcs.h"

#define UFCS_VBUS_OVER_MV	11000	/* twice the 5.5 V battery ceiling */
#define UFCS_VOL_ACTION_START_MV 20
#define UFCS_DEFAULT_VSTEP_MV	40

struct ufcs_state {
	struct ufcs link;
	u64 pdo[UFCS_PDO_MAX];
	int pdo_count;
	int count_low;
	int src_mv, src_ma;
};

static const struct {
	int istep_ma;
	int vstep_mv;
} ufcs_update_table[] = {
	{ 0, 20 }, { 200, 40 }, { 400, 80 }, { 800, 160 }, { 2000, 240 }, { 3000, 320 },
};

static void ufcs_sink_info(void *data, struct ufcs_sink_info *info)
{
	struct fc_instance *inst = data;
	const struct fc_sample *s = fc_sample(inst->fc);

	info->batt_temp_c = s->batt_temp / 10;
	info->usb_temp_c = s->conn_usb_mc == INT_MIN ? 25 : s->conn_usb_mc / 1000;
	info->vbat_mv = s->vbat_mv;
	info->ibat_ma = s->ibat_ma;
}

int fc_ufcs_alloc(struct fc_instance *inst)
{
	struct ufcs_state *st;

	if (!sc8547d_has_ufcs_phy(fc_main_pump(inst->fc)))
		return dev_err_probe(fc_dev(inst->fc), -ENODEV, "the main pump has no UFCS PHY\n");
	st = devm_kzalloc(fc_dev(inst->fc), sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;
	ufcs_init(&st->link, fc_dev(inst->fc), fc_main_pump(inst->fc));
	st->link.get_sink_info = ufcs_sink_info;
	st->link.sink_data = inst;
	inst->priv = st;
	return 0;
}

static int ufcs_probe(struct fc_instance *inst)
{
	struct ufcs_state *st = inst->priv;
	const struct fc_profile *p = &inst->prof;
	u64 dev_info;
	int ret, i, max_ma = 0;
	bool covered = false;

	st->count_low = 0;
	sc8547d_set_irq_handler(fc_main_pump(inst->fc), ufcs_irq, &st->link);
	ret = ufcs_handshake(&st->link);
	if (ret)
		goto fail;
	ret = ufcs_get_device_info(&st->link, &dev_info);
	if (ret)
		goto fail;
	dev_info(fc_dev(inst->fc), "UFCS adapter vendor %04llx hw %04llx\n",
		 UFCS_DEV_VENDOR(dev_info), UFCS_DEV_HW_VER(dev_info));
	ret = ufcs_config_watchdog(&st->link, p->tune.watchdog_ms);
	if (ret)
		goto fail;
	ret = ufcs_get_output_caps(&st->link, st->pdo, &st->pdo_count);
	if (ret)
		goto fail;
	for (i = 0; i < st->pdo_count; i++) {
		u64 m = st->pdo[i];

		if (i && UFCS_CAP_VOL_MIN(m) > UFCS_CAP_VOL_MAX(st->pdo[i - 1]))
			break;
		if (p->target_vbus_mv >= UFCS_CAP_VOL_MIN(m) &&
		    p->target_vbus_mv <= UFCS_CAP_VOL_MAX(m)) {
			covered = true;
			max_ma = max_t(int, max_ma, UFCS_CAP_CURR_MAX(m));
		}
	}
	if (!covered || max_ma < p->start_curr_ma) {
		ret = -ENODEV;
		goto fail;
	}
	inst->adapter_max_ma = max_ma;
	return 0;
fail:
	ufcs_exit(&st->link);
	sc8547d_set_irq_handler(fc_main_pump(inst->fc), NULL, NULL);
	return ret;
}

static int ufcs_do_request(struct fc_instance *inst, int mv, int ma)
{
	struct ufcs_state *st = inst->priv;

	return ufcs_request(&st->link, mv, clamp(ma, 100, inst->adapter_max_ma));
}

static int ufcs_vstep(struct ufcs_state *st, int target_mv)
{
	int i, vstep = 0;

	for (i = 0; i < st->pdo_count; i++) {
		u64 m = st->pdo[i];

		if (target_mv >= UFCS_CAP_VOL_MIN(m) && target_mv <= UFCS_CAP_VOL_MAX(m))
			vstep = max_t(int, vstep, UFCS_CAP_VOL_STEP(m));
	}
	return vstep > 0 ? vstep : UFCS_DEFAULT_VSTEP_MV;
}

static int ufcs_update_vstep(int diff_ma)
{
	int i;

	for (i = ARRAY_SIZE(ufcs_update_table) - 1; i >= 0; i--)
		if (diff_ma >= ufcs_update_table[i].istep_ma)
			return ufcs_update_table[i].vstep_mv;
	return 0;
}

/*
 * The adapter reports its own output current; the voltage request moves
 * by a step that grows with the distance to the target current.
 */
static int ufcs_tick(struct fc_instance *inst, const struct fc_sample *s)
{
	struct ufcs_state *st = inst->priv;
	const struct fc_tuning *t = &inst->prof.tune;
	u64 info;
	int ret, vstep, ibus, next, vol_min, vol_max;

	ret = ufcs_get_source_info(&st->link, &info);
	if (ret)
		return ret;
	st->src_mv = UFCS_SRC_VOL(info);
	st->src_ma = UFCS_SRC_CURR(info);
	if (st->src_ma < t->curr_low_ma) {
		if (++st->count_low >= t->curr_low_counts)
			return -ENOLINK;
	} else {
		st->count_low = 0;
	}
	vstep = ufcs_vstep(st, inst->request_mv);
	vol_min = (s->vbat_mv * 2 / vstep) * vstep + UFCS_VOL_ACTION_START_MV;
	vol_max = min(UFCS_VBUS_OVER_MV, inst->prof.target_vbus_mv);
	if (inst->vbus_cap_mv) {
		if (inst->vbus_cap_mv < vol_min)
			return -ERANGE;
		vol_max = min(inst->vbus_cap_mv, vol_max);
	}
	ibus = st->src_ma;
	next = inst->request_mv;
	if (ibus > inst->target_ma + t->ibus_adjust_ma)
		next -= ufcs_update_vstep(abs(ibus - inst->target_ma));
	else if (ibus < inst->target_ma - t->ibus_adjust_ma)
		next += ufcs_update_vstep(abs(ibus - inst->target_ma));
	next = clamp(next, vol_min, vol_max);
	inst->target_mv = next;
	return 0;
}

static void ufcs_stop(struct fc_instance *inst)
{
	struct ufcs_state *st = inst->priv;

	ufcs_exit(&st->link);
	sc8547d_set_irq_handler(fc_main_pump(inst->fc), NULL, NULL);
}

const struct fc_protocol_ops fc_ufcs_ops = {
	.name = "UFCS",
	.id = FC_PROTO_UFCS,
	.needs_dpdm = true,
	.probe = ufcs_probe,
	.request = ufcs_do_request,
	.tick = ufcs_tick,
	.stop = ufcs_stop,
};
