// SPDX-License-Identifier: GPL-2.0-only
/*
 * Device tree constants of the fast charging protocol instances.
 *
 * Each protocol child node carries its own thresholds, tables and curve.
 * Protection counts, steps and timings have stock defaults that the node
 * may override; every other number must come from the device tree.
 */
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#include "core.h"

static const struct {
	const char *name;
	const struct fc_protocol_ops *ops;
} fc_protocols[FC_PROTO_COUNT] = {
	[FC_PROTO_VOOC] = { "vooc", &fc_vooc_ops },
	[FC_PROTO_UFCS] = { "ufcs", &fc_ufcs_ops },
	[FC_PROTO_PPS] = { "pps", &fc_pps_ops },
};

const char *fc_protocol_name(enum fc_proto proto)
{
	return fc_protocols[proto].name;
}

static int fc_dt_u32(struct device_node *np, const char *name, int min, int max, int *out)
{
	u32 v;
	int ret;

	ret = of_property_read_u32(np, name, &v);
	if (ret)
		return ret;
	if ((int)v < min || (int)v > max)
		return -ERANGE;
	*out = v;
	return 0;
}

static int fc_dt_s32_array(struct device_node *np, const char *name, int count, s32 *out)
{
	int n;

	n = of_property_count_u32_elems(np, name);
	if (n != count)
		return n < 0 ? n : -EINVAL;
	return of_property_read_u32_array(np, name, (u32 *)out, count);
}

static int fc_dt_matrix(struct fastchg *fc, struct device_node *np, const char *name,
			int cols, int max_rows, void **rows, unsigned int *count)
{
	int n;
	void *buf;

	n = of_property_count_u32_elems(np, name);
	if (n <= 0)
		return dev_err_probe(fc->dev, n ?: -ENODATA, "%pOFn: missing %s\n", np, name);
	if (n % cols || n / cols > max_rows)
		return dev_err_probe(fc->dev, -EINVAL, "%pOFn: %s needs %d cells per row\n",
				     np, name, cols);
	buf = devm_kcalloc(fc->dev, n, sizeof(u32), GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	if (of_property_read_u32_array(np, name, buf, n))
		return -EINVAL;
	*rows = buf;
	*count = n / cols;
	return 0;
}

static int fc_dt_curve(struct fastchg *fc, struct device_node *np, struct fc_instance *inst,
		       struct fc_curve **out)
{
	struct fc_curve *curve;
	int n, ret;

	n = of_property_count_u32_elems(np, "oplus,charge-curve");
	if (n <= 0 || n % 10 || n / 10 > FC_CURVE_MAX_ROWS)
		return dev_err_probe(fc->dev, -EINVAL, "%pOFn: invalid charge curve\n", np);
	curve = kvzalloc(struct_size(curve, rows, n / 10), GFP_KERNEL);
	if (!curve)
		return -ENOMEM;
	curve->count = n / 10;
	ret = of_property_read_u32_array(np, "oplus,charge-curve", (u32 *)curve->rows, n);
	if (!ret)
		ret = fc_curve_validate(curve, inst->ops->adapter_regulated);
	if (ret) {
		kvfree(curve);
		return dev_err_probe(fc->dev, ret, "%pOFn: charge curve rejected\n", np);
	}
	*out = curve;
	return 0;
}

/* Current (mA), voltage (mV) pairs the adapter reads, sent last pair first. */
static int fc_dt_battery_curve(struct fastchg *fc, struct device_node *np, const char *name,
			       u8 *out)
{
	s32 bc[6];
	int ret, i;

	ret = fc_dt_s32_array(np, name, 6, bc);
	if (ret)
		return dev_err_probe(fc->dev, ret, "%pOFn: %s\n", np, name);
	for (i = 0; i < 3; i++) {
		int ma = bc[2 * i], mv = bc[2 * i + 1];

		if (ma < 100 || ma > 12700 || ma % 100 || mv < 3404 || mv > 4664)
			return dev_err_probe(fc->dev, -ERANGE, "%pOFn: %s\n", np, name);
		out[5 - 2 * i] = ma / 100;
		out[4 - 2 * i] = (mv - 3404) / 10;
	}
	return 0;
}

/* VOOC 2.0/3.0: the pumps run 1:1 on a curve of their own. */
static int fc_dt_bypass(struct fastchg *fc, struct device_node *np, struct fc_instance *inst)
{
	struct fc_profile *p = &inst->prof;
	struct device_node *child;
	int ret;

	child = of_get_child_by_name(np, "bypass");
	if (!child)
		return 0;
	ret = fc_dt_curve(fc, child, inst, &p->bypass_curve);
	if (!ret)
		ret = fc_dt_battery_curve(fc, child, "oplus,battery-curve", p->bypass_battery_curve);
	if (!ret) {
		p->bypass_curr_max_ma = p->curr_max_ma;
		ret = fc_dt_u32(child, "oplus,maximum-current-milliamp", 100, 20000,
				&p->bypass_curr_max_ma);
		if (ret == -EINVAL)
			ret = 0;
	}
	of_node_put(child);
	return ret;
}

static int fc_dt_region(struct fastchg *fc, struct device_node *np, struct fc_instance *inst)
{
	struct fc_region_cfg *r = &inst->prof.region;
	s32 temps[6], currs[6];
	int ret, i;

	ret = fc_dt_s32_array(np, "oplus,temperature-boundaries", FC_BOUND_COUNT, r->bound);
	if (ret)
		return dev_err_probe(fc->dev, ret, "%pOFn: temperature boundaries\n", np);
	for (i = 1; i < FC_BOUND_COUNT; i++)
		if (r->bound[i] <= r->bound[i - 1])
			return dev_err_probe(fc->dev, -EINVAL, "%pOFn: boundaries not increasing\n", np);
	ret = fc_dt_u32(np, "oplus,low-temperature", -200, r->bound[0], &r->low_temp);
	if (!ret)
		ret = fc_dt_u32(np, "oplus,warm-soc-percent", 0, 100, &r->warm_soc);
	if (!ret)
		ret = fc_dt_u32(np, "oplus,warm-voltage-millivolt", 3000, 4700, &r->warm_vol);
	if (ret)
		return dev_err_probe(fc->dev, ret, "%pOFn: region limits\n", np);
	if (inst->ops->adapter_regulated)
		return 0;
	ret = fc_dt_u32(np, "oplus,normal-current-milliamp", 100, 20000, &r->normal_curr);
	if (!ret)
		ret = fc_dt_s32_array(np, "oplus,high-temperature-thresholds", 6, temps);
	if (!ret)
		ret = fc_dt_s32_array(np, "oplus,high-temperature-currents-milliamp", 6, currs);
	if (ret)
		return dev_err_probe(fc->dev, ret, "%pOFn: high temperature sub-states\n", np);
	r->over_curr = r->normal_curr;
	for (i = 0; i < 3; i++) {
		r->high_temp[i] = temps[i];
		r->low_temp_sub[i] = temps[3 + i];
		r->high_curr[i] = currs[i];
		r->low_curr[i] = currs[3 + i];
	}
	r->strict = inst->ops->id == FC_PROTO_UFCS;
	r->low0_reset = inst->ops->id == FC_PROTO_PPS;
	return 0;
}

static void fc_dt_opt(struct device_node *np, const char *name, int *out, int min, int max,
		      struct fastchg *fc, int *err)
{
	u32 v;
	int ret;

	ret = of_property_read_u32(np, name, &v);
	if (ret == -EINVAL)
		return;	/* absent: keep the default */
	if (ret) {
		dev_err(fc->dev, "%pOFn: %s malformed\n", np, name);
		*err = ret;
		return;
	}
	if ((int)v < min || (int)v > max) {
		dev_err(fc->dev, "%pOFn: %s out of range\n", np, name);
		*err = -ERANGE;
		return;
	}
	*out = v;
}

static void fc_dt_opt_array(struct device_node *np, const char *name, int *out, int n,
			    int min, int max, struct fastchg *fc, int *err)
{
	s32 v[8];
	int i, count;

	count = of_property_count_u32_elems(np, name);
	if (count < 0)
		return;
	if (count != n || n > ARRAY_SIZE(v) ||
	    of_property_read_u32_array(np, name, (u32 *)v, n)) {
		dev_err(fc->dev, "%pOFn: %s needs %d cells\n", np, name, n);
		*err = -EINVAL;
		return;
	}
	for (i = 0; i < n; i++) {
		if (v[i] < min || v[i] > max) {
			dev_err(fc->dev, "%pOFn: %s entry %d out of range\n", np, name, i);
			*err = -ERANGE;
			return;
		}
		out[i] = v[i];
	}
}

/* Stock defaults from oplus_chg_pps.c, oplus_chg_ufcs.c and oplus_virtual_cp.c. */
static int fc_dt_tuning(struct fastchg *fc, struct device_node *np, struct fc_instance *inst)
{
	struct fc_tuning *t = &inst->prof.tune;
	int err = 0, v[4];

	t->full_cool_counts = 6;
	t->full_sw_counts = 6;
	t->full_hw_counts = 3;
	t->full_warm_counts = 6;
	t->ibat_high_counts = 8;
	t->ibat_low_ma = 3000;
	t->ibat_low_counts = 7;
	t->conn_counts = 8;
	t->tfg_dc = 800;
	t->tfg_counts = 6;
	t->start_delta_mv = 450;
	t->start_window_mv = 200;
	t->start_step_ms = 500;
	t->start_retries = 3;
	t->second_settle_ms = 300;
	t->second_min_ma = 500;
	t->second_retries = 2;
	t->vbus_step_mv = 40;
	t->vbus_step_over_mv = 100;
	t->vbus_debounce_mv = 80;
	t->cp_min_delta_mv = 100;
	t->cp_max_delta_mv = 500;
	t->ibus_ok_ma = 100;
	t->ibus_ok_low_ma = 200;
	t->ibus_over_ma = 500;
	t->ibus_disconnect_ma = 300;
	t->over_counts = 10;
	t->abnormal_counts = 4;
	t->req_low_counts = 4;
	t->watchdog_ms = 3000;
	t->curr_low_ma = 150;
	t->curr_low_counts = 6;
	t->ibus_adjust_ma = 50;

	v[0] = t->full_cool_counts; v[1] = t->full_sw_counts;
	v[2] = t->full_hw_counts; v[3] = t->full_warm_counts;
	fc_dt_opt_array(np, "oplus,full-counts", v, 4, 1, 100, fc, &err);
	t->full_cool_counts = v[0]; t->full_sw_counts = v[1];
	t->full_hw_counts = v[2]; t->full_warm_counts = v[3];
	fc_dt_opt(np, "oplus,battery-overcurrent-counts", &t->ibat_high_counts, 1, 100, fc, &err);
	fc_dt_opt(np, "oplus,battery-discharge-milliamp", &t->ibat_low_ma, 100, 10000, fc, &err);
	fc_dt_opt(np, "oplus,battery-discharge-counts", &t->ibat_low_counts, 1, 100, fc, &err);
	fc_dt_opt(np, "oplus,connector-over-counts", &t->conn_counts, 1, 100, fc, &err);
	fc_dt_opt(np, "oplus,gauge-overtemperature", &t->tfg_dc, 500, 1500, fc, &err);
	fc_dt_opt(np, "oplus,gauge-overtemperature-counts", &t->tfg_counts, 1, 100, fc, &err);
	fc_dt_opt(np, "oplus,start-vbus-delta-millivolt", &t->start_delta_mv, 100, 2000, fc, &err);
	fc_dt_opt(np, "oplus,start-vbus-window-millivolt", &t->start_window_mv, 50, 1000, fc, &err);
	fc_dt_opt(np, "oplus,start-step-ms", &t->start_step_ms, 50, 2000, fc, &err);
	fc_dt_opt(np, "oplus,start-retries", &t->start_retries, 1, 10, fc, &err);
	fc_dt_opt(np, "oplus,switch-curve-current-milliamp", &t->switch_curve_ma, 500, 10000, fc, &err);
	fc_dt_opt(np, "oplus,pump-maximum-milliamp", &t->pump_max_ma, 1000, 10000, fc, &err);
	fc_dt_opt(np, "oplus,second-pump-settle-ms", &t->second_settle_ms, 50, 5000, fc, &err);
	fc_dt_opt(np, "oplus,second-pump-minimum-milliamp", &t->second_min_ma, 100, 3000, fc, &err);
	fc_dt_opt(np, "oplus,second-pump-retries", &t->second_retries, 0, 10, fc, &err);
	v[0] = t->vbus_step_mv; v[1] = t->vbus_step_over_mv; v[2] = t->vbus_debounce_mv;
	fc_dt_opt_array(np, "oplus,feedback-vbus-step-millivolt", v, 3, 10, 1000, fc, &err);
	t->vbus_step_mv = v[0]; t->vbus_step_over_mv = v[1]; t->vbus_debounce_mv = v[2];
	v[0] = t->cp_min_delta_mv; v[1] = t->cp_max_delta_mv;
	fc_dt_opt_array(np, "oplus,feedback-vbus-margin-millivolt", v, 2, 20, 2000, fc, &err);
	t->cp_min_delta_mv = v[0]; t->cp_max_delta_mv = v[1];
	v[0] = t->ibus_ok_ma; v[1] = t->ibus_ok_low_ma; v[2] = t->ibus_over_ma; v[3] = t->ibus_disconnect_ma;
	fc_dt_opt_array(np, "oplus,feedback-current-window-milliamp", v, 4, 10, 3000, fc, &err);
	t->ibus_ok_ma = v[0]; t->ibus_ok_low_ma = v[1]; t->ibus_over_ma = v[2]; t->ibus_disconnect_ma = v[3];
	v[0] = t->over_counts; v[1] = t->abnormal_counts; v[2] = t->req_low_counts;
	fc_dt_opt_array(np, "oplus,feedback-counts", v, 3, 1, 100, fc, &err);
	t->over_counts = v[0]; t->abnormal_counts = v[1]; t->req_low_counts = v[2];
	fc_dt_opt(np, "oplus,adapter-watchdog-ms", &t->watchdog_ms, 500, 30000, fc, &err);
	fc_dt_opt(np, "oplus,adapter-low-current-milliamp", &t->curr_low_ma, 50, 2000, fc, &err);
	fc_dt_opt(np, "oplus,adapter-low-current-counts", &t->curr_low_counts, 1, 100, fc, &err);
	fc_dt_opt(np, "oplus,adapter-current-window-milliamp", &t->ibus_adjust_ma, 10, 1000, fc, &err);
	if (!err && (t->cp_min_delta_mv >= t->cp_max_delta_mv ||
		     t->vbus_debounce_mv >= t->cp_max_delta_mv ||
		     t->ibus_ok_ma >= t->ibus_over_ma || t->ibus_ok_low_ma >= t->ibus_over_ma)) {
		dev_err(fc->dev, "%pOFn: inconsistent feedback tuning\n", np);
		err = -EINVAL;
	}
	return err;
}

static int fc_dt_protocol(struct fastchg *fc, struct device_node *np, struct fc_instance *inst)
{
	struct fc_profile *p = &inst->prof;
	s32 range[2], full[4];
	unsigned int count;
	int ret;

	ret = fc_dt_region(fc, np, inst);
	if (!ret)
		ret = fc_dt_tuning(fc, np, inst);
	if (ret)
		return ret;
	ret = fc_dt_u32(np, "oplus,maximum-current-milliamp", 100, 20000, &p->curr_max_ma);
	if (!ret)
		ret = fc_dt_u32(np, "oplus,start-current-milliamp", 100, 3000, &p->start_curr_ma);
	if (!ret)
		ret = fc_dt_u32(np, "oplus,session-timeout-seconds", 60, 86400, &p->timeout_s);
	if (!ret)
		ret = fc_dt_u32(np, "oplus,battery-overcurrent-milliamp", 1000, 30000,
				&p->ibat_over_ma);
	if (!ret)
		ret = fc_dt_u32(np, "oplus,full-recheck-temperature", -200, 600,
				&p->full_recheck_temp);
	if (!ret)
		ret = fc_dt_u32(np, "oplus,high-temperature", p->region.low_temp, 700, &p->high_temp);
	if (!ret)
		ret = fc_dt_s32_array(np, "oplus,soc-range-percent", 2, range);
	if (ret)
		return dev_err_probe(fc->dev, ret, "%pOFn: session limits\n", np);
	p->soc_low = range[0];
	p->soc_high = range[1];
	p->region.soc_high = range[1];
	if (p->soc_low < 0 || p->soc_high > 100 || p->soc_low > p->soc_high)
		return dev_err_probe(fc->dev, -ERANGE, "%pOFn: SOC range\n", np);
	if (!of_property_read_u32_array(np, "oplus,second-pump-milliamp", (u32 *)range, 2)) {
		p->second_on_ma = range[0];
		p->second_off_ma = range[1];
		if (p->second_off_ma >= p->second_on_ma || p->second_on_ma > 10000)
			return dev_err_probe(fc->dev, -ERANGE, "%pOFn: second pump thresholds\n", np);
	}
	if (inst->ops->adapter_regulated) {
		s32 v[2 * FC_REGION_COUNT], bc[6];
		int i;

		ret = fc_dt_s32_array(np, "oplus,full-voltages-millivolt", 2 * FC_REGION_COUNT, v);
		if (ret)
			return dev_err_probe(fc->dev, ret, "%pOFn: full voltages\n", np);
		for (i = 0; i < FC_REGION_COUNT; i++) {
			p->full_1time_mv[i] = v[2 * i];
			p->full_ntime_mv[i] = v[2 * i + 1];
		}
		p->pump_max_ma = p->tune.pump_max_ma;
		ret = p->pump_max_ma ? 0 : -EINVAL;
		if (!ret)
			ret = fc_dt_s32_array(np, "oplus,vbus-window-millivolt", 2, range);
		if (!ret) {
			p->vbus_low_mv = range[0];
			p->vbus_high_mv = range[1];
			ret = fc_dt_s32_array(np, "oplus,circuit-codes", 3, bc);
		}
		if (ret)
			return dev_err_probe(fc->dev, ret, "%pOFn: VOOC limits\n", np);
		for (i = 0; i < 3; i++) {
			if (bc[i] < 0 || bc[i] > 0x3f)
				return dev_err_probe(fc->dev, -ERANGE, "%pOFn: circuit code\n", np);
			p->circuit[i] = bc[i];
		}
		ret = fc_dt_battery_curve(fc, np, "oplus,battery-curve", p->battery_curve);
		if (!ret)
			ret = fc_dt_bypass(fc, np, inst);
		if (ret)
			return ret;
	} else {
		ret = fc_dt_u32(np, "oplus,target-vbus-millivolt", 5500, 21000, &p->target_vbus_mv);
		if (!ret)
			ret = fc_dt_s32_array(np, "oplus,full-voltages-millivolt", 4, full);
		if (ret)
			return dev_err_probe(fc->dev, ret, "%pOFn: adapter limits\n", np);
		p->full_cool_sw_mv = full[0];
		p->full_normal_sw_mv = full[1];
		p->full_normal_hw_mv = full[2];
		p->full_warm_mv = full[3];
	}
	ret = fc_dt_matrix(fc, np, "oplus,cgcl", 5, 16, (void **)&inst->cgcl_rows, &count);
	if (ret)
		return ret;
	p->cgcl.rows = inst->cgcl_rows;
	p->cgcl.count = count;
	if (fc_cgcl_validate(&p->cgcl))
		return dev_err_probe(fc->dev, -EINVAL, "%pOFn: CGCL table rejected\n", np);
	ret = fc_dt_matrix(fc, np, "oplus,low-current-full", 5, 64, (void **)&inst->lcf_rows, &count);
	if (ret)
		return ret;
	p->lcf.rows = inst->lcf_rows;
	p->lcf.count = count;
	if (fc_lcf_validate(&p->lcf))
		return dev_err_probe(fc->dev, -EINVAL, "%pOFn: low current full table rejected\n", np);
	return fc_dt_curve(fc, np, inst, &p->curve);
}

static int fc_dt_charger(struct fastchg *fc)
{
	struct device_node *np = fc->dev->of_node;
	int n, i;

	n = of_property_count_u32_elems(np, "oplus,comm-temperature-thresholds");
	if (n > 0) {
		/*
		 * The stock V3P7 table has eight boundaries; the common charger
		 * expands them to its ten regions (oplus_comm_temp_region_map):
		 * LITTLE_COLD_HIGH repeats LITTLE_COLD, NORMAL_MID repeats NORMAL.
		 */
		static const u8 map[10] = { 0, 1, 1, 2, 3, 4, 5, 5, 6, 7 };
		s32 compact[8];

		if (n != ARRAY_SIZE(compact) ||
		    of_property_read_u32_array(np, "oplus,comm-temperature-thresholds",
					       (u32 *)compact, n))
			return dev_err_probe(fc->dev, -EINVAL, "comm temperature thresholds\n");
		for (i = 1; i < n; i++)
			if (compact[i] <= compact[i - 1])
				return dev_err_probe(fc->dev, -EINVAL, "comm thresholds not increasing\n");
		for (i = 0; i < ARRAY_SIZE(map); i++)
			fc->comm_thr[i] = compact[map[i]];
		fc->comm_count = ARRAY_SIZE(map);
		memcpy(fc->comm_dyn, fc->comm_thr, sizeof(fc->comm_thr));
		fc->comm_region = -1;
	}
	n = of_property_count_u32_elems(np, "oplus,cooling-battery-current-milliamp");
	if (n > 0) {
		if (n < 2 || n > 64)
			return dev_err_probe(fc->dev, -EINVAL, "cooling table size\n");
		fc->cooling_ma = devm_kcalloc(fc->dev, n, sizeof(u32), GFP_KERNEL);
		if (!fc->cooling_ma)
			return -ENOMEM;
		if (of_property_read_u32_array(np, "oplus,cooling-battery-current-milliamp",
					       fc->cooling_ma, n))
			return -EINVAL;
		/* State 0 is no cooling; each further state may only lower the current. */
		for (i = 0; i < n; i++) {
			if (fc->cooling_ma[i] < 200 || fc->cooling_ma[i] > 40000)
				return dev_err_probe(fc->dev, -ERANGE, "cooling table entry %d\n", i);
			if (i && fc->cooling_ma[i] > fc->cooling_ma[i - 1])
				return dev_err_probe(fc->dev, -EINVAL,
						     "cooling table must not increase at %d\n", i);
		}
		fc->cooling_states = n;
	}
	return 0;
}

int fc_dt_parse(struct fastchg *fc)
{
	struct device_node *np;
	const char *name;
	int ret, i, j, n;

	ret = fc_dt_charger(fc);
	if (ret)
		return ret;

	n = of_property_count_strings(fc->dev->of_node, "oplus,protocols");
	if (n < 1 || n > FC_PROTO_COUNT)
		return dev_err_probe(fc->dev, -EINVAL, "oplus,protocols lists the probe order\n");
	for (i = 0; i < n; i++) {
		ret = of_property_read_string_index(fc->dev->of_node, "oplus,protocols", i, &name);
		if (ret)
			return ret;
		for (j = 0; j < FC_PROTO_COUNT; j++)
			if (!strcmp(name, fc_protocols[j].name))
				break;
		if (j == FC_PROTO_COUNT || fc->inst[j].ops)
			return dev_err_probe(fc->dev, -EINVAL, "unknown or repeated protocol %s\n", name);
		np = of_get_child_by_name(fc->dev->of_node, name);
		if (!np)
			return dev_err_probe(fc->dev, -EINVAL, "missing %s node\n", name);
		fc->inst[j].ops = fc_protocols[j].ops;
		ret = fc_dt_protocol(fc, np, &fc->inst[j]);
		of_node_put(np);
		if (ret) {
			fc_dt_free(fc);
			return ret;
		}
		fc->order[fc->order_count++] = j;
	}
	return 0;
}

void fc_dt_free(struct fastchg *fc)
{
	int i;

	for (i = 0; i < FC_PROTO_COUNT; i++) {
		kvfree(fc->inst[i].prof.curve);
		fc->inst[i].prof.curve = NULL;
		kvfree(fc->inst[i].prof.bypass_curve);
		fc->inst[i].prof.bypass_curve = NULL;
	}
}

/*
 * Replace a protocol's curve. @proto equal to FC_PROTO_COUNT selects the
 * VOOC bypass curve. Refused while that protocol runs a session.
 */
int fc_curve_install(struct fastchg *fc, int proto, struct fc_curve *curve)
{
	bool bypass = proto == FC_PROTO_COUNT;
	struct fc_instance *inst = &fc->inst[bypass ? FC_PROTO_VOOC : proto];
	struct fc_curve **slot = bypass ? &inst->prof.bypass_curve : &inst->prof.curve;
	int ret;

	ret = fc_curve_validate(curve, inst->ops->adapter_regulated);
	if (ret)
		return ret;
	if (fc->active == inst && fc->state != FC_IDLE && fc->state != FC_HOLD)
		return -EBUSY;
	kvfree(*slot);
	*slot = curve;
	return 0;
}
