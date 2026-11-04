// SPDX-License-Identifier: GPL-2.0-only
/*
 * Charging strategy algorithms shared by the fast charging protocols.
 *
 * Adapted from the Oplus v2 charger strategies,
 * Copyright (C) 2020-2024 Oplus. All rights reserved.
 */
#include <linux/errno.h>
#include <linux/limits.h>
#include <linux/module.h>

#include "strategy.h"

#define FC_CURVE_OVER_MS	2500
#define FC_CURVE_CV_OVER_MS	10000
#define FC_TEMP_OVER_COUNTS	2
#define FC_TEMP_HYST		10
#define FC_LCF_RANGE_COUNTS	2
#define FC_LCF_FULL_COUNTS	6

/* CGCL */

int fc_cgcl_validate(const struct fc_cgcl *t)
{
	unsigned int i;

	if (!t->count)
		return -EINVAL;
	for (i = 0; i < t->count; i++) {
		const struct fc_cgcl_row *r = &t->rows[i];

		if (r->cool >= r->heat || r->value <= 0 ||
		    r->heat_next < 0 || r->heat_next >= (s32)t->count ||
		    r->cool_next < 0 || r->cool_next >= (s32)t->count)
			return -EINVAL;
		if (i && t->rows[i - 1].heat > r->heat)
			return -EINVAL;
	}
	return 0;
}

/* The first row whose heat limit holds; past them all, the last row. */
void fc_cgcl_init(const struct fc_cgcl *t, struct fc_cgcl_state *s, int temp)
{
	unsigned int i;

	s->temp = temp;
	s->index = t->count - 1;
	for (i = 0; i < t->count; i++) {
		if (temp <= t->rows[i].heat) {
			s->index = i;
			break;
		}
	}
}

/* Heating moves past heat, cooling at or below cool; a jump may cross rows. */
int fc_cgcl_update(const struct fc_cgcl *t, struct fc_cgcl_state *s, int temp)
{
	int temp_min = t->rows[0].cool;
	int temp_max = t->rows[t->count - 1].heat;
	int diff = temp - s->temp;

	while (diff) {
		const struct fc_cgcl_row *r = &t->rows[s->index];

		if (diff > 0 && temp > r->heat) {
			s->index = r->heat_next;
			r = &t->rows[s->index];
			s->temp = (temp > r->heat && r->heat < temp_max) ? r->heat : temp;
		} else if (diff < 0 && temp <= r->cool) {
			s->index = r->cool_next;
			r = &t->rows[s->index];
			s->temp = (temp <= r->cool && r->cool > temp_min) ? r->cool : temp;
		} else {
			s->temp = temp;
		}
		diff = temp - s->temp;
	}
	return t->rows[s->index].value;
}

/* Curve */

static bool fc_curve_same_group(const struct fc_curve_row *a, const struct fc_curve_row *b)
{
	return a->soc_lo == b->soc_lo && a->soc_hi == b->soc_hi &&
	       a->temp_lo == b->temp_lo && a->temp_hi == b->temp_hi;
}

int fc_curve_validate(const struct fc_curve *c, bool voltage_free)
{
	unsigned int i, j;

	if (!c->count || c->count > FC_CURVE_MAX_ROWS)
		return -EINVAL;
	for (i = 0; i < c->count; i++) {
		const struct fc_curve_row *r = &c->rows[i];

		if (r->soc_lo < 0 || r->soc_hi > 100 || r->soc_lo > r->soc_hi ||
		    r->temp_lo < -200 || r->temp_hi > 600 || r->temp_lo >= r->temp_hi ||
		    (voltage_free ? r->mv != 0 : (r->mv < 5000 || r->mv > 21000)) ||
		    r->vbat_mv < 2500 || r->vbat_mv > 4700 ||
		    r->ma < 100 || r->ma > 20000 ||
		    r->flags < 0 || r->flags > 3 || r->seconds < 0 ||
		    r->seconds > 86400 || r->level < 0 || r->level >= 64)
			return -ERANGE;
		if (r->level) {
			if (!i || !fc_curve_same_group(r, &c->rows[i - 1]) ||
			    r->level != c->rows[i - 1].level + 1 ||
			    c->rows[i - 1].flags & FC_CURVE_LAST)
				return -EINVAL;
		} else {
			if (i && !(c->rows[i - 1].flags & FC_CURVE_LAST))
				return -EINVAL;
			for (j = 0; j < i; j++) {
				const struct fc_curve_row *p = &c->rows[j];

				if (!p->level && p->soc_lo <= r->soc_hi &&
				    r->soc_lo <= p->soc_hi && p->temp_lo < r->temp_hi &&
				    r->temp_lo < p->temp_hi)
					return -EEXIST;
			}
		}
	}
	return c->rows[c->count - 1].flags & FC_CURVE_LAST ? 0 : -EINVAL;
}

static void fc_curve_enter(const struct fc_curve *c, struct fc_curve_state *s, u64 now_ms)
{
	s->over_deadline = 0;
	s->deadline = 0;
	s->count = 0;
	if (s->index < s->end && c->rows[s->index].seconds)
		s->deadline = now_ms + (u64)c->rows[s->index].seconds * 1000;
}

/*
 * fc_curve_select() - pick the group for a SOC and temperature region
 * @temp_lo: the lower boundary of the session's temperature region; the
 *	region comes from the session's state machine, not the raw reading
 *
 * The first gear is the first whose battery threshold is still above the
 * battery; past them all, the last gear.
 */
int fc_curve_select(const struct fc_curve *c, struct fc_curve_state *s, int soc,
		    int temp_lo, int vbat_mv, u64 now_ms)
{
	unsigned int i;

	for (i = 0; i < c->count; i++) {
		const struct fc_curve_row *r = &c->rows[i];

		if (r->level || soc < r->soc_lo || soc > r->soc_hi || r->temp_lo != temp_lo)
			continue;
		s->first = i;
		s->end = i + 1;
		while (s->end < c->count && fc_curve_same_group(r, &c->rows[s->end]))
			s->end++;
		for (s->index = i; s->index < s->end; s->index++)
			if (vbat_mv < c->rows[s->index].vbat_mv)
				break;
		if (s->index == s->end)
			s->index--;
		fc_curve_enter(c, s, now_ms);
		return 0;
	}
	return -ENOENT;
}

/*
 * fc_curve_step() - advance the gear by time or battery voltage
 *
 * Return: the gear to run, or NULL once the curve is finished.
 */
const struct fc_curve_row *fc_curve_step(const struct fc_curve *c, struct fc_curve_state *s,
					 int vbat_mv, u64 now_ms)
{
	const struct fc_curve_row *r;

	if (s->index >= s->end)
		return NULL;
	r = &c->rows[s->index];
	if (s->deadline && now_ms >= s->deadline) {
		s->index++;
		fc_curve_enter(c, s, now_ms);
	} else if (vbat_mv > r->vbat_mv &&
		   (r->flags & (FC_CURVE_LAST | FC_CURVE_CV)) != (FC_CURVE_LAST | FC_CURVE_CV)) {
		if (!s->over_deadline) {
			s->over_deadline = now_ms +
				(r->flags & FC_CURVE_CV ? FC_CURVE_CV_OVER_MS : FC_CURVE_OVER_MS);
		} else if (now_ms >= s->over_deadline) {
			s->index++;
			fc_curve_enter(c, s, now_ms);
		}
	} else {
		s->over_deadline = 0;
	}
	return fc_curve_current(c, s);
}

/*
 * fc_curve_step_vooc() - oplus_voocphy_request_fastchg_curv()
 *
 * The VOOC curve moves on once the battery is above the gear threshold
 * while the input current has come down near the gear's current, or when
 * the gear's time is up, seen on four consecutive ticks. The last gear is
 * left to the full-charge checks.
 */
const struct fc_curve_row *fc_curve_step_vooc(const struct fc_curve *c, struct fc_curve_state *s,
					      int vbat_mv, int ibus_ma, u64 now_ms)
{
	const struct fc_curve_row *r;

	if (s->index >= s->end)
		return NULL;
	r = &c->rows[s->index];
	if (r->flags & FC_CURVE_LAST)
		return r;
	if ((vbat_mv > r->vbat_mv && ibus_ma < r->ma + 1000) ||
	    (s->deadline && now_ms >= s->deadline)) {
		if (++s->count > 3) {
			s->index++;
			fc_curve_enter(c, s, now_ms);
		}
	} else {
		s->count = 0;
	}
	return fc_curve_current(c, s);
}

/* Low current full */

int fc_lcf_validate(const struct fc_lcf *t)
{
	unsigned int i;

	for (i = 0; i < t->count; i++) {
		const struct fc_lcf_row *r = &t->rows[i];

		if (r->temp_lo >= r->temp_hi || r->iterm <= 0 || r->vterm < 3000 ||
		    r->vterm > 4700 || r->exit < 0 || r->exit > 1)
			return -EINVAL;
	}
	return 0;
}

static int fc_lcf_range(const struct fc_lcf *t, int temp)
{
	unsigned int i;

	for (i = 0; i < t->count; i++)
		if (temp >= t->rows[i].temp_lo && temp < t->rows[i].temp_hi)
			return t->rows[i].temp_lo;
	return INT_MIN;
}

void fc_lcf_init(const struct fc_lcf *t, struct fc_lcf_state *s, int temp)
{
	s->range_lo = fc_lcf_range(t, temp);
	s->pending_lo = s->range_lo;
	s->pending_count = 0;
	s->full_count = 0;
}

/* A range change needs two readings; full needs more than six matches. */
bool fc_lcf_update(const struct fc_lcf *t, struct fc_lcf_state *s, int temp,
		   int vbat_mv, int ibat_ma)
{
	int lo = fc_lcf_range(t, temp);
	bool low = false;
	unsigned int i;

	if (lo != s->range_lo) {
		if (lo != s->pending_lo) {
			s->pending_lo = lo;
			s->pending_count = 1;
		} else if (++s->pending_count >= FC_LCF_RANGE_COUNTS) {
			s->range_lo = lo;
			s->pending_count = 0;
		}
	}
	if (s->range_lo == INT_MIN)
		return false;
	for (i = 0; i < t->count; i++) {
		const struct fc_lcf_row *r = &t->rows[i];

		if (r->temp_lo == s->range_lo && ibat_ma <= r->iterm && vbat_mv >= r->vterm) {
			low = true;
			break;
		}
	}
	if (!low) {
		s->full_count = 0;
		return false;
	}
	if (++s->full_count > FC_LCF_FULL_COUNTS) {
		s->full_count = 0;
		return true;
	}
	return false;
}

/* Temperature regions */

void fc_region_reset(const struct fc_region_cfg *cfg, struct fc_region_state *st)
{
	memcpy(st->eff, cfg->bound, sizeof(st->eff));
}

enum fc_region fc_region_of(const struct fc_region_cfg *cfg, int temp)
{
	if (temp < cfg->bound[FC_BOUND_LITTLE_COLD])
		return FC_REGION_LITTLE_COLD;
	if (temp < cfg->bound[FC_BOUND_COOL])
		return FC_REGION_COOL;
	if (temp < cfg->bound[FC_BOUND_LITTLE_COOL])
		return FC_REGION_LITTLE_COOL;
	if (temp < cfg->bound[FC_BOUND_LITTLE_COOL_HIGH])
		return FC_REGION_LITTLE_COOL_HIGH;
	if (temp < cfg->bound[FC_BOUND_NORMAL_LOW])
		return FC_REGION_NORMAL_LOW;
	if (temp < cfg->bound[FC_BOUND_NORMAL_HIGH])
		return FC_REGION_NORMAL_HIGH;
	return FC_REGION_WARM;
}

static const enum fc_temp_status fc_region_status[FC_REGION_COUNT] = {
	[FC_REGION_LITTLE_COLD] = FC_TEMP_LITTLE_COLD,
	[FC_REGION_COOL] = FC_TEMP_COOL,
	[FC_REGION_LITTLE_COOL] = FC_TEMP_LITTLE_COOL,
	[FC_REGION_LITTLE_COOL_HIGH] = FC_TEMP_LITTLE_COOL_HIGH,
	[FC_REGION_NORMAL_LOW] = FC_TEMP_NORMAL_LOW,
	[FC_REGION_NORMAL_HIGH] = FC_TEMP_NORMAL_HIGH,
	[FC_REGION_WARM] = FC_TEMP_WARM,
};

/* oplus_pps_temp_cur_range_init(): the plain boundaries, no hysteresis. */
void fc_region_init(const struct fc_region_cfg *cfg, struct fc_region_state *st, int temp)
{
	fc_region_reset(cfg, st);
	st->region = fc_region_of(cfg, temp);
	st->status = fc_region_status[st->region];
	st->change_count = 0;
}

static bool fc_region_counted(struct fc_region_state *st)
{
	if (++st->change_count < FC_TEMP_OVER_COUNTS)
		return false;
	st->change_count = 0;
	return true;
}

static void fc_region_switch_curve(const struct fc_region_cfg *cfg, struct fc_region_state *st,
				   bool *reselect)
{
	st->status = FC_TEMP_SWITCH_CURVE;
	st->region = FC_REGION_INIT;
	*reselect = true;
}

static void fc_region_move(const struct fc_region_cfg *cfg, struct fc_region_state *st,
			   enum fc_region region, enum fc_bound bound, int shift)
{
	st->region = region;
	st->status = fc_region_status[region];
	fc_region_reset(cfg, st);
	st->eff[bound] += shift;
}

static int fc_region_warm(const struct fc_region_cfg *cfg, struct fc_region_state *st,
			  int temp, bool *reselect)
{
	if (temp > cfg->bound[FC_BOUND_OVER_HIGH]) {
		if (fc_region_counted(st)) {
			st->status = FC_TEMP_EXIT;
			return cfg->over_curr;
		}
	} else if (temp < st->eff[FC_BOUND_NORMAL_HIGH]) {
		if (fc_region_counted(st)) {
			fc_region_switch_curve(cfg, st, reselect);
			fc_region_reset(cfg, st);
			st->eff[FC_BOUND_NORMAL_HIGH] += FC_TEMP_HYST;
		}
	} else {
		st->status = FC_TEMP_WARM;
		st->region = FC_REGION_WARM;
	}
	return cfg->normal_curr;
}

static bool fc_region_ge(const struct fc_region_cfg *cfg, int temp, int bound)
{
	return cfg->strict ? temp > bound : temp >= bound;
}

static int fc_region_normal_high(const struct fc_region_cfg *cfg, struct fc_region_state *st,
				 int temp, int soc, int vbat_mv, bool *reselect)
{
	int ret = cfg->normal_curr;

	switch (st->status) {
	case FC_TEMP_NORMAL_HIGH:
		if (fc_region_ge(cfg, temp, cfg->high_temp[0])) {
			st->status = FC_TEMP_HIGH0;
			ret = cfg->high_curr[0];
		} else if (temp >= st->eff[FC_BOUND_NORMAL_LOW]) {
			st->status = FC_TEMP_NORMAL_HIGH;
		} else {
			fc_region_move(cfg, st, FC_REGION_NORMAL_LOW, FC_BOUND_NORMAL_LOW, FC_TEMP_HYST);
		}
		break;
	case FC_TEMP_HIGH0:
		if (fc_region_ge(cfg, temp, cfg->high_temp[1])) {
			st->status = FC_TEMP_HIGH1;
			ret = cfg->high_curr[1];
		} else if (temp < cfg->low_temp_sub[0]) {
			st->status = FC_TEMP_LOW0;
			ret = cfg->low_curr[0];
		} else {
			ret = cfg->high_curr[0];
		}
		break;
	case FC_TEMP_HIGH1:
		if (fc_region_ge(cfg, temp, cfg->high_temp[2])) {
			st->status = FC_TEMP_HIGH2;
			ret = cfg->high_curr[2];
		} else if (temp < cfg->low_temp_sub[1]) {
			st->status = FC_TEMP_LOW1;
			ret = cfg->low_curr[1];
		} else {
			ret = cfg->high_curr[1];
		}
		break;
	case FC_TEMP_HIGH2:
		if (fc_region_ge(cfg, temp, st->eff[FC_BOUND_NORMAL_HIGH])) {
			if (fc_region_counted(st)) {
				if (soc <= cfg->warm_soc && vbat_mv <= cfg->warm_vol) {
					fc_region_switch_curve(cfg, st, reselect);
					ret = cfg->high_curr[2];
					fc_region_reset(cfg, st);
					st->eff[FC_BOUND_NORMAL_HIGH] -= FC_TEMP_HYST;
				} else {
					st->status = FC_TEMP_EXIT;
					ret = cfg->over_curr;
				}
			}
		} else if (fc_region_ge(cfg, temp, cfg->bound[FC_BOUND_OVER_HIGH])) {
			if (fc_region_counted(st)) {
				st->status = FC_TEMP_EXIT;
				ret = cfg->over_curr;
			}
		} else if (temp < cfg->low_temp_sub[2]) {
			st->status = FC_TEMP_LOW2;
			ret = cfg->low_curr[2];
		} else {
			ret = cfg->high_curr[2];
		}
		break;
	case FC_TEMP_LOW0:
		if (fc_region_ge(cfg, temp, cfg->high_temp[0])) {
			st->status = FC_TEMP_HIGH0;
			ret = cfg->high_curr[0];
		} else if (temp < cfg->low_temp_sub[0]) {
			st->status = FC_TEMP_NORMAL_HIGH;
			if (cfg->low0_reset)
				fc_region_reset(cfg, st);
		} else {
			ret = cfg->low_curr[0];
		}
		break;
	case FC_TEMP_LOW1:
		if (fc_region_ge(cfg, temp, cfg->high_temp[1])) {
			st->status = FC_TEMP_HIGH1;
			/* The vendor code returns the HIGH2 current here. */
			ret = cfg->high_curr[2];
		} else if (temp < cfg->low_temp_sub[0]) {
			st->status = FC_TEMP_LOW0;
			ret = cfg->low_curr[0];
		} else {
			ret = cfg->low_curr[1];
		}
		break;
	case FC_TEMP_LOW2:
		if (fc_region_ge(cfg, temp, cfg->high_temp[2])) {
			st->status = FC_TEMP_HIGH2;
			ret = cfg->high_curr[2];
		} else if (temp < cfg->low_temp_sub[1]) {
			st->status = FC_TEMP_LOW1;
			ret = cfg->low_curr[1];
		} else {
			ret = cfg->low_curr[2];
		}
		break;
	default:
		break;
	}
	return ret;
}

static int fc_region_normal_low(const struct fc_region_cfg *cfg, struct fc_region_state *st,
				int temp)
{
	if (temp < st->eff[FC_BOUND_NORMAL_LOW] && temp >= st->eff[FC_BOUND_LITTLE_COOL_HIGH]) {
		st->region = FC_REGION_NORMAL_LOW;
		st->status = FC_TEMP_NORMAL_LOW;
	} else if (temp >= st->eff[FC_BOUND_NORMAL_LOW]) {
		st->eff[FC_BOUND_NORMAL_LOW] -= FC_TEMP_HYST;
		st->status = FC_TEMP_NORMAL_HIGH;
		st->region = FC_REGION_NORMAL_HIGH;
	} else {
		fc_region_move(cfg, st, FC_REGION_LITTLE_COOL_HIGH, FC_BOUND_LITTLE_COOL_HIGH,
			       FC_TEMP_HYST);
	}
	return cfg->normal_curr;
}

static int fc_region_little_cool_high(const struct fc_region_cfg *cfg,
				      struct fc_region_state *st, int temp, int soc,
				      bool *reselect)
{
	if (temp < st->eff[FC_BOUND_LITTLE_COOL_HIGH] && temp >= st->eff[FC_BOUND_LITTLE_COOL]) {
		st->status = FC_TEMP_LITTLE_COOL_HIGH;
		st->region = FC_REGION_LITTLE_COOL_HIGH;
	} else if (temp >= st->eff[FC_BOUND_LITTLE_COOL_HIGH]) {
		if (soc <= cfg->soc_high) {
			if (fc_region_counted(st))
				fc_region_switch_curve(cfg, st, reselect);
		} else {
			st->status = FC_TEMP_NORMAL_LOW;
			st->region = FC_REGION_NORMAL_LOW;
		}
		fc_region_reset(cfg, st);
		st->eff[FC_BOUND_LITTLE_COOL_HIGH] -= FC_TEMP_HYST;
	} else {
		fc_region_move(cfg, st, FC_REGION_LITTLE_COOL, FC_BOUND_LITTLE_COOL, FC_TEMP_HYST);
	}
	return cfg->normal_curr;
}

static int fc_region_little_cool(const struct fc_region_cfg *cfg, struct fc_region_state *st,
				 int temp, int soc, bool *reselect)
{
	if (temp < st->eff[FC_BOUND_LITTLE_COOL] && temp >= st->eff[FC_BOUND_COOL]) {
		st->status = FC_TEMP_LITTLE_COOL;
		st->region = FC_REGION_LITTLE_COOL;
	} else if (temp >= st->eff[FC_BOUND_LITTLE_COOL]) {
		if (soc <= cfg->soc_high) {
			if (fc_region_counted(st))
				fc_region_switch_curve(cfg, st, reselect);
		} else {
			st->status = FC_TEMP_LITTLE_COOL_HIGH;
			st->region = FC_REGION_LITTLE_COOL_HIGH;
		}
		fc_region_reset(cfg, st);
		st->eff[FC_BOUND_LITTLE_COOL] -= FC_TEMP_HYST;
	} else {
		if (soc <= cfg->soc_high) {
			if (fc_region_counted(st))
				fc_region_switch_curve(cfg, st, reselect);
		} else {
			st->status = FC_TEMP_COOL;
			st->region = FC_REGION_COOL;
		}
		fc_region_reset(cfg, st);
		st->eff[FC_BOUND_COOL] += FC_TEMP_HYST;
	}
	return cfg->normal_curr;
}

static int fc_region_cool(const struct fc_region_cfg *cfg, struct fc_region_state *st,
			  int temp, int soc, bool *reselect)
{
	if (temp < cfg->low_temp - FC_TEMP_HYST) {
		if (fc_region_counted(st)) {
			st->status = FC_TEMP_EXIT;
			return cfg->over_curr;
		}
	} else if (temp < st->eff[FC_BOUND_COOL] && temp >= st->eff[FC_BOUND_LITTLE_COLD]) {
		st->region = FC_REGION_COOL;
		st->status = FC_TEMP_COOL;
	} else if (temp >= st->eff[FC_BOUND_COOL]) {
		if (fc_region_counted(st)) {
			if (soc <= cfg->soc_high) {
				fc_region_switch_curve(cfg, st, reselect);
			} else {
				st->status = FC_TEMP_LITTLE_COOL;
				st->region = FC_REGION_LITTLE_COOL;
			}
			fc_region_reset(cfg, st);
			st->eff[FC_BOUND_COOL] -= FC_TEMP_HYST;
		}
	} else {
		st->status = FC_TEMP_LITTLE_COLD;
		st->region = FC_REGION_LITTLE_COLD;
		fc_region_reset(cfg, st);
		if (cfg->bound[FC_BOUND_LITTLE_COLD] != cfg->low_temp)
			st->eff[FC_BOUND_LITTLE_COLD] += FC_TEMP_HYST;
	}
	return cfg->normal_curr;
}

static int fc_region_little_cold(const struct fc_region_cfg *cfg, struct fc_region_state *st,
				 int temp)
{
	if (temp < cfg->low_temp - FC_TEMP_HYST) {
		if (fc_region_counted(st)) {
			st->status = FC_TEMP_EXIT;
			return cfg->over_curr;
		}
	} else if (temp < st->eff[FC_BOUND_LITTLE_COLD]) {
		st->region = FC_REGION_LITTLE_COLD;
		st->status = FC_TEMP_LITTLE_COLD;
	} else {
		st->status = FC_TEMP_COOL;
		st->region = FC_REGION_COOL;
		fc_region_reset(cfg, st);
		if (cfg->bound[FC_BOUND_LITTLE_COLD] != cfg->low_temp)
			st->eff[FC_BOUND_LITTLE_COLD] -= FC_TEMP_HYST;
	}
	return cfg->normal_curr;
}

/*
 * fc_region_update() - oplus_pps_get_batt_temp_curr() without the CGCL override
 * @reselect: set when the curve must be chosen again for a new region
 *
 * Return: the current limit of the region, before the CGCL table.
 */
int fc_region_update(const struct fc_region_cfg *cfg, struct fc_region_state *st,
		     int temp, int soc, int vbat_mv, bool *reselect)
{
	*reselect = false;
	switch (st->region) {
	case FC_REGION_WARM:
		return fc_region_warm(cfg, st, temp, reselect);
	case FC_REGION_NORMAL_HIGH:
		return fc_region_normal_high(cfg, st, temp, soc, vbat_mv, reselect);
	case FC_REGION_NORMAL_LOW:
		return fc_region_normal_low(cfg, st, temp);
	case FC_REGION_LITTLE_COOL_HIGH:
		return fc_region_little_cool_high(cfg, st, temp, soc, reselect);
	case FC_REGION_LITTLE_COOL:
		return fc_region_little_cool(cfg, st, temp, soc, reselect);
	case FC_REGION_COOL:
		return fc_region_cool(cfg, st, temp, soc, reselect);
	case FC_REGION_LITTLE_COLD:
		return fc_region_little_cold(cfg, st, temp);
	default:
		return cfg->normal_curr;
	}
}
