/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Charging strategy algorithms shared by the fast charging protocols.
 *
 * Algorithms follow the Oplus v2 charger strategies
 * (vendor/oplus/kernel/charger/v2/strategy/oplus_strategy_cgcl.c,
 * oplus_strategy_pps_ufcs_curve_v2.c, oplus_strategy_low_curr_full.c and
 * the temperature range state machine of oplus_chg_pps.c),
 * Copyright (C) 2020-2024 Oplus. All rights reserved.
 *
 * Every function is pure: it reads the table it is given and the sample the
 * caller took, and updates only the state object it is handed. Each protocol
 * instance owns its own tables and states. Temperatures are in tenths of a
 * degree Celsius, voltages in mV, currents in mA.
 */
#ifndef _OPLUS_FC_STRATEGY_H
#define _OPLUS_FC_STRATEGY_H

#include <linux/types.h>

/* Charge-gear-by-temperature with stateful hysteresis (CGCL). */
struct fc_cgcl_row {
	s32 cool;	/* move to cool_next when temp <= cool */
	s32 heat;	/* move to heat_next when temp > heat */
	s32 value;	/* the row's output: a current or a gear */
	s32 heat_next;
	s32 cool_next;
};

struct fc_cgcl {
	const struct fc_cgcl_row *rows;
	unsigned int count;
};

struct fc_cgcl_state {
	int index;
	int temp;
};

int fc_cgcl_validate(const struct fc_cgcl *t);
void fc_cgcl_init(const struct fc_cgcl *t, struct fc_cgcl_state *s, int temp);
int fc_cgcl_update(const struct fc_cgcl *t, struct fc_cgcl_state *s, int temp);

/*
 * SOC/temperature charging curve. A group is the run of rows with the same
 * SOC and temperature bounds; gears are numbered from zero within a group.
 * SOC bounds are inclusive, temperature bounds half-open.
 */
#define FC_CURVE_LAST	BIT(0)
#define FC_CURVE_CV	BIT(1)
#define FC_CURVE_MAX_ROWS 1024

struct fc_curve_row {
	s32 soc_lo, soc_hi, temp_lo, temp_hi, level;
	s32 mv, vbat_mv, ma, flags, seconds;
};

struct fc_curve {
	unsigned int count;
	struct fc_curve_row rows[];
};

struct fc_curve_state {
	unsigned int first, end, index;
	u64 deadline;		/* gear timeout, 0 when none */
	u64 over_deadline;	/* vbat above threshold since, 0 when not */
	unsigned int count;	/* consecutive matches of the VOOC switch condition */
};

int fc_curve_validate(const struct fc_curve *c, bool voltage_free);
int fc_curve_select(const struct fc_curve *c, struct fc_curve_state *s, int soc,
		    int temp_lo, int vbat_mv, u64 now_ms);
const struct fc_curve_row *fc_curve_step(const struct fc_curve *c, struct fc_curve_state *s,
					 int vbat_mv, u64 now_ms);
const struct fc_curve_row *fc_curve_step_vooc(const struct fc_curve *c, struct fc_curve_state *s,
					      int vbat_mv, int ibus_ma, u64 now_ms);
static inline const struct fc_curve_row *fc_curve_current(const struct fc_curve *c,
							  const struct fc_curve_state *s)
{
	return s->index < s->end ? &c->rows[s->index] : NULL;
}

/* Low current full: per temperature range rows of (iterm, vterm). */
struct fc_lcf_row {
	s32 temp_lo, temp_hi, iterm, vterm, exit;
};

struct fc_lcf {
	const struct fc_lcf_row *rows;
	unsigned int count;
};

struct fc_lcf_state {
	int range_lo;		/* temp_lo of the active range, INT_MIN when none */
	int pending_lo;
	int pending_count;
	int full_count;
};

int fc_lcf_validate(const struct fc_lcf *t);
void fc_lcf_init(const struct fc_lcf *t, struct fc_lcf_state *s, int temp);
bool fc_lcf_update(const struct fc_lcf *t, struct fc_lcf_state *s, int temp,
		   int vbat_mv, int ibat_ma);

/* Temperature range state machine of the PPS and UFCS sessions. */
enum fc_region {
	FC_REGION_LITTLE_COLD,
	FC_REGION_COOL,
	FC_REGION_LITTLE_COOL,
	FC_REGION_LITTLE_COOL_HIGH,
	FC_REGION_NORMAL_LOW,
	FC_REGION_NORMAL_HIGH,
	FC_REGION_WARM,
	FC_REGION_COUNT,
	FC_REGION_INIT = FC_REGION_COUNT,
};

enum fc_temp_status {
	FC_TEMP_NATURAL,
	FC_TEMP_LITTLE_COLD,
	FC_TEMP_COOL,
	FC_TEMP_LITTLE_COOL,
	FC_TEMP_LITTLE_COOL_HIGH,
	FC_TEMP_NORMAL_LOW,
	FC_TEMP_NORMAL_HIGH,
	FC_TEMP_HIGH0,
	FC_TEMP_HIGH1,
	FC_TEMP_HIGH2,
	FC_TEMP_LOW0,
	FC_TEMP_LOW1,
	FC_TEMP_LOW2,
	FC_TEMP_WARM,
	FC_TEMP_EXIT,
	FC_TEMP_SWITCH_CURVE,
};

/* Boundaries, indexed like the region they start */
enum fc_bound {
	FC_BOUND_LITTLE_COLD,	/* little_cold_temp: end of LITTLE_COLD */
	FC_BOUND_COOL,
	FC_BOUND_LITTLE_COOL,
	FC_BOUND_LITTLE_COOL_HIGH,
	FC_BOUND_NORMAL_LOW,
	FC_BOUND_NORMAL_HIGH,
	FC_BOUND_OVER_HIGH,
	FC_BOUND_COUNT,
};

struct fc_region_cfg {
	s32 bound[FC_BOUND_COUNT];
	s32 low_temp;		/* below: no session */
	s32 high_temp[3];	/* NORMAL_HIGH sub-states, rising */
	s32 high_curr[3];
	s32 low_temp_sub[3];	/* NORMAL_HIGH sub-states, falling */
	s32 low_curr[3];
	s32 normal_curr;
	s32 over_curr;
	s32 warm_soc;
	s32 warm_vol;
	s32 soc_high;
	bool strict;		/* UFCS compares with > where PPS uses >= */
	bool low0_reset;	/* PPS shifts the LOW0 boundary when it returns */
};

struct fc_region_state {
	enum fc_region region;
	enum fc_temp_status status;
	s32 eff[FC_BOUND_COUNT];	/* boundaries with hysteresis applied */
	int change_count;
};

void fc_region_reset(const struct fc_region_cfg *cfg, struct fc_region_state *st);
void fc_region_init(const struct fc_region_cfg *cfg, struct fc_region_state *st, int temp);
int fc_region_update(const struct fc_region_cfg *cfg, struct fc_region_state *st,
		     int temp, int soc, int vbat_mv, bool *reselect);
enum fc_region fc_region_of(const struct fc_region_cfg *cfg, int temp);

#endif
