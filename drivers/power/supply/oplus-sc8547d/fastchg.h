/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Fast charging session core: one driver, one protocol instance per
 * supported adapter protocol, shared hardware and strategy algorithms.
 *
 * The session owns both charge pumps, the D+/D- switch and the PMIC input.
 * On a new charger it probes the protocol instances in the configured
 * order; the one that recognizes the adapter loads its own constants and
 * runs the charge. Everything hardware related goes through the session so
 * two protocols can never drive the pumps at the same time.
 */
#ifndef _OPLUS_FASTCHG_H
#define _OPLUS_FASTCHG_H

#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include "sc8547d.h"
#include "strategy.h"

struct fastchg;
struct fc_instance;

enum fc_proto {
	FC_PROTO_VOOC,
	FC_PROTO_UFCS,
	FC_PROTO_PPS,
	FC_PROTO_COUNT,
};

enum fc_state {
	FC_IDLE,	/* no charger, or waiting for one to settle */
	FC_PROBING,	/* a protocol is talking to the adapter */
	FC_STARTING,	/* raising the adapter voltage, pumps off */
	FC_RUNNING,	/* pumps on */
	FC_RESTORING,	/* leaving: pumps off, PHY released, PMIC input back */
	FC_HOLD,	/* not charging until the hold expires or a replug */
};

/* Why the session left fast charging; decides the hold. */
enum fc_stop_reason {
	FC_STOP_NONE,
	FC_STOP_UNPLUG,
	FC_STOP_NOT_SUPPORTED,	/* no protocol took the adapter */
	FC_STOP_NOT_ALLOWED,	/* admission: retry later */
	FC_STOP_SWITCH_CURVE,	/* temperature region change: restart at once */
	FC_STOP_FULL,		/* full or timeout: until replug */
	FC_STOP_FULL_COOL,	/* full in a cold region: retry when warmer */
	FC_STOP_ERROR,		/* protocol or hardware error: bounded retries */
	FC_STOP_FATAL,		/* over-current or similar: until replug */
	FC_STOP_DISABLED,	/* user or suspend */
};

/*
 * Protection counts, feedback steps and timings of one protocol instance.
 * All have stock defaults; the device tree may override any of them.
 */
struct fc_tuning {
	int full_cool_counts, full_sw_counts, full_hw_counts, full_warm_counts;
	int ibat_high_counts, ibat_low_ma, ibat_low_counts;
	int conn_counts, tfg_dc, tfg_counts;
	int start_delta_mv, start_window_mv, start_step_ms, start_retries;
	int switch_curve_ma;	/* lower to this before leaving for a new curve, 0: none */
	int pump_max_ma;	/* one pump's input ceiling, 0: none */
	int second_settle_ms, second_min_ma, second_retries;
	/* pump-feedback voltage loop (PPS) */
	int vbus_step_mv, vbus_step_over_mv, vbus_debounce_mv;
	int cp_min_delta_mv, cp_max_delta_mv;
	int ibus_ok_ma, ibus_ok_low_ma, ibus_over_ma, ibus_disconnect_ma;
	int over_counts, abnormal_counts, req_low_counts;
	/* adapter-reported current loop (UFCS) */
	int watchdog_ms, curr_low_ma, curr_low_counts, ibus_adjust_ma;
};

/* DT constants of one protocol instance. Temperatures in 0.1 C. */
struct fc_profile {
	struct fc_tuning tune;
	int target_vbus_mv;
	int curr_max_ma;	/* input side */
	int start_curr_ma;
	int timeout_s;
	int ibat_over_ma;
	int full_cool_sw_mv;
	int full_normal_sw_mv;
	int full_normal_hw_mv;
	int full_warm_mv;
	int full_recheck_temp;
	int high_temp;		/* no new session at or above */
	int soc_low, soc_high;
	int second_on_ma;	/* second pump open/close thresholds, 0: single */
	int second_off_ma;
	struct fc_region_cfg region;
	struct fc_cgcl cgcl;
	struct fc_lcf lcf;
	struct fc_curve *curve;
	/* VOOC */
	int full_1time_mv[FC_REGION_COUNT];
	int full_ntime_mv[FC_REGION_COUNT];
	int pump_max_ma;
	int vbus_low_mv, vbus_high_mv;
	u8 circuit[3];			/* SuperVOOC, VOOC 3.0, VOOC 2.0 */
	u8 battery_curve[6];
	/* VOOC 2.0/3.0 run the pumps 1:1 on their own curve */
	struct fc_curve *bypass_curve;
	u8 bypass_battery_curve[6];
	int bypass_curr_max_ma;
};

struct fc_sample {
	bool usb_online;
	int usb_type;
	bool oem_type_valid;
	u32 oem_type, oem_subtype, adsp_generation;
	int usb_mv, usb_ma;
	int soc;
	int batt_temp;		/* gauge, 0.1 C */
	int temp;		/* strategy temperature, 0.1 C */
	int vbat_mv;
	int ibat_ma;		/* positive while charging */
	int conn_batt_mc, conn_usb_mc;	/* INT_MIN when absent */
	struct sc8547d_sample pump[2];
	bool pump_valid[2];
	u64 now_ms;
};

struct fc_protocol_ops {
	const char *name;
	enum fc_proto id;
	bool adapter_regulated;	/* the adapter sets the current (VOOC) */
	bool needs_dpdm;	/* the pump PHY must own D+/D- */
	/* Does this adapter speak the protocol? 0 yes, -ENODEV no, -EAGAIN later. */
	int (*probe)(struct fc_instance *inst);
	/* Ask the adapter for a voltage and current (pump-feedback protocols). */
	int (*request)(struct fc_instance *inst, int mv, int ma);
	/* Adapter-reported output, when the protocol has it. */
	int (*source_info)(struct fc_instance *inst, int *mv, int *ma);
	/* Adapter-regulated protocols: the current the adapter should supply. */
	void (*set_current)(struct fc_instance *inst, int ma);
	/* Adapters that deliver one fixed current: that current, else 0. */
	int (*fixed_current)(struct fc_instance *inst);
	/* Called each monitor tick while running; may return an error to stop. */
	int (*tick)(struct fc_instance *inst, const struct fc_sample *s);
	/* Release the adapter and the PHY. Must be idempotent. */
	void (*stop)(struct fc_instance *inst);
	void (*irq)(void *data);
};

struct fc_instance {
	struct fastchg *fc;
	const struct fc_protocol_ops *ops;
	struct fc_profile prof;
	struct fc_cgcl_row *cgcl_rows;
	struct fc_lcf_row *lcf_rows;
	void *priv;
	/* Session state, reset at every start. */
	struct fc_region_state region;
	struct fc_cgcl_state cgcl;
	struct fc_lcf_state lcf;
	struct fc_curve_state curve;
	bool curve_valid;
	int adapter_max_ma;
	bool bypass;			/* pumps run 1:1 for this adapter */
	int vbus_cap_mv;		/* user ceiling on the adapter voltage, 0: none */
	int request_mv, request_ma;	/* last request to the adapter */
	int target_mv, target_ma;	/* where the loop is heading */
};

/* Services the core offers to protocol instances. */
struct device *fc_dev(struct fastchg *fc);
struct power_supply *fc_usb_supply(struct fastchg *fc);
struct sc8547d *fc_main_pump(struct fastchg *fc);
const struct fc_sample *fc_sample(struct fastchg *fc);
int fc_pmic_pps(struct fastchg *fc, int cmd, u32 *values);
void fc_stop_async(struct fastchg *fc, enum fc_stop_reason reason, int err);
void fc_kick(struct fastchg *fc);
bool fc_second_pump_available(struct fastchg *fc);
int fc_pumps_prepare(struct fastchg *fc, enum sc8547d_mode mode);
int fc_pumps_start(struct fastchg *fc);
int fc_pumps_stop(struct fastchg *fc);

/* Protocol instances */
extern const struct fc_protocol_ops fc_pps_ops;
extern const struct fc_protocol_ops fc_ufcs_ops;
extern const struct fc_protocol_ops fc_vooc_ops;
int fc_ufcs_alloc(struct fc_instance *inst);
int fc_vooc_alloc(struct fc_instance *inst);
int fc_vooc_sample(struct fc_instance *inst, const struct fc_sample *s, int *limit_ma);
void fc_vooc_allow(struct fc_instance *inst);
int fc_vooc_suspend_input(struct fastchg *fc);

#endif
