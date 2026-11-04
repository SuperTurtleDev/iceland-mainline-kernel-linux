/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _OPLUS_FC_CORE_H
#define _OPLUS_FC_CORE_H

#include <linux/completion.h>
#include <linux/notifier.h>
#include <linux/pm.h>
#include <linux/power_supply.h>

#include "fastchg.h"

struct iio_channel;
struct mux_state;
struct thermal_zone_device;

struct fastchg {
	struct device *dev;
	struct power_supply *usb, *bat, *psy;
	struct sc8547d *pump[2];
	int pump_count;
	struct mux_state *dpdm;
	struct iio_channel *conn_batt, *conn_usb;
	struct thermal_zone_device *temp_zone;
	int conn_max_mc;
	/* COMM shell temperature selection, thresholds expanded to the full region enum */
	s32 comm_thr[10];
	s32 comm_dyn[10];
	int comm_count;
	int comm_region;
	/* system thermal cooling: battery-side current per state, halved on the input */
	struct thermal_cooling_device *cdev;
	u32 *cooling_ma;
	int cooling_states;
	unsigned long cooling_state;
	int user_icl_ma;		/* power_supply input current limit, 0: none */
	int user_vmax_mv;		/* power_supply voltage max, 0: none */
	int order[FC_PROTO_COUNT];
	int order_count;
	struct fc_instance inst[FC_PROTO_COUNT];

	struct mutex lock;		/* session state */
	struct mutex pump_lock;		/* pump on/off and second pump state */
	struct workqueue_struct *wq;
	struct delayed_work work;
	struct notifier_block psy_nb, pm_nb;
	struct wakeup_source *ws;
	bool ready, enabled, suspended, removing, block_suspend;

	enum fc_state state;
	struct fc_instance *active;
	int probe_index;
	struct fc_sample s;
	u32 adsp_generation, adapter_type, adapter_subtype;
	bool adapter_known;
	u64 plugged_ms, started_ms, running_ms, second_on_ms, hold_until_ms;
	unsigned long stop_pending;	/* async stop requests, by reason */
	int stop_err;
	enum fc_stop_reason last_reason;
	int last_err;
	int retries;
	enum fc_region full_cool_region;
	bool full_cool;
	u32 native_icl_ua;
	bool pmic_suspended;
	bool dpdm_taken;
	bool pump_on[2];
	bool pump_prepared;
	bool second_tried;
	bool second_disabled;
	int second_failures;
	bool probed_phy;
	bool ucp_enabled;
	int second_votes;
	/* leaving fast charging */
	enum fc_stop_reason restore_reason;
	int restore_err;
	int restore_tries;
	bool pumps_off;
	u64 stopped_ms;
	/* pump-feedback loop */
	int step_ma;		/* curve current */
	int region_ma;		/* temperature region or CGCL current */
	int target_ma;
	int ramp_mv;		/* start phase: voltage being requested */
	int start_retry;
	bool start_check;
	u64 last_protect_ms;
	int count_cool_full, count_sw_full, count_hw_full, count_warm_full;
	int count_ibat_high, count_ibat_low, count_btb, count_tfg, count_tbatt;
	int count_curr_over, count_curr_abnormal, count_output_low, count_fault;
	bool request_too_low;
	int time_left_s;
	u64 last_timeout_ms;
	/* sysfs curve reload */
	struct fc_curve *staging;
	int staging_proto;
	int staging_error;
	struct pid *loader;
};

int fc_dt_parse(struct fastchg *fc);
const char *fc_protocol_name(enum fc_proto proto);
void fc_dt_free(struct fastchg *fc);
int fc_curve_install(struct fastchg *fc, int proto, struct fc_curve *curve);

#endif
