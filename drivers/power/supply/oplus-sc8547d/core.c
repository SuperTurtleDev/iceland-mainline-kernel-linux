// SPDX-License-Identifier: GPL-2.0-only
/*
 * OnePlus Iceland fast charging session over two SC8547D charge pumps.
 *
 * Session sequence, protections and their thresholds follow the Oplus v2
 * charger (vendor/oplus/kernel/charger/v2/oplus_chg_pps.c, oplus_chg_ufcs.c,
 * oplus_chg_vooc.c, voocphy/oplus_ap_voocphy.c, hal/oplus_virtual_cp.c),
 * Copyright (C) 2020-2024 Oplus. All rights reserved.
 *
 * All adapters are treated as third-party ones. The strategy temperature
 * follows the stock COMM rule: the shell estimate of the named thermal zone
 * while the battery is warm enough and the two agree, the battery otherwise.
 */
#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/iio/consumer.h>
#include <linux/ktime.h>
#include <linux/limits.h>
#include <linux/module.h>
#include <linux/mux/consumer.h>
#include <linux/of.h>
#include <linux/pid.h>
#include <linux/platform_device.h>
#include <linux/power/qcom-battmgr-pps.h>
#include <linux/rcupdate.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/thermal.h>
#include <linux/vmalloc.h>

#include "core.h"

#define FC_PLUG_SETTLE_MS	2000
#define FC_MONITOR_MS		500
#define FC_PROTECT_MS		1000
#define FC_START_VOL_OVER_MAX	3
#define FC_SESSION_RETRIES	3
#define FC_RESTORE_RETRY_MS	1000
#define FC_RESTORE_RETRIES	10
#define FC_SWITCH_CURVE_WAIT_MS	200
#define FC_COMM_HYST		20
#define FC_COMM_SHELL_MIN	320
#define FC_COMM_SHELL_MAX	600
#define FC_COMM_SHELL_DIFF	150
#define FC_COMM_LITTLE_COLD	1	/* TEMP_REGION_LITTLE_COLD */
#define FC_COMM_WARM		9	/* TEMP_REGION_WARM */
#define FC_CP_RATIO		2
#define FC_USER_MIN_MA		100
#define FC_USER_MIN_MV		5000
#define FC_RESTORE_VBUS_MAX_MV	6000
#define FC_HOLD_NOT_ALLOWED_MS	5000
#define FC_HOLD_SWITCH_MS	100
#define FC_HOLD_ERROR_MS	10000
#define FC_HOLD_RESET_MS	5000
#define FC_RESTORE_WAIT_MS	3000
#define FC_TEARDOWN_STEP_MS	200
#define FC_ADAPTER_RESET_MS	1000
#define FC_UCP_SS_IBUS_MIN_MA	600
#define FC_BTB_CHECK_TRIES	3
#define FC_SECOND_VOTES		3
#define FC_PUMP_FAULT_COUNTS	2

static const char * const fc_state_names[] = {
	"idle", "probing", "starting", "running", "restoring", "hold",
};

static const char * const fc_reason_names[] = {
	"none", "unplug", "not-supported", "not-allowed", "switch-curve",
	"full", "full-cool", "error", "fatal", "disabled",
};

static u64 fc_now_ms(void)
{
	return ktime_to_ms(ktime_get_boottime());
}

static void fc_schedule(struct fastchg *fc, unsigned int delay_ms)
{
	if (READ_ONCE(fc->ready) && !READ_ONCE(fc->removing))
		mod_delayed_work(fc->wq, &fc->work, msecs_to_jiffies(delay_ms));
}

/* Services for the protocol instances */

struct device *fc_dev(struct fastchg *fc)
{
	return fc->dev;
}

struct power_supply *fc_usb_supply(struct fastchg *fc)
{
	return fc->usb;
}

struct sc8547d *fc_main_pump(struct fastchg *fc)
{
	return fc->pump[0];
}

const struct fc_sample *fc_sample(struct fastchg *fc)
{
	return &fc->s;
}

int fc_pmic_pps(struct fastchg *fc, int cmd, u32 *values)
{
	return qcom_battmgr_pps_request(fc->usb, cmd, values);
}

/* A protocol asks the session to stop; the worker does the work. */
void fc_stop_async(struct fastchg *fc, enum fc_stop_reason reason, int err)
{
	set_bit(reason, &fc->stop_pending);
	if (err)
		WRITE_ONCE(fc->stop_err, err);
	fc_schedule(fc, 0);
}

void fc_kick(struct fastchg *fc)
{
	fc_schedule(fc, 0);
}

/* Pumps */

int fc_pumps_prepare(struct fastchg *fc, enum sc8547d_mode mode)
{
	int i, ret = 0;

	mutex_lock(&fc->pump_lock);
	for (i = 0; i < fc->pump_count && !ret; i++)
		ret = sc8547d_cp_prepare(fc->pump[i], mode);
	/* Reading the flags clears what piled up while the pumps were idle. */
	for (i = 0; i < fc->pump_count && !ret; i++) {
		struct sc8547d_sample s;

		ret = sc8547d_sample(fc->pump[i], &s);
	}
	fc->pump_prepared = !ret;
	fc->ucp_enabled = false;
	mutex_unlock(&fc->pump_lock);
	return ret;
}

static int fc_pump_on(struct fastchg *fc, int i, bool on)
{
	int ret;

	ret = sc8547d_cp_start(fc->pump[i], on);
	if (!ret)
		fc->pump_on[i] = on;
	return ret;
}

/* Start the main pump; the second one joins by current later. */
int fc_pumps_start(struct fastchg *fc)
{
	int ret;

	mutex_lock(&fc->pump_lock);
	ret = fc->pump_prepared ? fc_pump_on(fc, 0, true) : -EINVAL;
	mutex_unlock(&fc->pump_lock);
	return ret;
}

/*
 * Switch both pumps off and verify they stopped. On failure the software
 * state keeps the pump as running so the caller retries; nothing else may
 * proceed until this succeeds.
 */
int fc_pumps_stop(struct fastchg *fc)
{
	int i, ret = 0, err;
	bool on;

	mutex_lock(&fc->pump_lock);
	for (i = fc->pump_count - 1; i >= 0; i--) {
		err = sc8547d_cp_stop(fc->pump[i]);
		if (!err)
			err = sc8547d_switching(fc->pump[i], &on);
		if (!err && on)
			err = -EIO;
		if (err) {
			dev_crit_ratelimited(fc->dev, "cannot stop pump %d: %d\n", i, err);
			ret = ret ?: err;
			continue;
		}
		fc->pump_on[i] = false;
	}
	if (!ret) {
		fc->pump_prepared = false;
		fc->second_votes = 0;
		fc->second_tried = false;
	}
	mutex_unlock(&fc->pump_lock);
	return ret;
}



static int fc_pumps_sample(struct fastchg *fc, struct fc_sample *s)
{
	int i, ret = 0;

	guard(mutex)(&fc->pump_lock);
	for (i = 0; i < fc->pump_count; i++) {
		s->pump_valid[i] = false;
		if (!fc->pump_prepared)
			continue;
		ret = sc8547d_sample(fc->pump[i], &s->pump[i]);
		if (ret)
			return ret;
		s->pump_valid[i] = true;
	}
	return 0;
}

static int fc_ibus_total(const struct fc_sample *s, int count)
{
	int i, ma = 0;

	for (i = 0; i < count; i++)
		if (s->pump_valid[i] && s->pump[i].switching)
			ma += s->pump[i].ibus_ma;
	return ma;
}

/*
 * Open-by-current from oplus_virtual_cp.c: the second pump opens once the
 * input current reaches the open threshold and closes below the close
 * threshold. When the measured and the targeted current disagree, the
 * decision waits three ticks.
 */
/*
 * Open-by-current from oplus_virtual_cp.c: the second pump opens once the
 * input current reaches the open threshold and closes below the close
 * threshold. When the measured and the targeted current disagree, the
 * decision waits three ticks. A pump that does not carry current after
 * opening is closed; after the configured retries it stays closed for the
 * session and the target is capped to what one pump may carry.
 */
static int fc_pumps_manage(struct fastchg *fc, const struct fc_sample *s, int target_ma)
{
	const struct fc_profile *p = &fc->active->prof;
	const struct fc_tuning *t = &p->tune;
	bool on, want_actual, want_target, want;
	int ibus = fc_ibus_total(s, fc->pump_count), ret = 0;

	if (fc->pump_count < 2 || !p->second_on_ma)
		return 0;
	mutex_lock(&fc->pump_lock);
	on = fc->pump_on[1];
	if (on && s->pump_valid[1] && fc_now_ms() - fc->second_on_ms > t->second_settle_ms &&
	    (!s->pump[1].switching || s->pump[1].ibus_ma < t->second_min_ma) &&
	    s->pump[0].ibus_ma >= p->second_on_ma) {
		fc->second_failures++;
		fc->second_disabled = fc->second_failures >= t->second_retries;
		dev_warn(fc->dev, "second pump not carrying current (%d/%d), closing it%s\n",
			 fc->second_failures, t->second_retries,
			 fc->second_disabled ? " for this session" : "");
		ret = fc_pump_on(fc, 1, false);
		fc->second_votes = 0;
		goto out;
	}
	want_actual = on ? ibus > p->second_off_ma : ibus >= p->second_on_ma;
	want_target = on ? target_ma > p->second_off_ma : target_ma >= p->second_on_ma;
	if (want_actual != want_target && ++fc->second_votes < FC_SECOND_VOTES)
		goto out;
	fc->second_votes = 0;
	want = want_actual && !fc->second_disabled;
	if (want == on)
		goto out;
	ret = fc_pump_on(fc, 1, want);
	if (!ret && want)
		fc->second_on_ms = fc_now_ms();
out:
	mutex_unlock(&fc->pump_lock);
	return ret;
}

bool fc_second_pump_available(struct fastchg *fc)
{
	return fc->pump_count > 1 && !READ_ONCE(fc->second_disabled);
}

/* The current one pump may carry when the second is out of play. */
static int fc_single_pump_cap(struct fastchg *fc, int target_ma)
{
	const struct fc_tuning *t = &fc->active->prof.tune;

	if (fc_second_pump_available(fc))
		return target_ma;
	return t->pump_max_ma ? min(target_ma, t->pump_max_ma) : target_ma;
}

static void fc_pumps_watchdog(struct fastchg *fc)
{
	int i;

	guard(mutex)(&fc->pump_lock);
	if (!fc->pump_prepared)
		return;
	for (i = 0; i < fc->pump_count; i++)
		sc8547d_cp_watchdog_kick(fc->pump[i]);
}

/* Sampling */

static int fc_psy_get(struct power_supply *psy, enum power_supply_property prop, int *v)
{
	union power_supply_propval val;
	int ret;

	ret = power_supply_get_property(psy, prop, &val);
	if (!ret)
		*v = val.intval;
	return ret;
}

static int fc_read_therm(struct iio_channel *ch, int *mc)
{
	int ret;

	*mc = INT_MIN;
	if (!ch)
		return 0;
	ret = iio_read_channel_processed(ch, mc);
	return ret < 0 ? ret : 0;
}

/*
 * oplus_comm_temp_thr_update() over the expanded region enum: a move down by
 * one region below WARM raises that boundary by 2 C, a move up by one region
 * above LITTLE_COLD lowers the boundary just crossed by 2 C.
 */
static void fc_comm_track(struct fastchg *fc, int temp)
{
	int i, now, pre = fc->comm_region;

	if (!fc->comm_count)
		return;
	now = fc->comm_count;
	for (i = 0; i < fc->comm_count; i++) {
		if (temp < fc->comm_dyn[i]) {
			now = i;
			break;
		}
	}
	if (pre < 0 || now == pre) {
		fc->comm_region = now;
		return;
	}
	memcpy(fc->comm_dyn, fc->comm_thr, sizeof(fc->comm_thr));
	if (pre - now == 1 && now < FC_COMM_WARM && pre < fc->comm_count)
		fc->comm_dyn[now] = fc->comm_thr[now] + FC_COMM_HYST;
	else if (now - pre == 1 && now > FC_COMM_LITTLE_COLD && pre < fc->comm_count)
		fc->comm_dyn[pre] = fc->comm_thr[pre] - FC_COMM_HYST;
	fc->comm_region = now;
}

/*
 * oplus_comm_check_shell_temp(): the shell estimate drives the strategy only
 * while the battery is above 32 C and below the highest COMM threshold (the
 * dynamic WARM boundary, 53 C on this board), and the two agree within 15 C.
 */
static int fc_strategy_temp(struct fastchg *fc, int batt_dc, int shell_dc, bool shell_ok)
{
	int temp = batt_dc, upper, warm;

	if (shell_ok && fc->comm_count) {
		upper = min(fc->comm_thr[fc->comm_count - 1], FC_COMM_SHELL_MAX);
		warm = fc->comm_dyn[FC_COMM_WARM];
		if (batt_dc > FC_COMM_SHELL_MIN && batt_dc < upper &&
		    abs(shell_dc - batt_dc) < FC_COMM_SHELL_DIFF && batt_dc <= warm)
			temp = shell_dc;
	}
	fc_comm_track(fc, temp);
	return temp;
}

static int fc_read_sample(struct fastchg *fc, struct fc_sample *s)
{
	int ret, type_ret, online, uv, shell_dc = 0;
	u32 adapter[3];
	bool shell_ok = false;

	s->now_ms = fc_now_ms();
	ret = fc_psy_get(fc->usb, POWER_SUPPLY_PROP_ONLINE, &online);
	if (ret)
		return ret;
	s->usb_online = online == 1;
	if (!s->usb_online) {
		s->usb_mv = s->usb_ma = 0;
		return 0;
	}
	type_ret = fc_psy_get(fc->usb, POWER_SUPPLY_PROP_USB_TYPE, &s->usb_type);
	if (type_ret)
		s->usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
	/* Until the Oplus service answers, retain the existing generic path. */
	ret = fc_pmic_pps(fc, QCOM_BATTMGR_PPS_ADAPTER_INFO, adapter);
	s->oem_type_valid = !ret;
	if (!ret) {
		s->oem_type = adapter[0];
		s->oem_subtype = adapter[1];
		s->adsp_generation = adapter[2];
	} else if (type_ret) {
		return type_ret;
	}
	ret = fc_psy_get(fc->usb, POWER_SUPPLY_PROP_VOLTAGE_NOW, &uv);
	if (ret)
		return ret;
	s->usb_mv = uv / 1000;
	ret = fc_psy_get(fc->usb, POWER_SUPPLY_PROP_CURRENT_NOW, &uv);
	if (ret)
		return ret;
	s->usb_ma = uv / 1000;
	ret = fc_psy_get(fc->bat, POWER_SUPPLY_PROP_CAPACITY, &s->soc);
	if (!ret)
		ret = fc_psy_get(fc->bat, POWER_SUPPLY_PROP_TEMP, &s->batt_temp);
	if (!ret)
		ret = fc_psy_get(fc->bat, POWER_SUPPLY_PROP_VOLTAGE_NOW, &uv);
	if (ret)
		return ret;
	s->vbat_mv = uv / 1000;
	ret = fc_psy_get(fc->bat, POWER_SUPPLY_PROP_CURRENT_NOW, &uv);
	if (ret)
		return ret;
	s->ibat_ma = uv / 1000;
	if (fc->temp_zone) {
		int mc;

		/* An estimator without data yet counts as unavailable, not as an error. */
		if (!thermal_zone_get_temp(fc->temp_zone, &mc)) {
			shell_dc = mc / 100;
			shell_ok = true;
		}
	}
	s->temp = fc_strategy_temp(fc, s->batt_temp, shell_dc, shell_ok);
	ret = fc_read_therm(fc->conn_batt, &s->conn_batt_mc);
	if (!ret)
		ret = fc_read_therm(fc->conn_usb, &s->conn_usb_mc);
	if (ret)
		return ret;
	if (s->soc < 0 || s->soc > 100 || s->batt_temp < -400 || s->batt_temp > 1200 ||
	    s->vbat_mv < 2500 || s->vbat_mv > 5000)
		return -ERANGE;
	return fc_pumps_sample(fc, s);
}

/* PMIC input and D+/D- */

static int fc_pmic_suspend(struct fastchg *fc)
{
	u32 v;
	int ret;

	if (fc->pmic_suspended)
		return 0;
	ret = fc_psy_get(fc->usb, POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &v);
	if (ret)
		return ret;
	if (v < 100000 || v > 5000000)
		return -ERANGE;
	fc->native_icl_ua = v;
	fc->pmic_suspended = true;
	v = 0;
	ret = fc_pmic_pps(fc, QCOM_BATTMGR_PPS_CHARGE_ENABLE, &v);
	if (!ret) {
		v = 1;
		ret = fc_pmic_pps(fc, QCOM_BATTMGR_PPS_INPUT_SUSPEND, &v);
	}
	return ret;
}

/*
 * VOOC calls this from its threaded IRQ, after sending the allow reply.
 * The protocol lock serializes it with stop(); restoration must quiesce
 * that callback before touching the PMIC. Never acquire fc->lock here:
 * the session worker may hold it while waiting for the protocol lock.
 */
int fc_vooc_suspend_input(struct fastchg *fc)
{
	return fc_pmic_suspend(fc);
}

/*
 * Back to the PMIC's own charging: 5 V, charge enabled, the native limit.
 * Each step must succeed before the next; a failure leaves the input
 * suspended and the caller retries the whole sequence.
 */
static int fc_pmic_restore(struct fastchg *fc)
{
	u32 v = 1;
	int ret;

	if (!fc->pmic_suspended)
		return 0;
	ret = fc_pmic_pps(fc, QCOM_BATTMGR_PPS_PDO_5V, NULL);
	if (!ret)
		ret = fc_pmic_pps(fc, QCOM_BATTMGR_PPS_CHARGE_ENABLE, &v);
	if (!ret) {
		v = 0;
		ret = fc_pmic_pps(fc, QCOM_BATTMGR_PPS_INPUT_SUSPEND, &v);
	}
	if (!ret)
		ret = fc_pmic_pps(fc, QCOM_BATTMGR_PPS_ICL, &fc->native_icl_ua);
	if (!ret)
		fc->pmic_suspended = false;
	return ret;
}

static int fc_take_dpdm(struct fastchg *fc)
{
	int ret;

	if (fc->dpdm_taken)
		return 0;
	ret = mux_state_try_select(fc->dpdm);
	if (!ret)
		fc->dpdm_taken = true;
	return ret;
}

static int fc_release_dpdm(struct fastchg *fc)
{
	int ret;

	if (!fc->dpdm_taken)
		return 0;
	ret = mux_state_deselect(fc->dpdm);
	if (ret) {
		dev_err_ratelimited(fc->dev, "cannot release D+/D-: %d\n", ret);
		return ret;
	}
	fc->dpdm_taken = false;
	return 0;
}

/* Admission */

static bool fc_connectors_hot(struct fastchg *fc, const struct fc_sample *s)
{
	return (s->conn_batt_mc != INT_MIN && s->conn_batt_mc >= fc->conn_max_mc) ||
	       (s->conn_usb_mc != INT_MIN && s->conn_usb_mc >= fc->conn_max_mc);
}

/* oplus_pps_charge_allow_check(): the gates for a new session. */
static enum fc_stop_reason fc_allowed(struct fastchg *fc, struct fc_instance *inst,
				      struct fc_sample *s)
{
	const struct fc_profile *p = &inst->prof;
	int tries;

	if (s->temp < p->region.low_temp || s->temp >= p->high_temp)
		return FC_STOP_NOT_ALLOWED;
	if (s->temp > p->region.bound[FC_BOUND_NORMAL_HIGH] - 10 &&
	    (s->soc > p->region.warm_soc || s->vbat_mv > p->region.warm_vol))
		return FC_STOP_NOT_ALLOWED;
	if (s->soc < p->soc_low || s->soc > p->soc_high)
		return FC_STOP_NOT_ALLOWED;
	if (fc->full_cool && fc_region_of(&p->region, s->temp) <= fc->full_cool_region)
		return FC_STOP_FULL_COOL;
	for (tries = 0; tries < FC_BTB_CHECK_TRIES; tries++) {
		if (!fc_connectors_hot(fc, s))
			break;
		usleep_range(10000, 11000);
		if (fc_read_therm(fc->conn_batt, &s->conn_batt_mc) ||
		    fc_read_therm(fc->conn_usb, &s->conn_usb_mc))
			return FC_STOP_ERROR;
	}
	if (tries == FC_BTB_CHECK_TRIES)
		return FC_STOP_NOT_ALLOWED;
	fc->full_cool = false;
	return FC_STOP_NONE;
}

/* Stopping */

static unsigned int fc_hold_for(struct fastchg *fc, enum fc_stop_reason reason)
{
	switch (reason) {
	case FC_STOP_NOT_ALLOWED:
	case FC_STOP_FULL_COOL:
		return FC_HOLD_NOT_ALLOWED_MS;
	case FC_STOP_SWITCH_CURVE:
		return FC_HOLD_SWITCH_MS;
	case FC_STOP_NOT_SUPPORTED:
		return fc->retries < 2 ? FC_HOLD_RESET_MS : 0;
	case FC_STOP_ERROR:
		return fc->retries < FC_SESSION_RETRIES ? FC_HOLD_ERROR_MS : 0;
	default:
		return 0;	/* until replug or re-enable */
	}
}

/* The session is over and the hardware is back to normal: decide what next. */
static void fc_restored(struct fastchg *fc)
{
	enum fc_stop_reason reason = fc->restore_reason;
	unsigned int hold;

	fc->active = NULL;
	fc->restore_tries = 0;
	switch (reason) {
	case FC_STOP_UNPLUG:
		fc->retries = 0;
		fc->full_cool = false;
		fc->state = FC_IDLE;
		fc->hold_until_ms = 0;
		break;
	case FC_STOP_ERROR:
	case FC_STOP_NOT_SUPPORTED:
		fc->retries++;
		fallthrough;
	default:
		hold = fc_hold_for(fc, reason);
		fc->state = FC_HOLD;
		fc->hold_until_ms = hold ? fc_now_ms() + hold : 0;
		if (hold)
			fc_schedule(fc, hold);
		break;
	}
	__pm_relax(fc->ws);
}

/*
 * One attempt at leaving fast charging: quiesce VOOC's IRQ first, verify
 * pumps off, release the protocol and D+/D-, see the adapter near 5 V,
 * the PMIC input restored. A failed step is retried from the worker; the
 * software never reports the hardware restored before it is.
 *
 * Return: 0 when done, -EAGAIN to try again later, -EBUSY to wait for VBUS.
 */
static int fc_restore_step(struct fastchg *fc)
{
	struct fc_instance *inst = fc->active;
	int online = 0, uv = 0, ret;

	if (!fc->pumps_off) {
		/* VOOC's IRQ can prepare/start pumps: quiesce it before teardown. */
		if (inst && inst->ops->adapter_regulated)
			inst->ops->stop(inst);
		ret = fc_pumps_stop(fc);
		if (ret) {
			/*
			 * The pumps did not answer: release the protocol anyway so
			 * the adapter stops supplying, and keep trying the pumps.
			 */
			if (inst && fc->restore_tries)
				inst->ops->stop(inst);
			return -EAGAIN;
		}
		fc->pumps_off = true;
		if (inst)
			inst->ops->stop(inst);
	}
	ret = fc_release_dpdm(fc);
	if (ret)
		return -EAGAIN;
	if (fc->pmic_suspended) {
		/* The adapter drops to 5 V once nothing draws fast charging power. */
		if (!fc_psy_get(fc->usb, POWER_SUPPLY_PROP_ONLINE, &online) && online &&
		    !fc_psy_get(fc->usb, POWER_SUPPLY_PROP_VOLTAGE_NOW, &uv) && uv > 5500000 &&
		    fc_now_ms() - fc->stopped_ms < FC_RESTORE_WAIT_MS)
			return -EBUSY;
		ret = fc_pmic_restore(fc);
		if (ret) {
			dev_err_ratelimited(fc->dev, "cannot restore the PMIC input: %d\n", ret);
			return -EAGAIN;
		}
	}
	/* A requested 5 V is not a measured one. */
	if (!fc_psy_get(fc->usb, POWER_SUPPLY_PROP_ONLINE, &online) && online &&
	    !fc_psy_get(fc->usb, POWER_SUPPLY_PROP_VOLTAGE_NOW, &uv) &&
	    uv > FC_RESTORE_VBUS_MAX_MV * 1000) {
		dev_err_ratelimited(fc->dev, "VBUS still %d mV after the 5 V request\n", uv / 1000);
		return -EAGAIN;
	}
	return 0;
}

/* Enter the restore sequence from a state with no session, e.g. a failed adapter reset. */
static void fc_enter_restoring(struct fastchg *fc, enum fc_stop_reason reason, int err)
{
	fc->state = FC_RESTORING;
	fc->restore_reason = reason;
	fc->restore_err = err;
	fc->restore_tries = 0;
	fc->pumps_off = true;
	fc->stopped_ms = fc_now_ms();
	fc->last_reason = reason;
	fc->last_err = err;
}

static void fc_restore_tick(struct fastchg *fc)
{
	int ret;

	ret = fc_restore_step(fc);
	if (!ret) {
		fc_restored(fc);
		return;
	}
	if (ret == -EBUSY) {
		fc_schedule(fc, 100);
		return;
	}
	if (++fc->restore_tries == FC_RESTORE_RETRIES)
		dev_crit(fc->dev, "fast charging hardware not restored after %d attempts; retrying, no new session\n",
			 fc->restore_tries);
	fc_schedule(fc, FC_RESTORE_RETRY_MS);
}

/*
 * Leave fast charging. The hardware work happens in fc_restore_step(); a
 * second stop while restoring only records the more severe reason.
 */
static void fc_stop(struct fastchg *fc, enum fc_stop_reason reason, int err)
{
	struct fc_instance *inst = fc->active;
	enum fc_state was = fc->state;

	if (was == FC_IDLE || was == FC_HOLD) {
		if (reason == FC_STOP_UNPLUG) {
			fc->state = FC_IDLE;
			fc->hold_until_ms = 0;
			fc->retries = 0;
			fc->full_cool = false;
		}
		return;
	}
	if (was == FC_RESTORING) {
		if (reason == FC_STOP_UNPLUG || reason > fc->restore_reason)
			fc->restore_reason = reason;
		return;
	}
	fc->state = FC_RESTORING;
	fc->stop_pending = 0;
	fc->restore_reason = reason;
	fc->restore_err = err;
	fc->restore_tries = 0;
	fc->pumps_off = false;
	fc->stopped_ms = fc_now_ms();
	fc->last_reason = reason;
	fc->last_err = err;
	switch (reason) {
	case FC_STOP_UNPLUG:
	case FC_STOP_SWITCH_CURVE:
		break;
	case FC_STOP_NOT_SUPPORTED:
		dev_dbg(fc->dev, "%s: no supported protocol (%d)\n",
			inst ? inst->ops->name : "none", err);
		break;
	case FC_STOP_ERROR:
		dev_warn_ratelimited(fc->dev, "%s: stop in %s: error (%d)\n",
				     inst ? inst->ops->name : "none", fc_state_names[was], err);
		break;
	case FC_STOP_FATAL:
		dev_err_ratelimited(fc->dev, "%s: protection stop in %s (%d)\n",
				    inst ? inst->ops->name : "none", fc_state_names[was], err);
		break;
	default:
		dev_info(fc->dev, "%s: stop in %s: %s (%d)\n", inst ? inst->ops->name : "none",
			 fc_state_names[was], fc_reason_names[reason], err);
		break;
	}
	fc_restore_tick(fc);
}

/* Protocol probing */

/*
 * oplus_reset_adapter(): a VOOC or UFCS adapter forgets a failed or ended
 * session once its input is cut for a second. Done before a new probe of
 * the pump PHY protocols; PD adapters are not reset.
 */
static int fc_reset_adapter(struct fastchg *fc)
{
	int ret;

	ret = fc_pmic_suspend(fc);
	if (ret)
		return ret;
	msleep(FC_ADAPTER_RESET_MS);
	ret = fc_pmic_restore(fc);
	if (ret) {
		dev_err(fc->dev, "cannot restore the input after an adapter reset: %d\n", ret);
		return ret;
	}
	msleep(500);
	return 0;
}

static bool fc_reset_wanted(const struct fastchg *fc)
{
	if (!fc->probed_phy)
		return false;
	switch (fc->last_reason) {
	case FC_STOP_NOT_SUPPORTED:
	case FC_STOP_ERROR:
	case FC_STOP_SWITCH_CURVE:
	case FC_STOP_NOT_ALLOWED:
	case FC_STOP_FULL_COOL:
		return true;
	default:
		return false;
	}
}

/* Firmware ABI, not enum power_supply_usb_type (which gained AVS entries). */
enum fc_oplus_usb_type {
	FC_USB_UNKNOWN, FC_USB_SDP, FC_USB_DCP, FC_USB_CDP, FC_USB_ACA,
	FC_USB_C, FC_USB_PD, FC_USB_PD_DRP, FC_USB_PD_PPS, FC_USB_PD_SDP,
	FC_USB_APPLE, FC_USB_QC2, FC_USB_QC3, FC_USB_VOOC, FC_USB_SVOOC,
	FC_USB_UFCS,
};

static int fc_oem_charger_type(const struct fc_sample *s)
{
	int type = s->oem_type;

	/* oplus_chg_8350_get_charger_type(), soccp_support branch. */
	if (type == FC_USB_APPLE)
		type = FC_USB_DCP;
	else if (type > FC_USB_PD_SDP)
		type = FC_USB_UNKNOWN;
	if (type == FC_USB_PD || type == FC_USB_PD_DRP || type == FC_USB_PD_PPS)
		return type;
	switch (s->oem_subtype) {
	case 1:
		return FC_USB_VOOC;
	case 2:
		return FC_USB_SVOOC;
	case 4:
		return FC_USB_QC2;
	default:
		return type;
	}
}

static bool fc_type_matches(const struct fc_protocol_ops *ops, const struct fc_sample *s)
{
	int usb_type = s->usb_type;
	bool pd = usb_type == POWER_SUPPLY_USB_TYPE_PD || usb_type == POWER_SUPPLY_USB_TYPE_PD_PPS ||
		  usb_type == POWER_SUPPLY_USB_TYPE_PD_DRP;

	if (s->oem_type_valid) {
		int type = fc_oem_charger_type(s);

		if (ops->id == FC_PROTO_PPS)
			return type == FC_USB_PD_PPS;
		/* CPA excludes data ports and UNKNOWN. PD-SVOOC needs separate authorization. */
		switch (type) {
		case FC_USB_DCP:
		case FC_USB_ACA:
		case FC_USB_C:
		case FC_USB_QC2:
		case FC_USB_QC3:
		case FC_USB_VOOC:
		case FC_USB_SVOOC:
		case FC_USB_UFCS:
			return true;
		default:
			return false;
		}
	}

	/*
	 * Iceland's standard battmgr property can remain SDP on a PPS adapter.
	 * The PPS probe reads the authoritative Oplus type and APDOs without
	 * changing the adapter, so let it decide even when BC1.2 says SDP.
	 */
	if (ops->id == FC_PROTO_PPS)
		return true;
	/* VOOC and UFCS adapters present as dedicated chargers. */
	return !pd && usb_type != POWER_SUPPLY_USB_TYPE_SDP && usb_type != POWER_SUPPLY_USB_TYPE_CDP;
}

static void fc_session_reset(struct fc_instance *inst)
{
	inst->request_mv = inst->request_ma = 0;
	inst->target_mv = inst->target_ma = 0;
	inst->curve_valid = false;
	inst->adapter_max_ma = 0;
	inst->bypass = false;
}

static int fc_probe_protocols(struct fastchg *fc)
{
	int i, ret = -ENODEV;

	for (i = 0; i < fc->order_count; i++) {
		struct fc_instance *inst = &fc->inst[fc->order[i]];

		if (!fc_type_matches(inst->ops, &fc->s))
			continue;
		dev_dbg(fc->dev, "probing %s (type source=%s, usb=%d, oem=%u/%u)\n",
			 inst->ops->name, fc->s.oem_type_valid ? "adsp" : "generic",
			 fc->s.usb_type, fc->s.oem_type, fc->s.oem_subtype);
		fc_session_reset(inst);
		if (inst->ops->needs_dpdm) {
			fc->probed_phy = true;
			ret = fc_take_dpdm(fc);
			if (ret) {
				dev_dbg(fc->dev, "cannot take D+/D-: %d\n", ret);
				continue;
			}
		}
		fc->active = inst;
		fc->state = FC_PROBING;
		if (inst->ops->adapter_regulated) {
			enum fc_stop_reason reason = fc_allowed(fc, inst, &fc->s);

			/* OEM init_vooc checks real_allow before enabling the PHY. */
			if (reason != FC_STOP_NONE) {
				fc_stop(fc, reason, 0);
				return -ECANCELED;
			}
		}
		mutex_unlock(&fc->lock);
		ret = inst->ops->probe(inst);
		mutex_lock(&fc->lock);
		if (!ret && fc->state == FC_PROBING)
			return 0;
		inst->ops->stop(inst);
		fc->active = NULL;
		if (inst->ops->needs_dpdm && fc_release_dpdm(fc))
			return -EIO;
		if (fc->state != FC_PROBING)
			return -ECANCELED;
		if (ret != -ENODEV)
			dev_dbg(fc->dev, "%s: no session: %d\n", inst->ops->name, ret);
	}
	return -ENODEV;
}

/* Session start */

static void fc_counters_reset(struct fastchg *fc)
{
	fc->count_cool_full = fc->count_sw_full = fc->count_hw_full = fc->count_warm_full = 0;
	fc->count_ibat_high = fc->count_ibat_low = fc->count_btb = fc->count_tfg = 0;
	fc->count_tbatt = fc->count_curr_over = fc->count_curr_abnormal = 0;
	fc->count_output_low = fc->count_fault = 0;
	fc->second_failures = 0;
	fc->second_disabled = false;
	fc->request_too_low = false;
	fc->start_retry = 0;
	fc->start_check = false;
	fc->ramp_mv = 0;
	fc->last_protect_ms = 0;
}

static int fc_temp_lo_of(const struct fc_profile *p, enum fc_region region)
{
	return region == FC_REGION_LITTLE_COLD ? p->region.low_temp : p->region.bound[region - 1];
}

static int fc_curve_begin(struct fastchg *fc, struct fc_instance *inst)
{
	const struct fc_profile *p = &inst->prof;
	int ret;

	if (inst->region.region == FC_REGION_INIT)
		fc_region_init(&p->region, &inst->region, fc->s.temp);
	ret = fc_curve_select(p->curve, &inst->curve, fc->s.soc,
			      fc_temp_lo_of(p, inst->region.region), fc->s.vbat_mv, fc->s.now_ms);
	inst->curve_valid = !ret;
	return ret;
}

/* Initialize policy; VOOC defers hardware handover until its allow reply. */
static int fc_limits_apply(struct fastchg *fc, struct fc_instance *inst, int target_ma);

static int fc_begin(struct fastchg *fc)
{
	struct fc_instance *inst = fc->active;
	const struct fc_profile *p = &inst->prof;
	enum fc_stop_reason reason;
	int ret;

	reason = fc_allowed(fc, inst, &fc->s);
	if (reason != FC_STOP_NONE) {
		fc_stop(fc, reason, 0);
		return -EPERM;
	}
	fc_region_init(&p->region, &inst->region, fc->s.temp);
	fc_cgcl_init(&p->cgcl, &inst->cgcl, fc->s.temp);
	fc_lcf_init(&p->lcf, &inst->lcf, fc->s.temp);
	fc_counters_reset(fc);
	fc->time_left_s = p->timeout_s;
	fc->last_timeout_ms = fc->s.now_ms;
	fc->started_ms = fc->s.now_ms;
	if (!inst->ops->adapter_regulated) {
		ret = fc_pmic_suspend(fc);
		if (!ret)
			ret = fc_pumps_prepare(fc, inst->bypass ? SC8547D_MODE_BYPASS : SC8547D_MODE_2_TO_1);
		if (ret) {
			fc_stop(fc, FC_STOP_ERROR, ret);
			return ret;
		}
	}
	fc->state = FC_STARTING;
	fc->step_ma = p->start_curr_ma;
	fc->region_ma = p->region.normal_curr;
	fc->target_ma = max(fc_limits_apply(fc, inst, p->start_curr_ma), FC_USER_MIN_MA);
	inst->vbus_cap_mv = fc->user_vmax_mv;
	if (fc->user_vmax_mv &&
	    fc->user_vmax_mv < fc->s.vbat_mv * (inst->bypass ? 1 : FC_CP_RATIO) + p->tune.start_delta_mv) {
		dev_info(fc->dev, "%s: user voltage limit %d mV below the pump window\n",
			 inst->ops->name, fc->user_vmax_mv);
		fc_stop(fc, FC_STOP_NOT_ALLOWED, -ERANGE);
		return -ERANGE;
	}
	if (inst->ops->adapter_regulated) {
		int limit = p->curr_max_ma;

		ret = fc_vooc_sample(inst, &fc->s, &limit);
		if (ret) {
			fc_stop(fc, ret == -EPERM ? FC_STOP_NOT_ALLOWED : FC_STOP_ERROR, ret);
			return ret;
		}
		inst->ops->set_current(inst, fc_limits_apply(fc, inst, limit));
		fc_vooc_allow(inst);
		return 0;
	}
	fc->ramp_mv = 5500;
	ret = inst->ops->request(inst, fc->ramp_mv, fc->target_ma);
	if (ret) {
		fc_stop(fc, FC_STOP_ERROR, ret);
		return ret;
	}
	inst->request_mv = fc->ramp_mv;
	inst->request_ma = fc->target_ma;
	return 0;
}

/*
 * oplus_pps_charge_start(): step the adapter voltage up until the pump
 * input sits in the 2:1 start window, start the pump, verify switching.
 * Returns the delay until the next step.
 */
static int fc_start_tick(struct fastchg *fc)
{
	struct fc_instance *inst = fc->active;
	const struct fc_profile *p = &inst->prof;
	const struct fc_tuning *t = &p->tune;
	const struct sc8547d_sample *m = &fc->s.pump[0];
	int target_vbus, cp_vin, step, req, ret;

	if (!fc->s.pump_valid[0])
		return -EIO;
	if (fc->s.now_ms - fc->started_ms > 60000)
		return -ETIMEDOUT;
	target_vbus = (fc->s.vbat_mv * 2 / 100) * 100 + t->start_delta_mv;
	cp_vin = m->vbus_mv;
	if (fc->start_check) {
		fc->start_check = false;
		if (m->switching) {
			fc->state = FC_RUNNING;
			fc->running_ms = fc->s.now_ms;
			fc->start_retry = 0;
			inst->target_mv = fc->ramp_mv;
			ret = fc_curve_begin(fc, inst);
			if (ret)
				return ret;
			dev_info(fc->dev, "%s: pumps on at %d mV, adapter max %d mA\n",
				 inst->ops->name, cp_vin, inst->adapter_max_ma);
			return FC_MONITOR_MS;
		}
		if (++fc->start_retry >= t->start_retries)
			return -EFAULT;
		ret = fc_pumps_stop(fc);
		if (!ret)
			ret = fc_pumps_prepare(fc, inst->bypass ? SC8547D_MODE_BYPASS :
					       SC8547D_MODE_2_TO_1);
		if (ret)
			return ret;
	} else if (cp_vin >= target_vbus && cp_vin <= target_vbus + t->start_window_mv) {
		ret = fc_pumps_start(fc);
		if (ret)
			return ret;
		fc->start_check = true;
		return t->start_step_ms;
	}
	if (abs(cp_vin - target_vbus) >= 2000)
		step = 2000;
	else if (abs(cp_vin - target_vbus) >= 1000)
		step = 1000;
	else if (abs(cp_vin - target_vbus) >= 500)
		step = 500;
	else if (abs(cp_vin - target_vbus) >= 200)
		step = 200;
	else
		step = 100;
	req = fc->ramp_mv + (cp_vin < target_vbus ? step : -step);
	if (req > p->target_vbus_mv) {
		req = p->target_vbus_mv;
		if (++fc->count_output_low > FC_START_VOL_OVER_MAX)
			return -EFAULT;
	}
	ret = inst->ops->request(inst, req, fc->target_ma);
	if (ret)
		return ret;
	fc->ramp_mv = req;
	inst->request_mv = req;
	inst->request_ma = fc->target_ma;
	fc_pumps_watchdog(fc);
	return t->start_step_ms;
}

/* Running: protections */

static bool fc_full_vote(struct fastchg *fc, int *count, int limit)
{
	if (++*count < limit)
		return false;
	*count = 0;
	return true;
}

/* A full in the cold regions only pauses until the battery warms up. */
static enum fc_stop_reason fc_full_reason(struct fastchg *fc, struct fc_instance *inst)
{
	const struct fc_profile *p = &inst->prof;

	if (fc->s.temp <= p->full_recheck_temp) {
		fc->full_cool = true;
		fc->full_cool_region = inst->region.region == FC_REGION_INIT ?
			fc_region_of(&p->region, fc->s.temp) : inst->region.region;
		return FC_STOP_FULL_COOL;
	}
	return FC_STOP_FULL;
}

/* oplus_pps_check_sw_full() with the third-party thresholds. */
static enum fc_stop_reason fc_check_sw_full(struct fastchg *fc, struct fc_instance *inst,
					    const struct fc_curve_row *gear)
{
	const struct fc_profile *p = &inst->prof;
	int temp = fc->s.temp, vbat = fc->s.vbat_mv;

	if (temp < p->region.bound[FC_BOUND_COOL] && vbat > p->full_cool_sw_mv) {
		if (fc_full_vote(fc, &fc->count_cool_full, p->tune.full_cool_counts))
			return fc_full_reason(fc, inst);
	} else {
		fc->count_cool_full = 0;
	}
	if (temp >= p->region.bound[FC_BOUND_COOL] && temp < p->region.bound[FC_BOUND_OVER_HIGH]) {
		if (vbat > p->full_normal_sw_mv && gear && (gear->flags & FC_CURVE_LAST)) {
			if (fc_full_vote(fc, &fc->count_sw_full, p->tune.full_sw_counts))
				return fc_full_reason(fc, inst);
		} else {
			fc->count_sw_full = 0;
		}
		if (vbat > p->full_normal_hw_mv) {
			if (fc_full_vote(fc, &fc->count_hw_full, p->tune.full_hw_counts))
				return fc_full_reason(fc, inst);
		} else {
			fc->count_hw_full = 0;
		}
	} else {
		fc->count_sw_full = fc->count_hw_full = 0;
	}
	if (inst->region.status == FC_TEMP_WARM && vbat > p->full_warm_mv) {
		if (fc_full_vote(fc, &fc->count_warm_full, p->tune.full_warm_counts))
			return FC_STOP_FULL;
	} else {
		fc->count_warm_full = 0;
	}
	return FC_STOP_NONE;
}

/* One second checks: timeout, battery current, temperatures, pump faults. */
static enum fc_stop_reason fc_protect(struct fastchg *fc, struct fc_instance *inst, int *err)
{
	const struct fc_profile *p = &inst->prof;
	const struct fc_tuning *t = &p->tune;
	const struct fc_sample *s = &fc->s;
	int i;

	*err = 0;
	for (i = 0; i < fc->pump_count; i++) {
		if (s->pump_valid[i] && s->pump[i].fault) {
			dev_err(fc->dev, "pump %d fault: %s\n", i,
				sc8547d_fault_name(s->pump[i].fault));
			*err = -EIO;
			return FC_STOP_ERROR;
		}
	}
	if (fc->pump_on[0] && s->pump_valid[0] && !s->pump[0].switching) {
		if (++fc->count_fault >= FC_PUMP_FAULT_COUNTS) {
			*err = -EIO;
			return FC_STOP_ERROR;
		}
	} else {
		fc->count_fault = 0;
	}
	/* Session time */
	fc->time_left_s -= (int)div_u64(s->now_ms - fc->last_timeout_ms, 1000);
	fc->last_timeout_ms += (u64)div_u64(s->now_ms - fc->last_timeout_ms, 1000) * 1000;
	if (fc->time_left_s <= 0)
		return FC_STOP_FULL;
	if (s->now_ms - fc->last_protect_ms < FC_PROTECT_MS)
		return FC_STOP_NONE;
	fc->last_protect_ms = s->now_ms;
	/* Battery current: too much charging, or discharging under the pumps */
	if (s->ibat_ma > p->ibat_over_ma) {
		if (++fc->count_ibat_high >= t->ibat_high_counts) {
			*err = -EOVERFLOW;
			return FC_STOP_FATAL;
		}
	} else {
		fc->count_ibat_high = 0;
	}
	if (s->ibat_ma < -t->ibat_low_ma) {
		if (++fc->count_ibat_low > t->ibat_low_counts) {
			*err = -ENOLINK;
			return FC_STOP_FATAL;
		}
	} else {
		fc->count_ibat_low = 0;
	}
	if (!fc->ucp_enabled && s->ibat_ma / 2 > FC_UCP_SS_IBUS_MIN_MA) {
		fc->ucp_enabled = true;
		for (i = 0; i < fc->pump_count; i++)
			sc8547d_cp_ucp_enable(fc->pump[i], true);
	}
	/* Connector and pump temperatures */
	if (fc_connectors_hot(fc, s) ||
	    (s->pump_valid[0] && s->pump[0].tdie_dc >= fc->conn_max_mc / 100) ||
	    (s->pump_valid[1] && s->pump[1].tdie_dc >= fc->conn_max_mc / 100)) {
		if (++fc->count_btb >= t->conn_counts) {
			*err = -EHOSTDOWN;
			return FC_STOP_FATAL;
		}
	} else {
		fc->count_btb = 0;
	}
	if (s->batt_temp >= t->tfg_dc) {
		if (++fc->count_tfg >= t->tfg_counts) {
			*err = -EHOSTDOWN;
			return FC_STOP_FATAL;
		}
	} else {
		fc->count_tfg = 0;
	}
	return FC_STOP_NONE;
}

/* Limits every protocol shares: profile, adapter, pumps, system cooling, the user. */
static int fc_limits_apply(struct fastchg *fc, struct fc_instance *inst, int target_ma)
{
	const struct fc_profile *p = &inst->prof;

	int ratio = inst->bypass ? 1 : FC_CP_RATIO;

	target_ma = min(target_ma, inst->bypass ? p->bypass_curr_max_ma : p->curr_max_ma);
	if (inst->adapter_max_ma)
		target_ma = min(target_ma, inst->adapter_max_ma);
	target_ma = fc_single_pump_cap(fc, target_ma);
	/* Cooling states run from no limit to the strongest limit. */
	if (fc->cooling_states && fc->cooling_state)
		target_ma = min_t(int, target_ma,
				  fc->cooling_ma[min_t(unsigned long, fc->cooling_state,
						       fc->cooling_states - 1)] / ratio);
	if (fc->user_icl_ma)
		target_ma = min(target_ma, fc->user_icl_ma);
	return target_ma;
}

/* Running: the pump-feedback loop (PPS, UFCS) */

static int fc_run_feedback(struct fastchg *fc, struct fc_instance *inst, int *delay)
{
	const struct fc_profile *p = &inst->prof;
	const struct fc_curve_row *gear;
	enum fc_stop_reason reason;
	bool reselect = false;
	int ret, err, mv, ma, limit;

	/* Temperature region, then the CGCL table on top of it */
	fc->region_ma = fc_region_update(&p->region, &inst->region, fc->s.temp, fc->s.soc,
					 fc->s.vbat_mv, &reselect);
	limit = fc_cgcl_update(&p->cgcl, &inst->cgcl, fc->s.temp);
	if (limit > 0)
		fc->region_ma = limit;
	if (inst->region.status == FC_TEMP_EXIT) {
		fc_stop(fc, FC_STOP_NOT_ALLOWED, 0);
		return -EPERM;
	}
	if (reselect || inst->region.status == FC_TEMP_SWITCH_CURVE) {
		/* oplus_ufcs_check_temp(): come down to a safe current before leaving. */
		if (p->tune.switch_curve_ma && inst->target_ma >= p->tune.switch_curve_ma) {
			if (!inst->ops->request(inst, inst->request_mv, p->tune.switch_curve_ma))
				inst->request_ma = p->tune.switch_curve_ma;
			msleep(FC_SWITCH_CURVE_WAIT_MS);
		}
		fc_stop(fc, FC_STOP_SWITCH_CURVE, 0);
		return -EAGAIN;
	}
	/* Curve gear */
	gear = fc_curve_step(p->curve, &inst->curve, fc->s.vbat_mv, fc->s.now_ms);
	if (!gear) {
		fc_stop(fc, fc_full_reason(fc, inst), 0);
		return -ENODATA;
	}
	fc->step_ma = gear->ma;
	/* Protections */
	reason = fc_protect(fc, inst, &err);
	if (reason == FC_STOP_NONE)
		reason = fc_check_sw_full(fc, inst, gear);
	if (reason == FC_STOP_NONE &&
	    fc_lcf_update(&p->lcf, &inst->lcf, fc->s.temp, fc->s.vbat_mv, fc->s.ibat_ma))
		reason = FC_STOP_FULL;
	if (reason != FC_STOP_NONE) {
		fc_stop(fc, reason, err);
		return -ECANCELED;
	}
	/* Current target: the smallest of every limit */
	fc->target_ma = min(fc->step_ma, fc->region_ma);
	fc->target_ma = fc_limits_apply(fc, inst, fc->target_ma);
	ret = fc_pumps_manage(fc, &fc->s, fc->target_ma);
	if (ret) {
		fc_stop(fc, FC_STOP_ERROR, ret);
		return ret;
	}
	/* A pump closed just now lowers what may be carried from this tick on. */
	fc->target_ma = fc_limits_apply(fc, inst, fc->target_ma);
	inst->target_ma = fc->target_ma;
	/* The protocol decides how to walk voltage and current */
	ret = inst->ops->tick(inst, &fc->s);
	if (ret == -ERANGE) {
		fc_stop(fc, FC_STOP_NOT_ALLOWED, ret);
		return ret;
	}
	if (ret) {
		fc_stop(fc, ret == -EOVERFLOW ? FC_STOP_FATAL : FC_STOP_ERROR, ret);
		return ret;
	}
	mv = inst->target_mv;
	ma = inst->request_ma;
	if (mv != inst->request_mv || inst->target_ma != inst->request_ma) {
		ma = inst->target_ma;
		ret = inst->ops->request(inst, mv, ma);
		if (ret) {
			fc_stop(fc, FC_STOP_ERROR, ret);
			return ret;
		}
		inst->request_mv = mv;
		inst->request_ma = ma;
	}
	fc_pumps_watchdog(fc);
	*delay = FC_MONITOR_MS;
	return 0;
}

/* Running: adapter-regulated loop (VOOC) */

static int fc_run_adapter(struct fastchg *fc, struct fc_instance *inst, int *delay)
{
	const struct fc_profile *p = &inst->prof;
	enum fc_stop_reason reason;
	int ret, err, limit = INT_MAX;

	reason = fc_protect(fc, inst, &err);
	if (reason == FC_STOP_NONE &&
	    fc_lcf_update(&p->lcf, &inst->lcf, fc->s.temp, fc->s.vbat_mv, fc->s.ibat_ma))
		reason = FC_STOP_FULL;
	if (reason != FC_STOP_NONE) {
		fc_stop(fc, reason, err);
		return -ECANCELED;
	}
	ret = fc_vooc_sample(inst, &fc->s, &limit);
	if (ret == -ENODATA) {
		fc_stop(fc, fc_full_reason(fc, inst), 0);
		return ret;
	}
	if (ret == -EAGAIN) {
		fc_stop(fc, FC_STOP_SWITCH_CURVE, 0);
		return ret;
	}
	if (ret == -EPERM) {
		fc_stop(fc, FC_STOP_NOT_ALLOWED, 0);
		return ret;
	}
	if (ret) {
		fc_stop(fc, FC_STOP_ERROR, ret);
		return ret;
	}
	fc->target_ma = fc_limits_apply(fc, inst, limit);
	ret = fc_pumps_manage(fc, &fc->s, fc->target_ma);
	if (ret) {
		fc_stop(fc, FC_STOP_ERROR, ret);
		return ret;
	}
	fc->target_ma = fc_limits_apply(fc, inst, limit);
	/* An adapter with one fixed current cannot be limited below it. */
	if (inst->ops->fixed_current && inst->ops->fixed_current(inst) &&
	    fc->target_ma < inst->ops->fixed_current(inst)) {
		fc_stop(fc, FC_STOP_NOT_ALLOWED, -ERANGE);
		return -ERANGE;
	}
	inst->ops->set_current(inst, fc->target_ma);
	ret = inst->ops->tick(inst, &fc->s);
	if (ret) {
		fc_stop(fc, ret == -EOVERFLOW ? FC_STOP_FATAL : FC_STOP_ERROR, ret);
		return ret;
	}
	fc_pumps_watchdog(fc);
	*delay = FC_MONITOR_MS;
	return 0;
}

/* Worker */

static bool fc_adapter_changed(struct fastchg *fc)
{
	const struct fc_sample *s = &fc->s;
	bool new_service;

	if (!s->oem_type_valid)
		return false;
	new_service = !fc->adapter_known || fc->adsp_generation != s->adsp_generation;
	if (!new_service && fc->adapter_type == s->oem_type &&
	    fc->adapter_subtype == s->oem_subtype)
		return false;
	fc->adapter_known = true;
	fc->adsp_generation = s->adsp_generation;
	fc->adapter_type = s->oem_type;
	fc->adapter_subtype = s->oem_subtype;
	dev_dbg(fc->dev, "ADSP adapter type=%u subtype=%u generation=%u (generic=%d)\n",
		 s->oem_type, s->oem_subtype, s->adsp_generation, s->usb_type);

	/* Re-evaluate an existing cable after ADSP startup or a late type update.
	 * Do not clear full, thermal or fault protection holds on a supply event.
	 */
	if (fc->state == FC_HOLD &&
	    (fc->last_reason == FC_STOP_NOT_SUPPORTED || fc->last_reason == FC_STOP_ERROR)) {
		fc->retries = 0;
		fc->hold_until_ms = 0;
		fc->state = FC_IDLE;
	} else if (new_service && fc->active) {
		/* Restore an old/fallback session before touching the PHY again. */
		fc_stop(fc, FC_STOP_SWITCH_CURVE, 0);
		return true;
	}
	return false;
}

static void fc_handle_pending(struct fastchg *fc)
{
	unsigned long pending = fc->stop_pending;
	int reason;

	if (!pending)
		return;
	/* The most severe reason wins. */
	for (reason = FC_STOP_DISABLED; reason > FC_STOP_NONE; reason--)
		if (test_bit(reason, &pending))
			break;
	fc_stop(fc, reason, fc->stop_err);
	fc->stop_err = 0;
}

static void fc_work(struct work_struct *work)
{
	struct fastchg *fc = container_of(to_delayed_work(work), struct fastchg, work);
	struct fc_instance *inst;
	int ret, delay = 0;

	mutex_lock(&fc->lock);
	if (fc->removing || fc->suspended)
		goto out;
	/* Leaving fast charging needs neither a full sample nor the enable. */
	if (fc->state == FC_RESTORING) {
		fc_restore_tick(fc);
		goto out;
	}
	ret = fc_read_sample(fc, &fc->s);
	if (ret) {
		if (fc->state != FC_IDLE && fc->state != FC_HOLD)
			fc_stop(fc, FC_STOP_ERROR, ret);
		else if (ret == -EAGAIN || ret == -ECONNRESET || ret == -ENOTCONN)
			delay = 0;	/* ADSP still starting: its event comes */
		goto out;
	}
	if (!fc->s.usb_online) {
		fc_stop(fc, FC_STOP_UNPLUG, 0);
		if (fc->state == FC_RESTORING)
			fc_restore_tick(fc);
		fc->plugged_ms = 0;
		goto out;
	}
	if (fc->block_suspend)
		__pm_stay_awake(fc->ws);
	if (!fc->plugged_ms)
		fc->plugged_ms = fc->s.now_ms;
	fc_handle_pending(fc);
	if (!fc->enabled) {
		if (fc->state != FC_IDLE && fc->state != FC_HOLD)
			fc_stop(fc, FC_STOP_DISABLED, 0);
		goto out;
	}
	if (fc_adapter_changed(fc))
		goto out;

	switch (fc->state) {
	case FC_HOLD:
		if (!fc->hold_until_ms || fc->s.now_ms < fc->hold_until_ms) {
			if (fc->hold_until_ms)
				delay = fc->hold_until_ms - fc->s.now_ms;
			goto out;
		}
		fc->state = FC_IDLE;
		if (fc_reset_wanted(fc)) {
			ret = fc_reset_adapter(fc);
			if (ret) {
				fc_enter_restoring(fc, FC_STOP_ERROR, ret);
				fc_restore_tick(fc);
				goto out;
			}
		}
		fallthrough;
	case FC_IDLE:
		if (fc->s.now_ms - fc->plugged_ms < FC_PLUG_SETTLE_MS) {
			delay = FC_PLUG_SETTLE_MS - (fc->s.now_ms - fc->plugged_ms);
			goto out;
		}
		__pm_stay_awake(fc->ws);
		fc->probed_phy = false;
		ret = fc_probe_protocols(fc);
		if (ret) {
			if (fc->state == FC_IDLE)
				fc->state = FC_PROBING;
			fc_stop(fc, FC_STOP_NOT_SUPPORTED, ret);
			goto out;
		}
		dev_dbg(fc->dev, "%s adapter\n", fc->active->ops->name);
		ret = fc_read_sample(fc, &fc->s);
		if (ret || !fc->s.usb_online) {
			fc_stop(fc, ret ? FC_STOP_ERROR : FC_STOP_UNPLUG, ret);
			goto out;
		}
		if (fc_begin(fc))
			goto out;
		delay = fc->active->ops->adapter_regulated ? FC_MONITOR_MS : 30;
		break;
	case FC_STARTING:
		inst = fc->active;
		if (inst->ops->adapter_regulated) {
			ret = fc_run_adapter(fc, inst, &delay);
			if (!ret && fc->pump_on[0] && fc->s.pump_valid[0] && fc->s.pump[0].switching) {
				fc->state = FC_RUNNING;
				fc->running_ms = fc->s.now_ms;
				dev_dbg(fc->dev, "%s: pumps on at %d mV\n", inst->ops->name,
					 fc->s.pump[0].vbus_mv);
			}
			break;
		}
		ret = fc_start_tick(fc);
		if (ret < 0) {
			fc_stop(fc, FC_STOP_ERROR, ret);
			goto out;
		}
		delay = ret;
		break;
	case FC_RUNNING:
		inst = fc->active;
		if (inst->ops->adapter_regulated)
			fc_run_adapter(fc, inst, &delay);
		else
			fc_run_feedback(fc, inst, &delay);
		break;
	default:
		break;
	}
out:
	if (fc->state == FC_HOLD && fc->hold_until_ms && !delay)
		delay = max_t(s64, 1, (s64)fc->hold_until_ms - (s64)fc_now_ms());
	if (delay > 0 && fc->s.usb_online && fc->state != FC_RESTORING)
		fc_schedule(fc, delay);
	if (!fc->s.usb_online || !fc->block_suspend)
		if (fc->state == FC_IDLE || fc->state == FC_HOLD)
			__pm_relax(fc->ws);
	mutex_unlock(&fc->lock);
	power_supply_changed(fc->psy);
}

static int fc_psy_changed(struct notifier_block *nb, unsigned long event, void *data)
{
	struct fastchg *fc = container_of(nb, struct fastchg, psy_nb);

	if (event == PSY_EVENT_PROP_CHANGED && (data == fc->usb || data == fc->bat))
		fc_schedule(fc, 0);
	return NOTIFY_OK;
}

/* Power supply */

static enum power_supply_property fc_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPES,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_MODEL_NAME,
};

static int fc_get_property(struct power_supply *psy, enum power_supply_property prop,
			   union power_supply_propval *val)
{
	struct fastchg *fc = power_supply_get_drvdata(psy);
	bool running;

	guard(mutex)(&fc->lock);
	running = fc->state == FC_RUNNING;
	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = running;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = running ? POWER_SUPPLY_STATUS_CHARGING :
			POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPES:
		val->intval = fc->enabled ? POWER_SUPPLY_CHARGE_TYPE_FAST :
			POWER_SUPPLY_CHARGE_TYPE_STANDARD;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = running && fc->s.pump_valid[0] ? fc->s.pump[0].vbus_mv * 1000 : 0;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = running ? fc_ibus_total(&fc->s, fc->pump_count) * 1000 : 0;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = running ? fc->target_ma * 1000 : 0;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		val->intval = fc->user_icl_ma * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = fc->user_vmax_mv * 1000;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = fc->s.pump_valid[0] ? fc->s.pump[0].tdie_dc : 0;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = fc->active ? fc->active->ops->name : "none";
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/*
 * Userspace limits: the charge type enables or disables fast charging, the
 * input current limit caps the adapter current and the voltage maximum caps
 * the adapter voltage. Zero removes a cap. A running session applies them on
 * its next tick.
 */
static int fc_set_property(struct power_supply *psy, enum power_supply_property prop,
			   const union power_supply_propval *val)
{
	struct fastchg *fc = power_supply_get_drvdata(psy);

	mutex_lock(&fc->lock);
	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGE_TYPES:
		if (val->intval != POWER_SUPPLY_CHARGE_TYPE_STANDARD &&
		    val->intval != POWER_SUPPLY_CHARGE_TYPE_FAST) {
			mutex_unlock(&fc->lock);
			return -EINVAL;
		}
		fc->enabled = val->intval == POWER_SUPPLY_CHARGE_TYPE_FAST;
		if (fc->enabled && fc->state == FC_HOLD && fc->last_reason == FC_STOP_DISABLED) {
			fc->state = FC_IDLE;
			fc->hold_until_ms = 0;
		}
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		/* 0 removes the cap; a cap is a whole mA of at least the protocol minimum. */
		if (val->intval < 0 || val->intval > 20000000 || val->intval % 1000 ||
		    (val->intval && val->intval < FC_USER_MIN_MA * 1000)) {
			mutex_unlock(&fc->lock);
			return -EINVAL;
		}
		fc->user_icl_ma = val->intval / 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		if (val->intval < 0 || val->intval > 21000000 || val->intval % 1000 ||
		    (val->intval && val->intval < FC_USER_MIN_MV * 1000)) {
			mutex_unlock(&fc->lock);
			return -EINVAL;
		}
		fc->user_vmax_mv = val->intval / 1000;
		if (fc->active)
			fc->active->vbus_cap_mv = fc->user_vmax_mv;
		break;
	default:
		mutex_unlock(&fc->lock);
		return -EINVAL;
	}
	mutex_unlock(&fc->lock);
	fc_schedule(fc, 0);
	return 0;
}

static int fc_property_is_writeable(struct power_supply *psy, enum power_supply_property prop)
{
	return prop == POWER_SUPPLY_PROP_CHARGE_TYPES ||
	       prop == POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT ||
	       prop == POWER_SUPPLY_PROP_VOLTAGE_MAX;
}

/* Thermal cooling device: the stock cool_down levels, battery current per state. */
static int fc_cdev_get_max_state(struct thermal_cooling_device *cdev, unsigned long *state)
{
	struct fastchg *fc = cdev->devdata;

	*state = fc->cooling_states - 1;
	return 0;
}

static int fc_cdev_get_cur_state(struct thermal_cooling_device *cdev, unsigned long *state)
{
	struct fastchg *fc = cdev->devdata;

	*state = READ_ONCE(fc->cooling_state);
	return 0;
}

static int fc_cdev_set_cur_state(struct thermal_cooling_device *cdev, unsigned long state)
{
	struct fastchg *fc = cdev->devdata;

	if (state >= fc->cooling_states)
		return -EINVAL;
	mutex_lock(&fc->lock);
	fc->cooling_state = state;
	mutex_unlock(&fc->lock);
	fc_schedule(fc, 0);
	return 0;
}

static const struct thermal_cooling_device_ops fc_cdev_ops = {
	.get_max_state = fc_cdev_get_max_state,
	.get_cur_state = fc_cdev_get_cur_state,
	.set_cur_state = fc_cdev_set_cur_state,
};

/* sysfs: state, probe priority and the transactional curve reload */

static ssize_t state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fastchg *fc = power_supply_get_drvdata(dev_get_drvdata(dev));
	const struct fc_instance *inst;

	guard(mutex)(&fc->lock);
	inst = fc->active;
	return sysfs_emit(buf, "%s protocol=%s last=%s(%d) target=%dmV/%dmA step=%d region=%d ibus=%d vbus=%d temp=%d batt=%d cooling=%lu user=%dmA/%dmV second=%s retries=%d restore_tries=%d type_source=%s usb_type=%d oem_type=%u oem_subtype=%u adsp_generation=%u\n",
			  fc_state_names[fc->state], inst ? inst->ops->name : "none",
			  fc_reason_names[fc->last_reason], fc->last_err,
			  inst ? inst->request_mv : 0, inst ? inst->request_ma : 0,
			  fc->step_ma, fc->region_ma, fc_ibus_total(&fc->s, fc->pump_count),
			  fc->s.pump_valid[0] ? fc->s.pump[0].vbus_mv : 0, fc->s.temp, fc->s.batt_temp,
			  fc->cooling_state, fc->user_icl_ma, fc->user_vmax_mv,
			  fc->second_disabled ? "disabled" : fc->pump_on[1] ? "on" : "off",
			  fc->retries, fc->restore_tries,
			  fc->s.oem_type_valid ? "adsp" : "generic", fc->s.usb_type,
			  fc->s.oem_type, fc->s.oem_subtype, fc->s.adsp_generation);
}
static DEVICE_ATTR_RO(state);

static ssize_t priority_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fastchg *fc = power_supply_get_drvdata(dev_get_drvdata(dev));
	int i, len = 0;

	guard(mutex)(&fc->lock);
	for (i = 0; i < fc->order_count; i++)
		len += sysfs_emit_at(buf, len, "%s%s", i ? " " : "",
				     fc_protocol_name(fc->order[i]));
	return len + sysfs_emit_at(buf, len, "\n");
}

static ssize_t priority_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct fastchg *fc = power_supply_get_drvdata(dev_get_drvdata(dev));
	int order[FC_PROTO_COUNT], n = 0, i;
	unsigned long seen = 0;
	char *copy, *cursor, *token;
	ssize_t ret = -EINVAL;

	if (!count || count >= PAGE_SIZE || memchr(buf, '\0', count))
		return -EINVAL;
	copy = kstrndup(buf, count, GFP_KERNEL);
	if (!copy)
		return -ENOMEM;

	mutex_lock(&fc->lock);
	/* probe() drops this lock while waiting for the adapter. */
	if (!fc->ready || fc->removing || fc->state == FC_PROBING) {
		ret = -EBUSY;
		goto out;
	}
	cursor = copy;
	while ((token = strsep(&cursor, " \t\r\n"))) {
		if (!*token)
			continue;
		if (n == fc->order_count)
			goto out;
		for (i = 0; i < FC_PROTO_COUNT; i++)
			if (!strcmp(token, fc_protocol_name(i)))
				break;
		if (i == FC_PROTO_COUNT || !fc->inst[i].ops || (seen & BIT(i)))
			goto out;
		seen |= BIT(i);
		order[n++] = i;
	}
	if (n != fc->order_count)
		goto out;
	/* No kick: a new priority applies only to the next complete probe pass. */
	memcpy(fc->order, order, n * sizeof(*order));
	ret = count;
out:
	mutex_unlock(&fc->lock);
	kfree(copy);
	return ret;
}
static DEVICE_ATTR_RW(priority);

static ssize_t block_suspend_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fastchg *fc = power_supply_get_drvdata(dev_get_drvdata(dev));

	return sysfs_emit(buf, "%u\n", READ_ONCE(fc->block_suspend));
}

static ssize_t block_suspend_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct fastchg *fc = power_supply_get_drvdata(dev_get_drvdata(dev));
	bool block;
	int ret;

	ret = kstrtobool(buf, &block);
	if (ret)
		return ret;
	mutex_lock(&fc->lock);
	fc->block_suspend = block;
	mutex_unlock(&fc->lock);
	fc_schedule(fc, 0);
	return count;
}
static DEVICE_ATTR_RW(block_suspend);

static void fc_discard_staging(struct fastchg *fc)
{
	kvfree(fc->staging);
	fc->staging = NULL;
	put_pid(fc->loader);
	fc->loader = NULL;
	fc->staging_error = 0;
}

static int fc_parse_row(const char *buf, struct fc_curve_row *r)
{
	char *copy, *cursor, *token;
	s32 values[10];
	unsigned int n = 0;
	int ret = 0;

	copy = kstrdup(buf, GFP_KERNEL);
	if (!copy)
		return -ENOMEM;
	cursor = copy;
	while ((token = strsep(&cursor, " \t\n"))) {
		if (!*token)
			continue;
		if (n == ARRAY_SIZE(values)) {
			ret = -EINVAL;
			break;
		}
		ret = kstrtos32(token, 10, &values[n++]);
		if (ret)
			break;
	}
	if (!ret && n != ARRAY_SIZE(values))
		ret = -EINVAL;
	if (!ret)
		memcpy(r, values, sizeof(values));
	kfree(copy);
	return ret;
}

/*
 * One writer loads a complete curve for one protocol: "begin <protocol>",
 * one row per write, then "commit" or "abort". Protocols are pps, ufcs, vooc
 * and vooc-bypass. The new curve replaces the old one atomically; a commit
 * while that protocol runs a session is refused with EBUSY. Disable fast
 * charging first to reload the active protocol.
 */
static ssize_t curve_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct fastchg *fc = power_supply_get_drvdata(dev_get_drvdata(dev));
	static const char * const names[FC_PROTO_COUNT + 1] = {
		[FC_PROTO_VOOC] = "vooc", [FC_PROTO_UFCS] = "ufcs", [FC_PROTO_PPS] = "pps",
		[FC_PROTO_COUNT] = "vooc-bypass",
	};
	struct fc_curve_row r;
	bool alive;
	int ret = 0, i;

	if (!count || count >= PAGE_SIZE || memchr(buf, '\0', count))
		return -EINVAL;
	guard(mutex)(&fc->lock);
	if (!fc->ready || fc->removing)
		return -EBUSY;
	if (!strncmp(buf, "begin ", 6)) {
		for (i = 0; i <= FC_PROTO_COUNT; i++)
			if (sysfs_streq(buf + 6, names[i]))
				break;
		if (i > FC_PROTO_COUNT || !fc->inst[i == FC_PROTO_COUNT ? FC_PROTO_VOOC : i].ops)
			return -EINVAL;
		rcu_read_lock();
		alive = fc->loader && pid_has_task(fc->loader, PIDTYPE_TGID);
		rcu_read_unlock();
		if (alive && fc->loader != task_tgid(current))
			return -EBUSY;
		fc_discard_staging(fc);
		fc->staging = kvzalloc(struct_size(fc->staging, rows, FC_CURVE_MAX_ROWS), GFP_KERNEL);
		if (!fc->staging)
			return -ENOMEM;
		fc->staging_proto = i;
		fc->loader = get_pid(task_tgid(current));
		return count;
	}
	if (!fc->staging || fc->loader != task_tgid(current))
		return -EPERM;
	if (sysfs_streq(buf, "abort")) {
		fc_discard_staging(fc);
		return count;
	}
	if (fc->staging_error)
		return fc->staging_error;
	if (sysfs_streq(buf, "commit")) {
		ret = fc_curve_install(fc, fc->staging_proto, fc->staging);
		if (!ret)
			fc->staging = NULL;
		fc_discard_staging(fc);
		return ret ?: count;
	}
	ret = fc_parse_row(buf, &r);
	if (!ret) {
		if (fc->staging->count == FC_CURVE_MAX_ROWS)
			ret = -E2BIG;
		else
			fc->staging->rows[fc->staging->count++] = r;
	}
	fc->staging_error = ret;
	return ret ?: count;
}
static DEVICE_ATTR_WO(curve);

static struct attribute *fc_attrs[] = {
	&dev_attr_state.attr,
	&dev_attr_priority.attr,
	&dev_attr_block_suspend.attr,
	&dev_attr_curve.attr,
	NULL,
};
ATTRIBUTE_GROUPS(fc);

static const struct power_supply_desc fc_psy_desc = {
	.name = "oplus-fastchg",
	.type = POWER_SUPPLY_TYPE_USB,
	.usb_types = BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN),
	.charge_types = BIT(POWER_SUPPLY_CHARGE_TYPE_STANDARD) | BIT(POWER_SUPPLY_CHARGE_TYPE_FAST),
	.properties = fc_props,
	.num_properties = ARRAY_SIZE(fc_props),
	.get_property = fc_get_property,
	.set_property = fc_set_property,
	.property_is_writeable = fc_property_is_writeable,
};

/*
 * System sleep. By default USB power blocks suspend, as a frozen worker
 * cannot run the thermal checks. When sleep is allowed, the session leaves
 * fast charging and restores the PMIC input before devices suspend, and a
 * new session is probed after resume.
 */
static int fc_pm_event(struct notifier_block *nb, unsigned long event, void *data)
{
	struct fastchg *fc = container_of(nb, struct fastchg, pm_nb);
	int online, i;

	switch (event) {
	case PM_SUSPEND_PREPARE:
	case PM_HIBERNATION_PREPARE:
		mutex_lock(&fc->lock);
		if (fc->block_suspend && !fc_psy_get(fc->usb, POWER_SUPPLY_PROP_ONLINE, &online) &&
		    online) {
			mutex_unlock(&fc->lock);
			return notifier_from_errno(-EBUSY);
		}
		fc->suspended = true;
		fc_stop(fc, FC_STOP_DISABLED, 0);
		/* Devices suspend next: the hardware must be restored before. */
		for (i = 0; fc->state == FC_RESTORING && i < 50; i++) {
			if (fc_restore_step(fc) == 0) {
				fc_restored(fc);
				break;
			}
			msleep(100);
		}
		if (fc->state == FC_RESTORING) {
			fc->suspended = false;
			mutex_unlock(&fc->lock);
			fc_schedule(fc, 0);
			dev_err(fc->dev, "cannot leave fast charging before suspend\n");
			return notifier_from_errno(-EBUSY);
		}
		if (fc->state == FC_HOLD) {
			fc->state = FC_IDLE;
			fc->hold_until_ms = 0;
		}
		mutex_unlock(&fc->lock);
		cancel_delayed_work_sync(&fc->work);
		return NOTIFY_OK;
	case PM_POST_SUSPEND:
	case PM_POST_HIBERNATION:
		mutex_lock(&fc->lock);
		fc->suspended = false;
		fc->plugged_ms = 0;
		mutex_unlock(&fc->lock);
		fc_schedule(fc, 0);
		return NOTIFY_OK;
	default:
		return NOTIFY_DONE;
	}
}

/* Probe */

static void fc_put_supply(void *data)
{
	power_supply_put(data);
}

static void fc_unregister_wakeup(void *data)
{
	struct wakeup_source *ws = data;

	__pm_relax(ws);
	wakeup_source_unregister(ws);
}

static int fc_get_pumps(struct fastchg *fc)
{
	struct device_node *np;
	int i, count;

	count = of_count_phandle_with_args(fc->dev->of_node, "southchip,charge-pumps", NULL);
	if (count < 1 || count > 2)
		return dev_err_probe(fc->dev, -EINVAL, "one or two charge pumps expected\n");
	for (i = 0; i < count; i++) {
		np = of_parse_phandle(fc->dev->of_node, "southchip,charge-pumps", i);
		if (!np)
			return -EINVAL;
		fc->pump[i] = sc8547d_get(fc->dev, of_fwnode_handle(np));
		of_node_put(np);
		if (IS_ERR(fc->pump[i]))
			return dev_err_probe(fc->dev, PTR_ERR(fc->pump[i]),
					     "cannot get charge pump %d\n", i);
	}
	fc->pump_count = count;
	return 0;
}

/* A thermistor channel is optional: without it the connector is not watched. */
static struct iio_channel *fc_get_therm(struct device *dev, const char *name)
{
	struct iio_channel *ch;

	if (device_property_match_string(dev, "io-channel-names", name) < 0)
		return NULL;
	ch = devm_iio_channel_get(dev, name);
	return ch;
}

static int fc_get_inputs(struct fastchg *fc)
{
	struct device *dev = fc->dev;
	const char *zone;
	struct device_node *node;
	struct device *supplier;

	fc->dpdm = devm_mux_state_get(dev, NULL);
	if (IS_ERR(fc->dpdm))
		return dev_err_probe(dev, PTR_ERR(fc->dpdm), "cannot get the D+/D- mux\n");
	fc->conn_batt = fc_get_therm(dev, "battery-connector");
	if (IS_ERR(fc->conn_batt))
		return dev_err_probe(dev, PTR_ERR(fc->conn_batt), "battery connector thermistor\n");
	fc->conn_usb = fc_get_therm(dev, "usb-connector");
	if (IS_ERR(fc->conn_usb))
		return dev_err_probe(dev, PTR_ERR(fc->conn_usb), "USB connector thermistor\n");
	fc->conn_max_mc = 80000;
	device_property_read_u32(dev, "oplus,connector-maximum-millicelsius", &fc->conn_max_mc);
	if (!device_property_read_string(dev, "oplus,temperature-zone", &zone)) {
		fc->temp_zone = thermal_zone_get_zone_by_name(zone);
		if (IS_ERR(fc->temp_zone))
			return dev_err_probe(dev, -EPROBE_DEFER, "thermal zone %s\n", zone);
	}
	fc->usb = power_supply_get_by_name("qcom-battmgr-usb");
	if (!fc->usb)
		return -EPROBE_DEFER;
	if (devm_add_action_or_reset(dev, fc_put_supply, fc->usb))
		return -ENOMEM;
	fc->bat = power_supply_get_by_name("qcom-battmgr-bat");
	if (!fc->bat)
		return -EPROBE_DEFER;
	if (devm_add_action_or_reset(dev, fc_put_supply, fc->bat))
		return -ENOMEM;
	/* The supplies must come from the described GLINK, not any battery. */
	supplier = fc->usb->dev.parent;
	node = of_parse_phandle(dev->of_node, "oplus,pmic-glink", 0);
	if (!node || !supplier || !supplier->parent || supplier->parent->of_node != node) {
		of_node_put(node);
		return dev_err_probe(dev, -EINVAL, "USB supply is not the described PMIC GLINK\n");
	}
	of_node_put(node);
	if (!device_link_add(dev, supplier, DL_FLAG_AUTOREMOVE_CONSUMER))
		return -ENOMEM;
	return 0;
}

static void fc_teardown(struct fastchg *fc)
{
	int i;

	mutex_lock(&fc->lock);
	fc->removing = true;
	mutex_unlock(&fc->lock);
	unregister_pm_notifier(&fc->pm_nb);
	power_supply_unreg_notifier(&fc->psy_nb);
	cancel_delayed_work_sync(&fc->work);
	mutex_lock(&fc->lock);
	fc_stop(fc, FC_STOP_DISABLED, 0);
	/* Include the VBUS fall window before counting the restore retries. */
	for (i = 0; fc->state == FC_RESTORING &&
	     i < FC_RESTORE_WAIT_MS / FC_TEARDOWN_STEP_MS + FC_RESTORE_RETRIES; i++) {
		if (fc_restore_step(fc) == 0) {
			fc_restored(fc);
			break;
		}
		msleep(FC_TEARDOWN_STEP_MS);
	}
	if (fc->state == FC_RESTORING)
		dev_crit(fc->dev, "fast charging hardware not restored at teardown\n");
	/* Whatever the hardware did, no protocol may be called back after this. */
	for (i = 0; i < FC_PROTO_COUNT; i++)
		if (fc->inst[i].ops)
			fc->inst[i].ops->stop(&fc->inst[i]);
	for (i = 0; i < fc->pump_count; i++)
		sc8547d_set_irq_handler(fc->pump[i], NULL, NULL);
	mutex_unlock(&fc->lock);
}

static int fc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct power_supply_config cfg = {};
	struct fastchg *fc;
	int ret, i;

	fc = devm_kzalloc(dev, sizeof(*fc), GFP_KERNEL);
	if (!fc)
		return -ENOMEM;
	fc->dev = dev;
	fc->enabled = true;
	fc->block_suspend = true;
	mutex_init(&fc->lock);
	mutex_init(&fc->pump_lock);
	platform_set_drvdata(pdev, fc);
	ret = fc_get_pumps(fc);
	if (ret)
		return ret;
	ret = fc_get_inputs(fc);
	if (ret)
		return ret;
	ret = fc_dt_parse(fc);
	if (ret)
		return ret;
	for (i = 0; i < FC_PROTO_COUNT; i++) {
		struct fc_instance *inst = &fc->inst[i];

		if (!inst->ops)
			continue;
		inst->fc = fc;
		if (i == FC_PROTO_UFCS)
			ret = fc_ufcs_alloc(inst);
		else if (i == FC_PROTO_VOOC)
			ret = fc_vooc_alloc(inst);
		if (ret)
			goto free_dt;
	}
	fc->ws = wakeup_source_register(dev, "oplus-fastchg");
	if (IS_ENABLED(CONFIG_PM_SLEEP) && !fc->ws) {
		ret = -ENOMEM;
		goto free_dt;
	}
	ret = devm_add_action_or_reset(dev, fc_unregister_wakeup, fc->ws);
	if (ret)
		goto free_dt;
	fc->wq = alloc_ordered_workqueue("oplus-fastchg", WQ_MEM_RECLAIM);
	if (!fc->wq) {
		ret = -ENOMEM;
		goto free_dt;
	}
	INIT_DELAYED_WORK(&fc->work, fc_work);
	cfg.drv_data = fc;
	cfg.fwnode = dev_fwnode(dev);
	cfg.attr_grp = fc_groups;
	fc->psy = power_supply_register(dev, &fc_psy_desc, &cfg);
	if (IS_ERR(fc->psy)) {
		ret = PTR_ERR(fc->psy);
		goto destroy_wq;
	}
	fc->psy_nb.notifier_call = fc_psy_changed;
	ret = power_supply_reg_notifier(&fc->psy_nb);
	if (ret)
		goto unregister_psy;
	fc->pm_nb.notifier_call = fc_pm_event;
	ret = register_pm_notifier(&fc->pm_nb);
	if (ret)
		goto unregister_notifier;
	if (fc->cooling_states) {
		fc->cdev = devm_thermal_of_cooling_device_register(dev, 0, "oplus-fastchg", fc,
								   &fc_cdev_ops);
		if (IS_ERR(fc->cdev)) {
			ret = PTR_ERR(fc->cdev);
			unregister_pm_notifier(&fc->pm_nb);
			goto unregister_notifier;
		}
	}
	mutex_lock(&fc->lock);
	fc->ready = true;
	mutex_unlock(&fc->lock);
	fc_schedule(fc, 0);
	dev_info(dev, "fast charging ready: %d pump(s), %d protocol(s)\n", fc->pump_count,
		 fc->order_count);
	return 0;
unregister_notifier:
	power_supply_unreg_notifier(&fc->psy_nb);
unregister_psy:
	power_supply_unregister(fc->psy);
destroy_wq:
	destroy_workqueue(fc->wq);
free_dt:
	fc_dt_free(fc);
	return dev_err_probe(dev, ret, "cannot initialize fast charging\n");
}

static void fc_remove(struct platform_device *pdev)
{
	struct fastchg *fc = platform_get_drvdata(pdev);

	fc_teardown(fc);
	power_supply_unregister(fc->psy);
	destroy_workqueue(fc->wq);
	mutex_lock(&fc->lock);
	fc_discard_staging(fc);
	mutex_unlock(&fc->lock);
	fc_dt_free(fc);
}

static void fc_shutdown(struct platform_device *pdev)
{
	fc_teardown(platform_get_drvdata(pdev));
}

static const struct of_device_id fc_of_match[] = {
	{ .compatible = "oplus,sc8547d-fastcharger" },
	{}
};
MODULE_DEVICE_TABLE(of, fc_of_match);

static struct platform_driver fc_driver = {
	.probe = fc_probe,
	.remove = fc_remove,
	.shutdown = fc_shutdown,
	.driver = {
		.name = "oplus-sc8547d-fastcharger",
		.of_match_table = fc_of_match,
	},
};
module_platform_driver(fc_driver);

MODULE_DESCRIPTION("OnePlus SC8547D fast charging (PPS, UFCS, SuperVOOC)");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("IIO_CONSUMER");
