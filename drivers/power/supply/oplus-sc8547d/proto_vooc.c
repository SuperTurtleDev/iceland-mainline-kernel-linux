// SPDX-License-Identifier: GPL-2.0-only
/*
 * SuperVOOC through the main charge pump's VOOC PHY.
 *
 * The adapter regulates its output current and asks the phone every few
 * tens of milliseconds; the interrupt thread answers, the session tick sets
 * the current it may ask for. Session logic follows oplus_ap_voocphy.c and
 * oplus_chg_vooc.c, Copyright (C) 2018-2024 Oplus. All rights reserved.
 * SuperVOOC adapters run the pumps 2:1; VOOC 2.0 and 3.0 adapters run
 * them 1:1 on the bypass curve. A VOOC 2.0 adapter delivers a fixed
 * current and only hears the battery voltage.
 */
#include <linux/completion.h>
#include <linux/jiffies.h>
#include <linux/limits.h>
#include <linux/minmax.h>
#include <linux/random.h>
#include <linux/slab.h>

#include "core.h"
#include "svooc.h"

#define VOOC_FIRST_FRAME_MS	3000
#define VOOC_IDENTIFY_MS	30000
#define VOOC_FRAME_MS		1000
#define VOOC_PERMIT_MS		5000
#define VOOC_GRANT_MS		30000
#define VOOC_START_MS		2000
#define VOOC_ACK_MS		1000
#define VOOC_CHANGE_MS		1000
#define VOOC_IGNORE_MAX		6
#define VOOC_SKIP_MAX		3
#define VOOC_VBUS_OK_MAX	80
#define VOOC_MIN_MA		1000
#define VOOC_FULL_NTIME_COUNTS	5
#define VOOC_TEMP_OVER_COUNTS	2
#define VOOC_TEMP_HYST		10
#define VOOC_TEMP_HYST_HIGH	20
#define VOOC2_SOC_HIGH		75
#define VOOC2_CURR_MA		3000
#define VOOC2_DUAL_CURR_MA	3750

enum vooc_state {
	VOOC_OFF,
	VOOC_IDENTIFY,		/* PHY on, adapter identifying itself */
	VOOC_WAIT_PERMIT,	/* identified; policy/curve exchange pending */
	VOOC_GRANTED,		/* allow sent; adapter raising VBUS */
	VOOC_STARTING,		/* main pump on, first current pending */
	VOOC_RUNNING,
	VOOC_STOPPED,
};

struct vooc {
	struct fc_instance *inst;
	struct sc8547d *cp;
	struct mutex lock;		/* interrupt thread against the session tick */
	struct completion identified;
	struct svooc proto;
	enum vooc_state st;
	int err;
	bool phy_on;
	/* replies */
	int vbus_mv, vout_mv, vbat_cp_mv, ibus_ma, vbat_mv;
	unsigned int fault_seen;
	int limit_ma, applied_ma, pending_ma, target_ma;
	bool change;
	unsigned int frames, ignored, skipped, vbus_ok_count;
	u64 last_rx_ms, ack_deadline_ms, change_deadline_ms, granted_ms, started_ms;
	u64 permit_wait_ms;
	u8 last_cmd;
	int ratio;			/* 2 for SuperVOOC, 1 for VOOC 2.0/3.0 */
	bool vooc2_asked_again;
	/* strategy */
	enum fc_region region;
	s32 eff[FC_BOUND_COUNT];
	int change_count;
	bool exit, switch_curve;
	int current_max_ma;
	int ntime_count;
	bool curve_ready;
};

static void vooc_fail_locked(struct vooc *v, int err)
{
	if (v->st == VOOC_STOPPED)
		return;
	v->st = VOOC_STOPPED;
	v->err = err;
	v->phy_on = false;
	sc8547d_vooc_disable(v->cp);
	complete(&v->identified);
	fc_stop_async(v->inst->fc, err == -EOPNOTSUPP ? FC_STOP_NOT_SUPPORTED :
		      err == -ENOLINK ? FC_STOP_FATAL : FC_STOP_ERROR, err);
}

static int vooc_send(struct vooc *v, u8 payload, bool shifted)
{
	dev_dbg(fc_dev(v->inst->fc), "VOOC reply cmd=%02x payload=%02x state=%u permit=%d\n",
		v->last_cmd, payload, v->st, v->proto.permit);
	return sc8547d_vooc_tx(v->cp, svooc_frame(svooc_reply_head(&v->proto), payload, shifted));
}

/*
 * GET_VBAT. SuperVOOC: half the input less the drop to the battery under
 * load. VOOC 3.0: the gauge voltage. VOOC 2.0: the gauge voltage in its own
 * coding, once asking the adapter to re-read above 75 % charge.
 */
static u8 vooc_vbat_reply(struct vooc *v, const struct fc_sample *s)
{
	int mv;

	if (v->proto.adapter == SVOOC_ADAPTER_VOOC2) {
		if (s && s->soc > VOOC2_SOC_HIGH && !v->vooc2_asked_again) {
			v->vooc2_asked_again = true;
			return SVOOC_VOOC2_VBAT_AGAIN | SVOOC_ALLOW;
		}
		return svooc_vooc2_vbat_payload(v->vbat_mv, true);
	}
	if (v->change)
		return SVOOC_CHANGE_CURRENT;
	if (v->proto.adapter == SVOOC_ADAPTER_VOOC3)
		return svooc_vbat_payload(v->vbat_mv);
	mv = v->vbus_mv / 2 - (v->vout_mv - v->vbat_mv) / 4 - v->ibus_ma * 75 / 1000;
	return svooc_vbat_payload(mv);
}

static int vooc_power_command(struct vooc *v, const struct svooc_request *req, u8 *payload)
{
	const struct fc_profile *p = &v->inst->prof;
	bool vooc2 = v->proto.adapter == SVOOC_ADAPTER_VOOC2;
	int ret, ma;

	switch (req->command) {
	case SVOOC_CMD_VBUS_OK:
		if (v->st < VOOC_GRANTED)
			return -EPROTO;
		*payload = svooc_vbus_payload(v->vbus_mv, v->vbat_mv, v->ratio,
					      p->vbus_low_mv, p->vbus_high_mv);
		/* VOOC 2.0 keeps the allow bit set while the voltage is off. */
		if (vooc2 && *payload != SVOOC_VBUS_NORMAL)
			*payload |= SVOOC_ALLOW;
		if ((*payload & ~SVOOC_ALLOW) == SVOOC_VBUS_NORMAL && v->st == VOOC_GRANTED) {
			ret = fc_pumps_start(v->inst->fc);
			if (ret)
				return ret;
			dev_dbg(fc_dev(v->inst->fc), "VOOC VBUS_OK; main pump enabled\n");
			v->st = VOOC_STARTING;
			v->started_ms = ktime_to_ms(ktime_get_boottime());
			v->vbus_ok_count = 0;
		} else if (v->st == VOOC_GRANTED && ++v->vbus_ok_count > VOOC_VBUS_OK_MAX) {
			return -EPROTO;
		}
		return 0;
	case SVOOC_CMD_GET_CURRENT:
		if (v->st != VOOC_STARTING && v->st != VOOC_RUNNING)
			return -EPROTO;
		if (vooc2) {
			/* Fixed: 3 A, or 3.75 A with two pumps sharing it. */
			bool dual = fc_second_pump_available(v->inst->fc);

			*payload = dual ? SVOOC_VOOC2_CURR_3P75A : SVOOC_VOOC2_CURR_3A;
			v->applied_ma = dual ? VOOC2_DUAL_CURR_MA : VOOC2_CURR_MA;
			v->change = false;
			return 0;
		}
		if (v->st == VOOC_STARTING && !v->applied_ma)
			ma = v->limit_ma;
		else
			ma = v->change ? v->target_ma : v->applied_ma;
		ret = svooc_current_payload(ma, payload);
		if (ret)
			return ret;
		ret = sc8547d_vooc_irq_mask(v->cp, SC8547D_VOOC_MASK_TX);
		if (ret)
			return ret;
		v->pending_ma = ma;
		v->ack_deadline_ms = ktime_to_ms(ktime_get_boottime()) + VOOC_ACK_MS;
		return 0;
	case SVOOC_CMD_GET_VBAT:
		if (v->st < VOOC_STARTING)
			return -EPROTO;
		*payload = vooc_vbat_reply(v, fc_sample(v->inst->fc));
		return 0;
	case SVOOC_CMD_AP_STATUS:
		*payload = 0;
		return 0;
	default:
		return -EPROTO;
	}
}

static int vooc_receive(struct vooc *v, u8 rx, u8 flags)
{
	struct svooc_request req;
	u8 payload;
	int ret;

	/* Until the adapter's first frame, ignore what the handshake leaves. */
	if (!v->frames && (!(flags & SC8547D_VOOC_RX_DONE) || (flags & SC8547D_VOOC_ERRORS) ||
			   svooc_decode(rx, &req)))
		return (flags & SC8547D_VOOC_RX_DONE) && ++v->ignored > VOOC_IGNORE_MAX ?
		       -EPROTO : 0;
	if (flags & SC8547D_VOOC_ERRORS) {
		/* After the curve a late reply passes: the adapter asks again. */
		if ((flags & SC8547D_VOOC_ERRORS) == SC8547D_VOOC_TX_FAILED &&
		    v->proto.curve_sent && !v->pending_ma && ++v->skipped <= VOOC_SKIP_MAX)
			return 0;
		return -EPROTO;
	}
	if (v->pending_ma && (flags & SC8547D_VOOC_TX_DONE)) {
		v->applied_ma = v->pending_ma;
		v->pending_ma = 0;
		v->change = false;
		ret = sc8547d_vooc_irq_mask(v->cp, SC8547D_VOOC_MASK_RX);
		if (ret)
			return ret;
	} else if (v->pending_ma && (flags & SC8547D_VOOC_RX_DONE)) {
		return -EPROTO;
	}
	if (!(flags & SC8547D_VOOC_RX_DONE))
		return 0;
	v->skipped = 0;
	ret = svooc_decode(rx, &req);
	if (ret)
		return ret;
	v->frames++;
	v->last_cmd = req.command;
	v->last_rx_ms = ktime_to_ms(ktime_get_boottime());
	ret = svooc_handle(&v->proto, &req, get_random_u16(), &payload);
	if (ret < 0)
		return ret;
	/*
	 * Let the worker select the software current limit for this family.
	 * Hardware stays on basic charging throughout the curve exchange;
	 * svooc_handle() gates the allow reply on permit AND curve_sent.
	 */
	if (v->st == VOOC_IDENTIFY && v->proto.identified && v->proto.model &&
	    v->proto.exchange == SVOOC_EXCHANGE_COMMAND) {
		v->st = VOOC_WAIT_PERMIT;
		v->permit_wait_ms = 0;
		v->ratio = svooc_bypass(&v->proto) ? 1 : 2;
		complete(&v->identified);
	}
	if (ret) {
		/* OEM svooc_commu_with_voocphy(): transmit first, then hand over. */
		ret = vooc_send(v, payload, req.shifted);
		if (ret)
			return ret;
		if (req.command == SVOOC_CMD_FASTCHG && v->proto.granted &&
		    v->st == VOOC_WAIT_PERMIT) {
			dev_dbg(fc_dev(v->inst->fc), "VOOC allow sent; suspending basic charging\n");
			ret = fc_vooc_suspend_input(v->inst->fc);
			if (ret)
				return ret;
			/* Missed replies make the PHY ask for a lower voltage. */
			ret = sc8547d_vooc_predata(v->cp, svooc_frame(svooc_reply_head(&v->proto),
								     SVOOC_VBUS_HIGH, false));
			if (!ret)
				ret = fc_pumps_prepare(v->inst->fc, v->ratio == 1 ?
						       SC8547D_MODE_BYPASS : SC8547D_MODE_2_TO_1);
			if (ret)
				return ret;
			v->st = VOOC_GRANTED;
			v->granted_ms = ktime_to_ms(ktime_get_boottime());
			dev_dbg(fc_dev(v->inst->fc), "VOOC pumps prepared; waiting for VBUS_OK\n");
		}
		return 0;
	}
	ret = vooc_power_command(v, &req, &payload);
	if (!ret)
		ret = vooc_send(v, payload, req.shifted);
	if (!ret && req.command == SVOOC_CMD_GET_VBAT)
		ret = sc8547d_vooc_predata(v->cp, svooc_frame(svooc_reply_head(&v->proto),
							     payload, req.shifted));
	return ret;
}

/* Refresh the pump readings in the quiet gap after a reply. */
static void vooc_sample_pump(struct vooc *v)
{
	struct sc8547d_sample s;

	if (sc8547d_sample(v->cp, &s))
		return;
	v->vbus_mv = s.vbus_mv;
	v->vout_mv = s.vout_mv;
	v->vbat_cp_mv = s.vbat_mv;
	v->ibus_ma = s.ibus_ma;
	v->fault_seen |= s.fault;
}

static void vooc_irq(void *data)
{
	struct vooc *v = data;
	u8 rx, flags;
	int ret;

	mutex_lock(&v->lock);
	if (v->phy_on) {
		ret = sc8547d_vooc_rx(v->cp, &rx, &flags);
		if (!ret)
			ret = vooc_receive(v, rx, flags);
		if (!ret && v->frames && (flags & SC8547D_VOOC_RX_DONE) &&
		    !(flags & SC8547D_VOOC_ERRORS) && v->st >= VOOC_GRANTED)
			vooc_sample_pump(v);
		if (ret)
			vooc_fail_locked(v, ret);
	}
	mutex_unlock(&v->lock);
}

int fc_vooc_alloc(struct fc_instance *inst)
{
	struct vooc *v;

	if (!sc8547d_has_vooc_phy(fc_main_pump(inst->fc)))
		return dev_err_probe(fc_dev(inst->fc), -ENODEV, "the main pump has no VOOC PHY\n");
	v = devm_kzalloc(fc_dev(inst->fc), sizeof(*v), GFP_KERNEL);
	if (!v)
		return -ENOMEM;
	v->inst = inst;
	v->cp = fc_main_pump(inst->fc);
	mutex_init(&v->lock);
	init_completion(&v->identified);
	inst->priv = v;
	return 0;
}

static int vooc_probe(struct fc_instance *inst)
{
	struct vooc *v = inst->priv;
	const struct fc_profile *p = &inst->prof;
	int ret;

	mutex_lock(&v->lock);
	svooc_reset(&v->proto);
	memcpy(v->proto.circuit, p->circuit, sizeof(v->proto.circuit));
	memcpy(v->proto.curve, p->battery_curve, sizeof(v->proto.curve));
	memcpy(v->proto.bypass_curve, p->bypass_battery_curve, sizeof(v->proto.bypass_curve));
	v->ratio = 2;
	v->vooc2_asked_again = false;
	v->st = VOOC_IDENTIFY;
	v->err = 0;
	v->frames = v->ignored = v->skipped = v->vbus_ok_count = 0;
	v->applied_ma = v->pending_ma = v->target_ma = 0;
	v->change = false;
	v->last_cmd = 0;
	v->fault_seen = 0;
	v->limit_ma = min(p->curr_max_ma, p->pump_max_ma);
	v->region = FC_REGION_INIT;
	v->exit = v->switch_curve = false;
	v->curve_ready = false;
	v->ntime_count = 0;
	reinit_completion(&v->identified);
	v->phy_on = true;
	mutex_unlock(&v->lock);
	sc8547d_set_irq_handler(v->cp, vooc_irq, v);
	ret = sc8547d_vooc_enable(v->cp);
	if (!ret)
		ret = sc8547d_vooc_handshake(v->cp);
	if (ret)
		goto fail;
	/*
	 * A charger that never sends a frame is no VOOC adapter. One that does
	 * gets the full identification time, counted from the handshake.
	 */
	if (!wait_for_completion_timeout(&v->identified, msecs_to_jiffies(VOOC_FIRST_FRAME_MS))) {
		if (!READ_ONCE(v->frames)) {
			ret = -ENODEV;
			goto fail;
		}
		wait_for_completion_timeout(&v->identified,
					    msecs_to_jiffies(VOOC_IDENTIFY_MS - VOOC_FIRST_FRAME_MS));
	}
	mutex_lock(&v->lock);
	if (v->st == VOOC_STOPPED)
		ret = v->err == -EOPNOTSUPP ? -ENODEV : (v->err ?: -EPROTO);
	else if (v->st != VOOC_WAIT_PERMIT)
		ret = -ETIMEDOUT;
	mutex_unlock(&v->lock);
	if (ret)
		goto fail;
	inst->bypass = svooc_bypass(&v->proto);
	if (inst->bypass && !p->bypass_curve) {
		dev_info(fc_dev(inst->fc), "VOOC adapter but no bypass curve configured\n");
		ret = -ENODEV;
		goto fail;
	}
	if (inst->bypass)
		v->limit_ma = min(p->bypass_curr_max_ma, p->pump_max_ma);
	dev_info(fc_dev(inst->fc), "%s adapter model %02x\n",
		 v->proto.adapter == SVOOC_ADAPTER_SVOOC ? "SuperVOOC" :
		 v->proto.adapter == SVOOC_ADAPTER_VOOC3 ? "VOOC 3.0" : "VOOC 2.0", v->proto.model);
	return 0;
fail:
	mutex_lock(&v->lock);
	v->st = VOOC_STOPPED;
	v->phy_on = false;
	sc8547d_vooc_disable(v->cp);
	mutex_unlock(&v->lock);
	sc8547d_set_irq_handler(v->cp, NULL, NULL);
	return ret;
}

static void vooc_set_current(struct fc_instance *inst, int ma)
{
	struct vooc *v = inst->priv;

	ma = ma / 100 * 100;
	mutex_lock(&v->lock);
	v->limit_ma = ma;
	if (v->proto.adapter == SVOOC_ADAPTER_VOOC2) {
		mutex_unlock(&v->lock);
		return;
	}
	if (v->st == VOOC_RUNNING && ma != v->applied_ma && !v->pending_ma && !v->change) {
		v->target_ma = ma;
		v->change = true;
		v->change_deadline_ms = ktime_to_ms(ktime_get_boottime()) + VOOC_CHANGE_MS;
	}
	mutex_unlock(&v->lock);
}

/* Software admission/limits are ready; hardware handover follows the allow reply. */
void fc_vooc_allow(struct fc_instance *inst)
{
	struct vooc *v = inst->priv;

	mutex_lock(&v->lock);
	if (v->st == VOOC_WAIT_PERMIT && v->phy_on)
		v->proto.permit = true;
	mutex_unlock(&v->lock);
}

/* Temperature regions: oplus_vooc_check_temp_range() and its updates. */

static void vooc_region_reset(struct vooc *v)
{
	memcpy(v->eff, v->inst->prof.region.bound, sizeof(v->eff));
}

static enum fc_region vooc_region_of(struct vooc *v, int temp)
{
	enum fc_region r;

	for (r = FC_REGION_LITTLE_COLD; r < FC_REGION_WARM; r++)
		if (temp < v->eff[r])
			return r;
	return FC_REGION_WARM;
}

static void vooc_region_rise(struct vooc *v, int soc)
{
	const struct fc_region_cfg *cfg = &v->inst->prof.region;

	switch (v->region) {
	case FC_REGION_WARM:
		if (++v->change_count >= VOOC_TEMP_OVER_COUNTS) {
			v->change_count = 0;
			v->exit = true;
		}
		break;
	case FC_REGION_NORMAL_HIGH:
		v->switch_curve = true;
		vooc_region_reset(v);
		v->eff[FC_BOUND_NORMAL_HIGH] -= VOOC_TEMP_HYST;
		break;
	case FC_REGION_NORMAL_LOW:
		v->eff[FC_BOUND_NORMAL_LOW] -= VOOC_TEMP_HYST;
		v->region = FC_REGION_NORMAL_HIGH;
		break;
	case FC_REGION_LITTLE_COOL:
		if (soc <= cfg->soc_high) {
			v->switch_curve = true;
			vooc_region_reset(v);
			v->eff[FC_BOUND_LITTLE_COOL] -= VOOC_TEMP_HYST_HIGH;
		} else {
			v->eff[FC_BOUND_LITTLE_COOL] -= VOOC_TEMP_HYST;
			v->region = FC_REGION_LITTLE_COOL_HIGH;
		}
		break;
	case FC_REGION_LITTLE_COOL_HIGH:
		if (soc <= cfg->soc_high) {
			v->switch_curve = true;
			vooc_region_reset(v);
		} else {
			v->region = FC_REGION_NORMAL_LOW;
		}
		v->eff[FC_BOUND_LITTLE_COOL_HIGH] -= VOOC_TEMP_HYST;
		break;
	case FC_REGION_COOL:
		if (soc <= cfg->soc_high)
			v->switch_curve = true;
		else
			v->region = FC_REGION_LITTLE_COOL;
		vooc_region_reset(v);
		v->eff[FC_BOUND_COOL] -= VOOC_TEMP_HYST;
		break;
	case FC_REGION_LITTLE_COLD:
		v->region = FC_REGION_COOL;
		vooc_region_reset(v);
		v->eff[FC_BOUND_LITTLE_COLD] -= VOOC_TEMP_HYST;
		break;
	default:
		break;
	}
}

static void vooc_region_drop(struct vooc *v, int soc)
{
	const struct fc_region_cfg *cfg = &v->inst->prof.region;

	switch (v->region) {
	case FC_REGION_WARM:
		v->switch_curve = true;
		vooc_region_reset(v);
		v->eff[FC_BOUND_NORMAL_HIGH] += VOOC_TEMP_HYST;
		break;
	case FC_REGION_NORMAL_HIGH:
		v->region = FC_REGION_NORMAL_LOW;
		vooc_region_reset(v);
		v->eff[FC_BOUND_NORMAL_LOW] += VOOC_TEMP_HYST;
		break;
	case FC_REGION_NORMAL_LOW:
		v->region = FC_REGION_LITTLE_COOL_HIGH;
		vooc_region_reset(v);
		v->eff[FC_BOUND_LITTLE_COOL_HIGH] += VOOC_TEMP_HYST;
		break;
	case FC_REGION_LITTLE_COOL_HIGH:
		v->region = FC_REGION_LITTLE_COOL;
		vooc_region_reset(v);
		v->eff[FC_BOUND_LITTLE_COOL] += VOOC_TEMP_HYST;
		break;
	case FC_REGION_LITTLE_COOL:
		if (soc <= cfg->soc_high)
			v->switch_curve = true;
		else
			v->region = FC_REGION_COOL;
		vooc_region_reset(v);
		v->eff[FC_BOUND_COOL] += VOOC_TEMP_HYST;
		break;
	case FC_REGION_COOL:
		v->region = FC_REGION_LITTLE_COLD;
		vooc_region_reset(v);
		v->eff[FC_BOUND_LITTLE_COLD] += VOOC_TEMP_HYST;
		break;
	case FC_REGION_LITTLE_COLD:
		if (++v->change_count >= VOOC_TEMP_OVER_COUNTS) {
			v->change_count = 0;
			v->exit = true;
		}
		break;
	default:
		break;
	}
}

static void vooc_region_update(struct vooc *v, int temp, int soc)
{
	const struct fc_region_cfg *cfg = &v->inst->prof.region;
	int start, end;

	if (v->region == FC_REGION_INIT) {
		vooc_region_reset(v);
		v->region = vooc_region_of(v, temp);
		v->change_count = 0;
	}
	start = v->region == FC_REGION_LITTLE_COLD ? cfg->low_temp - VOOC_TEMP_HYST :
		v->eff[v->region - 1];
	end = v->region == FC_REGION_WARM ? cfg->bound[FC_BOUND_OVER_HIGH] : v->eff[v->region];
	if (temp > end)
		vooc_region_rise(v, soc);
	else if (temp < start)
		vooc_region_drop(v, soc);
	if (v->switch_curve &&
	    (temp < cfg->low_temp - VOOC_TEMP_HYST || temp > cfg->bound[FC_BOUND_OVER_HIGH]))
		v->exit = true;
}

static int vooc_temp_lo(const struct fc_profile *p, enum fc_region r)
{
	return r == FC_REGION_LITTLE_COLD ? p->region.low_temp : p->region.bound[r - 1];
}

/*
 * fc_vooc_sample() - the session tick's strategy part
 * @limit_ma: the input current the adapter may be asked for
 *
 * Return: 0, -EPERM to leave for temperature, -EAGAIN to restart for a new
 * region, -ENODATA when full, or another error.
 */
int fc_vooc_sample(struct fc_instance *inst, const struct fc_sample *s, int *limit_ma)
{
	struct vooc *v = inst->priv;
	const struct fc_profile *p = &inst->prof;
	const struct fc_curve *curve;
	const struct fc_curve_row *gear;
	int ibus = s->pump[0].ibus_ma + (s->pump_valid[1] && s->pump[1].switching ?
					 s->pump[1].ibus_ma : 0);
	int cgcl_ma, ret;

	mutex_lock(&v->lock);
	v->vbat_mv = s->vbat_mv;
	if (v->fault_seen) {
		dev_err(fc_dev(inst->fc), "pump fault: %s\n", sc8547d_fault_name(v->fault_seen));
		mutex_unlock(&v->lock);
		return -EIO;
	}
	mutex_unlock(&v->lock);

	vooc_region_update(v, s->temp, s->soc);
	if (v->exit)
		return -EPERM;
	if (v->switch_curve)
		return -EAGAIN;
	curve = inst->bypass ? p->bypass_curve : p->curve;
	if (!v->curve_ready) {
		ret = fc_curve_select(curve, &inst->curve, s->soc, vooc_temp_lo(p, v->region),
				      s->vbat_mv, s->now_ms);
		if (ret)
			return ret;
		v->curve_ready = true;
		gear = fc_curve_current(curve, &inst->curve);
		v->current_max_ma = gear ? gear->ma : VOOC_MIN_MA;
	} else {
		gear = fc_curve_step_vooc(curve, &inst->curve, s->vbat_mv, ibus, s->now_ms);
		if (gear)
			v->current_max_ma = min(v->current_max_ma, gear->ma);
	}
	if (!gear)
		return -ENODATA;
	/* Full by voltage: once above the first threshold, five times above the second */
	if (s->vbat_mv > p->full_1time_mv[v->region])
		return -ENODATA;
	if (s->vbat_mv > p->full_ntime_mv[v->region]) {
		if (++v->ntime_count > VOOC_FULL_NTIME_COUNTS)
			return -ENODATA;
	} else {
		v->ntime_count = 0;
	}
	/* The CGCL gear is a battery current; a 2:1 pump halves it on the input. */
	cgcl_ma = fc_cgcl_update(&p->cgcl, &inst->cgcl, s->temp) / v->ratio;
	*limit_ma = min3(v->current_max_ma, cgcl_ma, p->pump_max_ma);
	if (*limit_ma < VOOC_MIN_MA)
		return -ENODATA;
	return 0;
}

/* VOOC 2.0 delivers one current; the core compares it with the final limit. */
static int vooc_fixed_current(struct fc_instance *inst)
{
	struct vooc *v = inst->priv;

	if (v->proto.adapter != SVOOC_ADAPTER_VOOC2)
		return 0;
	return fc_second_pump_available(inst->fc) ? VOOC2_DUAL_CURR_MA : VOOC2_CURR_MA;
}

/* Protocol bookkeeping each tick: the grant, timeouts, replies gone stale. */
static int vooc_tick(struct fc_instance *inst, const struct fc_sample *s)
{
	struct vooc *v = inst->priv;
	int ret = 0;
	u64 now_ms;

	mutex_lock(&v->lock);
	/* IRQ timestamps may be newer than the sample collected before this lock. */
	now_ms = ktime_to_ms(ktime_get_boottime());
	switch (v->st) {
	case VOOC_STOPPED:
		ret = v->err ?: -EIO;
		break;
	case VOOC_WAIT_PERMIT:
		if (!v->permit_wait_ms)
			v->permit_wait_ms = now_ms;
		if (now_ms - v->permit_wait_ms > VOOC_PERMIT_MS)
			ret = -ETIMEDOUT;
		break;
	case VOOC_GRANTED:
		if (now_ms - v->granted_ms > VOOC_GRANT_MS)
			ret = -ETIMEDOUT;
		break;
	case VOOC_STARTING:
		if (s->pump_valid[0] && s->pump[0].switching && v->applied_ma) {
			v->st = VOOC_RUNNING;
			dev_info(fc_dev(inst->fc), "%s running; requested current %d mA\n",
				 v->proto.adapter == SVOOC_ADAPTER_SVOOC ? "SuperVOOC" :
				 v->proto.adapter == SVOOC_ADAPTER_VOOC3 ? "VOOC 3.0" : "VOOC 2.0",
				 v->applied_ma);
		} else if (now_ms - v->started_ms > VOOC_START_MS) {
			ret = -ETIMEDOUT;
		}
		break;
	default:
		break;
	}
	if (!ret && v->frames && now_ms - v->last_rx_ms > VOOC_FRAME_MS)
		ret = -ETIMEDOUT;
	if (!ret && v->pending_ma && now_ms > v->ack_deadline_ms)
		ret = -ETIMEDOUT;
	if (!ret && v->change && now_ms > v->change_deadline_ms)
		ret = -ETIMEDOUT;
	if (ret && v->st != VOOC_STOPPED) {
		dev_warn(fc_dev(inst->fc),
			 "VOOC timeout state=%u frames=%u rx_age=%llums last_cmd=%02x usb=%dmV/%dmA permit=%d pending=%dmA\n",
			 v->st, v->frames, now_ms - v->last_rx_ms, v->last_cmd,
			 s->usb_mv, s->usb_ma, v->proto.permit, v->pending_ma);
		v->st = VOOC_STOPPED;
		v->err = ret;
		v->phy_on = false;
		sc8547d_vooc_disable(v->cp);
	}
	mutex_unlock(&v->lock);
	return ret;
}

static void vooc_stop(struct fc_instance *inst)
{
	struct vooc *v = inst->priv;

	mutex_lock(&v->lock);
	v->st = VOOC_STOPPED;
	v->phy_on = false;
	v->proto.permit = false;
	sc8547d_vooc_disable(v->cp);
	complete(&v->identified);
	mutex_unlock(&v->lock);
	sc8547d_set_irq_handler(v->cp, NULL, NULL);
}

const struct fc_protocol_ops fc_vooc_ops = {
	.name = "SuperVOOC",
	.id = FC_PROTO_VOOC,
	.adapter_regulated = true,
	.needs_dpdm = true,
	.probe = vooc_probe,
	.set_current = vooc_set_current,
	.fixed_current = vooc_fixed_current,
	.tick = vooc_tick,
	.stop = vooc_stop,
};
