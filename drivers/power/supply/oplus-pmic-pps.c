// SPDX-License-Identifier: GPL-2.0-only
/*
 * OnePlus PMIC PPS policy, using the existing ADSP-controlled PMIC buck.
 * PPS commands and curve progression derived from the public Oplus v2 driver:
 * android_kernel_modules_and_devicetree_oneplus_sm8850, 8be53e8b737a.
 * Copyright (C) 2020-2024 Oplus. All rights reserved.
 */
#include <linux/cleanup.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pid.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/power/qcom-battmgr-pps.h>
#include <linux/power_supply.h>
#include <linux/rcupdate.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/usb/pd.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>

#define PPS_MAX_ROWS 512
#define PPS_LAST BIT(0)
#define PPS_CV BIT(1)
#define PPS_SESSION_MS 33595000ULL

/* Inclusive SOC, half-open temperature (deci-Celsius), mV, mA, seconds. */
struct pps_row {
	s32 soc_lo, soc_hi, temp_lo, temp_hi, level;
	s32 mv, vbat_mv, ma, flags, seconds;
};

struct pps_table {
	unsigned int count;
	struct pps_row rows[PPS_MAX_ROWS];
};

struct oplus_pps {
	struct device *dev;
	struct power_supply *usb, *battery, *psy;
	struct mutex lock;
	struct workqueue_struct *wq;
	struct delayed_work work;
	struct notifier_block psy_nb, pm_nb;
	struct wakeup_source *charging_ws;
	struct pps_table *table, *staging;
	struct pid *loader;
	int staging_error;
	bool ready, enabled, suspended, removing, controlled, active, fault, online;
	bool block_suspend;
	u32 baseline_ua;
	int soc, temp, vbat_mv, vbus_uv, ibus_ua;
	int min_mv, max_mv, max_ma, request_mv, request_ma;
	unsigned int first, end, index;
	u64 session_start, row_deadline, over_deadline, ramp_deadline;
	int last_error;
	atomic_t sample_event;
	unsigned int over_current_count;
};

static bool pps_same_group(const struct pps_row *a, const struct pps_row *b)
{
	return a->soc_lo == b->soc_lo && a->soc_hi == b->soc_hi &&
	       a->temp_lo == b->temp_lo && a->temp_hi == b->temp_hi;
}

static int pps_validate(const struct pps_table *t)
{
	unsigned int i, j;

	if (!t->count || t->count > PPS_MAX_ROWS)
		return -EINVAL;
	for (i = 0; i < t->count; i++) {
		const struct pps_row *r = &t->rows[i];

		if (r->soc_lo < 0 || r->soc_hi > 100 || r->soc_lo > r->soc_hi ||
		    r->temp_lo < 0 || r->temp_hi > 600 || r->temp_lo >= r->temp_hi ||
		    r->mv < 5500 || r->mv > 21000 ||
		    r->vbat_mv < 2500 || r->vbat_mv > 4560 ||
		    r->ma < 100 || r->ma > 15000 || r->ma % 50 ||
		    r->flags < 0 || r->flags > 3 || r->seconds < 0 ||
		    r->seconds > 86400 || r->level < 0 || r->level >= 64)
			return -ERANGE;
		if (r->level) {
			if (!i || !pps_same_group(r, &t->rows[i - 1]) ||
			    r->level != t->rows[i - 1].level + 1 ||
			    t->rows[i - 1].flags & PPS_LAST)
				return -EINVAL;
		} else {
			if (i && !(t->rows[i - 1].flags & PPS_LAST))
				return -EINVAL;
			for (j = 0; j < i; j++) {
				const struct pps_row *p = &t->rows[j];

				if (!p->level && p->soc_lo <= r->soc_hi &&
				    r->soc_lo <= p->soc_hi && p->temp_lo < r->temp_hi &&
				    r->temp_lo < p->temp_hi)
					return -EEXIST;
			}
		}
	}
	return t->rows[t->count - 1].flags & PPS_LAST ? 0 : -EINVAL;
}

static int pps_read(struct power_supply *psy, enum power_supply_property prop, int *v)
{
	union power_supply_propval val;
	int ret;

	ret = power_supply_get_property(psy, prop, &val);
	if (!ret)
		*v = val.intval;
	return ret;
}

/* lock held; independent of PPS activation, including basic charging. */
static void pps_update_suspend_block(struct oplus_pps *p)
{
	if (p->block_suspend && p->online && !p->removing)
		__pm_stay_awake(p->charging_ws);
	else
		__pm_relax(p->charging_ws);
}

static int pps_sample(struct oplus_pps *p)
{
	int ret, online, uv;

	ret = pps_read(p->usb, POWER_SUPPLY_PROP_ONLINE, &online);
	if (ret)
		return ret;
	p->online = online == 1;
	pps_update_suspend_block(p);
	if (!p->online) {
		p->vbus_uv = p->ibus_ua = 0;
		return 0;
	}
	ret = pps_read(p->battery, POWER_SUPPLY_PROP_CAPACITY, &p->soc);
	if (ret)
		return ret;
	ret = pps_read(p->battery, POWER_SUPPLY_PROP_TEMP, &p->temp);
	if (ret)
		return ret;
	ret = pps_read(p->battery, POWER_SUPPLY_PROP_VOLTAGE_NOW, &uv);
	if (ret)
		return ret;
	p->vbat_mv = uv / 1000;
	ret = pps_read(p->usb, POWER_SUPPLY_PROP_VOLTAGE_NOW, &p->vbus_uv);
	if (ret)
		return ret;
	ret = pps_read(p->usb, POWER_SUPPLY_PROP_CURRENT_NOW, &p->ibus_ua);
	if (ret)
		return ret;
	if (p->soc < 0 || p->soc > 100 || p->temp < 0 || p->temp >= 500 ||
	    p->vbat_mv < 2500 || p->vbat_mv > 4560 ||
	    p->vbus_uv < 0 || p->vbus_uv > 21000000 ||
	    p->ibus_ua < 0 || p->ibus_ua > 15000000)
		return -ERANGE;
	return 0;
}

/* lock held; fixed 5 V and the captured native ICL, not a 500 mA clamp. */
static int pps_stop(struct oplus_pps *p)
{
	int ret = 0, online;

	if (p->controlled) {
		ret = pps_read(p->usb, POWER_SUPPLY_PROP_ONLINE, &online);
		if (ret)
			return ret;
		if (online) {
			ret = qcom_battmgr_pps_request(p->usb, QCOM_BATTMGR_PPS_BASELINE,
						      &p->baseline_ua);
			if (ret)
				return ret;
		}
	}
	p->controlled = p->active = false;
	p->request_mv = p->request_ma = 0;
	p->row_deadline = p->over_deadline = p->ramp_deadline = 0;
	cancel_delayed_work(&p->work);
	return ret;
}

static int pps_discover(struct oplus_pps *p)
{
	u32 type, pdos[7];
	int i, ret;

	ret = qcom_battmgr_pps_request(p->usb, QCOM_BATTMGR_PPS_TYPE, &type);
	if (ret)
		return ret;
	if (type != 8)
		return -EOPNOTSUPP;
	ret = qcom_battmgr_pps_request(p->usb, QCOM_BATTMGR_PPS_CAPS, pdos);
	if (ret)
		return ret;
	p->min_mv = p->max_mv = p->max_ma = 0;
	for (i = 0; i < ARRAY_SIZE(pdos); i++) {
		u32 v = pdos[i];
		int lo, hi, ma;

		if (pdo_type(v) != PDO_TYPE_APDO || pdo_apdo_type(v) != APDO_TYPE_PPS)
			continue;
		lo = max(5500, (int)pdo_pps_apdo_min_voltage(v));
		hi = min(10500, (int)pdo_pps_apdo_max_voltage(v));
		ma = min(3000, (int)pdo_pps_apdo_max_current(v));
		if (lo > 5500 || hi < 5500 || ma < 100 ||
		    hi * ma <= p->max_mv * p->max_ma)
			continue;
		p->min_mv = lo;
		p->max_mv = hi;
		p->max_ma = ma;
	}
	return p->max_ma ? 0 : -EOPNOTSUPP;
}

static void pps_enter_row(struct oplus_pps *p, u64 now)
{
	p->over_deadline = 0;
	p->row_deadline = p->index < p->end && p->table->rows[p->index].seconds ?
		now + (u64)p->table->rows[p->index].seconds * 1000 : 0;
}

static int pps_begin_curve(struct oplus_pps *p, u64 now)
{
	unsigned int i;

	for (i = 0; i < p->table->count; i++) {
		const struct pps_row *r = &p->table->rows[i];

		if (r->level || p->soc < r->soc_lo || p->soc > r->soc_hi ||
		    p->temp < r->temp_lo || p->temp >= r->temp_hi)
			continue;
		p->first = i;
		p->end = i + 1;
		while (p->end < p->table->count &&
		       pps_same_group(r, &p->table->rows[p->end]))
			p->end++;
		for (p->index = i; p->index < p->end; p->index++)
			if (p->vbat_mv < p->table->rows[p->index].vbat_mv)
				break;
		if (p->index == p->end)
			p->index--;
		pps_enter_row(p, now);
		p->session_start = now;
		return 0;
	}
	return -ENOENT;
}

static int pps_current_step(int previous, int target)
{
	int d = abs(previous - target);
	int step = d >= 500 ? 500 : d >= 300 ? 300 : d >= 100 ? 100 : 50;

	return previous + clamp(target - previous, -step, step);
}

static u64 pps_sooner(u64 a, u64 b)
{
	return !a ? b : !b ? a : min(a, b);
}

static void pps_queue(struct oplus_pps *p)
{
	if (READ_ONCE(p->ready) && !READ_ONCE(p->removing) && !READ_ONCE(p->suspended))
		mod_delayed_work(p->wq, &p->work, 0);
}

/* Only supply notifications and finite ramp/curve/session deadlines schedule
 * work. There is no recurring telemetry or charger-discovery timer.
 */
static void pps_work(struct work_struct *work)
{
	struct oplus_pps *p = container_of(to_delayed_work(work), struct oplus_pps, work);
	const struct pps_row *r;
	u64 now, next = 0;
	u32 target[2];
	int ret, mv, ma, goal_mv, goal_ma, native_icl;
	bool ramp_due, fresh;

	mutex_lock(&p->lock);
	fresh = atomic_xchg(&p->sample_event, 0);
	if (p->removing || p->suspended)
		goto out;
	ret = pps_sample(p);
	if (ret) {
		/* Cold boot may precede ADSP readiness. Retry only on its event. */
		if (!p->controlled && !p->active &&
		    (ret == -EAGAIN || ret == -ECONNRESET || ret == -ENOTCONN))
			goto out;
		goto fail;
	}
	if (!p->online) {
		ret = pps_stop(p);
		if (ret)
			goto fail;
		p->fault = false;
		p->last_error = 0;
		goto out;
	}
	if (!p->enabled || p->fault) {
		if (p->controlled) {
			ret = pps_stop(p);
			if (ret)
				goto fail;
		}
		goto out;
	}
	now = ktime_to_ms(ktime_get_boottime());
	if (!p->active) {
		ret = pps_discover(p);
		if (ret == -EOPNOTSUPP || ret == -EAGAIN)
			goto out;
		if (ret)
			goto fail;
		ret = pps_begin_curve(p, now);
		if (ret == -ENOENT)
			goto out;
		if (ret)
			goto fail;
		ret = pps_read(p->usb, POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &native_icl);
		if (ret)
			goto fail;
		if (native_icl < 100000 || native_icl > 5000000) {
			ret = -ERANGE;
			goto fail;
		}
		p->baseline_ua = native_icl;
		p->over_current_count = 0;
		p->active = true;
	}
	r = &p->table->rows[p->first];
	if (p->soc < r->soc_lo || p->soc > r->soc_hi ||
	    p->temp < r->temp_lo || p->temp >= r->temp_hi) {
		ret = pps_stop(p);
		if (ret)
			goto fail;
		pps_queue(p);
		goto out;
	}
	if (now >= p->session_start + PPS_SESSION_MS) {
		ret = -ETIME;
		goto fail;
	}
	r = &p->table->rows[p->index];
	if (p->row_deadline && now >= p->row_deadline) {
		p->index++;
		pps_enter_row(p, now);
	} else if (p->vbat_mv > r->vbat_mv &&
		   (r->flags & (PPS_LAST | PPS_CV)) != (PPS_LAST | PPS_CV)) {
		if (!p->over_deadline)
			p->over_deadline = now + (r->flags & PPS_CV ? 10000 : 2500);
		else if (now >= p->over_deadline) {
			p->index++;
			pps_enter_row(p, now);
		}
	} else {
		p->over_deadline = 0;
	}
	if (p->index == p->end) {
		ret = -ENODATA;
		goto fail;
	}
	r = &p->table->rows[p->index];
	goal_mv = min(r->mv, p->max_mv);
	goal_mv -= (goal_mv - p->min_mv) % 20;
	goal_ma = min(r->ma, p->max_ma) / 50 * 50;
	/* Count real supply notifications, never ramp timer invocations. */
	if (fresh) {
		if (p->ibus_ua / 1000 > goal_ma + 500) {
			if (++p->over_current_count >= 10) {
				ret = -EOVERFLOW;
				goto fail;
			}
		} else {
			p->over_current_count = 0;
		}
	}
	ramp_due = !p->ramp_deadline || now >= p->ramp_deadline;
	mv = p->request_mv;
	ma = p->request_ma;
	if (ramp_due) {
		mv = mv ? clamp(goal_mv, mv - 500, mv + 500) : 5500;
		ma = pps_current_step(ma, goal_ma);
	}
	target[0] = mv;
	target[1] = ma;
	if (mv != p->request_mv || ma != p->request_ma) {
		/* A partial command also requires baseline restoration. */
		p->controlled = true;
		ret = qcom_battmgr_pps_request(p->usb, QCOM_BATTMGR_PPS_TARGET, target);
		if (ret)
			goto fail;
		p->request_mv = mv;
		p->request_ma = ma;
	}
	now = ktime_to_ms(ktime_get_boottime());
	if (mv != goal_mv || ma != goal_ma) {
		if (ramp_due)
			p->ramp_deadline = now + 500;
		next = p->ramp_deadline;
	} else {
		p->ramp_deadline = 0;
	}
	next = pps_sooner(next, p->row_deadline);
	next = pps_sooner(next, p->over_deadline);
	next = pps_sooner(next, p->session_start + PPS_SESSION_MS);
	mod_delayed_work(p->wq, &p->work, msecs_to_jiffies(next > now ? next - now : 1));
	goto out;
fail:
	p->last_error = ret;
	p->fault = true;
	ret = pps_stop(p);
	if (ret)
		p->last_error = ret;
	dev_err_ratelimited(p->dev, "PPS stopped: %d (baseline restore: %d)\n",
			    p->last_error, ret);
out:
	mutex_unlock(&p->lock);
	power_supply_changed(p->psy);
}

static int pps_changed(struct notifier_block *nb, unsigned long event, void *data)
{
	struct oplus_pps *p = container_of(nb, struct oplus_pps, psy_nb);

	if (event == PSY_EVENT_PROP_CHANGED && (data == p->usb || data == p->battery)) {
		/* This is a blocking notifier. Serialize event queueing with the
		 * worker's deadline scheduling so an immediate event stays immediate.
		 */
		mutex_lock(&p->lock);
		atomic_set(&p->sample_event, 1);
		pps_queue(p);
		mutex_unlock(&p->lock);
	}
	return NOTIFY_OK;
}

static enum power_supply_property pps_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
};

static int pps_get_property(struct power_supply *psy, enum power_supply_property prop,
			    union power_supply_propval *val)
{
	struct oplus_pps *p = power_supply_get_drvdata(psy);

	guard(mutex)(&p->lock);
	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = !p->online ? 0 : (p->active && p->request_mv) ? 2 : 1;
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = (p->active && p->request_mv) ? POWER_SUPPLY_USB_TYPE_PD_PPS :
			POWER_SUPPLY_USB_TYPE_UNKNOWN;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		val->intval = p->min_mv * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = p->max_mv * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = p->max_ma * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = p->vbus_uv;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = p->ibus_ua;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int pps_set_property(struct power_supply *psy, enum power_supply_property prop,
			    const union power_supply_propval *val)
{
	struct oplus_pps *p = power_supply_get_drvdata(psy);
	int ret = 0;

	if (prop != POWER_SUPPLY_PROP_ONLINE || (val->intval != 1 && val->intval != 2))
		return -EINVAL;
	mutex_lock(&p->lock);
	if (!p->ready || p->suspended || p->removing) {
		ret = -EBUSY;
		goto out;
	}
	if (val->intval == 1)
		ret = pps_stop(p);
	if (!ret) {
		p->enabled = val->intval == 2;
		p->fault = false;
		p->last_error = 0;
	}
	if (!ret)
		pps_queue(p);
out:
	mutex_unlock(&p->lock);
	return ret;
}

static int pps_property_writeable(struct power_supply *psy, enum power_supply_property prop)
{
	return prop == POWER_SUPPLY_PROP_ONLINE;
}

static const struct power_supply_desc pps_desc = {
	.name = "oplus-pmic-pps",
	.type = POWER_SUPPLY_TYPE_USB,
	.usb_types = BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN) | BIT(POWER_SUPPLY_USB_TYPE_PD_PPS),
	.properties = pps_properties,
	.num_properties = ARRAY_SIZE(pps_properties),
	.get_property = pps_get_property,
	.set_property = pps_set_property,
	.property_is_writeable = pps_property_writeable,
};

static void pps_discard_staging(struct oplus_pps *p)
{
	kvfree(p->staging);
	p->staging = NULL;
	put_pid(p->loader);
	p->loader = NULL;
	p->staging_error = 0;
}

/* A single writer publishes a complete validated table. No partially loaded
 * curve can become active. Disable automatic PPS (online=1) before reloading.
 */
static int pps_parse_row(const char *buf, struct pps_row *r)
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

static ssize_t curve_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct oplus_pps *p = power_supply_get_drvdata(dev_get_drvdata(dev));
	struct pps_row r;
	bool alive;
	int ret = 0;

	if (!count || count >= PAGE_SIZE || memchr(buf, '\0', count))
		return -EINVAL;
	guard(mutex)(&p->lock);
	if (!p->ready || p->enabled || p->controlled || p->suspended || p->removing)
		return -EBUSY;
	if (sysfs_streq(buf, "begin 1")) {
		rcu_read_lock();
		alive = p->loader && pid_has_task(p->loader, PIDTYPE_TGID);
		rcu_read_unlock();
		if (alive && p->loader != task_tgid(current))
			return -EBUSY;
		pps_discard_staging(p);
		p->staging = kvzalloc(sizeof(*p->staging), GFP_KERNEL);
		if (!p->staging)
			return -ENOMEM;
		p->loader = get_pid(task_tgid(current));
		return count;
	}
	if (!p->staging || p->loader != task_tgid(current))
		return -EPERM;
	if (sysfs_streq(buf, "abort")) {
		pps_discard_staging(p);
		return count;
	}
	if (p->staging_error)
		return p->staging_error;
	if (sysfs_streq(buf, "commit")) {
		ret = pps_validate(p->staging);
		if (!ret) {
			kvfree(p->table);
			p->table = p->staging;
			p->staging = NULL;
			pps_discard_staging(p);
		}
	} else {
		ret = pps_parse_row(buf, &r);
		if (!ret) {
			if (p->staging->count == PPS_MAX_ROWS)
				ret = -E2BIG;
			else
				p->staging->rows[p->staging->count++] = r;
		}
	}
	p->staging_error = ret;
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(curve);

static ssize_t policy_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_pps *p = power_supply_get_drvdata(dev_get_drvdata(dev));

	guard(mutex)(&p->lock);
	return sysfs_emit(buf, "%s error=%d target=%dmV/%dmA\n",
			 p->fault ? "fault" : p->suspended ? "suspended" :
			 p->controlled ? "running" : "basic", p->last_error,
			 p->request_mv, p->request_ma);
}
static DEVICE_ATTR_RO(policy_state);

static ssize_t block_suspend_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct oplus_pps *p = power_supply_get_drvdata(dev_get_drvdata(dev));

	guard(mutex)(&p->lock);
	return sysfs_emit(buf, "%u\n", p->block_suspend);
}

static ssize_t block_suspend_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct oplus_pps *p = power_supply_get_drvdata(dev_get_drvdata(dev));
	bool block;
	int online, ret;

	ret = kstrtobool(buf, &block);
	if (ret)
		return ret;
	guard(mutex)(&p->lock);
	if (!p->ready || p->suspended || p->removing)
		return -EBUSY;
	/* Refresh on enable so the new preference takes effect immediately.
	 * If ADSP is unavailable, preserve the last known connection state;
	 * PM prepare will reject suspend until it can confirm disconnection.
	 */
	if (block && !pps_read(p->usb, POWER_SUPPLY_PROP_ONLINE, &online))
		p->online = online == 1;
	p->block_suspend = block;
	pps_update_suspend_block(p);
	return count;
}
static DEVICE_ATTR_RW(block_suspend);

static struct attribute *pps_attrs[] = {
	&dev_attr_curve.attr,
	&dev_attr_policy_state.attr,
	&dev_attr_block_suspend.attr,
	NULL,
};
ATTRIBUTE_GROUPS(pps);

/* PM notifier runs before devices are suspended. GLINK/battmgr are still
 * available here; failure to leave PPS aborts system suspend.
 */
static int pps_pm_event(struct notifier_block *nb, unsigned long event, void *data)
{
	struct oplus_pps *p = container_of(nb, struct oplus_pps, pm_nb);
	int ret, online;

	switch (event) {
	case PM_SUSPEND_PREPARE:
	case PM_HIBERNATION_PREPARE:
		mutex_lock(&p->lock);
		if (p->block_suspend) {
			/* Also veto explicit suspend that bypasses wakeup_count.
			 * Do not stop PPS on a rejected suspend attempt.
			 */
			ret = pps_read(p->usb, POWER_SUPPLY_PROP_ONLINE, &online);
			if (!ret) {
				p->online = online == 1;
				pps_update_suspend_block(p);
				if (p->online)
					ret = -EBUSY;
			}
			if (ret) {
				mutex_unlock(&p->lock);
				return notifier_from_errno(ret);
			}
		}
		p->suspended = true;
		ret = pps_stop(p);
		if (ret) {
			p->suspended = false;
			p->fault = true;
			p->last_error = ret;
		}
		mutex_unlock(&p->lock);
		cancel_delayed_work_sync(&p->work);
		if (ret) {
			dev_err(p->dev, "cannot restore basic charging before suspend: %d\n", ret);
			mutex_lock(&p->lock);
			pps_queue(p);
			mutex_unlock(&p->lock);
		}
		return notifier_from_errno(ret);
	case PM_POST_SUSPEND:
	case PM_POST_HIBERNATION:
		mutex_lock(&p->lock);
		p->suspended = false;
		pps_queue(p);
		mutex_unlock(&p->lock);
		return NOTIFY_OK;
	default:
		return NOTIFY_DONE;
	}
}

static void pps_put_supply(void *data)
{
	power_supply_put(data);
}

static void pps_unregister_wakeup(void *data)
{
	struct wakeup_source *ws = data;

	__pm_relax(ws);
	wakeup_source_unregister(ws);
}

static int pps_probe(struct platform_device *pdev)
{
	struct power_supply_config cfg = {};
	struct device *dev = &pdev->dev;
	struct device_node *node;
	struct oplus_pps *p;
	struct device *supplier;
	int cells, ret;

	p = devm_kzalloc(dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	p->dev = dev;
	p->block_suspend = true;
	mutex_init(&p->lock);
	platform_set_drvdata(pdev, p);
	/* Table lifetime is independent of devres so sysfs can replace it. */
	p->table = kvzalloc(sizeof(*p->table), GFP_KERNEL);
	if (!p->table)
		return -ENOMEM;
	cells = of_property_count_u32_elems(dev->of_node, "oplus,pps-charge-curve");
	if (cells <= 0 || cells % 10 || cells / 10 > PPS_MAX_ROWS) {
		ret = -EINVAL;
		goto free_table;
	}
	p->table->count = cells / 10;
	ret = of_property_read_u32_array(dev->of_node, "oplus,pps-charge-curve",
					 (u32 *)p->table->rows, cells);
	if (!ret)
		ret = pps_validate(p->table);
	if (ret)
		goto free_table;
	p->usb = power_supply_get_by_name("qcom-battmgr-usb");
	if (!p->usb) {
		ret = -EPROBE_DEFER;
		goto free_table;
	}
	ret = devm_add_action_or_reset(dev, pps_put_supply, p->usb);
	if (ret)
		goto free_table;
	p->battery = power_supply_get_by_name("qcom-battmgr-bat");
	if (!p->battery) {
		ret = -EPROBE_DEFER;
		goto free_table;
	}
	ret = devm_add_action_or_reset(dev, pps_put_supply, p->battery);
	if (ret)
		goto free_table;
	/* Match the explicitly described GLINK supplier, not an arbitrary psy. */
	supplier = p->usb->dev.parent;
	node = of_parse_phandle(dev->of_node, "oplus,pmic-glink", 0);
	if (!node || !supplier || !supplier->parent ||
	    supplier->parent->of_node != node) {
		of_node_put(node);
		ret = -EINVAL;
		goto free_table;
	}
	of_node_put(node);
	if (!device_link_add(dev, supplier, DL_FLAG_AUTOREMOVE_CONSUMER)) {
		ret = -ENOMEM;
		goto free_table;
	}
	p->charging_ws = wakeup_source_register(dev, "oplus-pmic-pps-charging");
	if (IS_ENABLED(CONFIG_PM_SLEEP) && !p->charging_ws) {
		ret = -ENOMEM;
		goto free_table;
	}
	ret = devm_add_action_or_reset(dev, pps_unregister_wakeup, p->charging_ws);
	if (ret)
		goto free_table;
	p->wq = alloc_ordered_workqueue("oplus-pmic-pps", WQ_MEM_RECLAIM);
	if (!p->wq) {
		ret = -ENOMEM;
		goto free_table;
	}
	INIT_DELAYED_WORK(&p->work, pps_work);
	cfg.drv_data = p;
	cfg.fwnode = dev_fwnode(dev);
	cfg.attr_grp = pps_groups;
	p->psy = power_supply_register(dev, &pps_desc, &cfg);
	if (IS_ERR(p->psy)) {
		ret = PTR_ERR(p->psy);
		goto destroy_wq;
	}
	p->psy_nb.notifier_call = pps_changed;
	ret = power_supply_reg_notifier(&p->psy_nb);
	if (ret)
		goto unregister_psy;
	p->pm_nb.notifier_call = pps_pm_event;
	ret = register_pm_notifier(&p->pm_nb);
	if (ret)
		goto unregister_notifier;
	mutex_lock(&p->lock);
	p->enabled = true;
	p->ready = true;
	pps_queue(p);
	mutex_unlock(&p->lock);
	return 0;
unregister_notifier:
	power_supply_unreg_notifier(&p->psy_nb);
unregister_psy:
	power_supply_unregister(p->psy);
destroy_wq:
	destroy_workqueue(p->wq);
free_table:
	kvfree(p->table);
	return dev_err_probe(dev, ret, "cannot initialize PPS\n");
}

static void pps_remove(struct platform_device *pdev)
{
	struct oplus_pps *p = platform_get_drvdata(pdev);
	int ret;

	mutex_lock(&p->lock);
	p->removing = true;
	mutex_unlock(&p->lock);
	unregister_pm_notifier(&p->pm_nb);
	power_supply_unreg_notifier(&p->psy_nb);
	cancel_delayed_work_sync(&p->work);
	mutex_lock(&p->lock);
	ret = pps_stop(p);
	pps_update_suspend_block(p);
	mutex_unlock(&p->lock);
	if (ret)
		dev_err(p->dev, "basic charging restore failed during remove: %d\n", ret);
	power_supply_unregister(p->psy);
	destroy_workqueue(p->wq);
	pps_discard_staging(p);
	kvfree(p->table);
}

static void pps_shutdown(struct platform_device *pdev)
{
	struct oplus_pps *p = platform_get_drvdata(pdev);
	int ret;

	mutex_lock(&p->lock);
	p->removing = true;
	mutex_unlock(&p->lock);
	unregister_pm_notifier(&p->pm_nb);
	power_supply_unreg_notifier(&p->psy_nb);
	cancel_delayed_work_sync(&p->work);
	mutex_lock(&p->lock);
	ret = pps_stop(p);
	pps_update_suspend_block(p);
	mutex_unlock(&p->lock);
	if (ret)
		dev_err(p->dev, "basic charging restore failed during shutdown: %d\n", ret);
}

static const struct of_device_id pps_of_match[] = {
	{ .compatible = "oplus,iceland-pmic-pps" },
	{}
};
MODULE_DEVICE_TABLE(of, pps_of_match);

static struct platform_driver pps_driver = {
	.probe = pps_probe,
	.remove = pps_remove,
	.shutdown = pps_shutdown,
	.driver = {
		.name = "oplus-pmic-pps",
		.of_match_table = pps_of_match,
	},
};
module_platform_driver(pps_driver);

MODULE_DESCRIPTION("OnePlus PMIC PPS power supply and charging policy");
MODULE_LICENSE("GPL");
