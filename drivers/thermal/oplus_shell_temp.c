// SPDX-License-Identifier: GPL-2.0-only
/*
 * OnePlus shell temperature estimator.
 *
 * The stock Android thermal daemon (Horae) estimates the device shell
 * temperature from a few board thermistors and the battery with a linear
 * model over the last four samples, taken five seconds apart:
 *
 *	Y = sum_k sum_i C[k][i] * T[k][i] + b
 *
 * where k runs over the sample history, newest first, and i over the model
 * inputs. Several models may describe one surface; the surface reports the
 * largest. Until the history is complete a model reports the battery
 * temperature plus a bias, and a gap of more than the reset interval
 * between samples starts the history over. Model shape and parameters come
 * from the device tree, in micro units so the kernel needs no floating
 * point. The daemon's input jump limiting, sensor fault detection and
 * output smoothing are not reproduced. Input zones are looked up by name
 * and must outlive this device; on this board they are built from the
 * device tree and never go away.
 */
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/thermal.h>
#include <linux/workqueue.h>

#define SHELL_MAX_ZONES		8
#define SHELL_MAX_INPUTS	8
#define SHELL_MAX_HISTORY	4
#define SHELL_MAX_MODELS	4
#define SHELL_SCALE		1000000LL

struct shell_model {
	const char *name;
	unsigned int ninputs;
	unsigned int history;
	u8 input[SHELL_MAX_INPUTS];
	s32 coeff[SHELL_MAX_HISTORY][SHELL_MAX_INPUTS];	/* micro */
	s64 constant_uc;
	int bat_bias_mc;
	int samples[SHELL_MAX_HISTORY][SHELL_MAX_INPUTS];	/* mC, newest first */
	unsigned int filled;
};

struct shell_temp {
	struct device *dev;
	struct thermal_zone_device *zone[SHELL_MAX_ZONES];
	unsigned int nzones;
	struct power_supply *battery;
	struct thermal_zone_device *tz;
	struct delayed_work work;
	struct mutex lock;
	unsigned int period_ms;
	unsigned int reset_ms;
	struct shell_model model[SHELL_MAX_MODELS];
	unsigned int nmodels;
	u64 last_ms;
	int temp_mc;
	bool valid;
};

static int shell_read_inputs(struct shell_temp *st, int *in, int *bat_mc)
{
	union power_supply_propval val;
	int i, ret;

	for (i = 0; i < st->nzones; i++) {
		ret = thermal_zone_get_temp(st->zone[i], &in[i]);
		if (ret)
			return ret;
	}
	ret = power_supply_get_property(st->battery, POWER_SUPPLY_PROP_TEMP, &val);
	if (ret)
		return ret;
	*bat_mc = val.intval * 100;
	in[st->nzones] = *bat_mc;
	return 0;
}

static void shell_model_push(struct shell_model *m, const int *in)
{
	int k, i;

	for (k = m->history - 1; k > 0; k--)
		memcpy(m->samples[k], m->samples[k - 1], sizeof(m->samples[k]));
	for (i = 0; i < m->ninputs; i++)
		m->samples[0][i] = in[m->input[i]];
	if (m->filled < m->history)
		m->filled++;
}

/* The daemon computes in floating point and converts to an integer. */
static int shell_model_output(const struct shell_model *m, int bat_mc)
{
	s64 acc = 0;
	int k, i;

	if (m->filled < m->history)
		return bat_mc + m->bat_bias_mc;
	for (k = 0; k < m->history; k++)
		for (i = 0; i < m->ninputs; i++)
			acc += (s64)m->coeff[k][i] * m->samples[k][i];
	acc += m->constant_uc;
	return (int)div_s64(acc, SHELL_SCALE);
}

static void shell_work(struct work_struct *work)
{
	struct shell_temp *st = container_of(to_delayed_work(work), struct shell_temp, work);
	int in[SHELL_MAX_ZONES + 1], bat_mc, out, best = INT_MIN;
	u64 now = ktime_to_ms(ktime_get_boottime());
	int i, ret;

	ret = shell_read_inputs(st, in, &bat_mc);
	mutex_lock(&st->lock);
	if (ret) {
		st->valid = false;
		goto out;
	}
	if (st->last_ms && now - st->last_ms > st->reset_ms)
		for (i = 0; i < st->nmodels; i++)
			st->model[i].filled = 0;
	st->last_ms = now;
	for (i = 0; i < st->nmodels; i++) {
		shell_model_push(&st->model[i], in);
		out = shell_model_output(&st->model[i], bat_mc);
		best = max(best, out);
	}
	st->temp_mc = best;
	st->valid = true;
out:
	mutex_unlock(&st->lock);
	if (!ret && st->tz)
		thermal_zone_device_update(st->tz, THERMAL_EVENT_UNSPECIFIED);
	schedule_delayed_work(&st->work, msecs_to_jiffies(st->period_ms));
}

static int shell_get_temp(struct thermal_zone_device *tz, int *temp)
{
	struct shell_temp *st = thermal_zone_device_priv(tz);
	int ret = 0;

	mutex_lock(&st->lock);
	if (st->valid)
		*temp = st->temp_mc;
	else
		ret = -ENODATA;
	mutex_unlock(&st->lock);
	return ret;
}

static const struct thermal_zone_device_ops shell_ops = {
	.get_temp = shell_get_temp,
};

static int shell_parse_model(struct shell_temp *st, struct device_node *np, struct shell_model *m)
{
	u32 hist, v;
	int n, ret, k, i;
	s64 c;

	m->name = np->name;
	n = of_property_count_u32_elems(np, "oplus,inputs");
	if (n < 1 || n > SHELL_MAX_INPUTS)
		return dev_err_probe(st->dev, -EINVAL, "%pOFn: inputs\n", np);
	m->ninputs = n;
	for (i = 0; i < n; i++) {
		ret = of_property_read_u32_index(np, "oplus,inputs", i, &v);
		if (ret || v > st->nzones)
			return dev_err_probe(st->dev, -EINVAL, "%pOFn: input %d\n", np, i);
		m->input[i] = v;
	}
	ret = of_property_read_u32(np, "oplus,history", &hist);
	if (ret || hist < 1 || hist > SHELL_MAX_HISTORY)
		return dev_err_probe(st->dev, -EINVAL, "%pOFn: history\n", np);
	m->history = hist;
	n = of_property_count_u32_elems(np, "oplus,coefficients-micro");
	if (n != (int)(hist * m->ninputs))
		return dev_err_probe(st->dev, -EINVAL, "%pOFn: %u coefficients expected\n", np,
				     hist * m->ninputs);
	for (k = 0; k < hist; k++)
		for (i = 0; i < m->ninputs; i++) {
			of_property_read_u32_index(np, "oplus,coefficients-micro",
						   k * m->ninputs + i, &v);
			m->coeff[k][i] = (s32)v;
		}
	ret = of_property_read_u32(np, "oplus,constant-microcelsius", &v);
	if (ret)
		return dev_err_probe(st->dev, ret, "%pOFn: constant\n", np);
	c = (s32)v;
	m->constant_uc = c * 1000;	/* micro-celsius to micro-millicelsius */
	ret = of_property_read_u32(np, "oplus,battery-bias-millicelsius", &v);
	if (ret)
		return dev_err_probe(st->dev, ret, "%pOFn: battery bias\n", np);
	m->bat_bias_mc = (s32)v;
	return 0;
}

static void shell_put_battery(void *psy)
{
	power_supply_put(psy);
}

static void shell_stop(void *data)
{
	struct shell_temp *st = data;

	cancel_delayed_work_sync(&st->work);
}

static int shell_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node, *child;
	struct shell_temp *st;
	const char *name;
	int n, i, ret;

	st = devm_kzalloc(dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;
	st->dev = dev;
	mutex_init(&st->lock);
	n = of_property_count_strings(np, "oplus,thermal-zones");
	if (n < 1 || n > SHELL_MAX_ZONES)
		return dev_err_probe(dev, -EINVAL, "oplus,thermal-zones\n");
	for (i = 0; i < n; i++) {
		of_property_read_string_index(np, "oplus,thermal-zones", i, &name);
		st->zone[i] = thermal_zone_get_zone_by_name(name);
		if (IS_ERR(st->zone[i]))
			return dev_err_probe(dev, -EPROBE_DEFER, "thermal zone %s\n", name);
	}
	st->nzones = n;
	ret = of_property_read_string(np, "oplus,battery", &name);
	if (ret)
		return dev_err_probe(dev, ret, "oplus,battery\n");
	st->battery = power_supply_get_by_name(name);
	if (!st->battery)
		return -EPROBE_DEFER;
	ret = devm_add_action_or_reset(dev, shell_put_battery, st->battery);
	if (ret)
		return ret;
	st->period_ms = 5000;
	st->reset_ms = 8000;
	of_property_read_u32(np, "oplus,period-ms", &st->period_ms);
	of_property_read_u32(np, "oplus,history-reset-ms", &st->reset_ms);
	if (st->period_ms < 500 || st->reset_ms < st->period_ms)
		return dev_err_probe(dev, -EINVAL, "period or reset interval\n");
	for_each_child_of_node(np, child) {
		if (st->nmodels == SHELL_MAX_MODELS) {
			of_node_put(child);
			return dev_err_probe(dev, -EINVAL, "too many models\n");
		}
		ret = shell_parse_model(st, child, &st->model[st->nmodels]);
		if (ret) {
			of_node_put(child);
			return ret;
		}
		st->nmodels++;
	}
	if (!st->nmodels)
		return dev_err_probe(dev, -EINVAL, "no model\n");
	INIT_DELAYED_WORK(&st->work, shell_work);
	st->tz = devm_thermal_of_zone_register(dev, 0, st, &shell_ops);
	if (IS_ERR(st->tz))
		return dev_err_probe(dev, PTR_ERR(st->tz), "thermal zone\n");
	/* Registered after the zone, so it runs before the zone goes away. */
	ret = devm_add_action_or_reset(dev, shell_stop, st);
	if (ret)
		return ret;
	schedule_delayed_work(&st->work, 0);
	return 0;
}

static const struct of_device_id shell_of_match[] = {
	{ .compatible = "oplus,shell-temperature" },
	{}
};
MODULE_DEVICE_TABLE(of, shell_of_match);

static struct platform_driver shell_driver = {
	.probe = shell_probe,
	.driver = {
		.name = "oplus-shell-temperature",
		.of_match_table = shell_of_match,
	},
};
module_platform_driver(shell_driver);

MODULE_DESCRIPTION("OnePlus shell temperature estimator");
MODULE_LICENSE("GPL");
