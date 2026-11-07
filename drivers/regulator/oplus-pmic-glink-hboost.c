// SPDX-License-Identifier: GPL-2.0-only
/*
 * Oplus PMIC GLINK HBOOST regulator.
 *
 * On Oplus devices built around a Qualcomm PMIC GLINK charger, the PMIC
 * HBOOST rail feeding the pencil wireless charging transmitter is programmed
 * by the charger firmware. The message format follows the Oplus
 * wireless_pen/oplus_wireless_pen_glink.[ch] driver,
 * Copyright (C) 2020-2022 Oplus. All rights reserved.
 *
 * The firmware offers neither readback nor enable control: the rail is
 * always on and only its voltage can be set, in 50 mV steps from 2 V.
 *
 * Probe waits for the firmware service and registers the regulator
 * synchronously, so that the registration-time constraint voltage is
 * applied, consumers deferred on the supply are retried by the driver core
 * when this device binds, and every resource is torn down by devres in
 * order. A firmware restart brings the rail back to its default voltage;
 * the last acknowledged selector is programmed again when the service
 * returns.
 */
#include <linux/auxiliary_bus.h>
#include <linux/cleanup.h>
#include <linux/completion.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/soc/qcom/pdr.h>
#include <linux/soc/qcom/pmic_glink.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#define HBOOST_OWNER_ID			32785
#define HBOOST_OPCODE_SET_VOUT		0x10007
#define HBOOST_RESPONSE_TIMEOUT_MS	2500
/*
 * The charger service normally comes up within seconds of boot. Probing is
 * asynchronous, so waiting this long costs nothing; giving up earlier would
 * leave the retry to the driver core's deferred-probe triggers, which only
 * fire when some other driver binds.
 */
#define HBOOST_SERVICE_TIMEOUT_MS	60000
/* Fast attempts to restore the voltage after a firmware restart, then slow ones. */
#define HBOOST_RESYNC_RETRIES		5
#define HBOOST_RESYNC_RETRY_MS		1000
#define HBOOST_RESYNC_SLOW_MS		30000

#define HBOOST_MIN_UV			2000000
#define HBOOST_STEP_UV			50000
#define HBOOST_N_VOLTAGES		256

struct hboost_set_vout_req {
	struct pmic_glink_hdr hdr;
	u8 selector;
	u8 reserved[3];
};

struct hboost_set_vout_resp {
	struct pmic_glink_hdr hdr;
	__le32 status;
};

struct hboost {
	struct device *dev;
	struct pmic_glink_client *client;
	struct workqueue_struct *wq;
	struct delayed_work resync_work;
	unsigned int resync_attempts;
	struct completion service_up;

	/*
	 * One request in flight. Also covers the decision of what to send,
	 * so a restart recovery cannot race a consumer's newer request.
	 */
	struct mutex request_lock;

	/*
	 * Protects the fields below, shared with the GLINK callbacks that run
	 * in atomic context. Responses carry no transaction ID: a response is
	 * attributed to the pending request, a response with none pending is
	 * dropped. A response to a request that already timed out, arriving
	 * while the next request is pending, is indistinguishable from that
	 * request's own response; the firmware is expected to answer in order.
	 */
	spinlock_t lock;
	struct completion response;
	bool pending;
	int result;
	bool online;
	bool registered;
	bool removing;
	/* Bumped on every service down, invalidating in-flight requests. */
	unsigned int generation;
	/* Last selector the firmware acknowledged, negative when unknown. */
	int selector;
	/* The firmware restarted since the selector was acknowledged. */
	bool stale;
};

static void hboost_callback(const void *data, size_t len, void *priv)
{
	const struct hboost_set_vout_resp *resp = data;
	struct hboost *hb = priv;

	if (len < sizeof(*resp) ||
	    le32_to_cpu(resp->hdr.opcode) != HBOOST_OPCODE_SET_VOUT)
		return;

	guard(spinlock_irqsave)(&hb->lock);
	if (!hb->pending)
		return;

	hb->pending = false;
	hb->result = le32_to_cpu(resp->status) ? -EIO : 0;
	complete(&hb->response);
}

/* Must be called with hb->lock held. */
static void hboost_queue_resync(struct hboost *hb, unsigned long delay)
{
	if (hb->registered && hb->stale && hb->online && !hb->removing)
		mod_delayed_work(hb->wq, &hb->resync_work, delay);
}

static void hboost_pdr_notify(void *priv, int state)
{
	struct hboost *hb = priv;
	bool online = state == SERVREG_SERVICE_STATE_UP;

	guard(spinlock_irqsave)(&hb->lock);
	if (hb->online == online)
		return;

	hb->online = online;
	if (online) {
		hb->resync_attempts = 0;
		complete(&hb->service_up);
		hboost_queue_resync(hb, 0);
		return;
	}

	/* The firmware restarts with its default voltage; abort any waiter. */
	hb->generation++;
	hb->stale = true;
	if (hb->pending) {
		hb->pending = false;
		hb->result = -ENOTCONN;
		complete(&hb->response);
	}
}

/* Must be called with hb->request_lock held. */
static int hboost_send(struct hboost *hb, unsigned int selector)
{
	struct hboost_set_vout_req req = {
		.hdr.owner = cpu_to_le32(HBOOST_OWNER_ID),
		.hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
		.hdr.opcode = cpu_to_le32(HBOOST_OPCODE_SET_VOUT),
		.selector = selector,
	};
	unsigned int generation;
	int ret;

	scoped_guard(spinlock_irqsave, &hb->lock) {
		if (hb->removing)
			return -ENODEV;
		if (!hb->online)
			return -EAGAIN;
		generation = hb->generation;
		reinit_completion(&hb->response);
		hb->pending = true;
	}

	ret = pmic_glink_send(hb->client, &req, sizeof(req));
	if (!ret && !wait_for_completion_timeout(&hb->response,
						 msecs_to_jiffies(HBOOST_RESPONSE_TIMEOUT_MS)))
		ret = -ETIMEDOUT;

	guard(spinlock_irqsave)(&hb->lock);
	if (ret) {
		hb->pending = false;
		return ret;
	}

	if (hb->result)
		return hb->result;

	/* A service restart during the exchange leaves the rail at its default. */
	if (generation != hb->generation)
		return -ENOTCONN;

	hb->selector = selector;
	hb->stale = false;

	return 0;
}

static int hboost_set_voltage_sel(struct regulator_dev *rdev, unsigned int selector)
{
	struct hboost *hb = rdev_get_drvdata(rdev);
	int ret;

	scoped_guard(mutex, &hb->request_lock)
		ret = hboost_send(hb, selector);

	if (ret && ret != -EAGAIN)
		dev_err(hb->dev, "failed to set selector %u: %d\n", selector, ret);

	return ret;
}

static int hboost_get_voltage_sel(struct regulator_dev *rdev)
{
	struct hboost *hb = rdev_get_drvdata(rdev);

	guard(spinlock_irqsave)(&hb->lock);

	/* Report the requested selector while a restart is being recovered. */
	return hb->selector < 0 ? -ENOTRECOVERABLE : hb->selector;
}

static const struct regulator_ops hboost_ops = {
	.list_voltage = regulator_list_voltage_linear,
	.map_voltage = regulator_map_voltage_linear,
	.set_voltage_sel = hboost_set_voltage_sel,
	.get_voltage_sel = hboost_get_voltage_sel,
};

static const struct regulator_desc hboost_desc = {
	.name = "hboost",
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.ops = &hboost_ops,
	.min_uV = HBOOST_MIN_UV,
	.uV_step = HBOOST_STEP_UV,
	.n_voltages = HBOOST_N_VOLTAGES,
};

static void hboost_resync_work(struct work_struct *work)
{
	struct hboost *hb = container_of(to_delayed_work(work), struct hboost,
					 resync_work);
	int selector;
	int ret;

	guard(mutex)(&hb->request_lock);

	/* Decided under the request lock: a newer consumer request wins. */
	scoped_guard(spinlock_irqsave, &hb->lock) {
		if (!hb->stale || hb->selector < 0)
			return;
		selector = hb->selector;
	}

	ret = hboost_send(hb, selector);
	if (!ret || ret == -ENODEV)
		return;

	/*
	 * Keep trying while the rail is known to be off target: the regulator
	 * core answers a repeated request for the same range from its cache,
	 * so no consumer action would otherwise reprogram the firmware.
	 */
	guard(spinlock_irqsave)(&hb->lock);
	if (hb->resync_attempts < HBOOST_RESYNC_RETRIES) {
		hb->resync_attempts++;
		hboost_queue_resync(hb, msecs_to_jiffies(HBOOST_RESYNC_RETRY_MS));
		return;
	}

	if (hb->resync_attempts == HBOOST_RESYNC_RETRIES) {
		hb->resync_attempts++;
		dev_err(hb->dev, "failed to restore selector %d after firmware restart: %d\n",
			selector, ret);
	}
	hboost_queue_resync(hb, msecs_to_jiffies(HBOOST_RESYNC_SLOW_MS));
}

/* Runs before the GLINK client is released: stop queuing and drain the work. */
static void hboost_stop(void *data)
{
	struct hboost *hb = data;

	scoped_guard(spinlock_irqsave, &hb->lock)
		hb->removing = true;

	cancel_delayed_work_sync(&hb->resync_work);
}

static int hboost_register(struct hboost *hb)
{
	struct device *dev = hb->dev;
	struct regulator_config config = { .dev = dev, .driver_data = hb };
	struct regulator_dev *rdev;

	/* The auxiliary device shares the pmic-glink node; the rail is referenced from it. */
	struct device_node *np __free(device_node) =
		of_parse_phandle(dev->of_node, "oplus,hboost-regulator", 0);
	if (!np)
		return -ENODEV;

	config.of_node = np;
	config.init_data = of_get_regulator_init_data(dev, np, &hboost_desc);
	if (!config.init_data)
		return -ENOMEM;

	/* The unknown voltage makes the core apply the constraints now. */
	rdev = devm_regulator_register(dev, &hboost_desc, &config);
	if (IS_ERR(rdev))
		return PTR_ERR(rdev);

	/* A restart during registration is recovered like any other. */
	guard(spinlock_irqsave)(&hb->lock);
	hb->registered = true;
	hboost_queue_resync(hb, 0);

	return 0;
}

static int hboost_probe(struct auxiliary_device *adev,
			const struct auxiliary_device_id *id)
{
	struct device *dev = &adev->dev;
	struct hboost *hb;
	int ret;

	hb = devm_kzalloc(dev, sizeof(*hb), GFP_KERNEL);
	if (!hb)
		return -ENOMEM;

	hb->dev = dev;
	hb->selector = -1;
	spin_lock_init(&hb->lock);
	init_completion(&hb->response);
	init_completion(&hb->service_up);
	INIT_DELAYED_WORK(&hb->resync_work, hboost_resync_work);
	ret = devm_mutex_init(dev, &hb->request_lock);
	if (ret)
		return ret;

	/* One long-sleeping work item: an unbound worker, frozen across suspend. */
	hb->wq = devm_alloc_workqueue(dev, "%s", WQ_UNBOUND | WQ_FREEZABLE, 1,
				      dev_name(dev));
	if (!hb->wq)
		return -ENOMEM;

	hb->client = devm_pmic_glink_client_alloc(dev, HBOOST_OWNER_ID,
						  hboost_callback,
						  hboost_pdr_notify, hb);
	if (IS_ERR(hb->client))
		return PTR_ERR(hb->client);

	ret = devm_add_action_or_reset(dev, hboost_stop, hb);
	if (ret)
		return ret;

	pmic_glink_client_register(hb->client);

	if (!wait_for_completion_timeout(&hb->service_up,
					 msecs_to_jiffies(HBOOST_SERVICE_TIMEOUT_MS)))
		return dev_err_probe(dev, -EPROBE_DEFER, "firmware service not up\n");

	/*
	 * Anything but a resource or description error is the firmware link
	 * failing to apply the initial voltage (service gone, endpoint reset,
	 * timeout, rejection), which is transient.
	 */
	ret = hboost_register(hb);
	if (ret == -ENOMEM || ret == -ENODEV || ret == -EINVAL)
		return dev_err_probe(dev, ret, "failed to register regulator\n");
	if (ret)
		return dev_err_probe(dev, -EPROBE_DEFER,
				     "failed to set initial voltage: %d\n", ret);

	return 0;
}

static const struct auxiliary_device_id hboost_id_table[] = {
	{ .name = "pmic_glink.hboost" },
	{ }
};
MODULE_DEVICE_TABLE(auxiliary, hboost_id_table);

static struct auxiliary_driver hboost_driver = {
	.driver = {
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.name = "oplus_pmic_glink_hboost",
	.probe = hboost_probe,
	.id_table = hboost_id_table,
};
module_auxiliary_driver(hboost_driver);

MODULE_DESCRIPTION("Oplus PMIC GLINK HBOOST regulator driver");
MODULE_LICENSE("GPL");
