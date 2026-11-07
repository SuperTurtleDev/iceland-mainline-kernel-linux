// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm RPMh on/off resource regulator.
 *
 * Some PMIC resources exposed through RPMh, such as the bidirectional GPIO
 * level shifters of PMH0101, only support being switched on or off. Each is
 * described as its own regulator node under the RSC and voted on by name.
 *
 * Votes are sent without waiting for the resource to acknowledge them: the
 * request completes once RPMh has accepted it, so a resource that never
 * answers cannot tie up a transmit slot of the shared RSC.
 */
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <soc/qcom/cmd-db.h>
#include <soc/qcom/rpmh.h>
#include <soc/qcom/tcs.h>

#define RPMH_XOB_REG_ENABLE	0x4

struct rpmh_xob {
	struct device *dev;
	u32 addr;
	bool enabled;
};

static int rpmh_xob_set(struct rpmh_xob *xob, bool enable)
{
	struct tcs_cmd cmd = {
		.addr = xob->addr + RPMH_XOB_REG_ENABLE,
		.data = enable,
	};
	int ret;

	ret = rpmh_write_async(xob->dev, RPMH_ACTIVE_ONLY_STATE, &cmd, 1);
	if (ret)
		return ret;

	xob->enabled = enable;

	return 0;
}

static int rpmh_xob_enable(struct regulator_dev *rdev)
{
	return rpmh_xob_set(rdev_get_drvdata(rdev), true);
}

static int rpmh_xob_disable(struct regulator_dev *rdev)
{
	return rpmh_xob_set(rdev_get_drvdata(rdev), false);
}

static int rpmh_xob_is_enabled(struct regulator_dev *rdev)
{
	struct rpmh_xob *xob = rdev_get_drvdata(rdev);

	return xob->enabled;
}

static const struct regulator_ops rpmh_xob_ops = {
	.enable = rpmh_xob_enable,
	.disable = rpmh_xob_disable,
	.is_enabled = rpmh_xob_is_enabled,
};

static int rpmh_xob_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regulator_config config = { .dev = dev, .of_node = dev->of_node };
	struct regulator_desc *desc;
	struct regulator_dev *rdev;
	const char *resource;
	struct rpmh_xob *xob;
	int ret;

	xob = devm_kzalloc(dev, sizeof(*xob), GFP_KERNEL);
	if (!xob)
		return -ENOMEM;

	xob->dev = dev;

	ret = device_property_read_string(dev, "qcom,resource-name", &resource);
	if (ret)
		return dev_err_probe(dev, ret, "missing resource name\n");

	xob->addr = cmd_db_read_addr(resource);
	if (!xob->addr)
		return dev_err_probe(dev, -ENODEV, "unknown RPMh resource %s\n",
				     resource);

	desc = devm_kzalloc(dev, sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;

	desc->name = resource;
	desc->type = REGULATOR_VOLTAGE;
	desc->owner = THIS_MODULE;
	desc->ops = &rpmh_xob_ops;

	config.init_data = of_get_regulator_init_data(dev, dev->of_node, desc);
	if (!config.init_data)
		return -ENOMEM;
	config.driver_data = xob;

	rdev = devm_regulator_register(dev, desc, &config);
	if (IS_ERR(rdev))
		return dev_err_probe(dev, PTR_ERR(rdev),
				     "failed to register %s\n", resource);

	return 0;
}

static const struct of_device_id rpmh_xob_of_match[] = {
	{ .compatible = "qcom,rpmh-xob-regulator" },
	{ }
};
MODULE_DEVICE_TABLE(of, rpmh_xob_of_match);

static struct platform_driver rpmh_xob_driver = {
	.driver = {
		.name = "qcom-rpmh-xob-regulator",
		.of_match_table = rpmh_xob_of_match,
	},
	.probe = rpmh_xob_probe,
};
module_platform_driver(rpmh_xob_driver);

MODULE_DESCRIPTION("Qualcomm RPMh on/off resource regulator driver");
MODULE_LICENSE("GPL");
