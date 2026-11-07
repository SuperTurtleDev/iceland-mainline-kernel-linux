// SPDX-License-Identifier: GPL-2.0-only
/*
 * OnePlus/Oplus pencil wireless charging transmitter (MT5806).
 *
 * Register map, interrupt bits, packet formats and the charging policy
 * follow the Oplus wireless_pen/oplus_mt5806.[ch] driver,
 * Copyright (C) 2025 Oplus. All rights reserved.
 *
 * The transmitter detects a pencil docked on the tablet's magnetic strip,
 * receives the pencil's identity (Bluetooth address and ID) over the
 * wireless power link and charges it from the PMIC HBOOST rail. Bluetooth
 * itself is left to userspace, which feeds the pencil's state of charge
 * back through the writable capacity property so that a full pencil is
 * topped up again once it has drained far enough.
 */
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/if_ether.h>
#include <linux/interrupt.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm_wakeirq.h>
#include <linux/pm_wakeup.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

/* 16-bit big-endian register address, 8-bit data, multi-byte fields little-endian. */
#define MT5806_REG_FW_MINOR		0x0002
#define MT5806_REG_FW_MAJOR		0x0006
#define MT5806_REG_CMD			0x0008
#define MT5806_REG_CMD_SYNC		0x000f
#define MT5806_REG_IRQ_STATUS		0x0014
#define MT5806_REG_IRQ_CLEAR		0x0018
#define MT5806_REG_PACKET		0x0020
#define MT5806_REG_EPT_TYPE		0x00b0
#define MT5806_REG_CHIP_ID		0x166c

#define MT5806_CHIP_ID			0x5806
#define MT5806_CMD_SYNC_MAGIC		0x55

/* Command register bit numbers; a command is latched by the sync write. */
#define MT5806_CMD_CLEAR_IRQ		5
#define MT5806_CMD_CLEAR_EPT		14

/* Interrupt status bits */
#define MT5806_IRQ_RX_REMOVED		BIT(6)
#define MT5806_IRQ_INIT_DONE		BIT(9)
#define MT5806_IRQ_PP_PACKET		BIT(11)
#define MT5806_IRQ_EPT			BIT(15)
#define MT5806_IRQ_RX_ATTACHED		BIT(25)
#define MT5806_IRQ_MISPLACED_CLEAR	BIT(26)
#define MT5806_IRQ_MISPLACED		BIT(28)

/* End-of-power-transfer faults in bytes 1 and 2 of MT5806_REG_EPT_TYPE */
#define MT5806_EPT1_FAULTS		GENMASK(6, 2)	/* OCP, OVP, LVP, FOD, OTP */
#define MT5806_EPT2_FAULTS		GENMASK(2, 1)	/* ping OVP, ping OCP */

/* Proprietary packets received from the pencil */
#define MT5806_PACKET_LEN		9
#define MT5806_PKT_IDENTITY		0x48
#define MT5806_PKT_IDENTITY_CHECK	0xc1
#define MT5806_PKT_IDENTITY_ADDR0	0xb6
#define MT5806_PKT_IDENTITY_ADDR1	0xb7
#define MT5806_PKT_POWER		0x28
#define MT5806_PKT_POWER_STOP		0x17
#define MT5806_ADDR_PART_LEN		5

#define MT5806_POWER_CYCLE_MS		1000
#define MT5806_BOOT_MS			2500
#define MT5806_CHIP_ID_RETRIES		5
#define MT5806_CHIP_ID_RETRY_MS		100
#define MT5806_BOOST_RETRIES		60
#define MT5806_BOOST_RETRY_MS		500
#define MT5806_IDENTITY_TIMEOUT_MS	5000
#define MT5806_INHIBIT_DELAY_MS		100
#define MT5806_MISPLACED_COOLDOWN_MS	300
#define MT5806_FULL_SOC			100

#define MT5806_DEFAULT_HBOOST_UV	5800000
#define MT5806_DEFAULT_RECHARGE_SOC	90
#define MT5806_DEFAULT_TIMEOUT_SEC	(120 * 60)

enum mt5806_stop_reason {
	MT5806_STOP_NONE,
	MT5806_STOP_FULL,
	MT5806_STOP_RECEIVER,
	MT5806_STOP_IDENTITY,
	MT5806_STOP_TIMEOUT,
	MT5806_STOP_FAULT,
};

/* Identity packets arrive as a check packet followed by two address halves. */
struct mt5806_identity {
	u8 addr[2 * MT5806_ADDR_PART_LEN];
	u16 check;
	u8 id;
	u8 parts;
};

struct mt5806 {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	struct regulator *hboost;
	struct gpio_desc *power_gpio;		/* transmitter supply switch */
	struct gpio_desc *wakeup_gpio;		/* hold the transmitter awake */
	struct gpio_desc *scan_gpio;		/* fast pencil scan rate */
	struct gpio_desc *charge_off_gpio;	/* stop power transfer */
	int irq;

	/* Serializes interrupt handling, work items, sysfs and PM callbacks. */
	struct mutex lock;
	/* Private queue for the long-running HBOOST request; timers stay off it. */
	struct workqueue_struct *boost_wq;
	struct work_struct boost_work;
	struct delayed_work identity_work;
	struct delayed_work timeout_work;
	/* Held from the start of a charging session until the pencil is identified. */
	struct wakeup_source *identity_ws;

	u32 hboost_idle_uv;
	u32 hboost_charge_uv;
	u32 hboost_target_uv;
	u32 recharge_soc;
	u32 charge_timeout_ms;

	struct mt5806_identity identity;
	ktime_t charge_start;
	ktime_t identity_deadline;
	ktime_t misplaced_stamp;
	enum mt5806_stop_reason stop_reason;
	u64 ble_addr;
	u16 fw_major;
	u16 fw_minor;
	u8 pencil_id;
	int soc;
	bool attached;
	bool present;
	bool charging;
	bool recharging;
	bool inhibit;
	bool misplaced;
	bool fast_scan;
	/* Set once the driver is going away; no new sessions or work after that. */
	bool stopping;
};

static const struct regmap_config mt5806_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0xffff,
};

static int mt5806_command(struct mt5806 *chip, unsigned int bit)
{
	int ret;

	ret = regmap_set_bits(chip->regmap, MT5806_REG_CMD + bit / BITS_PER_BYTE,
			      BIT(bit % BITS_PER_BYTE));
	if (ret)
		return ret;

	return regmap_write(chip->regmap, MT5806_REG_CMD_SYNC, MT5806_CMD_SYNC_MAGIC);
}

/* Must be called with chip->lock held. */
static void mt5806_set_charging(struct mt5806 *chip, bool on)
{
	chip->charging = on;
	gpiod_set_value_cansleep(chip->charge_off_gpio, !on);
}

/*
 * End the identity wait: the pencil identified itself, charging stopped for
 * another reason, the pencil left, or the deadline passed. Must be called
 * with chip->lock held.
 */
static void mt5806_identity_wait_end(struct mt5806 *chip)
{
	cancel_delayed_work(&chip->identity_work);
	__pm_relax(chip->identity_ws);
}

/* Must be called with chip->lock held. */
static void mt5806_stop_charging(struct mt5806 *chip, enum mt5806_stop_reason reason)
{
	mt5806_set_charging(chip, false);
	chip->stop_reason = reason;
	mt5806_identity_wait_end(chip);
}

/* Start the session time limit. Must be called with chip->lock held. */
static void mt5806_start_session(struct mt5806 *chip)
{
	chip->charge_start = ktime_get_boottime();
	mod_delayed_work(system_freezable_wq, &chip->timeout_work,
			 msecs_to_jiffies(chip->charge_timeout_ms));
}

/*
 * Wait for the pencil to identify itself. The OEM polls 50 x 100 ms holding a
 * wakeup source and exits early once identified or stopped; here the deadline
 * is absolute so that a callback left over from an earlier session cannot
 * end a newer one. Must be called with chip->lock held.
 */
static void mt5806_start_identity_wait(struct mt5806 *chip)
{
	chip->identity_deadline = ktime_add_ms(ktime_get_boottime(),
					       MT5806_IDENTITY_TIMEOUT_MS);
	__pm_stay_awake(chip->identity_ws);
	mod_delayed_work(system_freezable_wq, &chip->identity_work,
			 msecs_to_jiffies(MT5806_IDENTITY_TIMEOUT_MS));
}

/* Must be called with chip->lock held. */
static void mt5806_check_timeout(struct mt5806 *chip)
{
	if (!chip->attached || !chip->charging)
		return;

	if (ktime_ms_delta(ktime_get_boottime(), chip->charge_start) <
	    chip->charge_timeout_ms)
		return;

	dev_info(chip->dev, "charging time limit reached\n");
	mt5806_stop_charging(chip, MT5806_STOP_TIMEOUT);
	power_supply_changed(chip->psy);
}

static void mt5806_timeout_work(struct work_struct *work)
{
	struct mt5806 *chip = container_of(to_delayed_work(work), struct mt5806,
					   timeout_work);

	guard(mutex)(&chip->lock);
	mt5806_check_timeout(chip);
}

static void mt5806_identity_work(struct work_struct *work)
{
	struct mt5806 *chip = container_of(to_delayed_work(work), struct mt5806,
					   identity_work);

	guard(mutex)(&chip->lock);

	/* A newer session re-armed the wait while this callback was waiting for the lock. */
	if (ktime_before(ktime_get_boottime(), chip->identity_deadline))
		return;

	__pm_relax(chip->identity_ws);

	/* Nothing to do once identified, or if charging already stopped for another reason. */
	if (!chip->attached || chip->present || !chip->charging)
		return;

	dev_warn(chip->dev, "no pencil identity received, charging stopped\n");
	mt5806_stop_charging(chip, MT5806_STOP_IDENTITY);
	power_supply_changed(chip->psy);
}

static int mt5806_set_hboost(struct mt5806 *chip, u32 uv)
{
	unsigned int attempt;
	int ret;

	for (attempt = 0; attempt < MT5806_BOOST_RETRIES; attempt++) {
		ret = regulator_set_voltage(chip->hboost, uv, uv);
		if (!ret)
			return 0;
		if (READ_ONCE(chip->stopping))
			return -ENODEV;
		msleep(MT5806_BOOST_RETRY_MS);
	}

	dev_err(chip->dev, "failed to set HBOOST to %u uV: %d\n", uv, ret);

	return ret;
}

static void mt5806_boost_work(struct work_struct *work)
{
	struct mt5806 *chip = container_of(work, struct mt5806, boost_work);
	u32 uv;
	int ret;

	scoped_guard(mutex, &chip->lock)
		uv = chip->hboost_target_uv;

	/* The retries may span tens of seconds; keep the system from suspending. */
	pm_stay_awake(chip->dev);
	ret = mt5806_set_hboost(chip, uv);
	pm_relax(chip->dev);
	if (ret)
		return;

	/*
	 * Power transfer is released only once the rail is at the requested
	 * level, and only if nothing stopped charging while the request was
	 * in flight: a fault, a failed identity, a stop packet or an inhibit.
	 */
	guard(mutex)(&chip->lock);
	if (uv != chip->hboost_target_uv || chip->stopping ||
	    chip->stop_reason != MT5806_STOP_NONE ||
	    (chip->present && chip->inhibit))
		return;

	mt5806_set_charging(chip, true);
}

/* Must be called with chip->lock held. */
static void mt5806_request_hboost(struct mt5806 *chip, u32 uv)
{
	chip->hboost_target_uv = uv;
	queue_work(chip->boost_wq, &chip->boost_work);
}

/*
 * Power-cycle the transmitter and wait for its firmware to boot. The wakeup
 * line must be asserted by the caller for the firmware to accept register
 * access afterwards.
 */
static void mt5806_power_cycle(struct mt5806 *chip)
{
	gpiod_set_value_cansleep(chip->power_gpio, 0);
	msleep(MT5806_POWER_CYCLE_MS);
	gpiod_set_value_cansleep(chip->power_gpio, 1);
	msleep(MT5806_BOOT_MS);
}

/* Must be called with chip->lock held. */
static void mt5806_attached(struct mt5806 *chip)
{
	chip->attached = true;
	chip->recharging = false;
	chip->stop_reason = MT5806_STOP_NONE;
	mt5806_request_hboost(chip, chip->hboost_charge_uv);
	mt5806_start_session(chip);
	mt5806_start_identity_wait(chip);
	power_supply_changed(chip->psy);
}

/* Must be called with chip->lock held. */
static void mt5806_removed(struct mt5806 *chip)
{
	bool faulted = chip->stop_reason == MT5806_STOP_FAULT;

	gpiod_set_value_cansleep(chip->scan_gpio, chip->fast_scan);
	mt5806_identity_wait_end(chip);
	cancel_delayed_work(&chip->timeout_work);

	chip->attached = false;
	chip->present = false;
	chip->recharging = false;
	chip->stop_reason = MT5806_STOP_NONE;
	chip->ble_addr = 0;
	chip->pencil_id = 0;
	chip->soc = -1;
	memset(&chip->identity, 0, sizeof(chip->identity));

	mt5806_request_hboost(chip, chip->hboost_idle_uv);

	/* A transmitter that ended a session on a fault is restarted. */
	if (faulted) {
		gpiod_set_value_cansleep(chip->wakeup_gpio, 1);
		mt5806_power_cycle(chip);
		gpiod_set_value_cansleep(chip->wakeup_gpio, 0);
	}

	mt5806_set_charging(chip, true);
	power_supply_changed(chip->psy);
}

/*
 * The 48-bit Bluetooth address is spread nibble-wise over the payload bytes
 * 2..4 and 7..9 of the two address packets: address byte k takes the low
 * nibble of byte 2 + k and the high nibble of byte 7 + k, address byte 3 + k
 * takes the opposite nibbles.
 */
static u64 mt5806_decode_addr(const u8 *a)
{
	u64 addr = 0;
	unsigned int k;

	for (k = 0; k < 3; k++) {
		u8 x = a[2 + k], y = a[7 + k];

		addr |= (u64)((x & 0xf) << 4 | y >> 4) << (8 * k);
		addr |= (u64)((y & 0xf) << 4 | x >> 4) << (8 * (3 + k));
	}

	return addr;
}

/* Must be called with chip->lock held. */
static void mt5806_identity_complete(struct mt5806 *chip)
{
	const u8 *a = chip->identity.addr;
	u16 check = (a[9] ^ a[8] ^ a[7]) << 8 | (a[4] ^ a[3] ^ a[2]);
	u64 addr = mt5806_decode_addr(a);

	if (!addr || check != chip->identity.check) {
		dev_warn(chip->dev, "pencil identity check failed\n");
		mt5806_stop_charging(chip, MT5806_STOP_IDENTITY);
		return;
	}

	chip->ble_addr = addr;
	chip->pencil_id = chip->identity.id;
	chip->present = true;
	mt5806_identity_wait_end(chip);

	/* A userspace inhibit applies once the pencil is known. */
	if (chip->inhibit)
		mt5806_set_charging(chip, false);

	if (!chip->recharging)
		power_supply_changed(chip->psy);
}

/* Must be called with chip->lock held. */
static void mt5806_handle_identity(struct mt5806 *chip, const u8 *pkt)
{
	struct mt5806_identity *id = &chip->identity;

	switch (pkt[1]) {
	case MT5806_PKT_IDENTITY_CHECK:
		id->parts = 0;
		id->check = get_unaligned_be16(pkt + 2);
		id->id = pkt[4];
		return;
	case MT5806_PKT_IDENTITY_ADDR0:
		memcpy(id->addr, pkt, MT5806_ADDR_PART_LEN);
		id->parts |= BIT(0);
		break;
	case MT5806_PKT_IDENTITY_ADDR1:
		memcpy(id->addr + MT5806_ADDR_PART_LEN, pkt, MT5806_ADDR_PART_LEN);
		id->parts |= BIT(1);
		break;
	default:
		return;
	}

	if (id->parts != (BIT(0) | BIT(1)))
		return;

	id->parts = 0;
	mt5806_identity_complete(chip);
}

/* Must be called with chip->lock held. */
static void mt5806_handle_stop(struct mt5806 *chip, u8 soc)
{
	if (soc <= MT5806_FULL_SOC)
		chip->soc = soc;

	mt5806_stop_charging(chip, soc == MT5806_FULL_SOC ? MT5806_STOP_FULL :
			     MT5806_STOP_RECEIVER);
	power_supply_changed(chip->psy);
}

/* Must be called with chip->lock held. */
static void mt5806_handle_packet(struct mt5806 *chip)
{
	u8 pkt[MT5806_PACKET_LEN];
	int ret;

	ret = regmap_bulk_read(chip->regmap, MT5806_REG_PACKET, pkt, sizeof(pkt));
	if (ret) {
		dev_err(chip->dev, "failed to read packet: %d\n", ret);
		return;
	}

	dev_dbg(chip->dev, "packet %*ph\n", (int)sizeof(pkt), pkt);

	if (pkt[0] == MT5806_PKT_IDENTITY)
		mt5806_handle_identity(chip, pkt);
	else if (pkt[0] == MT5806_PKT_POWER && pkt[1] == MT5806_PKT_POWER_STOP)
		mt5806_handle_stop(chip, pkt[2]);
}

/* Must be called with chip->lock held. */
static void mt5806_handle_ept(struct mt5806 *chip)
{
	u8 ept[4];
	int ret;

	ret = regmap_bulk_read(chip->regmap, MT5806_REG_EPT_TYPE, ept, sizeof(ept));
	if (ret) {
		dev_err(chip->dev, "failed to read EPT reason: %d\n", ret);
		return;
	}

	if ((ept[1] & MT5806_EPT1_FAULTS) || (ept[2] & MT5806_EPT2_FAULTS)) {
		dev_warn(chip->dev, "power transfer fault %*ph\n", (int)sizeof(ept), ept);
		mt5806_stop_charging(chip, MT5806_STOP_FAULT);
		gpiod_set_value_cansleep(chip->scan_gpio, 1);
		power_supply_changed(chip->psy);
	}

	ret = mt5806_command(chip, MT5806_CMD_CLEAR_EPT);
	if (ret)
		dev_err(chip->dev, "failed to clear EPT: %d\n", ret);
}

/* Must be called with chip->lock held. */
static void mt5806_set_misplaced(struct mt5806 *chip, bool misplaced)
{
	ktime_t now = ktime_get();

	chip->misplaced = misplaced;

	/* The transmitter repeats the warning while the pencil sits off-centre. */
	if (misplaced) {
		bool recent = ktime_ms_delta(now, chip->misplaced_stamp) <=
			      MT5806_MISPLACED_COOLDOWN_MS;

		chip->misplaced_stamp = now;
		if (recent)
			return;
	}

	power_supply_changed(chip->psy);
}

/* Must be called with chip->lock held. */
static int mt5806_handle_events(struct mt5806 *chip)
{
	__le32 raw;
	u32 status;
	int ret;

	ret = regmap_bulk_read(chip->regmap, MT5806_REG_IRQ_STATUS, &raw, sizeof(raw));
	if (ret)
		return ret;

	status = le32_to_cpu(raw);
	if (!status)
		return 0;

	dev_dbg(chip->dev, "interrupt status %#010x\n", status);

	if (status & MT5806_IRQ_RX_REMOVED)
		mt5806_removed(chip);
	if (status & MT5806_IRQ_INIT_DONE)
		memset(&chip->identity, 0, sizeof(chip->identity));
	if (status & MT5806_IRQ_PP_PACKET)
		mt5806_handle_packet(chip);
	if (status & MT5806_IRQ_EPT)
		mt5806_handle_ept(chip);
	if (status & MT5806_IRQ_RX_ATTACHED)
		mt5806_attached(chip);
	if (status & MT5806_IRQ_MISPLACED_CLEAR)
		mt5806_set_misplaced(chip, false);
	if (status & MT5806_IRQ_MISPLACED)
		mt5806_set_misplaced(chip, true);

	ret = regmap_bulk_write(chip->regmap, MT5806_REG_IRQ_CLEAR, &raw, sizeof(raw));
	if (ret)
		return ret;

	return mt5806_command(chip, MT5806_CMD_CLEAR_IRQ);
}

static irqreturn_t mt5806_irq(int irq, void *data)
{
	struct mt5806 *chip = data;
	int ret;

	guard(mutex)(&chip->lock);
	if (chip->stopping)
		return IRQ_HANDLED;

	ret = mt5806_handle_events(chip);
	if (ret)
		dev_err_ratelimited(chip->dev, "failed to handle interrupt: %d\n", ret);

	return IRQ_HANDLED;
}

/* Must be called with chip->lock held. */
static int mt5806_set_soc(struct mt5806 *chip, int soc)
{
	if (soc < 0 || soc > MT5806_FULL_SOC)
		return -EINVAL;

	chip->soc = soc;

	/* A full pencil is charged again once it has drained to the threshold. */
	if (soc <= chip->recharge_soc && chip->attached && !chip->charging &&
	    chip->stop_reason == MT5806_STOP_FULL && !chip->inhibit) {
		chip->recharging = true;
		chip->stop_reason = MT5806_STOP_NONE;
		mt5806_set_charging(chip, true);
		mt5806_start_session(chip);
		/* A stop packet from a pencil that never identified itself is unusual. */
		if (!chip->present)
			mt5806_start_identity_wait(chip);
	}

	return 0;
}

/* Must be called with chip->lock held. */
static void mt5806_set_inhibit(struct mt5806 *chip, bool inhibit)
{
	chip->inhibit = inhibit;

	if (!chip->present)
		return;

	if (inhibit) {
		msleep(MT5806_INHIBIT_DELAY_MS);
		mt5806_set_charging(chip, false);
	} else if (chip->stop_reason == MT5806_STOP_NONE && !chip->charging) {
		mt5806_set_charging(chip, true);
	}
}

static int mt5806_get_status(struct mt5806 *chip)
{
	if (!chip->attached)
		return POWER_SUPPLY_STATUS_UNKNOWN;
	if (chip->charging)
		return POWER_SUPPLY_STATUS_CHARGING;
	if (chip->stop_reason == MT5806_STOP_FULL)
		return POWER_SUPPLY_STATUS_FULL;

	return POWER_SUPPLY_STATUS_NOT_CHARGING;
}

static int mt5806_get_property(struct power_supply *psy,
			       enum power_supply_property psp,
			       union power_supply_propval *val)
{
	struct mt5806 *chip = power_supply_get_drvdata(psy);

	guard(mutex)(&chip->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = mt5806_get_status(chip);
		return 0;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = chip->present;
		return 0;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = chip->attached;
		return 0;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (chip->soc < 0)
			return -ENODATA;
		val->intval = chip->soc;
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		val->intval = chip->inhibit ?
			      POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE :
			      POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO;
		return 0;
	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_DEVICE;
		return 0;
	default:
		return -EINVAL;
	}
}

static int mt5806_set_property(struct power_supply *psy,
			       enum power_supply_property psp,
			       const union power_supply_propval *val)
{
	struct mt5806 *chip = power_supply_get_drvdata(psy);

	guard(mutex)(&chip->lock);
	if (chip->stopping)
		return -ENODEV;

	switch (psp) {
	case POWER_SUPPLY_PROP_CAPACITY:
		return mt5806_set_soc(chip, val->intval);
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		switch (val->intval) {
		case POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO:
			mt5806_set_inhibit(chip, false);
			return 0;
		case POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE:
			mt5806_set_inhibit(chip, true);
			return 0;
		default:
			return -EINVAL;
		}
	default:
		return -EINVAL;
	}
}

static int mt5806_property_is_writeable(struct power_supply *psy,
					enum power_supply_property psp)
{
	return psp == POWER_SUPPLY_PROP_CAPACITY ||
	       psp == POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR;
}

static const enum power_supply_property mt5806_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR,
	POWER_SUPPLY_PROP_SCOPE,
};

static ssize_t ble_mac_addr_show(struct device *dev, struct device_attribute *attr,
				 char *buf)
{
	struct mt5806 *chip = power_supply_get_drvdata(dev_get_drvdata(dev));
	u8 mac[ETH_ALEN];

	scoped_guard(mutex, &chip->lock)
		put_unaligned_be48(chip->ble_addr, mac);

	return sysfs_emit(buf, "%pM\n", mac);
}
static DEVICE_ATTR_RO(ble_mac_addr);

static ssize_t pencil_id_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct mt5806 *chip = power_supply_get_drvdata(dev_get_drvdata(dev));

	guard(mutex)(&chip->lock);
	return sysfs_emit(buf, "%u\n", chip->pencil_id);
}
static DEVICE_ATTR_RO(pencil_id);

static ssize_t misplaced_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct mt5806 *chip = power_supply_get_drvdata(dev_get_drvdata(dev));

	guard(mutex)(&chip->lock);
	return sysfs_emit(buf, "%u\n", chip->misplaced);
}
static DEVICE_ATTR_RO(misplaced);

static ssize_t fw_version_show(struct device *dev, struct device_attribute *attr,
			       char *buf)
{
	struct mt5806 *chip = power_supply_get_drvdata(dev_get_drvdata(dev));

	return sysfs_emit(buf, "%04x.%04x\n", chip->fw_major, chip->fw_minor);
}
static DEVICE_ATTR_RO(fw_version);

static struct attribute *mt5806_attrs[] = {
	&dev_attr_ble_mac_addr.attr,
	&dev_attr_pencil_id.attr,
	&dev_attr_misplaced.attr,
	&dev_attr_fw_version.attr,
	NULL
};
ATTRIBUTE_GROUPS(mt5806);

static const struct power_supply_desc mt5806_psy_desc = {
	.name = "oplus-pencil",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.charge_behaviours = BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO) |
			     BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE),
	.properties = mt5806_properties,
	.num_properties = ARRAY_SIZE(mt5806_properties),
	.get_property = mt5806_get_property,
	.set_property = mt5806_set_property,
	.property_is_writeable = mt5806_property_is_writeable,
};

static int mt5806_check_chip_id(struct mt5806 *chip)
{
	unsigned int attempt;
	u8 id[2];
	int ret;

	for (attempt = 0; attempt < MT5806_CHIP_ID_RETRIES; attempt++) {
		ret = regmap_bulk_read(chip->regmap, MT5806_REG_CHIP_ID, id, sizeof(id));
		if (!ret && get_unaligned_le16(id) == MT5806_CHIP_ID)
			return 0;
		msleep(MT5806_CHIP_ID_RETRY_MS);
	}

	if (ret)
		return dev_err_probe(chip->dev, ret, "failed to read chip ID\n");

	return dev_err_probe(chip->dev, -ENODEV, "unexpected chip ID %#x\n",
			     get_unaligned_le16(id));
}

static int mt5806_read_fw_version(struct mt5806 *chip)
{
	u8 major[2], minor[2];
	int ret;

	ret = regmap_bulk_read(chip->regmap, MT5806_REG_FW_MAJOR, major, sizeof(major));
	if (ret)
		return ret;

	ret = regmap_bulk_read(chip->regmap, MT5806_REG_FW_MINOR, minor, sizeof(minor));
	if (ret)
		return ret;

	chip->fw_major = get_unaligned_le16(major);
	chip->fw_minor = get_unaligned_le16(minor);

	return 0;
}

static int mt5806_parse_properties(struct mt5806 *chip)
{
	struct device *dev = chip->dev;
	u32 timeout_sec = MT5806_DEFAULT_TIMEOUT_SEC;

	chip->hboost_idle_uv = MT5806_DEFAULT_HBOOST_UV;
	device_property_read_u32(dev, "oplus,hboost-idle-microvolt",
				 &chip->hboost_idle_uv);
	chip->hboost_charge_uv = chip->hboost_idle_uv;
	device_property_read_u32(dev, "oplus,hboost-charge-microvolt",
				 &chip->hboost_charge_uv);

	chip->recharge_soc = MT5806_DEFAULT_RECHARGE_SOC;
	device_property_read_u32(dev, "oplus,recharge-soc-percent", &chip->recharge_soc);
	if (chip->recharge_soc > MT5806_FULL_SOC)
		return dev_err_probe(dev, -EINVAL, "invalid recharge threshold\n");

	device_property_read_u32(dev, "oplus,charge-timeout-sec", &timeout_sec);
	if (!timeout_sec || timeout_sec > U32_MAX / MSEC_PER_SEC)
		return dev_err_probe(dev, -EINVAL, "invalid charge timeout\n");
	chip->charge_timeout_ms = timeout_sec * MSEC_PER_SEC;

	return 0;
}

static int mt5806_get_gpios(struct mt5806 *chip)
{
	struct device *dev = chip->dev;

	/* Power stays off and power transfer blocked until the chip has booted. */
	chip->power_gpio = devm_gpiod_get(dev, "power", GPIOD_OUT_LOW);
	if (IS_ERR(chip->power_gpio))
		return dev_err_probe(dev, PTR_ERR(chip->power_gpio),
				     "failed to get power GPIO\n");

	chip->wakeup_gpio = devm_gpiod_get(dev, "wakeup", GPIOD_OUT_HIGH);
	if (IS_ERR(chip->wakeup_gpio))
		return dev_err_probe(dev, PTR_ERR(chip->wakeup_gpio),
				     "failed to get wakeup GPIO\n");

	chip->scan_gpio = devm_gpiod_get(dev, "scan", GPIOD_OUT_HIGH);
	if (IS_ERR(chip->scan_gpio))
		return dev_err_probe(dev, PTR_ERR(chip->scan_gpio),
				     "failed to get scan GPIO\n");

	chip->charge_off_gpio = devm_gpiod_get(dev, "charge-off", GPIOD_OUT_HIGH);
	if (IS_ERR(chip->charge_off_gpio))
		return dev_err_probe(dev, PTR_ERR(chip->charge_off_gpio),
				     "failed to get charge-off GPIO\n");

	return 0;
}

/* Close every entry point that can queue work, then drain what is queued. */
static void mt5806_stop(void *data)
{
	struct mt5806 *chip = data;

	scoped_guard(mutex, &chip->lock) {
		chip->stopping = true;
		__pm_relax(chip->identity_ws);
	}

	cancel_work_sync(&chip->boost_work);
	cancel_delayed_work_sync(&chip->identity_work);
	cancel_delayed_work_sync(&chip->timeout_work);
}

static void mt5806_unregister_identity_ws(void *data)
{
	wakeup_source_unregister(data);
}

static void mt5806_power_off(void *data)
{
	struct mt5806 *chip = data;

	gpiod_set_value_cansleep(chip->charge_off_gpio, 1);
	gpiod_set_value_cansleep(chip->power_gpio, 0);
}

static int mt5806_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct power_supply_config psy_cfg = {};
	struct mt5806 *chip;
	int ret;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = dev;
	chip->irq = client->irq;
	chip->soc = -1;
	chip->fast_scan = true;
	i2c_set_clientdata(client, chip);

	ret = devm_mutex_init(dev, &chip->lock);
	if (ret)
		return ret;

	INIT_WORK(&chip->boost_work, mt5806_boost_work);
	INIT_DELAYED_WORK(&chip->identity_work, mt5806_identity_work);
	INIT_DELAYED_WORK(&chip->timeout_work, mt5806_timeout_work);
	/* One long-sleeping work item: an unbound worker, frozen across suspend. */
	chip->boost_wq = devm_alloc_workqueue(dev, "%s-boost",
					      WQ_UNBOUND | WQ_FREEZABLE, 1, dev_name(dev));
	if (!chip->boost_wq)
		return -ENOMEM;

	chip->identity_ws = wakeup_source_register(dev, "mt5806-identity");
	if (!chip->identity_ws)
		return -ENOMEM;
	ret = devm_add_action_or_reset(dev, mt5806_unregister_identity_ws,
				       chip->identity_ws);
	if (ret)
		return ret;

	ret = mt5806_parse_properties(chip);
	if (ret)
		return ret;

	chip->regmap = devm_regmap_init_i2c(client, &mt5806_regmap_config);
	if (IS_ERR(chip->regmap))
		return dev_err_probe(dev, PTR_ERR(chip->regmap),
				     "failed to init regmap\n");

	chip->hboost = devm_regulator_get(dev, "hboost");
	if (IS_ERR(chip->hboost))
		return dev_err_probe(dev, PTR_ERR(chip->hboost),
				     "failed to get HBOOST supply\n");

	ret = mt5806_get_gpios(chip);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, mt5806_power_off, chip);
	if (ret)
		return ret;

	/* The charger firmware may still be starting; keep deferring until it answers. */
	chip->hboost_target_uv = chip->hboost_idle_uv;
	ret = mt5806_set_hboost(chip, chip->hboost_idle_uv);
	if (ret)
		return dev_err_probe(dev, ret == -EAGAIN ? -EPROBE_DEFER : ret,
				     "HBOOST not available\n");

	mt5806_power_cycle(chip);

	ret = mt5806_check_chip_id(chip);
	if (ret)
		return ret;

	ret = mt5806_read_fw_version(chip);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read firmware version\n");

	psy_cfg.drv_data = chip;
	psy_cfg.attr_grp = mt5806_groups;
	chip->psy = devm_power_supply_register(dev, &mt5806_psy_desc, &psy_cfg);
	if (IS_ERR(chip->psy))
		return dev_err_probe(dev, PTR_ERR(chip->psy),
				     "failed to register power supply\n");

	/* Work items notify the power supply; stop them before it goes away. */
	ret = devm_add_action_or_reset(dev, mt5806_stop, chip);
	if (ret)
		return ret;

	ret = devm_device_init_wakeup(dev);
	if (ret)
		return ret;

	ret = devm_pm_set_wake_irq(dev, chip->irq);
	if (ret)
		return ret;

	ret = devm_request_threaded_irq(dev, chip->irq, NULL, mt5806_irq,
					IRQF_ONESHOT, dev_name(dev), chip);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request interrupt\n");

	/* Let the transmitter manage its own sleep and start looking for a pencil. */
	gpiod_set_value_cansleep(chip->wakeup_gpio, 0);
	scoped_guard(mutex, &chip->lock) {
		mt5806_set_charging(chip, true);
		ret = mt5806_handle_events(chip);
	}
	if (ret)
		dev_warn(dev, "failed to read initial events: %d\n", ret);

	dev_info(dev, "MT5806 firmware %04x.%04x\n", chip->fw_major, chip->fw_minor);

	return 0;
}

static void mt5806_shutdown(struct i2c_client *client)
{
	struct mt5806 *chip = i2c_get_clientdata(client);

	disable_irq(chip->irq);
	mt5806_stop(chip);
	mt5806_power_off(chip);
}

static int mt5806_suspend(struct device *dev)
{
	struct mt5806 *chip = dev_get_drvdata(dev);

	/* Events are served after resume, once the I2C controller is back. */
	disable_irq(chip->irq);

	guard(mutex)(&chip->lock);
	chip->fast_scan = false;
	gpiod_set_value_cansleep(chip->scan_gpio, 0);

	return 0;
}

static int mt5806_resume(struct device *dev)
{
	struct mt5806 *chip = dev_get_drvdata(dev);

	scoped_guard(mutex, &chip->lock) {
		chip->fast_scan = true;
		gpiod_set_value_cansleep(chip->scan_gpio, 1);
		mt5806_check_timeout(chip);
	}

	enable_irq(chip->irq);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(mt5806_pm_ops, mt5806_suspend, mt5806_resume);

static const struct of_device_id mt5806_of_match[] = {
	{ .compatible = "oplus,wls-tx-mt5806" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt5806_of_match);

static struct i2c_driver mt5806_driver = {
	.driver = {
		.name = "oplus-pencil-mt5806",
		.of_match_table = mt5806_of_match,
		.pm = pm_sleep_ptr(&mt5806_pm_ops),
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe = mt5806_probe,
	.shutdown = mt5806_shutdown,
};
module_i2c_driver(mt5806_driver);

MODULE_DESCRIPTION("OnePlus/Oplus MT5806 pencil wireless charging transmitter driver");
MODULE_LICENSE("GPL");
