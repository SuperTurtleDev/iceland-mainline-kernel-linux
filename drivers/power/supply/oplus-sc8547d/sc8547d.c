// SPDX-License-Identifier: GPL-2.0-only
/*
 * Southchip SC8547D 2:1 charge pump with VOOC and UFCS PHYs.
 *
 * Register sequences follow the Oplus SC8547D HAL
 * (vendor/oplus/kernel/charger/v2/ufcs_ic/oplus_hal_sc8547d.c),
 * Copyright (C) 2022-2023 Oplus. All rights reserved.
 *
 * The chip only provides register access: a fast charging session driver
 * prepares it, starts switching once the adapter voltage is in range and
 * kicks its watchdog while it runs. The interrupt is handed to whichever
 * protocol owns the PHY.
 */
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/regmap.h>

#include "sc8547d.h"

#define REG_VBAT_OVP		0x00
#define REG_VBAT_OCP		0x01
#define REG_VAC_OVP		0x02
#define REG_VBUS_OVP		0x04
#define REG_IBUS_OCP		0x05
#define REG_STATUS		0x06
#define REG_CONTROL		0x07
#define REG_SS_TIMEOUT		0x08
#define REG_MODE		0x09
#define REG_PMID2OUT		0x0d
#define REG_PROT_STAT		0x0e
#define REG_PROT_FLAG		0x0f
#define REG_IRQ_MASK		0x10
#define REG_ADC_CTRL		0x11
#define REG_ADC_IBUS		0x13
#define REG_ADC_TDIE		0x1f
#define REG_DPDM_CTRL		0x21
#define REG_DPDM_CTRL2		0x22
#define REG_VOOC_CTRL		0x2b
#define REG_VOOC_TX		0x2c
#define REG_VOOC_RX		0x2e
#define REG_VOOC_FLAG		0x2f
#define REG_VOOC_IRQ_MASK	0x30
#define REG_VOOC_PREDATA	0x31
#define REG_DET_CTRL		0x33
#define REG_PHY_CTRL		0x34
#define REG_DEVICE_ID		0x36
#define REG_OTG			0x3b
#define REG_VBUS_IN_RANGE	0x3c
#define REG_UFCS_CTRL1		0x40
#define REG_UFCS_CTRL2		0x41
#define REG_UFCS_FLAG1		0x43
#define REG_UFCS_MASK1		0x46
#define REG_UFCS_MASK2		0x47
#define REG_UFCS_TX_LEN		0x49
#define REG_UFCS_TX_BUF		0x4a
#define REG_UFCS_RX_LEN		0x8a
#define REG_UFCS_RX_BUF		0x8b
#define REG_UFCS_TIMING		0xcc
#define REG_MAX			0xcc

#define SC8547D_ID		0x49

#define STATUS_TSHUT_FLAG	BIT(7)
#define STATUS_TSHUT		BIT(6)
#define STATUS_VBUS_LOW		BIT(5)
#define STATUS_VBUS_HIGH	BIT(4)
#define STATUS_SS_TIMEOUT	BIT(3)
#define STATUS_SWITCHING	BIT(2)
#define STATUS_PIN_DIAG		BIT(0)
#define CONTROL_EN		BIT(7)
#define CONTROL_RESET		BIT(6)
#define CONTROL_FSW_MASK	GENMASK(6, 4)
#define CONTROL_FSW_600K	BIT(0)
#define SS_TIMEOUT_MASK		GENMASK(7, 5)
#define SS_TIMEOUT_81920MS	7
#define MODE_BYPASS		BIT(7)
#define MODE_UCP_RISE_MASK	BIT(4)
#define MODE_WD_TIMEOUT		BIT(3)
#define MODE_WATCHDOG		GENMASK(2, 0)
#define WATCHDOG_OFF		0
#define WATCHDOG_5S		4
#define PROT_VOUT_OVP		BIT(7)
#define PROT_VBAT_OVP		BIT(6)
#define PROT_IBAT_OCP		BIT(5)
#define PROT_VBUS_OVP		BIT(4)
#define PROT_IBUS_OCP		BIT(3)
#define PMID2OUT_UVP_FLAG	BIT(3)
#define PMID2OUT_OVP_FLAG	BIT(2)
#define ADC_EN			BIT(7)
#define ADC_FREEZE		BIT(5)
#define IBUS_UCP_DISABLE	BIT(7)
#define IBUS_UCP_50MS		BIT(5)
#define OTG_EN			BIT(2)
#define DPDM_DP_20K_PD		BIT(4)
#define DPDM_DM_20K_PD		BIT(5)
#define UFCS_CTRL1_EN		BIT(7)
#define UFCS_CTRL1_HANDSHAKE	BIT(5)
#define UFCS_CTRL1_BAUD		GENMASK(4, 3)
#define UFCS_CTRL1_SEND		BIT(2)
#define UFCS_CTRL1_CABLE_RESET	BIT(1)
#define UFCS_CTRL1_SRC_RESET	BIT(0)
#define UFCS_CTRL2_CLR_TX_RX	0x30
#define UFCS_CTRL2_CLR_RX	BIT(4)

#define VBAT_OVP_MIN_UV		3500000
#define VBAT_OVP_STEP_UV	25000
#define VBUS_OVP_MIN_UV		6000000
#define VBUS_OVP_STEP_UV	50000
#define IBUS_OCP_MIN_UA		1200000
#define IBUS_OCP_STEP_UA	300000

struct sc8547d {
	struct device *dev;
	struct regmap *regmap;
	/* Serializes multi-register sequences and the ADC snapshot. */
	struct mutex lock;
	/* Separate so an interrupt never waits behind an ADC read. */
	struct mutex irq_lock;
	void (*irq_handler)(void *data);
	void *irq_data;
	u8 vbat_ovp;
	u8 vbus_ovp;
	u8 ibus_ocp;
	u8 fsw;
	bool always_otg;
	bool vooc_phy;
	bool ufcs_phy;
	bool enabled;
};

static int sc8547d_write(struct sc8547d *cp, unsigned int reg, unsigned int val)
{
	return regmap_write(cp->regmap, reg, val);
}

static int sc8547d_update(struct sc8547d *cp, unsigned int reg, unsigned int mask,
			  unsigned int val)
{
	return regmap_update_bits(cp->regmap, reg, mask, val);
}

/* Factory protection setup with switching off: sc8547_init_device(). */
static int sc8547d_init_device(struct sc8547d *cp)
{
	const struct reg_sequence seq[] = {
		{ REG_ADC_CTRL, 0 },
		{ REG_VAC_OVP, 0x01 },
		{ REG_VBUS_OVP, cp->vbus_ovp },
		{ REG_VBAT_OVP, cp->vbat_ovp },
		{ REG_IBUS_OCP, IBUS_UCP_50MS | cp->ibus_ocp },
		{ REG_VBAT_OCP, 0xbf },
		{ REG_VOOC_CTRL, 0 },
		{ REG_PHY_CTRL, 0x60 },
	};
	int ret;

	ret = regmap_multi_reg_write(cp->regmap, seq, ARRAY_SIZE(seq));
	if (!ret)
		ret = sc8547d_update(cp, REG_MODE, MODE_UCP_RISE_MASK, MODE_UCP_RISE_MASK);
	if (!ret)
		ret = sc8547d_write(cp, REG_IRQ_MASK, 0x02);
	if (!ret && cp->always_otg)
		ret = sc8547d_update(cp, REG_OTG, OTG_EN, OTG_EN);
	return ret;
}

static int sc8547d_reset(struct sc8547d *cp)
{
	int ret;

	ret = sc8547d_update(cp, REG_CONTROL, CONTROL_RESET, CONTROL_RESET);
	if (ret)
		return ret;
	usleep_range(10000, 11000);
	return sc8547d_init_device(cp);
}

/* Turn the converter off; the enable bit clears at once. */
static int sc8547d_off_locked(struct sc8547d *cp)
{
	int ret;

	ret = sc8547d_write(cp, REG_CONTROL, cp->fsw);
	if (!ret)
		ret = sc8547d_update(cp, REG_MODE, MODE_WATCHDOG, WATCHDOG_OFF);
	cp->enabled = false;
	return ret;
}

/* sc8547_svooc_hw_setting() / sc8547_vooc_hw_setting() without watchdog. */
int sc8547d_cp_prepare(struct sc8547d *cp, enum sc8547d_mode mode)
{
	bool bypass = mode == SC8547D_MODE_BYPASS;
	const struct reg_sequence seq[] = {
		{ REG_VAC_OVP, bypass ? 0x07 : 0x01 },
		{ REG_VBUS_OVP, bypass ? 0x64 : cp->vbus_ovp },
		{ REG_IBUS_OCP, bypass ? 0x2c : IBUS_UCP_50MS | cp->ibus_ocp },
		{ REG_DET_CTRL, 0xd1 },
		{ REG_PHY_CTRL, 0x60 },
	};
	int ret;

	mutex_lock(&cp->lock);
	ret = regmap_multi_reg_write(cp->regmap, seq, ARRAY_SIZE(seq));
	if (!ret)
		ret = sc8547d_update(cp, REG_MODE, MODE_BYPASS | MODE_UCP_RISE_MASK,
				     (bypass ? MODE_BYPASS : 0) | MODE_UCP_RISE_MASK);
	if (!ret)
		ret = sc8547d_update(cp, REG_ADC_CTRL, ADC_EN, ADC_EN);
	if (!ret && !bypass)
		ret = sc8547d_write(cp, REG_PMID2OUT, 0x70);
	if (!ret && cp->fsw)
		ret = sc8547d_write(cp, REG_VBUS_IN_RANGE, 0x60);
	/* Soft start timeout and UCP are enabled once the pump carries current. */
	if (!ret)
		ret = sc8547d_update(cp, REG_IBUS_OCP, IBUS_UCP_DISABLE, IBUS_UCP_DISABLE);
	if (!ret)
		ret = sc8547d_update(cp, REG_SS_TIMEOUT, SS_TIMEOUT_MASK, 0);
	if (!ret)
		ret = sc8547d_update(cp, REG_MODE, MODE_WATCHDOG, WATCHDOG_5S);
	mutex_unlock(&cp->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(sc8547d_cp_prepare);

int sc8547d_cp_start(struct sc8547d *cp, bool on)
{
	int ret;

	mutex_lock(&cp->lock);
	ret = sc8547d_write(cp, REG_CONTROL, (on ? CONTROL_EN : 0) | cp->fsw);
	if (!ret)
		cp->enabled = on;
	mutex_unlock(&cp->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(sc8547d_cp_start);

/* Switching off, ADC off, watchdog off, protections back to defaults. */
int sc8547d_cp_stop(struct sc8547d *cp)
{
	int ret, err;

	mutex_lock(&cp->lock);
	ret = sc8547d_off_locked(cp);
	err = sc8547d_write(cp, REG_VBAT_OVP, cp->vbat_ovp);
	ret = ret ?: err;
	err = sc8547d_write(cp, REG_VAC_OVP, 0x01);
	ret = ret ?: err;
	err = sc8547d_write(cp, REG_VBUS_OVP, cp->vbus_ovp);
	ret = ret ?: err;
	err = sc8547d_write(cp, REG_IBUS_OCP, IBUS_UCP_50MS | cp->ibus_ocp);
	ret = ret ?: err;
	err = sc8547d_update(cp, REG_SS_TIMEOUT, SS_TIMEOUT_MASK,
			     FIELD_PREP(SS_TIMEOUT_MASK, SS_TIMEOUT_81920MS));
	ret = ret ?: err;
	err = sc8547d_write(cp, REG_ADC_CTRL, 0);
	ret = ret ?: err;
	mutex_unlock(&cp->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(sc8547d_cp_stop);

/* Reading the control register services the watchdog. */
int sc8547d_cp_watchdog_kick(struct sc8547d *cp)
{
	unsigned int val;

	return regmap_read(cp->regmap, REG_CONTROL, &val);
}
EXPORT_SYMBOL_GPL(sc8547d_cp_watchdog_kick);

/* sc8547d_cp_set_sstimeout_ucp_enable(): once IBUS is established. */
int sc8547d_cp_ucp_enable(struct sc8547d *cp, bool enable)
{
	int ret;

	mutex_lock(&cp->lock);
	ret = sc8547d_update(cp, REG_IBUS_OCP, IBUS_UCP_DISABLE, enable ? 0 : IBUS_UCP_DISABLE);
	if (!ret)
		ret = sc8547d_update(cp, REG_SS_TIMEOUT, SS_TIMEOUT_MASK,
				     enable ? FIELD_PREP(SS_TIMEOUT_MASK, SS_TIMEOUT_81920MS) : 0);
	mutex_unlock(&cp->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(sc8547d_cp_ucp_enable);

static unsigned int sc8547d_fault_bits(unsigned int status, unsigned int prot,
				       unsigned int mode, unsigned int pmid)
{
	unsigned int fault = 0;

	if (status & (STATUS_TSHUT_FLAG | STATUS_TSHUT))
		fault |= SC8547D_FAULT_TSHUT;
	if (status & STATUS_SS_TIMEOUT)
		fault |= SC8547D_FAULT_SS_TIMEOUT;
	if (status & STATUS_PIN_DIAG)
		fault |= SC8547D_FAULT_PIN_DIAG;
	if (prot & PROT_VOUT_OVP)
		fault |= SC8547D_FAULT_VOUT_OVP;
	if (prot & PROT_VBAT_OVP)
		fault |= SC8547D_FAULT_VBAT_OVP;
	if (prot & PROT_IBAT_OCP)
		fault |= SC8547D_FAULT_IBAT_OCP;
	if (prot & PROT_VBUS_OVP)
		fault |= SC8547D_FAULT_VBUS_OVP;
	if (prot & PROT_IBUS_OCP)
		fault |= SC8547D_FAULT_IBUS_OCP;
	if (mode & MODE_WD_TIMEOUT)
		fault |= SC8547D_FAULT_WATCHDOG;
	if (pmid & (PMID2OUT_UVP_FLAG | PMID2OUT_OVP_FLAG))
		fault |= SC8547D_FAULT_PMID2OUT;
	return fault;
}

const char *sc8547d_fault_name(unsigned int fault)
{
	static const char * const names[] = {
		"thermal shutdown", "soft start timeout", "VOUT OVP", "VBAT OVP",
		"IBAT OCP", "VBUS OVP", "IBUS OCP", "watchdog", "PMID2OUT", "pin diag",
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(names); i++)
		if (fault & BIT(i))
			return names[i];
	return "none";
}
EXPORT_SYMBOL_GPL(sc8547d_fault_name);

static int sc8547d_adc(const u8 *adc, int n, u8 high_mask)
{
	return ((adc[2 * n] & high_mask) << 8) | adc[2 * n + 1];
}

/*
 * sc8547d_sample() - status flags, protection flags and the ADC snapshot
 * @cp: the pump
 * @s: the snapshot
 *
 * Reading the flag registers clears them: the caller owns the faults it saw.
 */
int sc8547d_sample(struct sc8547d *cp, struct sc8547d_sample *s)
{
	u8 adc[REG_ADC_TDIE + 2 - REG_ADC_IBUS];
	unsigned int status, prot, mode, pmid;
	int ret, thaw;

	mutex_lock(&cp->lock);
	ret = regmap_read(cp->regmap, REG_STATUS, &status);
	if (!ret)
		ret = regmap_read(cp->regmap, REG_PROT_FLAG, &prot);
	if (!ret)
		ret = regmap_read(cp->regmap, REG_MODE, &mode);
	if (!ret)
		ret = regmap_read(cp->regmap, REG_PMID2OUT, &pmid);
	if (!ret)
		ret = sc8547d_update(cp, REG_ADC_CTRL, ADC_FREEZE, ADC_FREEZE);
	if (ret)
		goto out;
	ret = regmap_bulk_read(cp->regmap, REG_ADC_IBUS, adc, sizeof(adc));
	thaw = sc8547d_update(cp, REG_ADC_CTRL, ADC_FREEZE, 0);
	ret = ret ?: thaw;
	if (ret)
		goto out;
	s->switching = status & STATUS_SWITCHING;
	s->fault = sc8547d_fault_bits(status, prot, mode, pmid);
	s->ibus_ma = sc8547d_adc(adc, 0, 0x0f) * 1875 / 1000;
	s->vbus_mv = sc8547d_adc(adc, 1, 0x0f) * 375 / 100;
	s->vac_mv = sc8547d_adc(adc, 2, 0x0f) * 5;
	s->vout_mv = sc8547d_adc(adc, 3, 0x0f) * 125 / 100;
	s->vbat_mv = sc8547d_adc(adc, 4, 0x0f) * 125 / 100;
	s->tdie_dc = sc8547d_adc(adc, 6, 0x01) * 5;
out:
	mutex_unlock(&cp->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(sc8547d_sample);

int sc8547d_switching(struct sc8547d *cp, bool *on)
{
	unsigned int status;
	int ret;

	ret = regmap_read(cp->regmap, REG_STATUS, &status);
	if (!ret)
		*on = status & STATUS_SWITCHING;
	return ret;
}
EXPORT_SYMBOL_GPL(sc8547d_switching);

void sc8547d_set_irq_handler(struct sc8547d *cp, void (*handler)(void *data), void *data)
{
	mutex_lock(&cp->irq_lock);
	cp->irq_handler = handler;
	cp->irq_data = data;
	mutex_unlock(&cp->irq_lock);
}
EXPORT_SYMBOL_GPL(sc8547d_set_irq_handler);

/* VOOC PHY: sc8547_voocphy_init_vooc() without the register reset. */
int sc8547d_vooc_enable(struct sc8547d *cp)
{
	static const u8 zero[2];
	int ret;

	mutex_lock(&cp->lock);
	ret = regmap_bulk_write(cp->regmap, REG_VOOC_PREDATA, zero, sizeof(zero));
	/* Hold bit 0 for 800 us, or the adapter misreads current commands. */
	if (!ret)
		ret = sc8547d_update(cp, REG_PHY_CTRL, BIT(6), BIT(6));
	if (!ret)
		ret = sc8547d_write(cp, REG_DPDM_CTRL, 0x21);
	if (!ret)
		ret = sc8547d_write(cp, REG_DPDM_CTRL2, 0);
	if (!ret)
		ret = sc8547d_write(cp, REG_DET_CTRL, 0xd1);
	if (!ret)
		ret = regmap_bulk_write(cp->regmap, REG_VOOC_TX, zero, sizeof(zero));
	if (!ret)
		ret = sc8547d_write(cp, REG_VOOC_IRQ_MASK, SC8547D_VOOC_MASK_RX);
	mutex_unlock(&cp->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(sc8547d_vooc_enable);

/* sc8547_voocphy_reset_voocphy(): the pump itself is stopped separately. */
int sc8547d_vooc_disable(struct sc8547d *cp)
{
	static const u8 zero[2];
	int ret, err;

	mutex_lock(&cp->lock);
	ret = sc8547d_write(cp, REG_VOOC_CTRL, 0);
	err = sc8547d_write(cp, REG_VOOC_IRQ_MASK, 0x7f);
	ret = ret ?: err;
	err = sc8547d_write(cp, REG_DPDM_CTRL, 0xc0);
	ret = ret ?: err;
	err = regmap_bulk_write(cp->regmap, REG_VOOC_TX, zero, sizeof(zero));
	ret = ret ?: err;
	err = regmap_bulk_write(cp->regmap, REG_VOOC_PREDATA, zero, sizeof(zero));
	ret = ret ?: err;
	err = sc8547d_write(cp, REG_IRQ_MASK, 0x02);
	ret = ret ?: err;
	mutex_unlock(&cp->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(sc8547d_vooc_disable);

int sc8547d_vooc_handshake(struct sc8547d *cp)
{
	return sc8547d_write(cp, REG_VOOC_CTRL, 0x81);
}
EXPORT_SYMBOL_GPL(sc8547d_vooc_handshake);

int sc8547d_vooc_rx(struct sc8547d *cp, u8 *data, u8 *flags)
{
	u8 rx[2];
	int ret;

	ret = regmap_bulk_read(cp->regmap, REG_VOOC_RX, rx, sizeof(rx));
	if (ret)
		return ret;
	*data = rx[0];
	*flags = rx[1];
	return 0;
}
EXPORT_SYMBOL_GPL(sc8547d_vooc_rx);

static int sc8547d_vooc_write_frame(struct sc8547d *cp, unsigned int reg, u16 frame)
{
	u8 data[2] = { frame >> 8, frame & 0xff };

	return regmap_bulk_write(cp->regmap, reg, data, sizeof(data));
}

int sc8547d_vooc_tx(struct sc8547d *cp, u16 frame)
{
	return sc8547d_vooc_write_frame(cp, REG_VOOC_TX, frame);
}
EXPORT_SYMBOL_GPL(sc8547d_vooc_tx);

int sc8547d_vooc_predata(struct sc8547d *cp, u16 frame)
{
	return sc8547d_vooc_write_frame(cp, REG_VOOC_PREDATA, frame);
}
EXPORT_SYMBOL_GPL(sc8547d_vooc_predata);

int sc8547d_vooc_irq_mask(struct sc8547d *cp, u8 mask)
{
	return sc8547d_write(cp, REG_VOOC_IRQ_MASK, mask);
}
EXPORT_SYMBOL_GPL(sc8547d_vooc_irq_mask);

/* UFCS PHY: sc8547d_ufcs_enable(), after the pump reset and defaults. */
int sc8547d_ufcs_enable(struct sc8547d *cp)
{
	static const struct reg_sequence seq[] = {
		{ REG_DPDM_CTRL, 0x01 },
		{ REG_UFCS_CTRL1, UFCS_CTRL1_EN },
		{ REG_UFCS_CTRL2, UFCS_CTRL2_CLR_TX_RX },
		{ REG_UFCS_MASK1, 0x01 },
		{ REG_UFCS_MASK2, 0x40 },
		/* hard reset 1900 us; ACK timeout 10 ms plus a frame */
		{ REG_UFCS_TIMING, 0x80 },
	};
	int ret;

	mutex_lock(&cp->lock);
	ret = sc8547d_reset(cp);
	if (!ret)
		ret = regmap_multi_reg_write(cp->regmap, seq, ARRAY_SIZE(seq));
	if (!ret)
		ret = sc8547d_write(cp, REG_MODE, WATCHDOG_5S);
	mutex_unlock(&cp->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(sc8547d_ufcs_enable);

int sc8547d_ufcs_disable(struct sc8547d *cp)
{
	int ret, err;

	mutex_lock(&cp->lock);
	ret = sc8547d_write(cp, REG_UFCS_CTRL1, 0);
	err = sc8547d_update(cp, REG_MODE, MODE_WATCHDOG, WATCHDOG_OFF);
	ret = ret ?: err;
	err = sc8547d_write(cp, REG_DPDM_CTRL, 0xc0);
	ret = ret ?: err;
	mutex_unlock(&cp->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(sc8547d_ufcs_disable);

int sc8547d_ufcs_handshake(struct sc8547d *cp)
{
	return sc8547d_update(cp, REG_UFCS_CTRL1, UFCS_CTRL1_HANDSHAKE, UFCS_CTRL1_HANDSHAKE);
}
EXPORT_SYMBOL_GPL(sc8547d_ufcs_handshake);

/* @baud: 0 is 115200, 1 is 57600, 2 is 38400, 3 is 19200. */
int sc8547d_ufcs_set_baud(struct sc8547d *cp, unsigned int baud)
{
	return sc8547d_update(cp, REG_UFCS_CTRL1, UFCS_CTRL1_BAUD,
			      FIELD_PREP(UFCS_CTRL1_BAUD, baud));
}
EXPORT_SYMBOL_GPL(sc8547d_ufcs_set_baud);

int sc8547d_ufcs_hard_reset(struct sc8547d *cp)
{
	int ret;

	ret = sc8547d_update(cp, REG_UFCS_CTRL1, UFCS_CTRL1_SRC_RESET, UFCS_CTRL1_SRC_RESET);
	if (!ret)
		msleep(100);
	return ret;
}
EXPORT_SYMBOL_GPL(sc8547d_ufcs_hard_reset);

int sc8547d_ufcs_read_flags(struct sc8547d *cp, u8 flags[3])
{
	return regmap_bulk_read(cp->regmap, REG_UFCS_FLAG1, flags, 3);
}
EXPORT_SYMBOL_GPL(sc8547d_ufcs_read_flags);

int sc8547d_ufcs_write_msg(struct sc8547d *cp, const u8 *buf, int len)
{
	int ret;

	if (len < 1 || len > SC8547D_UFCS_MSG_MAX)
		return -EMSGSIZE;
	mutex_lock(&cp->lock);
	ret = sc8547d_write(cp, REG_UFCS_TX_LEN, len);
	if (!ret)
		ret = regmap_bulk_write(cp->regmap, REG_UFCS_TX_BUF, buf, len);
	if (!ret)
		ret = sc8547d_update(cp, REG_UFCS_CTRL1, UFCS_CTRL1_SEND, UFCS_CTRL1_SEND);
	mutex_unlock(&cp->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(sc8547d_ufcs_write_msg);

/* Return: the message length, 0 for an empty buffer, or a negative error. */
int sc8547d_ufcs_read_msg(struct sc8547d *cp, u8 *buf, int cap)
{
	unsigned int len;
	int ret;

	mutex_lock(&cp->lock);
	ret = regmap_read(cp->regmap, REG_UFCS_RX_LEN, &len);
	if (!ret && len > SC8547D_UFCS_MSG_MAX)
		ret = -EMSGSIZE;
	if (!ret && len > cap)
		ret = -EMSGSIZE;
	if (!ret && len)
		ret = regmap_bulk_read(cp->regmap, REG_UFCS_RX_BUF, buf, len);
	mutex_unlock(&cp->lock);
	return ret ?: len;
}
EXPORT_SYMBOL_GPL(sc8547d_ufcs_read_msg);

int sc8547d_ufcs_clear_rx(struct sc8547d *cp)
{
	return sc8547d_update(cp, REG_UFCS_CTRL2, UFCS_CTRL2_CLR_RX, UFCS_CTRL2_CLR_RX);
}
EXPORT_SYMBOL_GPL(sc8547d_ufcs_clear_rx);

static irqreturn_t sc8547d_irq(int irq, void *data)
{
	struct sc8547d *cp = data;

	mutex_lock(&cp->irq_lock);
	if (cp->irq_handler)
		cp->irq_handler(cp->irq_data);
	mutex_unlock(&cp->irq_lock);
	return IRQ_HANDLED;
}

struct device *sc8547d_device(struct sc8547d *cp)
{
	return cp->dev;
}
EXPORT_SYMBOL_GPL(sc8547d_device);

bool sc8547d_has_vooc_phy(struct sc8547d *cp)
{
	return cp->vooc_phy;
}
EXPORT_SYMBOL_GPL(sc8547d_has_vooc_phy);

bool sc8547d_has_ufcs_phy(struct sc8547d *cp)
{
	return cp->ufcs_phy;
}
EXPORT_SYMBOL_GPL(sc8547d_has_ufcs_phy);

static void sc8547d_put_device(void *dev)
{
	put_device(dev);
}

/*
 * sc8547d_get() - look up a bound charge pump for a session driver
 * @consumer: the session device; it is unbound before the pump
 * @fwnode: the pump's node
 *
 * Return: the pump, or -EPROBE_DEFER until it is bound.
 */
struct sc8547d *sc8547d_get(struct device *consumer, struct fwnode_handle *fwnode)
{
	struct i2c_client *client;
	struct sc8547d *cp;
	int ret;

	client = i2c_find_device_by_fwnode(fwnode);
	if (!client)
		return ERR_PTR(-EPROBE_DEFER);
	ret = devm_add_action_or_reset(consumer, sc8547d_put_device, &client->dev);
	if (ret)
		return ERR_PTR(ret);
	if (!device_link_add(consumer, &client->dev, DL_FLAG_AUTOREMOVE_CONSUMER))
		return ERR_PTR(-EINVAL);
	cp = i2c_get_clientdata(client);
	return cp ?: ERR_PTR(-EPROBE_DEFER);
}
EXPORT_SYMBOL_GPL(sc8547d_get);

static int sc8547d_read_limit(struct device *dev, const char *name, u32 def,
			      u32 min, u32 step, u32 max_code, u8 *code)
{
	u32 value = def;

	device_property_read_u32(dev, name, &value);
	if (value < min || (value - min) % step || (value - min) / step > max_code)
		return dev_err_probe(dev, -EINVAL, "invalid %s %u\n", name, value);
	*code = (value - min) / step;
	return 0;
}

static int sc8547d_parse(struct sc8547d *cp)
{
	struct device *dev = cp->dev;
	u32 frequency = 500000;
	int ret;

	ret = sc8547d_read_limit(dev, "southchip,vbat-ovp-microvolt", 4850000,
				 VBAT_OVP_MIN_UV, VBAT_OVP_STEP_UV, 0x3f, &cp->vbat_ovp);
	if (!ret)
		ret = sc8547d_read_limit(dev, "southchip,vbus-ovp-microvolt", 10500000,
					 VBUS_OVP_MIN_UV, VBUS_OVP_STEP_UV, 0x7f, &cp->vbus_ovp);
	if (!ret)
		ret = sc8547d_read_limit(dev, "southchip,ibus-ocp-microamp", 5700000,
					 IBUS_OCP_MIN_UA, IBUS_OCP_STEP_UA, 0x0f, &cp->ibus_ocp);
	if (ret)
		return ret;
	device_property_read_u32(dev, "southchip,switching-frequency-hz", &frequency);
	if (frequency != 500000 && frequency != 600000)
		return dev_err_probe(dev, -EINVAL, "invalid switching frequency %u\n", frequency);
	cp->fsw = frequency == 600000 ? CONTROL_FSW_600K : 0;
	cp->always_otg = device_property_read_bool(dev, "southchip,always-otg-en");
	cp->vooc_phy = device_property_read_bool(dev, "southchip,vooc-phy");
	cp->ufcs_phy = device_property_read_bool(dev, "southchip,ufcs-phy");
	return 0;
}

static void sc8547d_cleanup(void *data)
{
	struct sc8547d *cp = data;

	mutex_lock(&cp->lock);
	if (sc8547d_off_locked(cp))
		dev_crit(cp->dev, "cannot stop the pump\n");
	sc8547d_write(cp, REG_UFCS_CTRL1, 0);
	sc8547d_write(cp, REG_VOOC_CTRL, 0);
	sc8547d_write(cp, REG_ADC_CTRL, 0);
	mutex_unlock(&cp->lock);
}

static void sc8547d_clear_clientdata(void *client)
{
	i2c_set_clientdata(client, NULL);
}

static const struct regmap_config sc8547d_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = REG_MAX,
};

static int sc8547d_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct sc8547d *cp;
	unsigned int id;
	int ret;

	cp = devm_kzalloc(dev, sizeof(*cp), GFP_KERNEL);
	if (!cp)
		return -ENOMEM;
	cp->dev = dev;
	mutex_init(&cp->lock);
	mutex_init(&cp->irq_lock);
	ret = sc8547d_parse(cp);
	if (ret)
		return ret;
	cp->regmap = devm_regmap_init_i2c(client, &sc8547d_regmap_config);
	if (IS_ERR(cp->regmap))
		return PTR_ERR(cp->regmap);
	ret = regmap_read(cp->regmap, REG_DEVICE_ID, &id);
	if (ret)
		return dev_err_probe(dev, ret, "cannot read the device ID\n");
	if (id != SC8547D_ID)
		return dev_err_probe(dev, -ENODEV, "unknown device ID %02x\n", id);
	ret = sc8547d_reset(cp);
	if (ret)
		return dev_err_probe(dev, ret, "cannot initialize\n");
	ret = devm_add_action_or_reset(dev, sc8547d_cleanup, cp);
	if (ret)
		return ret;
	if (client->irq > 0) {
		ret = devm_request_threaded_irq(dev, client->irq, NULL, sc8547d_irq,
						IRQF_ONESHOT, dev_name(dev), cp);
		if (ret)
			return dev_err_probe(dev, ret, "cannot request interrupt\n");
	} else if (cp->vooc_phy || cp->ufcs_phy) {
		return dev_err_probe(dev, -EINVAL, "a PHY needs an interrupt\n");
	}
	i2c_set_clientdata(client, cp);
	return devm_add_action_or_reset(dev, sc8547d_clear_clientdata, client);
}

static void sc8547d_shutdown(struct i2c_client *client)
{
	struct sc8547d *cp = i2c_get_clientdata(client);

	if (cp)
		sc8547d_cleanup(cp);
}

static const struct of_device_id sc8547d_of_match[] = {
	{ .compatible = "southchip,sc8547d" },
	{}
};
MODULE_DEVICE_TABLE(of, sc8547d_of_match);

static struct i2c_driver sc8547d_driver = {
	.driver = {
		.name = "sc8547d",
		.of_match_table = sc8547d_of_match,
	},
	.probe = sc8547d_probe,
	.shutdown = sc8547d_shutdown,
};
module_i2c_driver(sc8547d_driver);

MODULE_DESCRIPTION("Southchip SC8547D charge pump");
MODULE_LICENSE("GPL");
