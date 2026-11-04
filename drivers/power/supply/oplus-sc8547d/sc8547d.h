/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Southchip SC8547D charge pump with VOOC and UFCS PHYs.
 *
 * Register sequences follow the Oplus SC8547D HAL
 * (vendor/oplus/kernel/charger/v2/ufcs_ic/oplus_hal_sc8547d.c),
 * Copyright (C) 2022-2023 Oplus. All rights reserved.
 */
#ifndef _OPLUS_SC8547D_H
#define _OPLUS_SC8547D_H

#include <linux/types.h>

struct device;
struct fwnode_handle;
struct sc8547d;

enum sc8547d_mode {
	SC8547D_MODE_2_TO_1,
	SC8547D_MODE_BYPASS,
};

/**
 * struct sc8547d_sample - one ADC and status snapshot
 * @ibus_ma: input current.
 * @vbus_mv: input voltage behind the input switch.
 * @vac_mv: connector side voltage.
 * @vout_mv: output voltage.
 * @vbat_mv: battery voltage at the pump's sense pins.
 * @tdie_dc: die temperature in tenths of a degree Celsius.
 * @switching: the converter is switching.
 * @fault: a latched protection bit; see sc8547d_fault_name().
 */
struct sc8547d_sample {
	int ibus_ma;
	int vbus_mv;
	int vac_mv;
	int vout_mv;
	int vbat_mv;
	int tdie_dc;
	bool switching;
	unsigned int fault;
};

#define SC8547D_FAULT_TSHUT		BIT(0)
#define SC8547D_FAULT_SS_TIMEOUT	BIT(1)
#define SC8547D_FAULT_VOUT_OVP		BIT(2)
#define SC8547D_FAULT_VBAT_OVP		BIT(3)
#define SC8547D_FAULT_IBAT_OCP		BIT(4)
#define SC8547D_FAULT_VBUS_OVP		BIT(5)
#define SC8547D_FAULT_IBUS_OCP		BIT(6)
#define SC8547D_FAULT_WATCHDOG		BIT(7)
#define SC8547D_FAULT_PMID2OUT		BIT(8)
#define SC8547D_FAULT_PIN_DIAG		BIT(9)

/* VOOC PHY flags, register 0x2f */
#define SC8547D_VOOC_TX_DONE		BIT(0)
#define SC8547D_VOOC_RX_DONE		BIT(1)
#define SC8547D_VOOC_RX_START		BIT(2)
#define SC8547D_VOOC_TX_FAILED		BIT(3)
#define SC8547D_VOOC_ERR_TRANS		BIT(4)
#define SC8547D_VOOC_TXSEQ_DONE		BIT(5)
#define SC8547D_VOOC_NINTH_CLK_ERR	BIT(6)
#define SC8547D_VOOC_PULSE_FILTERED	BIT(7)
#define SC8547D_VOOC_ERRORS		(SC8547D_VOOC_TX_FAILED | SC8547D_VOOC_ERR_TRANS | \
					 SC8547D_VOOC_NINTH_CLK_ERR | SC8547D_VOOC_PULSE_FILTERED)
/* Interrupt masks: replies only, or also the end of a current reply. */
#define SC8547D_VOOC_MASK_RX		0x05
#define SC8547D_VOOC_MASK_TX		0x04

/* UFCS flags, registers 0x43..0x45 */
#define SC8547D_UFCS_ACK_TIMEOUT	BIT(0)
#define SC8547D_UFCS_TRANS_FAIL		BIT(1)
#define SC8547D_UFCS_RX_BUSY		BIT(2)
#define SC8547D_UFCS_RX_OVERFLOW	BIT(3)
#define SC8547D_UFCS_DATA_READY		BIT(4)
#define SC8547D_UFCS_SENT		BIT(5)
#define SC8547D_UFCS_HANDSHAKE_OK	BIT(6)
#define SC8547D_UFCS_HANDSHAKE_FAIL	BIT(7)
#define SC8547D_UFCS2_HARD_RESET	BIT(0)
#define SC8547D_UFCS2_CRC_ERROR		BIT(1)
#define SC8547D_UFCS2_STOP_ERROR	BIT(2)
#define SC8547D_UFCS2_START_FAIL	BIT(3)
#define SC8547D_UFCS2_LENGTH_ERROR	BIT(4)
#define SC8547D_UFCS2_BYTE_TIMEOUT	BIT(5)
#define SC8547D_UFCS2_TRAINING_ERROR	BIT(6)
#define SC8547D_UFCS2_BAUD_ERROR	BIT(7)
#define SC8547D_UFCS3_BAUD_CHANGE	BIT(6)
#define SC8547D_UFCS3_BUS_CONFLICT	BIT(7)
#define SC8547D_UFCS_MSG_MAX		64

struct sc8547d *sc8547d_get(struct device *consumer, struct fwnode_handle *fwnode);
struct device *sc8547d_device(struct sc8547d *cp);
bool sc8547d_has_vooc_phy(struct sc8547d *cp);
bool sc8547d_has_ufcs_phy(struct sc8547d *cp);
const char *sc8547d_fault_name(unsigned int fault);

/* Charge pump path */
int sc8547d_cp_prepare(struct sc8547d *cp, enum sc8547d_mode mode);
int sc8547d_cp_start(struct sc8547d *cp, bool on);
int sc8547d_cp_stop(struct sc8547d *cp);
int sc8547d_cp_watchdog_kick(struct sc8547d *cp);
int sc8547d_cp_ucp_enable(struct sc8547d *cp, bool enable);
int sc8547d_sample(struct sc8547d *cp, struct sc8547d_sample *s);
int sc8547d_switching(struct sc8547d *cp, bool *on);

/* Interrupt dispatch to the protocol that owns the PHY */
void sc8547d_set_irq_handler(struct sc8547d *cp, void (*handler)(void *data), void *data);

/* VOOC PHY */
int sc8547d_vooc_enable(struct sc8547d *cp);
int sc8547d_vooc_disable(struct sc8547d *cp);
int sc8547d_vooc_handshake(struct sc8547d *cp);
int sc8547d_vooc_rx(struct sc8547d *cp, u8 *data, u8 *flags);
int sc8547d_vooc_tx(struct sc8547d *cp, u16 frame);
int sc8547d_vooc_predata(struct sc8547d *cp, u16 frame);
int sc8547d_vooc_irq_mask(struct sc8547d *cp, u8 mask);

/* UFCS PHY */
int sc8547d_ufcs_enable(struct sc8547d *cp);
int sc8547d_ufcs_disable(struct sc8547d *cp);
int sc8547d_ufcs_handshake(struct sc8547d *cp);
int sc8547d_ufcs_set_baud(struct sc8547d *cp, unsigned int baud);
int sc8547d_ufcs_hard_reset(struct sc8547d *cp);
int sc8547d_ufcs_read_flags(struct sc8547d *cp, u8 flags[3]);
int sc8547d_ufcs_write_msg(struct sc8547d *cp, const u8 *buf, int len);
int sc8547d_ufcs_read_msg(struct sc8547d *cp, u8 *buf, int cap);
int sc8547d_ufcs_clear_rx(struct sc8547d *cp);

#endif
