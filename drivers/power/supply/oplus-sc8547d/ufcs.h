/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * UFCS (Universal Fast Charging Specification) sink protocol over the
 * SC8547D UFCS PHY.
 */
#ifndef _OPLUS_FC_UFCS_H
#define _OPLUS_FC_UFCS_H

#include <linux/completion.h>
#include <linux/mutex.h>
#include <linux/types.h>

struct sc8547d;

#define UFCS_PDO_MAX		7

/* Output capability, request and source info field accessors (BE u64). */
#define UFCS_CAP_INDEX(m)	(((m) >> 60) & 0xf)
#define UFCS_CAP_CURR_STEP(m)	(((((m) >> 57) & 0x7) + 1) * 10)
#define UFCS_CAP_VOL_STEP(m)	(((((m) >> 56) & 0x1) + 1) * 10)
#define UFCS_CAP_VOL_MAX(m)	((((m) >> 40) & 0xffff) * 10)
#define UFCS_CAP_VOL_MIN(m)	((((m) >> 24) & 0xffff) * 10)
#define UFCS_CAP_CURR_MAX(m)	((((m) >> 8) & 0xffff) * 10)
#define UFCS_CAP_CURR_MIN(m)	(((m) & 0xff) * 10)
#define UFCS_SRC_VOL(i)		((((i) >> 16) & 0xffff) * 10)
#define UFCS_SRC_CURR(i)	(((i) & 0xffff) * 10)
#define UFCS_DEV_VENDOR(i)	(((i) >> 48) & 0xffff)
#define UFCS_DEV_HW_VER(i)	(((i) >> 16) & 0xffff)

struct ufcs_sink_info {
	int batt_temp_c;
	int usb_temp_c;
	int vbat_mv;
	int ibat_ma;
};

/**
 * struct ufcs - one UFCS link
 * @cp: the SC8547D whose UFCS PHY carries the link
 * @get_sink_info: fills what the adapter asks about the battery
 */
struct ufcs {
	struct sc8547d *cp;
	struct device *dev;
	void (*get_sink_info)(void *data, struct ufcs_sink_info *info);
	void *sink_data;

	/* private */
	struct mutex lock;		/* one transaction at a time */
	struct mutex tx_lock;		/* one message in the PHY transmit buffer */
	unsigned int tx_seq;		/* owner of the next sent flag */
	unsigned int sent_seq;
	struct work_struct reply_work;	/* answers to adapter-initiated messages */
	spinlock_t rx_lock;
	u8 pending[8];			/* type, index, cmd, len, payload */
	int pending_len;		/* 0 when nothing to answer */
	bool pending_data;
	struct completion handshake;
	struct completion sent;
	struct completion response;
	bool linked;
	bool hard_reset;
	bool exited;
	int handshake_result;
	int send_result;
	u8 msg_index;
	u8 want_type;
	u8 want_cmd;
	u8 want_cmd2;
	u8 rx[64];
	int rx_len;
	bool rx_valid;
	u64 pdo[UFCS_PDO_MAX];
	int pdo_count;
	u32 power_change[UFCS_PDO_MAX];
	bool power_changed;
};

void ufcs_init(struct ufcs *u, struct device *dev, struct sc8547d *cp);
void ufcs_irq(void *data);
int ufcs_handshake(struct ufcs *u);
int ufcs_get_device_info(struct ufcs *u, u64 *info);
int ufcs_get_output_caps(struct ufcs *u, u64 *pdo, int *count);
int ufcs_get_source_info(struct ufcs *u, u64 *info);
int ufcs_config_watchdog(struct ufcs *u, u16 ms);
int ufcs_request(struct ufcs *u, int mv, int ma);
int ufcs_exit(struct ufcs *u);
bool ufcs_linked(struct ufcs *u);

#endif
