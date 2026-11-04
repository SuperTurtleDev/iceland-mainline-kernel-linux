/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _QCOM_BATTMGR_PPS_H
#define _QCOM_BATTMGR_PPS_H

#include <linux/types.h>

struct power_supply;

/*
 * Direct control of the OnePlus Kaanapali battery manager for a kernel
 * fast charging policy. All requests share the battery manager transaction
 * lock. Values: TYPE[1 out], CAPS[7 out], TARGET[mV, mA], ICL[uA],
 * CHARGE_ENABLE[0/1], PDO_5V[none], INPUT_SUSPEND[0/1],
 * ADAPTER_INFO[3 out: raw Oplus type, subtype, ADSP service generation].
 */
enum qcom_battmgr_pps_command {
	QCOM_BATTMGR_PPS_TYPE,
	QCOM_BATTMGR_PPS_CAPS,
	QCOM_BATTMGR_PPS_TARGET,
	QCOM_BATTMGR_PPS_ICL,
	QCOM_BATTMGR_PPS_CHARGE_ENABLE,
	QCOM_BATTMGR_PPS_PDO_5V,
	QCOM_BATTMGR_PPS_INPUT_SUSPEND,
	QCOM_BATTMGR_PPS_ADAPTER_INFO,
};

int qcom_battmgr_pps_request(struct power_supply *usb,
			     enum qcom_battmgr_pps_command command, u32 *values);
#endif
