/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _QCOM_BATTMGR_PPS_H
#define _QCOM_BATTMGR_PPS_H

#include <linux/types.h>

struct power_supply;
enum qcom_battmgr_pps_command {
	QCOM_BATTMGR_PPS_TYPE,
	QCOM_BATTMGR_PPS_CAPS,
	QCOM_BATTMGR_PPS_TARGET,
	QCOM_BATTMGR_PPS_BASELINE,
};

/* Caller provides 1, 7, 2, or 1 u32 values respectively. */
int qcom_battmgr_pps_request(struct power_supply *usb,
			   enum qcom_battmgr_pps_command command, u32 *values);
#endif
