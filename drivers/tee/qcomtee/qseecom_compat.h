/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef QCOMTEE_QSEECOM_COMPAT_H
#define QCOMTEE_QSEECOM_COMPAT_H

struct qcomtee;
struct qcomtee_uefisecapp;

#if IS_ENABLED(CONFIG_QCOMTEE_UEFISECAPP)
int qcomtee_uefisecapp_init(struct qcomtee *qcomtee);
void qcomtee_uefisecapp_remove(struct qcomtee *qcomtee);
#else
static inline int qcomtee_uefisecapp_init(struct qcomtee *qcomtee)
{
	return 0;
}

static inline void qcomtee_uefisecapp_remove(struct qcomtee *qcomtee) { }
#endif

#endif /* QCOMTEE_QSEECOM_COMPAT_H */
