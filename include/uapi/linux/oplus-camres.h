/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
#ifndef _UAPI_OPLUS_CAMRES_H
#define _UAPI_OPLUS_CAMRES_H

#include <linux/ioctl.h>
#include <linux/types.h>

/*
 * /dev/camres - ownership choreography of the camera I2C bus shared
 * between the AP and the SEE.
 *
 * The device node is created root:root 0600; granting access is purely
 * a userspace/udev matter.  Every ioctl below requires the file to
 * have been opened for writing (FMODE_WRITE), so node permissions are
 * the only authority - there are no capability checks.
 *
 * Lifecycle:
 *
 *   fd = open("/dev/camres", O_RDWR)
 *   ioctl(fd, CAMRES_IOC_DEINIT)        -> blocks until every AP user
 *                                           (device-link runtime PM,
 *                                           ioctl and sysfs votes) is
 *                                           done, unsubscribes AON,
 *                                           makes this fd the owner
 *   ... run the SEE INIT handshake from userspace ...
 *   ioctl(fd, CAMRES_IOC_INIT, &ack_rc) -> ack 0: normal arbitration
 *                                           ack != 0: back to the
 *                                           unconfigured (AP-held) mode
 *
 * If the owner fd is destroyed before INIT, the bus enters the ERR
 * state: acquires fail until a fresh DEINIT/INIT cycle recovers it.
 */

#define CAMRES_IOC_DEINIT	_IO('c', 0)
#define CAMRES_IOC_INIT		_IOW('c', 1, __s32)
#define CAMRES_IOC_ACQUIRE	_IO('c', 2)
#define CAMRES_IOC_RELEASE	_IO('c', 3)

#endif /* _UAPI_OPLUS_CAMRES_H */
