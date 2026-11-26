/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef QCOMTEE_RPMB_H
#define QCOMTEE_RPMB_H

struct qcomtee_object;
struct qcomtee_rpmb;
struct rpmb_dev;
struct tee_context;

/**
 * qcomtee_rpmb_register() - Register the QTEE UFS RPMB listener
 * @ctx: kernel TEE context used to allocate shared memory and invoke QTEE
 * @env: client environment with permission to open service UID 0x57
 * @rdev: UFS RPMB region 0 device selected by the caller
 *
 * Register before opening applications that need RPMB. The listener takes its
 * own reference to @rdev; @env is borrowed only for this call. Shared memory
 * holds a reference to @ctx until QTEE releases the listener and its mappings.
 *
 * Context: process context, may sleep.
 * Return: listener handle on success, or ERR_PTR() on failure.
 */
struct qcomtee_rpmb *qcomtee_rpmb_register(struct tee_context *ctx,
					   struct qcomtee_object *env,
					   struct rpmb_dev *rdev);

/**
 * qcomtee_rpmb_unregister() - Stop and release the QTEE RPMB listener
 * @rpmb: listener returned by qcomtee_rpmb_register(), or NULL
 *
 * The caller must stop applications using the listener first. This consumes
 * the handle and starts asynchronous release of the registration object.
 * Resources stay alive until the last callback and QTEE memory object
 * references are released. The caller must keep the QTEE transport available
 * while those releases drain, and must not use the handle after this call.
 *
 * Context: process context, may sleep.
 */
void qcomtee_rpmb_unregister(struct qcomtee_rpmb *rpmb);

#endif /* QCOMTEE_RPMB_H */
