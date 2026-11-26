/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Driver for Qualcomm Secure Execution Environment (SEE) interface (QSEECOM).
 * Responsible for setting up and managing QSEECOM client devices.
 *
 * Copyright (C) 2023 Maximilian Luz <luzmaximilian@gmail.com>
 */

#ifndef __QCOM_QSEECOM_H
#define __QCOM_QSEECOM_H

#include <linux/auxiliary_bus.h>
#include <linux/dma-mapping.h>
#include <linux/types.h>

#include <linux/firmware/qcom/qcom_scm.h>

/**
 * struct qseecom_client - QSEECOM client device.
 * @aux_dev: Underlying auxiliary device.
 * @app_id: ID of the loaded application, used by the legacy SCM transport.
 * @app_send: Optional synchronous transport callback with the same arguments
 *            and return convention as qcom_qseecom_app_send(). If NULL, use
 *            qcom_scm_qseecom_app_send() with @app_id.
 */
struct qseecom_client {
	struct auxiliary_device aux_dev;
	u32 app_id;
	int (*app_send)(struct qseecom_client *client, void *req, size_t req_size,
			void *rsp, size_t rsp_size);
};

/**
 * qcom_qseecom_app_send() - Send to and receive data from a given QSEE app.
 * @client:   The QSEECOM client associated with the target app.
 * @req:      Request buffer sent to the app (must be TZ memory).
 * @req_size: Size of the request buffer.
 * @rsp:      Response buffer, written to by the app (must be TZ memory).
 * @rsp_size: Size of the response buffer.
 *
 * Sends a request to the QSEE app associated with the given client and read
 * back its response. The caller must provide two DMA memory regions, one for
 * the request and one for the response, and fill out the @req region with the
 * respective (app-specific) request data. The QSEE app reads this and returns
 * its response in the @rsp region.
 *
 * Uses the client's optional app_send callback if provided, otherwise falls
 * back to qcom_scm_qseecom_app_send(). The response must be available before
 * the callback returns; it must not retain the request or response buffers.
 * Clients should prefer to use this wrapper.
 *
 * Return: Zero on successful transfer, nonzero on transport failure. Any
 * app-specific status is returned in the response buffer.
 */
static inline int qcom_qseecom_app_send(struct qseecom_client *client,
					void *req, size_t req_size,
					void *rsp, size_t rsp_size)
{
	if (client->app_send)
		return client->app_send(client, req, req_size, rsp, rsp_size);

	return qcom_scm_qseecom_app_send(client->app_id, req, req_size, rsp, rsp_size);
}

#endif /* __QCOM_QSEECOM_H */
