// SPDX-License-Identifier: GPL-2.0-only
/* Qualcomm QTEE transport for the UEFI Secure Application. */

#include <linux/firmware/qcom/qcom_qseecom.h>
#include <linux/of.h>
#include <linux/rpmb.h>

#include "qcomtee.h"
#include "rpmb.h"
#include "qseecom_compat.h"

#define QCOMTEE_APP_LOADER_UID		0x7a
#define QCOMTEE_APP_LOOKUP_TA		2
#define QCOMTEE_APP_SEND_REQUEST		0
#define QCOMTEE_APP_DISCONNECT		1

struct qcomtee_uefi_client {
	struct qseecom_client client;
	struct tee_context *ctx;
	struct qcomtee_object *app;
	struct qcomtee_object_invoke_ctx close_oic;
};

struct qcomtee_uefisecapp {
	struct qcomtee *qcomtee;
	struct class_interface rpmb_intf;
	/* Serialize RPMB arrival/removal with asynchronous client setup. */
	struct mutex lock;
	struct work_struct work;
	struct rpmb_dev *rdev;
	struct qcomtee_rpmb *rpmb;
	struct qcomtee_object *env;
	struct qcomtee_object *loader;
	struct qcomtee_uefi_client *client;
};

/* EFI variables and the firmware RPMB listener are single-instance services. */
static DEFINE_MUTEX(qcomtee_uefi_owner_lock);
static struct qcomtee_uefisecapp *qcomtee_uefi_owner;

static int qcomtee_uefi_send(struct qseecom_client *client, void *req,
			     size_t req_size, void *rsp, size_t rsp_size)
{
	struct qcomtee_uefi_client *uefi =
		container_of(client, struct qcomtee_uefi_client, client);
	struct qcomtee_object_invoke_ctx *oic;
	u32 is64 = 1;
	int ret, result;
	struct qcomtee_arg args[] = {
		{ .type = QCOMTEE_ARG_TYPE_IB, .b = { .addr = req, .size = req_size } },
		{ .type = QCOMTEE_ARG_TYPE_IB, .b = { .addr = rsp, .size = rsp_size } },
		{ .type = QCOMTEE_ARG_TYPE_IB },
		{ .type = QCOMTEE_ARG_TYPE_IB, .b = { .addr = &is64, .size = sizeof(is64) } },
		{ .type = QCOMTEE_ARG_TYPE_OB, .b = { .addr = req, .size = req_size } },
		{ .type = QCOMTEE_ARG_TYPE_OB, .b = { .addr = rsp, .size = rsp_size } },
		{ .type = QCOMTEE_ARG_TYPE_IO },
		{ .type = QCOMTEE_ARG_TYPE_IO },
		{ .type = QCOMTEE_ARG_TYPE_IO },
		{ .type = QCOMTEE_ARG_TYPE_IO },
		{ .type = QCOMTEE_ARG_TYPE_INV },
	};

	/* The compat protocol sends the response area as both input and output. */
	memset(rsp, 0, rsp_size);
	oic = qcomtee_object_invoke_ctx_alloc(uefi->ctx);
	if (!oic)
		return -ENOMEM;

	ret = qcomtee_object_do_invoke(oic, uefi->app, QCOMTEE_APP_SEND_REQUEST,
				       args, &result);
	kfree(oic);
	if (ret)
		return ret;
	if (result) {
		dev_err_ratelimited(&client->aux_dev.dev,
				    "QTEE request failed: %d\n", result);
		return -EIO;
	}
	if (args[5].b.size < 3 * sizeof(u32))
		return -EPROTO;

	return 0;
}

static void qcomtee_uefi_put_app(struct qcomtee_uefi_client *client)
{
	struct qcomtee_arg args[] = { { .type = QCOMTEE_ARG_TYPE_INV } };
	int ret, result;

	if (!client->app)
		return;

	/* Disconnect preserves the bootloader-loaded TA when releasing its handle. */
	memset(&client->close_oic, 0, sizeof(client->close_oic));
	client->close_oic.ctx = client->ctx;
	ret = qcomtee_object_do_invoke(&client->close_oic, client->app,
				       QCOMTEE_APP_DISCONNECT, args, &result);
	if (ret || result)
		pr_warn("qcomtee: uefisecapp disconnect failed: %d (%d)\n",
			ret, ret ? 0 : result);
	qcomtee_object_put(client->app);
	client->app = NULL;
}

static void qcomtee_uefi_client_release(struct device *dev)
{
	struct qcomtee_uefi_client *client =
		container_of(dev, struct qcomtee_uefi_client, client.aux_dev.dev);

	qcomtee_uefi_put_app(client);
	kfree(client);
}

static int qcomtee_uefi_connect(struct qcomtee_uefisecapp *uefi)
{
	static const char name[] = "qcom.tz.uefisecapp";
	struct qcomtee_object_invoke_ctx *oic;
	struct qcomtee_uefi_client *client;
	u32 app_arch = 0;
	int ret, result;
	struct qcomtee_arg args[] = {
		{ .type = QCOMTEE_ARG_TYPE_IB,
		  .b = { .addr = (void *)name, .size = sizeof(name) - 1 } },
		{ .type = QCOMTEE_ARG_TYPE_OB,
		  .b = { .addr = &app_arch, .size = sizeof(app_arch) } },
		{ .type = QCOMTEE_ARG_TYPE_OO },
		{ .type = QCOMTEE_ARG_TYPE_INV },
	};

	oic = qcomtee_object_invoke_ctx_alloc(uefi->qcomtee->ctx);
	if (!oic)
		return -ENOMEM;

	uefi->loader = qcomtee_object_get_service(oic, uefi->env,
						  QCOMTEE_APP_LOADER_UID);
	kfree(oic);
	if (!uefi->loader)
		return -ENODEV;

	client = kzalloc_obj(*client);
	if (!client)
		return -ENOMEM;
	client->ctx = uefi->qcomtee->ctx;
	oic = &client->close_oic;
	oic->ctx = client->ctx;
	ret = qcomtee_object_do_invoke(oic, uefi->loader,
				       QCOMTEE_APP_LOOKUP_TA, args, &result);
	if (ret || result || !args[2].o) {
		pr_err("qcomtee: uefisecapp lookup failed: transport=%d result=%d\n",
		       ret, ret ? 0 : result);
		kfree(client);
		return ret ?: -EIO;
	}

	client->app = args[2].o;
	client->client.app_send = qcomtee_uefi_send;
	client->client.aux_dev.name = "uefisecapp";
	client->client.aux_dev.dev.parent = &uefi->rdev->dev;
	client->client.aux_dev.dev.release = qcomtee_uefi_client_release;
	ret = auxiliary_device_init(&client->client.aux_dev);
	if (ret) {
		qcomtee_uefi_put_app(client);
		kfree(client);
		return ret;
	}
	ret = auxiliary_device_add(&client->client.aux_dev);
	if (ret) {
		auxiliary_device_uninit(&client->client.aux_dev);
		return ret;
	}

	uefi->client = client;
	pr_info("qcomtee: uefisecapp connected with RPMB backing\n");
	return 0;
}

static void qcomtee_uefi_stop(struct qcomtee_uefisecapp *uefi)
{
	if (uefi->client) {
		auxiliary_device_delete(&uefi->client->client.aux_dev);
		qcomtee_uefi_put_app(uefi->client);
		auxiliary_device_uninit(&uefi->client->client.aux_dev);
		uefi->client = NULL;
	}

	qcomtee_object_put(uefi->loader);
	uefi->loader = NULL;
	/* Drain app releases before withdrawing its storage service. */
	flush_workqueue(uefi->qcomtee->wq);
	qcomtee_rpmb_unregister(uefi->rpmb);
	uefi->rpmb = NULL;
	qcomtee_object_put(uefi->env);
	uefi->env = NULL;
	flush_workqueue(uefi->qcomtee->wq);
}

static void qcomtee_uefi_start(struct work_struct *work)
{
	struct qcomtee_uefisecapp *uefi =
		container_of(work, struct qcomtee_uefisecapp, work);
	struct qcomtee_object_invoke_ctx *oic;
	int ret;

	guard(mutex)(&uefi->lock);
	if (!uefi->rdev || uefi->client)
		return;

	oic = qcomtee_object_invoke_ctx_alloc(uefi->qcomtee->ctx);
	if (!oic)
		return;
	uefi->env = qcomtee_object_get_client_env(oic);
	kfree(oic);
	if (!uefi->env) {
		pr_err("qcomtee: failed to obtain uefisecapp client environment\n");
		return;
	}

	uefi->rpmb = qcomtee_rpmb_register(uefi->qcomtee->ctx, uefi->env,
					   uefi->rdev);
	if (IS_ERR(uefi->rpmb)) {
		ret = PTR_ERR(uefi->rpmb);
		uefi->rpmb = NULL;
		goto fail;
	}

	ret = qcomtee_uefi_connect(uefi);
	if (!ret)
		return;
fail:
	pr_err("qcomtee: uefisecapp initialization failed: %d\n", ret);
	qcomtee_uefi_stop(uefi);
}

static int qcomtee_uefi_rpmb_add(struct device *dev)
{
	struct qcomtee_uefisecapp *uefi = qcomtee_uefi_owner;
	struct rpmb_dev *rdev = to_rpmb_dev(dev);

	/* Android's RPMB listener uses UFS region 0 (SPSP 0x0001). */
	if (rdev->descr.type != RPMB_TYPE_UFS || !dev->parent ||
	    strcmp(dev_name(dev->parent), "ufs_rpmb0"))
		return 0;
	if (!rdev->descr.capacity || !rdev->descr.reliable_wr_count)
		return -EINVAL;

	guard(mutex)(&uefi->lock);
	if (uefi->rdev)
		return -EBUSY;
	uefi->rdev = rpmb_dev_get(rdev);
	schedule_work(&uefi->work);
	return 0;
}

static void qcomtee_uefi_rpmb_remove(struct device *dev)
{
	struct qcomtee_uefisecapp *uefi = qcomtee_uefi_owner;

	guard(mutex)(&uefi->lock);
	if (uefi->rdev != to_rpmb_dev(dev))
		return;
	qcomtee_uefi_stop(uefi);
	rpmb_dev_put(uefi->rdev);
	uefi->rdev = NULL;
}

int qcomtee_uefisecapp_init(struct qcomtee *qcomtee)
{
	struct qcomtee_uefisecapp *uefi;
	int ret;

	if (!of_machine_is_compatible("oneplus,iceland"))
		return 0;

	guard(mutex)(&qcomtee_uefi_owner_lock);
	if (qcomtee_uefi_owner)
		return -EBUSY;

	uefi = kzalloc_obj(*uefi);
	if (!uefi)
		return -ENOMEM;
	uefi->qcomtee = qcomtee;
	mutex_init(&uefi->lock);
	INIT_WORK(&uefi->work, qcomtee_uefi_start);
	uefi->rpmb_intf.add_dev = qcomtee_uefi_rpmb_add;
	uefi->rpmb_intf.remove_dev = qcomtee_uefi_rpmb_remove;
	qcomtee_uefi_owner = uefi;
	ret = rpmb_interface_register(&uefi->rpmb_intf);
	if (ret) {
		qcomtee_uefi_owner = NULL;
		kfree(uefi);
		return ret;
	}
	qcomtee->uefisecapp = uefi;
	return 0;
}

void qcomtee_uefisecapp_remove(struct qcomtee *qcomtee)
{
	struct qcomtee_uefisecapp *uefi = qcomtee->uefisecapp;

	if (!uefi)
		return;

	guard(mutex)(&qcomtee_uefi_owner_lock);
	rpmb_interface_unregister(&uefi->rpmb_intf);
	cancel_work_sync(&uefi->work);
	qcomtee_uefi_owner = NULL;
	kfree(uefi);
	qcomtee->uefisecapp = NULL;
}
