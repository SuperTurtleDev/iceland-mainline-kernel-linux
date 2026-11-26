// SPDX-License-Identifier: GPL-2.0-only
/* Qualcomm QTEE callback listener for UFS RPMB region 0. */

#include <linux/atomic.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/mm.h>
#include <linux/overflow.h>
#include <linux/rpmb.h>
#include <linux/slab.h>
#include <linux/unaligned.h>
#include <linux/wait.h>

#include "qcomtee.h"
#include "rpmb.h"

#define QCOMTEE_RPMB_REGISTER_UID	0x57
#define QCOMTEE_RPMB_REGISTER_OP		0
#define QCOMTEE_RPMB_LISTENER_ID		0x2000
#define QCOMTEE_RPMB_REQUEST_OP		0
#define QCOMTEE_RPMB_WAIT_OP		1
#define QCOMTEE_RPMB_BUFFER_SIZE		0x6400
#define QCOMTEE_RPMB_WAIT_MS		4500
#define QCOMTEE_RPMB_UFS_TYPE		9

#define QCOMTEE_RPMB_INIT		0x101
#define QCOMTEE_RPMB_READ		0x102
#define QCOMTEE_RPMB_WRITE		0x103
#define QCOMTEE_RPMB_PARTITION		0x104

/* Listener headers are little endian; JEDEC RPMB frames are big endian. */
struct qcomtee_rpmb_init_req {
	__le32 cmd;
	__le32 version;
};

struct qcomtee_rpmb_init_rsp {
	__le32 cmd;
	__le32 version;
	__le32 status;
	__le32 sectors;
	__le32 reliable_wr_count;
};

struct qcomtee_rpmb_init_rsp_v2 {
	struct qcomtee_rpmb_init_rsp v1;
	__le32 dev_type;
	__le32 reserved[4];
};

struct qcomtee_rpmb_rw_req {
	__le32 cmd;
	__le32 num_sectors;
	__le32 req_buff_len;
	__le32 req_buff_offset;
	__le32 version;
	__le32 rel_wr_count;
};

struct qcomtee_rpmb_rw_rsp {
	__le32 cmd;
	__le32 status;
	__le32 res_buff_len;
	__le32 res_buff_offset;
	__le32 version;
};

struct qcomtee_rpmb_partition_req {
	__le32 cmd;
	__le32 version;
	__le32 dev_id;
};

struct qcomtee_rpmb_partition_rsp {
	__le32 cmd;
	__le32 status;
	__le32 num_partitions;
	__le32 rsp_buff_offset;
};

static_assert(sizeof(struct rpmb_frame) == 512);
static_assert(sizeof(struct qcomtee_rpmb_init_req) == 8);
static_assert(sizeof(struct qcomtee_rpmb_init_rsp) == 20);
static_assert(sizeof(struct qcomtee_rpmb_init_rsp_v2) == 40);
static_assert(sizeof(struct qcomtee_rpmb_rw_req) == 24);
static_assert(sizeof(struct qcomtee_rpmb_rw_rsp) == 20);
static_assert(sizeof(struct qcomtee_rpmb_partition_req) == 12);
static_assert(sizeof(struct qcomtee_rpmb_partition_rsp) == 16);

enum qcomtee_rpmb_stat {
	RPMB_STAT_INIT,
	RPMB_STAT_READ,
	RPMB_STAT_WRITE,
	RPMB_STAT_PARTITION,
	RPMB_STAT_ERRORS,
	RPMB_STAT_ROUTES,
	RPMB_STAT_RESULT_ERRORS,
	RPMB_STAT_BUSY,
	RPMB_STAT_WAITS,
	RPMB_STAT_WAIT_TIMEOUTS,
	RPMB_STAT_COUNT,
};

struct qcomtee_rpmb {
	struct qcomtee_object object;
	struct qcomtee_object *registration;
	struct qcomtee_object *mem_object;
	struct tee_shm *shm;
	struct rpmb_dev *rdev;
	void *shared;
	/* kmalloc buffers are also suitable for the storage driver's DMA I/O. */
	u8 *request;
	u8 *response;
	atomic_t busy;
	bool stopping;
	wait_queue_head_t waitq;
	atomic64_t stats[RPMB_STAT_COUNT];
};

static int qcomtee_rpmb_init(struct qcomtee_rpmb *rpmb)
{
	const struct qcomtee_rpmb_init_req *req = (void *)rpmb->request;
	struct qcomtee_rpmb_init_rsp_v2 *rsp = (void *)rpmb->response;
	u32 version = le32_to_cpu(req->version);
	int ret = 0;

	rsp->v1.cmd = req->cmd;
	rsp->v1.version = cpu_to_le32(version >= 2 ? 2 : 1);
	if (!version) {
		ret = -EINVAL;
	} else {
		/* Capacity is in 128 KiB units; QTEE expects 512-byte sectors. */
		rsp->v1.sectors = cpu_to_le32((u32)rpmb->rdev->descr.capacity * 256);
		rsp->v1.reliable_wr_count =
			cpu_to_le32(rpmb->rdev->descr.reliable_wr_count);
		if (version >= 2)
			rsp->dev_type = cpu_to_le32(QCOMTEE_RPMB_UFS_TYPE);
	}
	rsp->v1.status = cpu_to_le32(ret);

	return ret;
}

/* Validate every frame before starting I/O, including later write batches. */
static int qcomtee_rpmb_check_frames(u8 *frames, u32 count, u16 type)
{
	struct rpmb_frame *frame;
	u16 frame_type;
	u32 i;

	for (i = 0; i < count; i++) {
		frame = (void *)(frames + i * sizeof(*frame));
		frame_type = get_unaligned_be16(&frame->req_resp);
		/* Key provisioning is deliberately unavailable through this service. */
		if (frame_type == RPMB_PROGRAM_KEY)
			return -EACCES;
		if (frame_type != type)
			return -EINVAL;
	}

	return 0;
}

static int qcomtee_rpmb_route(struct qcomtee_rpmb *rpmb, u8 *req,
			      u32 req_len, u8 *rsp, u32 rsp_len)
{
	int ret;

	atomic64_inc(&rpmb->stats[RPMB_STAT_ROUTES]);
	ret = rpmb_route_frames(rpmb->rdev, req, req_len, rsp, rsp_len);
	/* Some SCSI backends return a positive command status, never success. */
	return ret > 0 ? -EIO : ret;
}

static int qcomtee_rpmb_read(struct qcomtee_rpmb *rpmb, u8 *frames,
			     u32 req_len, u32 count, u8 *response)
{
	struct rpmb_frame *frame = (void *)frames;
	u32 limit = rpmb->rdev->descr.reliable_wr_count;
	u32 requests, batch, done = 0;
	u16 type = get_unaligned_be16(&frame->req_resp);
	int ret;

	if (type == RPMB_PROGRAM_KEY)
		return -EACCES;
	if (type == RPMB_GET_WRITE_COUNTER) {
		if (count != 1)
			return -EINVAL;
		requests = 1;
	} else if (type == RPMB_READ_DATA) {
		/* count is the number of response frames, not request frames. */
		requests = DIV_ROUND_UP(count, limit);
	} else {
		return -EINVAL;
	}

	if (req_len != requests * sizeof(*frame))
		return -EINVAL;
	ret = qcomtee_rpmb_check_frames(frames, requests, type);
	if (ret)
		return ret;

	while (done < count) {
		batch = min(limit, count - done);
		ret = qcomtee_rpmb_route(rpmb, frames, sizeof(*frame),
					 response + done * sizeof(*frame),
					 batch * sizeof(*frame));
		if (ret)
			return ret;
		frames += sizeof(*frame);
		done += batch;
	}

	return 0;
}

static int qcomtee_rpmb_write(struct qcomtee_rpmb *rpmb, u8 *frames,
			      u32 count, u32 batch, u8 *response)
{
	struct rpmb_frame *result = (void *)response;
	u32 done;
	int ret;

	/* TZ has already signed each batch. Never split or re-sign it. */
	if (!batch || batch > rpmb->rdev->descr.reliable_wr_count || count % batch)
		return -EINVAL;
	ret = qcomtee_rpmb_check_frames(frames, count, RPMB_WRITE_DATA);
	if (ret)
		return ret;

	for (done = 0; done < count; done += batch) {
		ret = qcomtee_rpmb_route(rpmb, frames + done * sizeof(*result),
					 batch * sizeof(*result), response, sizeof(*result));
		if (ret)
			return ret;
		/*
		 * Android returns one result frame, from the last transaction.
		 * Preserve an earlier device error instead of overwriting it with
		 * a later success. TZ must authenticate and interpret the result.
		 */
		if (get_unaligned_be16(&result->result)) {
			atomic64_inc(&rpmb->stats[RPMB_STAT_RESULT_ERRORS]);
			break;
		}
	}

	return 0;
}

static int qcomtee_rpmb_rw(struct qcomtee_rpmb *rpmb, bool write)
{
	const struct qcomtee_rpmb_rw_req *req = (void *)rpmb->request;
	struct qcomtee_rpmb_rw_rsp *rsp = (void *)rpmb->response;
	u32 count = le32_to_cpu(req->num_sectors);
	u32 len = le32_to_cpu(req->req_buff_len);
	u32 offset = le32_to_cpu(req->req_buff_offset);
	u32 frame_bytes, response_len;
	u8 *payload = rpmb->response + sizeof(*rsp);
	int ret = -EINVAL;

	rsp->cmd = req->cmd;
	rsp->version = req->version;
	rsp->res_buff_offset = cpu_to_le32(sizeof(*rsp));

	/* Subtract before checking length so offset + length cannot wrap. */
	if (offset < sizeof(*req) || offset > QCOMTEE_RPMB_BUFFER_SIZE ||
	    !len || len > QCOMTEE_RPMB_BUFFER_SIZE - offset ||
	    len % sizeof(struct rpmb_frame) || !count)
		goto out;
	if (check_mul_overflow(count, (u32)sizeof(struct rpmb_frame), &frame_bytes)) {
		ret = -EOVERFLOW;
		goto out;
	}
	/* This bound also makes all later frame and batch arithmetic safe. */
	if (frame_bytes > QCOMTEE_RPMB_BUFFER_SIZE - sizeof(*rsp)) {
		ret = -EMSGSIZE;
		goto out;
	}

	if (write) {
		if (len != frame_bytes)
			goto out;
		response_len = sizeof(struct rpmb_frame);
		ret = qcomtee_rpmb_write(rpmb, rpmb->request + offset, count,
					 le32_to_cpu(req->rel_wr_count), payload);
	} else {
		response_len = frame_bytes;
		ret = qcomtee_rpmb_read(rpmb, rpmb->request + offset, len, count, payload);
	}
	if (!ret)
		rsp->res_buff_len = cpu_to_le32(response_len);

out:
	if (ret) {
		/* Discard partial I/O results, including previous successful batches. */
		memset(payload, 0, QCOMTEE_RPMB_BUFFER_SIZE - sizeof(*rsp));
		rsp->res_buff_len = 0;
	}
	rsp->status = cpu_to_le32(ret);
	return ret;
}

static int qcomtee_rpmb_partition(struct qcomtee_rpmb *rpmb)
{
	const struct qcomtee_rpmb_partition_req *req = (void *)rpmb->request;
	struct qcomtee_rpmb_partition_rsp *rsp = (void *)rpmb->response;

	dev_dbg(&rpmb->rdev->dev, "QTEE RPMB partition version=%#x dev_id=%u\n",
		le32_to_cpu(req->version), le32_to_cpu(req->dev_id));
	/* No rpmb_sec_parti.cfg is available; never invent a partition table. */
	rsp->cmd = req->cmd;
	rsp->status = cpu_to_le32(-1);
	rsp->num_partitions = 0;
	rsp->rsp_buff_offset = cpu_to_le32(sizeof(*rsp));

	return -1;
}

static int qcomtee_rpmb_process(struct qcomtee_rpmb *rpmb)
{
	u32 cmd = get_unaligned_le32(rpmb->request);
	int status, ret = 0;

	if (cmd >= QCOMTEE_RPMB_INIT && cmd <= QCOMTEE_RPMB_PARTITION)
		atomic64_inc(&rpmb->stats[cmd - QCOMTEE_RPMB_INIT]);

	switch (cmd) {
	case QCOMTEE_RPMB_INIT:
		status = qcomtee_rpmb_init(rpmb);
		break;
	case QCOMTEE_RPMB_READ:
		status = qcomtee_rpmb_rw(rpmb, false);
		break;
	case QCOMTEE_RPMB_WRITE:
		status = qcomtee_rpmb_rw(rpmb, true);
		break;
	case QCOMTEE_RPMB_PARTITION:
		status = qcomtee_rpmb_partition(rpmb);
		break;
	default:
		status = -EOPNOTSUPP;
		ret = QCOMTEE_MSG_ERROR_INVALID;
		break;
	}
	if (status)
		atomic64_inc(&rpmb->stats[RPMB_STAT_ERRORS]);
	dev_dbg(&rpmb->rdev->dev, "QTEE RPMB cmd=%#x status=%d\n", cmd, status);

	/* Known commands report storage errors in their shared-memory response. */
	return ret;
}

static int qcomtee_rpmb_request(struct qcomtee_rpmb *rpmb,
				struct qcomtee_arg *args)
{
	int i, ret;

	/* IListenerCBO.request: counts 0x4020, two OBs and four OOs. */
	if (qcomtee_args_len(args) != 6 ||
	    args[0].type != QCOMTEE_ARG_TYPE_OB ||
	    args[1].type != QCOMTEE_ARG_TYPE_OB ||
	    args[1].b.size != sizeof(u32) || !args[1].b.addr)
		return QCOMTEE_MSG_ERROR_INVALID;
	for (i = 0; i < 6; i++) {
		if (args[i].flags || (i >= 2 && args[i].type != QCOMTEE_ARG_TYPE_OO))
			return QCOMTEE_MSG_ERROR_INVALID;
	}
	args[0].b.size = 0;
	put_unaligned_le32(0, args[1].b.addr);
	for (i = 2; i < 6; i++)
		args[i].o = NULL_QCOMTEE_OBJECT;

	if (READ_ONCE(rpmb->stopping))
		return -ENODEV;
	if (atomic_cmpxchg(&rpmb->busy, 0, 1)) {
		atomic64_inc(&rpmb->stats[RPMB_STAT_BUSY]);
		return -EBUSY;
	}
	if (READ_ONCE(rpmb->stopping)) {
		ret = -ENODEV;
		goto out;
	}

	/* Snapshot before parsing; request and response must never alias. */
	memcpy(rpmb->request, rpmb->shared, QCOMTEE_RPMB_BUFFER_SIZE);
	memset(rpmb->response, 0, QCOMTEE_RPMB_BUFFER_SIZE);
	ret = qcomtee_rpmb_process(rpmb);
	memcpy(rpmb->shared, rpmb->response, QCOMTEE_RPMB_BUFFER_SIZE);

out:
	memzero_explicit(rpmb->request, QCOMTEE_RPMB_BUFFER_SIZE);
	memzero_explicit(rpmb->response, QCOMTEE_RPMB_BUFFER_SIZE);
	atomic_set_release(&rpmb->busy, 0);
	wake_up_all(&rpmb->waitq);
	return ret;
}

static int qcomtee_rpmb_wait(struct qcomtee_rpmb *rpmb)
{
	long left;

	atomic64_inc(&rpmb->stats[RPMB_STAT_WAITS]);
	left = wait_event_timeout(rpmb->waitq, READ_ONCE(rpmb->stopping) ||
				  !atomic_read_acquire(&rpmb->busy),
				  msecs_to_jiffies(QCOMTEE_RPMB_WAIT_MS));
	if (READ_ONCE(rpmb->stopping))
		return -ENODEV;
	if (!left)
		atomic64_inc(&rpmb->stats[RPMB_STAT_WAIT_TIMEOUTS]);
	/* As in Android, a timeout lets QTEE queue another attempt. */
	return 0;
}

static int qcomtee_rpmb_dispatch(struct qcomtee_object_invoke_ctx *oic,
				 struct qcomtee_object *object, u32 op,
				 struct qcomtee_arg *args)
{
	struct qcomtee_rpmb *rpmb = container_of(object, struct qcomtee_rpmb, object);
	int i, ret;

	switch (op) {
	case QCOMTEE_RPMB_REQUEST_OP:
		ret = qcomtee_rpmb_request(rpmb, args);
		break;
	case QCOMTEE_RPMB_WAIT_OP:
		ret = qcomtee_args_len(args) ? QCOMTEE_MSG_ERROR_INVALID :
			qcomtee_rpmb_wait(rpmb);
		break;
	default:
		ret = QCOMTEE_MSG_ERROR_INVALID;
		break;
	}
	/* Malformed calls may carry input objects; dispatch owns those references. */
	qcomtee_arg_for_each_input_object(i, args)
		qcomtee_object_put(args[i].o);
	return ret;
}

static void qcomtee_rpmb_free(struct qcomtee_rpmb *rpmb)
{
	qcomtee_object_put(rpmb->mem_object);
	if (rpmb->shm)
		tee_shm_put(rpmb->shm);
	rpmb_dev_put(rpmb->rdev);
	kfree_sensitive(rpmb->request);
	kfree_sensitive(rpmb->response);
	kfree(rpmb);
}

static void qcomtee_rpmb_release(struct qcomtee_object *object)
{
	struct qcomtee_rpmb *rpmb = container_of(object, struct qcomtee_rpmb, object);

	dev_dbg(&rpmb->rdev->dev,
		"QTEE RPMB released: init=%lld read=%lld write=%lld partition=%lld errors=%lld routes=%lld result_errors=%lld busy=%lld waits=%lld timeouts=%lld\n",
		atomic64_read(&rpmb->stats[RPMB_STAT_INIT]),
		atomic64_read(&rpmb->stats[RPMB_STAT_READ]),
		atomic64_read(&rpmb->stats[RPMB_STAT_WRITE]),
		atomic64_read(&rpmb->stats[RPMB_STAT_PARTITION]),
		atomic64_read(&rpmb->stats[RPMB_STAT_ERRORS]),
		atomic64_read(&rpmb->stats[RPMB_STAT_ROUTES]),
		atomic64_read(&rpmb->stats[RPMB_STAT_RESULT_ERRORS]),
		atomic64_read(&rpmb->stats[RPMB_STAT_BUSY]),
		atomic64_read(&rpmb->stats[RPMB_STAT_WAITS]),
		atomic64_read(&rpmb->stats[RPMB_STAT_WAIT_TIMEOUTS]));
	/* Any remaining QTEE mapping retains the memory object's own shm ref. */
	qcomtee_rpmb_free(rpmb);
}

static struct qcomtee_object_operations qcomtee_rpmb_ops = {
	.release = qcomtee_rpmb_release,
	.dispatch = qcomtee_rpmb_dispatch,
};

struct qcomtee_rpmb *qcomtee_rpmb_register(struct tee_context *ctx,
					   struct qcomtee_object *env,
					   struct rpmb_dev *rdev)
{
	struct qcomtee_object_invoke_ctx *oic;
	struct qcomtee_arg args[4] = { };
	struct qcomtee_rpmb *rpmb;
	u32 uid = QCOMTEE_RPMB_REGISTER_UID;
	u32 listener_id = QCOMTEE_RPMB_LISTENER_ID;
	int ret, result = 0, i;

	if (IS_ERR_OR_NULL(ctx) || IS_ERR_OR_NULL(env) || IS_ERR_OR_NULL(rdev) ||
	    typeof_qcomtee_object(env) != QCOMTEE_OBJECT_TYPE_TEE)
		return ERR_PTR(-EINVAL);

	rpmb = kzalloc_obj(*rpmb);
	if (!rpmb)
		return ERR_PTR(-ENOMEM);
	rpmb->rdev = rpmb_dev_get(rdev);
	if (!rpmb->rdev) {
		ret = -ENODEV;
		goto err_free;
	}
	/* Region selection belongs to the caller; this protocol only supports UFS. */
	if (rdev->descr.type != RPMB_TYPE_UFS) {
		ret = -EOPNOTSUPP;
		goto err_free;
	}
	if (!rdev->descr.route_frames || !rdev->descr.capacity ||
	    !rdev->descr.reliable_wr_count) {
		ret = -EINVAL;
		goto err_free;
	}

	atomic_set(&rpmb->busy, 0);
	init_waitqueue_head(&rpmb->waitq);
	for (i = 0; i < RPMB_STAT_COUNT; i++)
		atomic64_set(&rpmb->stats[i], 0);
	rpmb->request = kmalloc(QCOMTEE_RPMB_BUFFER_SIZE, GFP_KERNEL);
	rpmb->response = kmalloc(QCOMTEE_RPMB_BUFFER_SIZE, GFP_KERNEL);
	if (!rpmb->request || !rpmb->response) {
		ret = -ENOMEM;
		goto err_free;
	}
	rpmb->shm = tee_shm_alloc_kernel_buf(ctx, PAGE_ALIGN(QCOMTEE_RPMB_BUFFER_SIZE));
	if (IS_ERR(rpmb->shm)) {
		ret = PTR_ERR(rpmb->shm);
		rpmb->shm = NULL;
		goto err_free;
	}
	rpmb->shared = tee_shm_get_va(rpmb->shm, 0);
	if (IS_ERR(rpmb->shared)) {
		ret = PTR_ERR(rpmb->shared);
		goto err_free;
	}
	/* Also clear page-alignment padding visible through the memory object. */
	memset(rpmb->shared, 0, tee_shm_get_size(rpmb->shm));
	rpmb->mem_object = qcomtee_mem_object_alloc(rpmb->shm);
	if (IS_ERR(rpmb->mem_object)) {
		ret = PTR_ERR(rpmb->mem_object);
		rpmb->mem_object = NULL;
		goto err_free;
	}
	ret = qcomtee_object_user_init(&rpmb->object, QCOMTEE_OBJECT_TYPE_CB,
				       &qcomtee_rpmb_ops, "rpmb-listener");
	if (ret)
		goto err_free;

	oic = qcomtee_object_invoke_ctx_alloc(ctx);
	if (!oic) {
		ret = -ENOMEM;
		goto err_unregister;
	}
	/* Open directly so transport errno is not discarded by get_service(). */
	args[0].type = QCOMTEE_ARG_TYPE_IB;
	args[0].b.addr = &uid;
	args[0].b.size = sizeof(uid);
	args[1].type = QCOMTEE_ARG_TYPE_OO;
	ret = qcomtee_object_do_invoke(oic, env, QCOMTEE_CLIENT_ENV_OPEN, args, &result);
	if (ret || result) {
		dev_dbg(&rdev->dev, "QTEE RPMB service open failed: ret=%d result=%d\n",
			ret, result);
		ret = ret ?: -EREMOTEIO;
		goto err_oic;
	}
	rpmb->registration = args[1].o;
	if (typeof_qcomtee_object(rpmb->registration) != QCOMTEE_OBJECT_TYPE_TEE) {
		ret = -EPROTO;
		goto err_oic;
	}

	/* A fresh invocation must not inherit the previous call's SHARED flag. */
	memset(oic, 0, sizeof(*oic));
	oic->ctx = ctx;
	memset(args, 0, sizeof(args));
	args[0].type = QCOMTEE_ARG_TYPE_IB;
	args[0].b.addr = &listener_id;
	args[0].b.size = sizeof(listener_id);
	args[1].type = QCOMTEE_ARG_TYPE_IO;
	args[1].o = &rpmb->object;
	args[2].type = QCOMTEE_ARG_TYPE_IO;
	args[2].o = rpmb->mem_object;
	/*
	 * counts 0x201. Invocation consumes these two CB references, including
	 * failure paths. Keep the original references until listener release.
	 */
	qcomtee_object_get(&rpmb->object);
	qcomtee_object_get(rpmb->mem_object);
	result = 0;
	ret = qcomtee_object_do_invoke(oic, rpmb->registration,
				       QCOMTEE_RPMB_REGISTER_OP, args, &result);
	if (ret || result) {
		dev_dbg(&rdev->dev, "QTEE RPMB register failed: ret=%d result=%d\n",
			ret, result);
		ret = ret ?: -EREMOTEIO;
		goto err_oic;
	}
	kfree(oic);
	dev_dbg(&rdev->dev, "QTEE RPMB listener %#x registered, sectors=%u reliable_wr=%u\n",
		listener_id, (u32)rdev->descr.capacity * 256, rdev->descr.reliable_wr_count);
	return rpmb;

err_oic:
	kfree(oic);
err_unregister:
	qcomtee_rpmb_unregister(rpmb);
	return ERR_PTR(ret);
err_free:
	qcomtee_rpmb_free(rpmb);
	return ERR_PTR(ret);
}

void qcomtee_rpmb_unregister(struct qcomtee_rpmb *rpmb)
{
	if (IS_ERR_OR_NULL(rpmb))
		return;

	WRITE_ONCE(rpmb->stopping, true);
	wake_up_all(&rpmb->waitq);
	/*
	 * Releasing the registration asks QTEE to release its listener and
	 * memory references asynchronously. Only drop our ownership here:
	 * the callback's release, not unregister, frees its working buffers,
	 * rdev and shm. The memory object separately pins shm for mappings.
	 */
	qcomtee_object_put(rpmb->registration);
	rpmb->registration = NULL;
	qcomtee_object_put(&rpmb->object);
}
