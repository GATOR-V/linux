// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2026 NXP
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/errno.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/riscv_rpmi_tee.h>
#include <linux/rhashtable.h>
#include <linux/rpmb.h>
#include <linux/scatterlist.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/tee_core.h>
#include <linux/types.h>
#include <linux/uuid.h>
#include "optee_private.h"
#include "optee_rpc_cmd.h"
#include "optee_rpmi.h"

/*
 * This file implements the OP-TEE ABI over the TEE service group of the
 * RISC-V Platform Management Interface (RPMI). It mirrors ffa_abi.c: the
 * RPMI TEE framework driver plays the role of the FF-A driver, TEE_CALL
 * that of FF-A direct requests and memory parcels that of FF-A memory
 * sharing. The OP-TEE service invoked with TEE_CALL is defined in
 * optee_rpmi.h.
 *
 * 1. Shared memory handles
 * 2. Convert between struct tee_param and struct optee_msg_param
 * 3. Register shared memory with OP-TEE
 * 4. Dynamic shared memory pool based on alloc_pages()
 * 5. Do a normal scheduled call into OP-TEE
 * 6. Driver initialization
 */

static const uuid_t optee_rpmi_service_uuid = OPTEE_RPMI_SERVICE_UUID;

/*
 * Send a struct optee_rpmi_msg to OP-TEE and get the response message
 * back. Framework errors, when OP-TEE was not reached, are returned as
 * negative error codes.
 */
static int optee_rpmi_msg_call(struct optee *optee,
			       const struct optee_rpmi_msg *req,
			       struct optee_rpmi_msg *rsp)
{
	size_t len = 0;
	int rc;

	memset(rsp, 0, sizeof(*rsp));
	rc = rpmi_tee_call(optee->rpmi.tdev, &optee_rpmi_service_uuid,
			   req, sizeof(*req), rsp, sizeof(*rsp), &len);
	if (rc)
		return rc;
	if (len != sizeof(*rsp))
		return -EIO;

	return 0;
}

/*
 * 1. Shared memory handles
 *
 * Each piece of shared memory is a memory parcel identified by a 64-bit
 * handle which is then used when communicating with OP-TEE, like
 * the FF-A global memory handle.
 *
 * Main functions are optee_shm_add_rpmi_handle() and
 * optee_shm_rem_rpmi_handle()
 */

struct shm_rhash {
	struct tee_shm *shm;
	u64 global_id;
	struct rhash_head linkage;
};

static void rh_free_fn(void *ptr, void *arg)
{
	kfree(ptr);
}

static const struct rhashtable_params shm_rhash_params = {
	.head_offset = offsetof(struct shm_rhash, linkage),
	.key_len     = sizeof(u64),
	.key_offset  = offsetof(struct shm_rhash, global_id),
	.automatic_shrinking = true,
};

static struct tee_shm *optee_shm_from_rpmi_handle(struct optee *optee,
						  u64 global_id)
{
	struct tee_shm *shm = NULL;
	struct shm_rhash *r;

	mutex_lock(&optee->rpmi.mutex);
	r = rhashtable_lookup_fast(&optee->rpmi.global_ids, &global_id,
				   shm_rhash_params);
	if (r)
		shm = r->shm;
	mutex_unlock(&optee->rpmi.mutex);

	return shm;
}

static int optee_shm_add_rpmi_handle(struct optee *optee, struct tee_shm *shm,
				     u64 global_id)
{
	struct shm_rhash *r;
	int rc;

	r = kmalloc_obj(*r);
	if (!r)
		return -ENOMEM;
	r->shm = shm;
	r->global_id = global_id;

	mutex_lock(&optee->rpmi.mutex);
	rc = rhashtable_lookup_insert_fast(&optee->rpmi.global_ids,
					   &r->linkage, shm_rhash_params);
	mutex_unlock(&optee->rpmi.mutex);

	if (rc)
		kfree(r);

	return rc;
}

static int optee_shm_rem_rpmi_handle(struct optee *optee, u64 global_id)
{
	struct shm_rhash *r;
	int rc = -ENOENT;

	mutex_lock(&optee->rpmi.mutex);
	r = rhashtable_lookup_fast(&optee->rpmi.global_ids, &global_id,
				   shm_rhash_params);
	if (r)
		rc = rhashtable_remove_fast(&optee->rpmi.global_ids,
					    &r->linkage, shm_rhash_params);
	mutex_unlock(&optee->rpmi.mutex);

	if (!rc)
		kfree(r);

	return rc;
}

/*
 * 2. Convert between struct tee_param and struct optee_msg_param
 *
 * optee_rpmi_from_msg_param() and optee_rpmi_to_msg_param() are the main
 * functions.
 */

static void from_msg_param_rpmi_mem(struct optee *optee, struct tee_param *p,
				    u32 attr, const struct optee_msg_param *mp)
{
	struct tee_shm *shm = NULL;
	u64 offs_high = 0;
	u64 offs_low = 0;

	p->attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INPUT +
		  attr - OPTEE_MSG_ATTR_TYPE_FMEM_INPUT;
	p->u.memref.size = mp->u.fmem.size;

	if (mp->u.fmem.global_id != OPTEE_MSG_FMEM_INVALID_GLOBAL_ID)
		shm = optee_shm_from_rpmi_handle(optee, mp->u.fmem.global_id);
	p->u.memref.shm = shm;

	if (shm) {
		offs_low = mp->u.fmem.offs_low;
		offs_high = mp->u.fmem.offs_high;
	}
	p->u.memref.shm_offs = offs_low | offs_high << 32;
}

/**
 * optee_rpmi_from_msg_param() - convert from OPTEE_MSG parameters to
 *				 struct tee_param
 * @optee:	main service struct
 * @params:	subsystem internal parameter representation
 * @num_params:	number of elements in the parameter arrays
 * @msg_params:	OPTEE_MSG parameters
 *
 * Returns 0 on success or <0 on failure
 */
static int optee_rpmi_from_msg_param(struct optee *optee,
				     struct tee_param *params, size_t num_params,
				     const struct optee_msg_param *msg_params)
{
	size_t n;

	for (n = 0; n < num_params; n++) {
		struct tee_param *p = params + n;
		const struct optee_msg_param *mp = msg_params + n;
		u32 attr = mp->attr & OPTEE_MSG_ATTR_TYPE_MASK;

		switch (attr) {
		case OPTEE_MSG_ATTR_TYPE_NONE:
			p->attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
			memset(&p->u, 0, sizeof(p->u));
			break;
		case OPTEE_MSG_ATTR_TYPE_VALUE_INPUT:
		case OPTEE_MSG_ATTR_TYPE_VALUE_OUTPUT:
		case OPTEE_MSG_ATTR_TYPE_VALUE_INOUT:
			optee_from_msg_param_value(p, attr, mp);
			break;
		case OPTEE_MSG_ATTR_TYPE_FMEM_INPUT:
		case OPTEE_MSG_ATTR_TYPE_FMEM_OUTPUT:
		case OPTEE_MSG_ATTR_TYPE_FMEM_INOUT:
			from_msg_param_rpmi_mem(optee, p, attr, mp);
			break;
		default:
			return -EINVAL;
		}
	}

	return 0;
}

static int to_msg_param_rpmi_mem(struct optee_msg_param *mp,
				 const struct tee_param *p)
{
	struct tee_shm *shm = p->u.memref.shm;

	mp->attr = OPTEE_MSG_ATTR_TYPE_FMEM_INPUT + p->attr -
		   TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INPUT;

	if (shm) {
		u64 shm_offs = p->u.memref.shm_offs;

		mp->u.fmem.internal_offs = shm->offset;

		mp->u.fmem.offs_low = shm_offs;
		mp->u.fmem.offs_high = shm_offs >> 32;
		/* Check that the entire offset could be stored. */
		if (mp->u.fmem.offs_high != shm_offs >> 32)
			return -EINVAL;

		mp->u.fmem.global_id = shm->sec_world_id;
	} else {
		memset(&mp->u, 0, sizeof(mp->u));
		mp->u.fmem.global_id = OPTEE_MSG_FMEM_INVALID_GLOBAL_ID;
	}
	mp->u.fmem.size = p->u.memref.size;

	return 0;
}

/**
 * optee_rpmi_to_msg_param() - convert from struct tee_params to OPTEE_MSG
 *			       parameters
 * @optee:	main service struct
 * @msg_params:	OPTEE_MSG parameters
 * @num_params:	number of elements in the parameter arrays
 * @params:	subsystem internal parameter representation
 * Returns 0 on success or <0 on failure
 */
static int optee_rpmi_to_msg_param(struct optee *optee,
				   struct optee_msg_param *msg_params,
				   size_t num_params,
				   const struct tee_param *params)
{
	size_t n;

	for (n = 0; n < num_params; n++) {
		const struct tee_param *p = params + n;
		struct optee_msg_param *mp = msg_params + n;

		switch (p->attr) {
		case TEE_IOCTL_PARAM_ATTR_TYPE_NONE:
			mp->attr = TEE_IOCTL_PARAM_ATTR_TYPE_NONE;
			memset(&mp->u, 0, sizeof(mp->u));
			break;
		case TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT:
		case TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_OUTPUT:
		case TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT:
			optee_to_msg_param_value(mp, p);
			break;
		case TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INPUT:
		case TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_OUTPUT:
		case TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT:
			if (to_msg_param_rpmi_mem(mp, p))
				return -EINVAL;
			break;
		default:
			return -EINVAL;
		}
	}

	return 0;
}

/*
 * 3. Register shared memory with OP-TEE
 *
 * The memory is shared with OP-TEE as a memory parcel and the handle of
 * the parcel is used to refer to it in the OP-TEE message protocol.
 */

static int optee_rpmi_shm_register(struct tee_context *ctx,
				   struct tee_shm *shm, struct page **pages,
				   size_t num_pages, unsigned long start)
{
	struct optee *optee = tee_get_drvdata(ctx->teedev);
	struct rpmi_tee_device *tdev = optee->rpmi.tdev;
	struct sg_table sgt;
	u64 handle;
	int rc;

	rc = optee_check_mem_type(start, num_pages);
	if (rc)
		return rc;

	rc = sg_alloc_table_from_pages(&sgt, pages, num_pages, 0,
				       num_pages * PAGE_SIZE, GFP_KERNEL);
	if (rc)
		return rc;
	rc = rpmi_tee_mem_share(tdev, sgt.sgl, RPMI_TEE_MEM_ACCESS_RW,
				&handle);
	sg_free_table(&sgt);
	if (rc)
		return rc;

	rc = optee_shm_add_rpmi_handle(optee, shm, handle);
	if (rc) {
		rpmi_tee_mem_reclaim(tdev, handle);
		return rc;
	}

	shm->sec_world_id = handle;

	return 0;
}

static int optee_rpmi_shm_unregister(struct tee_context *ctx,
				     struct tee_shm *shm)
{
	struct optee *optee = tee_get_drvdata(ctx->teedev);
	struct rpmi_tee_device *tdev = optee->rpmi.tdev;
	u64 handle = shm->sec_world_id;
	struct optee_rpmi_msg req = {
		.w[0] = cpu_to_le32(OPTEE_RPMI_UNREGISTER_SHM),
		.w[1] = cpu_to_le32(lower_32_bits(handle)),
		.w[2] = cpu_to_le32(upper_32_bits(handle)),
	};
	struct optee_rpmi_msg rsp;
	int rc;

	/*
	 * A shm whose parcel was already released and reclaimed (e.g. an RPC
	 * buffer dropped by handle_rpmi_rpc_func_cmd_shm_free()) reaches here
	 * with a cleared handle. There is nothing to unregister or reclaim;
	 * a real parcel never has handle 0 (parcel id starts at 1).
	 */
	if (!handle)
		return 0;

	optee_shm_rem_rpmi_handle(optee, handle);
	shm->sec_world_id = 0;

	rc = optee_rpmi_msg_call(optee, &req, &rsp);
	if (!rc && le32_to_cpu(rsp.w[0]))
		rc = -EIO;
	if (rc)
		pr_err("Unregister SHM id 0x%llx rc %d\n", handle, rc);

	rc = rpmi_tee_mem_reclaim(tdev, handle);
	if (rc)
		pr_err("mem_reclaim: 0x%llx %d", handle, rc);

	return rc;
}

static int optee_rpmi_shm_unregister_supp(struct tee_context *ctx,
					  struct tee_shm *shm)
{
	struct optee *optee = tee_get_drvdata(ctx->teedev);
	struct rpmi_tee_device *tdev = optee->rpmi.tdev;
	u64 handle = shm->sec_world_id;
	int rc;

	/*
	 * We're skipping the OPTEE_RPMI_UNREGISTER_SHM call since this
	 * memory is not mapped in OP-TEE.
	 */

	/* Already released and reclaimed (see optee_rpmi_shm_unregister()). */
	if (!handle)
		return 0;

	optee_shm_rem_rpmi_handle(optee, handle);
	shm->sec_world_id = 0;

	rc = rpmi_tee_mem_reclaim(tdev, handle);
	if (rc)
		pr_err("mem_reclaim: 0x%llx %d", handle, rc);

	return rc;
}

/*
 * 4. Dynamic shared memory pool based on alloc_pages()
 *
 * Shared memory can be allocated from a fixed pool of memory or it can
 * be allocated from a pool of pages, this pool of pages is used for
 * the OP-TEE RPMI ABI.
 */

static int pool_rpmi_op_alloc(struct tee_shm_pool *pool,
			      struct tee_shm *shm, size_t size, size_t align)
{
	return tee_dyn_shm_alloc_helper(shm, size, align,
					optee_rpmi_shm_register);
}

static void pool_rpmi_op_free(struct tee_shm_pool *pool,
			      struct tee_shm *shm)
{
	tee_dyn_shm_free_helper(shm, optee_rpmi_shm_unregister);
}

static void pool_rpmi_op_destroy_pool(struct tee_shm_pool *pool)
{
	kfree(pool);
}

static const struct tee_shm_pool_ops pool_rpmi_ops = {
	.alloc = pool_rpmi_op_alloc,
	.free = pool_rpmi_op_free,
	.destroy_pool = pool_rpmi_op_destroy_pool,
};

/**
 * optee_rpmi_shm_pool_alloc_pages() - create page-based allocator pool
 *
 * This pool is used with OP-TEE over RPMI. In this case command buffers
 * and such are allocated from kernel's own memory.
 */
static struct tee_shm_pool *optee_rpmi_shm_pool_alloc_pages(void)
{
	struct tee_shm_pool *pool = kzalloc_obj(*pool);

	if (!pool)
		return ERR_PTR(-ENOMEM);

	pool->ops = &pool_rpmi_ops;

	return pool;
}

/*
 * 5. Do a normal scheduled call into OP-TEE
 *
 * The function optee_rpmi_do_call_with_arg() performs a normal scheduled
 * call into OP-TEE. During this call may OP-TEE request help from the REE
 * using RPCs, Remote Procedure Calls. This includes delivery of the REE
 * interrupts to for instance allow rescheduling of the current task.
 */

static void handle_rpmi_rpc_func_cmd_shm_alloc(struct tee_context *ctx,
					       struct optee *optee,
					       struct optee_msg_arg *arg)
{
	struct tee_shm *shm;

	if (arg->num_params != 1 ||
	    arg->params[0].attr != OPTEE_MSG_ATTR_TYPE_VALUE_INPUT) {
		arg->ret = TEEC_ERROR_BAD_PARAMETERS;
		return;
	}

	switch (arg->params[0].u.value.a) {
	case OPTEE_RPC_SHM_TYPE_APPL:
		shm = optee_rpc_cmd_alloc_suppl(ctx, arg->params[0].u.value.b);
		break;
	case OPTEE_RPC_SHM_TYPE_KERNEL:
		shm = tee_shm_alloc_priv_buf(optee->ctx,
					     arg->params[0].u.value.b);
		break;
	default:
		arg->ret = TEEC_ERROR_BAD_PARAMETERS;
		return;
	}

	if (IS_ERR(shm)) {
		arg->ret = TEEC_ERROR_OUT_OF_MEMORY;
		return;
	}

	/*
	 * Buffers allocated for an RPC do not come from the registered
	 * shared memory pool, so they are not memory parcels yet. Share
	 * them so that OP-TEE gets a valid parcel handle as global ID.
	 */
	if (!shm->sec_world_id) {
		struct sg_table sgt;
		struct page **pages;
		size_t num_pages;
		u64 handle;
		int rc;

		pages = tee_shm_get_pages(shm, &num_pages);
		if (IS_ERR(pages) || !num_pages) {
			arg->ret = TEEC_ERROR_OUT_OF_MEMORY;
			goto err_free;
		}
		rc = sg_alloc_table_from_pages(&sgt, pages, num_pages, 0,
					       num_pages * PAGE_SIZE,
					       GFP_KERNEL);
		if (rc) {
			arg->ret = TEEC_ERROR_OUT_OF_MEMORY;
			goto err_free;
		}
		rc = rpmi_tee_mem_share(optee->rpmi.tdev, sgt.sgl,
					RPMI_TEE_MEM_ACCESS_RW, &handle);
		sg_free_table(&sgt);
		if (rc) {
			arg->ret = TEEC_ERROR_OUT_OF_MEMORY;
			goto err_free;
		}
		rc = optee_shm_add_rpmi_handle(optee, shm, handle);
		if (rc) {
			rpmi_tee_mem_reclaim(optee->rpmi.tdev, handle);
			arg->ret = TEEC_ERROR_OUT_OF_MEMORY;
			goto err_free;
		}
		shm->sec_world_id = handle;
	}

	arg->params[0] = (struct optee_msg_param){
		.attr = OPTEE_MSG_ATTR_TYPE_FMEM_OUTPUT,
		.u.fmem.size = tee_shm_get_size(shm),
		.u.fmem.global_id = shm->sec_world_id,
		.u.fmem.internal_offs = shm->offset,
	};

	arg->ret = TEEC_SUCCESS;
	return;

err_free:
	tee_shm_free(shm);
}

static void handle_rpmi_rpc_func_cmd_shm_free(struct tee_context *ctx,
					      struct optee *optee,
					      struct optee_msg_arg *arg)
{
	struct tee_shm *shm;

	if (arg->num_params != 1 ||
	    arg->params[0].attr != OPTEE_MSG_ATTR_TYPE_VALUE_INPUT)
		goto err_bad_param;

	shm = optee_shm_from_rpmi_handle(optee, arg->params[0].u.value.b);
	if (!shm)
		goto err_bad_param;
	switch (arg->params[0].u.value.a) {
	case OPTEE_RPC_SHM_TYPE_APPL:
	case OPTEE_RPC_SHM_TYPE_KERNEL:
		break;
	default:
		goto err_bad_param;
	}

	/* Drop the parcel shared in handle_rpmi_rpc_func_cmd_shm_alloc() */
	if (shm->sec_world_id) {
		u64 handle = shm->sec_world_id;

		optee_shm_rem_rpmi_handle(optee, handle);
		shm->sec_world_id = 0;
		rpmi_tee_mem_reclaim(optee->rpmi.tdev, handle);
	}

	if (arg->params[0].u.value.a == OPTEE_RPC_SHM_TYPE_APPL)
		optee_rpc_cmd_free_suppl(ctx, shm);
	else
		tee_shm_free(shm);
	arg->ret = TEEC_SUCCESS;
	return;

err_bad_param:
	arg->ret = TEEC_ERROR_BAD_PARAMETERS;
}

static void handle_rpmi_rpc_func_cmd(struct tee_context *ctx,
				     struct optee *optee,
				     struct optee_msg_arg *arg)
{
	arg->ret_origin = TEEC_ORIGIN_COMMS;
	switch (arg->cmd) {
	case OPTEE_RPC_CMD_SHM_ALLOC:
		handle_rpmi_rpc_func_cmd_shm_alloc(ctx, optee, arg);
		break;
	case OPTEE_RPC_CMD_SHM_FREE:
		handle_rpmi_rpc_func_cmd_shm_free(ctx, optee, arg);
		break;
	default:
		optee_rpc_cmd(ctx, optee, arg);
	}
}

static void optee_handle_rpmi_rpc(struct tee_context *ctx, struct optee *optee,
				  u32 cmd, struct optee_msg_arg *arg)
{
	switch (cmd) {
	case OPTEE_RPMI_YIELDING_CALL_RETURN_RPC_CMD:
		handle_rpmi_rpc_func_cmd(ctx, optee, arg);
		break;
	case OPTEE_RPMI_YIELDING_CALL_RETURN_INTERRUPT:
		/* Interrupt delivered by now */
		break;
	default:
		pr_warn("Unknown RPC func 0x%x\n", cmd);
		break;
	}
}

static int optee_rpmi_yielding_call(struct tee_context *ctx,
				    const struct optee_rpmi_msg *start,
				    struct optee_msg_arg *rpc_arg,
				    bool system_thread)
{
	struct optee *optee = tee_get_drvdata(ctx->teedev);
	struct optee_rpmi_msg req = *start;
	struct optee_rpmi_msg rsp;
	struct optee_call_waiter w;
	u32 ret;
	int rc;

	/* Initialize waiter */
	optee_cq_wait_init(&optee->call_queue, &w, system_thread);
	while (true) {
		rc = optee_rpmi_msg_call(optee, &req, &rsp);
		if (rc)
			goto done;

		switch (le32_to_cpu(rsp.w[0])) {
		case TEEC_SUCCESS:
			break;
		case TEEC_ERROR_BUSY:
			if (le32_to_cpu(req.w[0]) ==
			    OPTEE_RPMI_YIELDING_CALL_RESUME) {
				rc = -EIO;
				goto done;
			}

			/*
			 * Out of threads in OP-TEE, wait for a thread
			 * become available.
			 */
			optee_cq_wait_for_completion(&optee->call_queue, &w);
			req = *start;
			continue;
		default:
			rc = -EIO;
			goto done;
		}

		ret = le32_to_cpu(rsp.w[1]);
		if (ret == OPTEE_RPMI_YIELDING_CALL_RETURN_DONE)
			goto done;

		/*
		 * OP-TEE has returned with a RPC request.
		 *
		 * Note that the resume information in rsp.w[4] has to be
		 * passed back with the resume request.
		 */
		cond_resched();
		optee_handle_rpmi_rpc(ctx, optee, ret, rpc_arg);
		memset(&req, 0, sizeof(req));
		req.w[0] = cpu_to_le32(OPTEE_RPMI_YIELDING_CALL_RESUME);
		req.w[4] = rsp.w[4];
	}
done:
	/*
	 * We're done with our thread in OP-TEE, if there's any
	 * thread waiters wake up one.
	 */
	optee_cq_wait_final(&optee->call_queue, &w);

	return rc;
}

/**
 * optee_rpmi_do_call_with_arg() - Do a TEE_CALL to enter OP-TEE
 * @ctx:	calling context
 * @shm:	shared memory holding the message to pass to OP-TEE
 * @offs:	offset of the message in @shm
 * @system_thread: true if caller requests TEE system thread support
 *
 * Does a TEE_CALL to OP-TEE and handles eventual resulting
 * Remote Procedure Calls (RPC) from OP-TEE.
 *
 * Returns return code from the framework, 0 is OK
 */
static int optee_rpmi_do_call_with_arg(struct tee_context *ctx,
				       struct tee_shm *shm, u_int offs,
				       bool system_thread)
{
	struct optee_rpmi_msg req = {
		.w[0] = cpu_to_le32(OPTEE_RPMI_YIELDING_CALL_WITH_ARG),
		.w[1] = cpu_to_le32(lower_32_bits(shm->sec_world_id)),
		.w[2] = cpu_to_le32(upper_32_bits(shm->sec_world_id)),
		.w[3] = cpu_to_le32(offs),
	};
	struct optee_msg_arg *arg;
	unsigned int rpc_arg_offs;
	struct optee_msg_arg *rpc_arg;

	/*
	 * The shared memory object has to start on a page when passed as
	 * an argument struct. This is also what the shm pool allocator
	 * returns, but check this before calling OP-TEE to catch
	 * eventual errors early in case something changes.
	 */
	if (shm->offset)
		return -EINVAL;

	arg = tee_shm_get_va(shm, offs);
	if (IS_ERR(arg))
		return PTR_ERR(arg);

	rpc_arg_offs = OPTEE_MSG_GET_ARG_SIZE(arg->num_params);
	rpc_arg = tee_shm_get_va(shm, offs + rpc_arg_offs);
	if (IS_ERR(rpc_arg))
		return PTR_ERR(rpc_arg);

	return optee_rpmi_yielding_call(ctx, &req, rpc_arg, system_thread);
}

/*
 * 6. Driver initialization
 *
 * During driver inititialization is the OP-TEE service probed to find
 * out which features it supports so the driver can be initialized with
 * a matching configuration.
 */

static bool optee_rpmi_get_os_revision(struct optee *optee)
{
	struct optee_rpmi_msg req = {
		.w[0] = cpu_to_le32(OPTEE_RPMI_GET_OS_VERSION),
	};
	struct optee_rpmi_msg rsp;
	u32 major, minor, build_id;
	int rc;

	rc = optee_rpmi_msg_call(optee, &req, &rsp);
	if (rc) {
		pr_err("Unexpected error %d\n", rc);
		return false;
	}

	major = le32_to_cpu(rsp.w[0]);
	minor = le32_to_cpu(rsp.w[1]);
	build_id = le32_to_cpu(rsp.w[2]);
	optee->revision.os_major = major;
	optee->revision.os_minor = minor;
	optee->revision.os_build_id = build_id;

	if (build_id)
		pr_info("revision %u.%u (%08x)", major, minor, build_id);
	else
		pr_info("revision %u.%u", major, minor);

	return true;
}

static bool optee_rpmi_api_is_compatible(struct optee *optee)
{
	struct optee_rpmi_msg req = {
		.w[0] = cpu_to_le32(OPTEE_RPMI_GET_API_VERSION),
	};
	struct optee_rpmi_msg rsp;
	u32 major, minor;
	int rc;

	rc = optee_rpmi_msg_call(optee, &req, &rsp);
	if (rc) {
		pr_err("Unexpected error %d\n", rc);
		return false;
	}

	major = le32_to_cpu(rsp.w[0]);
	minor = le32_to_cpu(rsp.w[1]);
	if (major != OPTEE_RPMI_VERSION_MAJOR ||
	    minor < OPTEE_RPMI_VERSION_MINOR) {
		pr_err("Incompatible OP-TEE API version %u.%u", major, minor);
		return false;
	}

	return true;
}

static bool optee_rpmi_exchange_caps(struct optee *optee, u32 *sec_caps,
				     unsigned int *rpc_param_count)
{
	struct optee_rpmi_msg req = {
		.w[0] = cpu_to_le32(OPTEE_RPMI_EXCHANGE_CAPABILITIES),
	};
	struct optee_rpmi_msg rsp;
	int rc;

	rc = optee_rpmi_msg_call(optee, &req, &rsp);
	if (rc) {
		pr_err("Unexpected error %d", rc);
		return false;
	}
	if (rsp.w[0]) {
		pr_err("Unexpected exchange error %u", le32_to_cpu(rsp.w[0]));
		return false;
	}

	*sec_caps = le32_to_cpu(rsp.w[1]);
	if (*sec_caps & OPTEE_RPMI_SEC_CAP_ARG_OFFSET)
		*rpc_param_count = le32_to_cpu(rsp.w[2]);
	else
		*rpc_param_count = 0;

	return true;
}

static void optee_rpmi_get_version(struct tee_device *teedev,
				   struct tee_ioctl_version_data *vers)
{
	struct tee_ioctl_version_data v = {
		.impl_id = TEE_IMPL_ID_OPTEE,
		.impl_caps = TEE_OPTEE_CAP_TZ,
		.gen_caps = TEE_GEN_CAP_GP | TEE_GEN_CAP_REG_MEM |
			    TEE_GEN_CAP_MEMREF_NULL,
	};

	*vers = v;
}

static int optee_rpmi_open(struct tee_context *ctx)
{
	return optee_open(ctx, true);
}

static const struct tee_driver_ops optee_rpmi_clnt_ops = {
	.get_version = optee_rpmi_get_version,
	.get_tee_revision = optee_get_revision,
	.open = optee_rpmi_open,
	.release = optee_release,
	.open_session = optee_open_session,
	.close_session = optee_close_session,
	.invoke_func = optee_invoke_func,
	.cancel_req = optee_cancel_req,
	.shm_register = optee_rpmi_shm_register,
	.shm_unregister = optee_rpmi_shm_unregister,
};

static const struct tee_desc optee_rpmi_clnt_desc = {
	.name = DRIVER_NAME "-rpmi-clnt",
	.ops = &optee_rpmi_clnt_ops,
	.owner = THIS_MODULE,
};

static const struct tee_driver_ops optee_rpmi_supp_ops = {
	.get_version = optee_rpmi_get_version,
	.get_tee_revision = optee_get_revision,
	.open = optee_rpmi_open,
	.release = optee_release_supp,
	.supp_recv = optee_supp_recv,
	.supp_send = optee_supp_send,
	.shm_register = optee_rpmi_shm_register, /* same as for clnt ops */
	.shm_unregister = optee_rpmi_shm_unregister_supp,
};

static const struct tee_desc optee_rpmi_supp_desc = {
	.name = DRIVER_NAME "-rpmi-supp",
	.ops = &optee_rpmi_supp_ops,
	.owner = THIS_MODULE,
	.flags = TEE_DESC_PRIVILEGED,
};

static const struct optee_ops optee_rpmi_ops = {
	.do_call_with_arg = optee_rpmi_do_call_with_arg,
	.to_msg_param = optee_rpmi_to_msg_param,
	.from_msg_param = optee_rpmi_from_msg_param,
};

static void optee_rpmi_remove(struct rpmi_tee_device *tdev)
{
	struct optee *optee = rpmi_tee_dev_get_drvdata(tdev);

	optee_remove_common(optee);

	mutex_destroy(&optee->rpmi.mutex);
	rhashtable_free_and_destroy(&optee->rpmi.global_ids, rh_free_fn, NULL);

	kfree(optee);
}

static int optee_rpmi_probe(struct rpmi_tee_device *tdev)
{
	unsigned int rpc_param_count;
	struct tee_shm_pool *pool;
	struct tee_device *teedev;
	struct tee_context *ctx;
	u32 arg_cache_flags = 0;
	struct optee *optee;
	u32 sec_caps;
	int rc;

	if (rpmi_tee_max_call_data(tdev) < sizeof(struct optee_rpmi_msg))
		return -EINVAL;

	optee = kzalloc_obj(*optee);
	if (!optee)
		return -ENOMEM;
	optee->rpmi.tdev = tdev;

	if (!optee_rpmi_api_is_compatible(optee)) {
		rc = -EINVAL;
		goto err_free_optee;
	}

	if (!optee_rpmi_exchange_caps(optee, &sec_caps, &rpc_param_count)) {
		rc = -EINVAL;
		goto err_free_optee;
	}
	if (sec_caps & OPTEE_RPMI_SEC_CAP_ARG_OFFSET)
		arg_cache_flags |= OPTEE_SHM_ARG_SHARED;

	if (!optee_rpmi_get_os_revision(optee)) {
		rc = -EINVAL;
		goto err_free_optee;
	}

	pool = optee_rpmi_shm_pool_alloc_pages();
	if (IS_ERR(pool)) {
		rc = PTR_ERR(pool);
		goto err_free_optee;
	}
	optee->pool = pool;

	optee->ops = &optee_rpmi_ops;
	optee->rpc_param_count = rpc_param_count;

	if (IS_REACHABLE(CONFIG_RPMB) &&
	    (sec_caps & OPTEE_RPMI_SEC_CAP_RPMB_PROBE))
		optee->in_kernel_rpmb_routing = true;

	teedev = tee_device_alloc(&optee_rpmi_clnt_desc, NULL, optee->pool,
				  optee);
	if (IS_ERR(teedev)) {
		rc = PTR_ERR(teedev);
		goto err_free_shm_pool;
	}
	optee->teedev = teedev;

	teedev = tee_device_alloc(&optee_rpmi_supp_desc, NULL, optee->pool,
				  optee);
	if (IS_ERR(teedev)) {
		rc = PTR_ERR(teedev);
		goto err_unreg_teedev;
	}
	optee->supp_teedev = teedev;

	optee_set_dev_group(optee);

	rc = tee_device_register(optee->teedev);
	if (rc)
		goto err_unreg_supp_teedev;

	rc = tee_device_register(optee->supp_teedev);
	if (rc)
		goto err_unreg_supp_teedev;

	rc = rhashtable_init(&optee->rpmi.global_ids, &shm_rhash_params);
	if (rc)
		goto err_unreg_supp_teedev;
	mutex_init(&optee->rpmi.mutex);
	optee_cq_init(&optee->call_queue, 0);
	optee_supp_init(&optee->supp);
	optee_shm_arg_cache_init(optee, arg_cache_flags);
	mutex_init(&optee->rpmb_dev_mutex);
	rpmi_tee_dev_set_drvdata(tdev, optee);
	ctx = teedev_open(optee->teedev);
	if (IS_ERR(ctx)) {
		rc = PTR_ERR(ctx);
		goto err_rhashtable_free;
	}
	optee->ctx = ctx;
	rc = optee_notif_init(optee, OPTEE_DEFAULT_MAX_NOTIF_VALUE);
	if (rc)
		goto err_close_ctx;

	rc = optee_enumerate_devices(PTA_CMD_GET_DEVICES);
	if (rc)
		goto err_unregister_devices;

	INIT_WORK(&optee->rpmb_scan_bus_work, optee_bus_scan_rpmb);
	optee->rpmb_intf.notifier_call = optee_rpmb_intf_rdev;
	blocking_notifier_chain_register(&optee_rpmb_intf_added,
					 &optee->rpmb_intf);
	pr_info("initialized driver\n");
	return 0;

err_unregister_devices:
	optee_unregister_devices();
	optee_notif_uninit(optee);
err_close_ctx:
	teedev_close_context(ctx);
err_rhashtable_free:
	rhashtable_free_and_destroy(&optee->rpmi.global_ids, rh_free_fn, NULL);
	rpmb_dev_put(optee->rpmb_dev);
	mutex_destroy(&optee->rpmb_dev_mutex);
	optee_supp_uninit(&optee->supp);
	mutex_destroy(&optee->call_queue.mutex);
	mutex_destroy(&optee->rpmi.mutex);
err_unreg_supp_teedev:
	tee_device_unregister(optee->supp_teedev);
err_unreg_teedev:
	tee_device_unregister(optee->teedev);
err_free_shm_pool:
	tee_shm_pool_free(pool);
err_free_optee:
	kfree(optee);
	return rc;
}

static const struct of_device_id optee_rpmi_of_match[] = {
	{ .compatible = "linaro,optee-rpmi" },
	{}
};
MODULE_DEVICE_TABLE(of, optee_rpmi_of_match);

static struct rpmi_tee_driver optee_rpmi_driver = {
	.driver = {
		.name = "optee",
		.of_match_table = optee_rpmi_of_match,
	},
	.probe = optee_rpmi_probe,
	.remove = optee_rpmi_remove,
};

int optee_rpmi_abi_register(void)
{
	return rpmi_tee_driver_register(&optee_rpmi_driver);
}

void optee_rpmi_abi_unregister(void)
{
	rpmi_tee_driver_unregister(&optee_rpmi_driver);
}
