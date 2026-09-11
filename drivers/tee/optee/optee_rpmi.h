/* SPDX-License-Identifier: (GPL-2.0 OR BSD-2-Clause) */
/*
 * Copyright 2026 NXP
 */

/*
 * This file is exported by OP-TEE and is kept in sync between OP-TEE OS
 * and the REE drivers. It defines the OP-TEE service invoked with the
 * TEE_CALL service of the RISC-V RPMI TEE service group.
 *
 * TEE_CALL identifies the service by OPTEE_RPMI_SERVICE_UUID and carries
 * a struct optee_rpmi_msg as service data and another one as service
 * response. Word 0 of the request is the function ID, words 1-7 its
 * arguments. Word 0 of the response is the status of the function, words
 * 1-7 its results. Unless stated otherwise, unused words must be zero.
 *
 * Memory is shared with OP-TEE using memory parcels created by the REE
 * with OP-TEE as receiver. A parcel is referred to with a 64-bit handle
 * whose lower 32 bits are the MEM_PARCEL_ID and upper 32 bits the NONCE
 * of the parcel. OP-TEE accepts the parcel with TEE_MEMORY_PARCEL_ACCEPT
 * and releases it with TEE_MEMORY_PARCEL_RELEASE before the REE reclaims
 * it. Memory references in the OP-TEE message protocol (optee_msg.h) use
 * the OPTEE_MSG_ATTR_TYPE_FMEM_* parameters with the handle as global ID.
 */

#ifndef __OPTEE_RPMI_H
#define __OPTEE_RPMI_H

#include <linux/bits.h>
#include <linux/types.h>
#include <linux/uuid.h>

/*
 * UUID of the OP-TEE service, the same value as the OP-TEE API UID
 * (OPTEE_MSG_UID_0..3): 384fb3e0-e7f8-11e3-af63-0002a5d5c51b
 */
#define OPTEE_RPMI_SERVICE_UUID \
	UUID_INIT(0x384fb3e0, 0xe7f8, 0x11e3, \
		  0xaf, 0x63, 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1b)

/*
 * Version of the OP-TEE RPMI ABI, returned by OPTEE_RPMI_GET_API_VERSION.
 * A change in the major version breaks compatibility, a change in the
 * minor version adds functionality.
 */
#define OPTEE_RPMI_VERSION_MAJOR	1
#define OPTEE_RPMI_VERSION_MINOR	0

#define OPTEE_RPMI_MSG_NUM_WORDS	8

/**
 * struct optee_rpmi_msg - service data and service response of TEE_CALL
 * @w: 32-bit little-endian words
 */
struct optee_rpmi_msg {
	__le32 w[OPTEE_RPMI_MSG_NUM_WORDS];
};

#define OPTEE_RPMI_BLOCKING_CALL(id)	(id)
#define OPTEE_RPMI_YIELDING_CALL_BIT	31
#define OPTEE_RPMI_YIELDING_CALL(id)	((id) | BIT(OPTEE_RPMI_YIELDING_CALL_BIT))

/*
 * Get the version of the OP-TEE RPMI ABI.
 * Request:
 * w0:	OPTEE_RPMI_GET_API_VERSION
 * Response:
 * w0:	OPTEE_RPMI_VERSION_MAJOR
 * w1:	OPTEE_RPMI_VERSION_MINOR
 */
#define OPTEE_RPMI_GET_API_VERSION	OPTEE_RPMI_BLOCKING_CALL(0)

/*
 * Get the version of OP-TEE OS.
 * Request:
 * w0:	OPTEE_RPMI_GET_OS_VERSION
 * Response:
 * w0:	Major version
 * w1:	Minor version
 * w2:	Revision, a build identifier
 */
#define OPTEE_RPMI_GET_OS_VERSION	OPTEE_RPMI_BLOCKING_CALL(1)

/*
 * Exchange capabilities between the REE and OP-TEE.
 * Request:
 * w0:	OPTEE_RPMI_EXCHANGE_CAPABILITIES
 * w1:	Capabilities of the REE, must be zero
 * Response:
 * w0:	Status, zero on success
 * w1:	Capabilities of OP-TEE, OPTEE_RPMI_SEC_CAP_*
 * w2:	Number of parameters reserved for the RPC argument after the
 *	message argument when OPTEE_RPMI_SEC_CAP_ARG_OFFSET is set
 * w3:	Maximum notification value, valid when
 *	OPTEE_RPMI_SEC_CAP_ASYNC_NOTIF is set
 */
/* OP-TEE supports the RPC argument following the message argument */
#define OPTEE_RPMI_SEC_CAP_ARG_OFFSET	BIT(0)
/* OP-TEE supports asynchronous notifications, reserved */
#define OPTEE_RPMI_SEC_CAP_ASYNC_NOTIF	BIT(1)
/* OP-TEE supports probing RPMB devices through the kernel */
#define OPTEE_RPMI_SEC_CAP_RPMB_PROBE	BIT(2)

#define OPTEE_RPMI_EXCHANGE_CAPABILITIES OPTEE_RPMI_BLOCKING_CALL(2)

/*
 * Unregister shared memory before the REE reclaims the memory parcel.
 * Request:
 * w0:	OPTEE_RPMI_UNREGISTER_SHM
 * w1:	Lower 32 bits of the memory parcel handle
 * w2:	Upper 32 bits of the memory parcel handle
 * Response:
 * w0:	Status, zero on success
 */
#define OPTEE_RPMI_UNREGISTER_SHM	OPTEE_RPMI_BLOCKING_CALL(3)

/*
 * Call with struct optee_msg_arg as argument in the supplied shared
 * memory, with a zero internal offset and normal cached memory attributes.
 * Request:
 * w0:	OPTEE_RPMI_YIELDING_CALL_WITH_ARG
 * w1:	Lower 32 bits of the memory parcel handle
 * w2:	Upper 32 bits of the memory parcel handle
 * w3:	Offset into the shared memory of the struct optee_msg_arg. Right
 *	after the parameters of this struct, at offset
 *	OPTEE_MSG_GET_ARG_SIZE(num_params), follows a struct optee_msg_arg
 *	for RPC with reserved space for the number of parameters returned
 *	by OPTEE_RPMI_EXCHANGE_CAPABILITIES. Must be zero unless
 *	OPTEE_RPMI_SEC_CAP_ARG_OFFSET is set.
 *
 * Resume from RPC.
 * Request:
 * w0:	OPTEE_RPMI_YIELDING_CALL_RESUME
 * w4:	Resume information
 *
 * Response of both, normal return (the call is completed):
 * w0:	Status, zero on success
 * w1:	OPTEE_RPMI_YIELDING_CALL_RETURN_DONE
 *
 * Response of both, RPC return (OP-TEE requests something from the REE):
 * w0:	Status, zero
 * w1:	OPTEE_RPMI_YIELDING_CALL_RETURN_RPC_CMD or
 *	OPTEE_RPMI_YIELDING_CALL_RETURN_INTERRUPT
 * w4:	Resume information
 *
 * The status is a TEEC_* value: TEEC_ERROR_BUSY when all OP-TEE threads
 * are in use, try again later; TEEC_ERROR_BAD_PARAMETERS on a bad memory
 * parcel handle, offset or resume information.
 */
#define OPTEE_RPMI_YIELDING_CALL_WITH_ARG	OPTEE_RPMI_YIELDING_CALL(0)
#define OPTEE_RPMI_YIELDING_CALL_RESUME		OPTEE_RPMI_YIELDING_CALL(1)

#define OPTEE_RPMI_YIELDING_CALL_RETURN_DONE		0
#define OPTEE_RPMI_YIELDING_CALL_RETURN_RPC_CMD		1
#define OPTEE_RPMI_YIELDING_CALL_RETURN_INTERRUPT	2

#endif /*__OPTEE_RPMI_H*/
