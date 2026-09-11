// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2026 NXP
 *
 * RISC-V Platform Management Interface (RPMI) TEE service group framework
 * driver.
 *
 * The TEE service group lets endpoints in a Rich Execution Environment
 * (REE) invoke services in Trusted Execution Environments (TEE) through a
 * framework implemented by the M-mode firmware or the hypervisor. This
 * driver is the REE side of it: it owns the SBI MPXY channel implementing
 * the service group, sends TEE_CALL requests, manages memory parcels and
 * enumerates TEE endpoints on the rpmi_tee bus for the TEE drivers.
 */

#define pr_fmt(fmt) "rpmi-tee: " fmt

#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/mailbox_client.h>
#include <linux/mailbox/riscv-rpmi-message.h>
#include <linux/mailbox/riscv-sbi-mpxy-mbox.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/random.h>
#include <linux/riscv_rpmi_tee.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/string.h>

/* TEE_PROBE_FEATURES */
#define RPMI_TEE_FEATURE_MEMORY_DONATE		1
#define RPMI_TEE_FEATURE_MEMORY_LEND		2
#define RPMI_TEE_FEATURE_MEMORY_SHARE		3
#define RPMI_TEE_FEATURE_SIGNAL_BUS		4
#define RPMI_TEE_FEATURE_MULTISEGMENT_OPS	5
#define RPMI_TEE_FEATURE_SYSINFO_FORMAT		6

struct rpmi_tee_probe_features_req {
	__le32 feature_id;
};

struct rpmi_tee_probe_features_rsp {
	__le32 status;
	__le32 value;
};

/* TEE_CALL */
struct rpmi_tee_call_req {
	__le32 sender_id;
	__le32 target_id;
	u8 service[UUID_SIZE];
	__le32 data_len;
	u8 data[];
} __packed;

struct rpmi_tee_call_rsp {
	__le32 status;
	__le32 rsp_len;
	u8 rsp[];
};

/* TEE_MEMORY_PARCEL_CREATE, fixed part followed by the variable arrays */
struct rpmi_tee_parcel_create_req {
	__le32 creator_id;
	__le32 creator_access;
	__le32 receiver_cnt;
	__le32 flags;
	__le32 nonce;
	__le32 block_cnt;
	u8 label[16];
	/* __le32 receiver_id[N], access[N], block_high[M], block_low[M] */
	__le32 arrays[];
};

#define RPMI_TEE_PARCEL_FLAG_MULTI_SEGMENT	BIT(31)
#define RPMI_TEE_PARCEL_FLAG_OWNER_XFER		BIT(30)

struct rpmi_tee_parcel_create_rsp {
	__le32 status;
	__le32 mem_parcel_id;
};

/* TEE_MEMORY_PARCEL_RECLAIM */
struct rpmi_tee_parcel_reclaim_req {
	__le32 mem_parcel_id;
};

struct rpmi_tee_parcel_reclaim_rsp {
	__le32 status;
	__le32 flags;
};

/*
 * Memory blocks: BLOCK_HIGH is the upper 32 bits of the 4kB page number,
 * BLOCK_LOW holds its lower 20 bits and the page count minus one.
 */
#define RPMI_TEE_BLOCK_PAGE_SHIFT	12
#define RPMI_TEE_BLOCK_PAGE_SIZE	BIT(RPMI_TEE_BLOCK_PAGE_SHIFT)
#define RPMI_TEE_BLOCK_LOW_PFN		GENMASK(31, 12)
#define RPMI_TEE_BLOCK_LOW_COUNT	GENMASK(11, 0)
#define RPMI_TEE_BLOCK_MAX_PAGES	(RPMI_TEE_BLOCK_LOW_COUNT + 1)

/**
 * struct rpmi_tee - RPMI TEE framework instance
 * @dev:		platform device
 * @cl:			mailbox client owning @chan
 * @chan:		SBI MPXY channel implementing the TEE service group
 * @endpoint_id:	identifier of this REE, assigned by the framework
 * @max_msg_data:	maximum message data size of the channel, in bytes
 * @mem_share:		memory sharing supported by the framework
 * @multiseg:		multi-segment memory operations supported
 * @lock:		serializes @buf
 * @buf:		message buffer of @max_msg_data bytes
 */
struct rpmi_tee {
	struct device *dev;
	struct mbox_client cl;
	struct mbox_chan *chan;
	u32 endpoint_id;
	size_t max_msg_data;
	bool mem_share;
	bool multiseg;
	/* Serializes @buf */
	struct mutex lock;
	void *buf;
};

static int rpmi_tee_status(const struct rpmi_mbox_message *msg,
			   const __le32 *status, size_t min_len)
{
	if (msg->data.out_response_len < sizeof(*status))
		return -EIO;

	if (le32_to_cpu(*status) != RPMI_SUCCESS)
		return rpmi_to_linux_error((s32)le32_to_cpu(*status));

	if (msg->data.out_response_len < min_len)
		return -EIO;

	return 0;
}

static int rpmi_tee_probe_feature(struct rpmi_tee *tee, u32 feature_id,
				  u32 *value)
{
	struct rpmi_tee_probe_features_req req = {
		.feature_id = cpu_to_le32(feature_id),
	};
	struct rpmi_tee_probe_features_rsp rsp = {};
	struct rpmi_mbox_message msg;
	int rc;

	rpmi_mbox_init_send_with_response(&msg, RPMI_TEE_SRV_PROBE_FEATURES,
					  &req, sizeof(req), &rsp, sizeof(rsp));
	rc = rpmi_mbox_send_message(tee->chan, &msg);
	if (rc)
		return rc;

	rc = rpmi_tee_status(&msg, &rsp.status, sizeof(rsp));
	if (rc)
		return rc;

	*value = le32_to_cpu(rsp.value);
	return 0;
}

/**
 * rpmi_tee_max_call_data() - Maximum service data size of a TEE_CALL
 * @tdev: TEE endpoint
 *
 * Return: the maximum number of service data bytes rpmi_tee_call() accepts
 */
size_t rpmi_tee_max_call_data(struct rpmi_tee_device *tdev)
{
	return tdev->tee->max_msg_data - sizeof(struct rpmi_tee_call_req);
}
EXPORT_SYMBOL_GPL(rpmi_tee_max_call_data);

/**
 * rpmi_tee_call() - Invoke a service of a TEE endpoint with TEE_CALL
 * @tdev:	TEE endpoint
 * @service:	UUID of the service
 * @data:	service data
 * @data_len:	length of @data in bytes
 * @rsp:	buffer for the service response
 * @rsp_size:	size of @rsp in bytes
 * @rsp_len:	length of the service response returned in @rsp
 *
 * The target TEE executes on the calling hart until it responds, so this
 * can take an unbounded time and must be called from a sleepable context.
 * Callers may invoke this concurrently from different harts. Errors
 * returned by the framework, which mean the service was not reached,
 * are returned as negative error codes; whatever the service reports is
 * in the service response.
 *
 * Return: 0 on success or a negative error code
 */
int rpmi_tee_call(struct rpmi_tee_device *tdev, const uuid_t *service,
		  const void *data, size_t data_len,
		  void *rsp, size_t rsp_size, size_t *rsp_len)
{
	struct rpmi_tee *tee = tdev->tee;
	struct rpmi_tee_call_req *req;
	struct rpmi_tee_call_rsp *crsp;
	struct rpmi_mbox_message msg;
	size_t req_len, crsp_size, len;
	int rc;

	if (data_len > rpmi_tee_max_call_data(tdev))
		return -E2BIG;
	if (rsp_size > tee->max_msg_data - sizeof(*crsp))
		return -E2BIG;

	req_len = sizeof(*req) + data_len;
	crsp_size = sizeof(*crsp) + rsp_size;
	req = kzalloc(req_len + crsp_size, GFP_KERNEL);
	if (!req)
		return -ENOMEM;
	crsp = (void *)req + req_len;

	req->sender_id = cpu_to_le32(tee->endpoint_id);
	req->target_id = cpu_to_le32(tdev->endpoint_id);
	export_uuid(req->service, service);
	req->data_len = cpu_to_le32(data_len);
	memcpy(req->data, data, data_len);

	rpmi_mbox_init_send_with_response(&msg, RPMI_TEE_SRV_CALL,
					  req, req_len, crsp, crsp_size);
	rc = riscv_sbi_mpxy_mbox_call(tee->chan, &msg);
	if (rc)
		goto out;

	rc = rpmi_tee_status(&msg, &crsp->status, sizeof(*crsp));
	if (rc)
		goto out;

	len = le32_to_cpu(crsp->rsp_len);
	if (len > rsp_size || sizeof(*crsp) + len > msg.data.out_response_len) {
		rc = -EIO;
		goto out;
	}

	memcpy(rsp, crsp->rsp, len);
	*rsp_len = len;
out:
	kfree(req);
	return rc;
}
EXPORT_SYMBOL_GPL(rpmi_tee_call);

static unsigned int rpmi_tee_count_blocks(struct scatterlist *sg)
{
	unsigned int count = 0;

	for (; sg; sg = sg_next(sg)) {
		if (!IS_ALIGNED(sg_phys(sg), RPMI_TEE_BLOCK_PAGE_SIZE) ||
		    !IS_ALIGNED(sg->length, RPMI_TEE_BLOCK_PAGE_SIZE))
			return 0;
		count += DIV_ROUND_UP(sg->length >> RPMI_TEE_BLOCK_PAGE_SHIFT,
				      RPMI_TEE_BLOCK_MAX_PAGES);
	}

	return count;
}

static void rpmi_tee_encode_blocks(struct scatterlist *sg, __le32 *high,
				   __le32 *low)
{
	for (; sg; sg = sg_next(sg)) {
		u64 pfn = sg_phys(sg) >> RPMI_TEE_BLOCK_PAGE_SHIFT;
		unsigned long pages = sg->length >> RPMI_TEE_BLOCK_PAGE_SHIFT;

		while (pages) {
			unsigned long n = min(pages, RPMI_TEE_BLOCK_MAX_PAGES);

			*high++ = cpu_to_le32(pfn >> 20);
			*low++ = cpu_to_le32(FIELD_PREP(RPMI_TEE_BLOCK_LOW_PFN,
							pfn & GENMASK(19, 0)) |
					     FIELD_PREP(RPMI_TEE_BLOCK_LOW_COUNT,
							n - 1));
			pfn += n;
			pages -= n;
		}
	}
}

/**
 * rpmi_tee_mem_share() - Share memory with a TEE endpoint
 * @tdev:	TEE endpoint receiving access to the memory
 * @sg:		scatterlist of the memory, page aligned
 * @access:	RPMI_TEE_MEM_ACCESS_* rights granted to @tdev
 * @handle:	handle of the memory parcel, to be passed to the TEE which
 *		accepts the parcel with it and to rpmi_tee_mem_reclaim()
 *
 * Creates a memory parcel sharing the memory with @tdev while this REE
 * keeps its own access. The handle combines the parcel identifier and the
 * nonce the TEE has to present to accept the parcel.
 *
 * Return: 0 on success, -E2BIG when the memory description does not fit
 * in a single message, or another negative error code
 */
int rpmi_tee_mem_share(struct rpmi_tee_device *tdev, struct scatterlist *sg,
		       u32 access, u64 *handle)
{
	struct rpmi_tee *tee = tdev->tee;
	struct rpmi_tee_parcel_create_rsp rsp = {};
	struct rpmi_tee_parcel_create_req *req;
	struct rpmi_mbox_message msg;
	unsigned int blocks;
	size_t req_len;
	u32 nonce;
	int rc;

	if (!tee->mem_share)
		return -EOPNOTSUPP;

	blocks = rpmi_tee_count_blocks(sg);
	if (!blocks)
		return -EINVAL;

	/* receiver_id[1], access[1], block_high[M], block_low[M] */
	req_len = sizeof(*req) + (2 + 2 * blocks) * sizeof(__le32);
	if (req_len > tee->max_msg_data)
		return -E2BIG;

	nonce = get_random_u32();

	guard(mutex)(&tee->lock);

	req = tee->buf;
	memset(req, 0, req_len);
	req->creator_id = cpu_to_le32(tee->endpoint_id);
	req->creator_access = cpu_to_le32(RPMI_TEE_MEM_ACCESS_RW);
	req->receiver_cnt = cpu_to_le32(1);
	req->nonce = cpu_to_le32(nonce);
	req->block_cnt = cpu_to_le32(blocks);
	req->arrays[0] = cpu_to_le32(tdev->endpoint_id);
	req->arrays[1] = cpu_to_le32(access);
	rpmi_tee_encode_blocks(sg, &req->arrays[2], &req->arrays[2 + blocks]);

	rpmi_mbox_init_send_with_response(&msg,
					  RPMI_TEE_SRV_MEMORY_PARCEL_CREATE,
					  req, req_len, &rsp, sizeof(rsp));
	rc = rpmi_mbox_send_message(tee->chan, &msg);
	if (rc)
		return rc;

	rc = rpmi_tee_status(&msg, &rsp.status, sizeof(rsp));
	if (rc)
		return rc;

	*handle = (u64)nonce << 32 | le32_to_cpu(rsp.mem_parcel_id);
	return 0;
}
EXPORT_SYMBOL_GPL(rpmi_tee_mem_share);

/**
 * rpmi_tee_mem_reclaim() - Reclaim memory shared with rpmi_tee_mem_share()
 * @tdev:	TEE endpoint the memory was shared with
 * @handle:	handle returned by rpmi_tee_mem_share()
 *
 * The TEE must have released the parcel first.
 *
 * Return: 0 on success or a negative error code
 */
int rpmi_tee_mem_reclaim(struct rpmi_tee_device *tdev, u64 handle)
{
	struct rpmi_tee_parcel_reclaim_req req = {
		.mem_parcel_id = cpu_to_le32(lower_32_bits(handle)),
	};
	struct rpmi_tee_parcel_reclaim_rsp rsp = {};
	struct rpmi_mbox_message msg;
	int rc;

	rpmi_mbox_init_send_with_response(&msg,
					  RPMI_TEE_SRV_MEMORY_PARCEL_RECLAIM,
					  &req, sizeof(req), &rsp, sizeof(rsp));
	rc = rpmi_mbox_send_message(tdev->tee->chan, &msg);
	if (rc)
		return rc;

	return rpmi_tee_status(&msg, &rsp.status, sizeof(rsp));
}
EXPORT_SYMBOL_GPL(rpmi_tee_mem_reclaim);

/* rpmi_tee bus */

static int rpmi_tee_bus_match(struct device *dev,
			      const struct device_driver *drv)
{
	return of_driver_match_device(dev, drv);
}

static int rpmi_tee_bus_probe(struct device *dev)
{
	struct rpmi_tee_driver *drv = to_rpmi_tee_driver(dev->driver);

	return drv->probe(to_rpmi_tee_device(dev));
}

static void rpmi_tee_bus_remove(struct device *dev)
{
	struct rpmi_tee_driver *drv = to_rpmi_tee_driver(dev->driver);

	if (drv->remove)
		drv->remove(to_rpmi_tee_device(dev));
}

static int rpmi_tee_bus_uevent(const struct device *dev,
			       struct kobj_uevent_env *env)
{
	return of_device_uevent_modalias(dev, env);
}

static const struct bus_type rpmi_tee_bus_type = {
	.name = "rpmi_tee",
	.match = rpmi_tee_bus_match,
	.probe = rpmi_tee_bus_probe,
	.remove = rpmi_tee_bus_remove,
	.uevent = rpmi_tee_bus_uevent,
};

/**
 * __rpmi_tee_driver_register() - Register a TEE endpoint driver
 * @drv:	driver to register
 * @owner:	module owning @drv
 *
 * Return: 0 on success or a negative error code
 */
int __rpmi_tee_driver_register(struct rpmi_tee_driver *drv,
			       struct module *owner)
{
	if (!drv->probe)
		return -EINVAL;

	drv->driver.bus = &rpmi_tee_bus_type;
	drv->driver.owner = owner;

	return driver_register(&drv->driver);
}
EXPORT_SYMBOL_GPL(__rpmi_tee_driver_register);

/**
 * rpmi_tee_driver_unregister() - Unregister a TEE endpoint driver
 * @drv:	driver to unregister
 */
void rpmi_tee_driver_unregister(struct rpmi_tee_driver *drv)
{
	driver_unregister(&drv->driver);
}
EXPORT_SYMBOL_GPL(rpmi_tee_driver_unregister);

static void rpmi_tee_device_release(struct device *dev)
{
	struct rpmi_tee_device *tdev = to_rpmi_tee_device(dev);

	of_node_put(dev->of_node);
	kfree(tdev);
}

static int rpmi_tee_add_endpoint(struct rpmi_tee *tee,
				 struct device_node *node)
{
	struct rpmi_tee_device *tdev;
	int rc;

	tdev = kzalloc_obj(*tdev);
	if (!tdev)
		return -ENOMEM;

	rc = of_property_read_u32(node, "reg", &tdev->endpoint_id);
	if (rc) {
		kfree(tdev);
		return rc;
	}

	tdev->tee = tee;
	device_initialize(&tdev->dev);
	tdev->dev.bus = &rpmi_tee_bus_type;
	tdev->dev.parent = tee->dev;
	tdev->dev.of_node = of_node_get(node);
	tdev->dev.release = rpmi_tee_device_release;
	dev_set_name(&tdev->dev, "%s:%u", dev_name(tee->dev), tdev->endpoint_id);

	rc = device_add(&tdev->dev);
	if (rc) {
		put_device(&tdev->dev);
		return rc;
	}

	return 0;
}

static int rpmi_tee_remove_endpoint(struct device *dev, void *data)
{
	if (dev->bus == &rpmi_tee_bus_type)
		device_unregister(dev);

	return 0;
}

static int rpmi_tee_get_attr(struct rpmi_tee *tee,
			     enum rpmi_mbox_attribute_id id, u32 *value)
{
	struct rpmi_mbox_message msg;
	int rc;

	rpmi_mbox_init_get_attribute(&msg, id);
	rc = rpmi_mbox_send_message(tee->chan, &msg);
	if (rc)
		return rc;

	*value = msg.attr.value;
	return 0;
}

static int rpmi_tee_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rpmi_tee *tee;
	u32 value;
	int rc;

	tee = devm_kzalloc(dev, sizeof(*tee), GFP_KERNEL);
	if (!tee)
		return -ENOMEM;
	tee->dev = dev;

	rc = devm_mutex_init(dev, &tee->lock);
	if (rc)
		return rc;

	rc = device_property_read_u32(dev, "riscv,rpmi-tee-endpoint-id",
				      &tee->endpoint_id);
	if (rc)
		return dev_err_probe(dev, rc,
				     "missing \"riscv,rpmi-tee-endpoint-id\" property\n");

	tee->cl.dev = dev;
	tee->cl.tx_block = false;
	tee->cl.knows_txdone = true;
	tee->chan = mbox_request_channel(&tee->cl, 0);
	if (IS_ERR(tee->chan))
		return dev_err_probe(dev, PTR_ERR(tee->chan),
				     "failed to request MPXY channel\n");

	rc = rpmi_tee_get_attr(tee, RPMI_MBOX_ATTR_SERVICEGROUP_ID, &value);
	if (rc) {
		dev_err_probe(dev, rc, "failed to read RPMI service group ID\n");
		goto err_free_channel;
	}
	if (value != RPMI_SRVGRP_TEE) {
		dev_err(dev, "MPXY channel implements RPMI service group 0x%x, not TEE\n",
			value);
		rc = -ENODEV;
		goto err_free_channel;
	}

	rc = rpmi_tee_get_attr(tee, RPMI_MBOX_ATTR_MAX_MSG_DATA_SIZE, &value);
	if (rc) {
		dev_err_probe(dev, rc, "failed to read RPMI max message size\n");
		goto err_free_channel;
	}
	if (value < sizeof(struct rpmi_tee_parcel_create_req) +
		    4 * sizeof(__le32)) {
		dev_err(dev, "MPXY channel message size %u is too small\n",
			value);
		rc = -EINVAL;
		goto err_free_channel;
	}
	tee->max_msg_data = value;

	tee->buf = devm_kzalloc(dev, tee->max_msg_data, GFP_KERNEL);
	if (!tee->buf) {
		rc = -ENOMEM;
		goto err_free_channel;
	}

	rc = rpmi_tee_probe_feature(tee, RPMI_TEE_FEATURE_MEMORY_SHARE, &value);
	if (rc) {
		dev_err_probe(dev, rc, "failed to probe framework features\n");
		goto err_free_channel;
	}
	tee->mem_share = value;

	rc = rpmi_tee_probe_feature(tee, RPMI_TEE_FEATURE_MULTISEGMENT_OPS,
				    &value);
	if (rc) {
		dev_err_probe(dev, rc, "failed to probe framework features\n");
		goto err_free_channel;
	}
	tee->multiseg = value;

	platform_set_drvdata(pdev, tee);

	for_each_available_child_of_node_scoped(dev->of_node, node) {
		rc = rpmi_tee_add_endpoint(tee, node);
		if (rc) {
			dev_err(dev, "failed to add endpoint %pOF: %d\n",
				node, rc);
			goto err_remove_endpoints;
		}
	}

	dev_info(dev, "REE endpoint %u, memory sharing %ssupported\n",
		 tee->endpoint_id, tee->mem_share ? "" : "not ");

	return 0;

err_remove_endpoints:
	device_for_each_child(dev, NULL, rpmi_tee_remove_endpoint);
err_free_channel:
	mbox_free_channel(tee->chan);
	return rc;
}

static void rpmi_tee_remove(struct platform_device *pdev)
{
	struct rpmi_tee *tee = platform_get_drvdata(pdev);

	device_for_each_child(&pdev->dev, NULL, rpmi_tee_remove_endpoint);
	mbox_free_channel(tee->chan);
}

static const struct of_device_id rpmi_tee_of_match[] = {
	{ .compatible = "riscv,rpmi-mpxy-tee" },
	{}
};
MODULE_DEVICE_TABLE(of, rpmi_tee_of_match);

static struct platform_driver rpmi_tee_driver = {
	.driver = {
		.name = "riscv-rpmi-tee",
		.of_match_table = rpmi_tee_of_match,
	},
	.probe = rpmi_tee_probe,
	.remove = rpmi_tee_remove,
};

static int __init rpmi_tee_init(void)
{
	int rc;

	rc = bus_register(&rpmi_tee_bus_type);
	if (rc)
		return rc;

	rc = platform_driver_register(&rpmi_tee_driver);
	if (rc)
		bus_unregister(&rpmi_tee_bus_type);

	return rc;
}
subsys_initcall(rpmi_tee_init);

static void __exit rpmi_tee_exit(void)
{
	platform_driver_unregister(&rpmi_tee_driver);
	bus_unregister(&rpmi_tee_bus_type);
}
module_exit(rpmi_tee_exit);

MODULE_DESCRIPTION("RISC-V RPMI TEE service group framework driver");
MODULE_LICENSE("GPL");
