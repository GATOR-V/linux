/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2026 NXP
 *
 * RISC-V Platform Management Interface (RPMI) TEE service group framework.
 */
#ifndef _LINUX_RISCV_RPMI_TEE_H
#define _LINUX_RISCV_RPMI_TEE_H

#include <linux/bits.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/uuid.h>

struct scatterlist;
struct rpmi_tee;

/* Memory access rights, as encoded by the memory parcel services */
#define RPMI_TEE_MEM_ACCESS_EXECUTE	BIT(31)
#define RPMI_TEE_MEM_ACCESS_WRITE	BIT(30)
#define RPMI_TEE_MEM_ACCESS_READ	BIT(29)
#define RPMI_TEE_MEM_ACCESS_RW		(RPMI_TEE_MEM_ACCESS_READ | \
					 RPMI_TEE_MEM_ACCESS_WRITE)

/**
 * struct rpmi_tee_device - TEE endpoint reachable through an RPMI TEE framework
 * @dev:	device on the rpmi_tee bus
 * @tee:	framework this endpoint belongs to
 * @endpoint_id: identifier of the endpoint, assigned by the framework
 */
struct rpmi_tee_device {
	struct device dev;
	struct rpmi_tee *tee;
	u32 endpoint_id;
};

/**
 * struct rpmi_tee_driver - driver for a TEE endpoint
 * @driver:	device driver, its of_match_table selects the endpoints
 * @probe:	probe callback
 * @remove:	remove callback
 */
struct rpmi_tee_driver {
	struct device_driver driver;
	int (*probe)(struct rpmi_tee_device *tdev);
	void (*remove)(struct rpmi_tee_device *tdev);
};

#define to_rpmi_tee_device(d)	container_of(d, struct rpmi_tee_device, dev)
#define to_rpmi_tee_driver(d)	container_of(d, struct rpmi_tee_driver, driver)

static inline void rpmi_tee_dev_set_drvdata(struct rpmi_tee_device *tdev,
					    void *data)
{
	dev_set_drvdata(&tdev->dev, data);
}

static inline void *rpmi_tee_dev_get_drvdata(struct rpmi_tee_device *tdev)
{
	return dev_get_drvdata(&tdev->dev);
}

#if IS_REACHABLE(CONFIG_RISCV_RPMI_TEE)
int __rpmi_tee_driver_register(struct rpmi_tee_driver *drv,
			       struct module *owner);
void rpmi_tee_driver_unregister(struct rpmi_tee_driver *drv);
size_t rpmi_tee_max_call_data(struct rpmi_tee_device *tdev);
int rpmi_tee_call(struct rpmi_tee_device *tdev, const uuid_t *service,
		  const void *data, size_t data_len,
		  void *rsp, size_t rsp_size, size_t *rsp_len);
int rpmi_tee_mem_share(struct rpmi_tee_device *tdev, struct scatterlist *sg,
		       u32 access, u64 *handle);
int rpmi_tee_mem_reclaim(struct rpmi_tee_device *tdev, u64 handle);
#else
static inline int __rpmi_tee_driver_register(struct rpmi_tee_driver *drv,
					     struct module *owner)
{
	return -ENODEV;
}

static inline void rpmi_tee_driver_unregister(struct rpmi_tee_driver *drv)
{
}

static inline size_t rpmi_tee_max_call_data(struct rpmi_tee_device *tdev)
{
	return 0;
}

static inline int rpmi_tee_call(struct rpmi_tee_device *tdev,
				const uuid_t *service, const void *data,
				size_t data_len, void *rsp, size_t rsp_size,
				size_t *rsp_len)
{
	return -ENODEV;
}

static inline int rpmi_tee_mem_share(struct rpmi_tee_device *tdev,
				     struct scatterlist *sg, u32 access,
				     u64 *handle)
{
	return -ENODEV;
}

static inline int rpmi_tee_mem_reclaim(struct rpmi_tee_device *tdev,
				       u64 handle)
{
	return -ENODEV;
}
#endif

#define rpmi_tee_driver_register(drv) \
	__rpmi_tee_driver_register(drv, THIS_MODULE)

#endif /* _LINUX_RISCV_RPMI_TEE_H */
