// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/firmware.h>
#include "connfem.h"

#if (CONNFEM_TEST_ENABLED == 1)
#include "connfem_test.h"
#endif

/*******************************************************************************
 *				M A C R O S
 ******************************************************************************/
#define CONNFEM_DRIVER_NAME	"connfem"
#define CONNFEM_DEV_NUM		1

/*******************************************************************************
 *			    D A T A   T Y P E S
 ******************************************************************************/
struct connfem_cdev_context {
	dev_t devId;
	struct class *class;
	struct device *chrdev;
	struct cdev cdev;
	struct connfem_context *cfm;
};

/*******************************************************************************
 *		    F U N C T I O N   D E C L A R A T I O N S
 ******************************************************************************/
static int connfem_plat_probe(struct platform_device *pdev);
static int connfem_plat_remove(struct platform_device *pdev);

static long connfem_dev_unlocked_ioctl(struct file *filp, unsigned int cmd,
					unsigned long arg);
#ifdef CONFIG_COMPAT
static long connfem_dev_compat_ioctl(struct file *filp, unsigned int cmd,
					unsigned long arg);
#endif

/*******************************************************************************
 *			    P U B L I C   D A T A
 ******************************************************************************/
struct connfem_context *connfem_ctx;

/*******************************************************************************
 *			   P R I V A T E   D A T A
 ******************************************************************************/
/* Platform Device */
#ifdef CONFIG_OF

struct connfem_context connfem_ctx_mt6893 = {
	.id = 0x6893
};
struct connfem_context connfem_ctx_mt6983 = {
	.id = 0x6983
};
struct connfem_context connfem_ctx_mt6879 = {
	.id = 0x6879
};
struct connfem_context connfem_ctx_mt6895 = {
	.id = 0x6895
};
struct connfem_context connfem_ctx_mt6985 = {
	.id = 0x6985
};
struct connfem_context connfem_ctx_mt6886 = {
	.id = 0x6886
};
struct connfem_context connfem_ctx_mt6897 = {
	.id = 0x6897
};
struct connfem_context connfem_ctx_mt6989 = {
	.id = 0x6989
};
struct connfem_context connfem_ctx_mt6878 = {
	.id = 0x6878
};

static const struct of_device_id connfem_of_ids[] = {
	{
		.compatible = "mediatek,mt6893-connfem",
		.data = (void *)&connfem_ctx_mt6893
	},
	{
		.compatible = "mediatek,mt6983-connfem",
		.data = (void *)&connfem_ctx_mt6983
	},
	{
		.compatible = "mediatek,mt6879-connfem",
		.data = (void *)&connfem_ctx_mt6879
	},
	{
		.compatible = "mediatek,mt6895-connfem",
		.data = (void *)&connfem_ctx_mt6895
	},
	{
		.compatible = "mediatek,mt6985-connfem",
		.data = (void *)&connfem_ctx_mt6985
	},
	{
		.compatible = "mediatek,mt6886-connfem",
		.data = (void *)&connfem_ctx_mt6886
	},
	{
		.compatible = "mediatek,mt6897-connfem",
		.data = (void *)&connfem_ctx_mt6897
	},
	{
		.compatible = "mediatek,mt6989-connfem",
		.data = (void *)&connfem_ctx_mt6989
	},
	{
		.compatible = "mediatek,mt6878-connfem",
		.data = (void *)&connfem_ctx_mt6878
	},
	{}
};
#endif

static struct platform_driver connfem_plat_drv = {
	.probe = connfem_plat_probe,
	.remove = connfem_plat_remove,
	.driver = {
		   .name = "mtk_connfem",
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = connfem_of_ids,
#endif
		   },
};

/* Char Device */
static struct connfem_cdev_context connfem_cdev_ctx;

static const struct file_operations connfem_dev_fops = {
	.unlocked_ioctl = connfem_dev_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = connfem_dev_compat_ioctl,
#endif
};

/* Module Parameters */
static unsigned int connfem_major;
static unsigned int epa_elna_hwid = CFM_PARAM_EPAELNA_HWID_INVALID;
static char *config_file = "connfem.cfg";
static char *internal_file = "connfem_internal";

/*******************************************************************************
 *			      F U N C T I O N S
 ******************************************************************************/
bool __weak connfem_is_internal(void)
{
	const struct firmware *data = NULL;

	if (request_firmware_direct(&data, internal_file, NULL) == 0) {
		release_firmware(data);
		return true;
	}

	release_firmware(data);
	return false;
}

void cfm_context_free(struct connfem_context *cfm)
{
	if (!cfm)
		return;

	cfm_dt_free(&cfm->dt);
	cfm_epaelna_config_free(&cfm->epaelna, true);
}

unsigned int cfm_param_epaelna_hwid(void)
{
	return epa_elna_hwid;
}

static int cfm_ioc_is_available_hdlr(unsigned long usr_arg)
{
	struct cfm_ioc_is_available data;

	if (!usr_arg) {
		pr_info("%s, invalid parameter", __func__);
		return -EINVAL;
	}

	if (copy_from_user(&data, (void *)usr_arg, sizeof(data)) != 0) {
		pr_info("%s, copy_from_user failed", __func__);
		return -EINVAL;
	}

	data.is_available = connfem_is_available(data.fem_type);
	pr_info("%s, type:%d, return %d",
		__func__, data.fem_type, data.is_available);

	if (copy_to_user((void *)usr_arg, &data, sizeof(data)) != 0) {
		pr_info("%s, copy_to_user failed", __func__);
		return -EINVAL;
	}

	return 0;
}

static int cfm_ioc_epa_cont_empty(uint64_t usr_cont)
{
	struct cfm_container empty;

	empty.cnt = 0;
	empty.entry_sz = 0;

	if (copy_to_user((void *)usr_cont, &empty, sizeof(empty)) != 0) {
		pr_info("%s, copy_to_user failed", __func__);
		return -EINVAL;
	}
	return 0;
}

static int cfm_ioc_epa_cont_trans(struct cfm_container *cont,
				  unsigned int usr_cnt,
				  unsigned int usr_entry_sz,
				  uint64_t usr_cont)
{
	unsigned int sz;

	if (!cont) {
		pr_info("%s, no cont, set container size to 0", __func__);
		return cfm_ioc_epa_cont_empty(usr_cont);
	}

	if (usr_cnt < cont->cnt || usr_entry_sz != cont->entry_sz) {
		pr_info("%s, not enough space, user(%d*%d) < need(%d*%d)",
			__func__,
			usr_cnt, usr_entry_sz,
			cont->cnt, cont->entry_sz);
		return -ENOMEM;
	}

	sz = sizeof(struct cfm_container) + (cont->cnt * cont->entry_sz);
	if (copy_to_user((void *)usr_cont, cont, sz) != 0) {
		pr_info("%s, copy_to_user failed", __func__);
		return -EINVAL;
	}

	return 0;
}


static int cfm_ioc_epa_fn_stat_hdlr(unsigned long usr_arg)
{
	int err = 0;
	struct cfm_ioc_epa_fn_stat data;
	struct cfm_container *pairs = NULL;

	if (!usr_arg) {
		pr_info("%s, invalid parameter", __func__);
		return -EINVAL;
	}

	if (copy_from_user(&data, (void *)usr_arg, sizeof(data)) != 0) {
		pr_info("%s, copy_from_user failed", __func__);
		return -EINVAL;
	}

	data.cnt = 0;
	data.entry_sz = 0;

	if (data.subsys >= CONNFEM_SUBSYS_NUM) {
		pr_info("%s, unsupported subsys %d", __func__, data.subsys);
		err = -EINVAL;
		goto fn_stat_done;
	}

	if (!connfem_ctx) {
		pr_info("%s, %d '%s', No ConnFem context",
			__func__, data.subsys, cfm_subsys_name[data.subsys]);
		err = -EOPNOTSUPP;
		goto fn_stat_done;
	}

	pairs = connfem_ctx->epaelna.flags_cfg[data.subsys].pairs;
	if (!pairs) {
		pr_info("%s, %d '%s', no flags",
			__func__,
			data.subsys,
			cfm_subsys_name[data.subsys]);
	} else {
		data.cnt = pairs->cnt;
		data.entry_sz = CONNFEM_FLAG_NAME_SIZE;
	}

	pr_info("%s, %d '%s', return cnt:%d, entry_sz:%d",
		__func__, data.subsys, cfm_subsys_name[data.subsys],
		data.cnt, data.entry_sz);

fn_stat_done:
	if (copy_to_user((void *)usr_arg, &data, sizeof(data)) != 0) {
		pr_info("%s, copy_to_user failed", __func__);
		return -EINVAL;
	}

	return err;
}

static int cfm_ioc_epa_fn_hdlr(unsigned long usr_arg)
{
	/* status & control variables */
	int err = 0;
	unsigned int i;

	/* user data variables */
	struct cfm_ioc_epa_fn data;
	struct cfm_container names_hdr;
	struct cfm_container *names;
	char *name;

	/* kernel data variables */
	struct cfm_container *subsys_pairs;
	struct connfem_epaelna_flag_pair *pair;

	if (!usr_arg) {
		pr_info("%s, invalid parameter", __func__);
		return -EINVAL;
	}

	err = copy_from_user(&data, (void *)usr_arg, sizeof(data));
	if (err != 0) {
		pr_info("%s, copy_from_user failed", __func__);
		return -EINVAL;
	}

	if (!data.names) {
		pr_info("%s, invalid parameter, names is NULL", __func__);
		return -EINVAL;
	}

	if (copy_from_user(&names_hdr, (void *)data.names,
			   sizeof(struct cfm_container)) != 0) {
		pr_info("%s, copy_from_user failed", __func__);
		return -EINVAL;
	}

	if (data.subsys >= CONNFEM_SUBSYS_NUM) {
		pr_info("%s, unsupported subsys %d", __func__, data.subsys);
		cfm_ioc_epa_cont_empty(data.names);
		return -EINVAL;
	}

	if (!connfem_ctx) {
		pr_info("%s, %d '%s', No ConnFem context",
			__func__, data.subsys, cfm_subsys_name[data.subsys]);
		return -EOPNOTSUPP;
	}

	pr_info("%s, %d '%s'",
		__func__, data.subsys, cfm_subsys_name[data.subsys]);

	subsys_pairs = connfem_ctx->epaelna.flags_cfg[data.subsys].pairs;

	if (names_hdr.cnt < subsys_pairs->cnt ||
	    names_hdr.entry_sz != CONNFEM_FLAG_NAME_SIZE) {
		pr_info("%s, not enough space, user(%d*%d) < need(%d*%d)",
			__func__,
			names_hdr.cnt, names_hdr.entry_sz,
			subsys_pairs->cnt, CONNFEM_FLAG_NAME_SIZE);
		return -ENOMEM;
	}

	names = cfm_container_alloc(subsys_pairs->cnt, CONNFEM_FLAG_NAME_SIZE);
	if (!names)
		return -ENOMEM;

	for (i = 0; i < subsys_pairs->cnt; i++) {
		pair = (struct connfem_epaelna_flag_pair*)
				cfm_container_entry(subsys_pairs, i);
		name = (char*)cfm_container_entry(names, i);

		if (!pair || !name) {
			pr_info("%s, unexpected %d/%d pair %p, name %p",
				__func__,
				i,
				subsys_pairs->cnt,
				pair,
				name);
			break;
		}
		memcpy(name, pair->name, CONNFEM_FLAG_NAME_SIZE);
	}

	err = cfm_ioc_epa_cont_trans(names,
				     names_hdr.cnt,
				     names_hdr.entry_sz,
				     data.names);
	cfm_container_free(names);
	names = NULL;
	return err;
}

static int cfm_ioc_epa_info_hdlr(unsigned long usr_arg)
{
	int err;
	struct connfem_epaelna_fem_info fem_info;

	if (!usr_arg) {
		pr_info("%s, invalid parameter", __func__);
		return -EINVAL;
	}

	err = connfem_epaelna_get_fem_info(&fem_info);
	if (err < 0) {
		pr_info("%s, retrieve FEM info failed", __func__);
		return err;
	}

	if (copy_to_user((void *)usr_arg, &fem_info, sizeof(fem_info)) != 0) {
		pr_info("%s, copy_to_user failed", __func__);
		return -EINVAL;
	}

	return 0;
}

static int cfm_ioc_epa_flags_stat_hdlr(unsigned long usr_arg)
{
	int err = 0;
	struct cfm_ioc_epa_flags_stat data;
	struct cfm_container *pairs = NULL;

	if (!usr_arg) {
		pr_info("%s, invalid parameter", __func__);
		return -EINVAL;
	}

	if (copy_from_user(&data, (void *)usr_arg, sizeof(data)) != 0) {
		pr_info("%s, copy_from_user failed", __func__);
		return -EINVAL;
	}

	data.cnt = 0;
	data.entry_sz = 0;

	if (data.subsys >= CONNFEM_SUBSYS_NUM) {
		pr_info("%s, unsupported subsys %d", __func__, data.subsys);
		err = -EINVAL;
		goto flags_stat_done;
	}

	if (!connfem_ctx) {
		pr_info("%s, %d '%s', No ConnFem context",
			__func__, data.subsys, cfm_subsys_name[data.subsys]);
		err = -EOPNOTSUPP;
		goto flags_stat_done;
	}

	pairs = connfem_ctx->epaelna.flags_cfg[data.subsys].pairs;
	if (!pairs) {
		pr_info("%s, %d '%s', no flags",
			__func__,
			data.subsys,
			cfm_subsys_name[data.subsys]);
	} else {
		data.cnt = pairs->cnt;
		data.entry_sz = pairs->entry_sz;
	}

	pr_info("%s, %d '%s', return cnt:%d, entry_sz:%d",
		__func__, data.subsys, cfm_subsys_name[data.subsys],
		data.cnt, data.entry_sz);

flags_stat_done:
	if (copy_to_user((void *)usr_arg, &data, sizeof(data)) != 0) {
		pr_info("%s, copy_to_user failed", __func__);
		return -EINVAL;
	}

	return err;
}

static int cfm_ioc_epa_flags_hdlr(unsigned long usr_arg)
{
	int err = 0;
	struct cfm_ioc_epa_flags data;
	struct cfm_container pairs;
	struct cfm_container *subsys_pairs;

	if (!usr_arg) {
		pr_info("%s, invalid parameter", __func__);
		return -EINVAL;
	}

	err = copy_from_user(&data, (void *)usr_arg, sizeof(data));
	if (err != 0) {
		pr_info("%s, copy_from_user failed", __func__);
		return -EINVAL;
	}

	if (!data.pairs) {
		pr_info("%s, invalid parameter, pairs is NULL", __func__);
		return -EINVAL;
	}

	if (copy_from_user(&pairs, (void *)data.pairs,
			   sizeof(struct cfm_container)) != 0) {
		pr_info("%s, copy_from_user failed", __func__);
		return -EINVAL;
	}

	if (data.subsys >= CONNFEM_SUBSYS_NUM) {
		pr_info("%s, unsupported subsys %d", __func__, data.subsys);
		cfm_ioc_epa_cont_empty(data.pairs);
		return -EINVAL;
	}

	if (!connfem_ctx) {
		pr_info("%s, %d '%s', No ConnFem context",
			__func__, data.subsys, cfm_subsys_name[data.subsys]);
		return -EOPNOTSUPP;
	}

	pr_info("%s, %d '%s'",
		__func__, data.subsys, cfm_subsys_name[data.subsys]);

	/* Updating flags pairs */
	subsys_pairs = connfem_ctx->epaelna.flags_cfg[data.subsys].pairs;
	return cfm_ioc_epa_cont_trans(subsys_pairs,
				      pairs.cnt,
				      pairs.entry_sz,
				      data.pairs);
}

static long connfem_dev_unlocked_ioctl(struct file *filp, unsigned int cmd,
				       unsigned long arg)
{
	int err = 0;

	switch (cmd) {
	case CFM_IOC_IS_AVAILABLE:
		err = cfm_ioc_is_available_hdlr(arg);
		break;

	case CFM_IOC_EPA_FN_STAT:
		err = cfm_ioc_epa_fn_stat_hdlr(arg);
		break;

	case CFM_IOC_EPA_FN:
		err = cfm_ioc_epa_fn_hdlr(arg);
		break;

	case CFM_IOC_EPA_INFO:
		err = cfm_ioc_epa_info_hdlr(arg);
		break;

	case CFM_IOC_EPA_FLAGS_STAT:
		err = cfm_ioc_epa_flags_stat_hdlr(arg);
		break;

	case CFM_IOC_EPA_FLAGS:
		err = cfm_ioc_epa_flags_hdlr(arg);
		break;

	default:
		pr_info("Unsupported ioctl cmd 0x%x", cmd);
		err = -EOPNOTSUPP;
		break;
	};

	return err;
}

#ifdef CONFIG_COMPAT
static long connfem_dev_compat_ioctl(struct file *filp, unsigned int cmd,
							unsigned long arg)
{
	pr_info("cmd:0x%x, arg:0x%lx, redirecting to unlocked ioctl", cmd, arg);
	return connfem_dev_unlocked_ioctl(filp, cmd, arg);
}
#endif

static int connfem_plat_probe(struct platform_device *pdev)
{
	struct connfem_context *cfm = NULL;
	int err = 0;

	if (connfem_ctx && connfem_ctx->epaelna.available == true) {
		pr_info("Config file parse done, check platform device data");

		/* To operate platform device, device tree data must be used. 
		 * For example, kenerl provide pinctrl API to apply GPIO PINMUX.
		 * The parameters are get from platform device and device tree.
		 * Hence, beside config file, the device tree should also be checked.
		 */
		if (of_device_get_match_data(&pdev->dev)) {
			connfem_ctx->pdev = pdev;
			cfm_dt_cfg_ext(connfem_ctx);
		}
		goto probe_end;
	}

	cfm = (struct connfem_context *)of_device_get_match_data(&pdev->dev);
	if (!cfm) {
		pr_info("Probe, missing platform device data for '%s'",
			pdev->name);
		err = -EOPNOTSUPP;
		goto probe_end;
	}

	cfm->pdev = pdev;
	pr_info("Probed '%s': 0x%x", pdev->name, cfm->id);

	err = cfm_dt_parse(cfm);
	if (err < 0) {
		pr_info("%s device tree error %d",
			pdev->name, err);

		/* Set error to EPROBE_DEFER due to currently EAGAIN is
		 * get only when PMIC is not ready. Return immediately
		 * and wait for next probe() callback
		 */
		if (err == -EAGAIN) {
			pr_info("Get -EAGAIN and set error to -EPROBE_DEFER");
			err = -EPROBE_DEFER;
			goto probe_end;
		}

		/* We dont goto probe_end because there could be some critical
		 * information being parsed despite an error occurred.
		 *	ex.
		 *	- cfm->dt.hwid
		 *	- cfm->epaelna.available
		 *	- cfm->epaelna.fem_info
		 *	- flags, parts, ...
		 * These should perhaps be moved to a separate "persistent"
		 * section in the connfem_context structure.
		 *
		 *   // goto probe_end;
		 */
	}

	cfm->src = CFM_SRC_DEVICE_TREE;

	if (connfem_ctx) {
		pr_info("Failed to register '%s' context, '%s' exists",
			pdev->name,
			connfem_ctx->pdev->name);
		cfm_context_free(cfm);
		err = -EINVAL;
		goto probe_end;
	}

	connfem_ctx = cfm;
	connfem_cdev_ctx.cfm = cfm;
	pr_info("ConnFem context '%s' registered, linked to cdev", pdev->name);

probe_end:
#if (CONNFEM_TEST_ENABLED == 1)
	connfem_test();
#endif

	return err;
}

static int connfem_plat_remove(struct platform_device *pdev)
{
	struct connfem_context *cfm = NULL;

	cfm = (struct connfem_context *)of_device_get_match_data(&pdev->dev);
	if (!cfm) {
		pr_info("Remove, missing platform device data for '%s'",
			pdev->name);
		return -EOPNOTSUPP;
	}

	if (connfem_ctx == cfm)
		connfem_ctx = NULL;

	cfm_context_free(cfm);

	return 0;
}

static int __init connfem_mod_init(void)
{
	int ret = 0;

	pr_info("Internal load: %d", connfem_is_internal());

	cfm_cfg_process(config_file);

	/* Init global context */
	memset(&connfem_cdev_ctx, 0, sizeof(struct connfem_cdev_context));

	if (connfem_ctx && connfem_ctx->epaelna.available == true)
		connfem_cdev_ctx.cfm = connfem_ctx;

	/* Platform device */
	ret = platform_driver_register(&connfem_plat_drv);
	if (ret < 0) {
		pr_info("ConnFem platform driver registration failed: %d",
				ret);
		goto mod_init_err_skip_free;
	}

	/* Char Device: Dynamic allocate Major number */
	connfem_cdev_ctx.devId = MKDEV(connfem_major, 0);
	ret = alloc_chrdev_region(&connfem_cdev_ctx.devId, 0, CONNFEM_DEV_NUM,
							  CONNFEM_DRIVER_NAME);
	if (ret < 0) {
		pr_info("ConnFem alloc chrdev region failed: %d", ret);
		ret = -20;
		goto mod_init_err_skip_free;
	}
	connfem_major = MAJOR(connfem_cdev_ctx.devId);
	pr_info("ConnFem DevID major %d", connfem_major);

	/* Char Device: Create class */
	connfem_cdev_ctx.class = class_create(THIS_MODULE, CONNFEM_DRIVER_NAME);
	if (IS_ERR(connfem_cdev_ctx.class)) {
		pr_info("ConnFem create class failed");
		ret = -30;
		goto mod_init_err;
	}

	/* Char Device: Create device */
	connfem_cdev_ctx.chrdev = device_create(connfem_cdev_ctx.class, NULL,
						connfem_cdev_ctx.devId, NULL,
						CONNFEM_DRIVER_NAME);
	if (!connfem_cdev_ctx.chrdev) {
		pr_info("ConnFem create device failed");
		ret = -40;
		goto mod_init_err;
	}

	/* Char Device: Init device */
	cdev_init(&connfem_cdev_ctx.cdev, &connfem_dev_fops);
	connfem_cdev_ctx.cdev.owner = THIS_MODULE;

	/* Char Device: Add device, visible from file system hereafter */
	ret = cdev_add(&connfem_cdev_ctx.cdev, connfem_cdev_ctx.devId,
				   CONNFEM_DEV_NUM);
	if (ret < 0) {
		pr_info("ConnFem add device failed");
		ret = -50;
		goto mod_init_err;
	}

	pr_info("%s, ret: %d", __func__, ret);
	return ret;

mod_init_err:
	if (connfem_cdev_ctx.chrdev) {
		device_destroy(connfem_cdev_ctx.class, connfem_cdev_ctx.devId);
		connfem_cdev_ctx.chrdev = NULL;
	}

	if (connfem_cdev_ctx.class) {
		class_destroy(connfem_cdev_ctx.class);
		connfem_cdev_ctx.class = NULL;
	}

	unregister_chrdev_region(connfem_cdev_ctx.devId, CONNFEM_DEV_NUM);

mod_init_err_skip_free:
	pr_info("%s, failed: %d", __func__, ret);
	return ret;
}

static void __exit connfem_mod_exit(void)
{
	pr_info("%s", __func__);

	cdev_del(&connfem_cdev_ctx.cdev);

	if (connfem_cdev_ctx.chrdev) {
		device_destroy(connfem_cdev_ctx.class, connfem_cdev_ctx.devId);
		connfem_cdev_ctx.chrdev = NULL;
	}

	if (connfem_cdev_ctx.class) {
		class_destroy(connfem_cdev_ctx.class);
		connfem_cdev_ctx.class = NULL;
	}

	unregister_chrdev_region(connfem_cdev_ctx.devId, CONNFEM_DEV_NUM);

	platform_driver_unregister(&connfem_plat_drv);
}

module_init(connfem_mod_init);
module_exit(connfem_mod_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Connsys FEM (Front-End-Module) Driver");
MODULE_AUTHOR("Dennis Lin <dennis.lin@mediatek.com>");
MODULE_AUTHOR("Brad Chou <brad.chou@mediatek.com>");
module_param(connfem_major, uint, 0644);
module_param(epa_elna_hwid, uint, 0644);
module_param(config_file, charp, 0644);
