// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2023 MediaTek Inc.
 */

#define pr_fmt(fmt) KBUILD_MODNAME "@(%s:%d) " fmt, __func__, __LINE__

#include <asm/atomic.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>

#include <connectivity_build_in_adapter.h>

#include "../../../../base/include/osal.h"
#include "../include/consys_hw.h"
#include "../include/consys_reg_util.h"
#include "../include/pmic_mng.h"
#include "include/mt6878.h"
#include "include/mt6878_consys_reg_offset.h"
#include "include/mt6878_pmic.h"
#include "include/mt6878_pos.h"
#include "include/mt6878_pos_gen.h"
#include "conninfra_conf.h"

/*******************************************************************************
*                         C O M P I L E R   F L A G S
********************************************************************************
*/

/*******************************************************************************
*                                 M A C R O S
********************************************************************************
*/

/*******************************************************************************
*                    E X T E R N A L   R E F E R E N C E S
********************************************************************************
*/

/*******************************************************************************
*                              C O N S T A N T S
********************************************************************************
*/

/*******************************************************************************
*                             D A T A   T Y P E S
********************************************************************************
*/
static struct regulator *reg_VCN13;
static struct regulator *reg_VRFIO18; /* MT6363 workaround VCN15 -> VRFIO18 */

static struct regulator *reg_VCN33_1;
static struct regulator *reg_VCN33_2;
static struct regulator *reg_VANT18;

static struct regulator *reg_buckboost;

static struct notifier_block vrfio18_nb;
static struct notifier_block vcn13_nb;

static struct conninfra_dev_cb* g_dev_cb;

/*******************************************************************************
*                  F U N C T I O N   D E C L A R A T I O N S
********************************************************************************
*/
static int consys_plt_pmic_get_from_dts_mt6878(struct platform_device*, struct conninfra_dev_cb*);

static int consys_plt_pmic_common_power_ctrl_mt6878(unsigned int, unsigned int curr_status, unsigned int next_status);
static int consys_plt_pmic_common_power_ctrl_mt6878_6631(unsigned int, unsigned int curr_status, unsigned int next_status);
static int consys_plt_pmic_common_power_ctrl_mt6878_6631_6686(unsigned int, unsigned int curr_status, unsigned int next_status);
static int consys_plt_pmic_common_power_low_power_mode_mt6878(unsigned int, unsigned int curr_status, unsigned int next_status);
static int consys_plt_pmic_common_power_low_power_mode_mt6878_6631(unsigned int, unsigned int curr_status, unsigned int next_status);
static int consys_plt_pmic_common_power_low_power_mode_mt6878_6631_6686(unsigned int, unsigned int curr_status, unsigned int next_status);
static int consys_plt_pmic_wifi_power_ctrl_mt6878(unsigned int);
static int consys_plt_pmic_bt_power_ctrl_mt6878(unsigned int);
static int consys_plt_pmic_gps_power_ctrl_mt6878(unsigned int);
static int consys_plt_pmic_gps_power_ctrl_mt6878_6631(unsigned int);
static int consys_plt_pmic_gps_power_ctrl_mt6878_6631_6686(unsigned int);
static int consys_plt_pmic_fm_power_ctrl_mt6878(unsigned int);
static int consys_plt_pmic_fm_power_ctrl_mt6878_6631(unsigned int);
static int consys_plt_pmic_fm_power_ctrl_mt6878_6631_6686(unsigned int);
static int consys_plt_pmic_no_need_ctrl_mt6878(unsigned int);
static int consys_pmic_vcn33_1_power_ctl_mt6878_lg(bool);
static int consys_pmic_vcn33_1_power_ctl_mt6878_6631_lg(bool);
static int consys_pmic_vcn33_1_power_ctl_mt6878_rc(bool);
static int consys_pmic_vcn33_1_power_ctl_mt6878_6631_rc(bool);
static int consys_pmic_vcn33_1_power_ctl_mt6878_6631_6686_rc(bool);
static int consys_pmic_vcn33_2_power_ctl_mt6878_lg(bool);
static int consys_pmic_vcn33_2_power_ctl_mt6878_6631_lg(bool);
static int consys_pmic_vcn33_2_power_ctl_mt6878_rc(bool);
static int consys_pmic_vcn33_2_power_ctl_mt6878_6631_rc(bool);
static int consys_pmic_vcn33_2_power_ctl_mt6878_6631_6686_rc(bool);
static int consys_pmic_vant18_power_ctl_mt6878(bool);
static int consys_pmic_vant18_power_ctl_mt6878_6631_6686_rc(bool);
static int consys_pmic_vant18_power_ctl_mt6878_6631_6686_lg(bool);

static int consys_vcn13_oc_notify(struct notifier_block*, unsigned long, void*);
static int consys_vrfio18_oc_notify(struct notifier_block*, unsigned long, void*);
static int consys_plt_pmic_event_notifier_mt6878(unsigned int, unsigned int);
static int consys_plt_pmic_event_notifier_mt6878_6631(unsigned int, unsigned int);

const struct consys_platform_pmic_ops g_consys_platform_pmic_ops_mt6878_6637 = {
	.consys_pmic_get_from_dts = consys_plt_pmic_get_from_dts_mt6878,
	.consys_pmic_common_power_ctrl = consys_plt_pmic_common_power_ctrl_mt6878,
	.consys_pmic_common_power_low_power_mode = consys_plt_pmic_common_power_low_power_mode_mt6878,
	.consys_pmic_wifi_power_ctrl = consys_plt_pmic_wifi_power_ctrl_mt6878,
	.consys_pmic_bt_power_ctrl = consys_plt_pmic_bt_power_ctrl_mt6878,
	.consys_pmic_gps_power_ctrl = consys_plt_pmic_gps_power_ctrl_mt6878,
	.consys_pmic_fm_power_ctrl = consys_plt_pmic_fm_power_ctrl_mt6878,
	.consys_pmic_event_notifier = consys_plt_pmic_event_notifier_mt6878,
};

const struct consys_platform_pmic_ops g_consys_platform_pmic_ops_mt6878_6631 = {
	.consys_pmic_get_from_dts = consys_plt_pmic_get_from_dts_mt6878,
	.consys_pmic_common_power_ctrl = consys_plt_pmic_common_power_ctrl_mt6878_6631,
	.consys_pmic_common_power_low_power_mode = consys_plt_pmic_common_power_low_power_mode_mt6878_6631,
	.consys_pmic_wifi_power_ctrl = consys_plt_pmic_no_need_ctrl_mt6878,
	.consys_pmic_bt_power_ctrl = consys_plt_pmic_no_need_ctrl_mt6878,
	.consys_pmic_gps_power_ctrl = consys_plt_pmic_gps_power_ctrl_mt6878_6631,
	.consys_pmic_fm_power_ctrl = consys_plt_pmic_fm_power_ctrl_mt6878_6631,
	.consys_pmic_event_notifier = consys_plt_pmic_event_notifier_mt6878_6631,
};

const struct consys_platform_pmic_ops g_consys_platform_pmic_ops_mt6878_6631_6686 = {
	.consys_pmic_get_from_dts = consys_plt_pmic_get_from_dts_mt6878,
	.consys_pmic_common_power_ctrl = consys_plt_pmic_common_power_ctrl_mt6878_6631_6686,
	.consys_pmic_common_power_low_power_mode = consys_plt_pmic_common_power_low_power_mode_mt6878_6631_6686,
	.consys_pmic_wifi_power_ctrl = consys_plt_pmic_no_need_ctrl_mt6878,
	.consys_pmic_bt_power_ctrl = consys_plt_pmic_no_need_ctrl_mt6878,
	.consys_pmic_gps_power_ctrl = consys_plt_pmic_gps_power_ctrl_mt6878_6631_6686,
	.consys_pmic_fm_power_ctrl = consys_plt_pmic_fm_power_ctrl_mt6878_6631_6686,
	.consys_pmic_event_notifier = consys_plt_pmic_event_notifier_mt6878_6631,
};

int consys_plt_pmic_get_from_dts_mt6878(struct platform_device *pdev, struct conninfra_dev_cb* dev_cb)
{
	int ret;
	const struct conninfra_conf *conf = NULL;
	unsigned int vcn33_1_voltage = 0;

	g_dev_cb = dev_cb;
	reg_VCN13 = devm_regulator_get_optional(&pdev->dev, "mt6363_vcn13");
	if (IS_ERR(reg_VCN13)) {
		pr_err("Regulator_get VCN_13 fail\n");
		reg_VCN13 = NULL;
	} else {
		vcn13_nb.notifier_call = consys_vcn13_oc_notify;
		ret = devm_regulator_register_notifier(reg_VCN13, &vcn13_nb);
		if (ret)
			pr_info("VCN13 regulator notifier request failed\n");
	}

	reg_VRFIO18 = devm_regulator_get(&pdev->dev, "mt6363_vrfio18");
	if (IS_ERR(reg_VRFIO18)) {
		pr_err("Regulator_get VCN_18 fail\n");
		reg_VRFIO18 = NULL;
	} else {
		vrfio18_nb.notifier_call = consys_vrfio18_oc_notify;
		ret = devm_regulator_register_notifier(reg_VRFIO18, &vrfio18_nb);
		if (ret)
			pr_info("VRFIO18 regulator notifier request failed\n");
	}

	reg_VCN33_1 = devm_regulator_get(&pdev->dev, "mt6369_vcn33_1");
	if (IS_ERR(reg_VCN33_1)) {
		pr_err("Regulator_get VCN33_1 fail\n");
		reg_VCN33_1 = NULL;
	}
	reg_VCN33_2 = devm_regulator_get(&pdev->dev, "mt6369_vcn33_2");
	if (IS_ERR(reg_VCN33_2)) {
		pr_err("Regulator_get VCN33_2 fail\n");
		reg_VCN33_2 = NULL;
	}
	reg_VANT18 = devm_regulator_get(&pdev->dev, "mt6369_vant18");
	if (IS_ERR(reg_VANT18)) {
		pr_err("Regulator_get VANT18 fail\n");
		reg_VANT18 = NULL;
	}
	reg_buckboost = devm_regulator_get_optional(&pdev->dev, "rt6160-buckboost-2");
	if (IS_ERR(reg_buckboost)) {
		pr_info("Regulator_get buckboost fail\n");
		reg_buckboost = NULL;
	}

	/* raise VCN33_1 to 3.5V */
	conf = conninfra_conf_get_cfg();
	if (NULL == conf)
		pr_notice("[%s] Get conf fail", __func__);
	else
		vcn33_1_voltage = conf->vcn33_1_voltage;

	if (reg_VCN33_1) {
		if ((vcn33_1_voltage != 0) && (consys_get_adie_chipid_mt6878() != ADIE_6637)) {
			regulator_set_voltage(reg_VCN33_1, vcn33_1_voltage, vcn33_1_voltage);
			pr_info("[%s] Raise VCN33_1 by customized options = %u", __func__, vcn33_1_voltage);
		} else {
			regulator_set_voltage(reg_VCN33_1, 3300000, 3300000);
			pr_info("[%s] Set VCN33_1 by default options = 3300000", __func__);
		}
	}

	return 0;
}

int consys_plt_pmic_common_power_ctrl_mt6878(unsigned int enable, unsigned int curr_status, unsigned int next_status)
{
	int sleep_mode;
	int ret = 0;
	struct regmap *r = g_regmap_mt6363;

	if ((curr_status != 0) && (next_status != 0))
		return 0;

	sleep_mode = consys_get_sleep_mode_mt6878();
	if (enable) {
		/* set PMIC VRFIO18 LDO 1.7V */
		regulator_set_voltage(reg_VRFIO18, 1700000, 1700000);
		/* set PMIC VRFIO18 LDO SW_OP_EN = 1, SW_EN = 1, SW_LP = 0 (SW ON) */
		/*
		 * 1. set PMIC VRFIO18 LDO SW_OP_EN =1, SW_EN = 1, SW_LP =0 (SW ON)
		 * (by "standard kernal PMIC API" and "PMIC table")
		 */
		regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_SW_OP_EN_ADDR, 1 << 7, 1 << 7);
		ret = regulator_enable(reg_VRFIO18);
		if (ret)
			pr_err("Enable VRFIO18 fail. ret=%d\n", ret);
		regulator_set_mode(reg_VRFIO18, REGULATOR_MODE_NORMAL);

		/* set PMIC VCN13 LDO 1.35V @Normal mode; 0.95V @LPM */
		/* no need for LPM because 0.95V is default setting. */
		regulator_set_voltage(reg_VCN13, 1350000, 1350000);
		/*
		 * 1. set PMIC VCN13 LDO SW_OP_EN =1, SW_EN = 1, SW_LP =0 (SW ON)
		 * (by "standard kernal PMIC API" and "PMIC table")
		 */
		regmap_update_bits(r, MT6363_RG_LDO_VCN13_SW_OP_EN_ADDR, 1 << 7, 1 << 7);
		ret = regulator_enable(reg_VCN13);
		if (ret)
			pr_err("Enable VCN13 fail. ret=%d\n", ret);
		regulator_set_mode(reg_VCN13, REGULATOR_MODE_NORMAL);
	} else {
		/* vant18 is enabled in consys_plt_pmic_common_power_low_power_mode_mt6878 */
		/* Please refer to POS for more information */
		if (next_status != 0)
			return ret;
		consys_pmic_vant18_power_ctl_mt6878(0);

		/* vcn33_1/2 is enabled in consys_plt_pmic_common_power_low_power_mode_mt6878 */
		if (consys_is_rc_mode_enable_mt6878()) {
			consys_pmic_vcn33_2_power_ctl_mt6878_rc(0);
			consys_pmic_vcn33_1_power_ctl_mt6878_rc(0);
		}

		/* wait 1ms for off VCN33_X */
		msleep(1);

		/*
		 * set PMIC VCN13 LDO SW_OP_EN =0, SW_EN = 0, SW_LP =0 (sw disable)
		 * (by "standard kernal PMIC API" and "PMIC table")
		 * No need to turn off MT6363_RG_LDO_VCN13_SW_OP_EN_ADDR
		 */
		ret = regulator_disable(reg_VCN13);
		if (ret)
			pr_notice("%s regulator_disable err: %d", __func__, ret);
		regulator_set_mode(reg_VCN13, REGULATOR_MODE_NORMAL);

		/*
		 * set PMIC VRFIO18 LDO SW_OP_EN =0, SW_EN = 0, SW_LP =0 (sw disable)
		 * (by "standard kernal PMIC API" and "PMIC table")
		 */
		/* set PMIC VRFIO18 LDO SW_EN = 0, SW_LP =0 (sw disable)
		 * No need to turn off  MT6363_RG_LDO_VRFIO18_SW_OP_EN_ADDR
		 */
		if (!consys_is_rc_mode_enable_mt6878() || (sleep_mode == 1 || sleep_mode == 3)) {
			ret = regulator_disable(reg_VRFIO18);
			if (ret)
				pr_notice("%s regulator_disable err: %d", __func__, ret);
		}
		regulator_set_mode(reg_VRFIO18, REGULATOR_MODE_NORMAL);

		/* Set buckboost to 3.45V (for VCN33_1 & VCN33_2) */
		if (reg_buckboost) {
			regulator_set_voltage(reg_buckboost, 3450000, 3450000);
			pr_info("Set buckboost to 3.45V\n");
		}
	}
	return ret;
}

int consys_plt_pmic_common_power_low_power_mode_mt6878(unsigned int enable, unsigned int curr_status, unsigned int next_status)
{
	int ret = 0;
	int sleep_mode;
	struct regmap *r = g_regmap_mt6363;

	if ((curr_status != 0) && (next_status != 0))
		return 0;

	/* Set buckboost to 3.65V (for VCN33_1 & VCN33_2) */
	/* Notice that buckboost might not be enabled. */
	if (reg_buckboost) {
		regulator_set_voltage(reg_buckboost, 3650000, 3650000);
		pr_info("Set buckboost to 3.65V\n");
	}

	sleep_mode = consys_get_sleep_mode_mt6878();
	/* set PMIC VCN13 LDO 1.35V @Normal mode; 0.95V @LPM */
	/* no need for LPM because 0.95V is default setting. */
	/* sleep mode 3 with LPM voltage 1.05V */
	if (sleep_mode == 3)
		regmap_update_bits(r, MT6363_RG_LDO_VCN13_VOSEL_SLEEP_ADDR, 0x7f, 0x1);

	if (consys_is_rc_mode_enable_mt6878()) {
		/* 1. set PMIC VRFIO18 LDO PMIC HW mode control by PMRC_EN[9][8][7][6] */
		/* 1.1. set PMIC VRFIO18 LDO op_mode = 0 */
		/* 1.2. set PMIC VRFIO18 LDO HW_OP_EN = 1, HW_OP_CFG = 0 */
		regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_RC9_OP_MODE_ADDR, 1 << 1, 0 << 1);
		regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_RC9_OP_EN_ADDR,   1 << 1, 1 << 1);
		regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_RC9_OP_CFG_ADDR,  1 << 1, 0 << 1);
		regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_RC8_OP_MODE_ADDR, 1 << 0, 0 << 0);
		regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_RC8_OP_EN_ADDR,   1 << 0, 1 << 0);
		regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_RC8_OP_CFG_ADDR,  1 << 0, 0 << 0);
		regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_RC7_OP_MODE_ADDR, 1 << 7, 0 << 7);
		regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_RC7_OP_EN_ADDR,   1 << 7, 1 << 7);
		regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_RC7_OP_CFG_ADDR,  1 << 7, 0 << 7);
		regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_RC6_OP_MODE_ADDR, 1 << 6, 0 << 6);
		regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_RC6_OP_EN_ADDR,   1 << 6, 1 << 6);
		regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_RC6_OP_CFG_ADDR,  1 << 6, 0 << 6);

		if (sleep_mode == 1 || sleep_mode == 3) {
			/* 2. set PMIC VRFIO18 LDO SW_OP_EN =1, SW_EN = 1, SW_LP =1 */
			/*
			 * skip set PMIC VRFIO18 LDO SW_OP_EN =1, SW_EN = 1
			 * it is set on .consys_pmic_common_power_ctrl
			 */
			regulator_set_mode(reg_VRFIO18, REGULATOR_MODE_IDLE);
		} else {
			/* 2. set PMIC VRFIO18 LDO SW_OP_EN =1, SW_EN = 0, SW_LP =0 */
			regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_SW_OP_EN_ADDR, 1 << 7, 1 << 7);
			ret = regulator_disable(reg_VRFIO18);
			if (ret)
				pr_notice("%s regulator_disable err: %d", __func__, ret);
			regulator_set_mode(reg_VRFIO18, REGULATOR_MODE_NORMAL);
		}

		/* 1. set PMIC VCN13 LDO PMIC HW mode control by PMRC_EN[9][8][7][6] */
		/* 1.1. set PMIC VCN13 LDO op_mode = 0 */
		/* 1.2. set PMIC VCN13 LDO HW_OP_EN = 1, HW_OP_CFG = 0 */
		regmap_update_bits(r, MT6363_RG_LDO_VCN13_RC9_OP_MODE_ADDR, 1 << 1, 0 << 1);
		regmap_update_bits(r, MT6363_RG_LDO_VCN13_RC9_OP_EN_ADDR,   1 << 1, 1 << 1);
		regmap_update_bits(r, MT6363_RG_LDO_VCN13_RC9_OP_CFG_ADDR,  1 << 1, 0 << 1);
		regmap_update_bits(r, MT6363_RG_LDO_VCN13_RC8_OP_MODE_ADDR, 1 << 0, 0 << 0);
		regmap_update_bits(r, MT6363_RG_LDO_VCN13_RC8_OP_EN_ADDR,   1 << 0, 1 << 0);
		regmap_update_bits(r, MT6363_RG_LDO_VCN13_RC8_OP_CFG_ADDR,  1 << 0, 0 << 0);
		regmap_update_bits(r, MT6363_RG_LDO_VCN13_RC7_OP_MODE_ADDR, 1 << 7, 0 << 7);
		regmap_update_bits(r, MT6363_RG_LDO_VCN13_RC7_OP_EN_ADDR,   1 << 7, 1 << 7);
		regmap_update_bits(r, MT6363_RG_LDO_VCN13_RC7_OP_CFG_ADDR,  1 << 7, 0 << 7);
		regmap_update_bits(r, MT6363_RG_LDO_VCN13_RC6_OP_MODE_ADDR, 1 << 6, 0 << 6);
		regmap_update_bits(r, MT6363_RG_LDO_VCN13_RC6_OP_EN_ADDR,   1 << 6, 1 << 6);
		regmap_update_bits(r, MT6363_RG_LDO_VCN13_RC6_OP_CFG_ADDR,  1 << 6, 0 << 6);

		/* 2. set PMIC VCN13 LDO SW_OP_EN =1, SW_EN = 1, SW_LP =1 */
		regmap_update_bits(r, MT6363_RG_LDO_VCN13_SW_OP_EN_ADDR, 1 << 7, 1 << 7);
		regulator_set_mode(reg_VCN13, REGULATOR_MODE_IDLE);
	} else {
		/* 1. set PMIC VRFIO18 LDO PMIC HW mode control by SRCCLKENA0 */
		/* 1.1. set PMIC VRFIO18 LDO op_mode = 1 */
		regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_HW0_OP_MODE_ADDR, 1 << 0, 1 << 0);

		/*  if (A-die sleep mode-2 ){ */
		/*    1.2. set PMIC VRFIO18 LDO HW_OP_EN = 1, HW_OP_CFG = 0 */
		/*  }else{ //A-die sleep mode-1 */
		/*    1.2. set PMIC VRFIO18 LDO HW_OP_EN = 1, HW_OP_CFG = 1 */
		if (sleep_mode == 2) {
			regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_HW0_OP_EN_ADDR,   1 << 0, 1 << 0);
			regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_HW0_OP_CFG_ADDR,  1 << 0, 0 << 0);
		} else {
			regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_HW0_OP_EN_ADDR,   1 << 0, 1 << 0);
			regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_HW0_OP_CFG_ADDR,  1 << 0, 1 << 0);
		}

		/* 2. set PMIC VRFIO18 LDO SW_OP_EN =1, SW_EN = 1, SW_LP =0 */
		/*
		 * skip set PMIC VRFIO18 LDO SW_OP_EN =1, SW_EN = 1
		 * it is set on .consys_pmic_common_power_ctrl
		 */
		regulator_set_mode(reg_VRFIO18, REGULATOR_MODE_NORMAL);

		/* 1. set PMIC VCN13 LDO PMIC HW mode control by SRCCLKENA0 */
		/* 1.1. set PMIC VCN13 LDO op_mode = 1 */
		regmap_update_bits(r, MT6363_RG_LDO_VCN13_HW0_OP_MODE_ADDR, 1 << 0, 1 << 0);

		/* 1.2. set PMIC VCN13 LDO HW_OP_EN = 1, HW_OP_CFG = 1 */
		regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_HW0_OP_EN_ADDR,   1 << 0, 1 << 0);
		regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_HW0_OP_CFG_ADDR,  1 << 0, 1 << 0);

		/* 2. set PMIC VCN13 LDO SW_OP_EN =1, SW_EN = 1, SW_LP =0 */
		/*
		 * skip set PMIC reg_VCN13 LDO SW_OP_EN =1, SW_EN = 1
		 * it is set on .consys_pmic_common_power_ctrl
		 */
		regulator_set_mode(reg_VCN13, REGULATOR_MODE_NORMAL);
	}

	if (consys_is_rc_mode_enable_mt6878()) {
		consys_pmic_vcn33_1_power_ctl_mt6878_rc(enable);
		consys_pmic_vcn33_2_power_ctl_mt6878_rc(enable);
	}
	consys_pmic_vant18_power_ctl_mt6878(enable);
	return ret;
}

int consys_plt_pmic_wifi_power_ctrl_mt6878(unsigned int enable)
{
	int ret;

	/* necessary in legacy mode only */
	if (consys_is_rc_mode_enable_mt6878())
		return 0;

	ret = consys_pmic_vcn33_1_power_ctl_mt6878_lg(enable);
	if (ret)
		pr_info("%s VCN33_1 fail\n", (enable? "Enable" : "Disable"));

	ret = consys_pmic_vcn33_2_power_ctl_mt6878_lg(enable);
	if (ret)
		pr_info("%s VCN33_2 fail\n", (enable? "Enable" : "Disable"));

	return ret;
}

int consys_plt_pmic_bt_power_ctrl_mt6878(unsigned int enable)
{
	/* necessary in legacy mode only */
	if (consys_is_rc_mode_enable_mt6878())
		return 0;
	return consys_pmic_vcn33_1_power_ctl_mt6878_lg(enable);
}

int consys_plt_pmic_gps_power_ctrl_mt6878(unsigned int enable)
{
	return 0;
}

int consys_plt_pmic_fm_power_ctrl_mt6878(unsigned int enable)
{
	return 0;
}

int consys_plt_pmic_gps_power_ctrl_mt6878_6631(unsigned int enable)
{
	if (consys_is_rc_mode_enable_mt6878()) {
		consys_pmic_vcn33_2_power_ctl_mt6878_6631_rc(enable);
	} else {
		consys_pmic_vcn33_2_power_ctl_mt6878_6631_lg(enable);
	}

	return 0;
}

int consys_plt_pmic_fm_power_ctrl_mt6878_6631(unsigned int enable)
{
	if (consys_is_rc_mode_enable_mt6878()) {
		consys_pmic_vcn33_2_power_ctl_mt6878_6631_rc(enable);
	} else {
		consys_pmic_vcn33_2_power_ctl_mt6878_6631_lg(enable);
	}

	return 0;
}

int consys_plt_pmic_gps_power_ctrl_mt6878_6631_6686(unsigned int enable)
{
	if (consys_is_rc_mode_enable_mt6878()) {
		consys_pmic_vant18_power_ctl_mt6878_6631_6686_rc(enable);
	} else {
		consys_pmic_vant18_power_ctl_mt6878_6631_6686_lg(enable);
	}

	return 0;
}

int consys_plt_pmic_fm_power_ctrl_mt6878_6631_6686(unsigned int enable)
{
	if (consys_is_rc_mode_enable_mt6878()) {
		consys_pmic_vcn33_2_power_ctl_mt6878_6631_6686_rc(enable);
		consys_m10_srclken_cfg_mt6878_gen(enable);
	} else {
		consys_pmic_vcn33_2_power_ctl_mt6878_6631_lg(enable);
	}

	return 0;
}

static int consys_pmic_vcn33_1_power_ctl_mt6878_rc(bool enable)
{
	int sleep_mode;
	struct regmap *r = g_regmap_mt6369;
	int ret;

	sleep_mode = consys_get_sleep_mode_mt6878();
	if (enable) {
		/* 1. set PMIC VCN33_1 LDO PMIC HW mode control by PMRC_EN[8][7] */
		/* 1.1. set PMIC VCN33_1 LDO op_mode = 0 */
		/* 1.2. set PMIC VCN33_1 LDO  HW_OP_EN = 1, HW_OP_CFG = 0 */
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_1_RC8_OP_MODE_ADDR, 1 << 0, 0 << 0);
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_1_RC8_OP_EN_ADDR,   1 << 0, 1 << 0);
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_1_RC8_OP_CFG_ADDR,  1 << 0, 0 << 0);
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_1_RC7_OP_MODE_ADDR, 1 << 7, 0 << 7);
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_1_RC7_OP_EN_ADDR,   1 << 7, 1 << 7);
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_1_RC7_OP_CFG_ADDR,  1 << 7, 0 << 7);

		if (sleep_mode == 3) {
			/*
			 * 2. set PMIC VCN33_1 LDO SW_OP_EN =1, SW_EN = 1, SW_LP =0 (sw on)
			 * (by ""standard kernal PMIC API"" and ""PMIC table"")
			 */
			regmap_update_bits(r, MT6369_RG_LDO_VCN33_1_SW_OP_EN_ADDR, 1 << 7, 1 << 7);
			ret = regulator_enable(reg_VCN33_1);
			if (ret)
				pr_notice("%s regulator_enable err: %d", __func__, ret);
			regulator_set_mode(reg_VCN33_1, REGULATOR_MODE_NORMAL);

			/* 3. wait 210us */
			usleep_range(210, 1000);

			/*
			 * 4. set PMIC VCN33_1 LDO SW_OP_EN =1, SW_EN = 1, SW_LP =1 (sw lp)
			 * (by ""standard kernal PMIC API"" and ""PMIC table"")
			 */
			regulator_set_mode(reg_VCN33_1, REGULATOR_MODE_IDLE);
		} else {
			/*
			 * 2. set PMIC VCN33_1 LDO SW_OP_EN =0, SW_EN = 0, SW_LP =0 (sw disable)
			 * (by ""standard kernal PMIC API"" and ""PMIC table"")
			 * No need to turn off MT6369_RG_LDO_VCN33_1_SW_OP_EN_ADDR
			 */
			regulator_set_mode(reg_VCN33_1, REGULATOR_MODE_NORMAL);
		}
	} else {
		/*
		 * set PMIC VCN33_1 LDO SW_OP_EN =0, SW_EN = 0, SW_LP =0 (sw disable)
		 * (by "standard kernal PMIC API" and "PMIC table")
		 * No need to turn off MT6369_RG_LDO_VCN33_1_SW_OP_EN_ADDR
		 */
		if (sleep_mode == 3)
			regulator_disable(reg_VCN33_1);
		regulator_set_mode(reg_VCN33_1, REGULATOR_MODE_NORMAL);
	}

	return 0;
}

static int consys_pmic_vcn33_1_power_ctl_mt6878_lg(bool enable)
{
	struct regmap *r = g_regmap_mt6369;
	static int enable_count = 0;
	int ret;

	/* In legacy mode, VCN33_1 should be turned on either WIFI or BT is on */
	/* we use a counter to record the usage. */
	if (enable)
		enable_count++;
	else
		enable_count--;

	pr_info("%s enable_count %d\n", __func__, enable_count);
	if (enable_count < 0 || enable_count >= 2) {
		pr_info("enable_count %d is unexpected!!!\n", enable_count);
		return 0;
	}

	if (enable_count == 0) {
		/*
		 * set PMIC VCN33_1 LDO SW_OP_EN =0, SW_EN = 0, SW_LP =0 (sw disable)
		 * (by "standard kernal PMIC API" and "PMIC table")
		 * No need to turn off MT6369_RG_LDO_VCN33_1_SW_OP_EN_ADDR
		 */
		ret = regulator_disable(reg_VCN33_1);
		if (ret)
			pr_notice("%s regulator_disable err: %d", __func__, ret);
		regulator_set_mode(reg_VCN33_1, REGULATOR_MODE_NORMAL);
		return ret;
	}

	/* vcn33_1 is still on */
	if (enable_count == 1 && enable == 0)
		return 0;

	/* !!! Notice that following steps will be executed only when enable_count == 1 !!!*/
	/* 1. set PMIC VCN33_1 LDO PMIC HW mode control by SRCCLKENA0 */
	/* 1.1. set PMIC VCN33_1 LDO op_mode = 1 */
	/* 1.2. set PMIC VCN33_1 LDO HW_OP_EN = 1, HW_OP_CFG = 0 */
	regmap_update_bits(r, MT6369_RG_LDO_VCN33_1_HW0_OP_MODE_ADDR, 1 << 0, 1 << 0);
	regmap_update_bits(r, MT6369_RG_LDO_VCN33_1_HW0_OP_EN_ADDR, 1 << 0, 1 << 0);
	regmap_update_bits(r, MT6369_RG_LDO_VCN33_1_HW0_OP_CFG_ADDR, 1 << 0, 0 << 0);

	/* 2. set PMIC VCN33_1 LDO SW_OP_EN =1, SW_EN = 1, SW_LP =0 */
	regmap_update_bits(r, MT6369_RG_LDO_VCN33_1_SW_OP_EN_ADDR, 1 << 7, 1 << 7);
	ret = regulator_enable(reg_VCN33_1);
	if (ret)
		pr_notice("%s regulator_enable err: %d", __func__, ret);
	regulator_set_mode(reg_VCN33_1, REGULATOR_MODE_NORMAL);

	return ret;
}

static int consys_pmic_vcn33_1_power_ctl_mt6878_6631_lg(bool enable)
{
	struct regmap *r = g_regmap_mt6369;
	int ret = 0;

	if (enable) {
		/* 1. set PMIC VCN33_1 LDO PMIC HW mode control by SRCCLKENA0
		 * 1.1. set PMIC VCN33_1 LDO op_mode = 1
		 * 1.2. set PMIC VCN33_1 LDO HW_OP_EN = 1, HW_OP_CFG = 1
		 */
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_1_HW0_OP_MODE_ADDR, 1 << 0, 1 << 0);
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_1_HW0_OP_EN_ADDR, 1 << 0, 1 << 0);
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_1_HW0_OP_CFG_ADDR, 1 << 0, 1 << 0);

		/* 2. set PMIC VCN33_1 LDO SW_OP_EN =1, SW_EN = 1, SW_LP =0 */
		/* Skip set PMIC SW_EN = 1, it is set on .consys_pmic_common_power_ctrl */
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_1_SW_OP_EN_ADDR, 1 << 7, 1 << 7);
		regulator_set_mode(reg_VCN33_1, REGULATOR_MODE_NORMAL);
	} else{
		/*
		 * set PMIC VCN33_1 LDO SW_OP_EN =0, SW_EN = 0, SW_LP =0 (sw disable)
		 * (by "standard kernal PMIC API" and "PMIC table")
		 * No need to turn off MT6369_RG_LDO_VCN33_1_SW_OP_EN_ADDR
		 */
		ret = regulator_disable(reg_VCN33_1);
		if (ret)
			pr_notice("%s regulator_disable err: %d", __func__, ret);
		regulator_set_mode(reg_VCN33_1, REGULATOR_MODE_NORMAL);
	}

	return ret;
}

static int consys_pmic_vcn33_2_power_ctl_mt6878_lg(bool enable)
{
	struct regmap *r = g_regmap_mt6369;
	int ret = 0;

	if (!enable) {
		/*
		 * set PMIC VCN33_2 LDO SW_OP_EN =0, SW_EN = 0, SW_LP =0 (sw disable)
		 * (by "standard kernal PMIC API" and "PMIC table")
		 * No need to turn off MT6369_RG_LDO_VCN33_2_SW_OP_EN_ADDR
		 */
		ret = regulator_disable(reg_VCN33_2);
		if (ret)
			pr_notice("%s regulator_disable err: %d", __func__, ret);
		regulator_set_mode(reg_VCN33_2, REGULATOR_MODE_NORMAL);
        } else {
		/* 1. set PMIC VCN33_2 LDO PMIC HW mode control by SRCCLKENA0 */
		/* 1.1. set PMIC VCN33_2 LDO op_mode = 1 */
		/* 1.2. set PMIC VCN33_2 LDO HW_OP_EN = 1, HW_OP_CFG = 0 */
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_2_HW0_OP_MODE_ADDR, 1 << 0, 1 << 0);
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_2_HW0_OP_EN_ADDR, 1 << 0, 1 << 0);
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_2_HW0_OP_CFG_ADDR, 1 << 0, 0 << 0);

		/* 2. set PMIC VCN33_2 LDO SW_OP_EN =1, SW_EN = 1, SW_LP =0 */
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_2_SW_OP_EN_ADDR, 1 << 7, 1 << 7);
		ret = regulator_enable(reg_VCN33_2);
		if (ret)
			pr_notice("%s regulator_enable err: %d", __func__, ret);
		regulator_set_mode(reg_VCN33_2, REGULATOR_MODE_NORMAL);
	}
	return ret;
}

static int consys_pmic_vcn33_2_power_ctl_mt6878_6631_lg(bool enable)
{
	struct regmap *r = g_regmap_mt6369;
	int ret = 0;

	if (enable) {
		/* 1. set PMIC VCN33_2 LDO PMIC HW mode control by SRCCLKENA0 */
		/* 1.1. set PMIC VCN33_2 LDO HW_OP_EN = 1, HW_OP_CFG = 1 */
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_2_HW0_OP_EN_ADDR, 1 << 0, 1 << 0);
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_2_HW0_OP_CFG_ADDR, 1 << 0, 1 << 0);

		/* 2. set PMIC VCN33_2 LDO SW_EN = 1, SW_LP =0 */
		ret = regulator_enable(reg_VCN33_2);
		if (ret)
			pr_notice("%s regulator_enable err: %d", __func__, ret);
		regulator_set_mode(reg_VCN33_2, REGULATOR_MODE_NORMAL);
        } else {
		/*
		 * set PMIC VCN33_2 LDO SW_OP_EN =0, SW_EN = 0, SW_LP =0 (sw disable)
		 * (by "standard kernal PMIC API" and "PMIC table")
		 * No need to turn off MT6369_RG_LDO_VCN33_2_SW_OP_EN_ADDR
		 */
		ret = regulator_disable(reg_VCN33_2);
		if (ret)
			pr_notice("%s regulator_disable err: %d", __func__, ret);
		regulator_set_mode(reg_VCN33_2, REGULATOR_MODE_NORMAL);
	}
	return ret;
}

static int consys_pmic_vcn33_2_power_ctl_mt6878_rc(bool enable)
{
	int sleep_mode;
	struct regmap *r = g_regmap_mt6369;
	int ret = 0;

	sleep_mode = consys_get_sleep_mode_mt6878();
	if (enable) {
		/* 1. set PMIC VCN33_2 LDO PMIC HW mode control by PMRC_EN[8] */
		/* 1.1. set PMIC VCN33_2 LDO op_mode = 0 */
		/* 1.2. set PMIC VCN33_2 LDO  HW_OP_EN = 1, HW_OP_CFG = 0 */
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_2_RC8_OP_MODE_ADDR, 1 << 0, 0 << 0);
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_2_RC8_OP_EN_ADDR,   1 << 0, 1 << 0);
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_2_RC8_OP_CFG_ADDR,  1 << 0, 0 << 0);

		if (sleep_mode == 3) {
			/*
			 * 2. set PMIC VCN33_2 LDO SW_OP_EN =1, SW_EN = 1, SW_LP =0 (sw on)
			 * (by ""standard kernal PMIC API"" and ""PMIC table"")
			 */
			regmap_update_bits(r, MT6369_RG_LDO_VCN33_2_SW_OP_EN_ADDR, 1 << 7, 1 << 7);
			ret = regulator_enable(reg_VCN33_2);
			if (ret)
				pr_notice("%s regulator_enable err: %d", __func__, ret);
			regulator_set_mode(reg_VCN33_2, REGULATOR_MODE_NORMAL);

			/* 3. wait 210us */
			usleep_range(210, 1000);

			/*
			 * 4. set PMIC VCN33_2 LDO SW_OP_EN =1, SW_EN = 1, SW_LP =1 (sw lp)
			 * (by ""standard kernal PMIC API"" and ""PMIC table"")
			 */
			regulator_set_mode(reg_VCN33_2, REGULATOR_MODE_IDLE);
		} else {
			/*
			 * 2. set PMIC VCN33_2 LDO SW_OP_EN =0, SW_EN = 0, SW_LP =0 (sw disable)
			 * (by ""standard kernal PMIC API"" and ""PMIC table"")
			 * No need to turn off MT6369_RG_LDO_VCN33_2_SW_OP_EN_ADDR
			 */
			regulator_set_mode(reg_VCN33_2, REGULATOR_MODE_NORMAL);
		}
	} else {
		/*
		 * set PMIC VCN33_2 LDO SW_OP_EN =0, SW_EN = 0, SW_LP =0 (sw disable)
		 * (by "standard kernal PMIC API" and "PMIC table")
		 * No need to turn off MT6369_RG_LDO_VCN33_2_SW_OP_EN_ADDR
		 */
		if (sleep_mode == 3)
			regulator_disable(reg_VCN33_2);
		regulator_set_mode(reg_VCN33_2, REGULATOR_MODE_NORMAL);
	}

	return 0;
}

static int consys_pmic_vant18_power_ctl_mt6878(bool enable)
{
	struct regmap *r = g_regmap_mt6369;
	int ret = 0;

	if (!enable) {
		/* 1. VANT18 will be set to SW_EN=1 only in legacy momde. */
		/* 2. VANT18 might not be enabled because power on fail before low power control is executed. */
		if (consys_is_rc_mode_enable_mt6878() == 0 && regulator_is_enabled(reg_VANT18)) {
			ret = regulator_disable(reg_VANT18);
			if (ret)
				pr_notice("%s regulator_disable err:%d", __func__, ret);
		}
		return ret;
	}

	if (consys_is_rc_mode_enable_mt6878()) {
		/* 1. set PMIC VANT18 LDO PMIC HW mode control by PMRC_EN[10][6] */
		/* 1.1. set PMIC VANT18 LDO op_mode = 0 */
		/* 1.2. set PMIC VANT18 LDO  HW_OP_EN = 1, HW_OP_CFG = 0 */
		regmap_update_bits(r, MT6369_RG_LDO_VANT18_RC10_OP_MODE_ADDR, 1 << 2, 0 << 2);
		regmap_update_bits(r, MT6369_RG_LDO_VANT18_RC10_OP_EN_ADDR,   1 << 2, 1 << 2);
		regmap_update_bits(r, MT6369_RG_LDO_VANT18_RC10_OP_CFG_ADDR,  1 << 2, 0 << 2);
		regmap_update_bits(r, MT6369_RG_LDO_VANT18_RC6_OP_MODE_ADDR,  1 << 6, 0 << 6);
		regmap_update_bits(r, MT6369_RG_LDO_VANT18_RC6_OP_EN_ADDR,    1 << 6, 1 << 6);
		regmap_update_bits(r, MT6369_RG_LDO_VANT18_RC6_OP_CFG_ADDR,   1 << 6, 0 << 6);
	} else {
		/* 1. set PMIC VANT18 LDO PMIC HW mode control by SRCCLKENA0 */
		/* 1.1. set PMIC VANT18 LDO op_mode = 1 */
		/* 1.2. set PMIC VANT18 LDO HW_OP_EN = 1, HW_OP_CFG = 0 */
		regmap_update_bits(r, MT6369_RG_LDO_VANT18_HW0_OP_MODE_ADDR, 1 << 0, 1 << 0);
		regmap_update_bits(r, MT6369_RG_LDO_VANT18_HW0_OP_EN_ADDR, 1 << 0, 1 << 0);
		regmap_update_bits(r, MT6369_RG_LDO_VANT18_HW0_OP_CFG_ADDR, 1 << 0, 0 << 0);

		/* 2. set PMIC VANT18 LDO SW_OP_EN =1, SW_EN = 1, SW_LP =0 */
		regmap_update_bits(r, MT6369_RG_LDO_VANT18_SW_OP_EN_ADDR, 1 << 7, 1 << 7);
		ret = regulator_enable(reg_VANT18);
		if (ret)
			pr_notice("%s regulator_enable err: %d", __func__, ret);
		regulator_set_mode(reg_VANT18, REGULATOR_MODE_NORMAL);
	}

	return ret;
}

#define LOG_ADIE_REG_ARRAY_SZ 512
static char adie_reg_array_buf[LOG_ADIE_REG_ARRAY_SZ] = {'\0'};
static void dump_adie_cr(enum sys_spi_subsystem subsystem, const unsigned int *adie_cr, int num, char *title)
{
#define LOG_TMP_REG_SZ 32
	char tmp[LOG_TMP_REG_SZ] = {'\0'};
	unsigned int adie_value;
	int i;

	memset(adie_reg_array_buf, '\0', LOG_ADIE_REG_ARRAY_SZ);
	for (i = 0; i < num; i++) {
		if (consys_hw_spi_read(subsystem, adie_cr[i], &adie_value) < 0) {
			pr_notice("[%s] consys_hw_spi_read failed\n", __func__);
			continue;
		}
		if (snprintf(tmp, LOG_TMP_REG_SZ, "[0x%04x: 0x%08x]", adie_cr[i], adie_value) >= 0)
			strncat(adie_reg_array_buf, tmp,
				LOG_ADIE_REG_ARRAY_SZ - strlen(adie_reg_array_buf) - 1);
	}
	pr_info("%s:%s\n", title, adie_reg_array_buf);
}

static int consys_plt_pmic_event_notifier_mt6878(unsigned int id, unsigned int event)
{
#define ATOP_DUMP_NUM 14
#define ABT_DUMP_NUM 6
#define AWF_DUMP_NUM 3
	int ret = 0;
	const unsigned int adie_top_cr_list[ATOP_DUMP_NUM] = {
		0x03C, 0x090, 0x094, 0x0A0,
		0x0C8, 0x0FC, 0xA10, 0xB00,
		0xAFC, 0x160, 0xC54, 0xC58,
		0x0BC, 0x078,
	};
	const unsigned int adie_bt_cr_list[ABT_DUMP_NUM] = {
		0xFF, 0xA4, 0x41, 0x42, 0x18, 0x15,
	};
	const unsigned int adie_wf_cr_list[AWF_DUMP_NUM] = {
		0xFFF, 0x81, 0x80,
	};


	consys_pmic_debug_log_mt6878();

	ret = consys_hw_force_conninfra_wakeup();
	if (ret) {
		pr_info("[%s] force conninfra wakeup fail\n", __func__);
		return -1;
	}

	/* dump d-die cr */
	consys_hw_is_bus_hang();

	/* dump a-die cr */
	dump_adie_cr(SYS_SPI_TOP, adie_top_cr_list, ATOP_DUMP_NUM, "A-die TOP");
	dump_adie_cr(SYS_SPI_BT, adie_bt_cr_list, ABT_DUMP_NUM, "A-die BT");
	consys_hw_adie_top_ck_en_on(CONNSYS_ADIE_CTL_HOST_CONNINFRA);
	consys_hw_spi_update_bits(SYS_SPI_TOP, 0x580, 0x00, 0x10);
	consys_hw_adie_top_ck_en_off(CONNSYS_ADIE_CTL_HOST_CONNINFRA);
	dump_adie_cr(SYS_SPI_TOP, adie_top_cr_list, ATOP_DUMP_NUM, "A-die TOP");
	dump_adie_cr(SYS_SPI_WF, adie_wf_cr_list, AWF_DUMP_NUM, "A-die WF0");
	dump_adie_cr(SYS_SPI_WF1, adie_wf_cr_list, AWF_DUMP_NUM, "A-die WF1");

	consys_hw_force_conninfra_sleep();

	return 0;
}

static int consys_plt_pmic_event_notifier_mt6878_6631(unsigned int id, unsigned int event)
{
#define ATOP_DUMP_NUM_6631 8
#define ABT_DUMP_NUM_6631 1
#define AGPS_DUMP_NUM_6631 2
	int ret = 0;
	const unsigned int adie_top_cr_list[ATOP_DUMP_NUM_6631] = {
		0x080, 0x084, 0x0C0, 0xA00,
		0xA04, 0xA08, 0xA0C, 0xA10,
	};
	const unsigned int adie_bt_cr_list[ABT_DUMP_NUM_6631] = {
		0xA4,
	};
	const unsigned int adie_gps_cr_list[AGPS_DUMP_NUM_6631] = {
		0x500, 0x501,
	};

	consys_pmic_debug_log_mt6878();

	ret = consys_hw_force_conninfra_wakeup();
	if (ret) {
		pr_info("[%s] force conninfra wakeup fail\n", __func__);
		return -1;
	}

	/* dump d-die cr */
	consys_hw_is_bus_hang();

	/* dump a-die cr */
	dump_adie_cr(SYS_SPI_TOP, adie_top_cr_list, ATOP_DUMP_NUM_6631, "A-die TOP");
	dump_adie_cr(SYS_SPI_BT, adie_bt_cr_list, ABT_DUMP_NUM_6631, "A-die BT");
	dump_adie_cr(SYS_SPI_GPS, adie_gps_cr_list, AGPS_DUMP_NUM_6631, "A-die GPS");

	consys_hw_force_conninfra_sleep();

	return 0;
}

static int consys_vcn13_oc_notify(struct notifier_block *nb, unsigned long event,
				  void *unused)
{
	static int oc_counter = 0;
	static int oc_dump = 0;

	if (event != REGULATOR_EVENT_OVER_CURRENT)
		return NOTIFY_OK;

	oc_counter++;
	pr_info("[%s] VCN13 OC times: %d\n", __func__, oc_counter);

	if (oc_counter <= 30)
		oc_dump = 1;
	else if (oc_counter == (oc_dump * 100))
		oc_dump++;
	else
		return NOTIFY_OK;

	if (g_dev_cb != NULL && g_dev_cb->conninfra_pmic_event_notifier != NULL)
		g_dev_cb->conninfra_pmic_event_notifier(0, 0);

	return NOTIFY_OK;
}

static int consys_vrfio18_oc_notify(struct notifier_block *nb, unsigned long event,
				  void *unused)
{
	static int oc_counter = 0;
	static int oc_dump = 0;

	if (event != REGULATOR_EVENT_OVER_CURRENT)
		return NOTIFY_OK;

	oc_counter++;
	pr_info("[%s] VRFIO18 OC times: %d\n", __func__, oc_counter);

	if (oc_counter <= 30)
		oc_dump = 1;
	else if (oc_counter == (oc_dump * 100))
		oc_dump++;
	else
		return NOTIFY_OK;

	if (g_dev_cb != NULL && g_dev_cb->conninfra_pmic_event_notifier != NULL)
		g_dev_cb->conninfra_pmic_event_notifier(0, 0);

	return NOTIFY_OK;
}

void consys_pmic_debug_log_mt6878(void)
{
	struct regmap *r = g_regmap_mt6363;
	struct regmap *r2 = g_regmap_mt6369;
	int vcn13 = 0, vrfio18 = 0, vcn33_1 = 0, vcn33_2 = 0, vant18 = 0;

	if (!r || !r2) {
		pr_notice("%s regmap is NULL\n", __func__);
		return;
	}

	regmap_read(r, MT6363_RG_LDO_VCN13_MON_ADDR, &vcn13);
	regmap_read(r, MT6363_RG_LDO_VRFIO18_MON_ADDR, &vrfio18);
	regmap_read(r2, MT6369_RG_LDO_VCN33_1_MON_ADDR, &vcn33_1);
	regmap_read(r2, MT6369_RG_LDO_VCN33_2_MON_ADDR, &vcn33_2);
	regmap_read(r2, MT6369_RG_LDO_VANT18_MON_ADDR, &vant18);

	pr_info("%s vcn13:0x%x,vrfio18:0x%x,vcn33_1:0x%x,vcn33_2:0x%x,vant18:0x%x\n",
		__func__, vcn13, vrfio18, vcn33_1, vcn33_2, vant18);
}

int consys_plt_pmic_common_power_ctrl_mt6878_6631(unsigned int enable, unsigned int curr_status, unsigned int next_status)
{
	int ret = 0;
	struct regmap *r = g_regmap_mt6363;
	struct regmap *r2 = g_regmap_mt6369;

	if ((curr_status != 0) && (next_status != 0))
		return 0;

	if (enable) {
		/* set PMIC VCN33_2 LDO 2.8V */
		regulator_set_voltage(reg_VCN33_2, 2800000, 2800000);

		/* Set buckboost to 3.65V (for VCN33_1 & VCN33_2) */
		/* Notice that buckboost might not be enabled. */
		if (reg_buckboost) {
			regulator_set_voltage(reg_buckboost, 3650000, 3650000);
			pr_info("Set buckboost to 3.65V\n");
		}

		/* set PMIC VRFIO18 LDO SW_OP_EN = 1, SW_EN = 1, SW_LP = 0 (SW ON) */
		/*
		 * 1. set PMIC VRFIO18 LDO SW_OP_EN =1, SW_EN = 1, SW_LP =0 (SW ON)
		 * (by "standard kernal PMIC API" and "PMIC table")
		 * (For bring-up, we use external LDO instead)
		 * (For normal case, we should use PMIC)
		 * (MT6363 workaround VCN15/18 -> VRFIO18 )
		 */
		regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_SW_OP_EN_ADDR, 1 << 7, 1 << 7);
		ret = regulator_enable(reg_VRFIO18);
		if (ret)
			pr_err("Enable VRFIO18 fail. ret=%d\n", ret);
		regulator_set_mode(reg_VRFIO18, REGULATOR_MODE_NORMAL);

		/*
		 * 2. set PMIC VCN33_1 LDO SW_OP_EN =1, SW_EN = 1, SW_LP =0 (sw on)
		 * (by ""standard kernal PMIC API"" and ""PMIC table"")
		 * (For bring-up, we use external LDO instead)
		 * (For normal case, we should use PMIC)
		 * (MT6363 workaround VCN15/18 -> VRFIO18 )
		 */
		regmap_update_bits(r2, MT6369_RG_LDO_VCN33_1_SW_OP_EN_ADDR, 1 << 7, 1 << 7);
		ret = regulator_enable(reg_VCN33_1);
		if (ret)
			pr_notice("%s regulator_enable err: %d", __func__, ret);
		regulator_set_mode(reg_VCN33_1, REGULATOR_MODE_NORMAL);

		/* 2.1 set PMIC VCN33_2 LDO SW_OP_EN =1, SW_EN = 1, SW_LP =0 */
		regmap_update_bits(r2, MT6369_RG_LDO_VCN33_2_SW_OP_EN_ADDR, 1 << 7, 1 << 7);
		ret = regulator_enable(reg_VCN33_2);
		if (ret)
			pr_notice("%s regulator_enable err: %d", __func__, ret);
		regulator_set_mode(reg_VCN33_2, REGULATOR_MODE_NORMAL);
	} else {
		/* vcn33_1/2 is enabled in consys_plt_pmic_common_power_low_power_mode_mt6878 */
		consys_pmic_vcn33_1_power_ctl_mt6878_6631_rc(0);

		/*
		 * set PMIC VRFIO18 LDO SW_OP_EN =0, SW_EN = 0, SW_LP =0 (sw disable)
		 * (by "standard kernal PMIC API" and "PMIC table")
		 */
		/* set PMIC VRFIO18 LDO SW_EN = 0, SW_LP =0 (sw disable)
		 * No need to turn off  MT6363_RG_LDO_VRFIO18_SW_OP_EN_ADDR
		 */
		ret = regulator_disable(reg_VRFIO18);
		if (ret)
			pr_notice("%s regulator_disable err: %d", __func__, ret);
		regulator_set_mode(reg_VRFIO18, REGULATOR_MODE_NORMAL);

		/* Set buckboost to 3.45V (for VCN33_1 & VCN33_2) */
		if (reg_buckboost) {
			regulator_set_voltage(reg_buckboost, 3450000, 3450000);
			pr_info("Set buckboost to 3.45V\n");
		}
	}
	return ret;
}

int consys_plt_pmic_common_power_low_power_mode_mt6878_6631(unsigned int enable, unsigned int curr_status, unsigned int next_status)
{
	int ret = 0;
	struct regmap *r = g_regmap_mt6363;
	struct regmap *r2 = g_regmap_mt6369;

	if ((curr_status != 0) && (next_status != 0))
		return 0;

	if (consys_is_rc_mode_enable_mt6878()) {
		/* 1. set PMIC VRFIO18 LDO PMIC HW mode control by PMRC_EN[9][8][7][6] */
		/* 1.1. set PMIC VRFIO18 LDO op_mode = 0 */
		/* 1.2. set PMIC VRFIO18 LDO HW_OP_EN = 1, HW_OP_CFG = 0 */
		regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_RC9_OP_MODE_ADDR, 1 << 1, 0 << 1);
		regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_RC9_OP_EN_ADDR,   1 << 1, 1 << 1);
		regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_RC9_OP_CFG_ADDR,  1 << 1, 0 << 1);
		regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_RC8_OP_MODE_ADDR, 1 << 0, 0 << 0);
		regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_RC8_OP_EN_ADDR,   1 << 0, 1 << 0);
		regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_RC8_OP_CFG_ADDR,  1 << 0, 0 << 0);
		regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_RC7_OP_MODE_ADDR, 1 << 7, 0 << 7);
		regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_RC7_OP_EN_ADDR,   1 << 7, 1 << 7);
		regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_RC7_OP_CFG_ADDR,  1 << 7, 0 << 7);
		regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_RC6_OP_MODE_ADDR, 1 << 6, 0 << 6);
		regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_RC6_OP_EN_ADDR,   1 << 6, 1 << 6);
		regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_RC6_OP_CFG_ADDR,  1 << 6, 0 << 6);

		/* 2. set PMIC VRFIO18 LDO SW_OP_EN =1, SW_EN = 1, SW_LP =1 */
		/*
		 * skip set PMIC VRFIO18 LDO SW_OP_EN =1, SW_EN = 1
		 * it is set on .consys_pmic_common_power_ctrl
		 */
		regulator_set_mode(reg_VRFIO18, REGULATOR_MODE_IDLE);

		/* 3. vcn33_1 enabled */
		consys_pmic_vcn33_1_power_ctl_mt6878_6631_rc(enable);

		/* 4. set PMIC VCN33_2 LDO SW_OP_EN =1, SW_EN = 0, SW_LP =0 */
		regmap_update_bits(r2, MT6369_RG_LDO_VCN33_2_SW_OP_EN_ADDR, 1 << 7, 1 << 7);
		ret = regulator_disable(reg_VCN33_2);
		if (ret)
			pr_notice("%s regulator_disable err: %d", __func__, ret);
		regulator_set_mode(reg_VCN33_2, REGULATOR_MODE_NORMAL);

		/* 4.1. set PMIC VCN33_2 LDO HW_OP_EN = 0, HW_OP_CFG = 0 */
		regmap_update_bits(r2, MT6369_RG_LDO_VCN33_2_HW0_OP_EN_ADDR, 1 << 0, 0 << 0);
		regmap_update_bits(r2, MT6369_RG_LDO_VCN33_2_HW0_OP_CFG_ADDR, 1 << 0, 0 << 0);
	} else {
		/* 1. set PMIC VRFIO18 LDO PMIC HW mode control by SRCCLKENA0 */
		/* 1.1. set PMIC VRFIO18 LDO op_mode = 1 */
		regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_HW0_OP_MODE_ADDR, 1 << 0, 1 << 0);

		/*    1.2. set PMIC VRFIO18 LDO HW_OP_EN = 1, HW_OP_CFG = 1 */
		regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_HW0_OP_EN_ADDR,   1 << 0, 1 << 0);
		regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_HW0_OP_CFG_ADDR,  1 << 0, 1 << 0);

		/* 2. set PMIC VRFIO18 LDO SW_OP_EN =1, SW_EN = 1, SW_LP =0 */
		/*
		 * skip set PMIC VRFIO18 LDO SW_OP_EN =1, SW_EN = 1
		 * it is set on .consys_pmic_common_power_ctrl
		 */
		regulator_set_mode(reg_VRFIO18, REGULATOR_MODE_NORMAL);

		/* 3. set PMIC VCN33_1 LDO PMIC HW mode control by SRCCLKENA0 */
		consys_pmic_vcn33_1_power_ctl_mt6878_6631_lg(enable);
	}

	return ret;
}

static int consys_pmic_vcn33_1_power_ctl_mt6878_6631_rc(bool enable)
{
	struct regmap *r = g_regmap_mt6369;

	if (enable) {
		/* 1. set PMIC VCN33_1 LDO PMIC HW mode control by PMRC_EN[8][7] */
		/* 1.1. set PMIC VCN33_1 LDO op_mode = 0 */
		/* 1.2. set PMIC VCN33_1 LDO  HW_OP_EN = 1, HW_OP_CFG = 0 */
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_1_RC8_OP_MODE_ADDR, 1 << 0, 0 << 0);
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_1_RC8_OP_EN_ADDR,   1 << 0, 1 << 0);
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_1_RC8_OP_CFG_ADDR,  1 << 0, 0 << 0);
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_1_RC7_OP_MODE_ADDR, 1 << 7, 0 << 7);
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_1_RC7_OP_EN_ADDR,   1 << 7, 1 << 7);
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_1_RC7_OP_CFG_ADDR,  1 << 7, 0 << 7);

		/*
		 * 2. set PMIC VCN33_1 LDO SW_OP_EN =1, SW_EN = 1, SW_LP =0 (sw on)
		 * (by ""standard kernal PMIC API"" and ""PMIC table"")
		 * skip set PMIC SW_EN = 1, it is set on .consys_pmic_common_power_ctrl
		 */
		regulator_set_mode(reg_VCN33_1, REGULATOR_MODE_NORMAL);

		/* 3. wait 210us */
		usleep_range(210, 1000);

		/*
		 * 4. set PMIC VCN33_1 LDO SW_OP_EN =1, SW_EN = 1, SW_LP =1 (sw lp)
		 * (by ""standard kernal PMIC API"" and ""PMIC table"")
		 */
		regulator_set_mode(reg_VCN33_1, REGULATOR_MODE_IDLE);
	} else {
		/*
		 * set PMIC VCN33_1 LDO SW_OP_EN =1, SW_EN = 0, SW_LP =0 (sw disable)
		 * (by "standard kernal PMIC API" and "PMIC table")
		 */
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_1_SW_OP_EN_ADDR, 1 << 7, 1 << 7);
		regulator_disable(reg_VCN33_1);
		regulator_set_mode(reg_VCN33_1, REGULATOR_MODE_NORMAL);
	}
	return 0;
}

static int consys_pmic_vcn33_2_power_ctl_mt6878_6631_rc(bool enable)
{
	struct regmap *r = g_regmap_mt6369;
	static int VCN28_enable_flag = 0;

	if (enable) {
		VCN28_enable_flag++;
		if (VCN28_enable_flag > 1)
			return 0;

		/* 1. set PMIC VCN33_2 LDO PMIC HW mode control by PMRC_EN[9][6] */
		/* 1.1. set PMIC VCN33_2 LDO op_mode = 0 */
		/* 1.2. set PMIC VCN33_2 LDO  HW_OP_EN = 1, HW_OP_CFG = 0 */
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_2_RC9_OP_MODE_ADDR, 1 << 1, 0 << 1);
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_2_RC9_OP_EN_ADDR,   1 << 1, 1 << 1);
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_2_RC9_OP_CFG_ADDR,  1 << 1, 0 << 1);
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_2_RC6_OP_MODE_ADDR, 1 << 6, 0 << 6);
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_2_RC6_OP_EN_ADDR,   1 << 6, 1 << 6);
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_2_RC6_OP_CFG_ADDR,  1 << 6, 0 << 6);

		/*
		 * 2. set PMIC VCN33_2 LDO SW_OP_EN =0, SW_EN = 0, SW_LP =0 (sw disable)
		 * (by ""standard kernal PMIC API"" and ""PMIC table"")
		 * No need to turn off MT6369_RG_LDO_VCN33_2_SW_OP_EN_ADDR
		 */
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_2_SW_OP_EN_ADDR, 1 << 7, 0 << 7);
		regulator_set_mode(reg_VCN33_2, REGULATOR_MODE_NORMAL);
	} else {
		VCN28_enable_flag--;
	}

	return 0;
}

int consys_plt_pmic_no_need_ctrl_mt6878(unsigned int enable)
{
	return 0;
}

int consys_plt_pmic_common_power_ctrl_mt6878_6631_6686(unsigned int enable, unsigned int curr_status, unsigned int next_status)
{
	int ret = 0;
	unsigned int curr_bt = 0;
	unsigned int curr_wifi = 0;
	unsigned int curr_fm = 0;
	unsigned int next_bt = 0;
	unsigned int next_wifi = 0;
	unsigned int next_fm = 0;
	bool bt_wifi_fm_on = false;
	bool bt_wifi_fm_off = false;
	struct regmap *r = g_regmap_mt6363;
	struct regmap *r2 = g_regmap_mt6369;

	curr_bt = (curr_status & (0x1U << CONNDRV_TYPE_BT)) >> CONNDRV_TYPE_BT;
	curr_wifi = (curr_status & (0x1U << CONNDRV_TYPE_WIFI)) >> CONNDRV_TYPE_WIFI;
	curr_fm = (curr_status & (0x1U << CONNDRV_TYPE_FM)) >> CONNDRV_TYPE_FM;

	next_bt = (next_status & (0x1U << CONNDRV_TYPE_BT)) >> CONNDRV_TYPE_BT;
	next_wifi = (next_status & (0x1U << CONNDRV_TYPE_WIFI)) >> CONNDRV_TYPE_WIFI;
	next_fm = (next_status & (0x1U << CONNDRV_TYPE_FM)) >> CONNDRV_TYPE_FM;

	if (((curr_bt + curr_wifi + curr_fm) == 0) && ((next_bt + next_wifi + next_fm) != 0))
		bt_wifi_fm_on = true;

	if (((curr_bt + curr_wifi + curr_fm) != 0) && ((next_bt + next_wifi + next_fm) == 0))
		bt_wifi_fm_off = true;

	if (bt_wifi_fm_on) {
		/* set PMIC VCN33_2 LDO 2.8V */
		regulator_set_voltage(reg_VCN33_2, 2800000, 2800000);

		/* Set buckboost to 3.65V (for VCN33_1 & VCN33_2) */
		/* Notice that buckboost might not be enabled. */
		if (reg_buckboost) {
			regulator_set_voltage(reg_buckboost, 3650000, 3650000);
			pr_info("Set buckboost to 3.65V\n");
		}

		/* set PMIC VRFIO18 LDO SW_OP_EN = 1, SW_EN = 1, SW_LP = 0 (SW ON) */
		/*
		* 1. set PMIC VRFIO18 LDO SW_OP_EN =1, SW_EN = 1, SW_LP =0 (SW ON)
		* (by "standard kernal PMIC API" and "PMIC table")
		* (For bring-up, we use external LDO instead)
		* (For normal case, we should use PMIC)
		* (MT6363 workaround VCN15/18 -> VRFIO18 )
		*/
		regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_SW_OP_EN_ADDR, 1 << 7, 1 << 7);
		ret = regulator_enable(reg_VRFIO18);
		if (ret != 0)
			pr_err("Enable VRFIO18 fail. ret=%d\n", ret);
		regulator_set_mode(reg_VRFIO18, REGULATOR_MODE_NORMAL);

		/*
		* 2. set PMIC VCN33_1 LDO SW_OP_EN =1, SW_EN = 1, SW_LP =0 (sw on)
		* (by ""standard kernal PMIC API"" and ""PMIC table"")
		* (For bring-up, we use external LDO instead)
		* (For normal case, we should use PMIC)
		* (MT6363 workaround VCN15/18 -> VRFIO18 )
		*/
		regmap_update_bits(r2, MT6369_RG_LDO_VCN33_1_SW_OP_EN_ADDR, 1 << 7, 1 << 7);
		ret = regulator_enable(reg_VCN33_1);
		if (ret != 0)
			pr_notice("%s regulator_enable err: %d", __func__, ret);
		regulator_set_mode(reg_VCN33_1, REGULATOR_MODE_NORMAL);

		/* 2. set PMIC VCN33_2 LDO SW_OP_EN =1, SW_EN = 1, SW_LP =0 */
		regmap_update_bits(r2, MT6369_RG_LDO_VCN33_2_SW_OP_EN_ADDR, 1 << 7, 1 << 7);
		ret = regulator_enable(reg_VCN33_2);
		if (ret)
			pr_notice("%s regulator_enable err: %d", __func__, ret);
		regulator_set_mode(reg_VCN33_2, REGULATOR_MODE_NORMAL);
	}

	if (bt_wifi_fm_off == true){
		consys_m10_srclken_cfg_mt6878_gen(0);
		consys_pmic_vcn33_1_power_ctl_mt6878_6631_6686_rc(0);


		/* 2. set PMIC VRFIO18 LDO SW_OP_EN =1, SW_EN = 0, SW_LP =0 */
		regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_SW_OP_EN_ADDR, 1 << 7, 1 << 7);
		ret = regulator_disable(reg_VRFIO18);
		if (ret)
			pr_notice("%s regulator_disable err: %d", __func__, ret);
		regulator_set_mode(reg_VRFIO18, REGULATOR_MODE_NORMAL);

		/* Set buckboost to 3.45V (for VCN33_1 & VCN33_2) */
		if (reg_buckboost) {
			regulator_set_voltage(reg_buckboost, 3450000, 3450000);
			pr_info("Set buckboost to 3.45V\n");
		}
	}

	return ret;
}

int consys_plt_pmic_common_power_low_power_mode_mt6878_6631_6686(unsigned int enable, unsigned int curr_status, unsigned int next_status)
{
	int ret = 0;
	struct regmap *r = g_regmap_mt6363;
	struct regmap *r2 = g_regmap_mt6369;
	unsigned int curr_bt = 0;
	unsigned int curr_wifi = 0;
	unsigned int curr_fm = 0;
	unsigned int next_bt = 0;
	unsigned int next_wifi = 0;
	unsigned int next_fm = 0;
	bool bt_wifi_fm_on = false;

	curr_bt = (curr_status & (0x1U << CONNDRV_TYPE_BT)) >> CONNDRV_TYPE_BT;
	curr_wifi = (curr_status & (0x1U << CONNDRV_TYPE_WIFI)) >> CONNDRV_TYPE_WIFI;
	curr_fm = (curr_status & (0x1U << CONNDRV_TYPE_FM)) >> CONNDRV_TYPE_FM;

	next_bt = (next_status & (0x1U << CONNDRV_TYPE_BT)) >> CONNDRV_TYPE_BT;
	next_wifi = (next_status & (0x1U << CONNDRV_TYPE_WIFI)) >> CONNDRV_TYPE_WIFI;
	next_fm = (next_status & (0x1U << CONNDRV_TYPE_FM)) >> CONNDRV_TYPE_FM;

	if (((curr_bt + curr_wifi + curr_fm) == 0) && ((next_bt + next_wifi + next_fm) != 0))
		bt_wifi_fm_on = true;

	if (consys_is_rc_mode_enable_mt6878()) {
		if (bt_wifi_fm_on) {
			/* 1. set PMIC VRFIO18 LDO PMIC HW mode control by PMRC_EN[10][8][7] */
			/* 1.1. set PMIC VRFIO18 LDO op_mode = 0 */
			/* 1.2. set PMIC VRFIO18 LDO HW_OP_EN = 1, HW_OP_CFG = 0 */

			regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_RC10_OP_MODE_ADDR, 1 << 1, 0 << 1);
			regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_RC10_OP_EN_ADDR,   1 << 1, 1 << 1);
			regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_RC10_OP_CFG_ADDR,  1 << 1, 0 << 1);
			regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_RC8_OP_MODE_ADDR, 1 << 0, 0 << 0);
			regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_RC8_OP_EN_ADDR,   1 << 0, 1 << 0);
			regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_RC8_OP_CFG_ADDR,  1 << 0, 0 << 0);
			regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_RC7_OP_MODE_ADDR, 1 << 7, 0 << 7);
			regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_RC7_OP_EN_ADDR,   1 << 7, 1 << 7);
			regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_RC7_OP_CFG_ADDR,  1 << 7, 0 << 7);


			/* 2. set PMIC VRFIO18 LDO SW_OP_EN =1, SW_EN = 1, SW_LP =1 */
			regulator_set_mode(reg_VRFIO18, REGULATOR_MODE_IDLE);
			ret = regulator_enable(reg_VRFIO18);

			/* 3. vcn33_1 enabled, disable vcn33_2 */
			consys_pmic_vcn33_1_power_ctl_mt6878_6631_6686_rc(enable);

			/* 4. set PMIC VCN33_2 LDO SW_OP_EN =1, SW_EN = 0, SW_LP =0 */
			regmap_update_bits(r2, MT6369_RG_LDO_VCN33_2_SW_OP_EN_ADDR, 1 << 7, 1 << 7);
			ret = regulator_disable(reg_VCN33_2);
			if (ret)
				pr_notice("%s regulator_disable err: %d", __func__, ret);
			regulator_set_mode(reg_VCN33_2, REGULATOR_MODE_NORMAL);

			/* 4.1. set PMIC VCN33_2 LDO HW_OP_EN = 0, HW_OP_CFG = 0 */
			regmap_update_bits(r2, MT6369_RG_LDO_VCN33_2_HW0_OP_EN_ADDR, 1 << 0, 0 << 0);
			regmap_update_bits(r2, MT6369_RG_LDO_VCN33_2_HW0_OP_CFG_ADDR, 1 << 0, 0 << 0);
		}
	} else {
		if (bt_wifi_fm_on) {
			/* 1. set PMIC VRFIO18 LDO PMIC HW mode control by SRCCLKENA0 */
			/* 1.1. set PMIC VRFIO18 LDO op_mode = 1 */
			regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_HW0_OP_MODE_ADDR, 1 << 0, 1 << 0);

			/*    1.2. set PMIC VRFIO18 LDO HW_OP_EN = 1, HW_OP_CFG = 1 */
			regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_HW0_OP_EN_ADDR,   1 << 0, 1 << 0);
			regmap_update_bits(r, MT6363_RG_LDO_VRFIO18_HW0_OP_CFG_ADDR,  1 << 0, 1 << 0);

			/* 2. set PMIC VRFIO18 LDO SW_OP_EN =1, SW_EN = 1, SW_LP =0 */
			/*
			* skip set PMIC VRFIO18 LDO SW_OP_EN =1, SW_EN = 1
			* it is set on .consys_pmic_common_power_ctrl
			*/
			regulator_set_mode(reg_VRFIO18, REGULATOR_MODE_NORMAL);

			/* 3. set PMIC VCN33_1 LDO PMIC HW mode control by SRCCLKENA0 */
			consys_pmic_vcn33_1_power_ctl_mt6878_6631_lg(enable);
		}
	}

	return ret;
}

static int consys_pmic_vcn33_1_power_ctl_mt6878_6631_6686_rc(bool enable)
{
	struct regmap *r = g_regmap_mt6369;

	if (enable) {
		/* 1. set PMIC VCN33_1 LDO PMIC HW mode control by PMRC_EN[10][8][7] */
		/* 1.1. set PMIC VCN33_1 LDO op_mode = 0 */
		/* 1.2. set PMIC VCN33_1 LDO  HW_OP_EN = 1, HW_OP_CFG = 0 */

		regmap_update_bits(r, MT6369_RG_LDO_VCN33_1_RC10_OP_MODE_ADDR, 1 << 2, 0 << 2);
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_1_RC10_OP_EN_ADDR,   1 << 2, 1 << 2);
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_1_RC10_OP_CFG_ADDR,  1 << 2, 0 << 2);
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_1_RC8_OP_MODE_ADDR, 1 << 0, 0 << 0);
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_1_RC8_OP_EN_ADDR,   1 << 0, 1 << 0);
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_1_RC8_OP_CFG_ADDR,  1 << 0, 0 << 0);
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_1_RC7_OP_MODE_ADDR, 1 << 7, 0 << 7);
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_1_RC7_OP_EN_ADDR,   1 << 7, 1 << 7);
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_1_RC7_OP_CFG_ADDR,  1 << 7, 0 << 7);

		/*
		 * 2. set PMIC VCN33_1 LDO SW_OP_EN =1, SW_EN = 1, SW_LP =0 (sw on)
		 * (by ""standard kernal PMIC API"" and ""PMIC table"")
		 * skip set PMIC SW_EN = 1, it is set on .consys_pmic_common_power_ctrl
		 */
		regulator_set_mode(reg_VCN33_1, REGULATOR_MODE_NORMAL);

		/* 3. wait 210us */
		usleep_range(210, 1000);

		/*
		 * 4. set PMIC VCN33_1 LDO SW_OP_EN =1, SW_EN = 1, SW_LP =1 (sw lp)
		 * (by ""standard kernal PMIC API"" and ""PMIC table"")
		 */
		regulator_set_mode(reg_VCN33_1, REGULATOR_MODE_IDLE);
	} else {
		/*
		 * set PMIC VCN33_1 LDO SW_OP_EN =1, SW_EN = 0, SW_LP =0 (sw disable)
		 * (by "standard kernal PMIC API" and "PMIC table")
		 */
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_1_SW_OP_EN_ADDR, 1 << 7, 1 << 7);
		regulator_disable(reg_VCN33_1);
		regulator_set_mode(reg_VCN33_1, REGULATOR_MODE_NORMAL);
	}
	return 0;
}

static int consys_pmic_vcn33_2_power_ctl_mt6878_6631_6686_rc(bool enable)
{
	struct regmap *r = g_regmap_mt6369;

	if (enable) {
		/* 1. set PMIC VCN33_2 LDO PMIC HW mode control by PMRC_EN[10] */
		/* 1.1. set PMIC VCN33_2 LDO op_mode = 0 */
		/* 1.2. set PMIC VCN33_2 LDO  HW_OP_EN = 1, HW_OP_CFG = 0 */
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_2_RC10_OP_MODE_ADDR, 1 << 2, 0 << 2);
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_2_RC10_OP_EN_ADDR,   1 << 2, 1 << 2);
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_2_RC10_OP_CFG_ADDR,  1 << 2, 0 << 2);

		/*
		 * 2. set PMIC VCN33_2 LDO SW_OP_EN =0, SW_EN = 0, SW_LP =0 (sw disable)
		 * (by ""standard kernal PMIC API"" and ""PMIC table"")
		 * No need to turn off MT6369_RG_LDO_VCN33_2_SW_OP_EN_ADDR
		 */
		regmap_update_bits(r, MT6369_RG_LDO_VCN33_2_SW_OP_EN_ADDR, 1 << 7, 0 << 7);
		regulator_set_mode(reg_VCN33_2, REGULATOR_MODE_NORMAL);
	}

	return 0;
}

static int consys_pmic_vant18_power_ctl_mt6878_6631_6686_rc(bool enable)
{
	struct regmap *r = g_regmap_mt6369;

	if (enable) {
		/* 1. set PMIC VANT18 LDO PMIC HW mode control by PMRC_EN[6] */
		/* 1.1. set PMIC VANT18 LDO op_mode = 0 */
		/* 1.2. set PMIC VANT18 LDO  HW_OP_EN = 1, HW_OP_CFG = 0 */
		regmap_update_bits(r, MT6369_RG_LDO_VANT18_RC6_OP_MODE_ADDR, 1 << 6, 0 << 6);
		regmap_update_bits(r, MT6369_RG_LDO_VANT18_RC6_OP_EN_ADDR,   1 << 6, 1 << 6);
		regmap_update_bits(r, MT6369_RG_LDO_VANT18_RC6_OP_CFG_ADDR,  1 << 6, 0 << 6);

			/*
		* 2. set PMIC VANT18 LDO SW_OP_EN =0, SW_EN = 0, SW_LP =0 (sw disable)
		* (by ""standard kernal PMIC API"" and ""PMIC table"")
		* No need to turn off MT6369_RG_LDO_VANT18_SW_OP_EN_ADDR
		*/
		regulator_set_mode(reg_VANT18, REGULATOR_MODE_NORMAL);
	}

	return 0;
}

static int consys_pmic_vant18_power_ctl_mt6878_6631_6686_lg(bool enable)
{
	struct regmap *r = g_regmap_mt6369;
	int ret = 0;

	if (enable) {
		/* 1. set PMIC VANT18 LDO PMIC HW mode control by SRCCLKENA0 */
		/* 1.1. set PMIC VANT18 LDO op_mode = 1 */
		/* 1.2. set PMIC VANT18 LDO HW_OP_EN = 1, HW_OP_CFG = 0 */
		regmap_update_bits(r, MT6369_RG_LDO_VANT18_HW0_OP_MODE_ADDR, 1 << 0, 1 << 0);
		regmap_update_bits(r, MT6369_RG_LDO_VANT18_HW0_OP_EN_ADDR, 1 << 0, 1 << 0);
		regmap_update_bits(r, MT6369_RG_LDO_VANT18_HW0_OP_CFG_ADDR, 1 << 0, 0 << 0);

		/* 2. set PMIC VANT18 LDO SW_OP_EN =1, SW_EN = 1, SW_LP =0 */
		regmap_update_bits(r, MT6369_RG_LDO_VANT18_SW_OP_EN_ADDR, 1 << 7, 1 << 7);
		ret = regulator_enable(reg_VANT18);
		if (ret)
			pr_notice("%s regulator_enable err: %d", __func__, ret);
		regulator_set_mode(reg_VANT18, REGULATOR_MODE_NORMAL);
	} else {
		/* 1. VANT18 will be set to SW_EN=1 only in legacy momde. */
		/* 2. VANT18 might not be enabled because power on fail before low power control is executed. */
		ret = regulator_disable(reg_VANT18);
		if (ret)
			pr_notice("%s regulator_disable err:%d", __func__, ret);
	}

	return ret;
}