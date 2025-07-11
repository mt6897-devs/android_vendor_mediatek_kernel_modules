// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#define PFX "CAM_CAL"
#define pr_fmt(fmt) PFX "[%s] " fmt, __func__

#include <linux/kernel.h>
#include "cam_cal_list.h"
#include "eeprom_i2c_common_driver.h"
#include "eeprom_i2c_custom_driver.h"
#include "cam_cal_config.h"

static struct STRUCT_CALIBRATION_LAYOUT_STRUCT cal_layout_table = {
	0x00000008, 0xFFFFFFBA, CAM_CAL_SINGLE_EEPROM_DATA,
	{
		{0x00000001, 0x00000001, 0x0000000F, xiaomi_do_module_version},
		{0x00000001, 0x00000010, 0x00000011, xiaomi_do_part_number},
		{0x00000001, 0x0000078C, 0x0000074C, xiaomi_do_single_lsc},
		{0x00000001, 0x00000754, 0x00000010, xiaomi_do_2a_gain},
		{0x00000000, 0x00000820, 0x000005F9, xiaomi_do_pdaf},
		{0x00000000, 0x00000FAE, 0x00000550, NULL},
		{0x00000000, 0x00000000, 0x00001600, xiaomi_do_dump_all},
		{0x00000000, 0x00000008, 0x00000002, NULL}
	}
};

struct STRUCT_CAM_CAL_CONFIG_STRUCT malachiteov20bfront_mtk_eeprom_aac = {
	.name = "malachiteov20bfront_mtk_eeprom_aac",
	.check_layout_function = layout_check,
	.read_function = Common_read_region,
	.layout = &cal_layout_table,
	.sensor_id = MALACHITEOV20BFRONT_SENSOR_ID,
	.i2c_write_id = 0xA2,
	.max_size = 0x2000,
	.enable_preload = 1,
	.preload_size = 0x2000,
};
