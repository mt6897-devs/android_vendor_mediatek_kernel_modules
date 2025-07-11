// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/kernel.h>
#include "cam_cal_list.h"
#include "eeprom_i2c_common_driver.h"
#include "eeprom_i2c_custom_driver.h"
#include "kd_imgsensor.h"

#define MAX_EEPROM_SIZE_32K 0x8000
#define MAX_EEPROM_SIZE_16K 0x4000

struct stCAM_CAL_LIST_STRUCT g_camCalList[] = {
	/*Below is commom sensor */
	{OV48B_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX766_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_32K},
	{IMX766DUAL_SENSOR_ID, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{GC8054_SENSOR_ID, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{S5K3P9SP_SENSOR_ID, 0xA0, Common_read_region},
	{IMX481_SENSOR_ID, 0xA2, Common_read_region},
	{GC02M0_SENSOR_ID, 0xA8, Common_read_region},
	{IMX586_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX576_SENSOR_ID, 0xA2, Common_read_region},
	{IMX519_SENSOR_ID, 0xA0, Common_read_region},
	{IMX319_SENSOR_ID, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{S5K3M5SX_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX686_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{HI846_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{S5KGD1SP_SENSOR_ID, 0xA8, Common_read_region, MAX_EEPROM_SIZE_16K},
	{S5K2T7SP_SENSOR_ID, 0xA4, Common_read_region},
	{IMX386_SENSOR_ID, 0xA0, Common_read_region},
	{S5K2L7_SENSOR_ID, 0xA0, Common_read_region},
	{IMX398_SENSOR_ID, 0xA0, Common_read_region},
	{IMX350_SENSOR_ID, 0xA0, Common_read_region},
	{IMX386_MONO_SENSOR_ID, 0xA0, Common_read_region},
	{IMX499_SENSOR_ID, 0xA0, Common_read_region},
        /* o16 otp start */
        {MALACHITES5KHP3WIDE_SENSOR_ID, 0xA2, Common_read_region, MAX_EEPROM_SIZE_32K},
        {MALACHITEIMX882WIDE_SENSOR_ID, 0xA2, Common_read_region, MAX_EEPROM_SIZE_32K},
        {MALACHITEOV20BFRONT_SENSOR_ID, 0xA2, Common_read_region, MAX_EEPROM_SIZE_32K},
        {MALACHITEIMX355ULTRA_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_32K},
        {MALACHITEOV02B10MACRO_SENSOR_ID, 0xA4, Common_read_region, MAX_EEPROM_SIZE_32K},
        /* o16 otp end */
	{ROTHKOIMX906WIDE_SENSOR_ID, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{ROTHKOOV50H40WIDE_SENSOR_ID, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{ROTHKOOV20BFRONT_SENSOR_ID, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{ROTHKOGC02M1MACRO_SENSOR_ID, 0xA4, Common_read_region, MAX_EEPROM_SIZE_16K},
	{ROTHKOIMX355ULTRA_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{ROTHKOOV13B10ULTRA_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{ROTHKOS5KJN1TELE_SENSOR_ID, 0xA8, Common_read_region, MAX_EEPROM_SIZE_16K},
	{ROTHKOS5KKD1FRONT_SENSOR_ID, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},

	/*  ADD before this line */
	// XIAOMI ADD START
	{ROTHKOS5KJN1TELE_SENSOR_ID, 0xA8, Common_read_region},
	{DEGASS5KJN1TELE_SENSOR_ID, 0xA8, Common_read_region},
	{0, 0, 0}       /*end of list */
};

unsigned int cam_cal_get_sensor_list(
	struct stCAM_CAL_LIST_STRUCT **ppCamcalList)
{
	if (ppCamcalList == NULL)
		return 1;

	*ppCamcalList = &g_camCalList[0];
	return 0;
}


