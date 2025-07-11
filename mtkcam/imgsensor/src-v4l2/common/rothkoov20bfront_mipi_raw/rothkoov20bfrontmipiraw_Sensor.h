/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2022 MediaTek Inc.
 */
/*****************************************************************************
 *
 * Filename:
 * ---------
 *     ROTHKOOV20BFRONTmipi_Sensor.h
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     CMOS sensor header file
 *
 ****************************************************************************/
#ifndef _ROTHKOOV20BFRONTMIPI_SENSOR_H
#define _ROTHKOOV20BFRONTMIPI_SENSOR_H

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define_v4l2.h"
#include "kd_imgsensor_errcode.h"

#include "rothkoov20bfront_ana_gain_table.h"
#include "rothkoov20bfront_Sensor_setting.h"

#include "adaptor-subdrv-ctrl.h"
#include "adaptor-i2c.h"
#include "adaptor.h"

#define EEPROM_READY 0	// #define it when eeprom ready

// #define it to debug for power sequence if needed
#undef PWR_SEQ_ALL_USE_FOR_AOV_MODE_TRANSITION
// #deinfe it to do AOV EINT UT under sensing mode
#undef AOV_EINT_UT
//#define AOV_EINT_UT
#endif
