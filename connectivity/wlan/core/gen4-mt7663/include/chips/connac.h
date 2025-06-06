/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

/*
 ** Id: //Department/DaVinci/BRANCHES/
 *			MT6620_WIFI_DRIVER_V2_3/include/chips/connac.h#1
 */

/*! \file  connac.h
 *    \brief This file contains the info of CONNAC
 */

#ifdef CONNAC

#ifndef _CONNAC_H
#define _CONNAC_H

/*******************************************************************************
 *                         C O M P I L E R   F L A G S
 *******************************************************************************
 */

/*******************************************************************************
 *                    E X T E R N A L   R E F E R E N C E S
 *******************************************************************************
 */
#include "linux/sched.h"
#if CFG_MTK_ANDROID_WMT
extern void connectivity_export_show_stack(struct task_struct *tsk,
					   unsigned long *sp);
#endif

/*******************************************************************************
 *                              C O N S T A N T S
 *******************************************************************************
 */
#define CONNAC_CHIP_ID                          (0x0001)
#define CONNAC_SW_SYNC0                         CONN_CFG_ON_CONN_ON_MISC_ADDR
#define CONNAC_SW_SYNC0_RDY_OFFSET \
	CONN_CFG_ON_CONN_ON_MISC_DRV_FM_STAT_SYNC_SHFT
#define CONNAC_PATCH_START_ADDR                 (0x0001C000)
#define CONNAC_TOP_CFG_BASE			CONN_CFG_BASE
#define CONNAC_TX_DESC_APPEND_LENGTH            32
#define CONNAC_RX_DESC_LENGTH                   16
#define CONNAC_RX_INIT_EVENT_LENGTH             8
#define CONNAC_RX_EVENT_HDR_LENGTH              12

/*******************************************************************************
 *                         D A T A   T Y P E S
 *******************************************************************************
 */

/*******************************************************************************
 *                            P U B L I C   D A T A
 *******************************************************************************
 */

/*******************************************************************************
 *                           P R I V A T E   D A T A
 *******************************************************************************
 */

/*******************************************************************************
 *                                 M A C R O S
 *******************************************************************************
 */

/*******************************************************************************
 *                  F U N C T I O N   D E C L A R A T I O N S
 *******************************************************************************
 */

/*******************************************************************************
 *                              F U N C T I O N S
 *******************************************************************************
 */

#endif /* _CONNAC_H */

#endif /* CONNAC */

