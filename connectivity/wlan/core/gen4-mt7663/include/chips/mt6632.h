/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

/*
 ** Id: //Department/DaVinci/BRANCHES/
 *			MT6620_WIFI_DRIVER_V2_3/include/chips/mt6632.h#1
 */

/*! \file  mt6632.h
 *    \brief This file contains the info of MT6632
 */

#ifndef _MT6632_H
#define _MT6632_H

/*******************************************************************************
 *                         C O M P I L E R   F L A G S
 *******************************************************************************
 */

/*******************************************************************************
 *                    E X T E R N A L   R E F E R E N C E S
 *******************************************************************************
 */

/*******************************************************************************
 *                              C O N S T A N T S
 *******************************************************************************
 */
#define MT6632_CHIP_ID					(0x6632)
#define MT6632_SW_SYNC0					WIFI_CFG_SW_SYNC0
#define MT6632_SW_SYNC0_RDY_OFFSET		WIFI_CFG_SYNC0_RDY_OFFSET
#define MT6632_PATCH_START_ADDR			(0x000B4000)
#define MT6632_TOP_CFG_BASE			(0x0000)
#define MT6632_TX_DESC_APPEND_LENGTH            44
#define MT6632_RX_DESC_LENGTH                   16
#define MT6632_RX_INIT_EVENT_LENGTH             8
#define MT6632_RX_EVENT_HDR_LENGTH              12

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

#endif /* _MT6632_H */
