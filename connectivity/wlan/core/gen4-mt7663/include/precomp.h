/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

/*
 * Id: //Department/DaVinci/BRANCHES/MT6620_WIFI_DRIVER_V2_3/include/precomp.h#2
 */

/*! \file   precomp.h
 *    \brief  Collection of most compiler flags are described here.
 *
 *    In this file we collect all compiler flags and detail the driver behavior
 *    if enable/disable such switch or adjust numeric parameters.
 */

#ifndef _PRECOMP_H
#define _PRECOMP_H

/*******************************************************************************
 *                         C O M P I L E R   F L A G S
 *******************************************************************************
 */

#ifdef __GNUC__
#if (DBG == 0)
#pragma GCC diagnostic ignored "-Wformat"
#endif
#endif

/*******************************************************************************
 *                    E X T E R N A L   R E F E R E N C E S
 *******************************************************************************
 */
#include "hif_cmm.h"
#include "gl_os.h"		/* Include "config.h" */

#if CFG_ENABLE_WIFI_DIRECT
#include "gl_p2p_os.h"
#endif

#include "debug.h"

#include "link.h"
#include "queue.h"

/*------------------------------------------------------------------------------
 * .\include\mgmt
 *------------------------------------------------------------------------------
 */
#include "wlan_typedef.h"

#include "mac.h"

/* Dependency:  mac.h (MAC_ADDR_LEN) */
#include "wlan_def.h"

#if CFG_SUPPORT_SWCR
#include "swcr.h"
#endif

/*------------------------------------------------------------------------------
 * .\include\nic
 *------------------------------------------------------------------------------
 */
/* Dependency:  wlan_def.h (ENUM_NETWORK_TYPE_T) */
#include "cmd_buf.h"

/* Dependency:  mac.h (MAC_ADDR_LEN) */
#include "nic_cmd_event.h"

#include "rlm_obss.h"

/* Dependency:  rlm_obss.h (OBSS_SCAN_MIN_INTERVAL) */
#include "cnm_timer.h"

/* Dependency:  nic_cmd_event.h (P_EVENT_CONNECTION_STATUS)
 *				cnm_timer.h (TIMER_T)
 */
#include "nic.h"

#include "nic_init_cmd_event.h"

#include "hif_rx.h"
#include "hif_tx.h"

#include "nic_tx.h"

/* Dependency:  hif_rx.h (P_HIF_RX_HEADER_T) */
#include "nic_rx.h"

#include "nic_umac.h"

#include "bss.h"

#include "nic_rate.h"

#if CFG_ENABLE_WIFI_DIRECT
#include "p2p_typedef.h"
#include "p2p_cmd_buf.h"
#include "p2p_nic_cmd_event.h"
#include "p2p_mac.h"
#include "p2p_nic.h"
#endif

/*------------------------------------------------------------------------------
 * .\include\mgmt
 *------------------------------------------------------------------------------
 */

#include "hem_mbox.h"

#include "scan.h"

#include "wlan_lib.h"
#include "wlan_oid.h"
#include "wlan_bow.h"

#include "fw_dl.h"

#if CFG_ENABLE_WIFI_DIRECT
#include "wlan_p2p.h"
#endif

#include "hal.h"

#include "mt66xx_reg.h"

#include "connac_reg.h"
#include "connac_dmashdl.h"
#include "cmm_asic_connac.h"

#include "rlm.h"
#include "rlm_domain.h"
#include "rlm_protection.h"
#include "rate.h"
#include "wnm.h"

#include "qosmap.h"

#include "aa_fsm.h"

#include "que_mgt.h"

#include "wmm.h"
#if CFG_ENABLE_BT_OVER_WIFI
#include "bow.h"
#include "bow_fsm.h"
#endif

#include "pwr_mgt.h"

#if (CFG_SUPPORT_STATISTICS == 1)
#include "stats.h"
#endif /* CFG_SUPPORT_STATISTICS */

#include "cnm.h"
/* Dependency:  aa_fsm.h (ENUM_AA_STATE_T), p2p_fsm.h
 *   (WPS_ATTRI_MAX_LEN_DEVICE_NAME)
 */
#include "cnm_mem.h"
#include "cnm_scan.h"

#if CFG_ENABLE_WIFI_DIRECT
#include "p2p_rlm_obss.h"
#include "p2p_bss.h"
#include "p2p.h"

#include "p2p_rlm.h"
#include "p2p_assoc.h"
#include "p2p_ie.h"
#include "p2p_role.h"

#include "p2p_func.h"
#include "p2p_scan.h"
#include "p2p_dev.h"
#include "p2p_fsm.h"
#endif

#include "privacy.h"

#include "mib.h"

#include "auth.h"
#include "assoc.h"

#if CFG_SUPPORT_ROAMING
#include "roaming_fsm.h"
#endif /* CFG_SUPPORT_ROAMING */

#include "ais_fsm.h"

#include "adapter.h"

#include "que_mgt.h"
#include "rftest.h"

#include "rsn.h"

#if CFG_SUPPORT_WAPI
#include "wapi.h"
#endif

/* Support AP Selection */
#include "ap_selection.h"

/*------------------------------------------------------------------------------
 * NVRAM structure
 *------------------------------------------------------------------------------
 */
#include "CFG_Wifi_File.h"

#if CFG_ENABLE_WIFI_DIRECT
#include "gl_p2p_kal.h"
#endif

#if CFG_SUPPORT_TDLS
#include "tdls.h"
#endif

#if CFG_SUPPORT_QA_TOOL
#include "gl_qa_agent.h"
#include "gl_ate_agent.h"
#endif
/*------------------------------------------------------------------------------
 * Memory Prealloc
 *------------------------------------------------------------------------------
 */
#ifdef CFG_PREALLOC_MEMORY
#include "prealloc.h"
#endif
/*------------------------------------------------------------------------------
 * Reset ko
 *------------------------------------------------------------------------------
 */
#if CFG_CHIP_RESET_KO_SUPPORT
#include "reset.h"
#endif

/*******************************************************************************
 *                              C O N S T A N T S
 *******************************************************************************
 */

/*******************************************************************************
 *                             D A T A   T Y P E S
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

#endif /* _PRECOMP_H */
