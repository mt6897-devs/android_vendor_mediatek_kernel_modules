// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

/*
 ** Id: @(#) gl_rst.c@@
 */

/*! \file   gl_rst.c
 *    \brief  Main routines for supporintg MT6620 whole-chip reset mechanism
 *
 *    This file contains the support routines of Linux driver for MediaTek Inc.
 *    802.11 Wireless LAN Adapters.
 */


/*******************************************************************************
 *                         C O M P I L E R   F L A G S
 *******************************************************************************
 */

/*******************************************************************************
 *                    E X T E R N A L   R E F E R E N C E S
 *******************************************************************************
 */
#include <linux/kernel.h>
#include <linux/workqueue.h>

#include "precomp.h"
#include "gl_rst.h"

/*******************************************************************************
 *                              C O N S T A N T S
 *******************************************************************************
 */



/*******************************************************************************
 *                            P U B L I C   D A T A
 *******************************************************************************
 */
#if CFG_CHIP_RESET_SUPPORT
enum _ENUM_CHIP_RESET_REASON_TYPE_T eResetReason;
uint64_t u8ResetTime;

#if CFG_CHIP_RESET_HANG
u_int8_t fgIsResetHangState = SER_L0_HANG_RST_NONE;
#endif

#endif
static uint32_t u4ProbeCount;

/*******************************************************************************
 *                           P R I V A T E   D A T A
 *******************************************************************************
 */
#if CFG_CHIP_RESET_SUPPORT
static struct RESET_STRUCT wifi_rst;
u_int8_t fgIsResetting = FALSE;
static wlanRemoveFunc pfWlanRemove;
#if CFG_CHIP_RESET_KO_SUPPORT
static uint32_t u4RstCount;
static uint32_t u4PowerOffCount;
static u_int8_t fgIsPendingForReady = FALSE;
#endif
#endif

/*******************************************************************************
 *                   F U N C T I O N   D E C L A R A T I O N S
 *******************************************************************************
 */
#if CFG_CHIP_RESET_SUPPORT
static void mtk_wifi_reset(struct work_struct *work);

#if CFG_WMT_RESET_API_SUPPORT
static void mtk_wifi_trigger_reset(struct work_struct *work);
static void *glResetCallback(enum ENUM_WMTDRV_TYPE eSrcType,
			     enum ENUM_WMTDRV_TYPE eDstType,
			     enum ENUM_WMTMSG_TYPE eMsgType, void *prMsgBody,
			     unsigned int u4MsgLength);
#else
static u_int8_t is_bt_exist(void);
static u_int8_t rst_L0_notify_step1(void);
static u_int8_t rst_L0_notify_step2(void);
static void wait_core_dump_end(void);
#endif
#endif

/*******************************************************************************
 *                              F U N C T I O N S
 *******************************************************************************
 */
#if CFG_CHIP_RESET_SUPPORT

/*----------------------------------------------------------------------------*/
/*!
 * @brief This routine is responsible for
 *        1. register wifi reset callback
 *        2. initialize wifi reset work
 *
 * @param none
 *
 * @retval none
 */
/*----------------------------------------------------------------------------*/
void glResetInit(struct GLUE_INFO *prGlueInfo, wlanRemoveFunc pfRemove)
{
#if CFG_WMT_RESET_API_SUPPORT
	/* 1. Register reset callback */
	mtk_wcn_wmt_msgcb_reg(WMTDRV_TYPE_WIFI,
			      (PF_WMT_CB) glResetCallback);

	/* 2. Initialize reset work */
	INIT_WORK(&(wifi_rst.rst_trigger_work),
		  mtk_wifi_trigger_reset);
#endif
#if CFG_CHIP_RESET_KO_SUPPORT
	u4RstCount = 0;
	u4PowerOffCount = 0;
	fgIsPendingForReady = FALSE;
#endif
	pfWlanRemove = pfRemove;
	u4ProbeCount = 0;
	fgIsResetting = FALSE;
	wifi_rst.prGlueInfo = prGlueInfo;
	INIT_WORK(&(wifi_rst.rst_work), mtk_wifi_reset);
}

void glReseProbeRemoveDone(struct GLUE_INFO *prGlueInfo, int32_t i4Status,
			   u_int8_t fgIsProbe)
{
	if (!prGlueInfo)
		return;
#if CFG_CHIP_RESET_KO_SUPPORT
	if (fgIsProbe) {
		if (i4Status == WLAN_STATUS_SUCCESS) {
			send_reset_event(RESET_MODULE_TYPE_WIFI,
					 RFSM_EVENT_PROBED);
#if defined(_HIF_SDIO)
			update_hif_info(HIF_INFO_SDIO_HOST,
					prGlueInfo->rHifInfo.func);
#endif
		}
	} else {
		send_reset_event(RESET_MODULE_TYPE_WIFI, RFSM_EVENT_REMOVED);
	}

	if (fgIsPendingForReady) {
		fgIsPendingForReady = FALSE;
		send_reset_event(RESET_MODULE_TYPE_WIFI, RFSM_EVENT_READY);
	}
#endif

	if (fgIsProbe) {
		u4ProbeCount++;
		DBGLOG(INIT, WARN,
			"[SER][L0] %s: probe count %d, status %d\n",
			__func__, u4ProbeCount, i4Status);
	}
}

/*----------------------------------------------------------------------------*/
/*!
 * @brief This routine is responsible for
 *        1. deregister wifi reset callback
 *
 * @param none
 *
 * @retval none
 */
/*----------------------------------------------------------------------------*/
void glResetUninit(void)
{
#if CFG_WMT_RESET_API_SUPPORT
	/* 1. Deregister reset callback */
	mtk_wcn_wmt_msgcb_unreg(WMTDRV_TYPE_WIFI);
#endif
}
/*----------------------------------------------------------------------------*/
/*!
 * @brief This routine is called for generating reset request to WMT
 *
 * @param   None
 *
 * @retval  None
 */
/*----------------------------------------------------------------------------*/
void glSendResetRequest(void)
{
#if CFG_WMT_RESET_API_SUPPORT

	/* WMT thread would trigger whole chip reset itself */
#endif
}

/*----------------------------------------------------------------------------*/
/*!
 * @brief This routine is called for checking if connectivity chip is resetting
 *
 * @param   None
 *
 * @retval  TRUE
 *          FALSE
 */
/*----------------------------------------------------------------------------*/
u_int8_t kalIsResetting(void)
{
	return fgIsResetting;
}


u_int8_t glResetTrigger(struct ADAPTER *prAdapter,
		uint32_t u4RstFlag, const uint8_t *pucFile, uint32_t u4Line)
{
	u_int8_t fgResult = TRUE;
	uint16_t u2FwOwnVersion;
	uint16_t u2FwPeerVersion;

	if (kalIsResetting())
		return fgResult;

	dump_stack();

	fgIsResetting = TRUE;
	if (eResetReason != RST_BT_TRIGGER)
		DBGLOG(INIT, STATE, "[SER][L0] wifi trigger eResetReason=%d\n",
								eResetReason);
	else
		DBGLOG(INIT, STATE, "[SER][L0] BT trigger\n");

#if CFG_WMT_RESET_API_SUPPORT
	if (u4RstFlag & RST_FLAG_DO_CORE_DUMP)
		if (glIsWmtCodeDump())
			DBGLOG(INIT, WARN, "WMT is code dumping !\n");
#endif
	if (prAdapter == NULL) {
		DBGLOG(INIT, ERROR,
			"[SER][L0] Adapter is null, skip dump\n");
#if CFG_WMT_RESET_API_SUPPORT
		wifi_rst.rst_trigger_flag = u4RstFlag;
		schedule_work(&(wifi_rst.rst_trigger_work));
#else
#if CFG_CHIP_RESET_KO_SUPPORT
		send_reset_event(RESET_MODULE_TYPE_WIFI,
				 RFSM_EVENT_TRIGGER_RESET);
#else
		schedule_work(&(wifi_rst.rst_work));
#endif
#endif
		return fgResult;
	}
	u2FwOwnVersion = prAdapter->rVerInfo.u2FwOwnVersion;
	u2FwPeerVersion = prAdapter->rVerInfo.u2FwPeerVersion;

	DBGLOG(INIT, ERROR,
		"Trigger chip reset in %s line %u! Chip[%04X E%u] FW Ver DEC[%u.%u] HEX[%x.%x], Driver Ver[%u.%u]\n",
		 pucFile, u4Line, MTK_CHIP_REV,
	wlanGetEcoVersion(prAdapter),
		(uint16_t)(u2FwOwnVersion >> 8),
		(uint16_t)(u2FwOwnVersion & BITS(0, 7)),
		(uint16_t)(u2FwOwnVersion >> 8),
		(uint16_t)(u2FwOwnVersion & BITS(0, 7)),
		(uint16_t)(u2FwPeerVersion >> 8),
		(uint16_t)(u2FwPeerVersion & BITS(0, 7)));

	prAdapter->u4HifDbgFlag |= DEG_HIF_DEFAULT_DUMP;
	halPrintHifDbgInfo(prAdapter);

#if CFG_WMT_RESET_API_SUPPORT
	wifi_rst.rst_trigger_flag = u4RstFlag;
	schedule_work(&(wifi_rst.rst_trigger_work));
#else
#if CFG_CHIP_RESET_KO_SUPPORT
	send_reset_event(RESET_MODULE_TYPE_WIFI,
			 RFSM_EVENT_TRIGGER_RESET);
#else
	wifi_rst.prGlueInfo = prAdapter->prGlueInfo;
	schedule_work(&(wifi_rst.rst_work));
#endif
#endif

	return fgResult;
}

void glGetRstReason(enum _ENUM_CHIP_RESET_REASON_TYPE_T
		    eReason)
{
	if (kalIsResetting())
		return;

	u8ResetTime = sched_clock();
	eResetReason = eReason;
}


/*----------------------------------------------------------------------------*/
/*!
 * @brief This routine is called for wifi reset
 *
 * @param   skb
 *          info
 *
 * @retval  0
 *          nonzero
 */
/*----------------------------------------------------------------------------*/
static void mtk_wifi_reset(struct work_struct *work)
{
	struct RESET_STRUCT *rst = container_of(work,
						struct RESET_STRUCT, rst_work);
	u_int8_t fgResult = FALSE;

#if CFG_WMT_RESET_API_SUPPORT
	wifi_reset_end(rst->rst_data);
#else
	fgResult = rst_L0_notify_step1();

	wait_core_dump_end();

	fgResult = rst_L0_notify_step2();

#if CFG_CHIP_RESET_HANG
	if (fgIsResetHangState == SER_L0_HANG_RST_NONE)
		fgIsResetHangState = SER_L0_HANG_RST_TRGING;
#endif

	if (is_bt_exist() == FALSE)
		kalRemoveProbe(rst->prGlueInfo);

#endif

	DBGLOG(INIT, STATE, "[SER][L0] flow end, fgResult=%d\n", fgResult);

}

#if CFG_CHIP_RESET_KO_SUPPORT
void resetkoNotifyFunc(unsigned int event, void *data)
{
	DBGLOG(INIT, INFO, "%s: %d\n", __func__, event);
	if (event == MODULE_NOTIFY_PRE_POWER_OFF) {
		fgIsResetting = TRUE;
		if ((wlanIsProbing() == FALSE) && (wlanIsRemoving() == FALSE))
			send_reset_event(RESET_MODULE_TYPE_WIFI,
					 RFSM_EVENT_READY);
		else
			fgIsPendingForReady = TRUE;
	} else if (event == MODULE_NOTIFY_RESET_DONE) {
		u4RstCount++;
		DBGLOG(INIT, INFO, "%s: reset count %d\n",
			__func__, u4RstCount);
	} else if (event == MODULE_NOTIFY_POWER_OFF_DONE) {
		u4PowerOffCount++;
		DBGLOG(INIT, INFO, "%s: power off count %d\n",
			__func__, u4PowerOffCount);
	}
}
#if (RESETKO_API_VERSION == 1)
void resetkoReset(void)
{
	DBGLOG(INIT, WARN, "%s\n", __func__);
	kalRemoveProbe(wifi_rst.prGlueInfo);
}
#endif
#endif

#if CFG_WMT_RESET_API_SUPPORT

static void mtk_wifi_trigger_reset(struct work_struct *work)
{
	u_int8_t fgResult = FALSE;
	struct RESET_STRUCT *rst = container_of(work,
					struct RESET_STRUCT, rst_trigger_work);

	fgIsResetting = TRUE;
	/* Set the power off flag to FALSE in WMT to prevent chip power off
	 * after wlanProbe return failure, because we need to do core dump
	 * afterward.
	 */
	if (rst->rst_trigger_flag & RST_FLAG_PREVENT_POWER_OFF)
		mtk_wcn_set_connsys_power_off_flag(FALSE);

	fgResult = mtk_wcn_wmt_assert_timeout(WMTDRV_TYPE_WIFI, 0x40, 0);
	DBGLOG(INIT, INFO, "reset result %d, trigger flag 0x%x\n",
				fgResult, rst->rst_trigger_flag);
}

/* Weak reference for those platform doesn't support wmt functions */
u_int8_t __weak mtk_wcn_stp_coredump_start_get(void)
{
	return FALSE;
}


/*0= f/w assert flag is not set, others=f/w assert flag is set */
u_int8_t glIsWmtCodeDump(void)
{
	return mtk_wcn_stp_coredump_start_get();
}

/*----------------------------------------------------------------------------*/
/*!
 * @brief This routine is invoked when there is reset messages indicated
 *
 * @param   eSrcType
 *          eDstType
 *          eMsgType
 *          prMsgBody
 *          u4MsgLength
 *
 * @retval
 */
/*----------------------------------------------------------------------------*/
static void *glResetCallback(enum ENUM_WMTDRV_TYPE eSrcType,
			     enum ENUM_WMTDRV_TYPE eDstType,
			     enum ENUM_WMTMSG_TYPE eMsgType, void *prMsgBody,
			     unsigned int u4MsgLength)
{
	switch (eMsgType) {
	case WMTMSG_TYPE_RESET:
		if (u4MsgLength == sizeof(enum ENUM_WMTRSTMSG_TYPE)) {
			enum ENUM_WMTRSTMSG_TYPE *prRstMsg =
					(enum ENUM_WMTRSTMSG_TYPE *) prMsgBody;

			switch (*prRstMsg) {
			case WMTRSTMSG_RESET_START:
				DBGLOG(INIT, WARN, "Whole chip reset start!\n");
				wifi_reset_start();
				break;

			case WMTRSTMSG_RESET_END:
				DBGLOG(INIT, WARN, "Whole chip reset end!\n");
				fgIsResetting = FALSE;
				wifi_rst.rst_data = RESET_SUCCESS;
				schedule_work(&(wifi_rst.rst_work));
				break;

			case WMTRSTMSG_RESET_END_FAIL:
				DBGLOG(INIT, WARN, "Whole chip reset fail!\n");
				fgIsResetting = FALSE;
				wifi_rst.rst_data = RESET_FAIL;
				schedule_work(&(wifi_rst.rst_work));
				break;

			default:
				break;
			}
		}
		break;

	default:
		break;
	}

	return NULL;
}
#else
static u_int8_t is_bt_exist(void)
{
	typedef int (*p_bt_fun_type) (int);
	p_bt_fun_type bt_func;
	char *bt_func_name = "WF_rst_L0_notify_BT_step1";

	bt_func = (p_bt_fun_type) kal_kallsyms_lookup_name(bt_func_name);
	if (bt_func) {
		kal_kallsyms_put(bt_func_name);
		return TRUE;
	}
	DBGLOG(INIT, ERROR, "[SER][L0] %s does not exist\n", bt_func_name);
	return FALSE;
}

static u_int8_t rst_L0_notify_step1(void)
{
	if (eResetReason != RST_BT_TRIGGER) {
		typedef int (*p_bt_fun_type) (int);
		p_bt_fun_type bt_func;
		char *bt_func_name = "WF_rst_L0_notify_BT_step1";

		DBGLOG(INIT, STATE, "[SER][L0] %s\n", bt_func_name);
		bt_func =
			(p_bt_fun_type) kal_kallsyms_lookup_name(bt_func_name);
		if (bt_func) {
			bt_func(0);
			kal_kallsyms_put(bt_func_name);
		} else {
			DBGLOG(INIT, ERROR,
				"[SER][L0] %s does not exist\n", bt_func_name);
			return FALSE;
		}
	}

	return TRUE;
}

static u_int8_t rst_L0_notify_step2(void)
{

#ifdef CFG_SUPPORT_CONNAC2X
	if (eResetReason == RST_BT_TRIGGER) {
		typedef void (*p_bt_fun_type) (void);
		p_bt_fun_type bt_func;
		char *bt_func_name = "WF_rst_L0_notify_BT_step2";

		bt_func =
			(p_bt_fun_type) kal_kallsyms_lookup_name(bt_func_name);
		if (bt_func) {
			bt_func();
			kal_kallsyms_put(bt_func_name);
		} else {
			DBGLOG(INIT, WARN, "[SER][L0] %s does not exist\n",
							bt_func_name);
			return FALSE;
		}
	} else {
		/* if wifi trigger, then wait bt ready notify */
		DBGLOG(INIT, WARN, "[SER][L0] not support..\n");
	}
#else
	typedef int (*p_bt_fun_type) (void);
	p_bt_fun_type bt_func;
	char *bt_func_name = "WF_rst_L0_notify_BT_step2";

	DBGLOG(INIT, STATE, "[SER][L0] %s\n", bt_func_name);
	bt_func =
		(p_bt_fun_type) kal_kallsyms_lookup_name(bt_func_name);
	if (bt_func) {
		bt_func();
		kal_kallsyms_put(bt_func_name);
	} else {
		DBGLOG(INIT, WARN, "[SER][L0] %s does not exist\n",
							bt_func_name);
		return FALSE;
	}
#endif
	return TRUE;
}

static void wait_core_dump_end(void)
{
#ifdef CFG_SUPPORT_CONNAC2X
	if (eResetReason == RST_OID_TIMEOUT)
		return;
	DBGLOG(INIT, WARN, "[SER][L0] not support..\n");
#endif
}

int32_t BT_rst_L0_notify_WF_step1(int32_t reserved)
{
	glGetRstReason(RST_BT_TRIGGER);
	GL_RESET_TRIGGER(NULL, RST_FLAG_CHIP_RESET);

	return 0;
}
EXPORT_SYMBOL(BT_rst_L0_notify_WF_step1);

int32_t BT_rst_L0_notify_WF_step2(int32_t reserved)
{
	DBGLOG(INIT, WARN, "[SER][L0] not support...\n");

	return 0;
}
EXPORT_SYMBOL(BT_rst_L0_notify_WF_step2);

#endif


#else

u_int8_t kalIsResetting(void)
{
	return FALSE;
}

#endif

