/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

/*! \file   "cnm.h"
 *    \brief
 */


#ifndef _CNM_H
#define _CNM_H

/*******************************************************************************
 *                         C O M P I L E R   F L A G S
 *******************************************************************************
 */

/*******************************************************************************
 *                    E X T E R N A L   R E F E R E N C E S
 *******************************************************************************
 */
#include "nic_cmd_event.h"

/*******************************************************************************
 *                              C O N S T A N T S
 *******************************************************************************
 */
#define DBDC_5G_WMM_INDEX	0
#define DBDC_2G_WMM_INDEX	1
#define HW_WMM_NUM		(prAdapter->ucWmmSetNum)
#define MAX_HW_WMM_INDEX	(HW_WMM_NUM - 1)
#define DEFAULT_HW_WMM_INDEX	MAX_HW_WMM_INDEX
/*******************************************************************************
 *                             D A T A   T Y P E S
 *******************************************************************************
 */

#if (CFG_SUPPORT_IDC_CH_SWITCH == 1)
enum ENUM_CH_SWITCH_TYPE {
	CH_SWITCH_2G, /* Default */
	CH_SWITCH_5G,
	CH_SWITCH_6G,
	CH_SWITCH_NUM
};
#endif

struct MSG_CH_REQ {
	struct MSG_HDR rMsgHdr;	/* Must be the first member */
	uint8_t ucBssIndex;
	uint8_t ucTokenID;
	uint8_t ucPrimaryChannel;
	enum ENUM_CHNL_EXT eRfSco;
	enum ENUM_BAND eRfBand;

	/* To support 80/160MHz bandwidth */
	enum ENUM_CHANNEL_WIDTH eRfChannelWidth;

	uint8_t ucRfCenterFreqSeg1;	/* To support 80/160MHz bandwidth */
	uint8_t ucRfCenterFreqSeg2;	/* To support 80/160MHz bandwidth */
	enum ENUM_CH_REQ_TYPE eReqType;
	uint32_t u4MaxInterval;	/* In unit of ms */
	enum ENUM_MBMC_BN eDBDCBand;
	uint8_t ucExtraChReqNum;
	uint8_t aucBuffer[0];
};

struct MSG_CH_ABORT {
	struct MSG_HDR rMsgHdr;	/* Must be the first member */
	uint8_t ucBssIndex;
	uint8_t ucTokenID;
	enum ENUM_MBMC_BN eDBDCBand;
	uint8_t ucExtraChReqNum;
};

struct MSG_CH_GRANT {
	struct MSG_HDR rMsgHdr;	/* Must be the first member */
	uint8_t ucBssIndex;
	uint8_t ucTokenID;
	uint8_t ucPrimaryChannel;
	enum ENUM_CHNL_EXT eRfSco;
	enum ENUM_BAND eRfBand;

	/* To support 80/160MHz bandwidth */
	enum ENUM_CHANNEL_WIDTH eRfChannelWidth;

	uint8_t ucRfCenterFreqSeg1;	/* To support 80/160MHz bandwidth */
	uint8_t ucRfCenterFreqSeg2;	/* To support 80/160MHz bandwidth */
	enum ENUM_CH_REQ_TYPE eReqType;
	uint32_t u4GrantInterval;	/* In unit of ms */
	enum ENUM_MBMC_BN eDBDCBand;
};

struct MSG_CH_REOCVER {
	struct MSG_HDR rMsgHdr;	/* Must be the first member */
	uint8_t ucBssIndex;
	uint8_t ucTokenID;
	uint8_t ucPrimaryChannel;
	enum ENUM_CHNL_EXT eRfSco;
	enum ENUM_BAND eRfBand;

	/*  To support 80/160MHz bandwidth */
	enum ENUM_CHANNEL_WIDTH eRfChannelWidth;

	uint8_t ucRfCenterFreqSeg1;	/* To support 80/160MHz bandwidth */
	uint8_t ucRfCenterFreqSeg2;	/* To support 80/160MHz bandwidth */
	enum ENUM_CH_REQ_TYPE eReqType;
};

struct CNM_INFO {
	u_int8_t fgChGranted;
	uint8_t ucBssIndex;
	uint8_t ucTokenID;
};

#if CFG_ENABLE_WIFI_DIRECT
/* Moved from p2p_fsm.h */
__KAL_ATTRIB_PACKED_FRONT__
struct DEVICE_TYPE {
	uint16_t u2CategoryId;		/* Category ID */
	uint8_t aucOui[4];		/* OUI */
	uint16_t u2SubCategoryId;	/* Sub Category ID */
} __KAL_ATTRIB_PACKED__;
#endif

enum ENUM_CNM_DBDC_MODE {
	/* A/G traffic separate by WMM, but both
	 * TRX on band 0, CANNOT enable DBDC
	 */
	ENUM_DBDC_MODE_DISABLED,

	/* A/G traffic separate by WMM, WMM0/1
	 * TRX on band 0/1, CANNOT disable DBDC
	 */
	ENUM_DBDC_MODE_STATIC,

	/* Automatically enable/disable DBDC,
	 * setting just like static/disable mode
	 */
	ENUM_DBDC_MODE_DYNAMIC,

	ENUM_DBDC_MODE_NUM
};

#if CFG_SUPPORT_DBDC
enum ENUM_CNM_DBDC_SWITCH_MECHANISM { /* When DBDC available in dynamic DBDC */
	/* Switch to DBDC when available (less latency) */
	ENUM_DBDC_SWITCH_MECHANISM_LATENCY_MODE,

	/* Switch to DBDC when DBDC T-put > MCC T-put */
	ENUM_DBDC_SWITCH_MECHANISM_THROUGHPUT_MODE,

	ENUM_DBDC_SWITCH_MECHANISM_NUM
};
#endif	/* CFG_SUPPORT_DBDC */

enum ENUM_CNM_NETWORK_TYPE_T {
	ENUM_CNM_NETWORK_TYPE_OTHER,
	ENUM_CNM_NETWORK_TYPE_AIS,
	ENUM_CNM_NETWORK_TYPE_P2P_GC,
	ENUM_CNM_NETWORK_TYPE_P2P_GO,
	ENUM_CNM_NETWORK_TYPE_NUM
};

/* Priority Order !!!! */
enum ENUM_CNM_OPMODE_REQ_T {
	CNM_OPMODE_REQ_START      = 0,
	CNM_OPMODE_REQ_ANT_CTRL   = 0,
	CNM_OPMODE_REQ_DBDC       = 1,
	CNM_OPMODE_REQ_DBDC_SCAN  = 2,
	CNM_OPMODE_REQ_COEX       = 3,
	CNM_OPMODE_REQ_SMARTGEAR  = 4,
	CNM_OPMODE_REQ_USER_CONFIG     = 5,
	CNM_OPMODE_REQ_SMARTGEAR_1T2R  = 6,
	CNM_OPMODE_REQ_ANT_CTRL_1T2R   = 7,
	CNM_OPMODE_REQ_COANT      = 8,
	CNM_OPMODE_REQ_RDD_OPCHNG = 9,
	CNM_OPMODE_REQ_NUM        = 10,
	CNM_OPMODE_REQ_MAX_CAP    = 11 /* just for coding */
};

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
/* True if our TxNss > 1 && peer support 2ss rate && peer no Rx limit. */
#if (CFG_SUPPORT_WIFI_6G == 1)
#define IS_CONNECTION_NSS2(prBssInfo, prStaRec) \
	((((prBssInfo)->ucOpTxNss > 1) && \
	((prStaRec)->aucRxMcsBitmask[1] != 0x00) \
	&& (((prStaRec)->u2HtCapInfo & HT_CAP_INFO_SM_POWER_SAVE) != 0)) || \
	(((prBssInfo)->ucOpTxNss > 1) && ((((prStaRec)->u2VhtRxMcsMap \
	& BITS(2, 3)) >> 2) != BITS(0, 1)) && ((((prStaRec)->ucVhtOpMode \
	& VHT_OP_MODE_RX_NSS) >> VHT_OP_MODE_RX_NSS_OFFSET) > 0)) || \
	(((prBssInfo)->ucOpTxNss > 1) \
	&& ((prBssInfo)->eBand == BAND_6G) \
	&& ((((prStaRec)->u2HeRxMcsMapBW80 & BITS(2, 3)) >> 2) != BITS(0, 1)) \
	&& (((prStaRec)->u2He6gBandCapInfo \
	& HE_6G_CAP_INFO_SM_POWER_SAVE) != 0)))

#else
#define IS_CONNECTION_NSS2(prBssInfo, prStaRec) \
	((((prBssInfo)->ucOpTxNss > 1) && \
	((prStaRec)->aucRxMcsBitmask[1] != 0x00) \
	&& (((prStaRec)->u2HtCapInfo & HT_CAP_INFO_SM_POWER_SAVE) != 0)) || \
	(((prBssInfo)->ucOpTxNss > 1) && ((((prStaRec)->u2VhtRxMcsMap \
	& BITS(2, 3)) >> 2) != BITS(0, 1)) && ((((prStaRec)->ucVhtOpMode \
	& VHT_OP_MODE_RX_NSS) >> VHT_OP_MODE_RX_NSS_OFFSET) > 0)))
#endif

#define CNM_DBDC_ADD_DECISION_INFO(rDeciInfo, \
		_ucBId, _eBand, _ucCh, _ucWmmQId) {\
	rDeciInfo.dbdcElem[rDeciInfo.ucLinkNum].ucBssIndex = _ucBId;\
	rDeciInfo.dbdcElem[rDeciInfo.ucLinkNum].eRfBand = _eBand;\
	rDeciInfo.dbdcElem[rDeciInfo.ucLinkNum].ucPrimaryChannel = _ucCh;\
	rDeciInfo.dbdcElem[rDeciInfo.ucLinkNum].ucWmmQueIndex = _ucWmmQId;\
	rDeciInfo.ucLinkNum++;\
}

/*******************************************************************************
 *                   F U N C T I O N   D E C L A R A T I O N S
 *******************************************************************************
 */
void cnmInit(struct ADAPTER *prAdapter);

void cnmUninit(struct ADAPTER *prAdapter);

void cnmChMngrRequestPrivilege(struct ADAPTER *prAdapter,
	struct MSG_HDR *prMsgHdr);

void cnmChMngrAbortPrivilege(struct ADAPTER *prAdapter,
	struct MSG_HDR *prMsgHdr);

void cnmChMngrHandleChEvent(struct ADAPTER *prAdapter,
	struct WIFI_EVENT *prEvent);

#if (CFG_SUPPORT_DFS_MASTER == 1)
void cnmRadarDetectEvent(struct ADAPTER *prAdapter,
	struct WIFI_EVENT *prEvent);
#endif

#if (CFG_SUPPORT_IDC_CH_SWITCH == 1)
uint8_t cnmIdcCsaReq(struct ADAPTER *prAdapter,
	enum ENUM_BAND eBand,
	uint8_t ch_num, uint8_t ucRoleIdx);

void cnmIdcSwitchSapChannel(struct ADAPTER *prAdapter);

void cnmIdcDetectHandler(struct ADAPTER *prAdapter,
	struct WIFI_EVENT *prEvent);
#endif

#if CFG_ENABLE_WIFI_DIRECT
void cnmCsaDoneEvent(struct ADAPTER *prAdapter,
	struct WIFI_EVENT *prEvent);

void cnmCsaResetParams(struct ADAPTER *prAdapter,
	struct BSS_INFO *prBssInfo);

uint8_t cnmOwnGcCsaReq(struct ADAPTER *prAdapter,
	enum ENUM_BAND eBand,
	uint8_t ucCh,
	uint8_t ucRoleIdx);

void cnmPeerGcCsaHandler(struct ADAPTER *prAdapter,
	struct WIFI_EVENT *prEvent);

void cnmOwnGcCsaHandler(struct ADAPTER *prAdapter,
		struct MSG_HDR *prMsgHdr);

uint8_t cnmSapChannelSwitchReq(struct ADAPTER *prAdapter,
	struct RF_CHANNEL_INFO *prRfChannelInfo,
	uint8_t ucRoleIdx);
#endif

u_int8_t cnmPreferredChannel(struct ADAPTER *prAdapter, enum ENUM_BAND *prBand,
	uint8_t *pucPrimaryChannel, enum ENUM_CHNL_EXT *prBssSCO);

u_int8_t cnmAisInfraChannelFixed(struct ADAPTER *prAdapter,
	enum ENUM_BAND *prBand, uint8_t *pucPrimaryChannel);

void cnmAisInfraConnectNotify(struct ADAPTER *prAdapter);

u_int8_t cnmAisIbssIsPermitted(struct ADAPTER *prAdapter);

u_int8_t cnmP2PIsPermitted(struct ADAPTER *prAdapter);

u_int8_t cnmBowIsPermitted(struct ADAPTER *prAdapter);

u_int8_t cnmBss40mBwPermitted(struct ADAPTER *prAdapter, uint8_t ucBssIndex);

u_int8_t cnmBss80mBwPermitted(struct ADAPTER *prAdapter, uint8_t ucBssIndex);

uint8_t cnmGetBssMaxBw(struct ADAPTER *prAdapter, uint8_t ucBssIndex);

uint8_t cnmGetBssMaxBwToChnlBW(struct ADAPTER *prAdapter, uint8_t ucBssIndex);

struct BSS_INFO *cnmGetBssInfoAndInit(struct ADAPTER *prAdapter,
	enum ENUM_NETWORK_TYPE eNetworkType, u_int8_t fgIsP2pDevice);

void cnmFreeBssInfo(struct ADAPTER *prAdapter, struct BSS_INFO *prBssInfo);

#if CFG_SUPPORT_CHNL_CONFLICT_REVISE
u_int8_t cnmAisDetectP2PChannel(struct ADAPTER *prAdapter,
	enum ENUM_BAND *prBand, uint8_t *pucPrimaryChannel);
#endif

void cnmWmmIndexDecision(struct ADAPTER *prAdapter,
	struct BSS_INFO *prBssInfo);
void cnmFreeWmmIndex(struct ADAPTER *prAdapter,
	struct BSS_INFO *prBssInfo);

uint8_t cnmGetDbdcBwCapability(
	struct ADAPTER *prAdapter,
	uint8_t ucBssIndex
);

#if CFG_SUPPORT_DBDC
void cnmInitDbdcSetting(struct ADAPTER *prAdapter);

uint32_t cnmUpdateDbdcSetting(
	struct ADAPTER *prAdapter,
	u_int8_t fgDbdcEn);

bool cnmDbdcIsDisabled(struct ADAPTER *prAdapter);

void cnmDbdcPreConnectionEnableDecision(
	struct ADAPTER *prAdapter,
	struct DBDC_DECISION_INFO *prDbdcDecisionInfo);

void cnmDbdcRuntimeCheckDecision(struct ADAPTER *prAdapter,
	uint8_t ucChangedBssIndex,
	u_int8_t ucForceLeaveEnGuard);

#if (CFG_DBDC_SW_FOR_P2P_LISTEN == 1)
u_int8_t cnmDbdcIsP2pListenDbdcEn(void);
#endif

void cnmDbdcGuardTimerCallback(struct ADAPTER *prAdapter,
	uintptr_t plParamPtr);
void cnmDbdcEventHwSwitchDone(struct ADAPTER *prAdapter,
	struct WIFI_EVENT *prEvent);
u_int8_t cnmDBDCIsReqPeivilegeLock(void);
#endif /*CFG_SUPPORT_DBDC*/

enum ENUM_CNM_NETWORK_TYPE_T cnmGetBssNetworkType(struct BSS_INFO *prBssInfo);

#if CFG_ENABLE_WIFI_DIRECT

u_int8_t cnmSapIsActive(struct ADAPTER *prAdapter);

u_int8_t cnmSapIsConcurrent(struct ADAPTER *prAdapter);

struct BSS_INFO *cnmGetSapBssInfo(struct ADAPTER *prAdapter);

struct BSS_INFO *
cnmGetOtherSapBssInfo(
	struct ADAPTER *prAdapter,
	struct BSS_INFO *prSapBssInfo);
#endif

void cnmOpModeGetTRxNss(
	struct ADAPTER *prAdapter,
	uint8_t ucBssIndex,
	uint8_t *pucOpRxNss,
	uint8_t *pucOpTxNss
);
#if CFG_SUPPORT_SMART_GEAR
void cnmEventSGStatus(
	struct ADAPTER *prAdapter,
	struct WIFI_EVENT *prEvent
);
#endif

void cnmOpmodeEventHandler(
	struct ADAPTER *prAdapter,
	struct WIFI_EVENT *prEvent
);

#if (CFG_SUPPORT_DFS_MASTER == 1)
void cnmRddOpmodeEventHandler(
	struct ADAPTER *prAdapter,
	struct WIFI_EVENT *prEvent
);
#endif

#if CFG_ENABLE_WIFI_DIRECT
u_int8_t cnmP2pIsActive(struct ADAPTER *prAdapter);

struct BSS_INFO *cnmGetP2pBssInfo(struct ADAPTER *prAdapter);
#endif

bool cnmIsMccMode(struct ADAPTER *prAdapter);

enum ENUM_BAND_80211 cnmGet80211Band(enum ENUM_BAND eBand);

#if (CFG_SUPPORT_POWER_THROTTLING == 1 && CFG_SUPPORT_CNM_POWER_CTRL == 1)
int cnmPowerControl(struct ADAPTER *prAdapter, uint8_t level);

void cnmPowerControlErrorHandling(
	struct ADAPTER *prAdapter,
	struct BSS_INFO *prBssInfo
);
#endif

#if (CFG_SUPPORT_DBDC == 1 && CFG_UPDATE_STATIC_DBDC_QUOTA == 1)
void cnmUpdateStaticDbdcQuota(
	struct ADAPTER *prAdapter);
#endif

/*******************************************************************************
 *                              F U N C T I O N S
 *******************************************************************************
 */
#ifndef _lint
/* We don't have to call following function to inspect the data structure.
 * It will check automatically while at compile time.
 * We'll need this to guarantee the same member order in different structures
 * to simply handling effort in some functions.
 */
static __KAL_INLINE__ void cnmMsgDataTypeCheck(void)
{
	DATA_STRUCT_INSPECTING_ASSERT(OFFSET_OF(
		struct MSG_CH_GRANT, rMsgHdr)
			== 0);

	DATA_STRUCT_INSPECTING_ASSERT(OFFSET_OF(
		struct MSG_CH_GRANT, rMsgHdr)
			== OFFSET_OF(struct MSG_CH_REOCVER, rMsgHdr));

	DATA_STRUCT_INSPECTING_ASSERT(OFFSET_OF(
		struct MSG_CH_GRANT, ucBssIndex)
			== OFFSET_OF(struct MSG_CH_REOCVER, ucBssIndex));

	DATA_STRUCT_INSPECTING_ASSERT(OFFSET_OF(
		struct MSG_CH_GRANT, ucTokenID)
			== OFFSET_OF(struct MSG_CH_REOCVER, ucTokenID));

	DATA_STRUCT_INSPECTING_ASSERT(OFFSET_OF(
		struct MSG_CH_GRANT, ucPrimaryChannel)
			== OFFSET_OF(struct MSG_CH_REOCVER, ucPrimaryChannel));

	DATA_STRUCT_INSPECTING_ASSERT(OFFSET_OF(
		struct MSG_CH_GRANT, eRfSco)
			== OFFSET_OF(struct MSG_CH_REOCVER, eRfSco));

	DATA_STRUCT_INSPECTING_ASSERT(OFFSET_OF(
		struct MSG_CH_GRANT, eRfBand)
			== OFFSET_OF(struct MSG_CH_REOCVER, eRfBand));

	DATA_STRUCT_INSPECTING_ASSERT(
		OFFSET_OF(struct MSG_CH_GRANT, eReqType)
			== OFFSET_OF(struct MSG_CH_REOCVER, eReqType));
}
#endif /* _lint */

uint8_t cnmIncreaseTokenId(struct ADAPTER *prAdapter);

uint8_t cnmOpModeGetMaxBw(struct ADAPTER *prAdapter,
	struct BSS_INFO *prBssInfo);

#endif /* _CNM_H */
