/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

/*
 ** Id: //Department/DaVinci/BRANCHES/MT6620_WIFI_DRIVER_V2_3/include
 *      /wlan_lib.h#3
 */

/*! \file   "wlan_lib.h"
 *    \brief  The declaration of the functions of the wlanAdpater objects
 *
 *    Detail description.
 */

#ifndef _WLAN_LIB_H
#define _WLAN_LIB_H

/*******************************************************************************
 *                         C O M P I L E R   F L A G S
 *******************************************************************************
 */

/*******************************************************************************
 *                    E X T E R N A L   R E F E R E N C E S
 *******************************************************************************
 */
#include "CFG_Wifi_File.h"
#include "rlm_domain.h"
#include "nic_init_cmd_event.h"
#include "fw_dl.h"


/*******************************************************************************
 *                              C O N S T A N T S
 *******************************************************************************
 */
/* These values must sync from Wifi HAL
  * /hardware/libhardware_legacy/include/hardware_legacy/wifi_hal.h
  */
/* Basic infrastructure mode */
#define WIFI_FEATURE_INFRA              (0x0001)
/* Support for 5 GHz Band */
#define WIFI_FEATURE_INFRA_5G           (0x0002)
/* Support for GAS/ANQP */
#define WIFI_FEATURE_HOTSPOT            (0x0004)
/* Wifi-Direct */
#define WIFI_FEATURE_P2P                (0x0008)
/* Soft AP */
#define WIFI_FEATURE_SOFT_AP            (0x0010)
/* Google-Scan APIs */
#define WIFI_FEATURE_GSCAN              (0x0020)
/* Neighbor Awareness Networking */
#define WIFI_FEATURE_NAN                (0x0040)
/* Device-to-device RTT */
#define WIFI_FEATURE_D2D_RTT            (0x0080)
/* Device-to-AP RTT */
#define WIFI_FEATURE_D2AP_RTT           (0x0100)
/* Batched Scan (legacy) */
#define WIFI_FEATURE_BATCH_SCAN         (0x0200)
/* Preferred network offload */
#define WIFI_FEATURE_PNO                (0x0400)
/* Support for two STAs */
#define WIFI_FEATURE_ADDITIONAL_STA     (0x0800)
/* Tunnel directed link setup */
#define WIFI_FEATURE_TDLS               (0x1000)
/* Support for TDLS off channel */
#define WIFI_FEATURE_TDLS_OFFCHANNEL    (0x2000)
/* Enhanced power reporting */
#define WIFI_FEATURE_EPR                (0x4000)
/* Support for AP STA Concurrency */
#define WIFI_FEATURE_AP_STA             (0x8000)
/* Link layer stats collection */
#define WIFI_FEATURE_LINK_LAYER_STATS   (0x10000)
/* WiFi Logger */
#define WIFI_FEATURE_LOGGER             (0x20000)
/* WiFi PNO enhanced */
#define WIFI_FEATURE_HAL_EPNO           (0x40000)
/* RSSI Monitor */
#define WIFI_FEATURE_RSSI_MONITOR       (0x80000)
/* WiFi mkeep_alive */
#define WIFI_FEATURE_MKEEP_ALIVE        (0x100000)
/* ND offload configure */
#define WIFI_FEATURE_CONFIG_NDO         (0x200000)
/* Capture Tx transmit power levels */
#define WIFI_FEATURE_TX_TRANSMIT_POWER  (0x400000)
/* Enable/Disable firmware roaming */
#define WIFI_FEATURE_CONTROL_ROAMING    (0x800000)
/* Support Probe IE white listing */
#define WIFI_FEATURE_IE_WHITELIST       (0x1000000)
/* Support MAC & Probe Sequence Number randomization */
#define WIFI_FEATURE_SCAN_RAND          (0x2000000)
/* Support Tx Power Limit setting */
#define WIFI_FEATURE_SET_TX_POWER_LIMIT (0x4000000)
/* Support Using Body/Head Proximity for SAR */
#define WIFI_FEATURE_USE_BODY_HEAD_SAR  (0x8000000)

/* note: WIFI_FEATURE_GSCAN be enabled  just for ACTS test item: scanner  */
#define WIFI_HAL_FEATURE_SET ((WIFI_FEATURE_P2P) |\
					  (WIFI_FEATURE_SOFT_AP) |\
					  (WIFI_FEATURE_TDLS) \
					  )

#define MAX_NUM_GROUP_ADDR		32 /* max number of group addresses */
#define AUTO_RATE_NUM			8
#define AR_RATE_TABLE_ENTRY_MAX		25
#define AR_RATE_ENTRY_INDEX_NULL	0x80
#define MAX_TX_QUALITY_INDEX		4

#define TX_CS_TCP_UDP_GEN	BIT(1)
#define TX_CS_IP_GEN		BIT(0)

#define CSUM_OFFLOAD_EN_TX_TCP	BIT(0)
#define CSUM_OFFLOAD_EN_TX_UDP	BIT(1)
#define CSUM_OFFLOAD_EN_TX_IP	BIT(2)
#define CSUM_OFFLOAD_EN_RX_TCP	BIT(3)
#define CSUM_OFFLOAD_EN_RX_UDP	BIT(4)
#define CSUM_OFFLOAD_EN_RX_IPv4	BIT(5)
#define CSUM_OFFLOAD_EN_RX_IPv6	BIT(6)
#define CSUM_OFFLOAD_EN_TX_MASK	BITS(0, 2)
#define CSUM_OFFLOAD_EN_ALL	BITS(0, 6)

/* TCP, UDP, IP Checksum */
#define RX_CS_TYPE_UDP		BIT(7)
#define RX_CS_TYPE_TCP		BIT(6)
#define RX_CS_TYPE_IPv6		BIT(5)
#define RX_CS_TYPE_IPv4		BIT(4)

#define RX_CS_STATUS_UDP	BIT(3)
#define RX_CS_STATUS_TCP	BIT(2)
#define RX_CS_STATUS_IP		BIT(0)

#define CSUM_NOT_SUPPORTED	0x0

#define TXPWR_USE_PDSLOPE	0

/* NVRAM error code definitions */
#define NVRAM_ERROR_VERSION_MISMATCH        BIT(1)
#define NVRAM_ERROR_INVALID_TXPWR           BIT(2)
#define NVRAM_ERROR_INVALID_DPD             BIT(3)
#define NVRAM_ERROR_INVALID_MAC_ADDR        BIT(4)
#if CFG_SUPPORT_PWR_LIMIT_COUNTRY
#define NVRAM_POWER_LIMIT_TABLE_INVALID     BIT(5)
#endif

#define NUM_TC_RESOURCE_TO_STATISTICS       4
#if CFG_SUPPORT_NCHO
#define WLAN_CFG_ARGV_MAX 64
#elif CFG_SUPPORT_WAKE_ON_PNO
#define WLAN_CFG_ARGV_MAX 16
#else
#define WLAN_CFG_ARGV_MAX 8
#endif
#define WLAN_CFG_ARGV_MAX_LONG	22	/* for WOW, 2+20 */
#define WLAN_CFG_ENTRY_NUM_MAX	200	/* 128 */
#define WLAN_CFG_KEY_LEN_MAX	32	/* include \x00  EOL */
#define WLAN_CFG_VALUE_LEN_MAX	32	/* include \x00 EOL */
#define WLAN_CFG_FLAG_SKIP_CB	BIT(0)
#define WLAN_CFG_FILE_BUF_SIZE	2048

#define WLAN_CFG_REC_ENTRY_NUM_MAX 200
#define WLAN_CFG_REC_FLAG_BIT BIT(0)


#define WLAN_CFG_SET_CHIP_LEN_MAX 10
#define WLAN_CFG_SET_DEBUG_LEVEL_LEN_MAX 10
#define WLAN_CFG_SET_SW_CTRL_LEN_MAX 10

/* OID timeout (in ms) */
#define WLAN_OID_TIMEOUT_THRESHOLD			2000

/* OID timeout during chip-resetting  (in ms) */
#define WLAN_OID_TIMEOUT_THRESHOLD_IN_RESETTING		300

#define WLAN_OID_NO_ACK_THRESHOLD			3


/* If not setting the priority, 0 is the default */
#define WLAN_THREAD_TASK_PRIORITY			0

/* If not setting the nice, -10 is the default */
#define WLAN_THREAD_TASK_NICE				(-10)

#define WLAN_TX_STATS_LOG_TIMEOUT			30000
#define WLAN_TX_STATS_LOG_DURATION			1500

/* Define for wifi path usage */
#define WLAN_FLAG_2G4_WF0		BIT(0)	/*1: support, 0: NOT support */
#define WLAN_FLAG_5G_WF0		BIT(1)	/*1: support, 0: NOT support */
#define WLAN_FLAG_2G4_WF1		BIT(2)	/*1: support, 0: NOT support */
#define WLAN_FLAG_5G_WF1		BIT(3)	/*1: support, 0: NOT support */
#define WLAN_FLAG_2G4_COANT_SUPPORT	BIT(4)	/*1: support, 0: NOT support */
#define WLAN_FLAG_2G4_COANT_PATH	BIT(5)	/*1: WF1, 0:WF0 */
#define WLAN_FLAG_5G_COANT_SUPPORT	BIT(6)	/*1: support, 0: NOT support */
#define WLAN_FLAG_5G_COANT_PATH		BIT(7)	/*1: WF1, 0:WF0 */

/* Define concurrent network channel number, using by CNM/CMD */
#define MAX_OP_CHNL_NUM			3

#if CFG_SUPPORT_EASY_DEBUG

#define MAX_CMD_ITEM_MAX		4	/* Max item per cmd. */
#define MAX_CMD_NAME_MAX_LENGTH		32	/* Max name string length */
#define MAX_CMD_VALUE_MAX_LENGTH	32	/* Max value string length */
#define MAX_CMD_TYPE_LENGTH		1
#define MAX_CMD_STRING_LENGTH		1
#define MAX_CMD_VALUE_LENGTH		1
#define MAX_CMD_RESERVE_LENGTH		1

#define CMD_FORMAT_V1_LENGTH	\
	(MAX_CMD_NAME_MAX_LENGTH + MAX_CMD_VALUE_MAX_LENGTH + \
	MAX_CMD_TYPE_LENGTH + MAX_CMD_STRING_LENGTH + MAX_CMD_VALUE_LENGTH + \
	MAX_CMD_RESERVE_LENGTH)

#define MAX_CMD_BUFFER_LENGTH	(CMD_FORMAT_V1_LENGTH * MAX_CMD_ITEM_MAX)

#if 1
#define ED_STRING_SITE		0
#define ED_VALUE_SITE		1


#else
#define ED_ITEMTYPE_SITE	0
#define ED_STRING_SITE		1
#define ED_VALUE_SITE		2
#endif

#if CFG_AUTO_CHANNEL_SEL_SUPPORT
#define ACS_AP_RSSI_LEVEL_HIGH		-50
#define ACS_AP_RSSI_LEVEL_LOW		-80
#define ACS_DIRTINESS_LEVEL_HIGH	52
#define ACS_DIRTINESS_LEVEL_MID		40
#define ACS_DIRTINESS_LEVEL_LOW		32
#endif

#if CFG_SUPPORT_TPENHANCE_MODE
#define TPENHANCE_SESSION_MAP_LEN	20
#define TPENHANCE_PKT_LATCH_MIN	    10
#define TPENHANCE_PKT_KEEP_MAX	    256
struct TPENHANCE_PKT_MAP {
	uint16_t au2SPort;
	uint16_t au2DPort;
	uint32_t au4Ip;
	uint16_t au2Hit;
};
#endif /* CFG_SUPPORT_TPENHANCE_MODE */

#if CFG_WOW_SUPPORT
#define INVALID_WOW_WAKE_UP_REASON 255
#endif

#if CFG_SUPPORT_ADVANCE_CONTROL
#define KEEP_FULL_PWR_TRAFFIC_REPORT_BIT BIT(0)
#define KEEP_FULL_PWR_NOISE_HISTOGRAM_BIT BIT(1)
#endif

#define ED_CCA_BW20_2G_DEFAULT (0)
#define ED_CCA_BW20_5G_DEFAULT (0)

#if CFG_RCPI_COMPENSATION
/* wf path module channel group info (Tx FW loss compensation) */
#define FELOSS_CH_GROUP_NUM 4 /* G-Band:1 A-Band:3*/

/* channel group boundary (FW_LOSS On for G-Band) */
#define G_BAND_FELOSS_BOUND_0 14

/* channel group boundary (FW_LOSS On for A-Band) */
#define A_BAND_FELOSS_BOUND_0 66
#define A_BAND_FELOSS_BOUND_1 130
#define A_BAND_FELOSS_BOUND_2 182

/* EFUSE offset for FE Loss */
#define G_BAND_WF0_FELOSS 0xE0
#define G_BAND_WF1_FELOSS 0xF0
#define A_BAND_WF0_LB_FELOSS 0x240
#define A_BAND_WF0_MB_FELOSS 0x241
#define A_BAND_WF0_HB_FELOSS 0x242
#define A_BAND_WF1_LB_FELOSS 0x2B0
#define A_BAND_WF1_MB_FELOSS 0x2B1
#define A_BAND_WF1_HB_FELOSS 0x2B2

#define TX_FELOSS_MASK            BITS(4, 7)
#define TX_FELOSS_OFFSET              4
#define RX_FELOSS_MASK            BITS(0, 3)
#define RX_FELOSS_OFFSET              0
#endif

#if CFG_SUPPORT_HW_1T2R
#define EFUSE_1T2R_ADDR					0x1B2
#define EFUSE_1T2R_OFFSET					5
#endif


enum CMD_VER {
	CMD_VER_1,	/* Type[2]+String[32]+Value[32] */
	CMD_VER_2	/* for furtur define. */
};


#if 0
enum ENUM_AIS_REQUEST_TYPE {
	AIS_REQUEST_SCAN,
	AIS_REQUEST_RECONNECT,
	AIS_REQUEST_ROAMING_SEARCH,
	AIS_REQUEST_ROAMING_CONNECT,
	AIS_REQUEST_REMAIN_ON_CHANNEL,
	AIS_REQUEST_NUM
};
#endif
enum CMD_TYPE {
	CMD_TYPE_QUERY,
	CMD_TYPE_SET
};


#define ITEM_TYPE_DEC	1
#define ITEM_TYPE_HEX	2
#define ITEM_TYPE_STR	3

enum CMD_DEFAULT_SETTING_VALUE {
	CMD_PNO_ENABLE,
	CMD_PNO_SCAN_PERIOD,
	CMD_SCN_CHANNEL_PLAN,
	CMD_SCN_DWELL_TIME,
	CMD_SCN_STOP_SCAN,
	CMD_MAX,
};

enum CMD_DEFAULT_STR_SETTING_VALUE {
	CMD_STR_TEST_STR,
	CMD_STR_MAX,
};

struct CMD_FORMAT_V1 {
	uint8_t itemType;
	uint8_t itemStringLength;
	uint8_t itemValueLength;
	uint8_t Reserved;
	uint8_t itemString[MAX_CMD_NAME_MAX_LENGTH];
	uint8_t itemValue[MAX_CMD_VALUE_MAX_LENGTH];
};

struct CMD_HEADER {
	enum CMD_VER	cmdVersion;
	enum CMD_TYPE	cmdType;
	uint8_t	itemNum;
	uint16_t	cmdBufferLen;
	uint8_t	buffer[MAX_CMD_BUFFER_LENGTH];
};

struct CFG_DEFAULT_SETTING_TABLE {
	uint32_t itemNum;
	const char *String;
	uint8_t itemType;
	uint32_t defaultValue;
	uint32_t minValue;
	uint32_t maxValue;
};

struct CFG_DEFAULT_SETTING_STR_TABLE {
	uint32_t itemNum;
	const char *String;
	uint8_t itemType;
	const char *DefString;
	uint16_t minLen;
	uint16_t maxLen;
};

struct CFG_QUERY_FORMAT {
	uint32_t Length;
	uint32_t Value;
	uint32_t Type;
	uint32_t *ptr;
};

/*Globol Configure define */
struct CFG_SETTING {
	uint8_t	PnoEnable;
	uint32_t PnoScanPeriod;
	uint8_t ScnChannelPlan;
	uint16_t ScnDwellTime;
	uint8_t ScnStopScan;
	uint8_t TestStr[80];
};

#endif

/*******************************************************************************
 *                             D A T A   T Y P E S
 *******************************************************************************
 */
typedef uint32_t(*PFN_OID_HANDLER_FUNC) (IN struct ADAPTER *prAdapter,
					 IN void *pvBuf, IN uint32_t u4BufLen,
					 OUT uint32_t *pu4OutInfoLen);

enum ENUM_CSUM_TYPE {
	CSUM_TYPE_IPV4,
	CSUM_TYPE_IPV6,
	CSUM_TYPE_TCP,
	CSUM_TYPE_UDP,
	CSUM_TYPE_NUM
};

enum ENUM_CSUM_RESULT {
	CSUM_RES_NONE,
	CSUM_RES_SUCCESS,
	CSUM_RES_FAILED,
	CSUM_RES_NUM
};

enum ENUM_PHY_MODE {
	ENUM_PHY_2G4_CCK,
	ENUM_PHY_2G4_OFDM_BPSK,
	ENUM_PHY_2G4_OFDM_QPSK,
	ENUM_PHY_2G4_OFDM_16QAM,
	ENUM_PHY_2G4_OFDM_48M,
	ENUM_PHY_2G4_OFDM_54M,
	ENUM_PHY_2G4_HT20_BPSK,
	ENUM_PHY_2G4_HT20_QPSK,
	ENUM_PHY_2G4_HT20_16QAM,
	ENUM_PHY_2G4_HT20_MCS5,
	ENUM_PHY_2G4_HT20_MCS6,
	ENUM_PHY_2G4_HT20_MCS7,
	ENUM_PHY_2G4_HT40_BPSK,
	ENUM_PHY_2G4_HT40_QPSK,
	ENUM_PHY_2G4_HT40_16QAM,
	ENUM_PHY_2G4_HT40_MCS5,
	ENUM_PHY_2G4_HT40_MCS6,
	ENUM_PHY_2G4_HT40_MCS7,
	ENUM_PHY_5G_OFDM_BPSK,
	ENUM_PHY_5G_OFDM_QPSK,
	ENUM_PHY_5G_OFDM_16QAM,
	ENUM_PHY_5G_OFDM_48M,
	ENUM_PHY_5G_OFDM_54M,
	ENUM_PHY_5G_HT20_BPSK,
	ENUM_PHY_5G_HT20_QPSK,
	ENUM_PHY_5G_HT20_16QAM,
	ENUM_PHY_5G_HT20_MCS5,
	ENUM_PHY_5G_HT20_MCS6,
	ENUM_PHY_5G_HT20_MCS7,
	ENUM_PHY_5G_HT40_BPSK,
	ENUM_PHY_5G_HT40_QPSK,
	ENUM_PHY_5G_HT40_16QAM,
	ENUM_PHY_5G_HT40_MCS5,
	ENUM_PHY_5G_HT40_MCS6,
	ENUM_PHY_5G_HT40_MCS7,
	ENUM_PHY_MODE_NUM
};

enum ENUM_POWER_SAVE_POLL_MODE {
	ENUM_POWER_SAVE_POLL_DISABLE,
	ENUM_POWER_SAVE_POLL_LEGACY_NULL,
	ENUM_POWER_SAVE_POLL_QOS_NULL,
	ENUM_POWER_SAVE_POLL_NUM
};

enum ENUM_AC_TYPE {
	ENUM_AC_TYPE_AC0,
	ENUM_AC_TYPE_AC1,
	ENUM_AC_TYPE_AC2,
	ENUM_AC_TYPE_AC3,
	ENUM_AC_TYPE_AC4,
	ENUM_AC_TYPE_AC5,
	ENUM_AC_TYPE_AC6,
	ENUM_AC_TYPE_BMC,
	ENUM_AC_TYPE_NUM
};

enum ENUM_ADV_AC_TYPE {
	ENUM_ADV_AC_TYPE_RX_NSW,
	ENUM_ADV_AC_TYPE_RX_PTA,
	ENUM_ADV_AC_TYPE_RX_SP,
	ENUM_ADV_AC_TYPE_TX_PTA,
	ENUM_ADV_AC_TYPE_TX_RSP,
	ENUM_ADV_AC_TYPE_NUM
};

enum ENUM_REG_CH_MAP {
	REG_CH_MAP_COUNTRY_CODE,
	REG_CH_MAP_TBL_IDX,
	REG_CH_MAP_CUSTOMIZED,
	REG_CH_MAP_NUM
};

enum ENUM_FEATURE_OPTION {
	FEATURE_DISABLED,
	FEATURE_ENABLED,
	FEATURE_FORCE_ENABLED
};

/* This enum is for later added feature options which use command reserved field
 * as option switch
 */
enum ENUM_FEATURE_OPTION_IN_CMD {
	FEATURE_OPT_CMD_AUTO,
	FEATURE_OPT_CMD_DISABLED,
	FEATURE_OPT_CMD_ENABLED,
	FEATURE_OPT_CMD_FORCE_ENABLED
};

#define DEBUG_MSG_SIZE_MAX 1200
enum {
	DEBUG_MSG_ID_UNKNOWN = 0x00,
	DEBUG_MSG_ID_PRINT = 0x01,
	DEBUG_MSG_ID_FWLOG = 0x02,
	DEBUG_MSG_ID_END
};

enum {
	DEBUG_MSG_TYPE_UNKNOWN = 0x00,
	DEBUG_MSG_TYPE_MEM8 = 0x01,
	DEBUG_MSG_TYPE_MEM32 = 0x02,
	DEBUG_MSG_TYPE_ASCII = 0x03,
	DEBUG_MSG_TYPE_BINARY = 0x04,
	DEBUG_MSG_TYPE_DRIVER = 0x05,
	DEBUG_MSG_TYPE_END
};

#if (CFG_SUPPORT_PKT_OFLD == 1)

#define PKT_OFLD_BUF_SIZE 1488
enum {
	PKT_OFLD_TYPE_APF = 0,
	PKT_OFLD_TYPE_IGMP,
	PKT_OFLD_TYPE_MDNS,
	PKT_OFLD_TYPE_CUSTOM,
	PKT_OFLD_TYPE_END
};

enum {
	PKT_OFLD_OP_DISABLE = 0,
	PKT_OFLD_OP_ENABLE,
	PKT_OFLD_OP_INSTALL,
	PKT_OFLD_OP_QUERY,
	PKT_OFLD_OP_END
};
#endif /* CFG_SUPPORT_PKT_OFLD */
#define CHIP_CONFIG_RESP_SIZE 320
enum {
	CHIP_CONFIG_TYPE_WO_RESPONSE = 0x00,
	CHIP_CONFIG_TYPE_MEM8 = 0x01,
	CHIP_CONFIG_TYPE_MEM32 = 0x02,
	CHIP_CONFIG_TYPE_ASCII = 0x03,
	CHIP_CONFIG_TYPE_BINARY = 0x04,
	CHIP_CONFIG_TYPE_DRV_PASSTHROUGH = 0x05,
	CHIP_CONFIG_TYPE_END
};

struct SET_TXPWR_CTRL {
	int8_t c2GLegacyStaPwrOffset;	/* Unit: 0.5dBm, default: 0 */
	int8_t c2GHotspotPwrOffset;
	int8_t c2GP2pPwrOffset;
	int8_t c2GBowPwrOffset;
	int8_t c5GLegacyStaPwrOffset;	/* Unit: 0.5dBm, default: 0 */
	int8_t c5GHotspotPwrOffset;
	int8_t c5GP2pPwrOffset;
	int8_t c5GBowPwrOffset;
	uint8_t ucConcurrencePolicy;	/* TX power policy when concurrence
					 *  in the same channel
					 *  0: Highest power has priority
					 *  1: Lowest power has priority
					 */

	int8_t acReserved1[3];		/* Must be zero */

	/* Power limit by channel for all data rates */
	int8_t acTxPwrLimit2G[14];	/* Channel 1~14, Unit: 0.5dBm */
	int8_t acTxPwrLimit5G[4];	/* UNII 1~4 */
	int8_t acReserved2[2];		/* Must be zero */
};

#if CFG_WOW_SUPPORT

struct WOW_WAKE_HIF {
	/* use in-band signal to wakeup system, ENUM_HIF_TYPE */
	uint8_t		ucWakeupHif;

	/* GPIO Pin */
	uint8_t		ucGpioPin;

	/* refer to PF_WAKEUP_CMD_BIT0_OUTPUT_MODE_EN */
	uint8_t		ucTriggerLvl;

	/* non-zero means output reverse wakeup signal after delay time */
	uint32_t	u4GpioInterval;

	uint8_t		aucResv[5];
};

struct WOW_PORT {
	uint8_t ucIPv4UdpPortCnt;
	uint8_t ucIPv4TcpPortCnt;
	uint8_t ucIPv6UdpPortCnt;
	uint8_t ucIPv6TcpPortCnt;
	uint16_t ausIPv4UdpPort[MAX_TCP_UDP_PORT];
	uint16_t ausIPv4TcpPort[MAX_TCP_UDP_PORT];
	uint16_t ausIPv6UdpPort[MAX_TCP_UDP_PORT];
	uint16_t ausIPv6TcpPort[MAX_TCP_UDP_PORT];
};

struct WOW_CTRL {
	uint8_t fgWowEnable;	/* 0: disable, 1: wow enable */
	uint8_t ucScenarioId;	/* just a profile ID */
	uint8_t ucBlockCount;
	uint8_t ucDetectType;
	struct WOW_WAKE_HIF astWakeHif[2];
	struct WOW_PORT stWowPort;
	uint8_t ucReason;
	uint8_t aucReserved2[3];
};

#if CFG_SUPPORT_MDNS_OFFLOAD

/* Maximum size of the data array is defined as 4KB */
#define MAX_MDNS_USE_SIZE 4096
/* Maximum size of the transfer size is  as 1KB */
#define MAX_MDNS_TRANSFER_SIZE 1024

/* fail of mdns data struct oversize */
#define FAIL_MDNS_OVERSIZE 65535

/* mdns record max number */
#define MAX_MDNS_CACHE_NUM	10
/* mdns passthrough max number */
#define MAX_MDNS_PASSTHTOUGH_NUM 20

/*
 * DataBlock structure to hold the actual data.
 * The data in 'data' array is organized in a way
 * that it holds a two-byte length and then the data,
 * following this pattern: LENGTH HIGH_BYTE LENGTH LOW_BYTE DATA[LENGTH].
 *
 DataBlock Design
+----------------------------------------------------------------------------+
|                                   DataBlock                                |
| +------------------------------------------------------------------------+ |
| | length1 H-byte | length1 L-byte | data1[1] | data1[2] |  | data1[length1]|
| | length2 H-byte | length2 L-byte | data2[1] | data2[2] |  | data2[length2]|
| |                             ............................               | |
| | lengthN H-byte | lengthN L-byte | dataN[1] | dataN[2] |  | dataN[lengthN]|
| +------------------------------------------------------------------------+ |
| |                                     index                                |
+----------------------------------------------------------------------------+
 */
struct MDNS_DATABLOCK_T  {
    /* An array to mdns record and passthrough payload */
	uint8_t data[MAX_MDNS_USE_SIZE];
	/*the used size in data ,max 4096 */
	uint16_t index;
};

/*
 * Index structure to hold the indices pointing to locations of specific data
 * (response and name) in the data array within a DataBlock structure.
 *
 * Index Design
 *    +-------------------+     +-------------------+     +-------------------+
 *     |       Index       |     |       Index       |     |       Index       |
 *     | +---------------+ |     | +---------------+ |     | +---------------+ |
 *     | |      type     | |     | |      type     | |     | |      type     | |
 *     | +---------------+ |     | +---------------+ |     | +---------------+ |
 *     | | responseIndex | |     | | responseIndex | |     | | responseIndex | |
 *     | +---------------+ |     | +---------------+ |     | +---------------+ |
 *     | |   nameIndex   | |     | |  nameIndex    | |     | |  nameIndex    | |
 *     | +---------------+ |     | +---------------+ |     | +---------------+ |
 *     +-------------------+     +-------------------+     +-------------------+
 */

struct MDNS_RECORD_T {
/*
 * Type variable structure:
 *
 * |--------8 bits--------|
 * |  RRRR    |   TTTT    |
 * |__________|___________|
 *
 * The first four bits (RRRR) represent the number of records in current record.
 * For example, if the value is 2, there are 2 records in the current record.
 * max to 4
 *
 * The last four bits (TTTT) represent the type of the current record,
 * with each bitrepresenting a different type. If a bit is set,
 * that means the corresponding recordtype is valid. For example,
 * if the value is `1100`, the first two types are valid, and
 * the last two types are not valid.
 *
 * bit 0 : MDNS_ELEM_TYPE_PTR
 * bit 1 : MDNS_ELEM_TYPE_PTR
 * bit 2 : MDNS_ELEM_TYPE_SRV
 * bit 3 : MDNS_ELEM_TYPE_A
 */
	uint8_t  type;
/* index of the 'response' data in the data array */
/* The first two are the length, and the rest is the valid data */
/*|length1 H-byte | length1 L-byte | data1[1] | data1[2] || data1[length1]*/
	uint16_t responseIndex;
/* index of the 'name' data in the data array */
/* The first two are the length, and the rest is the valid data */
/*|length1 H-byte | length1 L-byte | data1[1] | data1[2] || data1[length1]*/
	uint16_t nameIndex[4];
};


/*
 * Name Index Array structure to hold indices of specific
 * names in the data array
 * It can store up to MAX_MDNS_PASSTHTOUGH_NUM indices
 * For passrthrough
 * Passrthrough
 * +------------------------------------+
 * |            Passrthrough            |
 * | +-------------------------------+  |
 * | |nameIndices[MAX_MDNS_PASSTHTOUGH_NUM]
 * | +-------------------------------+  |
 * | |             count             |  |
 * | +-------------------------------+  |
 * +------------------------------------+
 */
struct MDNS_PASSTHROUGH_T {
/* Current number of passthrough stored name indices */
/*  passthrough number max to MAX_MDNS_PASSTHTOUGH_NUM */
	uint8_t count;
/* index of the 'passthroughname' data in the data array */
/* The first two are the length, and the rest is the valid data */
/*|length1 H-byte | length1 L-byte | data1[1] | data1[2] || data1[length1]*/
	uint16_t nameIndices[MAX_MDNS_PASSTHTOUGH_NUM];
};

/* mdns record max response length */
#define MDNS_RESPONSE_RECORD_MAX_LEN	500
/* mdns record max name length */
#define MDNS_QUESTION_NAME_MAX_LEN	102

/* mdns and mdns record cmd */
#define MDNS_CMD_ENABLE		1
#define MDNS_CMD_DISABLE	2
#define MDNS_CMD_ADD_RECORD	3
#define MDNS_CMD_CLEAR_RECORD	4
#define MDNS_CMD_DEL_RECORD	5

/* mdns and mdns passthrough cmd */
#define MDNS_CMD_SET_PASSTHTOUGH    6
#define MDNS_CMD_ADD_PASSTHTOUGH    7
#define MDNS_CMD_DEL_PASSTHTOUGH    8
#define MDNS_CMD_GET_HITCOUNTER     9
#define MDNS_CMD_GET_MISSCOUNTER    10
#define MDNS_CMD_RESETALL           11
#define MDNS_CMD_CLEAR_PASSTHTOUGH  12
/* IPV6 wake up host*/
#define MDNS_CMD_SET_IPV6_WAKEUP_FLAG	13
#define MDNS_CMD_SET_WAKEUP_FLAG	14

/* ucCmd passthrouth Behavior  */
enum MDNS_PassthroughBehavior {
	MDNS_PASSTHROUGH_FORWARD_ALL = 1,
	MDNS_PASSTHROUGH_DROP_ALL = 2,
	MDNS_PASSTHROUGH_LIST = 3
};

#define MDNS_PAYLOAD_TYPE_LEN				2
#define MDNS_PAYLOAD_CLASS_LEN				2
#define MDNS_PAYLOAD_TTL_LEN				4
#define MDNS_PAYLOAD_DATALEN_LEN			2

#define MDNS_ELEM_TYPE_PTR		12
#define MDNS_ELEM_TYPE_SRV		33
#define MDNS_ELEM_TYPE_TXT		16
#define MDNS_ELEM_TYPE_A		1


#define MDNS_WAKEUP_BY_NO_MATCH_RECORD BIT(0)
#define MDNS_WAKEUP_BY_SUB_REQ	BIT(1)

struct WLAN_MAC_HEADER_QoS_T {
	uint16_t u2FrameCtrl;
	uint16_t DurationID;
	uint8_t aucAddr1[MAC_ADDR_LEN];
	uint8_t aucAddr2[MAC_ADDR_LEN];
	uint8_t aucAddr3[MAC_ADDR_LEN];
	uint16_t u2SeqCtrl;
	uint16_t u2QosCtrl;
};

struct WLAN_MDNS_HDR_T {
	uint16_t usMdnsId;
	uint16_t usMdnsFlags;
	uint16_t usQuestionCnt;
	uint16_t usAnswerCnt;
	uint16_t usAuthCnt;
	uint16_t usAddtionCnt;
};

#define UDP_HEADER_LENGTH 8
#define IPV4_HEADER_LENGTH 20

struct MDNS_TEMPLATE_T {
	uint8_t name[MDNS_QUESTION_NAME_MAX_LEN];
	uint8_t name_length;
	uint8_t ucPadding0; /*padding*/
	uint16_t class;
	uint16_t type;
};

struct MDNS_PARAM_T {
	struct MDNS_TEMPLATE_T query_ptr;
	struct MDNS_TEMPLATE_T query_srv;
	struct MDNS_TEMPLATE_T query_txt;
	struct MDNS_TEMPLATE_T query_a;
	uint16_t response_len;
	uint8_t ucPadding0[2]; /*padding*/
	uint8_t response[MDNS_RESPONSE_RECORD_MAX_LEN];
};

struct MDNS_PASSTHROUGHLIST_T {
	uint8_t name[MDNS_QUESTION_NAME_MAX_LEN];
	uint16_t u2PassthroghLength;
};

struct MDNS_INFO_UPLAYER_T {
	uint8_t ucCmd;
	struct MDNS_PARAM_T mdns_param;
	uint8_t recordKey;
	uint8_t name[MDNS_QUESTION_NAME_MAX_LEN];
	uint8_t passthroughBehavior;
	uint8_t ucIPV6WakeupFlag;
};

struct MDNS_PARAM_ENTRY_T {
	struct LINK_ENTRY rLinkEntry;
	struct MDNS_PARAM_T mdns_param;
	uint8_t recordKey;
};

struct MDNS_PASSTHROUGH_ENTRY_T {
	struct LINK_ENTRY rLinkEntry;
	struct MDNS_PASSTHROUGHLIST_T mdns_passthrough;
};

struct CMD_MDNS_PARAM_T {
    /* 1 Byte fields, total 8 bytes */
	uint8_t ucCmd;
	uint8_t ucRecordId;
	uint8_t ucWakeFlag;
	uint8_t ucPassthrouthId;

	uint8_t ucPassthroughBehavior;
	uint8_t ucIPV6WakeupFlag;
	/* mdns total transfer length 0 - 4096 */
	uint8_t ucPayloadOrder;
	uint8_t ucPadding;

	/* 26 bytes */
	struct WLAN_MAC_HEADER_QoS_T aucMdnsMacHdr;

	/* 2 bytes */
	/* mdns total transfer length 0 - 4096 */
	uint16_t u2PayloadTotallength;

	/* 20 bytes */
	uint8_t aucMdnsIPHdr[IPV4_HEADER_LENGTH];

	/* 8 bytes */
	uint8_t aucMdnsUdpHdr[UDP_HEADER_LENGTH];

	/* 1024 bytes */
	/* mdns of 1024 per transmission*/
	uint8_t ucPayload[MAX_MDNS_TRANSFER_SIZE];
};

struct EVENT_ID_MDNS_RECORD_T {
	/* DWORD_0 */
	uint8_t ucVersion;
	uint8_t ucType; /* 0: invalid, 1: Hit 2: Miss */
	uint16_t u2ControlFlag;
	/* DWORD_1 */
	uint32_t u4MdnsHitMiss;
	/* DWORD_2 */
	uint8_t aucReserved2[64];
};

struct MDNS_SETTING_FLAGS_T {
/* DWORD_0 */
	uint8_t ucSetPortFlag;
	uint8_t ucPassthroughBehavior;
	uint8_t ucIPV6WakeupFlag;
	uint8_t ucPadding1[1]; /*padding*/
};

struct MDNS_INFO_T {
	struct LINK rMdnsRecordList;
	struct LINK rMdnsRecordFreeList;
	struct MDNS_PARAM_ENTRY_T rMdnsEntry[MAX_MDNS_CACHE_NUM];
	int rMdnsRecordCout;
	int rMdnsPassthroughCout;

	struct LINK rMdnsPassthroughList;
	struct LINK rMdnsPassthroughFreeList;
	struct MDNS_PASSTHROUGH_ENTRY_T
		rMdnsPassthroughEntry[MAX_MDNS_PASSTHTOUGH_NUM];
	struct EVENT_ID_MDNS_RECORD_T rMdnsRecordEvent;
	struct MDNS_SETTING_FLAGS_T rMdnsSaveFlags;

	struct MDNS_RECORD_T rMdnsRecordIndices[MAX_MDNS_CACHE_NUM];
	uint16_t currentIndex;

	struct MDNS_PASSTHROUGH_T passrthrough;
	struct MDNS_DATABLOCK_T  dataBlock;

};
#endif
#endif

#if CFG_SUPPORT_CSI
struct CSI_DATA_T;
#endif

enum ENUM_NVRAM_MTK_FEATURE {
	MTK_FEATURE_2G_256QAM_DISABLED = 0,
	MTK_FEATURE_NUM
};

/* For storing driver initialization value from glue layer */
struct REG_INFO {
	uint32_t u4SdBlockSize;	/* SDIO block size */
	uint32_t u4SdBusWidth;	/* SDIO bus width. 1 or 4 */
	uint32_t u4SdClockRate;	/* SDIO clock rate. (in unit of HZ) */

	/* Start Frequency for Ad-Hoc network : in unit of KHz */
	uint32_t u4StartFreq;

	/* Default mode for Ad-Hoc network : ENUM_PARAM_AD_HOC_MODE_T */
	uint32_t u4AdhocMode;

	uint32_t u4RddStartFreq;
	uint32_t u4RddStopFreq;
	uint32_t u4RddTestMode;
	uint32_t u4RddShutFreq;
	uint32_t u4RddDfs;
	int32_t i4HighRssiThreshold;
	int32_t i4MediumRssiThreshold;
	int32_t i4LowRssiThreshold;
	int32_t au4TxPriorityTag[ENUM_AC_TYPE_NUM];
	int32_t au4RxPriorityTag[ENUM_AC_TYPE_NUM];
	int32_t au4AdvPriorityTag[ENUM_ADV_AC_TYPE_NUM];
	uint32_t u4FastPSPoll;
	uint32_t u4PTA;		/* 0: disable, 1: enable */
	uint32_t u4TXLimit;	/* 0: disable, 1: enable */
	uint32_t u4SilenceWindow;	/* range: 100 - 625, unit: us */
	uint32_t u4TXLimitThreshold;	/* range: 250 - 1250, unit: us */
	uint32_t u4PowerMode;
	uint32_t fgEnArpFilter;
	uint32_t u4PsCurrentMeasureEn;
	uint32_t u4UapsdAcBmp;
	uint32_t u4MaxSpLen;

	/* 0: enable online scan, non-zero: disable online scan */
	uint32_t fgDisOnlineScan;

	/* 0: enable online scan, non-zero: disable online scan */
	uint32_t fgDisBcnLostDetection;

	/* 0: automatic, non-zero: fixed rate */
	uint32_t u4FixedRate;

	uint32_t u4ArSysParam0;
	uint32_t u4ArSysParam1;
	uint32_t u4ArSysParam2;
	uint32_t u4ArSysParam3;

	/* 0:enable roaming 1:disable */
	uint32_t fgDisRoaming;

	/* NVRAM - MP Data -START- */
#if 1
	uint16_t u2Part1OwnVersion;
	uint16_t u2Part1PeerVersion;
#endif
	uint8_t aucMacAddr[6];

	/* Country code (in ISO 3166-1 expression, ex: "US", "TW")  */
	uint16_t au2CountryCode[4];

	struct TX_PWR_PARAM rTxPwr;
	uint8_t aucEFUSE[144];
	uint8_t ucTxPwrValid;
	uint8_t ucSupport5GBand;
	uint8_t fg2G4BandEdgePwrUsed;
	int8_t cBandEdgeMaxPwrCCK;
	int8_t cBandEdgeMaxPwrOFDM20;
	int8_t cBandEdgeMaxPwrOFDM40;
	enum ENUM_REG_CH_MAP eRegChannelListMap;
	uint8_t ucRegChannelListIndex;
	struct DOMAIN_INFO_ENTRY rDomainInfo;
	struct RSSI_PATH_COMPASATION rRssiPathCompasation;
	uint8_t ucRssiPathCompasationUsed;
	/* NVRAM - MP Data -END- */

	/* NVRAM - Functional Data -START- */
	uint8_t uc2G4BwFixed20M;
	uint8_t uc5GBwFixed20M;
	uint8_t ucEnable5GBand;
	uint8_t ucGpsDesense;
	uint8_t ucRxDiversity;
	/* NVRAM - Functional Data -END- */

	struct NEW_EFUSE_MAPPING2NVRAM *prOldEfuseMapping;

	uint8_t aucNvram[512];
	struct WIFI_CFG_PARAM_STRUCT *prNvramSettings;
};

/* for divided firmware loading */
struct FWDL_SECTION_INFO {
#if 0
	uint32_t u4Offset;
	uint32_t u4Reserved;
	uint32_t u4Length;
	uint32_t u4DestAddr;
#endif
	uint32_t u4DestAddr;
	uint8_t ucChipInfo;
	uint8_t ucFeatureSet;
	uint8_t ucEcoCode;
	uint8_t aucReserved[9];
	uint8_t aucBuildDate[16];
	uint32_t u4Length;
};

struct FIRMWARE_DIVIDED_DOWNLOAD {
#if 0
	uint32_t u4Signature;

	/* CRC calculated without first 8 bytes included */
	uint32_t u4CRC;

	uint32_t u4NumOfEntries;
	uint32_t u4Reserved;
	struct FWDL_SECTION_INFO arSection[];
#endif
	struct FWDL_SECTION_INFO arSection[2];
};

struct PARAM_MCR_RW_STRUCT {
	uint32_t u4McrOffset;
	uint32_t u4McrData;
};

/* per access category statistics */
struct WIFI_WMM_AC_STAT {
	uint32_t u4TxMsdu;
	uint32_t u4RxMsdu;
	uint32_t u4TxDropMsdu;
	uint32_t u4TxFailMsdu;
	uint32_t u4TxRetryMsdu;
};

struct TX_VECTOR_BBP_LATCH {
	uint32_t u4TxVector1;
	uint32_t u4TxVector2;
	uint32_t u4TxVector4;
};

struct MIB_INFO_STAT {
	uint32_t u4RxMpduCnt;
	uint32_t u4FcsError;
	uint32_t u4RxFifoFull;
	uint32_t u4AmpduTxSfCnt;
	uint32_t u4AmpduTxAckSfCnt;
	uint16_t u2TxRange1AmpduCnt;
	uint16_t u2TxRange2AmpduCnt;
	uint16_t u2TxRange3AmpduCnt;
	uint16_t u2TxRange4AmpduCnt;
	uint16_t u2TxRange5AmpduCnt;
	uint16_t u2TxRange6AmpduCnt;
	uint16_t u2TxRange7AmpduCnt;
	uint16_t u2TxRange8AmpduCnt;
};

struct PARAM_GET_STA_STATISTICS {
	/* Per-STA statistic */
	uint8_t ucInvalid;
	uint8_t ucVersion;
	uint8_t aucMacAddr[MAC_ADDR_LEN];

	uint32_t u4LinkScore;
	uint32_t u4Flag;

	uint8_t ucReadClear;
	uint8_t ucLlsReadClear;

	/* From driver */
	uint32_t u4TxTotalCount;
	uint32_t u4TxExceedThresholdCount;

	uint32_t u4TxMaxTime;
	uint32_t u4TxAverageProcessTime;

	uint32_t u4TxMaxHifTime;
	uint32_t u4TxAverageHifTime;

	uint32_t u4RxTotalCount;

	/*
	 * How many packages Enqueue/Deqeue during statistics interval
	 */
	uint32_t u4EnqueueCounter;
	uint32_t u4DequeueCounter;

	uint32_t u4EnqueueStaCounter;
	uint32_t u4DequeueStaCounter;

	uint32_t IsrCnt;
	uint32_t IsrPassCnt;
	uint32_t TaskIsrCnt;

	uint32_t IsrAbnormalCnt;
	uint32_t IsrSoftWareCnt;
	uint32_t IsrRxCnt;
	uint32_t IsrTxCnt;

	uint32_t au4TcResourceEmptyCount[NUM_TC_RESOURCE_TO_STATISTICS];
	uint32_t au4DequeueNoTcResource[NUM_TC_RESOURCE_TO_STATISTICS];
	uint32_t au4TcResourceBackCount[NUM_TC_RESOURCE_TO_STATISTICS];
	uint32_t au4TcResourceUsedPageCount[NUM_TC_RESOURCE_TO_STATISTICS];
	uint32_t au4TcResourceWantedPageCount[NUM_TC_RESOURCE_TO_STATISTICS];

	uint32_t au4TcQueLen[NUM_TC_RESOURCE_TO_STATISTICS];

	/* From FW */
	uint8_t ucPer;		/* base: 128 */
	uint8_t ucRcpi;
	uint32_t u4PhyMode;
	uint16_t u2LinkSpeed;	/* unit is 0.5 Mbits */

	uint32_t u4TxFailCount;
	uint32_t u4TxLifeTimeoutCount;

	uint32_t u4TxAverageAirTime;

	/* Transmit in the air (wtbl) */
	uint32_t u4TransmitCount;

	/* Transmit without ack/ba in the air (wtbl) */
	uint32_t u4TransmitFailCount;

	/*link layer statistics */
	struct WIFI_WMM_AC_STAT arLinkStatistics[AC_NUM];

	/* Global queue management statistic */
	uint32_t au4TcAverageQueLen[NUM_TC_RESOURCE_TO_STATISTICS];
	uint32_t au4TcCurrentQueLen[NUM_TC_RESOURCE_TO_STATISTICS];

	uint8_t ucTemperature;
	uint8_t ucSkipAr;
	uint8_t ucArTableIdx;
	uint8_t ucRateEntryIdx;
	uint8_t ucRateEntryIdxPrev;
	uint8_t ucTxSgiDetectPassCnt;
	uint8_t ucAvePer;
#if (CFG_SUPPORT_RA_GEN == 0)
	uint8_t aucArRatePer[AR_RATE_TABLE_ENTRY_MAX];
	uint8_t aucRateEntryIndex[AUTO_RATE_NUM];
#else
	uint32_t u4AggRangeCtrl_0;
	uint32_t u4AggRangeCtrl_1;
	uint8_t ucRangeType;
	uint8_t aucReserved5[24];
#endif
	uint8_t ucArStateCurr;
	uint8_t ucArStatePrev;
	uint8_t ucArActionType;
	uint8_t ucHighestRateCnt;
	uint8_t ucLowestRateCnt;
	uint16_t u2TrainUp;
	uint16_t u2TrainDown;
	uint32_t u4Rate1TxCnt;
	uint32_t u4Rate1FailCnt;
	struct TX_VECTOR_BBP_LATCH rTxVector[ENUM_BAND_NUM];
	struct MIB_INFO_STAT rMibInfo[ENUM_BAND_NUM];
	uint8_t ucResetCounter;
	u_int8_t fgIsForceTxStream;
	u_int8_t fgIsForceSeOff;
#if (CFG_SUPPORT_RA_GEN == 0)
	uint8_t aucReserved6[17];
#else
	uint16_t u2RaRunningCnt;
	uint8_t ucRaStatus;
	uint8_t ucFlag;
	uint8_t aucTxQuality[MAX_TX_QUALITY_INDEX];
	uint8_t ucTxRateUpPenalty;
	uint8_t ucLowTrafficMode;
	uint8_t ucLowTrafficCount;
	uint8_t ucLowTrafficDashBoard;
	uint8_t ucDynamicSGIState;
	uint8_t ucDynamicSGIScore;
	uint8_t ucDynamicBWState;
	uint8_t ucDynamicGband256QAMState;
	uint8_t ucVhtNonSpRateState;
#endif
	/* Reserved fields */
	uint8_t au4Reserved[3];
};

struct PARAM_GET_BSS_STATISTICS {
	/* Per-STA statistic */
	uint8_t aucMacAddr[MAC_ADDR_LEN];

	uint32_t u4Flag;

	uint8_t ucReadClear;

	uint8_t ucLlsReadClear;

	uint8_t ucBssIndex;

	/* From driver */
	uint32_t u4TxTotalCount;
	uint32_t u4TxExceedThresholdCount;

	uint32_t u4TxMaxTime;
	uint32_t u4TxAverageProcessTime;

	uint32_t u4RxTotalCount;

	uint32_t au4TcResourceEmptyCount[NUM_TC_RESOURCE_TO_STATISTICS];
	uint32_t au4TcQueLen[NUM_TC_RESOURCE_TO_STATISTICS];

	/* From FW */
	uint8_t ucPer;		/* base: 128 */
	uint8_t ucRcpi;
	uint32_t u4PhyMode;
	uint16_t u2LinkSpeed;	/* unit is 0.5 Mbits */

	uint32_t u4TxFailCount;
	uint32_t u4TxLifeTimeoutCount;

	uint32_t u4TxAverageAirTime;

	/* Transmit in the air (wtbl) */
	uint32_t u4TransmitCount;

	/* Transmit without ack/ba in the air (wtbl) */
	uint32_t u4TransmitFailCount;

	/*link layer statistics */
	struct WIFI_WMM_AC_STAT arLinkStatistics[AC_NUM];

	/* Global queue management statistic */
	uint32_t au4TcAverageQueLen[NUM_TC_RESOURCE_TO_STATISTICS];
	uint32_t au4TcCurrentQueLen[NUM_TC_RESOURCE_TO_STATISTICS];

	/* Reserved fields */
	uint8_t au4Reserved[32];	/* insufficient for LLS?? */
};

struct PARAM_GET_DRV_STATISTICS {
	int32_t i4TxPendingFrameNum;
	int32_t i4TxPendingSecurityFrameNum;
	int32_t i4TxPendingCmdNum;

	/* sync i4PendingFwdFrameCount in _TX_CTRL_T */
	int32_t i4PendingFwdFrameCount;

	/* sync pad->rTxCtrl.rFreeMsduInfoList.u4NumElem */
	uint32_t u4MsduNumElem;

	/* sync pad->rTxCtrl.rTxMgmtTxingQueue.u4NumElem */
	uint32_t u4TxMgmtTxringQueueNumElem;

	/* sync pad->prRxCtrl.rFreeSwRfbList.u4NumElem */
	uint32_t u4RxFreeSwRfbMsduNumElem;

	/* sync pad->prRxCtrl.rReceivedRfbList.u4NumElem */
	uint32_t u4RxReceivedRfbNumElem;

	/* sync pad->prRxCtrl.rIndicatedRfbList.u4NumElem */
	uint32_t u4RxIndicatedNumElem;
};

struct NET_INTERFACE_INFO {
	uint8_t ucBssIndex;
	void *pvNetInterface;
};

enum ENUM_TX_RESULT_CODE {
	TX_RESULT_SUCCESS = 0,
	TX_RESULT_LIFE_TIMEOUT,
	TX_RESULT_RTS_ERROR,
	TX_RESULT_MPDU_ERROR,
	TX_RESULT_AGING_TIMEOUT,
	TX_RESULT_FLUSHED,
	TX_RESULT_BIP_ERROR,
	TX_RESULT_UNSPECIFIED_ERROR,
	TX_RESULT_DROPPED_IN_DRIVER = 32,
	TX_RESULT_DROPPED_IN_FW,
	TX_RESULT_QUEUE_CLEARANCE,
	TX_RESULT_INACTIVE_BSS,
	TX_RESULT_NUM
};

/* enum of BEACON_TIMEOUT_REASON */
enum _ENUM_PM_BEACON_TIME_OUT_REACON_CODE_T {
	BEACON_TIMEOUT_DUE_2_HW_BEACON_LOST_NONADHOC,
	BEACON_TIMEOUT_DUE_2_HW_BEACON_LOST_ADHOC,
	BEACON_TIMEOUT_DUE_2_HW_TSF_DRIFT,
	BEACON_TIMEOUT_DUE_2_NULL_FRAME_THRESHOLD,
	BEACON_TIMEOUT_DUE_2_AGING_THRESHOLD,
	BEACON_TIMEOUT_DUE_2_BSSID_BEACON_PEIROD_NOT_ILLIGAL,
	BEACON_TIMEOUT_DUE_2_CONNECTION_FAIL,
	BEACON_TIMEOUT_DUE_2_ALLOCAT_NULL_PKT_FAIL_THRESHOLD,
	BEACON_TIMEOUT_DUE_2_NO_TX_DONE_EVENT,
	BEACON_TIMEOUT_DUE_2_UNSPECIF_REASON,
	BEACON_TIMEOUT_DUE_2_SET_CHIP,
	BEACON_TIMEOUT_DUE_2_KEEP_SCAN_AP_MISS_CHECK_FAIL,
	BEACON_TIMEOUT_DUE_2_KEEP_UNCHANGED_LOW_RSSI_CHECK_FAIL,
	BEACON_TIMEOUT_DUE_2_NULL_FRAME_LIFE_TIMEOUT,
	BEACON_TIMEOUT_DUE_2_APR_NO_RESPONSE,
	BEACON_TIMEOUT_DUE_2_NUM
};

struct WLAN_CFG_ENTRY {
	uint8_t aucKey[WLAN_CFG_KEY_LEN_MAX];
	uint8_t aucValue[WLAN_CFG_VALUE_LEN_MAX];
	WLAN_CFG_SET_CB pfSetCb;
	void *pPrivate;
	uint32_t u4Flags;
};

struct WLAN_CFG {
	uint32_t u4WlanCfgEntryNumMax;
	uint32_t u4WlanCfgKeyLenMax;
	uint32_t u4WlanCfgValueLenMax;
	struct WLAN_CFG_ENTRY arWlanCfgBuf[WLAN_CFG_ENTRY_NUM_MAX];
};

struct WLAN_CFG_REC {
	uint32_t u4WlanCfgEntryNumMax;
	uint32_t u4WlanCfgKeyLenMax;
	uint32_t u4WlanCfgValueLenMax;
	struct WLAN_CFG_ENTRY arWlanCfgBuf[WLAN_CFG_REC_ENTRY_NUM_MAX];
};

enum ENUM_MAX_BANDWIDTH_SETTING {
	MAX_BW_20MHZ = 0,
	MAX_BW_40MHZ,
	MAX_BW_80MHZ,
	MAX_BW_160MHZ,
	MAX_BW_80_80_MHZ,
	MAX_BW_UNKNOWN
};

struct TX_PACKET_INFO {
	uint8_t ucPriorityParam;
	uint32_t u4PacketLen;
	uint8_t aucEthDestAddr[MAC_ADDR_LEN];
	uint16_t u2Flag;

#if 0
	u_int8_t fgIs1X;
	u_int8_t fgIsPAL;
	u_int8_t fgIs802_3;
	u_int8_t fgIsVlanExists;
	u_int8_t fgIsDhcp;
	u_int8_t fgIsArp;
#endif
};

enum ENUM_TX_PROFILING_TAG {
	TX_PROF_TAG_OS_TO_DRV = 0,
	TX_PROF_TAG_DRV_ENQUE,
	TX_PROF_TAG_DRV_DEQUE,
	TX_PROF_TAG_DRV_TX_DONE,
	TX_PROF_TAG_DRV_FREE_MSDU,
	TX_PROF_TAG_MAC_TX_DONE
};

#if (CFG_SUPPORT_SPE_IDX_CONTROL == 1)
enum ENUM_WF_PATH_FAVOR_T {
	ENUM_WF_NON_FAVOR = 0xff,
	ENUM_WF_0_ONE_STREAM_PATH_FAVOR = 0,
	ENUM_WF_1_ONE_STREAM_PATH_FAVOR = 1,
	ENUM_WF_0_1_TWO_STREAM_PATH_FAVOR = 2,
	ENUM_WF_0_1_DUP_STREAM_PATH_FAVOR = 3,
};
#endif

struct PARAM_GET_CNM_T {
	uint8_t	fgIsDbdcEnable;

	uint8_t	ucOpChNum[ENUM_BAND_NUM];
	uint8_t	ucChList[ENUM_BAND_NUM][MAX_OP_CHNL_NUM];
	uint8_t	ucChBw[ENUM_BAND_NUM][MAX_OP_CHNL_NUM];
	uint8_t	ucChSco[ENUM_BAND_NUM][MAX_OP_CHNL_NUM];
	uint8_t	ucChNetNum[ENUM_BAND_NUM][MAX_OP_CHNL_NUM];
	uint8_t	ucChBssList[ENUM_BAND_NUM][MAX_OP_CHNL_NUM][BSSID_NUM];

	uint8_t	ucBssInuse[BSSID_NUM + 1];
	uint8_t	ucBssActive[BSSID_NUM + 1];
	uint8_t	ucBssConnectState[BSSID_NUM + 1];

	uint8_t	ucBssCh[BSSID_NUM + 1];
	uint8_t	ucBssDBDCBand[BSSID_NUM + 1];
	uint8_t	ucBssWmmSet[BSSID_NUM + 1];
	uint8_t	ucBssWmmDBDCBand[BSSID_NUM + 1];
	uint8_t	ucBssOMACSet[BSSID_NUM + 1];
	uint8_t	ucBssOMACDBDCBand[BSSID_NUM + 1];

	/* Reserved fields */
	uint8_t	au4Reserved[65]; /*Total 160 byte*/
};

/*******************************************************************************
 *                            P U B L I C   D A T A
 *******************************************************************************
 */
extern struct MIB_INFO_STAT g_arMibInfo[ENUM_BAND_NUM];

/*******************************************************************************
 *                           P R I V A T E   D A T A
 *******************************************************************************
 */

/*******************************************************************************
 *                                 M A C R O S
 *******************************************************************************
 */
#define BUILD_SIGN(ch0, ch1, ch2, ch3) \
	((uint32_t)(uint8_t)(ch0) | ((uint32_t)(uint8_t)(ch1) << 8) |   \
	((uint32_t)(uint8_t)(ch2) << 16) | ((uint32_t)(uint8_t)(ch3) << 24))

#define MTK_WIFI_SIGNATURE BUILD_SIGN('M', 'T', 'K', 'W')

#define IS_FEATURE_ENABLED(_ucFeature) \
	(((_ucFeature) == FEATURE_ENABLED) || \
	((_ucFeature) == FEATURE_FORCE_ENABLED))
#define IS_FEATURE_FORCE_ENABLED(_ucFeature) \
	((_ucFeature) == FEATURE_FORCE_ENABLED)
#define IS_FEATURE_DISABLED(_ucFeature) ((_ucFeature) == FEATURE_DISABLED)

/* This macro is for later added feature options which use command reserved
 * field as option switch
 */
/* 0: AUTO
 * 1: Disabled
 * 2: Enabled
 * 3: Force disabled
 */
#define FEATURE_OPT_IN_COMMAND(_ucFeature) ((_ucFeature) + 1)

/*******************************************************************************
 *                   F U N C T I O N   D E C L A R A T I O N S
 *******************************************************************************
 */

struct ADAPTER *wlanAdapterCreate(IN struct GLUE_INFO *prGlueInfo);

void wlanAdapterDestroy(IN struct ADAPTER *prAdapter);

void wlanCardEjected(IN struct ADAPTER *prAdapter);

void wlanIST(IN struct ADAPTER *prAdapter);

u_int8_t wlanISR(IN struct ADAPTER *prAdapter, IN u_int8_t fgGlobalIntrCtrl);

uint32_t wlanProcessCommandQueue(IN struct ADAPTER *prAdapter,
				 IN struct QUE *prCmdQue);

uint32_t wlanSendCommand(IN struct ADAPTER *prAdapter,
			 IN struct CMD_INFO *prCmdInfo);

#if CFG_SUPPORT_MULTITHREAD
uint32_t wlanSendCommandMthread(IN struct ADAPTER *prAdapter,
				IN struct CMD_INFO *prCmdInfo);

uint32_t wlanTxCmdMthread(IN struct ADAPTER *prAdapter);

uint32_t wlanTxCmdDoneMthread(IN struct ADAPTER *prAdapter);

void wlanClearTxCommandQueue(IN struct ADAPTER *prAdapter);

void wlanClearTxCommandDoneQueue(IN struct ADAPTER *prAdapter);

void wlanClearDataQueue(IN struct ADAPTER *prAdapter);

void wlanClearRxToOsQueue(IN struct ADAPTER *prAdapter);
#endif

void wlanClearPendingCommandQueue(IN struct ADAPTER *prAdapter);

void wlanReleaseCommand(IN struct ADAPTER *prAdapter,
			IN struct CMD_INFO *prCmdInfo,
			IN enum ENUM_TX_RESULT_CODE rTxDoneStatus);

void wlanReleasePendingOid(IN struct ADAPTER *prAdapter,
			   IN unsigned long ulParamPtr);

void wlanReleasePendingCMDbyBssIdx(IN struct ADAPTER *prAdapter,
				   IN uint8_t ucBssIndex);

void wlanReturnPacketDelaySetupTimeout(IN struct ADAPTER *prAdapter,
				       IN unsigned long ulParamPtr);

void wlanReturnPacket(IN struct ADAPTER *prAdapter, IN void *pvPacket);

uint32_t
wlanQueryInformation(IN struct ADAPTER *prAdapter,
		     IN PFN_OID_HANDLER_FUNC pfOidQryHandler,
		     IN void *pvInfoBuf, IN uint32_t u4InfoBufLen,
		     OUT uint32_t *pu4QryInfoLen);

uint32_t
wlanSetInformation(IN struct ADAPTER *prAdapter,
		   IN PFN_OID_HANDLER_FUNC pfOidSetHandler,
		   IN void *pvInfoBuf, IN uint32_t u4InfoBufLen,
		   OUT uint32_t *pu4SetInfoLen);

uint32_t wlanAdapterStart(IN struct ADAPTER *prAdapter,
			  IN struct REG_INFO *prRegInfo);

uint32_t wlanAdapterStop(IN struct ADAPTER *prAdapter);

uint32_t wlanCheckWifiFunc(IN struct ADAPTER *prAdapter,
			   IN u_int8_t fgRdyChk);

#if CFG_SUPPORT_WAPI
u_int8_t wlanQueryWapiMode(IN struct ADAPTER *prAdapter);
#endif

void wlanReturnRxPacket(IN void *pvAdapter, IN void *pvPacket);

void wlanRxSetBroadcast(IN struct ADAPTER *prAdapter,
			IN u_int8_t fgEnableBroadcast);

u_int8_t wlanIsHandlerNeedHwAccess(IN PFN_OID_HANDLER_FUNC pfnOidHandler,
				   IN u_int8_t fgSetInfo);

void wlanSetPromiscuousMode(IN struct ADAPTER *prAdapter,
			    IN u_int8_t fgEnablePromiscuousMode);

uint32_t wlanSendDummyCmd(IN struct ADAPTER *prAdapter,
			  IN u_int8_t fgIsReqTxRsrc);

uint32_t wlanSendNicPowerCtrlCmd(IN struct ADAPTER *prAdapter,
				 IN uint8_t ucPowerMode);

u_int8_t wlanIsHandlerAllowedInRFTest(IN PFN_OID_HANDLER_FUNC pfnOidHandler,
				      IN u_int8_t fgSetInfo);

uint32_t wlanProcessQueuedSwRfb(IN struct ADAPTER *prAdapter,
				IN struct SW_RFB *prSwRfbListHead);

uint32_t wlanProcessQueuedMsduInfo(IN struct ADAPTER *prAdapter,
				   IN struct MSDU_INFO *prMsduInfoListHead);

u_int8_t wlanoidTimeoutCheck(IN struct ADAPTER *prAdapter,
			     IN PFN_OID_HANDLER_FUNC pfnOidHandler);

void wlanoidClearTimeoutCheck(IN struct ADAPTER *prAdapter);

uint32_t wlanUpdateNetworkAddress(IN struct ADAPTER *prAdapter);

uint32_t wlanUpdateBasicConfig(IN struct ADAPTER *prAdapter);

u_int8_t wlanQueryTestMode(IN struct ADAPTER *prAdapter);

u_int8_t wlanProcessTxFrame(IN struct ADAPTER *prAdapter,
			    IN void *prPacket);

/* Security Frame Handling */
u_int8_t wlanProcessSecurityFrame(IN struct ADAPTER *prAdapter,
				  IN void *prPacket);

void wlanSecurityFrameTxDone(IN struct ADAPTER *prAdapter,
			     IN struct CMD_INFO *prCmdInfo,
			     IN uint8_t *pucEventBuf);

void wlanSecurityFrameTxTimeout(IN struct ADAPTER *prAdapter,
				IN struct CMD_INFO *prCmdInfo);

#if CFG_SUPPORT_CSI
u_int8_t
wlanPushCSIData(IN struct ADAPTER *prAdapter, struct CSI_DATA_T *prCSIData);
u_int8_t
wlanPopCSIData(IN struct ADAPTER *prAdapter, struct CSI_DATA_T *prCSIData);
void
wlanApplyCSIToneMask(
	uint8_t ucRxMode,
	uint8_t ucCBW,
	uint8_t ucDBW,
	uint8_t ucPrimaryChIdx,
	int16_t *ai2IData,
	int16_t *ai2QData);

void
wlanShiftCSI(
	uint8_t ucRxMode,
	uint8_t ucCBW,
	uint8_t ucDBW,
	uint8_t ucPrimaryChIdx,
	int16_t *ai2IData,
	int16_t *ai2QData,
	int16_t *ai2ShiftIData,
	int16_t *ai2ShiftQData);
#endif

/*----------------------------------------------------------------------------*/
/* OID/IOCTL Handling                                                         */
/*----------------------------------------------------------------------------*/
void wlanClearScanningResult(IN struct ADAPTER *prAdapter);

void wlanClearBssInScanningResult(IN struct ADAPTER *prAdapter,
				  IN uint8_t *arBSSID);

#if CFG_TEST_WIFI_DIRECT_GO
void wlanEnableP2pFunction(IN struct ADAPTER *prAdapter);

void wlanEnableATGO(IN struct ADAPTER *prAdapter);
#endif

/*----------------------------------------------------------------------------*/
/* NIC Capability Retrieve by Polling                                         */
/*----------------------------------------------------------------------------*/
uint32_t wlanQueryNicCapability(IN struct ADAPTER *prAdapter);

/*----------------------------------------------------------------------------*/
/* PD MCR Retrieve by Polling                                                 */
/*----------------------------------------------------------------------------*/
uint32_t wlanQueryPdMcr(IN struct ADAPTER *prAdapter,
			IN struct PARAM_MCR_RW_STRUCT *prMcrRdInfo);

/*----------------------------------------------------------------------------*/
/* Loading Manufacture Data                                                   */
/*----------------------------------------------------------------------------*/
uint32_t wlanLoadManufactureData(IN struct ADAPTER *prAdapter,
				 IN struct REG_INFO *prRegInfo);

/*----------------------------------------------------------------------------*/
/* Media Stream Mode                                                          */
/*----------------------------------------------------------------------------*/
u_int8_t wlanResetMediaStreamMode(IN struct ADAPTER *prAdapter);

/*----------------------------------------------------------------------------*/
/* Timer Timeout Check (for Glue Layer)                                       */
/*----------------------------------------------------------------------------*/
uint32_t wlanTimerTimeoutCheck(IN struct ADAPTER *prAdapter);

/*----------------------------------------------------------------------------*/
/* Mailbox Message Check (for Glue Layer)                                     */
/*----------------------------------------------------------------------------*/
uint32_t wlanProcessMboxMessage(IN struct ADAPTER *prAdapter);

/*----------------------------------------------------------------------------*/
/* TX Pending Packets Handling (for Glue Layer)                               */
/*----------------------------------------------------------------------------*/
uint32_t wlanEnqueueTxPacket(IN struct ADAPTER *prAdapter,
			     IN void *prNativePacket);

uint32_t wlanFlushTxPendingPackets(IN struct ADAPTER *prAdapter);

uint32_t wlanTxPendingPackets(IN struct ADAPTER *prAdapter,
			      IN OUT u_int8_t *pfgHwAccess);

/*----------------------------------------------------------------------------*/
/* Low Power Acquire/Release (for Glue Layer)                                 */
/*----------------------------------------------------------------------------*/
uint32_t wlanAcquirePowerControl(IN struct ADAPTER *prAdapter);

uint32_t wlanReleasePowerControl(IN struct ADAPTER *prAdapter);

/*----------------------------------------------------------------------------*/
/* Pending Packets Number Reporting (for Glue Layer)                          */
/*----------------------------------------------------------------------------*/
uint32_t wlanGetTxPendingFrameCount(IN struct ADAPTER *prAdapter);

/*----------------------------------------------------------------------------*/
/* ACPI state inquiry (for Glue Layer)                                        */
/*----------------------------------------------------------------------------*/
enum ENUM_ACPI_STATE wlanGetAcpiState(IN struct ADAPTER *prAdapter);

void wlanSetAcpiState(IN struct ADAPTER *prAdapter,
		      IN enum ENUM_ACPI_STATE ePowerState);

void wlanDefTxPowerCfg(IN struct ADAPTER *prAdapter);

/*----------------------------------------------------------------------------*/
/* get ECO version from Revision ID register (for Win32)                      */
/*----------------------------------------------------------------------------*/
uint8_t wlanGetEcoVersion(IN struct ADAPTER *prAdapter);

/*----------------------------------------------------------------------------*/
/* get Rom version                     */
/*----------------------------------------------------------------------------*/
uint8_t wlanGetRomVersion(IN struct ADAPTER *prAdapter);
/*----------------------------------------------------------------------------*/
/* set preferred band configuration corresponding to network type             */
/*----------------------------------------------------------------------------*/
void wlanSetPreferBandByNetwork(IN struct ADAPTER *prAdapter,
				IN enum ENUM_BAND eBand, IN uint8_t ucBssIndex);

/*----------------------------------------------------------------------------*/
/* get currently operating channel information                                */
/*----------------------------------------------------------------------------*/
uint8_t wlanGetChannelNumberByNetwork(IN struct ADAPTER *prAdapter,
				      IN uint8_t ucBssIndex);

/*----------------------------------------------------------------------------*/
/* check for system configuration to generate message on scan list            */
/*----------------------------------------------------------------------------*/
uint32_t wlanCheckSystemConfiguration(IN struct ADAPTER *prAdapter);

/*----------------------------------------------------------------------------*/
/* query bss statistics information from driver and firmware                  */
/*----------------------------------------------------------------------------*/
uint32_t
wlanoidQueryBssStatistics(IN struct ADAPTER *prAdapter,
			  IN void *pvQueryBuffer, IN uint32_t u4QueryBufferLen,
			  OUT uint32_t *pu4QueryInfoLen);

/*----------------------------------------------------------------------------*/
/* dump per-BSS statistics            */
/*----------------------------------------------------------------------------*/
void wlanDumpBssStatistics(IN struct ADAPTER *prAdapter, uint8_t ucBssIndex);

/*----------------------------------------------------------------------------*/
/* query sta statistics information from driver and firmware                  */
/*----------------------------------------------------------------------------*/
uint32_t
wlanoidQueryStaStatistics(IN struct ADAPTER *prAdapter,
			  IN void *pvQueryBuffer, IN uint32_t u4QueryBufferLen,
			  OUT uint32_t *pu4QueryInfoLen);

uint32_t
wlanQueryStaStatistics(IN struct ADAPTER *prAdapter, IN void *pvQueryBuffer,
		       IN uint32_t u4QueryBufferLen,
		       OUT uint32_t *pu4QueryInfoLen,
		       u_int8_t fgIsOid);

/*----------------------------------------------------------------------------*/
/* query NIC resource information from chip and reset Tx resource for normal  */
/* operation                                                                  */
/*----------------------------------------------------------------------------*/
void wlanQueryNicResourceInformation(IN struct ADAPTER *prAdapter);

uint32_t wlanQueryNicCapabilityV2(IN struct ADAPTER *prAdapter);

void wlanUpdateNicResourceInformation(IN struct ADAPTER *prAdapter);

/*----------------------------------------------------------------------------*/
/* GET/SET BSS index mapping for network interfaces                           */
/*----------------------------------------------------------------------------*/
void wlanBindNetInterface(IN struct GLUE_INFO *prGlueInfo,
			  IN uint8_t ucNetInterfaceIndex,
			  IN void *pvNetInterface);

void wlanBindBssIdxToNetInterface(IN struct GLUE_INFO *prGlueInfo,
				  IN uint8_t ucBssIndex,
				  IN void *pvNetInterface);

uint8_t wlanGetBssIdxByNetInterface(IN struct GLUE_INFO *prGlueInfo,
				    IN void *pvNetInterface);

void *wlanGetNetInterfaceByBssIdx(IN struct GLUE_INFO *prGlueInfo,
				  IN uint8_t ucBssIndex);

/* for windows as windows glue cannot see through P_ADAPTER_T */
uint8_t wlanGetAisBssIndex(IN struct ADAPTER *prAdapter);

void wlanInitFeatureOption(IN struct ADAPTER *prAdapter);

void wlanCfgSetSwCtrl(IN struct ADAPTER *prAdapter);

void wlanCfgSetChip(IN struct ADAPTER *prAdapter);

void wlanCfgSetDebugLevel(IN struct ADAPTER *prAdapter);

void wlanCfgSetCountryCode(IN struct ADAPTER *prAdapter);

struct WLAN_CFG_ENTRY *wlanCfgGetEntry(IN struct ADAPTER *prAdapter,
				       const int8_t *pucKey,
				       u_int8_t fgGetCfgRec);

uint32_t
wlanCfgGet(IN struct ADAPTER *prAdapter, const int8_t *pucKey, int8_t *pucValue,
	   int8_t *pucValueDef, uint32_t u4Flags);

uint32_t wlanCfgGetUint32(IN struct ADAPTER *prAdapter, const int8_t *pucKey,
			  uint32_t u4ValueDef);

int32_t wlanCfgGetInt32(IN struct ADAPTER *prAdapter, const int8_t *pucKey,
			int32_t i4ValueDef);

uint32_t wlanCfgSetUint32(IN struct ADAPTER *prAdapter, const int8_t *pucKey,
			  uint32_t u4Value);

uint32_t wlanCfgSet(IN struct ADAPTER *prAdapter, const int8_t *pucKey,
		    int8_t *pucValue, uint32_t u4Flags);

uint32_t wlanCfgSetCb(IN struct ADAPTER *prAdapter, const int8_t *pucKey,
		      WLAN_CFG_SET_CB pfSetCb, void *pPrivate,
		      uint32_t u4Flags);

#if CFG_SUPPORT_EASY_DEBUG

uint32_t wlanCfgParse(IN struct ADAPTER *prAdapter, uint8_t *pucConfigBuf,
		      uint32_t u4ConfigBufLen, u_int8_t isFwConfig);
void wlanFeatureToFw(IN struct ADAPTER *prAdapter, const uint8_t *pucKey,
			bool partial_update);
#endif

void wlanLoadDefaultCustomerSetting(IN struct ADAPTER *prAdapter);

uint32_t wlanCfgInit(IN struct ADAPTER *prAdapter, uint8_t *pucConfigBuf,
		     uint32_t u4ConfigBufLen, uint32_t u4Flags);

uint32_t wlanCfgParseArgument(int8_t *cmdLine, int32_t *argc, int8_t *argv[]);

#if CFG_WOW_SUPPORT
uint32_t wlanCfgParseArgumentLong(int8_t *cmdLine, int32_t *argc,
				  int8_t *argv[]);
#endif

int32_t wlanHexToNum(int8_t c);

int32_t wlanHexToByte(int8_t *hex);

int32_t wlanHwAddrToBin(int8_t *txt, uint8_t *addr);

u_int8_t wlanIsChipNoAck(IN struct ADAPTER *prAdapter);

u_int8_t wlanIsChipRstRecEnabled(IN struct ADAPTER *prAdapter);

u_int8_t wlanIsChipAssert(IN struct ADAPTER *prAdapter);

void wlanChipRstPreAct(IN struct ADAPTER *prAdapter);

void wlanTxProfilingTagPacket(IN struct ADAPTER *prAdapter, IN void *prPacket,
			      IN enum ENUM_TX_PROFILING_TAG eTag);

void wlanTxProfilingTagMsdu(IN struct ADAPTER *prAdapter,
			    IN struct MSDU_INFO *prMsduInfo,
			    IN enum ENUM_TX_PROFILING_TAG eTag);

void wlanTxLifetimeTagPacket(IN struct ADAPTER *prAdapter,
			     IN struct MSDU_INFO *prMsduInfoListHead,
			     IN enum ENUM_TX_PROFILING_TAG eTag);

#if CFG_ASSERT_DUMP
void wlanCorDumpTimerInit(IN struct ADAPTER *prAdapter, u_int8_t fgIsResetN9);

void wlanCorDumpTimerReset(IN struct ADAPTER *prAdapter, u_int8_t fgIsResetN9);

void wlanN9CorDumpTimeOut(IN struct ADAPTER *prAdapter,
			  IN unsigned long ulParamPtr);

void wlanCr4CorDumpTimeOut(IN struct ADAPTER *prAdapter,
			   IN unsigned long ulParamPtr);
#endif
#endif /* _WLAN_LIB_H */


u_int8_t wlanGetWlanIdxByAddress(IN struct ADAPTER *prAdapter,
				 IN uint8_t *pucAddr, OUT uint8_t *pucIndex);

uint8_t *wlanGetStaAddrByWlanIdx(IN struct ADAPTER *prAdapter,
				 IN uint8_t ucIndex);

struct WLAN_CFG_ENTRY *wlanCfgGetEntryByIndex(IN struct ADAPTER *prAdapter,
					      const uint8_t ucIdx,
					      uint32_t flag);

uint32_t wlanGetStaIdxByWlanIdx(IN struct ADAPTER *prAdapter,
				IN uint8_t ucIndex, OUT uint8_t *pucStaIdx);

/*----------------------------------------------------------------------------*/
/* update per-AC statistics for LLS                */
/*----------------------------------------------------------------------------*/
void wlanUpdateTxStatistics(IN struct ADAPTER *prAdapter,
			    IN struct MSDU_INFO *prMsduInfo, u_int8_t fgTxDrop);

void wlanUpdateRxStatistics(IN struct ADAPTER *prAdapter,
			    IN struct SW_RFB *prSwRfb);

uint32_t wlanTriggerStatsLog(IN struct ADAPTER *prAdapter,
			     IN uint32_t u4DurationInMs);

uint32_t wlanPktTxDone(IN struct ADAPTER *prAdapter,
		       IN struct MSDU_INFO *prMsduInfo,
		       IN enum ENUM_TX_RESULT_CODE rTxDoneStatus);

uint32_t wlanPowerOffWifi(IN struct ADAPTER *prAdapter);

void wlanPrintVersion(struct ADAPTER *prAdapter);

uint32_t wlanAccessRegister(IN struct ADAPTER *prAdapter,
			    IN uint32_t u4Addr, IN uint32_t *pru4Result,
			    IN uint32_t u4Data,
			    IN uint8_t ucSetQuery);

uint32_t wlanAccessRegisterStatus(IN struct ADAPTER
				  *prAdapter, IN uint8_t ucCmdSeqNum,
				  IN uint8_t ucSetQuery, IN void *prEvent,
				  IN uint32_t u4EventLen);

uint32_t wlanSetChipEcoInfo(IN struct ADAPTER *prAdapter);

void wlanNotifyFwSuspend(struct GLUE_INFO *prGlueInfo,
			 struct net_device *prDev, u_int8_t fgSuspend);

void wlanClearPendingInterrupt(IN struct ADAPTER *prAdapter);

#if (MTK_WCN_HIF_SDIO && CFG_WMT_WIFI_PATH_SUPPORT)
extern int32_t mtk_wcn_wmt_wifi_fem_cfg_report(void *pvInfoBuf);
#endif

#if ((CFG_SISO_SW_DEVELOP == 1) || (CFG_SUPPORT_SPE_IDX_CONTROL == 1))
uint8_t wlanGetAntPathType(IN struct ADAPTER *prAdapter,
			   IN enum ENUM_WF_PATH_FAVOR_T eWfPathFavor,
			   IN uint8_t ucBssIndex);
#endif

uint8_t wlanGetSpeIdx(IN struct ADAPTER *prAdapter,
		      IN uint8_t ucBssIndex,
		      IN enum ENUM_WF_PATH_FAVOR_T eWfPathFavor);

uint8_t wlanGetSupportNss(IN struct ADAPTER *prAdapter, IN uint8_t ucBssIndex);

#if CFG_SUPPORT_LOWLATENCY_MODE
uint32_t wlanAdapterStartForLowLatency(IN struct ADAPTER *prAdapter);
#endif /* CFG_SUPPORT_LOWLATENCY_MODE */
int32_t wlanGetFileContent(struct ADAPTER *prAdapter,
	const uint8_t *pcFileName, uint8_t *pucBuf,
	uint32_t u4MaxFileLen, uint32_t *pu4ReadFileLen, u_int8_t bReqFw);

void wlanReleasePendingCmdById(struct ADAPTER *prAdapter, uint8_t ucCid);
void wlanReleaseAllTxCmdQueue(struct ADAPTER *prAdapter);
void wlanWaitCfg80211SuspendDone(struct GLUE_INFO *prGlueInfo);

uint32_t wlanDecimalStr2Hexadecimals(uint8_t *pucDecimalStr, uint16_t *pu2Out);
uint32_t wlanGetSupportedFeatureSet(IN struct GLUE_INFO *prGlueInfo);

uint32_t wlanSetEd(IN struct ADAPTER *prAdapter, int32_t u4EdVal2G,
					int32_t u4EdVal5G, uint32_t u4Sel);
u_int8_t wlanIsDriverReady(IN struct GLUE_INFO *prGlueInfo);
int wlanSuspendRekeyOffload(struct GLUE_INFO *prGlueInfo, uint8_t ucRekeyMode);
void wlanSuspendPmHandle(struct GLUE_INFO *prGlueInfo);
void wlanResumePmHandle(struct GLUE_INFO *prGlueInfo);
uint32_t wlanWakeUpWiFi(IN struct ADAPTER *prAdapter);

void disconnect_sta(struct ADAPTER *prAdapter, struct STA_RECORD *prStaRec);

uint32_t wlanData2RateInMs(uint32_t data, uint32_t interval);
#if CFG_RCPI_COMPENSATION
void wlanLoadEfuseRxFELoss(struct ADAPTER *prAdapter);
void wlanLoadBufferbinRxFELoss(struct ADAPTER *prAdapter);
void wlanUpdateRxFELoss(IN struct SW_RFB *prSwRfb);
uint8_t wlanGetCurrChRxFELoss(struct ADAPTER *prAdapter,
					uint8_t ucStaIdx, uint8_t ucAnt);
#endif
#if CFG_SUPPORT_HW_1T2R
void wlanLoadEfuse1T2R(struct ADAPTER *prAdapter);
void wlanLoadBufferbin1T2R(struct ADAPTER *prAdapter);
#endif

#if CFG_SUPPORT_ADVANCE_CONTROL
uint32_t wlanKeepFullPwr(struct ADAPTER *prAdapter, uint8_t fgEnable);
#endif

#if CFG_SUPPORT_GET_MCS_INFO
void wlanRxMcsInfoMonitor(struct ADAPTER *prAdapter,
					    unsigned long ulParamPtr);
#endif

#if CFG_SUPPORT_TPENHANCE_MODE
inline uint64_t wlanTpeTimeUs(void);
void wlanTpeUpdate(struct GLUE_INFO *prGlueInfo, struct QUE *prSrcQue,
		uint8_t ucPktJump);
void wlanTpeFlush(struct GLUE_INFO *prGlueInfo);
#if KERNEL_VERSION(4, 14, 0) <= LINUX_VERSION_CODE
void wlanTpeTimeoutHandler(struct timer_list *timer);
#else
void wlanTpeTimeoutHandler(unsigned long data);
#endif
void wlanTpeInit(struct GLUE_INFO *prGlueInfo);
void wlanTpeUninit(struct GLUE_INFO *prGlueInfo);
int wlanTpeProcess(struct GLUE_INFO *prGlueInfo,
			struct sk_buff *prSkb,
			struct net_device *prDev);
#endif /* CFG_SUPPORT_TPENHANCE_MODE */
