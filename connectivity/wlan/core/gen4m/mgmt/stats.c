/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

/*******************************************************************************
 *            C O M P I L E R	 F L A G S
 *******************************************************************************
 */

/*******************************************************************************
 *            E X T E R N A L	R E F E R E N C E S
 *******************************************************************************
 */
#include "precomp.h"

#if (CFG_SUPPORT_STATISTICS == 1)

enum EVENT_TYPE {
	EVENT_RX,
	EVENT_TX,
};
/*******************************************************************************
 *            C O N S T A N T S
 *******************************************************************************
 */

/*******************************************************************************
 *            F U N C T I O N   D E C L A R A T I O N S
 *******************************************************************************
 */

/*******************************************************************************
 *            P U B L I C   D A T A
 *******************************************************************************
 */
/* function pointer array for tx stats*/
static struct STATS_TLV_MAP_T apfnTxTbl[STATS_TX_TAG_MAX_NUM] = {
	{STATS_TX_TAG_QUEUE, {statsTxGetQueueLen, statsTxQueueHdlr} },
	{STATS_TX_TAG_RETRY, {statsTxGetRetryLen, statsTxGetRetryHdlr} },
	{STATS_TX_TAG_TIME, {statsTxGetTimeLen, statsTxTimeHdlr} },
	{STATS_TX_TAG_LAT, {statsTxGetLatLen, statsTxLatHdlr} },
};

static struct STATS_TLV_MAP_T apfnRxTbl[STATS_RX_TAG_MAX_NUM] = {
	{STATS_RX_TAG_REORDER_DROP,
		{statsGetTlvU8Len, statsRxReorderDropHdlr} },
	{STATS_RX_TAG_AVG_RSSI,
		{statsRxGetAvgRssiLen, statsRxAvgRssiHdlr} },
};

static struct STATS_TLV_MAP_T apfnCgsTbl[STATS_CGS_TAG_MAX_NUM] = {
	{STATS_CGS_TAG_B0_IDLE_SLOT,
		{statsGetTlvU8Len, statsCgsB0IdleSlotHdlr} },
	{STATS_CGS_TAG_AIR_LAT,
		{statsCgsGetAirLatLen, statsCgsAirLatHdlr} },
};

/*******************************************************************************
 *            P R I V A T E  F U N C T I O N S
 *******************************************************************************
 */

uint32_t u4TotalTx;
uint32_t u4NoDelayTx;
uint32_t u4TotalRx;
uint32_t u4NoDelayRx;

static uint8_t g_ucTxRxFlag;
static uint8_t g_ucTxIpProto;
static uint16_t g_u2TxUdpPort;
static uint32_t g_u4TxDelayThreshold;
static uint8_t g_ucRxIpProto;
static uint16_t g_u2RxUdpPort;
static uint32_t g_u4RxDelayThreshold;

void StatsResetTxRx(void)
{
	u4TotalRx = 0;
	u4TotalTx = 0;
	u4NoDelayRx = 0;
	u4NoDelayTx = 0;
}

uint64_t StatsEnvTimeGet(void)
{
	uint64_t u8Clk;
	u8Clk = kalGetTimeTickNs(); /* unit: naro seconds */
	return (uint64_t) u8Clk;	/* sched_clock *//* jiffies size = 4B */
}

void StatsEnvGetPktDelay(uint8_t *pucTxRxFlag,
	uint8_t *pucTxIpProto, uint16_t *pu2TxUdpPort,
	uint32_t *pu4TxDelayThreshold, uint8_t *pucRxIpProto,
	uint16_t *pu2RxUdpPort, uint32_t *pu4RxDelayThreshold)
{
	*pucTxRxFlag = g_ucTxRxFlag;
	*pucTxIpProto = g_ucTxIpProto;
	*pu2TxUdpPort = g_u2TxUdpPort;
	*pu4TxDelayThreshold = g_u4TxDelayThreshold;
	*pucRxIpProto = g_ucRxIpProto;
	*pu2RxUdpPort = g_u2RxUdpPort;
	*pu4RxDelayThreshold = g_u4RxDelayThreshold;
}

void StatsEnvSetPktDelay(uint8_t ucTxOrRx, uint8_t ucIpProto,
	uint16_t u2UdpPort, uint32_t u4DelayThreshold)
{
#define MODULE_RESET 0
#define MODULE_TX 1
#define MODULE_RX 2

	if (ucTxOrRx == MODULE_TX) {
		g_ucTxRxFlag |= BIT(0);
		g_ucTxIpProto = ucIpProto;
		g_u2TxUdpPort = u2UdpPort;
		g_u4TxDelayThreshold = u4DelayThreshold;
	} else if (ucTxOrRx == MODULE_RX) {
		g_ucTxRxFlag |= BIT(1);
		g_ucRxIpProto = ucIpProto;
		g_u2RxUdpPort = u2UdpPort;
		g_u4RxDelayThreshold = u4DelayThreshold;
	} else if (ucTxOrRx == MODULE_RESET) {
		g_ucTxRxFlag = 0;
		g_ucTxIpProto = 0;
		g_u2TxUdpPort = 0;
		g_u4TxDelayThreshold = 0;
		g_ucRxIpProto = 0;
		g_u2RxUdpPort = 0;
		g_u4RxDelayThreshold = 0;
	}
}

void StatsEnvRxTime2Host(struct ADAPTER *prAdapter,
	void *pvPacket, void *prNetDev)
{
	uint16_t u2EthType = 0;
	uint8_t ucIpVersion = 0;
	uint8_t ucIpProto = 0;
	uint16_t u2IPID = 0;
	uint8_t *pucEth = NULL;
	uint16_t u2UdpDstPort = 0;
	uint16_t u2UdpSrcPort = 0;
	uint64_t u8IntTime = 0;
	uint64_t u8RxTime = 0;
	uint32_t u4Delay = 0;
	OS_SYSTIME rCurrentTime;
	uint32_t rCurrentSec;

	kalGetPacketBuf(pvPacket, &pucEth);
	u2EthType = (pucEth[ETH_TYPE_LEN_OFFSET] << 8)
		| (pucEth[ETH_TYPE_LEN_OFFSET + 1]);
	pucEth += ETH_HLEN;
	u2IPID = pucEth[4] << 8 | pucEth[5];

	DBGLOG(RX, TEMP, "u2IpId=%d rx_packets=%u\n",
		u2IPID, kalGetNetDevRxPacket(prNetDev));

	if ((g_ucTxRxFlag & BIT(1)) == 0)
		return;

	if (kalQueryPacketLength(pvPacket) <= 24 + ETH_HLEN)
		return;

	if (u2EthType != ETH_P_IPV4)
		return;

	ucIpProto = pucEth[9];
	if (g_ucRxIpProto && (ucIpProto != g_ucRxIpProto))
		return;
	ucIpVersion = (pucEth[0] & IPVH_VERSION_MASK) >> IPVH_VERSION_OFFSET;
	if (ucIpVersion != IPVERSION)
		return;
	u2IPID = pucEth[4] << 8 | pucEth[5];
	u8IntTime = GLUE_RX_GET_PKT_INT_TIME(pvPacket);
	u4Delay = ((uint32_t)(kalGetTimeTickNs() - u8IntTime))/NSEC_PER_USEC;
	u8RxTime = GLUE_RX_GET_PKT_RX_TIME(pvPacket);
	rCurrentTime = kalGetTimeTick();
	rCurrentSec = SYSTIME_TO_SEC(rCurrentTime);


	switch (ucIpProto) {
	case IP_PRO_TCP:
	case IP_PRO_UDP:
		u2UdpSrcPort = (pucEth[20] << 8) | pucEth[21];
		u2UdpDstPort = (pucEth[22] << 8) | pucEth[23];
		if (g_u2RxUdpPort && (u2UdpSrcPort != g_u2RxUdpPort))
			break;
		kal_fallthrough;
	case IP_PRO_ICMP:
		u4TotalRx++;
		if (g_u4RxDelayThreshold && (u4Delay <= g_u4RxDelayThreshold)) {
			u4NoDelayRx++;
			break;
		}
		DBGLOG(RX, INFO,
	"IPID 0x%04x src %d dst %d UP %d,delay %u us,int2rx %u us,IntTime %llu,%u/%u,leave at %02d:%02d:%02d.%06u\n",
			u2IPID, u2UdpSrcPort, u2UdpDstPort,
			((pucEth[1] & IPTOS_PREC_MASK) >> IPTOS_PREC_OFFSET),
			u4Delay,
			((uint32_t)(u8RxTime - u8IntTime))/NSEC_PER_USEC,
			u8IntTime, u4NoDelayRx, u4TotalRx,
			SEC_TO_TIME_HOUR(rCurrentSec),
			SEC_TO_TIME_MINUTE(rCurrentSec),
			SEC_TO_TIME_SECOND(rCurrentSec),
			SYSTIME_TO_USEC(rCurrentTime) % USEC_PER_SEC);
		break;
	default:
		break;
	}
}

void StatsEnvTxTime2Hif(struct ADAPTER *prAdapter,
	struct MSDU_INFO *prMsduInfo)
{
	uint64_t u8SysTime, u8SysTimeIn;
	uint32_t u4TimeDiff;
	uint8_t *pucEth;
	uint32_t u4PacketLen;
	uint8_t ucIpVersion = 0;
	uint8_t ucIpProto = 0;
	uint8_t *pucEthBody = NULL;
	uint16_t u2EthType = 0;
	uint8_t *pucAheadBuf = NULL;
	uint16_t u2IPID = 0;
	uint16_t u2UdpDstPort = 0;
	uint16_t u2UdpSrcPort = 0;

	if (prMsduInfo == NULL) {
		DBGLOG(TX, ERROR, "prMsduInfo=NULL");
		return;
	}

	if (prMsduInfo->prPacket == NULL) {
		DBGLOG(TX, ERROR, "prMsduInfo->prPacket=NULL");
		return;
	}

	kalTraceEvent("Move ipid=0x%04x sn=%d",
		GLUE_GET_PKT_IP_ID(prMsduInfo->prPacket),
		GLUE_GET_PKT_SEQ_NO(prMsduInfo->prPacket));

	kalGetPacketBuf(prMsduInfo->prPacket, &pucEth);

	if (pucEth == NULL) {
		DBGLOG(TX, ERROR, "pucEth=NULL");
		return;
	}

	u4PacketLen = kalQueryPacketLength(prMsduInfo->prPacket);

	u8SysTime = StatsEnvTimeGet();
	u8SysTimeIn = GLUE_GET_PKT_XTIME(prMsduInfo->prPacket);

	if ((g_ucTxRxFlag & BIT(0)) == 0)
		return;

	if ((u8SysTimeIn == 0) || (u8SysTime <= u8SysTimeIn))
		return;

	/* units of u4TimeDiff is micro seconds (us) */
	if (u4PacketLen < 24 + ETH_HLEN)
		return;
	pucAheadBuf = &pucEth[76];
	u2EthType = (pucAheadBuf[ETH_TYPE_LEN_OFFSET] << 8)
		| (pucAheadBuf[ETH_TYPE_LEN_OFFSET + 1]);
	pucEthBody = &pucAheadBuf[ETH_HLEN];
	if (u2EthType != ETH_P_IPV4)
		return;
	ucIpProto = pucEthBody[9];
	if (g_ucTxIpProto && (ucIpProto != g_ucTxIpProto))
		return;
	ucIpVersion = (pucEthBody[0] & IPVH_VERSION_MASK)
		>> IPVH_VERSION_OFFSET;
	if (ucIpVersion != IPVERSION)
		return;
	u2IPID = pucEthBody[4]<<8 | pucEthBody[5];
	u8SysTime = u8SysTime - u8SysTimeIn;
	u4TimeDiff = (uint32_t) u8SysTime;
	u4TimeDiff = u4TimeDiff / 1000;	/* ns to us */

	switch (ucIpProto) {
	case IP_PRO_TCP:
	case IP_PRO_UDP:
		u2UdpDstPort = (pucEthBody[22] << 8) | pucEthBody[23];
		u2UdpSrcPort = (pucEthBody[20] << 8) | pucEthBody[21];
		if (g_u2TxUdpPort && (u2UdpDstPort != g_u2TxUdpPort))
			break;
		kal_fallthrough;
	case IP_PRO_ICMP:
		u4TotalTx++;
		if (g_u4TxDelayThreshold
			&& (u4TimeDiff <= g_u4TxDelayThreshold)) {
			u4NoDelayTx++;
			break;
		}
		DBGLOG(TX, INFO,
			"IPID 0x%04x src %d dst %d UP %d,delay %u us,u8SysTimeIn %llu, %u/%u\n",
			u2IPID, u2UdpSrcPort, u2UdpDstPort,
			((pucEthBody[1] & IPTOS_PREC_MASK)
				>> IPTOS_PREC_OFFSET),
			u4TimeDiff, u8SysTimeIn, u4NoDelayTx, u4TotalTx);
		break;
	default:
		break;
	}
}

void statsParseARPInfo(void *pvPacket, uint8_t *pucEthBody,
	uint8_t eventType, uint16_t u2SSN)
{
	uint16_t u2OpCode = (pucEthBody[6] << 8) | pucEthBody[7];

	switch (eventType) {
	case EVENT_RX:
		GLUE_SET_INDEPENDENT_PKT(pvPacket, TRUE);
		GLUE_SET_PKT_FLAG(pvPacket, ENUM_PKT_ARP);
		if (u2OpCode == ARP_PRO_REQ)
			DBGLOG_LIMITED(RX, INFO,
				"<RX> Arp Req From IP: " IPV4STR " SSN:%u\n",
				IPV4TOSTR(&pucEthBody[ARP_SENDER_IP_OFFSET]),
				u2SSN);
		else if (u2OpCode == ARP_PRO_RSP)
			DBGLOG(RX, INFO,
				"<RX> Arp Rsp From IP: " IPV4STR " SSN:%u\n",
				IPV4TOSTR(&pucEthBody[ARP_SENDER_IP_OFFSET]),
				u2SSN);
		break;
	case EVENT_TX:
		DBGLOG(TX, INFO,
			"ARP %s SRC MAC/IP["
			MACSTR "]/[" IPV4STR "], TAR MAC/IP["
			MACSTR "]/[" IPV4STR "], SeqNo: %d\n",
			u2OpCode == ARP_OPERATION_REQUEST ? "REQ" : "RSP",
			MAC2STR(&pucEthBody[ARP_SENDER_MAC_OFFSET]),
			IPV4TOSTR(&pucEthBody[ARP_SENDER_IP_OFFSET]),
			MAC2STR(&pucEthBody[ARP_TARGET_MAC_OFFSET]),
			IPV4TOSTR(&pucEthBody[ARP_TARGET_IP_OFFSET]),
			GLUE_GET_PKT_SEQ_NO(pvPacket));
		break;
	}
}

void statsParseUDPInfo(void *pvPacket, uint8_t *pucEthBody,
	uint8_t eventType, uint16_t u2IpId, uint16_t u2SSN)
{
	/* the number of DHCP packets is seldom so we print log here */
	struct UDP_HEADER *pUdp = (struct UDP_HEADER *)
					&pucEthBody[IP_HEADER_LEN];
	struct DHCP_PROTOCOL *prDhcp;
	uint16_t u2UdpDstPort;
	uint16_t u2UdpSrcPort;
	uint32_t u4TransID;
	uint32_t u4DhcpMagicCode;
	char *msg_type = " ";
	uint32_t u4DhcpOpt = 0;
	uint16_t u2DnsTransId = 0;

	u2UdpDstPort = NTOHS(pUdp->u2DstPort);
	u2UdpSrcPort = NTOHS(pUdp->u2SrcPort);
	if (u2UdpDstPort == UDP_PORT_DHCPS || u2UdpDstPort == UDP_PORT_DHCPC) {
		prDhcp = (struct DHCP_PROTOCOL *)pUdp->aucData;
		u4TransID = NTOHL(prDhcp->u4TransId);
		u4DhcpMagicCode = NTOHL(prDhcp->u4MagicCookie);

		if (u4DhcpMagicCode != DHCP_MAGIC_NUMBER)
			return;

		WLAN_GET_FIELD_BE32(&prDhcp->aucDhcpOption[0], &u4DhcpOpt);
		switch (eventType) {
		case EVENT_RX:
			GLUE_SET_INDEPENDENT_PKT(pvPacket, TRUE);
			GLUE_SET_PKT_FLAG(pvPacket, ENUM_PKT_DHCP);

			switch (u4DhcpOpt & 0xffffff00) {
			case 0x35010100:
				msg_type = "DISCOVER";
				break;
			case 0x35010200:
				msg_type = "OFFER";
				break;
			case 0x35010300:
				msg_type = "REQUEST";
				break;
			case 0x35010500:
				msg_type = "ACK";
				break;
			case 0x35010600:
				msg_type = "NAK";
				break;
			}

			DBGLOG_LIMITED(RX, INFO,
				"<RX> DHCP: Recv %s IPID 0x%02x, MsgType 0x%x, TransID 0x%04x SSN:%u\n",
				msg_type, u2IpId, prDhcp->aucDhcpOption[2],
				u4TransID, u2SSN);
#if (CFG_SUPPORT_CONN_LOG == 1)
			connLogDhcpRx(g_prAdapter,
				GLUE_GET_PKT_BSS_IDX(pvPacket),
				u4DhcpOpt);
#endif
			break;

		case EVENT_TX:
			switch (u4DhcpOpt & 0xffffff00) {
			case 0x35010100:
				msg_type = "client DISCOVER";
				break;
			case 0x35010200:
				msg_type = "server OFFER";
				break;
			case 0x35010300:
				msg_type = "client REQUEST";
				break;
			case 0x35010500:
				msg_type = "server ACK";
				break;
			case 0x35010600:
				msg_type = "server NAK";
				break;
			}

			DBGLOG_LIMITED(TX, INFO,
				"<TX> DHCP %s, XID[0x%08x] OPT[0x%08x] TYPE[%u], SeqNo: %d\n",
				msg_type, u4TransID, u4DhcpOpt,
				prDhcp->aucDhcpOption[2],
				GLUE_GET_PKT_SEQ_NO(pvPacket));
#if (CFG_SUPPORT_CONN_LOG == 1)
			connLogDhcpTx(g_prAdapter,
				GLUE_GET_PKT_BSS_IDX(pvPacket),
				u4DhcpOpt,
				GLUE_GET_PKT_SEQ_NO(pvPacket));
#endif
			break;
		}
	} else if (u2UdpSrcPort == UDP_PORT_DNS ||
		   u2UdpDstPort == UDP_PORT_DNS) {
		WLAN_GET_FIELD_BE16(&pUdp->aucData[0], &u2DnsTransId);

		if (eventType == EVENT_RX) {
			GLUE_SET_INDEPENDENT_PKT(pvPacket, TRUE);
			GLUE_SET_PKT_FLAG(pvPacket, ENUM_PKT_DNS);
			DBGLOG_LIMITED(RX, INFO,
				"<RX> DNS: IPID 0x%02x, TransID 0x%04x, SSN:%u\n",
				u2IpId, u2DnsTransId, u2SSN);
		} else if (eventType == EVENT_TX) {
			DBGLOG_LIMITED(TX, INFO,
				"<TX> DNS: IPID[0x%02x] TransID[0x%04x] SeqNo[%d]\n",
				u2IpId, u2DnsTransId,
				GLUE_GET_PKT_SEQ_NO(pvPacket));
		}
	}
}

void statsParseIPV4Info(void *pvPacket, uint8_t *pucEthBody,
	uint8_t eventType, uint16_t u2SSN)
{
	/* IP header without options */
	uint8_t ucIpProto = pucEthBody[9];
	uint8_t ucIpVersion =
		(pucEthBody[0] & IPVH_VERSION_MASK)
			>> IPVH_VERSION_OFFSET;
	uint16_t u2IpId = pucEthBody[4] << 8 | pucEthBody[5];

	if (ucIpVersion != IPVERSION)
		return;

	GLUE_SET_PKT_IP_ID(pvPacket, u2IpId);

	switch (ucIpProto) {
	case IP_PRO_ICMP:
	{
		/* the number of ICMP packets is seldom so we print log here */
		uint8_t ucIcmpType;
		uint16_t u2IcmpId, u2IcmpSeq;
		uint8_t *pucIcmp = &pucEthBody[20];

		ucIcmpType = pucIcmp[ICMP_TYPE_OFFSET];
		/* don't log network unreachable packet */
		if (ucIcmpType == 3)
			break;
		u2IcmpId = HTONS(*(uint16_t *)&pucIcmp[ICMP_IDENTIFIER_OFFSET]);
		u2IcmpSeq = HTONS(*(uint16_t *)&pucIcmp[ICMP_SEQ_NUM_OFFSET]);

		switch (eventType) {
		case EVENT_RX:
			GLUE_SET_INDEPENDENT_PKT(pvPacket, TRUE);
			GLUE_SET_PKT_FLAG(pvPacket, ENUM_PKT_ICMP);
			DBGLOG_LIMITED(RX, INFO,
				"<RX> ICMP: IPID[0x%04x] Type %u, Id 0x%04x, Seq BE 0x%04x, SSN:%u\n",
				u2IpId, ucIcmpType, u2IcmpId, u2IcmpSeq, u2SSN);
			break;
		case EVENT_TX:
			DBGLOG_LIMITED(TX, INFO,
				"<TX> ICMP: IPID[0x%04x] Type %u, Id 0x%04x, Seq BE 0x%04x, SeqNo: %d\n",
				u2IpId, ucIcmpType, u2IcmpId, u2IcmpSeq,
				GLUE_GET_PKT_SEQ_NO(pvPacket));
			break;
		}
		break;
	}
	case IP_PRO_UDP:
		statsParseUDPInfo(pvPacket, pucEthBody, eventType, u2IpId,
				u2SSN);
	}
}

void statsLogData(uint8_t eventType, enum WAKE_DATA_TYPE wakeType)
{
	if (eventType == EVENT_TX)
		wlanLogTxData(wakeType);
	else if (eventType == EVENT_RX)
		wlanLogRxData(wakeType);
}

static void statsParsePktInfo(uint8_t *pucData, void *pvPacket,
	uint8_t status, uint8_t eventType, uint16_t u2SSN)

{
	/* get ethernet protocol */
	uint16_t u2EtherType =
		(pucData[ETH_TYPE_LEN_OFFSET] << 8)
			| (pucData[ETH_TYPE_LEN_OFFSET + 1]);
	uint8_t *pucEthBody = &pucData[ETH_HLEN];

	switch (u2EtherType) {
	case ETH_P_ARP:
	{
		statsLogData(eventType, WLAN_WAKE_ARP);
		statsParseARPInfo(pvPacket, pucEthBody, eventType, u2SSN);
		break;
	}
	case ETH_P_IPV4:
	{
		statsLogData(eventType, WLAN_WAKE_IPV4);
		statsParseIPV4Info(pvPacket, pucEthBody, eventType, u2SSN);
		break;
	}
	case ETH_P_IPV6:
	{
		/* IPv6 header without options */
		uint8_t ucIpv6Proto =
			pucEthBody[IPV6_HDR_PROTOCOL_OFFSET];
		uint8_t ucIpVersion =
			(pucEthBody[0] & IPVH_VERSION_MASK)
				>> IPVH_VERSION_OFFSET;

		if (ucIpVersion != IP_VERSION_6)
			break;

		statsLogData(eventType, WLAN_WAKE_IPV6);
		switch (ucIpv6Proto) {
		case IP_PRO_TCP:
			switch (eventType) {
			case EVENT_RX:
				DBGLOG(RX, TRACE,
					"<RX><IPv6> tcp packet SSN:%u\n",
					u2SSN);
				break;
			case EVENT_TX:
				DBGLOG(TX, TRACE, "<TX><IPv6> tcp packet\n");
				break;
			}
			break;

		case IP_PRO_UDP:
			switch (eventType) {
			case EVENT_RX:
			{
				uint16_t ucIpv6UDPSrcPort = 0;

				/* IPv6 header without options */
				ucIpv6UDPSrcPort = pucEthBody[IPV6_HDR_LEN];
				ucIpv6UDPSrcPort = ucIpv6UDPSrcPort << 8;
				ucIpv6UDPSrcPort +=
					pucEthBody[IPV6_HDR_LEN + 1];

				switch (ucIpv6UDPSrcPort) {
				case UDP_PORT_DNS:
					DBGLOG(RX, TRACE,
						"<RX><IPv6> dns packet SSN:%u\n",
						u2SSN);
					GLUE_SET_INDEPENDENT_PKT(
						pvPacket, TRUE);
					GLUE_SET_PKT_FLAG(pvPacket,
						ENUM_PKT_DNS);
					break;
				case IPV6_UDP_PORT_DHCPC:
				case IPV6_UDP_PORT_DHCPS:
					DBGLOG(RX, INFO,
						"<RX><IPv6> dhcp packet SSN:%u\n",
						u2SSN);
					GLUE_SET_INDEPENDENT_PKT(
						pvPacket, TRUE);
					GLUE_SET_PKT_FLAG(pvPacket,
						ENUM_PKT_DHCP);
					break;
				case UDP_PORT_NTP:
					DBGLOG(RX, INFO,
						"<RX><IPv6> ntp packet SSN:%u\n",
						u2SSN);
					GLUE_SET_INDEPENDENT_PKT(
						pvPacket, TRUE);
					break;
				default:
					DBGLOG(RX, TRACE,
					"<RX><IPv6> other packet srtport=%u SSN:%u\n",
						ucIpv6UDPSrcPort, u2SSN);
					break;
				}
			}
				break;
			case EVENT_TX:
				DBGLOG(TX, TRACE, "<TX><IPv6> UDP packet\n");
				break;
			}
			break;

		case IPV6_PROTOCOL_HOP_BY_HOP:
			switch (eventType) {
			case EVENT_RX:
				/*need chech detai pakcet type*/
				/*130 mlti listener query*/
				/*143 multi listener report v2*/
				GLUE_SET_INDEPENDENT_PKT(pvPacket, TRUE);
				GLUE_SET_PKT_FLAG(pvPacket,
					ENUM_PKT_IPV6_HOP_BY_HOP);
				DBGLOG_LIMITED(RX, INFO,
					"<RX><IPv6> hop-by-hop packet, SSN:%u\n",
					u2SSN);
				break;
			case EVENT_TX:
				DBGLOG_LIMITED(TX, INFO,
					"<TX><IPv6> hop-by-hop packet\n");
				break;
			}
			break;

		case IPV6_PROTOCOL_ICMPV6:
			switch (eventType) {
			case EVENT_RX:
			{
				uint8_t ucICMPv6Type = 0;

				/* IPv6 header without options */
				ucICMPv6Type = pucEthBody[IPV6_HDR_LEN];
				GLUE_SET_INDEPENDENT_PKT(pvPacket, TRUE);
				GLUE_SET_PKT_FLAG(pvPacket,
					ENUM_PKT_ICMPV6);
				switch (ucICMPv6Type) {
				case ICMPV6_TYPE_ROUTER_SOLICITATION:
					DBGLOG_LIMITED(RX, INFO,
				"<RX><IPv6> ICMPV6 Router Solicitation SSN:%u\n",
						u2SSN);
					break;

				case ICMPV6_TYPE_ROUTER_ADVERTISEMENT:
					DBGLOG_LIMITED(RX, INFO,
				"<RX><IPv6> ICMPV6 Router Advertisement SSN:%u\n",
						u2SSN);
					break;

				case ICMPV6_TYPE_NEIGHBOR_SOLICITATION:
					DBGLOG_LIMITED(RX, INFO,
				"<RX><IPv6> ICMPV6 Neighbor Solicitation SSN:%u\n",
						u2SSN);
					break;

				case ICMPV6_TYPE_NEIGHBOR_ADVERTISEMENT:
					DBGLOG_LIMITED(RX, INFO,
				"<RX><IPv6> ICMPV6 Neighbor Advertisement SSN:%u\n",
						u2SSN);
					break;
				default:
					DBGLOG_LIMITED(RX, INFO,
						"<RX><IPv6> ICMPV6 type=%u SSN:%u\n",
						ucICMPv6Type, u2SSN);
					break;
				}
			}
				break;
			case EVENT_TX:
				DBGLOG_LIMITED(TX, INFO,
					"<TX><IPv6> ICMPV6 packet\n");
				break;
			}
			break;
		default:
			if (eventType == EVENT_RX)
				DBGLOG(RX, INFO,
				"<RX><IPv6> default protocol=%u SSN:%u\n",
				ucIpv6Proto, u2SSN);
			break;
		}
		break;
	}
	case ETH_P_1X:
	{
		uint8_t *pucEapol = pucEthBody;
		uint8_t ucEapolType = pucEapol[1];
		uint16_t u2KeyInfo = 0;
		uint8_t m = 0;
#if (CFG_SUPPORT_CONN_LOG == 1)
		uint16_t u2EapLen = 0;
#endif

		if (eventType == EVENT_RX)
			GLUE_SET_PKT_FLAG(pvPacket, ENUM_PKT_1X);

		statsLogData(eventType, WLAN_WAKE_1X);
		switch (ucEapolType) {
		case 0: /* eap packet */
#if (CFG_SUPPORT_CONN_LOG == 1)
			WLAN_GET_FIELD_BE16(&pucEapol[6], &u2EapLen);
#endif
			switch (eventType) {
			case EVENT_RX:
				DBGLOG(RX, INFO,
					"<RX> EAP Packet: code=%u id=%u len=%u type=%d SSN:%u\n",
					pucEapol[4], pucEapol[5],
					NTOHS(*(uint16_t *)&pucEapol[6]),
					pucEapol[8], u2SSN);
#if (CFG_SUPPORT_CONN_LOG == 1)
				connLogEapRx(
					g_prAdapter,
					GLUE_GET_PKT_BSS_IDX(pvPacket),
					u2EapLen,
					pucEapol[8],
					pucEapol[4]);
#endif
				break;
			case EVENT_TX:
				DBGLOG(TX, INFO,
					"<TX> EAP Packet: code=%u id=%u len=%u type=%d SeqNo=%d\n",
					pucEapol[4], pucEapol[5],
					NTOHS(*(uint16_t *)&pucEapol[6]),
					pucEapol[8],
					GLUE_GET_PKT_SEQ_NO(pvPacket));
#if (CFG_SUPPORT_CONN_LOG == 1)
				connLogEapTx(
					g_prAdapter,
					GLUE_GET_PKT_BSS_IDX(pvPacket),
					u2EapLen,
					pucEapol[8],
					pucEapol[4],
					GLUE_GET_PKT_SEQ_NO(pvPacket));
#endif
				break;
			}
			break;
		case 1: /* eapol start */
			switch (eventType) {
			case EVENT_RX:
				DBGLOG(RX, INFO,
					"<RX> EAPOL: start SSN:%u\n", u2SSN);
				break;
			case EVENT_TX:
				DBGLOG(TX, INFO,
				       "<TX> EAPOL: start, SeqNo: %d\n",
						GLUE_GET_PKT_SEQ_NO(pvPacket));
				break;
			}
			break;
		case ETH_EAPOL_KEY: /* key */
			WLAN_GET_FIELD_BE16(&pucEapol[5], &u2KeyInfo);
#if (CFG_SUPPORT_CONN_LOG == 1)
			connLogEapKey(g_prAdapter,
				GLUE_GET_PKT_BSS_IDX(pvPacket),
				eventType,
				pucEapol,
				GLUE_GET_PKT_SEQ_NO(pvPacket));
#endif
			switch (eventType) {
			case EVENT_RX:
			case EVENT_TX:
				if ((u2KeyInfo & 0x1100) == 0x0000 ||
					(u2KeyInfo & 0x0008) == 0x0000)
					m = 1;
				else if ((u2KeyInfo & 0xfff0) == 0x0100)
					m = 2;
				else if ((u2KeyInfo & 0xfff0) == 0x13c0)
					m = 3;
				else if ((u2KeyInfo & 0xfff0) == 0x0300)
					m = 4;
				if (eventType == EVENT_RX)
					DBGLOG(RX, INFO,
						"<RX> EAPOL: key, M%d, KeyInfo 0x%04x, SSN:%u\n",
						m, u2KeyInfo, u2SSN);
				else
					DBGLOG(TX, INFO,
					       "<TX> EAPOL: key, M%d, KeyInfo 0x%04x SeqNo: %d\n",
					       m, u2KeyInfo,
						GLUE_GET_PKT_SEQ_NO(pvPacket));
				break;
			}
			/* Record EAPOL key type */
			GLUE_SET_INDEPENDENT_EAPOL(pvPacket, m);
			break;
		}
		break;
	}
#if CFG_SUPPORT_WAPI
	case ETH_WPI_1X:
	{
		uint8_t ucSubType = pucEthBody[3]; /* sub type filed*/
		uint16_t u2Length = *(uint16_t *)&pucEthBody[6];
		uint16_t u2Seq = *(uint16_t *)&pucEthBody[8];

		statsLogData(eventType, WLAN_WAKE_1X);
		switch (eventType) {
		case EVENT_RX:
			DBGLOG(RX, INFO,
				"<RX> WAPI: subType %d, Len %d, Seq %d, SSN:%u\n",
				ucSubType, u2Length, u2Seq, u2SSN);
			GLUE_SET_PKT_FLAG(pvPacket, ENUM_PKT_1X);
			break;
		case EVENT_TX:
			DBGLOG(TX, INFO,
			       "<TX> WAPI: subType %d, Len %d, Seq %d, SeqNo: %d\n",
			       ucSubType, u2Length, u2Seq,
					GLUE_GET_PKT_SEQ_NO(pvPacket));
			break;
		}
		break;
	}
#endif
	case ETH_PRO_TDLS:
		statsLogData(eventType, WLAN_WAKE_TDLS);
		switch (eventType) {
		case EVENT_RX:
			DBGLOG(RX, INFO,
				"<RX> TDLS type %d, category %d, Action %d, Token %d, SSN:%u\n",
				pucEthBody[0], pucEthBody[1],
				pucEthBody[2], pucEthBody[3],
				u2SSN);
			GLUE_SET_PKT_FLAG(pvPacket, ENUM_PKT_TDLS);
			break;
		case EVENT_TX:
			DBGLOG(TX, INFO,
				"<TX> TDLS type %d, category %d, Action %d, Token %d\n",
				pucEthBody[0], pucEthBody[1],
				pucEthBody[2], pucEthBody[3]);
			break;
		}
		break;
	default:
		statsLogData(eventType, WLAN_WAKE_OTHER);
		break;
	}
}
/*----------------------------------------------------------------------------*/
/*! \brief  This routine is called to display rx packet information.
 *
 * \param[in] pPkt			Pointer to the packet
 * \param[out] None
 *
 * \retval None
 */
/*----------------------------------------------------------------------------*/
void StatsRxPktInfoDisplay(struct SW_RFB *prSwRfb)
{
	uint8_t *pPkt = NULL;

	if (prSwRfb->u2PacketLen <= ETHER_HEADER_LEN)
		return;

	pPkt = prSwRfb->pvHeader;
	if (!pPkt)
		return;

	if (!prSwRfb->pvPacket)
		return;

	statsParsePktInfo(pPkt, prSwRfb->pvPacket, 0, EVENT_RX, prSwRfb->u2SSN);

	DBGLOG(RX, TEMP, "RxPkt p=%p ipid=%d\n",
		prSwRfb, GLUE_GET_PKT_IP_ID(prSwRfb->pvPacket));
	kalTraceEvent("RxPkt p=%p ipid=0x%04x",
		prSwRfb, GLUE_GET_PKT_IP_ID(prSwRfb->pvPacket));
}

/*----------------------------------------------------------------------------*/
/*! \brief  This routine is called to display tx packet information.
 *
 * \param[in] pPkt			Pointer to the packet
 * \param[out] None
 *
 * \retval None
 */
/*----------------------------------------------------------------------------*/
void StatsTxPktInfoDisplay(void *pvPacket)
{
	uint8_t *pPktBuf;

	kalGetPacketBuf(pvPacket, &pPktBuf);
	statsParsePktInfo(pPktBuf, pvPacket, 0, EVENT_TX, 0);
}

uint32_t
statsGetTlvU4Len(struct GLUE_INFO *prGlueInfo)
{
	return sizeof(uint32_t);
}

uint32_t
statsGetTlvU8Len(struct GLUE_INFO *prGlueInfo)
{
	return sizeof(uint64_t);
}

uint32_t
statsTxGetQueueLen(struct GLUE_INFO *prGlueInfo)
{
	return sizeof(struct STATS_TX_QUEUE_STAT_T);
}

uint32_t
statsTxGetRetryLen(struct GLUE_INFO *prGlueInfo)
{
	return sizeof(struct STATS_TX_RETRY_STAT_T);
}

uint32_t
statsTxGetTimeLen(struct GLUE_INFO *prGlueInfo)
{
	return sizeof(struct STATS_TX_TIME_STAT_T);
}

uint32_t
statsTxGetLatLen(struct GLUE_INFO *prGlueInfo)
{
	return sizeof(struct STATS_TX_LAT_STAT_T);
}

uint32_t
statsRxGetAvgRssiLen(struct GLUE_INFO *prGlueInfo)
{
	return sizeof(struct STATS_RX_AVG_RSSI_STAT_T);
}

uint32_t
statsCgsGetAirLatLen(struct GLUE_INFO *prGlueInfo)
{
	return sizeof(struct STATS_CGS_LAT_STAT_T);
}

uint32_t
stateGetTlvTag(struct STATS_TLV_MAP_T *apfnTbl, uint32_t u4TargetTag,
	uint32_t u4MaxTagNum)
{

	uint32_t i = 0;

	for (i = 0; i < u4MaxTagNum; i++) {
		if (u4TargetTag == apfnTbl[i].u4Tag)
			break;
	}
	return i;
}

uint32_t
statsGetTlvStatTotalLen(struct GLUE_INFO *prGlueInfo, uint8_t type,
	uint8_t ucNum, uint32_t *arTagList)
{
	uint32_t u4TlvLen = 0;
	uint32_t u4TlvIdx = 0;
	uint8_t i = 0;
	struct STATS_TLV_MAP_T *pTlvTbl = NULL;
	uint32_t u4MaxTagNum = 0;

	switch (type) {
	case STATS_TX_TAG:
		pTlvTbl = apfnTxTbl;
		u4MaxTagNum = STATS_TX_TAG_MAX_NUM;
		break;
	case STATS_RX_TAG:
		pTlvTbl = apfnRxTbl;
		u4MaxTagNum = STATS_RX_TAG_MAX_NUM;
		break;
	case STATS_CGS_TAG:
		pTlvTbl = apfnCgsTbl;
		u4MaxTagNum = STATS_CGS_TAG_MAX_NUM;
		break;
	}

	if (!pTlvTbl || ucNum == 0 || !arTagList)
		return u4TlvLen;

	for (i = 0; i < ucNum; i++) {
		u4TlvIdx = stateGetTlvTag(pTlvTbl, arTagList[i], u4MaxTagNum);

		if (u4TlvIdx == u4MaxTagNum) {
			DBGLOG(TX, TRACE, "type=%u invalid tag=%u\n",
				type, arTagList[i]);
			continue;
		}

		u4TlvLen += (pTlvTbl[u4TlvIdx].tlvHdl.pfnTlvGetLen(prGlueInfo)
				+ sizeof(struct STATS_TRX_TLV_T));
	}

	DBGLOG(TX, TRACE, "type=%u len=%u\n", type, u4TlvLen);
	return u4TlvLen;
}

void
statsTxQueueHdlr(uint8_t ucBssIdx,
	struct GLUE_INFO *prGlueInfo,
	void *prTlvBuf, uint32_t u4TlvLen)
{
#if defined(_HIF_PCIE) || defined(_HIF_AXI)
	struct ADAPTER *prAdapter;
	struct BUS_INFO *prBusInfo;
	struct PLE_TOP_CR *prCr;
	struct CMD_ACCESS_REG rCmdAccessReg;
	struct STATS_TRX_TLV_T *prStatTlv = prTlvBuf;
	struct STATS_TX_QUEUE_STAT_T *prQueueStat;
	uint32_t u4MsduTokenUsed = 0, u4MsduTokenNum = 0;
	uint32_t u4BufLen = 0;
	uint32_t rStatus;

	prQueueStat = (struct STATS_TX_QUEUE_STAT_T *)(
		&prStatTlv->aucBuffer[0]);
	/* MSDU token */
	prAdapter = prGlueInfo->prAdapter;
	u4MsduTokenUsed = prGlueInfo->rHifInfo.rTokenInfo.u4UsedCnt;
	u4MsduTokenNum = prGlueInfo->rHifInfo.rTokenInfo.u4TokenNum;
	prQueueStat->u4MsduTokenUsed = u4MsduTokenUsed;
	prQueueStat->u4MsduTokenRsvd = u4MsduTokenNum - u4MsduTokenUsed;

	/* ple hif */
	prBusInfo = prAdapter->chip_info->bus_info;
	prCr = prBusInfo->prPleTopCr;
	rCmdAccessReg.u4Address = prCr->rHifPgInfo.u4Addr;
	rCmdAccessReg.u4Data = 0;

	rStatus = kalIoctl(prGlueInfo, wlanoidQueryMcrRead,
			&rCmdAccessReg, sizeof(rCmdAccessReg),
			&u4BufLen);
	prQueueStat->u4PleHifUsed = ((rCmdAccessReg.u4Data &
		prCr->rHifPgInfoHifSrcCnt.u4Mask) >>
		prCr->rHifPgInfoHifSrcCnt.u4Shift);
	prQueueStat->u4PleHifRsvd = ((rCmdAccessReg.u4Data &
		prCr->rHifPgInfoHifRsvCnt.u4Mask) >>
		prCr->rHifPgInfoHifRsvCnt.u4Shift);

	prStatTlv->u4Tag = STATS_TX_TAG_QUEUE;
	prStatTlv->u4Len = u4TlvLen;
	DBGLOG(TX, TRACE, "len=%u Msdu=[%u/%u] PLE Hif=[0x%03x/0x%03x]\n",
		u4TlvLen, prQueueStat->u4MsduTokenUsed,
		prQueueStat->u4MsduTokenRsvd,
		prQueueStat->u4PleHifUsed, prQueueStat->u4PleHifRsvd);
#endif
}

void
statsTxGetRetryHdlr(uint8_t ucBssIdx,
	struct GLUE_INFO *prGlueInfo,
	void *prTlvBuf, uint32_t u4TlvLen)
{
#if CFG_SUPPORT_LINK_QUALITY_MONITOR
	struct ADAPTER *prAdapter = prGlueInfo->prAdapter;
	struct PARAM_GET_LINK_QUALITY_INFO rParam;
	struct WIFI_LINK_QUALITY_INFO rLinkQualityInfo;
	struct STATS_TRX_TLV_T *prStatTlv = prTlvBuf;
	struct STATS_TX_RETRY_STAT_T *prRetryStat;
	uint32_t u4BufLen;
	int32_t i4Status;
	uint64_t u8Retry = 0;
	uint64_t u8RtsFail = 0;
	uint64_t u8AckFail = 0;

	prRetryStat = (struct STATS_TX_RETRY_STAT_T *)(
		&prStatTlv->aucBuffer[0]);
	rParam.ucBssIdx = ucBssIdx;
	rParam.prLinkQualityInfo = &rLinkQualityInfo;
	i4Status = kalIoctl(prGlueInfo, wlanoidGetLinkQualityInfo,
		 &rParam, sizeof(struct PARAM_GET_LINK_QUALITY_INFO),
		 &u4BufLen);
	if (i4Status != WLAN_STATUS_SUCCESS)
		DBGLOG(REQ, ERROR, "wlanoidGetLinkQualityInfo error\n");
	else {
		if (kalGetMediaStateIndicated(prGlueInfo,
		    aisGetDefaultLinkBssIndex(prAdapter)) ==
			MEDIA_STATE_CONNECTED) {
			u8Retry = rLinkQualityInfo.u8TxRetryCount;
			u8RtsFail = rLinkQualityInfo.u8TxRtsFailCount;
			u8AckFail = rLinkQualityInfo.u8TxAckFailCount;
		} else {
			DBGLOG(TX, TRACE, "Bss not connected yet.\n");
		}
	}
	prRetryStat->u8Retry = u8Retry;
	prRetryStat->u8RtsFail = u8RtsFail;
	prRetryStat->u8AckFail = u8AckFail;
	prStatTlv->u4Tag = STATS_TX_TAG_RETRY;
	prStatTlv->u4Len = u4TlvLen;
	DBGLOG(TX, TRACE, "len=%u retry=%llu RtsFail=%llu AckFail=%llu\n",
		u4TlvLen, prRetryStat->u8Retry, prRetryStat->u8RtsFail,
		prRetryStat->u8AckFail);
#endif
}

void
statsTxTimeHdlr(uint8_t ucBssIdx,
	struct GLUE_INFO *prGlueInfo,
	void *prTlvBuf, uint32_t u4TlvLen)
{
	struct STATS_TRX_TLV_T *prStatTlv = prTlvBuf;
	struct STATS_TX_TIME_STAT_T *prTimeStat;
#if CFG_SUPPORT_TX_LATENCY_STATS
	struct ADAPTER *prAdapter = prGlueInfo->prAdapter;
	struct TX_LATENCY_STATS *stats;
	uint8_t i;
#endif

	prTimeStat = (struct STATS_TX_TIME_STAT_T *)(
		&prStatTlv->aucBuffer[0]);
	kalMemZero(prTimeStat, sizeof(*prTimeStat));

#if CFG_SUPPORT_TX_LATENCY_STATS
	stats = &prAdapter->rMsduReportStats.rCounting;
	for (i = 0; i < TX_TIME_CAT_NUM; i++) {
		prTimeStat->au4Success[i] = GLUE_GET_REF_CNT(
			    stats->au4ConnsysLatency[ucBssIdx][i]);
		prTimeStat->au4Fail[i] = GLUE_GET_REF_CNT(
			    stats->au4FailConnsysLatency[ucBssIdx][i]);
	}

#else
	DBGLOG(TX, INFO, "tx latency not support.\n");
#endif
	prStatTlv->u4Tag = STATS_TX_TAG_TIME;
	prStatTlv->u4Len = u4TlvLen;
	DBGLOG(TX, TRACE,
		"len=%u bssIdx:%u suc=%u/%u/%u/%u/%u fail=%u/%u/%u/%u/%u\n",
		u4TlvLen, ucBssIdx,
		prTimeStat->au4Success[0], prTimeStat->au4Success[1],
		prTimeStat->au4Success[2], prTimeStat->au4Success[3],
		prTimeStat->au4Success[4], prTimeStat->au4Fail[0],
		prTimeStat->au4Fail[1], prTimeStat->au4Fail[2],
		prTimeStat->au4Fail[3], prTimeStat->au4Fail[4]);
}

void
statsTxLatHdlr(uint8_t ucBssIdx,
	struct GLUE_INFO *prGlueInfo,
	void *prTlvBuf, uint32_t u4TlvLen)
{
	struct STATS_TRX_TLV_T *prStatTlv = prTlvBuf;
	struct STATS_TX_LAT_STAT_T *prLatStat;
#if CFG_SUPPORT_TX_LATENCY_STATS
	struct ADAPTER *prAdapter = prGlueInfo->prAdapter;
	struct TX_LATENCY_STATS *stats;
	uint8_t i;
#endif

	prLatStat = (struct STATS_TX_LAT_STAT_T *)(
		&prStatTlv->aucBuffer[0]);
	kalMemZero(prLatStat, sizeof(*prLatStat));

#if CFG_SUPPORT_TX_LATENCY_STATS
	stats = &prAdapter->rMsduReportStats.rCounting;
	for (i = 0; i < TX_TIME_CAT_NUM; i++) {
		prLatStat->au4DriverLat[i] = GLUE_GET_REF_CNT(
			stats->au4DriverLatency[ucBssIdx][i]);
		prLatStat->au4MacLat[i] = GLUE_GET_REF_CNT(
			stats->au4MacLatency[ucBssIdx][i]);
	}
#else
	DBGLOG(TX, INFO, "tx latency not support.\n");
#endif
	prStatTlv->u4Tag = STATS_TX_TAG_LAT;
	prStatTlv->u4Len = u4TlvLen;
	DBGLOG(TX, TRACE,
		"len=%u bssIdx:%u driver=%u/%u/%u/%u/%u mac=%u/%u/%u/%u/%u\n",
		u4TlvLen, ucBssIdx,
		prLatStat->au4DriverLat[0], prLatStat->au4DriverLat[1],
		prLatStat->au4DriverLat[2], prLatStat->au4DriverLat[3],
		prLatStat->au4DriverLat[4], prLatStat->au4MacLat[0],
		prLatStat->au4MacLat[1], prLatStat->au4MacLat[2],
		prLatStat->au4MacLat[3], prLatStat->au4MacLat[4]);
}

void
statsRxReorderDropHdlr(uint8_t ucBssIdx,
	struct GLUE_INFO *prGlueInfo,
	void *prTlvBuf, uint32_t u4TlvLen)
{
	struct ADAPTER *prAdapter = prGlueInfo->prAdapter;
	struct STATS_TRX_TLV_T *prStatTlv = prTlvBuf;
	uint64_t *pu8RxReorderDrop = (uint64_t *)(&prStatTlv->aucBuffer[0]);

	*pu8RxReorderDrop = RX_GET_CNT(&prAdapter->rRxCtrl,
		RX_REORDER_BEHIND_DROP_COUNT);
	prStatTlv->u4Tag = STATS_RX_TAG_REORDER_DROP;
	prStatTlv->u4Len = u4TlvLen;
	DBGLOG(RX, TRACE, "ReorderDrop len=%u val=%llu\n", u4TlvLen,
		*pu8RxReorderDrop);
}

void
statsRxAvgRssiHdlr(uint8_t ucBssIdx,
	struct GLUE_INFO *prGlueInfo,
	void *prTlvBuf, uint32_t u4TlvLen)
{
	struct STATS_TRX_TLV_T *prStatTlv = prTlvBuf;
	struct STATS_RX_AVG_RSSI_STAT_T *prRssiStat;
	struct ADAPTER *prAdapter = prGlueInfo->prAdapter;
	struct CHIP_DBG_OPS *prChipDbg;
	uint8_t ucWlanIndex = 0;
	int32_t reserved[2];

	prRssiStat = (struct STATS_RX_AVG_RSSI_STAT_T *)(
		&prStatTlv->aucBuffer[0]);
	kalMemZero(prRssiStat, sizeof(*prRssiStat));

	if (!wlanGetWlanIdxByAddress(prAdapter, NULL,
		&ucWlanIndex)) {
		DBGLOG(REQ, INFO, "wlan index is not found!\n");
		goto out;
	}

	prChipDbg = prAdapter->chip_info->prDebugOps;
	if (prChipDbg && prChipDbg->get_rssi_from_wtbl) {
		prChipDbg->get_rssi_from_wtbl(
			prAdapter, ucWlanIndex,
			&(prRssiStat->i4Rssi0), &(prRssiStat->i4Rssi1),
			&reserved[0], &reserved[1]);
	}

out:
	prStatTlv->u4Tag = STATS_RX_TAG_AVG_RSSI;
	prStatTlv->u4Len = u4TlvLen;
	DBGLOG(RX, TRACE,
		"len=%u rssi=%d/%d\n",
		u4TlvLen, prRssiStat->i4Rssi0, prRssiStat->i4Rssi1);
}

void
statsCgsB0IdleSlotHdlr(uint8_t ucBssIdx,
	struct GLUE_INFO *prGlueInfo,
	void *prTlvBuf, uint32_t u4TlvLen)
{
#if CFG_SUPPORT_LINK_QUALITY_MONITOR
	struct ADAPTER *prAdapter = prGlueInfo->prAdapter;
	struct WIFI_LINK_QUALITY_INFO *prLinkQualityInfo;
#endif
	struct STATS_TRX_TLV_T *prStatTlv = prTlvBuf;
	uint64_t *pu8B0IdleSlot = (uint64_t *)(&prStatTlv->aucBuffer[0]);

#if CFG_SUPPORT_LINK_QUALITY_MONITOR
	prLinkQualityInfo = &(prAdapter->rLinkQualityInfo);
	if (prLinkQualityInfo)
		*pu8B0IdleSlot = prLinkQualityInfo->u8IdleSlotCount;
#else
	goto err;
#endif
	prStatTlv->u4Tag = STATS_CGS_TAG_B0_IDLE_SLOT;
	prStatTlv->u4Len = u4TlvLen;
	DBGLOG(TX, TRACE, "len=%u val=%llu\n", u4TlvLen, *pu8B0IdleSlot);

#if !CFG_SUPPORT_LINK_QUALITY_MONITOR
err:
	*pu8B0IdleSlot = 0;
	prStatTlv->u4Tag = STATS_CGS_TAG_B0_IDLE_SLOT;
	prStatTlv->u4Len = 0;
	DBGLOG(TX, TRACE, "len=%u val=%llu\n", u4TlvLen, *pu8B0IdleSlot);
#endif
}

void
statsCgsAirLatHdlr(uint8_t ucBssIdx,
	struct GLUE_INFO *prGlueInfo,
	void *prTlvBuf, uint32_t u4TlvLen)
{
	struct STATS_TRX_TLV_T *prStatTlv = prTlvBuf;
	struct STATS_CGS_LAT_STAT_T *prAirLat;
#if CFG_SUPPORT_LLS
	union {
		struct CMD_GET_STATS_LLS cmd;
		struct EVENT_STATS_LLS_TX_LATENCY latency;
	} query = {0};
	uint32_t u4QueryBufLen;
	uint32_t u4QueryInfoLen;
	uint32_t rStatus = WLAN_STATUS_SUCCESS;
#endif

	prAirLat = (struct STATS_CGS_LAT_STAT_T *)(&prStatTlv->aucBuffer[0]);
	kalMemZero(prAirLat, sizeof(*prAirLat));

#if CFG_SUPPORT_LLS
	u4QueryBufLen = sizeof(query);
	u4QueryInfoLen = sizeof(query.cmd);

	kalMemZero(&query, sizeof(query));
	query.cmd.u4Tag = STATS_LLS_TAG_PPDU_LATENCY;
	query.cmd.ucArg0 = ucBssIdx;

	rStatus = kalIoctl(prGlueInfo,
			wlanQueryLinkStats,
			&query,
			u4QueryBufLen,
			&u4QueryInfoLen);
	DBGLOG(REQ, INFO, "kalIoctl=%x, %u bytes",
				rStatus, u4QueryInfoLen);

	if (rStatus == WLAN_STATUS_SUCCESS &&
		u4QueryInfoLen == sizeof(struct EVENT_STATS_LLS_TX_LATENCY)) {
		DBGLOG(REQ, INFO, "query.lat=%u/%u/%u/%u; %u/%u/%u/%u/%u",
			query.latency.arLatencyLevel[0],
			query.latency.arLatencyLevel[1],
			query.latency.arLatencyLevel[2],
			query.latency.arLatencyLevel[3],
			query.latency.arLatencyMpduCntPerLevel[0],
			query.latency.arLatencyMpduCntPerLevel[1],
			query.latency.arLatencyMpduCntPerLevel[2],
			query.latency.arLatencyMpduCntPerLevel[3],
			query.latency.arLatencyMpduCntPerLevel[4]);
		kalMemCopy(prAirLat->au4AirLatLvl,
			query.latency.arLatencyLevel,
			sizeof(uint32_t) * AIR_LAT_LVL_NUM);
		kalMemCopy(prAirLat->au4AirLatMpdu,
			query.latency.arLatencyMpduCntPerLevel,
			sizeof(uint32_t) * AIR_LAT_CAT_NUM);
	} else if (rStatus != WLAN_STATUS_SUCCESS) {
		DBGLOG(REQ, WARN, "wlanQueryLinkStats return fail\n");
	} else {
		DBGLOG(REQ, WARN, "wlanQueryLinkStats return len unexpected\n");
	}
#else
	DBGLOG(TX, INFO, "LLS not support.\n");
#endif
	prStatTlv->u4Tag = STATS_CGS_TAG_AIR_LAT;
	prStatTlv->u4Len = u4TlvLen;
	DBGLOG(TX, TRACE,
		"len=%u bssIdx:%u lvl=%u/%u/%u/%u cnt=%u/%u/%u/%u/%u\n",
		u4TlvLen, ucBssIdx,
		prAirLat->au4AirLatLvl[0], prAirLat->au4AirLatLvl[1],
		prAirLat->au4AirLatLvl[2], prAirLat->au4AirLatLvl[3],
		prAirLat->au4AirLatMpdu[0], prAirLat->au4AirLatMpdu[1],
		prAirLat->au4AirLatMpdu[2], prAirLat->au4AirLatMpdu[3],
		prAirLat->au4AirLatMpdu[4]);
}

void
statsGetInfoHdlr(uint8_t ucBssIdx, struct GLUE_INFO *prGlueInfo,
	void *prTlvBuf, uint8_t type, uint8_t ucNum, uint32_t *arTagList)
{
	uint32_t u4TlvLen = 0;
	uint32_t u4TlvIdx = 0;
	uint8_t i = 0;
	struct STATS_TLV_MAP_T *pTlvTbl = NULL;
	uint32_t u4MaxTagNum = 0;
	uint8_t *ptr = prTlvBuf;

	switch (type) {
	case STATS_TX_TAG:
		pTlvTbl = apfnTxTbl;
		u4MaxTagNum = STATS_TX_TAG_MAX_NUM;
		break;
	case STATS_RX_TAG:
		pTlvTbl = apfnRxTbl;
		u4MaxTagNum = STATS_RX_TAG_MAX_NUM;
		break;
	case STATS_CGS_TAG:
		pTlvTbl = apfnCgsTbl;
		u4MaxTagNum = STATS_CGS_TAG_MAX_NUM;
		break;
	}

	if (!pTlvTbl || ucNum == 0 || !arTagList)
		return;

	for (i = 0; i < ucNum; i++) {
		u4TlvIdx = stateGetTlvTag(pTlvTbl, arTagList[i], u4MaxTagNum);
		if (u4TlvIdx == u4MaxTagNum)
			continue;

		u4TlvLen = pTlvTbl[u4TlvIdx].tlvHdl.pfnTlvGetLen(prGlueInfo);
		pTlvTbl[u4TlvIdx].tlvHdl.pfnStstsHdl(ucBssIdx, prGlueInfo,
			ptr, u4TlvLen);
		ptr += (u4TlvLen + sizeof(struct STATS_TRX_TLV_T));
	}
}

#endif /* CFG_SUPPORT_STATISTICS */

/* End of stats.c */
