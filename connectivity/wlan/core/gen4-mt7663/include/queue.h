/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

/*
 ** Id: //Department/DaVinci/BRANCHES/
 *      MT6620_WIFI_DRIVER_V2_3/include/queue.h#1
 */

/*! \file   queue.h
 *    \brief  Definition for singly queue operations.
 *
 *    In this file we define the singly queue data structure and its
 *    queue operation MACROs.
 */

#ifndef _QUEUE_H
#define _QUEUE_H

/*******************************************************************************
 *                         C O M P I L E R   F L A G S
 *******************************************************************************
 */

/*******************************************************************************
 *                    E X T E R N A L   R E F E R E N C E S
 *******************************************************************************
 */
#include "gl_typedef.h"

/*******************************************************************************
 *                              C O N S T A N T S
 *******************************************************************************
 */

/*******************************************************************************
 *                             D A T A   T Y P E S
 *******************************************************************************
 */
/* Singly Queue Structures - Entry Part */
struct QUE_ENTRY {
	struct QUE_ENTRY *prNext;
	struct QUE_ENTRY
		*prPrev;	/* For Rx buffer reordering used only */
};

/* Singly Queue Structures - Queue Part */
struct QUE {
	struct QUE_ENTRY *prHead;
	struct QUE_ENTRY *prTail;
	uint32_t u4NumElem;
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
#define MAXNUM_TDLS_PEER            4

#define QUEUE_INITIALIZE(prQueue) \
	{ \
	    (prQueue)->prHead = (struct QUE_ENTRY *)NULL; \
	    (prQueue)->prTail = (struct QUE_ENTRY *)NULL; \
	    (prQueue)->u4NumElem = 0; \
	}

#define QUEUE_IS_EMPTY(prQueue)	\
	(((struct QUE *)(prQueue))->prHead == (struct QUE_ENTRY *)NULL)

#define QUEUE_IS_NOT_EMPTY(prQueue)         ((prQueue)->u4NumElem > 0)

#define QUEUE_GET_HEAD(prQueue)             ((prQueue)->prHead)

#define QUEUE_GET_TAIL(prQueue)             ((prQueue)->prTail)

#define QUEUE_GET_NEXT_ENTRY(prQueueEntry)  ((prQueueEntry)->prNext)

#define QUEUE_INSERT_HEAD(prQueue, prQueueEntry) \
	{ \
		ASSERT(prQueue); \
		ASSERT(prQueueEntry); \
		(prQueueEntry)->prNext = (prQueue)->prHead; \
		(prQueue)->prHead = (prQueueEntry); \
		if ((prQueue)->prTail == (struct QUE_ENTRY *)NULL) { \
			(prQueue)->prTail = (prQueueEntry); \
		} \
		((prQueue)->u4NumElem)++; \
	}

#define QUEUE_INSERT_TAIL(prQueue, prQueueEntry) \
	{ \
		ASSERT(prQueue); \
	  ASSERT(prQueueEntry); \
	  (prQueueEntry)->prNext = (struct QUE_ENTRY *)NULL; \
		if ((prQueue)->prTail) { \
			((prQueue)->prTail)->prNext = (prQueueEntry); \
		} else { \
			(prQueue)->prHead = (prQueueEntry); \
		} \
		(prQueue)->prTail = (prQueueEntry); \
		((prQueue)->u4NumElem)++; \
	}

/* NOTE: We assume the queue entry located at the beginning
 * of "prQueueEntry Type",
 * so that we can cast the queue entry to other data type without doubts.
 * And this macro also decrease the total entry count at the same time.
 */
#define QUEUE_REMOVE_HEAD(prQueue, prQueueEntry, _P_TYPE) \
	{ \
		ASSERT(prQueue); \
		prQueueEntry = (_P_TYPE)((prQueue)->prHead); \
		if (prQueueEntry) { \
			(prQueue)->prHead = \
				((struct QUE_ENTRY *)(prQueueEntry))->prNext; \
			if ((prQueue)->prHead == (struct QUE_ENTRY *)NULL) { \
				(prQueue)->prTail = (struct QUE_ENTRY *)NULL; \
			} \
			((struct QUE_ENTRY *)(prQueueEntry))->prNext = \
				(struct QUE_ENTRY *)NULL; \
			((prQueue)->u4NumElem)--; \
		} \
	}

#define QUEUE_MOVE_ALL(prDestQueue, prSrcQueue) \
	{ \
		ASSERT(prDestQueue); \
		ASSERT(prSrcQueue); \
	    *(struct QUE *)prDestQueue = *(struct QUE *)prSrcQueue; \
	    QUEUE_INITIALIZE(prSrcQueue); \
	}

#define QUEUE_CONCATENATE_QUEUES(prDestQueue, prSrcQueue) \
	{ \
	    ASSERT(prDestQueue); \
	    ASSERT(prSrcQueue); \
		if ((prSrcQueue)->u4NumElem > 0) { \
			if ((prDestQueue)->prTail) { \
				((prDestQueue)->prTail)->prNext = \
					(prSrcQueue)->prHead; \
			} else { \
				(prDestQueue)->prHead = (prSrcQueue)->prHead; \
			} \
			(prDestQueue)->prTail = (prSrcQueue)->prTail; \
			((prDestQueue)->u4NumElem) += \
				((prSrcQueue)->u4NumElem); \
			QUEUE_INITIALIZE(prSrcQueue); \
	    } \
	}

#define QUEUE_CONCATENATE_QUEUES_HEAD(prDestQueue, prSrcQueue) \
	{ \
		ASSERT(prDestQueue); \
		ASSERT(prSrcQueue); \
		if ((prSrcQueue)->u4NumElem > 0) { \
			((prSrcQueue)->prTail)->prNext = \
				(prDestQueue)->prHead; \
			(prDestQueue)->prHead = (prSrcQueue)->prHead; \
			((prDestQueue)->u4NumElem) += \
				((prSrcQueue)->u4NumElem); \
			if ((prDestQueue)->prTail == NULL) {                 \
				(prDestQueue)->prTail = (prSrcQueue)->prTail; \
			}  \
			QUEUE_INITIALIZE(prSrcQueue); \
		} \
	}

/*******************************************************************************
 *                            E X T E R N A L  D A T A
 *******************************************************************************
 */
extern uint8_t g_arTdlsLink[MAXNUM_TDLS_PEER];

/*******************************************************************************
 *                  F U N C T I O N   D E C L A R A T I O N S
 *******************************************************************************
 */

/*******************************************************************************
 *                              F U N C T I O N S
 *******************************************************************************
 */

#endif /* _QUEUE_H */
