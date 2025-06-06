/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

//[File]            : wf_uwtbl_top.h
//[Revision time]   : Wed Jun 16 13:34:41 2021
//[Description]     : This file is auto generated by CODA
//[Copyright]       : Copyright (C) 2021 Mediatek Incorportion. All rights reserved.

#ifndef __WF_UWTBL_TOP_REGS_H__
#define __WF_UWTBL_TOP_REGS_H__

#include "hal_common.h"

#ifdef __cplusplus
extern "C" {
#endif


//****************************************************************************
//
//                     WF_UWTBL_TOP CR Definitions
//
//****************************************************************************

#define WF_UWTBL_TOP_BASE                                      0x820c4000

#define WF_UWTBL_TOP_KTVLBR0_ADDR                              (WF_UWTBL_TOP_BASE + 0x0000) // 4000
#define WF_UWTBL_TOP_UCR_ADDR                                  (WF_UWTBL_TOP_BASE + 0x0100) // 4100
#define WF_UWTBL_TOP_WDUCR_ADDR                                (WF_UWTBL_TOP_BASE + 0x0104) // 4104
#define WF_UWTBL_TOP_KTCR_ADDR                                 (WF_UWTBL_TOP_BASE + 0x0108) // 4108
#define WF_UWTBL_TOP_WIUCR_ADDR                                (WF_UWTBL_TOP_BASE + 0x0110) // 4110
#define WF_UWTBL_TOP_WMUDR_ADDR                                (WF_UWTBL_TOP_BASE + 0x0114) // 4114
#define WF_UWTBL_TOP_WMUMR_ADDR                                (WF_UWTBL_TOP_BASE + 0x0118) // 4118
#define WF_UWTBL_TOP_PICR0_ADDR                                (WF_UWTBL_TOP_BASE + 0x0120) // 4120
#define WF_UWTBL_TOP_PICR1_ADDR                                (WF_UWTBL_TOP_BASE + 0x0124) // 4124
#define WF_UWTBL_TOP_ITCR_ADDR                                 (WF_UWTBL_TOP_BASE + 0x0130) // 4130
#define WF_UWTBL_TOP_ITDR0_ADDR                                (WF_UWTBL_TOP_BASE + 0x0138) // 4138
#define WF_UWTBL_TOP_ITDR1_ADDR                                (WF_UWTBL_TOP_BASE + 0x013C) // 413C
#define WF_UWTBL_TOP_DFR_ADDR                                  (WF_UWTBL_TOP_BASE + 0x1FE0) // 5FE0
#define WF_UWTBL_TOP_DMY0_ADDR                                 (WF_UWTBL_TOP_BASE + 0x1FF0) // 5FF0
#define WF_UWTBL_TOP_DMY1_ADDR                                 (WF_UWTBL_TOP_BASE + 0x1FF4) // 5FF4




/* =====================================================================================

  ---KTVLBR0 (0x820c4000 + 0x0000)---

    VLB[31..0]                   - (RW) Valid Lookaside Buffer for key table
                                     Each bit is mapped to 1 entry in key table
                                     The max # = # of wlan_entry + (# of MBSS+ # of BSS) * (# of band)

 =====================================================================================*/
#define WF_UWTBL_TOP_KTVLBR0_VLB_ADDR                          WF_UWTBL_TOP_KTVLBR0_ADDR
#define WF_UWTBL_TOP_KTVLBR0_VLB_MASK                          0xFFFFFFFF                // VLB[31..0]
#define WF_UWTBL_TOP_KTVLBR0_VLB_SHFT                          0

/* =====================================================================================

  ---UCR (0x820c4000 + 0x0100)---

    PN_INCR_MODE[0]              - (RW) The condition of PN+1
    MGMT_SYNC_PNSN[1]            - (RW) synchronize PN/SN across MLD entry
                                     for a MLO device which transmits management frames by primary or secondary MLD entry, they should use a PN/SN in primary MLD entry
    RESERVED2[15..2]             - (RO) Reserved bits
    MAX_BIPN_RANGE[31..16]       - (RW) Max BIPN range
                                     Replay check is checked by the following condition
                                     oldBIPN < newBIPN <= oldBIPN+ MAX_BIPN_RANGE

 =====================================================================================*/
#define WF_UWTBL_TOP_UCR_MAX_BIPN_RANGE_ADDR                   WF_UWTBL_TOP_UCR_ADDR
#define WF_UWTBL_TOP_UCR_MAX_BIPN_RANGE_MASK                   0xFFFF0000                // MAX_BIPN_RANGE[31..16]
#define WF_UWTBL_TOP_UCR_MAX_BIPN_RANGE_SHFT                   16
#define WF_UWTBL_TOP_UCR_MGMT_SYNC_PNSN_ADDR                   WF_UWTBL_TOP_UCR_ADDR
#define WF_UWTBL_TOP_UCR_MGMT_SYNC_PNSN_MASK                   0x00000002                // MGMT_SYNC_PNSN[1]
#define WF_UWTBL_TOP_UCR_MGMT_SYNC_PNSN_SHFT                   1
#define WF_UWTBL_TOP_UCR_PN_INCR_MODE_ADDR                     WF_UWTBL_TOP_UCR_ADDR
#define WF_UWTBL_TOP_UCR_PN_INCR_MODE_MASK                     0x00000001                // PN_INCR_MODE[0]
#define WF_UWTBL_TOP_UCR_PN_INCR_MODE_SHFT                     0

/* =====================================================================================

  ---WDUCR (0x820c4000 + 0x0104)---

    GROUP[5..0]                  - (RW) The selected group of key table/uwtbl
                                     128 entries for each group
    RESERVED6[30..6]             - (RO) Reserved bits
    TARGET[31]                   - (RW) select the target of table

 =====================================================================================*/
#define WF_UWTBL_TOP_WDUCR_TARGET_ADDR                         WF_UWTBL_TOP_WDUCR_ADDR
#define WF_UWTBL_TOP_WDUCR_TARGET_MASK                         0x80000000                // TARGET[31]
#define WF_UWTBL_TOP_WDUCR_TARGET_SHFT                         31
#define WF_UWTBL_TOP_WDUCR_GROUP_ADDR                          WF_UWTBL_TOP_WDUCR_ADDR
#define WF_UWTBL_TOP_WDUCR_GROUP_MASK                          0x0000003F                // GROUP[5..0]
#define WF_UWTBL_TOP_WDUCR_GROUP_SHFT                          0

/* =====================================================================================

  ---KTCR (0x820c4000 + 0x0108)---

    INDEX[12..0]                 - (RW) Target index if write command of KTCR is performed
                                     if OPERATION = search, the free key entry is shown in this field
    RESERVED13[13]               - (RO) Reserved bits
    OPERATION[15..14]            - (RW) The operation if write command of KTCR is performed
    RESERVED16[29..16]           - (RO) Reserved bits
    REASON[31..30]               - (RO) The reason code for the operation of KTCR

 =====================================================================================*/
#define WF_UWTBL_TOP_KTCR_REASON_ADDR                          WF_UWTBL_TOP_KTCR_ADDR
#define WF_UWTBL_TOP_KTCR_REASON_MASK                          0xC0000000                // REASON[31..30]
#define WF_UWTBL_TOP_KTCR_REASON_SHFT                          30
#define WF_UWTBL_TOP_KTCR_OPERATION_ADDR                       WF_UWTBL_TOP_KTCR_ADDR
#define WF_UWTBL_TOP_KTCR_OPERATION_MASK                       0x0000C000                // OPERATION[15..14]
#define WF_UWTBL_TOP_KTCR_OPERATION_SHFT                       14
#define WF_UWTBL_TOP_KTCR_INDEX_ADDR                           WF_UWTBL_TOP_KTCR_ADDR
#define WF_UWTBL_TOP_KTCR_INDEX_MASK                           0x00001FFF                // INDEX[12..0]
#define WF_UWTBL_TOP_KTCR_INDEX_SHFT                           0

/* =====================================================================================

  ---WIUCR (0x820c4000 + 0x0110)---

    INDEX[11..0]                 - (RW) Target index for indirect update
    RESERVED12[12]               - (RO) Reserved bits
    PEERINFO_UPDATE[13]          - (W1) Update peer info according to the value of WIUDR0n
    RESERVED14[17..14]           - (RO) Reserved bits
    PNSN_CLEAR[18]               - (W1) Clear PN/SN* to 0
    RESERVED19[19]               - (RO) Reserved bits
    MASK_UPDATE[20]              - (W1) Mask Update
                                     WTBL loads target wlan_idx & dw and update the target field by WMUDR & WMUMR
    RESERVED21[23..21]           - (RO) Reserved bits
    DW[27..24]                   - (RW) Target DW for indirect update
    RESERVED28[30..28]           - (RO) Reserved bits
    IU_BUSY[31]                  - (RO) Indirect update status
                                     HW will set up this bit when it is updating WTBL.

 =====================================================================================*/
#define WF_UWTBL_TOP_WIUCR_IU_BUSY_ADDR                        WF_UWTBL_TOP_WIUCR_ADDR
#define WF_UWTBL_TOP_WIUCR_IU_BUSY_MASK                        0x80000000                // IU_BUSY[31]
#define WF_UWTBL_TOP_WIUCR_IU_BUSY_SHFT                        31
#define WF_UWTBL_TOP_WIUCR_DW_ADDR                             WF_UWTBL_TOP_WIUCR_ADDR
#define WF_UWTBL_TOP_WIUCR_DW_MASK                             0x0F000000                // DW[27..24]
#define WF_UWTBL_TOP_WIUCR_DW_SHFT                             24
#define WF_UWTBL_TOP_WIUCR_MASK_UPDATE_ADDR                    WF_UWTBL_TOP_WIUCR_ADDR
#define WF_UWTBL_TOP_WIUCR_MASK_UPDATE_MASK                    0x00100000                // MASK_UPDATE[20]
#define WF_UWTBL_TOP_WIUCR_MASK_UPDATE_SHFT                    20
#define WF_UWTBL_TOP_WIUCR_PNSN_CLEAR_ADDR                     WF_UWTBL_TOP_WIUCR_ADDR
#define WF_UWTBL_TOP_WIUCR_PNSN_CLEAR_MASK                     0x00040000                // PNSN_CLEAR[18]
#define WF_UWTBL_TOP_WIUCR_PNSN_CLEAR_SHFT                     18
#define WF_UWTBL_TOP_WIUCR_PEERINFO_UPDATE_ADDR                WF_UWTBL_TOP_WIUCR_ADDR
#define WF_UWTBL_TOP_WIUCR_PEERINFO_UPDATE_MASK                0x00002000                // PEERINFO_UPDATE[13]
#define WF_UWTBL_TOP_WIUCR_PEERINFO_UPDATE_SHFT                13
#define WF_UWTBL_TOP_WIUCR_INDEX_ADDR                          WF_UWTBL_TOP_WIUCR_ADDR
#define WF_UWTBL_TOP_WIUCR_INDEX_MASK                          0x00000FFF                // INDEX[11..0]
#define WF_UWTBL_TOP_WIUCR_INDEX_SHFT                          0

/* =====================================================================================

  ---WMUDR (0x820c4000 + 0x0114)---

    UPDATE_DATA[31..0]           - (RW) Data to update wlan entry

 =====================================================================================*/
#define WF_UWTBL_TOP_WMUDR_UPDATE_DATA_ADDR                    WF_UWTBL_TOP_WMUDR_ADDR
#define WF_UWTBL_TOP_WMUDR_UPDATE_DATA_MASK                    0xFFFFFFFF                // UPDATE_DATA[31..0]
#define WF_UWTBL_TOP_WMUDR_UPDATE_DATA_SHFT                    0

/* =====================================================================================

  ---WMUMR (0x820c4000 + 0x0118)---

    UPDATE_MASK[31..0]           - (RW) Mask of data to update wlan entry

 =====================================================================================*/
#define WF_UWTBL_TOP_WMUMR_UPDATE_MASK_ADDR                    WF_UWTBL_TOP_WMUMR_ADDR
#define WF_UWTBL_TOP_WMUMR_UPDATE_MASK_MASK                    0xFFFFFFFF                // UPDATE_MASK[31..0]
#define WF_UWTBL_TOP_WMUMR_UPDATE_MASK_SHFT                    0

/* =====================================================================================

  ---PICR0 (0x820c4000 + 0x0120)---

    DW0_WTBL[31..0]              - (RW) The value of DW in WTBL for updating
                                     the data are updated to WTBL when WIUCR.PEERINFO_UPDATE is asserted.

 =====================================================================================*/
#define WF_UWTBL_TOP_PICR0_DW0_WTBL_ADDR                       WF_UWTBL_TOP_PICR0_ADDR
#define WF_UWTBL_TOP_PICR0_DW0_WTBL_MASK                       0xFFFFFFFF                // DW0_WTBL[31..0]
#define WF_UWTBL_TOP_PICR0_DW0_WTBL_SHFT                       0

/* =====================================================================================

  ---PICR1 (0x820c4000 + 0x0124)---

    DW1_WTBL[31..0]              - (RW) The value of DW in WTBL for updating
                                     the data are updated to WTBL when WIUCR.PEERINFO_UPDATE is asserted.

 =====================================================================================*/
#define WF_UWTBL_TOP_PICR1_DW1_WTBL_ADDR                       WF_UWTBL_TOP_PICR1_ADDR
#define WF_UWTBL_TOP_PICR1_DW1_WTBL_MASK                       0xFFFFFFFF                // DW1_WTBL[31..0]
#define WF_UWTBL_TOP_PICR1_DW1_WTBL_SHFT                       0

/* =====================================================================================

  ---ITCR (0x820c4000 + 0x0130)---

    INDEX[5..0]                  - (RW) index
    RESERVED6[15..6]             - (RO) Reserved bits
    OP[16]                       - (RW) Operation
    RESERVED17[23..17]           - (RO) Reserved bits
    SELECT[25..24]               - (RW) indirect table select
    RESERVED26[30..26]           - (RO) Reserved bits
    EXECUTE[31]                  - (WO) execute the command

 =====================================================================================*/
#define WF_UWTBL_TOP_ITCR_EXECUTE_ADDR                         WF_UWTBL_TOP_ITCR_ADDR
#define WF_UWTBL_TOP_ITCR_EXECUTE_MASK                         0x80000000                // EXECUTE[31]
#define WF_UWTBL_TOP_ITCR_EXECUTE_SHFT                         31
#define WF_UWTBL_TOP_ITCR_SELECT_ADDR                          WF_UWTBL_TOP_ITCR_ADDR
#define WF_UWTBL_TOP_ITCR_SELECT_MASK                          0x03000000                // SELECT[25..24]
#define WF_UWTBL_TOP_ITCR_SELECT_SHFT                          24
#define WF_UWTBL_TOP_ITCR_OP_ADDR                              WF_UWTBL_TOP_ITCR_ADDR
#define WF_UWTBL_TOP_ITCR_OP_MASK                              0x00010000                // OP[16]
#define WF_UWTBL_TOP_ITCR_OP_SHFT                              16
#define WF_UWTBL_TOP_ITCR_INDEX_ADDR                           WF_UWTBL_TOP_ITCR_ADDR
#define WF_UWTBL_TOP_ITCR_INDEX_MASK                           0x0000003F                // INDEX[5..0]
#define WF_UWTBL_TOP_ITCR_INDEX_SHFT                           0

/* =====================================================================================

  ---ITDR0 (0x820c4000 + 0x0138)---

    INDIRECT_TABLE_DATA[31..0]   - (RW) data for updating indirect table

 =====================================================================================*/
#define WF_UWTBL_TOP_ITDR0_INDIRECT_TABLE_DATA_ADDR            WF_UWTBL_TOP_ITDR0_ADDR
#define WF_UWTBL_TOP_ITDR0_INDIRECT_TABLE_DATA_MASK            0xFFFFFFFF                // INDIRECT_TABLE_DATA[31..0]
#define WF_UWTBL_TOP_ITDR0_INDIRECT_TABLE_DATA_SHFT            0

/* =====================================================================================

  ---ITDR1 (0x820c4000 + 0x013C)---

    INDIRECT_TABLE_DATA[31..0]   - (RW) data for updating indirect table

 =====================================================================================*/
#define WF_UWTBL_TOP_ITDR1_INDIRECT_TABLE_DATA_ADDR            WF_UWTBL_TOP_ITDR1_ADDR
#define WF_UWTBL_TOP_ITDR1_INDIRECT_TABLE_DATA_MASK            0xFFFFFFFF                // INDIRECT_TABLE_DATA[31..0]
#define WF_UWTBL_TOP_ITDR1_INDIRECT_TABLE_DATA_SHFT            0

/* =====================================================================================

  ---DFR (0x820c4000 + 0x1FE0)---

    DEBUG_FSM[31..0]             - (RO) debug fsm

 =====================================================================================*/
#define WF_UWTBL_TOP_DFR_DEBUG_FSM_ADDR                        WF_UWTBL_TOP_DFR_ADDR
#define WF_UWTBL_TOP_DFR_DEBUG_FSM_MASK                        0xFFFFFFFF                // DEBUG_FSM[31..0]
#define WF_UWTBL_TOP_DFR_DEBUG_FSM_SHFT                        0

/* =====================================================================================

  ---DMY0 (0x820c4000 + 0x1FF0)---

    DMY0[31..0]                  - (RW) Dummy Register with default value 0 for ECO purpose

 =====================================================================================*/
#define WF_UWTBL_TOP_DMY0_DMY0_ADDR                            WF_UWTBL_TOP_DMY0_ADDR
#define WF_UWTBL_TOP_DMY0_DMY0_MASK                            0xFFFFFFFF                // DMY0[31..0]
#define WF_UWTBL_TOP_DMY0_DMY0_SHFT                            0

/* =====================================================================================

  ---DMY1 (0x820c4000 + 0x1FF4)---

    DMY1[31..0]                  - (RW) Dummy Register with default value 1 for ECO purpose

 =====================================================================================*/
#define WF_UWTBL_TOP_DMY1_DMY1_ADDR                            WF_UWTBL_TOP_DMY1_ADDR
#define WF_UWTBL_TOP_DMY1_DMY1_MASK                            0xFFFFFFFF                // DMY1[31..0]
#define WF_UWTBL_TOP_DMY1_DMY1_SHFT                            0

#ifdef __cplusplus
}
#endif

#endif // __WF_UWTBL_TOP_REGS_H__
