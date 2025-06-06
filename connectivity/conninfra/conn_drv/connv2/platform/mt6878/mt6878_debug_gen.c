/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2023 MediaTek Inc.
 */


/* AUTO-GENERATED FILE.  DO NOT MODIFY.
 *
 * This file, mt6878_debug_gen.c was automatically generated
 * by the tool from the DEBUG data DE provided.
 * It should not be modified by hand.
 *
 * Reference debug file,
 * - [Rxxxs] connsys_power_debug_20240105.xlsx (Modified date: 2024-01-05)
 * - [Rxxxs]conn_infra_bus_debug_ctrl_20240105.xlsx (Modified date: 2023-08-18)
 */


#ifndef CFG_CONNINFRA_ON_CTP
#include "../include/plat_library.h"
#endif
#include "../include/consys_hw.h"
#include "../include/consys_reg_util.h"
#include "../include/plat_def.h"
#include "include/mt6878_consys_reg_offset.h"
#include "include/mt6878_debug_gen.h"


mapped_addr vir_addr_consys_dbg_gen_srclken_rc_base_mt6878;
mapped_addr vir_addr_consys_dbg_gen_cksys_reg_base_mt6878;
mapped_addr vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878;
mapped_addr vir_addr_consys_dbg_gen_conn_wt_slp_ctl_reg_base_mt6878;
mapped_addr vir_addr_consys_dbg_gen_inst2_conn_wt_slp_ctl_reg_base_mt6878;
mapped_addr vir_addr_consys_dbg_gen_conn_rf_spi_mst_reg_base_mt6878;
mapped_addr vir_addr_consys_dbg_gen_conn_infra_sysram_offset_mt6878;
mapped_addr vir_addr_consys_dbg_gen_inst2_conn_rf_spi_mst_reg_base_mt6878;
mapped_addr vir_addr_consys_dbg_gen_conn_infra_axi_layer_bus_bcrm_base_mt6878;

void consys_debug_init_mt6878_debug_gen(void)
{
	vir_addr_consys_dbg_gen_srclken_rc_base_mt6878
		= ioremap(CONSYS_DBG_GEN_SRCLKEN_RC_BASE_ADDR, 0x77C);
	vir_addr_consys_dbg_gen_cksys_reg_base_mt6878
		= ioremap(CONSYS_DBG_GEN_CKSYS_REG_BASE_ADDR, 0xF0);
	vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878
		= ioremap(CONSYS_DBG_GEN_CONN_DBG_CTL_BASE_ADDR, 0x41c);
	vir_addr_consys_dbg_gen_conn_wt_slp_ctl_reg_base_mt6878
		= ioremap(CONSYS_DBG_GEN_CONN_WT_SLP_CTL_REG_BASE_ADDR, 0x134);
	vir_addr_consys_dbg_gen_inst2_conn_wt_slp_ctl_reg_base_mt6878
		= ioremap(CONSYS_DBG_GEN_INST2_CONN_WT_SLP_CTL_REG_BASE_ADDR, 0x134);
	vir_addr_consys_dbg_gen_conn_rf_spi_mst_reg_base_mt6878
		= ioremap(CONSYS_DBG_GEN_CONN_RF_SPI_MST_REG_BASE_ADDR, 0x320);
	vir_addr_consys_dbg_gen_conn_infra_sysram_offset_mt6878
		= ioremap(CONSYS_DBG_GEN_CONN_INFRA_SYSRAM_BASE_OFFSET_ADDR, 0xE9C);
	vir_addr_consys_dbg_gen_inst2_conn_rf_spi_mst_reg_base_mt6878
		= ioremap(CONSYS_DBG_GEN_INST2_CONN_RF_SPI_MST_REG_BASE_ADDR, 0x108);
	vir_addr_consys_dbg_gen_conn_infra_axi_layer_bus_bcrm_base_mt6878
		= ioremap(CONSYS_DBG_GEN_CONN_INFRA_AXI_LAYER_BUS_BCRM_BASE_ADDR, 0xc);
}

void consys_debug_deinit_mt6878_debug_gen(void)
{
	if (vir_addr_consys_dbg_gen_srclken_rc_base_mt6878)
		iounmap(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878);

	if (vir_addr_consys_dbg_gen_cksys_reg_base_mt6878)
		iounmap(vir_addr_consys_dbg_gen_cksys_reg_base_mt6878);

	if (vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878)
		iounmap(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878);

	if (vir_addr_consys_dbg_gen_conn_wt_slp_ctl_reg_base_mt6878)
		iounmap(vir_addr_consys_dbg_gen_conn_wt_slp_ctl_reg_base_mt6878);

	if (vir_addr_consys_dbg_gen_inst2_conn_wt_slp_ctl_reg_base_mt6878)
		iounmap(vir_addr_consys_dbg_gen_inst2_conn_wt_slp_ctl_reg_base_mt6878);

	if (vir_addr_consys_dbg_gen_conn_rf_spi_mst_reg_base_mt6878)
		iounmap(vir_addr_consys_dbg_gen_conn_rf_spi_mst_reg_base_mt6878);

	if (vir_addr_consys_dbg_gen_conn_infra_sysram_offset_mt6878)
		iounmap(vir_addr_consys_dbg_gen_conn_infra_sysram_offset_mt6878);

	if (vir_addr_consys_dbg_gen_inst2_conn_rf_spi_mst_reg_base_mt6878)
		iounmap(vir_addr_consys_dbg_gen_inst2_conn_rf_spi_mst_reg_base_mt6878);

	if (vir_addr_consys_dbg_gen_conn_infra_axi_layer_bus_bcrm_base_mt6878)
		iounmap(vir_addr_consys_dbg_gen_conn_infra_axi_layer_bus_bcrm_base_mt6878);
}

void update_debug_read_info_mt6878_debug_gen(
		struct conn_debug_info_mt6878 *info,
		char *tag,
		unsigned int rd_addr,
		unsigned int rd_data)
{
	if (info == NULL)
		return;

	if (info->length >= CONN_DEBUG_INFO_SIZE)
		return;

	if (tag != NULL) {
		if (snprintf(&(info->tag[info->length][0]), DEBUG_TAG_SIZE, "%s", tag) < 0)
			pr_notice("%s snprintf fail", __func__);
	}

	info->rd_addr[info->length] = rd_addr;
	info->rd_data[info->length] = rd_data;
	info->length++;
}

void update_debug_write_info_mt6878_debug_gen(
		struct conn_debug_info_mt6878 *info,
		char *tag,
		unsigned int wr_addr,
		int wr_addr_lsb,
		int wr_addr_msb,
		unsigned int wr_data)
{
	int len = 0;

	if (info == NULL)
		return;

	if (info->wr_addr[info->length] != 0)
		info->length++;

	if (info->length >= CONN_DEBUG_INFO_SIZE)
		return;

	if (tag != NULL)
		len = snprintf(&(info->tag[info->length][0]), DEBUG_TAG_SIZE, "%s", tag);
	else
		len = snprintf(&(info->tag[info->length][0]), DEBUG_TAG_SIZE, "%s", "");

	if (len < 0)
		pr_notice("%s snprintf fail", __func__);

	info->wr_addr[info->length] = wr_addr;
	info->wr_addr_lsb[info->length] = wr_addr_lsb;
	info->wr_addr_msb[info->length] = wr_addr_msb;
	info->wr_data[info->length] = wr_data;
}

void consys_print_power_debug_dbg_level_0_mt6878_debug_gen(
		int level,
		struct conn_debug_info_mt6878 *pdbg_level_0_info)
{
	if (pdbg_level_0_info == NULL)
		return;

	memset(pdbg_level_0_info, 0, sizeof(struct conn_debug_info_mt6878));

	if (level < 0)
		return;

	if (!vir_addr_consys_dbg_gen_srclken_rc_base_mt6878) {
		pr_notice("vir_addr_consys_dbg_gen_srclken_rc_base_mt6878(%x) ioremap fail\n",
				CONSYS_DBG_GEN_SRCLKEN_RC_BASE_ADDR);
		return;
	}

	if (!vir_addr_consys_dbg_gen_cksys_reg_base_mt6878) {
		pr_notice("vir_addr_consys_dbg_gen_cksys_reg_base_mt6878(%x) ioremap fail\n",
				CONSYS_DBG_GEN_CKSYS_REG_BASE_ADDR);
		return;
	}

	if (SPM_REG_BASE == 0) {
		pr_notice("SPM_REG_BASE is not defined\n");
		return;
	}

	/* A0 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A0", 0x1C001000 + CONSYS_DBG_GEN_SPM_REQ_STA_4_OFFSET_ADDR,
		CONSYS_REG_READ(SPM_REG_BASE +
			CONSYS_DBG_GEN_SPM_REQ_STA_4_OFFSET_ADDR));

	/* A1 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A1", 0x1C00D000 + CONSYS_DBG_GEN_FSM_STA_0_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_FSM_STA_0_OFFSET_ADDR));

	/* A2 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A2", 0x1C00D000 + CONSYS_DBG_GEN_CMD_STA_0_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_CMD_STA_0_OFFSET_ADDR));

	/* A3 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A4", 0x1C00D000 + CONSYS_DBG_GEN_CMD_STA_1_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_CMD_STA_1_OFFSET_ADDR));

	/* A4 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A4", 0x1C00D000 + CONSYS_DBG_GEN_SPI_STA_0_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_SPI_STA_0_OFFSET_ADDR));

	/* A5 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A5", 0x1C00D000 + CONSYS_DBG_GEN_PI_PO_STA_0_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_PI_PO_STA_0_OFFSET_ADDR));

	/* A6 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A6", 0x1C00D000 + CONSYS_DBG_GEN_M00_REQ_STA_0_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_M00_REQ_STA_0_OFFSET_ADDR));

	/* A7 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A7", 0x1C00D000 + CONSYS_DBG_GEN_M01_REQ_STA_0_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_M01_REQ_STA_0_OFFSET_ADDR));

	/* A8 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A8", 0x1C00D000 + CONSYS_DBG_GEN_M02_REQ_STA_0_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_M02_REQ_STA_0_OFFSET_ADDR));

	/* A9 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A9", 0x1C00D000 + CONSYS_DBG_GEN_M03_REQ_STA_0_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_M03_REQ_STA_0_OFFSET_ADDR));

	/* A10 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A10", 0x1C00D000 + CONSYS_DBG_GEN_M04_REQ_STA_0_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_M04_REQ_STA_0_OFFSET_ADDR));

	/* A11 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A11", 0x1C00D000 + CONSYS_DBG_GEN_M05_REQ_STA_0_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_M05_REQ_STA_0_OFFSET_ADDR));

	/* A12 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A12", 0x1C00D000 + CONSYS_DBG_GEN_M06_REQ_STA_0_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_M06_REQ_STA_0_OFFSET_ADDR));

	/* A13 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A13", 0x1C00D000 + CONSYS_DBG_GEN_M07_REQ_STA_0_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_M07_REQ_STA_0_OFFSET_ADDR));

	/* A14 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A14", 0x1C00D000 + CONSYS_DBG_GEN_M08_REQ_STA_0_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_M08_REQ_STA_0_OFFSET_ADDR));

	/* A15 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A15", 0x1C00D000 + CONSYS_DBG_GEN_M09_REQ_STA_0_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_M09_REQ_STA_0_OFFSET_ADDR));

	/* A16 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A16", 0x1C00D000 + CONSYS_DBG_GEN_M10_REQ_STA_0_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_M10_REQ_STA_0_OFFSET_ADDR));

	/* A17 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A17", 0x1C00D000 + CONSYS_DBG_GEN_M11_REQ_STA_0_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_M11_REQ_STA_0_OFFSET_ADDR));

	/* A18 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A18", 0x1C00D000 + CONSYS_DBG_GEN_M12_REQ_STA_0_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_M12_REQ_STA_0_OFFSET_ADDR));

	/* A19 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A19", 0x1C00D000 + CONSYS_DBG_GEN_M13_REQ_STA_0_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_M13_REQ_STA_0_OFFSET_ADDR));

	/* A20 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A20", 0x1C00D000 + CONSYS_DBG_GEN_DEBUG_STA_0_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_DEBUG_STA_0_OFFSET_ADDR));

	/* A21 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A21", 0x1C00D000 + CONSYS_DBG_GEN_SPMI_P_STA_0_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_SPMI_P_STA_0_OFFSET_ADDR));

	/* A22 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A22", 0x1C001000 + CONSYS_DBG_GEN_CONN_PWR_CON_OFFSET_ADDR,
		CONSYS_REG_READ(SPM_REG_BASE +
			CONSYS_DBG_GEN_CONN_PWR_CON_OFFSET_ADDR));

	/* A23 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A23", 0x10000000 + CONSYS_DBG_GEN_CLK_CFG_14_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_cksys_reg_base_mt6878 +
			CONSYS_DBG_GEN_CLK_CFG_14_OFFSET_ADDR));

	/* A24 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A24", 0x1C00D000 + CONSYS_DBG_GEN_DEBUG_TRACE_0_LSB_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_DEBUG_TRACE_0_LSB_OFFSET_ADDR));

	/* A25 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A25", 0x1C00D000 + CONSYS_DBG_GEN_DEBUG_TRACE_0_MSB_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_DEBUG_TRACE_0_MSB_OFFSET_ADDR));

	/* A26 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A26", 0x1C00D000 + CONSYS_DBG_GEN_DEBUG_TRACE_1_LSB_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_DEBUG_TRACE_1_LSB_OFFSET_ADDR));

	/* A27 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A27", 0x1C00D000 + CONSYS_DBG_GEN_DEBUG_TRACE_1_MSB_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_DEBUG_TRACE_1_MSB_OFFSET_ADDR));

	/* A28 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A28", 0x1C00D000 + CONSYS_DBG_GEN_DEBUG_TRACE_2_LSB_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_DEBUG_TRACE_2_LSB_OFFSET_ADDR));

	/* A29 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A29", 0x1C00D000 + CONSYS_DBG_GEN_DEBUG_TRACE_2_MSB_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_DEBUG_TRACE_2_MSB_OFFSET_ADDR));

	/* A30 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A30", 0x1C00D000 + CONSYS_DBG_GEN_DEBUG_TRACE_3_LSB_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_DEBUG_TRACE_3_LSB_OFFSET_ADDR));

	/* A31 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A31", 0x1C00D000 + CONSYS_DBG_GEN_DEBUG_TRACE_3_MSB_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_DEBUG_TRACE_3_MSB_OFFSET_ADDR));

	/* A32 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A32", 0x1C00D000 + CONSYS_DBG_GEN_DEBUG_TRACE_4_LSB_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_DEBUG_TRACE_4_LSB_OFFSET_ADDR));

	/* A33 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A33", 0x1C00D000 + CONSYS_DBG_GEN_DEBUG_TRACE_4_MSB_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_DEBUG_TRACE_4_MSB_OFFSET_ADDR));

	/* A34 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A34", 0x1C00D000 + CONSYS_DBG_GEN_DEBUG_TRACE_5_LSB_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_DEBUG_TRACE_5_LSB_OFFSET_ADDR));

	/* A35 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A35", 0x1C00D000 + CONSYS_DBG_GEN_DEBUG_TRACE_5_MSB_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_DEBUG_TRACE_5_MSB_OFFSET_ADDR));

	/* A36 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A36", 0x1C00D000 + CONSYS_DBG_GEN_DEBUG_TRACE_6_LSB_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_DEBUG_TRACE_6_LSB_OFFSET_ADDR));

	/* A37 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A37", 0x1C00D000 + CONSYS_DBG_GEN_DEBUG_TRACE_6_MSB_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_DEBUG_TRACE_6_MSB_OFFSET_ADDR));

	/* A38 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A38", 0x1C00D000 + CONSYS_DBG_GEN_DEBUG_TRACE_7_LSB_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_DEBUG_TRACE_7_LSB_OFFSET_ADDR));

	/* A39 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A39", 0x1C00D000 + CONSYS_DBG_GEN_DEBUG_TRACE_7_MSB_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_DEBUG_TRACE_7_MSB_OFFSET_ADDR));

	/* A40 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A40", 0x1C00D000 + CONSYS_DBG_GEN_SYS_TIMER_LATCH_0_LSB_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_SYS_TIMER_LATCH_0_LSB_OFFSET_ADDR));

	/* A41 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A41", 0x1C00D000 + CONSYS_DBG_GEN_SYS_TIMER_LATCH_0_MSB_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_SYS_TIMER_LATCH_0_MSB_OFFSET_ADDR));

	/* A42 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A42", 0x1C00D000 + CONSYS_DBG_GEN_SYS_TIMER_LATCH_1_LSB_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_SYS_TIMER_LATCH_1_LSB_OFFSET_ADDR));

	/* A43 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A43", 0x1C00D000 + CONSYS_DBG_GEN_SYS_TIMER_LATCH_1_MSB_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_SYS_TIMER_LATCH_1_MSB_OFFSET_ADDR));

	/* A44 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A44", 0x1C00D000 + CONSYS_DBG_GEN_SYS_TIMER_LATCH_2_LSB_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_SYS_TIMER_LATCH_2_LSB_OFFSET_ADDR));

	/* A45 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A45", 0x1C00D000 + CONSYS_DBG_GEN_SYS_TIMER_LATCH_2_MSB_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_SYS_TIMER_LATCH_2_MSB_OFFSET_ADDR));

	/* A46 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A46", 0x1C00D000 + CONSYS_DBG_GEN_SYS_TIMER_LATCH_3_LSB_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_SYS_TIMER_LATCH_3_LSB_OFFSET_ADDR));

	/* A47 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A47", 0x1C00D000 + CONSYS_DBG_GEN_SYS_TIMER_LATCH_3_MSB_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_SYS_TIMER_LATCH_3_MSB_OFFSET_ADDR));

	/* A48 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A48", 0x1C00D000 + CONSYS_DBG_GEN_SYS_TIMER_LATCH_4_LSB_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_SYS_TIMER_LATCH_4_LSB_OFFSET_ADDR));

	/* A49 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A49", 0x1C00D000 + CONSYS_DBG_GEN_SYS_TIMER_LATCH_4_MSB_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_SYS_TIMER_LATCH_4_MSB_OFFSET_ADDR));

	/* A50 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A50", 0x1C00D000 + CONSYS_DBG_GEN_SYS_TIMER_LATCH_5_LSB_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_SYS_TIMER_LATCH_5_LSB_OFFSET_ADDR));

	/* A51 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A51", 0x1C00D000 + CONSYS_DBG_GEN_SYS_TIMER_LATCH_5_MSB_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_SYS_TIMER_LATCH_5_MSB_OFFSET_ADDR));

	/* A52 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A52", 0x1C00D000 + CONSYS_DBG_GEN_SYS_TIMER_LATCH_6_LSB_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_SYS_TIMER_LATCH_6_LSB_OFFSET_ADDR));

	/* A53 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A53", 0x1C00D000 + CONSYS_DBG_GEN_SYS_TIMER_LATCH_6_MSB_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_SYS_TIMER_LATCH_6_MSB_OFFSET_ADDR));

	/* A54 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A54", 0x1C00D000 + CONSYS_DBG_GEN_SYS_TIMER_LATCH_7_LSB_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_SYS_TIMER_LATCH_7_LSB_OFFSET_ADDR));

	/* A55 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A55", 0x1C00D000 + CONSYS_DBG_GEN_SYS_TIMER_LATCH_7_MSB_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_SYS_TIMER_LATCH_7_MSB_OFFSET_ADDR));

	/* A56 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"A56", 0x1C00D000 + CONSYS_DBG_GEN_CMD_STA_0_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_CMD_STA_0_OFFSET_ADDR));
}

void consys_print_power_debug_dbg_level_1_mt6878_debug_gen(
		int level,
		struct conn_debug_info_mt6878 *pdbg_level_1_info)
{
	if (pdbg_level_1_info == NULL)
		return;

	memset(pdbg_level_1_info, 0, sizeof(struct conn_debug_info_mt6878));

	if (level < 1)
		return;

	if (!vir_addr_consys_dbg_gen_srclken_rc_base_mt6878) {
		pr_notice("vir_addr_consys_dbg_gen_srclken_rc_base_mt6878(%x) ioremap fail\n",
				CONSYS_DBG_GEN_SRCLKEN_RC_BASE_ADDR);
		return;
	}

	if (!vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878) {
		pr_notice("vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878(%x) ioremap fail\n",
				CONSYS_DBG_GEN_CONN_DBG_CTL_BASE_ADDR);
		return;
	}

	if (CONN_HOST_CSR_TOP_BASE == 0) {
		pr_notice("CONN_HOST_CSR_TOP_BASE is not defined\n");
		return;
	}

	/* B0 */
	CONSYS_REG_WRITE_MASK(CONN_HOST_CSR_TOP_BASE +
		CONSYS_DBG_GEN_CR_CONN_INFRA_CFG_ON_DBG_MUX_SEL_OFFSET_ADDR, 0x0, 0x7);
	update_debug_write_info_mt6878_debug_gen(pdbg_level_1_info,
		"B0", 0x18060000 + CONSYS_DBG_GEN_CR_CONN_INFRA_CFG_ON_DBG_MUX_SEL_OFFSET_ADDR,
		0, 2, 0x0);

	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		NULL, 0x18060000 + CONSYS_DBG_GEN_CONN_INFRA_CFG_ON_DBG_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_HOST_CSR_TOP_BASE +
			CONSYS_DBG_GEN_CONN_INFRA_CFG_ON_DBG_OFFSET_ADDR));

	/* B1 */
	CONSYS_REG_WRITE_MASK(CONN_HOST_CSR_TOP_BASE +
		CONSYS_DBG_GEN_CR_CONN_INFRA_CFG_ON_DBG_MUX_SEL_OFFSET_ADDR, 0x4, 0x7);
	update_debug_write_info_mt6878_debug_gen(pdbg_level_1_info,
		"B1", 0x18060000 + CONSYS_DBG_GEN_CR_CONN_INFRA_CFG_ON_DBG_MUX_SEL_OFFSET_ADDR,
		0, 2, 0x4);

	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		NULL, 0x18060000 + CONSYS_DBG_GEN_CONN_INFRA_CFG_ON_DBG_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_HOST_CSR_TOP_BASE +
			CONSYS_DBG_GEN_CONN_INFRA_CFG_ON_DBG_OFFSET_ADDR));

	/* B2 */
	CONSYS_REG_WRITE_MASK(CONN_HOST_CSR_TOP_BASE +
		CONSYS_DBG_GEN_CR_CONN_INFRA_CFG_ON_DBG_MUX_SEL_OFFSET_ADDR, 0x5, 0x7);
	update_debug_write_info_mt6878_debug_gen(pdbg_level_1_info,
		"B2", 0x18060000 + CONSYS_DBG_GEN_CR_CONN_INFRA_CFG_ON_DBG_MUX_SEL_OFFSET_ADDR,
		0, 2, 0x5);

	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		NULL, 0x18060000 + CONSYS_DBG_GEN_CONN_INFRA_CFG_ON_DBG_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_HOST_CSR_TOP_BASE +
			CONSYS_DBG_GEN_CONN_INFRA_CFG_ON_DBG_OFFSET_ADDR));

	/* B3 */
	CONSYS_REG_WRITE_MASK(CONN_HOST_CSR_TOP_BASE +
		CONSYS_DBG_GEN_CR_CONN_INFRA_CFG_ON_DBG_MUX_SEL_OFFSET_ADDR, 0x1, 0x7);
	update_debug_write_info_mt6878_debug_gen(pdbg_level_1_info,
		"B3", 0x18060000 + CONSYS_DBG_GEN_CR_CONN_INFRA_CFG_ON_DBG_MUX_SEL_OFFSET_ADDR,
		0, 2, 0x1);

	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		NULL, 0x18060000 + CONSYS_DBG_GEN_CONN_INFRA_CFG_ON_DBG_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_HOST_CSR_TOP_BASE +
			CONSYS_DBG_GEN_CONN_INFRA_CFG_ON_DBG_OFFSET_ADDR));

	/* B4 */
	CONSYS_REG_WRITE_MASK(CONN_HOST_CSR_TOP_BASE +
		CONSYS_DBG_GEN_CR_CONN_INFRA_CFG_ON_DBG_MUX_SEL_OFFSET_ADDR, 0x2, 0x7);
	update_debug_write_info_mt6878_debug_gen(pdbg_level_1_info,
		"B4", 0x18060000 + CONSYS_DBG_GEN_CR_CONN_INFRA_CFG_ON_DBG_MUX_SEL_OFFSET_ADDR,
		0, 2, 0x2);

	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		NULL, 0x18060000 + CONSYS_DBG_GEN_CONN_INFRA_CFG_ON_DBG_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_HOST_CSR_TOP_BASE +
			CONSYS_DBG_GEN_CONN_INFRA_CFG_ON_DBG_OFFSET_ADDR));

	/* B5 */
	CONSYS_REG_WRITE_MASK(CONN_HOST_CSR_TOP_BASE +
		CONSYS_DBG_GEN_CR_CONN_INFRA_CFG_ON_DBG_MUX_SEL_OFFSET_ADDR, 0x3, 0x7);
	update_debug_write_info_mt6878_debug_gen(pdbg_level_1_info,
		"B5", 0x18060000 + CONSYS_DBG_GEN_CR_CONN_INFRA_CFG_ON_DBG_MUX_SEL_OFFSET_ADDR,
		0, 2, 0x3);

	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		NULL, 0x18060000 + CONSYS_DBG_GEN_CONN_INFRA_CFG_ON_DBG_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_HOST_CSR_TOP_BASE +
			CONSYS_DBG_GEN_CONN_INFRA_CFG_ON_DBG_OFFSET_ADDR));

	/* B6 */
	CONSYS_REG_WRITE_MASK(CONN_HOST_CSR_TOP_BASE +
		CONSYS_DBG_GEN_CR_CONN_INFRA_CFG_ON_DBG_MUX_SEL_OFFSET_ADDR, 0x6, 0x7);
	update_debug_write_info_mt6878_debug_gen(pdbg_level_1_info,
		"B6", 0x18060000 + CONSYS_DBG_GEN_CR_CONN_INFRA_CFG_ON_DBG_MUX_SEL_OFFSET_ADDR,
		0, 2, 0x6);

	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		NULL, 0x18060000 + CONSYS_DBG_GEN_CONN_INFRA_CFG_ON_DBG_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_HOST_CSR_TOP_BASE +
			CONSYS_DBG_GEN_CONN_INFRA_CFG_ON_DBG_OFFSET_ADDR));

	/* B7 */
	CONSYS_REG_WRITE_MASK(CONN_HOST_CSR_TOP_BASE +
		CONSYS_DBG_GEN_CR_CONN_INFRA_CFG_ON_DBG_MUX_SEL_OFFSET_ADDR, 0x7, 0x7);
	update_debug_write_info_mt6878_debug_gen(pdbg_level_1_info,
		"B7", 0x18060000 + CONSYS_DBG_GEN_CR_CONN_INFRA_CFG_ON_DBG_MUX_SEL_OFFSET_ADDR,
		0, 2, 0x7);

	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		NULL, 0x18060000 + CONSYS_DBG_GEN_CONN_INFRA_CFG_ON_DBG_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_HOST_CSR_TOP_BASE +
			CONSYS_DBG_GEN_CONN_INFRA_CFG_ON_DBG_OFFSET_ADDR));

	/* B8 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		"B8", 0x1C00D000 + CONSYS_DBG_GEN_M10_REQ_STA_0_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_M10_REQ_STA_0_OFFSET_ADDR));

	/* B9 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		"B9", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_MONFLAG_OUT_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
			CONSYS_DBG_GEN_CONN_INFRA_MONFLAG_OUT_OFFSET_ADDR));

	/* B10 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		"B10", 0x18060000 + CONSYS_DBG_GEN_CONNSYS_PWR_STATES_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_HOST_CSR_TOP_BASE +
			CONSYS_DBG_GEN_CONNSYS_PWR_STATES_OFFSET_ADDR));

	/* B11 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		"B11", 0x18060000 + CONSYS_DBG_GEN_CONN_INFRA_WAKEPU_TOP_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_HOST_CSR_TOP_BASE +
			CONSYS_DBG_GEN_CONN_INFRA_WAKEPU_TOP_OFFSET_ADDR));

	/* B12 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		"B12", 0x18060000 + CONSYS_DBG_GEN_CONN_INFRA_WAKEPU_WF_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_HOST_CSR_TOP_BASE +
			CONSYS_DBG_GEN_CONN_INFRA_WAKEPU_WF_OFFSET_ADDR));

	/* B13 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		"B13", 0x18060000 + CONSYS_DBG_GEN_CONN_INFRA_WAKEPU_BT_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_HOST_CSR_TOP_BASE +
			CONSYS_DBG_GEN_CONN_INFRA_WAKEPU_BT_OFFSET_ADDR));

	/* B14 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		"B14", 0x18060000 + CONSYS_DBG_GEN_CONN_INFRA_WAKEPU_GPS_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_HOST_CSR_TOP_BASE +
			CONSYS_DBG_GEN_CONN_INFRA_WAKEPU_GPS_OFFSET_ADDR));

	/* B15 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		"B15", 0x18060000 + CONSYS_DBG_GEN_CONN_INFRA_WAKEPU_FM_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_HOST_CSR_TOP_BASE +
			CONSYS_DBG_GEN_CONN_INFRA_WAKEPU_FM_OFFSET_ADDR));
}

void consys_print_power_debug_dbg_level_2_mt6878_debug_gen(
		int level,
		struct conn_debug_info_mt6878 *pdbg_level_2_info)
{
	if (pdbg_level_2_info == NULL)
		return;

	memset(pdbg_level_2_info, 0, sizeof(struct conn_debug_info_mt6878));

	if (level < 2)
		return;

	if (!vir_addr_consys_dbg_gen_srclken_rc_base_mt6878) {
		pr_notice("vir_addr_consys_dbg_gen_srclken_rc_base_mt6878(%x) ioremap fail\n",
				CONSYS_DBG_GEN_SRCLKEN_RC_BASE_ADDR);
		return;
	}

	if (!vir_addr_consys_dbg_gen_conn_wt_slp_ctl_reg_base_mt6878) {
		pr_notice("vir_addr_consys_dbg_gen_conn_wt_slp_ctl_reg_base_mt6878(%x) ioremap fail\n",
				CONSYS_DBG_GEN_CONN_WT_SLP_CTL_REG_BASE_ADDR);
		return;
	}

	if (!vir_addr_consys_dbg_gen_inst2_conn_wt_slp_ctl_reg_base_mt6878) {
		pr_notice("vir_addr_consys_dbg_gen_inst2_conn_wt_slp_ctl_reg_base_mt6878(%x) ioremap fail\n",
				CONSYS_DBG_GEN_INST2_CONN_WT_SLP_CTL_REG_BASE_ADDR);
		return;
	}

	if (!vir_addr_consys_dbg_gen_conn_rf_spi_mst_reg_base_mt6878) {
		pr_notice("vir_addr_consys_dbg_gen_conn_rf_spi_mst_reg_base_mt6878(%x) ioremap fail\n",
				CONSYS_DBG_GEN_CONN_RF_SPI_MST_REG_BASE_ADDR);
		return;
	}

	if (!vir_addr_consys_dbg_gen_conn_infra_sysram_offset_mt6878) {
		pr_notice("vir_addr_consys_dbg_gen_conn_infra_sysram_offset_mt6878(%x) ioremap fail\n",
				CONSYS_DBG_GEN_CONN_INFRA_SYSRAM_BASE_OFFSET_ADDR);
		return;
	}

	if (!vir_addr_consys_dbg_gen_inst2_conn_rf_spi_mst_reg_base_mt6878) {
		pr_notice("vir_addr_consys_dbg_gen_inst2_conn_rf_spi_mst_reg_base_mt6878(%x) ioremap fail\n",
				CONSYS_DBG_GEN_INST2_CONN_RF_SPI_MST_REG_BASE_ADDR);
		return;
	}

	if (CONN_CFG_BASE == 0) {
		pr_notice("CONN_CFG_BASE is not defined\n");
		return;
	}

	if (CONN_CLKGEN_TOP_BASE == 0) {
		pr_notice("CONN_CLKGEN_TOP_BASE is not defined\n");
		return;
	}

	if (CONN_CFG_ON_BASE == 0) {
		pr_notice("CONN_CFG_ON_BASE is not defined\n");
		return;
	}

	if (CONN_RGU_ON_BASE == 0) {
		pr_notice("CONN_RGU_ON_BASE is not defined\n");
		return;
	}

	/* C0 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C0", 0x18011000 + CONSYS_DBG_GEN_PLL_STATUS_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_CFG_BASE +
			CONSYS_DBG_GEN_PLL_STATUS_OFFSET_ADDR));

	/* C1 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C1", 0x18012000 + CONSYS_DBG_GEN_CKGEN_BUS_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_CLKGEN_TOP_BASE +
			CONSYS_DBG_GEN_CKGEN_BUS_OFFSET_ADDR));

	/* C2 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C2", 0x18001000 + CONSYS_DBG_GEN_CONN_INFRA_CFG_RC_STATUS_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_CFG_ON_BASE +
			CONSYS_DBG_GEN_CONN_INFRA_CFG_RC_STATUS_OFFSET_ADDR));

	/* C3 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C3", 0x1C00D000 + CONSYS_DBG_GEN_M10_REQ_STA_0_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_srclken_rc_base_mt6878 +
			CONSYS_DBG_GEN_M10_REQ_STA_0_OFFSET_ADDR));

	/* C4 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C4", 0x18000000 + CONSYS_DBG_GEN_WFSYS_ON_TOP_PWR_ST_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_RGU_ON_BASE +
			CONSYS_DBG_GEN_WFSYS_ON_TOP_PWR_ST_OFFSET_ADDR));

	/* C5 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C5", 0x18000000 + CONSYS_DBG_GEN_BGFSYS_ON_TOP_PWR_ST_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_RGU_ON_BASE +
			CONSYS_DBG_GEN_BGFSYS_ON_TOP_PWR_ST_OFFSET_ADDR));

	/* C6 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C6", 0x18003000 + CONSYS_DBG_GEN_WB_CK_STA_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_wt_slp_ctl_reg_base_mt6878 +
			CONSYS_DBG_GEN_WB_CK_STA_OFFSET_ADDR));

	/* C7 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C7", 0x18003000 + CONSYS_DBG_GEN_WB_SLP_TOP_CK_0_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_wt_slp_ctl_reg_base_mt6878 +
			CONSYS_DBG_GEN_WB_SLP_TOP_CK_0_OFFSET_ADDR));

	/* C8 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C8", 0x18003000 + CONSYS_DBG_GEN_WB_SLP_TOP_CK_1_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_wt_slp_ctl_reg_base_mt6878 +
			CONSYS_DBG_GEN_WB_SLP_TOP_CK_1_OFFSET_ADDR));

	/* C9 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C9", 0x18003000 + CONSYS_DBG_GEN_WB_SLP_TOP_CK_2_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_wt_slp_ctl_reg_base_mt6878 +
			CONSYS_DBG_GEN_WB_SLP_TOP_CK_2_OFFSET_ADDR));

	/* C10 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C10", 0x18003000 + CONSYS_DBG_GEN_WB_SLP_TOP_CK_3_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_wt_slp_ctl_reg_base_mt6878 +
			CONSYS_DBG_GEN_WB_SLP_TOP_CK_3_OFFSET_ADDR));

	/* C11 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C11", 0x18003000 + CONSYS_DBG_GEN_WB_SLP_TOP_CK_4_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_wt_slp_ctl_reg_base_mt6878 +
			CONSYS_DBG_GEN_WB_SLP_TOP_CK_4_OFFSET_ADDR));

	/* C12 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C12", 0x18003000 + CONSYS_DBG_GEN_WB_SLP_TOP_CK_5_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_wt_slp_ctl_reg_base_mt6878 +
			CONSYS_DBG_GEN_WB_SLP_TOP_CK_5_OFFSET_ADDR));

	/* C13 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C13", 0x18007000 + CONSYS_DBG_GEN_WB_CK_STA_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_inst2_conn_wt_slp_ctl_reg_base_mt6878 +
			CONSYS_DBG_GEN_WB_CK_STA_OFFSET_ADDR));

	/* C14 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C14", 0x18007000 + CONSYS_DBG_GEN_WB_SLP_TOP_CK_0_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_inst2_conn_wt_slp_ctl_reg_base_mt6878 +
			CONSYS_DBG_GEN_WB_SLP_TOP_CK_0_OFFSET_ADDR));

	/* C15 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C15", 0x18007000 + CONSYS_DBG_GEN_WB_SLP_TOP_CK_1_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_inst2_conn_wt_slp_ctl_reg_base_mt6878 +
			CONSYS_DBG_GEN_WB_SLP_TOP_CK_1_OFFSET_ADDR));

	/* C16 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C16", 0x18007000 + CONSYS_DBG_GEN_WB_SLP_TOP_CK_2_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_inst2_conn_wt_slp_ctl_reg_base_mt6878 +
			CONSYS_DBG_GEN_WB_SLP_TOP_CK_2_OFFSET_ADDR));

	/* C17 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C17", 0x18007000 + CONSYS_DBG_GEN_WB_SLP_TOP_CK_3_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_inst2_conn_wt_slp_ctl_reg_base_mt6878 +
			CONSYS_DBG_GEN_WB_SLP_TOP_CK_3_OFFSET_ADDR));

	/* C18 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C18", 0x18007000 + CONSYS_DBG_GEN_WB_SLP_TOP_CK_4_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_inst2_conn_wt_slp_ctl_reg_base_mt6878 +
			CONSYS_DBG_GEN_WB_SLP_TOP_CK_4_OFFSET_ADDR));

	/* C19 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C19", 0x18007000 + CONSYS_DBG_GEN_WB_SLP_TOP_CK_5_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_inst2_conn_wt_slp_ctl_reg_base_mt6878 +
			CONSYS_DBG_GEN_WB_SLP_TOP_CK_5_OFFSET_ADDR));

	/* write 0x18011100[22:21] = 0x0 */
	CONSYS_REG_WRITE_MASK(CONN_CFG_BASE +
		CONSYS_DBG_GEN_EMI_CTL_0_OFFSET_ADDR, 0x0, 0x600000);
	update_debug_write_info_mt6878_debug_gen(pdbg_level_2_info,
		NULL, 0x18011000 + CONSYS_DBG_GEN_EMI_CTL_0_OFFSET_ADDR,
		21, 22, 0x0);

	/* C20 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C20", 0x18011000 + CONSYS_DBG_GEN_EMI_PROBE_1_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_CFG_BASE +
			CONSYS_DBG_GEN_EMI_PROBE_1_OFFSET_ADDR));

	/* write 0x18011100[22:21] = 0x200000 */
	CONSYS_REG_WRITE_MASK(CONN_CFG_BASE +
		CONSYS_DBG_GEN_EMI_CTL_0_OFFSET_ADDR, 0x200000, 0x600000);
	update_debug_write_info_mt6878_debug_gen(pdbg_level_2_info,
		NULL, 0x18011000 + CONSYS_DBG_GEN_EMI_CTL_0_OFFSET_ADDR,
		21, 22, 0x1);

	/* C21 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C21", 0x18011000 + CONSYS_DBG_GEN_EMI_PROBE_1_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_CFG_BASE +
			CONSYS_DBG_GEN_EMI_PROBE_1_OFFSET_ADDR));

	/* write 0x18011100[22:21] = 0x400000 */
	CONSYS_REG_WRITE_MASK(CONN_CFG_BASE +
		CONSYS_DBG_GEN_EMI_CTL_0_OFFSET_ADDR, 0x400000, 0x600000);
	update_debug_write_info_mt6878_debug_gen(pdbg_level_2_info,
		NULL, 0x18011000 + CONSYS_DBG_GEN_EMI_CTL_0_OFFSET_ADDR,
		21, 22, 0x2);

	/* C22 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C22", 0x18011000 + CONSYS_DBG_GEN_EMI_PROBE_1_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_CFG_BASE +
			CONSYS_DBG_GEN_EMI_PROBE_1_OFFSET_ADDR));

	/* write 0x18011100[22:21] = 0x600000 */
	CONSYS_REG_WRITE_MASK(CONN_CFG_BASE +
		CONSYS_DBG_GEN_EMI_CTL_0_OFFSET_ADDR, 0x600000, 0x600000);
	update_debug_write_info_mt6878_debug_gen(pdbg_level_2_info,
		NULL, 0x18011000 + CONSYS_DBG_GEN_EMI_CTL_0_OFFSET_ADDR,
		21, 22, 0x3);

	/* C23 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C23", 0x18011000 + CONSYS_DBG_GEN_EMI_PROBE_1_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_CFG_BASE +
			CONSYS_DBG_GEN_EMI_PROBE_1_OFFSET_ADDR));

	/* C24 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C24", 0x18042000 + CONSYS_DBG_GEN_SPI_MCU_RECORD_CTL_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_rf_spi_mst_reg_base_mt6878 +
			CONSYS_DBG_GEN_SPI_MCU_RECORD_CTL_OFFSET_ADDR));

	/* C25 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C25", 0x18042000 + CONSYS_DBG_GEN_SPI_MCU_RECORD_ADDR_0_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_rf_spi_mst_reg_base_mt6878 +
			CONSYS_DBG_GEN_SPI_MCU_RECORD_ADDR_0_OFFSET_ADDR));

	/* C26 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C26", 0x18042000 + CONSYS_DBG_GEN_SPI_MCU_RECORD_DATA_0_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_rf_spi_mst_reg_base_mt6878 +
			CONSYS_DBG_GEN_SPI_MCU_RECORD_DATA_0_OFFSET_ADDR));

	/* C27 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C27", 0x18042000 + CONSYS_DBG_GEN_SPI_MCU_RECORD_ADDR_1_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_rf_spi_mst_reg_base_mt6878 +
			CONSYS_DBG_GEN_SPI_MCU_RECORD_ADDR_1_OFFSET_ADDR));

	/* C28 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C28", 0x18042000 + CONSYS_DBG_GEN_SPI_MCU_RECORD_DATA_1_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_rf_spi_mst_reg_base_mt6878 +
			CONSYS_DBG_GEN_SPI_MCU_RECORD_DATA_1_OFFSET_ADDR));

	/* C29 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C29", 0x18042000 + CONSYS_DBG_GEN_SPI_MCU_RECORD_ADDR_2_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_rf_spi_mst_reg_base_mt6878 +
			CONSYS_DBG_GEN_SPI_MCU_RECORD_ADDR_2_OFFSET_ADDR));

	/* C30 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C30", 0x18042000 + CONSYS_DBG_GEN_SPI_MCU_RECORD_DATA_2_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_rf_spi_mst_reg_base_mt6878 +
			CONSYS_DBG_GEN_SPI_MCU_RECORD_DATA_2_OFFSET_ADDR));

	/* C31 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C31", 0x18042000 + CONSYS_DBG_GEN_SPI_MCU_RECORD_ADDR_3_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_rf_spi_mst_reg_base_mt6878 +
			CONSYS_DBG_GEN_SPI_MCU_RECORD_ADDR_3_OFFSET_ADDR));

	/* C32 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C32", 0x18042000 + CONSYS_DBG_GEN_SPI_MCU_RECORD_DATA_3_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_rf_spi_mst_reg_base_mt6878 +
			CONSYS_DBG_GEN_SPI_MCU_RECORD_DATA_3_OFFSET_ADDR));

	/* C33 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C33", 0x18042000 + CONSYS_DBG_GEN_SPI_SLP_ADDR_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_rf_spi_mst_reg_base_mt6878 +
			CONSYS_DBG_GEN_SPI_SLP_ADDR_OFFSET_ADDR));

	/* C34 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C34", 0x18042000 + CONSYS_DBG_GEN_SLP_WDAT_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_rf_spi_mst_reg_base_mt6878 +
			CONSYS_DBG_GEN_SLP_WDAT_OFFSET_ADDR));

	/* C35 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C35", 0x18050000 + 0xE74,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_infra_sysram_offset_mt6878 + 0xE74));

	/* C36 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C36", 0x18050000 + 0xE78,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_infra_sysram_offset_mt6878 + 0xE78));

	/* C37 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C37", 0x18050000 + 0xE7C,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_infra_sysram_offset_mt6878 + 0xE7C));

	/* C38 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C38", 0x18050000 + 0xE80,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_infra_sysram_offset_mt6878 + 0xE80));

	/* C39 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C39", 0x18050000 + 0xE84,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_infra_sysram_offset_mt6878 + 0xE84));

	/* C40 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C40", 0x18050000 + 0xE88,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_infra_sysram_offset_mt6878 + 0xE88));

	/* C41 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C41", 0x18050000 + 0xE8C,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_infra_sysram_offset_mt6878 + 0xE8C));

	/* C42 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C42", 0x18050000 + 0xE90,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_infra_sysram_offset_mt6878 + 0xE90));

	/* C43 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C43", 0x18050000 + 0xE94,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_infra_sysram_offset_mt6878 + 0xE94));

	/* C44 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C44", 0x18050000 + 0xE98,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_infra_sysram_offset_mt6878 + 0xE98));

	/* C45 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C45", 0x18050000 + 0xE9C,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_infra_sysram_offset_mt6878 + 0xE9C));

	/* C46 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C46", 0x18050000 + 0xBCC,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_infra_sysram_offset_mt6878 + 0xBCC));

	/* C47 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C47", 0x18050000 + 0xBD0,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_infra_sysram_offset_mt6878 + 0xBD0));

	/* C48 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C48", 0x18050000 + 0xBD4,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_infra_sysram_offset_mt6878 + 0xBD4));

	/* C49 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C49", 0x18050000 + 0xBD8,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_infra_sysram_offset_mt6878 + 0xBD8));

	/* C50 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C50", 0x18050000 + 0xBDC,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_infra_sysram_offset_mt6878 + 0xBDC));

	/* C51 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C51", 0x18050000 + 0xBE0,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_infra_sysram_offset_mt6878 + 0xBE0));

	/* C52 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C52", 0x18050000 + 0xBE4,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_infra_sysram_offset_mt6878 + 0xBE4));

	/* C53 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C53", 0x18050000 + 0xBE8,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_infra_sysram_offset_mt6878 + 0xBE8));

	/* C54 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C54", 0x18050000 + 0xBEC,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_infra_sysram_offset_mt6878 + 0xBEC));

	/* C55 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C55", 0x18050000 + 0xBF0,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_infra_sysram_offset_mt6878 + 0xBF0));

	/* C56 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C56", 0x18050000 + 0xBF4,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_infra_sysram_offset_mt6878 + 0xBF4));

	/* C57 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C57", 0x18050000 + 0xBF8,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_infra_sysram_offset_mt6878 + 0xBF8));

	/* C58 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C58", 0x18050000 + 0xBFC,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_infra_sysram_offset_mt6878 + 0xBFC));

	/* C59 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C59", 0x18050000 + 0xC00,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_infra_sysram_offset_mt6878 + 0xC00));

	/* C60 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C60", 0x18042000 + CONSYS_DBG_GEN_SPI_STA_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_rf_spi_mst_reg_base_mt6878 +
			CONSYS_DBG_GEN_SPI_STA_OFFSET_ADDR));

	/* C61 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C61", 0x18042000 + CONSYS_DBG_GEN_SPI_TOP_ADDR_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_rf_spi_mst_reg_base_mt6878 +
			CONSYS_DBG_GEN_SPI_TOP_ADDR_OFFSET_ADDR));

	/* C62 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C62", 0x18042000 + CONSYS_DBG_GEN_SPI_TOP_WDAT_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_rf_spi_mst_reg_base_mt6878 +
			CONSYS_DBG_GEN_SPI_TOP_WDAT_OFFSET_ADDR));

	/* C63 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C63", 0x18042000 + CONSYS_DBG_GEN_SPI_TOP_RDAT_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_rf_spi_mst_reg_base_mt6878 +
			CONSYS_DBG_GEN_SPI_TOP_RDAT_OFFSET_ADDR));

	/* C64 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C64", 0x18042000 + CONSYS_DBG_GEN_SPI_HSCK_CTL_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_rf_spi_mst_reg_base_mt6878 +
			CONSYS_DBG_GEN_SPI_HSCK_CTL_OFFSET_ADDR));

	/* C65 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C65", 0x18042000 + CONSYS_DBG_GEN_SPI_CRTL_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_rf_spi_mst_reg_base_mt6878 +
			CONSYS_DBG_GEN_SPI_CRTL_OFFSET_ADDR));

	/* C66 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C66", 0x18046000 + CONSYS_DBG_GEN_SPI_STA_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_inst2_conn_rf_spi_mst_reg_base_mt6878 +
			CONSYS_DBG_GEN_SPI_STA_OFFSET_ADDR));

	/* C67 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C67", 0x18046000 + CONSYS_DBG_GEN_SPI_TOP_ADDR_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_inst2_conn_rf_spi_mst_reg_base_mt6878 +
			CONSYS_DBG_GEN_SPI_TOP_ADDR_OFFSET_ADDR));

	/* C68 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C68", 0x18046000 + CONSYS_DBG_GEN_SPI_TOP_WDAT_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_inst2_conn_rf_spi_mst_reg_base_mt6878 +
			CONSYS_DBG_GEN_SPI_TOP_WDAT_OFFSET_ADDR));

	/* C69 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C69", 0x18046000 + CONSYS_DBG_GEN_SPI_TOP_RDAT_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_inst2_conn_rf_spi_mst_reg_base_mt6878 +
			CONSYS_DBG_GEN_SPI_TOP_RDAT_OFFSET_ADDR));

	/* C70 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C70", 0x18046000 + CONSYS_DBG_GEN_SPI_HSCK_CTL_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_inst2_conn_rf_spi_mst_reg_base_mt6878 +
			CONSYS_DBG_GEN_SPI_HSCK_CTL_OFFSET_ADDR));

	/* C71 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C71", 0x18046000 + CONSYS_DBG_GEN_SPI_CRTL_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_inst2_conn_rf_spi_mst_reg_base_mt6878 +
			CONSYS_DBG_GEN_SPI_CRTL_OFFSET_ADDR));

	/* C72 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"C72", 0x18001000 + CONSYS_DBG_GEN_CONN_INFRA_CFG_GPS_FUNCTION_ENABLE,
		CONSYS_REG_READ(CONN_CFG_ON_BASE +
			CONSYS_DBG_GEN_CONN_INFRA_CFG_GPS_FUNCTION_ENABLE));
}

void consys_print_bus_debug_dbg_level_1_mt6878_debug_gen(
		int level,
		struct conn_debug_info_mt6878 *pdbg_level_1_info)
{
	if (pdbg_level_1_info == NULL)
		return;

	memset(pdbg_level_1_info, 0, sizeof(struct conn_debug_info_mt6878));

	if (level < 1)
		return;

	if (!vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878) {
		pr_notice("vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878(%x) ioremap fail\n",
				CONSYS_DBG_GEN_CONN_DBG_CTL_BASE_ADDR);
		return;
	}

	/* write 0x18023408[31:0] = 0x0 */
	CONSYS_REG_WRITE(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
		CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR, 0x0);
	update_debug_write_info_mt6878_debug_gen(pdbg_level_1_info,
		NULL, 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR,
		0, 31, 0x0);

	/* 1 */
	CONSYS_REG_WRITE(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
		CONSYS_DBG_GEN_CONN_INFRA_OFF_DEBUGSYS_CTRL_OFFSET_ADDR, 0x10001);
	update_debug_write_info_mt6878_debug_gen(pdbg_level_1_info,
		"1", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_DEBUGSYS_CTRL_OFFSET_ADDR,
		0, 31, 0x10001);

	/* 1 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		"1", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
			CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR));

	/* 2 */
	CONSYS_REG_WRITE(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
		CONSYS_DBG_GEN_CONN_INFRA_OFF_DEBUGSYS_CTRL_OFFSET_ADDR, 0x20001);
	update_debug_write_info_mt6878_debug_gen(pdbg_level_1_info,
		"2", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_DEBUGSYS_CTRL_OFFSET_ADDR,
		0, 31, 0x20001);

	/* 2 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		"2", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
			CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR));

	/* 3 */
	CONSYS_REG_WRITE(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
		CONSYS_DBG_GEN_CONN_INFRA_OFF_DEBUGSYS_CTRL_OFFSET_ADDR, 0x10002);
	update_debug_write_info_mt6878_debug_gen(pdbg_level_1_info,
		"3", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_DEBUGSYS_CTRL_OFFSET_ADDR,
		0, 31, 0x10002);

	/* 3 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		"3", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
			CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR));

	/* 4 */
	CONSYS_REG_WRITE(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
		CONSYS_DBG_GEN_CONN_INFRA_OFF_DEBUGSYS_CTRL_OFFSET_ADDR, 0x20002);
	update_debug_write_info_mt6878_debug_gen(pdbg_level_1_info,
		"4", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_DEBUGSYS_CTRL_OFFSET_ADDR,
		0, 31, 0x20002);

	/* 4 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		"4", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
			CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR));

	/* 5 */
	CONSYS_REG_WRITE(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
		CONSYS_DBG_GEN_CONN_INFRA_OFF_DEBUGSYS_CTRL_OFFSET_ADDR, 0x30002);
	update_debug_write_info_mt6878_debug_gen(pdbg_level_1_info,
		"5", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_DEBUGSYS_CTRL_OFFSET_ADDR,
		0, 31, 0x30002);

	/* 5 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		"5", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
			CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR));

	/* 6 */
	CONSYS_REG_WRITE(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
		CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR, 0x1);
	update_debug_write_info_mt6878_debug_gen(pdbg_level_1_info,
		"6", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR,
		0, 31, 0x1);

	/* 6 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		"6", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
			CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR));

	/* 7 */
	CONSYS_REG_WRITE(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
		CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR, 0x2);
	update_debug_write_info_mt6878_debug_gen(pdbg_level_1_info,
		"7", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR,
		0, 31, 0x2);

	/* 7 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		"7", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
			CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR));

	/* 8 */
	CONSYS_REG_WRITE(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
		CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR, 0x3);
	update_debug_write_info_mt6878_debug_gen(pdbg_level_1_info,
		"8", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR,
		0, 31, 0x3);

	/* 8 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		"8", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
			CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR));

	/* 9 */
	CONSYS_REG_WRITE(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
		CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR, 0x4);
	update_debug_write_info_mt6878_debug_gen(pdbg_level_1_info,
		"9", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR,
		0, 31, 0x4);

	/* 9 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		"9", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
			CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR));

	/* 10 */
	CONSYS_REG_WRITE(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
		CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR, 0x5);
	update_debug_write_info_mt6878_debug_gen(pdbg_level_1_info,
		"10", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR,
		0, 31, 0x5);

	/* 10 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		"10", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
			CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR));

	/* 11 */
	CONSYS_REG_WRITE(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
		CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR, 0x6);
	update_debug_write_info_mt6878_debug_gen(pdbg_level_1_info,
		"11", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR,
		0, 31, 0x6);

	/* 11 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		"11", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
			CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR));

	/* 12 */
	CONSYS_REG_WRITE(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
		CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR, 0x7);
	update_debug_write_info_mt6878_debug_gen(pdbg_level_1_info,
		"12", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR,
		0, 31, 0x7);

	/* 12 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		"12", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
			CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR));

	/* 13 */
	CONSYS_REG_WRITE(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
		CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR, 0x8);
	update_debug_write_info_mt6878_debug_gen(pdbg_level_1_info,
		"13", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR,
		0, 31, 0x8);

	/* 13 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		"13", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_BUS_TIMEOUT_IRQ_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
			CONSYS_DBG_GEN_CONN_INFRA_BUS_TIMEOUT_IRQ_OFFSET_ADDR));

	/* 14 */
	CONSYS_REG_WRITE(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
		CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR, 0x9);
	update_debug_write_info_mt6878_debug_gen(pdbg_level_1_info,
		"14", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR,
		0, 31, 0x9);

	/* 14 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		"14", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
			CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR));

	/* 15 */
	CONSYS_REG_WRITE(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
		CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR, 0xA);
	update_debug_write_info_mt6878_debug_gen(pdbg_level_1_info,
		"15", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR,
		0, 31, 0xA);

	/* 15 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		"15", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
			CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR));

	/* 16 */
	CONSYS_REG_WRITE(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
		CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR, 0xB);
	update_debug_write_info_mt6878_debug_gen(pdbg_level_1_info,
		"16", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR,
		0, 31, 0xB);

	/* 16 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		"16", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
			CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR));

	/* 17 */
	CONSYS_REG_WRITE(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
		CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR, 0xC);
	update_debug_write_info_mt6878_debug_gen(pdbg_level_1_info,
		"17", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR,
		0, 31, 0xC);

	/* 17 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		"17", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
			CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR));

	/* 18 */
	CONSYS_REG_WRITE(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
		CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR, 0xD);
	update_debug_write_info_mt6878_debug_gen(pdbg_level_1_info,
		"18", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR,
		0, 31, 0xD);

	/* 18 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		"18", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
			CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR));

	/* 19 */
	CONSYS_REG_WRITE(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
		CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR, 0xE);
	update_debug_write_info_mt6878_debug_gen(pdbg_level_1_info,
		"19", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR,
		0, 31, 0xE);

	/* 19 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		"19", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
			CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR));

	/* 20 */
	CONSYS_REG_WRITE(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
		CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR, 0xF);
	update_debug_write_info_mt6878_debug_gen(pdbg_level_1_info,
		"20", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR,
		0, 31, 0xF);

	/* 20 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		"20", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
			CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR));

	/* 21 */
	CONSYS_REG_WRITE(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
		CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR, 0x10);
	update_debug_write_info_mt6878_debug_gen(pdbg_level_1_info,
		"21", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR,
		0, 31, 0x10);

	/* 21 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		"21", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
			CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR));

	/* 22 */
	CONSYS_REG_WRITE(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
		CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR, 0x11);
	update_debug_write_info_mt6878_debug_gen(pdbg_level_1_info,
		"22", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR,
		0, 31, 0x11);

	/* 22 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		"22", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
			CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR));

	/* 23 */
	CONSYS_REG_WRITE(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
		CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR, 0x12);
	update_debug_write_info_mt6878_debug_gen(pdbg_level_1_info,
		"23", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR,
		0, 31, 0x12);

	/* 23 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		"23", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
			CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR));

	/* 24 */
	CONSYS_REG_WRITE(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
		CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR, 0x13);
	update_debug_write_info_mt6878_debug_gen(pdbg_level_1_info,
		"24", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR,
		0, 31, 0x13);

	/* 24 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		"24", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
			CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR));

	/* 25 */
	CONSYS_REG_WRITE(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
		CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR, 0x14);
	update_debug_write_info_mt6878_debug_gen(pdbg_level_1_info,
		"25", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR,
		0, 31, 0x14);

	/* 25 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		"25", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
			CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR));

	/* 26 */
	CONSYS_REG_WRITE(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
		CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR, 0x15);
	update_debug_write_info_mt6878_debug_gen(pdbg_level_1_info,
		"26", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR,
		0, 31, 0x15);

	/* 26 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		"26", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
			CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR));

	/* 27 */
	CONSYS_REG_WRITE(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
		CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR, 0x16);
	update_debug_write_info_mt6878_debug_gen(pdbg_level_1_info,
		"27", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR,
		0, 31, 0x16);

	/* 27 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		"27", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
			CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR));

	/* 28 */
	CONSYS_REG_WRITE(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
		CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR, 0x17);
	update_debug_write_info_mt6878_debug_gen(pdbg_level_1_info,
		"28", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR,
		0, 31, 0x17);

	/* 28 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		"28", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
			CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR));

	/* 29 */
	CONSYS_REG_WRITE(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
		CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR, 0x18);
	update_debug_write_info_mt6878_debug_gen(pdbg_level_1_info,
		"29", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_SEL_OFFSET_ADDR,
		0, 31, 0x18);

	/* 29 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		"29", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
			CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_DBG_OUT_OFFSET_ADDR));

	/* 30 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		"30", 0x18023000 + CONSYS_DBG_GEN_CONN_INFRA_VON_BUS_DEBUG_INFO_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
			CONSYS_DBG_GEN_CONN_INFRA_VON_BUS_DEBUG_INFO_OFFSET_ADDR));

	/* 31 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		"31", 0x18023000 + CONSYS_DBG_GEN_CONN_VON_BUS_APB_TIMEOUT_INFO_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
			CONSYS_DBG_GEN_CONN_VON_BUS_APB_TIMEOUT_INFO_OFFSET_ADDR));

	/* 32 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		"32", 0x18023000 + CONSYS_DBG_GEN_CONN_VON_BUS_APB_TIMEOUT_ADDR_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
			CONSYS_DBG_GEN_CONN_VON_BUS_APB_TIMEOUT_ADDR_OFFSET_ADDR));

	/* 33 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_1_info,
		"33", 0x18023000 + CONSYS_DBG_GEN_CONN_VON_BUS_APB_TIMEOUT_WDATA_OFFSET_ADDR,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_dbg_ctl_base_mt6878 +
			CONSYS_DBG_GEN_CONN_VON_BUS_APB_TIMEOUT_WDATA_OFFSET_ADDR));
}

void consys_print_bus_debug_dbg_level_2_mt6878_debug_gen(
		int level,
		struct conn_debug_info_mt6878 *pdbg_level_2_info)
{
	if (pdbg_level_2_info == NULL)
		return;

	memset(pdbg_level_2_info, 0, sizeof(struct conn_debug_info_mt6878));

	if (level < 2)
		return;

	if (!vir_addr_consys_dbg_gen_conn_infra_axi_layer_bus_bcrm_base_mt6878) {
		pr_notice("vir_addr_consys_dbg_gen_conn_infra_axi_layer_bus_bcrm_base_mt6878(%x) ioremap fail\n",
				CONSYS_DBG_GEN_CONN_INFRA_AXI_LAYER_BUS_BCRM_BASE_ADDR);
		return;
	}

	if (CONN_BUS_CR_ON_BASE == 0) {
		pr_notice("CONN_BUS_CR_ON_BASE is not defined\n");
		return;
	}

	if (CONN_OFF_DEBUG_CTRL_AO_BASE == 0) {
		pr_notice("CONN_OFF_DEBUG_CTRL_AO_BASE is not defined\n");
		return;
	}

	/* 34 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"34", 0x1800e000 + CONSYS_DBG_GEN_CONN_INFRA_ON_BUS_APB_TIMEOUT_INFO_0_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_BUS_CR_ON_BASE +
			CONSYS_DBG_GEN_CONN_INFRA_ON_BUS_APB_TIMEOUT_INFO_0_OFFSET_ADDR));

	/* 35 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"35", 0x1800e000 + CONSYS_DBG_GEN_CONN_INFRA_ON_BUS_APB_TIMEOUT_INFO_1_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_BUS_CR_ON_BASE +
			CONSYS_DBG_GEN_CONN_INFRA_ON_BUS_APB_TIMEOUT_INFO_1_OFFSET_ADDR));

	/* 36 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"36", 0x1800e000 + CONSYS_DBG_GEN_CONN_INFRA_ON_BUS_APB_TIMEOUT_INFO_2_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_BUS_CR_ON_BASE +
			CONSYS_DBG_GEN_CONN_INFRA_ON_BUS_APB_TIMEOUT_INFO_2_OFFSET_ADDR));

	/* 37 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"37", 0x1804c000 + CONSYS_DGEN_INFRA_AXI_LAYER_BUS_BCRM_1804c004_OFFSET,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_infra_axi_layer_bus_bcrm_base_mt6878 +
			CONSYS_DGEN_INFRA_AXI_LAYER_BUS_BCRM_1804c004_OFFSET));

	/* 38 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"38", 0x1804c000 + CONSYS_DGEN_INFRA_AXI_LAYER_BUS_BCRM_1804c00c_OFFSET,
		CONSYS_REG_READ(vir_addr_consys_dbg_gen_conn_infra_axi_layer_bus_bcrm_base_mt6878 +
			CONSYS_DGEN_INFRA_AXI_LAYER_BUS_BCRM_1804c00c_OFFSET));

	/* 39 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"39", 0x1804d000 +
		CONSYS_DBG_GEN_CONN_INFRA_VDNR_GEN_U_DEBUG_CTRL_AO_CONN_INFRA_OFF_CTRL0_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_OFF_DEBUG_CTRL_AO_BASE +
			CONSYS_DBG_GEN_CONN_INFRA_VDNR_GEN_U_DEBUG_CTRL_AO_CONN_INFRA_OFF_CTRL0_OFFSET_ADDR));

	/* 40 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"40", 0x1804d000 +
		CONSYS_DBG_GEN_CONN_INFRA_VDNR_GEN_U_DEBUG_CTRL_AO_CONN_INFRA_OFF_CTRL1_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_OFF_DEBUG_CTRL_AO_BASE +
			CONSYS_DBG_GEN_CONN_INFRA_VDNR_GEN_U_DEBUG_CTRL_AO_CONN_INFRA_OFF_CTRL1_OFFSET_ADDR));

	/* 41 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"41", 0x1804d000 +
		CONSYS_DBG_GEN_CONN_INFRA_VDNR_GEN_U_DEBUG_CTRL_AO_CONN_INFRA_OFF_CTRL2_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_OFF_DEBUG_CTRL_AO_BASE +
			CONSYS_DBG_GEN_CONN_INFRA_VDNR_GEN_U_DEBUG_CTRL_AO_CONN_INFRA_OFF_CTRL2_OFFSET_ADDR));

	/* 42 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"42", 0x1804d000 +
		CONSYS_DBG_GEN_CONN_INFRA_VDNR_GEN_U_DEBUG_CTRL_AO_CONN_INFRA_OFF_RESULT0_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_OFF_DEBUG_CTRL_AO_BASE +
			CONSYS_DBG_GEN_CONN_INFRA_VDNR_GEN_U_DEBUG_CTRL_AO_CONN_INFRA_OFF_RESULT0_OFFSET_ADDR));

	/* 43 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"43", 0x1804d000 +
		CONSYS_DBG_GEN_CONN_INFRA_VDNR_GEN_U_DEBUG_CTRL_AO_CONN_INFRA_OFF_RESULT1_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_OFF_DEBUG_CTRL_AO_BASE +
			CONSYS_DBG_GEN_CONN_INFRA_VDNR_GEN_U_DEBUG_CTRL_AO_CONN_INFRA_OFF_RESULT1_OFFSET_ADDR));

	/* 44 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"44", 0x1804d000 +
		CONSYS_DBG_GEN_CONN_INFRA_VDNR_GEN_U_DEBUG_CTRL_AO_CONN_INFRA_OFF_RESULT2_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_OFF_DEBUG_CTRL_AO_BASE +
			CONSYS_DBG_GEN_CONN_INFRA_VDNR_GEN_U_DEBUG_CTRL_AO_CONN_INFRA_OFF_RESULT2_OFFSET_ADDR));

	/* 45 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"45", 0x1804d000 +
		CONSYS_DBG_GEN_CONN_INFRA_VDNR_GEN_U_DEBUG_CTRL_AO_CONN_INFRA_OFF_RESULT3_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_OFF_DEBUG_CTRL_AO_BASE +
			CONSYS_DBG_GEN_CONN_INFRA_VDNR_GEN_U_DEBUG_CTRL_AO_CONN_INFRA_OFF_RESULT3_OFFSET_ADDR));

	/* 46 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"46", 0x1804d000 +
		CONSYS_DBG_GEN_CONN_INFRA_VDNR_GEN_U_DEBUG_CTRL_AO_CONN_INFRA_OFF_RESULT4_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_OFF_DEBUG_CTRL_AO_BASE +
			CONSYS_DBG_GEN_CONN_INFRA_VDNR_GEN_U_DEBUG_CTRL_AO_CONN_INFRA_OFF_RESULT4_OFFSET_ADDR));

	/* 47 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"47", 0x1804d000 +
		CONSYS_DBG_GEN_CONN_INFRA_VDNR_GEN_U_DEBUG_CTRL_AO_CONN_INFRA_OFF_RESULT5_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_OFF_DEBUG_CTRL_AO_BASE +
			CONSYS_DBG_GEN_CONN_INFRA_VDNR_GEN_U_DEBUG_CTRL_AO_CONN_INFRA_OFF_RESULT5_OFFSET_ADDR));

	/* 48 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"48", 0x1804d000 +
		CONSYS_DBG_GEN_CONN_INFRA_VDNR_GEN_U_DEBUG_CTRL_AO_CONN_INFRA_OFF_RESULT6_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_OFF_DEBUG_CTRL_AO_BASE +
			CONSYS_DBG_GEN_CONN_INFRA_VDNR_GEN_U_DEBUG_CTRL_AO_CONN_INFRA_OFF_RESULT6_OFFSET_ADDR));
}

void consys_print_bus_slpprot_debug_dbg_level_2_mt6878_debug_gen(
		int level,
		struct conn_debug_info_mt6878 *pdbg_level_2_info)
{
	if (pdbg_level_2_info == NULL)
		return;

	memset(pdbg_level_2_info, 0, sizeof(struct conn_debug_info_mt6878));

	if (level < 2)
		return;

	if (CONN_CFG_ON_BASE == 0) {
		pr_notice("CONN_CFG_ON_BASE is not defined\n");
		return;
	}

	/* 1 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"1", 0x18001000 + CONSYS_DBG_GEN_CONN_INFRA_CONN2AP_SLP_STATUS_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_CFG_ON_BASE +
			CONSYS_DBG_GEN_CONN_INFRA_CONN2AP_SLP_STATUS_OFFSET_ADDR));

	/* 2 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"2", 0x18001000 + CONSYS_DBG_GEN_CONN_INFRA_CONN2AP_EMI_SLP_STATUS_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_CFG_ON_BASE +
			CONSYS_DBG_GEN_CONN_INFRA_CONN2AP_EMI_SLP_STATUS_OFFSET_ADDR));

	/* 3 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"3", 0x18001000 + CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_SLP_STATUS_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_CFG_ON_BASE +
			CONSYS_DBG_GEN_CONN_INFRA_OFF_BUS_SLP_STATUS_OFFSET_ADDR));

	/* 4 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"4", 0x18001000 + CONSYS_DBG_GEN_CONN_INFRA_WF_SLP_STATUS_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_CFG_ON_BASE +
			CONSYS_DBG_GEN_CONN_INFRA_WF_SLP_STATUS_OFFSET_ADDR));

	/* 5 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"5", 0x18001000 + CONSYS_DBG_GEN_GALS_CONN2BT_SLP_STATUS_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_CFG_ON_BASE +
			CONSYS_DBG_GEN_GALS_CONN2BT_SLP_STATUS_OFFSET_ADDR));

	/* 6 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"6", 0x18001000 + CONSYS_DBG_GEN_GALS_BT2CONN_SLP_STATUS_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_CFG_ON_BASE +
			CONSYS_DBG_GEN_GALS_BT2CONN_SLP_STATUS_OFFSET_ADDR));

	/* 7 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"7", 0x18001000 + CONSYS_DBG_GEN_GALS_CONN2GPS_SLP_STATUS_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_CFG_ON_BASE +
			CONSYS_DBG_GEN_GALS_CONN2GPS_SLP_STATUS_OFFSET_ADDR));

	/* 8 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_2_info,
		"8", 0x18001000 + CONSYS_DBG_GEN_GALS_GPS2CONN_SLP_STATUS_OFFSET_ADDR,
		CONSYS_REG_READ(CONN_CFG_ON_BASE +
			CONSYS_DBG_GEN_GALS_GPS2CONN_SLP_STATUS_OFFSET_ADDR));
}

void consys_print_bus_slpprot_debug_dbg_level_0_mt6878_debug_gen(
		int level,
		struct conn_debug_info_mt6878 *pdbg_level_0_info)
{
	if (pdbg_level_0_info == NULL)
		return;

	memset(pdbg_level_0_info, 0, sizeof(struct conn_debug_info_mt6878));

	if (level < 0)
		return;

	if (INFRACFG_AO_REG_BASE == 0) {
		pr_notice("INFRACFG_AO_REG_BASE is not defined\n");
		return;
	}

	/* 9 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"9", 0x10001000 + CONSYS_DBG_GEN_MCU_CONNSYS_PROTECT_RDY_STA_0_OFFSET_ADDR,
		CONSYS_REG_READ(INFRACFG_AO_REG_BASE +
			CONSYS_DBG_GEN_MCU_CONNSYS_PROTECT_RDY_STA_0_OFFSET_ADDR));

	/* 10 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"10", 0x10001000 + CONSYS_DBG_GEN_INFRASYS_PROTECT_RDY_STA_1_OFFSET_ADDR,
		CONSYS_REG_READ(INFRACFG_AO_REG_BASE +
			CONSYS_DBG_GEN_INFRASYS_PROTECT_RDY_STA_1_OFFSET_ADDR));

	/* 11 */
	update_debug_read_info_mt6878_debug_gen(pdbg_level_0_info,
		"11", 0x10001000 + CONSYS_DBG_GEN_INFRASYS_PROTECT_RDY_STA_0_OFFSET_ADDR,
		CONSYS_REG_READ(INFRACFG_AO_REG_BASE +
			CONSYS_DBG_GEN_INFRASYS_PROTECT_RDY_STA_0_OFFSET_ADDR));
}
