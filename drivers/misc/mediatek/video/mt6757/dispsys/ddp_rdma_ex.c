/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#define LOG_TAG "RDMA"
#include "ddp_log.h"
#ifdef CONFIG_MTK_CLKMGR
#include <mach/mt_clkmgr.h>
#else
#include "ddp_clkmgr.h"
#endif
#include <linux/delay.h>
#include "ddp_info.h"
#include "ddp_reg.h"
#include "ddp_matrix_para.h"
#include "ddp_rdma.h"
#include "ddp_rdma_ex.h"
#include "ddp_dump.h"
#include "lcm_drv.h"
#include "primary_display.h"
#include "m4u_port.h"
/* #include "mt_spm_reg.h" */ /* FIXME: tmp comment */
/* #include "pcm_def.h" */ /* FIXME: tmp comment */
#include "mtk_spm.h"
#include "mtk_smi.h"
/* #include "mmdvfs_mgr.h" */
#include "disp_lowpower.h"
#include "ddp_mmp.h"

#define MMSYS_CLK_LOW (0)
#define MMSYS_CLK_HIGH (1)

static unsigned int rdma_fps[RDMA_INSTANCES] = { 60, 60 };
static struct golden_setting_context *rdma_golden_setting;

static inline unsigned long rdma_to_cmdq_engine(enum DISP_MODULE_ENUM module)
{
	switch (module) {
	case DISP_MODULE_RDMA0:
		return CMDQ_ENG_DISP_RDMA0;
	case DISP_MODULE_RDMA1:
		return CMDQ_ENG_DISP_RDMA1;
	default:
		DDPERR("invalid rdma module=%d,rdma to cmdq engine fail\n", module);
		ASSERT(0);
		return DISP_MODULE_UNKNOWN;
	}
	return 0;
}

static inline unsigned long rdma_to_cmdq_event_nonsec_end(enum DISP_MODULE_ENUM module)
{
	switch (module) {
	case DISP_MODULE_RDMA0:
		return CMDQ_SYNC_DISP_RDMA0_2NONSEC_END;
	case DISP_MODULE_RDMA1:
		return CMDQ_SYNC_DISP_RDMA1_2NONSEC_END;
	default:
		DDPERR("invalid rdma module=%d,rmda to cmdq event fail\n", module);
		ASSERT(0);
		return DISP_MODULE_UNKNOWN;
	}

	return 0;
}

int rdma_enable_irq(enum DISP_MODULE_ENUM module, void *handle, enum DDP_IRQ_LEVEL irq_level)
{
	unsigned int idx = rdma_index(module);

	ASSERT(idx <= RDMA_INSTANCES);

	switch (irq_level) {
	case DDP_IRQ_LEVEL_ALL:
		DISP_REG_SET(handle, idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_INT_ENABLE, 0x1E);
		break;
	case DDP_IRQ_LEVEL_ERROR:
		DISP_REG_SET(handle, idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_INT_ENABLE, 0x18);
		break;
	case DDP_IRQ_LEVEL_NONE:
		DISP_REG_SET(handle, idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_INT_ENABLE, 0x0);
		break;
	default:
		break;
	}

	return 0;
}

int rdma_start(enum DISP_MODULE_ENUM module, void *handle)
{
	unsigned int idx = rdma_index(module);
	unsigned int regval;

	ASSERT(idx <= RDMA_INSTANCES);

	regval = REG_FLD_VAL(INT_STATUS_FLD_REG_UPDATE_INT_FLAG, 0) |
	    REG_FLD_VAL(INT_STATUS_FLD_FRAME_START_INT_FLAG, 1) |
	    REG_FLD_VAL(INT_STATUS_FLD_FRAME_END_INT_FLAG, 1) |
	    REG_FLD_VAL(INT_STATUS_FLD_EOF_ABNORMAL_INT_FLAG, 1) |
	    REG_FLD_VAL(INT_STATUS_FLD_FIFO_UNDERFLOW_INT_FLAG, 1) |
	    REG_FLD_VAL(INT_STATUS_FLD_TARGET_LINE_INT_FLAG, 0) |
	    REG_FLD_VAL(INT_STATUS_FLD_FIFO_EMPTY_INT_FLAG, 0);

	if (disp_helper_get_option(DISP_OPT_SHADOW_REGISTER)) {
		if (disp_helper_get_option(DISP_OPT_SHADOW_MODE) == 0) {
			/* full shadow mode: read shadow */
			DISP_REG_SET(handle, idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_SHADOW_UPDATE, 0x0<<2);
		} else if (disp_helper_get_option(DISP_OPT_SHADOW_MODE) == 1) {
			/* force commit: force_commit, read shadow */
			DISP_REG_SET(handle, idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_SHADOW_UPDATE,
			    (0x1<<0)|(0x0<<2));
		} else if (disp_helper_get_option(DISP_OPT_SHADOW_MODE) == 2) {
			/* bypass shadow: bypass_shadow, read shadow */
			DISP_REG_SET(handle, idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_SHADOW_UPDATE,
			    (0x1<<1)|(0x0<<2));
		}
	}
	DISP_REG_SET(handle, idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_INT_ENABLE, regval);
	DISP_REG_SET_FIELD(handle, GLOBAL_CON_FLD_ENGINE_EN,
			   idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_GLOBAL_CON, 1);

	return 0;
}

int rdma_stop(enum DISP_MODULE_ENUM module, void *handle)
{
	unsigned int idx = rdma_index(module);

	ASSERT(idx <= RDMA_INSTANCES);

	DISP_REG_SET_FIELD(handle, GLOBAL_CON_FLD_ENGINE_EN,
			   idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_GLOBAL_CON, 0);
	DISP_REG_SET(handle, idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_INT_ENABLE, 0);
	DISP_REG_SET(handle, idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_INT_STATUS, 0);
	return 0;
}

int rdma_reset_by_cmdq(enum DISP_MODULE_ENUM module, void *handle)
{
	int ret = 0;
	unsigned int idx = rdma_index(module);

	ASSERT(idx <= RDMA_INSTANCES);

	DISP_REG_SET_FIELD(handle, GLOBAL_CON_FLD_SOFT_RESET,
			   idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_GLOBAL_CON, 1);

	DISP_REG_SET_FIELD(handle, GLOBAL_CON_FLD_SOFT_RESET,
			   idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_GLOBAL_CON, 0);

	DISP_REG_CMDQ_POLLING(handle, idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_GLOBAL_CON,
				0x100, 0x700);

	return ret;
}

int rdma_reset(enum DISP_MODULE_ENUM module, void *handle)
{
	unsigned int delay_cnt = 0;
	int ret = 0;
	unsigned int idx = rdma_index(module);

	ASSERT(idx <= RDMA_INSTANCES);

	DISP_REG_SET_FIELD(handle, GLOBAL_CON_FLD_SOFT_RESET,
			   idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_GLOBAL_CON, 1);
	while ((DISP_REG_GET(idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_GLOBAL_CON) & 0x700) == 0x100) {
		delay_cnt++;
		udelay(10);
		if (delay_cnt > 10000) {
			ret = -1;
			DDPERR("rdma%d_reset timeout, stage 1! DISP_REG_RDMA_GLOBAL_CON=0x%x\n",
			       idx,
			       DISP_REG_GET(idx * DISP_RDMA_INDEX_OFFSET +
					    DISP_REG_RDMA_GLOBAL_CON));
			break;
		}
	}
	DISP_REG_SET_FIELD(handle, GLOBAL_CON_FLD_SOFT_RESET,
			   idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_GLOBAL_CON, 0);
	delay_cnt = 0;
	while ((DISP_REG_GET(idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_GLOBAL_CON) & 0x700) !=
	       0x100) {
		delay_cnt++;
		udelay(10);
		if (delay_cnt > 10000) {
			ret = -1;
			DDPERR("rdma%d_reset timeout, stage 2! DISP_REG_RDMA_GLOBAL_CON=0x%x\n",
			       idx,
			       DISP_REG_GET(idx * DISP_RDMA_INDEX_OFFSET +
					    DISP_REG_RDMA_GLOBAL_CON));
			break;
		}
	}
	return ret;
}

#if 1
/* set ultra registers */
void rdma_set_ultra_l(unsigned int idx, unsigned int bpp, void *handle, struct golden_setting_context *p_golden_setting)
{

	/* rdma golden setting variables */
	unsigned int mmsysclk = 315;
	unsigned int is_wrot_sram = 0;
	unsigned int fifo_mode = 1;

	unsigned int ultra_low_us = 4;
	unsigned int ultra_high_us = 6;
	unsigned int preultra_low_us = ultra_high_us;
	unsigned int preultra_high_us = 7;
	unsigned int sodi_low_us = 10;

	unsigned long long fill_rate = 0;
	unsigned long long consume_rate = 0;

	unsigned int fifo_valid_size = 640;

	/* working variables */
	unsigned int ultra_low;
	unsigned int preultra_low;
	unsigned int preultra_high;
	unsigned int ultra_high;

	unsigned int issue_req_threshold;
	unsigned int output_valid_fifo_threshold;

	unsigned int sodi_threshold_high;
	unsigned int sodi_threshold_low;
	unsigned int dvfs_threshold_high;
	unsigned int dvfs_threshold_low;

	unsigned int frame_rate;
	unsigned int Bytes_per_sec;
	long long temp;
	long long temp_for_div;


	if (!p_golden_setting) {
		DDPERR("golden setting is null, %s,%d\n", __FILE__, __LINE__);
		ASSERT(0);
		return;
	}
	rdma_golden_setting = p_golden_setting;

	frame_rate = rdma_golden_setting->fps;
	if (idx == 1) {
		/* hardcode bpp & frame_rate for rdma1 */
		bpp = 24;
		frame_rate = 60;

		if ((rdma_golden_setting->ext_dst_width == 3840) &&
				(rdma_golden_setting->ext_dst_height == 2160))
			frame_rate = 30;
	}

	/* get fifo parameters */
	switch (rdma_golden_setting->mmsys_clk) {
	case MMSYS_CLK_LOW:
		mmsysclk = 315;
		break;
	case MMSYS_CLK_HIGH:
		mmsysclk = 400;
		break;
	default:
		mmsysclk = 315; /* worse case */
		break;
	}

	Bytes_per_sec = bpp / 8;
	if (!Bytes_per_sec) {
		DDPERR("bpp is invalid, bpp=%d\n", bpp);
		return;
	}

	is_wrot_sram = rdma_golden_setting->is_wrot_sram;
	fifo_mode = rdma_golden_setting->fifo_mode;


	if (rdma_golden_setting->is_dc)
		fill_rate = 960*mmsysclk; /* FIFO depth / us  */
	else
		fill_rate = 960*mmsysclk*3/16; /* FIFO depth / us  */

	if (idx == 0) {
		consume_rate = rdma_golden_setting->dst_width * rdma_golden_setting->dst_height
				*frame_rate * Bytes_per_sec;
		do_div(consume_rate, 1000);

	} else {
		consume_rate = rdma_golden_setting->ext_dst_width
				* rdma_golden_setting->ext_dst_height*frame_rate*Bytes_per_sec;
		do_div(consume_rate, 1000);
	}
	consume_rate *= 1250;
	do_div(consume_rate, 16*1000);

	preultra_low = preultra_low_us * consume_rate;
	preultra_low = DIV_ROUND_UP(preultra_low, 1000);

	preultra_high = preultra_high_us * consume_rate;
	preultra_high = DIV_ROUND_UP(preultra_high, 1000);


	ultra_low = ultra_low_us * consume_rate;
	ultra_low = DIV_ROUND_UP(ultra_low, 1000);


	ultra_high = preultra_low;
	if (idx == 0) {
		/* only rdma0 can share sram */
		if (is_wrot_sram)
			fifo_valid_size = 2048;
		else
			fifo_valid_size = 640;
	} else {
		fifo_valid_size = 640;
	}

	issue_req_threshold = min(fifo_valid_size - preultra_low, (unsigned int)255);


	/* output valid should < total rdma data size, or hang will happen */
	temp = rdma_golden_setting->rdma_width * rdma_golden_setting->rdma_height * Bytes_per_sec;
	do_div(temp, 16);
	temp -= 1;
	output_valid_fifo_threshold = preultra_low < temp ? preultra_low : temp;

	/* SODI threshold */
	sodi_threshold_low = sodi_low_us * consume_rate;
	sodi_threshold_low = DIV_ROUND_UP(sodi_threshold_low, 1000);

	temp_for_div = 1200 * (fill_rate - consume_rate);
	do_div(temp_for_div, 1000000);
	temp = fifo_valid_size - temp_for_div;
	if (temp < 0)
		sodi_threshold_high = preultra_high;
	else
		sodi_threshold_high = preultra_high > temp ? preultra_high : temp;

	dvfs_threshold_low = preultra_low;
	dvfs_threshold_high = preultra_low+1;

	DISP_REG_SET(handle, idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_MEM_GMC_SETTING_0,
		preultra_low | (preultra_high << 16));

	DISP_REG_SET(handle, idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_MEM_GMC_SETTING_1,
		ultra_low | (ultra_high << 16));

	DISP_REG_SET(handle, idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_MEM_GMC_SETTING_2,
		issue_req_threshold);

	DISP_REG_SET(handle, idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_FIFO_CON,
		REG_FLD_VAL(FIFO_CON_FLD_OUTPUT_VALID_FIFO_THRESHOLD, output_valid_fifo_threshold) |
		REG_FLD_VAL(FIFO_CON_FLD_FIFO_PSEUDO_SIZE, fifo_valid_size) |
		REG_FLD_VAL(FIFO_CON_FLD_FIFO_UNDERFLOW_EN, 1));

	DISP_REG_SET(handle, idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_THRESHOLD_FOR_SODI,
		sodi_threshold_low | (sodi_threshold_high << 16));

	DISP_REG_SET(handle, idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_THRESHOLD_FOR_DVFS,
		dvfs_threshold_low | (dvfs_threshold_high << 16));

	/* only config RDMA0 SRAM_SEL */
	if (idx == 0)
		DISP_REG_SET(handle, DISP_REG_RDMA_SRAM_SEL, is_wrot_sram);

/*if (idx == 0)*/
		/*rdma_dump_golden_setting_context(DISP_MODULE_RDMA0);*/
	/*else*/
		/*rdma_dump_golden_setting_context(DISP_MODULE_RDMA1);*/

	if (rdma_golden_setting->dst_width == 0 || rdma_golden_setting->dst_height == 0
		|| bpp == 0 || frame_rate == 0) {
		DDPDUMP("==RDMA Golden Setting Value=============\n");

		DDPDUMP("width		= %d\n", rdma_golden_setting->dst_width);
		DDPDUMP("height		= %d\n", rdma_golden_setting->dst_height);
		DDPDUMP("bpp		= %d\n", bpp);
		DDPDUMP("frame_rate	= %d\n", frame_rate);

		DDPDUMP("fill_rate	= %lld\n", fill_rate);
		DDPDUMP("consume_rate	= %lld\n", consume_rate);
		DDPDUMP("ultra_low_us	= %d\n", ultra_low_us);
		DDPDUMP("ultra_high_us	= %d\n", ultra_high_us);
		DDPDUMP("preultra_high_us= %d\n", preultra_high_us);

		DDPDUMP("preultra_low	= %d\n", preultra_low);
		DDPDUMP("preultra_high	= %d\n", preultra_high);
		DDPDUMP("ultra_low	= %d\n", ultra_low);
		DDPDUMP("issue_req_threshold		= %d\n", issue_req_threshold);
		DDPDUMP("output_valid_fifo_threshold	= %d\n", output_valid_fifo_threshold);
		DDPDUMP("sodi_threshold_low	= %d\n", sodi_threshold_low);
		DDPDUMP("sodi_threshold_high	= %d\n", sodi_threshold_high);
		DDPDUMP("dvfs_threshold_low	= %d\n", dvfs_threshold_low);
		DDPDUMP("dvfs_threshold_high	= %d\n", dvfs_threshold_high);
	}

}

#else

/* set ultra registers */
void rdma_set_ultra_l(unsigned int idx, unsigned int bpp, void *handle, golden_setting_context *p_golden_setting)
{
	unsigned int regval;
	int is_large_resolution = 0;
	unsigned int dst_w, dst_h;
	unsigned long idx_offset = idx * DISP_RDMA_INDEX_OFFSET;
	int is_wrot_sram = 0;

	ASSERT(p_golden_setting);

	if (idx == 0) {
		/* use for primary display */
		dst_w = p_golden_setting->dst_width;
		dst_h = p_golden_setting->dst_height;
	} else {
		/* use extd display setting */
		dst_w = p_golden_setting->ext_dst_width;
		dst_h = p_golden_setting->ext_dst_height;
	}

	if (dst_w > 1260 && dst_h > 2240) {
		/* WQHD */
		is_large_resolution = 1;
	} else {
		/* FHD */
		is_large_resolution = 0;
	}

	if (idx == 0 && p_golden_setting->is_wrot_sram) {
		/* only rdma0 uses share sram */
		is_wrot_sram = 1;
	} else {
		is_wrot_sram = 0;
	}

	/* DISP_REG_RDMA_MEM_GMC_SETTING_0 */
	regval = 0;
	if (is_large_resolution) {
		regval |= REG_FLD_VAL(MEM_GMC_SETTING_0_FLD_PRE_ULTRA_THRESHOLD_LOW, 312);
		regval |= REG_FLD_VAL(MEM_GMC_SETTING_0_FLD_PRE_ULTRA_THRESHOLD_HIGH, 363);
	} else {
		regval |= REG_FLD_VAL(MEM_GMC_SETTING_0_FLD_PRE_ULTRA_THRESHOLD_LOW, 175);
		regval |= REG_FLD_VAL(MEM_GMC_SETTING_0_FLD_PRE_ULTRA_THRESHOLD_HIGH, 205);
	}
	DISP_REG_SET(handle, idx_offset + DISP_REG_RDMA_MEM_GMC_SETTING_0, regval);

	/* DISP_REG_RDMA_MEM_GMC_SETTING_1 */
	regval = 0;
	if (is_large_resolution) {
		regval |= REG_FLD_VAL(MEM_GMC_SETTING_1_FLD_ULTRA_THRESHOLD_LOW, 208);
		regval |= REG_FLD_VAL(MEM_GMC_SETTING_1_FLD_ULTRA_THRESHOLD_HIGH, 312);
	} else {
		regval |= REG_FLD_VAL(MEM_GMC_SETTING_1_FLD_ULTRA_THRESHOLD_LOW, 117);
		regval |= REG_FLD_VAL(MEM_GMC_SETTING_1_FLD_ULTRA_THRESHOLD_HIGH, 175);
	}
	DISP_REG_SET(handle, idx_offset + DISP_REG_RDMA_MEM_GMC_SETTING_1, regval);

	/* DISP_REG_RDMA_MEM_GMC_SETTING_2 */
	regval = REG_FLD_VAL(MEM_GMC_SETTING_2_FLD_ISSUE_REQ_THRESHOLD, 255);
	DISP_REG_SET(handle, idx_offset + DISP_REG_RDMA_MEM_GMC_SETTING_2, regval);

	/* DISP_REG_RDMA_FIFO_CON */
	regval = 0;
	if (is_large_resolution)
		regval |= REG_FLD_VAL(FIFO_CON_FLD_OUTPUT_VALID_FIFO_THRESHOLD, 312);
	else
		regval |= REG_FLD_VAL(FIFO_CON_FLD_OUTPUT_VALID_FIFO_THRESHOLD, 175);

	if (is_wrot_sram)
		regval |= REG_FLD_VAL(FIFO_CON_FLD_FIFO_PSEUDO_SIZE, 2048);
	else
		regval |= REG_FLD_VAL(FIFO_CON_FLD_FIFO_PSEUDO_SIZE, 640);
	regval |= REG_FLD_VAL(FIFO_CON_FLD_FIFO_UNDERFLOW_EN, 1);
	DISP_REG_SET(handle, idx_offset + DISP_REG_RDMA_FIFO_CON, regval);

	/* DISP_REG_RDMA_THRESHOLD_FOR_SODI */
	regval = 0;
	if (is_large_resolution) {
		regval |= REG_FLD_VAL(RDMA_THRESHOLD_FOR_SODI_FLD_LOW, 229);
		if (is_wrot_sram)
			regval |= REG_FLD_VAL(RDMA_THRESHOLD_FOR_SODI_FLD_HIGH, 1742);
		else if (p_golden_setting->is_dc)
			regval |= REG_FLD_VAL(RDMA_THRESHOLD_FOR_SODI_FLD_HIGH, 363);
		else
			regval |= REG_FLD_VAL(RDMA_THRESHOLD_FOR_SODI_FLD_HIGH, 606);
	} else {
		regval |= REG_FLD_VAL(RDMA_THRESHOLD_FOR_SODI_FLD_LOW, 129);
		if (is_wrot_sram)
			regval |= REG_FLD_VAL(RDMA_THRESHOLD_FOR_SODI_FLD_HIGH, 1715);
		else if (p_golden_setting->is_dc)
			regval |= REG_FLD_VAL(RDMA_THRESHOLD_FOR_SODI_FLD_HIGH, 205);
		else
			regval |= REG_FLD_VAL(RDMA_THRESHOLD_FOR_SODI_FLD_HIGH, 578);
	}
	DISP_REG_SET(handle, idx_offset + DISP_REG_RDMA_THRESHOLD_FOR_SODI, regval);

	/* DISP_REG_RDMA_THRESHOLD_FOR_DVFS */
	regval = 0;
	if (is_large_resolution) {
		regval |= REG_FLD_VAL(RDMA_THRESHOLD_FOR_SODI_FLD_LOW, 312);
		regval |= REG_FLD_VAL(RDMA_THRESHOLD_FOR_SODI_FLD_HIGH, 313);
	} else {
		regval |= REG_FLD_VAL(RDMA_THRESHOLD_FOR_SODI_FLD_LOW, 175);
		regval |= REG_FLD_VAL(RDMA_THRESHOLD_FOR_SODI_FLD_HIGH, 176);
	}
	DISP_REG_SET(handle, idx_offset + DISP_REG_RDMA_THRESHOLD_FOR_DVFS, regval);

	/* DISP_REG_RDMA_SRAM_SEL */
	DISP_REG_SET(handle, DISP_REG_RDMA_SRAM_SEL, is_wrot_sram);

}
#endif
static int rdma_config(enum DISP_MODULE_ENUM module,
		       enum RDMA_MODE mode,
		       unsigned long address,
		       enum UNIFIED_COLOR_FMT inFormat,
		       unsigned pitch,
		       unsigned width,
		       unsigned height,
		       unsigned ufoe_enable,
		       enum DISP_BUFFER_TYPE sec,
		       unsigned int yuv_range, struct rdma_bg_ctrl_t *bg_ctrl, void *handle,
		       struct golden_setting_context *p_golden_setting, unsigned int bpp)
{

	unsigned int output_is_yuv = 0;
	unsigned int input_is_yuv = !UFMT_GET_RGB(inFormat);
	unsigned int input_swap = UFMT_GET_BYTESWAP(inFormat);
	unsigned int input_format_reg = UFMT_GET_FORMAT(inFormat);
	unsigned int idx = rdma_index(module);
	unsigned int color_matrix;
	unsigned int regval;

	DDPDBG("RDMAConfig idx %d, mode %d, address 0x%lx, inputformat %s, pitch %u, width %u, height %u,sec%d\n",
	       idx, mode, address, unified_color_fmt_name(inFormat), pitch, width, height, sec);
	ASSERT(idx <= RDMA_INSTANCES);
	if ((width > RDMA_MAX_WIDTH) || (height > RDMA_MAX_HEIGHT))
		DDPERR("RDMA input overflow, w=%d, h=%d, max_w=%d, max_h=%d\n", width, height,
		       RDMA_MAX_WIDTH, RDMA_MAX_HEIGHT);

	if (input_is_yuv == 1 && output_is_yuv == 0) {
		switch (yuv_range) {
		case 0:
			color_matrix = 4;
			break;	/* BT601_full */
		case 1:
			color_matrix = 6;
			break;	/* BT601 */
		case 2:
			color_matrix = 7;
			break;	/* BT709 */
		default:
			DDPERR("%s,un-recognized yuv_range=%d!\n", __func__, yuv_range);
			color_matrix = 4;
		}

		DISP_REG_SET_FIELD(handle, SIZE_CON_0_FLD_MATRIX_ENABLE,
				   idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_SIZE_CON_0, 1);
		DISP_REG_SET_FIELD(handle, SIZE_CON_0_FLD_MATRIX_INT_MTX_SEL,
				   idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_SIZE_CON_0,
				   color_matrix);
	} else if (input_is_yuv == 0 && output_is_yuv == 1) {
		color_matrix = 0x2;	/* 0x0010, RGB_TO_BT601 */
		DISP_REG_SET_FIELD(handle, SIZE_CON_0_FLD_MATRIX_ENABLE,
				   idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_SIZE_CON_0, 1);
		DISP_REG_SET_FIELD(handle, SIZE_CON_0_FLD_MATRIX_INT_MTX_SEL,
				   idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_SIZE_CON_0,
				   color_matrix);
	} else {
		DISP_REG_SET_FIELD(handle, SIZE_CON_0_FLD_MATRIX_ENABLE,
				   idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_SIZE_CON_0, 0);
		DISP_REG_SET_FIELD(handle, SIZE_CON_0_FLD_MATRIX_INT_MTX_SEL,
				   idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_SIZE_CON_0, 0);
	}

	DISP_REG_SET_FIELD(handle, GLOBAL_CON_FLD_MODE_SEL,
			   idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_GLOBAL_CON, mode);
	/* FORMAT & SWAP only works when RDMA memory mode, set both to 0 when RDMA direct link mode. */
	DISP_REG_SET_FIELD(handle, MEM_CON_FLD_MEM_MODE_INPUT_FORMAT,
			   idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_MEM_CON,
			   ((mode == RDMA_MODE_DIRECT_LINK) ? 0 : input_format_reg & 0xf));
	DISP_REG_SET_FIELD(handle, MEM_CON_FLD_MEM_MODE_INPUT_SWAP,
			   idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_MEM_CON,
			   ((mode == RDMA_MODE_DIRECT_LINK) ? 0 : input_swap));

	if (sec != DISP_SECURE_BUFFER) {
		DISP_REG_SET(handle, idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_MEM_START_ADDR,
			     address);
	} else {
		int m4u_port;
		unsigned int size = pitch * height;

		m4u_port = idx == 0 ? M4U_PORT_DISP_RDMA0 : M4U_PORT_DISP_RDMA1;
		/* for sec layer, addr variable stores sec handle */
		/* we need to pass this handle and offset to cmdq driver */
		/* cmdq sec driver will help to convert handle to correct address */
		cmdqRecWriteSecure(handle,
				   disp_addr_convert(idx * DISP_RDMA_INDEX_OFFSET +
						     DISP_REG_RDMA_MEM_START_ADDR),
				   CMDQ_SAM_H_2_MVA, address, 0, size, m4u_port);
	}

	DISP_REG_SET(handle, idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_MEM_SRC_PITCH, pitch);
	/* DISP_REG_SET(handle,idx*DISP_RDMA_INDEX_OFFSET+ DISP_REG_RDMA_INT_ENABLE, 0x3F); */
	DISP_REG_SET_FIELD(handle, SIZE_CON_0_FLD_OUTPUT_FRAME_WIDTH,
			   idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_SIZE_CON_0, width);
	DISP_REG_SET_FIELD(handle, SIZE_CON_1_FLD_OUTPUT_FRAME_HEIGHT,
			   idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_SIZE_CON_1, height);

	/* rdma bg control */
	regval = REG_FLD_VAL(RDMA_BG_CON_0_LEFT, bg_ctrl->left);
	regval |= REG_FLD_VAL(RDMA_BG_CON_0_RIGHT, bg_ctrl->right);
	DISP_REG_SET(handle, idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_BG_CON_0, regval);

	regval = REG_FLD_VAL(RDMA_BG_CON_1_TOP, bg_ctrl->top);
	regval |= REG_FLD_VAL(RDMA_BG_CON_1_BOTTOM, bg_ctrl->bottom);
	DISP_REG_SET(handle, idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_BG_CON_1, regval);

	set_rdma_width_height(width, height);
	rdma_set_ultra_l(idx, bpp, handle, p_golden_setting);

	return 0;
}

int rdma_clock_on(enum DISP_MODULE_ENUM module, void *handle)
{
	unsigned int idx = rdma_index(module);
#ifdef ENABLE_CLK_MGR
#ifdef CONFIG_MTK_CLKMGR
	if (idx == 0)
		enable_clock(MT_CG_DISP0_DISP_RDMA0, "RDMA0");
	else
		enable_clock(MT_CG_DISP0_DISP_RDMA1, "RDMA1");
#else
	if (idx == 0)
		ddp_clk_enable(DISP0_DISP_RDMA0);
	else
		ddp_clk_enable(DISP0_DISP_RDMA1);
#endif
#endif
	DDPMSG("rdma_%d_clock_on CG 0x%x\n", idx, DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_CON0));
	return 0;
}

int rdma_clock_off(enum DISP_MODULE_ENUM module, void *handle)
{
	unsigned int idx = rdma_index(module);

#ifdef ENABLE_CLK_MGR
#ifdef CONFIG_MTK_CLKMGR
	if (idx == 0)
		disable_clock(MT_CG_DISP0_DISP_RDMA0, "RDMA0");
	else
		disable_clock(MT_CG_DISP0_DISP_RDMA1, "RDMA1");
#else
	if (idx == 0)
		ddp_clk_disable(DISP0_DISP_RDMA0);
	else
		ddp_clk_disable(DISP0_DISP_RDMA1);

#endif
#endif
	DDPMSG("rdma_%d_clock_off CG 0x%x\n", idx, DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_CON0));
	return 0;
}

void rdma_dump_golden_setting_context(enum DISP_MODULE_ENUM module)
{
	if (rdma_golden_setting) {
		DDPDUMP("-- RDMA Golden Setting Context --\n");
		DDPDUMP("fifo_mode=%d\n", rdma_golden_setting->fifo_mode);
		DDPDUMP("hrt_num=%d\n", rdma_golden_setting->hrt_num);
		DDPDUMP("is_display_idle=%d\n", rdma_golden_setting->is_display_idle);
		DDPDUMP("is_wrot_sram=%d\n", rdma_golden_setting->is_wrot_sram);
		DDPDUMP("is_dc=%d\n", rdma_golden_setting->is_dc);
		DDPDUMP("mmsys_clk=%d\n", rdma_golden_setting->mmsys_clk);
		DDPDUMP("fps=%d\n", rdma_golden_setting->fps);
		DDPDUMP("is_one_layer=%d\n", rdma_golden_setting->is_one_layer);
		DDPDUMP("rdma_width=%d\n", rdma_golden_setting->dst_width);
		DDPDUMP("rdma_height=%d\n", rdma_golden_setting->dst_height);
	}
}

void rdma_dump_reg(enum DISP_MODULE_ENUM module)
{
	if (disp_helper_get_option(DISP_OPT_REG_PARSER_RAW_DUMP)) {
		unsigned int idx = rdma_index(module);
		unsigned long module_base = DDP_REG_BASE_DISP_RDMA0 + idx * DISP_RDMA_INDEX_OFFSET;

		DDPDUMP("== START: DISP RDMA0 REGS ==\n");
		DDPDUMP("RDMA0: 0x%04x=0x%08x, 0x%04x=0x%08x, 0x%04x=0x%08x, 0x%04x=0x%08x\n",
			0x000, INREG32(module_base + 0x000),
			0x004, INREG32(module_base + 0x004),
			0x010, INREG32(module_base + 0x010),
			0x014, INREG32(module_base + 0x014));
		DDPDUMP("RDMA0: 0x%04x=0x%08x, 0x%04x=0x%08x, 0x%04x=0x%08x, 0x%04x=0x%08x\n",
			0x018, INREG32(module_base + 0x018),
			0x01c, INREG32(module_base + 0x01c),
			0x024, INREG32(module_base + 0x024),
			0x02C, INREG32(module_base + 0x02C));
		DDPDUMP("RDMA0: 0x%04x=0x%08x, 0x%04x=0x%08x, 0x%04x=0x%08x, 0x%04x=0x%08x\n",
			0x030, INREG32(module_base + 0x030),
			0x034, INREG32(module_base + 0x034),
			0x038, INREG32(module_base + 0x038),
			0x03C, INREG32(module_base + 0x03C));
		DDPDUMP("RDMA0: 0x%04x=0x%08x, 0x%04x=0x%08x, 0x%04x=0x%08x, 0x%04x=0x%08x\n",
			0x040, INREG32(module_base + 0x040),
			0x044, INREG32(module_base + 0x044),
			0x054, INREG32(module_base + 0x054),
			0x058, INREG32(module_base + 0x058));
		DDPDUMP("RDMA0: 0x%04x=0x%08x, 0x%04x=0x%08x, 0x%04x=0x%08x, 0x%04x=0x%08x\n",
			0x05C, INREG32(module_base + 0x05C),
			0x060, INREG32(module_base + 0x060),
			0x064, INREG32(module_base + 0x064),
			0x068, INREG32(module_base + 0x068));
		DDPDUMP("RDMA0: 0x%04x=0x%08x, 0x%04x=0x%08x, 0x%04x=0x%08x, 0x%04x=0x%08x\n",
			0x06C, INREG32(module_base + 0x06C),
			0x070, INREG32(module_base + 0x070),
			0x074, INREG32(module_base + 0x074),
			0x078, INREG32(module_base + 0x078));
		DDPDUMP("RDMA0: 0x%04x=0x%08x, 0x%04x=0x%08x, 0x%04x=0x%08x, 0x%04x=0x%08x\n",
			0x07C, INREG32(module_base + 0x07C),
			0x080, INREG32(module_base + 0x080),
			0x084, INREG32(module_base + 0x084),
			0x088, INREG32(module_base + 0x088));
		DDPDUMP("RDMA0: 0x%04x=0x%08x, 0x%04x=0x%08x, 0x%04x=0x%08x, 0x%04x=0x%08x\n",
			0x08C, INREG32(module_base + 0x08C),
			0x090, INREG32(module_base + 0x090),
			0x094, INREG32(module_base + 0x094),
			0xF00, INREG32(module_base + 0xF00));
		DDPDUMP("RDMA0: 0x%04x=0x%08x, 0x%04x=0x%08x, 0x%04x=0x%08x, 0x%04x=0x%08x\n",
			0x0a0, INREG32(module_base + 0x0a0),
			0x0a4, INREG32(module_base + 0x0a4),
			0x0a8, INREG32(module_base + 0x0a8),
			0x0ac, INREG32(module_base + 0x0ac));
		DDPDUMP("RDMA0: 0x%04x=0x%08x, 0x%04x=0x%08x, 0x%04x=0x%08x, 0x%04x=0x%08x\n",
			0x0b0, INREG32(module_base + 0x0b0),
			0x0b4, INREG32(module_base + 0x0b4),
			0x0b8, INREG32(module_base + 0x0b8),
			0x0c0, INREG32(module_base + 0x0c0));
		DDPDUMP("-- END: DISP RDMA0 REGS --\n");
	} else {
		unsigned int idx = rdma_index(module);

		DDPDUMP("== DISP RDMA%d REGS ==\n", idx);
		DDPDUMP("(0x000)R_INTEN=0x%x\n",
			DISP_REG_GET(DISP_REG_RDMA_INT_ENABLE + DISP_RDMA_INDEX_OFFSET * idx));
		DDPDUMP("(0x004)R_INTS=0x%x\n",
			DISP_REG_GET(DISP_REG_RDMA_INT_STATUS + DISP_RDMA_INDEX_OFFSET * idx));
		DDPDUMP("(0x010)R_CON=0x%x\n",
			DISP_REG_GET(DISP_REG_RDMA_GLOBAL_CON + DISP_RDMA_INDEX_OFFSET * idx));
		DDPDUMP("(0x014)R_SIZE0=0x%x\n",
			DISP_REG_GET(DISP_REG_RDMA_SIZE_CON_0 + DISP_RDMA_INDEX_OFFSET * idx));
		DDPDUMP("(0x018)R_SIZE1=0x%x\n",
			DISP_REG_GET(DISP_REG_RDMA_SIZE_CON_1 + DISP_RDMA_INDEX_OFFSET * idx));
		DDPDUMP("(0x01c)R_TAR_LINE=0x%x\n",
			DISP_REG_GET(DISP_REG_RDMA_TARGET_LINE + DISP_RDMA_INDEX_OFFSET * idx));
		DDPDUMP("(0x024)R_M_CON=0x%x\n",
			DISP_REG_GET(DISP_REG_RDMA_MEM_CON + DISP_RDMA_INDEX_OFFSET * idx));
		DDPDUMP("(0xf00)R_M_S_ADDR=0x%x\n",
			DISP_REG_GET(DISP_REG_RDMA_MEM_START_ADDR + DISP_RDMA_INDEX_OFFSET * idx));
		DDPDUMP("(0x02c)R_M_SRC_PITCH=0x%x\n",
			DISP_REG_GET(DISP_REG_RDMA_MEM_SRC_PITCH + DISP_RDMA_INDEX_OFFSET * idx));
		DDPDUMP("(0x030)R_M_GMC_SET0=0x%x\n",
			DISP_REG_GET(DISP_REG_RDMA_MEM_GMC_SETTING_0 + DISP_RDMA_INDEX_OFFSET * idx));
		DDPDUMP("(0x034)R_M_GMC_SET1=0x%x\n",
			DISP_REG_GET(DISP_REG_RDMA_MEM_GMC_SETTING_1 + DISP_RDMA_INDEX_OFFSET * idx));
		DDPDUMP("(0x038)R_M_SLOW_CON=0x%x\n",
			DISP_REG_GET(DISP_REG_RDMA_MEM_SLOW_CON + DISP_RDMA_INDEX_OFFSET * idx));
		DDPDUMP("(0x03c)R_M_GMC_SET2=0x%x\n",
			DISP_REG_GET(DISP_REG_RDMA_MEM_GMC_SETTING_2 + DISP_RDMA_INDEX_OFFSET * idx));
		DDPDUMP("(0x040)R_FIFO_CON=0x%x\n",
			DISP_REG_GET(DISP_REG_RDMA_FIFO_CON + DISP_RDMA_INDEX_OFFSET * idx));
		DDPDUMP("(0x044)R_FIFO_LOG=0x%x\n",
			DISP_REG_GET(DISP_REG_RDMA_FIFO_LOG + DISP_RDMA_INDEX_OFFSET * idx));
		DDPDUMP("(0x078)R_PRE_ADD0=0x%x\n",
			DISP_REG_GET(DISP_REG_RDMA_PRE_ADD_0 + DISP_RDMA_INDEX_OFFSET * idx));
		DDPDUMP("(0x07c)R_PRE_ADD1=0x%x\n",
			DISP_REG_GET(DISP_REG_RDMA_PRE_ADD_1 + DISP_RDMA_INDEX_OFFSET * idx));
		DDPDUMP("(0x080)R_PRE_ADD2=0x%x\n",
			DISP_REG_GET(DISP_REG_RDMA_PRE_ADD_2 + DISP_RDMA_INDEX_OFFSET * idx));
		DDPDUMP("(0x084)R_POST_ADD0=0x%x\n",
			DISP_REG_GET(DISP_REG_RDMA_POST_ADD_0 + DISP_RDMA_INDEX_OFFSET * idx));
		DDPDUMP("(0x088)R_POST_ADD1=0x%x\n",
			DISP_REG_GET(DISP_REG_RDMA_POST_ADD_1 + DISP_RDMA_INDEX_OFFSET * idx));
		DDPDUMP("(0x08c)R_POST_ADD2=0x%x\n",
			DISP_REG_GET(DISP_REG_RDMA_POST_ADD_2 + DISP_RDMA_INDEX_OFFSET * idx));
		DDPDUMP("(0x090)R_DUMMY=0x%x\n",
			DISP_REG_GET(DISP_REG_RDMA_DUMMY + DISP_RDMA_INDEX_OFFSET * idx));
		DDPDUMP("(0x094)R_OUT_SEL=0x%x\n",
			DISP_REG_GET(DISP_REG_RDMA_DEBUG_OUT_SEL + DISP_RDMA_INDEX_OFFSET * idx));
		DDPDUMP("(0x094)R_M_START=0x%x\n",
			DISP_REG_GET(DISP_REG_RDMA_MEM_START_ADDR + DISP_RDMA_INDEX_OFFSET * idx));
		DDPDUMP("(0x0a0)R_BG_CON_0=0x%x\n",
			DISP_REG_GET(DISP_REG_RDMA_BG_CON_0 + DISP_RDMA_INDEX_OFFSET * idx));
		DDPDUMP("(0x0a4)R_BG_CON_1=0x%x\n",
			DISP_REG_GET(DISP_REG_RDMA_BG_CON_1 + DISP_RDMA_INDEX_OFFSET * idx));
		DDPDUMP("(0x0a8)R_FOR_SODI=0x%x\n",
			DISP_REG_GET(DISP_REG_RDMA_THRESHOLD_FOR_SODI + DISP_RDMA_INDEX_OFFSET * idx));
		DDPDUMP("(0x0ac)R_FOR_DVFS=0x%x\n",
			DISP_REG_GET(DISP_REG_RDMA_THRESHOLD_FOR_DVFS + DISP_RDMA_INDEX_OFFSET * idx));
		DDPDUMP("(0x0b0)R_FOR_SRAM=0x%x\n",
			DISP_REG_GET(DISP_REG_RDMA_SRAM_SEL + DISP_RDMA_INDEX_OFFSET * idx));
		DDPDUMP("(0x0b4)DISP_REG_RDMA_STALL_CG_CON=0x%x\n",
			DISP_REG_GET(DISP_REG_RDMA_STALL_CG_CON + DISP_RDMA_INDEX_OFFSET * idx));
		DDPDUMP("(0x0b8)DISP_REG_RDMA_SHADOW_UPDATE=0x%x\n",
			DISP_REG_GET(DISP_REG_RDMA_SHADOW_UPDATE + DISP_RDMA_INDEX_OFFSET * idx));
		DDPDUMP("(0x0f0)R_IN_PXL_CNT=0x%x\n",
			DISP_REG_GET(DISP_REG_RDMA_IN_P_CNT + DISP_RDMA_INDEX_OFFSET * idx));
		DDPDUMP("(0x0f4)R_IN_LINE_CNT=0x%x\n",
			DISP_REG_GET(DISP_REG_RDMA_IN_LINE_CNT + DISP_RDMA_INDEX_OFFSET * idx));
		DDPDUMP("(0x0f8)R_OUT_PXL_CNT=0x%x\n",
			DISP_REG_GET(DISP_REG_RDMA_OUT_P_CNT + DISP_RDMA_INDEX_OFFSET * idx));
		DDPDUMP("(0x0fc)R_OUT_LINE_CNT=0x%x\n",
			DISP_REG_GET(DISP_REG_RDMA_OUT_LINE_CNT + DISP_RDMA_INDEX_OFFSET * idx));
		DDPDUMP("(0x100)R_DBG_OUT=0x%x\n",
			DISP_REG_GET(DISP_REG_RDMA_DBG_OUT + DISP_RDMA_INDEX_OFFSET * idx));
	}
}

void rdma_dump_analysis(enum DISP_MODULE_ENUM module)
{
	unsigned int idx = rdma_index(module);
	unsigned int global_ctrl = DISP_REG_GET(DISP_REG_RDMA_GLOBAL_CON + DISP_RDMA_INDEX_OFFSET * idx);
	unsigned int bg0 = DISP_REG_GET(idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_BG_CON_0);
	unsigned int bg1 = DISP_REG_GET(idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_BG_CON_1);

	DDPDUMP("== DISP RDMA%d ANALYSIS ==\n", idx);
	DDPDUMP("rdma%d: en=%d,memory_mode=%d,smi_busy=%d,w=%d,h=%d,pitch=%d,addr=0x%x,fmt=%s,fifo_min=%d,\n",
		idx, REG_FLD_VAL_GET(GLOBAL_CON_FLD_ENGINE_EN, global_ctrl),
		REG_FLD_VAL_GET(GLOBAL_CON_FLD_MODE_SEL, global_ctrl),
		REG_FLD_VAL_GET(GLOBAL_CON_FLD_SMI_BUSY, global_ctrl),
		DISP_REG_GET(DISP_REG_RDMA_SIZE_CON_0 + DISP_RDMA_INDEX_OFFSET * idx) & 0xfff,
		DISP_REG_GET(DISP_REG_RDMA_SIZE_CON_1 + DISP_RDMA_INDEX_OFFSET * idx) & 0xfffff,
		DISP_REG_GET(DISP_REG_RDMA_MEM_SRC_PITCH + DISP_RDMA_INDEX_OFFSET * idx),
		DISP_REG_GET(DISP_REG_RDMA_MEM_START_ADDR + DISP_RDMA_INDEX_OFFSET * idx),
		unified_color_fmt_name(
			display_fmt_reg_to_unified_fmt((DISP_REG_GET(DISP_REG_RDMA_MEM_CON +
								     DISP_RDMA_INDEX_OFFSET * idx) >> 4) & 0xf,
						       (DISP_REG_GET(DISP_REG_RDMA_MEM_CON +
								      DISP_RDMA_INDEX_OFFSET * idx) >> 8) & 0x1, 0)),
		DISP_REG_GET(DISP_REG_RDMA_FIFO_LOG + DISP_RDMA_INDEX_OFFSET * idx));
	DDPDUMP("in_p=%d,in_l=%d,out_p=%d,out_l=%d,bg(t%d,b%d,l%d,r%d),start=%lld ns,end=%lld ns\n",
		DISP_REG_GET(DISP_REG_RDMA_IN_P_CNT + DISP_RDMA_INDEX_OFFSET * idx),
		DISP_REG_GET(DISP_REG_RDMA_IN_LINE_CNT + DISP_RDMA_INDEX_OFFSET * idx),
		DISP_REG_GET(DISP_REG_RDMA_OUT_P_CNT + DISP_RDMA_INDEX_OFFSET * idx),
		DISP_REG_GET(DISP_REG_RDMA_OUT_LINE_CNT + DISP_RDMA_INDEX_OFFSET * idx),
		REG_FLD_VAL_GET(RDMA_BG_CON_1_TOP, bg1),
		REG_FLD_VAL_GET(RDMA_BG_CON_1_BOTTOM, bg1),
		REG_FLD_VAL_GET(RDMA_BG_CON_0_LEFT, bg0),
		REG_FLD_VAL_GET(RDMA_BG_CON_0_RIGHT, bg0),
		rdma_start_time[idx], rdma_end_time[idx]);
	DDPDUMP("irq cnt: start=%d, end=%d, underflow=%d, targetline=%d\n",
		rdma_start_irq_cnt[idx], rdma_done_irq_cnt[idx], rdma_underflow_irq_cnt[idx],
		rdma_targetline_irq_cnt[idx]);

	rdma_dump_golden_setting_context(module);
}

static int rdma_dump(enum DISP_MODULE_ENUM module, int level)
{
	rdma_dump_analysis(module);
	rdma_dump_reg(module);

	return 0;
}

void rdma_get_info(int idx, struct RDMA_BASIC_STRUCT *info)
{
	struct RDMA_BASIC_STRUCT *p = info;

	p->addr = DISP_REG_GET(DISP_REG_RDMA_MEM_START_ADDR + DISP_RDMA_INDEX_OFFSET * idx);
	p->src_w = DISP_REG_GET(DISP_REG_RDMA_SIZE_CON_0 + DISP_RDMA_INDEX_OFFSET * idx) & 0xfff;
	p->src_h = DISP_REG_GET(DISP_REG_RDMA_SIZE_CON_1 + DISP_RDMA_INDEX_OFFSET * idx) & 0xfffff,
	p->bpp = UFMT_GET_bpp(display_fmt_reg_to_unified_fmt((DISP_REG_GET(DISP_REG_RDMA_MEM_CON +
									   DISP_RDMA_INDEX_OFFSET *
									   idx) >> 4) & 0xf,
							     (DISP_REG_GET(DISP_REG_RDMA_MEM_CON +
									   DISP_RDMA_INDEX_OFFSET *
									   idx) >> 8) & 0x1, 0)) / 8;
}

static inline enum RDMA_MODE get_rdma_mode(enum DISP_MODULE_ENUM module)
{
	unsigned int idx = rdma_index(module);

	return DISP_REG_GET_FIELD(GLOBAL_CON_FLD_MODE_SEL, (DISP_RDMA_INDEX_OFFSET * idx) + DISP_REG_RDMA_GLOBAL_CON);
}

static inline enum RDMA_MODE rdma_config_mode(unsigned long address)
{
	return address ? RDMA_MODE_MEMORY : RDMA_MODE_DIRECT_LINK;
}

static int do_rdma_config_l(enum DISP_MODULE_ENUM module, struct disp_ddp_path_config *pConfig, void *handle)
{
	struct RDMA_CONFIG_STRUCT *r_config = &pConfig->rdma_config;
	enum RDMA_MODE mode = rdma_config_mode(r_config->address);
	LCM_PARAMS *lcm_param = &(pConfig->dispif_config);
	unsigned int width = pConfig->dst_dirty ? pConfig->dst_w : r_config->width;
	unsigned int height = pConfig->dst_dirty ? pConfig->dst_h : r_config->height;
	struct golden_setting_context *p_golden_setting = pConfig->p_golden_setting_context;
	enum UNIFIED_COLOR_FMT inFormat = r_config->inputFormat;

	if (pConfig->fps)
		rdma_fps[rdma_index(module)] = pConfig->fps / 100;

	if (mode == RDMA_MODE_DIRECT_LINK && r_config->security != DISP_NORMAL_BUFFER)
		DDPERR("%s: rdma directlink BUT is sec ??!!\n", __func__);

	if (mode == RDMA_MODE_DIRECT_LINK) {
		pConfig->rdma_config.bg_ctrl.top = 0;
		pConfig->rdma_config.bg_ctrl.bottom = 0;
		pConfig->rdma_config.bg_ctrl.left = 0;
		pConfig->rdma_config.bg_ctrl.right = 0;
	} else if (mode == RDMA_MODE_MEMORY) {
		pConfig->rdma_config.bg_ctrl.top = r_config->dst_y;
		pConfig->rdma_config.bg_ctrl.bottom = r_config->dst_h -
			r_config->dst_y - height;
		pConfig->rdma_config.bg_ctrl.left = r_config->dst_x;
		pConfig->rdma_config.bg_ctrl.right = r_config->dst_w -
			r_config->dst_x - width;
	}
	DDPDBG("top=%d,bottom=%d,left=%d,right=%d,r.dst_x=%d,r.dst_y=%d,r.dst_w=%d,r.dst_h=%d,width=%d,height=%d\n",
		pConfig->rdma_config.bg_ctrl.top, pConfig->rdma_config.bg_ctrl.bottom,
		pConfig->rdma_config.bg_ctrl.left, pConfig->rdma_config.bg_ctrl.right,
		r_config->dst_x, r_config->dst_y, r_config->dst_w,
		r_config->dst_h, width, height);
	/*PARGB,etc need convert ARGB,etc*/
	ufmt_disable_P(r_config->inputFormat, &inFormat);
	rdma_config(module,
		    mode,
		    (mode == RDMA_MODE_DIRECT_LINK) ? 0 : r_config->address,
		    (mode == RDMA_MODE_DIRECT_LINK) ? UFMT_RGB888 : inFormat,
		    (mode == RDMA_MODE_DIRECT_LINK) ? 0 : r_config->pitch,
		    width,
		    height,
		    lcm_param->dsi.ufoe_enable,
		    r_config->security, r_config->yuv_range,
		    &(r_config->bg_ctrl), handle, p_golden_setting, pConfig->lcm_bpp);

	return 0;
}

static int rdma_is_sec[2];
#if 0
static int setup_rdma_sec(enum DISP_MODULE_ENUM module, struct disp_ddp_path_config *pConfig, void *handle)
{
	static int rdma_is_sec[2];
	enum CMDQ_ENG_ENUM cmdq_engine;
	enum CMDQ_EVENT_ENUM cmdq_event_nonsec_end;
	int rdma_idx = rdma_index(module);
	enum DISP_BUFFER_TYPE security = pConfig->rdma_config.security;
	enum RDMA_MODE mode = rdma_config_mode(pConfig->rdma_config.address);

	/*cmdq_engine = rdma_idx == 0 ? CMDQ_ENG_DISP_RDMA0 : CMDQ_ENG_DISP_RDMA1;*/
	cmdq_engine = rdma_to_cmdq_engine(module);
	cmdq_event_nonsec_end = rdma_to_cmdq_event_nonsec_end(module);

	if (!handle) {
		DDPMSG("[SVP] bypass rdma sec setting sec=%d,handle=NULL\n", security);
		return 0;
	}
	/* sec setting make sence only in memory mode ! */
	if (mode == RDMA_MODE_MEMORY) {
		if (security == DISP_SECURE_BUFFER) {
			cmdqRecSetSecure(handle, 1);
			/* set engine as sec */
			cmdqRecSecureEnablePortSecurity(handle, (1LL << cmdq_engine));
			/* cmdqRecSecureEnableDAPC(handle, (1LL << cmdq_engine)); */

			if (rdma_is_sec[rdma_idx] == 0)
				DDPMSG("[SVP] switch rdma%d to sec\n", rdma_idx);
			rdma_is_sec[rdma_idx] = 1;
		} else {
			if (rdma_is_sec[rdma_idx]) {
				/* rdma is in sec stat, we need to switch it to nonsec */
				struct cmdqRecStruct *nonsec_switch_handle;
				int ret;

				ret = cmdqRecCreate(CMDQ_SCENARIO_DISP_PRIMARY_DISABLE_SECURE_PATH,
						    &(nonsec_switch_handle));
				if (ret)
					DDPAEE("[SVP]fail to create disable handle %s ret=%d\n",
					       __func__, ret);

				cmdqRecReset(nonsec_switch_handle);
				_cmdq_insert_wait_frame_done_token_mira(nonsec_switch_handle);
				cmdqRecSetSecure(nonsec_switch_handle, 1);

				 /* To avoid translation fault like ovl (see notes in ovl.c)*/
				if (get_rdma_mode(module) == RDMA_MODE_MEMORY)
					do_rdma_config_l(module, pConfig, nonsec_switch_handle);

				/*in fact, dapc/port_sec will be disabled by cmdq */
				cmdqRecSecureEnablePortSecurity(nonsec_switch_handle,
								(1LL << cmdq_engine));
				/* cmdqRecSecureEnableDAPC(nonsec_switch_handle, (1LL << cmdq_engine)); */
				cmdqRecSetEventToken(nonsec_switch_handle, cmdq_event_nonsec_end);
				cmdqRecFlushAsync(nonsec_switch_handle);
				/*cmdqRecFlush(nonsec_switch_handle);*/
				cmdqRecDestroy(nonsec_switch_handle);
				cmdqRecWait(handle, cmdq_event_nonsec_end);
				DDPMSG("[SVP] switch rdma%d to nonsec done\n", rdma_idx);
			}
			rdma_is_sec[rdma_idx] = 0;
		}
	}
	return 0;
}
#else

static inline int rdma_switch_to_sec(enum DISP_MODULE_ENUM module, void *handle)
{
	unsigned int rdma_idx = rdma_index(module);
	enum CMDQ_ENG_ENUM cmdq_engine;

	cmdq_engine = rdma_to_cmdq_engine(module);
	cmdqRecSetSecure(handle, 1);
	/* set engine as sec port */
	cmdqRecSecureEnablePortSecurity(handle, (1LL << cmdq_engine));
	/* cmdqRecSecureEnableDAPC(handle, (1LL << cmdq_engine)); */
	if (rdma_is_sec[rdma_idx] == 0) {
		DDPSVPMSG("[SVP] switch rdma%d to sec\n", rdma_idx);
		MMProfileLogEx(ddp_mmp_get_events()->svp_module[module],
			MMProfileFlagStart, 0, 0);
		/*MMProfileLogEx(ddp_mmp_get_events()->svp_module[module],
		 *	MMProfileFlagPulse, rdma_idx, 1);
		 */
	}
	rdma_is_sec[rdma_idx] = 1;

	return 0;
}

int rdma_switch_to_nonsec(enum DISP_MODULE_ENUM module, struct disp_ddp_path_config *pConfig, void *handle)
{
	unsigned int rdma_idx = rdma_index(module);

	enum CMDQ_ENG_ENUM cmdq_engine;

	cmdq_engine = rdma_to_cmdq_engine(module);
	if (rdma_is_sec[rdma_idx] == 1) {
		/* rdma is in sec stat, we need to switch it to nonsec */
		struct cmdqRecStruct *nonsec_switch_handle;
		int ret;

		ret = cmdqRecCreate(CMDQ_SCENARIO_DISP_PRIMARY_DISABLE_SECURE_PATH,
				&(nonsec_switch_handle));
		if (ret)
			DDPAEE("[SVP]fail to create disable handle %s ret=%d\n",
				__func__, ret);

		cmdqRecReset(nonsec_switch_handle);

		if (rdma_idx == 0) {
			/*Primary Decouple Mode*/
			_cmdq_insert_wait_frame_done_token_mira(nonsec_switch_handle);
		} else {
			/*External Mode*/
			/*ovl1->Rdma1, do not used.*/
			_cmdq_insert_wait_frame_done_token_mira(nonsec_switch_handle);
		}

		cmdqRecSetSecure(nonsec_switch_handle, 1);
		/*ugly work around by kzhang !!. will remove when cmdq delete disable scenario.
		 * To avoid translation fault like ovl (see notes in ovl.c)
		 *check the mode now, bypass the frame during DL->DC(), avoid hang when vdo mode.
		 */
		if (get_rdma_mode(module) == RDMA_MODE_MEMORY)
			do_rdma_config_l(module, pConfig, nonsec_switch_handle);

		/*in fact, dapc/port_sec will be disabled by cmdq */
		cmdqRecSecureEnablePortSecurity(nonsec_switch_handle,
			(1LL << cmdq_engine));
		/* cmdqRecSecureEnableDAPC(nonsec_switch_handle, (1LL << cmdq_engine)); */
		if (handle != NULL) {
			/*Async Flush method*/
			enum CMDQ_EVENT_ENUM cmdq_event_nonsec_end;
			/*cmdq_event_nonsec_end = module_to_cmdq_event_nonsec_end(module);*/
			cmdq_event_nonsec_end = rdma_to_cmdq_event_nonsec_end(module);
			cmdqRecSetEventToken(nonsec_switch_handle, cmdq_event_nonsec_end);
			cmdqRecFlushAsync(nonsec_switch_handle);
			cmdqRecWait(handle, cmdq_event_nonsec_end);
		} else {
			/*Sync Flush method*/
			cmdqRecFlush(nonsec_switch_handle);
		}

		cmdqRecDestroy(nonsec_switch_handle);
		DDPSVPMSG("[SVP] switch rdma%d to nonsec\n", rdma_idx);
		MMProfileLogEx(ddp_mmp_get_events()->svp_module[module],
			MMProfileFlagEnd, 0, 0);
		/*MMProfileLogEx(ddp_mmp_get_events()->svp_module[module],
		 *	MMProfileFlagPulse, rdma_idx, 0);
		 */
	}

	rdma_is_sec[rdma_idx] = 0;

	return 0;
}

static int setup_rdma_sec(enum DISP_MODULE_ENUM module, struct disp_ddp_path_config *pConfig, void *handle)
{
	int ret;
	int is_engine_sec = 0;

	enum RDMA_MODE mode = rdma_config_mode(pConfig->rdma_config.address);

	if (pConfig->rdma_config.security == DISP_SECURE_BUFFER)
		is_engine_sec = 1;

	if (!handle) {
		DDPDBG("[SVP] bypass rdma sec setting sec=%d,handle=NULL\n", is_engine_sec);

		return 0;
		}

	/* sec setting make sence only in memory mode ! */
	if (mode == RDMA_MODE_MEMORY) {
		if (is_engine_sec == 1)
			ret = rdma_switch_to_sec(module, handle);
		else
			ret = rdma_switch_to_nonsec(module, pConfig, NULL);/*hadle = NULL, use the sync flush method*/
		if (ret)
			DDPAEE("[SVP]fail to setup_ovl_sec: %s ret=%d\n",
				__func__, ret);
	}

	return is_engine_sec;
}
#endif


static int rdma_config_l(enum DISP_MODULE_ENUM module, struct disp_ddp_path_config *pConfig, void *handle)
{
	if (pConfig->dst_dirty || pConfig->rdma_dirty) {
		setup_rdma_sec(module, pConfig, handle);
		do_rdma_config_l(module, pConfig, handle);
	}
	return 0;
}

void rdma_enable_color_transform(enum DISP_MODULE_ENUM module)
{
	unsigned int idx = rdma_index(module);
	UINT32 value = DISP_REG_GET(DISP_REG_RDMA_SIZE_CON_0 + DISP_RDMA_INDEX_OFFSET * idx);

	value = value | REG_FLD_VAL((SIZE_CON_0_FLD_MATRIX_EXT_MTX_EN), 1) |
		REG_FLD_VAL((SIZE_CON_0_FLD_MATRIX_ENABLE), 1);
	DISP_REG_SET(NULL, idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_SIZE_CON_0, value);
}

void rdma_disable_color_transform(enum DISP_MODULE_ENUM module)
{
	unsigned int idx = rdma_index(module);
	UINT32 value = DISP_REG_GET(DISP_REG_RDMA_SIZE_CON_0 + DISP_RDMA_INDEX_OFFSET * idx);

	value = value & ~(REG_FLD_VAL((SIZE_CON_0_FLD_MATRIX_EXT_MTX_EN), 1) |
		REG_FLD_VAL((SIZE_CON_0_FLD_MATRIX_ENABLE), 1));
	DISP_REG_SET(NULL, idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_SIZE_CON_0, value);
}

void rdma_set_color_matrix(enum DISP_MODULE_ENUM module,
			   struct rdma_color_matrix *matrix, struct rdma_color_pre *pre, struct rdma_color_post *post)
{
	unsigned int idx = rdma_index(module);

	DISP_REG_SET(NULL, idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_C00, matrix->C00);
	DISP_REG_SET(NULL, idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_C01, matrix->C01);
	DISP_REG_SET(NULL, idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_C02, matrix->C02);
	DISP_REG_SET(NULL, idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_C10, matrix->C10);
	DISP_REG_SET(NULL, idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_C11, matrix->C11);
	DISP_REG_SET(NULL, idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_C12, matrix->C12);
	DISP_REG_SET(NULL, idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_C20, matrix->C20);
	DISP_REG_SET(NULL, idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_C21, matrix->C21);
	DISP_REG_SET(NULL, idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_C22, matrix->C22);

	DISP_REG_SET(NULL, idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_PRE_ADD_0, pre->ADD0);
	DISP_REG_SET(NULL, idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_PRE_ADD_1, pre->ADD1);
	DISP_REG_SET(NULL, idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_PRE_ADD_2, pre->ADD2);

	DISP_REG_SET(NULL, idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_POST_ADD_0, post->ADD0);
	DISP_REG_SET(NULL, idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_POST_ADD_1, post->ADD1);
	DISP_REG_SET(NULL, idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_POST_ADD_2, post->ADD2);
}

static int _rdma_partial_update(enum DISP_MODULE_ENUM module, void *arg, void *handle)
{
	struct disp_rect *roi = (struct disp_rect *)arg;
	int width = roi->width;
	int height = roi->height;
	unsigned int idx = rdma_index(module);

	DISP_REG_SET_FIELD(handle, SIZE_CON_0_FLD_OUTPUT_FRAME_WIDTH,
			idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_SIZE_CON_0, width);
	DISP_REG_SET_FIELD(handle, SIZE_CON_1_FLD_OUTPUT_FRAME_HEIGHT,
			idx * DISP_RDMA_INDEX_OFFSET + DISP_REG_RDMA_SIZE_CON_1, height);
	return 0;
}

int rdma_ioctl(enum DISP_MODULE_ENUM module, void *cmdq_handle, unsigned int ioctl_cmd, unsigned long *params)
{
	int ret = 0;
	enum DDP_IOCTL_NAME ioctl = (enum DDP_IOCTL_NAME)ioctl_cmd;
	unsigned int idx = rdma_index(module);

	switch (ioctl) {
	case DDP_RDMA_GOLDEN_SETTING:
	{
		struct disp_ddp_path_config *pConfig = (struct disp_ddp_path_config *)params;
		struct golden_setting_context *p_golden_setting = pConfig->p_golden_setting_context;

		rdma_set_ultra_l(idx, pConfig->lcm_bpp, cmdq_handle, p_golden_setting);
		break;
	}
	case DDP_PARTIAL_UPDATE:
		_rdma_partial_update(module, params, cmdq_handle);
		break;
	default:
		break;
	}

	return ret;
}

static int rdma_build_cmdq(enum DISP_MODULE_ENUM module, void *handle, enum CMDQ_STATE state)
{
	if (handle == NULL) {
		DDPERR("cmdq_trigger_handle is NULL\n");
		return -1;
	}
	if (state == CMDQ_RESET_AFTER_STREAM_EOF) {
		/* if rdma frame done with underflow, rdma will hold dvfs request forever */
		/* we reset here to solve this issue */
		rdma_reset_by_cmdq(module, handle);
	}

	return 0;
}

struct DDP_MODULE_DRIVER ddp_driver_rdma = {
	.init = rdma_init,
	.deinit = rdma_deinit,
	.config = rdma_config_l,
	.start = rdma_start,
	.trigger = NULL,
	.stop = rdma_stop,
	.reset = rdma_reset,
	.power_on = rdma_clock_on,
	.power_off = rdma_clock_off,
	.is_idle = NULL,
	.is_busy = NULL,
	.dump_info = rdma_dump,
	.bypass = NULL,
	.build_cmdq = rdma_build_cmdq,
	.set_lcm_utils = NULL,
	.enable_irq = rdma_enable_irq,
	.ioctl = (int (*)(enum DISP_MODULE_ENUM, void *, enum DDP_IOCTL_NAME, void *))rdma_ioctl,
	.switch_to_nonsec = NULL,
};
