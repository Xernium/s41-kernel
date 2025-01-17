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
*
* You should have received a copy of the GNU General Public License
* along with this program.
* If not, see <http://www.gnu.org/licenses/>.
*/

/*******************************************************************************
 *
 * Filename:
 * ---------
 *   mtk_pcm_i2s0_awb.c
 *
 * Project:
 * --------
 *   Audio Driver Kernel Function
 *
 * Description:
 * ------------
 *   Audio i2s0 to  awb capture
 *
 * Author:
 * -------
 * Chipeng Chang
 *
 *------------------------------------------------------------------------------
 *
 *
 *******************************************************************************/


/*****************************************************************************
 *                     C O M P I L E R   F L A G S
 *****************************************************************************/


/*****************************************************************************
 *                E X T E R N A L   R E F E R E N C E S
 *****************************************************************************/

#include <linux/dma-mapping.h>
#include "mtk-auddrv-common.h"
#include "mtk-soc-pcm-common.h"
#include "mtk-auddrv-def.h"
#include "mtk-auddrv-afe.h"
#include "mtk-auddrv-ana.h"
#include "mtk-auddrv-clk.h"
#include "mtk-auddrv-kernel.h"
#include "mtk-soc-afe-control.h"
#include "mtk-soc-pcm-platform.h"

/* information about */
static AFE_MEM_CONTROL_T  *I2S0_AWB_Control_context;
static struct snd_dma_buffer *Awb_Capture_dma_buf;
/*
 *    function implementation
 */
static void StartAudioI2sInAWBHardware(struct snd_pcm_substream *substream);
static void StopAudioI2sInAWBHardware(struct snd_pcm_substream *substream);
static int mtk_i2s0_awb_probe(struct platform_device *pdev);
static int mtk_i2s0_awb_pcm_close(struct snd_pcm_substream *substream);
static int mtk_asoc_i2s0_awb_pcm_new(struct snd_soc_pcm_runtime *rtd);
static int mtk_i2s0_dl1_awb_probe(struct snd_soc_platform *platform);

static struct snd_pcm_hardware mtk_I2S0_awb_hardware = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED),
	.formats =      SND_SOC_STD_MT_FMTS,
	.rates =        SOC_HIGH_USE_RATE,
	.rate_min =     SOC_HIGH_USE_RATE_MIN,
	.rate_max =     SOC_HIGH_USE_RATE_MAX,
	.channels_min =     SOC_NORMAL_USE_CHANNELS_MIN,
	.channels_max =     SOC_NORMAL_USE_CHANNELS_MAX,
	.buffer_bytes_max = AWB_MAX_BUFFER_SIZE,
	.period_bytes_max = AWB_MAX_BUFFER_SIZE,
	.periods_min =     AWB_MIN_PERIOD_SIZE,
	.periods_max =      AWB_MAX_PERIOD_SIZE,
	.fifo_size =        0,
};

static void StopAudioI2sInAWBHardware(struct snd_pcm_substream *substream)
{
	pr_warn("StopAudioI2sInAWBHardware\n");

	if (get_afe_platform_ops()->set_smartpa_echo_ref != NULL) {
		get_afe_platform_ops()->set_smartpa_echo_ref(substream->runtime->rate,
							     extcodec_echoref_control,
							     false);
		goto bypass_default_i2s_in;
	}

	/* stop default i2s in: i2s0 */
	SetIntfConnection(Soc_Aud_InterCon_DisConnect,
			  Soc_Aud_AFE_IO_Block_I2S0, Soc_Aud_AFE_IO_Block_MEM_AWB);

	SetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_IN_2, false);
	if (GetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_IN_2) == false)
		Set2ndI2SEnable(false);

bypass_default_i2s_in:

	SetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_AWB, false);

	/* here to set interrupt */
	irq_remove_user(substream, Soc_Aud_IRQ_MCU_MODE_IRQ2_MCU_MODE);

	EnableAfe(false);
}

static void StartAudioI2sInAWBHardware(struct snd_pcm_substream *substream)
{
	uint32 u32Audio2ndI2sIn = 0;
	uint32 MclkDiv0 = 0;

	pr_warn("StartAudioI2sInAWBHardware\n");

	/*
	 * SmartPa might use different i2s in different chips.
	 * Check if there is callback function for specific config in the chip.
	 * If not, use default i2s in config.
	 */
	if (get_afe_platform_ops()->set_smartpa_echo_ref != NULL) {
		get_afe_platform_ops()->set_smartpa_echo_ref(substream->runtime->rate,
							     extcodec_echoref_control,
							     true);
		goto bypass_default_i2s_in;
	}

	/* default i2s in for echo reference is i2s0 */
	MclkDiv0 = SetCLkMclk(Soc_Aud_I2S0, substream->runtime->rate); /* select I2S */
	SetCLkBclk(MclkDiv0, substream->runtime->rate, substream->runtime->channels,
		   Soc_Aud_I2S_WLEN_WLEN_32BITS);

	SetSampleRate(Soc_Aud_Digital_Block_MEM_I2S, substream->runtime->rate);

	Afe_Set_Reg(AUDIO_TOP_CON1, 0x1 << 4,  0x1 << 4); /* I2S0 clock-gated */
	SetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_IN_2, true);
	u32Audio2ndI2sIn |= (Soc_Aud_LR_SWAP_NO_SWAP << 31);
	u32Audio2ndI2sIn |= (Soc_Aud_LOW_JITTER_CLOCK << 12);
	u32Audio2ndI2sIn |= (Soc_Aud_I2S_IN_PAD_SEL_I2S_IN_FROM_IO_MUX << 28);
	u32Audio2ndI2sIn |= (Soc_Aud_INV_LRCK_NO_INVERSE << 5);
	u32Audio2ndI2sIn |= (Soc_Aud_I2S_FORMAT_I2S << 3);
	u32Audio2ndI2sIn |= (Soc_Aud_I2S_WLEN_WLEN_32BITS << 1);
	Afe_Set_Reg(AFE_I2S_CON, u32Audio2ndI2sIn, MASK_ALL);

	Afe_Set_Reg(AUDIO_TOP_CON1, 0 << 4,  0x1 << 4); /* Clear I2S0 clock-gated */
	Set2ndI2SEnable(true);				 /* Enable I2S0 */

	SetIntfConnection(Soc_Aud_InterCon_Connection,
			  Soc_Aud_AFE_IO_Block_I2S0, Soc_Aud_AFE_IO_Block_MEM_AWB);

bypass_default_i2s_in:

	/* here to set interrupt */
	irq_add_user(substream,
		     Soc_Aud_IRQ_MCU_MODE_IRQ2_MCU_MODE,
		     substream->runtime->rate,
		     substream->runtime->period_size >> 1);

	SetSampleRate(Soc_Aud_Digital_Block_MEM_AWB, substream->runtime->rate);
	SetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_AWB, true);

	EnableAfe(true);
}

static int mtk_i2s0_awb_pcm_prepare(struct snd_pcm_substream *substream)
{
	pr_warn("mtk_i2s0_awb_pcm_prepare substream->rate = %d  substream->channels = %d\n",
	       substream->runtime->rate, substream->runtime->channels);
	return 0;
}

static int mtk_i2s0_awb_alsa_stop(struct snd_pcm_substream *substream)
{
	/* AFE_BLOCK_T *Awb_Block = &(I2S0_AWB_Control_context->rBlock); */
	pr_warn("mtk_i2s0_awb_alsa_stop\n");
	StopAudioI2sInAWBHardware(substream);
	RemoveMemifSubStream(Soc_Aud_Digital_Block_MEM_AWB, substream);
	return 0;
}

static snd_pcm_uframes_t mtk_i2s0_awb_pcm_pointer(struct snd_pcm_substream
						  *substream)
{
	return get_mem_frame_index(substream,
		I2S0_AWB_Control_context, Soc_Aud_Digital_Block_MEM_AWB);
}

static int mtk_i2s0_awb_pcm_hw_params(struct snd_pcm_substream *substream,
				      struct snd_pcm_hw_params *hw_params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_dma_buffer *dma_buf = &substream->dma_buffer;
	int ret = 0;

	pr_warn("mtk_i2s0_awb_pcm_hw_params\n");

	dma_buf->dev.type = SNDRV_DMA_TYPE_DEV;
	dma_buf->dev.dev = substream->pcm->card->dev;
	dma_buf->private_data = NULL;

	if (Awb_Capture_dma_buf->area) {
		pr_warn("mtk_i2s0_awb_pcm_hw_params Awb_Capture_dma_buf->area\n");
		runtime->dma_bytes = params_buffer_bytes(hw_params);
		runtime->dma_area = Awb_Capture_dma_buf->area;
		runtime->dma_addr = Awb_Capture_dma_buf->addr;
		SetHighAddr(Soc_Aud_Digital_Block_MEM_AWB, true, runtime->dma_addr);
	} else {
		pr_warn("mtk_i2s0_awb_pcm_hw_params snd_pcm_lib_malloc_pages\n");
		ret =  snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(hw_params));
	}
	pr_warn("mtk_i2s0_awb_pcm_hw_params dma_bytes = %zu dma_area = %p dma_addr = 0x%lx\n",
	       runtime->dma_bytes, runtime->dma_area, (long)runtime->dma_addr);

	pr_warn("runtime->hw.buffer_bytes_max = %zu\n", runtime->hw.buffer_bytes_max);
	set_mem_block(substream, hw_params,
		I2S0_AWB_Control_context, Soc_Aud_Digital_Block_MEM_AWB);

	AudDrv_Emi_Clk_On();

	pr_warn("dma_bytes = %zu dma_area = %p dma_addr = 0x%lx\n",
	       substream->runtime->dma_bytes, substream->runtime->dma_area,
	       (long)substream->runtime->dma_addr);
	return ret;
}

static int mtk_i2s0_capture_pcm_hw_free(struct snd_pcm_substream *substream)
{
	pr_warn("mtk_i2s0_capture_pcm_hw_free\n");

	AudDrv_Emi_Clk_Off();

	if (Awb_Capture_dma_buf->area)
		return 0;
	else
		return snd_pcm_lib_free_pages(substream);
}

static struct snd_pcm_hw_constraint_list dl1_awb_constraints_sample_rates = {
	.count = ARRAY_SIZE(soc_high_supported_sample_rates),
	.list = soc_high_supported_sample_rates,
};

static int mtk_i2s0_awb_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret = 0;

	pr_warn("mtk_i2s0_awb_pcm_open\n");
	I2S0_AWB_Control_context = Get_Mem_ControlT(Soc_Aud_Digital_Block_MEM_AWB);
	runtime->hw = mtk_I2S0_awb_hardware;
	memcpy((void *)(&(runtime->hw)), (void *)&mtk_I2S0_awb_hardware,
	       sizeof(struct snd_pcm_hardware));

	ret = snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
					 &dl1_awb_constraints_sample_rates);
	ret = snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);

	if (ret < 0)
		pr_warn("snd_pcm_hw_constraint_integer failed\n");

	/* here open audio clocks */
	AudDrv_Clk_On();

	/* print for hw pcm information */
	pr_warn("mtk_i2s0_awb_pcm_open runtime rate = %d channels = %d\n",
	       runtime->rate, runtime->channels);
	runtime->hw.info |= SNDRV_PCM_INFO_INTERLEAVED;
	runtime->hw.info |= SNDRV_PCM_INFO_NONINTERLEAVED;
	runtime->hw.info |= SNDRV_PCM_INFO_MMAP_VALID;

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		pr_warn("SNDRV_PCM_STREAM_CAPTURE\n");
	else
		return -1;

	if (ret < 0) {
		pr_err("mtk_i2s0_awb_pcm_close\n");
		mtk_i2s0_awb_pcm_close(substream);
		return ret;
	}
	pr_warn("mtk_i2s0_awb_pcm_open return\n");
	return 0;
}

static int mtk_i2s0_awb_pcm_close(struct snd_pcm_substream *substream)
{
	AudDrv_Clk_Off();
	return 0;
}

static int mtk_i2s0_awb_alsa_start(struct snd_pcm_substream *substream)
{
	pr_warn("mtk_i2s0_awb_alsa_start\n");
	SetMemifSubStream(Soc_Aud_Digital_Block_MEM_AWB, substream);
	StartAudioI2sInAWBHardware(substream);
	return 0;
}

static int mtk_i2s0_awb_pcm_trigger(struct snd_pcm_substream *substream,
				    int cmd)
{
	pr_warn("mtk_i2s0_awb_pcm_trigger cmd = %d\n", cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		return mtk_i2s0_awb_alsa_start(substream);
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		return mtk_i2s0_awb_alsa_stop(substream);
	}
	return -EINVAL;
}

static int mtk_i2s0_awb_pcm_copy(struct snd_pcm_substream *substream,
				 int channel, snd_pcm_uframes_t pos,
				 void __user *dst, snd_pcm_uframes_t count)
{
	return mtk_memblk_copy(substream, channel, pos, dst, count,
		I2S0_AWB_Control_context, Soc_Aud_Digital_Block_MEM_AWB);
}

static int mtk_capture_pcm_silence(struct snd_pcm_substream *substream,
				   int channel, snd_pcm_uframes_t pos,
				   snd_pcm_uframes_t count)
{
	pr_warn("dummy_pcm_silence\n");
	return 0; /* do nothing */
}


static void *dummy_page[2];

static struct page *mtk_i2s0_capture_pcm_page(struct snd_pcm_substream
					      *substream,
					      unsigned long offset)
{
	pr_warn("dummy_pcm_page\n");
	return virt_to_page(dummy_page[substream->stream]); /* the same page */
}


static struct snd_pcm_ops mtk_i2s0_awb_ops = {
	.open =     mtk_i2s0_awb_pcm_open,
	.close =    mtk_i2s0_awb_pcm_close,
	.ioctl =    snd_pcm_lib_ioctl,
	.hw_params =    mtk_i2s0_awb_pcm_hw_params,
	.hw_free =  mtk_i2s0_capture_pcm_hw_free,
	.prepare =  mtk_i2s0_awb_pcm_prepare,
	.trigger =  mtk_i2s0_awb_pcm_trigger,
	.pointer =  mtk_i2s0_awb_pcm_pointer,
	.copy =     mtk_i2s0_awb_pcm_copy,
	.silence =  mtk_capture_pcm_silence,
	.page =     mtk_i2s0_capture_pcm_page,
};

static struct snd_soc_platform_driver mtk_soc_platform = {
	.ops        = &mtk_i2s0_awb_ops,
	.pcm_new    = mtk_asoc_i2s0_awb_pcm_new,
	.probe      = mtk_i2s0_dl1_awb_probe,
};

static int mtk_i2s0_awb_probe(struct platform_device *pdev)
{
	pr_warn("mtk_i2s0_awb_probe\n");

	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(64);
	if (!pdev->dev.dma_mask)
		pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;

	if (pdev->dev.of_node)
		dev_set_name(&pdev->dev, "%s", MT_SOC_I2S0_AWB_PCM);

	pr_warn("%s: dev name %s\n", __func__, dev_name(&pdev->dev));
	return snd_soc_register_platform(&pdev->dev,
					 &mtk_soc_platform);
}

static int mtk_asoc_i2s0_awb_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	pr_warn("mtk_asoc_i2s0_awb_pcm_new\n");
	return 0;
}

static int mtk_i2s0_dl1_awb_probe(struct snd_soc_platform *platform)
{
	pr_warn("mtk_i2s0_dl1_awb_probe\n");
	AudDrv_Allocate_mem_Buffer(platform->dev, Soc_Aud_Digital_Block_MEM_AWB,
				   AWB_MAX_BUFFER_SIZE);
	Awb_Capture_dma_buf =  Get_Mem_Buffer(Soc_Aud_Digital_Block_MEM_AWB);
	return 0;
}

static int mtk_dl1_awb_remove(struct platform_device *pdev)
{
	pr_debug("%s\n", __func__);
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id mt_soc_pcm_i2s0_awb_of_ids[] = {
	{ .compatible = "mediatek,mt_soc_pcm_i2s0_awb", },
	{}
};
#endif

static struct platform_driver mtk_i2s0_awb_capture_driver = {
	.driver = {
		.name = MT_SOC_I2S0_AWB_PCM,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = mt_soc_pcm_i2s0_awb_of_ids,
#endif
	},
	.probe = mtk_i2s0_awb_probe,
	.remove = mtk_dl1_awb_remove,
};

#ifndef CONFIG_OF
static struct platform_device *soc_i2s0_awb_capture_dev;
#endif

static int __init mtk_soc_i2s0_awb_platform_init(void)
{
	int ret = 0;

	pr_warn("%s\n", __func__);
#ifndef CONFIG_OF
	soc_i2s0_awb_capture_dev = platform_device_alloc(MT_SOC_I2S0_AWB_PCM, -1);
	if (!soc_i2s0_awb_capture_dev)
		return -ENOMEM;

	ret = platform_device_add(soc_i2s0_awb_capture_dev);
	if (ret != 0) {
		platform_device_put(soc_i2s0_awb_capture_dev);
		return ret;
	}
#endif
	ret = platform_driver_register(&mtk_i2s0_awb_capture_driver);
	return ret;
}

static void __exit mtk_soc_i2s0_awb_platform_exit(void)
{
	pr_warn("%s\n", __func__);
	platform_driver_unregister(&mtk_i2s0_awb_capture_driver);
}

module_init(mtk_soc_i2s0_awb_platform_init);
module_exit(mtk_soc_i2s0_awb_platform_exit);

MODULE_DESCRIPTION("I2S0 AWB module platform driver");
MODULE_LICENSE("GPL");
