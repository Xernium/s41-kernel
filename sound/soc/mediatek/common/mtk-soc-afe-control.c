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
 *  mt_sco_afe_control.c
 *
 * Project:
 * --------
 *   MT6797  Audio Driver Kernel Function
 *
 * Description:
 * ------------
 *   Audio register
 *
 * Author:
 * -------
 * Chipeng Chang
 *
 *------------------------------------------------------------------------------
 *
 *******************************************************************************/

/*****************************************************************************
 *                     C O M P I L E R   F L A G S
 *****************************************************************************/

/*****************************************************************************
 *                E X T E R N A L   R E F E R E N C E S
 *****************************************************************************/

#include "mtk-auddrv-common.h"
#include "mtk-soc-pcm-common.h"
#include "mtk-auddrv-def.h"
#include "mtk-auddrv-afe.h"
#include "mtk-auddrv-ana.h"
#include "mtk-auddrv-clk.h"
#include "mtk-auddrv-kernel.h"
#include "mtk-soc-afe-control.h"
#include "mtk-soc-pcm-platform.h"
#include "mtk-soc-analog-type.h"
#include "mtk-soc-digital-type.h"
#include "mtk-auddrv-gpio.h"
#include "mtk-soc-afe-connection.h"

#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/jack.h>
/*
* #include <mt-plat/mt_boot.h>
* #include <mt-plat/mt_boot_common.h>
* #include <mt-plat/mt_lpae.h>
*/
#ifdef CONFIG_MTK_AUXADC_INTF
#include <mt-plat/mtk_auxadc_intf.h>
#else
#include <mt-plat/upmu_common.h>
#endif

#include <linux/ftrace.h>

#ifdef AUDIO_VCOREFS_SUPPORT
#include <mtk_vcorefs_manager.h>
#endif

static DEFINE_SPINLOCK(afe_control_lock);
static DEFINE_SPINLOCK(afe_sram_control_lock);
static DEFINE_SPINLOCK(afe_mem_blk_dl1_lock);
static DEFINE_SPINLOCK(afe_mem_blk_dl1_2_lock);
static DEFINE_SPINLOCK(afe_mem_blk_dl2_lock);
static DEFINE_SPINLOCK(afe_mem_blk_dl3_lock);
static DEFINE_SPINLOCK(afe_dl_abnormal_context_lock);
static DEFINE_SPINLOCK(afe_mem_blk_ul1_lock);
static DEFINE_SPINLOCK(afe_mem_blk_ul2_lock);
static DEFINE_SPINLOCK(afe_mem_blk_dai_lock);
static DEFINE_SPINLOCK(afe_mem_blk_moddai_lock);
static DEFINE_SPINLOCK(afe_mem_blk_awb_lock);
static DEFINE_SPINLOCK(auddrv_dl2_lock);
static DEFINE_SPINLOCK(auddrv_dl3_lock);
static unsigned long spinlock_flags[Soc_Aud_Digital_Block_MEM_HDMI+1];

static DEFINE_MUTEX(afe_control_mutex);

/* static  variable */
static bool AudioDaiBtStatus;
static bool AudioAdcI2SStatus;
static bool Audio2ndAdcI2SStatus;
static bool AudioMrgStatus;
static bool mAudioInit;
static bool mVOWStatus;
static AudioDigtalI2S *m2ndI2S;	/* input */
static AudioDigtalI2S *m2ndI2Sout;	/* output */
static bool mFMEnable;
static bool mOffloadEnable;
static bool mOffloadSWMode;

static AudioHdmi *mHDMIOutput;
static AudioMrgIf *mAudioMrg;
static AudioDigitalDAIBT *AudioDaiBt;

static AFE_MEM_CONTROL_T *AFE_Mem_Control_context[Soc_Aud_Digital_Block_NUM_OF_MEM_INTERFACE] = { NULL };
static struct snd_dma_buffer *Audio_dma_buf[Soc_Aud_Digital_Block_NUM_OF_MEM_INTERFACE] = { NULL };

static AudioMemIFAttribute *mAudioMEMIF[Soc_Aud_Digital_Block_NUM_OF_DIGITAL_BLOCK] = { NULL };

AFE_DL_ABNORMAL_CONTROL_T AFE_dL_Abnormal_context;

/* static AudioAfeRegCache mAudioRegCache; */
static AudioSramManager mAudioSramManager;

const size_t AudioInterruptLimiter = 100;
static int irqcount;
static int APLL1TunerCounter;
static int APLL2TunerCounter;

static Aud_Sram_Manager mAud_Sram_Manager;

static bool mExternalModemStatus;

static struct mtk_dai mtk_dais[Soc_Aud_Digital_Block_NUM_OF_DIGITAL_BLOCK];

static struct irq_manager irq_managers[Soc_Aud_IRQ_MCU_MODE_NUM];
static struct mtk_mem_blk_ops *s_mem_blk_ops;
static struct mtk_afe_platform_ops *s_afe_platform_ops;

#define IrqShortCounter  512

static bool ScreenState;

static uint32 LowLatencyDebug;


/*
 * Function Forward Declaration
  */

static irqreturn_t AudDrv_IRQ_handler(int irq, void *dev_id);
static void Clear_Mem_CopySize(Soc_Aud_Digital_Block MemBlock);
static kal_uint32 Get_Mem_MaxCopySize(Soc_Aud_Digital_Block MemBlock);

static uint32 GeneralSampleRateTransform(uint32 sampleRate);
static uint32 DAIMEMIFSampleRateTransform(uint32 sampleRate);
static uint32 ADDADLSampleRateTransform(uint32 sampleRate);
static uint32 ADDAULSampleRateTransform(uint32 sampleRate);
static uint32 MDSampleRateTransform(uint32 sampleRate);

/*
 *    function implementation
 */

static bool CheckSize(uint32 size)
{
	if (size == 0) {
		pr_debug("CheckSize size = 0\n");
		return true;
	}

	return false;
}

static void AfeGlobalVarInit(void)
{
	AudioDaiBtStatus = false;
	AudioAdcI2SStatus = false;
	Audio2ndAdcI2SStatus = false;
	AudioMrgStatus = false;
	mAudioInit = false;
	mVOWStatus = false;
	m2ndI2S = NULL;		/* input */
	m2ndI2Sout = NULL;	/* output */
	mFMEnable = false;
	mOffloadEnable = false;
	mOffloadSWMode = false;
	mHDMIOutput = NULL;
	mAudioMrg = NULL;
	AudioDaiBt = NULL;
	mExternalModemStatus = false;
	irqcount = 0;
}

void AfeControlMutexLock(void)
{
	mutex_lock(&afe_control_mutex);
}

void AfeControlMutexUnLock(void)
{
	mutex_unlock(&afe_control_mutex);
}

void AfeControlSramLock(void)
{
	spin_lock(&afe_sram_control_lock);
}

void AfeControlSramUnLock(void)
{
	spin_unlock(&afe_sram_control_lock);
}

unsigned int GetSramState(void)
{
	return mAudioSramManager.mMemoryState;
}

void SetSramState(unsigned int State)
{
	pr_debug("%s state= %d\n", __func__, State);
	mAudioSramManager.mMemoryState |= State;
}

void ClearSramState(unsigned int State)
{
	pr_debug("%s state= %d\n", __func__, State);
	mAudioSramManager.mMemoryState &= (~State);
}

unsigned int GetPLaybackSramFullSize(void)
{
	unsigned int Sramsize = AUDIO_SRAM_PLAYBACK_FULL_SIZE;

	if (AUDIO_SRAM_PLAYBACK_FULL_SIZE > AFE_INTERNAL_SRAM_SIZE)
		Sramsize = AFE_INTERNAL_SRAM_SIZE;

	return Sramsize;
}


unsigned int GetPLaybackSramPartial(void)
{
	unsigned int Sramsize = AUDIO_SRAM_PLAYBACK_PARTIAL_SIZE;

	return Sramsize;
}


unsigned int GetPLaybackDramSize(void)
{
	return AUDIO_DRAM_PLAYBACK_SIZE;
}


size_t GetCaptureSramSize(void)
{
	unsigned int Sramsize = AUDIO_SRAM_CAPTURE_SIZE;

	return Sramsize;
}


size_t GetCaptureDramSize(void)
{
	return AUDIO_DRAM_CAPTURE_SIZE;
}


void SetFMEnableFlag(bool bEnable)
{
	mFMEnable = bEnable;
}

void SetOffloadEnableFlag(bool bEnable)
{
	mOffloadEnable = bEnable;
}

void SetOffloadSWMode(bool bEnable)
{
	mOffloadSWMode = bEnable;
}

bool GetOffloadSWMode(void)
{
	return mOffloadSWMode;
}

bool ConditionEnterSuspend(void)
{
	if ((mFMEnable == true) ||
	    (mOffloadEnable == true) ||
	    (GetMemoryPathEnable(Soc_Aud_Digital_Block_ADDA_ANC) == true))
		return false;

	return true;
}


/* function get internal mode status. */
bool get_internalmd_status(void)
{
	bool ret = (get_voice_bt_status() ||
		    get_voice_usb_status() ||
		    get_voice_status() ||
		    get_voice_md2_status() ||
		    get_voice_md2_bt_status());
#ifdef _NON_COMMON_FEATURE_READY
		    get_voice_ultra_status();
#endif
	return (mExternalModemStatus == true) ? false : ret;
}



static void FillDatatoDlmemory(volatile unsigned int *memorypointer, unsigned int fillsize,
			       unsigned short value)
{
	int addr = 0;
	unsigned int tempvalue = value;

	tempvalue = tempvalue << 16;
	tempvalue |= value;

	/* set memory to DC value */
	for (addr = 0; addr < (fillsize >> 2); addr++) {
		*memorypointer = tempvalue;
		memorypointer++;
	}
}

static struct snd_dma_buffer *Dl1_Playback_dma_buf;

static void SetDL1BufferwithBuf(void)
{
	/* #define Dl1_MAX_BUFFER_SIZE     (48*1024) */
	AudDrv_Allocate_mem_Buffer(NULL, Soc_Aud_Digital_Block_MEM_DL1, Dl1_MAX_BUFFER_SIZE);
	Dl1_Playback_dma_buf = Get_Mem_Buffer(Soc_Aud_Digital_Block_MEM_DL1);

	set_memif_addr(Soc_Aud_Digital_Block_MEM_DL1,
		       Dl1_Playback_dma_buf->addr,
		       Dl1_MAX_BUFFER_SIZE);
}


void OpenAfeDigitaldl1(bool bEnable)
{
	volatile unsigned int *Sramdata;

	if (bEnable == true) {
		SetDL1BufferwithBuf();
		SetMemIfFetchFormatPerSample(Soc_Aud_Digital_Block_MEM_DL1, AFE_WLEN_16_BIT);
		SetMemIfFetchFormatPerSample(Soc_Aud_Digital_Block_MEM_DL2, AFE_WLEN_16_BIT);
		SetConnectionFormat(OUTPUT_DATA_FORMAT_16BIT, Soc_Aud_Digital_Block_I2S_OUT_DAC);
		SetSampleRate(Soc_Aud_Digital_Block_MEM_I2S, 44100);
		SetIntfConnection(Soc_Aud_InterCon_Connection,
				Soc_Aud_Digital_Block_MEM_DL1, Soc_Aud_Digital_Block_I2S_OUT_DAC);
		SetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_DL1, true);
		Sramdata = (unsigned int *)(Dl1_Playback_dma_buf->area);
		FillDatatoDlmemory(Sramdata, Dl1_Playback_dma_buf->bytes, 0);
		usleep_range(5 * 1000, 20 * 1000);

		if (GetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_OUT_DAC) == false) {
			SetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_OUT_DAC, true);
			SetI2SDacOut(44100, false, Soc_Aud_I2S_WLEN_WLEN_16BITS);
			SetI2SDacEnable(true);
		} else {
			SetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_OUT_DAC, true);
		}

		EnableAfe(true);
	} else {
		SetIntfConnection(Soc_Aud_InterCon_DisConnect,
				  Soc_Aud_Digital_Block_MEM_DL1,
				  Soc_Aud_Digital_Block_I2S_OUT_DAC);

		SetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_OUT_DAC, false);
		SetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_DL1, false);

		if (GetI2SDacEnable() == false)
			SetI2SDacEnable(false);

		EnableAfe(false);
	}
}


void SetExternalModemStatus(const bool bEnable)
{
	pr_debug("%s(), mExternalModemStatus : %d => %d\n", __func__, mExternalModemStatus,
		 bEnable);
	mExternalModemStatus = bEnable;
}

/*****************************************************************************
 * FUNCTION
 *  InitAfeControl ,ResetAfeControl
 *
 * DESCRIPTION
 *  afe init function
 *
 *****************************************************************************
 */
bool InitAfeControl(struct device *pDev)
{
	int i = 0;

	pr_debug("InitAfeControl\n");

	/* first time to init , reg init. */
	AfeGlobalVarInit();
	Auddrv_Reg_map(pDev);
	AudDrv_Clk_Global_Variable_Init();
	AudDrv_Clk_Power_On();
	AudDrv_Bus_Init();
	Auddrv_Read_Efuse_HPOffset();
	AfeControlMutexLock();

	/* allocate memory for pointers */
	if (mAudioInit == false) {
		mAudioInit = true;
		mAudioMrg = kzalloc(sizeof(AudioMrgIf), GFP_KERNEL);
		AudioDaiBt = kzalloc(sizeof(AudioDigitalDAIBT), GFP_KERNEL);
		m2ndI2S = kzalloc(sizeof(AudioDigtalI2S), GFP_KERNEL);
		m2ndI2Sout = kzalloc(sizeof(AudioDigtalI2S), GFP_KERNEL);
		mHDMIOutput = kzalloc(sizeof(AudioHdmi), GFP_KERNEL);

		for (i = 0; i < Soc_Aud_Digital_Block_NUM_OF_DIGITAL_BLOCK; i++)
			mAudioMEMIF[i] = kzalloc(sizeof(AudioMemIFAttribute), GFP_KERNEL);

		for (i = 0; i < Soc_Aud_Digital_Block_NUM_OF_MEM_INTERFACE; i++) {
			AFE_Mem_Control_context[i] = kzalloc(sizeof(AFE_MEM_CONTROL_T), GFP_KERNEL);
			AFE_Mem_Control_context[i]->substreamL = NULL;
			spin_lock_init(&AFE_Mem_Control_context[i]->substream_lock);
		}

		for (i = 0; i < Soc_Aud_Digital_Block_NUM_OF_MEM_INTERFACE; i++)
			Audio_dma_buf[i] = kzalloc(sizeof(Audio_dma_buf), GFP_KERNEL);
		memset((void *)&AFE_dL_Abnormal_context, 0, sizeof(AFE_DL_ABNORMAL_CONTROL_T));
		memset((void *)&mtk_dais, 0, sizeof(mtk_dais));
	}


	AudioDaiBtStatus = false;
	AudioAdcI2SStatus = false;
	Audio2ndAdcI2SStatus = false;
	AudioMrgStatus = false;
	InitSramManager(pDev, SramBlockSize);
	init_irq_manager();

	mAudioMrg->Mrg_I2S_SampleRate = SampleRateTransform(44100, Soc_Aud_Digital_Block_MRG_I2S_OUT);

#if 0
	for (i = AUDIO_APLL1_DIV0; i < AUDIO_APLL_DIV_NUM; i++)
		EnableI2SDivPower(i, false);
#else
	PowerDownAllI2SDiv();
#endif
	init_afe_ops();
	trigger_mtkaif_calibration();
	/* set APLL clock setting */
	AfeControlMutexUnLock();

	return true;
}


bool ResetAfeControl(void)
{
	int i = 0;

	pr_debug("ResetAfeControl\n");
	AfeControlMutexLock();
	mAudioInit = false;
	memset((void *)(mAudioMrg), 0, sizeof(AudioMrgIf));
	memset((void *)(AudioDaiBt), 0, sizeof(AudioDigitalDAIBT));

	for (i = 0; i < Soc_Aud_Digital_Block_NUM_OF_DIGITAL_BLOCK; i++)
		memset((void *)(mAudioMEMIF[i]), 0, sizeof(AudioMemIFAttribute));

	for (i = 0; i < Soc_Aud_Digital_Block_NUM_OF_MEM_INTERFACE; i++)
		memset((void *)(AFE_Mem_Control_context[i]), 0, sizeof(AFE_MEM_CONTROL_T));

	AfeControlMutexUnLock();

	return true;
}


/*****************************************************************************
 * FUNCTION
 *  Register_aud_irq
 *
 * DESCRIPTION
 *  IRQ handler
 *
 *****************************************************************************
 */
bool Register_Aud_Irq(void *dev, uint32 afe_irq_number)
{
	int ret;

#ifdef CONFIG_OF
	ret =
	    request_irq(afe_irq_number, AudDrv_IRQ_handler, IRQF_TRIGGER_LOW, "Afe_ISR_Handle",
			dev);
	if (ret)
		pr_debug("Register_Aud_Irq AFE IRQ register fail!!!\n");
#else
	pr_debug("%s dev name =%s\n", __func__, dev_name(dev));
	ret =
	    request_irq(MT6735_AFE_MCU_IRQ_LINE, AudDrv_IRQ_handler,
			IRQF_TRIGGER_LOW, "Afe_ISR_Handle", dev);
#endif
	return ret;
}


/*****************************************************************************
 * FUNCTION
 *  AudDrv_IRQ_handler / AudDrv_magic_tasklet
 *
 * DESCRIPTION
 *  IRQ handler
 *
 *****************************************************************************
 */
irqreturn_t AudDrv_IRQ_handler(int irq, void *dev_id)
{
	/* unsigned long flags; */
	kal_uint32 volatile u4RegValue;
	kal_uint32 volatile irq_mcu_en;
	uint32 irqIndex = 0;
	const struct Aud_RegBitsInfo *irqOnReg, *irqEnReg, *irqStatusReg, *irqMcuEnReg;

	u4RegValue = Afe_Get_Reg(AFE_IRQ_MCU_STATUS);
	u4RegValue &= AFE_IRQ_MASK;

	/* here is error handle , for interrupt is trigger but not status , clear all interrupt with bit 6 */
	if (u4RegValue == 0) {
		irqMcuEnReg = GetIRQPurposeReg(Soc_Aud_IRQ_MCU);
		irq_mcu_en = Afe_Get_Reg(irqMcuEnReg->reg);
		pr_warn("%s(), [AudioWarn] u4RegValue = 0x%x, irqcount = %d, AFE_IRQ_MCU_EN = 0x%x\n",
			__func__,
			u4RegValue,
			irqcount,
			irq_mcu_en);

		/* only clear IRQ which is sent to MCU */
		irq_mcu_en &= irqMcuEnReg->mask;
		Afe_Set_Reg(AFE_IRQ_MCU_CLR, irq_mcu_en, irq_mcu_en);
		irqcount++;

		if (irqcount > AudioInterruptLimiter) {
			for (irqIndex = 0; irqIndex < Soc_Aud_IRQ_MCU_MODE_NUM; irqIndex++) {
				if (GetIRQCtrlReg(irqIndex)->irqPurpose != Soc_Aud_IRQ_MCU)
					continue;

				irqEnReg = &GetIRQCtrlReg(irqIndex)->en;
				if (irq_mcu_en & (1 << irqEnReg->sbit)) {
					irqOnReg = &GetIRQCtrlReg(irqIndex)->on;
					Afe_Set_Reg(irqOnReg->reg,
						    0 << irqOnReg->sbit,
						    irqOnReg->mask << irqOnReg->sbit);
				}
			}
			irqcount = 0;
		}

		goto AudDrv_IRQ_handler_exit;
	}

	/* clear irq */
	Afe_Set_Reg(AFE_IRQ_MCU_CLR, u4RegValue, AFE_IRQ_MASK);

	/*call each IRQ handler function*/
	for (irqIndex = 0; irqIndex < Soc_Aud_IRQ_MCU_MODE_NUM; irqIndex++) {
		if (GetIRQCtrlReg(irqIndex)->irqPurpose != Soc_Aud_IRQ_MCU)
			continue;

		irqStatusReg = &GetIRQCtrlReg(irqIndex)->status;
		if (u4RegValue & (0x1 << irqStatusReg->sbit))
			RunIRQHandler(irqIndex);
	}

AudDrv_IRQ_handler_exit:
	return IRQ_HANDLED;
}

void EnableAPLLTunerbySampleRate(uint32 SampleRate)
{
	pr_debug("%s APLL1Counter = %d APLL2Counter = %d SampleRate = %d\n", __func__, APLL1TunerCounter,
		APLL2TunerCounter, SampleRate);

	if (GetApllbySampleRate(SampleRate) == Soc_Aud_APLL1) {
		APLL1TunerCounter++;
		if (APLL1TunerCounter == 1) {
			Afe_Set_Reg(AFE_APLL1_TUNER_CFG, 0x00000832, 0x0000FFF7);
			Afe_Set_Reg(AFE_APLL1_TUNER_CFG, 0x1, 0x1);
		}
	} else if (GetApllbySampleRate(SampleRate) == Soc_Aud_APLL2) {
		APLL2TunerCounter++;
		if (APLL2TunerCounter == 1) {
			Afe_Set_Reg(AFE_APLL2_TUNER_CFG, 0x00000634, 0x0000FFF7);
			Afe_Set_Reg(AFE_APLL2_TUNER_CFG, 0x1, 0x1);
		}
	}

}

void DisableAPLLTunerbySampleRate(uint32 SampleRate)
{
	pr_debug("%s APLL1Counter = %d APLL2Counter = %d SampleRate = %d\n", __func__, APLL1TunerCounter,
		APLL2TunerCounter, SampleRate);

	if (GetApllbySampleRate(SampleRate) == Soc_Aud_APLL1) {
		APLL1TunerCounter--;
		if (APLL1TunerCounter == 0)
			Afe_Set_Reg(AFE_APLL1_TUNER_CFG, 0x0, 0x1);
	} else if (GetApllbySampleRate(SampleRate) == Soc_Aud_APLL2) {
		APLL2TunerCounter--;
		if (APLL2TunerCounter == 0)
			Afe_Set_Reg(AFE_APLL2_TUNER_CFG, 0x0, 0x1);
	}

}

static bool CheckMemIfEnable(void)
{
	int i = 0;

	for (i = 0; i < Soc_Aud_Digital_Block_NUM_OF_DIGITAL_BLOCK; i++) {
		if ((mAudioMEMIF[i]->mState) == true) {
			/* printk("CheckMemIfEnable == true\n"); */
			return true;
		}
	}

	/* printk("CheckMemIfEnable == false\n"); */
	return false;
}


/* record VOW status for AFE GPIO control */
void SetVOWStatus(bool bEnable)
{
	unsigned long flags;

	if (mVOWStatus != bEnable) {
		spin_lock_irqsave(&afe_control_lock, flags);
		mVOWStatus = bEnable;
		pr_debug("SetVOWStatus, mVOWStatus= %d\n", mVOWStatus);
		spin_unlock_irqrestore(&afe_control_lock, flags);
	}
}


/*****************************************************************************
 * FUNCTION
 *  Auddrv_Reg_map
 *
 * DESCRIPTION
 * Auddrv_Reg_map
 *
 *****************************************************************************
 */
static bool afe_on;
void EnableAfe(bool bEnable)
{
	unsigned long flags;
	bool MemEnable = false;

	spin_lock_irqsave(&afe_control_lock, flags);
	MemEnable = CheckMemIfEnable();

	if (false == bEnable && false == MemEnable) {
		if (afe_on && mtk_soc_always_hd) {
			DisableAPLLTunerbySampleRate(44100);
			DisableAPLLTunerbySampleRate(48000);
		}

		set_chip_afe_enable(false);

		if (afe_on && mtk_soc_always_hd) {
			DisableALLbySampleRate(44100);
			DisableALLbySampleRate(48000);
		}

		afe_on = false;
	} else if (true == bEnable && true == MemEnable) {
		if (!afe_on && mtk_soc_always_hd) {
			EnableALLbySampleRate(44100);
			EnableALLbySampleRate(48000);
		}

		set_chip_afe_enable(true);

		if (!afe_on && mtk_soc_always_hd) {
			EnableAPLLTunerbySampleRate(44100);
			EnableAPLLTunerbySampleRate(48000);
		}

		afe_on = true;
	}

	spin_unlock_irqrestore(&afe_control_lock, flags);
}


uint32 SampleRateTransform(uint32 sampleRate, Soc_Aud_Digital_Block audBlock)
{
	switch (audBlock) {
	case Soc_Aud_Digital_Block_ADDA_DL:
		return ADDADLSampleRateTransform(sampleRate);
	case Soc_Aud_Digital_Block_ADDA_UL:
		return ADDAULSampleRateTransform(sampleRate);
	case Soc_Aud_Digital_Block_MEM_MOD_DAI:
	case Soc_Aud_Digital_Block_MEM_DAI:
		return DAIMEMIFSampleRateTransform(sampleRate);
	case Soc_Aud_Digital_Block_MODEM_PCM_1_O:
	case Soc_Aud_Digital_Block_MODEM_PCM_2_O:
		return MDSampleRateTransform(sampleRate);
	default:
		return GeneralSampleRateTransform(sampleRate);
	}
}

uint32 GeneralSampleRateTransform(uint32 sampleRate)
{
	switch (sampleRate) {
	case 8000:
		return Soc_Aud_I2S_SAMPLERATE_I2S_8K;
	case 11025:
		return Soc_Aud_I2S_SAMPLERATE_I2S_11K;
	case 12000:
		return Soc_Aud_I2S_SAMPLERATE_I2S_12K;
	case 16000:
		return Soc_Aud_I2S_SAMPLERATE_I2S_16K;
	case 22050:
		return Soc_Aud_I2S_SAMPLERATE_I2S_22K;
	case 24000:
		return Soc_Aud_I2S_SAMPLERATE_I2S_24K;
	case 32000:
		return Soc_Aud_I2S_SAMPLERATE_I2S_32K;
	case 44100:
		return Soc_Aud_I2S_SAMPLERATE_I2S_44K;
	case 48000:
		return Soc_Aud_I2S_SAMPLERATE_I2S_48K;
	case 88200:
		return Soc_Aud_I2S_SAMPLERATE_I2S_88K;
	case 96000:
		return Soc_Aud_I2S_SAMPLERATE_I2S_96K;
	case 130000:
		return Soc_Aud_I2S_SAMPLERATE_I2S_130K;
	case 176400:
		return Soc_Aud_I2S_SAMPLERATE_I2S_174K;
	case 192000:
		return Soc_Aud_I2S_SAMPLERATE_I2S_192K;
	case 260000:
		return Soc_Aud_I2S_SAMPLERATE_I2S_260K;
	default:
		pr_warn("[AudioWarn] %s() sampleRate(%d) is invalid, use 44.1kHz!!!\n", __func__,
			sampleRate);
		return Soc_Aud_I2S_SAMPLERATE_I2S_44K;
	}
}

uint32 DAIMEMIFSampleRateTransform(uint32 sampleRate)
{
	switch (sampleRate) {
	case 8000:
		return Soc_Aud_DAI_MEMIF_SAMPLERATE_8K;
	case 16000:
		return Soc_Aud_DAI_MEMIF_SAMPLERATE_16K;
	case 32000:
		return Soc_Aud_DAI_MEMIF_SAMPLERATE_32K;
	default:
		pr_warn("[AudioWarn] %s() sampleRate(%d) is invalid, use 16kHz!!!\n", __func__,
			sampleRate);
		return Soc_Aud_DAI_MEMIF_SAMPLERATE_16K;
	}
}


uint32 ADDADLSampleRateTransform(uint32 sampleRate)
{
	switch (sampleRate) {
	case 8000:
		return Soc_Aud_ADDA_DL_SAMPLERATE_8K;
	case 11025:
		return Soc_Aud_ADDA_DL_SAMPLERATE_11K;
	case 12000:
		return Soc_Aud_ADDA_DL_SAMPLERATE_12K;
	case 16000:
		return Soc_Aud_ADDA_DL_SAMPLERATE_16K;
	case 22050:
		return Soc_Aud_ADDA_DL_SAMPLERATE_22K;
	case 24000:
		return Soc_Aud_ADDA_DL_SAMPLERATE_24K;
	case 32000:
		return Soc_Aud_ADDA_DL_SAMPLERATE_32K;
	case 44100:
		return Soc_Aud_ADDA_DL_SAMPLERATE_44K;
	case 48000:
		return Soc_Aud_ADDA_DL_SAMPLERATE_48K;
	case 96000:
		return Soc_Aud_ADDA_DL_SAMPLERATE_96K;
	case 192000:
		return Soc_Aud_ADDA_DL_SAMPLERATE_192K;
	default:
		pr_warn("[AudioWarn] %s() sampleRate(%d) is invalid, use 44.1kHz!!!\n", __func__,
			sampleRate);
		return Soc_Aud_ADDA_DL_SAMPLERATE_44K;
	}
}

uint32 ADDAULSampleRateTransform(uint32 sampleRate)
{
	switch (sampleRate) {
	case 8000:
		return Soc_Aud_ADDA_UL_SAMPLERATE_8K;
	case 16000:
		return Soc_Aud_ADDA_UL_SAMPLERATE_16K;
	case 32000:
		return Soc_Aud_ADDA_UL_SAMPLERATE_32K;
	case 48000:
		return Soc_Aud_ADDA_UL_SAMPLERATE_48K;
	case 96000:
		return Soc_Aud_ADDA_UL_SAMPLERATE_96K;
	case 192000:
		return Soc_Aud_ADDA_UL_SAMPLERATE_192K;
	default:
		pr_warn("[AudioWarn] %s() sampleRate(%d) is invalid, use 48kHz(24bit)!!!\n",
			__func__, sampleRate);
		return Soc_Aud_ADDA_UL_SAMPLERATE_48K;
	}
}

uint32 MDSampleRateTransform(uint32 sampleRate)
{
	switch (sampleRate) {
	case 8000:
		return Soc_Aud_PCM_MODE_PCM_MODE_8K;
	case 16000:
		return Soc_Aud_PCM_MODE_PCM_MODE_16K;
	case 32000:
		return Soc_Aud_PCM_MODE_PCM_MODE_32K;
	case 48000:
		return Soc_Aud_PCM_MODE_PCM_MODE_48K;
	default:
		pr_err("%s(), rate %u not support, use 16k\n",
		       __func__, sampleRate);
		return Soc_Aud_PCM_MODE_PCM_MODE_16K;
	}
}

bool Set2ndI2SOutAttribute(uint32_t sampleRate)
{
	pr_debug("+%s(), sampleRate = %d\n", __func__, sampleRate);
	m2ndI2Sout->mLR_SWAP = Soc_Aud_LR_SWAP_NO_SWAP;
	m2ndI2Sout->mI2S_SLAVE = Soc_Aud_I2S_SRC_MASTER_MODE;
	m2ndI2Sout->mINV_LRCK = Soc_Aud_INV_LRCK_NO_INVERSE;
	m2ndI2Sout->mI2S_FMT = Soc_Aud_I2S_FORMAT_I2S;
	m2ndI2Sout->mI2S_WLEN = Soc_Aud_I2S_WLEN_WLEN_16BITS;
	m2ndI2Sout->mI2S_HDEN = Soc_Aud_NORMAL_CLOCK;
	m2ndI2Sout->mI2S_SAMPLERATE = sampleRate;
	Set2ndI2SOut(m2ndI2Sout);
	return true;
}


bool Set2ndI2SOut(AudioDigtalI2S *DigtalI2S)
{
	uint32 u32AudioI2S = 0;

	memcpy((void *)m2ndI2Sout, (void *)DigtalI2S, sizeof(AudioDigtalI2S));
	u32AudioI2S = SampleRateTransform(m2ndI2Sout->mI2S_SAMPLERATE, Soc_Aud_Digital_Block_I2S_OUT_2) << 8;
	u32AudioI2S |= m2ndI2Sout->mLR_SWAP << 31;
	u32AudioI2S |= m2ndI2Sout->mI2S_HDEN << 12;
	u32AudioI2S |= m2ndI2Sout->mINV_LRCK << 5;
	u32AudioI2S |= m2ndI2Sout->mI2S_FMT << 3;
	u32AudioI2S |= m2ndI2Sout->mI2S_WLEN << 1;
	Afe_Set_Reg(AFE_I2S_CON3, u32AudioI2S, AFE_MASK_ALL);

	return true;
}


bool Set2ndI2SOutEnable(bool benable)
{
	if (benable)
		Afe_Set_Reg(AFE_I2S_CON3, 0x1, 0x1);
	else
		Afe_Set_Reg(AFE_I2S_CON3, 0x0, 0x1);

	return true;
}


bool SetDaiBt(AudioDigitalDAIBT *mAudioDaiBt)
{
	AudioDaiBt->mBT_LEN = mAudioDaiBt->mBT_LEN;
	AudioDaiBt->mUSE_MRGIF_INPUT = mAudioDaiBt->mUSE_MRGIF_INPUT;
	AudioDaiBt->mDAI_BT_MODE = mAudioDaiBt->mDAI_BT_MODE;
	AudioDaiBt->mDAI_DEL = mAudioDaiBt->mDAI_DEL;
	AudioDaiBt->mBT_LEN = mAudioDaiBt->mBT_LEN;
	AudioDaiBt->mDATA_RDY = mAudioDaiBt->mDATA_RDY;
	AudioDaiBt->mBT_SYNC = mAudioDaiBt->mBT_SYNC;

	return true;
}

bool SetDaiBtEnable(bool bEanble)
{
	pr_debug("%s bEanble = %d\n", __func__, bEanble);
	return set_chip_dai_bt_enable(bEanble, AudioDaiBt, mAudioMrg);
}


bool GetMrgI2SEnable(void)
{
	return mAudioMEMIF[Soc_Aud_Digital_Block_MRG_I2S_OUT]->mState;
}


bool SetMrgI2SEnable(bool bEnable, unsigned int sampleRate)
{
	uint32 sampleRateType;

	sampleRateType = SampleRateTransform(sampleRate, Soc_Aud_Digital_Block_MRG_I2S_OUT);

	pr_warn("%s bEnable = %d\n", __func__, bEnable);

	if (bEnable == true) {
		/* To enable MrgI2S */
		if (mAudioMrg->MrgIf_En == true) {
			/* Merge Interface already turn on. */
			/* if sample Rate change, then it need to restart with new setting; else do nothing. */
			if (mAudioMrg->Mrg_I2S_SampleRate != sampleRateType) {
				/* Turn off Merge Interface first to switch I2S sampling rate */
				Afe_Set_Reg(AFE_MRGIF_CON, 0, 1 << 16);	/* Turn off I2S */
				if (AudioDaiBt->mDAIBT_ON == true)
					Afe_Set_Reg(AFE_DAIBT_CON0, 0, 0x1);	/* Turn off DAIBT first */
				udelay(100);
				Afe_Set_Reg(AFE_MRGIF_CON, 0, 0x1);	/* Turn off Merge Interface */
				udelay(100);
				Afe_Set_Reg(AFE_MRGIF_CON, 1, 0x1);	/* Turn on Merge Interface */
				if (AudioDaiBt->mDAIBT_ON == true) {
					Afe_Set_Reg(AFE_DAIBT_CON0, AudioDaiBt->mDAI_BT_MODE << 9,
						    0x1 << 9);
					/* use merge */
					Afe_Set_Reg(AFE_DAIBT_CON0, 0x1 << 12, 0x1 << 12);	/* use merge */
					Afe_Set_Reg(AFE_DAIBT_CON0, 0x1 << 3, 0x1 << 3);	/* data ready */
					Afe_Set_Reg(AFE_DAIBT_CON0, 0x3, 0x3);	/* Turn on DAIBT */
				}
				mAudioMrg->Mrg_I2S_SampleRate = sampleRateType;
				Afe_Set_Reg(AFE_MRGIF_CON, mAudioMrg->Mrg_I2S_SampleRate << 20,
					    0xF00000);
				/* set Mrg_I2S Samping Rate */
				Afe_Set_Reg(AFE_MRGIF_CON, 1 << 16, 1 << 16);	/* set Mrg_I2S enable */
			}
		} else {
			/* turn on merge Interface from off state */
			mAudioMrg->Mrg_I2S_SampleRate = sampleRateType;
			Afe_Set_Reg(AFE_MRGIF_CON, mAudioMrg->Mrg_I2S_SampleRate << 20, 0xF << 20);

			/* set Mrg_I2S Samping rates */
			Afe_Set_Reg(AFE_MRGIF_CON, 1 << 16, 1 << 16);	/* set Mrg_I2S enable */
			udelay(100);
			Afe_Set_Reg(AFE_MRGIF_CON, 1, 0x1);	/* Turn on Merge Interface */
			udelay(100);

			if (AudioDaiBt->mDAIBT_ON == true) {
				Afe_Set_Reg(AFE_DAIBT_CON0, AudioDaiBt->mDAI_BT_MODE << 9,
					    0x1 << 9);

				/* use merge */
				Afe_Set_Reg(AFE_DAIBT_CON0, 0x1 << 12, 0x1 << 12);	/* use merge */
				Afe_Set_Reg(AFE_DAIBT_CON0, 0x1 << 3, 0x1 << 3);	/* data ready */
				Afe_Set_Reg(AFE_DAIBT_CON0, 0x3, 0x3);	/* Turn on DAIBT */
			}
		}
		mAudioMrg->MrgIf_En = true;
		mAudioMrg->Mergeif_I2S_Enable = true;
	} else {
		if (mAudioMrg->MrgIf_En == true) {
			Afe_Set_Reg(AFE_MRGIF_CON, 0, 1 << 16);	/* Turn off I2S */
			if (AudioDaiBt->mDAIBT_ON == false) {
				udelay(100);
				/* DAIBT also not using, then it's OK to disable Merge Interface */
				Afe_Set_Reg(AFE_MRGIF_CON, 0, 0x1);	/* Turn off Merge Interface */
				mAudioMrg->MrgIf_En = false;
			}
		}
		mAudioMrg->Mergeif_I2S_Enable = false;
	}

	return true;
}


bool Set2ndI2SAdcIn(AudioDigtalI2S *DigtalI2S)
{
	/* 6752 todo? */
	return true;
}

bool SetExtI2SAdcIn(AudioDigtalI2S *DigtalI2S)
{
	uint32 Audio_I2S_Adc = 0;
	uint32 sampleRateType;

	sampleRateType =
		    SampleRateTransform(DigtalI2S->mI2S_SAMPLERATE, Soc_Aud_Digital_Block_I2S_IN_ADC);

	/* Set I2S_ADC_IN */
	Audio_I2S_Adc |= (DigtalI2S->mLR_SWAP << 31);
	Audio_I2S_Adc |= (DigtalI2S->mBuffer_Update_word << 24);
	Audio_I2S_Adc |= (DigtalI2S->mINV_LRCK << 23);
	Audio_I2S_Adc |= (DigtalI2S->mFpga_bit_test << 22);
	Audio_I2S_Adc |= (DigtalI2S->mFpga_bit << 21);
	Audio_I2S_Adc |= (DigtalI2S->mloopback << 20);
	Audio_I2S_Adc |= (sampleRateType << 8);
	Audio_I2S_Adc |= (DigtalI2S->mI2S_FMT << 3);
	Audio_I2S_Adc |= (DigtalI2S->mI2S_WLEN << 1);
	pr_debug("%s Audio_I2S_Adc = 0x%x", __func__, Audio_I2S_Adc);
	Afe_Set_Reg(AFE_I2S_CON2, Audio_I2S_Adc, MASK_ALL);

	return true;
}

bool SetExtI2SAdcInEnable(bool bEnable)
{
	Afe_Set_Reg(AFE_I2S_CON2, bEnable, 0x1);
	return true;
}

bool SetI2SAdcIn(AudioDigtalI2S *DigtalI2S)
{
	mtk_dais[Soc_Aud_Digital_Block_ADDA_UL].sample_rate =
		DigtalI2S->mI2S_SAMPLERATE;

	return SetChipI2SAdcIn(DigtalI2S, AudioAdcI2SStatus);
}

#ifdef AFE_CONNSYS_I2S_CON
int setConnsysI2SIn(AudioDigtalI2S *mDigitalI2S)
{
	unsigned int i2s_con = 0;

	/* slave mode, rate is for asrc */
	i2s_con |= (mDigitalI2S->mINV_LRCK << 7);
	i2s_con |= (mDigitalI2S->mI2S_FMT << 3);
	i2s_con |= (mDigitalI2S->mI2S_SLAVE << 2);
	i2s_con |= (mDigitalI2S->mI2S_WLEN << 1);
	i2s_con |= (mDigitalI2S->mI2S_IN_PAD_SEL << 28);
	pr_debug("%s(), i2s_con= 0x%x", __func__, i2s_con);
	Afe_Set_Reg(AFE_CONNSYS_I2S_CON, i2s_con, 0xfffffffe);

	return 0;
}

int setConnsysI2SInEnable(bool enable)
{
	pr_warn("%s(), enable = %d", __func__, enable);
	Afe_Set_Reg(AFE_CONNSYS_I2S_CON, enable, 0x1);

	return 0;
}

int setConnsysI2SAsrc(bool bIsUseASRC, unsigned int dToSampleRate)
{
	unsigned int rate = SampleRateTransform(dToSampleRate,
						Soc_Aud_Digital_Block_I2S_IN_CONNSYS);

	pr_debug("+%s() bIsUseASRC [%d] dToSampleRate [%d]\n",
		 __func__,
		 bIsUseASRC,
		 dToSampleRate);

	Afe_Set_Reg(AFE_CONNSYS_I2S_CON, (bIsUseASRC ? 0 : 1) << 6, 0x1 << 6);

	if (bIsUseASRC) {
		AUDIO_ASSERT(!(dToSampleRate == 44100 || dToSampleRate == 48000));

		/* slave mode, set i2s for asrc */
		Afe_Set_Reg(AFE_CONNSYS_I2S_CON, rate << 8, 0xf << 8);

		if (dToSampleRate == 44100)
			Afe_Set_Reg(AFE_ASRC_CONNSYS_CON14, 0x001B9000, AFE_MASK_ALL);
		 else
			Afe_Set_Reg(AFE_ASRC_CONNSYS_CON14, 0x001E0000, AFE_MASK_ALL);

		Afe_Set_Reg(AFE_ASRC_CONNSYS_CON15, 0x00140000, AFE_MASK_ALL);
		Afe_Set_Reg(AFE_ASRC_CONNSYS_CON16, 0x00FF5987, AFE_MASK_ALL);
		Afe_Set_Reg(AFE_ASRC_CONNSYS_CON17, 0x00007EF4, AFE_MASK_ALL);
		Afe_Set_Reg(AFE_ASRC_CONNSYS_CON16, 0x00FF5986, AFE_MASK_ALL);
		Afe_Set_Reg(AFE_ASRC_CONNSYS_CON16, 0x00FF5987, AFE_MASK_ALL);

		/* 0:Stereo 1:Mono */
		Afe_Set_Reg(AFE_ASRC_CONNSYS_CON13, 0, 1 << 16);

		/* Calibration setting */
		Afe_Set_Reg(AFE_ASRC_CONNSYS_CON20, 0x00036000, AFE_MASK_ALL);
		Afe_Set_Reg(AFE_ASRC_CONNSYS_CON21, 0x0002FC00, AFE_MASK_ALL);
	}

	return 0;
}

int setConnsysI2SEnable(bool enable)
{
	if (enable) {
		Afe_Set_Reg(AFE_ASRC_CONNSYS_CON0, ((1 << 6) | (1 << 0)), ((1 << 6) | (1 << 0)));
	} else {
		uint32 dNeedDisableASM = (Afe_Get_Reg(AFE_ASRC_CONNSYS_CON0) & 0x0030) ? 1 : 0;

		Afe_Set_Reg(AFE_ASRC_CONNSYS_CON0, 0, (1 << 6 | dNeedDisableASM));
	}

	return 0;
}
#else

int setConnsysI2SIn(AudioDigtalI2S *DigtalI2S)
{
	return -EINVAL;
}

int setConnsysI2SInEnable(bool enable)
{
	return -EINVAL;
}

int setConnsysI2SAsrc(bool bIsUseASRC, unsigned int dToSampleRate)
{
	return -EINVAL;
}

int setConnsysI2SEnable(bool enable)
{
	return -EINVAL;
}
#endif

bool setDmicPath(bool _enable)
{
	uint32 sample_rate =
		mtk_dais[Soc_Aud_Digital_Block_ADDA_UL].sample_rate;

	return setChipDmicPath(_enable, sample_rate);
}

bool EnableSineGen(uint32 connection, bool direction, bool Enable)
{
	bool ret = 0;

	pr_debug("+%s(): connection = %d, direction = %d, Enable= %d\n", __func__, connection,
		 direction, Enable);

	/* by platform to implement*/
	if (s_afe_platform_ops != NULL) {
		ret = s_afe_platform_ops->set_sinegen(connection, direction, Enable, mAudioMEMIF);
	} else {
		pr_err("+%s(): No sine gen implementation, return false!\n", __func__);
		ret = false;
	}

	return ret;
}


bool SetSineGenSampleRate(uint32 SampleRate)
{
	return set_chip_sine_gen_sample_rate(SampleRate);
}

bool SetSineGenAmplitude(uint32 ampDivide)
{
	return set_chip_sine_gen_amplitude(ampDivide);
}

bool Set2ndI2SAdcEnable(bool bEnable)
{
	/* 6752 todo? */
	return true;
}


bool SetI2SAdcEnable(bool bEnable)
{
	mAudioMEMIF[Soc_Aud_Digital_Block_I2S_IN_ADC]->mState = bEnable;

	if (bEnable) {
		/* Enable UL SRC order:
		* UL clock (AUDIO_TOP_CON0) -> AFE (AFE_DAC_CON0) ->
		* ADDA UL DL (AFE_ADDA_UL_DL_CON0) ->
		* ADDA UL SRC (AFE_ADDA_UL_SRC_CON0)
		*/
		EnableAfe(true);
		SetADDAEnable(true);
		SetULSrcEnable(true);
	} else {
		/* Disable UL SRC order: (reverse)
		* ADDA UL SRC (AFE_ADDA_UL_SRC_CON0) ->
		* ADDA UL DL (AFE_ADDA_UL_DL_CON0) ->
		* AFE (AFE_DAC_CON0) -> UL clock (AUDIO_TOP_CON0)
		*/
		SetULSrcEnable(false);
		SetADDAEnable(false);
		if (mtk_dais[Soc_Aud_Digital_Block_ADDA_UL].sample_rate > 48000) {
			/* power on adc hires */
			AudDrv_ADC_Hires_Clk_Off();
		}
#ifdef CONFIG_FPGA_EARLY_PORTING
		pr_warn("%s(), disable fpga clock divide by 4", __func__);
		Afe_Set_Reg(FPGA_CFG0, 0x0 << 1, 0x1 << 1);
#endif
	}

	AudDrv_GPIO_Request(bEnable, Soc_Aud_Digital_Block_ADDA_UL);

	return true;
}


bool Set2ndI2SEnable(bool bEnable)
{
	Afe_Set_Reg(AFE_I2S_CON, bEnable, 0x1);

	return true;
}

bool SetI2SDacOut(uint32 SampleRate, bool lowjitter, bool I2SWLen)
{
	uint32 Audio_I2S_Dac = 0;

	/* force use 32bit for speaker codec */
	I2SWLen = Soc_Aud_I2S_WLEN_WLEN_32BITS;

	pr_aud("SetI2SDacOut SampleRate %d, lowjitter %d, I2SWLen %d\n", SampleRate, lowjitter,
		I2SWLen);
	CleanPreDistortion();
	SetDLSrc2(SampleRate);

	Audio_I2S_Dac |= (Soc_Aud_LR_SWAP_NO_SWAP << 31);
	Audio_I2S_Dac |= (1 << 16);				/* select source from o3o4 */
	Audio_I2S_Dac |= (lowjitter << 12);			/* low gitter mode */
	Audio_I2S_Dac |= (SampleRateTransform(SampleRate, Soc_Aud_Digital_Block_I2S_OUT_DAC) << 8);
	Audio_I2S_Dac |= (Soc_Aud_INV_LRCK_NO_INVERSE << 5);
	Audio_I2S_Dac |= (Soc_Aud_I2S_FORMAT_I2S << 3);
	Audio_I2S_Dac |= (I2SWLen << 1);
	Afe_Set_Reg(AFE_I2S_CON1, Audio_I2S_Dac, MASK_ALL);

	return true;
}


bool SetHwDigitalGainMode(uint32 GainType, uint32 SampleRate, uint32 SamplePerStep)
{
	/*
	  * printk("SetHwDigitalGainMode GainType = %d, SampleRate = %d,
	  * SamplePerStep= %d\n", GainType, SampleRate, SamplePerStep);
	  */
	return set_chip_hw_digital_gain_mode(GainType, SampleRate, SamplePerStep);
}

bool SetHwDigitalGainEnable(int GainType, bool Enable)
{
	pr_debug("+%s(), GainType = %d, Enable = %d\n", __func__, GainType, Enable);
	return set_chip_hw_digital_gain_enable(GainType, Enable);
}


bool SetHwDigitalGain(uint32 Gain, int GainType)
{
	pr_debug("+%s(), Gain = 0x%x, gain type = %d\n", __func__, Gain, GainType);
	return set_chip_hw_digital_gain(Gain, GainType);
}


bool SetModemPcmConfig(int modem_index, AudioDigitalPCM p_modem_pcm_attribute)
{
	pr_debug("+%s()\n", __func__);
	SetChipModemPcmConfig(modem_index, p_modem_pcm_attribute);
	return true;
}


bool SetModemPcmEnable(int modem_index, bool modem_pcm_on)
{
	bool ret;

	pr_debug("+%s(), modem_index = %d, modem_pcm_on = %d\n", __func__, modem_index,
		 modem_pcm_on);

	ret = SetChipModemPcmEnable(modem_index, modem_pcm_on);

	if (modem_index == MODEM_1)
		mAudioMEMIF[Soc_Aud_Digital_Block_MODEM_PCM_1_O]->mState = modem_pcm_on;
	else if (modem_index == MODEM_2 || modem_index == MODEM_EXTERNAL)
		mAudioMEMIF[Soc_Aud_Digital_Block_MODEM_PCM_2_O]->mState = modem_pcm_on;
	else
		pr_err("%s(), no such modem_index: %d!!", __func__, modem_index);

	return ret;
}

bool SetMemoryPathEnableReg(uint32 Aud_block, bool bEnable)
{
	uint32 offset = GetEnableAudioBlockRegOffset(Aud_block);
	uint32 reg = GetEnableAudioBlockRegAddr(Aud_block);

	if (reg == 0) {
		pr_err("%s(), no such memory path enable bit!!", __func__);
		return false;
	}
	Afe_Set_Reg(reg, bEnable << offset, 1 << offset);
	return true;
}

bool SetMemoryPathEnable(uint32 Aud_block, bool bEnable)
{
	pr_aud("%s Aud_block = %d bEnable = %d\n", __func__, Aud_block, bEnable);
	if (Aud_block >= Soc_Aud_Digital_Block_NUM_OF_DIGITAL_BLOCK)
		return false;

	/* set for counter */
	if (bEnable == true) {
		if (mAudioMEMIF[Aud_block]->mUserCount == 0)
			mAudioMEMIF[Aud_block]->mState = true;
		mAudioMEMIF[Aud_block]->mUserCount++;
	} else {
		mAudioMEMIF[Aud_block]->mUserCount--;
		if (mAudioMEMIF[Aud_block]->mUserCount == 0)
			mAudioMEMIF[Aud_block]->mState = false;
		if (mAudioMEMIF[Aud_block]->mUserCount < 0) {
			mAudioMEMIF[Aud_block]->mUserCount = 0;
			pr_err("[AudioError] , user count < 0\n");
		}
	}
	pr_aud("%s Aud_block = %d mAudioMEMIF[Aud_block]->mUserCount = %d\n", __func__, Aud_block,
		mAudioMEMIF[Aud_block]->mUserCount);

	if (Aud_block >= Soc_Aud_Digital_Block_NUM_OF_MEM_INTERFACE)
		return true;
	/*Let DSP enable DL3*/
	if (Aud_block == Soc_Aud_Digital_Block_MEM_DL3)
		return true;

	if ((bEnable == true) && (mAudioMEMIF[Aud_block]->mUserCount == 1))
		SetMemoryPathEnableReg(Aud_block, bEnable);
	else if ((bEnable == false) && (mAudioMEMIF[Aud_block]->mUserCount == 0))
		SetMemoryPathEnableReg(Aud_block, bEnable);

	return true;
}

bool GetMemoryPathEnable(uint32 Aud_block)
{
	if (Aud_block < Soc_Aud_Digital_Block_NUM_OF_DIGITAL_BLOCK)
		return mAudioMEMIF[Aud_block]->mState;

	return false;
}

void SetULSrcEnable(bool bEnable)
{
	unsigned long flags;

	pr_debug("%s bEnable = %d\n", __func__, bEnable);

	spin_lock_irqsave(&afe_control_lock, flags);
	if (bEnable == true) {
		set_chip_ul_src_enable(true);
	} else {
		if (mAudioMEMIF[Soc_Aud_Digital_Block_I2S_IN_ADC]->mState == false &&
				mAudioMEMIF[Soc_Aud_Digital_Block_I2S_IN_ADC_2]->mState == false &&
				mAudioMEMIF[Soc_Aud_Digital_Block_ADDA_ANC]->mState == false) {
			set_chip_ul_src_enable(false);
		}
	}
	spin_unlock_irqrestore(&afe_control_lock, flags);
}

void SetDLSrcEnable(bool bEnable)
{
	unsigned long flags;

	pr_debug("%s bEnable = %d\n", __func__, bEnable);

	spin_lock_irqsave(&afe_control_lock, flags);
	if (bEnable == true) {
		set_chip_dl_src_enable(true);
	} else {
		if (mAudioMEMIF[Soc_Aud_Digital_Block_I2S_IN_ADC]->mState == false &&
				mAudioMEMIF[Soc_Aud_Digital_Block_I2S_IN_ADC_2]->mState == false &&
				mAudioMEMIF[Soc_Aud_Digital_Block_ADDA_ANC]->mState == false) {
			set_chip_dl_src_enable(false);
		}
	}
	spin_unlock_irqrestore(&afe_control_lock, flags);
}

void SetADDAEnable(bool bEnable)
{
	unsigned long flags;

	pr_debug("%s bEnable = %d\n", __func__, bEnable);

	spin_lock_irqsave(&afe_control_lock, flags);
	if (bEnable == true) {
		set_chip_adda_enable(true);
	} else {
		if (mAudioMEMIF[Soc_Aud_Digital_Block_I2S_OUT_DAC]->mState == false &&
		    mAudioMEMIF[Soc_Aud_Digital_Block_I2S_IN_ADC]->mState == false &&
		    mAudioMEMIF[Soc_Aud_Digital_Block_I2S_IN_ADC_2]->mState == false &&
		    mAudioMEMIF[Soc_Aud_Digital_Block_ADDA_ANC]->mState == false) {
			set_chip_adda_enable(false);
		}
	}
	spin_unlock_irqrestore(&afe_control_lock, flags);
}

bool SetI2SDacEnable(bool bEnable)
{
	pr_aud("%s bEnable = %d", __func__, bEnable);

	if (bEnable) {
		/* Enable DL SRC order:
		* DL clock (AUDIO_TOP_CON0) -> AFE (AFE_DAC_CON0) ->
		* ADDA UL DL (AFE_ADDA_UL_DL_CON0) ->
		* ADDA DL SRC (AFE_ADDA_DL_SRC2_CON0)
		*/
		EnableAfe(true);
		SetADDAEnable(true);
		SetDLSrcEnable(true);
		Afe_Set_Reg(AFE_I2S_CON1, bEnable, 0x1);
	} else {
		/* Disable DL SRC order: (reverse)
		* ADDA DL SRC (AFE_ADDA_DL_SRC2_CON0) ->
		* ADDA UL DL (AFE_ADDA_UL_DL_CON0) ->
		* AFE (AFE_DAC_CON0) -> DL clock (AUDIO_TOP_CON0)
		*/
		SetDLSrcEnable(false);
		Afe_Set_Reg(AFE_I2S_CON1, bEnable, 0x1);
		SetADDAEnable(false);

		AudDrv_AUD_Sel(0);
#ifdef CONFIG_FPGA_EARLY_PORTING
		pr_warn("%s(), disable fpga clock divide by 4", __func__);
		Afe_Set_Reg(FPGA_CFG0, 0x0 << 1, 0x1 << 1);
#endif
	}

	AudDrv_GPIO_Request(bEnable, Soc_Aud_Digital_Block_ADDA_DL);

	return true;
}

bool GetI2SDacEnable(void)
{
	return mAudioMEMIF[Soc_Aud_Digital_Block_I2S_OUT_DAC]->mState;
}


bool checkDllinkMEMIfStatus(void)
{
	int i = 0;

	for (i = Soc_Aud_Digital_Block_MEM_DL1 ; i <= Soc_Aud_Digital_Block_MEM_DL2 ; i++) {
		if (mAudioMEMIF[i]->mState  == true)
			return true;
	}
	return false;
}

bool checkUplinkMEMIfStatus(void)
{
	return mAudioMEMIF[Soc_Aud_Digital_Block_MEM_VUL]->mState ||
		mAudioMEMIF[Soc_Aud_Digital_Block_MEM_DAI]->mState ||
		mAudioMEMIF[Soc_Aud_Digital_Block_MEM_AWB]->mState ||
		mAudioMEMIF[Soc_Aud_Digital_Block_MEM_MOD_DAI]->mState ||
		mAudioMEMIF[Soc_Aud_Digital_Block_MEM_VUL_DATA2]->mState;
}

bool SetConnection(uint32 ConnectionState, uint32 Input, uint32 Output)
{
	return SetConnectionState(ConnectionState, Input, Output);
}

bool SetConnectionFormat(uint32 ConnectionFormat, uint32 Aud_block)
{
	return SetIntfConnectionFormat(ConnectionFormat, Aud_block);
}

bool SetIntfConnection(uint32 ConnectionState, uint32 Aud_block_In, uint32 Aud_block_Out)
{
	return SetIntfConnectionState(ConnectionState, Aud_block_In, Aud_block_Out);
}

static bool SetIrqEnable(uint32 irqmode, bool bEnable)
{
	const struct Aud_RegBitsInfo *irqOnReg, *irqEnReg, *irqClrReg, *irqMissClrReg;
	const struct Aud_RegBitsInfo *irqPurposeEnReg;
	uint32 enShift, purposeIndex;
	bool enSet;

	pr_debug("+ %s(), Irqmode %d, bEnable %d\n", __func__, irqmode, bEnable);

	if (irqmode >= Soc_Aud_IRQ_MCU_MODE_NUM) {
		pr_err("%s(), error, not supported IRQ %d", __func__, irqmode);
		return false;
	}

	irqOnReg = &GetIRQCtrlReg(irqmode)->on;
	Afe_Set_Reg(irqOnReg->reg,
		    (bEnable << irqOnReg->sbit),
		    (irqOnReg->mask << irqOnReg->sbit));

	/* set irq signal target */
	irqEnReg = &GetIRQCtrlReg(irqmode)->en;
	for (purposeIndex = 0; purposeIndex < Soc_Aud_IRQ_PURPOSE_NUM; purposeIndex++) {
		irqPurposeEnReg = GetIRQPurposeReg(purposeIndex);
		enSet = bEnable &&
			(GetIRQCtrlReg(irqmode)->irqPurpose == purposeIndex);
		enShift = irqPurposeEnReg->sbit + irqEnReg->sbit;
		if (irqPurposeEnReg->reg != AFE_REG_UNDEFINED)
			Afe_Set_Reg(irqPurposeEnReg->reg,
					(enSet << enShift),
					(irqEnReg->mask << enShift));
	}

	/* clear irq status */
	if (bEnable == false) {
		irqClrReg = &GetIRQCtrlReg(irqmode)->clr;
		Afe_Set_Reg(irqClrReg->reg,
			    (1 << irqClrReg->sbit),
			    (irqClrReg->mask << irqClrReg->sbit));

		irqMissClrReg = &GetIRQCtrlReg(irqmode)->missclr;
		Afe_Set_Reg(irqMissClrReg->reg,
			    (1 << irqMissClrReg->sbit),
			    (irqMissClrReg->mask << irqMissClrReg->sbit));
	}

	return true;
}


static bool SetIrqMcuSampleRate(uint32 irqmode, uint32 SampleRate)
{
	uint32 SRIdx = SampleRateTransform(SampleRate, 0);
	const struct Aud_RegBitsInfo *irqModeReg;

	pr_aud("%s(), Irqmode %d, SampleRate %d\n",
		__func__, irqmode, SampleRate);

	if (irqmode >= Soc_Aud_IRQ_MCU_MODE_NUM)
		return false;

	irqModeReg = &GetIRQCtrlReg(irqmode)->mode;
	Afe_Set_Reg(irqModeReg->reg,
		    SRIdx << irqModeReg->sbit,
		    irqModeReg->mask << irqModeReg->sbit);
	return true;
}

static bool SetIrqMcuCounter(uint32 irqmode, uint32 Counter)
{
	const struct Aud_RegBitsInfo *irqCntReg;

	pr_aud("%s(), Irqmode %d, Counter %d\n", __func__, irqmode, Counter);

	if (irqmode >= Soc_Aud_IRQ_MCU_MODE_NUM)
		return false;

	irqCntReg = &GetIRQCtrlReg(irqmode)->cnt;
	Afe_Set_Reg(irqCntReg->reg, Counter, irqCntReg->mask);
	return true;
}

bool Set2ndI2SInConfig(unsigned int sampleRate, bool bIsSlaveMode)
{
	AudioDigtalI2S I2S2ndIn_attribute;

	memset((void *)&I2S2ndIn_attribute, 0, sizeof(I2S2ndIn_attribute));
	I2S2ndIn_attribute.mLR_SWAP = Soc_Aud_LR_SWAP_NO_SWAP;
	I2S2ndIn_attribute.mI2S_SLAVE = bIsSlaveMode;
	I2S2ndIn_attribute.mI2S_SAMPLERATE = sampleRate;
	I2S2ndIn_attribute.mINV_LRCK = Soc_Aud_INV_LRCK_NO_INVERSE;
	I2S2ndIn_attribute.mI2S_FMT = Soc_Aud_I2S_FORMAT_I2S;
	I2S2ndIn_attribute.mI2S_WLEN = Soc_Aud_I2S_WLEN_WLEN_16BITS;
	Set2ndI2SIn(&I2S2ndIn_attribute);

	return true;
}

bool Set2ndI2SIn(AudioDigtalI2S *mDigitalI2S)
{
	uint32 Audio_I2S_Adc = 0;

	memcpy((void *)m2ndI2S, (void *)mDigitalI2S, sizeof(AudioDigtalI2S));

	if (!m2ndI2S->mI2S_SLAVE) {	/* Master setting SampleRate only */
		SetSampleRate(Soc_Aud_Digital_Block_MEM_I2S, m2ndI2S->mI2S_SAMPLERATE);
	}

	Audio_I2S_Adc |= (m2ndI2S->mINV_LRCK << 5);
	Audio_I2S_Adc |= (m2ndI2S->mI2S_FMT << 3);
	Audio_I2S_Adc |= (m2ndI2S->mI2S_SLAVE << 2);
	Audio_I2S_Adc |= (m2ndI2S->mI2S_WLEN << 1);
	Audio_I2S_Adc |= (m2ndI2S->mI2S_IN_PAD_SEL << 28);
	pr_debug("Set2ndI2SIn Audio_I2S_Adc= 0x%x", Audio_I2S_Adc);
	Afe_Set_Reg(AFE_I2S_CON, Audio_I2S_Adc, 0xfffffffe);

	return true;
}

bool Set2ndI2SInEnable(bool bEnable)
{
	pr_warn("Set2ndI2SInEnable bEnable = %d", bEnable);
	m2ndI2S->mI2S_EN = bEnable;
	Afe_Set_Reg(AFE_I2S_CON, bEnable, 0x1);
	mAudioMEMIF[Soc_Aud_Digital_Block_I2S_IN_2]->mState = bEnable;

	return true;
}

bool SetMemIfFetchFormatPerSample(uint32 InterfaceType, uint32 eFetchFormat)
{
	mAudioMEMIF[InterfaceType]->mFetchFormatPerSample = eFetchFormat;
	return SetMemIfFormatReg(InterfaceType, eFetchFormat);
}

bool SetoutputConnectionFormat(uint32 ConnectionFormat, uint32 Output)
{
	Afe_Set_Reg(AFE_CONN_24BIT, (ConnectionFormat << Output), (1 << Output));
	return true;
}

int set_memif_pbuf_size(int aud_blk, enum memif_pbuf_size pbuf_size)
{
	if (pbuf_size < 0 || pbuf_size >= MEMIF_PBUF_SIZE_NUM) {
		pr_err("%s(), invalid pbuf_size %d\n", __func__, pbuf_size);
		return -EINVAL;
	}

	switch (aud_blk) {
	case Soc_Aud_Digital_Block_MEM_DL1:
		Afe_Set_Reg(AFE_MEMIF_PBUF_SIZE, pbuf_size << 0, 0x3 << 0);
		Afe_Set_Reg(AFE_MEMIF_MINLEN, pbuf_size << 0, 0xf << 0);
		break;
	case Soc_Aud_Digital_Block_MEM_DL2:
		Afe_Set_Reg(AFE_MEMIF_PBUF_SIZE, pbuf_size << 2, 0x3 << 2);
		Afe_Set_Reg(AFE_MEMIF_MINLEN, pbuf_size << 8, 0xf << 8);
		break;
	default:
		pr_err("%s(): invalid aud_blk %d\n", __func__, aud_blk);
		return -EINVAL;
	}

	return 0;
}

/*****************************************************************************
 * FUNCTION
 *  AudDrv_Allocate_DL1_Buffer / AudDrv_Free_DL1_Buffer
 *
 * DESCRIPTION
 *  allocate DL1 Buffer
 *
******************************************************************************/
int AudDrv_Allocate_DL1_Buffer(struct device *pDev, kal_uint32 Afe_Buf_Length,
	dma_addr_t dma_addr, unsigned char *dma_area)
{
	AFE_BLOCK_T *pblock;

	pblock = &(AFE_Mem_Control_context[Soc_Aud_Digital_Block_MEM_DL1]->rBlock);
	pblock->u4BufferSize = Afe_Buf_Length;

	if (Afe_Buf_Length > AFE_INTERNAL_SRAM_SIZE) {
		pr_err("%s(), Afe_Buf_Length %d > %d\n",
		       __func__,
		       Afe_Buf_Length,
		       AFE_INTERNAL_SRAM_SIZE);
		return -1;
	}

	pblock->pucPhysBufAddr = (kal_uint32)dma_addr;
	pblock->pucVirtBufAddr = dma_area;

	pr_warn("%s(), Afe_Buf_Length = %d, pucVirtBufAddr = %p, pblock->pucPhysBufAddr = 0x%x\n",
		__func__, Afe_Buf_Length, pblock->pucVirtBufAddr, pblock->pucPhysBufAddr);

	/* check 32 bytes align */
	if ((pblock->pucPhysBufAddr & 0x1f) != 0) {
		pr_warn("[Auddrv] AudDrv_Allocate_DL1_Buffer is not aligned (0x%x)\n",
			pblock->pucPhysBufAddr);
	}

	pblock->u4SampleNumMask = 0x001f;	/* 32 byte align */
	pblock->u4WriteIdx = 0;
	pblock->u4DMAReadIdx = 0;
	pblock->u4DataRemained = 0;
	pblock->u4fsyncflag = false;
	pblock->uResetFlag = true;

	/* set sram address top hardware */
	set_memif_addr(Soc_Aud_Digital_Block_MEM_DL1,
		       pblock->pucPhysBufAddr,
		       Afe_Buf_Length);

	memset_io(pblock->pucVirtBufAddr, 0, pblock->u4BufferSize);

	return 0;
}

int AudDrv_Allocate_mem_Buffer(struct device *pDev, Soc_Aud_Digital_Block MemBlock,
			       uint32 Buffer_length)
{
	switch (MemBlock) {
	case Soc_Aud_Digital_Block_MEM_DL1:
	case Soc_Aud_Digital_Block_MEM_DL2:
	case Soc_Aud_Digital_Block_MEM_DL3:
	case Soc_Aud_Digital_Block_MEM_DAI:
	case Soc_Aud_Digital_Block_MEM_AWB:
	case Soc_Aud_Digital_Block_MEM_MOD_DAI:
	/*case Soc_Aud_Digital_Block_MEM_DL1_DATA2:*/
	case Soc_Aud_Digital_Block_MEM_VUL_DATA2:
	case Soc_Aud_Digital_Block_MEM_VUL:
	case Soc_Aud_Digital_Block_MEM_HDMI:
		pr_debug("%s MemBlock =%d Buffer_length = %d\n ", __func__, MemBlock, Buffer_length);
		if (Audio_dma_buf[MemBlock] != NULL) {
			pr_debug("AudDrv_Allocate_mem_Buffer MemBlock = %d dma_alloc_coherent\n", MemBlock);
			if (Audio_dma_buf[MemBlock]->area == NULL) {
				pr_debug("dma_alloc_coherent\n");
				Audio_dma_buf[MemBlock]->area =
				    dma_alloc_coherent(pDev, Buffer_length,
					&Audio_dma_buf[MemBlock]->addr,
					GFP_KERNEL | GFP_DMA);
				if (Audio_dma_buf[MemBlock]->area)
					Audio_dma_buf[MemBlock]->bytes = Buffer_length;
			}
			pr_debug("area = %p\n", Audio_dma_buf[MemBlock]->area);
		}
		break;
	default:
		pr_debug("%s not support\n", __func__);
	}

	return true;
}


AFE_MEM_CONTROL_T *Get_Mem_ControlT(Soc_Aud_Digital_Block MemBlock)
{
	if (MemBlock >= 0 && MemBlock < Soc_Aud_Digital_Block_NUM_OF_MEM_INTERFACE)
		return AFE_Mem_Control_context[MemBlock];

	pr_warn("%s error\n", __func__);
	return NULL;
}


bool SetMemifSubStream(Soc_Aud_Digital_Block MemBlock, struct snd_pcm_substream *substream)
{
	substreamList *head;
	substreamList *temp = NULL;
	unsigned long flags;

	pr_aud("+%s MemBlock = %d substream = %p\n", __func__, MemBlock, substream);
	spin_lock_irqsave(&AFE_Mem_Control_context[MemBlock]->substream_lock, flags);
	head = AFE_Mem_Control_context[MemBlock]->substreamL;
	if (head == NULL) {	/* frst item is NULL */
		/* pr_debug("%s head == NULL\n ", __func__); */
		temp = kzalloc(sizeof(substreamList), GFP_ATOMIC);
		temp->substream = substream;
		temp->next = NULL;
		AFE_Mem_Control_context[MemBlock]->substreamL = temp;
	} else {		/* find out Null pointer */
		while (head->next != NULL)
			head = head->next;

		/* head->next is NULL */
		temp = kzalloc(sizeof(substreamList), GFP_ATOMIC);
		temp->substream = substream;
		temp->next = NULL;
		head->next = temp;
	}

	AFE_Mem_Control_context[MemBlock]->MemIfNum++;
	spin_unlock_irqrestore(&AFE_Mem_Control_context[MemBlock]->substream_lock, flags);
	/*pr_debug("-%s MemBlock = %d\n ", __func__, MemBlock);*/

	/* DumpMemifSubStream(); */
	return true;
}


bool ClearMemBlock(Soc_Aud_Digital_Block MemBlock)
{
	if (MemBlock >= 0 && MemBlock < Soc_Aud_Digital_Block_NUM_OF_MEM_INTERFACE) {
		AFE_BLOCK_T *pBlock = &AFE_Mem_Control_context[MemBlock]->rBlock;

		pBlock->u4WriteIdx = 0;
		pBlock->u4DMAReadIdx = 0;
		pBlock->u4DataRemained = 0;
		pBlock->u4fsyncflag = false;
		pBlock->uResetFlag = true;
	} else {
		pr_debug("%s error\n", __func__);
		return NULL;
	}

	return true;
}

#define MEM_TIMEOUT_CNT 4
bool RemoveMemifSubStream(Soc_Aud_Digital_Block MemBlock, struct snd_pcm_substream *substream)
{
	substreamList *head;
	substreamList *temp = NULL;
	unsigned long flags;
	int i;

	spin_lock_irqsave(&AFE_Mem_Control_context[MemBlock]->substream_lock, flags);

	for (i = 0; i < MEM_TIMEOUT_CNT; i++) {
		if (AFE_Mem_Control_context[MemBlock]->mWaitForIRQ == true) {
			pr_debug("%s: enter udelay.\n", __func__);
			mdelay(5);
		} else {
			break;
		}
	}

	if (AFE_Mem_Control_context[MemBlock]->MemIfNum == 0)
		pr_warn("%s AFE_Mem_Control_context[%d]->MemIfNum == 0\n ", __func__, MemBlock);
	else
		AFE_Mem_Control_context[MemBlock]->MemIfNum--;

	head = AFE_Mem_Control_context[MemBlock]->substreamL;
	pr_aud("+ %s MemBlock = %d substream = %p\n ", __func__, MemBlock, substream);

	if (head == NULL) {	/* no object */
		/* do nothing */
	} else {
		/* condition for first item hit */
		if (head->substream == substream) {
			/* pr_debug("%s head->substream = %p\n ", __func__, head->substream); */
			AFE_Mem_Control_context[MemBlock]->substreamL = head->next;
			head->substream = NULL;
			kfree(head);
			head = NULL;
			/* DumpMemifSubStream(); */
		} else {
			temp = head;
			head = head->next;
			while (head) {
				if (head->substream == substream) {
					temp->next = head->next;
					head->substream = NULL;
					kfree(head);
					head = NULL;
					break;
				}
				temp = head;
				head = head->next;
			}
		}
	}

	/* DumpMemifSubStream(); */
	if (AFE_Mem_Control_context[MemBlock]->substreamL == NULL)
		ClearMemBlock(MemBlock);
	else
		pr_debug("%s substreram is not NULL MemBlock = %d\n", __func__, MemBlock);

	spin_unlock_irqrestore(&AFE_Mem_Control_context[MemBlock]->substream_lock, flags);
	pr_aud("- %s MemBlock = %d\n ", __func__, MemBlock);

	return true;
}

static unsigned long dl2_flags;
void Auddrv_Dl2_Spinlock_lock(void)
{
	spin_lock_irqsave(&auddrv_dl2_lock, dl2_flags);
}

void Auddrv_Dl2_Spinlock_unlock(void)
{
	spin_unlock_irqrestore(&auddrv_dl2_lock, dl2_flags);
}

static unsigned long dl3_flags;
void Auddrv_Dl3_Spinlock_lock(void)
{
	spin_lock_irqsave(&auddrv_dl3_lock, dl3_flags);
}

void Auddrv_Dl3_Spinlock_unlock(void)
{
	spin_unlock_irqrestore(&auddrv_dl3_lock, dl3_flags);
}

void Auddrv_HDMI_Interrupt_Handler(void)
{
	/* irq5 ISR handler */
	/* Wait for HDMI refactor */
}

void Auddrv_AWB_Interrupt_Handler(void)
{
	AFE_MEM_CONTROL_T *Mem_Block = AFE_Mem_Control_context[Soc_Aud_Digital_Block_MEM_AWB];
	kal_uint32 HW_Cur_ReadIdx = 0;
	kal_uint32 MaxCopySize = 0;
	kal_int32 Hw_Get_bytes = 0;
	substreamList *temp = NULL;
	AFE_BLOCK_T *mBlock = NULL;
	unsigned long flags;
	kal_uint32 temp_cnt = 0;

	if (Mem_Block == NULL) {
		pr_err("-%s()Mem_Block == NULL\n ", __func__);
		return;
	}

	spin_lock_irqsave(&Mem_Block->substream_lock, flags);

	if (GetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_AWB) == false) {
		/* printk("%s(), GetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_AWB) == false, return\n ", __func__); */
		spin_unlock_irqrestore(&Mem_Block->substream_lock, flags);
		pr_err("-%s(), GetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_AWB) = %d\n ",
		       __func__, GetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_AWB));
		return;
	}

	mBlock = &Mem_Block->rBlock;
	HW_Cur_ReadIdx = word_size_align(Afe_Get_Reg(AFE_AWB_CUR));
	PRINTK_AUD_AWB("+%s HW_Cur_ReadIdx = 0x%x\n ", __func__, HW_Cur_ReadIdx);

	if (CheckSize(HW_Cur_ReadIdx)) {
		spin_unlock_irqrestore(&Mem_Block->substream_lock, flags);
		return;
	}

	if (mBlock->pucVirtBufAddr == NULL) {
		spin_unlock_irqrestore(&Mem_Block->substream_lock, flags);
		return;
	}

	MaxCopySize = Get_Mem_MaxCopySize(Soc_Aud_Digital_Block_MEM_AWB);
	PRINTK_AUD_AWB("1  mBlock = %p MaxCopySize = 0x%x u4BufferSize = 0x%x\n", mBlock,
		       MaxCopySize, mBlock->u4BufferSize);

	if (MaxCopySize) {
		if (MaxCopySize > mBlock->u4BufferSize)
			MaxCopySize = mBlock->u4BufferSize;
		mBlock->u4DataRemained -= MaxCopySize;
		mBlock->u4DMAReadIdx += MaxCopySize;
		mBlock->u4DMAReadIdx %= mBlock->u4BufferSize;
		Clear_Mem_CopySize(Soc_Aud_Digital_Block_MEM_AWB);
		PRINTK_AUD_AWB("%s read  ReadIdx:0x%x, WriteIdx:0x%x,BufAddr:0x%x  CopySize =0x%x\n",
			       __func__, mBlock->u4DMAReadIdx, mBlock->u4WriteIdx,
			       mBlock->pucPhysBufAddr, mBlock->u4MaxCopySize);
	}

	/* HW already fill in */
	Hw_Get_bytes = (HW_Cur_ReadIdx - mBlock->pucPhysBufAddr) - mBlock->u4WriteIdx;
	if (Hw_Get_bytes < 0)
		Hw_Get_bytes += mBlock->u4BufferSize;

	PRINTK_AUD_AWB
	    ("%s Get_bytes:0x%x,Cur_ReadIdx:0x%x,ReadIdx:0x%x,WriteIdx:0x%x,BufAddr:0x%x Remained = 0x%x\n",
	     __func__, Hw_Get_bytes, HW_Cur_ReadIdx, mBlock->u4DMAReadIdx, mBlock->u4WriteIdx,
	     mBlock->pucPhysBufAddr, mBlock->u4DataRemained);
	mBlock->u4WriteIdx += Hw_Get_bytes;
	mBlock->u4WriteIdx %= mBlock->u4BufferSize;
	mBlock->u4DataRemained += Hw_Get_bytes;

	/* buffer overflow */
	if (mBlock->u4DataRemained > mBlock->u4BufferSize) {
		pr_debug
		    ("%s buffer overflow u4DMAReadIdx:%x, u4WriteIdx:%x, u4DataRemained:%x, u4BufferSize:%x\n",
		     __func__, mBlock->u4DMAReadIdx, mBlock->u4WriteIdx, mBlock->u4DataRemained,
		     mBlock->u4BufferSize);
		mBlock->u4DataRemained %= mBlock->u4BufferSize;
	}

	Mem_Block->interruptTrigger = 1;
	temp = Mem_Block->substreamL;

	while (temp != NULL) {
		if (temp->substream != NULL) {
			temp_cnt = Mem_Block->MemIfNum;
			Mem_Block->mWaitForIRQ = true;
			spin_unlock_irqrestore(&Mem_Block->substream_lock, flags);
			snd_pcm_period_elapsed(temp->substream);
			Mem_Block->mWaitForIRQ = false;
			spin_lock_irqsave(&Mem_Block->substream_lock, flags);

			if (temp_cnt != Mem_Block->MemIfNum) {
				pr_debug("%s() temp_cnt = %u, Mem_Block->MemIfNum = %u\n", __func__,
					temp_cnt, Mem_Block->MemIfNum);
				temp = Mem_Block->substreamL;
			}
		}
		if (temp != NULL)
			temp = temp->next;
	}

	spin_unlock_irqrestore(&Mem_Block->substream_lock, flags);
	PRINTK_AUD_AWB("-%s u4DMAReadIdx:0x%x, u4WriteIdx:0x%x mBlock->u4DataRemained = 0x%x\n",
		       __func__, mBlock->u4DMAReadIdx, mBlock->u4WriteIdx, mBlock->u4DataRemained);
}


void Auddrv_DAI_Interrupt_Handler(void)
{
	AFE_MEM_CONTROL_T *Mem_Block = AFE_Mem_Control_context[Soc_Aud_Digital_Block_MEM_DAI];
	kal_uint32 HW_Cur_ReadIdx = 0;
	kal_int32 Hw_Get_bytes = 0;
	AFE_BLOCK_T *mBlock = NULL;
	unsigned long flags;

	if (Mem_Block == NULL)
		return;

	spin_lock_irqsave(&Mem_Block->substream_lock, flags);

	if (GetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_DAI) == false) {
		/* printk("%s(), GetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_DAI) == false, return\n ", __func__); */
		spin_unlock_irqrestore(&Mem_Block->substream_lock, flags);
		return;
	}

	mBlock = &Mem_Block->rBlock;
	HW_Cur_ReadIdx = word_size_align(Afe_Get_Reg(AFE_DAI_CUR));

	if (CheckSize(HW_Cur_ReadIdx)) {
		spin_unlock_irqrestore(&Mem_Block->substream_lock, flags);
		return;
	}

	if (mBlock->pucVirtBufAddr == NULL) {
		spin_unlock_irqrestore(&Mem_Block->substream_lock, flags);
		return;
	}

	/* HW already fill in */
	Hw_Get_bytes = (HW_Cur_ReadIdx - mBlock->pucPhysBufAddr) - mBlock->u4WriteIdx;

	if (Hw_Get_bytes < 0)
		Hw_Get_bytes += mBlock->u4BufferSize;

	PRINTK_AUD_DAI(
	"%s Hw_Get_bytes:0x%x, Cur_ReadIdx:0x%x,ReadIdx:0x%x,WriteIdx:0x%x, PhysAddr:0x%x Block->MemIfNum = %d\n",
	__func__, Hw_Get_bytes, HW_Cur_ReadIdx, mBlock->u4DMAReadIdx, mBlock->u4WriteIdx,
	mBlock->pucPhysBufAddr, Mem_Block->MemIfNum);

	mBlock->u4WriteIdx += Hw_Get_bytes;
	mBlock->u4WriteIdx %= mBlock->u4BufferSize;
	mBlock->u4DataRemained += Hw_Get_bytes;

	/* buffer overflow */
	if (mBlock->u4DataRemained > mBlock->u4BufferSize) {
		PRINTK_AUD_DAI(
			"%s buffer overflow u4DMAReadIdx:%x,WriteIdx:%x, Remained:%x, u4BufferSize:%x\n",
		__func__, mBlock->u4DMAReadIdx, mBlock->u4WriteIdx,
		mBlock->u4DataRemained, mBlock->u4BufferSize);
	}

	Mem_Block->interruptTrigger = 1;

	if (Mem_Block->substreamL != NULL) {
		if (Mem_Block->substreamL->substream != NULL) {
			Mem_Block->mWaitForIRQ = true;
			spin_unlock_irqrestore(&Mem_Block->substream_lock, flags);
			snd_pcm_period_elapsed(Mem_Block->substreamL->substream);
			Mem_Block->mWaitForIRQ = false;
			spin_lock_irqsave(&Mem_Block->substream_lock, flags);
		}
	}

	spin_unlock_irqrestore(&Mem_Block->substream_lock, flags);
}

void Auddrv_DL1_Interrupt_Handler(void)
{
	/* irq1 ISR handler */
	AFE_MEM_CONTROL_T *Mem_Block = AFE_Mem_Control_context[Soc_Aud_Digital_Block_MEM_DL1];
	kal_int32 Afe_consumed_bytes = 0;
	kal_int32 HW_memory_index = 0;
	kal_int32 HW_Cur_ReadIdx = 0;
	AFE_BLOCK_T *Afe_Block = &(AFE_Mem_Control_context[Soc_Aud_Digital_Block_MEM_DL1]->rBlock);
	unsigned long flags;

	if (Mem_Block == NULL)
		return;

	if (get_voice_usb_status() &&
	    GetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_DL1)) {
		if (Mem_Block->substreamL != NULL) {
			if (Mem_Block->substreamL->substream != NULL)
				snd_pcm_period_elapsed(Mem_Block->substreamL->substream);
		}
		return;
	}

	spin_lock_irqsave(&Mem_Block->substream_lock, flags);

	if (GetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_DL1) == false) {
		/* printk("%s(), GetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_DL1) == false, return\n ", __func__); */
		spin_unlock_irqrestore(&Mem_Block->substream_lock, flags);
		return;
	}

	HW_Cur_ReadIdx = Afe_Get_Reg(AFE_DL1_CUR);

	if (HW_Cur_ReadIdx == 0) {
		PRINTK_AUDDRV("[Auddrv] HW_Cur_ReadIdx ==0\n");
		HW_Cur_ReadIdx = Afe_Block->pucPhysBufAddr;
	}

	HW_memory_index = (HW_Cur_ReadIdx - Afe_Block->pucPhysBufAddr);
	PRINTK_AUD_DL1
	    ("[Auddrv] HW_Cur_ReadIdx=0x%x HW_memory_index = 0x%x Afe_Block->pucPhysBufAddr = 0x%x\n",
	     HW_Cur_ReadIdx, HW_memory_index, Afe_Block->pucPhysBufAddr);

	/* get hw consume bytes */
	if (HW_memory_index > Afe_Block->u4DMAReadIdx) {
		Afe_consumed_bytes = HW_memory_index - Afe_Block->u4DMAReadIdx;
	} else {
		Afe_consumed_bytes =
		    Afe_Block->u4BufferSize + HW_memory_index - Afe_Block->u4DMAReadIdx;
	}

	Afe_consumed_bytes = word_size_align(Afe_consumed_bytes);
	PRINTK_AUD_DL1("+%s ReadIdx:%x WriteIdx:%x,Remained:%x,  consumed_bytes:%x HW_memory_index = %x\n",
	__func__, Afe_Block->u4DMAReadIdx, Afe_Block->u4WriteIdx, Afe_Block->u4DataRemained,
	Afe_consumed_bytes, HW_memory_index);

	if (Afe_Block->u4DataRemained < Afe_consumed_bytes
	    || Afe_Block->u4DataRemained <= 0 || Afe_Block->u4DataRemained >
	    Afe_Block->u4BufferSize) {
		if (AFE_dL_Abnormal_context.u4UnderflowCnt < DL_ABNORMAL_CONTROL_MAX) {
			AFE_dL_Abnormal_context.pucPhysBufAddr[AFE_dL_Abnormal_context.u4UnderflowCnt] =
									Afe_Block->pucPhysBufAddr;
			AFE_dL_Abnormal_context.u4BufferSize[AFE_dL_Abnormal_context.u4UnderflowCnt] =
									Afe_Block->u4BufferSize;
			AFE_dL_Abnormal_context.u4ConsumedBytes[AFE_dL_Abnormal_context.u4UnderflowCnt] =
									Afe_consumed_bytes;
			AFE_dL_Abnormal_context.u4DataRemained[AFE_dL_Abnormal_context.u4UnderflowCnt] =
									Afe_Block->u4DataRemained;
			AFE_dL_Abnormal_context.u4DMAReadIdx[AFE_dL_Abnormal_context.u4UnderflowCnt] =
									Afe_Block->u4DMAReadIdx;
			AFE_dL_Abnormal_context.u4HwMemoryIndex[AFE_dL_Abnormal_context.u4UnderflowCnt] =
									HW_memory_index;
			AFE_dL_Abnormal_context.u4WriteIdx[AFE_dL_Abnormal_context.u4UnderflowCnt] =
									Afe_Block->u4WriteIdx;
			AFE_dL_Abnormal_context.MemIfNum[AFE_dL_Abnormal_context.u4UnderflowCnt] =
									Soc_Aud_Digital_Block_MEM_DL1;
		}
		AFE_dL_Abnormal_context.u4UnderflowCnt++;
	} else {
		PRINTK_AUD_DL1("+DL_Handling normal ReadIdx:%x ,DataRemained:%x, WriteIdx:%x\n",
			       Afe_Block->u4DMAReadIdx, Afe_Block->u4DataRemained,
			       Afe_Block->u4WriteIdx);
		Afe_Block->u4DataRemained -= Afe_consumed_bytes;
		Afe_Block->u4DMAReadIdx += Afe_consumed_bytes;
		Afe_Block->u4DMAReadIdx %= Afe_Block->u4BufferSize;
	}

	AFE_Mem_Control_context[Soc_Aud_Digital_Block_MEM_DL1]->interruptTrigger = 1;
	PRINTK_AUD_DL1("-DL_Handling normal ReadIdx:%x ,DataRemained:%x, WriteIdx:%x\n",
		       Afe_Block->u4DMAReadIdx, Afe_Block->u4DataRemained, Afe_Block->u4WriteIdx);

	if (Mem_Block->substreamL != NULL) {
		if (Mem_Block->substreamL->substream != NULL) {
			Mem_Block->mWaitForIRQ = true;
			spin_unlock_irqrestore(&Mem_Block->substream_lock, flags);
			snd_pcm_period_elapsed(Mem_Block->substreamL->substream);
			Mem_Block->mWaitForIRQ = false;
			spin_lock_irqsave(&Mem_Block->substream_lock, flags);
		}
	}
	spin_unlock_irqrestore(&Mem_Block->substream_lock, flags);
}


void Auddrv_DL2_Interrupt_Handler(void)
{
	/* irq2 ISR handler */
	AFE_MEM_CONTROL_T *Mem_Block = AFE_Mem_Control_context[Soc_Aud_Digital_Block_MEM_DL2];
	unsigned long flags;

	if (Mem_Block == NULL)
		return;

	Auddrv_Dl2_Spinlock_lock();
	spin_lock_irqsave(&Mem_Block->substream_lock, flags);

	if (GetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_DL2) == false) {
		/* printk("%s(), GetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_DL2) == false, return\n ", __func__); */
		spin_unlock_irqrestore(&Mem_Block->substream_lock, flags);
		Auddrv_Dl2_Spinlock_unlock();
		return;
	}

	if (Mem_Block->substreamL != NULL) {
		if (Mem_Block->substreamL->substream != NULL) {
			Mem_Block->mWaitForIRQ = true;
			spin_unlock_irqrestore(&Mem_Block->substream_lock, flags);
			snd_pcm_period_elapsed(Mem_Block->substreamL->substream);
			Mem_Block->mWaitForIRQ = false;
			spin_lock_irqsave(&Mem_Block->substream_lock, flags);
		}
	}
	spin_unlock_irqrestore(&Mem_Block->substream_lock, flags);

#ifdef AUDIO_DL2_ISR_COPY_SUPPORT
	mtk_dl2_copy_l();
#endif

	Auddrv_Dl2_Spinlock_unlock();
}

struct snd_dma_buffer *Get_Mem_Buffer(Soc_Aud_Digital_Block MemBlock)
{
	pr_debug("%s MemBlock = %d\n", __func__, MemBlock);
	switch (MemBlock) {
	case Soc_Aud_Digital_Block_MEM_DL1:
		return Audio_dma_buf[MemBlock];
	case Soc_Aud_Digital_Block_MEM_DL2:
		return Audio_dma_buf[MemBlock];
	case Soc_Aud_Digital_Block_MEM_DL3:
		return Audio_dma_buf[MemBlock];
	case Soc_Aud_Digital_Block_MEM_VUL:
		return Audio_dma_buf[MemBlock];
	case Soc_Aud_Digital_Block_MEM_DAI:
		return Audio_dma_buf[MemBlock];
	case Soc_Aud_Digital_Block_MEM_AWB:
		return Audio_dma_buf[MemBlock];
	case Soc_Aud_Digital_Block_MEM_MOD_DAI:
		return Audio_dma_buf[MemBlock];
	/*
	* case Soc_Aud_Digital_Block_MEM_DL1_DATA2:
	* return Audio_dma_buf[MemBlock];
	*/
	case Soc_Aud_Digital_Block_MEM_VUL_DATA2:
		return Audio_dma_buf[MemBlock];
	case Soc_Aud_Digital_Block_MEM_HDMI:
		return Audio_dma_buf[MemBlock];
	default:
		break;
	}

	return NULL;
}


void Auddrv_UL1_Interrupt_Handler(void)
{
	AFE_MEM_CONTROL_T *Mem_Block = AFE_Mem_Control_context[Soc_Aud_Digital_Block_MEM_VUL];
	kal_uint32 HW_Cur_ReadIdx = 0;
	kal_int32 Hw_Get_bytes = 0;
	AFE_BLOCK_T *mBlock = NULL;
	unsigned long flags;
	struct snd_pcm_substream *temp_substream = NULL;

	if (Mem_Block == NULL) {
		pr_err("Mem_Block == NULL\n ");
		return;
	}

	spin_lock_irqsave(&Mem_Block->substream_lock, flags);

	if (GetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_VUL) == false) {
		/* printk("%s(), GetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_VUL) == false, return\n ", __func__); */
		spin_unlock_irqrestore(&Mem_Block->substream_lock, flags);
		return;

	}

	mBlock = &Mem_Block->rBlock;
	HW_Cur_ReadIdx = word_size_align(Afe_Get_Reg(AFE_VUL_CUR));

	if (CheckSize(HW_Cur_ReadIdx)) {
		spin_unlock_irqrestore(&Mem_Block->substream_lock, flags);
		return;

	}

	if (mBlock->pucVirtBufAddr == NULL) {
		spin_unlock_irqrestore(&Mem_Block->substream_lock, flags);
		return;
	}

	/* HW already fill in */
	Hw_Get_bytes = (HW_Cur_ReadIdx - mBlock->pucPhysBufAddr) - mBlock->u4WriteIdx;

	if (Hw_Get_bytes < 0)
		Hw_Get_bytes += mBlock->u4BufferSize;

	PRINTK_AUD_UL1("%s Get_bytes:%x, Cur_ReadIdx:%x,ReadIdx:%x, WriteIdx:0x%x, BufAddr:%x MemIfNum = %d\n",
	__func__, Hw_Get_bytes, HW_Cur_ReadIdx, mBlock->u4DMAReadIdx, mBlock->u4WriteIdx,
	mBlock->pucPhysBufAddr, Mem_Block->MemIfNum);

	mBlock->u4WriteIdx += Hw_Get_bytes;
	mBlock->u4WriteIdx %= mBlock->u4BufferSize;
	mBlock->u4DataRemained += Hw_Get_bytes;

	/* buffer overflow */
	if (mBlock->u4DataRemained > mBlock->u4BufferSize) {
		pr_debug
		    ("buffer overflow u4DMAReadIdx:%x,u4WriteIdx:%x, u4DataRemained:%x, u4BufferSize:%x\n",
		     mBlock->u4DMAReadIdx, mBlock->u4WriteIdx, mBlock->u4DataRemained,
		     mBlock->u4BufferSize);
	}

	AFE_Mem_Control_context[Soc_Aud_Digital_Block_MEM_VUL]->interruptTrigger = 1;

	if (Mem_Block->substreamL != NULL) {
		if (Mem_Block->substreamL->substream != NULL) {
			temp_substream = Mem_Block->substreamL->substream;
			Mem_Block->mWaitForIRQ = true;
			spin_unlock_irqrestore(&Mem_Block->substream_lock, flags);
			snd_pcm_period_elapsed(temp_substream);
			Mem_Block->mWaitForIRQ = false;
			spin_lock_irqsave(&Mem_Block->substream_lock, flags);
		}
	}
	spin_unlock_irqrestore(&Mem_Block->substream_lock, flags);
}

static void Clear_Mem_CopySize(Soc_Aud_Digital_Block MemBlock)
{
	substreamList *head;
	/* unsigned long flags; */
	/* spin_lock_irqsave(&AFE_Mem_Control_context[MemBlock]->substream_lock, flags); */
	head = AFE_Mem_Control_context[MemBlock]->substreamL;

	/* pr_debug("+%s MemBlock = %d\n ", __func__, MemBlock); */
	while (head != NULL) {	/* frst item is NULL */
		head->u4MaxCopySize = 0;
		head = head->next;
	}

	/* spin_unlock_irqrestore(&AFE_Mem_Control_context[MemBlock]->substream_lock, flags); */
	/* printk("-%s MemBlock = %d\n ", __func__, MemBlock); */
}

kal_uint32 Get_Mem_CopySizeByStream(Soc_Aud_Digital_Block MemBlock,
				    struct snd_pcm_substream *substream)
{
	substreamList *head;
	unsigned long flags;
	kal_uint32 MaxCopySize;

	spin_lock_irqsave(&AFE_Mem_Control_context[MemBlock]->substream_lock, flags);
	head = AFE_Mem_Control_context[MemBlock]->substreamL;

	/* printk("+%s MemBlock = %d\n ", __func__, MemBlock); */
	while (head != NULL) {	/* frst item is NULL */
		if (head->substream == substream) {
			MaxCopySize = head->u4MaxCopySize;
			spin_unlock_irqrestore(&AFE_Mem_Control_context[MemBlock]->substream_lock,
					       flags);
			return MaxCopySize;
		}
		head = head->next;
	}
	spin_unlock_irqrestore(&AFE_Mem_Control_context[MemBlock]->substream_lock, flags);
	/* printk("-%s MemBlock = %d\n ", __func__, MemBlock); */

	return 0;
}

static kal_uint32 Get_Mem_MaxCopySize(Soc_Aud_Digital_Block MemBlock)
{
	substreamList *head;

	/* unsigned long flags; */
	kal_uint32 MaxCopySize;

	/* spin_lock_irqsave(&AFE_Mem_Control_context[MemBlock]->substream_lock, flags); */
	head = AFE_Mem_Control_context[MemBlock]->substreamL;
	MaxCopySize = 0;
	/* printk("+%s MemBlock = %d\n ", __func__, MemBlock); */
	while (head != NULL) {	/* frst item is NULL */
		if (MaxCopySize < head->u4MaxCopySize)
			MaxCopySize = head->u4MaxCopySize;
		head = head->next;
	}

	/* spin_unlock_irqrestore(&AFE_Mem_Control_context[MemBlock]->substream_lock, flags); */
	/* printk("-%s MemBlock = %d\n ", __func__, MemBlock); */
	return MaxCopySize;
}

void Set_Mem_CopySizeByStream(Soc_Aud_Digital_Block MemBlock,
			      struct snd_pcm_substream *substream, uint32 size)
{
	substreamList *head;
	unsigned long flags;

	spin_lock_irqsave(&AFE_Mem_Control_context[MemBlock]->substream_lock, flags);
	head = AFE_Mem_Control_context[MemBlock]->substreamL;

	/* printk("+%s MemBlock = %d\n ", __func__, MemBlock); */
	while (head != NULL) {	/* frst item is NULL */
		if (head->substream == substream) {
			head->u4MaxCopySize += size;
			break;
		}
		head = head->next;
	}
	spin_unlock_irqrestore(&AFE_Mem_Control_context[MemBlock]->substream_lock, flags);
	/* printk("-%s MemBlock = %d\n ", __func__, MemBlock); */
}

void Auddrv_UL2_Interrupt_Handler(void)
{
	AFE_MEM_CONTROL_T *Mem_Block = AFE_Mem_Control_context[Soc_Aud_Digital_Block_MEM_VUL_DATA2];
	kal_uint32 HW_Cur_ReadIdx = 0;
	kal_int32 Hw_Get_bytes = 0;
	AFE_BLOCK_T *mBlock = NULL;
	unsigned long flags;

	PRINTK_AUD_UL2("Auddrv_UL2_Interrupt_Handler\n ");

	if (Mem_Block == NULL) {
		pr_err("Mem_Block == NULL\n ");
		return;
	}

	spin_lock_irqsave(&Mem_Block->substream_lock, flags);

	if (GetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_VUL_DATA2) == false) {
		spin_unlock_irqrestore(&Mem_Block->substream_lock, flags);
		return;
	}

	mBlock = &Mem_Block->rBlock;
	HW_Cur_ReadIdx = word_size_align(Afe_Get_Reg(AFE_VUL_D2_CUR));
	PRINTK_AUD_UL2("Auddrv_UL2_Interrupt_Handler HW_Cur_ReadIdx = 0x%x\n ", HW_Cur_ReadIdx);

	if (CheckSize(HW_Cur_ReadIdx)) {
		spin_unlock_irqrestore(&Mem_Block->substream_lock, flags);
		return;
	}

	if (mBlock->pucVirtBufAddr == NULL) {
		spin_unlock_irqrestore(&Mem_Block->substream_lock, flags);
		return;
	}

	/* HW already fill in */
	Hw_Get_bytes = (HW_Cur_ReadIdx - mBlock->pucPhysBufAddr) - mBlock->u4WriteIdx;

	if (Hw_Get_bytes < 0)
		Hw_Get_bytes += mBlock->u4BufferSize;

	PRINTK_AUD_UL2("%s Get_bytes:%x, Cur_ReadIdx:%x,RadIdx:%x, WriteIdx:0x%x, BufAddr:%x MemIfNum = %d\n",
	__func__, Hw_Get_bytes, HW_Cur_ReadIdx, mBlock->u4DMAReadIdx, mBlock->u4WriteIdx,
	mBlock->pucPhysBufAddr, Mem_Block->MemIfNum);

	mBlock->u4WriteIdx += Hw_Get_bytes;
	mBlock->u4WriteIdx %= mBlock->u4BufferSize;
	mBlock->u4DataRemained += Hw_Get_bytes;

	/* buffer overflow */
	if (mBlock->u4DataRemained > mBlock->u4BufferSize) {
		pr_debug
		    ("buffer overflow u4DMAReadIdx:%x,u4WriteIdx:%x, u4DataRemained:%x, u4BufferSize:%x\n",
		     mBlock->u4DMAReadIdx, mBlock->u4WriteIdx, mBlock->u4DataRemained,
		     mBlock->u4BufferSize);
	}

	AFE_Mem_Control_context[Soc_Aud_Digital_Block_MEM_VUL_DATA2]->interruptTrigger = 1;

	if (Mem_Block->substreamL != NULL) {
		if (Mem_Block->substreamL->substream != NULL) {
			Mem_Block->mWaitForIRQ = true;
			spin_unlock_irqrestore(&Mem_Block->substream_lock, flags);
			snd_pcm_period_elapsed(Mem_Block->substreamL->substream);
			Mem_Block->mWaitForIRQ = false;
			spin_lock_irqsave(&Mem_Block->substream_lock, flags);
		}
	}
	spin_unlock_irqrestore(&Mem_Block->substream_lock, flags);
}

void Auddrv_MOD_DAI_Interrupt_Handler(void)
{
	AFE_MEM_CONTROL_T *Mem_Block = AFE_Mem_Control_context[Soc_Aud_Digital_Block_MEM_MOD_DAI];
	kal_uint32 HW_Cur_ReadIdx = 0;
	kal_int32 Hw_Get_bytes = 0;
	AFE_BLOCK_T  *mBlock = NULL;
	unsigned long flags;

	if (Mem_Block == NULL)
		return;

	spin_lock_irqsave(&Mem_Block->substream_lock, flags);
	if (GetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_MOD_DAI) == false) {
		spin_unlock_irqrestore(&Mem_Block->substream_lock, flags);
		return;
	}

	mBlock = &Mem_Block->rBlock;
	HW_Cur_ReadIdx = word_size_align(Afe_Get_Reg(AFE_MOD_DAI_CUR));

	if (CheckSize(HW_Cur_ReadIdx)) {
		spin_unlock_irqrestore(&Mem_Block->substream_lock, flags);
		return;
	}
	if (mBlock->pucVirtBufAddr  == NULL) {
		spin_unlock_irqrestore(&Mem_Block->substream_lock, flags);
		return;
	}

	Hw_Get_bytes = (HW_Cur_ReadIdx - mBlock->pucPhysBufAddr) - mBlock->u4WriteIdx;
	if (Hw_Get_bytes < 0)
		Hw_Get_bytes += mBlock->u4BufferSize;

	mBlock->u4WriteIdx  += Hw_Get_bytes;
	mBlock->u4WriteIdx  %= mBlock->u4BufferSize;
	mBlock->u4DataRemained += Hw_Get_bytes;

	if (mBlock->u4DataRemained > mBlock->u4BufferSize) {
		pr_err("%s buffer overflow u4DMAReadIdx:%x, u4WriteIdx:%x, u4DataRemained:%x, u4BufferSize:%x\n",
		__func__, mBlock->u4DMAReadIdx, mBlock->u4WriteIdx, mBlock->u4DataRemained, mBlock->u4BufferSize);
	}
	Mem_Block->interruptTrigger = 1;

	if (Mem_Block->substreamL != NULL) {
		if (Mem_Block->substreamL->substream != NULL) {
			spin_unlock_irqrestore(&Mem_Block->substream_lock, flags);
			snd_pcm_period_elapsed(Mem_Block->substreamL->substream);
			spin_lock_irqsave(&Mem_Block->substream_lock, flags);
		}
	}
	spin_unlock_irqrestore(&Mem_Block->substream_lock, flags);
}

bool Restore_Audio_Register(void)
{
	/* 6752 TODO? */
	return true;
}

unsigned int word_size_align(unsigned int in_size)
{
	unsigned int align_size;

	/* sram is device memory, need word size align, 8 byte for 64 bit platform */
	/* [3:0] = 4'h0 for the convenience of the hardware implementation */
	align_size = in_size & 0xFFFFFFF0;

	return align_size;
}

void AudDrv_checkDLISRStatus(void)
{
	unsigned long flags1;
	AFE_DL_ABNORMAL_CONTROL_T localctl;
	bool dumplog = false;

	spin_lock_irqsave(&afe_dl_abnormal_context_lock, flags1);
	if (AFE_dL_Abnormal_context.IrqDelayCnt || AFE_dL_Abnormal_context.u4UnderflowCnt) {
		memcpy((void *)&localctl, (void *)&AFE_dL_Abnormal_context, sizeof(AFE_DL_ABNORMAL_CONTROL_T));
		dumplog = true;
		if (AFE_dL_Abnormal_context.IrqDelayCnt > 0)
			AFE_dL_Abnormal_context.IrqDelayCnt = 0;

		if (AFE_dL_Abnormal_context.u4UnderflowCnt > 0)
			AFE_dL_Abnormal_context.u4UnderflowCnt = 0;
	}
	spin_unlock_irqrestore(&afe_dl_abnormal_context_lock, flags1);

	if (dumplog) {
		int index = 0;

		if (localctl.IrqDelayCnt) {
			for (index = 0; index < localctl.IrqDelayCnt && index < DL_ABNORMAL_CONTROL_MAX; index++) {
				pr_warn("AudWarn isr blocked [%d/%d] %llu - %llu = %llu > %d ms\n",
					index, localctl.IrqDelayCnt,
					localctl.IrqCurrentTimeNs[index],
					localctl.IrqLastTimeNs[index],
					localctl.IrqIntervalNs[index],
					localctl.IrqIntervalLimitMs[index]);
			}
		}
		if (localctl.u4UnderflowCnt) {
			for (index = 0; index < localctl.u4UnderflowCnt && index < DL_ABNORMAL_CONTROL_MAX; index++) {
				pr_warn("AudWarn data underflow [%d/%d] MemType %d, Remain:0x%x, R:0x%x,",
					index,
					localctl.u4UnderflowCnt,
					localctl.MemIfNum[index],
					localctl.u4DataRemained[index],
					localctl.u4DMAReadIdx[index]);
				pr_warn("W:0x%x, BufSize:0x%x, consumebyte:0x%x, hw index:0x%x, addr:0x%x\n",
					localctl.u4WriteIdx[index],
					localctl.u4BufferSize[index],
					localctl.u4ConsumedBytes[index],
					localctl.u4HwMemoryIndex[index],
					localctl.pucPhysBufAddr[index]);
			}
		}
	}
}

bool InitSramManager(struct device *pDev, unsigned int sramblocksize)
{
	int i = 0;

	memset((void *)&mAud_Sram_Manager, 0, sizeof(Aud_Sram_Manager));
	mAud_Sram_Manager.msram_phys_addr = Get_Afe_Sram_Phys_Addr();
	mAud_Sram_Manager.msram_virt_addr = Get_Afe_SramBase_Pointer();
	mAud_Sram_Manager.mSramLength =  Get_Afe_Sram_Length();
	mAud_Sram_Manager.mBlockSize = sramblocksize;
	mAud_Sram_Manager.mBlocknum = (mAud_Sram_Manager.mSramLength / mAud_Sram_Manager.mBlockSize);

	pr_warn("%s mBlocknum = %d mAud_Sram_Manager.mSramLength = %d mAud_Sram_Manager.mBlockSize = %d\n",
			__func__, mAud_Sram_Manager.mBlocknum,
			mAud_Sram_Manager.mSramLength,
			mAud_Sram_Manager.mBlockSize);

	/* Dynamic allocate mAud_Sram_Block according to mBlocknum */
	mAud_Sram_Manager.mAud_Sram_Block = devm_kzalloc(pDev,
			mAud_Sram_Manager.mBlocknum * sizeof(Aud_Sram_Block), GFP_KERNEL);

	for (i = 0; i < mAud_Sram_Manager.mBlocknum ; i++) {
		mAud_Sram_Manager.mAud_Sram_Block[i].mValid = true;
		mAud_Sram_Manager.mAud_Sram_Block[i].mLength = mAud_Sram_Manager.mBlockSize;
		mAud_Sram_Manager.mAud_Sram_Block[i].mUser = 0;
		mAud_Sram_Manager.mAud_Sram_Block[i].msram_phys_addr =
			mAud_Sram_Manager.msram_phys_addr + (sramblocksize * i);
		mAud_Sram_Manager.mAud_Sram_Block[i].msram_virt_addr =
			(void *)((char *)mAud_Sram_Manager.msram_virt_addr + (sramblocksize * i));
	}
	return true;
}

bool CheckSramAvail(unsigned int mSramLength, unsigned int *mSramBlockidx, unsigned int *mSramBlocknum)
{
	unsigned int MaxSramSize = 0;
	bool StartRecord = false;
	Aud_Sram_Block  *SramBlock = NULL;
	int i = 0;

	*mSramBlockidx = 0;

	for (i = 0; i < mAud_Sram_Manager.mBlocknum; i++) {
		SramBlock = &mAud_Sram_Manager.mAud_Sram_Block[i];
		if ((SramBlock->mUser == NULL) && SramBlock->mValid) {
			MaxSramSize += mAud_Sram_Manager.mBlockSize;
			if (StartRecord == false) {
				StartRecord = true;
				*mSramBlockidx = i;
			}
			(*mSramBlocknum)++;

			/* can callocate sram */
			if (MaxSramSize >= mSramLength)
				break;
		}

		/* when reach allocate buffer , reset condition*/
		if ((SramBlock->mUser != NULL) && SramBlock->mValid) {
			MaxSramSize = 0;
			*mSramBlocknum = 0;
			*mSramBlockidx = 0;
			StartRecord = false;
		}

		if (SramBlock->mValid == 0) {
			pr_warn("%s SramBlock->mValid == 0 i = %d\n", __func__, i);
			break;
		}
	}

	pr_warn("%s MaxSramSize = %d mSramLength = %d mSramBlockidx = %d mSramBlocknum= %d\n",
	 __func__, MaxSramSize, mSramLength, *mSramBlockidx, *mSramBlocknum);

	if (MaxSramSize >= mSramLength)
		return true;
	else
		return false;
}

int AllocateAudioSram(dma_addr_t *sram_phys_addr, unsigned char **msram_virt_addr,
	unsigned int mSramLength, void *user)
{
	unsigned int SramBlockNum = 0;
	unsigned int SramBlockidx = 0;
	int ret = 0;

	AfeControlSramLock();
	if (CheckSramAvail(mSramLength, &SramBlockidx, &SramBlockNum) == true) {
		*sram_phys_addr = mAud_Sram_Manager.mAud_Sram_Block[SramBlockidx].msram_phys_addr;
		*msram_virt_addr = (char *)mAud_Sram_Manager.mAud_Sram_Block[SramBlockidx].msram_virt_addr;

		/* set aud sram with user*/
		while (SramBlockNum) {
			mAud_Sram_Manager.mAud_Sram_Block[SramBlockidx].mUser = user;
			SramBlockNum--;
			SramBlockidx++;
		}
		AfeControlSramUnLock();
	} else {
		AfeControlSramUnLock();
		ret =  -ENOMEM;
	}

	return ret;
}

int freeAudioSram(void *user)
{
	unsigned int i = 0;
	Aud_Sram_Block  *SramBlock = NULL;

	AfeControlSramLock();
	for (i = 0; i < mAud_Sram_Manager.mBlocknum ; i++) {
		SramBlock = &mAud_Sram_Manager.mAud_Sram_Block[i];
		if (SramBlock->mUser == user) {
			SramBlock->mUser = NULL;
			pr_aud("%s SramBlockidx = %d\n", __func__, i);
		}
	}
	AfeControlSramUnLock();
	return 0;
}

/* IRQ Manager */
static int enable_aud_irq(const struct irq_user *_irq_user,
			  enum Soc_Aud_IRQ_MCU_MODE _irq,
			  unsigned int _rate,
			  unsigned int _count)
{
	SetIrqMcuSampleRate(_irq, _rate);
	SetIrqMcuCounter(_irq, _count);
	SetIrqEnable(_irq, true);

	irq_managers[_irq].is_on = true;
	irq_managers[_irq].rate = _rate;
	irq_managers[_irq].count = _count;
	irq_managers[_irq].selected_user = _irq_user;

	return 0;
}

static int disable_aud_irq(enum Soc_Aud_IRQ_MCU_MODE _irq)
{
	SetIrqEnable(_irq, false);
	SetIrqMcuCounter(_irq, 0);

	irq_managers[_irq].is_on = false;
	irq_managers[_irq].count = 0;
	irq_managers[_irq].selected_user = NULL;
	return 0;
}

static int update_aud_irq(const struct irq_user *_irq_user,
			  enum Soc_Aud_IRQ_MCU_MODE _irq,
			  unsigned int _count)
{
	SetIrqMcuCounter(_irq, _count);
	irq_managers[_irq].count = _count;
	irq_managers[_irq].selected_user = _irq_user;
	return 0;
}

static void dump_irq_manager(void)
{
	struct irq_user *ptr;
	int i;

	for (i = 0; i < Soc_Aud_IRQ_MCU_MODE_NUM; i++) {
		pr_warn("irq_managers[%d], is_on %d, rate %d, count %d, selected_user %p\n",
			i,
			irq_managers[i].is_on,
			irq_managers[i].rate,
			irq_managers[i].count,
			(void *)irq_managers[i].selected_user);

		list_for_each_entry(ptr, &irq_managers[i].users, list) {
			pr_warn("\tirq_user: user %p, rate %d, count %d\n",
				ptr->user,
				ptr->request_rate,
				ptr->request_count);
		}
	}
}

static unsigned int get_tgt_count(unsigned int _rate,
				  unsigned int _count,
				  unsigned int _tgt_rate)
{
	return ((_tgt_rate / 100) * _count) / (_rate / 100);
}

static bool is_tgt_rate_ok(unsigned int _rate,
			   unsigned int _count,
			   unsigned int _tgt_rate)
{
	unsigned int tgt_rate = _tgt_rate / 100;
	unsigned int request_rate = _rate / 100;
	unsigned int target_cnt = get_tgt_count(_rate, _count, _tgt_rate);
	unsigned int val_1 = _count * tgt_rate;
	unsigned int val_2 = target_cnt * request_rate;
	unsigned int val_3 = (IRQ_TOLERANCE_US * tgt_rate * request_rate)
			     / 100;

	if (target_cnt <= 1)
		return false;

	if (val_1 > val_2) {
		if (val_1 - val_2 >= val_3)
			return false;
	} else {
		if (val_2 - val_1 >= val_3)
			return false;
	}

	return true;
}

static bool is_period_smaller(enum Soc_Aud_IRQ_MCU_MODE _irq,
			      struct irq_user *_user)
{
	const struct irq_user *selected_user = irq_managers[_irq].selected_user;

	if (selected_user != NULL) {
		if (get_tgt_count(_user->request_rate,
				  _user->request_count,
				  IRQ_MAX_RATE) >=
		    get_tgt_count(selected_user->request_rate,
				  selected_user->request_count,
				  IRQ_MAX_RATE))
			return false;
	}

	return true;
}

static const struct irq_user *get_min_period_user(
	enum Soc_Aud_IRQ_MCU_MODE _irq)
{
	struct irq_user *ptr;
	struct irq_user *min_user = NULL;
	unsigned int min_count = IRQ_MAX_RATE;
	unsigned int cur_count;

	if (list_empty(&irq_managers[_irq].users)) {
		pr_err("error, irq_managers[%d].users is empty\n", _irq);
		dump_irq_manager();
		AUDIO_AEE("error, irq_managers[].users is empty\n");
	}

	list_for_each_entry(ptr, &irq_managers[_irq].users, list) {
		cur_count = get_tgt_count(ptr->request_rate,
					  ptr->request_count,
					  IRQ_MAX_RATE);
		if (cur_count < min_count) {
			min_count = cur_count;
			min_user = ptr;
		}
	}

	return min_user;
}

static int check_and_update_irq(const struct irq_user *_irq_user,
				enum Soc_Aud_IRQ_MCU_MODE _irq)
{
	if (!is_tgt_rate_ok(_irq_user->request_rate,
			    _irq_user->request_count,
			    irq_managers[_irq].rate)) {
		/* if you got here, you should reconsider your irq usage */
		pr_err("error, irq not updated, irq %d, irq rate %d, rate %d, count %d\n",
			_irq,
			irq_managers[_irq].rate,
			_irq_user->request_rate,
			_irq_user->request_count);
		dump_irq_manager();

		/* mt6797 disable for MP, enable before enter SQC !!!! */
		/* AUDIO_AEE("error, irq not updated\n"); */

		return -EINVAL;
	}

	update_aud_irq(_irq_user,
		       _irq,
		       get_tgt_count(_irq_user->request_rate,
				     _irq_user->request_count,
				     irq_managers[_irq].rate));

	return 0;
}

int init_irq_manager(void)
{
	int i;

	memset((void *)&irq_managers, 0, sizeof(irq_managers));
	for (i = 0; i < Soc_Aud_IRQ_MCU_MODE_NUM; i++)
		INIT_LIST_HEAD(&irq_managers[i].users);

	return 0;
}

int irq_add_user(const void *_user,
		 enum Soc_Aud_IRQ_MCU_MODE _irq,
		 unsigned int _rate,
		 unsigned int _count)
{
	unsigned long flags;
	struct irq_user *new_user;
	struct irq_user *ptr;

	spin_lock_irqsave(&afe_control_lock, flags);
	list_for_each_entry(ptr, &irq_managers[_irq].users, list) {
		if (ptr->user == _user) {
			pr_err("error, _user %p already exist\n", _user);
			dump_irq_manager();
			AUDIO_AEE("error, _user already exist\n");
		}
	}

	/* create instance */
	new_user = kzalloc(sizeof(*new_user), GFP_ATOMIC);
	if (!new_user) {
		spin_unlock_irqrestore(&afe_control_lock, flags);
		return -ENOMEM;
	}

	new_user->user = _user;
	new_user->request_rate = _rate;
	new_user->request_count = _count;
	INIT_LIST_HEAD(&new_user->list);

	/* add user to list */
	list_add(&new_user->list, &irq_managers[_irq].users);

	/* */
	if (irq_managers[_irq].is_on) {
		if (is_period_smaller(_irq, new_user))
			check_and_update_irq(new_user, _irq);
	} else {
		enable_aud_irq(new_user,
			       _irq,
			       _rate,
			       _count);
	}

	spin_unlock_irqrestore(&afe_control_lock, flags);
	return 0;
}

int irq_remove_user(const void *_user,
		    enum Soc_Aud_IRQ_MCU_MODE _irq)
{
	unsigned long flags;
	struct irq_user *ptr;
	struct irq_user *corr_user = NULL;

	spin_lock_irqsave(&afe_control_lock, flags);
	list_for_each_entry(ptr, &irq_managers[_irq].users, list) {
		if (ptr->user == _user) {
			corr_user = ptr;
			break;
		}
	}
	if (corr_user == NULL) {
		pr_err("%s(), error, _user not found\n", __func__);
		dump_irq_manager();
		AUDIO_AEE("error, _user not found\n");
		spin_unlock_irqrestore(&afe_control_lock, flags);
		return -EINVAL;
	}
	/* remove from irq_handler[_irq].users */
	list_del(&corr_user->list);

	/* check if is selected user */
	if (corr_user == irq_managers[_irq].selected_user) {
		if (list_empty(&irq_managers[_irq].users))
			disable_aud_irq(_irq);
		else
			check_and_update_irq(get_min_period_user(_irq), _irq);
	}
	/* free */
	kfree(corr_user);

	spin_unlock_irqrestore(&afe_control_lock, flags);
	return 0;
}

int irq_update_user(const void *_user,
		    enum Soc_Aud_IRQ_MCU_MODE _irq,
		    unsigned int _rate,
		    unsigned int _count)
{
	unsigned long flags;
	struct irq_user *ptr;
	struct irq_user *corr_user = NULL;

	spin_lock_irqsave(&afe_control_lock, flags);
	list_for_each_entry(ptr, &irq_managers[_irq].users, list) {
		if (ptr->user == _user) {
			corr_user = ptr;
			break;
		}
	}
	if (corr_user == NULL) {
		pr_err("%s(), error, _user not found\n", __func__);
		dump_irq_manager();
		AUDIO_AEE("error, _user not found\n");
		spin_unlock_irqrestore(&afe_control_lock, flags);
		return -EINVAL;
	}

	/* if _rate == 0, just update count */
	if (_rate)
		corr_user->request_rate = _rate;

	corr_user->request_count = _count;

	/* update irq user */
	if (corr_user == irq_managers[_irq].selected_user) {
		/* selected user */
		check_and_update_irq(get_min_period_user(_irq), _irq);
	} else {
		/* not selected user */
		if (is_period_smaller(_irq, corr_user))
			check_and_update_irq(corr_user, _irq);
	}

	spin_unlock_irqrestore(&afe_control_lock, flags);
	return 0;
}

int irq_get_total_user(enum Soc_Aud_IRQ_MCU_MODE _irq)
{
	unsigned long flags;
	struct irq_user *ptr;
	unsigned int users = 0;

	spin_lock_irqsave(&afe_control_lock, flags);
	list_for_each_entry(ptr, &irq_managers[_irq].users, list) {
		users++;
	}

	spin_unlock_irqrestore(&afe_control_lock, flags);
	return users;
}
/* IRQ Manager END*/

/* memif lpbk api */
/*
 * implementation of hw data delay mechanism using memif
 * dl and ul memif share one same memory,
 * buffer size is 2 * delay_us
 * dl memif will start first, after a delay_us the ul will start
 */
static struct memif_lpbk *cur_memif_lpbk;
int memif_lpbk_enable(struct memif_lpbk *memif_lpbk)
{
	size_t mem_size;
	unsigned int format_bytes;
	unsigned int rate = memif_lpbk->rate;
	unsigned int delay_us = memif_lpbk->delay_us;
	unsigned int format = memif_lpbk->format;

	if (cur_memif_lpbk != NULL) {
		pr_err("%s(), cur_memif_lpbk %p != NULL\n",
		       __func__, cur_memif_lpbk);
		return -EINVAL;
	}

	if (memif_lpbk->enable) {
		pr_err("%s(), memif_lpbk %p already enabled\n",
		       __func__, memif_lpbk);
		return -EINVAL;
	}

	if (GetMemoryPathEnable(memif_lpbk->dl_memif)) {
		pr_err("%s(), dl_memif %d in use\n",
		       __func__, memif_lpbk->dl_memif);
		return -EBUSY;
	}

	if (GetMemoryPathEnable(memif_lpbk->ul_memif)) {
		pr_err("%s(), ul_memif %d in use\n",
		       __func__, memif_lpbk->ul_memif);
		return -EBUSY;
	}

#ifdef MEMIF_LPBK_IRQ
	if (irq_get_total_user(memif_lpbk->irq) != 0) {
		pr_err("%s(), irq %d in use\n",
		       __func__, memif_lpbk->irq);
		return -EBUSY;
	}
#endif
	/* calculate memory size for delay_us */
	if (format == SNDRV_PCM_FORMAT_S32_LE ||
	    format == SNDRV_PCM_FORMAT_U32_LE ||
	    format == SNDRV_PCM_FORMAT_S24_LE ||
	    format == SNDRV_PCM_FORMAT_U24_LE)
		format_bytes = 4;
	else
		format_bytes = 2;

	mem_size = ((delay_us * rate) / 1000000);
	mem_size *= memif_lpbk->channel * format_bytes;
	mem_size = word_size_align(mem_size);

	pr_debug("%s(), ul_memif %d, dl_memif %d, rate %u, ch %u, fmt %d, delay_us %u, mem_size %zu\n",
		 __func__,
		 memif_lpbk->ul_memif, memif_lpbk->dl_memif,
		 rate, memif_lpbk->channel, format,
		 delay_us,
		 mem_size);

	/* allocate memory */
	memif_lpbk->dma_bytes = mem_size;
	if (AllocateAudioSram(&memif_lpbk->dma_addr,
			      &memif_lpbk->dma_area,
			      memif_lpbk->dma_bytes,
			      memif_lpbk) == 0) {
		memif_lpbk->use_dram = false;
	} else {
		memif_lpbk->dma_area = dma_alloc_coherent(memif_lpbk->dev,
							  memif_lpbk->dma_bytes,
							  &memif_lpbk->dma_addr,
							  GFP_KERNEL | GFP_DMA);
		if (!memif_lpbk->dma_area) {
			pr_err("%s(), dma_alloc_coherent fail\n", __func__);
			return -ENOMEM;
		}
		memif_lpbk->use_dram = true;
		AudDrv_Emi_Clk_On();
	}

	AudDrv_Clk_On();
	memset_io(memif_lpbk->dma_area, 0, memif_lpbk->dma_bytes);

	/* setup memif */
	SetHighAddr(memif_lpbk->dl_memif,
		    memif_lpbk->use_dram, memif_lpbk->dma_addr);
	SetHighAddr(memif_lpbk->ul_memif,
		    memif_lpbk->use_dram, memif_lpbk->dma_addr);

	set_memif_addr(memif_lpbk->dl_memif,
		       memif_lpbk->dma_addr, memif_lpbk->dma_bytes);
	SetSampleRate(memif_lpbk->dl_memif, memif_lpbk->rate);
	SetChannels(memif_lpbk->dl_memif, memif_lpbk->channel);

	set_memif_addr(memif_lpbk->ul_memif,
		       memif_lpbk->dma_addr, memif_lpbk->dma_bytes);
	SetSampleRate(memif_lpbk->ul_memif, memif_lpbk->rate);
	SetChannels(memif_lpbk->ul_memif, memif_lpbk->channel);

	/* set memif format */
	if (format == SNDRV_PCM_FORMAT_S32_LE ||
	    format == SNDRV_PCM_FORMAT_U32_LE ||
	    format == SNDRV_PCM_FORMAT_S24_LE ||
	    format == SNDRV_PCM_FORMAT_U24_LE) {
		SetMemIfFetchFormatPerSample(memif_lpbk->dl_memif,
					     AFE_WLEN_32_BIT_ALIGN_8BIT_0_24BIT_DATA);
		SetMemIfFetchFormatPerSample(memif_lpbk->ul_memif,
					     AFE_WLEN_32_BIT_ALIGN_8BIT_0_24BIT_DATA);
	} else {
		SetMemIfFetchFormatPerSample(memif_lpbk->dl_memif,
					     AFE_WLEN_16_BIT);
		SetMemIfFetchFormatPerSample(memif_lpbk->ul_memif,
					     AFE_WLEN_16_BIT);
	}

	/* set pbuf size */
	set_memif_pbuf_size(memif_lpbk->dl_memif, MEMIF_PBUF_SIZE_32_BYTES);

	/* enable memif with a bit delay */
	EnableAfe(true);

	/* note: dl memif have prefetch buffer, it will have a leap at the beginning */
	SetMemoryPathEnable(memif_lpbk->dl_memif, true);
	udelay(30);
	SetMemoryPathEnable(memif_lpbk->ul_memif, true);

	pr_debug("%s(), memif_lpbk path hw enabled\n", __func__);

	memif_lpbk->enable = true;
	cur_memif_lpbk = memif_lpbk;
	return 0;
}

int memif_lpbk_disable(struct memif_lpbk *memif_lpbk)
{
	pr_debug("%s()\n", __func__);

	if (!cur_memif_lpbk) {
		pr_err("%s(), cur_memif_lpbk %p == NULL\n",
		       __func__, cur_memif_lpbk);
		return -EINVAL;
	}

	if (!memif_lpbk->enable) {
		pr_err("%s(), memif_lpbk %p not enabled\n",
		       __func__, memif_lpbk);
		return -EINVAL;
	}

	SetMemoryPathEnable(memif_lpbk->dl_memif, false);
	SetMemoryPathEnable(memif_lpbk->ul_memif, false);
	/* resume pbuf size */
	set_memif_pbuf_size(memif_lpbk->dl_memif, MEMIF_PBUF_SIZE_256_BYTES);

	EnableAfe(false);

	/* free memory */
	if (memif_lpbk->use_dram) {
		dma_free_coherent(memif_lpbk->dev,
				  memif_lpbk->dma_bytes,
				  memif_lpbk->dma_area,
				  memif_lpbk->dma_addr);
		AudDrv_Emi_Clk_Off();
	} else {
		freeAudioSram(memif_lpbk);
	}


	AudDrv_Clk_Off();
	memif_lpbk->enable = false;
	cur_memif_lpbk = NULL;
	return 0;
}

#ifdef MEMIF_LPBK_IRQ
int memif_lpbk_irq_handler(void)
{
	pr_debug("%s()\n", __func__);
	if (!cur_memif_lpbk) {
		pr_err("%s(), cur_memif_lpbk %p == NULL\n",
		       __func__, cur_memif_lpbk);
		return -EINVAL;
	}

	SetMemoryPathEnable(cur_memif_lpbk->ul_memif, true);
	irq_remove_user(cur_memif_lpbk, cur_memif_lpbk->irq);

	return 0;
}

bool memif_lpbk_is_enable(void)
{
	if (cur_memif_lpbk) {
		if (cur_memif_lpbk->enable)
			return true;
	}

	return false;
}

int memif_lpbk_get_irq(void)
{
	if (cur_memif_lpbk)
		return cur_memif_lpbk->irq;

	return 0;
}
#endif

int audio_get_auxadc_value(void)
{
#ifdef _GIT318_PMIC_READY
#ifdef CONFIG_MTK_AUXADC_INTF
	return pmic_get_auxadc_value(AUXADC_LIST_HPOFS_CAL);
#else
	return PMIC_IMM_GetOneChannelValue(PMIC_AUX_CH9, 20, 0);
#endif
#else
	return 0;
#endif
}

/* api for other module */
static int irq_from_ext_module;

bool is_irq_from_ext_module(void)
{
	return irq_from_ext_module > 0 ? true : false;
}

/* VCORE DVFS START*/
static void vcore_dvfs_enable(bool enable, bool reset)
{
#ifdef AUDIO_VCOREFS_SUPPORT
	static DEFINE_MUTEX(vcore_control_mutex);
	static int counter;

	/* pr_warn("%s(), Caller %pf -> %pf\n", __func__, (void *)CALLER_ADDR1, (void *)CALLER_ADDR2); */
	pr_warn("%s(), counter %d, enable %d, reset %d\n", __func__, counter, enable, reset);

	mutex_lock(&vcore_control_mutex);
	if (enable) {
		counter++;
		if (counter == 1) {
			vcorefs_request_dvfs_opp(KIR_AUDIO, OPPI_ULTRA_LOW_PWR);
			pr_warn("%s(), OPPI_ULTRA_LOW_PWR\n", __func__);
		}
	} else {
		counter--;
		if (counter == 0) {
			vcorefs_request_dvfs_opp(KIR_AUDIO, OPPI_UNREQ);
			pr_warn("%s(), OPPI_UNREQ\n", __func__);
		}
	}

	if (reset) {
		counter = 0;
		vcorefs_request_dvfs_opp(KIR_AUDIO, OPPI_UNREQ);
	}
	mutex_unlock(&vcore_control_mutex);
#endif
}

int vcore_dvfs(bool *enable, bool reset)
{
	if (ScreenState == false && reset == false && *enable == false) {
		*enable = true;
		vcore_dvfs_enable(true, false);
	} else if ((ScreenState == true || reset == true) && *enable == true) {
		*enable = false;
		vcore_dvfs_enable(false, false);
	}

	return 0;
}
EXPORT_SYMBOL(vcore_dvfs);


void set_screen_state(bool state)
{
	ScreenState = state;
}
EXPORT_SYMBOL(set_screen_state);
/* VCORE DVFS END*/

struct timeval ext_time;
struct timeval ext_time_prev;
struct timeval ext_time_diff;

static struct timeval ext_diff(struct timeval start, struct timeval end)
{
	struct timeval temp;

	if ((end.tv_usec - start.tv_usec) < 0) {
		temp.tv_sec = end.tv_sec - start.tv_sec - 1;
		temp.tv_usec = 1000000 + end.tv_usec - start.tv_usec;
	} else {
		temp.tv_sec = end.tv_sec - start.tv_sec;
		temp.tv_usec = end.tv_usec - start.tv_usec;
	}
	return temp;
}

int start_ext_sync_signal(void)
{
	unsigned int dl1_state;

	ext_sync_signal_lock();

	dl1_state = GetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_DL1);

	do_gettimeofday(&ext_time);
	ext_time_prev = ext_time;
	pr_warn("%s(), irq_from_ext_module = %d, dl1_state = %d, time = %ld, %ld\n",
		__func__,
		irq_from_ext_module,
		dl1_state,
		ext_time.tv_sec,
		ext_time.tv_usec);

	irq_from_ext_module++;

	if (dl1_state == true)
		Auddrv_DL1_Interrupt_Handler();

	ext_sync_signal_unlock();
	return 0;
}
EXPORT_SYMBOL(start_ext_sync_signal);

int stop_ext_sync_signal(void)
{
	unsigned int dl1_state;

	ext_sync_signal_lock();

	dl1_state = GetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_DL1);

	do_gettimeofday(&ext_time);

	ext_time_diff = ext_diff(ext_time_prev, ext_time);
	ext_time_prev = ext_time;
	pr_warn("%s(), irq_from_ext_module = %d, dl1_state = %d, time diff= %ld, %ld\n",
		__func__,
		irq_from_ext_module,
		dl1_state,
		ext_time_diff.tv_sec,
		ext_time_diff.tv_usec);

	if (irq_from_ext_module > 0) {
		irq_from_ext_module--;
	} else {
		irq_from_ext_module = 0;
		pr_err("%s(), irq_from_ext_module %d <= 0\n",
		       __func__, irq_from_ext_module);
	}

	if (dl1_state == true)
		Auddrv_DL1_Interrupt_Handler();

	ext_sync_signal_unlock();
	return 0;
}
EXPORT_SYMBOL(stop_ext_sync_signal);

int ext_sync_signal(void)
{
	unsigned int dl1_state;

	ext_sync_signal_lock();

	dl1_state = GetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_DL1);

	do_gettimeofday(&ext_time);

	ext_time_diff = ext_diff(ext_time_prev, ext_time);
	ext_time_prev = ext_time;
#if 0
	pr_warn("%s(), irq_from_ext_module = %d, dl1_state = %d, time diff= %ld, %ld\n",
		__func__,
		irq_from_ext_module,
		dl1_state,
		ext_time_diff.tv_sec,
		ext_time_diff.tv_usec);
#endif
	if (irq_from_ext_module && dl1_state == true)
		Auddrv_DL1_Interrupt_Handler();

	ext_sync_signal_unlock();
	return 0;
}
EXPORT_SYMBOL(ext_sync_signal);

static DEFINE_SPINLOCK(ext_sync_lock);
static unsigned long ext_sync_lock_flags;
void ext_sync_signal_lock(void)
{
	spin_lock_irqsave(&ext_sync_lock, ext_sync_lock_flags);
}

void ext_sync_signal_unlock(void)
{
	spin_unlock_irqrestore(&ext_sync_lock, ext_sync_lock_flags);
}

/* api for other modules */
static int request_sram_count;
int mtk_audio_request_sram(dma_addr_t *phys_addr,
			   unsigned char **virt_addr,
			   unsigned int length,
			   void *user)
{
	int ret;

	pr_debug("%s(), user = %p, length = %d, count = %d\n",
		 __func__, user, length, request_sram_count);

	ret = AllocateAudioSram(phys_addr, virt_addr, length, user);
	if (ret) {
		pr_warn("%s(), allocate sram fail, ret %d\n", __func__, ret);
		return ret;
	}

	request_sram_count++;
	AudDrv_Clk_On();

	if (s_afe_platform_ops->set_sram_format != NULL)
		s_afe_platform_ops->set_sram_format(AUDIO_SRAM_FORMAT_32BIT);

	pr_debug("%s(), return 0, count = %d\n", __func__, request_sram_count);
	return 0;
}
EXPORT_SYMBOL(mtk_audio_request_sram);

void mtk_audio_free_sram(void *user)
{
	pr_debug("%s(), user = %p, count = %d\n",
		 __func__, user, request_sram_count);

	freeAudioSram(user);
	request_sram_count--;

	if (request_sram_count == 0) {
		if (s_afe_platform_ops->set_sram_format != NULL)
			s_afe_platform_ops->set_sram_format(AUDIO_SRAM_FORMAT_24BIT);
	}

	AudDrv_Clk_Off();

	pr_debug("%s(), return, count = %d\n", __func__, request_sram_count);
}
EXPORT_SYMBOL(mtk_audio_free_sram);

static snd_pcm_uframes_t get_dlmem_frame_index(struct snd_pcm_substream *substream,
	AFE_MEM_CONTROL_T *afe_mem_control, Soc_Aud_Digital_Block mem_block)
{
	kal_int32 HW_memory_index = 0;
	kal_int32 HW_Cur_ReadIdx = 0;
	unsigned long Frameidx = 0;
	kal_int32 Afe_consumed_bytes = 0;
	AFE_BLOCK_T *Afe_Block = &afe_mem_control->rBlock;
	unsigned long flags;

	if (afe_mem_control == NULL) {
		pr_err("%s err afe_mem_control = NULL", __func__);
		return 0;
	}
	spin_lock_irqsave(&afe_mem_control->substream_lock, flags);
	PRINTK_AUD_DL1(" %s u4DMAReadIdx = 0x%x\n", __func__, Afe_Block->u4DMAReadIdx);

	if (GetMemoryPathEnable(mem_block) == true) {
		switch (mem_block) {
		case Soc_Aud_Digital_Block_MEM_DL1:
			HW_Cur_ReadIdx = Afe_Get_Reg(AFE_DL1_CUR);
			break;
		case Soc_Aud_Digital_Block_MEM_DL2:
			HW_Cur_ReadIdx = Afe_Get_Reg(AFE_DL2_CUR);
			break;
		case Soc_Aud_Digital_Block_MEM_DL3:
			break;
		default:
			pr_err("%s err mem_block = %d", __func__, mem_block);
		}
		if (HW_Cur_ReadIdx == 0) {
			pr_warn("[Auddrv] HW_Cur_ReadIdx ==0\n");
			HW_Cur_ReadIdx = Afe_Block->pucPhysBufAddr;
		}
		HW_memory_index = (HW_Cur_ReadIdx - Afe_Block->pucPhysBufAddr);
		if (HW_memory_index >=  Afe_Block->u4DMAReadIdx)
			Afe_consumed_bytes = HW_memory_index - Afe_Block->u4DMAReadIdx;
		else {
			Afe_consumed_bytes = Afe_Block->u4BufferSize + HW_memory_index -
					     Afe_Block->u4DMAReadIdx;
		}

		Afe_consumed_bytes = word_size_align(Afe_consumed_bytes);

		Afe_Block->u4DataRemained -= Afe_consumed_bytes;
		Afe_Block->u4DMAReadIdx += Afe_consumed_bytes;
		Afe_Block->u4DMAReadIdx %= Afe_Block->u4BufferSize;
		if (Afe_Block->u4DataRemained < 0) {
			pr_warn("[AudioWarn] u4DataRemained=0x%x\n", Afe_Block->u4DataRemained);
		};
		Frameidx = bytes_to_frames(substream->runtime, Afe_Block->u4DMAReadIdx);
	} else {
		Frameidx = bytes_to_frames(substream->runtime, Afe_Block->u4DMAReadIdx);
	}
	spin_unlock_irqrestore(&afe_mem_control->substream_lock, flags);
	return Frameidx;
}

static snd_pcm_uframes_t get_ulmem_frame_index(struct snd_pcm_substream *substream,
	AFE_MEM_CONTROL_T *afe_mem_control, Soc_Aud_Digital_Block mem_block)
{
	kal_int32 HW_memory_index = 0;
	kal_int32 HW_Cur_ReadIdx = 0;
	kal_int32 Hw_Get_bytes = 0;
	bool bIsOverflow = false;
	unsigned long flags;
	AFE_BLOCK_T *UL1_Block = &(afe_mem_control->rBlock);

	PRINTK_AUD_UL1("%s Awb_Block->u4WriteIdx;= 0x%x\n", __func__, UL1_Block->u4WriteIdx);
	mem_blk_spinlock(mem_block);
	spin_lock_irqsave(&afe_mem_control->substream_lock, flags);
	PRINTK_AUD_UL1("mtk_capture_pcm_pointer UL1_Block->u4WriteIdx= 0x%x, u4DataRemained=0x%x\n",
		UL1_Block->u4WriteIdx, UL1_Block->u4DataRemained);

	if (GetMemoryPathEnable(mem_block) == true) {
		switch (mem_block) {
		case Soc_Aud_Digital_Block_MEM_VUL:
			HW_Cur_ReadIdx = word_size_align(Afe_Get_Reg(AFE_VUL_CUR));
			break;
		case Soc_Aud_Digital_Block_MEM_DAI:
			HW_Cur_ReadIdx = word_size_align(Afe_Get_Reg(AFE_DAI_CUR));
			break;
		case Soc_Aud_Digital_Block_MEM_AWB:
			HW_Cur_ReadIdx = word_size_align(Afe_Get_Reg(AFE_AWB_CUR));
			break;
		case Soc_Aud_Digital_Block_MEM_MOD_DAI:
			HW_Cur_ReadIdx = word_size_align(Afe_Get_Reg(AFE_MOD_DAI_CUR));
			break;
		default:
			pr_err("%s error mem_block = %d", __func__, mem_block);
		}
		if (HW_Cur_ReadIdx == 0) {
			PRINTK_AUD_UL1("[Auddrv] %s HW_Cur_ReadIdx ==0\n", __func__);
			HW_Cur_ReadIdx = UL1_Block->pucPhysBufAddr;
		}
		HW_memory_index = (HW_Cur_ReadIdx - UL1_Block->pucPhysBufAddr);

		/* update for data get to hardware */
		Hw_Get_bytes = (HW_Cur_ReadIdx - UL1_Block->pucPhysBufAddr) - UL1_Block->u4WriteIdx;

		if (Hw_Get_bytes < 0)
			Hw_Get_bytes += UL1_Block->u4BufferSize;

		UL1_Block->u4WriteIdx   += Hw_Get_bytes;
		UL1_Block->u4WriteIdx   %= UL1_Block->u4BufferSize;
		UL1_Block->u4DataRemained += Hw_Get_bytes;

		 PRINTK_AUD_UL1("%s ReadIdx=0x%x WriteIdx = 0x%x Remained = 0x%x BufferSize= 0x%x, Get_bytes= 0x%x\n",
			__func__, UL1_Block->u4DMAReadIdx, UL1_Block->u4WriteIdx, UL1_Block->u4DataRemained,
			UL1_Block->u4BufferSize, Hw_Get_bytes);

		/* buffer overflow */
		if (UL1_Block->u4DataRemained > UL1_Block->u4BufferSize) {
			bIsOverflow = true;
			pr_warn("%s buffer overflow u4DMAReadIdx:%x, u4WriteIdx:%x, DataRemained:%x, BufferSize:%x\n",
			       __func__, UL1_Block->u4DMAReadIdx, UL1_Block->u4WriteIdx,
			       UL1_Block->u4DataRemained, UL1_Block->u4BufferSize);
		}

		PRINTK_AUD_UL1("[Auddrv] mtk_capture_pcm_pointer =0x%x HW_memory_index = 0x%x\n",
			HW_Cur_ReadIdx, HW_memory_index);

		spin_unlock_irqrestore(&afe_mem_control->substream_lock, flags);
		mem_blk_spinunlock(mem_block);

		if (bIsOverflow == true)
			return -1;

		return audio_bytes_to_frame(substream, HW_memory_index);
	}

	spin_unlock_irqrestore(&afe_mem_control->substream_lock, flags);
	mem_blk_spinunlock(mem_block);

	return 0;
}

snd_pcm_uframes_t get_mem_frame_index(struct snd_pcm_substream *substream,
	AFE_MEM_CONTROL_T *afe_mem_control, Soc_Aud_Digital_Block mem_block)
{
	switch (mem_block) {
	case Soc_Aud_Digital_Block_MEM_DL1:
	case Soc_Aud_Digital_Block_MEM_DL2:
	case Soc_Aud_Digital_Block_MEM_DL3:
	case Soc_Aud_Digital_Block_MEM_DL1_DATA2:
		return get_dlmem_frame_index(substream, afe_mem_control, mem_block);
	case Soc_Aud_Digital_Block_MEM_VUL:
	case Soc_Aud_Digital_Block_MEM_DAI:
	case Soc_Aud_Digital_Block_MEM_AWB:
	case Soc_Aud_Digital_Block_MEM_MOD_DAI:
	case Soc_Aud_Digital_Block_MEM_VUL_DATA2:
		return get_ulmem_frame_index(substream, afe_mem_control, mem_block);
	default:
		pr_err("%s not support", __func__);
	}
	return 0;
}

void mem_blk_spinlock(Soc_Aud_Digital_Block mem_blk)
{
	switch (mem_blk) {
	case Soc_Aud_Digital_Block_MEM_DL1:
		spin_lock_irqsave(&afe_mem_blk_dl1_lock, spinlock_flags[Soc_Aud_Digital_Block_MEM_DL1]);
		break;
	case Soc_Aud_Digital_Block_MEM_DL2:
		spin_lock_irqsave(&afe_mem_blk_dl2_lock, spinlock_flags[Soc_Aud_Digital_Block_MEM_DL2]);
		break;
	case Soc_Aud_Digital_Block_MEM_DL3:
		spin_lock_irqsave(&afe_mem_blk_dl3_lock, spinlock_flags[Soc_Aud_Digital_Block_MEM_DL3]);
		break;
	case Soc_Aud_Digital_Block_MEM_DL1_DATA2:
		spin_lock_irqsave(&afe_mem_blk_dl1_2_lock, spinlock_flags[Soc_Aud_Digital_Block_MEM_DL1_DATA2]);
		break;
	case Soc_Aud_Digital_Block_MEM_VUL:
		spin_lock_irqsave(&afe_mem_blk_ul1_lock, spinlock_flags[Soc_Aud_Digital_Block_MEM_VUL]);
		break;
	case Soc_Aud_Digital_Block_MEM_DAI:
		spin_lock_irqsave(&afe_mem_blk_dai_lock, spinlock_flags[Soc_Aud_Digital_Block_MEM_DAI]);
		break;
	case Soc_Aud_Digital_Block_MEM_AWB:
		spin_lock_irqsave(&afe_mem_blk_awb_lock, spinlock_flags[Soc_Aud_Digital_Block_MEM_AWB]);
		break;
	case Soc_Aud_Digital_Block_MEM_MOD_DAI:
		spin_lock_irqsave(&afe_mem_blk_moddai_lock, spinlock_flags[Soc_Aud_Digital_Block_MEM_MOD_DAI]);
		break;
	case Soc_Aud_Digital_Block_MEM_VUL_DATA2:
		spin_lock_irqsave(&afe_mem_blk_ul2_lock, spinlock_flags[Soc_Aud_Digital_Block_MEM_VUL_DATA2]);
		break;
	default:
		pr_warn("%s is not support", __func__);
	}
}

void mem_blk_spinunlock(Soc_Aud_Digital_Block mem_blk)
{
	switch (mem_blk) {
	case Soc_Aud_Digital_Block_MEM_DL1:
		spin_unlock_irqrestore(&afe_mem_blk_dl1_lock, spinlock_flags[Soc_Aud_Digital_Block_MEM_DL1]);
		break;
	case Soc_Aud_Digital_Block_MEM_DL2:
		spin_unlock_irqrestore(&afe_mem_blk_dl2_lock, spinlock_flags[Soc_Aud_Digital_Block_MEM_DL2]);
		break;
	case Soc_Aud_Digital_Block_MEM_DL3:
		spin_unlock_irqrestore(&afe_mem_blk_dl3_lock, spinlock_flags[Soc_Aud_Digital_Block_MEM_DL3]);
		break;
	case Soc_Aud_Digital_Block_MEM_DL1_DATA2:
		spin_unlock_irqrestore(&afe_mem_blk_dl1_2_lock, spinlock_flags[Soc_Aud_Digital_Block_MEM_DL1_DATA2]);
		break;
	case Soc_Aud_Digital_Block_MEM_VUL:
		spin_unlock_irqrestore(&afe_mem_blk_ul1_lock, spinlock_flags[Soc_Aud_Digital_Block_MEM_VUL]);
		break;
	case Soc_Aud_Digital_Block_MEM_DAI:
		spin_unlock_irqrestore(&afe_mem_blk_dai_lock, spinlock_flags[Soc_Aud_Digital_Block_MEM_DAI]);
		break;
	case Soc_Aud_Digital_Block_MEM_AWB:
		spin_unlock_irqrestore(&afe_mem_blk_awb_lock, spinlock_flags[Soc_Aud_Digital_Block_MEM_AWB]);
		break;
	case Soc_Aud_Digital_Block_MEM_MOD_DAI:
		spin_unlock_irqrestore(&afe_mem_blk_moddai_lock, spinlock_flags[Soc_Aud_Digital_Block_MEM_MOD_DAI]);
		break;
	case Soc_Aud_Digital_Block_MEM_VUL_DATA2:
		spin_unlock_irqrestore(&afe_mem_blk_ul2_lock, spinlock_flags[Soc_Aud_Digital_Block_MEM_VUL_DATA2]);
		break;
	default:
		pr_warn("%s is not support", __func__);
	}

}


static int mtk_mem_dlblk_copy(struct snd_pcm_substream *substream,
				int channel, snd_pcm_uframes_t pos, void __user *dst, snd_pcm_uframes_t count,
				AFE_MEM_CONTROL_T *pMemControl, Soc_Aud_Digital_Block mem_blk)
{
	AFE_BLOCK_T  *Afe_Block = NULL;
	int copy_size = 0, Afe_WriteIdx_tmp;
	char *data_w_ptr = (char *)dst;

	PRINTK_AUD_DL1("mtk_pcm_copy pos = %lu count = %lu\n", pos, count);

	/* get total bytes to copy */
	count = audio_frame_to_bytes(substream, count);

	/* check which memif nned to be write */
	Afe_Block = &pMemControl->rBlock;

	/* handle for buffer management */

	PRINTK_AUD_DL1(" WriteIdx=0x%x, ReadIdx=0x%x, DataRemained=0x%x\n",
		       Afe_Block->u4WriteIdx, Afe_Block->u4DMAReadIdx, Afe_Block->u4DataRemained);

	if (Afe_Block->u4BufferSize == 0) {
		pr_err(" u4BufferSize=0 Error");
		return 0;
	}

	if (mem_blk == Soc_Aud_Digital_Block_MEM_DL1)
		AudDrv_checkDLISRStatus();

	mem_blk_spinlock(mem_blk);
	/* free space of the buffer */
	copy_size = Afe_Block->u4BufferSize - Afe_Block->u4DataRemained;
	mem_blk_spinunlock(mem_blk);
	if (count <=  copy_size) {
		if (copy_size < 0)
			copy_size = 0;
		else
			copy_size = count;
	}

	copy_size = word_size_align(copy_size);
	PRINTK_AUD_DL1("copy_size=0x%x, count=0x%x\n", copy_size, (unsigned int)count);

	if (copy_size != 0) {
		mem_blk_spinlock(mem_blk);
		Afe_WriteIdx_tmp = Afe_Block->u4WriteIdx;
		mem_blk_spinunlock(mem_blk);

		if (Afe_WriteIdx_tmp + copy_size < Afe_Block->u4BufferSize) { /* copy once */
			if (!access_ok(VERIFY_READ, data_w_ptr, copy_size)) {
				pr_warn("0ptr invalid data_w_ptr=%p, size=%d", data_w_ptr, copy_size);
				pr_warn("u4BufferSize=%d, u4DataRemained=%d", Afe_Block->u4BufferSize,
					      Afe_Block->u4DataRemained);
			} else {

				PRINTK_AUD_DL1("memcpy Idx= %p data_w_ptr = %p copy_size = 0x%x\n",
					       Afe_Block->pucVirtBufAddr + Afe_WriteIdx_tmp, data_w_ptr, copy_size);
				if (copy_from_user((Afe_Block->pucVirtBufAddr + Afe_WriteIdx_tmp), data_w_ptr,
						   copy_size)) {
					pr_warn("[AudioWarn] Fail copy from user\n");
					return -1;
				}
			}

			mem_blk_spinlock(mem_blk);
			Afe_Block->u4DataRemained += copy_size;
			Afe_Block->u4WriteIdx = Afe_WriteIdx_tmp + copy_size;
			Afe_Block->u4WriteIdx %= Afe_Block->u4BufferSize;
			mem_blk_spinunlock(mem_blk);
			data_w_ptr += copy_size;
			count -= copy_size;

			PRINTK_AUD_DL1(" finish1, copy_size:%x, WriteIdx:%x, ReadIdx=%x, Remained:%x, count=%x \r\n",
				       copy_size, Afe_Block->u4WriteIdx, Afe_Block->u4DMAReadIdx,
				       Afe_Block->u4DataRemained, (unsigned int)count);

		} else { /* copy twice */
			kal_uint32 size_1 = 0, size_2 = 0;

			size_1 = word_size_align((Afe_Block->u4BufferSize - Afe_WriteIdx_tmp));
			size_2 = word_size_align((copy_size - size_1));
			PRINTK_AUD_DL1("size_1=0x%x, size_2=0x%x\n", size_1, size_2);
			if (!access_ok(VERIFY_READ, data_w_ptr, size_1)) {
				pr_warn(" 1ptr invalid data_w_ptr=%p, size_1=%d", data_w_ptr, size_1);
				pr_warn(" u4BufferSize=%d, u4DataRemained=%d", Afe_Block->u4BufferSize,
				       Afe_Block->u4DataRemained);
			} else {

				PRINTK_AUD_DL1("mcmcpy Idx= %p data_w_ptr = %p size_1 = %x\n",
					       Afe_Block->pucVirtBufAddr + Afe_WriteIdx_tmp, data_w_ptr, size_1);
				if ((copy_from_user((Afe_Block->pucVirtBufAddr + Afe_WriteIdx_tmp), data_w_ptr,
						    size_1))) {
					pr_warn(" Fail 1 copy from user");
					return -1;
				}
			}
			mem_blk_spinlock(mem_blk);
			Afe_Block->u4DataRemained += size_1;
			Afe_Block->u4WriteIdx = Afe_WriteIdx_tmp + size_1;
			Afe_Block->u4WriteIdx %= Afe_Block->u4BufferSize;
			Afe_WriteIdx_tmp = Afe_Block->u4WriteIdx;
			mem_blk_spinunlock(mem_blk);

			if (!access_ok(VERIFY_READ, data_w_ptr + size_1, size_2)) {
				pr_warn("2ptr invalid data_w_ptr=%p, size_1=%d, size_2=%d", data_w_ptr,
					      size_1, size_2);
				pr_warn("u4BufferSize=%d, u4DataRemained=%d", Afe_Block->u4BufferSize,
					      Afe_Block->u4DataRemained);
			} else {

				PRINTK_AUD_DL1("mcmcpy Idx= %p data_w_ptr+size_1 = %p size_2 = %x\n",
					       Afe_Block->pucVirtBufAddr + Afe_WriteIdx_tmp,
					       data_w_ptr + size_1, size_2);
				if ((copy_from_user((Afe_Block->pucVirtBufAddr + Afe_WriteIdx_tmp),
						    (data_w_ptr + size_1), size_2))) {
					pr_warn("AudDrv_write Fail 2  copy from user");
					return -1;
				}
			}
			mem_blk_spinlock(mem_blk);
			Afe_Block->u4DataRemained += size_2;
			Afe_Block->u4WriteIdx = Afe_WriteIdx_tmp + size_2;
			Afe_Block->u4WriteIdx %= Afe_Block->u4BufferSize;
			mem_blk_spinunlock(mem_blk);

			count -= copy_size;
			data_w_ptr += copy_size;

			PRINTK_AUD_DL1(" finish2, copy size:%x, WriteIdx:%x,ReadIdx=%x DataRemained:%x \r\n",
				       copy_size, Afe_Block->u4WriteIdx, Afe_Block->u4DMAReadIdx,
				       Afe_Block->u4DataRemained);
		}
	}
	PRINTK_AUD_DL1("pcm_copy return\n");
	return 0;
}

static bool CheckNullPointer(void *pointer)
{
	if (pointer == NULL) {
		pr_warn("CheckNullPointer pointer = NULL");
		return true;
	}
	return false;
}

static int mtk_mem_ulblk_copy(struct snd_pcm_substream *substream,
				int channel, snd_pcm_uframes_t pos, void __user *dst, snd_pcm_uframes_t count,
				AFE_MEM_CONTROL_T *pMemControl, Soc_Aud_Digital_Block mem_blk)
{
	AFE_MEM_CONTROL_T *pVUL_MEM_ConTrol = NULL;
	AFE_BLOCK_T  *Vul_Block = NULL;
	char *Read_Data_Ptr = (char *)dst;
	ssize_t DMA_Read_Ptr = 0, read_size = 0, read_count = 0;
	 struct snd_pcm_runtime *runtime = substream->runtime;

	PRINTK_AUD_UL1("mtk_capture_pcm_copy pos = %lucount = %lu\n ", pos, count);
	/* get total bytes to copy */
	count = word_size_align(audio_frame_to_bytes(substream, count));

	/* check which memif nned to be write */
	pVUL_MEM_ConTrol = pMemControl;
	Vul_Block = &(pVUL_MEM_ConTrol->rBlock);

	if (pVUL_MEM_ConTrol == NULL) {
		pr_err("cannot find MEM control !!!!!!!\n");
		msleep(50);
		return 0;
	}

	if (Vul_Block->u4BufferSize <= 0) {
		msleep(50);
		pr_err("Vul_Block->u4BufferSize <= 0  =%d\n", Vul_Block->u4BufferSize);
		return 0;
	}

	if (CheckNullPointer((void *)Vul_Block->pucVirtBufAddr)) {
		pr_err("CheckNullPointer  pucVirtBufAddr = %p\n", Vul_Block->pucVirtBufAddr);
		return 0;
	}

	mem_blk_spinlock(mem_blk);
	if (Vul_Block->u4DataRemained >  Vul_Block->u4BufferSize) {
		PRINTK_AUD_UL1("%s u4DataRemained=%x > u4BufferSize=%x",
			__func__, Vul_Block->u4DataRemained, Vul_Block->u4BufferSize);
		Vul_Block->u4DataRemained = 0;
		Vul_Block->u4DMAReadIdx   = Vul_Block->u4WriteIdx;
	}

	if (count >  Vul_Block->u4DataRemained)
		read_size = Vul_Block->u4DataRemained;
	else
		read_size = count;

	DMA_Read_Ptr = Vul_Block->u4DMAReadIdx;
	mem_blk_spinunlock(mem_blk);

	PRINTK_AUD_UL1("%s finish0, read_count:%x, read_size:%x, Remained:%x, ReadIdx:0x%x, WriteIdx:%x \r\n",
		       __func__, (unsigned int)read_count,
		       (unsigned int)read_size, Vul_Block->u4DataRemained,
		       Vul_Block->u4DMAReadIdx, Vul_Block->u4WriteIdx);

	if (DMA_Read_Ptr + read_size < Vul_Block->u4BufferSize) {
		if (DMA_Read_Ptr != Vul_Block->u4DMAReadIdx) {
			pr_warn("%s 1, read_size:%zu, Remained:%x, Ptr:%zu, DMAReadIdx:%x \r\n",
			       __func__, read_size, Vul_Block->u4DataRemained,
			       DMA_Read_Ptr, Vul_Block->u4DMAReadIdx);
		}

		if (copy_to_user((void __user *)Read_Data_Ptr,
			(Vul_Block->pucVirtBufAddr + DMA_Read_Ptr), read_size)) {

			pr_err("%s Fail 1 copy to user Ptr:%p, Addr:%p, ReadIdx:0x%x, Read_Ptr:%zu,size:%zu",
				__func__, Read_Data_Ptr, Vul_Block->pucVirtBufAddr,
				Vul_Block->u4DMAReadIdx, DMA_Read_Ptr, read_size);
			return 0;
		}

		read_count += read_size;
		mem_blk_spinlock(mem_blk);
		Vul_Block->u4DataRemained -= read_size;
		Vul_Block->u4DMAReadIdx += read_size;
		Vul_Block->u4DMAReadIdx %= Vul_Block->u4BufferSize;
		DMA_Read_Ptr = Vul_Block->u4DMAReadIdx;
		mem_blk_spinunlock(mem_blk);

		Read_Data_Ptr += read_size;
		count -= read_size;

		PRINTK_AUD_UL1("%s finish1, copy size:%x, ReadIdx:0x%x, WriteIdx:%x, Remained:%x \r\n",
			       __func__, (unsigned int)read_size,
			       Vul_Block->u4DMAReadIdx, Vul_Block->u4WriteIdx,
			       Vul_Block->u4DataRemained);
	}

	else {
		uint32 size_1 = Vul_Block->u4BufferSize - DMA_Read_Ptr;
		uint32 size_2 = read_size - size_1;

		if (DMA_Read_Ptr != Vul_Block->u4DMAReadIdx) {

			pr_warn("%s 2, read_size1:%x, Remained:%x, Read_Ptr:%zu, ReadIdx:%x \r\n",
			       __func__, size_1, Vul_Block->u4DataRemained,
			       DMA_Read_Ptr, Vul_Block->u4DMAReadIdx);
		}
		if (copy_to_user((void __user *)Read_Data_Ptr,
			(Vul_Block->pucVirtBufAddr + DMA_Read_Ptr), (unsigned int)size_1)) {

			pr_err("%s Fail 2 copy to user Ptr:%p, Addr:%p, ReadIdx:0x%x, Read_Ptr:%zu,read_size:%zu",
			       __func__, Read_Data_Ptr, Vul_Block->pucVirtBufAddr,
			       Vul_Block->u4DMAReadIdx, DMA_Read_Ptr, read_size);
			return 0;
		}

		read_count += size_1;
		mem_blk_spinlock(mem_blk);
		Vul_Block->u4DataRemained -= size_1;
		Vul_Block->u4DMAReadIdx += size_1;
		Vul_Block->u4DMAReadIdx %= Vul_Block->u4BufferSize;
		DMA_Read_Ptr = Vul_Block->u4DMAReadIdx;
		mem_blk_spinunlock(mem_blk);

		PRINTK_AUD_UL1("%s finish2, copy size_1:%x, ReadIdx:0x%x, WriteIdx:0x%x, Remained:%x \r\n",
			       __func__, size_1, Vul_Block->u4DMAReadIdx,
			       Vul_Block->u4WriteIdx, Vul_Block->u4DataRemained);

		if (DMA_Read_Ptr != Vul_Block->u4DMAReadIdx) {

			pr_warn("%s 3, read_size2:%x, Remained:%x, DMA_Read_Ptr:%zu, DMAReadIdx:%x \r\n",
			       __func__, size_2, Vul_Block->u4DataRemained,
			       DMA_Read_Ptr, Vul_Block->u4DMAReadIdx);
		}
		if (copy_to_user((void __user *)(Read_Data_Ptr + size_1),
			(Vul_Block->pucVirtBufAddr + DMA_Read_Ptr), size_2)) {

			pr_err("%s Fail 3 copy to user Ptr:%p, Addr:%p, ReadIdx:0x%x , Read_Ptr:%zu, read_size:%zu",
				__func__, Read_Data_Ptr, Vul_Block->pucVirtBufAddr,
				Vul_Block->u4DMAReadIdx, DMA_Read_Ptr, read_size);
			return bytes_to_frames(runtime, read_count);
		}

		read_count += size_2;
		mem_blk_spinlock(mem_blk);
		Vul_Block->u4DataRemained -= size_2;
		Vul_Block->u4DMAReadIdx += size_2;
		DMA_Read_Ptr = Vul_Block->u4DMAReadIdx;
		mem_blk_spinunlock(mem_blk);
		count -= read_size;
		Read_Data_Ptr += read_size;

		PRINTK_AUD_UL1("%s finish3, copy size_2:%x, u4DMAReadIdx:0x%x, u4WriteIdx:0x%x u4DataRemained:%x \r\n",
			       __func__, size_2, Vul_Block->u4DMAReadIdx,
			       Vul_Block->u4WriteIdx, Vul_Block->u4DataRemained);
	}

	return bytes_to_frames(runtime, read_count);
}

int mtk_memblk_copy(struct snd_pcm_substream *substream,
				int channel, snd_pcm_uframes_t pos, void __user *dst, snd_pcm_uframes_t count,
				AFE_MEM_CONTROL_T *pMemControl, Soc_Aud_Digital_Block mem_blk)
{
	if (pMemControl == NULL)
		return 0;

	switch (mem_blk) {
	case Soc_Aud_Digital_Block_MEM_DL1:
	case Soc_Aud_Digital_Block_MEM_DL2:
	case Soc_Aud_Digital_Block_MEM_DL3:
	case Soc_Aud_Digital_Block_MEM_DL1_DATA2:
		mtk_mem_dlblk_copy(substream, channel, pos, dst, count, pMemControl, mem_blk);
		break;
	case Soc_Aud_Digital_Block_MEM_VUL:
	case Soc_Aud_Digital_Block_MEM_DAI:
	case Soc_Aud_Digital_Block_MEM_AWB:
	case Soc_Aud_Digital_Block_MEM_MOD_DAI:
	case Soc_Aud_Digital_Block_MEM_VUL_DATA2:
		mtk_mem_ulblk_copy(substream, channel, pos, dst, count, pMemControl, mem_blk);
		break;
	default:
		pr_err("%s not support", __func__);
	}
	return 0;
}

int set_memif_addr(int mem_blk, dma_addr_t addr, size_t size)
{
	int ret;

	/* by platform to implement*/
	if (s_mem_blk_ops != NULL &&
	    s_mem_blk_ops->set_chip_memif_addr != NULL) {
		ret = s_mem_blk_ops->set_chip_memif_addr(mem_blk,
							 addr,
							 size);
		return ret;
	}

	/* set address hardware , default implemant*/
	switch (mem_blk) {
	case Soc_Aud_Digital_Block_MEM_DL1:
		Afe_Set_Reg(AFE_DL1_BASE, addr, 0xffffffff);
		Afe_Set_Reg(AFE_DL1_END, addr + (size - 1), 0xffffffff);
		break;
	case Soc_Aud_Digital_Block_MEM_DL2:
		Afe_Set_Reg(AFE_DL2_BASE, addr, 0xffffffff);
		Afe_Set_Reg(AFE_DL2_END, addr + (size - 1), 0xffffffff);
		break;
	case Soc_Aud_Digital_Block_MEM_VUL:
		Afe_Set_Reg(AFE_VUL_BASE, addr, 0xffffffff);
		Afe_Set_Reg(AFE_VUL_END, addr + (size - 1), 0xffffffff);
		break;
	case Soc_Aud_Digital_Block_MEM_DAI:
		Afe_Set_Reg(AFE_DAI_BASE, addr, 0xffffffff);
		Afe_Set_Reg(AFE_DAI_END, addr + (size - 1), 0xffffffff);
		break;
	case Soc_Aud_Digital_Block_MEM_MOD_DAI:
		Afe_Set_Reg(AFE_MOD_DAI_BASE, addr, 0xffffffff);
		Afe_Set_Reg(AFE_MOD_DAI_END, addr + (size - 1), 0xffffffff);
		break;
	case Soc_Aud_Digital_Block_MEM_VUL_DATA2:
		Afe_Set_Reg(AFE_VUL_D2_BASE, addr, 0xffffffff);
		Afe_Set_Reg(AFE_VUL_D2_END, addr + (size - 1), 0xffffffff);
		break;
	case Soc_Aud_Digital_Block_MEM_AWB:
		Afe_Set_Reg(AFE_AWB_BASE, addr, 0xffffffff);
		Afe_Set_Reg(AFE_AWB_END, addr + (size - 1), 0xffffffff);
		break;
	case Soc_Aud_Digital_Block_MEM_DL1_DATA2:
	case Soc_Aud_Digital_Block_MEM_DL3:
	case Soc_Aud_Digital_Block_MEM_HDMI:
	default:
		pr_warn("%s not suuport mem_blk = %d", __func__, mem_blk);
		return -EINVAL;
	}
	return 0;
}

int set_mem_block(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *hw_params,
	AFE_MEM_CONTROL_T *pMemControl, Soc_Aud_Digital_Block mem_blk)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	AFE_BLOCK_T *pblock = &pMemControl->rBlock;

	pblock->pucPhysBufAddr = runtime->dma_addr;
	pblock->pucVirtBufAddr = runtime->dma_area;
	pblock->u4BufferSize = runtime->dma_bytes;
	pblock->u4SampleNumMask = 0x001f;	/* 32 byte align */
	pblock->u4WriteIdx = 0;
	pblock->u4DMAReadIdx = 0;
	pblock->u4DataRemained = 0;
	pblock->u4fsyncflag = false;
	pblock->uResetFlag = true;
	/*
	*pr_debug("%s u4BufferSize = %d pucVirtBufAddr = %p pucPhysBufAddr = 0x%x\n",
	*	__func__, pblock->u4BufferSize, pblock->pucVirtBufAddr, pblock->pucPhysBufAddr);
	*/

	set_memif_addr(mem_blk, pblock->pucPhysBufAddr, pblock->u4BufferSize);

	memset_io((void *)pblock->pucVirtBufAddr, 0, pblock->u4BufferSize);
	return 0;
}

void trigger_mtkaif_calibration(void)
{
	/* by platform to implement*/
	if (s_afe_platform_ops->trigger_mtkaif_calibration != NULL) {
		s_afe_platform_ops->trigger_mtkaif_calibration();
		return;
	}

	pr_warn("%s(), not support this platform", __func__);
}

bool handle_suspend(bool suspend)
{
	bool ret = false;

	if (s_afe_platform_ops->handle_suspend != NULL) {
		s_afe_platform_ops->handle_suspend(suspend);
		ret = true;
	}
	return ret;
}

void set_mem_blk_ops(struct mtk_mem_blk_ops *ops)
{
	s_mem_blk_ops = ops;
}

void set_afe_platform_ops(struct mtk_afe_platform_ops *ops)
{
	s_afe_platform_ops = ops;
}

struct mtk_afe_platform_ops *get_afe_platform_ops(void)
{
	return s_afe_platform_ops;
}

/* low latency debug */
int get_LowLatencyDebug(void)
{
	return LowLatencyDebug;
}

void set_LowLatencyDebug(uint32 bFlag)
{
	LowLatencyDebug = bFlag;
	pr_warn("%s LowLatencyDebug = %d\n", __func__, LowLatencyDebug);
}

