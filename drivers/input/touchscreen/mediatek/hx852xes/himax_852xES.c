/* Himax Android Driver Sample Code for HMX852xES chipset
*
* Copyright (C) 2014 Himax Corporation.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*/

#include "himax_852xES.h"

/* 20160119 reboot every 30s issue */
#include "../../../../watchdog/mediatek/wdt/mt6757/mtk_wdt.h"

/* CY33 on 20160119 reboot every 30s issue */
#include "../../../../misc/mediatek/include/mt-plat/mt6757/include/mach/wd_api.h"

/* CY33 on 20160517 
#include <linux/irq.h>
#include "../kernel/irq/internals.h"
*/
extern void mt_eint_dump_status(unsigned int eint); //20160419 add mtklog

#define SUPPORT_FINGER_DATA_CHECKSUM 0x0F
#define TS_WAKE_LOCK_TIMEOUT		(2 * HZ)

#if defined(CONFIG_TOUCHSCREEN_PROXIMITY)
/* Proximity */
static u8 proximity_flag = 0;
static u8 g_proximity_en = 0;
#endif

#if defined(HX_AUTO_UPDATE_FW)||defined(HX_AUTO_UPDATE_CONFIG)
	static unsigned char i_CTPM_FW[]=
	{
		/* #include "CEI_CT41_CAT_C04_20170303.i" */
		/* #include "CEI_CT41_E1909_CAT_C04_20170315.i" */ /* enable smart wake up */
		#include "CEI_CT41_E190C_CAT_C05_20170329.i"	/* add glove mode for enable/disable cmd 0x8F */
	};

	static unsigned char i_CTPM_FW_ELLY[]=
	{
		/* #include "CEI_CT41_E190C_C22_ELLY_20170407.i" */
		/* TP Frequency : 202.9K for avoid LCM noise */
		/* #include "CEI_CT41_E190C_C23_ELLY_20170418.i" */
		/* Add Auto detect for glove mode */
		/* #include "CEI_CT41_E190C_C24_ELLY_20170419.i" */
		/* Add Water Proof function */
		/* #include "CEI_CT41_E190D_C25_ELLY_20170607.i" */
		/* Adjust Frequency Hopping Parameter to improve noise */
		/* #include "CEI_CT41_E190D_C26_ELLY_20170623.i" */
		/* Add handshaking workaround to prevent TP no function */
		/* #include "CEI_CT41_E190E_C27_ELLY_20170628.i" */
		/* Fine tune tapping parameter to improve fast tapping */
		/* #include "CEI_CT41_E190E_C29_ELLY_20170720.i" */
		/* for issue 1418, protect mapping and config register */
		/* #include "CEI_CT41_E190F_C29_ELLY_20170726.i" */
		#include "CEI_CT41_E1910_C2A_ELLY_20170929.i"
	};

#endif

#ifdef HX_ESD_WORKAROUND
	static u8 ESD_recovery_count = 0;
#endif

#ifdef HX_TPID_CHECK
	static int CTP_ID_g = -1;
#endif

#ifdef PS_DISABLE_CALLPHONE_BUTTON_UI
static bool disable_pull_status_bar = false;
static bool touch_on_statusbar_callphoneUI = false;

#ifdef PS_DISABLE_SPECIFIC_AREA
static bool touch_on_UIbar_callphoneUI = false;
#endif
#endif

#ifdef MTK_DMA
extern u8 *gpDMABuf_va;
extern u8 *gpDMABuf_pa;
#endif
//Touch panel Recovery 
#ifdef HX_TOUCH_REC	
static int checktouch=3; //times of recover
static int check_touchData(void);
static void TP_REC(void);
#endif

/* for CY33 start */
unsigned int himax_touch_irq = 0;
static int fw_status = 0; //update fw_statu 20160119
static int FW_H,FW_L,FW_CON;//20160302 fw_version update

extern int mtk_wdt_enable (enum wk_wdt_en en);//20160119 reboot every 30s issue
static int int_status = 0; //int_status 20160425
/* for CY33 end */
//static int		tpd_keys_local[HX_KEY_MAX_COUNT] = HX_KEY_ARRAY; // for Virtual key array
static unsigned char	IC_CHECKSUM               = 0;
static unsigned char	IC_TYPE                   = 0;

static int		HX_TOUCH_INFO_POINT_CNT   = 0;

static int		HX_RX_NUM                 = 0;
static int		HX_TX_NUM                 = 0;
static int		HX_BT_NUM                 = 0;
static int		HX_X_RES                  = 0;
static int		HX_Y_RES                  = 0;
static int		HX_MAX_PT                 = 0;
static bool		HX_XY_REVERSE             = false;
static bool		HX_INT_IS_EDGE            = false;

static unsigned int		FW_VER_MAJ_FLASH_ADDR;
static unsigned int 	FW_VER_MAJ_FLASH_LENG;
static unsigned int 	FW_VER_MIN_FLASH_ADDR;
static unsigned int 	FW_VER_MIN_FLASH_LENG;

static unsigned int 	FW_CFG_VER_FLASH_ADDR;

static unsigned int 	CFG_VER_MAJ_FLASH_ADDR;
static unsigned int 	CFG_VER_MAJ_FLASH_LENG;
static unsigned int 	CFG_VER_MIN_FLASH_ADDR;
static unsigned int 	CFG_VER_MIN_FLASH_LENG;

static uint8_t 	vk_press = 0x00;
static uint8_t 	AA_press = 0x00;
static uint8_t	IC_STATUS_CHECK	= 0xAA;
static uint8_t 	EN_NoiseFilter = 0x00;
static uint8_t	Last_EN_NoiseFilter = 0x00;
static uint8_t  HX_DRIVER_PROBE_Fial = 0;
static int	hx_point_num	= 0;																	// for himax_ts_work_func use
static int	p_point_num	= 0xFFFF;
static int	tpd_key	   	= 0x00;
static int	tpd_key_old	= 0x00;

static bool	config_load		= false;
static struct himax_config *config_selected = NULL;

static int iref_number = 11;
static bool iref_found = false;

/* for CY33 start */
static int self_test_inter_flag = 0; //20160216 Speed Fw
/* 20160506
#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void himax_ts_early_suspend(struct early_suspend *h);
static void himax_ts_late_resume(struct early_suspend *h);
#endif
*/
/* for CY33 end */
#ifdef CONFIG_OF
#if defined(HX_LOADIN_CONFIG)||defined(HX_AUTO_UPDATE_CONFIG)
static int himax_parse_config(struct himax_ts_data *ts, struct himax_config *pdata);
#endif
#endif

#ifdef CONFIG_OF
MODULE_DEVICE_TABLE(of, himax_match_table);

static struct of_device_id himax_match_table[] = {
	{.compatible = "mediatek,cap_touch" },	
	{},
};

#else
#define himax_match_table NULL
#endif

static int of_get_himax85xx_platform_data(struct device *dev)
{
	if (dev->of_node) {
		const struct of_device_id *match;

		match = of_match_device(of_match_ptr(himax_match_table), dev);
		if (!match) {
			E("No device match found\n");
			return -ENODEV;
		}
	}

	return 0;
}

static int himax_hand_shaking(void)    //0:Running, 1:Stop, 2:I2C Fail
{
	int ret, result;
	uint8_t hw_reset_check[1];
	uint8_t hw_reset_check_2[1];
	uint8_t buf0[2];

	memset(hw_reset_check, 0x00, sizeof(hw_reset_check));
	memset(hw_reset_check_2, 0x00, sizeof(hw_reset_check_2));

	buf0[0] = 0xF2;
	if (IC_STATUS_CHECK == 0xAA) {
		buf0[1] = 0xAA;
		IC_STATUS_CHECK = 0x55;
	} else {
		buf0[1] = 0x55;
		IC_STATUS_CHECK = 0xAA;
	}

	ret = i2c_himax_master_write(private_ts->client, buf0, 2, DEFAULT_RETRY_CNT);
	if (ret < 0) {
		E("[Himax]:write 0xF2 failed line: %d \n",__LINE__);
		goto work_func_send_i2c_msg_fail;
	}
	msleep(50); 
  	
	buf0[0] = 0xF2;
	buf0[1] = 0x00;
	ret = i2c_himax_master_write(private_ts->client, buf0, 2, DEFAULT_RETRY_CNT);
	if (ret < 0) {
		E("[Himax]:write 0x92 failed line: %d \n",__LINE__);
		goto work_func_send_i2c_msg_fail;
	}
	msleep(2);
  	
	ret = i2c_himax_read(private_ts->client, 0x90, hw_reset_check, 1, DEFAULT_RETRY_CNT);
	if (ret < 0) {
		E("[Himax]:i2c_himax_read 0x90 failed line: %d \n",__LINE__);
		goto work_func_send_i2c_msg_fail;
	}
	
	if ((IC_STATUS_CHECK != hw_reset_check[0])) {
		msleep(2);
		ret = i2c_himax_read(private_ts->client, 0x90, hw_reset_check_2, 1, DEFAULT_RETRY_CNT);
		if (ret < 0) {
			E("[Himax]:i2c_himax_read 0x90 failed line: %d \n",__LINE__);
			goto work_func_send_i2c_msg_fail;
		}
	
		if (hw_reset_check[0] == hw_reset_check_2[0]) {
			result = 1; 
		} else {
			result = 0; 
		}
	} else {
		result = 0; 
	}
	
	return result;

work_func_send_i2c_msg_fail:
	return 2;
}

static int himax_ManualMode(int enter)
{
	uint8_t cmd[2];
	cmd[0] = enter;
	if ( i2c_himax_write(private_ts->client, HX_CMD_MANUALMODE ,&cmd[0], 1, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}
	return 0;
}

static int himax_FlashMode(int enter)
{
	uint8_t cmd[2];
	cmd[0] = enter;
	if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE ,&cmd[0], 1, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}
	return 0;
}

static int himax_lock_flash(int enable)
{
	uint8_t cmd[5];

	if (i2c_himax_write(private_ts->client, 0xAA ,&cmd[0], 0, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
			return 0;
	}
	
	/* lock sequence start */
	cmd[0] = 0x01;cmd[1] = 0x00;cmd[2] = 0x06;
	if (i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE ,&cmd[0], 3, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}

	cmd[0] = 0x03;cmd[1] = 0x00;cmd[2] = 0x00;
	if (i2c_himax_write(private_ts->client, HX_CMD_FLASH_SET_ADDRESS ,&cmd[0], 3, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}
	
	if(enable!=0){
			cmd[0] = 0x63;cmd[1] = 0x02;cmd[2] = 0x70;cmd[3] = 0x03;
		}
	else{
			cmd[0] = 0x63;cmd[1] = 0x02;cmd[2] = 0x30;cmd[3] = 0x00;
		}
	
	if (i2c_himax_write(private_ts->client, HX_CMD_FLASH_WRITE_REGISTER ,&cmd[0], 4, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}

	if (i2c_himax_write_command(private_ts->client, HX_CMD_4A, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}
	msleep(50);

	if (i2c_himax_write(private_ts->client, 0xA9 ,&cmd[0], 0, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}
	
	return 0;
	/* lock sequence stop */
}


static void himax_changeIref(int selected_iref){

	unsigned char temp_iref[16][2] = {	{0x00,0x00},{0x00,0x00},{0x00,0x00},{0x00,0x00},
										{0x00,0x00},{0x00,0x00},{0x00,0x00},{0x00,0x00},
										{0x00,0x00},{0x00,0x00},{0x00,0x00},{0x00,0x00},
										{0x00,0x00},{0x00,0x00},{0x00,0x00},{0x00,0x00}};  													
	uint8_t cmd[10]; 
	int i = 0;
	int j = 0;

	I("%s: start to check iref,iref number = %d\n",__func__,selected_iref);

	if (i2c_himax_write(private_ts->client, 0xAA ,&cmd[0], 0, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return;
	}	

	for(i=0; i<16; i++){
		for(j=0; j<2; j++){
			if(selected_iref == 1){
				temp_iref[i][j] = E_IrefTable_1[i][j];
			}
					else if(selected_iref == 2){
				temp_iref[i][j] = E_IrefTable_2[i][j];
			}
			else if(selected_iref == 3){
				temp_iref[i][j] = E_IrefTable_3[i][j];
			}
			else if(selected_iref == 4){
				temp_iref[i][j] = E_IrefTable_4[i][j];
			}
			else if(selected_iref == 5){
				temp_iref[i][j] = E_IrefTable_5[i][j];
			}
			else if(selected_iref == 6){
				temp_iref[i][j] = E_IrefTable_6[i][j];
			}
			else if(selected_iref == 7){
				temp_iref[i][j] = E_IrefTable_7[i][j];
			}
		}
	}
	
	if(!iref_found){
		//Read Iref
		//Register 0x43
		cmd[0] = 0x01;
		cmd[1] = 0x00;
		cmd[2] = 0x0A;
		if( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE ,&cmd[0], 3, 3) < 0){
			E("%s: i2c access fail!\n", __func__);
			return;
		}
		
		//Register 0x44
		cmd[0] = 0x00;
		cmd[1] = 0x00;
		cmd[2] = 0x00;
		if( i2c_himax_write(private_ts->client, HX_CMD_FLASH_SET_ADDRESS ,&cmd[0], 3, 3) < 0){
			E("%s: i2c access fail!\n", __func__);
			return ;
		}														
	 
		//Register 0x46
		if( i2c_himax_write(private_ts->client, 0x46 ,&cmd[0], 0, 3) < 0){
			E("%s: i2c access fail!\n", __func__);
			return ;
		}  
		
		//Register 0x59
		if( i2c_himax_read(private_ts->client, 0x59, cmd, 4, 3) < 0){
			E("%s: i2c access fail!\n", __func__);
			return ;
		}
		
		//find iref group , default is iref 3
		for (i = 0; i < 16; i++){
			if ((cmd[0] == temp_iref[i][0]) &&
					(cmd[1] == temp_iref[i][1])){
				iref_number = i;
				iref_found = true;
				break;
			}
		}
		
		if(!iref_found ){
			E("%s: Can't find iref number!\n", __func__);
			return ;
		}
		else{
			I("%s: iref_number=%d, cmd[0]=0x%x, cmd[1]=0x%x\n", __func__, iref_number, cmd[0], cmd[1]);
		}
	}

	msleep(5);

	//iref write
	//Register 0x43
	cmd[0] = 0x01;
	cmd[1] = 0x00;
	cmd[2] = 0x06;
	if( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE ,&cmd[0], 3, 3) < 0){
		E("%s: i2c access fail!\n", __func__);
		return ;
	}  
	
	//Register 0x44
	cmd[0] = 0x00;
	cmd[1] = 0x00;
	cmd[2] = 0x00;
	if( i2c_himax_write(private_ts->client, HX_CMD_FLASH_SET_ADDRESS ,&cmd[0], 3, 3) < 0){
		E("%s: i2c access fail!\n", __func__);
		return ;
	}  
	
	//Register 0x45
	cmd[0] = temp_iref[iref_number][0];
	cmd[1] = temp_iref[iref_number][1];
	cmd[2] = 0x17;
	cmd[3] = 0x28;

	if( i2c_himax_write(private_ts->client, HX_CMD_FLASH_WRITE_REGISTER ,&cmd[0], 4, 3) < 0){
		E("%s: i2c access fail!\n", __func__);
		return ;
	}  
	
	//Register 0x4A
	if( i2c_himax_write(private_ts->client, HX_CMD_4A ,&cmd[0], 0, 3) < 0){
		E("%s: i2c access fail!\n", __func__);
		return ;
	}  

	//Read SFR to check the result
	//Register 0x43
	cmd[0] = 0x01;
	cmd[1] = 0x00;
	cmd[2] = 0x0A;
	if( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE ,&cmd[0], 3, 3) < 0){
		E("%s: i2c access fail!\n", __func__);
		return ;
	} 
	
	//Register 0x44
	cmd[0] = 0x00;
	cmd[1] = 0x00;
	cmd[2] = 0x00;
	if( i2c_himax_write(private_ts->client, HX_CMD_FLASH_SET_ADDRESS ,&cmd[0], 3, 3) < 0){
		E("%s: i2c access fail!\n", __func__);
		return ;
	} 
	
	//Register 0x46
	if( i2c_himax_write(private_ts->client, 0x46 ,&cmd[0], 0, 3) < 0){
		E("%s: i2c access fail!\n", __func__);
		return ;
	} 
	
	//Register 0x59
	if( i2c_himax_read(private_ts->client, 0x59, cmd, 4, 3) < 0){
		E("%s: i2c access fail!\n", __func__);
		return ;
	}

	I( "%s:cmd[0]=%d,cmd[1]=%d,temp_iref_1=%d,temp_iref_2=%d\n",__func__, cmd[0], cmd[1], temp_iref[iref_number][0], temp_iref[iref_number][1]);
	
	if(cmd[0] != temp_iref[iref_number][0] || cmd[1] != temp_iref[iref_number][1]){
		E("%s: IREF Read Back is not match.\n", __func__);
		E("%s: Iref [0]=%d,[1]=%d\n", __func__,cmd[0],cmd[1]);
	}
	else{
		I("%s: IREF Pass",__func__);
	}
	
	if (i2c_himax_write(private_ts->client, 0xA9 ,&cmd[0], 0, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return;
	}
	
}

static uint8_t himax_calculateChecksum(bool change_iref)
{	
	int iref_flag = 0;
	int ret = 0;
	uint8_t cmd[10];

	memset(cmd, 0x00, sizeof(cmd));
  
	I("%s: start\n", __func__);
  //Sleep out
	ret = i2c_himax_write(private_ts->client, HX_CMD_TSSLPOUT ,&cmd[0], 0, DEFAULT_RETRY_CNT);
	if(ret < 0) {
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}
	msleep(120);

	while(true) {
		if(change_iref) {
			if(iref_flag == 0) {
				himax_changeIref(2); //iref 2		
			} else if(iref_flag == 1) {
				himax_changeIref(5); //iref 5
			} else if(iref_flag == 2) {
				himax_changeIref(1); //iref 1
			} else {
				goto CHECK_FAIL;	
			}
			iref_flag++;
		}
		
		cmd[0] = 0x00;
		cmd[1] = 0x04;
		cmd[2] = 0x0A;
		cmd[3] = 0x02;
    	
		if (i2c_himax_write(private_ts->client, 0xED ,&cmd[0], 4, DEFAULT_RETRY_CNT) < 0) {
			E("%s: i2c access fail!\n", __func__);
			return 0;
		}
    	
		//Enable Flash
		cmd[0] = 0x01;
		cmd[1] = 0x00;
		cmd[2] = 0x02;
		ret = i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE ,&cmd[0], 3, DEFAULT_RETRY_CNT);
		if (ret < 0) {
			E("%s: i2c access fail!\n", __func__);
			return 0;
		}

		cmd[0] = 0x05;
		ret = i2c_himax_write(private_ts->client, 0xD2 ,&cmd[0], 1, DEFAULT_RETRY_CNT);
		if (ret < 0) {
			E("%s: i2c access fail!\n", __func__);
			return 0;
		}
    	
		cmd[0] = 0x01;
		ret = i2c_himax_write(private_ts->client, 0x53 ,&cmd[0], 1, DEFAULT_RETRY_CNT);
		if (ret < 0) {
			E("%s: i2c access fail!\n", __func__);
			return 0;
		}
		
		msleep(200);

		ret = i2c_himax_read(private_ts->client, 0xAD, cmd, 4, DEFAULT_RETRY_CNT);
		if (ret < 0) {
			E("%s: i2c access fail!\n", __func__);
			return -1;
		}
		
		I("%s: 0xAD[0,1,2,3] = %d,%d,%d,%d\n",
			__func__, cmd[0], cmd[1], cmd[2], cmd[3]);

		if (cmd[0] == 0 && cmd[1] == 0 && cmd[2] == 0 && cmd[3] == 0 ) {
			himax_FlashMode(0);
			goto CHECK_PASS;
		} else {
			himax_FlashMode(0);
			goto CHECK_FAIL;
		}

		CHECK_PASS:
			I("%s: Pass\n", __func__);
			if(change_iref) {
				if(iref_flag < 3) {
					continue;
				} else {
					return 1;
				}
			} else {
				return 1;
			}
	
		CHECK_FAIL:
			E("%s: checksum fail!!\n", __func__);
			return 0;
	}
	return 0;
}

int fts_ctpm_fw_upgrade_with_fs(unsigned char *fw, int len, bool change_iref)
{
	unsigned char* ImageBuffer = fw;
	int fullFileLength = len;
	int i;
	uint8_t cmd[5], last_byte, prePage;
	int FileLength;
	uint8_t checksumResult = 0;

	FileLength = fullFileLength;

	if ( i2c_himax_write(private_ts->client, HX_CMD_TSSLPOUT ,&cmd[0], 0, DEFAULT_RETRY_CNT) < 0)
	{
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}

	msleep(120);

	himax_lock_flash(0);

	cmd[0] = 0x05;cmd[1] = 0x00;cmd[2] = 0x02;
	if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE ,&cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
	{
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}

	if ( i2c_himax_write(private_ts->client, 0x4F ,&cmd[0], 0, DEFAULT_RETRY_CNT) < 0)
	{
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}
	msleep(50);

	himax_ManualMode(1);
	himax_FlashMode(1);

	FileLength = (FileLength + 3) / 4;
	for (i = 0, prePage = 0; i < FileLength; i++)
	{
		last_byte = 0;
		cmd[0] = i & 0x1F;
		if (cmd[0] == 0x1F || i == FileLength - 1)
		{
			last_byte = 1;
		}
		cmd[1] = (i >> 5) & 0x1F;
		cmd[2] = (i >> 10) & 0x1F;
		if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_SET_ADDRESS ,&cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
		{
			E("%s: i2c access fail!\n", __func__);
			return 0;
		}

		if (prePage != cmd[1] || i == 0)
		{
			prePage = cmd[1];
			cmd[0] = 0x01;cmd[1] = 0x09;//cmd[2] = 0x02;
			if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
			{
				E("%s: i2c access fail!\n", __func__);
				return 0;
			}

			cmd[0] = 0x01;cmd[1] = 0x0D;//cmd[2] = 0x02;
			if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
			{
				E("%s: i2c access fail!\n", __func__);
				return 0;
			}

			cmd[0] = 0x01;cmd[1] = 0x09;//cmd[2] = 0x02;
			if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
			{
				E("%s: i2c access fail!\n", __func__);
				return 0;
			}
		}

		memcpy(&cmd[0], &ImageBuffer[4*i], 4);
		if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_WRITE_REGISTER ,&cmd[0], 4, DEFAULT_RETRY_CNT) < 0)
		{
			E("%s: i2c access fail!\n", __func__);
			return 0;
		}

		cmd[0] = 0x01;cmd[1] = 0x0D;//cmd[2] = 0x02;
		if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
		{
			E("%s: i2c access fail!\n", __func__);
			return 0;
		}

		cmd[0] = 0x01;cmd[1] = 0x09;//cmd[2] = 0x02;
		if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
		{
			E("%s: i2c access fail!\n", __func__);
			return 0;
		}

		if (last_byte == 1)
		{
			cmd[0] = 0x01;cmd[1] = 0x01;//cmd[2] = 0x02;
			if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
			{
				E("%s: i2c access fail!\n", __func__);
				return 0;
			}

			cmd[0] = 0x01;cmd[1] = 0x05;//cmd[2] = 0x02;
			if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
			{
				E("%s: i2c access fail!\n", __func__);
				return 0;
			}

			cmd[0] = 0x01;cmd[1] = 0x01;//cmd[2] = 0x02;
			if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
			{
				E("%s: i2c access fail!\n", __func__);
				return 0;
			}

			cmd[0] = 0x01;cmd[1] = 0x00;//cmd[2] = 0x02;
			if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
			{
				E("%s: i2c access fail!\n", __func__);
				return 0;
			}

			msleep(10);
			if (i == (FileLength - 1))
			{
				himax_FlashMode(0);
				himax_ManualMode(0);
				checksumResult = himax_calculateChecksum(change_iref);
				//himax_ManualMode(0);
				himax_lock_flash(1);

				if (checksumResult) //Success
				{
					return 1;
				}
				else //Fail
				{
					E("%s: checksumResult fail!\n", __func__);
					return 0;
				}
			}
		}
	}
	return 0;
}

static int himax_input_register(struct himax_ts_data *ts)
{
	int ret;
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		E("%s: Failed to allocate input device\n", __func__);
		return ret;
	}
	ts->input_dev->name = "himax-touchscreen";

	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);

	set_bit(KEY_BACK, ts->input_dev->keybit);
	set_bit(KEY_HOME, ts->input_dev->keybit);
	set_bit(KEY_MENU, ts->input_dev->keybit);
	set_bit(KEY_SEARCH, ts->input_dev->keybit);
#if defined(HX_SMART_WAKEUP)||defined(HX_PALM_REPORT)
	set_bit(KEY_POWER, ts->input_dev->keybit);
	set_bit(KEY_CUST_01, ts->input_dev->keybit);
	set_bit(KEY_CUST_02, ts->input_dev->keybit);
	set_bit(KEY_CUST_03, ts->input_dev->keybit);
	set_bit(KEY_CUST_04, ts->input_dev->keybit);
	set_bit(KEY_CUST_05, ts->input_dev->keybit);
	set_bit(KEY_CUST_06, ts->input_dev->keybit);
	set_bit(KEY_CUST_07, ts->input_dev->keybit);
	set_bit(KEY_CUST_08, ts->input_dev->keybit);
	set_bit(KEY_CUST_09, ts->input_dev->keybit);
	set_bit(KEY_CUST_10, ts->input_dev->keybit);
	set_bit(KEY_CUST_11, ts->input_dev->keybit);
	set_bit(KEY_CUST_12, ts->input_dev->keybit);
	set_bit(KEY_CUST_13, ts->input_dev->keybit);
	set_bit(KEY_CUST_14, ts->input_dev->keybit);
	set_bit(KEY_CUST_15, ts->input_dev->keybit);
#endif	
	set_bit(BTN_TOUCH, ts->input_dev->keybit);

	set_bit(KEY_MENU, ts->input_dev->keybit);

	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

	if (ts->protocol_type == PROTOCOL_TYPE_A) {
		//ts->input_dev->mtsize = ts->nFinger_support;
		input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID,
		0, ts->nFinger_support, 0, 0);
	} else {	/* PROTOCOL_TYPE_B */
		set_bit(MT_TOOL_FINGER, ts->input_dev->keybit);
		input_mt_init_slots(ts->input_dev, ts->nFinger_support,0);
	}

	I("input_set_abs_params: min_x %d, max_x %d, min_y %d, max_y %d\n",
		ts->pdata->abs_x_min, ts->pdata->abs_x_max, ts->pdata->abs_y_min, ts->pdata->abs_y_max);

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X,ts->pdata->abs_x_min, ts->pdata->abs_x_max, ts->pdata->abs_x_fuzz, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,ts->pdata->abs_y_min, ts->pdata->abs_y_max, ts->pdata->abs_y_fuzz, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR,ts->pdata->abs_pressure_min, ts->pdata->abs_pressure_max, ts->pdata->abs_pressure_fuzz, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE,ts->pdata->abs_pressure_min, ts->pdata->abs_pressure_max, ts->pdata->abs_pressure_fuzz, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR,ts->pdata->abs_width_min, ts->pdata->abs_width_max, ts->pdata->abs_pressure_fuzz, 0);

//	input_set_abs_params(ts->input_dev, ABS_MT_AMPLITUDE, 0, ((ts->pdata->abs_pressure_max << 16) | ts->pdata->abs_width_max), 0, 0);
//	input_set_abs_params(ts->input_dev, ABS_MT_POSITION, 0, (BIT(31) | (ts->pdata->abs_x_max << 16) | ts->pdata->abs_y_max), 0, 0);

	return input_register_device(ts->input_dev);
}

static void calcDataSize(uint8_t finger_num)
{
	struct himax_ts_data *ts_data = private_ts;
	ts_data->coord_data_size = 4 * finger_num;
	ts_data->area_data_size = ((finger_num / 4) + (finger_num % 4 ? 1 : 0)) * 4;
	ts_data->raw_data_frame_size = 128 - ts_data->coord_data_size - ts_data->area_data_size - 4 - 4 - 1;
	ts_data->raw_data_nframes = ((uint32_t)ts_data->x_channel * ts_data->y_channel + ts_data->x_channel + ts_data->y_channel) / ts_data->raw_data_frame_size + (((uint32_t)ts_data->x_channel * ts_data->y_channel + ts_data->x_channel + ts_data->y_channel) % ts_data->raw_data_frame_size)? 1 : 0;
	I("%s: coord_data_size: %d, area_data_size:%d, raw_data_frame_size:%d, raw_data_nframes:%d\n",
		__func__, ts_data->coord_data_size, ts_data->area_data_size,
		ts_data->raw_data_frame_size, ts_data->raw_data_nframes);
}

static void calculate_point_number(void)
{
	HX_TOUCH_INFO_POINT_CNT = HX_MAX_PT * 4 ;

	if ( (HX_MAX_PT % 4) == 0)
		HX_TOUCH_INFO_POINT_CNT += (HX_MAX_PT / 4) * 4 ;
	else
		HX_TOUCH_INFO_POINT_CNT += ((HX_MAX_PT / 4) +1) * 4 ;
}
#ifdef HX_LOADIN_CONFIG
static int himax_config_reg_write(struct i2c_client *client, uint8_t StartAddr, uint8_t *data, uint8_t length, uint8_t toRetry)
{
	char cmd[12] = {0};
	
	cmd[0] = 0x8C;cmd[1] = 0x14;
	if ((i2c_himax_master_write(client, &cmd[0],2,toRetry))<0)
		return -1;

	cmd[0] = 0x8B;cmd[1] = 0x00;cmd[2] = StartAddr;	//Start addr
	if ((i2c_himax_master_write(client, &cmd[0],3,toRetry))<0)
		return -1;

	if ((i2c_himax_master_write(client, data,length,toRetry))<0)
		return -1;

	cmd[0] = 0x8C;cmd[1] = 0x00;
	if ((i2c_himax_master_write(client, &cmd[0],2,toRetry))<0)
		return -1;
	
	return 0;
}
#endif
void himax_touch_information(void)
{
	char data[12] = {0};
	
	I("%s:IC_TYPE =%d\n", __func__, IC_TYPE);
	
	if (IC_TYPE == HX_85XX_ES_SERIES_PWON) {
  		data[0] = 0x8C;
  		data[1] = 0x14;
	  	i2c_himax_master_write(private_ts->client, &data[0],2,DEFAULT_RETRY_CNT);
	  	msleep(10);
	  	data[0] = 0x8B;
	  	data[1] = 0x00;
	  	data[2] = 0x70;
	  	i2c_himax_master_write(private_ts->client, &data[0],3,DEFAULT_RETRY_CNT);
	  	msleep(10);
	  	i2c_himax_read(private_ts->client, 0x5A, data, 12, DEFAULT_RETRY_CNT);
	  	HX_RX_NUM = data[0];	/* FE(70) */
	  	HX_TX_NUM = data[1];	/* FE(71) */
	  	HX_MAX_PT = (data[2] & 0xF0) >> 4;	/* FE(72) */
#ifdef HX_EN_SEL_BUTTON
	  	HX_BT_NUM = (data[2] & 0x0F);	/* FE(72) */
#endif

	  	if((data[4] & 0x04) == 0x04) {	/* FE(74) */
			HX_XY_REVERSE = true;
			HX_Y_RES = data[6]*256 + data[7];	/* FE(76),FE(77) */
			HX_X_RES = data[8]*256 + data[9];	/* FE(78),FE(79) */
	  	} else {
			HX_XY_REVERSE = false;
			HX_X_RES = data[6]*256 + data[7];	/* FE(76),FE(77) */
			HX_Y_RES = data[8]*256 + data[9];	/* FE(78),FE(79) */
	  	}
	  	data[0] = 0x8C;
	  	data[1] = 0x00;
	  	i2c_himax_master_write(private_ts->client, &data[0],2,DEFAULT_RETRY_CNT);
	  	msleep(10);
#ifdef HX_EN_MUT_BUTTON
		data[0] = 0x8C;
	  	data[1] = 0x14;
	  	i2c_himax_master_write(private_ts->client, &data[0],2,DEFAULT_RETRY_CNT);
	  	msleep(10);
	  	data[0] = 0x8B;
	  	data[1] = 0x00;
	  	data[2] = 0x64;
	  	i2c_himax_master_write(private_ts->client, &data[0],3,DEFAULT_RETRY_CNT);
	  	msleep(10);
	  	i2c_himax_read(private_ts->client, 0x5A, data, 4, DEFAULT_RETRY_CNT);
	  	HX_BT_NUM = (data[0] & 0x03);
	  	data[0] = 0x8C;
	  	data[1] = 0x00;
	  	i2c_himax_master_write(private_ts->client, &data[0],2,DEFAULT_RETRY_CNT);
	  	msleep(10);
#endif
#ifdef HX_TP_PROC_2T2R
		data[0] = 0x8C;
		data[1] = 0x14;
		i2c_himax_master_write(private_ts->client, &data[0],2,DEFAULT_RETRY_CNT);
		msleep(10);
		
		data[0] = 0x8B;
		data[1] = 0x00;
		data[2] = HX_2T2R_Addr;
		i2c_himax_master_write(private_ts->client, &data[0],3,DEFAULT_RETRY_CNT);
		msleep(10);
		
		i2c_himax_read(private_ts->client, 0x5A, data, 10, DEFAULT_RETRY_CNT);
		
		HX_RX_NUM_2 = data[0];				 
		HX_TX_NUM_2 = data[1];				 
		
		I("%s:Touch Panel Type=%d \n", __func__,data[2]);
		if(data[2]==HX_2T2R_en_setting)	//2T2R type panel
			Is_2T2R = true;
		else
			Is_2T2R = false;
		
		data[0] = 0x8C;
		data[1] = 0x00;
		i2c_himax_master_write(private_ts->client, &data[0],2,DEFAULT_RETRY_CNT);
		msleep(10);
#endif
	  	data[0] = 0x8C;
	  	data[1] = 0x14;
	  	i2c_himax_master_write(private_ts->client, &data[0],2,DEFAULT_RETRY_CNT);
	  	msleep(10);
	  	data[0] = 0x8B;
	  	data[1] = 0x00;
	  	data[2] = 0x02;
	  	i2c_himax_master_write(private_ts->client, &data[0],3,DEFAULT_RETRY_CNT);
	  	msleep(10);
	  	i2c_himax_read(private_ts->client, 0x5A, data, 10, DEFAULT_RETRY_CNT);
	  	if((data[1] & 0x01) == 1) {	/* FE(02) */
			HX_INT_IS_EDGE = true;
	  	} else {
			HX_INT_IS_EDGE = false;
	  	}
	  	data[0] = 0x8C;
		data[1] = 0x00;
		i2c_himax_master_write(private_ts->client, &data[0],2,DEFAULT_RETRY_CNT);

		msleep(10);

	switch (private_ts->vendor_sensor_id) {
	case OFILM_SENSOR_ID:
		HX_RX_NUM = 25;
		HX_TX_NUM = 15;
		HX_MAX_PT = 10;
		HX_XY_REVERSE = false;
		HX_X_RES = 1080;
		HX_Y_RES = 1920;
		HX_INT_IS_EDGE = false;
	break;
	case ELLY_SENSOR_ID:
		HX_RX_NUM = 15;
		HX_TX_NUM = 25;
		HX_MAX_PT = 10;
		HX_XY_REVERSE = false;
		HX_X_RES = 1080;
		HX_Y_RES = 1920;
		HX_INT_IS_EDGE = false;
	break;
	default:
		E("%s: unknown, sensor id : 0x%02x\n", __func__, private_ts->vendor_sensor_id);
	break;
	}
		I("%s: HX_RX_NUM = %d, HX_TX_NUM = %d, HX_MAX_PT = %d \n",
			__func__, HX_RX_NUM, HX_TX_NUM, HX_MAX_PT);
		I("%s: HX_INT_IS_EDGE = %s\n",
			__func__, HX_INT_IS_EDGE?"true":"false");

		if (i2c_himax_read(private_ts->client, HX_VER_FW_CFG, data, 1, 3) < 0) {
			E("%s: i2c access fail!\n", __func__);
		}
		private_ts->vendor_config_ver = data[0];
		I("%s: config_ver=%x\n", __func__, private_ts->vendor_config_ver);
	} else {
		HX_RX_NUM				= 0;
		HX_TX_NUM				= 0;
		HX_BT_NUM				= 0;
		HX_X_RES				= 0;
		HX_Y_RES				= 0;
		HX_MAX_PT				= 0;
		HX_XY_REVERSE			= false;
		HX_INT_IS_EDGE			= false;
	}
}
static uint8_t himax_read_Sensor_ID(struct i2c_client *client)
{	
	uint8_t val_high[1], val_low[1], ID0=0, ID1=0;
	uint8_t sensor_id;
	char data[3];
	
	data[0] = 0x56; data[1] = 0x02; data[2] = 0x02;/*ID pin PULL High*/
	i2c_himax_master_write(client, &data[0],3,DEFAULT_RETRY_CNT);
	msleep(1);

	//read id pin high
	i2c_himax_read(client, 0x57, val_high, 1, DEFAULT_RETRY_CNT);

	data[0] = 0x56; data[1] = 0x01; data[2] = 0x01;/*ID pin PULL Low*/
	i2c_himax_master_write(client, &data[0],3,DEFAULT_RETRY_CNT);
	msleep(1);

	//read id pin low
	i2c_himax_read(client, 0x57, val_low, 1, DEFAULT_RETRY_CNT);

	if((val_high[0] & 0x01) ==0)
		ID0=0x02;/*GND*/
	else if((val_low[0] & 0x01) ==0)
		ID0=0x01;/*Floating*/
	else
		ID0=0x04;/*VCC*/
	
	if((val_high[0] & 0x02) ==0)
		ID1=0x02;/*GND*/
	else if((val_low[0] & 0x02) ==0)
		ID1=0x01;/*Floating*/
	else
		ID1=0x04;/*VCC*/
	if((ID0==0x04)&&(ID1!=0x04))
		{
			data[0] = 0x56; data[1] = 0x02; data[2] = 0x01;/*ID pin PULL High,Low*/
			i2c_himax_master_write(client, &data[0],3,DEFAULT_RETRY_CNT);
			msleep(1);

		}
	else if((ID0!=0x04)&&(ID1==0x04))
		{
			data[0] = 0x56; data[1] = 0x01; data[2] = 0x02;/*ID pin PULL Low,High*/
			i2c_himax_master_write(client, &data[0],3,DEFAULT_RETRY_CNT);
			msleep(1);

		}
	else if((ID0==0x04)&&(ID1==0x04))
		{
			data[0] = 0x56; data[1] = 0x02; data[2] = 0x02;/* ID pin PULL High,High */
			i2c_himax_master_write(client, &data[0],3,DEFAULT_RETRY_CNT);
			msleep(1);

		}
	sensor_id=(ID1<<4)|ID0;

	data[0] = 0xF3; data[1] = sensor_id;
	i2c_himax_master_write(client, &data[0],2,DEFAULT_RETRY_CNT);/* Write to MCU */
	msleep(1);

	return sensor_id;

}
static void himax_power_on_initCMD(struct i2c_client *client)
{
	I("%s:\n", __func__);
	/* Sense on to update the information */
	i2c_himax_write_command(client, HX_CMD_TSSON, DEFAULT_RETRY_CNT);
	msleep(30);

	i2c_himax_write_command(client, HX_CMD_TSSLPOUT, DEFAULT_RETRY_CNT);
	msleep(50);

	i2c_himax_write_command(client, HX_CMD_TSSOFF, DEFAULT_RETRY_CNT);
	msleep(50);

	i2c_himax_write_command(client, HX_CMD_TSSLPIN, DEFAULT_RETRY_CNT);
	msleep(50);

	himax_touch_information();	

	i2c_himax_write_command(client, HX_CMD_TSSON, DEFAULT_RETRY_CNT);
	msleep(30);

	i2c_himax_write_command(client, HX_CMD_TSSLPOUT, DEFAULT_RETRY_CNT);
	msleep(50);
}

#ifdef HX_AUTO_UPDATE_FW
static int i_update_FW(void)
{
	unsigned char *ImageBuffer = NULL;
	int fullFileLength = 0;

	switch (private_ts->vendor_sensor_id) {
	case OFILM_SENSOR_ID:
		ImageBuffer = i_CTPM_FW;
		fullFileLength = sizeof(i_CTPM_FW);

		FW_H = i_CTPM_FW[FW_VER_MAJ_FLASH_ADDR];
		FW_L = i_CTPM_FW[FW_VER_MIN_FLASH_ADDR];
		FW_CON = i_CTPM_FW[FW_CFG_VER_FLASH_ADDR];
	break;
	case ELLY_SENSOR_ID:
		ImageBuffer = i_CTPM_FW_ELLY;
		fullFileLength = sizeof(i_CTPM_FW_ELLY);

		FW_H = i_CTPM_FW_ELLY[FW_VER_MAJ_FLASH_ADDR];
		FW_L = i_CTPM_FW_ELLY[FW_VER_MIN_FLASH_ADDR];
		FW_CON = i_CTPM_FW_ELLY[FW_CFG_VER_FLASH_ADDR];
	break;
	default:
		E("unknown, sensor id : 0x%02x\n", private_ts->vendor_sensor_id);
		return 0;
	break;
	}

	I("IMAGE FW_VER = 0x%02x, 0x%02x\n", FW_H, FW_L);
	I("IMAGE CFG_VER = 0x%02x\n", FW_CON);

	if ((private_ts->vendor_fw_ver_H < FW_H)
		|| (private_ts->vendor_fw_ver_L < FW_L)
		|| (private_ts->vendor_config_ver < FW_CON)
		|| (himax_calculateChecksum(false) == 0)) {

		I("%s: TP upgrade start\n", __func__);
		fw_status = 2; /* CY33 20160119 update fw_status */
#ifdef HX_RST_PIN_FUNC
		himax_HW_reset(false, true);
#endif
		if (fts_ctpm_fw_upgrade_with_fs(ImageBuffer, fullFileLength, true) == 0) {
			E("%s: TP upgrade error\n", __func__);
			fw_status = 4; /* update fw_status 20160119 */
			return -1;
		} else {
			I("%s: TP upgrade OK\n", __func__);
		}
#ifdef HX_RST_PIN_FUNC
		himax_HW_reset(false, true);
#endif
		fw_status = 3; /* 20160119 update fw_status */
		return 1;
	} else {
		fw_status = 3; /* 20160119 update fw_status */
	}

	return 0;
}
#endif

#ifdef HX_AUTO_UPDATE_CONFIG
int fts_ctpm_fw_upgrade_with_i_file_flash_cfg(struct himax_config *cfg)
{
	unsigned char* ImageBuffer= i_CTPM_FW;//NULL ;//
	int fullFileLength = FLASH_SIZE;
	
	int i, k;
	uint8_t cmd[5], last_byte, prePage;
	uint8_t tmp[5];
	int FileLength;
	int Polynomial = 0x82F63B78;
	int CRC32 = 0xFFFFFFFF;
	int BinDataWord = 0;
	int current_index = 0;
	uint8_t checksumResult = 0;

	I(" %s: flash CONFIG only START!\n", __func__);

#if 0
	//=========Dump FW image from Flash========
	setSysOperation(1);
	setFlashCommand(0x0F);
	setFlashDumpProgress(0);
	setFlashDumpComplete(0);
	setFlashDumpFail(0);
	queue_work(private_ts->flash_wq, &private_ts->flash_work);
	for(i=0 ; i<1024 ; i++)
		{
			if(getFlashDumpComplete())
				{
					ImageBuffer = flash_buffer;
					fullFileLength = FLASH_SIZE;
					I(" %s: Load FW from flash OK! cycle=%d fullFileLength=%x\n", __func__,i,fullFileLength);
					break;
				}
			msleep(5);
		}
	if(i==400)
	{
		E(" %s: Load FW from flash time out fail!\n", __func__);
		return 0;
	}
	//===================================
#endif	
	current_index = CFB_START_ADDR + CFB_INFO_LENGTH;		

	//Update the Config Part from config array
	for(i=1 ; i<sizeof(cfg->c1)/sizeof(cfg->c1[0]) ; i++) ImageBuffer[current_index++] = cfg->c1[i];
	for(i=1 ; i<sizeof(cfg->c2)/sizeof(cfg->c2[0]) ; i++) ImageBuffer[current_index++] = cfg->c2[i];
	for(i=1 ; i<sizeof(cfg->c3)/sizeof(cfg->c3[0]) ; i++) ImageBuffer[current_index++] = cfg->c3[i];
	for(i=1 ; i<sizeof(cfg->c4)/sizeof(cfg->c4[0]) ; i++) ImageBuffer[current_index++] = cfg->c4[i];
	for(i=1 ; i<sizeof(cfg->c5)/sizeof(cfg->c5[0]) ; i++) ImageBuffer[current_index++] = cfg->c5[i];
	for(i=1 ; i<sizeof(cfg->c6)/sizeof(cfg->c6[0]) ; i++) ImageBuffer[current_index++] = cfg->c6[i];
	for(i=1 ; i<sizeof(cfg->c7)/sizeof(cfg->c7[0]) ; i++) ImageBuffer[current_index++] = cfg->c7[i];
	for(i=1 ; i<sizeof(cfg->c8)/sizeof(cfg->c8[0]) ; i++) ImageBuffer[current_index++] = cfg->c8[i];
	for(i=1 ; i<sizeof(cfg->c9)/sizeof(cfg->c9[0]) ; i++) ImageBuffer[current_index++] = cfg->c9[i];
	for(i=1 ; i<sizeof(cfg->c10)/sizeof(cfg->c10[0]) ; i++) ImageBuffer[current_index++] = cfg->c10[i];
	for(i=1 ; i<sizeof(cfg->c11)/sizeof(cfg->c11[0]) ; i++) ImageBuffer[current_index++] = cfg->c11[i];
	for(i=1 ; i<sizeof(cfg->c12)/sizeof(cfg->c12[0]) ; i++) ImageBuffer[current_index++] = cfg->c12[i];
	for(i=1 ; i<sizeof(cfg->c13)/sizeof(cfg->c13[0]) ; i++) ImageBuffer[current_index++] = cfg->c13[i];
	for(i=1 ; i<sizeof(cfg->c14)/sizeof(cfg->c14[0]) ; i++) ImageBuffer[current_index++] = cfg->c14[i];
	for(i=1 ; i<sizeof(cfg->c15)/sizeof(cfg->c15[0]) ; i++) ImageBuffer[current_index++] = cfg->c15[i];
	for(i=1 ; i<sizeof(cfg->c16)/sizeof(cfg->c16[0]) ; i++) ImageBuffer[current_index++] = cfg->c16[i];
	for(i=1 ; i<sizeof(cfg->c17)/sizeof(cfg->c17[0]) ; i++) ImageBuffer[current_index++] = cfg->c17[i];
	for(i=1 ; i<sizeof(cfg->c18)/sizeof(cfg->c18[0]) ; i++) ImageBuffer[current_index++] = cfg->c18[i];
	for(i=1 ; i<sizeof(cfg->c19)/sizeof(cfg->c19[0]) ; i++) ImageBuffer[current_index++] = cfg->c19[i];
	for(i=1 ; i<sizeof(cfg->c20)/sizeof(cfg->c20[0]) ; i++) ImageBuffer[current_index++] = cfg->c20[i];
	for(i=1 ; i<sizeof(cfg->c21)/sizeof(cfg->c21[0]) ; i++) ImageBuffer[current_index++] = cfg->c21[i];
	for(i=1 ; i<sizeof(cfg->c22)/sizeof(cfg->c22[0]) ; i++) ImageBuffer[current_index++] = cfg->c22[i];
	for(i=1 ; i<sizeof(cfg->c23)/sizeof(cfg->c23[0]) ; i++) ImageBuffer[current_index++] = cfg->c23[i];
	for(i=1 ; i<sizeof(cfg->c24)/sizeof(cfg->c24[0]) ; i++) ImageBuffer[current_index++] = cfg->c24[i];
	for(i=1 ; i<sizeof(cfg->c25)/sizeof(cfg->c25[0]) ; i++) ImageBuffer[current_index++] = cfg->c25[i];
	for(i=1 ; i<sizeof(cfg->c26)/sizeof(cfg->c26[0]) ; i++) ImageBuffer[current_index++] = cfg->c26[i];
	for(i=1 ; i<sizeof(cfg->c27)/sizeof(cfg->c27[0]) ; i++) ImageBuffer[current_index++] = cfg->c27[i];
	for(i=1 ; i<sizeof(cfg->c28)/sizeof(cfg->c28[0]) ; i++) ImageBuffer[current_index++] = cfg->c28[i];
	for(i=1 ; i<sizeof(cfg->c29)/sizeof(cfg->c29[0]) ; i++) ImageBuffer[current_index++] = cfg->c29[i];
	for(i=1 ; i<sizeof(cfg->c30)/sizeof(cfg->c30[0]) ; i++) ImageBuffer[current_index++] = cfg->c30[i];
	for(i=1 ; i<sizeof(cfg->c31)/sizeof(cfg->c31[0]) ; i++) ImageBuffer[current_index++] = cfg->c31[i];
	for(i=1 ; i<sizeof(cfg->c32)/sizeof(cfg->c32[0]) ; i++) ImageBuffer[current_index++] = cfg->c32[i];
	for(i=1 ; i<sizeof(cfg->c33)/sizeof(cfg->c33[0]) ; i++) ImageBuffer[current_index++] = cfg->c33[i];
	for(i=1 ; i<sizeof(cfg->c34)/sizeof(cfg->c34[0]) ; i++) ImageBuffer[current_index++] = cfg->c34[i];
	for(i=1 ; i<sizeof(cfg->c35)/sizeof(cfg->c35[0]) ; i++) ImageBuffer[current_index++] = cfg->c35[i];
	for(i=1 ; i<sizeof(cfg->c36)/sizeof(cfg->c36[0]) ; i++) ImageBuffer[current_index++] = cfg->c36[i];
	for(i=1 ; i<sizeof(cfg->c37)/sizeof(cfg->c37[0]) ; i++) ImageBuffer[current_index++] = cfg->c37[i];
	for(i=1 ; i<sizeof(cfg->c38)/sizeof(cfg->c38[0]) ; i++) ImageBuffer[current_index++] = cfg->c38[i];
	for(i=1 ; i<sizeof(cfg->c39)/sizeof(cfg->c39[0]) ; i++) ImageBuffer[current_index++] = cfg->c39[i];
	current_index=current_index+50;//dummy data
	//I(" %s: current_index=%d\n", __func__,current_index);
	for(i=1 ; i<sizeof(cfg->c40)/sizeof(cfg->c40[0]) ; i++) ImageBuffer[current_index++] = cfg->c40[i];
	for(i=1 ; i<sizeof(cfg->c41)/sizeof(cfg->c41[0]) ; i++) ImageBuffer[current_index++] = cfg->c41[i];

	//cal_checksum start//
	FileLength = fullFileLength;		
	FileLength = (FileLength + 3) / 4;
	for (i = 0; i < FileLength; i++)
	{
		memcpy(&cmd[0], &ImageBuffer[4*i], 4);
		if (i < (FileLength - 1))/*cal_checksum*/
		{
			for(k = 0; k < 4; k++)
			{
				BinDataWord |= cmd[k] << (k * 8);
			}

			CRC32 = BinDataWord ^ CRC32;
			for(k = 0; k < 32; k++)
			{
				if((CRC32 % 2) != 0)
				{
					CRC32 = ((CRC32 >> 1) & 0x7FFFFFFF) ^ Polynomial;
				}
				else
				{
					CRC32 = ((CRC32 >> 1) & 0x7FFFFFFF);
				}
			}
			BinDataWord = 0;
		}
	}
	//cal_checksum end//

	FileLength = fullFileLength;

	if( i2c_himax_write(private_ts->client, HX_CMD_TSSLPOUT ,&cmd[0], 0, DEFAULT_RETRY_CNT) < 0)
	{
		E(" %s: i2c access fail!\n", __func__);
		return 0;
	}

	msleep(120);

	himax_lock_flash(0);

	himax_ManualMode(1);
	himax_FlashMode(1);

	FileLength = (FileLength + 3) / 4;
	for (i = 0, prePage = 0; i < FileLength; i++)
	{
		last_byte = 0;

		if(i<0x20)
			continue;
		else if((i>0xBF)&&(i<(FileLength-0x20)))
			continue;
		
		cmd[0] = i & 0x1F;
		if (cmd[0] == 0x1F || i == FileLength - 1)
		{
			last_byte = 1;
		}
		cmd[1] = (i >> 5) & 0x1F;
		cmd[2] = (i >> 10) & 0x1F;

		if( i2c_himax_write(private_ts->client, HX_CMD_FLASH_SET_ADDRESS ,&cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
		{
			E(" %s: i2c access fail!\n", __func__);
			return 0;
		}

		//I(" %s: CMD 44 [0,1,2]=[%x,%x,%x]\n", __func__,cmd[0],cmd[1],cmd[2]);
		if (prePage != cmd[1] || i == 0x20)
		{
			prePage = cmd[1];
			//I(" %s: %d page erase\n",__func__,prePage);
			tmp[0] = 0x01;tmp[1] = 0x09;//tmp[2] = 0x02;
			if( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE ,&tmp[0], 2, DEFAULT_RETRY_CNT) < 0)
			{
				E(" %s: i2c access fail!\n", __func__);
				return 0;
			}
			
			tmp[0] = 0x05;tmp[1] = 0x2D;//tmp[2] = 0x02;
			if( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE ,&tmp[0], 2, DEFAULT_RETRY_CNT) < 0)
			{
				E(" %s: i2c access fail!\n", __func__);
				return 0;
			}
			msleep(30);
			tmp[0] = 0x01;tmp[1] = 0x09;//tmp[2] = 0x02;
			if( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE ,&tmp[0], 2, DEFAULT_RETRY_CNT) < 0)
			{
				E(" %s: i2c access fail!\n", __func__);
				return 0;
			}

			if( i2c_himax_write(private_ts->client, HX_CMD_FLASH_SET_ADDRESS ,&cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
			{
				E(" %s: i2c access fail!\n", __func__);
				return 0;
			}
				
			//I(" %s: CMD 44-47 [0,1,2]=[%x,%x,%x]\n", __func__,cmd[0],cmd[1],cmd[2]);
				
			tmp[0] = 0x01;tmp[1] = 0x09;//tmp[2] = 0x02;
			if( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE ,&tmp[0], 2, DEFAULT_RETRY_CNT) < 0)
			{
				E(" %s: i2c access fail!\n", __func__);
				return 0;
			}
			
			tmp[0] = 0x01;tmp[1] = 0x0D;//tmp[2] = 0x02;
			if( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE ,&tmp[0], 2, DEFAULT_RETRY_CNT) < 0)
			{
				E(" %s: i2c access fail!\n", __func__);
				return 0;
			}
			tmp[0] = 0x01;tmp[1] = 0x09;//tmp[2] = 0x02;
			if( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE ,&tmp[0], 2, DEFAULT_RETRY_CNT) < 0)
			{
				E(" %s: i2c access fail!\n", __func__);
				return 0;
			}

			if( i2c_himax_write(private_ts->client, HX_CMD_FLASH_SET_ADDRESS ,&cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
			{
				E(" %s: i2c access fail!\n", __func__);
				return 0;
			}

			//I(" %s: CMD 44-45 [0,1,2]=[%x,%x,%x]\n", __func__,cmd[0],cmd[1],cmd[2]);
			//I(" %s: Erase Page num=%x\n", __func__,prePage);
		}
	
		memcpy(&cmd[0], &ImageBuffer[4*i], 4);
						
		if (i == (FileLength - 1))
		{
			tmp[0]= (CRC32 & 0xFF);
			tmp[1]= ((CRC32 >>8) & 0xFF);
			tmp[2]= ((CRC32 >>16) & 0xFF);
			tmp[3]= ((CRC32 >>24) & 0xFF);
					
			memcpy(&cmd[0], &tmp[0], 4);
			I("%s last_byte = 1, CRC32= %x, =[0,1,2,3] = %x,%x,%x,%x \n",__func__, CRC32,tmp[0],tmp[1],tmp[2],tmp[3]);
		}
			
		if( i2c_himax_write(private_ts->client, HX_CMD_FLASH_WRITE_REGISTER ,&cmd[0], 4, DEFAULT_RETRY_CNT) < 0)
		{
			E(" %s: i2c access fail!\n", __func__);
			return 0;
		}
		//I(" %s: CMD 48 \n", __func__);
		cmd[0] = 0x01;cmd[1] = 0x0D;//cmd[2] = 0x02;
		if( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
		{
			E(" %s: i2c access fail!\n", __func__);
			return 0;
		}

		cmd[0] = 0x01;cmd[1] = 0x09;//cmd[2] = 0x02;
		if( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
		{
			E(" %s: i2c access fail!\n", __func__);
			return 0;
		}

		if (last_byte == 1)
		{
			cmd[0] = 0x01;cmd[1] = 0x01;//cmd[2] = 0x02;
			if( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
			{
				E(" %s: i2c access fail!\n", __func__);
				return 0;
			}

			cmd[0] = 0x01;cmd[1] = 0x05;//cmd[2] = 0x02;
			if( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
			{
				E(" %s: i2c access fail!\n", __func__);
				return 0;
			}

			cmd[0] = 0x01;cmd[1] = 0x01;//cmd[2] = 0x02;
			if( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
			{
				E(" %s: i2c access fail!\n", __func__);
				return 0;
			}

			cmd[0] = 0x01;cmd[1] = 0x00;//cmd[2] = 0x02;
			if( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
			{
				E(" %s: i2c access fail!\n", __func__);
				return 0;
			}

			msleep(10);
			if (i == (FileLength - 1))
			{
				himax_FlashMode(0);
				himax_ManualMode(0);
				checksumResult = himax_calculateChecksum(true);//
				//himax_ManualMode(0);
				himax_lock_flash(1);

				I(" %s: flash CONFIG only END!\n", __func__);
				if (checksumResult) //Success
				{
					return 1;
				}
				else //Fail
				{
					E("%s: checksumResult fail!\n", __func__);
					return 0;
				}
			}
		}
	}

	return 0;
}

static int i_update_FWCFG(struct himax_config *cfg)
{
	int upgrade_times = 0;

	I("%s: CHIP CONFG version=%x\n", __func__,private_ts->vendor_config_ver);
	I("%s: LOAD CONFG version=%x\n", __func__,cfg->c40[1]);
	
	if ( private_ts->vendor_config_ver != cfg->c40[1] )
		{
update_retry:
#ifdef HX_RST_PIN_FUNC
			himax_HW_reset(false,true);
#endif
			if(fts_ctpm_fw_upgrade_with_i_file_flash_cfg(cfg) == 0)
				{
					upgrade_times++;
					E("%s: TP upgrade error times=%d\n", __func__, upgrade_times);
					if(upgrade_times < 3)
						goto update_retry;
				}
			else
				I("%s: TP upgrade OK\n", __func__);
#ifdef HX_RST_PIN_FUNC
			himax_HW_reset(false,true);
#endif
			return 1;	
		}
	else
		return 0;
}

#endif

static int himax_loadSensorConfig(struct i2c_client *client, struct himax_i2c_platform_data *pdata)
{
#if defined(HX_LOADIN_CONFIG)||defined(HX_AUTO_UPDATE_CONFIG)
	int rc= 0;
#endif

#ifndef CONFIG_OF
#if defined(HX_LOADIN_CONFIG)||defined(HX_AUTO_UPDATE_CONFIG)
	int i = 0;
#endif
#endif

#ifdef HX_ESD_WORKAROUND
	char data[12] = {0};
#endif

	if (!client) {
		E("%s: Necessary parameters client are null!\n", __func__);
		return -1;
	}

	if(config_load == false) {
		config_selected = kzalloc(sizeof(*config_selected), GFP_KERNEL);
		if (config_selected == NULL) {
			E("%s: alloc config_selected fail!\n", __func__);
			return -1;
		}
	}
#ifndef CONFIG_OF
	pdata = client->dev.platform_data;
	if (!pdata) {
		E("%s: Necessary parameters pdata are null!\n", __func__);
		return -1;
	}
#endif

#if defined(HX_LOADIN_CONFIG)||defined(HX_AUTO_UPDATE_CONFIG)
#ifdef CONFIG_OF
	if(config_load == false) {
		rc = himax_parse_config(private_ts, config_selected);
		if (rc < 0) {
			E(" DT:cfg table parser FAIL. ret=%d\n", rc);
			goto HimaxErr;
		} else if (rc == 0){
			if ((private_ts->tw_x_max)&&(private_ts->tw_y_max)) {
					pdata->abs_x_min = private_ts->tw_x_min;
					pdata->abs_x_max = private_ts->tw_x_max;
					pdata->abs_y_min = private_ts->tw_y_min;
					pdata->abs_y_max = private_ts->tw_y_max;
					I(" DT-%s:config-panel-coords = %d, %d, %d, %d\n", __func__, pdata->abs_x_min,
					pdata->abs_x_max, pdata->abs_y_min, pdata->abs_y_max);
			}

			if ((private_ts->pl_x_max)&&(private_ts->pl_y_max)) {
					pdata->screenWidth = private_ts->pl_x_max;
					pdata->screenHeight= private_ts->pl_y_max;
					I(" DT-%s:config-display-coords = (%d, %d)", __func__, pdata->screenWidth,
					pdata->screenHeight);
			}
			config_load = true;
			I(" DT parser Done\n");
		}
	}
#else
	I("pdata->hx_config_size=%x.\n",(pdata->hx_config_size+1));
	I("config_type_size=%x.\n",sizeof(struct himax_config));

	if (pdata->hx_config) {
		for (i = 0; i < pdata->hx_config_size/sizeof(struct himax_config); ++i) {
		I("(pdata->hx_config)[%x].fw_ver_main=%x.\n",i,(pdata->hx_config)[i].fw_ver_main);
		I("(pdata->hx_config)[%x].fw_ver_minor=%x.\n",i,(pdata->hx_config)[i].fw_ver_minor);
		I("(pdata->hx_config)[%x].sensor_id=%x.\n",i,(pdata->hx_config)[i].sensor_id);

			if ((private_ts->vendor_fw_ver_H << 8 | private_ts->vendor_fw_ver_L) <
				((pdata->hx_config)[i].fw_ver_main << 8 | (pdata->hx_config)[i].fw_ver_minor)) {
				continue;
			} else {
				if ((private_ts->vendor_sensor_id == (pdata->hx_config)[i].sensor_id)) {
					config_selected = &((pdata->hx_config)[i]);
					I("hx_config selected, %X\n", 
						(uint32_t)config_selected);						
					config_load = true;
					break;
				} else if ((pdata->hx_config)[i].default_cfg) {
					I("default_cfg detected.\n");
					config_selected = &((pdata->hx_config)[i]);
					I("hx_config selected, %X\n", 
						(uint32_t)config_selected);						
					config_load = true;
					break;
				}
			}
		}
	} else {
		E("[HimaxError] %s pdata->hx_config is not exist \n",__func__);
		goto HimaxErr;
	}
#endif
#endif
#ifdef HX_LOADIN_CONFIG
	if (config_selected) {
		//Read config id
		private_ts->vendor_config_ver = config_selected->c40[1];

		//Normal Register c1~c35
		i2c_himax_master_write(client, config_selected->c1,sizeof(config_selected->c1),DEFAULT_RETRY_CNT);
		i2c_himax_master_write(client, config_selected->c2,sizeof(config_selected->c2),DEFAULT_RETRY_CNT);
		i2c_himax_master_write(client, config_selected->c3,sizeof(config_selected->c3),DEFAULT_RETRY_CNT);
		i2c_himax_master_write(client, config_selected->c4,sizeof(config_selected->c4),DEFAULT_RETRY_CNT);
		i2c_himax_master_write(client, config_selected->c5,sizeof(config_selected->c5),DEFAULT_RETRY_CNT);
		i2c_himax_master_write(client, config_selected->c6,sizeof(config_selected->c6),DEFAULT_RETRY_CNT);
		i2c_himax_master_write(client, config_selected->c7,sizeof(config_selected->c7),DEFAULT_RETRY_CNT);
		i2c_himax_master_write(client, config_selected->c8,sizeof(config_selected->c8),DEFAULT_RETRY_CNT);
		i2c_himax_master_write(client, config_selected->c9,sizeof(config_selected->c9),DEFAULT_RETRY_CNT);
		i2c_himax_master_write(client, config_selected->c10,sizeof(config_selected->c10),DEFAULT_RETRY_CNT);
		i2c_himax_master_write(client, config_selected->c11,sizeof(config_selected->c11),DEFAULT_RETRY_CNT);
		i2c_himax_master_write(client, config_selected->c12,sizeof(config_selected->c12),DEFAULT_RETRY_CNT);
		i2c_himax_master_write(client, config_selected->c13,sizeof(config_selected->c13),DEFAULT_RETRY_CNT);
		i2c_himax_master_write(client, config_selected->c14,sizeof(config_selected->c14),DEFAULT_RETRY_CNT);
		i2c_himax_master_write(client, config_selected->c15,sizeof(config_selected->c15),DEFAULT_RETRY_CNT);
		i2c_himax_master_write(client, config_selected->c16,sizeof(config_selected->c16),DEFAULT_RETRY_CNT);
		i2c_himax_master_write(client, config_selected->c17,sizeof(config_selected->c17),DEFAULT_RETRY_CNT);
		i2c_himax_master_write(client, config_selected->c18,sizeof(config_selected->c18),DEFAULT_RETRY_CNT);
		i2c_himax_master_write(client, config_selected->c19,sizeof(config_selected->c19),DEFAULT_RETRY_CNT);
		i2c_himax_master_write(client, config_selected->c20,sizeof(config_selected->c20),DEFAULT_RETRY_CNT);
		i2c_himax_master_write(client, config_selected->c21,sizeof(config_selected->c21),DEFAULT_RETRY_CNT);
		i2c_himax_master_write(client, config_selected->c22,sizeof(config_selected->c22),DEFAULT_RETRY_CNT);
		i2c_himax_master_write(client, config_selected->c23,sizeof(config_selected->c23),DEFAULT_RETRY_CNT);
		i2c_himax_master_write(client, config_selected->c24,sizeof(config_selected->c24),DEFAULT_RETRY_CNT);
		i2c_himax_master_write(client, config_selected->c25,sizeof(config_selected->c25),DEFAULT_RETRY_CNT);
		i2c_himax_master_write(client, config_selected->c26,sizeof(config_selected->c26),DEFAULT_RETRY_CNT);
		i2c_himax_master_write(client, config_selected->c27,sizeof(config_selected->c27),DEFAULT_RETRY_CNT);
		i2c_himax_master_write(client, config_selected->c28,sizeof(config_selected->c28),DEFAULT_RETRY_CNT);
		i2c_himax_master_write(client, config_selected->c29,sizeof(config_selected->c29),DEFAULT_RETRY_CNT);
		i2c_himax_master_write(client, config_selected->c30,sizeof(config_selected->c30),DEFAULT_RETRY_CNT);
		i2c_himax_master_write(client, config_selected->c31,sizeof(config_selected->c31),DEFAULT_RETRY_CNT);
		i2c_himax_master_write(client, config_selected->c32,sizeof(config_selected->c32),DEFAULT_RETRY_CNT);
		i2c_himax_master_write(client, config_selected->c33,sizeof(config_selected->c33),DEFAULT_RETRY_CNT);
		i2c_himax_master_write(client, config_selected->c34,sizeof(config_selected->c34),DEFAULT_RETRY_CNT);
		i2c_himax_master_write(client, config_selected->c35,sizeof(config_selected->c35),DEFAULT_RETRY_CNT);
		i2c_himax_master_write(client, config_selected->c36,sizeof(config_selected->c36),DEFAULT_RETRY_CNT);
		i2c_himax_master_write(client, config_selected->c37,sizeof(config_selected->c37),DEFAULT_RETRY_CNT);
		i2c_himax_master_write(client, config_selected->c38,sizeof(config_selected->c38),DEFAULT_RETRY_CNT);
		i2c_himax_master_write(client, config_selected->c39,sizeof(config_selected->c39),DEFAULT_RETRY_CNT);
		
		//Config Bank register
		himax_config_reg_write(client, 0x00,config_selected->c40,sizeof(config_selected->c40),DEFAULT_RETRY_CNT);
		himax_config_reg_write(client, 0x9E,config_selected->c41,sizeof(config_selected->c41),DEFAULT_RETRY_CNT);

		msleep(1);
	} else {
		E("[HimaxError] %s config_selected is null.\n",__func__);
		goto HimaxErr;
	}
#endif
#ifdef HX_ESD_WORKAROUND
	/* Check R36 to check IC Status */
	i2c_himax_read(client, 0x36, data, 2, 10);
	if(data[0] != 0x0F || data[1] != 0x53)
	{
		/* IC is abnormal */
		E("[HimaxError] %s R36 Fail : R36[0]=%d,R36[1]=%d,R36 Counter=%d \n",__func__,data[0],data[1],ESD_R36_FAIL);
		return -1;
	}
#endif

#ifdef HX_AUTO_UPDATE_CONFIG
		
	if (i_update_FWCFG(config_selected)==false)
		I("NOT Have new FWCFG=NOT UPDATE=\n");
	else
		I("Have new FWCFG=UPDATE=\n");
#endif

	himax_power_on_initCMD(client);
	I("%s: initialization complete\n", __func__);
	return 1;

#if defined(HX_LOADIN_CONFIG)||defined(HX_AUTO_UPDATE_CONFIG)
HimaxErr:
	return -1;
#endif	
}

#ifdef HX_RST_PIN_FUNC
void himax_HW_reset(uint8_t loadconfig,uint8_t int_off)
{
	struct himax_ts_data *ts = private_ts;
	int ret = 0;

	HW_RESET_ACTIVATE = 1;

	if (!int_off) {
		if (ts->use_irq) {
			himax_int_enable(private_ts->client->irq,0,true);
		} else {
			hrtimer_cancel(&ts->timer);
			ret = cancel_work_sync(&ts->work);
		}
	}

	I("%s: Now reset the Touch chip.\n", __func__);

	tpd_gpio_output(TP_RST_PIN, 0);
	msleep(20);
	tpd_gpio_output(TP_RST_PIN, 1);
	msleep(20);

	if(loadconfig)
		himax_loadSensorConfig(private_ts->client,private_ts->pdata);

	if(!int_off) {
		if (ts->use_irq)
			himax_int_enable(private_ts->client->irq,1,true);
		else
			hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}
}
#endif

static u8 himax_read_FW_ver(bool hw_reset)
{
	uint8_t cmd[3];
#ifdef HX_CHIP_STATUS_MONITOR
	HX_CHIP_POLLING_COUNT=0;
#endif

	himax_int_enable(private_ts->client->irq,0,true);

	/* ==== Enter Sleep mode ==== */
	i2c_himax_write_command(private_ts->client, HX_CMD_TSSOFF, DEFAULT_RETRY_CNT);
	msleep(50);

	i2c_himax_write_command(private_ts->client, HX_CMD_TSSLPIN, DEFAULT_RETRY_CNT);
	msleep(50);
	/* ========================== */

	if (i2c_himax_read(private_ts->client, HX_VER_FW_MAJ, cmd, 1, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}
	private_ts->vendor_fw_ver_H = cmd[0];
	if (i2c_himax_read(private_ts->client, HX_VER_FW_MIN, cmd, 1, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}
	private_ts->vendor_fw_ver_L = cmd[0];
	I("FW_VER : %d,%d \n",private_ts->vendor_fw_ver_H,private_ts->vendor_fw_ver_L);
	
	if (i2c_himax_read(private_ts->client, HX_VER_FW_CFG, cmd, 1, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}
	private_ts->vendor_config_ver = cmd[0];
	I("CFG_VER : %d \n",private_ts->vendor_config_ver);

	/* ==== Enter Normal mode ==== */
	i2c_himax_write_command(private_ts->client, HX_CMD_TSSON, DEFAULT_RETRY_CNT);
	msleep(30);

	i2c_himax_write_command(private_ts->client, HX_CMD_TSSLPOUT, DEFAULT_RETRY_CNT);
	msleep(50);
	/* ========================== */

	himax_int_enable(private_ts->client->irq,1,true);
	
	return 0;
}

static bool himax_ic_package_check(struct himax_ts_data *ts)
{
	uint8_t cmd[3];

	memset(cmd, 0x00, sizeof(cmd));

	if (i2c_himax_read(ts->client, 0xD1, cmd, 3, DEFAULT_RETRY_CNT) < 0)
		return false ;

	if(cmd[0] == 0x05 && cmd[1] == 0x85 &&
		(cmd[2] == 0x25 || cmd[2] == 0x26 || cmd[2] == 0x27 || cmd[2] == 0x28)) {
			IC_TYPE         	= HX_85XX_ES_SERIES_PWON;
			IC_CHECKSUM 		= HX_TP_BIN_CHECKSUM_CRC;
			/* Himax: Set FW and CFG Flash Address */                                        
			FW_VER_MAJ_FLASH_ADDR   = 133;	/* 0x0085 */
			FW_VER_MAJ_FLASH_LENG   = 1;;
			FW_VER_MIN_FLASH_ADDR   = 134;	/* 0x0086 */
			FW_VER_MIN_FLASH_LENG   = 1;
			CFG_VER_MAJ_FLASH_ADDR 	= 160;	/* 0x00A0 */
			CFG_VER_MAJ_FLASH_LENG 	= 12;
			CFG_VER_MIN_FLASH_ADDR 	= 172;	/* 0x00AC */
			CFG_VER_MIN_FLASH_LENG 	= 12;
			FW_CFG_VER_FLASH_ADDR	= 132;	/* 0x0084 */
#ifdef HX_AUTO_UPDATE_CONFIG
			CFB_START_ADDR 			= 0x80;
			CFB_LENGTH				= 638;
			CFB_INFO_LENGTH 		= 68;
#endif
			I("%s: Himax IC package 852x ES\n", __func__);
		} else {
			E("%s: Himax IC package incorrect!!PKG[0]=%x,PKG[1]=%x,PKG[2]=%x\n",
				__func__, cmd[0], cmd[1], cmd[2]);
			return false ;
		}
	return true;
}

static void himax_read_TP_info(struct i2c_client *client)
{
	char data[12] = {0};

	//read fw version
	if (i2c_himax_read(client, HX_VER_FW_MAJ, data, 1, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return;
	}
	private_ts->vendor_fw_ver_H = data[0];

	if (i2c_himax_read(client, HX_VER_FW_MIN, data, 1, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return;
	}
	private_ts->vendor_fw_ver_L = data[0];
	//read config version
	if (i2c_himax_read(client, HX_VER_FW_CFG, data, 1, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return;
	}
	private_ts->vendor_config_ver = data[0];

	//read sensor ID
	private_ts->vendor_sensor_id = himax_read_Sensor_ID(client);

	I("sensor_id=%x.\n",private_ts->vendor_sensor_id);
	I("fw_ver=%x,%x.\n",private_ts->vendor_fw_ver_H,private_ts->vendor_fw_ver_L);
	I("config_ver=%x.\n",private_ts->vendor_config_ver);
}

#ifdef HX_ESD_WORKAROUND
	void ESD_HW_REST(void)
	{
		ESD_recovery_count++;
		//==== 20160216 for Speed Fw
		if (self_test_inter_flag == 1 )
		{
			I("In self test ,not  TP: ESD - Reset\n");
			return;
		}
		//====
		ESD_RESET_ACTIVATE = 1;
		ESD_COUNTER = 0;
		ESD_R36_FAIL = 0;
#ifdef HX_CHIP_STATUS_MONITOR
		HX_CHIP_POLLING_COUNT=0;
#endif
		I("START_Himax TP: ESD - Reset\n");

		while(ESD_R36_FAIL <=3 )
		{

		himax_rst_gpio_set(private_ts->rst_gpio, 0);
		msleep(20);
		himax_rst_gpio_set(private_ts->rst_gpio, 1);
		msleep(20);

		if(himax_loadSensorConfig(private_ts->client, private_ts->pdata)<0)
			ESD_R36_FAIL++;
		else
			break;
		}
		I("END_Himax TP: ESD - Reset\n");
	}
#endif

#ifdef HX_CHIP_STATUS_MONITOR
static void himax_chip_monitor_function(struct work_struct *work) //for ESD solution
{
	int ret=0;

	I(" %s: POLLING_COUNT=%x, STATUS=%x, ESD_recovery_count=%d\n", __func__,HX_CHIP_POLLING_COUNT,ret, ESD_recovery_count);
	if(HX_CHIP_POLLING_COUNT >= (HX_POLLING_TIMES-1))//POLLING TIME
	{
		HX_ON_HAND_SHAKING=1;
		ret = himax_hand_shaking(); //0:Running, 1:Stop, 2:I2C Fail
		HX_ON_HAND_SHAKING=0;
		if(ret == 2)
		{
			I(" %s: I2C Fail \n", __func__);
			ESD_HW_REST();
		}
		else if(ret == 1)
		{
			I(" %s: MCU Stop \n", __func__);
			ESD_HW_REST();
		}
		HX_CHIP_POLLING_COUNT=0;//clear polling counter
	}
	else
		HX_CHIP_POLLING_COUNT++;

	queue_delayed_work(private_ts->himax_chip_monitor_wq, &private_ts->himax_chip_monitor, HX_POLLING_TIMER*HZ);

	return;
}
#endif

#ifdef HX_DOT_VIEW
static void himax_set_cover_func(unsigned int enable)
{
	uint8_t cmd[4];
	
	if (enable)
		cmd[0] = 0x40;
	else
		cmd[0] = 0x00;
	if ( i2c_himax_write(private_ts->client, 0x8F ,&cmd[0], 1, DEFAULT_RETRY_CNT) < 0)
		E("%s i2c write fail.\n",__func__);

	return;
}

static int hallsensor_hover_status_handler_func(struct notifier_block *this,
	unsigned long status, void *unused)
{
	int pole = 0, pole_value = 0;
	struct himax_ts_data *ts = private_ts;

	pole_value = 0x1 & status;
	pole = (0x2 & status) >> 1;
	I("[HL] %s[%s]\n", pole? "att_s" : "att_n", pole_value ? "Near" : "Far");

	if (pole == 1) {
		if (pole_value == 0)
			ts->cover_enable = 0;
		else{
				ts->cover_enable = 1;
			}

		himax_set_cover_func(ts->cover_enable);
		I("[HL] %s: cover_enable = %d.\n", __func__, ts->cover_enable);
	}

	return NOTIFY_OK;
}

static struct notifier_block hallsensor_status_handler = {
	.notifier_call = hallsensor_hover_status_handler_func,
};
#endif

#ifdef HX_SMART_WAKEUP
static int himax_parse_wake_event(struct himax_ts_data *ts)
{
	uint8_t buf[128];
	unsigned char check_sum_cal = 0;
	int ret = 0;
	int i=0, check_FC = 0, gesture_flag = 0;

	ret = i2c_himax_read(ts->client, 0x86, buf, 128, DEFAULT_RETRY_CNT);
	if (ret < 0) {
		E("%s: can't read data from chip!\n", __func__);
	}

	/* buf [0 ~ 7] expected result : 0x80 0x80 0x80 0x80 0xCC 0x44 0x00 0xF0 */
	/* checksum total sum 0x00 */
	for (i = 0 ; i < (GEST_PTLG_ID_LEN+GEST_PTLG_HDR_LEN) ; i++) {
		check_sum_cal += buf[i];
		I("%s: buf[%d] = 0x%02X , checksum = 0x%02X\n",
			__func__, i, buf[i], check_sum_cal);
	}

	for (i = 0 ; i < GEST_PTLG_ID_LEN ; i++) {
		if (check_FC == 0) {
			if ((buf[0] != 0x00) && ((buf[0] <= 0x0F) || (buf[0] == SMART_WAKEUP_FLAG))) {
				check_FC = 1;
				gesture_flag = buf[i];
			} else {
				check_FC = 0;
				E("%s: data != SMART_WAKEUP_FLAG(0x%02x) , buf[%d] = 0x%02x , skip the event\n",
					__func__, SMART_WAKEUP_FLAG, i, buf[i]);
				break;
			}
		} else {
			if (buf[i] != gesture_flag) {
				check_FC = 0;
				E("%s: ID NOT the same , buf[%d](0x%02x) != gesture_flag(0x%02x) So STOP parse event\n",
					__func__, i, buf[i], gesture_flag);
				break;
			}
		}
	}

	I("%s: gesture_flag = %02x , check_FC = %d\n",
		__func__, gesture_flag, check_FC);

	if (check_FC == 0) {
		E("%s: check_FC fail ,  %d\n",
			__func__, check_FC);
		return 0;
	}

	if (buf[GEST_PTLG_ID_LEN] != GEST_PTLG_HDR_ID1 ||
			buf[GEST_PTLG_ID_LEN+1] != GEST_PTLG_HDR_ID2) {
		E("%s: GEST_PTLG_ID_LEN = %d , GEST_PTLG_ID_LEN+1 = %d\n",
			__func__, GEST_PTLG_ID_LEN, GEST_PTLG_ID_LEN+1);
		E("%s: check GEST_PTLG_ID fail, buf[GEST_PTLG_ID_LEN] = 0x%02x , buf[GEST_PTLG_ID_LEN+1] = 0x%02x\n",
			__func__, buf[GEST_PTLG_ID_LEN], buf[GEST_PTLG_ID_LEN+1]);

		return 0;
	}

	if ((check_sum_cal != 0x00)) {
		E("%s : check_sum_cal fail : 0x%02X\n", __func__, check_sum_cal);
		return 0;
	}

	if (gesture_flag != SMART_WAKEUP_FLAG) {
		/* NOT report customer key */
		if (!ts->gesture_cust_en[gesture_flag]) {
			E("%s NOT report customer key , 0x%02X\n", __func__, gesture_flag);
			return 0;
		}
	} else {
		/* NOT report power key */
		if (!ts->gesture_cust_en[0]) {
			E("%s: NOT report report double click , 0x%02x\n", __func__, gesture_flag);
			return 0;
		}
	}

	if (gesture_flag == SMART_WAKEUP_FLAG) {
		I("%s, send EV_GESTURE_PWR(0x%02x) for smart wakeup\n", __func__, gesture_flag);
	} else {
		I("%s, send gesture_flag : 0x%02x\n", __func__, gesture_flag);
	}
	return gesture_flag;
}

#endif

static void himax_ts_button_func(int tp_key_index,struct himax_ts_data *ts)
{
	uint16_t x_position = 0, y_position = 0;
if ( tp_key_index != 0x00)
	{
		I("virtual key index =%x\n",tp_key_index);
		if ( tp_key_index == 0x01) {
			vk_press = 1;
			I("back key pressed\n");
				if (ts->pdata->virtual_key)
				{
					if (ts->button[0].index) {
						x_position = (ts->button[0].x_range_min + ts->button[0].x_range_max) / 2;
						y_position = (ts->button[0].y_range_min + ts->button[0].y_range_max) / 2;
					}
					if (ts->protocol_type == PROTOCOL_TYPE_A) {
						input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, 0);
						input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
							100);
						input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
							100);
						input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
							100);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
							x_position);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
							y_position);
						input_mt_sync(ts->input_dev);
					} else if (ts->protocol_type == PROTOCOL_TYPE_B) {
						input_mt_slot(ts->input_dev, 0);
						input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER,
						1);
						input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
							100);
						input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
							100);
						input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
							100);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
							x_position);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
							y_position);
					}
				}
				else
					input_report_key(ts->input_dev, KEY_BACK, 1);
		}
		else if ( tp_key_index == 0x02) {
			vk_press = 1;
			I("home key pressed\n");
				if (ts->pdata->virtual_key)
				{
					if (ts->button[1].index) {
						x_position = (ts->button[1].x_range_min + ts->button[1].x_range_max) / 2;
						y_position = (ts->button[1].y_range_min + ts->button[1].y_range_max) / 2;
					}
						if (ts->protocol_type == PROTOCOL_TYPE_A) {
						input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, 0);
						input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
							100);
						input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
							100);
						input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
							100);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
							x_position);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
							y_position);
						input_mt_sync(ts->input_dev);
					} else if (ts->protocol_type == PROTOCOL_TYPE_B) {
						input_mt_slot(ts->input_dev, 0);
						input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER,
						1);
						input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
							100);
						input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
							100);
						input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
							100);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
							x_position);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
							y_position);
					}
				}
				else
					input_report_key(ts->input_dev, KEY_HOME, 1);
		}
		else if ( tp_key_index == 0x04) {
			vk_press = 1;
			I("APP_switch key pressed\n");
				if (ts->pdata->virtual_key)
				{
					if (ts->button[2].index) {
						x_position = (ts->button[2].x_range_min + ts->button[2].x_range_max) / 2;
						y_position = (ts->button[2].y_range_min + ts->button[2].y_range_max) / 2;
					}
						if (ts->protocol_type == PROTOCOL_TYPE_A) {
						input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, 0);
						input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
							100);
						input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
							100);
						input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
							100);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
							x_position);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
							y_position);
						input_mt_sync(ts->input_dev);
					} else if (ts->protocol_type == PROTOCOL_TYPE_B) {
						input_mt_slot(ts->input_dev, 0);
						input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER,
						1);
						input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
							100);
						input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
							100);
						input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
							100);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
							x_position);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
							y_position);
					}
				}
				else
					input_report_key(ts->input_dev, KEY_MENU, 1);	
		}
		input_sync(ts->input_dev);
	}
else/*tp_key_index =0x00*/
	{
		I("virtual key released\n");
		vk_press = 0;
		if (ts->protocol_type == PROTOCOL_TYPE_A) {
			input_mt_sync(ts->input_dev);
		}
		else if (ts->protocol_type == PROTOCOL_TYPE_B) {
			input_mt_slot(ts->input_dev, 0);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
		}
		input_report_key(ts->input_dev, KEY_BACK, 0);
		input_report_key(ts->input_dev, KEY_HOME, 0);
		input_report_key(ts->input_dev, KEY_MENU, 0);
	input_sync(ts->input_dev);
	}
}

inline void himax_ts_work(void)
{
	int ret = 0;
	uint8_t buf[128], finger_num, hw_reset_check[2];
	uint16_t finger_pressed;
	uint8_t finger_on = 0;
	int32_t loop_i;
	unsigned char check_sum_cal = 0;
	int RawDataLen = 0;
	int raw_cnt_max ;
	int raw_cnt_rmd ;
	int hx_touch_info_size;
	int base,x,y,w;
	struct himax_ts_data *ts = private_ts;
	uint8_t coordInfoSize;
#ifdef HX_TP_PROC_DIAG
	uint8_t *mutual_data;
	uint8_t *self_data;
	uint8_t diag_cmd;
	int  	i;
	int 	mul_num;
	int 	self_num;
	int 	index = 0;
	int  	temp1, temp2;
	//coordinate dump start
	char coordinate_char[15+(HX_MAX_PT+5)*2*5+2];
	struct timeval t;
	struct tm broken;
	//coordinate dump end
#endif
#ifdef HX_CHIP_STATUS_MONITOR
		int j=0;
#endif

	memset(buf, 0x00, sizeof(buf));
	memset(hw_reset_check, 0x00, sizeof(hw_reset_check));
	coordInfoSize = ts->coord_data_size + ts->area_data_size + 4;
#if defined(HX_USB_DETECT2)
	himax_cable_detect_func();
#endif
	int_status=3; //int_status: himax_ts_work 20160425
#ifdef HX_CHIP_STATUS_MONITOR
		HX_CHIP_POLLING_COUNT=0;
		if(HX_ON_HAND_SHAKING)//chip on hand shaking,wait hand shaking
		{
			for(j=0; j<100; j++)
				{
					if(HX_ON_HAND_SHAKING==0)//chip on hand shaking end
						{
							I("%s:HX_ON_HAND_SHAKING OK check %d times\n",__func__,j);
							break;
						}
					else
						msleep(1);
				}
			if(j==100)
				{
					E("%s:HX_ON_HAND_SHAKING timeout reject interrupt\n",__func__);
					return;
				}
		}
#endif
	//check touch panel should be recover or not
#ifdef HX_TOUCH_REC
	checktouch=3;
	if(check_touchData()==1) //check data which is like MXPT,RX NUM,TX NUM and so on...if it's need torecover
	{
		goto err_need2recover;
	}
#endif

	raw_cnt_max = HX_MAX_PT/4;
	raw_cnt_rmd = HX_MAX_PT%4;

	if (raw_cnt_rmd != 0x00) //more than 4 fingers
	{
		RawDataLen = 128 - ((HX_MAX_PT+raw_cnt_max+3)*4) - 1;
		hx_touch_info_size = (HX_MAX_PT+raw_cnt_max+2)*4;
	}
	else //less than 4 fingers
	{
		RawDataLen = 128 - ((HX_MAX_PT+raw_cnt_max+2)*4) - 1;
		hx_touch_info_size = (HX_MAX_PT+raw_cnt_max+1)*4;
	}

#ifdef HX_TP_PROC_DIAG
	diag_cmd = getDiagCommand();
#ifdef HX_ESD_WORKAROUND
if((diag_cmd) || (ESD_RESET_ACTIVATE) || (HW_RESET_ACTIVATE))
#else
if((diag_cmd) || (HW_RESET_ACTIVATE))
#endif
	{
		ret = i2c_himax_read(ts->client, 0x86, buf, 128,DEFAULT_RETRY_CNT);
	}
	else{
		if(touch_monitor_stop_flag != 0){
			ret = i2c_himax_read(ts->client, 0x86, buf, 128,DEFAULT_RETRY_CNT);
			touch_monitor_stop_flag-- ;
		}
		else{
			ret = i2c_himax_read(ts->client, 0x86, buf, hx_touch_info_size,DEFAULT_RETRY_CNT);
		}
	}
	if (ret < 0)
#else
	if (i2c_himax_read(ts->client, 0x86, buf, hx_touch_info_size,DEFAULT_RETRY_CNT))
#endif
	{
		E("%s: can't read data from chip!\n", __func__);
		goto err_workqueue_out;
	}
	else
	{
#ifdef HX_ESD_WORKAROUND
			for(i = 0; i < hx_touch_info_size; i++)
			{
				if (buf[i] == 0x00) //case 2 ESD recovery flow-Disable
				{
					check_sum_cal = 1;
				}
				else if(buf[i] == 0xED)/*case 1 ESD recovery flow*/
				{
					check_sum_cal = 2;
				}
				else
				{
					check_sum_cal = 0;
					i = hx_touch_info_size;
					break;
				}
			}

			//IC status is abnormal ,do hand shaking
#ifdef HX_TP_PROC_DIAG
			diag_cmd = getDiagCommand();
#ifdef HX_ESD_WORKAROUND
			//20160216 for Speed Fw
			//if (check_sum_cal != 0 && ESD_RESET_ACTIVATE == 0 && HW_RESET_ACTIVATE == 0 && diag_cmd == 0)  //ESD Check
	if (check_sum_cal != 0 && ESD_RESET_ACTIVATE == 0 && HW_RESET_ACTIVATE == 0 && diag_cmd == 0 && self_test_inter_flag == 0)
#else
			if (check_sum_cal != 0 && diag_cmd == 0)
#endif
#else
#ifdef HX_ESD_WORKAROUND
			//20160216 for Speed Fw
			//if (check_sum_cal != 0 && ESD_RESET_ACTIVATE == 0 && HW_RESET_ACTIVATE == 0 )  //ESD Check
	if (check_sum_cal != 0 && ESD_RESET_ACTIVATE == 0 && HW_RESET_ACTIVATE == 0 && self_test_inter_flag == 0)
#else
			if (check_sum_cal !=0)
#endif
#endif
			{
				ret = himax_hand_shaking(); //0:Running, 1:Stop, 2:I2C Fail
				if (ret == 2)
				{
					goto err_workqueue_out;
				}

				if ((ret == 1) && (check_sum_cal == 1))
				{
					I("[HIMAX TP MSG]: ESD event checked - ALL Zero.\n");
					ESD_HW_REST();
				}
				else if (check_sum_cal == 2)
				{
					I("[HIMAX TP MSG]: ESD event checked - ALL 0xED.\n");
					ESD_HW_REST();
				}

				//himax_int_enable(ts->client->irq,1,true);
				return;
			}
			else if (ESD_RESET_ACTIVATE)
			{
				ESD_RESET_ACTIVATE = 0;/*drop 1st interrupts after chip reset*/
				I("[HIMAX TP MSG]:%s: Back from reset, ready to serve.\n", __func__);
				return;
			}
			else if (HW_RESET_ACTIVATE)
#else
			if (HW_RESET_ACTIVATE)
#endif
			{
				HW_RESET_ACTIVATE = 0;/*drop 1st interrupts after chip reset*/
				I("[HIMAX TP MSG]:%s: HW_RST Back from reset, ready to serve.\n", __func__);
				return;
			}

		for (loop_i = 0, check_sum_cal = 0; loop_i < hx_touch_info_size; loop_i++)
			check_sum_cal += buf[loop_i];

		if ((check_sum_cal != 0x00) )
		{
			I("[HIMAX TP MSG] checksum fail : check_sum_cal: 0x%02X\n", check_sum_cal);
			return;
		}
	}

	if (ts->debug_log_level & BIT(0)) {
		I("%s: raw data:\n", __func__);
		for (loop_i = 0; loop_i < hx_touch_info_size; loop_i++) {
			I("0x%2.2X ", buf[loop_i]);
			if (loop_i % 8 == 7)
				I("\n");
		}
	}

	//touch monitor raw data fetch
#ifdef HX_TP_PROC_DIAG
	diag_cmd = getDiagCommand();
	if (diag_cmd >= 1 && diag_cmd <= 6)
	{
		//Check 128th byte CRC
		for (i = hx_touch_info_size, check_sum_cal = 0; i < 128; i++)
		{
			check_sum_cal += buf[i];
		}
		if (check_sum_cal % 0x100 != 0)
		{
			goto bypass_checksum_failed_packet;
		}
#ifdef HX_TP_PROC_2T2R
		if(Is_2T2R && diag_cmd == 4)
		{
			mutual_data = getMutualBuffer_2();
			self_data 	= getSelfBuffer();

			// initiallize the block number of mutual and self
			mul_num = getXChannel_2() * getYChannel_2();

#ifdef HX_EN_SEL_BUTTON
			self_num = getXChannel_2() + getYChannel_2() + HX_BT_NUM;
#else
			self_num = getXChannel_2() + getYChannel_2();
#endif
		}
		else
#endif			
		{
			mutual_data = getMutualBuffer();
			self_data 	= getSelfBuffer();

			// initiallize the block number of mutual and self
			mul_num = getXChannel() * getYChannel();

#ifdef HX_EN_SEL_BUTTON
			self_num = getXChannel() + getYChannel() + HX_BT_NUM;
#else
			self_num = getXChannel() + getYChannel();
#endif
		}

		//Himax: Check Raw-Data Header
		if (buf[hx_touch_info_size] == buf[hx_touch_info_size+1] && buf[hx_touch_info_size+1] == buf[hx_touch_info_size+2]
		&& buf[hx_touch_info_size+2] == buf[hx_touch_info_size+3] && buf[hx_touch_info_size] > 0)
		{
			index = (buf[hx_touch_info_size] - 1) * RawDataLen;
			//I("Header[%d]: %x, %x, %x, %x, mutual: %d, self: %d\n", index, buf[56], buf[57], buf[58], buf[59], mul_num, self_num);
			for (i = 0; i < RawDataLen; i++)
			{
				temp1 = index + i;

				if (temp1 < mul_num)
				{ //mutual
					mutual_data[index + i] = buf[i + hx_touch_info_size+4];	//4: RawData Header
#ifdef HX_TP_SELF_TEST_DRIVER
					if (Selftest_flag == 1)
						mutual_bank[index + i] = buf[i + hx_touch_info_size+4];
#endif
				}
				else
				{//self
					temp1 = i + index;
					temp2 = self_num + mul_num;
					
					if (temp1 >= temp2)
					{
						break;
					}

					self_data[i+index-mul_num] = buf[i + hx_touch_info_size+4];	//4: RawData Header
#ifdef HX_TP_SELF_TEST_DRIVER
					if (Selftest_flag == 1)
						self_bank[i+index-mul_num] = buf[i + hx_touch_info_size+4];
#endif
				}
			}
		}
		else
		{
			I("[HIMAX TP MSG]%s: header format is wrong!\n", __func__);
		}
	}
	else if (diag_cmd == 7)
	{
		memcpy(&(diag_coor[0]), &buf[0], 128);
	}
	//coordinate dump start
	if (coordinate_dump_enable == 1)
	{
		for(i=0; i<(15 + (HX_MAX_PT+5)*2*5); i++)
		{
			coordinate_char[i] = 0x20;
		}
		coordinate_char[15 + (HX_MAX_PT+5)*2*5] = 0xD;
		coordinate_char[15 + (HX_MAX_PT+5)*2*5 + 1] = 0xA;
	}
	//coordinate dump end
bypass_checksum_failed_packet:
#endif
		EN_NoiseFilter = (buf[HX_TOUCH_INFO_POINT_CNT+2]>>3);
		//I("EN_NoiseFilter=%d\n",EN_NoiseFilter);
		EN_NoiseFilter = EN_NoiseFilter & 0x01;
		//I("EN_NoiseFilter2=%d\n",EN_NoiseFilter);
#if defined(CONFIG_TOUCHSCREEN_PROXIMITY)
	if(ts->pdata->proximity_bytp_enable){
			if ( (buf[HX_TOUCH_INFO_POINT_CNT] & 0x0F)  == 0x00 && (buf[HX_TOUCH_INFO_POINT_CNT+2]>>2 & 0x01) )
			{
				if(proximity_flag==0)
					{
						I(" %s near event trigger\n",__func__);
						touch_report_psensor_input_event(0);
						proximity_flag = 1;
					}
				wake_lock_timeout(&ts->ts_wake_lock, TS_WAKE_LOCK_TIMEOUT);
				return;
			}
		}
#endif
#if defined(HX_EN_SEL_BUTTON) || defined(HX_EN_MUT_BUTTON)
		tpd_key = (buf[HX_TOUCH_INFO_POINT_CNT+2]>>4);
		if (tpd_key == 0x0F)/*All (VK+AA)leave*/
		{
			tpd_key = 0x00;
		}
		//I("[DEBUG] tpd_key:  %x\r\n", tpd_key);
#else
		tpd_key = 0x00;
#endif

		p_point_num = hx_point_num;

		if (buf[HX_TOUCH_INFO_POINT_CNT] == 0xff)
			hx_point_num = 0;
		else
			hx_point_num = buf[HX_TOUCH_INFO_POINT_CNT] & 0x0f;

#ifdef PS_DISABLE_CALLPHONE_BUTTON_UI
		if (disable_pull_status_bar == true && phone_calling_status == PHONE_CALL_ON) {
			finger_pressed = buf[coordInfoSize - 2] << 8 | buf[coordInfoSize - 3];
			for (loop_i = 0; loop_i < ts->nFinger_support; loop_i++) {
				if (((finger_pressed >> loop_i) & 1) == 1) {
					base = loop_i * 4;
					x = buf[base] << 8 | buf[base + 1];
					y = (buf[base + 2] << 8 | buf[base + 3]);
					if(y < STATUS_BAR_AXIS_Y) {
						if (touch_on_statusbar_callphoneUI == false) {
							touch_on_statusbar_callphoneUI = true;
							input_mt_sync(ts->input_dev);
							input_report_key(ts->input_dev, BTN_TOUCH, finger_on);
							input_sync(ts->input_dev);
							I("%s: Touch_on_statusbar, release touch event\n", __func__);
						}
						return;
					} else {
						touch_on_statusbar_callphoneUI = false;
					}
#ifdef PS_DISABLE_SPECIFIC_AREA
					if(x > CALL_PHONE_UI_RANGE_X_MIN && y > CALL_PHONE_UI_RANGE_Y_MIN &&
						x < CALL_PHONE_UI_RANGE_X_MAX && y < CALL_PHONE_UI_RANGE_Y_MAX) {
						if (touch_on_UIbar_callphoneUI == false) {
							touch_on_UIbar_callphoneUI = true;
							input_mt_sync(ts->input_dev);
							input_report_key(ts->input_dev, BTN_TOUCH, finger_on);
							input_sync(ts->input_dev);
							I("%s: Touch_on_UIbar, release touch event\n", __func__);
						}
						return;
					} else {
						touch_on_UIbar_callphoneUI = false;
					}
#endif
				}
			}
		}
#endif
		// Touch Point information
		if (hx_point_num != 0 ) {
			if(vk_press == 0x00)
				{
					uint16_t old_finger = ts->pre_finger_mask;
					finger_num = buf[coordInfoSize - 4] & 0x0F;
					finger_pressed = buf[coordInfoSize - 2] << 8 | buf[coordInfoSize - 3];
					finger_on = 1;
					AA_press = 1;
					for (loop_i = 0; loop_i < ts->nFinger_support; loop_i++) {
						if (((finger_pressed >> loop_i) & 1) == 1) {
							base = loop_i * 4;
							x = buf[base] << 8 | buf[base + 1];
							y = (buf[base + 2] << 8 | buf[base + 3]);
							w = buf[(ts->nFinger_support * 4) + loop_i];
							finger_num--;

							if ((ts->debug_log_level & BIT(3)) > 0)
							{
								if ((((old_finger >> loop_i) ^ (finger_pressed >> loop_i)) & 1) == 1)
								{
									if (ts->useScreenRes)
									{
										I("status:%X, Screen:F:%02d Down, X:%d, Y:%d, W:%d, N:%d\n",
										finger_pressed, loop_i+1, x * ts->widthFactor >> SHIFTBITS,
										y * ts->heightFactor >> SHIFTBITS, w, EN_NoiseFilter);
									}
									else
									{
										I("status:%X, Raw:F:%02d Down, X:%d, Y:%d, W:%d, N:%d\n",
										finger_pressed, loop_i+1, x, y, w, EN_NoiseFilter);
									}
								}
							}

							if (ts->protocol_type == PROTOCOL_TYPE_B)
							{
								input_mt_slot(ts->input_dev, loop_i);
							}

							
							if (ts->protocol_type == PROTOCOL_TYPE_A)
							{
								input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, loop_i);
								/* input_mt_sync(ts->input_dev); */
							}
							else
							{
								ts->last_slot = loop_i;
								input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 1);
							}

							input_report_key(ts->input_dev, BTN_TOUCH, finger_on);
							input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
							input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
							input_report_abs(ts->input_dev, ABS_MT_PRESSURE, w);
							input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
							input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);

							if (ts->protocol_type == PROTOCOL_TYPE_A)
							{
								input_mt_sync(ts->input_dev);
							}

							if (!ts->first_pressed)
							{
								ts->first_pressed = 1;
								I("S1@%d, %d\n", x, y);
							}

							ts->pre_finger_data[loop_i][0] = x;
							ts->pre_finger_data[loop_i][1] = y;


							if (ts->debug_log_level & BIT(1))
								I("Finger %d=> X:%d, Y:%d W:%d, Z:%d, F:%d, N:%d\n",
									loop_i + 1, x, y, w, w, loop_i + 1, EN_NoiseFilter);						
						} else {
							if (ts->protocol_type == PROTOCOL_TYPE_B)
							{
								input_mt_slot(ts->input_dev, loop_i);
								input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
								if (ts->debug_log_level & BIT(1))
									I("All Finger leave_Clear_last_event\n");
							}

							if (loop_i == 0 && ts->first_pressed == 1)
							{
								ts->first_pressed = 2;
								I("E1@%d, %d\n",
								ts->pre_finger_data[0][0] , ts->pre_finger_data[0][1]);
							}

							if ((((old_finger >> loop_i) ^ (finger_pressed >> loop_i)) & 1) == 1)
							{
								if (ts->useScreenRes)
								{
									I("status:0x%02x, Screen:F:%02d Up, X:%d, Y:%d, N:%d\n",
									finger_pressed, loop_i+1, ts->pre_finger_data[loop_i][0] * ts->widthFactor >> SHIFTBITS,
									ts->pre_finger_data[loop_i][1] * ts->heightFactor >> SHIFTBITS, Last_EN_NoiseFilter);
								}
								else
								{
									I("status:0x%02x, Raw:F:%02d Up, X:%d, Y:%d, N:%d\n",
									finger_pressed, loop_i+1, ts->pre_finger_data[loop_i][0],
									ts->pre_finger_data[loop_i][1], Last_EN_NoiseFilter);
								}
							}
						}
					}
					ts->pre_finger_mask = finger_pressed;
				}else if ((tpd_key_old != 0x00)&&(tpd_key == 0x00)) {
					//temp_x[0] = 0xFFFF;
					//temp_y[0] = 0xFFFF;
					//temp_x[1] = 0xFFFF;
					//temp_y[1] = 0xFFFF;
					himax_ts_button_func(tpd_key,ts);
					finger_on = 0;
				}
#ifdef HX_ESD_WORKAROUND
				ESD_COUNTER = 0;
#endif
			input_report_key(ts->input_dev, BTN_TOUCH, finger_on);
			input_sync(ts->input_dev);
		} else if (hx_point_num == 0){
#if defined(HX_PALM_REPORT)
			loop_i = 0;
			base = loop_i * 4;
			x = buf[base] << 8 | buf[base + 1];
			y = (buf[base + 2] << 8 | buf[base + 3]);
			w = buf[(ts->nFinger_support * 4) + loop_i];
			I(" %s HX_PALM_REPORT_loopi=%d,base=%x,X=%x,Y=%x,W=%x \n",__func__,loop_i,base,x,y,w);

			if((!atomic_read(&ts->suspend_mode))&&(x==0xFA5A)&&(y==0xFA5A)&&(w==0x00))
				{
					I(" %s HX_PALM_REPORT KEY power event press\n",__func__);
					input_report_key(ts->input_dev, KEY_POWER, 1);
					input_sync(ts->input_dev);
					msleep(100);
					I(" %s HX_PALM_REPORT KEY power event release\n",__func__);
					input_report_key(ts->input_dev, KEY_POWER, 0);
					input_sync(ts->input_dev);
					return;
				}
#endif
#if defined(CONFIG_TOUCHSCREEN_PROXIMITY)
			if ((ts->pdata->proximity_bytp_enable)&&(proximity_flag))//Proximity Far event
				{
					I(" %s far event trigger\n",__func__);
					touch_report_psensor_input_event(1);
					proximity_flag = 0;	//clear flag , avoid touch point cant leave.
					wake_lock_timeout(&ts->ts_wake_lock, TS_WAKE_LOCK_TIMEOUT);
				}
			else if(AA_press)
#else
			if(AA_press)
#endif
				{
				// leave event
				finger_on = 0;
				AA_press = 0;
				if (ts->protocol_type == PROTOCOL_TYPE_A)
					input_mt_sync(ts->input_dev);

				for (loop_i = 0; loop_i < ts->nFinger_support; loop_i++) {
						if (((ts->pre_finger_mask >> loop_i) & 1) == 1) {
							if (ts->protocol_type == PROTOCOL_TYPE_B) {
								input_mt_slot(ts->input_dev, loop_i);
								input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
							}
						}
					}
				if (ts->pre_finger_mask > 0) {
					for (loop_i = 0; loop_i < ts->nFinger_support; loop_i++) {
						if (((ts->pre_finger_mask >> loop_i) & 1) == 1) {
							if (ts->useScreenRes) {
								I("status:0x%02x, Screen:F:%02d Up, X:%d, Y:%d, N:%d\n", 0, loop_i+1, ts->pre_finger_data[loop_i][0] * ts->widthFactor >> SHIFTBITS,
									ts->pre_finger_data[loop_i][1] * ts->heightFactor >> SHIFTBITS, Last_EN_NoiseFilter);
							} else {
								I("status:0x%02x, Raw:F:%02d Up, X:%d, Y:%d, N:%d\n",0, loop_i+1, ts->pre_finger_data[loop_i][0],ts->pre_finger_data[loop_i][1], Last_EN_NoiseFilter);
							}
						}
					}
					ts->pre_finger_mask = 0;
				}

				if (ts->first_pressed == 1) {
					ts->first_pressed = 2;
					I("E1@%d, %d\n",ts->pre_finger_data[0][0] , ts->pre_finger_data[0][1]);
				}

				if (ts->debug_log_level & BIT(1))
					I("All Finger leave\n");

#ifdef HX_TP_PROC_DIAG
					//coordinate dump start
					if (coordinate_dump_enable == 1)
					{
						do_gettimeofday(&t);
						time_to_tm(t.tv_sec, 0, &broken);

						sprintf(&coordinate_char[0], "%2d:%2d:%2d:%lu,", broken.tm_hour, broken.tm_min, broken.tm_sec, t.tv_usec/1000);
						sprintf(&coordinate_char[15], "Touch up!");
						coordinate_fn->f_op->write(coordinate_fn,&coordinate_char[0],15 + (HX_MAX_PT+5)*2*sizeof(char)*5 + 2,&coordinate_fn->f_pos);
					}
					//coordinate dump end
#endif
			}
			else if (tpd_key != 0x00) {
				//report key
				//temp_x[0] = 0xFFFF;
				//temp_y[0] = 0xFFFF;
				//temp_x[1] = 0xFFFF;
				//temp_y[1] = 0xFFFF;
				himax_ts_button_func(tpd_key,ts);
				finger_on = 1;
			}
			else if ((tpd_key_old != 0x00)&&(tpd_key == 0x00)) {
				//temp_x[0] = 0xFFFF;
				//temp_y[0] = 0xFFFF;
				//temp_x[1] = 0xFFFF;
				//temp_y[1] = 0xFFFF;
				himax_ts_button_func(tpd_key,ts);
				finger_on = 0;
			}
#ifdef HX_ESD_WORKAROUND
				ESD_COUNTER = 0;
#endif
			input_report_key(ts->input_dev, BTN_TOUCH, finger_on);
			input_sync(ts->input_dev);
		}
		tpd_key_old = tpd_key;
		Last_EN_NoiseFilter = EN_NoiseFilter;

workqueue_out:
	return;

err_workqueue_out:
	I("%s: Now reset the Touch chip.\n", __func__);

#ifdef HX_RST_PIN_FUNC
	himax_HW_reset(true,false);
#endif

	goto workqueue_out;
	
#ifdef HX_TOUCH_REC
	err_need2recover:

	while(checktouch>0)
	{
		if(check_touchData()==1)  //check data which is like MXPT,RX NUM,TX NUM and so on... if it's need torecover
		{
			TP_REC();
			checktouch--;
		}
		else{
			I("Recover Success!!!");
			break;
		}
	}
	I("%s: Recover FAIL!!!",__func__);
	goto workqueue_out;
#endif
	
}

static enum hrtimer_restart himax_ts_timer_func(struct hrtimer *timer)
{
	struct himax_ts_data *ts;

	ts = container_of(timer, struct himax_ts_data, timer);
	queue_work(ts->himax_wq, &ts->work);
	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static void himax_ts_work_func(struct work_struct *work)
{
	himax_ts_work();
}

#ifdef CONFIG_OF_TOUCH
static irqreturn_t tpd_eint_interrupt_handler(int irq, void *desc)
{
	tpd_flag = 1;
	/* enter EINT handler disable INT, make sure INT is disable when handle touch event including top/bottom half */
	/* use _nosync to avoid deadlock */
	himax_int_enable(private_ts->client->irq, 0, false);
	int_status=1; //int_status: tpd_eint_interrupt_handler 20160425
	wake_up_interruptible(&waiter);
    return IRQ_HANDLED;
}
#else
static void tpd_eint_interrupt_handler(void)
{
	tpd_flag = 1;
	wake_up_interruptible(&waiter);
}
#endif
static int touch_event_handler(void *ptr)
{
#ifdef HX_SMART_WAKEUP
	int ret_event = 0, KEY_EVENT = 0;
#endif
	struct timespec timeStart, timeEnd, timeDelta;
	struct sched_param param = {.sched_priority = 4};
	
	sched_setscheduler(current, SCHED_RR, &param);

	do
	{
		set_current_state(TASK_INTERRUPTIBLE); 
		wait_event_interruptible(waiter, tpd_flag != 0);
		int_status=2; //int_status: touch_event_handler 20160425
		tpd_flag = 0;
		set_current_state(TASK_RUNNING);

		if (private_ts->debug_log_level & BIT(2)) {
				getnstimeofday(&timeStart);
				/*I(" Irq start time = %ld.%06ld s\n",
					timeStart.tv_sec, timeStart.tv_nsec/1000);*/
		}
#ifdef HX_SMART_WAKEUP
	if (atomic_read(&private_ts->suspend_mode) && (!FAKE_POWER_KEY_SEND) && (private_ts->SMWP_enable)) {
		I("Start to parse wake event\n");
		wake_lock_timeout(&private_ts->ts_SMWP_wake_lock, TS_WAKE_LOCK_TIMEOUT);
		msleep(200);
		ret_event = himax_parse_wake_event(private_ts);
		switch (ret_event) {
			case EV_GESTURE_PWR:
				KEY_EVENT = KEY_POWER;
			break;
			case EV_GESTURE_01:
				KEY_EVENT = KEY_CUST_01;
			break;
			case EV_GESTURE_02:
				KEY_EVENT = KEY_CUST_02;
			break;
			case EV_GESTURE_03:
				KEY_EVENT = KEY_CUST_03;
			break;
			case EV_GESTURE_04:
				KEY_EVENT = KEY_CUST_04;
			break;
			case EV_GESTURE_05:
				KEY_EVENT = KEY_CUST_05;
			break;
			case EV_GESTURE_06:
				KEY_EVENT = KEY_CUST_06;
			break;
			case EV_GESTURE_07:
				KEY_EVENT = KEY_CUST_07;
			break;
			case EV_GESTURE_08:
				KEY_EVENT = KEY_CUST_08;
			break;
			case EV_GESTURE_09:
				KEY_EVENT = KEY_CUST_09;
			break;
			case EV_GESTURE_10:
				KEY_EVENT = KEY_CUST_10;
			break;
			case EV_GESTURE_11:
				KEY_EVENT = KEY_CUST_11;
			break;
			case EV_GESTURE_12:
				KEY_EVENT = KEY_CUST_12;
			break;
			case EV_GESTURE_13:
				KEY_EVENT = KEY_CUST_13;
			break;
			case EV_GESTURE_14:
				KEY_EVENT = KEY_CUST_14;
			break;
			case EV_GESTURE_15:
				KEY_EVENT = KEY_CUST_15;
			break;
			default:
				E("%s: un-define KEY event 0x%02x\n", __func__, ret_event);
			break;
		}

		if(KEY_EVENT) {
				input_report_key(private_ts->input_dev, KEY_EVENT, 1);
				input_sync(private_ts->input_dev);
				msleep(10);
				input_report_key(private_ts->input_dev, KEY_EVENT, 0);
				input_sync(private_ts->input_dev);
				I("%s: SMART WAKEUP KEY event 0x%02x finish\n", __func__, KEY_EVENT);
				FAKE_POWER_KEY_SEND = true;
				KEY_EVENT = 0;
		}

		himax_int_enable(private_ts->client->irq,1,false);
		continue;
	}
#endif
		himax_ts_work();
		int_status=4; //int_status after do himax_ts_work 20160425
		if(private_ts->debug_log_level & BIT(2)) {
				getnstimeofday(&timeEnd);
					timeDelta.tv_nsec = (timeEnd.tv_sec*1000000000+timeEnd.tv_nsec)
					-(timeStart.tv_sec*1000000000+timeStart.tv_nsec);
				/*I("Irq finish time = %ld.%06ld s\n",
					timeEnd.tv_sec, timeEnd.tv_nsec/1000);*/
				I("Touch latency = %ld us\n", timeDelta.tv_nsec/1000);
		}
		himax_int_enable(private_ts->client->irq,1,false);
	}
	while(!kthread_should_stop());
	int_status=5;
	return 0;
}

int himax_ts_register_interrupt(struct i2c_client *client)
{
	struct device_node *node = NULL;
	struct himax_ts_data *ts = i2c_get_clientdata(client);
	uint8_t i=0, j=0, buf[128];
	u32 ints[2] = {0, 0};
	int ret = 0;
	I("Himax_interrupt = %s\n", __func__);
	node = of_find_matching_node(node, touch_of_match);
	if (node) {
		/* touch_irq = gpio_to_irq(tpd_int_gpio_number); */
		of_property_read_u32_array(node,"debounce", ints, ARRAY_SIZE(ints));
		gpio_set_debounce(ints[0], ints[1]);

		himax_touch_irq = irq_of_parse_and_map(node, 0);	
		
		I("himax_touch_irq=%ud \n",himax_touch_irq);
		
		client->irq = himax_touch_irq;
		ts->client->irq = himax_touch_irq;		
	} else {
		I("[%s] tpd request_irq can not find touch eint device node!.", __func__);
	}
	
	ts->irq_enabled = 0;
	ts->use_irq = 0;

	if(himax_int_gpio_read(ts->pdata->gpio_irq) == 0) {
		for (i=0;i<50;i++) {
			ret = i2c_himax_read(ts->client, 0x86, buf, 128, DEFAULT_RETRY_CNT);
			msleep(10);
			if(himax_int_gpio_read(ts->pdata->gpio_irq)) {
				I("%s event stack has been clear\n ", __func__);
				j++;
				if (j>5) break;
			}
		}
	}

	//Work functon
	if (client->irq) {	/*INT mode*/
		ts->use_irq = 1;
		if(HX_INT_IS_EDGE) {
			I("%s edge triiger falling\n ",__func__);
#ifdef CONFIG_OF_TOUCH
			ret = request_irq(client->irq, tpd_eint_interrupt_handler, IRQF_TRIGGER_FALLING, "TOUCH_PANEL-eint", NULL);
			if(ret > 0){
			    ret = -1;
			E("tpd request_irq IRQ LINE NOT AVAILABLE\n");
			}
#else
			//mt_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_TYPE);
			//mt_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
			mt_eint_registration(client->irq, IRQF_TRIGGER_FALLING, tpd_eint_interrupt_handler, 1);
#endif
		} else {
			I("%s level trigger low\n ",__func__);
#ifdef CONFIG_OF_TOUCH
			ret = request_irq(client->irq, tpd_eint_interrupt_handler, IRQF_TRIGGER_LOW, "TOUCH_PANEL-eint", NULL);
			if(ret > 0){
				ret = -1;
				E("tpd request_irq IRQ LINE NOT AVAILABLE\n");
			}
#else
			//mt_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_TYPE);
			//mt_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
			mt_eint_registration(client->irq, IRQF_TRIGGER_LOW, tpd_eint_interrupt_handler, 1);
#endif
		}
		
		ts->irq_enabled = 1;
		irq_enable_count = 1;
		//I("%s: irq register at qpio: %02X\n", __func__, pdata->gpio_irq);
#ifdef HX_SMART_WAKEUP
		ret = irq_set_irq_wake(client->irq, 1);

		I("%s: gpio: %02X, set irq wake for himax smart wake up, ret = %d\n",
			__func__, client->irq, ret);
#endif		
		touch_thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
		if (IS_ERR(touch_thread))
		{
			ret = PTR_ERR(touch_thread);
			E(" Failed to create kernel thread: %d\n", ret);
			return ret;
		}
	}
	else {
		I("%s: client->irq is empty, use polling mode.\n", __func__);
	}

	if (!ts->use_irq) {/*if use polling mode need to disable HX_ESD_WORKAROUND function*/
		ts->himax_wq = create_singlethread_workqueue("himax_touch");

		INIT_WORK(&ts->work, himax_ts_work_func);

		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = himax_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
		I("%s: polling mode enabled\n", __func__);
	}
	tpd_load_status = 1;
	return 0;
}

#if defined(HX_USB_DETECT)
static void himax_cable_tp_status_handler_func(int connect_status)
{
	struct himax_ts_data *ts;
	I("Touch: cable change to %d\n", connect_status);
	ts = private_ts;
	if (ts->cable_config) {
		if (!atomic_read(&ts->suspend_mode)) {
			if ((!!connect_status) != ts->usb_connected) {
				if (!!connect_status) {
					ts->cable_config[1] = 0x01;
					ts->usb_connected = 0x01;
				} else {
					ts->cable_config[1] = 0x00;
					ts->usb_connected = 0x00;
				}

				i2c_himax_master_write(ts->client, ts->cable_config,
					sizeof(ts->cable_config), DEFAULT_RETRY_CNT);

				I("%s: Cable status change: 0x%2.2X\n", __func__, ts->cable_config[1]);
			} else
				I("%s: Cable status is the same as previous one, ignore.\n", __func__);
		} else {
			if (connect_status)
				ts->usb_connected = 0x01;
			else
				ts->usb_connected = 0x00;
			I("%s: Cable status remembered: 0x%2.2X\n", __func__, ts->usb_connected);
		}
	}
}

static struct t_cable_status_notifier himax_cable_status_handler = {
	.name = "usb_tp_connected",
	.func = himax_cable_tp_status_handler_func,
};
#endif

#if defined(HX_USB_DETECT2)
static void himax_cable_detect_func(void)
{
	struct himax_ts_data *ts;
	u32 connect_status = 0;

	connect_status = upmu_is_chr_det();
	//I("Touch: cable status%d\n", connect_status);
	ts = private_ts;
	if (ts->cable_config) {
		if ((!!connect_status) != ts->usb_connected) {
			if (!!connect_status) {
				ts->cable_config[1] = 0x01;
				ts->usb_connected = 0x01;
			} else {
				ts->cable_config[1] = 0x00;
				ts->usb_connected = 0x00;
			}

			i2c_himax_master_write(ts->client, ts->cable_config,
				sizeof(ts->cable_config), DEFAULT_RETRY_CNT);

			I("%s: Cable status change: 0x%2.2X\n", __func__, ts->cable_config[1]);
			}
		//else
		//	I("%s: Cable status is the same as previous one, ignore.\n", __func__);
	}
}
#endif
/* 20160506
#ifdef CONFIG_FB
static void himax_fb_register(struct work_struct *work)
{
	int ret = 0;
	struct himax_ts_data *ts = container_of(work, struct himax_ts_data,
							work_att.work);
	I(" %s in", __func__);

	ts->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&ts->fb_notif);
	if (ret)
		E(" Unable to register fb_notifier: %d\n", ret);
}
#endif
*/
#if defined(CONFIG_TOUCHSCREEN_PROXIMITY)
int proximity_enable_from_ps(int on)
{
	char buf_tmp[5];
	if (on)
	{
		touch_report_psensor_input_event(1);//Workaround for screen off on phone APP
		buf_tmp[0] = 0x92;
		buf_tmp[1] = 0x01;
		g_proximity_en=1;
		enable_irq_wake(private_ts->client->irq);
	}
	else
	{
		buf_tmp[0] = 0x92;
		buf_tmp[1] = 0x00;
		g_proximity_en=0;
		disable_irq_wake(private_ts->client->irq);
	}

	I("Proximity=%d\n",on);
	i2c_himax_master_write(private_ts->client, buf_tmp, 2, DEFAULT_RETRY_CNT);

	return 0;
}
EXPORT_SYMBOL_GPL(proximity_enable_from_ps);
#endif

//=============================================================================================================
//
//	Segment : Himax SYS Debug Function
//
//=============================================================================================================
#if defined(CONFIG_TOUCHSCREEN_HIMAX_DEBUG)

static ssize_t himax_debug_level_read(struct file *file, char *buf,
	size_t len, loff_t *pos)
{
	struct himax_ts_data *ts_data;
	ssize_t ret = 0;

	ts_data = private_ts;
	if(!HX_PROC_SEND_FLAG)
		{
			ret += sprintf(buf, "%d\n", ts_data->debug_log_level);
			HX_PROC_SEND_FLAG=1;
		}
	else
		HX_PROC_SEND_FLAG=0;

	return ret;
}

static ssize_t himax_debug_level_write(struct file *file, const char *buff,
	size_t len, loff_t *pos)
{
	struct himax_ts_data *ts;
	char buf_tmp[12]= {0};
	int i;

	if (len >= 12)
	{
		I("%s: no command exceeds 12 chars.\n", __func__);
		return -EFAULT;
	}
	if (copy_from_user(buf_tmp, buff, len))
	{
		return -EFAULT;
	}
	ts = private_ts;

	ts->debug_log_level = 0;
	for(i=0; i<len-1; i++)
	{
		if( buf_tmp[i]>='0' && buf_tmp[i]<='9' )
			ts->debug_log_level |= (buf_tmp[i]-'0');
		else if( buf_tmp[i]>='A' && buf_tmp[i]<='F' )
			ts->debug_log_level |= (buf_tmp[i]-'A'+10);
		else if( buf_tmp[i]>='a' && buf_tmp[i]<='f' )
			ts->debug_log_level |= (buf_tmp[i]-'a'+10);

		if(i!=len-2)
			ts->debug_log_level <<= 4;
	}

	if (ts->debug_log_level & BIT(3)) {
		if (ts->pdata->screenWidth > 0 && ts->pdata->screenHeight > 0 &&
		 (ts->pdata->abs_x_max - ts->pdata->abs_x_min) > 0 &&
		 (ts->pdata->abs_y_max - ts->pdata->abs_y_min) > 0) {
			ts->widthFactor = (ts->pdata->screenWidth << SHIFTBITS)/(ts->pdata->abs_x_max - ts->pdata->abs_x_min);
			ts->heightFactor = (ts->pdata->screenHeight << SHIFTBITS)/(ts->pdata->abs_y_max - ts->pdata->abs_y_min);
			if (ts->widthFactor > 0 && ts->heightFactor > 0)
				ts->useScreenRes = 1;
			else {
				ts->heightFactor = 0;
				ts->widthFactor = 0;
				ts->useScreenRes = 0;
			}
		} else
			I("Enable finger debug with raw position mode!\n");
	} else {
		ts->useScreenRes = 0;
		ts->widthFactor = 0;
		ts->heightFactor = 0;
	}

	return len;
}

static struct file_operations himax_proc_debug_level_ops =
{
	.owner = THIS_MODULE,
	.read = himax_debug_level_read,
	.write = himax_debug_level_write,
};

static ssize_t himax_vendor_read(struct file *file, char *buf,
	size_t len, loff_t *pos)
{
	ssize_t ret = 0;
	struct himax_ts_data *ts_data;
	ts_data = private_ts;
	if(!HX_PROC_SEND_FLAG)
		{
			ret += sprintf(buf, "%s_FW:%#x,%x_CFG:%#x_SensorId:%#x\n", HIMAX852xes_NAME,
				ts_data->vendor_fw_ver_H, ts_data->vendor_fw_ver_L, ts_data->vendor_config_ver, ts_data->vendor_sensor_id);
			HX_PROC_SEND_FLAG=1;
		}
	else
		HX_PROC_SEND_FLAG=0;

	return ret;
}

static struct file_operations himax_proc_vendor_ops =
{
	.owner = THIS_MODULE,
	.read = himax_vendor_read,	
};

static ssize_t himax_attn_read(struct file *file, char *buf,
	size_t len, loff_t *pos)
{
	ssize_t ret = 0;
	struct himax_ts_data *ts_data;
	ts_data = private_ts;

	if(!HX_PROC_SEND_FLAG)
		{
			sprintf(buf, "attn = %x\n", himax_int_gpio_read(ts_data->pdata->gpio_irq));
			ret = strlen(buf) + 1;
			HX_PROC_SEND_FLAG=1;
		}
	else
		HX_PROC_SEND_FLAG=0;

	return ret;
}

static struct file_operations himax_proc_attn_ops =
{
	.owner = THIS_MODULE,
	.read = himax_attn_read,	
};

static ssize_t himax_int_en_read(struct file *file, char *buf,
	size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = private_ts;
	struct irq_desc *desc=irq_to_desc(private_ts->client->irq);
	int depth = desc->depth;
	size_t ret = 0;

	if (!HX_PROC_SEND_FLAG) {
		ret += sprintf(buf + ret, "irq_enabled : %d\n", ts->irq_enabled);
		ret += sprintf(buf + ret, "irq_depth=%d\n", depth);

		HX_PROC_SEND_FLAG=1;
	} else {
		HX_PROC_SEND_FLAG=0;
	}

	return ret;
}

/* CT41 BUG 1147, to prevent false touch when Calling */
#ifdef PS_DISABLE_CALLPHONE_BUTTON_UI
void disable_pull_status_bar_by_Y_axis(bool enable)
{
	if (enable)
		I("%s: disable_pull_status_bar Enable\n", __func__);
	else
		I("%s: disable_pull_status_bar Disable\n", __func__);

	disable_pull_status_bar = enable;
}
#endif

static ssize_t himax_int_en_write(struct file *file, const char *buff,
	size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = private_ts;
	char buf_tmp[12]= {0};
	int value, ret=0;

	if (len >= 12)
	{
		I("%s: no command exceeds 12 chars.\n", __func__);
		return -EFAULT;
	}
	if (copy_from_user(buf_tmp, buff, len))
	{
		return -EFAULT;
	}

	if (buf_tmp[0] == '0')
		value = false;
	else if (buf_tmp[0] == '1')
		value = true;
	else
		return -EINVAL;
	if (value) {
				if(HX_INT_IS_EDGE)
				{
#ifdef CONFIG_OF_TOUCH
					himax_int_enable(ts->client->irq,1,true);
#else
					//mt_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_TYPE);
					//mt_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
					mt_eint_registration(ts->client->irq, EINTF_TRIGGER_FALLING, tpd_eint_interrupt_handler, 1);
#endif
				} else {
#ifdef CONFIG_OF_TOUCH
					himax_int_enable(ts->client->irq,1,true);
#else
					//mt_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_TYPE);
					//mt_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
					mt_eint_registration(ts->client->irq, EINTF_TRIGGER_LOW, tpd_eint_interrupt_handler, 1);
#endif
				}
		if (ret == 0) {
			ts->irq_enabled = 1;
			irq_enable_count = 1;
		}
	} else {
		himax_int_enable(ts->client->irq,0,true);
		free_irq(ts->client->irq, ts);
		ts->irq_enabled = 0;
	}

	return len;
}

static struct file_operations himax_proc_int_en_ops =
{
	.owner = THIS_MODULE,
	.read = himax_int_en_read,
	.write = himax_int_en_write,
};

static ssize_t himax_layout_read(struct file *file, char *buf,
	size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = private_ts;
	size_t ret = 0;
	
	if(!HX_PROC_SEND_FLAG)
		{
			ret += sprintf(buf + ret, "%d ", ts->pdata->abs_x_min);
			ret += sprintf(buf + ret, "%d ", ts->pdata->abs_x_max);
			ret += sprintf(buf + ret, "%d ", ts->pdata->abs_y_min);
			ret += sprintf(buf + ret, "%d ", ts->pdata->abs_y_max);
			ret += sprintf(buf + ret, "\n");
			HX_PROC_SEND_FLAG=1;
		}
	else
		HX_PROC_SEND_FLAG=0;

	return ret;
}

static ssize_t himax_layout_write(struct file *file, const char *buff,
	size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = private_ts;
	char buf_tmp[5];
	int i = 0, j = 0, k = 0, ret;
	unsigned long value;
	int layout[4] = {0};
	char buf[80] = {0};

	if (len >= 80)
	{
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}
	if (copy_from_user(buf, buff, len))
	{
		return -EFAULT;
	}

	for (i = 0; i < 20; i++) {
		if (buf[i] == ',' || buf[i] == '\n') {
			memset(buf_tmp, 0x0, sizeof(buf_tmp));
			if (i - j <= 5)
				memcpy(buf_tmp, buf + j, i - j);
			else {
				I("buffer size is over 5 char\n");
				return len;
			}
			j = i + 1;
			if (k < 4) {
				ret = kstrtoul(buf_tmp, 10, &value);
				layout[k++] = value;
			}
		}
	}
	if (k == 4) {
		ts->pdata->abs_x_min=layout[0];
		ts->pdata->abs_x_max=layout[1];
		ts->pdata->abs_y_min=layout[2];
		ts->pdata->abs_y_max=layout[3];
		I("%d, %d, %d, %d\n",
			ts->pdata->abs_x_min, ts->pdata->abs_x_max, ts->pdata->abs_y_min, ts->pdata->abs_y_max);
		input_unregister_device(ts->input_dev);
		himax_input_register(ts);
	} else
		I("ERR@%d, %d, %d, %d\n",
			ts->pdata->abs_x_min, ts->pdata->abs_x_max, ts->pdata->abs_y_min, ts->pdata->abs_y_max);
	return len;
}

static struct file_operations himax_proc_layout_ops =
{
	.owner = THIS_MODULE,
	.read = himax_layout_read,
	.write = himax_layout_write,
};

#ifdef HX_TP_PROC_RESET
static ssize_t himax_reset_write(struct file *file, const char *buff,
	size_t len, loff_t *pos)
{
	char buf_tmp[12];

	if (len >= 12)
	{
		I("%s: no command exceeds 12 chars.\n", __func__);
		return -EFAULT;
	}
	if (copy_from_user(buf_tmp, buff, len))
	{
		return -EFAULT;
	}

	if (buf_tmp[0] == '1')
		himax_HW_reset(true,false);
	//else if (buf_tmp[0] == '2')
	//	himax_HW_reset(true,true);
	//else if (buf_tmp[0] == '3')
	//	himax_HW_reset(false,true);
	//else if (buf_tmp[0] == '4')
	//	ESD_HW_REST();
	return len;
}

static struct file_operations himax_proc_reset_ops =
{
	.owner = THIS_MODULE,
	.write = himax_reset_write,
};
#endif

#ifdef HX_TP_PROC_DIAG
static uint8_t *getMutualBuffer(void)
{
	return diag_mutual;
}
static uint8_t *getSelfBuffer(void)
{
	return &diag_self[0];
}
static uint8_t getXChannel(void)
{
	return x_channel;
}
static uint8_t getYChannel(void)
{
	return y_channel;
}
static uint8_t getDiagCommand(void)
{
	return diag_command;
}
static void setXChannel(uint8_t x)
{
	x_channel = x;
}
static void setYChannel(uint8_t y)
{
	y_channel = y;
}
static void setMutualBuffer(void)
{
	diag_mutual = kzalloc(x_channel * y_channel * sizeof(uint8_t), GFP_KERNEL);
}

#ifdef HX_TP_PROC_2T2R
static uint8_t *getMutualBuffer_2(void)
{
	return diag_mutual_2;
}
static uint8_t getXChannel_2(void)
{
	return x_channel_2;
}
static uint8_t getYChannel_2(void)
{
	return y_channel_2;
}
static void setXChannel_2(uint8_t x)
{
	x_channel_2 = x;
}
static void setYChannel_2(uint8_t y)
{
	y_channel_2 = y;
}
static void setMutualBuffer_2(void)
{
	diag_mutual_2 = kzalloc(x_channel_2 * y_channel_2 * sizeof(uint8_t), GFP_KERNEL);
}
#endif
static void *himax_diag_seq_start(struct seq_file *s, loff_t *pos)
{
	if (*pos>=1) return NULL;
	return (void *)((unsigned long) *pos+1);
}

static void *himax_diag_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	return NULL;
}
static void himax_diag_seq_stop(struct seq_file *s, void *v)
{
}
static int himax_diag_seq_read(struct seq_file *s, void *v)
{
	size_t count = 0;
	uint32_t loop_i;
	uint16_t mutual_num, self_num, width;
#ifdef HX_TP_PROC_2T2R
	if(Is_2T2R && diag_command == 4)
	{
		mutual_num	= x_channel_2 * y_channel_2;
		self_num	= x_channel_2 + y_channel_2; //don't add KEY_COUNT
		width		= x_channel_2;
		seq_printf(s, "ChannelStart: %4d, %4d\n\n", x_channel_2, y_channel_2);
	}
	else
#endif
	{
		mutual_num	= x_channel * y_channel;
		self_num	= x_channel + y_channel; //don't add KEY_COUNT
		width		= x_channel;
		seq_printf(s, "ChannelStart: %4d, %4d\n\n", x_channel, y_channel);
	}

	// start to show out the raw data in adb shell
	if (diag_command >= 1 && diag_command <= 6) {
		if (diag_command <= 3) {
			for (loop_i = 0; loop_i < mutual_num; loop_i++) {
				seq_printf(s, "%4d", diag_mutual[loop_i]);
				if ((loop_i % width) == (width - 1))
					seq_printf(s, " %3d\n", diag_self[width + loop_i/width]);
			}
			seq_printf(s, "\n");
			for (loop_i = 0; loop_i < width; loop_i++) {
				seq_printf(s, "%4d", diag_self[loop_i]);
				if (((loop_i) % width) == (width - 1))
					seq_printf(s, "\n");
			}
#ifdef HX_EN_SEL_BUTTON
			seq_printf(s, "\n");
			for (loop_i = 0; loop_i < HX_BT_NUM; loop_i++)
					seq_printf(s, "%4d", diag_self[HX_RX_NUM + HX_TX_NUM + loop_i]);
#endif
#ifdef HX_TP_PROC_2T2R
		}else if(Is_2T2R && diag_command == 4 ) {
			for (loop_i = 0; loop_i < mutual_num; loop_i++) {
				seq_printf(s, "%4d", diag_mutual_2[loop_i]);
				if ((loop_i % width) == (width - 1))
					seq_printf(s, " %3d\n", diag_self[width + loop_i/width]);
			}
			seq_printf(s, "\n");
			for (loop_i = 0; loop_i < width; loop_i++) {
				seq_printf(s, "%4d", diag_self[loop_i]);
				if (((loop_i) % width) == (width - 1))
					seq_printf(s, "\n");
			}

#ifdef HX_EN_SEL_BUTTON
			seq_printf(s, "\n");
			for (loop_i = 0; loop_i < HX_BT_NUM; loop_i++)
				seq_printf(s, "%4d", diag_self[HX_RX_NUM_2 + HX_TX_NUM_2 + loop_i]);
#endif
#endif
		} else if (diag_command > 4) {
			for (loop_i = 0; loop_i < self_num; loop_i++) {
				seq_printf(s, "%4d", diag_self[loop_i]);
				if (((loop_i - mutual_num) % width) == (width - 1))
					seq_printf(s, "\n");
			}
		} else {
			for (loop_i = 0; loop_i < mutual_num; loop_i++) {
				seq_printf(s, "%4d", diag_mutual[loop_i]);
				if ((loop_i % width) == (width - 1))
					seq_printf(s, "\n");
			}
		}
		seq_printf(s, "ChannelEnd");
		seq_printf(s, "\n");
	} else if (diag_command == 7) {
		for (loop_i = 0; loop_i < 128 ;loop_i++) {
			if ((loop_i % 16) == 0)
				seq_printf(s, "LineStart:");
				seq_printf(s, "%4d", diag_coor[loop_i]);
			if ((loop_i % 16) == 15)
				seq_printf(s, "\n");
		}
	}
	return count;
}

static struct seq_operations himax_diag_seq_ops =
{
	.start	= himax_diag_seq_start,
	.next	= himax_diag_seq_next,
	.stop	= himax_diag_seq_stop,
	.show	= himax_diag_seq_read,
};
static int himax_diag_proc_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &himax_diag_seq_ops);
};
static ssize_t himax_diag_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	const uint8_t command_ec_128_raw_flag 		= 0x02;
	const uint8_t command_ec_24_normal_flag 	= 0x00;
	uint8_t command_ec_128_raw_baseline_flag 	= 0x01;
	uint8_t command_ec_128_raw_bank_flag 		= 0x03;
	uint8_t command_F1h[2] = {0xF1, 0x00};
	char messages[80] = {0};

	if (len >= 80)
	{
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}
	if (copy_from_user(messages, buff, len))
	{
		return -EFAULT;
	}

	diag_command = messages[0] - '0';

	I("[Himax]diag_command=0x%x\n",diag_command);
	if (diag_command == 0x01)	{//DC
		command_F1h[1] = command_ec_128_raw_baseline_flag;
		i2c_himax_write(private_ts->client, command_F1h[0] ,&command_F1h[1], 1, DEFAULT_RETRY_CNT);
	} else if (diag_command == 0x02) {//IIR
		command_F1h[1] = command_ec_128_raw_flag;
		i2c_himax_write(private_ts->client, command_F1h[0] ,&command_F1h[1], 1, DEFAULT_RETRY_CNT);
	} else if (diag_command == 0x03) {	//BANK
		command_F1h[1] = command_ec_128_raw_bank_flag;	//0x03
		i2c_himax_write(private_ts->client, command_F1h[0] ,&command_F1h[1], 1, DEFAULT_RETRY_CNT);
	} else if (diag_command == 0x04 ) { // 2T3R IIR
		command_F1h[1] = 0x04; //2T3R IIR
		i2c_himax_write(private_ts->client, command_F1h[0] ,&command_F1h[1], 1, DEFAULT_RETRY_CNT);
	} else if (diag_command == 0x00) {//Disable
		command_F1h[1] = command_ec_24_normal_flag;
		i2c_himax_write(private_ts->client, command_F1h[0] ,&command_F1h[1], 1, DEFAULT_RETRY_CNT);
		touch_monitor_stop_flag = touch_monitor_stop_limit;
	}

	//coordinate dump start
	else if (diag_command == 0x08)	{
		coordinate_fn = filp_open(DIAG_COORDINATE_FILE,O_CREAT | O_WRONLY | O_APPEND | O_TRUNC,0666);
		if (IS_ERR(coordinate_fn))
		{
			E("%s: coordinate_dump_file_create error\n", __func__);
			coordinate_dump_enable = 0;
			filp_close(coordinate_fn,NULL);
		}
		coordinate_dump_enable = 1;
	}
	else if (diag_command == 0x09){
		coordinate_dump_enable = 0;

		if (!IS_ERR(coordinate_fn))
		{
			filp_close(coordinate_fn,NULL);
		}
	}
	//coordinate dump end
	else{
			E("[Himax]Diag command error!diag_command=0x%x\n",diag_command);
	}
	return len;
}

static struct file_operations himax_proc_diag_ops =
{
	.owner = THIS_MODULE,
	.open = himax_diag_proc_open,
	.read = seq_read,
	.write = himax_diag_write,
};
#endif

#ifdef HX_TP_PROC_REGISTER
static ssize_t himax_register_read(struct file *file, char *buf,
	size_t len, loff_t *pos)
{
	int ret = 0;
	int base = 0;
	uint16_t loop_i,loop_j;
	uint8_t inData[128];
	uint8_t outData[5];

	memset(outData, 0x00, sizeof(outData));
	memset(inData, 0x00, sizeof(inData));

	I("Himax multi_register_command = %d \n",multi_register_command);
	if(!HX_PROC_SEND_FLAG)
		{
			if (multi_register_command == 1) {
				base = 0;

				for(loop_i = 0; loop_i < 6; loop_i++) {
					if (multi_register[loop_i] != 0x00) {
						if (multi_cfg_bank[loop_i] == 1) {//config bank register
							outData[0] = 0x14;
							i2c_himax_write(private_ts->client, 0x8C ,&outData[0], 1, DEFAULT_RETRY_CNT);
							msleep(10);

							outData[0] = 0x00;
							outData[1] = multi_register[loop_i];
							i2c_himax_write(private_ts->client, 0x8B ,&outData[0], 2, DEFAULT_RETRY_CNT);
							msleep(10);

							i2c_himax_read(private_ts->client, 0x5A, inData, 128, DEFAULT_RETRY_CNT);

							outData[0] = 0x00;
							i2c_himax_write(private_ts->client, 0x8C ,&outData[0], 1, DEFAULT_RETRY_CNT);
							
							for(loop_j=0; loop_j<128; loop_j++)
								multi_value[base++] = inData[loop_j];
						} else {//normal register
							i2c_himax_read(private_ts->client, multi_register[loop_i], inData, 128, DEFAULT_RETRY_CNT);

							for(loop_j=0; loop_j<128; loop_j++)
								multi_value[base++] = inData[loop_j];
						}
					}
				}

				base = 0;
				for(loop_i = 0; loop_i < 6; loop_i++) {
					if (multi_register[loop_i] != 0x00) {
						if (multi_cfg_bank[loop_i] == 1)
							ret += sprintf(buf + ret, "Register: FE(%x)\n", multi_register[loop_i]);
						else
							ret += sprintf(buf + ret, "Register: %x\n", multi_register[loop_i]);

						for (loop_j = 0; loop_j < 128; loop_j++) {
							ret += sprintf(buf + ret, "0x%2.2X ", multi_value[base++]);
							if ((loop_j % 16) == 15)
								ret += sprintf(buf + ret, "\n");
						}
					}
				}
				return ret;
			}

			if (config_bank_reg) {
				I("%s: register_command = FE(%x)\n", __func__, register_command);

				//Config bank register read flow.
			
				outData[0] = 0x14;
				i2c_himax_write(private_ts->client, 0x8C,&outData[0], 1, DEFAULT_RETRY_CNT);
		    	
				msleep(10);
		    	
				outData[0] = 0x00;
				outData[1] = register_command;
				i2c_himax_write(private_ts->client, 0x8B,&outData[0], 2, DEFAULT_RETRY_CNT);
		    	msleep(10);
		    	
				i2c_himax_read(private_ts->client, 0x5A, inData, 128, DEFAULT_RETRY_CNT);
		    	msleep(10);
		    	
				outData[0] = 0x00;
				i2c_himax_write(private_ts->client, 0x8C,&outData[0], 1, DEFAULT_RETRY_CNT);
			} else {
				if (i2c_himax_read(private_ts->client, register_command, inData, 128, DEFAULT_RETRY_CNT) < 0)
					return ret;
			}

			if (config_bank_reg)
				ret += sprintf(buf, "command: FE(%x)\n", register_command);
			else
				ret += sprintf(buf, "command: %x\n", register_command);

			for (loop_i = 0; loop_i < 128; loop_i++) {
				ret += sprintf(buf + ret, "0x%2.2X ", inData[loop_i]);
				if ((loop_i % 16) == 15)
					ret += sprintf(buf + ret, "\n");
			}
			ret += sprintf(buf + ret, "\n");
			HX_PROC_SEND_FLAG=1;
		}
	else
		HX_PROC_SEND_FLAG=0;
	
	return ret;
}

static ssize_t himax_register_write(struct file *file, const char *buff,
	size_t len, loff_t *pos)
{
	char buf_tmp[6], length = 0;
	unsigned long result    = 0;
	uint8_t loop_i          = 0;
	uint16_t base           = 5;
	uint8_t write_da[128];
	uint8_t outData[5];
	char buf[80] = {0};

	if (len >= 80)
	{
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}
	if (copy_from_user(buf, buff, len))
	{
		return -EFAULT;
	}

	memset(buf_tmp, 0x0, sizeof(buf_tmp));
	memset(write_da, 0x0, sizeof(write_da));
	memset(outData, 0x0, sizeof(outData));

	I("himax %s \n",buf);

	if (buf[0] == 'm' && buf[1] == 'r' && buf[2] == ':') {
		memset(multi_register, 0x00, sizeof(multi_register));
		memset(multi_cfg_bank, 0x00, sizeof(multi_cfg_bank));
		memset(multi_value, 0x00, sizeof(multi_value));

		I("himax multi register enter\n");

		multi_register_command = 1;

		base 	= 2;
		loop_i 	= 0;

		while(true) {
			if (buf[base] == '\n')
				break;

			if (loop_i >= 6 )
				break;

			if (buf[base] == ':' && buf[base+1] == 'x' && buf[base+2] == 'F' &&
					buf[base+3] == 'E' && buf[base+4] != ':') {
				memcpy(buf_tmp, buf + base + 4, 2);
				if (!kstrtoul(buf_tmp, 16, &result)) {
					multi_register[loop_i] = result;
					multi_cfg_bank[loop_i++] = 1;
				}
				base += 6;
			} else {
				memcpy(buf_tmp, buf + base + 2, 2);
				if (!kstrtoul(buf_tmp, 16, &result)) {
					multi_register[loop_i] = result;
					multi_cfg_bank[loop_i++] = 0;
				}
				base += 4;
			}
		}

		I("========================== \n");
		for(loop_i = 0; loop_i < 6; loop_i++)
			I("%d,%d:",multi_register[loop_i],multi_cfg_bank[loop_i]);
		I("\n");
	} else if ((buf[0] == 'r' || buf[0] == 'w') && buf[1] == ':') {
		multi_register_command = 0;

		if (buf[2] == 'x') {
			if (buf[3] == 'F' && buf[4] == 'E') {//Config bank register
				config_bank_reg = true;

				memcpy(buf_tmp, buf + 5, 2);
				if (!kstrtoul(buf_tmp, 16, &result))
					register_command = result;
				base = 7;

				I("CMD: FE(%x)\n", register_command);
			} else {
				config_bank_reg = false;

				memcpy(buf_tmp, buf + 3, 2);
				if (!kstrtoul(buf_tmp, 16, &result))
					register_command = result;
				base = 5;
				I("CMD: %x\n", register_command);
			}

			for (loop_i = 0; loop_i < 128; loop_i++) {
				if (buf[base] == '\n') {
					if (buf[0] == 'w') {
						if (config_bank_reg) {
							outData[0] = 0x14;
							i2c_himax_write(private_ts->client, 0x8C, &outData[0], 1, DEFAULT_RETRY_CNT);
              				msleep(10);
              	
							outData[0] = 0x00;
							outData[1] = register_command;
							i2c_himax_write(private_ts->client, 0x8B, &outData[0], 2, DEFAULT_RETRY_CNT);
              	
							msleep(10);
							i2c_himax_write(private_ts->client, 0x40, &write_da[0], length, DEFAULT_RETRY_CNT);
              	
							msleep(10);
              				outData[0] = 0x00;
							i2c_himax_write(private_ts->client, 0x8C, &outData[0], 1, DEFAULT_RETRY_CNT);
							
							I("CMD: FE(%x), %x, %d\n", register_command,write_da[0], length);
						} else {
							i2c_himax_write(private_ts->client, register_command, &write_da[0], length, DEFAULT_RETRY_CNT);
							I("CMD: %x, %x, %d\n", register_command,write_da[0], length);
						}
					}
					I("\n");
					return len;
				}
				if (buf[base + 1] == 'x') {
					buf_tmp[4] = '\n';
					buf_tmp[5] = '\0';
					memcpy(buf_tmp, buf + base + 2, 2);
					if (!kstrtoul(buf_tmp, 16, &result)) {
						write_da[loop_i] = result;
					}
					length++;
				}
				base += 4;
			}
		}
	}
	return len;
}

static struct file_operations himax_proc_register_ops =
{
	.owner = THIS_MODULE,
	.read = himax_register_read,
	.write = himax_register_write,
};
#endif

#ifdef HX_TP_PROC_DEBUG
static ssize_t himax_debug_read(struct file *file, char *buf,
	size_t len, loff_t *pos)
{
	size_t ret = 0;
	
	if(!HX_PROC_SEND_FLAG)
	{
		if (debug_level_cmd == 't')
		{
			if (fw_update_complete)
			{
				ret += sprintf(buf, "FW Update Complete ");
			}
			else
			{
				ret += sprintf(buf, "FW Update Fail ");
			}
		}
		else if (debug_level_cmd == 'h')
		{
			if (handshaking_result == 0)
			{
				ret += sprintf(buf, "Handshaking Result = %d (MCU Running)\n",handshaking_result);
			}
			else if (handshaking_result == 1)
			{
				ret += sprintf(buf, "Handshaking Result = %d (MCU Stop)\n",handshaking_result);
			}
			else if (handshaking_result == 2)
			{
				ret += sprintf(buf, "Handshaking Result = %d (I2C Error)\n",handshaking_result);
			}
			else
			{
				ret += sprintf(buf, "Handshaking Result = error \n");
			}
		}
		else if (debug_level_cmd == 'v')
		{
			ret += sprintf(buf + ret, "FW_VER = ");
	           ret += sprintf(buf + ret, "0x%2.2X, %2.2X \n",private_ts->vendor_fw_ver_H,private_ts->vendor_fw_ver_L);

			ret += sprintf(buf + ret, "CONFIG_VER = ");
	           ret += sprintf(buf + ret, "0x%2.2X \n",private_ts->vendor_config_ver);
			ret += sprintf(buf + ret, "\n");
		}
		else if (debug_level_cmd == 'd')
		{
			ret += sprintf(buf + ret, "Himax Touch IC Information :\n");
			if (IC_TYPE == HX_85XX_D_SERIES_PWON)
			{
				ret += sprintf(buf + ret, "IC Type : D\n");
			}
			else if (IC_TYPE == HX_85XX_E_SERIES_PWON)
			{
				ret += sprintf(buf + ret, "IC Type : E\n");
			}
			else if (IC_TYPE == HX_85XX_ES_SERIES_PWON)
			{
				ret += sprintf(buf + ret, "IC Type : ES\n");
			}
			else
			{
				ret += sprintf(buf + ret, "IC Type error.\n");
			}

			if (IC_CHECKSUM == HX_TP_BIN_CHECKSUM_SW)
			{
				ret += sprintf(buf + ret, "IC Checksum : SW\n");
			}
			else if (IC_CHECKSUM == HX_TP_BIN_CHECKSUM_HW)
			{
				ret += sprintf(buf + ret, "IC Checksum : HW\n");
			}
			else if (IC_CHECKSUM == HX_TP_BIN_CHECKSUM_CRC)
			{
				ret += sprintf(buf + ret, "IC Checksum : CRC\n");
			}
			else
			{
				ret += sprintf(buf + ret, "IC Checksum error.\n");
			}

			if (HX_INT_IS_EDGE)
			{
				ret += sprintf(buf + ret, "Interrupt : EDGE TIRGGER\n");
			}
			else
			{
				ret += sprintf(buf + ret, "Interrupt : LEVEL TRIGGER\n");
			}

			ret += sprintf(buf + ret, "RX Num : %d\n",HX_RX_NUM);
			ret += sprintf(buf + ret, "TX Num : %d\n",HX_TX_NUM);
			ret += sprintf(buf + ret, "BT Num : %d\n",HX_BT_NUM);
			ret += sprintf(buf + ret, "X Resolution : %d\n",HX_X_RES);
			ret += sprintf(buf + ret, "Y Resolution : %d\n",HX_Y_RES);
			ret += sprintf(buf + ret, "Max Point : %d\n",HX_MAX_PT);
#ifdef HX_TP_PROC_2T2R
			if(Is_2T2R)
			{
			ret += sprintf(buf + ret, "2T2R panel\n");
			ret += sprintf(buf + ret, "RX Num_2 : %d\n",HX_RX_NUM_2);
			ret += sprintf(buf + ret, "TX Num_2 : %d\n",HX_TX_NUM_2);
			}
#endif	
		}

		else if (debug_level_cmd == 'i')
		{
			ret += sprintf(buf + ret, "Himax Touch Driver Version:\n");
			ret += sprintf(buf + ret, "%s \n", HIMAX_DRIVER_VER);
		}
		HX_PROC_SEND_FLAG=1;
	}
	else
		HX_PROC_SEND_FLAG=0;
	return ret;
}

static ssize_t himax_debug_write(struct file *file, const char *buff,
	size_t len, loff_t *pos)
{
	struct file* hx_filp = NULL;
	mm_segment_t oldfs;
	int result = 0;
	char fileName[128];
	char buf[80] = {0};
	char data[12] = {0};//20160302 fw_version update

	if (len >= 80)
	{
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}
	if (copy_from_user(buf, buff, len))
	{
		return -EFAULT;
	}

	if ( buf[0] == 'h') //handshaking
	{
		debug_level_cmd = buf[0];

		himax_int_enable(private_ts->client->irq,0,true);

		handshaking_result = himax_hand_shaking(); //0:Running, 1:Stop, 2:I2C Fail

		himax_int_enable(private_ts->client->irq,1,true);

		return len;
	}

	else if ( buf[0] == 'v') //firmware version
	{
		debug_level_cmd = buf[0];
		himax_read_FW_ver(true);
		return len;
	}

	else if ( buf[0] == 'd') //test
	{
		debug_level_cmd = buf[0];
		return len;
	}

	else if ( buf[0] == 'i') //driver version
	{
		debug_level_cmd = buf[0];
		return len;
	}

	else if (buf[0] == 't')
	{

		himax_int_enable(private_ts->client->irq,0,true);
		wake_lock(&private_ts->ts_flash_wake_lock);
#ifdef HX_CHIP_STATUS_MONITOR
		HX_CHIP_POLLING_COUNT = 0;
		cancel_delayed_work_sync(&private_ts->himax_chip_monitor);
#endif

		debug_level_cmd 		= buf[0];
		fw_update_complete		= false;

		memset(fileName, 0, 128);
		// parse the file name
		snprintf(fileName, len-2, "%s", &buf[2]);
		I("%s: upgrade from file(%s) start!\n", __func__, fileName);
		// open file
		hx_filp = filp_open(fileName, O_RDONLY, 0);
		if (IS_ERR(hx_filp))
		{
			E("%s: open firmware file failed\n", __func__);
			goto firmware_upgrade_done;
			//return len;
		}
		oldfs = get_fs();
		set_fs(get_ds());

		// read the latest firmware binary file
		result=hx_filp->f_op->read(hx_filp,upgrade_fw,sizeof(upgrade_fw), &hx_filp->f_pos);
		if (result < 0)
		{
			E("%s: read firmware file failed\n", __func__);
			goto firmware_upgrade_done;
			//return len;
		}

		set_fs(oldfs);
		filp_close(hx_filp, NULL);

		I("%s: upgrade start,len %d: %02X, %02X, %02X, %02X\n", __func__, result, upgrade_fw[0], upgrade_fw[1], upgrade_fw[2], upgrade_fw[3]);

		if (result > 0)
		{
		// start to upgrade
#ifdef HX_RST_PIN_FUNC
			himax_HW_reset(false,true);
#endif
			if (fts_ctpm_fw_upgrade_with_fs(upgrade_fw, result, true) == 0)
			{
				E("%s: TP upgrade error, line: %d\n", __func__, __LINE__);
				fw_update_complete = false;
			}
			else
			{
				I("%s: TP upgrade OK, line: %d\n", __func__, __LINE__);
				fw_update_complete = true;
			}
			goto firmware_upgrade_done;
			//return len;
		}
	}

	firmware_upgrade_done:

#ifdef HX_RST_PIN_FUNC
	himax_HW_reset(true,true);
#endif
	//=====================20160302 fw_version update
			msleep(120);//20160302 fw_version update
			if (i2c_himax_read(private_ts->client, HX_VER_FW_MAJ, data, 1, 3) < 0) {
				E("%s: i2c access fail!\n", __func__);
				return 0;
			}
			FW_H= data[0];
			I("%s: TP FW_H=%X \n", __func__,FW_H);

			if (i2c_himax_read(private_ts->client, HX_VER_FW_MIN, data, 1, 3) < 0) {
				E("%s: i2c access fail!\n", __func__);
				return 0;
			}
			FW_L= data[0];
			I("%s: TP FW_L=%X \n", __func__,FW_L);

			if (i2c_himax_read(private_ts->client, HX_VER_FW_CFG, data, 1, 3) < 0) {
				E("%s: i2c access fail!\n", __func__);
				return 0;
			}
			FW_CON= data[0];
			I("%s: TP FW_CON=%X \n", __func__,FW_CON);
	//=====================20160302
	wake_unlock(&private_ts->ts_flash_wake_lock);
	himax_int_enable(private_ts->client->irq,1,true);
#ifdef HX_CHIP_STATUS_MONITOR
	HX_CHIP_POLLING_COUNT = 0;
	queue_delayed_work(private_ts->himax_chip_monitor_wq, &private_ts->himax_chip_monitor, HX_POLLING_TIMES*HZ);
#endif
	//todo himax_chip->tp_firmware_upgrade_proceed = 0;
	//todo himax_chip->suspend_state = 0;
	//todo enable_irq(himax_chip->irq);
	return len;
}

static struct file_operations himax_proc_debug_ops =
{
	.owner = THIS_MODULE,
	.read = himax_debug_read,
	.write = himax_debug_write,
};
#endif

#ifdef HX_TP_FWVERSION
//========================================== 20160119 add version node start
static int himax_fw_version_read(struct seq_file *m, void *v)
{
	/* update FW info */
	himax_read_FW_ver(false);

	seq_printf(m, "FW ver = %2.2X%2.2X%2.2X\n",
			private_ts->vendor_fw_ver_H,
			private_ts->vendor_fw_ver_L,
			private_ts->vendor_config_ver);

	D("%s: FW major ver = 0x%2.2X\n", __func__,
			private_ts->vendor_fw_ver_H);
	D("%s: FW minor ver = 0x%2.2X\n", __func__,
			private_ts->vendor_fw_ver_L);
	D("%s: FW config ver = 0x%2.2X\n", __func__,
			private_ts->vendor_config_ver);


	return 0;
}

static int himax_fw_version_read_open(struct inode *inode, struct file *file)
{
	return single_open(file, himax_fw_version_read, inode->i_private);
}

static const struct file_operations himax_proc_fwversion_ops = {
	.owner = THIS_MODULE,
	.open = himax_fw_version_read_open,
	.read = seq_read,
	.release = single_release,
};

static ssize_t himax_fwversion_for_i_read(struct file *file, char *buf,
	size_t len, loff_t *pos)
{
	size_t ret = 0;

	if (!HX_PROC_SEND_FLAG) {
		ret += sprintf(buf + ret, "i FW ver = %2.2X%2.2X%2.2X\n",
				FW_H, FW_L, FW_CON);

		D("%s: i FW major ver = 0x%2.2X\n", __func__, FW_H);
		D("%s: i FW minor ver = 0x%2.2X\n", __func__, FW_L);
		D("%s: i FW config ver = 0x%2.2X \n", __func__, FW_CON);

		HX_PROC_SEND_FLAG = 1;
	} else {
		HX_PROC_SEND_FLAG = 0;
	}

	return ret;
}

static struct file_operations himax_proc_fwversion_for_i_ops =
{
	.owner = THIS_MODULE,
	.read = himax_fwversion_for_i_read,
};
//=========================================== 20160119 add version node end

//=========================================== 20160119 add get fw state start
static ssize_t himax_fwstate_read(struct file *file, char *buf,
	size_t len, loff_t *pos)
{
	//fw_status:
//      0: UNKNOWN
//      1: Do update
//      2: Updating
//      3: Done
//      4: Fail
	size_t ret = 0;
        I("%s: get_fw_state() fw_status : %d\n", __func__, fw_status);

	if(!HX_PROC_SEND_FLAG)
	{
		if(fw_status ==1)
			ret += sprintf(buf + ret, "DO UPDATE\n");
		else if(fw_status ==2)
			ret += sprintf(buf + ret, "UPDATING\n");
		else if(fw_status ==3)
			ret += sprintf(buf + ret, "DONE\n");
		else if(fw_status ==4)
			ret += sprintf(buf + ret, "FAIL\n");
		else
			ret += sprintf(buf + ret, "UNKNOWN\n");

		HX_PROC_SEND_FLAG=1;
	}
	else
		HX_PROC_SEND_FLAG=0;

	return ret;
}

static struct file_operations himax_proc_fwstate_ops =
{
	.owner = THIS_MODULE,
	.read = himax_fwstate_read,
};

static ssize_t himax_intstate_read(struct file *file, char *buf,
	size_t len, loff_t *pos)
{
	size_t ret = 0;

        if(!HX_PROC_SEND_FLAG)
        {
                if(int_status ==1)
                        ret += sprintf(buf + ret, "Int\n");
                else if(int_status ==2)
                        ret += sprintf(buf + ret, "Event\n");
                else if(int_status ==3)
                        ret += sprintf(buf + ret, "Ts\n");
                else if(int_status ==4)
                        ret += sprintf(buf + ret, "Report\n");
		else if(int_status ==5)
			ret += sprintf(buf + ret, "Stop\n");
                else
                        ret += sprintf(buf + ret, "default\n");

                HX_PROC_SEND_FLAG=1;
        }
        else
                HX_PROC_SEND_FLAG=0;

        return ret;
}

static struct file_operations himax_proc_intstate_ops =
{
        .owner = THIS_MODULE,
        .read = himax_intstate_read,
};

//========================================== 20160119 add get fw state end
#endif

/* TPID_CHECK node start */
#ifdef HX_TPID_CHECK
static ssize_t himax_tpid_check_read(struct file *file, char *buf,
	size_t len, loff_t *pos)
{
	size_t ret = 0;
	u8 tp_id = -1;

	if(!HX_PROC_SEND_FLAG)
	{
		tp_id = __gpio_get_value(TPID_GPIO);

		switch(tp_id) {
			case 0:
			case 1:
				ret += sprintf(buf + ret, "TP ID = %d\n", tp_id);
			break;
			default:
				ret += sprintf(buf + ret, "TP ID read fail, ret : %d\n", tp_id);
			break;
		}

		HX_PROC_SEND_FLAG=1;
	}
	else
		HX_PROC_SEND_FLAG=0;

	return ret;
}

static struct file_operations himax_proc_tpid_check_ops =
{
	.owner = THIS_MODULE,
	.read = himax_tpid_check_read,
};

#endif
/* TPID_CHECK node end */

/* fw_check node start */
#ifdef HX_TP_FW_CHECK
static ssize_t himax_fw_check_read(struct file *file, char *buf,
	size_t len, loff_t *pos)
{
	size_t ret = 0;
	u8 result;

	if (!HX_PROC_SEND_FLAG) {
		/* update FW info. */
		himax_read_FW_ver(false);

		if (DEFAULT_FW_VER_H == private_ts->vendor_fw_ver_H) {
			result = FW_CHECK_PASS;
		} else {
			result = FW_CHECK_FAIL;
		}

		switch (result) {
		case FW_CHECK_PASS:
			ret += sprintf(buf + ret,
				"Pass, (fw_ver : 0x%x)\n",
				private_ts->vendor_fw_ver_H);
		break;
		case FW_CHECK_FAIL:
			ret += sprintf(buf + ret,
				"Fail, (default fw_ver = 0x%x, fw_ver : 0x%x)\n",
				DEFAULT_FW_VER_H, private_ts->vendor_fw_ver_H);
		break;
		default:
		break;
		}

		HX_PROC_SEND_FLAG=1;
	} else {
		HX_PROC_SEND_FLAG=0;
	}

	return ret;
}

static struct file_operations himax_proc_fw_check_ops =
{
	.owner = THIS_MODULE,
	.read = himax_fw_check_read,
};
#endif
/* fw_check node end */

/* sensor_id node start */
#ifdef HX_TP_SENSOR_ID
static ssize_t himax_sensor_id_read(struct file *file, char *buf,
	size_t len, loff_t *pos)
{
	size_t ret = 0;
	u8 id_0 = 0, id_1 = 0;

	if (!HX_PROC_SEND_FLAG) {
		private_ts->vendor_sensor_id = himax_read_Sensor_ID(private_ts->client);

		ret += sprintf(buf + ret,
				"sensor ID : 0x%x\n",
				private_ts->vendor_sensor_id);

		id_0 = private_ts->vendor_sensor_id & 0x0f;
		id_1 = (private_ts->vendor_sensor_id & 0xf0) >> 4;

		switch(id_0) {
		case SENSOR_ID_FLOATING:
			ret += sprintf(buf + ret, "ID 0, Floating\n");
		break;
		case SENSOR_ID_GROUND:
			ret += sprintf(buf + ret, "ID 0, Ground\n");
		break;
		case SENSOR_ID_VCC:
			ret += sprintf(buf + ret, "ID 0, Vcc\n");
		break;
		default:
		break;
		}

		switch(id_1) {
		case SENSOR_ID_FLOATING:
			ret += sprintf(buf + ret, "ID 1, Floating\n");
		break;
		case SENSOR_ID_GROUND:
			ret += sprintf(buf + ret, "ID 1, Ground\n");
		break;
		case SENSOR_ID_VCC:
			ret += sprintf(buf + ret, "ID 1, Vcc\n");
		break;
		default:
		break;
		}

		HX_PROC_SEND_FLAG=1;
	} else {
		HX_PROC_SEND_FLAG=0;
	}

	return ret;
}

static struct file_operations himax_proc_sensor_id_ops =
{
	.owner = THIS_MODULE,
	.read = himax_sensor_id_read,
};
#endif
/* sensor id node end */

/* glove mode node start */
#ifdef HX_TP_GLOVE_MODE
static ssize_t himax_glove_mode_read(struct file *file, char *buf,
	size_t len, loff_t *pos)
{
	size_t ret = 0;
	uint8_t data;

	if (!HX_PROC_SEND_FLAG) {

		if (i2c_himax_read(private_ts->client, HX_GLOVE_REG, &data, 1, 3) < 0) {
			E("%s: i2c access fail!\n", __func__);
		}
		ret += sprintf(buf + ret,
				"glove register 0x%02x : 0x%02x , glove_mode : 0x%02x\n", HX_GLOVE_REG, data, glove_state);

		HX_PROC_SEND_FLAG=1;
	} else {
		HX_PROC_SEND_FLAG=0;
	}

	return ret;
}

static ssize_t himax_glove_mode_write(struct file *file, const char *buff,
	size_t len, loff_t *pos)
{
	char buf_tmp[12];
	uint8_t cmd[2];

	if (atomic_read(&private_ts->suspend_mode) == 1 && atomic_read(&private_ts->resume_mode) == 0) {
		E("%s: system is suspend mode, cannot set glove reg.\n", __func__);
		return len;
	}

	if (len >= 3) {
		E("%s: no command exceeds 3 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf_tmp, buff, len)) {
		E("%s: copy_from_user fail\n", __func__);
		return -EFAULT;
	}

	cmd[0] = HX_GLOVE_REG;

	if (buf_tmp[0] == '0') {
		cmd[1] = DISABLE_GLOVE_MODE;
		glove_state = DISABLE_GLOVE_MODE;
	} else if (buf_tmp[0] == '1') {
		cmd[1] = ENABLE_GLOVE_MODE;
		glove_state = ENABLE_GLOVE_MODE;
	} else {
		I("%s: un-define cmd, %d\n", __func__, buf_tmp[0]);
		return len;
	}

	I("%s: buf_tmp : %d , glove state : 0x%02x\n", __func__, buf_tmp[0], glove_state);
	if (i2c_himax_master_write(private_ts->client, &cmd[0], 2, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
	}

	return len;
}

static struct file_operations himax_proc_glove_mode_ops =
{
	.owner = THIS_MODULE,
	.read = himax_glove_mode_read,
	.write = himax_glove_mode_write,
};

static ssize_t himax_glove_on_read(struct file *file, char *buf,
	size_t len, loff_t *pos)
{
	size_t ret = 0;
	uint8_t cmd[2];

	if (!HX_PROC_SEND_FLAG) {
		cmd[0] = HX_GLOVE_REG;
		cmd[1] = ENABLE_GLOVE_MODE;
		glove_state = ENABLE_GLOVE_MODE;

		if (i2c_himax_master_write(private_ts->client, &cmd[0], 2, 3) < 0) {
			E("%s: i2c access fail!\n", __func__);
		}

		I("%s: Gloves status = On\n", __func__);
		ret += sprintf(buf + ret,
				"Gloves status = On\n");

		HX_PROC_SEND_FLAG=1;
	} else {
		HX_PROC_SEND_FLAG=0;
	}

	return ret;
}

static struct file_operations himax_proc_glove_on_ops =
{
	.owner = THIS_MODULE,
	.read = himax_glove_on_read,
};

static ssize_t himax_glove_off_read(struct file *file, char *buf,
	size_t len, loff_t *pos)
{
	size_t ret = 0;
	uint8_t cmd[2];

	if (!HX_PROC_SEND_FLAG) {
		cmd[0] = HX_GLOVE_REG;
		cmd[1] = DISABLE_GLOVE_MODE;
		glove_state = DISABLE_GLOVE_MODE;

		if (i2c_himax_master_write(private_ts->client, &cmd[0], 2, 3) < 0) {
			E("%s: i2c access fail!\n", __func__);
		}

		I("%s: Gloves status = Off\n", __func__);
		ret += sprintf(buf + ret,
				"Gloves status = Off\n");

		HX_PROC_SEND_FLAG=1;
	} else {
		HX_PROC_SEND_FLAG=0;
	}

	return ret;
}

static struct file_operations himax_proc_glove_off_ops =
{
	.owner = THIS_MODULE,
	.read = himax_glove_off_read,
};

#endif
/* glove mode node end */

/* calling status node start */
#ifdef PHONE_CALLING_STATUS
static int himax_calling_status_read(struct seq_file *m, void *v)
{
	if (phone_calling_status) {
		D("%s: phone_calling_status is call ON\n", __func__);
		seq_puts(m, "phone_calling_status is call ON\n");
	} else {
		D("%s: phone_calling_status is call OFF\n", __func__);
		seq_puts(m, "phone_calling_status is call OFF\n");
	}

	return 0;
}

static ssize_t himax_calling_status_write(struct file *file, const char *buff,
	size_t len, loff_t *pos)
{
	char buf_tmp[12];

	if (len >= 3) {
		E("%s: no command exceeds 3 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf_tmp, buff, len)) {
		E("%s: copy_from_user fail\n", __func__);
		return -EFAULT;
	}

	if (buf_tmp[0] == '0') {
		glove_state = PHONE_CALL_OFF;
	} else if (buf_tmp[0] == '1') {
		D("%s: phone_calling_status is call ON\n", __func__);
		glove_state = PHONE_CALL_ON;
	} else {
		I("%s: un-define cmd, %d\n", __func__, buf_tmp[0]);
		return len;
	}

	return len;
}

static int himax_calling_status_read_open(struct inode *inode, struct file *file)
{
	return single_open(file, himax_calling_status_read, inode->i_private);
}

static const struct file_operations himax_proc_calling_status_ops = {
	.owner = THIS_MODULE,
	.open = himax_calling_status_read_open,
	.read = seq_read,
	.write = himax_calling_status_write,
	.release = single_release,
};

static int himax_phone_call_on_read(struct seq_file *m, void *v)
{
	D("%s: phone_calling_status is set call ON\n", __func__);
	phone_calling_status = PHONE_CALL_ON;

	return 0;
}

static int himax_phone_call_on_read_open(struct inode *inode, struct file *file)
{
	return single_open(file, himax_phone_call_on_read, inode->i_private);
}

static const struct file_operations himax_proc_phone_call_on_ops = {
	.owner = THIS_MODULE,
	.open = himax_phone_call_on_read_open,
	.read = seq_read,
	.release = single_release,
};

static int himax_phone_call_off_read(struct seq_file *m, void *v)
{
	D("%s: phone_calling_status is set call OFF\n", __func__);
	phone_calling_status = PHONE_CALL_OFF;

	return 0;
}

static int himax_phone_call_off_read_open(struct inode *inode, struct file *file)
{
	return single_open(file, himax_phone_call_off_read, inode->i_private);
}

static const struct file_operations himax_proc_phone_call_off_ops = {
	.owner = THIS_MODULE,
	.open = himax_phone_call_off_read_open,
	.read = seq_read,
	.release = single_release,
};
#endif
/* calling status node end */

/* ESD Recovery Count node start */
#ifdef HX_ESD_WORKAROUND
static ssize_t himax_ESD_recovery_count_read(struct file *file, char *buf,
	size_t len, loff_t *pos)
{
	size_t ret = 0;

	if (!HX_PROC_SEND_FLAG) {
		I("%s: ESD_recovery_count = %d\n", __func__, ESD_recovery_count);
		ret += sprintf(buf + ret,
				"ESD_recovery_count = %d\n", ESD_recovery_count);

		HX_PROC_SEND_FLAG=1;
	} else {
		HX_PROC_SEND_FLAG=0;
	}

	return ret;
}

static struct file_operations himax_proc_ESD_recovery_count_ops =
{
	.owner = THIS_MODULE,
	.read = himax_ESD_recovery_count_read,
};

/* ESD off node start */
static int himax_ESD_off_read(struct seq_file *m, void *v)
{
	/* update FW info */
	cancel_delayed_work_sync(&private_ts->himax_chip_monitor);
	seq_printf(m, "cancel ESD work\n");

	return 0;
}

static int himax_ESD_off_read_open(struct inode *inode, struct file *file)
{
	return single_open(file, himax_ESD_off_read, inode->i_private);
}

static const struct file_operations himax_proc_ESD_off_ops = {
	.owner = THIS_MODULE,
	.open = himax_ESD_off_read_open,
	.read = seq_read,
	.release = single_release,
};
/* ESD off node END */

#endif
/* ESD Recovery Count node end */

#ifdef HX_TP_PROC_FLASH_DUMP

static uint8_t getFlashCommand(void)
{
	return flash_command;
}

static uint8_t getFlashDumpProgress(void)
{
	return flash_progress;
}

static uint8_t getFlashDumpComplete(void)
{
	return flash_dump_complete;
}

static uint8_t getFlashDumpFail(void)
{
	return flash_dump_fail;
}

static uint8_t getSysOperation(void)
{
	return sys_operation;
}

static uint8_t getFlashReadStep(void)
{
	return flash_read_step;
}

static uint8_t getFlashDumpSector(void)
{
	return flash_dump_sector;
}

static uint8_t getFlashDumpPage(void)
{
	return flash_dump_page;
}

static bool getFlashDumpGoing(void)
{
	return flash_dump_going;
}

static void setFlashBuffer(void)
{
	//int i=0;
	flash_buffer = kzalloc(FLASH_SIZE * sizeof(uint8_t), GFP_KERNEL);
	memset(flash_buffer,0x00,FLASH_SIZE);
	//for(i=0; i<FLASH_SIZE; i++)
	//{
	//	flash_buffer[i] = 0x00;
	//}
}

static void setSysOperation(uint8_t operation)
{
	sys_operation = operation;
}

static void setFlashDumpProgress(uint8_t progress)
{
	flash_progress = progress;
	//I("setFlashDumpProgress : progress = %d ,flash_progress = %d \n",progress,flash_progress);
}

static void setFlashDumpComplete(uint8_t status)
{
	flash_dump_complete = status;
}

static void setFlashDumpFail(uint8_t fail)
{
	flash_dump_fail = fail;
}

static void setFlashCommand(uint8_t command)
{
	flash_command = command;
}

static void setFlashReadStep(uint8_t step)
{
	flash_read_step = step;
}

static void setFlashDumpSector(uint8_t sector)
{
	flash_dump_sector = sector;
}

static void setFlashDumpPage(uint8_t page)
{
	flash_dump_page = page;
}

static void setFlashDumpGoing(bool going)
{
	flash_dump_going = going;
}

static void himax_ts_flash_work_func(struct work_struct *work)
{
	struct himax_ts_data *ts = container_of(work, struct himax_ts_data, flash_work);

	uint8_t page_tmp[128];
	uint8_t x59_tmp[4] = {0,0,0,0};
	int i=0, j=0, k=0, l=0,/* j_limit = 0,*/ buffer_ptr = 0/*, flash_end_count = 0*/;
	uint8_t local_flash_command = 0;
	uint8_t sector = 0;
	uint8_t page = 0;

	/*uint8_t xAA_command[2] = {0xAA,0x00};*/
	uint8_t x81_command[2] = {HX_CMD_TSSLPOUT,0x00};
	uint8_t x82_command[2] = {HX_CMD_TSSOFF,0x00};
	uint8_t x35_command[2] = {HX_CMD_SETMICROOFF,0x00};
	uint8_t x43_command[4] = {HX_CMD_FLASH_ENABLE,0x00,0x00,0x00};
	uint8_t x44_command[4] = {HX_CMD_FLASH_SET_ADDRESS,0x00,0x00,0x00};
	uint8_t x45_command[5] = {HX_CMD_FLASH_WRITE_REGISTER,0x00,0x00,0x00,0x00};
	uint8_t x46_command[2] = {0x46,0x00};
	/*uint8_t x4A_command[2] = {HX_CMD_4A,0x00};*/
	uint8_t x4D_command[2] = {0x4D,0x00};
	/*uint8_t x59_command[2] = {0x59,0x00};*/

	himax_int_enable(private_ts->client->irq,0,true);
#ifdef HX_CHIP_STATUS_MONITOR
	HX_CHIP_POLLING_COUNT = 0;
	cancel_delayed_work_sync(&private_ts->himax_chip_monitor);
#endif

	setFlashDumpGoing(true);

	sector = getFlashDumpSector();
	page = getFlashDumpPage();

	local_flash_command = getFlashCommand();

#ifdef HX_RST_PIN_FUNC
	if(local_flash_command<0x0F)
		himax_HW_reset(false,true);
#endif

	//if( i2c_himax_master_write(ts->client, xAA_command, 1, 3) < 0 )//sleep out
	//{
	//	E("%s i2c write AA fail.\n",__func__);
	//	goto Flash_Dump_i2c_transfer_error;
	//}
	//msleep(120);

	if ( i2c_himax_master_write(ts->client, x81_command, 1, 3) < 0 )//sleep out
	{
		E("%s i2c write 81 fail.\n",__func__);
		goto Flash_Dump_i2c_transfer_error;
	}
	msleep(120);
	if ( i2c_himax_master_write(ts->client, x82_command, 1, 3) < 0 )
	{
		E("%s i2c write 82 fail.\n",__func__);
		goto Flash_Dump_i2c_transfer_error;
	}
	msleep(100);

	I("%s: local_flash_command = %d enter.\n", __func__,local_flash_command);

	if ((local_flash_command == 1 || local_flash_command == 2)|| (local_flash_command==0x0F))
	{
		x43_command[1] = 0x01;
		if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 1, DEFAULT_RETRY_CNT) < 0)
		{
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(100);

		for( i=0 ; i<8 ;i++)
		{
			for(j=0 ; j<32 ; j++)
			{
				//I(" Step 2 i=%d , j=%d %s\n",i,j,__func__);
				//read page start
				for(k=0; k<128; k++)
				{
					page_tmp[k] = 0x00;
				}
				for(k=0; k<32; k++)
				{
					x44_command[1] = k;
					x44_command[2] = j;
					x44_command[3] = i;
					if ( i2c_himax_write(ts->client, x44_command[0],&x44_command[1], 3, DEFAULT_RETRY_CNT) < 0 )
					{
						E("%s i2c write 44 fail.\n",__func__);
						goto Flash_Dump_i2c_transfer_error;
					}
					if ( i2c_himax_write_command(ts->client, x46_command[0], DEFAULT_RETRY_CNT) < 0)
					{
						E("%s i2c write 46 fail.\n",__func__);
						goto Flash_Dump_i2c_transfer_error;
					}
					//msleep(2);
					if ( i2c_himax_read(ts->client, 0x59, x59_tmp, 4, DEFAULT_RETRY_CNT) < 0)
					{
						E("%s i2c write 59 fail.\n",__func__);
						goto Flash_Dump_i2c_transfer_error;
					}
					//msleep(2);
					for(l=0; l<4; l++)
					{
						page_tmp[k*4+l] = x59_tmp[l];
					}
					//msleep(10);
				}
				//read page end

				for(k=0; k<128; k++)
				{
					flash_buffer[buffer_ptr++] = page_tmp[k];

				}
				setFlashDumpProgress(i*32 + j);
			}
		}
	}
	else if (local_flash_command == 3)
	{
		x43_command[1] = 0x01;
		if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 1, DEFAULT_RETRY_CNT) < 0 )
		{
			E("%s i2c write 43 fail.\n",__func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(100);

		for(i=0; i<128; i++)
		{
			page_tmp[i] = 0x00;
		}

		for(i=0; i<32; i++)
		{
			x44_command[1] = i;
			x44_command[2] = page;
			x44_command[3] = sector;

			if ( i2c_himax_write(ts->client, x44_command[0],&x44_command[1], 3, DEFAULT_RETRY_CNT) < 0 )
			{
				E("%s i2c write 44 fail.\n",__func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			if ( i2c_himax_write_command(ts->client, x46_command[0], DEFAULT_RETRY_CNT) < 0 )
			{
				E("%s i2c write 46 fail.\n",__func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			//msleep(2);
			if ( i2c_himax_read(ts->client, 0x59, x59_tmp, 4, DEFAULT_RETRY_CNT) < 0 )
			{
				E("%s i2c write 59 fail.\n",__func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			//msleep(2);
			for(j=0; j<4; j++)
			{
				page_tmp[i*4+j] = x59_tmp[j];
			}
			//msleep(10);
		}
		//read page end
		for(i=0; i<128; i++)
		{
			flash_buffer[buffer_ptr++] = page_tmp[i];
		}
	}
	else if (local_flash_command == 4)
	{
		//page write flow.
		//I("%s: local_flash_command = 4, enter.\n", __func__);

		// unlock flash
		himax_lock_flash(0);

		msleep(50);

		// page erase
		x43_command[1] = 0x01;
		x43_command[2] = 0x00;
		x43_command[3] = 0x02;
		if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 3, DEFAULT_RETRY_CNT) < 0 )
		{
			E("%s i2c write 43 fail.\n",__func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		x44_command[1] = 0x00;
		x44_command[2] = page;
		x44_command[3] = sector;
		if ( i2c_himax_write(ts->client, x44_command[0],&x44_command[1], 3, DEFAULT_RETRY_CNT) < 0 )
		{
			E("%s i2c write 44 fail.\n",__func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);
		if ( i2c_himax_write_command(ts->client, x4D_command[0], DEFAULT_RETRY_CNT) < 0 )
		{
			E("%s i2c write 4D fail.\n",__func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(100);

		// enter manual mode

		x35_command[1] = 0x01;
		if( i2c_himax_write(ts->client, x35_command[0],&x35_command[1], 1, DEFAULT_RETRY_CNT) < 0 )
		{
			E("%s i2c write 35 fail.\n",__func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		
		msleep(100);

		// flash enable
		x43_command[1] = 0x01;
		x43_command[2] = 0x00;
		if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
		{
			E("%s i2c write 43 fail.\n",__func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		// set flash address
		x44_command[1] = 0x00;
		x44_command[2] = page;
		x44_command[3] = sector;
		if ( i2c_himax_write(ts->client, x44_command[0],&x44_command[1], 3, DEFAULT_RETRY_CNT) < 0 )
		{
			E("%s i2c write 44 fail.\n",__func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		// manual mode command : 47 to latch the flash address when page address change.
		x43_command[1] = 0x01;
		x43_command[2] = 0x09;
		if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
		{
			E("%s i2c write 43 fail.\n",__func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		x43_command[1] = 0x01;
		x43_command[2] = 0x0D;
		if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
		{
			E("%s i2c write 43 fail.\n",__func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		x43_command[1] = 0x01;
		x43_command[2] = 0x09;
		if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
		{
			E("%s i2c write 43 fail.\n",__func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		for(i=0; i<32; i++)
		{
			I("himax :i=%d \n",i);
			x44_command[1] = i;
			x44_command[2] = page;
			x44_command[3] = sector;
			if ( i2c_himax_write(ts->client, x44_command[0],&x44_command[1], 3, DEFAULT_RETRY_CNT) < 0 )
			{
				E("%s i2c write 44 fail.\n",__func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			msleep(10);

			x45_command[1] = flash_buffer[i*4 + 0];
			x45_command[2] = flash_buffer[i*4 + 1];
			x45_command[3] = flash_buffer[i*4 + 2];
			x45_command[4] = flash_buffer[i*4 + 3];
			if ( i2c_himax_write(ts->client, x45_command[0],&x45_command[1], 4, DEFAULT_RETRY_CNT) < 0 )
			{
				E("%s i2c write 45 fail.\n",__func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			msleep(10);

			// manual mode command : 48 ,data will be written into flash buffer
			x43_command[1] = 0x01;
			x43_command[2] = 0x0D;
			if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
			{
				E("%s i2c write 43 fail.\n",__func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			msleep(10);

			x43_command[1] = 0x01;
			x43_command[2] = 0x09;
			if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
			{
				E("%s i2c write 43 fail.\n",__func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			msleep(10);
		}

		// manual mode command : 49 ,program data from flash buffer to this page
		x43_command[1] = 0x01;
		x43_command[2] = 0x01;
		if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
		{
			E("%s i2c write 43 fail.\n",__func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		x43_command[1] = 0x01;
		x43_command[2] = 0x05;
		if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
		{
			E("%s i2c write 43 fail.\n",__func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		x43_command[1] = 0x01;
		x43_command[2] = 0x01;
		if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
		{
			E("%s i2c write 43 fail.\n",__func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		x43_command[1] = 0x01;
		x43_command[2] = 0x00;
		if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
		{
			E("%s i2c write 43 fail.\n",__func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		// flash disable
		x43_command[1] = 0x00;
		if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 1, DEFAULT_RETRY_CNT) < 0 )
		{
			E("%s i2c write 43 fail.\n",__func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		// leave manual mode
		x35_command[1] = 0x01;
		if( i2c_himax_write(ts->client, x35_command[0],&x35_command[1], 1, DEFAULT_RETRY_CNT) < 0 )
		{
			E("%s i2c write 35 fail.\n",__func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		
		msleep(10);

		// lock flash
		himax_lock_flash(1);
		msleep(50);

		buffer_ptr = 128;
		I("Himax: Flash page write Complete~~~~~~~~~~~~~~~~~~~~~~~\n");
	}

	I("Complete~~~~~~~~~~~~~~~~~~~~~~~\n");
	if(local_flash_command==0x01)
		{
			I(" buffer_ptr = %d \n",buffer_ptr);
			
			for (i = 0; i < buffer_ptr; i++) 
			{
				I("%2.2X ", flash_buffer[i]);
			
				if ((i % 16) == 15)
				{
					I("\n");
				}
			}
			I("End~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
		}
	i2c_himax_master_write(ts->client, x43_command, 1, 3);
	msleep(50);

	if (local_flash_command == 2)
	{
		struct file *fn;

		fn = filp_open(FLASH_DUMP_FILE,O_CREAT | O_WRONLY ,0);
		if (!IS_ERR(fn))
		{
			fn->f_op->write(fn,flash_buffer,buffer_ptr*sizeof(uint8_t),&fn->f_pos);
			filp_close(fn,NULL);
		}
	}

#ifdef HX_RST_PIN_FUNC
	if(local_flash_command<0x0F)
		himax_HW_reset(true,true);
#endif

	himax_int_enable(private_ts->client->irq,1,true);
#ifdef HX_CHIP_STATUS_MONITOR
	HX_CHIP_POLLING_COUNT = 0;
	queue_delayed_work(private_ts->himax_chip_monitor_wq, &private_ts->himax_chip_monitor, HX_POLLING_TIMES*HZ);
#endif
	setFlashDumpGoing(false);

	setFlashDumpComplete(1);
	setSysOperation(0);
	return;

	Flash_Dump_i2c_transfer_error:

#ifdef HX_RST_PIN_FUNC
	himax_HW_reset(true,true);
#endif

	himax_int_enable(private_ts->client->irq,1,true);
#ifdef HX_CHIP_STATUS_MONITOR
	HX_CHIP_POLLING_COUNT = 0;
	queue_delayed_work(private_ts->himax_chip_monitor_wq, &private_ts->himax_chip_monitor, HX_POLLING_TIMES*HZ);
#endif
	setFlashDumpGoing(false);
	setFlashDumpComplete(0);
	setFlashDumpFail(1);
	setSysOperation(0);
	return;
}

static ssize_t himax_flash_read(struct file *file, char *buf,
	size_t len, loff_t *pos)
{
	int ret = 0;
	int loop_i;
	uint8_t local_flash_read_step=0;
	uint8_t local_flash_complete = 0;
	uint8_t local_flash_progress = 0;
	uint8_t local_flash_command = 0;
	uint8_t local_flash_fail = 0;

	local_flash_complete = getFlashDumpComplete();
	local_flash_progress = getFlashDumpProgress();
	local_flash_command = getFlashCommand();
	local_flash_fail = getFlashDumpFail();

	I("flash_progress = %d \n",local_flash_progress);
	if(!HX_PROC_SEND_FLAG)
	{

		if (local_flash_fail)
		{
			ret += sprintf(buf + ret, "FlashStart:Fail \n");
			ret += sprintf(buf + ret, "FlashEnd");
			ret += sprintf(buf + ret, "\n");
			HX_PROC_SEND_FLAG=1;
			return ret;
		}

		if (!local_flash_complete)
		{
			ret += sprintf(buf + ret, "FlashStart:Ongoing:0x%2.2x \n",flash_progress);
			ret += sprintf(buf + ret, "FlashEnd");
			ret += sprintf(buf + ret, "\n");
			HX_PROC_SEND_FLAG=1;
			return ret;
		}

		if (local_flash_command == 1 && local_flash_complete)
		{
			ret += sprintf(buf + ret, "FlashStart:Complete \n");
			ret += sprintf(buf + ret, "FlashEnd");
			ret += sprintf(buf + ret, "\n");
			HX_PROC_SEND_FLAG=1;
			return ret;
		}

		if (local_flash_command == 3 && local_flash_complete)
		{
			ret += sprintf(buf + ret, "FlashStart: \n");
			for(loop_i = 0; loop_i < 128; loop_i++)
			{
				ret += sprintf(buf + ret, "x%2.2x", flash_buffer[loop_i]);
				if ((loop_i % 16) == 15)
				{
					ret += sprintf(buf + ret, "\n");
				}
			}
			ret += sprintf(buf + ret, "FlashEnd");
			ret += sprintf(buf + ret, "\n");
			HX_PROC_SEND_FLAG=1;
			return ret;
		}

		//flash command == 0 , report the data
		local_flash_read_step = getFlashReadStep();

		ret += sprintf(buf + ret, "FlashStart:%2.2x \n",local_flash_read_step);

		for (loop_i = 0; loop_i < 1024; loop_i++)
		{
			ret += sprintf(buf + ret, "x%2.2X", flash_buffer[local_flash_read_step*1024 + loop_i]);

			if ((loop_i % 16) == 15)
			{
				ret += sprintf(buf + ret, "\n");
			}
		}

		ret += sprintf(buf + ret, "FlashEnd");
		ret += sprintf(buf + ret, "\n");
		HX_PROC_SEND_FLAG=1;
	}
	else
		HX_PROC_SEND_FLAG=0;
	return ret;
}

static ssize_t himax_flash_write(struct file *file, const char *buff,
	size_t len, loff_t *pos)
{
	char buf_tmp[6];
	unsigned long result = 0;
	uint8_t loop_i = 0;
	int base = 0;
	char buf[80] = {0};

	if (len >= 80)
	{
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}
	if (copy_from_user(buf, buff, len))
	{
		return -EFAULT;
	}
	memset(buf_tmp, 0x0, sizeof(buf_tmp));

	I("%s: buf[0] = %s\n", __func__, buf);

	if (getSysOperation() == 1)
	{
		E("%s: SYS is busy , return!\n", __func__);
		return len;
	}

	if (buf[0] == '0')
	{
		setFlashCommand(0);
		if (buf[1] == ':' && buf[2] == 'x')
		{
			memcpy(buf_tmp, buf + 3, 2);
			I("%s: read_Step = %s\n", __func__, buf_tmp);
			if (!kstrtoul(buf_tmp, 16, &result))
			{
				I("%s: read_Step = %lu \n", __func__, result);
				setFlashReadStep(result);
			}
		}
	}
	else if (buf[0] == '1')
	{
		setSysOperation(1);
		setFlashCommand(1);
		setFlashDumpProgress(0);
		setFlashDumpComplete(0);
		setFlashDumpFail(0);
		queue_work(private_ts->flash_wq, &private_ts->flash_work);
	}
	else if (buf[0] == '2')
	{
		setSysOperation(1);
		setFlashCommand(2);
		setFlashDumpProgress(0);
		setFlashDumpComplete(0);
		setFlashDumpFail(0);

		queue_work(private_ts->flash_wq, &private_ts->flash_work);
	}
	else if (buf[0] == '3')
	{
		setSysOperation(1);
		setFlashCommand(3);
		setFlashDumpProgress(0);
		setFlashDumpComplete(0);
		setFlashDumpFail(0);

		memcpy(buf_tmp, buf + 3, 2);
		if (!kstrtoul(buf_tmp, 16, &result))
		{
			setFlashDumpSector(result);
		}

		memcpy(buf_tmp, buf + 7, 2);
		if (!kstrtoul(buf_tmp, 16, &result))
		{
			setFlashDumpPage(result);
		}

		queue_work(private_ts->flash_wq, &private_ts->flash_work);
	}
	else if (buf[0] == '4')
	{
		I("%s: command 4 enter.\n", __func__);
		setSysOperation(1);
		setFlashCommand(4);
		setFlashDumpProgress(0);
		setFlashDumpComplete(0);
		setFlashDumpFail(0);

		memcpy(buf_tmp, buf + 3, 2);
		if (!kstrtoul(buf_tmp, 16, &result))
		{
			setFlashDumpSector(result);
		}
		else
		{
			E("%s: command 4 , sector error.\n", __func__);
			return len;
		}

		memcpy(buf_tmp, buf + 7, 2);
		if (!kstrtoul(buf_tmp, 16, &result))
		{
			setFlashDumpPage(result);
		}
		else
		{
			E("%s: command 4 , page error.\n", __func__);
			return len;
		}

		base = 11;

		I("=========Himax flash page buffer start=========\n");
		for(loop_i=0;loop_i<128;loop_i++)
		{
			memcpy(buf_tmp, buf + base, 2);
			if (!kstrtoul(buf_tmp, 16, &result))
			{
				flash_buffer[loop_i] = result;
				I("%d ",flash_buffer[loop_i]);
				if (loop_i % 16 == 15)
				{
					I("\n");
				}
			}
			base += 3;
		}
		I("=========Himax flash page buffer end=========\n");

		queue_work(private_ts->flash_wq, &private_ts->flash_work);
	}
	return len;
}

static struct file_operations himax_proc_flash_ops =
{
	.owner = THIS_MODULE,
	.read = himax_flash_read,
	.write = himax_flash_write,
};
#endif

#ifdef HX_TP_SELF_TEST_DRIVER
static uint8_t Self_Test_Bank(uint8_t *RB1H)
{
	uint8_t i;
	uint32_t bank_sum, m;
	uint8_t bank_avg;
	uint8_t bank_ulmt, bank_dlmt;
	uint8_t bank_min, bank_max;
	uint8_t slf_tx_fail_cnt, slf_rx_fail_cnt;
	uint8_t mut_fail_cnt;
	int fail_flag;
	uint8_t bank_val_tmp;

	uint8_t set_bnk_ulmt;
	uint8_t set_bnk_dlmt;
	uint8_t set_avg_bnk_ulmt;
	uint8_t set_avg_bnk_dlmt;
	uint8_t set_slf_bnk_ulmt;
	uint8_t set_slf_bnk_dlmt;
	uint16_t mutual_num, self_num;
	uint8_t command_F1h_bank[2] = {0xF1, 0x03};

	//=============================== Get Bank value Start===============================
	I("Start self_test\n");

	mutual_num	= x_channel * y_channel;
	self_num	= x_channel + y_channel; //don't add KEY_COUNT

	mutual_bank = kzalloc(mutual_num * sizeof(uint8_t), GFP_KERNEL);
	self_bank = kzalloc(self_num * sizeof(uint8_t), GFP_KERNEL);
	memset(mutual_bank, 0xFF, mutual_num * sizeof(uint8_t));
	memset(self_bank, 0xFF, self_num * sizeof(uint8_t));

	I(" x_channel = %d, y_channel = %d \n",x_channel, y_channel);
	I(" mutual_num = %d, self_num = %d \n",mutual_num, self_num);
	I(" Selftest_flag = %d \n",Selftest_flag);

	i2c_himax_write(private_ts->client, command_F1h_bank[0] ,&command_F1h_bank[1], 1, DEFAULT_RETRY_CNT);
	msleep(200);
	command_F1h_bank[1] = 0x00;
	i2c_himax_write(private_ts->client, command_F1h_bank[0] ,&command_F1h_bank[1], 1, DEFAULT_RETRY_CNT);
	msleep(10);
	//=============================== Get Bank value End===============================

		if (RB1H[0] == 0)
		{
			I(" Enter Test flow \n");
			set_bnk_ulmt = RB1H[2];
			set_bnk_dlmt = RB1H[3];
			set_avg_bnk_ulmt = RB1H[4];
			set_avg_bnk_dlmt = RB1H[5];
			set_slf_bnk_ulmt = RB1H[6]; //Increase @ 2012/05/24 for weak open/short
			set_slf_bnk_dlmt = RB1H[7];

			fail_flag = 0;
			bank_sum = 0;
			bank_avg = 0;
			mut_fail_cnt = 0;

			//Calculate Bank Average
			for (m = 0; m < mutual_num; m++){
				bank_sum += mutual_bank[m];
			}
			I(" bank_sum = %d \n", bank_sum);
			bank_avg = (bank_sum / mutual_num);
			I(" bank_avg = %d \n", bank_avg);
			//======Condition 1======Check average bank with absolute value
			if ((bank_avg > set_avg_bnk_ulmt) || (bank_avg < set_avg_bnk_dlmt))
				fail_flag = 1;
			I(" fail_flag = %d\n", fail_flag);
			if (fail_flag)
			{
				RB1H[0] = 0xF1;     //Fail ID for Condition 1
				RB1H[1] = bank_avg;
				RB1H[2] = set_avg_bnk_ulmt;
				RB1H[3] = set_avg_bnk_dlmt;
				RB1H[4] = 0xFF;
				for (i = 0;i < 8; i++){
					I(" RB1H[%d] = %X \n", i, RB1H[i]);
				}
			}
			else
			{
		//======Condition 2======Check every block's bank with average value
				if ((bank_avg + set_bnk_ulmt) > 245)
					bank_ulmt = 245;
				else
					bank_ulmt = bank_avg + set_bnk_ulmt;

				if (bank_avg > set_bnk_dlmt)
				{
					bank_dlmt = bank_avg - set_bnk_dlmt;
					if (bank_dlmt < 10)
						bank_dlmt = 10;
				}
				else
				bank_dlmt = 10;
				bank_min = 0xFF;
				bank_max = 0x00;
				I(" bank_ulmt = %d, bank_dlmt = %d \n", bank_ulmt, bank_dlmt);
				for (m = 0; m < mutual_num; m++)
				{
					bank_val_tmp = mutual_bank[m];
					if ((bank_val_tmp > bank_ulmt) || (bank_val_tmp < bank_dlmt))
						{
							fail_flag = 1;
							mut_fail_cnt++;
						}

					//Bank information record
					if (bank_val_tmp > bank_max)
						bank_max = bank_val_tmp;
					else if (bank_val_tmp < bank_min)
						bank_min = bank_val_tmp;
				}
				I(" fail_flag = %d, mut_fail_cnt = %d \n", fail_flag, mut_fail_cnt);
				if (fail_flag)
				{
					RB1H[0] = 0xF2; //Fail ID for Condition 2
					RB1H[1] = mut_fail_cnt;
					RB1H[2] = bank_avg;
					RB1H[3] = bank_max;
					RB1H[4] = bank_min;
					RB1H[5] = bank_ulmt;
					RB1H[6] = bank_dlmt;
					RB1H[7] = 0xFF;
					for (i = 0;i < 8; i++){
						I(" RB1H[%d] = %X \n", i, RB1H[i]);
					}
					for (m = 0; m < mutual_num; m++){
						I(" mutual_bank[%d] = %X \n", m, mutual_bank[m]);
					}
					for (m = 0; m < self_num; m++){
						I(" self_bank[%d] = %X \n", m, self_bank[m]);
					}
				}
				else
				{
					//======Condition 3======Check every self channel bank
					slf_rx_fail_cnt = 0x00; //Check SELF RX BANK
					slf_tx_fail_cnt = 0x00; //Check SELF TX BANK
					for (i = 0; i < (x_channel + y_channel); i++)
					{
						bank_val_tmp = self_bank[i];
						if ((bank_val_tmp > set_slf_bnk_ulmt) ||
								(bank_val_tmp < set_slf_bnk_dlmt))
						{
							fail_flag = 1;
							if (i < x_channel)
								slf_rx_fail_cnt++;
							else
								slf_tx_fail_cnt++;
						}
					}
					I(" slf_rx_fail_cnt = %d, slf_tx_fail_cnt = %d \n", slf_rx_fail_cnt, slf_tx_fail_cnt);
					if (fail_flag)
					{
						RB1H[0] = 0xF3; //Fail ID for Condition 3
						RB1H[1] = slf_rx_fail_cnt;
						RB1H[2] = slf_tx_fail_cnt;
						RB1H[3] = set_slf_bnk_ulmt;
						RB1H[4] = set_slf_bnk_dlmt;
						RB1H[5] = 0xFF;
						for (i = 0;i < 8; i++){
							I(" RB1H[%d] = %X \n", i, RB1H[i]);
						}
						for (m = 0; m < mutual_num; m++){
							I(" mutual_bank[%d] = %X \n", m, mutual_bank[m]);
						}
						for (m = 0; m < self_num; m++){
							I(" self_bank[%d] = %X \n", m, self_bank[m]);
						}
					}
					else
					{
						RB1H[0] = 0xAA; ////PASS ID
						RB1H[1] = bank_avg;
						RB1H[2] = bank_max;
						RB1H[3] = bank_min;
					}
				}
			}
		}
	kfree(mutual_bank);
	kfree(self_bank);
	return RB1H[0];
}
#endif

#ifdef HX_TP_PROC_SELF_TEST
static int himax_chip_self_test(void)
{
	uint8_t cmdbuf[11];
	uint8_t valuebuf[16];
	int pf_value=0x00;
#ifdef HX_TP_SELF_TEST_DRIVER
	uint8_t RB1H[8];
#else
	int i=0;
#endif

	memset(cmdbuf, 0x00, sizeof(cmdbuf));
	memset(valuebuf, 0x00, sizeof(valuebuf));
#ifdef HX_TP_SELF_TEST_DRIVER
	memset(RB1H, 0x00, sizeof(RB1H));


	Selftest_flag = 1;
	diag_command = 0x03;
	himax_int_enable(private_ts->client->irq,1,true);
	//Get Criteria
	i2c_himax_read(private_ts->client, 0xB1, RB1H, 8, DEFAULT_RETRY_CNT);
	msleep(10);
#else
	cmdbuf[0] = 0x06;
	i2c_himax_write(private_ts->client, 0xF1,&cmdbuf[0], 1, DEFAULT_RETRY_CNT);
	msleep(120);
#endif
	i2c_himax_write(private_ts->client, HX_CMD_TSSON,&cmdbuf[0], 0, DEFAULT_RETRY_CNT);
	msleep(120);

	i2c_himax_write(private_ts->client, HX_CMD_TSSLPOUT,&cmdbuf[0], 0, DEFAULT_RETRY_CNT);
#ifdef HX_TP_SELF_TEST_DRIVER
	msleep(120);
	valuebuf[0] = Self_Test_Bank(&RB1H[0]);
#else
	msleep(2000);
#endif

	i2c_himax_write(private_ts->client, HX_CMD_TSSOFF,&cmdbuf[0], 0, DEFAULT_RETRY_CNT);
	msleep(120);

	i2c_himax_write(private_ts->client, HX_CMD_TSSLPIN,&cmdbuf[0], 0, DEFAULT_RETRY_CNT);
	msleep(120);

#ifdef HX_TP_SELF_TEST_DRIVER
	himax_int_enable(private_ts->client->irq,0,true);
	diag_command = 0x00;
	Selftest_flag = 0;
#else
	cmdbuf[0] = 0x00;
	i2c_himax_write(private_ts->client, 0xF1,&cmdbuf[0], 1, DEFAULT_RETRY_CNT);
	msleep(120);

	i2c_himax_read(private_ts->client, 0xB1, valuebuf, 8, DEFAULT_RETRY_CNT);
	msleep(10);

	for(i=0;i<8;i++) {
		I("[Himax]: After slf test 0xB1 buff_back[%d] = 0x%x\n",i,valuebuf[i]);
	}

	msleep(30);
#endif
	if (valuebuf[0]==0xAA) {
		I("[Himax]: self-test pass\n");
		pf_value = 0x0;
	} else {
		E("[Himax]: self-test fail, result : 0x%02x\n", valuebuf[0]);
		pf_value = 0x1;
	}			
	
	return pf_value;
}

static ssize_t himax_self_test_read(struct file *file, char *buf,
	size_t len, loff_t *pos)
{
	int val = 0x00;
	int ret = 0;

	/* 20160216 for Speed Fw */
	self_test_inter_flag = 1;

	if (!HX_PROC_SEND_FLAG) {
		val = himax_chip_self_test();

		if (val == 0x00)
			ret += sprintf(buf + ret, "Self_Test Pass\n");
		else
			ret += sprintf(buf + ret, "Self_Test Fail\n");

		HX_PROC_SEND_FLAG = 1;
	} else {
		HX_PROC_SEND_FLAG = 0;
	}

	self_test_inter_flag = 0;/* 20160216 for Speed Fw */
	return ret;
}

static struct file_operations himax_proc_self_test_ops =
{
	.owner = THIS_MODULE,
	.read = himax_self_test_read,
};

/* cci selftest start */
/* dean add : create file for record rawdata */
static mm_segment_t oldfs;

#define TEST_BANK 1
#define TEST_NOISE 2

static struct file *himax_selftest_file_open(u8 test_item)
{
	struct file *filp = NULL;
	char filepath[128];
	int err = 0;

	memset(filepath, 0, sizeof(filepath));

	switch (test_item) {
	case TEST_BANK:
		snprintf(filepath, 128, "/sdcard/%s", "test_bank.txt");
	break;
	case TEST_NOISE:
		snprintf(filepath, 128, "/sdcard/%s", "test_noise.txt");
	break;
	default:
		return NULL;
	break;
	}

	oldfs = get_fs();
	set_fs(get_ds());

	filp = filp_open(filepath, O_WRONLY|O_CREAT , 0644);
	if (IS_ERR(filp)) {
		err = PTR_ERR(filp);
		return NULL;
	}

	return filp;
}

int himax_selftest_file_write(struct file *file, unsigned char *data, int len)
{

	int ret;

	ret = file->f_op->write(file, data, len, &file->f_pos);

	return ret;
}

void himax_selftest_file_close(struct file *file)
{

	set_fs(oldfs);
	filp_close(file, NULL);
}

static int himax_chip_cci_self_test(void)
{
	uint8_t cmdbuf[11];
	uint8_t valuebuf[16];
	int pf_value = 0x00;
	uint8_t cmd[128];

#ifdef HX_TP_SELF_TEST_DRIVER
	uint8_t RB1H[8];
#else
	int i = 0;
#endif

	memset(cmdbuf, 0x00, sizeof(cmdbuf));
	memset(valuebuf, 0x00, sizeof(valuebuf));

	D("%s: start\n", __func__);
	/* dean add : selftest in driver */
	i2c_himax_write_command(private_ts->client, 0x82, DEFAULT_RETRY_CNT);
	msleep(300);
	i2c_himax_write_command(private_ts->client, 0x80, DEFAULT_RETRY_CNT);
	msleep(300);

	D("%s: i2c write 82 & 80 OK\n", __func__);

	/* PVT criteria*/
	/* echo "w:xb1:x00:x01:x2E:x2B:x7B:x49:xB6:x32" > /proc/android_touch/register */
	cmd[0] = 0x00;
	cmd[1] = 0x01;	/* enable noise item */
	/* mutual bank */
	cmd[2] = 0x2E;	/* avg upper limit */
	cmd[3] = 0x2B;	/* avg lower limit */
	cmd[4] = 0x7B;	/* avg upper limit for absolute value */
	cmd[5] = 0x49;	/* avg lower limit for absolute value */
	/* self bank */
	cmd[6] = 0xB6;	/* avg upper limit for absolute value */
	cmd[7] = 0x32;	/* avg lower limit for absolute value */

	if (i2c_himax_write(private_ts->client, 0xb1, &cmd[0], 8, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		pf_value = 0x01;
		return pf_value;
	}
	D("%s: i2c write criteria OK\n", __func__);
	/* dean add : end */

#ifdef HX_TP_SELF_TEST_DRIVER
	memset(RB1H, 0x00, sizeof(RB1H));

	Selftest_flag = 1;
	diag_command = 0x03;
	himax_int_enable(private_ts->client->irq, 1, true);
	/* Get Criteria */
	i2c_himax_read(private_ts->client, 0xB1, RB1H, 8, DEFAULT_RETRY_CNT);
	msleep(10);
#else
	cmdbuf[0] = 0x06;
	i2c_himax_write(private_ts->client, 0xF1,&cmdbuf[0], 1, DEFAULT_RETRY_CNT);
	msleep(120);
#endif

	D("%s: i2c write 0xF1 0x06 OK\n", __func__);

	i2c_himax_write(private_ts->client, HX_CMD_TSSON,&cmdbuf[0], 0, DEFAULT_RETRY_CNT);
	msleep(120);
	D("%s: i2c write HX_CMD_TSSON OK\n", __func__);

	i2c_himax_write(private_ts->client, HX_CMD_TSSLPOUT,&cmdbuf[0], 0, DEFAULT_RETRY_CNT);
	D("%s: i2c write HX_CMD_TSSLPOUT OK\n", __func__);

#ifdef HX_TP_SELF_TEST_DRIVER
	msleep(120);
	valuebuf[0] = Self_Test_Bank(&RB1H[0]);
#else
	msleep(2000);
#endif
	D("%s: Self Test Bank ret : 0x%x\n", __func__, RB1H[0]);

	i2c_himax_write(private_ts->client, HX_CMD_TSSOFF,&cmdbuf[0], 0, DEFAULT_RETRY_CNT);
	msleep(120);
	D("%s: i2c write HX_CMD_TSSOFF OK\n", __func__);

	i2c_himax_write(private_ts->client, HX_CMD_TSSLPIN,&cmdbuf[0], 0, DEFAULT_RETRY_CNT);
	msleep(120);
	D("%s: i2c write HX_CMD_TSSLPIN OK\n", __func__);

#ifdef HX_TP_SELF_TEST_DRIVER
	himax_int_enable(private_ts->client->irq,0,true);
	diag_command = 0x00;
	Selftest_flag = 0;
#else
	cmdbuf[0] = 0x00;
	i2c_himax_write(private_ts->client, 0xF1,&cmdbuf[0], 1, DEFAULT_RETRY_CNT);
	msleep(120);
	D("%s: i2c write 0xF1 00 OK\n", __func__);

	i2c_himax_read(private_ts->client, 0xB1, valuebuf, 8, DEFAULT_RETRY_CNT);
	msleep(10);
	D("%s: i2c write 0xB1 OK\n", __func__);

	for(i=0; i<8; i++)
		I("[Himax]: After slf test 0xB1 buff_back[%d] = 0x%x\n",i,valuebuf[i]);
#endif
	msleep(30);

	if (valuebuf[0] == 0xAA) {
		I("%s: self-test pass\n", __func__);
		pf_value = 0x00;
	} else {
		E("%s: self-test fail : 0x%x\n", __func__, valuebuf[0]);
		pf_value = 0x01;
	}

	return pf_value;
}

static int himax_chip_cci_get_bank(struct file *file)
{
	uint8_t cmd[1];

	char tmp_buf[128];
	int w_len = 0;
	int tmp_len = 0;
	uint32_t loop_i;
	uint16_t mutual_num, self_num, width;

	char *w_buf = NULL;
	int w_buf_len = 1024*2;
	int ret = -1;

	w_buf = kmalloc(w_buf_len, GFP_ATOMIC);

	if (w_buf == NULL) {
		E("%s: alloc buffer fail!\n", __func__);
		goto err_alloc_buffer_fail;
	}

	diag_command = 3;	/* rawdata bank */

	/* get bank */
	/* echo w:xF1:x03 > /proc/android_touch/register */
	cmd[0] = diag_command;
	if (i2c_himax_write(private_ts->client, 0xF1, &cmd[0], 1, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		goto err_i2c_write_fail;
	}

	msleep(300);

	mutual_num	= x_channel * y_channel;
	self_num	= x_channel + y_channel; /* don't add KEY_COUNT */
	width		= x_channel;
	I("ChannelStart: %4d, %4d\n\n", x_channel, y_channel);
	tmp_len = snprintf(tmp_buf, 128, "ChannelStart: %4d, %4d\n\n", x_channel, y_channel);
	memcpy(w_buf + w_len, tmp_buf, tmp_len);
	w_len += tmp_len;

	I(" ");
	for (loop_i = 0; loop_i < mutual_num; loop_i++) {
		printk("%4d", diag_mutual[loop_i]);
		tmp_len = snprintf(tmp_buf, 8, "%4d", diag_mutual[loop_i]);
		memcpy(w_buf + w_len, tmp_buf, tmp_len);
		w_len += tmp_len;

		if ((loop_i % width) == (width - 1)) {
			printk(" %3d\n", diag_self[width + loop_i/width]);
			tmp_len = snprintf(tmp_buf, 8, " %3d\n", diag_self[width + loop_i/width]);
			memcpy(w_buf + w_len, tmp_buf, tmp_len);
			w_len += tmp_len;

			I(" ");
		}
	}

	I("\n");
	tmp_len = snprintf(tmp_buf, 8, "\n");
	memcpy(w_buf + w_len, tmp_buf, tmp_len);
	w_len += tmp_len;

	I(" ");
	for (loop_i = 0; loop_i < width; loop_i++) {
		printk("%4d", diag_self[loop_i]);
		tmp_len = snprintf(tmp_buf, 8, "%4d", diag_self[loop_i]);
		memcpy(w_buf + w_len, tmp_buf, tmp_len);
		w_len += tmp_len;

		if (((loop_i) % width) == (width - 1)) {
			printk("\n");
			tmp_len = snprintf(tmp_buf, 8, "\n");
			memcpy(w_buf + w_len, tmp_buf, tmp_len);
			w_len += tmp_len;

			I(" ");
		}
	}

	I("ChannelEnd\n");
	tmp_len = snprintf(tmp_buf, 16, "ChannelEnd\n");
	memcpy(w_buf + w_len, tmp_buf, tmp_len);
	w_len += tmp_len;

	if (w_len < w_buf_len) {
		ret = himax_selftest_file_write(file, w_buf, w_len);
		if (ret < 0)
			E("%s : Write file fail !\n", __func__);
	} else {
		E("%s : w_len > w_buf_len !\n", __func__);
	}

	/* disable */
	/* echo 0 > /proc/android_touch/diag */
	diag_command = 0;
	cmd[0] = 0;
	if (i2c_himax_write(private_ts->client, 0xF1, &cmd[0], 1, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		goto err_i2c_write_fail;
	}

	if(w_buf != NULL)
		kfree(w_buf);

	if (ret < 0)
		return -1;
	else
		return 0;

err_i2c_write_fail:
	if(w_buf != NULL)
		kfree(w_buf);

err_alloc_buffer_fail:
	return -1;
}

static int himax_chip_cci_get_iir(struct file *file)
{
	uint8_t cmd[1];

	char tmp_buf[128];
	int w_len = 0;
	int tmp_len = 0;
	uint32_t loop_i, loop_j;
	uint16_t mutual_num, self_num, width;

	char *w_buf = NULL;
	int w_buf_len = 1024*2;
	int ret = -1;

	u8 data_num = 50;

	w_buf = kmalloc(w_buf_len, GFP_ATOMIC);

	if (w_buf == NULL) {
		E("%s: alloc buffer fail!\n", __func__);
		goto err_alloc_buffer_fail;
	}

	diag_command = 1;	/* rawdata IIR */

	/* get bank */
	/* echo w:xF1:x01 > /proc/android_touch/register */
	cmd[0] = diag_command;
	if (i2c_himax_write(private_ts->client, 0xF1, &cmd[0], 1, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		goto err_i2c_write_fail;
	}

	msleep(300);

	mutual_num	= x_channel * y_channel;
	self_num	= x_channel + y_channel; /* don't add KEY_COUNT */
	width		= x_channel;

	for (loop_j = 0; loop_j < data_num; loop_j++) {
		I("ChannelStart: %4d, %4d\n\n", x_channel, y_channel);
		tmp_len = snprintf(tmp_buf, 128, "ChannelStart: %4d, %4d\n\n", x_channel, y_channel);
		memcpy(w_buf + w_len, tmp_buf, tmp_len);
		w_len += tmp_len;

		I(" ");
		for (loop_i = 0; loop_i < mutual_num; loop_i++) {
			printk("%4d", diag_mutual[loop_i]);
			tmp_len = snprintf(tmp_buf, 8, "%4d", diag_mutual[loop_i]);
			memcpy(w_buf + w_len, tmp_buf, tmp_len);
			w_len += tmp_len;

			if ((loop_i % width) == (width - 1)) {
				printk(" %3d\n", diag_self[width + loop_i/width]);
				tmp_len = snprintf(tmp_buf, 8, " %3d\n", diag_self[width + loop_i/width]);
				memcpy(w_buf + w_len, tmp_buf, tmp_len);
				w_len += tmp_len;

				I(" ");
			}
		}

		I("\n");
		tmp_len = snprintf(tmp_buf, 8, "\n");
		memcpy(w_buf + w_len, tmp_buf, tmp_len);
		w_len += tmp_len;

		I(" ");
		for (loop_i = 0; loop_i < width; loop_i++) {
			printk("%4d", diag_self[loop_i]);
			tmp_len = snprintf(tmp_buf, 8, "%4d", diag_self[loop_i]);
			memcpy(w_buf + w_len, tmp_buf, tmp_len);
			w_len += tmp_len;

			if (((loop_i) % width) == (width - 1)) {
				printk("\n");
				tmp_len = snprintf(tmp_buf, 8, "\n");
				memcpy(w_buf + w_len, tmp_buf, tmp_len);
				w_len += tmp_len;

				I(" ");
			}
		}

		I("ChannelEnd\n");
		tmp_len = snprintf(tmp_buf, 16, "ChannelEnd\n");
		memcpy(w_buf + w_len, tmp_buf, tmp_len);
		w_len += tmp_len;

		msleep(50);

		if (w_len < w_buf_len) {
			ret = himax_selftest_file_write(file, w_buf, w_len);
			if (ret < 0) {
				E("%s : Write file fail !\n", __func__);
				break;
			}
		} else {
			E("%s : w_len > w_buf_len !\n", __func__);
			break;
		}
		w_len = 0;
		memset(w_buf, 0, w_buf_len);
	}

	/* disable */
	/* echo 0 > /proc/android_touch/diag */
	diag_command = 0;
	cmd[0] = 0;
	if (i2c_himax_write(private_ts->client, 0xF1, &cmd[0], 1, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		goto err_i2c_write_fail;
	}

	if(w_buf != NULL)
		kfree(w_buf);

	if (ret < 0)
		return -1;
	else
		return 0;

err_i2c_write_fail:
	if(w_buf != NULL)
		kfree(w_buf);

err_alloc_buffer_fail:
	return -1;
}

static int himax_cci_self_test_read(struct seq_file *m, void *v)
{
	int val = 0x00;

	/* dean add : selftest in driver */
	struct file *w_file_bank, *w_file_noise;
	char result_buf[32];

	int w_len = 0;
	int ret;

	D("%s : self test start\n", __func__);

	w_file_bank = himax_selftest_file_open(TEST_BANK);
	if (!w_file_bank) {
		E("%s : Open log bank file fail !\n", __func__);
		goto err_open_bank_file_fail;
	}
	w_file_noise = himax_selftest_file_open(TEST_NOISE);
	if (!w_file_noise) {
		E("%s : Open log noise file fail !\n", __func__);
		goto err_open_noise_file_fail;
	}
	/* dean add : end */

	self_test_inter_flag = 1;


	val = himax_chip_cci_self_test();

	if (val == 0x00) {
		seq_printf(m, "Self_Test Pass\n");
		D("%s : self test Pass\n", __func__);
		w_len = snprintf(result_buf, 32, "Self_Test Pass\n");
	} else {
		seq_printf(m, "Self_Test Fail\n");
		D("%s : self test Fail\n", __func__);
		w_len = snprintf(result_buf, 32, "Self_Test fail\n");
	}


	D("%s : write file selftest result\n", __func__);
	/* dean add : selftest in driver */
	ret = himax_selftest_file_write(w_file_bank, result_buf, w_len);
	if (ret < 0) {
		E("%s : write selftest result fail !\n", __func__);
		goto err_write_bank_file_fail;
	}

	himax_HW_reset(true, false);

	D("%s : write file testbank\n", __func__);
	ret = himax_chip_cci_get_bank(w_file_bank);
	if (ret < 0) {
		E("%s : write selftest rawdata fail !\n", __func__);
		goto err_write_bank_file_fail;
	}

	D("%s : write file testnoise\n", __func__);
	ret = himax_chip_cci_get_iir(w_file_noise);
	if (ret < 0) {
		E("%s : write noise rawdata fail !\n", __func__);
		goto err_write_noise_file_fail;
	}

	himax_selftest_file_close(w_file_bank);
	himax_selftest_file_close(w_file_noise);

	himax_HW_reset(true, false);
	self_test_inter_flag = 0;


err_write_noise_file_fail:
err_write_bank_file_fail:
	himax_selftest_file_close(w_file_noise);

err_open_noise_file_fail:
	himax_selftest_file_close(w_file_bank);

err_open_bank_file_fail:

	return 0;
}

static int himax_cci_self_test_read_open(struct inode *inode, struct file *file)
{
	return single_open(file, himax_cci_self_test_read, inode->i_private);
}

static const struct file_operations himax_proc_cci_self_test_ops = {
	.owner = THIS_MODULE,
	.open = himax_cci_self_test_read_open,
	.read = seq_read,
	.release = single_release,
};

/* cci selftest end */
#endif

#ifdef HX_TP_PROC_HITOUCH
static ssize_t himax_hitouch_read(struct file *file, char *buf,
	size_t len, loff_t *pos)
{
	int ret = 0;
	if(!HX_PROC_SEND_FLAG)
	{
		if(hitouch_command == 0)
		{
			ret += sprintf(buf + ret, "Himax Touch Driver Version:\n");
			ret += sprintf(buf + ret, "%s \n", HIMAX_DRIVER_VER);
		}
		else if(hitouch_command == 1)
		{
			ret += sprintf(buf + ret, "hitouch_is_connect = true\n");
		}
		else if(hitouch_command == 2)
		{
			ret += sprintf(buf + ret, "hitouch_is_connect = false\n");
		}
		HX_PROC_SEND_FLAG=1;
	}
	else
		HX_PROC_SEND_FLAG=0;

	return ret;
}

//-----------------------------------------------------------------------------------
//himax_hitouch_store
//command 0 : Get Driver Version
//command 1 : Hitouch Connect
//command 2 : Hitouch Disconnect
//-----------------------------------------------------------------------------------
static ssize_t himax_hitouch_write(struct file *file, const char *buff,
	size_t len, loff_t *pos)
{
	char buf[80] = {0};

	if (len >= 80)
	{
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}
	if (copy_from_user(buf, buff, len))
	{
		return -EFAULT;
	}

	if(buf[0] == '0')
	{
		hitouch_command = 0;
	}
	else if(buf[0] == '1')
	{
		hitouch_command = 1;
		hitouch_is_connect = true;	
		I("hitouch_is_connect = true\n");	
	}
	else if(buf[0] == '2')
	{
		hitouch_command = 2;
		hitouch_is_connect = false;
		I("hitouch_is_connect = false\n"); 
	}
	return len;
}

static struct file_operations himax_proc_hitouch_ops =
{
	.owner = THIS_MODULE,
	.read = himax_hitouch_read,
	.write = himax_hitouch_write,
};
#endif

#ifdef HX_DOT_VIEW
static ssize_t himax_cover_read(struct file *file, char *buf,
	size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = private_ts;
	size_t ret = 0;

	if(!HX_PROC_SEND_FLAG)
	{
		ret = snprintf(buf, PAGE_SIZE, "%d\n", ts->cover_enable);
		HX_PROC_SEND_FLAG=1;
	}
	else
		HX_PROC_SEND_FLAG=0;

	return ret;
}

static ssize_t himax_cover_write(struct file *file, const char *buff,
	size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = private_ts;
	char buf[80] = {0};

	if (len >= 80)
	{
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}
	if (copy_from_user(buf, buff, len))
	{
		return -EFAULT;
	}

	if(buf[0] == '0')
		ts->cover_enable = 0;
	else if(buf[0] == '1')
		ts->cover_enable = 1;
	else
		return -EINVAL;
	himax_set_cover_func(ts->cover_enable);

	I("%s: cover_enable = %d.\n", __func__, ts->cover_enable);

	return len;
}

static struct file_operations himax_proc_cover_ops =
{
	.owner = THIS_MODULE,
	.read = himax_cover_read,
	.write = himax_cover_write,
};
#endif

#ifdef HX_SMART_WAKEUP
static ssize_t himax_SMWP_read(struct file *file, char *buf,
	size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = private_ts;
	size_t ret = 0;

	if(!HX_PROC_SEND_FLAG)
	{
		ret = snprintf(buf, PAGE_SIZE, "%d\n", ts->SMWP_enable);
		HX_PROC_SEND_FLAG=1;
	}
	else
		HX_PROC_SEND_FLAG=0;

	return ret;
}

static ssize_t himax_SMWP_write(struct file *file, const char *buff,
	size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = private_ts;
	char buf[80] = {0};

	if (atomic_read(&private_ts->suspend_mode) == 1 && atomic_read(&private_ts->resume_mode) == 0) {
		E("%s: system is suspend mode, cannot set SMWP mode.\n", __func__);
		return len;
	}

	if (len >= 80)
	{
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}
	if (copy_from_user(buf, buff, len))
	{
		return -EFAULT;
	}

	if(buf[0] == '0')
		ts->SMWP_enable = 0;
	else if(buf[0] == '1')
		ts->SMWP_enable = 1;
	else
		return -EINVAL;
	
	I("%s: SMART_WAKEUP_enable = %d.\n", __func__, ts->SMWP_enable);

	return len;
}

static struct file_operations himax_proc_SMWP_ops =
{
	.owner = THIS_MODULE,
	.read = himax_SMWP_read,
	.write = himax_SMWP_write,
};

static ssize_t himax_GESTURE_read(struct file *file, char *buf,
	size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = private_ts;
	size_t ret = 0;

	if(HX_PROC_SEND_FLAG<16)
	{
		ret = sprintf(buf, "ges_en[%d]=%d \n",HX_PROC_SEND_FLAG ,ts->gesture_cust_en[HX_PROC_SEND_FLAG]);
		HX_PROC_SEND_FLAG++;
	}
	else
	{
		HX_PROC_SEND_FLAG = 0;
		ret = 0;
	}
	return ret;
}

static ssize_t himax_GESTURE_write(struct file *file, const char *buff,
	size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = private_ts;
	int i =0;
	char buf[80] = {0};

	if (len >= 80)
	{
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}
	if (copy_from_user(buf, buff, len))
	{
		return -EFAULT;
	}

	for (i=0;i<16;i++)
		{
			if (buf[i] == '0')
				ts->gesture_cust_en[i]= 0;
			else if (buf[i] == '1')
				ts->gesture_cust_en[i]= 1;
			else
				ts->gesture_cust_en[i]= 0;
			I("gesture en[%d]=%d \n", i, ts->gesture_cust_en[i]);
		}
	return len;
}

static struct file_operations himax_proc_Gesture_ops =
{
	.owner = THIS_MODULE,
	.read = himax_GESTURE_read,
	.write = himax_GESTURE_write,
};

static ssize_t himax_SMWP_on_read(struct file *file, char *buf,
	size_t len, loff_t *pos)
{
	size_t ret = 0;

	if (!HX_PROC_SEND_FLAG) {
		private_ts->SMWP_enable = 1;
		I("%s: SMART_WAKEUP_enable = %d.\n", __func__, private_ts->SMWP_enable);
		ret += sprintf(buf + ret,
				"SMWP status = On\n");

		HX_PROC_SEND_FLAG=1;
	} else {
		HX_PROC_SEND_FLAG=0;
	}

	return ret;
}

static struct file_operations himax_proc_SMWP_on_ops =
{
	.owner = THIS_MODULE,
	.read = himax_SMWP_on_read,
};

static ssize_t himax_SMWP_off_read(struct file *file, char *buf,
	size_t len, loff_t *pos)
{
	size_t ret = 0;

	if (!HX_PROC_SEND_FLAG) {
		private_ts->SMWP_enable = 0;
		I("%s: SMART_WAKEUP_enable = %d.\n", __func__, private_ts->SMWP_enable);
		ret += sprintf(buf + ret,
				"SMWP status = Off\n");

		HX_PROC_SEND_FLAG=1;
	} else {
		HX_PROC_SEND_FLAG=0;
	}

	return ret;
}

static struct file_operations himax_proc_SMWP_off_ops =
{
	.owner = THIS_MODULE,
	.read = himax_SMWP_off_read,
};
#endif

static int himax_touch_proc_init(void)
{
	himax_touch_proc_dir = proc_mkdir( HIMAX_PROC_TOUCH_FOLDER, NULL);
	if (himax_touch_proc_dir == NULL)
	{

		E(" %s: himax_touch_proc_dir file create failed!\n", __func__);
		return -ENOMEM;
	}

	himax_proc_debug_level_file = proc_create(HIMAX_PROC_DEBUG_LEVEL_FILE, (S_IWUSR|S_IRUGO),
		himax_touch_proc_dir, &himax_proc_debug_level_ops);
	if (himax_proc_debug_level_file == NULL)
	{
		E(" %s: proc debug_level file create failed!\n", __func__);
		goto fail_1;
	}

	himax_proc_vendor_file = proc_create(HIMAX_PROC_VENDOR_FILE, (S_IRUGO),
		himax_touch_proc_dir, &himax_proc_vendor_ops);
	if(himax_proc_vendor_file == NULL)
	{
		E(" %s: proc vendor file create failed!\n", __func__);
		goto fail_2;
	}

	himax_proc_attn_file = proc_create(HIMAX_PROC_ATTN_FILE, (S_IRUGO),
		himax_touch_proc_dir, &himax_proc_attn_ops);
	if(himax_proc_attn_file == NULL)	
	{
		E(" %s: proc attn file create failed!\n", __func__);
		goto fail_3;
	}

	himax_proc_int_en_file = proc_create(HIMAX_PROC_INT_EN_FILE, (S_IWUSR|S_IRUGO),
		himax_touch_proc_dir, &himax_proc_int_en_ops);
	if(himax_proc_int_en_file == NULL)
	{
		E(" %s: proc int en file create failed!\n", __func__);
		goto fail_4;
	}

	himax_proc_layout_file = proc_create(HIMAX_PROC_LAYOUT_FILE, (S_IWUSR|S_IRUGO),
		himax_touch_proc_dir, &himax_proc_layout_ops);
	if(himax_proc_layout_file == NULL)
	{
		E(" %s: proc layout file create failed!\n", __func__);
		goto fail_5;
	}
#ifdef HX_TP_PROC_RESET
	himax_proc_reset_file = proc_create(HIMAX_PROC_RESET_FILE, (S_IWUSR),
		himax_touch_proc_dir, &himax_proc_reset_ops);
	if(himax_proc_reset_file == NULL)
	{
		E(" %s: proc reset file create failed!\n", __func__);
		goto fail_6;
	}
#endif

#ifdef HX_TP_PROC_DIAG
	himax_proc_diag_file = proc_create(HIMAX_PROC_DIAG_FILE, (S_IWUSR|S_IRUGO),
		himax_touch_proc_dir, &himax_proc_diag_ops);
	if(himax_proc_diag_file == NULL)
	{
		E(" %s: proc diag file create failed!\n", __func__);
		goto fail_7;
	}
#endif

#ifdef HX_TP_PROC_REGISTER
	himax_proc_register_file = proc_create(HIMAX_PROC_REGISTER_FILE, (S_IWUSR|S_IRUGO),
		himax_touch_proc_dir, &himax_proc_register_ops);
	if(himax_proc_register_file == NULL)
	{
		E(" %s: proc register file create failed!\n", __func__);
		goto fail_8;
	}
#endif

#ifdef HX_TP_PROC_DEBUG
	himax_proc_debug_file = proc_create(HIMAX_PROC_DEBUG_FILE, (S_IWUSR|S_IRUGO),
		himax_touch_proc_dir, &himax_proc_debug_ops);
	if(himax_proc_debug_file == NULL)
	{
		E(" %s: proc debug file create failed!\n", __func__);
		goto fail_9;
	}
#endif

#ifdef HX_TP_PROC_FLASH_DUMP
	himax_proc_flash_dump_file = proc_create(HIMAX_PROC_FLASH_DUMP_FILE, (S_IWUSR|S_IRUGO),
		himax_touch_proc_dir, &himax_proc_flash_ops);
	if(himax_proc_flash_dump_file == NULL)
	{
		E(" %s: proc flash dump file create failed!\n", __func__);
		goto fail_10;
	}
#endif
#ifdef HX_TP_PROC_SELF_TEST
	himax_proc_self_test_file = proc_create(HIMAX_PROC_SELF_TEST_FILE, (S_IRUGO),
		himax_touch_proc_dir, &himax_proc_self_test_ops);
	if(himax_proc_self_test_file == NULL)
	{
		E(" %s: proc self_test file create failed!\n", __func__);
		goto fail_11;
	}

	himax_proc_cci_self_test_file = proc_create(HIMAX_PROC_CCI_SELF_TEST_FILE, (S_IRUGO),
		himax_touch_proc_dir, &himax_proc_cci_self_test_ops);
	if(himax_proc_cci_self_test_file == NULL)
	{
		E(" %s: proc self_test file create failed!\n", __func__);
		goto fail_11_1;
	}
#endif

#ifdef HX_TP_PROC_HITOUCH
	himax_proc_hitouch_file = proc_create(HIMAX_PROC_HITOUCH_FILE, (S_IWUSR|S_IRUGO),
		himax_touch_proc_dir, &himax_proc_hitouch_ops);
	if(himax_proc_hitouch_file == NULL)
	{
		E(" %s: proc hitouch file create failed!\n", __func__);
		goto fail_12;
	}
#endif

#ifdef HX_DOT_VIEW
	himax_proc_cover_file = proc_create(HIMAX_PROC_COVER_FILE, (S_IWUSR|S_IRUGO|S_IWUGO),
		himax_touch_proc_dir, &himax_proc_cover_ops);
	if(himax_proc_cover_file == NULL)
	{
		E(" %s: proc cover file create failed!\n", __func__);
		goto fail_13;
	}
#endif

#ifdef HX_SMART_WAKEUP
	himax_proc_SMWP_file = proc_create(HIMAX_PROC_SMWP_FILE, (S_IWUSR|S_IRUGO|S_IWUGO),
		himax_touch_proc_dir, &himax_proc_SMWP_ops);
	if(himax_proc_SMWP_file == NULL)
	{
		E(" %s: proc SMWP file create failed!\n", __func__);
		goto fail_14;
	}
	himax_proc_GESTURE_file = proc_create(HIMAX_PROC_GESTURE_FILE, (S_IWUSR|S_IRUGO|S_IWUGO),
		himax_touch_proc_dir, &himax_proc_Gesture_ops);
	if(himax_proc_GESTURE_file == NULL)
	{
		E(" %s: proc GESTURE file create failed!\n", __func__);
		goto fail_14;
	}
#endif

//===================== add fwversion 20160119
#ifdef HX_TP_FWVERSION
	himax_proc_fwversion_file = proc_create(HIMAX_PROC_FWVERSION_FILE, (S_IRUGO),
		himax_touch_proc_dir, &himax_proc_fwversion_ops);
	if(himax_proc_fwversion_file == NULL)
	{
		E(" %s: proc FW_VERSION file create failed!\n", __func__);
		goto fail_15;
	}

	himax_proc_fwversion_for_i_file = proc_create(HIMAX_PROC_FWVERSION_FOR_I_FILE, (S_IRUGO),
		himax_touch_proc_dir, &himax_proc_fwversion_for_i_ops);
	if(himax_proc_fwversion_for_i_file == NULL)
	{
		E(" %s: proc FW_VERSION file create failed!\n", __func__);
		goto fail_15_1;
	}
#endif
//=====================

//===================== add fw_status 20160119
#ifdef HX_TP_FWSTATE_CHECK
	himax_proc_fwstate_file = proc_create(HIMAX_PROC_FWSTATE_FILE, (S_IRUGO),
		himax_touch_proc_dir, &himax_proc_fwstate_ops);

	himax_proc_intstate_file = proc_create(HIMAX_PROC_INTSTATE_FILE, (S_IRUGO),
		himax_touch_proc_dir, &himax_proc_intstate_ops);

	if(himax_proc_fwstate_file == NULL)
	{
		E(" %s: proc FW_STATE file create failed!\n", __func__);
		goto fail_16;
	}
	if(himax_proc_intstate_file == NULL)
	{
		E(" %s: proc INT_STATE file create failed!\n", __func__);
		goto fail_17;
	}
#endif
//=====================

//===================== add TPID_CHECK 20160201
#ifdef HX_TPID_CHECK
	himax_proc_tpid_check_file = proc_create(HIMAX_PROC_TPID_CHECK_FILE, (S_IRUGO),
		himax_touch_proc_dir, &himax_proc_tpid_check_ops);
	if(himax_proc_tpid_check_file == NULL)
	{
		E(" %s: proc TPID_CHECK file create failed!\n", __func__);
		goto fail_18;
	}
#endif
//=====================

/* add fw_check */
#ifdef HX_TP_FW_CHECK
	himax_proc_fw_check_file = proc_create(HIMAX_PROC_FW_CHECK_FILE, (S_IRUGO),
		himax_touch_proc_dir, &himax_proc_fw_check_ops);
	if(himax_proc_fw_check_file == NULL)
	{
		E(" %s: proc FW_CHECK file create failed!\n", __func__);
		goto fail_19;
	}
#endif

/* add sensor id */
#ifdef HX_TP_SENSOR_ID
	himax_proc_sensor_id_file = proc_create(HIMAX_PROC_SENSOR_ID_FILE, (S_IRUGO),
		himax_touch_proc_dir, &himax_proc_sensor_id_ops);
	if(himax_proc_sensor_id_file == NULL)
	{
		E(" %s: proc sensor id file create failed!\n", __func__);
		goto fail_20;
	}
#endif

/* add glove mode */
#ifdef HX_TP_GLOVE_MODE
	himax_proc_glove_mode_file = proc_create(HIMAX_PROC_GLOVE_MODE_FILE, (S_IWUSR|S_IRUGO|S_IWUGO),
		himax_touch_proc_dir, &himax_proc_glove_mode_ops);
	if(himax_proc_glove_mode_file == NULL)
	{
		E(" %s: proc sensor id file create failed!\n", __func__);
		goto fail_21;
	}

	himax_proc_glove_on_file = proc_create(HIMAX_PROC_GLOVE_ON_FILE, (S_IRUGO),
		himax_touch_proc_dir, &himax_proc_glove_on_ops);
	if(himax_proc_glove_on_file == NULL)
	{
		E(" %s: proc sensor id file create failed!\n", __func__);
		goto fail_22;
	}

	himax_proc_glove_off_file = proc_create(HIMAX_PROC_GLOVE_OFF_FILE, (S_IRUGO),
		himax_touch_proc_dir, &himax_proc_glove_off_ops);
	if(himax_proc_glove_off_file == NULL)
	{
		E(" %s: proc sensor id file create failed!\n", __func__);
		goto fail_23;
	}
#endif

#ifdef HX_SMART_WAKEUP
	himax_proc_SMWP_on_file = proc_create(HIMAX_PROC_SMWP_ON_FILE, (S_IRUGO),
		himax_touch_proc_dir, &himax_proc_SMWP_on_ops);
	if(himax_proc_SMWP_on_file == NULL)
	{
		E(" %s: proc sensor id file create failed!\n", __func__);
		goto fail_24;
	}

	himax_proc_SMWP_off_file = proc_create(HIMAX_PROC_SMWP_OFF_FILE, (S_IRUGO),
		himax_touch_proc_dir, &himax_proc_SMWP_off_ops);
	if(himax_proc_SMWP_off_file == NULL)
	{
		E(" %s: proc sensor id file create failed!\n", __func__);
		goto fail_25;
	}
#endif

#ifdef HX_ESD_WORKAROUND
	himax_proc_ESD_recovery_count_file = proc_create(HIMAX_PROC_ESD_RECOVERY_COUNT_FILE, (S_IRUGO),
		himax_touch_proc_dir, &himax_proc_ESD_recovery_count_ops);
	if(himax_proc_ESD_recovery_count_file == NULL)
	{
		E(" %s: proc ESD_recovery_count file create failed!\n", __func__);
		goto fail_26;
	}

	himax_proc_ESD_off_file = proc_create(HIMAX_PROC_ESD_OFF_FILE, (S_IRUGO),
		himax_touch_proc_dir, &himax_proc_ESD_off_ops);
	if(himax_proc_ESD_off_file == NULL)
	{
		E(" %s: proc ESD_recovery_count file create failed!\n", __func__);
		goto fail_27;
	}

#endif

/* add calling status */
#ifdef PHONE_CALLING_STATUS
	himax_proc_calling_status_file = proc_create(HIMAX_PROC_CALLING_STATUS_FILE, (S_IWUSR|S_IRUGO|S_IWUGO),
		himax_touch_proc_dir, &himax_proc_calling_status_ops);
	if (himax_proc_calling_status_file == NULL) {
		E(" %s: proc calling_status file create failed!\n", __func__);
		goto fail_28;
	}

	himax_proc_phone_call_on_file = proc_create(HIMAX_PROC_PHONE_CALL_ON_FILE, (S_IRUGO),
		himax_touch_proc_dir, &himax_proc_phone_call_on_ops);
	if (himax_proc_phone_call_on_file == NULL) {
		E(" %s: proc phone_call_on file create failed!\n", __func__);
		goto fail_29;
	}

	himax_proc_phone_call_off_file = proc_create(HIMAX_PROC_PHONE_CALL_OFF_FILE, (S_IRUGO),
		himax_touch_proc_dir, &himax_proc_phone_call_off_ops);
	if (himax_proc_phone_call_off_file == NULL) {
		E(" %s: proc phone_call_off file create failed!\n", __func__);
		goto fail_30;
	}
#endif

	return 0 ;

#ifdef PHONE_CALLING_STATUS
	fail_30:
	remove_proc_entry(HIMAX_PROC_PHONE_CALL_OFF_FILE, himax_touch_proc_dir);
	fail_29:
	remove_proc_entry(HIMAX_PROC_PHONE_CALL_ON_FILE, himax_touch_proc_dir);
	fail_28:
	remove_proc_entry(HIMAX_PROC_CALLING_STATUS_FILE, himax_touch_proc_dir);
#endif

#ifdef HX_ESD_WORKAROUND
	fail_27:
	remove_proc_entry( HIMAX_PROC_ESD_OFF_FILE, himax_touch_proc_dir );

	fail_26:
	remove_proc_entry( HIMAX_PROC_ESD_RECOVERY_COUNT_FILE, himax_touch_proc_dir );
#endif

#ifdef HX_SMART_WAKEUP
	fail_25:
	remove_proc_entry( HIMAX_PROC_SMWP_OFF_FILE, himax_touch_proc_dir );
	fail_24:
	remove_proc_entry( HIMAX_PROC_SMWP_ON_FILE, himax_touch_proc_dir );
#endif

#ifdef HX_TP_GLOVE_MODE
	fail_23:
	remove_proc_entry( HIMAX_PROC_GLOVE_OFF_FILE, himax_touch_proc_dir );
	fail_22:
	remove_proc_entry( HIMAX_PROC_GLOVE_ON_FILE, himax_touch_proc_dir );
	fail_21:
	remove_proc_entry( HIMAX_PROC_GLOVE_MODE_FILE, himax_touch_proc_dir );
#endif

#ifdef HX_TP_SENSOR_ID
	fail_20:
	remove_proc_entry( HIMAX_PROC_SENSOR_ID_FILE, himax_touch_proc_dir );
#endif

#ifdef HX_TP_FW_CHECK
	fail_19:
	remove_proc_entry( HIMAX_PROC_FW_CHECK_FILE, himax_touch_proc_dir );
#endif

#ifdef HX_TPID_CHECK
	fail_18:
	remove_proc_entry( HIMAX_PROC_TPID_CHECK_FILE, himax_touch_proc_dir );
#endif

#ifdef HX_TP_FWSTATE_CHECK
	fail_17:
	remove_proc_entry( HIMAX_PROC_INTSTATE_FILE, himax_touch_proc_dir );
	fail_16:
	remove_proc_entry( HIMAX_PROC_FWSTATE_FILE, himax_touch_proc_dir );
#endif

#ifdef HX_TP_FWVERSION
	fail_15_1:
	remove_proc_entry( HIMAX_PROC_FWVERSION_FOR_I_FILE, himax_touch_proc_dir );

	fail_15:
	remove_proc_entry( HIMAX_PROC_FWVERSION_FILE, himax_touch_proc_dir );
#endif

#ifdef HX_SMART_WAKEUP
	fail_14:
#endif
#ifdef HX_DOT_VIEW
	remove_proc_entry( HIMAX_PROC_COVER_FILE, himax_touch_proc_dir );
	fail_13:
#endif
#ifdef HX_TP_PROC_HITOUCH
	remove_proc_entry( HIMAX_PROC_HITOUCH_FILE, himax_touch_proc_dir );
	fail_12:
#endif
#ifdef HX_TP_PROC_SELF_TEST
	remove_proc_entry( HIMAX_PROC_CCI_SELF_TEST_FILE, himax_touch_proc_dir );
	fail_11_1:

	remove_proc_entry( HIMAX_PROC_SELF_TEST_FILE, himax_touch_proc_dir );
	fail_11:
#endif
#ifdef HX_TP_PROC_FLASH_DUMP
	remove_proc_entry( HIMAX_PROC_FLASH_DUMP_FILE, himax_touch_proc_dir );
	fail_10:
#endif
#ifdef HX_TP_PROC_DEBUG
	remove_proc_entry( HIMAX_PROC_DEBUG_FILE, himax_touch_proc_dir );
	fail_9:
#endif
#ifdef HX_TP_PROC_REGISTER
	remove_proc_entry( HIMAX_PROC_REGISTER_FILE, himax_touch_proc_dir );
	fail_8:
#endif
#ifdef HX_TP_PROC_DIAG
	remove_proc_entry( HIMAX_PROC_DIAG_FILE, himax_touch_proc_dir );
	fail_7:
#endif
#ifdef HX_TP_PROC_RESET
	remove_proc_entry( HIMAX_PROC_RESET_FILE, himax_touch_proc_dir );
	fail_6:
#endif
	remove_proc_entry( HIMAX_PROC_LAYOUT_FILE, himax_touch_proc_dir );
	fail_5: remove_proc_entry( HIMAX_PROC_INT_EN_FILE, himax_touch_proc_dir );
	fail_4: remove_proc_entry( HIMAX_PROC_ATTN_FILE, himax_touch_proc_dir );
	fail_3: remove_proc_entry( HIMAX_PROC_VENDOR_FILE, himax_touch_proc_dir );
	fail_2: remove_proc_entry( HIMAX_PROC_DEBUG_LEVEL_FILE, himax_touch_proc_dir );
	fail_1: remove_proc_entry( HIMAX_PROC_TOUCH_FOLDER, NULL );
		return -ENOMEM;
}

static void himax_touch_proc_deinit(void)
{
#ifdef HX_SMART_WAKEUP
		remove_proc_entry( HIMAX_PROC_GESTURE_FILE, himax_touch_proc_dir );
		remove_proc_entry( HIMAX_PROC_SMWP_FILE, himax_touch_proc_dir );
#endif
#ifdef HX_DOT_VIEW
		remove_proc_entry( HIMAX_PROC_COVER_FILE, himax_touch_proc_dir );
#endif
#ifdef HX_TP_PROC_HITOUCH
		remove_proc_entry( HIMAX_PROC_HITOUCH_FILE, himax_touch_proc_dir );
#endif
#ifdef HX_TP_PROC_SELF_TEST
		remove_proc_entry(HIMAX_PROC_SELF_TEST_FILE, himax_touch_proc_dir);
#endif
#ifdef HX_TP_PROC_FLASH_DUMP
		remove_proc_entry(HIMAX_PROC_FLASH_DUMP_FILE, himax_touch_proc_dir);
#endif
#ifdef HX_TP_PROC_DEBUG
		remove_proc_entry( HIMAX_PROC_DEBUG_FILE, himax_touch_proc_dir );
#endif
#ifdef HX_TP_PROC_REGISTER
		remove_proc_entry(HIMAX_PROC_REGISTER_FILE, himax_touch_proc_dir);
#endif
#ifdef HX_TP_PROC_DIAG
		remove_proc_entry(HIMAX_PROC_DIAG_FILE, himax_touch_proc_dir);
#endif
#ifdef HX_TP_PROC_RESET
		remove_proc_entry( HIMAX_PROC_RESET_FILE, himax_touch_proc_dir );
#endif
		remove_proc_entry( HIMAX_PROC_LAYOUT_FILE, himax_touch_proc_dir );
		remove_proc_entry( HIMAX_PROC_INT_EN_FILE, himax_touch_proc_dir );
		remove_proc_entry( HIMAX_PROC_ATTN_FILE, himax_touch_proc_dir );
		remove_proc_entry( HIMAX_PROC_VENDOR_FILE, himax_touch_proc_dir );
		remove_proc_entry( HIMAX_PROC_DEBUG_LEVEL_FILE, himax_touch_proc_dir );
		remove_proc_entry( HIMAX_PROC_TOUCH_FOLDER, NULL );
}

#endif

#ifdef CONFIG_OF
#if defined(HX_LOADIN_CONFIG)||defined(HX_AUTO_UPDATE_CONFIG)
static int himax_parse_config(struct himax_ts_data *ts, struct himax_config *pdata)
{
	struct himax_config *cfg_table;
	struct device_node *node, *pp = NULL;
	struct property *prop;
	uint8_t cnt = 0, i = 0;
	u32 data = 0;
	uint32_t coords[4] = {0};
	int len = 0;
	char str[6]={0};

	node = ts->client->dev.of_node;
	if (node == NULL) {
		E(" %s, can't find device_node", __func__);
		return -ENODEV;
	}

	while ((pp = of_get_next_child(node, pp)))
		cnt++;

	if (!cnt)
		return -ENODEV;

	cfg_table = kzalloc(cnt * (sizeof *cfg_table), GFP_KERNEL);
	if (!cfg_table)
		return -ENOMEM;

	pp = NULL;
	while ((pp = of_get_next_child(node, pp))) {
		if (of_property_read_u32(pp, "default_cfg", &data) == 0)
			cfg_table[i].default_cfg = data;

		if (of_property_read_u32(pp, "sensor_id", &data) == 0)
			cfg_table[i].sensor_id = (data);

		if (of_property_read_u32(pp, "fw_ver_main", &data) == 0)
			cfg_table[i].fw_ver_main = data;

		if (of_property_read_u32(pp, "fw_ver_minor", &data) == 0)
			cfg_table[i].fw_ver_minor = data;

		if (of_property_read_u32_array(pp, "himax,tw-coords", coords, 4) == 0) {
			cfg_table[i].tw_x_min = coords[0], cfg_table[i].tw_x_max = coords[1];	//x
			cfg_table[i].tw_y_min = coords[2], cfg_table[i].tw_y_max = coords[3];	//y
		}

		if (of_property_read_u32_array(pp, "himax,pl-coords", coords, 4) == 0) {
			cfg_table[i].pl_x_min = coords[0], cfg_table[i].pl_x_max = coords[1];	//x
			cfg_table[i].pl_y_min = coords[2], cfg_table[i].pl_y_max = coords[3];	//y
		}

		prop = of_find_property(pp, "c1", &len);
		if ((!prop)||(!len)) {
			strcpy(str,"c1");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c1, prop->value, len);
		prop = of_find_property(pp, "c2", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c2");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c2, prop->value, len);
		prop = of_find_property(pp, "c3", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c3");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c3, prop->value, len);
		prop = of_find_property(pp, "c4", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c4");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c4, prop->value, len);
		prop = of_find_property(pp, "c5", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c5");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c5, prop->value, len);
		prop = of_find_property(pp, "c6", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c6");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c6, prop->value, len);
		prop = of_find_property(pp, "c7", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c7");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c7, prop->value, len);
		prop = of_find_property(pp, "c8", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c8");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c8, prop->value, len);
		prop = of_find_property(pp, "c9", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c9");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c9, prop->value, len);
		prop = of_find_property(pp, "c10", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c10");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c10, prop->value, len);
		prop = of_find_property(pp, "c11", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c11");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c11, prop->value, len);
		prop = of_find_property(pp, "c12", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c12");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c12, prop->value, len);
		prop = of_find_property(pp, "c13", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c13");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c13, prop->value, len);
		prop = of_find_property(pp, "c14", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c14");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c14, prop->value, len);
		prop = of_find_property(pp, "c15", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c15");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c15, prop->value, len);
		prop = of_find_property(pp, "c16", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c16");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c16, prop->value, len);
		prop = of_find_property(pp, "c17", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c17");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c17, prop->value, len);
		prop = of_find_property(pp, "c18", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c18");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c18, prop->value, len);
		prop = of_find_property(pp, "c19", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c19");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c19, prop->value, len);
		prop = of_find_property(pp, "c20", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c20");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c20, prop->value, len);
		prop = of_find_property(pp, "c21", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c21");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c21, prop->value, len);
		prop = of_find_property(pp, "c22", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c22");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c22, prop->value, len);
		prop = of_find_property(pp, "c23", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c23");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c23, prop->value, len);
		prop = of_find_property(pp, "c24", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c24");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c24, prop->value, len);
		prop = of_find_property(pp, "c25", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c25");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c25, prop->value, len);
		prop = of_find_property(pp, "c26", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c26");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c26, prop->value, len);
		prop = of_find_property(pp, "c27", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c27");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c27, prop->value, len);
		prop = of_find_property(pp, "c28", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c28");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c28, prop->value, len);
		prop = of_find_property(pp, "c29", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c29");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c29, prop->value, len);
		prop = of_find_property(pp, "c30", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c30");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c30, prop->value, len);
		prop = of_find_property(pp, "c31", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c31");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c31, prop->value, len);
		prop = of_find_property(pp, "c32", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c32");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c32, prop->value, len);
		prop = of_find_property(pp, "c33", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c33");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c33, prop->value, len);
		prop = of_find_property(pp, "c34", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c34");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c34, prop->value, len);
		prop = of_find_property(pp, "c35", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c35");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c35, prop->value, len);
		prop = of_find_property(pp, "c36", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c36");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c36, prop->value, len);
		prop = of_find_property(pp, "c37", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c37");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c37, prop->value, len);
		prop = of_find_property(pp, "c38", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c38");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c38, prop->value, len);
		prop = of_find_property(pp, "c39", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c39");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c39, prop->value, len);
		prop = of_find_property(pp, "c40", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c40");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c40, prop->value, len);
#if 1
		I(" config version=[%02x]", cfg_table[i].c40[1]);
#endif
		prop = of_find_property(pp, "c41", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c41");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c41, prop->value, len);
		
#if 1
		I(" DT#%d-def_cfg:%d,id:%05x, FW:%x.%x, len:%d,", i,
			cfg_table[i].default_cfg, cfg_table[i].sensor_id,
			cfg_table[i].fw_ver_main, cfg_table[i].fw_ver_minor, cfg_table[i].length);
		//I(" pl=[x_m%02d y_M%04d]\n", cfg_table[i].pl_x_min, cfg_table[i].pl_y_max);
#endif
		i++;
	of_find_property_error:
	if (!prop) {
		D(" %s:Looking up %s property in node %s failed",
			__func__, str, pp->full_name);
		return -ENODEV;
	} else if (!len) {
		D(" %s:Invalid length of configuration data in %s\n",
			__func__, str);
		return -EINVAL;
		}
	}

	i = 0;	//confirm which config we should load
	while ((ts->vendor_fw_ver_H << 8 | ts->vendor_fw_ver_L)<
		(cfg_table[i].fw_ver_main << 8 | cfg_table[i].fw_ver_minor)) {
		i++;
	}
	if(cfg_table[i].default_cfg!=0)
		goto startloadconf;
	while (cfg_table[i].sensor_id > 0 && (cfg_table[i].sensor_id !=  ts->vendor_sensor_id)) {
		I(" id:%#x!=%#x, (i++)",cfg_table[i].sensor_id, ts->vendor_sensor_id);
		i++;
	}
	startloadconf:
	if (i <= cnt) {
		I(" DT-%s cfg idx(%d) in cnt(%d)", __func__, i, cnt);
		pdata->fw_ver_main  = cfg_table[i].fw_ver_main;
		pdata->fw_ver_minor = cfg_table[i].fw_ver_minor;
		pdata->sensor_id      	= cfg_table[i].sensor_id;

		memcpy(pdata->c1, cfg_table[i].c1,sizeof(pdata->c1));
		memcpy(pdata->c2, cfg_table[i].c2,sizeof(pdata->c2));
		memcpy(pdata->c3, cfg_table[i].c3,sizeof(pdata->c3));
		memcpy(pdata->c4, cfg_table[i].c4,sizeof(pdata->c4));
		memcpy(pdata->c5, cfg_table[i].c5,sizeof(pdata->c5));
		memcpy(pdata->c6, cfg_table[i].c6,sizeof(pdata->c6));
		memcpy(pdata->c7, cfg_table[i].c7,sizeof(pdata->c7));
		memcpy(pdata->c8, cfg_table[i].c8,sizeof(pdata->c8));
		memcpy(pdata->c9, cfg_table[i].c9,sizeof(pdata->c9));
		memcpy(pdata->c10, cfg_table[i].c10,sizeof(pdata->c10));
		memcpy(pdata->c11, cfg_table[i].c11,sizeof(pdata->c11));
		memcpy(pdata->c12, cfg_table[i].c12,sizeof(pdata->c12));
		memcpy(pdata->c13, cfg_table[i].c13,sizeof(pdata->c13));
		memcpy(pdata->c14, cfg_table[i].c14,sizeof(pdata->c14));
		memcpy(pdata->c15, cfg_table[i].c15,sizeof(pdata->c15));
		memcpy(pdata->c16, cfg_table[i].c16,sizeof(pdata->c16));
		memcpy(pdata->c17, cfg_table[i].c17,sizeof(pdata->c17));
		memcpy(pdata->c18, cfg_table[i].c18,sizeof(pdata->c18));
		memcpy(pdata->c19, cfg_table[i].c19,sizeof(pdata->c19));
		memcpy(pdata->c20, cfg_table[i].c20,sizeof(pdata->c20));
		memcpy(pdata->c21, cfg_table[i].c21,sizeof(pdata->c21));
		memcpy(pdata->c22, cfg_table[i].c22,sizeof(pdata->c22));
		memcpy(pdata->c23, cfg_table[i].c23,sizeof(pdata->c23));
		memcpy(pdata->c24, cfg_table[i].c24,sizeof(pdata->c24));
		memcpy(pdata->c25, cfg_table[i].c25,sizeof(pdata->c25));
		memcpy(pdata->c26, cfg_table[i].c26,sizeof(pdata->c26));
		memcpy(pdata->c27, cfg_table[i].c27,sizeof(pdata->c27));
		memcpy(pdata->c28, cfg_table[i].c28,sizeof(pdata->c28));
		memcpy(pdata->c29, cfg_table[i].c29,sizeof(pdata->c29));
		memcpy(pdata->c30, cfg_table[i].c30,sizeof(pdata->c30));
		memcpy(pdata->c31, cfg_table[i].c31,sizeof(pdata->c31));
		memcpy(pdata->c32, cfg_table[i].c32,sizeof(pdata->c32));
		memcpy(pdata->c33, cfg_table[i].c33,sizeof(pdata->c33));
		memcpy(pdata->c34, cfg_table[i].c34,sizeof(pdata->c34));
		memcpy(pdata->c35, cfg_table[i].c35,sizeof(pdata->c35));
		memcpy(pdata->c36, cfg_table[i].c36,sizeof(pdata->c36));
		memcpy(pdata->c37, cfg_table[i].c37,sizeof(pdata->c37));
		memcpy(pdata->c38, cfg_table[i].c38,sizeof(pdata->c38));
		memcpy(pdata->c39, cfg_table[i].c39,sizeof(pdata->c39));
		memcpy(pdata->c40, cfg_table[i].c40,sizeof(pdata->c40));
		memcpy(pdata->c41, cfg_table[i].c41,sizeof(pdata->c41));

		ts->tw_x_min = cfg_table[i].tw_x_min, ts->tw_x_max = cfg_table[i].tw_x_max;	//x
		ts->tw_y_min = cfg_table[i].tw_y_min, ts->tw_y_max = cfg_table[i].tw_y_max;	//y

		ts->pl_x_min = cfg_table[i].pl_x_min, ts->pl_x_max = cfg_table[i].pl_x_max;	//x
		ts->pl_y_min = cfg_table[i].pl_y_min, ts->pl_y_max = cfg_table[i].pl_y_max;	//y

		I(" DT#%d-def_cfg:%d,id:%05x, FW:%x.%x, len:%d,", i,
			cfg_table[i].default_cfg, cfg_table[i].sensor_id,
			cfg_table[i].fw_ver_main, cfg_table[i].fw_ver_minor, cfg_table[i].length);
		I(" DT-%s:tw-coords = %d, %d, %d, %d\n", __func__, ts->tw_x_min,
				ts->tw_x_max, ts->tw_y_min, ts->tw_y_max);
		I(" DT-%s:pl-coords = %d, %d, %d, %d\n", __func__, ts->pl_x_min,
				ts->pl_x_max, ts->pl_y_min, ts->pl_y_max);
		I(" config version=[%02x]", pdata->c40[1]);
	} else {
		E(" DT-%s cfg idx(%d) > cnt(%d)", __func__, i, cnt);
		return -EINVAL;
	}
	return 0;
}
#endif

#if 0
static void himax_vk_parser(struct device_node *dt,
				struct himax_i2c_platform_data *pdata)
{
	u32 data = 0;
	uint8_t cnt = 0, i = 0;
	uint32_t coords[4] = {0};
	struct device_node *node, *pp = NULL;
	struct himax_virtual_key *vk;

	node = of_parse_phandle(dt, "virtualkey", 0);
	if (node == NULL) {
		I(" DT-No vk info in DT");
		return;
	} else {
		while ((pp = of_get_next_child(node, pp)))
			cnt++;
		if (!cnt)
			return;

		vk = kzalloc(cnt * (sizeof *vk), GFP_KERNEL);
		pp = NULL;
		while ((pp = of_get_next_child(node, pp))) {
			if (of_property_read_u32(pp, "idx", &data) == 0)
				vk[i].index = data;
			if (of_property_read_u32_array(pp, "range", coords, 4) == 0) {
				vk[i].x_range_min = coords[0], vk[i].x_range_max = coords[1];
				vk[i].y_range_min = coords[2], vk[i].y_range_max = coords[3];
			} else
				I(" range faile");
			i++;
		}
		pdata->virtual_key = vk;
		for (i = 0; i < cnt; i++)
			I(" vk[%d] idx:%d x_min:%d, y_max:%d", i,pdata->virtual_key[i].index,
				pdata->virtual_key[i].x_range_min, pdata->virtual_key[i].y_range_max);
	}
}

static int himax_parse_dt(struct himax_ts_data *ts,
				struct himax_i2c_platform_data *pdata) {
	int rc, coords_size = 0;
	uint32_t coords[4] = {0};
	struct property *prop;
	struct device_node *dt = ts->client->dev.of_node;
	u32 data = 0;

	prop = of_find_property(dt, "himax,panel-coords", NULL);
	if (prop) {
		coords_size = prop->length / sizeof(u32);
		if (coords_size != 4)
			D(" %s:Invalid panel coords size %d", __func__, coords_size);
	}

	if (of_property_read_u32_array(dt, "himax,panel-coords", coords, coords_size) == 0) {
		pdata->abs_x_min = coords[0], pdata->abs_x_max = coords[1];
		pdata->abs_y_min = coords[2], pdata->abs_y_max = coords[3];
		I(" DT-%s:panel-coords = %d, %d, %d, %d\n", __func__, pdata->abs_x_min,
				pdata->abs_x_max, pdata->abs_y_min, pdata->abs_y_max);
	}

	prop = of_find_property(dt, "himax,display-coords", NULL);
	if (prop) {
		coords_size = prop->length / sizeof(u32);
		if (coords_size != 4)
			D(" %s:Invalid display coords size %d", __func__, coords_size);
	}
	rc = of_property_read_u32_array(dt, "himax,display-coords", coords, coords_size);
	if (rc && (rc != -EINVAL)) {
		D(" %s:Fail to read display-coords %d\n", __func__, rc);
		return rc;
	}
	pdata->screenWidth  = coords[1];
	pdata->screenHeight = coords[3];
	I(" DT-%s:display-coords = (%d, %d)", __func__, pdata->screenWidth,
		pdata->screenHeight);

	/*
	pdata->gpio_irq = of_get_named_gpio(dt, "himax,irq-gpio", 0);
	if (!gpio_is_valid(pdata->gpio_irq)) {
		I(" DT:gpio_irq value is not valid\n");
	}

	pdata->gpio_reset = of_get_named_gpio(dt, "himax,rst-gpio", 0);
	if (!gpio_is_valid(pdata->gpio_reset)) {
		I(" DT:gpio_rst value is not valid\n");
	}
	*/

	pdata->gpio_irq = TP_INT_PIN;
 	pdata->gpio_reset = TP_RST_PIN;
 	
	pdata->gpio_3v3_en = of_get_named_gpio(dt, "himax,3v3-gpio", 0);
	if (!gpio_is_valid(pdata->gpio_3v3_en)) {
		I(" DT:gpio_3v3_en value is not valid\n");
	}
	I(" DT:gpio_irq=%d, gpio_rst=%d, gpio_3v3_en=%d", pdata->gpio_irq, pdata->gpio_reset, pdata->gpio_3v3_en);

	if (of_property_read_u32(dt, "report_type", &data) == 0) {
		pdata->protocol_type = data;
		I(" DT:protocol_type=%d", pdata->protocol_type);
	}

#if defined(CONFIG_TOUCHSCREEN_PROXIMITY)
	if (of_property_read_u32(dt, "proximity_bytp_enable", &data) == 0) {
		pdata->proximity_bytp_enable = data;
		I(" DT:proximity_bytp_enable=%d", pdata->proximity_bytp_enable);
	}
#endif
	himax_vk_parser(dt, pdata);

	return 0;
}
#endif
#endif

#ifdef HX_TOUCH_REC
static int check_touchData(void)
{
	if(HX_MAX_PT>10 || HX_MAX_PT < 1)
		return 1;
#ifdef HX_TP_PROC_DIAG
	if(HX_RX_NUM > 40 || HX_RX_NUM<5)
		return 1;
	else if(HX_TX_NUM > 40 || HX_TX_NUM<5)
		return 1;
#endif
#ifdef HX_TP_PROC_2T2R
	if(Is_2T2R){
		if(HX_RX_NUM_2 > 40 || HX_RX_NUM_2<5)
			return 1;
		else if(HX_TX_NUM_2 > 40 || HX_TX_NUM_2<5)
			return 1;
	}
#endif
    return 0;
}
static void TP_REC(void)
{
	himax_power_on_initCMD(private_ts->client);
	//himax_HW_reset(true,false);
	calculate_point_number();
#ifdef HX_TP_PROC_DIAG
	setXChannel(HX_RX_NUM);
	setYChannel(HX_TX_NUM);
	kfree(diag_mutual);
	setMutualBuffer();
#endif
#ifdef HX_TP_PROC_2T2R
	setXChannel_2(HX_RX_NUM_2);
	setYChannel_2(HX_TX_NUM_2);
	kfree(diag_mutual_2);
	setMutualBuffer_2();
#endif
	private_ts->x_channel = HX_RX_NUM;
	private_ts->y_channel = HX_TX_NUM;
	private_ts->nFinger_support = HX_MAX_PT;
	calcDataSize(private_ts->nFinger_support);
}
#endif

static int himax852xes_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = -ENOMEM;

#ifdef HX_AUTO_UPDATE_FW
	int up_flag;	/*CY33, 20160119 for FW_STATE */
#endif

	struct himax_ts_data *ts;
	struct himax_i2c_platform_data *pdata;

	/* Check I2C functionality */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		E("%s: i2c check functionality error\n", __func__);
		err = -ENODEV;
		goto err_check_functionality_failed;
	}

/* Allocate the MTK's DMA memory */
#ifdef MTK_DMA
	client->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	gpDMABuf_va = (u8 *)dma_alloc_coherent(&client->dev, 4096, (dma_addr_t *)&gpDMABuf_pa, GFP_KERNEL);
	if(!gpDMABuf_va) {
		E("Allocate DMA I2C Buffer failed\n");
		goto err_alloc_MTK_DMA_failed;
	}
	memset(gpDMABuf_va, 0, 4096);
#endif

	ts = kzalloc(sizeof(struct himax_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		E("%s: allocate himax_ts_data failed\n", __func__);
		err = -ENOMEM;
		goto err_alloc_data_failed;
	}

	i2c_set_clientdata(client, ts);
	ts->client = client;
	ts->dev = &client->dev;

	/* Allocate Platform data space */
	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (pdata == NULL) {
		err = -ENOMEM;
		goto err_dt_platform_data_fail;
	}

/* DeviceTree Init Platform_data */
/* himax_parse_dt unuse */ 
/*
#ifdef CONFIG_OF
	if (client->dev.of_node) {
		err = himax_parse_dt(ts, pdata);
		if (err < 0) {
			I("%s: pdata is NULL for DT\n", __func__);
			goto err_alloc_dt_pdata_failed;
		}
	}
#else
	pdata = client->dev.platform_data;
	if (pdata == NULL) {
		I("%s: pdata is NULL(dev.platform_data)\n", __func__);
		goto err_get_platform_data_fail;
	}
#endif
*/
	/* replace himax_parse_dt for MTK */
	/* Set device tree data, defined in himax_852xES.h */
	pdata->abs_x_min = hx_panel_coords[0];
	pdata->abs_x_max = hx_panel_coords[1];
	pdata->abs_y_min = hx_panel_coords[2];
	pdata->abs_y_max = hx_panel_coords[3];
	I("%s: panel-coords = %d, %d, %d, %d\n", __func__, pdata->abs_x_min,
			pdata->abs_x_max, pdata->abs_y_min, pdata->abs_y_max);
	
	pdata->screenWidth  = hx_display_coords[1];
	pdata->screenHeight = hx_display_coords[3];
	I("%s: display-coords = (%d, %d)", __func__, pdata->screenWidth,
			pdata->screenHeight);

	/* input Protocol A, defined in himax_852xES.h */
	pdata->protocol_type = report_type;

	/* defined in himax_platform.h */
	pdata->gpio_irq = TP_INT_PIN;
 	pdata->gpio_reset = TP_RST_PIN;

  	hx_i2c_client_point = client;

#ifdef HX_RST_PIN_FUNC
	ts->rst_gpio = pdata->gpio_reset;
#endif

	of_get_himax85xx_platform_data(&client->dev);
	himax_gpio_power_config(ts->client,pdata);

#ifndef CONFIG_OF
	if (pdata->power) {
		err = pdata->power(1);
		if (err < 0) {
			E("%s: power on failed\n", __func__);
			goto err_power_failed;
		}
	}
#endif

	private_ts = ts;

	/* Get Himax IC Type & FW information & Calculate the point number */
	if (himax_ic_package_check(ts) == false) {
		E("%s: Himax chip doesn NOT EXIST", __func__);
		goto err_ic_package_failed;
	}

	if (pdata->virtual_key)
		ts->button = pdata->virtual_key;

#ifdef  HX_TP_PROC_FLASH_DUMP
		ts->flash_wq = create_singlethread_workqueue("himax_flash_wq");
		if (!ts->flash_wq) {
			E("%s: create flash workqueue failed\n", __func__);
			err = -ENOMEM;
			goto err_create_flash_wq_failed;
		}
	
		INIT_WORK(&ts->flash_work, himax_ts_flash_work_func);
	
		setSysOperation(0);
		setFlashBuffer();
#endif

	himax_read_TP_info(client);

#ifdef HX_TPID_CHECK
	CTP_ID_g = __gpio_get_value(TPID_GPIO);
	I("CTP_ID_g = %d\n",CTP_ID_g);
#endif

#ifdef HX_AUTO_UPDATE_FW
	fw_status = 1; /* pdate fw_status 20160119 */

	/* CY33 20160119 reboot every 30s issue */
	mtk_wdt_enable(WK_WDT_DIS);

	up_flag = i_update_FW();

	/* 20160119 reboot every 30s issue */
	mtk_wdt_enable(WK_WDT_EN);

	if (up_flag == 1) {
		I("FW upgrade done\n");
	} else if (up_flag == 0) {
		I("FW no upgrade\n");
	} else {
		E("%s: TP upgrade error\n", __func__);
		goto err_FW_CRC_failed;
	}

#else
	if(himax_calculateChecksum(false) == 0)
		goto err_FW_CRC_failed;
	fw_status = 3;
#endif

	/* Himax Power On and Load Config */
	if (himax_loadSensorConfig(client, pdata) < 0) {
		E("%s: Load Sesnsor configuration failed, unload driver.\n", __func__);
		goto err_detect_failed;
	}	

	calculate_point_number();
#ifdef HX_TP_PROC_DIAG
	setXChannel(HX_RX_NUM); /* X channel */
	setYChannel(HX_TX_NUM); /* Y channel */
	setMutualBuffer();

	if (getMutualBuffer() == NULL) {
		E("%s: mutual buffer allocate fail failed\n", __func__);
		goto err_setchannel_failed;
	}
#ifdef HX_TP_PROC_2T2R
	if(Is_2T2R){
		setXChannel_2(HX_RX_NUM_2); /* X channel */
		setYChannel_2(HX_TX_NUM_2); /* Y channel */
		setMutualBuffer_2();

		if (getMutualBuffer_2() == NULL) {
			E("%s: mutual buffer 2 allocate fail failed\n", __func__);
			goto err_setchannel_failed;
		}
	}
#endif	
#endif

#ifdef CONFIG_OF
	ts->power = pdata->power;
#endif
	ts->pdata = pdata;

	ts->x_channel = HX_RX_NUM;
	ts->y_channel = HX_TX_NUM;
	ts->nFinger_support = HX_MAX_PT;
	/* calculate the i2c data size */
	calcDataSize(ts->nFinger_support);
	I("%s: calcDataSize complete\n", __func__);

	ts->pdata->abs_pressure_min        = 0;
	ts->pdata->abs_pressure_max        = 200;
	ts->pdata->abs_width_min           = 0;
	ts->pdata->abs_width_max           = 200;
	pdata->cable_config[0]             = 0xF0;
	pdata->cable_config[1]             = 0x00;

	ts->suspended                      = false;
	atomic_set(&ts->suspend_mode, 0);/* 20160521 ts->suspend_mode default value */
#if defined(HX_USB_DETECT)||defined(HX_USB_DETECT2)
	ts->usb_connected = 0x00;
	ts->cable_config = pdata->cable_config;
#endif
	ts->protocol_type = pdata->protocol_type;
	I("%s: Use Protocol Type %c\n",
		__func__, ts->protocol_type == PROTOCOL_TYPE_A ? 'A' : 'B');

	err = himax_input_register(ts);
	if (err) {
		E("%s: Unable to register %s input device\n",
			__func__, ts->input_dev->name);
		goto err_input_register_device_failed;
	}
/* 20160506
#ifdef CONFIG_FB
	ts->himax_att_wq = create_singlethread_workqueue("HMX_ATT_reuqest");
	if (!ts->himax_att_wq) {
		E(" allocate HMX_att_wq failed\n");
		err = -ENOMEM;
		goto err_get_intr_bit_failed;
	}
	INIT_DELAYED_WORK(&ts->work_att, himax_fb_register);
	queue_delayed_work(ts->himax_att_wq, &ts->work_att, msecs_to_jiffies(15000));

#elif defined(CONFIG_HAS_EARLYSUSPEND)
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING + 1;
	ts->early_suspend.suspend = himax_ts_early_suspend;
	ts->early_suspend.resume = himax_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif
*/
#ifdef HX_CHIP_STATUS_MONITOR//for ESD solution
	ts->himax_chip_monitor_wq = create_singlethread_workqueue("himax_chip_monitor_wq");
	if (!ts->himax_chip_monitor_wq)
	{
		E(" %s: create workqueue failed\n", __func__);
		err = -ENOMEM;
		goto err_create_chip_monitor_wq_failed;
	}

	INIT_DELAYED_WORK(&ts->himax_chip_monitor, himax_chip_monitor_function);
	queue_delayed_work(ts->himax_chip_monitor_wq, &ts->himax_chip_monitor, HX_POLLING_TIMER*HZ);
#endif

#if defined(CONFIG_TOUCHSCREEN_PROXIMITY)
	if(pdata->proximity_bytp_enable)
	wake_lock_init(&ts->ts_wake_lock, WAKE_LOCK_SUSPEND, HIMAX852xes_NAME);
#endif
#ifdef HX_SMART_WAKEUP
	ts->SMWP_enable = 0;	/* default disable smart wake up */
	ts->gesture_cust_en[0] = 1; /* default enable report Power key */
	wake_lock_init(&ts->ts_SMWP_wake_lock, WAKE_LOCK_SUSPEND, HIMAX852xes_NAME);
#endif

#ifdef HX_TP_GLOVE_MODE
	glove_state = 0x00;	/* default disable glove mode */
#endif

wake_lock_init(&ts->ts_flash_wake_lock, WAKE_LOCK_SUSPEND, HIMAX852xes_NAME);

#if defined(CONFIG_TOUCHSCREEN_HIMAX_DEBUG)
	himax_touch_proc_init();
#endif

#ifdef HX_ESD_WORKAROUND
	ESD_RESET_ACTIVATE = 0;
#endif
HW_RESET_ACTIVATE = 0;


#if defined(HX_USB_DETECT)
	if (ts->cable_config)		
		cable_detect_register_notifier(&himax_cable_status_handler);
#endif
#ifdef HX_DOT_VIEW
	register_notifier_by_hallsensor(&hallsensor_status_handler);
#endif

err = himax_ts_register_interrupt(ts->client);
if (err)
	goto err_register_interrupt_failed;

I("%s: Himax prob ok\n", __func__);

return 0;

err_register_interrupt_failed:
if(ts->irq_enabled){
		himax_int_enable(ts->client->irq,0,true);
		free_irq(ts->client->irq, ts);
	}
if (!ts->use_irq){
		hrtimer_cancel(&ts->timer);
		destroy_workqueue(ts->himax_wq);
	}
#if defined(CONFIG_TOUCHSCREEN_HIMAX_DEBUG)
	himax_touch_proc_deinit();
#endif
wake_lock_destroy(&ts->ts_flash_wake_lock);
#ifdef HX_SMART_WAKEUP
	wake_lock_destroy(&ts->ts_SMWP_wake_lock);
#endif
#if defined(CONFIG_TOUCHSCREEN_PROXIMITY)
	if(pdata->proximity_bytp_enable)
		wake_lock_destroy(&ts->ts_wake_lock);
#endif
#ifdef  HX_CHIP_STATUS_MONITOR
	cancel_delayed_work_sync(&ts->himax_chip_monitor);
	destroy_workqueue(ts->himax_chip_monitor_wq);
err_create_chip_monitor_wq_failed:
#endif
/* 20160506
#ifdef CONFIG_FB
	cancel_delayed_work_sync(&ts->work_att);
	destroy_workqueue(ts->himax_att_wq);
	if (fb_unregister_client(&ts->fb_notif))
			I("Error occurred while unregistering fb_notifier.\n");
err_get_intr_bit_failed:
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&ts->early_suspend);
#endif
*/
	input_free_device(ts->input_dev);

err_input_register_device_failed:
#ifdef HX_TP_PROC_DIAG
err_setchannel_failed:
#endif
err_detect_failed:

err_FW_CRC_failed:

#ifdef  HX_TP_PROC_FLASH_DUMP
	destroy_workqueue(ts->flash_wq);
err_create_flash_wq_failed:
#endif
err_ic_package_failed:

#ifndef CONFIG_OF
err_power_failed:
#endif
err = regulator_disable(tpd->reg);
if (err != 0)
	E("Failed to disable reg-vgp6: %d\n", err);
regulator_put(tpd->reg);
gpio_free(TP_RST_PIN);
gpio_free(TP_INT_PIN);
/*
#ifdef CONFIG_OF

err_alloc_dt_pdata_failed:
#else
err_get_platform_data_fail:
#endif
*/
	kfree(pdata);
err_dt_platform_data_fail:
	kfree(ts);

err_alloc_data_failed:
#ifdef MTK_DMA
	if(gpDMABuf_va)
	{
		dma_free_coherent(&client->dev, 4096, gpDMABuf_va, (dma_addr_t)gpDMABuf_pa);
		gpDMABuf_va = NULL;
		gpDMABuf_pa = NULL;
	}
err_alloc_MTK_DMA_failed:
#endif
err_check_functionality_failed:
	HX_DRIVER_PROBE_Fial=1;
	err =-1;
	return err;

}

static int himax852xes_remove(struct i2c_client *client)
{
	struct himax_ts_data *ts = i2c_get_clientdata(client);

	if(ts->irq_enabled){
		himax_int_enable(ts->client->irq,0,true);
			free_irq(ts->client->irq, ts);
		}
	if (!ts->use_irq){
			hrtimer_cancel(&ts->timer);
			destroy_workqueue(ts->himax_wq);
		}
#if defined(CONFIG_TOUCHSCREEN_HIMAX_DEBUG)
		himax_touch_proc_deinit();
#endif
#ifdef HX_SMART_WAKEUP
		wake_lock_destroy(&ts->ts_SMWP_wake_lock);
#endif
#if defined(CONFIG_TOUCHSCREEN_PROXIMITY)
		if(pdata->proximity_bytp_enable)
			wake_lock_destroy(&ts->ts_wake_lock);
#endif
#ifdef  HX_CHIP_STATUS_MONITOR
		cancel_delayed_work_sync(&ts->himax_chip_monitor);
		destroy_workqueue(ts->himax_chip_monitor_wq);
#endif
/*
#ifdef CONFIG_FB
		cancel_delayed_work_sync(&ts->work_att);
		destroy_workqueue(ts->himax_att_wq);
		if (fb_unregister_client(&ts->fb_notif))
			I("Error occurred while unregistering fb_notifier.\n");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
		unregister_early_suspend(&ts->early_suspend);
#endif
*/
		input_free_device(ts->input_dev);
#ifdef  HX_TP_PROC_FLASH_DUMP
		destroy_workqueue(ts->flash_wq);
#endif
		if (regulator_disable(tpd->reg))
			E("Failed to disable reg-vgp6 \n");
		regulator_put(tpd->reg);
		gpio_free(TP_RST_PIN);
		gpio_free(TP_INT_PIN);
		kfree(ts);
#ifdef MTK_DMA
		if(gpDMABuf_va)
		{
			dma_free_coherent(&client->dev, 4096, gpDMABuf_va, (dma_addr_t)gpDMABuf_pa);
			gpDMABuf_va = NULL;
			gpDMABuf_pa = NULL;
		}
#endif

	return 0;

}

//==== 20160517
int himax_irqdepth(int irqnum)
{
	struct irq_desc *desc=irq_to_desc(irqnum);
	int depth;
	depth=desc->depth;
	//I("[himax] %s: irq_depth=%d\n",__func__,depth);
	return depth;
}
//====
void himax852xes_suspend(struct device *dev)
{
	int ret;
	uint8_t buf[2] = {0};
#ifdef HX_CHIP_STATUS_MONITOR
	int t=0;
#endif
	struct himax_ts_data *ts = dev_get_drvdata(&hx_i2c_client_point->dev);
	if(HX_DRIVER_PROBE_Fial)
	{
		I("%s: Driver probe fail. \n", __func__);
		return;
	}
	I("%s: Enter suspended. \n", __func__);

	if(atomic_read(&ts->suspend_mode))//20160517 add
	{
		I("%s: Already suspended. Skipped. \n", __func__);
		return;
	}
	else
	{
		//=== 20160517
		atomic_set(&ts->suspend_mode, 1);
		//====
		I("%s: enter \n", __func__);
	}

#if defined(CONFIG_TOUCHSCREEN_PROXIMITY)
	if(ts->pdata->proximity_bytp_enable){
			I("[Proximity],Proximity en=%d\r\n",g_proximity_en);
			if(g_proximity_en)
					I("[Proximity],Proximity %s\r\n",proximity_flag? "NEAR":"FAR");

			if((g_proximity_en)&&(proximity_flag)){
				I("[Proximity],Proximity on,and Near won't enter deep sleep now.\n");
				atomic_set(&ts->suspend_mode, 1);
				ts->first_pressed = 0;
				ts->pre_finger_mask = 0;
				return;
			}
		}
#endif
#ifdef HX_TP_PROC_FLASH_DUMP
	if (getFlashDumpGoing())
	{
		I("[himax] %s: Flash dump is going, reject suspend\n",__func__);
		return;
	}
#endif
#ifdef HX_TP_PROC_HITOUCH
	if(hitouch_is_connect)
	{
		I("[himax] %s: Hitouch connect, reject suspend\n",__func__);
		return;
	}
#endif

#ifdef HX_CHIP_STATUS_MONITOR
if(HX_ON_HAND_SHAKING)//chip on hand shaking,wait hand shaking
{
	for(t=0; t<100; t++)
		{
			if(HX_ON_HAND_SHAKING==0)//chip on hand shaking end
				{
					I("%s:HX_ON_HAND_SHAKING OK check %d times\n",__func__,t);
					break;
				}
			else
				msleep(1);
		}
	if(t==100)
		{
			E("%s:HX_ON_HAND_SHAKING timeout reject suspend\n",__func__);
			return;
		}
}
#endif

#ifdef HX_SMART_WAKEUP
	if (ts->SMWP_enable) {
		ts->pre_finger_mask = 0;
		FAKE_POWER_KEY_SEND = false;
		buf[0] = HX_SMART_WAKEUP_REG;
		buf[1] = ENABLE_SMART_WAKEUP_MODE;
		ret = i2c_himax_master_write(ts->client, buf, 2, DEFAULT_RETRY_CNT);
		if (ret < 0) {
			E("[himax] %s: I2C access failed addr = 0x%x\n", __func__, ts->client->addr);
		}
		I("[himax] %s: SMART_WAKEUP enable, reject suspend\n", __func__);

#ifdef HX_CHIP_STATUS_MONITOR
		HX_CHIP_POLLING_COUNT = 0;
		cancel_delayed_work_sync(&ts->himax_chip_monitor);
#endif
		atomic_set(&ts->resume_mode, 0);
		return;
	}
#endif

	himax_int_enable(ts->client->irq,0,true);

#ifdef HX_CHIP_STATUS_MONITOR
	HX_CHIP_POLLING_COUNT = 0;
	cancel_delayed_work_sync(&ts->himax_chip_monitor);
#endif

	//Himax 852xes IC enter sleep mode
	buf[0] = HX_CMD_TSSOFF;
	ret = i2c_himax_master_write(ts->client, buf, 1, DEFAULT_RETRY_CNT);
	if (ret < 0)
	{
		E("[himax] %s: I2C access failed addr = 0x%x\n", __func__, ts->client->addr);
	}
	msleep(40);

	buf[0] = HX_CMD_TSSLPIN;
	ret = i2c_himax_master_write(ts->client, buf, 1, DEFAULT_RETRY_CNT);
	if (ret < 0)
	{
		E("[himax] %s: I2C access failed addr = 0x%x\n", __func__, ts->client->addr);
	}

	if (!ts->use_irq) {
		ret = cancel_work_sync(&ts->work);
		if (ret)
			himax_int_enable(ts->client->irq,1,true);
	}

	atomic_set(&ts->resume_mode, 0);//20160520



	if (ts->pdata->powerOff3V3 && ts->pdata->power)
		ts->pdata->power(0);
	I("%s: suspend end. \n", __func__);
	return;
}

static void himax852xes_resume(struct device *dev)
{
#ifdef HX_SMART_WAKEUP
	int ret;
	uint8_t buf[2] = {0};
#endif
#ifdef HX_CHIP_STATUS_MONITOR
	int t=0;
#endif
#ifdef RESUME_HW_RESET
	uint8_t finger_on;
	int32_t loop_i;
#endif

	struct himax_ts_data *ts = dev_get_drvdata(&hx_i2c_client_point->dev);
	//========= 20160517
	int himax_depth_r;
	//========
	if(HX_DRIVER_PROBE_Fial)
	{
		I("%s: Driver probe fail. \n", __func__);
		return;
	}
	//20160517 add
	I("%s: Enter resumed. \n", __func__);
	if(atomic_read(&ts->resume_mode))//(ts->resumed)
	{
		I("%s: Already resumed. Skipped. \n", __func__);
		return ;
	}
	else
	{
		atomic_set(&ts->resume_mode, 1);
		I("%s: enter \n", __func__);
	}
	//20160517 add
	if (ts->pdata->powerOff3V3 && ts->pdata->power)
		ts->pdata->power(1);
#ifdef HX_CHIP_STATUS_MONITOR
	if(HX_ON_HAND_SHAKING)//chip on hand shaking,wait hand shaking
	{
		for(t=0; t<100; t++)
			{
				if(HX_ON_HAND_SHAKING==0)//chip on hand shaking end
					{
						I("%s:HX_ON_HAND_SHAKING OK check %d times\n",__func__,t);
						break;
					}
				else
					msleep(1);
			}
		if(t==100)
			{
				E("%s:HX_ON_HAND_SHAKING timeout reject resume\n",__func__);
				return;
			}
	}
#endif

#ifdef RESUME_HW_RESET
	HW_RESET_ACTIVATE = 1;
	I("%s: Now reset the Touch chip.\n", __func__);

	tpd_gpio_output(TP_RST_PIN, 0);
	msleep(20);
	tpd_gpio_output(TP_RST_PIN, 1);
	msleep(20);
#endif

#ifdef HX_SMART_WAKEUP
	if (ts->SMWP_enable) {
		/* Sense Off */
		i2c_himax_write_command(ts->client, HX_CMD_TSSOFF, DEFAULT_RETRY_CNT);
		msleep(40);
		/* Sleep in */
		i2c_himax_write_command(ts->client, HX_CMD_TSSLPIN, DEFAULT_RETRY_CNT);
		I("%s: Disable smart wake up for resume\n", __func__);
		buf[0] = HX_SMART_WAKEUP_REG;
		buf[1] = DISABLE_SMART_WAKEUP_MODE;
		ret = i2c_himax_master_write(ts->client, buf, 2, DEFAULT_RETRY_CNT);
		if (ret < 0) {
			E("%s: I2C access failed addr = 0x%x, reg = 0x%02x\n", __func__, ts->client->addr, buf[0]);
		}
		msleep(50);
	}
#endif

#if defined(HX_USB_DETECT2)
		himax_cable_detect_func();
#endif

#ifdef HX_TP_GLOVE_MODE
	/* check glove mode */
	if (glove_state == ENABLE_GLOVE_MODE) {
		I("%s: Enable glove state\n", __func__);
		buf[0] = HX_GLOVE_REG;
		buf[1] = ENABLE_GLOVE_MODE;
		ret = i2c_himax_master_write(ts->client, buf, 2, DEFAULT_RETRY_CNT);
		if (ret < 0) {
			E("%s: I2C access failed addr = 0x%02x, reg = 0x%02x\n", __func__, ts->client->addr, buf[0]);
		}
		msleep(5);
	}
#endif

#ifdef RESUME_HW_RESET
	/* leave event */
	finger_on = 0;
	AA_press = 0;
	if (ts->protocol_type == PROTOCOL_TYPE_A)
		input_mt_sync(ts->input_dev);

	for (loop_i = 0; loop_i < ts->nFinger_support; loop_i++) {
		if (((ts->pre_finger_mask >> loop_i) & 1) == 1) {
			if (ts->protocol_type == PROTOCOL_TYPE_B) {
				input_mt_slot(ts->input_dev, loop_i);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
			}
		}
	}
	if (ts->pre_finger_mask > 0) {
		for (loop_i = 0; loop_i < ts->nFinger_support; loop_i++) {
			if (((ts->pre_finger_mask >> loop_i) & 1) == 1) {

				I("status:0x%02X, Raw:F:%02d Up, X:%d, Y:%d, N:%d\n",
					0, loop_i+1,
					ts->pre_finger_data[loop_i][0],
					ts->pre_finger_data[loop_i][1],
					Last_EN_NoiseFilter);

			}
		}
		ts->pre_finger_mask = 0;
	}

	input_report_key(ts->input_dev, BTN_TOUCH, finger_on);
	input_sync(ts->input_dev);
#endif
	/* Sense On */
	i2c_himax_write_command(ts->client, HX_CMD_TSSON, DEFAULT_RETRY_CNT);
	msleep(30);
	i2c_himax_write_command(ts->client, HX_CMD_TSSLPOUT, DEFAULT_RETRY_CNT);

	/* check irq state */
	himax_depth_r=himax_irqdepth(ts->client->irq);
	I("%s: irq_depth=%d\n", __func__, himax_depth_r);

	if (ts->SMWP_enable == 0) {
		himax_int_enable(ts->client->irq, 1, true);

		himax_depth_r=himax_irqdepth(ts->client->irq);
		I("%s: himax_depth_r_end= %d \n", __func__, himax_depth_r);
	}

#ifdef HX_CHIP_STATUS_MONITOR
	HX_CHIP_POLLING_COUNT = 0;
	queue_delayed_work(ts->himax_chip_monitor_wq, &ts->himax_chip_monitor, HX_POLLING_TIMER*HZ); //for ESD solution
#endif

	atomic_set(&ts->suspend_mode, 0); //20160520

	I(" %s: mt_eint_dump_status start\n", __func__);
	//===20160419 mt_eint_dump_status
	mt_eint_dump_status(10);
	//===
	I(" %s: mt_eint_dump_status end\n", __func__);
	return;
}

/* 20160506
#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct himax_ts_data *ts=
		container_of(self, struct himax_ts_data, fb_notif);

	I(" %s\n", __func__);
	if (evdata && evdata->data && event == FB_EVENT_BLANK && ts &&
			ts->client) {
		blank = evdata->data;
		switch (*blank) {
		case FB_BLANK_UNBLANK:
			himax852xes_resume(&ts->client->dev);
		break;

		case FB_BLANK_POWERDOWN:
		case FB_BLANK_HSYNC_SUSPEND:
		case FB_BLANK_VSYNC_SUSPEND:
		case FB_BLANK_NORMAL:
			himax852xes_suspend(&ts->client->dev);
		break;
		}
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void himax_ts_early_suspend(struct early_suspend *h)
{
	struct himax_ts_data *ts;
	ts = container_of(h, struct himax_ts_data, early_suspend);
	himax852xes_suspend(h);
}

static void himax_ts_late_resume(struct early_suspend *h)
{
	struct himax_ts_data *ts;
	ts = container_of(h, struct himax_ts_data, early_suspend);
	himax852xes_resume(h);
}
#endif
*/
static const struct i2c_device_id himax852xes_ts_id[] = {
	{HIMAX852xes_NAME, 0},
	{}
};

/*
static struct i2c_board_info __initdata himax852xes_i2c = { I2C_BOARD_INFO(HIMAX852xes_NAME, (0x90 >> 1))};
*/

static struct i2c_driver tpd_i2c_driver = {
    .probe = himax852xes_probe,
    .remove = himax852xes_remove,
    .detect = himax852xes_detect,
    .driver.name = HIMAX852xes_NAME,
    .driver	= {
		.name = HIMAX852xes_NAME,		
		.of_match_table = of_match_ptr(himax_match_table),
		},
    .id_table = himax852xes_ts_id,
    .address_list = (const unsigned short *) forces,
};   

static int himax852xes_local_init(void)
{
	int retval=0;
	I("[Himax] Himax_ts I2C Touchscreen Driver local init\n");
        
	/* regulator power on ,  vtouch define in :
	drivers/misc/mediatek/dws/mt6757/k57pv1_4mtee_pre.dws */
	tpd->reg = regulator_get(tpd->tpd_dev, "vtouch");

	if(IS_ERR(tpd->reg)) {
		retval= PTR_ERR(tpd->reg);
		I("tpd regulator_get_voltage(%d) failed!\n", retval);
        }
	usleep_range(5000, 5500);

	retval = regulator_set_voltage(tpd->reg, 3000000, 3000000);
	if (retval != 0) {
		I("Failed to set reg-VLDO28 voltage: %d\n", retval);
		return -1;
	}

	if (i2c_add_driver(&tpd_i2c_driver) != 0) {
		I("unable to add i2c driver.\n");        
		return -1;
	}

	I("end %s, %d\n", __FUNCTION__, __LINE__);
	tpd_type_cap = 1;

	return 0;
}

static int himax852xes_detect (struct i2c_client *client, struct i2c_board_info *info) 
{
	strcpy(info->type, TPD_DEVICE);
	return 0;
}

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = HIMAX852xes_NAME,
	.tpd_local_init = himax852xes_local_init,
	.suspend = himax852xes_suspend,
	.resume = himax852xes_resume,
#ifdef TPD_HAVE_BUTTON
	.tpd_have_button = 1,
#else
	.tpd_have_button = 0,
#endif
};

static int __init himax852xes_init(void)
{
	I("Himax 852xES touch panel driver init\n");

	tpd_get_dts_info();
	if (tpd_driver_add(&tpd_device_driver) < 0)
		I("add hX852Xes driver failed\n");
	return 0;
}

static void __exit himax852xes_exit(void)
{
	tpd_driver_remove(&tpd_device_driver);
}

module_init(himax852xes_init);
module_exit(himax852xes_exit);

MODULE_DESCRIPTION("Himax852xes driver");
MODULE_LICENSE("GPL");
