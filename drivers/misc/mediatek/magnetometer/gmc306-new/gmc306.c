/* GME60X/GMC303 compass sensor driver
 *
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

#include <linux/kernel.h>

#include "cust_mag.h"
#include "gmc306.h"
#include "mag.h"

#define DEBUG 1  //jonny change from 0 to 1
#define GME60X_DEV_NAME         "gme60x"
#define DRIVER_VERSION          "1.0.0"
#define GME60X_DEBUG	1
#define GME60X_RETRY_COUNT	10
#define GME60X_DEFAULT_DELAY	100

#if GME60X_DEBUG
#define MAGN_TAG		 "[Msensor] "
#define MAGN_ERR(fmt, args...)	pr_err(MAGN_TAG fmt, ##args)
//#define MAGN_LOG(fmt, args...)	pr_debug(MAGN_TAG fmt, ##args)
//#define MAGN_LOG(fmt, args...)	printk(MAGN_TAG fmt, ##args) //jonny test
#define MAGN_LOG(fmt, args...)	printk(KERN_INFO MAGN_TAG fmt, ##args)
#else
#define MAGN_TAG
#define MAGN_ERR(fmt, args...)	do {} while (0)
#define MAGN_LOG(fmt, args...)	do {} while (0)
#endif
#define FST_TAG                  "[FST] "

/* Addresses to scan -- protected by sense_data_mutex */
static char sense_data[SENSOR_DATA_SIZE];
static struct mutex sense_data_mutex;
/* calibration msensor and orientation data */
static int sensor_data[CALIBRATION_DATA_SIZE];
static struct mutex sensor_data_mutex;
/* static DECLARE_WAIT_QUEUE_HEAD(data_ready_wq); */
static DECLARE_WAIT_QUEUE_HEAD(open_wq);

static short gmad_delay = GME60X_DEFAULT_DELAY;

static atomic_t open_flag = ATOMIC_INIT(0);
static atomic_t m_flag = ATOMIC_INIT(0);
static atomic_t o_flag = ATOMIC_INIT(0);

static int factory_mode;
static int mEnabled;
static int ecompass_status;
static struct i2c_client *this_client;

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id gme60x_i2c_id[] = {{GME60X_DEV_NAME, 0}, {} };

/* Maintain  cust info here */
struct mag_hw mag_cust_1;
static struct mag_hw *hw = &mag_cust_1;

/* For  driver get cust info */
struct mag_hw *get_cust_mag_gmc306(void)
{
	return &mag_cust_1;
}

/*----------------------------------------------------------------------------*/
static int gme60x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int gme60x_i2c_remove(struct i2c_client *client);
static int gme60x_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
//static int gme60x_suspend(struct i2c_client *client, pm_message_t msg);
//static int gme60x_resume(struct i2c_client *client);
static int gme60x_suspend(struct device *dev);
static int gme60x_resume(struct device *dev);
static int gme60x_local_init(void);
static int gme60x_remove(void);

static struct mag_init_info gme60x_init_info = {
	.name = GME60X_DEV_NAME,
	.init = gme60x_local_init,
	.uninit = gme60x_remove,
};


/*----------------------------------------------------------------------------*/
enum {
	GME_FUN_DEBUG  = 0x01,
	GME_DATA_DEBUG = 0X02,
	GME_HWM_DEBUG  = 0X04,
	GME_CTR_DEBUG  = 0X08,
	GME_I2C_DEBUG  = 0x10,
} GME_TRC_1;


/*----------------------------------------------------------------------------*/
struct gme60x_i2c_data {
	struct i2c_client *client;
	struct mag_hw *hw;
	atomic_t layout;
	atomic_t trace;
	struct hwmsen_convert   cvt;
};
/*----------------------------------------------------------------------------*/

static const struct dev_pm_ops gme60x_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(gme60x_suspend, gme60x_resume)
};

#ifdef CONFIG_OF
static const struct of_device_id mag_of_match[] = {
	{.compatible = "mediatek,msensor"},
	//{.compatible = "mediatek,msensor_second"},
	{},
};
#endif
static struct i2c_driver gma60x_i2c_driver = {
		.driver = {
		.name  = GME60X_DEV_NAME,
#ifdef CONFIG_OF
		.of_match_table = mag_of_match,
#endif
		.pm = &gme60x_pm_ops,
	},
	.probe	 = gme60x_i2c_probe,
	.remove	= gme60x_i2c_remove,
	.detect	= gme60x_i2c_detect,
//	.suspend	= gme60x_suspend,
//	.resume	= gme60x_resume,
	.id_table = gme60x_i2c_id,
};


/*----------------------------------------------------------------------------*/
static atomic_t dev_open_count;
/*----------------------------------------------------------------------------*/

static DEFINE_MUTEX(gme60x_i2c_mutex);

#ifndef CONFIG_MTK_I2C_EXTENSION
static int mag_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	int err = 0;
	u8 beg = addr;
	struct i2c_msg msgs[2] = { {0}, {0} };

	mutex_lock(&gme60x_i2c_mutex);
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &beg;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = data;

	if (!client) {
		mutex_unlock(&gme60x_i2c_mutex);
		return -EINVAL;
	} else if (len > C_I2C_FIFO_SIZE) {
		mutex_unlock(&gme60x_i2c_mutex);
		MAGN_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}

	//err = i2c_transfer(client->adapter, msgs, sizeof(msgs) / sizeof(msgs[0]));
	err = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs) );
	if (err != 2) {
		MAGN_ERR("i2c_transfer error: (%d %p %d) %d\n", addr, data, len, err);
		err = -EIO;
	} else {
		err = 0;
	}
	mutex_unlock(&gme60x_i2c_mutex);
	return err;

}

static int mag_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{				/*because address also occupies one byte, the maximum length for write is 7 bytes */
	int err = 0, idx = 0, num = 0;
	char buf[C_I2C_FIFO_SIZE];

	err = 0;
	mutex_lock(&gme60x_i2c_mutex);
	if (!client) {
		mutex_unlock(&gme60x_i2c_mutex);
		return -EINVAL;
	} else if (len >= C_I2C_FIFO_SIZE) {
		mutex_unlock(&gme60x_i2c_mutex);
		MAGN_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}

	num = 0;
	buf[num++] = addr;
	for (idx = 0; idx < len; idx++)
		buf[num++] = data[idx];

	err = i2c_master_send(client, buf, num);
	if (err < 0) {
		mutex_unlock(&gme60x_i2c_mutex);
		MAGN_ERR("send command error!!\n");
		return -EFAULT;
	}
	mutex_unlock(&gme60x_i2c_mutex);
	return err;
}
#endif

static void gme60x_power(struct mag_hw *hw, unsigned int on)
{
}
static long AKI2C_RxData(char *rxData, int length)
{
#ifndef CONFIG_MTK_I2C_EXTENSION
	struct i2c_client *client = this_client;
	int res = 0;
	char addr = rxData[0];

	if ((rxData == NULL) || (length < 1))
		return -EINVAL;
	res = mag_i2c_read_block(client, addr, rxData, length);
	if (res < 0)
		return -1;
	return 0;
#else
	uint8_t loop_i;

#if DEBUG
	int i;
	struct i2c_client *client = this_client;
	struct gme60x_i2c_data *data = i2c_get_clientdata(client);
	char addr = rxData[0];
#endif

	/* Caller should check parameter validity.*/
	if ((rxData == NULL) || (length < 1))
		return -EINVAL;

	mutex_lock(&gme60x_i2c_mutex);
	for (loop_i = 0; loop_i < GME60X_RETRY_COUNT; loop_i++) {
//#ifdef CONFIG_MTK_I2C_EXTENSION  //GME modify
		this_client->addr = this_client->addr & I2C_MASK_FLAG;
		this_client->addr = this_client->addr | I2C_WR_FLAG;
//#endif
		//if (i2c_master_recv(this_client, (char *)rxData, length))
		if(i2c_master_send(this_client, (const char*)rxData, ((length<<0X08) | 0X01)))
			break;
		mdelay(10);
	}

	if (loop_i >= GME60X_RETRY_COUNT) {
		mutex_unlock(&gme60x_i2c_mutex);
		MAG_ERR("%s retry over %d\n", __func__, GME60X_RETRY_COUNT);
		return -EIO;
	}
	mutex_unlock(&gme60x_i2c_mutex);
#if DEBUG
	if (atomic_read(&data->trace) & GME_I2C_DEBUG) {
		MAGN_LOG("RxData: len=%02x, addr=%02x\n  data=", length, addr);
	for (i = 0; i < length; i++)
		MAGN_LOG(" %02x", rxData[i]);

	MAGN_LOG("\n");
	}
#endif

	return 0;
#endif
}

static long AKI2C_TxData(char *txData, int length)
{
#ifndef CONFIG_MTK_I2C_EXTENSION
	struct i2c_client *client = this_client;
	int res = 0;
	char addr = txData[0];
	u8 *buff = &txData[1];

	if ((txData == NULL) || (length < 2))
		return -EINVAL;
	res = mag_i2c_write_block(client, addr, buff, (length - 1));
	if (res < 0)
		return -1;
	return 0;
#else
	uint8_t loop_i;

#if DEBUG
	int i;
	struct i2c_client *client = this_client;
	struct gme60x_i2c_data *data = i2c_get_clientdata(client);
#endif

	/* Caller should check parameter validity.*/
	if ((txData == NULL) || (length < 2))
		return -EINVAL;
	mutex_lock(&gme60x_i2c_mutex);
//#ifdef CONFIG_MTK_I2C_EXTENSION
	this_client->addr = this_client->addr & I2C_MASK_FLAG;
//#endif
	for (loop_i = 0; loop_i < GME60X_RETRY_COUNT; loop_i++) {
		if (i2c_master_send(this_client, (const char *)txData, length) > 0)
			break;
		mdelay(10);
	}
	
	if(loop_i >= GME60X_RETRY_COUNT){
	    mutex_unlock(&gme60x_i2c_mutex);
		MAG_ERR( "%s retry over %d\n", __func__, GME60X_RETRY_COUNT);
		return -EIO;
	}
	mutex_unlock(&gme60x_i2c_mutex);
#if DEBUG
	if(atomic_read(&data->trace) & GME_I2C_DEBUG){
		MAGN_LOG( "TxData: len=%02x, addr=%02x\n  data=", length, txData[0]);
		for(i = 0; i < (length-1); i++)
			MAGN_LOG( " %02x", txData[i + 1]);

		MAGN_LOG( "\n");
	}
#endif

	return 0;
#endif
}

#if 0
static long AKECS_SetMode_SngMeasure(void)
{
	char buffer[2];
	/* Set measure mode */
	buffer[0] = GME_REG_MODE;
	buffer[1] = GME_MODE_SNG_MEASURE; 
	
	/* Set data */
	return AKI2C_TxData(buffer, 2);
}

static long AKECS_SetMode_SelfTest(void)
{
	char buffer[2];
	/* Set measure mode */
	buffer[0] = GME_REG_MODE;
	buffer[1] = GME_MODE_SELF_TEST;  
	/* Set data */
	return AKI2C_TxData(buffer, 2);
}
static long AKECS_SetMode_FUSEAccess(void)
{
	char buffer[2];
	/* Set measure mode */
	buffer[0] = GME_REG_MODE;
	buffer[1] = GME_MODE_FUSE_ACCESS;
	/* Set data */
	return AKI2C_TxData(buffer, 2);
}
static int AKECS_SetMode_PowerDown(void)
{
	char buffer[2];	
	/* Set powerdown mode */
	buffer[0] = GME_REG_MODE;
	buffer[1] = GME_MODE_POWERDOWN;
	/* Set data */
	return AKI2C_TxData(buffer, 2);
}
static long AKECS_SetMode(char mode)
{
	long ret;
	
	switch (mode & 0x1F){
			case GME_MODE_SNG_MEASURE:
			ret = AKECS_SetMode_SngMeasure();
			break;

			case GME_MODE_SELF_TEST:
			ret = AKECS_SetMode_SelfTest();
			break;

			case GME_MODE_FUSE_ACCESS:
			ret = AKECS_SetMode_FUSEAccess();
			break;

			case GME_MODE_POWERDOWN:
			ret = AKECS_SetMode_PowerDown();
			break;

			default:
			MAGN_LOG("%s: Unknown mode(%d)", __func__, mode);
			return -EINVAL;
	}

	// wait at least 100us after changing mode 
	udelay(100);

	return ret;
}
#endif
static long AKECS_Reset(int hard)
{
	unsigned char buffer[2];
	long err = 0;

	if (hard != 0) {
		/*TODO change to board setting*/
	/* gpio_set_value(akm->rstn, 0); */
		udelay(5);
	/* gpio_set_value(akm->rstn, 1); */
	} else {
		/* Set measure mode */
		buffer[0] = GME_REG_RESET;
		buffer[1] = GMC306_RESET_COMMAND;
		err = AKI2C_TxData(buffer, 2);
		if (err < 0) 
			MAGN_LOG("%s: Can not set SRST bit.", __func__);
		else 
			MAGN_LOG("Soft reset is done.");
	}

	/* Device will be accessible 300 us after */
	udelay(300); /* 100 */

	return err;
}
static long AKECS_SetMode(char mode)
{	
	long ret;
	char buffer[2];	/* Set measure mode */
	buffer[0] = GME_REG_MODE;/*checked*/
	buffer[1] = mode; 		/* Set data */	
	ret = AKI2C_TxData(buffer, 2);	
	/* wait at least 100us after changing mode */	
	udelay(100);	
	return ret;
}



//jonny GME test m s
#if 0
static int GME_CheckDevice(void)
{
	char buffer[2];
	int ret;
	MAGN_LOG(" GLOBALMEMS check device id\n");
	/* Set measure mode */
	buffer[0] = GME60X_REG_WIA1;
	/* Read data */
	ret = AKI2C_RxData(buffer, 1);
	MAGN_LOG(" GLOBALMEMS check device id = %x \n",buffer[0]);
	MAGN_LOG(" ret = %d\n",ret);
	if(ret < 0)
		return ret;

	/* Check read data */
	if(buffer[0] != 0x48)
		return -ENXIO;

	return 0;
}
#else
static int GME_CheckDevice(void)
{
	u8 databuf[10];    
	struct i2c_client *client = this_client;
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);    
	databuf[0] = GME_REG_CMPID; //  0x00

	res = i2c_master_send(client, databuf, 0x1);
	if(res <= 0)
	{
		goto exit_GME_CheckDeviceID;
	}
	
	mdelay(100);

	databuf[0] = 0x0;        
	res = i2c_master_recv(client, databuf, 0x01);
	if(res <= 0)
	{
		goto exit_GME_CheckDeviceID;
	}
	

	if(databuf[0] != GME_CMPID_VALUE)
	{
		res = -1;
		MAGN_ERR("GME_CheckDeviceID 0x%02x fail!\n ", databuf[0]);
		return res;
	}
	else
	{
		MAGN_LOG("GME_CheckDeviceID 0x%02x pass!\n ", databuf[0]);
	}

#if 1
	exit_GME_CheckDeviceID:
	if (res <= 0)
	{
		MAGN_ERR("GME_CheckDeviceID I2C failt!\n ");
		return  res;
	}
#endif

	return res;

}
#endif
//jonny GME test m e
/* Daemon application save the data */
static void AKECS_SaveData(int *buf)
{
#if DEBUG
	struct i2c_client *client = this_client;
	struct gme60x_i2c_data *data = i2c_get_clientdata(client);
#endif

	mutex_lock(&sensor_data_mutex);
	memcpy(sensor_data, buf, sizeof(sensor_data));
	mutex_unlock(&sensor_data_mutex);

#if DEBUG
	if (atomic_read(&data->trace) & GME_HWM_DEBUG) {
		MAGN_LOG("Get daemon data[0-11]: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
		sensor_data[0], sensor_data[1], sensor_data[2], sensor_data[3],
		sensor_data[4], sensor_data[5], sensor_data[6], sensor_data[7],
		sensor_data[8], sensor_data[9], sensor_data[10], sensor_data[11]);

		MAGN_LOG("Get daemon data[12-25]: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d!\n",
		sensor_data[12], sensor_data[13], sensor_data[14], sensor_data[15],
		sensor_data[16], sensor_data[17], sensor_data[18], sensor_data[19],
		sensor_data[20], sensor_data[21], sensor_data[22], sensor_data[23],
		sensor_data[24], sensor_data[25]);
	}
#endif

}

/* M-sensor daemon application have set the sng mode */
static long AKECS_GetData(char *rbuf, int size)
{
	char temp;
	int loop_i, ret;
#if DEBUG
	struct i2c_client *client = this_client;
	struct gme60x_i2c_data *data = i2c_get_clientdata(client);
#endif

	if (size < SENSOR_DATA_SIZE) {
		MAG_ERR("buff size is too small %d!\n", size);
		return -1;
	}

	memset(rbuf, 0, SENSOR_DATA_SIZE);
	rbuf[0] = GME_REG_STATUS;

	for (loop_i = 0; loop_i < GME60X_RETRY_COUNT; loop_i++) {
		ret = AKI2C_RxData(rbuf, 1);
		MAG_ERR("retry count =%d!\n",loop_i);
		if (ret) {
			MAG_ERR("read ST1 resigster failed!\n");
//			return -1;
	}

	if ((rbuf[0] & 0x01) == 0x01)
		break;

	mdelay(2);
	rbuf[0] = GME_REG_STATUS;
	}

	if (loop_i >= GME60X_RETRY_COUNT) {
		MAG_ERR("Data read retry larger the max count!\n");
		if (factory_mode == 0)
			/* if return we can not get data at factory mode */
			return -1;
	}

	temp = rbuf[0];
	rbuf[1] = GME_REG_RAW_DATA_START;
	ret = AKI2C_RxData(&rbuf[1], SENSOR_DATA_SIZE - 1);

	if(ret < 0){
		MAG_ERR( "GME60X work_func: I2C failed\n");
		return -1;
	}
	rbuf[0] = temp;

	mutex_lock(&sense_data_mutex);
	memcpy(sense_data, rbuf, sizeof(sense_data));
	mutex_unlock(&sense_data_mutex);

#if DEBUG
	if (atomic_read(&data->trace) & GME_DATA_DEBUG) {
		MAGN_LOG("Get device data: %d, %d, %d, %d, %d, %d, %d, %d!\n",
		sense_data[0], sense_data[1], sense_data[2], sense_data[3],
		sense_data[4], sense_data[5], sense_data[6], sense_data[7]);
	}
#endif

	return 0;
}

/* Get Msensor Raw data */
static int AKECS_GetRawData(char *rbuf, int size)
{
	char strbuf[SENSOR_DATA_SIZE];
	s16 data[3];

	if ((atomic_read(&open_flag) == 0) || (factory_mode == 1)) {
		AKECS_SetMode(GME_MODE_SNG_MEASURE);
		mdelay(10);
	}

	AKECS_GetData(strbuf, SENSOR_DATA_SIZE);
	data[0] = (s16)(strbuf[1] | (strbuf[2] << 8));
	data[1] = (s16)(strbuf[3] | (strbuf[4] << 8));
	data[2] = (s16)(strbuf[5] | (strbuf[6] << 8));

	sprintf(rbuf, "%x %x %x", data[0], data[1], data[2]);

	return 0;

}



static int AKECS_GetOpenStatus(void)
{
	wait_event_interruptible(open_wq, (atomic_read(&open_flag) != 0));
	return atomic_read(&open_flag);
}

static int AKECS_GetCloseStatus(void)
{
	wait_event_interruptible(open_wq, (atomic_read(&open_flag) <= 0));
	return atomic_read(&open_flag);
}

/*----------------------------------------------------------------------------*/
static int gme60x_ReadChipInfo(char *buf, int bufsize)
{
	if((!buf)||(bufsize <= GME60X_BUFSIZE -1))
		return -1;

	if (!this_client) {
		*buf = 0;
		return -2;
	}

	sprintf(buf, GME_ReadChipInfo);
	return 0;
}

/*----------------------------shipment test------------------------------------------------*/
/*!
 * @return If @a testdata is in the range of between @a lolimit and @a hilimit,
 * the return value is 1, otherwise -1.
 * @param[in] testno   A pointer to a text string.
 * @param[in] testname A pointer to a text string.
 * @param[in] testdata A data to be tested.
 * @param[in] lolimit  The maximum allowable value of @a testdata.
 * @param[in] hilimit  The minimum allowable value of @a testdata.
 * @param[in,out] pf_total
 */
static int TEST_DATA(const char testno[], const char testname[], const int testdata,
	const int lolimit, const int hilimit, int *pf_total)
{
	int pf;			/* Pass;1, Fail;-1 */

	if ((testno == NULL) && (strncmp(testname, "START", 5) == 0)) {
		printk(FST_TAG "--------------------------------------------------------------------\n");
		printk(FST_TAG " Test No. Test Name	Fail	Test Data	[	 Low	High]\n");
		printk(FST_TAG "--------------------------------------------------------------------\n");
		pf = 1;
	} else if ((testno == NULL) && (strncmp(testname, "END", 3) == 0)) {
		MAGN_LOG("--------------------------------------------------------------------\n");
		if (*pf_total == 1)
			printk(FST_TAG "Factory shipment test was passed.\n\n");
		else
			printk(FST_TAG "Factory shipment test was failed.\n\n");

		pf = 1;
	} else {
		if ((lolimit <= testdata) && (testdata <= hilimit))
			pf = 1;
		else
			pf = -1;

	/* display result */
	printk(FST_TAG " %7s  %-10s	 %c	%9d	[%9d	%9d]\n",
		testno, testname, ((pf == 1) ? ('.') : ('F')), testdata,
		lolimit, hilimit);
	}

	/* Pass/Fail check */
	if (*pf_total != 0) {
		if ((*pf_total == 1) && (pf == 1))
			*pf_total = 1;		/* Pass */
		else
			*pf_total = -1;		/* Fail */
	}
	return pf;
}

/*!
 * Execute "Onboard Function Test" (NOT includes "START" and "END" command).
 * @retval 1 The test is passed successfully.
 * @retval -1 The test is failed.
 * @retval 0 The test is aborted by kind of system error.
 */
static int FST_GME60X(void)
{
	int   pf_total;  /* p/f flag for this subtest */
	char	i2cData[16];
	int   hdata[3];
	int   asax;
	int   asay;
	int   asaz;

	/* *********************************************** */
	/* Reset Test Result */
	/* *********************************************** */
	pf_total = 1;

	/* *********************************************** */
	/* Step1 */
	/* *********************************************** */

	/* Reset device. */
	if (AKECS_Reset(0) < 0) {
		printk(FST_TAG "%s:%d Error.\n", __func__, __LINE__);
		return 0;
	}

	/* Read values from WIA. */
	i2cData[0] = GME_REG_CMPID;
	if (AKI2C_RxData(i2cData, 2) < 0) {
		printk(FST_TAG "%s:%d Error.\n", __func__, __LINE__);
		return 0;
	}

	/* TEST */
	TEST_DATA(TLIMIT_NO_RST_WIA1_60X,   TLIMIT_TN_RST_WIA1_60X,   (int)i2cData[0],
		TLIMIT_LO_RST_WIA1_60X,   TLIMIT_HI_RST_WIA1_60X,   &pf_total);
	TEST_DATA(TLIMIT_NO_RST_WIA2_60X,   TLIMIT_TN_RST_WIA2_60X,   (int)i2cData[1],
		TLIMIT_LO_RST_WIA2_60X,   TLIMIT_HI_RST_WIA2_60X,   &pf_total);

	/* Set to FUSE ROM access mode */
	if (AKECS_SetMode(GME_MODE_FUSE_ACCESS) < 0) {
		printk(FST_TAG "%s:%d Error.\n", __func__, __LINE__);
		return 0;
	}

	/* Read values from ASAX to ASAZ */
	i2cData[0] = GME_FUSE_ASAX;
	if (AKI2C_RxData(i2cData, 3) < 0) {
		printk(FST_TAG "%s:%d Error.\n", __func__, __LINE__);
		return 0;
	}
	asax = (int)i2cData[0];
	asay = (int)i2cData[1];
	asaz = (int)i2cData[2];

	/* TEST */
	TEST_DATA(TLIMIT_NO_ASAX_60X, TLIMIT_TN_ASAX_60X, asax, TLIMIT_LO_ASAX_60X,
		TLIMIT_HI_ASAX_60X, &pf_total);
	TEST_DATA(TLIMIT_NO_ASAY_60X, TLIMIT_TN_ASAY_60X, asay, TLIMIT_LO_ASAY_60X,
		TLIMIT_HI_ASAY_60X, &pf_total);
	TEST_DATA(TLIMIT_NO_ASAZ_60X, TLIMIT_TN_ASAZ_60X, asaz, TLIMIT_LO_ASAZ_60X,
		TLIMIT_HI_ASAZ_60X, &pf_total);

	/* Set to PowerDown mode */
	if (AKECS_SetMode(GME_MODE_POWERDOWN) < 0) {
		printk(FST_TAG "%s:%d Error.\n", __func__, __LINE__);
		return 0;
	}

	/* *********************************************** */
	/* Step2 */
	/* *********************************************** */

	/* Set to SNG measurement pattern (Set CNTL register) */
	if (AKECS_SetMode(GME_MODE_SNG_MEASURE) < 0) {
		printk(FST_TAG "%s:%d Error.\n", __func__, __LINE__);
		return 0;
	}

	/* Wait for DRDY pin changes to HIGH. */
	/* usleep(GME_MEASURE_TIME_US); */
	/* Get measurement data from GME60X */
	/* ST1 + (HXL + HXH) + (HYL + HYH) + (HZL + HZH) + TEMP + ST2 */
	/* = 1 + (1 + 1) + (1 + 1) + (1 + 1) + 1 + 1 = 9yte */
	/* if (AKD_GetMagneticData(i2cData) != AKD_SUCCESS) { */
	if (AKECS_GetData(i2cData, SENSOR_DATA_SIZE) < 0) {
		printk(FST_TAG "%s:%d Error.\n", __func__, __LINE__);
		return 0;
	}

	/* hdata[0] = (int)((((uint)(i2cData[2]))<<8)+(uint)(i2cData[1])); */
	/* hdata[1] = (int)((((uint)(i2cData[4]))<<8)+(uint)(i2cData[3])); */
	/* hdata[2] = (int)((((uint)(i2cData[6]))<<8)+(uint)(i2cData[5])); */

	hdata[0] = (s16)(i2cData[2] | (i2cData[1] << 8));
	hdata[1] = (s16)(i2cData[4] | (i2cData[3] << 8));
	hdata[2] = (s16)(i2cData[6] | (i2cData[5] << 8));

	/* TEST */
	i2cData[0] &= 0x7F;
	TEST_DATA(TLIMIT_NO_SNG_ST1_60X,  TLIMIT_TN_SNG_ST1_60X,  (int)i2cData[0], TLIMIT_LO_SNG_ST1_60X,
		TLIMIT_HI_SNG_ST1_60X,  &pf_total);

	/* TEST */
	TEST_DATA(TLIMIT_NO_SNG_HX_60X,   TLIMIT_TN_SNG_HX_60X,   hdata[0],	 TLIMIT_LO_SNG_HX_60X,
		TLIMIT_HI_SNG_HX_60X,   &pf_total);
	TEST_DATA(TLIMIT_NO_SNG_HY_60X,   TLIMIT_TN_SNG_HY_60X,   hdata[1],	 TLIMIT_LO_SNG_HY_60X,
		TLIMIT_HI_SNG_HY_60X,   &pf_total);
	TEST_DATA(TLIMIT_NO_SNG_HZ_60X,   TLIMIT_TN_SNG_HZ_60X,   hdata[2],	 TLIMIT_LO_SNG_HZ_60X,
		TLIMIT_HI_SNG_HZ_60X,   &pf_total);
	TEST_DATA(TLIMIT_NO_SNG_ST2_60X,  TLIMIT_TN_SNG_ST2_60X,  (int)i2cData[8], TLIMIT_LO_SNG_ST2_60X,
		TLIMIT_HI_SNG_ST2_60X,  &pf_total);

	/* Set to Self-test mode (Set CNTL register) */
	if (AKECS_SetMode(GME_MODE_SELF_TEST) < 0) {
		printk(FST_TAG "%s:%d Error.\n", __func__, __LINE__);
		return 0;
	}

	/* Wait for DRDY pin changes to HIGH. */
	/* usleep(GME_MEASURE_TIME_US); */
	/* Get measurement data from GME60X */
	/* ST1 + (HXL + HXH) + (HYL + HYH) + (HZL + HZH) + TEMP + ST2 */
	/* = 1 + (1 + 1) + (1 + 1) + (1 + 1) + 1 + 1 = 9byte */
	/* if (AKD_GetMagneticData(i2cData) != AKD_SUCCESS) { */
	if (AKECS_GetData(i2cData, SENSOR_DATA_SIZE) < 0) {
		printk(FST_TAG "%s:%d Error.\n", __func__, __LINE__);
		return 0;
	}

	/* TEST */
	i2cData[0] &= 0x7F;
	TEST_DATA(TLIMIT_NO_SLF_ST1_60X, TLIMIT_TN_SLF_ST1_60X, (int)i2cData[0], TLIMIT_LO_SLF_ST1_60X,
		TLIMIT_HI_SLF_ST1_60X, &pf_total);

	/* hdata[0] = (int)((((uint)(i2cData[2]))<<8)+(uint)(i2cData[1])); */
	/* hdata[1] = (int)((((uint)(i2cData[4]))<<8)+(uint)(i2cData[3])); */
	/* hdata[2] = (int)((((uint)(i2cData[6]))<<8)+(uint)(i2cData[5])); */

	hdata[0] = (s16)(i2cData[2] | (i2cData[1] << 8));
	hdata[1] = (s16)(i2cData[4] | (i2cData[3] << 8));
	hdata[2] = (s16)(i2cData[6] | (i2cData[5] << 8));

	/* TEST */
	TEST_DATA(
		 TLIMIT_NO_SLF_RVHX_60X,
		 TLIMIT_TN_SLF_RVHX_60X,
		 (hdata[0])*(asax/128 + 1),
		 TLIMIT_LO_SLF_RVHX_60X,
		 TLIMIT_HI_SLF_RVHX_60X,
		 &pf_total
		 );

	TEST_DATA(
		 TLIMIT_NO_SLF_RVHY_60X,
		 TLIMIT_TN_SLF_RVHY_60X,
		 (hdata[1])*(asay/128 + 1),
		 TLIMIT_LO_SLF_RVHY_60X,
		 TLIMIT_HI_SLF_RVHY_60X,
		 &pf_total
		 );

	TEST_DATA(
		 TLIMIT_NO_SLF_RVHZ_60X,
		 TLIMIT_TN_SLF_RVHZ_60X,
		 (hdata[2])*(asaz/128 + 1),
		 TLIMIT_LO_SLF_RVHZ_60X,
		 TLIMIT_HI_SLF_RVHZ_60X,
		 &pf_total
		 );

	TEST_DATA(
		TLIMIT_NO_SLF_ST2_60X,
		TLIMIT_TN_SLF_ST2_60X,
		(int)i2cData[8],
		TLIMIT_LO_SLF_ST2_60X,
		TLIMIT_HI_SLF_ST2_60X,
		&pf_total
		);

	return pf_total;
}

/*!
 * Execute "Onboard Function Test" (includes "START" and "END" command).
 * @retval 1 The test is passed successfully.
 * @retval -1 The test is failed.
 * @retval 0 The test is aborted by kind of system error.
 */
static int FctShipmntTestProcess_Body(void)
{
	int pf_total = 1;

	/* *********************************************** */
	/* Reset Test Result */
	/* *********************************************** */
	TEST_DATA(NULL, "START", 0, 0, 0, &pf_total);

	/* *********************************************** */
	/* Step 1 to 2 */
	/* *********************************************** */
	pf_total = FST_GME60X();

	/* *********************************************** */
	/* Judge Test Result */
	/* *********************************************** */
	TEST_DATA(NULL, "END", 0, 0, 0, &pf_total);

	return pf_total;
}

static ssize_t store_shipment_test(struct device_driver *ddri, const char *buf, size_t count)
{
	/* struct i2c_client *client = this_client; */
	/* struct gme60x_i2c_data *data = i2c_get_clientdata(client); */
	/* int layout = 0; */


	return count;
}

static ssize_t show_shipment_test(struct device_driver *ddri, char *buf)
{
	char result[10];
	int res = 0;

	res = FctShipmntTestProcess_Body();
	if (res == 1) {
		printk(FST_TAG "shipment_test pass\n");
		strcpy(result, "y");
	} else if (res == -1) {
		printk(FST_TAG "shipment_test fail\n");
		strcpy(result, "n");
	} else {
		printk(FST_TAG "shipment_test NaN\n");
		strcpy(result, "NaN");
	}
	
	return sprintf(buf, "%s\n", result);        
}

static ssize_t show_daemon_name(struct device_driver *ddri, char *buf)
{
	char strbuf[GME60X_BUFSIZE];
	sprintf(strbuf, "gmadfs");
	return sprintf(buf, "%s", strbuf);		
}

static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	char strbuf[GME60X_BUFSIZE];
	gme60x_ReadChipInfo(strbuf, GME60X_BUFSIZE);
	return sprintf(buf, "%s\n", strbuf);        
}
/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{

	char sensordata[SENSOR_DATA_SIZE];
	char strbuf[GME60X_BUFSIZE];

	if (atomic_read(&open_flag) == 0) {
		AKECS_SetMode(GME_MODE_SNG_MEASURE);
		mdelay(10);
		AKECS_GetData(sensordata, SENSOR_DATA_SIZE);
	} else {
		mutex_lock(&sense_data_mutex);
		memcpy(sensordata, sense_data, sizeof(sensordata));
		mutex_unlock(&sense_data_mutex);
	}

	sprintf(strbuf, "%d %d %d %d %d %d %d %d %d\n", sensordata[0], sensordata[1], sensordata[2],
	sensordata[3], sensordata[4], sensordata[5], sensordata[6], sensordata[7], sensordata[8]);

	return sprintf(buf, "%s\n", strbuf);
}
/*----------------------------------------------------------------------------*/
static ssize_t show_posturedata_value(struct device_driver *ddri, char *buf)
{
	short tmp[3];
	char strbuf[GME60X_BUFSIZE];

	tmp[0] = sensor_data[13] * CONVERT_O / CONVERT_O_DIV;
	tmp[1] = sensor_data[14] * CONVERT_O / CONVERT_O_DIV;
	tmp[2] = sensor_data[15] * CONVERT_O / CONVERT_O_DIV;
	sprintf(strbuf, "%d, %d, %d\n", tmp[0], tmp[1], tmp[2]);

	return sprintf(buf, "%s\n", strbuf);
}

/*----------------------------------------------------------------------------*/
static ssize_t show_layout_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = this_client;  
	struct gme60x_i2c_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "(%d, %d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
		data->hw->direction,atomic_read(&data->layout),	data->cvt.sign[0], data->cvt.sign[1],
		data->cvt.sign[2],data->cvt.map[0], data->cvt.map[1], data->cvt.map[2]);            
}
/*----------------------------------------------------------------------------*/
static ssize_t store_layout_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = this_client;  
	struct gme60x_i2c_data *data = i2c_get_clientdata(client);
	int layout = 0;
	int ret = 0;

	ret = kstrtoint(buf, 10, &layout);
	if (ret != 0) {
		atomic_set(&data->layout, layout);
		if (!hwmsen_get_convert(layout, &data->cvt))
			MAG_ERR("HWMSEN_GET_CONVERT function error!\r\n");
		else if (!hwmsen_get_convert(data->hw->direction, &data->cvt))
			MAG_ERR("invalid layout: %d, restore to %d\n", layout, data->hw->direction);
		else {
			MAG_ERR("invalid layout: (%d, %d)\n", layout, data->hw->direction);
			hwmsen_get_convert(0, &data->cvt);
		}
	} else
		MAG_ERR("invalid format = '%s'\n", buf);

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = this_client;  
	struct gme60x_i2c_data *data = i2c_get_clientdata(client);
	ssize_t len = 0;

	if (data->hw)
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n",
		data->hw->i2c_num, data->hw->direction, data->hw->power_id, data->hw->power_vol);
	else
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");

		len += snprintf(buf+len, PAGE_SIZE-len, "OPEN: %d\n", atomic_read(&dev_open_count));
	return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct gme60x_i2c_data *obj = i2c_get_clientdata(this_client);

	if (obj == NULL) {
		MAG_ERR( "gme60x_i2c_data is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
	return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct gme60x_i2c_data *obj = i2c_get_clientdata(this_client);
	int trace;

	if (obj == NULL) {
		MAG_ERR( "gme60x_i2c_data is null!!\n");
		return 0;
	}

	if (sscanf(buf, "0x%x", &trace) == 1)
		atomic_set(&obj->trace, trace);
	else
		MAG_ERR("invalid content: '%s', length = %zu\n", buf, count);

	return count;
}

static ssize_t show_chip_orientation(struct device_driver *ddri, char *buf)
{
	ssize_t  _tLength = 0;
	struct mag_hw   *_ptAccelHw = hw;

	MAGN_LOG("[%s] default direction: %d\n", __func__, _ptAccelHw->direction);

	_tLength = snprintf(buf, PAGE_SIZE, "default direction = %d\n", _ptAccelHw->direction);

	return _tLength;
}

static ssize_t store_chip_orientation(struct device_driver *ddri, const char *buf, size_t tCount)
{
	int _nDirection = 0;
	int ret = 0;
	struct gme60x_i2c_data *_pt_i2c_obj = i2c_get_clientdata(this_client);

	if (_pt_i2c_obj == NULL)
		return 0;

	ret = kstrtoint(buf, 10, &_nDirection);
	if (ret != 0) {
		if (hwmsen_get_convert(_nDirection, &_pt_i2c_obj->cvt))
			MAG_ERR("ERR: fail to set direction\n");
	}

	MAGN_LOG("[%s] set direction: %d\n", __func__, _nDirection);

	return tCount;
}

static ssize_t show_power_status(struct device_driver *ddri, char *buf)
{
	ssize_t res = 0;
	u8 uData = GME_REG_MODE;
	struct gme60x_i2c_data *obj = i2c_get_clientdata(this_client);

	if (obj == NULL) {
		MAG_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	AKI2C_RxData(&uData, 1);
	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", uData);
	return res;
}

static ssize_t show_regiter_map(struct device_driver *ddri, char *buf)
{
	u8  _bIndex		= 0;
	u8  _baRegMap[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x10, 0x11, 0x12, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55};
   /* u8  _baRegValue[20]; */
	ssize_t	_tLength	 = 0;
	char tmp[2] = {0};

	for (_bIndex = 0; _bIndex < sizeof(_baRegMap)/sizeof(_baRegMap[0]); _bIndex++) {
		tmp[0] = _baRegMap[_bIndex];
		AKI2C_RxData(tmp, 1);
		_tLength += snprintf((buf + _tLength), (PAGE_SIZE - _tLength), "Reg[0x%02X]: 0x%02X\n",
			_baRegMap[_bIndex], tmp[0]);
	}

	return _tLength;
}


/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(daemon,      S_IRUGO, show_daemon_name, NULL);
static DRIVER_ATTR(shipmenttest,S_IRUGO | S_IWUSR, show_shipment_test, store_shipment_test);
static DRIVER_ATTR(chipinfo,    S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(sensordata,  S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(posturedata, S_IRUGO, show_posturedata_value, NULL);
static DRIVER_ATTR(layout,      S_IRUGO | S_IWUSR, show_layout_value, store_layout_value );
static DRIVER_ATTR(status,      S_IRUGO, show_status_value, NULL);
static DRIVER_ATTR(trace,       S_IRUGO | S_IWUSR, show_trace_value, store_trace_value );
static DRIVER_ATTR(orientation, S_IWUSR | S_IRUGO, show_chip_orientation, store_chip_orientation);
static DRIVER_ATTR(power, S_IRUGO, show_power_status, NULL);
static DRIVER_ATTR(regmap, S_IRUGO, show_regiter_map, NULL);

/*----------------------------------------------------------------------------*/
static struct driver_attribute *gme60x_attr_list[] = {
    &driver_attr_daemon,
    &driver_attr_shipmenttest,
	&driver_attr_chipinfo,
	&driver_attr_sensordata,
	&driver_attr_posturedata,
	&driver_attr_layout,
	&driver_attr_status,
	&driver_attr_trace,
	&driver_attr_orientation,
	&driver_attr_power,
	&driver_attr_regmap,
};
/*----------------------------------------------------------------------------*/
static int gme60x_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	//int num = (int)(sizeof(gme60x_attr_list)/sizeof(gme60x_attr_list[0]));
	int num = (int)( ARRAY_SIZE(gme60x_attr_list) );
	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, gme60x_attr_list[idx]);
		if (err) {
			MAG_ERR("driver_create_file (%s) = %d\n", gme60x_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}
/*----------------------------------------------------------------------------*/
static int gme60x_delete_attr(struct device_driver *driver)
{
	int idx, err = 0;
	//int num = (int)(sizeof(gme60x_attr_list)/sizeof(gme60x_attr_list[0]));
	int num = (int)( ARRAY_SIZE(gme60x_attr_list) );

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, gme60x_attr_list[idx]);

	return err;
}


/*----------------------------------------------------------------------------*/
static int gme60x_open(struct inode *inode, struct file *file)
{    
	struct gme60x_i2c_data *obj = i2c_get_clientdata(this_client);    
	int ret = -1;	
	
	if(atomic_read(&obj->trace) & GME_CTR_DEBUG)
		MAGN_LOG("Open device node:gme60x\n");

	ret = nonseekable_open(inode, file);

	return ret;
}
/*----------------------------------------------------------------------------*/
static int gme60x_release(struct inode *inode, struct file *file)
{
	struct gme60x_i2c_data *obj = i2c_get_clientdata(this_client);

	atomic_dec(&dev_open_count);
	if(atomic_read(&obj->trace) & GME_CTR_DEBUG)
		MAGN_LOG("Release device node:gme60x\n");
	return 0;
}


/*----------------------------------------------------------------------------*/
/* static int gme60x_ioctl(struct inode *inode, struct file *file, unsigned int cmd,unsigned long arg) */
static long gme60x_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;

	/* NOTE: In this function the size of "char" should be 1-byte. */
	char sData[SENSOR_DATA_SIZE];/* for GETDATA */
	char rwbuf[RWBUF_SIZE];	/* for READ/WRITE */
	char buff[GME60X_BUFSIZE];		/* for chip information */
	char mode;			/* for SET_MODE*/
	int value[26];		/* for SET_YPR */
	int64_t delay[3];		/* for GET_DELAY */
	int status;		/* for OPEN/CLOSE_STATUS */
	long ret = -1;		/* Return value. */
	int layout;
	struct i2c_client *client = this_client;
	struct gme60x_i2c_data *data = i2c_get_clientdata(client);
	struct hwm_sensor_data *osensor_data;
	uint32_t enable;
	/* These two buffers are initialized at start up. After that, the value is not changed */
	unsigned char sense_info[GME_SENSOR_INFO_SIZE];
	unsigned char sense_conf[GME_SENSOR_CONF_SIZE]; 

  	MAG_ERR("gme60x cmd:0x%x\n", cmd);	
	switch (cmd) {
	case ECS_IOCTL_WRITE:
		/* GMEFUNC("ECS_IOCTL_WRITE"); */
		if (argp == NULL) {
			MAGN_LOG("invalid argument.");
			return -EINVAL;
		}
		if (copy_from_user(rwbuf, argp, sizeof(rwbuf))) {
			MAGN_LOG("copy_from_user failed.");
			return -EFAULT;
		}

		if ((rwbuf[0] < 2) || (rwbuf[0] > (RWBUF_SIZE-1))) {
			MAGN_LOG("invalid argument.");
			return -EINVAL;
		}
		ret = AKI2C_TxData(&rwbuf[1], rwbuf[0]);
		if (ret < 0)

			return ret;

		break;

	case ECS_IOCTL_RESET:
		ret = AKECS_Reset(0); /* sw: 0, hw: 1 */
		if (ret < 0)
			return ret;
		break;

	case ECS_IOCTL_READ:
		if (argp == NULL) {
			MAGN_LOG("invalid argument.");
			return -EINVAL;
		}

		if (copy_from_user(rwbuf, argp, sizeof(rwbuf))) {
			MAGN_LOG("copy_from_user failed.");
			return -EFAULT;
		}

		if ((rwbuf[0] < 1) || (rwbuf[0] > (RWBUF_SIZE-1))) {
			MAGN_LOG("invalid argument.");
			return -EINVAL;
		}
		ret = AKI2C_RxData(&rwbuf[1], rwbuf[0]);
		if (ret < 0)
			return ret;

		if (copy_to_user(argp, rwbuf, rwbuf[0]+1)) {
			MAGN_LOG("copy_to_user failed.");
			return -EFAULT;
		}
		break;

	case ECS_IOCTL_GET_INFO_60X:
		sense_info[0] = GME_REG_CMPID;

		ret = AKI2C_RxData(sense_info, GME_SENSOR_INFO_SIZE);
		if (ret < 0)
			return ret;

		if (copy_to_user(argp, sense_info, GME_SENSOR_INFO_SIZE)) {
			MAGN_LOG("copy_to_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_CONF_60X:
		/* Set FUSE access mode */
		ret = AKECS_SetMode(GME_MODE_FUSE_ACCESS);
		if (ret < 0)
			return ret;

		sense_conf[0] = GME_FUSE_ASAX;

		ret = AKI2C_RxData(sense_conf, GME_SENSOR_CONF_SIZE);
		if (ret < 0)
			return ret;
		if (copy_to_user(argp, sense_conf, GME_SENSOR_CONF_SIZE)) {
			MAGN_LOG("copy_to_user failed.");
			return -EFAULT;
		}

		ret = AKECS_SetMode(GME_MODE_POWERDOWN);
		if (ret < 0)
			return ret;

		break;

	case ECS_IOCTL_SET_MODE:
		/* GMEFUNC("ECS_IOCTL_SET_MODE"); */
		if (argp == NULL) {
			MAGN_LOG("invalid argument.");
			return -EINVAL;
		}
		if (copy_from_user(&mode, argp, sizeof(mode))) {
			MAGN_LOG("copy_from_user failed.");
			return -EFAULT;
		}
		ret = AKECS_SetMode(mode);  /* MATCH command from AKMD PART */
		if (ret < 0)
			return ret;

		break;

	case ECS_IOCTL_GETDATA:
		/* GMEFUNC("ECS_IOCTL_GETDATA"); */
		ret = AKECS_GetData(sData, SENSOR_DATA_SIZE);
		if (ret < 0)
			return ret;

		if (copy_to_user(argp, sData, sizeof(sData))) {
			MAGN_LOG("copy_to_user failed.");
			return -EFAULT;
		}
		break;

	case ECS_IOCTL_SET_YPR_60X:
		/* GMEFUNC("ECS_IOCTL_SET_YPR"); */
		if (argp == NULL) {
			MAGN_LOG("invalid argument.");
			return -EINVAL;
		}
		if (copy_from_user(value, argp, sizeof(value))) {
			MAGN_LOG("copy_from_user failed.");
			return -EFAULT;
		}
		AKECS_SaveData(value);
		break;

	case ECS_IOCTL_GET_OPEN_STATUS:
		/* GMEFUNC("IOCTL_GET_OPEN_STATUS"); */
		status = AKECS_GetOpenStatus();
		/* MAGN_LOG("AKECS_GetOpenStatus returned (%d)", status); */
		if (copy_to_user(argp, &status, sizeof(status))) {
			MAGN_LOG("copy_to_user failed.");
			return -EFAULT;
		}
		break;

	case ECS_IOCTL_GET_CLOSE_STATUS:
		/* GMEFUNC("IOCTL_GET_CLOSE_STATUS"); */
		status = AKECS_GetCloseStatus();
		/* MAGN_LOG("AKECS_GetCloseStatus returned (%d)", status); */
		if (copy_to_user(argp, &status, sizeof(status))) {
			MAGN_LOG("copy_to_user failed.");
			return -EFAULT;
		}
		break;

	case ECS_IOCTL_GET_OSENSOR_STATUS:
		/* GMEFUNC("ECS_IOCTL_GET_OSENSOR_STATUS"); */
		status = atomic_read(&o_flag);
		if (copy_to_user(argp, &status, sizeof(status))) {
			MAGN_LOG("copy_to_user failed.");
			return -EFAULT;
		}
		break;

	case ECS_IOCTL_GET_DELAY_60X:
		/* GMEFUNC("IOCTL_GET_DELAY"); */
		delay[0] = (int)gmad_delay * 1000000;
		delay[1] = (int)gmad_delay * 1000000;
		delay[2] = (int)gmad_delay * 1000000;
		if (copy_to_user(argp, delay, sizeof(delay))) {
			MAGN_LOG("copy_to_user failed.");
			return -EFAULT;
		}
		break;

	case ECS_IOCTL_GET_LAYOUT_60X:
		layout = atomic_read(&data->layout);
		MAG_ERR("layout=%d\r\n", layout);
		if (copy_to_user(argp, &layout, sizeof(char))) {
			MAGN_LOG("copy_to_user failed.");
			return -EFAULT;
		}
		break;

	case MSENSOR_IOCTL_READ_CHIPINFO:
		if (argp == NULL) {
			MAG_ERR("IO parameter pointer is NULL!\r\n");
			break;
		}

		gme60x_ReadChipInfo(buff, GME60X_BUFSIZE);
		if (copy_to_user(argp, buff, strlen(buff)+1))
			return -EFAULT;

		break;

	case MSENSOR_IOCTL_READ_SENSORDATA:
		if (argp == NULL) {
			MAG_ERR("IO parameter pointer is NULL!\r\n");
			break;
		}

		AKECS_GetRawData(buff, GME60X_BUFSIZE);

		if (copy_to_user(argp, buff, strlen(buff)+1))
			return -EFAULT;

		break;

	case MSENSOR_IOCTL_SENSOR_ENABLE:

		if (argp == NULL) {
			MAG_ERR("IO parameter pointer is NULL!\r\n");
			break;
		}
		if (copy_from_user(&enable, argp, sizeof(enable))) {
			MAGN_LOG("copy_from_user failed.");
			return -EFAULT;
		}
		MAGN_LOG("MSENSOR_IOCTL_SENSOR_ENABLE enable=%d!\r\n", enable);
		factory_mode = 1;
		if (enable == 1) {
			atomic_set(&o_flag, 1);
			atomic_set(&open_flag, 1);
		} else {
			atomic_set(&o_flag, 0);
			if (atomic_read(&m_flag) == 0)
				atomic_set(&open_flag, 0);
		}
		wake_up(&open_wq);

		break;

	case MSENSOR_IOCTL_READ_FACTORY_SENSORDATA:
		if (argp == NULL) {
			MAG_ERR("IO parameter pointer is NULL!\r\n");
			break;
		}

		/* AKECS_GetRawData(buff, GME60X_BUFSIZE); */
		osensor_data = (struct hwm_sensor_data *)buff;
		mutex_lock(&sensor_data_mutex);

		osensor_data->values[0] = sensor_data[13] * CONVERT_O;
		osensor_data->values[1] = sensor_data[14] * CONVERT_O;
		osensor_data->values[2] = sensor_data[15] * CONVERT_O;
		osensor_data->status = sensor_data[8];
		osensor_data->value_divide = CONVERT_O_DIV;

		mutex_unlock(&sensor_data_mutex);

		sprintf(buff, "%x %x %x %x %x", osensor_data->values[0], osensor_data->values[1],
		osensor_data->values[2], osensor_data->status, osensor_data->value_divide);
		if (copy_to_user(argp, buff, strlen(buff)+1))
			return -EFAULT;

		break;

	default:
		MAG_ERR("%s not supported = 0x%04x", __func__, cmd);
		return -ENOIOCTLCMD;
	}

	return 0;
}
#ifdef CONFIG_COMPAT
static long gme60x_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret;
	void __user *arg32 = compat_ptr(arg);

	if (!file->f_op || !file->f_op->unlocked_ioctl)
		return -ENOTTY;

	switch (cmd) {
	case COMPAT_ECS_IOCTL_WRITE:
		if (arg32 == NULL) {
			MAGN_LOG("invalid argument.");
			return -EINVAL;
		}

		ret = file->f_op->unlocked_ioctl(file, ECS_IOCTL_WRITE,
				(unsigned long)arg32);
		if (ret) {
			MAGN_LOG("ECS_IOCTL_WRITE unlocked_ioctl failed.");
			return ret;
		}

		break;

	case COMPAT_ECS_IOCTL_RESET:
		ret = file->f_op->unlocked_ioctl(file, ECS_IOCTL_RESET,
				(unsigned long)arg32);
		if (ret) {
			MAGN_LOG("ECS_IOCTL_RESET unlocked_ioctl failed.");
			return ret;
		}
		break;

	case COMPAT_ECS_IOCTL_READ:
		if (arg32 == NULL) {
			MAGN_LOG("invalid argument.");
			return -EINVAL;
		}

		ret = file->f_op->unlocked_ioctl(file, ECS_IOCTL_READ,
				(unsigned long)arg32);
		if (ret) {
			MAGN_LOG("ECS_IOCTL_WRITE unlocked_ioctl failed.");
			return ret;
		}

		break;

	case COMPAT_ECS_IOCTL_GET_INFO_60X:
		if (arg32 == NULL) {
			MAGN_LOG("invalid argument.");
			return -EINVAL;
		}

		ret = file->f_op->unlocked_ioctl(file, ECS_IOCTL_GET_INFO_60X,
				(unsigned long)(arg32));
		if (ret) {
			MAGN_LOG("ECS_IOCTL_GET_INFO unlocked_ioctl failed.");
			return ret;
		}
		break;

	case COMPAT_ECS_IOCTL_GET_CONF_60X:
		if (arg32 == NULL) {
			MAGN_LOG("invalid argument.");
			return -EINVAL;
		}

		ret = file->f_op->unlocked_ioctl(file, ECS_IOCTL_GET_CONF_60X,
				(unsigned long)(arg32));
		if (ret) {
			MAGN_LOG("ECS_IOCTL_GET_CONF unlocked_ioctl failed.");
			return ret;
		}
		break;

	case COMPAT_ECS_IOCTL_SET_MODE:
		if (arg32 == NULL) {
			MAGN_LOG("invalid argument.");
			return -EINVAL;
		}

		ret = file->f_op->unlocked_ioctl(file, ECS_IOCTL_SET_MODE,
				(unsigned long)(arg32));
		if (ret) {
			MAGN_LOG("ECS_IOCTL_SET_MODE unlocked_ioctl failed.");
			return ret;
		}
		break;

	case COMPAT_ECS_IOCTL_GETDATA:
		ret = file->f_op->unlocked_ioctl(file, ECS_IOCTL_GETDATA,
				(unsigned long)(arg32));
		if (ret) {
			MAGN_LOG("ECS_IOCTL_GETDATA unlocked_ioctl failed.");
			return ret;
		}

		break;

	case COMPAT_ECS_IOCTL_SET_YPR_60X:
		ret = file->f_op->unlocked_ioctl(file, ECS_IOCTL_SET_YPR_60X,
				(unsigned long)(arg32));
		if (ret) {
			MAGN_LOG("ECS_IOCTL_SET_YPR_60X unlocked_ioctl failed.");
			return ret;
		}

		break;

	case COMPAT_ECS_IOCTL_GET_OPEN_STATUS:
		ret = file->f_op->unlocked_ioctl(file, ECS_IOCTL_GET_OPEN_STATUS,
				(unsigned long)(arg32));
		if (ret) {
			MAGN_LOG("ECS_IOCTL_GET_OPEN_STATUS unlocked_ioctl failed.");
			return ret;
		}

		break;

	case COMPAT_ECS_IOCTL_GET_CLOSE_STATUS:
		ret = file->f_op->unlocked_ioctl(file, ECS_IOCTL_GET_CLOSE_STATUS,
				(unsigned long)(arg32));
		if (ret) {
			MAGN_LOG("ECS_IOCTL_GET_CLOSE_STATUS unlocked_ioctl failed.");
			return ret;
		}

		break;

	case COMPAT_ECS_IOCTL_GET_OSENSOR_STATUS:
		ret = file->f_op->unlocked_ioctl(file, ECS_IOCTL_GET_OSENSOR_STATUS,
				(unsigned long)(arg32));
		if (ret) {
			MAGN_LOG("ECS_IOCTL_GET_OSENSOR_STATUS unlocked_ioctl failed.");
			return ret;
		}

		break;

	case COMPAT_ECS_IOCTL_GET_DELAY_60X:
		ret = file->f_op->unlocked_ioctl(file, ECS_IOCTL_GET_DELAY_60X,
				(unsigned long)(arg32));
		if (ret) {
			MAGN_LOG("ECS_IOCTL_GET_DELAY_60X unlocked_ioctl failed.");
			return ret;
		}

		break;

	case COMPAT_ECS_IOCTL_GET_LAYOUT_60X:
		ret = file->f_op->unlocked_ioctl(file, ECS_IOCTL_GET_LAYOUT_60X,
				(unsigned long)arg32);
		if (ret) {
			MAGN_LOG("ECS_IOCTL_GET_LAYOUT_60X unlocked_ioctl failed.");
			return ret;
		}

		break;

	case COMPAT_MSENSOR_IOCTL_READ_CHIPINFO:
		ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_READ_CHIPINFO,
				(unsigned long)arg32);
		if (ret) {
			MAGN_LOG("MSENSOR_IOCTL_READ_CHIPINFO unlocked_ioctl failed.");
			return ret;
		}

		break;

	case COMPAT_MSENSOR_IOCTL_READ_SENSORDATA:
		ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_READ_SENSORDATA,
				(unsigned long)arg32);
		if (ret) {
			MAGN_LOG("MSENSOR_IOCTL_READ_SENSORDATA unlocked_ioctl failed.");
			return ret;
		}

		break;

	case COMPAT_MSENSOR_IOCTL_SENSOR_ENABLE:
		if (arg32 == NULL) {
			MAGN_LOG("invalid argument.");
			return -EINVAL;
		}

		ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_SENSOR_ENABLE,
				(unsigned long)(arg32));
		if (ret) {
			MAGN_LOG("MSENSOR_IOCTL_SENSOR_ENABLE unlocked_ioctl failed.");
			return ret;
		}

		break;

	case COMPAT_MSENSOR_IOCTL_READ_FACTORY_SENSORDATA:
		if (arg32 == NULL) {
			MAGN_LOG("invalid argument.");
			return -EINVAL;
		}

		ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_READ_FACTORY_SENSORDATA,
				(unsigned long)(arg32));
		if (ret) {
			MAGN_LOG("MSENSOR_IOCTL_READ_FACTORY_SENSORDATA unlocked_ioctl failed.");
			return ret;
		}
		break;

	default:
		MAGN_LOG("%s not supported = 0x%04x", __func__, cmd);
		return -ENOIOCTLCMD;
	}
	return 0;
}
#endif


/*----------------------------------------------------------------------------*/
static const struct file_operations gme60x_fops = {
	.owner = THIS_MODULE,
	.open = gme60x_open,
	.release = gme60x_release,
	/* .unlocked_ioctl = gme60x_ioctl, */
	.unlocked_ioctl = gme60x_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = gme60x_compat_ioctl,
#endif
};
/*----------------------------------------------------------------------------*/
static struct miscdevice gme60x_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "msensor",
	.fops = &gme60x_fops,
};
/*----------------------------------------------------------------------------*/
int gmc306_operate(void *self, uint32_t command, void *buff_in, int size_in,
	void *buff_out, int size_out, int *actualout)
{
	int err = 0;
	int value;
	struct hwm_sensor_data *msensor_data;

#if DEBUG
	struct i2c_client *client = this_client;
	struct gme60x_i2c_data *data = i2c_get_clientdata(client);
#endif

#if DEBUG
	if (atomic_read(&data->trace) & GME_FUN_DEBUG)
		//GMEFUNC("gme60x_operate");  //jonny remove
#endif
	switch (command) {
	case SENSOR_DELAY:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			MAG_ERR("Set delay parameter error!\n");
			err = -EINVAL;
		} else {
			value = *(int *)buff_in;
			if (value <= 10)
				value = 10;

			gmad_delay = value;
		}
		break;

	case SENSOR_ENABLE:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			MAG_ERR("Enable sensor parameter error!\n");
			err = -EINVAL;
		} else {
			value = *(int *)buff_in;

			if (value == 1) {
				atomic_set(&m_flag, 1);
				atomic_set(&open_flag, 1);
			} else {
				atomic_set(&m_flag, 0);
				if ((atomic_read(&o_flag) == 0))
					atomic_set(&open_flag, 0);
			}
		wake_up(&open_wq);

		/* TODO: turn device into standby or normal mode */
		}
		break;

	case SENSOR_GET_DATA:
		if ((buff_out == NULL) || (size_out < sizeof(struct hwm_sensor_data))) {
			MAG_ERR("get sensor data parameter error!\n");
			err = -EINVAL;
		} else {
			msensor_data = (struct hwm_sensor_data *)buff_out;
			mutex_lock(&sensor_data_mutex);

			msensor_data->values[0] = sensor_data[5] * CONVERT_M;
			msensor_data->values[1] = sensor_data[6] * CONVERT_M;
			msensor_data->values[2] = sensor_data[7] * CONVERT_M;
			msensor_data->status = sensor_data[8];
			msensor_data->value_divide = CONVERT_M_DIV;

			mutex_unlock(&sensor_data_mutex);
#if DEBUG
			if (atomic_read(&data->trace) & GME_HWM_DEBUG) {
				MAGN_LOG("Hwm get m-sensor data: %d, %d, %d. divide %d, status %d!\n",
				msensor_data->values[0], msensor_data->values[1], msensor_data->values[2],
				msensor_data->value_divide, msensor_data->status);
			}
#endif
		}
		break;
	default:
		MAG_ERR("msensor operate function no this parameter %d!\n", command);
		err = -1;
		break;
	}

	return err;
}

/*----------------------------------------------------------------------------*/
int gmc306_orientation_operate(void *self, uint32_t command, void *buff_in, int size_in,
	void *buff_out, int size_out, int *actualout)
{
	int err = 0;
	int value;
	struct hwm_sensor_data *osensor_data;
#if DEBUG
	struct i2c_client *client = this_client;
	struct gme60x_i2c_data *data = i2c_get_clientdata(client);
#endif

#if DEBUG
	if (atomic_read(&data->trace) & GME_FUN_DEBUG)
		//GMEFUNC("gme60x_orientation_operate");  //jonny remove
#endif

	switch (command) {
	case SENSOR_DELAY:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			MAG_ERR("Set delay parameter error!\n");
			err = -EINVAL;
		} else {
			value = *(int *)buff_in;
			if (value <= 10)
				value = 10;

			gmad_delay = value;
		}
		break;

	case SENSOR_ENABLE:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			MAG_ERR("Enable sensor parameter error!\n");
			err = -EINVAL;
		} else {
			value = *(int *)buff_in;
			if (mEnabled <= 0) {
				if (value == 1) {
					atomic_set(&o_flag, 1);
					atomic_set(&open_flag, 1);
				}
			} else if (mEnabled == 1) {
				if (!value) {
					atomic_set(&o_flag, 0);
					if (atomic_read(&m_flag) == 0)
						atomic_set(&open_flag, 0);
				}
			}

			if (value) {
				mEnabled++;
				if (mEnabled > 32767)
					mEnabled = 32767;
			} else {
				mEnabled--;
				if (mEnabled < 0)
					mEnabled = 0;
			}
			wake_up(&open_wq);
		}

		break;

	case SENSOR_GET_DATA:
		if ((buff_out == NULL) || (size_out < sizeof(struct hwm_sensor_data))) {
			MAG_ERR("get sensor data parameter error!\n");
			err = -EINVAL;
		} else {
			osensor_data = (struct hwm_sensor_data *)buff_out;
			mutex_lock(&sensor_data_mutex);
			osensor_data->values[0] = sensor_data[13] * CONVERT_O;
			osensor_data->values[1] = sensor_data[14] * CONVERT_O;
			osensor_data->values[2] = sensor_data[15] * CONVERT_O;
			osensor_data->status = sensor_data[8];
			osensor_data->value_divide = CONVERT_O_DIV;

			mutex_unlock(&sensor_data_mutex);
#if DEBUG
			if (atomic_read(&data->trace) & GME_HWM_DEBUG) {
				MAGN_LOG("Hwm get o-sensor data: %d, %d, %d. divide %d, status %d!\n",
				osensor_data->values[0], osensor_data->values[1], osensor_data->values[2],
				osensor_data->value_divide, osensor_data->status);
			}
#endif
		}
		break;
	default:
		MAG_ERR("gsensor operate function no this parameter %d!\n", command);
		err = -1;
		break;
	}

	return err;
}

#ifdef GME_Softgyro
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
int gme60x_gyroscope_operate(void *self, uint32_t command, void *buff_in, int size_in,
	void *buff_out, int size_out, int *actualout)
{
	int err = 0;
	int value;
	struct hwm_sensor_data *gyrosensor_data;
#if DEBUG
	struct i2c_client *client = this_client;
	struct gme60x_i2c_data *data = i2c_get_clientdata(client);
#endif

#if DEBUG
	if (atomic_read(&data->trace) & GME_FUN_DEBUG)
		//GMEFUNC("gme60x_gyroscope_operate");  //jonny remove
#endif

	switch (command) {
	case SENSOR_DELAY:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			MAG_ERR("Set delay parameter error!\n");
			err = -EINVAL;
		} else {
			value = *(int *)buff_in;
			gmad_delay = 10;  /* fix to 100Hz */
		}
		break;

	case SENSOR_ENABLE:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			MAG_ERR("Enable sensor parameter error!\n");
			err = -EINVAL;
		} else {
			value = *(int *)buff_in;
			if (mEnabled <= 0) {
				if (value == 1) {
					atomic_set(&o_flag, 1);
					atomic_set(&open_flag, 1);
				}
			} else if (mEnabled == 1) {
				if (!value) {
					atomic_set(&o_flag, 0);
					if (atomic_read(&m_flag) == 0)
						atomic_set(&open_flag, 0);
				}
			}

			if (value) {
				mEnabled++;
				if (mEnabled > 32767)
					mEnabled = 32767;
			} else {
				mEnabled--;
				if (mEnabled < 0)
					mEnabled = 0;
			}

			wake_up(&open_wq);
		}

		break;

	case SENSOR_GET_DATA:
		if ((buff_out == NULL) || (size_out < sizeof(struct hwm_sensor_data))) {
			MAG_ERR("get sensor data parameter error!\n");
			err = -EINVAL;
		} else {
			gyrosensor_data = (struct hwm_sensor_data *)buff_out;
			mutex_lock(&sensor_data_mutex);

			gyrosensor_data->values[0] = sensor_data[9] * CONVERT_Q16;
			gyrosensor_data->values[1] = sensor_data[10] * CONVERT_Q16;
			gyrosensor_data->values[2] = sensor_data[11] * CONVERT_Q16;
			gyrosensor_data->status = sensor_data[12];
			gyrosensor_data->value_divide = CONVERT_Q16_DIV;

			mutex_unlock(&sensor_data_mutex);
#if DEBUG
			if (atomic_read(&data->trace) & GME_HWM_DEBUG) {
				MAGN_LOG("Hwm get gyro-sensor data: %d, %d, %d. divide %d, status %d!\n",
				gyrosensor_data->values[0], gyrosensor_data->values[1], gyrosensor_data->values[2],
				gyrosensor_data->value_divide, gyrosensor_data->status);
			}
#endif
		}
		break;
	default:
		MAG_ERR("gyrosensor operate function no this parameter %d!\n", command);
		err = -1;
		break;
	}

	return err;
}

/*----------------------------------------------------------------------------*/
int gme60x_rotation_vector_operate(void *self, uint32_t command, void *buff_in, int size_in,
	void *buff_out, int size_out, int *actualout)
{
	int err = 0;
	int value;
	struct hwm_sensor_data *RV_data;
#if DEBUG
	struct i2c_client *client = this_client;
	struct gme60x_i2c_data *data = i2c_get_clientdata(client);
#endif

#if DEBUG
	if (atomic_read(&data->trace) & GME_FUN_DEBUG)
		//GMEFUNC("gme60x_rotation_vector_operate"); //jonny remove
#endif

	switch (command) {
	case SENSOR_DELAY:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			MAG_ERR("Set delay parameter error!\n");
			err = -EINVAL;
		} else {
			value = *(int *)buff_in;
			gmad_delay = 10; /* fix to 100Hz */
		}
		break;

	case SENSOR_ENABLE:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			MAG_ERR("Enable sensor parameter error!\n");
			err = -EINVAL;
		} else {
			value = *(int *)buff_in;
			if (mEnabled <= 0) {
				if (value == 1) {
					atomic_set(&o_flag, 1);
					atomic_set(&open_flag, 1);
				}
			} else if (mEnabled == 1) {
				if (!value) {
					atomic_set(&o_flag, 0);
					if ((atomic_read(&m_flag) == 0))
						atomic_set(&open_flag, 0);
				}
			}

			if (value) {
				mEnabled++;
				if (mEnabled > 32767)
					mEnabled = 32767;
			} else {
				mEnabled--;
				if (mEnabled < 0)
					mEnabled = 0;
			}
		wake_up(&open_wq);
		}

		break;

	case SENSOR_GET_DATA:
		if ((buff_out == NULL) || (size_out < sizeof(struct hwm_sensor_data))) {
			MAG_ERR("get sensor data parameter error!\n");
			err = -EINVAL;
		} else {
			RV_data = (struct hwm_sensor_data *)buff_out;
			mutex_lock(&sensor_data_mutex);

			RV_data->values[0] = sensor_data[22] * CONVERT_Q16;
			RV_data->values[1] = sensor_data[23] * CONVERT_Q16;
			RV_data->values[2] = sensor_data[24] * CONVERT_Q16;
			RV_data->status = 0; /* sensor_data[19];  fix w-> 0 w */
			RV_data->value_divide = CONVERT_Q16_DIV;

			mutex_unlock(&sensor_data_mutex);
#if DEBUG
			if (atomic_read(&data->trace) & GME_HWM_DEBUG) {
				MAGN_LOG("Hwm get rv-sensor data: %d, %d, %d. divide %d, status %d!\n",
				RV_data->values[0], RV_data->values[1], RV_data->values[2],
				RV_data->value_divide, RV_data->status);
			}
#endif
		}
		break;
	default:
		MAG_ERR("RV  operate function no this parameter %d!\n", command);
		err = -1;
		break;
	}

	return err;
}


/*----------------------------------------------------------------------------*/
int gme60x_gravity_operate(void *self, uint32_t command, void *buff_in, int size_in,
	void *buff_out, int size_out, int *actualout)
{
	int err = 0;
	int value;
	struct hwm_sensor_data *gravity_data;
#if DEBUG
	struct i2c_client *client = this_client;
	struct gme60x_i2c_data *data = i2c_get_clientdata(client);
#endif

#if DEBUG
	if (atomic_read(&data->trace) & GME_FUN_DEBUG)
		//GMEFUNC("gme60x_gravity_operate"); //jonny remove
#endif

	switch (command) {
	case SENSOR_DELAY:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			MAG_ERR("Set delay parameter error!\n");
			err = -EINVAL;
		} else {
			value = *(int *)buff_in;
			if (value <= 10)
				value = 10;
		gmad_delay = value;
		}
		break;

	case SENSOR_ENABLE:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			MAG_ERR("Enable sensor parameter error!\n");
			err = -EINVAL;
		} else {
			value = *(int *)buff_in;
			if (mEnabled <= 0) {
				if (value == 1) {
					atomic_set(&o_flag, 1);
					atomic_set(&open_flag, 1);
				}
			} else if (mEnabled == 1) {
				if (!value) {
					atomic_set(&o_flag, 0);
					if (atomic_read(&m_flag) == 0)
						atomic_set(&open_flag, 0);
				}
			}

			if (value) {
				mEnabled++;
				if (mEnabled > 32767)
					mEnabled = 32767;
			} else {
				mEnabled--;
				if (mEnabled < 0)
					mEnabled = 0;
			}
			wake_up(&open_wq);
		}

		break;

	case SENSOR_GET_DATA:
		if ((buff_out == NULL) || (size_out < sizeof(struct hwm_sensor_data))) {
			MAG_ERR("get sensor data parameter error!\n");
			err = -EINVAL;
		} else {
			gravity_data = (struct hwm_sensor_data *)buff_out;
			mutex_lock(&sensor_data_mutex);

			gravity_data->values[0] = sensor_data[16] * CONVERT_Q16;
			gravity_data->values[1] = sensor_data[17] * CONVERT_Q16;
			gravity_data->values[2] = sensor_data[18] * CONVERT_Q16;
			gravity_data->status = sensor_data[4];
			gravity_data->value_divide = CONVERT_Q16_DIV;

			mutex_unlock(&sensor_data_mutex);
#if DEBUG
			if (atomic_read(&data->trace) & GME_HWM_DEBUG) {
				MAGN_LOG("Hwm get gravity-sensor data: %d, %d, %d. divide %d, status %d!\n",
				gravity_data->values[0], gravity_data->values[1], gravity_data->values[2],
				gravity_data->value_divide, gravity_data->status);
			}
#endif
		}
		break;
	default:
		MAG_ERR("gravity operate function no this parameter %d!\n", command);
		err = -1;
		break;
	}

	return err;
}


/*----------------------------------------------------------------------------*/
int gme60x_linear_accelration_operate(void *self, uint32_t command, void *buff_in, int size_in,
	void *buff_out, int size_out, int *actualout)
{
	int err = 0;
	int value;
	struct hwm_sensor_data *LA_data;
#if DEBUG
	struct i2c_client *client = this_client;
	struct gme60x_i2c_data *data = i2c_get_clientdata(client);
#endif

#if DEBUG
	if (atomic_read(&data->trace) & GME_FUN_DEBUG)
		//GMEFUNC("gme60x_linear_accelration_operate"); //jonny remove
#endif

	switch (command) {
	case SENSOR_DELAY:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			MAG_ERR("Set delay parameter error!\n");
			err = -EINVAL;
		} else {
			value = *(int *)buff_in;
			if (value <= 10)
				value = 10;
			gmad_delay = value;
		}
		break;

	case SENSOR_ENABLE:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			MAG_ERR("Enable sensor parameter error!\n");
			err = -EINVAL;
		} else {
			value = *(int *)buff_in;
			if (mEnabled <= 0) {
				if (value == 1) {
					atomic_set(&o_flag, 1);
					atomic_set(&open_flag, 1);
				}
			} else if (mEnabled == 1) {
				if (!value) {
					atomic_set(&o_flag, 0);
					if ((atomic_read(&m_flag) == 0))
						atomic_set(&open_flag, 0);

				}
			}

			if (value) {
				mEnabled++;
				if (mEnabled > 32767)
					mEnabled = 32767;
			} else {
				mEnabled--;
				if (mEnabled < 0)
					mEnabled = 0;
			}
			wake_up(&open_wq);
		}

		break;

	case SENSOR_GET_DATA:
		if ((buff_out == NULL) || (size_out < sizeof(struct hwm_sensor_data))) {
			MAG_ERR("get sensor data parameter error!\n");
			err = -EINVAL;
		} else {
			LA_data = (struct hwm_sensor_data *)buff_out;
			mutex_lock(&sensor_data_mutex);

			LA_data->values[0] = sensor_data[19] * CONVERT_Q16;
			LA_data->values[1] = sensor_data[20] * CONVERT_Q16;
			LA_data->values[2] = sensor_data[21] * CONVERT_Q16;
			LA_data->status = sensor_data[4];
			LA_data->value_divide = CONVERT_Q16_DIV;

			mutex_unlock(&sensor_data_mutex);
#if DEBUG
			if(atomic_read(&data->trace) & GME_HWM_DEBUG) {
				MAGN_LOG("Hwm get LA-sensor data: %d, %d, %d. divide %d, status %d!\n",
				LA_data->values[0], LA_data->values[1], LA_data->values[2],
				LA_data->value_divide, LA_data->status);
			}
#endif
		}
		break;
	default:
		MAG_ERR("linear_accelration operate function no this parameter %d!\n", command);
		err = -1;
		break;
	}

	return err;
}

#endif

/*----------------------------------------------------------------------------*/
#if 0
static int gme60x_suspend(struct i2c_client *client, pm_message_t msg)
{
	struct gme60x_i2c_data *obj = i2c_get_clientdata(client);

	if (msg.event == PM_EVENT_SUSPEND)
		gme60x_power(obj->hw, 0);

	return 0;
}
/*----------------------------------------------------------------------------*/
static int gme60x_resume(struct i2c_client *client)
{
	struct gme60x_i2c_data *obj = i2c_get_clientdata(client);

	gme60x_power(obj->hw, 1);
	return 0;
}
#else
static int gme60x_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct gme60x_i2c_data *obj = i2c_get_clientdata(client);

	gme60x_power(obj->hw, 0);

	return 0;
}
static int gme60x_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct gme60x_i2c_data *obj = i2c_get_clientdata(client);

	gme60x_power(obj->hw, 1);
	return 0;
}
#endif
/*----------------------------------------------------------------------------*/
static int gme60x_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, GME60X_DEV_NAME);
	return 0;
}

static int gme60x_m_enable(int en)
{
	int value = 0;
	int err = 0;

	value = en;
	factory_mode = 1;
	if (value == 1) {
		atomic_set(&m_flag, 1);
		atomic_set(&open_flag, 1);

		err = AKECS_SetMode(GME_MODE_SNG_MEASURE);
		if (err < 0) {
			MAG_ERR("%s:AKECS_SetMode Error.\n", __func__);
			return err;
		}
	} else {
		atomic_set(&m_flag, 0);
		if (atomic_read(&o_flag) == 0) {
			atomic_set(&open_flag, 0);
			err = AKECS_SetMode(GME_MODE_POWERDOWN);
			if (err < 0) {
				MAG_ERR("%s:AKECS_SetMode Error.\n", __func__);
				return err;
			}
		}
	}
	wake_up(&open_wq);
	return err;
}

static int gme60x_m_set_delay(u64 ns)
{
	int value = 0;

	value = (int)ns/1000/1000;

	if (value <= 10)
		gmad_delay = 10;
	else
		gmad_delay = value;

	return 0;
}
static int gme60x_m_open_report_data(int open)
{
	return 0;
}

static int gme60x_m_get_data(int *x, int *y, int *z, int *status)
{
	mutex_lock(&sensor_data_mutex);
	
	*x = sensor_data[9] * CONVERT_M;
	*y = sensor_data[10] * CONVERT_M;
	*z = sensor_data[11] * CONVERT_M;
	*status = sensor_data[4];
		
	mutex_unlock(&sensor_data_mutex);		
	return 0;
}


static int gme60x_o_enable(int en)
{
	int value = 0;
	int err = 0;

	value = en;

	if (value == 1) {
		atomic_set(&o_flag, 1);
		atomic_set(&open_flag, 1);
		err = AKECS_SetMode(GME_MODE_SNG_MEASURE);
		if (err < 0) {
			MAG_ERR("%s:AKECS_SetMode Error.\n", __func__);
			return err;
		}
	} else {
		atomic_set(&o_flag, 0);
		if (atomic_read(&m_flag) == 0) {
			atomic_set(&open_flag, 0);
			err = AKECS_SetMode(GME_MODE_POWERDOWN);
			if (err < 0) {
				MAG_ERR("%s:AKECS_SetMode Error.\n", __func__);
				return err;
			}
		}
	}
	wake_up(&open_wq);
	return err;
}

static int gme60x_o_set_delay(u64 ns)
{
	int value = 0;

	value = (int)ns/1000/1000;
	if (value <= 10)
		gmad_delay = 10;
	else
		gmad_delay = value;

	return 0;
}
static int gme60x_o_open_report_data(int open)
{
	return 0;
}

static int gme60x_o_get_data(int *x, int *y, int *z, int *status)
{
	mutex_lock(&sensor_data_mutex);

	*x = sensor_data[13] * CONVERT_O;
	*y = sensor_data[14] * CONVERT_O;
	*z = sensor_data[15] * CONVERT_O;
	*status = sensor_data[8];

	mutex_unlock(&sensor_data_mutex);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int gme60x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	struct i2c_client *new_client;
	struct gme60x_i2c_data *data;
	struct mag_control_path ctl = {0};
	struct mag_data_path mag_data = {0};

	MAGN_LOG("gme60x_i2c_probe\n");
	data = kzalloc(sizeof(struct gme60x_i2c_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	data->hw = hw;

/*

obj->hw = hw;

err = hwmsen_get_convert(obj->hw->direction, &obj->cvt);
if (err) {
	GSE_ERR("invalid direction: %d\n", obj->hw->direction);
	goto exit;
}

obj_i2c_data = obj;
obj->client = client;





*/
	atomic_set(&data->layout, data->hw->direction);
	atomic_set(&data->trace, 0);
	mutex_init(&sense_data_mutex);
	mutex_init(&sensor_data_mutex);
	/* init_waitqueue_head(&data_ready_wq); */
	init_waitqueue_head(&open_wq);

	
	//client->addr=0x0d;

	MAGN_LOG("[%s] client->addr S= 0x%02X\n", __func__, client->addr);
	MAGN_LOG("[%s] i2c_addr[0] = 0x%02X\n", __func__, data->hw->i2c_addr[0]);
	MAGN_LOG("[%s] client->addr E= 0x%02X\n", __func__, client->addr);
	
	data->client = client;
	new_client = data->client;
	i2c_set_clientdata(new_client, data);
	this_client = new_client;

	/* Check connection */
	err = GME_CheckDevice();
	if(err < 0){
		MAG_ERR("GME60X gme60x_probe: check device connect error\n");
		goto exit_init_failed;
	}


	/* Register sysfs attribute */
	err = gme60x_create_attr(&(gme60x_init_info.platform_diver_addr->driver));
	if (err) {
		MAG_ERR("create attribute err = %d\n", err);
		goto exit_sysfs_create_group_failed;
	}

	err = misc_register(&gme60x_device);
	if (err) {
		MAG_ERR("gme60x_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	ctl.is_use_common_factory = false;
	ctl.m_enable = gme60x_m_enable;
	ctl.m_set_delay  = gme60x_m_set_delay;
	ctl.m_open_report_data = gme60x_m_open_report_data;
	ctl.o_enable = gme60x_o_enable;
	ctl.o_set_delay  = gme60x_o_set_delay;
	ctl.o_open_report_data = gme60x_o_open_report_data;
	ctl.is_report_input_direct = false;
	ctl.is_support_batch = data->hw->is_batch_supported;

	err = mag_register_control_path(&ctl);
	if (err) {
		MAG_ERR("register mag control path err\n");
		goto exit_kfree;
	}

	mag_data.div_m = CONVERT_M_DIV;
	mag_data.div_o = CONVERT_O_DIV;
	mag_data.get_data_o = gme60x_o_get_data;
	mag_data.get_data_m = gme60x_m_get_data;

	err = mag_register_data_path(&mag_data);
	if (err) {
		MAG_ERR("register data control path err\n");
		goto exit_kfree;
	}

	MAG_ERR("%s: OK\n", __func__);
	ecompass_status = 1;
	return 0;

exit_sysfs_create_group_failed:
exit_init_failed:
exit_misc_device_register_failed:
exit_kfree:
	kfree(data);
exit:
	MAG_ERR("%s: err = %d\n", __func__, err);
	ecompass_status = -1;
	return err;
}
/*----------------------------------------------------------------------------*/
static int gme60x_i2c_remove(struct i2c_client *client)
{
	int err;

	err = gme60x_delete_attr(&(gme60x_init_info.platform_diver_addr->driver));
	if (err)
		MAG_ERR("gme60x_delete_attr fail: %d\n", err);

	this_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	misc_deregister(&gme60x_device);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int gme60x_remove(void)
{
	gme60x_power(hw, 0);
	atomic_set(&dev_open_count, 0);
	i2c_del_driver(&gma60x_i2c_driver);
	return 0;
}

static int gme60x_local_init(void)
{
	gme60x_power(hw, 1);
	MAGN_LOG("[%s] GMC306 Enter\n", __func__); //jonny m 
	if (i2c_add_driver(&gma60x_i2c_driver)) {
		MAG_ERR("i2c_add_driver error\n");
		return -1;
	}

	MAGN_LOG("[%s] ecompass_status = %d\n", __func__, ecompass_status);
	if (ecompass_status == -1)
		return -1;
	return 0;
}

/*----------------------------------------------------------------------------*/
static int __init gme60x_init(void)
{
	const char *name = "mediatek,gme60x";//gmc306

	hw = get_mag_dts_func(name, hw);
	if (!hw)
		MAGN_ERR("get dts info fail\n");

//jonny test  s
#if 1
	MAGN_LOG("[%s] m_pHw->i2c_num = %d\n", __func__, hw->i2c_num);
	MAGN_LOG("[%s] m_pHw->i2c_addr[0] = %x\n", __func__, hw->i2c_addr[0]);
	MAGN_LOG("[%s] m_pHw->i2c_addr[1] = %x\n", __func__, hw->i2c_addr[1]);
	MAGN_LOG("[%s] m_pHw->direction = %d\n", __func__, hw->direction);
	MAGN_LOG("[%s] m_pHw->power_id = %x\n", __func__, hw->power_id);
	MAGN_LOG("[%s] m_pHw->power_vol = %d\n", __func__, hw->power_vol);
	MAGN_ERR("[%s] m_pHw->i2c_num = %d\n", __func__, hw->i2c_num);
	MAGN_ERR("[%s] m_pHw->i2c_addr[0] = %x\n", __func__, hw->i2c_addr[0]);
	MAGN_ERR("[%s] m_pHw->i2c_addr[1] = %x\n", __func__, hw->i2c_addr[1]);
	MAGN_ERR("[%s] m_pHw->direction = %d\n", __func__, hw->direction);
	MAGN_ERR("[%s] m_pHw->power_id = %x\n", __func__, hw->power_id);
	MAGN_ERR("[%s] m_pHw->power_vol = %d\n", __func__, hw->power_vol);
#endif
//jonny test e
	MAGN_ERR("%s of GMC306\n", __func__);
	mag_driver_add(&gme60x_init_info);
	return 0; 
}
/*----------------------------------------------------------------------------*/
static void __exit gme60x_exit(void)
{	
}
/*----------------------------------------------------------------------------*/
module_init(gme60x_init);
module_exit(gme60x_exit);

MODULE_AUTHOR("MTK");
MODULE_DESCRIPTION("GME60x compass driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
