/***************************************************************************
 * Filename:
 * ---------
 *  pn54x.c
 *
 * Project:
 * --------
 *
 * Description:
 * ------------
 *
 * Author:
 * -------
 *  LiangChi Huang, ext 25609, LiangChi.Huang@mediatek.com, 2012-08-09
 *
 *****************************************************************************/

#define pr_fmt(fmt) "["KBUILD_MODNAME"]" fmt
/*****************************************************************************
 * Include
 *****************************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>

#include <linux/nfc/pn547.h>
#include <linux/wakelock.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif

/* #include <mach/eint.h> */
/* #include <mach/pn54x.h> */

#ifndef CONFIG_MTK_FPGA
/* #include <mt_clkbuf_ctl.h>  for clock buffer */
#endif

/* #include <cust_eint.h> */
/* #include <cust_i2c.h>  */

#if defined(CONFIG_MTK_LEGACY)
#include <mach/mt_gpio.h>
#include <cust_gpio_usage.h>
#endif

/*****************************************************************************
 * Define
 *****************************************************************************/

/* #define NFC_I2C_BUSNUM  I2C_NFC_CHANNEL */

#define NFC_DEBUG 0

#define I2C_ID_NAME "pn54x"

#define MAX_BUFFER_SIZE	512

#define NFC_CLIENT_TIMING 400	/* I2C speed */

enum {
	MTK_NFC_GPIO_EN_B = 0x0,
	MTK_NFC_GPIO_SYSRST_B,
	MTK_NFC_GPIO_EINT,
	MTK_NFC_GPIO_IRQ,
	MTK_NFC_GPIO_IOCTL,
	MTK_NFC_GPIO_MAX_NUM
};

enum {
	MTK_NFC_IOCTL_READ = 0x0,
	MTK_NFC_IOCTL_WRITE,
	MTK_NFC_IOCTL_MAX_NUM
};

enum {
	MTK_NFC_IOCTL_CMD_CLOCK_BUF_ENABLE = 0x0,
	MTK_NFC_IOCTL_CMD_CLOCK_BUF_DISABLE,
	MTK_NFC_IOCTL_CMD_EXIT_EINT,
	MTK_NFC_IOCTL_CMD_GET_CHIP_ID,
	MTK_NFC_IOCTL_CMD_READ_DATA,
	MTK_NFC_IOCTL_CMD_MAX_NUM
};

enum {
	MTK_NFC_PULL_LOW = 0x0,
	MTK_NFC_PULL_HIGH,
	MTK_NFC_PULL_INVALID,
};

enum {
	MTK_NFC_GPIO_DIR_IN = 0x0,
	MTK_NFC_GPIO_DIR_OUT,
	MTK_NFC_GPIO_DIR_INVALID,
};

/*****************************************************************************
 * Global Variable
 *****************************************************************************/
struct pn54x_dev *pn54x_dev_ptr = NULL;
struct pn54x_dev _gpn54x_dev;

#if !defined(CONFIG_MTK_LEGACY)
struct platform_device *nfc_plt_dev = NULL;
struct pinctrl *gpctrl = NULL;
struct pinctrl_state *st_ven_h = NULL;
struct pinctrl_state *st_ven_l = NULL;
/*
struct pinctrl_state *st_rst_h = NULL;
struct pinctrl_state *st_rst_l = NULL;
*/
struct pinctrl_state *st_eint_h = NULL;
struct pinctrl_state *st_eint_l = NULL;
struct pinctrl_state *st_irq_init = NULL;
/*
struct pinctrl_state *st_osc_init = NULL;
*/
#endif

/* static struct i2c_board_info nfc_board_info __initdata = */
/*    { I2C_BOARD_INFO(I2C_ID_NAME, I2C_NFC_SLAVE_7_BIT_ADDR) }; */

/* For DMA */
#ifdef CONFIG_MTK_I2C_EXTENSION
static char *I2CDMAWriteBuf;	/*= NULL;*//* unnecessary initialise */
static unsigned int I2CDMAWriteBuf_pa;	/* = NULL; */
static char *I2CDMAReadBuf;	/*= NULL;*//* unnecessary initialise */
static unsigned int I2CDMAReadBuf_pa;	/* = NULL; */
#else
static char I2CDMAWriteBuf[512];
static char I2CDMAReadBuf[512];
#endif
//static int fgNfcChip;		/*= 0;*//* unnecessary initialise */
int forceExitBlockingRead = 0;

/*  NFC IRQ */
static u32 nfc_irq;
static int nfc_irq_count;	/*= 0;*//* unnecessary initialise */

/*****************************************************************************
 * Function Prototype
 *****************************************************************************/
static int pn54x_probe(struct i2c_client *client,
			const struct i2c_device_id *id);
static int pn54x_remove(struct i2c_client *client);
/* static int pn54x_detect(struct i2c_client *client, */
/*                                int kind, struct i2c_board_info *info); */
static int pn54x_dev_open(struct inode *inode, struct file *filp);
static long pn54x_dev_unlocked_ioctl(struct file *filp, unsigned int cmd,
				      unsigned long arg);
static ssize_t pn54x_dev_read(struct file *filp, char __user *buf,
			       size_t count, loff_t *offset);
static ssize_t pn54x_dev_write(struct file *filp, const char __user *buf,
				size_t count, loff_t *offset);

static int mt_nfc_probe(struct platform_device *pdev);
static int mt_nfc_remove(struct platform_device *pdev);

/* void pn54x_dev_irq_handler(void); */
static irqreturn_t pn54x_dev_irq_handler(int irq, void *data);	/*IRQ handler */

/* static void pn54x_disable_irq(struct pn54x_dev *pn54x_dev); */
#if !defined(CONFIG_MTK_LEGACY)
static int mt_nfc_pinctrl_init(struct platform_device *pdev);
static int mt_nfc_pinctrl_select(struct pinctrl *p, struct pinctrl_state *s);
static int mt_nfc_gpio_init(void);
#endif
static int mt_nfc_get_gpio_value(int gpio_num);
static int mt_nfc_get_gpio_dir(int gpio_num);

/*****************************************************************************
 * Data Structure
 *****************************************************************************/

struct pn54x_dev {
	wait_queue_head_t read_wq;
	struct mutex read_mutex;
	struct i2c_client *client;
	struct miscdevice pn54x_device;
	unsigned int ven_gpio;
/*	unsigned int sysrstb_gpio; */
	unsigned int irq_gpio;	/* Chip inform Host */
	unsigned int eint_gpio;	/* Host inform Chip */
	atomic_t irq_enabled;
	struct mutex irq_enabled_lock;
	/* spinlock_t            irq_enabled_lock; */
};

struct pn54x_i2c_platform_data {
	unsigned int irq_gpio;	/* Chip inform Host */
	unsigned int ven_gpio;
/*	unsigned int sysrstb_gpio; */
	unsigned int eint_gpio;	/* Host inform Chip */
/*	unsigned int osc_en; */
};

static const struct i2c_device_id pn54x_id[] = {
	{I2C_ID_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id nfc_switch_of_match[] = {
	{.compatible = "mediatek,nfc"},
	{},
};
#endif

/* #if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)) */
/* static struct i2c_client_address_data addr_data = { .forces = forces,}; */
/* #endif */
static struct i2c_driver pn54x_dev_driver = {
	.id_table = pn54x_id,
	.probe = pn54x_probe,
	.remove = pn54x_remove,
	/* .detect               = pn54x_detect, */
	.driver = {
		   .name = "pn54x",
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = nfc_switch_of_match,
#endif
		   },
};

/*  platform driver */
static const struct of_device_id nfc_dev_of_match[] = {
	{.compatible = "mediatek,nfc-gpio-v2",},
	{},
};

static struct platform_driver mtk_nfc_platform_driver = {
	.probe = mt_nfc_probe,
	.remove = mt_nfc_remove,
	.driver = {
		   .name = I2C_ID_NAME,
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = nfc_dev_of_match,
#endif
		   },
};

#if !defined(CONFIG_MTK_LEGACY)
struct pn54x_i2c_platform_data pn54x_platform_data;
#else
static struct pn54x_i2c_platform_data pn54x_platform_data = {
	.irq_gpio = GPIO_IRQ_NFC_PIN,
	.ven_gpio = GPIO_NFC_VENB_PIN,
/*	.sysrstb_gpio = GPIO_NFC_RST_PIN,*/
	.eint_gpio = GPIO_NFC_EINT_PIN,
};
#endif

static const struct file_operations pn54x_dev_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read = pn54x_dev_read,
	.write = pn54x_dev_write,
	.open = pn54x_dev_open,
#ifdef CONFIG_COMPAT
	.compat_ioctl = pn54x_dev_unlocked_ioctl,
#endif
	.unlocked_ioctl = pn54x_dev_unlocked_ioctl,
};

/*****************************************************************************
 * Extern Area
 *****************************************************************************/

/*****************************************************************************
 * Function
 *****************************************************************************/
static void pn54x_enable_irq(u32 irq_line)
{
	/* pr_debug("%s : irq_line=%d, nfc_irq_count=%d.\n", __func__,
		 irq_line, nfc_irq_count); */

	nfc_irq_count++;	/* It must set before call enable_irq */

	enable_irq(irq_line);

	/* pr_debug("%s : nfc_irq_count = %d.\n", __func__, nfc_irq_count); */
}

static void pn54x_disable_irq(u32 irq_line)
{
	/* pr_debug("%s : irq_line=%d, nfc_irq_count=%d.\n", __func__,
		 irq_line, nfc_irq_count); */
	if (nfc_irq_count >= 1) {
		disable_irq_nosync(irq_line);
		nfc_irq_count--;
	} else {
		pr_debug("%s : disable irq fail.\n", __func__);
	}
	/* pr_debug("%s : nfc_irq_count = %d.\n", __func__, nfc_irq_count); */
}

static int pn54x_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ret = 0;
	struct pn54x_i2c_platform_data *platform_data;
	struct device_node *node;

	pr_debug("pn54x_dev_probe\n");

	platform_data = &pn54x_platform_data;

	if (platform_data == NULL) {
		pr_err("%s : nfc probe fail\n", __func__);
		return -ENODEV;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s : need I2C_FUNC_I2C\n", __func__);
		return -ENODEV;
	}
#if defined(CONFIG_MTK_LEGACY)
	mt_set_gpio_mode(platform_data->irq_gpio, GPIO_IRQ_NFC_PIN_M_GPIO);
	mt_set_gpio_dir(platform_data->irq_gpio, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(platform_data->irq_gpio, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(platform_data->irq_gpio, GPIO_PULL_DOWN);

	/* ven_gpio */
	mt_set_gpio_mode(platform_data->ven_gpio, GPIO_NFC_VENB_PIN_M_GPIO);
	mt_set_gpio_dir(platform_data->ven_gpio, GPIO_DIR_OUT);
	mt_set_gpio_out(platform_data->ven_gpio, GPIO_OUT_ONE);
	usleep_range(900, 1000);	/* debug */

	/* firm_gpio */

/*
	mt_set_gpio_mode(platform_data->sysrstb_gpio, GPIO_NFC_RST_PIN_M_GPIO);
	mt_set_gpio_dir(platform_data->sysrstb_gpio, GPIO_DIR_OUT);
	mt_set_gpio_out(platform_data->sysrstb_gpio, GPIO_OUT_ONE);
	usleep_range(900, 1000);
*/
	/* EINT_gpio */
	mt_set_gpio_mode(platform_data->eint_gpio, GPIO_NFC_EINT_PIN_M_GPIO);
	mt_set_gpio_dir(platform_data->eint_gpio, GPIO_DIR_OUT);
	/* Set output High */
	mt_set_gpio_out(platform_data->eint_gpio, GPIO_OUT_ZERO);

	_gpn54x_dev.irq_gpio = platform_data->irq_gpio;
	_gpn54x_dev.ven_gpio = platform_data->ven_gpio;
/*	_gpn54x_dev.sysrstb_gpio = platform_data->sysrstb_gpio;*/
	_gpn54x_dev.eint_gpio = platform_data->eint_gpio;
#endif

	_gpn54x_dev.client = client;
	/* init mutex and queues */
	init_waitqueue_head(&_gpn54x_dev.read_wq);
	mutex_init(&_gpn54x_dev.read_mutex);
	/* spin_lock_init(&pn54x_dev->irq_enabled_lock); */
	mutex_init(&_gpn54x_dev.irq_enabled_lock);

#if 0
	_gpn54x_dev.pn54x_device.minor = MISC_DYNAMIC_MINOR;
	_gpn54x_dev.pn54x_device.name = "pn54x";
	_gpn54x_dev.pn54x_device.fops = &pn54x_dev_fops;

	ret = misc_register(&_gpn54x_dev.pn54x_device);
	if (ret) {
		pr_err("%s : misc_register failed\n", __FILE__);
		goto err_misc_register;
	}
#endif

#ifdef CONFIG_MTK_I2C_EXTENSION
#ifdef CONFIG_64BIT
	I2CDMAWriteBuf =
	    (char *)dma_alloc_coherent(&client->dev, MAX_BUFFER_SIZE,
				       (dma_addr_t *) &I2CDMAWriteBuf_pa,
				       GFP_KERNEL);
#else
	I2CDMAWriteBuf =
	    (char *)dma_alloc_coherent(NULL, MAX_BUFFER_SIZE,
				       (dma_addr_t *) &I2CDMAWriteBuf_pa,
				       GFP_KERNEL);
#endif

	if (I2CDMAWriteBuf == NULL) {
		pr_err("%s : failed to allocate dma buffer\n", __func__);
		mutex_destroy(&_gpn54x_dev.read_mutex);
/*		gpio_free(platform_data->sysrstb_gpio);*/
		return ret;
	}
#ifdef CONFIG_64BIT
	I2CDMAReadBuf =
	    (char *)dma_alloc_coherent(&client->dev, MAX_BUFFER_SIZE,
				       (dma_addr_t *) &I2CDMAReadBuf_pa,
				       GFP_KERNEL);
#else
	I2CDMAReadBuf =
	    (char *)dma_alloc_coherent(NULL, MAX_BUFFER_SIZE,
				       (dma_addr_t *) &I2CDMAReadBuf_pa,
				       GFP_KERNEL);
#endif

	if (I2CDMAReadBuf == NULL) {
		pr_err("%s : failed to allocate dma buffer\n", __func__);
		mutex_destroy(&_gpn54x_dev.read_mutex);
/*		gpio_free(platform_data->sysrstb_gpio);*/
		return ret;
	}
	pr_debug("%s :I2CDMAWriteBuf_pa %d, I2CDMAReadBuf_pa,%d\n", __func__,
		 I2CDMAWriteBuf_pa, I2CDMAReadBuf_pa);
#else
	memset(I2CDMAWriteBuf, 0x00, sizeof(I2CDMAWriteBuf));
	memset(I2CDMAReadBuf, 0x00, sizeof(I2CDMAReadBuf));
#endif
	/* request irq.  the irq is set whenever the chip has data available
	 * for reading.  it is cleared when all data has been read.
	 */

	/* client->irq = CUST_EINT_IRQ_NFC_NUM; */
	/* pr_debug("%s : requesting IRQ %d\n", __func__, client->irq);  */
	/* pr_debug("pn54x_Prob2\n");     */
	/* mt_eint_set_hw_debounce(CUST_EINT_IRQ_NFC_NUM, */
	/*                              CUST_EINT_IRQ_NFC_DEBOUNCE_CN); */
	/* mt_eint_registration(CUST_EINT_IRQ_NFC_NUM,  */
	/*                   CUST_EINT_IRQ_NFC_TYPE, pn54x_dev_irq_handler, */
	/*                   0); */

	/*  NFC IRQ settings     */
	node = of_find_compatible_node(NULL, NULL, "mediatek,nfc-gpio-v2");

	if (node) {

		nfc_irq = irq_of_parse_and_map(node, 0);

		client->irq = nfc_irq;

		ret =
		    request_irq(nfc_irq, pn54x_dev_irq_handler,
				IRQF_TRIGGER_NONE, "irq_nfc-eint", NULL);

		if (ret) {

			pr_err("%s : EINT IRQ LINE NOT AVAILABLE, ret = %d\n", __func__, ret);

		} else {

			pr_debug("%s : set EINT finished, nfc_irq=%d", __func__,
				 nfc_irq);
			nfc_irq_count++;
			pn54x_disable_irq(nfc_irq);
		}

	} else {
		pr_err("%s : can not find NFC eint compatible node\n",
		       __func__);
	}

	/* mt_eint_unmask(CUST_EINT_IRQ_NFC_NUM); */
	/* pn54x_disable_irq(pn54x_dev); */
	/* mt_eint_mask(CUST_EINT_IRQ_NFC_NUM); */

	i2c_set_clientdata(client, &_gpn54x_dev);

	forceExitBlockingRead = 0;

	return 0;
}

static int pn54x_remove(struct i2c_client *client)
{
	/* struct pn54x_dev *pn54x_dev; */

	pr_debug("pn54x_remove\n");

#ifdef CONFIG_MTK_I2C_EXTENSION
	if (I2CDMAWriteBuf) {
		#ifdef CONFIG_64BIT
		dma_free_coherent(&client->dev, MAX_BUFFER_SIZE, I2CDMAWriteBuf,
				  I2CDMAWriteBuf_pa);
		#else
		dma_free_coherent(NULL, MAX_BUFFER_SIZE, I2CDMAWriteBuf,
				  I2CDMAWriteBuf_pa);
		#endif
		I2CDMAWriteBuf = NULL;
		I2CDMAWriteBuf_pa = 0;
	}

	if (I2CDMAReadBuf) {
		#ifdef CONFIG_64BIT
		dma_free_coherent(&client->dev, MAX_BUFFER_SIZE, I2CDMAReadBuf,
				  I2CDMAReadBuf_pa);
		#else
		dma_free_coherent(NULL, MAX_BUFFER_SIZE, I2CDMAReadBuf,
				  I2CDMAReadBuf_pa);
		#endif
		I2CDMAReadBuf = NULL;
		I2CDMAReadBuf_pa = 0;
	}
#endif

	/* pn54x_dev = i2c_get_clientdata(client); */
	/* free_irq(client->irq, &_gpn54x_dev);  */
	free_irq(nfc_irq, &_gpn54x_dev);
	misc_deregister(&_gpn54x_dev.pn54x_device);
	mutex_destroy(&_gpn54x_dev.read_mutex);

#if defined(CONFIG_MTK_LEGACY)
	gpio_free(_gpn54x_dev.irq_gpio);
	gpio_free(_gpn54x_dev.ven_gpio);
/*	gpio_free(_gpn54x_dev.sysrstb_gpio);*/
	gpio_free(_gpn54x_dev.eint_gpio);
#endif

	/* kfree(pn54x_dev); */

	return 0;
}

static int mt_nfc_probe(struct platform_device *pdev)
{
	int ret = 0;

#if !defined(CONFIG_MTK_LEGACY)

	nfc_plt_dev = pdev;

	pr_debug("%s : &nfc_plt_dev=%p\n", __func__, nfc_plt_dev);

	/* pinctrl init */
	ret = mt_nfc_pinctrl_init(pdev);

	/* gpio init */
	if (mt_nfc_gpio_init() != 0)
		pr_debug("%s : mt_nfc_gpio_init err.\n", __func__);

#endif

	return 0;
}

static int mt_nfc_remove(struct platform_device *pdev)
{
	pr_debug("%s : &pdev=%p\n", __func__, pdev);
	return 0;
}

/* void pn54x_dev_irq_handler(void) */
irqreturn_t pn54x_dev_irq_handler(int irq, void *data)
{
	struct pn54x_dev *pn54x_dev = pn54x_dev_ptr;
	/* pr_debug("%s : &pn54x_dev=%p\n", __func__, pn54x_dev); */

	if (pn54x_dev == NULL) {
		pr_debug("pn54x_dev NULL.\n");
		return IRQ_HANDLED;
	}
	/* pn54x_disable_irq(pn54x_dev); */
	pn54x_disable_irq(nfc_irq);

	wake_up(&pn54x_dev->read_wq);
	/* wake_up_interruptible(&pn54x_dev->read_wq); */

	#if NFC_DEBUG
	pr_info("%s, IRQ_HANDLED\n", __func__);
	#endif
		
	/* pr_debug("%s : wake_up &read_wq=%p\n", __func__, &pn54x_dev->read_wq); */

	return IRQ_HANDLED;
}

static ssize_t pn54x_dev_read(struct file *filp, char __user *buf,
			       size_t count, loff_t *offset)
{
	struct pn54x_dev *pn54x_dev = filp->private_data;
	int ret = 0;
	int read_retry = 5;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

#if NFC_DEBUG
    dev_info(&pn54x_dev->client->dev, "pn54x : + r\n");
#endif
		
	if (!mt_nfc_get_gpio_value(pn54x_dev->irq_gpio)) {
		if (filp->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			pr_debug("%s : goto fail.\n", __func__);
			goto fail;
		}
		/* mutex_lock(&pn54x_dev->irq_enabled_lock); */
		/* pn54x_dev->irq_enabled = true; */
		/* mutex_unlock(&pn54x_dev->irq_enabled_lock); */

		mutex_lock(&pn54x_dev->read_mutex);
		if (forceExitBlockingRead == 1) {
			pr_debug("%s :forceExitBlockingRead.\n", __func__);
			forceExitBlockingRead = 0;	/* clear flag */
			mutex_unlock(&pn54x_dev->read_mutex);
			goto fail;
		}
		mutex_unlock(&pn54x_dev->read_mutex);

		/* mt_eint_unmask(pn54x_dev->client->irq);     */
		/* pr_debug("%s : mt_eint_unmask %d, IRQ, %d\n", */
		/*                __func__,   */
		/*                pn54x_dev->client->irq, */
		/*                mt_get_gpio_in(pn54x_dev->irq_gpio)); */

		pn54x_enable_irq(nfc_irq);

#if NFC_DEBUG
		pr_debug("%s : enable_irq %d, irq status=%d\n",
			 __func__,
			 nfc_irq, mt_nfc_get_gpio_value(pn54x_dev->irq_gpio));
#endif

		ret = wait_event_interruptible(pn54x_dev->read_wq,
					       (mt_nfc_get_gpio_value
						(pn54x_dev->irq_gpio) 
						|| forceExitBlockingRead));

		/*pr_debug("%s : wait_event_interruptible ret=%d,irq status=%d\n",
			 __func__, ret,
			 mt_nfc_get_gpio_value(pn54x_dev->irq_gpio));*/

		if (ret || forceExitBlockingRead) {
			pn54x_disable_irq(nfc_irq);
			pr_debug("%s : goto fail\n", __func__);
			mutex_lock(&pn54x_dev->read_mutex);
			if (forceExitBlockingRead == 1) {
				pr_debug
				    ("%s:clear flag,orceExitBlockingRead\n",
				     __func__);
				forceExitBlockingRead = 0;	/* clear flag */
			}
			mutex_unlock(&pn54x_dev->read_mutex);
			goto fail;
		}

	}
	#ifdef CONFIG_MTK_I2C_EXTENSION
	pn54x_dev->client->addr = (pn54x_dev->client->addr & I2C_MASK_FLAG);
	pn54x_dev->client->ext_flag |= I2C_DMA_FLAG;
	/* pn54x_dev->client->ext_flag |= I2C_DIRECTION_FLAG; */
	/* pn54x_dev->client->ext_flag |= I2C_A_FILTER_MSG; */
	pn54x_dev->client->timing = NFC_CLIENT_TIMING;

	/* Read data */
	ret =
	    i2c_master_recv(pn54x_dev->client,
			    (unsigned char *)(uintptr_t) I2CDMAReadBuf_pa,
			    count);

	#else
	while (read_retry) {
		ret =
		    i2c_master_recv(pn54x_dev->client,
				    (unsigned char *)(uintptr_t) I2CDMAReadBuf,
				    count);

		/* mutex_unlock(&pn54x_dev->read_mutex); */

		/*pr_debug("%s : i2c_master_recv returned=%d, irq status=%d\n", __func__,
			 ret, mt_nfc_get_gpio_value(pn54x_dev->irq_gpio));*/

		if (ret < 0) {
			pr_debug("%s: i2c_master_recv failed: %d, read_retry: %d\n",
				__func__, ret, read_retry);
			read_retry--;
			usleep_range(900, 1000);
			continue;
		}
		break;
	}
	#endif
	if (ret < 0) {
		pr_err("%s: i2c_master_recv failed: %d, read_retry: %d\n",
			__func__, ret, read_retry);
		return ret;
	}

	if (ret > count) {
		pr_debug("%s: received too many bytes from i2c (%d)\n",
			 __func__, ret);
		return -EIO;
	}

	if (copy_to_user(buf, I2CDMAReadBuf, ret)) {
		pr_debug("%s : failed to copy to user space\n", __func__);
		return -EFAULT;
	}

	/* pr_debug("%s: return,ret,%d\n", __func__, ret); */
	
#if NFC_DEBUG
    if(ret > 0)
    {
        int i;
        for (i = 0; i < ret; i++)
            dev_info(&pn54x_dev->client->dev, "Read_buf[%d] = 0x%x\n",	i, I2CDMAReadBuf[i]);
    }
    dev_info(&pn54x_dev->client->dev, "pn54x : - r\n");
#endif

	return ret;

fail:
	/* mutex_unlock(&pn54x_dev->read_mutex); */
	pr_debug("%s: return, fail: %d\n", __func__, ret);
	return ret;

}

static ssize_t pn54x_dev_write(struct file *filp, const char __user *buf,
				size_t count, loff_t *offset)
{
	struct pn54x_dev *pn54x_dev;
	int ret = 0, ret_tmp = 0, count_ori = 0, count_remain = 0, idx = 0;

#if NFC_DEBUG
    int i;
//    char *tmp;
#endif

	pn54x_dev = filp->private_data;
	count_ori = count;
	count_remain = count_ori;

#if NFC_DEBUG
    dev_info(&pn54x_dev->client->dev, "pn54x : + w\n");
    for (i = 0; i < count; i++)
        dev_info(&pn54x_dev->client->dev, "buf[%d] = 0x%x\n",	i, buf[i]);
#endif

        
	if (count > MAX_BUFFER_SIZE) {
		count = MAX_BUFFER_SIZE;
		count_remain -= count;
	}
    
	while (1) {
		if (copy_from_user
		    (I2CDMAWriteBuf, &buf[(idx * MAX_BUFFER_SIZE)], count)) {
			pr_err("%s : failed to copy from user space.\n",
				 __func__);
			return -EFAULT;
		}

		/*pr_debug("%s : writing %zu bytes, remain bytes %d.\n", __func__,
			 count, count_remain); */

		/* Write data */
		#ifdef CONFIG_MTK_I2C_EXTENSION
		pn54x_dev->client->addr =
		    (pn54x_dev->client->addr & I2C_MASK_FLAG);

		pn54x_dev->client->ext_flag |= I2C_DMA_FLAG;
		/* pn54x_dev->client->ext_flag |= I2C_DIRECTION_FLAG; */
		/* pn54x_dev->client->ext_flag |= I2C_A_FILTER_MSG; */
		pn54x_dev->client->timing = NFC_CLIENT_TIMING;

		ret_tmp =
		    i2c_master_send(pn54x_dev->client,
				    (unsigned char *)(uintptr_t)
				    I2CDMAWriteBuf_pa, count);
		#else
		ret_tmp =
		    i2c_master_send(pn54x_dev->client,
				    (unsigned char *)(uintptr_t)
				    I2CDMAWriteBuf, count);
		#endif

		if (ret_tmp != count) {
			pr_debug("%s : i2c_master_send returned %d\n", __func__,
				 ret);
			ret = -EIO;
			return ret;
		}

		ret += ret_tmp;
		/* pr_debug("%s : ret_tmp=%d,ret=%d,count_ori=%d\n", __func__,
		   ret_tmp, ret, count_ori); */

		if (ret == count_ori) {
			/* pr_debug("%s : ret == count_ori\n", __func__); */
			dev_info(&pn54x_dev->client->dev, "ret == count_ori");
			break;
		}

			if (count_remain > MAX_BUFFER_SIZE) {
				count = MAX_BUFFER_SIZE;
				count_remain -= MAX_BUFFER_SIZE;
			} else {
				count = count_remain;
				count_remain = 0;
			}
			idx++;
		}

#if NFC_DEBUG
	pr_debug("%s : writing %d bytes. Status %d\n", __func__, count_ori,
		 ret);
		 
	dev_info(&pn54x_dev->client->dev, "pn54x : - w\n");
#endif
		 
	return ret;
}

static int pn54x_dev_open(struct inode *inode, struct file *filp)
{

	struct pn54x_dev *pn54x_dev =
	    container_of(filp->private_data, struct pn54x_dev, pn54x_device);

	filp->private_data = pn54x_dev;
	pn54x_dev_ptr = pn54x_dev;

	pr_debug("pn54x_dev_open,%s : %d,%d, &pn54x_dev_open=%p\n", __func__,
		 imajor(inode), iminor(inode), pn54x_dev_ptr);

	forceExitBlockingRead = 0;
	return 0;
}

static long pn54x_dev_unlocked_ioctl(struct file *filp, unsigned int cmd,
				      unsigned long arg)
{

    struct pn54x_dev *pn54x_dev = filp->private_data;
    int ret = 0;//[VY36 M] 
		
    switch (cmd) {
        case PN547_SET_PWR:
            if (arg == 2) {
                /* power on with firmware download (requires hw reset) */
                									
                ret = mt_nfc_pinctrl_select(gpctrl, st_ven_h);
                ret = mt_nfc_pinctrl_select(gpctrl, st_eint_h);		
                usleep_range(10000, 10000);
                ret = mt_nfc_pinctrl_select(gpctrl, st_ven_l);
                usleep_range(10000, 10000);
                ret = mt_nfc_pinctrl_select(gpctrl, st_ven_h);
                usleep_range(10000, 10000);
                /*
                if (atomic_read(&pn54x_dev->irq_enabled) == 0) {
                		atomic_set(&pn54x_dev->irq_enabled, 1);
                		pn54x_enable_irq(nfc_irq);
                }*/								
					
                dev_info(&pn54x_dev->client->dev,	"%s power on with firmware, irq=%d\n", __func__, atomic_read(&pn54x_dev->irq_enabled));
    						
    		}
            else if (arg == 1)
            {
                /* power on */

                ret = mt_nfc_pinctrl_select(gpctrl, st_eint_l);		
                ret = mt_nfc_pinctrl_select(gpctrl, st_ven_h);
                usleep_range(10000, 10000);
                /*
                if (atomic_read(&pn54x_dev->irq_enabled) == 0) {
                		atomic_set(&pn54x_dev->irq_enabled, 1);
                		pn54x_enable_irq(nfc_irq);
                }*/									

                dev_info(&pn54x_dev->client->dev, "%s power on, irq=%d\n", __func__, atomic_read(&pn54x_dev->irq_enabled));
            } 
            else if (arg == 0)
            {
                /* power off */
                /*
                if (atomic_read(&pn54x_dev->irq_enabled) == 1) {
                		pn54x_disable_irq(nfc_irq);
                		atomic_set(&pn54x_dev->irq_enabled, 0);
                }*/
                dev_info(&pn54x_dev->client->dev, "%s power off, irq=%d\n", __func__, atomic_read(&pn54x_dev->irq_enabled));
                ret = mt_nfc_pinctrl_select(gpctrl, st_eint_l);		
                ret = mt_nfc_pinctrl_select(gpctrl, st_ven_l);								
            }
#ifdef NXP_KR_READ_IRQ_MODIFY
            else if (arg == 3) {
        		pr_info("%s Read Cancle\n", __func__);
        		cancle_read = true;
        		do_reading = true;
        		wake_up(&pn54x_dev->read_wq);
            }
#endif            
            else
            {
        		dev_err(&pn54x_dev->client->dev, "%s bad arg %lu\n", __func__, arg);
        		return -EINVAL;
			}
			return ret;/*[VY36 M] JackBB 2015/10/12 Kernel standardization */

        break;
        default:
            dev_err(&pn54x_dev->client->dev, "%s bad ioctl %u\n", __func__,	cmd);
            return -EINVAL;
	}
	return 0;
}

#if !defined(CONFIG_MTK_LEGACY)
static int mt_nfc_pinctrl_select(struct pinctrl *p, struct pinctrl_state *s)
{
	int ret = 0;

	if (p != NULL && s != NULL) {
		ret = pinctrl_select_state(p, s);
	} else {
		pr_debug("%s : pinctrl_select err\n", __func__);
		ret = -1;
	}
	return ret;
}

static int mt_nfc_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;

	gpctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(gpctrl)) {
		dev_err(&pdev->dev, "Cannot find pinctrl!");
		ret = PTR_ERR(gpctrl);
		goto end;
	}

	st_ven_h = pinctrl_lookup_state(gpctrl, "ven_high");
	if (IS_ERR(st_ven_h)) {
		ret = PTR_ERR(st_ven_h);
		pr_debug("%s : pinctrl err, ven_high\n", __func__);
	}
	if (st_ven_h == NULL) {
		pr_err("%s : st_ven_h is NULL\n", __func__);
		goto end;
	}

	st_ven_l = pinctrl_lookup_state(gpctrl, "ven_low");
	if (IS_ERR(st_ven_l)) {
		ret = PTR_ERR(st_ven_l);
		pr_debug("%s : pinctrl err, ven_low\n", __func__);
	}
	if (st_ven_l == NULL) {
		pr_err("%s : st_ven_l is NULL\n", __func__);
		goto end;
	}
/*
	st_rst_h = pinctrl_lookup_state(gpctrl, "rst_high");
	if (IS_ERR(st_rst_h)) {
		ret = PTR_ERR(st_rst_h);
		pr_debug("%s : pinctrl err, rst_high\n", __func__);
	}
	if (st_rst_h == NULL) {
		pr_err("%s : st_rst_h is NULL\n", __func__);
		goto end;
	}

	st_rst_l = pinctrl_lookup_state(gpctrl, "rst_low");
	if (IS_ERR(st_rst_l)) {
		ret = PTR_ERR(st_rst_l);
		pr_debug("%s : pinctrl err, rst_low\n", __func__);
	}
	if (st_rst_l == NULL) {
		pr_err("%s : st_rst_l is NULL\n", __func__);
		goto end;
	}
*/
	st_eint_h = pinctrl_lookup_state(gpctrl, "eint_high");
	if (IS_ERR(st_eint_h)) {
		ret = PTR_ERR(st_eint_h);
		pr_debug("%s : pinctrl err, eint_high\n", __func__);
	}
	if (st_eint_h == NULL) {
		pr_err("%s : st_eint_h is NULL\n", __func__);
		goto end;
	}

	st_eint_l = pinctrl_lookup_state(gpctrl, "eint_low");
	if (IS_ERR(st_eint_l)) {
		ret = PTR_ERR(st_eint_l);
		pr_debug("%s : pinctrl err, eint_low\n", __func__);
	}
	if (st_eint_l == NULL) {
		pr_err("%s : st_eint_l is NULL\n", __func__);
		goto end;
	}

	st_irq_init = pinctrl_lookup_state(gpctrl, "irq_init");
	if (IS_ERR(st_irq_init)) {
		ret = PTR_ERR(st_irq_init);
		pr_debug("%s : pinctrl err, irq_init\n", __func__);
	}
	if (st_irq_init == NULL) {
		pr_err("%s : st_irq_init is NULL\n", __func__);
		goto end;
	}
#if 0
	st_osc_init = pinctrl_lookup_state(gpctrl, "osc_init");
	if (IS_ERR(st_osc_init)) {
		ret = PTR_ERR(st_osc_init);
		pr_debug("%s : pinctrl err, osc_init\n", __func__);
	}
	if (st_osc_init == NULL) {
		pr_err("%s : st_osc_init is NULL\n", __func__);
		goto end;
	}
#endif
	/* select state */
#if 0
	ret = mt_nfc_pinctrl_select(gpctrl, st_osc_init);
	usleep_range(900, 1000);
#endif
	ret = mt_nfc_pinctrl_select(gpctrl, st_irq_init);
	usleep_range(900, 1000);

	ret = mt_nfc_pinctrl_select(gpctrl, st_ven_h);
	usleep_range(900, 1000);
/*
	ret = mt_nfc_pinctrl_select(gpctrl, st_rst_h);
	usleep_range(900, 1000);
*/
	ret = mt_nfc_pinctrl_select(gpctrl, st_eint_l);

end:

	return ret;
}

static int mt_nfc_gpio_init(void)
{

	struct device_node *node;

	node = of_find_compatible_node(NULL, NULL, "mediatek,nfc-gpio-v2");

	if (node) {
		pn54x_platform_data.ven_gpio = of_get_named_gpio(node, "gpio-ven", 0);
/*		pn54x_platform_data.sysrstb_gpio = of_get_named_gpio(node, "gpio-rst", 0);*/
		pn54x_platform_data.eint_gpio = of_get_named_gpio(node, "gpio-eint", 0);
		pn54x_platform_data.irq_gpio = of_get_named_gpio(node, "gpio-irq", 0);
	} else {
		pr_debug("%s : get gpio num err.\n", __func__);
		return -1;
	}

	_gpn54x_dev.irq_gpio = pn54x_platform_data.irq_gpio;
	_gpn54x_dev.ven_gpio = pn54x_platform_data.ven_gpio;
/*	_gpn54x_dev.sysrstb_gpio = pn54x_platform_data.sysrstb_gpio;*/
	_gpn54x_dev.eint_gpio = pn54x_platform_data.eint_gpio;

	return 0;
}
#endif

static int mt_nfc_get_gpio_value(int gpio_num)
{
	int value = 0;

    pr_info("%s : gpio_num = %d \n", __func__ , gpio_num);//BBTEST

	if (mt_nfc_get_gpio_dir(gpio_num) != MTK_NFC_GPIO_DIR_INVALID) {
#if !defined(CONFIG_MTK_LEGACY)
		value = __gpio_get_value(gpio_num);
#else
		value = mt_get_gpio_in(gpio_num);
#endif
	}

	return value;
}

static int mt_nfc_get_gpio_dir(int gpio_num)
{
	if (gpio_num == pn54x_platform_data.irq_gpio) {
		return MTK_NFC_GPIO_DIR_IN;	/* input */

	} else if ((gpio_num == pn54x_platform_data.ven_gpio) ||
/*		   gpio_num == pn54x_platform_data.sysrstb_gpio) || */
		   (gpio_num == pn54x_platform_data.eint_gpio)) {
		return MTK_NFC_GPIO_DIR_OUT;	/* output */

	} else {
		return MTK_NFC_GPIO_DIR_INVALID;

	}
}

/*
 * module load/unload record keeping
 */
static int __init pn54x_dev_init(void)
{
	int ret;

	pr_debug("pn54x_dev_init\n");
	/*  i2c_register_board_info(NFC_I2C_BUSNUM, &nfc_board_info, 1); */
	/*  pr_debug("pn54x_dev_init2\n"); */

	platform_driver_register(&mtk_nfc_platform_driver);

	i2c_add_driver(&pn54x_dev_driver);

	_gpn54x_dev.pn54x_device.minor = MISC_DYNAMIC_MINOR;
	_gpn54x_dev.pn54x_device.name = "pn54x";
	_gpn54x_dev.pn54x_device.fops = &pn54x_dev_fops;

	ret = misc_register(&_gpn54x_dev.pn54x_device);
	if (ret) {
		pr_debug("%s : misc_register failed\n", __FILE__);
		return ret;
	}

	pr_debug("pn54x_dev_init success\n");
	return 0;
}

static void __exit pn54x_dev_exit(void)
{
	pr_debug("pn54x_dev_exit\n");

	i2c_del_driver(&pn54x_dev_driver);
}

module_init(pn54x_dev_init);
module_exit(pn54x_dev_exit);

MODULE_AUTHOR("LiangChi Huang");
MODULE_DESCRIPTION("MTK NFC driver");
MODULE_LICENSE("GPL");
