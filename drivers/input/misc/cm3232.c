/* drivers/input/misc/cm3232.c - cm3232 optical sensors driver
 *
 * Copyright (C) 2017 Vishay Capella Microsystems Limited
 * Author: Frank Hsieh <Frank.Hsieh@vishay.com>
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

#include <linux/delay.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/miscdevice.h>
#include <linux/lightsensor.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
//#include <asm/mach-types.h>
#include <linux/cm3232.h>
#include <asm/setup.h>
#include <linux/jiffies.h>

#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif

#define D(x...) pr_info(x)

#define I2C_RETRY_COUNT 10

#define CALIBRATION_FILE_PATH	"/persist/sensors/als_fac"

#define LS_POLLING_DELAY 600

#define CONTROL_ALS                   0x01
#define CONTROL_ALS_REPORT            0x03

static void report_do_work(struct work_struct *w);
static DECLARE_DELAYED_WORK(report_work, report_do_work);

struct cm3232_info {
	struct class *cm3232_class;
	struct device *ls_dev;

	struct input_dev *ls_input_dev;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	struct i2c_client *i2c_client;
  
  struct workqueue_struct *lp_als_wq;

	int als_enable;
  int als_polling_delay;

#ifdef CONFIG_PM_SLEEP
	int als_enabled_before_suspend;
#endif
	
	int (*power)(int, uint8_t); /* power to the chip */

	uint32_t cal_data; // represented using a fixed 10(-5) notation
	uint32_t cal_data_temp[3];

	int lightsensor_opened;
	uint8_t slave_addr;

	uint32_t current_lux;
	uint16_t current_adc;

	uint16_t ls_cmd;
};
struct cm3232_info *lp_info;
bool cal_data_retrieved = false;
static struct mutex als_enable_mutex, als_disable_mutex, als_get_adc_mutex;
static struct mutex CM3232_control_mutex;
static int lightsensor_enable(struct cm3232_info *lpi);
static int lightsensor_disable(struct cm3232_info *lpi);

static int control_and_report(struct cm3232_info *lpi, uint8_t mode, uint16_t param);

static uint16_t cm3232_ls_adc;

static int I2C_RxData(uint16_t slaveAddr, uint8_t *rxData, int length)
{
	uint8_t loop_i;
	struct cm3232_info *lpi = lp_info;
  uint8_t subaddr = CM3232_ALS_DATA; 
		
	struct i2c_msg msgs[] = {
		{
		 .addr = slaveAddr,
		 .flags = 0,
		 .len = 1,
		 .buf = &subaddr,
		},
		{
		 .addr = slaveAddr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = rxData,
		},		 
	};

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(lpi->i2c_client->adapter, msgs, 2) > 0)
			break;

		if (loop_i == 0 || loop_i == I2C_RETRY_COUNT -1)
			D("[PS][CM3232 error] %s, i2c err, slaveAddr 0x%x \n",
				__func__, slaveAddr);

		msleep(10);
	}
	if (loop_i >= I2C_RETRY_COUNT) {
		printk(KERN_ERR "[PS_ERR][CM3232 error] %s retry over %d\n",
			__func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int I2C_TxData(uint16_t slaveAddr, uint8_t *txData, int length)
{
	uint8_t loop_i;
	struct cm3232_info *lpi = lp_info;
	struct i2c_msg msg[] = {
		{
		 .addr = slaveAddr,
		 .flags = 0,
		 .len = length,
		 .buf = txData,
		},
	};

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(lpi->i2c_client->adapter, msg, 1) > 0)
			break;

		if (loop_i == 0 || loop_i == I2C_RETRY_COUNT -1)
			D("[PS][CM3232 error] %s, i2c err, slaveAddr 0x%x, value 0x%x\n",
				__func__, slaveAddr, txData[0]);

		msleep(10);
	}

	if (loop_i >= I2C_RETRY_COUNT) {
		printk(KERN_ERR "[ALS+PS_ERR][CM3232 error] %s retry over %d\n",
			__func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int _cm3232_I2C_Read_Word(uint16_t slaveAddr, uint8_t cmd, uint16_t *pdata)
{
	int ret = 0;
	uint8_t buffer[2];

	if (pdata == NULL)
		return -EFAULT;

	ret = I2C_RxData(slaveAddr, buffer, 2);
	if (ret < 0) {
		pr_err(
			"[ALS+PS_ERR][CM3232 error]%s: I2C_RxData fail slave addr: 0x%x\n",
			__func__, slaveAddr);
		return ret;
	}

	*pdata = (buffer[1]<<8)|buffer[0];
#if 0
	/* Debug use */
	printk(KERN_DEBUG "[CM3232] %s: I2C_RxData slave addr: 0x%x = 0x%x\n",
		__func__, slaveAddr, *pdata);
#endif
	return ret;
}

static int _cm3232_I2C_Write_Byte(uint16_t SlaveAddress, uint8_t data)
{
	int ret = 0;
	char buffer[2];

	buffer[0] = CM3232_ALS_CMD; 
	buffer[1] = data;
	
	ret = I2C_TxData(SlaveAddress, buffer, 2);
	if (ret < 0) {
		pr_err("[ALS+PS_ERR][CM3232 error]%s: I2C_TxData fail\n", __func__);
		return -EIO;
	}

	return ret;
}

static void read_ls_adc_value(void)
{
	struct cm3232_info *lpi = lp_info;

  mutex_lock(&als_get_adc_mutex);
  
	_cm3232_I2C_Read_Word(lpi->slave_addr, CM3232_ALS_DATA, &cm3232_ls_adc);
  
	mutex_unlock(&als_get_adc_mutex);	
}

static void report_lsensor_input_event(struct cm3232_info *lpi)
{
	lpi->current_lux = (uint32_t)div64_u64((uint64_t)cm3232_ls_adc * lpi->cal_data, (uint64_t) 100000);
  lpi->current_adc = cm3232_ls_adc;	
    
  D("[LS][CM3232] %s [ADC:%04x]: %d lux \n", __func__, lpi->current_adc, lpi->current_lux);

	input_report_abs(lpi->ls_input_dev, ABS_MISC, (int) lpi->current_lux);
  input_sync(lpi->ls_input_dev);  
}

static void report_do_work(struct work_struct *work)
{
	struct cm3232_info *lpi = lp_info;
	
  uint16_t dummy=CONTROL_ALS_REPORT;

#if 0 
	D("[CM3232] %s\n", __func__);
#endif  

  control_and_report(lpi, CONTROL_ALS_REPORT, dummy);
}

static int als_power(int enable)
{
	struct cm3232_info *lpi = lp_info;

	if (lpi->power)
		lpi->power(LS_PWR_ON, 1);

	return 0;
}

static int lightsensor_get_cal_data(struct cm3232_info *lpi)
{
	struct file *cal_filp = NULL;
	int err = 0;
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(CALIBRATION_FILE_PATH, O_RDONLY, 0666);
	if (IS_ERR(cal_filp))
	{
		err = PTR_ERR(cal_filp);
		if (err != -ENOENT)
		{
			pr_err("%s: Can't open calibration data file\n", __func__);
		}
		set_fs(old_fs);
		return err;
	}

	err = cal_filp->f_op->read(cal_filp,
		(char *)&lpi->cal_data, sizeof(uint16_t), &cal_filp->f_pos);
	if (err != sizeof(uint16_t))
	{
		pr_err("%s: Can't read the calibration data from file\n", __func__);
		err = -EIO;
	}

	pr_info("%s: cal_data = %d\n",
		__func__, lpi->cal_data);

	filp_close(cal_filp, current->files);
	set_fs(old_fs);

	return err;
}

static int lightsensor_enable(struct cm3232_info *lpi)
{
	int ret = -EIO;
	
	mutex_lock(&als_enable_mutex);
	D("[LS][CM3232] %s\n", __func__);

	if (!cal_data_retrieved)
	{
		/* get calibration data */		
		ret = lightsensor_get_cal_data(lpi);
		if (ret < 0 && ret != -ENOENT)
		{
			pr_err("%s: lightsensor_get_cal_data() failed\n",
				__func__);
		}
		else
		{
			cal_data_retrieved = true;
		}
	}

	if (lpi->als_enable) {
		D("[LS][CM3232] %s: already enabled\n", __func__);
		ret = 0;
	} else
		ret = control_and_report(lpi, CONTROL_ALS, 1);
  	
	mutex_unlock(&als_enable_mutex);
	return ret;
}

static int lightsensor_disable(struct cm3232_info *lpi)
{
	int ret = -EIO;
	mutex_lock(&als_disable_mutex);
	D("[LS][CM3232] %s\n", __func__);

	if ( lpi->als_enable == 0 ) {
		D("[LS][CM3232] %s: already disabled\n", __func__);
		ret = 0;
	} else
    ret = control_and_report(lpi, CONTROL_ALS, 0); 
    
	mutex_unlock(&als_disable_mutex);
	return ret;
}

static int lightsensor_open(struct inode *inode, struct file *file)
{
	struct cm3232_info *lpi = lp_info;
	int rc = 0;

	D("[LS][CM3232] %s\n", __func__);
	if (lpi->lightsensor_opened) {
		pr_err("[LS][CM3232 error]%s: already opened\n", __func__);
		rc = -EBUSY;
	}
	lpi->lightsensor_opened = 1;
	return rc;
}

static int lightsensor_release(struct inode *inode, struct file *file)
{
	struct cm3232_info *lpi = lp_info;

	D("[LS][CM3232] %s\n", __func__);
	lpi->lightsensor_opened = 0;
	return 0;
}

static long lightsensor_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	int rc, val;
	struct cm3232_info *lpi = lp_info;

	switch (cmd) {
	case LIGHTSENSOR_IOCTL_ENABLE:
		if (get_user(val, (unsigned long __user *)arg)) {
			rc = -EFAULT;
			break;
		}
		D("[LS][CM3232] %s LIGHTSENSOR_IOCTL_ENABLE, value = %d\n",
			__func__, val);
		rc = val ? lightsensor_enable(lpi) : lightsensor_disable(lpi);
		break;
	case LIGHTSENSOR_IOCTL_GET_ENABLED:
		val = lpi->als_enable;
		D("[LS][CM3232] %s LIGHTSENSOR_IOCTL_GET_ENABLED, enabled %d\n",
			__func__, val);
		rc = put_user(val, (unsigned long __user *)arg);
		break;
	default:
		pr_err("[LS][CM3232 error]%s: invalid cmd %d\n",
			__func__, _IOC_NR(cmd));
		rc = -EINVAL;
	}

	return rc;
}

static const struct file_operations lightsensor_fops = {
	.owner = THIS_MODULE,
	.open = lightsensor_open,
	.release = lightsensor_release,
	.unlocked_ioctl = lightsensor_ioctl
};

static struct miscdevice lightsensor_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "lightsensor",
	.fops = &lightsensor_fops
};

static ssize_t ls_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{

	int ret = 0;
	struct cm3232_info *lpi = lp_info;

	ret = sprintf(buf, "Light sensor Enable = %d\n",
			lpi->als_enable);

	return ret;
}

static ssize_t ls_enable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int ret = 0;
	int ls_auto;
	struct cm3232_info *lpi = lp_info;

	ls_auto = -1;
	sscanf(buf, "%d", &ls_auto);

	if (ls_auto != 0 && ls_auto != 1)
		return -EINVAL;

	if (ls_auto) {
		ret = lightsensor_enable(lpi);
	} else {
		ret = lightsensor_disable(lpi);
	}

	D("[LS][CM3232] %s: lpi->als_enable = %d, ls_auto=%d\n",
		__func__, lpi->als_enable, ls_auto);

	if (ret < 0)
		pr_err("[LS][CM3232 error]%s: set auto light sensor fail\n",
			__func__);

	return count;
}

static ssize_t ls_cal_data_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct cm3232_info *lpi = lp_info;
	int ret;

	ret = sprintf(buf, "%d\n", lpi->cal_data);

	return ret;
}

static ssize_t ls_cal_data_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	uint32_t new_cal_data = 0;
	struct cm3232_info *lpi = lp_info;	
	struct file *cal_filp = NULL;
	mm_segment_t old_fs;
	int err = 0;

	sscanf(buf, "%d", &new_cal_data);
	if (new_cal_data != 0)
	{
		lpi->cal_data = new_cal_data;
	}
	else  // reset calibration data
	{
		lpi->cal_data = CM3232_CAL_DATA_DEF;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(CALIBRATION_FILE_PATH,
			O_CREAT | O_TRUNC | O_WRONLY, 0666);
	if (IS_ERR(cal_filp))
	{
		pr_err("%s: Can't open calibration file\n", __func__);
		set_fs(old_fs);
		err = PTR_ERR(cal_filp);
		return err;
	}

	err = cal_filp->f_op->write(cal_filp,
		(char *)&lpi->cal_data, sizeof(uint32_t), &cal_filp->f_pos);
	if (err != sizeof(uint32_t))
	{
		pr_err("%s: Can't write the calibration data to file\n", __func__);
		err = -EIO;
	}

	filp_close(cal_filp, current->files);
	set_fs(old_fs);

	return count;
}

static ssize_t ls_poll_delay_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct cm3232_info *lpi = lp_info;

	ret = sprintf(buf, "Light sensor Poll Delay = %d ms\n",
			jiffies_to_msecs(lpi->als_polling_delay));

	return ret;
}

static ssize_t ls_poll_delay_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int new_delay;
	struct cm3232_info *lpi = lp_info;

	sscanf(buf, "%d", &new_delay);
  
	D("new delay = %d ms, old delay = %d ms \n", 
		new_delay, jiffies_to_msecs(lpi->als_polling_delay));

	if (lpi->als_enable) {
		lightsensor_disable(lpi);
		lpi->als_polling_delay = msecs_to_jiffies(new_delay);     
		lightsensor_enable(lpi);
	} else {
		lpi->als_polling_delay = msecs_to_jiffies(new_delay);
  }  

	return count;
}

static ssize_t ls_conf_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct cm3232_info *lpi = lp_info;
	return sprintf(buf, "LS_CONF = %x\n", lpi->ls_cmd);
}

static ssize_t ls_conf_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct cm3232_info *lpi = lp_info;
	int value = 0;
	sscanf(buf, "0x%x", &value);

	lpi->ls_cmd = value;
	printk(KERN_INFO "[LS]set LS_CONF = %x\n", lpi->ls_cmd);
	
	_cm3232_I2C_Write_Byte(lpi->slave_addr, lpi->ls_cmd);
	return count;
}

static ssize_t ls_adc_dbg_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct cm3232_info *lpi = lp_info;
  return sprintf(buf, "[LS] ADC[0x%04X] => %d lux\n",
		lpi->current_adc, lpi->current_lux);    
}

static ssize_t selftest_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct cm3232_info *lpi = lp_info;
	uint16_t value=0;
	int ret = 0;

	mutex_lock(&als_get_adc_mutex);

	ret=_cm3232_I2C_Read_Word(lpi->slave_addr, CM3232_ALS_DATA, &value);

	mutex_unlock(&als_get_adc_mutex);

	if (ret) {
		return sprintf(buf, "fail\n");
	}

	return sprintf(buf, "pass");
}

static ssize_t fac_data_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct cm3232_info *lpi = lp_info;
	int ret;

	ret = sprintf(buf, "%d\n", lpi->cal_data);

	return ret;
}

static ssize_t fac_data_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	uint32_t new_cal_data = 0;
	struct cm3232_info *lpi = lp_info;

	sscanf(buf, "%d", &new_cal_data);
	if (new_cal_data != 0)
	{
		lpi->cal_data = new_cal_data;
	}
	else  // reset calibration data
	{
		lpi->cal_data = CM3232_CAL_DATA_DEF;
	}


	return count;
}

static ssize_t raw_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct cm3232_info *lpi = lp_info;

	read_ls_adc_value();
	report_lsensor_input_event(lpi);
	return sprintf(buf, "%d\n",lpi->current_adc);
}

static ssize_t lux_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct cm3232_info *lpi = lp_info;

	read_ls_adc_value();
	report_lsensor_input_event(lpi);
	return sprintf(buf, "%d\n",lpi->current_lux);
}

static struct device_attribute dev_attr_ls_enable =
__ATTR(ls_enable, 0664, ls_enable_show, ls_enable_store);

static struct device_attribute dev_attr_ls_poll_delay =
__ATTR(ls_poll_delay, 0664, ls_poll_delay_show, ls_poll_delay_store);

static struct device_attribute dev_attr_ls_conf =
__ATTR(ls_conf, 0664, ls_conf_show, ls_conf_store);

static struct device_attribute dev_attr_ls_adc_dbg =
__ATTR(ls_adc_dbg, 0664, ls_adc_dbg_show, NULL);

static struct device_attribute dev_attr_light_cal_data =
__ATTR(ls_cali, 0664, ls_cal_data_show, ls_cal_data_store);

static struct device_attribute dev_attr_raw =
__ATTR(raw, 0664, raw_show, NULL);

static struct device_attribute dev_attr_lux =
__ATTR(lux, 0664, lux_show, NULL);

static struct device_attribute dev_attr_fac_data =
__ATTR(fac_data, 0664, fac_data_show, fac_data_store);

static struct device_attribute dev_attr_selftest =
__ATTR(cm_selftest, 0664, selftest_show, NULL);

static struct attribute *light_sysfs_attrs[] = {
	&dev_attr_ls_enable.attr,
	&dev_attr_ls_poll_delay.attr,  
	&dev_attr_ls_conf.attr,
	&dev_attr_ls_adc_dbg.attr,
	&dev_attr_light_cal_data.attr,
	&dev_attr_raw.attr,
	&dev_attr_lux.attr,
	&dev_attr_fac_data.attr,
	&dev_attr_selftest.attr,
	NULL
};

static struct attribute_group light_attribute_group = {
	.attrs = light_sysfs_attrs,
};

static int lightsensor_setup(struct cm3232_info *lpi)
{
	int ret;

	lpi->ls_input_dev = input_allocate_device();
	if (!lpi->ls_input_dev) {
		pr_err(
			"[LS][CM3232 error]%s: could not allocate ls input device\n",
			__func__);
		return -ENOMEM;
	}
	lpi->ls_input_dev->name = "cm3232-ls";

  set_bit(EV_ABS, lpi->ls_input_dev->evbit);
	input_set_abs_params(lpi->ls_input_dev, ABS_MISC, 0, 9, 0, 0);

	ret = input_register_device(lpi->ls_input_dev);
	if (ret < 0) {
		pr_err("[LS][CM3232 error]%s: can not register ls input device\n",
				__func__);
		goto err_free_ls_input_device;
	}

	ret = misc_register(&lightsensor_misc);
	if (ret < 0) {
		pr_err("[LS][CM3232 error]%s: can not register ls misc device\n",
				__func__);
		goto err_unregister_ls_input_device;
	}

	return ret;

err_unregister_ls_input_device:
	input_unregister_device(lpi->ls_input_dev);
err_free_ls_input_device:
	input_free_device(lpi->ls_input_dev);
	return ret;
}

static int cm3232_setup(struct cm3232_info *lpi)
{
	int ret = 0;

	als_power(1);
	msleep(5);
	
  /*initialize l-sensor*/
  //shutdown
	lpi->ls_cmd |= CM3232_ALS_SD;
	_cm3232_I2C_Write_Byte(lpi->slave_addr, lpi->ls_cmd);
	if (ret < 0)
	{
		goto fail_setup_cmd;
	}
	msleep(10);

  //reset
	lpi->ls_cmd |= CM3232_ALS_RESET;
	_cm3232_I2C_Write_Byte(lpi->slave_addr, lpi->ls_cmd);
	if (ret < 0)
	{
		goto fail_setup_cmd;
	}
	msleep(10);

  //clear reset bit
	lpi->ls_cmd &= CM3232_ALS_RESET_MASK;
	_cm3232_I2C_Write_Byte(lpi->slave_addr, lpi->ls_cmd); 

fail_setup_cmd:
	return ret;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cm3232_early_suspend(struct early_suspend *h)
{
	struct cm3232_info *lpi = lp_info;

	D("[LS][CM3232] %s\n", __func__);

	if (lpi->als_enable)
		lightsensor_disable(lpi);
}

static void cm3232_late_resume(struct early_suspend *h)
{
	struct cm3232_info *lpi = lp_info;

	D("[LS][CM3232] %s\n", __func__);

	if (!lpi->als_enable)
		lightsensor_enable(lpi);
}
#endif

#ifdef CONFIG_OF
static int cm3232_parse_dt(struct device *dev,
				struct cm3232_info *lpi)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int rc;
	
	D("[LS][CM3232] %s\n", __func__);

	rc = of_property_read_u32(np, "reg", &temp_val);
	if (rc)
	{
		dev_err(dev, "Unable to read slave_address\n");
		return rc;
	} 
	else
	{
		lpi->slave_addr = (uint8_t)temp_val;
	}
  
	D("[PS][CM3232]%s PARSE OK \n", __func__);

	return 0;
}
#endif

static int cm3232_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = 0;
	struct cm3232_info *lpi;
#ifndef CONFIG_OF  
	struct cm3232_platform_data *pdata;
#endif

	D("[ALS+PS][CM3232] %s\n", __func__);

	lpi = kzalloc(sizeof(struct cm3232_info), GFP_KERNEL);
	if (!lpi)
		return -ENOMEM;

	lpi->i2c_client = client;
	i2c_set_clientdata(client, lpi);
  lpi->als_polling_delay = msecs_to_jiffies(LS_POLLING_DELAY);

#ifndef CONFIG_OF
	pdata = client->dev.platform_data;
	if (!pdata) {
		pr_err("[ALS+PS][CM3232 error]%s: Assign platform_data error!!\n",
			__func__);
		ret = -EBUSY;
		goto err_platform_data_null;
	}
	
	lpi->power = pdata->power;
	lpi->slave_addr = pdata->slave_addr;
	lpi->ls_cmd  = pdata->ls_cmd;
#else
	if( cm3232_parse_dt(&client->dev, lpi) < 0 )
	{
		ret = -EBUSY;
		goto err_platform_data_null;  
	}
  
	lpi->ls_cmd = CM3232_ALS_IT_200MS | CM3232_ALS_HS_HIGH;
	lpi->power = NULL;
#endif
  	
	D("[PS][CM3232] %s: ls_cmd 0x%x\n",
		__func__, lpi->ls_cmd);
	
	if (lpi->ls_cmd == 0) {
		lpi->ls_cmd  = CM3232_ALS_IT_200MS | CM3232_ALS_HS_HIGH;
	}

	lp_info = lpi;

	mutex_init(&CM3232_control_mutex);

	mutex_init(&als_enable_mutex);
	mutex_init(&als_disable_mutex);
	mutex_init(&als_get_adc_mutex);

	lpi->cal_data = CM3232_CAL_DATA_DEF;
	lpi->cal_data_temp[0]=lpi->cal_data_temp[1]=lpi->cal_data_temp[2]=0xFFFFFFFF;

	lpi->lp_als_wq = create_singlethread_workqueue("cm3232_als_wq");
	if (!lpi->lp_als_wq) {
		pr_err("[CM3232 error]%s: can't create workqueue\n", __func__);
		ret = -ENOMEM;
		goto err_create_singlethread_als_workqueue;
	}

	ret = lightsensor_setup(lpi);
	if (ret < 0) {
		pr_err("[LS][CM3232 error]%s: lightsensor_setup error!!\n",
			__func__);
		goto err_lightsensor_setup;
	}

	ret = cm3232_setup(lpi);
	if (ret < 0) {
		pr_err("[PS_ERR][CM3232 error]%s: cm3232_setup error!\n", __func__);
		goto err_cm3232_setup;
	}
  
	lpi->cm3232_class = class_create(THIS_MODULE, "capella_sensors");
	if (IS_ERR(lpi->cm3232_class)) {
		ret = PTR_ERR(lpi->cm3232_class);
		lpi->cm3232_class = NULL;
		goto err_create_class;
	}

	lpi->ls_dev = device_create(lpi->cm3232_class,
				NULL, 0, "%s", "lightsensor");
	if (unlikely(IS_ERR(lpi->ls_dev))) {
		ret = PTR_ERR(lpi->ls_dev);
		lpi->ls_dev = NULL;
		goto err_create_ls_device;
	}

	/* register the attributes */
	ret = sysfs_create_group(&lpi->ls_input_dev->dev.kobj, &light_attribute_group);
	if (ret)
		goto err_sysfs_create_group_light;

#ifdef CONFIG_HAS_EARLYSUSPEND
	lpi->early_suspend.level =
			EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	lpi->early_suspend.suspend = cm3232_early_suspend;
	lpi->early_suspend.resume = cm3232_late_resume;
	register_early_suspend(&lpi->early_suspend);
#endif

	ret  = sysfs_create_link(&client->dev.kobj, &lpi->ls_input_dev->dev.kobj, "hal");
	if (ret){
		pr_err("sysfs_create_link failed\n");
	}

	D("[PS][CM3232] %s: Probe success!\n", __func__);

	return ret;

err_sysfs_create_group_light:
	device_destroy(lpi->cm3232_class, lpi->ls_dev->devt);
err_create_ls_device:
	class_destroy(lpi->cm3232_class);
err_create_class:
err_cm3232_setup:

	input_unregister_device(lpi->ls_input_dev);
	input_free_device(lpi->ls_input_dev);

	mutex_destroy(&CM3232_control_mutex);
	misc_deregister(&lightsensor_misc); //lightsensor_setup

	destroy_workqueue(lpi->lp_als_wq);  
err_create_singlethread_als_workqueue:  
err_lightsensor_setup:
	mutex_destroy(&als_enable_mutex);
	mutex_destroy(&als_disable_mutex);
	mutex_destroy(&als_get_adc_mutex);
err_platform_data_null:
	kfree(lpi);
	return ret;
}
   
static int control_and_report( struct cm3232_info *lpi, uint8_t mode, uint16_t param ) {
	int ret=0;
  
	mutex_lock(&CM3232_control_mutex);
  
	if( mode == CONTROL_ALS ){
		if(param){
			lpi->ls_cmd &= CM3232_ALS_SD_MASK;            
		} else {
			lpi->ls_cmd |= CM3232_ALS_SD;
      cancel_delayed_work_sync(&report_work);     
		}
		_cm3232_I2C_Write_Byte(lpi->slave_addr, lpi->ls_cmd);
		lpi->als_enable=param;

		if( param==1 ){
			msleep(400);  
		}
	} 

	if( mode == CONTROL_ALS_REPORT || ( (mode==CONTROL_ALS) && param ) ){
		read_ls_adc_value();
		report_lsensor_input_event(lpi);
		queue_delayed_work(lpi->lp_als_wq, &report_work, lpi->als_polling_delay);          
	} 

  mutex_unlock(&CM3232_control_mutex);
  return ret;
}

#ifdef CONFIG_PM_SLEEP
static int cm3232_suspend(struct device *dev)
{
	struct cm3232_info *lpi;
	lpi = dev_get_drvdata(dev);

	/*
	  * Save sensor state and disable them,
	  * this is to ensure internal state flags are set correctly.
	  * device will power off after both sensors are disabled.
	  * P sensor will not be disabled because it  is a wakeup sensor.
	*/

	lpi->als_enabled_before_suspend = lpi->als_enable;  
 
#ifdef UNSUPPORT_AUTO_BACKLIGHT
	if (lpi->als_enable == 1)
		lightsensor_disable(lpi);
#endif

	return 0;
}

static int cm3232_resume(struct device *dev)
{
	struct cm3232_info *lpi;
	lpi = dev_get_drvdata(dev);

	/* Don't disable light at phone calling
	  * while the automatic backlight is on.
	  */
#ifdef UNSUPPORT_AUTO_BACKLIGHT
	if (lpi->als_enabled_before_suspend)
		lightsensor_enable(lpi);
#endif	

	return 0;
}
#endif

static UNIVERSAL_DEV_PM_OPS(cm3232_pm, cm3232_suspend, cm3232_resume, NULL);


static const struct i2c_device_id cm3232_i2c_id[] = {
	{CM3232_I2C_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static struct of_device_id cm3232_match_table[] = {
	{ .compatible = "capella,cm3232",},
	{ },
};
#else
#define cm3232_match_table NULL
#endif

static struct i2c_driver cm3232_driver = {
	.id_table = cm3232_i2c_id,
	.probe = cm3232_probe,
	.driver = {
		.name = CM3232_I2C_NAME,
		.owner = THIS_MODULE,
		.pm = &cm3232_pm,    
    .of_match_table = of_match_ptr(cm3232_match_table),     
	},
};

static int __init cm3232_init(void)
{
	return i2c_add_driver(&cm3232_driver);
}

static void __exit cm3232_exit(void)
{
	i2c_del_driver(&cm3232_driver);
}

module_init(cm3232_init);
module_exit(cm3232_exit);

MODULE_AUTHOR("Frank Hsieh <Frank.Hsieh@vishay.com>");
MODULE_DESCRIPTION("CM3232 Optical Sensor Driver");
MODULE_LICENSE("GPL v2");
