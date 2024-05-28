/*
 * Copyright (c) 2018, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

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
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/i2c-dev.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/reboot.h>
#include <linux/hwid.h>
#include <linux/msm_drm_notify.h>

#define SET_GPIO_HIGH		1
#define SET_GPIO_LOW		0

#define I2C_APP_COMMAND_SIZE						5

#define ATMEL_CMD_MAJOR_FW_VERSION				0x01
#define ATMEL_CMD_HALL_ADC						0x02
#define ATMEL_CMD_LED_CTRL						0x03
#define ATMEL_CMD_LED_STATUS					0x04
#define ATMEL_CMD_MINOR_FW_VERSION				0x05
#define ATMEL_CMD_IRQ_ACTION					0x06
#define ATMEL_CMD_GET_HINGE_THRESHOLD			0x08
#define ATMEL_CMD_SET_HINGE_THRESHOLD			0x09
#define ATMEL_CMD_GET_CAMERA_KEY_THRESHOLD		0x0a
#define ATMEL_CMD_SET_CAMERA_KEY_THRESHOLD		0x0b
#define ATMEL_CMD_GET_HINGE_DEBOUNCE			0x0e
#define ATMEL_CMD_SET_HINGE_DEBOUNCE			0x0f
#define ATMEL_CMD_GET_CAMKEY_DEBOUNCE			0x10
#define ATMEL_CMD_SET_CAMKEY_DEBOUNCE			0x11
#define ATMEL_CMD_GET_HINGE_STATUS				0x12
#define ATMEL_CMD_GET_CAMKEY_STATUS				0x13
#define ATMEL_CMD_SET_HINGE_FUNC				0x14
#define ATMEL_CMD_GET_HINGE_FUNC				0x15

#define ATMEL_CMD_GET_TEST						0xFD
#define ATMEL_CMD_CHANGE_POWER_STATE			0xFE
#define ATMEL_CMD_POWER_STATE					0xFF

#define  MSB0W(u32)     (((uint8_t  *)&(u32))[3])
#define  MSB1W(u32)     (((uint8_t  *)&(u32))[2])
#define  MSB2W(u32)     (((uint8_t  *)&(u32))[1])
#define  MSB3W(u32)     (((uint8_t  *)&(u32))[0])

#define  MSB(u16)       (((uint8_t  *)&(u16))[1])
#define  LSB(u16)       (((uint8_t  *)&(u16))[0])

#define	HINGE_STATUS_SYSFS_ATTR	"hinge_status"

enum dev_irq_action_status {
	IRQ_ACTION_NOTHING,
	IRQ_ACTION_HINGE_CLOSE,
	IRQ_ACTION_HINGE_OPEN,
	IRQ_ACTION_CAMERA_KEY_PRESS,
	IRQ_ACTION_CAMERA_KEY_RELESS,
	IRQ_ACTION_REBOOT,
	IRQ_ACTION_SHUTDOWN,
};

enum dev_power_state {
	PWR_STATE_POWER_OFF,
	PWR_STATE_BOOTING,
	PWR_STATE_LOW_BATT,
	PWR_STATE_ALIVE,
	PWR_STATE_SHUTTING_DOWN,
	PWR_STATE_SYSTEM_TRIGGER_OFF,
	PWR_STATE_SLEEP,
	PWR_STATE_OFF_CHARGING_ENABLE,
	PWR_STATE_OFF_CHARGING_ALIVE,
	PWR_STATE_OFF_CHARGING_REBOOT,
	PWR_STATE_FASTBOOT,
	PWR_STATE_OFF_CHARGING_SLEEP,
	PWR_STATE_RECOVERY,
	
	PWR_STATE_UNKNOWN,
	PWR_STATE_FACTORY = 99,
	PWR_STATE_STANDBY = 100,
	PWR_STATE_NO_FUNCTION = 101,
};

enum dev_set_led_mode {
	SET_LED_OFF_MODE,
	SET_LED_PLAY_MODE,
	SET_LED_FOLLOW_MODE,
	SET_LED_MODE_MAX,
};

enum dev_set_led_state {
	SET_LED_OFF,
	SET_LED_ON_BREATH,
	SET_LED_ON_ALWAYS,
	SET_LED_BLINK_FAST,
	SET_LED_BLINK_MEDIUM,
	SET_LED_BLINK_SLOW,
	SET_LED_PLAY,
};

enum dev_hinge_func {
	HINGE_FUNC_UNKNOWN,
	HINGE_FUNC_ENABLE,
	HINGE_FUNC_DISABLE,
};

enum dev_hinge_status {
	HINGE_ON,
	HINGE_OFF,
	HINGE_UNKNOWN,
};

#define ATMEL_DEBUG(format, arg...)		\
	do {						\
		if (debug_level > 0)			\
			pr_err(format , ## arg);	\
	} while (0)

struct atmel_app_dev {
	struct i2c_client	*client;
	struct delayed_work mcu_wq_task;
	struct workqueue_struct *mcu_wq;
	struct delayed_work shutdown_wq_task;
	struct workqueue_struct *shutdown_wq;
	struct input_dev *input;
	int polling_time;
	struct wakeup_source timeout_wake_lock;
	bool call_reboot;

#ifdef CONFIG_FB
	struct notifier_block fb_notifier;
#endif
	bool fb_ready;

	int mcu_irq;
	int mcu_int_gpio;
	int mcu_rst_gpio;
	int mcu_bld_en_gpio;
	int mcu_swd_clk_gpio;
	int mcu_swd_dio_gpio;

	int hw_id;
	int evb_id;
	int offline_charging;
	int dis_hinge;

	uint32_t hinge_threshold;
	uint32_t camkey_threshold;
	uint8_t hinge_func;
	uint8_t hinge_status;

	int enter_standby;
	int enter_factory;
	int enter_recovery;
};

struct atmel_app_dev *atmel_app_dev;

static int debug_level = 0;
static uint32_t test_tmp_val = 0;
static uint32_t test_tmp_cmd = 0;

static void atmel_app_reset(void)
{
	if (atmel_app_dev->hw_id == 0) {
		pr_info("%s(%d) gpio%d=%d\n", __func__, __LINE__,
				atmel_app_dev->mcu_rst_gpio, SET_GPIO_LOW);
		gpio_set_value(atmel_app_dev->mcu_rst_gpio, SET_GPIO_LOW);
		mdelay(100);

		pr_info("%s(%d) gpio%d=%d\n", __func__, __LINE__,
				atmel_app_dev->mcu_rst_gpio, SET_GPIO_HIGH);
		gpio_set_value(atmel_app_dev->mcu_rst_gpio, SET_GPIO_HIGH);
		mdelay(100);
	} else {
		pr_info("%s(%d) gpio%d=%d\n", __func__, __LINE__,
				atmel_app_dev->mcu_rst_gpio, SET_GPIO_HIGH);
		gpio_set_value(atmel_app_dev->mcu_rst_gpio, SET_GPIO_HIGH);
		mdelay(100);

		pr_info("%s(%d) gpio%d=%d\n", __func__, __LINE__,
				atmel_app_dev->mcu_rst_gpio, SET_GPIO_LOW);
		gpio_set_value(atmel_app_dev->mcu_rst_gpio, SET_GPIO_LOW);
		mdelay(100);
	}
}

static void atmel_app_stop_mcu(void)
{
	if (atmel_app_dev->hw_id == 0) {
		pr_info("%s(%d) gpio%d=%d\n", __func__, __LINE__,
				atmel_app_dev->mcu_rst_gpio, SET_GPIO_LOW);
		gpio_set_value(atmel_app_dev->mcu_rst_gpio, SET_GPIO_LOW);
	} else {
		pr_info("%s(%d) gpio%d=%d\n", __func__, __LINE__,
				atmel_app_dev->mcu_rst_gpio, SET_GPIO_HIGH);
		gpio_set_value(atmel_app_dev->mcu_rst_gpio, SET_GPIO_HIGH);
	}
}

void atmel_app_enter_application_mode(void)
{
	if (atmel_app_dev->evb_id == 0 || atmel_app_dev->hw_id > 1) {
		pr_info("%s(%d) gpio%d=%d\n", __func__, __LINE__,
					atmel_app_dev->mcu_bld_en_gpio, SET_GPIO_LOW);
		gpio_set_value(atmel_app_dev->mcu_bld_en_gpio, SET_GPIO_LOW);
	} else {
		pr_info("%s(%d) gpio%d=%d\n", __func__, __LINE__,
					atmel_app_dev->mcu_bld_en_gpio, SET_GPIO_HIGH);
		gpio_set_value(atmel_app_dev->mcu_bld_en_gpio, SET_GPIO_HIGH);
	}

	if (atmel_app_dev->evb_id == 0) {
		pr_info("%s(%d) gpio%d=%d\n", __func__, __LINE__,
					atmel_app_dev->mcu_swd_clk_gpio, SET_GPIO_HIGH);
		gpio_set_value(atmel_app_dev->mcu_swd_clk_gpio, SET_GPIO_HIGH);
	} else {
		pr_info("%s(%d) gpio%d=%d\n", __func__, __LINE__,
					atmel_app_dev->mcu_swd_clk_gpio, SET_GPIO_LOW);
		gpio_set_value(atmel_app_dev->mcu_swd_clk_gpio, SET_GPIO_LOW);
	}
	mdelay(100);

	atmel_app_reset();
}

void atmel_app_enter_bootloader_mode(void)
{
	if (atmel_app_dev->evb_id == 0 || atmel_app_dev->hw_id > 1) {
		pr_info("%s(%d) gpio%d=%d\n", __func__, __LINE__,
					atmel_app_dev->mcu_bld_en_gpio, SET_GPIO_HIGH);
		gpio_set_value(atmel_app_dev->mcu_bld_en_gpio, SET_GPIO_HIGH);
	} else {
		pr_info("%s(%d) gpio%d=%d\n", __func__, __LINE__,
					atmel_app_dev->mcu_bld_en_gpio, SET_GPIO_LOW);
		gpio_set_value(atmel_app_dev->mcu_bld_en_gpio, SET_GPIO_LOW);
	}

	if (atmel_app_dev->evb_id == 0) {
		pr_info("%s(%d) gpio%d=%d\n", __func__, __LINE__,
					atmel_app_dev->mcu_swd_clk_gpio, SET_GPIO_HIGH);
		gpio_set_value(atmel_app_dev->mcu_swd_clk_gpio, SET_GPIO_HIGH);
	} else {
		pr_info("%s(%d) gpio%d=%d\n", __func__, __LINE__,
					atmel_app_dev->mcu_swd_clk_gpio, SET_GPIO_LOW);
		gpio_set_value(atmel_app_dev->mcu_swd_clk_gpio, SET_GPIO_LOW);
	}
	mdelay(100);

	atmel_app_reset();
}

static int atmel_app_i2c_read(char *tmp, int count)
{
	int rc = 0;
	int retry = 3;

	do {
		rc = i2c_master_recv(atmel_app_dev->client, tmp, count);
		if (rc != count)
			mdelay(10);

		retry--;
	}while(rc != count && retry >= 0);

	if (rc != count) {
		pr_err("%s(%d) i2c_master_recv(0x%x) failed(rc:%d)\n", __func__,
			__LINE__, atmel_app_dev->client->addr, rc);
		rc = -EIO;
	}

	return rc;
}

static ssize_t atmel_app_i2c_write(char *tmp, int count)
{
	int rc = 0;
	int retry = 3;


	do {
		rc = i2c_master_send(atmel_app_dev->client, tmp, count);
		if (rc != count)
			mdelay(10);

		retry--;
	}while(rc != count && retry >= 0);

	if (rc != count) {
		pr_err("%s(%d) i2c_master_send(0x%x) failed(rc:%d)\n", __func__,
			__LINE__, atmel_app_dev->client->addr, rc);
		rc = -EIO;
	}

	return rc;
}

static int atmel_app_get_data_32bit(uint8_t cmd_type, uint32_t *value)
{
	uint8_t cmd[I2C_APP_COMMAND_SIZE] = {0};
	int rc = 0;
	uint8_t ret[4] = {0};

	*value = 0;

	cmd[0] = cmd_type;

	rc = atmel_app_i2c_write(cmd, I2C_APP_COMMAND_SIZE);
	if (rc < 0) {
		pr_err("atmel_app_i2c_write(cmd) fail\n");
		rc = -EPERM;
		return rc;
	}

	mdelay(10);

	rc = atmel_app_i2c_read(ret, 4);
	if (rc < 0) {
		pr_err("atmel_app_i2c_read() fail\n");
		rc = -EPERM;
		return rc;
	}

	*value = ret[3];
	*value |= ret[2] << 8;
	*value |= ret[1] << 16;
	*value |= ret[0] << 24;
	rc = 0;

	return rc;
}

static int atmel_app_get_data_16bit(uint8_t cmd_type, uint16_t *value)
{
	uint8_t cmd[I2C_APP_COMMAND_SIZE] = {0};
	int rc = 0;
	uint8_t ret[2] = {0};

	*value = 0;

	cmd[0] = cmd_type;

	rc = atmel_app_i2c_write(cmd, I2C_APP_COMMAND_SIZE);
	if (rc < 0) {
		pr_err("atmel_app_i2c_write(cmd) fail\n");
		rc = -EPERM;
		return rc;
	}

	mdelay(10);

	rc = atmel_app_i2c_read(ret, 2);
	if (rc < 0) {
		pr_err("atmel_app_i2c_read() fail\n");
		rc = -EPERM;
		return rc;
	}

	*value = ret[1];
	*value |= ret[0] << 8;
	rc = 0;

	return rc;
}

static int atmel_app_get_data_8bit(uint8_t cmd_type, uint8_t *value)
{
	uint8_t cmd[I2C_APP_COMMAND_SIZE] = {0};
	int rc = 0;
	uint8_t ret = 0;

	*value = 0;

	cmd[0] = cmd_type;

	rc = atmel_app_i2c_write(cmd, I2C_APP_COMMAND_SIZE);
	if (rc < 0) {
		pr_err("atmel_app_i2c_write(cmd) fail\n");
		rc = -EPERM;
		return rc;
	}

	mdelay(10);

	rc = atmel_app_i2c_read(&ret, 1);
	if (rc < 0) {
		pr_err("atmel_app_i2c_read() fail\n");
		rc = -EPERM;
		return rc;
	}

	*value = ret;
	rc = 0;

	return rc;
}

static int atmel_app_set_data_32bit(uint8_t cmd_type, uint32_t value)
{
	uint8_t cmd[I2C_APP_COMMAND_SIZE] = {0};
	int rc = 0;

	cmd[0] = cmd_type;
	cmd[1] = MSB0W(value);
	cmd[2] = MSB1W(value);
	cmd[3] = MSB2W(value);
	cmd[4] = MSB3W(value);

	rc = atmel_app_i2c_write(cmd, I2C_APP_COMMAND_SIZE);
	if (rc < 0) {
		pr_err("atmel_app_i2c_write(cmd) fail\n");
		rc = -EPERM;
		return rc;
	}

	return 0;
}


static int atmel_app_set_data_8bit(uint8_t cmd_type, uint8_t value)
{
	uint8_t cmd[I2C_APP_COMMAND_SIZE] = {0};
	int rc = 0;

	cmd[0] = cmd_type;
	cmd[1] = value;

	rc = atmel_app_i2c_write(cmd, I2C_APP_COMMAND_SIZE);
	if (rc < 0) {
		pr_err("atmel_app_i2c_write(cmd) fail\n");
		rc = -EPERM;
		return rc;
	}

	return 0;
}

static int atmel_app_read_power_state(void)
{
	int rc = 0;
	uint8_t ret = 0;

	rc = atmel_app_get_data_8bit(ATMEL_CMD_POWER_STATE, &ret);
	if (rc < 0)
		return rc;

	rc = ret;

	return rc;
}

static int atmel_app_read_led_status(void)
{
	int rc = 0;
	uint8_t ret = 0;

	rc = atmel_app_get_data_8bit(ATMEL_CMD_LED_STATUS, &ret);
	if (rc < 0)
		return rc;

	rc = ret;

	return rc;
}

static int atmel_app_test_cmd(uint8_t cmd_type, uint32_t *ret_value)
{
	uint8_t cmd[I2C_APP_COMMAND_SIZE] = {0};
	int rc = 0;
	uint8_t ret[4] = {0};

	cmd[0] = ATMEL_CMD_GET_TEST;
	cmd[1] = cmd_type;

	rc = atmel_app_i2c_write(cmd, I2C_APP_COMMAND_SIZE);
	if (rc < 0) {
		pr_err("atmel_app_i2c_write(cmd) fail\n");
		rc = -EPERM;
		return rc;
	}

	mdelay(10);

	rc = atmel_app_i2c_read(ret, 4);
	if (rc < 0) {
		pr_err("atmel_app_i2c_read() fail\n");
		rc = -EPERM;
		return rc;
	}

	*ret_value = ret[3];
	*ret_value |= ret[2] << 8;
	*ret_value |= ret[1] << 16;
	*ret_value |= ret[0] << 24;
	rc = 0;

	return rc;
}

static int atmel_app_led_ctrl(uint8_t ctrl, uint16_t on_t, uint16_t off_t)
{
	uint8_t cmd[I2C_APP_COMMAND_SIZE] = {0};
	int rc = 0;
	uint16_t on_time = 0;
	uint16_t off_time = 0;

	cmd[0] = ATMEL_CMD_LED_CTRL;

	switch(ctrl) {
		default:
		case SET_LED_OFF:
		case SET_LED_ON_ALWAYS:
			cmd[1] = SET_LED_OFF;
			break;

		case SET_LED_ON_BREATH:
			cmd[1] = ctrl;
			break;

		case SET_LED_BLINK_FAST:
			cmd[1] = ctrl;
			on_time = 200;
			off_time = 200;
			break;

		case SET_LED_BLINK_MEDIUM:
			cmd[1] = ctrl;
			on_time = 800;
			off_time = 800;
			break;

		case SET_LED_BLINK_SLOW:
			cmd[1] = ctrl;
			on_time = 1500;
			off_time = 1500;
			break;

		case SET_LED_PLAY:
			cmd[1] = ctrl;
			break;

		case 0xff:
			cmd[1] = ctrl;
			on_time = on_t;
			off_time = off_t;
			break;
	}

	cmd[2] = LSB(on_time);
	cmd[3] = MSB(on_time) << 4;
	cmd[3] |= MSB(off_time);
	cmd[4] = LSB(off_time);

	rc = atmel_app_i2c_write(cmd, I2C_APP_COMMAND_SIZE);
	if (rc < 0) {
		pr_err("atmel_app_i2c_write(cmd) fail\n");
		rc = -EPERM;
		return rc;
	}

	return rc;
}

static int atmel_app_change_power_state(uint8_t state)
{
	int rc = atmel_app_set_data_8bit(ATMEL_CMD_CHANGE_POWER_STATE, state);

	return rc;
}

static int atmel_app_read_hall_adc(uint32_t *hall_adc)
{
	int rc = atmel_app_get_data_32bit(ATMEL_CMD_HALL_ADC, hall_adc);

	return rc;
}

static int atmel_app_read_irq_action(void)
{
	int rc = 0;
	uint8_t irq_action = 0;

	rc = atmel_app_get_data_8bit(ATMEL_CMD_IRQ_ACTION, &irq_action);
	if (rc < 0)
		return rc;

	rc = (int)irq_action;

	return rc;
}


static int atmel_app_read_fw_version(uint8_t type, uint16_t *ver)
{
	int rc = atmel_app_get_data_16bit(type, ver);

	return rc;
}

static int atmel_app_parse_dtree(struct i2c_client *client,
				       struct atmel_app_dev *atmel_app_dev)
{	
	int gpio;
	struct device_node *np = atmel_app_dev->client->dev.of_node;

	if (atmel_app_dev->client->dev.of_node) {
		gpio = of_get_named_gpio(np, "mcu_int-gpio", 0);
		if (gpio < 0)
			return -EPERM;
		atmel_app_dev->mcu_int_gpio= gpio;

		gpio = of_get_named_gpio(np, "mcu_rst-gpio", 0);
		if (gpio < 0)
			return -EPERM;
		atmel_app_dev->mcu_rst_gpio = gpio;

		gpio = of_get_named_gpio(np, "mcu_bld_en-gpio", 0);
		if (gpio < 0)
			return -EPERM;
		atmel_app_dev->mcu_bld_en_gpio = gpio;

		gpio = of_get_named_gpio(np, "mcu_swd_clk-gpio", 0);
		if (gpio < 0)
			return -EPERM;
		atmel_app_dev->mcu_swd_clk_gpio= gpio;

		gpio = of_get_named_gpio(np, "mcu_swd_dio-gpio", 0);
		if (gpio < 0)
			return -EPERM;
		atmel_app_dev->mcu_swd_dio_gpio= gpio;
	}

	return 0;
}

static ssize_t  atmel_app_sysfs_led_ctrl_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", atmel_app_read_led_status());
}

static ssize_t atmel_app_sysfs_led_ctrl_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long val;
	int rc = 0;
	char * pch;
	uint16_t on_time = 0;
	uint16_t off_time = 0;
	int p;
	char str[10];

	pch = strchr(buf, ':');

	if (pch == NULL) {
		rc = kstrtoul(buf, 0, &val);
		if (rc)
			pr_err("%s(%d) %s is not in hex or decimal form.\n",
				__func__, __LINE__, buf);
		else
			atmel_app_led_ctrl((uint8_t)val, on_time, off_time);
	} else {
		p = pch - buf +1;

		memset(str, 0, sizeof(str));
		memcpy(str, buf, p-1);
		rc = kstrtoul(str, 0, &val);
		if (rc)
			pr_err("%s(%d) %s is not in hex or decimal form.\n",
				__func__, __LINE__, str);
		else {
			on_time = val;
			memset(str, 0, sizeof(str));
			memcpy(str, &buf[p], strlen(buf) - p);
			rc  = kstrtoul(str, 0, &val);
			if (rc)
				pr_err("%s(%d) %s is not in hex or decimal form.\n",
					__func__, __LINE__, str);
			else {
				off_time = val;
				if (on_time > 1500)
					on_time = 1500;
				if (on_time < 10)
					on_time = 10;
				if (off_time > 4000)
					off_time = 4000;
				if (off_time < 10)
					off_time = 10;
				atmel_app_led_ctrl(0xff, on_time, off_time);
			}
		}
	}

	return size;
}

static DEVICE_ATTR(led_ctrl, S_IRUGO | S_IWUSR,
	atmel_app_sysfs_led_ctrl_show, atmel_app_sysfs_led_ctrl_store);

static ssize_t  atmel_app_sysfs_power_state_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", atmel_app_read_power_state());
}

static ssize_t atmel_app_sysfs_power_state_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long val;
	int rc = 0;

	rc = kstrtoul(buf, 0, &val);
	if (rc)
		pr_err("%s(%d) %s is not in hex or decimal form.\n",
			__func__, __LINE__, buf);
	else {
		if (atmel_app_change_power_state((unsigned char)val) < 0)
			pr_err("%s(%d) change_power_state(%lu) -- fail.\n",
			__func__, __LINE__, val);
		else {
			if (val == PWR_STATE_FACTORY) {
				atmel_app_dev->enter_factory = 1;
				atmel_app_dev->enter_standby= 0;
				atmel_app_dev->enter_recovery= 0;
			} else if (val == PWR_STATE_STANDBY) {
				atmel_app_dev->enter_factory = 0;
				atmel_app_dev->enter_standby= 1;
				atmel_app_dev->enter_recovery= 0;
			} else if (val == PWR_STATE_RECOVERY) {
				atmel_app_dev->enter_factory = 0;
				atmel_app_dev->enter_standby= 0;
				atmel_app_dev->enter_recovery= 1;
			} else {
				atmel_app_dev->enter_factory = 0;
				atmel_app_dev->enter_standby= 0;
			}
		}
	}

	return size;
}

static DEVICE_ATTR(power_state, S_IRUGO | S_IWUSR,
	atmel_app_sysfs_power_state_show, atmel_app_sysfs_power_state_store);

static ssize_t  atmel_app_sysfs_hall_adc_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	uint32_t hall_adc = 0;

	atmel_app_read_hall_adc(&hall_adc);

	return snprintf(buf, PAGE_SIZE, "%d\n", hall_adc);
}

static DEVICE_ATTR(hall_adc, S_IRUGO | S_IWUSR,
	atmel_app_sysfs_hall_adc_show, NULL);

static ssize_t atmel_app_sysfs_polling_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long val;
	int rc = 0;

	rc = kstrtoul(buf, 0, &val);
	if (rc)
		pr_err("%s(%d) %s is not in hex or decimal form.\n",
			__func__, __LINE__, buf);
	else {
		cancel_delayed_work(&atmel_app_dev->mcu_wq_task);
		if (val == 1)
			queue_delayed_work(atmel_app_dev->mcu_wq, &atmel_app_dev->mcu_wq_task, 1);
	}

	return size;
}

static DEVICE_ATTR(polling, S_IRUGO | S_IWUSR,
	NULL, atmel_app_sysfs_polling_store);

static ssize_t  atmel_app_sysfs_debug_level_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", debug_level);
}


static ssize_t atmel_app_sysfs_debug_level_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long val;
	int rc = 0;

	rc = kstrtoul(buf, 0, &val);
	if (rc)
		pr_err("%s(%d) %s is not in hex or decimal form.\n",
			__func__, __LINE__, buf);
	else {
		debug_level = (uint32_t)val;
	}

	return size;
}

static DEVICE_ATTR(debug_level, S_IRUGO | S_IWUSR,
	atmel_app_sysfs_debug_level_show, atmel_app_sysfs_debug_level_store);

static ssize_t  atmel_app_sysfs_polling_time_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", atmel_app_dev->polling_time);
}


static ssize_t atmel_app_sysfs_polling_time_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long val;
	int rc = 0;

	rc = kstrtoul(buf, 0, &val);
	if (rc)
		pr_err("%s(%d) %s is not in hex or decimal form.\n",
			__func__, __LINE__, buf);
	else {
		atmel_app_dev->polling_time = (uint32_t)val;
	}

	return size;
}

static DEVICE_ATTR(polling_time, S_IRUGO | S_IWUSR,
	atmel_app_sysfs_polling_time_show, atmel_app_sysfs_polling_time_store);


static ssize_t  atmel_app_sysfs_test_cmd_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	atmel_app_test_cmd(test_tmp_cmd, &test_tmp_val);

	return snprintf(buf, PAGE_SIZE, "cmd:%d val:%u\n", test_tmp_cmd, test_tmp_val);
}

static ssize_t atmel_app_sysfs_test_cmd_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long val;
	int rc = 0;

	rc = kstrtoul(buf, 0, &val);
	if (rc)
		pr_err("%s(%d) %s is not in hex or decimal form.\n",
			__func__, __LINE__, buf);
	else
		test_tmp_cmd = (uint8_t)val;

	return size;
}

static DEVICE_ATTR(test_cmd, S_IRUGO | S_IWUSR,
	atmel_app_sysfs_test_cmd_show, atmel_app_sysfs_test_cmd_store);

static ssize_t  atmel_app_sysfs_camera_key_debounce_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	uint32_t threshold = 0;

	atmel_app_get_data_32bit(ATMEL_CMD_GET_CAMKEY_DEBOUNCE, &threshold);

	return snprintf(buf, PAGE_SIZE, "%d\n", threshold);
}

static ssize_t atmel_app_sysfs_camera_key_debounce_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long val;
	int rc = 0;

	rc = kstrtoul(buf, 0, &val);
	if (rc)
		pr_err("%s(%d) %s is not in hex or decimal form.\n",
			__func__, __LINE__, buf);
	else {
		if (atmel_app_set_data_32bit(ATMEL_CMD_SET_CAMKEY_DEBOUNCE, (uint32_t)val) < 0)
			pr_err("%s(%d) atmel_app_set_data_32bit(CAMKEY_DEBOUNCE, %s) -- fail.\n",
			__func__, __LINE__, buf);
	}

	return size;
}

static DEVICE_ATTR(camera_key_debounce, S_IRUGO | S_IWUSR,
	atmel_app_sysfs_camera_key_debounce_show, atmel_app_sysfs_camera_key_debounce_store);

static ssize_t  atmel_app_sysfs_camera_key_threshold_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	uint32_t threshold = 0;

	atmel_app_get_data_32bit(ATMEL_CMD_GET_CAMERA_KEY_THRESHOLD, &threshold);

	return snprintf(buf, PAGE_SIZE, "%d\n", threshold);
}

static ssize_t atmel_app_sysfs_camera_key_threshold_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long val;
	int rc = 0;

	rc = kstrtoul(buf, 0, &val);
	if (rc)
		pr_err("%s(%d) %s is not in hex or decimal form.\n",
			__func__, __LINE__, buf);
	else {
		if (atmel_app_set_data_32bit(ATMEL_CMD_SET_CAMERA_KEY_THRESHOLD, (uint32_t)val) < 0)
			pr_err("%s(%d) atmel_app_set_data_32bit(CAMERA_KEY_THRESHOLD, %s) -- fail.\n",
			__func__, __LINE__, buf);
		else {
			atmel_app_dev->camkey_threshold = (uint32_t)val;
			pr_err("%s : update camkey_threshold = %u\n", __func__, atmel_app_dev->camkey_threshold);
		}
	}

	return size;
}

static DEVICE_ATTR(camera_key_threshold, S_IRUGO | S_IWUSR,
	atmel_app_sysfs_camera_key_threshold_show, atmel_app_sysfs_camera_key_threshold_store);

static ssize_t  atmel_app_sysfs_hinge_debounce_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	uint32_t threshold = 0;

	atmel_app_get_data_32bit(ATMEL_CMD_GET_HINGE_DEBOUNCE, &threshold);

	return snprintf(buf, PAGE_SIZE, "%d\n", threshold);
}

static ssize_t atmel_app_sysfs_hinge_debounce_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long val;
	int rc = 0;

	rc = kstrtoul(buf, 0, &val);
	if (rc)
		pr_err("%s(%d) %s is not in hex or decimal form.\n",
			__func__, __LINE__, buf);
	else {
		if (atmel_app_set_data_32bit(ATMEL_CMD_SET_HINGE_DEBOUNCE, (uint32_t)val) < 0)
			pr_err("%s(%d) atmel_app_set_data_32bit(HINGE_DEBOUNCE, %s) -- fail.\n",
			__func__, __LINE__, buf);
	}

	return size;
}

static DEVICE_ATTR(hinge_debounce, S_IRUGO | S_IWUSR,
	atmel_app_sysfs_hinge_debounce_show, atmel_app_sysfs_hinge_debounce_store);

static ssize_t  atmel_app_sysfs_hinge_threshold_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	uint32_t threshold = 0;

	atmel_app_get_data_32bit(ATMEL_CMD_GET_HINGE_THRESHOLD, &threshold);

	return snprintf(buf, PAGE_SIZE, "%d\n", threshold);
}

static ssize_t atmel_app_sysfs_hinge_threshold_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long val;
	int rc = 0;

	rc = kstrtoul(buf, 0, &val);
	if (rc)
		pr_err("%s(%d) %s is not in hex or decimal form.\n",
			__func__, __LINE__, buf);
	else {
		if (atmel_app_set_data_32bit(ATMEL_CMD_SET_HINGE_THRESHOLD, (uint32_t)val) < 0)
			pr_err("%s(%d) atmel_app_set_data_32bit(HINGE_THRESHOLD, %s) -- fail.\n",
			__func__, __LINE__, buf);
		else {
			atmel_app_dev->hinge_threshold = (uint32_t)val;
			pr_err("%s : update hinge_threshold = %u\n", __func__, atmel_app_dev->hinge_threshold);
		}

		if (atmel_app_set_data_8bit(ATMEL_CMD_SET_HINGE_FUNC, atmel_app_dev->hinge_func) < 0)
			pr_err("%s(%d) atmel_app_set_data_8bit(HINGE_FUNC, %d) -- fail.\n",
			__func__, __LINE__, atmel_app_dev->hinge_func);
		pr_err("%s : update hinge_func = %d\n", __func__, atmel_app_dev->hinge_func);
	}

	return size;
}

static DEVICE_ATTR(hinge_threshold, S_IRUGO | S_IWUSR,
	atmel_app_sysfs_hinge_threshold_show, atmel_app_sysfs_hinge_threshold_store);

static ssize_t  atmel_app_sysfs_camera_key_status_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	uint8_t val = 0;

	atmel_app_get_data_8bit(ATMEL_CMD_GET_CAMKEY_STATUS, &val);

	return snprintf(buf, PAGE_SIZE, "%d\n", val);
}

static DEVICE_ATTR(camera_key_status, S_IRUGO | S_IWUSR,
	atmel_app_sysfs_camera_key_status_show, NULL);


static ssize_t  atmel_app_sysfs_hinge_status_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	uint8_t val = 0;

	atmel_app_get_data_8bit(ATMEL_CMD_GET_HINGE_STATUS, &val);

	return snprintf(buf, PAGE_SIZE, "%d\n", val);
}

static DEVICE_ATTR(hinge_status, S_IRUGO | S_IWUSR,
	atmel_app_sysfs_hinge_status_show, NULL);

static ssize_t  atmel_app_sysfs_firmware_version_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	uint16_t major;
	uint16_t minor;
	int rc;

	rc = atmel_app_read_fw_version(ATMEL_CMD_MAJOR_FW_VERSION, &major);

	if (rc < 0)
		return snprintf(buf, PAGE_SIZE, "0\n");

	rc = atmel_app_read_fw_version(ATMEL_CMD_MINOR_FW_VERSION, &minor);
	if (rc < 0)
		return snprintf(buf, PAGE_SIZE, "0\n");

	return snprintf(buf, PAGE_SIZE, "%d%04d\n", major, minor);
}

static DEVICE_ATTR(firmware_version, S_IRUGO | S_IWUSR,
	atmel_app_sysfs_firmware_version_show, NULL);

static ssize_t atmel_app_sysfs_change_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	char value[10];

	if (sscanf(buf, "%s", value) != 1) {
		pr_err("sscanf fail.\n");
		return count;
	}

	if (strcmp(value, "bl") == 0)
		atmel_app_enter_bootloader_mode();
	else if (strcmp(value, "app") == 0)
		atmel_app_enter_application_mode();
	else if (strcmp(value, "stop") == 0)
		atmel_app_stop_mcu();

	return count;
}

static DEVICE_ATTR(change_mode, S_IRUGO | S_IWUSR,
	NULL, atmel_app_sysfs_change_mode_store);

static struct attribute *atmel_app_attributes[] = {
	&dev_attr_firmware_version.attr,
	&dev_attr_hall_adc.attr,
	&dev_attr_led_ctrl.attr,
	&dev_attr_polling.attr,
	&dev_attr_hinge_status.attr,
	&dev_attr_camera_key_status.attr,
	&dev_attr_hinge_threshold.attr,
	&dev_attr_camera_key_threshold.attr,
	&dev_attr_hinge_debounce.attr,
	&dev_attr_camera_key_debounce.attr,
	&dev_attr_debug_level.attr,
	&dev_attr_polling_time.attr,
	&dev_attr_power_state.attr,
	&dev_attr_test_cmd.attr,
	&dev_attr_change_mode.attr,
	NULL
};
static const struct attribute_group atmel_app_attr_group = {
	.attrs = atmel_app_attributes,
};

void atmel_app_delay_shutdown_wq_task(struct work_struct *work)
{
	if (atmel_app_dev->call_reboot) {
		pr_err("%s: call kernel_restart()\n", __func__);
		kernel_restart(NULL);
	} else {
		pr_err("%s: call kernel_power_off()\n", __func__);
		kernel_power_off();
	}
}

void atmel_app_delay_wq_task(struct work_struct *work)
{
	uint32_t hall_adc_tmp;
	uint32_t hall_adc = 0;
	int adc_count = 1;
	int i;

	for (i=0; i<adc_count; i++) {
		atmel_app_read_hall_adc(&hall_adc_tmp);
		if (hall_adc_tmp > 0)
			hall_adc += hall_adc_tmp;
	}
	hall_adc = hall_adc / adc_count;

	ATMEL_DEBUG("[polling] hall_adc=%d\n", hall_adc);

	queue_delayed_work(atmel_app_dev->mcu_wq, &atmel_app_dev->mcu_wq_task, atmel_app_dev->polling_time);
}

void atmel_app_notify_hinge_status(void) {
	uint8_t hinge_status = HINGE_UNKNOWN;
	atmel_app_get_data_8bit(ATMEL_CMD_GET_HINGE_STATUS, &hinge_status);
	if (atmel_app_dev->hinge_status != hinge_status) {
		atmel_app_dev->hinge_status = hinge_status;
		sysfs_notify(&atmel_app_dev->client->dev.kobj, NULL,
			HINGE_STATUS_SYSFS_ATTR);
	}
}

static void atmel_app_irq_action(void)
{
	int irq_action = atmel_app_read_irq_action();

	pr_err("%s: irq_action = %d\n", __func__, irq_action);

	if (atmel_app_dev->enter_factory == 1 ||
		atmel_app_dev->enter_standby == 1 ||
		atmel_app_dev->enter_recovery== 1) {
		pr_err("%s: factory:%d, standby:%d, recovery:%d irq_action disable \n",
			__func__,
			atmel_app_dev->enter_factory,
			atmel_app_dev->enter_standby,
			atmel_app_dev->enter_recovery);
		return;
	}

	switch(irq_action) {
		case IRQ_ACTION_SHUTDOWN:
		case IRQ_ACTION_REBOOT:
			if (irq_action == IRQ_ACTION_SHUTDOWN)
				atmel_app_dev->call_reboot = false;
			else
				atmel_app_dev->call_reboot = true;
			__pm_wakeup_event(&atmel_app_dev->timeout_wake_lock, HZ * 20);
			queue_delayed_work(atmel_app_dev->shutdown_wq, &atmel_app_dev->shutdown_wq_task, 1);
			break;

		case IRQ_ACTION_CAMERA_KEY_PRESS:
			if (atmel_app_dev->fb_ready)
				input_report_key(atmel_app_dev->input, KEY_CAMERA, 1);
			else
				input_report_key(atmel_app_dev->input, KEY_WAKEUP, 1);
			input_sync(atmel_app_dev->input);
			break;

		case IRQ_ACTION_CAMERA_KEY_RELESS:
			if (atmel_app_dev->fb_ready)
				input_report_key(atmel_app_dev->input, KEY_CAMERA, 0);
			else
				input_report_key(atmel_app_dev->input, KEY_WAKEUP, 0);
			input_sync(atmel_app_dev->input);
			break;

		case IRQ_ACTION_HINGE_CLOSE:
		case IRQ_ACTION_HINGE_OPEN:
			atmel_app_notify_hinge_status();
			break;

		default:
		case IRQ_ACTION_NOTHING:
			break;
	}
}

static irqreturn_t atmel_app_event_irq(int irq, void *_data)
{
	atmel_app_irq_action();

	return IRQ_HANDLED;
}

static int atmel_app_config_gpio(void)
{
	int rc;
	int cf_gpio;
	int gpio_value;
	int irq_action;

	cf_gpio = atmel_app_dev->mcu_rst_gpio;
	if (atmel_app_dev->hw_id == 0) {
		rc = gpio_request_one(cf_gpio,
				GPIOF_DIR_OUT | GPIOF_INIT_HIGH,
			      "mcu_rst_gpio");
	} else {
		rc = gpio_request_one(cf_gpio,
				GPIOF_DIR_OUT | GPIOF_INIT_LOW,
			      "mcu_rst_gpio");
	}
	if (rc) {
		pr_err("%s(%d) failed to request GPIO %d (%s)\n", __func__, __LINE__,
			cf_gpio, "mcu_rst_gpio");

		goto error_mcu_rst_gpio;
	}

	cf_gpio = atmel_app_dev->mcu_bld_en_gpio;
	if (atmel_app_dev->evb_id == 0 || atmel_app_dev->hw_id > 1) {
		rc = gpio_request_one(cf_gpio,
				GPIOF_DIR_OUT | GPIOF_INIT_LOW,
			      "mcu_bld_en_gpio");
	} else {
		rc = gpio_request_one(cf_gpio,
				GPIOF_DIR_OUT | GPIOF_INIT_HIGH,
			      "mcu_bld_en_gpio");
	}
	if (rc) {
		pr_err("%s(%d) failed to request GPIO %d (%s)\n", __func__, __LINE__,
			cf_gpio, "mcu_bld_en_gpio");

		goto error_mcu_bld_en_gpio;
	}

	cf_gpio = atmel_app_dev->mcu_swd_clk_gpio;
	if (atmel_app_dev->evb_id == 0) {
		rc = gpio_request_one(cf_gpio,
			GPIOF_DIR_OUT | GPIOF_INIT_HIGH,
		      "mcu_swd_clk_gpio");
	} else {
		rc = gpio_request_one(cf_gpio,
			GPIOF_DIR_OUT | GPIOF_INIT_LOW,
		      "mcu_swd_clk_gpio");
	}
	if (rc) {
		pr_err("%s(%d) failed to request GPIO %d (%s)\n", __func__, __LINE__,
			cf_gpio, "mcu_swd_clk_gpio");
			goto error_mcu_swd_clk_gpio;
	}

	cf_gpio = atmel_app_dev->mcu_swd_dio_gpio;
	rc = gpio_request_one(cf_gpio,
			GPIOF_DIR_OUT | GPIOF_INIT_HIGH,
		      "mcu_swd_dio_gpio");
	if (rc) {
		pr_err("%s(%d) failed to request GPIO %d (%s)\n", __func__, __LINE__,
			cf_gpio, "mcu_swd_dio_gpio");
			goto error_mcu_swd_dio_gpio;
	}

	if (atmel_app_dev->evb_id == 0 || atmel_app_dev->hw_id > 1) {
		gpio_export(atmel_app_dev->mcu_rst_gpio, true);
		gpio_export(atmel_app_dev->mcu_bld_en_gpio, true);
		gpio_export(atmel_app_dev->mcu_swd_clk_gpio, true);
		gpio_export(atmel_app_dev->mcu_swd_dio_gpio, true);
	}

	cf_gpio = atmel_app_dev->mcu_int_gpio;
	rc = gpio_request_one(cf_gpio,
			GPIOF_DIR_IN | GPIOF_INIT_LOW,
		      "mcu_rst_gpio");
	if (rc) {
		pr_err("%s(%d) failed to request GPIO %d (%s)\n", __func__, __LINE__,
			cf_gpio, "mcu_int_gpio");

		goto error_mcu_gpios;
	}
	gpio_export(atmel_app_dev->mcu_int_gpio, true);

	gpio_value = gpio_get_value(atmel_app_dev->mcu_int_gpio);
	if (gpio_value) {
		/* atmel_app_irq_action(); */
		irq_action = atmel_app_read_irq_action();
		pr_info("%s: irq_action = %d\n", __func__, irq_action);
	}

	atmel_app_dev->mcu_irq = gpio_to_irq(atmel_app_dev->mcu_int_gpio);

	if (atmel_app_dev->mcu_irq < 0) {
		pr_err("%s (%d)unable to map GPIO mcu_int_gpio(%d) to an IRQ\n",
			__func__, __LINE__, atmel_app_dev->mcu_int_gpio);
		goto error_mcu_irq;
	}

	rc = request_threaded_irq(atmel_app_dev->mcu_irq,
					NULL,
					atmel_app_event_irq,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					"mcu_int_gpio_irq",
					&atmel_app_dev->client->dev);
	if (rc) {
		pr_err("%s (%d)failed to register GPIO %d as interrupt\n",
					__func__, __LINE__,
					atmel_app_dev->mcu_irq);
		goto error_mcu_irq;
	}

	/* enable_irq_wake(atmel_app_dev->mcu_irq); */

	return 0;

error_mcu_irq:
	gpio_free(atmel_app_dev->mcu_int_gpio);
error_mcu_gpios:
	gpio_free(atmel_app_dev->mcu_swd_dio_gpio);
error_mcu_swd_dio_gpio:
	gpio_free(atmel_app_dev->mcu_swd_clk_gpio);
error_mcu_swd_clk_gpio:
	gpio_free(atmel_app_dev->mcu_bld_en_gpio);
error_mcu_bld_en_gpio:
	gpio_free(atmel_app_dev->mcu_rst_gpio);
error_mcu_rst_gpio:
	pr_err("%s(%d) config gpio tree failed\n", __func__, __LINE__);
	return -EPERM;
}

static void atmel_app_mcu_state_check(void)
{
	uint32_t threshold;
	uint8_t tmp;

	if (atmel_app_read_power_state() < 0) {
		pr_err("%s(%d) Read mcu state fail, call reset mcu...\n",
			__func__, __LINE__);
		atmel_app_enter_application_mode();
	}

	if (atmel_app_dev->hinge_threshold != 0xFFFF) {
		if (atmel_app_get_data_32bit(ATMEL_CMD_GET_HINGE_THRESHOLD, &threshold) >= 0) {
			if (atmel_app_dev->hinge_threshold != threshold)
				atmel_app_set_data_32bit(ATMEL_CMD_SET_HINGE_THRESHOLD,
						atmel_app_dev->hinge_threshold);
		}
	}

	if (atmel_app_dev->hinge_func != 0xFF) {
		if (atmel_app_get_data_8bit(ATMEL_CMD_GET_HINGE_FUNC, &tmp) >= 0) {
			if (atmel_app_dev->hinge_func != tmp)
				atmel_app_set_data_8bit(ATMEL_CMD_SET_HINGE_FUNC,
						atmel_app_dev->hinge_func);
		}
	}

	if (atmel_app_set_data_8bit(ATMEL_CMD_SET_HINGE_FUNC, atmel_app_dev->hinge_func) < 0)
			pr_err("%s(%d) atmel_app_set_data_8bit(HINGE_FUNC, %d) -- fail.\n",
			__func__, __LINE__, atmel_app_dev->hinge_func);

	if (atmel_app_dev->camkey_threshold != 0xFFFF) {
		if (atmel_app_get_data_32bit(ATMEL_CMD_GET_CAMERA_KEY_THRESHOLD, &threshold) >= 0) {
			if (atmel_app_dev->camkey_threshold != threshold)
				atmel_app_set_data_32bit(ATMEL_CMD_SET_CAMERA_KEY_THRESHOLD,
						atmel_app_dev->camkey_threshold);
		}
	}
}

static void atmel_app_mcu_init(void)
{
	uint32_t threshold;
	uint16_t major;
	uint16_t minor;
	uint8_t tmp;

	if (atmel_app_read_fw_version(ATMEL_CMD_MAJOR_FW_VERSION, &major) < 0) {
		pr_err("%s(%d) Read mcu fw ver(major) fail, call reset mcu...\n",
			__func__, __LINE__);
		atmel_app_enter_application_mode();
	} else {
		if (atmel_app_read_fw_version(ATMEL_CMD_MINOR_FW_VERSION, &minor) < 0)
			pr_err("%s(%d) Read mcu fw ver(minor) fail...\n",
			__func__, __LINE__);
		else
			pr_err("%s: Firmware ver : %d%04d\n", __func__, major, minor);
	}

	if (atmel_app_get_data_32bit(ATMEL_CMD_GET_HINGE_THRESHOLD, &threshold) < 0)
		pr_err("%s(%d) Read hinge threshold fail...\n",
			__func__, __LINE__);
	else {
		atmel_app_dev->hinge_threshold = threshold;
		pr_err("%s: default hinge_threshold : %d\n", __func__, atmel_app_dev->hinge_threshold);
	}

	if (atmel_app_get_data_32bit(ATMEL_CMD_GET_CAMERA_KEY_THRESHOLD, &threshold) < 0)
		pr_err("%s(%d) Read camkey threshold fail...\n",
			__func__, __LINE__);
	else {
		atmel_app_dev->camkey_threshold = threshold;
		pr_err("%s: default camkey_threshold : %d\n", __func__, atmel_app_dev->camkey_threshold);
	}

	if (atmel_app_get_data_8bit(ATMEL_CMD_GET_HINGE_FUNC, &tmp) < 0)
		pr_err("%s(%d) Read camkey threshold fail...\n",
			__func__, __LINE__);
	if (atmel_app_dev->dis_hinge == 1) {
		atmel_app_dev->hinge_func = HINGE_FUNC_DISABLE;
		pr_err("%s: cmdline hinge_func : %d (Disable)[def=%d]\n",
			__func__, atmel_app_dev->hinge_func, tmp);
	} else {
		atmel_app_dev->hinge_func = HINGE_FUNC_ENABLE;
		pr_err("%s: cmdline hinge_func : %d (Enable)[def=%d]\n",
			__func__, atmel_app_dev->hinge_func, tmp);
	}

	if (atmel_app_read_power_state() == PWR_STATE_RECOVERY)
		atmel_app_dev->enter_recovery = 1;
	else
		atmel_app_dev->enter_recovery = 0;
	pr_err("%s: enter_recovery=%d\n", __func__, atmel_app_dev->enter_recovery);

	if (atmel_app_dev->offline_charging == 1) {
		if (atmel_app_change_power_state(PWR_STATE_OFF_CHARGING_ALIVE) < 0)
			pr_err("%s: Set OFF_CHARGING_ALIVE fail...\n", __func__);

		if (atmel_app_set_data_8bit(ATMEL_CMD_SET_HINGE_FUNC, atmel_app_dev->hinge_func) < 0)
			pr_err("%s(%d) atmel_app_set_data_8bit(HINGE_FUNC, %d) -- fail.\n",
				__func__, __LINE__, atmel_app_dev->hinge_func);
	}

	atmel_app_dev->hinge_status = HINGE_UNKNOWN;
}

#ifdef CONFIG_FB
static int atmel_app_dsi_panel_notifier_cb(struct notifier_block *self,
		unsigned long event, void *data)
{
	int transition;
	struct msm_drm_notifier *evdata = data;

	if (!evdata || (evdata->id != 0))
		return 0;

	if (evdata && evdata->data) {
		if (event == MSM_DRM_EVENT_BLANK) {
			transition = *(int *)evdata->data;
			if (transition == MSM_DRM_BLANK_POWERDOWN) {
				/* pr_err("%s : MSM_DRM_BLANK_POWERDOWN\n", __func__); */
				atmel_app_dev->fb_ready = false;
			} else if (transition == MSM_DRM_BLANK_UNBLANK) {
				/* pr_err("%s : MSM_DRM_BLANK_UNBLANK\n", __func__); */
				atmel_app_dev->fb_ready = true;
			}
		}
	}

	return 0;
}
#endif

static int atmel_app_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret;

	pr_info("%s - start\n",__func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s : need I2C_FUNC_I2C\n", __func__);
		return  -ENODEV;
	}
	
	atmel_app_dev = kzalloc(sizeof(*atmel_app_dev), GFP_KERNEL);
	if (atmel_app_dev == NULL) {
		pr_err("%s(%d) failed to allocate memory for module data\n",
			__func__, __LINE__);
		ret = -ENOMEM;
		goto err_exit;
	}
	
	atmel_app_dev->client = client;

	atmel_app_dev->hw_id = read_hw_id();
	pr_err("%s: hwid=%d\n",
		__func__, atmel_app_dev->hw_id);

	atmel_app_dev->evb_id = read_evb_id();
	pr_err("%s: evbid=%d\n",
		__func__, atmel_app_dev->evb_id);

	atmel_app_dev->dis_hinge= read_dis_hinge();
	pr_err("%s: dis_hinge=%d\n",
		__func__, atmel_app_dev->dis_hinge);

	ret = atmel_app_parse_dtree(client, atmel_app_dev);
	if (0 != ret)
		goto err_parse_dtree;

	atmel_app_config_gpio();

	wakeup_source_init(&atmel_app_dev->timeout_wake_lock, "timeout_suspend_lock");

	ret = sysfs_create_group(&atmel_app_dev->client->dev.kobj,
			&atmel_app_attr_group);
	if (ret) {
		pr_err("%s (%d)failed to register attiny sysfs\n", __func__, __LINE__);
		goto err_register_sysfs;
	}

	atmel_app_dev->mcu_wq = create_singlethread_workqueue("MCU workqueue");
	if (!atmel_app_dev->mcu_wq) {
		pr_err("%s(%d) Create MCU workqueue failed!\n",
			__func__, __LINE__);
		goto err_mcu_wq;
	}

	INIT_DELAYED_WORK(&atmel_app_dev->mcu_wq_task, atmel_app_delay_wq_task);

	atmel_app_dev->shutdown_wq = create_singlethread_workqueue("Shutdown workqueue");
	if (!atmel_app_dev->shutdown_wq) {
		pr_err("%s(%d) Create Shutdown workqueue failed!\n",
			__func__, __LINE__);
		goto err_shutdown_wq;
	}

	INIT_DELAYED_WORK(&atmel_app_dev->shutdown_wq_task, atmel_app_delay_shutdown_wq_task);

	atmel_app_dev->polling_time = 50;
	atmel_app_dev->camkey_threshold = 0xFFFF;
	atmel_app_dev->hinge_threshold = 0xFFFF;
	atmel_app_dev->hinge_func= 0xFF;
	atmel_app_dev->enter_standby = 0;
	atmel_app_dev->enter_factory = 0;

	atmel_app_dev->offline_charging= read_offline_charging();
	pr_err("%s: offline_charging=%d\n",
		__func__, atmel_app_dev->offline_charging);

	atmel_app_dev->input = input_allocate_device();
	if (!atmel_app_dev->input ) {
		pr_err("%s(%d) input_allocate_device failed!\n",
			__func__, __LINE__);
		goto err_input;
	}
	atmel_app_dev->input->name = "atmel_input";
	__set_bit(EV_KEY, atmel_app_dev->input->evbit);
	__set_bit(KEY_CAMERA, atmel_app_dev->input->keybit);
	__set_bit(KEY_WAKEUP, atmel_app_dev->input->keybit);
	__set_bit(EV_SW, atmel_app_dev->input->evbit);
	__set_bit(SW_LID, atmel_app_dev->input->swbit);
	device_set_wakeup_capable(&atmel_app_dev->input->dev, true);

	ret = input_register_device(atmel_app_dev->input);
	if (ret) {
		pr_err("%s(%d) input_register_device failed!\n",
			__func__, __LINE__);
		goto err_input;
	}

	atmel_app_mcu_init();

	atmel_app_dev->fb_ready = true;
#ifdef CONFIG_FB
	atmel_app_dev->fb_notifier.notifier_call = atmel_app_dsi_panel_notifier_cb;
	ret = msm_drm_register_client(&atmel_app_dev->fb_notifier);
	if (ret < 0) {
		pr_err("%s(%d) msm_drm_register_client failed!\n",
			__func__, __LINE__);
	}
#endif

	pr_info("%s - end\n",__func__);

	return 0;

err_input:
	destroy_workqueue(atmel_app_dev->shutdown_wq);
err_shutdown_wq:
	destroy_workqueue(atmel_app_dev->mcu_wq);
err_mcu_wq:
err_parse_dtree:
err_register_sysfs:
	wakeup_source_trash(&atmel_app_dev->timeout_wake_lock);
	kfree(atmel_app_dev);

err_exit:
	pr_err("%s(%d) err_exit\n",__func__, __LINE__);

	return ret;
}

static int atmel_app_remove(struct i2c_client *client)
{
	pr_info("%s\n",__func__);

#ifdef CONFIG_FB
	msm_drm_unregister_client(&atmel_app_dev->fb_notifier);
#endif

	wakeup_source_trash(&atmel_app_dev->timeout_wake_lock);
	sysfs_remove_group(&client ->dev.kobj, &atmel_app_attr_group);
	cancel_delayed_work(&atmel_app_dev->shutdown_wq_task);
	flush_workqueue(atmel_app_dev->shutdown_wq);
	destroy_workqueue(atmel_app_dev->shutdown_wq);
	cancel_delayed_work(&atmel_app_dev->mcu_wq_task);
	flush_workqueue(atmel_app_dev->mcu_wq);
	destroy_workqueue(atmel_app_dev->mcu_wq);
	input_unregister_device(atmel_app_dev->input);
	free_irq(atmel_app_dev->mcu_irq, atmel_app_dev);
	if (atmel_app_dev->evb_id  == 0) {
		gpio_unexport(atmel_app_dev->mcu_swd_clk_gpio);
		gpio_free(atmel_app_dev->mcu_swd_clk_gpio);
		gpio_unexport(atmel_app_dev->mcu_swd_dio_gpio);
		gpio_free(atmel_app_dev->mcu_swd_dio_gpio);
		gpio_unexport(atmel_app_dev->mcu_rst_gpio);
		gpio_free(atmel_app_dev->mcu_rst_gpio);
		gpio_unexport(atmel_app_dev->mcu_bld_en_gpio);
		gpio_free(atmel_app_dev->mcu_bld_en_gpio);
	} else {
		gpio_free(atmel_app_dev->mcu_rst_gpio);
		gpio_free(atmel_app_dev->mcu_bld_en_gpio);
	}
	kfree(atmel_app_dev);

	return 0;
}

static void atmel_app_shutdown(struct i2c_client *client)
{
	/* pr_info("%s\n",__func__);
	atmel_app_led_ctrl(0); */
}

#ifdef CONFIG_PM
static int atmel_app_suspend(struct device *dev)
{
	if (atmel_app_dev->enter_factory == 0 &&
		atmel_app_dev->enter_standby == 0 &&
		atmel_app_dev->enter_recovery== 0) {

		/* pr_info("%s\n",__func__); */

		atmel_app_mcu_state_check();

		if (atmel_app_dev->offline_charging == 0) {
			if (atmel_app_change_power_state(PWR_STATE_SLEEP) < 0)
				pr_err("%s(%d) Set SLEEP fail...\n",
					__func__, __LINE__);
		} else {
			if (atmel_app_change_power_state(PWR_STATE_OFF_CHARGING_SLEEP) < 0)
				pr_err("%s(%d) Set OFF_CHARGING_SLEEP fail...\n",
					__func__, __LINE__);
		}
	}

	return 0;
}

static int atmel_app_resume(struct device *dev)
{
	if (atmel_app_dev->enter_factory == 0 &&
		atmel_app_dev->enter_standby == 0 &&
		atmel_app_dev->enter_recovery== 0) {

		/* pr_info("%s\n",__func__); */

		atmel_app_mcu_state_check();

		if (atmel_app_dev->offline_charging == 0) {
			if (atmel_app_change_power_state(PWR_STATE_ALIVE) < 0)
				pr_err("%s(%d) Set ALIVE fail...\n",
					__func__, __LINE__);
		} else {
			if (atmel_app_change_power_state(PWR_STATE_OFF_CHARGING_ALIVE) < 0)
				pr_err("%s(%d) Set OFF_CHARGING_ALIVE fail...\n",
					__func__, __LINE__);
		}

		atmel_app_notify_hinge_status();
	}

	return 0;
}

static const struct dev_pm_ops atmel_app_pm = {
	.suspend        = atmel_app_suspend,
	.resume         = atmel_app_resume,
};

#define atmel_app_pm_ops (&atmel_app_pm)
#else
#define atmel_app_pm_ops NULL
#endif

static struct of_device_id atmel_app_dt_match[] = {
	{ .compatible = "atmel,atmel_app_i2c",},
	{ },
};


static const struct i2c_device_id atmel_app_id[] = {
	{ "atmel_app_i2c", 0 },
	{ }
};

static struct i2c_driver atmel_app_driver = {
	.id_table		= atmel_app_id,
	.probe		= atmel_app_probe,
	.remove		= atmel_app_remove,
	.shutdown	= atmel_app_shutdown,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "atmel_app_i2c",
		.of_match_table = atmel_app_dt_match,
		.pm	= atmel_app_pm_ops,
	},
};

static int __init atmel_app_dev_init(void)
{
	pr_info("Loading atmel_app driver\n");
	return i2c_add_driver(&atmel_app_driver);
}
module_init(atmel_app_dev_init);

static void __exit atmel_app_dev_exit(void)
{
	pr_info("Unloading atmel_app driver\n");
	i2c_del_driver(&atmel_app_driver);
}
module_exit(atmel_app_dev_exit);

MODULE_AUTHOR("Quanta");
MODULE_DESCRIPTION("Atmel Application I2C driver");
MODULE_LICENSE("GPL");
