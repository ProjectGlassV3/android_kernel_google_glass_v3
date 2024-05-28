/*
 * Nuvoton EC interface I2C Chip Driver
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see http://www.gnu.org/licenses/>.
 *
 */

#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fcntl.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/hwid.h> //add for HW ID use
#include <linux/i2c.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/sched.h>
#include <linux/slab.h>

#define DEFAULT_I2C_DEV 0

extern void set_esd_backlight(void);

struct a254_chip {
	/* power sequence is 1.2v, 1.8v, 4.5v, 6.0v */
	struct regulator *a254_pwr1p2;
	struct regulator *a254_pwr1p8;
	struct regulator *a254_pwr4p5;
	struct regulator *a254_pwr6p0;
	struct device *dev;
	struct delayed_work work; // andy0x, add workqueue
	struct workqueue_struct *workqueue; // andy0x, add workqueue
	int en_pwr1p2_gpio; /* gpio to enable power 1.2v */
	int a254_reset_gpio; /* a254 reset pin */
	bool en_a254_reset_low; /* enable to set reset pin to low */
	bool en_reset_high_detect; /* detect reset pin is high or low */
	bool disp_drv_id; /* High: A255, Low:A254 */
	int board_id; /* 0:EVB, 1:EVT */
};

static struct a254_chip *this_chip = NULL;
static bool need_post_power_on = false;

static int a254_reset_high_detect(struct a254_chip *chip)
{
#define DISP_DRV_ID_PIN 95
#define RESET_HIGH_PIN 89

	int disp_id = 0;
	int reset_high = 0;
	int rc = 0;

	chip->en_a254_reset_low = 1;
	chip->board_id = read_hw_id(); // board id read

	pr_debug("board id: 0x%08x\n", chip->board_id);
	if (chip->board_id == 0) {
		if (gpio_is_valid(RESET_HIGH_PIN)) {
			rc = gpio_request(RESET_HIGH_PIN, "RESET_HIGH_PIN");
			if (rc) {
				pr_err("request RESET_HIGH_PIN gpio failed, rc=%d\n",
				       rc);
				return rc;
				;
			} else {
				gpio_direction_input(RESET_HIGH_PIN);
				reset_high = gpio_get_value(RESET_HIGH_PIN);
				gpio_free(RESET_HIGH_PIN);
			}
		}
		if (reset_high == 1) // MLB
		{
			chip->en_a254_reset_low = 0;
			pr_debug("detect a254 reset high\n");
		}
	} else if (chip->board_id == 1) { // add for EVT
		if (gpio_is_valid(DISP_DRV_ID_PIN)) {
			rc = gpio_request(DISP_DRV_ID_PIN, "DISP_DRV_ID_PIN");
			if (rc) {
				pr_err("request DISP_DRV_ID_PIN gpio failed, rc=%d\n",
				       rc);
				return rc;
				;
			} else {
				gpio_direction_input(DISP_DRV_ID_PIN);
				disp_id = gpio_get_value(DISP_DRV_ID_PIN);
				gpio_free(DISP_DRV_ID_PIN);
			}

			if (disp_id == 1)
				chip->disp_drv_id = 1; // EVT, A255
			else
				chip->disp_drv_id = 0; // EVT, A254

			chip->en_a254_reset_low = 1; // EVT, ML board

		} else {
			pr_err("request disp_id gpio is not valid\n");
		}
	}

	pr_info("%s: disp drv id: 0x%x, chip_rst: 0x%x\n", __func__,
		chip->disp_drv_id, chip->en_a254_reset_low);

	return rc;
}

int a254_send_receive(uint8_t command, uint8_t value, uint8_t *data,
		      int data_len)
{
	struct i2c_adapter *a;
	int ret = 0;

	struct i2c_msg msgs[2];
	uint8_t cmd_buffer[2];
	int num_xfers = 1;

	cmd_buffer[0] = command;
	cmd_buffer[1] = value;
	msgs[0].addr = 0x34;
	msgs[0].flags = 0;
	msgs[0].buf = (char *)cmd_buffer;
	msgs[0].len = 2;
	if (data != 0) {
		pr_debug("%s: A254 I2C read\n", __func__);
		msgs[0].len =
			1; // andy0x, for read, msg[0].len needs to change to 1
		msgs[1].addr = 0x34;
		msgs[1].flags = I2C_M_RD;
		msgs[1].buf = (char *)data;
		msgs[1].len = data_len;
		num_xfers = 2;
	}

	// mutex_lock(&a254_i2c_mut);

	a = i2c_get_adapter(DEFAULT_I2C_DEV);
	if (!a) {
		ret = -ENODEV;
		goto a254_i2c_err;
	}

	ret = i2c_transfer(a, msgs, num_xfers);

	/* SET commands: caller defined value with data[0] */
a254_i2c_err:
	i2c_put_adapter(a);
	// mutex_unlock(&a254_i2c_mut);
	return ret >= 0;
}

void a254_init(void)
{
	pr_info("%s\n", __func__);
	a254_send_receive(0x00, 0x00, 0, 0);
	a254_send_receive(0x01, 0x00, 0, 0);
	a254_send_receive(0x02, 0x00, 0, 0);
	a254_send_receive(0x03, 0x86, 0, 0);
	a254_send_receive(0x04, 0x00, 0, 0);
	a254_send_receive(0x05, 0x44, 0, 0);
	a254_send_receive(0x06, 0x70, 0, 0);
	a254_send_receive(0x07, 0x00, 0, 0);
	a254_send_receive(0x08, 0x00, 0, 0);
	a254_send_receive(0x09, 0x00, 0, 0);
	a254_send_receive(0x0A, 0x03, 0, 0);
	a254_send_receive(0x0B, 0x88, 0, 0);
	a254_send_receive(0x0C, 0x00, 0, 0);
	a254_send_receive(0x0D, 0x00, 0, 0);
	a254_send_receive(0x0E, 0x00, 0, 0);
	a254_send_receive(0x0F, 0x00, 0, 0);
}

static int a254_regulator_gpio_init(struct a254_chip *chip)
{
	int rc = 0;

	if (gpio_is_valid(chip->en_pwr1p2_gpio)) {
		rc = gpio_request(chip->en_pwr1p2_gpio, "a254-1p2-en");
		if (rc) {
			pr_err("request pwr1p2 gpio failed, rc=%d\n", rc);
			// return rc;;
		} else {
			rc = gpio_direction_output(chip->en_pwr1p2_gpio, 1);
		}
		gpio_free(chip->en_pwr1p2_gpio);
	}

	if (gpio_is_valid(chip->a254_reset_gpio)) {
		rc = gpio_request(chip->a254_reset_gpio, "a254-reset-gpio");
		if (rc) {
			pr_err("request reset gpio failed, rc=%d\n", rc);
			return rc;
			;
		}
		if (chip->en_a254_reset_low)
			rc = gpio_direction_output(chip->a254_reset_gpio, 0);
		else
			rc = gpio_direction_output(chip->a254_reset_gpio, 1);

		gpio_export(chip->a254_reset_gpio, true);
		gpio_free(chip->a254_reset_gpio);
	}

	return rc;
}

static int a254_regulator_hw_init(struct a254_chip *chip)
{
	int rc = 0;

	if (chip->en_reset_high_detect)
		a254_reset_high_detect(chip);
	else
		chip->en_a254_reset_low = 1;

	rc = a254_regulator_gpio_init(chip);
	if (rc) {
		pr_err("gpios initialize failed, rc = %d\n", rc);
		return rc;
	}

	if (chip->a254_pwr1p2) {
		if (regulator_count_voltages(chip->a254_pwr1p2) > 0) {
			rc = regulator_set_voltage(chip->a254_pwr1p2, 1350000,
						   1350000);
			if (rc < 0) {
				pr_err("set a254_pwr1p2 voltage failed, rc = %d\n",
				       rc);
				return rc;
			}
		}
		rc = regulator_enable(chip->a254_pwr1p2);
		if (rc) {
			pr_err("enable a254_pwr1p2 voltage failed, rc = %d\n",
			       rc);
			return rc;
		}
		usleep_range(100, 100);
	}

	if (chip->a254_pwr1p8) {
		if (regulator_count_voltages(chip->a254_pwr1p8) > 0) {
			rc = regulator_set_voltage(chip->a254_pwr1p8, 1800000,
						   1800000);
			if (rc < 0) {
				pr_err("set a254_pwr1p8 voltage failed, rc = %d\n",
				       rc);
				return rc;
			}
		}
		rc = regulator_enable(chip->a254_pwr1p8);
		if (rc) {
			pr_err("enable a254_pwr1p8 voltage failed, rc = %d\n",
			       rc);
			return rc;
		}
		usleep_range(100, 100);
	}

	if (chip->a254_pwr4p5) {
		regulator_count_voltages(chip->a254_pwr4p5);
		rc = regulator_set_voltage(chip->a254_pwr4p5, 4500000, 4500000);
		if (rc < 0) {
			pr_err("set a254_pwr4p5 voltage failed, rc = %d\n", rc);
			return rc;
		}
		rc = regulator_enable(chip->a254_pwr4p5);
		if (rc) {
			pr_err("enable a254_pwr4p5 voltage failed, rc = %d\n",
			       rc);
			return rc;
		}
	}

	if (chip->a254_pwr6p0) {
		regulator_count_voltages(chip->a254_pwr6p0);
		rc = regulator_set_voltage(chip->a254_pwr6p0, 6000000, 6000000);
		if (rc < 0) {
			pr_err("set a254_pwr6p0 voltage failed, rc = %d\n", rc);
			return rc;
		}
		rc = regulator_enable(chip->a254_pwr6p0);
		if (rc) {
			pr_err("enable a254_pwr6p0 voltage failed, rc = %d\n",
			       rc);
			return rc;
		}
	}

	return rc;
}

static int a254_parse_dt(struct a254_chip *chip, struct i2c_client *client)
{
	int rc = 0;
	if (!client->dev.of_node) {
		pr_err("device node missing\n");
		return -EINVAL;
	}

	chip->en_pwr1p2_gpio =
		of_get_named_gpio(client->dev.of_node, "a254,1p2-gpio", 0);

	chip->a254_reset_gpio =
		of_get_named_gpio(client->dev.of_node, "a254,reset-gpio", 0);

	chip->en_reset_high_detect = of_property_read_bool(
		client->dev.of_node, "a254,reset-high-detect");

	if (of_find_property(client->dev.of_node, "a254-1p2-supply", NULL)) {
		chip->a254_pwr1p2 =
			devm_regulator_get(&client->dev, "a254-1p2");
		if (IS_ERR_OR_NULL(chip->a254_pwr1p2)) {
			rc = PTR_RET(chip->a254_pwr1p2);
			if (rc != EPROBE_DEFER)
				pr_err("get a254_pwr1p2 failed, rc = %d\n", rc);
			return rc;
		}
	}
	if (of_find_property(client->dev.of_node, "a254-1p8-supply", NULL)) {
		chip->a254_pwr1p8 =
			devm_regulator_get(&client->dev, "a254-1p8");
		if (IS_ERR_OR_NULL(chip->a254_pwr1p8)) {
			rc = PTR_RET(chip->a254_pwr1p8);
			if (rc != EPROBE_DEFER)
				pr_err("get a254_pwr1p8 failed, rc = %d\n", rc);
			return rc;
		}
	}
	if (of_find_property(client->dev.of_node, "a254-4p5-supply", NULL)) {
		chip->a254_pwr4p5 =
			devm_regulator_get(&client->dev, "a254-4p5");
		if (IS_ERR_OR_NULL(chip->a254_pwr4p5)) {
			rc = PTR_RET(chip->a254_pwr4p5);
			if (rc != EPROBE_DEFER)
				pr_err("get a254_pwr4p5 failed, rc = %d\n", rc);
			return rc;
		}
	}
	if (of_find_property(client->dev.of_node, "a254-6p0-supply", NULL)) {
		chip->a254_pwr6p0 =
			devm_regulator_get(&client->dev, "a254-6p0");
		if (IS_ERR_OR_NULL(chip->a254_pwr6p0)) {
			rc = PTR_RET(chip->a254_pwr6p0);
			if (rc != EPROBE_DEFER)
				pr_err("get a254_pwr6p0 failed, rc = %d\n", rc);
			return rc;
		}
	}

	return rc;
}

static void A254_work_func(struct work_struct *work)
{
	// struct a254_chip *td = container_of(work, struct a254_chip, work.work);
	// int workqueu_timer;
	// uint8_t buf[2];

	pr_debug("%s\n", __func__);

	/* memset(buf, 0, 2);

   a254_send_receive(0x03, 0, buf, 1);
   pr_err("[%s] ----0x03: %x \n", __func__, buf[0]);

   if (buf[0] ==0x02) {
       pr_err("[%s] ---- A254 is reset by ESD, re-init it.\n", __func__);
       a254_init();
       set_esd_backlight();		//andy0x, set backlight
       workqueu_timer = 5000;
   } else {
       pr_err("[%s] ---- A254 is Alive. \n", __func__);
       set_esd_backlight();		//andy0x, set backlight
       workqueu_timer = 10000;
       }

   queue_delayed_work(td->workqueue,
   &td->work,msecs_to_jiffies(workqueu_timer));*/
}

static int a254_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct a254_chip *chip;
	int rc = 0;

	chip = devm_kzalloc(&client->dev, sizeof(struct a254_chip), GFP_KERNEL);
	if (!chip) {
		pr_err("memory allocation failed for a254_chip\n");
		return -ENOMEM;
	}

	rc = a254_parse_dt(chip, client);
	if (rc < 0) {
		pr_err("parse device tree failed for a254, rc = %d\n", rc);
		return rc;
	}
	chip->dev = &client->dev;

	rc = a254_regulator_hw_init(chip);
	if (rc < 0) {
		pr_err("hardware init failed for a254, rc = %d\n", rc);
		return rc;
	}
	i2c_set_clientdata(client, chip);

	// a254_init();

	INIT_DELAYED_WORK(&chip->work, A254_work_func);

	chip->workqueue = create_singlethread_workqueue("A254_status_work");
	if (chip->workqueue == NULL) {
		pr_err("%s: failed to create work queue\n", __func__);
	} else {
		queue_delayed_work(chip->workqueue, &chip->work, 1000);
	}
	this_chip = chip;

	return 0;
}

static int a254_remove(struct i2c_client *i2c)
{
	struct a254_data *data = i2c_get_clientdata(i2c);
	devm_kfree(&i2c->dev, data);
	this_chip = NULL;
	return 0;
}

int a254_power_off(void)
{
	int rc = 0, num_of_v = 0;
	struct a254_chip *chip = this_chip;

	pr_debug("%s: enter\n", __func__);

	if (!chip)
		return -ENODEV;

	rc = gpio_request(chip->en_pwr1p2_gpio, "a254-1p2-en");
	if (rc) {
		pr_err("%s: request en_pwr1p2_gpio gpio failed, rc=%d\n",
		       __func__, rc);
	} else {
		rc = gpio_direction_output(chip->en_pwr1p2_gpio, 0);
	}
	gpio_free(chip->en_pwr1p2_gpio);
	pr_debug("%s: 1P2 OFF\n", __func__);

	if (chip->a254_pwr1p8) {
		num_of_v = regulator_count_voltages(chip->a254_pwr1p8);
		pr_debug("%s: 1p8 = %dmV\n", __func__, num_of_v);
	}
	rc = regulator_disable(chip->a254_pwr1p8);
	if (rc) {
		pr_err("%s: disable a254_pwr1p8 voltage failed, rc = %d\n",
		       __func__, rc);
		return rc;
	}
	pr_debug("%s: 1P8 OFF\n", __func__);

	if (chip->a254_pwr4p5) {
		rc = regulator_disable(chip->a254_pwr4p5);
		if (rc) {
			pr_err("%s: disable a254_pwr4p5 voltage failed, rc = %d\n",
			       __func__, rc);
			return rc;
		}
		regulator_set_voltage(chip->a254_pwr4p5, 0, 4500000);
	}
	pr_debug("%s: 4P5 OFF\n", __func__);

	if (chip->a254_pwr6p0) {
		rc = regulator_disable(chip->a254_pwr6p0);
		if (rc) {
			pr_err("%s: disable a254_pwr6p0 voltage failed, rc = %d\n",
			       __func__, rc);
			return rc;
		}
		regulator_set_voltage(chip->a254_pwr6p0, 0, 6000000);
	}
	pr_debug("%s: 6P0 OFF\n", __func__);

	pr_debug("%s: exit\n", __func__);
	return 0;
}
EXPORT_SYMBOL(a254_power_off);

int a254_power_on(void)
{
	int rc = 0;
	struct a254_chip *chip = this_chip;

	pr_debug("%s: enter\n", __func__);

	if (!chip)
		return -ENODEV;

	rc = gpio_request(chip->en_pwr1p2_gpio, "a254-1p2-en");
	if (rc) {
		pr_err("%s: request en_pwr1p2_gpio gpio failed, rc=%d\n",
		       __func__, rc);
	} else {
		rc = gpio_direction_output(chip->en_pwr1p2_gpio, 1);
	}
	gpio_free(chip->en_pwr1p2_gpio);
	pr_debug("%s: 1P2 ON\n", __func__);

	rc = regulator_enable(chip->a254_pwr1p8);
	if (rc) {
		pr_err("%s: enable a254_pwr1p8 voltage failed, rc = %d\n",
		       __func__, rc);
		return rc;
	}
	pr_debug("%s: 1P8 ON\n", __func__);

	gpio_set_value(chip->a254_reset_gpio, 1);
	need_post_power_on = true;

	pr_debug("%s: exit\n", __func__);

	return 0;
}
EXPORT_SYMBOL(a254_power_on);

int a254_post_power_on(void)
{
	int rc = 0;
	struct a254_chip *chip = this_chip;

	pr_debug("%s: enter\n", __func__);

	if (!chip)
		return -ENODEV;

	if (!need_post_power_on)
		return 0;

	need_post_power_on = false;
	usleep_range(100, 100);
	gpio_set_value(chip->a254_reset_gpio, 0);
	usleep_range(1000, 1000);
	pr_debug("%s: reset A254\n", __func__);

	a254_init();
	usleep_range(10000, 10000);

	if (chip->a254_pwr4p5) {
		rc = regulator_set_voltage(chip->a254_pwr4p5, 4500000, 4500000);
		if (rc) {
			pr_err("%s: set a254_pwr4p5 voltage failed\n",
			       __func__);
			return rc;
		}

		rc = regulator_enable(chip->a254_pwr4p5);
		if (rc) {
			pr_err("%s: enable a254_pwr4p5 voltage failed, rc = %d\n",
			       __func__, rc);
			return rc;
		}
	}
	pr_debug("%s: 4P5 ON\n", __func__);

	if (chip->a254_pwr6p0) {
		rc = regulator_set_voltage(chip->a254_pwr6p0, 6000000, 6000000);
		if (rc) {
			pr_err("%s: set a254_pwr6p0 voltage failed\n",
			       __func__);
			return rc;
		}

		rc = regulator_enable(chip->a254_pwr6p0);
		if (rc) {
			pr_err("%s: enable a254_pwr6p0 voltage failed, rc = %d\n",
			       __func__, rc);
			return rc;
		}
	}
	pr_debug("%s: 6P0 ON\n", __func__);

	pr_debug("%s: exit\n", __func__);
	return 0;
}
EXPORT_SYMBOL(a254_post_power_on);

static const struct i2c_device_id a254_id[] = { { "a254", 0 }, {} };

#ifdef CONFIG_OF
static const struct of_device_id a254_of_match[] = {
	{
		.compatible = "asic,a254",
	},
	{},
};
MODULE_DEVICE_TABLE(of, a254_of_match);
#endif

static struct i2c_driver a254_driver = {
    .driver =
        {
            .name = "a254",
            .pm = NULL,
            .of_match_table = of_match_ptr(a254_of_match),
        },
    .probe = a254_probe,
    .remove = a254_remove,
    .id_table = a254_id,
};

module_i2c_driver(a254_driver);

MODULE_DESCRIPTION("Kopin A254 ASIC driver");
MODULE_LICENSE("GPL v2");
