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
#include <linux/hwid.h>
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
#include <misc/machxo.h>

#define ceil(x, y) (x / y + (x % y != 0))
#define i2c_buf_length 2048

// static DEFINE_MUTEX(fpga_i2c_mut);

struct lifmd6000_chip {
	struct regulator *fpga_pwr1p2; /*Add for fpga, a254*/
	struct regulator *fpga_pwr1p8; /*Add for fpga, a254*/
	struct regulator *fpga_pwr2p5; /*Add for fpga*/
	struct device *dev;
	int en_pwr1p2_gpio; /*Add for fpga, a254*/
	int fpga_cresetb_gpio; /* fpga CRSETB pin */
	int fpga_cdone_gpio;
	bool fpga_programming_enable; /* enable programming fpga */
};

extern const unsigned char g_fpgaDataArray[];
extern const int g_fpgaDataSize;
static struct lifmd6000_chip *this_chip = NULL;

int lifmd6000_send_receive(uint8_t command, uint32_t operand, int direction,
			   uint8_t *data, int data_len)
{
	struct i2c_adapter *a;
	int ret = 0;

	struct i2c_msg msgs[100];
	uint8_t cmd_buffer[4];
	int num_xfers = 1;
	int oplen = 4, i;

	switch (command) {
	case ISC_ENABLE:
	case ISC_ENABLE_X:
	case ISC_DISABLE:
	case LSC_REFRESH:
		oplen = 3;
		break;
	default:
		oplen = 4;
		break;
	}
	cmd_buffer[0] = command;
	cmd_buffer[1] = (operand & 0xFF0000) >> 16;
	cmd_buffer[2] = (operand & 0x00FF00) >> 8;
	cmd_buffer[3] = (operand & 0x0000FF);

	msgs[0].addr = 0x40;
	msgs[0].flags = 0;
	msgs[0].buf = (char *)cmd_buffer;
	msgs[0].len = command == ISC_ENABLE ? 3 : 4;
	if (data != 0) {
		for (i = 0; i < ceil(data_len, i2c_buf_length); i++) {
			msgs[i + 1].addr = 0x40;
			if (i > 0)
				msgs[i + 1].flags =
					direction == DIRECTION_SEND ? 0 :
									    I2C_M_RD;
			else
				msgs[i + 1].flags =
					(direction == DIRECTION_SEND ?
						       0 :
						       I2C_M_RD) |
					I2C_M_NOSTART;

			if (ceil((data_len - (i * i2c_buf_length)),
				 i2c_buf_length) > 1)
				msgs[i + 1].len = i2c_buf_length;
			else
				msgs[i + 1].len =
					data_len - (i * i2c_buf_length);
			msgs[i + 1].buf = data + i * i2c_buf_length;

			num_xfers = i + 2;
		}
	}

	// mutex_lock(&fpga_i2c_mut);

	a = i2c_get_adapter(DEFAULT_I2C_DEV);
	if (!a) {
		ret = -ENODEV;
		goto fpga_i2c_err;
	}

	ret = i2c_transfer(a, msgs, num_xfers);

	/* SET commands: caller defined value with data[0] */
fpga_i2c_err:
	i2c_put_adapter(a);
	// mutex_unlock(&fpga_i2c_mut);
	return ret >= 0;
}

int lifmd6000_run_upgrade(struct lifmd6000_chip *chip)
{
	int i, rc = 0;
	uint8_t buffer[4];
	uint8_t *pfpga_data = (uint8_t *)g_fpgaDataArray;
	int fpga_data_size = g_fpgaDataSize;

	pr_info("%s: enter\n", __func__);

	// 1).key activation
	gpio_set_value(chip->fpga_cresetb_gpio, 0);
	usleep_range(100000, 100000);
	rc = lifmd6000_send_receive(KEY_ACTIVATION, 0xC6F48A, DIRECTION_SEND, 0,
				    0);
	if (rc != 1)
		return -1;
	gpio_set_value(chip->fpga_cresetb_gpio, 1);
	usleep_range(10000, 10000);

	// 2).read IDCODE (0xE0)
	rc = lifmd6000_send_receive(IDCODE_PUB, 0, DIRECTION_RECEIVE, buffer,
				    4);
	pr_debug("%s: ID: 0x%02x%02x%02x%02x\n", __func__, buffer[0], buffer[1],
		 buffer[2], buffer[3]);
	if (rc != 1)
		return -1;

	// 3).enter programming mode (0xC6)
	rc = lifmd6000_send_receive(ISC_ENABLE, 0, DIRECTION_SEND, 0,
				    0); /* TODO: special command for i2c */
	if (rc != 1)
		return -1;
	usleep_range(1000, 1000);

	// 4).erase_sram (0x0E)
	rc = lifmd6000_send_receive(ISC_ERASE, 0, DIRECTION_SEND, 0, 0);
	if (rc != 1)
		return -1;
	usleep_range(100000, 100000);
	// 5).reset_configuration_flash_address (0x46)
	rc = lifmd6000_send_receive(LSC_INIT_ADDRESS, 0, DIRECTION_SEND, 0, 0);
	if (rc != 1)
		return -1;

	/*program_configuration_sram */
	pr_info("%s: fpga_data_size = %d\n", __func__, fpga_data_size);
	rc = lifmd6000_send_receive(LSC_BITSTREAM_BURST, 0, DIRECTION_SEND,
				    pfpga_data, fpga_data_size);
	if (rc != 1)
		return -1;
	/* read_status_register */
	for (i = 0; i < 5; i++) {
		usleep_range(10000, 10000);
		rc = lifmd6000_send_receive(LSC_READ_STATUS, 0,
					    DIRECTION_RECEIVE, buffer, 4);
		if (rc != 1)
			return -1;
		pr_debug("%s: status: 0x%02x%02x%02x%02x\n", __func__,
			 buffer[0], buffer[1], buffer[2], buffer[3]);
	}

	// ).exit sram programming mode
	usleep_range(1000000, 1000000);
	rc = lifmd6000_send_receive(ISC_DISABLE, 0, DIRECTION_SEND, 0, 0);
	if (rc != 1)
		return -1;

	pr_debug("%s: exit\n", __func__);
	return rc;
}

static int lifmd6000_regulator_gpio_init(struct lifmd6000_chip *chip)
{
	int rc = 0;

	if (gpio_is_valid(chip->en_pwr1p2_gpio)) {
		rc = gpio_request(chip->en_pwr1p2_gpio, "fpga_1p2_en");
		if (rc) {
			pr_err("request en_pwr1p2_gpio failed, rc=%d\n", rc);
		} else {
			rc = gpio_direction_output(chip->en_pwr1p2_gpio, 1);
		}

		gpio_export(chip->en_pwr1p2_gpio, true);
		gpio_free(chip->en_pwr1p2_gpio);
	}

	if (gpio_is_valid(chip->fpga_cresetb_gpio)) {
		rc = gpio_request(chip->fpga_cresetb_gpio, "fpga_cresetb_pin");
		if (rc) {
			pr_err("request fpga_cresetb_gpio failed, rc=%d\n", rc);
			return rc;
		}
		rc = gpio_direction_output(chip->fpga_cresetb_gpio, 1);

		gpio_export(chip->fpga_cresetb_gpio, true);
		gpio_free(chip->fpga_cresetb_gpio);
	}

	return rc;
}

static int lifmd6000_regulator_hw_init(struct lifmd6000_chip *chip)
{
	int rc = 0;

	rc = lifmd6000_regulator_gpio_init(chip);
	if (rc) {
		pr_err("gpios initialize failed, rc = %d\n", rc);
		return rc;
	}

	if (chip->fpga_pwr2p5) {
		if (regulator_count_voltages(chip->fpga_pwr2p5) > 0) {
			rc = regulator_set_voltage(chip->fpga_pwr2p5, 2500000,
						   2500000);
			if (rc < 0) {
				pr_err("set fpga_pwr2p5 voltage failed, rc = %d\n",
				       rc);
				return rc;
			}
		}
		rc = regulator_enable(chip->fpga_pwr2p5);
		if (rc) {
			pr_err("enable fpga_pwr2p5 voltage failed, rc = %d\n",
			       rc);
			return rc;
		}
	}

	if (chip->fpga_pwr1p2) {
		if (regulator_count_voltages(chip->fpga_pwr1p2) > 0) {
			rc = regulator_set_voltage(chip->fpga_pwr1p2, 1350000,
						   1350000);
			if (rc < 0) {
				pr_err("set fpga_pwr1p2 voltage failed, rc = %d\n",
				       rc);
				return rc;
			}
		}
		rc = regulator_enable(chip->fpga_pwr1p2);
		if (rc) {
			pr_err("enable fpga_pwr1p2 voltage failed, rc = %d\n",
			       rc);
			return rc;
		}
		usleep_range(100, 100);
	}

	if (chip->fpga_pwr1p8) {
		if (regulator_count_voltages(chip->fpga_pwr1p8) > 0) {
			rc = regulator_set_voltage(chip->fpga_pwr1p8, 1800000,
						   1800000);
			if (rc < 0) {
				pr_err("set fpga_pwr1p8 voltage failed, rc = %d\n",
				       rc);
				return rc;
			}
		}
		rc = regulator_enable(chip->fpga_pwr1p8);
		if (rc) {
			pr_err("enable fpga_pwr1p8 voltage failed, rc = %d\n",
			       rc);
			return rc;
		}
		usleep_range(100, 100);
	}

	return rc;
}

static int lifmd6000_parse_dt(struct lifmd6000_chip *chip,
			      struct i2c_client *client)
{
	int rc = 0;
	if (!client->dev.of_node) {
		pr_err("device node missing\n");
		return -EINVAL;
	}

	chip->en_pwr1p2_gpio =
		of_get_named_gpio(client->dev.of_node, "fpga,1p2-gpio", 0);

	chip->fpga_cresetb_gpio =
		of_get_named_gpio(client->dev.of_node, "fpga,cresetb-gpio", 0);

	chip->fpga_cdone_gpio =
		of_get_named_gpio(client->dev.of_node, "fpga,cdone-gpio", 0);

	chip->fpga_programming_enable = of_property_read_bool(
		client->dev.of_node, "fpga,programming-enable");

	if (of_find_property(client->dev.of_node, "fpga-1p2-supply", NULL)) {
		chip->fpga_pwr1p2 =
			devm_regulator_get(&client->dev, "fpga-1p2");
		if (IS_ERR_OR_NULL(chip->fpga_pwr1p2)) {
			rc = PTR_RET(chip->fpga_pwr1p2);
			if (rc != EPROBE_DEFER)
				pr_err("get fpga_pwr1p2 failed, rc = %d\n", rc);
			return rc;
		}
	}
	if (of_find_property(client->dev.of_node, "fpga-1p8-supply", NULL)) {
		chip->fpga_pwr1p8 =
			devm_regulator_get(&client->dev, "fpga-1p8");
		if (IS_ERR_OR_NULL(chip->fpga_pwr1p8)) {
			rc = PTR_RET(chip->fpga_pwr1p8);
			if (rc != EPROBE_DEFER)
				pr_err("get fpga_pwr1p2 failed, rc = %d\n", rc);
			return rc;
		}
	}
	if (of_find_property(client->dev.of_node, "fpga-2p5-supply", NULL)) {
		chip->fpga_pwr2p5 =
			devm_regulator_get(&client->dev, "fpga-2p5");
		if (IS_ERR_OR_NULL(chip->fpga_pwr2p5)) {
			rc = PTR_RET(chip->fpga_pwr2p5);
			if (rc != EPROBE_DEFER)
				pr_err("get fpga_pwr1p2 failed, rc = %d\n", rc);
			return rc;
		}
	}

	return rc;
}

static int lifmd6000_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct lifmd6000_chip *chip;
	int rc = 0, hwid = 0, i, cdone = 0;

	chip = devm_kzalloc(&client->dev, sizeof(struct lifmd6000_chip),
			    GFP_KERNEL);
	if (!chip) {
		pr_err("memory allocation failed for lifmd6000_chip\n");
		return -ENOMEM;
	}
	rc = lifmd6000_parse_dt(chip, client);
	if (rc < 0) {
		pr_err("parse device tree failed for lifmd6000, rc = %d\n", rc);
		return rc;
	}
	chip->dev = &client->dev;

	rc = lifmd6000_regulator_hw_init(chip);
	if (rc < 0) {
		pr_err("hardware init failed for lifmd6000, rc = %d\n", rc);
		return rc;
	}
	i2c_set_clientdata(client, chip);

	hwid = read_hw_id();
	pr_debug("%s: HWID 0x%x\n", __func__, hwid);
	if (hwid == 0 && chip->fpga_programming_enable) { // I2C
		pr_debug("%s: I2C mode\n", __func__);
		lifmd6000_run_upgrade(chip);
	} else { // SPI Flash
		pr_debug("%s: SPI Flash mode\n", __func__);
	}

	if (gpio_is_valid(chip->fpga_cdone_gpio)) {
		rc = gpio_request(chip->fpga_cdone_gpio, "fpga_cdone_pin");
		if (rc) {
			pr_err("request fpga_cdone_gpio failed, rc=%d\n", rc);
			return rc;
		}

		pr_debug("%s: check CDONE status\n", __func__);
		gpio_direction_input(chip->fpga_cdone_gpio);
		for (i = 0; i < 3000; i++) {
			usleep_range(1000, 1000);
			cdone = gpio_get_value(chip->fpga_cdone_gpio);

			if (cdone == 1) {
				pr_debug("%s: CDONE High take %d ms\n",
					 __func__, i);
				break;
			}
		}
		/* BUG?  what if cdone doens't go high ? */
		gpio_free(chip->fpga_cdone_gpio);
	}
	this_chip = chip;
	return 0;
}

static int lifmd6000_remove(struct i2c_client *i2c)
{
	struct lifmd6000_data *data = i2c_get_clientdata(i2c);
	devm_kfree(&i2c->dev, data);
	this_chip = NULL;
	return 0;
}

int lifmd6000_power_off(void)
{
	int rc = 0, num_of_v = 0;
	struct lifmd6000_chip *chip = this_chip;

	pr_debug("%s: enter\n", __func__);

	if (!chip)
		return -ENODEV;

	if (chip->fpga_pwr2p5) {
		num_of_v = regulator_count_voltages(chip->fpga_pwr2p5);
		pr_debug("%s: 2p5 = %dmV \n", __func__, num_of_v);
	}
	rc = regulator_disable(chip->fpga_pwr2p5);
	if (rc) {
		pr_err("%s: enable fpga_pwr2p5 voltage failed, rc = %d\n",
		       __func__, rc);
		return rc;
	}
	pr_debug("%s: 2P5 OFF\n", __func__);

	rc = gpio_request(chip->en_pwr1p2_gpio, "fpga_1p2_en");
	if (rc) {
		pr_err("%s: request en_pwr1p2_gpio gpio failed, rc=%d\n",
		       __func__, rc);
	} else {
		rc = gpio_direction_output(chip->en_pwr1p2_gpio, 0);
	}
	gpio_free(chip->en_pwr1p2_gpio);
	pr_debug("%s: 1P2 OFF\n", __func__);

	if (chip->fpga_pwr1p8) {
		num_of_v = regulator_count_voltages(chip->fpga_pwr1p8);
		pr_debug("%s: 1p8 = %d mV\n", __func__, num_of_v);
	}
	rc = regulator_disable(chip->fpga_pwr1p8);
	if (rc) {
		pr_err("%s: disable fpga_pwr1p8 voltage failed, rc = %d\n",
		       __func__, rc);
		return rc;
	}
	pr_debug("%s: 1P8 OFF\n", __func__);

	pr_debug("%s: exit\n", __func__);
	return 0;
}
EXPORT_SYMBOL(lifmd6000_power_off);

int lifmd6000_power_on(void)
{
	int rc = 0, i, cdone = 0;
	struct lifmd6000_chip *chip = this_chip;

	pr_debug("%s: enter\n", __func__);

	if (!chip)
		return -ENODEV;

	rc = regulator_enable(chip->fpga_pwr2p5);
	if (rc) {
		pr_err("%s: enable fpga_pwr2p5 voltage failed, rc = %d\n",
		       __func__, rc);
		return rc;
	}
	pr_debug("%s: 2P5 ON\n", __func__);

	rc = gpio_request(chip->en_pwr1p2_gpio, "fpga_1p2_en");
	if (rc) {
		pr_err("%s: request en_pwr1p2_gpio gpio failed, rc=%d\n",
		       __func__, rc);
	} else {
		rc = gpio_direction_output(chip->en_pwr1p2_gpio, 1);
	}
	gpio_free(chip->en_pwr1p2_gpio);
	pr_debug("%s: 1P2 ON\n", __func__);

	rc = regulator_enable(chip->fpga_pwr1p8);
	if (rc) {
		pr_err("%s: enable fpga_pwr1p8 voltage failed, rc = %d\n",
		       __func__, rc);
		return rc;
	}
	pr_debug("%s: 1P8 ON\n", __func__);

	gpio_set_value(chip->fpga_cresetb_gpio, 0);
	usleep_range(10000, 10000);
	gpio_set_value(chip->fpga_cresetb_gpio, 1);
	pr_info("%s: load flash firmware\n", __func__);

	if (gpio_is_valid(chip->fpga_cdone_gpio)) {
		rc = gpio_request(chip->fpga_cdone_gpio, "fpga_cdone_pin");
		if (rc) {
			pr_err("request fpga_cdone_gpio failed, rc=%d\n", rc);
			return rc;
		}

		pr_debug("%s: check CDONE status\n", __func__);
		gpio_direction_input(chip->fpga_cdone_gpio);
		for (i = 0; i < 3000; i++) {
			usleep_range(1000, 1000);
			cdone = gpio_get_value(chip->fpga_cdone_gpio);

			if (cdone == 1) {
				pr_debug("%s: CDONE High take %d ms\n",
					 __func__, i);
				rc = gpio_direction_output(
					chip->fpga_cdone_gpio, 0);
				usleep_range(10000, 10000);
				rc = gpio_direction_output(
					chip->fpga_cdone_gpio, 1);
				break;
			}
		}
		/* BUG?  what if cdone doens't go high ? */
		gpio_free(chip->fpga_cdone_gpio);
	}
	pr_debug("%s: done\n", __func__);
	return 0;
}
EXPORT_SYMBOL(lifmd6000_power_on);

static const struct i2c_device_id lifmd6000_id[] = { { "lifmd6000", 0 }, {} };

#ifdef CONFIG_OF
static const struct of_device_id lifmd6000_of_match[] = {
	{
		.compatible = "fpga,lifmd6000",
	},
	{},
};
MODULE_DEVICE_TABLE(of, lifmd6000_of_match);
#endif

static struct i2c_driver lifmd6000_driver = {
    .driver =
        {
            .name = "lifmd6000",
            .pm = NULL,
            .of_match_table = of_match_ptr(lifmd6000_of_match),
        },
    .probe = lifmd6000_probe,
    .remove = lifmd6000_remove,
    .id_table = lifmd6000_id,
};

module_i2c_driver(lifmd6000_driver);

MODULE_DESCRIPTION("Lattice LIF-MD6000 FPGA driver");
MODULE_LICENSE("GPL v2");
