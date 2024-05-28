/* Silicon Labs 1141 Proximity Android Driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */
#define ENABLE_EARLYSUSPEND 0

#if ENABLE_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif	/* ENABLE_EARLYSUSPEND */

#ifdef SI1141_USE_INPUT_POLL
#include <linux/input-polldev.h>
#else
#include <linux/input.h>
#endif  /* SI1141_USE_INPUT_POLL */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/proximitysensor.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/errno.h>
#include <linux/jiffies.h>
#include <linux/interrupt.h>
#include <linux/err.h>

/* Used to provide access via misc device */
#include <linux/miscdevice.h>

#include <linux/si1141ps.h>

#define DRIVER_VERSION "0.1"
#define DEVICE_NAME "si1141ps"

#define PS_CT_PATH	"/persist/sensors/ps_ct"
#define PS_LO_PATH	"/persist/sensors/ps_low"
#define PS_HI_PATH	"/persist/sensors/ps_high"

#define SI1141_MAX_LED 1

/* The names of the various input sensors this driver supports */
static  char INPUT_DEVICE_NAME_PS[25]="si1141ps";
#define INPUT_DEVICE_NAME_PS_MIC "si1141ps_mic"
#define INPUT_DEVICE_NAME_PS_FRONT "si1141ps_front"

#define SI114x_MAX_LED 3

struct si1141_led {
	int drive_strength;
	int enable;
};

static void sensor_irq_do_work(struct work_struct *work);
static DECLARE_WORK(sensor_irq_work, sensor_irq_do_work);

/* Both ALS and PS input device structures.  They could
 * be either interrupt driven or polled
 */
struct si1141_input_dev {
	struct input_dev *input;
#ifdef SI1141_USE_INPUT_POLL
	struct input_polled_dev *input_poll;
	unsigned int poll_interval;
#endif  /* SI1141_USE_INPUT_POLL */
};

struct si1141_data {
	/* Device */
	struct i2c_client *client;
	uint8_t slave_addr;

#ifdef SI1141_USE_INPUT_POLL
	struct si1141_input_dev input_dev_ps;
#else
	struct input_dev *si1141_input_dev;
#endif
	struct si1141_led led[SI114x_MAX_LED];
	struct workqueue_struct *lp_ps_wq;
	struct delayed_work dwork;

	/* Counter used for the command and response sync */
	int resp_id;
	bool ps_enable;
	uint16_t base;
	int diff;
	uint16_t thld_hi;
	uint16_t thld_lo;
	uint16_t thld_ct;
	int ps_opened;
	struct regulator *vdd;
	int intr;
	int ps_polling_delay;
#if ENABLE_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif

};

/*
 * Filesystem API is implemented with the kernel fifo.
 */
#define SI1141_RD_QUEUE_SZ 256
static DECLARE_WAIT_QUEUE_HEAD(si1141_read_wait);
static struct si1141_data_user si1141_fifo_buffer[SI1141_RD_QUEUE_SZ];
static struct si1141_data_user *si1141_user_wr_ptr = NULL;
static struct si1141_data_user *si1141_user_rd_ptr = NULL;
static struct mutex si1141_user_lock;
static struct mutex si1141_resp_lock;
static atomic_t si1141_opened = ATOMIC_INIT(0);
//static struct wake_lock pwake_lock;
struct si1141_data *si1141_private = NULL;
bool calidata_retrieved = true;
bool ps_reported = true;
bool ps_diff = false;
bool ps_dbg = false;
static int pirq;
static struct workqueue_struct *lp_wq;
#ifdef SI1141_USE_INPUT_POLL
static atomic_t si1141ps_opened = ATOMIC_INIT(0);
#endif
static struct si1141_data *g_si1141;
int g_prox_status=1;

#define DONDOFF_DEBOUNCE 2
int g_prox_debounce = DONDOFF_DEBOUNCE;

#define DONDOFF_SIZE 5
int dondoff2_index = 0;
int dondoff2_count = 0;
int dondoff2_temp = 0;
int dondoff2_array[10] = {0};
int dondoff2_max = 0;
int dondoff2_min = 0;
int dondoff2_variant = 4;	// /4
int dondoff2_state = -1;
extern int dondoff_state;

/*
 * I2C bus transaction read for consecutive data.
 * Returns 0 on success.
 */
static int _i2c_read_mult(struct si1141_data *si114x, uint8_t *rxData, int length)
{
	int i;
	struct i2c_msg data[] = {
		{
			.addr = si114x->client->addr,
			.flags = 0,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = si114x->client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};

	for (i = 0; i < I2C_RETRY; i++) {
		if (i2c_transfer(si114x->client->adapter, data, 2) > 0)
			break;
		dev_warn(&si114x->client->dev, "%s Retried read count:%d\n", __func__, i);
		msleep(10);
	}

	if (i >= I2C_RETRY) {
		dev_err(&si114x->client->dev, "%s i2c read retry exceeded\n", __func__);
		return -EIO;
	}
	return 0;
}

/*
 * I2C bus transaction to write consecutive data.
 * Returns 0 on success.
 */
static int _i2c_write_mult(struct si1141_data *si114x, uint8_t *txData, int length)
{
	int i;
	struct i2c_msg data[] = {
		{
			.addr = si114x->client->addr,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};

	for (i = 0; i < I2C_RETRY; i++) {
		if (i2c_transfer(si114x->client->adapter, data, 1) > 0)
			break;
		dev_warn(&si114x->client->dev, "%s Retried read count:%d\n", __func__, i);
		msleep(10);
	}

	if (i >= I2C_RETRY) {
		dev_err(&si114x->client->dev, "%s i2c write retry exceeded\n", __func__);
		return -EIO;
	}
	return 0;
}

/*
 * I2C bus transaction to read single byte.
 * Returns 0 on success.
 */
static int _i2c_read_one(struct si1141_data *si114x, int addr, int *data)
{
	int rc = 0;
	uint8_t buffer[1];

	buffer[0] = (uint8_t)addr;
	msleep(1);
	rc = _i2c_read_mult(si114x, buffer, sizeof(buffer));
	if (rc == 0) {
		*data = buffer[0];
	}
	return rc;
}

/*
 * I2C bus transaction to write single byte.
 * Returns 0 on success.
 */
static int _i2c_write_one(struct si1141_data *si114x, int addr, int data)
{
	uint8_t buffer[2];

	buffer[0] = (uint8_t)addr;
	buffer[1] = (uint8_t)data;
	msleep(1);
	return _i2c_write_mult(si114x, buffer, sizeof(buffer));
}


static int ps_get_cal_data(struct si1141_data *si1141)
{
	struct file *ps_filp = NULL;
	mm_segment_t old_fs;
	int err = 0;


	old_fs = get_fs();
	set_fs(KERNEL_DS);

	ps_filp = filp_open(PS_CT_PATH, O_RDONLY, 0666);
	if (IS_ERR(ps_filp))
	{
		err = PTR_ERR(ps_filp);
		if (err != -ENOENT)
		{
			pr_err("%s: Can't open calibration data file\n", __func__);
		}
		set_fs(old_fs);
		return err;
	}

	err = ps_filp->f_op->read(ps_filp,
		(char *)&si1141->thld_ct, sizeof(uint16_t), &ps_filp->f_pos);
	if (err != sizeof(uint16_t))
	{
		pr_err("%s: Can't read the calibration data from file\n", __func__);
		err = -EIO;
	}

	pr_info("%s: cal_data = %d\n",
		__func__, si1141->thld_ct);

	filp_close(ps_filp, current->files);
	set_fs(old_fs);

	return err;
}

/*
 * Read and analyze the response after a write to the command register.
 * Returns 0 if response was ok.
 */
int _read_response(struct si1141_data *si114x) {
	int rc = 0;
	int data = 0;
	struct i2c_client *client = si114x->client;

	msleep(25);

	rc = _i2c_read_one(si114x, SI1141_RESPONSE, &data);
	if (rc) {
		return rc;
	}

	if (data & SI1141_RESPONSE_ERR) {
		dev_err(&client->dev, "[%s] Response error value:0x%02x\n", __func__, data);
		return rc;
	}

	data &= SI1141_RESPONSE_ID;
	if (si114x->resp_id != data) {
		dev_err(&client->dev, "[%s] Unexpected response id act:%d exp:%d\n", __func__, data, si114x->resp_id);
		si114x->resp_id = data;
	}
	si114x->resp_id = (si114x->resp_id + 1) & SI1141_RESPONSE_ID;

	return rc;

}


int _pre_cmd_write(struct si1141_data *si114x) {

	int data = 0;
	struct i2c_client *client = si114x->client;
	int rc = 0;

	rc = _i2c_write_one(si114x, SI1141_COMMAND, CMD_NOP);
	if (rc) {
		return rc;
	}

	rc = _i2c_read_one(si114x, SI1141_RESPONSE, &data);
	if (rc) {
		return rc;
	}

	if (data & SI1141_RESPONSE_ERR) {
		dev_err(&client->dev, "%s Response error value:0x%02x\n", __func__, data);
		return rc;
	}

	data &= SI1141_RESPONSE_ID;

	si114x->resp_id = (data +1 ) & SI1141_RESPONSE_ID;

	return rc;

}

/*
 * Write a parameter and a command to the internal microprocessor on the ALS/PS sensor.
 * Read and verify response value.
 */
int _parm_write(struct si1141_data *si114x, int param_type, int param_addr, int param_data) {
	int rc = 0;
	uint8_t buffer[3];

	mutex_lock(&si1141_resp_lock);
	rc += _pre_cmd_write(si114x);

	buffer[0] = SI1141_PARAM_WR;
	buffer[1] = param_data;
	buffer[2] = param_type | (param_addr & CMD_PARAM_ADDR_MASK);

	rc += _i2c_write_mult(si114x, buffer, sizeof(buffer));

	rc += _read_response(si114x);
	mutex_unlock(&si1141_resp_lock);
	return rc;

}



/*
 * Write a command to the internal microprocessor on the ALS/PS sensor
 * Read and verify response value.
 */
static int _cmd_write(struct si1141_data *si114x, int cmd) {
	int rc = 0;


	mutex_lock(&si1141_resp_lock);
	rc += _pre_cmd_write(si114x);

	rc += _i2c_write_one(si114x, SI1141_COMMAND, cmd);

	rc += _read_response(si114x);
	mutex_unlock(&si1141_resp_lock);


	return rc;

}



static void si114x_input_report_ps_values(struct input_dev *input, uint16_t value, int report)
{
	input_report_abs(input, ABS_DISTANCE, report);

	/* One nanonsecond only fills 30 bits of a 32 bit field, so shift to keep
	 * maximum precision */
	input_sync(input);

	if ( report == 0 ) {
		input_report_key(input, KEY_WAKEUP, 1);
		mdelay(10);
		input_report_key(input, KEY_WAKEUP, 0);
		input_sync(input);
	} else {
		input_report_key(input, KEY_SLEEP, 1);
		mdelay(10);
		input_report_key(input, KEY_SLEEP, 0);
		input_sync(input);
	}

	/* Now stick the values into the queue */
	si1141_user_wr_ptr->led_a = value;

	//g_prox_status=report;

	mutex_lock(&si1141_user_lock);
	ps_reported=true;
	si1141_user_wr_ptr++;
	if (si1141_user_wr_ptr == (si1141_fifo_buffer + SI1141_RD_QUEUE_SZ)) {
		si1141_user_wr_ptr = si1141_fifo_buffer;
	}
	mutex_unlock(&si1141_user_lock);

}


static int set_led_drive_strength(struct si1141_data *si1141)
{
	int rc = 0;

	rc += _i2c_write_one(si1141, SI1141_PS_LED21, si1141->led[1].drive_strength << 4
	                     | si1141->led[0].drive_strength);
	rc += _i2c_write_one(si1141, SI1141_PS_LED3, si1141->led[2].drive_strength);

	return rc;
}


static int turn_on_leds(struct si1141_data *si1141) {
	int rc = 0;

	rc += _parm_write(si1141, CMD_PARAM_SET, PARAM_PSLED12_SELECT, 0x01);
	rc += _parm_write(si1141, CMD_PARAM_SET, PARAM_PSLED3_SELECT, 0x00);

	return rc;
}

static int set_measurement_rates(struct si1141_data *si1141) {

	int rc = 0;

	rc += _i2c_write_one(si1141, SI1141_MEAS_RATE, PS_MEAS_DEFAULT);
	rc += _i2c_write_one(si1141, SI1141_ALS_RATE, SI1141_ALS_RATE_NONE);
	rc += _i2c_write_one(si1141, SI1141_PS_RATE, PS_RATE_DEFAULT);

	return rc;
}

static int _check_part_id(struct si1141_data *si1141)
{
	int rc = 0;
	int data = 0;
	struct i2c_client *client = si1141->client;

	rc = _i2c_read_one(si1141, SI1141_PART_ID, &data);
	if (rc < 0) {
		dev_err(&client->dev, "%s: Unable to read part identifier\n",
		        __func__);
		return -EIO;
	}

	switch (data) {
		case PART_ID_SI1141:
			if (ps_dbg) dev_err(&client->dev, "Don/Doff.\n");
			break;

		default:
			dev_err(&client->dev, "%s: Unable to determine SI1141 part_id:%02x\n", __func__, data);
			rc = -ENODEV;
	}
	return rc;
}

static int si1141ps_irq_status(struct si1141_data *si1141)
{
	int rc = 0;
	int data = 0;
	struct i2c_client *client = si1141->client;

	rc += _i2c_read_one(si1141, SI1141_IRQ_STATUS, &data);
	if (rc < 0) {
		dev_err(&client->dev, "%s: Unable to read irq status\n",
		        __func__);
		return -EIO;
	}

	if ( (data & IRQ_PS1_INT) == IRQ_PS1_INT) {
		rc += _i2c_write_one(si1141, SI1141_IRQ_STATUS, IRQ_PS1_INT);
	}


	return rc;
}


static int si1141ps_thld_get(struct si1141_data *si1141, uint16_t* value)
{
	uint8_t buffer[2];
	struct i2c_client *client = si1141->client;
	uint16_t *data16 = (uint16_t *)buffer;
	int rc = 0;

	buffer[0] = SI1141_PS1_TH;
	rc += _i2c_read_mult(si1141, buffer, sizeof(buffer));
	if (rc) {
		dev_err(&client->dev, "%s: Unable to get ps thld\n", __func__);
		return rc;
	}
	*value = data16[0];
	return rc;
}


static int si1141ps_thld_set(struct si1141_data *si1141, uint16_t data)
{
	uint8_t buffer[3];
	struct i2c_client *client = si1141->client;
	uint16_t *data16 = (uint16_t *)(buffer+1);
	int rc = 0;

	buffer[0] = SI1141_PS1_TH;
	*data16 = (uint16_t)data;
	rc += _i2c_write_mult(si1141, buffer, sizeof(buffer));
	if (rc) {
		dev_err(&client->dev, "%s: Unable to set ps thld\n", __func__);
	}

	return rc;
}



static int si1141_ps_setup(struct si1141_data *si1141) {

	int rc = 0;

	rc += _check_part_id(si1141);

	rc += _i2c_write_one(si1141, SI1141_HW_KEY, HW_KEY);

	rc += set_led_drive_strength(si1141);

	rc += _i2c_write_one(si1141, SI1141_INT_CFG, SI1141_INT_CFG_DEF);

	rc += _i2c_write_one(si1141, SI1141_IRQ_MODE1, SI1141_IRQ_MODE1_DEF);

	rc += turn_on_leds(si1141);

	rc += _parm_write(si1141, CMD_PARAM_SET, PARAM_ADC_MISC, 0x24);

	rc += _parm_write(si1141, CMD_PARAM_SET, PARAM_ALS_VIS_ADC_GAIN, 0x05);

	rc += _parm_write(si1141, CMD_PARAM_SET, PARAM_ADC_GAIN, PS_ADC_GAIN_DEFAULT);

	rc += _parm_write(si1141, CMD_PARAM_SET, PARAM_ADC_COUNTER, PS_ADC_REC_DEFAULT);

	rc += set_measurement_rates(si1141);


	if (!calidata_retrieved)
	{		
		rc = ps_get_cal_data(si1141);
		if (rc < 0 && rc != -ENOENT)
		{
			pr_err("%s: ps_get_cal_data() failed\n", __func__);
		} else {
			calidata_retrieved = true;
		}
	}

	return rc;
}

static int si1141ps_doing(uint8_t opt)
{
	uint8_t buffer[2];
	struct si1141_data *si1141 = g_si1141;
	struct input_dev *input;
	uint16_t *data16 = (uint16_t *)buffer;
	int value = 0;
	static bool first = false;
	uint16_t 	thld_lo, thld_hi;

	if(opt==0)
		first=false;
	else if(opt==1)
		first=true;

	thld_lo = si1141->thld_lo;
	thld_hi = si1141->thld_hi;
	input = si1141->si1141_input_dev;
	buffer[0] = SI1141_PS1_DATA0;

	si1141ps_irq_status(si1141);

	if(true){//si1141->ps_enable){

		if (_i2c_read_mult(si1141, buffer, sizeof(buffer))) {
			pr_err("%s: Unable to read ps data\n", __func__);
			return 0;
		}

		if(first){
			if(data16[0]<1000)
				si1141->base = data16[0] ;
			else
				si1141->base = 1000;
			first=false;
		}

		value = data16[0];

		if( ps_diff && si1141->thld_ct > 0){
			si1141->diff= si1141->base - si1141->thld_ct;
			if( si1141->diff <0)
				si1141->diff =0;

			value = data16[0] - si1141->diff ;
			if(value<0)
				value=1;
			if( value+si1141->diff > 65534)
				value=65534-si1141->diff;

			if(thld_lo+si1141->diff > 65534)
				thld_lo=65534;
			else
				thld_lo+=si1141->diff;

			if(thld_hi+si1141->diff > thld_lo)
				thld_hi=thld_lo-1;
			else
				thld_hi+=si1141->diff;
		}

		if(false && ps_reported && opt != 50){
			if( value <= si1141->thld_hi ){
				si114x_input_report_ps_values(input, data16[0], 1);
				si1141ps_thld_set(si1141, thld_lo);
				if (ps_dbg) pr_err("si1141: value is 1\n");
			}else if(value >= si1141->thld_lo){
				si114x_input_report_ps_values(input, data16[0], 0);
				si1141ps_thld_set(si1141, thld_hi);
				if (ps_dbg) pr_err("si1141: value is 0\n");
			}
		}
	}

	return value;

}


/* Enable ps. */
static int enable_ps(struct si1141_data *si1141, bool enable) {
	int rc = 0;
	int i;
	uint8_t first=99;
	struct i2c_client *client = si1141->client;

	for (i = 0; rc=0, i < PS_I2C_RETRY; i++) {
		if( enable ){
			first=1;
			si1141->base=0;
			si1141->diff=0;
			rc += si1141_ps_setup(si1141);
			#ifndef SI1141_USE_INPUT_POLL
			rc += si1141ps_thld_set(si1141, si1141->thld_lo);
			rc += _cmd_write(si1141, CMD_PS_AUTO);
			#endif
			rc +=  _parm_write(si1141, CMD_PARAM_SET, PARAM_PS_HISTORY, PS_HIS_DEFAULT);
			rc += _i2c_write_one(si1141, SI1141_IRQ_ENABLE, SI1141_IRQ_ENABLE_DEF);
			rc +=  _parm_write(si1141, CMD_PARAM_SET, PARAM_I2C_CHLIST, CHLIST_EN_PS1);
		}else{
			first=0;
			rc += _i2c_write_one(si1141, SI1141_IRQ_ENABLE, 0);
			rc += _parm_write(si1141, CMD_PARAM_SET, PARAM_I2C_CHLIST, 0);
		}

		if( rc == 0)
			break;

		dev_err(&client->dev, "%s rc=%d, Retried read count:%d\n", __func__, rc, i);
		msleep(10);
	}

	if (rc)
		si1141->ps_enable = 0;
	else
		si1141->ps_enable =enable;

	si1141ps_doing(first);

	return rc;
}

#if ENABLE_EARLYSUSPEND
static void si114x_early_suspend(struct early_suspend *h)
{
	struct si1141_data *si1141 = container_of(h, struct si1141_data, early_suspend);
	struct i2c_client *client = si1141->client;

	dev_info(&client->dev, "%s:\n", __func__);
}

static void si114x_late_resume(struct early_suspend *h)
{
	struct si1141_data *si1141 = container_of(h, struct si1141_data, early_suspend);
	struct i2c_client *client = si1141->client;

	dev_info(&client->dev, "%s:\n", __func__);
}
#endif



#ifdef SI1141_USE_INPUT_POLL
/*
 * This callback is called by the input subsystem at the approrpriate polling
 * interval.
 */
static void si1141_input_poll_ps_cb(struct input_polled_dev *dev)
{
	uint8_t buffer[2];
	struct si1141_data *si1141 = dev->private;
	struct i2c_client *client = si1141->client;
	struct input_dev *input;
	uint16_t *data16 = (uint16_t *)buffer;
	int value;
	static bool first;
	const int vote_decide=2;
	const int vote_to_who_def=9999;
	static int vote_to_who=vote_to_who_def;
	static int vote_count=1;

	if(!si1141->ps_enable) first=true;
	input = si1141->input_dev_ps.input_poll->input;
	buffer[0] = SI1141_PS1_DATA0;

	if(si1141->ps_enable){
		_cmd_write(si1141, CMD_PS_FORCE);
		msleep(5);
		if (_i2c_read_mult(si1141, buffer, sizeof(buffer))) {
			dev_err(&client->dev, "%s: Unable to read ps data\n", __func__);
			return;
		}

		if(first){
			if(data16[0]<1000)
				si1141->base = data16[0] ;
			else
				si1141->base = 1000;
			first=false;
		}

		value = data16[0];

		if( si1141->thld_ct > 0){
			si1141->diff= si1141->base - si1141->thld_ct;
			if( si1141->diff <0)
				si1141->diff =0;

			value = data16[0] - si1141->diff ;
			if(value<0)
				value=1;
		}

		if(ps_reported){
			if( value < si1141->thld_hi ){
				if(vote_to_who == 1 || vote_to_who == vote_to_who_def)
					++vote_count;
				else if(vote_to_who == 0)
					vote_count=1;
				vote_to_who=1;

				if(vote_count > vote_decide){
					si114x_input_report_ps_values(input, data16[0], 1);
					vote_count = vote_decide+1;
				}
			}else if(value > si1141->thld_lo){
				if(vote_to_who == 0  || vote_to_who == vote_to_who_def)
					++vote_count;
				else if(vote_to_who == 1)
					vote_count=1;
				vote_to_who=0;
				if(vote_count > vote_decide){
					si114x_input_report_ps_values(input, data16[0], 0);
					vote_count = vote_decide+1;
				}
			}
		}else{
			if(value < si1141->thld_hi){
				si114x_input_report_ps_values(input, data16[0], 1);
			}else{
				si114x_input_report_ps_values(input, data16[0], 0);
			}
		}

	}

	return;
}

/* Call from the input subsystem to start polling the device */
static int si114x_input_ps_open(struct input_dev *input)
{
	struct si1141_data *si1141 = input_get_drvdata(input);
	struct i2c_client *client = si1141->client;
	dev_info(&client->dev, "%s\n", __func__);

	if (atomic_xchg(&si1141ps_opened, 1) == 1) {
		return -EBUSY;
	}

	return 0;
}

/* Call from the input subsystem to stop polling the device */
static void si114x_input_ps_close(struct input_dev *input)
{
	struct si1141_data *si1141 = input_get_drvdata(input);
	struct i2c_client *client = si1141->client;
	dev_info(&client->dev, "%s\n", __func__);

	if (atomic_xchg(&si1141ps_opened, 0) == 0) {
		dev_err(&client->dev, "%s: Device has not been opened\n", __func__);
		return;
	}
	
	return;
}

static int setup_ps_polled_input(struct si1141_data *si1141)
{
	int rc;
	struct i2c_client *client = si1141->client;
	struct input_polled_dev *input_poll;
	struct input_dev *input;

	input_poll = input_allocate_polled_device();
	if (!input_poll) {
		dev_err(&client->dev, "%s: Unable to allocate input device\n", __func__);
		return -ENOMEM;
	}

	input_poll->private = si1141;
	input_poll->poll = si1141_input_poll_ps_cb;
	input_poll->poll_interval = PS_POLL_INTERVAL;


	input = input_poll->input;

	input->name = INPUT_DEVICE_NAME_PS;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &si1141->client->dev;

	set_bit(EV_ABS, input->evbit);
	set_bit(EV_KEY, input->evbit);

	input->open = si114x_input_ps_open;
	input->close = si114x_input_ps_close;

	input_set_abs_params(input, ABS_DISTANCE, PS_MIN_MEASURE_VAL, PS_MAX_MEASURE_VAL, 0, 0);

	/* max is a signed value */
	input_set_abs_params(input, ABS_DISTANCE, 0, 0x7fffffff, 0, 0);
	input_set_capability(input, EV_KEY, KEY_WAKEUP);
	input_set_capability(input, EV_KEY, KEY_SLEEP);

	input_set_drvdata(input, input_poll);

	rc = input_register_polled_device(input_poll);
	if (rc) {
		dev_err(&si1141->client->dev,
		        "Unable to register input polled device %s\n",
		        input_poll->input->name);
		goto err_ps_register_input_device;
	}

	dev_dbg(&client->dev, "%s Registered input polling device rate:%d\n",
	        __func__, input_poll->poll_interval);
	si1141->input_dev_ps.input = input;
	si1141->input_dev_ps.input_poll = input_poll;
	return rc;

err_ps_register_input_device:
	input_free_polled_device(si1141->input_dev_ps.input_poll);

	return rc;
}

#else

static int setup_ps_input(struct si1141_data *si1141)
{
	int rc;
	struct i2c_client *client = si1141->client;
	struct input_dev *input_dev=si1141->si1141_input_dev;

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&client->dev, "%s: Unable to allocate input device\n", __func__);
		return -ENOMEM;
	}

	input_dev->name = INPUT_DEVICE_NAME_PS;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &si1141->client->dev;
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);

	input_set_abs_params(input_dev, ABS_DISTANCE, PS_MIN_MEASURE_VAL, PS_MAX_MEASURE_VAL, 0, 0);
	input_set_capability(input_dev, EV_KEY, KEY_WAKEUP);
	input_set_capability(input_dev, EV_KEY, KEY_SLEEP);
	input_set_capability(input_dev, EV_KEY, KEY_POWER);


	rc = input_register_device(input_dev);
	if (rc < 0) {
		dev_err(&client->dev, "%s: PS Register Input Device Fail...\n", __func__);
		goto err_ps_register_input_device;
	}
	si1141->si1141_input_dev = input_dev;

	return rc;

err_ps_register_input_device:
	input_free_device(input_dev);

	return rc;
}
#endif  /* SI1141_USE_INPUT_POLL */


static irqreturn_t si1141ps_irq_handler(int irq, void *data)
{
	return IRQ_HANDLED;
}

static ssize_t ps_selftest_show(struct device *dev,
                            struct device_attribute *attr, char *buf)
{
	int ret=0;
#ifdef SI1141_USE_INPUT_POLL
	struct si1141_data *si1141;
	struct input_polled_dev *input_poll;

	input_poll = dev_get_drvdata(dev);
	if (!input_poll) {
		return -ENODEV;
	}

	si1141 = (struct si1141_data *)input_poll->private;
	if (!si1141) {
		return -ENODEV;
	}
#else
	struct si1141_data *si1141 = dev_get_drvdata(dev);
#endif
	ret=_check_part_id(si1141);
	if(ret){
		return sprintf(buf, "fail\n");
	}

	return sprintf(buf, "pass");
}
static DEVICE_ATTR(selftest, 0444, ps_selftest_show, NULL);

static ssize_t ps_intr_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
	int val=99;
#ifdef SI1141_USE_INPUT_POLL
	struct si1141_data *si1141;
	struct input_polled_dev *input_poll;

	input_poll = dev_get_drvdata(dev);
	if (!input_poll) {
		return -ENODEV;
	}

	si1141 = (struct si1141_data *)input_poll->private;
	if (!si1141) {
		return -ENODEV;
	}
#else
       struct si1141_data *si1141 = dev_get_drvdata(dev);
#endif

	val = gpio_get_value(si1141->intr);

	return sprintf(buf, "%d\n",val);

}
static DEVICE_ATTR(intr, 0444, ps_intr_show, NULL);

static ssize_t ps_data_show(struct device *dev,
                            struct device_attribute *attr, char *buf)
{
	uint8_t buffer[sizeof(uint16_t)*3];
	uint16_t *data16 = (uint16_t *)buffer;
#ifdef SI1141_USE_INPUT_POLL
	struct si1141_data *si1141;
	struct input_polled_dev *input_poll;

	input_poll = dev_get_drvdata(dev);
	if (!input_poll) {
		return -ENODEV;
	}

	si1141 = (struct si1141_data *)input_poll->private;
	if (!si1141) {
		return -ENODEV;
	}
#else
       struct si1141_data *si1141 = dev_get_drvdata(dev);
#endif

#ifdef SI1141_USE_INPUT_POLL
	_cmd_write(si1141, CMD_PS_FORCE);
#else
	_cmd_write(si1141, CMD_PS_AUTO);
#endif
	msleep(5);

	buffer[0] = SI1141_PS1_DATA0;
	if (_i2c_read_mult(si1141, buffer, sizeof(buffer))) {
		return -EIO;
	}

	return sprintf(buf, "%u\n", data16[0]);
}
static DEVICE_ATTR(ps_input_data, 0444, ps_data_show, NULL);

static ssize_t led_input_drive_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
#ifdef SI1141_USE_INPUT_POLL
	struct si1141_data *si1141;
	struct input_polled_dev *input_poll;

	input_poll = dev_get_drvdata(dev);
	if (!input_poll) {
		return -ENODEV;
	}

	si1141 = (struct si1141_data *)input_poll->private;
	if (!si1141) {
		return -ENODEV;
	}
#else
       struct si1141_data *si1141 = dev_get_drvdata(dev);
#endif

	return sprintf(buf, "%u %u %u\n", si1141->led[0].drive_strength,
	               si1141->led[1].drive_strength,
	               si1141->led[2].drive_strength);
}

static ssize_t led_input_drive_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count)
{
	int i;
	unsigned int drive_strength[SI114x_MAX_LED];
#ifdef SI1141_USE_INPUT_POLL
	struct si1141_data *si1141;
	struct input_polled_dev *input_poll;

	input_poll = dev_get_drvdata(dev);
	if (!input_poll) {
		return -ENODEV;
	}

	si1141 = (struct si1141_data *)input_poll->private;
	if (!si1141) {
		return -ENODEV;
	}
#else
       struct si1141_data *si1141 = dev_get_drvdata(dev);
#endif

	sscanf(buf, "%d %d %d", &drive_strength[0], &drive_strength[1], &drive_strength[2]);

	/* Validate parameters */
	for (i = 0; i < SI114x_MAX_LED; i++) {
		if (drive_strength[i] > 15) {
			return -EINVAL;
		}
	}

	/* Update local copies of parameters */
	for (i = 0; i < SI1141_MAX_LED; i++) {
		si1141->led[i].drive_strength = drive_strength[i];
	}

	/* Write parameters to device */
	set_led_drive_strength(si1141);

	return count;
}

static DEVICE_ATTR(led_input_drive, 0664, led_input_drive_show, led_input_drive_store);

static ssize_t ps_input_meas_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
	int value=0xffff;

#ifdef SI1141_USE_INPUT_POLL
	struct si1141_data *si1141;
	struct input_polled_dev *input_poll;

	input_poll = dev_get_drvdata(dev);
	if (!input_poll) {
		return -ENODEV;
	}

	si1141 = (struct si1141_data *)input_poll->private;
	if (!si1141) {
		return -ENODEV;
	}
#else
       struct si1141_data *si1141 = dev_get_drvdata(dev);
#endif

	_i2c_read_one(si1141, SI1141_MEAS_RATE, &value);

	return sprintf(buf, "%x\n",value);

}

static ssize_t ps_input_meas_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count)
{
	int value=0xffff;

#ifdef SI1141_USE_INPUT_POLL
	struct si1141_data *si1141;
	struct input_polled_dev *input_poll;

	input_poll = dev_get_drvdata(dev);
	if (!input_poll) {
		return -ENODEV;
	}

	si1141 = (struct si1141_data *)input_poll->private;
	if (!si1141) {
		return -ENODEV;
	}
#else
       struct si1141_data *si1141 = dev_get_drvdata(dev);
#endif

	sscanf(buf, "%x", &value);

	if( value != 0xffff)
		_i2c_write_one(si1141, SI1141_MEAS_RATE, value);

	return count;

}

static DEVICE_ATTR(meas, 0664, ps_input_meas_show, ps_input_meas_store);

static ssize_t ps_input_gain_show(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
	int data1 = 0, data2 = 0;
#ifdef SI1141_USE_INPUT_POLL
	struct si1141_data *si1141;
	struct input_polled_dev *input_poll;

	input_poll = dev_get_drvdata(dev);
	if (!input_poll) {
		return -ENODEV;
	}

	si1141 = (struct si1141_data *)input_poll->private;
	if (!si1141) {
		return -ENODEV;
	}
#else
       struct si1141_data *si1141 = dev_get_drvdata(dev);
#endif

	if ( _cmd_write(si1141,  CMD_PARAM_QUERY | PARAM_ADC_COUNTER ) ) {
		return -1;
	}

	msleep(1);
	_i2c_read_one(si1141, SI1141_PARAM_RD, &data1);

	msleep(1);
	if ( _cmd_write(si1141,  CMD_PARAM_QUERY | PARAM_ADC_GAIN ) ) {
		return -1;
	}

	msleep(1);
	_i2c_read_one(si1141, SI1141_PARAM_RD, &data2);

	return sprintf(buf, "%02x %02x\n", (data1&0x70) >> 4, data2&0x07 );
}

static ssize_t ps_input_gain_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count)
{
	int invalue=0xffff;
	int setvalue1=0,setvalue2=0;
#ifdef SI1141_USE_INPUT_POLL
	struct si1141_data *si1141;
	struct input_polled_dev *input_poll;

	input_poll = dev_get_drvdata(dev);
	if (!input_poll) {
		return -ENODEV;
	}

	si1141 = (struct si1141_data *)input_poll->private;
	if (!si1141) {
		return -ENODEV;
	}
#else
       struct si1141_data *si1141 = dev_get_drvdata(dev);
#endif

	sscanf(buf, "%x", &invalue);

	if( invalue < 8 && invalue >= 0){
		setvalue1 = invalue << 4;
		setvalue2 = (~invalue) & 0x07;
	}

	dev_err(&si1141->client->dev, "%s [%d] [%d]\n", __func__,setvalue1, setvalue2);

	if( invalue != 0xffff){
		dev_err(&si1141->client->dev, "%s set value\n", __func__);
		_parm_write(si1141, CMD_PARAM_SET, PARAM_ADC_COUNTER, setvalue1);
		_parm_write(si1141, CMD_PARAM_SET, PARAM_ADC_GAIN, setvalue2);
	}

	return count;

}
static DEVICE_ATTR(gain, 0664, ps_input_gain_show, ps_input_gain_store);


static ssize_t ps_input_rate_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
	int value=0xffff;
#ifdef SI1141_USE_INPUT_POLL
	struct si1141_data *si1141;
	struct input_polled_dev *input_poll;

	input_poll = dev_get_drvdata(dev);
	if (!input_poll) {
		return -ENODEV;
	}

	si1141 = (struct si1141_data *)input_poll->private;
	if (!si1141) {
		return -ENODEV;
	}
#else
       struct si1141_data *si1141 = dev_get_drvdata(dev);
#endif

	_i2c_read_one(si1141, SI1141_PS_RATE, &value);

	return sprintf(buf, "%x\n",value);

}

static ssize_t ps_input_rate_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count)
{
	int value=0xffff;
#ifdef SI1141_USE_INPUT_POLL
	struct si1141_data *si1141;
	struct input_polled_dev *input_poll;

	input_poll = dev_get_drvdata(dev);
	if (!input_poll) {
		return -ENODEV;
	}

	si1141 = (struct si1141_data *)input_poll->private;
	if (!si1141) {
		return -ENODEV;
	}
#else
       struct si1141_data *si1141 = dev_get_drvdata(dev);
#endif

	sscanf(buf, "%x", &value);

	if( value != 0xffff)
		_i2c_write_one(si1141, SI1141_PS_RATE, value);

	return count;

}

static DEVICE_ATTR(rate, 0664, ps_input_rate_show, ps_input_rate_store);


static ssize_t ps_input_enable_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
#ifdef SI1141_USE_INPUT_POLL
	struct si1141_data *si1141;
	struct input_polled_dev *input_poll;

	input_poll = dev_get_drvdata(dev);
	if (!input_poll) {
		return -ENODEV;
	}

	si1141 = (struct si1141_data *)input_poll->private;
	if (!si1141) {
		return -ENODEV;
	}
#else
       struct si1141_data *si1141 = dev_get_drvdata(dev);
#endif

	if (si1141->ps_enable)
		return sprintf(buf, "%u\n",1);
	else
		return sprintf(buf, "%u\n",0);
}

static ssize_t ps_input_enable_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count)
{
	int ps_en;
#ifdef SI1141_USE_INPUT_POLL
	struct si1141_data *si1141;
	struct input_polled_dev *input_poll;

	input_poll = dev_get_drvdata(dev);
	if (!input_poll) {
		return -ENODEV;
	}

	si1141 = (struct si1141_data *)input_poll->private;
	if (!si1141) {
		return -ENODEV;
	}
#else
       struct si1141_data *si1141 = dev_get_drvdata(dev);
#endif

	sscanf(buf, "%d", &ps_en);

	if(ps_en==0)
		enable_ps(si1141,false);
	else
		enable_ps(si1141,true);


	return count;

}

static DEVICE_ATTR(enable, 0664, ps_input_enable_show, ps_input_enable_store);

static ssize_t ps_diff_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
	if (ps_diff)
		return sprintf(buf, "%u\n",1);
	else
		return sprintf(buf, "%u\n",0);
}

static ssize_t ps_diff_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count)
{
	int my_value;

	sscanf(buf, "%d", &my_value);

	if(my_value==0)
		ps_diff=false;
	else
		ps_diff=true;


	return count;

}

static DEVICE_ATTR(ps_diff, 0664, ps_diff_show, ps_diff_store);

static ssize_t ps_input_thld_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{

#ifdef SI1141_USE_INPUT_POLL

	struct si1141_data *si1141;
	struct input_polled_dev *input_poll;

	input_poll = dev_get_drvdata(dev);
	if (!input_poll) {
		return -ENODEV;
	}

	si1141 = (struct si1141_data *)input_poll->private;
	if (!si1141) {
		return -ENODEV;
	}

	return sprintf(buf, "%u %u %u\n",si1141->thld_ct, si1141->thld_lo, si1141->thld_hi);

#else

	struct si1141_data *si1141 = dev_get_drvdata(dev);
	uint16_t value=0;

	si1141ps_thld_get(si1141,&value);

	if (ps_dbg) return sprintf(buf, "%u %u %u -> [%d]\n",si1141->thld_ct, si1141->thld_lo, si1141->thld_hi, value);

	return sprintf(buf, "%u %u %u\n",si1141->thld_ct, si1141->thld_lo, si1141->thld_hi);

#endif

}

static ssize_t ps_input_thld_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count)
{
	long int value1, value2, value3;
#ifdef SI1141_USE_INPUT_POLL
	struct si1141_data *si1141;
	struct input_polled_dev *input_poll;

	input_poll = dev_get_drvdata(dev);
	if (!input_poll) {
		return -ENODEV;
	}

	si1141 = (struct si1141_data *)input_poll->private;
	if (!si1141) {
		return -ENODEV;
	}
#else
       struct si1141_data *si1141 = dev_get_drvdata(dev);
#endif

	sscanf(buf, "%ld %ld %ld", &value1, &value2, &value3);

	if(value1 != 65535) si1141->thld_ct= (uint16_t)value1;
	if(value2 != 65535) si1141->thld_lo= (uint16_t)value2;
	if(value3 != 65535) si1141->thld_hi= (uint16_t)value3;

	return count;

}


static DEVICE_ATTR(thld, 0664, ps_input_thld_show, ps_input_thld_store);

static ssize_t ps_dbg_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
	if (ps_dbg)
		return sprintf(buf, "%u\n",1);
	else
		return sprintf(buf, "%u\n",0);
}

static ssize_t ps_dbg_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count)
{
	int my_value;
	sscanf(buf, "%d", &my_value);

	if(my_value==0)
		ps_dbg=false;
	else
		ps_dbg=true;


	return count;

}

static DEVICE_ATTR(dbg, 0664, ps_dbg_show, ps_dbg_store);

static ssize_t param_data_show(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
	int i;
	int data = 0;
	char *pbuf = buf;
	struct si1141_data *si1141;
	si1141 = dev_get_drvdata(dev);

	for (i = 0; i < PARAM_MAX; i++) {
		if (i % 8 == 0) {
			pbuf += sprintf(pbuf, "%04x: ", i);
		}

		if (_cmd_write(si1141,  CMD_PARAM_QUERY | ( i & PARAM_MASK))) {
			return -1;
		}
		/* Wait for the data to present. */
		msleep(1);
		_i2c_read_one(si1141, SI1141_PARAM_RD, &data);
		pbuf += sprintf(pbuf, "%02x ", data);
		if ((i + 1) % 8 == 0) {
			pbuf += sprintf(pbuf, "\n");
		}
	}
	for (i = 0; i < 0x2E; i++) {
		if (i % 8 == 0) {
			pbuf += sprintf(pbuf, "0x%02x: ", i);
		}
		msleep(1);
		_i2c_read_one(si1141, i, &data);
		pbuf += sprintf(pbuf, "%02x ", data);
		if ((i + 1) % 8 == 0) {
			pbuf += sprintf(pbuf, "\n");
		}
	}
	pbuf += sprintf(pbuf, "\n");
	pbuf += sprintf(pbuf, "clh: (%d %d %d), bd:(%d %d)", si1141->thld_ct, si1141->thld_lo, si1141->thld_hi, si1141->base, si1141->diff);
	pbuf += sprintf(pbuf, "\n");

	return strlen(buf);
}

static DEVICE_ATTR(param_data, 0440, param_data_show, NULL);

static ssize_t dondoff1_state_show(struct device *dev,
                            struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", dondoff_state);
}
static DEVICE_ATTR(dondoff1_func, 0444, dondoff1_state_show, NULL);


static ssize_t dondoff2_state_show(struct device *dev,
                            struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", dondoff2_state);
}
static DEVICE_ATTR(dondoff2_func, 0444, dondoff2_state_show, NULL);


static void sysfs_register_bus_entry(struct si1141_data *si114x, struct i2c_client *client) {
	int rc = 0;

	/* Store the driver data into our device private structure */
	dev_set_drvdata(&client->dev, si114x);

	rc += device_create_file(&client->dev, &dev_attr_param_data);
	if (rc) {
		dev_err(&client->dev, "%s Unable to create sysfs bus files\n", __func__);
	} else {
		dev_dbg(&client->dev, "%s Created sysfs bus files\n", __func__);
	}

}


static void sysfs_register_class_input_entry_ps(struct si1141_data *si1141, struct device *dev) {
	int rc = 0;
	struct i2c_client *client = si1141->client;

	rc += device_create_file(dev, &dev_attr_ps_input_data);
	rc += device_create_file(dev, &dev_attr_ps_diff);
	rc += device_create_file(dev, &dev_attr_intr);
	rc += device_create_file(dev, &dev_attr_led_input_drive);
	rc += device_create_file(dev, &dev_attr_enable);
	rc += device_create_file(dev, &dev_attr_dbg);
	rc += device_create_file(dev, &dev_attr_thld);
	rc += device_create_file(dev, &dev_attr_meas);
	rc += device_create_file(dev, &dev_attr_rate);
	rc += device_create_file(dev, &dev_attr_gain);
	rc += device_create_file(dev, &dev_attr_selftest);
	rc += device_create_file(dev, &dev_attr_dondoff1_func);
	rc += device_create_file(dev, &dev_attr_dondoff2_func);

	if (rc) {
		dev_err(&client->dev, "%s Unable to create sysfs class files\n", __func__);
	} else {
		dev_dbg(&client->dev, "%s Created sysfs class files\n", __func__);
	}
}

static void sensor_irq_do_work(struct work_struct *work)
{
	disable_irq_nosync(pirq);
	si1141ps_doing(100);
	enable_irq(pirq);
}


/* PS open fops */
static int si1141_open(struct inode *inode, struct file *file)
{
	struct i2c_client *client;

	if (atomic_xchg(&si1141_opened, 1) == 1) {
		return -EBUSY;
	}

	file->private_data = (void*)si1141_private;
	client = si1141_private->client;

	/* Set the read pointer equal to the write pointer */
	if (mutex_lock_interruptible(&si1141_user_lock)) {
		dev_err(&client->dev, "%s: Unable to set read pointer\n", __func__);
		return -EAGAIN;
	}

	si1141_user_rd_ptr = si1141_user_wr_ptr;
	mutex_unlock(&si1141_user_lock);

        client = si1141_private->client;
	dev_info(&client->dev, "%s: Opened device\n", __func__);
	dev_info(&client->dev, "%p %s\n", inode, inode->i_sb->s_id);
	return 0;
}

/* PS release fops */
static int si1141_release(struct inode *inode, struct file *file)
{
	struct si1141_data *si1141 = (struct si1141_data *)file->private_data;
	struct i2c_client *client = si1141->client;

	if (atomic_xchg(&si1141_opened, 0) == 0) {
		dev_err(&client->dev, "%s: Device has not been opened\n", __func__);
		return -EBUSY;
	}

	dev_info(&client->dev, "%s: Closed device\n", __func__);
	return 0;
}

/* PS IOCTL */
static long si1141_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct si1141_data *si1141 = (struct si1141_data *)file->private_data;
	struct i2c_client *client = si1141->client;
	int rc, val;

	switch (cmd) {
	case PROXIMITYSENSOR_IOCTL_GET_ENABLED:
		val = si1141->ps_enable;
		dev_info(&client->dev, "%s: PROXIMITYSENSOR_IOCTL_GET_ENABLED, value = %d\n", __func__, val);
		rc = put_user(val, (unsigned long __user *)arg);
		break;

	case PROXIMITYSENSOR_IOCTL_ENABLE:
		if (get_user(val, (unsigned long __user *)arg)) {
			rc = -EFAULT;
			break;
		}
		dev_info(&client->dev, "%s: PROXIMITYSENSOR_IOCTL_ENABLE, value = %d\n", __func__, val);
		// Always enable it for dondoff
		//rc = val ? enable_ps(si1141, true) : enable_ps(si1141, false);
		rc = enable_ps(si1141, true);
		break;

	default:
		dev_info(&client->dev, "%s: invalid cmd\n", __func__);
		rc = -EINVAL;
	}

	return rc;
}


static const struct file_operations si1141_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = si1141_ioctl,
	.open = si1141_open,
	.release = si1141_release,
};

struct miscdevice si1141_misc = {
	.name = "si1141ps",
	.minor = MISC_DYNAMIC_MINOR,
	.fops = &si1141_fops
};

static int si1141ps_addr(struct device *dev,
				struct si1141_data *si1141)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int rc;
	

	rc = of_property_read_u32(np, "reg", &temp_val);
	if (rc)
	{
		dev_err(dev, "Unable to read slave_address\n");
		return rc;
	} 
	else
	{
		si1141->slave_addr = (uint8_t)temp_val;
	}
  


	return 0;
}

static void calculate_dondoff(struct si1141_data *si114x) {
	int max = dondoff2_array[0];
	int min = dondoff2_array[0];
	int i;

	for(i = 1; i < DONDOFF_SIZE; i++) {
		if(dondoff2_array[i] > max)
			max = dondoff2_array[i];
		if(dondoff2_array[i] < min)
			min = dondoff2_array[i];
	}

	dondoff2_max = max;
	dondoff2_min = min;

	if(dondoff2_state == 0) {
		si1141ps_thld_set(si114x, dondoff2_min + (dondoff2_min >> dondoff2_variant));
	} else if(dondoff2_state == 1) {
		si1141ps_thld_set(si114x, dondoff2_max - (dondoff2_max >> dondoff2_variant));
	}
}

static void si1141ps_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct si1141_data *led = container_of(dwork, struct si1141_data, dwork);

	int ret = si1141ps_doing(50);
	schedule_delayed_work(&led->dwork, msecs_to_jiffies(500));

	if(dondoff2_count == 0) {
		dondoff2_temp  /= 2;
		dondoff2_array[dondoff2_index] = dondoff2_temp;
		calculate_dondoff(led);

		if(dondoff2_max && (dondoff2_max - ret > dondoff2_max >> dondoff2_variant) && dondoff2_state != 0) {
			dondoff2_state = 0;
		}
		else if(dondoff2_min && (ret - dondoff2_min > dondoff2_min >> dondoff2_variant) && dondoff2_state != 1) {
			dondoff2_state = 1;
		}

		if(dondoff_state == 0 && dondoff2_state == 0) {
			if(!g_prox_status) {
				if(g_prox_debounce == 0) {
					input_report_abs(led->si1141_input_dev, ABS_DISTANCE, 1);
					input_sync(led->si1141_input_dev);

					input_report_key(led->si1141_input_dev, KEY_SLEEP, 1);
					mdelay(10);
					input_report_key(led->si1141_input_dev, KEY_SLEEP, 0);
					input_sync(led->si1141_input_dev);

					g_prox_status = 1;
					g_prox_debounce = DONDOFF_DEBOUNCE;
				} else {
					g_prox_debounce--;
				}
			}
		} else {
			if(g_prox_status) {
				input_report_abs(led->si1141_input_dev, ABS_DISTANCE, 0);
				input_sync(led->si1141_input_dev);

				input_report_key(led->si1141_input_dev, KEY_WAKEUP, 1);
				mdelay(10);
				input_report_key(led->si1141_input_dev, KEY_WAKEUP, 0);
				input_sync(led->si1141_input_dev);

				g_prox_status = 0;
			}

			g_prox_debounce = DONDOFF_DEBOUNCE;
		}

		dondoff2_temp = ret;
		dondoff2_count++;
		dondoff2_count %= 2;
		dondoff2_index++;
		dondoff2_index %= DONDOFF_SIZE;
	} else {
		dondoff2_temp += ret;
		dondoff2_count++;
		dondoff2_count %= 2;
	}

	return;
}

static int si1141_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct si1141_data *si1141 = NULL;
	int rc=0;

	g_si1141 = kzalloc(sizeof(struct si1141_data), GFP_KERNEL);
	si1141 = g_si1141;
	if (!si1141)
	{
		dev_err(&client->dev, "%s: Unable to allocate memory for driver structure\n", __func__);
		return -ENOMEM;
	}

	rc = of_get_named_gpio_flags(client->dev.of_node, "si1141,int-gpio",
			0, NULL);
	if (rc < 0) {
		dev_err(&client->dev, "Unable to read interrupt pin number\n");
		return rc;
	} else {
		si1141->intr = rc;
		dev_err(&client->dev, "intr: %d\n", si1141->intr);
	}

	if (of_property_read_bool(client->dev.of_node, "mic-fpc")){
		strcpy(INPUT_DEVICE_NAME_PS,INPUT_DEVICE_NAME_PS_MIC);
		msleep(3000);
	}

	if (of_property_read_bool(client->dev.of_node, "front-fpc"))
		strcpy(INPUT_DEVICE_NAME_PS,INPUT_DEVICE_NAME_PS_FRONT);

	if (of_find_property(client->dev.of_node, "si1141-vdd-supply", NULL)) {
		si1141->vdd = devm_regulator_get(&client->dev, "si1141-vdd");
		if (IS_ERR_OR_NULL(si1141->vdd)) {
			rc = PTR_RET(si1141->vdd);
			if (rc != EPROBE_DEFER)
				pr_err("get vdd failed, rc = %d\n", rc);
			return rc;
		}
	}

	INIT_DELAYED_WORK(&si1141->dwork, si1141ps_work);
	schedule_delayed_work(&si1141->dwork, msecs_to_jiffies(500));

	lp_wq = create_singlethread_workqueue("si1141_wq");
	if (!lp_wq) {
		pr_err("%s: can't create workqueue\n", __func__);
		rc = -ENOMEM;
		goto err_out;
	}


	si1141->client = client;
	i2c_set_clientdata(client, si1141);
	si1141->ps_polling_delay = msecs_to_jiffies(PS_POLL_INTERVAL);
	si1141->resp_id = 0;
	si1141->thld_hi= PS1_THLD_HI;
	si1141->thld_lo= PS1_THLD_LO;
	si1141->thld_ct= PS1_THLD_CT;

	if( si1141ps_addr(&client->dev, si1141) < 0 )
	{
		rc = -EBUSY;
		goto err_out;  
	}

	if (si1141->vdd) {
		if (regulator_count_voltages(si1141->vdd) > 0) {
			rc = regulator_set_voltage(si1141->vdd,
				1800000, 1800000);
			if (rc < 0) {
				pr_err("set vdd voltage failed, rc = %d\n", rc);
				return rc;
			}
		}

		rc = regulator_enable(si1141->vdd);
		if (rc) {
			pr_err("enable vdd voltage failed, rc = %d\n", rc);
			return rc;
		}
		usleep_range(250000, 251000);

	}

	_i2c_write_one(si1141, SI1141_COMMAND, CMD_RESET);
	usleep_range(100, 110);

	si1141->led[0].drive_strength = PS1_LED_DEFAULT;
	if (si1141->led[0].drive_strength)
		si1141->led[0].enable = 1;

	si1141->led[1].drive_strength = SI1141_LED_OFF;
	if (si1141->led[1].drive_strength)
		si1141->led[1].enable = 1;

	si1141->led[2].drive_strength = SI1141_LED_OFF;
	if (si1141->led[2].drive_strength)
		si1141->led[2].enable = 1;

	/* Register the sysfs bus entries */
	sysfs_register_bus_entry(si1141, client);

	si1141->lp_ps_wq = create_singlethread_workqueue("si1141ps_wq");
	if (!si1141->lp_ps_wq) {
		pr_err("%s: can't create workqueue\n", __func__);
		rc = -ENOMEM;
		goto err_create_singlethread_als_workqueue;
	}

	msleep(5);
	rc = gpio_request(si1141->intr , "si1141ps_intr");
	if (rc < 0) {
		pr_err("%s: gpio %d request failed (%d)\n",
			__func__, si1141->intr , rc);
		return rc;
	}

	rc = gpio_direction_input(si1141->intr );
	if (rc < 0) {
		pr_err(
			"%s: fail to set gpio %d as input (%d)\n",
			__func__, si1141->intr , rc);
		goto fail_free_intr_pin;
	}

	pirq = gpio_to_irq(si1141->intr);

#ifdef SI1141_USE_INPUT_POLL
	/* Setup the input subsystem for the PS */
	if (setup_ps_polled_input(si1141)) {
		dev_err(&client->dev, "%s: Unable to allocate ps polled input resource\n", __func__);
		goto err_out;
	}
	/* Store the driver data into our private structure */
	sysfs_register_class_input_entry_ps(si1141, &si1141->input_dev_ps.input->dev);
	/* Store the driver data into our private structure */
	si1141->input_dev_ps.input_poll->private = si1141;

	rc  = sysfs_create_link(&client->dev.kobj, &si1141->input_dev_ps.input->dev.kobj, "hal");
	if (rc){
		pr_err("sysfs_create_link failed, rc = %d\n", rc);
	}

#else

	/* Setup the input subsystem for the PS */
	if (setup_ps_input(si1141)) {
		dev_err(&client->dev, "%s: Unable to allocate ps input resource\n", __func__);
		goto err_out;
	}
	sysfs_register_class_input_entry_ps(si1141, &si1141->si1141_input_dev->dev);

	dev_set_drvdata(&si1141->si1141_input_dev->dev, si1141);

	rc  = sysfs_create_link(&client->dev.kobj, &si1141->si1141_input_dev->dev.kobj, "hal");
	if (rc){
		pr_err("sysfs_create_link failed, rc = %d\n", rc);
	}


#endif  /* SI1141_USE_INPUT_POLL */



#if ENABLE_EARLYSUSPEND
	/* Setup the suspend and resume functionality */
	INIT_LIST_HEAD(&si1141->early_suspend.link);
	si1141->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	si1141->early_suspend.suspend = si114x_early_suspend;
	si1141->early_suspend.resume = si114x_late_resume;
	register_early_suspend(&si1141->early_suspend);
#endif

	rc = request_any_context_irq(pirq,
				si1141ps_irq_handler,
				IRQF_TRIGGER_FALLING,
				"si1141ps",
				si1141);
	if (rc < 0) {
		pr_err(
			"%s: req_irq(%d) fail for gpio %d (%d)\n",
			__func__, pirq, si1141->intr , rc);
	}

	/* Setup the device syncronization API via the fileops */
	mutex_init(&si1141_user_lock);
	mutex_init(&si1141_resp_lock);
	si1141_user_wr_ptr = si1141_fifo_buffer;
	si1141_user_rd_ptr = si1141_fifo_buffer;
	misc_register(&si1141_misc);
	si1141_private = si1141;
	rc = enable_irq_wake(pirq);
	dev_err(&client->dev, "%s: si1141ps go ...\n", __func__);

	// Always enable it for dondoff
	enable_ps(si1141, true);

	return 0;

err_create_singlethread_als_workqueue:  

fail_free_intr_pin:
	gpio_free(si1141->intr);

err_out:

	kfree(si1141);
	return -ENODEV;
}

int si1142ps_query_status(void)
{
	return g_prox_status;
}
EXPORT_SYMBOL_GPL(si1142ps_query_status);


static const struct i2c_device_id si1141_id[] = {
	{ DEVICE_NAME, 0 },
};

static struct of_device_id si1141ps_match_table[] = {
	{ .compatible = "silab,1141ps",},
	{ },
};

static struct i2c_driver si1141_driver = {
	.probe = si1141_probe,
	.id_table = si1141_id,
	.driver = {
		.owner = THIS_MODULE,
		.name = DEVICE_NAME,
		.of_match_table = of_match_ptr(si1141ps_match_table),
	},
};

static int __init si1141ps_init(void)
{
	return i2c_add_driver(&si1141_driver);
}

static void __exit si1141ps_exit(void)
{
	i2c_del_driver(&si1141_driver);
}

module_init(si1141ps_init);
module_exit(si1141ps_exit);

MODULE_AUTHOR("quanta@quanta.corp");
MODULE_DESCRIPTION("SI1141 Proximity Driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION(DRIVER_VERSION);
