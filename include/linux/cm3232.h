/* include/linux/CM3232.h
 *
 * Copyright (C) 2016 Vishay Capella Microsystems Limited
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

#ifndef __LINUX_CM3232_H
#define __LINUX_CM3232_H

#define CM3232_I2C_NAME "cm3232"

/* Define Slave Address*/
#define	CM3232_slave_add	0x20>>1


/*Define Command Code*/
#define		CM3232_ALS_CMD		    0x00
#define		CM3232_ALS_DATA		    0x50

/*for ALS command*/
#define CM3232_ALS_RESET 	    (1 << 6)
#define CM3232_ALS_RESET_MASK			(0xbf) 

#define CM3232_ALS_IT_100MS 	(0 << 2)
#define CM3232_ALS_IT_200MS 	(1 << 2)
#define CM3232_ALS_IT_400MS 	(2 << 2)
#define CM3232_ALS_IT_800MS 	(3 << 2)
#define CM3232_ALS_IT_1600MS 	(4 << 2)
#define CM3232_ALS_IT_3200MS 	(5 << 2)
#define CM3232_ALS_HS_HIGH		(1 << 1)
#define CM3232_ALS_SD			(1 << 0)
#define CM3232_ALS_SD_MASK			(0xfe)


#define LS_PWR_ON				(1 << 0)

struct cm3232_platform_data {
	int intr;
	int (*power)(int, uint8_t); /* power to the chip */
	uint8_t slave_addr;
	uint16_t ls_cmd;	
};

#define		CM3232_CAL_DATA_DEF		    6000

#endif
