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

#include <linux/module.h>
#include <linux/hwid.h>
#include <linux/string.h>

static uint8_t hw_id;
static uint16_t dis_drvid;
static uint8_t evb_id;
static uint8_t offine_charging;
static uint8_t dis_hinge;
 
static uint8_t atoi(const char *name)
{
         uint8_t val = 0xFF;
 
         switch (*name) {
                 case '0' ... '9':
                         val = (*name - '0');
                         break;
                 case 'A' ... 'F':
                         val = 10 + (*name - 'A');
                         break;
                 case 'a' ... 'f':
                         val = 10 + (*name - 'a');
                         break;
                 default:
                         break;
         }
 
         return val;
}

static int __init hw_id_setup(char *p)
{
        hw_id = atoi(p);

        return 1;
}
__setup("hwid=", hw_id_setup);

uint8_t read_hw_id(void)
{
        return hw_id;
}

static int __init dis_drvid_setup(char *p)
{
	dis_drvid = 0xFFFF;

	if (!strncmp(p, "254", 3))
		dis_drvid = 254;
	else if (!strncmp(p, "255", 3))
		dis_drvid = 255;

	return 1;
}
__setup("dis_drvid=", dis_drvid_setup);

uint16_t read_dis_drvid(void)
{
        return dis_drvid;
}

static int __init evb_id_setup(char *p)
{
        evb_id = atoi(p);

        return 1;
}
__setup("evbid=", evb_id_setup);

uint8_t read_evb_id(void)
{
        return evb_id;
}

static int __init dis_hinge_setup(char *p)
{
        dis_hinge = atoi(p);

        return 1;
}
__setup("dis_hinge=", dis_hinge_setup);

uint8_t read_dis_hinge(void)
{
        return dis_hinge;
}

static int __init offline_charging_setup(char *p)
{
	offine_charging = 0;

	if (!strncmp(p, "charger", 7))
		offine_charging = 1;

	return 1;
}
__setup("androidboot.mode=", offline_charging_setup);

uint8_t read_offline_charging(void)
{
        return offine_charging;
}

