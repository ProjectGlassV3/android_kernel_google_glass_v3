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

#ifndef __BOARD_HWID_H
#define __BOARD_HWID_H

uint8_t read_hw_id(void);
uint16_t read_dis_drvid(void);
uint8_t read_evb_id(void);
uint8_t read_dis_hinge(void);
uint8_t read_offline_charging(void);

#endif
