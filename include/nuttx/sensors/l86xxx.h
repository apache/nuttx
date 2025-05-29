/****************************************************************************
 * include/nuttx/sensors/l86xxx.h
 *
 * NOTE: EXPERIMENTAL DRIVER
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_SENSORS_L86XXX_H
#define __INCLUDE_NUTTX_SENSORS_L86XXX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>

/****************************************************************************
 * Pre-Processor Declarations
 ****************************************************************************/

/****************************************************************************
 * Public Data Types
 ****************************************************************************/

typedef enum
{
  CMD_HOT_START = 101,
  CMD_WARM_START = 102,
  CMD_COLD_START = 103,
  CMD_FULL_COLD_START = 104,
  CMD_STANDBY_MODE = 161,
  SET_POS_FIX = 220,
  SET_NMEA_BAUDRATE = 251,
  FR_MODE = 886,
} L86XXX_PMTK_COMMAND;

typedef enum
{
  NORMAL = 0,
  FITNESS = 1,
  AVIATION = 2,
  BALLOON = 3,
  STANDBY = 4,
} L86XXX_OPERATIONAL_MODE;

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: l86xxx_register
 *
 * Description:
 *   Register the L86-XXX GNSS device driver.
 *
 * Arguments:
 *    uartpath  -  The path to the UART character driver connected to the
 *                 GNSS module
 *    devno     -  The device number to use for the topic (i.e. /dev/mag0)
 ****************************************************************************/

int l86xxx_register(FAR const char *uartpath, int devno);

#endif /* __INCLUDE_NUTTX_SENSORS_L86XXX_H */