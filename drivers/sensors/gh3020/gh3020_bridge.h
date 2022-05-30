/****************************************************************************
 * drivers/sensors/gh3020/gh3020_bridge.h
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

#ifndef __DRIVERS_SENSORS_GH3020_GH3020_BRIDGE_H
#define __DRIVERS_SENSORS_GH3020_GH3020_BRIDGE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <nuttx/kmalloc.h>
#include <nuttx/sensors/sensor.h>
#include <syslog.h>
#include "gh3x2x_drv.h"
#include "gh3020_def.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GH3020 PPG has 4 different measurements and each one should be a node. */

#define GH3020_PPG0_SENSOR_IDX   0          /* PPG0 (green dynamic) */
#define GH3020_PPG1_SENSOR_IDX   1          /* PPG1 (red dynamic) */
#define GH3020_PPG2_SENSOR_IDX   2          /* PPG2 (IR dynamic) */
#define GH3020_PPG3_SENSOR_IDX   3          /* PPG3 (dark fixed) */
#define GH3020_PPG4_SENSOR_IDX   4          /* PPG4 (green fixed) */
#define GH3020_PPG5_SENSOR_IDX   5          /* PPG5 (IR fixed). */
#define GH3020_SENSOR_NUM        6          /* Total PPG sensors number. */

/* Macros */

/* Print log with syslog if debug config enabled, otherwise don't print. */

#if defined(CONFIG_SENSORS_GH3020_DEBUG_LOG)
#define GH3020_DEBUG_LOG(format, ...)   syslog(LOG_INFO, format, ##__VA_ARGS__)
#else
#define GH3020_DEBUG_LOG(format, ...)
#endif

/* Print log with syslog if factest config enabled, otherwise don't print. */

#if defined(CONFIG_FACTEST_SENSORS_GH3020)
#define GH3020_FACTEST_LOG(format, ...) syslog(LOG_INFO, format, ##__VA_ARGS__)
#else
#define GH3020_FACTEST_LOG(format, ...)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void gh3020_spi_sendcmd(uint8_t cmd);
void gh3020_spi_writereg(uint16_t regaddr, uint16_t regval);
uint16_t gh3020_spi_readreg(uint16_t regaddr);
void gh3020_spi_readfifo(FAR uint8_t *pbuf, uint16_t len);
void gh3020_spi_writebits(uint16_t regaddr, uint8_t lsb, uint8_t msb,
                          uint16_t val);
uint16_t gh3020_spi_readbits(uint16_t regaddr, uint8_t lsb, uint8_t msb);
void gh3020_rstctrl(uint8_t pinlevel);
uint16_t gh3020_get_efuse(void);
void gh3020_get_ppg_data(FAR const struct gh3020_frameinfo_s *pfameinfo);
#ifdef CONFIG_FACTEST_SENSORS_GH3020
void gh3020_get_rawdata(FAR uint8_t *pbuf, uint16_t len);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif // __DRIVERS_SENSORS_GH3020_GH3020_BRIDGE_H
