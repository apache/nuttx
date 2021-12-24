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
#include <debug.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GH3020 PPG has 4 different measurements and each one should be a node. */

#define GH3020_PPG0_SENSOR_IDX   0          /* PPG0 (green) sensor index. */
#define GH3020_PPG1_SENSOR_IDX   1          /* PPG1 (red) sensor index. */
#define GH3020_PPG2_SENSOR_IDX   2          /* PPG2 (IR sensor index. */
#define GH3020_PPG3_SENSOR_IDX   3          /* PPG3 (dark) sensor index. */
#define GH3020_SENSOR_NUM        4          /* Total PPG sensors number. */

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

int gh3020_spiread(FAR uint8_t *recvbuf, uint16_t nbytes);
int gh3020_spiwrite(FAR uint8_t *sendbuf, uint16_t nbytes);
void gh3020_spi_csctrl(uint8_t pinlevel);
void gh3020_rstctrl(uint8_t pinlevel);
void gh3020_transdata(FAR struct sensor_event_ppgq *ppg, uint8_t chidx);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif // __DRIVERS_SENSORS_GH3020_GH3020_BRIDGE_H
