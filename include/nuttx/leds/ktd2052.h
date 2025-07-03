/****************************************************************************
 * include/nuttx/leds/ktd2052.h
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

#ifndef __INCLUDE_NUTTX_LEDS_KTD2052_H
#define __INCLUDE_NUTTX_LEDS_KTD2052_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Set RGB value for a single module */
#define KTDIOSETRGB         _ULEDIOC(0)

/* Set operating mode */
#define KTDIOSETMODE        _ULEDIOC(1)

/* Set pattern configuration */
#define KTDIOSETPATTERN     _ULEDIOC(2)

/* Set pattern slots for a module */
#define KTDIOSETSLOTS       _ULEDIOC(3)

/* Get monitor register status */
#define KTDIOGETMONITOR     _ULEDIOC(4)

/* Set watchdog value */
#define KTDIOSETWDOG        _ULEDIOC(5)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#define KTD2052_MODE_SHUTDOWN     0
#define KTD2052_MODE_NIGHT        1
#define KTD2052_MODE_NORMAL       2

#define KTD2052_TEMP_LIMIT_135    0
#define KTD2052_TEMP_LIMIT_120    1
#define KTD2052_TEMP_LIMIT_105    2
#define KTD2052_TEMP_LIMIT_90     3

#define KTD2052_FADE_RATE_32MS    0
#define KTD2052_FADE_RATE_63MS    1
#define KTD2052_FADE_RATE_125MS   2
#define KTD2052_FADE_RATE_250MS   3
#define KTD2052_FADE_RATE_500MS   4
#define KTD2052_FADE_RATE_1000MS  5
#define KTD2052_FADE_RATE_2000MS  6
#define KTD2052_FADE_RATE_4000MS  7

/* MONITOR status bits */

#define KTD2052_MONITOR_UV_OT_STAT  0x01 /* Under voltage/over temperature */
#define KTD2052_MONITOR_COOL_STAT   0x02 /* CoolExtend is active */
#define KTD2052_MONITOR_BE_STAT     0x04 /* BrightExtend is active */
#define KTD2052_MONITOR_SC_STAT     0x08 /* Short circuit detected */

/* Structure for setting operating mode */

struct ktd2052_mode_s
{
  uint8_t mode;          /* 0=off, 1=night mode, 2=normal mode */
  bool bright_extend;    /* Enable BrightExtend feature */
  uint8_t temp_limit;    /* Temperature limit (0=135C to 3=90C) */
  uint8_t fade_rate;     /* Fade rate (0-7) */
};

/* Structure for configuring pattern generator */

struct ktd2052_pattern_s
{
  uint8_t slots;         /* Number of pattern slots (4, 6, or 8) */
  uint8_t duration;      /* Duration for each slot (0-7) */
  uint8_t fade_rate1;    /* Secondary fade rate (0-7) */
  uint8_t watchdog;      /* Number of pattern cycles (0-255) */
};

/* Structure for setting pattern slots */

struct ktd2052_slots_s
{
  uint8_t module;        /* RGB module number (1-4) */
  uint8_t slots;         /* Bit pattern for active slots */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: ktd2052_register
 *
 * Description:
 *   Register the KTD2052 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/leds0"
 *   i2c     - An instance of the I2C interface to communicate with KTD2052
 *   addr    - The I2C address of the KTD2052
 *   freq    - The I2C frequency to use for the KTD2052
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ktd2052_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                     uint8_t addr, uint32_t freq);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_LEDS_KTD2052_H */
