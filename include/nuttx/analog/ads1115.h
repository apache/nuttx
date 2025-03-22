/****************************************************************************
 * include/nuttx/analog/ads1115.h
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

#ifndef __INCLUDE_NUTTX_ANALOG_ADS1115_H
#define __INCLUDE_NUTTX_ANALOG_ADS1115_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/analog/ioctl.h>
#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IOCTL Commands
 * Cmd: ANIOC_ADS1115_SET_PGA         Arg: enum ads1115_pga_e
 * Cmd: ANIOC_ADS1115_SET_MODE        Arg: enum ads1115_mode_e
 * Cmd: ANIOC_ADS1115_SET_DR          Arg: enum ads1115_dr_e
 * Cmd: ANIOC_ADS1115_SET_COMP_MODE   Arg: enum ads1115_comp_mode_e
 * Cmd: ANIOC_ADS1115_SET_COMP_POL    Arg: enum ads1115_comp_pol_e
 * Cmd: ANIOC_ADS1115_SET_COMP_LAT    Arg: enum ads1115_comp_lat_e
 * Cmd: ANIOC_ADS1115_SET_COMP_QUEUE  Arg: enum ads1115_comp_queue_e
 * Cmd: ANIOC_ADS1115_READ_CHANNEL    Arg: struct adc_msg_s *channel
 * Cmd: ANIOC_ADS1115_SET_HI_THRESH   Arg: uint16_t value
 * Cmd: ANIOC_ADS1115_SET_LO_THRESH   Arg: uint16_t value
 */

#define ANIOC_ADS1115_SET_PGA _ANIOC(AN_ADS1115_FIRST + 0)
#define ANIOC_ADS1115_SET_MODE _ANIOC(AN_ADS1115_FIRST + 1)
#define ANIOC_ADS1115_SET_DR _ANIOC(AN_ADS1115_FIRST + 2)
#define ANIOC_ADS1115_SET_COMP_MODE _ANIOC(AN_ADS1115_FIRST + 3)
#define ANIOC_ADS1115_SET_COMP_POL _ANIOC(AN_ADS1115_FIRST + 4)
#define ANIOC_ADS1115_SET_COMP_LAT _ANIOC(AN_ADS1115_FIRST + 5)
#define ANIOC_ADS1115_SET_COMP_QUEUE _ANIOC(AN_ADS1115_FIRST + 6)
#define ANIOC_ADS1115_READ_CHANNEL _ANIOC(AN_ADS1115_FIRST + 7)
#define ANIOC_ADS1115_SET_HI_THRESH _ANIOC(AN_ADS1115_FIRST + 8)
#define ANIOC_ADS1115_SET_LO_THRESH _ANIOC(AN_ADS1115_FIRST + 9)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Input Multiplexer configuration. */

enum ads1115_mux_e
{
  ADS1115_MUX1 = 0u, /* AINP = AIN0 and AINN = AIN1 (default) */
  ADS1115_MUX2,      /* AINP = AIN0 and AINN = AIN3 */
  ADS1115_MUX3,      /* AINP = AIN1 and AINN = AIN3 */
  ADS1115_MUX4,      /* AINP = AIN2 and AINN = AIN3 */
  ADS1115_MUX5,      /* AINP = AIN0 and AINN = GND */
  ADS1115_MUX6,      /* AINP = AIN1 and AINN = GND */
  ADS1115_MUX7,      /* AINP = AIN2 and AINN = GND */
  ADS1115_MUX8,      /* AINP = AIN3 and AINN = GND */
};

/* Programmable gain amplifier configuration */

enum ads1115_pga_e
{
  ADS1115_PGA1 = 0u, /* FSR = ±6.144V */
  ADS1115_PGA2,      /* FSR = ±4.096V */
  ADS1115_PGA3,      /* FSR = ±2.048V (default) */
  ADS1115_PGA4,      /* FSR = ±1.024V */
  ADS1115_PGA5,      /* FSR = ±0.512V */
  ADS1115_PGA6,      /* FSR = ±0.256V */
  ADS1115_PGA7,      /* FSR = ±0.256V */
  ADS1115_PGA8,      /* FSR = ±0.256V */
};

/* Device Operating Mode */

enum ads1115_mode_e
{
  ADS1115_MODE1 = 0u, /* Continuous conversion mode */
  ADS1115_MODE2       /* Power-down single-shot mode (default) */
};

/* Data rate */

enum ads1115_dr_e
{
  ADS1115_DR1 = 0u, /* 8 SPS */
  ADS1115_DR2,      /* 16 SPS */
  ADS1115_DR3,      /* 32 SPS */
  ADS1115_DR4,      /* 64 SPS */
  ADS1115_DR5,      /* 128 SPS (default) */
  ADS1115_DR6,      /* 250 SPS */
  ADS1115_DR7,      /* 475 SPS */
  ADS1115_DR8,      /* 860 SPS */
};

/* Comparator mode */

enum ads1115_comp_mode_e
{
  ADS1115_COMP_MODE1 = 0u, /* Traditional comparator (default) */
  ADS1115_COMP_MODE2       /* Window comparator */
};

/* Comparator Polarity */

enum ads1115_comp_pol_e
{
  ADS1115_COMP_POL1 = 0u, /* Active low (default) */
  ADS1115_COMP_POL2       /* Active high */
};

/* Latching comparator */

enum ads1115_comp_lat_e
{
  ADS1115_COMP_LAT1 = 0u, /* Nonlatching comparator. The ALERT/RDY pin does
                           * not latch when asserted (default) */
  ADS1115_COMP_LAT2       /* Latching comparator. See datasheet. */
};

/* Comparator queue and disable */

enum ads1115_comp_queue_e
{
  ADS1115_COMP_QUEUE1 = 0u, /* Assert after one conversion */
  ADS1115_COMP_QUEUE2,      /* Assert after two conversions */
  ADS1115_COMP_QUEUE3,      /* Assert after four conversions */
  ADS1115_COMP_QUEUE4,      /* Disable comparator and set ALERT/RDY pin to
                             * high-impedance (default) */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

FAR struct adc_dev_s *ads1115_initialize(struct i2c_master_s *i2c,
                                         uint8_t addr);

#endif /* __INCLUDE_NUTTX_ANALOG_ADS1115_H */
