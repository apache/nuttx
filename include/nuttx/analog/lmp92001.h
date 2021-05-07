/****************************************************************************
 * include/nuttx/analog/lmp92001.h
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

#ifndef __INCLUDE_NUTTX_ANALOG_LMP92001_H
#define __INCLUDE_NUTTX_ANALOG_LMP92001_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/analog/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IOCTL Commands ***********************************************************/

/* Cmd: ANIOC_LMP92001_DAC_SET_REF         Arg: bool value
 * Cmd: ANIOC_LMP92001_DAC_UPDATEALL       Arg: uint16_t value
 * Cmd: ANIOC_LMP92001_ADC_SET_REF         Arg: bool value
 * Cmd: ANIOC_LMP92001_ADC_ENABLE          Arg: lmp92001_adc_enable_e channel
 * Cmd: ANIOC_LMP92001_ADC_SINGLESHOT_CONV Arg: none
 * Cmd: ANIOC_LMP92001_ADC_CONTINUOUS_CONV Arg: none
 * Cmd: ANIOC_LMP92001_ADC_READ_CHANNEL    Arg: struct adc_msg_s *channel
 */

#define ANIOC_LMP92001_DAC_SET_REF          _ANIOC(AN_LMP92001_FIRST + 0)
#define ANIOC_LMP92001_DAC_UPDATEALL        _ANIOC(AN_LMP92001_FIRST + 1)
#define ANIOC_LMP92001_ADC_SET_REF          _ANIOC(AN_LMP92001_FIRST + 2)
#define ANIOC_LMP92001_ADC_ENABLE           _ANIOC(AN_LMP92001_FIRST + 3)
#define ANIOC_LMP92001_ADC_SINGLESHOT_CONV  _ANIOC(AN_LMP92001_FIRST + 4)
#define ANIOC_LMP92001_ADC_CONTINUOUS_CONV  _ANIOC(AN_LMP92001_FIRST + 5)
#define ANIOC_LMP92001_ADC_READ_CHANNEL     _ANIOC(AN_LMP92001_FIRST + 6)

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum lmp92001_ref_e
{
  LMP92001_REF_INTERNAL = 0U,
  LMP92001_REF_EXTERNAL
};

enum lmp92001_adc_enable_e
{
  LMP92001_ADC_EN_CH1   = 1 << 0U,
  LMP92001_ADC_EN_CH2   = 1 << 1U,
  LMP92001_ADC_EN_CH3   = 1 << 2U,
  LMP92001_ADC_EN_CH4   = 1 << 3U,
  LMP92001_ADC_EN_CH5   = 1 << 4U,
  LMP92001_ADC_EN_CH6   = 1 << 5U,
  LMP92001_ADC_EN_CH7   = 1 << 6U,
  LMP92001_ADC_EN_CH8   = 1 << 7U,
  LMP92001_ADC_EN_CH9   = 1 << 8U,
  LMP92001_ADC_EN_CH10  = 1 << 9U,
  LMP92001_ADC_EN_CH11  = 1 << 10U,
  LMP92001_ADC_EN_CH12  = 1 << 11U,
  LMP92001_ADC_EN_CH13  = 1 << 12U,
  LMP92001_ADC_EN_CH14  = 1 << 13U,
  LMP92001_ADC_EN_CH15  = 1 << 14U,
  LMP92001_ADC_EN_CH16  = 1 << 15U,
  LMP92001_ADC_EN_CH17  = 1 << 16U,
  LMP92001_ADC_EN_ALL   = 0x1ffffu
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: lmp92001_gpio_initialize
 *
 * Description:
 *   Instantiate and configure the LMP92001 device driver to use the provided
 *   I2C device instance.
 *
 * Input Parameters:
 *   i2c     - An I2C driver instance
 *   minor   - The device i2c address
 *
 * Returned Value:
 *   An ioexpander_dev_s instance on success, NULL on failure.
 *
 ****************************************************************************/

struct i2c_master_s;      /* Forward reference */
struct ioexpander_dev_s;  /* Forward reference */

FAR struct ioexpander_dev_s *
lmp92001_gpio_initialize(FAR struct i2c_master_s *i2c, uint8_t addr);

#endif /* __INCLUDE_NUTTX_ANALOG_LMP92001_H */
