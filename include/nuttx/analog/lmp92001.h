/****************************************************************************
 * include/nuttx/analog/lmp92001.h
 *
 *   Copyright (C) 2018 Abdelatif Guettouche. All rights reserved.
 *   Author: Abdelatif Guettouche <abdelatif.guettouche@gmail.com>
 *
 * This file is a part of NuttX:
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
/* Cmd: ANIOC_LMP92001_DAC_SET_REF          Arg: bool value
 * Cmd: ANIOC_LMP92001_DAC_UPDATEALL        Arg: uint16_t value
 * Cmd: ANIOC_LMP92001_ADC_SET_REF          Arg: bool value
 * Cmd: ANIOC_LMP92001_ADC_ENABLE           Arg: lmp92001_adc_enable_e channel
 * Cmd: ANIOC_LMP92001_ADC_SINGLESHOT_CONV  Arg: none
 * Cmd: ANIOC_LMP92001_ADC_CONTINUOUS_CONV  Arg: none
 * Cmd: ANIOC_LMP92001_ADC_READ_CHANNEL     Arg: struct adc_msg_s *channel
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
  LMP92001_ADC_EN_ALL   = 0x1FFFFU
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
