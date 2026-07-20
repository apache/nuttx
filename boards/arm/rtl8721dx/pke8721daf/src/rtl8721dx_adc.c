/****************************************************************************
 * boards/arm/rtl8721dx/pke8721daf/src/rtl8721dx_adc.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/param.h>
#include <syslog.h>
#include <errno.h>

#include "ameba_gpio.h"
#include "ameba_adc.h"
#include "rtl8721dx_pke8721daf.h"

#ifdef CONFIG_AMEBA_ADC

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Channels sampled on /dev/adc0 and the analog pad each is wired to.  The
 * external channels CH0..CH6 map to pads PB19..PB13; this board exposes CH0
 * on PB19 and CH1 on PB18.  Any external channel can be listed here;
 * internal channels (CH7..CH10: temperature, VBAT) would carry
 * AMEBA_ADC_PIN_NC.
 */

static const uint8_t g_adc_channels[] =
{
  0,                 /* ADC_CH0 */
  1,                 /* ADC_CH1 */
};

static const uint8_t g_adc_pins[] =
{
  AMEBA_PB(19),      /* CH0 -> PB19 */
  AMEBA_PB(18),      /* CH1 -> PB18 */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rtl8721dx_adc_initialize
 *
 * Description:
 *   Register the board's ADC channels at /dev/adc0.
 *
 ****************************************************************************/

int rtl8721dx_adc_initialize(void)
{
  int ret;

  ret = ameba_adc_register("/dev/adc0", g_adc_channels, g_adc_pins,
                           nitems(g_adc_channels));
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: ameba_adc_register(/dev/adc0) failed: %d\n", ret);
      return ret;
    }

  return OK;
}

#endif /* CONFIG_AMEBA_ADC */
