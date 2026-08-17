/****************************************************************************
 * boards/arm/rtl8721f/rtl8721f_evb/src/rtl8721f_adc.c
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
#include "rtl8721f_rtl8721f_evb.h"

#ifdef CONFIG_AMEBA_ADC

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Channels sampled on /dev/adc0 and the analog pad each is wired to.  The
 * external channels CH0..CH7 map to pads PA20,PA19,PA18,PA17,PA15,PA14,
 * PA13,PA12; this board exposes CH5 on PA14 and CH6 on PA13.  Any external
 * channel can be listed here; internal channels would carry
 * AMEBA_ADC_PIN_NC.
 */

static const uint8_t g_adc_channels[] =
{
  5,                 /* ADC_CH5 */
  6,                 /* ADC_CH6 */
};

static const uint8_t g_adc_pins[] =
{
  AMEBA_PA(14),      /* CH5 -> PA14 */
  AMEBA_PA(13),      /* CH6 -> PA13 */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rtl8721f_adc_initialize
 *
 * Description:
 *   Register the board's ADC channels at /dev/adc0.
 *
 ****************************************************************************/

int rtl8721f_adc_initialize(void)
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
