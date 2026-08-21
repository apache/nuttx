/****************************************************************************
 * boards/arm/rtl8720f/rtl8720f_evb/src/rtl8720f_adc.c
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
#include "rtl8720f_rtl8720f_evb.h"

#ifdef CONFIG_AMEBA_ADC

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Channels sampled on /dev/adc0 and the analog pad each is wired to.  The
 * external channels CH0..CH5 map to pads PA13..PA18; this board exposes CH0
 * on PA13 and CH1 on PA14.  Any external channel can be listed here;
 * internal channels (CH6..CH8) would carry AMEBA_ADC_PIN_NC.
 */

static const uint8_t g_adc_channels[] =
{
  0,                 /* ADC_CH0 */
  1,                 /* ADC_CH1 */
};

static const uint8_t g_adc_pins[] =
{
  AMEBA_PA(13),      /* CH0 -> PA13 */
  AMEBA_PA(14),      /* CH1 -> PA14 */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rtl8720f_adc_initialize
 *
 * Description:
 *   Register the board's ADC channels at /dev/adc0.
 *
 ****************************************************************************/

int rtl8720f_adc_initialize(void)
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
