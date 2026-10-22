/****************************************************************************
 * boards/arm/rtl8730e/rtl8730e_evb/src/rtl8730e_adc.c
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
#include "rtl8730e_evb.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Channels sampled on /dev/adc0 and the analog pad each is wired to.  The
 * external channels CH0..CH5 are bound to the fixed pads PA0..PA5 (the chip
 * pin-mux table names them TOUCHn_ADCn).  PA0 and PA1 are not bonded out on
 * the RTL8730EAM/EAH packages, so this board lists CH2..CH5 only; the same
 * four are present on every package.
 *
 * PA3/PA4 are also I2C1 pads and PA4/PA5 are UART1 or PWM pads, so a config
 * registering those too must drop the clashing channel here.  The pads are
 * shared with the cap-touch controller and the DMIC, which this port does
 * not use.
 */

static const uint8_t g_adc_channels[] =
{
  2,                 /* ADC_CH2 */
  3,                 /* ADC_CH3 */
  4,                 /* ADC_CH4 */
  5,                 /* ADC_CH5 */
};

static const uint8_t g_adc_pins[] =
{
  AMEBA_PA(2),       /* CH2 -> PA2 */
  AMEBA_PA(3),       /* CH3 -> PA3 */
  AMEBA_PA(4),       /* CH4 -> PA4 */
  AMEBA_PA(5),       /* CH5 -> PA5 */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rtl8730e_adc_initialize
 *
 * Description:
 *   Register the board's ADC channels at /dev/adc0.
 *
 ****************************************************************************/

int rtl8730e_adc_initialize(void)
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
