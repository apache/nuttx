/****************************************************************************
 * boards/arm/tiva/tm4c123g-launchpad/src/tm4c_adc.c
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

#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/analog/adc.h>

#include "tiva_adc.h"
#include "tiva_timer.h"
#include "tm4c123g-launchpad.h"
#include "hardware/tiva_pinmap.h"

#ifdef CONFIG_TIVA_ADC

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tm4c_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

int tm4c_adc_setup(void)
{
#ifdef CONFIG_ADC
  static bool initialized = false;
  int ret;
  uint8_t srate = 0;

  struct tiva_adc_sse_cfg_s sse_cfg0;
  struct tiva_adc_cfg_s adc_cfg;
  struct tiva_adc_step_cfg_s step_cfg[] =
  {
    {
      0,
      0,
      0,
      0,
      (TIVA_ADC_FLAG_TS |
       TIVA_ADC_FLAG_IE |
       TIVA_ADC_FLAG_END),
      0},
  };

  sse_cfg0.priority = 0;
#ifdef CONFIG_EXAMPLES_ADC_SWTRIG
  sse_cfg0.trigger = TIVA_ADC_TRIG_SW;
#else
  sse_cfg0.trigger = TIVA_ADC_TRIG_ALWAYS;
#endif

  adc_cfg.adc = 0;
  adc_cfg.sse[0] = true;
  adc_cfg.sse[1] = false;
  adc_cfg.sse[2] = false;
  adc_cfg.sse[3] = false;
  adc_cfg.ssecfg[0] = sse_cfg0;
  adc_cfg.steps = 1;
  adc_cfg.stepcfg = step_cfg;

#ifdef CONFIG_EXAMPLES_ADC_SWTRIG
  srate = TIVA_ADC_SAMPLE_RATE_FASTEST;
#else
  srate = TIVA_ADC_SAMPLE_RATE_SLOWEST;
#endif

  /* Check if we have already initialized */

  if (!initialized)
    {
      /* Call tiva_adc_initialize to configure an instance of the ADC
       * interface and register it to the character level driver.
       */

      ret = tiva_adc_initialize(CONFIG_EXAMPLES_ADC_DEVPATH, &adc_cfg,
                                TIVA_ADC_CLOCK_MAX, srate);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register /dev/adc0"
                          " to character driver: %d\n", ret);
          return ret;
        }

      /* Now we are initialized */

      initialized = true;
    }
#endif /* CONFIG_ADC */

  return OK;
}

/****************************************************************************
 * Name: adc_timer_init
 *
 * Description:
 *   Initialize timer specifically for ADC use.
 *
 ****************************************************************************/

/* Tiva timer interface does not currently support user configuration */

#if 0 /* defined(CONFIG_TIVA_TIMER) */
TIMER_HANDLE adc_timer_init(void)
{
  struct tiva_gptm32config_s adctimer =
  {
    .cmn =
    {
      .gptm = 0;
      .mode = TIMER32_MODE_PERIODIC;
      .alternate = false;
    };
    .config =
    {
      .flags = TIMER_FLAG_ADCTIMEOUT;
      .handler = NULL;
      .u.periodic = TIVA_TIME_MS_TO_TICKS(100); /* in clock ticks */
    };
  };

  return tiva_gptm_configure((const struct tiva_gptmconfig_s *)&adctimer);
}
#endif /* CONFIG_TIVA_TIMER */

#endif /* CONFIG_TIVA_ADC */
