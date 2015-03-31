/************************************************************************************
 * configs/tm4c123g-launchpad/tm4c_adc.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/analog/adc.h>

#include "tiva_adc.h"
#include "tiva_timer.h"
#include "tm4c123g-launchpad.h"
#include "chip/tiva_pinmap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Private Data
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: board_adc_initialize
 *
 * Description:
 *   Initialize and register the ADC driver.
 *
 ************************************************************************************/

#ifdef CONFIG_TIVA_ADC
int board_adc_initialize(void)
 {
#  if defined (CONFIG_TIVA_ADC) && defined (CONFIG_ADC)
  static bool initialized = false;
  int ret;
  uint8_t srate = 0;

  struct tiva_adc_sse_cfg_s sse_cfg0;
  struct tiva_adc_cfg_s adc_cfg;
  struct tiva_adc_step_cfg_s step_cfg[] =
  {
    {0, 0, 0, 0, (TIVA_ADC_FLAG_TS | TIVA_ADC_FLAG_IE | TIVA_ADC_FLAG_END), 0},
  };

  sse_cfg0.priority = 0;
#    ifdef CONFIG_EXAMPLES_ADC_SWTRIG
  sse_cfg0.trigger = TIVA_ADC_TRIG_SW;
#    else
  sse_cfg0.trigger = TIVA_ADC_TRIG_ALWAYS;
#    endif

  adc_cfg.adc = 0;
  adc_cfg.sse[0] = true;
  adc_cfg.sse[1] = false;
  adc_cfg.sse[2] = false;
  adc_cfg.sse[3] = false;
  adc_cfg.ssecfg[0] = sse_cfg0;
  adc_cfg.steps = 1;
  adc_cfg.stepcfg = step_cfg;

#    ifdef CONFIG_EXAMPLES_ADC_SWTRIG
  srate = TIVA_ADC_SAMPLE_RATE_FASTEST;
#    else
  srate = TIVA_ADC_SAMPLE_RATE_SLOWEST;
#    endif

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
  return OK;
}
#endif /* CONFIG_ADC */

/************************************************************************************
 * Name: board_adc_setup
 *
 * Description:
 *   All Tiva architectures must provide the following interface to work with
 *   examples/adc.
 *
 ************************************************************************************/

#ifdef CONFIG_EXAMPLES_ADC
int board_adc_setup(void)
{
#ifdef CONFIG_TIVA_ADC
  return board_adc_initialize();
#else
  return -ENOSYS;
#endif
}
#endif /* CONFIG_EXAMPLES_ADC */

#if defined (CONFIG_TIVA_ADC) && defined (CONFIG_TIVA_TIMER)

/* Tiva timer interface does not currently support user configuration */

#if 0
/************************************************************************************
 * Name: adc_timer_init
 *
 * Description:
 *   Initialize timer specifically for ADC use.
 *
 ************************************************************************************/

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
      .u.periodic = TIVA_TIME_MS_TO_TICKS(100); // in clock ticks
    };
  };

  return tiva_gptm_configure((const struct tiva_gptmconfig_s *)&adctimer);
}

#endif
#endif /* defined (CONFIG_TIVA_ADC) && defined (CONFIG_TIVA_TIMER) */
