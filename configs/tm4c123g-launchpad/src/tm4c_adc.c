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

#include <nuttx/analog/adc.h>

#include "tiva_adc.h"
#include "tiva_timer.h"
#include "tm4c123g-launchpad.h"

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
 *   Initialize and register the ADC driver
 *
 ************************************************************************************/

#ifdef CONFIG_TIVA_ADC
int board_adc_initialize(void)
{
  static bool initialized = false;
  struct adc_dev_s *adc;
  int ret;

  /* Check if we have already initialized */

  if (!initialized)
    {
      /* Call tiva_adcinitialize() to get an instance of the ADC interface */

      adc = tiva_adc_initialize(0);
      if (adc == NULL)
        {
          adbg("ERROR: Failed to get ADC interface\n");
          return -ENODEV;
        }

      /* Register the ADC driver at "/dev/adc0" */

      ret = adc_register("/dev/adc0", adc);
      if (ret < 0)
        {
          adbg("adc_register failed: %d\n", ret);
          return ret;
        }

      /* Enable ADC interrupts */

      adc->ad_ops->ao_rxint(adc, true);

      /* Now we are initialized */

      initialized = true;
    }

  return OK;
}
#endif /* CONFIG_ADC */

/************************************************************************************
 * Name: adc_devinit
 *
 * Description:
 *   All Tiva architectures must provide the following interface to work with
 *   examples/adc.
 *
 ************************************************************************************/

#ifdef CONFIG_EXAMPLES_ADC
int adc_devinit(void)
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

#  if 0
/************************************************************************************
 * Name: adc_timer_init
 *
 * Description:
 *   Inititalize timer specifically for ADC use.
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
#  endif
#endif /* defined (CONFIG_TIVA_ADC) && defined (CONFIG_TIVA_TIMER) */
