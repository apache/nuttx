/****************************************************************************
 * boards/arm/imxrt/imxrt1060-evk/src/imxrt_adc.c
 *
 *   Copyright (C) 2020 Actia Nordic AB. All rights reserved.
 *   Author: Thomas Axelsson <thomas.axelsson@actia.se>
 *
 * Based on boards/arm/lpc17xx_40xx/mbed/src/lpc17_40_adc.c
 *
 * Based on boards/zkit-arm-176/src/up-adc
 *
 *   Copyright (C) 2013 Zilogic Systems. All rights reserved.
 *   Author: Kannan <code@nuttx.org>
 *
 * Based on boards/lpc1720g-eval/src/lpc17_40_adc.c
 *
 *   Copyright (C) 2012, 2014, 2016 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/analog/adc.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_arch.h"

#include "imxrt_adc.h"

#ifdef CONFIG_IMXRT_ADC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define ADC1_NCHANNELS 3
#define ADC2_NCHANNELS 3

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t g_chanlist1[ADC1_NCHANNELS] =
  {
    /* Arduino Interface pins A0 to A2 */

    15,
    0,
    9,
  };

static const uint8_t g_chanlist2[ADC2_NCHANNELS] =
  {
    /* Arduino Interface pins A3 to A5 */

    10,
    6,
    5
  };

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

int imxrt_adc_initialize(void)
{
  static bool initialized = false;
  struct adc_dev_s *adc;
  int ret;

  /* Check if we have already initialized */

  if (!initialized)
    {
      /* Call imxrt_adcinitialize() to get an instance of the ADC interface */

#ifdef CONFIG_IMXRT_ADC1
      adc = imxrt_adcinitialize(1, g_chanlist1, ADC1_NCHANNELS);
      if (adc == NULL)
        {
          aerr("ERROR: Failed to get ADC interface for ADC1\n");
          return -ENODEV;
        }

      /* Register the ADC driver at "/dev/adc1" */

      ret = adc_register("/dev/adc1", adc);
      if (ret < 0)
        {
          aerr("ERROR: adc_register adc1 failed: %d\n", ret);
          return ret;
        }
#endif

#ifdef CONFIG_IMXRT_ADC2
      adc = imxrt_adcinitialize(2, g_chanlist2, ADC2_NCHANNELS);
      if (adc == NULL)
        {
          aerr("ERROR: Failed to get ADC interface for ADC2\n");
          return -ENODEV;
        }

      /* Register the ADC driver at "/dev/adc2" */

      ret = adc_register("/dev/adc2", adc);
      if (ret < 0)
        {
          aerr("ERROR: adc_register adc2 failed: %d\n", ret);
          return ret;
        }
#endif

      /* Now we are initialized */

      initialized = true;
    }

  return OK;
}

#endif /* CONFIG_ADC */
