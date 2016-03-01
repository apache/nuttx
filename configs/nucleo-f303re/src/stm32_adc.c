/****************************************************************************
 * configs/nucleo-f303re/src/stm32_adc.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2015-2016 Omni Hoverboards Inc. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Paul Alexander Patience <paul-a.patience@polymtl.ca>
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

#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/analog/adc.h>

#include "stm32.h"

#if defined(CONFIG_ADC) && \
    (defined(CONFIG_STM32_ADC1) || defined(CONFIG_STM32_ADC2) || \
     defined(CONFIG_STM32_ADC3) || defined(CONFIG_STM32_ADC4))

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#if  defined(CONFIG_STM32_ADC1) && !defined(CONFIG_STM32_ADC2) && \
    !defined(CONFIG_STM32_ADC3) && !defined(CONFIG_STM32_ADC4)
#  define ADC_PORT 1
#elif  defined(CONFIG_STM32_ADC2) && !defined(CONFIG_STM32_ADC1) && \
      !defined(CONFIG_STM32_ADC3) && !defined(CONFIG_STM32_ADC4)
#  define ADC_PORT 2
#elif  defined(CONFIG_STM32_ADC3) && !defined(CONFIG_STM32_ADC1) && \
      !defined(CONFIG_STM32_ADC2) && !defined(CONFIG_STM32_ADC4)
#  define ADC_PORT 3
#elif  defined(CONFIG_STM32_ADC4) && !defined(CONFIG_STM32_ADC1) && \
      !defined(CONFIG_STM32_ADC2) && !defined(CONFIG_STM32_ADC3)
#  define ADC_PORT 4
#else
#  error "Choose only one of ADC1, ADC2, ADC3, ADC4"
#endif

/* The number of ADC channels in the conversion list */

#if defined(CONFIG_STM32_ADC1) && defined(ADC1_HAVE_DMA)
#  define ADC_NCHANNELS 4
#elif defined(CONFIG_STM32_ADC2) && defined(ADC2_HAVE_DMA)
#  define ADC_NCHANNELS 3
#elif defined(CONFIG_STM32_ADC3) && defined(ADC3_HAVE_DMA)
#  define ADC_NCHANNELS 1
#elif defined(CONFIG_STM32_ADC4) && defined(ADC4_HAVE_DMA)
#  define ADC_NCHANNELS 1
#else
#  define ADC_NCHANNELS 1
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_STM32_ADC1)

/* Identifying number of each ADC channel */

static const uint8_t  g_chanlist[ADC_NCHANNELS] =
{
  1,
#ifdef ADC1_HAVE_DMA
  2,
  6,
  7,
#endif
};

/* Configurations of pins used by each ADC channel */

static const uint32_t g_pinlist[ADC_NCHANNELS]  =
{
  GPIO_ADC1_IN1,
#ifdef ADC1_HAVE_DMA
  GPIO_ADC1_IN2,
  GPIO_ADC1_IN6,
  GPIO_ADC1_IN7,
#endif
};

#elif defined(CONFIG_STM32_ADC2)

/* Identifying number of each ADC channel */

static const uint8_t  g_chanlist[ADC_NCHANNELS] =
{
  1,
#ifdef ADC2_HAVE_DMA
  6,
  7,
#endif
};

/* Configurations of pins used by each ADC channel */

static const uint32_t g_pinlist[ADC_NCHANNELS]  =
{
  GPIO_ADC2_IN1,
#ifdef ADC2_HAVE_DMA
  GPIO_ADC2_IN6,
  GPIO_ADC2_IN7,
#endif
};

#elif defined(CONFIG_STM32_ADC3)

/* Identifying number of each ADC channel */

static const uint8_t  g_chanlist[ADC_NCHANNELS] =
{
  12,
};

/* Configurations of pins used by each ADC channel */

static const uint32_t g_pinlist[ADC_NCHANNELS]  =
{
  GPIO_ADC3_IN12,
};

#elif defined(CONFIG_STM32_ADC4)

/* Identifying number of each ADC channel */

static const uint8_t  g_chanlist[ADC_NCHANNELS] =
{
  3,
};

/* Configurations of pins used by each ADC channel */

static const uint32_t g_pinlist[ADC_NCHANNELS]  =
{
  GPIO_ADC4_IN3,
};

#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_adc_setup
 *
 * Description:
 *   All STM32 architectures must provide the following interface to work
 *   with examples/adc.
 *
 ****************************************************************************/

int board_adc_setup(void)
{
  static bool initialized = false;
  FAR struct adc_dev_s *adc;
  int ret;
  int i;

  /* Check if we have already initialized */

  if (!initialized)
    {
      /* Configure the pins as analog inputs for the selected channels */

      for (i = 0; i < ADC_NCHANNELS; i++)
        {
          stm32_configgpio(g_pinlist[i]);
        }

      /* Call stm32_adcinitialize() to get an instance of the ADC interface */

      adc = stm32_adcinitialize(ADC_PORT, g_chanlist, ADC_NCHANNELS);
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

      /* Now we are initialized */

      initialized = true;
    }

  return OK;
}

#endif /* CONFIG_ADC && (CONFIG_STM32_ADC1 || CONFIG_STM32_ADC2 ||
        *                CONFIG_STM32_ADC3 || CONFIG_STM32_ADC4) */
