/****************************************************************************
 * boards/arm/stm32/nucleo-f303re/src/stm32_adc.c
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

#if (defined(CONFIG_STM32_ADC1) && defined(CONFIG_STM32_ADC2)) || \
    (defined(CONFIG_STM32_ADC3) && defined(CONFIG_STM32_ADC4))
# error "will not work with this combination of ADCs"
#endif

/* 1 or 2 ADC devices (DEV1, DEV2) */

#if defined(CONFIG_STM32_ADC1)
#  define DEV1_PORT 1
#endif

#if defined(CONFIG_STM32_ADC2)
#  if defined(DEV1_PORT)
#    define DEV2_PORT 2
#  else
#    define DEV1_PORT 2
#  endif
#endif

#if defined(CONFIG_STM32_ADC3)
#  if defined(DEV2_PORT)
#    error "Choose maximum two of ADC1, ADC2, ADC3, ADC4"
#  else
#    if defined(DEV1_PORT)
#      define DEV2_PORT 3
#    else
#      define DEV1_PORT 3
#    endif
#  endif
#endif

#if defined(CONFIG_STM32_ADC4)
#  if defined(DEV2_PORT)
#    error "Choose maximum two of ADC1, ADC2, ADC3, ADC4"
#  else
#    if defined(DEV1_PORT)
#      define DEV2_PORT 4
#    else
#      define DEV1_PORT 4
#    endif
#  endif
#endif

/* The number of ADC channels in the conversion list */
/* TODO DMA */

#define ADC1_NCHANNELS 4
#define ADC2_NCHANNELS 3
#define ADC3_NCHANNELS 3
#define ADC4_NCHANNELS 1

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* DEV 1 */

#if DEV1_PORT == 1

#define DEV1_NCHANNELS ADC1_NCHANNELS

/* Identifying number of each ADC channel (even if NCHANNELS is less ) */

static const uint8_t g_chanlist1[4] =
{
  1,
  2,
  6,
  7,
};

/* Configurations of pins used by each ADC channel */

static const uint32_t g_pinlist1[4]  =
{
  GPIO_ADC1_IN1,
  GPIO_ADC1_IN2,
  GPIO_ADC1_IN6,
  GPIO_ADC1_IN7
};

#elif DEV1_PORT == 2

#define DEV1_NCHANNELS ADC2_NCHANNELS

/* Identifying number of each ADC channel */

static const uint8_t g_chanlist1[3] =
{
  1,
  3,
  4
};

/* Configurations of pins used by each ADC channel */

static const uint32_t g_pinlist1[3] =
{
  GPIO_ADC2_IN1,
  GPIO_ADC2_IN3,
  GPIO_ADC2_IN4
};

#elif DEV1_PORT == 3

#define DEV1_NCHANNELS ADC3_NCHANNELS

/* Identifying number of each ADC channel */

static const uint8_t g_chanlist1[3] =
{
  1,
  5,
  12
};

/* Configurations of pins used by each ADC channel */

static const uint32_t g_pinlist1[3] =
{
  GPIO_ADC3_IN1,
  GPIO_ADC3_IN5,
  GPIO_ADC3_IN12
};

#elif DEV1_PORT == 4

#define DEV1_NCHANNELS ADC4_NCHANNELS

/* Identifying number of each ADC channel */

static const uint8_t g_chanlist1[1] =
{
  3
};

/* Configurations of pins used by each ADC channel */

static const uint32_t g_pinlist1[1] =
{
  GPIO_ADC4_IN3
};

#endif

#ifdef DEV2_PORT

/* DEV 2 */

#if DEV2_PORT == 1

#define DEV2_NCHANNELS ADC1_NCHANNELS

/* Identifying number of each ADC channel (even if NCHANNELS is less ) */

static const uint8_t g_chanlist2[4] =
{
  1,
  2,
  6,
  7
};

/* Configurations of pins used by each ADC channel */

static const uint32_t g_pinlist2[4] =
{
  GPIO_ADC1_IN1,
  GPIO_ADC1_IN2,
  GPIO_ADC1_IN6,
  GPIO_ADC1_IN7
};

#elif DEV2_PORT == 2

#define DEV2_NCHANNELS ADC2_NCHANNELS

/* Identifying number of each ADC channel */

static const uint8_t g_chanlist2[3] =
{
  1,
  3,
  4
};

/* Configurations of pins used by each ADC channel */

static const uint32_t g_pinlist2[3] =
{
  GPIO_ADC2_IN1,
  GPIO_ADC2_IN3,
  GPIO_ADC2_IN4
};

#elif DEV2_PORT == 3

#define DEV2_NCHANNELS ADC3_NCHANNELS

/* Identifying number of each ADC channel */

static const uint8_t g_chanlist2[3] =
{
  1,
  5,
  12
};

/* Configurations of pins used by each ADC channel */

static const uint32_t g_pinlist2[3] =
{
  GPIO_ADC3_IN1,
  GPIO_ADC3_IN5,
  GPIO_ADC3_IN12
};

#elif DEV2_PORT == 4

#define DEV2_NCHANNELS ADC4_NCHANNELS

/* Identifying number of each ADC channel */

static const uint8_t g_chanlist2[1] =
{
  3
};

/* Configurations of pins used by each ADC channel */

static const uint32_t g_pinlist2[1] =
{
  GPIO_ADC4_IN3
};

#endif
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

int stm32_adc_setup(void)
{
  FAR struct adc_dev_s *adc;
  int ret;
  int i;

  /* DEV1 */
  /* Configure the pins as analog inputs for the selected channels */

  for (i = 0; i < DEV1_NCHANNELS; i++)
    {
      stm32_configgpio(g_pinlist1[i]);
    }

  /* Call stm32_adcinitialize() to get an instance of the ADC interface */

  adc = stm32_adcinitialize(DEV1_PORT, g_chanlist1, DEV1_NCHANNELS);
  if (adc == NULL)
    {
      aerr("ERROR: Failed to get ADC interface 1\n");
      return -ENODEV;
    }

  /* Register the ADC driver at "/dev/adc0" */

  ret = adc_register("/dev/adc0", adc);
  if (ret < 0)
    {
      aerr("ERROR: adc_register /dev/adc0 failed: %d\n", ret);
      return ret;
    }

#ifdef DEV2_PORT

  /* DEV2 */
  /* Configure the pins as analog inputs for the selected channels */

  for (i = 0; i < DEV2_NCHANNELS; i++)
    {
      stm32_configgpio(g_pinlist2[i]);
    }

  /* Call stm32_adcinitialize() to get an instance of the ADC interface */

  adc = stm32_adcinitialize(DEV2_PORT, g_chanlist2, DEV2_NCHANNELS);
  if (adc == NULL)
    {
      aerr("ERROR: Failed to get ADC interface 2\n");
      return -ENODEV;
    }

  /* Register the ADC driver at "/dev/adc1" */

  ret = adc_register("/dev/adc1", adc);
  if (ret < 0)
    {
      aerr("ERROR: adc_register /dev/adc1 failed: %d\n", ret);
      return ret;
    }
#endif

  return OK;
}

#endif /* CONFIG_ADC && (CONFIG_STM32_ADC1 || CONFIG_STM32_ADC2 ||
        *                CONFIG_STM32_ADC3 || CONFIG_STM32_ADC4) */
