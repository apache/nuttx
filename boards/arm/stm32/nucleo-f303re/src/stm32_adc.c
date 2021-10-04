/****************************************************************************
 * boards/arm/stm32/nucleo-f303re/src/stm32_adc.c
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
