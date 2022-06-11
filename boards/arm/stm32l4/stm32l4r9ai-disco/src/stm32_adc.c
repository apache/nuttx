/****************************************************************************
 * boards/arm/stm32l4/stm32l4r9ai-disco/src/stm32_adc.c
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
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include <nuttx/random.h>
#include <nuttx/board.h>
#include <nuttx/fs/fs.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>

#include "stm32l4_gpio.h"
#include "stm32l4_adc.h"
#include "stm32l4r9ai-disco.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* STM32 chip specific calibration values ***********************************/

/* Voltage used for calibration of internal analog reference voltage */

#if defined(CONFIG_ARCH_CHIP_STM32F7) \
 || defined(CONFIG_ARCH_CHIP_STM32F4) \
 || defined(CONFIG_ARCH_CHIP_STM32F0)
#  define STM32_VREFINT_MVOLTS               ((uint32_t)3300)
#elif defined(CONFIG_ARCH_CHIP_STM32L4) || defined(CONFIG_STM32L15XXX)
#  define STM32_VREFINT_MVOLTS               ((uint32_t)3000)
#endif

/* Internal reference voltage calibration value locations. Taken from
 * https://github.com/micropython/micropython/commit/87215a0f0480dd0324a1b9c1
 *
 * F0 value from DM00115237 Rev 4 p. 19. for STM32F091xB and STM32F091xC
 *
 * Note: These seems to not vary between MCUs of a given family, but
 * should nevertheless be verified from data-sheet when porting this
 * file to a new chip.
 */

#if defined(CONFIG_ARCH_CHIP_STM32F7)
#  define STM32_VREFINT_CAL (*(uint16_t *)((uint32_t)0x1ff0f44a))
#elif defined(CONFIG_ARCH_CHIP_STM32F4)
#  define STM32_VREFINT_CAL (*(uint16_t *)((uint32_t)0x1fff7a2a))
#elif defined(CONFIG_ARCH_CHIP_STM32F0)
#  define STM32_VREFINT_CAL (*(uint16_t *)((uint32_t)0x1ffff7ba))
#elif defined(CONFIG_ARCH_CHIP_STM32L4)
#  define STM32_VREFINT_CAL (*(uint16_t *)((uint32_t)0x1fff75aa))
#elif defined(CONFIG_STM32L15XXX)
#  define STM32_VREFINT_CAL (*(uint16_t *)((uint32_t)0x1ff800f8))
#endif

/* Internal temperature sensor calibration locations. Taken from
 * STM32L4R9AI datasheet.
 */

#if defined(CONFIG_ARCH_CHIP_STM32L4)

/* TS ADC raw data acquired at a temperature of 30 °C (± 5 °C) */

#  define STM32_TSENSE_TSCAL1 (*(int16_t *)((uint32_t)0x1fff75a8))

/* TS ADC raw data acquired at a temperature of 130 °C (± 5 °C) */

#  define STM32_TSENSE_TSCAL2 (*(int16_t *)((uint32_t)0x1fff75ca))
#endif

/* Configuration ************************************************************/

/* These are internal to STM32L4 */

#define ADC1_INTERNAL_VREFINT_CHANNEL       0
#define ADC1_INTERNAL_TSENSE_CHANNEL        17
#define ADC1_INTERNAL_VBATDIV3_CHANNEL      18

/* Application specific channel defined in board.h */

/* The number of ADC channels in the conversion list */

#define ADC1_NCHANNELS 4

#if ADC1_NCHANNELS > 1 && !defined(CONFIG_STM32L4_ADC1_DMA)
#  warning "Reading multiple channels without DMA might cause overruns!"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct adc_dev_s *g_adc;

static const uint8_t g_chanlist[ADC1_NCHANNELS] =
{
  ADC1_INTERNAL_VREFINT_CHANNEL,
  ADC1_INTERNAL_TSENSE_CHANNEL,
  ADC1_INTERNAL_VBATDIV3_CHANNEL,
  ADC1_MEASURE_CHANNEL,
};

/* Configurations of pins used by each ADC channel */

static const uint32_t g_pinlist[ADC1_NCHANNELS] =
{
  0xffffffffu,
  0xffffffffu,
  0xffffffffu,
  GPIO_MEASURE_ADC,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l4_adc_measure_voltages
 *
 * Description:
 *   Read internal reference voltage, internal VBAT and one external voltage.
 *
 ****************************************************************************/

int stm32l4_adc_measure_voltages(uint32_t *vrefint, uint32_t *vbat,
                                 uint32_t *vext)
{
  struct adc_msg_s sample[ADC1_NCHANNELS];
  struct file filestruct;
  ssize_t nbytes;
  int nsamples;
  int ret;

  ret = file_open(&filestruct, "/dev/adc0", O_RDONLY);
  if (ret < 0)
    {
      aerr("ERROR: Cannot open ADC converter\n");
      goto out;
    }

  ret = file_ioctl(&filestruct, ANIOC_TRIGGER, 0);
  if (ret < 0)
    {
      aerr("ERROR: Cannot trigger ADC conversion\n");
      goto out_close;
    }

  nbytes = file_read(&filestruct, sample, sizeof(sample));
  if (nbytes < 0)
    {
      if (nbytes != -EINTR)
        {
          aerr("ERROR: file_read() failed: %d\n", nbytes);
          ret = (int)nbytes;
          goto out_close;
        }

      ainfo("Interrupted read...\n");
      goto out_close;
    }
  else if (nbytes == 0)
    {
      aerr("ERROR: No data read\n");
      ret = ERROR;
      goto out_close;
    }

  nsamples = nbytes / sizeof(struct adc_msg_s);
  if (nsamples * sizeof(struct adc_msg_s) != nbytes)
    {
      ainfo("Read size=%ld is not a multiple of sample size=%d, Ignoring\n",
            (long)nbytes, sizeof(struct adc_msg_s));
    }
  else
    {
      int i;
      int32_t tsense;
      ainfo("Sample (nsamples = %d):\n", nsamples);

      *vrefint = *vbat = *vext = 0;

      for (i = 0; i < nsamples ; i++)
        {
          ainfo("%d: channel: %d value: %d\n",
                i + 1, sample[i].am_channel, sample[i].am_data);

          /* Add the raw value to entropy pool. */

          add_sensor_randomness(sample[i].am_data);

          switch (sample[i].am_channel)
            {
              case ADC1_INTERNAL_VREFINT_CHANNEL:

                /* Calculate corrected Vrefint with factory value. */

                *vrefint = STM32_VREFINT_MVOLTS * STM32_VREFINT_CAL /
                           sample[i].am_data;
                ainfo("VREFINT: %d -> %u mV\n", sample[i].am_data, *vrefint);
                break;

              case ADC1_INTERNAL_TSENSE_CHANNEL:
                /* Calculate final temperature. Sensor precision is ±2 °C,
                 * so it does not matter much if we use integer type here.
                 */

                tsense = 30 + (110 - 30) *
                         (sample[i].am_data - STM32_TSENSE_TSCAL1) /
                         (STM32_TSENSE_TSCAL2 - STM32_TSENSE_TSCAL1);
                ainfo("TSENSE: %d -> %d °C\n", sample[i].am_data, tsense);
                UNUSED(tsense);
                break;

              case ADC1_INTERNAL_VBATDIV3_CHANNEL:
                *vbat = 3 * sample[i].am_data;
                ainfo("VBAT/3: %d -> %u mV\n", sample[i].am_data, *vbat);
                break;

              case ADC1_MEASURE_CHANNEL:
                *vext = sample[i].am_data;
                ainfo("External channel: %d\n", *vext);
                break;

              default:
                aerr("ERROR: ADC got value from unknown channel %d\n",
                     sample[i].am_channel);
                break;
            }
        }
    }

out_close:
  file_close(&filestruct);
out:
  return ret;
}

/****************************************************************************
 * Name: stm32l4_adc_setup
 ****************************************************************************/

int stm32l4_adc_setup(void)
{
  static bool initialized = false;

  if (!initialized)
    {
#ifdef CONFIG_STM32L4_ADC1
      int ret;
      int i;

      /* Configure the pins as analog inputs for the selected channels */

      for (i = 0; i < ADC1_NCHANNELS; i++)
        {
          if (g_pinlist[i] != 0xffffffffu)
            {
              stm32l4_configgpio(g_pinlist[i]);
            }
        }

      /* Call stm32l4_adc_initialize() to get an instance of the ADC */

      g_adc = stm32l4_adc_initialize(1, g_chanlist, ADC1_NCHANNELS);
      if (g_adc == NULL)
        {
          aerr("ERROR: Failed to get ADC interface\n");
          return -ENODEV;
        }

      /* Register the ADC driver at "/dev/adc0" */

      ret = adc_register("/dev/adc0", g_adc);
      if (ret < 0)
        {
          aerr("ERROR: adc_register failed: %d\n", ret);
          return ret;
        }
#endif

      initialized = true;
    }

  return OK;
}
