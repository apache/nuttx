/*****************************************************************************
 * boards/arm/stm32l4/nucleo-l452re/src/stm32_adc.c
 *
 *   Copyright (C) 2017 Haltian Ltd. All rights reserved.
 *   Authors: Juha Niskanen <juha.niskanen@haltian.com>
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
#include "nucleo-l452re.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* STM32 chip specific calibration values ***********************************/

/* Voltage used for calibration of internal analog reference voltage (Vrefint) */

#if defined(CONFIG_ARCH_CHIP_STM32F7) \
 || defined(CONFIG_ARCH_CHIP_STM32F4) \
 || defined(CONFIG_ARCH_CHIP_STM32F0)
#  define STM32_VREFINT_MVOLTS               ((uint32_t)3300)
#elif defined(CONFIG_ARCH_CHIP_STM32L4) || defined(CONFIG_STM32L15XXX)
#  define STM32_VREFINT_MVOLTS               ((uint32_t)3000)
#endif

/* Internal reference voltage calibration value locations. Taken from
 * https://github.com/micropython/micropython/commit/87215a0f0480dd0324a1b9c1d3fc3a5c2806249d
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
 * STM32L452 datasheet.
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
  0xffffffffU,
  0xffffffffU,
  0xffffffffU,
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

int stm32l4_adc_measure_voltages(uint32_t *vrefint, uint32_t *vbat, uint32_t *vext)
{
  FAR struct file filestruct;
  ssize_t nbytes;
  struct adc_msg_s sample[ADC1_NCHANNELS] = { 0 };
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
          aerr("ERROR: nx_read() failed: %d\n", nbytes);
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
                i+1, sample[i].am_channel, sample[i].am_data);

          /* Add the raw value to entropy pool. */

          add_sensor_randomness(sample[i].am_data);

          switch (sample[i].am_channel)
            {
              case ADC1_INTERNAL_VREFINT_CHANNEL:
                /* Calculate corrected Vrefint with factory calibration value. */

                *vrefint = STM32_VREFINT_MVOLTS * STM32_VREFINT_CAL / sample[i].am_data;
                ainfo("VREFINT: %d -> %u mV\n", sample[i].am_data, *vrefint);
                break;

              case ADC1_INTERNAL_TSENSE_CHANNEL:
                /* Calculate final temperature. Sensor precision is ±2 °C,
                 * so it does not matter much if we use integer type here.
                 */

                tsense = (110 - 30) * (sample[i].am_data - STM32_TSENSE_TSCAL1)
                                    / (STM32_TSENSE_TSCAL2 - STM32_TSENSE_TSCAL1) + 30;
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
      int ret, i;

      /* Configure the pins as analog inputs for the selected channels */

      for (i = 0; i < ADC1_NCHANNELS; i++)
        {
          if (g_pinlist[i] != 0xffffffffU)
            {
              stm32l4_configgpio(g_pinlist[i]);
            }
        }

      /* Call stm32l4_adc_initialize() to get an instance of the ADC interface */

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
