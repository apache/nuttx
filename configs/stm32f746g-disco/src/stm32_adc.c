/****************************************************************************
 * configs/stm32f334-disco/src/stm32_adc.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Ivan Ucherdzhiev <ivanucherdjiev@gmail.com>
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

#include "stm32_gpio.h"
#include "stm32_adc.h"

#ifndef CONFIG_STM32F7_ADC3
#  error "Only ADC3 channels are availiable on the arduino header of the board"
#endif

#if defined(CONFIG_ADC) && defined(CONFIG_STM32F7_ADC3)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* If DMA support is not enabled, then only a single channel
 * can be sampled.  Otherwise, data overruns would occur.
 */

#ifdef ADC_HAVE_DMA
# define ADC3_NCHANNELS 6
#else
# define ADC3_NCHANNELS 1
#endif

/* The number of ADC channels in the conversion list */
/* TODO DMA */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Identifying number of each ADC channel (even if NCHANNELS is less ) */

static const uint8_t g_chanlist[6] =
{
  0,
  4,
  5,
  6,
  7,
  8
};

/* Configurations of pins used by each ADC channel */

static const uint32_t g_pinlist[6]  =
{
  GPIO_ADC3_IN0,                /* PA0/A0 */
  GPIO_ADC3_IN4,                /* PF/A1 */
  GPIO_ADC3_IN5,                /* PF7/A3 */
  GPIO_ADC3_IN6,                /* PF8/A0 */
  GPIO_ADC3_IN7,                /* PF9/A1 */
  GPIO_ADC3_IN8                 /* PF10/A3 */
};

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
 #ifdef CONFIG_STM32F7_ADC3
   static bool initialized = false;
   struct adc_dev_s *adc;
   int ret;
   int i;

   /* Check if we have already initialized */

   if (!initialized)
     {
       /* Configure the pins as analog inputs for the selected channels */

       for (i = 0; i < ADC3_NCHANNELS; i++)
         {
           stm32_configgpio(g_pinlist[i]);
         }

       /* Call stm32_adcinitialize() to get an instance of the ADC interface */

       adc = stm32_adc_initialize(3, g_chanlist, ADC3_NCHANNELS);
       if (adc == NULL)
         {
           aerr("ERROR: Failed to get ADC interface\n");
           return -ENODEV;
         }

       /* Register the ADC driver at "/dev/adc0" */

       ret = adc_register("/dev/adc3", adc);
       if (ret < 0)
         {
           aerr("ERROR: adc_register failed: %d\n", ret);
           return ret;
         }

       /* Now we are initialized */

       initialized = true;
     }

   return OK;
 #else
   return -ENOSYS;
 #endif
 }

#endif /* (CONFIG_ADC) && (CONFIG_STM32F7_ADC3) */
