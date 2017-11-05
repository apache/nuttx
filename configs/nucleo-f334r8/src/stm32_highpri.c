/****************************************************************************
 * configs/nucleo-f334r8/src/stm32_highpri.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Mateusz Szafoni <raiden00@railab.me>
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

#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/signal.h>

#include <arch/irq.h>
#include <arch/chip/chip.h>
#include <arch/board/board.h>

#include "up_internal.h"
#include "ram_vectors.h"

#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>

#include "stm32_hrtim.h"
#include "stm32_adc.h"
#include "stm32_dma.h"

#ifdef CONFIG_NUCLEOF334R8_HIGHPRI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_ARCH_CHIP_STM32F334R8
#  warning This only have been verified with CONFIG_ARCH_CHIP_STM32F334R8
#endif

#ifndef CONFIG_ARCH_HIPRI_INTERRUPT
#  error CONFIG_ARCH_HIPRI_INTERRUPT is required
#endif

#ifndef CONFIG_ARCH_RAMVECTORS
#  error CONFIG_ARCH_RAMVECTORS is required
#endif

#ifndef CONFIG_ARCH_IRQPRIO
#  error CONFIG_ARCH_IRQPRIO is required
#endif

#ifndef CONFIG_ARCH_FPU
#  warning Set CONFIG_ARCH_FPU for hardware FPU support
#endif

#ifdef CONFIG_STM32_ADC1_DMA
#  if !defined(CONFIG_STM32_HRTIM1) || !defined(CONFIG_STM32_HRTIM_TIMA)
#    error "Needs HRTIM TIMA to trigger ADC"
#  endif
#endif

#ifndef CONFIG_STM32_ADC1_DMA
#  define ADC1_NCHANNELS  1
#else
#  define ADC1_NCHANNELS  3
#endif

#define DEV1_PORT       1
#define DEV1_NCHANNELS  ADC1_NCHANNELS
#define ADC_REF_VOLTAGE 3.3
#define ADC_VAL_MAX     4095

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* High priority example private data */

struct highpri_s
{
  FAR struct stm32_adc_dev_s *adc;
#ifdef CONFIG_STM32_ADC1_DMA
  FAR struct hrtim_dev_s     *hrtim;
#endif
  volatile uint32_t  cntr;
  volatile uint8_t   current;
  uint16_t           val[DEV1_NCHANNELS];
  float              volt[DEV1_NCHANNELS];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* ADC channel list  */

static const uint8_t g_chanlist1[DEV1_NCHANNELS] =
{
  1,
#ifdef CONFIG_STM32_ADC1_DMA
  2,
  11
#endif
};

/* Configurations of pins used by ADC channel */

static const uint32_t g_pinlist1[DEV1_NCHANNELS] =
{
  GPIO_ADC1_IN1,                /* PA0/A0 */

#ifdef CONFIG_STM32_ADC1_DMA
  GPIO_ADC1_IN2,                /* PA1/A1 */
  GPIO_ADC1_IN11,               /* PB0/A3 */
#endif
};

static struct highpri_s g_highpri;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: adc12_handler
 *
 * Description:
 *   This is the handler for the high speed ADC interrupt.
 *
 ****************************************************************************/

#ifndef CONFIG_STM32_ADC1_DMA
void adc12_handler(void)
{
  FAR struct stm32_adc_dev_s *adc = g_highpri.adc;
  float ref = ADC_REF_VOLTAGE;
  float bit = ADC_VAL_MAX;
  uint32_t pending;

  g_highpri.cntr += 1;

  pending = adc->ops->int_get(adc);

  if (pending & ADC_INT_EOC)
    {
      g_highpri.val[g_highpri.current] = adc->ops->val_get(adc);

      /* Do some floating point operations */

      g_highpri.volt[g_highpri.current] = (float)g_highpri.val[g_highpri.current] * ref / bit;

      if (g_highpri.current >= DEV1_NCHANNELS-1)
        {
          g_highpri.current = 0;
        }
      else
        {
          g_highpri.current += 1;
        }
    }

  adc->ops->int_ack(adc, pending);
}
#endif

/****************************************************************************
 * Name: dmach1_handler
 *
 * Description:
 *   This is the handler for the high speed ADC interrupt using DMA transfer.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_ADC1_DMA
void dma1ch1_handler(void)
{
  float ref = ADC_REF_VOLTAGE;
  float bit = ADC_VAL_MAX;
  uint32_t pending;
  int i;

  pending = stm32_dma_intget(STM32_DMA1_CHAN1);

  g_highpri.cntr += 1;

  for (i = 0; i < 3; i += 1)
    {
      /* Do some floating point operations */

      g_highpri.volt[i] = (float)g_highpri.val[i] * ref / bit;
    }

  stm32_dma_intack(STM32_DMA1_CHAN1, pending);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: highpri_main
 *
 * Description:
 *   Main entry point in into the high priority interrupt test.
 *
 ****************************************************************************/

int highpri_main(int argc, char *argv[])
{
#ifdef CONFIG_STM32_ADC1_DMA
  FAR struct hrtim_dev_s *hrtim;
#endif
  FAR struct adc_dev_s *adc;
  FAR struct highpri_s *highpri;
  int ret;
  int i;

  highpri = &g_highpri;

  /* Initialize highpri structure */

  memset(highpri, 0, sizeof(struct highpri_s));

  printf("\nhighpri_main: Started\n");

  /* Configure the pins as analog inputs for the selected channels */

  for (i = 0; i < DEV1_NCHANNELS; i++)
    {
      stm32_configgpio(g_pinlist1[i]);
    }

  /* Initialize ADC driver */

  adc = stm32_adcinitialize(DEV1_PORT, g_chanlist1, DEV1_NCHANNELS);
  if (adc == NULL)
    {
      aerr("ERROR: Failed to get ADC interface 1\n");
      ret = EXIT_FAILURE;
      goto errout;
    }

  highpri->adc   = (struct stm32_adc_dev_s *)adc->ad_priv;

#ifdef CONFIG_STM32_ADC1_DMA
  /* Configure HRTIM */

  hrtim = stm32_hrtiminitialize();
  if (hrtim == NULL)
    {
      printf("ERROR: Failed to get HRTIM1 interface\n");
      ret = EXIT_FAILURE;
      goto errout;
    }

  highpri->hrtim = hrtim;

  /* Set Timer A Period */

  HRTIM_PER_SET(hrtim, HRTIM_TIMER_TIMA, 0xFFD0);
#endif

#ifndef CONFIG_STM32_ADC1_DMA

  /* Attach ADC12 ram vector if no DMA */

  ret = up_ramvec_attach(STM32_IRQ_ADC12, adc12_handler);
  if (ret < 0)
    {
      fprintf(stderr, "highpri_main: ERROR: up_ramvec_attach failed: %d\n", ret);
      ret = EXIT_FAILURE;
      goto errout;
    }

  /* Set the priority of the ADC12 interrupt vector */

  ret = up_prioritize_irq(STM32_IRQ_ADC12, NVIC_SYSH_HIGH_PRIORITY);
  if (ret < 0)
    {
      fprintf(stderr, "highpri_main: ERROR: up_prioritize_irq failed: %d\n", ret);
      ret = EXIT_FAILURE;
      goto errout;
    }

  up_enable_irq(STM32_IRQ_ADC12);
#else

  /* Attach DMA1 CH1 ram vector if DMA */

  ret = up_ramvec_attach(STM32_IRQ_DMA1CH1, dma1ch1_handler);
  if (ret < 0)
    {
      fprintf(stderr, "highpri_main: ERROR: up_ramvec_attach failed: %d\n", ret);
      ret = EXIT_FAILURE;
      goto errout;
    }

  /* Set the priority of the DMA1CH1 interrupt vector */

  ret = up_prioritize_irq(STM32_IRQ_DMA1CH1, NVIC_SYSH_HIGH_PRIORITY);
  if (ret < 0)
    {
      fprintf(stderr, "highpri_main: ERROR: up_prioritize_irq failed: %d\n", ret);
      ret = EXIT_FAILURE;
      goto errout;
    }

  up_enable_irq(STM32_IRQ_DMA1CH1);
#endif

  /* Setup ADC hardware */

  adc->ad_ops->ao_setup(adc);

#ifndef CONFIG_STM32_ADC1_DMA
  /* Enable ADC interrupts if no DMA */

  highpri->adc->ops->int_en(highpri->adc, ADC_INT_EOC);
#else
  /* Register ADC buffer for DMA transfer */

  highpri->adc->ops->regbuf_reg(highpri->adc, g_highpri.val, 3);
#endif

  while(1)
    {
#ifndef CONFIG_STM32_ADC1_DMA
      /* Software triger */

      adc->ad_ops->ao_ioctl(adc, ANIOC_TRIGGER, 0);

      usleep(100);

      printf("%d [%d] %0.3fV\n", g_highpri.cntr, g_highpri.current,
              g_highpri.volt[g_highpri.current]);
#else
      printf("%d ", g_highpri.cntr);

      for (i = 0; i < DEV1_NCHANNELS; i += 1)
        {
          printf("[%d] %0.3fV, ", i, g_highpri.volt[i]);
        }

      printf("\n");
#endif
      sleep(1);
    }

errout:
  return ret;
}

#endif /* CONFIG_NUCLEOF334R8_HIGHPRI */
