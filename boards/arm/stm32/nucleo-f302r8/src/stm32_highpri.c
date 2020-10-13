/****************************************************************************
 * boards/arm/stm32/nucleo-f302r8/src/stm32_highpri.c
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

#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/signal.h>
#include <nuttx/timers/pwm.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>

#include <arch/irq.h>
#include <arch/armv7-m/nvicpri.h>

#include "arm_internal.h"
#include "ram_vectors.h"

#include "stm32_pwm.h"
#include "stm32_adc.h"
#include "stm32_dma.h"

#include <arch/board/board.h>

#ifdef CONFIG_NUCLEOF302R8_HIGHPRI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

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
#  if defined(CONFIG_STM32_TIM1_PWM)
#    define HIGHPRI_HAVE_TIM1
#  endif
#  if (CONFIG_STM32_ADC1_DMA_CFG != 1)
#    error ADC1 DMA must be configured in Circular Mode
#  endif
#  if !defined(HIGHPRI_HAVE_TIM1)
#    error "Needs TIM1 to trigger ADC DMA"
#  endif
#endif

#if (CONFIG_STM32_ADC1_INJECTED_CHAN > 0)
#  if (CONFIG_STM32_ADC1_INJECTED_CHAN > 2)
#    error Max 2 injected channels supported for now
#  else
#    define HIGHPRI_HAVE_INJECTED
#  endif
#endif

#ifdef HIGHPRI_HAVE_INJECTED
#  define INJ_NCHANNELS CONFIG_STM32_ADC1_INJECTED_CHAN
#else
#  define INJ_NCHANNELS (0)
#endif

#ifndef CONFIG_STM32_ADC1_DMA
#  define REG_NCHANNELS (1)
#else
#  define REG_NCHANNELS (3)
#endif

#define ADC1_NCHANNELS  (REG_NCHANNELS + INJ_NCHANNELS)

#define DEV1_PORT       (1)
#define DEV1_NCHANNELS  ADC1_NCHANNELS
#define ADC_REF_VOLTAGE (3.3f)
#define ADC_VAL_MAX     (4095)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* High priority example private data */

struct highpri_s
{
  FAR struct stm32_adc_dev_s *adc1;
#ifdef HIGHPRI_HAVE_TIM1
  struct stm32_pwm_dev_s     *pwm;
#endif
  volatile uint32_t  cntr1;
  volatile uint32_t  cntr2;
  volatile uint8_t   current;
  uint16_t           r_val[REG_NCHANNELS];
  float              r_volt[REG_NCHANNELS];
#ifdef HIGHPRI_HAVE_INJECTED
  uint16_t           j_val[INJ_NCHANNELS];
  float              j_volt[INJ_NCHANNELS];
#endif
  bool               lock;
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
  11,
#endif
#if INJ_NCHANNELS > 0
  7,
#endif
#if INJ_NCHANNELS > 1
  6
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
#if INJ_NCHANNELS > 0
  GPIO_ADC1_IN7,                /* PC1/A4 */
#endif
#if INJ_NCHANNELS > 1
  GPIO_ADC1_IN6                 /* PC0/A5 */
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

#if !defined(CONFIG_STM32_ADC1_DMA) || defined(HIGHPRI_HAVE_INJECTED)
void adc12_handler(void)
{
  FAR struct stm32_adc_dev_s *adc = g_highpri.adc1;
  float ref = ADC_REF_VOLTAGE;
  float bit = ADC_VAL_MAX;
  uint32_t pending;
#ifdef HIGHPRI_HAVE_INJECTED
  int i = 0;
#endif

  /* Get pending ADC interrupts */

  pending = STM32_ADC_INT_GET(adc);

  if (g_highpri.lock == true)
    {
      goto irq_out;
    }

#ifndef CONFIG_STM32_ADC1_DMA
  /* Regular channel end of conversion */

  if (pending & ADC_ISR_EOC)
    {
      /* Increase regular sequence counter */

      g_highpri.cntr1 += 1;

      /* Get regular data */

      g_highpri.r_val[g_highpri.current] = STM32_ADC_REGDATA_GET(adc);

      /* Do some floating point operations */

      g_highpri.r_volt[g_highpri.current] =
        (float)g_highpri.r_val[g_highpri.current] * ref / bit;

      if (g_highpri.current >= REG_NCHANNELS - 1)
        {
          g_highpri.current = 0;
        }
      else
        {
          g_highpri.current += 1;
        }
    }
#endif

#ifdef HIGHPRI_HAVE_INJECTED
  /* Injected channel end of sequence */

  if (pending & ADC_ISR_JEOS)
    {
      /* Increase injected sequence counter */

      g_highpri.cntr2 += 1;

      /* Get injected channels */

      for (i = 0; i < INJ_NCHANNELS; i += 1)
        {
          g_highpri.j_val[i] = STM32_ADC_INJDATA_GET(adc, i);
        }

      /* Do some floating point operations */

      for (i = 0; i < INJ_NCHANNELS; i += 1)
        {
          g_highpri.j_volt[i] = (float)g_highpri.j_val[i] * ref / bit;
        }
    }
#endif

irq_out:

  /* Clear ADC pending interrupts */

  STM32_ADC_INT_ACK(adc, pending);
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

  if (g_highpri.lock == true)
    {
      goto irq_out;
    }

  /* Increase regular sequence counter */

  g_highpri.cntr1 += 1;

  for (i = 0; i < REG_NCHANNELS; i += 1)
    {
      /* Do some floating point operations */

      g_highpri.r_volt[i] = (float)g_highpri.r_val[i] * ref / bit;
    }

irq_out:

  /* Clear DMA pending interrupts */

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
#ifdef HIGHPRI_HAVE_TIM1
  struct stm32_pwm_dev_s *pwm1;
#endif
  FAR struct adc_dev_s *adc1;
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

  adc1 = stm32_adcinitialize(DEV1_PORT, g_chanlist1, DEV1_NCHANNELS);
  if (adc1 == NULL)
    {
      aerr("ERROR: Failed to get ADC interface 1\n");
      ret = EXIT_FAILURE;
      goto errout;
    }

  highpri->adc1 = (struct stm32_adc_dev_s *)adc1->ad_priv;

#ifdef HIGHPRI_HAVE_TIM1
  /* Initialize TIM1 */

  pwm1 = (FAR struct stm32_pwm_dev_s *) stm32_pwminitialize(1);
  if (pwm1 == NULL)
    {
      printf("ERROR: Failed to get PWM1 interface\n");
      ret = EXIT_FAILURE;
      goto errout;
    }

  highpri->pwm = pwm1;

  /* Setup PWM device */

  PWM_SETUP(pwm1);

  /* Set timer frequency */

  PWM_FREQ_UPDATE(pwm1, 1000);

  /* Set CCR1 */

  PWM_CCR_UPDATE(pwm1, 1, 0x0f00);

  /* Enable TIM1 OUT1 */

  PWM_OUTPUTS_ENABLE(pwm1, STM32_PWM_OUT1, true);

#ifdef CONFIG_DEBUG_PWM_INFO
  /* Print debug */

  PWM_DUMP_REGS(pwm1);
#endif

#endif /* HIGHPRI_HAVE_TIM1 */

#if !defined(CONFIG_STM32_ADC1_DMA) || defined(HIGHPRI_HAVE_INJECTED)
  /* Attach ADC12 ram vector if no DMA or injected channels support */

  ret = arm_ramvec_attach(STM32_IRQ_ADC12, adc12_handler);
  if (ret < 0)
    {
      fprintf(stderr, "highpri_main: ERROR: arm_ramvec_attach failed: %d\n",
              ret);
      ret = EXIT_FAILURE;
      goto errout;
    }

  /* Set the priority of the ADC12 interrupt vector */

  ret = up_prioritize_irq(STM32_IRQ_ADC12, NVIC_SYSH_HIGH_PRIORITY);
  if (ret < 0)
    {
      fprintf(stderr, "highpri_main: ERROR: up_prioritize_irq failed: %d\n",
              ret);
      ret = EXIT_FAILURE;
      goto errout;
    }

  up_enable_irq(STM32_IRQ_ADC12);
#endif

#ifdef CONFIG_STM32_ADC1_DMA
  /* Attach DMA1 CH1 ram vector if DMA */

  ret = arm_ramvec_attach(STM32_IRQ_DMA1CH1, dma1ch1_handler);
  if (ret < 0)
    {
      fprintf(stderr, "highpri_main: ERROR: arm_ramvec_attach failed: %d\n",
              ret);
      ret = EXIT_FAILURE;
      goto errout;
    }

  /* Set the priority of the DMA1CH1 interrupt vector */

  ret = up_prioritize_irq(STM32_IRQ_DMA1CH1, NVIC_SYSH_HIGH_PRIORITY);
  if (ret < 0)
    {
      fprintf(stderr, "highpri_main: ERROR: up_prioritize_irq failed: %d\n",
              ret);
      ret = EXIT_FAILURE;
      goto errout;
    }

  up_enable_irq(STM32_IRQ_DMA1CH1);
#endif

  /* Setup ADC hardware */

  adc1->ad_ops->ao_setup(adc1);

  /* Configure regular channels trigger to T1CC1 */

  STM32_ADC_EXTCFG_SET(highpri->adc1,
                       ADC1_EXTSEL_T1CC1 | ADC_EXTREG_EXTEN_DEFAULT);

#ifndef CONFIG_STM32_ADC1_DMA
  /* Enable ADC regular conversion interrupts if no DMA */

  STM32_ADC_INT_ENABLE(highpri->adc1, ADC_IER_EOC);
#else
  /* Register ADC buffer for DMA transfer */

  STM32_ADC_REGBUF_REGISTER(highpri->adc1, g_highpri.r_val, REG_NCHANNELS);
#endif

#ifdef HIGHPRI_HAVE_INJECTED
  /* Enable ADC injected sequence end interrupts */

  STM32_ADC_INT_ENABLE(highpri->adc1, ADC_IER_JEOS);
#endif

#ifdef HIGHPRI_HAVE_TIM1
  /* Enable timer counter after ADC configuration */

  PWM_TIM_ENABLE(pwm1, true);
#endif

  while (1)
    {
#ifndef CONFIG_STM32_ADC1_DMA
      /* Software trigger for regular sequence */

      adc1->ad_ops->ao_ioctl(adc1, IO_TRIGGER_REG, 0);

      usleep(100);
#endif

#ifdef HIGHPRI_HAVE_INJECTED
      /* Software trigger for injected sequence */

      adc1->ad_ops->ao_ioctl(adc1, IO_TRIGGER_INJ, 0);

      usleep(100);
#endif
      /* Lock global data */

      g_highpri.lock = true;

#ifndef CONFIG_STM32_ADC1_DMA
      printf("%d [%d] %0.3fV\n", g_highpri.cntr1, g_highpri.current,
              g_highpri.r_volt[g_highpri.current]);
#else
      printf("%d ", g_highpri.cntr1);

      for (i = 0; i < REG_NCHANNELS; i += 1)
        {
          printf("r:[%d] %0.3fV, ", i, g_highpri.r_volt[i]);
        }

      printf("\n");
#endif

#ifdef HIGHPRI_HAVE_INJECTED
      /* Print data from injected channels */

      printf("%d ", g_highpri.cntr2);

      for (i = 0; i < INJ_NCHANNELS; i += 1)
        {
          printf("j:[%d] %0.3fV, ", i, g_highpri.j_volt[i]);
        }

      printf("\n");
#endif
      /* Unlock global data */

      g_highpri.lock = false;

      nxsig_sleep(1);
    }

errout:
  return ret;
}

#endif /* CONFIG_NUCLEOF302R8_HIGHPRI */
