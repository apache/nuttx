/****************************************************************************
 * boards/arm/stm32/nucleo-f334r8/src/stm32_spwm.c
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

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <unistd.h>
#include <math.h>
#include <assert.h>

#include <nuttx/arch.h>
#include <nuttx/signal.h>

#include <arch/irq.h>
#include <arch/chip/chip.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "ram_vectors.h"

#include "stm32_hrtim.h"
#include "stm32_pwm.h"
#include "stm32_tim.h"

#ifdef CONFIG_NUCLEOF334R8_SPWM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Asserions ****************************************************************/

#ifndef CONFIG_ARCH_CHIP_STM32F334R8
#  warning "This only have been verified with CONFIG_ARCH_CHIP_STM32F334R8"
#endif

#ifndef CONFIG_ARCH_HIPRI_INTERRUPT
#  error "CONFIG_ARCH_HIPRI_INTERRUPT is required"
#endif

#ifndef CONFIG_ARCH_RAMVECTORS
#  error "CONFIG_ARCH_RAMVECTORS is required"
#endif

#ifndef CONFIG_ARCH_IRQPRIO
#  error "CONFIG_ARCH_IRQPRIO is required"
#endif

#ifndef CONFIG_ARCH_FPU
#  warning "Set CONFIG_ARCH_FPU for hardware FPU support"
#endif

/* Check the configuration for TIM1 */

#ifdef CONFIG_NUCLEOF334R8_SPWM_USE_TIM1

/* Phase 1 is TIM1 CH1 */

#  if CONFIG_NUCLEOF334R8_SPWM_PHASE_NUM > 0
#    ifndef CONFIG_STM32_TIM1_CH1OUT
#      error
#    endif
#    ifndef CONFIG_STM32_TIM6
#      error
#    endif
#  endif

/* Phase 2 is TIM1 CH2 */

#  if CONFIG_NUCLEOF334R8_SPWM_PHASE_NUM > 1
#    ifndef CONFIG_STM32_TIM1_CH2OUT
#      error
#    endif
#  endif

/* Phase 3 is TIM1 CH3 */

#  if CONFIG_NUCLEOF334R8_SPWM_PHASE_NUM > 2
#    ifndef CONFIG_STM32_TIM1_CH3OUT
#      error
#    endif
#  endif

/* Phase 4 is TIM1 CH4 */

#  if CONFIG_NUCLEOF334R8_SPWM_PHASE_NUM > 3
#    ifndef CONFIG_STM32_TIM1_CH4OUT
#      error
#    endif
#  endif

#  if CONFIG_NUCLEOF334R8_SPWM_PHASE_NUM != PWM_TIM1_NCHANNELS
#    error
#  endif

#endif /* CONFIG_NUCLEOF334R8_SPWM_USE_TIM1 */

/* Check the configuration for HRTIM */

#ifdef CONFIG_NUCLEOF334R8_SPWM_USE_HRTIM1

/* Phase 1 is TIMA */

#  if CONFIG_NUCLEOF334R8_SPWM_PHASE_NUM > 0
#    ifndef CONFIG_STM32_HRTIM_TIMA
#      error
#    endif
#    ifndef CONFIG_STM32_HRTIM_MASTER
#      error
#    endif
#  endif

/* Phase 2 is TIMB */

#  if CONFIG_NUCLEOF334R8_SPWM_PHASE_NUM > 1
#    ifndef CONFIG_STM32_HRTIM_TIMB
#      error
#    endif
#  endif

/* Phase 3 is TIMC */

#  if CONFIG_NUCLEOF334R8_SPWM_PHASE_NUM > 2
#    ifndef CONFIG_STM32_HRTIM_TIMC
#      error
#    endif
#  endif

/* Phase 4 is TIMD */

#  if CONFIG_NUCLEOF334R8_SPWM_PHASE_NUM > 3
#    ifndef CONFIG_STM32_HRTIM_TIMD
#      error
#    endif
#  endif

/* Phase 5 is TIME */

#  if CONFIG_NUCLEOF334R8_SPWM_PHASE_NUM > 4
#    ifndef CONFIG_STM32_HRTIM_TIME
#      error
#    endif
#  endif

#endif /* CONFIG_NUCLEOF334R8_SPWM_USE_HRTIM1 */

/* Configuration ************************************************************/

#ifdef CONFIG_NUCLEOF334R8_SPWM_USE_HRTIM1
#  define PWM_TIMERS_IN_USE CONFIG_NUCLEOF334R8_SPWM_PHASE_NUM
#endif

#ifdef CONFIG_NUCLEOF334R8_SPWM_USE_TIM1
#  define PWM_TIMERS_IN_USE 1
#endif

#define SPWM_PHASE_SHIFT ((360.0f/CONFIG_NUCLEOF334R8_SPWM_PHASE_NUM))

#if defined(CONFIG_NUCLEOF334R8_SPWM_USE_HRTIM1)
#  define pwm_timer_s hrtim_dev_s
#elif defined(CONFIG_NUCLEOF334R8_SPWM_USE_TIM1)
#  define pwm_timer_s stm32_pwm_dev_s
#else
#  error
#endif

#define SAMPLES_NUM CONFIG_NUCLEOF334R8_SPWM_SAMPLES
#define PHASES_NUM CONFIG_NUCLEOF334R8_SPWM_PHASE_NUM

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* SPWM private data */

struct spwm_s
{
  struct pwm_timer_s *pwm;
#ifdef CONFIG_NUCLEOF334R8_SPWM_USE_TIM1
  struct stm32_tim_dev_s *tim;
#endif
  float waveform[SAMPLES_NUM];               /* Waveform samples */
  float phase_step;                          /* Waveform phase step */
  float waveform_freq;                       /* Waveform frequency */
  uint16_t cmp[SAMPLES_NUM];                 /* PWM TIM compare table */
  uint16_t per;                              /* PWM TIM period */
  uint16_t samples;                          /* Modulation waveform samples num */
  uint16_t phase_shift[PHASES_NUM];          /* Phase offset */
  volatile uint16_t sample_now[PHASES_NUM];  /* Current sample number for
                                              * phase */
  uint8_t phases;                            /* Number of PWM phases */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct spwm_s g_spwm =
{
  .waveform_freq = ((float)CONFIG_NUCLEOF334R8_SPWM_FREQ),
  .phases        = PHASES_NUM,
  .samples       = SAMPLES_NUM,
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static float waveform_func(float x);
static int waveform_init(struct spwm_s *spwm, float (*f)(float));
static int spwm_start(struct spwm_s *spwm);
static int spwm_start(struct spwm_s *spwm);
static int spwm_stop(struct spwm_s *spwm);
#ifdef CONFIG_NUCLEOF334R8_SPWM_USE_HRTIM1
static int master_configure(struct spwm_s *spwm);
static int slaves_configure(struct spwm_s *spwm);
static void hrtim_master_handler(void);
static int spwm_hrtim_setup(struct spwm_s *spwm);
static int spwm_hrtim_start(struct spwm_s *spwm);
static int spwm_hrtim_stop(struct spwm_s *spwm);
#endif /* CONFIG_NUCLEOF334R8_SPWM_USE_HRTIM1 */
#ifdef CONFIG_NUCLEOF334R8_SPWM_USE_TIM1
static int spwm_tim1_setup(struct spwm_s *spwm);
static int spwm_tim6_setup(struct spwm_s *spwm);
static int spwm_tim1_start(struct spwm_s *spwm);
static int spwm_tim6_start(struct spwm_s *spwm);
static int spwm_tim1_stop(struct spwm_s *spwm);
static int spwm_tim6_stop(struct spwm_s *spwm);
#endif /* CONFIG_NUCLEOF334R8_SPWM_USE_TIM1 */
static int spwm_setup(struct spwm_s *spwm);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: waveform_func
 *
 * Description:
 *   Modulation function. This function must return values from <0.0, 1.0>!
 *
 ****************************************************************************/

static float waveform_func(float x)
{
  DEBUGASSERT(x >= 0 && x <= 2 * M_PI);

  /* Sine modulation */

  return (sinf(x) + 1.0f) / 2.0f;
}

/****************************************************************************
 * Name: waveform_init
 *
 * Description:
 *   Initialize modulation waveform
 *
 ****************************************************************************/

static int waveform_init(struct spwm_s *spwm, float (*f)(float))
{
  uint16_t i = 0;
  int ret = 0;

  printf("Initialize waveform\n");

  /* Get phase step to achieve one sine waveform period */

  spwm->phase_step = (float)(2 * M_PI / spwm->samples);

  /* Initialize sine and PWM compare tables */

  for (i = 0; i < spwm->samples; i += 1)
    {
      /* We need sine in range from 0 to 1.0 */

      spwm->waveform[i] = f(spwm->phase_step * i);

      DEBUGASSERT(spwm->waveform[i] >= 0.0 && spwm->waveform[i] <= 2 * M_PI);

      spwm->cmp[i] = (uint16_t)(spwm->waveform[i] * spwm->per);
    }

  /* Configure phase shift TODO: this should be configurable */

  for (i = 0; i < spwm->phases; i += 1)
    {
      spwm->phase_shift[i] =
        (spwm->samples / CONFIG_NUCLEOF334R8_SPWM_PHASE_NUM) * i;
    }

  /* Initialize offstes */

  for (i = 0; i < spwm->phases; i += 1)
    {
      spwm->sample_now[i] = spwm->phase_shift[i];
    }

  printf("\tsamples = %d\n", spwm->samples);
  printf("\tper     = %d\n", spwm->per);
  printf("\tphase   = %d\n", spwm->phases);
  for (i = 0; i < spwm->phases; i += 1)
    {
      printf("\tsnow%d   = %d\n", i, spwm->sample_now[i]);
    }

  return ret;
}

/****************************************************************************
 * Name: spwm_start
 *
 * Description:
 *   Start SPWM
 *
 ****************************************************************************/

static int spwm_start(struct spwm_s *spwm)
{
#if defined(CONFIG_NUCLEOF334R8_SPWM_USE_HRTIM1)
  /* Start HRTIM */

  spwm_hrtim_start(spwm);

#elif defined(CONFIG_NUCLEOF334R8_SPWM_USE_TIM1)
  /* Start TIM1 */

  spwm_tim1_start(spwm);

  /* Start TIM6 */

  spwm_tim6_start(spwm);
#else
#  error
#endif

  return OK;
}

/****************************************************************************
 * Name: spwm_stop
 *
 * Description:
 *   Stop SPWM
 *
 ****************************************************************************/

static int spwm_stop(struct spwm_s *spwm)
{
#if defined(CONFIG_NUCLEOF334R8_SPWM_USE_HRTIM1)
  /* Stop HRTIM */

  spwm_hrtim_stop(spwm);

#elif defined(CONFIG_NUCLEOF334R8_SPWM_USE_TIM1)
  /* Stop TIM1 */

  spwm_tim1_stop(spwm);

  /* Stop TIM6 */

  spwm_tim6_stop(spwm);
#else
#  error
#endif

  return OK;
}

#ifdef CONFIG_NUCLEOF334R8_SPWM_USE_HRTIM1

/****************************************************************************
 * Name: master_configure
 ****************************************************************************/

static int master_configure(struct spwm_s *spwm)
{
  struct hrtim_dev_s *hrtim = spwm->pwm;
  uint64_t per = 0;
  uint64_t fclk = 0;
  uint64_t freq = 0;
  uint16_t cmp = 0;
  int ret = 0;

  /* Frequency with which we will change samples. master_freq = samples_num *
   * waveform_freq.
   */

  freq = spwm->samples * spwm->waveform_freq;

  /* Configure Master Timer period */

  fclk = HRTIM_FCLK_GET(hrtim, HRTIM_TIMER_MASTER);
  per = fclk / freq;
  if (per > HRTIM_PER_MAX)
    {
      printf("ERROR: can not achieve master freq=%llu if fclk=%llu\n",
             freq, fclk);
      ret = -EINVAL;
      goto errout;
    }

  HRTIM_PER_SET(hrtim, HRTIM_TIMER_MASTER, (uint16_t)per);

  /* Configure Master Timer interrupt trigger (compare 1) */

  cmp = 1;
  HRTIM_CMP_SET(hrtim, HRTIM_TIMER_MASTER, HRTIM_CMP1, cmp);

errout:
  return ret;
}

/****************************************************************************
 * Name: slaves_configure
 ****************************************************************************/

static int slaves_configure(struct spwm_s *spwm)
{
  struct hrtim_dev_s *hrtim = spwm->pwm;
  uint64_t fclk = 0;
  uint64_t per = 0;
  uint8_t index = 0;
  uint8_t i = 0;
  int ret = 0;

  /* Get timer period value for given frequency */

  fclk = HRTIM_FCLK_GET(hrtim, HRTIM_TIMER_TIMA);
  per = fclk / CONFIG_NUCLEOF334R8_SPWM_PWM_FREQ;
  if (per > HRTIM_PER_MAX)
    {
      printf("ERROR: can not achieve pwm freq=%ju if fclk=%llu\n",
             (uintmax_t)CONFIG_NUCLEOF334R8_SPWM_PWM_FREQ, fclk);
      ret = -EINVAL;
      goto errout;
    }

  spwm->per = (uint16_t)per;

  for (i = 0; i < spwm->phases; i += 1)
    {
      /* Get slave timer index */

      index = (1 << (i + 1));

      /* Prescalers for slave timers must be the same !!! */

      if (fclk != HRTIM_FCLK_GET(hrtim, index))
        {
          printf("ERROR: prescaler for HRTIM TIMER %d doesn't match!\n",
                 index);
          ret = -EINVAL;
          goto errout;
        }

      /* Set timer period value */

      HRTIM_PER_SET(hrtim, index, (uint16_t)per);
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: hrtim_master_handler
 ****************************************************************************/

static void hrtim_master_handler(void)
{
  struct spwm_s *spwm = &g_spwm;
  struct hrtim_dev_s *hrtim = spwm->pwm;
  uint32_t pending = 0;
  uint8_t i = 0;

  pending = HRTIM_IRQ_GET(hrtim, HRTIM_TIMER_MASTER);

  if (pending & HRTIM_IRQ_MCMP1)
    {
      /* Update CMP for slaves */

      for (i = 0; i < spwm->phases; i += 1)
        {
          /* Set new CMP for timers */

          HRTIM_CMP_SET(hrtim, (1 << (i + 1)), HRTIM_CMP1,
                        spwm->cmp[spwm->sample_now[i]]);

          /* Increase sample pointer */

          spwm->sample_now[i] += 1;

          if (spwm->sample_now[i] > spwm->samples)
            {
              spwm->sample_now[i] = 0;
            }
        }

      /* Software update all slaves */

      for (i = 0; i < spwm->phases; i += 1)
        {
          HRTIM_SOFT_UPDATE(hrtim, 1 << (i + 1));
        }
    }

  HRTIM_IRQ_ACK(hrtim, HRTIM_TIMER_MASTER, pending);
}

/****************************************************************************
 * Name: spwm_hrtim_setup
 ****************************************************************************/

static int spwm_hrtim_setup(struct spwm_s *spwm)
{
  struct hrtim_dev_s *pwm = NULL;
  int ret = OK;

  /* Configure HRTIM */

  pwm = stm32_hrtiminitialize();
  if (pwm == NULL)
    {
      printf("ERROR: Failed to get HRTIM1 interface\n");
      ret = -1;
      goto errout;
    }

  spwm->pwm = pwm;

  /* Attach HRTIM Master TImer IRQ */

  ret = arm_ramvec_attach(STM32_IRQ_HRTIMTM, hrtim_master_handler);
  if (ret < 0)
    {
      fprintf(stderr, "spwm_main: ERROR: arm_ramvec_attach failed: %d\n",
              ret);
      ret = -1;
      goto errout;
    }

  /* Set the priority of the HRTIM Master interrupt vector */

  ret = up_prioritize_irq(STM32_IRQ_HRTIMTM, NVIC_SYSH_HIGH_PRIORITY);
  if (ret < 0)
    {
      fprintf(stderr, "spwm_main: ERROR: up_prioritize_irq failed: %d\n",
              ret);
      ret = -1;
      goto errout;
    }

  /* Disable HRTIM Master interrupt */

  up_disable_irq(STM32_IRQ_HRTIMTM);

  /* Configure Master Timer */

  ret = master_configure(spwm);
  if (ret < 0)
    {
      printf("ERROR: failed initialize master timer %d!\n", ret);
      goto errout;
    }

  /* Configure Slave Timers */

  ret = slaves_configure(spwm);
  if (ret < 0)
    {
      printf("ERROR: failed initialize timers %d!\n", ret);
      goto errout;
    }

  spwm_hrtim_stop(spwm);

errout:
  return ret;
}

/****************************************************************************
 * Name: spwm_hrtim_start
 ****************************************************************************/

static int spwm_hrtim_start(struct spwm_s *spwm)
{
  struct hrtim_dev_s *hrtim = spwm->pwm;
  uint8_t timers = 0;
  uint16_t outputs = 0;
  int i = 0;

  /* Enable HRTIM Master interrupt */

  up_enable_irq(STM32_IRQ_HRTIMTM);

  /* Get HRTIM timers (master+slaves) */

  for (i = 0; i < spwm->phases + 1; i += 1)
    {
      timers |= (1 << i);
    }

  /* Get HRTIM outputs */

  for (i = 0; i < spwm->phases; i += 1)
    {
      outputs |= (1 << (i * 2));
    }

  /* Enable HRTIM outpus */

  HRTIM_OUTPUTS_ENABLE(hrtim, outputs, true);

  /* Enable HRTIM timers */

  HRTIM_TIM_ENABLE(hrtim, timers, true);

  return OK;
}

/****************************************************************************
 * Name: spwm_hrtim_stop
 ****************************************************************************/

static int spwm_hrtim_stop(struct spwm_s *spwm)
{
  struct hrtim_dev_s *hrtim = spwm->pwm;
  uint8_t timers = 0;
  uint16_t outputs = 0;
  int i = 0;

  /* Disable HRTIM Master interrupt */

  up_disable_irq(STM32_IRQ_HRTIMTM);

  /* Get HRTIM timers (master+slaves) */

  for (i = 0; i < spwm->phases + 1; i += 1)
    {
      timers |= (1 << i);
    }

  /* Get HRTIM outputs */

  for (i = 0; i < spwm->phases; i += 1)
    {
      outputs |= (1 << (i * 2));
    }

  /* Disable HRTIM outpus */

  HRTIM_OUTPUTS_ENABLE(hrtim, outputs, false);

  /* Disable HRTIM timers */

  HRTIM_TIM_ENABLE(hrtim, timers, false);

  return OK;
}

#endif /* CONFIG_NUCLEOF334R8_SPWM_USE_HRTIM1 */

#ifdef CONFIG_NUCLEOF334R8_SPWM_USE_TIM1

/****************************************************************************
 * Name: tim6_handler
 ****************************************************************************/

static void tim6_handler(void)
{
  struct spwm_s *spwm = &g_spwm;
  struct stm32_pwm_dev_s *pwm = spwm->pwm;
  struct stm32_tim_dev_s *tim = spwm->tim;
  uint8_t i = 0;

  for (i = 0; i < spwm->phases; i += 1)
    {
      /* Set new CMP for timers */

      PWM_CCR_UPDATE(pwm, i + 1, spwm->cmp[spwm->sample_now[i]]);

      /* Increase sample pointer */

      spwm->sample_now[i] += 1;

      if (spwm->sample_now[i] > spwm->samples)
        {
          spwm->sample_now[i] = 0;
        }
    }

  /* TODO: Software update */

  STM32_TIM_ACKINT(tim, ATIM_SR_UIF);
}

/****************************************************************************
 * Name: spwm_tim6_setup
 ****************************************************************************/

static int spwm_tim6_setup(struct spwm_s *spwm)
{
  struct stm32_tim_dev_s *tim = NULL;
  uint64_t freq = 0;
  uint32_t per = 0;
  int ret = OK;

  /* Get TIM6 interface */

  tim = stm32_tim_init(6);
  if (tim == NULL)
    {
      printf("ERROR: Failed to get TIM6 interface\n");
      ret = -1;
      goto errout;
    }

  spwm->tim = tim;

  /* Frequency with which we will change samples.
   *
   * tim6_freq = samples_num * waveform_freq.
   */

  freq = spwm->samples * spwm->waveform_freq;
  per = BOARD_TIM6_FREQUENCY / freq;
  if (per > 0xffff)
    {
      printf("ERROR: can not achieve TIM6 frequency\n");
      ret = -1;
      goto errout;
    }

  /* TODO: TIM_SETFREQ */

  STM32_TIM_SETCLOCK(tim, BOARD_TIM6_FREQUENCY);
  STM32_TIM_SETPERIOD(tim, per);

  /* Attach TIM6 ram vector */

  ret = arm_ramvec_attach(STM32_IRQ_TIM6, tim6_handler);
  if (ret < 0)
    {
      printf("ERROR: arm_ramvec_attach failed: %d\n", ret);
      ret = -1;
      goto errout;
    }

  /* Set the priority of the TIM6 interrupt vector */

  ret = up_prioritize_irq(STM32_IRQ_TIM6, NVIC_SYSH_HIGH_PRIORITY);
  if (ret < 0)
    {
      printf("ERROR: up_prioritize_irq failed: %d\n", ret);
      ret = -1;
      goto errout;
    }

  spwm_tim6_stop(spwm);

errout:
  return ret;
}

/****************************************************************************
 * Name: spwm_tim6_start
 ****************************************************************************/

static int spwm_tim6_start(struct spwm_s *spwm)
{
  struct stm32_tim_dev_s *tim = spwm->tim;

  /* Enable the timer interrupt at the NVIC and at TIM6 */

  up_enable_irq(STM32_IRQ_TIM6);
  STM32_TIM_ENABLEINT(tim, BTIM_DIER_UIE);

  return OK;
}

/****************************************************************************
 * Name: spwm_tim6_stop
 ****************************************************************************/

static int spwm_tim6_stop(struct spwm_s *spwm)
{
  struct stm32_tim_dev_s *tim = spwm->tim;

  /* Disable the timer interrupt at the NVIC and at TIM6 */

  up_disable_irq(STM32_IRQ_TIM6);
  STM32_TIM_DISABLEINT(tim, BTIM_DIER_UIE);

  return OK;
}

/****************************************************************************
 * Name: spwm_tim1_setup
 ****************************************************************************/

static int spwm_tim1_setup(struct spwm_s *spwm)
{
  struct stm32_pwm_dev_s *pwm = NULL;
  int ret = OK;

  /* Get TIM1 PWM interface */

  pwm = (struct stm32_pwm_dev_s *)stm32_pwminitialize(1);
  if (pwm == NULL)
    {
      printf("ERROR: Failed to get TIM1 PWM interface\n");
      ret = -1;
      goto errout;
    }

  spwm->pwm = pwm;

  /* Initial PWM1 setup */

  ret = PWM_SETUP(pwm);
  if (ret < 0)
    {
      printf("ERROR: Failed to get setup TIM1 PWM\n");
      ret = -1;
      goto errout;
    }

  /* Configure TIM1 PWM frequency */

  ret = PWM_FREQ_UPDATE(pwm, CONFIG_NUCLEOF334R8_SPWM_PWM_FREQ);
  if (ret < 0)
    {
      printf("ERROR: Failed to set TIM1 PWM frequency\n");
      ret = -1;
      goto errout;
    }

  /* Get TIM1 period (ARR) */

  spwm->per = PWM_ARR_GET(pwm);

  spwm_tim1_stop(spwm);

errout:
  return ret;
}

/****************************************************************************
 * Name: spwm_tim1_start
 ****************************************************************************/

static int spwm_tim1_start(struct spwm_s *spwm)
{
  struct stm32_pwm_dev_s *pwm = spwm->pwm;
  uint16_t outputs = 0;
  int i = 0;

  /* Get outputs */

  for (i = 0; i < spwm->phases; i += 1)
    {
      outputs |= (1 << (i * 2));
    }

  /* Enable PWM outputs */

  PWM_OUTPUTS_ENABLE(pwm, outputs, true);

  /* Enable TIM1 */

  PWM_TIM_ENABLE(pwm, true);

  return OK;
}

/****************************************************************************
 * Name: spwm_tim1_stop
 ****************************************************************************/

static int spwm_tim1_stop(struct spwm_s *spwm)
{
  struct stm32_pwm_dev_s *pwm = spwm->pwm;
  uint16_t outputs = 0;
  int i = 0;

  /* Get outputs */

  for (i = 0; i < spwm->phases; i += 1)
    {
      outputs |= (1 << (i * 2));
    }

  /* Disable PWM outputs */

  PWM_OUTPUTS_ENABLE(pwm, outputs, false);

  /* Disable TIM1 */

  PWM_TIM_ENABLE(pwm, false);

  return OK;
}

#endif /* CONFIG_NUCLEOF334R8_SPWM_USE_TIM1 */

/****************************************************************************
 * Name: spwm_setup
 ****************************************************************************/

static int spwm_setup(struct spwm_s *spwm)
{
  int ret = OK;

#if defined(CONFIG_NUCLEOF334R8_SPWM_USE_HRTIM1)
  /* HRTIM setup */

  printf("Setup HRTIM\n");
  ret = spwm_hrtim_setup(spwm);
  if (ret < 0)
    {
      goto errout;
    }
#elif defined(CONFIG_NUCLEOF334R8_SPWM_USE_TIM1)
  /* TIM1 setup - PWM */

  printf("Setup TIM1 and TIM6\n");
  ret = spwm_tim1_setup(spwm);
  if (ret < 0)
    {
      goto errout;
    }

  /* TIM6 setup - IRQ */

  ret = spwm_tim6_setup(spwm);
  if (ret < 0)
    {
      goto errout;
    }
#else
#  error
#endif

errout:
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spwm_main
 *
 * Description:
 *   Entrypoint for SPWM example.
 *
 ****************************************************************************/

int spwm_main(int argc, char *argv[])
{
  struct spwm_s *spwm = NULL;
  int ret = OK;
  int i = 0;

  spwm = &g_spwm;

  printf("\nspwm_main: Started\n");

  /* Setup SPWM example */

  ret = spwm_setup(spwm);
  if (ret < 0)
    {
      printf("ERROR: failed to setup SPWM %d!\n", ret);
      goto errout;
    }

  /* Initialize modulation waveform */

  ret = waveform_init(spwm, waveform_func);
  if (ret < 0)
    {
      printf("ERROR: failed initialize modulation wavefrom %d!\n", ret);
      goto errout;
    }

  /* Start SPWM */

  ret = spwm_start(spwm);
  if (ret < 0)
    {
      printf("ERROR: failed start SPWM %d!\n", ret);
      goto errout;
    }

  /* Main loop */

  while (1)
    {
      /* Print counter */

      printf("%d\n", i);

      /* Increase counter */

      i += 1;

      /* Sleep */

      nxsig_sleep(1);
    }

errout:
  spwm_stop(spwm);

  return 0;
}

#endif /* CONFIG_NUCLEOF334R8_SPWM */
