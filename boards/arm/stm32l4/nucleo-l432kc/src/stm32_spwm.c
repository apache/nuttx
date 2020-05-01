/****************************************************************************
 * boards/arm/stm32/nucleo-l432kc/src/stm32_spwm.c
 *
 *   Copyright (C) 2018, 2019 Gregory Nutt. All rights reserved.
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
#include <math.h>

#include <nuttx/arch.h>
#include <nuttx/signal.h>

#include <arch/irq.h>
#include <arch/chip/chip.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "ram_vectors.h"

#include "stm32l4_pwm.h"
#include "stm32l4_tim.h"

#ifdef CONFIG_NUCLEOL432KC_SPWM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Assertions ***************************************************************/

#ifndef CONFIG_ARCH_CHIP_STM32L432KC
#  warning "This only have been verified with CONFIG_ARCH_CHIP_STM32L432KC"
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

#ifdef CONFIG_NUCLEOL432KC_SPWM_USE_TIM1

/* Phase 1 is TIM1 CH1 */

#  if CONFIG_NUCLEOL432KC_SPWM_PHASE_NUM > 0
#    ifndef CONFIG_STM32L4_TIM1_CH1OUT
#      error
#    endif
#    ifndef CONFIG_STM32L4_TIM6
#      error
#    endif
#  endif

/* Phase 2 is TIM1 CH2 */

#  if CONFIG_NUCLEOL432KC_SPWM_PHASE_NUM > 1
#    ifndef CONFIG_STM32L4_TIM1_CH2OUT
#      error
#    endif
#  endif

/* Phase 3 is TIM1 CH3 */

#  if CONFIG_NUCLEOL432KC_SPWM_PHASE_NUM > 2
#    ifndef CONFIG_STM32L4_TIM1_CH3OUT
#      error
#    endif
#  endif

/* Phase 4 is TIM1 CH4 */

#  if CONFIG_NUCLEOL432KC_SPWM_PHASE_NUM > 3
#    ifndef CONFIG_STM32L4_TIM1_CH4OUT
#      error
#    endif
#  endif

#  if CONFIG_NUCLEOL432KC_SPWM_PHASE_NUM != PWM_TIM1_NCHANNELS
#    error
#  endif

#endif /* CONFIG_NUCLEOL432KC_SPWM_USE_TIM1 */

/* Configuration ************************************************************/

#ifdef CONFIG_NUCLEOL432KC_SPWM_USE_TIM1
#  define PWM_TIMERS_IN_USE 1
#endif

#define SPWM_PHASE_SHIFT ((360.0f/CONFIG_NUCLEOL432KC_SPWM_PHASE_NUM))

#define SAMPLES_NUM CONFIG_NUCLEOL432KC_SPWM_SAMPLES
#define PHASES_NUM CONFIG_NUCLEOL432KC_SPWM_PHASE_NUM

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* SPWM private data */

struct spwm_s
{
  FAR struct stm32l4_pwm_dev_s *pwm;
#ifdef CONFIG_NUCLEOL432KC_SPWM_USE_TIM1
  FAR struct stm32l4_tim_dev_s *tim;
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
  .waveform_freq = ((float)CONFIG_NUCLEOL432KC_SPWM_FREQ),
  .phases        = PHASES_NUM,
  .samples       = SAMPLES_NUM,
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static float waveform_func(float x);
static int waveform_init(FAR struct spwm_s *spwm, float (*f)(float));
static int spwm_start(FAR struct spwm_s *spwm);
static int spwm_start(FAR struct spwm_s *spwm);
static int spwm_stop(FAR struct spwm_s *spwm);
#ifdef CONFIG_NUCLEOL432KC_SPWM_USE_TIM1
static int spwm_tim1_setup(FAR struct spwm_s *spwm);
static int spwm_tim6_setup(FAR struct spwm_s *spwm);
static int spwm_tim1_start(FAR struct spwm_s *spwm);
static int spwm_tim6_start(FAR struct spwm_s *spwm);
static int spwm_tim1_stop(FAR struct spwm_s *spwm);
static int spwm_tim6_stop(FAR struct spwm_s *spwm);
#endif /* CONFIG_NUCLEOL432KC_SPWM_USE_TIM1 */
static int spwm_setup(FAR struct spwm_s *spwm);

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

static int waveform_init(FAR struct spwm_s *spwm, float (*f)(float))
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
        (spwm->samples / CONFIG_NUCLEOL432KC_SPWM_PHASE_NUM) * i;
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

static int spwm_start(FAR struct spwm_s *spwm)
{
  /* Start TIM1 */

  spwm_tim1_start(spwm);

  /* Start TIM6 */

  spwm_tim6_start(spwm);

  return OK;
}

/****************************************************************************
 * Name: spwm_stop
 *
 * Description:
 *   Stop SPWM
 *
 ****************************************************************************/

static int spwm_stop(FAR struct spwm_s *spwm)
{
  /* Stop TIM1 */

  spwm_tim1_stop(spwm);

  /* Stop TIM6 */

  spwm_tim6_stop(spwm);

  return OK;
}

#ifdef CONFIG_NUCLEOL432KC_SPWM_USE_TIM1

/****************************************************************************
 * Name: tim6_handler
 ****************************************************************************/

static void tim6_handler(void)
{
  FAR struct spwm_s *spwm = &g_spwm;
  FAR struct stm32l4_pwm_dev_s *pwm = spwm->pwm;
  FAR struct stm32l4_tim_dev_s *tim = spwm->tim;
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

  STM32L4_TIM_ACKINT(tim, ATIM_SR_UIF);
}

/****************************************************************************
 * Name: spwm_tim6_setup
 ****************************************************************************/

static int spwm_tim6_setup(FAR struct spwm_s *spwm)
{
  FAR struct stm32l4_tim_dev_s *tim = NULL;
  uint64_t freq = 0;
  uint32_t per = 0;
  int ret = OK;

  /* Get TIM6 interface */

  tim = stm32l4_tim_init(6);
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

  STM32L4_TIM_SETCLOCK(tim, BOARD_TIM6_FREQUENCY);
  STM32L4_TIM_SETPERIOD(tim, per);

  /* Attach TIM6 ram vector */

  ret = arm_ramvec_attach(STM32L4_IRQ_TIM6, tim6_handler);
  if (ret < 0)
    {
      printf("ERROR: arm_ramvec_attach failed: %d\n", ret);
      ret = -1;
      goto errout;
    }

  /* Set the priority of the TIM6 interrupt vector */

  ret = up_prioritize_irq(STM32L4_IRQ_TIM6, NVIC_SYSH_HIGH_PRIORITY);
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

static int spwm_tim6_start(FAR struct spwm_s *spwm)
{
  FAR struct stm32l4_tim_dev_s *tim = spwm->tim;

  /* Enable the timer interrupt at the NVIC and at TIM6 */

  up_enable_irq(STM32L4_IRQ_TIM6);
  STM32L4_TIM_ENABLEINT(tim, BTIM_DIER_UIE);

  return OK;
}

/****************************************************************************
 * Name: spwm_tim6_stop
 ****************************************************************************/

static int spwm_tim6_stop(FAR struct spwm_s *spwm)
{
  FAR struct stm32l4_tim_dev_s *tim = spwm->tim;

  /* Disable the timer interrupt at the NVIC and at TIM6 */

  up_disable_irq(STM32L4_IRQ_TIM6);
  STM32L4_TIM_DISABLEINT(tim, BTIM_DIER_UIE);

  return OK;
}

/****************************************************************************
 * Name: spwm_tim1_setup
 ****************************************************************************/

static int spwm_tim1_setup(FAR struct spwm_s *spwm)
{
  FAR struct stm32l4_pwm_dev_s *pwm = NULL;
  int ret = OK;

  /* Get TIM1 PWM interface */

  pwm = (FAR struct stm32l4_pwm_dev_s *)stm32l4_pwminitialize(1);
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

  ret = PWM_FREQ_UPDATE(pwm, CONFIG_NUCLEOL432KC_SPWM_PWM_FREQ);
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

static int spwm_tim1_start(FAR struct spwm_s *spwm)
{
  FAR struct stm32l4_pwm_dev_s *pwm = spwm->pwm;
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

static int spwm_tim1_stop(FAR struct spwm_s *spwm)
{
  FAR struct stm32l4_pwm_dev_s *pwm = spwm->pwm;
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

#endif /* CONFIG_NUCLEOL432KC_SPWM_USE_TIM1 */

/****************************************************************************
 * Name: spwm_setup
 ****************************************************************************/

static int spwm_setup(FAR struct spwm_s *spwm)
{
  int ret = OK;

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
  FAR struct spwm_s *spwm = NULL;
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

#endif /* CONFIG_NUCLEOL432KC_SPWM */
