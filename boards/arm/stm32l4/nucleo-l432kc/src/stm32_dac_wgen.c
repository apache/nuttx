/****************************************************************************
 * boards/arm/stm32l4/nucleo-l432kc/src/stm32_dac_wgen.c
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

#include <debug.h>
#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/signal.h>
#include <arch/board/board.h>

#include "stm32l4_dac.h"

#include "nucleo-l432kc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_NUCLEOL432KC_DAC_WGEN_SAMPLES
#  define CONFIG_NUCLEOL432KC_DAC_WGEN_SAMPLES 40
#endif

#ifndef CONFIG_NUCLEOL432KC_DAC_WGEN_FREQ
#  define CONFIG_NUCLEOL432KC_DAC_WGEN_FREQ 50
#endif

/* Assertions ***************************************************************/

#ifndef CONFIG_ARCH_CHIP_STM32L432KC
#  warning "This only have been verified with CONFIG_ARCH_CHIP_STM32L432KC"
#endif

#ifndef CONFIG_DAC
#  error "CONFIG_DAC is required"
#endif

#ifndef CONFIG_STM32L4_DAC1
#  error "CONFIG_STM32L4_DAC1 is required"
#endif

#ifndef CONFIG_STM32L4_DAC_LL_OPS
#  error "CONFIG_STM32L4_DAC_LL_OPS is required"
#endif

#ifndef CONFIG_STM32L4_DAC1_DMA
#  error "CONFIG_STM32L4_DAC1_DMA is required"
#endif

#if (CONFIG_STM32L4_DAC1_DMA_BUFFER_SIZE < CONFIG_NUCLEOL432KC_DAC_WGEN_SAMPLES)
#  error "DMA buffer size should be equal or greater than the number of samples."
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* DAC wave generator private data */

struct dac_wgen_s
{
  struct stm32_dac_dev_s *dac;
  uint16_t               *dac_dmabuffer;
  uint16_t               samples;        /* Waveform samples num */
  float                  phase_step;     /* Waveform phase step */
  float                  waveform_freq;  /* Waveform frequency */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct dac_wgen_s g_dac_wgen =
{
  .dac           = NULL,
  .dac_dmabuffer = stm32l4_dac1_dmabuffer,
  .samples       = CONFIG_NUCLEOL432KC_DAC_WGEN_SAMPLES,
  .waveform_freq = ((float)CONFIG_NUCLEOL432KC_DAC_WGEN_FREQ)
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static float waveform_func(float x);
static void waveform_init(struct dac_wgen_s *dac_wgen, float (*f)(float));

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
 *   Initialize waveform
 *
 ****************************************************************************/

static void waveform_init(struct dac_wgen_s *dac_wgen, float (*f)(float))
{
  int i;
  float value;

  /* Get phase step to achieve one sine waveform period */

  dac_wgen->phase_step = (float)(2 * M_PI / dac_wgen->samples);

  /* Initialize sine table */

  for (i = 0; i < dac_wgen->samples; i += 1)
    {
      /* We need sine in range from 0 to 1.0 */

      value = (sinf(dac_wgen->phase_step * i) + 1.0f) / 2.0f;
      dac_wgen->dac_dmabuffer[i] = (uint16_t)(value * 4095);
    }
}

/****************************************************************************
 * Name: dac_wgen_start
 ****************************************************************************/

static int dac_wgen_start(struct dac_wgen_s *dac_wgen)
{
  DAC_ENABLE(dac_wgen->dac, true);
  DAC_START_DMA(dac_wgen->dac);

  return OK;
}

/****************************************************************************
 * Name: dac_wgen_stop
 ****************************************************************************/

static int dac_wgen_stop(struct dac_wgen_s *dac_wgen)
{
  DAC_STOP_DMA(dac_wgen->dac);
  DAC_ENABLE(dac_wgen->dac, false);

  return OK;
}

/****************************************************************************
 * Name: dac_wgen_setup
 ****************************************************************************/

int dac_wgen_setup(struct dac_wgen_s *dac_wgen)
{
  struct dac_dev_s *dac;

  int ret = OK;

  dac = stm32l4_dacinitialize(0);
  if (dac == NULL)
    {
      syslog(LOG_ERR, "Failed to get DAC interface\n");
      ret = ERROR;
      goto errout;
    }

  dac_wgen->dac = (struct stm32_dac_dev_s *)dac->ad_priv;

errout:
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dac_wgen_main
 *
 * Description:
 *   Entrypoint for DAC waveform generator example.
 *
 ****************************************************************************/

int dac_wgen_main(int argc, char *argv[])
{
  struct dac_wgen_s *dac_wgen = NULL;

  int i = 0;
  int ret = OK;

  dac_wgen = &g_dac_wgen;

  syslog(LOG_INFO, "\ndac_wgen_main: Started\n");

  /* Setup DAC wave generator */

  ret = dac_wgen_setup(dac_wgen);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to setup DAC WGEN %d!\n", ret);
      goto errout;
    }

  /* Initialize modulation waveform */

  waveform_init(dac_wgen, waveform_func);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed initialize waveform %d!\n", ret);
      goto errout;
    }

  /* Start DAC wave generator */

  ret = dac_wgen_start(dac_wgen);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed start DAC WGEN %d!\n", ret);
      goto errout;
    }

  /* Main loop */

  while (1)
    {
      /* Print counter */

      syslog(LOG_INFO, "%d\n", i);

      /* Increase counter */

      i += 1;

      /* Sleep */

      nxsig_sleep(1);
    }

errout:
  dac_wgen_stop(dac_wgen);

  return OK;
}
