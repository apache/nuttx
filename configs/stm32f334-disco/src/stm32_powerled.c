/****************************************************************************
 * configs/stm32f334-disco/src/stm32_powerled.c
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

#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdlib.h>
#include <sys/boardctl.h>
#include <sys/ioctl.h>
#include <sys/types.h>

#include <nuttx/board.h>
#include <nuttx/fs/fs.h>

#include <nuttx/analog/comp.h>
#include <nuttx/analog/dac.h>

#include <nuttx/power/powerled.h>

#include "stm32.h"

#if defined(CONFIG_EXAMPLES_POWERLED) && defined(CONFIG_DRIVERS_POWERLED)

#if !defined(CONFIG_STM32_HRTIM1) || !defined(CONFIG_HRTIM)
#error "Powerled example requires HRTIM1 support"
#endif

#if !defined(CONFIG_STM32_DAC1CH1) || !defined(CONFIG_DAC)                    \
    || !defined(CONFIG_STM32_DAC1CH1_DMA)
#error "Powerled example requires DAC1 with DMA support"
#endif

#if !defined(CONFIG_STM32_COMP4) || !defined(CONFIG_COMP)
#error "Powerled example requires COMP4 support"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* REVISIT: Move to stm32_hrtim.h ? */

#define HRTIM_CMP_SET(hrtim, tim, index, cmp)       \
        hrtim->hd_ops->cmp_update(hrtim, tim, index, cmp)
#define HRTIM_PER_SET(hrtim, tim, per)              \
        hrtim->hd_ops->per_update(hrtim, tim, per)
#define HRTIM_OUTPUTS_ENABLE(hrtim, tim, state)     \
        hrtim->hd_ops->outputs_enable(hrtim, tim, state)
#define HRTIM_BURST_CMP_SET(hrtim, cmp)             \
        hrtim->hd_ops->burst_cmp_set(hrtim, cmp);
#define HRTIM_BURST_PER_SET(hrtim, per)             \
        hrtim->hd_ops->burst_per_set(hrtim, per);
#define HRTIM_BURST_ENABLE(hrtim, state)            \
        hrtim->hd_ops->burst_enable(hrtim, state);

#if CONFIG_STM32_DAC1CH1_DMA_BUFFER_SIZE != 5
  #error "This example requires DAC1CH1 DMA buffer size == 5"
#endif

/* Peripheral selection */

#define DAC_CURRENT_LIMIT 1
#define COMP_CURRENT_LIMIT 4

/* Maximum onboard LED current is 350mA */

#define LED_ABSOLUTE_CURRENT_LIMIT_mA 250

/* Voltage reference for DAC */

#define DAC_REF_VOLTAGE_mV 3300

/* DAC resolution */

#define DAC_RESOLUTION 4096

/* Current sense resistance */

#define LED_CURRENT_SENSE_RESISTOR 1

/* There is 1 Ohm resistor in series with MOSFET which gives 1A/1V ratio */

#define CURRENT_TO_VOLTAGE_RATIO (1/(LED_CURRENT_SENSE_RESISTOR))

/****************************************************************************
 * Private Type Definition
 ****************************************************************************/

/* Private data for powerled */

struct powerled_priv_s
{
  bool running;
  uint16_t dacbuffer[CONFIG_STM32_DAC1CH1_DMA_BUFFER_SIZE];
  uint16_t current_tab[CONFIG_STM32_DAC1CH1_DMA_BUFFER_SIZE];
};

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

static int powerled_setup(FAR struct powerled_dev_s *dev);
static int powerled_shutdown(FAR struct powerled_dev_s *dev);
static int powerled_start(FAR struct powerled_dev_s *dev);
static int powerled_stop(FAR struct powerled_dev_s *dev);
static int powerled_params_set(FAR struct powerled_dev_s *dev,
                           FAR struct powerled_params_s *param);
static int powerled_mode_set(FAR struct powerled_dev_s *dev, uint8_t mode);
static int powerled_limits_set(FAR struct powerled_dev_s *dev,
                           FAR struct powerled_limits_s *limits);
static int powerled_state_get(FAR struct powerled_dev_s *dev,
                          FAR struct powerled_state_s *state);
static int powerled_fault_set(FAR struct powerled_dev_s *dev, uint8_t fault);
static int powerled_fault_get(FAR struct powerled_dev_s *dev,
                              FAR uint8_t *fault);
static int powerled_fault_clean(FAR struct powerled_dev_s *dev,
                                uint8_t fault);
static int powerled_ioctl(FAR struct powerled_dev_s *dev, int cmd,
                          unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct powerled_priv_s powerled_priv;

struct powerled_s g_powerled =
{
  .opmode = POWERLED_OPMODE_INIT,
  .state = POWERLED_STATE_INIT,
  .priv = &powerled_priv
};

struct powerled_ops_s g_powerled_ops =
{
  .setup       = powerled_setup,
  .shutdown    = powerled_shutdown,
  .start       = powerled_start,
  .stop        = powerled_stop,
  .params_set  = powerled_params_set,
  .mode_set    = powerled_mode_set,
  .limits_set  = powerled_limits_set,
  .state_get   = powerled_state_get,
  .fault_set   = powerled_fault_set,
  .fault_get   = powerled_fault_get,
  .fault_clean = powerled_fault_clean,
  .ioctl       = powerled_ioctl
};

struct powerled_dev_s g_powerled_dev =
{
  .ops = &g_powerled_ops,
  .priv = &g_powerled
};

/* Lower part of the POWERLED driver gather 3 pheriperals:
 * - HRTIM   - responsible for PWM generation
 * - COMP4   - acting as current limit
 * - DAC1CH1 - provides threshold voltage to comparator inverting input
 */

struct powerled_lower_dev_s
{
  struct hrtim_dev_s *hrtim; /* PWM generation */
  struct comp_dev_s  *comp;  /* current limit control */
  struct dac_dev_s   *dac;   /* current limit threshold */
  struct adc_dev_s   *adc;   /* not used in this demo - only as reference */
  struct opamp_dev_s *opamp; /* not used in this demo - only as reference */
};

struct powerled_lower_dev_s g_powerled_lower =
{
  .hrtim = NULL,
  .comp  = NULL,
  .dac   = NULL,
  .adc   = NULL,
  .opamp = NULL
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int powerled_shutdown(FAR struct powerled_dev_s *dev)
{
  /* TODO ? */

  return 0;
}

/****************************************************************************
 * Name: powerled_setup
 *
 * Description:
 *
 *
 * Returned Value:
 *  0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int powerled_setup(FAR struct powerled_dev_s *dev)
{
  FAR struct powerled_lower_dev_s *lower = dev->lower;
  FAR struct powerled_s  *powerled    = (FAR struct powerled_s *)dev->priv;
  FAR struct hrtim_dev_s *hrtim      = NULL;
  FAR struct comp_dev_s *comp        = NULL;
  FAR struct dac_dev_s *dac          = NULL;
  FAR struct powerled_priv_s *priv   = (struct powerled_priv_s *)powerled->priv;

  /* Check lower half drivers */

  hrtim = lower->hrtim;
  if (hrtim == NULL)
    {
      printf("ERROR: failed to get hrtim ");
    }

  comp = lower->comp;
  if (comp == NULL)
    {
      printf("ERROR: failed to get lower level interface");
    }

  dac = lower->dac;
  if (dac == NULL)
    {
      printf("ERROR: failed to get lower level interface");
    }

  /* Do nothing */

  return OK;
}

static int powerled_start(FAR struct powerled_dev_s *dev)
{
  FAR struct powerled_lower_dev_s *lower    = dev->lower;
  FAR struct powerled_s  *powerled = (FAR struct powerled_s *)dev->priv;
  FAR struct hrtim_dev_s *hrtim    = lower->hrtim;
  FAR struct dac_dev_s   *dac      = lower->dac;
  struct powerled_priv_s *priv     = (struct powerled_priv_s *)powerled->priv;
  int i = 0;
  int current_av_mA = 0.0;
  int current_max_mA = ((int)(powerled->limits.current * 1000));
  uint16_t burst_cmp = 0;
  uint16_t burst_per = 0;

  /* Stop HRTIM PWM */

  HRTIM_OUTPUTS_ENABLE(hrtim, HRTIM_OUT_TIMC_CH1, false);
  HRTIM_BURST_ENABLE(hrtim, false);

  /* Configure according to mode */

  if (powerled->opmode == POWERLED_OPMODE_CONTINUOUS)
    {
      /* Average curent set to max */

      current_av_mA = ((uint16_t)(current_max_mA));

      /* Dimming through burst mode IDLE state */

      burst_per = 1000;

      burst_cmp = ((uint16_t)(((float)burst_per)*(100.0-powerled->param.brightness)/100.0));
    }

  else if (powerled->opmode == POWERLED_OPMODE_FLASH)
    {
      /* Average current - brightness */
      /* Flashing through burst mode IDLE state */

      /* Maximum brightness is achieved when average LED current is equalt to
       * LED current limit, and there is no IDLE state */

      current_av_mA = ((uint16_t)(powerled->param.brightness * current_max_mA
                                  / POWERLED_BRIGHTNESS_MAX));

      /* TODO: configure burst mode period and compare.
       * Probably need to change burst mode prescaler in run-time ??
       */

      burst_per = 65000;
      burst_cmp = 64000;
    }

  /* Configure DAC buffer */

  for (i = 0; i < CONFIG_STM32_DAC1CH1_DMA_BUFFER_SIZE; i += 1)
    {
      /* TODO: add slope compensation */

      priv->current_tab[i] = current_av_mA ;
    }

  /* Convert current sense value thresholds for DAC */

  for (i = 0; i < CONFIG_STM32_DAC1CH1_DMA_BUFFER_SIZE; i += 1)
    {
      priv->dacbuffer[i] = priv->current_tab[i] * DAC_RESOLUTION / DAC_REF_VOLTAGE_mV;
    }

  /* Write DAC buffer */

  dac->ad_ops->ao_ioctl(dac, IO_DMABUFFER_INIT, (unsigned long)priv->dacbuffer);

  /* Configure HRTIM PWM */

  /* 1 period is 4us - 100% time */

  HRTIM_PER_SET(hrtim, HRTIM_TIMER_TIMC, 18432);

  /* 20% time */

  HRTIM_CMP_SET(hrtim, HRTIM_TIMER_TIMC, HRTIM_CMP1, 3686);

  /* 40% time */

  HRTIM_CMP_SET(hrtim, HRTIM_TIMER_TIMC, HRTIM_CMP2, 7373);

  /* 60% time */

  HRTIM_CMP_SET(hrtim, HRTIM_TIMER_TIMC, HRTIM_CMP3, 11059);

  /* 80% time */

  HRTIM_CMP_SET(hrtim, HRTIM_TIMER_TIMC, HRTIM_CMP4, 14746);

  /* Start DAC */

  dac->ad_ops->ao_send(dac, NULL);

  /* Start HRTIM PWM */

  HRTIM_OUTPUTS_ENABLE(hrtim, HRTIM_OUT_TIMC_CH1, true);

  /* Configure burst mode */

  HRTIM_BURST_CMP_SET(hrtim, burst_cmp);
  HRTIM_BURST_PER_SET(hrtim, burst_per);
  HRTIM_BURST_ENABLE(hrtim, true);

  /* Set running flag */

  if (priv->running == false)
    {
      priv->running = true;
    }

  return OK;
}

static int powerled_stop(FAR struct powerled_dev_s *dev)
{
  FAR struct powerled_lower_dev_s *lower = dev->lower;
  FAR struct hrtim_dev_s *hrtim      = lower->hrtim;

  printf("powerled_stop\n");

  HRTIM_OUTPUTS_ENABLE(hrtim, HRTIM_OUT_TIMC_CH1, false);

  return OK;
}

static int powerled_params_set(FAR struct powerled_dev_s *dev,
                           FAR struct powerled_params_s *param)
{
  FAR struct powerled_s *powerled    = (FAR struct powerled_s *)dev->priv;
  FAR struct powerled_lower_dev_s *lower  = dev->lower;
  FAR struct hrtim_dev_s *hrtim       = lower->hrtim;
  FAR struct dac_dev_s *dac           = lower->dac;
  int ret = OK;

  /* Store params in pirvate struct. */

  powerled->param.brightness = param->brightness;
  powerled->param.frequency  = param->frequency;
  powerled->param.duty       = param->duty;

  return ret;
}

static int powerled_mode_set(FAR struct powerled_dev_s *dev, uint8_t mode)
{
  FAR struct powerled_s *powerled    = (FAR struct powerled_s *)dev->priv;
  int ret = OK;

  switch (mode)
    {
      case POWERLED_OPMODE_CONTINUOUS:
        {
          powerled->opmode = mode;
          break;
        }

      case POWERLED_OPMODE_FLASH:
        {
          powerled->opmode = mode;
          break;
        }

      default:
        {
          printf("ERROR: unsupported POWERLED mode %d!\n", mode);
          ret = ERROR;
          goto errout;
        }
    }

errout:
  return ret;
}

static int powerled_limits_set(FAR struct powerled_dev_s *dev,
                           FAR struct powerled_limits_s *limits)
{
  FAR struct powerled_s *powerled    = (FAR struct powerled_s *)dev->priv;
  int ret = OK;

  /* Some assertions */

  if (limits->current <= 0)
    {
      printf("Output current limit must be set!\n");
      ret = ERROR;
      goto errout;
    }

  if (limits->current > LED_ABSOLUTE_CURRENT_LIMIT_mA)
    {
      limits->current = LED_ABSOLUTE_CURRENT_LIMIT_mA;
      printf("LED current limiit > LED absoulute current limit."
             " Set current limit to %d.\n",
             limits->current);
    }

  /* We need only output current */

  powerled->limits.current = limits->current;

  /* Lock limits */

  powerled->limits.lock = true;

errout:
  return ret;
}

static int powerled_state_get(FAR struct powerled_dev_s *dev,
                          FAR struct powerled_state_s *state)
{
  return 0;
}

static int powerled_fault_set(FAR struct powerled_dev_s *dev, uint8_t fault)
{
  return 0;
}

static int powerled_fault_get(FAR struct powerled_dev_s *dev, FAR uint8_t *fault)
{
  return 0;
}

static int powerled_fault_clean(FAR struct powerled_dev_s *dev, uint8_t fault)
{
  return 0;
}

static int powerled_ioctl(FAR struct powerled_dev_s *dev, int cmd, unsigned long arg)
{
  int ret = ERROR;

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_powerled_setup
 *
 * Description:
 *  Initialize POWERLED driver.
 *  This function should be call by board_app_initialize() (?)
 *
 * Returned Value:
 *  0 on success, a negated errno value on failure
 *
 ****************************************************************************/

int stm32_powerled_setup(void)
{
  static bool initialized        = false;
  struct hrtim_dev_s *hrtim      = NULL;
  struct comp_dev_s *comp        = NULL;
  struct dac_dev_s *dac          = NULL;
  struct powerled_lower_dev_s *lower = &g_powerled_lower;
  struct powerled_dev_s *powerled        = &g_powerled_dev;
  int ret;

  if (!initialized)
    {
      /* Get the HRTIM interface */

      hrtim = stm32_hrtiminitialize();
      if (hrtim == NULL)
        {
          printf("ERROR: Failed to get HRTIM1 interface\n");
          return -ENODEV;
        }

      /* Get the DAC interface */

      dac = stm32_dacinitialize(DAC_CURRENT_LIMIT);
      if (dac == NULL)
        {
          printf("ERROR: Failed to get DAC %d interface\n", DAC_CURRENT_LIMIT);
          return -ENODEV;
        }

      /* Get the COMP interface */

      comp = stm32_compinitialize(COMP_CURRENT_LIMIT);
      if (comp == NULL)
        {
          printf("ERROR: Failed to get COMP %d interface\n",
                 COMP_CURRENT_LIMIT);
          return -ENODEV;
        }

      /* Initialize POWERLED lower driver interfaces */

      lower->hrtim = hrtim;
      lower->comp  = comp;
      lower->dac   = dac;
      lower->adc   = NULL;
      lower->opamp = NULL;

      /* We do not need register character drivers for POWERLED lower peripherals.
       * All control should be done via POWERLED character driver.
       */

      ret = powerled_register("/dev/powerled0", powerled, (void *)lower);
      if (ret < 0)
        {
          printf("ERROR: powerled_register failed: %d\n", ret);
          return ret;
        }

      initialized = true;
    }

  return OK;
}

#endif /* CONFIG_EXAMPLE_POWERLED && CONFIG_DRIVERS_POWERLED*/
