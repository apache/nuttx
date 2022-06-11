/****************************************************************************
 * boards/arm/stm32/stm32f334-disco/src/stm32_powerled.c
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

#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/boardctl.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/fs/fs.h>

#include <nuttx/analog/comp.h>
#include <nuttx/analog/dac.h>

#include <nuttx/power/powerled.h>

#include "stm32_comp.h"
#include "stm32_hrtim.h"
#include "stm32_dac.h"

#if defined(CONFIG_EXAMPLES_POWERLED) && defined(CONFIG_DRIVERS_POWERLED)

#if !defined(CONFIG_STM32_HRTIM1) || !defined(CONFIG_STM32_HRTIM)
#  error "Powerled example requires HRTIM1 support"
#endif

#if !defined(CONFIG_STM32_DAC1CH1) || !defined(CONFIG_DAC) || \
    !defined(CONFIG_STM32_DAC1CH1_DMA)
#  error "Powerled example requires DAC1 with DMA support"
#endif

#if !defined(CONFIG_STM32_COMP4) || !defined(CONFIG_COMP)
#  error "Powerled example requires COMP4 support"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DAC_BUFFER_INIT(dac, buffer) \
        dac->ad_ops->ao_ioctl(dac, IO_DMABUFFER_INIT, (unsigned long)buffer)

#define DAC_START(dac) dac->ad_ops->ao_send(dac, NULL);

#if CONFIG_STM32_DAC1CH1_DMA_BUFFER_SIZE != 5
  #error "This example requires DAC1CH1 DMA buffer size == 5"
#endif

/* Peripheral selection */

#define DAC_CURRENT_LIMIT  1
#define COMP_CURRENT_LIMIT 4

/* Maximum onboard LED current is 350mA */

#define LED_ABSOLUTE_CURRENT_LIMIT 250 /* in mA */

#if (CONFIG_EXAMPLES_POWERLED_CURRENT_LIMIT > LED_ABSOLUTE_CURRENT_LIMIT)
#  error "Board LED maximum current is 250 mA"
#endif

/* Voltage reference for DAC (in mV) */

#define DAC_REV_VOLTAGE 3300

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

static int powerled_setup(struct powerled_dev_s *dev);
static int powerled_shutdown(struct powerled_dev_s *dev);
static int powerled_start(struct powerled_dev_s *dev);
static int powerled_stop(struct powerled_dev_s *dev);
static int powerled_params_set(struct powerled_dev_s *dev,
                               struct powerled_params_s *param);
static int powerled_mode_set(struct powerled_dev_s *dev, uint8_t mode);
static int powerled_limits_set(struct powerled_dev_s *dev,
                               struct powerled_limits_s *limits);
static int powerled_state_get(struct powerled_dev_s *dev,
                              struct powerled_state_s *state);
static int powerled_fault_set(struct powerled_dev_s *dev, uint8_t fault);
static int powerled_fault_get(struct powerled_dev_s *dev,
                              uint8_t *fault);
static int powerled_fault_clean(struct powerled_dev_s *dev,
                                uint8_t fault);
static int powerled_ioctl(struct powerled_dev_s *dev, int cmd,
                          unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct powerled_priv_s powerled_priv;

struct powerled_s g_powerled;

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

struct powerled_lower_dev_s g_powerled_lower;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int powerled_shutdown(struct powerled_dev_s *dev)
{
  struct powerled_s      *powerled = (struct powerled_s *)dev->priv;
  struct powerled_priv_s *priv =
    (struct powerled_priv_s *)powerled->priv;

  /* Stop powerled if running */

  if (priv->running == true)
    {
      powerled_stop(dev);
    }

  /* Reset powerled structure */

  memset(powerled, 0, sizeof(struct powerled_s));

  return OK;
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

static int powerled_setup(struct powerled_dev_s *dev)
{
  struct powerled_lower_dev_s *lower = dev->lower;
  struct powerled_s  *powerled   = (struct powerled_s *)dev->priv;
  struct hrtim_dev_s *hrtim      = NULL;
  struct comp_dev_s  *comp       = NULL;
  struct dac_dev_s   *dac        = NULL;

  /* Initialize powerled structure */

  powerled->opmode       = POWERLED_OPMODE_INIT;
  powerled->state.state  = POWERLED_STATE_INIT;
  powerled->priv         = &powerled_priv;

  /* Check lower half drivers */

  hrtim = lower->hrtim;
  if (hrtim == NULL)
    {
      pwrerr("ERROR: failed to get hrtim ");
    }

  comp = lower->comp;
  if (comp == NULL)
    {
      pwrerr("ERROR: failed to get lower level interface");
    }

  dac = lower->dac;
  if (dac == NULL)
    {
      pwrerr("ERROR: failed to get lower level interface");
    }

  /* Do nothing */

  return OK;
}

static int powerled_start(struct powerled_dev_s *dev)
{
  struct powerled_lower_dev_s *lower = dev->lower;
  struct powerled_s      *powerled   =
    (struct powerled_s *)dev->priv;
  struct hrtim_dev_s     *hrtim      = lower->hrtim;
  struct dac_dev_s       *dac        = lower->dac;
  struct powerled_priv_s *priv =
    (struct powerled_priv_s *)powerled->priv;
  uint16_t burst_cmp = 0;
  uint16_t burst_per = 0;
  uint16_t burst_pre = 0;
  int current_av  = 0;
  int current_max;
  int i;

  /* Set max current in mA */

  current_max = (int)(powerled->limits.current * 1000);

  /* Stop HRTIM PWM */

  HRTIM_OUTPUTS_ENABLE(hrtim, HRTIM_OUT_TIMC_CH1, false);
  HRTIM_BURST_ENABLE(hrtim, false);

  /* Configure according to mode */

  if (powerled->opmode == POWERLED_OPMODE_CONTINUOUS)
    {
      /* Average current set to max */

      current_av = (uint16_t)(current_max);

      /* Dimming through burst mode IDLE state */

      burst_pre = HRTIM_BURST_PRESCALER_1;
      burst_per = 1000;
      burst_cmp = (uint16_t)(((float)burst_per) *
                              (100.0 - powerled->param.brightness) / 100.0);
    }

  else if (powerled->opmode == POWERLED_OPMODE_FLASH)
    {
      /* Average current - brightness */

      /* Flashing through burst mode IDLE state */

      /* Maximum brightness is achieved when average LED current is equalt to
       * LED current limit, and there is no IDLE state
       */

      current_av = (uint16_t)(powerled->param.brightness * current_max
                              / POWERLED_BRIGHTNESS_MAX);

      /* HRTIM clock           = 144000000 Hz
       * HRTIM burst prescaler = 32768,
       * HRTIM burst clock     = 4394 Hz
       */

      burst_pre = HRTIM_BURST_PRESCALER_32768;
      burst_per = (uint16_t)(((float)HRTIM_CLOCK / (1 << burst_pre)) /
                             powerled->param.frequency);
      burst_cmp = (uint16_t)((float)burst_per *
                             ((100 - powerled->param.duty) / 100.0));
    }

  /* Configure DAC buffer */

  for (i = 0; i < CONFIG_STM32_DAC1CH1_DMA_BUFFER_SIZE; i += 1)
    {
      /* TODO: add slope compensation */

      priv->current_tab[i] = current_av ;
    }

  /* Convert current sense value thresholds for DAC */

  for (i = 0; i < CONFIG_STM32_DAC1CH1_DMA_BUFFER_SIZE; i += 1)
    {
      priv->dacbuffer[i] =
        priv->current_tab[i] * DAC_RESOLUTION / DAC_REV_VOLTAGE;
    }

  /* Write DAC buffer */

  DAC_BUFFER_INIT(dac, priv->dacbuffer);

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

  DAC_START(dac);

  /* Configure burst mode */

  HRTIM_BURST_CMP_SET(hrtim, burst_cmp);
  HRTIM_BURST_PER_SET(hrtim, burst_per);
  HRTIM_BURST_PRE_SET(hrtim, burst_pre);

  /* Enable burst mode */

  HRTIM_BURST_ENABLE(hrtim, true);

  /* Start HRTIM PWM */

  HRTIM_OUTPUTS_ENABLE(hrtim, HRTIM_OUT_TIMC_CH1, true);

  /* Set running flag */

  if (priv->running == false)
    {
      priv->running = true;
    }

  return OK;
}

static int powerled_stop(struct powerled_dev_s *dev)
{
  struct powerled_lower_dev_s *lower = dev->lower;
  struct hrtim_dev_s     *hrtim      = lower->hrtim;
  struct powerled_s      *powerled   =
    (struct powerled_s *)dev->priv;
  struct powerled_priv_s *priv =
    (struct powerled_priv_s *)powerled->priv;

  /* Disable output */

  HRTIM_OUTPUTS_ENABLE(hrtim, HRTIM_OUT_TIMC_CH1, false);

  /* Reset running flag */

  priv->running = false;

  return OK;
}

static int powerled_params_set(struct powerled_dev_s *dev,
                               struct powerled_params_s *param)
{
  struct powerled_s *powerled = (struct powerled_s *)dev->priv;
  int ret = OK;

  /* Store params in pirvate struct. */

  powerled->param.brightness = param->brightness;
  powerled->param.frequency  = param->frequency;
  powerled->param.duty       = param->duty;

  return ret;
}

static int powerled_mode_set(struct powerled_dev_s *dev, uint8_t mode)
{
  struct powerled_s *powerled = (struct powerled_s *)dev->priv;
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
          pwrerr("ERROR: unsupported POWERLED mode %d!\n", mode);
          ret = ERROR;
          goto errout;
        }
    }

errout:
  return ret;
}

static int powerled_limits_set(struct powerled_dev_s *dev,
                               struct powerled_limits_s *limits)
{
  struct powerled_s *powerled = (struct powerled_s *)dev->priv;
  int ret = OK;

  /* Some assertions */

  if (limits->current <= 0)
    {
      pwrerr("ERROR: Output current limit must be set!\n");
      ret = ERROR;
      goto errout;
    }

  if (limits->current * 1000 > LED_ABSOLUTE_CURRENT_LIMIT)
    {
      limits->current = (float)LED_ABSOLUTE_CURRENT_LIMIT / 1000.0;
      pwrwarn("WARNING: "
              "LED current limiit > LED absolute current limit."
              " Set current limit to %lf.\n",
              limits->current);
    }

  /* We need only output current */

  powerled->limits.current = limits->current;

  /* Lock limits */

  powerled->limits.lock = true;

errout:
  return ret;
}

static int powerled_state_get(struct powerled_dev_s *dev,
                              struct powerled_state_s *state)
{
  struct powerled_s *powerled = (struct powerled_s *)dev->priv;

  memcpy(state, &powerled->state, sizeof(struct powerled_state_s));

  return OK;
}

static int powerled_fault_set(struct powerled_dev_s *dev, uint8_t fault)
{
  /* Do nothing */

  return -1;
}

static int powerled_fault_get(struct powerled_dev_s *dev,
                              uint8_t *fault)
{
  /* Do nothing */

  return -1;
}

static int powerled_fault_clean(struct powerled_dev_s *dev,
                                uint8_t fault)
{
  /* Do nothing */

  return -1;
}

static int powerled_ioctl(struct powerled_dev_s *dev, int cmd,
                          unsigned long arg)
{
  /* Do nothing */

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_powerled_setup
 *
 * Description:
 *  Initialize POWERLED driver.
 *
 *  This function should be call by board_app_initialize().
 *
 * Returned Value:
 *  0 on success, a negated errno value on failure
 *
 ****************************************************************************/

int stm32_powerled_setup(void)
{
  struct powerled_lower_dev_s *lower       = &g_powerled_lower;
  struct powerled_dev_s       *powerled    = &g_powerled_dev;
  struct hrtim_dev_s          *hrtim       = NULL;
  struct comp_dev_s           *comp        = NULL;
  struct dac_dev_s            *dac         = NULL;
  static bool                  initialized = false;
  int                          ret         = OK;

  /* Initialize only once */

  if (!initialized)
    {
      /* Get the HRTIM interface */

      hrtim = stm32_hrtiminitialize();
      if (hrtim == NULL)
        {
          pwrerr("ERROR: Failed to get HRTIM1 interface\n");
          return -ENODEV;
        }

      /* Get the DAC interface */

      dac = stm32_dacinitialize(DAC_CURRENT_LIMIT);
      if (dac == NULL)
        {
          pwrerr("ERROR: Failed to get DAC %d interface\n",
                 DAC_CURRENT_LIMIT);
          return -ENODEV;
        }

      /* Get the COMP interface */

      comp = stm32_compinitialize(COMP_CURRENT_LIMIT);
      if (comp == NULL)
        {
          pwrerr("ERROR: Failed to get COMP %d interface\n",
                 COMP_CURRENT_LIMIT);
          return -ENODEV;
        }

      /* Initialize POWERLED lower driver interfaces */

      lower->hrtim = hrtim;
      lower->comp  = comp;
      lower->dac   = dac;
      lower->adc   = NULL;
      lower->opamp = NULL;

      /* We do not need register character drivers for POWERLED lower
       * peripherals.
       * All control should be done via POWERLED character driver.
       */

      ret = powerled_register("/dev/powerled0", powerled, (void *)lower);
      if (ret < 0)
        {
          pwrerr("ERROR: powerled_register failed: %d\n", ret);
          return ret;
        }

      initialized = true;
    }

  return ret;
}

#endif /* CONFIG_EXAMPLE_POWERLED && CONFIG_DRIVERS_POWERLED*/
