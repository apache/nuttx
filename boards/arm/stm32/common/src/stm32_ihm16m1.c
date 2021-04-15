/****************************************************************************
 * boards/arm/stm32/common/src/stm32_ihm16m1.c
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
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>

#include <nuttx/analog/adc.h>

#include "stm32_foc.h"
#include "stm32_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if CONFIG_MOTOR_FOC_SHUNTS != 3
#  error For now ony 3-shunts configuration is supported
#endif

/* Configuration specific for STSPIN830:
 * 1. PWM enable on high
 * 2. PWM channels must have positive polarity
 * 3. EN_FAULT enable on high, configured as open drain output
 */

#if CONFIG_STM32_TIM1_CH1POL != 0
#  error
#endif
#if CONFIG_STM32_TIM1_CH2POL != 0
#  error
#endif
#if CONFIG_STM32_TIM1_CH3POL != 0
#  error
#endif

/* Aux ADC needs DMA enabled */

#ifdef CONFIG_ADC
#  ifndef CONFIG_STM32_ADC1_DMA
#    error
#  endif
#endif

/* REVISIT: dead-time STSPIN830 */

#define PWM_DEADTIME    (20)
#define PWM_DEADTIME_NS (500)

/* Devpath for FOC driver */

#define FOC_DEVPATH "/dev/foc0"

/* Board parameters:
 *   Current shunt resistance                    = 0.33
 *   Current sense gain                          = -1.53 (inverted current)
 *   Vbus sense gain                             = 0.0522
 *   Vbus min                                    = 7V
 *   Vbus max                                    = 45V
 *   Iout max                                    = 1.5A RMS
 *   IPHASE_RATIO = 1/(R_shunt*gain)             = -1.98
 *   ADC_REF_VOLTAGE                             = 3.3
 *   ADC_VAL_MAX                                 = 4095
 *   ADC_TO_VOLT = ADC_REF_VOLTAGE / ADC_VAL_MAX
 *   IPHASE_ADC = IPHASE_RATIO * ADC_TO_VOLT     = -0.00160
 *   VBUS_RATIO = 1/VBUS_gain                    = 16
 */

/* Center-aligned PWM duty cycle limits */

#define MAX_DUTY_B16 ftob16(0.95f)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

static int board_foc_setup(FAR struct foc_dev_s *dev);
static int board_foc_shutdown(FAR struct foc_dev_s *dev);
static int board_foc_calibration(FAR struct foc_dev_s *dev, bool state);
static int board_foc_fault_clear(FAR struct foc_dev_s *dev);
static int board_foc_pwm_start(FAR struct foc_dev_s *dev, bool state);
static int board_foc_current_get(FAR struct foc_dev_s *dev,
                                 FAR int16_t *curr_raw,
                                 FAR foc_current_t *curr);
#ifdef CONFIG_MOTOR_FOC_TRACE
static int board_foc_trace_init(FAR struct foc_dev_s *dev);
static void board_foc_trace(FAR struct foc_dev_s *dev, int type, bool state);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Board specific ops */

static struct stm32_foc_board_ops_s g_stm32_foc_board_ops =
{
  .setup       = board_foc_setup,
  .shutdown    = board_foc_shutdown,
  .calibration = board_foc_calibration,
  .fault_clear = board_foc_fault_clear,
  .pwm_start   = board_foc_pwm_start,
  .current_get = board_foc_current_get,
#ifdef CONFIG_MOTOR_FOC_TRACE
  .trace_init  = board_foc_trace_init,
  .trace       = board_foc_trace
#endif
};

/* Board specific data */

static struct stm32_foc_board_data_s g_stm32_foc_board_data =
{
  .adc_cfg   = NULL,     /* board-specific */
  .duty_max  = (MAX_DUTY_B16),
  .pwm_dt    = (PWM_DEADTIME),
  .pwm_dt_ns = (PWM_DEADTIME_NS)
};

/* Board specific configuration */

static struct stm32_foc_board_s g_stm32_foc_board =
{
  .data = &g_stm32_foc_board_data,
  .ops  = &g_stm32_foc_board_ops,
};

/* Global pointer to the upper FOC driver */

static FAR struct foc_dev_s *g_foc_dev = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_foc_setup
 ****************************************************************************/

static int board_foc_setup(FAR struct foc_dev_s *dev)
{
  DEBUGASSERT(dev);

  UNUSED(dev);

  /* Configure EN_U, EN_V, EN_W, EN_FAULT GPIOs */

  stm32_configgpio(GPIO_FOC_EN_U);
  stm32_configgpio(GPIO_FOC_EN_V);
  stm32_configgpio(GPIO_FOC_EN_W);

  stm32_configgpio(GPIO_FOC_ENFAULT);

  return OK;
}

/****************************************************************************
 * Name: board_foc_shutdown
 ****************************************************************************/

static int board_foc_shutdown(FAR struct foc_dev_s *dev)
{
  DEBUGASSERT(dev);

  UNUSED(dev);

  return OK;
}

/****************************************************************************
 * Name: board_foc_calibration
 ****************************************************************************/

static int board_foc_calibration(FAR struct foc_dev_s *dev, bool state)
{
  DEBUGASSERT(dev);

  UNUSED(dev);

  return OK;
}

/****************************************************************************
 * Name: board_foc_fault_clear
 ****************************************************************************/

static int board_foc_fault_clear(FAR struct foc_dev_s *dev)
{
  DEBUGASSERT(dev);

  UNUSED(dev);

  return OK;
}

/****************************************************************************
 * Name: board_foc_pwm_start
 ****************************************************************************/

static int board_foc_pwm_start(FAR struct foc_dev_s *dev, bool state)
{
  DEBUGASSERT(dev);

  UNUSED(dev);

  /* Set/reset ENABLE input */

  stm32_gpiowrite(GPIO_FOC_ENFAULT, state);

  /* Enable/disable UVW output */

  stm32_gpiowrite(GPIO_FOC_EN_U, state);
  stm32_gpiowrite(GPIO_FOC_EN_V, state);
  stm32_gpiowrite(GPIO_FOC_EN_W, state);

  return OK;
}

/****************************************************************************
 * Name: board_foc_current_get
 ****************************************************************************/

static int board_foc_current_get(FAR struct foc_dev_s *dev,
                                 FAR int16_t *curr_raw,
                                 FAR foc_current_t *curr)
{
  DEBUGASSERT(dev);
  DEBUGASSERT(curr_raw);
  DEBUGASSERT(curr);

  /* Get currents */

  curr[0] = curr_raw[0];
  curr[1] = curr_raw[1];
  curr[2] = curr_raw[2];

  return OK;
}

#ifdef CONFIG_MOTOR_FOC_TRACE
/****************************************************************************
 * Name: board_foc_trace_init
 ****************************************************************************/

static int board_foc_trace_init(FAR struct foc_dev_s *dev)
{
  DEBUGASSERT(dev);

  UNUSED(dev);

  /* Configure debug GPIO */

  stm32_configgpio(GPIO_FOC_DEBUG0);
  stm32_configgpio(GPIO_FOC_DEBUG1);
  stm32_configgpio(GPIO_FOC_DEBUG2);
  stm32_configgpio(GPIO_FOC_DEBUG3);

  return OK;
}

/****************************************************************************
 * Name: board_foc_trace
 ****************************************************************************/

static void board_foc_trace(FAR struct foc_dev_s *dev, int type, bool state)
{
  DEBUGASSERT(dev);

  UNUSED(dev);

  switch (type)
    {
      case FOC_TRACE_NONE:
        {
          break;
        }

      case FOC_TRACE_PARAMS:
        {
          stm32_gpiowrite(GPIO_FOC_DEBUG0, state);

          break;
        }

      case FOC_TRACE_STATE:
        {
          stm32_gpiowrite(GPIO_FOC_DEBUG1, state);

          break;
        }

      case FOC_TRACE_NOTIFIER:
        {
          stm32_gpiowrite(GPIO_FOC_DEBUG2, state);

          break;
        }

      case FOC_TRACE_LOWER:
        {
          stm32_gpiowrite(GPIO_FOC_DEBUG3, state);

          break;
        }

      default:
        {
          mtrerr("board_foc_trace type=%d not supported!\n", type);
          DEBUGASSERT(0);
        }
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_ihm16m1_initialize
 ****************************************************************************/

int board_ihm16m1_initialize(FAR struct stm32_foc_adc_s *adc_cfg)
{
  FAR struct foc_dev_s *foc = NULL;
  int                   ret = OK;

  DEBUGASSERT(adc_cfg);

  /* Initialize only once */

  if (g_foc_dev == NULL)
    {
      /* Connect ADC configuration */

      g_stm32_foc_board_data.adc_cfg = adc_cfg;

      /* Initialize arch specific FOC lower-half */

      foc = stm32_foc_initialize(0, &g_stm32_foc_board);
      if (foc == NULL)
        {
          ret = -errno;
          mtrerr("Failed to initialize STM32 FOC: %d\n", ret);
          goto errout;
        }

      DEBUGASSERT(foc->lower);

      /* Register FOC device */

      ret = foc_register(FOC_DEVPATH, foc);
      if (ret < 0)
        {
          mtrerr("Failed to register FOC device: %d\n", ret);
          goto errout;
        }

      /* Store pointer to driver */

      g_foc_dev = foc;
    }

errout:
  return ret;
}

#ifdef CONFIG_ADC
/****************************************************************************
 * Name: stm32_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

int stm32_adc_setup(void)
{
  FAR struct adc_dev_s *adc         = NULL;
  int                   ret         = OK;
  static bool           initialized = false;

  /* Initialize only once */

  if (initialized == false)
    {
      if (g_foc_dev == NULL)
        {
          mtrerr("Failed to get g_foc_dev device\n");
          ret = -EACCES;
          goto errout;
        }

      /* Register regular channel ADC */

      adc = stm32_foc_adcget(g_foc_dev);
      if (adc == NULL)
        {
          mtrerr("Failed to get ADC device: %d\n", ret);
          goto errout;
        }

      ret = adc_register("/dev/adc0", adc);
      if (ret < 0)
        {
          mtrerr("adc_register failed: %d\n", ret);
          goto errout;
        }

      initialized = true;
    }

errout:
  return ret;
}
#endif
