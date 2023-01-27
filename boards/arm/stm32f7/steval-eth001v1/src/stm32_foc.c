/****************************************************************************
 * boards/arm/stm32f7/steval-eth001v1/src/stm32_foc.c
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

#include "stm32_gpio.h"
#include "stm32_foc.h"

#include "steval-eth001v1.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 3-shunts configuration supported */

#if CONFIG_MOTOR_FOC_SHUNTS != 3
#  error Only 3-shunts configuration is supported
#endif

/* Configuration specific for STDRIVE101:
 *   1. PWM channels must have positive polarity
 *   2. PWM complementary channels must have positive polarity
 */

#ifndef CONFIG_STM32F7_FOC_HAS_PWM_COMPLEMENTARY
#  error
#endif

#if CONFIG_STM32F7_TIM1_CH1POL != 0
#  error
#endif
#if CONFIG_STM32F7_TIM1_CH2POL != 0
#  error
#endif
#if CONFIG_STM32F7_TIM1_CH3POL != 0
#  error
#endif
#if CONFIG_STM32F7_TIM1_CH1NPOL != 0
#  error
#endif
#if CONFIG_STM32F7_TIM1_CH2NPOL != 0
#  error
#endif
#if CONFIG_STM32F7_TIM1_CH3NPOL != 0
#  error
#endif

/* Aux ADC needs DMA enabled  */

#ifdef CONFIG_ADC
#  ifndef CONFIG_STM32F7_ADC1_DMA
#    error
#  endif
#endif

/* STDRIVE101 in INHx/INLx mode - no deadtime generation */

#define PWM_DEADTIME    (54)
#define PWM_DEADTIME_NS (500)

/* Devpath for FOC driver */

#define FOC_DEVPATH "/dev/foc0"

/* Board parameters:
 *   Current shunt resistance                    = 0.003
 *   Current sense gain                          = -11.74 (inverted current)
 *   Vbus sense gain                             = 0.0522
 *   Vbus min                                    = 20V
 *   Vbus max                                    = 48V
 *   Iout max                                    = ? (max 700W)
 *   IPHASE_RATIO = 1/(R_shunt*gain)             = -28.39
 *   ADC_REF_VOLTAGE                             = 3.3
 *   ADC_VAL_MAX                                 = 4095
 *   ADC_TO_VOLT = ADC_REF_VOLTAGE / ADC_VAL_MAX
 *   IPHASE_ADC = IPHASE_RATIO * ADC_TO_VOLT     = -0.02287
 *   VBUS_RATIO = 1/VBUS_gain                    = 19.1570881226
 */

/* Center-aligned PWM duty cycle limits */

#define MAX_DUTY_B16 ftob16(0.95f)

/* ADC sample time */

#define CURRENT_SAMPLE_TIME ADC_SMPR_3
#define VBUS_SAMPLE_TIME    ADC_SMPR_480

/* ADC1 channels used in this example */

#define ADC1_INJECTED  (CONFIG_MOTOR_FOC_SHUNTS)

#ifdef CONFIG_BOARD_STM32F7_STEVALETH001V1_FOC_VBUS
#  define STEVALETH001V1_FOC_VBUS 1
#else
#  define STEVALETH001V1_FOC_VBUS 0
#endif

#define ADC1_REGULAR   (STEVALETH001V1_FOC_VBUS)
#define ADC1_NCHANNELS (ADC1_INJECTED + ADC1_REGULAR)

/* Check ADC1 configuration */

#if ADC1_INJECTED != CONFIG_STM32F7_ADC1_INJECTED_CHAN
#  error
#endif

#if CONFIG_STM32F7_ADC1_RESOLUTION != 0
#  error
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

static int board_foc_setup(struct foc_dev_s *dev);
static int board_foc_shutdown(struct foc_dev_s *dev);
static int board_foc_calibration(struct foc_dev_s *dev, bool state);
static int board_foc_fault_clear(struct foc_dev_s *dev);
static int board_foc_pwm_start(struct foc_dev_s *dev, bool state);
static int board_foc_current_get(struct foc_dev_s *dev,
                                 int16_t *curr_raw,
                                 foc_current_t *curr);
#ifdef CONFIG_MOTOR_FOC_TRACE
static int board_foc_trace_init(struct foc_dev_s *dev);
static void board_foc_trace(struct foc_dev_s *dev, int type, bool state);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* ADC configuration:
 *    - Current Phase V    -> ADC1 INJ1 -> ADC1_IN15 (PC5)
 *    - Current Phase U    -> ADC1 INJ2 -> ADC1_IN11 (PC1)
 *    - Current Phase W    -> ADC1 INJ3 -> ADC1_IN10 (PC0)
 *    optional:
 *    - VBUS               -> ADC1 REG  -> ADC1_IN14 (PC4)
 *
 * TIM1 PWM configuration:
 *    - Phase U high -> TIM1_CH1  (PA8)
 *    - Phase U low  -> TIM1_CH1N (PC13)
 *    - Phase V high -> TIM1_CH2  (PA9)
 *    - Phase V low  -> TIM1_CH2N (PB0)
 *    - Phase W high -> TIM1_CH3  (PA10)
 *    - Phase W low  -> TIM1_CH3N (PB1)
 */

static uint8_t g_adc1_chan[] =
{
#ifdef CONFIG_BOARD_STM32F7_STEVALETH001V1_FOC_VBUS
  14,                           /* ADC1 REG - VBUS */
#endif
  15,                           /* ADC1 INJ1 - PHASE 1 */
  11,                           /* ADC1 INJ2 - PHASE 2 */
  10,                           /* ADC1 INJ3 - PHASE 3 */
};

static uint32_t g_adc1_pins[] =
{
#ifdef CONFIG_BOARD_STM32F7_STEVALETH001V1_FOC_VBUS
  GPIO_ADC1_IN14,
#endif
  GPIO_ADC1_IN15,
  GPIO_ADC1_IN11,
  GPIO_ADC1_IN10,
};

/* ADC1 sample time configuration */

static adc_channel_t g_adc1_stime[] =
{
#ifdef CONFIG_BOARD_STM32F7_STEVALETH001V1_FOC_VBUS
  {
    .channel     = 14,
    .sample_time = VBUS_SAMPLE_TIME
  },
#endif
  {
    .channel     = 15,
    .sample_time = CURRENT_SAMPLE_TIME
  },
  {
    .channel     = 11,
    .sample_time = CURRENT_SAMPLE_TIME
  },
  {
    .channel     = 10,
    .sample_time = CURRENT_SAMPLE_TIME
  },
};

/* Board specific ADC configuration for FOC */

static struct stm32_foc_adc_s g_adc_cfg =
{
  .chan  = g_adc1_chan,
  .pins  = g_adc1_pins,
  .stime = g_adc1_stime,
  .nchan = ADC1_NCHANNELS,
  .regch = ADC1_REGULAR,
  .intf  = 1
};

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
  .adc_cfg   = &g_adc_cfg,
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

static struct foc_dev_s *g_foc_dev = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_foc_setup
 ****************************************************************************/

static int board_foc_setup(struct foc_dev_s *dev)
{
  DEBUGASSERT(dev);

  UNUSED(dev);

  return OK;
}

/****************************************************************************
 * Name: board_foc_shutdown
 ****************************************************************************/

static int board_foc_shutdown(struct foc_dev_s *dev)
{
  DEBUGASSERT(dev);

  UNUSED(dev);

  return OK;
}

/****************************************************************************
 * Name: board_foc_calibration
 ****************************************************************************/

static int board_foc_calibration(struct foc_dev_s *dev, bool state)
{
  DEBUGASSERT(dev);

  UNUSED(dev);

  return OK;
}

/****************************************************************************
 * Name: board_foc_fault_clear
 ****************************************************************************/

static int board_foc_fault_clear(struct foc_dev_s *dev)
{
  DEBUGASSERT(dev);

  UNUSED(dev);

  return OK;
}

/****************************************************************************
 * Name: board_foc_pwm_start
 ****************************************************************************/

static int board_foc_pwm_start(struct foc_dev_s *dev, bool state)
{
  DEBUGASSERT(dev);

  UNUSED(dev);

  return OK;
}

/****************************************************************************
 * Name: board_foc_current_get
 ****************************************************************************/

static int board_foc_current_get(struct foc_dev_s *dev,
                                 int16_t *curr_raw,
                                 foc_current_t *curr)
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

static int board_foc_trace_init(struct foc_dev_s *dev)
{
  DEBUGASSERT(dev);

  UNUSED(dev);

  /* Not supported */

  return -1;
}

/****************************************************************************
 * Name: board_foc_trace
 ****************************************************************************/

static void board_foc_trace(struct foc_dev_s *dev, int type, bool state)
{
  DEBUGASSERT(dev);

  UNUSED(dev);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_foc_setup
 *
 * Description:
 *   Initialize FOC driver.
 *
 *   This function should be call by board_app_initialize().
 *
 * Returned Value:
 *   0 on success, a negated errno value on failure
 *
 ****************************************************************************/

int stm32_foc_setup(void)
{
  struct foc_dev_s *foc = NULL;
  int               ret = OK;

  /* Initialize only once */

  if (g_foc_dev == NULL)
    {
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
  struct adc_dev_s *adc         = NULL;
  int               ret         = OK;
  static bool       initialized = false;

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
