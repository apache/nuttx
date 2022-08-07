/****************************************************************************
 * boards/arm/stm32/b-g431b-esc1/src/stm32_foc.c
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

#include "hardware/stm32g4xxxx_opamp.h"

#if defined(CONFIG_SENSORS_QENCODER) || defined(CONFIG_SENSORS_HALL3PHASE)
#  include "hardware/stm32g4xxxx_pwr.h"
#endif

#include "stm32_foc.h"

#ifdef CONFIG_SENSORS_QENCODER
#  include "stm32_qencoder.h"
#endif

#include "arm_internal.h"
#include "b-g431b-esc1.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* We don't use phase 2 feedback as it is no connected to ADC1 */

#if CONFIG_MOTOR_FOC_SHUNTS != 2
#  error Only 2-shunts configuration is supported
#endif

/* Configuration specific for L6387ED:
 *   1. PWM channels must have positive polarity
 *   2. PWM complementary channels must have positive polarity
 */

#ifndef CONFIG_STM32_FOC_HAS_PWM_COMPLEMENTARY
#  error
#endif

#if CONFIG_STM32_TIM1_CH1POL != 0
#  error
#endif
#if CONFIG_STM32_TIM1_CH2POL != 0
#  error
#endif
#if CONFIG_STM32_TIM1_CH3POL != 0
#  error
#endif
#if CONFIG_STM32_TIM1_CH1NPOL != 0
#  error
#endif
#if CONFIG_STM32_TIM1_CH2NPOL != 0
#  error
#endif
#if CONFIG_STM32_TIM1_CH3NPOL != 0
#  error
#endif

/* SYSCFG must be enabled for OPAMP */

#ifndef CONFIG_STM32_SYSCFG
#  error
#endif

/* Aux ADC needs DMA enabled and workaround for G4 ADC CHAN0 enabled */

#ifdef CONFIG_ADC
#  ifndef CONFIG_STM32_ADC1_DMA
#    error
#  endif
#  ifndef CONFIG_STM32_FOC_G4_ADCCHAN0_WORKAROUND
#    error
#  endif
#endif

/* REVISIT:  */

#define PWM_DEADTIME    (20)
#define PWM_DEADTIME_NS (500)

/* Devpath for FOC driver */

#define FOC_DEVPATH "/dev/foc0"

/* Board parameters:
 *   Current shunt resistance                    = 0.003
 *   PGA gain                                    = 16
 *   Current sense gain                          = -9.14 (inverted current)
 *   Vbus sense gain                             = 0.0962
 *   Vbus min                                    = 7V
 *   Vbus max                                    = 25V (6S LiPo battery pack)
 *   Iout max                                    = 40A peak
 *   IPHASE_RATIO = 1/(R_shunt*gain)             = -36.47
 *   ADC_REF_VOLTAGE                             = 3.3
 *   ADC_VAL_MAX                                 = 4095
 *   ADC_TO_VOLT = ADC_REF_VOLTAGE / ADC_VAL_MAX
 *   IPHASE_ADC = IPHASE_RATIO * ADC_TO_VOLT     = -0.02939
 *   VBUS_RATIO = 1/VBUS_gain                    = 10.4
 */

/* OPAMP gain */

#define CURRENT_PGA_GAIN 16

/* Center-aligned PWM duty cycle limits */

#define MAX_DUTY_B16 ftob16(0.95f)

/* ADC sample time */

#define CURRENT_SAMPLE_TIME ADC_SMPR_2p5
#define VBUS_SAMPLE_TIME    ADC_SMPR_640p5
#define POT_SAMPLE_TIME     ADC_SMPR_640p5

/* ADC1 channels used in this example */

#define ADC1_INJECTED  (CONFIG_MOTOR_FOC_SHUNTS)

#ifdef CONFIG_BOARD_STM32_BG431BESC1_FOC_VBUS
#  define BG431BESC1_FOC_VBUS 1
#else
#  define BG431BESC1_FOC_VBUS 0
#endif

#ifdef CONFIG_BOARD_STM32_BG431BESC1_FOC_POT
#  define BG431BESC1_FOC_POT 1
#else
#  define BG431BESC1_FOC_POT 0
#endif

#define ADC1_REGULAR   (BG431BESC1_FOC_VBUS + BG431BESC1_FOC_POT)
#define ADC1_NCHANNELS (ADC1_INJECTED + ADC1_REGULAR)

/* Check ADC1 configuration */

#ifdef CONFIG_STM32_FOC_G4_ADCCHAN0_WORKAROUND
#  if ADC1_INJECTED != (CONFIG_STM32_ADC1_INJECTED_CHAN - 1)
#    error
#  endif
#else
#  if ADC1_INJECTED != CONFIG_STM32_ADC1_INJECTED_CHAN
#    error
#  endif
#endif

#if CONFIG_STM32_ADC1_RESOLUTION != 0
#  error
#endif

/* Qenco configuration - only TIM4 */

#ifdef CONFIG_SENSORS_QENCODER
#  ifndef CONFIG_STM32_TIM4_QE
#    error
#  endif
#  if CONFIG_STM32_TIM4_QEPSC != 0
#    error
#  endif
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

/* OPAMP configuration:
 *   - connected with ADC through output pin (OPAINTOEN=0)
 *   - Current U+ - OPAMP1_VINP0 (PA1)
 *   - Current U- - OPAMP1_VINP0 (PA3)
 *   - Current V+ - OPAMP2_VINP0 (PA7)
 *   - Current V- - OPAMP2_VINP0 (PA5)
 *   - Current W+ - OPAMP3_VINP0 (PB0)
 *   - Current W- - OPAMP3_VINP0 (PB2)
 *
 * ADC configuration:
 *    - Current Phase V    -> ADC1 INJ1 -> ADC1_IN3 (OPAMP1_VOUT/PA2)
 *    - Current Phase U    -> Not used, no ADC1 connection
 *    - Current Phase W    -> ADC1 INJ2 -> ADC1_IN12 (OPAMP3_VOUT/PB12)
 *    optional:
 *    - VBUS               -> ADC1 REG  -> ADC1_IN1 (PA0)
 *    - POT                -> ADC1 REG  -> ADC1_IN11 (PB12)
 *
 * TIM1 PWM configuration:
 *    - Phase U high -> TIM1_CH1  (PA8)
 *    - Phase U low  -> TIM1_CH1N (PC13)
 *    - Phase V high -> TIM1_CH2  (PA9)
 *    - Phase V low  -> TIM1_CH2N (PA12)
 *    - Phase W high -> TIM1_CH3  (PA10)
 *    - Phase W low  -> TIM1_CH3N (PB15)
 */

static uint8_t g_adc1_chan[] =
{
#ifdef CONFIG_BOARD_STM32_BG431BESC1_FOC_VBUS
  1,                            /* ADC1 REG - VBUS */
#endif
#ifdef CONFIG_BOARD_STM32_BG431BESC1_FOC_POT
  11,                           /* ADC1 REG - POT */
#endif
  3,                            /* ADC1 INJ1 - PHASE 1 */
  12,                           /* ADC1 INJ2 - PHASE 3 */
};

static uint32_t g_adc1_pins[] =
{
#ifdef CONFIG_BOARD_STM32_BG431BESC1_FOC_VBUS
  GPIO_ADC1_IN1,
#endif
#ifdef CONFIG_BOARD_STM32_BG431BESC1_FOC_POT
  GPIO_ADC1_IN11,
#endif
  GPIO_ADC1_IN3,
  GPIO_ADC1_IN12,
};

/* ADC1 sample time configuration */

static adc_channel_t g_adc1_stime[] =
{
#ifdef CONFIG_BOARD_STM32_BG431BESC1_FOC_VBUS
  {
    .channel     = 1,
    .sample_time = VBUS_SAMPLE_TIME
  },
#endif
#ifdef CONFIG_BOARD_STM32_BG431BESC1_FOC_POT
  {
    .channel     = 11,
    .sample_time = POT_SAMPLE_TIME
  },
#endif
  {
    .channel     = 3,
    .sample_time = CURRENT_SAMPLE_TIME
  },
  {
    .channel     = 12,
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
  uint32_t regval = 0;

  DEBUGASSERT(dev);

  UNUSED(dev);

  /* OPAMP1/2/3 pins:
   *   OPAMP1_VINM - PA3 (VINM0)
   *   OPAMP1_VINP - PA1 (VINP0)
   *   OPAMP2_VINM - PA5 (VINM0)
   *   OPAMP2_VINP - PA7 (VINP0)
   *   OPAMP3_VINM - PB2 (VINM0)
   *   OPAMP3_VINP - PB0 (VINP0)
   */

  /* Configure GPIO */

  stm32_configgpio(GPIO_OPAMP1_VINM0);
  stm32_configgpio(GPIO_OPAMP1_VINP0);
  stm32_configgpio(GPIO_OPAMP1_VOUT);
  stm32_configgpio(GPIO_OPAMP2_VINM0);
  stm32_configgpio(GPIO_OPAMP2_VINP0);
  stm32_configgpio(GPIO_OPAMP2_VOUT);
  stm32_configgpio(GPIO_OPAMP3_VINM0);
  stm32_configgpio(GPIO_OPAMP3_VINP0);
  stm32_configgpio(GPIO_OPAMP3_VOUT);

  /* Configure OPAMP inputs */

  regval += (OPAMP_CSR_VPSEL_VINP0 | OPAMP_CSR_VMSEL_PGA);

  /* PGA mode, non-inverting configuration with external bias on VINM0 */

#if CURRENT_PGA_GAIN == 16
  regval += ((0b01011 << OPAMP_CSR_PGAGAIN_SHIFT) & OPAMP_CSR_PGAGAIN_MASK);
#else
#  error Not supported
#endif

  /* Enable high-speed mode */

  regval += OPAMP_CSR_OPAHSM;

  /* Write configuration */

  putreg32(regval, STM32_OPAMP1_CSR);
  putreg32(regval, STM32_OPAMP2_CSR);
  putreg32(regval, STM32_OPAMP3_CSR);

  /* Enable OPAMPs in separate write */

  regval += OPAMP_CSR_OPAMPEN;

  putreg32(regval, STM32_OPAMP1_CSR);
  putreg32(regval, STM32_OPAMP2_CSR);
  putreg32(regval, STM32_OPAMP3_CSR);

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
  curr[2] = curr_raw[1];

  /* Phase 2 reconstruction */

  curr[1] = -(curr_raw[0] + curr_raw[1]);

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
#if defined(CONFIG_SENSORS_QENCODER) || defined(CONFIG_SENSORS_HALL3PHASE)
      /* Disable USB Type-C and Power Delivery Dead Battery */

      modifyreg32(STM32_PWR_CR3, 0, PWR_CR3_UCPD1_DBDIS);
#endif

#if defined(CONFIG_SENSORS_QENCODER) && defined(CONFIG_STM32_QENCODER_INDEX_PIN)
      /* Configure encoder index GPIO */

      ret = stm32_qe_index_init(4, QENCODER_TIM4_INDEX_GPIO);
      if (ret < 0)
        {
          mtrerr("Failed to register encoder index pin %d\n", ret);
          ret = -EACCES;
          goto errout;
        }
#endif

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
