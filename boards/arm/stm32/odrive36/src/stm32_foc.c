/****************************************************************************
 * boards/arm/stm32/odrive36/src/stm32_foc.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <nuttx/mutex.h>
#include <nuttx/signal.h>
#include <nuttx/analog/adc.h>

#include <nuttx/motor/foc/drv8301.h>
#include <nuttx/motor/motor_ioctl.h>

#include "stm32_foc.h"
#include "stm32_gpio.h"
#ifdef CONFIG_ADC
#  include "stm32_adc.h"
#endif

#include "odrive.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_ODRIVE_HW_VOLTAGE_56
#  error Tested only for ODrive 56V version
#endif

/* Supported FOC instances */

#ifdef CONFIG_ODRIVE_FOC_FOC0
#  define ODRIVE_FOC_FOC0 1
#else
#  define ODRIVE_FOC_FOC0 0
#endif

#ifdef CONFIG_ODRIVE_FOC_FOC1
#  define ODRIVE_FOC_FOC1 1
#else
#  define ODRIVE_FOC_FOC1 0
#endif

#define ODRIVE_FOC_INST (ODRIVE_FOC_FOC0 + ODRIVE_FOC_FOC1)

#ifdef CONFIG_ODRIVE_FOC_FOC0
#  define ODRIVE32_FOC0_DEVPATH "/dev/foc0"
#  define ODRIVE32_FOC0_INST    (0)
#endif

#ifdef CONFIG_ODRIVE_FOC_FOC1
#  define ODRIVE32_FOC1_DEVPATH "/dev/foc1"
#  define ODRIVE32_FOC1_INST     (1)
#endif

/* Must match upper-half configuration */

#if ODRIVE_FOC_INST != CONFIG_MOTOR_FOC_INST
#  error Invalid configuration
#endif

/* Only 2-shunt configuration supported by board */

#if CONFIG_MOTOR_FOC_SHUNTS != 2
#  error For now ony 2-shunts configuration is supported
#endif

/* Configuration specific for DRV8301:
 *   1. PWM channels must have positive polarity
 *   2. PWM complementary channels must have positive polarity
 */

#ifndef CONFIG_STM32_FOC_HAS_PWM_COMPLEMENTARY
#  error
#endif

#ifdef CONFIG_ODRIVE_FOC_FOC0

#  if CONFIG_STM32_TIM1_CH1POL != 0
#    error
#  endif
#  if CONFIG_STM32_TIM1_CH2POL != 0
#    error
#  endif
#  if CONFIG_STM32_TIM1_CH3POL != 0
#    error
#  endif
#  if CONFIG_STM32_TIM1_CH1NPOL != 0
#    error
#  endif
#  if CONFIG_STM32_TIM1_CH2NPOL != 0
#    error
#  endif
#  if CONFIG_STM32_TIM1_CH3NPOL != 0
#    error
#  endif

/* FOC0 uses ADC2 */

#  ifndef CONFIG_STM32_FOC_FOC0_ADC2
#    error
#  endif

#  if CONFIG_STM32_ADC2_RESOLUTION != 0
#    error
#  endif

#endif  /* CONFIG_ODRIVE_FOC_FOC0 */

#ifdef CONFIG_ODRIVE_FOC_FOC1

#  if CONFIG_STM32_TIM8_CH1POL != 0
#    error
#  endif
#  if CONFIG_STM32_TIM8_CH2POL != 0
#    error
#  endif
#  if CONFIG_STM32_TIM8_CH3POL != 0
#    error
#  endif
#  if CONFIG_STM32_TIM8_CH1NPOL != 0
#    error
#  endif
#  if CONFIG_STM32_TIM8_CH2NPOL != 0
#    error
#  endif
#  if CONFIG_STM32_TIM8_CH3NPOL != 0
#    error
#  endif

/* FOC1 uses ADC3 */

#  ifndef CONFIG_STM32_FOC_FOC1_ADC3
#    error
#  endif

#  if CONFIG_STM32_ADC3_RESOLUTION != 0
#    error
#  endif

#endif  /* CONFIG_ODRIVE_FOC_FOC1 */

/* Aux ADC needs DMA enabled */

#ifdef CONFIG_ADC
#  ifndef CONFIG_STM32_ADC1_DMA
#    error
#  endif
#  ifndef CONFIG_STM32_ADC1_SCAN
#    error
#  endif
#endif

/* TODO: */

#define PWM_DEADTIME    (50)
#define PWM_DEADTIME_NS (320)

/* Board parameters:
 *   Current shunt resistance                    = 0.0005
 *   Current sense gain                          = (10/20/40/80)
 *   Vbus min                                    = 12V
 *   Vbus max                                    = 24V or 56V
 *   Iout max                                    = 40A (no cooling for
 *                                                      MOSFETs)
 *   IPHASE_RATIO                                = 1/(R_shunt*gain)
 *   ADC_REF_VOLTAGE                             = 3.3
 *   ADC_VAL_MAX                                 = 4095
 *   ADC_TO_VOLT = ADC_REF_VOLTAGE / ADC_VAL_MAX
 *   IPHASE_ADC = IPHASE_RATIO * ADC_TO_VOLT     = 0.02014 (gain=80)
 *   VBUS_RATIO = 1/VBUS_gain                    = 11 or 19
 */

#define ADC_VOLT_REF    3300000 /* micro volt */
#define ADC_VAL_MAX     4095
#define R_SHUNT         500     /* micro ohm */

/* Center-aligned PWM duty cycle limits */

#define MAX_DUTY_B16 ftob16(0.95f)

/* ADC configuration */

#define CURRENT_SAMPLE_TIME    ADC_SMPR_3
#define VBUS_SAMPLE_TIME       ADC_SMPR_15
#define TEMP_SAMPLE_TIME       ADC_SMPR_15

#define ODRIVE_ADC_AUX  (1)
#define ODRIVE_ADC_FOC0 (2)
#define ODRIVE_ADC_FOC1 (3)

#ifdef CONFIG_ODRIVE_FOC_VBUS
#  define ODRIVE_FOC_VBUS 1
#else
#  define ODRIVE_FOC_VBUS 0
#endif
#ifdef CONFIG_ODRIVE_FOC_TEMP
#  define ODRIVE_FOC_TEMP 3
#else
#  define ODRIVE_FOC_TEMP 0
#endif

#ifdef CONFIG_ADC
#  define ODRIVE_ADC_AUX_DEVPATH "/dev/adc0"
#  define ODRIVE_ADC_AUX_NCHAN   (ODRIVE_FOC_VBUS + ODRIVE_FOC_TEMP)
#endif

#define ADC1_INJECTED  (0)
#define ADC1_REGULAR   (0)
#define ADC1_NCHANNELS (ADC1_INJECTED + ADC1_REGULAR)

#define ADC2_INJECTED  (CONFIG_MOTOR_FOC_SHUNTS)
#define ADC2_REGULAR   (0)
#define ADC2_NCHANNELS (ADC2_INJECTED + ADC2_REGULAR)

#define ADC3_INJECTED  (CONFIG_MOTOR_FOC_SHUNTS)
#define ADC3_REGULAR   (0)
#define ADC3_NCHANNELS (ADC3_INJECTED + ADC3_REGULAR)

#if ADC1_INJECTED != CONFIG_STM32_ADC1_INJECTED_CHAN
#  error
#endif

#if ADC2_INJECTED != CONFIG_STM32_ADC2_INJECTED_CHAN
#  error
#endif

#if ADC3_INJECTED != CONFIG_STM32_ADC3_INJECTED_CHAN
#  error
#endif

/* DRV8301 configuration */

#ifndef CONFIG_STM32_SPI3
#  error
#endif

#define DRV8301_0_SPI (3)
#define DRV8301_1_SPI (3)

#define DRV8301_FREQUENCY (500000)

/* Qenco configuration */

#ifdef CONFIG_SENSORS_QENCODER
#  ifndef CONFIG_STM32_QENCODER_DISABLE_EXTEND16BTIMERS
#    error Invalid configuration
#  endif
#  ifndef CONFIG_STM32_QENCODER_INDEX_PIN
#    error Invalid configuration
#  endif
#  ifdef CONFIG_STM32_TIM3_QE
#    if CONFIG_STM32_TIM3_QEPSC != 0
#      error Invalid TIM3 QEPSC value
#    endif
#  endif
#  ifdef CONFIG_STM32_TIM4_QE
#    if CONFIG_STM32_TIM4_QEPSC != 0
#      error Invalid TIM4 QEPSC value
#    endif
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
static int board_foc_current_get(struct foc_dev_s *dev, int16_t *curr_raw,
                                 foc_current_t *curr);
static int board_foc_info_get(struct foc_dev_s *dev,
                              struct foc_info_s *info);
static int board_foc_ioctl(struct foc_dev_s *dev, int cmd,
                           unsigned long arg);
#ifdef CONFIG_MOTOR_FOC_TRACE
static int board_foc_trace_init(struct foc_dev_s *dev);
static void board_foc_trace(struct foc_dev_s *dev, int type, bool state);
#endif

static int stm32_foc_drv8301_fault_attach(struct focpwr_dev_s *dev,
                                          xcpt_t isr, void *arg);
static int stm32_foc_drv8301_gate_enable(struct focpwr_dev_s *dev,
                                         bool enable);
static void stm32_foc_drv8301_fault_handle(struct focpwr_dev_s *dev);

static int stm32_focdev_setup(int devno, int spino,
                              struct stm32_foc_board_s *board);

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
  .info_get    = board_foc_info_get,
  .ioctl       = board_foc_ioctl,
#ifdef CONFIG_MOTOR_FOC_TRACE
  .trace_init  = board_foc_trace_init,
  .trace       = board_foc_trace
#endif
};

/* Board specific ADC configuration
 *
 * AUX (only VBUS used):
 *    VBUS     - ADC1 - ADC1_IN6  (PA6)
 *    M0_TEMP  - ADC1 - ADC1_IN15 (PC5)
 *    M1_TEMP  - ADC1 - ADC1_IN4  (PA4)
 *    AUX_TEMP - ADC1 - ADC1_IN5  (PA5)
 *
 * FOC device 0:
 *   Phase 1 - ADC2 - ADC2_IN10   (PC0)
 *   Phase 2 - ADC2 - ADC2_IN11   (PC1)
 *
 * FOC device 1:
 *   Phase 1 - ADC3 - ADC3_IN13   (PC3)
 *   Phase 2 - ADC3 - ADC3_IN12   (PC2)
 *
 */

#ifdef CONFIG_ADC

/* AUX ADC configuration */

static uint8_t g_adc_aux_chan[] =
{
#ifdef CONFIG_ODRIVE_FOC_VBUS
  6,
#endif
#ifdef ODRIVE_ADC_TEMP
  15,
  4,
  5
#endif
};

static uint32_t g_adc_aux_pins[] =
{
#ifdef CONFIG_ODRIVE_FOC_VBUS
  GPIO_ADC1_IN6,
#endif
#ifdef ODRIVE_ADC_TEMP
  GPIO_ADC1_IN15,
  GPIO_ADC1_IN4,
  GPIO_ADC1_IN5
#endif
};

static adc_channel_t g_adc_aux_stime[] =
{
#ifdef CONFIG_ODRIVE_FOC_VBUS
  {
    .channel     = 6,
    .sample_time = VBUS_SAMPLE_TIME
  },
#endif
#ifdef ODRIVE_ADC_TEMP
  {
    .channel     = 15,
    .sample_time = TEMP_SAMPLE_TIME
  },
  {
    .channel     = 4,
    .sample_time = TEMP_SAMPLE_TIME
  },
  {
    .channel     = 5,
    .sample_time = TEMP_SAMPLE_TIME
  }
#endif
};
#endif

#ifdef CONFIG_ODRIVE_FOC_FOC0
/* Board specific ADC configuration for FOC device 0 */

static uint8_t g_adc_foc0_chan[] =
{
  10,
  11
};

static uint32_t g_adc_foc0_pins[] =
{
  GPIO_ADC2_IN10,
  GPIO_ADC2_IN11,
};

static adc_channel_t g_adc_foc0_stime[] =
{
  {
    .channel     = 10,
    .sample_time = CURRENT_SAMPLE_TIME
  },
  {
    .channel     = 11,
    .sample_time = CURRENT_SAMPLE_TIME
  }
};

static struct stm32_foc_adc_s g_adc_foc0_cfg =
{
  .chan  = g_adc_foc0_chan,
  .pins  = g_adc_foc0_pins,
  .stime = g_adc_foc0_stime,
  .nchan = ADC2_NCHANNELS,
  .regch = ADC2_REGULAR,
  .intf  = ODRIVE_ADC_FOC0
};
#endif

#ifdef CONFIG_ODRIVE_FOC_FOC1
/* Board specific ADC configuration for FOC device 1 */

static uint8_t g_adc_foc1_chan[] =
{
  13,
  12
};

static uint32_t g_adc_foc1_pins[] =
{
  GPIO_ADC3_IN13,
  GPIO_ADC3_IN12,
};

static adc_channel_t g_adc_foc1_stime[] =
{
  {
    .channel     = 13,
    .sample_time = CURRENT_SAMPLE_TIME
  },
  {
    .channel     = 12,
    .sample_time = CURRENT_SAMPLE_TIME
  }
};

static struct stm32_foc_adc_s g_adc_foc1_cfg =
{
  .chan  = g_adc_foc1_chan,
  .pins  = g_adc_foc1_pins,
  .stime = g_adc_foc1_stime,
  .nchan = ADC3_NCHANNELS,
  .regch = ADC3_REGULAR,
  .intf  = ODRIVE_ADC_FOC1
};
#endif

#ifdef CONFIG_ODRIVE_FOC_FOC0
/* Board specific data - FOC 0 */

static struct stm32_foc_board_data_s g_stm32_foc0_board_data =
{
  .adc_cfg = &g_adc_foc0_cfg,
  .pwm_dt  = (PWM_DEADTIME),
};

/* Board specific configuration */

static struct stm32_foc_board_s g_stm32_foc0_board =
{
  .data = &g_stm32_foc0_board_data,
  .ops  = &g_stm32_foc_board_ops,
};
#endif

#ifdef CONFIG_ODRIVE_FOC_FOC1
/* Board specific data - FOC 1 */

static struct stm32_foc_board_data_s g_stm32_foc1_board_data =
{
  .adc_cfg = &g_adc_foc1_cfg,
  .pwm_dt  = (PWM_DEADTIME),
};

/* Board specific configuration */

static struct stm32_foc_board_s g_stm32_foc1_board =
{
  .data = &g_stm32_foc1_board_data,
  .ops  = &g_stm32_foc_board_ops,
};
#endif

/* DRV8301 board ops */

static struct drv8301_ops_s g_drv8301_board_ops =
{
  .fault_attach = stm32_foc_drv8301_fault_attach,
  .gate_enable  = stm32_foc_drv8301_gate_enable,
  .fault_handle = stm32_foc_drv8301_fault_handle
};

/* Global data */

static mutex_t g_common_lock = NXMUTEX_INITIALIZER;
static bool g_fault_attached = false;
static bool g_gate_enabled   = false;

static struct foc_dev_s *g_foc_dev[2] =
{
  NULL,
  NULL
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_foc_setup
 ****************************************************************************/

static int board_foc_setup(struct foc_dev_s *dev)
{
  DEBUGASSERT(dev);
  DEBUGASSERT(dev->pwr);

  return dev->pwr->ops->setup(dev->pwr);
}

/****************************************************************************
 * Name: board_foc_shutdown
 ****************************************************************************/

static int board_foc_shutdown(struct foc_dev_s *dev)
{
  DEBUGASSERT(dev);
  DEBUGASSERT(dev->pwr);

  return dev->pwr->ops->shutdown(dev->pwr);
}

/****************************************************************************
 * Name: board_foc_calibration
 ****************************************************************************/

static int board_foc_calibration(struct foc_dev_s *dev, bool state)
{
  DEBUGASSERT(dev);
  DEBUGASSERT(dev->pwr);

  return dev->pwr->ops->calibration(dev->pwr, state);
}

/****************************************************************************
 * Name: board_foc_fault_clear
 ****************************************************************************/

static int board_foc_fault_clear(struct foc_dev_s *dev)
{
  DEBUGASSERT(dev);

  UNUSED(dev);

  /* TODO: clear DRV8301 faults */

  return OK;
}

/****************************************************************************
 * Name: board_foc_pwm_start
 ****************************************************************************/

static int board_foc_pwm_start(struct foc_dev_s *dev, bool state)
{
  DEBUGASSERT(dev);

  return OK;
}

/****************************************************************************
 * Name: board_foc_current_get
 ****************************************************************************/

static int board_foc_current_get(struct foc_dev_s *dev, int16_t *curr_raw,
                                 foc_current_t *curr)
{
  DEBUGASSERT(dev);
  DEBUGASSERT(curr_raw);
  DEBUGASSERT(curr);

  /* Get currents */

  curr[1] = curr_raw[0];
  curr[2] = curr_raw[1];

  /* From Kirchhoff's current law: ia = -(ib + ic) */

  curr[0] = -(curr[1] + curr[2]);

  return OK;
}

/****************************************************************************
 * Name: board_foc_info_get
 ****************************************************************************/

static int board_foc_info_get(struct foc_dev_s *dev,
                              struct foc_info_s *info)
{
  struct foc_get_boardcfg_s cfg;

  DEBUGASSERT(dev);
  DEBUGASSERT(info);

  UNUSED(dev);

  /* PWM */

  info->hw_cfg.pwm_dt_ns = PWM_DEADTIME_NS;
  info->hw_cfg.pwm_max   = MAX_DUTY_B16;

  /* Get power stage configuration */

  board_foc_ioctl(dev, MTRIOC_GET_BOARDCFG, (unsigned long)&cfg);

  /* ADC Current */

  info->hw_cfg.iphase_max   = 40000;

  info->hw_cfg.iphase_scale = ((100000ul * (ADC_VOLT_REF / ADC_VAL_MAX)) /
                               (cfg.gain * R_SHUNT));

  return OK;
}

/****************************************************************************
 * Name: board_foc_ioctl
 ****************************************************************************/

static int board_foc_ioctl(struct foc_dev_s *dev, int cmd, unsigned long arg)
{
  DEBUGASSERT(dev);
  DEBUGASSERT(dev->pwr);

  return dev->pwr->ops->ioctl(dev->pwr, cmd, arg);
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
 * Name: stm32_foc_drv8301_fault_attach
 ****************************************************************************/

static int stm32_foc_drv8301_fault_attach(struct focpwr_dev_s *dev,
                                          xcpt_t isr, void *arg)
{
  int ret = OK;

  nxmutex_lock(&g_common_lock);

  /* nFAULT is common for both FOC instances */

  if (g_fault_attached != (bool) isr)
    {
      ret = stm32_gpiosetevent(GPIO_DRV8301_NFAULT, false, true, false,
                               isr, arg);

      g_fault_attached = (bool) isr;
    }

  nxmutex_unlock(&g_common_lock);

  return ret;
}

/****************************************************************************
 * Name: stm32_foc_drv8301_gate_enable
 ****************************************************************************/

static int stm32_foc_drv8301_gate_enable(struct focpwr_dev_s *dev,
                                         bool enable)
{
  /* ENGATE is common for both FOC instances */

  nxmutex_lock(&g_common_lock);

  if (enable != g_gate_enabled)
    {
      stm32_gpiowrite(GPIO_DRV8301_ENGATE, enable);

      g_gate_enabled = enable;
    }

  nxmutex_unlock(&g_common_lock);

  return OK;
}

/****************************************************************************
 * Name: stm32_foc_drv8301_fault_handle
 ****************************************************************************/

static void stm32_foc_drv8301_fault_handle(struct focpwr_dev_s *dev)
{
  UNUSED(dev);

  /* Set fault state for both instances */

#ifdef CONFIG_ODRIVE_FOC_FOC0
  g_foc_dev[0]->state.fault |= FOC_FAULT_BOARD;
#endif

#ifdef CONFIG_ODRIVE_FOC_FOC1
  g_foc_dev[1]->state.fault |= FOC_FAULT_BOARD;
#endif

  /* Disable gates for both instances */

  stm32_gpiowrite(GPIO_DRV8301_ENGATE, false);
}

/****************************************************************************
 * Name: stm32_focdev_setup
 ****************************************************************************/

static int stm32_focdev_setup(int devno, int spino,
                              struct stm32_foc_board_s *board)
{
  struct drv8301_cfg_s    drv8301_cfg;
  struct drv8301_board_s  drv8301_board;
  struct spi_dev_s       *spi = NULL;
  struct foc_dev_s       *foc = NULL;
  int                     ret = OK;
  char                    devpath[20];

  /* Initialize arch specific FOC 0 lower-half */

  foc = stm32_foc_initialize(devno, board);
  if (foc == NULL)
    {
      ret = -errno;
      mtrerr("Failed to initialize STM32 FOC: %d\n", ret);
      goto errout;
    }

  DEBUGASSERT(foc->lower);

  /* Get devpath */

  snprintf(devpath, sizeof(devpath), "/dev/foc%d", devno);

  /* Get SPI device */

  spi = stm32_spibus_initialize(spino);
  if (spi == NULL)
    {
      ret = -errno;
      goto errout;
    }

  /* DRV8301 configuration */

  drv8301_cfg.freq      = DRV8301_FREQUENCY;
  drv8301_cfg.gate_curr = DRV8301_GATECURR_1p7;
  drv8301_cfg.gain      = DRV8301_GAIN_80;
  drv8301_cfg.pwm_mode  = DRV8301_PWM_6IN;
  drv8301_cfg.oc_adj    = DRV8301_OCADJ_DEFAULT;

  /* DRV8301 board data */

  drv8301_board.spi   = spi;
  drv8301_board.ops   = &g_drv8301_board_ops;
  drv8301_board.cfg   = &drv8301_cfg;
  drv8301_board.devno = devno;

  /* Register DRV8301 device */

  ret = drv8301_register(devpath, foc, &drv8301_board);
  if (ret < 0)
    {
      mtrerr("Failed to register drv8301 device: %d\n", ret);
      goto errout;
    }

errout:
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_foc_setup
 *
 * Description:
 *   Setup FOC devices
 *
 *   This function should be call by board_app_initialize().
 *
 * Returned Value:
 *   0 on success, a negated errno value on failure
 *
 ****************************************************************************/

int stm32_foc_setup(void)
{
  int ret = OK;

  /* Configure common EN_GATE */

  stm32_configgpio(GPIO_DRV8301_ENGATE);

#ifdef CONFIG_ODRIVE_FOC_FOC0
  ret = stm32_focdev_setup(0, DRV8301_0_SPI, &g_stm32_foc0_board);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_focdev_setup 0 failed: %d\n", ret);
      goto errout;
    }
#endif

#ifdef CONFIG_ODRIVE_FOC_FOC1
  ret = stm32_focdev_setup(1, DRV8301_1_SPI, &g_stm32_foc1_board);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_focdev_setup 1 failed: %d\n", ret);
      goto errout;
    }
#endif

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
  struct adc_dev_s         *adc       = NULL;
  struct stm32_adc_dev_s   *stm32_adc = NULL;
  struct adc_sample_time_s  stime;
  int                       i         = 0;
  int                       ret       = OK;

  /* Configure pins */

  for (i = 0; i < ODRIVE_ADC_AUX_NCHAN; i += 1)
    {
      stm32_configgpio(g_adc_aux_pins[i]);
    }

  /* Initialize ADC */

  adc = stm32_adcinitialize(ODRIVE_ADC_AUX, g_adc_aux_chan,
                            ODRIVE_ADC_AUX_NCHAN);
  if (adc == NULL)
    {
      aerr("ERROR: Failed to get ADC interface %d\n", ODRIVE_ADC_AUX);
      ret = -ENODEV;
      goto errout;
    }

  /* Regsiter ADC */

  ret = adc_register(ODRIVE_ADC_AUX_DEVPATH, adc);
  if (ret < 0)
    {
      aerr("ERROR: adc_register %s failed: %d\n",
           ODRIVE_ADC_AUX_DEVPATH, ret);
      goto errout;
    }

  /* Get lower-half ADC */

  stm32_adc = (struct stm32_adc_dev_s *)adc->ad_priv;
  DEBUGASSERT(stm32_adc);

  /* Configure ADC sample time */

  memset(&stime, 0, sizeof(struct adc_sample_time_s));

  stime.channels_nbr = ODRIVE_ADC_AUX_NCHAN;
  stime.channel      = g_adc_aux_stime;

  STM32_ADC_SAMPLETIME_SET(stm32_adc, &stime);
  STM32_ADC_SAMPLETIME_WRITE(stm32_adc);

  ret = OK;

errout:
  return ret;
}
#endif
