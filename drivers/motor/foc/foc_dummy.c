/****************************************************************************
 * drivers/motor/foc/foc_dummy.c
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
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/motor/foc/foc_dummy.h>
#include <nuttx/motor/foc/foc_lower.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_MOTOR_FOC
#  error Upper-half FOC driver must be enabled
#endif

/* Board HW configuration */

#define FOC_DUMMY_HW_PWM_NS      (500)
#define FOC_DUMMY_HW_PWM_MAX     (0.95f)

/* Helper macros ************************************************************/

#define FOC_DUMMY_DATA_FROM_DEV_GET(d) \
  ((FAR struct foc_dummy_data_s *)(d)->lower->data)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* SIM FOC board data */

struct foc_dummy_board_s
{
  uint32_t reserved;
};

/* SIM FOC specific data */

struct foc_dummy_data_s
{
  /* Upper-half FOC controller callbacks */

  FAR const struct foc_callbacks_s *cb;

  /* SIM FOC board data */

  FAR struct foc_dummy_board_s *board;

  /* Phase currents */

  foc_current_t current[CONFIG_MOTOR_FOC_PHASES];

#ifdef CONFIG_MOTOR_FOC_BEMF_SENSE
  /* BEMF voltage */

  foc_voltage_t volt[CONFIG_MOTOR_FOC_PHASES];
#endif

  /* FOC worker loop helpers */

  bool     state;
  uint32_t notifier_cntr;
  uint32_t pwm_freq;
  uint32_t notifier_freq;
  uint32_t notifier_div;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Lower-half FOC operations */

static int foc_dummy_configure(FAR struct foc_dev_s *dev,
                               FAR struct foc_cfg_s *cfg);
static int foc_dummy_setup(FAR struct foc_dev_s *dev);
static int foc_dummy_shutdown(FAR struct foc_dev_s *dev);
static int foc_dummy_start(FAR struct foc_dev_s *dev, bool state);
static int foc_dummy_pwm_duty_set(FAR struct foc_dev_s *dev,
                                  FAR foc_duty_t *duty);
static int foc_dummy_ioctl(FAR struct foc_dev_s *dev, int cmd,
                           unsigned long arg);
static int foc_dummy_bind(FAR struct foc_dev_s *dev,
                          FAR struct foc_callbacks_s *cb);
static int foc_dummy_fault_clear(FAR struct foc_dev_s *dev);
#ifdef CONFIG_MOTOR_FOC_TRACE
static void foc_dummy_trace(FAR struct foc_dev_s *dev, int type, bool state);
#endif

/* Handlers */

static void foc_dummy_notifier_handler(FAR struct foc_dev_s *dev);

/* Helpers */

static void foc_dummy_hw_config_get(FAR struct foc_dev_s *dev);
static int foc_dummy_notifier_cfg(FAR struct foc_dev_s *dev, uint32_t freq);
static int foc_dummy_pwm_setup(FAR struct foc_dev_s *dev, uint32_t freq);
static int foc_dummy_pwm_start(FAR struct foc_dev_s *dev, bool state);
static int foc_dummy_adc_setup(FAR struct foc_dev_s *dev);
static int foc_dummy_adc_start(FAR struct foc_dev_s *dev, bool state);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* SIM FOC specific data */

static struct foc_dummy_data_s  g_foc_dummy_data[CONFIG_MOTOR_FOC_INST];
static struct foc_dummy_board_s g_foc_dummy_board[CONFIG_MOTOR_FOC_INST];

/* SIM specific FOC ops */

static struct foc_lower_ops_s g_foc_dummy_ops =
{
  foc_dummy_configure,
  foc_dummy_setup,
  foc_dummy_shutdown,
  foc_dummy_pwm_duty_set,
  foc_dummy_start,
  foc_dummy_ioctl,
  foc_dummy_bind,
  foc_dummy_fault_clear,
#ifdef CONFIG_MOTOR_FOC_TRACE
  foc_dummy_trace
#endif
};

/* FOC lower-half */

static struct foc_lower_s g_foc_dummy_lower[CONFIG_MOTOR_FOC_INST];

/* FOC upper-half device data */

static struct foc_dev_s g_foc_dev[CONFIG_MOTOR_FOC_INST];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: foc_dummy_pwm_setup
 *
 * Description:
 *   Setup PWM for FOC controller
 *
 ****************************************************************************/

static int foc_dummy_pwm_setup(FAR struct foc_dev_s *dev, uint32_t freq)
{
  FAR struct foc_dummy_data_s *sim = FOC_DUMMY_DATA_FROM_DEV_GET(dev);

  DEBUGASSERT(dev);
  DEBUGASSERT(sim);

  mtrinfo("[PWM_SETUP] freq=%d\n", freq);

  DEBUGASSERT(freq > 0);

  /* Store frequency */

  sim->pwm_freq = freq;

  return OK;
}

/****************************************************************************
 * Name: foc_dummy_start
 *
 * Description:
 *   Start/stop PWM for the FOC controller
 *
 ****************************************************************************/

static int foc_dummy_start(FAR struct foc_dev_s *dev, bool state)
{
  FAR struct foc_dummy_data_s *sim = FOC_DUMMY_DATA_FROM_DEV_GET(dev);
  irqstate_t                   flags;
  int                          ret = OK;

  mtrinfo("[FOC_START] state=%d\n", state);

  /* Start PWM */

  ret = foc_dummy_pwm_start(dev, state);
  if (ret < 0)
    {
      mtrerr("foc_dummy_pwm_start failed %d\n", ret);
      goto errout;
    }

  /* Start ADC */

  ret = foc_dummy_adc_start(dev, state);
  if (ret < 0)
    {
      mtrerr("foc_dummy_adc_start failed %d\n", ret);
      goto errout;
    }

  /* Store FOC worker state */

  flags = enter_critical_section();
  sim->state = state;
  leave_critical_section(flags);

errout:
  return ret;
}

/****************************************************************************
 * Name: foc_dummy_pwm_start
 *
 * Description:
 *   Start/stop PWM for the FOC controller
 *
 ****************************************************************************/

static int foc_dummy_pwm_start(FAR struct foc_dev_s *dev, bool state)
{
  DEBUGASSERT(dev);

  mtrinfo("[PWM_START] state=%d\n", state);

  return OK;
}

/****************************************************************************
 * Name: foc_dummy_adc_setup
 *
 * Description:
 *   Setup ADC for the FOC controller
 *
 ****************************************************************************/

static int foc_dummy_adc_setup(FAR struct foc_dev_s *dev)
{
  DEBUGASSERT(dev);

  mtrinfo("[ADC_SETUP]\n");

  return OK;
}

/****************************************************************************
 * Name: foc_dummy_adc_start
 *
 * Description:
 *   Start/stop ADC conversion for the FOC controller
 *
 ****************************************************************************/

static int foc_dummy_adc_start(FAR struct foc_dev_s *dev, bool state)
{
  DEBUGASSERT(dev);

  mtrinfo("[ADC_START] state=%d\n", state);

  return OK;
}

/****************************************************************************
 * Name: foc_dummy_notifier_cfg
 *
 * Description:
 *   Configure FOC notifier
 *
 ****************************************************************************/

static int foc_dummy_notifier_cfg(FAR struct foc_dev_s *dev, uint32_t freq)
{
  FAR struct foc_dummy_data_s *sim = FOC_DUMMY_DATA_FROM_DEV_GET(dev);
  int                          ret = OK;

  DEBUGASSERT(dev);
  DEBUGASSERT(sim);

  mtrinfo("[NOTIFIER_CFG] freq=%d\n", freq);

  DEBUGASSERT(freq > 0);

  if (sim->pwm_freq % freq != 0)
    {
      ret = -EINVAL;
      goto errout;
    }

  /* Call FOC notifier on every update */

  sim->notifier_freq = freq;
  sim->notifier_div  = 1;

errout:
  return ret;
}

/****************************************************************************
 * Name: foc_dummy_configure
 *
 * Description:
 *   Arch-specific FOC controller configuration
 *
 ****************************************************************************/

static int foc_dummy_configure(FAR struct foc_dev_s *dev,
                               FAR struct foc_cfg_s *cfg)
{
  FAR struct foc_dummy_data_s *sim = FOC_DUMMY_DATA_FROM_DEV_GET(dev);
  int                          ret = OK;

  DEBUGASSERT(dev);
  DEBUGASSERT(sim);

  DEBUGASSERT(cfg->pwm_freq > 0);
  DEBUGASSERT(cfg->notifier_freq > 0);

  mtrinfo("[FOC_SETUP]\n");

  /* Configure ADC */

  ret = foc_dummy_adc_setup(dev);
  if (ret < 0)
    {
      mtrerr("foc_dummy_adc_setup failed %d\n", ret);
      goto errout;
    }

  /* Configure PWM */

  ret = foc_dummy_pwm_setup(dev, cfg->pwm_freq);
  if (ret < 0)
    {
      mtrerr("foc_dummy_pwm_setup failed %d\n", ret);
      goto errout;
    }

  /* Configure notifier */

  ret = foc_dummy_notifier_cfg(dev, cfg->notifier_freq);
  if (ret < 0)
    {
      mtrerr("foc_dummy_notifier_cfg failed %d\n", ret);
      goto errout;
    }

  /* Reset some data */

  sim->notifier_cntr = 0;

errout:
  return ret;
}

/****************************************************************************
 * Name: foc_dummy_setup
 *
 * Description:
 *   Arch-specific FOC controller setup
 *
 ****************************************************************************/

static int foc_dummy_setup(FAR struct foc_dev_s *dev)
{
  DEBUGASSERT(dev);

  mtrinfo("[FOC_SETUP]\n");

  /* Get HW configuration */

  foc_dummy_hw_config_get(dev);

  return OK;
}

/****************************************************************************
 * Name: foc_dummy_shutdown
 *
 * Description:
 *   Arch-specific FOC controller shutdown
 *
 ****************************************************************************/

static int foc_dummy_shutdown(FAR struct foc_dev_s *dev)
{
  DEBUGASSERT(dev);

  mtrinfo("[FOC_SHUTDOWN]\n");

  return OK;
}

/****************************************************************************
 * Name: foc_dummy_ioctl
 *
 * Description:
 *   Arch-specific FOC controller ioctl
 *
 ****************************************************************************/

static int foc_dummy_ioctl(FAR struct foc_dev_s *dev, int cmd,
                           unsigned long arg)
{
  int ret = OK;

  DEBUGASSERT(dev);

  mtrinfo("[FOC_IOCTL]cmd=%d\n", cmd);

  switch (cmd)
    {
      default:
        {
          ret = -1;
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: foc_dummy_notifier_handler
 *
 * Description:
 *   Handle ADC conversion and notofiy user-space
 *
 ****************************************************************************/

static void foc_dummy_notifier_handler(FAR struct foc_dev_s *dev)
{
  FAR struct foc_dummy_data_s *sim = FOC_DUMMY_DATA_FROM_DEV_GET(dev);

  DEBUGASSERT(dev);
  DEBUGASSERT(sim);

  mtrinfo("[FOC_NOTIFIER_HANDLER] cntr=%d\n", sim->notifier_cntr);

  /* Call FOC notifier handler */

  if (sim->notifier_cntr % sim->notifier_div == 0)
    {
      /* Call FOC notifier */

#ifdef CONFIG_MOTOR_FOC_BEMF_SENSE
      sim->cb->notifier(dev, sim->current, sim->voltage);
#else
      sim->cb->notifier(dev, sim->current, NULL);
#endif
    }

  /* Increase counter */

  sim->notifier_cntr += 1;
}

/****************************************************************************
 * Name: sim_duty_set
 *
 * Description:
 *   Set 3-phase PWM duty cycle
 *
 ****************************************************************************/

static int foc_dummy_pwm_duty_set(FAR struct foc_dev_s *dev,
                                  FAR foc_duty_t *duty)
{
  int i = 0;

  DEBUGASSERT(dev);
  DEBUGASSERT(duty);

  for (i = 0; i < CONFIG_MOTOR_FOC_PHASES; i += 1)
    {
      DEBUGASSERT(duty[i] >= 0);
    }

  mtrinfo("[PWM_DUTY_SET]\n");

#if CONFIG_MOTOR_FOC_PHASES == 2
  mtrinfo("[%d %d]\n", duty[0], duty[1]);
#elif CONFIG_MOTOR_FOC_PHASES == 3
  mtrinfo("[%d %d %d]\n", duty[0], duty[1], duty[2]);
#elif CONFIG_MOTOR_FOC_PHASES == 4
  mtrinfo("[%d %d %d %d]\n", duty[0], duty[1], duty[2], duty[3]);
#else
#  error
#endif

  return OK;
}

/****************************************************************************
 * Name: foc_dummy_hw_config_get
 *
 * Description:
 *   Get HW configuration for FOC controller
 *
 ****************************************************************************/

static void foc_dummy_hw_config_get(FAR struct foc_dev_s *dev)
{
  DEBUGASSERT(dev);

  /* Get HW configuration */

  dev->info.hw_cfg.pwm_dt_ns = FOC_DUMMY_HW_PWM_NS;
  dev->info.hw_cfg.pwm_max   = ftob16(FOC_DUMMY_HW_PWM_MAX);
}

/****************************************************************************
 * Name: foc_dummy_bind
 *
 * Description:
 *   Bind lower-half FOC controller with upper-half FOC logic
 *
 ****************************************************************************/

static int foc_dummy_bind(FAR struct foc_dev_s *dev,
                          FAR struct foc_callbacks_s *cb)
{
  FAR struct foc_dummy_data_s *sim = FOC_DUMMY_DATA_FROM_DEV_GET(dev);
  int                          ret = OK;

  DEBUGASSERT(dev);
  DEBUGASSERT(cb);
  DEBUGASSERT(sim);

  mtrinfo("[FOC_BIND]\n");

  /* Bind upper-half FOC controller callbacks */

  sim->cb = cb;
  return ret;
}

/****************************************************************************
 * Name: foc_dummy_fault_clear
 *
 * Description:
 *   Arch-specific fault clear
 *
 ****************************************************************************/

static int foc_dummy_fault_clear(FAR struct foc_dev_s *dev)
{
  DEBUGASSERT(dev);

  mtrinfo("[FAULT_CLEAR]\n");

  return OK;
}

#ifdef CONFIG_MOTOR_FOC_TRACE
/****************************************************************************
 * Name: foc_dummy_trace
 *
 * Description:
 *   SIM FOC trace
 *
 ****************************************************************************/

static void foc_dummy_trace(FAR struct foc_dev_s *dev, int type, bool state)
{
  DEBUGASSERT(dev);

  mtrinfo("[FOC_TRACE] type=%d state=%d\n", type, state);
}
#endif  /* CONFIG_MOTOR_FOC_TRACE */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: foc_dummy_initialize
 *
 * Description:
 *   Initialize the FOC controller lower-half support.
 *
 ****************************************************************************/

FAR struct foc_dev_s *foc_dummy_initialize(int inst)
{
  FAR struct foc_lower_s *foc_lower = NULL;
  FAR struct foc_dev_s   *dev       = NULL;

  mtrinfo("[FOC_INITIALIZE] inst=%d\n", inst);

  /* Reset data */

  memset(&g_foc_dummy_data[inst], 0, sizeof(struct foc_dummy_data_s));

  /* Get SIM FOC arch-specific driver */

  foc_lower = &g_foc_dummy_lower[inst];

  /* Connect ops, data and dev with arch-specific FOC driver */

  foc_lower->data = &g_foc_dummy_data[inst];
  foc_lower->ops  = &g_foc_dummy_ops;

  /* Connect board data */

  g_foc_dummy_data[inst].board = &g_foc_dummy_board[inst];

  /* Get FOC device */

  dev = &g_foc_dev[inst];

  /* Connect lower half FOC with FOC device */

  dev->lower = (FAR void *)foc_lower;

  /* Return lower-half driver instance */

  return dev;
}

/****************************************************************************
 * Name: foc_dummy_update
 *
 * Description:
 *   Called periodically from the IDLE loop to simulate FOC driver interrupts
 *
 ****************************************************************************/

void foc_dummy_update(void)
{
  FAR struct foc_dev_s        *dev  = NULL;
  FAR struct foc_dummy_data_s *sim  = NULL;
  int                          i    = 0;
  irqstate_t                   flags;

  flags = enter_critical_section();

  /* Update all FOC instances */

  for (i = 0; i < CONFIG_MOTOR_FOC_INST; i += 1)
    {
      /* Get FOC device */

      dev = &g_foc_dev[i];

      /* Get SIM data */

      sim = FOC_DUMMY_DATA_FROM_DEV_GET(dev);

      if (sim->state == true)
        {
          foc_dummy_notifier_handler(dev);
        }
    }

  leave_critical_section(flags);
}
