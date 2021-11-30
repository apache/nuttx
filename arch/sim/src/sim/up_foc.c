/****************************************************************************
 * arch/sim/src/sim/up_foc.c
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

#include <nuttx/motor/foc/foc_lower.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_MOTOR_FOC
#  error Upper-half FOC driver must be enabled
#endif

/* Board HW configuration */

#define SIM_FOC_HW_PWM_NS      (500)
#define SIM_FOC_HW_PWM_MAX     (0.95f)

/* Helper macros ************************************************************/

#define SIM_FOC_DATA_FROM_DEV_GET(d)                                     \
  ((FAR struct sim_foc_data_s *)(d)->lower->data)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* SIM FOC board data */

struct sim_foc_board_s
{
  uint32_t reserved;
};

/* SIM FOC specific data */

struct sim_foc_data_s
{
  /* Upper-half FOC controller callbacks */

  FAR const struct foc_callbacks_s *cb;

  /* SIM FOC board data */

  FAR struct sim_foc_board_s *board;

  /* Phase currents */

  foc_current_t current[CONFIG_MOTOR_FOC_PHASES];

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

static int sim_foc_configure(FAR struct foc_dev_s *dev,
                             FAR struct foc_cfg_s *cfg);
static int sim_foc_setup(FAR struct foc_dev_s *dev);
static int sim_foc_shutdown(FAR struct foc_dev_s *dev);
static int sim_foc_start(FAR struct foc_dev_s *dev, bool state);
static int sim_foc_pwm_duty_set(FAR struct foc_dev_s *dev,
                                FAR foc_duty_t *duty);
static int sim_foc_ioctl(FAR struct foc_dev_s *dev, int cmd,
                         unsigned long arg);
static int sim_foc_bind(FAR struct foc_dev_s *dev,
                        FAR struct foc_callbacks_s *cb);
static int sim_foc_fault_clear(FAR struct foc_dev_s *dev);
#ifdef CONFIG_MOTOR_FOC_TRACE
static void sim_foc_trace(FAR struct foc_dev_s *dev, int type, bool state);
#endif

/* Handlers */

static int sim_foc_notifier_handler(FAR struct foc_dev_s *dev);

/* Helpers */

static void sim_foc_hw_config_get(FAR struct foc_dev_s *dev);
static int sim_foc_notifier_cfg(FAR struct foc_dev_s *dev, uint32_t freq);
static int sim_foc_pwm_setup(FAR struct foc_dev_s *dev, uint32_t freq);
static int sim_foc_pwm_start(FAR struct foc_dev_s *dev, bool state);
static int sim_foc_adc_setup(FAR struct foc_dev_s *dev);
static int sim_foc_adc_start(FAR struct foc_dev_s *dev, bool state);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* SIM FOC specific data */

static struct sim_foc_data_s  g_sim_foc_data[CONFIG_MOTOR_FOC_INST];
static struct sim_foc_board_s g_sim_foc_board[CONFIG_MOTOR_FOC_INST];

/* SIM specific FOC ops */

static struct foc_lower_ops_s g_sim_foc_ops =
{
  .configure      = sim_foc_configure,
  .setup          = sim_foc_setup,
  .shutdown       = sim_foc_shutdown,
  .start          = sim_foc_start,
  .pwm_duty_set   = sim_foc_pwm_duty_set,
  .ioctl          = sim_foc_ioctl,
  .bind           = sim_foc_bind,
  .fault_clear    = sim_foc_fault_clear,
#ifdef CONFIG_MOTOR_FOC_TRACE
  .trace          = sim_foc_trace
#endif
};

/* FOC lower-half */

static struct foc_lower_s g_sim_foc_lower[CONFIG_MOTOR_FOC_INST];

/* FOC upper-half device data */

static struct foc_dev_s g_foc_dev[CONFIG_MOTOR_FOC_INST];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_foc_pwm_setup
 *
 * Description:
 *   Setup PWM for FOC controller
 *
 ****************************************************************************/

static int sim_foc_pwm_setup(FAR struct foc_dev_s *dev, uint32_t freq)
{
  FAR struct sim_foc_data_s *sim = SIM_FOC_DATA_FROM_DEV_GET(dev);

  DEBUGASSERT(dev);
  DEBUGASSERT(sim);

  mtrinfo("[PWM_SETUP] freq=%d\n", freq);

  DEBUGASSERT(freq > 0);

  /* Store frequency */

  sim->pwm_freq = freq;

  return OK;
}

/****************************************************************************
 * Name: sim_foc_start
 *
 * Description:
 *   Start/stop PWM for the FOC controller
 *
 ****************************************************************************/

static int sim_foc_start(FAR struct foc_dev_s *dev, bool state)
{
  FAR struct sim_foc_data_s *sim = SIM_FOC_DATA_FROM_DEV_GET(dev);
  irqstate_t                 flags;
  int                        ret = OK;

  mtrinfo("[FOC_START] state=%d\n", state);

  /* Start PWM */

  ret = sim_foc_pwm_start(dev, state);
  if (ret < 0)
    {
      mtrerr("sim_foc_pwm_start failed %d\n", ret);
      goto errout;
    }

  /* Start ADC */

  ret = sim_foc_adc_start(dev, state);
  if (ret < 0)
    {
      mtrerr("sim_foc_adc_start failed %d\n", ret);
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
 * Name: sim_foc_pwm_start
 *
 * Description:
 *   Start/stop PWM for the FOC controller
 *
 ****************************************************************************/

static int sim_foc_pwm_start(FAR struct foc_dev_s *dev, bool state)
{
  DEBUGASSERT(dev);

  mtrinfo("[PWM_START] state=%d\n", state);

  return OK;
}

/****************************************************************************
 * Name: sim_foc_adc_setup
 *
 * Description:
 *   Setup ADC for the FOC controller
 *
 ****************************************************************************/

static int sim_foc_adc_setup(FAR struct foc_dev_s *dev)
{
  DEBUGASSERT(dev);

  mtrinfo("[ADC_SETUP]\n");

  return OK;
}

/****************************************************************************
 * Name: sim_foc_adc_start
 *
 * Description:
 *   Start/stop ADC conversion for the FOC controller
 *
 ****************************************************************************/

static int sim_foc_adc_start(FAR struct foc_dev_s *dev, bool state)
{
  DEBUGASSERT(dev);

  mtrinfo("[ADC_START] state=%d\n", state);

  return OK;
}

/****************************************************************************
 * Name: sim_foc_notifier_cfg
 *
 * Description:
 *   Configure FOC notifier
 *
 ****************************************************************************/

static int sim_foc_notifier_cfg(FAR struct foc_dev_s *dev, uint32_t freq)
{
  FAR struct sim_foc_data_s *sim = SIM_FOC_DATA_FROM_DEV_GET(dev);
  int                        ret = OK;

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
 * Name: sim_foc_configure
 *
 * Description:
 *   Arch-specific FOC controller configuration
 *
 ****************************************************************************/

static int sim_foc_configure(FAR struct foc_dev_s *dev,
                             FAR struct foc_cfg_s *cfg)
{
  FAR struct sim_foc_data_s *sim = SIM_FOC_DATA_FROM_DEV_GET(dev);
  int                        ret = OK;

  DEBUGASSERT(dev);
  DEBUGASSERT(sim);

  DEBUGASSERT(cfg->pwm_freq > 0);
  DEBUGASSERT(cfg->notifier_freq > 0);

  mtrinfo("[FOC_SETUP]\n");

  /* Configure ADC */

  ret = sim_foc_adc_setup(dev);
  if (ret < 0)
    {
      mtrerr("sim_foc_adc_setup failed %d\n", ret);
      goto errout;
    }

  /* Configure PWM */

  ret = sim_foc_pwm_setup(dev, cfg->pwm_freq);
  if (ret < 0)
    {
      mtrerr("sim_foc_pwm_setup failed %d\n", ret);
      goto errout;
    }

  /* Configure notifier */

  ret = sim_foc_notifier_cfg(dev, cfg->notifier_freq);
  if (ret < 0)
    {
      mtrerr("sim_foc_notifier_cfg failed %d\n", ret);
      goto errout;
    }

  /* Reset some data */

  sim->notifier_cntr = 0;

errout:
  return ret;
}

/****************************************************************************
 * Name: sim_foc_setup
 *
 * Description:
 *   Arch-specific FOC controller setup
 *
 ****************************************************************************/

static int sim_foc_setup(FAR struct foc_dev_s *dev)
{
  DEBUGASSERT(dev);

  mtrinfo("[FOC_SETUP]\n");

  /* Get HW configuration */

  sim_foc_hw_config_get(dev);

  return OK;
}

/****************************************************************************
 * Name: sim_foc_shutdown
 *
 * Description:
 *   Arch-specific FOC controller shutdown
 *
 ****************************************************************************/

static int sim_foc_shutdown(FAR struct foc_dev_s *dev)
{
  DEBUGASSERT(dev);

  mtrinfo("[FOC_SHUTDOWN]\n");

  return OK;
}

/****************************************************************************
 * Name: sim_foc_ioctl
 *
 * Description:
 *   Arch-specific FOC controller ioctl
 *
 ****************************************************************************/

static int sim_foc_ioctl(FAR struct foc_dev_s *dev, int cmd,
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
 * Name: sim_foc_notifier_handler
 *
 * Description:
 *   Handle ADC conversion and notofiy user-space
 *
 ****************************************************************************/

static int sim_foc_notifier_handler(FAR struct foc_dev_s *dev)
{
  FAR struct sim_foc_data_s *sim = SIM_FOC_DATA_FROM_DEV_GET(dev);
  irqstate_t                 flags;

  DEBUGASSERT(dev);
  DEBUGASSERT(sim);

  mtrinfo("[FOC_NOTIFIER_HANDLER] cntr=%d\n", sim->notifier_cntr);

  flags = enter_critical_section();

  /* Call FOC notifier handler */

  if (sim->notifier_cntr % sim->notifier_div == 0)
    {
      /* Call FOC notifier */

      sim->cb->notifier(dev, sim->current);
    }

  /* Increase counter */

  sim->notifier_cntr += 1;

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: sim_duty_set
 *
 * Description:
 *   Set 3-phase PWM duty cycle
 *
 ****************************************************************************/

static int sim_foc_pwm_duty_set(FAR struct foc_dev_s *dev,
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
 * Name: sim_foc_hw_config_get
 *
 * Description:
 *   Get HW configuration for FOC controller
 *
 ****************************************************************************/

static void sim_foc_hw_config_get(FAR struct foc_dev_s *dev)
{
  DEBUGASSERT(dev);

  /* Get HW configuration */

  dev->info.hw_cfg.pwm_dt_ns = SIM_FOC_HW_PWM_NS;
  dev->info.hw_cfg.pwm_max   = ftob16(SIM_FOC_HW_PWM_MAX);
}

/****************************************************************************
 * Name: sim_foc_bind
 *
 * Description:
 *   Bind lower-half FOC controller with upper-half FOC logic
 *
 ****************************************************************************/

static int sim_foc_bind(FAR struct foc_dev_s *dev,
                        FAR struct foc_callbacks_s *cb)
{
  FAR struct sim_foc_data_s *sim = SIM_FOC_DATA_FROM_DEV_GET(dev);
  int                        ret = OK;

  DEBUGASSERT(dev);
  DEBUGASSERT(cb);
  DEBUGASSERT(sim);

  mtrinfo("[FOC_BIND]\n");

  /* Bind upper-half FOC controller callbacks */

  sim->cb = cb;
  return ret;
}

/****************************************************************************
 * Name: sim_foc_fault_clear
 *
 * Description:
 *   Arch-specific fault clear
 *
 ****************************************************************************/

static int sim_foc_fault_clear(FAR struct foc_dev_s *dev)
{
  DEBUGASSERT(dev);

  mtrinfo("[FAULT_CLEAR]\n");

  return OK;
}

#ifdef CONFIG_MOTOR_FOC_TRACE
/****************************************************************************
 * Name: sim_foc_trace
 *
 * Description:
 *   SIM FOC trace
 *
 ****************************************************************************/

static void sim_foc_trace(FAR struct foc_dev_s *dev, int type, bool state)
{
  DEBUGASSERT(dev);

  mtrinfo("[FOC_TRACE] type=%d state=%d\n", type, state);
}
#endif  /* CONFIG_MOTOR_FOC_TRACE */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_foc_initialize
 *
 * Description:
 *   Initialize the FOC controller lower-half support.
 *
 ****************************************************************************/

FAR struct foc_dev_s *sim_foc_initialize(int inst)
{
  FAR struct foc_lower_s *foc_lower = NULL;
  FAR struct foc_dev_s   *dev       = NULL;

  mtrinfo("[FOC_INITIALIZE] inst=%d\n", inst);

  /* Reset data */

  memset(&g_sim_foc_data[inst], 0, sizeof(struct sim_foc_data_s));

  /* Get SIM FOC arch-specific driver */

  foc_lower = &g_sim_foc_lower[inst];

  /* Connect ops, data and dev with arch-specific FOC driver */

  foc_lower->data  = &g_sim_foc_data[inst];
  foc_lower->ops   = &g_sim_foc_ops;

  /* Connect board data */

  g_sim_foc_data[inst].board = &g_sim_foc_board[inst];

  /* Get FOC device */

  dev = &g_foc_dev[inst];

  /* Connect lower half FOC with FOC devic */

  dev->lower = (FAR void *)foc_lower;

  /* Return lower-half driver instance */

  return dev;
}

/****************************************************************************
 * Name: sim_foc_update
 *
 * Description:
 *   Called periodically from the IDLE loop to simulate FOC driver interrupts
 *
 ****************************************************************************/

void sim_foc_update(void)
{
  FAR struct foc_dev_s      *dev  = NULL;
  FAR struct sim_foc_data_s *sim  = NULL;
  static uint32_t            cntr = 0;
  int                        i    = 0;
  irqstate_t                 flags;

  /* Increase local counter */

  cntr += 1;

  flags = enter_critical_section();

  /* Update all FOC instances */

  for (i = 0; i < CONFIG_MOTOR_FOC_INST; i += 1)
    {
      /* Get FOC device */

      dev = &g_foc_dev[i];

      /* Get SIM data */

      sim = SIM_FOC_DATA_FROM_DEV_GET(dev);

      if (sim->state == true)
        {
          sim_foc_notifier_handler(dev);
        }
    }

  leave_critical_section(flags);

  return;
}
