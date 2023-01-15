/****************************************************************************
 * include/nuttx/motor/foc/foc_lower.h
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

#ifndef __INCLUDE_NUTTX_MOTOR_FOC_FOC_LOWER_H
#define __INCLUDE_NUTTX_MOTOR_FOC_FOC_LOWER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/motor/foc/foc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Only for kernel side */

#ifndef __KERNEL__
#  error
#endif

/* Helper macros */

#define FOC_OPS_CONFIGURE(d, c)         (d)->lower->ops->configure(d, c)
#define FOC_OPS_SETUP(d)                (d)->lower->ops->setup(d)
#define FOC_OPS_SHUTDOWN(d)             (d)->lower->ops->shutdown(d)
#define FOC_OPS_START(d, s)             (d)->lower->ops->start(d, s)
#define FOC_OPS_DUTY(d, x)              (d)->lower->ops->pwm_duty_set(d, x)
#define FOC_OPS_IOCTL(d, c, a)          (d)->lower->ops->ioctl(d, c, a)
#define FOC_OPS_BIND(d, c)              (d)->lower->ops->bind(d, c)
#define FOC_OPS_FAULT_CLEAR(d)          (d)->lower->ops->fault_clear(d)
#ifdef CONFIG_MOTOR_FOC_TRACE
#  define FOC_OPS_TRACE(d, t, s)        (d)->lower->ops->trace(d, t, s)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_MOTOR_FOC_TRACE
/* FOC trace type */

enum foc_trace_type_e
{
  FOC_TRACE_NONE     = 0,       /* Not used */
  FOC_TRACE_PARAMS   = 1,       /* In foc_params_set() */
  FOC_TRACE_STATE    = 2,       /* In foc_state_get() */
  FOC_TRACE_NOTIFIER = 3,       /* In foc_notifier() */
  FOC_TRACE_LOWER    = 4        /* Reserved for lower-half code */
};
#endif

/* Upper-half FOC callbacks  */

struct foc_callbacks_s
{
  /* FOC notifier callback
   *
   * Description:
   *   Deliver the phase current samples (and optional BEMF voltages)
   *   and wake up the thread waiting. Must be called by lower-half
   *   logic at a frequency determined by configuration (notifier_freq
   *   in foc_cfg_s).
   */

  CODE int (*notifier)(FAR struct foc_dev_s *dev,
                       FAR foc_current_t *current,
                       FAR foc_voltage_t *voltage);
};

/* Lower-half FOC operations */

struct foc_lower_ops_s
{
  /* Lower-half configuration */

  CODE int (*configure)(FAR struct foc_dev_s *dev,
                        FAR struct foc_cfg_s *cfg);

  /* Lower-half setup */

  CODE int (*setup)(FAR struct foc_dev_s *dev);

  /* Lower-half shutdwon */

  CODE int (*shutdown)(FAR struct foc_dev_s *dev);

  /* Set the PWM duty cycles */

  CODE int (*pwm_duty_set)(FAR struct foc_dev_s *dev,
                           FAR foc_duty_t *duty);

  /* Lower-half start/stop */

  CODE int (*start)(FAR struct foc_dev_s *dev, bool state);

  /* Lower-half IOCTL */

  CODE int (*ioctl)(FAR struct foc_dev_s *dev, int cmd,
                    unsigned long arg);

  /* Bind the upper-half driver with the lower-half logic */

  CODE int (*bind)(FAR struct foc_dev_s *dev,
                   FAR struct foc_callbacks_s *cb);

  /* Lower-half fault clear */

  CODE int (*fault_clear)(FAR struct foc_dev_s *dev);

#ifdef CONFIG_MOTOR_FOC_TRACE
  /* FOC trace */

  CODE void (*trace)(FAR struct foc_dev_s *dev, int type, bool state);
#endif
};

/* Lower-half FOC data - must be provided by lower-half implementation */

struct foc_lower_s
{
  FAR struct foc_lower_ops_s *ops;    /* The FOC lower-half operations */
  FAR void                   *data;   /* The FOC lower-half data */
};

#endif /* __INCLUDE_NUTTX_MOTOR_FOC_FOC_LOWER_H */
