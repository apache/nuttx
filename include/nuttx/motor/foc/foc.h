/****************************************************************************
 * include/nuttx/motor/foc/foc.h
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

#ifndef __INCLUDE_NUTTX_MOTOR_FOC_FOC_H
#define __INCLUDE_NUTTX_MOTOR_FOC_FOC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>

#include <nuttx/motor/foc/foc_pwr.h>

#include <stdbool.h>

#include <fixedmath.h>

#ifdef CONFIG_MOTOR_FOC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FOCDUTY_FROM_FLOAT(d)   (ftob16(d))
#define FOCDUTY_FROM_FIXED16(d) (d)

#define FOCDUTY_TO_FLOAT(d)     (b16tof(d))
#define FOCDUTY_TO_FIXED16(d)   (d)

#define FOC_BOARDCFG_GAINLIST_LEN 4

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* FOC device fault code */

enum foc_fault_e
{
  FOC_FAULT_NONE    = (0),      /* No fault */
  FOC_FAULT_TIMEOUT = (1 << 1), /* Timeout fault */
  FOC_FAULT_ARCH    = (1 << 2), /* Arch-specific fault */
  FOC_FAULT_BOARD   = (1 << 3), /* Board-specific fault */
};

/* Phase current and BEMF voltage as signed 32-bit integer */

typedef int32_t foc_current_t;
typedef int32_t foc_voltage_t;

/* Phase duty cycle as unsigned fixed16.
 * We use range [0.0 to 1.0] so this gives us a 16-bit resolution.
 */

typedef ub16_t foc_duty_t;

/* FOC device configuration */

struct foc_cfg_s
{
  uint32_t pwm_freq;            /* FOC PWM frequency */
  uint32_t notifier_freq;       /* FOC notifier frequency */
};

/* Output data from the FOC device */

struct foc_state_s
{
  bool          pwm_off;                       /* PWM switches disabled */
  uint8_t       fault;                         /* Fault state */
  foc_current_t curr[CONFIG_MOTOR_FOC_PHASES]; /* Phase current feedback */
#ifdef CONFIG_MOTOR_FOC_BEMF_SENSE
  foc_voltage_t volt[CONFIG_MOTOR_FOC_PHASES]; /* BEMF voltage feedback */
#endif
};

/* Input data to the FOC device */

struct foc_params_s
{
  foc_duty_t duty[CONFIG_MOTOR_FOC_PHASES]; /* PWM duty cycle for phases */
};

/* Hardware specific informations */

struct foc_info_hw_s
{
  /* PWM configuration */

  uint32_t   pwm_dt_ns;   /* PWM dead-time in nano seconds */
  foc_duty_t pwm_max;     /* Maximum PWM duty cycle */

  /* ADC configuration for phase current sampling
   *
   * In most cases it will be:
   *
   *   iphase_scale = iphase_ratio * adc_to_volt
   *   adc_to_volt  = adc_ref_volt / adc_val_max
   *   for shunt sensors iphase_ratio = 1 / (R_shunt * gain)
   */

  int32_t    iphase_scale;              /* Current phase scale [x100000] */
  int32_t    iphase_max;                /* Maximum phase curretn [x1000] */

  /* ADC configuration for BEMF sampling */

#ifdef CONFIG_MOTOR_FOC_BEMF_SENSE
  int32_t    bemf_scale;                /* BEMF sampling scale [x1000] */
#endif
};

/* FOC driver info */

struct foc_info_s
{
  struct foc_info_hw_s hw_cfg; /* Hardware specific informations  */
};

/* FOC board-specific configuration */

struct foc_set_boardcfg_s
{
  int gain;
};

struct foc_get_boardcfg_s
{
  int gain;
  int gain_list[FOC_BOARDCFG_GAINLIST_LEN];
};

/* FOC device upper-half */

struct foc_lower_s;
struct foc_dev_s
{
  /* Fields managed by common upper-half FOC logic **************************/

  uint8_t                    ocount;     /* The number of times the device
                                          * has been opened
                                          */
  mutex_t                    closelock;  /* Locks out new opens while close
                                          * is in progress
                                          */
  sem_t                      statesem;   /* Notifier semaphore */

  /* Fields provided by lower-half foc logic ********************************/

  FAR struct foc_lower_s    *lower;      /* Reference to the FOC lower-half */

  /* FOC device specific data ***********************************************/

  struct foc_info_s          info;       /* Device info */
  struct foc_cfg_s           cfg;        /* FOC common configuration */

  /* FOC device input/output data *******************************************/

  struct foc_state_s         state;      /* FOC device state */

  /* (Optional) FOC power-stage driver  *************************************/

  FAR struct focpwr_dev_s    *pwr;        /* FOC power-stage driver */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

int foc_register(FAR const char *path, FAR struct foc_dev_s *dev);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_MOTOR_FOC */
#endif /* __INCLUDE_NUTTX_MOTOR_FOC_FOC_H */
