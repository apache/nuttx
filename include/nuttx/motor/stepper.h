/*****************************************************************************
 * include/nuttx/motor/stepper.h
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
 *****************************************************************************/

#ifndef __INCLUDE_NUTTX_MOTOR_STEPPER_H
#define __INCLUDE_NUTTX_MOTOR_STEPPER_H

/* The motor driver is split into two parts:
 *
 * 1) An "upper half", generic driver that provides the common motor
 *    interface to application level code, and
 * 2) A "lower half", platform-specific driver that implements the low-level
 *    functionality eg.:
 *      - timer controls to implement the PWM signals,
 *      - analog peripherals configuration such as ADC, DAC and comparators,
 *      - control algorithm for motor driver (eg. FOC control for BLDC)
 *
 * This 'upper-half' driver has been designed with flexibility in mind
 * to support all kinds of electric motors and their applications.
 */

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include <nuttx/motor/stepper_ioctl.h>

#ifdef CONFIG_STEPPER_UPPER

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

/*****************************************************************************
 * Public Types
 *****************************************************************************/

/* Stepper driver state */

enum stepper_state_e
{
  STEPPER_STATE_INIT     = 0,     /* Initial state */
  STEPPER_STATE_IDLE     = 1,     /* IDLE state */
  STEPPER_STATE_READY    = 2,     /* Ready to work */
  STEPPER_STATE_RUN      = 3,     /* Run state */
  STEPPER_STATE_FAULT    = 4      /* Fault state */
};

/* Stepper driver fault type */

enum stepper_fault_e
{
  STEPPER_FAULT_CLEAR        = 0,         /* No fault */
  STEPPER_FAULT_OVERCURRENT  = (1 << 0),  /* Over-current Fault */
  STEPPER_FAULT_OVERVOLTAGE  = (1 << 1),  /* Over-voltage Fault */
  STEPPER_FAULT_OVERPOWER    = (1 << 2),  /* Over-power Fault (electrical) */
  STEPPER_FAULT_OVERTEMP     = (1 << 3),  /* Over-temperature Fault */
  STEPPER_FAULT_OVERLOAD     = (1 << 4),  /* Stepper overload Fault (mechanical) */
  STEPPER_FAULT_LOCKED       = (1 << 5),  /* Stepper locked Fault */
  STEPPER_FAULT_INVAL_PARAM  = (1 << 6),  /* Invalid parameter Fault */
  STEPPER_FAULT_OTHER        = (1 << 7)   /* Other Fault */
};

/* Stepper IDLE control */

enum stepper_idle_e
{
  STEPPER_ENABLE_IDLE  = 0,  /* Enable IDLE mode */
  STEPPER_DISABLE_IDLE = 1,  /* Disable IDLE mode */
  STEPPER_AUTO_IDLE    = 2,  /* Set automaticaly IDLE when stepper not in movement */
};

/* Stepper driver status */

struct stepper_status_s
{
  uint8_t state;      /* Stepper driver state  */
  uint8_t fault;      /* Stepper driver faults */
  int32_t position;   /* Current absolute position */
};

/* Stepper parameters. */

struct stepper_job_s
{
  int32_t   steps;     /* Steps to do. Position: CW, Negative: CCW */
  uint32_t  speed;     /* Stepper speed in step/s */
};

/* Stepper operations used to call from the upper-half, generic stepper driver
 * into lower-half, platform-specific logic.
 */

struct stepper_lowerhalf_s;
struct stepper_ops_s
{
  /* Setup stepper for operational mode */

  CODE int (*setup)(FAR struct stepper_lowerhalf_s *dev);

  /* Disable stepper */

  CODE int (*shutdown)(FAR struct stepper_lowerhalf_s *dev);

  /* Work */

  CODE int (*work)(FAR struct stepper_lowerhalf_s *dev,
                   FAR struct stepper_job_s const *param);

  /* Update motor/driver status  */

  CODE int (*update_status)(FAR struct stepper_lowerhalf_s *dev);

  /* Clear fault state */

  CODE int (*clear)(FAR struct stepper_lowerhalf_s *dev, uint8_t fault);

  /* Configure IDLE mode */

  CODE int (*idle)(FAR struct stepper_lowerhalf_s *dev, uint8_t idle);

  /* Configure stepping resolution mode */

  CODE int (*microstepping)(FAR struct stepper_lowerhalf_s *dev,
                            uint16_t resolution);

  /* Lower-half logic may support platform-specific ioctl commands */

  CODE int (*ioctl)(FAR struct stepper_lowerhalf_s *dev, int cmd,
                    unsigned long arg);
};

/* This structure is the generic form of state structure used by lower half
 * motor driver.
 */

struct stepper_lowerhalf_s
{
  FAR const struct stepper_ops_s *ops; /* Arch-specific operations */
  struct stepper_job_s      param;     /* Motor settings */
  struct stepper_status_s   status;    /* Motor status */
  FAR void                  *priv;     /* Private data */
};

/*****************************************************************************
 * Public Data
 *****************************************************************************/

/*****************************************************************************
 * Public Function Prototypes
 *****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/*****************************************************************************
 * Name: stepper_register
 *
 * Description:
 *   This function binds an instance of a "lower half" stepper driver with the
 *   "upper half" stepper device and registers that device so that can be used
 *   by application code.
 *
 *   We will register the character device with specified path.
 *
 * Input Parameters:
 *   path  - The user specifies path name.
 *   lower - A pointer to an instance of lower half stepper driver. This
 *           instance is bound to the stepper driver and must persists as long
 *           as the driver persists.
 *
 * Returned Value:
 *   OK if the driver was successfully register; A negated errno value is
 *   returned on any failure.
 *
 *****************************************************************************/

int stepper_register(FAR const char *path,
                     FAR struct stepper_lowerhalf_s *lower);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_STEPPER_UPPER */
#endif /* __INCLUDE_NUTTX_DRIVERS_MOTOR_STEPPER_H */
