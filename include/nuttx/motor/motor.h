/****************************************************************************
 * include/nuttx/motor/motor.h
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

#ifndef __INCLUDE_NUTTX_MOTOR_MOTOR_H
#define __INCLUDE_NUTTX_MOTOR_MOTOR_H

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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <nuttx/motor/motor_ioctl.h>

#ifdef CONFIG_MOTOR_UPPER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Motor driver operation modes */

enum motor_opmode_e
{
  MOTOR_OPMODE_INIT      = 0,   /* Initial mode */
  MOTOR_OPMODE_POSITION  = 1,   /* Position control mode */
  MOTOR_OPMODE_SPEED     = 2,   /* Speed control mode */
  MOTOR_OPMODE_TORQUE    = 3,   /* Torque control mode */
  MOTOR_OPMODE_FORCE     = 4    /* Force control mode */
};

/* Motor driver state */

enum motor_state_e
{
  MOTOR_STATE_INIT     = 0,     /* Initial state */
  MOTOR_STATE_IDLE     = 1,     /* IDLE state */
  MOTOR_STATE_RUN      = 2,     /* Run state */
  MOTOR_STATE_FAULT    = 3,     /* Fault state */
  MOTOR_STATE_CRITICAL = 4      /* Critical Fault state */
};

/* Motor driver fault type */

enum motor_fault_e
{
  MOTOR_FAULT_OVERCURRENT  = (1 << 0),  /* Over-current Fault */
  MOTOR_FAULT_OVERVOLTAGE  = (1 << 1),  /* Over-voltage Fault */
  MOTOR_FAULT_OVERPOWER    = (1 << 2),  /* Over-power Fault (electrical) */
  MOTOR_FAULT_OVERTEMP     = (1 << 3),  /* Over-temperature Fault */
  MOTOR_FAULT_OVERLOAD     = (1 << 4),  /* Motor overload Fault (mechanical) */
  MOTOR_FAULT_LOCKED       = (1 << 5),  /* Motor locked Fault */
  MOTOR_FAULT_INVAL_PARAM  = (1 << 6),  /* Invalid parameter Fault */
  MOTOR_FAULT_OTHER        = (1 << 7)   /* Other Fault */
};

/* Motor direction */

enum motor_direction_e
{
  MOTOR_DIR_CCW = -1,
  MOTOR_DIR_CW  = 1
};

/* This structure contains feedback data from motor driver */

struct motor_feedback_s
{
#ifdef CONFIG_MOTOR_UPPER_HAVE_POSITION
  float position;               /* Current motor position */
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_SPEED
  float speed;                  /* Current motor speed */
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_TORQUE
  float torque;                 /* Current motor torque (rotary motor) */
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_FORCE
  float force;                  /* Current motor force (linear motor) */
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_INPUT_VOLTAGE
  float v_in;                   /* Current input voltage */
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_INPUT_CURRENT
  float i_in;                   /* Current input current */
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_INPUT_POWER
  float p_in;                   /* Current input power */
#endif
};

/* This structure describes motor driver state */

struct motor_state_s
{
  uint8_t                 state;     /* Motor state  */
  uint8_t                 fault;     /* Motor faults state */
  struct motor_feedback_s fb;        /* Feedback from motor */
};

/* Motor absolute limits. Exceeding this limits should cause critical error
 * This structure must be configured before motor params_set call.
 * When limit is set to 0 then it is ignored.
 */

struct motor_limits_s
{
  bool  lock;                        /* This bit must be set after
                                      * limits configuration.
                                      */
#ifdef CONFIG_MOTOR_UPPER_HAVE_POSITION
  float position;                    /* Maximum motor position */
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_SPEED
  float speed;                       /* Maximum motor speed */
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_TORQUE
  float torque;                      /* Maximum motor torque (rotary motor) */
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_FORCE
  float force;                       /* Maximum motor force (linear motor) */
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_ACCELERATION
  float acceleration;                /* Maximum motor acceleration */
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_DECELERATION
  float deceleration;                /* Maximum motor decelaration */
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_INPUT_VOLTAGE
  float v_in;                        /* Maximum input voltage */
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_INPUT_CURRENT
  float i_in;                        /* Maximum input current */
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_INPUT_POWER
  float p_in;                        /* Maximum input power */
#endif
};

/* Motor parameters.
 * NOTE: All parameters require not negative value.
 */

struct motor_params_s
{
  bool  lock;                        /* Lock this structure. Set this bit
                                      * if there is no need to change motor
                                      * parameter during run-time.
                                      */
#ifdef CONFIG_MOTOR_UPPER_HAVE_DIRECTION
  int8_t  direction;                 /* Motor movement direction. We do not
                                      * support negative values for parameters,
                                      * so this flag can be used to allow movement
                                      * in the positive and negative direction in
                                      * a given coordinate system.
                                      */
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_POSITION
  float position;                    /* Motor position */
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_SPEED
  float speed;                       /* Motor speed */
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_TORQUE
  float torque;                      /* Motor torque (rotary motor) */
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_FORCE
  float force;                       /* Motor force (linear motor) */
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_ACCELERATION
  float acceleration;                /* Motor acceleration */
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_DECELERATION
  float deceleration;                /* Motor deceleration */
#endif
};

/* Motor operations used to call from the upper-half, generic motor driver
 * into lower-half, platform-specific logic.
 */

struct motor_lowerhalf_s;
struct motor_ops_s
{
  /* Configure motor */

  CODE int (*setup)(FAR struct motor_lowerhalf_s *dev);

  /* Disable motor */

  CODE int (*shutdown)(FAR struct motor_lowerhalf_s *dev);

  /* Stop motor */

  CODE int (*stop)(FAR struct motor_lowerhalf_s *dev);

  /* Start motor */

  CODE int (*start)(FAR struct motor_lowerhalf_s *dev);

  /* Set motor parameters */

  CODE int (*params_set)(FAR struct motor_lowerhalf_s *dev,
                         FAR struct motor_params_s *param);

  /* Set motor operation mode */

  CODE int (*mode_set)(FAR struct motor_lowerhalf_s *dev, uint8_t mode);

  /* Set motor limits */

  CODE int (*limits_set)(FAR struct motor_lowerhalf_s *dev,
                         FAR struct motor_limits_s *limits);

  /* Set motor fault */

  CODE int (*fault_set)(FAR struct motor_lowerhalf_s *dev, uint8_t fault);

  /* Get motor state  */

  CODE int (*state_get)(FAR struct motor_lowerhalf_s *dev,
                        FAR struct motor_state_s *state);

  /* Get current fault state */

  CODE int (*fault_get)(FAR struct motor_lowerhalf_s *dev,
                        FAR uint8_t *fault);

  /* Clear fault state */

  CODE int (*fault_clear)(FAR struct motor_lowerhalf_s *dev, uint8_t fault);

  /* Lower-half logic may support platform-specific ioctl commands */

  CODE int (*ioctl)(FAR struct motor_lowerhalf_s *dev, int cmd,
                    unsigned long arg);
};

/* This structure is the generic form of state structure used by lower half
 * motor driver.
 */

struct motor_lowerhalf_s
{
  FAR const struct motor_ops_s *ops;    /* Arch-specific operations */
  uint8_t                      opmode;  /* Motor operation mode */
  uint8_t                      opflags; /* Motor operation flags */
  struct motor_limits_s        limits;  /* Motor absolute limits */
  struct motor_params_s        param;   /* Motor settings */
  struct motor_state_s         state;   /* Motor state */
  FAR void                     *priv;   /* Private data */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

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

/****************************************************************************
 * Name: motor_register
 *
 * Description:
 *   This function binds an instance of a "lower half" motor driver with the
 *   "upper half" motor device and registers that device so that can be used
 *   by application code.
 *
 *   We will register the chararter device with specified path.
 *
 * Input Parameters:
 *   path  - The user specifies path name.
 *   lower - A pointer to an instance of lower half motor driver. This
 *           instance is bound to the motor driver and must persists as long
 *           as the driver persists.
 *
 * Returned Value:
 *   OK if the driver was successfully register; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int motor_register(FAR const char *path,
                   FAR struct motor_lowerhalf_s *lower);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_MOTOR_UPPER */
#endif /* __INCLUDE_NUTTX_DRIVERS_MOTOR_MOTOR_H */
