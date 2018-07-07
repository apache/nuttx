/****************************************************************************
 * include/nuttx/power/motor.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Mateusz Szafoni <raiden00@railab.me>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_DRIVERS_POWER_MOTOR_H
#define __INCLUDE_NUTTX_DRIVERS_POWER_MOTOR_H

/*
 * The motor driver is split into two parts:
 *
 * 1) An "upper half", generic driver that provides the common motor interface
 *    to application level code, and
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

#include <nuttx/power/power_ioctl.h>

#ifdef CONFIG_DRIVERS_MOTOR

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
#ifdef CONFIG_MOTOR_HAVE_POSITION
  float position;               /* Current motor position */
#endif
#ifdef CONFIG_MOTOR_HAVE_SPEED
  float speed;                  /* Current motor speed */
#endif
#ifdef CONFIG_MOTOR_HAVE_TORQUE
  float torque;                 /* Current motor torque (rotary motor) */
#endif
#ifdef CONFIG_MOTOR_HAVE_FORCE
  float force;                  /* Current motor force (linear motor) */
#endif
#ifdef CONFIG_MOTOR_HAVE_INPUT_VOLTAGE
  float v_in;                   /* Current input voltage */
#endif
#ifdef CONFIG_MOTOR_HAVE_INPUT_CURRENT
  float i_in;                   /* Current input current */
#endif
#ifdef CONFIG_MOTOR_HAVE_INPUT_POWER
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
#ifdef CONFIG_MOTOR_HAVE_POSITION
  float position;                    /* Maximum motor position */
#endif
#ifdef CONFIG_MOTOR_HAVE_SPEED
  float speed;                       /* Maximum motor speed */
#endif
#ifdef CONFIG_MOTOR_HAVE_TORQUE
  float torque;                      /* Maximum motor torque (rotary motor) */
#endif
#ifdef CONFIG_MOTOR_HAVE_FORCE
  float force;                       /* Maximum motor force (linear motor) */
#endif
#ifdef CONFIG_MOTOR_HAVE_ACCELERATION
  float acceleration;                /* Maximum motor acceleration */
#endif
#ifdef CONFIG_MOTOR_HAVE_DECELERATION
  float deceleration;                /* Maximum motor decelaration */
#endif
#ifdef CONFIG_MOTOR_HAVE_INPUT_VOLTAGE
  float v_in;                        /* Maximum input voltage */
#endif
#ifdef CONFIG_MOTOR_HAVE_INPUT_CURRENT
  float i_in;                        /* Maximum input current */
#endif
#ifdef CONFIG_MOTOR_HAVE_INPUT_POWER
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
#ifdef CONFIG_MOTOR_HAVE_DIRECTION
  int8_t  direction;                 /* Motor movement direction. We do not
                                      * support negative values for parameters,
                                      * so this flag can be used to allow movement
                                      * in the positive and negative direction in
                                      * a given coordinate system.
                                      */
#endif
#ifdef CONFIG_MOTOR_HAVE_POSITION
  float position;                    /* Motor position */
#endif
#ifdef CONFIG_MOTOR_HAVE_SPEED
  float speed;                       /* Motor speed */
#endif
#ifdef CONFIG_MOTOR_HAVE_TORQUE
  float torque;                      /* Motor torque (rotary motor)*/
#endif
#ifdef CONFIG_MOTOR_HAVE_FORCE
  float force;                       /* Motor force (linear motor) */
#endif
#ifdef CONFIG_MOTOR_HAVE_ACCELERATION
  float acceleration;                /* Motor acceleration */
#endif
#ifdef CONFIG_MOTOR_HAVE_DECELERATION
  float deceleration;                /* Motor deceleration */
#endif
};

/* Motor private data structure */

struct motor_s
{
  uint8_t                    opmode;  /* Motor operation mode */
  uint8_t                    opflags; /* Motor operation flags */
  struct motor_limits_s      limits;  /* Motor absolute limits */
  struct motor_params_s      param;   /* Motor settings */
  struct motor_state_s       state;   /* Motor state */
  FAR void                   *priv;   /* Private data */
};

/* Motor operations used to call from the upper-half, generic motor driver
 * into lower-half, platform-specific logic.
 */

struct motor_dev_s;
struct motor_ops_s
{
  /* Configure motor */

  CODE int (*setup)(FAR struct motor_dev_s *dev);

  /* Disable motor */

  CODE int (*shutdown)(FAR struct motor_dev_s *dev);

  /* Stop motor */

  CODE int (*stop)(FAR struct motor_dev_s *dev);

  /* Start motor */

  CODE int (*start)(FAR struct motor_dev_s *dev);

  /* Set motor parameters */

  CODE int (*params_set)(FAR struct motor_dev_s *dev,
                         FAR struct motor_params_s *param);

  /* Set motor operation mode */

  CODE int (*mode_set)(FAR struct motor_dev_s *dev, uint8_t mode);

  /* Set motor limts */

  CODE int (*limits_set)(FAR struct motor_dev_s *dev,
                         FAR struct motor_limits_s *limits);

  /* Set motor fault */

  CODE int (*fault_set)(FAR struct motor_dev_s *dev, uint8_t fault);

  /* Get motor state  */

  CODE int (*state_get)(FAR struct motor_dev_s *dev,
                         FAR struct motor_state_s *state);

  /* Get current fault state */

  CODE int (*fault_get)(FAR struct motor_dev_s *dev, FAR uint8_t *fault);

  /* Clean fault state */

  CODE int (*fault_clean)(FAR struct motor_dev_s *dev, uint8_t fault);

  /* Lower-half logic may support platform-specific ioctl commands */

  CODE int (*ioctl)(FAR struct motor_dev_s *dev, int cmd, unsigned long arg);
};

/*  */

struct motor_dev_s
{
  /* Fields managed by common upper half motor logic */

  uint8_t                     ocount;   /* The number of times the device
                                         * has been opened
                                         */
  sem_t                       closesem; /* Locks out new opens while close
                                         * is in progress
                                         */

  /* Fields provided by lower half motor logic */

  FAR const struct motor_ops_s *ops;    /* Arch-specific operations */
  FAR void                     *priv;   /* Reference to motor private data */
  FAR void                     *lower;  /* Reference to lower level drivers */
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
 ****************************************************************************/

int motor_register(FAR const char *path, FAR struct motor_dev_s *dev,
                   FAR void *lower);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_DRIVERS_MOTOR */
#endif /* __INCLUDE_NUTTX_DRIVERS_POWER_MOTOR_H */
