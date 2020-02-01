/****************************************************************************
 * include/nuttx/power/powerled.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_DRIVERS_POWER_POWERLED_H
#define __INCLUDE_NUTTX_DRIVERS_POWER_POWERLED_H

/*
 * The powerled driver is split into two parts:
 *
 * 1) An "upper half", generic driver that provides the common high power LED
 *    interface to application level code, and
 * 2) A "lower half", platform-specific driver that implements the low-level
 *    functionality.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <nuttx/power/power_ioctl.h>
#include <nuttx/semaphore.h>

#ifdef CONFIG_DRIVERS_POWERLED

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define POWERLED_BRIGHTNESS_MAX ((float)100.0)
#define POWERLED_BRIGHTNESS_MIN ((float)0.0)

#define POWERLED_DUTY_MAX ((float)100.0)
#define POWERLED_DUTY_MIN ((float)0.0)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Powerled operation mode */

enum powerled_opmode_e
{
  POWERLED_OPMODE_INIT = 0,       /* Initial mode */
  POWERLED_OPMODE_CONTINUOUS,     /* Continuous mode */
  POWERLED_OPMODE_FLASH           /* Flash mode */
};

/* Powerled state */

enum powerled_state_e
{
  POWERLED_STATE_INIT         = 0,  /* Initial state */
  POWERLED_STATE_IDLE         = 1,  /* IDLE state */
  POWERLED_STATE_RUN          = 2,  /* Run state */
  POWERLED_STATE_FAULT        = 3,  /* Fault state */
  POWERLED_STATE_CRITICAL     = 4   /* Critical Fault state */
};

/* Powerled faults */

enum powerled_fault_e
{
  POWERLED_FAULT_OVERHEAT = (1<<0) /* Overheat fault */
};

/* This structure describes converter state */

struct powerled_state_s
{
  uint8_t                state;     /* Powerled state  */
  uint8_t                fault;     /* Fault state */
};

/* Powerled limits */

struct powerled_limits_s
{
  bool  lock;                      /* This bit must be set after
                                    * limits configuration.
                                    */
  float current;                    /* Max current for LED */
};

/* Powerled parameters */

struct powerled_params_s
{
  bool  lock;                      /* Lock this structure. Set this bit
                                    * if there is no need to change SMPS
                                    * parameter during run-time.
                                    */
  float brightness;                /* LED brightnes used in continuous and
                                    * flash mode.
                                    * Valid value: 0.0 - 100.0
                                    */
  float frequency;                 /* Flash frequency, used in flash mode */
  float duty;                      /* Flash duty, used in flash mode
                                    * Valid value: 0.0 - 100.0
                                    */
};

/* Powerled private data strucutre  */

struct powerled_s
{
  uint8_t                    opmode;      /* Powerled operation mode */
  uint8_t                    opflags;     /* Powerled operation flags */
  struct powerled_limits_s   limits;      /* Powerled limits */
  struct powerled_params_s   param;       /* Powerled settings */
  struct powerled_state_s    state;       /* Powerled state */
  FAR void                   *priv;       /* Private data */
};

/* Powerled operations used to call from the upper-half, generic powerled driver
 * into lower-half, platform-specific logic.
 */

struct powerled_dev_s;
struct powerled_ops_s
{
  /* Configure powerled */

  CODE int (*setup)(FAR struct powerled_dev_s *dev);

  /* Disable converter action */

  CODE int (*shutdown)(FAR struct powerled_dev_s *dev);

  /* Stop powerled action */

  CODE int (*stop)(FAR struct powerled_dev_s *dev);

  /* Start powerled action */

  CODE int (*start)(FAR struct powerled_dev_s *dev);

  /* Set powerled parameters */

  CODE int (*params_set)(FAR struct powerled_dev_s *dev,
                         struct powerled_params_s *param);

  /* Set powerled operation mode */

  CODE int (*mode_set)(FAR struct powerled_dev_s *dev, uint8_t mode);

  /* Set powerled limits */

  CODE int (*limits_set)(FAR struct powerled_dev_s *dev,
                         FAR struct powerled_limits_s *limits);

  /* Set powerled fault */

  CODE int (*fault_set)(FAR struct powerled_dev_s *dev, uint8_t fault);

  /* Get powerled state  */

  CODE int (*state_get)(FAR struct powerled_dev_s *dev,
                        FAR struct powerled_state_s *state);

  /* Get current fault state */

  CODE int (*fault_get)(FAR struct powerled_dev_s *dev, FAR uint8_t *fault);

  /* Clean fault state */

  CODE int (*fault_clean)(FAR struct powerled_dev_s *dev, uint8_t fault);

  /* Lower-half logic may support platform-specific ioctl commands */

  CODE int (*ioctl)(FAR struct powerled_dev_s *dev, int cmd, unsigned long arg);
};

/* Powerled device structure used by the driver. The caller of powerled_register
 * must allocate and initialize this structure. The calling logic need
 * provide 'ops', 'priv' and 'lower' elements.
 */

struct powerled_dev_s
{
  /* Fields managed by common upper half powerled logic */

  uint8_t                     ocount;    /* The number of times the device
                                          * has been opened
                                          */
  sem_t                       closesem;   /* Locks out new opens while close
                                          * is in progress
                                          */

  /* Fields provided by lower half powerled logic */

  FAR const struct powerled_ops_s *ops;   /* Arch-specific operations */
  FAR void                        *priv;  /* Reference to powerled private data */
  FAR void                        *lower; /* Reference to lower level drivers */
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
 * Name: powerled_register
 ****************************************************************************/

int powerled_register(FAR const char *path, FAR struct powerled_dev_s *dev,
                  FAR void *lower);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_DRIVERS_POWERLED */
#endif /* __INCLUDE_NUTTX_DRIVERS_POWER_POWERLED_H */
