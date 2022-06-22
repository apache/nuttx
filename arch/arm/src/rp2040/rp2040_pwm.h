/****************************************************************************
 * arch/arm/src/rp2040/rp2040_pwm.h
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

#ifndef __ARCH_ARM_SRC_RP2040_RP2040_PWM_H
#define __ARCH_ARM_SRC_RP2040_RP2040_PWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/rp2040_pwm.h"
#include "nuttx/timers/pwm.h"

#ifndef __ASSEMBLY__
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* This structure represents the state of one PWM timer */

struct rp2040_pwm_lowerhalf_s
{
  const struct pwm_ops_s   * ops;        /* PWM operations */

  uint32_t                   frequency;  /* PWM current frequency */
  uint32_t                   divisor;    /* PWM current clock divisor */
  uint32_t                   flags;      /* PWM mode flags */
  uint16_t                   top;        /* PWM current top value */

#if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS == 2
  uint16_t                   duty[2];
  int8_t                     pin[2];
#else
  uint16_t                   duty;       /* Time duty value */
  int8_t                     pin;
#endif

  uint8_t                    num;        /* Timer ID */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rp2040_pwm_initialize
 *
 * Description:
 *   Initialize the selected PWM port. And return a unique instance of struct
 *   struct rp2040_pwm_lowerhalf_s.  This function may be called to obtain
 *   multiple instances of the interface, each of which may be set up with a
 *   different frequency and address.
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple PWM interfaces)
 *   GPIO pin number for pin A
 *   GPIO pin number for pin B (CONFIG_PWM_NCHANNELS == 2)
 *
 * Returned Value:
 *   Valid PWM device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

#if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS == 2
struct rp2040_pwm_lowerhalf_s *rp2040_pwm_initialize(int      port,
                                                     int      pin_a,
                                                     int      pin_b,
                                                     uint32_t flags);
#else
struct rp2040_pwm_lowerhalf_s *rp2040_pwm_initialize(int      port,
                                                     int      pin,
                                                     uint32_t flags);
#endif

/****************************************************************************
 * Name: rp2040_pwmdev_uninitialize
 *
 * Description:
 *   De-initialize the selected pwm port, and power down the device.
 *
 * Input Parameter:
 *   Device structure as returned by the rp2040_pwmdev_initialize()
 *
 * Returned Value:
 *   OK on success, ERROR when internal reference count mismatch or dev
 *   points to invalid hardware device.
 *
 ****************************************************************************/

int rp2040_pwm_uninitialize(struct pwm_lowerhalf_s *dev);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RP2040_RP2040_I2C_H */
