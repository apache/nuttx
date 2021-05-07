/****************************************************************************
 * include/nuttx/sensors/zerocross.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_ZEROCROSS_H
#define __INCLUDE_NUTTX_SENSORS_ZEROCROSS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

#include <signal.h>

#ifdef CONFIG_SENSORS_ZEROCROSS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************
 * CONFIG_SENSORS_ZEROCROSS - Enables support for the zero cross AC detection
 *   upper half
 */

/* Command:     ZCIOC_REGISTER
 * Description: Register to receive a signal whenever there is zero cross
 *              interrupt event.
 * Argument:    A read-only pointer to an instance of struct sigevent
 * Return:      Zero (OK) on success.  Minus one will be returned on failure
 *              with the errno value set appropriately.
 */

#define ZCIOC_REGISTER   _ZCIOC(0x0001)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This is the vtable that is used to by the upper half of the zero cross
 * to call back into the lower half of the zero cross driver.
 */

struct zc_lowerhalf_s;

/* This is the type of the discrete zero cross interrupt handler used with
 * the struct zc_lowerhalf_s enable() method.
 */

typedef CODE void (*zc_interrupt_t)
  (FAR const struct zc_lowerhalf_s *lower, FAR void *arg);

/* This is the interface between the lower half zero cross detection driver
 * and the upper half zero cross detection driver.  A (device-specific)
 * instance of this structure is passed to the upper-half driver when the
 * zero cross driver is registered.
 *
 * Normally that lower half logic will have its own, custom state structure
 * that is simply cast to struct zc_lowerhalf_s.  In order to perform such
 * casts, the initial fields of the custom state structure match the initial
 * fields of the following generic lower half state structure.
 */

struct zc_lowerhalf_s
{
  /* Enable interrupt on the defined pin used to zero cross detection */

  CODE void (*zc_enable)(FAR const struct zc_lowerhalf_s *lower,
                         zc_interrupt_t handler, FAR void *arg);
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: zc_register
 *
 * Description:
 *   Register the Zero Cross lower half device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/zc0"
 *   lower - An instance of the lower half interface
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int zc_register(FAR const char *devpath, FAR struct zc_lowerhalf_s *lower);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SENSORS_ZEROCROSS */
#endif /* __INCLUDE_NUTTX_SENSORS_ZEROCROSS_H */
