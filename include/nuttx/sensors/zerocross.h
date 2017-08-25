/****************************************************************************
 * include/nuttx/zerocross.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __INCLUDE_NUTTX_SENSORS_ZEROCROSS_H
#define __INCLUDE_NUTTX_SENSORS_ZEROCROSS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

#ifdef CONFIG_SENSORS_ZEROCROSS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************
 * CONFIG_SENSORS_ZEROCROSS - Enables support for the zero cross AC detection upper
 *   half
 */

/* Command:     ZCIOC_REGISTER
 * Description: Register to receive a signal whenever there is zero cross
 *              interrupt event.
 * Argument:    A read-only pointer to an instance of struct zc_notify_s
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

/* A reference to this structure is provided with the ZCIOC_REGISTER IOCTL
 * command and describes the conditions under which the client would like
 * to receive notification.
 */

struct zc_notify_s
{
  uint8_t          zc_signo;   /* Signal number to use in the notification */
};

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
