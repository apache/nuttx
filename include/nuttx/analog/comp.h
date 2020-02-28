/************************************************************************************
 * include/nuttx/analog/comp.h
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
 ************************************************************************************/

#ifndef __INCLUDE_NUTTX_ANALOG_COMP_H
#define __INCLUDE_NUTTX_ANALOG_COMP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include <nuttx/fs/fs.h>
#include <nuttx/semaphore.h>

#ifndef CONFIG_DEV_COMP_NPOLLWAITERS
#  define CONFIG_DEV_COMP_NPOLLWAITERS 2
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

struct comp_dev_s;
struct comp_callback_s
{
  /* This method is called from the lower half, platform-specific COMP logic when
   * comparator output state changes.
   *
   * Input Parameters:
   *   dev - The COMP device structure that was previously registered by adc_register()
   *   val - The actual value of the comparator output.
   *
   * Returned Value:
   *   Zero on success; a negated errno value on failure.
   */

  CODE int (*au_notify)(FAR struct comp_dev_s *dev, uint8_t val);
};

struct comp_ops_s
{
  /* Bind the upper-half driver callbacks to the lower-half implementation.  This
   * must be called early in order to receive COMP event notifications.

   */

  CODE int (*ao_bind)(FAR struct comp_dev_s *dev,
                      FAR const struct comp_callback_s *callback);

  /* Configure the COMP. This method is called the first time that the COMP
   * device is opened.  This will occur when the port is first opened.
   * This setup includes configuring and attaching COMP interrupts.  Interrupts
   * are all disabled upon return.
   */

  CODE int (*ao_setup)(FAR struct comp_dev_s *dev);

  /* Disable the COMP.  This method is called when the COMP device is closed.
   * This method reverses the operation the setup method.
   * Works only if COMP device is not locked.
   */

  CODE void (*ao_shutdown)(FAR struct comp_dev_s *dev);

  /* Read COMP output state */

  CODE int (*ao_read)(FAR struct comp_dev_s *dev);

  /* All ioctl calls will be routed through this method */

  CODE int (*ao_ioctl)(FAR struct comp_dev_s *dev, int cmd, unsigned long arg);
};

struct comp_dev_s
{
#ifdef CONFIG_COMP
  /* Fields managed by common upper half COMP logic */

  uint8_t ad_ocount;            /* The number of times the device has been opened */
  uint8_t val;                 /* Comparator value after output transition event */
  sem_t   ad_sem;               /* Used to serialize access  */
  sem_t   ad_readsem;           /* Blocking read */

  /* pollfd's for output transition events */

  struct pollfd *d_fds[CONFIG_DEV_COMP_NPOLLWAITERS];
#endif

  /* Fields provided by lower half COMP logic */

  FAR const struct comp_ops_s *ad_ops;  /* Arch-specific operations */
  FAR void                    *ad_priv; /* Used by the arch-specific logic */
};

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#if defined(__cplusplus)
extern "C"
{
#endif

int comp_register(FAR const char *path, FAR struct comp_dev_s *dev);

#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_ANALOG_COMP_H */
