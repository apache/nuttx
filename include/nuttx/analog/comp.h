/************************************************************************************
 * include/nuttx/analog/comp.h
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
#include <nuttx/mutex.h>
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
   *   dev - The COMP device structure that was previously registered by
   *         adc_register()
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
   * This method reverses the operation of the setup method.
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

  uint8_t ad_ocount;           /* The number of times the device has been opened */
  uint8_t val;                 /* Comparator value after output transition event */
  mutex_t ad_lock;             /* Used to serialize access  */
  sem_t   ad_readsem;          /* Blocking read */

  /* pollfd's for output transition events */

  struct pollfd *d_fds[CONFIG_DEV_COMP_NPOLLWAITERS];
#endif

  /* Fields provided by lower half COMP logic */

  FAR const struct comp_ops_s *ad_ops;  /* Arch-specific operations */
  FAR void                    *ad_priv; /* Used by the arch-specific logic */
};

/************************************************************************************
 * Public Function Prototypes
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
