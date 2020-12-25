/************************************************************************************
 * include/nuttx/analog/opamp.h
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

#ifndef __INCLUDE_NUTTX_ANALOG_OPAMP_H
#define __INCLUDE_NUTTX_ANALOG_OPAMP_H

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

/************************************************************************************
 * Public Types
 ************************************************************************************/

struct opamp_dev_s;
struct opamp_ops_s
{
  /* Configure the OPAMP. This method is called the first time that the OPAMP
   * device is opened.  This will occur when the port is first opened.
   * This setup includes configuring and attaching OPAMP interrupts.  Interrupts
   * are all disabled upon return.
   */

  CODE int (*ao_setup)(FAR struct opamp_dev_s *dev);

  /* Disable the OPAMP.  This method is called when the OPAMP device is closed.
   * This method reverses the operation of the setup method.
   * Works only if OPAMP device is not locked.
   */

  CODE void (*ao_shutdown)(FAR struct opamp_dev_s *dev);

  /* All ioctl calls will be routed through this method */

  CODE int (*ao_ioctl)(FAR struct opamp_dev_s *dev, int cmd, unsigned long arg);
};

struct opamp_dev_s
{
#ifdef CONFIG_OPAMP
  /* Fields managed by common upper half OPAMP logic */

  uint8_t                 ad_ocount;    /* The number of times the device has been opened */
  sem_t                   ad_closesem;  /* Locks out new opens while close is in progress */
#endif

  /* Fields provided by lower half OPAMP logic */

  FAR const struct opamp_ops_s *ad_ops;  /* Arch-specific operations */
  FAR void                     *ad_priv; /* Used by the arch-specific logic */
};

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#if defined(__cplusplus)
extern "C"
{
#endif

int opamp_register(FAR const char *path, FAR struct opamp_dev_s *dev);

#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_ANALOG_OPAMP_H */
