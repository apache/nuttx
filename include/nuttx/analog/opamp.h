/************************************************************************************
 * include/nuttx/analog/opamp.h
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
   * This method reverses the operation the setup method.
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
 * Public Functions
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
