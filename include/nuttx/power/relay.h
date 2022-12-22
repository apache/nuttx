/****************************************************************************
 * include/nuttx/power/relay.h
 * NuttX Relay Interfaces
 *
 *   Copyright (C) 2015 Max Nekludov. All rights reserved.
 *   Author: Max Nekludov <macscomp@gmail.com>
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

#ifndef __INCLUDE_NUTTX_POWER_RELAY_H
#define __INCLUDE_NUTTX_POWER_RELAY_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/mutex.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* IOCTL Commands ***********************************************************/

#define RELAYIOC_SET    _RELAYIOC(0x0001) /* Relay state set
                                           * IN: pointer of bool
                                           *     false=disable, open
                                           *     true=enable, close
                                           * OUT: None */

#define RELAYIOC_GET    _RELAYIOC(0x0002) /* Relay state get
                                           * IN: pointer of bool
                                           * OUT:
                                           *     false=disable, open
                                           *     true=enable, close */

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct relay_dev_s;
struct relay_ops_s
{
  CODE int (*set)(FAR struct relay_dev_s *dev, bool onoff);
  CODE int (*get)(FAR struct relay_dev_s *dev, FAR bool *onoff);
  CODE int (*ioctl)(FAR struct relay_dev_s *dev, int cmd,
                    unsigned long arg);
};

struct relay_dev_s
{
  FAR const struct relay_ops_s *ops;
  mutex_t                       lock;
};

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
 * Name: relay_register
 *
 * Description:
 *   Register the relay driver to the vfs.
 *
 * Input Parameters:
 *   dev     - the relay device
 *   devname - the relay device name
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RELAY
int relay_register(FAR struct relay_dev_s *dev, FAR const char *devname);
#endif

/****************************************************************************
 * Name: relay_gpio_register
 *
 * Description:
 *   Register the relay device based on the ioexpander device
 *
 * Input Parameters:
 *   iodev     - The ioexpander dev pointer.
 *   iopin     - The relay gpio pin number.
 *   ioinvert  - true : enable (relay close) is low , disable (relay open)
 *                      is high.
 *               false: enable (rekat close) is high, disable (relay open)
 *                      is low.
 *   devname   - The relay device name.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RELAY_GPIO
int relay_gpio_register(FAR struct ioexpander_dev_s *iodev, uint8_t iopin,
                        bool ioinvert, FAR const char *devname);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_POWER_RELAY_H */
