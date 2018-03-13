/****************************************************************************
 * net/netdev/netdev_carrier.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Max Holtzberg <mh@uvc.de>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#if defined(CONFIG_NET) && CONFIG_NSOCKET_DESCRIPTORS > 0

#include <sys/socket.h>
#include <stdio.h>
#include <semaphore.h>
#include <assert.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <net/if.h>
#include <net/ethernet.h>
#include <nuttx/net/netdev.h>

#include "netdev/netdev.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netdev_carrier_on
 *
 * Description:
 *   Notifies the networking layer about an available carrier.
 *   (e.g. a cable was plugged in)
 *
 * Input Parameters:
 *   dev - The device driver structure
 *
 * Returned Value:
 *   0:Success; negated errno on failure
 *
 ****************************************************************************/

int netdev_carrier_on(FAR struct net_driver_s *dev)
{
  if (dev)
    {
      dev->d_flags |= IFF_RUNNING;
      return OK;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: netdev_carrier_off
 *
 * Description:
 *   Notifies the networking layer about an disappeared carrier.
 *   (e.g. a cable was unplugged)
 *
 * Input Parameters:
 *   dev - The device driver structure
 *
 * Returned Value:
 *   0:Success; negated errno on failure
 *
 ****************************************************************************/

int netdev_carrier_off(FAR struct net_driver_s *dev)
{
  if (dev)
    {
      dev->d_flags &= ~IFF_RUNNING;
      return OK;
    }

  return -EINVAL;
}

#endif /* CONFIG_NET && CONFIG_NSOCKET_DESCRIPTORS */
