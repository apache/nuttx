/****************************************************************************
 * net/netdev/netdev_findbyindex.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#if defined(CONFIG_NET) && CONFIG_NSOCKET_DESCRIPTORS > 0

#include <string.h>
#include <errno.h>

#include <nuttx/net/netdev.h>

#include "utils/utils.h"
#include "netdev/netdev.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netdev_findbyindex
 *
 * Description:
 *   Find a previously registered network device by its position in the
 *   list of registered devices.  NOTE that this function is not a safe way
 *   to enumerate network devices:  There could be changes to the list of
 *   registered device causing a given index to be meaningless (unless, of
 *   course, the caller keeps the network locked).
 *
 * Parameters:
 *   index - the index of the interface to file
 *
 * Returned Value:
 *  Pointer to driver on success; NULL on failure.  This function can only
 *  fail if there are fewer registered interfaces than could be indexed.
 *
 * Assumptions:
 *  Called from normal user mode
 *
 ****************************************************************************/

FAR struct net_driver_s *netdev_findbyindex(int index)
{
  FAR struct net_driver_s *dev;
  int i;

  net_lock();
  for (i = 0, dev = g_netdevices; dev; i++, dev = dev->flink)
    {
      if (i == index)
        {
          net_unlock();
          return dev;
        }
    }

  net_unlock();
  return NULL;
}

#endif /* CONFIG_NET && CONFIG_NSOCKET_DESCRIPTORS */
