/****************************************************************************
 * net/netdev/netdev_lladdrsize.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#include <string.h>
#include <errno.h>

#include <net/if.h>

#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>

#include "netdev/netdev.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netdev_type_lladdrsize
 *
 * Description:
 *   Returns the size of the MAC address associated with a link layer type.
 *
 * Parameters:
 *   lltype - link layer type code
 *
 * Returned Value:
 *   The size of the MAC address associated with this device
 *
 ****************************************************************************/

int netdev_type_lladdrsize(uint8_t lltype)
{
  /* Get the length of the address for this link layer type */

#ifdef CONFIG_NET_ETHERNET
  if (lltype == NET_LL_ETHERNET)
    {
      /* size of the Ethernet MAC address */

      return IFHWADDRLEN;
    }
  else
#endif
#ifdef CONFIG_NET_6LOWPAN
  if (lltype == NET_LL_IEEE802154)
    {
      /* 6LoWPAN can be configured to use either extended or short
       * addressing.
       */

#ifdef CONFIG_NET_6LOWPAN_EXTENDEDADDR
      return NET_6LOWPAN_EADDRSIZE;
#else
      return NET_6LOWPAN_SADDRSIZE;
#endif
    }
  else
#endif
    {
      /* Either the link layer type associated with lltype has no address,
       * or support for that link layer type is not enabled.
       */

      return 0;
    }
}
