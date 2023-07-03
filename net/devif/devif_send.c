/****************************************************************************
 * net/devif/devif_send.c
 *
 *   Copyright (C) 2007, 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Based in part on uIP which also has a BSD stylie license:
 *
 *   Author: Adam Dunkels <adam@dunkels.com>
 *   Copyright (c) 2001-2003, Adam Dunkels.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/net/netdev.h>

#include "devif/devif.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devif_send
 *
 * Description:
 *   Called from socket logic in response to a xmit or poll request from the
 *   the network interface driver.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

int devif_send(FAR struct net_driver_s *dev, FAR const void *buf,
               int len, unsigned int offset)
{
  int ret;

  if (dev == NULL)
    {
      ret = -ENODEV;
      goto errout;
    }

  if (len == 0)
    {
      ret = -EINVAL;
      goto errout;
    }

#ifndef CONFIG_NET_IPFRAG
  if (len > NETDEV_PKTSIZE(dev) - NET_LL_HDRLEN(dev) - offset)
    {
      ret = -EMSGSIZE;
      goto errout;
    }
#endif

  /* Append the send buffer after device buffer */

  if (len > iob_navail(false) * CONFIG_IOB_BUFSIZE ||
      netdev_iob_prepare(dev, false, 0) != OK)
    {
      ret = -ENOMEM;
      goto errout;
    }

  /* Prepare device buffer before poll callback */

  iob_update_pktlen(dev->d_iob, offset);

  ret = iob_trycopyin(dev->d_iob, buf, len, offset, false);
  if (ret != len)
    {
      netdev_iob_release(dev);
      goto errout;
    }

  dev->d_sndlen = len;

  return dev->d_sndlen;

errout:
  nerr("ERROR: devif_send error: %d\n", ret);
  return ret;
}
