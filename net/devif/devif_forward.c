/****************************************************************************
 * net/devif/devif_forward.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <string.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/mm/iob.h>
#include <nuttx/net/netdev.h>

#include "ipforward/ipforward.h"
#include "devif/devif.h"

#ifdef CONFIG_NET_IPFORWARD

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devif_forward
 *
 * Description:
 *   Called from protocol-specific IP forwarding logic to re-send a packet.
 *
 * Input Parameters:
 *   fwd - An initialized instance of the common forwarding structure that
 *         includes everything needed to perform the forwarding operation.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

void devif_forward(FAR struct forward_s *fwd)
{
  unsigned int offset;
  int ret;

  DEBUGASSERT(fwd != NULL && fwd->f_iob != NULL && fwd->f_dev != NULL);
  offset = NET_LL_HDRLEN(fwd->f_dev);

  /* Copy the IOB chain that contains the L3L3 headers and any data payload */

  DEBUGASSERT(offset + fwd->f_iob->io_pktlen <= NETDEV_PKTSIZE(fwd->f_dev));
  ret = iob_copyout(&fwd->f_dev->d_buf[offset], fwd->f_iob,
                    fwd->f_iob->io_pktlen, 0);

  DEBUGASSERT(ret == fwd->f_iob->io_pktlen);

  fwd->f_dev->d_sndlen = 0;
  fwd->f_dev->d_len    = fwd->f_iob->io_pktlen;

  UNUSED(ret);
}

#endif /* CONFIG_NET_IPFORWARD */
