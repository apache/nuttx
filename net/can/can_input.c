/****************************************************************************
 * net/can/can_input.c
 * Handling incoming packet input
 *
 *   Copyright (C) 2014, 2020 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Adapted for NuttX from logic in uIP which also has a BSD-like license:
 *
 *   Original author Adam Dunkels <adam@dunkels.com>
 *   Copyright () 2001-2003, Adam Dunkels.
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

#include <nuttx/config.h>
#if defined(CONFIG_NET) && defined(CONFIG_NET_CAN)

#include <errno.h>
#include <debug.h>

#include <nuttx/net/netdev.h>
#include <nuttx/net/can.h>

#include "devif/devif.h"
#include "can/can.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: can_input
 *
 * Description:
 *   Handle incoming packet input
 *
 * Input Parameters:
 *   dev - The device driver structure containing the received packet
 *
 * Returned Value:
 *   OK     The packet has been processed  and can be deleted
 *  -EAGAIN There is a matching connection, but could not dispatch the packet
 *          yet.  Useful when a packet arrives before a recv call is in
 *          place.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

int can_input(struct net_driver_s *dev)
{
  FAR struct can_conn_s *conn;
  int ret = OK;

  conn = can_nextconn(NULL); //FIXME 
  if (conn)
    {
      uint16_t flags;

      /* Setup for the application callback */

      dev->d_appdata = dev->d_buf;
      dev->d_sndlen  = 0;

      /* Perform the application callback */

      flags = can_callback(dev, conn, CAN_NEWDATA);

      /* If the operation was successful, the CAN_NEWDATA flag is removed
       * and thus the packet can be deleted (OK will be returned).
       */

      if ((flags & CAN_NEWDATA) != 0)
        {
          /* No.. the packet was not processed now.  Return -EAGAIN so
           * that the driver may retry again later.  We still need to
           * set d_len to zero so that the driver is aware that there
           * is nothing to be sent.
           */

           nwarn("WARNING: Packet not processed\n");
           ret = -EAGAIN;
        }
    }
  else
    {
      ninfo("No CAN listener\n");
    }

  return ret;
}

#endif /* CONFIG_NET && CONFIG_NET_CAN */
