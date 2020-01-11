/****************************************************************************
 * net/bluetooth/bluetooth_poll.c
 * Poll for the availability of ougoing Bluetooth frames
 *
 *   Copyright (C) 2018, 2020 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <assert.h>
#include <debug.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/radiodev.h>
#include <nuttx/net/bluetooth.h>

#include "devif/devif.h"
#include "bluetooth/bluetooth.h"

#ifdef CONFIG_NET_BLUETOOTH

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bluetooth_poll
 *
 * Description:
 *   Poll a packet "connection" structure for availability of TX data
 *
 * Input Parameters:
 *   dev - The device driver structure to use in the send operation
 *   conn - The packet "connection" to poll for TX data
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

void bluetooth_poll(FAR struct net_driver_s *dev,
                     FAR struct bluetooth_conn_s *conn)
{
  FAR struct radio_driver_s *radio;

  DEBUGASSERT(dev != NULL && conn != NULL);
  radio = (FAR struct radio_driver_s *)dev;

  /* Verify that the packet connection is valid */

  if (conn != NULL)
    {
      /* Setup for the application callback (NOTE:  These should not be
       * used by PF_BLUETOOTH sockets).
       */

      radio->r_dev.d_appdata = radio->r_dev.d_buf;
      radio->r_dev.d_len     = 0;
      radio->r_dev.d_sndlen  = 0;

      /* Perform the application callback.
       *
       * REVISIT: Need to pass the meta data and the IOB through the callback
       */

#warning Missing logic

      /* Perform the application callback */

      bluetooth_callback(radio, conn, BLUETOOTH_POLL);
    }
}

#endif /* CONFIG_NET && CONFIG_NET_PKT */
