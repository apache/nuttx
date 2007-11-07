/****************************************************************************
 * net/uip/uip-tcpcallback.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
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
#ifdef CONFIG_NET

#include <sys/types.h>
#include <debug.h>

#include <net/uip/uipopt.h>
#include <net/uip/uip.h>
#include <net/uip/uip-arch.h>

#include "uip-internal.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: uip_tcpcallback
 *
 * Description:
 *   Inform the application holding the TCP socket of a change in state.
 *
 * Assumptions:
 *   This function is called at the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

void uip_tcpcallback(struct uip_driver_s *dev)
{
  vdbg("uip_flags: %02x\n", uip_flags);

  /* Some sanity checking */

  if (uip_conn)
    {
      /* Check if there is a data callback */

      if (uip_conn->data_event)
      {
        /* Perform the callback */

        uip_conn->data_event(dev, uip_conn->data_private);
      }

      /* Check if there is a connection-related event and a connection
       * callback.
       */
      if (((uip_flags & UIP_CONN_EVENTS) != 0) && uip_conn->connection_event)
        {
          /* Perform the callback */

          uip_conn->connection_event(uip_conn->connection_private);
        }
    }
}

#endif /* CONFIG_NET */
