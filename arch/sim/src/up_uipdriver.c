/****************************************************************************
 * up_uipdriver.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * Based on code from uIP which also has a BSD-like license:
 *
 *   Copyright (c) 2001, Adam Dunkels.
 *   All rights reserved.
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
#include <sys/types.h>

#include <net/uip/uip.h>
#include <net/uip/uip-arch.h>
#include <net/uip/uip-arp.h>

#include "up_internal.h"

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

#define BUF ((struct uip_eth_hdr *)&uip_buf[0])

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct timer
{
  uint32 interval;
  uint32 start;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct timer periodic_timer;
static struct timer arp_timer;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void timer_set( struct timer *t, unsigned int interval )
{
  t->interval = interval;
  t->start    = up_getwalltime();
}

static boolean timer_expired( struct timer *t )
{
  return (up_getwalltime() - t->start) >= t->interval;
}

void timer_reset(struct timer *t)
{
  t->start += t->interval;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void uipdriver_loop(void)
{
  int i;

  uip_len = tapdev_read((char*)uip_buf, UIP_BUFSIZE);
  if (uip_len > 0)
    {
      if (BUF->type == htons(UIP_ETHTYPE_IP))
        {
          uip_arp_ipin();
          uip_input();

          /* If the above function invocation resulted in data that
           * should be sent out on the network, the global variable
           * uip_len is set to a value > 0.
           */

          if (uip_len > 0)
            {
              uip_arp_out();
              tapdev_send((char*)uip_buf, uip_len);
            }
        }
      else if (BUF->type == htons(UIP_ETHTYPE_ARP))
        {
          uip_arp_arpin();

          /* If the above function invocation resulted in data that
           * should be sent out on the network, the global variable
           * uip_len is set to a value > 0.
           */

          if (uip_len > 0)
            {
              tapdev_send((char*)uip_buf, uip_len);
            }
        }
    }
  else if (timer_expired(&periodic_timer))
    {
      timer_reset(&periodic_timer);
      for(i = 0; i < UIP_CONNS; i++)
        {
          uip_periodic(i);

          /* If the above function invocation resulted in data that
           * should be sent out on the network, the global variable
           * uip_len is set to a value > 0.
           */

          if (uip_len > 0)
            {
              uip_arp_out();
              tapdev_send((char*)uip_buf, uip_len);
            }
        }

#ifdef CONFIG_NET_UDP
      for(i = 0; i < UIP_UDP_CONNS; i++)
        {
          uip_udp_periodic(i);

          /* If the above function invocation resulted in data that
           * should be sent out on the network, the global variable
           * uip_len is set to a value > 0.
           */

          if (uip_len > 0)
            {
              uip_arp_out();
              tapdev_send((char*)uip_buf, uip_len);
            }
        }
#endif /* CONFIG_NET_UDP */

      /* Call the ARP timer function every 10 seconds. */

      if (timer_expired(&arp_timer))
        {
          timer_reset(&arp_timer);
          uip_arp_timer();
        }
    }
}

int uipdriver_init(void)
{
  timer_set(&periodic_timer, 500);
  timer_set(&arp_timer, 10000 );

  tapdev_init();
  uip_init();
  return OK;
}
