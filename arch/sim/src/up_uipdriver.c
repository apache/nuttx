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

#define BUF ((struct uip_eth_hdr *)g_sim_dev.d_buf)

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

static struct timer g_periodic_timer;
static struct timer g_arp_timer;
static struct uip_driver_s g_sim_dev;

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

  g_sim_dev.d_len = tapdev_read((unsigned char*)g_sim_dev.d_buf, UIP_BUFSIZE);
  if (g_sim_dev.d_len > 0)
    {
      if (BUF->type == htons(UIP_ETHTYPE_IP))
        {
          uip_arp_ipin();
          uip_input(&g_sim_dev);

          /* If the above function invocation resulted in data that
           * should be sent out on the network, the global variable
           * d_len is set to a value > 0.
           */

          if (g_sim_dev.d_len > 0)
            {
              uip_arp_out(&g_sim_dev);
              tapdev_send((char*)g_sim_dev.d_buf, g_sim_dev.d_len);
            }
        }
      else if (BUF->type == htons(UIP_ETHTYPE_ARP))
        {
          uip_arp_arpin(&g_sim_dev);

          /* If the above function invocation resulted in data that
           * should be sent out on the network, the global variable
           * d_len is set to a value > 0.
           */

          if (g_sim_dev.d_len > 0)
            {
              tapdev_send((char*)g_sim_dev.d_buf, g_sim_dev.d_len);
            }
        }
    }
  else if (timer_expired(&g_periodic_timer))
    {
      timer_reset(&g_periodic_timer);
      for(i = 0; i < UIP_CONNS; i++)
        {
          uip_tcppoll(&g_sim_dev,i);

          /* If the above function invocation resulted in data that
           * should be sent out on the network, the global variable
           * d_len is set to a value > 0.
           */

          if (g_sim_dev.d_len > 0)
            {
              uip_arp_out(&g_sim_dev);
              tapdev_send((char*)g_sim_dev.d_buf, g_sim_dev.d_len);
            }
        }

#ifdef CONFIG_NET_UDP
      for(i = 0; i < UIP_UDP_CONNS; i++)
        {
          uip_udppoll(&g_sim_dev,i);

          /* If the above function invocation resulted in data that
           * should be sent out on the network, the global variable
           * d_len is set to a value > 0.
           */

          if (g_sim_dev.d_len > 0)
            {
              uip_arp_out(&g_sim_dev);
              tapdev_send((char*)g_sim_dev.d_buf, g_sim_dev.d_len);
            }
        }
#endif /* CONFIG_NET_UDP */

      /* Call the ARP timer function every 10 seconds. */

      if (timer_expired(&g_arp_timer))
        {
          timer_reset(&g_arp_timer);
          uip_arp_timer();
        }
    }
}

int uipdriver_init(void)
{
  /* Internal initalization */

  timer_set(&g_periodic_timer, 500);
  timer_set(&g_arp_timer, 10000 );
  tapdev_init();

  /* Register the device with the OS so that socket IOCTLs can be performed */

  (void)netdev_register(&g_sim_dev);
  return OK;
}
