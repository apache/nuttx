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

#ifdef linux

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <string.h>
#include <sched.h>
#include <nuttx/net.h>

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

#ifdef CONFIG_NET_PROMISCUOUS
# define uipdriver_comparemac(a,b) (0)
#else
static inline int uip_comparemac(struct uip_eth_addr *paddr1, struct uip_eth_addr *paddr2)
{
  return memcmp(paddr1, paddr2, sizeof(struct uip_eth_addr));
}
#endif

static int sim_uiptxpoll(struct uip_driver_s *dev)
{
  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (g_sim_dev.d_len > 0)
    {
      uip_arp_out(&g_sim_dev);
      tapdev_send(g_sim_dev.d_buf, g_sim_dev.d_len);
    }

  /* If zero is returned, the polling will continue until all connections have
   * been examined.
   */

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void uipdriver_loop(void)
{
  int i;

  /* tapdev_read will return 0 on a timeout event and >0 on a data received event */

  g_sim_dev.d_len = tapdev_read((unsigned char*)g_sim_dev.d_buf, UIP_BUFSIZE);

  /* Disable preemption through to the following so that it behaves a little more
   * like an interrupt (otherwise, the following logic gets pre-empted an behaves
   * oddly.
   */

  sched_lock();
  if (g_sim_dev.d_len > 0)
    {
      /* Data received event.  Check for valid Ethernet header with destination == our
       * MAC address
       */

      if (g_sim_dev.d_len > UIP_LLH_LEN && uip_comparemac( &BUF->dest, &g_sim_dev.d_mac) == 0)
        {
          /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv6
          if (BUF->type == htons(UIP_ETHTYPE_IP6))
#else
          if (BUF->type == htons(UIP_ETHTYPE_IP))
#endif
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
                  tapdev_send(g_sim_dev.d_buf, g_sim_dev.d_len);
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
                  tapdev_send(g_sim_dev.d_buf, g_sim_dev.d_len);
                }
            }
        }
    }

  /* Otherwise, it must be a timeout event */

  else if (timer_expired(&g_periodic_timer))
    {
      timer_reset(&g_periodic_timer);
      uip_poll(&g_sim_dev, sim_uiptxpoll, UIP_DRV_TIMER);
    }
  sched_unlock();
}

int uipdriver_init(void)
{
  /* Internal initalization */

  timer_set(&g_periodic_timer, 500);
  tapdev_init();
  (void)tapdev_getmacaddr(g_sim_dev.d_mac.addr);

  /* Register the device with the OS so that socket IOCTLs can be performed */

  (void)netdev_register(&g_sim_dev);
  return OK;
}

#endif /* linux */

