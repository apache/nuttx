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
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/uio.h>
#include <sys/socket.h>

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <debug.h>

#include <net/uip/uip.h>
#include <net/uip/uip-arch.h>
#include <net/uip/uip-arp.h>

#ifdef linux
# include <sys/ioctl.h>
# include <linux/if.h>
# include <linux/if_tun.h>
# define DEVTAP "/dev/net/tun"
#else  /* linux */
# define DEVTAP "/dev/tap0"
#endif /* linux */

#include "up_internal.h"

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

#define BUF ((struct uip_eth_hdr *)&uip_buf[0])

#define UIP_DRIPADDR0   192
#define UIP_DRIPADDR1   168
#define UIP_DRIPADDR2   0
#define UIP_DRIPADDR3   1

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

#ifdef CONFIG_DEBUG
static int drop = 0;
#endif
static int fd;
static struct timer periodic_timer;
static struct timer arp_timer;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32 gettime( void )
{
  struct timespec tm;
  (void)clock_gettime(CLOCK_REALTIME, &tm);
  return tm.tv_sec*1000 + tm.tv_nsec/1000000;
}

static void timer_set( struct timer *t, unsigned int interval )
{
  t->interval = interval;
  t->start    = gettime();
}

static boolean timer_expired( struct timer *t )
{
  return (gettime() - t->start) >= t->interval;
}

void timer_reset(struct timer *t)
{
  t->start += t->interval;
}

static void tapdev_init(void)
{
  char buf[1024];

  fd = open(DEVTAP, O_RDWR);
  if(fd == -1)
    {
      lib_rawprintf("tapdev: tapdev_init: open");
      return;
    }

#ifdef linux
  {
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    ifr.ifr_flags = IFF_TAP|IFF_NO_PI;
    if (ioctl(fd, TUNSETIFF, (void *) &ifr) < 0)
      {
        lib_rawprintf(buf);
        return;
      }
  }
#endif /* Linux */

  snprintf(buf, sizeof(buf), "ifconfig tap0 inet %d.%d.%d.%d",
	   UIP_DRIPADDR0, UIP_DRIPADDR1, UIP_DRIPADDR2, UIP_DRIPADDR3);
  system(buf);
}

static unsigned int tapdev_read(void)
{
  fd_set fdset;
  struct timeval tv;
  int ret;

  tv.tv_sec = 0;
  tv.tv_usec = 1000;

  FD_ZERO(&fdset);
  FD_SET(fd, &fdset);

  ret = select(fd + 1, &fdset, NULL, NULL, &tv);
  if(ret == 0)
    {
      return 0;
    }
  ret = read(fd, uip_buf, UIP_BUFSIZE);
  if(ret == -1)
    {
      lib_rawprintf("tap_dev: tapdev_read: read");
    }

  dbg("tap_dev: tapdev_read: read %d bytes\n", ret);
  {
    int i;
    for(i = 0; i < 20; i++)
      {
        vdbg("%x ", uip_buf[i]);
      }
    vdbg("\n");
  }

#ifdef CONFIG_DEBUG
  check_checksum(uip_buf, ret);
#endif
  return ret;
}

static void tapdev_send(void)
{
  int ret;
#ifdef CONFIG_DEBUG
  dbg("tapdev_send: sending %d bytes\n", uip_len);
  check_checksum(uip_buf, uip_len);

  drop++;
  if(drop % 8 == 7)
    {
      dbg("Dropped a packet!\n");
      return;
    }
#endif

  ret = write(fd, uip_buf, uip_len);
  if(ret == -1)
    {
      perror("tap_dev: tapdev_send: writev");
      exit(1);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void uipdriver_loop(void)
{
  int i;

  uip_len = tapdev_read();
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
              tapdev_send();
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
              tapdev_send();
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
              tapdev_send();
            }
        }

#if UIP_UDP
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
              tapdev_send();
            }
        }
#endif /* UIP_UDP */

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
  timer_set(&periodic_timer, CLK_TCK / 2);
  timer_set(&arp_timer, CLK_TCK * 10);

  tapdev_init();
  uip_init();
  return OK;
}
