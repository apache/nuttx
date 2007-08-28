/****************************************************************************
 * up_tapdev.c
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

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/uio.h>
#include <sys/socket.h>

#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>

#if 0
#include "uip/uip.h"
#include <net/uip/uip-arch.h>
#include <net/uip/uip-arp.h>
#endif

#ifdef linux
# include <sys/ioctl.h>
# include <linux/if.h>
# include <linux/if_tun.h>
# define DEVTAP "/dev/net/tun"
#else  /* linux */
# define DEVTAP "/dev/tap0"
#endif /* linux */

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

#define TAPDEV_DEBUG    1

#define UIP_DRIPADDR0   192
#define UIP_DRIPADDR1   168
#define UIP_DRIPADDR2   0
#define UIP_DRIPADDR3   1

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef TAPDEV_DEBUG
static int drop = 0;
#endif
static int fd;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

unsigned long up_getwalltime( void )
{
  struct timeval tm;
  (void)gettimeofday(&tm, NULL);
  return tm.tv_sec*1000 + tm.tv_usec/1000;
}

void tapdev_init(void)
{
  char buf[1024];

  fd = open(DEVTAP, O_RDWR);
  if(fd == -1)
    {
      printf("tapdev: tapdev_init: open");
      return;
    }

#ifdef linux
  {
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    ifr.ifr_flags = IFF_TAP|IFF_NO_PI;
    if (ioctl(fd, TUNSETIFF, (void *) &ifr) < 0)
      {
        printf(buf);
        return;
      }
  }
#endif /* Linux */

  snprintf(buf, sizeof(buf), "ifconfig tap0 inet %d.%d.%d.%d",
	   UIP_DRIPADDR0, UIP_DRIPADDR1, UIP_DRIPADDR2, UIP_DRIPADDR3);
  system(buf);
}

unsigned int tapdev_read(char *buf, unsigned int buflen)
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

  ret = read(fd, buf, buflen);
  if(ret == -1)
    {
      printf("tap_dev: tapdev_read: read");
    }

#ifdef TAPDEV_DEBUG
  printf("tap_dev: tapdev_read: read %d bytes\n", ret);
  {
    int i;
    for(i = 0; i < 20; i++)
      {
        printf("%x ", buf[i]);
      }
    printf("\n");
  }
#endif

  return ret;
}

void tapdev_send(char *buf, unsigned int buflen)
{
  int ret;
#ifdef TAPDEV_DEBUG
  printf("tapdev_send: sending %d bytes\n", buflen);

  drop++;
  if(drop % 8 == 7)
    {
      printf("Dropped a packet!\n");
      return;
    }
#endif

  ret = write(fd, buf, buflen);
  if(ret == -1)
    {
      perror("tap_dev: tapdev_send: write");
      exit(1);
    }
}

