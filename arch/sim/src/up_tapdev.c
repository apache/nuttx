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

#ifdef linux

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

#include <linux/if.h>
#include <linux/if_tun.h>

extern int lib_rawprintf(const char *format, ...);

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

#define TAPDEV_DEBUG    1

#define DEVTAP          "/dev/net/tun"

#define UIP_DRIPADDR0   192
#define UIP_DRIPADDR1   168
#define UIP_DRIPADDR2   0
#define UIP_DRIPADDR3   1

#define READ   3
#define WRITE  4
#define OPEN   5
#define IOCTL  54
#define SELECT 82

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
static int gdrop = 0;
#endif
static int gtapdevfd;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* This is REALLY awkward.. we need to compile using the system header files,
 * but we can't use any of the libc calls because all of the symbols are
 * defined for NuttX (read, write, etc)!  So we do hand rolled syscalls
 * to get to the Linux functions.
 */

static inline int up_open(const char *filename, int flags, int mode)
{
  int result;

  __asm__ volatile ("int $0x80" \
                    : "=a" (result) \
                    : "0" (OPEN), "b" ((int)(filename)), "c" ((int)(flags)), "d" ((int)(mode)) \
                    : "memory");

  return (int)result;
}

static inline int up_read(int fd, void* buf, size_t count)
{
  ssize_t result;

  __asm__ volatile ("int $0x80" \
                    : "=a" (result) \
                    : "0" (READ), "b" ((int)(fd)), "c" ((int)(buf)), "d" ((int)(count)) \
                    : "memory");

  return (int)result;
}

static inline int up_write(int fd, const void* buf, size_t count)
{
  ssize_t result;

  __asm__ volatile ("int $0x80" \
                    : "=a" (result) \
                    : "0" (WRITE), "b" ((int)(fd)), "c" ((int)(buf)), "d" ((int)(count)) \
                    : "memory");

  return (int)result;
}

static inline int up_ioctl(int fd, unsigned int cmd, unsigned long arg)
{
  ssize_t result;

  __asm__ volatile ("int $0x80" \
                    : "=a" (result) \
                    : "0" (IOCTL), "b" ((int)(fd)), "c" ((int)(cmd)), "d" ((long)(arg)) \
                    : "memory");

  return (int)result;
}

static inline int up_select(int n, fd_set *inp, fd_set *outp, fd_set *exp, struct timeval *tvp)
{
  ssize_t result;

  __asm__ volatile ("int $0x80" \
                    : "=a" (result) \
                    : "0" (SELECT),"b" ((long)(n)),"c" ((long)(inp)), \
                      "d" ((long)(outp)),"S" ((long)(exp)), "D"((long)tvp) \
                    : "memory");

  return (int)result;
}

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
  int ret;

  gtapdevfd = up_open(DEVTAP, O_RDWR, 0644);
  if (gtapdevfd < 0)
    {
      lib_rawprintf("TAPDEV: open failed: %d\n", -gtapdevfd );
      return;
    }

#ifdef linux
  {
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    ifr.ifr_flags = IFF_TAP|IFF_NO_PI;
    ret = up_ioctl(gtapdevfd, TUNSETIFF, (unsigned long *) &ifr);
    if (ret < 0)
      {
        lib_rawprintf("TAPDEV: ioctl failed: %d\n", -ret );
        return;
      }
  }
#endif /* Linux */

  snprintf(buf, sizeof(buf), "ifconfig tap0 inet %d.%d.%d.%d\n",
           UIP_DRIPADDR0, UIP_DRIPADDR1, UIP_DRIPADDR2, UIP_DRIPADDR3);
  system(buf);
}

unsigned int tapdev_read(unsigned char *buf, unsigned int buflen)
{
  fd_set fdset;
  struct timeval tv;
  int ret;

  /* We can't do anything if we failed to open the tap device */
  if (gtapdevfd < 0)
    {
      return 0;
    }

  tv.tv_sec = 0;
  tv.tv_usec = 1000;

  FD_ZERO(&fdset);
  FD_SET(gtapdevfd, &fdset);

  ret = up_select(gtapdevfd + 1, &fdset, NULL, NULL, &tv);
  if(ret == 0)
    {
      return 0;
    }

  ret = up_read(gtapdevfd, buf, buflen);
  if (ret < 0)
    {
      lib_rawprintf("TAPDEV: read failed: %d\n", -ret);
      return 0;
    }

#ifdef TAPDEV_DEBUG
  lib_rawprintf("TAPDEV: read %d bytes\n", ret);
  {
    int i;
    for(i = 0; i < 20; i++)
      {
        lib_rawprintf("%02x ", buf[i]);
      }
    lib_rawprintf("\n");
  }
#endif

  return ret;
}

void tapdev_send(char *buf, unsigned int buflen)
{
  int ret;
#ifdef TAPDEV_DEBUG
  lib_rawprintf("tapdev_send: sending %d bytes\n", buflen);

  gdrop++;
  if(gdrop % 8 == 7)
    {
      lib_rawprintf("Dropped a packet!\n");
      return;
    }
#endif

  ret = up_write(gtapdevfd, buf, buflen);
  if (ret < 0)
    {
      lib_rawprintf("TAPDEV: write");
      exit(1);
    }
}

#endif /* linux */


