/****************************************************************************
 * examples/nsh/nsh_netcmds.c
 *
 *   Copyright (C) 2007, 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sched.h>

#include <nuttx/net.h>
#include <nuttx/clock.h>
#include <net/ethernet.h>
#include <net/uip/uip.h>
#include <net/uip/uip-arch.h>
#include <netinet/ether.h>

#ifdef CONFIG_NET_STATISTICS
#include <net/uip/uip.h>
#endif

#if defined(CONFIG_NET_ICMP) && defined(CONFIG_NET_ICMP_PING) && \
   !defined(CONFIG_DISABLE_CLOCK) && !defined(CONFIG_DISABLE_SIGNALS)
#include <net/uip/uip-lib.h>
#endif

#include "nsh.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define DEFAULT_PING_DATALEN 56

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_NET_ICMP) && defined(CONFIG_NET_ICMP_PING) && \
   !defined(CONFIG_DISABLE_CLOCK) && !defined(CONFIG_DISABLE_SIGNALS)
static uint16 g_pingid = 0;
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ping_newid
 ****************************************************************************/

#if defined(CONFIG_NET_ICMP) && defined(CONFIG_NET_ICMP_PING) && \
   !defined(CONFIG_DISABLE_CLOCK) && !defined(CONFIG_DISABLE_SIGNALS)
static inline uint16 ping_newid(void)
{
  irqstate_t save = irqsave();
  uint16 ret = ++g_pingid;
  irqrestore(save);
  return ret;
}
#endif

/****************************************************************************
 * Name: uip_statistics
 ****************************************************************************/

#ifdef CONFIG_NET_STATISTICS
static inline void uip_statistics(FAR struct nsh_vtbl_s *vtbl)
{
  nsh_output(vtbl, "uIP         IP ");
#ifdef CONFIG_NET_TCP
  nsh_output(vtbl, "  TCP");
#endif
#ifdef CONFIG_NET_UDP
  nsh_output(vtbl, "  UDP");
#endif
#ifdef CONFIG_NET_ICMP
  nsh_output(vtbl, "  ICMP");
#endif
  nsh_output(vtbl, "\n");

  /* Received packets */

  nsh_output(vtbl, "Received    %04x",uip_stat.ip.recv);
#ifdef CONFIG_NET_TCP
  nsh_output(vtbl, " %04x",uip_stat.tcp.recv);
#endif
#ifdef CONFIG_NET_UDP
  nsh_output(vtbl, " %04x",uip_stat.udp.recv);
#endif
#ifdef CONFIG_NET_ICMP
  nsh_output(vtbl, " %04x",uip_stat.icmp.recv);
#endif
  nsh_output(vtbl, "\n");

  /* Dropped packets */

  nsh_output(vtbl, "Dropped     %04x",uip_stat.ip.drop);
#ifdef CONFIG_NET_TCP
  nsh_output(vtbl, " %04x",uip_stat.tcp.drop);
#endif
#ifdef CONFIG_NET_UDP
  nsh_output(vtbl, " %04x",uip_stat.udp.drop);
#endif
#ifdef CONFIG_NET_ICMP
  nsh_output(vtbl, " %04x",uip_stat.icmp.drop);
#endif
  nsh_output(vtbl, "\n");

  nsh_output(vtbl, "  IP        VHL: %04x HBL: %04x\n",
             uip_stat.ip.vhlerr, uip_stat.ip.hblenerr);
  nsh_output(vtbl, "            LBL: %04x Frg: %04x\n",
             uip_stat.ip.lblenerr, uip_stat.ip.fragerr);

  nsh_output(vtbl, "  Checksum  %04x",uip_stat.ip.chkerr);
#ifdef CONFIG_NET_TCP
  nsh_output(vtbl, " %04x",uip_stat.tcp.chkerr);
#endif
#ifdef CONFIG_NET_UDP
  nsh_output(vtbl, " %04x",uip_stat.udp.chkerr);
#endif
#ifdef CONFIG_NET_ICMP
  nsh_output(vtbl, " ----");
#endif
  nsh_output(vtbl, "\n");

#ifdef CONFIG_NET_TCP
  nsh_output(vtbl, "  TCP       ACK: %04x SYN: %04x\n", 
            uip_stat.tcp.ackerr, uip_stat.tcp.syndrop);
  nsh_output(vtbl, "            RST: %04x %04x\n", 
            uip_stat.tcp.rst, uip_stat.tcp.synrst);
#endif

  nsh_output(vtbl, "  Type      %04x",uip_stat.ip.protoerr);
#ifdef CONFIG_NET_TCP
  nsh_output(vtbl, " ----");
#endif
#ifdef CONFIG_NET_UDP
  nsh_output(vtbl, " ----");
#endif
#ifdef CONFIG_NET_ICMP
  nsh_output(vtbl, " %04x",uip_stat.icmp.typeerr);
#endif
  nsh_output(vtbl, "\n");

  /* Sent packets */

  nsh_output(vtbl, "Sent        ----",uip_stat.ip.sent);
#ifdef CONFIG_NET_TCP
  nsh_output(vtbl, " %04x",uip_stat.tcp.sent);
#endif
#ifdef CONFIG_NET_UDP
  nsh_output(vtbl, " %04x",uip_stat.udp.sent);
#endif
#ifdef CONFIG_NET_ICMP
  nsh_output(vtbl, " %04x",uip_stat.icmp.sent);
#endif
  nsh_output(vtbl, "\n");

#ifdef CONFIG_NET_TCP
  nsh_output(vtbl, "  Rexmit    ---- %04x",uip_stat.tcp.rexmit);
#ifdef CONFIG_NET_UDP
  nsh_output(vtbl, " ----");
#endif
#ifdef CONFIG_NET_ICMP
   nsh_output(vtbl, " ----");
#endif
  nsh_output(vtbl, "\n");
#endif
  nsh_output(vtbl, "\n");
}
#else
# define uip_statistics(vtbl)
#endif

/****************************************************************************
 * Name: ifconfig_callback
 ****************************************************************************/

int ifconfig_callback(FAR struct uip_driver_s *dev, void *arg)
{
  struct nsh_vtbl_s *vtbl = (struct nsh_vtbl_s*)arg;
  struct in_addr addr;

  nsh_output(vtbl, "%s\tHWaddr %s\n", dev->d_ifname, ether_ntoa(&dev->d_mac));
  addr.s_addr = dev->d_ipaddr;
  nsh_output(vtbl, "\tIPaddr:%s ", inet_ntoa(addr));
  addr.s_addr = dev->d_draddr;
  nsh_output(vtbl, "DRaddr:%s ", inet_ntoa(addr));
  addr.s_addr = dev->d_netmask;
  nsh_output(vtbl, "Mask:%s\n\n", inet_ntoa(addr));
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cmd_ifconfig
 ****************************************************************************/

int cmd_ifconfig(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  netdev_foreach(ifconfig_callback, vtbl);
  uip_statistics(vtbl);
  return OK;
}

/****************************************************************************
 * Name: cmd_ping
 ****************************************************************************/

#if defined(CONFIG_NET_ICMP) && defined(CONFIG_NET_ICMP_PING) && \
   !defined(CONFIG_DISABLE_CLOCK) && !defined(CONFIG_DISABLE_SIGNALS)
int cmd_ping(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  FAR const char *fmt = g_fmtarginvalid;
  const char *staddr;
  uip_ipaddr_t ipaddr;
  uint32 start;
  uint32 next;
  uint32 dsec  = 10;
  uint16 id;
  int count    = 10;
  int option;
  int seqno;
  int replies  = 0;
  int elapsed;
  int tmp;
  int i;

  /* Get the ping options */

  while ((option = getopt(argc, argv, ":c:i:")) != ERROR)
    {
      switch (option)
        {
          case 'c':
            count = atoi(optarg);
            if (count < 1 || count > 10000)
              {
                fmt = g_fmtargrange;
                goto errout;
              }
            break;

          case 'i':
            tmp = atoi(optarg);
            if (tmp < 1 || tmp >= 4294)
              {
                fmt = g_fmtargrange;
                goto errout;
              }
            dsec = 10 * tmp;
            break;

          case ':':
            fmt = g_fmtargrequired;

          case '?':
          default:
            goto errout;
        }
    }

  /* There should be exactly on parameter left on the command-line */

  if (optind == argc-1)
    {
      staddr = argv[optind];
      if (!uiplib_ipaddrconv(staddr, (FAR unsigned char*)&ipaddr))
        {
          goto errout;
        }
    }
  else if (optind >= argc)
    {
      fmt = g_fmttoomanyargs;
      goto errout;
    }
  else
    {
      fmt = g_fmtargrequired;
      goto errout;
    }

  /* Get the ID to use */

  id = ping_newid();

  /* Loop for the specified count */

  nsh_output(vtbl, "PING %s %d bytes of data\n", staddr, DEFAULT_PING_DATALEN);
  start = g_system_timer;
  for (i = 1; i <= count; i++)
    {
      /* Send the ECHO request and wait for the response */

      next  = g_system_timer;
      seqno = uip_ping(ipaddr, id, i, DEFAULT_PING_DATALEN, dsec);

      /* Was any response returned? We can tell if a non-negative sequence
       * number was returned.
       */

      if (seqno >= 0 && seqno <= i)
        {
          /* Get the elpased time from the time that the request was
           * sent until the response was received.  If we got a response
           * to an earlier request, then fudge the elpased time.
           */

          elapsed = TICK2MSEC(g_system_timer - next);
          if (seqno < i)
            {
              elapsed += 100*dsec*(i - seqno);
            }

          /* Report the receipt of the reply */

          nsh_output(vtbl, "%d bytes from %s: icmp_seq=%d time=%d ms\n",
                     DEFAULT_PING_DATALEN, staddr, seqno, elapsed);
          replies++;
        }

      /* Wait for the remainder of the interval.  If the last seqno<i,
       * then this is a bad idea... we will probably lose the response
       * to the current request!
       */

      elapsed = TICK2DSEC(g_system_timer - next);
      if (elapsed < dsec)
        {
          usleep(100000*dsec);
        }
    }

  /* Get the total elapsed time */

  elapsed = TICK2MSEC(g_system_timer - start);

  /* Calculate the percentage of lost packets */

  tmp = (100*(count - replies) + (count >> 1)) / count;

  nsh_output(vtbl, "%d packets transmitted, %d received, %d%% packet loss, time %d ms\n",
             count, replies, tmp, elapsed);
  return OK;

errout:
  nsh_output(vtbl, fmt, argv[0]);
  return ERROR;
}
#endif /* CONFIG_NET_ICMP && CONFIG_NET_ICMP_PING */
#endif /* CONFIG_NET */
