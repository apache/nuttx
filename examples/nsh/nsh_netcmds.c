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
#if defined(CONFIG_NET) && CONFIG_NSOCKET_DESCRIPTORS > 0

#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <sched.h>

#include <nuttx/net.h>
#include <net/ethernet.h>
#include <net/uip/uip.h>
#include <net/uip/uip-arch.h>
#include <netinet/ether.h>

#ifdef CONFIG_NET_STATISTICS
#include <net/uip/uip.h>
#endif

#include "nsh.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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

#endif /* CONFIG_NET && CONFIG_NSOCKET_DESCRIPTORS */
