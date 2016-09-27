/****************************************************************************
 * net/arp/arp_dump.c
 *
 *   Copyright (C) 2007-2011, 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Based on uIP which also has a BSD style license:
 *
 *   Author: Adam Dunkels <adam@dunkels.com>
 *   Copyright (c) 2001-2003, Adam Dunkels.
 *   All rights reserved.
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

#include <debug.h>

#include "arp/arp.h"

#ifdef CONFIG_NET_ARP_DUMP

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arp_dump
 *
 * Description:
 *   Dump the contents of an ARP packet to the SYSLOG device
 *
 * Input Parameters:
 *   arp - A reference to the ARP header to be dumped.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void arp_dump(FAR struct arp_hdr_s *arp)
{
  ninfo("  HW type: %04x Protocol: %04x\n",
        arp->ah_hwtype, arp->ah_protocol);
  ninfo("  HW len: %02x Proto len: %02x Operation: %04x\n",
        arp->ah_hwlen, arp->ah_protolen, arp->ah_opcode);
  ninfo("  Sender MAC: %02x:%02x:%02x:%02x:%02x:%02x IP: %d.%d.%d.%d\n",
        arp->ah_shwaddr[0], arp->ah_shwaddr[1], arp->ah_shwaddr[2],
        arp->ah_shwaddr[3], arp->ah_shwaddr[4], arp->ah_shwaddr[5],
        arp->ah_sipaddr[0] & 0xff, arp->ah_sipaddr[0] >> 8,
        arp->ah_sipaddr[1] & 0xff, arp->ah_sipaddr[1] >> 8);
  ninfo("  Dest MAC:   %02x:%02x:%02x:%02x:%02x:%02x IP: %d.%d.%d.%d\n",
        arp->ah_dhwaddr[0], arp->ah_dhwaddr[1], arp->ah_dhwaddr[2],
        arp->ah_dhwaddr[3], arp->ah_dhwaddr[4], arp->ah_dhwaddr[5],
        arp->ah_dipaddr[0] & 0xff, arp->ah_dipaddr[0] >> 8,
        arp->ah_dipaddr[1] & 0xff, arp->ah_dipaddr[1] >> 8);
}

#endif /* CONFIG_NET_ARP_DUMP */
