/****************************************************************************
 * net/neighbor/neighbor.c
 *
 *   Copyright (C) 2007-2009, 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * A leverage of logic from uIP which also has a BSD style license
 *
 *   Copyright (c) 2006, Swedish Institute of Computer Science.  All rights
 *     reserved.
 *   Author: Adam Dunkels <adam@sics.se>
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
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>

#include <nuttx/net/ip.h>

#include "neighbor/neighbor.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  neighbor_lookup
 *
 * Description:
 *   Find an entry in the Neighbor Table and return its link layer address.
 *
 * Input Parameters:
 *   ipaddr - The IPv6 address to use in the lookup;
 *   lladdr - The location to return the link layer address
 *
 * Returned Value:
 *   Returns OK if the address was successfully obtain; a negated errno
 *   value is returned on failure.
 *
 ****************************************************************************/

FAR const struct neighbor_addr_s *neighbor_lookup(const net_ipv6addr_t ipaddr)
{
  FAR struct neighbor_entry *neighbor;

  neighbor = neighbor_findentry(ipaddr);
  if (neighbor != NULL)
    {
      return &neighbor->ne_addr;
    }

  return NULL;
}
