/****************************************************************************
 * net/neighbor/neighbor_initialize.c
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
#include <nuttx/clock.h>

#include "neighbor/neighbor.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* This is the Neighbor table.  The network should be locked when accessing
 * this table.
 */

struct neighbor_entry g_neighbors[CONFIG_NET_IPv6_NCONF_ENTRIES];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: neighbor_initialize
 *
 * Description:
 *   Initialize Neighbor table data structures.  This function is called
 *   prior to platform-specific driver initialization so that the networking
 *   subsystem is prepared to deal with network driver initialization
 *   actions.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void neighbor_initialize(void)
{
  int i;

  for (i = 0; i < CONFIG_NET_IPv6_NCONF_ENTRIES; ++i)
    {
      g_neighbors[i].ne_time = NEIGHBOR_MAXTIME;
    }
}
