/****************************************************************************
 * net/neighbor/neighbor_update.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "neighbor/neighbor.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: neighbor_update
 *
 * Description:
 *   Reset time on the Neighbor Table entry associated with the IPv6 address.
 *   This makes the associated entry the most recently used and not a
 *   candidate for removal.
 *
 * Input Parameters:
 *   ipaddr - The IPv6 address of the entry to be updated
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void neighbor_update(const net_ipv6addr_t ipaddr)
{
  struct neighbor_entry_s *neighbor;

  neighbor = neighbor_findentry(ipaddr);
  if (neighbor != NULL)
    {
      neighbor->ne_time = clock_systime_ticks();
    }
}
