/****************************************************************************
 * net/pkt/pkt_finddev.c
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_PKT)

#include <nuttx/net/netdev.h>

#include "netdev/netdev.h"
#include "pkt/pkt.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pkt_find_device
 *
 * Description:
 *   Select the network driver to use with the PKT transaction.
 *
 * Input Parameters:
 *   conn - PKT connection structure (not currently used).
 *
 * Returned Value:
 *   A pointer to the network driver to use.
 *
 ****************************************************************************/

FAR struct net_driver_s *pkt_find_device(FAR struct pkt_conn_s *conn)
{
  /* REVISIT:  This is bogus.  A better network device lookup is needed. */

  return netdev_findbyname("eth0");
}

#endif /* CONFIG_NET && CONFIG_NET_PKT */
