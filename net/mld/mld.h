/****************************************************************************
 * net/mld/mld.h
 * Multicast Listener Discovery (MLD) Definitions
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __NET_NETLINK_MLD_H
#define __NET_NETLINK_MLD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <queue.h>
#include <semaphore.h>

#include "devif/devif.h"
#include "socket/socket.h"

#ifdef CONFIG_NET_MLD

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct net_driver_s;                 /* Forward reference */
struct mld_mcast_listen_query_s;     /* Forward reference */
struct mld_mcast_listen_report_v1_s; /* Forward reference */
struct mld_mcast_listen_report_v2_s; /* Forward reference */
struct mld_mcast_listen_done_v1_s;   /* Forward reference */

/****************************************************************************
 * Name: mld_initialize()
 *
 * Description:
 *   Initialize the MLD structures.  Called once and only from the
 *   networking layer.
 *
 ****************************************************************************/

void mld_initialize(void);

/****************************************************************************
 * Name: mld_query
 *
 * Description:
 *  Called from icmpv6_input() when a Multicast Listener Query is received.
 *
 ****************************************************************************/

int mld_query_input(FAR struct net_driver_s *dev,
                    FAR const struct mld_mcast_listen_query_s *query);

/****************************************************************************
 * Name: mld_report_v1
 *
 * Description:
 *  Called from icmpv6_input() when a Version 1 Multicast Listener Report is
 *   received.
 *
 ****************************************************************************/

int mld_report_v1(FAR struct net_driver_s *dev,
                  FAR const struct mld_mcast_listen_report_v1_s *report);

/****************************************************************************
 * Name: mld_report_v2
 *
 * Description:
 *  Called from icmpv6_input() when a Version 2 Multicast Listener Report is
 *   received.
 *
 ****************************************************************************/

int mld_report_v2(FAR struct net_driver_s *dev,
                  FAR const struct mld_mcast_listen_report_v2_s *report);

/****************************************************************************
 * Name: mld_done_v1
 *
 * Description:
 *  Called from icmpv6_input() when a Version 1 Multicast Listener Done is
 *  received.
 *
 ****************************************************************************/

int mld_done_v1(FAR struct net_driver_s *dev,
                FAR const struct mld_mcast_listen_done_v1_s *done);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET_MLD */
#endif /* __NET_NETLINK_MLD_H */
