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

/* State transition diagram for a router in Querier state (RFC 2710):
 *                            ________________
 *                           |                |
 *                           |                |timer expired
 *              timer expired|                |(notify routing -,
 *         (notify routing -)|  No Listeners  |clear rxmt tmr)
 *                   ------->|    Present     |<---------
 *                  |        |                |          |
 *                  |        |                |          |
 *                  |        |________________|          |  ---------------
 *                  |                    |               | | rexmt timer   |
 *                  |     report received|               | |  expired      |
 *                  |  (notify routing +,|               | | (send m-a-s   |
 *                  |        start timer)|               | |  query,       |
 *        __________|______              |       ________|_|______ st rxmt |
 *       |                 |<------------       |                 | tmr)   |
 *       |                 |                    |                 |<-------
 *       |                 | report received    |                 |
 *       |                 | (start timer,      |                 |
 *       |                 |  clear rxmt tmr)   |                 |
 *       |    Listeners    |<-------------------|    Checking     |
 *       |     Present     | done received      |    Listeners    |
 *       |                 | (start timer*,     |                 |
 *       |                 |  start rxmt timer, |                 |
 *       |                 |  send m-a-s query) |                 |
 *   --->|                 |------------------->|                 |
 *  |    |_________________|                    |_________________|
 *  | report received |
 *  | (start timer)   |
 *   -----------------
 *
 * State transition diagram for a router in Non-Querier state is
 * similar, but non-Queriers do not send any messages and are only
 * driven by message reception.
 *
 *                             ________________
 *                            |                |
 *                            |                |
 *               timer expired|                |timer expired
 *          (notify routing -)|  No Listeners  |(notify routing -)
 *                  --------->|    Present     |<---------
 *                 |          |                |          |
 *                 |          |                |          |
 *                 |          |                |          |
 *                 |          |________________|          |
 *                 |                   |                  |
 *                 |                   |report received   |
 *                 |                   |(notify routing +,|
 *                 |                   | start timer)     |
 *         ________|________           |          ________|________
 *        |                 |<---------          |                 |
 *        |                 |  report received   |                 |
 *        |                 |  (start timer)     |                 |
 *        |    Listeners    |<-------------------|     Checking    |
 *        |     Present     | m-a-s query rec'd  |    Listeners    |
 *        |                 | (start timer*)     |                 |
 *   ---->|                 |------------------->|                 |
 *  |     |_________________|                    |_________________|
 *  | report received |
 *  | (start timer)   |
 *   -----------------
 */

#ifndef __NET_NETLINK_MLD_H
#define __NET_NETLINK_MLD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <queue.h>
#include <semaphore.h>

#include <nuttx/wqueue.h>
#include <nuttx/net/ip.h>

#include "devif/devif.h"
#include "socket/socket.h"

#ifdef CONFIG_NET_MLD

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Group flags */

#define MLD_QUERIER              (1 << 0)  /* Querier */
#define MLD_STARTUP              (1 << 2)  /* Startup unsolicited Reports */
#define MLD_V1COMPAT             (1 << 3)  /* Version 1 compatibility mode */
#define MLD_LASTREPORT           (1 << 3)  /* We were the last to report */
#define MLD_SCHEDMSG             (1 << 4)  /* Outgoing message scheduled */
#define MLD_WAITMSG              (1 << 5)  /* Block until message sent */

#define SET_MLD_QUERIER(f)       do { (f) |= MLD_QUERIER; } while (0)
#define SET_MLD_STARTUP(f)       do { (f) |= MLD_STARTUP; } while (0)
#define SET_MLD_V1COMPAT(f)      do { (f) |= MLD_V1COMPAT; } while (0)
#define SET_MLD_LASTREPORT(f)    do { (f) |= MLD_LASTREPORT; } while (0)
#define SET_MLD_SCHEDMSG(f)      do { (f) |= MLD_SCHEDMSG; } while (0)
#define SET_MLD_WAITMSG(f)       do { (f) |= MLD_WAITMSG; } while (0)

#define CLR_MLD_QUERIER(f)       do { (f) &= ~MLD_QUERIER; } while (0)
#define CLR_MLD_STARTUP(f)       do { (f) &= ~MLD_STARTUP; } while (0)
#define CLR_MLD_V1COMPAT(f)      do { (f) &= ~MLD_V1COMPAT; } while (0)
#define CLR_MLD_LASTREPORT(f)    do { (f) &= ~MLD_LASTREPORT; } while (0)
#define CLR_MLD_SCHEDMSG(f)      do { (f) &= ~MLD_SCHEDMSG; } while (0)
#define CLR_MLD_WAITMSG(f)       do { (f) &= ~MLD_WAITMSG; } while (0)

#define IS_MLD_QUERIER(f)        (((f) & MLD_QUERIER) != 0)
#define IS_MLD_STARTUP(f)        (((f) & MLD_STARTUP) != 0)
#define IS_MLD_V1COMPAT(f)       (((f) & MLD_V1COMPAT) != 0)
#define IS_MLD_LASTREPORT(f)     (((f) & MLD_LASTREPORT) != 0)
#define IS_MLD_SCHEDMSG(f)       (((f) & MLD_SCHEDMSG) != 0)
#define IS_MLD_WAITMSG(f)        (((f) & MLD_WAITMSG) != 0)

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* These are the types of messages that may be sent in response to a device
 * poll.
 */

enum mld_msgtype_e
{
  MLD_SEND_NONE = 0,           /* Nothing to send */
  MLD_SEND_GENQUERY,           /* Send General Query */
  MLD_SEND_REPORT,             /* Send Unsolicited report */
  MLD_SEND_DONE                /* Send Done message */
};

/* This structure represents one group member.  There is a list of groups
 * for each device interface structure.
 *
 * There will be a group for the all systems group address but this
 * will not run the state machine as it is used to kick off reports
 * from all the other groups
 */

typedef FAR struct wdog_s *WDOG_ID;
struct mld_group_s
{
  struct mld_group_s *next;    /* Implements a singly-linked list */
  net_ipv6addr_t      grpaddr; /* Group IPv6 address */
  struct work_s       work;    /* For deferred timeout operations */
  WDOG_ID             wdog;    /* WDOG used to detect timeouts */
  sem_t               sem;     /* Used to wait for message transmission */
  uint8_t             ifindex; /* Interface index */
  uint8_t             flags;   /* See MLD_ flags definitions */
  uint8_t             msgtype; /* Pending message type to send (if non-zero) */
  uint8_t             count;   /* Report repetition count */
};

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

struct ipv6_mreq;                    /* Forward reference */
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
 * Name:  mld_devinit
 *
 * Description:
 *   Called when a new network device is registered to configure that device
 *   for MLD support.
 *
 ****************************************************************************/

void mld_devinit(FAR struct net_driver_s *dev);

/****************************************************************************
 * Name: mld_query
 *
 * Description:
 *  Called from icmpv6_input() when a Multicast Listener Query is received.
 *
 ****************************************************************************/

int mld_query(FAR struct net_driver_s *dev,
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
 *   Called from icmpv6_input() when a Version 2 Multicast Listener Report is
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

/****************************************************************************
 * Name:  mld_grpalloc
 *
 * Description:
 *   Allocate a new group from heap memory.
 *
 ****************************************************************************/

FAR struct mld_group_s *mld_grpalloc(FAR struct net_driver_s *dev,
                                     FAR const net_ipv6addr_t addr);

/****************************************************************************
 * Name:  mld_grpfind
 *
 * Description:
 *   Find an existing group.
 *
 ****************************************************************************/

FAR struct mld_group_s *mld_grpfind(FAR struct net_driver_s *dev,
                                    FAR const net_ipv6addr_t addr);

/****************************************************************************
 * Name:  mld_grpallocfind
 *
 * Description:
 *   Find an existing group.  If not found, create a new group for the
 *   address.
 *
 ****************************************************************************/

FAR struct mld_group_s *mld_grpallocfind(FAR struct net_driver_s *dev,
                                         FAR const net_ipv6addr_t addr);

/****************************************************************************
 * Name:  mld_grpfree
 *
 * Description:
 *   Release a previously allocated group.
 *
 ****************************************************************************/

void mld_grpfree(FAR struct net_driver_s *dev,
                 FAR struct mld_group_s *group);

/****************************************************************************
 * Name: mld_schedmsg
 *
 * Description:
 *   Schedule a message to be send at the next driver polling interval.
 *
 ****************************************************************************/

int mld_schedmsg(FAR struct mld_group_s *group, uint8_t msgtype);

/****************************************************************************
 * Name: mld_waitmsg
 *
 * Description:
 *   Schedule a message to be send at the next driver polling interval and
 *   block, waiting for the message to be sent.
 *
 ****************************************************************************/

int mld_waitmsg(FAR struct mld_group_s *group, uint8_t msgtype);

/****************************************************************************
 * Name:  mld_poll
 *
 * Description:
 *   Poll the groups associated with the device to see if any MLD messages
 *   are pending transfer.
 *
 * Returned Value:
 *   Returns a non-zero value if a IGP message is sent.
 *
 ****************************************************************************/

void mld_poll(FAR struct net_driver_s *dev);

/****************************************************************************
 * Name: mld_send
 *
 * Description:
 *   Sends an MLD IP packet on a network interface. This function constructs
 *   the IP header and calculates the IP header checksum.
 *
 * Input Parameters:
 *   dev        - The device driver structure to use in the send operation.
 *   group      - Describes the multicast group member and identifies the
 *                message to be sent.
 *   destipaddr - The IP address of the recipient of the message
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

void mld_send(FAR struct net_driver_s *dev, FAR struct mld_group_s *group,
              FAR const net_ipv6addr_t dest);

/****************************************************************************
 * Name:  mld_joingroup
 *
 * Description:
 *   Add the specified group address to the group.  This function
 *   implements the logic for the IPV6_JOIN_GROUP socket option.
 *
 *   The IPV6_JOIN_GROUP socket option is used to join a multicast group.
 *   This is accomplished by using the setsockopt() API and specifying the
 *   address of the ipv6_mreq structure containing the IPv6 multicast address
 *   and the local IPv6 multicast interface index.  The stack chooses a
 *   default multicast interface if an interface index of 0 is passed. The
 *   values specified in the IPV6_MREQ structure used by IPV6_JOIN_GROUP
 *   and IPV6_LEAVE_GROUP must be symmetrical. The format of the ipv6_mreq
 *   structure can be found in include/netinet/in.h
 *
 ****************************************************************************/

int mld_joingroup(FAR const struct ipv6_mreq *mrec);

/****************************************************************************
 * Name:  mld_leavegroup
 *
 * Description:
 *   Remove the specified group address to the group.  This function
 *   implements the logic for the IPV6_LEAVE_GROUP socket option.
 *
 *   The IPV6_JOIN_GROUP socket option is used to join a multicast group.
 *   This is accomplished by using the setsockopt() API and specifying the
 *   address of the ipv6_mreq structure containing the IPv6 multicast address
 *   and the local IPv6 multicast interface index.  The stack chooses a
 *   default multicast interface if an interface index of 0 is passed. The
 *   values specified in the IPV6_MREQ structure used by IPV6_JOIN_GROUP
 *   and IPV6_LEAVE_GROUP must be symmetrical. The format of the ipv6_mreq
 *   structure can be found in include/netinet/in.h
 *
 ****************************************************************************/

int mld_leavegroup(FAR const struct ipv6_mreq *mrec);

/****************************************************************************
 * Name:  mld_starttimer
 *
 * Description:
 *   Start the MLD timer with differing time units (ticks or deciseconds).
 *
 ****************************************************************************/

void mld_starttimer(FAR struct mld_group_s *group, clock_t ticks);

/****************************************************************************
 * Name:  mld_cmptimer
 *
 * Description:
 *   Compare the timer remaining on the watching timer to the deci-second
 *   value. If maxticks > ticks-remaining, then (1) cancel the timer (to
 *   avoid race conditions) and return true.
 *
 *   If true is returned then the caller must call mld_starttimer() to
 *    restart the timer
 *
 ****************************************************************************/

bool mld_cmptimer(FAR struct mld_group_s *group, int maxticks);

/****************************************************************************
 * Name:  mld_addmcastmac
 *
 * Description:
 *   Add an MLD MAC address to the device's MAC filter table.
 *
 ****************************************************************************/

void mld_addmcastmac(FAR struct net_driver_s *dev,
                     FAR const net_ipv6addr_t ipaddr);

/****************************************************************************
 * Name:  mld_removemcastmac
 *
 * Description:
 *   Remove an MLD MAC address from the device's MAC filter table.
 *
 ****************************************************************************/

void mld_removemcastmac(FAR struct net_driver_s *dev,
                        FAR const net_ipv6addr_t ipaddr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET_MLD */
#endif /* __NET_NETLINK_MLD_H */
