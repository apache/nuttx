/****************************************************************************
 * net/igmp/igmp.h
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

/*                              ________________
 *                             |                |
 *                             |                |
 *                             |                |
 *                             |                |
 *                  +--------->|   Non-Member   |<---------+
 *                  |          |                |          |
 *                  |          |                |          |
 *                  |          |                |          |
 *                  |          |________________|          |
 *                  |                   |                  |
 *                  | leave group       | join group       | leave group
 *                  | (stop timer,      |(send report,     | (send leave
 *                  |  send leave if    | set flag,        |  if flag set)
 *                  |  flag set)        | start timer)     |
 *          ________|________           |          ________|________
 *         |                 |<---------+         |                 |
 *         |                 |                    |                 |
 *         |                 |<-------------------|                 |
 *         |                 |   query received   |                 |
 *         | Delaying Member |    (start timer)   |   Idle Member   |
 *   +---->|                 |------------------->|                 |
 *   |     |                 |   report received  |                 |
 *   |     |                 |    (stop timer,    |                 |
 *   |     |                 |     clear flag)    |                 |
 *   |     |_________________|------------------->|_________________|
 *   | query received    |        timer expired
 *   | (reset timer if   |        (send report,
 *   |  Max Resp Time    |         set flag)
 *   |  < current timer) |
 *   +-------------------+
 */

#ifndef __NET_IGMP_IGMP_H
#define __NET_IGMP_IGMP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <nuttx/semaphore.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/ip.h>
#include <nuttx/wdog.h>

#ifdef CONFIG_NET_IGMP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Group flags */

#define IGMP_IDLEMEMBER          (1 << 0)
#define IGMP_LASTREPORT          (1 << 1)
#define IGMP_SCHEDMSG            (1 << 2)
#define IGMP_WAITMSG             (1 << 3)

#define SET_IDLEMEMBER(f)        do { (f) |= IGMP_IDLEMEMBER; } while (0)
#define SET_LASTREPORT(f)        do { (f) |= IGMP_LASTREPORT; } while (0)
#define SET_SCHEDMSG(f)          do { (f) |= IGMP_SCHEDMSG; } while (0)
#define SET_WAITMSG(f)           do { (f) |= IGMP_WAITMSG; } while (0)

#define CLR_IDLEMEMBER(f)        do { (f) &= ~IGMP_IDLEMEMBER; } while (0)
#define CLR_LASTREPORT(f)        do { (f) &= ~IGMP_LASTREPORT; } while (0)
#define CLR_SCHEDMSG(f)          do { (f) &= ~IGMP_SCHEDMSG; } while (0)
#define CLR_WAITMSG(f)           do { (f) &= ~IGMP_WAITMSG; } while (0)

#define IS_IDLEMEMBER(f)         (((f) & IGMP_IDLEMEMBER) != 0)
#define IS_LASTREPORT(f)         (((f) & IGMP_LASTREPORT) != 0)
#define IS_SCHEDMSG(f)           (((f) & IGMP_SCHEDMSG) != 0)
#define IS_WAITMSG(f)            (((f) & IGMP_WAITMSG) != 0)

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* This structure represents one group member.  There is a list of groups
 * for each device interface structure.
 *
 * There will be a group for the all systems group address but this
 * will not run the state machine as it is used to kick off reports
 * from all the other groups
 */

struct igmp_group_s
{
  struct igmp_group_s *next;    /* Implements a singly-linked list */
  struct work_s        work;    /* For deferred timeout operations */
  in_addr_t            grpaddr; /* Group IPv4 address */
  struct wdog_s        wdog;    /* WDOG used to detect timeouts */
  sem_t                sem;     /* Used to wait for message transmission */
  uint8_t              ifindex; /* Interface index */
  uint8_t              flags;   /* See IGMP_ flags definitions */
  uint8_t              msgid;   /* Pending message ID (if non-zero) */
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

EXTERN const in_addr_t g_ipv4_allsystems;
EXTERN const in_addr_t g_ipv4_allrouters;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name:  igmp_initialize
 *
 * Description:
 *   Perform one-time IGMP initialization.
 *
 ****************************************************************************/

void igmp_initialize(void);

/****************************************************************************
 * Name:  igmp_devinit
 *
 * Description:
 *   Called when a new network device is registered to configure that device
 *   for IGMP support.
 *
 ****************************************************************************/

void igmp_devinit(FAR struct net_driver_s *dev);

/****************************************************************************
 * Name:  igmp_input
 *
 * Description:
 *   An IGMP packet has been received.
 *
 ****************************************************************************/

void igmp_input(struct net_driver_s *dev);

/****************************************************************************
 * Name:  igmp_grpalloc
 *
 * Description:
 *   Allocate a new group from heap memory.
 *
 ****************************************************************************/

FAR struct igmp_group_s *igmp_grpalloc(FAR struct net_driver_s *dev,
                                       FAR const in_addr_t *addr);

/****************************************************************************
 * Name:  igmp_grpfind
 *
 * Description:
 *   Find an existing group.
 *
 ****************************************************************************/

FAR struct igmp_group_s *igmp_grpfind(FAR struct net_driver_s *dev,
                                      FAR const in_addr_t *addr);

/****************************************************************************
 * Name:  igmp_grpallocfind
 *
 * Description:
 *   Find an existing group.  If not found, create a new group for the
 *   address.
 *
 ****************************************************************************/

FAR struct igmp_group_s *igmp_grpallocfind(FAR struct net_driver_s *dev,
                                           FAR const in_addr_t *addr);

/****************************************************************************
 * Name:  igmp_grpfree
 *
 * Description:
 *   Release a previously allocated group.
 *
 ****************************************************************************/

void igmp_grpfree(FAR struct net_driver_s *dev,
                  FAR struct igmp_group_s *group);

/****************************************************************************
 * Name: igmp_schedmsg
 *
 * Description:
 *   Schedule a message to be send at the next driver polling interval.
 *
 ****************************************************************************/

int igmp_schedmsg(FAR struct igmp_group_s *group, uint8_t msgid);

/****************************************************************************
 * Name: igmp_waitmsg
 *
 * Description:
 *   Schedule a message to be send at the next driver polling interval and
 *   block, waiting for the message to be sent.
 *
 ****************************************************************************/

int igmp_waitmsg(FAR struct igmp_group_s *group, uint8_t msgid);

/****************************************************************************
 * Name:  igmp_poll
 *
 * Description:
 *   Poll the groups associated with the device to see if any IGMP messages
 *   are pending transfer.
 *
 * Returned Value:
 *   Returns a non-zero value if a IGP message is sent.
 *
 ****************************************************************************/

void igmp_poll(FAR struct net_driver_s *dev);

/****************************************************************************
 * Name: igmp_send
 *
 * Description:
 *   Sends an IGMP IP packet on a network interface. This function constructs
 *   the IP header and calculates the IP header checksum.
 *
 * Input Parameters:
 *   dev        - The device driver structure to use in the send operation.
 *   group      - Describes the multicast group member and identifies the
 *                message to be sent.
 *   destipaddr - The IP address of the recipient of the message
 *   msgid      - ID of message to send
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

void igmp_send(FAR struct net_driver_s *dev, FAR struct igmp_group_s *group,
               FAR const in_addr_t *destipaddr, uint8_t msgid);

/****************************************************************************
 * Name:  igmp_joingroup
 *
 * Description:
 *   Add the specified group address to the group.
 *
 * RFC 2236, 3.  Protocol Description:
 *
 *  "When a host joins a multicast group, it should immediately transmit
 *   an unsolicited Version 2 Membership Report for that group, in case it
 *   is the first member of that group on the network.  To cover the
 *   possibility of the initial Membership Report being lost or damaged,
 *   it is recommended that it be repeated once or twice after short
 *   delays [Unsolicited Report Interval].  (A simple way to accomplish
 *   this is to send the initial Version 2 Membership Report and then act
 *   as if a Group-Specific Query was received for that group, and set a
 *   timer appropriately)."
 *
 ****************************************************************************/

int igmp_joingroup(FAR struct net_driver_s *dev,
                   FAR const struct in_addr *grpaddr);

/****************************************************************************
 * Name:  igmp_leavegroup
 *
 * Description:
 *   Remove the specified group address to the group.
 *
 * RFC 2236, 3.  Protocol Description:
 *
 *  "When a host leaves a multicast group, if it was the last host to
 *   reply to a Query with a Membership Report for that group, it SHOULD
 *   send a Leave Group message to the all-routers multicast group
 *   (224.0.0.2). If it was not the last host to reply to a Query, it MAY
 *   send nothing as there must be another member on the subnet.  This is
 *   an optimization to reduce traffic; a host without sufficient storage
 *   to remember whether or not it was the last host to reply MAY always
 *   send a Leave Group message when it leaves a group.  Routers SHOULD
 *   accept a Leave Group message addressed to the group being left, in
 *   order to accommodate implementations of an earlier version of this
 *   standard.  Leave Group messages are addressed to the all-routers
 *   group because other group members have no need to know that a host
 *   has left the group, but it does no harm to address the message to the
 *   group."
 *
 ****************************************************************************/

int igmp_leavegroup(FAR struct net_driver_s *dev,
                    FAR const struct in_addr *grpaddr);

/****************************************************************************
 * Name:  igmp_startticks and igmp_starttimer
 *
 * Description:
 *   Start the IGMP timer with differing time units (ticks or deciseconds).
 *
 ****************************************************************************/

void igmp_startticks(FAR struct igmp_group_s *group, unsigned int ticks);
void igmp_starttimer(FAR struct igmp_group_s *group, uint8_t decisecs);

/****************************************************************************
 * Name:  igmp_cmptimer
 *
 * Description:
 *   Compare the timer remaining on the watching timer to the deci-second
 *   value. If maxticks > ticks-remaining, then (1) cancel the timer (to
 *   avoid race conditions) and return true.
 *
 *   If true is returned then the caller must call igmp_startticks() to
 *    restart the timer
 *
 ****************************************************************************/

bool igmp_cmptimer(FAR struct igmp_group_s *group, int maxticks);

/****************************************************************************
 * Name:  igmp_addmcastmac
 *
 * Description:
 *   Add an IGMP MAC address to the device's MAC filter table.
 *
 ****************************************************************************/

void igmp_addmcastmac(FAR struct net_driver_s *dev, FAR const in_addr_t *ip);

/****************************************************************************
 * Name:  igmp_removemcastmac
 *
 * Description:
 *   Remove an IGMP MAC address from the device's MAC filter table.
 *
 ****************************************************************************/

void igmp_removemcastmac(FAR struct net_driver_s *dev,
                         FAR const in_addr_t *ip);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET_IGMP */
#endif /* __NET_IGMP_IGMP_H */
