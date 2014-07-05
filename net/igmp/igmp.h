/****************************************************************************
 * net/igmp/igmp.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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
/*
 *                              ________________
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

#ifndef _NET_IGMP_IGMP_H
#define _NET_IGMP_IGMP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <nuttx/net/ip.h>

#ifdef CONFIG_NET_IGMP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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

typedef FAR struct wdog_s *WDOG_ID;
struct igmp_group_s
{
  struct igmp_group_s *next;    /* Implements a singly-linked list */
  net_ipaddr_t         grpaddr; /* Group IP address */
  WDOG_ID              wdog;    /* WDOG used to detect timeouts */
  sem_t                sem;     /* Used to wait for message transmission */
  volatile uint8_t     flags;   /* See IGMP_ flags definitions */
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

EXTERN net_ipaddr_t g_allsystems;
EXTERN net_ipaddr_t g_allrouters;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Defined in igmp_init.c ***************************************************/
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

/* Defined in igmp_input.c **************************************************/
/****************************************************************************
 * Name:  igmp_input
 *
 * Description:
 *   An IGMP packet has been received.
 *
 * NOTE: This is most likely executing from an interrupt handler.
 *
 ****************************************************************************/

void igmp_input(struct net_driver_s *dev);

/* Defined in igmp_group.c **************************************************/
/****************************************************************************
 * Name:  igmp_grpinit
 *
 * Description:
 *   One-time initialization of group data.
 *
 * Assumptions:
 *   Called only during early boot phases (pre-multitasking).
 *
 ****************************************************************************/

void igmp_grpinit(void);

/****************************************************************************
 * Name:  igmp_grpalloc
 *
 * Description:
 *   Allocate a new group from heap memory.
 *
 * Assumptions:
 *   May be called from either user or interrupt level processing.
 *
 ****************************************************************************/

FAR struct igmp_group_s *igmp_grpalloc(FAR struct net_driver_s *dev,
                                       FAR const net_ipaddr_t *addr);

/****************************************************************************
 * Name:  igmp_grpfind
 *
 * Description:
 *   Find an existing group.
 *
 * Assumptions:
 *   May be called from either user or interrupt level processing.
 *
 ****************************************************************************/

FAR struct igmp_group_s *igmp_grpfind(FAR struct net_driver_s *dev,
                                      FAR const net_ipaddr_t *addr);

/****************************************************************************
 * Name:  igmp_grpallocfind
 *
 * Description:
 *   Find an existing group.  If not found, create a new group for the
 *   address.
 *
 * Assumptions:
 *   May be called from either user or interrupt level processing.
 *
 ****************************************************************************/

FAR struct igmp_group_s *igmp_grpallocfind(FAR struct net_driver_s *dev,
                                           FAR const net_ipaddr_t *addr);

/****************************************************************************
 * Name:  igmp_grpfree
 *
 * Description:
 *   Release a previously allocated group.
 *
 * Assumptions:
 *   May be called from either user or interrupt level processing.
 *
 ****************************************************************************/

void igmp_grpfree(FAR struct net_driver_s *dev,
                  FAR struct igmp_group_s *group);

/* Defined in igmp_msg.c ****************************************************/
/****************************************************************************
 * Name: igmp_schedmsg
 *
 * Description:
 *   Schedule a message to be send at the next driver polling interval.
 *
 * Assumptions:
 *   This function may be called in most any context.
 *
 ****************************************************************************/

void igmp_schedmsg(FAR struct igmp_group_s *group, uint8_t msgid);

/****************************************************************************
 * Name: igmp_waitmsg
 *
 * Description:
 *   Schedule a message to be send at the next driver polling interval and
 *   block, waiting for the message to be sent.
 *
 * Assumptions:
 *   This function cannot be called from an interrupt handler (if you try it,
 *   net_lockedwait will assert).
 *
 ****************************************************************************/

void igmp_waitmsg(FAR struct igmp_group_s *group, uint8_t msgid);

/* Defined in igmp_poll.c ***************************************************/
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
 * Assumptions:
 *   This function is called from the driver polling logic... probably within
 *   an interrupt handler.
 *
 ****************************************************************************/

void igmp_poll(FAR struct net_driver_s *dev);

/* Defined in igmp_send.c ***************************************************/
/****************************************************************************
 * Name: igmp_send
 *
 * Description:
 *   Sends an IGMP IP packet on a network interface. This function constructs
 *   the IP header and calculates the IP header checksum.
 *
 * Parameters:
 *   dev        - The device driver structure to use in the send operation.
 *   group      - Describes the multicast group member and identifies the
 *                message to be sent.
 *   destipaddr - The IP address of the recipient of the message
 *
 * Return:
 *   None
 *
 * Assumptions:
 *   Called from the interrupt level or with interrupts disabled.
 *
 ****************************************************************************/

void igmp_send(FAR struct net_driver_s *dev, FAR struct igmp_group_s *group,
                  FAR net_ipaddr_t *dest);

/* Defined in igmp_join.c ***************************************************/
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
 * Assumptions:
 *   This function cannot be called from interrupt handling logic!
 *
 ****************************************************************************/

int igmp_joingroup(FAR struct net_driver_s *dev,
                   FAR const struct in_addr *grpaddr);

/* Defined in igmp_leave.c **************************************************/
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
 * Assumptions:
 *   This function cannot be called from interrupt handling logic!
 *
 ****************************************************************************/

int igmp_leavegroup(FAR struct net_driver_s *dev,
                    FAR const struct in_addr *grpaddr);

/* Defined in igmp_timer.c **************************************************/
/****************************************************************************
 * Name:  igmp_startticks and igmp_starttimer
 *
 * Description:
 *   Start the IGMP timer with differing time units (ticks or deciseconds).
 *
 * Assumptions:
 *   This function may be called from most any context.
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
 * Assumptions:
 *   This function may be called from most any context.  If true is returned
 *   then the caller must call igmp_startticks() to restart the timer
 *
 ****************************************************************************/

bool igmp_cmptimer(FAR struct igmp_group_s *group, int maxticks);

/* Defined in igmp_mcastmac *************************************************/
/****************************************************************************
 * Name:  igmp_addmcastmac
 *
 * Description:
 *   Add an IGMP MAC address to the device's MAC filter table.
 *
 ****************************************************************************/

void igmp_addmcastmac(FAR struct net_driver_s *dev, FAR net_ipaddr_t *ip);

/****************************************************************************
 * Name:  igmp_removemcastmac
 *
 * Description:
 *   Remove an IGMP MAC address from the device's MAC filter table.
 *
 ****************************************************************************/

void igmp_removemcastmac(FAR struct net_driver_s *dev, FAR net_ipaddr_t *ip);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET_IGMP */
#endif /* _NET_IGMP_IGMP_H */
