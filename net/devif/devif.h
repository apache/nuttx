/****************************************************************************
 * net/devif/devif.h
 *
 *   Copyright (C) 2007-2009, 2013-2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This logic was leveraged from uIP which also has a BSD-style license:
 *
 *   Author Adam Dunkels <adam@dunkels.com>
 *   Copyright (c) 2001-2003, Adam Dunkels.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
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

#ifndef _NET_DEVIF_DEVIF_H
#define _NET_DEVIF_DEVIF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifdef CONFIG_NET

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <arch/irq.h>

#include <nuttx/clock.h>
#include <nuttx/net/ip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* The following flags may be set in the set of flags by the lower, device-
 * interfacing layer before calling through the socket layer callback. The
 * TCP_ACKDATA, XYZ_NEWDATA, and TCP_CLOSE flags may be set at the same time,
 * whereas the others are mutually exclusive.
 *
 * Connection Specific Events:  These are events that may be notified
 * through callback lists residing in TCP, UDP, or PKT port connection
 * structures:
 *
 *   TCP_ACKDATA      IN: Signifies that the outstanding data was ACKed and
 *                        the socket layer should send out new data instead
 *                        of retransmitting the last data (TCP only)
 *                   OUT: Input state must be preserved on output.
 *
 *   TCP_NEWDATA      IN: Set to indicate that the peer has sent us new data.
 *   UDP_NEWDATA     OUT: Cleared (only) by the socket layer logic to indicate
 *   PKT_NEWDATA          that the new data was consumed, suppressing further
 *   BLUETOOTH_NEWDATA    attempts to process the new data.
 *   IEEE802154_NEWDATA
 *
 *   TCP_SNDACK       IN: Not used; always zero
 *                   OUT: Set by the socket layer if the new data was consumed
 *                        and an ACK should be sent in the response. (TCP only)
 *
 *   TCP_REXMIT       IN: Tells the socket layer to retransmit the data that
 *                        was last sent. (TCP only)
 *                   OUT: Not used
 *
 *   TCP_POLL        IN:  Used for polling the socket layer.  This is provided
 *   UDP_POLL             periodically from the drivers to support (1) timed
 *   PKT_POLL             operations, and (2) to check if the socket layer has
 *   BLUETOOTH_POLL       data that it wants to send.  These are socket oriented
 *   IEEE802154_POLL      callbacks where the context depends on the specific
 *                        set
 *                   OUT: Not used
 *
 *   TCP_BACKLOG      IN: There is a new connection in the backlog list set
 *                        up by the listen() command. (TCP only)
 *                   OUT: Not used
 *
 *   TCP_CLOSE        IN: The remote host has closed the connection, thus the
 *                        connection has gone away. (TCP only)
 *                   OUT: The socket layer signals that it wants to close the
 *                        connection. (TCP only)
 *
 *   TCP_ABORT        IN: The remote host has aborted the connection, thus the
 *                        connection has gone away. (TCP only)
 *                   OUT: The socket layer signals that it wants to abort the
 *                        connection. (TCP only)
 *
 *   TCP_CONNECTED    IN: We have got a connection from a remote host and have
 *                        set up a new connection for it, or an active connection
 *                        has been successfully established. (TCP only)
 *                   OUT: Not used
 *
 *   TCP_TIMEDOUT     IN: The connection has been aborted due to too many
 *                        retransmissions. (TCP only)
 *                   OUT: Not used
 *
 * Device Specific Events:  These are events that may be notified through
 * callback lists residing in the network device structure.
 *
 *   ICMP_NEWDATA     IN: Set to indicate that the peer has sent us new data.
 *   ICMPv6_NEWDATA OUT: Cleared (only) by the socket layer logic to indicate
 *                       that the new data was consumed, suppressing further
 *                       attempts to process the new data.
 *
 *   ARP_POLL         IN: Used for polling the socket layer.  This is provided
 *                        periodically from the drivers to support (1) timed
 *                        operations, and (2) to check if the ARP layer needs
 *                        to send an ARP request.  This is a device oriented
 *                        event, not associated with a socket.
 *                   OUT: Not used
 *
 *   ICMP_POLL        IN: Used for polling the socket layer.  This is provided
 *                        periodically from the drivers to support (1) timed
 *                        operations, and (2) to check if the ICMP layer needs
 *                        to send an ARP request.  This is a device oriented
 *                        event, not associated with a socket.  This differs
 *                        from ICMPv6_POLL only in that the appdata pointer
 *                        is set differently
 *                   OUT: Not used
 *
 *   ICMPv6_POLL      IN: Used for polling the socket layer.  This is provided
 *                        periodically from the drivers to support (1) timed
 *                        operations, and (2) to check if the ICMP layer needs
 *                        to send an ARP request.  This is a device oriented
 *                        event, not associated with a socket.  This differs
 *                        from ICMP_POLL only in that the appdata pointer
 *                        is set differently
 *                   OUT: Not used
 *
 *   IPFWD_POLL       IN: Used for polling for forwarded packets layer.  This
 *                        is provided periodically from the drivers to support
 *                        to check if there is a packet waiting to be forward
 *                        on the device.  This is a device oriented event,
 *                        not associated with a socket.  The appdata pointer
 *                        The appdata pointer is not used in this case.
 *                   OUT: Not used
 *
 *   ICMP_ECHOREPLY   IN: An ICMP Echo Reply has been received.  Used to support
 *                        ICMP ping from the socket layer. (ICMPv4 only)
 *                   OUT: Cleared (only) by the socket layer logic to indicate
 *                        that the reply was processed, suppressing further
 *                        attempts to process the reply.
 *
 *   ICMPv6_ECHOREPLY IN: An ICMP Echo Reply has been received.  Used to support
 *                        ICMP ping from the socket layer. (ICMPv6 only)
 *                   OUT: Cleared (only) by the socket layer logic to indicate
 *                        that the reply was processed, suppressing further
 *                        attempts to process the reply.
 *
 *   NETDEV_DOWN:     IN: The network device has been taken down.
 *                   OUT: Not used
 */

/* Bits 0-9: Connection specific event bits */

#define TCP_ACKDATA        (1 << 0)
#define TCP_NEWDATA        (1 << 1)
#define UDP_NEWDATA        TCP_NEWDATA
#define BLUETOOTH_NEWDATA  TCP_NEWDATA
#define IEEE802154_NEWDATA TCP_NEWDATA
#define PKT_NEWDATA        TCP_NEWDATA
#define WPAN_NEWDATA       TCP_NEWDATA
#define IPFWD_NEWDATA      TCP_NEWDATA
#define TCP_SNDACK         (1 << 2)
#define TCP_REXMIT         (1 << 3)
#define TCP_POLL           (1 << 4)
#define UDP_POLL           TCP_POLL
#define PKT_POLL           TCP_POLL
#define BLUETOOTH_POLL     TCP_POLL
#define IEEE802154_POLL    TCP_POLL
#define WPAN_POLL          TCP_POLL
#define TCP_BACKLOG        (1 << 5)
#define TCP_CLOSE          (1 << 6)
#define TCP_ABORT          (1 << 7)
#define TCP_CONNECTED      (1 << 8)
#define TCP_TIMEDOUT       (1 << 9)

/* Bits 10-12: Device specific event bits */

#define ICMP_NEWDATA       TCP_NEWDATA
#define ICMPv6_NEWDATA     TCP_NEWDATA
#define ICMP_ECHOREPLY     (1 << 10)
#define ICMPv6_ECHOREPLY   (1 << 11)
#define NETDEV_DOWN        (1 << 12)

/* Bits 13-15: Encoded device specific poll events.  Unlike connection
 * oriented poll events, device related poll events must distinguish
 * between what is being polled for since the callbacks all reside in
 * the same list in the network device structure.
 */

#define DEVPOLL_SHIFT      (13)
#define DEVPOLL_MASK       (7 << DEVPOLL_SHIFT)
#  define DEVPOLL_NONE     (0 << DEVPOLL_SHIFT)
#  define ARP_POLL         (1 << DEVPOLL_SHIFT)
#  define ICMP_POLL        (2 << DEVPOLL_SHIFT)
#  define ICMPv6_POLL      (3 << DEVPOLL_SHIFT)
#  define IPFWD_POLL       (4 << DEVPOLL_SHIFT)

/* The set of events that and implications to the TCP connection state */

#define TCP_CONN_EVENTS \
  (TCP_CLOSE | TCP_ABORT | TCP_CONNECTED | TCP_TIMEDOUT | NETDEV_DOWN)

#define TCP_DISCONN_EVENTS \
  (TCP_CLOSE | TCP_ABORT | TCP_TIMEDOUT | NETDEV_DOWN)

/* IPv4/IPv6 Helpers */

#ifdef CONFIG_NET_IPv4
#  define DEVIF_IS_IPv4(dev) IFF_IS_IPv4(dev->d_flags)
#else
#  define DEVIF_IS_IPv4(dev) (0)
#endif

#ifdef CONFIG_NET_IPv6
#  define DEVIF_IS_IPv6(dev) IFF_IS_IPv6(dev->d_flags)
#else
#  define DEVIF_IS_IPv6(dev) (0)
#endif

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Describes a connection/device event callback interface
 *
 *   nxtconn - Supports a singly linked list that supports connection
 *             specific event handlers.
 *   nxtdev  - Supports a singly linked list that supports device specific
 *             event handlers
 *   event   - Provides the address of the callback function entry point.
 *             pvconn is a pointer to a connection-specific datat structure
 *             such as struct tcp_conn_s or struct udp_conn_s.
 *   priv    - Holds a reference to socket layer specific data that will
 *             provided
 *   flags   - Set by the socket layer to inform the lower layer which flags
 *             are and are not handled by the callback.
 */

struct net_driver_s;       /* Forward reference */

typedef CODE uint16_t (*devif_callback_event_t)(FAR struct net_driver_s *dev,
                                                FAR void *pvconn,
                                                FAR void *pvpriv,
                                                uint16_t flags);

struct devif_callback_s
{
  FAR struct devif_callback_s *nxtconn;
  FAR struct devif_callback_s *nxtdev;
  FAR devif_callback_event_t event;
  FAR void *priv;
  uint16_t flags;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#ifdef CONFIG_NET_IPv4_REASSEMBLY
/* Reassembly timer (units: deci-seconds) */

EXTERN uint8_t g_reassembly_timer;
#endif

/* Time of last poll */

EXTERN clock_t g_polltime;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: devif_initialize
 *
 * Description:
 *   Perform initialization of the network device interface layer
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void devif_initialize(void);

/****************************************************************************
 * Name: devif_callback_init
 *
 * Description:
 *   Configure the pre-allocated callback structures into a free list.
 *
 * Assumptions:
 *   This function must be called with the network locked.
 *
 ****************************************************************************/

void devif_callback_init(void);

/****************************************************************************
 * Name: devif_callback_alloc
 *
 * Description:
 *   Allocate a callback container from the free list.
 *
 *   If dev is non-NULL, then this function verifies that the device
 *   reference is still  valid and that the device is still UP status.  If
 *   those conditions are not true, this function will fail to allocate the
 *   callback.
 *
 * Assumptions:
 *   This function must be called with the network locked.
 *
 ****************************************************************************/

FAR struct devif_callback_s *
  devif_callback_alloc(FAR struct net_driver_s *dev,
                       FAR struct devif_callback_s **list);

/****************************************************************************
 * Name: devif_conn_callback_free
 *
 * Description:
 *   Return a connection/port callback container to the free list.
 *
 *   This function is just a front-end for devif_callback_free().  If the
 *   dev argument is non-NULL, it will verify that the device reference is
 *   still valid before attempting to free the callback structure.  A
 *   non-NULL list pointer is assumed to be valid in any case.
 *
 *   The callback structure will be freed in any event.
 *
 * Assumptions:
 *   This function must be called with the network locked.
 *
 ****************************************************************************/

void devif_conn_callback_free(FAR struct net_driver_s *dev,
                              FAR struct devif_callback_s *cb,
                              FAR struct devif_callback_s **list);

/****************************************************************************
 * Name: devif_dev_callback_free
 *
 * Description:
 *   Return a device callback container to the free list.
 *
 *   This function is just a front-end for devif_callback_free().  If the
 *   de argument is non-NULL, it will verify that the device reference is
 *   still valid before attempting to free the callback structure.  It
 *   differs from devif_conn_callback_free in that connection/port-related
 *   connections are also associated with the device and, hence, also will
 *   not be valid if the device pointer is not valid.
 *
 *   The callback structure will be freed in any event.
 *
 * Assumptions:
 *   This function must be called with the network locked.
 *
 ****************************************************************************/

void devif_dev_callback_free(FAR struct net_driver_s *dev,
                             FAR struct devif_callback_s *cb);

/****************************************************************************
 * Name: devif_conn_event
 *
 * Description:
 *   Execute a list of callbacks.
 *
 * Input Parameters:
 *   dev - The network device state structure associated with the network
 *     device that initiated the callback event.
 *   pvconn - Holds a reference to the TCP connection structure or the UDP
 *     port structure.  May be NULL if the even is not related to a TCP
 *     connection or UDP port.
 *   flags - The bit set of events to be notified.
 *   list - The list to traverse in performing the notifications
 *
 * Returned Value:
 *   The updated flags as modified by the callback functions.
 *
 * Assumptions:
 *   This function must be called with the network locked.
 *
 ****************************************************************************/

uint16_t devif_conn_event(FAR struct net_driver_s *dev, FAR void *pvconn,
                          uint16_t flags, FAR struct devif_callback_s *list);

/****************************************************************************
 * Name: devif_dev_event
 *
 * Description:
 *   Execute a list of callbacks using the device event chain.
 *
 * Input Parameters:
 *   dev - The network device state structure associated with the network
 *     device that initiated the callback event.
 *   pvconn - Holds a reference to the TCP connection structure or the UDP
 *     port structure.  May be NULL if the even is not related to a TCP
 *     connection or UDP port.
 *   flags - The bit set of events to be notified.
 *
 * Returned Value:
 *   The updated flags as modified by the callback functions.
 *
 * Assumptions:
 *   This function must be called with the network locked.
 *
 ****************************************************************************/

uint16_t devif_dev_event(FAR struct net_driver_s *dev, void *pvconn,
                         uint16_t flags);

/****************************************************************************
 * Send data on the current connection.
 *
 * This function is used to send out a single segment of TCP data.  Only
 * socket logic that have been invoked by the lower level for event
 * processing can send data.
 *
 * The amount of data that actually is sent out after a call to this
 * function is determined by the maximum amount of data TCP allows. The
 * network will automatically crop the data so that only the appropriate
 * amount of data is sent. The mss field of the TCP connection structure
 * can be used to determine the amount of data that actually will be sent.
 *
 * Note:  This function does not guarantee that the sent data will
 * arrive at the destination.  If the data is lost in the network, the
 * TCP socket layer will be invoked with the TCP_REXMIT flag set.  The
 * socket layer will then have to resend the data using this function.
 *
 * data A pointer to the data which is to be sent.
 *
 * len The maximum amount of data bytes to be sent.
 *
 ****************************************************************************/

void devif_send(FAR struct net_driver_s *dev, FAR const void *buf, int len);

/****************************************************************************
 * Name: devif_iob_send
 *
 * Description:
 *   Called from socket logic in response to a xmit or poll request from the
 *   the network interface driver.
 *
 *   This is identical to calling devif_send() except that the data is
 *   in an I/O buffer chain, rather than a flat buffer.
 *
 * Assumptions:
 *   This function must be called with the network locked.
 *
 ****************************************************************************/

#ifdef CONFIG_MM_IOB
struct iob_s;
void devif_iob_send(FAR struct net_driver_s *dev, FAR struct iob_s *buf,
                    unsigned int len, unsigned int offset);
#endif

/****************************************************************************
 * Name: devif_pkt_send
 *
 * Description:
 *   Called from socket logic in order to send a raw packet in response to
 *   an xmit or poll request from the network interface driver.
 *
 *   This is almost identical to calling devif_send() except that the data to
 *   be sent is copied into dev->d_buf (vs. dev->d_appdata), since there is
 *   no header on the data.
 *
 * Assumptions:
 *   This function must be called with the network locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_PKT
void devif_pkt_send(FAR struct net_driver_s *dev, FAR const void *buf,
                    unsigned int len);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET */
#endif /* _NET_DEVIF_DEVIF_H */
