/****************************************************************************
 * net/tcp/tcp.h
 *
 *   Copyright (C) 2014-2015 Gregory Nutt. All rights reserved.
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

#ifndef _NET_TCP_TCP_H
#define _NET_TCP_TCP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <queue.h>

#include <nuttx/net/iob.h>
#include <nuttx/net/ip.h>

#ifdef CONFIG_NET_TCP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Conditions for support TCP poll/select operations */

#if !defined(CONFIG_DISABLE_POLL) && CONFIG_NSOCKET_DESCRIPTORS > 0 && \
    defined(CONFIG_NET_TCP_READAHEAD)
#  define HAVE_TCP_POLL
#endif

/* Allocate a new TCP data callback */

#ifdef CONFIG_NETDEV_MULTINIC
/* These macros allocate and free callback structures used for receiving
 * notifications of TCP data-related events.
 */

#  define tcp_callback_alloc(conn) \
    devif_callback_alloc(conn->dev, &conn->list)
#  define tcp_callback_free(conn,cb) \
    devif_conn_callback_free(conn->dev, cb, &conn->list)

/* These macros allocate and free callback structures used for receiving
 * notifications of device-related events.
 */

#  define tcp_monitor_callback_alloc(conn) \
    devif_callback_alloc(conn->dev, NULL)
#  define tcp_monitor_callback_free(conn,cb) \
    devif_conn_callback_free(conn->dev, cb, NULL)

#else
/* These macros allocate and free callback structures used for receiving
 * notifications of TCP data-related events.
 */

#  define tcp_callback_alloc(conn) \
    devif_callback_alloc(g_netdevices, &conn->list)
#  define tcp_callback_free(conn,cb) \
    devif_conn_callback_free(g_netdevices, cb, &conn->list)

/* These macros allocate and free callback structures used for receiving
 * notifications of device-related events.
 */

#  define tcp_monitor_callback_alloc(conn) \
    devif_callback_alloc(g_netdevices, NULL)
#  define tcp_monitor_callback_free(conn,cb) \
    devif_conn_callback_free(g_netdevices, cb, NULL)
#endif

/* Get the current maximum segment size that can be sent on the current
 * TCP connection.
 */

#define tcp_mss(conn)              ((conn)->mss)

#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
/* TCP write buffer access macros */

#  define WRB_SEQNO(wrb)          ((wrb)->wb_seqno)
#  define WRB_PKTLEN(wrb)         ((wrb)->wb_iob->io_pktlen)
#  define WRB_SENT(wrb)           ((wrb)->wb_sent)
#  define WRB_NRTX(wrb)           ((wrb)->wb_nrtx)
#  define WRB_IOB(wrb)            ((wrb)->wb_iob)
#  define WRB_COPYOUT(wrb,dest,n) (iob_copyout(dest,(wrb)->wb_iob,(n),0))
#  define WRB_COPYIN(wrb,src,n)   (iob_copyin((wrb)->wb_iob,src,(n),0,false))

#  define WRB_TRIM(wrb,n) \
  do { (wrb)->wb_iob = iob_trimhead((wrb)->wb_iob,(n)); } while (0)

#ifdef CONFIG_DEBUG
#  define WRB_DUMP(msg,wrb,len,offset) \
     tcp_wrbuffer_dump(msg,wrb,len,offset)
#else
#  define WRB_DUMP(msg,wrb,len,offset)
#endif
#endif

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/
/* Representation of a TCP connection.
 *
 * The tcp_conn_s structure is used for identifying a connection. All
 * but one field in the structure are to be considered read-only by an
 * application. The only exception is the 'private' fields whose purpose
 * is to let the application store application-specific state (e.g.,
 * file pointers) for the connection.
 */

struct net_driver_s;      /* Forward reference */
struct devif_callback_s;  /* Forward reference */
struct tcp_backlog_s;     /* Forward reference */
struct tcp_hdr_s;         /* Forward reference */

struct tcp_conn_s
{
  dq_entry_t node;        /* Implements a doubly linked list */
  union ip_binding_u u;   /* IP address binding */
  uint8_t  rcvseq[4];     /* The sequence number that we expect to
                           * receive next */
  uint8_t  sndseq[4];     /* The sequence number that was last sent by us */
  uint8_t  crefs;         /* Reference counts on this instance */
#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NET_IPv6)
  uint8_t  domain;        /* IP domain: PF_INET or PF_INET6 */
#endif
  uint8_t  sa;            /* Retransmission time-out calculation state
                           * variable */
  uint8_t  sv;            /* Retransmission time-out calculation state
                           * variable */
  uint8_t  rto;           /* Retransmission time-out */
  uint8_t  tcpstateflags; /* TCP state and flags */
  uint8_t  timer;         /* The retransmission timer (units: half-seconds) */
  uint8_t  nrtx;          /* The number of retransmissions for the last
                           * segment sent */
  uint16_t lport;         /* The local TCP port, in network byte order */
  uint16_t rport;         /* The remoteTCP port, in network byte order */
  uint16_t mss;           /* Current maximum segment size for the
                           * connection */
  uint16_t winsize;       /* Current window size of the connection */
#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
  uint32_t unacked;       /* Number bytes sent but not yet ACKed */
#else
  uint16_t unacked;       /* Number bytes sent but not yet ACKed */
#endif

#ifdef CONFIG_NETDEV_MULTINIC
  /* If the TCP socket is bound to a local address, then this is
   * a reference to the device that routes traffic on the corresponding
   * network.
   */

  FAR struct net_driver_s *dev;
#endif

#ifdef CONFIG_NET_TCP_READAHEAD
  /* Read-ahead buffering.
   *
   *   readahead - A singly linked list of type struct iob_qentry_s
   *               where the TCP/IP read-ahead data is retained.
   */

  struct iob_queue_s readahead;   /* Read-ahead buffering */
#endif

#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
  /* Write buffering
   *
   *   write_q   - The queue of unsent I/O buffers.  The head of this
   *               list may be partially sent.  FIFO ordering.
   *   unacked_q - A queue of completely sent, but unacked I/O buffer
   *               chains.  Sequence number ordering.
   */

  sq_queue_t write_q;     /* Write buffering for segments */
  sq_queue_t unacked_q;   /* Write buffering for un-ACKed segments */
  uint16_t   expired;     /* Number segments retransmitted but not yet ACKed,
                           * it can only be updated at TCP_ESTABLISHED state */
  uint32_t   sent;        /* The number of bytes sent (ACKed and un-ACKed) */
  uint32_t   isn;         /* Initial sequence number */
#endif

#ifdef CONFIG_NET_TCPBACKLOG
  /* Listen backlog support
   *
   *   blparent - The backlog parent.  If this connection is backlogged,
   *     this field will be non-null and will refer to the TCP connection
   *     structure in which this connection is backlogged.
   *   backlog - The pending connection backlog.  If this connection is
   *     configured as a listener with backlog, then this refers to the
   *     struct tcp_backlog_s tear-off structure that manages that backlog.
   */

  FAR struct tcp_conn_s    *blparent;
  FAR struct tcp_backlog_s *backlog;
#endif

  /* Application callbacks:
   *
   * Data transfer events are retained in 'list'.  Event handlers in 'list'
   * are called for events specified in the flags set within struct
   * devif_callback_s
   *
   * When an callback is executed from 'list', the input flags are normally
   * returned, however, the implementation may set one of the following:
   *
   *   TCP_CLOSE   - Gracefully close the current connection
   *   TCP_ABORT   - Abort (reset) the current connection on an error that
   *                 prevents TCP_CLOSE from working.
   *
   * And/Or set/clear the following:
   *
   *   TCP_NEWDATA - May be cleared to indicate that the data was consumed
   *                 and that no further process of the new data should be
   *                 attempted.
   *   TCP_SNDACK  - If TCP_NEWDATA is cleared, then TCP_SNDACK may be set
   *                 to indicate that an ACK should be included in the response.
   *                 (In TCP_NEWDATA is cleared bu TCP_SNDACK is not set, then
   *                 dev->d_len should also be cleared).
   */

  FAR struct devif_callback_s *list;

  /* accept() is called when the TCP logic has created a connection
   *
   *   accept_private: This is private data that will be available to the
   *     accept() handler when it is invoked with a point to this structure
   *     as an argument.
   *   accept: This is the the pointer to the accept handler.
   */

  FAR void *accept_private;
  int (*accept)(FAR struct tcp_conn_s *listener, FAR struct tcp_conn_s *conn);

  /* connection_event() is called on any of the subset of connection-related
   * events.
   *
   *   connection_private: This is private data that will be available to
   *     the connection_event() handler when it is invoked with a point to
   *     this structure as an argument.
   *   connection_devcb: this is the allocated callback structure that is
   *     used to
   *   connection_event: This is the the pointer to the connection event
   *     handler.
   */

  FAR void *connection_private;
  FAR struct devif_callback_s *connection_devcb;
  uint16_t (*connection_event)(FAR struct net_driver_s *dev,
                               FAR void *pvconn, FAR void *pvpriv,
                               uint16_t flags);
};

/* This structure supports TCP write buffering */

#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
struct tcp_wrbuffer_s
{
  sq_entry_t wb_node;      /* Supports a singly linked list */
  uint32_t   wb_seqno;     /* Sequence number of the write segment */
  uint16_t   wb_sent;      /* Number of bytes sent from the I/O buffer chain */
  uint8_t    wb_nrtx;      /* The number of retransmissions for the last
                            * segment sent */
  struct iob_s *wb_iob;    /* Head of the I/O buffer chain */
};
#endif

/* Support for listen backlog:
 *
 *   struct tcp_blcontainer_s describes one backlogged connection
 *   struct tcp_backlog_s is a "tear-off" describing all backlog for a
 *      listener connection
 */

#ifdef CONFIG_NET_TCPBACKLOG
struct tcp_blcontainer_s
{
  sq_entry_t bc_node;             /* Implements a singly linked list */
  FAR struct tcp_conn_s *bc_conn; /* Holds reference to the new connection structure */
};

struct tcp_backlog_s
{
  sq_queue_t bl_free;             /* Implements a singly-linked list of free containers */
  sq_queue_t bl_pending;          /* Implements a singly-linked list of pending connections */
};
#endif

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

#if CONFIG_NSOCKET_DESCRIPTORS > 0
/* List of registered Ethernet device drivers.  You must have the network
 * locked in order to access this list.
 *
 * NOTE that this duplicates a declaration in net/netdev/netdev.h
 */

EXTERN struct net_driver_s *g_netdevices;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct sockaddr;  /* Forward reference */
struct socket;    /* Forward reference */
struct pollfd;    /* Forward reference */

/****************************************************************************
 * Name: tcp_initialize
 *
 * Description:
 *   Initialize the TCP/IP connection structures.  Called only once and only
 *   from the UIP layer at start-up in normal user mode.
 *
 ****************************************************************************/

void tcp_initialize(void);

/****************************************************************************
 * Name: tcp_alloc
 *
 * Description:
 *   Find a free TCP/IP connection structure and allocate it
 *   for use.  This is normally something done by the implementation of the
 *   socket() API but is also called from the driver level when a TCP
 *   packet is received while "listening"
 *
 ****************************************************************************/

FAR struct tcp_conn_s *tcp_alloc(uint8_t domain);

/****************************************************************************
 * Name: tcp_free
 *
 * Description:
 *   Free a connection structure that is no longer in use. This should be
 *   done by the implementation of close()
 *
 ****************************************************************************/

void tcp_free(FAR struct tcp_conn_s *conn);

/****************************************************************************
 * Name: tcp_active
 *
 * Description:
 *   Find a connection structure that is the appropriate
 *   connection to be used with the provided TCP/IP header
 *
 * Assumptions:
 *   Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

FAR struct tcp_conn_s *tcp_active(FAR struct net_driver_s *dev,
                                  FAR struct tcp_hdr_s *tcp);

/****************************************************************************
 * Name: tcp_nextconn
 *
 * Description:
 *   Traverse the list of active TCP connections
 *
 * Assumptions:
 *   Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

FAR struct tcp_conn_s *tcp_nextconn(FAR struct tcp_conn_s *conn);

/****************************************************************************
 * Function: tcp_find_ipv4_device
 *
 * Description:
 *   Select the network driver to use with the IPv4 TCP transaction.
 *
 * Input Parameters:
 *   conn - TCP connection structure.  The locally bound address, laddr,
 *     should be set to a non-zero value in this structure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned
 *   on failure.  -ENODEV is the only expected error value.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
int tcp_find_ipv4_device(FAR struct tcp_conn_s *conn);
#endif

/****************************************************************************
 * Function: tcp_find_ipv6_device
 *
 * Description:
 *   Select the network driver to use with the IPv6 TCP transaction.
 *
 * Input Parameters:
 *   conn - TCP connection structure.  The locally bound address, laddr,
 *     should be set to a non-zero value in this structure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned
 *   on failure.  -EHOSTUNREACH is the only expected error value.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
int tcp_find_ipv6_device(FAR struct tcp_conn_s *conn);
#endif

/****************************************************************************
 * Name: tcp_alloc_accept
 *
 * Description:
 *    Called when driver processing matches the incoming packet with a
 *    connection in LISTEN. In that case, this function will create a new
 *    connection and initialize it to send a SYNACK in return.
 *
 * Assumptions:
 *   Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

FAR struct tcp_conn_s *tcp_alloc_accept(FAR struct net_driver_s *dev,
                                        FAR struct tcp_hdr_s *tcp);

/****************************************************************************
 * Name: tcp_bind
 *
 * Description:
 *   This function implements the lower level parts of the standard TCP
 *   bind() operation.
 *
 * Return:
 *   0 on success or -EADDRINUSE on failure
 *
 * Assumptions:
 *   This function is called from normal user level code.
 *
 ****************************************************************************/

int tcp_bind(FAR struct tcp_conn_s *conn, FAR const struct sockaddr *addr);

/****************************************************************************
 * Name: tcp_connect
 *
 * Description:
 *   This function implements the lower level parts of the standard
 *   TCP connect() operation:  It connects to a remote host using TCP.
 *
 *   This function is used to start a new connection to the specified
 *   port on the specified host. It uses the connection structure that was
 *   allocated by a preceding socket() call.  It sets the connection to
 *   the SYN_SENT state and sets the retransmission timer to 0. This will
 *   cause a TCP SYN segment to be sent out the next time this connection
 *   is periodically processed, which usually is done within 0.5 seconds
 *   after the call to tcp_connect().
 *
 * Assumptions:
 *   This function is called from normal user level code.
 *
 ****************************************************************************/

int tcp_connect(FAR struct tcp_conn_s *conn, FAR const struct sockaddr *addr);

/****************************************************************************
 * Function: tcp_ipv4_select
 *
 * Description:
 *   Configure to send or receive an TCP IPv4 packet
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
void tcp_ipv4_select(FAR struct net_driver_s *dev);
#endif

/****************************************************************************
 * Function: tcp_ipv6_select
 *
 * Description:
 *   Configure to send or receive an TCP IPv6 packet
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
void tcp_ipv6_select(FAR struct net_driver_s *dev);
#endif

/****************************************************************************
 * Name: tcp_setsequence
 *
 * Description:
 *   Set the TCP/IP sequence number
 *
 * Assumptions:
 *   Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

void tcp_setsequence(FAR uint8_t *seqno, uint32_t value);

/****************************************************************************
 * Name: tcp_getsequence
 *
 * Description:
 *   Get the TCP/IP sequence number
 *
 * Assumptions:
 *   Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

uint32_t tcp_getsequence(FAR uint8_t *seqno);

/****************************************************************************
 * Name: tcp_addsequence
 *
 * Description:
 *   Add the length to get the next TCP sequence number.
 *
 * Assumptions:
 *   Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

uint32_t tcp_addsequence(FAR uint8_t *seqno, uint16_t len);

/****************************************************************************
 * Name: tcp_initsequence
 *
 * Description:
 *   Set the (initial) the TCP/IP sequence number when a TCP connection is
 *   established.
 *
 * Assumptions:
 *   Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

void tcp_initsequence(FAR uint8_t *seqno);

/****************************************************************************
 * Name: tcp_nextsequence
 *
 * Description:
 *   Increment the TCP/IP sequence number
 *
 * Assumptions:
 *   Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

void tcp_nextsequence(void);

/****************************************************************************
 * Name: tcp_poll
 *
 * Description:
 *   Poll a TCP connection structure for availability of TX data
 *
 * Parameters:
 *   dev - The device driver structure to use in the send operation
 *   conn - The TCP "connection" to poll for TX data
 *
 * Return:
 *   None
 *
 * Assumptions:
 *   Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

void tcp_poll(FAR struct net_driver_s *dev, FAR struct tcp_conn_s *conn);

/****************************************************************************
 * Name: tcp_timer
 *
 * Description:
 *   Handle a TCP timer expiration for the provided TCP connection
 *
 * Parameters:
 *   dev  - The device driver structure to use in the send operation
 *   conn - The TCP "connection" to poll for TX data
 *   hsed - The polling interval in halves of a second
 *
 * Return:
 *   None
 *
 * Assumptions:
 *   Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

void tcp_timer(FAR struct net_driver_s *dev, FAR struct tcp_conn_s *conn,
               int hsec);

/****************************************************************************
 * Function: tcp_listen_initialize
 *
 * Description:
 *   Setup the listening data structures
 *
 * Assumptions:
 *   Called early in the initialization phase while the system is still
 *   single-threaded.
 *
 ****************************************************************************/

void tcp_listen_initialize(void);

/****************************************************************************
 * Function: tcp_unlisten
 *
 * Description:
 *   Stop listening to the port bound to the specified TCP connection
 *
 * Assumptions:
 *   Called from normal user code.
 *
 ****************************************************************************/

int tcp_unlisten(FAR struct tcp_conn_s *conn);

/****************************************************************************
 * Function: tcp_listen
 *
 * Description:
 *   Start listening to the port bound to the specified TCP connection
 *
 * Assumptions:
 *   Called from normal user code.
 *
 ****************************************************************************/

int tcp_listen(FAR struct tcp_conn_s *conn);

/****************************************************************************
 * Function: tcp_islistener
 *
 * Description:
 *   Return true is there is a listener for the specified port
 *
 * Assumptions:
 *   Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

bool tcp_islistener(uint16_t portno);

/****************************************************************************
 * Function: tcp_accept_connection
 *
 * Description:
 *   Accept the new connection for the specified listening port.
 *
 * Assumptions:
 *   Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

int tcp_accept_connection(FAR struct net_driver_s *dev,
                          FAR struct tcp_conn_s *conn, uint16_t portno);

/****************************************************************************
 * Name: tcp_send
 *
 * Description:
 *   Setup to send a TCP packet
 *
 * Parameters:
 *   dev    - The device driver structure to use in the send operation
 *   conn   - The TCP connection structure holding connection information
 *   flags  - flags to apply to the TCP header
 *   len    - length of the message
 *
 * Return:
 *   None
 *
 * Assumptions:
 *   Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

void tcp_send(FAR struct net_driver_s *dev, FAR struct tcp_conn_s *conn,
              uint16_t flags, uint16_t len);

/****************************************************************************
 * Name: tcp_reset
 *
 * Description:
 *   Send a TCP reset (no-data) message
 *
 * Parameters:
 *   dev    - The device driver structure to use in the send operation
 *
 * Return:
 *   None
 *
 * Assumptions:
 *   Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

void tcp_reset(FAR struct net_driver_s *dev);

/****************************************************************************
 * Name: tcp_ack
 *
 * Description:
 *   Send the SYN or SYNACK response.
 *
 * Parameters:
 *   dev  - The device driver structure to use in the send operation
 *   conn - The TCP connection structure holding connection information
 *   ack  - The ACK response to send
 *
 * Return:
 *   None
 *
 * Assumptions:
 *   Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

void tcp_ack(FAR struct net_driver_s *dev, FAR struct tcp_conn_s *conn,
             uint8_t ack);

/****************************************************************************
 * Name: tcp_appsend
 *
 * Description:
 *   Handle application or TCP protocol response.  If this function is called
 *   with dev->d_sndlen > 0, then this is an application attempting to send
 *   packet.
 *
 * Parameters:
 *   dev    - The device driver structure to use in the send operation
 *   conn   - The TCP connection structure holding connection information
 *   result - App result event sent
 *
 * Return:
 *   None
 *
 * Assumptions:
 *   Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

void tcp_appsend(FAR struct net_driver_s *dev, FAR struct tcp_conn_s *conn,
                 uint16_t result);

/****************************************************************************
 * Name: tcp_rexmit
 *
 * Description:
 *   Handle application retransmission
 *
 * Parameters:
 *   dev    - The device driver structure to use in the send operation
 *   conn   - The TCP connection structure holding connection information
 *   result - App result event sent
 *
 * Return:
 *   None
 *
 * Assumptions:
 *   Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

void tcp_rexmit(FAR struct net_driver_s *dev, FAR struct tcp_conn_s *conn,
                uint16_t result);

/****************************************************************************
 * Name: tcp_ipv4_input
 *
 * Description:
 *   Handle incoming TCP input with IPv4 header
 *
 * Parameters:
 *   dev - The device driver structure containing the received TCP packet.
 *
 * Return:
 *   None
 *
 * Assumptions:
 *   Called from the Ethernet driver with the network stack locked
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
void tcp_ipv4_input(FAR struct net_driver_s *dev);
#endif

/****************************************************************************
 * Name: tcp_ipv6_input
 *
 * Description:
 *   Handle incoming TCP input with IPv4 header
 *
 * Parameters:
 *   dev - The device driver structure containing the received TCP packet.
 *
 * Return:
 *   None
 *
 * Assumptions:
 *   Called from the Ethernet driver with the network stack locked
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
void tcp_ipv6_input(FAR struct net_driver_s *dev);
#endif

/****************************************************************************
 * Function: tcp_callback
 *
 * Description:
 *   Inform the application holding the TCP socket of a change in state.
 *
 * Assumptions:
 *   Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

uint16_t tcp_callback(FAR struct net_driver_s *dev,
                      FAR struct tcp_conn_s *conn, uint16_t flags);

/****************************************************************************
 * Function: tcp_datahandler
 *
 * Description:
 *   Handle data that is not accepted by the application.  This may be called
 *   either (1) from the data receive logic if it cannot buffer the data, or
 *   (2) from the TCP event logic is there is no listener in place ready to
 *   receive the data.
 *
 * Input Parameters:
 *   conn - A pointer to the TCP connection structure
 *   buffer - A pointer to the buffer to be copied to the read-ahead
 *     buffers
 *   buflen - The number of bytes to copy to the read-ahead buffer.
 *
 * Returned value:
 *   The number of bytes actually buffered is returned.  This will be either
 *   zero or equal to buflen; partial packets are not buffered.
 *
 * Assumptions:
 * - The caller has checked that TCP_NEWDATA is set in flags and that is no
 *   other handler available to process the incoming data.
 * - Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP_READAHEAD
uint16_t tcp_datahandler(FAR struct tcp_conn_s *conn, FAR uint8_t *buffer,
                         uint16_t nbytes);
#endif

/****************************************************************************
 * Function: tcp_backlogcreate
 *
 * Description:
 *   Called from the listen() logic to setup the backlog as specified in the
 *   the listen arguments.
 *
 * Assumptions:
 *   Called from normal user code. Interrupts may be disabled.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCPBACKLOG
int tcp_backlogcreate(FAR struct tcp_conn_s *conn, int nblg);
#else
#  define tcp_backlogcreate(c,n) (-ENOSYS)
#endif

/****************************************************************************
 * Function: tcp_backlogdestroy
 *
 * Description:
 *   (1) Called from tcp_free() whenever a connection is freed.
 *   (2) Called from tcp_backlogcreate() to destroy any old backlog
 *
 *   NOTE: This function may re-enter tcp_free when a connection that
 *   is freed that has pending connections.
 *
 * Assumptions:
 *   Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCPBACKLOG
int tcp_backlogdestroy(FAR struct tcp_conn_s *conn);
#else
#  define tcp_backlogdestroy(conn)     (-ENOSYS)
#endif

/****************************************************************************
 * Function: tcp_backlogadd
 *
 * Description:
 *  Called tcp_listen when a new connection is made with a listener socket
 *  but when there is no accept() in place to receive the connection.  This
 *  function adds the new connection to the backlog.
 *
 * Assumptions:
 *   Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCPBACKLOG
int tcp_backlogadd(FAR struct tcp_conn_s *conn,
                   FAR struct tcp_conn_s *blconn);
#else
#  define tcp_backlogadd(conn,blconn)  (-ENOSYS)
#endif

/****************************************************************************
 * Function: tcp_backlogavailable
 *
 * Description:
 *  Called from poll().  Before waiting for a new connection, poll will
 *  call this API to see if there are pending connections in the backlog.
 *
 * Assumptions:
 *   Called from normal user code, but with interrupts disabled,
 *
 ****************************************************************************/

#if defined(CONFIG_NET_TCPBACKLOG) && !defined(CONFIG_DISABLE_POLL)
bool tcp_backlogavailable(FAR struct tcp_conn_s *conn);
#else
#  define tcp_backlogavailable(c) (false);
#endif

/****************************************************************************
 * Function: tcp_backlogremove
 *
 * Description:
 *  Called from accept().  Before waiting for a new connection, accept will
 *  call this API to see if there are pending connections in the backlog.
 *
 * Assumptions:
 *   Called from normal user code, but with interrupts disabled,
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCPBACKLOG
FAR struct tcp_conn_s *tcp_backlogremove(FAR struct tcp_conn_s *conn);
#else
#  define tcp_backlogremove(c) (NULL)
#endif

/****************************************************************************
 * Function: tcp_backlogdelete
 *
 * Description:
 *  Called from tcp_free() when a connection is freed that this also
 *  retained in the pending connection list of a listener.  We simply need
 *  to remove the defunct connection from the list.
 *
 * Assumptions:
 *   Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCPBACKLOG
int tcp_backlogdelete(FAR struct tcp_conn_s *conn,
                      FAR struct tcp_conn_s *blconn);
#else
#  define tcp_backlogdelete(c,b) (-ENOSYS)
#endif

/****************************************************************************
 * Function: tcp_accept
 *
 * Description:
 *   This function implements accept() for TCP/IP sockets.  See the
 *   description of accept() for further information.
 *
 * Parameters:
 *   psock    The listening TCP socket structure
 *   addr     Receives the address of the connecting client
 *   addrlen  Input: allocated size of 'addr', Return: returned size of 'addr'
 *   newconn  The new, accepted TCP connection structure
 *
 * Returned Value:
 *   Returns zero (OK) on success or a negated errno value on failure.
 *   See the description of accept of the possible errno values in the
 *   description of accept().
 *
 * Assumptions:
 *   Network is locked.
 *
 ****************************************************************************/

int psock_tcp_accept(FAR struct socket *psock, FAR struct sockaddr *addr,
                     FAR socklen_t *addrlen, FAR void **newconn);

/****************************************************************************
 * Function: psock_tcp_send
 *
 * Description:
 *   The psock_tcp_send() call may be used only when the TCP socket is in a
 *   connected state (so that the intended recipient is known).
 *
 * Parameters:
 *   psock    An instance of the internal socket structure.
 *   buf      Data to send
 *   len      Length of data to send
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   -1 is returned, and errno is set appropriately:
 *
 *   EAGAIN or EWOULDBLOCK
 *     The socket is marked non-blocking and the requested operation
 *     would block.
 *   EBADF
 *     An invalid descriptor was specified.
 *   ECONNRESET
 *     Connection reset by peer.
 *   EDESTADDRREQ
 *     The socket is not connection-mode, and no peer address is set.
 *   EFAULT
 *      An invalid user space address was specified for a parameter.
 *   EINTR
 *      A signal occurred before any data was transmitted.
 *   EINVAL
 *      Invalid argument passed.
 *   EISCONN
 *     The connection-mode socket was connected already but a recipient
 *     was specified. (Now either this error is returned, or the recipient
 *     specification is ignored.)
 *   EMSGSIZE
 *     The socket type requires that message be sent atomically, and the
 *     size of the message to be sent made this impossible.
 *   ENOBUFS
 *     The output queue for a network interface was full. This generally
 *     indicates that the interface has stopped sending, but may be
 *     caused by transient congestion.
 *   ENOMEM
 *     No memory available.
 *   ENOTCONN
 *     The socket is not connected, and no target has been given.
 *   ENOTSOCK
 *     The argument s is not a socket.
 *   EPIPE
 *     The local end has been shut down on a connection oriented socket.
 *     In this case the process will also receive a SIGPIPE unless
 *     MSG_NOSIGNAL is set.
 *
 * Assumptions:
 *
 ****************************************************************************/

struct socket;
ssize_t psock_tcp_send(FAR struct socket *psock, FAR const void *buf,
                       size_t len);

/****************************************************************************
 * Function: tcp_wrbuffer_initialize
 *
 * Description:
 *   Initialize the list of free write buffers
 *
 * Assumptions:
 *   Called once early initialization.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
void tcp_wrbuffer_initialize(void);
#endif /* CONFIG_NET_TCP_WRITE_BUFFERS */

/****************************************************************************
 * Function: tcp_wrbuffer_alloc
 *
 * Description:
 *   Allocate a TCP write buffer by taking a pre-allocated buffer from
 *   the free list.  This function is called from TCP logic when a buffer
 *   of TCP data is about to sent
 *
 * Input parameters:
 *   None
 *
 * Assumptions:
 *   Called from user logic with the network locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
struct tcp_wrbuffer_s;

FAR struct tcp_wrbuffer_s *tcp_wrbuffer_alloc(void);
#endif /* CONFIG_NET_TCP_WRITE_BUFFERS */

/****************************************************************************
 * Function: tcp_wrbuffer_release
 *
 * Description:
 *   Release a TCP write buffer by returning the buffer to the free list.
 *   This function is called from user logic after it is consumed the
 *   buffered data.
 *
 * Assumptions:
 *   Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
void tcp_wrbuffer_release(FAR struct tcp_wrbuffer_s *wrb);
#endif /* CONFIG_NET_TCP_WRITE_BUFFERS */

/****************************************************************************
 * Function: tcp_wrbuffer_dump
 *
 * Description:
 *   Dump the contents of a write buffer.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
#ifdef CONFIG_DEBUG
void tcp_wrbuffer_dump(FAR const char *msg, FAR struct tcp_wrbuffer_s *wrb,
                       unsigned int len, unsigned int offset);
#else
#  define tcp_wrbuffer_dump(msg,wrb)
#endif
#endif /* CONFIG_NET_TCP_WRITE_BUFFERS */

/****************************************************************************
 * Function: tcp_pollsetup
 *
 * Description:
 *   Setup to monitor events on one TCP/IP socket
 *
 * Input Parameters:
 *   psock - The TCP/IP socket of interest
 *   fds   - The structure describing the events to be monitored, OR NULL if
 *           this is a request to stop monitoring events.
 *
 * Returned Value:
 *  0: Success; Negated errno on failure
 *
 ****************************************************************************/

#ifdef HAVE_TCP_POLL
int tcp_pollsetup(FAR struct socket *psock, FAR struct pollfd *fds);
#endif

/****************************************************************************
 * Function: tcp_pollteardown
 *
 * Description:
 *   Teardown monitoring of events on an TCP/IP socket
 *
 * Input Parameters:
 *   psock - The TCP/IP socket of interest
 *   fds   - The structure describing the events to be monitored, OR NULL if
 *           this is a request to stop monitoring events.
 *
 * Returned Value:
 *  0: Success; Negated errno on failure
 *
 ****************************************************************************/

#ifdef HAVE_TCP_POLL
int tcp_pollteardown(FAR struct socket *psock, FAR struct pollfd *fds);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET_TCP */
#endif /* _NET_TCP_TCP_H */
