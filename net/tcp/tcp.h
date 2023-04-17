/****************************************************************************
 * net/tcp/tcp.h
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

#ifndef __NET_TCP_TCP_H
#define __NET_TCP_TCP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <nuttx/clock.h>
#include <nuttx/queue.h>
#include <nuttx/semaphore.h>
#include <nuttx/mm/iob.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/net.h>
#include <nuttx/net/tcp.h>
#include <nuttx/wqueue.h>

#ifdef CONFIG_NET_TCP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This is a helper pointer for accessing the contents of the tcp header */

#define TCPIPv4BUF ((FAR struct tcp_hdr_s *)IPBUF(IPv4_HDRLEN))
#define TCPIPv6BUF ((FAR struct tcp_hdr_s *)IPBUF(IPv6_HDRLEN))

#ifndef CONFIG_NET_TCP_NO_STACK

#define NET_TCP_HAVE_STACK 1

/* Allocate a new TCP data callback */

/* These macros allocate and free callback structures used for receiving
 * notifications of TCP data-related events.
 */

#define tcp_callback_alloc(conn) \
  devif_callback_alloc((conn)->dev, &(conn)->sconn.list, &(conn)->sconn.list_tail)
#define tcp_callback_free(conn,cb) \
  devif_conn_callback_free((conn)->dev, (cb), &(conn)->sconn.list, &(conn)->sconn.list_tail)

#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
/* TCP write buffer access macros */

#  define TCP_WBSEQNO(wrb)           ((wrb)->wb_seqno)
#  define TCP_WBPKTLEN(wrb)          ((wrb)->wb_iob->io_pktlen)
#  define TCP_WBSENT(wrb)            ((wrb)->wb_sent)
#  define TCP_WBNRTX(wrb)            ((wrb)->wb_nrtx)
#ifdef CONFIG_NET_TCP_FAST_RETRANSMIT
#  define TCP_WBNACK(wrb)            ((wrb)->wb_nack)
#endif
#  define TCP_WBIOB(wrb)             ((wrb)->wb_iob)
#  define TCP_WBCOPYOUT(wrb,dest,n)  (iob_copyout(dest,(wrb)->wb_iob,(n),0))
#  define TCP_WBCOPYIN(wrb,src,n,off) \
     (iob_copyin((wrb)->wb_iob,src,(n),(off),true))
#  define TCP_WBTRYCOPYIN(wrb,src,n,off) \
     (iob_trycopyin((wrb)->wb_iob,src,(n),(off),true))

#  define TCP_WBTRIM(wrb,n) \
     do { (wrb)->wb_iob = iob_trimhead((wrb)->wb_iob,(n)); } while (0)

#ifdef CONFIG_DEBUG_FEATURES
#  define TCP_WBDUMP(msg,wrb,len,offset) \
     tcp_wrbuffer_dump(msg,wrb,len,offset)
#  else
#    define TCP_WBDUMP(msg,wrb,len,offset)
#  endif
#endif

/* 32-bit modular arithmetics for tcp sequence numbers */

#define TCP_SEQ_LT(a, b)	((int32_t)((a) - (b)) < 0)
#define TCP_SEQ_GT(a, b)	TCP_SEQ_LT(b, a)
#define TCP_SEQ_LTE(a, b)	(!TCP_SEQ_GT(a, b))
#define TCP_SEQ_GTE(a, b)	(!TCP_SEQ_LT(a, b))

#define TCP_SEQ_ADD(a, b)	((uint32_t)((a) + (b)))
#define TCP_SEQ_SUB(a, b)	((uint32_t)((a) - (b)))

/* The TCP options flags */

#define TCP_WSCALE            0x01U /* Window Scale option enabled */
#define TCP_SACK              0x02U /* Selective ACKs enabled */

/* The Max Range count of TCP Selective ACKs */

#define TCP_SACK_RANGES_MAX   4

/* After receiving 3 duplicate ACKs, TCP performs a retransmission
 * (RFC 5681 (3.2))
 */

#define TCP_FAST_RETRANSMISSION_THRESH 3

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

struct file;      /* Forward reference */
struct sockaddr;  /* Forward reference */
struct socket;    /* Forward reference */
struct pollfd;    /* Forward reference */

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

/* This is a container that holds the poll-related information */

struct tcp_poll_s
{
  FAR struct tcp_conn_s *conn;     /* Needed to handle loss of connection */
  struct pollfd *fds;              /* Needed to handle poll events */
  FAR struct devif_callback_s *cb; /* Needed to teardown the poll */
};

/* Out-of-order segments */

struct tcp_ofoseg_s
{
  uint32_t         left;  /* Left edge of segment */
  uint32_t         right; /* Right edge of segment */
  FAR struct iob_s *data; /* Out-of-order buffering */
};

/* SACK ranges to include in ACK packets. */

struct tcp_sack_s
{
  uint32_t left;    /* Left edge of the SACK */
  uint32_t right;   /* Right edge of the SACK */
};

struct tcp_conn_s
{
  /* Common prologue of all connection structures. */

  /* TCP callbacks:
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
   *   TCP_SNDACK  - If TCP_NEWDATA is cleared, then TCP_SNDACK may be set to
   *                 indicate that an ACK should be included in the response.
   *                 (In TCP_NEWDATA is cleared bu TCP_SNDACK is not set,
   *                 then dev->d_len should also be cleared).
   */

  struct socket_conn_s sconn;

  /* TCP-specific content follows */

  union ip_binding_u u;   /* IP address binding */
  uint8_t  rcvseq[4];     /* The sequence number that we expect to
                           * receive next */
  uint8_t  sndseq[4];     /* The sequence number that was last sent by us */
#if !defined(CONFIG_NET_TCP_WRITE_BUFFERS) || \
    defined(CONFIG_NET_SENDFILE)
  uint32_t rexmit_seq;    /* The sequence number to be retrasmitted */
#endif
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
  struct   work_s work;   /* TCP timer handle */
  bool     timeout;       /* Trigger from timer expiry */
  uint8_t  timer;         /* The retransmission timer (units: half-seconds) */
  uint8_t  nrtx;          /* The number of retransmissions for the last
                           * segment sent */
#ifdef CONFIG_NET_TCP_DELAYED_ACK
  uint8_t  rx_unackseg;   /* Number of un-ACKed received segments */
  uint8_t  rx_acktimer;   /* Time since last ACK sent (units: half-seconds) */
#endif
  uint16_t lport;         /* The local TCP port, in network byte order */
  uint16_t rport;         /* The remoteTCP port, in network byte order */
  uint16_t mss;           /* Current maximum segment size for the
                           * connection */
#ifdef CONFIG_NET_TCPPROTO_OPTIONS
  uint16_t user_mss;      /* Configured maximum segment size for the
                           * connection */
#endif
  uint32_t rcv_adv;       /* The right edge of the recv window advertized */
#ifdef CONFIG_NET_TCP_WINDOW_SCALE
  uint32_t snd_wnd;       /* Sequence and acknowledgement numbers of last
                           * window update */
  uint8_t  snd_scale;     /* Sender window scale factor */
  uint8_t  rcv_scale;     /* Receiver windows scale factor */
#else
  uint16_t snd_wnd;       /* Sequence and acknowledgement numbers of last
                           * window update */
#endif
  uint32_t snd_wl1;
  uint32_t snd_wl2;
#if CONFIG_NET_RECV_BUFSIZE > 0
  int32_t  rcv_bufs;      /* Maximum amount of bytes queued in recv */
#endif
#if CONFIG_NET_SEND_BUFSIZE > 0
  int32_t  snd_bufs;      /* Maximum amount of bytes queued in send */
  sem_t    snd_sem;       /* Semaphore signals send completion */
#endif
#if defined(CONFIG_NET_TCP_WRITE_BUFFERS) || \
    defined(CONFIG_NET_TCP_WINDOW_SCALE)
  uint32_t tx_unacked;    /* Number bytes sent but not yet ACKed */
#else
  uint16_t tx_unacked;    /* Number bytes sent but not yet ACKed */
#endif
  uint16_t flags;         /* Flags of TCP-specific options */
#ifdef CONFIG_NET_SOLINGER
  sclock_t ltimeout;      /* Linger timeout expiration */
#endif
  /* If the TCP socket is bound to a local address, then this is
   * a reference to the device that routes traffic on the corresponding
   * network.
   */

  FAR struct net_driver_s *dev;

  /* Read-ahead buffering.
   *
   *   readahead - A singly linked list of type struct iob_s
   *               where the TCP/IP read-ahead data is retained.
   */

  struct iob_s *readahead;   /* Read-ahead buffering */

#ifdef CONFIG_NET_TCP_OUT_OF_ORDER

  /* Number of out-of-order segments */

  uint8_t nofosegs;

  /* This defines a out of order segment block. */

  struct tcp_ofoseg_s ofosegs[TCP_SACK_RANGES_MAX];
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
  uint32_t   sndseq_max;  /* The sequence number of next not-retransmitted
                           * segment (next greater sndseq) */
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

#ifdef CONFIG_NET_TCP_KEEPALIVE
  /* There fields manage TCP/IP keep-alive.  All times are in units of the
   * system clock tick.
   */

  uint32_t   keeptimer;   /* KeepAlive timer (dsec) */
  uint32_t   keepidle;    /* Elapsed idle time before first probe sent (dsec) */
  uint32_t   keepintvl;   /* Interval between probes (dsec) */
  bool       keepalive;   /* True: KeepAlive enabled; false: disabled */
  uint8_t    keepcnt;     /* Number of retries before the socket is closed */
  uint8_t    keepretries; /* Number of retries attempted */
#endif

#if defined(CONFIG_NET_SENDFILE) && defined(CONFIG_NET_TCP_WRITE_BUFFERS)
  bool       sendfile;    /* True if sendfile operation is in progress */
#endif

  /* connevents is a list of callbacks for each socket the uses this
   * connection (there can be more that one in the event that the the socket
   * was dup'ed).  It is used with the network monitor to handle
   * asynchronous loss-of-connection events.
   */

  FAR struct devif_callback_s *connevents;
  FAR struct devif_callback_s *connevents_tail;

  /* Reference to TCP shutdown/close callback instance */

  FAR struct devif_callback_s *shdcb;
  FAR struct devif_callback_s *clscb;
  struct work_s                clswork;

#if defined(CONFIG_NET_TCP_WRITE_BUFFERS)
  /* Callback instance for TCP send() */

  FAR struct devif_callback_s *sndcb;
#endif

  /* accept() is called when the TCP logic has created a connection
   *
   *   accept_private: This is private data that will be available to the
   *     accept() handler when it is invoked with a point to this structure
   *     as an argument.
   *   accept: This is the pointer to the accept handler.
   */

  FAR void *accept_private;
  int (*accept)(FAR struct tcp_conn_s *listener,
                FAR struct tcp_conn_s *conn);

  /* The following is a list of poll structures of threads waiting for
   * socket events.
   */

  struct tcp_poll_s pollinfo[CONFIG_NET_TCP_NPOLLWAITERS];
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
#ifdef CONFIG_NET_TCP_FAST_RETRANSMIT
  uint8_t    wb_nack;      /* The number of ack count */
#endif
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
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_initialize
 *
 * Description:
 *   Initialize the TCP/IP connection structures.  Called only once and only
 *   from the network layer at start-up.
 *
 ****************************************************************************/

void tcp_initialize(void);

/****************************************************************************
 * Name: tcp_alloc
 *
 * Description:
 *   Find a free TCP/IP connection structure and allocate it
 *   for use.  This is normally something done by the implementation of the
 *   socket() API but is also called from the event processing logic when a
 *   TCP packet is received while "listening"
 *
 ****************************************************************************/

FAR struct tcp_conn_s *tcp_alloc(uint8_t domain);

/****************************************************************************
 * Name: tcp_free_rx_buffers
 *
 * Description:
 *   Free rx buffer of a connection
 *
 ****************************************************************************/

void tcp_free_rx_buffers(FAR struct tcp_conn_s *conn);

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
 * Name: tcp_local_ipv4_device
 *
 * Description:
 *   Select the network driver to use with the IPv4 TCP transaction based
 *   on the locally bound IPv4 address
 *
 * Input Parameters:
 *   conn - TCP connection structure.  The locally bound address, laddr,
 *     should be set to a non-zero value in this structure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned
 *   on failure.  -ENETUNREACH is the only expected error value.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
int tcp_local_ipv4_device(FAR struct tcp_conn_s *conn);
#endif

/****************************************************************************
 * Name: tcp_remote_ipv4_device
 *
 * Description:
 *   Select the network driver to use with the IPv4 TCP transaction based
 *   on the remotely connected IPv4 address
 *
 * Input Parameters:
 *   conn - TCP connection structure.  The remotely connected address, raddr,
 *     should be set to a non-zero value in this structure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned
 *   on failure.  -ENETUNREACH is the only expected error value.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
int tcp_remote_ipv4_device(FAR struct tcp_conn_s *conn);
#endif

/****************************************************************************
 * Name: tcp_local_ipv6_device
 *
 * Description:
 *   Select the network driver to use with the IPv6 TCP transaction based
 *   on the locally bound IPv6 address
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
int tcp_local_ipv6_device(FAR struct tcp_conn_s *conn);
#endif

/****************************************************************************
 * Name: tcp_remote_ipv6_device
 *
 * Description:
 *   Select the network driver to use with the IPv6 TCP transaction based
 *   on the remotely connected IPv6 address
 *
 * Input Parameters:
 *   conn - TCP connection structure.  The remotely connected address, raddr,
 *     should be set to a non-zero value in this structure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned
 *   on failure.  -EHOSTUNREACH is the only expected error value.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
int tcp_remote_ipv6_device(FAR struct tcp_conn_s *conn);
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
                                        FAR struct tcp_hdr_s *tcp,
                                        FAR struct tcp_conn_s *listener);

/****************************************************************************
 * Name: tcp_selectport
 *
 * Description:
 *   If the port number is zero; select an unused port for the connection.
 *   If the port number is non-zero, verify that no other connection has
 *   been created with this port number.
 *
 * Returned Value:
 *   Selected or verified port number in network order on success, a negated
 *   errno on failure.
 *
 * Assumptions:
 *   Interrupts are disabled
 *
 ****************************************************************************/

int tcp_selectport(uint8_t domain,
                   FAR const union ip_addr_u *ipaddr,
                   uint16_t portno);

/****************************************************************************
 * Name: tcp_bind
 *
 * Description:
 *   This function implements the lower level parts of the standard TCP
 *   bind() operation.
 *
 * Returned Value:
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

int tcp_connect(FAR struct tcp_conn_s *conn,
                FAR const struct sockaddr *addr);

/****************************************************************************
 * Name: psock_tcp_connect
 *
 * Description:
 *   Perform a TCP connection
 *
 * Input Parameters:
 *   psock - A reference to the structure of the socket to be connected
 *   addr  - The address of the remote server to connect to
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked
 *
 ****************************************************************************/

int psock_tcp_connect(FAR struct socket *psock,
                      FAR const struct sockaddr *addr);

/****************************************************************************
 * Name: tcp_start_monitor
 *
 * Description:
 *   Set up to receive TCP connection state changes for a given socket
 *
 * Input Parameters:
 *   psock - The socket of interest
 *
 * Returned Value:
 *   On success, tcp_start_monitor returns OK; On any failure,
 *   tcp_start_monitor will return a negated errno value.  The only failure
 *   that can occur is if the socket has already been closed and, in this
 *   case, -ENOTCONN is returned.
 *
 * Assumptions:
 *   The caller holds the network lock (if not, it will be locked momentarily
 *   by this function).
 *
 ****************************************************************************/

int tcp_start_monitor(FAR struct socket *psock);

/****************************************************************************
 * Name: tcp_stop_monitor
 *
 * Description:
 *   Stop monitoring TCP connection changes for a sockets associated with
 *   a given TCP connection structure.
 *
 * Input Parameters:
 *   conn - The TCP connection of interest
 *   flags    Set of disconnection events
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The caller holds the network lock (if not, it will be locked momentarily
 *   by this function).
 *
 ****************************************************************************/

void tcp_stop_monitor(FAR struct tcp_conn_s *conn, uint16_t flags);

/****************************************************************************
 * Name: tcp_lost_connection
 *
 * Description:
 *   Called when a loss-of-connection event has been detected by network
 *   event handling logic.  Perform operations like tcp_stop_monitor but
 *   (1) explicitly mark this socket and (2) disable further callbacks
 *   the event handler.
 *
 * Input Parameters:
 *   conn  - The TCP connection of interest
 *   cb    - devif callback structure
 *   flags - Set of connection events events
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The caller holds the network lock (if not, it will be locked momentarily
 *   by this function).
 *
 ****************************************************************************/

void tcp_lost_connection(FAR struct tcp_conn_s *conn,
                         FAR struct devif_callback_s *cb, uint16_t flags);

/****************************************************************************
 * Name: tcp_close
 *
 * Description:
 *   Break any current TCP connection
 *
 * Input Parameters:
 *   psock - An instance of the internal socket structure.
 *
 * Assumptions:
 *   Called from normal user-level logic
 *
 ****************************************************************************/

int tcp_close(FAR struct socket *psock);

/****************************************************************************
 * Name: tcp_shutdown
 *
 * Description:
 *   Gracefully shutdown a TCP connection by sending a SYN
 *
 * Input Parameters:
 *   psock - An instance of the internal socket structure.
 *   how   - Specifies the type of shutdown.
 *
 * Assumptions:
 *   Called from normal user-level logic
 *
 ****************************************************************************/

int tcp_shutdown(FAR struct socket *psock, int how);

/****************************************************************************
 * Name: tcp_ipv4_select
 *
 * Description:
 *   Configure to send or receive an TCP IPv4 packet
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
void tcp_ipv4_select(FAR struct net_driver_s *dev);
#endif

/****************************************************************************
 * Name: tcp_ipv6_select
 *
 * Description:
 *   Configure to send or receive an TCP IPv6 packet
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
void tcp_ipv6_select(FAR struct net_driver_s *dev);
#endif

/****************************************************************************
 * Name: tcp_ip_select
 *
 * Description:
 *   Configure to send or receive an TCP IPv[4|6] packet for connection
 *
 ****************************************************************************/

void tcp_ip_select(FAR struct tcp_conn_s *conn);

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
 * Input Parameters:
 *   dev - The device driver structure to use in the send operation
 *   conn - The TCP "connection" to poll for TX data
 *
 * Returned Value:
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
 * Input Parameters:
 *   dev  - The device driver structure to use in the send operation
 *   conn - The TCP "connection" to poll for TX data
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

void tcp_timer(FAR struct net_driver_s *dev, FAR struct tcp_conn_s *conn);

/****************************************************************************
 * Name: tcp_update_timer
 *
 * Description:
 *   Update the TCP timer for the provided TCP connection,
 *   The timeout is accurate
 *
 * Input Parameters:
 *   conn - The TCP "connection" to poll for TX data
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   conn is not NULL.
 *   The connection (conn) is bound to the polling device (dev).
 *
 ****************************************************************************/

void tcp_update_timer(FAR struct tcp_conn_s *conn);

/****************************************************************************
 * Name: tcp_update_retrantimer
 *
 * Description:
 *   Update the retransmit TCP timer for the provided TCP connection,
 *   The timeout is accurate
 *
 * Input Parameters:
 *   conn    - The TCP "connection" to poll for TX data
 *   timeout - Time for the next timeout
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   conn is not NULL.
 *   The connection (conn) is bound to the polling device (dev).
 *
 ****************************************************************************/

void tcp_update_retrantimer(FAR struct tcp_conn_s *conn, int timeout);

/****************************************************************************
 * Name: tcp_update_keeptimer
 *
 * Description:
 *   Update the keeplive TCP timer for the provided TCP connection,
 *   The timeout is accurate
 *
 * Input Parameters:
 *   conn    - The TCP "connection" to poll for TX data
 *   timeout - Time for the next timeout
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   conn is not NULL.
 *   The connection (conn) is bound to the polling device (dev).
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP_KEEPALIVE
void tcp_update_keeptimer(FAR struct tcp_conn_s *conn, int timeout);
#endif

/****************************************************************************
 * Name: tcp_stop_timer
 *
 * Description:
 *   Stop TCP timer for the provided TCP connection
 *   When the connection is closed
 *
 * Input Parameters:
 *   conn - The TCP "connection" to poll for TX data
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   conn is not NULL.
 *
 ****************************************************************************/

void tcp_stop_timer(FAR struct tcp_conn_s *conn);

/****************************************************************************
 * Name: tcp_findlistener
 *
 * Description:
 *   Return the connection listener for connections on this port (if any)
 *
 * Assumptions:
 *   The network is locked
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NET_IPv6)
FAR struct tcp_conn_s *tcp_findlistener(FAR union ip_binding_u *uaddr,
                                        uint16_t portno,
                                        uint8_t domain);
#else
FAR struct tcp_conn_s *tcp_findlistener(FAR union ip_binding_u *uaddr,
                                        uint16_t portno);
#endif

/****************************************************************************
 * Name: tcp_unlisten
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
 * Name: tcp_listen
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
 * Name: tcp_islistener
 *
 * Description:
 *   Return true is there is a listener for the specified port
 *
 * Assumptions:
 *   Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NET_IPv6)
bool tcp_islistener(FAR union ip_binding_u *uaddr, uint16_t portno,
                    uint8_t domain);
#else
bool tcp_islistener(FAR union ip_binding_u *uaddr, uint16_t portno);
#endif

/****************************************************************************
 * Name: tcp_accept_connection
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
 * Input Parameters:
 *   dev    - The device driver structure to use in the send operation
 *   conn   - The TCP connection structure holding connection information
 *   flags  - flags to apply to the TCP header
 *   len    - length of the message
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

void tcp_send(FAR struct net_driver_s *dev, FAR struct tcp_conn_s *conn,
              uint16_t flags, uint16_t len);

/****************************************************************************
 * Name: tcp_sendfile
 *
 * Description:
 *   The tcp_sendfile() call may be used only when the INET socket is in a
 *   connected state (so that the intended recipient is known).
 *
 * Input Parameters:
 *   psock    An instance of the internal socket structure.
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   a negated errno value is returned.  See sendfile() for a list
 *   appropriate error return values.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_SENDFILE
ssize_t tcp_sendfile(FAR struct socket *psock, FAR struct file *infile,
                      FAR off_t *offset, size_t count);
#endif

/****************************************************************************
 * Name: tcp_reset
 *
 * Description:
 *   Send a TCP reset (no-data) message
 *
 * Input Parameters:
 *   dev    - The device driver structure to use in the send operation
 *   conn   - The TCP connection structure holding connection information
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

void tcp_reset(FAR struct net_driver_s *dev, FAR struct tcp_conn_s *conn);

/****************************************************************************
 * Name: tcp_rx_mss
 *
 * Description:
 *   Return the MSS to advertize to the peer.
 *
 * Input Parameters:
 *   dev  - The device driver structure
 *
 * Returned Value:
 *   The MSS value.
 *
 ****************************************************************************/

uint16_t tcp_rx_mss(FAR struct net_driver_s *dev);

/****************************************************************************
 * Name: tcp_synack
 *
 * Description:
 *   Send the SYN or SYNACK response.
 *
 * Input Parameters:
 *   dev  - The device driver structure to use in the send operation
 *   conn - The TCP connection structure holding connection information
 *   ack  - The ACK response to send
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

void tcp_synack(FAR struct net_driver_s *dev, FAR struct tcp_conn_s *conn,
                uint8_t ack);

/****************************************************************************
 * Name: tcp_appsend
 *
 * Description:
 *   Handle application or TCP protocol response.  If this function is called
 *   with dev->d_sndlen > 0, then this is an application attempting to send
 *   packet.
 *
 * Input Parameters:
 *   dev    - The device driver structure to use in the send operation
 *   conn   - The TCP connection structure holding connection information
 *   result - App result event sent
 *
 * Returned Value:
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
 * Input Parameters:
 *   dev    - The device driver structure to use in the send operation
 *   conn   - The TCP connection structure holding connection information
 *   result - App result event sent
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

void tcp_rexmit(FAR struct net_driver_s *dev, FAR struct tcp_conn_s *conn,
                uint16_t result);

/****************************************************************************
 * Name: tcp_send_txnotify
 *
 * Description:
 *   Notify the appropriate device driver that we are have data ready to
 *   be send (TCP)
 *
 * Input Parameters:
 *   psock - Socket state structure
 *   conn  - The TCP connection structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void tcp_send_txnotify(FAR struct socket *psock,
                       FAR struct tcp_conn_s *conn);

/****************************************************************************
 * Name: tcp_ipv4_input
 *
 * Description:
 *   Handle incoming TCP input with IPv4 header
 *
 * Input Parameters:
 *   dev - The device driver structure containing the received TCP packet.
 *
 * Returned Value:
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
 * Input Parameters:
 *   dev   - The device driver structure containing the received TCP packet.
 *   iplen - The size of the IPv6 header.  This may be larger than
 *           IPv6_HDRLEN the IPv6 header if IPv6 extension headers are
 *           present.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called from the Ethernet driver with the network stack locked
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
void tcp_ipv6_input(FAR struct net_driver_s *dev, unsigned int iplen);
#endif

/****************************************************************************
 * Name: tcp_callback
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
 * Name: tcp_datahandler
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
 * Returned Value:
 *   The number of bytes actually buffered is returned.  This will be either
 *   zero or equal to buflen; partial packets are not buffered.
 *
 * Assumptions:
 * - The caller has checked that TCP_NEWDATA is set in flags and that is no
 *   other handler available to process the incoming data.
 * - Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

uint16_t tcp_datahandler(FAR struct net_driver_s *dev,
                         FAR struct tcp_conn_s *conn,
                         uint16_t offset);

/****************************************************************************
 * Name: tcp_dataconcat
 *
 * Description:
 *   Concatenate iob_s chain iob2 to iob1, if CONFIG_NET_TCP_RECV_PACK is
 *   endabled, pack all data in the I/O buffer chain.
 *
 * Returned Value:
 *   The number of bytes actually buffered is returned.  This will be either
 *   zero or equal to iob1->io_pktlen.
 *
 ****************************************************************************/

uint16_t tcp_dataconcat(FAR struct iob_s **iob1, FAR struct iob_s **iob2);

/****************************************************************************
 * Name: tcp_backlogcreate
 *
 * Description:
 *   Called from the listen() logic to setup the backlog as specified in the
 *   the listen arguments.
 *
 * Assumptions:
 *   Called from network socket logic.  The network may or may not be locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCPBACKLOG
int tcp_backlogcreate(FAR struct tcp_conn_s *conn, int nblg);
#else
#  define tcp_backlogcreate(c,n) (-ENOSYS)
#endif

/****************************************************************************
 * Name: tcp_backlogdestroy
 *
 * Description:
 *   (1) Called from tcp_free() whenever a connection is freed.
 *   (2) Called from tcp_backlogcreate() to destroy any old backlog
 *
 *   NOTE: This function may re-enter tcp_free when a connection that
 *   is freed that has pending connections.
 *
 * Assumptions:
 *   Called from network socket logic with the network stack locked
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCPBACKLOG
int tcp_backlogdestroy(FAR struct tcp_conn_s *conn);
#else
#  define tcp_backlogdestroy(conn)     (-ENOSYS)
#endif

/****************************************************************************
 * Name: tcp_backlogadd
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
 * Name: tcp_backlogpending
 *
 * Description:
 *   Called from poll().  Before waiting for a new connection, poll will
 *   call this API to see if there are pending connections in the backlog.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCPBACKLOG
bool tcp_backlogpending(FAR struct tcp_conn_s *conn);
#else
#  define tcp_backlogpending(c) (false)
#endif

/****************************************************************************
 * Name: tcp_backlogavailable
 *
 * Description:
 *  Called from tcp_input().  Before alloc a new accept connection, tcp_input
 *  will call this API to see if there are free node in the backlog.
 *
 * Assumptions:
 *   Called from network socket logic with the network locked
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCPBACKLOG
bool tcp_backlogavailable(FAR struct tcp_conn_s *conn);
#else
#  define tcp_backlogavailable(c) (true)
#endif

/****************************************************************************
 * Name: tcp_backlogremove
 *
 * Description:
 *   Called from accept().  Before waiting for a new connection, accept will
 *   call this API to see if there are pending connections in the backlog.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCPBACKLOG
FAR struct tcp_conn_s *tcp_backlogremove(FAR struct tcp_conn_s *conn);
#else
#  define tcp_backlogremove(c) (NULL)
#endif

/****************************************************************************
 * Name: tcp_backlogdelete
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
 * Name: tcp_accept
 *
 * Description:
 *   This function implements accept() for TCP/IP sockets.  See the
 *   description of accept() for further information.
 *
 * Input Parameters:
 *   psock    The listening TCP socket structure
 *   addr     Receives the address of the connecting client
 *   addrlen  Input: allocated size of 'addr'
 *            Return: returned size of 'addr'
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
 * Name: psock_tcp_recvfrom
 *
 * Description:
 *   Perform the recvfrom operation for a TCP/IP SOCK_STREAM
 *
 * Input Parameters:
 *   psock    Pointer to the socket structure for the SOCK_DRAM socket
 *   msg      Receive info and buffer for receive data
 *   flags    Receive flags
 *
 * Returned Value:
 *   On success, returns the number of characters received.  On  error,
 *   -errno is returned (see recvfrom for list of errnos).
 *
 * Assumptions:
 *
 ****************************************************************************/

ssize_t psock_tcp_recvfrom(FAR struct socket *psock, FAR struct msghdr *msg,
                           int flags);

/****************************************************************************
 * Name: psock_tcp_send
 *
 * Description:
 *   The psock_tcp_send() call may be used only when the TCP socket is in a
 *   connected state (so that the intended recipient is known).
 *
 * Input Parameters:
 *   psock    An instance of the internal socket structure.
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags
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

ssize_t psock_tcp_send(FAR struct socket *psock, FAR const void *buf,
                       size_t len, int flags);

/****************************************************************************
 * Name: tcp_setsockopt
 *
 * Description:
 *   tcp_setsockopt() sets the TCP-protocol option specified by the
 *   'option' argument to the value pointed to by the 'value' argument for
 *   the socket specified by the 'psock' argument.
 *
 *   See <netinet/tcp.h> for the a complete list of values of TCP protocol
 *   options.
 *
 * Input Parameters:
 *   psock     Socket structure of socket to operate on
 *   option    identifies the option to set
 *   value     Points to the argument value
 *   value_len The length of the argument value
 *
 * Returned Value:
 *   Returns zero (OK) on success.  On failure, it returns a negated errno
 *   value to indicate the nature of the error.  See psock_setcockopt() for
 *   the list of possible error values.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCPPROTO_OPTIONS
int tcp_setsockopt(FAR struct socket *psock, int option,
                   FAR const void *value, socklen_t value_len);
#endif

/****************************************************************************
 * Name: tcp_getsockopt
 *
 * Description:
 *   tcp_getsockopt() retrieves the value for the option specified by the
 *   'option' argument for the socket specified by the 'psock' argument.  If
 *   the size of the option value is greater than 'value_len', the value
 *   stored in the object pointed to by the 'value' argument will be silently
 *   truncated. Otherwise, the length pointed to by the 'value_len' argument
 *   will be modified to indicate the actual length of the 'value'.
 *
 *   The 'level' argument specifies the protocol level of the option. To
 *   retrieve options at the socket level, specify the level argument as
 *   SOL_SOCKET; to retrieve options at the TCP-protocol level, the level
 *   argument is SOL_TCP.
 *
 *   See <sys/socket.h> a complete list of values for the socket-level
 *   'option' argument.  Protocol-specific options are are protocol specific
 *   header files (such as netinet/tcp.h for the case of the TCP protocol).
 *
 * Input Parameters:
 *   psock     Socket structure of the socket to query
 *   level     Protocol level to set the option
 *   option    identifies the option to get
 *   value     Points to the argument value
 *   value_len The length of the argument value
 *
 * Returned Value:
 *   Returns zero (OK) on success.  On failure, it returns a negated errno
 *   value to indicate the nature of the error.  See psock_getsockopt() for
 *   the complete list of appropriate return error codes.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCPPROTO_OPTIONS
int tcp_getsockopt(FAR struct socket *psock, int option,
                   FAR void *value, FAR socklen_t *value_len);
#endif

/****************************************************************************
 * Name: tcp_get_recvwindow
 *
 * Description:
 *   Calculate the TCP receive window for the specified device.
 *
 * Input Parameters:
 *   dev  - The device whose TCP receive window will be updated.
 *   conn - The TCP connection structure holding connection information.
 *
 * Returned Value:
 *   The value of the TCP receive window to use.
 *
 ****************************************************************************/

uint32_t tcp_get_recvwindow(FAR struct net_driver_s *dev,
                            FAR struct tcp_conn_s *conn);

/****************************************************************************
 * Name: tcp_should_send_recvwindow
 *
 * Description:
 *   Determine if we should advertize the new recv window to the peer.
 *
 * Input Parameters:
 *   conn - The TCP connection structure holding connection information.
 *
 * Returned Value:
 *   If we should send an update.
 *
 ****************************************************************************/

bool tcp_should_send_recvwindow(FAR struct tcp_conn_s *conn);

/****************************************************************************
 * Name: psock_tcp_cansend
 *
 * Description:
 *   psock_tcp_cansend() returns a value indicating if a write to the socket
 *   would block.  No space in the buffer is actually reserved, so it is
 *   possible that the write may still block if the buffer is filled by
 *   another means.
 *
 * Input Parameters:
 *   conn     The TCP connection of interest
 *
 * Returned Value:
 *   OK
 *     At least one byte of data could be successfully written.
 *   -EWOULDBLOCK
 *     There is no room in the output buffer.
 *   -EBADF
 *     An invalid descriptor was specified.
 *   -ENOTCONN
 *     The socket is not connected.
 *
 ****************************************************************************/

int psock_tcp_cansend(FAR struct tcp_conn_s *conn);

/****************************************************************************
 * Name: tcp_wrbuffer_initialize
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

#ifdef CONFIG_NET_TCP_WRITE_BUFFERS

struct tcp_wrbuffer_s;

/****************************************************************************
 * Name: tcp_wrbuffer_timedalloc
 *
 * Description:
 *   Allocate a TCP write buffer by taking a pre-allocated buffer from
 *   the free list.  This function is called from TCP logic when a buffer
 *   of TCP data is about to sent
 *   This function is wrapped version of tcp_wrbuffer_alloc(),
 *   this wait will be terminated when the specified timeout expires.
 *
 * Input Parameters:
 *   timeout   - The relative time to wait until a timeout is declared.
 *
 * Assumptions:
 *   Called from user logic with the network locked.
 *
 ****************************************************************************/

FAR struct tcp_wrbuffer_s *tcp_wrbuffer_timedalloc(unsigned int timeout);

/****************************************************************************
 * Name: tcp_wrbuffer_alloc
 *
 * Description:
 *   Allocate a TCP write buffer by taking a pre-allocated buffer from
 *   the free list.  This function is called from TCP logic when a buffer
 *   of TCP data is about to sent
 *
 * Input Parameters:
 *   None
 *
 * Assumptions:
 *   Called from user logic with the network locked.
 *
 ****************************************************************************/

FAR struct tcp_wrbuffer_s *tcp_wrbuffer_alloc(void);

/****************************************************************************
 * Name: tcp_wrbuffer_tryalloc
 *
 * Description:
 *   Try to allocate a TCP write buffer by taking a pre-allocated buffer from
 *   the free list.  This function is called from TCP logic when a buffer
 *   of TCP data is about to be sent if the socket is non-blocking. Returns
 *   immediately if allocation fails.
 *
 * Input parameters:
 *   None
 *
 * Assumptions:
 *   Called from user logic with the network locked.
 *
 ****************************************************************************/

FAR struct tcp_wrbuffer_s *tcp_wrbuffer_tryalloc(void);
#endif /* CONFIG_NET_TCP_WRITE_BUFFERS */

/****************************************************************************
 * Name: tcp_wrbuffer_release
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
 * Name: tcp_wrbuffer_inqueue_size
 *
 * Description:
 *   Get the in-queued write buffer size from connection
 *
 * Input Parameters:
 *   conn - The TCP connection of interest
 *
 * Assumptions:
 *   Called from user logic with the network locked.
 *
 ****************************************************************************/

#if CONFIG_NET_SEND_BUFSIZE > 0
uint32_t tcp_wrbuffer_inqueue_size(FAR struct tcp_conn_s *conn);
#endif

/****************************************************************************
 * Name: tcp_wrbuffer_test
 *
 * Description:
 *   Check if there is room in the write buffer.  Does not reserve any space.
 *
 * Assumptions:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
int tcp_wrbuffer_test(void);
#endif /* CONFIG_NET_TCP_WRITE_BUFFERS */

/****************************************************************************
 * Name: tcp_event_handler_dump
 *
 * Description:
 *  Dump the TCP event handler related variables
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
void tcp_event_handler_dump(FAR struct net_driver_s *dev,
                            FAR void *pvpriv,
                            uint16_t flags,
                            FAR struct tcp_conn_s *conn);
#endif

/****************************************************************************
 * Name: tcp_wrbuffer_dump
 *
 * Description:
 *   Dump the contents of a write buffer.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
#ifdef CONFIG_DEBUG_FEATURES
void tcp_wrbuffer_dump(FAR const char *msg, FAR struct tcp_wrbuffer_s *wrb,
                       unsigned int len, unsigned int offset);
#else
#  define tcp_wrbuffer_dump(msg,wrb)
#endif
#endif /* CONFIG_NET_TCP_WRITE_BUFFERS */

/****************************************************************************
 * Name: tcp_pollsetup
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

int tcp_pollsetup(FAR struct socket *psock, FAR struct pollfd *fds);

/****************************************************************************
 * Name: tcp_pollteardown
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

int tcp_pollteardown(FAR struct socket *psock, FAR struct pollfd *fds);

/****************************************************************************
 * Name: tcp_readahead_notifier_setup
 *
 * Description:
 *   Set up to perform a callback to the worker function when an TCP data
 *   is added to the read-ahead buffer.  The worker function will execute
 *   on the high priority worker thread.
 *
 * Input Parameters:
 *   worker - The worker function to execute on the high priority work
 *            queue when data is available in the TCP read-ahead buffer.
 *   conn   - The TCP connection where read-ahead data is needed.
 *   arg    - A user-defined argument that will be available to the worker
 *            function when it runs.
 *
 * Returned Value:
 *   > 0   - The signal notification is in place.  The returned value is a
 *           key that may be used later in a call to
 *           tcp_notifier_teardown().
 *   == 0  - There is already buffered read-ahead data.  No signal
 *           notification will be provided.
 *   < 0   - An unexpected error occurred and no signal will be sent.  The
 *           returned value is a negated errno value that indicates the
 *           nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP_NOTIFIER
int tcp_readahead_notifier_setup(worker_t worker,
                                 FAR struct tcp_conn_s *conn,
                                 FAR void *arg);
#endif

/****************************************************************************
 * Name: tcp_writebuffer_notifier_setup
 *
 * Description:
 *   Set up to perform a callback to the worker function when an TCP write
 *   buffer is emptied.  The worker function will execute on the high
 *   priority worker thread.
 *
 * Input Parameters:
 *   worker - The worker function to execute on the high priority work
 *            queue when all buffer TX data has been sent.
 *   conn   - The TCP connection where buffer write data is pending.
 *   arg    - A user-defined argument that will be available to the worker
 *            function when it runs.
 *
 * Returned Value:
 *   > 0   - The signal notification is in place.  The returned value is a
 *           key that may be used later in a call to
 *           tcp_notifier_teardown().
 *   == 0  - There is already buffered read-ahead data.  No signal
 *           notification will be provided.
 *   < 0   - An unexpected error occurred and no signal will be sent.  The
 *           returned value is a negated errno value that indicates the
 *           nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP_NOTIFIER
int tcp_writebuffer_notifier_setup(worker_t worker,
                                   FAR struct tcp_conn_s *conn,
                                   FAR void *arg);
#endif

/****************************************************************************
 * Name: tcp_disconnect_notifier_setup
 *
 * Description:
 *   Set up to perform a callback to the worker function if the TCP
 *   connection is lost.
 *
 * Input Parameters:
 *   worker - The worker function to execute on the high priority work
 *            queue when data is available in the TCP read-ahead buffer.
 *   conn  - The TCP connection where read-ahead data is needed.
 *   arg    - A user-defined argument that will be available to the worker
 *            function when it runs.
 *
 * Returned Value:
 *   > 0   - The signal notification is in place.  The returned value is a
 *           key that may be used later in a call to
 *           tcp_notifier_teardown().
 *   == 0  - No connection has been established.
 *   < 0   - An unexpected error occurred and no signal will be sent.  The
 *           returned value is a negated errno value that indicates the
 *           nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP_NOTIFIER
int tcp_disconnect_notifier_setup(worker_t worker,
                                  FAR struct tcp_conn_s *conn,
                                  FAR void *arg);
#endif

/****************************************************************************
 * Name: tcp_notifier_teardown
 *
 * Description:
 *   Eliminate a TCP read-ahead notification previously setup by
 *   tcp_readahead_notifier_setup().  This function should only be called
 *   if the notification should be aborted prior to the notification.  The
 *   notification will automatically be torn down after the signal is sent.
 *
 * Input Parameters:
 *   key - The key value returned from a previous call to
 *         tcp_readahead_notifier_setup().
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP_NOTIFIER
void tcp_notifier_teardown(int key);
#endif

/****************************************************************************
 * Name: tcp_readahead_signal
 *
 * Description:
 *   Read-ahead data has been buffered.  Signal all threads waiting for
 *   read-ahead data to become available.
 *
 *   When read-ahead data becomes available, *all* of the workers waiting
 *   for read-ahead data will be executed.  If there are multiple workers
 *   waiting for read-ahead data then only the first to execute will get the
 *   data.  Others will need to call tcp_readahead_notifier_setup() once
 *   again.
 *
 * Input Parameters:
 *   conn  - The TCP connection where read-ahead data was just buffered.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP_NOTIFIER
void tcp_readahead_signal(FAR struct tcp_conn_s *conn);
#endif

/****************************************************************************
 * Name: tcp_writebuffer_signal
 *
 * Description:
 *   All buffer Tx data has been sent.  Signal all threads waiting for the
 *   write buffers to become empty.
 *
 *   When write buffer becomes empty, *all* of the workers waiting
 *   for that event data will be executed.  If there are multiple workers
 *   waiting for read-ahead data then only the first to execute will get the
 *   data.  Others will need to call tcp_writebuffer_notifier_setup() once
 *   again.
 *
 * Input Parameters:
 *   conn  - The TCP connection where read-ahead data was just buffered.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_TCP_WRITE_BUFFERS) && defined(CONFIG_NET_TCP_NOTIFIER)
void tcp_writebuffer_signal(FAR struct tcp_conn_s *conn);
#endif

/****************************************************************************
 * Name: tcp_disconnect_signal
 *
 * Description:
 *   The TCP connection has been lost.  Signal all threads monitoring TCP
 *   state events.
 *
 * Input Parameters:
 *   conn  - The TCP connection where read-ahead data was just buffered.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP_NOTIFIER
void tcp_disconnect_signal(FAR struct tcp_conn_s *conn);
#endif

/****************************************************************************
 * Name: tcp_txdrain
 *
 * Description:
 *   Wait for all write buffers to be sent (or for a timeout to occur).
 *
 * Input Parameters:
 *   psock   - An instance of the internal socket structure.
 *   timeout - The relative time when the timeout will occur
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_TCP_WRITE_BUFFERS) && defined(CONFIG_NET_TCP_NOTIFIER)
int tcp_txdrain(FAR struct socket *psock, unsigned int timeout);
#else
#  define tcp_txdrain(conn, timeout) (0)
#endif

/****************************************************************************
 * Name: tcp_ioctl
 *
 * Description:
 *   This function performs tcp specific ioctl() operations.
 *
 * Parameters:
 *   conn     The TCP connection of interest
 *   cmd      The ioctl command
 *   arg      The argument of the ioctl cmd
 *
 ****************************************************************************/

int tcp_ioctl(FAR struct tcp_conn_s *conn, int cmd, unsigned long arg);

/****************************************************************************
 * Name: tcp_sendbuffer_notify
 *
 * Description:
 *   Notify the send buffer semaphore
 *
 * Input Parameters:
 *   conn - The TCP connection of interest
 *
 * Assumptions:
 *   Called from user logic with the network locked.
 *
 ****************************************************************************/

#if CONFIG_NET_SEND_BUFSIZE > 0
void tcp_sendbuffer_notify(FAR struct tcp_conn_s *conn);
#endif /* CONFIG_NET_SEND_BUFSIZE */

/****************************************************************************
 * Name: tcpip_hdrsize
 *
 * Description:
 *   Get the total size of L3 and L4 TCP header
 *
 * Input Parameters:
 *   conn     The connection structure associated with the socket
 *
 * Returned Value:
 *   the total size of L3 and L4 TCP header
 *
 ****************************************************************************/

uint16_t tcpip_hdrsize(FAR struct tcp_conn_s *conn);

/****************************************************************************
 * Name: tcp_ofoseg_bufsize
 *
 * Description:
 *   Calculate the pending size of out-of-order buffer
 *
 * Input Parameters:
 *   conn   - The TCP connection of interest
 *
 * Returned Value:
 *   Total size of out-of-order buffer
 *
 * Assumptions:
 *   This function must be called with the network locked.
 *
 ****************************************************************************/

int tcp_ofoseg_bufsize(FAR struct tcp_conn_s *conn);

/****************************************************************************
 * Name: tcp_reorder_ofosegs
 *
 * Description:
 *   Sort out-of-order segments by left edge
 *
 * Input Parameters:
 *   nofosegs - Number of out-of-order semgnets
 *   ofosegs  - Pointer to out-of-order segments
 *
 * Returned Value:
 *   True if re-order occurs
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

bool tcp_reorder_ofosegs(int nofosegs, FAR struct tcp_ofoseg_s *ofosegs);

#ifdef __cplusplus
}
#endif

#endif /* !CONFIG_NET_TCP_NO_STACK */
#endif /* CONFIG_NET_TCP */
#endif /* __NET_TCP_TCP_H */
