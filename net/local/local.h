/****************************************************************************
 * net/local/local.h
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

#ifndef __NET_LOCAL_LOCAL_H
#define __NET_LOCAL_LOCAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/un.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdint.h>
#include <poll.h>

#include <nuttx/fs/fs.h>
#include <nuttx/queue.h>
#include <nuttx/net/net.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>

#ifdef CONFIG_NET_LOCAL

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LOCAL_NPOLLWAITERS 2
#define LOCAL_NCONTROLFDS  4

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Local, Unix domain socket types */

enum local_type_e
{
  LOCAL_TYPE_UNTYPED = 0,      /* Type is not determined until the socket is bound */
  LOCAL_TYPE_UNNAMED,          /* A Unix socket that is not bound to any name */
  LOCAL_TYPE_PATHNAME,         /* lc_path holds a null terminated string */
  LOCAL_TYPE_ABSTRACT          /* lc_path is length zero */
};

/* The state of a Unix socket */

enum local_state_s
{
  /* Common states */

  LOCAL_STATE_UNBOUND = 0,     /* Created by socket, but not bound */
  LOCAL_STATE_BOUND,           /* Bound to an path */

  /* SOCK_STREAM server only */

  LOCAL_STATE_LISTENING,       /* Server listening for connections */

  /* SOCK_STREAM peers only */

  LOCAL_STATE_ACCEPT,          /* Client waiting for a connection */
  LOCAL_STATE_CONNECTING,      /* Non-blocking connect */
  LOCAL_STATE_CONNECTED,       /* Peer connected */
  LOCAL_STATE_DISCONNECTED     /* Peer disconnected */
};

/* Representation of a local connection.  There are four types of
 * connection structures:
 *
 * 1. Server.  A SOCK_STREAM that only listens for and accepts
 *    connections from server.
 * 2. Client.  A SOCK_STREAM peer that connects via the server.
 * 3. Peer. A connected SOCK_STREAM that sends() and recvs() packets.
 *    May either be the client that connect with the server of the
 *    new peer connect generated with the connection was accepted by
 *    the server.
 *
 * And
 *
 * 4. Connectionless.  Like a peer but using a connectionless datagram
 *    style of communication.  SOCK_DRAM support has not yet been
 *    implemented.
 */

struct devif_callback_s;       /* Forward reference */

struct local_conn_s
{
  /* Common prologue of all connection structures. */

  struct socket_conn_s lc_conn;

  /* Local-socket specific content follows */

  /* Fields common to SOCK_STREAM and SOCK_DGRAM */

  uint8_t lc_crefs;              /* Reference counts on this instance */
  uint8_t lc_proto;              /* SOCK_STREAM or SOCK_DGRAM */
  uint8_t lc_type;               /* See enum local_type_e */
  uint8_t lc_state;              /* See enum local_state_e */
  struct file lc_infile;         /* File for read-only FIFO (peers) */
  struct file lc_outfile;        /* File descriptor of write-only FIFO (peers) */
  char lc_path[UNIX_PATH_MAX];   /* Path assigned by bind() */
  int32_t lc_instance_id;        /* Connection instance ID for stream
                                  * server<->client connection pair */
#ifdef CONFIG_NET_LOCAL_SCM
  FAR struct local_conn_s *
                        lc_peer; /* Peer connection instance */
  uint16_t lc_cfpcount;          /* Control file pointer counter */
  FAR struct file *
     lc_cfps[LOCAL_NCONTROLFDS]; /* Socket message control filep */
  struct ucred lc_cred;          /* The credentials of connection instance */
#endif /* CONFIG_NET_LOCAL_SCM */

  mutex_t lc_sendlock;           /* Make sending multi-thread safe */

#ifdef CONFIG_NET_LOCAL_STREAM
  /* SOCK_STREAM fields common to both client and server */

  sem_t lc_waitsem;            /* Use to wait for a connection to be accepted */
  sem_t lc_donesem;            /* Use to wait for client connected done */
  FAR struct socket *lc_psock; /* A reference to the socket structure */

  /* The following is a list if poll structures of threads waiting for
   * socket events.
   */

  struct pollfd *lc_event_fds[LOCAL_NPOLLWAITERS];
  struct pollfd lc_inout_fds[2*LOCAL_NPOLLWAITERS];

  /* Union of fields unique to SOCK_STREAM client, server, and connected
   * peers.
   */

  union
  {
    /* Fields unique to the SOCK_STREAM server side */

    struct
    {
      uint8_t lc_pending;      /* Number of pending connections */
      uint8_t lc_backlog;      /* Maximum number of pending connections */
      dq_queue_t lc_waiters;   /* List of connections waiting to be accepted */
    } server;

    /* Fields unique to the connecting client side */

    struct
    {
      volatile int lc_result;  /* Result of the connection operation (client) */
      dq_entry_t lc_waiter;    /* Linked to the lc_waiters lists */
    } client;
  } u;
#endif /* CONFIG_NET_LOCAL_STREAM */
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

/* The local socket interface */

EXTERN const struct sock_intf_s g_local_sockif;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct sockaddr; /* Forward reference */
struct socket;   /* Forward reference */

/****************************************************************************
 * Name: local_alloc
 *
 * Description:
 *   Allocate a new, uninitialized local connection structure.  This is
 *   normally something done by the implementation of the socket() API
 *
 ****************************************************************************/

FAR struct local_conn_s *local_alloc(void);

/****************************************************************************
 * Name: local_free
 *
 * Description:
 *   Free a local connection structure that is no longer in use. This should
 *   be done by the implementation of close().
 *
 ****************************************************************************/

void local_free(FAR struct local_conn_s *conn);

/****************************************************************************
 * Name: local_nextconn
 *
 * Description:
 *   Traverse the list of allocated Local connections
 *
 * Assumptions:
 *   Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

FAR struct local_conn_s *local_nextconn(FAR struct local_conn_s *conn);

/****************************************************************************
 * Name: local_peerconn
 *
 * Description:
 *   Traverse the connections list to find the peer
 *
 * Assumptions:
 *   This function must be called with the network locked.
 *
 ****************************************************************************/

FAR struct local_conn_s *local_peerconn(FAR struct local_conn_s *conn);

/****************************************************************************
 * Name: psock_local_bind
 *
 * Description:
 *   This function implements the low-level parts of the standard local
 *   bind()operation.
 *
 ****************************************************************************/

int psock_local_bind(FAR struct socket *psock,
                     FAR const struct sockaddr *addr, socklen_t addrlen);

/****************************************************************************
 * Name: psock_local_connect
 *
 * Description:
 *   This function sets up a new local connection. The function will
 *   automatically allocate an unused local port for the new
 *   connection. However, another port can be chosen by using the
 *   psock_local_bind() call, after the psock_local_connect() function has
 *   been called.
 *
 *   This function is called as part of the implementation of sendto
 *   and recvfrom.
 *
 * Input Parameters:
 *   psock - A reference to the client-side socket structure
 *   addr - The address of the remote host.
 *
 ****************************************************************************/

int psock_local_connect(FAR struct socket *psock,
                        FAR const struct sockaddr *addr);

/****************************************************************************
 * Name: local_release
 *
 * Description:
 *   If the local, Unix domain socket is in the connected state, then
 *   disconnect it.  Release the local connection structure in any event
 *
 * Input Parameters:
 *   conn - A reference to local connection structure
 *
 ****************************************************************************/

int local_release(FAR struct local_conn_s *conn);

/****************************************************************************
 * Name: local_listen
 *
 * Description:
 *   To accept connections, a socket is first created with psock_socket(), a
 *   willingness to accept incoming connections and a queue limit for
 *   incoming connections are specified with psock_listen(), and then the
 *   connections are accepted with psock_accept().  For the case of local
 *   Unix sockets, psock_listen() calls this function.  The psock_listen()
 *   call applies only to sockets of type SOCK_STREAM or SOCK_SEQPACKET.
 *
 * Input Parameters:
 *   psock    Reference to an internal, boound socket structure.
 *   backlog  The maximum length the queue of pending connections may grow.
 *            If a connection request arrives with the queue full, the client
 *            may receive an error with an indication of ECONNREFUSED or,
 *            if the underlying protocol supports retransmission, the request
 *            may be ignored so that retries succeed.
 *
 * Returned Value:
 *   On success, zero is returned. On error, a negated errno value is
 *   returned.  See listen() for the set of appropriate error values.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_LOCAL_STREAM
int local_listen(FAR struct socket *psock, int backlog);
#endif

/****************************************************************************
 * Name: local_accept
 *
 * Description:
 *   This function implements accept() for Unix domain sockets.  See the
 *   description of accept() for further information.
 *
 * Input Parameters:
 *   psock    The listening Unix domain socket structure
 *   addr     Receives the address of the connecting client
 *   addrlen  Input: allocated size of 'addr'
 *            Return: returned size of 'addr'
 *   newconn  The new, accepted  Unix domain connection structure
 *
 * Returned Value:
 *   Returns zero (OK) on success or a negated errno value on failure.
 *   See the description of accept of the possible errno values in the
 *   description of accept().
 *
 * Assumptions:
 *   Network is NOT locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_LOCAL_STREAM
int local_accept(FAR struct socket *psock, FAR struct sockaddr *addr,
                 FAR socklen_t *addrlen, FAR struct socket *newsock);
#endif

/****************************************************************************
 * Name: local_sendmsg
 *
 * Description:
 *   Implements the sendmsg() operation for the case of the local Unix socket
 *
 * Input Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   msg      msg to send
 *   flags    Send flags
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error, a negated
 *   errno value is returned (see sendmsg() for the list of appropriate error
 *   values.
 *
 ****************************************************************************/

ssize_t local_sendmsg(FAR struct socket *psock, FAR struct msghdr *msg,
                      int flags);

/****************************************************************************
 * Name: local_send_packet
 *
 * Description:
 *   Send a packet on the write-only FIFO.
 *
 * Input Parameters:
 *   filep    File structure of write-only FIFO.
 *   buf      Data to send
 *   len      Length of data to send
 *   preamble Flag to indicate the preamble sync header assembly
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

int local_send_packet(FAR struct file *filep, FAR const struct iovec *buf,
                      size_t len, bool preamble);

/****************************************************************************
 * Name: local_recvmsg
 *
 * Description:
 *   recvmsg() receives messages from a local socket and may be used to
 *   receive data on a socket whether or not it is connection-oriented.
 *
 *   If from is not NULL, and the underlying protocol provides the source
 *   address, this source address is filled in. The argument fromlen
 *   initialized to the size of the buffer associated with from, and modified
 *   on return to indicate the actual size of the address stored there.
 *
 * Input Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   msg      Buffer to receive the message
 *   flags    Receive flags (ignored for now)
 *
 * Returned Value:
 *   On success, returns the number of characters received. If no data is
 *   available to be received and the peer has performed an orderly shutdown,
 *   recvmsg() will return 0.  Otherwise, on errors, a negated errno value is
 *   returned (see recvmsg() for the list of appropriate error values).
 *
 ****************************************************************************/

ssize_t local_recvmsg(FAR struct socket *psock, FAR struct msghdr *msg,
                      int flags);

/****************************************************************************
 * Name: local_fifo_read
 *
 * Description:
 *   Read a data from the read-only FIFO.
 *
 * Input Parameters:
 *   filep - File structure of write-only FIFO.
 *   buf   - Local to store the received data
 *   len   - Length of data to receive [in]
 *           Length of data actually received [out]
 *   once  - Flag to indicate the buf may only be read once
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.  If -ECONNRESET is received, then the sending side has closed
 *   the FIFO. In this case, the returned data may still be valid (if the
 *   returned len > 0).
 *
 ****************************************************************************/

int local_fifo_read(FAR struct file *filep, FAR uint8_t *buf,
                    size_t *len, bool once);

/****************************************************************************
 * Name: local_getaddr
 *
 * Description:
 *   Return the Unix domain address of a connection.
 *
 * Input Parameters:
 *   conn - The connection
 *   addr - The location to return the address
 *   addrlen - The size of the memory allocate by the caller to receive the
 *             address.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int local_getaddr(FAR struct local_conn_s *conn, FAR struct sockaddr *addr,
                  FAR socklen_t *addrlen);

/****************************************************************************
 * Name: local_sync
 *
 * Description:
 *   Read a sync bytes until the start of the packet is found.
 *
 * Input Parameters:
 *   filep - File structure of write-only FIFO.
 *
 * Returned Value:
 *   The non-zero size of the following packet is returned on success; a
 *   negated errno value is returned on any failure.
 *
 ****************************************************************************/

int local_sync(FAR struct file *filep);

/****************************************************************************
 * Name: local_create_fifos
 *
 * Description:
 *   Create the FIFO pair needed for a SOCK_STREAM connection.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_LOCAL_STREAM
int local_create_fifos(FAR struct local_conn_s *conn);
#endif

/****************************************************************************
 * Name: local_create_halfduplex
 *
 * Description:
 *   Create the half-duplex FIFO needed for SOCK_DGRAM communication.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_LOCAL_DGRAM
int local_create_halfduplex(FAR struct local_conn_s *conn,
                            FAR const char *path);
#endif

/****************************************************************************
 * Name: local_release_fifos
 *
 * Description:
 *   Release references to the FIFO pair used for a SOCK_STREAM connection.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_LOCAL_STREAM
int local_release_fifos(FAR struct local_conn_s *conn);
#endif

/****************************************************************************
 * Name: local_release_halfduplex
 *
 * Description:
 *   Release a reference to the FIFO used for SOCK_DGRAM communication
 *
 ****************************************************************************/

#ifdef CONFIG_NET_LOCAL_DGRAM
int local_release_halfduplex(FAR struct local_conn_s *conn);
#endif

/****************************************************************************
 * Name: local_open_client_rx
 *
 * Description:
 *   Only the client-side Rx FIFO.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_LOCAL_STREAM
int local_open_client_rx(FAR struct local_conn_s *client, bool nonblock);
#endif

/****************************************************************************
 * Name: local_open_client_tx
 *
 * Description:
 *   Only the client-side Tx FIFO.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_LOCAL_STREAM
int local_open_client_tx(FAR struct local_conn_s *client, bool nonblock);
#endif

/****************************************************************************
 * Name: local_open_server_rx
 *
 * Description:
 *   Only the server-side Rx FIFO.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_LOCAL_STREAM
int local_open_server_rx(FAR struct local_conn_s *server, bool nonblock);
#endif

/****************************************************************************
 * Name: local_open_server_tx
 *
 * Description:
 *   Only the server-side Tx FIFO.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_LOCAL_STREAM
int local_open_server_tx(FAR struct local_conn_s *server, bool nonblock);
#endif

/****************************************************************************
 * Name: local_open_receiver
 *
 * Description:
 *   Only the receiving side of the half duplex FIFO.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_LOCAL_DGRAM
int local_open_receiver(FAR struct local_conn_s *conn, bool nonblock);
#endif

/****************************************************************************
 * Name: local_open_sender
 *
 * Description:
 *   Only the sending side of the half duplex FIFO.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_LOCAL_DGRAM
int local_open_sender(FAR struct local_conn_s *conn, FAR const char *path,
                      bool nonblock);
#endif

/****************************************************************************
 * Name: local_event_pollnotify
 ****************************************************************************/

void local_event_pollnotify(FAR struct local_conn_s *conn,
                            pollevent_t eventset);

/****************************************************************************
 * Name: local_pollsetup
 *
 * Description:
 *   Setup to monitor events on one Unix domain socket
 *
 * Input Parameters:
 *   psock - The Unix domain socket of interest
 *   fds   - The structure describing the events to be monitored, OR NULL if
 *           this is a request to stop monitoring events.
 *
 * Returned Value:
 *  0: Success; Negated errno on failure
 *
 ****************************************************************************/

int local_pollsetup(FAR struct socket *psock, FAR struct pollfd *fds);

/****************************************************************************
 * Name: local_pollteardown
 *
 * Description:
 *   Teardown monitoring of events on a Unix domain socket
 *
 * Input Parameters:
 *   psock - The Unix domain socket of interest
 *   fds   - The structure describing the events to be monitored, OR NULL if
 *           this is a request to stop monitoring events.
 *
 * Returned Value:
 *  0: Success; Negated errno on failure
 *
 ****************************************************************************/

int local_pollteardown(FAR struct socket *psock, FAR struct pollfd *fds);

/****************************************************************************
 * Name: local_generate_instance_id
 *
 * Description:
 *   Generate instance ID for stream
 *
 ****************************************************************************/

int32_t local_generate_instance_id(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET_LOCAL */
#endif /* __NET_LOCAL_LOCAL_H */
