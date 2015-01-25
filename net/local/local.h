/****************************************************************************
 * net/local/loal.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#ifndef __NET_LOCAL_LOCAL_H
#define __NET_LOCAL_LOCAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/un.h>
#include <queue.h>
#include <stdint.h>

#ifdef CONFIG_NET_LOCAL

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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
  LOCAL_STATE_BOUND,           /* Bound to an pipe */

  /* SOCK_STREAM only */

  LOCAL_STATE_LISTENING,       /* Server listening for connections */
  LOCAL_STATE_CLOSED,          /* Server closed, no longer connected */
  LOCAL_STATE_ACCEPT,          /* Client waiting for a connection */
  LOCAL_STATE_CONNECTED,       /* Client connected */
  LOCAL_STATE_DISCONNECTED     /* Client disconnected */
};

/* Representation of a local connection */

struct local_conn_s
{
  /* Fields common to SOCK_STREAM and SOCK_DGRAM */

  dq_entry_t lc_node;          /* Supports a doubly linked list */
  uint8_t lc_crefs;            /* Reference counts on this instance */
  uint8_t lc_family;           /* SOCK_STREAM or SOCK_DGRAM */
  uint8_t lc_type;             /* See enum local_type_e */
  uint8_t lc_state;            /* See enum local_state_e */
  int16_t lc_fd;               /* File descriptor of underlying pipe */
  char lc_path[UNIX_PATH_MAX]; /* Path assigned by bind() */

  /* SOCK_STREAM fields common to both client and server */

  sem_t lc_waitsem;            /* Use to wait for a connection to be accepted */

  /* Union of fields unique to SOCK_STREAM client and servers  */

  union
  {
    /* Fields unique to the SOCK_STREAM server side */

    struct
    {
      uint8_t lc_pending;      /* Number of pending connections */
      uint8_t lc_backlog;      /* Maximum number of pending connections */
      dq_queue_t lc_waiters;   /* List of connections waiting to be accepted */
      dq_queue_t lc_conns;     /* List of connections */
    } server;

    /* Fields unique to the client side */

    struct
    {
      FAR struct local_conn_s *lc_server; /* Server connection */
      volatile int lc_result;  /* Result of the connection operation */
    } client;
  } u;
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

struct sockaddr; /* Forward reference */
struct socket;   /* Forward reference */

/****************************************************************************
 * Name: local_initialize
 *
 * Description:
 *   Initialize the local, Unix domain connection structures.  Called once
 *   and only from the common network initialization logic.
 *
 ****************************************************************************/

void local_initialize(void);

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
 * Name: local_bind
 *
 * Description:
 *   This function implements the low-level parts of the standard local
 *   bind()operation.
 *
 ****************************************************************************/

int local_bind(FAR struct local_conn_s *conn,
               FAR const struct sockaddr *addr, socklen_t addrlen);

/****************************************************************************
 * Name: local_connect
 *
 * Description:
 *   This function sets up a new local connection. The function will
 *   automatically allocate an unused local port for the new
 *   connection. However, another port can be chosen by using the
 *   local_bind() call, after the local_connect() function has been
 *   called.
 *
 *   This function is called as part of the implementation of sendto
 *   and recvfrom.
 *
 * Input Parameters:
 *   client - A reference to the client-side local connection structure
 *   addr - The address of the remote host.
 *
 ****************************************************************************/

int local_connect(FAR struct local_conn_s *client,
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
 * Name: psock_local_send
 *
 * Description:
 *   Send a local packet as a stream.
 *
 * Parameters:
 *   psock    An instance of the internal socket structure.
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags
 *
 * Return:
 *   On success, returns the number of characters sent.  On  error,
 *   -1 is returned, and errno is set appropriately (see send() for the
 *   list of errno numbers).
 *
 ****************************************************************************/

ssize_t psock_local_send(FAR struct socket *psock, FAR const void *buf,
                         size_t len, int flags);

/****************************************************************************
 * Function: psock_sendto
 *
 * Description:
 *   Send a local packet as a datagram.
 *
 * Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags
 *   to       Address of recipient
 *   tolen    The length of the address structure
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   -1 is returned, and errno is set appropriately (see sendto() for the
 *   list of errno numbers).
 *
 ****************************************************************************/

ssize_t psock_local_sendto(FAR struct socket *psock, FAR const void *buf,
                           size_t len, int flags, FAR const struct sockaddr *to,
                           socklen_t tolen);

/****************************************************************************
 * Function: psock_recvfrom
 *
 * Description:
 *   recvfrom() receives messages from a local socket, and may be used to
 *   receive data on a socket whether or not it is connection-oriented.
 *
 *   If from is not NULL, and the underlying protocol provides the source
 *   address, this source address is filled in. The argument fromlen
 *   initialized to the size of the buffer associated with from, and modified
 *   on return to indicate the actual size of the address stored there.
 *
 * Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   buf      Buffer to receive data
 *   len      Length of buffer
 *   flags    Receive flags
 *   from     Address of source (may be NULL)
 *   fromlen  The length of the address structure
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  If no data is
 *   available to be received and the peer has performed an orderly shutdown,
 *   recv() will return 0.  Otherwise, on errors, -1 is returned, and errno
 *   is set appropriately (see receivefrom for the complete list).
 *
 ****************************************************************************/

ssize_t psock_local_recvfrom(FAR struct socket *psock, FAR void *buf,
                             size_t len, int flags, FAR struct sockaddr *from,
                             FAR socklen_t *fromlen);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET_LOCAL */
#endif /* __NET_LOCAL_LOCAL_H */
