/****************************************************************************
 * net/usrsock/usrsock.h
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

#ifndef __NET_USRSOCK_USRSOCK_H
#define __NET_USRSOCK_USRSOCK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_NET_USRSOCK

#include <sys/types.h>
#include <sys/uio.h>

#include <nuttx/semaphore.h>

#include "devif/devif.h"
#include "socket/socket.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Internal socket type/domain for marking usrsock sockets */

#define SOCK_USRSOCK_TYPE   0x7f
#define PF_USRSOCK_DOMAIN   0x7f

/* Internal event flags */

#define USRSOCK_EVENT_CONNECT_READY (1 << 0)
#define USRSOCK_EVENT_REQ_COMPLETE  (1 << 15)
#define USRSOCK_EVENT_INTERNAL_MASK (USRSOCK_EVENT_CONNECT_READY | \
                                     USRSOCK_EVENT_REQ_COMPLETE)

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

enum usrsock_conn_state_e
{
  USRSOCK_CONN_STATE_UNINITIALIZED = 0,
  USRSOCK_CONN_STATE_ABORTED,
  USRSOCK_CONN_STATE_READY,
  USRSOCK_CONN_STATE_CONNECTING
};

struct usrsock_poll_s
{
  FAR struct usrsock_conn_s *conn; /* Needed to handle loss of connection */
  struct pollfd *fds;              /* Needed to handle poll events */
  FAR struct devif_callback_s *cb; /* Needed to teardown the poll */
};

struct usrsock_conn_s
{
  /* Common prologue of all connection structures. */

  struct socket_conn_s sconn;

  /* usrsock-specific content follows */

  uint8_t    crefs;                  /* Reference counts on this instance */

  enum usrsock_conn_state_e state;   /* State of kernel<->daemon link for conn */
  bool          connected;           /* Socket has been connected */
  int8_t        type;                /* Socket type (SOCK_STREAM, etc) */
  int16_t       usockid;             /* Connection number used for kernel<->daemon */
  uint16_t      flags;               /* Socket state flags */

  struct
  {
    sem_t    sem;               /* Request semaphore (only one outstanding request) */
    uint32_t xid;               /* Expected message exchange id */
    bool     inprogress;        /* Request was received but daemon is still processing */
    uint16_t valuelen;          /* Length of value from daemon */
    uint16_t valuelen_nontrunc; /* Actual length of value at daemon */
    int      result;            /* Result for request */
    uint16_t events;            /* Response events for the request */

    struct
    {
      FAR struct iovec *iov; /* Data request input buffers */
      int    iovcnt;         /* Number of input buffers */
      size_t total;          /* Total length of buffers */
      size_t pos;            /* Writer position on input buffer */
    } datain;
  } resp;

  /* The following is a list of poll structures of threads waiting for
   * socket events.
   */

  struct usrsock_poll_s pollinfo[CONFIG_NET_USRSOCK_NPOLLWAITERS];
};

struct usrsock_reqstate_s
{
  FAR struct usrsock_conn_s *conn;   /* Reference to connection structure */
  FAR struct devif_callback_s *cb;   /* Reference to callback instance */
  sem_t                   recvsem;   /* Semaphore signals recv completion */
  int                     result;    /* OK on success, otherwise a negated errno. */
  bool                    completed;
  bool                    unlock;    /* True: unlock is required */
};

struct usrsock_data_reqstate_s
{
  struct usrsock_reqstate_s reqstate;
  uint16_t                  valuelen;
  uint16_t                  valuelen_nontrunc;
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

EXTERN const struct sock_intf_s g_usrsock_sockif;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: usrsock_initialize()
 *
 * Description:
 *   Initialize the User Socket connection structures.  Called once and only
 *   from the networking layer.
 *
 ****************************************************************************/

void usrsock_initialize(void);

/****************************************************************************
 * Name: usrsock_alloc()
 *
 * Description:
 *   Allocate a new, uninitialized usrsock connection structure.  This is
 *   normally something done by the implementation of the socket() API
 *
 ****************************************************************************/

FAR struct usrsock_conn_s *usrsock_alloc(void);

/****************************************************************************
 * Name: usrsock_free()
 *
 * Description:
 *   Free a usrsock connection structure that is no longer in use. This
 *   should be done by the implementation of close().
 *
 ****************************************************************************/

void usrsock_free(FAR struct usrsock_conn_s *conn);

/****************************************************************************
 * Name: usrsock_nextconn()
 *
 * Description:
 *   Traverse the list of allocated usrsock connections
 *
 * Assumptions:
 *   This function is called from usrsock device logic.
 *
 ****************************************************************************/

FAR struct usrsock_conn_s *usrsock_nextconn(FAR struct usrsock_conn_s *conn);

/****************************************************************************
 * Name: usrsock_active()
 *
 * Description:
 *   Find a connection structure that is the appropriate
 *   connection for usrsock
 *
 ****************************************************************************/

FAR struct usrsock_conn_s *usrsock_active(int16_t usockid);

/****************************************************************************
 * Name: usrsock_setup_request_callback()
 ****************************************************************************/

int usrsock_setup_request_callback(FAR struct usrsock_conn_s *conn,
                                   FAR struct usrsock_reqstate_s *pstate,
                                   FAR devif_callback_event_t event,
                                   uint16_t flags);

/****************************************************************************
 * Name: usrsock_setup_data_request_callback()
 ****************************************************************************/

int usrsock_setup_data_request_callback(FAR struct usrsock_conn_s *conn,
                                FAR struct usrsock_data_reqstate_s *pstate,
                                FAR devif_callback_event_t event,
                                uint16_t flags);

/****************************************************************************
 * Name: usrsock_teardown_request_callback()
 ****************************************************************************/

void usrsock_teardown_request_callback(
                                FAR struct usrsock_reqstate_s *pstate);

/****************************************************************************
 * Name: usrsock_teardown_data_request_callback()
 ****************************************************************************/

#define usrsock_teardown_data_request_callback(datastate) \
  usrsock_teardown_request_callback(&(datastate)->reqstate)

/****************************************************************************
 * Name: usrsock_setup_datain
 ****************************************************************************/

void usrsock_setup_datain(FAR struct usrsock_conn_s *conn,
                          FAR struct iovec *iov, unsigned int iovcnt);

/****************************************************************************
 * Name: usrsock_teardown_datain
 ****************************************************************************/

#define usrsock_teardown_datain(conn) usrsock_setup_datain(conn, NULL, 0)

/****************************************************************************
 * Name: usrsock_event
 *
 * Description:
 *   Handler for received connection events
 *
 ****************************************************************************/

int usrsock_event(FAR struct usrsock_conn_s *conn);

/****************************************************************************
 * Name: usrsock_do_request
 *
 * Description:
 *   The usrsock_do_request() function will send usrsock request message
 *   to the usrsock network interface driver
 *
 ****************************************************************************/

int usrsock_do_request(FAR struct usrsock_conn_s *conn,
                       FAR struct iovec *iov, unsigned int iovcnt);

/****************************************************************************
 * Name: usrsock_socket
 *
 * Description:
 *   socket() creates an endpoint for communication and returns a socket
 *   structure.
 *
 * Input Parameters:
 *   domain   (see sys/socket.h)
 *   type     (see sys/socket.h)
 *   protocol (see sys/socket.h)
 *   psock    A pointer to a user allocated socket structure to be
 *            initialized.
 *
 * Returned Value:
 *   0 on success; negative error-code on error
 *
 *   EACCES
 *     Permission to create a socket of the specified type and/or protocol
 *     is denied.
 *   EAFNOSUPPORT
 *     The implementation does not support the specified address family.
 *   EINVAL
 *     Unknown protocol, or protocol family not available.
 *   EMFILE
 *     Process file table overflow.
 *   ENFILE
 *     The system limit on the total number of open files has been reached.
 *   ENOBUFS or ENOMEM
 *     Insufficient memory is available. The socket cannot be created until
 *     sufficient resources are freed.
 *   EPROTONOSUPPORT
 *     The protocol type or the specified protocol is not supported within
 *     this domain.
 *
 ****************************************************************************/

int usrsock_socket(int domain, int type, int protocol,
                   FAR struct socket *psock);

/****************************************************************************
 * Name: usrsock_close
 *
 * Description:
 *   Performs the close operation on a usrsock connection instance
 *
 * Input Parameters:
 *   conn   usrsock connection instance
 *
 * Returned Value:
 *   0 on success; -1 on error with errno set appropriately.
 *
 ****************************************************************************/

int usrsock_close(FAR struct usrsock_conn_s *conn);

/****************************************************************************
 * Name: usrsock_bind
 *
 * Description:
 *   usrsock_bind() gives the socket 'psock' the local address 'addr'. 'addr'
 *   is 'addrlen' bytes long. Traditionally, this is called "assigning a name
 *   to a socket." When a socket is created with socket, it exists in a name
 *   space (address family) but has no name assigned.
 *
 * Input Parameters:
 *   psock    A reference to the socket structure of the socket to be bound
 *   addr     Socket local address
 *   addrlen  Length of 'addr'
 *
 * Returned Value:
 *   0 on success; -1 on error with errno set appropriately
 *
 *   EACCES
 *     The address is protected, and the user is not the superuser.
 *   EADDRINUSE
 *     The given address is already in use.
 *   EINVAL
 *     The socket is already bound to an address.
 *   ENOTSOCK
 *     psock is a descriptor for a file, not a socket.
 *
 ****************************************************************************/

int usrsock_bind(FAR struct socket *psock,
                 FAR const struct sockaddr *addr,
                 socklen_t addrlen);

/****************************************************************************
 * Name: usrsock_connect
 *
 * Description:
 *   Perform a usrsock connection
 *
 * Input Parameters:
 *   psock   A reference to the socket structure of the socket
 *   addr    The address of the remote server to connect to
 *   addrlen Length of address buffer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

int usrsock_connect(FAR struct socket *psock,
                    FAR const struct sockaddr *addr, socklen_t addrlen);

/****************************************************************************
 * Name: usrsock_listen
 *
 * Description:
 *   To accept connections, a socket is first created with psock_socket(), a
 *   willingness to accept incoming connections and a queue limit for
 *   incoming connections are specified with psock_listen(), and then the
 *   connections are accepted with psock_accept().  For the case of AFINET
 *   and AFINET6 sockets, psock_listen() calls this function.  The
 *   psock_listen() call applies only to sockets of type SOCK_STREAM or
 *   SOCK_SEQPACKET.
 *
 * Parameters:
 *   psock    Reference to an internal, bound socket structure.
 *   backlog  The maximum length the queue of pending connections may grow.
 *            If a connection request arrives with the queue full, the client
 *            may receive an error with an indication of ECONNREFUSED or,
 *            if the underlying protocol supports retransmission, the request
 *            may be ignored so that retries succeed.
 *
 * Returned Value:
 *   On success, zero is returned. On error, a negated errno value is
 *   returned.  See listen() for a description of the appropriate error
 *   value.
 *
 ****************************************************************************/

int usrsock_listen(FAR struct socket *psock, int backlog);

/****************************************************************************
 * Name: usrsock_accept
 *
 * Description:
 *   The usrsock_accept function is used with connection-based socket
 *   types (SOCK_STREAM, SOCK_SEQPACKET and SOCK_RDM). It extracts the first
 *   connection request on the queue of pending connections, creates a new
 *   connected socket with mostly the same properties as 'psock', and
 *   allocates a new socket descriptor for the socket, which is returned. The
 *   newly created socket is no longer in the listening state. The original
 *   socket 'psock' is unaffected by this call.  Per file descriptor flags
 *   are not inherited across an usrsock_accept.
 *
 *   The 'psock' argument is a socket descriptor that has been created with
 *   socket(), bound to a local address with bind(), and is listening for
 *   connections after a call to listen().
 *
 *   On return, the 'addr' structure is filled in with the address of the
 *   connecting entity. The 'addrlen' argument initially contains the size
 *   of the structure pointed to by 'addr'; on return it will contain the
 *   actual length of the address returned.
 *
 *   If no pending connections are present on the queue, and the socket is
 *   not marked as non-blocking, usrsock_accept blocks the caller until a
 *   connection is present. If the socket is marked non-blocking and no
 *   pending connections are present on the queue, usrsock_accept returns
 *   EAGAIN.
 *
 * Parameters:
 *   psock    Reference to the listening socket structure
 *   addr     Receives the address of the connecting client
 *   addrlen  Input: allocated size of 'addr'
 *            Return: returned size of 'addr'
 *   newsock  Location to return the accepted socket information.
 *
 * Returned Value:
 *   Returns 0 (OK) on success.  On failure, it returns a negated errno
 *   value.  See accept() for a description of the appropriate error value.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

int usrsock_accept(FAR struct socket *psock, FAR struct sockaddr *addr,
                   FAR socklen_t *addrlen, FAR struct socket *newsock);

/****************************************************************************
 * Name: usrsock_poll
 *
 * Description:
 *   The standard poll() operation redirects operations on socket descriptors
 *   to this function.
 *
 * Input Parameters:
 *   psock - An instance of the internal socket structure.
 *   fds   - The structure describing the events to be monitored.
 *   setup - true: Setup up the poll; false: Teardown the poll
 *
 * Returned Value:
 *  0: Success; Negated errno on failure
 *
 ****************************************************************************/

int usrsock_poll(FAR struct socket *psock, FAR struct pollfd *fds,
                 bool setup);

/****************************************************************************
 * Name: usrsock_sendmsg
 *
 * Description:
 *   If sendmsg() is used on a connection-mode (SOCK_STREAM, SOCK_SEQPACKET)
 *   socket, the parameters 'msg_name' and 'msg_namelen' are ignored (and the
 *   error EISCONN may be returned when they are not NULL and 0), and the
 *   error ENOTCONN is returned when the socket was not actually connected.
 *
 * Input Parameters:
 *   psock    A reference to the socket structure of the socket
 *   msg      Message to send
 *   flags    Send flags (ignored)
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On any failure, a
 *   negated errno value is returned.
 *
 ****************************************************************************/

ssize_t usrsock_sendmsg(FAR struct socket *psock, FAR struct msghdr *msg,
                        int flags);

/****************************************************************************
 * Name: usrsock_recvmsg
 *
 * Description:
 *   recvmsg() receives messages from a socket, and may be used to receive
 *   data on a socket whether or not it is connection-oriented.
 *
 *   If from is not NULL, and the underlying protocol provides the source
 *   address, this source address is filled in. The argument fromlen
 *   initialized to the size of the buffer associated with from, and modified
 *   on return to indicate the actual size of the address stored there.
 *
 * Input Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   msg      Buffer to receive the message
 *   flags    Receive flags (ignored)
 *
 * Returned Value:
 *   On success, returns the number of characters received.  If no data is
 *   available to be received and the peer has performed an orderly shutdown,
 *   recvfrom() will return 0.  Otherwise, on any failure, a negated errno
 *   value is returned.
 *
 ****************************************************************************/

ssize_t usrsock_recvmsg(FAR struct socket *psock, FAR struct msghdr *msg,
                        int flags);

/****************************************************************************
 * Name: usrsock_getsockopt
 *
 * Description:
 *   getsockopt() retrieve the value for the option specified by the
 *   'option' argument at the protocol level specified by the 'level'
 *   argument. If the size of the option value is greater than 'value_len',
 *   the value stored in the object pointed to by the 'value' argument will
 *   be silently truncated. Otherwise, the length pointed to by the
 *   'value_len' argument will be modified to indicate the actual length
 *   of the 'value'.
 *
 *   The 'level' argument specifies the protocol level of the option. To
 *   retrieve options at the socket level, specify the level argument as
 *   SOL_SOCKET.
 *
 *   See <sys/socket.h> a complete list of values for the 'option' argument.
 *
 * Input Parameters:
 *   conn      usrsock socket connection structure
 *   level     Protocol level to set the option
 *   option    identifies the option to get
 *   value     Points to the argument value
 *   value_len The length of the argument value
 *
 ****************************************************************************/

int usrsock_getsockopt(FAR struct usrsock_conn_s *conn, int level,
                       int option, FAR void *value,
                       FAR socklen_t *value_len);

/****************************************************************************
 * Name: usrsock_setsockopt
 *
 * Description:
 *   psock_setsockopt() sets the option specified by the 'option' argument,
 *   at the protocol level specified by the 'level' argument, to the value
 *   pointed to by the 'value' argument for the usrsock connection.
 *
 *   The 'level' argument specifies the protocol level of the option. To set
 *   options at the socket level, specify the level argument as SOL_SOCKET.
 *
 *   See <sys/socket.h> a complete list of values for the 'option' argument.
 *
 * Input Parameters:
 *   conn      usrsock socket connection structure
 *   level     Protocol level to set the option
 *   option    identifies the option to set
 *   value     Points to the argument value
 *   value_len The length of the argument value
 *
 ****************************************************************************/

int usrsock_setsockopt(FAR struct usrsock_conn_s *conn, int level,
                       int option, FAR const void *value,
                       FAR socklen_t value_len);

/****************************************************************************
 * Name: usrsock_getsockname
 *
 * Description:
 *   The getsockname() function retrieves the locally-bound name of the
 *   specified socket, stores this address in the sockaddr structure pointed
 *   to by the 'addr' argument, and stores the length of this address in the
 *   object pointed to by the 'addrlen' argument.
 *
 *   If the actual length of the address is greater than the length of the
 *   supplied sockaddr structure, the stored address will be truncated.
 *
 *   If the socket has not been bound to a local name, the value stored in
 *   the object pointed to by address is unspecified.
 *
 * Input Parameters:
 *   psock    A reference to the socket structure of the socket
 *   addr     sockaddr structure to receive data [out]
 *   addrlen  Length of sockaddr structure [in/out]
 *
 ****************************************************************************/

int usrsock_getsockname(FAR struct socket *psock,
                        FAR struct sockaddr *addr, FAR socklen_t *addrlen);

/****************************************************************************
 * Name: usrsock_getpeername
 *
 * Description:
 *   The getpeername() function retrieves the remote-connected name of the
 *   specified socket, stores this address in the sockaddr structure pointed
 *   to by the 'addr' argument, and stores the length of this address in the
 *   object pointed to by the 'addrlen' argument.
 *
 *   If the actual length of the address is greater than the length of the
 *   supplied sockaddr structure, the stored address will be truncated.
 *
 *   If the socket has not been bound to a local name, the value stored in
 *   the object pointed to by address is unspecified.
 *
 * Input Parameters:
 *   psock    A reference to the socket structure of the socket
 *   addr     sockaddr structure to receive data [out]
 *   addrlen  Length of sockaddr structure [in/out]
 *
 ****************************************************************************/

int usrsock_getpeername(FAR struct socket *psock,
                        FAR struct sockaddr *addr, FAR socklen_t *addrlen);

/****************************************************************************
 * Name: usrsock_ioctl
 *
 * Description:
 *   The usrsock_ioctl() function performs network device specific
 *   operations.
 *
 * Parameters:
 *   psock    A reference to the socket structure of the socket
 *   cmd      The ioctl command
 *   arg      The argument of the ioctl cmd
 *
 ****************************************************************************/

int usrsock_ioctl(FAR struct socket *psock, int cmd, unsigned long arg);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET_USRSOCK */
#endif /* __NET_USRSOCK_USRSOCK_H */
