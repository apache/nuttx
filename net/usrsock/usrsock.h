/****************************************************************************
 * net/usrsock/usrsock.h
 *
 *  Copyright (C) 2015, 2017 Haltian Ltd. All rights reserved.
 *  Author: Jussi Kivilinna <jussi.kivilinna@haltian.com>
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

#ifndef __NET_USRSOCK_USRSOCK_H
#define __NET_USRSOCK_USRSOCK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_NET_USRSOCK

#include <sys/types.h>
#include <queue.h>
#include <semaphore.h>

#include "devif/devif.h"
#include "socket/socket.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

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

struct usrsockdev_s;

enum usrsock_conn_state_e
{
  USRSOCK_CONN_STATE_UNINITIALIZED = 0,
  USRSOCK_CONN_STATE_ABORTED,
  USRSOCK_CONN_STATE_READY,
  USRSOCK_CONN_STATE_CONNECTING,
};

struct iovec
{
  FAR void *iov_base; /* Starting address */
  size_t iov_len;     /* Number of bytes to transfer */
};

struct usrsock_conn_s
{
  dq_entry_t node;                   /* Supports a doubly linked list */
  uint8_t    crefs;                  /* Reference counts on this instance */

  enum usrsock_conn_state_e state;   /* State of kernel<->daemon link for conn */
  bool          connected;           /* Socket has been connected */
  int8_t        type;                /* Socket type (SOCK_STREAM, etc) */
  int16_t       usockid;             /* Connection number used for kernel<->daemon */
  uint16_t      flags;               /* Socket state flags */
  struct usrsockdev_s *dev;          /* Device node used for this conn */

  struct
  {
    uint8_t  xid;         /* Expected message exchange id */
    bool     inprogress;  /* Request was received but daemon is still processing */
    uint16_t valuelen;    /* Length of value from daemon */
    uint16_t valuelen_nontrunc; /* Actual length of value at daemon */
    int      result;      /* Result for request */

    struct
    {
      FAR struct iovec *iov; /* Data request input buffers */
      int    iovcnt;         /* Number of input buffers */
      size_t total;          /* Total length of buffers */
      size_t pos;            /* Writer position on input buffer */
    } datain;
  } resp;

  /* Defines the list of usrsock callbacks */

  FAR struct devif_callback_s *list;
};

struct usrsock_reqstate_s
{
  FAR struct usrsock_conn_s *conn;   /* Reference to connection structure */
  FAR struct devif_callback_s *cb;   /* Reference to callback instance */
  sem_t                   recvsem;   /* Semaphore signals recv completion */
  int                     result;    /* OK on success, otherwise a negated errno. */
  bool                    completed;
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
 *   Free a usrsock connection structure that is no longer in use. This should
 *   be done by the implementation of close().
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
 * Name: usrsock_connidx()
 ****************************************************************************/

int usrsock_connidx(FAR struct usrsock_conn_s *conn);

/****************************************************************************
 * Name: usrsock_active()
 *
 * Description:
 *   Find a connection structure that is the appropriate
 *   connection for usrsock
 *
 * Assumptions:
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

void usrsock_teardown_request_callback(FAR struct usrsock_reqstate_s *pstate);

/****************************************************************************
 * Name: usrsock_teardown_data_request_callback()
 ****************************************************************************/

#define usrsock_teardown_data_request_callback(datastate) \
  usrsock_teardown_request_callback(&(datastate)->reqstate)

/****************************************************************************
 * Name: usrsock_event
 *
 * Description:
 *   Handler for received connection events
 *
 ****************************************************************************/

int usrsock_event(FAR struct usrsock_conn_s *conn, uint16_t events);

/****************************************************************************
 * Name: usrsockdev_do_request
 ****************************************************************************/

int usrsockdev_do_request(FAR struct usrsock_conn_s *conn,
                          FAR struct iovec *iov, unsigned int iovcnt);

/****************************************************************************
 * Name: usrsockdev_register
 *
 * Description:
 *   Register /dev/usrsock
 *
 ****************************************************************************/

void usrsockdev_register(void);

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
 *   psock    A pointer to a user allocated socket structure to be initialized.
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
 * Assumptions:
 *
 ****************************************************************************/

int usrsock_socket(int domain, int type, int protocol, FAR struct socket *psock);

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
 * Assumptions:
 *
 ****************************************************************************/

int usrsock_close(FAR struct usrsock_conn_s *conn);

/****************************************************************************
 * Name: usrsock_bind
 *
 * Description:
 *   usrsock_bind() gives the socket 'conn' the local address 'addr'. 'addr'
 *   is 'addrlen' bytes long. Traditionally, this is called "assigning a name
 *   to a socket." When a socket is created with socket, it exists in a name
 *   space (address family) but has no name assigned.
 *
 * Input Parameters:
 *   conn     usrsock socket connection structure
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
 * Assumptions:
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
 *   psock   A reference to the socket structure of the socket to be connected
 *   addr    The address of the remote server to connect to
 *   addrlen Length of address buffer
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

int usrsock_connect(FAR struct socket *psock,
                    FAR const struct sockaddr *addr, socklen_t addrlen);

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

#ifndef CONFIG_DISABLE_POLL
int usrsock_poll(FAR struct socket *psock, FAR struct pollfd *fds, bool setup);
#endif

/****************************************************************************
 * Name: usrsock_sendto
 *
 * Description:
 *   If sendto() is used on a connection-mode (SOCK_STREAM, SOCK_SEQPACKET)
 *   socket, the parameters to and 'tolen' are ignored (and the error EISCONN
 *   may be returned when they are not NULL and 0), and the error ENOTCONN is
 *   returned when the socket was not actually connected.
 *
 * Input Parameters:
 *   psock    A reference to the socket structure of the socket to be connected
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags (ignored)
 *   to       Address of recipient
 *   tolen    The length of the address structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

ssize_t usrsock_sendto(FAR struct socket *psock, FAR const void *buf,
                       size_t len, int flags, FAR const struct sockaddr *to,
                       socklen_t tolen);

/****************************************************************************
 * Name: usrsock_recvfrom
 *
 * Description:
 *   recvfrom() receives messages from a socket, and may be used to receive
 *   data on a socket whether or not it is connection-oriented.
 *
 *   If from is not NULL, and the underlying protocol provides the source
 *   address, this source address is filled in. The argument fromlen
 *   initialized to the size of the buffer associated with from, and modified
 *   on return to indicate the actual size of the address stored there.
 *
 * Input Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   buf      Buffer to receive data
 *   len      Length of buffer
 *   flags    Receive flags (ignored)
 *   from     Address of source (may be NULL)
 *   fromlen  The length of the address structure
 *
 ****************************************************************************/

ssize_t usrsock_recvfrom(FAR struct socket *psock, FAR void *buf, size_t len,
                         int flags, FAR struct sockaddr *from,
                         FAR socklen_t *fromlen);

/****************************************************************************
 * Name: usrsock_getsockopt
 *
 * Description:
 *   getsockopt() retrieve thse value for the option specified by the
 *   'option' argument for the socket specified by the 'psock' argument. If
 *   the size of the option value is greater than 'value_len', the value
 *   stored in the object pointed to by the 'value' argument will be silently
 *   truncated. Otherwise, the length pointed to by the 'value_len' argument
 *   will be modified to indicate the actual length of the'value'.
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

int usrsock_getsockopt(FAR struct usrsock_conn_s *conn, int level, int option,
                       FAR void *value, FAR socklen_t *value_len);

/****************************************************************************
 * Name: usrsock_setsockopt
 *
 * Description:
 *   psock_setsockopt() sets the option specified by the 'option' argument,
 *   at the protocol level specified by the 'level' argument, to the value
 *   pointed to by the 'value' argument for the socket on the 'psock' argument.
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

int usrsock_setsockopt(FAR struct usrsock_conn_s *conn, int level, int option,
                       FAR const void *value, FAR socklen_t value_len);

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
 *   conn     usrsock socket connection structure
 *   addr     sockaddr structure to receive data [out]
 *   addrlen  Length of sockaddr structure [in/out]
 *
 ****************************************************************************/

int usrsock_getsockname(FAR struct socket *psock,
                        FAR struct sockaddr *addr, FAR socklen_t *addrlen);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET_USRSOCK */
#endif /* __NET_USRSOCK_USRSOCK_H */
