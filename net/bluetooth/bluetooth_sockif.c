/****************************************************************************
 * net/bluetooth/bluetooth_sockif.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <socket/socket.h>
#include <netpacket/bluetooth.h>

#include <nuttx/net/net.h>
#include <nuttx/net/radiodev.h>
#include <nuttx/net/bluetooth.h>
#include <nuttx/wireless/bluetooth/bt_hci.h>

#include "bluetooth/bluetooth.h"

#ifdef CONFIG_NET_BLUETOOTH

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int        bluetooth_setup(FAR struct socket *psock);
static sockcaps_t bluetooth_sockcaps(FAR struct socket *psock);
static void       bluetooth_addref(FAR struct socket *psock);
static int        bluetooth_bind(FAR struct socket *psock,
                    FAR const struct sockaddr *addr, socklen_t addrlen);
static int        bluetooth_getsockname(FAR struct socket *psock,
                    FAR struct sockaddr *addr, FAR socklen_t *addrlen);
static int        bluetooth_getpeername(FAR struct socket *psock,
                    FAR struct sockaddr *addr, FAR socklen_t *addrlen);
static int        bluetooth_connect(FAR struct socket *psock,
                    FAR const struct sockaddr *addr, socklen_t addrlen);
static int        bluetooth_close(FAR struct socket *psock);

/* Protocol Specific Interfaces */

static int        bluetooth_l2cap_bind(FAR struct socket *psock,
                    FAR const struct sockaddr_l2 *addr, socklen_t addrlen);
static int        bluetooth_hci_bind(FAR struct socket *psock,
                    FAR const struct sockaddr_hci *addr,
                    socklen_t addrlen);

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct sock_intf_s g_bluetooth_sockif =
{
  bluetooth_setup,       /* si_setup */
  bluetooth_sockcaps,    /* si_sockcaps */
  bluetooth_addref,      /* si_addref */
  bluetooth_bind,        /* si_bind */
  bluetooth_getsockname, /* si_getsockname */
  bluetooth_getpeername, /* si_getpeername */
  NULL,                  /* si_listen */
  bluetooth_connect,     /* si_connect */
  NULL,                  /* si_accept */
  NULL,                  /* si_poll */
  bluetooth_sendmsg,     /* si_sendmsg */
  bluetooth_recvmsg,     /* si_recvmsg */
  bluetooth_close        /* si_close */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bluetooth_sockif_alloc
 *
 * Description:
 *   Allocate and attach a PF_BLUETOOTH connection structure.
 *
 * Assumptions:
 *   The network is locked
 *
 ****************************************************************************/

static int bluetooth_sockif_alloc(FAR struct socket *psock)
{
  /* Allocate the packet socket connection structure and save in the new
   * socket instance.
   */

  FAR struct bluetooth_conn_s *conn = bluetooth_conn_alloc();
  if (conn == NULL)
    {
      /* Failed to reserve a connection structure */

      return -ENOMEM;
    }

  /* Set the reference count on the connection structure.  This reference
   * count will be incremented only if the socket is dup'ed
   */

  DEBUGASSERT(conn->bc_crefs == 0);
  conn->bc_crefs = 1;

  /* Save the pre-allocated connection in the socket structure */

  psock->s_conn = conn;
  return OK;
}

/****************************************************************************
 * Name: bluetooth_setup
 *
 * Description:
 *   Called for socket() to verify that the provided socket type and
 *   protocol are usable by this address family.  Perform any family-
 *   specific socket fields.
 *
 * Input Parameters:
 *   psock    A pointer to a user allocated socket structure to be
 *            initialized.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise, a negated errno value is
 *   returned.
 *
 ****************************************************************************/

static int bluetooth_setup(FAR struct socket *psock)
{
  /* Allocate the appropriate connection structure.  This reserves the
   * connection structure, it is unallocated at this point.  It will not
   * actually be initialized until the socket is connected.
   *
   * SOCK_RAW and SOCK_CTRL are supported
   */

  if (psock->s_type == SOCK_RAW || psock->s_type == SOCK_CTRL)
    {
      return bluetooth_sockif_alloc(psock);
    }
  else
    {
      return -EPROTONOSUPPORT;
    }
}

/****************************************************************************
 * Name: bluetooth_sockcaps
 *
 * Description:
 *   Return the bit encoded capabilities of this socket.
 *
 * Input Parameters:
 *   psock - Socket structure of the socket whose capabilities are being
 *           queried.
 *
 * Returned Value:
 *   The set of socket capabilities is returned.
 *
 ****************************************************************************/

static sockcaps_t bluetooth_sockcaps(FAR struct socket *psock)
{
  return 0;
}

/****************************************************************************
 * Name: bluetooth_addref
 *
 * Description:
 *   Increment the reference count on the underlying connection structure.
 *
 * Input Parameters:
 *   psock - Socket structure of the socket whose reference count will be
 *           incremented.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void bluetooth_addref(FAR struct socket *psock)
{
  FAR struct bluetooth_conn_s *conn;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL &&
              (psock->s_type == SOCK_RAW || psock->s_type == SOCK_CTRL));

  conn = psock->s_conn;
  DEBUGASSERT(conn->bc_crefs > 0 && conn->bc_crefs < 255);
  conn->bc_crefs++;
}

/****************************************************************************
 * Name: bluetooth_connect
 *
 * Description:
 *   bluetooth_connect() connects the local socket referred to by the
 *   structure 'psock' to the address specified by 'addr'. The addrlen
 *   argument specifies the size of 'addr'.  The format of the address in
 *   'addr' is determined by the address space of the socket 'psock'.
 *
 *   Generally, connection-based protocol sockets may successfully
 *   bluetooth_connect() only once; connectionless protocol sockets may use
 *   bluetooth_connect() multiple times to change their association.
 *   Connectionless sockets may dissolve the association by connecting to
 *   an address with the bt_family member of sockaddr set to AF_UNSPEC.
 *
 * Input Parameters:
 *   psock     Pointer to a socket structure initialized by psock_socket()
 *   addr      Server address (form depends on type of socket)
 *   addrlen   Length of actual 'addr'
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure.  See connect() for the
 *   list of appropriate errno values to be returned.
 *
 ****************************************************************************/

static int bluetooth_connect(FAR struct socket *psock,
                             FAR const struct sockaddr *addr,
                             socklen_t addrlen)
{
  FAR struct bluetooth_conn_s *conn;
  FAR struct sockaddr_l2 *btaddr;
  int ret = OK;

  DEBUGASSERT(psock != NULL || addr != NULL);
  conn = psock->s_conn;
  DEBUGASSERT(conn != NULL);

  /* Verify the address family */

  if (addr->sa_family == AF_BLUETOOTH)
    {
      /* Verify the Protocol */

      if (psock->s_proto != BTPROTO_L2CAP)
        {
          return -EPFNOSUPPORT;
        }

      /* Save the "connection" address */

      btaddr = (FAR struct sockaddr_l2 *)addr;
      memcpy(&conn->bc_raddr, &btaddr->l2_bdaddr, sizeof(bt_addr_t));
      conn->bc_channel = btaddr->l2_cid;
      conn->bc_proto = psock->s_proto;
    }
  else
    {
      /* The specified address is not a valid address for the address family
       * of the specified socket.
       */

      ret = -EAFNOSUPPORT;
    }

  return ret;
}

/****************************************************************************
 * Name: bluetooth_bind
 *
 * Description:
 *   bluetooth_bind() gives the socket 'psock' the local address 'addr'.
 *   'addr' is 'addrlen' bytes long.  Traditionally, this is called
 *   "assigning a name to a socket."  When a socket is created with
 *   socket(), it exists in a name space (address family) but has no name
 *   assigned.
 *
 * Input Parameters:
 *   psock    Socket structure of the socket to bind
 *   addr     Socket local address
 *   addrlen  Length of 'addr'
 *
 * Returned Value:
 *   0 on success;  A negated errno value is returned on failure.  See
 *   bind() for a list a appropriate error values.
 *
 ****************************************************************************/

static int bluetooth_bind(FAR struct socket *psock,
                          FAR const struct sockaddr *addr, socklen_t addrlen)
{
  DEBUGASSERT(psock != NULL && addr != NULL);

  /* Verify that a valid address has been provided */

  if (addr->sa_family != AF_BLUETOOTH)
    {
      nerr("ERROR: Invalid family: %u\n", addr->sa_family);
      return -EBADF;
    }

  switch (psock->s_proto)
    {
      case BTPROTO_L2CAP:
        {
          FAR const struct sockaddr_l2 *iaddr;
          if (addrlen < sizeof(struct sockaddr_l2))
            {
              nerr("ERROR: Invalid address length: %zu < %zu\n",
                   (size_t)addrlen, sizeof(struct sockaddr_l2));
              return -EBADF;
            }

          iaddr = (FAR const struct sockaddr_l2 *)addr;
          return bluetooth_l2cap_bind(psock, iaddr, addrlen);
        }

      case BTPROTO_HCI:
        {
          FAR const struct sockaddr_hci *hciaddr;
          if (addrlen < sizeof(struct sockaddr_hci))
            {
              nerr("ERROR: Invalid address length: %zu < %zu\n",
                   (size_t)addrlen, sizeof(struct sockaddr_hci));
              return -EBADF;
            }

          hciaddr = (FAR const struct sockaddr_hci *)addr;
          return bluetooth_hci_bind(psock, hciaddr, addrlen);
        }

      default:
        return -EPFNOSUPPORT;
    }

  return OK;
}

/****************************************************************************
 * Name: bluetooth_l2cap_bind
 *
 * Description:
 *   bluetooth_bind() gives the socket 'psock' the local address 'iaddr'.
 *   'iaddr' is 'addrlen' bytes long.  Traditionally, this is called
 *   "assigning a name to a socket."  When a socket is created with
 *   socket(), it exists in a name space (address family) but has no name
 *   assigned.
 *
 * Input Parameters:
 *   psock    Socket structure of the socket to bind
 *   iaddr    Socket local address
 *   addrlen  Length of 'addr'
 *
 * Returned Value:
 *   0 on success;  A negated errno value is returned on failure.  See
 *   bind() for a list a appropriate error values.
 *
 ****************************************************************************/

static int bluetooth_l2cap_bind(FAR struct socket *psock,
                                FAR const struct sockaddr_l2 *iaddr,
                                socklen_t addrlen)
{
  FAR struct radio_driver_s *radio;
  FAR struct bluetooth_conn_s *conn;

  /* Bind a PF_BLUETOOTH socket to an network device.
   *
   * SOCK_RAW and SOCK_CTRL are supported
   */

  if (psock == NULL || psock->s_conn == NULL ||
      (psock->s_type != SOCK_RAW && psock->s_type != SOCK_CTRL))
    {
      nerr("ERROR: Invalid socket type: %u\n", psock->s_type);
      return -EBADF;
    }

  conn = psock->s_conn;

  /* Verify that the socket is not already bound. */

  if (_SS_ISBOUND(conn->bc_conn.s_flags))
    {
      nerr("ERROR: Already bound\n");
      return -EINVAL;
    }

  /* Very that some address was provided.
   *
   * REVISIT: Currently and explicit address must be assigned.  Should we
   * support some moral equivalent to INADDR_ANY?
   */

  conn->bc_proto = psock->s_proto;

  /* Find the device associated with the requested address */

  radio = bluetooth_find_device(conn, &iaddr->l2_bdaddr);
  if (radio == NULL)
    {
      nerr("ERROR: No radio at this address\n");
      return -ENODEV;
    }

  /* Save the address as the socket's local address */

  memcpy(&conn->bc_laddr, &iaddr->l2_bdaddr, sizeof(bt_addr_t));

  return OK;
}

/****************************************************************************
 * Name: bluetooth_l2cap_bind
 *
 * Description:
 *   bluetooth_bind() gives the socket 'psock' the local address 'hciaddr'.
 *   'hciaddr' is 'addrlen' bytes long.  Traditionally, this is called
 *   "assigning a name to a socket."  When a socket is created with
 *   socket(), it exists in a name space (address family) but has no name
 *   assigned.
 *
 * Input Parameters:
 *   psock    Socket structure of the socket to bind
 *   hciaddr  Socket local address
 *   addrlen  Length of 'addr'
 *
 * Returned Value:
 *   0 on success;  A negated errno value is returned on failure.  See
 *   bind() for a list a appropriate error values.
 *
 ****************************************************************************/

static int bluetooth_hci_bind(FAR struct socket *psock,
                              FAR const struct sockaddr_hci *hciaddr,
                             socklen_t addrlen)
{
  FAR struct bluetooth_conn_s *conn;

  /* Bind a PF_BLUETOOTH socket to an network device.
   *
   * SOCK_RAW and SOCK_CTRL are supported
   */

  if (psock == NULL || psock->s_conn == NULL ||
      (psock->s_type != SOCK_RAW && psock->s_type != SOCK_CTRL))
    {
      nerr("ERROR: Invalid socket type: %u\n", psock->s_type);
      return -EBADF;
    }

  conn = psock->s_conn;

  /* Verify that the socket is not already bound. */

  if (_SS_ISBOUND(conn->bc_conn.s_flags))
    {
      nerr("ERROR: Already bound\n");
      return -EINVAL;
    }

  conn->bc_proto = psock->s_proto;
  conn->bc_channel = hciaddr->hci_channel;
  conn->bc_ldev = hciaddr->hci_dev;

  return OK;
}

/****************************************************************************
 * Name: bluetooth_getsockname
 *
 * Description:
 *   The bluetooth_getsockname() function retrieves the locally-bound name of
 *   the specified packet socket, stores this address in the sockaddr
 *   structure pointed to by the 'addr' argument, and stores the length of
 *   this address in the object pointed to by the 'addrlen' argument.
 *
 *   If the actual length of the address is greater than the length of the
 *   supplied sockaddr structure, the stored address will be truncated.
 *
 *   If the socket has not been bound to a local name, the value stored in
 *   the object pointed to by address is unspecified.
 *
 * Input Parameters:
 *   psock    Socket structure of the socket to be queried
 *   addr     sockaddr structure to receive data [out]
 *   addrlen  Length of sockaddr structure [in/out]
 *
 * Returned Value:
 *   On success, 0 is returned, the 'addr' argument points to the address
 *   of the socket, and the 'addrlen' argument points to the length of the
 *   address.  Otherwise, a negated errno value is returned.  See
 *   getsockname() for the list of appropriate error numbers.
 *
 ****************************************************************************/

static int bluetooth_getsockname(FAR struct socket *psock,
                                  FAR struct sockaddr *addr, FAR
                                  socklen_t *addrlen)
{
  FAR struct bluetooth_conn_s *conn;
  FAR struct sockaddr_l2 tmp;
  socklen_t copylen;

  DEBUGASSERT(psock != NULL && addr != NULL && addrlen != NULL);

  conn = psock->s_conn;
  DEBUGASSERT(conn != NULL);

  /* Create a copy of the full address on the stack */

  tmp.l2_family = AF_BLUETOOTH;
  memcpy(&tmp.l2_bdaddr, &conn->bc_laddr, sizeof(bt_addr_t));

  /* Copy to the user buffer, truncating if necessary */

  copylen = sizeof(struct sockaddr_l2);
  if (copylen > *addrlen)
    {
      copylen = *addrlen;
    }

  memcpy(addr, &tmp, copylen);

  /* Return the actual size transferred */

  *addrlen = copylen;
  return OK;
}

/****************************************************************************
 * Name: bluetooth_getpeername
 *
 * Description:
 *   The bluetooth_getpeername() function retrieves the remote-connected name
 *   of the specified local socket, stores this address in the sockaddr
 *   structure pointed to by the 'addr' argument, and stores the length of
 *   this address in the object pointed to by the 'addrlen' argument.
 *
 *   If the actual length of the address is greater than the length of the
 *   supplied sockaddr structure, the stored address will be truncated.
 *
 *   If the socket has not been bound to a local name, the value stored in
 *   the object pointed to by address is unspecified.
 *
 * Parameters:
 *   psock    Socket structure of the socket to be queried
 *   addr     sockaddr structure to receive data [out]
 *   addrlen  Length of sockaddr structure [in/out]
 *
 * Returned Value:
 *   On success, 0 is returned, the 'addr' argument points to the address
 *   of the socket, and the 'addrlen' argument points to the length of the
 *   address.  Otherwise, a negated errno value is returned.  See
 *   getpeername() for the list of appropriate error numbers.
 *
 ****************************************************************************/

static int bluetooth_getpeername(FAR struct socket *psock,
                                 FAR struct sockaddr *addr,
                                 FAR socklen_t *addrlen)
{
  FAR struct bluetooth_conn_s *conn;
  FAR struct sockaddr_l2 tmp;
  socklen_t copylen;

  DEBUGASSERT(psock != NULL && addr != NULL && addrlen != NULL);

  if (psock->s_proto != BTPROTO_L2CAP)
    {
      return -EPFNOSUPPORT;
    }

  conn = psock->s_conn;
  DEBUGASSERT(conn != NULL);

  /* Create a copy of the full address on the stack */

  tmp.l2_family = AF_BLUETOOTH;
  memcpy(&tmp.l2_bdaddr, &conn->bc_raddr, sizeof(bt_addr_t));

  /* Copy to the user buffer, truncating if necessary */

  copylen = sizeof(struct sockaddr_l2);
  if (copylen > *addrlen)
    {
      copylen = *addrlen;
    }

  memcpy(addr, &tmp, copylen);

  /* Return the actual size transferred */

  *addrlen = copylen;
  return OK;
}

/****************************************************************************
 * Name: bluetooth_close
 *
 * Description:
 *   Performs the close operation on a PF_BLUETOOTH socket instance
 *
 * Input Parameters:
 *   psock   Socket instance
 *
 * Returned Value:
 *   0 on success; a negated errno value is returned on any failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int bluetooth_close(FAR struct socket *psock)
{
  /* Perform some pre-close operations for the PF_BLUETOOTH address type */

  switch (psock->s_type)
    {
      /* SOCK_RAW and SOCK_CTRL are supported */

      case SOCK_RAW:
      case SOCK_CTRL:
        {
          FAR struct bluetooth_conn_s *conn = psock->s_conn;

          /* Is this the last reference to the connection structure (there
           * could be more if the socket was dup'ed).
           */

          if (conn->bc_crefs <= 1)
            {
              /* Yes... free the connection structure */

              conn->bc_crefs = 0;                  /* No more references on the connection */
              bluetooth_conn_free(psock->s_conn);  /* Free network resources */
            }
          else
            {
              /* No.. Just decrement the reference count */

              conn->bc_crefs--;
            }

          return OK;
        }

      default:
        return -EBADF;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* CONFIG_NET_BLUETOOTH */
