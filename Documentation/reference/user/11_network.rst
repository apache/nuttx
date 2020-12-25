==================
Network Interfaces
==================

NuttX supports a BSD-compatible socket interface layer. These socket
interface can be enabled by settings in the architecture configuration
file. Those socket APIs are discussed in
the following paragraphs.

  - :c:func:`socket`
  - :c:func:`bind`
  - :c:func:`connect`
  - :c:func:`listen`
  - :c:func:`accept`
  - :c:func:`send`
  - :c:func:`sendto`
  - :c:func:`recv`
  - :c:func:`recvfrom`
  - :c:func:`setsockopt`
  - :c:func:`getsockopt`

.. c:function:: int socket(int domain, int type, int protocol);

  Creates an endpoint for communication and
  returns a descriptor.

  :param domain: (see sys/socket.h)
  :param type: (see sys/socket.h)
  :param protocol: (see sys/socket.h)

  :return:
    0 on success; -1 on error with
    ``errno`` set appropriately:

    -  ``EACCES``. Permission to create a socket of the specified type
       and/or protocol is denied.
    -  ``EAFNOSUPPORT``. The implementation does not support the specified
       address family.
    -  ``EINVAL``. Unknown protocol, or protocol family not available.
    -  ``EMFILE``. Process file table overflow.
    -  ``ENFILE`` The system limit on the total number of open files has
       been reached.
    -  ``ENOBUFS`` or ``ENOMEM``. Insufficient memory is available. The
       socket cannot be created until sufficient resources are freed.
    -  ``EPROTONOSUPPORT``. The protocol type or the specified protocol is
       not supported within this domain.

.. c:function:: int bind(int sockfd, const struct sockaddr *addr, socklen_t addrlen)

  Gives the socket sockfd the local address
  ``addr``. ``addr`` is ``addrlen`` bytes long. Traditionally, this is
  called "assigning a name to a socket." When a socket is created with
  ``socket()``, it exists in a name space (address family) but has no name
  assigned.

  :param sockfd: Socket descriptor from socket.
  :param addr: Socket local address.
  :param addrlen: Length of ``addr``.

  :return: 0 on success; -1 on error with ``errno`` set appropriately:
    - ``EACCES`` The address is protected, and the user is not the
    superuser.
    - ``EADDRINUSE`` The given address is already in use.
    - ``EBADF`` ``sockfd`` is not a valid descriptor.
    - ``EINVAL`` The socket is already bound to an address.
    - ``ENOTSOCK`` ``sockfd`` is a descriptor for a file, not a socket.

.. c:function:: int connect(int sockfd, const struct sockaddr *addr, socklen_t addrlen);

  ``connect()`` connects the socket referred to by the
  file descriptor ``sockfd`` to the address specified by ``addr``. The
  ``addrlen`` argument specifies the size of ``addr``. The format of the
  address in ``addr`` is determined by the address space of the socket
  sockfd. If the socket sockfd is of type SOCK_DGRAM then ``addr`` is the
  address to which datagrams are sent by default, and the only address
  from which datagrams are received. If the socket is of type SOCK_STREAM
  or SOCK_SEQPACKET, this call attempts to make a connection to the socket
  that is bound to the address specified by ``addr``. Generally,
  connection-based protocol sockets may successfully ``connect()`` only
  once; connectionless protocol sockets may use ``connect()`` multiple
  times to change their association. Connectionless sockets may dissolve
  the association by connecting to an address with the sa_family member of
  sockaddr set to AF_UNSPEC.

  **Input Parameters:**

  -  ``sockfd``: Socket descriptor returned by ``socket()``
  -  ``addr``: Server address (form depends on type of socket)
  -  ``addrlen``: Length of actual ``addr``

  **Returned Value:** 0 on success; -1 on error with
  ```errno`` <#ErrnoAccess>`__ set appropriately:

  ``EACCES`` or EPERM: The user tried to connect to a broadcast address
  without having the socket broadcast flag enabled or the connection
  request failed because of a local firewall rule.

  ``EADDRINUSE`` Local address is already in use.

  ``EAFNOSUPPORT`` The passed address didn't have the correct address
  family in its sa_family field.

  ``EAGAIN`` No more free local ports or insufficient entries in the
  routing cache. For PF_INET.

  ``EALREADY`` The socket is non-blocking and a previous connection
  attempt has not yet been completed.

  ``EBADF`` The file descriptor is not a valid index in the descriptor
  table.

  ``ECONNREFUSED`` No one listening on the remote address.

  ``EFAULT`` The socket structure address is outside the user's address
  space.

  ``EINPROGRESS`` The socket is non-blocking and the connection cannot be
  completed immediately.

  ``EINTR`` The system call was interrupted by a signal that was caught.

  ``EISCONN`` The socket is already connected.

  ``ENETUNREACH`` Network is unreachable.

  ``ENOTSOCK`` The file descriptor is not associated with a socket.

  ``ETIMEDOUT`` Timeout while attempting connection. The server may be too
  busy to accept new connections.

.. c:function:: int listen(int sockfd, int backlog);

  To accept connections, a socket is first created with
  ``socket()``, a willingness to accept incoming connections and a queue
  limit for incoming connections are specified with ``listen()``, and then
  the connections are accepted with ``accept()``. The ``listen()`` call
  applies only to sockets of type ``SOCK_STREAM`` or ``SOCK_SEQPACKET``.

  **Input Parameters:**

  -  ``sockfd``: Socket descriptor of the bound socket.
  -  ``backlog``: The maximum length the queue of pending connections may
     grow. If a connection request arrives with the queue full, the client
     may receive an error with an indication of ECONNREFUSED or, if the
     underlying protocol supports retransmission, the request may be
     ignored so that retries succeed.

  **Returned Value:** On success, zero is returned. On error, -1 is
  returned, and ```errno`` <#ErrnoAccess>`__ is set appropriately.

  -  ``EADDRINUSE``: Another socket is already listening on the same port.
  -  ``EBADF``: The argument ``sockfd`` is not a valid descriptor.
  -  ``ENOTSOCK``: The argument ``sockfd`` is not a socket.
  -  ``EOPNOTSUPP``: The socket is not of a type that supports the listen
     operation.

.. c:function:: int accept(int sockfd, struct sockaddr *addr, socklen_t *addrlen);

  The ``accept()`` function is used with connection-based
  socket types (``SOCK_STREAM``, ``SOCK_SEQPACKET`` and ``SOCK_RDM``). It
  extracts the first connection request on the queue of pending
  connections, creates a new connected socket with most of the same
  properties as ``sockfd``, and allocates a new socket descriptor for the
  socket, which is returned. The newly created socket is no longer in the
  listening state. The original socket ``sockfd`` is unaffected by this
  call. Per file descriptor flags are not inherited across an accept.

  The ``sockfd`` argument is a socket descriptor that has been created
  with ``socket()``, bound to a local address with ``bind()``, and is
  listening for connections after a call to ``listen()``.

  On return, the ``addr`` structure is filled in with the address of the
  connecting entity. The ``addrlen`` argument initially contains the size
  of the structure pointed to by ``addr``; on return it will contain the
  actual length of the address returned.

  If no pending connections are present on the queue, and the socket is
  not marked as non-blocking, accept blocks the caller until a connection
  is present. If the socket is marked non-blocking and no pending
  connections are present on the queue, accept returns ``EAGAIN``.

  **Input Parameters:**

  -  ``sockfd``: Socket descriptor of the listening socket.
  -  ``addr``: Receives the address of the connecting client.
  -  ``addrlen``: Input: allocated size of ``addr``, Return: returned size
     of ``addr``.

  **Returned Value:** Returns -1 on error. If it succeeds, it returns a
  non-negative integer that is a descriptor for the accepted socket.

  -  ``EAGAIN`` or ``EWOULDBLOCK``: The socket is marked non-blocking and
     no connections are present to be accepted.
  -  ``EBADF``: The descriptor is invalid.
  -  ``ENOTSOCK``: The descriptor references a file, not a socket.
  -  ``EOPNOTSUPP``: The referenced socket is not of type ``SOCK_STREAM``.
  -  ``EINTR``: The system call was interrupted by a signal that was
     caught before a valid connection arrived.
  -  ``ECONNABORTED``: A connection has been aborted.
  -  ``EINVAL``: Socket is not listening for connections.
  -  ``EMFILE``: The per-process limit of open file descriptors has been
     reached.
  -  ``ENFILE``: The system maximum for file descriptors has been reached.
  -  ``EFAULT``: The addr parameter is not in a writable part of the user
     address space.
  -  ``ENOBUFS`` or ``ENOMEM``: Not enough free memory.
  -  ``EPROTO``: Protocol error.
  -  ``EPERM``: Firewall rules forbid connection.

.. c:function:: ssize_t send(int sockfd, const void *buf, size_t len, int flags);

  The ``send()`` call may be used only when the socket is
  in a connected state (so that the intended recipient is known). The only
  difference between ``send()`` and ``write()`` is the presence of
  ``flags``. With ``zero`` flags parameter, ``send()`` is equivalent to
  ``write()``. Also, ``send(s,buf,len,flags)`` is equivalent to
  ``sendto(s,buf,len,flags,NULL,0)``.

  **Input Parameters:**

  -  ``sockfd``: Socket descriptor of socket
  -  ``buf``: Data to send
  -  ``len``: Length of data to send
  -  ``flags``: Send flags

  **Returned Value:** See ```sendto()`` <#sendto>`__.

.. c:function:: ssize_t sendto(int sockfd, const void *buf, size_t len, int flags, \
               const struct sockaddr *to, socklen_t tolen);

  If ``sendto()`` is used on a connection-mode
  (SOCK_STREAM, SOCK_SEQPACKET) socket, the parameters to and tolen are
  ignored (and the error EISCONN may be returned when they are not NULL
  and 0), and the error ENOTCONN is returned when the socket was not
  actually connected.

  **Input Parameters:**

  -  ``sockfd``: Socket descriptor of socket
  -  ``buf``: Data to send
  -  ``len``: Length of data to send
  -  ``flags``: Send flags
  -  ``to``: Address of recipient
  -  ``tolen``: The length of the address structure

  **Returned Value:** On success, returns the number of characters sent.
  On error, -1 is returned, and ```errno`` <#ErrnoAccess>`__ is set
  appropriately:

  -  ``EAGAIN`` or ``EWOULDBLOCK``. The socket is marked non-blocking and
     the requested operation would block.
  -  ``EBADF``. An invalid descriptor was specified.
  -  ``ECONNRESET``. Connection reset by peer.
  -  ``EDESTADDRREQ``. The socket is not connection-mode, and no peer
     address is set.
  -  ``EFAULT``. An invalid user space address was specified for a
     parameter.
  -  ``EINTR``. A signal occurred before any data was transmitted.
  -  ``EINVAL``. Invalid argument passed.
  -  ``EISCONN``. The connection-mode socket was connected already but a
     recipient was specified. (Now either this error is returned, or the
     recipient specification is ignored.)
  -  ``EMSGSIZE``. The socket type requires that message be sent
     atomically, and the size of the message to be sent made this
     impossible.
  -  ``ENOBUFS``. The output queue for a network interface was full. This
     generally indicates that the interface has stopped sending, but may
     be caused by transient congestion.
  -  ``ENOMEM``. No memory available.
  -  ``ENOTCONN``. The socket is not connected, and no target has been
     given.
  -  ``ENOTSOCK``. The argument s is not a socket.
  -  ``EOPNOTSUPP``. Some bit in the flags argument is inappropriate for
     the socket type.
  -  ``EPIPE``. The local end has been shut down on a connection oriented
     socket. In this case the process will also receive a SIGPIPE unless
     MSG_NOSIGNAL is set.


.. c:function:: ssize_t recv(int sockfd, void *buf, size_t len, int flags);

  The ``recv()`` call is identical to
  ```recvfrom()`` <#recvfrom>`__ with a NULL ``from`` parameter.

  **Input Parameters:**

  -  sockfd: Socket descriptor of socket
  -  buf: Buffer to receive data
  -  len: Length of buffer
  -  flags: Receive flags

  **Returned Value:** See ```recvfrom()`` <#recvfrom>`__.


.. c:function:: ssize_t recvfrom(int sockfd, void *buf, size_t len, int flags, \
                 struct sockaddr *from, socklen_t *fromlen);

  ``recvfrom()`` receives messages from a socket, and may
  be used to receive data on a socket whether or not it is
  connection-oriented.

  If ``from`` is not NULL, and the underlying protocol provides the source
  address, this source address is filled in. The argument ``fromlen``
  initialized to the size of the buffer associated with ``from``, and
  modified on return to indicate the actual size of the address stored
  there.

  **Input Parameters:**

  -  ``sockfd``: Socket descriptor of socket.
  -  ``buf``: Buffer to receive data.
  -  ``len``: Length of buffer.
  -  ``flags``: Receive flags.
  -  ``from``: Address of source.
  -  ``fromlen``: The length of the address structure.

  **Returned Value:** On success, returns the number of characters sent.
  If no data is available to be received and the peer has performed an
  orderly shutdown, recv() will return 0. Otherwise, on errors, -1 is
  returned, and ```errno`` <#ErrnoAccess>`__ is set appropriately:

  -  ``EAGAIN``. The socket is marked non-blocking and the receive
     operation would block, or a receive timeout had been set and the
     timeout expired before data was received.
  -  ``EBADF``. The argument ``sockfd`` is an invalid descriptor.
  -  ``ECONNREFUSED``. A remote host refused to allow the network
     connection (typically because it is not running the requested
     service).
  -  ``EFAULT``. The receive buffer pointer(s) point outside the process's
     address space.
  -  ``EINTR``. The receive was interrupted by delivery of a signal before
     any data were available.
  -  ``EINVAL``. Invalid argument passed.
  -  ``ENOMEM``. Could not allocate memory.
  -  ``ENOTCONN``. The socket is associated with a connection-oriented
     protocol and has not been connected.
  -  ``ENOTSOCK``. The argument ``sockfd`` does not refer to a socket.

.. c:function:: int setsockopt(int sockfd, int level, int option, \
               const void *value, socklen_t value_len);

  ``setsockopt()`` sets the option specified by the
  ``option`` argument, at the protocol level specified by the ``level``
  argument, to the value pointed to by the ``value`` argument for the
  socket associated with the file descriptor specified by the ``sockfd``
  argument.

  The ``level`` argument specifies the protocol level of the option. To
  set options at the socket level, specify the level argument as
  SOL_SOCKET.

  See ``sys/socket.h`` for a complete list of values for the ``option``
  argument.

  **Input Parameters:**

  -  ``sockfd``: Socket descriptor of socket
  -  ``level``: Protocol level to set the option
  -  ``option``: identifies the option to set
  -  ``value``: Points to the argument value
  -  ``value_len``: The length of the argument value

  **Returned Value:** On success, returns the number of characters sent.
  On error, -1 is returned, and ```errno`` <#ErrnoAccess>`__ is set
  appropriately:

  -  ``BADF``. The ``sockfd`` argument is not a valid socket descriptor.
  -  ``DOM``. The send and receive timeout values are too big to fit into
     the timeout fields in the socket structure.
  -  ``INVAL``. The specified option is invalid at the specified socket
     ``level`` or the socket has been shut down.
  -  ``ISCONN``. The socket is already connected, and a specified option
     cannot be set while the socket is connected.
  -  ``NOPROTOOPT``. The ``option`` is not supported by the protocol.
  -  ``NOTSOCK``. The ``sockfd`` argument does not refer to a socket.
  -  ``NOMEM``. There was insufficient memory available for the operation
     to complete.
  -  ``NOBUFS``. Insufficient resources are available in the system to
     complete the call.

.. c:function:: int getsockopt(int sockfd, int level, int option, void *value, socklen_t *value_len);

  ``getsockopt()`` retrieve those value for the option
  specified by the ``option`` argument for the socket specified by the
  ``sockfd`` argument. If the size of the option value is greater than
  ``value_len``, the value stored in the object pointed to by the
  ``value`` argument will be silently truncated. Otherwise, the length
  pointed to by the ``value_len`` argument will be modified to indicate
  the actual length of the ``value``.

  The ``level`` argument specifies the protocol level of the option. To
  retrieve options at the socket level, specify the level argument as
  SOL_SOCKET.

  See ``sys/socket.h`` for a complete list of values for the ``option``
  argument.

  **Input Parameters:**

  -  ``sockfd``: Socket descriptor of socket
  -  ``level``: Protocol level to set the option
  -  ``option``: Identifies the option to get
  -  ``value``: Points to the argument value
  -  ``value_len``: The length of the argument value

  **Returned Value:** On success, returns the number of characters sent.
  On error, -1 is returned, and ```errno`` <#ErrnoAccess>`__ is set
  appropriately:

  -  ``BADF``. The ``sockfd`` argument is not a valid socket descriptor.
  -  ``INVAL``. The specified option is invalid at the specified socket
     ``level`` or the socket has been shutdown.
  -  ``NOPROTOOPT``. The ``option`` is not supported by the protocol.
  -  ``NOTSOCK``. The ``sockfd`` argument does not refer to a socket.
  -  ``NOBUFS``. Insufficient resources are available in the system to
     complete the call.

