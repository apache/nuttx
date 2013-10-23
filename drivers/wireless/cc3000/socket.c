/*****************************************************************************
 *  socket.c  - CC3000 Host Driver Implementation.
 *  Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <debug.h>
#include <stdlib.h>
#include <nuttx/wireless/cc3000/hci.h>
#include <nuttx/wireless/cc3000/include/sys/socket.h>
#include <nuttx/wireless/cc3000/evnt_handler.h>
#include <nuttx/wireless/cc3000/netapp.h>

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

/* Enable this flag if and only if you must comply with BSD socket close()
 * function
 */

#ifdef _API_USE_BSD_CLOSE
#  define close(sd) closesocket(sd)
#endif

/* Enable this flag if and only if you must comply with BSD socket read() and
 * write() functions
 */

#ifdef _API_USE_BSD_READ_WRITE
#  define read(sd, buf, len, flags) recv(sd, buf, len, flags)
#  define write(sd, buf, len, flags) send(sd, buf, len, flags)
#endif

#define SOCKET_OPEN_PARAMS_LEN             (12)
#define SOCKET_CLOSE_PARAMS_LEN            (4)
#define SOCKET_ACCEPT_PARAMS_LEN           (4)
#define SOCKET_BIND_PARAMS_LEN             (20)
#define SOCKET_LISTEN_PARAMS_LEN           (8)
#define SOCKET_GET_HOST_BY_NAME_PARAMS_LEN (9)
#define SOCKET_CONNECT_PARAMS_LEN          (20)
#define SOCKET_SELECT_PARAMS_LEN           (44)
#define SOCKET_SET_SOCK_OPT_PARAMS_LEN     (20)
#define SOCKET_GET_SOCK_OPT_PARAMS_LEN     (12)
#define SOCKET_RECV_FROM_PARAMS_LEN        (12)
#define SOCKET_SENDTO_PARAMS_LEN           (24)
#define SOCKET_MDNS_ADVERTISE_PARAMS_LEN   (12)

/* The legnth of arguments for the SEND command: sd + buff_offset + len + flags,
 * while size of each parameter is 32 bit - so the total length is 16 bytes;
 */

#define HCI_CMND_SEND_ARG_LENGTH          (16)

#define SELECT_TIMEOUT_MIN_MICRO_SECONDS  5000

#define HEADERS_SIZE_DATA                 (SPI_HEADER_SIZE + 5)

#define SIMPLE_LINK_HCI_CMND_TRANSPORT_HEADER_SIZE \
  (SPI_HEADER_SIZE + SIMPLE_LINK_HCI_CMND_HEADER_SIZE)

#define MDNS_DEVICE_SERVICE_MAX_LENGTH    (32)

/*****************************************************************************
 * Public Functions
 *****************************************************************************/
/*****************************************************************************
 * Name: HostFlowControlConsumeBuff
 *
 * Input Parameters:
 *   sd  socket descriptor
 *
 * Returned Value:
 *   0 in case there are buffers available,
 *   -1 in case of bad socket
 *   -2 if there are no free buffers present (only when
 *   SEND_NON_BLOCKING is enabled)
 *
 * Decription:
 *   if SEND_NON_BLOCKING not define - block until have free buffer
 *   becomes available, else return immediately  with correct status
 *   regarding the buffers available.
 *
 *****************************************************************************/

int HostFlowControlConsumeBuff(int sd)
{
#ifndef SEND_NON_BLOCKING
  /* Wait in busy loop */

  do
    {
      /* In case last transmission failed then we will return the last failure
       * reason here.
       * Note that the buffer will not be allocated in this case
       */

      if (tSLInformation.slTransmitDataError != 0)
        {
          errno = tSLInformation.slTransmitDataError;
          tSLInformation.slTransmitDataError = 0;
          return errno;
        }

      if (SOCKET_STATUS_ACTIVE != get_socket_active_status(sd))
        {
          return -1;
        }

      /* We must yield here for the the Event to get processed that returns
       * the buffers
       */

      usleep(100000);
    }
  while (0 == tSLInformation.usNumberOfFreeBuffers);

  tSLInformation.usNumberOfFreeBuffers--;

  return 0;
#else

  /* In case last transmission failed then we will return the last failure
   * reason here.
   * Note that the buffer will not be allocated in this case
   */

  if (tSLInformation.slTransmitDataError != 0)
    {
      errno = tSLInformation.slTransmitDataError;
      tSLInformation.slTransmitDataError = 0;
      return errno;
    }

  if (SOCKET_STATUS_ACTIVE != get_socket_active_status(sd))
    {
      return -1;
    }

  /* If there are no available buffers, return -2. It is recommended to use
   * select or receive to see if there is any buffer occupied with received data
   * If so, call receive() to release the buffer.
   */

  if (0 == tSLInformation.usNumberOfFreeBuffers)
    {
      return -2;
    }
  else
    {
      tSLInformation.usNumberOfFreeBuffers--;
      return 0;
    }
#endif
}

/*****************************************************************************
 * Name: socket
 *
 * Decription:
 *   create an endpoint for communication. The socket function creates a
 *   socket that is bound to a specific transport service provider.  This
 *   function is called by the application layer to obtain a socket handle.
 *
 * Input Parameters:
 *   domain    selects the protocol family which will be used for
 *    communication. On this version only AF_INET is supported
 *   type      specifies the communication semantics. On this version
 *    only SOCK_STREAM, SOCK_DGRAM, SOCK_RAW are supported
 *   protocol  specifies a particular protocol to be used with the
 *    socket IPPROTO_TCP, IPPROTO_UDP or IPPROTO_RAW are
 *    supported.
 *
 * Returned Value:
 *   On success, socket handle that is used for consequent socket
 *    operations. On error, -1 is returned.
 *
 *****************************************************************************/

int socket(long domain, long type, long protocol)
{
  long ret;
  uint8_t *ptr, *args;

  ret = EFAIL;
  ptr = tSLInformation.pucTxCommandBuffer;
  args = (ptr + HEADERS_SIZE_CMD);

  /* Fill in HCI packet structure */

  args = UINT32_TO_STREAM(args, domain);
  args = UINT32_TO_STREAM(args, type);
  args = UINT32_TO_STREAM(args, protocol);

  /* Initiate a HCI command */

  hci_command_send(HCI_CMND_SOCKET, ptr, SOCKET_OPEN_PARAMS_LEN);

  /* Since we are in blocking state - wait for event complete */

  SimpleLinkWaitEvent(HCI_CMND_SOCKET, &ret);

  /* Process the event */

  errno = ret;

  set_socket_active_status(ret, SOCKET_STATUS_ACTIVE);

  return ret;
}

/*****************************************************************************
 * Name: closesocket
 *
 * Decription:
 *   The socket function closes a created socket.
 *
 * Input Parameters:
 *   sd    socket handle.
 *
 * Returned Value:
 *   On success, zero is returned. On error, -1 is returned.
 *
 *****************************************************************************/

long closesocket(long sd)
{
  long ret;
  uint8_t *ptr, *args;

  ret = EFAIL;
  ptr = tSLInformation.pucTxCommandBuffer;
  args = (ptr + HEADERS_SIZE_CMD);

  /* Fill in HCI packet structure */

  args = UINT32_TO_STREAM(args, sd);

  /* Initiate a HCI command */

  hci_command_send(HCI_CMND_CLOSE_SOCKET,
                   ptr, SOCKET_CLOSE_PARAMS_LEN);

  /* Since we are in blocking state - wait for event complete */

  SimpleLinkWaitEvent(HCI_CMND_CLOSE_SOCKET, &ret);
  errno = ret;

  /* Since 'close' call may result in either OK (and then it closed) or error
   * mark this socket as invalid
   */

  set_socket_active_status(sd, SOCKET_STATUS_INACTIVE);

  return ret;
}

/*****************************************************************************
 * Name: accept
 *
 * Decription:
 *   accept a connection on a socket:
 *   This function is used with connection-based socket types
 *   (SOCK_STREAM). It extracts the first connection request on the
 *   queue of pending connections, creates a new connected socket, and
 *   returns a new file descriptor referring to that socket.
 *   The newly created socket is not in the listening state.
 *   The original socket sd is unaffected by this call.
 *   The argument sd is a socket that has been created with socket(),
 *   bound to a local address with bind(), and is  listening for
 *   connections after a listen(). The argument addr is a pointer
 *   to a sockaddr structure. This structure is filled in with the
 *   address of the peer socket, as known to the communications layer.
 *   The exact format of the address returned addr is determined by the
 *   socket's address family. The addrlen argument is a value-result
 *   argument: it should initially contain the size of the structure
 *   pointed to by addr, on return it will contain the actual
 *   length (in bytes) of the address returned.
 *
 * Input Parameters:
 *   sd      socket descriptor (handle)
 *   addr    the argument addr is a pointer to a sockaddr structure
 *           This structure is filled in with the address of the
 *           peer socket, as known to the communications layer.
 *           determined. The exact format of the address returned
 *           addr is by the socket's address sockaddr.
 *           On this version only AF_INET is supported.
 *           This argument returns in network order.
 *   addrlen The addrlen argument is a value-result argument:
 *           it should initially contain the size of the structure
 *            pointed to by addr.
 *
 * Returned Value:
 *   For socket in blocking mode:
 *    On success, socket handle. on failure negative
 *     For socket in non-blocking mode:
 *   - On connection establishment, socket handle
 *   - On connection pending, SOC_IN_PROGRESS (-2)
 *   - On failure, SOC_ERROR  (-1)
 *
 *****************************************************************************/

long accept(long sd, sockaddr *addr, socklen_t *addrlen)
{
  long ret;
  uint8_t *ptr, *args;
  tBsdReturnParams tAcceptReturnArguments;

  ret = EFAIL;
  ptr = tSLInformation.pucTxCommandBuffer;
  args = (ptr + HEADERS_SIZE_CMD);

  /* Fill in temporary command buffer */

  args = UINT32_TO_STREAM(args, sd);

  /* Initiate a HCI command */

  hci_command_send(HCI_CMND_ACCEPT,
                   ptr, SOCKET_ACCEPT_PARAMS_LEN);

  /* Since we are in blocking state - wait for event complete */

  SimpleLinkWaitEvent(HCI_CMND_ACCEPT, &tAcceptReturnArguments);


  /* Need specify return parameters!!! */

  memcpy(addr, &tAcceptReturnArguments.tSocketAddress, ASIC_ADDR_LEN);
  *addrlen = ASIC_ADDR_LEN;
  errno = tAcceptReturnArguments.iStatus;
  ret = errno;

  /* if succeeded, iStatus = new socket descriptor. otherwise - error number */

  if (M_IS_VALID_SD(ret))
    {
      set_socket_active_status(ret, SOCKET_STATUS_ACTIVE);
    }
  else
    {
      set_socket_active_status(sd, SOCKET_STATUS_INACTIVE);
    }

  return ret;
}

/*****************************************************************************
 * Name: bind
 *
 * Decription:
 *   assign a name to a socket
 *   This function gives the socket the local address addr.
 *   addr is addrlen bytes long. Traditionally, this is called when a
 *   socket is created with socket, it exists in a name space (address
 *   family) but has no name assigned.
 *   It is necessary to assign a local address before a SOCK_STREAM
 *   socket may receive connections.
 *
 * Input Parameters:
 *   sd      socket descriptor (handle)
 *   addr    specifies the destination address. On this version
 *    only AF_INET is supported.
 *   addrlen  contains the size of the structure pointed to by addr.
 *
 * Returned Value:
 *   On success, zero is returned. On error, -1 is returned.
 *
 *****************************************************************************/

long bind(long sd, const sockaddr *addr, long addrlen)
{
  long ret;
  uint8_t *ptr, *args;

  ret = EFAIL;
  ptr = tSLInformation.pucTxCommandBuffer;
  args = (ptr + HEADERS_SIZE_CMD);

  addrlen = ASIC_ADDR_LEN;

  /* Fill in temporary command buffer */

  args = UINT32_TO_STREAM(args, sd);
  args = UINT32_TO_STREAM(args, 0x00000008);
  args = UINT32_TO_STREAM(args, addrlen);
  ARRAY_TO_STREAM(args, ((uint8_t *)addr), addrlen);

  /* Initiate a HCI command */

  hci_command_send(HCI_CMND_BIND,
                   ptr, SOCKET_BIND_PARAMS_LEN);

  /* Since we are in blocking state - wait for event complete */

  SimpleLinkWaitEvent(HCI_CMND_BIND, &ret);

  errno = ret;

  return ret;
}

/*****************************************************************************
 * Name: listen
 *
 * Decription:
 *   listen for connections on a socket
 *   The willingness to accept incoming connections and a queue
 *   limit for incoming connections are specified with listen(),
 *   and then the connections are accepted with accept.
 *   The listen() call applies only to sockets of type SOCK_STREAM
 *   The backlog parameter defines the maximum length the queue of
 *   pending connections may grow to.
 *
 * NOTE: On this version, backlog is not supported
 *
 * Input Parameters:
 *   sd      socket descriptor (handle)
 *   backlog  specifies the listen queue depth. On this version
 *    backlog is not supported.
 *
 * Returned Value:
 *   On success, zero is returned. On error, -1 is returned.
 *
 *****************************************************************************/

long listen(long sd, long backlog)
{
  long ret;
  uint8_t *ptr, *args;

  ret = EFAIL;
  ptr = tSLInformation.pucTxCommandBuffer;
  args = (ptr + HEADERS_SIZE_CMD);

  /* Fill in temporary command buffer */

  args = UINT32_TO_STREAM(args, sd);
  args = UINT32_TO_STREAM(args, backlog);

  /* Initiate a HCI command */

  hci_command_send(HCI_CMND_LISTEN,
                   ptr, SOCKET_LISTEN_PARAMS_LEN);

  /* Since we are in blocking state - wait for event complete */

  SimpleLinkWaitEvent(HCI_CMND_LISTEN, &ret);
  errno = ret;

  return ret;
}

/*****************************************************************************
 * Name: gethostbyname
 *
 * Decription:
 *   Get host IP by name. Obtain the IP Address of machine on network,
 *   by its name.
 *
 * NOTE: On this version, only blocking mode is supported. Also note that
 *       the function requires DNS server to be configured prior to its
 *       usage.
 *
 * Input Parameters:
 *   hostname     host name
 *   usNameLen    name length
 *   out_ip_addr  This parameter is filled in with host IP address.
 *   In case that host name is not resolved,
 *   out_ip_addr is zero.
 *
 * Returned Value:
 *   On success, positive is returned. On error, negative is returned
 *
 *****************************************************************************/

#ifndef CC3000_TINY_DRIVER
int gethostbyname(char * hostname, uint16_t usNameLen, unsigned long* out_ip_addr)
{
  tBsdGethostbynameParams ret;
  uint8_t *ptr, *args;

  errno = EFAIL;

  if (usNameLen > HOSTNAME_MAX_LENGTH)
    {
      return errno;
    }

  ptr = tSLInformation.pucTxCommandBuffer;
  args = (ptr + SIMPLE_LINK_HCI_CMND_TRANSPORT_HEADER_SIZE);

  /* Fill in HCI packet structure */

  args = UINT32_TO_STREAM(args, 8);
  args = UINT32_TO_STREAM(args, usNameLen);
  ARRAY_TO_STREAM(args, hostname, usNameLen);

  /* Initiate a HCI command */

  hci_command_send(HCI_CMND_GETHOSTNAME, ptr, SOCKET_GET_HOST_BY_NAME_PARAMS_LEN
                   + usNameLen - 1);

  /* Since we are in blocking state - wait for event complete */

  SimpleLinkWaitEvent(HCI_EVNT_BSD_GETHOSTBYNAME, &ret);

  errno = ret.retVal;

  (*((long*)out_ip_addr)) = ret.outputAddress;

  return errno;
}
#endif

/*****************************************************************************
 * Name: connect
 *
 * Decription:
 *   initiate a connection on a socket
 *   Function connects the socket referred to by the socket descriptor
 *   sd, to the address specified by addr. The addrlen argument
 *   specifies the size of addr. The format of the address in addr is
 *   determined by the address space of the socket. If it is of type
 *   SOCK_DGRAM, this call specifies the peer with which the socket is
 *   to be associated; this address is that to which datagrams are to be
 *   sent, and the only address from which datagrams are to be received.
 *   If the socket is of type SOCK_STREAM, this call attempts to make a
 *   connection to another socket. The other socket is specified  by
 *   address, which is an address in the communications space of the
 *   socket. Note that the function implements only blocking behavior
 *   thus the caller will be waiting either for the connection
 *   establishment or for the connection establishment failure.
 *
 * Input Parameters:
 *   sd       socket descriptor (handle)
 *   addr     specifies the destination addr. On this version
 *     only AF_INET is supported.
 *   addrlen  contains the size of the structure pointed to by addr
 *
 * Returned Value:
 *   On success, zero is returned. On error, -1 is returned
 *
 *****************************************************************************/

long connect(long sd, const sockaddr *addr, long addrlen)
{
  long int ret;
  uint8_t *ptr, *args;

  ret = EFAIL;
  ptr = tSLInformation.pucTxCommandBuffer;
  args = (ptr + SIMPLE_LINK_HCI_CMND_TRANSPORT_HEADER_SIZE);
  addrlen = 8;

  /* Fill in temporary command buffer */

  args = UINT32_TO_STREAM(args, sd);
  args = UINT32_TO_STREAM(args, 0x00000008);
  args = UINT32_TO_STREAM(args, addrlen);
  ARRAY_TO_STREAM(args, ((uint8_t *)addr), addrlen);

  /* Initiate a HCI command */

  hci_command_send(HCI_CMND_CONNECT,
                   ptr, SOCKET_CONNECT_PARAMS_LEN);

  /* Since we are in blocking state - wait for event complete */

  SimpleLinkWaitEvent(HCI_CMND_CONNECT, &ret);

  errno = ret;

  return (long)ret;
}

/*****************************************************************************
 * Name: select
 *
 * Decription:
 *   Monitor socket activity
 *   Select allow a program to monitor multiple file descriptors,
 *   waiting until one or more of the file descriptors become
 *   "ready" for some class of I/O operation
 *
 * NOTE: If the timeout value set to less than 5ms it will automatically set
 *   to 5ms to prevent overload of the system
 *
 * Input Parameters:
 *   nfds       the highest-numbered file descriptor in any of the
 *     three sets, plus 1.
 *   writesds   socket descriptors list for write monitoring
 *   readsds    socket descriptors list for read monitoring
 *   exceptsds  socket descriptors list for exception monitoring
 *   timeout     is an upper bound on the amount of time elapsed
 *     before select() returns. Null means infinity
 *     timeout. The minimum timeout is 5 milliseconds,
 *    less than 5 milliseconds will be set
 *     automatically to 5 milliseconds.
 *
 * Returned Value:
 *   On success, select() returns the number of file descriptors
 *   contained in the three returned descriptor sets (that is, the
 *   total number of bits that are set in readfds, writefds,
 *   exceptfds) which may be zero if the timeout expires before
 *   anything interesting  happens.
 *   On error, -1 is returned.
 *   *readsds - return the sockets on which Read request will
 *     return without delay with valid data.
 *   *writesds - return the sockets on which Write request
 *     will return without delay.
 *   *exceptsds - return the sockets which closed recently.
 *
 *****************************************************************************/

int select(long nfds, TICC3000fd_set *readsds, TICC3000fd_set *writesds,
           TICC3000fd_set *exceptsds, struct timeval *timeout)
{
  uint8_t *ptr, *args;
  tBsdSelectRecvParams tParams;
  unsigned long is_blocking;

  if (timeout == NULL)
    {
      is_blocking = 1; /* blocking , infinity timeout */
    }
  else
    {
      is_blocking = 0; /* no blocking, timeout */
    }

  /* Fill in HCI packet structure */

  ptr = tSLInformation.pucTxCommandBuffer;
  args = (ptr + HEADERS_SIZE_CMD);

  /* Fill in temporary command buffer */

  args = UINT32_TO_STREAM(args, nfds);
  args = UINT32_TO_STREAM(args, 0x00000014);
  args = UINT32_TO_STREAM(args, 0x00000014);
  args = UINT32_TO_STREAM(args, 0x00000014);
  args = UINT32_TO_STREAM(args, 0x00000014);
  args = UINT32_TO_STREAM(args, is_blocking);
  args = UINT32_TO_STREAM(args, ((readsds) ? *(unsigned long*)readsds : 0));
  args = UINT32_TO_STREAM(args, ((writesds) ? *(unsigned long*)writesds : 0));
  args = UINT32_TO_STREAM(args, ((exceptsds) ? *(unsigned long*)exceptsds : 0));

  if (timeout)
    {
      if (0 == timeout->tv_sec && timeout->tv_usec <
          SELECT_TIMEOUT_MIN_MICRO_SECONDS)
        {
          timeout->tv_usec = SELECT_TIMEOUT_MIN_MICRO_SECONDS;
        }

      args = UINT32_TO_STREAM(args, timeout->tv_sec);
      args = UINT32_TO_STREAM(args, timeout->tv_usec);
   }

  /* Initiate a HCI command */

  hci_command_send(HCI_CMND_BSD_SELECT, ptr, SOCKET_SELECT_PARAMS_LEN);

  /* Since we are in blocking state - wait for event complete */

  SimpleLinkWaitEvent(HCI_EVNT_SELECT, &tParams);

  /* Update actually read FD */

  if (tParams.iStatus >= 0)
    {
      if (readsds)
        {
          memcpy(readsds, &tParams.uiRdfd, sizeof(tParams.uiRdfd));
        }

      if (writesds)
        {
          memcpy(writesds, &tParams.uiWrfd, sizeof(tParams.uiWrfd));
        }

      if (exceptsds)
        {
          memcpy(exceptsds, &tParams.uiExfd, sizeof(tParams.uiExfd));
        }

      return tParams.iStatus;
    }
  else
    {
      errno = tParams.iStatus;
      return -1;
    }
}

/*****************************************************************************
 * Name: setsockopt
 *
 * Decription:
 *   set socket options
 *   This function manipulate the options associated with a socket.
 *   Options may exist at multiple protocol levels; they are always
 *   present at the uppermost socket level.
 *   When manipulating socket options the level at which the option
 *   resides and the name of the option must be specified.
 *   To manipulate options at the socket level, level is specified as
 *   SOL_SOCKET. To manipulate options at any other level the protocol
 *   number of the appropriate protocol controlling the option is
 *   supplied. For example, to indicate that an option is to be
 *   interpreted by the TCP protocol, level should be set to the
 *   protocol number of TCP;
 *   The parameters optval and optlen are used to access optval -
 *   use for setsockopt(). For getsockopt() they identify a buffer
 *   in which the value for the requested option(s) are to
 *   be returned. For getsockopt(), optlen is a value-result
 *   parameter, initially containing the size of the buffer
 *   pointed to by option_value, and modified on return to
 *   indicate the actual size of the value returned. If no option
 *   value is to be supplied or returned, option_value may be NULL.
 *
 * NOTE: On this version the following two socket options are enabled:
 *    The only protocol level supported in this version
 *   is SOL_SOCKET (level).
 *    1. SOCKOPT_RECV_TIMEOUT (optname)
 *     SOCKOPT_RECV_TIMEOUT configures recv and recvfrom timeout
 *    in milliseconds.
 *     In that case optval should be pointer to unsigned long.
 *    2. SOCKOPT_NONBLOCK (optname). sets the socket non-blocking mode on
 *    or off.
 *     In that case optval should be SOCK_ON or SOCK_OFF (optval).
 *
 * Input Parameters:
 *   sd          socket handle
 *   level       defines the protocol level for this option
 *   optname     defines the option name to Interrogate
 *   optval      specifies a value for the option
 *   optlen      specifies the length of the option value
 *
 * Returned Value:
 *   On success, zero is returned. On error, -1 is returned
 *
 *****************************************************************************/

#ifndef CC3000_TINY_DRIVER
int setsockopt(long sd, long level, long optname, const void *optval, socklen_t optlen)
{
  int ret;
  uint8_t *ptr, *args;

  ptr = tSLInformation.pucTxCommandBuffer;
  args = (ptr + HEADERS_SIZE_CMD);

  /* Fill in temporary command buffer */

  args = UINT32_TO_STREAM(args, sd);
  args = UINT32_TO_STREAM(args, level);
  args = UINT32_TO_STREAM(args, optname);
  args = UINT32_TO_STREAM(args, 0x00000008);
  args = UINT32_TO_STREAM(args, optlen);
  ARRAY_TO_STREAM(args, ((uint8_t *)optval), optlen);

  /* Initiate a HCI command */

  hci_command_send(HCI_CMND_SETSOCKOPT,
                   ptr, SOCKET_SET_SOCK_OPT_PARAMS_LEN  + optlen);

  /* Since we are in blocking state - wait for event complete */

  SimpleLinkWaitEvent(HCI_CMND_SETSOCKOPT, &ret);

  if (ret >= 0)
    {
      return 0;
    }
  else
    {
      errno = ret;
      return ret;
    }
}
#endif

/*****************************************************************************
 * Name: getsockopt
 *
 * Decription:
 *   set socket options
 *   This function manipulate the options associated with a socket.
 *   Options may exist at multiple protocol levels; they are always
 *   present at the uppermost socket level.
 *   When manipulating socket options the level at which the option
 *   resides and the name of the option must be specified.
 *   To manipulate options at the socket level, level is specified as
 *   SOL_SOCKET. To manipulate options at any other level the protocol
 *   number of the appropriate protocol controlling the option is
 *   supplied. For example, to indicate that an option is to be
 *   interpreted by the TCP protocol, level should be set to the
 *   protocol number of TCP;
 *   The parameters optval and optlen are used to access optval -
 *   use for setsockopt(). For getsockopt() they identify a buffer
 *   in which the value for the requested option(s) are to
 *   be returned. For getsockopt(), optlen is a value-result
 *   parameter, initially containing the size of the buffer
 *   pointed to by option_value, and modified on return to
 *   indicate the actual size of the value returned. If no option
 *   value is to be supplied or returned, option_value may be NULL.
 *
 * NOTE: On this version the following two socket options are enabled:
 *    The only protocol level supported in this version
 *   is SOL_SOCKET (level).
 *    1. SOCKOPT_RECV_TIMEOUT (optname)
 *     SOCKOPT_RECV_TIMEOUT configures recv and recvfrom timeout
 *    in milliseconds.
 *     In that case optval should be pointer to unsigned long.
 *    2. SOCKOPT_NONBLOCK (optname). sets the socket non-blocking mode on
 *    or off.
 *     In that case optval should be SOCK_ON or SOCK_OFF (optval).
 *
 * Input Parameters:
 *   sd          socket handle
 *   level       defines the protocol level for this option
 *   optname     defines the option name to Interrogate
 *   optval      specifies a value for the option
 *   optlen      specifies the length of the option value
 *
 * Returned Value:
 *   On success, zero is returned. On error, -1 is returned
 *
 *****************************************************************************/

int getsockopt (long sd, long level, long optname, void *optval, socklen_t *optlen)
{
  uint8_t *ptr, *args;
  tBsdGetSockOptReturnParams  tRetParams;

  ptr = tSLInformation.pucTxCommandBuffer;
  args = (ptr + HEADERS_SIZE_CMD);

  /* Fill in temporary command buffer */

  args = UINT32_TO_STREAM(args, sd);
  args = UINT32_TO_STREAM(args, level);
  args = UINT32_TO_STREAM(args, optname);

  /* Initiate a HCI command */

  hci_command_send(HCI_CMND_GETSOCKOPT,
                   ptr, SOCKET_GET_SOCK_OPT_PARAMS_LEN);

  /* Since we are in blocking state - wait for event complete */

  SimpleLinkWaitEvent(HCI_CMND_GETSOCKOPT, &tRetParams);

  if (((int8_t)tRetParams.iStatus) >= 0)
    {
      *optlen = 4;
      memcpy(optval, tRetParams.ucOptValue, 4);
      return 0;
    }
  else
    {
      errno = tRetParams.iStatus;
      return errno;
    }
}

/*****************************************************************************
 * Name: simple_link_recv
 *
 * Input Parameters:
 *   sd       socket handle
 *   buf      read buffer
 *   len      buffer length
 *   flags    indicates blocking or non-blocking operation
 *   from     pointer to an address structure indicating source address
 *   fromlen  source address structure size
 *
 * Returned Value:
 *     Return the number of bytes received, or -1 if an error
 *     occurred
 *
 * Decription:
 *     Read data from socket
 *     Return the length of the message on successful completion.
 *     If a message is too long to fit in the supplied buffer,
 *     excess bytes may be discarded depending on the type of
 *     socket the message is received from
 *
 *****************************************************************************/

int simple_link_recv(long sd, void *buf, long len, long flags, sockaddr *from,
                     socklen_t *fromlen, long opcode)
{
  uint8_t *ptr, *args;
  tBsdReadReturnParams tSocketReadEvent;

  ptr = tSLInformation.pucTxCommandBuffer;
  args = (ptr + HEADERS_SIZE_CMD);

  /* Fill in HCI packet structure */

  args = UINT32_TO_STREAM(args, sd);
  args = UINT32_TO_STREAM(args, len);
  args = UINT32_TO_STREAM(args, flags);

  /* Generate the read command, and wait for the */

  hci_command_send(opcode,  ptr, SOCKET_RECV_FROM_PARAMS_LEN);

  /* Since we are in blocking state - wait for event complete */

  SimpleLinkWaitEvent(opcode, &tSocketReadEvent);

  /* In case the number of bytes is more then zero - read data */

  if (tSocketReadEvent.iNumberOfBytes > 0)
    {
      /* Wait for the data in a synchronous way. Here we assume that the bug is
       * big enough to store also parameters of receive from too....
       */

      SimpleLinkWaitData((uint8_t *)buf, (uint8_t *)from, (uint8_t *)fromlen);
    }

  errno = tSocketReadEvent.iNumberOfBytes;

  return tSocketReadEvent.iNumberOfBytes;
}

/*****************************************************************************
 * Name: recv
 *
 * Decription:
 *     function receives a message from a connection-mode socket
 *
 * NOTE: On this version, only blocking mode is supported.
 *
 * Input Parameters:
 *   sd     socket handle
 *   buf    Points to the buffer where the message should be stored
 *   len    Specifies the length in bytes of the buffer pointed to
 *     by the buffer argument.
 *   flags   Specifies the type of message reception.
 *     On this version, this parameter is not supported.
 *
 * Returned Value:
 *     Return the number of bytes received, or -1 if an error
 *     occurred
 *
 *****************************************************************************/

int recv(long sd, void *buf, long len, long flags)
{
  return(simple_link_recv(sd, buf, len, flags, NULL, NULL, HCI_CMND_RECV));
}

/*****************************************************************************
 * Name: recvfrom
 *
 * Decription:
 *    read data from socket
 *    function receives a message from a connection-mode or
 *    connectionless-mode socket. Note that raw sockets are not
 *    supported.
 *
 * NOTE: On this version, only blocking mode is supported.
 *
 * Input Parameters:
 *   sd     socket handle
 *   buf    Points to the buffer where the message should be stored
 *   len    Specifies the length in bytes of the buffer pointed to
 *     by the buffer argument.
 *   flags   Specifies the type of message reception.
 *     On this version, this parameter is not supported.
 *   from   pointer to an address structure indicating the source
 *    address: sockaddr. On this version only AF_INET is
 *    supported.
 *   fromlen   source address tructure size
 *
 * Returned Value:
 *     Return the number of bytes received, or -1 if an error
 *     occurred
 *
 *****************************************************************************/

int recvfrom(long sd, void *buf, long len, long flags, sockaddr *from,
             socklen_t *fromlen)
{
  return(simple_link_recv(sd, buf, len, flags, from, fromlen,
                          HCI_CMND_RECVFROM));
}

/*****************************************************************************
 * Name: simple_link_send
 *
 * Input Parameters:
 *   sd       socket handle
 *   buf      write buffer
 *   len      buffer length
 *   flags    On this version, this parameter is not supported
 *   to       pointer to an address structure indicating destination
 *     address
 *   tolen    destination address structure size
 *
 * Returned Value:
 *     Return the number of bytes transmitted, or -1 if an error
 *     occurred, or -2 in case there are no free buffers available
 *    (only when SEND_NON_BLOCKING is enabled)
 *
 * Decription:
 *     This function is used to transmit a message to another
 *     socket
 *
 *****************************************************************************/

int simple_link_send(long sd, const void *buf, long len, long flags,
                     const sockaddr *to, long tolen, long opcode)
{
  tBsdReadReturnParams tSocketSendEvent;
  uint8_t uArgSize = 0,  addrlen;
  uint8_t *ptr, *pDataPtr = NULL, *args;
  unsigned long addr_offset = 0;
  int res;

  /* Check the bsd_arguments */

  if (0 != (res = HostFlowControlConsumeBuff(sd)))
    {
      return res;
    }

  /* Update the number of sent packets */

  tSLInformation.NumberOfSentPackets++;

  /* Allocate a buffer and construct a packet and send it over spi */

  ptr = tSLInformation.pucTxCommandBuffer;
  args = (ptr + HEADERS_SIZE_DATA);

  /* Update the offset of data and parameters according to the command */

  switch(opcode)
    {
    case HCI_CMND_SENDTO:
      {
        addr_offset = len + sizeof(len) + sizeof(len);
        addrlen = 8;
        uArgSize = SOCKET_SENDTO_PARAMS_LEN;
        pDataPtr = ptr + HEADERS_SIZE_DATA + SOCKET_SENDTO_PARAMS_LEN;
        break;
      }

    case HCI_CMND_SEND:
      {
        tolen = 0;
        to = NULL;
        uArgSize = HCI_CMND_SEND_ARG_LENGTH;
        pDataPtr = ptr + HEADERS_SIZE_DATA + HCI_CMND_SEND_ARG_LENGTH;
        break;
      }

    default:
      {
        break;
      }
  }

  /* Fill in temporary command buffer */

  args = UINT32_TO_STREAM(args, sd);
  args = UINT32_TO_STREAM(args, uArgSize - sizeof(sd));
  args = UINT32_TO_STREAM(args, len);
  args = UINT32_TO_STREAM(args, flags);

  if (opcode == HCI_CMND_SENDTO)
    {
      args = UINT32_TO_STREAM(args, addr_offset);
      args = UINT32_TO_STREAM(args, addrlen);
    }

  /* Copy the data received from user into the TX Buffer */

  ARRAY_TO_STREAM(pDataPtr, ((uint8_t *)buf), len);

  /* In case we are using SendTo, copy the to parameters */

  if (opcode == HCI_CMND_SENDTO)
    {
      ARRAY_TO_STREAM(pDataPtr, ((uint8_t *)to), tolen);
    }

  /* Initiate a HCI command */

  hci_data_send(opcode, ptr, uArgSize, len,(uint8_t*)to, tolen);

  if (opcode == HCI_CMND_SENDTO)
    {
      SimpleLinkWaitEvent(HCI_EVNT_SENDTO, &tSocketSendEvent);
    }
  else
    {
      SimpleLinkWaitEvent(HCI_EVNT_SEND, &tSocketSendEvent);
    }

  return len;
}

/*****************************************************************************
 * Name: send
 *
 * Decription:
 *     Write data to TCP socket
 *     This function is used to transmit a message to another
 *     socket.
 *
 * NOTE: On this version, only blocking mode is supported.
 *
 * Input Parameters:
 *   sd       socket handle
 *   buf      Points to a buffer containing the message to be sent
 *   len      message size in bytes
 *   flags    On this version, this parameter is not supported
 *
 * Returned Value:
 *     Return the number of bytes transmitted, or -1 if an
 *     error occurred
 *
 *****************************************************************************/

int send(long sd, const void *buf, long len, long flags)
{
  return(simple_link_send(sd, buf, len, flags, NULL, 0, HCI_CMND_SEND));
}

/*****************************************************************************
 * Name: sendto
 *
 * Decription:
 *     Write data to TCP socket
 *     This function is used to transmit a message to another
 *     socket.
 *
 * NOTE: On this version, only blocking mode is supported.
 *
 * Input Parameters:
 *   sd       socket handle
 *   buf      Points to a buffer containing the message to be sent
 *   len      message size in bytes
 *   flags    On this version, this parameter is not supported
 *   to       pointer to an address structure indicating the destination
 *     address: sockaddr. On this version only AF_INET is
 *     supported.
 *   tolen    destination address structure size
 *
 * Returned Value:
 *     Return the number of bytes transmitted, or -1 if an
 *     error occurred
 *
 *****************************************************************************/

int sendto(long sd, const void *buf, long len, long flags, const sockaddr *to,
           socklen_t tolen)
{
  return(simple_link_send(sd, buf, len, flags, to, tolen, HCI_CMND_SENDTO));
}

/*****************************************************************************
 * Name: mdnsAdvertiser
 *
 * Decription:
 *     Set CC3000 in mDNS advertiser mode in order to advertise itself.
 *
 * Input Parameters:
 *   mdnsEnabled             flag to enable/disable the mDNS feature
 *   deviceServiceName       Service name as part of the published
 *                           canonical domain name
 *   deviceServiceNameLength Length of the service name
 *
 * Returned Value:
 *   On success, zero is returned, return SOC_ERROR if socket was not
 *   opened successfully, or if an error occurred.
 *
 *****************************************************************************/

int mdnsAdvertiser(uint16_t mdnsEnabled, char * deviceServiceName,
                   uint16_t deviceServiceNameLength)
{
  uint8_t *pTxBuffer;
  uint8_t *pArgs;
  int ret;

  if (deviceServiceNameLength > MDNS_DEVICE_SERVICE_MAX_LENGTH)
    {
      return EFAIL;
    }

  pTxBuffer = tSLInformation.pucTxCommandBuffer;
  pArgs = (pTxBuffer + SIMPLE_LINK_HCI_CMND_TRANSPORT_HEADER_SIZE);

  /* Fill in HCI packet structure */

  pArgs = UINT32_TO_STREAM(pArgs, mdnsEnabled);
  pArgs = UINT32_TO_STREAM(pArgs, 8);
  pArgs = UINT32_TO_STREAM(pArgs, deviceServiceNameLength);
  ARRAY_TO_STREAM(pArgs, deviceServiceName, deviceServiceNameLength);

  /* Initiate a HCI command */

  hci_command_send(HCI_CMND_MDNS_ADVERTISE, pTxBuffer, SOCKET_MDNS_ADVERTISE_PARAMS_LEN + deviceServiceNameLength);

  /* Since we are in blocking state - wait for event complete */

  SimpleLinkWaitEvent(HCI_EVNT_MDNS_ADVERTISE, &ret);

  return ret;
}
