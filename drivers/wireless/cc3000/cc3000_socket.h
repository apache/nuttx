/*****************************************************************************
 * drivers/wireless/cc3000_socket.h  - CC3000 Host Driver Implementation.
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

#ifndef __DRIVERS_WIRELESS_CC3000_SOCKET_H
#define __DRIVERS_WIRELESS_CC3000_SOCKET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <sys/select.h>
#include <sys/socket.h>

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

#define CC3000_HOSTNAME_MAX_LENGTH    (230)  /* 230 bytes + header shouldn't exceed 8
                                              * bit value */

/*--------- Address Families --------*/

#define  CC3000_AF_INET                2
#define  CC3000_AF_INET6               23


/*------------ Socket Types ------------*/

#define  CC3000_SOCK_STREAM     1
#define  CC3000_SOCK_DGRAM      2
#define  CC3000_SOCK_RAW        3           /* Raw sockets allow new IPv4
                                             * protocols to be implemented in
                                             * user space. A raw socket receives
                                             * or sends the raw datagram not
                                             * including link level headers */
#define  CC3000_SOCK_RDM        4
#define  CC3000_SOCK_SEQPACKET  5

/*----------- Socket Protocol ----------*/

#define CC3000_IPPROTO_IP              0                        /* Dummy for IP */
#define CC3000_IPPROTO_ICMP            1                        /* Control message protocol */
#define CC3000_IPPROTO_IPV4            CC3000_IPPROTO_IP        /* IP inside IP */
#define CC3000_IPPROTO_TCP             6                        /* TCP */
#define CC3000_IPPROTO_UDP             17                       /* User datagram protocol */
#define CC3000_IPPROTO_IPV6            41                       /* IPv6 in IPv6 */
#define CC3000_IPPROTO_NONE            59                       /* No next header */
#define CC3000_IPPROTO_TX_TEST_RAW     150                      /* Raw 802.11 Tx Test packet */
#define CC3000_IPPROTO_RAW             255                      /* Raw IP packet */
#define CC3000_IPPROTO_MAX             256

/*----------- Socket retunr codes  -----------*/

#define CC3000_SOC_ERROR               (-1)        /* Error  */
#define CC3000_SOC_IN_PROGRESS         (-2)        /* Socket in progress */

/*----------- Socket Options -----------*/
#define  CC3000_SOL_SOCKET              0xffff     /* Socket level */
#define  CC3000_SOCKOPT_RECV_NONBLOCK   0          /* recv non block mode, set SOCK_ON or
                                                    * SOCK_OFF (default block mode) */
#define  CC3000_SOCKOPT_RECV_TIMEOUT    1          /* optname to configure recv and recvfromtimeout */
#define  CC3000_SOCKOPT_ACCEPT_NONBLOCK 2          /* accept non block mode, set SOCK_ON or SOCK_OFF
                                                    * (default block mode) */
#define  CC3000_SOCK_ON                 0          /* socket non-blocking mode  is enabled */
#define  CC3000_SOCK_OFF                1          /* socket blocking mode is enabled */

#define  CC3000_TCP_NODELAY             0x0001
#define  CC3000_TCP_BSDURGENT           0x7000

#define  CC3000_MAX_PACKET_SIZE         1500
#define  CC3000_MAX_LISTEN_QUEUE        4

#define  CC3000_IOCTL_SOCKET_EVENTMASK

#define CC3000_FD_SETSIZE             32

#define  CC3000_ASIC_ADDR_LEN           8

#define CC3000_NO_QUERY_RECIVED        -3

/* It's easier to assume 8-bit bytes than to get CHAR_BIT. */

#define __NFDBITS               (8 * sizeof (__fd_mask))
#define __FDELT(d)              ((d) / __NFDBITS)
#define __FDMASK(d)             ((__fd_mask) 1 << ((d) % __NFDBITS))

#define __FDS_BITS(set)        ((set)->fds_bits)

/* We don't use `memset' because this would require a prototype and
 *   the array isn't too big.
 */

#define __FD_ZERO(set) \
  do { \
    unsigned int __i; \
    TICC3000fd_set *__arr = (set); \
    for (__i = 0; __i < sizeof (TICC3000fd_set) / sizeof (__fd_mask); ++__i) \
      __FDS_BITS (__arr)[__i] = 0; \
  } while (0)
#define __FD_SET(d, set)       (__FDS_BITS (set)[__FDELT (d)] |= __FDMASK (d))
#define __FD_CLR(d, set)       (__FDS_BITS (set)[__FDELT (d)] &= ~__FDMASK (d))
#define __FD_ISSET(d, set)     (__FDS_BITS (set)[__FDELT (d)] & __FDMASK (d))

/* Access macros for 'TICC3000fd_set' */

#define CC3000_FD_SET(fd, fdsetp)      __FD_SET (fd, fdsetp)
#define CC3000_FD_CLR(fd, fdsetp)      __FD_CLR (fd, fdsetp)
#define CC3000_FD_ISSET(fd, fdsetp)    __FD_ISSET (fd, fdsetp)
#define CC3000_FD_ZERO(fdsetp)         __FD_ZERO (fdsetp)


/* mDNS port - 5353    mDNS multicast address - 224.0.0.251 */

#define SET_mDNS_ADD(sockaddr)            sockaddr.sa_data[0] = 0x14; \
                                          sockaddr.sa_data[1] = 0xe9; \
                                          sockaddr.sa_data[2] = 0xe0; \
                                          sockaddr.sa_data[3] = 0x0; \
                                          sockaddr.sa_data[4] = 0x0; \
                                          sockaddr.sa_data[5] = 0xfb;

/*****************************************************************************
 * Public Types
 *****************************************************************************/

/* The fd_set member is required to be an array of longs. */

typedef long int __fd_mask;

/* fd_set for select and pselect. */

typedef struct
{
  __fd_mask fds_bits[CC3000_FD_SETSIZE / __NFDBITS];
} TICC3000fd_set;

/*****************************************************************************
 * Public Data
 *****************************************************************************/

#ifdef  __cplusplus
extern "C" {
#endif

/*****************************************************************************
 * Public Function Prototypes
 *****************************************************************************/

/*****************************************************************************
 * Name: cc3000_socket_impl
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

int cc3000_socket_impl(long domain, long type, long protocol);

/*****************************************************************************
 * Name: cc3000_closesocket_impl
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

long cc3000_closesocket_impl(long sd);

/*****************************************************************************
 * Name: cc3000_accept_impl
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

long cc3000_accept_impl(long sd, struct sockaddr *addr, socklen_t *addrlen);

/*****************************************************************************
 * Name: cc3000_bind_impl
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

long cc3000_bind_impl(long sd, FAR const struct sockaddr *addr, socklen_t addrlen);

/*****************************************************************************
 * Name: cc3000_listen_impl
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
 *   sd  socket descriptor (handle)
 *   backlog  specifies the listen queue depth. On this version
 *    backlog is not supported.
 *
 * Returned Value:
 *   On success, zero is returned. On error, -1 is returned.
 *
 *****************************************************************************/

long cc3000_listen_impl(long sd, long backlog);

/*****************************************************************************
 * Name: cc3000_connect_impl
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

long cc3000_connect_impl(long sd, FAR const struct sockaddr *addr, socklen_t addrlen);

/*****************************************************************************
 * Name: cc3000_select_impl
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
 *   readfds    socket descriptors list for read monitoring
 *   writefds   socket descriptors list for write monitoring
 *   exceptfds  socket descriptors list for exception monitoring
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
 *   *readfds - return the sockets on which Read request will
 *     return without delay with valid data.
 *   *writefds - return the sockets on which Write request
 *     will return without delay.
 *   *exceptfds - return the sockets which closed recently.
 *
 *****************************************************************************/

int cc3000_select_impl(long nfds, TICC3000fd_set *readfds, TICC3000fd_set *writefds,TICC3000fd_set *exceptfds,
           struct timeval *timeout);

int cc3000_select(int nfds, fd_set *readfds, fd_set *writefds,fd_set *exceptfds, struct timeval *timeout);


#ifndef CC3000_TINY_DRIVER
/*****************************************************************************
 * Name: cc3000_setsockopt_impl
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

int cc3000_setsockopt_impl(long sd, long level, long optname, const void *optval, socklen_t optlen);
#endif

/*****************************************************************************
 * Name: cc3000_getsockopt_impl
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

int cc3000_getsockopt_impl(long sd, long level, long optname, void *optval, socklen_t *optlen);


/*****************************************************************************
 * Name: cc3000_recv_impl
 *
 * Decription:
 *     function receives a message from a connection-mode socket
 *
 * NOTE: On this version, only blocking mode is supported.
 *
 * Input Parameters:
 *   sd socket handle
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

int cc3000_recv_impl(long sd, void *buf, long len, long flags);

/*****************************************************************************
 * Name: cc3000_recvfrom_impl
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
 *   sd socket handle
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

int cc3000_recvfrom_impl(long sd, void *buf, long len, long flags,
                             struct sockaddr *from, socklen_t *fromlen);

/*****************************************************************************
 * Name: cc3000_send_impl
 *
 * Decription:
 *     Write data to TCP socket
 *     This function is used to transmit a message to another
 *     socket.
 *
 * NOTE: On this version, only blocking mode is supported.
 *
 * Input Parameters:
 *   sd   socket handle
 *   buf      Points to a buffer containing the message to be sent
 *   len      message size in bytes
 *   flags    On this version, this parameter is not supported
 *
 * Returned Value:
 *     Return the number of bytes transmitted, or -1 if an
 *     error occurred
 *
 *****************************************************************************/

int cc3000_send_impl(long sd, const void *buf, long len, long flags);

/*****************************************************************************
 * Name: cc3000_sendto_impl
 *
 * Decription:
 *     Write data to TCP socket
 *     This function is used to transmit a message to another
 *     socket.
 *
 * NOTE: On this version, only blocking mode is supported.
 *
 * Input Parameters:
 *   sd   socket handle
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

int cc3000_sendto_impl(long sd, FAR const void *buf, long len, long flags,
                      FAR const struct sockaddr *to, socklen_t tolen);

#ifndef CC3000_TINY_DRIVER
/*****************************************************************************
 * Name: cc3000_gethostbyname_impl
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

//struct hostent *gethostbyname(const char *name);
int cc3000_gethostbyname_impl(char * hostname, uint16_t usNameLen, unsigned long* out_ip_addr);
#endif

/*****************************************************************************
 * Name: cc3000_mdnsAdvertiser_impl
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

int cc3000_mdnsadvertiser_impl(uint16_t mdnsEnabled, char * deviceServiceName,
                   uint16_t deviceServiceNameLength);

#ifdef  __cplusplus
}
#endif // __cplusplus

#endif // __DRIVERS_WIRELESS_CC3000_SOCKET_H
