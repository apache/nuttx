/****************************************************************************
 * net/bluetooth/bluetooth.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

#ifndef _NET_BLUETOOTH_BLUETOOTH_H
#define _NET_BLUETOOTH_BLUETOOTH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <queue.h>

#include <nuttx/wireless/bluetooth/bt_hci.h>

#ifdef CONFIG_NET_BLUETOOTH

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Allocate a new Bluetooth socket data callback */

#define bluetooth_callback_alloc(dev,conn) \
  devif_callback_alloc(dev, &conn->list)
#define bluetooth_callback_free(dev,conn,cb) \
  devif_conn_callback_free(dev, cb, &conn->list)

/* Memory Pools */

#define BLUETOOTH_POOL_PREALLOCATED  0
#define BLUETOOTH_POOL_DYNAMIC       1

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Used for containing and queuing IOBs along with information about the
 * source of the frame.
 */

struct iob_s;  /* Forward reference */

struct bluetooth_container_s
{
  FAR struct bluetooth_container_s *bn_flink; /* Supports a singly linked list */
  FAR struct iob_s *bn_iob;                   /* Contained IOB */
  bt_addr_t bn_raddr;                         /* Source address of the packet */
  uint8_t bn_channel;                         /* Source channel of the packet */
  uint8_t bn_pool;                            /* See BLUETOOTH_POOL_* definitions */
};

/* Representation of a Bluetooth socket connection */

struct devif_callback_s;                      /* Forward reference */

struct bluetooth_conn_s
{
  dq_entry_t bc_node;                         /* Supports a double linked list */
  bt_addr_t bc_laddr;                         /* Locally bound / source address.
                                               * Necessary only to support multiple
                                               * Bluetooth devices */
  bt_addr_t bc_raddr;                         /* Connected remote address */
  uint8_t bc_channel;                         /* Connection channel */
  uint8_t bc_crefs;                           /* Reference counts on this instance */
#if CONFIG_NET_BLUETOOTH_BACKLOG > 0
  uint8_t bc_backlog;                         /* Number of frames in RX queue */
#endif

  /* Queue of incoming packets */

  FAR struct bluetooth_container_s *bc_rxhead;
  FAR struct bluetooth_container_s *bc_rxtail;

  /* This is a list of Bluetooth callbacks.  Each callback represents
   * a thread that is stalled, waiting for a device-specific event.
   */

  FAR struct devif_callback_s *list;
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

/* The Bluetooth socket interface */

EXTERN const struct sock_intf_s g_bluetooth_sockif;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct bluetooth_frame_meta_s;  /* Forward reference */
struct radio_driver_s;        /* Forward reference */
struct net_driver_s;          /* Forward reference */
struct socket;                /* Forward reference */
struct sockaddr;              /* Forward reference */

/****************************************************************************
 * Name: bluetooth_initialize()
 *
 * Description:
 *   Initialize the Bluetooth socket support.  Called once and only
 *   from the network initialization logic.
 *
 * Assumptions:
 *   Called early in the initialization sequence
 *
 ****************************************************************************/

void bluetooth_initialize(void);

/****************************************************************************
 * Name: bluetooth_conn_initialize
 *
 * Description:
 *   Initialize the Bluetooth connection structure allocator.  Called
 *   once and only from bluetooth_initialize().
 *
 * Assumptions:
 *   Called early in the initialization sequence
 *
 ****************************************************************************/

void bluetooth_conn_initialize(void);

/****************************************************************************
 * Name: bluetooth_conn_alloc()
 *
 * Description:
 *   Allocate a new, uninitialized Bluetooth socket connection
 *   structure. This is normally something done by the implementation of
 *   the socket() API
 *
 ****************************************************************************/

FAR struct bluetooth_conn_s *bluetooth_conn_alloc(void);

/****************************************************************************
 * Name: bluetooth_conn_free()
 *
 * Description:
 *   Free a Bluetooth socket connection structure that is no longer in
 *   use.  This should be done by the implementation of close().
 *
 ****************************************************************************/

void bluetooth_conn_free(FAR struct bluetooth_conn_s *conn);

/****************************************************************************
 * Name: bluetooth_conn_active()
 *
 * Description:
 *   Find a connection structure that is the appropriate
 *   connection to be used with the provided Ethernet header
 *
 * Assumptions:
 *   This function is called from network logic at with the network locked.
 *
 ****************************************************************************/

FAR struct bluetooth_conn_s *
  bluetooth_conn_active(FAR const struct bluetooth_frame_meta_s *meta);

/****************************************************************************
 * Name: bluetooth_conn_next()
 *
 * Description:
 *   Traverse the list of allocated Bluetooth connections
 *
 * Assumptions:
 *   This function is called from network logic at with the network locked.
 *
 ****************************************************************************/

FAR struct bluetooth_conn_s *
  bluetooth_conn_next(FAR struct bluetooth_conn_s *conn);

/****************************************************************************
 * Name: bluetooth_input
 *
 * Description:
 *   Handle incoming Bluetooth input
 *
 *   This function is called when the radio device driver has received an
 *   frame from the network.  The frame from the device driver must be
 *   provided in by the IOB frame argument of the  function call:
 *
 *   - The frame data is in the IOB io_data[] buffer,
 *   - The length of the frame is in the IOB io_len field, and
 *   - The offset past and radio MAC header is provided in the io_offset
 *     field.
 *
 *   The frame argument may refer to a single frame (a list of length one)
 *   or may it be the head of a list of multiple frames.
 *
 *   - The io_flink field points to the next frame in the list (if enable)
 *   - The last frame in the list will have io_flink == NULL.
 *
 * Input Parameters:
 *   radio       The radio network driver interface.
 *   framelist - The head of an incoming list of frames.  Normally this
 *               would be a single frame.  A list may be provided if
 *               appropriate, however.
 *   meta      - Meta data characterizing the received frame.
 *
 *               If there are multiple frames in the list, this metadata
 *               must apply to all of the frames in the list.
 *
 * Returned Value:
 *   OK    The Bluetooth has been processed  and can be deleted
 *   ERROR Hold the Bluetooth and try again later. There is a listening
 *         socket but no recv in place to catch the Bluetooth yet.
 *         Useful when a packet arrives before a recv call is in place.
 *
 * Assumptions:
 *   Called from the network diver with the network locked.
 *
 ****************************************************************************/

/* bluetooth_input() is prototyped in include/nuttx/net/bluetooth.h */

/****************************************************************************
 * Name: bluetooth_callback
 *
 * Description:
 *   Inform the application holding the Bluetooth socket of a change in state.
 *
 * Returned Value:
 *   OK if Bluetooth has been processed, otherwise ERROR.
 *
 * Assumptions:
 *   This function is called from network logic at with the network locked.
 *
 ****************************************************************************/

uint16_t bluetooth_callback(FAR struct radio_driver_s *radio,
                             FAR struct bluetooth_conn_s *conn,
                             uint16_t flags);

/****************************************************************************
 * Name: bluetooth_recvfrom
 *
 * Description:
 *   Implements the socket recvfrom interface for the case of the AF_INET
 *   and AF_INET6 address families.  bluetooth_recvfrom() receives messages from
 *   a socket, and may be used to receive data on a socket whether or not it
 *   is connection-oriented.
 *
 *   If 'from' is not NULL, and the underlying protocol provides the source
 *   address, this source address is filled in.  The argument 'fromlen' is
 *   initialized to the size of the buffer associated with from, and
 *   modified on return to indicate the actual size of the address stored
 *   there.
 *
 * Input Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   buf      Buffer to receive data
 *   len      Length of buffer
 *   flags    Receive flags
 *   from     Address of source (may be NULL)
 *   fromlen  The length of the address structure
 *
 * Returned Value:
 *   On success, returns the number of characters received.  If no data is
 *   available to be received and the peer has performed an orderly shutdown,
 *   recv() will return 0.  Otherwise, on errors, a negated errno value is
 *   returned (see recvfrom() for the list of appropriate error values).
 *
 ****************************************************************************/

ssize_t bluetooth_recvfrom(FAR struct socket *psock, FAR void *buf,
                           size_t len, int flags, FAR struct sockaddr *from,
                           FAR socklen_t *fromlen);

/****************************************************************************
 * Name: bluetooth_find_device
 *
 * Description:
 *   Select the network driver to use with the Bluetooth transaction.
 *
 * Input Parameters:
 *   conn - Bluetooth connection structure (not currently used).
 *   addr - The address to match the devices assigned address
 *
 * Returned Value:
 *   A pointer to the network driver to use.  NULL is returned on any
 *   failure.
 *
 ****************************************************************************/

FAR struct radio_driver_s *
  bluetooth_find_device(FAR struct bluetooth_conn_s *conn,
                        FAR const bt_addr_t *addr);

/****************************************************************************
 * Name: bluetooth_poll
 *
 * Description:
 *   Poll a Bluetooth "connection" structure for availability of TX data
 *
 * Input Parameters:
 *   dev - The device driver structure to use in the send operation
 *   conn - The Bluetooth "connection" to poll for TX data
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called from the network device interface (devif) with the network
 *   locked.
 *
 ****************************************************************************/

void bluetooth_poll(FAR struct net_driver_s *dev,
                     FAR struct bluetooth_conn_s *conn);

/****************************************************************************
 * Name: psock_bluetooth_sendto
 *
 * Description:
 *   If sendto() is used on a connection-mode (SOCK_STREAM, SOCK_SEQPACKET)
 *   socket, the parameters to and 'tolen' are ignored (and the error EISCONN
 *   may be returned when they are not NULL and 0), and the error ENOTCONN is
 *   returned when the socket was not actually connected.
 *
 * Input Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags
 *   to       Address of recipient
 *   tolen    The length of the address structure
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   a negated errno value is retruend.  See sendto() for the complete list
 *   of return values.
 *
 ****************************************************************************/

ssize_t psock_bluetooth_sendto(FAR struct socket *psock,
                                FAR const void *buf,
                                size_t len, int flags,
                                FAR const struct sockaddr *to, socklen_t tolen);

/****************************************************************************
 * Name: bluetooth_container_initialize
 *
 * Description:
 *   This function initializes the container allocator.  This function must
 *   be called early in the initialization sequence before any socket
 *   activity.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called early in the initialization sequence
 *
 ****************************************************************************/

void bluetooth_container_initialize(void);

/****************************************************************************
 * Name: bluetooth_container_allocate
 *
 * Description:
 *   The bluetooth_container_allocate function will get a free continer
 *   for use by the recvfrom() logic.
 *
 *   This function will first attempt to allocate from the g_free_container
 *   list.  If that the list is empty, then the meta-data structure will be
 *   allocated from the dynamic memory pool.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   A reference to the allocated container structure.  All user fields in this
 *   structure have been zeroed.  On a failure to allocate, NULL is
 *   returned.
 *
 * Assumptions:
 *   The caller has locked the network.
 *
 ****************************************************************************/

FAR struct bluetooth_container_s *bluetooth_container_allocate(void);

/****************************************************************************
 * Name: bluetooth_container_free
 *
 * Description:
 *   The bluetooth_container_free function will return a container structure
 *   to the free list of containers if it was a pre-allocated container
 *   structure. If the container structure was allocated dynamically it will
 *   be deallocated.
 *
 * Input Parameters:
 *   container - container structure to free
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The caller has locked the network.
 *
 ****************************************************************************/

void bluetooth_container_free(FAR struct bluetooth_container_s *container);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET_BLUETOOTH */
#endif /* _NET_BLUETOOTH_BLUETOOTH_H */
