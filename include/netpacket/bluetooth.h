/****************************************************************************
 * include/netpacket/bluetooth.h
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

#ifndef  __INCLUDE_NETPACKET_BLUETOOTH_H
#define  __INCLUDE_NETPACKET_BLUETOOTH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <stdint.h>

#include <nuttx/wireless/bt_hci.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Well known addresses: */

#define BDADDR_ANY   {0, 0, 0, 0, 0, 0}
#define BDADDR_LOCAL {0, 0, 0, 0xff, 0xff, 0xff}

/* Socket protocols.
 *
 * All Bluetooth sockets should select address family = AF_BLUETOOTH and
 * type = SOCK_RAW.  Protocol options are listed here (from NetBSD):
 *
 * BTPROTO_HCI
 *   This gives raw access to the Host Controller Interface of local devices
 *   using the HCI protocol as described in the Bluetooth Core Specification.
 *   The local address specified by bind() may be used to select the device
 *   that the socket will receive packets from.  If BDADDR_ANY is specified
 *   then the socket will receive packets from all devices on the system.
 *   connect() may be used to create connections such that packets sent with
 *   send() will be delivered to the specified device, otherwise sendto()
 *   should be used.
 * BTPROTO_L2CAP
 *   L2CAP sockets give sequential packet access over channels to other
 *   Bluetooth devices and make use of the bt_psm field in the sockaddr_bt_s
 *   structure to select the Protocol/Sevice Multiplexer to specify when
 *   making connections.  If the special value of L2CAP_PSM_ANY is bound
 *   when the listen() call is made, the next available PSM from the
 *   dynamic range above 0x1001 will be selected and may be discovered
 *   using the getsockname() call.
 * BTPROTO_RFCOMM
 *   RFCOMM sockets provide streamed data over Bluetooth connection and
 *   make use of the bt_psm, and bt_channel fields in the sockaddr_bt_s
 *   structure.  The channel number must be between 1 and 30 inclusive
 *   except that if the special value RFCOMM_CHANNEL_ANY is bound, when
 *   the listen() call is made, the first unused channel for the relevant
 *   bdaddr will be allocated and may be discovered using the
 *   getsockname(2) call.  If no PSM is specified, a default value of
 *   L2CAP_PSM_RFCOMM (0x0003) will be used.
 *
 * NOTE:  All protocol values currently ignored.  Only BTPROTO_L2CAP is
 * supported by default.
 */

#define BTPROTO_L2CAP   0
#define BTPROTO_HCI     1
#define BTPROTO_SCO     2
#define BTPROTO_RFCOMM  3
#define BTPROTO_BNEP    4
#define BTPROTO_CMTP    5
#define BTPROTO_HIDP    6
#define BTPROTO_AVDTP   7

/* HCI socket options:
 *
 * SO_HCI_EVT_FILTER
 *   Controls which events will be received at the socket.  By default,
 *   Command_Complete and Command_Status events only are enabled.
 * SO_HCI_PKT_FILTER [struct hci_filter]
 *   This filter controls the type of packets that will be received at
 *   the socket.  By default, Event packets only are enabled.
 * SO_HCI_DIRECTION [int]
 *   When set, this enables control messages on packets received at the
 *   socket indicating the direction of travel of the packet.
 */

#define SO_HCI_EVT_FILTER   (__SO_PROTOCOL + 0)
#define SO_HCI_PKT_FILTER   (__SO_PROTOCOL + 1)
#define SO_HCI_DIRECTION    (__SO_PROTOCOL + 2)

/* L2CAP socket options:

 * SO_L2CAP_IMTU [uint16_t]
 *   Incoming MTU
 * SO_L2CAP_OMTU [uint16_t]
 *   Outgoing MTU (read-only)
 * SO_L2CAP_LM [int]
 *   Link Mode.  The following bits may be set:
 *
 *     L2CAP_LM_AUTH         Request authentication (pairing).
 *     L2CAP_LM_ENCRYPT      Request encryption (includes authentication).
 *     L2CAP_LM_SECURE       Request secured link (encryption, plus
 *                           change link key).
 *
 *   Link mode settings will be applied to the baseband link during L2CAP
 *   connection establishment.  If the L2CAP connection is already
 *   established, EINPROGRESS may be returned, and it is not possible to
 *   guarantee that data already queued (from either end) will not be
 *   delivered.  If the mode change fails, the L2CAP connection will be
 *   aborted.
 */

#define SO_L2CAP_IMTU       (__SO_PROTOCOL + 3)
#define SO_L2CAP_OMTU       (__SO_PROTOCOL + 4)
#define SO_L2CAP_LM         (__SO_PROTOCOL + 5)
#  define L2CAP_LM_AUTH     (1 << 0)
#  define L2CAP_LM_ENCRYPT  (1 << 1)
#  define L2CAP_LM_SECURE   (1 << 2)

/* RFCOMM socket options:
 *
 * SO_RFCOMM_MTU [uint16_t]
 *   Maximum Frame Size to use for this link.
 * SO_RFCOMM_LM [int]
 *   Link Mode.  The following bits may be set at any time:
 *
 *     RFCOMM_LM_AUTH        Request authentication (pairing).
 *     RFCOMM_LM_ENCRYPT     Request encryption (includes authentication).
 *     RFCOMM_LM_SECURE      Request secured link (encryption, plus
 *                           change link key).
 *
 *   Link mode settings will be applied to the baseband link during RFCOMM
 *   connection establishment.  If the RFCOMM connection is already
 *   established, EINPROGRESS may be returned, and it is not possible to
 *   guarantee that data already queued (from either end) will not be
 *   delivered.  If the mode change fails, the RFCOMM connection will be
 *   aborted.
 */

#define SO_RFCOMM_MTU       (__SO_PROTOCOL + 6)
#define SO_RFCOMM_LM        (__SO_PROTOCOL + 7)
#  define RFCOMM_LM_AUTH    (1 << 0)
#  define RFCOMM_LM_ENCRYPT (1 << 1)
#  define RFCOMM_LM_SECURE  (1 << 2)

/* SCO socket options:
 *
 * SO_SCO_MTU [uint16_t]
 *   Maximum packet size for use on this link.  This is read-only and will
 *   be set by the protocol code when a connection is made.
 * SO_SCO_HANDLE [uint16_t]
 *   Connection handle for this link.  This is read-only and provided for
 *   informational purposes only.
 */

#define SO_SCO_MTU          (__SO_PROTOCOL + 8)
#define SO_SCO_HANDLE       (__SO_PROTOCOL + 9)

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* See include/nuttx/wireless/bt_hci.h for address definitions.  In
 * particular, type bt_addr_t
 */

/* Socket address used with:
 *
 *   bind()    - Associates local address with socket
 *   connect() - Associates a remote address with the socket (for send())
 *   sendto()  - Send to specified remote address
 *   recvfrom()- Receive from indicated remote address.
 *
 * REVISIT: Some protocols would require a bt_psm field as well.
 */

struct sockaddr_bt_s
{
  sa_family_t  bt_family;
  bt_addr_t    bt_bdaddr;
  uint8_t      bt_channel;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /*  __INCLUDE_NETPACKET_BLUETOOTH_H */
