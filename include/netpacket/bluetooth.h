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

#include <nuttx/wireless/bluetooth/bt_hci.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Well known addresses: */

#define BT_ADDR_ANY   {0, 0, 0, 0, 0, 0}
#define BT_ADDR_LOCAL {0, 0, 0, 0xff, 0xff, 0xff}

/* Any channel, any PSM */

#define BT_CHANNEL_ANY 0
#define BT_PSM_ANY     0

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

/* HCI socket options (SOL_HCI, see include/sys/socket.h):
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

/* L2CAP socket options (SOL_L2CAP, see include/sys/socket.h):

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

/* RFCOMM socket options (SOL_RFCOMM, see include/sys/socket.h):
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

/* SCO socket options (SOL_SCO, see include/sys/socket.h):
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

/* The CID name space for the ACL-U, ASB-C, and AMP-U logical links is as
 * follows:
 */

#define BT_CID_NULL             0x0000  /* Null identifier */
#define BT_CID_L2CAP            0x0001  /* L2CAP Signaling channel */
#define BT_CID_CONNECTIONLESS   0x0002  /* Connectionless channel */
#define BT_CID_AMP              0x0003  /* AMP Manager Protocol */
                                        /* 0x0004-0x0006 Reserved for future
                                         * use */
#define BT_CID_BREDR            0x0007  /* BR/EDR Security Manager */
                                        /* 0x0008-0x003e Reserved for future
                                         * use */
#define BT_CID_AMPTEST          0x003f  /* AMP Test Manager */
                                        /* 0x0040-0xffff Dynamically
                                         * allocated */

/* The CID name space for the LE-U logical link is as follows:
 *
 * NOTE: 0x0004, 0x0005, and 0x0006 are used internally by ATT, L2CAP, and
 * and SMP.  These are unavailable for use use in socket connection.
 */

#define BT_LE_CID_NULL          0x0000  /* Null identifier */
                                        /* 0x0001-0x0003 Reserved for future
                                         * use */
#define BT_LE_CID_ATT           0x0004  /* Attribute Protocol */
#define BT_LE_CID_L2CAP         0x0005  /* Low Energy L2CAP Signaling channel */
#define BT_LE_CID_SMP           0x0006  /* Security Manager Protocol
                                        /* 0x0007-0x001f Reserved for future
                                        /* 0x0020-0x003e Assigned Numbers
                                        /* 0x003f Reserved for future use */
                                        /* 0x0040-0x007f Dynamically allocated
                                         * using the L2CAP LE credit based
                                         * connection mechanism
                                        /* Others reserved for future use */

/* Protocol and Service Multiplexers (PSMs) */

#define BT_PSM_SDP              0x0001  /* Bluetooth Service Discovery
                                         * Protocol (SDP), Bluetooth SIG */
#define BT_PSM_RFCOMM           0x0003  /* RFCOMM with TS 07.10, Bluetooth
                                         * SIG */
#define BT_PSM_TCS_BIN          0x0005  /* Bluetooth Telephony Control
                                         * Specification / TCS Binary,
                                         * Bluetooth SIG */
#define BT_PSM_TCS_BIN_CORDLESS 0x0007  /* Bluetooth Telephony Control
                                         * Specification / TCS Binary,
                                         * Bluetooth SIG */
#define BT_PSM_BNEP             0x000f  /* Bluetooth Network Encapsulation
                                         * Protocol, Bluetooth SIG */
#define BT_PSM_HID_CTRL         0x0011  /* Human Interface Device, Bluetooth
                                         * SIG */
#define BT_PSM_HID_INT          0x0013  /* Human Interface Device, Bluetooth
                                         * SIG */
#define BT_PSM_UPnP             0x0015  /* [ESDP] , Bluetooth SIG */
#define BT_PSM_AVCTP            0x0017  /* Audio/Video Control Transport
                                         * Protocol, Bluetooth SIG */
#define BT_PSM_AVDTP            0x0019  /* Audio/Video Distribution Transport
                                         * Protocol, Bluetooth SIG */
#define BT_PSM_AVCTP_BROWSING   0x001b  /* Audio/Video Remote Control
                                         * Profile, Bluetooth SIG */
#define BT_PSM_UDI_CPLANE       0x001d  /* Unrestricted Digital Information
                                         * Profile [UDI], Bluetooth SIG */
#define BT_PSM_ATT              0x001f  /* Bluetooth Core Specification */
#define BT_PSM_3DSP             0x0021  /* 3D Synchronization Profile,
                                         * Bluetooth SIG. */
#define BT_PSM_LE_PSM_IPSP      0x0023  /* Internet Protocol Support Profile
                                         * (IPSP), Bluetooth SIG */
#define BT_PSM_OTS              0x0025  /* Object Transfer Service (OTS),
                                         * Bluetooth SIG  */

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* See include/nuttx/wireless/bluetooth/bt_hci.h for address definitions.  In
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
  sa_family_t  bt_family;  /* Must be AF_BLUETOOTH */
  bt_addr_t    bt_bdaddr;  /* 6-byte Bluetooth address */
  uint8_t      bt_channel; /* Channel identifier (CID) */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /*  __INCLUDE_NETPACKET_BLUETOOTH_H */
