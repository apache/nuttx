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

/* Well known addresses */

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
 * The 'rc' in the naming derives from RFCCOMM as used in Linux.  The
 * resemblances are superficial beyond that, however.
 */

struct sockaddr_rc_s
{
  sa_family_t  rc_family;
  bt_addr_t    rc_bdaddr;
  uint8_t      rc_channel;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /*  __INCLUDE_NETPACKET_BLUETOOTH_H */
