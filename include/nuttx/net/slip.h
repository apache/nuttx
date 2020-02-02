/****************************************************************************
 * include/nuttx/net/slip.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author : Gregory Nutt <gnutt@nuttx.org>
 *
 * Includes some definitions that a compatible with the LGPL GNU C Library
 * header file of the same name.
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

#ifndef __INCLUDE_NUTTX_NET_SLIP_H
#define __INCLUDE_NUTTX_NET_SLIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_NET_SLIP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ***********************************************************/
/* SLIP Configuration:
 *
 *   CONFIG_NET_SLIP - Enables building of the SLIP driver
 *   CONFIG_NET_SLIP_STACKSIZE - Provides the stack size for SLIP RX and TX
 *     threads.  Default: 2048
 *   CONFIG_NET_SLIP_DEFPRIO - Provides the priority for SLIP RX and TX
 *     threads.  Default 128.
 *   CONFIG_NET_NET_SLIP_PKTSIZE - Provides the size of the SLIP packet buffers.
 *     Default 296
 *
 *     The Linux slip module hard-codes its MTU size to 296 (40 bytes for the
 *     IP+TCP headers plus 256 bytes of data).  So you might as well set
 *     CONFIG_NET_SLIP_PKTSIZE to 296 as well.
 *
 *     There may be an issue with this setting, however.  I see that Linux
 *     uses a MTU of 296 and window of 256, but actually only sends 168 bytes
 *     of data: 40 + 128.  I believe that is to allow for the 2x worst cast
 *     packet expansion.  Ideally we would like to advertise the 256 MSS,
 *     but restrict transfers to 128 bytes (possibly by modifying the
 *     tcp_mss() macro).
 *
 *   CONFIG_NET_SLIP_NINTERFACES determines the number of physical interfaces
 *   that will be supported.
 */

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: slip_initialize
 *
 * Description:
 *   Instantiate a SLIP network interface.
 *
 * Input Parameters:
 *   intf    - In the case where there are multiple SLIP interfaces, this
 *             value identifies which is to be initialized. The number of
 *             possible SLIP interfaces is determined by
 *   devname - This is the path to the serial device that will support SLIP.
 *             For example, this might be "/dev/ttyS1"
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int slip_initialize(int intf, FAR const char *devname);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET_SLIP */
#endif /* __INCLUDE_NUTTX_NET_SLIP_H */
