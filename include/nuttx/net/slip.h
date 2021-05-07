/****************************************************************************
 * include/nuttx/net/slip.h
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

/* Configuration ************************************************************/

/* SLIP Configuration:
 *
 *   CONFIG_NET_SLIP - Enables building of the SLIP driver
 *   CONFIG_NET_SLIP_STACKSIZE - Provides the stack size for SLIP RX and TX
 *     threads.  Default: 2048
 *   CONFIG_NET_SLIP_DEFPRIO - Provides the priority for SLIP RX and TX
 *     threads.  Default 128.
 *   CONFIG_NET_NET_SLIP_PKTSIZE - Provides the size of the SLIP packet
 *     buffers.  Default 296
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
