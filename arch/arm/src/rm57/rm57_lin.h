/****************************************************************************
 * arch/arm/src/rm57/rm57_lin.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

/* Foundation for LIN 2.x mode operation of the SCI1/LIN1 and SCI2/LIN2
 * dual-role modules (TRM SPNU562A section 28.3). This is a scaffold, not
 * a complete LIN stack: it covers module configuration (28.3.4) and
 * simple polled master header+response transfer / slave response
 * reception. Explicitly NOT implemented here (left for a later phase):
 *
 *   - Interrupt-driven operation (LIN uses the same INTVECT mechanism as
 *     SCI - see SCI_INTVECT_ID/ISFE/PBE/CE/NRE/TOAWUS/TOA3WUS/TIMEOUT in
 *     hardware/rm57_sci.h - but no ISR path is wired up here).
 *   - Sleep/wakeup handling (GCR2 POWERDOWN/GENWU).
 *   - Adaptive baud rate detection (GCR1 ADAPT, MBRS programming).
 *   - LIN DMA (TRM 28.3.3 reuses the SCI DMA interface, but is not
 *     connected to the rm57_dma.c driver here).
 *   - Any character-device/upper-half integration - this is a bare
 *     arch-level API, not a NuttX driver.
 *   - Full slave ID filtering: rm57_lin_recv() blocks for the *next*
 *     matching header, but frame-level extended features (timeout
 *     control, TED, message filtering per 28.3.1.6-9) are not modeled.
 *
 * A LIN module (LIN1 or LIN2) cannot simultaneously be used in SCI mode
 * (CONFIG_RM57_SCI1/RM57_SCI2) and LIN mode - that is a runtime
 * configuration choice made once via rm57_lin_initialize(), not enforced
 * at build time, since a board could use SCI1 in SCI mode and SCI2/LIN2
 * in LIN mode simultaneously.
 */

#ifndef __ARCH_ARM_SRC_RM57_RM57_LIN_H
#define __ARCH_ARM_SRC_RM57_RM57_LIN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Configuration of a LIN interface. See TRM 28.3.4 for the fields this
 * maps to.
 */

struct rm57_lin_config_s
{
  uint32_t baud;       /* LIN bit rate (LIN 2.x: up to 20 kbit/s) */
  bool     master;     /* true: master node (GCR1 CLOCK=1)
                         * false: slave node (GCR1 CLOCK=0) */
  bool     classic_cs; /* true: classic (SCI-style) checksum
                         * false: enhanced checksum (GCR1 CTYPE=1) */
  bool     hgen;       /* true: hardware header generation/ID
                         * filtering (GCR1 HGEN_CTRL=1) - TRM 28.7.27
                         * recommends this be set, with RX/TX ID masks
                         * both 0xff, for compatibility with future LIN
                         * modules */
  uint8_t  idmask_tx;  /* LINMASK TX ID mask (0xff = compare all bits) */
  uint8_t  idmask_rx;  /* LINMASK RX ID mask (0xff = compare all bits) */
  uint8_t  length;     /* Response length in bytes, 1-8 */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#if defined(__cplusplus)
extern "C"
{
#endif

/****************************************************************************
 * Name: rm57_lin_initialize
 *
 * Description:
 *   Configure the LIN1 (intf=1) or LIN2 (intf=2) module for LIN mode
 *   operation per TRM 28.3.4: module reset, frame format, RX/TX pin
 *   function, baud rate, ID masks, then release from software reset.
 *
 * Returned Value:
 *   OK on success; -EINVAL for a bad interface number or configuration.
 *
 ****************************************************************************/

int rm57_lin_initialize(int intf, const struct rm57_lin_config_s *config);

/****************************************************************************
 * Name: rm57_lin_uninitialize
 *
 * Description:
 *   Hold the given LIN module in reset (GCR1 SWnRST=0), disabling it.
 *
 ****************************************************************************/

void rm57_lin_uninitialize(int intf);

/****************************************************************************
 * Name: rm57_lin_send
 *
 * Description:
 *   Master-mode transfer: write the response data and hardware checksum
 *   configuration, then write the ID byte to trigger header (and,
 *   because HW header generation is expected, response) transmission.
 *   Polls (does not block on interrupts) until the transfer completes.
 *
 * Input Parameters:
 *   intf - 1 (LIN1) or 2 (LIN2)
 *   id   - LIN frame identifier (6 data bits + 2 parity bits if HGEN is
 *          not used; write the raw ID byte otherwise, per TRM 28.7.27)
 *   data - Response data bytes to transmit
 *   len  - Number of bytes in 'data', 1-8
 *
 * Returned Value:
 *   OK on success; -EINVAL for a bad interface/length; -ETIMEDOUT if the
 *   transfer does not complete.
 *
 ****************************************************************************/

int rm57_lin_send(int intf, uint8_t id, const uint8_t *data, size_t len);

/****************************************************************************
 * Name: rm57_lin_recv
 *
 * Description:
 *   Blocks (polling FLR, not interrupt-driven) for the next LIN response
 *   whose identifier matches this node's RX ID mask, then reads it into
 *   'data' along with a hardware checksum comparison.
 *
 * Input Parameters:
 *   intf - 1 (LIN1) or 2 (LIN2)
 *   id   - Set to the identifier of the received frame
 *   data - Buffer to receive the response data bytes
 *   len  - Number of bytes to read into 'data', 1-8
 *
 * Returned Value:
 *   OK on success; -EINVAL for a bad interface/length; -ETIMEDOUT if no
 *   matching frame arrives; -EIO on checksum/frame/bus error (see
 *   SCI_FLR_CE/PBE/ISFE/NRE/BE in hardware/rm57_sci.h).
 *
 ****************************************************************************/

int rm57_lin_recv(int intf, uint8_t *id, uint8_t *data, size_t len);

#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RM57_RM57_LIN_H */
