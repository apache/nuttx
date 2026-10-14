/****************************************************************************
 * arch/arm/src/rm57/rm57_mdio.h
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

/* MDIO bus driver for the RM57L843's shared EMAC MDIO module.  This is a
 * separate peripheral from the EMAC itself (it can address up to 32 PHYs
 * and is not tied to any single EMAC channel), so it gets its own file,
 * mirroring the way this port splits DMA (rm57_dma.c) out from its
 * consumers rather than folding it into rm57_dcan.c.
 *
 * All accesses go through the single-transaction USERACCESS0 register
 * (SPNU562A Section 31.4.11): a transaction is a 32-bit write followed by
 * polling the GO bit.  Since only one hardware transaction slot (user
 * channel 0) is used by this driver, all calls are serialized by
 * g_rm57mdio_lock.
 */

#ifndef __ARCH_ARM_SRC_RM57_RM57_MDIO_H
#define __ARCH_ARM_SRC_RM57_RM57_MDIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: rm57_mdio_initialize
 *
 * Description:
 *   Enable the MDIO state machine and configure its clock divider so
 *   that mdio_freq is produced from a vclk3_freq input.  Must be called
 *   once before any other rm57_mdio_* function; safe to call again to
 *   reconfigure the divider (e.g. on ifdown/ifup).
 *
 * Input Parameters:
 *   vclk3_freq - Frequency in Hz of the VCLK3 domain feeding the MDIO
 *                clock divider (BOARD_VCLK3_FREQUENCY).
 *   mdio_freq  - Desired MDIO clock frequency in Hz (1MHz typical, 2.5MHz
 *                maximum per SPNU562A Section 31.2.1).
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

int rm57_mdio_initialize(uint32_t vclk3_freq, uint32_t mdio_freq);

/****************************************************************************
 * Name: rm57_mdio_shutdown
 *
 * Description:
 *   Disable the MDIO state machine.
 *
 ****************************************************************************/

void rm57_mdio_shutdown(void);

/****************************************************************************
 * Name: rm57_mdio_read
 *
 * Description:
 *   Read a PHY register over MDIO (clause 22 access).
 *
 * Input Parameters:
 *   phyaddr - PHY address, 0-31
 *   regaddr - PHY register address, 0-31
 *   value   - Location to store the 16-bit register value
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure (e.g. -ETIMEDOUT if
 *   the transaction never completed, -ENXIO if the PHY did not
 *   acknowledge).
 *
 ****************************************************************************/

int rm57_mdio_read(uint8_t phyaddr, uint8_t regaddr, uint16_t *value);

/****************************************************************************
 * Name: rm57_mdio_write
 *
 * Description:
 *   Write a PHY register over MDIO (clause 22 access).
 *
 ****************************************************************************/

int rm57_mdio_write(uint8_t phyaddr, uint8_t regaddr, uint16_t value);

/****************************************************************************
 * Name: rm57_mdio_alive
 *
 * Description:
 *   Return the MDIO_ALIVE register: bit n set means the MDIO state
 *   machine has detected a PHY responding at address n.  Takes up to
 *   ~50us per address to update after rm57_mdio_initialize(), and the
 *   full 32-address scan can take time - callers should not busy-poll
 *   this immediately after initialization.
 *
 ****************************************************************************/

uint32_t rm57_mdio_alive(void);

/****************************************************************************
 * Name: rm57_mdio_linkstatus
 *
 * Description:
 *   Return true if the MDIO state machine currently reports a link for
 *   the given PHY address (MDIO_LINK register).
 *
 ****************************************************************************/

bool rm57_mdio_linkstatus(uint8_t phyaddr);

/****************************************************************************
 * Name: rm57_mdio_linkint
 *
 * Description:
 *   Enable or disable the MDIO link-change interrupt (LINKINT0) for the
 *   given PHY address via USERPHYSEL0.  When enabled, a link state
 *   change on this PHY sets MDIO_LINKINTRAW bit 0, which the EMAC
 *   Control module routes to the C0_MISC_PULSE VIM channel (see
 *   RM57_REQ_EMACMISC) when EMAC_CTRL_C0MISC_LINKINT0EN is also set.
 *
 ****************************************************************************/

void rm57_mdio_linkint(uint8_t phyaddr, bool enable);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_RM57_RM57_MDIO_H */
