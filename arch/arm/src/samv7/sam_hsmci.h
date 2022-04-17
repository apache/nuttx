/****************************************************************************
 * arch/arm/src/samv7/sam_hsmci.h
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

#ifndef __ARCH_ARM_SRC_SAMV7_SAM_HSMCI_H
#define __ARCH_ARM_SRC_SAMV7_SAM_HSMCI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
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
 * Name: sdio_initialize
 *
 * Description:
 *   Initialize SDIO for operation.
 *
 * Input Parameters:
 *   slotno - Not used.
 *
 * Returned Value:
 *   A reference to an SDIO interface structure.
 *   NULL is returned on failures.
 *
 ****************************************************************************/

struct sdio_dev_s; /* See include/nuttx/sdio.h */
struct sdio_dev_s *sdio_initialize(int slotno);

/****************************************************************************
 * Name: sam_hsmci_clkdiv
 *
 * Description:
 *   Multimedia Card Interface clock (MCCK or MCI_CK) is Master Clock (MCK)
 *   divided by (2*(CLKDIV) + CLOCKODD + 2).
 *
 *     CLKFULLDIV = 2*CLKDIV + CLOCKODD;
 *     MCI_SPEED  = MCK / (CLKFULLDIV + 2)
 *     CLKFULLDIV = MCK / MCI_SPEED - 2
 *
 *     CLKDIV     = CLKFULLDIV >> 1
 *     CLOCKODD   = CLKFULLDIV & 1
 *
 *   Where CLKDIV has a range of 0-255.
 *
 *   NOTE: The primary use of this function is for cases where the clock
 *   frequencies are not known a priori and so HSMCI clock dividers must
 *   be determined dynamically.  This is the case, for example, when we
 *   execute out of SDRAM.  In that case, the clocking was set up by the
 *   bootloader that brought us into SDRAM and it is that bootloader that
 *   has configured the clocking.
 *
 * Input Parameters:
 *   target - The target SD frequency
 *
 * Returned Value:
 *   A bitset containing the CLKDIV and CLKODD bits as needed to configure
 *   the HSMCI clock output.
 *
 ****************************************************************************/

uint32_t sam_hsmci_clkdiv(uint32_t target);

/****************************************************************************
 * Name: sdio_mediachange
 *
 * Description:
 *   Called by board-specific logic -- possibly from an interrupt handler --
 *   in order to signal to the driver that a card has been inserted or
 *   removed from the slot
 *
 * Input Parameters:
 *   dev        - An instance of the SDIO driver device state structure.
 *   cardinslot - true is a card has been detected in the slot; false if a
 *                card has been removed from the slot.  Only transitions
 *                (inserted->removed or removed->inserted should be reported)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sdio_mediachange(struct sdio_dev_s *dev, bool cardinslot);

/****************************************************************************
 * Name: sdio_wrprotect
 *
 * Description:
 *   Called by board-specific logic to report if the card in the slot is
 *   mechanically write protected.
 *
 * Input Parameters:
 *   dev       - An instance of the SDIO driver device state structure.
 *   wrprotect - true is a card is writeprotected.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sdio_wrprotect(struct sdio_dev_s *dev, bool wrprotect);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_SAMV7_SAM_HSMCI_H */
