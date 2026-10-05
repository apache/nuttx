/****************************************************************************
 * arch/arm/src/ra8m1/ra_dtc.c
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

/* Data Transfer Controller (DTC).
 *
 * The vector table is a plain array of 32-bit words in SRAM (RA8M1 User's
 * Manual, figure 17.3), one per IELSR slot (0-95), pointed to by DTCVBR
 * (and, since this port is a flat, no-TrustZone image that always uses the
 * secure register/memory aliases, DTCVBR_SEC too -- both are programmed
 * with the same address as a belt-and-braces measure: which one a given
 * vector's DTC activation actually reads depends on that IELSR slot's
 * security attribution, in ICU.ICUSARG/H/I, which this port does not
 * currently touch, so its reset state is what decides).  Each entry's bits
 * 31:2 are the start address of that vector's transfer information block
 * (struct dtc_transfer_info_s, see ra_dtc.h); bit 0 selects
 * privileged/unprivileged access -- always privileged here.
 *
 * DTC and DMAC share one module-stop bit, MSTPCRA.MSTPA22 (RA8M1 User's
 * Manual 16.8/17.10; its own SVD-derived name in ra8m1_mstp.h even says
 * "DMA Controller/Data Transfer Controller unit0 Module Stop").
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <stdint.h>
#include <string.h>

#include <arch/barriers.h>

#include <nuttx/cache.h>
#include <nuttx/irq.h>

#include "arm_internal.h"
#include "hardware/ra8m1_dtc.h"
#include "hardware/ra8m1_icu.h"
#include "hardware/ra8m1_mstp.h"
#include "ra_dtc.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* One 32-bit entry per IELSR slot.  1024-byte aligned, per manual 17.3.1;
 * this is also the alignment DTCVBR(_SEC) itself is shifted for.
 */

static uint32_t g_dtc_vectbl[RA_IRQ_NEXTINT]
  aligned_data(R_DTC_VECTBL_ALIGN);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra8m1_dtc_vectbl_entry
 ****************************************************************************/

uint32_t *ra8m1_dtc_vectbl_entry(uint32_t vectbl_base, uint8_t vector)
{
  return (uint32_t *)(vectbl_base + ((uint32_t)vector << 2));
}

/****************************************************************************
 * Name: ra8m1_dtc_vectbl_set
 ****************************************************************************/

void ra8m1_dtc_vectbl_set(uint32_t vectbl_base, uint8_t vector,
                          FAR struct dtc_transfer_info_s *info)
{
  DEBUGASSERT(((uintptr_t)info & 3) == 0);

  *ra8m1_dtc_vectbl_entry(vectbl_base, vector) =
    ((uint32_t)info) & R_DTC_VECTBL_ADDR_MASK;
}

/****************************************************************************
 * Name: ra8m1_dtc_transfer_info
 ****************************************************************************/

FAR struct dtc_transfer_info_s *
ra8m1_dtc_transfer_info(uint32_t vectbl_base, uint8_t vector)
{
  uint32_t entry = *ra8m1_dtc_vectbl_entry(vectbl_base, vector);

  return (FAR struct dtc_transfer_info_s *)(entry & R_DTC_VECTBL_ADDR_MASK);
}

/****************************************************************************
 * Name: ra_dtc_initialize
 ****************************************************************************/

void ra_dtc_initialize(void)
{
  uint32_t vectbl_addr = (uint32_t)g_dtc_vectbl;

  DEBUGASSERT((vectbl_addr & (R_DTC_VECTBL_ALIGN - 1)) == 0);

  /* Release the shared DMAC/DTC module-stop bit */

  modifyreg32(R_MSTP_MSTPCRA, R_MSTP_MSTPCRA_MSTPA22, 0);
  getreg32(R_MSTP_MSTPCRA);

  /* Every entry starts unset (0): no vector activates the DTC until
   * ra_dtc_configure() points it at a real transfer information block.
   */

  memset(g_dtc_vectbl, 0, sizeof(g_dtc_vectbl));
  up_clean_dcache(vectbl_addr, vectbl_addr + sizeof(g_dtc_vectbl));

  /* Program the vector table base.  See the file header comment on why
   * both DTCVBR and DTCVBR_SEC get the same address.
   */

  putreg32(vectbl_addr, R_DTC_DTCVBR);
  putreg32(vectbl_addr, R_DTC_DTCVBR_SEC);

  /* Leave DTCCR.RRS (read-skip) at its reset value of 0: with read-skip
   * enabled the DTC would reuse the previous transfer information instead
   * of re-reading what ra_dtc_configure() just wrote.
   */

  UP_DSB();

  putreg8(R_DTC_DTCST_DTCST, R_DTC_DTCST);
}

/****************************************************************************
 * Name: ra_dtc_configure
 ****************************************************************************/

void ra_dtc_configure(uint8_t vector, FAR struct dtc_transfer_info_s *info)
{
  uint32_t vectbl_addr = (uint32_t)g_dtc_vectbl;
  uintptr_t info_addr = (uintptr_t)info;

  DEBUGASSERT(vector < RA_IRQ_NEXTINT);

  ra8m1_dtc_vectbl_set(vectbl_addr, vector, info);

  up_clean_dcache(info_addr, info_addr + sizeof(*info));
  up_clean_dcache(vectbl_addr + ((uint32_t)vector << 2),
                  vectbl_addr + ((uint32_t)vector << 2) + sizeof(uint32_t));

  UP_DSB();
}

/****************************************************************************
 * Name: ra_dtc_enable
 ****************************************************************************/

void ra_dtc_enable(uint8_t vector)
{
  DEBUGASSERT(vector < RA_IRQ_NEXTINT);

  modifyreg32(R_ICU_IELSR(vector), 0, R_ICU_IELSR_DTCE);
}

/****************************************************************************
 * Name: ra_dtc_disable
 ****************************************************************************/

void ra_dtc_disable(uint8_t vector)
{
  DEBUGASSERT(vector < RA_IRQ_NEXTINT);

  modifyreg32(R_ICU_IELSR(vector), R_ICU_IELSR_DTCE, 0);
}
