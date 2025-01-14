/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_plic.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/irq.h>

#include "mpfs.h"
#include "mpfs_plic.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Offset to privilege mode, note that hart0 does not have S-mode */

#ifdef CONFIG_ARCH_USE_S_MODE
#  define MPFS_PLIC_IEPRIV_OFFSET         (MPFS_HART_SIE_OFFSET)
#  define MPFS_PLIC_CLAIMPRIV_OFFSET      (MPFS_PLIC_CLAIM_S_OFFSET)
#  define MPFS_PLIC_THRESHOLDPRIV_OFFSET  (MPFS_PLIC_THRESHOLD_S_OFFSET)
#else
#  define MPFS_PLIC_IEPRIV_OFFSET         (0)
#  define MPFS_PLIC_CLAIMPRIV_OFFSET      (0)
#  define MPFS_PLIC_THRESHOLDPRIV_OFFSET  (0)
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: get_iebase
 *
 * Description:
 *   Get base address for interrupt enable bits for a specific hart.
 *
 * Input Parameters:
 *   hartid - Hart ID to query.
 *
 * Returned Value:
 *   Interrupt enable base address.
 *
 ****************************************************************************/

static uintptr_t get_iebase(uintptr_t hartid)
{
  uintptr_t iebase;

  if (hartid == 0)
    {
      iebase = MPFS_PLIC_H0_MIE0;
    }
  else
    {
      iebase = MPFS_PLIC_H1_MIE0 + MPFS_PLIC_IEPRIV_OFFSET +
        (hartid - 1) * MPFS_HART_MIE_OFFSET;
    }

  return iebase;
}

/****************************************************************************
 * Name: get_claimbase
 *
 * Description:
 *   Get base address for interrupt claim for a specific hart.
 *
 * Input Parameters:
 *   hartid - Hart ID to query.
 *
 * Returned Value:
 *   Interrupt enable claim address.
 *
 ****************************************************************************/

uintptr_t get_claimbase(uintptr_t hartid)
{
  uintptr_t claim_address;

  if (hartid == 0)
    {
      claim_address = MPFS_PLIC_H0_MCLAIM;
    }
  else
    {
      claim_address = MPFS_PLIC_H1_MCLAIM + MPFS_PLIC_CLAIMPRIV_OFFSET +
        (hartid - 1) * MPFS_PLIC_NEXTHART_OFFSET;
    }

  return claim_address;
}

/****************************************************************************
 * Name: get_thresholdbase
 *
 * Description:
 *   Get base address for interrupt threshold for a specific hart.
 *
 * Input Parameters:
 *   hartid - Hart ID to query.
 *
 * Returned Value:
 *   Interrupt enable threshold address.
 *
 ****************************************************************************/

uintptr_t get_thresholdbase(uintptr_t hartid)
{
  uintptr_t threshold_address;

  if (hartid == 0)
    {
      threshold_address = MPFS_PLIC_H0_MTHRESHOLD;
    }
  else
    {
      threshold_address = MPFS_PLIC_H1_MTHRESHOLD +
          MPFS_PLIC_THRESHOLDPRIV_OFFSET +
          (hartid - 1) * MPFS_PLIC_NEXTHART_OFFSET;
    }

  return threshold_address;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_plic_init_hart
 *
 * Description:
 *   Initialize current hart's PLIC.
 *
 * Input Parameters:
 *   hartid - Hart ID to init.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void mpfs_plic_init_hart(uintptr_t hartid)
{
  /* Disable all global interrupts for current hart */

  uintptr_t iebase = get_iebase(hartid);

  putreg32(0x0, iebase + 0);
  putreg32(0x0, iebase + 4);
  putreg32(0x0, iebase + 8);
  putreg32(0x0, iebase + 12);
  putreg32(0x0, iebase + 16);
  putreg32(0x0, iebase + 20);

  /* Complete possibly claimed IRQs in PLIC (for current hart) in case
   * of warm reboot, e.g. after a crash in the middle of IRQ handler.
   * This has no effect on non-claimed or disabled interrupts.
   */

  uintptr_t claim_address = get_claimbase(hartid);

  for (int irq = MPFS_IRQ_EXT_START; irq < NR_IRQS; irq++)
    {
      putreg32(irq - MPFS_IRQ_EXT_START, claim_address);
    }

  /* Set irq threshold to 0 (permits all global interrupts) */

  uintptr_t threshold_address = get_thresholdbase(hartid);
  putreg32(0, threshold_address);
}

/****************************************************************************
 * Name: mpfs_plic_get_iebase
 *
 * Description:
 *   Context aware way to query PLIC interrupt enable base address
 *
 * Returned Value:
 *   Interrupt enable base address
 *
 ****************************************************************************/

uintptr_t mpfs_plic_get_iebase(void)
{
  return get_iebase(up_cpu_index());
}

/****************************************************************************
 * Name: mpfs_plic_get_claimbase
 *
 * Description:
 *   Context aware way to query PLIC interrupt claim base address
 *
 * Returned Value:
 *   Interrupt enable claim address
 *
 ****************************************************************************/

uintptr_t mpfs_plic_get_claimbase(void)
{
  return get_claimbase(up_cpu_index());
}

/****************************************************************************
 * Name: mpfs_plic_get_thresholdbase
 *
 * Description:
 *   Context aware way to query PLIC interrupt threshold base address
 *
 * Returned Value:
 *   Interrupt enable threshold address
 *
 ****************************************************************************/

uintptr_t mpfs_plic_get_thresholdbase(void)
{
  return get_thresholdbase(up_cpu_index());
}
