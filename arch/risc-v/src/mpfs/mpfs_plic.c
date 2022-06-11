/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_plic.c
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
 * Public Functions
 ****************************************************************************/

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
  uintptr_t iebase;
  uintptr_t hart_id = riscv_mhartid();

  if (hart_id == 0)
    {
      iebase = MPFS_PLIC_H0_MIE0;
    }
  else
    {
      iebase = MPFS_PLIC_H1_MIE0 + MPFS_PLIC_IEPRIV_OFFSET +
        (hart_id - 1) * MPFS_HART_MIE_OFFSET;
    }

  return iebase;
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
  uintptr_t claim_address;
  uintptr_t hart_id = riscv_mhartid();

  if (hart_id == 0)
    {
      claim_address = MPFS_PLIC_H0_MCLAIM;
    }
  else
    {
      claim_address = MPFS_PLIC_H1_MCLAIM + MPFS_PLIC_CLAIMPRIV_OFFSET +
        (hart_id - 1) * MPFS_PLIC_NEXTHART_OFFSET;
    }

  return claim_address;
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
  uintptr_t threshold_address;
  uintptr_t hart_id = riscv_mhartid();

  if (hart_id == 0)
    {
      threshold_address = MPFS_PLIC_H0_MTHRESHOLD;
    }
  else
    {
      threshold_address = MPFS_PLIC_H1_MTHRESHOLD +
          MPFS_PLIC_THRESHOLDPRIV_OFFSET +
          (hart_id - 1) * MPFS_PLIC_NEXTHART_OFFSET;
    }

  return threshold_address;
}
