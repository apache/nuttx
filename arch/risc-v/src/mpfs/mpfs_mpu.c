/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_mpu.c
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
#include <stdbool.h>
#include <errno.h>

#include <arch/csr.h>

#include <nuttx/lib/math32.h>

#include "riscv_internal.h"
#include "mpfs_memorymap.h"

#include "hardware/mpfs_mpucfg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MPUCFG entry is 64-bits */

#define MPFS_MPUCFG_WIDTH       64

/* Mode bits [63:56] */

#define MPFS_MPUCFG_MODE_SHIFT  56
#define MPFS_MPUCFG_MODE_WIDTH  8
#define MPFS_MPUCFG_MODE_MASK   \
  (((1ul << MPFS_MPUCFG_MODE_WIDTH) - 1) << MPFS_MPUCFG_MODE_SHIFT)

/* PMP entry bits [35:0] */

#define MPFS_MPUCFG_PMP_SHIFT   0
#define MPFS_MPUCFG_PMP_WIDTH   36
#define MPFS_MPUCFG_PMP_MASK    \
  (((1ul << MPFS_MPUCFG_PMP_WIDTH) - 1) << MPFS_MPUCFG_PMP_SHIFT)

/* Encode the MPUCFG register value */

#define MPFS_MPUCFG_ENCODE(mode, napot)                          \
 ((((mode) << MPFS_MPUCFG_MODE_SHIFT) & MPFS_MPUCFG_MODE_MASK) | \
  (((napot) << MPFS_MPUCFG_PMP_SHIFT) & MPFS_MPUCFG_PMP_MASK))

/* Decode the MPUCFG register value */

#define MPFS_MPUCFG_DECODE(reg, mode, napot)                              \
  do                                                                      \
    {                                                                     \
      uintptr_t val = getreg64(reg);                                      \
      *(mode)  = (val & MPFS_MPUCFG_MODE_MASK) >> MPFS_MPUCFG_MODE_SHIFT; \
      *(napot) = (val & MPFS_MPUCFG_PMP_MASK) >> MPFS_MPUCFG_PMP_SHIFT;   \
    }                                                                     \
  while(0)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: napot_decode
 *
 * Description:
 *   Decode base and size from NAPOT value
 *
 * Input Parameters:
 *   val  - Value to decode.
 *   size - Size out.
 *
 * Returned Value:
 *   Base address.
 *
 ****************************************************************************/

static void napot_decode(uintptr_t val, uintptr_t *base, uintptr_t *size)
{
  uintptr_t mask = (uintptr_t)(-1) >> 1;
  uintptr_t pot  = MPFS_MPUCFG_WIDTH + 2;

  while (mask)
    {
      if ((val & mask) == mask)
        {
          break;
        }

      pot--;
      mask >>= 1;
    }

  *size = UINT64_C(1) << pot;
  *base = (val & ~mask) << 2;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_mpu_set
 *
 * Description:
 *   Set value to MPFS MPUCFG register.
 *
 * Input Parameters:
 *   reg  - The MPUCFG register to write.
 *   perm - The region permissions.
 *   base - The base address of the region.
 *   size - The length of the region.
 *
 * Note:
 *   Only NAPOT encoded regions are supported, thus the base address and
 *   size must align with each other.
 *
 * Returned Value:
 *   0 on success; negated error on failure.
 *
 ****************************************************************************/

int mpfs_mpu_set(uintptr_t reg, uintptr_t perm, uintptr_t base,
                 uintptr_t size)
{
  uintptr_t mode;
  uintptr_t napot;

  /* Read the the permission and napot fields */

  MPFS_MPUCFG_DECODE(reg, &mode, &napot);

  /* First, check that the register is not already configured */

  if ((mode & PMPCFG_L) != 0)
    {
      /* The entry is locked, get out */

      return -EACCES;
    }

  /* Base must be word aligned,
   * minimum size is 4K and it has to be power-of-two
   */

  if ((base & 0x07) != 0 || size < 0x1000 || (size & (size - 1)) != 0)
    {
      return -EINVAL;
    }

  /* Make sure the base + size are NAPOT encodable */

  if ((base & ((UINT64_C(1) << log2ceil(size)) - 1)) != 0)
    {
      /* The start address is not properly aligned with size */

      return -EINVAL;
    }

  /* Sanity check the register */

  if (reg < MPFS_MPUCFG_BASE || reg >= MPFS_MPUCFG_END)
    {
      return -EINVAL;
    }

  /* Calculate mode (RWX), only NAPOT encoding is supported */

  mode = (perm & (PMPCFG_RWX_MASK | PMPCFG_L)) | PMPCFG_A_NAPOT;

  /* Do the NAPOT encoding */

  napot = (base >> 2) | ((size - 1) >> 3);

  /* Then set the value */

  putreg64(MPFS_MPUCFG_ENCODE(mode, napot), reg);

  return OK;
}

/****************************************************************************
 * Name: mpfs_mpu_access_ok
 *
 * Description:
 *   Check if MPFS MPUCFG access is OK for register.
 *
 * Input Parameters:
 *   reg  - The MPUCFG register to check.
 *   perm - The region permissions.
 *   base - The base address of the region.
 *   size - The length of the region.
 *
 * Returned Value:
 *   true if access OK; false if not.
 *
 ****************************************************************************/

bool mpfs_mpu_access_ok(uintptr_t reg, uintptr_t perm, uintptr_t base,
                        uintptr_t size)
{
  uintptr_t mode;
  uintptr_t napot;
  uintptr_t reg_base;
  uintptr_t reg_size;

  /* Read the the permission and napot fields */

  MPFS_MPUCFG_DECODE(reg, &mode, &napot);

  /* Check for permission match */

  if ((mode & PMPCFG_RWX_MASK) != perm)
    {
      return false;
    }

  /* Decode the napot field */

  napot_decode(napot, &reg_base, &reg_size);

  /* Then check if the area fits */

  return (base >= reg_base && (base + size) <= (reg_base + reg_size));
}

/****************************************************************************
 * Name: mpfs_mpu_lock
 *
 * Description:
 *   Lock an MPUCFG register from further modifications.
 *
 * Input Parameters:
 *   reg  - The MPUCFG register to lock.
 *
 * Returned Value:
 *   0 on success; negated error on failure.
 *
 ****************************************************************************/

int mpfs_mpu_lock(uintptr_t reg)
{
  uintptr_t mode;
  uintptr_t napot;

  /* Sanity check the register */

  if (reg < MPFS_MPUCFG_BASE || reg >= MPFS_MPUCFG_END)
    {
      return -EINVAL;
    }

  MPFS_MPUCFG_DECODE(reg, &mode, &napot);

  /* If the entry is already locked, everything is fine */

  if ((mode & PMPCFG_L) == 0)
    {
      /* Set the lock bit and write the value back */

      putreg64(MPFS_MPUCFG_ENCODE(mode | PMPCFG_L, napot), reg);
    }

  return OK;
}
