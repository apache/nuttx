/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_entrypoints.c
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

#ifdef CONFIG_MPFS_BOOTLOADER

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/atomic.h>
#include <nuttx/compiler.h>

#include <sys/types.h>

#include "riscv_internal.h"

/****************************************************************************
 * Extern Function Declarations
 ****************************************************************************/

extern void mpfs_opensbi_prepare_hart(void);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ENTRY_STACK 512
#define ENTRYPT_CNT sizeof(g_app_entrypoints) / sizeof(g_app_entrypoints[0])

/* Default PMP permissions */

#define PMP_DEFAULT_PERM (PMPCFG_A_NAPOT | PMPCFG_R | PMPCFG_W | PMPCFG_X)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The actual application entrypoints */

static uint64_t g_app_entrypoints[] =
{
  CONFIG_MPFS_HART0_ENTRYPOINT,
  CONFIG_MPFS_HART1_ENTRYPOINT,
  CONFIG_MPFS_HART2_ENTRYPOINT,
  CONFIG_MPFS_HART3_ENTRYPOINT,
  CONFIG_MPFS_HART4_ENTRYPOINT
};

static uint64_t g_hart_use_sbi =
#ifdef CONFIG_MPFS_HART1_SBI
  (1 << 1) |
#endif

#ifdef CONFIG_MPFS_HART2_SBI
  (1 << 2) |
#endif

#ifdef CONFIG_MPFS_HART3_SBI
  (1 << 3) |
#endif

#ifdef CONFIG_MPFS_HART4_SBI
  (1 << 4) |
#endif
  0;

#ifdef CONFIG_MPFS_BOARD_PMP
uint8_t g_mpfs_boot_stacks[ENTRY_STACK * ENTRYPT_CNT]
  aligned_data(STACK_ALIGNMENT);
#endif

static int g_cpus_booted;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void mpfs_jump_to_app(void) naked_function;
void mpfs_jump_to_app(void)
{
  __asm__ __volatile__
    (
      "csrr a0, mhartid\n"                   /* Hart ID */
#ifdef CONFIG_MPFS_BOARD_PMP
      "li   t1, %0\n"                        /* Size of hart's stack */
      "mul  t0, a0, t1\n"                    /* Hart stack base */
      "add  t0, t0, t1\n"                    /* Hart stack top */
      "la   sp, g_mpfs_boot_stacks\n"        /* Stack area base */
      "add  sp, sp, t0\n"                    /* Set stack pointer */
      "call mpfs_board_pmp_setup\n"          /* Run PMP configuration */
      "bne  a0, x0, mpfs_board_pmp_error\n"  /* If ret != 0, jump to errhan */
      "csrr a0, mhartid\n"                   /* Restore hartid */
#else
      "li   t0, -1\n"                        /* Open the whole SoC */
      "csrw pmpaddr0, t0\n"
      "li   t0, %0\n"                        /* Grant RWX permissions */
      "csrw pmpcfg0, t0\n"
      "csrw pmpcfg2, zero\n"
#endif
      "slli t1, a0, 3\n"                     /* To entrypoint offset */
      "la   t0, g_app_entrypoints\n"         /* Entrypoint table base */
      "add  t0, t0, t1\n"                    /* Index in table */
      "ld   a1, 0(t0)\n"                     /* Load the address from table */
      "li   t1, 1\n"
      "la   t2, g_cpus_booted\n"
      "ld   t0, g_hart_use_sbi\n"            /* Load sbi usage bitmask */
      "amoadd.w.aqrl zero, t1, 0(t2)\n"      /* g_cpus_booted + 1 */
#ifdef CONFIG_MPFS_OPENSBI
      "srl  t0, t0, a0\n"                    /* Shift right by this hart */
      "andi t0, t0, 1\n"                     /* Check the 0 bit */
      "beqz t0, 1f\n"                        /* If bit was 1, jump to sbi */
      "tail mpfs_opensbi_prepare_hart\n"
      "1:\n"
#endif
      "jr   a1\n"                            /* Jump to entrypoint */
      :
#ifdef CONFIG_MPFS_BOARD_PMP
      : "i" (ENTRY_STACK)
#else
      : "i" (PMP_DEFAULT_PERM)
#endif
      :
    );
}

/****************************************************************************
 * Name: mpfs_set_entrypt
 *
 * Description:
 *   Modify Hart entrypoint
 *
 * Input Parameters:
 *   hartid - Hart ID to modify
 *   entry - Entrypoint to set
 *
 * Returned value:
 *   OK on success, ERROR on failure
 *
 ****************************************************************************/

int mpfs_set_entrypt(uint64_t hartid, uintptr_t entry)
{
  if (hartid < ENTRYPT_CNT)
    {
      g_app_entrypoints[hartid] = entry;
      return OK;
    }

  return ERROR;
}

/****************************************************************************
 * Name: mpfs_get_entrypt
 *
 * Description:
 *   Obtain Hart entrypoint
 *
 * Input Parameters:
 *   hartid - Hart ID to read
 *
 * Returned value:
 *   Entrypoint on success; 0 on failure
 *
 ****************************************************************************/

uintptr_t mpfs_get_entrypt(uint64_t hartid)
{
  if (hartid < ENTRYPT_CNT)
    {
      return g_app_entrypoints[hartid];
    }

  return 0;
}

/****************************************************************************
 * Name: mpfs_set_use_sbi
 *
 * Description:
 *   Set booting via SBI.
 *
 * Input Parameters:
 *   use_sbi - set to true if sbi is needed, false otherwise
 *
 * Returned value:
 *   OK on success, ERROR on failure
 *
 ****************************************************************************/

int mpfs_set_use_sbi(uint64_t hartid, bool use_sbi)
{
  if (hartid < ENTRYPT_CNT)
    {
      if (use_sbi)
        {
          g_hart_use_sbi |= (1 << hartid);
        }
      else
        {
          g_hart_use_sbi &= ~(1 << hartid);
        }

      return OK;
    }

  return ERROR;
}

/****************************************************************************
 * Name: mpfs_get_use_sbi
 *
 * Description:
 *   Get if hart boots via SBI.
 *
 * Input Parameters:
 *   hartid - hart id to check
 *
 * Returned value:
 *   true if SBI is used, false otherwise
 *
 ****************************************************************************/

bool mpfs_get_use_sbi(uint64_t hartid)
{
  if (hartid < ENTRYPT_CNT)
    {
      return (g_hart_use_sbi & (1 << hartid)) != 0;
    }

  return false;
}

/****************************************************************************
 * Name: mpfs_cpus_booted
 *
 * Description:
 *   Get amount of CPUs that have completed boot.
 *
 * Input Parameters:
 *   None.
 *
 * Returned value:
 *   Amount of CPUs that have booted.
 *
 ****************************************************************************/

int mpfs_cpus_booted(void)
{
  return atomic_load(&g_cpus_booted);
}

#endif /* CONFIG_MPFS_BOOTLOADER */
