/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_entrypoints.c
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

#include <nuttx/compiler.h>

#include <sys/types.h>

/****************************************************************************
 * Extern Function Declarations
 ****************************************************************************/

extern void mpfs_opensbi_prepare_hart(void);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ENTRYPT_CNT sizeof(g_app_entrypoints) / sizeof(g_app_entrypoints[0])

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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void mpfs_jump_to_app(void) naked_function;
void mpfs_jump_to_app(void)
{
  __asm__ __volatile__
    (
      "csrr a0, mhartid\n"                   /* Hart ID */
#ifdef CONFIG_MPFS_OPENSBI
      "ld   t0, g_hart_use_sbi\n"            /* Load sbi usage bitmask */
      "srl  t0, t0, a0\n"                    /* Shift right by this hart */
      "andi t0, t0, 1\n"                     /* Check the 0 bit */
      "bgtz t0, mpfs_opensbi_prepare_hart\n" /* If bit was 1, jump to sbi */
#endif
      "slli t1, a0, 3\n"                     /* To entrypoint offset */
      "la   t0, g_app_entrypoints\n"         /* Entrypoint table base */
      "add  t0, t0, t1\n"                    /* Index in table */
      "ld   t0, 0(t0)\n"                     /* Load the address from table */
      "jr   t0\n"                            /* Jump to entrypoint */
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

#endif /* CONFIG_MPFS_BOOTLOADER */
