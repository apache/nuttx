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

#include <nuttx/compiler.h>

#include <sys/types.h>

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

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void jump_to_app(void) naked_function;
static void jump_to_app(void)
{
  __asm__ __volatile__
    (
      "csrr a0, mhartid\n"           /* Hart ID */
      "slli t1, a0, 3\n"             /* To entrypoint offset */
      "la   t0, g_app_entrypoints\n" /* Entrypoint table base */
      "add  t0, t0, t1\n"            /* Index in table */
      "ld   t0, 0(t0)\n"             /* Load the address from table */
      "jr   t0"                      /* Jump to entrypoint */
    );
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Default boot address for every hart */

extern void mpfs_opensbi_prepare_hart(void);

/* Trampoline functions, jump to SBI if so configured, to app if not */

const uint64_t g_entrypoints[5] =
{
#ifdef CONFIG_MPFS_HART0_SBI
  (uint64_t)mpfs_opensbi_prepare_hart,
#else
  (uint64_t)jump_to_app,
#endif

#ifdef CONFIG_MPFS_HART1_SBI
  (uint64_t)mpfs_opensbi_prepare_hart,
#else
  (uint64_t)jump_to_app,
#endif

#ifdef CONFIG_MPFS_HART2_SBI
  (uint64_t)mpfs_opensbi_prepare_hart,
#else
  (uint64_t)jump_to_app,
#endif

#ifdef CONFIG_MPFS_HART3_SBI
  (uint64_t)mpfs_opensbi_prepare_hart,
#else
  (uint64_t)jump_to_app,
#endif

#ifdef CONFIG_MPFS_HART4_SBI
  (uint64_t)mpfs_opensbi_prepare_hart,
#else
  (uint64_t)jump_to_app,
#endif
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

#endif /* CONFIG_MPFS_BOOTLOADER */
