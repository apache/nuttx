/****************************************************************************
 * arch/arm/src/rzv2h/rzv2h_start.c
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
#include <assert.h>
#include <debug.h>

#include <arch/barriers.h>

#include "arm_internal.h"
#include "sctlr.h"
#include "gic.h"
/* #include "rzv2h_irq.h" */
#include "rzv2h_start.h"
#include "rzv2h_clockconfig.h"
#include "rzv2h_lowputc.h"
#include "arch/board/board.h"
#include "rzv2h_serial.h"
#include "rzv2h_mpu_region.h"

#ifdef CONFIG_ARCH_ARMV7R
#  include <nuttx/init.h>
#endif

#define TCM_SIZE_128K       0x20
#define TCM_ENABLE          0x1

/* Fixed low-vector exception table address.  With CONFIG_ARCH_LOWVECTORS the
 * core fetches every exception vector from here, so the table must be copied
 * to this address before any exception can be taken.
 */

#define VECTOR_ADDRESS      0x00000000

/****************************************************************************
 * Private Data
 ****************************************************************************/

extern uint8_t _vector_start[]; /* Beginning of vector block (run address) */
extern uint8_t _vector_end[];   /* End+1 of vector block (run address) */
extern uint8_t _vector_load[];  /* Load copy of the vector block in RAM */

/****************************************************************************
 * Private function
 ****************************************************************************/

/* Cortex-R8 TCM region registers.  Note the opc2 order is the opposite of
 * what the names suggest:
 *
 *   CP15 c9, c1, 0  BTCM Region Register  -> DTCM on this device
 *   CP15 c9, c1, 1  ATCM Region Register  -> ITCM on this device
 *
 * Layout, bits [31:12] base address, [6:2] size, [0] enable.
 *
 * The core-local addresses come from the HAL Renesas
 * [FSP: bsp_slave_address.h:19-25]: ITCM at 0x00000000 and DTCM at
 * 0x00020000, both 128 KB.
 *
 * These two used to be the other way round, which put the BTCM (DTCM) at
 * 0x00000000 and the ATCM (ITCM) at 0x00020000.  Data accesses to 0 still
 * worked - BTCM is real memory - so rzv2h_vectorsetup() copied the exception
 * table there without complaint and nothing failed for the whole of boot,
 * because no exception was ever taken.  The first one that was taken faulted
 * on the instruction fetch from the vector: the BTCM port does not serve
 * instruction fetches.  That prefetch abort vectors to 0x0C, which faults
 * the same way, so the core wedges on the abort vector with no way to
 * report it.
 */

inline static void rzv2h_itcmenable(void)
{
  uint32_t value = 0x00000000 | TCM_SIZE_128K | TCM_ENABLE;

  /* Enable ITCM (ATCM) at address 0x00000000, size 128KB */

  __asm__ volatile("mcr p15, 0, %0, c9, c1, 1\n" : : "r"(value));
}

inline static void rzv2h_dtcmenable(void)
{
  uint32_t value = 0x00020000 | TCM_SIZE_128K | TCM_ENABLE;

  /* Enable DTCM (BTCM) at address 0x20000, size 128KB */

  __asm__ volatile("mcr p15, 0, %0, c9, c1, 0\n" : : "r"(value));
}

/****************************************************************************
 * Name: rzv2h_vectorsetup
 *
 * Description:
 *   Publish the ARM exception table in ITCM at VECTOR_ADDRESS.
 *
 *   CONFIG_ARCH_LOWVECTORS is mandatory here, so the core fetches every
 *   exception vector from 0x00000000 and the table has to be present there
 *   before interrupts or aborts can be taken.  Both image types keep a load
 *   copy of the table in RAM at _vector_load, and this copies it to ITCM.
 *   _vector_start cannot be the source: in a U-Boot image it is the ITCM
 *   run address.  In a U-Boot image ITCM only holds the two-word reset
 *   trampoline U-Boot wrote there (see the board linker script), and this
 *   copy replaces it with the full table.
 *
 *   Must run before anything that could raise an exception.
 *
 ****************************************************************************/

static void rzv2h_vectorsetup(void)
{
  /* "base" is deliberately volatile.  VECTOR_ADDRESS is 0, and storing
   * through a pointer the compiler can prove is the literal 0 is
   * undefined behaviour: GCC assumes the path is unreachable and deletes
   * the copy loop.  That is not theoretical - it removed every
   * instruction of this function until the volatile was added.  Reading
   * the address out of a volatile local blocks the constant propagation
   * that the deletion relies on, and "dest" is volatile too so the
   * stores themselves cannot be dropped.
   */

  volatile uintptr_t base   = VECTOR_ADDRESS;
  volatile uint32_t *dest   = (volatile uint32_t *)base;
  const uint32_t    *src    = (const uint32_t *)_vector_load;
  const uint32_t    *end    = (const uint32_t *)
                              (_vector_load + (_vector_end - _vector_start));

  while (src < end)
    {
      *dest++ = *src++;
    }

  /* Publish the writes and flush the prefetch pipeline: the words just
   * written are about to be fetched as instructions.
   */

  UP_DSB();
  UP_ISB();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
#ifndef CONFIG_ARCH_LOWVECTORS
#  error CONFIG_ARCH_LOWVECTORS is required by this architecture.
#endif
/****************************************************************************
 * Name: showprogress
 *
 * Description:
 *   Print a character on the UART to show boot status.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#define showprogress(c) rzv2h_lowputc(c)
#else
#define showprogress(c)
#endif

/****************************************************************************
 * Name: arm_boot
 *
 * Description:
 *   Complete boot operations started in arm_head.S
 *
 ****************************************************************************/

void arm_boot(void)
{
  /* Map the TCMs, then get the exception table into ITCM.  Nothing above
   * this point may take an exception - there are no vectors yet.
   */

  rzv2h_itcmenable();
  rzv2h_dtcmenable();

  /* The region registers change the memory map, so the writes have to be in
   * effect before anything touches the TCMs.
   */

  UP_DSB();
  UP_ISB();

  rzv2h_vectorsetup();

  arm_fpuconfig();

#ifdef CONFIG_ARM_MPU
  /* MPU setting */

  rzv2h_mpuinit();
#endif

  /* Initialize clock */

  rzv2h_clockconfig();
  rzv2h_lowsetup();
  showprogress('A');

  showprogress('C');

  /* Perform early serial initialization */

#if defined(USE_EARLYSERIALINIT) && defined(CONFIG_RZV2H_SCI_B)
  rzv2h_earlyserialinit();
#endif
  showprogress('D');

  /* Initialize onboard resources */

  rzv2h_board_initialize();
  showprogress('E');

  /* Then start NuttX */

  showprogress('\r');
  showprogress('\n');
  nx_start();

#ifdef CONFIG_DISABLE_IDLE_LOOP
  /* Should never return */

  for (; ; );
#endif
}
