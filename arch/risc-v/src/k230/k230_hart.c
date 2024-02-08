/****************************************************************************
 * arch/risc-v/src/k230/k230_hart.c
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
#include <nuttx/irq.h>

#include "riscv_internal.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_K230_PBMT_THEAD
#define THEAD_PBMT 0x638000
#define XSTATUS    THEAD_PBMT
#else
#define RISCV_PBMT 0x438000
#define XSTATUS    RISCV_PBMT
#endif
#define XSTATE_MSK ((1 << 30) - 1)

#define MCOR       0x70013
#define MHCR       0x11ff
#define MHINT      0x6e30c
#define MCCR2      0xe0000009
#define MHINT_BIG  0x16e30c
#define MCCR2_BIG  0xe0410009
#define MSMPR      1

#define MHCR_IE_MSK    (1 << 0)
#define MHCR_DE_MSK    (1 << 1)

#define RISCV_PMBT_EN  (1 << 62)

#define K230_DAT_SYNC_B   ".long 0x0ff0000f\n"
#define K230_INS_SYNC_B   ".long 0x0000100f\n .long 0x0220000f\n"
#define K230_I_IALL       ".long 0x0100000b\n"
#define K230_D_CIALL      ".long 0x0030000b\n"
#define K230_L2_IALL      ".long 0x0170000b\n"
#define K230_SYNC_IS      ".long 0x01b0000b\n"

#define ASM               __asm__ __volatile__

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if !defined(CONFIG_BUILD_KERNEL) || defined(CONFIG_NUTTSBI)

/****************************************************************************
 * Name: hart_cleanup
 * Description: from cleanup_before_linux() in K230 U-Boot
 ****************************************************************************/

static void k230_hart_cleanup(void)
{
  uintptr_t reg = READ_CSR(CSR_MHCR);

  ASM(K230_DAT_SYNC_B);
  ASM(K230_INS_SYNC_B);
  ASM(K230_I_IALL);           /* icache.iall */
  ASM(K230_D_CIALL);          /* dcache.ciall */
  ASM(K230_DAT_SYNC_B);
  ASM(K230_INS_SYNC_B);
  reg &= ~MHCR_IE_MSK;        /* icache.disable */
  reg &= ~MHCR_DE_MSK;        /* dcache.disable */
  WRITE_CSR(CSR_MHCR, reg);
  ASM(K230_L2_IALL);          /* l2.iall */
  ASM(K230_DAT_SYNC_B);
  ASM(K230_INS_SYNC_B);
}

#endif /* !defined(CONFIG_BUILD_KERNEL) || defined(CONFIG_NUTTSBI) */

/****************************************************************************
 * Public Functions
 ****************************************************************************/
#if !defined(CONFIG_BUILD_KERNEL) || defined(CONFIG_NUTTSBI)

/****************************************************************************
 * Name: k230_hart_on_big()
 * Description: returns true if running on big core
 ****************************************************************************/

int k230_hart_is_big(void)
{
  #define MISA_VECTOR_BIT   ('V'-'A')
  #define MISA_VECOTR_MASK  ( 1 << MISA_VECTOR_BIT )

  return (READ_CSR(CSR_MISA) & MISA_VECOTR_MASK);
}

/****************************************************************************
 * Name: k230_hart_init()
 * Description: K230 M-mode HART setup following K230 SDK
 ****************************************************************************/

void k230_hart_init(void)
{
  bool big = k230_hart_is_big();

  k230_hart_cleanup();

  WRITE_CSR(CSR_MXSTATUS, XSTATUS);
  WRITE_CSR(CSR_MHCR,  MHCR);
  WRITE_CSR(CSR_MCOR,  MCOR);
  WRITE_CSR(CSR_MSMPR, MSMPR);
  WRITE_CSR(CSR_MCCR2, big ? MCCR2_BIG : MCCR2);
  WRITE_CSR(CSR_MHINT, big ? MHINT_BIG : MHINT);

#ifdef RISCV_PBMT
  SET_CSR(CSR_MENVCFG, MENVCFG_PBMT);
#endif
}

/* Hart reset control bits */

#define CORE_DONE_BIT     (1 << 12)
#define CORE_REST_BIT     (1 << 0)
#define CORE_DONE_ENW     (1 << (12 + 16))
#define CORE_REST_ENW     (1 << (0 + 16))

#define RESET_DELAY_US    200

/****************************************************************************
 * Name: k230_hart_big_stop()
 * Description: stop big core
 ****************************************************************************/

void k230_hart_big_stop(void)
{
  volatile uint32_t *rctl = (uint32_t *)K230_CPU1_RESET;

  /* 0x10001000 clear DONE */

  *rctl = CORE_DONE_BIT | CORE_DONE_ENW;
  up_udelay(RESET_DELAY_US);

  /* 0x10001 set RESET */

  *rctl = CORE_REST_BIT | CORE_REST_ENW;
  up_udelay(RESET_DELAY_US);
}

/****************************************************************************
 * Name: k230_hart_big_boot()
 * Description: start big core from given address
 ****************************************************************************/

void k230_hart_big_boot(uintptr_t addr)
{
  volatile uint32_t *bctl = (uint32_t *)K230_CPU1_BOOTA;
  volatile uint32_t *rctl = (uint32_t *)K230_CPU1_RESET;

  if (k230_hart_is_big()) return;

  /* learned from U-Boot baremetal and RTT sysctl_reset_cpu */

  if (addr) *bctl = (uint32_t)addr;

  /* 0x10001000 clear DONE */

  *rctl = CORE_DONE_BIT | CORE_DONE_ENW;
  up_udelay(RESET_DELAY_US);

  /* 0x10001 set RESET */

  *rctl = CORE_REST_BIT | CORE_REST_ENW;
  up_udelay(RESET_DELAY_US);

  /* 0x10000 clear RESET */

  *rctl = CORE_REST_ENW;
  up_udelay(RESET_DELAY_US);
}

#endif /* !defined(CONFIG_BUILD_KERNEL) || defined(CONFIG_NUTTSBI) */

#ifdef CONFIG_NUTTSBI_LATE_INIT

/****************************************************************************
 * Name: sbi_late_initialize
 * Description: K230 specific setup in M-mode.
 ****************************************************************************/

void sbi_late_initialize(void)
{
  /* delegate K230 plic enable to S-mode */

  *((volatile uint32_t *)K230_PLIC_CTRL) = 1;
  k230_hart_init();
}
#endif
