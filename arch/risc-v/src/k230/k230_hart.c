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

/* K230 cache flush instructions */

#define K230_DAT_SYNC_B   ".long 0x0ff0000f\n"
#define K230_INS_SYNC_B   ".long 0x0000100f\n .long 0x0220000f\n"
#define K230_I_IALL       ".long 0x0100000b\n"
#define K230_D_CIALL      ".long 0x0030000b\n"
#define K230_L2_IALL      ".long 0x0170000b\n"
#define K230_SYNC_IS      ".long 0x01b0000b\n"

#define ASM               __asm__ __volatile__

/* Hart reset control bits and delays */

#define RESET_DONE_BIT     (1 << 12)
#define RESET_RQST_BIT     (1 << 0)
#define RESET_DONE_ENW     (1 << (12 + 16))
#define RESET_RQST_ENW     (1 << (0 + 16))

#define RESET_WAIT_USEC    100

/****************************************************************************
 * Private Variables
 ****************************************************************************/

#if !defined(CONFIG_BUILD_KERNEL) || defined(CONFIG_NUTTSBI)

static volatile uint64_t g_misa locate_data(".data");

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: k230_hart_init()
 * Description: K230 M-mode HART setup following K230 SDK
 ****************************************************************************/

void k230_hart_init(void)
{
  bool big;

  while (!(g_misa = READ_CSR(CSR_MISA)));
  big = g_misa & (1 << 21);

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

/****************************************************************************
 * Name: k230_hart_is_big()
 * Description: returns true if running on big core.
 ****************************************************************************/

bool k230_hart_is_big(void)
{
  return g_misa & (1 << 21);
}

/****************************************************************************
 * Name: k230_hart_big_stop()
 * Description: stop big core, can run in S-mode
 ****************************************************************************/

void k230_hart_big_stop(void)
{
  if (k230_hart_is_big()) return;

  /* 0x10001 set RESET */

  putreg32(RESET_RQST_BIT | RESET_RQST_ENW, K230_CPU1_RESET);
  up_udelay(RESET_WAIT_USEC);
  sinfo("reg: %x\n", getreg32(K230_CPU1_RESET));
}

/****************************************************************************
 * Name: k230_hart_big_boot()
 * Description: start big core from given address, can run in S-mode
 ****************************************************************************/

void k230_hart_big_boot(uintptr_t addr)
{
  if (k230_hart_is_big()) return;

  /* learned from U-Boot baremetal and RTT sysctl_reset_cpu */

  if (addr) putreg32(addr, K230_CPU1_BOOTA);
  sinfo("addr=%lx\n", addr);

  /* 0x10001000 clear DONE bit */

  putreg32(RESET_DONE_BIT | RESET_DONE_ENW, K230_CPU1_RESET);
  up_udelay(RESET_WAIT_USEC);

  /* 0x10001 set RQST bit */

  putreg32(RESET_RQST_BIT | RESET_RQST_ENW, K230_CPU1_RESET);
  up_udelay(RESET_WAIT_USEC);

  /* 0x10000 clear RQST bit */

  putreg32(RESET_RQST_ENW, K230_CPU1_RESET);
  up_udelay(RESET_WAIT_USEC);
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

  putreg32(1, K230_PLIC_CTRL);
  k230_hart_init();
}
#endif
