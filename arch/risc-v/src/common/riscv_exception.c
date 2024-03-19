/****************************************************************************
 * arch/risc-v/src/common/riscv_exception.c
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

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#ifdef CONFIG_PAGING
#  include <nuttx/pgalloc.h>
#endif

#ifdef CONFIG_PAGING
#  include "pgalloc.h"
#  include "riscv_mmu.h"
#endif

#include "sched/sched.h"
#include "riscv_internal.h"
#include "chip.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char *g_reasons_str[RISCV_MAX_EXCEPTION + 1] =
{
  "Instruction address misaligned",
  "Instruction access fault",
  "Illegal instruction",
  "Breakpoint",
  "Load address misaligned",
  "Load access fault",
  "Store/AMO address misaligned",
  "Store/AMO access fault",
  "Environment call from U-mode",
  "Environment call from S-mode",
  "Environment call from H-mode",
  "Environment call from M-mode",
  "Instruction page fault",
  "Load page fault",
  "Reserved",
  "Store/AMO page fault"
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: riscv_exception
 *
 * Description:
 *   This is the exception handler.
 *
 ****************************************************************************/

int riscv_exception(int mcause, void *regs, void *args)
{
#ifdef CONFIG_ARCH_KERNEL_STACK
  FAR struct tcb_s *tcb = this_task();
#endif
  uintptr_t cause = mcause & RISCV_IRQ_MASK;

  _alert("EXCEPTION: %s. MCAUSE: %" PRIxREG ", EPC: %" PRIxREG
         ", MTVAL: %" PRIxREG "\n",
         mcause > RISCV_MAX_EXCEPTION ? "Unknown" : g_reasons_str[cause],
         cause, READ_CSR(CSR_EPC), READ_CSR(CSR_TVAL));

#ifdef CONFIG_ARCH_KERNEL_STACK
  if ((tcb->flags & TCB_FLAG_TTYPE_MASK) != TCB_FLAG_TTYPE_KERNEL)
    {
      _alert("Segmentation fault in PID %d: %s\n", tcb->pid, tcb->name);

      tcb->flags |= TCB_FLAG_FORCED_CANCEL;

      /* Return to _exit function in privileged mode with argument SIGSEGV */

      CURRENT_REGS[REG_EPC] = (uintptr_t)_exit;
      CURRENT_REGS[REG_A0] = SIGSEGV;
      CURRENT_REGS[REG_INT_CTX] |= STATUS_PPP;

      /* Continue with kernel stack in use. The frame(s) in kernel stack
       * are no longer needed, so just set it to top
       */

      CURRENT_REGS[REG_SP] = (uintptr_t)tcb->xcp.ktopstk;
    }
  else
#endif
    {
      _alert("PANIC!!! Exception = %" PRIxREG "\n", cause);
      up_irq_save();
      CURRENT_REGS = regs;
      PANIC_WITH_REGS("panic", regs);
    }

  return 0;
}

/****************************************************************************
 * Name: riscv_fillpage
 *
 * Description:
 *   This function is an exception handler for page faults in a RISC-V.
 *   It is invoked when a page fault exception occurs, which is typically
 *   when a process tries to access a page that is not currently in memory.
 *
 *   The function takes as arguments the machine cause (mcause) which
 *   indicates the cause of the exception, a pointer to the register state
 *   at the time of the exception (regs), and a pointer to any additional
 *   arguments (args).
 *
 *   The function should handle the exception appropriately, typically by
 *   loading the required page into memory and updating the page table.
 *
 * Input Parameters:
 *   mcause - The machine cause of the exception.
 *   regs   - A pointer to the register state at the time of the exception.
 *   args   - A pointer to any additional arguments.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_PAGING
int riscv_fillpage(int mcause, void *regs, void *args)
{
  uintptr_t cause = mcause & RISCV_IRQ_MASK;
  uintptr_t ptlast;
  uintptr_t ptprev;
  uintptr_t paddr;
  uintptr_t vaddr;
  uint32_t  ptlevel;
  uintptr_t satp;
  uint32_t mmuflags;

  _info("EXCEPTION: %s. MCAUSE: %" PRIxREG ", EPC: %" PRIxREG
        ", MTVAL: %" PRIxREG "\n",
        mcause > RISCV_MAX_EXCEPTION ? "Unknown" : g_reasons_str[cause],
        cause, READ_CSR(CSR_EPC), READ_CSR(CSR_TVAL));
  vaddr = MM_PGALIGNDOWN(READ_CSR(CSR_TVAL));
  if (vaddr >= CONFIG_ARCH_TEXT_VBASE && vaddr <= ARCH_TEXT_VEND)
    {
      mmuflags = MMU_UTEXT_FLAGS;

      /* Write access to .text region needs to be set according to
       * https://github.com/apache/nuttx/pull/6193.
       */

      mmuflags |= PTE_W;
    }
  else if (vaddr >= CONFIG_ARCH_DATA_VBASE && vaddr <= ARCH_DATA_VEND)
    {
      mmuflags = MMU_UDATA_FLAGS;
    }
  else if (vaddr >= CONFIG_ARCH_HEAP_VBASE && vaddr <= ARCH_HEAP_VEND)
    {
      mmuflags = MMU_UDATA_FLAGS;
    }
  else
    {
      _alert("PANIC!!! virtual address not mappable: %" PRIxPTR "\n", vaddr);
      up_irq_save();
      CURRENT_REGS = regs;
      PANIC_WITH_REGS("panic", regs);
    }

  satp    = READ_CSR(CSR_SATP);
  ptprev  = riscv_pgvaddr(mmu_satp_to_paddr(satp));
  ptlevel = ARCH_SPGTS;
  paddr = mmu_pte_to_paddr(mmu_ln_getentry(ptlevel, ptprev, vaddr));
  if (!paddr)
    {
      /* Nothing yet, allocate one page for final level page table */

      paddr = mm_pgalloc(1);
      if (!paddr)
        {
          return -ENOMEM;
        }

      /* Map the page table to the prior level */

      mmu_ln_setentry(ptlevel, ptprev, paddr, vaddr, MMU_UPGT_FLAGS);

      /* This is then used to map the final level */

      riscv_pgwipe(paddr);
    }

  ptlast = riscv_pgvaddr(paddr);
  paddr = mm_pgalloc(1);
  if (!paddr)
    {
      return -ENOMEM;
    }

  /* Wipe the physical page memory */

  riscv_pgwipe(paddr);

  /* Then map the virtual address to the physical address */

  mmu_ln_setentry(ptlevel + 1, ptlast, paddr, vaddr, mmuflags);

  return 0;
}
#endif /* CONFIG_PAGING */

/****************************************************************************
 * Name: riscv_exception_attach
 *
 * Description:
 *   Attach standard exception with suitable handler
 *
 ****************************************************************************/

void riscv_exception_attach(void)
{
  irq_attach(RISCV_IRQ_IAMISALIGNED, riscv_exception, NULL);
  irq_attach(RISCV_IRQ_IAFAULT, riscv_exception, NULL);
  irq_attach(RISCV_IRQ_IINSTRUCTION, riscv_exception, NULL);
  irq_attach(RISCV_IRQ_BPOINT, riscv_exception, NULL);
  irq_attach(RISCV_IRQ_LAFAULT, riscv_exception, NULL);
  irq_attach(RISCV_IRQ_SAFAULT, riscv_exception, NULL);

#ifdef CONFIG_RISCV_MISALIGNED_HANDLER
  irq_attach(RISCV_IRQ_LAMISALIGNED, riscv_misaligned, NULL);
  irq_attach(RISCV_IRQ_SAMISALIGNED, riscv_misaligned, NULL);
#else
  irq_attach(RISCV_IRQ_LAMISALIGNED, riscv_exception, NULL);
  irq_attach(RISCV_IRQ_SAMISALIGNED, riscv_exception, NULL);
#endif

  /* Attach the ecall interrupt handler */

#ifndef CONFIG_BUILD_FLAT
  irq_attach(RISCV_IRQ_ECALLU, riscv_swint, NULL);
#else
  irq_attach(RISCV_IRQ_ECALLU, riscv_exception, NULL);
#endif

  irq_attach(RISCV_IRQ_ECALLS, riscv_exception, NULL);
  irq_attach(RISCV_IRQ_ECALLH, riscv_exception, NULL);

#ifndef CONFIG_ARCH_USE_S_MODE
  irq_attach(RISCV_IRQ_ECALLM, riscv_swint, NULL);
#else
  irq_attach(RISCV_IRQ_ECALLM, riscv_exception, NULL);
#endif

  irq_attach(RISCV_IRQ_INSTRUCTIONPF, riscv_exception, NULL);

#ifdef CONFIG_PAGING
  irq_attach(RISCV_IRQ_LOADPF, riscv_fillpage, NULL);
  irq_attach(RISCV_IRQ_STOREPF, riscv_fillpage, NULL);
#else
  irq_attach(RISCV_IRQ_LOADPF, riscv_exception, NULL);
  irq_attach(RISCV_IRQ_STOREPF, riscv_exception, NULL);
#endif

  irq_attach(RISCV_IRQ_RESERVED, riscv_exception, NULL);

#ifdef CONFIG_SMP
  irq_attach(RISCV_IRQ_SOFT, riscv_pause_handler, NULL);
#else
  irq_attach(RISCV_IRQ_MSOFT, riscv_exception, NULL);
#endif
}
