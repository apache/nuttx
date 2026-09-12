/****************************************************************************
 * arch/risc-v/src/common/riscv_swint.c
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

#include <inttypes.h>
#include <stdint.h>
#ifdef CONFIG_RISCV_FRAME_TRACE
#  include <stdbool.h>
#endif
#include <string.h>
#include <assert.h>
#include <nuttx/debug.h>

#include <arch/irq.h>
#include <nuttx/addrenv.h>
#include <nuttx/sched.h>
#include <nuttx/userspace.h>

#ifdef CONFIG_LIB_SYSCALL
#  include <syscall.h>
#endif

#include "sched/sched.h"
#include "signal/signal.h"
#include "riscv_internal.h"
#include "addrenv.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef uintptr_t (*syscall_t)(unsigned int, ...);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_LIB_SYSCALL

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dispatch_syscall
 *
 * Description:
 *   Call the stub function corresponding to the system call.  NOTE the non-
 *   standard parameter passing:
 *
 *     A0 = SYS_ call number
 *     A1 = parm0
 *     A2 = parm1
 *     A3 = parm2
 *     A4 = parm3
 *     A5 = parm4
 *     A6 = parm5
 *     A7 = context (aka SP)
 *
 ****************************************************************************/

uintptr_t dispatch_syscall(unsigned int nbr, uintptr_t parm1,
                           uintptr_t parm2, uintptr_t parm3,
                           uintptr_t parm4, uintptr_t parm5,
                           uintptr_t parm6, void *context)
{
  struct tcb_s *rtcb         = this_task();
  register long a0 asm("a0") = (long)(nbr);
  register long a1 asm("a1") = (long)(parm1);
  register long a2 asm("a2") = (long)(parm2);
  register long a3 asm("a3") = (long)(parm3);
  register long a4 asm("a4") = (long)(parm4);
  register long a5 asm("a5") = (long)(parm5);
  register long a6 asm("a6") = (long)(parm6);
  syscall_t do_syscall;
  uintptr_t ret;

  /* Valid system call ? */

  if (a0 > SYS_maxsyscall)
    {
      /* Nope, get out */

      return -ENOSYS;
    }

  /* Set the user register context to TCB */

  rtcb->xcp.sregs = context;

  /* Indicate that we are in a syscall handler */

  rtcb->flags |= TCB_FLAG_SYSCALL;

  /* Offset a0 to account for the reserved syscalls */

  a0 -= CONFIG_SYS_RESERVED;

  /* Find the system call from the lookup table */

  do_syscall = (syscall_t)g_stublookup[a0];

  /* Run the system call, save return value locally */

  ret = do_syscall(a0, a1, a2, a3, a4, a5, a6);

  /* System call is now done */

  rtcb->flags &= ~TCB_FLAG_SYSCALL;

  /* Unmask any pending signals now */

  nxsig_unmask_pendingsignal();

  return ret;
}
#endif

#ifdef CONFIG_RISCV_FRAME_TRACE

/****************************************************************************
 * Trap frame trace, CONFIG_RISCV_FRAME_TRACE.
 *
 * The question: when a user task blocks inside a syscall and is later
 * resumed, which frame does the kernel restore on the way back out, and does
 * its REG_INT_CTX carry MPP=M?  riscv_doirq() ends with
 * "regs = tcb->xcp.regs", so that is the single point where every trap
 * decides what to restore -- RISCV_TRACE_TAG_DOIRQ records it.  The
 * RISCV_TRACE_TAG_RESTORE_CTX and RISCV_TRACE_TAG_SWITCH_CTX tags record the
 * two context-switch cases below.
 *
 * Nothing is printed inline: on the polled UART console each character costs
 * ~87 us, and printing from inside the switch path would perturb the timing
 * under test.  The ring buffer is dumped from the "default" case instead --
 * i.e. exactly when the flip has produced its panic.
 ****************************************************************************/

#define TRACE_ENTRIES  64
#define TRACE_USER_LO  0x40200000  /* user image text window */
#define TRACE_USER_HI  0x40300000

struct riscv_trace_s
{
  uintreg_t *regs;
  uintreg_t  epc;
  uintreg_t  int_ctx;
  uintreg_t  mcause;
  int16_t    irq;
  uint8_t    tag;                  /* RISCV_TRACE_TAG_* */
  pid_t      pid;
};

static struct riscv_trace_s g_trace[TRACE_ENTRIES];
static unsigned int         g_trace_ndx;

/* Written by the asm probe in return_from_exception
 * (riscv_exception_common.S) immediately after the mepc/mstatus restores:
 * what the mret will actually consume, as opposed to what the frame holds.
 */

uintreg_t g_mret_epc;
uintreg_t g_mret_status;
uintreg_t g_mret_cause;

void riscv_trace_frame(int tag, struct tcb_s *tcb, uintreg_t *regs)
{
  struct riscv_trace_s *e = &g_trace[g_trace_ndx % TRACE_ENTRIES];

  e->tag  = (uint8_t)(tag & 0xff);
  e->irq  = (int16_t)(tag >> 8);
  e->pid  = tcb != NULL ? tcb->pid : -1;
  e->regs = regs;

  if (regs != NULL)
    {
      e->epc     = regs[REG_EPC];
      e->int_ctx = regs[REG_INT_CTX];
#ifdef REG_MCAUSE
      /* The frame's mcause, so mpil (23:16) can be tracked: mil pinned at 63
       * is what deadlocks the interrupt system.
       */

      e->mcause  = regs[REG_MCAUSE];
#endif
    }
  else
    {
      e->epc     = 0;
      e->int_ctx = 0;
      e->mcause  = 0;
    }

  g_trace_ndx++;
}

/* Called from return_from_syscall (riscv_exception_common.S).  User
 * syscall returns bypass riscv_doirq() entirely -- they restore the frame
 * at sp -- so this is the only place they can be observed.
 */

void riscv_trace_syscall_ret(uintreg_t *regs)
{
  riscv_trace_frame(RISCV_TRACE_TAG_SYSCALL_RET, this_task(), regs);
}

/* Called from exception_common (riscv_exception_common.S) on EVERY trap
 * entry, once the frame is fully formed.  This is the mirror of the
 * exit records: it shows the privilege the CPU was in when the trap was
 * taken, straight from the hardware, plus the raw mcause.
 */

void riscv_trace_trap_entry(uintreg_t *regs, uintreg_t mcause)
{
  struct riscv_trace_s *e = &g_trace[g_trace_ndx % TRACE_ENTRIES];

  riscv_trace_frame(RISCV_TRACE_TAG_TRAP_ENTRY, this_task(), regs);

  /* riscv_trace_frame() may have skipped the entry; only stamp mcause if it
   * actually recorded one.
   */

  if (e->tag == RISCV_TRACE_TAG_TRAP_ENTRY)
    {
      e->mcause = mcause;
    }
}

void riscv_trace_dump(uintreg_t *regs)
{
  unsigned int total;

  /* Record the panicking frame itself, so that its REG_INT_CTX can be
   * read out of RAM with an observe-only halt.  The _alert() output below is
   * unreliable on this board at panic time; the ring buffer is not.
   */

  /* Stamp what the last mret consumed, from the asm probe, so that it
   * comes out of the same ring-buffer read.
   */

  {
    struct riscv_trace_s *m = &g_trace[g_trace_ndx % TRACE_ENTRIES];

    m->tag     = RISCV_TRACE_TAG_MRET;
    m->pid     = -1;
    m->irq     = 0;
    m->regs    = NULL;
    m->epc     = g_mret_epc;
    m->int_ctx = g_mret_status;
    m->mcause  = g_mret_cause;
    g_trace_ndx++;
  }

  riscv_trace_frame(RISCV_TRACE_TAG_PANIC, this_task(), regs);

  total = g_trace_ndx;
  unsigned int n     = total < TRACE_ENTRIES ? total : TRACE_ENTRIES;
  unsigned int i;

  _alert("TRACE: panic cmd=%" PRIxREG " frame=%p epc=%" PRIxREG
         " int_ctx=%" PRIxREG "\n",
         regs[REG_A0], regs, regs[REG_EPC], regs[REG_INT_CTX]);
  _alert("TRACE: %u events, last %u shown; tag %u=doirq %u=restore "
         "%u=switch\n",
         total, n, RISCV_TRACE_TAG_DOIRQ, RISCV_TRACE_TAG_RESTORE_CTX,
         RISCV_TRACE_TAG_SWITCH_CTX);

  for (i = total - n; i < total; i++)
    {
      struct riscv_trace_s *e = &g_trace[i % TRACE_ENTRIES];
      bool user = e->epc >= TRACE_USER_LO && e->epc < TRACE_USER_HI;
      bool mpp  = (e->int_ctx & STATUS_PPP) != 0;

      _alert("  [%2u] tag=%u irq=%d pid=%d frame=%p epc=%" PRIxREG
             " int_ctx=%" PRIxREG " %s%s\n",
             i, e->tag, e->irq, e->pid, e->regs, e->epc, e->int_ctx,
             user ? "[user]" : "[kern]",
             (user && mpp) ? " <<< USER FRAME WITH MPP=M" : "");
    }
}
#endif /* CONFIG_RISCV_FRAME_TRACE */

/****************************************************************************
 * Name: riscv_swint
 *
 * Description:
 *   This is software interrupt exception handler that performs context
 *   switching and manages system calls
 *
 ****************************************************************************/

int riscv_swint(int irq, void *context, void *arg)
{
  uintreg_t *regs = (uintreg_t *)context;
  struct tcb_s *tcb = this_task();
  int cpu = this_cpu();

  /* Software interrupt 0 is invoked with REG_A0 (REG_X10) = system call
   * command and REG_A1-6 = variable number of
   * arguments depending on the system call.
   */

#ifdef CONFIG_DEBUG_SYSCALL_INFO
  svcinfo("Entry: regs: %p cmd: %d\n", regs, regs[REG_A0]);
  up_dump_register(regs);
#endif

  /* Handle the SWInt according to the command in $a0 */

  switch (regs[REG_A0])
    {
      case SYS_restore_context:
        {
#ifdef CONFIG_RISCV_FRAME_TRACE
          riscv_trace_frame(RISCV_TRACE_TAG_RESTORE_CTX, tcb,
                            tcb->xcp.regs);
#endif
          riscv_restorecontext(tcb);
          restore_critical_section(tcb, cpu);
        }
        break;

      case SYS_switch_context:
        {
#ifdef CONFIG_RISCV_FRAME_TRACE
          riscv_trace_frame(RISCV_TRACE_TAG_SWITCH_CTX, tcb,
                            tcb->xcp.regs);
#endif
          riscv_savecontext(g_running_tasks[cpu]);
          riscv_restorecontext(tcb);
          restore_critical_section(tcb, cpu);
        }
        break;

      /* R0=SYS_signal_handler:  This a user signal handler callback
       *
       * void signal_handler(_sa_sigaction_t sighand, int signo,
       *                     siginfo_t *info, void *ucontext);
       *
       * At this point, the following values are saved in context:
       *
       *   A0 = SYS_signal_handler
       *   A1 = sighand
       *   A2 = signo
       *   A3 = info
       *   A4 = ucontext
       */

#if !defined(CONFIG_BUILD_FLAT) && defined(CONFIG_ENABLE_ALL_SIGNALS)
      case SYS_signal_handler:
        {
#ifdef CONFIG_RISCV_FRAME_TRACE
          riscv_trace_frame(RISCV_TRACE_TAG_SIG_HANDLER, this_task(),
                            regs);
#endif
          struct tcb_s *rtcb   = this_task();

          /* Remember the caller's return address */

          DEBUGASSERT(rtcb->xcp.sigreturn == 0);
          rtcb->xcp.sigreturn  = regs[REG_EPC];

          /* Set up to return to the user-space trampoline function in
           * unprivileged mode.
           */

#if defined (CONFIG_BUILD_PROTECTED)
          regs[REG_EPC]        = (uintptr_t)USERSPACE->signal_handler;
#else
          regs[REG_EPC]        = (uintptr_t)ARCH_DATA_RESERVE->ar_sigtramp;
#endif
          regs[REG_INT_CTX]   &= ~STATUS_PPP; /* User mode */

          /* Change the parameter ordering to match the expectation of struct
           * userpace_s signal_handler.
           */

          regs[REG_A0]         = regs[REG_A1]; /* sighand */
          regs[REG_A1]         = regs[REG_A2]; /* signal */
          regs[REG_A2]         = regs[REG_A3]; /* info */
          regs[REG_A3]         = regs[REG_A4]; /* ucontext */

#ifdef CONFIG_ARCH_KERNEL_STACK
          /* If we are signalling a user process, then we must be operating
           * on the kernel stack now.  We need to switch back to the user
           * stack before dispatching the signal handler to the user code.
           * The existence of an allocated kernel stack is sufficient
           * information to make this decision.
           */

          if (rtcb->xcp.kstack != NULL)
            {
              uintptr_t usp;
              uintptr_t *usr_regs;

              /* Store the current kernel stack pointer so it is not lost */

              rtcb->xcp.kstkptr = (uintptr_t *)regs[REG_SP];

              /* Copy "info" into user stack */

              usr_regs = (uintptr_t *)((uintptr_t)rtcb->xcp.ktopstk -
                                                  XCPTCONTEXT_SIZE);
              usp = usr_regs[REG_SP];

              /* Create a frame for info and copy the kernel info */

              usp = usp - sizeof(siginfo_t);
              memcpy((void *)usp, (void *)regs[REG_A2], sizeof(siginfo_t));

              /* Now set the updated SP and user copy of "info" to A2 */

              regs[REG_SP] = usp;
              regs[REG_A2] = usp;
            }
#endif
        }
        break;
#endif

      /* R0=SYS_signal_handler_return:  This a user signal handler callback
       *
       *   void signal_handler_return(void);
       *
       * At this point, the following values are saved in context:
       *
       *   R0 = SYS_signal_handler_return
       */

#if !defined(CONFIG_BUILD_FLAT) && defined(CONFIG_ENABLE_ALL_SIGNALS)
      case SYS_signal_handler_return:
        {
#ifdef CONFIG_RISCV_FRAME_TRACE
          riscv_trace_frame(RISCV_TRACE_TAG_SIG_RETURN, this_task(),
                            regs);
#endif
          struct tcb_s *rtcb   = this_task();

          /* Set up to return to the kernel-mode signal dispatching logic. */

          DEBUGASSERT(rtcb->xcp.sigreturn != 0);
          regs[REG_EPC]        = rtcb->xcp.sigreturn;
          regs[REG_INT_CTX]   |= STATUS_PPP; /* Privileged mode */

          rtcb->xcp.sigreturn  = 0;

#ifdef CONFIG_ARCH_KERNEL_STACK
          /* We must restore the original kernel stack pointer before
           * returning to the kernel mode signal trampoline.
           */

          if (rtcb->xcp.kstack != NULL)
            {
              DEBUGASSERT(rtcb->xcp.kstkptr != NULL);

              regs[REG_SP]      = (uintptr_t)rtcb->xcp.kstkptr;
              rtcb->xcp.kstkptr = rtcb->xcp.ktopstk;
            }
#endif
        }
        break;
#endif

      default:
#ifdef CONFIG_RISCV_FRAME_TRACE
        /* Record the panicking frame but do NOT print the ring here.
         * riscv_trace_dump() pushes 64 syslog lines through the polled
         * console from inside a trap with interrupts disabled; under an
         * ostest-scale failure that takes minutes and buries the assert
         * output.  Read the ring out of RAM with an observe-only halt
         * instead.
         */

        riscv_trace_frame(RISCV_TRACE_TAG_PANIC, this_task(), regs);
#endif
        DEBUGPANIC();
        break;
    }

  /* Report what happened.  That might difficult in the case of a context
   * switch
   */

#ifdef CONFIG_DEBUG_SYSCALL_INFO
  if (regs[REG_A0] <= SYS_switch_context)
    {
      svcinfo("SWInt Return: Context switch!\n");
      up_dump_register(tcb->xcp.regs);
    }
  else
    {
      svcinfo("SWInt Return: %" PRIxPTR "\n", regs[REG_A0]);
    }
#endif

  return OK;
}
