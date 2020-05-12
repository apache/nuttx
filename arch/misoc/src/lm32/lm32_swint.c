/****************************************************************************
 * arch/misoc/src/lm32/lm32_swint.c
 *
 *   Copyright (C) 2016, 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Ramtin Amin <keytwo@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>
#include <syscall.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/sched.h>
#include <arch/irq.h>

#include "signal/signal.h"
#include "lm32.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_registerdump
 ****************************************************************************/

#ifdef CONFIG_DEBUG_SYSCALL_INFO
static void up_registerdump(const uint32_t *regs)
{
#if 0
  svcinfo("EPC:%08x\n",
          regs[REG_EPC]);
  svcinfo("A0:%08x A1:%08x A2:%08x A3:%08x "
          "A4:%08x A5:%08x A6:%08x A7:%08x\n",
          regs[REG_A0], regs[REG_A1], regs[REG_A2], regs[REG_A3],
          regs[REG_A4], regs[REG_A5], regs[REG_A6], regs[REG_A7]);
  svcinfo("T0:%08x T1:%08x T2:%08x T3:%08x T4:%08x T5:%08x T6:%08x\n",
          regs[REG_T0], regs[REG_T1], regs[REG_T2], regs[REG_T3],
          regs[REG_T4], regs[REG_T5], regs[REG_T6]);
  svcinfo("S0:%08x S1:%08x S2:%08x S3:%08x "
          "S4:%08x S5:%08x S6:%08x S7:%08x\n",
          regs[REG_S0], regs[REG_S1], regs[REG_S2], regs[REG_S3],
          regs[REG_S4], regs[REG_S5], regs[REG_S6], regs[REG_S7]);
  svcinfo("S8:%08x S9:%08x S10:%08x S11:%08x\n",
          regs[REG_S8], regs[REG_S9], regs[REG_S10], regs[REG_S11]);
#ifdef LM3232_SAVE_GP
  svcinfo("GP:%08x SP:%08x FP:%08x TP:%08x RA:%08x\n",
          regs[REG_GP], regs[REG_SP], regs[REG_FP], regs[REG_TP],
          regs[REG_RA]);
#else
  svcinfo("SP:%08x FP:%08x TP:%08x RA:%08x\n",
          regs[REG_SP], regs[REG_FP], regs[REG_TP], regs[REG_RA]);
#endif
#endif
}
#else
#  define up_registerdump(regs)
#endif

/****************************************************************************
 * Name: dispatch_syscall
 *
 * Description:
 *   Call the stub function corresponding to the system call.
 *
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
static void dispatch_syscall(void) naked_function;
static void dispatch_syscall(void)
{
#error "Missing logic"

  /* Refer to arch/arm/src/armv7-m/up_svcall.h for how this is done for ARM */

#if 0 /* REVISIT */
  __asm__ __volatile__
  (

      /* Save registers */

      /* Get the base of the stub lookup table */

      /* Get the offset of the stub for this syscall */

      /* Load the entry of the stub for this syscall */

      /* Call the stub */

      /* Restore registers */

      /* Return from the syscall */

  );
#endif
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lm32_swint
 *
 * Description:
 *   This is software interrupt exception handler that performs context
 *   switching and manages system calls
 *
 ****************************************************************************/

int lm32_swint(int irq, FAR void *context, FAR void *arg)
{
  uint32_t *regs = (uint32_t *)context;

  DEBUGASSERT(regs != NULL && regs == g_current_regs);
  g_current_regs = regs;

  /* Software interrupt 0 is invoked with REG_A0 (REG_X10) = system call
   * command and REG_A1-6 = variable number of
   * arguments depending on the system call.
   */

#ifdef CONFIG_DEBUG_SYSCALL_INFO
  svcinfo("Entry: regs: %p cmd: %d\n", regs, regs[REG_A0]);
  up_registerdump(regs);
#endif

  /* Handle the SWInt according to the command in $a0 */

  switch (regs[REG_A0])
    {
      /* A0=SYS_restore_context: This a restore context command:
       *
       *   void up_fullcontextrestore(uint32_t *restoreregs)
       *     noreturn_function;
       *
       * At this point, the following values are saved in context:
       *
       *   A0 = SYS_restore_context
       *   A1 = restoreregs
       *
       * In this case, we simply need to set g_current_regs to restore
       * register area referenced in the saved R1. context == g_current_regs
       * is the normal exception return.  By setting g_current_regs =
       * context[R1], we force the return to the saved context referenced in
       * $a1.
       */

      case SYS_restore_context:
        {
          DEBUGASSERT(regs[REG_A1] != 0);
          g_current_regs = (uint32_t *)regs[REG_A1];
        }
        break;

      /* A0=SYS_switch_context: This a switch context command:
       *
       *   void up_switchcontext(uint32_t *saveregs, uint32_t *restoreregs);
       *
       * At this point, the following values are saved in context:
       *
       *   A0 = SYS_switch_context
       *   A1 = saveregs
       *   A2 = restoreregs
       *
       * In this case, we save the context registers to the save register
       * area referenced by the saved contents of R5 and then set
       * g_current_regs to the save register area referenced by the saved
       * contents of R6.
       */

      case SYS_switch_context:
        {
          DEBUGASSERT(regs[REG_A1] != 0 && regs[REG_A2] != 0);
          lm32_copystate((uint32_t *)regs[REG_A1], regs);
          g_current_regs = (uint32_t *)regs[REG_A2];
        }
        break;

      /* A0=SYS_syscall_return: This a switch context command:
       *
       *   void up_sycall_return(void);
       *
       * At this point, the following values are saved in context:
       *
       *   A0 = SYS_syscall_return
       *
       * We need to restore the saved return address and return in
       * unprivileged thread mode.
       */

#ifdef CONFIG_BUILD_KERNEL
      case SYS_syscall_return:
        {
          struct tcb_s *rtcb = nxsched_self();
          int index = (int)rtcb->xcp.nsyscalls - 1;

          /* Make sure that there is a saved syscall return address. */

          DEBUGASSERT(index >= 0);

          /* Setup to return to the saved syscall return address in
           * the original mode.
           */

          g_current_regs[REG_EPC] = rtcb->xcp.syscall[index].sysreturn;
#error "Missing logic -- need to restore the original mode"
          rtcb->xcp.nsyscalls     = index;

          /* Handle any signal actions that were deferred while processing
           * the system call.
           */

          rtcb->flags            &= ~TCB_FLAG_SYSCALL;
          (void)nxsig_unmask_pendingsignal();
        }
        break;
#endif

      /* This is not an architecture-specify system call.  If NuttX is built
       * as a standalone kernel with a system call interface, then all of the
       * additional system calls must be handled as in the default case.
       */

      default:
        {
#ifdef CONFIG_BUILD_KERNEL
          FAR struct tcb_s *rtcb = nxsched_self();
          int index = rtcb->xcp.nsyscalls;

          /* Verify that the SYS call number is within range */

          DEBUGASSERT(g_current_regs[REG_A0] < SYS_maxsyscall);

          /* Make sure that we got here that there is a no saved syscall
           * return address.  We cannot yet handle nested system calls.
           */

          DEBUGASSERT(index < CONFIG_SYS_NNEST);

          /* Setup to return to dispatch_syscall in privileged mode. */

          rtcb->xcpsyscall[index].sysreturn = regs[REG_EPC];
#error "Missing logic -- Need to save mode"
          rtcb->xcp.nsyscalls  = index + 1;

          regs[REG_EPC]        = (uint32_t)dispatch_syscall;
#error "Missing logic -- Need to set privileged mode"

          /* Offset R0 to account for the reserved values */

          g_current_regs[REG_A0] -= CONFIG_SYS_RESERVED;

          /* Indicate that we are in a syscall handler. */

          rtcb->flags            |= TCB_FLAG_SYSCALL;
#else
          svcerr("ERROR: Bad SYS call: %d\n", regs[REG_A0]);
#endif
        }
        break;
    }

  /* Report what happened.  That might difficult in the case of a context
   * switch.
   */

#ifdef CONFIG_DEBUG_SYSCALL_INFO
  if (regs != g_current_regs)
    {
      svcinfo("SWInt Return: Context switch!\n");
      up_registerdump((const uint32_t *)g_current_regs);
    }
  else
    {
      svcinfo("SWInt Return: %d\n", regs[REG_A0]);
    }
#endif

#if defined(CONFIG_ARCH_FPU) || defined(CONFIG_ARCH_ADDRENV)
  /* Check for a context switch.  If a context switch occurred, then
   * g_current_regs will have a different value than it did on entry.  If an
   * interrupt level context switch has occurred, then restore the floating
   * point state and the establish the correct address environment before
   * returning from the interrupt.
   */

  if (regs != g_current_regs)
    {
#ifdef CONFIG_ARCH_FPU
      /* Restore floating point registers */

      up_restorefpu((uint32_t *)g_current_regs);
#endif

#ifdef CONFIG_ARCH_ADDRENV
      /* Make sure that the address environment for the previously
       * running task is closed down gracefully (data caches dump,
       * MMU flushed) and set up the address environment for the new
       * thread at the head of the ready-to-run list.
       */

      group_addrenv(NULL);
#endif
    }
#endif

  return OK;
}
