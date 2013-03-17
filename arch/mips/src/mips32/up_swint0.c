/****************************************************************************
 * arch/mips/src/mips32/up_swint0.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
#include <arch/mips32/cp0.h>

#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

/* Debug ********************************************************************/
/* Debug output from this file may interfere with context switching!  To get
 * debug output you must enabled the following in your NuttX configuration:
 *
 * CONFIG_DEBUG and CONFIG_DEBUG_SYSCALL
 */

#ifdef CONFIG_DEBUG_SYSCALL
# define swidbg(format, arg...) lldbg(format, ##arg)
#else
# define swidbg(x...)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_registerdump
 ****************************************************************************/

#ifdef CONFIG_DEBUG_SYSCALL
static void up_registerdump(const uint32_t *regs)
{
  swidbg("MFLO:%08x MFHI:%08x EPC:%08x STATUS:%08x\n",
         regs[REG_MFLO], regs[REG_MFHI], regs[REG_EPC], regs[REG_STATUS]);
  swidbg("AT:%08x V0:%08x V1:%08x A0:%08x A1:%08x A2:%08x A3:%08x\n",
         regs[REG_AT], regs[REG_V0], regs[REG_V1], regs[REG_A0],
         regs[REG_A1], regs[REG_A2], regs[REG_A3]);
  swidbg("T0:%08x T1:%08x T2:%08x T3:%08x T4:%08x T5:%08x T6:%08x T7:%08x\n",
         regs[REG_T0], regs[REG_T1], regs[REG_T2], regs[REG_T3],
         regs[REG_T4], regs[REG_T5], regs[REG_T6], regs[REG_T7]);
  swidbg("S0:%08x S1:%08x S2:%08x S3:%08x S4:%08x S5:%08x S6:%08x S7:%08x\n",
         regs[REG_S0], regs[REG_S1], regs[REG_S2], regs[REG_S3],
         regs[REG_S4], regs[REG_S5], regs[REG_S6], regs[REG_S7]);
#ifdef MIPS32_SAVE_GP
  swidbg("T8:%08x T9:%08x GP:%08x SP:%08x FP:%08x RA:%08x\n",
         regs[REG_T8], regs[REG_T9], regs[REG_GP], regs[REG_SP],
         regs[REG_FP], regs[REG_RA]);
#else
  swidbg("T8:%08x T9:%08x SP:%08x FP:%08x RA:%08x\n",
         regs[REG_T8], regs[REG_T9], regs[REG_SP], regs[REG_FP],
         regs[REG_RA]);
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

#ifdef CONFIG_NUTTX_KERNEL
static void dispatch_syscall(void) naked_function;
static void dispatch_syscall(void)
{
#  error "Missing logic"

/* Refer to arch/arm/src/armv7-m/up_svcall.h for how this is done for ARM */
/*  __asm__ __volatile__ */
/* ( */
/*   Save registers */
/*   Get the base of the stub lookup table */
/*   Get the offset of the stub for this syscall */
/*   Load the entry of the stub for this syscall */
/*   Call the stub */
/*   Restore regsisters */
/*   Return from the syscall */
/* ); */
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_swint0
 *
 * Description:
 *   This is software interrupt 0 exception handler that performs context
 *   switching and manages system calls
 *
 ****************************************************************************/

int up_swint0(int irq, FAR void *context)
{
  uint32_t *regs = (uint32_t*)context;
  uint32_t cause;

  DEBUGASSERT(regs && regs == current_regs);

  /* Software interrupt 0 is invoked with REG_A0 (REG_R4) = system call
   * command and REG_A1-3 and REG_T0-2 (REG_R5-10) = variable number of
   * arguments depending on the system call.
   */

#ifdef CONFIG_DEBUG_SYSCALL
  swidbg("Entry: regs: %p cmd: %d\n", regs, regs[REG_R4]);
  up_registerdump(regs);
#endif

  /* Handle the SWInt according to the command in $4 */

  switch (regs[REG_R4])
    {
      /* R4=SYS_restore_context: This a restore context command:
       *
       *   void up_fullcontextrestore(uint32_t *restoreregs) noreturn_function;
       *
       * At this point, the following values are saved in context:
       *
       *   R4 = SYS_restore_context
       *   R5 = restoreregs
       *
       * In this case, we simply need to set current_regs to restore register
       * area referenced in the saved R1. context == current_regs is the normal
       * exception return.  By setting current_regs = context[R1], we force
       * the return to the saved context referenced in R1.
       */

      case SYS_restore_context:
        {
          DEBUGASSERT(regs[REG_A1] != 0);
          current_regs = (uint32_t*)regs[REG_A1];
        }
        break;

      /* R4=SYS_switch_context: This a switch context command:
       *
       *   void up_switchcontext(uint32_t *saveregs, uint32_t *restoreregs);
       *
       * At this point, the following values are saved in context:
       *
       *   R4 = SYS_switch_context
       *   R5 = saveregs
       *   R6 = restoreregs
       *
       * In this case, we save the context registers to the save register
       * area reference by the saved contents of R5 and then set
       * current_regs to to the save register area referenced by the saved
       * contents of R6.
       */

      case SYS_switch_context:
        {
          DEBUGASSERT(regs[REG_A1] != 0 && regs[REG_A2] != 0);
          up_copystate((uint32_t*)regs[REG_A1], regs);
          current_regs = (uint32_t*)regs[REG_A2];
        }
        break;

      /* R0=SYS_syscall_return: This a switch context command:
       *
       *   void up_sycall_return(void);
       *
       * At this point, the following values are saved in context:
       *
       *   R0 = SYS_syscall_return
       *
       * We need to restore the saved return address and return in
       * unprivileged thread mode.
       */

#ifdef CONFIG_NUTTX_KERNEL
      case SYS_syscall_return:
        {
          struct tcb_s *rtcb = sched_self();
          int index = (int)rtcb->xcp.nsyscalls - 1;

          /* Make sure that there is a saved syscall return address. */

          DEBUGASSERT(index >= 0);

          /* Setup to return to the saved syscall return address in
           * the original mode.
           */

          current_regs[REG_EPC] = rtcb->xcp.syscall[index].sysreturn;
#error "Missing logic -- need to restore the original mode"
          rtcb->xcp.nsyscalls   = index;
        }
        break;
#endif

      /* This is not an architecture-specify system call.  If NuttX is built
       * as a standalone kernel with a system call interface, then all of the
       * additional system calls must be handled as in the default case.
       */

      default:
        {
#ifdef CONFIG_NUTTX_KERNEL
          FAR struct tcb_s *rtcb = sched_self();
          int index = rtcb->xcp.nsyscalls;

          /* Verify that the SYS call number is within range */

          DEBUGASSERT(current_regs[REG_A0] < SYS_maxsyscall);

          /* Make sure that we got here that there is a no saved syscall
           * return address.  We cannot yet handle nested system calls.
           */

          DEBUGASSERT(index < CONFIG_SYS_NNEST);

          /* Setup to return to dispatch_syscall in privileged mode. */

          rtcb->xcpsyscall[index].sysreturn = regs[REG_EPC];
#error "Missing logic -- Need to save mode"
          rtcb->xcp.nsyscalls  = index + 1;

          regs[REG_EPC] = (uint32_t)dispatch_syscall;
#error "Missing logic -- Need to set privileged mode"

          /* Offset R0 to account for the reserved values */

          current_regs[REG_R0] -= CONFIG_SYS_RESERVED;
#else
          slldbg("ERROR: Bad SYS call: %d\n", regs[REG_A0]);
#endif
        }
        break;
    }

  /* Report what happened.  That might difficult in the case of a context switch */

#ifdef CONFIG_DEBUG_SYSCALL
  if (regs != current_regs)
    {
      swidbg("SWInt Return: Context switch!\n");
      up_registerdump((const uint32_t*)current_regs);
    }
  else
    {
      swidbg("SWInt Return: %d\n", regs[REG_V0]);
    }
#endif

  /* Clear the pending software interrupt 0 in the PIC32 interrupt block */
 
  up_clrpend_irq(PIC32MX_IRQSRC_CS0);

  /* And reset the software interrupt bit in the MIPS CAUSE register */

  cause  = cp0_getcause();
  cause &= ~CP0_CAUSE_IP0;
  cp0_putcause(cause);

  return OK;
}
