/****************************************************************************
 *  arch/arm/src/armv7-r/arm_syscall.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#include <arch/irq.h>
#include <nuttx/sched.h>

#include "arm.h"
#include "svcall.h"
#include "up_internal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dispatch_syscall
 *
 * Description:
 *   Call the stub function corresponding to the system call.  NOTE the non-
 *   standard parameter passing:
 *
 *     R0 = SYS_ call number
 *     R1 = parm0
 *     R2 = parm1
 *     R3 = parm2
 *     R4 = parm3
 *     R5 = parm4
 *     R6 = parm5
 *
 *   The values of R4-R5 may be preserved in the proxy called by the user
 *   code if they are used (but otherwise will not be).
 *
 *   WARNING: There are hard-coded values in this logic!
 *
 *   Register usage:
 *
 *     R0 - Need not be preserved.
 *     R1-R3 - Need to be preserved until the stub is called.  The values of
 *       R0 and R1 returned by the stub must be preserved.
 *     R4-R11 must be preserved to support the expectations of the user-space
 *       callee.  R4-R6 may have been preserved by the proxy, but don't know
 *       for sure.
 *     R12 - Need not be preserved
 *     R13 - (stack pointer)
 *     R14 - Need not be preserved
 *     R15 - (PC)
 *
 ****************************************************************************/

#ifdef CONFIG_LIB_SYSCALL
static void dispatch_syscall(void) naked_function;
static void dispatch_syscall(void)
{
  __asm__ __volatile__
  (
    " sub sp, sp, #16\n"           /* Create a stack frame to hold 3 parms + lr */
    " str r4, [sp, #0]\n"          /* Move parameter 4 (if any) into position */
    " str r5, [sp, #4]\n"          /* Move parameter 5 (if any) into position */
    " str r6, [sp, #8]\n"          /* Move parameter 6 (if any) into position */
    " str lr, [sp, #12]\n"         /* Save lr in the stack frame */
    " ldr ip, =g_stublookup\n"     /* R12=The base of the stub lookup table */
    " ldr ip, [ip, r0, lsl #2]\n"  /* R12=The address of the stub for this SYSCALL */
    " blx ip\n"                    /* Call the stub (modifies lr) */
    " ldr lr, [sp, #12]\n"         /* Restore lr */
    " add sp, sp, #16\n"           /* Destroy the stack frame */
    " mov r2, r0\n"                /* R2=Save return value in R2 */
    " mov r0, #0\n"                /* R0=SYS_syscall_return */
    " svc #0x900001\n"             /* Return from the SYSCALL */
  );
}
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_syscall
 *
 * Description:
 *   SVC interrupts will vector here with insn=the SVC instruction and
 *   xcp=the interrupt context
 *
 *   The handler may get the SVC number be de-referencing the return
 *   address saved in the xcp and decoding the SVC instruction
 *
 ****************************************************************************/

#ifdef CONFIG_LIB_SYSCALL
uint32_t *arm_syscall(uint32_t *regs)
{
  uint32_t cmd;
#ifdef CONFIG_BUILD_PROTECTED
  uint32_t cpsr;
#endif

  /* Nested interrupts are not supported */

  DEBUGASSERT(regs);

  /* The SYSCALL command is in R0 on entry.  Parameters follow in R1..R7 */

  cmd = regs[REG_R0];

  /* The SVCall software interrupt is called with R0 = system call command
   * and R1..R7 =  variable number of arguments depending on the system call.
   */

  svcinfo("SYSCALL Entry: regs: %p cmd: %d\n", regs, cmd);
  svcinfo("  R0: %08x %08x %08x %08x %08x %08x %08x %08x\n",
          regs[REG_R0],  regs[REG_R1],  regs[REG_R2],  regs[REG_R3],
          regs[REG_R4],  regs[REG_R5],  regs[REG_R6],  regs[REG_R7]);
  svcinfo("  R8: %08x %08x %08x %08x %08x %08x %08x %08x\n",
          regs[REG_R8],  regs[REG_R9],  regs[REG_R10], regs[REG_R11],
          regs[REG_R12], regs[REG_R13], regs[REG_R14], regs[REG_R15]);
  svcinfo("CPSR: %08x\n", regs[REG_CPSR]);

  /* Handle the SVCall according to the command in R0 */

  switch (cmd)
    {
      /* R0=SYS_syscall_return:  This a SYSCALL return command:
       *
       *   void up_syscall_return(void);
       *
       * At this point, the following values are saved in context:
       *
       *   R0 = SYS_syscall_return
       *
       * We need to restore the saved return address and return in
       * unprivileged thread mode.
       */

      case SYS_syscall_return:
        {
          FAR struct tcb_s *rtcb = sched_self();
          int index = (int)rtcb->xcp.nsyscalls - 1;

          /* Make sure that there is a saved SYSCALL return address. */

          DEBUGASSERT(index >= 0);

          /* Setup to return to the saved SYSCALL return address in
           * the original mode.
           */

          regs[REG_PC]        = rtcb->xcp.syscall[index].sysreturn;
#ifdef CONFIG_BUILD_PROTECTED
          regs[REG_CPSR]      = rtcb->xcp.syscall[index].cpsr;
#endif
          /* The return value must be in R0-R1.  dispatch_syscall() temporarily
           * moved the value for R0 into R2.
           */

          regs[REG_R0]         = regs[REG_R2];

#ifdef CONFIG_ARCH_KERNEL_STACK
          /* If this is the outermost SYSCALL and if there is a saved user stack
           * pointer, then restore the user stack pointer on this final return to
           * user code.
           */

          if (index == 0 && rtcb->xcp.ustkptr != NULL)
            {
              regs[REG_SP]      = (uint32_t)rtcb->xcp.ustkptr;
              rtcb->xcp.ustkptr = NULL;
            }
#endif
          /* Save the new SYSCALL nesting level */

          rtcb->xcp.nsyscalls = index;
        }
        break;

      /* R0=SYS_context_restore:  Restore task context
       *
       * void up_fullcontextrestore(uint32_t *restoreregs) noreturn_function;
       *
       * At this point, the following values are saved in context:
       *
       *   R0 = SYS_context_restore
       *   R1 = restoreregs
       */

#ifdef CONFIG_BUILD_PROTECTED
      case SYS_context_restore:
        {
          /* Replace 'regs' with the pointer to the register set in
           * regs[REG_R1].  On return from the system call, that register
           * set will determine the restored context.
           */

          regs = (uint32_t *)regs[REG_R1];
          DEBUGASSERT(regs);
        }
        break;
#endif

      /* R0=SYS_task_start:  This a user task start
       *
       *   void up_task_start(main_t taskentry, int argc, FAR char *argv[]) noreturn_function;
       *
       * At this point, the following values are saved in context:
       *
       *   R0 = SYS_task_start
       *   R1 = taskentry
       *   R2 = argc
       *   R3 = argv
       */

#ifdef CONFIG_BUILD_PROTECTED
      case SYS_task_start:
        {
          /* Set up to return to the user-space _start function in
           * unprivileged mode.  We need:
           *
           *   R0   = argc
           *   R1   = argv
           *   PC   = taskentry
           *   CSPR = user mode
           */

          regs[REG_PC]   = regs[REG_R1];
          regs[REG_R0]   = regs[REG_R2];
          regs[REG_R1]   = regs[REG_R3];

          cpsr           = regs[REG_CPSR] & ~PSR_MODE_MASK;
          regs[REG_CPSR] = cpsr | PSR_MODE_USR;
        }
        break;
#endif

      /* R0=SYS_pthread_start:  This a user pthread start
       *
       *   void up_pthread_start(pthread_startroutine_t entrypt, pthread_addr_t arg) noreturn_function;
       *
       * At this point, the following values are saved in context:
       *
       *   R0 = SYS_pthread_start
       *   R1 = entrypt
       *   R2 = arg
       */

#if defined(CONFIG_BUILD_PROTECTED) && !defined(CONFIG_DISABLE_PTHREAD)
      case SYS_pthread_start:
        {
          /* Set up to return to the user-space pthread start-up function in
           * unprivileged mode. We need:
           *
           *   R0   = arg
           *   PC   = entrypt
           *   CSPR = user mode
           */


          regs[REG_PC]   = regs[REG_R1];
          regs[REG_R0]   = regs[REG_R2];

          cpsr           = regs[REG_CPSR] & ~PSR_MODE_MASK;
          regs[REG_CPSR] = cpsr | PSR_MODE_USR;
        }
        break;
#endif

#if defined(CONFIG_BUILD_PROTECTED) && !defined(CONFIG_DISABLE_SIGNALS)
      /* R0=SYS_signal_handler:  This a user signal handler callback
       *
       * void signal_handler(_sa_sigaction_t sighand, int signo,
       *                     FAR siginfo_t *info, FAR void *ucontext);
       *
       * At this point, the following values are saved in context:
       *
       *   R0 = SYS_signal_handler
       *   R1 = sighand
       *   R2 = signo
       *   R3 = info
       *   R4 = ucontext
       */

      case SYS_signal_handler:
        {
          FAR struct tcb_s *rtcb = sched_self();
          /* Remember the caller's return address */

          DEBUGASSERT(rtcb->xcp.sigreturn == 0);
          rtcb->xcp.sigreturn  = regs[REG_PC];

          /* Set up to return to the user-space pthread start-up function in
           * unprivileged mode.
           */

          regs[REG_PC]   = (uint32_t)ARCH_DATA_RESERVE->ar_sigtramp;
          cpsr           = regs[REG_CPSR] & ~PSR_MODE_MASK;
          regs[REG_CPSR] = cpsr | PSR_MODE_USR;

          /* Change the parameter ordering to match the expectation of struct
           * userpace_s signal_handler.
           */

          regs[REG_R0]   = regs[REG_R1]; /* sighand */
          regs[REG_R1]   = regs[REG_R2]; /* signal */
          regs[REG_R2]   = regs[REG_R3]; /* info */
          regs[REG_R3]   = regs[REG_R4]; /* ucontext */

#ifdef CONFIG_ARCH_KERNEL_STACK
          /* If we are signalling a user process, then we must be operating
           * on the kernel stack now.  We need to switch back to the user
           * stack before dispatching the signal handler to the user code.
           * The existence of an allocated kernel stack is sufficient
           * information to make this decision.
           */

          if (rtcb->xcp.kstack != NULL)
            {
              DEBUGASSERT(rtcb->xcp.kstkptr == NULL && rtcb->xcp.ustkptr != NULL);

              rtcb->xcp.kstkptr = (FAR uint32_t *)regs[REG_SP];
              regs[REG_SP]      = (uint32_t)rtcb->xcp.ustkptr;
            }
#endif
        }
        break;
#endif

#if defined(CONFIG_BUILD_PROTECTED) && !defined(CONFIG_DISABLE_SIGNALS)
      /* R0=SYS_signal_handler_return:  This a user signal handler callback
       *
       *   void signal_handler_return(void);
       *
       * At this point, the following values are saved in context:
       *
       *   R0 = SYS_signal_handler_return
       */

      case SYS_signal_handler_return:
        {
          FAR struct tcb_s *rtcb = sched_self();

          /* Set up to return to the kernel-mode signal dispatching logic. */

          DEBUGASSERT(rtcb->xcp.sigreturn != 0);

          regs[REG_PC]         = rtcb->xcp.sigreturn;
          cpsr                 = regs[REG_CPSR] & ~PSR_MODE_MASK;
          regs[REG_CPSR]       = cpsr | PSR_MODE_SVC;
          rtcb->xcp.sigreturn  = 0;

#ifdef CONFIG_ARCH_KERNEL_STACK
          /* We must enter here be using the user stack.  We need to switch
           * to back to the kernel user stack before returning to the kernel
           * mode signal trampoline.
           */

          if (rtcb->xcp.kstack != NULL)
            {
              DEBUGASSERT(rtcb->xcp.kstkptr != NULL &&
                          (uint32_t)rtcb->xcp.ustkptr == regs[REG_SP]);

              regs[REG_SP]      = (uint32_t)rtcb->xcp.kstkptr;
              rtcb->xcp.kstkptr = NULL;
            }
#endif
        }
        break;
#endif

      /* This is not an architecture-specific system call.  If NuttX is built
       * as a standalone kernel with a system call interface, then all of the
       * additional system calls must be handled as in the default case.
       */

      default:
        {
#ifdef CONFIG_LIB_SYSCALL
          FAR struct tcb_s *rtcb = sched_self();
          int index = rtcb->xcp.nsyscalls;

          /* Verify that the SYS call number is within range */

          DEBUGASSERT(cmd >= CONFIG_SYS_RESERVED && cmd < SYS_maxsyscall);

          /* Make sure that there is a no saved SYSCALL return address.  We
           * cannot yet handle nested system calls.
           */

          DEBUGASSERT(index < CONFIG_SYS_NNEST);

          /* Setup to return to dispatch_syscall in privileged mode. */

          rtcb->xcp.syscall[index].sysreturn = regs[REG_PC];
#ifdef CONFIG_BUILD_PROTECTED
          rtcb->xcp.syscall[index].cpsr      = regs[REG_CPSR];
#endif

          regs[REG_PC]   = (uint32_t)dispatch_syscall;
#ifdef CONFIG_BUILD_PROTECTED
          cpsr           = regs[REG_CPSR] & ~PSR_MODE_MASK;
          regs[REG_CPSR] = cpsr | PSR_MODE_SVC;
#endif
          /* Offset R0 to account for the reserved values */

          regs[REG_R0] -= CONFIG_SYS_RESERVED;
#else
          svcerr("ERROR: Bad SYS call: %d\n", regs[REG_R0]);
#endif

#ifdef CONFIG_ARCH_KERNEL_STACK
          /* If this is the first SYSCALL and if there is an allocated
           * kernel stack, then switch to the kernel stack.
           */

          if (index == 0 && rtcb->xcp.kstack != NULL)
            {
              rtcb->xcp.ustkptr = (FAR uint32_t *)regs[REG_SP];
              regs[REG_SP]      = (uint32_t)rtcb->xcp.kstack + ARCH_KERNEL_STACKSIZE;
            }
#endif
          /* Save the new SYSCALL nesting level */

          rtcb->xcp.nsyscalls   = index + 1;
        }
        break;
    }

  /* Report what happened */

  svcinfo("SYSCALL Exit: regs: %p\n", regs);
  svcinfo("  R0: %08x %08x %08x %08x %08x %08x %08x %08x\n",
          regs[REG_R0],  regs[REG_R1],  regs[REG_R2],  regs[REG_R3],
          regs[REG_R4],  regs[REG_R5],  regs[REG_R6],  regs[REG_R7]);
  svcinfo("  R8: %08x %08x %08x %08x %08x %08x %08x %08x\n",
          regs[REG_R8],  regs[REG_R9],  regs[REG_R10], regs[REG_R11],
          regs[REG_R12], regs[REG_R13], regs[REG_R14], regs[REG_R15]);
  svcinfo("CPSR: %08x\n", regs[REG_CPSR]);

  /* Return the last value of curent_regs.  This supports context switches
   * on return from the exception.  That capability is only used with the
   * SYS_context_switch system call.
   */

  return regs;
}

#else

uint32_t *arm_syscall(uint32_t *regs)
{
  _alert("SYSCALL from 0x%x\n", regs[REG_PC]);
  CURRENT_REGS = regs;
  PANIC();
}

#endif
