/****************************************************************************
 * arch/xtensa/src/common/xtensa_schedulesigaction.c
 *
 *   Copyright (C) 2016-2018 Gregory Nutt. All rights reserved.
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
#include <sched.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "sched/sched.h"
#include "xtensa.h"

#include "irq/irq.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_schedule_sigaction
 *
 * Description:
 *   This function is called by the OS when one or more
 *   signal handling actions have been queued for execution.
 *   The architecture specific code must configure things so
 *   that the 'sigdeliver' callback is executed on the thread
 *   specified by 'tcb' as soon as possible.
 *
 *   This function may be called from interrupt handling logic.
 *
 *   This operation should not cause the task to be unblocked
 *   nor should it cause any immediate execution of sigdeliver.
 *   Typically, a few cases need to be considered:
 *
 *   (1) This function may be called from an interrupt handler
 *       During interrupt processing, all xcptcontext structures
 *       should be valid for all tasks.  That structure should
 *       be modified to invoke sigdeliver() either on return
 *       from (this) interrupt or on some subsequent context
 *       switch to the recipient task.
 *   (2) If not in an interrupt handler and the tcb is NOT
 *       the currently executing task, then again just modify
 *       the saved xcptcontext structure for the recipient
 *       task so it will invoke sigdeliver when that task is
 *       later resumed.
 *   (3) If not in an interrupt handler and the tcb IS the
 *       currently executing task -- just call the signal
 *       handler now.
 *
 ****************************************************************************/

#ifndef CONFIG_SMP
void up_schedule_sigaction(struct tcb_s *tcb, sig_deliver_t sigdeliver)
{
  irqstate_t flags;

  sinfo("tcb=0x%p sigdeliver=0x%p\n", tcb, sigdeliver);

  /* Make sure that interrupts are disabled */

  flags = enter_critical_section();

  /* Refuse to handle nested signal actions */

  if (!tcb->xcp.sigdeliver)
    {
      /* First, handle some special cases when the signal is being delivered
       * to the currently executing task.
       */

      sinfo("rtcb=0x%p CURRENT_REGS=0x%p\n", this_task(), CURRENT_REGS);

      if (tcb == this_task())
        {
          /* CASE 1:  We are not in an interrupt handler and a task is
           * signaling itself for some reason.
           */

          if (!CURRENT_REGS)
            {
              /* In this case just deliver the signal now.
               * REVISIT:  Signal handler will run in a critical section!
               */

              sigdeliver(tcb);
            }

          /* CASE 2:  We are in an interrupt handler AND the interrupted
           * task is the same as the one that must receive the signal, then
           * we will have to modify the return state as well as the state
           * in the TCB.
           *
           * Hmmm... there looks like a latent bug here: The following logic
           * would fail in the strange case where we are in an interrupt
           * handler, the thread is signaling itself, but a context switch
           * to another task has occurred so that CURRENT_REGS does not
           * refer to the thread of this_task()!
           */

          else
            {
              /* Save the return pc and ps.  These will be restored by the
               * signal trampoline after the signals have been delivered.
               *
               * NOTE: that hi-priority interrupts are not disabled.
               */

              tcb->xcp.sigdeliver  = sigdeliver;
              tcb->xcp.saved_pc    = CURRENT_REGS[REG_PC];
              tcb->xcp.saved_ps    = CURRENT_REGS[REG_PS];

              /* Then set up to vector to the trampoline with interrupts
               * disabled
               */

              CURRENT_REGS[REG_PC] = (uint32_t)_xtensa_sig_trampoline;
#ifdef __XTENSA_CALL0_ABI__
              CURRENT_REGS[REG_PS] = (uint32_t)
                  (PS_INTLEVEL(XCHAL_EXCM_LEVEL) | PS_UM);
#else
              CURRENT_REGS[REG_PS] = (uint32_t)
                  (PS_INTLEVEL(XCHAL_EXCM_LEVEL) | PS_UM | PS_WOE);
#endif

              /* And make sure that the saved context in the TCB is the same
               * as the interrupt return context.
               */

              xtensa_savestate(tcb->xcp.regs);
            }
        }

      /* Otherwise, we are (1) signaling a task is not running from an
       * interrupt handler or (2) we are not in an interrupt handler and the
       * running task is signaling some non-running task.
       */

      else
        {
          /* Save the return pc and ps.  These will be restored by the
           * signal trampoline after the signals have been delivered.
           *
           * NOTE: that hi-priority interrupts are not disabled.
           */

          tcb->xcp.sigdeliver   = sigdeliver;
          tcb->xcp.saved_pc     = tcb->xcp.regs[REG_PC];
          tcb->xcp.saved_ps     = tcb->xcp.regs[REG_PS];

          /* Then set up to vector to the trampoline with interrupts
           * disabled
           */

          tcb->xcp.regs[REG_PC] = (uint32_t)_xtensa_sig_trampoline;
#ifdef __XTENSA_CALL0_ABI__
          tcb->xcp.regs[REG_PS] = (uint32_t)
              (PS_INTLEVEL(XCHAL_EXCM_LEVEL) | PS_UM);
#else
          tcb->xcp.regs[REG_PS] = (uint32_t)
              (PS_INTLEVEL(XCHAL_EXCM_LEVEL) | PS_UM | PS_WOE);
#endif
        }
    }

  leave_critical_section(flags);
}
#endif /* !CONFIG_SMP */

#ifdef CONFIG_SMP
void up_schedule_sigaction(struct tcb_s *tcb, sig_deliver_t sigdeliver)
{
  irqstate_t flags;
  int cpu;
  int me;

  sinfo("tcb=0x%p sigdeliver=0x%p\n", tcb, sigdeliver);

  /* Make sure that interrupts are disabled */

  flags = enter_critical_section();

  /* Refuse to handle nested signal actions */

  if (!tcb->xcp.sigdeliver)
    {
      /* First, handle some special cases when the signal is being delivered
       * to task that is currently executing on any CPU.
       */

      sinfo("rtcb=0x%p CURRENT_REGS=0x%p\n", this_task(), CURRENT_REGS);

      if (tcb->task_state == TSTATE_TASK_RUNNING)
        {
          me  = this_cpu();
          cpu = tcb->cpu;

          /* CASE 1:  We are not in an interrupt handler and a task is
           * signaling itself for some reason.
           */

          if (cpu == me && !CURRENT_REGS)
            {
              /* In this case just deliver the signal now.
               * REVISIT:  Signal handler will run in a critical section!
               */

              sigdeliver(tcb);
            }

          /* CASE 2:  The task that needs to receive the signal is running.
           * This could happen if the task is running on another CPU OR if
           * we are in an interrupt handler and the task is running on this
           * CPU.  In the former case, we will have to PAUSE the other CPU
           * first.  But in either case, we will have to modify the return
           * state as well as the state in the TCB.
           */

          else
            {
              /* If we signaling a task running on the other CPU, we have
               * to PAUSE the other CPU.
               */

              if (cpu != me)
                {
                  /* Pause the CPU */

                  up_cpu_pause(cpu);

                  /* Wait while the pause request is pending */

                  while (up_cpu_pausereq(cpu))
                    {
                    }

                  /* Now tcb on the other CPU can be accessed safely */

                  /* Copy tcb->xcp.regs to tcp.xcp.saved. These will be
                   * restored by the signal trampoline after the signal has
                   * been delivered.
                   *
                   * NOTE: that hi-priority interrupts are not disabled.
                   */

                  tcb->xcp.sigdeliver  = sigdeliver;
                  tcb->xcp.saved_pc    = tcb->xcp.regs[REG_PC];
                  tcb->xcp.saved_ps    = tcb->xcp.regs[REG_PS];

                  /* Then set up to vector to the trampoline with interrupts
                   * disabled
                   */

                  tcb->xcp.regs[REG_PC] = (uint32_t)_xtensa_sig_trampoline;
#ifdef __XTENSA_CALL0_ABI__
                  tcb->xcp.regs[REG_PS] = (uint32_t)
                      (PS_INTLEVEL(XCHAL_EXCM_LEVEL) | PS_UM);
#else
                  tcb->xcp.regs[REG_PS] = (uint32_t)
                      (PS_INTLEVEL(XCHAL_EXCM_LEVEL) | PS_UM | PS_WOE);
#endif
                }
              else
                {
                  /* tcb is running on the same CPU */

                  /* Copy tcb->xcp.regs to tcp.xcp.saved. These will be
                   * restored by the signal trampoline after the signal has
                   * been delivered.
                   *
                   * NOTE: that hi-priority interrupts are not disabled.
                   */

                  tcb->xcp.sigdeliver  = sigdeliver;
                  tcb->xcp.saved_pc    = CURRENT_REGS[REG_PC];
                  tcb->xcp.saved_ps    = CURRENT_REGS[REG_PS];

                  /* Then set up to vector to the trampoline with interrupts
                   * disabled
                   */

                  CURRENT_REGS[REG_PC] = (uint32_t)_xtensa_sig_trampoline;
#ifdef __XTENSA_CALL0_ABI__
                  CURRENT_REGS[REG_PS] = (uint32_t)
                      (PS_INTLEVEL(XCHAL_EXCM_LEVEL) | PS_UM);
#else
                  CURRENT_REGS[REG_PS] = (uint32_t)
                      (PS_INTLEVEL(XCHAL_EXCM_LEVEL) | PS_UM | PS_WOE);
#endif
                  /* And make sure that the saved context in the TCB is the
                   * same as the interrupt return context.
                   */

                  xtensa_savestate(tcb->xcp.regs);
                }

              /* Increment the IRQ lock count so that when the task is
               * restarted, it will hold the IRQ spinlock.
               */

              DEBUGASSERT(tcb->irqcount < INT16_MAX);
              tcb->irqcount++;

              /* In an SMP configuration, the interrupt disable logic also
               * involves spinlocks that are configured per the TCB irqcount
               * field.  This is logically equivalent to
               * enter_critical_section().
               * The matching call to leave_critical_section() will be
               * performed in up_sigdeliver().
               */

              spin_setbit(&g_cpu_irqset, cpu, &g_cpu_irqsetlock,
                          &g_cpu_irqlock);

              /* RESUME the other CPU if it was PAUSED */

              if (cpu != me)
                {
                  up_cpu_resume(cpu);
                }
            }
        }

      /* Otherwise, we are (1) signaling a task is not running from an
       * interrupt handler or (2) we are not in an interrupt handler and the
       * running task is signaling some other non-running task.
       */

      else
        {
          /* Save the return pc and ps.  These will be restored by the
           * signal trampoline after the signals have been delivered.
           *
           * NOTE: that hi-priority interrupts are not disabled.
           */

          tcb->xcp.sigdeliver   = sigdeliver;
          tcb->xcp.saved_pc     = tcb->xcp.regs[REG_PC];
          tcb->xcp.saved_ps     = tcb->xcp.regs[REG_PS];

          /* Increment the IRQ lock count so that when the task is restarted,
           * it will hold the IRQ spinlock.
           */

          DEBUGASSERT(tcb->irqcount < INT16_MAX);
          tcb->irqcount++;

          /* Then set up to vector to the trampoline with interrupts
           * disabled
           */

          tcb->xcp.regs[REG_PC] = (uint32_t)_xtensa_sig_trampoline;
#ifdef __XTENSA_CALL0_ABI__
          tcb->xcp.regs[REG_PS] = (uint32_t)
              (PS_INTLEVEL(XCHAL_EXCM_LEVEL) | PS_UM);
#else
          tcb->xcp.regs[REG_PS] = (uint32_t)
              (PS_INTLEVEL(XCHAL_EXCM_LEVEL) | PS_UM | PS_WOE);
#endif
        }
    }

  leave_critical_section(flags);
}
#endif /* CONFIG_SMP */
