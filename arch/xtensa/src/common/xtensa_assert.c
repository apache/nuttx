/****************************************************************************
 * arch/xtensa/src/common/xtensa_assert.c
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
#include <stdlib.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/syslog/syslog.h>
#include <nuttx/usb/usbdev_trace.h>

#include <arch/board/board.h>

#include "sched/sched.h"
#include "xtensa.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* USB trace dumping */

#ifndef CONFIG_USBDEV_TRACE
#  undef CONFIG_ARCH_USBDUMP
#endif

#ifndef CONFIG_BOARD_RESET_ON_ASSERT
#  define CONFIG_BOARD_RESET_ON_ASSERT 0
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: assert_tracecallback
 ****************************************************************************/

#ifdef CONFIG_ARCH_USBDUMP
static int usbtrace_syslog(FAR const char *fmt, ...)
{
  va_list ap;

  /* Let vsyslog do the real work */

  va_start(ap, fmt);
  vsyslog(LOG_EMERG, fmt, ap);
  va_end(ap);
  return OK;
}

static int assert_tracecallback(FAR struct usbtrace_s *trace, FAR void *arg)
{
  usbtrace_trprintf(usbtrace_syslog, trace->event, trace->value);
  return 0;
}
#endif

/****************************************************************************
 * Name: xtensa_assert
 ****************************************************************************/

static void xtensa_assert(void)
{
  /* Dump the processor state */

  xtensa_dumpstate();

#ifdef CONFIG_ARCH_USBDUMP
  /* Dump USB trace data */

  usbtrace_enumerate(assert_tracecallback, NULL);
#endif

#ifdef CONFIG_BOARD_CRASHDUMP
  /* Perform board-specific crash dump */

  board_crashdump(up_getsp(), running_task(), filename, lineno);
#endif

  /* Flush any buffered SYSLOG data (from the above) */

  syslog_flush();

  /* Are we in an interrupt handler or the idle task? */

  if (CURRENT_REGS || running_task()->flink == NULL)
    {
      /* Blink the LEDs forever */

      up_irq_save();
      for (; ; )
        {
#if CONFIG_BOARD_RESET_ON_ASSERT >= 1
          board_reset(CONFIG_BOARD_ASSERT_RESET_VALUE);
#endif
#ifdef CONFIG_ARCH_LEDS
          board_autoled_on(LED_PANIC);
          up_mdelay(250);
          board_autoled_off(LED_PANIC);
          up_mdelay(250);
#endif
        }
    }
  else
    {
      /* Assertions in other contexts only cause the thread to exit */

#if CONFIG_BOARD_RESET_ON_ASSERT >= 2
      board_reset(CONFIG_BOARD_ASSERT_RESET_VALUE);
#endif
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_assert
 ****************************************************************************/

void up_assert(const char *filename, int lineno)
{
  board_autoled_on(LED_ASSERTION);

  /* Flush any buffered SYSLOG data (from prior to the assertion) */

  syslog_flush();

#if CONFIG_TASK_NAME_SIZE > 0
  _alert("Assertion failed at file:%s line: %d task: %s\n",
         filename, lineno, running_task()->name);
#else
  _alert("Assertion failed at file:%s line: %d\n",
         filename, lineno);
#endif

  xtensa_assert();
}

/****************************************************************************
 * Name: xtensa_panic
 *
 * Description:
 *   PANIC if an unhandled exception is received:
 *
 *   - NMI exception
 *   - Debug exception
 *   - Double exception
 *   - Kernel exception
 *   - Co-processor exception
 *   - High priority level2-6 Exception.
 *
 * Input Parameters:
 *   xcptcode - Identifies the unhandled exception (see include/esp32/irq.h)
 *   regs - The register save are at the time of the interrupt.
 *
 * Returned Value:
 *   Does not return
 *
 ****************************************************************************/

void xtensa_panic(int xptcode, uint32_t *regs)
{
  /* We get here when a un-dispatch-able, irrecoverable exception occurs */

  board_autoled_on(LED_ASSERTION);

  /* Flush any buffered SYSLOG data (from prior to the panic) */

  syslog_flush();

#if CONFIG_TASK_NAME_SIZE > 0
  _alert("Unhandled Exception %d task: %s\n", xptcode, running_task()->name);
#else
  _alert("Unhandled Exception %d\n", xptcode);
#endif

  CURRENT_REGS = regs;
  xtensa_assert(); /* Should not return */
  for (; ; );
}

/****************************************************************************
 * Name: xtensa_user
 *
 * Description:
 *   PANIC if certain User Exceptions are received.  All values for EXCCAUSE
 *   are listed below (not all generate PANICs):
 *
 *   0  IllegalInstructionCause
 *      Illegal instruction
 *   1  SyscallCause
 *      SYSCALL instruction
 *   2  InstructionFetchErrorCause
 *      Processor internal physical address or data error during instruction
 *      fetch.
 *   3  LoadStoreErrorCause
 *      Processor internal physical address or data error during load or
 *      store.
 *   4  Level1InterruptCause
 *      Level-1 interrupt as indicated by set level-1 bits in the INTERRUPT
 *      register.
 *   5  AllocaCause
 *      MOVSP instruction, if caller’s registers are not in the register
 *      file.
 *   6  IntegerDivideByZeroCause
 *      QUOS, QUOU, REMS, or REMU divisor operand is zero.
 *   7  PCValueErrorCause Next PC Value Illegal
 *   8  PrivilegedCause
 *      Attempt to execute a privileged operation when CRING != 0
 *   9  LoadStoreAlignmentCause
 *      Load or store to an unaligned address.
 *   10..11 Reserved for Cadence
 *   12 InstrPIFDataErrorCause
 *      PIF data error during instruction fetch.
 *   13 LoadStorePIFDataErrorCause
 *      Synchronous PIF data error during LoadStore access.
 *   14 InstrPIFAddrErrorCause
 *      PIF address error during instruction fetch.
 *   15 LoadStorePIFAddrErrorCause
 *      Synchronous PIF address error during LoadStore access.
 *   16 InstTLBMissCause
 *      Error during Instruction TLB refill
 *   17 InstTLBMultiHitCause
 *      Multiple instruction TLB entries matched
 *   18 InstFetchPrivilegeCause
 *      An instruction fetch referenced a virtual address at a ring leve
 *      less than CRING.
 *   19 Reserved for Cadence
 *   20 InstFetchProhibitedCause
 *       An instruction fetch referenced a page mapped with an attribute
 *      that does not permit instruction fetch.
 *   21..23 Reserved for Cadence
 *   24 LoadStoreTLBMissCause
 *      Error during TLB refill for a load or store.
 *   25 LoadStoreTLBMultiHitCause
 *      Multiple TLB entries matched for a load or store.
 *   26 LoadStorePrivilegeCause
 *      A load or store referenced a virtual address at a ring level less
 *      than CRING.
 *   27 Reserved for Cadence
 *   28 LoadProhibitedCause
 *      A load referenced a page mapped with an attribute that does not
 *      permit loads.
 *   29 StoreProhibitedCause
 *     A store referenced a page mapped with an attribute that does not
 *     permit stores.
 *   30..31 Reserved for Cadence
 *   32..39 CoprocessornDisabled
 *     Coprocessor n instruction when cpn disabled. n varies 0..7 as the
 *     cause varies 32..39.
 *   40..63 Reserved
 *
 * Input Parameters:
 *   exccause - Identifies the EXCCAUSE of the user exception
 *   regs - The register save are at the time of the interrupt.
 *
 * Returned Value:
 *   Does not return
 *
 ****************************************************************************/

void xtensa_user_panic(int exccause, uint32_t *regs)
{
  /* We get here when a un-dispatch-able, irrecoverable exception occurs */

  board_autoled_on(LED_ASSERTION);

  /* Flush any buffered SYSLOG data (from prior to the error) */

  syslog_flush();

#if CONFIG_TASK_NAME_SIZE > 0
  _alert("User Exception: EXCCAUSE=%04x task: %s\n",
         exccause, running_task()->name);
#else
  _alert("User Exception: EXCCAUSE=%04x\n", exccause);
#endif

  CURRENT_REGS = regs;
  xtensa_assert(); /* Should not return */
  for (; ; );
}
