/****************************************************************************
 * arch/arm/src/tlsr82/tlsr82_start.c
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

#include <nuttx/arch.h>
#include <nuttx/init.h>

#include <syslog.h>

#include "arm_internal.h"

#include "hardware/tlsr82_gpio.h"
#include "tlsr82_gpio.h"
#include "tlsr82_gpio_cfg.h"
#include "tlsr82_serial.h"
#include "tlsr82_spi_console.h"
#include "tlsr82_clock.h"
#include "tlsr82_cpu.h"
#include "tlsr82_flash.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_idle_topstack: _sbss is the start of the BSS region as defined by the
 * linker script. _ebss lies at the end of the BSS region. The idle task
 * stack starts at the end of BSS and is of size CONFIG_IDLETHREAD_STACKSIZE.
 * The IDLE thread is the thread that the system boots on and, eventually,
 * becomes the IDLE, do nothing task that runs only when there is nothing
 * else to run.  The heap continues from there until the end of memory.
 * g_idle_topstack is a read-only variable the provides this computed
 * address.
 */

const uintptr_t g_idle_topstack = (uintptr_t)(&_ebss) +
                                  CONFIG_IDLETHREAD_STACKSIZE;

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ____tc32_start
 *
 * Description:
 *   This is the reset entry point.
 *
 ****************************************************************************/

void __tc32_start(void)
{
  tlsr82_cpu_wakeup_init(LDO_MODE, EXTERNAL_XTAL_24M);

  tlsr82_gpio_init();

  tlsr82_clock_init();

#ifdef CONFIG_SCHED_BACKTRACE
  extern uint32_t _sramcode;
  extern uint32_t _eramcode;
  static void *g_code_regions[] =
  {
    &_stext   , &_etext,
    &_sramcode, &_eramcode,
    NULL      , NULL,
  };

  extern void up_backtrace_init_code_regions(void **regions);
  up_backtrace_init_code_regions(g_code_regions);
#endif

#ifdef CONFIG_TLSR82_SPI
  spi_console_early_init();
#endif

  tlsr82_earlyserialinit();

  nx_start();

  /* Never reach here */

  for (; ; );
}
