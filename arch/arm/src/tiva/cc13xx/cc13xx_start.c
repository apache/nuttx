/****************************************************************************
 * arch/arm/src/tiva/cc13xx/cc13xx_start.c
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

#include <nuttx/init.h>
#include <arch/irq.h>

#include "nvic.h"
#include "arm_internal.h"
#include "tiva_enablepwr.h"
#include "tiva_enableclks.h"
#include "tiva_lowputc.h"
#include "tiva_userspace.h"
#include "tiva_eeprom.h"
#include "tiva_start.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* .data is positioned first in the primary RAM followed immediately by .bss.
 * The IDLE thread stack lies just after .bss and has size give by
 * CONFIG_IDLETHREAD_STACKSIZE;  The heap then begins just after the IDLE.
 * ARM EABI requires 64 bit stack alignment.
 */

#define HEAP_BASE      ((uintptr_t)&_ebss + CONFIG_IDLETHREAD_STACKSIZE)

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

const uintptr_t g_idle_topstack = HEAP_BASE;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* REVISIT: Should be moved to a header file */

void cc13xx_trim_device(void);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: showprogress
 *
 * Description:
 *   Print a character on the UART to show boot status.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#  define showprogress(c) arm_lowputc(c)
#else
#  define showprogress(c)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __start
 *
 * Description:
 *   This is the reset entry point.
 *
 ****************************************************************************/

void __start(void)
{
#ifdef CONFIG_BOOT_RUNFROMFLASH
  const uint32_t *src;
#endif
  uint32_t *dest;

  /* Perform the necessary trim of the device which is not done in boot
   * code.
   */

  cc13xx_trim_device();

  /* Clear .bss.  We'll do this inline (vs. calling memset) just to be
   * certain that there are no issues with the state of global variables.
   */

  for (dest = &_sbss; dest < &_ebss; )
    {
      *dest++ = 0;
    }

  /* Enable power and clocking for the GPIO peripheral. */

  tiva_gpio_enablepwr();
  tiva_gpio_enableclk();

  /* Configure the UART so that we can get debug output as soon as
   * possible
   */

  tiva_lowsetup();
  arm_fpuconfig();
  showprogress('A');

#ifdef CONFIG_BOOT_RUNFROMFLASH
  /* Move the initialized data section from his temporary holding spot in
   * FLASH into the correct place in SRAM.  The correct place in SRAM is
   * give by _sdata and _edata.  The temporary location is in FLASH at the
   * end of all of the other read-only data (.text, .rodata) at _eronly.
   */

  for (src = &_eronly, dest = &_sdata; dest < &_edata; )
    {
      *dest++ = *src++;
    }

  showprogress('B');
#endif

#ifdef USE_EARLYSERIALINIT
  /* Perform early serial initialization */

  arm_earlyserialinit();
  showprogress('C');
#endif

#ifdef CONFIG_BUILD_PROTECTED
  /* For the case of the separate user-/kernel-space build, perform whatever
   * platform specific initialization of the user memory is required.
   * Normally this just means initializing the user space .data and .bss
   * segments.
   */

  tiva_userspace();
  showprogress('D');
#endif

#ifdef CONFIG_TIVA_CC26X2_POWERLIB /* REVISIT:  Used with CC13x2 as well. */
  /* Initialize the Power Manager internal state.  This must be called prior
   * to any other Power API.
   */

  cc13xx_power_initialize();
  showprogress('E');
#endif

  /* Initialize on-board resources */

  tiva_boardinitialize();
  showprogress('F');

#ifdef CONFIG_TIVA_EEPROM
  /* Initialize the EEPROM */

  tiva_eeprom_initialize();
  showprogress('G');
#endif

  /* Then start NuttX */

  showprogress('\r');
  showprogress('\n');
  nx_start();

  /* Shouldn't get here */

  for (; ; );
}
