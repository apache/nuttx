/****************************************************************************
 * arch/arm/src/lpc43xx/lpc43_start.c
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

/* Power-Up Reset Overview
 * -----------------------
 *
 * The ARM core starts executing code on reset with the program counter set
 * to 0x0000:0000.  The LPC43xx contains a shadow pointer register that
 * allows areas of memory to be mapped to address 0x0000:0000. The default,
 * reset value of the shadow pointer is 0x1040:0000 so that on reset code in
 * the boot ROM is always executed first.
 *
 * The boot starts after reset is released.  The IRC is selected as CPU clock
 * and the Cortex-M4 starts the boot loader. By default the JTAG access to
 * the chip is disabled at reset.  The boot ROM determines the boot mode
 * based on the OTP BOOT_SRC value or reset state pins.  For flash-based
 * parts, the part boots from internal flash by default.  Otherwise, the boot
 * ROM copies the image to internal SRAM at location 0x1000:0000, sets the
 * ARM's shadow pointer to 0x1000:0000, and jumps to that location.
 *
 * However, using JTAG the executable image can be also loaded directly into
 * and executed from SRAM.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/init.h>
#include <arch/irq.h>

#include "arm_internal.h"
#include "nvic.h"

#include "hardware/lpc43_creg.h"

#include "lpc43_rgu.h"
#include "lpc43_cgu.h"
#include "lpc43_emc.h"
#include "lpc43_uart.h"
#include "lpc43_userspace.h"
#include "lpc43_start.h"

/****************************************************************************
 * Pre-processor Definitions
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
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_setbootrom
 *
 * Description:
 *   Set the shadow register to 0x1040:0000 and the VTOR to 0x0000:0000 so
 *   that any exceptions (particularly things like hard faults) that occur
 *   before we are initialized are caught by the BOOT ROM.
 *
 ****************************************************************************/

static inline void lpc43_setbootrom(void)
{
  /* Set the shadow register to the beginning of the boot ROM
   * (Only bits 12-31)
   */

  putreg32(LPC43_ROM_BASE, LPC43_CREG_M4MEMMAP);

  /* Address zero now maps to the Boot ROM.  Make sure that the VTOR will
   * use the ROM vector table at that address.
   */

  putreg32(0, NVIC_VECTAB);
}

/****************************************************************************
 * Name: lpc43_enabuffering
 *
 * Description:
 *   If we are executing from external FLASH, then enable buffering.
 *
 ****************************************************************************/

#if defined(CONFIG_LPC43_BOOT_CS0FLASH) || defined(CONFIG_LPC43_BOOT_CS1FLASH) || \
    defined(CONFIG_LPC43_BOOT_CS2FLASH) || defined(CONFIG_LPC43_BOOT_CS3FLASH)
static inline void lpc43_enabuffering(void)
{
  uint32_t regval;

#ifdef CONFIG_LPC43_BOOT_CS0FLASH
  regval = getreg32(LPC43_EMC_STATCONFIG0);
  regval |= EMC_STATCONFIG_BENA
  putreg32(regval, LPC43_EMC_STATCONFIG0);
#endif

#ifdef CONFIG_LPC43_BOOT_CS1FLASH
  regval = getreg32(LPC43_EMC_STATCONFIG1);
  regval |= EMC_STATCONFIG_BENA
  putreg32(regval, LPC43_EMC_STATCONFIG1);
#endif

#ifdef CONFIG_LPC43_BOOT_CS2FLASH
  regval = getreg32(LPC43_EMC_STATCONFIG2);
  regval |= EMC_STATCONFIG_BENA
  putreg32(regval, LPC43_EMC_STATCONFIG2);
#endif

#ifdef CONFIG_LPC43_BOOT_CS3FLASH
  regval = getreg32(LPC43_EMC_STATCONFIG3);
  regval |= EMC_STATCONFIG_BENA
  putreg32(regval, LPC43_EMC_STATCONFIG3);
#endif
}
#else
#  define lpc43_enabuffering()
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
  const uint32_t *src;
  uint32_t *dest;

  /* Reset as many of the LPC43 peripherals as possible. This is necessary
   * because the LPC43 does not provide any way of performing a full system
   * reset under debugger control.  So, if CONFIG_DEBUG_FEATURES is set
   * (indicating that a debugger is being used?), the boot logic will call
   * this function on all restarts.
   */

#ifdef CONFIG_DEBUG_FEATURES
  lpc43_softreset();
#endif

  /* Make sure that any exceptions (such as hard faults) that occur before
   * we are initialized are caught by the BOOT ROM.
   */

  lpc43_setbootrom();

  /* Configure the CGU clocking and the console uart so that we can get
   * debug output as soon as possible.
   */

  lpc43_clockconfig();
  lpc43_lowsetup();
  showprogress('A');

  /* If we are executing from external FLASH, then enable buffering */

  lpc43_enabuffering();

  /* Clear .bss.  We'll do this inline (vs. calling memset) just to be
   * certain that there are no issues with the state of global variables.
   */

  for (dest = &_sbss; dest < &_ebss; )
    {
      *dest++ = 0;
    }

  showprogress('B');

  /* Move the initialized data section from his temporary holding spot in
   * FLASH into the correct place in SRAM.  The correct place in SRAM is
   * give by _sdata and _edata.  The temporary location is in FLASH at the
   * end of all of the other read-only data (.text, .rodata) at _eronly.
   */

  for (src = &_eronly, dest = &_sdata; dest < &_edata; )
    {
      *dest++ = *src++;
    }

  showprogress('C');

  /* Initialize the FPU (if configured) */

  arm_fpuconfig();
  showprogress('D');

  /* Perform early serial initialization */

#ifdef USE_EARLYSERIALINIT
  arm_earlyserialinit();
#endif
  showprogress('E');

  /* For the case of the separate user-/kernel-space build, perform whatever
   * platform specific initialization of the user memory is required.
   * Normally this just means initializing the user space .data and .bss
   * segments.
   */

#ifdef CONFIG_BUILD_PROTECTED
  lpc43_userspace();
  showprogress('F');
#endif

  /* Initialize onboard resources */

  lpc43_boardinitialize();
  showprogress('G');

  /* Then start NuttX */

  showprogress('\r');
  showprogress('\n');
  nx_start();

  /* Shouldn't get here */

  for (; ; );
}
