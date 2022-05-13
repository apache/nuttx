/****************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_start.c
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
#include <arch/board/board.h>
#include <arch/irq.h>

#include "arm_internal.h"
#include "nvic.h"

#ifdef CONFIG_BUILD_PROTECTED
#  include "s32k1xx_userspace.h"
#endif

#include "hardware/s32k1xx_lmem.h"
#include "s32k1xx_clockconfig.h"
#include "s32k1xx_lowputc.h"
#include "s32k1xx_serial.h"
#include "s32k1xx_wdog.h"
#include "s32k1xx_start.h"
#if defined(CONFIG_ARCH_USE_MPU) && defined(CONFIG_S32K1XX_ENET)
#include "hardware/s32k1xx_mpu.h"
#endif

#ifdef CONFIG_S32K1XX_PROGMEM
#include "s32k1xx_progmem.h"
#endif

#ifdef CONFIG_S32K1XX_EEEPROM
#include "s32k1xx_eeeprom.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Memory Map ***************************************************************/

/* 0x0000:0000 - Beginning of the internal FLASH.   Address of vectors.
 *               Mapped as boot memory address 0x0000:0000 at reset.
 * 0x07ff:ffff - End of flash region (assuming the max of 2MiB of FLASH).
 * 0x1000:0000 - Start of internal SRAM and start of .data (_sdata)
 *
 *               The on-chip RAM is split in two regions: SRAM_L and SRAM_U.
 *               The RAM is implemented such that the SRAM_L and SRAM_U
 *               ranges form a contiguous block in the memory map.  Thus, the
 *               actual SRAM start address is SAM_L which some MCU-specific
 *               value in the range 0x1000:0000 and 0x1fff:ffff.  SRAM_U
 *               then always starts at 0x2000:0000

 *             - End of .data (_edata) and start of .bss (_sbss)
 *             - End of .bss (_ebss) and bottom of idle stack
 *             - _ebss + CONFIG_IDLETHREAD_STACKSIZE = end of idle stack,
 *               start of heap. NOTE that the ARM uses a decrement before
 *               store stack so that the correct initial value is the end of
 *               the stack + 4;
 * 0x2fff:ffff - End of internal SRAM and end of heap.  The actual end of
 *               SRAM_U will depend on the amount of memory supported by the
 *               MCU/
 *
 * NOTE:  ARM EABI requires 64 bit stack alignment.
 */

#define HEAP_BASE      ((uintptr_t)&_ebss + CONFIG_IDLETHREAD_STACKSIZE)

/****************************************************************************
 * Name: showprogress
 *
 * Description:
 *   Print a character on the UART to show boot status.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#  define showprogress(c) s32k1xx_lowputc(c)
#else
#  define showprogress(c)
#endif

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
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k1xx_cache_config
 *
 * Description:
 *   IInvalidate and enable code cache.
 *
 ****************************************************************************/

#ifdef CONFIG_S32K1XX_HAVE_LMEM
static inline void s32k1xx_cache_config(void)
{
  uint32_t regval;

  /* Invalidate and enable code cache */

  regval = (LMEM_PCCCR_ENCACHE | LMEM_PCCCR_INVW0 | LMEM_PCCCR_INVW1 |
            LMEM_PCCCR_GO);
  putreg32(regval, S32K1XX_LMEM_PCCCR);
}
#endif

/****************************************************************************
 * Name: s32k1xx_mpu_config
 *
 * Description:
 *   Enable all bus masters.
 *
 ****************************************************************************/

#if defined(CONFIG_ARCH_USE_MPU) && defined(CONFIG_S32K1XX_ENET)
static inline void s32k1xx_mpu_config(void)
{
  uint32_t regval;

  /* Bus masters 0-2 are already enabled r/w/x in supervisor and user modes
   * after reset.  Enable also bus master 3 (ENET) in S/U modes in default
   * region 0:  User=r+w+x, Supervisor=same as used.
   */

  regval = (MPU_RGDAAC_M3UM_XACCESS | MPU_RGDAAC_M3UM_WACCESS |
            MPU_RGDAAC_M3UM_RACCESS | MPU_RGDAAC_M3SM_M3UM);

  putreg32(regval, S32K1XX_MPU_RGDAAC(0));
}
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

  /* Make sure that interrupts are disabled */

  __asm__ __volatile__ ("\tcpsid  i\n");

#ifdef CONFIG_S32K1XX_WDT_DISABLE
  /* Disable the watchdog timer */

  s32k1xx_wdog_disable();
#endif

#ifdef CONFIG_S32K1XX_HAVE_LMEM
  /* Initialize the cache (if supported) */

  s32k1xx_cache_config();
#endif

  /* Clear .bss.  We'll do this inline (vs. calling memset) just to be
   * certain that there are no issues with the state of global variables.
   */

  for (dest = &_sbss; dest < &_ebss; )
    {
      *dest++ = 0;
    }

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
#endif

  /* Copy any necessary code sections from FLASH to RAM.  The correct
   * destination in SRAM is given by _sramfuncs and _eramfuncs.  The
   * temporary location is in flash after the data initialization code
   * at _framfuncs.  This should be done before s32k1xx_clockconfig() is
   * called (in case it has some dependency on initialized C variables).
   */

#ifdef CONFIG_ARCH_RAMFUNCS
  for (src = &_framfuncs, dest = &_sramfuncs; dest < &_eramfuncs; )
    {
      *dest++ = *src++;
    }
#endif

  /* Configure the clocking and the console uart so that we can get debug
   * output as soon as possible.  NOTE: That this logic must not assume that
   * .bss or .data have been initialized.
   */

  DEBUGVERIFY(s32k1xx_clockconfig(&g_initial_clkconfig));
  s32k1xx_lowsetup();
  showprogress('B');

  /* Initialize the FPU (if configured) */

  arm_fpuconfig();
  showprogress('C');

#if defined(CONFIG_ARCH_USE_MPU) && defined(CONFIG_S32K1XX_ENET)

  /* Enable all MPU bus masters */

  s32k1xx_mpu_config();
  showprogress('D');
#endif

  /* Perform early serial initialization */

#ifdef USE_EARLYSERIALINIT
  s32k1xx_earlyserialinit();
#endif
  showprogress('E');

#ifdef CONFIG_S32K1XX_PROGMEM
  s32k1xx_progmem_init();
#endif

#ifdef CONFIG_S32K1XX_EEEPROM
  s32k1xx_eeeprom_init();
#endif

  /* For the case of the separate user-/kernel-space build, perform whatever
   * platform specific initialization of the user memory is required.
   * Normally this just means initializing the user space .data and .bss
   * segments.
   */

#ifdef CONFIG_BUILD_PROTECTED
  s32k1xx_userspace();
  showprogress('F');
#endif

  /* Initialize on-board resources */

  s32k1xx_board_initialize();
  showprogress('G');

  /* Then start NuttX */

  showprogress('\r');
  showprogress('\n');
  nx_start();

  /* Shouldn't get here */

  for (; ; );
}
