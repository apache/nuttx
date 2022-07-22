/****************************************************************************
 * arch/arm/src/s32k3xx/s32k3xx_start.c
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

/* Copyright 2022 NXP */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/cache.h>
#include <nuttx/init.h>
#include <arch/board/board.h>
#include <arch/irq.h>

#include "arm_internal.h"
#include "nvic.h"

#ifdef CONFIG_BUILD_PROTECTED
#  include "s32k3xx_userspace.h"
#endif

#include "hardware/s32k3xx_mcm.h"
#include "hardware/s32k3xx_mc_me.h"
#include "s32k3xx_clockconfig.h"
#include "s32k3xx_lowputc.h"
#include "s32k3xx_serial.h"
#include "s32k3xx_swt.h"
#include "s32k3xx_start.h"
#if defined(CONFIG_ARCH_USE_MPU) && defined(CONFIG_S32K3XX_ENET)
#include "hardware/s32k3xx_mpu.h"
#endif

#ifdef CONFIG_S32K3XX_PROGMEM
#include "s32k3xx_progmem.h"
#endif

#ifdef CONFIG_S32K3XX_EEEPROM
#include "s32k3xx_eeeprom.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Memory Map ***************************************************************/

/* 0x0040:1000 - Beginning of the internal FLASH.   Address of vectors.
 *               Mapped as boot memory address CM7_0_START_ADDRESS at reset.
 * 0x007d:0fff - End of flash region (assuming the max of 2MiB of FLASH).
 * 0x2040:8000 - Start of internal SRAM and start of .data (_sdata)
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
 * 0x2044:4000 - End of internal SRAM and end of heap.  The actual end of
 *               SRAM_U will depend on the amount of memory supported by the
 *               MCU/
 *
 * NOTE:  ARM EABI requires 64 bit stack alignment.
 */

#define STARTUP_ECC_INITVALUE   0

/****************************************************************************
 * Name: showprogress
 *
 * Description:
 *   Print a character on the UART to show boot status.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#  define showprogress(c) s32k3xx_lowputc(c)
#else
#  define showprogress(c)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

extern const uint32_t SRAM_BASE_ADDR;
extern const uint32_t SRAM_END_ADDR;
extern const uint32_t ITCM_BASE_ADDR;
extern const uint32_t ITCM_END_ADDR;
extern const uint32_t DTCM_BASE_ADDR;
extern const uint32_t DTCM_END_ADDR;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k3xx_mpu_config
 *
 * Description:
 *   Enable all bus masters.
 *
 ****************************************************************************/

#if defined(CONFIG_ARCH_USE_MPU) && defined(CONFIG_S32K3XX_ENET)
static inline void s32k3xx_mpu_config(void)
{
  uint32_t regval;

  /* Bus masters 0-2 are already enabled r/w/x in supervisor and user modes
   * after reset.  Enable also bus master 3 (ENET) in S/U modes in default
   * region 0:  User=r+w+x, Supervisor=same as used.
   */

  regval = (MPU_RGDAAC_M3UM_XACCESS | MPU_RGDAAC_M3UM_WACCESS |
            MPU_RGDAAC_M3UM_RACCESS | MPU_RGDAAC_M3SM_M3UM);

  putreg32(regval, S32K3XX_MPU_RGDAAC(0));
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

#define STR(x) #x
#define XSTR(s) STR(s)

void s32k3xx_start(void)
{
  register uint64_t *src;
  register uint64_t *dest;

  /* Technically startup.S did initialize SRAM
   * but if don't set init value here again
   * then on a cold boot we go into a bootloop somehow
   */

  dest = (uint64_t *)&SRAM_BASE_ADDR;
  while (dest < (uint64_t *)&SRAM_END_ADDR)
    {
      *dest++ = STARTUP_ECC_INITVALUE;
    }

  /* ITCM */

  dest = (uint64_t *)&ITCM_BASE_ADDR;
  while (dest < (uint64_t *)&ITCM_END_ADDR)
    {
      *dest++ = STARTUP_ECC_INITVALUE;
    }

  /* DTCM */

  dest = (uint64_t *)&DTCM_BASE_ADDR;
  while (dest < (uint64_t *)&DTCM_END_ADDR)
    {
      *dest++ = STARTUP_ECC_INITVALUE;
    }

  /* Clear .bss.  We'll do this inline (vs. calling memset) just to be
   * certain that there are no issues with the state of global variables.
   */

  for (dest = (uint64_t *)&_sbss; dest < (uint64_t *)&_ebss; )
    {
      *dest++ = 0;
    }

#ifdef CONFIG_BOOT_RUNFROMFLASH
  /* Move the initialized data section from his temporary holding spot in
   * FLASH into the correct place in SRAM.  The correct place in SRAM is
   * give by _sdata and _edata.  The temporary location is in FLASH at the
   * end of all of the other read-only data (.text, .rodata) at _eronly.
   */

  for (src = (uint64_t *)&_eronly, dest = (uint64_t *)&_sdata;
     dest < (uint64_t *)&_edata;
      )
    {
      *dest++ = *src++;
    }
#endif

  /* Copy any necessary code sections from FLASH to RAM.  The correct
   * destination in SRAM is given by _sramfuncs and _eramfuncs.  The
   * temporary location is in flash after the data initialization code
   * at _framfuncs.  This should be done before s32k3xx_clockconfig() is
   * called (in case it has some dependency on initialized C variables).
   */

#ifdef CONFIG_ARCH_RAMFUNCS
  for (src = (uint64_t *)&_framfuncs, dest = (uint64_t *)&_sramfuncs;
     dest < (uint64_t *)&_eramfuncs;
      )
    {
      *dest++ = *src++;
    }
#endif

  /* Configure the clocking and the console uart so that we can get debug
   * output as soon as possible.  NOTE: That this logic must not assume that
   * .bss or .data have been initialized.
   */

  DEBUGVERIFY(s32k3xx_clockconfig(&g_initial_clkconfig));
  s32k3xx_lowsetup();
  showprogress('B');

  /* Initialize the FPU (if configured) */

  arm_fpuconfig();

  /* Enable I- and D-Caches */

  up_enable_icache();
  up_enable_dcache();

  showprogress('C');

#if defined(CONFIG_ARCH_USE_MPU) && defined(CONFIG_S32K3XX_ENET)

  /* Enable all MPU bus masters */

  s32k3xx_mpu_config();
  showprogress('D');
#endif

  /* Perform early serial initialization */

#ifdef USE_EARLYSERIALINIT
  s32k3xx_earlyserialinit();
#endif
  showprogress('E');

#ifdef CONFIG_S32K3XX_PROGMEM
  s32k3xx_progmem_init();
#endif

#ifdef CONFIG_S32K3XX_EEEPROM
  s32k3xx_eeeprom_init();
#endif

  /* For the case of the separate user-/kernel-space build, perform whatever
   * platform specific initialization of the user memory is required.
   * Normally this just means initializing the user space .data and .bss
   * segments.
   */

#ifdef CONFIG_BUILD_PROTECTED
  s32k3xx_userspace();
  showprogress('F');
#endif

  /* Initialize on-board resources */

  showprogress('G');

  s32k3xx_board_initialize();

  /* Then start NuttX */

  showprogress('\r');
  showprogress('\n');
  nx_start();

  /* Shouldn't get here */

  for (; ; );
}
