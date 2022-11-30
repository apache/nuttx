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
#include "barriers.h"
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
#if defined(CONFIG_ARCH_USE_MPU)
#include "mpu.h"
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

#ifndef CONFIG_ARMV7M_DCACHE
  /*  With Dcache off:
   *  Cacheable (MPU_RASR_C) and Bufferable (MPU_RASR_B) needs to be off
   */
#  undef  MPU_RASR_B
#  define MPU_RASR_B    0
#  define RASR_B_VALUE  0
#  define RASR_C_VALUE  0
#else
#  ifndef CONFIG_ARMV7M_DCACHE_WRITETHROUGH
  /*  With Dcache on:
   *  Cacheable (MPU_RASR_C) and Bufferable (MPU_RASR_B) needs to be on
   */
#  define RASR_B_VALUE  MPU_RASR_B
#  define RASR_C_VALUE  MPU_RASR_C

#  else
  /*  With Dcache in WRITETHROUGH Bufferable (MPU_RASR_B)
   * needs to be off, except for FLASH for alignment leniency
   */
#  define RASR_B_VALUE  0
#  define RASR_C_VALUE  MPU_RASR_C
#  endif
#endif

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

extern uint8_t SRAM_BASE_ADDR[];
extern uint8_t SRAM_INIT_END_ADDR[];
extern uint8_t ITCM_BASE_ADDR[];
extern uint8_t ITCM_END_ADDR[];
extern uint8_t DTCM_BASE_ADDR[];
extern uint8_t DTCM_END_ADDR[];
extern uint8_t FLASH_BASE_ADDR[];
extern uint8_t FLASH_END_ADDR[];

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

#if defined(CONFIG_ARCH_USE_MPU)
static inline void s32k3xx_mpu_config(void)
{
  uint32_t regval;
  uint32_t region;

  /* Show MPU information */

  mpu_showtype();

#ifdef CONFIG_ARMV7M_DCACHE
  /* Memory barrier */

  ARM_DMB();
#endif

  /* Reset MPU if enabled */

  mpu_reset();

  /* ARM errata 1013783-B Workaround */

  region = mpu_allocregion();
  DEBUGASSERT(region == 0);

  /* Select the region */

  putreg32(region, MPU_RNR);

  /* Select the region base address */

  putreg32(region | MPU_RBAR_VALID, MPU_RBAR);

  /* The configure the region */

  regval = MPU_RASR_ENABLE        | /* Enable region  */
           MPU_RASR_SIZE_LOG2(32) | /* entire memory */
           MPU_RASR_TEX_SO        | /* Strongly ordered */
           MPU_RASR_AP_RWRW       | /* P:RW   U:RW */
           MPU_RASR_XN;             /* Execute-never to prevent instruction fetch */
  putreg32(regval, MPU_RASR);

  mpu_configure_region((uintptr_t)FLASH_BASE_ADDR,
                     FLASH_END_ADDR - FLASH_BASE_ADDR,
                     MPU_RASR_TEX_SO   | /* Strongly ordered    */
                     RASR_C_VALUE      | /* Cacheable          */
                     MPU_RASR_B        | /* Bufferable
                                          * Not Shareable      */
                     MPU_RASR_AP_RORO);  /* P:RO   U:RO
                                          * Instruction access */

  mpu_configure_region((uintptr_t)SRAM_BASE_ADDR,
                     SRAM_INIT_END_ADDR - SRAM_BASE_ADDR,
                     MPU_RASR_TEX_SO   | /* Strongly ordered   */
                     RASR_C_VALUE      | /* Cacheable          */
                     RASR_B_VALUE      | /* Bufferable
                                          * Not Shareable      */
                     MPU_RASR_AP_RWRW);  /* P:RW   U:RW
                                          * Instruction access */

  mpu_configure_region((uintptr_t)ITCM_BASE_ADDR,
                     ITCM_END_ADDR - ITCM_BASE_ADDR,
                     MPU_RASR_TEX_SO   | /* Strongly ordered   */
                     RASR_C_VALUE      | /* Cacheable          */
                     RASR_B_VALUE      | /* Bufferable
                                          * Not Shareable      */
                     MPU_RASR_AP_RWRW);  /* P:RW   U:RW
                                          * Instruction access */

  mpu_configure_region((uintptr_t)DTCM_BASE_ADDR,
                     DTCM_END_ADDR - DTCM_BASE_ADDR,
                     MPU_RASR_TEX_SO   | /* Strongly ordered   */
                     RASR_C_VALUE      | /* Cacheable          */
                     RASR_B_VALUE      | /* Bufferable
                                          * Not Shareable      */
                     MPU_RASR_AP_RWRW);  /* P:RW   U:RW
                                          * Instruction access */

  mpu_configure_region(0x40000000, 3 * 2048 * 1024,
                     MPU_RASR_TEX_DEV  | /* Device
                                          * Not Cacheable
                                          * Not Bufferable
                                          * Not Shareable      */
                     MPU_RASR_AP_RWRW);  /* P:RW   U:RW
                                          * Instruction access */

  /* Then enable the MPU */

  mpu_control(true, false, true);
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

void s32k3xx_start(void)
{
  register uint64_t *src;
  register uint64_t *dest;

  /* Technically startup.S did initialize SRAM
   * but if don't set init value here again
   * then on a cold boot we go into a bootloop somehow
   */

  dest = (uint64_t *)SRAM_BASE_ADDR;
  while (dest < (uint64_t *)SRAM_INIT_END_ADDR)
    {
      *dest++ = STARTUP_ECC_INITVALUE;
    }

  /* ITCM */

  dest = (uint64_t *)ITCM_BASE_ADDR;
  while (dest < (uint64_t *)ITCM_END_ADDR)
    {
      *dest++ = STARTUP_ECC_INITVALUE;
    }

  /* DTCM */

  dest = (uint64_t *)DTCM_BASE_ADDR;
  while (dest < (uint64_t *)DTCM_END_ADDR)
    {
      *dest++ = STARTUP_ECC_INITVALUE;
    }

  /* Clear .bss.  We'll do this inline (vs. calling memset) just to be
   * certain that there are no issues with the state of global variables.
   */

  for (dest = (uint64_t *)_sbss; dest < (uint64_t *)_ebss; )
    {
      *dest++ = 0;
    }

#ifdef CONFIG_BOOT_RUNFROMFLASH
  /* Move the initialized data section from his temporary holding spot in
   * FLASH into the correct place in SRAM.  The correct place in SRAM is
   * give by _sdata and _edata.  The temporary location is in FLASH at the
   * end of all of the other read-only data (.text, .rodata) at _eronly.
   */

  for (src = (uint64_t *)_eronly, dest = (uint64_t *)_sdata;
     dest < (uint64_t *)_edata;
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
  for (src = (uint64_t *)_framfuncs, dest = (uint64_t *)_sramfuncs;
     dest < (uint64_t *)_eramfuncs;
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

#if defined(CONFIG_ARCH_USE_MPU)

  /* Config MPU regions */

  s32k3xx_mpu_config();
  showprogress('D');
#endif

  /* Enable I- and D-Caches */

  up_enable_icache();
  up_enable_dcache();

  showprogress('C');

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
