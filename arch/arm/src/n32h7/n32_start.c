/****************************************************************************
 * arch/arm/src/n32h7/n32_start.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <debug.h>

#include <nuttx/cache.h>
#include <nuttx/init.h>
#include <nuttx/board.h>
#include <arch/barriers.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "nvic.h"
#include "mpu.h"
#ifdef CONFIG_ARM_MPU
#  include "n32_mpuinit.h"
#endif

#include "n32_rcc.h"
#include "n32_lowputc.h"
#include "n32_start.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Memory Map ***************************************************************/

extern const uint32_t _sbss_rel[];
extern const uint32_t _ebss_rel[];
extern const uint32_t _sivectors[];
extern const uint32_t _svectors[];
extern const uint32_t _evectors[];
extern const uint32_t _sitext[];
extern const uint32_t __extab_start[];
extern const uint32_t __extab_end[];
extern const uint32_t __exidx_start[];
extern const uint32_t __exidx_end[];

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

const uintptr_t g_idle_topstack = (uintptr_t)_ebss_rel +
                                  CONFIG_IDLETHREAD_STACKSIZE;

/****************************************************************************
 * Private Function prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: showprogress
 *
 * Description:
 *   Print a character on the UART to show boot status.
 *
 ****************************************************************************/

void up_lowputc(int ch)
{
  arm_lowputc(ch);
}
#ifdef CONFIG_DEBUG_FEATURES
#  define showprogress(c) arm_lowputc(c)
#else
#  define showprogress(c)
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_ARMV7M_STACKCHECK
/* we need to get r10 set before we can allow instrumentation calls */

void __start(void) noinstrument_function;
#endif

#ifdef CONFIG_ARCH_CHIP_N32H7_CORTEXM7
/****************************************************************************
 * Name: n32_tcmenable
 *
 * Description:
 *   Enable/disable tightly coupled memories.  Size of tightly coupled
 *   memory regions is controlled by GPNVM Bits 7-8.
 *
 ****************************************************************************/

static inline no_builtin("memcpy") no_builtin("memset")
  void n32_tcmenable(void)
{
  uint32_t regval;

  UP_MB();

  /* Enabled/disabled ITCM */

#ifdef CONFIG_ARMV7M_ITCM
  regval  = NVIC_TCMCR_EN | NVIC_TCMCR_RMW | NVIC_TCMCR_RETEN;
#else
  regval  = getreg32(NVIC_ITCMCR);
  regval &= ~NVIC_TCMCR_EN;
#endif
  putreg32(regval, NVIC_ITCMCR);

  /* Enabled/disabled DTCM */

#ifdef CONFIG_ARMV7M_DTCM

  regval  = NVIC_TCMCR_EN | NVIC_TCMCR_RMW | NVIC_TCMCR_RETEN;
#else
  regval  = getreg32(NVIC_DTCMCR);
  regval &= ~NVIC_TCMCR_EN;
#endif
  putreg32(regval, NVIC_DTCMCR);

  UP_MB();

#ifdef CONFIG_ARMV7M_ITCM
  const uint32_t *src;
  uint32_t *dest;

  /* Copy TCM code from flash to ITCM */

  for (src = _sivectors, dest = (uint32_t *)_svectors; dest < _evectors; )
    {
      *dest++ = *src++;
    }

  for (src = _sitext, dest = (uint32_t *)_stext; dest < (uint32_t *)_etext; )
    {
      *dest++ = *src++;
    }

  for (dest = (uint32_t *)_sinit; dest < (uint32_t *)_einit; )
    {
      *dest++ = *src++;
    }

  for (dest = (uint32_t *)__extab_start; dest < __extab_end; )
    {
      *dest++ = *src++;
    }

  for (dest = (uint32_t *)__exidx_start; dest < __exidx_end; )
    {
      *dest++ = *src++;
    }
#endif
}
#endif

/****************************************************************************
 * Name: n32_tcm_init
 *
 * Description:
 *   Initialize the TCM Controller.
 *   No Memory Access, only register access.
 *
 * Input Parameters:
 *   tcm_size - TCM size config code
 *
 ****************************************************************************/

static inline void n32_tcm_init(uint32_t tcm_size)
{
  __asm__ volatile ("ldr r3, =0x1ff00f01\n\t"
                    "push {r0, lr}\n\t"
                    "blx r3\n\t"
                    "mov r3, r0\n\t"
                    "pop {r0, lr}\n\t"
                    "cmp r3, r0\n\t"
                    "beq tcm_ok\n\t"
                    "ldr r3, =0x51105280\n\t"
                    "mov r2, r0\n\t"
                    "str r2, [r3]\n\t"
                    "ldr r3, =0xE000ED0C\n\t"
                    "ldr r3, [r3]\n\t"
                    "and r2, r3, #0x0700\n\t"
                    "ldr r1, =0xE000ED0C\n\t"
                    "ldr r3, =0x05FA0004\n\t"
                    "orrs r3, r2\n\t"
                    "str r3, [r1]\n\t"
                    "dsb\n\t"
                    "loop:\n\t"
                    "nop\n\t"
                    "b loop\n\t"
                    "tcm_ok:\n\t"
                    "ldr r3, =g_idle_topstack\n\t"
                    "ldr r3, [r3]\n\t"
                    "mov sp, r3\n\t": : "r"(tcm_size) : "r3", "r1", "r2",
                    "memory");
}

/****************************************************************************
 * Name: n32_startup_m7
 *
 * Description:
 *   Startup function.
 *   No Flash access.
 *
 ****************************************************************************/

static noinline_function void n32_startup_m7(void)
{
  const uint32_t *src;
  uint32_t *dest;

#ifdef CONFIG_ARMV7M_STACKCHECK
  /* Set the stack limit before we attempt to call any functions */

  __asm__ volatile("sub r10, sp, %0" : :
                   "r"(CONFIG_IDLETHREAD_STACKSIZE - 64) :);
#endif

  /* If enabled reset the MPU */

  mpu_early_reset();

  /* In case of TCM configuration can be changed after POR,
   * we use AXI_RAM as .bss section before TCM configuration is initialized,
   * so we need to clear .bss_rel instead after TCM configuration has been
   * done.
   */

  for (dest = (uint32_t *)_sbss_rel; dest < (uint32_t *)_ebss_rel; )
    {
      *dest++ = 0;
    }

  /* Move the initialized data section from his temporary holding spot in
   * FLASH into the correct place in SRAM.  The correct place in SRAM is
   * give by _sdata and _edata.  The temporary location is in FLASH at the
   * end of all of the other read-only data (.text, .rodata) at _eronly.
   */

  for (src = (const uint32_t *)_eronly,
       dest = (uint32_t *)_sdata; dest < (uint32_t *)_edata;
      )
    {
      *dest++ = *src++;
    }

  /* Copy any necessary code sections from FLASH to RAM.  The correct
   * destination in SRAM is given by _sramfuncs and _eramfuncs.  The
   * temporary location is in flash after the data initialization code
   * at _framfuncs.  This should be done before n32_clockconfig() is
   * called (in case it has some dependency on initialized C variables).
   */

#ifdef CONFIG_ARCH_RAMFUNCS
  for (src = (const uint32_t *)_framfuncs,
       dest = (uint32_t *)_sramfuncs; dest < (uint32_t *)_eramfuncs;
      )
    {
      *dest++ = *src++;
    }
#endif

#ifdef CONFIG_ARMV7M_STACKCHECK
  arm_stack_check_init();
#endif
  /* Configure the UART so that we can get debug output as soon as possible */

  n32_clockconfig();
  arm_fpuconfig();
  n32_lowsetup();
  syslog(LOG_INFO, "\033[2J\033[0m");
  showprogress('A');

  /* Initialize onboard resources */

  n32_boardinitialize();
  showprogress('B');

#ifdef CONFIG_ARCH_CHIP_N32H7_CORTEXM7
  /* Enable I- and D-Caches */

  up_enable_icache();
  up_enable_dcache();
#endif
  showprogress('C');

#ifdef CONFIG_ARCH_PERF_EVENTS
  up_perf_init((void *)N32_M7CPU_FREQUENCY);
#endif

  /* Perform early serial initialization */

#ifdef USE_EARLYSERIALINIT
  arm_earlyserialinit();
#endif
  showprogress('D');

  /* For the case of the separate user-/kernel-space build, perform whatever
   * platform specific initialization of the user memory is required.
   * Normally this just means initializing the user space .data and .bss
   * segments.
   */

#ifdef CONFIG_BUILD_PROTECTED
  n32_userspace();
#endif
  showprogress('E');

  /* Then start NuttX */

  showprogress('\r');
  showprogress('\n');

#ifdef CONFIG_ARM_MPU
  /* Configure the MPU */

  n32_mpuinitialize();
#endif

  nx_start();

  /* Shouldn't get here */

  board_autoled_off(LED_STACKCREATED);
  for (; ; )
    {
      board_autoled_on(LED_PANIC);
      up_mdelay(250);
      board_autoled_off(LED_PANIC);
      up_mdelay(250);
    }
}

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

osentry_function
void __start(void)
{
#ifdef CONFIG_ARCH_CHIP_N32H7_CORTEXM7
  /* Initialize TCM */

  n32_tcm_init(TCM_SIZE_VALUE);

  /* Enable/disable tightly coupled memories */

  n32_tcmenable();

  /* CM7 Startup function */

  n32_startup_m7();
#else
#  error "N32H7_CORTEXM4 is not supported"
#endif
}
