/****************************************************************************
 * arch/arm/src/lc823450/lc823450_start.c
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
#include <string.h>
#include <stdio.h>

#include <nuttx/init.h>
#include <nuttx/arch.h>
#include <nuttx/mtd/mtd.h>

#ifdef CONFIG_LASTKMSG
#  include <nuttx/lastkmsg.h>
#endif /* CONFIG_LASTKMSG */

#include "arm_internal.h"
#include "nvic.h"
#include <arch/board/board.h>

#ifdef CONFIG_LC823450_SPIFI
#  include "lc823450_spifi2.h"
#endif
#include "lc823450_lowputc.h"
#include "lc823450_clockconfig.h"
#include "lc823450_syscontrol.h"

#ifdef CONFIG_BUILD_PROTECTED
#  include "lc823450_userspace.h"
#endif

#include "lc823450_gpio.h"

#ifdef CONFIG_MM_MULTIHEAP
#  include "lc823450_sram.h"
#endif

#ifdef CONFIG_LC823450_SDRAM
#  include "lc823450_sdram.h"
#endif

#include "lc823450_start.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* .data is positioned first in the primary RAM followed immediately by .bss.
 * The IDLE thread stack lies just after .bss and has size give by
 * CONFIG_IDLETHREAD_STACKSIZE;  The heap then begins just after the IDLE.
 * ARM EABI requires 64 bit stack alignment.
 */

#define HEAP_BASE      ((uintptr_t)_ebss + CONFIG_IDLETHREAD_STACKSIZE)

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
 * Public Data
 ****************************************************************************/

int icx_boot_reason;

extern uint32_t _stext_sram;
extern uint32_t _etext_sram;
extern uint32_t _ftext;
extern uint32_t _svect;

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
  const uint32_t *src;
  uint32_t *dest;

  /* Clear .bss.  We'll do this inline (vs. calling memset) just to be
   * certain that there are no issues with the state of global variables.
   */

#ifdef CONFIG_FS_EVFAT
  /* clear the work area in seg0 */

  dest = (uint32_t *)0x02000000;
  int i;
  for (i = 0; i < 0xe00 / sizeof(uint32_t); i++)
    {
      *dest++ = 0;
    }
#endif

  for (dest = (uint32_t *)_sbss; dest < (uint32_t *)_ebss; )
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

#ifdef CONFIG_LASTKMSG_LOWOUTS

  if (g_lastksg_buf.sig == LASTKMSG_SIG_REBOOT)
    {
      icx_boot_reason |= ICX_BOOT_REASON_REBOOT;
    }

  /* clrear kmsg buffer */

  memset(&g_lastksg_buf, 0, sizeof(g_lastksg_buf));

  /* set lastkmsg signature */

  g_lastksg_buf.sig = LASTKMSG_SIG;
#endif /* CONFIG_LASTKMSG */

#ifdef CONFIG_LC823450_SPIFI_BOOT

  /* Copy any necessary code sections from FLASH to RAM.  The correct
   * destination in SRAM is given by _sramfuncs and _eramfuncs.  The
   * temporary location is in flash after the data initialization code
   * at _framfuncs.  This should be done before lc823450_clockconfig() is
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

#else /* CONFIG_LC823450_SPIFI_BOOT */
  /* vector offset */

#ifdef CONFIG_LC823450_IPL2
  putreg32(0x02000e00, 0xe000ed08);
  putreg32(0x0, 0x40080008); /* XXX: remap disable */
#else /* CONFIG_LC823450_IPL2 */
  putreg32(0x02040000, 0xe000ed08);
#endif /* CONFIG_LC823450_IPL2 */

#endif /* CONFIG_LC823450_SPIFI_BOOT */

  /* Enable Mutex
   * NOTE: modyfyreg32() can not be used because it might use spin_lock.
   */

  uint32_t val = getreg32(MRSTCNTBASIC);
  val |= MRSTCNTBASIC_MUTEX_RSTB;
  putreg32(val, MRSTCNTBASIC);

  /* Configure the uart so that we can get debug output as soon as possible */

  lc823450_clockconfig();

  lc823450_lowsetup();

  showprogress('A');

  /* IPL2 don't change mux */

#ifdef CONFIG_LC823450_IPL2
  /* GPIO2F out High in IPL2 */

  modifyreg32(MCLKCNTAPB, 0, MCLKCNTAPB_PORT2_CLKEN);
  modifyreg32(MRSTCNTAPB, 0, MRSTCNTAPB_PORT2_RSTB);
  modifyreg32(P2DT,  0, 1 << 15  /* GPIO2F */);
  modifyreg32(P2DRC, 0, 1 << 15  /* GPIO2F */);
#ifdef CONFIG_DEBUG_FEATURES

  /* enable TXD0 for debug */

  modifyreg32(PMDCNT5, 0, 3 << 14);
#endif /* CONFIG_DEBUG_FEATURES */
#else  /* CONFIG_LC823450_IPL2 */
  up_init_default_mux();
#endif /* CONFIG_LC823450_IPL2 */

  showprogress('B');

#if defined(CONFIG_LC823450_SPIFI) && !defined(CONFIG_LC823450_SPIFI_BOOT)
  lc823450_spiflash_earlyinit();
#endif /* CONFIG_LC823450_SPIFI */

#ifdef CONFIG_LC823450_SDRAM
  lc823450_sdram_init();
#endif /* CONFIG_LC823450_SDRAM */

  showprogress('C');

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
  lc823450_userspace();
#endif
#if defined(CONFIG_BUILD_FLAT) && defined(CONFIG_ARM_MPU)
  lc823450_mpuinitialize();
#endif
  showprogress('E');

#ifdef CONFIG_MM_MULTIHEAP
  lc823450_sram_initialize();
#endif

  showprogress('F');

#ifndef CONFIG_LC823450_IPL2
  sinfo("icx_boot_reason = 0x%x\n", icx_boot_reason);
#endif /* CONFIG_LC823450_IPL2 */

#ifdef CONFIG_POWERBUTTON_LDOWN
  if (icx_boot_reason & ICX_BOOT_REASON_POWERBUTTON)
    {
      int t = 1000;

      while (--t && up_board_powerkey())
        {
          up_udelay(10 * 1000);
        }

      sinfo("t = %d\n", t);

      if (t)
        {
          up_board_poweren(0);
          up_udelay(1000 * 1000);
          sinfo("VBUS connected ?\n");

          /* VBUS is connected after powerup by key.
           * Resume PowerOn sequence. (cancel shutdown)
           */

          up_board_poweren(1);
        }
    }
#endif /* CONFIG_POWERBUTTON_LDOWN */

  /* Then start NuttX */

  showprogress('\r');
  showprogress('\n');

  get_cpu_ver();

  nx_start();

  /* Shouldn't get here */

  for (; ; );
}
