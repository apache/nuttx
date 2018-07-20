/****************************************************************************
 * arch/arm/src/lc823450/lc823450_start.c
 *
 *   Copyright 2014, 2015, 2016, 2017, 2018 Sony Video & Sound Products Inc.
 *   Author: Masatoshi Tateishi <Masatoshi.Tateishi@jp.sony.com>
 *   Author: Masayuki Ishikawa <Masayuki.Ishikawa@jp.sony.com>
 *   Author: Yasuhiro Osaki <Yasuhiro.Osaki@jp.sony.com>
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

#include "up_arch.h"
#include "up_internal.h"
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

#define IDLE_STACKSIZE (CONFIG_IDLETHREAD_STACKSIZE & ~7)
#define IDLE_STACK     ((uintptr_t)&_ebss + IDLE_STACKSIZE)
#define HEAP_BASE      ((uintptr_t)&_ebss + IDLE_STACKSIZE)

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

extern uint32_t _stext_sram, _etext_sram, _ftext, _svect;

/****************************************************************************
 * Private Function prototypes
 ****************************************************************************/

#ifdef CONFIG_DEBUG_STACK
static void go_os_start(void *pv, unsigned int nbytes)
  __attribute__ ((naked, no_instrument_function, noreturn));
#endif

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
#  define showprogress(c) up_lowputc(c)
#else
#  define showprogress(c)
#endif

/****************************************************************************
 * Name: go_os_start
 *
 * Description:
 *   Set the IDLE stack to the
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_STACK
static void go_os_start(void *pv, unsigned int nbytes)
{
  /* Set the IDLE stack to the stack coloration value then jump to
   * os_start().  We take extreme care here because were currently
   * executing on this stack.
   *
   * We want to avoid sneak stack access generated by the compiler.
   */

  __asm__ __volatile__
  (
    "\tmov  r1, r1, lsr #2\n"   /* R1 = nwords = nbytes >> 2 */
    "\tbeq  2f\n"               /* (should not happen) */

    "\tbic  r0, r0, #3\n"       /* R0 = Aligned stackptr */
    "\tmovw r2, #0xbeef\n"      /* R2 = STACK_COLOR = 0xdeadbeef */
    "\tmovt r2, #0xdead\n"

    "1:\n"                      /* Top of the loop */
    "\tsub  r1, r1, #1\n"       /* R1 nwords-- */
    "\tcmp  r1, #0\n"           /* Check (nwords == 0) */
    "\tstr  r2, [r0], #4\n"     /* Save stack color word, increment stackptr */
    "\tbne  1b\n"               /* Bottom of the loop */

    "2:\n"
    "\tmov  r14, #0\n"          /* LR = return address (none) */
    "\tb    os_start\n"         /* Branch to os_start */
  );
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _start
 *
 * Description:
 *   This is the reset entry point.
 *
 ****************************************************************************/

#ifdef CONFIG_SPIFLASH_BOOT
__attribute__((section (".start_text"))) void __start_main(void)
#else /* CONFIG_SPIFLASH_BOOT */
void __start(void)
#endif /* CONFIG_SPIFLASH_BOOT */
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

  for (dest = &_sbss; dest < &_ebss; )
    {
      *dest++ = 0;
    }

  /* Move the initialized data section from his temporary holding spot in
   * FLASH into the correct place in SRAM.  The correct place in SRAM is
   * give by _sdata and _edata.  The temporary location is in FLASH at the
   * end of all of the other read-only data (.text, .rodata) at _eronly.
   */

  for (src = &_eronly, dest = &_sdata; dest < &_edata; )
    {
      *dest++ = *src++;
    }

  /* run as interrupt context, before scheduler running */

  CURRENT_REGS = (uint32_t *)1;

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

#ifdef CONFIG_SPIFLASH_BOOT

  /* Copy any necessary code sections from FLASH to RAM.  The correct
   * destination in SRAM is given by _sramfuncs and _eramfuncs.  The
   * temporary location is in flash after the data initalization code
   * at _framfuncs.
   */

  /* copt text & vectors */

  for (src = &_ftext, dest = &_stext_sram; dest < &_etext_sram; )
    {
      *dest++ = *src++;
    }

  /* vector offset */

  putreg32((uint32_t)&_svect, NVIC_VECTAB);

#else /* CONFIG_SPIFLASH_BOOT */
  /* vector offset */

#ifdef CONFIG_LC823450_IPL2
  putreg32(0x02000e00, 0xe000ed08);
  putreg32(0x0, 0x40080008); /* XXX: remap disable */
#else /* CONFIG_LC823450_IPL2 */
  putreg32(0x02040000, 0xe000ed08);
#endif /* CONFIG_LC823450_IPL2 */
#endif /* CONFIG_LC823450_SPIFLASH_BOOT */

  /* Enable Mutex */
  /* NOTE: modyfyreg32() can not be used because it might use spin_lock */

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
  modifyreg32(rP2DT,  0, 1 << 15  /* GPIO2F */);
  modifyreg32(rP2DRC, 0, 1 << 15  /* GPIO2F */);
#ifdef CONFIG_DEBUG_FEATURES

  /* enable TXD0 for debug */

  modifyreg32(PMDCNT5, 0, 3 << 14);
#endif /* CONFIG_DEBUG_FEATURES */
#else  /* CONFIG_LC823450_IPL2 */
  up_init_default_mux();
#endif /* CONFIG_LC823450_IPL2 */

  showprogress('B');

#if defined(CONFIG_LC823450_SPIFI) && !defined(CONFIG_SPIFLASH_BOOT)
  lc823450_spiflash_earlyinit();
#endif /* CONFIG_LC823450_SPIFI */

#ifdef CONFIG_LC823450_SDRAM
  lc823450_sdram_init();
#endif /* CONFIG_LC823450_SDRAM */

  showprogress('C');

  /* Perform early serial initialization */

#ifdef USE_EARLYSERIALINIT
  up_earlyserialinit();
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

  (void)get_cpu_ver();

  /* run as interrupt context, before scheduler running */

  CURRENT_REGS = NULL;

#ifdef CONFIG_DEBUG_STACK
  /* Set the IDLE stack to the coloration value and jump into os_start() */

  go_os_start((FAR void *)&_ebss, CONFIG_IDLETHREAD_STACKSIZE);
#else
  /* Call os_start() */

  os_start();

  /* Shoulnd't get here */

  for (;;);
#endif
}

#if defined(CONFIG_SPIFLASH_BOOT)
__attribute__((section (".start_gdb"))) void __start(void)
{
  /* XXX: Don't use stack in this function */

  /* SPIF/CACHE clock */

  putreg32(0x0402, 0x40080100);

  /* SPIF/CACHE reset */

  putreg32(0x0402, 0x40080114);

  /* PinMux for QSPI */

  putreg32(0x540000c0, 0x40080400);
  putreg32(0x00000017, 0x40080404);

  /* BusAcc enable */

  putreg32(0x00000303, 0x40001028);

  /* Stack initialize: */

  __asm__ __volatile__
  (
    "ldr r0, =_vectors\n"
    "bic r0, r0, #1\n"
    "ldr sp, [r0, #0]\n"
  );

  __start_main();

  /* not reached */
}
#endif /* defined(CONFIG_SPIFLASH_BOOT) */
