/****************************************************************************
 * arch/risc-v/src/eic7700x/eic7700x_start.c
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
#include <nuttx/init.h>
#include <nuttx/arch.h>
#include <nuttx/serial/uart_16550.h>

#include <nuttx/debug.h>

#include <arch/board/board.h>
#include <arch/board/board_memorymap.h>

#include "riscv_internal.h"
#include "riscv_sbi.h"
#include "chip.h"
#include "eic7700x_mm_init.h"
#include "eic7700x_memorymap.h"
#ifdef CONFIG_EIC7700X_CLK
#  include "eic7700x_clk.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#define showprogress(c) up_putc(c)
#else
#define showprogress(c)
#endif

/****************************************************************************
 * Extern Function Declarations
 ****************************************************************************/

extern void __start(void);
extern void __trap_vec(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Hart the firmware handed control to, which is not necessarily the one
 * NuttX ends up running on: OpenSBI picks it, and it does not pick the same
 * one every time.  Recorded by whichever Hart arrives first, before it
 * restarts on Hart 0.
 *
 * This lives in .data rather than .bss, and the non zero initialiser is what
 * puts it there.  It is read before eic7700x_clear_bss() and has to survive
 * it.
 */

static int g_eic7700x_handoff_hart = -1;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: eic7700x_copy_overlap
 *
 * Description:
 *   Copy an overlapping memory region.  dest overlaps with src + count.
 *
 * Input Parameters:
 *   dest  - Destination address
 *   src   - Source address
 *   count - Number of bytes to copy
 *
 ****************************************************************************/

static void eic7700x_copy_overlap(uint8_t *dest, const uint8_t *src,
                               size_t count)
{
  uint8_t *d = dest + count - 1;
  const uint8_t *s = src + count - 1;

  if (dest <= src)
    {
      _err("dest and src should overlap");
      PANIC();
    }

  while (count--)
    {
      volatile uint8_t c = *s;  /* Prevent compiler optimization */

      *d = c;
      d--;
      s--;
    }
}

/****************************************************************************
 * Name: eic7700x_copy_ramdisk
 *
 * Description:
 *   Copy the RAM Disk from NuttX Image to RAM Disk Region.
 *
 ****************************************************************************/

static void eic7700x_copy_ramdisk(void)
{
  const char *header = "-rom1fs-";
  const uint8_t *limit = (uint8_t *)g_idle_topstack + (256 * 1024);
  uint8_t *ramdisk_addr = NULL;
  uint8_t *addr;
  uint32_t size;

  /* After _edata, search for "-rom1fs-". This is the RAM Disk Address.
   * Limit search to 256 KB after Idle Stack Top.
   */

  binfo("_edata=%p, _sbss=%p, _ebss=%p, idlestack_top=%p\n",
        (void *)_edata, (void *)_sbss, (void *)_ebss,
        (void *)g_idle_topstack);
  for (addr = _edata; addr < limit; addr++)
    {
      if (memcmp(addr, header, strlen(header)) == 0)
        {
          ramdisk_addr = addr;
          break;
        }
    }

  /* Stop if RAM Disk is missing */

  binfo("ramdisk_addr=%p\n", ramdisk_addr);
  if (ramdisk_addr == NULL)
    {
      _err("Missing RAM Disk. Check the initrd padding.");
      PANIC();
    }

  /* RAM Disk must be after Idle Stack, to prevent overwriting */

  if (ramdisk_addr <= (uint8_t *)g_idle_topstack)
    {
      const size_t pad = (size_t)g_idle_topstack - (size_t)ramdisk_addr;

      _err("RAM Disk must be after Idle Stack. Increase initrd padding "
            "by %ul bytes.", pad);
      PANIC();
    }

  /* Read the Filesystem Size from the next 4 bytes (Big Endian) */

  size = (ramdisk_addr[8] << 24) + (ramdisk_addr[9] << 16) +
         (ramdisk_addr[10] << 8) + ramdisk_addr[11] + 0x1f0;
  binfo("size=%d\n", size);

  /* Filesystem Size must be less than RAM Disk Memory Region */

  if (size > (size_t)__ramdisk_size)
    {
      _err("RAM Disk Region too small. Increase by %ul bytes.\n",
            size - (size_t)__ramdisk_size);
      PANIC();
    }

  /* Copy the RAM Disk from NuttX Image to RAM Disk Region.
   * __ramdisk_start overlaps with ramdisk_addr + size.
   */

  eic7700x_copy_overlap(__ramdisk_start, ramdisk_addr, size);
}

/****************************************************************************
 * Name: boot_secondary
 *
 * Description:
 *   Ask OpenSBI to start a Hart at the given address.  The opaque argument
 *   the SBI call carries to the Hart is unused: eic7700x_start() takes only
 *   the Hart ID, which SBI supplies itself.
 *
 * Input Parameters:
 *   hartid - Hart ID
 *   addr   - Start Address
 *
 * Returned Value:
 *   OK on success, or a negated errno on failure.
 *
 ****************************************************************************/

static int boot_secondary(uintreg_t hartid, uintreg_t addr)
{
  int ret = riscv_sbi_boot_secondary(hartid, addr, 0);

  if (ret < 0)
    {
      _err("Boot Hart %d failed: %d\n", (int)hartid, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: eic7700x_boot_harts
 *
 * Description:
 *   Release every Hart other than this one, each entering at __start.
 *
 *   Hart 0 is always among them, unless this already is Hart 0.  NuttX runs
 *   CPU0 on Hart 0 whichever Hart the firmware chose, because that is the
 *   only Hart riscv_set_inital_sp() gives a whole idle stack to; the others
 *   keep a frame back for the state up_initial_state() writes onto the stack
 *   they are already running on.
 *
 *   The Harts released here reach riscv_cpu_boot() and wait there for the
 *   IPI that nx_smp_start() sends much later, so this returning says only
 *   that SBI accepted them, not that they have arrived.
 *
 * Input Parameters:
 *   mhartid - The Hart calling this, which is not started again
 *
 ****************************************************************************/

static void eic7700x_boot_harts(int mhartid)
{
#ifdef CONFIG_SMP
  int hart;

  for (hart = 0; hart < CONFIG_SMP_NCPUS; hart++)
    {
      if (hart != mhartid && boot_secondary(hart, (uintptr_t)&__start) < 0)
        {
          PANIC();
        }
    }
#else
  /* One CPU, so Hart 0 is the only one wanted, and only if this is not it */

  if (mhartid != 0 && boot_secondary(0, (uintptr_t)&__start) < 0)
    {
      PANIC();
    }
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: eic7700x_clear_bss
 *
 * Description:
 *   Clear .bss.  We'll do this inline (vs. calling memset) just to be
 *   certain that there are no issues with the state of global variables.
 *
 ****************************************************************************/

void eic7700x_clear_bss(void)
{
  uint32_t *dest;

  for (dest = (uint32_t *)_sbss; dest < (uint32_t *)_ebss; )
    {
      *dest++ = 0;
    }
}

/****************************************************************************
 * Name: eic7700x_start_s
 *
 * Description:
 *   Start the NuttX Kernel.  Assume that we are in RISC-V Supervisor Mode.
 *
 * Input Parameters:
 *   mhartid - Hart ID
 *
 ****************************************************************************/

void eic7700x_start_s(int mhartid)
{
  /* Configure FPU */

  riscv_fpuconfig();

  /* CPU0 is Hart 0.  See eic7700x_boot_harts() for why it has to be. */

  if (mhartid != 0)
    {
      goto cpux;
    }

  /* Boot Hart starts here. Init the UART Driver. */

  showprogress('A');

#ifdef USE_EARLYSERIALINIT
  riscv_earlyserialinit();
#endif

  /* The console only exists from here, so this is the earliest the Hart the
   * firmware chose can be reported.  It is worth reporting because nothing
   * else in NuttX depends on it today, while everything about bringing up
   * the other Harts does.
   */

  _info("Firmware handed off on Hart %d, NuttX running on Hart %d\n",
        g_eic7700x_handoff_hart, mhartid);

  /* Setup page tables for kernel and enable MMU */

  showprogress('B');
  eic7700x_mm_init();

  /* Start NuttX */

  showprogress('C');
  nx_start();

cpux:

  /* Non-Boot Hart starts here. Init the CPU for the Hart. */

#ifdef CONFIG_SMP
  riscv_cpu_boot(mhartid);
#endif

  while (true)
    {
      asm("WFI");
    }
}

/****************************************************************************
 * Name: eic7700x_start
 *
 * Description:
 *   Start the NuttX Kernel.  Called by Boot Code.
 *
 * Input Parameters:
 *   mhartid - Hart ID
 *
 ****************************************************************************/

void eic7700x_start(int mhartid)
{
  /* Whichever Hart arrives first is the one the firmware chose, and it owns
   * the one time setup.  Everything below that only Hart does has to happen
   * before any other Hart is released, since the BSS clear would otherwise
   * run underneath them.
   *
   * The guard is read before the BSS is cleared, which is why it lives in
   * .data.
   */

  if (g_eic7700x_handoff_hart < 0)
    {
      g_eic7700x_handoff_hart = mhartid;

      /* Clear the BSS */

      eic7700x_clear_bss();

      /* Copy the RAM Disk */

      eic7700x_copy_ramdisk();

      /* Release the others, Hart 0 among them */

      eic7700x_boot_harts(mhartid);
    }

#ifndef CONFIG_SMP
  /* One CPU, and it is Hart 0.  A Hart that came here only to hand over has
   * nothing further to do.  It borrowed Hart 0's idle stack to get this far,
   * so the sooner it stops using that stack the better.
   */

  if (mhartid != 0)
    {
      while (true)
        {
          asm("WFI");
        }
    }
#endif

  /* Only the Hart that goes on to run CPU0 registers itself here.  The rest
   * are registered by riscv_cpu_boot() once their IPI arrives, and a Hart
   * that took a slot twice would exhaust a free list sized by the CPU count.
   */

  if (mhartid == 0)
    {
      riscv_percpu_add_hart(mhartid);
    }

  /* Disable MMU */

  WRITE_CSR(CSR_SATP, 0x0);

  /* Set the trap vector for S-mode */

  WRITE_CSR(CSR_STVEC, (uintptr_t)__trap_vec);

  /* Start S-mode */

  eic7700x_start_s(mhartid);
}

/****************************************************************************
 * Name: riscv_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during boot up.  This must be called
 *   before riscv_serialinit.  NOTE:  This function depends on GPIO pin
 *   configuration performed in up_consoleinit() and main clock
 *   initialization performed in up_clkinitialize().
 *
 ****************************************************************************/

void riscv_earlyserialinit(void)
{
  u16550_earlyserialinit();
}

/****************************************************************************
 * Name: riscv_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that riscv_earlyserialinit was called previously.
 *
 ****************************************************************************/

void riscv_serialinit(void)
{
  u16550_serialinit();
}

/****************************************************************************
 * Name: riscv_soc_initialize
 *
 * Description:
 *   SoC specific initialization, called from up_initialize() once the heap
 *   and the serial driver are available.
 *
 *   This has to live in a file the linker is already pulling out of the
 *   arch library.  up_initialize() reaches it through a weak undefined
 *   reference, and the link uses neither --whole-archive nor a strong
 *   reference elsewhere, so a member whose only symbol is this function
 *   would never be extracted and the body would silently never run.
 *
 ****************************************************************************/

void weak_function riscv_soc_initialize(void)
{
#ifdef CONFIG_EIC7700X_CLK
  eic7700x_clk_initialize();
#endif
}
