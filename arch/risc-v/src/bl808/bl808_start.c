/****************************************************************************
 * arch/risc-v/src/bl808/bl808_start.c
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

#include <debug.h>

#include <arch/board/board.h>
#include <arch/board/board_memorymap.h>

#include "riscv_internal.h"
#include "chip.h"
#include "bl808_mm_init.h"
#include "bl808_memorymap.h"
#include "bl808_serial.h"

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

extern void __trap_vec(void);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* NOTE: g_idle_topstack needs to point the top of the idle stack
 * for last CPU and this value is used in up_initial_state()
 */

uintptr_t g_idle_topstack = BL808_IDLESTACK_BASE +
                              SMP_STACK_SIZE * CONFIG_SMP_NCPUS;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bl808_copy_overlap
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

static void bl808_copy_overlap(uint8_t *dest, const uint8_t *src,
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
 * Name: bl808_copy_ramdisk
 *
 * Description:
 *   Copy the RAM Disk from NuttX Image to RAM Disk Region.
 *
 ****************************************************************************/

static void bl808_copy_ramdisk(void)
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

  bl808_copy_overlap(__ramdisk_start, ramdisk_addr, size);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bl808_clear_bss
 *
 * Description:
 *   Clear .bss.  We'll do this inline (vs. calling memset) just to be
 *   certain that there are no issues with the state of global variables.
 *
 ****************************************************************************/

void bl808_clear_bss(void)
{
  uint32_t *dest;

  for (dest = (uint32_t *)_sbss; dest < (uint32_t *)_ebss; )
    {
      *dest++ = 0;
    }
}

/****************************************************************************
 * Name: bl808_start_s
 *
 * Description:
 *   Start the NuttX Kernel.  Assume that we are in RISC-V Supervisor Mode.
 *
 * Input Parameters:
 *   mhartid - Hart ID
 *
 ****************************************************************************/

void bl808_start_s(int mhartid)
{
  /* Configure FPU */

  riscv_fpuconfig();

  if (mhartid > 0)
    {
      goto cpux;
    }

  showprogress('A');

#ifdef USE_EARLYSERIALINIT
  riscv_earlyserialinit();
#endif

  showprogress('B');

  /* Do board initialization */

  showprogress('C');

  /* Setup page tables for kernel and enable MMU */

  bl808_mm_init();

  /* Call nx_start() */

  nx_start();

cpux:

#ifdef CONFIG_SMP
  riscv_cpu_boot(mhartid);
#endif

  while (true)
    {
      asm("WFI");
    }
}

/****************************************************************************
 * Name: bl808_start
 *
 * Description:
 *   Start the NuttX Kernel.  Called by Boot Code.
 *
 * Input Parameters:
 *   mhartid - Hart ID
 *
 ****************************************************************************/

void bl808_start(int mhartid)
{
  DEBUGASSERT(mhartid == 0); /* Only Hart 0 supported for now */

  if (0 == mhartid)
    {
      /* Clear the BSS */

      bl808_clear_bss();

      /* Setup base stack */

      riscv_set_basestack(BL808_IDLESTACK_BASE, SMP_STACK_SIZE);

      /* Copy the RAM Disk */

      bl808_copy_ramdisk();

      /* Initialize the per CPU areas */

      riscv_percpu_add_hart(mhartid);
    }

  /* Disable MMU */

  WRITE_CSR(CSR_SATP, 0x0);

  /* Set the trap vector for S-mode */

  WRITE_CSR(CSR_STVEC, (uintptr_t)__trap_vec);

  /* Start S-mode */

  bl808_start_s(mhartid);
}

/****************************************************************************
 * Name: riscv_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before riscv_serialinit.  NOTE:  This function depends on GPIO pin
 *   configuration performed in up_consoleinit() and main clock
 *   initialization performed in up_clkinitialize().
 *
 ****************************************************************************/

void riscv_earlyserialinit(void)
{
  bl808_earlyserialinit();
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
  bl808_serialinit();
}
