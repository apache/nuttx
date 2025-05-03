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

#include <debug.h>

#include <arch/board/board.h>
#include <arch/board/board_memorymap.h>

#include "riscv_internal.h"
#include "riscv_sbi.h"
#include "chip.h"
#include "eic7700x_mm_init.h"
#include "eic7700x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#define showprogress(c) up_putc(c)
#else
#define showprogress(c)
#endif

/* SBI Extension ID and Function ID for Hart Start */

#define SBI_EXT_HSM 0x0048534D
#define SBI_EXT_HSM_HART_START 0x0

/****************************************************************************
 * Extern Function Declarations
 ****************************************************************************/

extern void __start(void);
extern void __trap_vec(void);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Hart ID that booted NuttX (0 to 3) */

int g_eic7700x_boot_hart = -1;

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
 * Name: sbi_ecall
 *
 * Description:
 *   Make a RISC-V ECALL to OpenSBI.
 *
 * Input Parameters:
 *   extid          - Extension ID
 *   fid            - Function ID
 *   parm0 to parm5 - Parameters to be passed
 *
 * Returned Value:
 *   Error and Value returned by OpenSBI.
 *
 ****************************************************************************/

static sbiret_t sbi_ecall(unsigned int extid, unsigned int fid,
                          uintreg_t parm0, uintreg_t parm1,
                          uintreg_t parm2, uintreg_t parm3,
                          uintreg_t parm4, uintreg_t parm5)
{
  register long r0 asm("a0") = (long)(parm0);
  register long r1 asm("a1") = (long)(parm1);
  register long r2 asm("a2") = (long)(parm2);
  register long r3 asm("a3") = (long)(parm3);
  register long r4 asm("a4") = (long)(parm4);
  register long r5 asm("a5") = (long)(parm5);
  register long r6 asm("a6") = (long)(fid);
  register long r7 asm("a7") = (long)(extid);
  sbiret_t ret;

  asm volatile
    (
     "ecall"
     : "+r"(r0), "+r"(r1)
     : "r"(r2), "r"(r3), "r"(r4), "r"(r5), "r"(r6), "r"(r7)
     : "memory"
     );

  ret.error = r0;
  ret.value = (uintreg_t)r1;

  return ret;
}

/****************************************************************************
 * Name: boot_secondary
 *
 * Description:
 *   Call OpenSBI to boot the Hart, starting at the specified address.
 *
 * Input Parameters:
 *   hartid - Hart ID
 *   addr   - Start Address
 *
 * Returned Value:
 *   OK is always returned.
 *
 ****************************************************************************/

static int boot_secondary(uintreg_t hartid, uintreg_t addr)
{
  sbiret_t ret = sbi_ecall(SBI_EXT_HSM, SBI_EXT_HSM_HART_START,
                          hartid, addr, 0, 0, 0, 0);

  if (ret.error < 0)
    {
      _err("Boot Hart %d failed\n", hartid);
      PANIC();
    }

  return 0;
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

  if (mhartid != g_eic7700x_boot_hart)
    {
      goto cpux;
    }

  /* Boot Hart starts here. Init the UART Driver. */

  showprogress('A');

#ifdef USE_EARLYSERIALINIT
  riscv_earlyserialinit();
#endif

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
  /* If Boot Hart is not 0, restart with Hart 0 */

  if (mhartid != 0)
    {
      /* Clear the BSS */

      eic7700x_clear_bss();

      /* Restart with Hart 0 */

      boot_secondary(0, (uintptr_t)&__start);

      /* Let this Hart idle forever */

      while (true)
        {
          asm("WFI");
        }

      PANIC(); /* Should not come here */
    }

  /* Init the globals once only. Remember the Boot Hart. */

  if (g_eic7700x_boot_hart < 0)
    {
      g_eic7700x_boot_hart = mhartid;

      /* Clear the BSS */

      eic7700x_clear_bss();

      /* Copy the RAM Disk */

      eic7700x_copy_ramdisk();

      /* Initialize the per CPU areas */

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
