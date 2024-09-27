/****************************************************************************
 * boards/arm/stm32/olimex-stm32-p407/scripts/memory.ld
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

/* The STM32F407VG has 1024Kb of FLASH beginning at address 0x0800:0000 and
 * 192Kb of SRAM. SRAM is split up into three blocks:
 *
 * 1) 112KB of SRAM beginning at address 0x2000:0000
 * 2)  16KB of SRAM beginning at address 0x2001:c000
 * 3)  64KB of CCM SRAM beginning at address 0x1000:0000
 *
 * When booting from FLASH, FLASH memory is aliased to address 0x0000:0000
 * where the code expects to begin execution by jumping to the entry point in
 * the 0x0800:0000 address range.
 *
 * For MPU support, the kernel-mode NuttX section is assumed to be 128Kb of
 * FLASH and 8Kb of SRAM.  (See boards/stm32f4discovery/scripts/kernel-space.ld).
 * Allowing additional memory permitis configuring debug instrumentation to
 * be added to the kernel space without overflowing the partition.  This could
 * just as easily be set to 256Kb or even 512Kb.
 *
 * Alignment of the user space FLASH partition is also a critical factor:
 * The user space FLASH partition will be spanned with a single region of
 * size 2**n bytes.  The alignment of the user-space region must be the same.
 * As a consequence, as the user-space increases in size, the alignment
 * requirement also increases.
 *
 * This alignment requirement means that the largest user space FLASH region
 * you can have will be 512KB at it would have to be positioned at
 * 0x08800000 (it cannot be positioned at 0x0800000 because vectors power-up
 * reset vectors are places at the beginning of that range).  If you change
 * this address, don't forget to change the CONFIG_NUTTX_USERSPACE
 * configuration setting to match and to modify the check in kernel/userspace.c.
 *
 * With 112Kb of SRAM a 64Kb user heap would seem possible but it is not in
 * the current organization of SRAM memory (that could be changed with a
 * little effort).  The current ordering of SRAM is:  (1) kernel .bss/.data,
 * (2) user .bss/.data, (3) kernel heap (determined by CONFIG_MM_KERNEL_HEAPSIZE),
 * and (4) the user heap.  The maximum size of the user space heap is then
 * limited to 32Kb beginning at address 0x20008000.
 *
 * Both of these alignment limitations could be reduced by using multiple
 * regions to map the FLASH/SDRAM range or perhaps with some clever use of
 * subregions or with multiple MPU regions per memory region.
 *
 * NOTE: The MPU is used in a mode where mappings are not required for
 * kernel addresses and, hence, there are not alignment issues for those
 * case.  Only the user address spaces suffer from alignment requirements.
 * However, in order to exploit this fact, we would still need to change
 * the ordering of memory regions in SRAM.
 *
 * A detailed memory map for the 112KB SRAM region is as follows:
 *
 *   0x2000 0000:  Kernel .data region.  Typical size: 0.1KB
 *   ------ ----   Kernel .bss  region.  Typical size: 1.8KB
 *   0x2000 0800:  Kernel IDLE thread stack (approximate).  Size is
 *                 determined by CONFIG_IDLETHREAD_STACKSIZE and
 *                 adjustments for alignment.  Typical is 1KB.
 *   ------ ----   Padded to 8KB
 *   0x2000 2000:  User .data region.  Size is variable.
 *   ------ ----   User .bss region  Size is variable.
 *   0x2000 4000:  Beginning of kernel heap.  Size determined by
 *                 CONFIG_MM_KERNEL_HEAPSIZE which must be set to 16Kb.
 *   0x2000 8000:  Beginning of 32Kb user heap.
 *   0x2001 0000:  The remainder of SRAM is, unfortunately, wasted.
 *   0x2001 c000:  End+1 of CPU RAM
 */

MEMORY
{
  /* 1024Kb FLASH */

  kflash (rx)      : ORIGIN = 0x08000000, LENGTH = 128K
  uflash (rx)      : ORIGIN = 0x08020000, LENGTH = 128K
  xflash (rx)      : ORIGIN = 0x08040000, LENGTH = 768K

  /* 112Kb of contiguous SRAM */

  ksram (rwx)      : ORIGIN = 0x20000000, LENGTH = 8K
  usram (rwx)      : ORIGIN = 0x20002000, LENGTH = 8K
  xsram (rwx)      : ORIGIN = 0x20008000, LENGTH = 96K
}
