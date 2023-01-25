/****************************************************************************
 * boards/arm/stm32f7/stm32f777zit6-meadow/scripts/memory.ld
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

/* The STM32F777ZIT6 has 2048Kb of main FLASH memory.  This FLASH memory can
 * be accessed from either the AXIM interface at address 0x0800:0000 or from
 * the ITCM interface at address 0x0020:0000.
 *
 * Additional information, including the option bytes, is available at at
 * FLASH at address 0x1ff0:0000.
 *
 * In the STM32F777ZIT6, two different boot spaces can be selected through
 * the BOOT pin and the boot base address programmed in the BOOT_ADD0 and
 * BOOT_ADD1 option bytes:
 *
 *   1) BOOT=0: Boot address defined by user option byte BOOT_ADD0[15:0].
 *      ST programmed value: Flash on ITCM at 0x0020:0000
 *   2) BOOT=1: Boot address defined by user option byte BOOT_ADD1[15:0].
 *      ST programmed value: System bootloader at 0x0010:0000
 *
 * NuttX does not modify these option bytes.  On the unmodified STM32F777ZIT6
 * MEADOW board, the BOOT0 pin is at ground so by default, the STM32 will boot
 * to address 0x0020:0000 in ITCM FLASH.
 *
 * The STM32F777ZIT6 also has 512Kb of data SRAM (in addition to ITCM SRAM).
 * SRAM is split up into three blocks:
 *
 *   1) 128Kb of DTCM SRM beginning at address 0x2000:0000
 *   2) 368Kb of SRAM1 beginning at address 0x2002:0000
 *   3)  16Kb of SRAM2 beginning at address 0x2007:c000
 *
 * When booting from FLASH, FLASH memory is aliased to address 0x0000:0000
 * where the code expects to begin execution by jumping to the entry point in
 * the 0x0800:0000 address range.
 *
 * For MPU support, the kernel-mode NuttX section is assumed to be 128Kb of
 * FLASH and 4Kb of SRAM.  That is an excessive amount for the kernel which
 * should fit into 64KB and, of course, can be optimized as needed (See
 * also boards/arm/stm32f7/stm32f777zit6-meadow/scripts/kernel-space.ld).  Allowing the
 * additional does permit addition debug instrumentation to be added to the
 * kernel space without overflowing the partition.
 *
 * Alignment of the user space FLASH partition is also a critical factor:
 * The user space FLASH partition will be spanned with a single region of
 * size 2**n bytes.  The alignment of the user-space region must be the same.
 * As a consequence, as the user-space increases in size, the alignment
 * requirement also increases.
 *
 * This alignment requirement means that the largest user space FLASH region
 * you can have will be 512KB at it would have to be positioned at
 * 0x08800000.  If you change this address, don't forget to change the
 * CONFIG_NUTTX_USERSPACE configuration setting to match and to modify
 * the check in kernel/userspace.c.
 *
 * For the same reasons, the maximum size of the SRAM mapping is limited to
 * 4KB.  Both of these alignment limitations could be reduced by using
 * multiple regions to map the FLASH/SDRAM range or perhaps with some
 * clever use of subregions.
 *
 * A detailed memory map for the 112KB SRAM region is as follows:
 *
 *   0x2002 0000:  Kernel .data region.  Typical size: 0.1KB
 *   ------ ----   Kernel .bss  region.  Typical size: 1.8KB
 *   0x2002 0800:  Kernel IDLE thread stack (approximate).  Size is
 *                 determined by CONFIG_IDLETHREAD_STACKSIZE and
 *                 adjustments for alignment.  Typical is 1KB.
 *   ------ ----   Padded to 4KB
 *   0x2002 1000:  User .data region.  Size is variable.
 *   ------ ----   User .bss region  Size is variable.
 *   0x2002 2000:  Beginning of kernel heap.  Size determined by
 *                 CONFIG_MM_KERNEL_HEAPSIZE.
 *   ------ ----   Beginning of user heap.  Can vary with other settings.
 *   0x2004 c000:  End+1 of SRAM1
 */

MEMORY
{
	/* ITCM boot address */

	itcm  (rwx) : ORIGIN = 0x00200000, LENGTH = 2048K

	/* 2048KB FLASH */

	kflash (rx) : ORIGIN = 0x08000000, LENGTH = 256K
	uflash (rx) : ORIGIN = 0x08040000, LENGTH = 1792K
	xflash (rx) : ORIGIN = 0x08200000, LENGTH = 0K

	/* 368KB of contiguous SRAM1 */

	ksram (rwx) : ORIGIN = 0x20020000, LENGTH = 64K
	usram (rwx) : ORIGIN = 0x20030000, LENGTH = 192K
	xsram (rwx) : ORIGIN = 0x20022000, LENGTH = 0K

	/* DTCM SRAM */

	dtcm  (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
	sram2 (rwx) : ORIGIN = 0x2007c000, LENGTH = 16K
}
