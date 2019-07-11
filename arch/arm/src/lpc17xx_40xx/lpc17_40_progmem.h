/******************************************************************************
 * arch/arm/src/lpc17xx_40xx/lpc17_40_progmem.c
 *
 *   Copyright (C) 2018 Michael Jung. All rights reserved.
 *   Author: Michael Jung <mijung@gmx.net>
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
 ******************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC17XX_40XX_LPC17_40_PROGMEM_H
#define __ARCH_ARM_SRC_LPC17XX_40XX_LPC17_40_PROGMEM_H

/******************************************************************************
 * See NXP UM10360 LPC176x/5x User manual, Rev 4.1, Chapter 32: LPC176x/5x
 * Flash memory interface and programming.
 *
 * The first 16 flash sectors (aka erase blocks) are 4kB in size, followed by
 * up to 14 sectors of 32 kB.  This progmem driver supports just 32 kB sectors.
 *
 * Flash write access is provided by an "In Application Programming" service
 * function stored in boot loader firmware.  Individual write accesses must be
 * 256 byte in size and must be aligned to a 256 byte boundary.
 *
 ******************************************************************************/

/******************************************************************************
* Included Files
******************************************************************************/

#include <nuttx/config.h>

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

/* The first 16 sectors are 4kB in size and thus not supported as progmem. */

#define LPC17_40_FLASH_NUM_4K_SECTORS  16

/* The number of 32kB sectors depends on the target device's flash size */

#define LPC17_40_FLASH_NUM_32K_SECTORS \
  ((LPC17_40_FLASH_SIZE - LPC17_40_FLASH_NUM_4K_SECTORS * 4096) / 32768)

/* The number of 32kB sectors to be used for progmem is configurable.  The
 * sectors at the end of the flash are used for progmem, the rest is left
 * for code and data. */

#define LPC17_40_PROGMEM_START_SECTOR \
  (LPC17_40_FLASH_NUM_4K_SECTORS + LPC17_40_FLASH_NUM_32K_SECTORS - \
   CONFIG_LPC17_40_PROGMEM_NSECTORS)

/* Base address of the flash segment used for progmem. */

#define LPC17_40_PROGMEM_START_ADDR \
  (LPC17_40_FLASH_NUM_4K_SECTORS * 4096 + \
   (LPC17_40_PROGMEM_START_SECTOR - LPC17_40_FLASH_NUM_4K_SECTORS) * 32768)

/* Size of the flash segment used for progmem. */

#define LPC17_40_PROGMEM_SIZE (CONFIG_LPC17_40_PROGMEM_NSECTORS * 32768)

/* Size of a read/write page. */

#define LPC17_40_PROGMEM_PAGE_SIZE 256

/* Total number of read/write pages. */

#define LPC17_40_PROGMEM_NUM_PAGES (LPC17_40_PROGMEM_SIZE / LPC17_40_PROGMEM_PAGE_SIZE)

/* Size of an erase page.  This driver only supports the 32kB sectors. */

#define LPC17_40_PROGMEM_SECTOR_SIZE 32768

/* Number of read/write pages per erase page. */

#define LPC17_40_PROGMEM_PAGES_PER_SECTOR \
  (LPC17_40_PROGMEM_SECTOR_SIZE / LPC17_40_PROGMEM_PAGE_SIZE)

/* LPC17 entry point for In-Application-Programming boot rom service function */

#define LPC17_40_IAP_ENTRY_ADDR 0x1fff1ff1

/* The IAP Commands required for progmem */

#define LPC17_40_IAP_CMD_PREPARE_SECTORS_FOR_WRITE_OPERATION    50
#define LPC17_40_IAP_CMD_COPY_RAM_TO_FLASH                      51
#define LPC17_40_IAP_CMD_ERASE_SECTORS                          52

/* IAP return codes */

#define LPC17_40_IAP_RC_CMD_SUCCESS                              0
#define LPC17_40_IAP_RC_INVALID_CMD                              1
#define LPC17_40_IAP_RC_SCR_ADDR_ERROR                           2
#define LPC17_40_IAP_RC_DST_ADDR_ERROR                           3
#define LPC17_40_IAP_RC_SRC_ADDR_NOT_MAPPED                      4
#define LPC17_40_IAP_RC_DST_ADDR_NOT_MAPPED                      5
#define LPC17_40_IAP_RC_COUNT_ERROR                              6
#define LPC17_40_IAP_RC_INVALID_SECTOR                           7
#define LPC17_40_IAP_RC_SECTOR_NOT_BLANK                         8
#define LPC17_40_IAP_RC_SECTOR_NOT_PREPARED_FOR_WRITE_OPERATION  9
#define LPC17_40_IAP_RC_COMPARE_ERROR                           10
#define LPC17_40_IAP_RC_BUSY                                    11
#define LPC17_40_IAP_RC_PARAM_ERROR                             12
#define LPC17_40_IAP_RC_ADDR_ERROR                              13
#define LPC17_40_IAP_RC_ADDR_NOT_MAPPED                         14
#define LPC17_40_IAP_RC_CMD_LOCKED                              15
#define LPC17_40_IAP_RC_INVALID_CODE                            16
#define LPC17_40_IAP_RC_INVALID_BAUD_RATE                       17
#define LPC17_40_IAP_RC_INVALID_STOP_BIT                        18
#define LPC17_40_IAP_RC_CODE_READ_PROTECTION_ENABLED            19

#endif /* __ARCH_ARM_SRC_LPC17XX_40XX_LPC17_40_PROGMEM_H */
