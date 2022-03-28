/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/lpc17_40_progmem.h
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

#ifndef __ARCH_ARM_SRC_LPC17XX_40XX_LPC17_40_PROGMEM_H
#define __ARCH_ARM_SRC_LPC17XX_40XX_LPC17_40_PROGMEM_H

/****************************************************************************
 * See NXP UM10360 LPC176x/5x User manual, Rev 4.1, Chapter 32: LPC176x/5x
 * Flash memory interface and programming.
 *
 * The first 16 flash sectors (aka erase blocks) are 4kB in size, followed
 * by up to 14 sectors of 32 kB.
 * This progmem driver supports just 32 kB sectors.
 *
 * Flash write access is provided by an "In Application Programming" service
 * function stored in boot loader firmware.  Individual write accesses must
 * be 256 byte in size and must be aligned to a 256 byte boundary.
 *
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The first 16 sectors are 4kB in size. */

#define LPC17_40_FLASH_NUM_4K_SECTORS  16

/* The number of 32kB sectors depends on the target device's flash size. */

#define LPC17_40_FLASH_NUM_32K_SECTORS \
  ((LPC17_40_FLASH_SIZE - LPC17_40_FLASH_NUM_4K_SECTORS * 4096) / 32768)

/* The total number of sectors is the sum of the 4k and 32k sectors. */

#define LPC17_40_FLASH_NUM_SECTORS \
  (LPC17_40_FLASH_NUM_4K_SECTORS + LPC17_40_FLASH_NUM_32K_SECTORS)

/* Flash erased byte value */

#define LPC17_40_FLASH_ERASEDVAL (0xffu)

/* Size of a write page. */

#define LPC17_40_WRITE_SIZE 256

/* LPC17 entry point for In-Application-Programming boot rom service
 * function
 */

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
