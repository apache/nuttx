/****************************************************************************
 * arch/arm/src/xmc4/xmc4_flash.h
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

#ifndef __ARCH_ARM_SRC_XMC4_XMC4_FLASH_H
#define __ARCH_ARM_SRC_XMC4_XMC4_FLASH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/xmc4_flash.h"

#include <stdint.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Flash Command Sequences API */

/****************************************************************************
 * Name: xmc4_flash_reset_to_read
 *
 * Description: Reset command interpreter, abort page mode.
 *              Clear error flags of FSR.
 ****************************************************************************/

void xmc4_flash_reset_to_read(void);

/****************************************************************************
 * Name: xmc4_flash_enter_page_mode
 *
 * Description: PFLASH enters page mode.
 ****************************************************************************/

void xmc4_flash_enter_page_mode(void);

/****************************************************************************
 * Name: xmc4_flash_load_page
 *
 * Description: Fill the page assembly buffer. Addressed bank must be in
 *              page mode. In case of overflow, overflow data is discarded.
 *              32 Load Page operations are required to fill the assembly
 *              buffer for one 256 byte page.
 ****************************************************************************/

void xmc4_flash_load_page(uint32_t low_word, uint32_t high_word);

/****************************************************************************
 * Name: xmc4_flash_write_page
 *
 * Description: Starts the programming process for one page with the data
 *              transferred previously by Load Page commands.
 ****************************************************************************/

void xmc4_flash_write_page(uint32_t page_start_address);

/****************************************************************************
 * Name: xmc4_flash_write_user_config_page
 *
 * Description: Starts the programming process for one page with the data
 *              transferred previously by Load Page commands.
 ****************************************************************************/

void xmc4_flash_write_user_config_page(uint32_t page_start_address);

/****************************************************************************
 * Name: xmc4_flash_erase_sector
 *
 * Description: Given sector is erased.
 *
 ****************************************************************************/

void xmc4_flash_erase_sector(uint32_t sector_start_address);

/****************************************************************************
 * Name: xmc4_flash_erase_physical_sector
 *
 * Description: Given physical sector is erased. Depending on config of
 *              PROCON1.PSR it repairs the sector.
 *
 ****************************************************************************/

void xmc4_flash_erase_physical_sector(uint32_t sector_start_address);

/****************************************************************************
 * Name: xmc4_flash_repair_physical_sector
 *
 * Description: Given physical sector is repaired. Depending on config of
 *              PROCON1.PSR it erases the sector.
 *
 ****************************************************************************/

void xmc4_flash_repair_physical_sector(uint32_t sector_start_address);

/****************************************************************************
 * Name: xmc4_flash_erase_user_config_block
 *
 * Description: Given user configurable block is erased.
 *
 ****************************************************************************/

void xmc4_flash_erase_user_config_block(uint32_t block_start_address);

/****************************************************************************
 * Name: xmc4_flash_disable_sector_write_protection
 *
 * Description: The sector write protection belonging to user level UL is
 *             temporarily disabled by setting FSR.WPRODIS when the passwords
 *             PW0 and PW1 match their configured values in the corresponding
 *             UCB
 ****************************************************************************/

void xmc4_flash_disable_sector_write_protection(uint32_t user_level,
                                                uint32_t password_0,
                                                uint32_t password_1);

/****************************************************************************
 * Name: xmc4_flash_disable_sector_read_protection
 *
 * Description: The Flash module read protection including the derived module
 *              wide write protection are temporarily disabled by setting
 *              FSR.RPRODIS when the passwords PW0 and PW1 match their
 *              configured values in the UCB0.
 ****************************************************************************/

void xmc4_flash_disable_sector_read_protection(uint32_t password_0,
                                               uint32_t password_1);

/****************************************************************************
 * Name: xmc4_flash_resume_protection
 *
 * Description: This command clears all FSR.WPRODISx and the FSR.RPRODIS
 *              effectively enabling again the Flash protection as it was
 *              configured.
 ****************************************************************************/

void xmc4_flash_resume_protection(void);

/****************************************************************************
 * Name: xmc4_flash_clear_status
 *
 * Description: The flags FSR.PROG and FSR.ERASE and the error flags of FSR
 *              (PFOPER, SQER, PROER, PFDBER, ORIER, VER) are cleared.
 ****************************************************************************/

void xmc4_flash_clear_status(void);

#endif /* __ARCH_ARM_SRC_XMC4_XMC4_FLASH_H */
