/****************************************************************************
 * arch/arm/src/gd32f4/gd32f4xx_fmc.h
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

#ifndef __ARCH_ARM_SRC_GD32F4_GD32F4XX_FMC_H
#define __ARCH_ARM_SRC_GD32F4_GD32F4XX_FMC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/progmem.h>

#include "chip.h"
#include "hardware/gd32f4xx_fmc.h"

/* FMC state */

typedef enum
{
  FMC_READY,                       /* the operation has been completed */
  FMC_BUSY,                        /* the operation is in progress */
  FMC_RDDERR,                      /* read D-bus protection error */
  FMC_PGSERR,                      /* program sequence error */
  FMC_PGMERR,                      /* program size not match error */
  FMC_WPERR,                       /* erase/program protection error */
  FMC_OPERR,                       /* operation error */
  FMC_PGERR,                       /* program error */
  FMC_TOERR                        /* timeout error */
} gd32_fmc_state_enum;

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_fmc_wscnt_set
 *
 * Description:
 *   Set the wait state counter value
 *
 * Parameters:
 *   wscnt - Wait state counter value
 *
 ****************************************************************************/

void gd32_fmc_wscnt_set(uint32_t wscnt);

/****************************************************************************
 * Name: gd32_fmc_unlock
 *
 * Description:
 *   Unlock the main FMC operation
 *
 ****************************************************************************/

int gd32_fmc_unlock(void);

/****************************************************************************
 * Name: gd32_fmc_lock
 *
 * Description:
 *   Lock the main FMC operation
 *
 ****************************************************************************/

int gd32_fmc_lock(void);

/****************************************************************************
 * Name: gd32_fmc_sector_erase
 *
 * Description:
 *   Erase sector
 *
 * Parameters:
 *   fmc_sector - Select the sector to erase
 *
 * Return Value:
 *    State of FMC
 *
 ****************************************************************************/

gd32_fmc_state_enum gd32_fmc_sector_erase(uint32_t fmc_sector);

/****************************************************************************
 * Name: gd32_fmc_mass_erase
 *
 * Description:
 *   Erase whole chip
 *
 ****************************************************************************/

gd32_fmc_state_enum gd32_fmc_mass_erase(void);

/****************************************************************************
 * Name: gd32_fmc_bank0_erase
 *
 * Description:
 *   Erase whole bank0
 *
 ****************************************************************************/

gd32_fmc_state_enum gd32_fmc_bank0_erase(void);

/****************************************************************************
 * Name: gd32_fmc_bank1_erase
 *
 * Description:
 *   Erase whole bank1
 *
 ****************************************************************************/

gd32_fmc_state_enum gd32_fmc_bank1_erase(void);

/****************************************************************************
 * Name: gd32_fmc_word_program
 *
 * Description:
 *   Program a word at the corresponding address
 *
 * Parameters:
 *   address - Address to program
 *   data - Word to program(0x00000000 - 0xFFFFFFFF)
 *
 * Return Value:
 *    State of FMC
 *
 ****************************************************************************/

gd32_fmc_state_enum gd32_fmc_word_program(uint32_t address, uint32_t data);

/****************************************************************************
 * Name: gd32_fmc_word_program
 *
 * Description:
 *   Program a half word at the corresponding address
 *
 * Parameters:
 *   address - Address to program
 *   data - Word to program(0x0000 - 0xFFFF)
 *
 * Return Value:
 *    State of FMC
 *
 ****************************************************************************/

gd32_fmc_state_enum gd32_fmc_halfword_program(uint32_t address,
                                              uint16_t data);

/****************************************************************************
 * Name: gd32_fmc_byte_program
 *
 * Description:
 *   Program a byte at the corresponding address
 *
 * Parameters:
 *   address - Address to program
 *   data - Byte to program(0x00 - 0xFF)
 *
 * Return Value:
 *    State of FMC
 *
 ****************************************************************************/

gd32_fmc_state_enum gd32_fmc_byte_program(uint32_t address, uint8_t data);

/****************************************************************************
 * Name: gd32_ob_unlock
 *
 * Description:
 *   Unlock the option byte operation
 *
 ****************************************************************************/

void gd32_ob_unlock(void);

/****************************************************************************
 * Name: gd32_ob_lock
 *
 * Description:
 *   Lock the option byte operation
 *
 ****************************************************************************/

void gd32_ob_lock(void);

/****************************************************************************
 * Name: gd32_ob_start
 *
 * Description:
 *   Send option byte change command
 *
 ****************************************************************************/

void gd32_ob_start(void);

/****************************************************************************
 * Name: gd32_ob_write_protection_enable
 *
 * Description:
 *   Enable write protection
 *
 * Parameters:
 *   ob_wp - Specify sector to be write protected
 *
 ****************************************************************************/

int gd32_ob_write_protection_enable(uint32_t ob_wp);

/****************************************************************************
 * Name: gd32_ob_write_protection_disable
 *
 * Description:
 *   Disable write protection
 *
 * Parameters:
 *   ob_wp - Specify sector to be write protected
 *
 ****************************************************************************/

int gd32_ob_write_protection_disable(uint32_t ob_wp);

/****************************************************************************
 * Name: gd32_fmc_flag_clear
 *
 * Description:
 *   Clear the FMC pending flag
 *
 * Parameters:
 *   fmc_flag - FMC flag
 *
 ****************************************************************************/

void gd32_fmc_flag_clear(uint32_t fmc_flag);

#endif /* __ARCH_ARM_SRC_GD32F4_GD32F4XX_FMC_H */
