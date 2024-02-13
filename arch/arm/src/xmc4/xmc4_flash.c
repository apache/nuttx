/****************************************************************************
 * arch/arm/src/xmc4/xmc4_flash.c
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

#include "xmc4_flash.h"

#include "arm_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void xmc4_flash_reset_to_read(void)
{
  putreg32(0xf0, XMC4_UNCACHED_PFLASH_BASE + 0x5554);
}

void xmc4_flash_enter_page_mode(void)
{
  putreg32(0x50, XMC4_UNCACHED_PFLASH_BASE + 0x5554);
}

void xmc4_flash_load_page(uint32_t low_word, uint32_t high_word)
{
  putreg32(low_word, XMC4_UNCACHED_PFLASH_BASE + 0x55f0);
  putreg32(high_word, XMC4_UNCACHED_PFLASH_BASE + 0x55f4);
}

void xmc4_flash_write_page(uint32_t page_start_address)
{
  putreg32(0xaa, XMC4_UNCACHED_PFLASH_BASE + 0x5554);
  putreg32(0x55, XMC4_UNCACHED_PFLASH_BASE + 0xaaa8);
  putreg32(0xa0, XMC4_UNCACHED_PFLASH_BASE + 0x5554);
  putreg32(0xaa, page_start_address);
}

void xmc4_flash_write_user_config_page(uint32_t page_start_address)
{
  putreg32(0xaa, XMC4_UNCACHED_PFLASH_BASE + 0x5554);
  putreg32(0x55, XMC4_UNCACHED_PFLASH_BASE + 0xaaa8);
  putreg32(0xc0, XMC4_UNCACHED_PFLASH_BASE + 0x5554);
  putreg32(0xaa, page_start_address);
}

void xmc4_flash_erase_sector(uint32_t sector_start_address)
{
  putreg32(0xaa, XMC4_UNCACHED_PFLASH_BASE + 0x5554);
  putreg32(0x55, XMC4_UNCACHED_PFLASH_BASE + 0xaaa8);
  putreg32(0x80, XMC4_UNCACHED_PFLASH_BASE + 0x5554);
  putreg32(0xaa, XMC4_UNCACHED_PFLASH_BASE + 0x5554);
  putreg32(0x55, XMC4_UNCACHED_PFLASH_BASE + 0xaaa8);
  putreg32(0x30, sector_start_address);
}

void xmc4_flash_erase_physical_sector(uint32_t sector_start_address)
{
  putreg32(0xaa, XMC4_UNCACHED_PFLASH_BASE + 0x5554);
  putreg32(0x55, XMC4_UNCACHED_PFLASH_BASE + 0xaaa8);
  putreg32(0x80, XMC4_UNCACHED_PFLASH_BASE + 0x5554);
  putreg32(0xaa, XMC4_UNCACHED_PFLASH_BASE + 0x5554);
  putreg32(0x55, XMC4_UNCACHED_PFLASH_BASE + 0xaaa8);
  putreg32(0x40, sector_start_address);
}

void xmc4_flash_repair_physical_sector(uint32_t sector_start_address)
{
  putreg32(0xaa, XMC4_UNCACHED_PFLASH_BASE + 0x5554);
  putreg32(0x55, XMC4_UNCACHED_PFLASH_BASE + 0xaaa8);
  putreg32(0x80, XMC4_UNCACHED_PFLASH_BASE + 0x5554);
  putreg32(0xaa, XMC4_UNCACHED_PFLASH_BASE + 0x5554);
  putreg32(0x55, XMC4_UNCACHED_PFLASH_BASE + 0xaaa8);
  putreg32(0x40, sector_start_address);
}

void xmc4_flash_erase_user_config_block(uint32_t block_start_address)
{
  putreg32(0xaa, XMC4_UNCACHED_PFLASH_BASE + 0x5554);
  putreg32(0x55, XMC4_UNCACHED_PFLASH_BASE + 0xaaa8);
  putreg32(0x80, XMC4_UNCACHED_PFLASH_BASE + 0x5554);
  putreg32(0xaa, XMC4_UNCACHED_PFLASH_BASE + 0x5554);
  putreg32(0x55, XMC4_UNCACHED_PFLASH_BASE + 0xaaa8);
  putreg32(0xc0, block_start_address);
}

void xmc4_flash_disable_sector_write_protection(uint32_t user_level,
                                                uint32_t password_0,
                                                uint32_t password_1)
{
  putreg32(0xaa, XMC4_UNCACHED_PFLASH_BASE + 0x5554);
  putreg32(0x55, XMC4_UNCACHED_PFLASH_BASE + 0xaaa8);
  putreg32(user_level, XMC4_UNCACHED_PFLASH_BASE + 0x553c);
  putreg32(password_0, XMC4_UNCACHED_PFLASH_BASE + 0xaaa8);
  putreg32(password_1, XMC4_UNCACHED_PFLASH_BASE + 0xaaa8);
  putreg32(0x05, XMC4_UNCACHED_PFLASH_BASE + 0x5558);
}

void xmc4_flash_disable_sector_read_protection(uint32_t password_0,
                                               uint32_t password_1)
{
  putreg32(0xaa, XMC4_UNCACHED_PFLASH_BASE + 0x5554);
  putreg32(0x55, XMC4_UNCACHED_PFLASH_BASE + 0xaaa8);
  putreg32(0x00, XMC4_UNCACHED_PFLASH_BASE + 0x553c);
  putreg32(password_0, XMC4_UNCACHED_PFLASH_BASE + 0xaaa8);
  putreg32(password_1, XMC4_UNCACHED_PFLASH_BASE + 0xaaa8);
  putreg32(0x08, XMC4_UNCACHED_PFLASH_BASE + 0x5558);
}

void xmc4_flash_resume_protection(void)
{
  putreg32(0x5e, XMC4_UNCACHED_PFLASH_BASE + 0x5554);
}

void xmc4_flash_clear_status(void)
{
  putreg32(0xf5, XMC4_UNCACHED_PFLASH_BASE + 0x5554);
}
