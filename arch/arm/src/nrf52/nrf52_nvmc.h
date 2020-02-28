/****************************************************************************
 * arch/arm/src/nrf52/nrf52_nvmc.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Ported from the Nordic SDK, this is the original license:
 *
 * Copyright (c) 2012 - 2018, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_NRF52_NRF52_NVMC_H
#define __ARCH_ARM_SRC_NRF52_NRF52_NVMC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: nrf_nvmc_page_erase
 *
 * Description:
 *   Erase a page in flash. This is required before writing to any
 *   address in the page.
 *
 * Input Parameter:
 *   address - Start address of the page.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

void nrf_nvmc_page_erase(uint32_t address);

/****************************************************************************
 * Name: nrf_nvmc_write_byte
 *
 * Description:
 *   The function reads the word containing the byte, and then
 *   rewrites the entire word.
 *
 * Input Parameter:
 *   address - Address to write to.
 *   value   - Value to write.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

void nrf_nvmc_write_byte(uint32_t address, uint8_t value);

/****************************************************************************
 * Name: nrf_nvmc_write_word
 *
 * Description:
 *   Write a 32-bit word to flash.
 *
 * Input Parameter:
 *   address - Address to write to.
 *   value   - Value to write.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

void nrf_nvmc_write_word(uint32_t address, uint32_t value);

/****************************************************************************
 * Name: nrf_nvmc_write_bytes
 *
 * Description:
 *   Write consecutive bytes to flash.
 *
 * Input Parameter:
 *   address   - Address to write to.
 *   src       - Pointer to data to copy from.
 *   num_bytes - Number of bytes in src to write.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

void nrf_nvmc_write_bytes(uint32_t address, const uint8_t *src,
                          uint32_t num_bytes);

/****************************************************************************
 * Name: nrf_nvmc_write_words
 *
 * Description:
 *   Write consecutive words to flash.
 *
 * Input Parameter:
 *   address   - Address to write to.
 *   src       - Pointer to data to copy from.
 *   num_words - Number of words in src to write.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

void nrf_nvmc_write_words(uint32_t address, const uint32_t *src,
                          uint32_t num_words);

/****************************************************************************
 * Name: nrf_nvmc_enable_icache
 *
 * Description:
 *   Enable I-Cache for Flash
 *
 * Input Parameter:
 *   flag - Flag to enable or disable.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

void nrf_nvmc_enable_icache(bool flag);

/****************************************************************************
 * Name: nrf_nvmc_enable_profile
 *
 * Description:
 *   Enable profiling I-Cache for flash
 *
 * Input Parameter:
 *   flag - Flag to enable or disable.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

void nrf_nvmc_enable_profile(bool flag);

/****************************************************************************
 * Name: nrf_nvmc_get_profiling_ihit
 *
 * Description:
 *   Get I-Hit for profiling
 *
 * Input Parameter:
 *   None
 *
 * Returned Values:
 *   Number of cache hits.
 *
 ****************************************************************************/

uint32_t nrf_nvmc_get_profiling_ihit(void);

/****************************************************************************
 * Name: nrf_nvmc_get_profiling_imiss
 *
 * Description:
 *   Get I-Miss for profiling
 *
 * Input Parameter:
 *   None
 *
 * Returned Values:
 *   Number of cache misses.
 *
 ****************************************************************************/

uint32_t nrf_nvmc_get_profiling_imiss(void);

/****************************************************************************
 * Name: nrf_nvmc_get_flash_size
 *
 * Description:
 *   Get internal FLASH size
 *
 * Input Parameter:
 *   None
 *
 * Returned Values:
 *   Flash size.
 *
 ****************************************************************************/

uint32_t nrf_nvmc_get_flash_size(void);

/****************************************************************************
 * Name: nrf_nvmc_get_ram_size
 *
 * Description:
 *   Get internal RAM size.
 *
 * Input Parameter:
 *   None
 *
 * Returned Values:
 *   RAM size.
 *
 ****************************************************************************/

uint32_t nrf_nvmc_get_ram_size(void);

/****************************************************************************
 * Name: system_image_start_address
 *
 * Description:
 *   Get system image code begin address.
 *
 * Input Parameter:
 *   None
 *
 * Returned Values:
 *   Address where code starts.
 *
 ****************************************************************************/

uint32_t system_image_start_address(void);

/****************************************************************************
 * Name: system_image_ro_section_end
 *
 * Description:
 *   Get system image end address of read only section.
 *
 * Input Parameter:
 *   None
 *
 * Returned Values:
 *   The end address of the RO section.
 *
 ****************************************************************************/

uint32_t system_image_ro_section_end(void);

/****************************************************************************
 * Name: system_image_data_section_size
 *
 * Description:
 *   Get system image data section size
 *
 * Input Parameter:
 *   None
 *
 * Returned Values:
 *   The size of the data section.
 *
 ****************************************************************************/

uint32_t system_image_data_section_size(void);

/****************************************************************************
 * Name: nrf_nvmc_read_dev_id0
 *
 * Description:
 *   Get device identifier 0
 *
 * Input Parameter:
 *   None
 *
 * Returned Values:
 *   Value of ID0
 *
 ****************************************************************************/

uint32_t nrf_nvmc_read_dev_id0(void);

/****************************************************************************
 * Name: nrf_nvmc_read_dev_id1
 *
 * Description:
 *   Get device identifier 1
 *
 * Input Parameter:
 *   None
 *
 * Returned Values:
 *   Value of ID1
 *
 ****************************************************************************/

uint32_t nrf_nvmc_read_dev_id1(void);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_NRF52_NRF52_NVMC_H */
