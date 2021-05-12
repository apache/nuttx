/**************************************************************************//**
 * @file     hal_flash.h
 * @brief    The header file of hal_flash.c.
 * @version  1.00
 * @date     2017-08-22
 *
 * @note
 *
 ******************************************************************************
 *
 * Copyright(c) 2007 - 2017 Realtek Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/

#ifndef _HAL_FLASH_H_
#define _HAL_FLASH_H_
#include "cmsis.h"

#ifdef  __cplusplus
extern "C"
{
#endif

/**

        \addtogroup hs_hal_flash FLASH
        @{
*/

#define FLASH_TYPE_GD 4
#define FLASH_TYPE_GD32 5
#define FLASH_TYPE_XTX 6
#define FLASH_TYPE_EON 7
#define FLASH_TYPE_BOYA 8
#define FLASH_TYPE_XMC 9
#define FLASH_TYPE_ZBIT 10

#define FLASH_CMD_ENSO      0xB1            // enter secured OTP
#define FLASH_CMD_EXSO      0xC1            // exit secured OTP
#define FLASH_CMD_RDSCUR    0x2B            // read security register
#define FLASH_CMD_WRSCUR    0x2F            // write security register    

/**

        \addtogroup hs_hal_gdma_flash_func FLASH HAL RAM APIs
        \ingroup hs_hal_flash
        @{
*/


void hal_flash_read_unique_id (phal_spic_adaptor_t phal_spic_adaptor, uint8_t *buf, uint8_t len);
hal_status_t hal_flash_read_id (phal_spic_adaptor_t phal_spic_adaptor);
void hal_flash_set_write_enable (phal_spic_adaptor_t phal_spic_adaptor);
void hal_flash_set_status (phal_spic_adaptor_t phal_spic_adaptor, u8 cmd, u8 data);
void hal_flash_set_status_no_check (phal_spic_adaptor_t phal_spic_adaptor, u8 cmd, u8 data);
void hal_flash_set_status_with_addr (phal_spic_adaptor_t phal_spic_adaptor, u8 cmd, u8 addr, u8 data);
void hal_flash_set_extended_addr (phal_spic_adaptor_t phal_spic_adaptor, u8 data);
void hal_flash_set_write_protect_mode (phal_spic_adaptor_t phal_spic_adaptor, u8 mode);
u8 hal_flash_get_status (phal_spic_adaptor_t phal_spic_adaptor, u8 cmd);
u8 hal_flash_get_status_with_addr (phal_spic_adaptor_t phal_spic_adaptor, u8 cmd, u8 addr);
u8 hal_flash_get_extended_addr (phal_spic_adaptor_t phal_spic_adaptor);
void hal_flash_wait_ready (phal_spic_adaptor_t phal_spic_adaptor);
void hal_flash_chip_erase (phal_spic_adaptor_t phal_spic_adaptor);
void hal_flash_64k_block_erase (phal_spic_adaptor_t phal_spic_adaptor, u32 address);
void hal_flash_32k_block_erase (phal_spic_adaptor_t phal_spic_adaptor, u32 address);
void hal_flash_sector_erase (phal_spic_adaptor_t phal_spic_adaptor, u32 address);
u8 hal_flash_query_sector_protect_state(phal_spic_adaptor_t phal_spic_adaptor, u32 address);
void hal_flash_protect_sector (phal_spic_adaptor_t phal_spic_adaptor, u32 address);
void hal_flash_unprotect_sector (phal_spic_adaptor_t phal_spic_adaptor, u32 address);
void hal_flash_global_lock (phal_spic_adaptor_t phal_spic_adaptor);
void hal_flash_global_unlock (phal_spic_adaptor_t phal_spic_adaptor);
void hal_flash_set_dummy_cycle (phal_spic_adaptor_t phal_spic_adaptor);
void hal_flash_set_quad_enable (phal_spic_adaptor_t phal_spic_adaptor);
void hal_flash_unset_quad_enable (phal_spic_adaptor_t phal_spic_adaptor);
void hal_flash_enable_qpi (phal_spic_adaptor_t phal_spic_adaptor);
void hal_flash_return_spi (phal_spic_adaptor_t phal_spic_adaptor);
hal_status_t hal_flash_enter_power_down (phal_spic_adaptor_t phal_spic_adaptor);
hal_status_t hal_flash_release_from_power_down (phal_spic_adaptor_t phal_spic_adaptor);
void hal_flash_stream_read (phal_spic_adaptor_t phal_spic_adaptor, u32 length, u32 addr, u8 *data);
void hal_flash_stream_write (phal_spic_adaptor_t phal_spic_adaptor, u32 length, u32 addr, u8 *data);
void hal_flash_burst_read (phal_spic_adaptor_t phal_spic_adaptor, u32 length, u32 addr, u8 *data);
void hal_flash_burst_write (phal_spic_adaptor_t phal_spic_adaptor, u32 length, u32 addr, u8 *data);
void hal_flash_read_write_flash (phal_spic_adaptor_t phal_spic_adaptor, u32 length, u32 addr, u8 *data);
void hal_flash_page_program (phal_spic_adaptor_t phal_spic_adaptor, u32 length, u32 addr, u8 *data);
void hal_flash_reset_to_spi (phal_spic_adaptor_t phal_spic_adaptor);
void hal_flash_support_new_type (phal_spic_adaptor_t phal_spic_adaptor);
u8 hal_flash_get_size (phal_spic_adaptor_t phal_spic_adaptor);


/** *@} */ /* End of group hs_hal_flash_ram_func */

/** *@} */ /* End of group hs_hal_flash */

#ifdef  __cplusplus
}
#endif


#endif  // end of "#define _HAL_FLASH_H_"


