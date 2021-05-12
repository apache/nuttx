/**************************************************************************//**
 * @file     hal_flash_boot.h
 * @brief    This file implements the entry functions of the Flasg Booting
 *           API ROM functions.
 * 
 * @version  V1.00
 * @date     2017-07-07
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

#ifndef _HAL_FLASH_BOOT_H_
#define _HAL_FLASH_BOOT_H_
#include "cmsis.h"
#include "fw_img.h"
  
#ifdef  __cplusplus
 extern "C"
 {
#endif
 
extern const hal_flash_boot_stubs_t hal_flash_boot_stubs;     // symbol from linker script
 
#if !defined(CONFIG_BUILD_NONSECURE)
__STATIC_INLINE 
void *boot_get_partition_tbl(void)
{
    return hal_flash_boot_stubs.ppartition_tbl;
}

__STATIC_INLINE 
void *boot_get_fw1_key_tbl(void)
{
    return hal_flash_boot_stubs.get_fw1_key_tbl();
}

__STATIC_INLINE
void *boot_get_fw2_key_tbl(void)
{
    return hal_flash_boot_stubs.get_fw2_key_tbl();
}

__STATIC_INLINE
void boot_clear_partition_tbl(void)
{
    hal_flash_boot_stubs.clear_export_partition_tbl();
}

__STATIC_INLINE
void boot_loader_erase(uint32_t code_start, uint32_t code_size, uint32_t img2_entry,
                       hal_xip_sce_cfg_t *pxip_sce_cfg, hal_img1_tmp_buf_t *ptmp_buf)
{
    hal_flash_boot_stubs.erase_boot_loader(code_start, code_size, img2_entry, pxip_sce_cfg, ptmp_buf);
}

__STATIC_INLINE
void *get_fw_img_info_tbl(void)
{
    return hal_flash_boot_stubs.fw_img_info_tbl_query();
}

__STATIC_INLINE
int32_t fw_img_update_over_uart (hal_uart_adapter_t *potu_uart, uint32_t flash_sel, uint32_t flash_offset)
{
    return hal_flash_boot_stubs.otu_fw_download (potu_uart, flash_sel, flash_offset);
}

#endif  // end of "#if !defined(CONFIG_BUILD_NONSECURE)"

#ifdef  __cplusplus
 }
#endif
 
 
#endif  // end of "#define _HAL_FLASH_BOOT_H_"
 
