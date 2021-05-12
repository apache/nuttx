/**************************************************************************//**
 * @file     rtl8710c_cache.h
 * @brief    The HAL related definition and macros for the cache control functions.
 * @version  V1.00
 * @date     2017-02-02
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


#ifndef _RTL8710C_CACHE_H_
#define _RTL8710C_CACHE_H_

#ifdef __cplusplus
extern "C" {
#endif
/** 
 * @addtogroup hs_hal_cache Cache
 * @ingroup 8710c_hal
 * @{
 * @brief The APIs for cache configures and data cache sync.
 */

#include "core_tm9_cache.h"

/// @cond DOXYGEN_ROM_HAL_API

/** 
 * @addtogroup hs_hal_cache_rom_func
 * @{
 */

void icache_enable_rtl8710c (void);
void icache_disable_rtl8710c (void);
void icache_invalidate_rtl8710c (void);
void dcache_enable_rtl8710c (void);
void dcache_disable_rtl8710c (void);
void dcache_invalidate_rtl8710c (void);
void dcache_clean_rtl8710c (void);
void dcache_clean_invalidate_rtl8710c (void);
void dcache_invalidate_by_addr_rtl8710c (uint32_t *addr, int32_t dsize);
void dcache_clean_by_addr_rtl8710c (uint32_t *addr, int32_t dsize);
void dcache_clean_invalidate_by_addr_rtl8710c (uint32_t *addr, int32_t dsize);

/** @} */ /* End of group hs_hal_cache_rom_func */
/// @endcond /* End of condition DOXYGEN_ROM_HAL_API */

/**
  \brief   Enable D-Cache
  \details Turns on D-Cache
  */
__STATIC_INLINE BOOLEAN is_dcache_enabled (void)
{
  #if defined (__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    return ((SCB->CCR & (uint32_t)SCB_CCR_DC_Msk)?1:0);
  #endif
}

/**
  \brief  The data type for the stubs functions table of the Cache Control HAL functions in ROM.
*/
typedef struct hal_cache_func_stubs_s {
    void (*icache_enable) (void);
    void (*icache_disable) (void);
    void (*icache_invalidate) (void);
    void (*dcache_enable) (void);
    void (*dcache_disable) (void);
    void (*dcache_invalidate) (void);
    void (*dcache_clean) (void);
    void (*dcache_clean_invalidate) (void);
    void (*dcache_invalidate_by_addr) (uint32_t *addr, int32_t dsize);
    void (*dcache_clean_by_addr) (uint32_t *addr, int32_t dsize);
    void (*dcache_clean_invalidate_by_addr) (uint32_t *addr, int32_t dsize);
    uint32_t reserved[4];  // reserved space for next ROM code version function table extending.
} hal_cache_func_stubs_t;

/** @} */ /* End of group hs_hal_cache */

#ifdef __cplusplus
}
#endif

#endif  // #ifndef _RTL8710C_CACHE_H_

