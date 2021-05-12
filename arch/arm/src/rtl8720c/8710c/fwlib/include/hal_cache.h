/**************************************************************************//**
 * @file     hal_cache.h
 * @brief    The HAL API implementation for the Cache control.
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

#ifndef _HAL_CACHE_H_
#define _HAL_CACHE_H_
#include "cmsis.h"
 
#ifdef  __cplusplus
extern "C"
{
#endif

/** 
* @addtogroup hs_hal_cache Cache
* @{
*/

extern const hal_cache_func_stubs_t hal_cache_stubs;

/**
  \brief   Enable I-Cache
  \details Turns on I-Cache
  */
__STATIC_INLINE void icache_enable (void)
{
    hal_cache_stubs.icache_enable ();
}


/**
  \brief   Disable I-Cache
  \details Turns off I-Cache
  */
__STATIC_INLINE void icache_disable (void)
{
    hal_cache_stubs.icache_disable ();
}


/**
  \brief   Invalidate I-Cache
  \details Invalidates I-Cache
  */
__STATIC_INLINE void icache_invalidate (void)
{
    hal_cache_stubs.icache_invalidate ();
}


/**
  \brief   Enable D-Cache
  \details Turns on D-Cache
  */
__STATIC_INLINE void dcache_enable (void)
{
    hal_cache_stubs.dcache_enable ();
}


/**
  \brief   Disable D-Cache
  \details Turns off D-Cache
  */
__STATIC_INLINE void dcache_disable (void)
{
    hal_cache_stubs.dcache_disable ();
}


/**
  \brief   Invalidate D-Cache
  \details Invalidates D-Cache
  */
__STATIC_INLINE void dcache_invalidate (void)
{
    hal_cache_stubs.dcache_invalidate ();
}


/**
  \brief   Clean D-Cache
  \details Cleans D-Cache
  */
__STATIC_INLINE void dcache_clean (void)
{
    hal_cache_stubs.dcache_clean ();
}


/**
  \brief   Clean & Invalidate D-Cache
  \details Cleans and Invalidates D-Cache
  */
__STATIC_INLINE void dcache_clean_invalidate (void)
{
    hal_cache_stubs.dcache_clean_invalidate ();
}


/**
  \brief   D-Cache Invalidate by address
  \details Invalidates D-Cache for the given address
  \param[in]   addr    address (aligned to 32-byte boundary)
  \param[in]   dsize   size of memory block (in number of bytes)
*/
__STATIC_INLINE void dcache_invalidate_by_addr (uint32_t *addr, int32_t dsize)
{
    hal_cache_stubs.dcache_invalidate_by_addr (addr, dsize);
}


/**
  \brief   D-Cache Clean by address
  \details Cleans D-Cache for the given address
  \param[in]   addr    address (aligned to 32-byte boundary)
  \param[in]   dsize   size of memory block (in number of bytes)
*/
__STATIC_INLINE void dcache_clean_by_addr (uint32_t *addr, int32_t dsize)
{
    hal_cache_stubs.dcache_clean_by_addr (addr, dsize);
}


/**
  \brief   D-Cache Clean and Invalidate by address
  \details Cleans and invalidates D_Cache for the given address
  \param[in]   addr    address (aligned to 32-byte boundary)
  \param[in]   dsize   size of memory block (in number of bytes)
*/
__STATIC_INLINE void dcache_clean_invalidate_by_addr (uint32_t *addr, int32_t dsize)
{
    hal_cache_stubs.dcache_clean_invalidate_by_addr (addr, dsize);
}

/** @} */ /* End of group hs_hal_cache */

#ifdef  __cplusplus
}
#endif


#endif  // end of "#define _HAL_CACHE_H_"

