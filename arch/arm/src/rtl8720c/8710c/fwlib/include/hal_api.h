/**************************************************************************//**
 * @file     hal_api.h
 * @brief    HAL common macro definitions. 
 * @version  V1.00
 * @date     2016-07-20
 *
 ******************************************************************************
 *
 * Copyright(c) 2007 - 2016 Realtek Corporation. All rights reserved.
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

#ifndef _HAL_API_H_
#define _HAL_API_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "basic_types.h"

#define HAL_READ32(base, addr)            \
        rtk_le32_to_cpu(*((volatile u32*)(base + addr)))
    
#define HAL_WRITE32(base, addr, value32)  \
        ((*((volatile u32*)(base + addr))) = rtk_cpu_to_le32(value32))


#define HAL_READ16(base, addr)            \
        rtk_le16_to_cpu(*((volatile u16*)(base + addr)))
        
#define HAL_WRITE16(base, addr, value)  \
        ((*((volatile u16*)(base + addr))) = rtk_cpu_to_le16(value))
    

#define HAL_READ8(base, addr)            \
        (*((volatile u8*)(base + addr)))
            
#define HAL_WRITE8(base, addr, value)  \
        ((*((volatile u8*)(base + addr))) = value)

#define HAL_WAIT_FOREVER            (0xFFFFFFFF)
#define HAL_OK              (0x00)
#define HAL_BUSY            (0x01)
#define HAL_TIMEOUT         (0x02)
#define HAL_ERR_PARA        (0x03)  // error with invaild parameters 
#define HAL_ERR_MEM         (0x04)  // error with memory allocation failed
#define HAL_ERR_HW          (0x05)  // error with hardware error
#define HAL_NOT_READY       (0x06)  // error with data not ready
#define HAL_NOT_FOUND       (0x07)  // error with data not found
#define HAL_NO_RESOURCE     (0x08)  // error with HW resource insufficent 
#define HAL_ERR_CONFLICT    (0x09)  // error with HW conflict
#define HAL_ERR_OS          (0x0A)  // error with RTOS functions
#define HAL_ERR_UNKNOWN     (0xEE)  // unknown error

typedef uint32_t   hal_status_t;
typedef uint32_t   HAL_Status;

#ifdef __cplusplus
}
#endif

#endif //_HAL_API_H_
