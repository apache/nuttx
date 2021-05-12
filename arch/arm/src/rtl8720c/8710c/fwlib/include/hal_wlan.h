/**************************************************************************//**
* @file         hal_wlan.h
* @brief       The HAL API implementation for the WLAN
* 
* @version    V1.00
* @date        2017-05-12
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


#ifndef _HAL_WLAN_H_
#define _HAL_WLAN_H_
#include "cmsis.h"
 
#ifdef  __cplusplus
 extern "C"
 {
#endif

hal_status_t hal_wlan_pwr_on(void);
hal_status_t hal_wlan_pwr_off(void);

#ifdef  __cplusplus
}
#endif

#endif //#ifndef _HAL_WLAN_H_

