/**************************************************************************//**
 * @file     hal_pinmux.h
 * @brief    The HAL API implementation for the pin mux managemment.
 * @version  V1.00
 * @date     2016-12-30
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

#ifndef _HAL_PINMUX_H_
#define _HAL_PINMUX_H_
#include "cmsis.h"

#ifdef  __cplusplus
extern "C"
{
#endif

/**
 *  @brief To register a list of IO pin to the pin mux magagement.
 *         The pin mux management will do the checking of pin conflict and pin valid.
 *         And then power up the IO pad.
 *
 *  @param[in]  pin_list  The IO pin list. The list is end with a byte value 0xFF.
 *  @param[in]  periphl_id  The ID of the peripheral will use these pins.
 *
 *  @return     HAL_OK:  Pin list register OK.
 *  @return     HAL_ERR_CONFLICT:  Pin conflict. At least one of pin in the list
 *              is occupied by other peripheral device.
 *  @return     HAL_ERR_HW: At least one of pin in the pin list is invalid for this IC.
 */
hal_status_t hal_pinmux_register (uint8_t pin_name, uint8_t periphl_id);

/**
 *  @brief Unregister a list of IO pins from the pin mux management.
 *         The pin mux management will power down the IO pads.
 *
 *  @param[in]  pin_list  The IO pin list. The list is end with a byte value 0xFF.
 *  @param[in]  periphl_id  The ID of the peripheral own these pins.
 *
 *  @return     HAL_OK:  Pin unregister OK.
 *  @return     HAL_ERR_PARA:  The peripheral doesn't own these pins.
 */
hal_status_t hal_pinmux_unregister (uint8_t pin_name, uint8_t periphl_id);

#if defined(CONFIG_BUILD_NONSECURE)
// Build for Non-Secure
hal_status_t hal_pinmux_register_nsc (uint8_t pin_name, uint8_t periphl_id);
hal_status_t hal_pinmux_unregister_nsc (uint8_t pin_name, uint8_t periphl_id);
#endif

#ifdef  __cplusplus
}
#endif

#endif  // end of "#define _HAL_PINMUX_H_"

