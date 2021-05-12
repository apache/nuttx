/**************************************************************************//**
 * @file     hal_spic.h
 * @brief    The header file of hal_spic.c
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

#ifndef _HAL_SPIC_H_
#define _HAL_SPIC_H_
#include "cmsis.h"

#ifdef  __cplusplus
extern "C"
{
#endif

/**

        \addtogroup hs_hal_spic Flash Controller
        @{
*/




#define SPIC_DATA_BASE 0x1040       //!< Define address offset to store calibration setting

#define FLASH_CS_PIN_SEL0 PB_0
#define FLASH_CS_PIN_SEL1 PA_7
#define FLASH_CS_PIN_SEL2 PA_15

#define FLASH_CLK_PIN_SEL0 PB_1
#define FLASH_CLK_PIN_SEL1 PA_8
#define FLASH_CLK_PIN_SEL2 PA_16

#define FLASH_D0_PIN_SEL0 PB_2
#define FLASH_D0_PIN_SEL1 PA_11
#define FLASH_D0_PIN_SEL2 PA_19

#define FLASH_D1_PIN_SEL0 PB_3
#define FLASH_D1_PIN_SEL1 PA_10
#define FLASH_D1_PIN_SEL2 PA_20

#define FLASH_D2_PIN_SEL0 PB_4
#define FLASH_D2_PIN_SEL1 PA_9
#define FLASH_D2_PIN_SEL2 PA_17

#define FLASH_D3_PIN_SEL0 PB_5
#define FLASH_D3_PIN_SEL1 PA_12
#define FLASH_D3_PIN_SEL2 PA_18

#define SPIC_RESTORE_VALID      0x871078EF
/**
  \brief  The data struct of recovery data
*/
typedef struct hal_spic_restore_setting_s {
    pflash_cmd_t cmd;                               //!< The pointer pointing to flash commands
    spic_init_para_t spic_init_data;                //!< Calibration settings for recovery
    flash_pin_sel_t flash_pin_sel;                  //!< Pinmux selection
    u8   flash_type;                                //!< Flash type
    u8   spic_bit_mode;                             //!< Current IO mode used by the adaptor
    u8   quad_pin_sel;                              //!< Record if the quad IO pins are used
    u32  recored;                                   //!< a pattern to make data in this struct is valid or not
} hal_spic_restore_setting_t, *phal_spic_restore_setting_t;

/**

        \addtogroup hs_hal_spic_ram_func Flash Controller HAL RAM APIs
        @{
*/


void spic_load_default_setting(pspic_init_para_t pspic_init_data);
u8 spic_query_system_clk(void);
void spic_clock_ctrl(u8 ctl);
hal_status_t spic_init_setting(phal_spic_adaptor_t phal_spic_adaptor, u8 spic_bit_mode);
void spic_load_calibration_setting(phal_spic_adaptor_t phal_spic_adaptor);
void spic_store_calibration_setting(phal_spic_adaptor_t phal_spic_adaptor);
void spic_config_auto_mode(phal_spic_adaptor_t phal_spic_adaptor);
void spic_config_user_mode(phal_spic_adaptor_t phal_spic_adaptor);
BOOL spic_verify_calibration_para(void);
void spic_set_chnl_num(phal_spic_adaptor_t phal_spic_adaptor);
void spic_set_delay_line(u8 delay_line);
void spic_rx_cmd(phal_spic_adaptor_t phal_spic_adaptor, u8 cmd, u8 data_phase_len, u8 *pdata);
void spic_tx_cmd_no_check(phal_spic_adaptor_t phal_spic_adaptor, u8 cmd, u8 data_phase_len, u8 *pdata);
void spic_tx_cmd(phal_spic_adaptor_t phal_spic_adaptor, u8 cmd, u8 data_phase_len, u8 *pdata);
void spic_wait_ready(SPIC_Type *spic_dev);
void spic_flush_fifo(SPIC_Type *spic_dev);
hal_status_t spic_pinmux_ctl(phal_spic_adaptor_t phal_spic_adaptor, u8 ctl);
hal_status_t spic_init(phal_spic_adaptor_t phal_spic_adaptor, u8 spic_bit_mode, pflash_pin_sel_t pflash_pin_sel);
hal_status_t spic_deinit(phal_spic_adaptor_t phal_spic_adaptor);
BOOL spic_calibration(phal_spic_adaptor_t phal_spic_adaptor, u32 default_dummy_cycle);
void spic_store_setting(phal_spic_adaptor_t phal_spic_adaptor, phal_spic_restore_setting_t phal_spic_setting);
void spic_recover_setting(phal_spic_adaptor_t phal_spic_adaptor, phal_spic_restore_setting_t phal_spic_setting);

/** *@} */ /* End of group hs_hal_spic_ram_func */

/** *@} */ /* End of group hs_hal_spic */

#ifdef  __cplusplus
}
#endif


#endif  // end of "#define _HAL_SPIC_H_"

