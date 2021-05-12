/**************************************************************************//**
* @file        rtl8710c_efuse.h
* @brief       The fundamental definition for rtl8710c Efuse module.
*
* @version    V1.00
* @date        2018-09-12
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

/**
 * @addtogroup hs_hal_efuse EFUSE
 * @ingroup 8710c_hal
 * @{
 * @brief The EFUSE HAL on AmebaZ2 platform.
 */

#ifndef RTL8710C_EFUSE_H
#define RTL8710C_EFUSE_H

#ifdef __cplusplus
extern "C" {
#endif

/// Defines the Super Security Key length.
#define SUSEC_KEY_LENGTH    32 //super security key length
/// Defines the Security Key length.
#define SEC_KEY_LENGTH      32 //security key length

#define LDO_OUT_NONSET_VOLT         0x0
/// Defines the default LDO Voltage.
#define LDO_OUT_DEFAULT_VOLT        0x0 // Default LDO Voltage not sure. Need to confirm with DD and test on asic board
/// Defines the default Efuse control setting.
#define EFUSE_CTRL_SETTING          (uint32_t)(0x33300000)

/// @cond DOXYGEN_ROM_HAL_API

/**
 * @addtogroup hs_hal_efuse_rom_func EFUSE HAL ROM APIs.
 * @ingroup hs_hal_efuse
 * @{
 */

/*  Function declaration   */
void hal_efuse_power_switch_rtl8710c(uint8_t bwrite, uint8_t	pwr_state, uint8_t l25out_voltage);
uint32_t hal_efuse_autoload_en_rtl8710c(uint8_t enable, uint8_t l25out_voltage);
uint32_t hal_efuse_hci_autoload_en_rtl8710c(uint8_t enable, uint8_t l25out_voltage);
uint32_t hal_efuse_read_rtl8710c(uint32_t ctrl_setting,	uint16_t addr, uint8_t *pdata, uint8_t l25out_voltage);
uint32_t hal_efuse_write_rtl8710c(uint32_t ctrl_setting, uint16_t	addr, uint8_t data, uint8_t l25out_voltage);
uint32_t hal_user_otp_get_rtl8710c(uint32_t ctrl_setting, uint8_t *puser_otp, uint8_t l25out_voltage);
uint32_t hal_user_otp_set_rtl8710c(uint32_t ctrl_setting, uint8_t *puser_otp, uint8_t l25out_voltage);
uint32_t hal_sec_efuse_read_rtl8710c(uint32_t ctrl_setting,	uint16_t addr, uint8_t *pdata, uint8_t l25out_voltage);
uint32_t hal_sec_efuse_write_rtl8710c(uint32_t ctrl_setting, uint16_t	addr, uint8_t data, uint8_t l25out_voltage);
uint32_t hal_sec_key_get_rtl8710c(uint32_t ctrl_setting, uint8_t *psec_key, uint8_t key_num, uint32_t length, uint8_t l25out_voltage);
uint32_t hal_sec_key_write_rtl8710c(uint32_t ctrl_setting, uint8_t *psec_key, uint8_t key_num, uint8_t l25out_voltage);
uint32_t hal_susec_key_get_rtl8710c(uint32_t ctrl_setting, uint8_t *psusec_key, uint8_t l25out_voltage);
uint32_t hal_susec_key_write_rtl8710c(uint32_t ctrl_setting, uint8_t *psusec_key, uint8_t l25out_voltage);
uint32_t hal_s_jtag_key_write_rtl8710c(uint32_t ctrl_setting, uint8_t *pkey, uint8_t l25out_voltage);
uint32_t hal_ns_jtag_key_write_rtl8710c(uint32_t ctrl_setting, uint8_t *pkey, uint8_t l25out_voltage);
uint32_t hal_check_already_write_rtl8710c(uint32_t ctrl_setting, uint32_t addr, uint8_t length, uint8_t l25out_voltage);
//uint32_t hal_sec_zone_write_rtl8710c(uint32_t ctrl_setting, uint8_t offset, uint8_t length, uint8_t *pcontent, uint8_t l25out_voltage);


/**
 * \brief  The data structure of the stubs function for the EFUSE HAL functions in ROM
 */
typedef struct hal_efuse_func_stubs_s {
    uint32_t (*hal_efuse_autoload_en) (uint8_t enable, uint8_t l25out_voltage);
    uint32_t (*hal_efuse_hci_autoload_en) (uint8_t enable, uint8_t l25out_voltage);
    uint32_t (*hal_efuse_read) (uint32_t ctrl_setting,	uint16_t addr, uint8_t *data, uint8_t l25out_voltage);
    uint32_t (*hal_efuse_write) (uint32_t ctrl_setting, uint16_t addr, uint8_t data, uint8_t l25out_voltage);
    uint32_t (*hal_user_otp_get) (uint32_t ctrl_setting, uint8_t *puser_otp, uint8_t l25out_voltage);
    uint32_t (*hal_user_otp_set) (uint32_t ctrl_setting, uint8_t *puser_otp, uint8_t l25out_voltage);
    uint32_t (*hal_sec_efuse_read) (uint32_t ctrl_setting,	uint16_t addr, uint8_t *pdata, uint8_t l25out_voltage);
    uint32_t (*hal_sec_efuse_write) (uint32_t ctrl_setting, uint16_t addr, uint8_t data, uint8_t l25out_voltage);
    uint32_t (*hal_sec_key_get) (uint32_t ctrl_setting, uint8_t *psec_key, uint8_t key_num, uint32_t length, uint8_t l25out_voltage);
    uint32_t (*hal_sec_key_write) (uint32_t ctrl_setting, uint8_t *psec_key, uint8_t key_num, uint8_t l25out_voltage);
    uint32_t (*hal_susec_key_get) (uint32_t ctrl_setting, uint8_t *psusec_key, uint8_t l25out_voltage);
    uint32_t (*hal_susec_key_write) (uint32_t ctrl_setting, uint8_t *psusec_key, uint8_t l25out_voltage);    
    uint32_t (*hal_sec_zone_write) (uint32_t ctrl_setting, uint8_t offset, uint8_t length, uint8_t *pcontent, uint8_t l25out_voltage);
    uint32_t (*hal_s_jtag_key_write) (uint32_t ctrl_setting, uint8_t *pkey, uint8_t l25out_voltage);
    uint32_t (*hal_ns_jtag_key_write) (uint32_t ctrl_setting, uint8_t *pkey, uint8_t l25out_voltage);
    uint32_t reserved[12]; // reserved space for next ROM code version function table extending.
}hal_efuse_func_stubs_t;

/** @} */ /* End of group hs_hal_efuse_rom_func */

/// @endcond    /* End of cond DOXYGEN_ROM_HAL_API */

#ifdef __cplusplus
}
#endif

#endif /* RTL8710C_EFUSE_H */

/** @} */ /* End of group hs_hal_efuse */

