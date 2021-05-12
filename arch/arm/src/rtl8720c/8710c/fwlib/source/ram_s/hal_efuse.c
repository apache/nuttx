/**************************************************************************//**
* @file        hal_efuse.c
* @brief       This file implements the EFUSE HAL functions.
*
* @version     V1.00
* @date        2019-11-25
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


#include "hal_efuse.h"

#if CONFIG_EFUSE_EN
/**
 * @addtogroup hs_hal_efuse EFUSE
 * @{
 */

/**
  * @brief The global EFUSE HAL adapter(management entity).
  */
extern hal_efuse_func_stubs_t hal_efuse_stubs;

/**
 *  @brief To enable hs efuse autoload
 *
 *  @param[in]  enable  enable.
 *
 *  @return  TRUE
 *  @return  FALSE
 */
uint32_t hal_efuse_autoload_en(uint8_t enable)
{
    return hal_efuse_stubs.hal_efuse_autoload_en(enable, LDO_OUT_DEFAULT_VOLT);
}

/**
 *  @brief To enable hci efuse autoload
 *
 *  @param[in]  enable  enable.
 *
 *  @return  TRUE
 *  @return  FALSE
 */
uint32_t hal_efuse_hci_autoload_en(uint8_t enable)
{
    return hal_efuse_stubs.hal_efuse_hci_autoload_en(enable, LDO_OUT_DEFAULT_VOLT);
}

/**
 *  @brief To read hs efuse
 *
 *  @param[in]  addr            read address.
 *  @param[in]  pdata           address of read back data.
 *  @param[in]  l25out_voltage  LDOE25 voltage select.
 *
 *  @return  TRUE
 *  @return  FALSE
 */
uint32_t hal_efuse_read (uint16_t addr, uint8_t *pdata, uint8_t l25out_voltage)
{
    return hal_efuse_stubs.hal_efuse_read(EFUSE_CTRL_SETTING, addr, pdata, l25out_voltage);
}

/**
 *  @brief To write hs efuse
 *
 *  @param[in]  addr            write address.
 *  @param[in]  data            write data.
 *  @param[in]  l25out_voltage  LDOE25 voltage select.
 *
 *  @return  TRUE
 *  @return  FALSE
 */
uint32_t hal_efuse_write (uint16_t addr, uint8_t data, uint8_t l25out_voltage)
{
    return hal_efuse_stubs.hal_efuse_write(EFUSE_CTRL_SETTING, addr, data, l25out_voltage);
}

/**
 *  @brief To read hs efuse through security register
 *
 *  @param[in]  addr            read address.
 *  @param[in]  pdata           address of read back data.
 *  @param[in]  l25out_voltage  LDOE25 voltage select.
 *
 *  @return  TRUE
 *  @return  FALSE
 */
uint32_t hal_sec_efuse_read(uint16_t addr, uint8_t *pdata, uint8_t l25out_voltage)
{
    return hal_efuse_stubs.hal_sec_efuse_read(EFUSE_CTRL_SETTING, addr, pdata, l25out_voltage);
}

/**
 *  @brief To write hs efuse through security register
 *
 *  @param[in]  addr            write address.
 *  @param[in]  data            write data.
 *  @param[in]  l25out_voltage  LDOE25 voltage select.
 *
 *  @return  TRUE
 *  @return  FALSE
 */
uint32_t hal_sec_efuse_write(uint16_t addr, uint8_t data, uint8_t l25out_voltage)
{
    return hal_efuse_stubs.hal_sec_efuse_write(EFUSE_CTRL_SETTING, addr, data, l25out_voltage);
}

/**
 *  @brief To get security key
 *
 *  @param[in]  psec_key        adress of read back security key.
 *  @param[in]  key_num         select key number.
 *  @param[in]  length          security key length.
 *
 *  @return  TRUE
 *  @return  FALSE
 */
uint32_t hal_sec_key_get(uint8_t *psec_key, uint8_t key_num, uint32_t length)
{
    uint8_t l25out_voltage = LDO_OUT_DEFAULT_VOLT;
    return hal_efuse_stubs.hal_sec_key_get(EFUSE_CTRL_SETTING, psec_key, key_num, length, l25out_voltage);
}

/**
 *  @brief To write security key
 *
 *  @param[in]  psec_key        address of 32-byte security key.
 *  @param[in]  key_num         select key number.
 *
 *  @return  TRUE
 *  @return  FALSE
 */
uint32_t hal_sec_key_write(uint8_t *psec_key, uint8_t key_num)
{
    uint8_t l25out_voltage = LDO_OUT_DEFAULT_VOLT;
    return hal_efuse_stubs.hal_sec_key_write(EFUSE_CTRL_SETTING, psec_key, key_num, l25out_voltage);
}

/**
 *  @brief To get super security key
 *
 *  @param[in]  psec_key        adress of read back 32-byte super security key.
 *
 *  @return  TRUE
 *  @return  FALSE
 */
uint32_t hal_susec_key_get(uint8_t *psusec_key)
{
    uint8_t l25out_voltage = LDO_OUT_DEFAULT_VOLT;
    return hal_efuse_stubs.hal_susec_key_get(EFUSE_CTRL_SETTING, psusec_key, l25out_voltage);
}

/**
 *  @brief To write super security key
 *
 *  @param[in]  psusec_key      address of 32-byte super security key.
 *
 *  @return  TRUE
 *  @return  FALSE
 */
uint32_t hal_susec_key_write(uint8_t *psusec_key)
{
    uint8_t l25out_voltage = LDO_OUT_DEFAULT_VOLT;
    return hal_efuse_stubs.hal_susec_key_write(EFUSE_CTRL_SETTING, psusec_key, l25out_voltage);
}

/** 
 *  @brief To write a 128-bit key to the efuse for the Secure S-JTAG.
 *
 *  @param[in]  pkey            address of 16-byte S-JTAG key.
 * 
 *  @return  TRUE
 *  @return  FALSE
 */
uint32_t hal_s_jtag_key_write(uint8_t *pkey)
{
    uint8_t l25out_voltage = LDO_OUT_DEFAULT_VOLT;
    return hal_efuse_stubs.hal_s_jtag_key_write(EFUSE_CTRL_SETTING, pkey, l25out_voltage);
}

/** 
 *  @brief To write a 128-bit key to the efuse for the Secure NS-JTAG.
 *
 *  @param[in]  pkey            address of 16-byte NS-JTAG key.
 * 
 *  @return  TRUE
 *  @return  FALSE
 */
uint32_t hal_ns_jtag_key_write(uint8_t *pkey)
{
    uint8_t l25out_voltage = LDO_OUT_DEFAULT_VOLT;
    return hal_efuse_stubs.hal_ns_jtag_key_write(EFUSE_CTRL_SETTING, pkey, l25out_voltage);
}

/**
 *  @brief To get user otp
 *
 *  @param[in]  puser_otp       address of read back user otp.
 *
 *  @return  TRUE
 *  @return  FALSE
 */
uint32_t hal_user_otp_get(uint8_t *puser_otp)
{
    uint8_t l25out_voltage = LDO_OUT_DEFAULT_VOLT;
    return hal_efuse_stubs.hal_user_otp_get(EFUSE_CTRL_SETTING, puser_otp, l25out_voltage);
}

/**
 *  @brief To set user otp
 *
 *  @param[in]  puser_otp       address of user otp value.
 *
 *  @return  TRUE
 *  @return  FALSE
 */
uint32_t hal_user_otp_set(uint8_t *puser_otp)
{
    uint8_t l25out_voltage = LDO_OUT_DEFAULT_VOLT;
    return hal_efuse_stubs.hal_user_otp_set(EFUSE_CTRL_SETTING, puser_otp, l25out_voltage);
}

/**
 *  @brief To disable secure jtag
 *
 *  @return  TRUE
 *  @return  FALSE
 */
uint32_t hal_efuse_disable_sec_jtag (void)
{
    return hal_efuse_stubs.hal_efuse_write(EFUSE_CTRL_SETTING, 0x00, 0xF3, LDO_OUT_DEFAULT_VOLT);
}

/**
 *  @brief To disable non-secure jtag
 *
 *  @return  TRUE
 *  @return  FALSE
 */
uint32_t hal_efuse_disable_nonsec_jtag (void)
{
    return hal_efuse_stubs.hal_efuse_write(EFUSE_CTRL_SETTING, 0x00, 0xCF, LDO_OUT_DEFAULT_VOLT);
}

/** @} */ /* End of group hs_hal_efuse */

#endif  /* end of "#if CONFIG_EFUSE_EN" */

