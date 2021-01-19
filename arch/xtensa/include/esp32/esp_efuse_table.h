/****************************************************************************
 * arch/xtensa/include/esp32/esp_efuse_table.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

extern const efuse_desc_t *ESP_EFUSE_MAC_FACTORY[];
extern const efuse_desc_t *ESP_EFUSE_MAC_FACTORY_CRC[];
extern const efuse_desc_t *ESP_EFUSE_MAC_CUSTOM_CRC[];
extern const efuse_desc_t *ESP_EFUSE_MAC_CUSTOM[];
extern const efuse_desc_t *ESP_EFUSE_MAC_CUSTOM_VER[];
extern const efuse_desc_t *ESP_EFUSE_SECURE_BOOT_KEY[];
extern const efuse_desc_t *ESP_EFUSE_ABS_DONE_0[];
extern const efuse_desc_t *ESP_EFUSE_ENCRYPT_FLASH_KEY[];
extern const efuse_desc_t *ESP_EFUSE_ENCRYPT_CONFIG[];
extern const efuse_desc_t *ESP_EFUSE_DISABLE_DL_ENCRYPT[];
extern const efuse_desc_t *ESP_EFUSE_DISABLE_DL_DECRYPT[];
extern const efuse_desc_t *ESP_EFUSE_DISABLE_DL_CACHE[];
extern const efuse_desc_t *ESP_EFUSE_FLASH_CRYPT_CNT[];
extern const efuse_desc_t *ESP_EFUSE_DISABLE_JTAG[];
extern const efuse_desc_t *ESP_EFUSE_CONSOLE_DEBUG_DISABLE[];
extern const efuse_desc_t *ESP_EFUSE_UART_DOWNLOAD_DIS[];
extern const efuse_desc_t *ESP_EFUSE_WR_DIS_FLASH_CRYPT_CNT[];
extern const efuse_desc_t *ESP_EFUSE_WR_DIS_BLK1[];
extern const efuse_desc_t *ESP_EFUSE_WR_DIS_BLK2[];
extern const efuse_desc_t *ESP_EFUSE_WR_DIS_BLK3[];
extern const efuse_desc_t *ESP_EFUSE_RD_DIS_BLK1[];
extern const efuse_desc_t *ESP_EFUSE_RD_DIS_BLK2[];
extern const efuse_desc_t *ESP_EFUSE_RD_DIS_BLK3[];
extern const efuse_desc_t *ESP_EFUSE_CHIP_VER_DIS_APP_CPU[];
extern const efuse_desc_t *ESP_EFUSE_CHIP_VER_DIS_BT[];
extern const efuse_desc_t *ESP_EFUSE_CHIP_VER_PKG[];
extern const efuse_desc_t *ESP_EFUSE_CHIP_CPU_FREQ_LOW[];
extern const efuse_desc_t *ESP_EFUSE_CHIP_CPU_FREQ_RATED[];
extern const efuse_desc_t *ESP_EFUSE_CHIP_VER_REV1[];
extern const efuse_desc_t *ESP_EFUSE_CHIP_VER_REV2[];
extern const efuse_desc_t *ESP_EFUSE_XPD_SDIO_REG[];
extern const efuse_desc_t *ESP_EFUSE_SDIO_TIEH[];
extern const efuse_desc_t *ESP_EFUSE_SDIO_FORCE[];
extern const efuse_desc_t *ESP_EFUSE_ADC_VREF_AND_SDIO_DREF[];
extern const efuse_desc_t *ESP_EFUSE_ADC1_TP_LOW[];
extern const efuse_desc_t *ESP_EFUSE_ADC2_TP_LOW[];
extern const efuse_desc_t *ESP_EFUSE_ADC1_TP_HIGH[];
extern const efuse_desc_t *ESP_EFUSE_ADC2_TP_HIGH[];
extern const efuse_desc_t *ESP_EFUSE_SECURE_VERSION[];

#ifdef __cplusplus
}
#endif

