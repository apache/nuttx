/****************************************************************************
 * arch/risc-v/include/esp32c3/esp_efuse_table.h
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

extern const efuse_desc_t *ESP_EFUSE_WR_DIS_RD_DIS[];
extern const efuse_desc_t *ESP_EFUSE_WR_DIS_GROUP_1[];
extern const efuse_desc_t *ESP_EFUSE_WR_DIS_GROUP_2[];
extern const efuse_desc_t *ESP_EFUSE_WR_DIS_SPI_BOOT_CRYPT_CNT[];
extern const efuse_desc_t *ESP_EFUSE_WR_DIS_SECURE_BOOT_KEY_REVOKE0[];
extern const efuse_desc_t *ESP_EFUSE_WR_DIS_SECURE_BOOT_KEY_REVOKE1[];
extern const efuse_desc_t *ESP_EFUSE_WR_DIS_SECURE_BOOT_KEY_REVOKE2[];
extern const efuse_desc_t *ESP_EFUSE_WR_DIS_KEY0_PURPOSE[];
extern const efuse_desc_t *ESP_EFUSE_WR_DIS_KEY1_PURPOSE[];
extern const efuse_desc_t *ESP_EFUSE_WR_DIS_KEY2_PURPOSE[];
extern const efuse_desc_t *ESP_EFUSE_WR_DIS_KEY3_PURPOSE[];
extern const efuse_desc_t *ESP_EFUSE_WR_DIS_KEY4_PURPOSE[];
extern const efuse_desc_t *ESP_EFUSE_WR_DIS_KEY5_PURPOSE[];
extern const efuse_desc_t *ESP_EFUSE_WR_DIS_SECURE_BOOT_EN[];
extern const efuse_desc_t *ESP_EFUSE_WR_DIS_SECURE_BOOT_AGGRESSIVE_REVOKE[];
extern const efuse_desc_t *ESP_EFUSE_WR_DIS_GROUP_3[];
extern const efuse_desc_t *ESP_EFUSE_WR_DIS_BLK1[];
extern const efuse_desc_t *ESP_EFUSE_WR_DIS_SYS_DATA_PART1[];
extern const efuse_desc_t *ESP_EFUSE_WR_DIS_USER_DATA[];
extern const efuse_desc_t *ESP_EFUSE_WR_DIS_KEY0[];
extern const efuse_desc_t *ESP_EFUSE_WR_DIS_KEY1[];
extern const efuse_desc_t *ESP_EFUSE_WR_DIS_KEY2[];
extern const efuse_desc_t *ESP_EFUSE_WR_DIS_KEY3[];
extern const efuse_desc_t *ESP_EFUSE_WR_DIS_KEY4[];
extern const efuse_desc_t *ESP_EFUSE_WR_DIS_KEY5[];
extern const efuse_desc_t *ESP_EFUSE_WR_DIS_SYS_DATA_PART2[];
extern const efuse_desc_t *ESP_EFUSE_RD_DIS_KEY0[];
extern const efuse_desc_t *ESP_EFUSE_RD_DIS_KEY1[];
extern const efuse_desc_t *ESP_EFUSE_RD_DIS_KEY2[];
extern const efuse_desc_t *ESP_EFUSE_RD_DIS_KEY3[];
extern const efuse_desc_t *ESP_EFUSE_RD_DIS_KEY4[];
extern const efuse_desc_t *ESP_EFUSE_RD_DIS_KEY5[];
extern const efuse_desc_t *ESP_EFUSE_RD_DIS_SYS_DATA_PART2[];
extern const efuse_desc_t *ESP_EFUSE_DIS_RTC_RAM_BOOT[];
extern const efuse_desc_t *ESP_EFUSE_DIS_ICACHE[];
extern const efuse_desc_t *ESP_EFUSE_DIS_USB_JTAG[];
extern const efuse_desc_t *ESP_EFUSE_DIS_DOWNLOAD_ICACHE[];
extern const efuse_desc_t *ESP_EFUSE_DIS_USB_DEVICE[];
extern const efuse_desc_t *ESP_EFUSE_DIS_FORCE_DOWNLOAD[];
extern const efuse_desc_t *ESP_EFUSE_DIS_USB[];
extern const efuse_desc_t *ESP_EFUSE_DIS_CAN[];
extern const efuse_desc_t *ESP_EFUSE_JTAG_SEL_ENABLE[];
extern const efuse_desc_t *ESP_EFUSE_SOFT_DIS_JTAG[];
extern const efuse_desc_t *ESP_EFUSE_DIS_PAD_JTAG[];
extern const efuse_desc_t *ESP_EFUSE_DIS_DOWNLOAD_MANUAL_ENCRYPT[];
extern const efuse_desc_t *ESP_EFUSE_USB_DREFH[];
extern const efuse_desc_t *ESP_EFUSE_USB_DREFL[];
extern const efuse_desc_t *ESP_EFUSE_USB_EXCHG_PINS[];
extern const efuse_desc_t *ESP_EFUSE_VDD_SPI_AS_GPIO[];
extern const efuse_desc_t *ESP_EFUSE_BTLC_GPIO_ENABLE[];
extern const efuse_desc_t *ESP_EFUSE_POWERGLITCH_EN[];
extern const efuse_desc_t *ESP_EFUSE_POWER_GLITCH_DSENSE[];
extern const efuse_desc_t *ESP_EFUSE_WDT_DELAY_SEL[];
extern const efuse_desc_t *ESP_EFUSE_SPI_BOOT_CRYPT_CNT[];
extern const efuse_desc_t *ESP_EFUSE_SECURE_BOOT_KEY_REVOKE0[];
extern const efuse_desc_t *ESP_EFUSE_SECURE_BOOT_KEY_REVOKE1[];
extern const efuse_desc_t *ESP_EFUSE_SECURE_BOOT_KEY_REVOKE2[];
extern const efuse_desc_t *ESP_EFUSE_KEY_PURPOSE_0[];
extern const efuse_desc_t *ESP_EFUSE_KEY_PURPOSE_1[];
extern const efuse_desc_t *ESP_EFUSE_KEY_PURPOSE_2[];
extern const efuse_desc_t *ESP_EFUSE_KEY_PURPOSE_3[];
extern const efuse_desc_t *ESP_EFUSE_KEY_PURPOSE_4[];
extern const efuse_desc_t *ESP_EFUSE_KEY_PURPOSE_5[];
extern const efuse_desc_t *ESP_EFUSE_SECURE_BOOT_EN[];
extern const efuse_desc_t *ESP_EFUSE_SECURE_BOOT_AGGRESSIVE_REVOKE[];
extern const efuse_desc_t *ESP_EFUSE_FLASH_TPUW[];
extern const efuse_desc_t *ESP_EFUSE_DIS_DOWNLOAD_MODE[];
extern const efuse_desc_t *ESP_EFUSE_DIS_LEGACY_SPI_BOOT[];
extern const efuse_desc_t *ESP_EFUSE_UART_PRINT_CHANNEL[];
extern const efuse_desc_t *ESP_EFUSE_FLASH_ECC_MODE[];
extern const efuse_desc_t *ESP_EFUSE_DIS_USB_DOWNLOAD_MODE[];
extern const efuse_desc_t *ESP_EFUSE_ENABLE_SECURITY_DOWNLOAD[];
extern const efuse_desc_t *ESP_EFUSE_UART_PRINT_CONTROL[];
extern const efuse_desc_t *ESP_EFUSE_PIN_POWER_SELECTION[];
extern const efuse_desc_t *ESP_EFUSE_FLASH_TYPE[];
extern const efuse_desc_t *ESP_EFUSE_FLASH_PAGE_SIZE[];
extern const efuse_desc_t *ESP_EFUSE_FLASH_ECC_EN[];
extern const efuse_desc_t *ESP_EFUSE_FORCE_SEND_RESUME[];
extern const efuse_desc_t *ESP_EFUSE_SECURE_VERSION[];
extern const efuse_desc_t *ESP_EFUSE_MAC_FACTORY[];
extern const efuse_desc_t *ESP_EFUSE_SPI_PAD_CONFIG_CLK[];
extern const efuse_desc_t *ESP_EFUSE_SPI_PAD_CONFIG_Q_D1[];
extern const efuse_desc_t *ESP_EFUSE_SPI_PAD_CONFIG_D_D0[];
extern const efuse_desc_t *ESP_EFUSE_SPI_PAD_CONFIG_CS[];
extern const efuse_desc_t *ESP_EFUSE_SPI_PAD_CONFIG_HD_D3[];
extern const efuse_desc_t *ESP_EFUSE_SPI_PAD_CONFIG_WP_D2[];
extern const efuse_desc_t *ESP_EFUSE_SPI_PAD_CONFIG_DQS[];
extern const efuse_desc_t *ESP_EFUSE_SPI_PAD_CONFIG_D4[];
extern const efuse_desc_t *ESP_EFUSE_SPI_PAD_CONFIG_D5[];
extern const efuse_desc_t *ESP_EFUSE_SPI_PAD_CONFIG_D6[];
extern const efuse_desc_t *ESP_EFUSE_SPI_PAD_CONFIG_D7[];
extern const efuse_desc_t *ESP_EFUSE_SYS_DATA_PART1[];
extern const efuse_desc_t *ESP_EFUSE_USER_DATA[];
extern const efuse_desc_t *ESP_EFUSE_KEY0[];
extern const efuse_desc_t *ESP_EFUSE_KEY1[];
extern const efuse_desc_t *ESP_EFUSE_KEY2[];
extern const efuse_desc_t *ESP_EFUSE_KEY3[];
extern const efuse_desc_t *ESP_EFUSE_KEY4[];
extern const efuse_desc_t *ESP_EFUSE_KEY5[];
extern const efuse_desc_t *ESP_EFUSE_SYS_DATA_PART2[];

#ifdef __cplusplus
}
#endif

