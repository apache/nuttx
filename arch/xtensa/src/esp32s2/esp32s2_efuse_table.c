/****************************************************************************
 * arch/xtensa/src/esp32s2/esp32s2_efuse_table.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stddef.h>
#include <nuttx/efuse/efuse.h>
#include "esp32s2_efuse.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

static const efuse_desc_t WR_DIS[] =
{
  {
    0, 32  /* Write protection */
  },
};

static const efuse_desc_t WR_DIS_RD_DIS[] =
{
  {
    0, 1  /* Write protection for RD_DIS.KEY0 RD_DIS.KEY1 RD_DIS.KEY2 RD_DIS.KEY3 RD_DIS.KEY4 RD_DIS.KEY5 RD_DIS.SYS_DATA_PART2, */
  },
};

static const efuse_desc_t WR_DIS_GROUP_1[] =
{
  {
    2, 1  /* Write protection for DIS_ICACHE DIS_DCACHE DIS_DOWNLOAD_ICACHE DIS_DOWNLOAD_DCACHE DIS_FORCE_DOWNLOAD DIS_USB DIS_CAN DIS_BOOT_REMAP SOFT_DIS_JTAG HARD_DIS.JTAG DIS_DOWNLOAD_MANUAL_ENCRYPT, */
  },
};

static const efuse_desc_t WR_DIS_GROUP_2[] =
{
  {
    3, 1  /* Write protection for VDD_SPI_XPD VDD_SPI_TIEH VDD_SPI_FORCE VDD_SPI_INIT VDD_SPI_DCAP WDT_DELAY_SEL, */
  },
};

static const efuse_desc_t WR_DIS_SPI_BOOT_CRYPT_CNT[] =
{
  {
    4, 1  /* Write protection for SPI_BOOT_CRYPT_CNT, */
  },
};

static const efuse_desc_t WR_DIS_SECURE_BOOT_KEY_REVOKE0[] =
{
  {
    5, 1  /* Write protection for SECURE_BOOT_KEY_REVOKE0, */
  },
};

static const efuse_desc_t WR_DIS_SECURE_BOOT_KEY_REVOKE1[] =
{
  {
    6, 1  /* Write protection for SECURE_BOOT_KEY_REVOKE1, */
  },
};

static const efuse_desc_t WR_DIS_SECURE_BOOT_KEY_REVOKE2[] =
{
  {
    7, 1  /* Write protection for SECURE_BOOT_KEY_REVOKE2, */
  },
};

static const efuse_desc_t WR_DIS_KEY0_PURPOSE[] =
{
  {
    8, 1  /* Write protection for key_purpose. KEY0, */
  },
};

static const efuse_desc_t WR_DIS_KEY1_PURPOSE[] =
{
  {
    9, 1  /* Write protection for key_purpose. KEY1, */
  },
};

static const efuse_desc_t WR_DIS_KEY2_PURPOSE[] =
{
  {
    10, 1  /* Write protection for key_purpose. KEY2, */
  },
};

static const efuse_desc_t WR_DIS_KEY3_PURPOSE[] =
{
  {
    11, 1  /* Write protection for key_purpose. KEY3, */
  },
};

static const efuse_desc_t WR_DIS_KEY4_PURPOSE[] =
{
  {
    12, 1  /* Write protection for key_purpose. KEY4, */
  },
};

static const efuse_desc_t WR_DIS_KEY5_PURPOSE[] =
{
  {
    13, 1  /* Write protection for key_purpose. KEY5, */
  },
};

static const efuse_desc_t WR_DIS_SECURE_BOOT_EN[] =
{
  {
    15, 1  /* Write protection for SECURE_BOOT_EN, */
  },
};

static const efuse_desc_t WR_DIS_SECURE_BOOT_AGGRESSIVE_REVOKE[] =
{
  {
    16, 1  /* Write protection for SECURE_BOOT_AGGRESSIVE_REVOKE, */
  },
};

static const efuse_desc_t WR_DIS_GROUP_3[] =
{
  {
    18, 1  /* Write protection for FLASH_TPUW DIS_DOWNLOAD_MODE DIS_LEGACY_SPI_BOOT UART_PRINT_CHANNEL DIS_USB_DOWNLOAD_MODE ENABLE_SECURITY_DOWNLOAD UART_PRINT_CONTROL PIN_POWER_SELECTION FLASH_TYPE FORCE_SEND_RESUME SECURE_VERSION, */
  },
};

static const efuse_desc_t WR_DIS_BLK1[] =
{
  {
    20, 1  /* Write protection for EFUSE_BLK1.  MAC_SPI_8M_SYS, */
  },
};

static const efuse_desc_t WR_DIS_SYS_DATA_PART1[] =
{
  {
    21, 1  /* Write protection for EFUSE_BLK2.  SYS_DATA_PART1, */
  },
};

static const efuse_desc_t WR_DIS_USER_DATA[] =
{
  {
    22, 1  /* Write protection for EFUSE_BLK3.  USER_DATA, */
  },
};

static const efuse_desc_t WR_DIS_KEY0[] =
{
  {
    23, 1  /* Write protection for EFUSE_BLK4.  KEY0, */
  },
};

static const efuse_desc_t WR_DIS_KEY1[] =
{
  {
    24, 1  /* Write protection for EFUSE_BLK5.  KEY1, */
  },
};

static const efuse_desc_t WR_DIS_KEY2[] =
{
  {
    25, 1  /* Write protection for EFUSE_BLK6.  KEY2, */
  },
};

static const efuse_desc_t WR_DIS_KEY3[] =
{
  {
    26, 1  /* Write protection for EFUSE_BLK7.  KEY3, */
  },
};

static const efuse_desc_t WR_DIS_KEY4[] =
{
  {
    27, 1  /* Write protection for EFUSE_BLK8.  KEY4, */
  },
};

static const efuse_desc_t WR_DIS_KEY5[] =
{
  {
    28, 1  /* Write protection for EFUSE_BLK9.  KEY5, */
  },
};

static const efuse_desc_t WR_DIS_SYS_DATA_PART2[] =
{
  {
    29, 1  /* Write protection for EFUSE_BLK10. SYS_DATA_PART2, */
  },
};

static const efuse_desc_t WR_DIS_USB_EXCHG_PINS[] =
{
  {
    30, 1  /* Write protection for USB_EXCHG_PINS, */
  },
};

static const efuse_desc_t RD_DIS[] =
{
  {
    32, 7  /* Read protection */
  },
};

static const efuse_desc_t RDWR_DIS_SYS_DATA_PART2_DIS_KEY0[] =
{
  {
    32, 1  /* Read protection for EFUSE_BLK4.  KEY0, */
  },
};

static const efuse_desc_t RD_DIS_KEY1[] =
{
  {
    33, 1  /* Read protection for EFUSE_BLK5.  KEY1, */
  },
};

static const efuse_desc_t RD_DIS_KEY2[] =
{
  {
    34, 1  /* Read protection for EFUSE_BLK6.  KEY2, */
  },
};

static const efuse_desc_t RD_DIS_KEY3[] =
{
  {
    35, 1  /* Read protection for EFUSE_BLK7.  KEY3, */
  },
};

static const efuse_desc_t RD_DIS_KEY4[] =
{
  {
    36, 1  /* Read protection for EFUSE_BLK8.  KEY4, */
  },
};

static const efuse_desc_t RD_DIS_KEY5[] =
{
  {
    37, 1  /* Read protection for EFUSE_BLK9.  KEY5, */
  },
};

static const efuse_desc_t RD_DIS_SYS_DATA_PART2[] =
{
  {
    38, 1  /* Read protection for EFUSE_BLK10. SYS_DATA_PART2, */
  },
};

static const efuse_desc_t DIS_RTC_RAM_BOOT[] =
{
  {
    39, 1  /* Disable boot from RTC RAM, */
  },
};

static const efuse_desc_t DIS_ICACHE[] =
{
  {
    40, 1  /* Disable Icache, */
  },
};

static const efuse_desc_t DIS_DCACHE[] =
{
  {
    41, 1  /* Disable DCACHE, */
  },
};

static const efuse_desc_t DIS_DOWNLOAD_ICACHE[] =
{
  {
    42, 1  /* Disable Icache in download mode, include boot_mode 0 1 2 3 6 7, */
  },
};

static const efuse_desc_t DIS_DOWNLOAD_DCACHE[] =
{
  {
    43, 1  /* Disable Dcache in download mode include boot_mode 0 1 2 3 6 7, */
  },
};

static const efuse_desc_t DIS_FORCE_DOWNLOAD[] =
{
  {
    44, 1  /* Disable force chip go to download mode function, */
  },
};

static const efuse_desc_t DIS_USB[] =
{
  {
    45, 1  /* Disable USB function, */
  },
};

static const efuse_desc_t DIS_CAN[] =
{
  {
    46, 1  /* Disable CAN function, */
  },
};

static const efuse_desc_t DIS_BOOT_REMAP[] =
{
  {
    47, 1  /* Disable boot from RAM. REMAP means RAM space can be mapped to ROM space. this signal will disable this function, */
  },
};

static const efuse_desc_t SOFT_DIS_JTAG[] =
{
  {
    49, 1  /* Software disable jtag jtag can be activated again by hmac module, */
  },
};

static const efuse_desc_t HARD_DIS_JTAG[] =
{
  {
    50, 1  /* Hardware disable jtag permanently disable jtag function, */
  },
};

static const efuse_desc_t DIS_DOWNLOAD_MANUAL_ENCRYPT[] =
{
  {
    51, 1  /* Disable flash encrypt function, */
  },
};

static const efuse_desc_t USB_EXCHG_PINS[] =
{
  {
    56, 1  /* Exchange D+ D- pins, */
  },
};

static const efuse_desc_t USB_EXT_PHY_ENABLE[] =
{
  {
    57, 1  /* Enable external PHY, */
  },
};

static const efuse_desc_t BLOCK0_VERSION[] =
{
  {
    59, 2  /* BLOCK0 efuse version, */
  },
};

static const efuse_desc_t VDD_SPI_XPD[] =
{
  {
    68, 1  /* VDD_SPI regulator power up, */
  },
};

static const efuse_desc_t VDD_SPI_TIEH[] =
{
  {
    69, 1  /* VDD_SPI regulator tie high to vdda, */
  },
};

static const efuse_desc_t VDD_SPI_FORCE[] =
{
  {
    70, 1  /* Force using eFuse configuration of VDD_SPI, */
  },
};

static const efuse_desc_t WDT_DELAY_SEL[] =
{
  {
    80, 2  /* Select RTC WDT time out threshold, */
  },
};

static const efuse_desc_t SPI_BOOT_CRYPT_CNT[] =
{
  {
    82, 3  /* SPI boot encrypt decrypt enable. odd number 1 enable. even number 1 disable, */
  },
};

static const efuse_desc_t SECURE_BOOT_KEY_REVOKE0[] =
{
  {
    85, 1  /* Enable revoke first secure boot key, */
  },
};

static const efuse_desc_t SECURE_BOOT_KEY_REVOKE1[] =
{
  {
    86, 1  /* Enable revoke second secure boot key, */
  },
};

static const efuse_desc_t SECURE_BOOT_KEY_REVOKE2[] =
{
  {
    87, 1  /* Enable revoke third secure boot key, */
  },
};

static const efuse_desc_t KEY_PURPOSE_0[] =
{
  {
    88, 4  /* Key0 purpose, */
  },
};

static const efuse_desc_t KEY_PURPOSE_1[] =
{
  {
    92, 4  /* Key1 purpose, */
  },
};

static const efuse_desc_t KEY_PURPOSE_2[] =
{
  {
    96, 4  /* Key2 purpose, */
  },
};

static const efuse_desc_t KEY_PURPOSE_3[] =
{
  {
    100, 4  /* Key3 purpose, */
  },
};

static const efuse_desc_t KEY_PURPOSE_4[] =
{
  {
    104, 4  /* Key4 purpose, */
  },
};

static const efuse_desc_t KEY_PURPOSE_5[] =
{
  {
    108, 4  /* Key5 purpose, */
  },
};

static const efuse_desc_t SECURE_BOOT_EN[] =
{
  {
    116, 1  /* Secure boot enable, */
  },
};

static const efuse_desc_t SECURE_BOOT_AGGRESSIVE_REVOKE[] =
{
  {
    117, 1  /* Enable aggressive secure boot revoke, */
  },
};

static const efuse_desc_t FLASH_TPUW[] =
{
  {
    124, 4  /* Flash wait time after power up. (unit is ms). When value is 15. the time is 30 ms, */
  },
};

static const efuse_desc_t DIS_DOWNLOAD_MODE[] =
{
  {
    128, 1  /* Disble download mode include boot_mode[3:0] is 0 1 2 3 6 7, */
  },
};

static const efuse_desc_t DIS_LEGACY_SPI_BOOT[] =
{
  {
    129, 1  /* Disable_Legcy_SPI_boot mode include boot_mode[3:0] is 4, */
  },
};

static const efuse_desc_t UART_PRINT_CHANNEL[] =
{
  {
    130, 1  /* 0: UART0. 1: UART1, */
  },
};

static const efuse_desc_t DIS_USB_DOWNLOAD_MODE[] =
{
  {
    132, 1  /* Disable download through USB, */
  },
};

static const efuse_desc_t ENABLE_SECURITY_DOWNLOAD[] =
{
  {
    133, 1  /* Enable security download mode, */
  },
};

static const efuse_desc_t UART_PRINT_CONTROL[] =
{
  {
    134, 2  /* b00:force print. b01:control by GPIO46 - low level print. b10:control by GPIO46 - high level print. b11:force disable print., */
  },
};

static const efuse_desc_t PIN_POWER_SELECTION[] =
{
  {
    136, 1  /* GPIO33-GPIO37 power supply selection in ROM code. 0:VDD3P3_CPU. 1:VDD_SPI., */
  },
};

static const efuse_desc_t FLASH_TYPE[] =
{
  {
    137, 1  /* Connected Flash interface type. 0: 4 data line. 1: 8 data line, */
  },
};

static const efuse_desc_t FORCE_SEND_RESUME[] =
{
  {
    138, 1  /* Force ROM code to send a resume command during SPI boot, */
  },
};

static const efuse_desc_t SECURE_VERSION[] =
{
  {
    139, 16  /* Secure version for anti-rollback, */
  },
};

static const efuse_desc_t MAC_FACTORY[] =
{
  {
    296, 8  /* Factory MAC addr [0], */
  },
  {
    288, 8  /* Factory MAC addr [1], */
  },
  {
    280, 8  /* Factory MAC addr [2], */
  },
  {
    272, 8  /* Factory MAC addr [3], */
  },
  {
    264, 8  /* Factory MAC addr [4], */
  },
  {
    256, 8  /* Factory MAC addr [5], */
  },
};

static const efuse_desc_t SPI_PAD_CONFIG_CLK[] =
{
  {
    304, 6  /* SPI_PAD_configure CLK, */
  },
};

static const efuse_desc_t SPI_PAD_CONFIG_Q_D1[] =
{
  {
    310, 6  /* SPI_PAD_configure Q(D1), */
  },
};

static const efuse_desc_t SPI_PAD_CONFIG_D_D0[] =
{
  {
    316, 6  /* SPI_PAD_configure D(D0), */
  },
};

static const efuse_desc_t SPI_PAD_CONFIG_CS[] =
{
  {
    322, 6  /* SPI_PAD_configure CS, */
  },
};

static const efuse_desc_t SPI_PAD_CONFIG_HD_D3[] =
{
  {
    328, 6  /* SPI_PAD_configure HD(D3), */
  },
};

static const efuse_desc_t SPI_PAD_CONFIG_WP_D2[] =
{
  {
    334, 6  /* SPI_PAD_configure WP(D2), */
  },
};

static const efuse_desc_t SPI_PAD_CONFIG_DQS[] =
{
  {
    340, 6  /* SPI_PAD_configure DQS, */
  },
};

static const efuse_desc_t SPI_PAD_CONFIG_D4[] =
{
  {
    346, 6  /* SPI_PAD_configure D4, */
  },
};

static const efuse_desc_t SPI_PAD_CONFIG_D5[] =
{
  {
    352, 6  /* SPI_PAD_configure D5, */
  },
};

static const efuse_desc_t SPI_PAD_CONFIG_D6[] =
{
  {
    358, 6  /* SPI_PAD_configure D6, */
  },
};

static const efuse_desc_t SPI_PAD_CONFIG_D7[] =
{
  {
    364, 6  /* SPI_PAD_configure D7, */
  },
};

static const efuse_desc_t SYS_DATA_PART1[] =
{
  {
    512, 256  /* System configuration, */
  },
};

static const efuse_desc_t USER_DATA[] =
{
  {
    768, 256  /* User data, */
  },
};

static const efuse_desc_t KEY0[] =
{
  {
    1024, 256  /* Key0 or user data, */
  },
};

static const efuse_desc_t KEY1[] =
{
  {
    1280, 256  /* Key1 or user data, */
  },
};

static const efuse_desc_t KEY2[] =
{
  {
    1536, 256  /* Key2 or user data, */
  },
};

static const efuse_desc_t KEY3[] =
{
  {
    1792, 256  /* Key3 or user data, */
  },
};

static const efuse_desc_t KEY4[] =
{
  {
    2048, 256  /* Key4 or user data, */
  },
};

static const efuse_desc_t KEY5[] =
{
  {
    2304, 256  /* Key5 or user data, */
  },
};

static const efuse_desc_t SYS_DATA_PART2[] =
{
  {
    2560, 256  /* System configuration, */
  },
};

const efuse_desc_t *ESP_EFUSE_WR_DIS_RD_DIS[] =
{
  &WR_DIS_RD_DIS[0],  /* Write protection for RD_DIS.KEY0 RD_DIS.KEY1 RD_DIS.KEY2 RD_DIS.KEY3 RD_DIS.KEY4 RD_DIS.KEY5 RD_DIS.SYS_DATA_PART2 */
  NULL
};

const efuse_desc_t *ESP_EFUSE_WR_DIS_DIS_RTC_RAM_BOOT[] =
{
  &WR_DIS_DIS_RTC_RAM_BOOT[0],  /* Write protection for DIS_RTC_RAM_BOOT */
  NULL
};

const efuse_desc_t *ESP_EFUSE_WR_DIS_GROUP_1[] =
{
  &WR_DIS_GROUP_1[0],  /* Write protection for DIS_ICACHE DIS_DCACHE DIS_DOWNLOAD_ICACHE DIS_DOWNLOAD_DCACHE DIS_FORCE_DOWNLOAD DIS_USB DIS_CAN DIS_BOOT_REMAP SOFT_DIS_JTAG HARD_DIS.JTAG DIS_DOWNLOAD_MANUAL_ENCRYPT */
  NULL
};

const efuse_desc_t *ESP_EFUSE_WR_DIS_GROUP_2[] =
{
  &WR_DIS_GROUP_2[0],  /* Write protection for VDD_SPI_XPD VDD_SPI_TIEH VDD_SPI_FORCE VDD_SPI_INIT VDD_SPI_DCAP WDT_DELAY_SEL */
  NULL
};

const efuse_desc_t *ESP_EFUSE_WR_DIS_SPI_BOOT_CRYPT_CNT[] =
{
  &WR_DIS_SPI_BOOT_CRYPT_CNT[0],  /* Write protection for SPI_BOOT_CRYPT_CNT */
  NULL
};

const efuse_desc_t *ESP_EFUSE_WR_DIS_SECURE_BOOT_KEY_REVOKE0[] =
{
  &WR_DIS_SECURE_BOOT_KEY_REVOKE0[0],  /* Write protection for SECURE_BOOT_KEY_REVOKE0 */
  NULL
};

const efuse_desc_t *ESP_EFUSE_WR_DIS_SECURE_BOOT_KEY_REVOKE1[] =
{
  &WR_DIS_SECURE_BOOT_KEY_REVOKE1[0],  /* Write protection for SECURE_BOOT_KEY_REVOKE1 */
  NULL
};

const efuse_desc_t *ESP_EFUSE_WR_DIS_SECURE_BOOT_KEY_REVOKE2[] =
{
  &WR_DIS_SECURE_BOOT_KEY_REVOKE2[0],  /* Write protection for SECURE_BOOT_KEY_REVOKE2 */
  NULL
};

const efuse_desc_t *ESP_EFUSE_WR_DIS_KEY0_PURPOSE[] =
{
  &WR_DIS_KEY0_PURPOSE[0],  /* Write protection for key_purpose. KEY0 */
  NULL
};

const efuse_desc_t *ESP_EFUSE_WR_DIS_KEY1_PURPOSE[] =
{
  &WR_DIS_KEY1_PURPOSE[0],  /* Write protection for key_purpose. KEY1 */
  NULL
};

const efuse_desc_t *ESP_EFUSE_WR_DIS_KEY2_PURPOSE[] =
{
  &WR_DIS_KEY2_PURPOSE[0],  /* Write protection for key_purpose. KEY2 */
  NULL
};

const efuse_desc_t *ESP_EFUSE_WR_DIS_KEY3_PURPOSE[] =
{
  &WR_DIS_KEY3_PURPOSE[0],  /* Write protection for key_purpose. KEY3 */
  NULL
};

const efuse_desc_t *ESP_EFUSE_WR_DIS_KEY4_PURPOSE[] =
{
  &WR_DIS_KEY4_PURPOSE[0],  /* Write protection for key_purpose. KEY4 */
  NULL
};

const efuse_desc_t *ESP_EFUSE_WR_DIS_KEY5_PURPOSE[] =
{
  &WR_DIS_KEY5_PURPOSE[0],  /* Write protection for key_purpose. KEY5 */
  NULL
};

const efuse_desc_t *ESP_EFUSE_WR_DIS_SECURE_BOOT_EN[] =
{
  &WR_DIS_SECURE_BOOT_EN[0],  /* Write protection for SECURE_BOOT_EN */
  NULL
};

const efuse_desc_t *ESP_EFUSE_WR_DIS_SECURE_BOOT_AGGRESSIVE_REVOKE[] =
{
  &WR_DIS_SECURE_BOOT_AGGRESSIVE_REVOKE[0],  /* Write protection for SECURE_BOOT_AGGRESSIVE_REVOKE */
  NULL
};

const efuse_desc_t *ESP_EFUSE_WR_DIS_GROUP_3[] =
{
  &WR_DIS_GROUP_3[0],  /* Write protection for FLASH_TPUW DIS_DOWNLOAD_MODE DIS_LEGACY_SPI_BOOT UART_PRINT_CHANNEL DIS_TINY_BASIC DIS_USB_DOWNLOAD_MODE ENABLE_SECURITY_DOWNLOAD UART_PRINT_CONTROL PIN_POWER_SELECTION FLASH_TYPE FORCE_SEND_RESUME SECURE_VERSION */
  NULL
};

const efuse_desc_t *ESP_EFUSE_WR_DIS_BLK1[] =
{
  &WR_DIS_BLK1[0],  /* Write protection for EFUSE_BLK1.  MAC_SPI_8M_SYS */
  NULL
};

const efuse_desc_t *ESP_EFUSE_WR_DIS_SYS_DATA_PART1[] =
{
  &WR_DIS_SYS_DATA_PART1[0],  /* Write protection for EFUSE_BLK2.  SYS_DATA_PART1 */
  NULL
};

const efuse_desc_t *ESP_EFUSE_WR_DIS_USER_DATA[] =
{
  &WR_DIS_USER_DATA[0],  /* Write protection for EFUSE_BLK3.  USER_DATA */
  NULL
};

const efuse_desc_t *ESP_EFUSE_WR_DIS_KEY0[] =
{
  &WR_DIS_KEY0[0],  /* Write protection for EFUSE_BLK4.  KEY0 */
  NULL
};

const efuse_desc_t *ESP_EFUSE_WR_DIS_KEY1[] =
{
  &WR_DIS_KEY1[0],  /* Write protection for EFUSE_BLK5.  KEY1 */
  NULL
};

const efuse_desc_t *ESP_EFUSE_WR_DIS_KEY2[] =
{
  &WR_DIS_KEY2[0],  /* Write protection for EFUSE_BLK6.  KEY2 */
  NULL
};

const efuse_desc_t *ESP_EFUSE_WR_DIS_KEY3[] =
{
  &WR_DIS_KEY3[0],  /* Write protection for EFUSE_BLK7.  KEY3 */
  NULL
};

const efuse_desc_t *ESP_EFUSE_WR_DIS_KEY4[] =
{
  &WR_DIS_KEY4[0],  /* Write protection for EFUSE_BLK8.  KEY4 */
  NULL
};

const efuse_desc_t *ESP_EFUSE_WR_DIS_KEY5[] =
{
  &WR_DIS_KEY5[0],  /* Write protection for EFUSE_BLK9.  KEY5 */
  NULL
};

const efuse_desc_t *ESP_EFUSE_WR_DIS_SYS_DATA_PART2[] =
{
  &WR_DIS_SYS_DATA_PART2[0],  /* Write protection for EFUSE_BLK10. SYS_DATA_PART2 */
  NULL
};

const efuse_desc_t *ESP_EFUSE_WR_DIS_USB_EXCHG_PINS[] =
{
  &WR_DIS_USB_EXCHG_PINS[0],  /* Write protection for USB_EXCHG_PINS */
  NULL
};

const efuse_desc_t *ESP_EFUSE_RD_DIS[] =
{
  &RD_DIS[0],  /* Read protection */
  NULL
};

const efuse_desc_t *ESP_EFUSE_RDWR_DIS_SYS_DATA_PART2_DIS_KEY0[] =
{
  &RDWR_DIS_SYS_DATA_PART2_DIS_KEY0[0],  /* Read protection for EFUSE_BLK4.  KEY0 */
  NULL
};

const efuse_desc_t *ESP_EFUSE_RD_DIS_KEY1[] =
{
  &RD_DIS_KEY1[0],  /* Read protection for EFUSE_BLK5.  KEY1 */
  NULL
};

const efuse_desc_t *ESP_EFUSE_RD_DIS_KEY2[] =
{
  &RD_DIS_KEY2[0],  /* Read protection for EFUSE_BLK6.  KEY2 */
  NULL
};

const efuse_desc_t *ESP_EFUSE_RD_DIS_KEY3[] =
{
  &RD_DIS_KEY3[0],  /* Read protection for EFUSE_BLK7.  KEY3 */
  NULL
};

const efuse_desc_t *ESP_EFUSE_RD_DIS_KEY4[] =
{
  &RD_DIS_KEY4[0],  /* Read protection for EFUSE_BLK8.  KEY4 */
  NULL
};

const efuse_desc_t *ESP_EFUSE_RD_DIS_KEY5[] =
{
  &RD_DIS_KEY5[0],  /* Read protection for EFUSE_BLK9.  KEY5 */
  NULL
};

const efuse_desc_t *ESP_EFUSE_RD_DIS_SYS_DATA_PART2[] =
{
  &RD_DIS_SYS_DATA_PART2[0],  /* Read protection for EFUSE_BLK10. SYS_DATA_PART2 */
  NULL
};

const efuse_desc_t *ESP_EFUSE_DIS_RTC_RAM_BOOT[] =
{
  &DIS_RTC_RAM_BOOT[0],  /* Disable boot from RTC RAM */
  NULL
};

const efuse_desc_t *ESP_EFUSE_DIS_ICACHE[] =
{
  &DIS_ICACHE[0],  /* Disable Icache */
  NULL
};

const efuse_desc_t *ESP_EFUSE_DIS_DCACHE[] =
{
  &DIS_DCACHE[0],  /* Disable DCACHE */
  NULL
};

const efuse_desc_t *ESP_EFUSE_DIS_DOWNLOAD_ICACHE[] =
{
  &DIS_DOWNLOAD_ICACHE[0],  /* Disable Icache in download mode */
  NULL
};

const efuse_desc_t *ESP_EFUSE_DIS_DOWNLOAD_DCACHE[] =
{
  &DIS_DOWNLOAD_DCACHE[0],  /* Disable Dcache in download mode include boot_mode 0 1 2 3 6 7, */
  NULL
};

const efuse_desc_t *ESP_EFUSE_DIS_FORCE_DOWNLOAD[] =
{
  &DIS_FORCE_DOWNLOAD[0],  /* Disable force chip go to download mode function */
  NULL
};

const efuse_desc_t *ESP_EFUSE_DIS_USB[] =
{
  &DIS_USB[0],  /* Disable USB function */
  NULL
};

const efuse_desc_t *ESP_EFUSE_DIS_CAN[] =
{
  &DIS_CAN[0],  /* Disable CAN function */
  NULL
};

const efuse_desc_t *ESP_EFUSE_DIS_BOOT_REMAP[] =
{
  &DIS_APP_CPU[0],  /* Disable boot from RAM. REMAP means RAM space can be mapped to ROM space. this signal will disable this function */
  NULL
};

const efuse_desc_t *ESP_EFUSE_SOFT_DIS_JTAG[] =
{
  &SOFT_DIS_JTAG[0],  /* Software disable jtag jtag can be activated again by hmac module */
  NULL
};

const efuse_desc_t *ESP_EFUSE_HARD_DIS_JTAG[] =
{
  &HARD_DIS_JTAG[0],  /* Hardware disable jtag permanently disable jtag function */
  NULL
};

const efuse_desc_t *ESP_EFUSE_DIS_DOWNLOAD_MANUAL_ENCRYPT[] =
{
  &DIS_DOWNLOAD_MANUAL_ENCRYPT[0],  /* Disable flash encrypt function */
  NULL
};

const efuse_desc_t *ESP_EFUSE_USB_EXCHG_PINS[] =
{
  &USB_EXCHG_PINS[0],  /* Exchange D+ D- pins */
  NULL
};

const efuse_desc_t *ESP_EFUSE_USB_EXT_PHY_ENABLE[] =
{
  &USB_EXT_PHY_ENABLE[0],  /* Enable external PHY */
  NULL
};

const efuse_desc_t *ESP_EFUSE_BLOCK0_VERSION[] =
{
  &BLOCK0_VERSION[0],  /* BLOCK0 efuse version */
  NULL
};

const efuse_desc_t *ESP_EFUSE_VDD_SPI_XPD[] =
{
  &VDD_SPI_XPD[0],  /* VDD_SPI regulator power up */
  NULL
};

const efuse_desc_t *ESP_EFUSE_VDD_SPI_TIEH[] =
{
  &VDD_SPI_TIEH[0],  /* VDD_SPI regulator tie high to vdda */
  NULL
};

const efuse_desc_t *ESP_EFUSE_VDD_SPI_FORCE[] =
{
  &VDD_SPI_FORCE[0],  /* Force using eFuse configuration of VDD_SPI */
  NULL
};

const efuse_desc_t *ESP_EFUSE_WDT_DELAY_SEL[] =
{
  &WDT_DELAY_SEL[0],  /* Select RTC WDT time out threshold */
  NULL
};

const efuse_desc_t *ESP_EFUSE_SPI_BOOT_CRYPT_CNT[] =
{
  &SPI_BOOT_CRYPT_CNT[0],  /* SPI boot encrypt decrypt enable. odd number 1 enable. even number 1 disable */
  NULL
};

const efuse_desc_t *ESP_EFUSE_SECURE_BOOT_KEY_REVOKE0[] =
{
  &SECURE_BOOT_KEY_REVOKE0[0],  /* Enable revoke first secure boot key */
  NULL
};

const efuse_desc_t *ESP_EFUSE_SECURE_BOOT_KEY_REVOKE1[] =
{
  &SECURE_BOOT_KEY_REVOKE1[0],  /* Enable revoke second secure boot key */
  NULL
};

const efuse_desc_t *ESP_EFUSE_SECURE_BOOT_KEY_REVOKE2[] =
{
  &SECURE_BOOT_KEY_REVOKE2[0],  /* Enable revoke third secure boot key */
  NULL
};

const efuse_desc_t *ESP_EFUSE_KEY_PURPOSE_0[] =
{
  &KEY_PURPOSE_0[0],  /* Key0 purpose */
  NULL
};

const efuse_desc_t *ESP_EFUSE_KEY_PURPOSE_1[] =
{
  &KEY_PURPOSE_1[0],  /* Key1 purpose */
  NULL
};

const efuse_desc_t *ESP_EFUSE_KEY_PURPOSE_2[] =
{
  &KEY_PURPOSE_2[0],  /* Key2 purpose */
  NULL
};

const efuse_desc_t *ESP_EFUSE_KEY_PURPOSE_3[] =
{
  &KEY_PURPOSE_3[0],  /* Key3 purpose */
  NULL
};

const efuse_desc_t *ESP_EFUSE_KEY_PURPOSE_4[] =
{
  &KEY_PURPOSE_4[0],  /* Key4 purpose */
  NULL
};

const efuse_desc_t *ESP_EFUSE_KEY_PURPOSE_5[] =
{
  &KEY_PURPOSE_5[0],  /* Key5 purpose */
  NULL
};

const efuse_desc_t *ESP_EFUSE_SECURE_BOOT_EN[] =
{
  &SECURE_BOOT_EN[0],  /* Secure boot enable */
  NULL
};

const efuse_desc_t *ESP_EFUSE_SECURE_BOOT_AGGRESSIVE_REVOKE[] =
{
  &SECURE_BOOT_AGGRESSIVE_REVOKE[0],  /* Enable aggressive secure boot revoke */
  NULL
};

const efuse_desc_t *ESP_EFUSE_FLASH_TPUW[] =
{
  &FLASH_TPUW[0],  /* Flash wait time after power up. (unit is ms). When value is 15. the time is 30 ms */
  NULL
};

const efuse_desc_t *ESP_EFUSE_DIS_DOWNLOAD_MODE[] =
{
  &DIS_DOWNLOAD_MODE[0],  /* Disble download mode include boot_mode[3:0] is 0 1 2 3 6 7 */
  NULL
};

const efuse_desc_t *ESP_EFUSE_DIS_LEGACY_SPI_BOOT[] =
{
  &DIS_LEGACY_SPI_BOOT[0],  /* Disable_Legcy_SPI_boot mode include boot_mode[3:0] is 4 */
  NULL
};

const efuse_desc_t *ESP_EFUSE_UART_PRINT_CHANNEL[] =
{
  &UART_PRINT_CHANNEL[0],  /* 0: UART0. 1: UART1 */
  NULL
};

const efuse_desc_t *ESP_EFUSE_DIS_USB_DOWNLOAD_MODE[] =
{
  &DIS_USB_DOWNLOAD_MODE[0],  /* Disable download through USB */
  NULL
};

const efuse_desc_t *ESP_EFUSE_ENABLE_SECURITY_DOWNLOAD[] =
{
  &ENABLE_SECURITY_DOWNLOAD[0],  /* Enable security download mode */
  NULL
};

const efuse_desc_t *ESP_EFUSE_UART_PRINT_CONTROL[] =
{
  &UART_PRINT_CONTROL[0],  /* b00:force print. b01:control by GPIO46 - low level print. b10:control by GPIO46 - high level print. b11:force disable print. */
  NULL
};

const efuse_desc_t *ESP_EFUSE_PIN_POWER_SELECTION[] =
{
  &PIN_POWER_SELECTION[0],  /* GPIO33-GPIO37 power supply selection in ROM code. 0:VDD3P3_CPU. 1:VDD_SPI. */
  NULL
};

const efuse_desc_t *ESP_EFUSE_FLASH_TYPE[] =
{
  &FLASH_TYPE[0],  /* Connected Flash interface type. 0: 4 data line. 1: 8 data line */
  NULL
};

const efuse_desc_t *ESP_EFUSE_FORCE_SEND_RESUME[] =
{
  &FORCE_SEND_RESUME[0],  /* Force ROM code to send a resume command during SPI boot */
  NULL
};

const efuse_desc_t *ESP_EFUSE_SECURE_VERSION[] =
{
  &SECURE_VERSION[0],  /* Secure version for anti-rollback */
  NULL
};

const efuse_desc_t *ESP_EFUSE_MAC_FACTORY[] =
{
  &MAC_FACTORY[0],  /* Factory MAC addr [0] */
  &MAC_FACTORY[1],  /* Factory MAC addr [1] */
  &MAC_FACTORY[2],  /* Factory MAC addr [2] */
  &MAC_FACTORY[3],  /* Factory MAC addr [3] */
  &MAC_FACTORY[4],  /* Factory MAC addr [4] */
  &MAC_FACTORY[5],  /* Factory MAC addr [5] */
  NULL
};

const efuse_desc_t *ESP_EFUSE_SPI_PAD_CONFIG_CLK[] =
{
  &SPI_PAD_CONFIG_CLK[0],  /* SPI_PAD_configure CLK */
  NULL
};

const efuse_desc_t *ESP_EFUSE_SPI_PAD_CONFIG_Q_D1[] =
{
  &SPI_PAD_CONFIG_Q_D1[0],  /* SPI_PAD_configure Q(D1) */
  NULL
};

const efuse_desc_t *ESP_EFUSE_SPI_PAD_CONFIG_D_D0[] =
{
  &SPI_PAD_CONFIG_D_D0[0],  /* SPI_PAD_configure D(D0) */
  NULL
};

const efuse_desc_t *ESP_EFUSE_SPI_PAD_CONFIG_CS[] =
{
  &SPI_PAD_CONFIG_CS[0],  /* SPI_PAD_configure CS */
  NULL
};

const efuse_desc_t *ESP_EFUSE_SPI_PAD_CONFIG_HD_D3[] =
{
  &SPI_PAD_CONFIG_HD_D3[0],  /* SPI_PAD_configure HD(D3) */
  NULL
};

const efuse_desc_t *ESP_EFUSE_SPI_PAD_CONFIG_WP_D2[] =
{
  &SPI_PAD_CONFIG_WP_D2[0],  /* SPI_PAD_configure WP(D2) */
  NULL
};

const efuse_desc_t *ESP_EFUSE_SPI_PAD_CONFIG_DQS[] =
{
  &SPI_PAD_CONFIG_DQS[0],  /* SPI_PAD_configure DQS */
  NULL
};

const efuse_desc_t *ESP_EFUSE_SPI_PAD_CONFIG_D4[] =
{
  &SPI_PAD_CONFIG_D4[0],  /* SPI_PAD_configure D4 */
  NULL
};

const efuse_desc_t *ESP_EFUSE_SPI_PAD_CONFIG_D5[] =
{
  &SPI_PAD_CONFIG_D5[0],  /* SPI_PAD_configure D5 */
  NULL
};

const efuse_desc_t *ESP_EFUSE_SPI_PAD_CONFIG_D6[] =
{
  &SPI_PAD_CONFIG_D6[0],  /* SPI_PAD_configure D6 */
  NULL
};

const efuse_desc_t *ESP_EFUSE_SPI_PAD_CONFIG_D7[] =
{
  &SPI_PAD_CONFIG_D7[0],  /* SPI_PAD_configure D7 */
  NULL
};

const efuse_desc_t *ESP_EFUSE_SYS_DATA_PART1[] =
{
  &SYS_DATA_PART1[0],  /* System configuration */
  NULL
};

const efuse_desc_t *ESP_EFUSE_USER_DATA[] =
{
  &USER_DATA[0],  /* User data */
  NULL
};

const efuse_desc_t *ESP_EFUSE_KEY0[] =
{
  &KEY0[0],  /* Key0 or user data */
  NULL
};

const efuse_desc_t *ESP_EFUSE_KEY1[] =
{
  &KEY1[0],  /* Key1 or user data */
  NULL
};

const efuse_desc_t *ESP_EFUSE_KEY2[] =
{
  &KEY2[0],  /* Key2 or user data */
  NULL
};

const efuse_desc_t *ESP_EFUSE_KEY3[] =
{
  &KEY3[0],  /* Key3 or user data */
  NULL
};

const efuse_desc_t *ESP_EFUSE_KEY4[] =
{
  &KEY4[0],  /* Key4 or user data */
  NULL
};

const efuse_desc_t *ESP_EFUSE_KEY5[] =
{
  &KEY5[0],  /* Key5 or user data */
  NULL
};

const efuse_desc_t *ESP_EFUSE_SYS_DATA_PART2[] =
{
  &SYS_DATA_PART2[0],  /* System configuration */
  NULL
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/
