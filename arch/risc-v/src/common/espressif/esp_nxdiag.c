/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_nxdiag.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this args for additional information regarding copyright ownership.  The
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

#include <stdio.h>
#include <nuttx/drivers/drivers.h>
#include <syslog.h>

#include "bootloader_flash.h"
#include "esp_mac.h"
#include "spi_flash_chip_generic.h"
#include "espressif/esp_nxdiag.h"
#include "hal/efuse_ll.h"
#include "esp_secure_boot.h"
#include "esp_flash_encrypt.h"
#include "bootloader_flash_priv.h"
#include "esp_rom_spiflash.h"
#include "soc/spi_mem_reg.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t esp_nxdiag_read(struct file *filep,
                               char *buffer,
                               size_t buflen);
void esp_nxdiag_read_flash_id(int *mfg_id_out, int *flash_id_out);
void esp_nxdiag_read_security_info(bool *crypt_cnt, bool *secure_boot,
                                   bool *flash_crpyt_en, int *chip_id,
                                   int *api_version, uint32_t *key_purposes);
int esp_nxdiag_read_flash_status(uint32_t *status);
int esp_nxdiag_read_flash_status(uint32_t *status);
int esp_nxdiag_read_mac(uint8_t *mac);
static ssize_t esp_nxdiag_read(struct file *filep,
                               char *buffer,
                               size_t buflen);
static int esp_nxdiag_register(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_nxdiagops =
{
  .read = esp_nxdiag_read,       /* read */
};

/****************************************************************************
 * Private functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_nxdiag_read_flash_id
 *
 * Description:
 *   Read flash id value.
 *
 * Input Parameters:
 *   mfg_id_out   - Manufacturer id value to return.
 *   flash_id_out - Flash id value to return.
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

void esp_nxdiag_read_flash_id(int *mfg_id_out, int *flash_id_out)
{
  uint32_t value = bootloader_read_flash_id();
  int mfg_id = (value >> 16) & 0xff;
  int flash_id = value & 0xffff;
  *mfg_id_out = mfg_id;
  *flash_id_out = flash_id;
}

/****************************************************************************
 * Name: esp_nxdiag_read_security_info
 *
 * Description:
 *   Read security information.
 *
 * Input Parameters:
 *   crypt_cnt       - Crypt count value to return.
 *   secure_boot     - Secure boot value to return.
 *   flash_crpyt_en  - Encryption enabled value to return.
 *   chip_id         - Chip id value to return.
 *   api_version     - API version value to return.
 *   key_purposes    - Key values for efuses
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_nxdiag_read_security_info(bool *crypt_cnt, bool *secure_boot,
                                   bool *flash_crpyt_en, int *chip_id,
                                   int *api_version, uint32_t *key_purposes)
{
  extern int _rom_chip_id;
  extern int _rom_eco_version;

  for (int i = 0; i < 7; i++)
    {
      key_purposes[i] = ets_efuse_get_key_purpose(ETS_EFUSE_BLOCK_KEY0 + i);
    }

  *secure_boot = esp_secure_boot_enabled();
  *crypt_cnt = efuse_ll_get_flash_crypt_cnt();
  *flash_crpyt_en = esp_flash_encryption_enabled();
  *chip_id = _rom_chip_id;
  *api_version = _rom_eco_version;
}

/****************************************************************************
 * Name: esp_nxdiag_read_flash_status
 *
 * Description:
 *   Read flash status value.
 *
 * Input Parameters:
 *   status - Flash status variable to fill
 *
 * Returned Value:
 *   OK on success; ERROR on failure.
 *
 ****************************************************************************/

int esp_nxdiag_read_flash_status(uint32_t *status)
{
  int ret = OK;
  ret = esp_rom_spiflash_read_status(&g_rom_flashchip, status);
  if (ret != OK)
    {
      syslog(LOG_ERR, "Failed to read flash status");
      return ERROR;
    }

  ret = esp_rom_spiflash_read_statushigh(&g_rom_flashchip, status);
  if (ret != OK)
    {
      syslog(LOG_ERR, "Failed to read flash status");
      return ERROR;
    }

  return ret;
}

/****************************************************************************
 * Name: esp_nxdiag_read_mac
 *
 * Description:
 *   Read MAC adress.
 *
 * Input Parameters:
 *   mac - Mac address to return.
 *
 * Returned Value:
 *   OK on success; ERROR on failure.
 *
 ****************************************************************************/

int esp_nxdiag_read_mac(uint8_t *mac)
{
  int ret = esp_read_mac(mac, ESP_MAC_WIFI_STA);
  return ret;
}

/****************************************************************************
 * Name: esp_nxdiag_read
 *
 * Description:
 *   Print diagnostic information.
 *
 * Input Parameters:
 *   filep         - Pointer to a file structure instance.
 *   buffer        - Pointer to buffer to fill with random numbers.
 *   buflen        - Length of buffer in bytes.
 *
 * Returned Value:
 *   OK on success; ERROR on failure.
 *
 ****************************************************************************/

static ssize_t esp_nxdiag_read(struct file *filep,
                               char *buffer,
                               size_t buflen)
{
  int ret;
  int mfg_id;
  int flash_id;
  int chip_id;
  int api_version;
  int i;
  int tmp_arr_size = 128;
  char tmp[128];
  uint32_t flash_status;
  uint8_t mac[6];
  uint32_t key_purposes[7];
  bool crypt_cnt;
  bool secure_boot;
  bool flash_crpyt_en;

  for (i = 0; i < buflen; i++)
    {
      buffer[i] = 0;
    }

  esp_nxdiag_read_flash_id(&mfg_id, &flash_id);
  snprintf(tmp, tmp_arr_size,
           "Flash ID:\n\t Manufacturer: %d\n\t Device: %d\n\n",
           mfg_id, flash_id);
  strcat(buffer, tmp);

  ret = esp_nxdiag_read_mac(mac);
  if (ret != OK)
    {
      printf("Failed to fetch MAC adress\n\n");
    }
  else
    {
      snprintf(tmp, tmp_arr_size,
               "MAC address: %02x:%02x:%02x:%02x:%02x:%02x\n\n",
               mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
      strcat(buffer, tmp);
    }

  ret = esp_nxdiag_read_flash_status(&flash_status);
  if (ret != OK)
    {
      printf("Failed to read flash status\n\n");
    }
  else
    {
      snprintf(tmp, tmp_arr_size, "Flash status: 0x%lx\n\n",
               (long unsigned int)flash_status);
      strcat(buffer, tmp);
    }

  esp_nxdiag_read_security_info(&crypt_cnt, &secure_boot,
                                &flash_crpyt_en, &chip_id,
                                &api_version, key_purposes);
  snprintf(tmp, tmp_arr_size, "Security information:\n\
           Key Purposes:(%ld, %ld, %ld, %ld, %ld, %ld, %ld)\n",
           key_purposes[0], key_purposes[1], key_purposes[2],
           key_purposes[3], key_purposes[4], key_purposes[5],
           key_purposes[6]);
  strcat(buffer, tmp);

  snprintf(tmp, tmp_arr_size, "\t Chip ID: %d\n", chip_id);
  strcat(buffer, tmp);

  snprintf(tmp, tmp_arr_size, "\t API Version: %ld\n",
           (long int)api_version);
  strcat(buffer, tmp);

  snprintf(tmp, tmp_arr_size, "\t Secure Boot: %s\n", (secure_boot == 1) ?
           "Enabled" : "Disabled");
  strcat(buffer, tmp);

  snprintf(tmp, tmp_arr_size, "\t Flash Encryption: %s\n",
           (flash_crpyt_en == 1) ? "Enabled" : "Disabled");
  strcat(buffer, tmp);

  snprintf(tmp, tmp_arr_size,
           "\t SPI Boot Crypt Count (SPI_BOOT_CRYPT_CNT): %x\n\n",
           crypt_cnt);
  strcat(buffer, tmp);

  return strlen(buffer);
}

/****************************************************************************
 * Name: devrandom_register
 *
 * Description:
 *   Initialize and register the /dev/nxdiag driver.
 *
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Returns OK on success; a negated errno value on failure
 *
 ****************************************************************************/

static int esp_nxdiag_register(void)
{
  return register_driver("/dev/nxdiag", &g_nxdiagops, 0444, NULL);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_nxdiag_initialize
 *
 * Description:
 *   This function initializes the internal temperature sensor with the
 *   provided configuration.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Returns OK on success; a negated errno value on failure
 *
 ****************************************************************************/

int esp_nxdiag_initialize(void)
{
  return esp_nxdiag_register();
}
