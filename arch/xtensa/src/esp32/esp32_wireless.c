/****************************************************************************
 * arch/xtensa/src/esp32/esp32_wireless.c
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

#include <nuttx/config.h>
#include <nuttx/kmalloc.h>
#include <netinet/in.h>
#include <sys/param.h>
#include <debug.h>
#include <assert.h>

#include "xtensa.h"
#include "hardware/esp32_soc.h"
#include "hardware/esp32_dport.h"
#include "hardware/esp32_emac.h"
#include "esp32_wireless.h"
#include "esp32_partition.h"
#include "esp_phy_init.h"
#include "phy_init_data.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_ESP32_UNIVERSAL_MAC_ADDRESSES_FOUR
#  define MAC_ADDR_UNIVERSE_BT_OFFSET 2
#else
#  define MAC_ADDR_UNIVERSE_BT_OFFSET 1
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline void phy_digital_regs_store(void);
static inline void phy_digital_regs_load(void);

/****************************************************************************
 * Extern Functions declaration
 ****************************************************************************/

extern uint8_t esp_crc8(const uint8_t *p, uint32_t len);
extern void coex_bt_high_prio(void);
extern void phy_wakeup_init(void);
extern void phy_close_rf(void);
extern uint8_t phy_dig_reg_backup(bool init, uint32_t *regs);
extern int  register_chipv7_phy(const esp_phy_init_data_t *init_data,
                                esp_phy_calibration_data_t *cal_data,
                                esp_phy_calibration_mode_t cal_mode);
extern uint32_t crc32_le(uint32_t crc, uint8_t const *buf, uint32_t len);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Wi-Fi sleep private data */

static uint32_t g_phy_clk_en_cnt;

/* Reference count of enabling PHY */

static uint8_t g_phy_access_ref;

/* Memory to store PHY digital registers */

static uint32_t *g_phy_digital_regs_mem = NULL;

/* Indicate PHY is calibrated or not */

static bool g_is_phy_calibrated = false;

#ifdef CONFIG_ESP32_PHY_INIT_DATA_IN_PARTITION
static const char *phy_partion_label = "phy_init";
#endif

#ifdef CONFIG_ESP32_SUPPORT_MULTIPLE_PHY_INIT_DATA

static phy_init_data_type_t g_phy_init_data_type;

static phy_init_data_type_t g_current_apply_phy_init_data;

static char g_phy_current_country[PHY_COUNTRY_CODE_LEN];

/* Whether it is a new bin */

static bool g_multiple_phy_init_data_bin;

/* PHY init data type array */

static const char *g_phy_type[ESP_PHY_INIT_DATA_TYPE_NUMBER] =
{
  "DEFAULT", "SRRC", "FCC", "CE", "NCC", "KCC", "MIC", "IC",
  "ACMA", "ANATEL", "ISED", "WPC", "OFCA", "IFETEL", "RCM"
};

/* Country and PHY init data type map */

static phy_country_to_bin_type_t g_country_code_map_type_table[] =
{
  {"AT", ESP_PHY_INIT_DATA_TYPE_CE},
  {"AU", ESP_PHY_INIT_DATA_TYPE_ACMA},
  {"BE", ESP_PHY_INIT_DATA_TYPE_CE},
  {"BG", ESP_PHY_INIT_DATA_TYPE_CE},
  {"BR", ESP_PHY_INIT_DATA_TYPE_ANATEL},
  {"CA", ESP_PHY_INIT_DATA_TYPE_ISED},
  {"CH", ESP_PHY_INIT_DATA_TYPE_CE},
  {"CN", ESP_PHY_INIT_DATA_TYPE_SRRC},
  {"CY", ESP_PHY_INIT_DATA_TYPE_CE},
  {"CZ", ESP_PHY_INIT_DATA_TYPE_CE},
  {"DE", ESP_PHY_INIT_DATA_TYPE_CE},
  {"DK", ESP_PHY_INIT_DATA_TYPE_CE},
  {"EE", ESP_PHY_INIT_DATA_TYPE_CE},
  {"ES", ESP_PHY_INIT_DATA_TYPE_CE},
  {"FI", ESP_PHY_INIT_DATA_TYPE_CE},
  {"FR", ESP_PHY_INIT_DATA_TYPE_CE},
  {"GB", ESP_PHY_INIT_DATA_TYPE_CE},
  {"GR", ESP_PHY_INIT_DATA_TYPE_CE},
  {"HK", ESP_PHY_INIT_DATA_TYPE_OFCA},
  {"HR", ESP_PHY_INIT_DATA_TYPE_CE},
  {"HU", ESP_PHY_INIT_DATA_TYPE_CE},
  {"IE", ESP_PHY_INIT_DATA_TYPE_CE},
  {"IN", ESP_PHY_INIT_DATA_TYPE_WPC},
  {"IS", ESP_PHY_INIT_DATA_TYPE_CE},
  {"IT", ESP_PHY_INIT_DATA_TYPE_CE},
  {"JP", ESP_PHY_INIT_DATA_TYPE_MIC},
  {"KR", ESP_PHY_INIT_DATA_TYPE_KCC},
  {"LI", ESP_PHY_INIT_DATA_TYPE_CE},
  {"LT", ESP_PHY_INIT_DATA_TYPE_CE},
  {"LU", ESP_PHY_INIT_DATA_TYPE_CE},
  {"LV", ESP_PHY_INIT_DATA_TYPE_CE},
  {"MT", ESP_PHY_INIT_DATA_TYPE_CE},
  {"MX", ESP_PHY_INIT_DATA_TYPE_IFETEL},
  {"NL", ESP_PHY_INIT_DATA_TYPE_CE},
  {"NO", ESP_PHY_INIT_DATA_TYPE_CE},
  {"NZ", ESP_PHY_INIT_DATA_TYPE_RCM},
  {"PL", ESP_PHY_INIT_DATA_TYPE_CE},
  {"PT", ESP_PHY_INIT_DATA_TYPE_CE},
  {"RO", ESP_PHY_INIT_DATA_TYPE_CE},
  {"SE", ESP_PHY_INIT_DATA_TYPE_CE},
  {"SI", ESP_PHY_INIT_DATA_TYPE_CE},
  {"SK", ESP_PHY_INIT_DATA_TYPE_CE},
  {"TW", ESP_PHY_INIT_DATA_TYPE_NCC},
  {"US", ESP_PHY_INIT_DATA_TYPE_FCC},
};

#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: phy_digital_regs_store
 *
 * Description:
 *    Store  PHY digital registers.
 *
 ****************************************************************************/

static inline void phy_digital_regs_store(void)
{
  if (g_phy_digital_regs_mem == NULL)
    {
      g_phy_digital_regs_mem = (uint32_t *)
                    kmm_malloc(SOC_PHY_DIG_REGS_MEM_SIZE);
    }

  DEBUGASSERT(g_phy_digital_regs_mem != NULL);

  phy_dig_reg_backup(true, g_phy_digital_regs_mem);
}

/****************************************************************************
 * Name: phy_digital_regs_load
 *
 * Description:
 *   Load  PHY digital registers.
 *
 ****************************************************************************/

static inline void phy_digital_regs_load(void)
{
  if (g_phy_digital_regs_mem != NULL)
    {
      phy_dig_reg_backup(false, g_phy_digital_regs_mem);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Functions needed by libphy.a
 ****************************************************************************/

/****************************************************************************
 * Name: esp_dport_access_reg_read
 *
 * Description:
 *   Read register value safely in SMP
 *
 * Input Parameters:
 *   reg - Register address
 *
 * Returned Value:
 *   Register value
 *
 ****************************************************************************/

uint32_t IRAM_ATTR esp_dport_access_reg_read(uint32_t reg)
{
  return getreg32(reg);
}

/****************************************************************************
 * Name: phy_enter_critical
 *
 * Description:
 *   Enter critical state
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   CPU PS value
 *
 ****************************************************************************/

uint32_t IRAM_ATTR phy_enter_critical(void)
{
  irqstate_t flags;

  flags = enter_critical_section();

  return flags;
}

/****************************************************************************
 * Name: phy_exit_critical
 *
 * Description:
 *   Exit from critical state
 *
 * Input Parameters:
 *   level - CPU PS value
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void IRAM_ATTR phy_exit_critical(uint32_t level)
{
  leave_critical_section(level);
}

/****************************************************************************
 * Name: phy_printf
 *
 * Description:
 *   Output format string and its arguments
 *
 * Input Parameters:
 *   format - format string
 *
 * Returned Value:
 *   0
 *
 ****************************************************************************/

int phy_printf(const char *format, ...)
{
#ifdef CONFIG_DEBUG_WIRELESS_INFO
  va_list arg;

  va_start(arg, format);
  vsyslog(LOG_INFO, format, arg);
  va_end(arg);
#endif

  return 0;
}

/****************************************************************************
 * Name: esp32_phy_enable_clock
 *
 * Description:
 *   Enable PHY hardware clock
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32_phy_enable_clock(void)
{
  irqstate_t flags;

  flags = enter_critical_section();

  if (g_phy_clk_en_cnt == 0)
    {
      modifyreg32(DPORT_WIFI_CLK_EN_REG, 0,
                  DPORT_WIFI_CLK_WIFI_BT_COMMON_M);
    }

  g_phy_clk_en_cnt++;

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: esp32_phy_disable_clock
 *
 * Description:
 *   Disable PHY hardware clock
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32_phy_disable_clock(void)
{
  irqstate_t flags;

  flags = enter_critical_section();

  if (g_phy_clk_en_cnt > 0)
    {
      g_phy_clk_en_cnt--;
      if (g_phy_clk_en_cnt == 0)
        {
          modifyreg32(DPORT_WIFI_CLK_EN_REG,
                      DPORT_WIFI_CLK_WIFI_BT_COMMON_M,
                      0);
        }
    }

  leave_critical_section(flags);
}

#ifdef CONFIG_ESP32_SUPPORT_MULTIPLE_PHY_INIT_DATA

/****************************************************************************
 * Name: phy_crc_check
 *
 * Description:
 *   Check the checksum value of data
 *
 * Input Parameters:
 *   data     - Data buffer pointer
 *   length   - Data length
 *   checksum - Checksum pointer
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

static int phy_crc_check(uint8_t *data, const uint8_t *checksum,
                         size_t length)
{
  uint32_t crc_data = crc32_le(0, data, length);
  uint32_t crc_size_conversion = HTONL(crc_data);
  uint32_t tmp_crc = checksum[0] | (checksum[1] << 8) | (checksum[2] << 16) |
                     (checksum[3] << 24);
  if (crc_size_conversion != tmp_crc)
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: phy_find_bin_type_according_country
 *
 * Description:
 *   Find the PHY initialization data type according to country code
 *
 * Input Parameters:
 *   country - Country code pointer
 *
 * Returned Value:
 *   PHY initialization data type
 *
 ****************************************************************************/

static uint8_t phy_find_bin_type_according_country(const char *country)
{
  uint8_t i;
  uint8_t phy_init_data_type;
  uint8_t num = nitems(g_country_code_map_type_table);
  for (i = 0; i < num; i++)
    {
      if (memcmp(country, g_country_code_map_type_table[i].cc,
                 PHY_COUNTRY_CODE_LEN) == 0)
        {
          phy_init_data_type = g_country_code_map_type_table[i].type;
          wlinfo("Current country is %c%c, PHY init data type is %s\n",
                  g_country_code_map_type_table[i].cc[0],
                  g_country_code_map_type_table[i].cc[1],
                  g_phy_type[g_country_code_map_type_table[i].type]);
          break;
        }
    }

  if (i == num)
    {
      phy_init_data_type = ESP_PHY_INIT_DATA_TYPE_DEFAULT;
      wlerr("Use the default certification code beacuse %c%c doesn't "
            "have a certificate\n", country[0], country[1]);
    }

  return phy_init_data_type;
}

/****************************************************************************
 * Name: phy_find_bin_data_according_type
 *
 * Description:
 *   Find the PHY initialization data according to PHY init data type
 *
 * Input Parameters:
 *   output_data    - Output data buffer pointer
 *   control_info   - PHY init data control infomation
 *   input_data     - Input data buffer pointer
 *   init_data_type - PHY init data type
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

static int phy_find_bin_data_according_type(uint8_t *output_data,
        const phy_control_info_data_t *control_info,
        const esp_phy_init_data_t *input_data,
        phy_init_data_type_t init_data_type)
{
  int i;
  for (i = 0; i < control_info->number; i++)
    {
      if (init_data_type == *((uint8_t *)(input_data + i)
                              + PHY_INIT_DATA_TYPE_OFFSET))
        {
          memcpy(output_data + sizeof(phy_init_magic_pre),
                 (input_data + i), sizeof(esp_phy_init_data_t));
          break;
        }
    }

  if (i == control_info->number)
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: phy_get_multiple_init_data
 *
 * Description:
 *   Get multiple PHY init data according to PHY init data type
 *
 * Input Parameters:
 *   data           - Data buffer pointer
 *   length         - Data length
 *   init_data_type - PHY init data type
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

static int phy_get_multiple_init_data(uint8_t *data, size_t length,
                                      phy_init_data_type_t init_data_type)
{
  phy_control_info_data_t *control_info = (phy_control_info_data_t *)
                                kmm_malloc(sizeof(phy_control_info_data_t));
  if (control_info == NULL)
    {
      wlerr("ERROR: Failed to allocate memory for\
            PHY init data control info\n");
      return -ENOMEM;
    }

  int ret = esp32_partition_read(phy_partion_label, length,
              control_info, sizeof(phy_control_info_data_t));
  if (ret != OK)
    {
      kmm_free(control_info);
      wlerr("ERROR: Failed to read PHY control info data partition\n");
      return ret;
    }

  if ((control_info->check_algorithm) == PHY_CRC_ALGORITHM)
    {
      ret = phy_crc_check(control_info->multiple_bin_checksum,
      control_info->control_info_checksum, sizeof(phy_control_info_data_t) -
                                sizeof(control_info->control_info_checksum));
      if (ret != OK)
        {
          kmm_free(control_info);
          wlerr("ERROR: PHY init data control info check error\n");
          return ret;
        }
    }
  else
    {
      kmm_free(control_info);
      wlerr("ERROR: Check algorithm not CRC, PHY init data update failed\n");
      return ERROR;
    }

  uint8_t *init_data_multiple = (uint8_t *)
        kmm_malloc(sizeof(esp_phy_init_data_t) * control_info->number);
  if (init_data_multiple == NULL)
    {
      kmm_free(control_info);
      wlerr("ERROR: Failed to allocate memory for PHY init data\n");
      return -ENOMEM;
    }

  ret = esp32_partition_read(phy_partion_label, length +
          sizeof(phy_control_info_data_t), init_data_multiple,
          sizeof(esp_phy_init_data_t) * control_info->number);
  if (ret != OK)
    {
      kmm_free(init_data_multiple);
      kmm_free(control_info);
      wlerr("ERROR: Failed to read PHY init data multiple bin partition\n");
      return ret;
    }

  if ((control_info->check_algorithm) == PHY_CRC_ALGORITHM)
    {
      ret = phy_crc_check(init_data_multiple,
              control_info->multiple_bin_checksum,
              sizeof(esp_phy_init_data_t) * control_info->number);
      if (ret != OK)
        {
          kmm_free(init_data_multiple);
          kmm_free(control_info);
          wlerr("ERROR: PHY init data multiple bin check error\n");
          return ret;
        }
    }
  else
    {
      kmm_free(init_data_multiple);
      kmm_free(control_info);
      wlerr("ERROR: Check algorithm not CRC, PHY init data update failed\n");
      return ERROR;
    }

  ret = phy_find_bin_data_according_type(data, control_info,
          (const esp_phy_init_data_t *)init_data_multiple, init_data_type);
  if (ret != OK)
    {
      wlerr("ERROR: %s has not been certified, use DEFAULT PHY init data\n",
            g_phy_type[init_data_type]);
      g_phy_init_data_type = ESP_PHY_INIT_DATA_TYPE_DEFAULT;
    }
  else
    {
      g_phy_init_data_type = init_data_type;
    }

  kmm_free(init_data_multiple);
  kmm_free(control_info);
  return OK;
}

/****************************************************************************
 * Name: phy_update_init_data
 *
 * Description:
 *   Update PHY init data according to PHY init data type
 *
 * Input Parameters:
 *   init_data_type - PHY init data type
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

static int phy_update_init_data(phy_init_data_type_t init_data_type)
{
  int ret;
  size_t length = sizeof(phy_init_magic_pre) +
      sizeof(esp_phy_init_data_t) + sizeof(phy_init_magic_post);
  uint8_t *init_data_store = (uint8_t *)kmm_malloc(length);
  if (init_data_store == NULL)
    {
      wlerr("ERROR: Failed to allocate memory for updated country code "
            "PHY init data\n");
      return -ENOMEM;
    }

  ret = esp32_partition_read(phy_partion_label, 0, init_data_store, length);
  if (ret != OK)
    {
      kmm_free(init_data_store);
      wlerr("ERROR: Failed to read updated country code PHY data\n");
      return ret;
    }

  if (memcmp(init_data_store, PHY_INIT_MAGIC,
             sizeof(phy_init_magic_pre)) != 0 ||
      memcmp(init_data_store + length - sizeof(phy_init_magic_post),
             PHY_INIT_MAGIC, sizeof(phy_init_magic_post)) != 0)
    {
      kmm_free(init_data_store);
      wlerr("ERROR: Failed to validate updated country code PHY data\n");
      return ERROR;
    }

  /* find init data bin according init data type */

  if (init_data_type != ESP_PHY_INIT_DATA_TYPE_DEFAULT)
    {
      ret = phy_get_multiple_init_data(init_data_store, length,
                                       init_data_type);
      if (ret != OK)
        {
          kmm_free(init_data_store);
#ifdef CONFIG_ESP32_PHY_INIT_DATA_ERROR
          abort();
#else
          return ret;
#endif
        }
    }
  else
    {
      g_phy_init_data_type = ESP_PHY_INIT_DATA_TYPE_DEFAULT;
    }

  if (g_current_apply_phy_init_data != g_phy_init_data_type)
    {
      ret = esp_phy_apply_phy_init_data(init_data_store +
                              sizeof(phy_init_magic_pre));
      if (ret != OK)
        {
          wlerr("ERROR: PHY init data failed to load\n");
          kmm_free(init_data_store);
          return ret;
        }

      wlinfo("PHY init data type updated from %s to %s\n",
             g_phy_type[g_current_apply_phy_init_data],
             g_phy_type[g_phy_init_data_type]);
      g_current_apply_phy_init_data = g_phy_init_data_type;
    }

  kmm_free(init_data_store);
  return OK;
}

#endif

#ifdef CONFIG_ESP32_PHY_INIT_DATA_IN_PARTITION

/****************************************************************************
 * Name: esp_phy_get_init_data
 *
 * Description:
 *   Get PHY init data
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Pointer to PHY init data structure
 *
 ****************************************************************************/

const esp_phy_init_data_t *esp_phy_get_init_data(void)
{
  int ret;
  size_t length = sizeof(phy_init_magic_pre) +
          sizeof(esp_phy_init_data_t) + sizeof(phy_init_magic_post);
  uint8_t *init_data_store = (uint8_t *)kmm_malloc(length);
  if (init_data_store == NULL)
    {
      wlerr("ERROR: Failed to allocate memory for PHY init data\n");
      return NULL;
    }

  ret = esp32_partition_read(phy_partion_label, 0, init_data_store, length);
  if (ret != OK)
    {
      wlerr("ERROR: Failed to get read data from MTD\n");
      kmm_free(init_data_store);
      return NULL;
    }

  if (memcmp(init_data_store, PHY_INIT_MAGIC, sizeof(phy_init_magic_pre))
      != 0 || memcmp(init_data_store + length - sizeof(phy_init_magic_post),
              PHY_INIT_MAGIC, sizeof(phy_init_magic_post)) != 0)
    {
#ifdef CONFIG_ESP32_PHY_DEFAULT_INIT_IF_INVALID
      wlerr("ERROR: Failed to validate PHY data partition, restoring "
            "default data into flash...");
      memcpy(init_data_store, PHY_INIT_MAGIC, sizeof(phy_init_magic_pre));
      memcpy(init_data_store + sizeof(phy_init_magic_pre),
             &phy_init_data, sizeof(phy_init_data));
      memcpy(init_data_store + sizeof(phy_init_magic_pre) +
             sizeof(phy_init_data), PHY_INIT_MAGIC,
             sizeof(phy_init_magic_post));
      DEBUGASSERT(memcmp(init_data_store, PHY_INIT_MAGIC,
                  sizeof(phy_init_magic_pre)) == 0);
      DEBUGASSERT(memcmp(init_data_store + length -
                  sizeof(phy_init_magic_post), PHY_INIT_MAGIC,
                  sizeof(phy_init_magic_post)) == 0);

      /* write default data */

      ret = esp32_partition_write(phy_partion_label, 0, init_data_store,
                                  length);
      if (ret != OK)
        {
          wlerr("ERROR: Failed to write default PHY data partition\n");
          kmm_free(init_data_store);
          return NULL;
        }
#else /* CONFIG_ESP32_PHY_DEFAULT_INIT_IF_INVALID */ 
      wlerr("ERROR: Failed to validate PHY data partition\n");
      kmm_free(init_data_store);
      return NULL;
#endif
    }

#ifdef CONFIG_ESP32_SUPPORT_MULTIPLE_PHY_INIT_DATA
  if (*(init_data_store + (sizeof(phy_init_magic_pre) +
      PHY_SUPPORT_MULTIPLE_BIN_OFFSET)))
    {
      g_multiple_phy_init_data_bin = true;
      wlinfo("Support multiple PHY init data bins\n");
    }
  else
    {
      wlinfo("Does not support multiple PHY init data bins\n");
    }
#endif

  return (const esp_phy_init_data_t *)
         (init_data_store + sizeof(phy_init_magic_pre));
}

/****************************************************************************
 * Name: esp_phy_release_init_data
 *
 * Description:
 *   Release PHY init data
 *
 * Input Parameters:
 *   init_data - Pointer to PHY init data structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp_phy_release_init_data(const esp_phy_init_data_t *init_data)
{
  kmm_free((uint8_t *)init_data - sizeof(phy_init_magic_pre));
}

#else /* CONFIG_ESP32_PHY_INIT_DATA_IN_PARTITION */ 

/****************************************************************************
 * Name: esp_phy_get_init_data
 *
 * Description:
 *   Get PHY init data
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Pointer to PHY init data structure
 *
 ****************************************************************************/

const esp_phy_init_data_t *esp_phy_get_init_data(void)
{
  wlinfo("Loading PHY init data from application binary\n");
  return &phy_init_data;
}

/****************************************************************************
 * Name: esp_phy_release_init_data
 *
 * Description:
 *   Release PHY init data
 *
 * Input Parameters:
 *   init_data - Pointer to PHY init data structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp_phy_release_init_data(const esp_phy_init_data_t *init_data)
{
}
#endif

/****************************************************************************
 * Name: esp_read_mac
 *
 * Description:
 *   Read MAC address from efuse
 *
 * Input Parameters:
 *   mac  - MAC address buffer pointer
 *   type - MAC address type
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int32_t esp_read_mac(uint8_t *mac, esp_mac_type_t type)
{
  uint32_t regval[2];
  uint8_t *data = (uint8_t *)regval;
  uint8_t crc;
  int i;

  if (type > ESP_MAC_BT)
    {
      wlerr("Input type is error=%d\n", type);
      return -1;
    }

  regval[0] = getreg32(MAC_ADDR0_REG);
  regval[1] = getreg32(MAC_ADDR1_REG);

  crc = data[6];
  for (i = 0; i < MAC_LEN; i++)
    {
      mac[i] = data[5 - i];
    }

  if (crc != esp_crc8(mac, MAC_LEN))
    {
      wlerr("Failed to check MAC address CRC\n");
      return -1;
    }

  if (type == ESP_MAC_WIFI_SOFTAP)
    {
#ifdef CONFIG_ESP_MAC_ADDR_UNIVERSE_WIFI_AP
      mac[5] += 1;
#else
      uint8_t tmp = mac[0];
      for (i = 0; i < 64; i++)
        {
          mac[0] = tmp | 0x02;
          mac[0] ^= i << 2;

          if (mac[0] != tmp)
            {
              break;
            }
        }

      if (i >= 64)
        {
          wlerr("Failed to generate SoftAP MAC\n");
          return -1;
        }
#endif
    }

  if (type == ESP_MAC_BT)
    {
#ifdef CONFIG_ESP_MAC_ADDR_UNIVERSE_BT
      mac[5] += MAC_ADDR_UNIVERSE_BT_OFFSET;
#endif
    }

  return 0;
}

/****************************************************************************
 * Name: esp32_phy_update_country_info
 *
 * Description:
 *   Update PHY init data according to country code
 *
 * Input Parameters:
 *   country - PHY init data type
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

int esp32_phy_update_country_info(const char *country)
{
#ifdef CONFIG_ESP32_SUPPORT_MULTIPLE_PHY_INIT_DATA
  uint8_t phy_init_data_type_map = 0;
  if (memcmp(country, g_phy_current_country, sizeof(g_phy_current_country))
      == 0)
    {
      return OK;
    }

  memcpy(g_phy_current_country, country, sizeof(g_phy_current_country));
  if (!g_multiple_phy_init_data_bin)
    {
      wlerr("ERROR: Does not support multiple PHY init data bins\n");
      return ERROR;
    }

  phy_init_data_type_map = phy_find_bin_type_according_country(country);
  if (phy_init_data_type_map == g_phy_init_data_type)
    {
      return OK;
    }

  int ret =  phy_update_init_data(phy_init_data_type_map);
  if (ret != OK)
    {
      wlerr("ERROR: Failed to update PHY init data\n");
      return ret;
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: esp32_phy_disable
 *
 * Description:
 *   Deinitialize PHY hardware
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32_phy_disable(void)
{
  irqstate_t flags;
  flags = enter_critical_section();

  g_phy_access_ref--;

  if (g_phy_access_ref == 0)
    {
      /* Disable PHY and RF. */

      phy_close_rf();

      /* Disable Wi-Fi/BT common peripheral clock.
       * Do not disable clock for hardware RNG.
       */

      esp32_phy_disable_clock();
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: esp32_phy_enable
 *
 * Description:
 *   Initialize PHY hardware
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32_phy_enable(void)
{
  static bool debug = false;
  irqstate_t flags;
  esp_phy_calibration_data_t *cal_data;
  if (debug == false)
    {
      char *phy_version = get_phy_version_str();
      wlinfo("phy_version %s\n", phy_version);
      debug = true;
    }

  cal_data = kmm_zalloc(sizeof(esp_phy_calibration_data_t));
  if (!cal_data)
    {
      wlerr("ERROR: Failed to allocate PHY calibration data buffer.");
      abort();
    }

  flags = enter_critical_section();

  if (g_phy_access_ref == 0)
    {
      esp32_phy_enable_clock();
      if (g_is_phy_calibrated == false)
        {
          const esp_phy_init_data_t *init_data = esp_phy_get_init_data();
          if (init_data == NULL)
            {
              wlerr("ERROR: Failed to obtain PHY init data");
              abort();
            }

          register_chipv7_phy(init_data, cal_data, PHY_RF_CAL_FULL);
          esp_phy_release_init_data(init_data);
          g_is_phy_calibrated = true;
        }
      else
        {
          phy_wakeup_init();
          phy_digital_regs_load();
        }

#ifdef CONFIG_ESP32_BLE
      coex_bt_high_prio();
#endif
    }

  g_phy_access_ref++;
  leave_critical_section(flags);
  kmm_free(cal_data);
}
