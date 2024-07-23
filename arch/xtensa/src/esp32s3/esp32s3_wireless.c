/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_wireless.c
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
#include <nuttx/mqueue.h>

#include <debug.h>
#include <assert.h>
#include <netinet/in.h>
#include <sys/param.h>

#include "xtensa.h"
#include "hardware/esp32s3_efuse.h"
#include "hardware/esp32s3_rtccntl.h"
#include "hardware/esp32s3_soc.h"
#include "hardware/esp32s3_syscon.h"
#include "hardware/esp32s3_system.h"
#include "esp32s3_irq.h"
#include "esp32s3_partition.h"

#include "esp_private/phy.h"
#ifdef CONFIG_ESP32S3_WIFI
#  include "esp_private/wifi.h"
#  include "esp_wpa.h"
#endif
#include "esp_coexist_internal.h"
#include "periph_ctrl.h"
#include "esp_phy_init.h"
#include "phy_init_data.h"

#include "esp32s3_wireless.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Software Interrupt */

#define SWI_IRQ       ESP32S3_IRQ_INT_FROM_CPU2
#define SWI_PERIPH    ESP32S3_PERIPH_INT_FROM_CPU2

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Wireless Private Data */

struct esp_wireless_priv_s
{
  volatile int ref;               /* Reference count */

  int cpuint;                     /* CPU interrupt assigned to SWI */

  struct list_node sc_list;       /* Semaphore cache list */
  struct list_node qc_list;       /* Queue cache list */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline void phy_digital_regs_store(void);
static inline void phy_digital_regs_load(void);
static int esp_swi_irq(int irq, void *context, void *arg);

/****************************************************************************
 * Extern Functions declaration
 ****************************************************************************/

extern uint8_t esp_crc8(const uint8_t *p, uint32_t len);
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

/* Private data of the wireless common interface */

static struct esp_wireless_priv_s g_esp_wireless_priv;

#ifdef CONFIG_ESP32S3_PHY_INIT_DATA_IN_PARTITION
static const char *phy_partion_label = "phy_init";
#endif

#ifdef CONFIG_ESP32S3_SUPPORT_MULTIPLE_PHY_INIT_DATA

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
  {"01", ESP_PHY_INIT_DATA_TYPE_DEFAULT},
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
 * Name: esp_swi_irq
 *
 * Description:
 *   Wireless software interrupt callback function.
 *
 * Parameters:
 *   cpuint  - CPU interrupt index
 *   context - Context data from the ISR
 *   arg     - NULL
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

static int esp_swi_irq(int irq, void *context, void *arg)
{
  int i;
  int ret;
  struct esp_semcache_s *sc;
  struct esp_semcache_s *sc_tmp;
  struct esp_queuecache_s *qc;
  struct esp_queuecache_s *qc_tmp;
  struct esp_wireless_priv_s *priv = &g_esp_wireless_priv;

  modifyreg32(SYSTEM_CPU_INTR_FROM_CPU_2_REG, SYSTEM_CPU_INTR_FROM_CPU_2, 0);

  list_for_every_entry_safe(&priv->sc_list, sc, sc_tmp,
                            struct esp_semcache_s, node)
    {
      for (i = 0; i < sc->count; i++)
        {
          ret = nxsem_post(sc->sem);
          if (ret < 0)
            {
              wlerr("ERROR: Failed to post sem ret=%d\n", ret);
            }
        }

      sc->count = 0;
      list_delete(&sc->node);
    }

  list_for_every_entry_safe(&priv->qc_list, qc, qc_tmp,
                            struct esp_queuecache_s, node)
    {
      ret = file_mq_send(qc->mq_ptr, (const char *)qc->buffer, qc->size, 0);
      if (ret < 0)
        {
          wlerr("ERROR: Failed to send queue ret=%d\n", ret);
        }

      list_delete(&qc->node);
    }

  return OK;
}

#ifdef CONFIG_ESP32S3_WIFI

/****************************************************************************
 * Name: esp_wifi_set_log_level
 *
 * Description:
 *   Sets the log level for the ESP32 WiFi module based on preprocessor
 *   definitions. The log level can be verbose, warning, or error.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp_wifi_set_log_level(void)
{
  wifi_log_level_t wifi_log_level = WIFI_LOG_NONE;

  /* set WiFi log level */

#if defined(CONFIG_DEBUG_WIRELESS_INFO)
  wifi_log_level = WIFI_LOG_VERBOSE;
#elif defined(CONFIG_DEBUG_WIRELESS_WARN)
  wifi_log_level = WIFI_LOG_WARNING;
#elif defined(CONFIG_LOG_MAXIMUM_LEVEL)
  wifi_log_level = WIFI_LOG_ERROR;
#endif

  esp_wifi_internal_set_log_level(wifi_log_level);
}
#endif /* CONFIG_ESP32S3_WIFI */

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

#ifdef CONFIG_ESP32S3_SUPPORT_MULTIPLE_PHY_INIT_DATA

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
                 sizeof(g_phy_current_country)) == 0)
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

  int ret = esp32s3_partition_read(phy_partion_label,
                                   length,
                                   control_info,
                                   sizeof(phy_control_info_data_t));
  if (ret != OK)
    {
      kmm_free(control_info);
      wlerr("ERROR: Failed to read PHY control info data partition\n");
      return ret;
    }

  if ((control_info->check_algorithm) == PHY_CRC_ALGORITHM)
    {
      ret = phy_crc_check(control_info->multiple_bin_checksum,
                          control_info->control_info_checksum,
                          sizeof(phy_control_info_data_t) -
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

  ret = esp32s3_partition_read(phy_partion_label, length +
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
  uint8_t *init_data_store = kmm_malloc(length);
  if (init_data_store == NULL)
    {
      wlerr("ERROR: Failed to allocate memory for updated country code "
            "PHY init data\n");
      return -ENOMEM;
    }

  ret = esp32s3_partition_read(phy_partion_label, 0, init_data_store,
                               length);
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
#ifdef CONFIG_ESP32S3_PHY_INIT_DATA_ERROR
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

#ifdef CONFIG_ESP32S3_PHY_INIT_DATA_IN_PARTITION

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
  uint8_t *init_data_store = kmm_malloc(length);
  if (init_data_store == NULL)
    {
      wlerr("ERROR: Failed to allocate memory for PHY init data\n");
      return NULL;
    }

  ret = esp32s3_partition_read(phy_partion_label, 0, init_data_store,
                               length);
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
#ifdef CONFIG_ESP32S3_PHY_DEFAULT_INIT_IF_INVALID
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

      ret = esp32s3_partition_write(phy_partion_label, 0, init_data_store,
                                    length);
      if (ret != OK)
        {
          wlerr("ERROR: Failed to write default PHY data partition\n");
          kmm_free(init_data_store);
          return NULL;
        }
#else /* CONFIG_ESP32S3_PHY_DEFAULT_INIT_IF_INVALID */
      wlerr("ERROR: Failed to validate PHY data partition\n");
      kmm_free(init_data_store);
      return NULL;
#endif
    }

#ifdef CONFIG_ESP32S3_SUPPORT_MULTIPLE_PHY_INIT_DATA
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

  wlinfo("PHY data partition validated\n");
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

#else /* CONFIG_ESP32S3_PHY_INIT_DATA_IN_PARTITION */

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
 * Name: esp32s3_phy_update_country_info
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

int esp32s3_phy_update_country_info(const char *country)
{
#ifdef CONFIG_ESP32S3_SUPPORT_MULTIPLE_PHY_INIT_DATA
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
 * Name: esp_timer_create
 *
 * Description:
 *   Create timer with given arguments
 *
 * Input Parameters:
 *   create_args - Timer arguments data pointer
 *   out_handle  - Timer handle pointer
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int32_t esp_timer_create(const esp_timer_create_args_t *create_args,
                         esp_timer_handle_t *out_handle)
{
  int ret;
  struct rt_timer_args_s rt_timer_args;
  struct rt_timer_s *rt_timer;

  rt_timer_args.arg = create_args->arg;
  rt_timer_args.callback = create_args->callback;

  ret = esp32s3_rt_timer_create(&rt_timer_args, &rt_timer);
  if (ret)
    {
      wlerr("Failed to create rt_timer error=%d\n", ret);
      return ret;
    }

  *out_handle = (esp_timer_handle_t)rt_timer;

  return 0;
}

/****************************************************************************
 * Name: esp_timer_start_once
 *
 * Description:
 *   Start timer with one shot mode
 *
 * Input Parameters:
 *   timer      - Timer handle pointer
 *   timeout_us - Timeout value by micro second
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int32_t esp_timer_start_once(esp_timer_handle_t timer, uint64_t timeout_us)
{
  struct rt_timer_s *rt_timer = (struct rt_timer_s *)timer;

  esp32s3_rt_timer_start(rt_timer, timeout_us, false);

  return 0;
}

/****************************************************************************
 * Name: esp_timer_start_periodic
 *
 * Description:
 *   Start timer with periodic mode
 *
 * Input Parameters:
 *   timer  - Timer handle pointer
 *   period - Timeout value by micro second
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int32_t esp_timer_start_periodic(esp_timer_handle_t timer, uint64_t period)
{
  struct rt_timer_s *rt_timer = (struct rt_timer_s *)timer;

  esp32s3_rt_timer_start(rt_timer, period, true);

  return 0;
}

/****************************************************************************
 * Name: esp_timer_stop
 *
 * Description:
 *   Stop timer
 *
 * Input Parameters:
 *   timer  - Timer handle pointer
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int32_t esp_timer_stop(esp_timer_handle_t timer)
{
  struct rt_timer_s *rt_timer = (struct rt_timer_s *)timer;

  esp32s3_rt_timer_stop(rt_timer);

  return 0;
}

/****************************************************************************
 * Name: esp_timer_delete
 *
 * Description:
 *   Delete timer and free resource
 *
 * Input Parameters:
 *   timer  - Timer handle pointer
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int32_t esp_timer_delete(esp_timer_handle_t timer)
{
  struct rt_timer_s *rt_timer = (struct rt_timer_s *)timer;

  esp32s3_rt_timer_delete(rt_timer);

  return 0;
}

/****************************************************************************
 * Name: esp_init_semcache
 *
 * Description:
 *   Initialize semaphore cache.
 *
 * Parameters:
 *   sc  - Semaphore cache data pointer
 *   sem - Semaphore data pointer
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_init_semcache(struct esp_semcache_s *sc, sem_t *sem)
{
  sc->sem   = sem;
  sc->count = 0;
  list_initialize(&sc->node);
}

/****************************************************************************
 * Name: esp_post_semcache
 *
 * Description:
 *   Store posting semaphore action into semaphore cache.
 *
 * Parameters:
 *   sc  - Semaphore cache data pointer
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

IRAM_ATTR void esp_post_semcache(struct esp_semcache_s *sc)
{
  struct esp_wireless_priv_s *priv = &g_esp_wireless_priv;

  if (!sc->count)
    {
      list_add_tail(&priv->sc_list, &sc->node);
    }

  sc->count++;

  /* Enable CPU 2 interrupt. This will generate an IRQ as soon as non-IRAM
   * are (re)enabled.
   */

  modifyreg32(SYSTEM_CPU_INTR_FROM_CPU_2_REG, 0, SYSTEM_CPU_INTR_FROM_CPU_2);
}

/****************************************************************************
 * Name: esp_init_queuecache
 *
 * Description:
 *   Initialize queue cache.
 *
 * Parameters:
 *   qc     - Queue cache data pointer
 *   mq_ptr - Queue data pointer
 *   buffer - Queue cache buffer pointer
 *   size   - Queue cache buffer size
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_init_queuecache(struct esp_queuecache_s *qc,
                         struct file *mq_ptr,
                         uint8_t *buffer,
                         size_t size)
{
  qc->mq_ptr = mq_ptr;
  qc->size   = size;
  qc->buffer = buffer;
  list_initialize(&qc->node);
}

/****************************************************************************
 * Name: esp_send_queuecache
 *
 * Description:
 *   Store posting queue action and data into queue cache.
 *
 * Parameters:
 *   qc     - Queue cache data pointer
 *   buffer - Data buffer
 *   size   - Buffer size
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

IRAM_ATTR void esp_send_queuecache(struct esp_queuecache_s *qc,
                                   uint8_t *buffer,
                                   int size)
{
  struct esp_wireless_priv_s *priv = &g_esp_wireless_priv;

  DEBUGASSERT(qc->size == size);

  list_add_tail(&priv->qc_list, &qc->node);
  memcpy(qc->buffer, buffer, size);

  /* Enable CPU 0 interrupt. This will generate an IRQ as soon as non-IRAM
   * are (re)enabled.
   */

  modifyreg32(SYSTEM_CPU_INTR_FROM_CPU_2_REG, 0, SYSTEM_CPU_INTR_FROM_CPU_2);
}

/****************************************************************************
 * Name: esp_wireless_init
 *
 * Description:
 *   Initialize ESP32 wireless common components for both BT and Wi-Fi.
 *
 * Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int esp_wireless_init(void)
{
  int ret;
  irqstate_t flags;
  struct esp_wireless_priv_s *priv = &g_esp_wireless_priv;

  flags = enter_critical_section();
  if (priv->ref != 0)
    {
      priv->ref++;
      leave_critical_section(flags);
      return OK;
    }

  priv->cpuint = esp32s3_setup_irq(0, SWI_PERIPH, ESP32S3_INT_PRIO_DEF, 0);
  if (priv->cpuint < 0)
    {
      /* Failed to allocate a CPU interrupt of this type. */

      wlerr("ERROR: Failed to attach IRQ ret=%d\n", ret);
      ret = priv->cpuint;
      leave_critical_section(flags);

      return ret;
    }

  ret = irq_attach(SWI_IRQ, esp_swi_irq, NULL);
  if (ret < 0)
    {
      esp32s3_teardown_irq(0, SWI_PERIPH, priv->cpuint);
      leave_critical_section(flags);
      wlerr("ERROR: Failed to attach IRQ ret=%d\n", ret);

      return ret;
    }

  list_initialize(&priv->sc_list);
  list_initialize(&priv->qc_list);

  up_enable_irq(SWI_IRQ);

  priv->ref++;

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: esp_wireless_deinit
 *
 * Description:
 *   De-initialize ESP32 wireless common components.
 *
 * Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int esp_wireless_deinit(void)
{
  irqstate_t flags;
  struct esp_wireless_priv_s *priv = &g_esp_wireless_priv;

  flags = enter_critical_section();

  if (priv->ref > 0)
    {
      priv->ref--;
      if (priv->ref == 0)
        {
          up_disable_irq(SWI_IRQ);
          irq_detach(SWI_IRQ);
          esp32s3_teardown_irq(0, SWI_PERIPH, priv->cpuint);
        }
    }

  leave_critical_section(flags);

  return OK;
}

#ifdef CONFIG_ESP32S3_WIFI

/****************************************************************************
 * Name: esp_wifi_init
 *
 * Description:
 *   Initialize Wi-Fi
 *
 * Input Parameters:
 *   config - Initialization config parameters
 *
 * Returned Value:
 *   0 if success or others if fail
 *
 ****************************************************************************/

int32_t esp_wifi_init(const wifi_init_config_t *config)
{
  int32_t ret;

  esp_wifi_power_domain_on();

#ifdef CONFIG_ESP32S3_WIFI_BT_COEXIST
  ret = coex_init();
  if (ret)
    {
      wlerr("ERROR: Failed to initialize coex error=%d\n", ret);
      return ret;
    }
#endif /* CONFIG_ESP32S3_WIFI_BT_COEXIST */

  esp_wifi_set_log_level();

  ret = esp_wifi_init_internal(config);
  if (ret)
    {
      wlerr("Failed to initialize Wi-Fi error=%d\n", ret);
      return ret;
    }

  esp_phy_modem_init();

  ret = esp_supplicant_init();
  if (ret)
    {
      wlerr("Failed to initialize WPA supplicant error=%d\n", ret);
      esp_wifi_deinit_internal();
      return ret;
    }

  return 0;
}

/****************************************************************************
 * Name: esp_wifi_deinit
 *
 * Description:
 *   Deinitialize Wi-Fi and free resource
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   0 if success or others if fail
 *
 ****************************************************************************/

int32_t esp_wifi_deinit(void)
{
  int ret;

  ret = esp_supplicant_deinit();
  if (ret)
    {
      wlerr("Failed to deinitialize supplicant\n");
      return ret;
    }

  ret = esp_wifi_deinit_internal();
  if (ret != 0)
    {
      wlerr("Failed to deinitialize Wi-Fi\n");
      return ret;
    }

  return ret;
}
#endif /* CONFIG_ESP32S3_WIFI */
