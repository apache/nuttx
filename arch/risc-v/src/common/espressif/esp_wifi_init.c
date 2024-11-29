/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_wifi_init.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <esp_event.h>
#include <esp_wifi.h>
#include "esp_log.h"
#include "esp_private/wifi.h"
#include "esp_private/adc_share_hw_ctrl.h"
#include "esp_private/sleep_modem.h"
#include "esp_sleep.h"
#include "esp_private/esp_clk.h"
#include "esp_wpa.h"
#include "private/esp_coexist_internal.h"
#include "esp_phy_init.h"
#include "esp_private/phy.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if (CONFIG_ESPRESSIF_WIFI_RX_BA_WIN > CONFIG_ESPRESSIF_WIFI_DYNAMIC_RX_BUFFER_NUM)
#error "WiFi configuration check: WARNING, WIFI_RX_BA_WIN should not be larger than WIFI_DYNAMIC_RX_BUFFER_NUM!"
#endif

#if (CONFIG_ESPRESSIF_WIFI_RX_BA_WIN > (CONFIG_ESPRESSIF_WIFI_STATIC_RX_BUFFER_NUM << 1))
#error "WiFi configuration check: WARNING, WIFI_RX_BA_WIN should not be larger than double of the WIFI_STATIC_RX_BUFFER_NUM!"
#endif

#if SOC_PM_SUPPORT_PMU_MODEM_STATE
# define WIFI_BEACON_MONITOR_CONFIG_DEFAULT(ena)   { \
    .enable = (ena), \
    .loss_timeout = CONFIG_ESP_WIFI_SLP_BEACON_LOST_TIMEOUT, \
    .loss_threshold = CONFIG_ESP_WIFI_SLP_BEACON_LOST_THRESHOLD, \
    .delta_intr_early = 0, \
    .delta_loss_timeout = 0, \
    .beacon_abort = 1, \
    .broadcast_wakeup = 1, \
    .tsf_time_sync_deviation = 5, \
    .modem_state_consecutive = 10, \
    .rf_ctrl_wait_cycle = 20 \
}
#else
# define WIFI_BEACON_MONITOR_CONFIG_DEFAULT(ena)   { \
    .enable = (ena), \
    .loss_timeout = CONFIG_ESP_WIFI_SLP_BEACON_LOST_TIMEOUT, \
    .loss_threshold = CONFIG_ESP_WIFI_SLP_BEACON_LOST_THRESHOLD, \
    .delta_intr_early = CONFIG_ESP_WIFI_SLP_PHY_ON_DELTA_EARLY_TIME, \
    .delta_loss_timeout = CONFIG_ESP_WIFI_SLP_PHY_OFF_DELTA_TIMEOUT_TIME \
}
#endif

/* Set additional WiFi features and capabilities */

uint64_t g_wifi_feature_caps =
#if CONFIG_ESP_WIFI_ENABLE_WPA3_SAE
    CONFIG_FEATURE_WPA3_SAE_BIT |
#endif
#if CONFIG_SPIRAM
    CONFIG_FEATURE_CACHE_TX_BUF_BIT |
#endif
#if CONFIG_ESP_WIFI_FTM_INITIATOR_SUPPORT
    CONFIG_FEATURE_FTM_INITIATOR_BIT |
#endif
#if CONFIG_ESP_WIFI_FTM_RESPONDER_SUPPORT
    CONFIG_FEATURE_FTM_RESPONDER_BIT |
#endif
    0;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void esp_wifi_set_log_level(void);
static void esp_wifi_config_info(void);

extern uint8_t esp_wifi_get_user_init_flag_internal(void);

/****************************************************************************
 * Public Data
 ****************************************************************************/

ESP_EVENT_DEFINE_BASE(WIFI_EVENT);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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

/****************************************************************************
 * Name: esp_wifi_config_info
 *
 * Description:
 *   This function logs the current configuration settings for the Wi-Fi
 *   module. It checks for various configuration options and logs if they
 *   are enabled.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp_wifi_config_info(void)
{
#ifdef CONFIG_ESPRESSIF_WIFI_RX_BA_WIN
  wlinfo("rx ba win: %d", CONFIG_ESPRESSIF_WIFI_RX_BA_WIN);
#endif

#ifdef CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP
  wlinfo("WiFi/LWIP prefer SPIRAM");
#endif

#ifdef CONFIG_ESP_WIFI_IRAM_OPT
  wlinfo("WiFi IRAM OP enabled");
#endif

#ifdef CONFIG_ESP_WIFI_RX_IRAM_OPT
  wlinfo("WiFi RX IRAM OP enabled");
#endif

#ifdef CONFIG_ESP_WIFI_SLP_IRAM_OPT
  wlinfo("WiFi SLP IRAM OP enabled");
#endif

#ifdef CONFIG_LWIP_IRAM_OPTIMIZATION
  wlinfo("LWIP IRAM OP enabled");
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifndef CONFIG_ESP_WIFI_FTM_ENABLE

/****************************************************************************
 * Name: ieee80211_ftm_attach
 *
 * Description:
 *   This function initializes and attaches the Fine Timing Measurement (FTM)
 *   capabilities to the IEEE 802.11 Wi-Fi driver. FTM is used for precise
 *   distance measurements between Wi-Fi devices.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void ieee80211_ftm_attach(void)
{
  /* Do not remove, stub to overwrite weak link in Wi-Fi Lib */
}
#endif

#ifndef CONFIG_ESP_WIFI_SOFTAP_SUPPORT

/****************************************************************************
 * Name: net80211_softap_funcs_init
 *
 * Description:
 *   This function is a placeholder for initializing the SoftAP (Software
 *   Access Point) functionalities of the IEEE 802.11 Wi-Fi driver. It is
 *   only compiled if the CONFIG_ESP_WIFI_SOFTAP_SUPPORT configuration
 *   option is not set.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void net80211_softap_funcs_init(void)
{
}
#endif

#ifndef CONFIG_ESP_WIFI_NAN_ENABLE

/****************************************************************************
 * Name: nan_start
 *
 * Description:
 *   This function is a stub to overwrite a weak link in the Wi-Fi library.
 *   It is used to start the NAN (Neighbor Awareness Networking) service.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Always returns ESP_OK.
 *
 ****************************************************************************/

esp_err_t nan_start(void)
{
  /* Do not remove, stub to overwrite weak link in Wi-Fi Lib */

  return ESP_OK;
}

/****************************************************************************
 * Name: nan_stop
 *
 * Description:
 *   This function is a stub to overwrite a weak link in the Wi-Fi library.
 *   It is used to stop the NAN (Neighbor Awareness Networking) service.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Always returns ESP_OK.
 *
 ****************************************************************************/

esp_err_t nan_stop(void)
{
  /* Do not remove, stub to overwrite weak link in Wi-Fi Lib */

  return ESP_OK;
}

/****************************************************************************
 * Name: nan_input
 *
 * Description:
 *   This function is a stub to overwrite a weak link in the Wi-Fi library.
 *   It is used to handle input for the NAN (Neighbor Awareness Networking)
 *   service.
 *
 * Input Parameters:
 *   p1 - First parameter for the input function.
 *   p2 - Second parameter for the input function.
 *   p3 - Third parameter for the input function.
 *
 * Returned Value:
 *   Always returns 0.
 *
 ****************************************************************************/

int nan_input(void *p1, int p2, int p3)
{
  /* Do not remove, stub to overwrite weak link in Wi-Fi Lib */

  return 0;
}

/****************************************************************************
 * Name: nan_sm_handle_event
 *
 * Description:
 *   This function is a stub to overwrite a weak link in the Wi-Fi library.
 *   It is used to handle events for the NAN (Neighbor Awareness Networking)
 *   service state machine.
 *
 * Input Parameters:
 *   p1 - First parameter for the event handler.
 *   p2 - Second parameter for the event handler.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nan_sm_handle_event(void *p1, int p2)
{
  /* Do not remove, stub to overwrite weak link in Wi-Fi Lib */
}
#endif

/****************************************************************************
 * Name: esp_wifi_deinit
 *
 * Description:
 *   Deinitialize Wi-Fi and free resource
 *
 * Input Parameters:
 *   None
 *
 * Returned Values: esp_err_t
 *   Zero (OK) is returned or a negative error.
 *
 ****************************************************************************/

esp_err_t esp_wifi_deinit(void)
{
  esp_err_t err = ESP_OK;
#ifdef CONFIG_ESP_WIFI_SLP_BEACON_LOST_OPT
  wifi_beacon_monitor_config_t monitor_config;
#endif

  if (esp_wifi_get_user_init_flag_internal())
    {
      wlerr("Wi-Fi not stop");
      return ESP_ERR_WIFI_NOT_STOPPED;
    }

  if (esp_wifi_internal_reg_rxcb(WIFI_IF_STA,  NULL) != ESP_OK ||
      esp_wifi_internal_reg_rxcb(WIFI_IF_AP,  NULL) != ESP_OK)
    {
      wlerr("Failed to unregister Rx callbacks");
    }

  esp_supplicant_deinit();
  err = esp_wifi_deinit_internal();
  if (err != ESP_OK)
    {
      wlerr("Failed to deinit Wi-Fi driver (0x%x)", err);
      return err;
    }

#ifdef CONFIG_ESP_WIFI_SLP_BEACON_LOST_OPT
  monitor_config = WIFI_BEACON_MONITOR_CONFIG_DEFAULT(false);
  esp_wifi_beacon_monitor_configure(&monitor_config);
#endif

#if CONFIG_MAC_BB_PD
  esp_unregister_mac_bb_pd_callback(pm_mac_sleep);
  esp_unregister_mac_bb_pu_callback(pm_mac_wakeup);
#endif
  esp_wifi_power_domain_off();
#if CONFIG_MAC_BB_PD
  esp_wifi_internal_set_mac_sleep(false);
  esp_mac_bb_pd_mem_deinit();
#endif
  esp_phy_modem_deinit();

  return err;
}

/****************************************************************************
 * Name: esp_wifi_init
 *
 * Description:
 *   Initialize Wi-Fi
 *
 * Input Parameters:
 *   config - Initialization config parameters
 *
 * Returned Values: esp_err_t
 *   Zero (OK) is returned or a negative error.
 *
 ****************************************************************************/

esp_err_t esp_wifi_init(const wifi_init_config_t *config)
{
  esp_err_t result = ESP_OK;
#ifdef CONFIG_ESP_WIFI_SLP_BEACON_LOST_OPT
  wifi_beacon_monitor_config_t monitor_config;
#endif

  if ((config->feature_caps & CONFIG_FEATURE_CACHE_TX_BUF_BIT) &&
      (WIFI_CACHE_TX_BUFFER_NUM == 0))
    {
      wlerr("Number of WiFi cache TX buffers should not equal 0 when"
            "enable SPIRAM");
      return ESP_ERR_NOT_SUPPORTED;
    }

  esp_wifi_power_domain_on();

#if CONFIG_SW_COEXIST_ENABLE || CONFIG_EXTERNAL_COEX_ENABLE
  coex_init();
#endif
  esp_wifi_set_log_level();

  result = esp_wifi_init_internal(config);
  if (result == ESP_OK)
    {
#if CONFIG_MAC_BB_PD
      esp_mac_bb_pd_mem_init();
      esp_wifi_internal_set_mac_sleep(true);
#endif
      esp_phy_modem_init();

      result = esp_supplicant_init();
      if (result != ESP_OK)
        {
          esp_err_t deinit_ret;

          wlerr("Failed to init supplicant (0x%x)", result);

          deinit_ret = esp_wifi_deinit();
          if (deinit_ret != ESP_OK)
            {
              wlerr("Failed to deinit Wi-Fi (0x%x)", deinit_ret);
            }

          return result;
        }
    }

#if CONFIG_ESP_WIFI_SLP_BEACON_LOST_OPT
  monitor_config = WIFI_BEACON_MONITOR_CONFIG_DEFAULT(true);
  esp_wifi_beacon_monitor_configure(&monitor_config);
#endif
  adc2_cal_include(); /* This enables the ADC2 calibration constructor at start up */

  esp_wifi_config_info();

  return result;
}
