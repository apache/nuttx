/****************************************************************************
 * arch/xtensa/src/common/espressif/esp_wifi_api.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <inttypes.h>
#include <assert.h>
#include <debug.h>
#include <math.h>

#include "esp_mac.h"
#include "esp_wifi_types_generic.h"
#include "esp_wifi.h"
#include "esp_private/wifi.h"

#include "esp_wifi_utils.h"
#include "esp_wifi_api.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define WIFI_CONNECT_TIMEOUT   CONFIG_ESPRESSIF_WIFI_CONNECT_TIMEOUT
#define WIFI_CONNECT_RETRY_CNT 3

#define ESP_WIFI_11B_MAX_BITRATE       11
#define ESP_WIFI_11G_MAX_BITRATE       54
#define ESP_WIFI_11N_MCS7_HT20_BITRATE 72
#define ESP_WIFI_11N_MCS7_HT40_BITRATE 150

/* Number of fractional bits in values returned by rtc_clk_cal */

#define RTC_CLK_CAL_FRACT 19

#define PWD_MAX_LEN                 (64)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_wifi_api_adapter_deinit
 *
 * Description:
 *   De-initialize Wi-Fi adapter, freeing all resources allocated by
 *   esp_wifi_init. Also stops the Wi-Fi task.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp_wifi_api_adapter_deinit(void)
{
  int ret;

  ret = esp_wifi_deinit();
  if (ret != 0)
    {
      ret = esp_wifi_to_errno(ret);
      wlerr("Failed to deinitialize Wi-Fi error=%d\n", ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: esp_wifi_api_adapter_init
 *
 * Description:
 *   Initialize the Wi-Fi driver, control structure, buffers and Wi-Fi task.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp_wifi_api_adapter_init(void)
{
  int ret;
  wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();

  esp_wifi_lock(true);

  esp_evt_work_init();

  wifi_cfg.nvs_enable = 0;

#ifdef CONFIG_ESPRESSIF_WIFI_AMPDU_TX_ENABLED
  wifi_cfg.ampdu_tx_enable = 1;
#else
  wifi_cfg.ampdu_tx_enable = 0;
#endif

#ifdef CONFIG_ESPRESSIF_WIFI_AMPDU_RX_ENABLED
  wifi_cfg.ampdu_rx_enable = 1;
#else
  wifi_cfg.ampdu_rx_enable = 0;
#endif

#ifdef CONFIG_ESPRESSIF_WIFI_STA_DISCONNECT_PM
  wifi_cfg.sta_disconnected_pm = true;
#else
  wifi_cfg.sta_disconnected_pm = false;
#endif

  wifi_cfg.rx_ba_win          = CONFIG_ESPRESSIF_WIFI_TX_BA_WIN;
  wifi_cfg.static_rx_buf_num  = CONFIG_ESPRESSIF_WIFI_STATIC_RX_BUFFER_NUM;
  wifi_cfg.static_tx_buf_num  = CONFIG_ESPRESSIF_WIFI_STATIC_TX_BUFFER_NUM;
  wifi_cfg.dynamic_rx_buf_num = CONFIG_ESPRESSIF_WIFI_DYNAMIC_RX_BUFFER_NUM;
  wifi_cfg.dynamic_tx_buf_num = CONFIG_ESPRESSIF_WIFI_DYNAMIC_TX_BUFFER_NUM;
  wifi_cfg.tx_buf_type        = CONFIG_ESPRESSIF_WIFI_TX_BUFFER_TYPE;

  ret = esp_wifi_init(&wifi_cfg);
  if (ret != 0)
    {
      wlerr("Failed to initialize Wi-Fi error=%d\n", ret);
      ret = esp_wifi_to_errno(ret);
      goto errout_init_wifi;
    }

  wlinfo("Wi-Fi adapter initialized\n");

  /* Set Wi-Fi mode to NULL, this way it requires ifup call to set
   * the proper wifi mode. Otherwise, it starts as STA mode.
   */

  ret = esp_wifi_set_mode(WIFI_MODE_NULL);
  if (ret != 0)
    {
      wlerr("Failed to set Wi-Fi mode to NULL ret=%d\n", ret);
      ret = esp_wifi_to_errno(ret);
      goto errout_init_wifi;
    }

  esp_wifi_lock(false);

  return OK;

errout_init_wifi:
  esp_wifi_lock(false);

  return ret;
}

/****************************************************************************
 * Name: esp_wifi_api_start
 *
 * Description:
 *   Start Wi-Fi station. This will start the proper Wi-Fi mode based on
 *   the AP/Station configuration.
 *
 * Input Parameters:
 *   start_mode - The Wi-Fi mode to start from
 *                nuttx/include/nuttx/wireless/wireless.h.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp_wifi_api_start(uint32_t start_mode)
{
  int ret;
  wifi_mode_t mode;
  wifi_mode_t current_mode;

  ret = esp_wifi_get_mode(&current_mode);
  if (ret != 0)
    {
      wlerr("Failed to get Wi-Fi mode ret=%d. Check if initialized\n",
            ret);
      ret = esp_wifi_to_errno(ret);
      goto errout;
    }

  mode = esp_wifi_mode_translate(start_mode);
  if (mode == -EINVAL)
    {
      wlerr("Invalid mode=%ld\n", start_mode);
      return -EINVAL;
    }

  wlinfo("current_mode=%d, mode=%d\n", current_mode, mode);

#ifdef ESP_WLAN_HAS_APSTA
  if ((mode == WIFI_MODE_STA && current_mode == WIFI_MODE_AP) ||
      (mode == WIFI_MODE_AP && current_mode == WIFI_MODE_STA))
    {
      wlinfo("Wi-Fi mode set to APSTA\n");
      mode = WIFI_MODE_APSTA;
    }
#endif

  esp_wifi_lock(true);

  ret = esp_wifi_set_mode(mode);
  if (ret != 0)
    {
      wlerr("Failed to set Wi-Fi mode=%d ret=%d\n", mode, ret);
      ret = esp_wifi_to_errno(ret);
      goto errout;
    }

  /* Use a default AP config for startup */

#ifdef ESP_WLAN_HAS_SOFTAP
  if (mode == WIFI_MODE_AP || mode == WIFI_MODE_APSTA)
    {
      wlinfo("Setting default AP config SSID=%s\n",
             CONFIG_ESPRESSIF_WIFI_SOFTAP_DEFAULT_SSID);
      wifi_config_t wifi_config =
        {
          .ap =
            {
              .ssid = CONFIG_ESPRESSIF_WIFI_SOFTAP_DEFAULT_SSID,
              .ssid_len = strlen(CONFIG_ESPRESSIF_WIFI_SOFTAP_DEFAULT_SSID),
              .channel = CONFIG_ESPRESSIF_WIFI_SOFTAP_CHANNEL,
              .password = CONFIG_ESPRESSIF_WIFI_SOFTAP_DEFAULT_PASSWORD,
              .max_connection = CONFIG_ESPRESSIF_WIFI_SOFTAP_MAX_CONNECTIONS,
#ifdef CONFIG_ESPRESSIF_WIFI_SOFTAP_SAE_SUPPORT
              .authmode = WIFI_AUTH_WPA3_PSK,
              .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
#else /* CONFIG_ESPRESSIF_WIFI_SOFTAP_SAE_SUPPORT */
              .authmode = WIFI_AUTH_WPA2_PSK,
#endif
              .pmf_cfg =
                {
                  .required = true,
                },
            },
        };

      ret = esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
      if (ret != 0)
        {
          wlerr("Failed to set Wi-Fi config data ret=%d\n", ret);
          ret = esp_wifi_to_errno(ret);
          goto errout;
        }
    }
#endif  /* ESP_WLAN_HAS_SOFTAP */

  ret = esp_wifi_start();
  if (ret != 0)
    {
      wlerr("Failed to start Wi-Fi with mode=%d ret=%d\n", mode, ret);
      ret = esp_wifi_to_errno(ret);
      goto errout;
    }

  wlinfo("Wi-Fi started with mode=%d\n", mode);

errout:
  esp_wifi_lock(false);
  return ret;
}

/****************************************************************************
 * Name: esp_wifi_api_stop
 *
 * Description:
 *   Stops Wi-Fi AP, Station or both. Can be later resumed by
 *   esp_wifi_restore, but must be reconfigured.
 *
 *   If AP + SoftAP are running, be aware that both will be stopped briefly,
 *   and then the remaining one will be restarted.
 *
 * Input Parameters:
 *   start_mode - The Wi-Fi mode to start from
 *                nuttx/include/nuttx/wireless/wireless.h.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp_wifi_api_stop(uint32_t stop_mode)
{
  int ret;
  wifi_mode_t mode;
  wifi_mode_t current_mode;

  esp_wifi_lock(true);

  ret = esp_wifi_get_mode(&current_mode);
  if (ret != 0)
    {
      wlerr("Failed to get Wi-Fi mode ret=%d. Check if initialized\n",
            ret);
      ret = esp_wifi_to_errno(ret);
      goto errout;
    }

  mode = esp_wifi_mode_translate(stop_mode);
  if (mode == -EINVAL)
    {
      wlerr("Invalid mode=%ld\n", stop_mode);
      return -EINVAL;
    }

  wlinfo("current_mode=%d, mode=%d\n", current_mode, mode);

  ret = esp_wifi_stop();
  if (ret != 0)
    {
      wlerr("Failed to stop Wi-Fi ret=%d\n", ret);
      ret = esp_wifi_to_errno(ret);
      goto errout;
    }

  wlinfo("Stopped Wi-Fi\n");

  if (current_mode == WIFI_MODE_APSTA)
    {
      if (mode == WIFI_MODE_STA)
        {
          wlinfo("Restarting Wi-Fi in AP mode\n");
          ret = esp_wifi_set_mode(WIFI_MODE_AP);
        }
      else if (mode == WIFI_MODE_AP)
        {
          wlinfo("Restarting Wi-Fi in STA mode\n");
          ret = esp_wifi_set_mode(WIFI_MODE_STA);
        }
      else
        {
          wlerr("Invalid mode=%d\n", mode);
          goto errout;
        }

      if (ret != 0)
        {
          wlerr("Failed to reset Wi-Fi mode=%d ret=%d\n",
                WIFI_MODE_AP, ret);
          ret = esp_wifi_to_errno(ret);
          goto errout;
        }

      ret = esp_wifi_start();
      if (ret != 0)
        {
          wlerr("Failed to restart Wi-Fi ret=%d\n", ret);
          ret = esp_wifi_to_errno(ret);
          goto errout;
        }
    }

errout:
  esp_wifi_lock(false);
  return ret;
}

/****************************************************************************
 * Name: esp_wifi_api_sta_register_rx_callback
 *
 * Description:
 *   Register a callback function for the Wi-Fi station interface.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp_wifi_api_sta_register_rx_callback(void *cb)
{
  return esp_wifi_internal_reg_rxcb(WIFI_IF_STA, (wifi_rxcb_t) cb);
}

/****************************************************************************
 * Name: esp_wifi_api_softap_register_rx_callback
 *
 * Description:
 *   Register a callback function for the Wi-Fi softAP interface.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp_wifi_api_softap_register_rx_callback(void *cb)
{
  return esp_wifi_internal_reg_rxcb(WIFI_IF_AP, (wifi_rxcb_t) cb);
}

/****************************************************************************
 * Name: esp_wifi_api_registe_tx_done_callback
 *
 * Description:
 *   Register a callback function for transmission done. Valid for
 *   both station and softAP and needs to be called only once on bringup.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp_wifi_api_register_tx_done_callback(void *cb)
{
  return esp_wifi_set_tx_done_cb((wifi_tx_done_cb_t) cb);
}

/****************************************************************************
 * Name: esp_wifi_api_free_rx_buffer
 *
 * Description:
 *   Free the RX buffer allocated by the Wi-Fi driver.
 *
 * Input Parameters:
 *   eb - The event buffer to free
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_wifi_api_free_rx_buffer(void *eb)
{
  esp_wifi_internal_free_rx_buffer(eb);
}

/****************************************************************************
 * Name: esp_wifi_txpower
 *
 * Description:
 *   Get/Set station transmit power (dBm). Used by both station and SoftAP,
 *   since they share the radio.
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success; Negated errno returned on failure.
 *
 ****************************************************************************/

static int esp_wifi_txpower(struct iwreq *iwr, bool set)
{
  int ret;
  int8_t power;
  double power_dbm;

  if (set)
    {
      if (iwr->u.txpower.flags == IW_TXPOW_RELATIVE)
        {
          power = (int8_t)iwr->u.txpower.value;
        }
      else
        {
          if (iwr->u.txpower.flags == IW_TXPOW_MWATT)
            {
              power_dbm = ceil(10 * log10(iwr->u.txpower.value));
            }
          else
            {
              power_dbm = iwr->u.txpower.value;
            }

          power = (int8_t)(power_dbm * 4);
        }

      /* The value set by this API will be mapped to the max_tx_power
       * of the structure wifi_country_t variable. Param power unit is
       * 0.25dBm, range is [8, 84] corresponding to 2dBm - 20dBm.
       * Relationship between set value and actual value.
       * As follows: {set value range, actual value} =
       * {{[8,  19],8}, {[20, 27],20}, {[28, 33],28},
       * {[34, 43],34}, {[44, 51],44}, {[52, 55],52},
       * {[56, 59],56}, {[60, 65],60}, {[66, 71],66},
       * {[72, 79],72}, {[80, 84],80}}.
       */

      if (power < 8 || power > 84)
        {
          wlerr("Failed to set transmit power =%d\n", power);
          return -ENOSYS;
        }

      esp_wifi_set_max_tx_power(power);
      return OK;
    }
  else
    {
      ret = esp_wifi_get_max_tx_power(&power);
      if (ret != 0)
        {
          wlerr("Failed to get transmit power ret=%d\n", ret);
          return esp_wifi_to_errno(ret);
        }

      iwr->u.txpower.disabled = 0;
      iwr->u.txpower.flags    = IW_TXPOW_DBM;
      iwr->u.txpower.value    = power / 4;
    }

  return OK;
}

/****************************************************************************
 * Name: esp_wifi_channel
 *
 * Description:
 *   Get station range of channel parameters.
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success; Negated errno returned on failure.
 *
 ****************************************************************************/

static int esp_wifi_channel(struct iwreq *iwr, bool set)
{
  int ret;
  int k;
  wifi_country_t country;
  struct iw_range *range;

  if (set)
    {
      return -ENOSYS;
    }
  else
    {
      ret = esp_wifi_get_country(&country);
      if (ret != 0)
        {
          wlerr("Failed to get country info ret=%d\n", ret);
          return esp_wifi_to_errno(ret);
        }

      range = (struct iw_range *)iwr->u.data.pointer;
      range->num_frequency = country.nchan;
      for (k = 1; k <= range->num_frequency; k++)
        {
          range->freq[k - 1].i = k;
          range->freq[k - 1].e = 0;
          range->freq[k - 1].m = 2407 + 5 * k;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: esp_wifi_country
 *
 * Description:
 *   Configure country info.
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success; Negated errno returned on failure.
 *
 ****************************************************************************/

static int esp_wifi_country(struct iwreq *iwr, bool set)
{
  int ret;
  char *country_code;
  wifi_country_t country;

  if (set)
    {
      memset(&country, 0x00, sizeof(wifi_country_t));
      country.schan  = 1;
      country.policy = 0;

      country_code = (char *)iwr->u.data.pointer;
      if (strlen(country_code) != 2)
        {
          wlerr("Invalid input arguments\n");
          return -EINVAL;
        }

      if (strncmp(country_code, "US", 3) == 0 ||
          strncmp(country_code, "CA", 3) == 0)
        {
          country.nchan  = 11;
        }
      else if(strncmp(country_code, "JP", 3) == 0)
        {
          country.nchan  = 14;
        }
      else
        {
          country.nchan  = 13;
        }

      memcpy(country.cc, country_code, 2);
      ret = esp_wifi_set_country(&country);
      if (ret != 0)
        {
          wlerr("Failed to  Configure country ret=%d\n", ret);
          return esp_wifi_to_errno(ret);
        }
    }
  else
    {
      return -ENOSYS;
    }

  return OK;
}

/****************************************************************************
 * Wi-Fi Station functions
 ****************************************************************************/

#ifdef ESP_WLAN_HAS_STA

/****************************************************************************
 * Name: esp_wifi_sta_connect
 *
 * Description:
 *   Trigger Wi-Fi station connection action.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_connect(void)
{
  int ret;

  wlinfo("Wi-Fi station connecting\n");

  esp_wifi_lock(true);

  ret = esp_wifi_connect();
  if (ret != 0)
    {
      wlerr("Failed to connect ret=%d\n", ret);
      ret = esp_wifi_to_errno(ret);
    }

  esp_wifi_lock(false);

  return ret;
}

/****************************************************************************
 * Name: esp_wifi_sta_disconnect
 *
 * Description:
 *   Trigger Wi-Fi station disconnection action.
 *
 * Input Parameters:
 *   allow_reconnect - True if Wi-Fi should attempt to reconnect.
 *                     This is used to prevent the Wi-Fi from reconnecting
 *                     when the user has actually asked to disconnect from
 *                     the AP, otherwise it will attempt reconnections on
 *                     WIFI_EVENT_STA_DISCONNECTED event.
 *
 * Returned Value:
 *   OK on success; Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_disconnect(bool allow_reconnect)
{
  int ret;
  wifi_config_t wifi_config;

  esp_wifi_lock(true);
  esp_wifi_get_config(WIFI_IF_STA, &wifi_config);

  if (allow_reconnect)
    {
      wifi_config.sta.failure_retry_cnt = WIFI_CONNECT_RETRY_CNT;
    }
  else
    {
      wifi_config.sta.failure_retry_cnt = 0;
    }

  esp_wifi_set_config(WIFI_IF_STA, &wifi_config);

  ret = esp_wifi_disconnect();
  if (ret != 0)
    {
      wlerr("Failed to disconnect ret=%d\n", ret);
      ret = esp_wifi_to_errno(ret);
    }
  else
    {
      wlinfo("Wi-Fi station disconnected\n");
    }

  esp_wifi_lock(false);

  return ret;
}

/****************************************************************************
 * Name: esp_wifi_sta_send_data
 *
 * Description:
 *   Use Wi-Fi station interface to send 802.3 frame.
 *
 * Input Parameters:
 *   pbuf - Packet buffer pointer
 *   len  - Packet length
 *
 * Returned Value:
 *   OK on success; Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_send_data(void *pbuf, size_t len)
{
  int ret;

  ret = esp_wifi_internal_tx(WIFI_IF_STA, pbuf, len);
  if (ret != 0)
    {
      wlerr("Failed to send Wi-Fi data ret=%d\n", ret);
      ret = esp_wifi_to_errno(ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: esp_wifi_sta_read_mac
 *
 * Description:
 *   Read station interface MAC address from efuse
 *
 * Input Parameters:
 *   mac  - MAC address buffer pointer
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int esp_wifi_sta_read_mac(uint8_t *mac)
{
  int ret = esp_read_mac(mac, ESP_MAC_WIFI_STA);
  return esp_wifi_to_errno(ret);
}

/****************************************************************************
 * Name: esp_wifi_sta_set_password
 *
 * Description:
 *   Set/Get Wi-Fi station password.
 *   Setting the Wi-Fi will disconnect from the current AP.
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set   - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success; Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_password(struct iwreq *iwr, bool set)
{
  int ret;
  int size;
  wifi_config_t wifi_cfg;
  struct iw_encode_ext *ext = iwr->u.encoding.pointer;
  uint8_t *pdata;
  uint8_t len;
#ifdef CONFIG_DEBUG_WIRELESS_INFO
  char buf[PWD_MAX_LEN + 1];
#endif

  DEBUGASSERT(ext != NULL);

  pdata = ext->key;

  esp_wifi_get_config(WIFI_IF_STA, &wifi_cfg);

  if (set)
    {
      len = ext->key_len;
      if (len > PWD_MAX_LEN)
        {
          return -EINVAL;
        }

      memset(wifi_cfg.sta.password, 0x0, PWD_MAX_LEN);

      if (ext->alg != IW_ENCODE_ALG_NONE)
        {
          memcpy(wifi_cfg.sta.password, pdata, len);
        }

      ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
      if (ret != 0)
        {
          wlerr("Failed to set Wi-Fi config data ret=%d\n", ret);
          return esp_wifi_to_errno(ret);
        }

      ret = esp_wifi_sta_disconnect(true);
      if (ret != 0)
        {
          wlerr("Failed to disconnect from Wi-Fi AP ret=%d\n", ret);
          return ret;
        }
    }
  else
    {
      len = iwr->u.encoding.length - sizeof(*ext);
      size = strnlen((char *)wifi_cfg.sta.password, PWD_MAX_LEN);
      if (len < size)
        {
          return -EINVAL;
        }
      else
        {
          ext->key_len = size;
          memcpy(pdata, wifi_cfg.sta.password, ext->key_len);
        }

      wifi_ap_record_t ap_info;

      ret = esp_wifi_sta_get_ap_info(&ap_info);
      if (ret != 0)
        {
          wlerr("Failed to get AP record ret=%d", ret);
          return esp_wifi_to_errno(ret);
        }

      switch (ap_info.pairwise_cipher)
        {
          case WIFI_CIPHER_TYPE_NONE:
              ext->alg = IW_ENCODE_ALG_NONE;
              break;

          case WIFI_CIPHER_TYPE_WEP40:
          case WIFI_CIPHER_TYPE_WEP104:
              ext->alg = IW_ENCODE_ALG_WEP;
              break;

          case WIFI_CIPHER_TYPE_TKIP:
              ext->alg = IW_ENCODE_ALG_TKIP;
              break;

          case WIFI_CIPHER_TYPE_CCMP:
          case WIFI_CIPHER_TYPE_TKIP_CCMP:
              ext->alg = IW_ENCODE_ALG_CCMP;
              break;

          case WIFI_CIPHER_TYPE_AES_CMAC128:
              ext->alg = IW_ENCODE_ALG_AES_CMAC;
              break;

          default:
              wlerr("Failed to transfer wireless authmode: %d",
                  ap_info.pairwise_cipher);
              return -EIO;
        }
    }

#ifdef CONFIG_DEBUG_WIRELESS_INFO
  memcpy(buf, pdata, len);
  buf[len] = 0;
  wlinfo("Wi-Fi station password=%s len=%d\n", buf, len);
#endif

  return OK;
}

/****************************************************************************
 * Name: esp_wifi_sta_essid
 *
 * Description:
 *   Set/Get Wi-Fi station ESSID.
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd.
 *   set - true: set data; false: get data.
 *
 * Returned Value:
 *   OK on success; Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_essid(struct iwreq *iwr, bool set)
{
  int ret;
  int size;
  wifi_config_t wifi_cfg;
  struct iw_point *essid = &iwr->u.essid;
  uint8_t *pdata;
  uint8_t len;
#ifdef CONFIG_DEBUG_WIRELESS_INFO
  char buf[IW_ESSID_MAX_SIZE + 1];
#endif

  DEBUGASSERT(essid != NULL);

  pdata = essid->pointer;
  len   = essid->length;

  if (set && len > IW_ESSID_MAX_SIZE)
    {
      return -EINVAL;
    }

  esp_wifi_get_config(WIFI_IF_STA, &wifi_cfg);

  if (set)
    {
      memset(wifi_cfg.sta.ssid, 0x0, IW_ESSID_MAX_SIZE);
      memcpy(wifi_cfg.sta.ssid, pdata, len);
      memset(wifi_cfg.sta.sae_h2e_identifier, 0x0, SAE_H2E_IDENTIFIER_LEN);
      wifi_cfg.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;

      ret = esp_wifi_sta_disconnect(true);
      if (ret != 0)
        {
          wlerr("Failed to disconnect from Wi-Fi AP ret=%d\n", ret);
          return ret;
        }

      ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
      if (ret != 0)
        {
          wlerr("Failed to set Wi-Fi config data ret=%d\n", ret);
          return esp_wifi_to_errno(ret);
        }
    }
  else
    {
      size = strnlen((char *)wifi_cfg.sta.ssid, IW_ESSID_MAX_SIZE);
      if (len < size)
        {
          return -EINVAL;
        }
      else
        {
          len = size;
          memcpy(pdata, wifi_cfg.sta.ssid, len);
        }
    }

#ifdef CONFIG_DEBUG_WIRELESS_INFO
  memcpy(buf, pdata, len);
  buf[len] = 0;
  wlinfo("Wi-Fi station ssid=%s len=%d\n", buf, len);
#endif

  return OK;
}

/****************************************************************************
 * Name: esp_wifi_sta_bssid
 *
 * Description:
 *   Set/Get Wi-Fi station BSSID.
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set   - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success; Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_bssid(struct iwreq *iwr, bool set)
{
  int ret;
  wifi_config_t wifi_cfg;
  struct sockaddr *sockaddr;
  char *pdata;

  sockaddr = &iwr->u.ap_addr;
  pdata    = sockaddr->sa_data;

  esp_wifi_get_config(WIFI_IF_STA, &wifi_cfg);

  if (set)
    {
      wifi_cfg.sta.bssid_set = true;
      memcpy(wifi_cfg.sta.bssid, pdata, MAC_LEN);

      wlinfo("set station BSSID: %02x:%02x:%02x:%02x:%02x:%02x\n",
        wifi_cfg.sta.bssid[0], wifi_cfg.sta.bssid[1], wifi_cfg.sta.bssid[2],
        wifi_cfg.sta.bssid[3], wifi_cfg.sta.bssid[4], wifi_cfg.sta.bssid[5]);

      ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
      if (ret != 0)
        {
          wlerr("Failed to set Wi-Fi config data ret=%d\n", ret);
          return esp_wifi_to_errno(ret);
        }

      ret = esp_wifi_sta_disconnect(true);
      if (ret != 0)
        {
          wlerr("Failed to connect to Wi-Fi AP ret=%d\n", ret);
          return ret;
        }
    }
  else
    {
      memcpy(pdata, wifi_cfg.sta.bssid, MAC_LEN);
    }

  return OK;
}

/****************************************************************************
 * Name: esp_wifi_sta_mode
 *
 * Description:
 *   Set/Get Wi-Fi Station mode code.
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success; Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_mode(struct iwreq *iwr, bool set)
{
  if (!set)
    {
      iwr->u.mode = IW_MODE_INFRA;
    }

  return OK;
}

/****************************************************************************
 * Name: esp_wifi_auth_trans
 *
 * Description:
 *   Converts authenticate mode values to WEXT authenticate mode.
 *
 * Input Parameters:
 *   wifi_auth - ESP Wi-Fi authenticate mode.
 *
 * Returned Value:
 *   int - authenticate mode.
 *
 ****************************************************************************/

static int esp_wifi_auth_trans(uint32_t wifi_auth)
{
  int auth_mode = IW_AUTH_WPA_VERSION_DISABLED;

  switch (wifi_auth)
    {
      case WIFI_AUTH_OPEN:
        auth_mode = IW_AUTH_WPA_VERSION_DISABLED;
        break;

      case WIFI_AUTH_WPA_PSK:
        auth_mode = IW_AUTH_WPA_VERSION_WPA;
        break;

      case WIFI_AUTH_WPA2_PSK:
      case WIFI_AUTH_WPA_WPA2_PSK:
        auth_mode = IW_AUTH_WPA_VERSION_WPA2;
        break;

      case WIFI_AUTH_WPA3_PSK:
      case WIFI_AUTH_WPA2_WPA3_PSK:
        auth_mode = IW_AUTH_WPA_VERSION_WPA3;
        break;

      default:
        wlerr("Failed to transfer wireless authmode: %ld", wifi_auth);
        break;
    }

  return auth_mode;
}

/****************************************************************************
 * Name: esp_wifi_cipher_trans
 *
 * Description:
 *   Converts a ESP32-C6 cipher type values to WEXT cipher type values.
 *
 * Input Parameters:
 *   wifi_cipher - ESP Wi-Fi cipher type.
 *
 * Returned Value:
 *   cipher type.
 *
 ****************************************************************************/

static int esp_wifi_cipher_trans(uint32_t wifi_cipher)
{
  int cipher_mode = IW_AUTH_CIPHER_NONE;

  switch (wifi_cipher)
    {
      case WIFI_CIPHER_TYPE_NONE:
        cipher_mode = IW_AUTH_CIPHER_NONE;
        break;

      case WIFI_CIPHER_TYPE_WEP40:
        cipher_mode = IW_AUTH_CIPHER_WEP40;
        break;

      case WIFI_CIPHER_TYPE_WEP104:
        cipher_mode = IW_AUTH_CIPHER_WEP104;
        break;

      case WIFI_CIPHER_TYPE_TKIP:
        cipher_mode = IW_AUTH_CIPHER_TKIP;
        break;

      case WIFI_CIPHER_TYPE_CCMP:
      case WIFI_CIPHER_TYPE_TKIP_CCMP:
        cipher_mode = IW_AUTH_CIPHER_CCMP;
        break;

      case WIFI_CIPHER_TYPE_AES_CMAC128:
        cipher_mode = IW_AUTH_CIPHER_AES_CMAC;
        break;

      default:
        wlerr("Failed to transfer wireless authmode: %ld",
              wifi_cipher);
        break;
    }

  return cipher_mode;
}

/****************************************************************************
 * Name: esp_wifi_sta_auth
 *
 * Description:
 *   Set/Get station authentication mode params.
 *   Setting will disconnect from the current AP.
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific).
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_auth(struct iwreq *iwr, bool set)
{
  int ret;
  int cmd;
  wifi_config_t wifi_cfg;
  wifi_ap_record_t ap_info;

  esp_wifi_get_config(WIFI_IF_STA, &wifi_cfg);

  if (set)
    {
      cmd = iwr->u.param.flags & IW_AUTH_INDEX;
      switch (cmd)
        {
          case IW_AUTH_WPA_VERSION:
            {
              switch (iwr->u.param.value)
                {
                  case IW_AUTH_WPA_VERSION_DISABLED:
                      wifi_cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;
                      wlinfo("set authmode to WIFI_AUTH_OPEN\n");
                      break;

                  case IW_AUTH_WPA_VERSION_WPA:
                      wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA_PSK;
                      wlinfo("set authmode to WIFI_AUTH_WPA_PSK\n");
                      break;

                  case IW_AUTH_WPA_VERSION_WPA2:
                      wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
                      wlinfo("set authmode to WIFI_AUTH_WPA2_PSK\n");
                      break;

                  case IW_AUTH_WPA_VERSION_WPA3:
                      wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA3_PSK;
                      wlinfo("set authmode to WIFI_AUTH_WPA3_PSK\n");
                      break;

                  default:
                      wlerr("Invalid wpa version %" PRId32 "\n",
                          iwr->u.param.value);
                      return -EINVAL;
                }
            }
            break;

          case IW_AUTH_CIPHER_PAIRWISE:
          case IW_AUTH_CIPHER_GROUP:
            {
              switch (iwr->u.param.value)
                {
                  case IW_AUTH_CIPHER_NONE:
                      wifi_cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;
                      wlinfo("set authmode to WIFI_AUTH_OPEN\n");
                      break;

                  case IW_AUTH_CIPHER_WEP40:
                  case IW_AUTH_CIPHER_WEP104:
                      wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WEP;
                      wlinfo("set authmode to WIFI_AUTH_WEP\n");
                      break;

                  case IW_AUTH_CIPHER_TKIP:
                  case IW_AUTH_CIPHER_CCMP:
                  case IW_AUTH_CIPHER_AES_CMAC:
                      break;

                  default:
                      wlerr("Invalid cipher mode %" PRId32 "\n",
                            iwr->u.param.value);
                      return -EINVAL;
                }
            }
            break;

          case IW_AUTH_KEY_MGMT:
          case IW_AUTH_TKIP_COUNTERMEASURES:
          case IW_AUTH_DROP_UNENCRYPTED:
          case IW_AUTH_80211_AUTH_ALG:
          case IW_AUTH_WPA_ENABLED:
          case IW_AUTH_RX_UNENCRYPTED_EAPOL:
          case IW_AUTH_ROAMING_CONTROL:
          case IW_AUTH_PRIVACY_INVOKED:
          default:
              wlerr("Unknown cmd %d\n", cmd);
              return -EINVAL;
          }

      ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
      if (ret != 0)
        {
          wlerr("Failed to set Wi-Fi config data ret=%d\n", ret);
          return esp_wifi_to_errno(ret);
        }
    }
  else
    {
      ret = esp_wifi_sta_get_ap_info(&ap_info);
      if (ret != 0)
        {
          wlerr("Failed to get AP record ret=%d\n", ret);
          return esp_wifi_to_errno(ret);
        }

      cmd = iwr->u.param.flags & IW_AUTH_INDEX;
      switch (cmd)
        {
          case IW_AUTH_WPA_VERSION:
              iwr->u.param.value = esp_wifi_auth_trans(ap_info.authmode);
              break;

          case IW_AUTH_CIPHER_PAIRWISE:
              iwr->u.param.value =
                esp_wifi_cipher_trans(ap_info.pairwise_cipher);
              break;

          case IW_AUTH_CIPHER_GROUP:
              iwr->u.param.value =
                esp_wifi_cipher_trans(ap_info.group_cipher);
              break;

          case IW_AUTH_KEY_MGMT:
          case IW_AUTH_TKIP_COUNTERMEASURES:
          case IW_AUTH_DROP_UNENCRYPTED:
          case IW_AUTH_80211_AUTH_ALG:
          case IW_AUTH_WPA_ENABLED:
          case IW_AUTH_RX_UNENCRYPTED_EAPOL:
          case IW_AUTH_ROAMING_CONTROL:
          case IW_AUTH_PRIVACY_INVOKED:
          default:
              wlerr("Unknown cmd %d\n", cmd);
              return -ENOSYS;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: esp_wifi_sta_freq
 *
 * Description:
 *   Set/Get station frequency.
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success; Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_freq(struct iwreq *iwr, bool set)
{
  int ret;
  wifi_config_t wifi_cfg;

  esp_wifi_get_config(WIFI_IF_STA, &wifi_cfg);

  if (set && (iwr->u.freq.flags == IW_FREQ_FIXED))
    {
      wifi_cfg.sta.channel = esp_freq_to_channel(iwr->u.freq.m);

      ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
      if (ret != 0)
        {
          wlerr("Failed to set Wi-Fi config data ret=%d\n", ret);
          return esp_wifi_to_errno(ret);
        }

      ret = esp_wifi_sta_disconnect(true);
      if (ret != 0)
        {
          wlerr("Failed to disconnect from Wi-Fi AP ret=%d\n", ret);
          return ret;
        }
    }
  else
    {
      wifi_ap_record_t ap_info;

      ret = esp_wifi_sta_get_ap_info(&ap_info);
      if (ret != 0)
        {
          wlerr("Failed to get AP record ret=%d\n", ret);
          return esp_wifi_to_errno(ret);
        }

      iwr->u.freq.flags = IW_FREQ_FIXED;
      iwr->u.freq.e     = 0;
      iwr->u.freq.m     = 2407 + 5 * ap_info.primary;
    }

  return OK;
}

/****************************************************************************
 * Name: esp_wifi_sta_bitrate
 *
 * Description:
 *   Get station default bit rate (Mbps).
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success; Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_bitrate(struct iwreq *iwr, bool set)
{
  int ret;
  wifi_ap_record_t ap_info;

  if (set)
    {
      return -ENOSYS;
    }
  else
    {
      ret = esp_wifi_sta_get_ap_info(&ap_info);
      if (ret != 0)
        {
          wlerr("Failed to get AP record ret=%d\n", ret);
          return esp_wifi_to_errno(ret);
        }

      iwr->u.bitrate.fixed = IW_FREQ_FIXED;
      if (ap_info.phy_11n)
        {
          if (ap_info.second)
            {
              iwr->u.bitrate.value = ESP_WIFI_11N_MCS7_HT40_BITRATE;
            }
          else
            {
              iwr->u.bitrate.value = ESP_WIFI_11N_MCS7_HT20_BITRATE;
            }
        }
      else if (ap_info.phy_11g)
        {
          iwr->u.bitrate.value = ESP_WIFI_11G_MAX_BITRATE;
        }
      else if (ap_info.phy_11b)
        {
          iwr->u.bitrate.value = ESP_WIFI_11B_MAX_BITRATE;
        }
      else
        {
          return -EIO;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: esp_wifi_sta_rssi
 *
 * Description:
 *   Get Wi-Fi sensitivity (dBm).
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success; Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_rssi(struct iwreq *iwr, bool set)
{
  int ret;
  wifi_ap_record_t ap_info;

  if (set)
    {
      wlwarn("WARN: rssi set is not supported\n");
      return -ENOSYS;
    }

  ret = esp_wifi_sta_get_ap_info(&ap_info);
  if (ret != 0)
    {
      wlerr("Failed to get AP record ret=%d "
              "Make sure you are connected to an AP\n", ret);
      return esp_wifi_to_errno(ret);
    }

  iwr->u.sens.value = -(ap_info.rssi);

  return OK;
}

/****************************************************************************
 * Name: esp_wifi_sta_txpower
 *
 * Description:
 *   Get/Set station transmit power (dBm).
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success; Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_txpower(struct iwreq *iwr, bool set)
{
  return esp_wifi_txpower(iwr, set);
}

#endif /* ESP_WLAN_HAS_STA */

/****************************************************************************
 * Name: esp_wifi_sta_channel
 *
 * Description:
 *   Get station range of channel parameters.
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success; Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_channel(struct iwreq *iwr, bool set)
{
  return esp_wifi_channel(iwr, set);
}

/****************************************************************************
 * Name: esp_wifi_sta_country
 *
 * Description:
 *   Configure country info.
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success; Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_country(struct iwreq *iwr, bool set)
{
  return esp_wifi_country(iwr, set);
}

/****************************************************************************
 * Wi-Fi SoftAP functions
 ****************************************************************************/

#ifdef ESP_WLAN_HAS_SOFTAP

/****************************************************************************
 * Name: esp_wifi_softap_send_data
 *
 * Description:
 *   Use Wi-Fi SoftAP interface to send 802.3 frame
 *
 * Input Parameters:
 *   pbuf - Packet buffer pointer
 *   len  - Packet length
 *
 * Returned Value:
 *   OK on success; Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_send_data(void *pbuf, size_t len)
{
  int ret;

  ret = esp_wifi_internal_tx(WIFI_IF_AP, pbuf, len);
  if (ret != 0)
    {
      wlerr("Failed to send Wi-Fi data ret=%d\n", ret);
      ret = esp_wifi_to_errno(ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: esp_wifi_softap_read_mac
 *
 * Description:
 *   Read softAP interface MAC address from efuse
 *
 * Input Parameters:
 *   mac  - MAC address buffer pointer
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int esp_wifi_softap_read_mac(uint8_t *mac)
{
  int ret = esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
  return esp_wifi_to_errno(ret);
}

/****************************************************************************
 * Name: esp_wifi_softap_password
 *
 * Description:
 *   Set/Get Wi-Fi SoftAP password
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set   - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success; Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_password(struct iwreq *iwr, bool set)
{
  int ret;
  int size;
  wifi_config_t wifi_cfg;
  struct iw_encode_ext *ext = iwr->u.encoding.pointer;
  uint8_t *pdata;
  uint8_t len;
#ifdef CONFIG_DEBUG_WIRELESS_INFO
  char buf[PWD_MAX_LEN + 1];
#endif

  DEBUGASSERT(ext != NULL);

  pdata = ext->key;
  len = ext->key_len;

  if (set && len > PWD_MAX_LEN)
    {
      return -EINVAL;
    }

  pdata = ext->key;
  len   = ext->key_len;

  esp_wifi_get_config(WIFI_IF_AP, &wifi_cfg);

  if (set)
    {
      /* Clear the password field and copy the user password to it */

      memset(wifi_cfg.ap.password, 0x0, PWD_MAX_LEN);

      if (ext->alg != IW_ENCODE_ALG_NONE)
        {
          memcpy(wifi_cfg.ap.password, pdata, len);
        }

      ret = esp_wifi_set_config(WIFI_IF_AP, &wifi_cfg);
      if (ret != 0)
        {
          wlerr("Failed to set Wi-Fi config data ret=%d\n", ret);
          return esp_wifi_to_errno(ret);
        }
    }
  else
    {
      size = strnlen((char *)wifi_cfg.ap.password, PWD_MAX_LEN);
      if (len < size)
        {
          return -EINVAL;
        }
      else
        {
          len = size;
          memcpy(pdata, wifi_cfg.ap.password, len);
        }
    }

#ifdef CONFIG_DEBUG_WIRELESS_INFO
  memcpy(buf, pdata, len);
  buf[len] = 0;
  wlinfo("Wi-Fi SoftAP password=%s len=%d\n", buf, len);
#endif

  return OK;
}

/****************************************************************************
 * Name: esp_wifi_softap_essid
 *
 * Description:
 *   Set/Get Wi-Fi SoftAP ESSID
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success; Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_essid(struct iwreq *iwr, bool set)
{
  int ret;
  int size;
  wifi_config_t wifi_cfg;
  struct iw_point *essid = &iwr->u.essid;
  uint8_t *pdata;
  uint8_t len;
#ifdef CONFIG_DEBUG_WIRELESS_INFO
  char buf[IW_ESSID_MAX_SIZE + 1];
#endif

  DEBUGASSERT(essid != NULL);

  pdata = essid->pointer;
  len   = essid->length;

  if (set && len > IW_ESSID_MAX_SIZE)
    {
      return -EINVAL;
    }

  esp_wifi_get_config(WIFI_IF_AP, &wifi_cfg);

  if (set)
    {
      memset(wifi_cfg.ap.ssid, 0x0, IW_ESSID_MAX_SIZE);
      memcpy(wifi_cfg.ap.ssid, pdata, len);
      wifi_cfg.ap.ssid_len = len;
      ret = esp_wifi_set_config(WIFI_IF_AP, &wifi_cfg);
      if (ret != 0)
        {
          wlerr("Failed to set Wi-Fi config data ret=%d\n", ret);
          return esp_wifi_to_errno(ret);
        }
    }
  else
    {
      size = strnlen((char *)wifi_cfg.ap.ssid, IW_ESSID_MAX_SIZE);
      if (len < size)
        {
          return -EINVAL;
        }
      else
        {
          len = size;
          memcpy(pdata, wifi_cfg.ap.ssid, len);
        }
    }

#ifdef CONFIG_DEBUG_WIRELESS_INFO
  memcpy(buf, pdata, len);
  buf[len] = 0;
  wlinfo("Wi-Fi SoftAP ssid=%s len=%d\n", buf, len);
#endif

  return OK;
}

/****************************************************************************
 * Name: esp_wifi_softap_bssid
 *
 * Description:
 *   Set/Get Wi-Fi SoftAP BSSID
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_bssid(struct iwreq *iwr, bool set)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: esp_wifi_softap_connect
 *
 * Description:
 *   Trigger Wi-Fi SoftAP accept connection action.
 *
 * Input Parameters:
 *   config - The Wi-Fi config to set.
 *
 * Returned Value:
 *   OK on success; Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_connect()
{
  return OK;
}

/****************************************************************************
 * Name: esp_wifi_softap_disconnect
 *
 * Description:
 *   Trigger Wi-Fi SoftAP drop connection action
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_disconnect(void)
{
  return OK;
}

/****************************************************************************
 * Name: esp_wifi_softap_mode
 *
 * Description:
 *   Set/Get Wi-Fi SoftAP mode code.
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success; Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_mode(struct iwreq *iwr, bool set)
{
  if (!set)
    {
      iwr->u.mode = IW_MODE_MASTER;
    }

  return OK;
}

/****************************************************************************
 * Name: esp_wifi_softap_auth
 *
 * Description:
 *   Set/get authentication mode params.
 *
 *   Note: Authmode threshold resets to WPA2 as default if password matches
 *   WPA2 standards (password len => 8).
 *   If you want to connect the device to deprecated WEP/WPA networks, set
 *   the threshold value to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the
 *   password with length and format matching to WEP/WPA standards.
 *
 *   Note 2: WAPI blocks passwords below 8 characters. WEP is not supported.
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_auth(struct iwreq *iwr, bool set)
{
  int ret;
  int cmd;
  wifi_config_t wifi_cfg;

  esp_wifi_get_config(WIFI_IF_AP, &wifi_cfg);

  if (set)
    {
      cmd = iwr->u.param.flags & IW_AUTH_INDEX;
      switch (cmd)
        {
          case IW_AUTH_WPA_VERSION:
            {
              switch (iwr->u.param.value)
                {
                  case IW_AUTH_WPA_VERSION_DISABLED:
                      wifi_cfg.ap.authmode = WIFI_AUTH_OPEN;
                      wlinfo("set authmode to WIFI_AUTH_OPEN\n");
                      break;

                  case IW_AUTH_WPA_VERSION_WPA:
                      wifi_cfg.ap.authmode = WIFI_AUTH_WPA_PSK;
                      wlinfo("set authmode to WIFI_AUTH_WPA_PSK\n");
                      break;

                  case IW_AUTH_WPA_VERSION_WPA2:
                      wifi_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
                      wlinfo("set authmode to WIFI_AUTH_WPA2_PSK\n");
                      break;

                  case IW_AUTH_WPA_VERSION_WPA3:
                      wifi_cfg.ap.pmf_cfg.required = true;
                      wifi_cfg.ap.pmf_cfg.capable = false;
                      wifi_cfg.ap.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;
                      wifi_cfg.ap.authmode = WIFI_AUTH_WPA3_PSK;
                      wlinfo("set authmode to WIFI_AUTH_WPA3_PSK\n");
                      break;

                  default:
                      wlerr("Invalid wpa version %" PRId32 "\n",
                            iwr->u.param.value);
                      return -EINVAL;
                }
            }
              break;

          case IW_AUTH_CIPHER_PAIRWISE:
          case IW_AUTH_CIPHER_GROUP:
            {
              switch (iwr->u.param.value)
                {
                  case IW_AUTH_CIPHER_NONE:
                      wifi_cfg.ap.pairwise_cipher = WIFI_CIPHER_TYPE_NONE;
                      wlinfo("set pairwise_cipher to " \
                             "WIFI_CIPHER_TYPE_NONE\n");
                      break;

                  case IW_AUTH_CIPHER_WEP40:
                      wifi_cfg.ap.pairwise_cipher = WIFI_CIPHER_TYPE_WEP40;
                      wlinfo("set pairwise_cipher to " \
                             "WIFI_CIPHER_TYPE_WEP40\n");
                      break;

                  case IW_AUTH_CIPHER_WEP104:
                      wifi_cfg.ap.pairwise_cipher = WIFI_CIPHER_TYPE_WEP104;
                      wlinfo("set pairwise_cipher to " \
                             "WIFI_CIPHER_TYPE_WEP104\n");
                      break;

                  case IW_AUTH_CIPHER_TKIP:
                      wifi_cfg.ap.pairwise_cipher = WIFI_CIPHER_TYPE_TKIP;
                      wlinfo("set pairwise_cipher to " \
                             "WIFI_CIPHER_TYPE_TKIP\n");
                      break;

                  case IW_AUTH_CIPHER_CCMP:
                      wifi_cfg.ap.pairwise_cipher = WIFI_CIPHER_TYPE_CCMP;
                      wlinfo("set pairwise_cipher to " \
                             "WIFI_CIPHER_TYPE_CCMP\n");
                      break;

                  case IW_AUTH_CIPHER_AES_CMAC:
                      wifi_cfg.ap.pairwise_cipher =
                        WIFI_CIPHER_TYPE_AES_CMAC128;
                      wlinfo("set pairwise_cipher to " \
                             "WIFI_CIPHER_TYPE_AES_CMAC128\n");
                      break;

                  default:
                      wlerr("Invalid cipher mode %" PRId32 "\n",
                            iwr->u.param.value);
                      return -EINVAL;
                }
            }
              break;

          case IW_AUTH_KEY_MGMT:
          case IW_AUTH_TKIP_COUNTERMEASURES:
          case IW_AUTH_DROP_UNENCRYPTED:
          case IW_AUTH_80211_AUTH_ALG:
          case IW_AUTH_WPA_ENABLED:
          case IW_AUTH_RX_UNENCRYPTED_EAPOL:
          case IW_AUTH_ROAMING_CONTROL:
          case IW_AUTH_PRIVACY_INVOKED:
          default:
              wlerr("Unknown cmd %d\n", cmd);
              return -EINVAL;
        }

      ret = esp_wifi_set_config(WIFI_IF_AP, &wifi_cfg);
      if (ret != 0)
        {
          wlerr("Failed to set Wi-Fi config data ret=%d\n", ret);
          return esp_wifi_to_errno(ret);
        }
    }
  else
    {
      return -ENOSYS;
    }

  return ret;
}

/****************************************************************************
 * Name: esp_wifi_softap_freq
 *
 * Description:
 *   Set/Get SoftAP frequency.
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_freq(struct iwreq *iwr, bool set)
{
  int ret;
  wifi_config_t wifi_cfg;

  esp_wifi_get_config(WIFI_IF_AP, &wifi_cfg);

  if (set)
    {
      int channel = esp_freq_to_channel(iwr->u.freq.m);
      wifi_cfg.ap.channel = channel;

      ret = esp_wifi_set_config(WIFI_IF_AP, &wifi_cfg);
      if (ret != 0)
        {
          wlerr("Failed to set Wi-Fi config data ret=%d\n", ret);
          return esp_wifi_to_errno(ret);
        }
    }
  else
    {
      iwr->u.freq.flags = IW_FREQ_FIXED;
      iwr->u.freq.e     = 0;
      iwr->u.freq.m     = 2407 + 5 * wifi_cfg.ap.channel;
    }

  return OK;
}

/****************************************************************************
 * Name: esp_wifi_softap_get_bitrate
 *
 * Description:
 *   Get SoftAP default bit rate (Mbps).
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success; Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_bitrate(struct iwreq *iwr, bool set)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: esp_wifi_softap_txpower
 *
 * Description:
 *   Get SoftAP transmit power (dBm).
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success; Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_txpower(struct iwreq *iwr, bool set)
{
  return esp_wifi_txpower(iwr, set);
}

/****************************************************************************
 * Name: esp_wifi_softap_channel
 *
 * Description:
 *   Get SoftAP range of channel parameters.
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success; Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_channel(struct iwreq *iwr, bool set)
{
  return esp_wifi_channel(iwr, set);
}

/****************************************************************************
 * Name: esp_wifi_softap_country
 *
 * Description:
 *   Configure country info.
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success; Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_country(struct iwreq *iwr, bool set)
{
  return esp_wifi_country(iwr, set);
}

/****************************************************************************
 * Name: esp_wifi_softap_rssi
 *
 * Description:
 *   Get Wi-Fi sensitivity (dBm).
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success; Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_rssi(struct iwreq *iwr, bool set)
{
  return -ENOSYS;
}

#endif /* ESP_WLAN_HAS_SOFTAP */
