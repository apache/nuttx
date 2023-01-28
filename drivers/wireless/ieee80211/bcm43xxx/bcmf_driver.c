/****************************************************************************
 * drivers/wireless/ieee80211/bcm43xxx/bcmf_driver.c
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
#include <nuttx/compiler.h>

#include <inttypes.h>
#include <stdint.h>
#include <string.h>
#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>

#include <netinet/arp.h>
#include <net/ethernet.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/wdog.h>
#include <nuttx/sdio.h>
#include <nuttx/wireless/ieee80211/ieee80211.h>

#include "bcmf_driver.h"
#include "bcmf_cdc.h"
#include "bcmf_ioctl.h"
#include "bcmf_utils.h"
#include "bcmf_netdev.h"
#include "bcmf_core.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DOT11_BSSTYPE_ANY         2
#define BCMF_SCAN_TIMEOUT_TICK    (5*CLOCKS_PER_SEC)
#define BCMF_AUTH_TIMEOUT_MS      15000  /* was 10000 */
#define BCMF_SCAN_RESULT_ENTRIES  CONFIG_IEEE80211_BROADCOM_SCAN_RESULT_ENTRIES

/* CLM file is cut into pieces of MAX_CHUNK_LEN.
 *
 * NOTE:  CONFIG_NET_ETH_PKTSIZE is the MTU plus the size of the Ethernet
 * header (14 bytes).
 */

#define MAX_CHUNK_LEN \
  (CONFIG_NET_ETH_PKTSIZE > 1514 ? 1400 : CONFIG_NET_ETH_PKTSIZE - 114)

/* Helper to get iw_event size */

#define BCMF_IW_EVENT_SIZE(field) \
  (offsetof(struct iw_event, u) + sizeof(((union iwreq_data *)0)->field))

/* CLM blob macros */

#define DLOAD_HANDLER_VER     1       /* Downloader version */
#define DLOAD_FLAG_VER_MASK   0xf000  /* Downloader version mask */
#define DLOAD_FLAG_VER_SHIFT  12      /* Downloader version shift */

#define DL_CRC_NOT_INUSE      0x0001
#define DL_BEGIN              0x0002
#define DL_END                0x0004

#define WPA_OUI_LEN           3               /* WPA OUI length */
#define WPA_VERSION           1               /* WPA version */
#define WPA_VERSION_LEN       2               /* WPA version length */
#define WLAN_WPA_OUI          0xf25000
#define WLAN_WPA_OUI_TYPE     0x01
#define WLAN_WPA_SEL(x)       (((x) << 24) | WLAN_WPA_OUI)
#define WLAN_AKM_PSK          0x02
#define SUITE(oui, id)        (((oui) << 8) | (id))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* CLM blob download head */

struct wl_dload_data
{
  uint16_t flag;
  uint16_t dload_type;
  uint32_t len;
  uint32_t crc;
};

/* AP scan state machine status */

enum
{
  BCMF_SCAN_TIMEOUT = 0,
  BCMF_SCAN_DISABLED,
  BCMF_SCAN_RUN,
  BCMF_SCAN_DONE
};

/* Generic download types & flags */

enum
{
  DL_TYPE_UCODE = 1,
  DL_TYPE_CLM = 2
};

begin_packed_struct struct wpa_cipher_suite
{
  uint8_t oui[3];
  uint8_t type;
} end_packed_struct;

typedef struct wpa_cipher_suite wpa_cipher_suite_t;

begin_packed_struct struct wpa_rsn
{
  uint16_t            version;
  wpa_cipher_suite_t  group;
  uint16_t            scount;
  wpa_cipher_suite_t  pairwise[0];
} end_packed_struct;

typedef struct wpa_rsn wpa_rsn_t;

begin_packed_struct struct wpa_akm
{
  uint16_t            scount;
  wpa_cipher_suite_t  suite[0];
} end_packed_struct;

typedef struct wpa_akm wpa_akm_t;

begin_packed_struct struct wpa_ie_fixed
{
  uint8_t tag;                  /* TAG */
  uint8_t length;               /* TAG length */
  uint8_t oui[3];               /* IE OUI */
  uint8_t oui_type;             /* OUI type */
  begin_packed_struct struct
    {
      uint8_t low;
      uint8_t high;
    }
  end_packed_struct version;    /* IE version */
} end_packed_struct;

typedef struct wpa_ie_fixed wpa_ie_fixed_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_IEEE80211_BROADCOM_HAVE_CLM
static int bcmf_driver_download_clm(FAR struct bcmf_dev_s *priv);
#endif

/* FIXME only for debug purpose */

static void bcmf_wl_radio_event_handler(FAR struct bcmf_dev_s *priv,
                                        FAR struct bcmf_event_s *event,
                                        unsigned int len);

static void bcmf_wl_scan_event_handler(FAR struct bcmf_dev_s *priv,
                                       FAR struct bcmf_event_s *event,
                                       unsigned int len);

static void bcmf_wl_auth_event_handler(FAR struct bcmf_dev_s *priv,
                                       FAR struct bcmf_event_s *event,
                                       unsigned int len);

static int bcmf_wl_get_interface(FAR struct bcmf_dev_s *priv,
                                 FAR struct iwreq *iwr);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bcmf_wl_channel_to_frequency
 ****************************************************************************/

static int bcmf_wl_channel_to_frequency(int chan)
{
  if (chan <= 0)
    {
      return 0;
    }
  else if (chan < 14)
    {
      return 2407 + chan * 5;
    }
  else if (chan == 14)
    {
      return 2484;
    }
  else if ((chan >= 36) && (chan <= 165))
    {
      return 5000 + chan * 5;
    }
  else if ((chan >= 182) && (chan <= 196))
    {
      return 4000 + chan * 5;
    }

  return 0; /* not supported */
}

/****************************************************************************
 * Name: bcmf_wl_set_mac_address
 ****************************************************************************/

int bcmf_wl_set_mac_address(FAR struct bcmf_dev_s *priv, struct ifreq *req)
{
  int ret;
  uint32_t out_len = IFHWADDRLEN;

  ret = bcmf_cdc_iovar_request(priv, CHIP_STA_INTERFACE, true,
                              IOVAR_STR_CUR_ETHERADDR,
                              (FAR uint8_t *)req->ifr_hwaddr.sa_data,
                              &out_len);
  if (ret != OK)
    {
      return ret;
    }

  wlinfo("MAC address updated %02X:%02X:%02X:%02X:%02X:%02X\n",
                req->ifr_hwaddr.sa_data[0], req->ifr_hwaddr.sa_data[1],
                req->ifr_hwaddr.sa_data[2], req->ifr_hwaddr.sa_data[3],
                req->ifr_hwaddr.sa_data[4], req->ifr_hwaddr.sa_data[5]);

  memcpy(priv->bc_dev.d_mac.ether.ether_addr_octet,
         req->ifr_hwaddr.sa_data, ETHER_ADDR_LEN);

  return OK;
}

/****************************************************************************
 * Name: bcmf_driver_download_clm
 ****************************************************************************/

#ifdef CONFIG_IEEE80211_BROADCOM_HAVE_CLM
#ifdef CONFIG_IEEE80211_BROADCOM_FWFILES
int bcmf_driver_download_clm(FAR struct bcmf_dev_s *priv)
{
  FAR uint8_t *downloadbuff;
  struct file finfo;
  ssize_t nread;
  uint16_t dl_flag;
  unsigned int datalen = 7222;
  int ret;

  wlinfo("Download %d bytes\n", datalen);

  ret = file_open(&finfo, CONFIG_IEEE80211_BROADCOM_FWCLMNAME,
                  O_RDONLY);
  if (ret < 0)
    {
      wlerr("ERROR: Failed to open the FILE MTD file\n", ret);
      return ret;
    }

  /* Divide CLM blob into chunks */

  downloadbuff = kmm_malloc(sizeof(struct wl_dload_data) + MAX_CHUNK_LEN);
  if (downloadbuff == NULL)
    {
      wlerr("ERROR:  Failed allocate memory for CLM data\n");
      ret = -ENOMEM;
      goto errout_with_file;
    }

  dl_flag = DL_BEGIN;
  do
    {
      FAR struct wl_dload_data *dlhead;
      unsigned int chunk_len;
      uint32_t out_len;

      chunk_len = datalen >= MAX_CHUNK_LEN ? MAX_CHUNK_LEN : datalen;

      nread = file_read(&finfo, downloadbuff + sizeof(struct wl_dload_data),
                        chunk_len);
      if (nread < 0)
        {
          ret = (int)nread;
          wlerr("ERROR: Failed to read CLM data: %d\n", ret);
          goto errout_with_buffer;
        }

      wlinfo("Read blob %d bytes on %d\n", nread, chunk_len);

      datalen -= chunk_len;
      if (datalen <= 0)
        {
          dl_flag |= DL_END;
        }

      /* CLM header */

      dlhead             = (FAR struct wl_dload_data *)downloadbuff;
      dlhead->flag       = (DLOAD_HANDLER_VER << DLOAD_FLAG_VER_SHIFT) |
                           dl_flag;
      dlhead->dload_type = DL_TYPE_CLM;
      dlhead->len        = chunk_len;
      dlhead->crc        = 0;

      out_len            = chunk_len + sizeof(struct wl_dload_data);
      out_len            = (out_len + 7) & ~0x7;

      ret = bcmf_cdc_iovar_request(priv, CHIP_STA_INTERFACE, true,
                                   IOVAR_STR_CLMLOAD, downloadbuff,
                                   &out_len);

      wlinfo("datalen=%d, ret=%d\n", datalen, ret);

      dl_flag &= (uint16_t)~DL_BEGIN;
    }
  while ((datalen > 0) && (ret == OK));

  wlinfo("Done writing blob");

errout_with_buffer:
  kmm_free(downloadbuff);

errout_with_file:
  file_close(&finfo);
  return ret;
}

#else
int bcmf_driver_download_clm(FAR struct bcmf_dev_s *priv)
{
  FAR bcmf_interface_dev_t *ibus = (FAR bcmf_interface_dev_t *)priv->bus;
  FAR uint8_t *srcbuff = ibus->chip->clm_blob_image;
  FAR uint8_t *downloadbuff;
  unsigned int datalen = *ibus->chip->clm_blob_image_size;
  uint16_t dl_flag;
  int ret = 0;

  if (srcbuff == NULL || datalen <= 0)
    {
      wlinfo("Skip CLM blob...\n");
      return 0;
    }
  else
    {
      wlinfo("Download %d bytes @ %p\n", datalen, srcbuff);
    }

  /* Divide CLM blob into chunks */

  downloadbuff = kmm_malloc(sizeof(struct wl_dload_data) + MAX_CHUNK_LEN);
  if (!downloadbuff)
    {
      wlerr("No memory for CLM data\n");
      return -ENOMEM;
    }

  dl_flag = DL_BEGIN;
  do
    {
      FAR struct wl_dload_data *dlhead;
      unsigned int chunk_len = datalen;
      uint32_t out_len;

      chunk_len = datalen >= MAX_CHUNK_LEN ? MAX_CHUNK_LEN : datalen;
      memcpy(downloadbuff + sizeof(struct wl_dload_data), srcbuff,
             chunk_len);
      datalen  -= chunk_len;
      srcbuff  += chunk_len;

      if (datalen <= 0)
        {
          dl_flag |= DL_END;
        }

      /* CLM header */

      dlhead             = (struct wl_dload_data *)downloadbuff;
      dlhead->flag       = (DLOAD_HANDLER_VER << DLOAD_FLAG_VER_SHIFT) |
                           dl_flag;
      dlhead->dload_type = DL_TYPE_CLM;
      dlhead->len        = chunk_len;
      dlhead->crc        = 0;

      out_len            = chunk_len + sizeof(struct wl_dload_data);
      out_len            = (out_len + 7) & ~0x7;

      ret = bcmf_cdc_iovar_request(priv, CHIP_STA_INTERFACE, true,
                                   IOVAR_STR_CLMLOAD, downloadbuff,
                                   &out_len);

      dl_flag &= (uint16_t)~DL_BEGIN;
    }
  while ((datalen > 0) && (ret == OK));

  kmm_free(downloadbuff);
  return ret;
}
#endif
#endif /* CONFIG_IEEE80211_BROADCOM_HAVE_CLM */

/****************************************************************************
 * Name: bcmf_wl_active
 ****************************************************************************/

int bcmf_wl_active(FAR struct bcmf_dev_s *priv, bool active)
{
  int interface = CHIP_STA_INTERFACE;
  uint8_t tmp_buf[64];
  uint32_t out_len;
  uint32_t value;
  int ret;

  ninfo("Entered\n");

  ret = bcmf_bus_interface_active(priv, active);
  if (ret != OK || !active)
    {
      return ret;
    }

  ninfo("Interface activated\n");

#ifdef CONFIG_IEEE80211_BROADCOM_HAVE_CLM
  /* Download CLM blob if needed */

  ret = bcmf_driver_download_clm(priv);
  if (ret != OK)
    {
      goto errout_in_sdio_active;
    }
#endif

  /* Disable TX Gloming feature */

  out_len = 4;
  *(FAR uint32_t *)tmp_buf = 0;
  ret = bcmf_cdc_iovar_request(priv, interface, true,
                               IOVAR_STR_TX_GLOM, tmp_buf,
                               &out_len);
  if (ret != OK)
    {
      goto errout_in_sdio_active;
    }

  /* Set default power save mode */

  out_len = 4;
  value   = PM_OFF;
  ret     = bcmf_cdc_ioctl(priv, interface, true, WLC_SET_PM,
                           (FAR uint8_t *)&value, &out_len);
  if (ret != OK)
    {
      goto errout_in_sdio_active;
    }

  /* Set the GMode to auto */

  out_len = 4;
  value = GMODE_AUTO;
  ret = bcmf_cdc_ioctl(priv, interface, true, WLC_SET_GMODE,
                       (FAR uint8_t *)&value, &out_len);
  if (ret != OK)
    {
      goto errout_in_sdio_active;
    }

  /* TODO configure roaming if needed. Disable for now */

  out_len = 4;
  value   = 1;
  ret     = bcmf_cdc_iovar_request(priv, interface, true,
                                   IOVAR_STR_ROAM_OFF,
                                   (FAR uint8_t *)&value,
                                   &out_len);
  if (ret != OK)
    {
      goto errout_in_sdio_active;
    }

  /* TODO configure EAPOL version to default */

  out_len = 8;
  ((FAR uint32_t *)tmp_buf)[0] = interface;
  ((FAR uint32_t *)tmp_buf)[1] = (uint32_t)-1;

  ret = bcmf_cdc_iovar_request(priv, interface, true,
                               "bsscfg:"IOVAR_STR_SUP_WPA2_EAPVER,
                               tmp_buf, &out_len);
  if (ret != OK)
    {
      goto errout_in_sdio_active;
    }

  /* Query firmware version string */

  out_len = sizeof(tmp_buf);
  ret     = bcmf_cdc_iovar_request(priv, interface, false,
                                   IOVAR_STR_VERSION, tmp_buf,
                                   &out_len);
  if (ret != OK)
    {
      goto errout_in_sdio_active;
    }

  tmp_buf[sizeof(tmp_buf)-1] = 0;

  /* Remove line feed */

  out_len = strlen((FAR char *)tmp_buf);
  if (out_len > 0 && tmp_buf[out_len - 1] == '\n')
    {
      tmp_buf[out_len - 1] = 0;
    }

  wlinfo("fw version <%s>\n", tmp_buf);

  ret = bcmf_event_push_config(priv);

errout_in_sdio_active:
  if (ret != OK)
    {
      bcmf_bus_interface_active(priv, false);
    }

  return ret;
}

/****************************************************************************
 * Name: bcmf_driver_initialize
 ****************************************************************************/

int bcmf_driver_initialize(FAR struct bcmf_dev_s *priv)
{
  /*  Register radio event */

  bcmf_event_register(priv, bcmf_wl_radio_event_handler, WLC_E_RADIO);

  /*  Register AP scan event */

  bcmf_event_register(priv, bcmf_wl_scan_event_handler, WLC_E_ESCAN_RESULT);

  /*  Register authentication related events */

  bcmf_event_register(priv, bcmf_wl_auth_event_handler,
                      WLC_E_ASSOC_IND_NDIS);
  bcmf_event_register(priv, bcmf_wl_auth_event_handler,
                      WLC_E_AUTH);
  bcmf_event_register(priv, bcmf_wl_auth_event_handler,
                      WLC_E_ASSOC);
  bcmf_event_register(priv, bcmf_wl_auth_event_handler,
                      WLC_E_LINK);
  bcmf_event_register(priv, bcmf_wl_auth_event_handler,
                      WLC_E_PSK_SUP);
  bcmf_event_register(priv, bcmf_wl_auth_event_handler,
                      WLC_E_JOIN);
  bcmf_event_register(priv, bcmf_wl_auth_event_handler,
                      WLC_E_SET_SSID);
  bcmf_event_register(priv, bcmf_wl_auth_event_handler,
                      WLC_E_DEAUTH_IND);
  bcmf_event_register(priv, bcmf_wl_auth_event_handler,
                      WLC_E_DISASSOC);
  bcmf_event_register(priv, bcmf_wl_auth_event_handler,
                      WLC_E_DISASSOC_IND);

  /* Register network driver */

  return bcmf_netdev_register(priv);
}

/****************************************************************************
 * Name: bcmf_wl_radio_event_handler
 ****************************************************************************/

void bcmf_wl_radio_event_handler(FAR struct bcmf_dev_s *priv,
                                 FAR struct bcmf_event_s *event,
                                 unsigned int len)
{
  wlinfo("Unhandled radio event from <%s>\n", event->src_name);
}

/****************************************************************************
 * Name: bcmf_wl_auth_event_handler
 ****************************************************************************/

void bcmf_wl_auth_event_handler(FAR struct bcmf_dev_s *priv,
                                FAR struct bcmf_event_s *event,
                                unsigned int len)
{
  bool auth = false;
  int carrier = -1;
  uint32_t reason;
  uint32_t status;
  uint32_t type;

  type = bcmf_getle32(&event->type);
  status = bcmf_getle32(&event->status);
  reason = bcmf_getle32(&event->reason);

  wlinfo("Got auth event %" PRId32 " status %" PRId32 " from <%s>\n",
         type, status, event->src_name);

  if (!priv->bc_bifup)
    {
      return;
    }

  bcmf_hexdump((uint8_t *)event, len, (unsigned long)event);

  if (type == WLC_E_PSK_SUP)
    {
      carrier = (reason == WLC_E_SUP_OTHER) ? 1 : 0;
      if (priv->auth_pending)
        {
          priv->auth_status = reason;
          auth = true;
        }
    }
  else if (type == WLC_E_SET_SSID)
    {
      carrier = (status == WLC_E_STATUS_SUCCESS) ? 1 : 0;
      if (!priv->auth_pending || !carrier)
        {
          priv->auth_status = status;
          auth = true;
        }
    }
  else if (type == WLC_E_DISASSOC)
    {
      carrier = 0;
    }

  if (carrier >= 0)
    {
      if (carrier)
        {
          netdev_carrier_on(&priv->bc_dev);
        }
      else
        {
          netdev_carrier_off(&priv->bc_dev);
        }
    }

  if (auth && priv->auth_signal)
    {
      nxsem_post(priv->auth_signal);
    }
}

/****************************************************************************
 * Name: bcmf_wl_scan_event_handler
 ****************************************************************************/

/* bcmf_wl_scan_event_handler must run at high priority else
 * race condition may occur on priv->scan_result field
 */

void bcmf_wl_scan_event_handler(FAR struct bcmf_dev_s *priv,
                                FAR struct bcmf_event_s *event,
                                unsigned int len)
{
  FAR struct wl_escan_result *result;
  unsigned int escan_result_len;
  FAR wl_bss_info_t *curr;
  FAR wl_bss_info_t *bss;
  unsigned int ie_offset;
  FAR uint8_t *ie_buffer;
  uint32_t event_len;
  int16_t worst_rssi;
  int worst_entry;
  uint32_t status;
  bool vaild_bss;
  int suitelen;
  int i;
  int j;

  event_len = len;

  if (priv->scan_status < BCMF_SCAN_RUN)
    {
      wlinfo("Got Unexpected scan event\n");
      goto exit_invalid_frame;
    }

  wlinfo("Scan event from <%s>\n", event->src_name);

  status = bcmf_getle32(&event->status);
  escan_result_len = bcmf_getle32(&event->len);

  len -= sizeof(struct bcmf_event_s);

  if (len > escan_result_len)
    {
      len = escan_result_len;
    }

  if (len == sizeof(struct wl_escan_result) - sizeof(struct wl_bss_info))
    {
      /* Nothing to process, may be scan done event */

      goto wl_escan_result_processed;
    }

  if (len < sizeof(struct wl_escan_result))
    {
      goto exit_invalid_frame;
    }

  /* Process escan result payload */

  result = (FAR struct wl_escan_result *)&event[1];

  if (len < result->buflen ||
      result->buflen < sizeof(struct wl_escan_result))
    {
      goto exit_invalid_frame;
    }

  /* Process bss_infos */

  for (i = 0; i < result->bss_count; i++)
    {
      bss = &result->bss_info[i];

      worst_entry = -1;
      worst_rssi = 0;
      vaild_bss = true;

      wlinfo("Scan result: <%.32s> "
             "%02x:%02x:%02x:%02x:%02x:%02x "
             "signal %d %d %d\n",
             bss->SSID,
             bss->BSSID.ether_addr_octet[0],
             bss->BSSID.ether_addr_octet[1],
             bss->BSSID.ether_addr_octet[2],
             bss->BSSID.ether_addr_octet[3],
             bss->BSSID.ether_addr_octet[4],
             bss->BSSID.ether_addr_octet[5],
             bss->RSSI, bss->phy_noise, bss->SNR);

      if (strnlen((FAR const char *)bss->SSID,
                   sizeof(bss->SSID)) == 0)
        {
          continue;
        }

      if (bss->ctl_ch == 0)
        {
          continue;
        }

      ie_offset = 0;
      ie_buffer = (FAR uint8_t *)bss + bss->ie_offset;

      while (1)
        {
          size_t ie_frame_size;

          if (bss->ie_length - ie_offset < 2)
            {
              /* Minimum Information element size is 2 bytes */

              break;
            }

          ie_frame_size = ie_buffer[ie_offset + 1] + 2;

          if (ie_frame_size > bss->ie_length - ie_offset)
            {
              /* Entry too big */

              break;
            }

          switch (ie_buffer[ie_offset])
            {
              case WLAN_EID_RSN:
                {
                  FAR wpa_rsn_t *rsn = (FAR wpa_rsn_t *)
                                       &ie_buffer[ie_offset + 2];
                  FAR wpa_akm_t *akm;

                  if (rsn->version != WPA_VERSION)
                    {
                      goto process_next_bss;
                    }

                  vaild_bss = false;

                  suitelen = sizeof(*rsn) + rsn->scount *
                             sizeof(wpa_cipher_suite_t);

                  if (ie_buffer[ie_offset + 1] > suitelen + 2)
                    {
                      akm = (FAR wpa_akm_t *)
                            &ie_buffer[ie_offset + suitelen + 2];
                      for (j = 0; j < akm->scount; j++)
                        {
                          uint32_t suite = ntohl
                                   (*(FAR uint32_t *)&akm->suite[j]);
                          if (suite == WLAN_AKM_SUITE_PSK)
                            {
                              goto vaild_bss;
                            }
                        }
                    }
                  break;
                }

              case WLAN_EID_VENDOR_SPECIFIC:
                {
                  FAR wpa_ie_fixed_t *ie = (FAR wpa_ie_fixed_t *)
                                           &ie_buffer[ie_offset];
                  FAR wpa_akm_t *akm;
                  FAR wpa_rsn_t *rsn;

                  /* WPA_OUI */

                  if (memcmp(&ie->oui[0], "\x00\x50\xf2\x01", 4))
                    {
                      break;
                    }

                  vaild_bss = false;

                  rsn = (FAR wpa_rsn_t *)&ie_buffer[ie_offset +
                        sizeof(wpa_ie_fixed_t) - 2];
                  suitelen = sizeof(wpa_ie_fixed_t) +
                             sizeof(*rsn) + rsn->scount *
                             sizeof(wpa_cipher_suite_t) - 2;
                  if (ie_buffer[ie_offset + 1] + 2 > suitelen)
                    {
                      akm = (FAR wpa_akm_t *)&ie_buffer[ie_offset +
                                                        suitelen];
                      for (j = 0; j < akm->scount; j++)
                        {
                          if (*(FAR uint32_t *)&akm->suite[j] ==
                              WLAN_WPA_SEL(WLAN_AKM_PSK))
                            {
                              goto vaild_bss;
                            }
                        }
                    }
                  break;
                }

              default:
                break;
            }

          ie_offset += ie_buffer[ie_offset + 1] + 2;
        }

      /* Check if AP is configured for WEP or unsupport privacy */

      if ((vaild_bss && (bss->capability & DOT11_CAP_PRIVACY)) || !vaild_bss)
        {
          goto process_next_bss;
        }

vaild_bss:

      for (j = 0; j < priv->scan_result_entries; j++)
        {
          curr = &priv->scan_result[j];

          /* Check if current bss AP is not already detected */

          if (memcmp(&curr->BSSID, &bss[i].BSSID,
                     sizeof(curr->BSSID)) == 0 ||
              memcmp(&curr->SSID, &bss[i].SSID,
                     sizeof(curr->SSID)) == 0)
            {
              /* Replace the duplicate entry if rssi is
               * better than before
               */

              if (curr->RSSI < bss[i].RSSI)
                {
                  memcpy(curr, bss, sizeof(*curr));
                }

              goto process_next_bss;
            }

          /* Find worst rssi and mark the entry */

          if (curr->RSSI < worst_rssi)
            {
              worst_entry = j;
              worst_rssi = curr->RSSI;
            }
        }

      if (priv->scan_result_entries == BCMF_SCAN_RESULT_ENTRIES)
        {
          /* Entries full and replace the worst entry */

          if (worst_entry >= 0)
            {
              curr = &priv->scan_result[worst_entry];
              if (curr->RSSI < bss->RSSI)
                {
                  memcpy(curr, bss, sizeof(*curr));
                }
            }

process_next_bss:
          continue;
        }

      curr = &priv->scan_result[priv->scan_result_entries];
      memcpy(curr, bss, sizeof(*curr));

      priv->scan_result_entries++;
    }

wl_escan_result_processed:
  if (status == WLC_E_STATUS_PARTIAL)
    {
      /* More frames to come */

      return;
    }

  if (status != WLC_E_STATUS_SUCCESS)
    {
      wlerr("Invalid event status %" PRId32 "\n", status);
      return;
    }

  /* Scan done */

  wlinfo("escan done event %" PRId32 " %" PRId32 "\n",
         status, bcmf_getle32(&event->reason));

  wd_cancel(&priv->scan_timeout);

  priv->scan_status = BCMF_SCAN_DONE;
  nxsem_post(&priv->control_mutex);

  return;

exit_invalid_frame:
  wlerr("Invalid scan result event\n");
  bcmf_hexdump((FAR uint8_t *)event, event_len, (unsigned long)event);
}

/****************************************************************************
 * Name: bcmf_wl_scan_format_results
 ****************************************************************************/

static int bcmf_wl_scan_format_results(FAR struct bcmf_dev_s *priv,
                                       FAR struct iwreq *iwr)
{
  FAR wl_bss_info_t *scan_result[BCMF_SCAN_RESULT_ENTRIES];
  FAR wl_bss_info_t *info;
  FAR struct iw_event *iwe;
  FAR char *pointer;
  int len;
  int i;
  int j;

  if (priv->scan_result_entries == 0)
    {
      iwr->u.data.length = 0;
      return OK;
    }

  len = IW_EV_LEN(ap_addr) + IW_EV_LEN(qual) +
        IW_EV_LEN(freq)    + IW_EV_LEN(data) +
        IW_EV_LEN(essid);

  len *= priv->scan_result_entries;

  for (i = 0; i < priv->scan_result_entries; i++)
    {
      scan_result[i] = &priv->scan_result[i];
      len += (min(strlen((FAR const char *)scan_result[i]->SSID),
                         32) + 3) & ~3;
    }

  if (iwr->u.data.pointer == NULL || iwr->u.data.length < len)
    {
      iwr->u.data.length = len;
      return -E2BIG;
    }

  /* Sort list by RSSI */

  for (i = 0; i < priv->scan_result_entries; i++)
    {
      for (j = 0; j + 1 < priv->scan_result_entries - i; j++)
        {
          if (scan_result[j]->RSSI < scan_result[j + 1]->RSSI)
            {
              info = scan_result[j];
              scan_result[j] = scan_result[j + 1];
              scan_result[j + 1] = info;
            }
        }
    }

  pointer = iwr->u.data.pointer;

  /* Copy scan result */

  for (i = 0; i < priv->scan_result_entries; i++)
    {
      info = scan_result[i];

      /* Copy BSSID */

      iwe = (FAR struct iw_event *)pointer;
      iwe->cmd = SIOCGIWAP;
      iwe->u.ap_addr.sa_family = ARPHRD_ETHER;
      memcpy(&iwe->u.ap_addr.sa_data,
             info->BSSID.ether_addr_octet, IFHWADDRLEN);
      iwe->len = IW_EV_LEN(ap_addr);
      pointer += iwe->len;

      /* Copy ESSID */

      iwe = (FAR struct iw_event *)pointer;
      iwe->cmd = SIOCGIWESSID;
      iwe->u.essid.flags = 0;
      iwe->u.essid.length = min(strlen((FAR const char *)info->SSID), 32);
      iwe->u.essid.pointer = (FAR void *)sizeof(iwe->u.essid);
      memcpy(&iwe->u.essid + 1, info->SSID, iwe->u.essid.length);
      iwe->len = IW_EV_LEN(essid) + ((iwe->u.essid.length + 3) & ~3);
      pointer += iwe->len;

      /* Copy link quality info */

      iwe = (FAR struct iw_event *)pointer;
      iwe->cmd = IWEVQUAL;
      iwe->u.qual.qual = info->SNR;
      iwe->u.qual.level = info->RSSI;
      iwe->u.qual.noise = info->phy_noise;
      iwe->u.qual.updated = IW_QUAL_DBM | IW_QUAL_ALL_UPDATED;
      iwe->len = IW_EV_LEN(qual);
      pointer += iwe->len;

      /* Copy AP control channel */

      iwe = (FAR struct iw_event *)pointer;
      iwe->cmd = SIOCGIWFREQ;
      iwe->u.freq.e = 0;
      iwe->u.freq.m = info->ctl_ch;
      iwe->len = IW_EV_LEN(freq);
      pointer += iwe->len;

      /* Copy AP encryption mode */

      iwe = (FAR struct iw_event *)pointer;
      iwe->cmd = SIOCGIWENCODE;
      iwe->u.data.flags = info->capability & DOT11_CAP_PRIVACY ?
                          IW_ENCODE_ENABLED | IW_ENCODE_NOKEY :
                          IW_ENCODE_DISABLED;
      iwe->u.data.length = 0;
      iwe->u.essid.pointer = NULL;
      iwe->len = IW_EV_LEN(data);
      pointer += iwe->len;
    }

  iwr->u.data.length = pointer - (FAR char *)iwr->u.data.pointer;
  return OK;
}

/****************************************************************************
 * Name: bcmf_wl_scan_timeout
 ****************************************************************************/

void bcmf_wl_scan_timeout(wdparm_t arg)
{
  FAR struct bcmf_dev_s *priv = (FAR struct bcmf_dev_s *)arg;

  if (priv->scan_status < BCMF_SCAN_RUN)
    {
      /* Fatal error, invalid scan status */

      wlerr("Unexpected scan timeout\n");
      return;
    }

  wlerr("Scan timeout detected\n");

  priv->scan_status = BCMF_SCAN_TIMEOUT;
  nxsem_post(&priv->control_mutex);
}

/****************************************************************************
 * Name: bcmf_wl_get_interface
 ****************************************************************************/

int bcmf_wl_get_interface(FAR struct bcmf_dev_s *priv, FAR struct iwreq *iwr)
{
  /* TODO resolve interface using iwr->ifr_name */

  return CHIP_STA_INTERFACE;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bcmf_allocate_device
 ****************************************************************************/

FAR struct bcmf_dev_s *bcmf_allocate_device(void)
{
  int ret;
  FAR struct bcmf_dev_s *priv;

  /* Allocate a bcmf device structure */

  priv = (FAR struct bcmf_dev_s *)kmm_malloc(sizeof(*priv));
  if (!priv)
    {
      return NULL;
    }

  /* Initialize bcmf device structure */

  memset(priv, 0, sizeof(*priv));

  /* Init control frames mutex and timeout signal */

  if ((ret = nxsem_init(&priv->control_mutex, 0, 1)) != OK)
    {
      goto exit_free_priv;
    }

  if ((ret = nxsem_init(&priv->control_timeout, 0, 0)) != OK)
    {
      goto exit_free_priv;
    }

  /* Init scan timeout timer */

  priv->scan_status = BCMF_SCAN_DISABLED;

  return priv;

exit_free_priv:
  kmm_free(priv);
  return NULL;
}

/****************************************************************************
 * Name: bcmf_free_device
 ****************************************************************************/

void bcmf_free_device(FAR struct bcmf_dev_s *priv)
{
  /* ## TODO ## deinitialize device structures */

  kmm_free(priv);
}

/****************************************************************************
 * Name: bcmf_wl_enable
 ****************************************************************************/

int bcmf_wl_enable(FAR struct bcmf_dev_s *priv, bool enable)
{
  int ret;
  uint32_t out_len;

  /* TODO check device state */

  out_len = 0;
  ret = bcmf_cdc_ioctl(priv, CHIP_STA_INTERFACE, true,
                       enable ? WLC_UP : WLC_DOWN, NULL, &out_len);

  /* TODO wait for WLC_E_RADIO event */

  nxsig_usleep(3000);

  if (ret == OK)
    {
      /* TODO update device state */
    }

  return ret;
}

/****************************************************************************
 * Name: bcmf_wl_start_scan
 *
 * Description:
 *   Start a WiFi scan.
 ****************************************************************************/

int bcmf_wl_start_scan(FAR struct bcmf_dev_s *priv, FAR struct iwreq *iwr)
{
  int ret;
  uint32_t out_len;
  uint32_t value;
  int interface;
  FAR struct iw_scan_req *req;
  struct wl_escan_params scan_params;

  interface = bcmf_wl_get_interface(priv, iwr);

  if (interface < 0)
    {
      return -EINVAL;
    }

  memset(&scan_params, 0, sizeof(scan_params));

  scan_params.version = ESCAN_REQ_VERSION;
  scan_params.action = WL_SCAN_ACTION_START;
  scan_params.sync_id = 0xabcd; /* Not used for now */

  memset(&scan_params.params.bssid, 0xff,
          sizeof(scan_params.params.bssid));

  scan_params.params.bss_type = DOT11_BSSTYPE_ANY;
  scan_params.params.nprobes = -1;
  scan_params.params.active_time = -1;
  scan_params.params.passive_time = -1;
  scan_params.params.home_time = -1;
  scan_params.params.channel_num = 0;

  if (iwr->u.data.pointer && iwr->u.data.length >= sizeof(*req))
    {
      req = (FAR struct iw_scan_req *)iwr->u.data.pointer;

      memcpy(&scan_params.params.bssid, req->bssid.sa_data,
             sizeof(scan_params.params.bssid));

      scan_params.params.scan_type =
                  req->scan_type == IW_SCAN_TYPE_ACTIVE ? 0:1;

      if (iwr->u.data.flags & IW_SCAN_THIS_ESSID &&
          req->essid_len < sizeof(scan_params.params.ssid.SSID))
        {
          /* Scan specific ESSID */

          memcpy(scan_params.params.ssid.SSID, req->essid, req->essid_len);
          scan_params.params.ssid.ssid_len = req->essid_len;
        }
    }
  else
    {
      /* Default scan parameters */

      wlinfo("Use default scan parameters\n");

      memset(&scan_params.params.bssid, 0xff,
             sizeof(scan_params.params.bssid));

      scan_params.params.scan_type = 0; /* Active scan */
    }

  /* Set active scan mode */

  value = scan_params.params.scan_type;
  out_len = 4;
  if (bcmf_cdc_ioctl(priv, CHIP_STA_INTERFACE, true,
                     WLC_SET_PASSIVE_SCAN, (FAR uint8_t *)&value, &out_len))
    {
      ret = -EIO;
      goto exit_failed;
    }

  /* Lock control_mutex semaphore */

  if ((ret = nxsem_wait_uninterruptible(&priv->control_mutex)) < 0)
    {
      goto exit_failed;
    }

  /* Allocate buffer to store scan result */

  if (priv->scan_result == NULL)
    {
      priv->scan_result = kmm_malloc(sizeof(wl_bss_info_t) *
                                     BCMF_SCAN_RESULT_ENTRIES);
      if (priv->scan_result == NULL)
        {
          wlerr("Cannot allocate result buffer\n");
          ret = -ENOMEM;
          goto exit_sem_post;
        }
    }

  wlinfo("start scan\n");

  priv->scan_result_entries = 0;
  priv->scan_status = BCMF_SCAN_RUN;

  out_len = sizeof(scan_params);

  if (bcmf_cdc_iovar_request_unsafe(priv, CHIP_STA_INTERFACE, true,
                                    IOVAR_STR_ESCAN,
                                    (FAR uint8_t *)&scan_params,
                                    &out_len))
    {
      ret = -EIO;
      goto exit_sem_post;
    }

  /*  Start scan_timeout timer */

  wd_start(&priv->scan_timeout, BCMF_SCAN_TIMEOUT_TICK,
           bcmf_wl_scan_timeout, (wdparm_t)priv);

  return OK;

exit_sem_post:
  priv->scan_status = BCMF_SCAN_DISABLED;
  nxsem_post(&priv->control_mutex);

  wlinfo("scan complete\n");

exit_failed:
  wlinfo("Failed\n");
  return ret;
}

/****************************************************************************
 * Name: bcmf_wl_get_scan_results
 *
 * Description:
 *   Get the results of a WiFi scan.
 ****************************************************************************/

int bcmf_wl_get_scan_results(FAR struct bcmf_dev_s *priv, struct iwreq *iwr)
{
  int ret = OK;

  if (priv->scan_status == BCMF_SCAN_RUN)
    {
      ret = -EAGAIN;
      goto exit_failed;
    }

  if (priv->scan_status != BCMF_SCAN_DONE)
    {
      ret = -EINVAL;
      goto exit_failed;
    }

  /* Lock control_mutex semaphore to avoid race condition */

  if ((ret = nxsem_wait_uninterruptible(&priv->control_mutex)) < 0)
    {
      goto exit_failed;
    }

  if (!priv->scan_result)
    {
      /* Result have already been requested */

      ret = OK;
      iwr->u.data.length = 0;
      goto exit_sem_post;
    }

  ret = bcmf_wl_scan_format_results(priv, iwr);
  if (ret == OK)
    {
      /* Free scan result buffer */

      kmm_free(priv->scan_result);
      priv->scan_result = NULL;
      priv->scan_result_entries = 0;
    }

exit_sem_post:
  nxsem_post(&priv->control_mutex);

exit_failed:
  if (ret < 0)
    {
      iwr->u.data.length = 0;
    }

  return ret;
}

/****************************************************************************
 * Name: bcmf_wl_set_auth_param
 *
 * Description:
 *   Set the authorization parameters for the device
 ****************************************************************************/

int bcmf_wl_set_auth_param(FAR struct bcmf_dev_s *priv, struct iwreq *iwr)
{
  int ret = -ENOSYS;
  int interface;
  uint32_t out_len;

  interface = bcmf_wl_get_interface(priv, iwr);

  if (interface < 0)
    {
      return -EINVAL;
    }

  switch (iwr->u.param.flags & IW_AUTH_INDEX)
    {
      case IW_AUTH_WPA_VERSION:
        {
          uint32_t wpa_version[2];
          uint32_t auth_mode;

          switch (iwr->u.param.value)
            {
              case IW_AUTH_WPA_VERSION_DISABLED:
                wpa_version[1] = 0;
                auth_mode = WPA_AUTH_DISABLED;
                break;

              case IW_AUTH_WPA_VERSION_WPA:
                wpa_version[1] = 1;
                auth_mode = WPA_AUTH_PSK;
                break;

              case IW_AUTH_WPA_VERSION_WPA2:
                wpa_version[1] = 1;
                auth_mode = WPA2_AUTH_PSK;
                break;

              default:
                wlerr("Invalid wpa version %" PRId32 "\n",
                      iwr->u.param.value);
                return -EINVAL;
            }

          out_len = 8;
          wpa_version[0] = interface;

          if (bcmf_cdc_iovar_request(priv, interface, true,
                                     "bsscfg:"IOVAR_STR_SUP_WPA,
                                     (FAR uint8_t *)wpa_version,
                                     &out_len))
            {
              return -EIO;
            }

          out_len = 4;
          if (bcmf_cdc_ioctl(priv, interface, true, WLC_SET_WPA_AUTH,
                             (FAR uint8_t *)&auth_mode, &out_len))
            {
              return -EIO;
            }
        }

        return OK;

      case IW_AUTH_CIPHER_PAIRWISE:
      case IW_AUTH_CIPHER_GROUP:
        {
          uint32_t cipher_mode;
          uint32_t wep_auth = 0;

          switch (iwr->u.param.value)
            {
              case IW_AUTH_CIPHER_NONE:
                cipher_mode = OPEN_AUTH;
                break;

              case IW_AUTH_CIPHER_WEP40:
              case IW_AUTH_CIPHER_WEP104:
                cipher_mode = WEP_ENABLED;
                wep_auth = 1;
                break;

              case IW_AUTH_CIPHER_TKIP:
                cipher_mode = TKIP_ENABLED;
                break;

              case IW_AUTH_CIPHER_CCMP:
                cipher_mode = AES_ENABLED;
                break;

              default:
                wlerr("Invalid cipher mode %" PRId32 "\n",
                      iwr->u.param.value);
                return -EINVAL;
            }

          out_len = 4;
          if (bcmf_cdc_ioctl(priv, interface, true,
                             WLC_SET_WSEC,
                             (FAR uint8_t *)&cipher_mode,
                             &out_len))
            {
              return -EIO;
            }

          /* Set authentication mode */

          out_len = 4;
          if (bcmf_cdc_ioctl(priv, interface, true,
                             WLC_SET_AUTH,
                             (FAR uint8_t *)&wep_auth,
                             &out_len))
            {
              return -EIO;
            }
        }

        return OK;

      case IW_AUTH_KEY_MGMT:
      case IW_AUTH_TKIP_COUNTERMEASURES:
      case IW_AUTH_DROP_UNENCRYPTED:
      case IW_AUTH_80211_AUTH_ALG:
      case IW_AUTH_WPA_ENABLED:
      case IW_AUTH_RX_UNENCRYPTED_EAPOL:
      case IW_AUTH_ROAMING_CONTROL:
      case IW_AUTH_PRIVACY_INVOKED:
      default:
        wlerr("Unknown cmd %d\n", iwr->u.param.flags);
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: bcmf_wl_set_mode
 *
 * Description:
 *   Set the mode for the device
 ****************************************************************************/

int bcmf_wl_set_mode(FAR struct bcmf_dev_s *priv, struct iwreq *iwr)
{
  uint32_t out_len;
  uint32_t value;
  int interface;

  interface = bcmf_wl_get_interface(priv, iwr);

  if (interface < 0)
    {
      return -EINVAL;
    }

  out_len = sizeof(value);
  value = iwr->u.mode == IW_MODE_INFRA ? 1 : 0;

  return bcmf_cdc_ioctl(priv, interface, true,
                        WLC_SET_INFRA, (FAR uint8_t *)&value, &out_len);
}

/****************************************************************************
 * Name: bcmf_wl_get_mode
 *
 * Description:
 *   Get the mode for the device
 ****************************************************************************/

int bcmf_wl_get_mode(FAR struct bcmf_dev_s *priv, struct iwreq *iwr)
{
  uint32_t out_len;
  uint32_t infra;
  int interface;
  uint32_t ap;
  int ret;

  interface = bcmf_wl_get_interface(priv, iwr);

  if (interface < 0)
    {
      return -EINVAL;
    }

  out_len = sizeof(infra);
  ret = bcmf_cdc_ioctl(priv, interface, false,
                       WLC_GET_INFRA, (FAR uint8_t *)&infra, &out_len);
  if (ret == OK)
    {
      out_len = sizeof(ap);
      ret = bcmf_cdc_ioctl(priv, interface, false,
                           WLC_GET_AP, (FAR uint8_t *)&ap, &out_len);
    }

  if (ret == OK)
    {
      if (infra == 0)
        {
          iwr->u.mode = IW_MODE_ADHOC;
        }
      else if (ap)
        {
          iwr->u.mode = IW_MODE_MASTER;
        }
      else
        {
          iwr->u.mode = IW_MODE_INFRA;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: bcmf_wl_set_bssid
 *
 * Description:
 *   Set the bssid for the device
 ****************************************************************************/

int bcmf_wl_set_bssid(FAR struct bcmf_dev_s *priv, struct iwreq *iwr)
{
  uint32_t out_len;
  int interface;
  int infra = 0;
  int ap = 0;
  int ret;

  interface = bcmf_wl_get_interface(priv, iwr);

  if (interface < 0)
    {
      return -EINVAL;
    }

  out_len = sizeof(ap);
  ret = bcmf_cdc_ioctl(priv, interface, false, WLC_GET_AP,
                       (FAR uint8_t *)&ap, &out_len);
  if (ret == OK)
    {
      out_len = sizeof(infra);
      ret = bcmf_cdc_ioctl(priv, interface, false, WLC_GET_INFRA,
                           (FAR uint8_t *)&infra, &out_len);

      if (ret == OK)
        {
          out_len = sizeof(struct ether_addr);
          ret = bcmf_cdc_ioctl(priv, interface, true,
                               ((ap || !infra) ? WLC_SET_BSSID :
                                                 WLC_REASSOC),
                               (FAR uint8_t *)iwr->u.ap_addr.sa_data,
                               &out_len);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: bcmf_wl_get_bssid
 *
 * Description:
 *   Get the bssid for the device
 ****************************************************************************/

int bcmf_wl_get_bssid(FAR struct bcmf_dev_s *priv, struct iwreq *iwr)
{
  uint32_t out_len;
  int interface;

  interface = bcmf_wl_get_interface(priv, iwr);

  if (interface < 0)
    {
      return -EINVAL;
    }

  iwr->u.ap_addr.sa_family = ARPHRD_ETHER;
  out_len = sizeof(struct ether_addr);

  return bcmf_cdc_ioctl(priv, interface, false, WLC_GET_BSSID,
                        (FAR uint8_t *)iwr->u.ap_addr.sa_data, &out_len);
}

/****************************************************************************
 * Name: bcmf_wl_get_channel
 *
 * Description:
 *   Get the channel for the device
 ****************************************************************************/

int bcmf_wl_get_channel(FAR struct bcmf_dev_s *priv, struct iwreq *iwr)
{
  channel_info_t ci;
  uint32_t out_len;
  int interface;
  int ret;

  interface = bcmf_wl_get_interface(priv, iwr);

  if (interface < 0)
    {
      return -EINVAL;
    }

  out_len = sizeof(ci);
  ret = bcmf_cdc_ioctl(priv, interface, false,
                       WLC_GET_CHANNEL, (FAR uint8_t *)&ci, &out_len);
  if (ret == OK)
    {
      iwr->u.freq.m = bcmf_wl_channel_to_frequency(ci.target_channel);
    }

  return ret;
}

/****************************************************************************
 * Name: bcmf_wl_get_rate
 *
 * Description:
 *   Get the data rate for the device
 ****************************************************************************/

int bcmf_wl_get_rate(FAR struct bcmf_dev_s *priv, struct iwreq *iwr)
{
  uint32_t out_len;
  uint32_t rate;
  int interface;
  int ret;

  interface = bcmf_wl_get_interface(priv, iwr);

  if (interface < 0)
    {
      return -EINVAL;
    }

  out_len = sizeof(rate);
  ret = bcmf_cdc_ioctl(priv, interface, false,
                       WLC_GET_RATE, (FAR uint8_t *)&rate, &out_len);
  if (ret == OK)
    {
      iwr->u.bitrate.value = ((rate / 2) * 1000) + ((rate & 1) ? 500 : 0);
      iwr->u.bitrate.fixed = 1;
    }

  return ret;
}

/****************************************************************************
 * Name: bcmf_wl_get_txpower
 *
 * Description:
 *   Get the tranmit power for the device
 ****************************************************************************/

int bcmf_wl_get_txpower(FAR struct bcmf_dev_s *priv, struct iwreq *iwr)
{
  uint32_t out_len;
  int interface;
  int radio;
  int ret;

  interface = bcmf_wl_get_interface(priv, iwr);

  if (interface < 0)
    {
      return -EINVAL;
    }

  out_len = sizeof(radio);
  ret = bcmf_cdc_ioctl(priv, interface, false,
                       WLC_GET_RADIO, (FAR uint8_t *)&radio, &out_len);
  if (ret == OK)
    {
      out_len = sizeof(iwr->u.txpower.value);
      ret = bcmf_cdc_iovar_request(priv, interface, false,
                                   IOVAR_STR_QTXPOWER,
                                   (FAR uint8_t *)&(iwr->u.txpower.value),
                                   &out_len);
      if (ret == OK)
        {
          iwr->u.txpower.value   &= ~WL_TXPWR_OVERRIDE;
          iwr->u.txpower.value   /= 4;

          iwr->u.txpower.fixed    = 0;
          iwr->u.txpower.disabled = radio;
          iwr->u.txpower.flags    = IW_TXPOW_DBM;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: bcmf_wl_get_iwrange
 *
 * Description:
 *   Get the iwrange for the device
 ****************************************************************************/

int bcmf_wl_get_iwrange(FAR struct bcmf_dev_s *priv, struct iwreq *iwr)
{
  struct iw_range *range;
  channel_info_t ci;
  uint32_t out_len;
  int interface;
  int ret;

  interface = bcmf_wl_get_interface(priv, iwr);

  if (interface < 0)
    {
      return -EINVAL;
    }

  if (iwr->u.data.length < sizeof(struct iw_range))
    {
      return -EINVAL;
    }

  range = iwr->u.data.pointer;

  memset(range, 0, sizeof(*range));

  out_len = sizeof(ci);
  ret = bcmf_cdc_ioctl(priv, interface, false,
                       WLC_GET_CHANNEL, (FAR uint8_t *)&ci, &out_len);
  if (ret == OK)
    {
      range->num_frequency = 1;
      range->freq[0].m     = bcmf_wl_channel_to_frequency(ci.target_channel);
      range->freq[0].i     = ci.target_channel;
    }

  return OK;
}

/****************************************************************************
 * Name: bcmf_wl_get_rssi
 *
 * Description:
 *   Get the RSSI for the device
 ****************************************************************************/

int bcmf_wl_get_rssi(FAR struct bcmf_dev_s *priv, struct iwreq *iwr)
{
  wl_sta_rssi_t rssi;
  uint32_t out_len;
  int interface;
  int ret;

  interface = bcmf_wl_get_interface(priv, iwr);

  if (interface < 0)
    {
      return -EINVAL;
    }

  memset(&rssi.sta_addr, 0x0, sizeof(rssi.sta_addr));

  out_len = sizeof(rssi);
  ret = bcmf_cdc_ioctl(priv, interface, false,
                       WLC_GET_RSSI, (FAR uint8_t *)&rssi, &out_len);
  if (ret == OK)
    {
      iwr->u.sens.value = -rssi.rssi;
    }

  return ret;
}

/****************************************************************************
 * Name: bcmf_wl_set_encode_ext
 *
 * Description:
 *   Set the encoding scheme for a device based on iwreq structure.
 ****************************************************************************/

int bcmf_wl_set_encode_ext(FAR struct bcmf_dev_s *priv, struct iwreq *iwr)
{
  struct iw_encode_ext *ext;
  wsec_pmk_t psk;
  int interface;

  interface = bcmf_wl_get_interface(priv, iwr);

  if (interface < 0)
    {
      return -EINVAL;
    }

  ext = (FAR struct iw_encode_ext *)iwr->u.encoding.pointer;

  switch (ext->alg)
    {
      case IW_ENCODE_ALG_TKIP:
        break;
      case IW_ENCODE_ALG_CCMP:
        break;
      case IW_ENCODE_ALG_NONE:
      case IW_ENCODE_ALG_WEP:
      default:
        wlerr("Unknown algo %d\n", ext->alg);
        return -EINVAL;
    }

  memset(&psk, 0, sizeof(wsec_pmk_t));
  memcpy(psk.key, &ext->key, ext->key_len);
  psk.key_len = ext->key_len;
  psk.flags = WSEC_PASSPHRASE;

  priv->auth_pending = true;
  memcpy(&priv->auth_pmk, &psk, sizeof(psk));

  return OK;
}

/****************************************************************************
 * Name: bcmf_wl_set_ssid
 *
 * Description:
 *   Get the SSID for a device based on iwreq structure.
 ****************************************************************************/

int bcmf_wl_set_ssid(FAR struct bcmf_dev_s *priv, struct iwreq *iwr)
{
  sem_t auth_signal;
  uint32_t out_len;
  scb_val_t scbval;
  wlc_ssid_t ssid;
  int interface;
  int ret;

  interface = bcmf_wl_get_interface(priv, iwr);

  if (interface < 0)
    {
      return -EINVAL;
    }

  out_len = sizeof(scbval);
  memset(&scbval, 0x0, out_len);
  ret = bcmf_cdc_ioctl(priv, interface, true,
                       WLC_DISASSOC, (FAR uint8_t *)&scbval, &out_len);
  if (ret < 0 || !iwr->u.essid.flags)
    {
      goto errout_with_auth;
    }

  if (priv->auth_pending)
    {
      out_len = sizeof(priv->auth_pmk);
      ret = bcmf_cdc_ioctl(priv, interface, true,
                           WLC_SET_WSEC_PMK,
                           (FAR uint8_t *)&priv->auth_pmk, &out_len);
      if (ret < 0)
        {
          goto errout_with_auth;
        }
    }

  /* Init authentication signal semaphore */

  nxsem_init(&auth_signal, 0, 0);
  priv->auth_signal = &auth_signal;

  ssid.ssid_len = iwr->u.essid.length;
  memcpy(ssid.SSID, iwr->u.essid.pointer, iwr->u.essid.length);

  /* Configure AP SSID and trig authentication request */

  out_len = sizeof(ssid);
  ret = bcmf_cdc_ioctl(priv, interface, true,
                       WLC_SET_SSID,
                       (FAR uint8_t *)&ssid, &out_len);
  if (ret == OK)
    {
      ret = nxsem_tickwait_uninterruptible(priv->auth_signal,
                                     MSEC2TICK(BCMF_AUTH_TIMEOUT_MS));
    }

  priv->auth_signal = NULL;
  nxsem_destroy(&auth_signal);

  if (ret < 0)
    {
      wlerr("Associate request timeout\n");
      goto errout_with_auth;
    }

  switch (priv->auth_status)
    {
      case OK:
        wlinfo("AP Join ok\n");
        break;

      default:
        wlerr("AP join failed %" PRIu32 "\n", priv->auth_status);
        ret = -EINVAL;
        break;
    }

errout_with_auth:
  priv->auth_pending = false;
  return ret;
}

/****************************************************************************
 * Name: bcmf_wl_get_ssid
 *
 * Description:
 *   Get the SSID for a device.
 ****************************************************************************/

int bcmf_wl_get_ssid(FAR struct bcmf_dev_s *priv, struct iwreq *iwr)
{
  uint32_t out_len;
  wlc_ssid_t ssid;
  int interface;
  int ret;

  interface = bcmf_wl_get_interface(priv, iwr);

  if (interface < 0)
    {
      return -EINVAL;
    }

  /* Configure AP SSID and trig authentication request */

  out_len = sizeof(ssid);
  ret = bcmf_cdc_ioctl(priv, interface, false,
                       WLC_GET_SSID, (FAR uint8_t *)&ssid, &out_len);
  if (ret == OK)
    {
      iwr->u.essid.flags  = iwr->u.data.flags = 1;
      iwr->u.essid.length = iwr->u.data.length = ssid.ssid_len + 1;
      memcpy(iwr->u.essid.pointer, ssid.SSID, iwr->u.essid.length);
    }

  return ret;
}

/****************************************************************************
 * Name: bcmf_wl_set_country_code
 *
 * Description:
 *   Set a new country code.
 ****************************************************************************/

int bcmf_wl_set_country_code(FAR struct bcmf_dev_s *priv,
                             int interface, FAR void *code)
{
  uint8_t country[4] =
    {
      0
    };

  uint32_t out_len;

  memcpy(country, code, 2);

  /* Why out_len = 4 ? Padding bytes to ensure array is
   * terminating with null byte
   */

  out_len = sizeof(country);

  return bcmf_cdc_iovar_request(priv, interface, true,
                                IOVAR_STR_COUNTRY, country,
                                &out_len);
}

/****************************************************************************
 * Name: bcmf_wl_set_country
 *
 * Description:
 *   Set a new country code based on data in an iwreq structure.
 ****************************************************************************/

int bcmf_wl_set_country(FAR struct bcmf_dev_s *priv, FAR struct iwreq *iwr)
{
  int interface;

  interface = bcmf_wl_get_interface(priv, iwr);

  if (interface < 0)
    {
      return -EINVAL;
    }

  return bcmf_wl_set_country_code(priv, interface, iwr->u.data.pointer);
}

/****************************************************************************
 * Name: bcmf_wl_get_country
 *
 * Description:
 *   Get the current country code.
 ****************************************************************************/

int bcmf_wl_get_country(FAR struct bcmf_dev_s *priv, FAR struct iwreq *iwr)
{
  uint8_t country[4] =
    {
      0
    };

  uint32_t out_len;
  int interface;
  int ret;

  interface = bcmf_wl_get_interface(priv, iwr);

  if (interface < 0 || iwr->u.data.pointer == NULL)
    {
      return -EINVAL;
    }

  out_len = sizeof(country);
  ret = bcmf_cdc_iovar_request(priv, interface, false,
                               IOVAR_STR_COUNTRY, country,
                               &out_len);
  if (ret == OK)
    {
      memcpy(iwr->u.data.pointer, country, 2);
      ((uint8_t *)iwr->u.data.pointer)[2] = '\0';
    }

  return ret;
}

/****************************************************************************
 * Name: bcmf_wl_set_dtim
 ****************************************************************************/

#ifdef CONFIG_IEEE80211_BROADCOM_LOWPOWER

int bcmf_wl_set_dtim(FAR struct bcmf_dev_s *priv,
                     uint32_t interval_ms)
{
  uint32_t value = interval_ms / 100;
  uint32_t out_len;
  int ret;

  out_len = sizeof(interval_ms);

  if (value == 0)
    {
      return -EINVAL;
    }

  if (priv->lp_dtim == interval_ms)
    {
      return OK;
    }

  ret = bcmf_cdc_iovar_request(priv, CHIP_STA_INTERFACE, true,
                               IOVAR_STR_LISTEN_INTERVAL_DTIM,
                               (FAR uint8_t *)&value, &out_len);
  if (ret == OK)
    {
      priv->lp_dtim = interval_ms;
    }

  return ret;
}

#endif
