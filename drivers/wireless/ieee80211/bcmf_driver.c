/****************************************************************************
 * drivers/wireless/ieee80211/bcmf_driver.c
 *
 *   Copyright (C) 2017-2018 Gregory Nutt. All rights reserved.
 *   Author: Simon Piriou <spiriou31@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <net/ethernet.h>

#include <nuttx/kmalloc.h>
#include <nuttx/wdog.h>
#include <nuttx/sdio.h>
#include <nuttx/net/arp.h>
#include <nuttx/wireless/ieee80211/ieee80211.h>

#include "bcmf_driver.h"
#include "bcmf_cdc.h"
#include "bcmf_ioctl.h"
#include "bcmf_utils.h"
#include "bcmf_netdev.h"
#include "bcmf_sdio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DOT11_BSSTYPE_ANY      2
#define BCMF_SCAN_TIMEOUT_TICK (5*CLOCKS_PER_SEC)
#define BCMF_AUTH_TIMEOUT_MS   10000
#define BCMF_SCAN_RESULT_SIZE  1024

/* clm file is cut into pieces of MAX_CHUNK_LEN.
 * It is relatively small because dongles (FW) have a small maximum size input
 * payload restriction for ioctl's ... something like 1900'ish bytes. So chunk
 * len should not exceed 1400 bytes
 */

#define MAX_CHUNK_LEN (CONFIG_NET_ETH_PKTSIZE > 1500 ? 1400 : CONFIG_NET_ETH_PKTSIZE - 100)

/* Helper to get iw_event size */

#define BCMF_IW_EVENT_SIZE(field) \
  (offsetof(struct iw_event, u) + sizeof(((union iwreq_data *)0)->field))

/* Clm blob marcos */

#define DLOAD_HANDLER_VER     1       /* Downloader version */
#define DLOAD_FLAG_VER_MASK   0xf000  /* Downloader version mask */
#define DLOAD_FLAG_VER_SHIFT  12      /* Downloader version shift */

#define DL_CRC_NOT_INUSE      0x0001
#define DL_BEGIN              0x0002
#define DL_END                0x0004

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* clm blob download head */

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

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static FAR struct bcmf_dev_s *bcmf_allocate_device(void);
static void bcmf_free_device(FAR struct bcmf_dev_s *priv);

static int bcmf_driver_initialize(FAR struct bcmf_dev_s *priv);

static int bcmf_driver_download_clm(FAR struct bcmf_dev_s *priv);

/* FIXME only for debug purpose */

static void bcmf_wl_default_event_handler(FAR struct bcmf_dev_s *priv,
                            struct bcmf_event_s *event, unsigned int len);

static void bcmf_wl_radio_event_handler(FAR struct bcmf_dev_s *priv,
                            struct bcmf_event_s *event, unsigned int len);

static void bcmf_wl_scan_event_handler(FAR struct bcmf_dev_s *priv,
                            struct bcmf_event_s *event, unsigned int len);

static void bcmf_wl_auth_event_handler(FAR struct bcmf_dev_s *priv,
                            struct bcmf_event_s *event, unsigned int len);

static int bcmf_wl_get_interface(FAR struct bcmf_dev_s *priv,
                            struct iwreq *iwr);

/****************************************************************************
 * Private Functions
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

  if ((ret = nxsem_setprotocol(&priv->control_timeout, SEM_PRIO_NONE)) != OK)
    {
      goto exit_free_priv;
    }

  /* Init authentication signal semaphore */

  if ((ret = nxsem_init(&priv->auth_signal, 0, 0)) != OK)
    {
      goto exit_free_priv;
    }

  if ((ret = nxsem_setprotocol(&priv->auth_signal, SEM_PRIO_NONE)) != OK)
    {
      goto exit_free_priv;
    }

  /* Init scan timeout timer */

  priv->scan_status = BCMF_SCAN_DISABLED;
  priv->scan_timeout = wd_create();
  if (!priv->scan_timeout)
    {
      ret = -ENOMEM;
      goto exit_free_priv;
    }

  return priv;

exit_free_priv:
  kmm_free(priv);
  return NULL;
}

void bcmf_free_device(FAR struct bcmf_dev_s *priv)
{
  /* TODO deinitialize device structures */

  kmm_free(priv);
}

int bcmf_wl_set_mac_address(FAR struct bcmf_dev_s *priv, struct ifreq *req)
{
  int ret;
  uint32_t out_len = IFHWADDRLEN;

  ret = bcmf_cdc_iovar_request(priv, CHIP_STA_INTERFACE, true,
                              IOVAR_STR_CUR_ETHERADDR,
                              (uint8_t *)req->ifr_hwaddr.sa_data,
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

int bcmf_driver_download_clm(FAR struct bcmf_dev_s *priv)
{
  FAR struct bcmf_sdio_dev_s *sbus = (FAR struct bcmf_sdio_dev_s *)priv->bus;
  FAR uint8_t *srcbuff = sbus->chip->clm_blob_image;
  FAR uint8_t *downloadbuff;
  unsigned int datalen = *sbus->chip->clm_blob_image_size;
  uint16_t dl_flag;
  int ret = 0;

  if (srcbuff == NULL || datalen <= 0)
    {
      wlinfo("Skip clm blob...\n");
      return 0;
    }
  else
    {
      wlinfo("Download %d bytes @ 0x%08x\n", datalen, srcbuff);
    }

  /* Divide clm blob into chunks */

  downloadbuff = kmm_malloc(sizeof(struct wl_dload_data) + MAX_CHUNK_LEN);
  if (!downloadbuff)
    {
      wlerr("No memory for clm data\n");
      return -ENOMEM;
    }

  dl_flag = DL_BEGIN;
  do
    {
      FAR struct wl_dload_data *dlhead;
      unsigned int chunk_len = datalen >= MAX_CHUNK_LEN ? MAX_CHUNK_LEN : datalen;
      uint32_t out_len;

      memcpy(downloadbuff + sizeof(struct wl_dload_data), srcbuff, chunk_len);
      datalen -= chunk_len;
      srcbuff += chunk_len;

      if (datalen <= 0)
        {
          dl_flag |= DL_END;
        }

      /* clm header */

      dlhead             = (struct wl_dload_data *)downloadbuff;
      dlhead->flag       = (DLOAD_HANDLER_VER << DLOAD_FLAG_VER_SHIFT) | dl_flag;
      dlhead->dload_type = DL_TYPE_CLM;
      dlhead->len        = chunk_len;
      dlhead->crc        = 0;

      out_len            = chunk_len + sizeof(struct wl_dload_data);
      out_len            = (out_len + 7) & ~0x7U;

      ret = bcmf_cdc_iovar_request(priv, CHIP_STA_INTERFACE, true,
                                   IOVAR_STR_CLMLOAD, downloadbuff,
                                   &out_len);

      dl_flag &= (uint16_t)~DL_BEGIN;
    }
  while ((datalen > 0) && (ret == OK));

  kmm_free(downloadbuff);
  return ret;
}

int bcmf_driver_initialize(FAR struct bcmf_dev_s *priv)
{
  int ret;
  uint32_t out_len;
  uint32_t value;
  uint8_t tmp_buf[64];
  int interface = CHIP_STA_INTERFACE;

  /* Download clm blob if needed */

  ret = bcmf_driver_download_clm(priv);
  if (ret != OK)
    {
      return -EIO;
    }

  /* Disable TX Gloming feature */

  out_len = 4;
  *(FAR uint32_t *)tmp_buf = 0;
  ret = bcmf_cdc_iovar_request(priv, interface, true,
                               IOVAR_STR_TX_GLOM, tmp_buf,
                               &out_len);
  if (ret != OK)
    {
      return -EIO;
    }

  /* FIXME disable power save mode */

  out_len = 4;
  value   = 0;
  ret     = bcmf_cdc_ioctl(priv, interface, true, WLC_SET_PM,
                           (uint8_t *)&value, &out_len);
  if (ret != OK)
    {
      return ret;
    }

  /* Set the GMode to auto */

  out_len = 4;
  value = GMODE_AUTO;
  ret = bcmf_cdc_ioctl(priv, interface, true, WLC_SET_GMODE,
                       (uint8_t *)&value, &out_len);
  if (ret != OK)
    {
      return ret;
    }

  /* TODO configure roaming if needed. Disable for now */

  out_len = 4;
  value   = 1;
  ret     = bcmf_cdc_iovar_request(priv, interface, true,
                                   IOVAR_STR_ROAM_OFF,
                                   (FAR uint8_t *)&value,
                                   &out_len);

  /* TODO configure EAPOL version to default */

  out_len = 8;
  ((FAR uint32_t *)tmp_buf)[0] = interface;
  ((FAR uint32_t *)tmp_buf)[1] = (uint32_t)-1;

  if (bcmf_cdc_iovar_request(priv, interface, true,
                             "bsscfg:"IOVAR_STR_SUP_WPA2_EAPVER, tmp_buf,
                             &out_len))
    {
      return -EIO;
    }

  /* Query firmware version string */

  out_len = sizeof(tmp_buf);
  ret     = bcmf_cdc_iovar_request(priv, interface, false,
                                   IOVAR_STR_VERSION, tmp_buf,
                                   &out_len);
  if (ret != OK)
    {
      return -EIO;
    }

  tmp_buf[sizeof(tmp_buf)-1] = 0;

  /* Remove line feed */

  out_len = strlen((char *)tmp_buf);
  if (out_len > 0 && tmp_buf[out_len - 1] == '\n')
    {
      tmp_buf[out_len - 1] = 0;
    }

  wlinfo("fw version <%s>\n", tmp_buf);

  /* FIXME Configure event mask to enable all asynchronous events */

  for (ret = 0; ret < BCMF_EVENT_COUNT; ret++)
    {
      bcmf_event_register(priv, bcmf_wl_default_event_handler, ret);
    }

  /*  Register radio event */

  bcmf_event_register(priv, bcmf_wl_radio_event_handler, WLC_E_RADIO);

  /*  Register AP scan event */

  bcmf_event_register(priv, bcmf_wl_scan_event_handler, WLC_E_ESCAN_RESULT);

  /*  Register authentication related events */

  bcmf_event_register(priv, bcmf_wl_auth_event_handler, WLC_E_ASSOC_IND_NDIS);
  bcmf_event_register(priv, bcmf_wl_auth_event_handler, WLC_E_AUTH);
  bcmf_event_register(priv, bcmf_wl_auth_event_handler, WLC_E_ASSOC);
  bcmf_event_register(priv, bcmf_wl_auth_event_handler, WLC_E_LINK);
  bcmf_event_register(priv, bcmf_wl_auth_event_handler, WLC_E_PSK_SUP);
  bcmf_event_register(priv, bcmf_wl_auth_event_handler, WLC_E_JOIN);
  bcmf_event_register(priv, bcmf_wl_auth_event_handler, WLC_E_SET_SSID);
  bcmf_event_register(priv, bcmf_wl_auth_event_handler, WLC_E_DEAUTH_IND);
  bcmf_event_register(priv, bcmf_wl_auth_event_handler, WLC_E_DISASSOC);
  bcmf_event_register(priv, bcmf_wl_auth_event_handler, WLC_E_DISASSOC_IND);

  if (bcmf_event_push_config(priv))
    {
      return -EIO;
    }

  /* Register network driver */

  return bcmf_netdev_register(priv);
}

void bcmf_wl_default_event_handler(FAR struct bcmf_dev_s *priv,
                                   struct bcmf_event_s *event, unsigned int len)
{
  wlinfo("Got event %d from <%s>\n", bcmf_getle32(&event->type),
                                     event->src_name);
}

void bcmf_wl_radio_event_handler(FAR struct bcmf_dev_s *priv,
                                 struct bcmf_event_s *event, unsigned int len)
{
}

void bcmf_wl_auth_event_handler(FAR struct bcmf_dev_s *priv,
                                   struct bcmf_event_s *event, unsigned int len)
{
  uint32_t type;
  uint32_t status;

  type = bcmf_getle32(&event->type);
  status = bcmf_getle32(&event->status);

  wlinfo("Got auth event %d from <%s>\n", type, event->src_name);

  bcmf_hexdump((uint8_t *)event, len, (unsigned long)event);

  if (type == WLC_E_SET_SSID && status == WLC_E_STATUS_SUCCESS)
    {
      /* Auth complete */

      priv->auth_status = OK;

      nxsem_post(&priv->auth_signal);
    }
}

/* bcmf_wl_scan_event_handler must run at high priority else
 * race condition may occur on priv->scan_result field
 */
void bcmf_wl_scan_event_handler(FAR struct bcmf_dev_s *priv,
                                   struct bcmf_event_s *event, unsigned int len)
{
  uint32_t status;
  uint32_t event_len;
  struct wl_escan_result *result;
  struct wl_bss_info *bss;
  unsigned int bss_info_len;
  unsigned int escan_result_len;
  unsigned int bss_count = 0;

  event_len = len;

  if (priv->scan_status < BCMF_SCAN_RUN)
    {
      wlinfo("Got Unexpected scan event\n");
      goto exit_invalid_frame;
    }

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

  result = (struct wl_escan_result *)&event[1];

  if (len < result->buflen || result->buflen < sizeof(struct wl_escan_result))
    {
      goto exit_invalid_frame;
    }

  /* wl_escan_result structure cointains a wl_bss_info field */

  len = result->buflen - sizeof(struct wl_escan_result)
                       + sizeof(struct wl_bss_info);

  /* Process bss_infos */

  bss = result->bss_info;

  while (len > 0 && bss_count < result->bss_count)
    {
      struct iw_event *iwe;
      unsigned int result_size;
      size_t essid_len;
      size_t essid_len_aligned;
      uint8_t *ie_buffer;
      unsigned int ie_offset;
      unsigned int check_offset;

      result_size = BCMF_SCAN_RESULT_SIZE - priv->scan_result_size;
      bss_info_len = bss->length;

      if (len < bss_info_len)
        {
          wlerr("bss_len error %d %d\n", len, bss_info_len);
          goto exit_invalid_frame;
        }

      /* Append current bss_info to priv->scan_results
       * FIXME protect this against race conditions
       */

      /* Check if current bss AP is not already detected */

      check_offset = 0;

      while (priv->scan_result_size - check_offset
                                     >= offsetof(struct iw_event, u))
        {
          iwe = (struct iw_event *)&priv->scan_result[check_offset];

          if (iwe->cmd == SIOCGIWAP)
            {
              if (memcmp(&iwe->u.ap_addr.sa_data, bss->BSSID.ether_addr_octet,
                         sizeof(bss->BSSID.ether_addr_octet)) == 0)
                {
                  goto process_next_bss;
                }
            }

          check_offset += iwe->len;
        }

      wlinfo("Scan result: <%.32s> %02x:%02x:%02x:%02x:%02x:%02x\n", bss->SSID,
               bss->BSSID.ether_addr_octet[0], bss->BSSID.ether_addr_octet[1],
               bss->BSSID.ether_addr_octet[2], bss->BSSID.ether_addr_octet[3],
               bss->BSSID.ether_addr_octet[4], bss->BSSID.ether_addr_octet[5]);

      /* Copy BSSID */

      if (result_size < BCMF_IW_EVENT_SIZE(ap_addr))
        {
          goto scan_result_full;
        }

      iwe = (struct iw_event *)&priv->scan_result[priv->scan_result_size];
      iwe->len = BCMF_IW_EVENT_SIZE(ap_addr);
      iwe->cmd = SIOCGIWAP;
      memcpy(&iwe->u.ap_addr.sa_data, bss->BSSID.ether_addr_octet,
             sizeof(bss->BSSID.ether_addr_octet));
      iwe->u.ap_addr.sa_family = ARPHRD_ETHER;

      priv->scan_result_size += BCMF_IW_EVENT_SIZE(ap_addr);
      result_size -= BCMF_IW_EVENT_SIZE(ap_addr);

      /* Copy ESSID */

      essid_len = min(strlen((const char *)bss->SSID), 32);
      essid_len_aligned = (essid_len + 3) & -4;

      if (result_size < BCMF_IW_EVENT_SIZE(essid)+essid_len_aligned)
        {
          goto scan_result_full;
        }

      iwe = (struct iw_event *)&priv->scan_result[priv->scan_result_size];
      iwe->len = BCMF_IW_EVENT_SIZE(essid)+essid_len_aligned;
      iwe->cmd = SIOCGIWESSID;
      iwe->u.essid.flags = 0;
      iwe->u.essid.length = essid_len;

      /* Special processing for iw_point, set offset in pointer field */

      iwe->u.essid.pointer = (FAR void *)sizeof(iwe->u.essid);
      memcpy(&iwe->u.essid+1, bss->SSID, essid_len);

      priv->scan_result_size += BCMF_IW_EVENT_SIZE(essid)+essid_len_aligned;
      result_size -= BCMF_IW_EVENT_SIZE(essid)+essid_len_aligned;

      /* Copy link quality info */

      if (result_size < BCMF_IW_EVENT_SIZE(qual))
        {
          goto scan_result_full;
        }

      iwe = (struct iw_event *)&priv->scan_result[priv->scan_result_size];
      iwe->len = BCMF_IW_EVENT_SIZE(qual);
      iwe->cmd = IWEVQUAL;
      iwe->u.qual.qual = bss->SNR;
      wlinfo("signal %d %d %d\n", bss->RSSI, bss->phy_noise, bss->SNR);
      iwe->u.qual.level = bss->RSSI;
      iwe->u.qual.noise = bss->phy_noise;
      iwe->u.qual.updated = IW_QUAL_DBM | IW_QUAL_ALL_UPDATED;

      priv->scan_result_size += BCMF_IW_EVENT_SIZE(qual);
      result_size -= BCMF_IW_EVENT_SIZE(qual);

      /* Copy AP mode */

      if (result_size < BCMF_IW_EVENT_SIZE(mode))
        {
          goto scan_result_full;
        }

      iwe = (struct iw_event *)&priv->scan_result[priv->scan_result_size];
      iwe->len = BCMF_IW_EVENT_SIZE(mode);
      iwe->cmd = SIOCGIWMODE;
      if (bss->capability & DOT11_CAP_ESS)
        {
          iwe->u.mode = IW_MODE_INFRA;
        }
      else if (bss->capability & DOT11_CAP_IBSS)
        {
          iwe->u.mode = IW_MODE_ADHOC;
        }
      else
        {
          iwe->u.mode = IW_MODE_AUTO;
        }

      priv->scan_result_size += BCMF_IW_EVENT_SIZE(mode);
      result_size -= BCMF_IW_EVENT_SIZE(mode);

      /* Copy AP encryption mode */

      if (result_size < BCMF_IW_EVENT_SIZE(data))
        {
          goto scan_result_full;
        }

      iwe = (struct iw_event *)&priv->scan_result[priv->scan_result_size];
      iwe->len = BCMF_IW_EVENT_SIZE(data);
      iwe->cmd = SIOCGIWENCODE;
      iwe->u.data.flags = bss->capability & DOT11_CAP_PRIVACY ?
                          IW_ENCODE_ENABLED | IW_ENCODE_NOKEY :
                          IW_ENCODE_DISABLED;
      iwe->u.data.length = 0;
      iwe->u.essid.pointer = NULL;

      priv->scan_result_size += BCMF_IW_EVENT_SIZE(data);
      result_size -= BCMF_IW_EVENT_SIZE(data);

      /* Copy relevant raw IE frame */

      if (bss->ie_offset >= bss_info_len ||
          bss->ie_length > bss_info_len-bss->ie_offset)
        {
          goto process_next_bss;
        }

      ie_offset = 0;
      ie_buffer = (uint8_t *)bss + bss->ie_offset;

      while (1)
        {
          size_t ie_frame_size;

          if (bss->ie_length - ie_offset < 2)
            {
              /* Minimum Information element size is 2 bytes */

              break;
            }

          ie_frame_size = ie_buffer[ie_offset+1] + 2;

          if (ie_frame_size > bss->ie_length - ie_offset)
            {
              /* Entry too big */

              break;
            }

          switch (ie_buffer[ie_offset])
            {
              case IEEE80211_ELEMID_RSN:
                {
                  size_t ie_frame_size_aligned;
                  ie_frame_size_aligned = (ie_frame_size + 3) & -4;

                  wlinfo("found RSN\n");
                  if (result_size < BCMF_IW_EVENT_SIZE(data) + ie_frame_size_aligned)
                    {
                      break;
                    }

                  iwe = (struct iw_event *)&priv->scan_result[priv->scan_result_size];
                  iwe->len = BCMF_IW_EVENT_SIZE(data)+ie_frame_size_aligned;
                  iwe->cmd = IWEVGENIE;
                  iwe->u.data.flags = 0;
                  iwe->u.data.length = ie_frame_size;
                  iwe->u.data.pointer = (FAR void *)sizeof(iwe->u.data);
                  memcpy(&iwe->u.data+1, &ie_buffer[ie_offset], ie_frame_size);

                  priv->scan_result_size += BCMF_IW_EVENT_SIZE(essid)+ie_frame_size_aligned;
                  result_size -= BCMF_IW_EVENT_SIZE(essid)+ie_frame_size_aligned;
                  break;
                }

              default:
                break;
            }

          ie_offset += ie_buffer[ie_offset+1] + 2;
        }

      goto process_next_bss;

    scan_result_full:
      /* Continue instead of break to log dropped AP results */

      wlerr("No more space in scan_result buffer\n");

    process_next_bss:
      /* Process next bss_info */

      len -= bss_info_len;
      bss = (struct wl_bss_info *)((uint8_t *)bss + bss_info_len);
      bss_count += 1;
    }

wl_escan_result_processed:

  if (status == WLC_E_STATUS_PARTIAL)
    {
      /* More frames to come */

      return;
    }

  if (status != WLC_E_STATUS_SUCCESS)
    {
      wlerr("Invalid event status %d\n", status);
      return;
    }

  /* Scan done */

  wlinfo("escan done event %d %d\n", status, bcmf_getle32(&event->reason));

  wd_cancel(priv->scan_timeout);

  priv->scan_status = BCMF_SCAN_DONE;
  nxsem_post(&priv->control_mutex);

  return;

exit_invalid_frame:
  wlerr("Invalid scan result event\n");
  bcmf_hexdump((uint8_t *)event, event_len, (unsigned long)event);
}

void bcmf_wl_scan_timeout(int argc, wdparm_t arg1, ...)
{
  FAR struct bcmf_dev_s *priv = (FAR struct bcmf_dev_s *)arg1;

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

int bcmf_wl_get_interface(FAR struct bcmf_dev_s *priv, struct iwreq *iwr)
{
  /* TODO resolve interface using iwr->ifr_name */

  return CHIP_STA_INTERFACE;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int bcmf_sdio_initialize(int minor, FAR struct sdio_dev_s *dev)
{
  int ret;
  FAR struct bcmf_dev_s *priv;

  wlinfo("minor: %d\n", minor);

  priv = bcmf_allocate_device();
  if (!priv)
    {
      return -ENOMEM;
    }

  /* Init sdio bus */

  ret = bcmf_bus_sdio_initialize(priv, minor, dev);
  if (ret != OK)
    {
      ret = -EIO;
      goto exit_free_device;
    }

  /* Bus initialized, register network driver */

  return bcmf_driver_initialize(priv);

exit_free_device:
  bcmf_free_device(priv);
  return ret;
}

int bcmf_wl_enable(FAR struct bcmf_dev_s *priv, bool enable)
{
  int ret;
  uint32_t out_len;

  /* TODO chek device state */

  out_len = 0;
  ret = bcmf_cdc_ioctl(priv, CHIP_STA_INTERFACE, true,
                         enable ? WLC_UP : WLC_DOWN, NULL, &out_len);

  /* TODO wait for WLC_E_RADIO event */

  usleep(3000);

  if (ret == OK)
    {
      /* TODO update device state */
    }

  return ret;
}

int bcmf_wl_start_scan(FAR struct bcmf_dev_s *priv, struct iwreq *iwr)
{
  int ret;
  uint32_t out_len;
  uint32_t value;
  int interface;
  struct iw_scan_req *req;
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

  memset(&scan_params.params.bssid, 0xFF,
          sizeof(scan_params.params.bssid));

  scan_params.params.bss_type = DOT11_BSSTYPE_ANY;
  scan_params.params.nprobes = -1;
  scan_params.params.active_time = -1;
  scan_params.params.passive_time = -1;
  scan_params.params.home_time = -1;
  scan_params.params.channel_num = 0;

  if (iwr->u.data.pointer && iwr->u.data.length >= sizeof(*req))
    {
      req = (struct iw_scan_req *)iwr->u.data.pointer;

      memcpy(&scan_params.params.bssid, req->bssid.sa_data,
             sizeof(scan_params.params.bssid));

      scan_params.params.scan_type =
                  req->scan_type == IW_SCAN_TYPE_ACTIVE ? 0:1;

      if (iwr->u.data.flags & IW_SCAN_THIS_ESSID &&
          req->essid_len < sizeof(scan_params.params.ssid.SSID))
        {
          /* Scan specific ESSID */

          memcpy(scan_params.params.ssid.SSID, req->essid, req->essid_len);
          scan_params.params.ssid.SSID_len = req->essid_len;
        }
    }
  else
    {
      /* Default scan parameters */

      wlinfo("Use default scan parameters\n");

      memset(&scan_params.params.bssid, 0xFF,
             sizeof(scan_params.params.bssid));

      scan_params.params.scan_type = 0; /* Active scan */
    }

  /* Set active scan mode */

  value = scan_params.params.scan_type;
  out_len = 4;
  if (bcmf_cdc_ioctl(priv, CHIP_STA_INTERFACE, true,
                         WLC_SET_PASSIVE_SCAN, (uint8_t *)&value, &out_len))
    {
      ret = -EIO;
      goto exit_failed;
    }

  /* Lock control_mutex semaphore */

  if ((ret = nxsem_wait(&priv->control_mutex)) < 0)
    {
       goto exit_failed;
    }

  /* Allocate buffer to store scan result */

  if (priv->scan_result == NULL)
   {
     priv->scan_result = kmm_malloc(BCMF_SCAN_RESULT_SIZE);
     if (priv->scan_result == NULL)
       {
         wlerr("Cannot allocate result buffer\n");
         ret = -ENOMEM;
         goto exit_sem_post;
       }
   }

  wlinfo("start scan\n");

  priv->scan_result_size = 0;
  priv->scan_status = BCMF_SCAN_RUN;

  out_len = sizeof(scan_params);

  if (bcmf_cdc_iovar_request_unsafe(priv, CHIP_STA_INTERFACE, true,
                                 IOVAR_STR_ESCAN, (uint8_t *)&scan_params,
                                 &out_len))
    {
      ret = -EIO;
      goto exit_sem_post;
    }

  /*  Start scan_timeout timer */

  (void)wd_start(priv->scan_timeout, BCMF_SCAN_TIMEOUT_TICK,
                 bcmf_wl_scan_timeout, (wdparm_t)priv);

  return OK;

exit_sem_post:
  priv->scan_status = BCMF_SCAN_DISABLED;
  nxsem_post(&priv->control_mutex);

exit_failed:
  wlinfo("Failed\n");
  return ret;
}

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

  if ((ret = nxsem_wait(&priv->control_mutex)) < 0)
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

  if (iwr->u.data.pointer == NULL ||
      iwr->u.data.length < priv->scan_result_size)
    {
      /* Stat request, return scan_result_size */

      ret = -E2BIG;
      iwr->u.data.pointer = NULL;
      iwr->u.data.length = priv->scan_result_size;
      goto exit_sem_post;
    }

  if (priv->scan_result_size <= 0)
    {
      ret = OK;
      iwr->u.data.length = 0;
      goto exit_free_buffer;
    }

  /* Copy result to user buffer */

  if (iwr->u.data.length > priv->scan_result_size)
    {
      iwr->u.data.length = priv->scan_result_size;
    }

  memcpy(iwr->u.data.pointer, priv->scan_result, iwr->u.data.length);

exit_free_buffer:
  /* Free scan result buffer */

  kmm_free(priv->scan_result);
  priv->scan_result = NULL;
  priv->scan_result_size = 0;

exit_sem_post:
  nxsem_post(&priv->control_mutex);

exit_failed:
  if (ret < 0)
    {
      iwr->u.data.length = 0;
    }

  return ret;
}

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
              wlerr("Invalid wpa version %d\n", iwr->u.param.value);
              return -EINVAL;
          }

        out_len = 8;
        wpa_version[0] = interface;

        if (bcmf_cdc_iovar_request(priv, interface, true,
                                   "bsscfg:"IOVAR_STR_SUP_WPA,
                                   (uint8_t *)wpa_version,
                                   &out_len))
          {
            return -EIO;
          }

        out_len = 4;
        if (bcmf_cdc_ioctl(priv, interface, true, WLC_SET_WPA_AUTH,
                           (uint8_t *)&auth_mode, &out_len))
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
              wlerr("Invalid cipher mode %d\n", iwr->u.param.value);
              return -EINVAL;
          }

        out_len = 4;
        if (bcmf_cdc_ioctl(priv, interface, true,
                           WLC_SET_WSEC, (uint8_t *)&cipher_mode, &out_len))
          {
            return -EIO;
          }

        /* Set authentication mode */

        out_len = 4;
        if (bcmf_cdc_ioctl(priv, interface, true,
                           WLC_SET_AUTH, (uint8_t *)&wep_auth, &out_len))
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

int bcmf_wl_set_mode(FAR struct bcmf_dev_s *priv, struct iwreq *iwr)
{
  int interface;
  uint32_t out_len;
  uint32_t value;

  interface = bcmf_wl_get_interface(priv, iwr);

  if (interface < 0)
    {
      return -EINVAL;
    }

  out_len = 4;
  value = iwr->u.mode == IW_MODE_INFRA ? 1 : 0;
  if (bcmf_cdc_ioctl(priv, interface, true,
                     WLC_SET_INFRA, (uint8_t *)&value, &out_len))
    {
      return -EIO;
    }

  return OK;
}

int bcmf_wl_set_encode_ext(FAR struct bcmf_dev_s *priv, struct iwreq *iwr)
{
  int interface;
  struct iw_encode_ext *ext;
  uint32_t out_len;
  wsec_pmk_t psk;

  interface = bcmf_wl_get_interface(priv, iwr);

  if (interface < 0)
    {
      return -EINVAL;
    }

  ext = (struct iw_encode_ext *)iwr->u.encoding.pointer;

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

  out_len = sizeof(psk);
  return bcmf_cdc_ioctl(priv, interface, true,
                        WLC_SET_WSEC_PMK, (uint8_t *)&psk, &out_len);
}

int bcmf_wl_set_ssid(FAR struct bcmf_dev_s *priv, struct iwreq *iwr)
{
  int ret;
  int interface;
  uint32_t out_len;
  wlc_ssid_t ssid;

  interface = bcmf_wl_get_interface(priv, iwr);

  if (interface < 0)
    {
      return -EINVAL;
    }

  ssid.SSID_len = iwr->u.essid.length;
  memcpy(ssid.SSID, iwr->u.essid.pointer, iwr->u.essid.length);

  /* Configure AP SSID and trig authentication request */

  out_len = sizeof(ssid);
  if (bcmf_cdc_ioctl(priv, interface, true,
                     WLC_SET_SSID, (uint8_t *)&ssid, &out_len))
    {
      return -EIO;
    }

  ret = bcmf_sem_wait(&priv->auth_signal, BCMF_AUTH_TIMEOUT_MS);

  wlinfo("semwait done ! %d\n", ret);

  if (ret < 0)
    {
      wlerr("Associate request timeout\n");
      return ret;
    }

  switch (priv->auth_status)
    {
      case OK:
        wlinfo("AP Join ok\n");
        break;

      default:
        wlerr("AP join failed %d\n", priv->auth_status);
        return -EINVAL;
    }

  return OK;
 }
