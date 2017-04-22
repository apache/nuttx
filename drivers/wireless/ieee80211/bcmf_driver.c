/****************************************************************************
 * drivers/wireless/ieee80211/bcmf_driver.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#include <nuttx/kmalloc.h>

#include "bcmf_driver.h"
#include "bcmf_sdpcm.h"
#include "bcmf_sdio_core.h"
#include "bcmf_ioctl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

// TODO move elsewhere
#define DOT11_BSSTYPE_ANY     2
#define WL_SCAN_CHANNEL_TIME   40
#define WL_SCAN_UNASSOC_TIME   40
#define WL_SCAN_PASSIVE_TIME   120

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int bcmf_run_escan(FAR struct bcmf_dev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

int bcmf_wl_set_mac_address(FAR struct bcmf_dev_s *priv, uint8_t *addr)
{
  int ret;
  uint32_t out_len = 6;

  ret = bcmf_sdpcm_iovar_request(priv, CHIP_STA_INTERFACE, true,
                                 IOVAR_STR_CUR_ETHERADDR, addr,
                                 &out_len);
  if (ret != OK)
    {
      return ret;
    }

  _info("MAC address updated %02X:%02X:%02X:%02X:%02X:%02X\n",
                            addr[0], addr[1], addr[2],
                            addr[3], addr[4], addr[5]);
  memcpy(priv->mac_addr, addr, 6);
    
  return OK;
}

int bcmf_wl_enable(FAR struct bcmf_dev_s *priv, bool enable)
{
  int ret;
  uint32_t out_len;

  /* TODO chek device state */

  out_len = 0;
  ret = bcmf_sdpcm_ioctl(priv, CHIP_STA_INTERFACE, true,
                         enable ? WLC_UP : WLC_DOWN, NULL, &out_len);
  
  if (ret == OK)
    {
      /* TODO update device state */
    }

  return ret;
}

int bcmf_dongle_scantime(FAR struct bcmf_dev_s *priv, int32_t scan_assoc_time,
          int32_t scan_unassoc_time, int32_t scan_passive_time)
{
  int ret;
  uint32_t out_len, value;

  out_len = 4;
  value = scan_assoc_time;
  ret = bcmf_sdpcm_ioctl(priv, CHIP_STA_INTERFACE, true,
                         WLC_SET_SCAN_CHANNEL_TIME, (uint8_t*)&value,
                         &out_len);
  if (ret != OK)
    {
      return -EIO;
    }

  out_len = 4;
  value = scan_unassoc_time;
  ret = bcmf_sdpcm_ioctl(priv, CHIP_STA_INTERFACE, true,
                         WLC_SET_SCAN_UNASSOC_TIME, (uint8_t*)&value,
                         &out_len);
  if (ret != OK)
    {
      return -EIO;
    }

  out_len = 4;
  value = scan_passive_time;
  ret = bcmf_sdpcm_ioctl(priv, CHIP_STA_INTERFACE, true,
                         WLC_SET_SCAN_PASSIVE_TIME, (uint8_t*)&value,
                         &out_len);
  if (ret != OK)
    {
      return -EIO;
    }

  return OK;
}

int bcmf_dongle_initialize(FAR struct bcmf_dev_s *priv)
{
  int ret;
  uint32_t out_len;
  uint32_t value;

  ret = bcmf_wl_enable(priv, true);
  if (ret)
    {
      return ret;
    }

  ret = bcmf_dongle_scantime(priv, WL_SCAN_CHANNEL_TIME,
            WL_SCAN_UNASSOC_TIME, WL_SCAN_PASSIVE_TIME);
  if (ret)
    {
      return ret;
    }

  /* FIXME disable power save mode */

  out_len = 4;
  value = 0;
  ret = bcmf_sdpcm_ioctl(priv, CHIP_STA_INTERFACE, true,
                         WLC_SET_PM, (uint8_t*)&value, &out_len);
  if (ret != OK)
    {
      return ret;
    }

  /* Set the GMode */

  out_len = 4;
  value = GMODE_AUTO;
  ret = bcmf_sdpcm_ioctl(priv, CHIP_STA_INTERFACE, true,
                         WLC_SET_GMODE, (uint8_t*)&value, &out_len);
  if (ret != OK)
    {
      return ret;
    }

  /* TODO configure roaming if needed. Disable for now */

  out_len = 4;
  value = 1;
  ret = bcmf_sdpcm_iovar_request(priv, CHIP_STA_INTERFACE, true,
                                 IOVAR_STR_ROAM_OFF, (uint8_t*)&value,
                                 &out_len);

  // FIXME remove

  /* Try scan */

  value = 0;
  out_len = 4;
  ret = bcmf_sdpcm_ioctl(priv, CHIP_STA_INTERFACE, true,
                         WLC_SET_PASSIVE_SCAN, (uint8_t*)&value, &out_len);
  bcmf_run_escan(priv);
  return OK;
}

int bcmf_run_escan(FAR struct bcmf_dev_s *priv)
{
  int ret;
  uint32_t out_len;

  /* Default request structure */

  struct wl_escan_params *params =
                  (struct wl_escan_params*)kmm_malloc(sizeof(*params));
  if (!params)
    {
      return -ENOMEM;
    }

  memset(params, 0, sizeof(*params));

  params->version = ESCAN_REQ_VERSION;
  params->action = WL_SCAN_ACTION_START;
  params->sync_id = 0x1234;

  
  memset(&params->params.bssid, 0xFF, sizeof(params->params.bssid));
  params->params.bss_type = DOT11_BSSTYPE_ANY;
  params->params.scan_type = 0; /* Active scan */
  params->params.nprobes = -1;
  params->params.active_time = -1;
  params->params.passive_time = -1;
  params->params.home_time = -1;

  params->params.channel_num = 0;

  _info("start scan\n");

  out_len = sizeof(*params);
  ret = bcmf_sdpcm_iovar_request(priv, CHIP_STA_INTERFACE, true,
                                 IOVAR_STR_ESCAN, (uint8_t*)params,
                                 &out_len);

  free(params);

  if (ret != OK)
    {
      return -EIO;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int bcmf_wl_initialize(FAR struct bcmf_dev_s *priv)
{
  int ret;
  uint32_t out_len;
  uint8_t tmp_buf[64];

  /* Disable TX Gloming feature */

  out_len = 4;
  *(uint32_t*)tmp_buf = 0;
  ret = bcmf_sdpcm_iovar_request(priv, CHIP_STA_INTERFACE, false,
                                 IOVAR_STR_TX_GLOM, tmp_buf,
                                 &out_len);
  if (ret != OK)
    {
      return -EIO;
    }

  /* Query MAC address */

  out_len = 6;
  ret = bcmf_sdpcm_iovar_request(priv, CHIP_STA_INTERFACE, false,
                                 IOVAR_STR_CUR_ETHERADDR, tmp_buf,
                                 &out_len);
  if (ret != OK)
    {
      return -EIO;
    }

  memcpy(priv->mac_addr, tmp_buf, 6);

  _info("MAC address is %02X:%02X:%02X:%02X:%02X:%02X\n",
                        tmp_buf[0], tmp_buf[1], tmp_buf[2],
                        tmp_buf[3], tmp_buf[4], tmp_buf[5]);
  
  /* Query firmware version string */

  out_len = sizeof(tmp_buf);
  ret = bcmf_sdpcm_iovar_request(priv, CHIP_STA_INTERFACE, false,
                                 IOVAR_STR_VERSION, tmp_buf,
                                 &out_len);
  if (ret != OK)
    {
      return -EIO;
    }

  /* Remove line feed */
  out_len = strlen((char*)tmp_buf);
  if (out_len > 0 && tmp_buf[out_len-1] == '\n')
    {
      tmp_buf[out_len-1] = 0;
    }

  _info("fw version <%s>\n", tmp_buf);

  /* FIXME Configure event mask to enable all asynchronous events */

  uint8_t event_mask[16];
  memset(event_mask, 0xff, sizeof(event_mask));

  out_len = sizeof(event_mask);
  ret = bcmf_sdpcm_iovar_request(priv, CHIP_STA_INTERFACE, true,
                                 IOVAR_STR_EVENT_MSGS, event_mask,
                                 &out_len);
  if (ret != OK)
    {
      return -EIO;
    }
  // TODO Create a wlan device name and register network driver

  return bcmf_dongle_initialize(priv);
}