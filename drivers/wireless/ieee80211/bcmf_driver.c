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

#include <net/ethernet.h>

#include <nuttx/kmalloc.h>
#include <nuttx/wdog.h>

#include "bcmf_driver.h"
#include "bcmf_cdc.h"
#include "bcmf_ioctl.h"
#include "bcmf_utils.h"

#include <nuttx/sdio.h>
#include "bcmf_sdio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

// TODO move elsewhere
#define DOT11_BSSTYPE_ANY     2

#define BCMF_SCAN_TIMEOUT_TICK (5*CLOCKS_PER_SEC)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* AP scan state machine status */

enum
{
  BCMF_SCAN_TIMEOUT = 0,
  BCMF_SCAN_DISABLED,
  BCMF_SCAN_RUN,
  BCMF_SCAN_DONE
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static FAR struct bcmf_dev_s* bcmf_allocate_device(void);
static void bcmf_free_device(FAR struct bcmf_dev_s *priv);

static int bcmf_driver_initialize(FAR struct bcmf_dev_s *priv);

// FIXME add bcmf_netdev.h file
int bcmf_netdev_register(FAR struct bcmf_dev_s *priv);

// FIXME only for debug purpose
static void bcmf_wl_default_event_handler(FAR struct bcmf_dev_s *priv,
                            struct bcmf_event_s *event, unsigned int len);

static void bcmf_wl_radio_event_handler(FAR struct bcmf_dev_s *priv,
                            struct bcmf_event_s *event, unsigned int len);

static void bcmf_wl_scan_event_handler(FAR struct bcmf_dev_s *priv,
                            struct bcmf_event_s *event, unsigned int len);

#if 0
static int bcmf_run_escan(FAR struct bcmf_dev_s *priv);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

FAR struct bcmf_dev_s* bcmf_allocate_device(void)
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

  if ((ret = sem_init(&priv->control_mutex, 0, 1)) != OK)
    {
      goto exit_free_priv;
    }

  if ((ret = sem_init(&priv->control_timeout, 0, 0)) != OK)
    {
      goto exit_free_priv;
    }

  if ((ret = sem_setprotocol(&priv->control_timeout, SEM_PRIO_NONE)) != OK)
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

int bcmf_wl_set_mac_address(FAR struct bcmf_dev_s *priv, uint8_t *addr)
{
  int ret;
  uint32_t out_len = 6;

  ret = bcmf_cdc_iovar_request(priv, CHIP_STA_INTERFACE, true,
                                 IOVAR_STR_CUR_ETHERADDR, addr,
                                 &out_len);
  if (ret != OK)
    {
      return ret;
    }

  wlinfo("MAC address updated %02X:%02X:%02X:%02X:%02X:%02X\n",
                            addr[0], addr[1], addr[2],
                            addr[3], addr[4], addr[5]);
  memcpy(priv->bc_dev.d_mac.ether.ether_addr_octet, addr, ETHER_ADDR_LEN);
    
  return OK;
}

int bcmf_driver_initialize(FAR struct bcmf_dev_s *priv)
{
  int ret;
  uint32_t out_len;
  uint32_t value;
  uint8_t tmp_buf[64];

  /* Disable TX Gloming feature */

  out_len = 4;
  *(uint32_t*)tmp_buf = 0;
  ret = bcmf_cdc_iovar_request(priv, CHIP_STA_INTERFACE, false,
                                 IOVAR_STR_TX_GLOM, tmp_buf,
                                 &out_len);
  if (ret != OK)
    {
      return -EIO;
    }

  /* FIXME disable power save mode */

  out_len = 4;
  value = 0;
  ret = bcmf_cdc_ioctl(priv, CHIP_STA_INTERFACE, true,
                         WLC_SET_PM, (uint8_t*)&value, &out_len);
  if (ret != OK)
    {
      return ret;
    }

  /* Set the GMode to auto */

  out_len = 4;
  value = GMODE_AUTO;
  ret = bcmf_cdc_ioctl(priv, CHIP_STA_INTERFACE, true,
                         WLC_SET_GMODE, (uint8_t*)&value, &out_len);
  if (ret != OK)
    {
      return ret;
    }

  /* TODO configure roaming if needed. Disable for now */

  out_len = 4;
  value = 1;
  ret = bcmf_cdc_iovar_request(priv, CHIP_STA_INTERFACE, true,
                                 IOVAR_STR_ROAM_OFF, (uint8_t*)&value,
                                 &out_len);
  
  /* Query firmware version string */

  out_len = sizeof(tmp_buf);
  ret = bcmf_cdc_iovar_request(priv, CHIP_STA_INTERFACE, false,
                                 IOVAR_STR_VERSION, tmp_buf,
                                 &out_len);
  if (ret != OK)
    {
      return -EIO;
    }

  tmp_buf[sizeof(tmp_buf)-1] = 0;

  /* Remove line feed */

  out_len = strlen((char*)tmp_buf);
  if (out_len > 0 && tmp_buf[out_len-1] == '\n')
    {
      tmp_buf[out_len-1] = 0;
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
  wlinfo("Got radio event %d from <%s>\n", bcmf_getle32(&event->type),
                                           event->src_name);
}

void bcmf_wl_scan_event_handler(FAR struct bcmf_dev_s *priv,
                                   struct bcmf_event_s *event, unsigned int len)
{
  uint32_t status;
  uint32_t reason;
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
  reason = bcmf_getle32(&event->reason);
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

  result = (struct wl_escan_result*)&event[1];

  if (len < result->buflen || result->buflen < sizeof(struct wl_escan_result))
    {
      goto exit_invalid_frame;
    }

  /* wl_escan_result already cointains a wl_bss_info field */

  len = result->buflen - sizeof(struct wl_escan_result)
                       + sizeof(struct wl_bss_info);

  /* Process bss_infos */

  bss = result->bss_info;

  do
    {
      bss_info_len = bss->length;

      if (len < bss_info_len)
        {
          wlerr("bss_len error %d %d\n", len, bss_info_len);
          goto exit_invalid_frame;
        }

      wlinfo("Scan result: <%.32s> %02x:%02x:%02x:%02x:%02x:%02x\n",
             bss->SSID, bss->BSSID.octet[0], bss->BSSID.octet[1],
                        bss->BSSID.octet[2], bss->BSSID.octet[3],
                        bss->BSSID.octet[4], bss->BSSID.octet[5]);

      /* Process next bss_info */

      len -= bss_info_len;
      bss = (struct wl_bss_info*)((uint8_t*)bss + bss_info_len);
      bss_count += 1;
    }
  while (len > 0 && bss_count < result->bss_count);

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

  wlinfo("escan done event %d %d\n", status, reason);

  wd_cancel(priv->scan_timeout);

  if (!priv->scan_params)
    {
      /* Scan has already timedout */

      return;
    }

  free(priv->scan_params);
  priv->scan_params = NULL;
  priv->scan_status = BCMF_SCAN_DONE;
  sem_post(&priv->control_mutex);

  return;

exit_invalid_frame:
  wlerr("Invalid scan result event\n");
  bcmf_hexdump((uint8_t*)event, event_len, (unsigned long)event);
}

void bcmf_wl_scan_timeout(int argc, wdparm_t arg1, ...)
{
  FAR struct bcmf_dev_s *priv = (FAR struct bcmf_dev_s*)arg1;

  if (priv->scan_status < BCMF_SCAN_RUN)
    {
        /* Fatal error, invalid scan status */
        wlerr("Unexpected scan timeout\n");
        return;
    }

  wlerr("Scan timeout detected\n");

  priv->scan_status = BCMF_SCAN_TIMEOUT;
  free(priv->scan_params);
  priv->scan_params = NULL;
  sem_post(&priv->control_mutex);
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

  if (ret == OK)
    {
      /* TODO update device state */
    }

  return ret;
}

int bcmf_wl_start_scan(FAR struct bcmf_dev_s *priv)
{
  int ret;
  uint32_t out_len;
  uint32_t value;

  wlinfo("Enter\n");

  /* Set active scan mode */

  value = 0;
  out_len = 4;
  if (bcmf_cdc_ioctl(priv, CHIP_STA_INTERFACE, true,
                         WLC_SET_PASSIVE_SCAN, (uint8_t*)&value, &out_len))
    {
      return -EIO;
    }

  /* Lock control_mutex semaphore */

  if ((ret = sem_wait(&priv->control_mutex)) != OK)
   {
      return ret;
   }

  /* Default request structure */

  priv->scan_params = (struct wl_escan_params*)
                      kmm_malloc(sizeof(*priv->scan_params));
  if (!priv->scan_params)
    {
      ret = -ENOMEM;
      goto exit_sem_post;
    }

  memset(priv->scan_params, 0, sizeof(*priv->scan_params));

  priv->scan_params->version = ESCAN_REQ_VERSION;
  priv->scan_params->action = WL_SCAN_ACTION_START;
  priv->scan_params->sync_id = 0xabcd; /* Not used for now */

  memset(&priv->scan_params->params.bssid, 0xFF,
          sizeof(priv->scan_params->params.bssid));
  priv->scan_params->params.bss_type = DOT11_BSSTYPE_ANY;
  priv->scan_params->params.scan_type = 0; /* Active scan */
  priv->scan_params->params.nprobes = -1;
  priv->scan_params->params.active_time = -1;
  priv->scan_params->params.passive_time = -1;
  priv->scan_params->params.home_time = -1;
  priv->scan_params->params.channel_num = 0;

  wlinfo("start scan\n");

  priv->scan_status = BCMF_SCAN_RUN;

  out_len = sizeof(*priv->scan_params);


  if (bcmf_cdc_iovar_request_unsafe(priv, CHIP_STA_INTERFACE, true,
                                 IOVAR_STR_ESCAN, (uint8_t*)priv->scan_params,
                                 &out_len))
    {
      ret = -EIO;
      goto exit_free_params;
    }

  /*  Start scan_timeout timer */

  wd_start(priv->scan_timeout, BCMF_SCAN_TIMEOUT_TICK,
           bcmf_wl_scan_timeout, (wdparm_t)priv);

  return OK;

exit_free_params:
  free(priv->scan_params);
  priv->scan_params = NULL;
exit_sem_post:
  sem_post(&priv->control_mutex);
  priv->scan_status = BCMF_SCAN_DISABLED;
  return ret;
}

int bcmf_wl_is_scan_done(FAR struct bcmf_dev_s *priv)
{
  if (priv->scan_status == BCMF_SCAN_RUN)
    {
      return -EAGAIN;
    }
  if (priv->scan_status == BCMF_SCAN_DONE)
    {
      return OK;
    }
  return -EINVAL;
}