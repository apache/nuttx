/****************************************************************************
 * wireless/bluetooth/bt_ioctl.c
 * Bluetooth network IOCTL handler
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/wqueue.h>
#include <nuttx/semaphore.h>
#include <nuttx/net/netdev.h>
#include <nuttx/wireless/bt_core.h>
#include <nuttx/wireless/bt_hci.h>
#include <nuttx/wireless/bt_ioctl.h>

#include "bt_hcicore.h"
#include "bt_ioctl.h"

#ifdef CONFIG_NETDEV_IOCTL  /* Not optional! */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure encapsulates all globals used by the IOCTL logic */

struct bt_scanstate_s
{
  sem_t bs_exclsem;                 /* Manages exclusive access */
  bool bs_scanning;                 /* True:  Scanning in progress */
  uint8_t bs_head;                  /* Head of circular list (for removal) */
  uint8_t bs_tail;                  /* Tail of circular list (for addition) */

  struct bt_scanresponse_s bs_rsp[CONFIG_BLUETOOTH_MAXSCANRESULT];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* At present only a single Bluetooth device is supported.  So we can simply
 * maintain the scan state as a global.
 */

static struct bt_scanstate_s g_scanstate;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bt_scan_callback
 *
 * Description:
 *   This is an HCI callback function.  HCI provides scan result data via
 *   this callback function.  The scan result data will be added to the
 *   cached scan results.
 *
 * Input Parameters:
 *   Scan result data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void bt_scan_callback(FAR const bt_addr_le_t *addr, int8_t rssi,
                             uint8_t adv_type, FAR const uint8_t *adv_data,
                             uint8_t len)
{
  FAR struct bt_scanresponse_s *rsp;
  uint8_t nexttail;
  uint8_t head;
  uint8_t tail;
  int ret;

  if (!g_scanstate.bs_scanning)
    {
      wlerr("ERROR:  Results received while not scanning\n");
      return;
    }

   if (len > CONFIG_BLUETOOTH_MAXSCANDATA)
     {
       wlerr("ERROR: Scan result is too big:  %u\n", len);
       return;
     }

  /* Get exclusive access to the scan data */

  while ((ret = nxsem_wait(&g_scanstate.bs_exclsem)) < 0)
    {
      DEBUGASSERT(ret == -EINTR || ret == -ECANCELED);
      if (ret != -EINTR)
        {
          return;
        }
    }

  /* Add the scan data to the cache */

  tail     = g_scanstate.bs_tail;
  nexttail = tail + 1;

  if (nexttail >= CONFIG_BLUETOOTH_MAXSCANRESULT)
    {
      nexttail = 0;
    }

  /* Is the circular buffer full? */

  head = g_scanstate.bs_head;
  if (nexttail == head)
    {
      wlerr("ERROR: Too many scan results\n");

      if (++head >= CONFIG_BLUETOOTH_MAXSCANRESULT)
        {
          head = 0;
        }

      g_scanstate.bs_head = head;
    }

  /* Save the new scan result */

  rsp = &g_scanstate.bs_rsp[tail];
  memcpy(&rsp->sr_addr, addr, sizeof(bt_addr_le_t));
  rsp->sr_rssi = rssi;
  rsp->sr_type = adv_type;
  rsp->sr_len  = len;
  memcpy(&rsp->sr_data, adv_data, len);

  g_scanstate.bs_tail = nexttail;
  nxsem_post(&g_scanstate.bs_exclsem);
}

/****************************************************************************
 * Name: bt_scan_result
 *
 * Description:
 *   This is an HCI callback function.  HCI provides scan result data via
 *   this callback function.  The scan result data will be added to the
 *   cached scan results.
 *
 * Input Parameters:
 *   Scan result data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int bt_scan_result(FAR struct bt_scanresult_s *result)
{
  uint8_t head;
  uint8_t tail;
  uint8_t maxrsp;
  uint8_t nrsp;
  int ret;

  if (!g_scanstate.bs_scanning)
    {
      wlerr("ERROR:  Results received while not scanning\n");
      return -EPIPE;
    }

  /* Get exclusive access to the scan data */

  ret = nxsem_wait(&g_scanstate.bs_exclsem);
  if (ret < 0)
    {
      DEBUGASSERT(ret == -EINTR || ret == -ECANCELED);
      return ret;
    }

  /* Copy all available results */

  head   = g_scanstate.bs_head;
  tail   = g_scanstate.bs_tail;
  maxrsp = result->sc_nrsp;

  for (nrsp = 0; nrsp < maxrsp && head != tail; nrsp++)
    {
      FAR const uint8_t *src;
      FAR uint8_t *dest;

      /* Copy data from the head index into the user buffer */

      src  = (FAR const uint8_t *)&g_scanstate.bs_rsp[head];
      dest = (FAR uint8_t *)&result->sc_rsp[nrsp];
      memcpy(dest, src, sizeof(struct bt_scanresponse_s));

      /* Increment the head index */

      if (++head >= CONFIG_BLUETOOTH_MAXSCANRESULT)
        {
          head = 0;
        }
    }

  g_scanstate.bs_head = head;
  result->sc_nrsp     = nrsp;
  nxsem_post(&g_scanstate.bs_exclsem);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bt_ioctl
 *
 * Description:
 *   Handle network IOCTL commands directed to this device.
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *   cmd - The IOCTL command
 *   arg - The argument for the IOCTL command
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int bt_ioctl(FAR struct net_driver_s *dev, int cmd, unsigned long arg)
{
  int ret;

  wlinfo("cmd=%04x arg=%ul\n", cmd, arg);
  DEBUGASSERT(dev != NULL && dev->d_private != NULL);

  switch (cmd)
    {
      /* SIOCBT_ADVERTISESTART
       *   Description:   Set advertisement data, scan response data,
       *                  advertisement parameters and start advertising.
       *   Input:         Pointer to read-write instance of struct
       *                  bt_advertisestart_s.
       *   Output:        None
       */

      case SIOCBT_ADVERTISESTART:
        {
          FAR struct bt_advertisestart_s *adv =
            (FAR struct bt_advertisestart_s *)((uintptr_t)arg);

          if (adv == NULL)
            {
              ret = -EINVAL;
            }
          else
            {
              ret = bt_start_advertising(adv->as_type, &adv->as_ad,
                                         &adv->as_sd);
              wlinfo("Start advertising: %d\n", ret);
            }
        }
        break;

      /* SIOCBT_ADVERTISESTOP
       *   Description:   Stop advertising.
       *   Input:         None
       *   Output:        None
       */

      case SIOCBT_ADVERTISESTOP:
        {
          wlinfo("Stop advertising\n");
          bt_stop_advertising();
          ret = OK;
        }
        break;

      /* SIOCBT_SCANSTART
       *   Description:   Start LE scanning.  Buffered scan results may be
       *                  obtained via SIOCBT_SCANGET
       *   Input:         1=Duplicate filtering enabled
       *   Output:        None
       */

      case SIOCBT_SCANSTART:
        {
          uint8_t dup_enable = (arg == 0) ? 0 : BT_LE_SCAN_FILTER_DUP_ENABLE;

          /* Are we already scanning? */

          if (g_scanstate.bs_scanning)
            {
              ret = -EBUSY;
            }
          else
            {
              /* Initialize scan state */

              nxsem_init(&g_scanstate.bs_exclsem, 0, 1);
              g_scanstate.bs_scanning = true;
              g_scanstate.bs_head     = 0;
              g_scanstate.bs_tail     = 0;

              ret = bt_start_scanning(dup_enable, bt_scan_callback);
              wlinfo("Start scan: %d\n", ret);

              if (ret < 0)
                {
                  nxsem_destroy(&g_scanstate.bs_exclsem);
                  g_scanstate.bs_scanning = false;
                }
            }
        }
        break;

      /* SIOCBT_SCANGET
       *   Description:   Return scan results buffered since the call time
       *                  that the SIOCBT_SCANGET command was invoked.
       *   Input:         A reference to a write-able instance of struct
       *                  bt_scanresult_s.
       *   Output:        Buffered scan result results are returned in the
       *                  user-provided buffer space.
       */

      case SIOCBT_SCANGET:
        {
          FAR struct bt_scanresult_s *result =
            (FAR struct bt_scanresult_s *)((uintptr_t)arg);

          if (result == NULL)
            {
              ret = -EINVAL;
            }
          else
            {
              ret = bt_scan_result(result);
              wlinfo("Get scan results: %d\n", ret);
            }
        }
        break;

      /* SIOCBT_SCANSTOP
       *   Description:   Stop LE scanning and discard any buffered results.
       *   Input:         None
       *   Output:        None
       */

      case SIOCBT_SCANSTOP:
        {
          /* Stop scanning */

          ret = bt_stop_scanning();
          wlinfo("Stop scanning: %d\n", ret);

          nxsem_destroy(&g_scanstate.bs_exclsem);
          g_scanstate.bs_scanning = false;
        }
        break;

      default:
        wlwarn("WARNING: Unrecognized IOCTL command: %02x\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

#endif /* CONFIG_NETDEV_IOCTL */
