/****************************************************************************
 * drivers/wireless/bluetooth/bt_uart.c
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: 2016, Intel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/net/bluetooth.h>

#include <nuttx/wireless/bluetooth/bt_core.h>
#include <nuttx/wireless/bluetooth/bt_hci.h>
#include <nuttx/wireless/bluetooth/bt_driver.h>

#include "bt_uart.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static ssize_t btuart_read(FAR struct btuart_upperhalf_s *upper,
                           FAR uint8_t *buffer, size_t buflen)
{
  FAR const struct btuart_lowerhalf_s *lower;
  ssize_t ntotal = 0;
  ssize_t nread;

  wlinfo("buflen %zu\n", buflen);

  DEBUGASSERT(upper != NULL && upper->lower != NULL);
  lower = upper->lower;
  DEBUGASSERT(lower->read != NULL);

  while (buflen > 0)
    {
      nread = lower->read(lower, buffer, buflen);
      if (nread == -EINTR)
        {
          continue;
        }
      else if (nread <= 0)
        {
          wlwarn("Returned error %zd\n", nread);
          return ntotal > 0 ? ntotal : nread;
        }

      wlinfo("read %zd remaining %zu\n", nread, buflen - nread);

      buflen -= nread;
      ntotal += nread;
      buffer += nread;
    }

  return ntotal;
}

static void btuart_rxwork(FAR void *arg)
{
  FAR struct btuart_upperhalf_s *upper;
  enum bt_buf_type_e type;
  unsigned int pktlen;
  ssize_t nread;
  union
    {
      struct bt_hci_evt_hdr_s evt;
      struct bt_hci_acl_hdr_s acl;
      struct bt_hci_iso_hdr_s iso;
    }

  *hdr;

  upper = (FAR struct btuart_upperhalf_s *)arg;

  nread = btuart_read(upper, &upper->rxbuf[upper->rxlen],
                      sizeof(upper->rxbuf) - upper->rxlen);
  if (nread <= 0)
    {
      wlerr("ERROR: btuart_read failed: %zd\n", nread);
      return;
    }

  upper->rxlen += (uint16_t)nread;

  while (upper->rxlen)
    {
      hdr = (FAR void *)&upper->rxbuf[H4_HEADER_SIZE];

      switch (upper->rxbuf[0])
        {
        case H4_EVT:
          if (upper->rxlen < H4_HEADER_SIZE +
              sizeof(struct bt_hci_evt_hdr_s))
            {
              wlwarn("WARNING: Incomplete HCI event header\n");
              return;
            }

          type = BT_EVT;
          pktlen = H4_HEADER_SIZE +
                   sizeof(struct bt_hci_evt_hdr_s) + hdr->evt.len;
          break;

        case H4_ACL:
          if (upper->rxlen < H4_HEADER_SIZE +
              sizeof(struct bt_hci_acl_hdr_s))
            {
              wlwarn("WARNING: Incomplete HCI ACL header\n");
              return;
            }

          type = BT_ACL_IN;
          pktlen = H4_HEADER_SIZE +
                   sizeof(struct bt_hci_acl_hdr_s) + hdr->acl.len;
          break;

        case H4_ISO:
          if (upper->rxlen < H4_HEADER_SIZE +
              sizeof(struct bt_hci_iso_hdr_s))
            {
              wlwarn("WARNING: Incomplete HCI ISO header\n");
              return;
            }

          type = BT_ISO_IN;
          pktlen = H4_HEADER_SIZE +
                   sizeof(struct bt_hci_iso_hdr_s) + hdr->iso.len;
          break;

        default:
          wlerr("ERROR: Unknown H4 type %u\n", upper->rxbuf[0]);
          return;
        }

      if (upper->rxlen < pktlen)
        {
          wlwarn("WARNING: Incomplete packet: rxlen=%u, pktlen=%u\n",
                 upper->rxlen, pktlen);
          return;
        }

      /* Pass buffer to the stack */

      BT_DUMP("Received", upper->rxbuf, pktlen);
      bt_netdev_receive(&upper->dev, type, &upper->rxbuf[H4_HEADER_SIZE],
                        pktlen - H4_HEADER_SIZE);

      upper->rxlen -= pktlen;
      memmove(upper->rxbuf, upper->rxbuf + pktlen, upper->rxlen);
    }
}

static void btuart_rxcallback(FAR const struct btuart_lowerhalf_s *lower,
                              FAR void *arg)
{
  FAR struct btuart_upperhalf_s *upper;

  DEBUGASSERT(lower != NULL && arg != NULL);
  upper = (FAR struct btuart_upperhalf_s *)arg;

  int ret = work_queue(HPWORK, &upper->work, btuart_rxwork, arg, 0);
  if (ret < 0)
    {
      wlerr("ERROR: work_queue failed: %d\n", ret);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int btuart_send(FAR struct bt_driver_s *dev,
                enum bt_buf_type_e type,
                FAR void *data, size_t len)
{
  FAR uint8_t *hdr = (FAR uint8_t *)data - dev->head_reserve;
  FAR struct btuart_upperhalf_s *upper;
  FAR const struct btuart_lowerhalf_s *lower;
  ssize_t ntotal = 0;

  upper = (FAR struct btuart_upperhalf_s *)dev;
  DEBUGASSERT(upper != NULL && upper->lower != NULL);
  lower = upper->lower;

  if (type == BT_CMD)
    {
      *hdr = H4_CMD;
    }
  else if (type == BT_ACL_OUT)
    {
      *hdr = H4_ACL;
    }
  else if (type == BT_ISO_OUT)
    {
      *hdr = H4_ISO;
    }
  else
    {
      return -EINVAL;
    }

  len += H4_HEADER_SIZE;

  BT_DUMP("Sending", hdr, len);

  while (ntotal < len)
    {
      ssize_t nwritten;

      nwritten = lower->write(lower, hdr + ntotal, len - ntotal);
      if (nwritten >= 0)
        {
          ntotal += nwritten;
        }
      else if (nwritten != -EINTR)
        {
          return nwritten;
        }
    }

  return OK;
}

int btuart_open(FAR struct bt_driver_s *dev)
{
  FAR struct btuart_upperhalf_s *upper;
  FAR const struct btuart_lowerhalf_s *lower;

  wlinfo("dev %p\n", dev);

  upper = (FAR struct btuart_upperhalf_s *)dev;
  DEBUGASSERT(upper != NULL && upper->lower != NULL);
  lower = upper->lower;

  /* Drain any cached Rx data */

  lower->rxdrain(lower);

  /* Attach the Rx event handler */

  lower->rxattach(lower, btuart_rxcallback, upper);

  /* Re-enable Rx callbacks */

  lower->rxenable(lower, true);
  return OK;
}

void btuart_close(FAR struct bt_driver_s *dev)
{
  FAR struct btuart_upperhalf_s *upper;
  FAR const struct btuart_lowerhalf_s *lower;

  upper = (FAR struct btuart_upperhalf_s *)dev;
  DEBUGASSERT(upper != NULL && upper->lower != NULL);
  lower = upper->lower;

  /* Disable Rx callbacks */

  lower->rxenable(lower, false);

  /* Detach the Rx event handler */

  lower->rxattach(lower, NULL, NULL);
}

int btuart_ioctl(FAR struct bt_driver_s *dev,
                 int cmd, unsigned long arg)
{
  FAR struct btuart_upperhalf_s *upper;
  FAR const struct btuart_lowerhalf_s *lower;

  upper = (FAR struct btuart_upperhalf_s *)dev;
  DEBUGASSERT(upper != NULL && upper->lower != NULL);
  lower = upper->lower;

  if (lower->ioctl)
    {
      return lower->ioctl(lower, cmd, arg);
    }
  else
    {
      return -ENOTTY;
    }
}

/****************************************************************************
 * Name: btuart_register
 *
 * Description:
 *   Register the UART-based bluetooth driver.
 *
 * Input Parameters:
 *   lower - an instance of the lower half driver interface
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

int btuart_register(FAR const struct btuart_lowerhalf_s *lower)
{
  FAR struct bt_driver_s *driver;
  int ret;

  ret = btuart_create(lower, &driver);
  if (ret < 0)
    {
      return ret;
    }

  ret = bt_driver_register(driver);
  if (ret < 0)
    {
      wlerr("ERROR: bt_driver_register failed: %d\n", ret);
      kmm_free(driver);
    }

  return ret;
}
