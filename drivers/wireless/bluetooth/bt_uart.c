/****************************************************************************
 * drivers/wireless/bluetooth/bt_uart.c
 *
 *   Copyright (c) 2016, Intel Corporation
 *   All rights reserved.
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
                           FAR uint8_t *buffer, size_t buflen,
                           size_t minread)
{
  FAR const struct btuart_lowerhalf_s *lower;
  ssize_t ntotal = 0;
  ssize_t nread;

  wlinfo("buflen %zu minread %zu\n", buflen, minread);

  DEBUGASSERT(upper != NULL && upper->lower != NULL);
  lower = upper->lower;
  DEBUGASSERT(lower->read != NULL);

  while (buflen > 0)
    {
      nread = lower->read(lower, buffer, buflen);
      if (nread == 0 || nread == -EINTR)
        {
          wlwarn("Got zero bytes from UART\n");
          if (ntotal < minread)
            {
              continue;
            }

          break;
        }
      else if (nread < 0)
        {
          wlwarn("Returned error %zd\n", nread);
          return nread;
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
  uint8_t data[BLUETOOTH_MAX_FRAMELEN];
  enum bt_buf_type_e type;
  unsigned int hdrlen;
  unsigned int pktlen;
  ssize_t nread;
  union
    {
      struct bt_hci_evt_hdr_s evt;
      struct bt_hci_acl_hdr_s acl;
    }

  *hdr;

  upper = (FAR struct btuart_upperhalf_s *)arg;

  /* Beginning of a new packet.
   * Read the first byte to get the packet type.
   */

  nread = btuart_read(upper, data, H4_HEADER_SIZE, 0);
  if (nread != H4_HEADER_SIZE)
    {
      wlwarn("WARNING: Unable to read H4 packet type: %zd\n", nread);
      goto errout_with_busy;
    }

  if (data[0] == H4_EVT)
    {
      hdrlen = sizeof(struct bt_hci_evt_hdr_s);
    }
  else if (data[0] == H4_ACL)
    {
      hdrlen = sizeof(struct bt_hci_acl_hdr_s);
    }
  else
    {
      wlerr("ERROR: Unknown H4 type %u\n", data[0]);
      goto errout_with_busy;
    }

  nread = btuart_read(upper, data + H4_HEADER_SIZE,
                      hdrlen, hdrlen);
  if (nread != hdrlen)
    {
      wlwarn("WARNING: Unable to read H4 packet header: %zd\n", nread);
      goto errout_with_busy;
    }

  hdr = (void *)(data + H4_HEADER_SIZE);

  if (data[0] == H4_EVT)
    {
      pktlen = hdr->evt.len;
      type = BT_EVT;
    }
  else if (data[0] == H4_ACL)
    {
      pktlen = hdr->acl.len;
      type = BT_ACL_IN;
    }
  else
    {
      wlerr("ERROR: Unknown H4 type %u\n", data[0]);
      goto errout_with_busy;
    }

  nread = btuart_read(upper, data + H4_HEADER_SIZE + hdrlen,
                      pktlen, pktlen);
  if (nread != pktlen)
    {
      wlwarn("WARNING: Unable to read H4 packet: %zd\n", nread);
      goto errout_with_busy;
    }

  /* Pass buffer to the stack */

  BT_DUMP("Received", data, H4_HEADER_SIZE + hdrlen + pktlen);
  upper->busy = false;
  bt_netdev_receive(&upper->dev, type, data + H4_HEADER_SIZE,
                    hdrlen + pktlen);
  return;

errout_with_busy:
  upper->busy = false;
}

static void btuart_rxcallback(FAR const struct btuart_lowerhalf_s *lower,
                              FAR void *arg)
{
  FAR struct btuart_upperhalf_s *upper;

  DEBUGASSERT(lower != NULL && arg != NULL);
  upper = (FAR struct btuart_upperhalf_s *)arg;

  if (!upper->busy)
    {
      upper->busy = true;
      int ret = work_queue(HPWORK, &upper->work, btuart_rxwork, arg, 0);
      if (ret < 0)
        {
          upper->busy = false;
          wlerr("ERROR: work_queue failed: %d\n", ret);
        }
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
