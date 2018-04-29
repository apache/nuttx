/****************************************************************************
 * drivers/wireless/bluetooth/bt_uart.c
 * UART based Bluetooth driver
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Ported from the Intel/Zephyr arduino101_firmware_source-v1.tar package
 * where the code was released with a compatible 3-clause BSD license:
 *
 *   Copyright (c) 2016, Intel Corporation
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
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

#include <nuttx/wireless/bluetooth/bt_core.h>
#include <nuttx/wireless/bluetooth/bt_hci.h>
#include <nuttx/wireless/bluetooth/bt_driver.h>
#include <nuttx/wireless/bluetooth/bt_uart.h>

#include "bt_uart.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static ssize_t btuart_read(FAR struct btuart_upperhalf_s *upper,
                           FAR uint8_t *buffer, size_t buflen,
                           size_t minread)
{
  FAR const struct btuart_lowerhalf_s *lower;
  ssize_t nread;
  ssize_t ntotal = 0;

  wlinfo("buflen %lu minread %lu\n",
         (unsigned long)buflen, (unsigned long)minread);

  DEBUGASSERT(upper != NULL && upper->lower != NULL);
  lower = upper->lower;
  DEBUGASSERT(lower->read != NULL);

  while (buflen > 0)
    {
      nread = lower->read(lower, buffer, buflen);
      if (nread == 0)
        {
          wlinfo("Got zero bytes from UART\n");
          if (ntotal < minread)
            {
              continue;
            }

          break;
        }
      else if (nread < 0)
        {
          return nread;
        }

      wlinfo("read %ld remaining %lu\n",
             (long)nread, (unsigned long)(buflen - nread));

      buflen -= nread;
      ntotal += nread;
      buffer += nread;
    }

  return ntotal;
}

static FAR struct bt_buf_s *btuart_evt_recv(FAR struct btuart_upperhalf_s *upper,
                                            FAR unsigned int *hdrlen)
{
  FAR struct bt_buf_s *buf;
  struct bt_hci_evt_hdr_s hdr;
  ssize_t nread;

  /* We can ignore the return value since we pass buflen == minread */

  nread = btuart_read(upper, (FAR uint8_t *)&hdr,
                      sizeof(struct bt_hci_evt_hdr_s),
                      sizeof(struct bt_hci_evt_hdr_s));

  if (nread != sizeof(struct bt_hci_evt_hdr_s))
    {
      wlerr("ERROR: btuart_read returned %ld\n", (long)nread);
      return NULL;
    }

  buf = bt_buf_alloc(BT_EVT, NULL, 0);
  if (buf != NULL)
    {
      memcpy(bt_buf_extend(buf, sizeof(struct bt_hci_evt_hdr_s)), &hdr,
             sizeof(struct bt_hci_evt_hdr_s));
    }
  else
    {
      wlerr("ERROR: No available event buffers!\n");
    }

  *hdrlen = (int)hdr.len;

  wlinfo("hdrlen %u\n", hdr.len);
  return buf;
}

static FAR struct bt_buf_s *btuart_acl_recv(FAR struct btuart_upperhalf_s *upper,
                                            FAR unsigned int *hdrlen)
{
  FAR struct bt_buf_s *buf;
  struct bt_hci_acl_hdr_s hdr;
  ssize_t nread;

  /* We can ignore the return value since we pass buflen == minread */

  nread = btuart_read(upper, (FAR uint8_t *)&hdr,
                      sizeof(struct bt_hci_acl_hdr_s),
                      sizeof(struct bt_hci_acl_hdr_s));

  if (nread != sizeof(struct bt_hci_acl_hdr_s))
    {
      wlerr("ERROR: btuart_read returned %ld\n", (long)nread);
      return NULL;
    }

  buf = bt_buf_alloc(BT_ACL_IN, NULL, 0);
  if (buf != NULL)
    {
      memcpy(bt_buf_extend(buf, sizeof(struct bt_hci_acl_hdr_s)), &hdr,
             sizeof(struct bt_hci_acl_hdr_s));
    }
  else
    {
      wlerr("ERROR: No available ACL buffers!\n");
    }

  *hdrlen = BT_LE162HOST(hdr.len);

  wlinfo("hdrlen %u\n", *hdrlen);
  return buf;
}

static void btuart_rxwork(FAR void *arg)
{
  FAR struct btuart_upperhalf_s *upper;
  FAR const struct btuart_lowerhalf_s *lower;
  static FAR struct bt_buf_s *buf;
  static unsigned int hdrlen;
  static int remaining;
  ssize_t nread;
  uint8_t type;

  upper = (FAR struct btuart_upperhalf_s *)arg;
  DEBUGASSERT(upper != NULL && upper->lower != NULL);
  lower = upper->lower;

  /* Beginning of a new packet.  Read the first byte to get the packet type. */

  buf    = NULL;
  hdrlen = 0;

  nread = btuart_read(upper, &type, 1, 0);
  if (nread != 1)
    {
      wlwarn("WARNING: Unable to read H4 packet type: %ld\n",
             (long)nread);
      goto errout_with_busy;
    }

  switch (type)
    {
      case H4_EVT:
        buf = btuart_evt_recv(upper, &hdrlen);
        break;

      case H4_ACL:
        buf = btuart_acl_recv(upper, &hdrlen);
        break;

      default:
        wlerr("ERROR: Unknown H4 type %u\n", type);
        goto errout_with_busy;
    }

  if (buf == NULL)
    {
      /* Failed to allocate a buffer.  Drain the Rx data and fail the read. */

      nread = lower->rxdrain(lower);
      wlwarn("WARNING: Discarded %ld bytes\n", (long)nread);
      goto errout_with_busy;
    }
  else if ((hdrlen - 1) > bt_buf_tailroom(buf))
    {
      wlerr("ERROR: Not enough space in buffer\n");
      goto errout_with_buf;
    }

  remaining = hdrlen;
  wlinfo("Need to get %u bytes\n", remaining);

  while (remaining > 0)
    {
      nread = btuart_read(upper, bt_buf_tail(buf), remaining, 0);
      wlinfo("Received %ld bytes\n", (long)nread);

      buf->len  += nread;
      remaining -= nread;
    }

  wlinfo("Full packet received\n");

  /* Drain any un-read bytes from the Rx buffer */

  nread = lower->rxdrain(lower);
  if (nread > 0)
    {
      wlwarn("WARNING: Discarded %ld bytes\n", (long)nread);
    }

  /* Pass buffer to the stack */

  upper->busy = false;

  BT_DUMP("Received",  buf->data, buf->len);
  bt_hci_receive(buf);
  return;

errout_with_buf:
  bt_buf_release(buf);

errout_with_busy:
  upper->busy = false;
  return;
}

static void btuart_rxcallback(FAR const struct btuart_lowerhalf_s *lower,
                              FAR void *arg)
{
  FAR struct btuart_upperhalf_s *upper;

  DEBUGASSERT(lower != NULL && arg != NULL);
  upper = (FAR struct btuart_upperhalf_s *)arg;

  if (!upper->busy)
    {
      int ret = work_queue(HPWORK, &upper->work, btuart_rxwork, arg, 0);
      if (ret < 0)
        {
          wlerr("ERROR: work_queue failed: %d\n", ret);
        }
      else
        {
          upper->busy = true;
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int btuart_send(FAR const struct bt_driver_s *dev, FAR struct bt_buf_s *buf)
{
  FAR struct btuart_upperhalf_s *upper;
  FAR const struct btuart_lowerhalf_s *lower;
  FAR uint8_t *type;
  ssize_t nwritten;

  upper = (FAR struct btuart_upperhalf_s *)dev;
  DEBUGASSERT(upper != NULL && upper->lower != NULL);
  lower = upper->lower;

  if (bt_buf_headroom(buf) < H4_HEADER_SIZE)
    {
      wlerr("Not enough headroom in buffer\n");
      return -EINVAL;
    }

  type = bt_buf_provide(buf, 1);

  switch (buf->type)
    {
      case BT_CMD:
        *type = H4_CMD;
        break;

      case BT_ACL_OUT:
        *type = H4_ACL;
        break;

      case BT_EVT:
        *type = H4_EVT;
        break;

      default:
        wlerr("Unknown buf type %u\n", buf->type);
        return -EINVAL;
    }

  BT_DUMP("Sending",  buf->data, buf->len);

  nwritten = lower->write(lower, buf->data, buf->len);
  if (nwritten == buf->len)
    {
      return OK;
    }

  if (nwritten < 0)
    {
      return (int)nwritten;
    }

  return -EIO;
}

int btuart_open(FAR const struct bt_driver_s *dev)
{
  FAR struct btuart_upperhalf_s *upper;
  FAR const struct btuart_lowerhalf_s *lower;

  wlinfo("dev %p\n", dev);

  upper = (FAR struct btuart_upperhalf_s *)dev;
  DEBUGASSERT(upper != NULL && upper->lower != NULL);
  lower = upper->lower;

  /* Disable Rx callbacks */

  lower->rxenable(lower, false);

  /* Drain any cached Rx data */

  (void)lower->rxdrain(lower);

  /* Attach the Rx event handler */

  lower->rxattach(lower, btuart_rxcallback, upper);

  /* Re-enable Rx callbacks */

  lower->rxenable(lower, true);
  return OK;
}
