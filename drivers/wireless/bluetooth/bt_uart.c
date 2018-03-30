/****************************************************************************
 * drivers/wireless/bluetooth/btuart.c
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

#include <stddef.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/wireless/bt_core.h>
#include <nuttx/wireless/bt_hci.h>
#include <nuttx/wireless/bt_driver.h>
#include <nuttx/wireless/bt_uart.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define H4_HEADER_SIZE  1

#define H4_CMD          0x01
#define H4_ACL          0x02
#define H4_SCO          0x03
#define H4_EVT          0x04

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct btuart_upperhalf_s
{
  /* This structure must appear first in the structure so that this structure
   * is cast compatible with struct bt_driver_s.
   */

  struct bt_driver_s dev;

  /* The cached lower half interface */

  FAR const struct btuart_lowerhalf_s *lower;
};

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
                                            FAR int *remaining)
{
  FAR struct bt_buf_s *buf;
  struct bt_hci_evt_hdr_s hdr;
  ssize_t nread;

  /* We can ignore the return value since we pass buflen == minread */

  nread = btuart_read(upper, (FAR uint8_t *)&hdr,
                      sizeof(struct bt_hci_evt_hdr_s),
                      sizeof(struct bt_hci_evt_hdr_s));

  if (nread < 0)
    {
      return NULL;
    }

  *remaining = hdr.len;

  buf = bt_buf_get(BT_EVT, 0);
  if (buf != NULL)
    {
      memcpy(bt_buf_add(buf, sizeof(struct bt_hci_evt_hdr_s)), &hdr,
             sizeof(struct bt_hci_evt_hdr_s));
    }
  else
    {
      wlerr("ERROR: No available event buffers!\n");
    }

  wlinfo("len %u\n", hdr.len);
  return buf;
}

static FAR struct bt_buf_s *btuart_acl_recv(FAR struct btuart_upperhalf_s *upper,
                                            FAR int *remaining)
{
  FAR struct bt_buf_s *buf;
  struct bt_hci_acl_hdr_s hdr;
  ssize_t nread;

  /* We can ignore the return value since we pass buflen == minread */

  nread = btuart_read(upper, (FAR uint8_t *)&hdr,
                      sizeof(struct bt_hci_acl_hdr_s),
                      sizeof(struct bt_hci_acl_hdr_s));

  if (nread < 0)
    {
      return NULL;
    }

  buf = bt_buf_get(BT_ACL_IN, 0);
  if (buf)
    {
      memcpy(bt_buf_add(buf, sizeof(struct bt_hci_acl_hdr_s)), &hdr,
             sizeof(struct bt_hci_acl_hdr_s));
    }
  else
    {
      wlerr("ERROR: No available ACL buffers!\n");
    }

  *remaining = BT_LE162HOST(hdr.len);

  wlinfo("len %u\n", *remaining);
  return buf;
}

static void btuart_interrupt(FAR const struct btuart_lowerhalf_s *lower,
                             FAR void *arg)
{
  FAR struct btuart_upperhalf_s *upper;
  static FAR struct bt_buf_s *buf;
  static int remaining;
  ssize_t nread;

  DEBUGASSERT(lower != NULL && lower->rxdrain != NULL && arg != NULL);
  upper = (FAR struct btuart_upperhalf_s *)arg;

  /* Beginning of a new packet */

  while (remaining > 0)
    {
      uint8_t type;

     /* Get packet type */

      nread = btuart_read(upper, &type, 1, 0);
      if (nread != sizeof(type))
        {
          wlwarn("WARNING: Unable to read H4 packet type\n");
          continue;
        }

      switch (type)
        {
          case H4_EVT:
            buf = btuart_evt_recv(upper, &remaining);
            break;

          case H4_ACL:
            buf = btuart_acl_recv(upper, &remaining);
            break;

          default:
            wlerr("ERROR: Unknown H4 type %u\n", type);
            return;
        }

      if (buf != NULL && remaining > bt_buf_tailroom(buf))
        {
          wlerr("ERROR: Not enough space in buffer\n");
          goto failed;
        }

      wlinfo("Need to get %u bytes\n", remaining);
    }

  if (buf == NULL)
    {
      nread = lower->rxdrain(lower);
      wlwarn("WARNING: Discarded %ld bytes\n", (long)nread);
      remaining -= nread;
    }

  nread = btuart_read(upper, bt_buf_tail(buf), remaining, 0);

  buf->len  += nread;
  remaining -= nread;

  wlinfo("Received %ld bytes\n", (long)nread);

  if (remaining == 0)
    {
      wlinfo("Full packet received\n");

      /* Pass buffer to the stack */

      bt_recv(buf);
      buf = NULL;
    }

  return;

failed:
  bt_buf_put(buf);
  remaining = 0;
  buf       = NULL;
  return;
}

static int btuart_send(FAR const struct bt_driver_s *dev,
                       FAR struct bt_buf_s *buf)
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

  type = bt_buf_push(buf, 1);

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

static int btuart_open(FAR const struct bt_driver_s *dev)
{
  FAR struct btuart_upperhalf_s *upper;
  FAR const struct btuart_lowerhalf_s *lower;

  upper = (FAR struct btuart_upperhalf_s *)dev;
  DEBUGASSERT(upper != NULL && upper->lower != NULL);
  lower = upper->lower;

  /* Disable Tx and Rx interrupts */

  lower->txenable(lower, false);
  lower->rxenable(lower, false);

  /* Drain any cached Rx data */

  (void)lower->rxdrain(lower);

  /* Enable Rx interrupts */

  lower->rxenable(lower, true);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: btuart_register
 *
 * Description:
 *   Create the UART-based Bluetooth device and register it with the
 *   Bluetooth stack.
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
  FAR struct btuart_upperhalf_s *upper;
  int ret;

  DEBUGASSERT(lower != NULL);

  /* Allocate a new instance of the upper half driver state structure */

  upper = (FAR struct btuart_upperhalf_s *)
    kmm_zalloc(sizeof(struct btuart_upperhalf_s));

  if (upper == NULL)
    {
      wlerr("ERROR: Failed to allocate upper-half state\n");
      return -ENOMEM;
    }

  /* Initialize the upper half driver state */

  upper->dev.head_reserve = H4_HEADER_SIZE;
  upper->dev.open         = btuart_open;
  upper->dev.send         = btuart_send;
  upper->lower            = lower;

  /* Attach the interrupt handler */

  lower->attach(lower, btuart_interrupt, upper);

  /* And register the driver with the Bluetooth stack */

  ret = bt_driver_register(&upper->dev);
  if (ret < 0)
    {
      kmm_free(upper);
    }

  return ret;
}
