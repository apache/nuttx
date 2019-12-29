/****************************************************************************
 * drivers/wireless/bluetooth/bt_uart_cc2564.c
 * CC2564 UART based Bluetooth driver
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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

#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/wireless/bluetooth/bt_uart.h>

#include "bt_uart.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The arrays below do not contain firmware. Find the firmware at ti.com.
 * Convert .bts files to C arrays as described there and merge into these
 * arrays.
 */

static const uint8_t ble_firmware[] =
{
#warning Missing CC2564 ble firmware.
  0
};

static const uint8_t cc256x_firmware[] =
{
#warning Missing CC2564 bluetooth firmware
  0
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int cc2564_send(FAR const struct btuart_lowerhalf_s *lower,
                       FAR const uint8_t *buf, ssize_t count)
{
  FAR const uint8_t *ptr = buf;
  ssize_t nwritten;

  while (count > 0)
    {
      nwritten = lower->write(lower, ptr, count);
      count   -= nwritten;
      ptr     += nwritten;
    }

  return 0;
}

static void cc2564_recv(FAR const struct btuart_lowerhalf_s *lower,
                        FAR uint8_t *buf, ssize_t count)
{
  FAR uint8_t *ptr = buf;
  ssize_t nread;

  while (count > 0)
    {
      nread  = lower->read(lower, ptr, count);
      count -= nread;
      ptr   += nread;
    }
}

static int cc2564_load(FAR const struct btuart_lowerhalf_s *lower,
                       FAR const uint8_t *chipdata)
{
  uint8_t buffer[32];
  uint8_t h4_event = 0x04;
  uint8_t h4_cmd   = 0x01;
  uint32_t length  = 0;
  FAR const uint8_t *data;

  for (data = chipdata; *data++; data += length)
    {
      uint16_t opcode;
      opcode = ((uint16_t)(*(data + 1)) << 8) + *data;

      length = data[2] + sizeof(opcode) + sizeof(data[2]);

      cc2564_send(lower, &h4_cmd, 1);
      cc2564_send(lower, data, length);

      cc2564_recv(lower, buffer, 1);
      if (h4_event == buffer[0])
        {
          cc2564_recv(lower, &buffer[1], 2);
          cc2564_recv(lower, &buffer[3], buffer[2]);
        }
      else
        {
          wlerr("ERROR: Unknown data\n");
          return -EIO;
        }
    }

  return 0;
}

int load_cc2564_firmware(FAR const struct btuart_lowerhalf_s *lower)
{
  int ret;

  /* Check for missing firmware */

  if (sizeof(cc256x_firmware) < 10 || sizeof(ble_firmware) < 10)
    {
      return -EINVAL;
    }

  ret = cc2564_load(lower, cc256x_firmware);
  if (ret == 0)
    {
      ret = cc2564_load(lower, ble_firmware);
    }

  return ret;
}

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

  wlinfo("lower %p\n", lower);

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

  /* Load firmware */

  ret = load_cc2564_firmware(lower);
  if (ret < 0)
    {
      wlerr("ERROR: Firmware error\n");
      kmm_free(upper);
      return -EINVAL;
    }

  /* And register the driver with the network and the Bluetooth stack. */

  ret = bt_netdev_register(&upper->dev);
  if (ret < 0)
    {
      wlerr("ERROR: bt_netdev_register failed: %d\n", ret);
      kmm_free(upper);
    }

  return ret;
}
