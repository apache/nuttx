/****************************************************************************
 * drivers/wireless/bluetooth/bt_uart.h
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

#include <stdbool.h>
#include <nuttx/wqueue.h>
#include <nuttx/wireless/bluetooth/bt_driver.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define H4_HEADER_SIZE  1

#define H4_CMD           0x01
#define H4_ACL           0x02
#define H4_SCO           0x03
#define H4_EVT           0x04

#ifdef CONFIG_BLUETOOTH_UART_DUMP
#  define BT_DUMP(m,b,l) lib_dumpbuffer(m,b,l)
#else
#  define BT_DUMP(m,b,l)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This type defines the state data generic UART upper half driver */

struct btuart_lowerhalf_s; /* Forward reference */

struct btuart_upperhalf_s
{
  /* This structure must appear first in the structure so that this structure
   * is cast compatible with struct bt_driver_s.
   */

  struct bt_driver_s dev;

  /* The cached lower half interface */

  FAR const struct btuart_lowerhalf_s *lower;

  /* Work queue support */

  struct work_s work;
  volatile bool busy;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Generic implementations of HCI UART methods */

struct bt_buf_s;  /* Forward reference */

int btuart_send(FAR const struct bt_driver_s *dev, FAR struct bt_buf_s *buf);
int btuart_open(FAR const struct bt_driver_s *dev);
