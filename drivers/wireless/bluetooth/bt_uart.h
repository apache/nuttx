/****************************************************************************
 * drivers/wireless/bluetooth/bt_uart.h
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

#ifndef __DRIVER_WIRELESS_BLUETOOTH_BT_UART_H
#define __DRIVER_WIRELESS_BLUETOOTH_BT_UART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <nuttx/wqueue.h>
#include <nuttx/wireless/bluetooth/bt_driver.h>
#include <nuttx/wireless/bluetooth/bt_uart.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_BLUETOOTH_UART_DUMP
#  define BT_DUMP(m,b,l) lib_dumpbuffer(m,b,l)
#else
#  define BT_DUMP(m,b,l)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This type defines the state data generic UART upper half driver */

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

int btuart_send(FAR struct bt_driver_s *dev,
                enum bt_buf_type_e type,
                FAR void *data, size_t len);
int btuart_open(FAR struct bt_driver_s *dev);
void btuart_close(FAR struct bt_driver_s *dev);
int btuart_ioctl(FAR struct bt_driver_s *dev,
                 int cmd, unsigned long arg);

#endif /* __DRIVER_WIRELESS_BLUETOOTH_BT_UART_H */
