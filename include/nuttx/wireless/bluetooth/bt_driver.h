/****************************************************************************
 * include/nuttx/wireless/bluetooth/bt_driver.h
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

#ifndef __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BT_DRIVER_H
#define __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BT_DRIVER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/wireless/bluetooth/bt_buf.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define bt_netdev_receive(btdev, type, data, len) \
        (btdev)->receive(btdev, type, data, len)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct bt_driver_s
{
  /* How much headroom is needed for HCI transport headers */

  size_t head_reserve;

  /* Open the HCI transport */

  CODE int (*open)(FAR struct bt_driver_s *btdev);

  /* Send data to HCI */

  CODE int (*send)(FAR struct bt_driver_s *btdev,
                   enum bt_buf_type_e type,
                   FAR void *data, size_t len);

  /* Close the HCI transport */

  CODE void (*close)(FAR struct bt_driver_s *btdev);

  /* Filled by register function but called by bt_driver_s */

  CODE int (*receive)(FAR struct bt_driver_s *btdev,
                      enum bt_buf_type_e type,
                      FAR void *data, size_t len);

  /* Lower-half logic may support platform-specific ioctl commands */

  CODE int (*ioctl)(FAR struct bt_driver_s *btdev, int cmd,
                    unsigned long arg);

  /* For private use by device drivers.
   * Should NOT be touched by the bluetooth stack.
   */

  FAR void *priv;

  /* Reserved for the bluetooth stack.
   * Should NOT be touched by drivers.
   */

  FAR void *bt_net;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: bt_netdev_register
 *
 * Description:
 *   Register a network driver to access the Bluetooth layer using a 6LoWPAN
 *   IPv6 or AF_BLUETOOTH socket.
 *
 * Input Parameters:
 *   btdev - An instance of the low-level drivers interface structure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int bt_netdev_register(FAR struct bt_driver_s *btdev);

#endif /* __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BT_DRIVER_H */
