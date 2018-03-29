/****************************************************************************
 * wireless/bluetooth/bt_driver.h
 * Bluetooth HCI driver API.
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
 *    and/or other materials provided with the distribution.
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

#ifndef __INCLUDE_NUTTX_WIRELESS_BT_DRIVER_H
#define __INCLUDE_NUTTX_WIRELESS_BT_DRIVER_H 1

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/wireless/bt_buf.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct bt_driver_s
{
  /* How much headroom is needed for HCI transport headers */

  size_t head_reserve;

  /* Open the HCI transport */

  CODE int (*open)(FAR const struct bt_driver_s *dev);

  /* Send data to HCI */

  CODE int (*send)(FAR const struct bt_driver_s *dev,
                   FAR struct bt_buf_s *buf);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Register a new HCI driver to the Bluetooth stack */

int bt_driver_register(FAR const struct bt_driver_s *dev);

/* Unregister a previously registered HCI driver */

void bt_driver_unregister(FAR const struct bt_driver_s *dev);

/* Receive data from the controller/HCI driver */

void bt_recv(FAR struct bt_buf_s *buf);

#endif /* __INCLUDE_NUTTX_WIRELESS_BT_DRIVER_H */
