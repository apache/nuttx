/****************************************************************************
 * include/nuttx/virtio/virtio-gpio.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_VIRTIO_VIRTIO_GPIO_H
#define __INCLUDE_NUTTX_VIRTIO_VIRTIO_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* OASIS VirtIO 1.4 GPIO device (device id 41).  Virtqueues: requestq (0) for
 * synchronous line requests, eventq (1) for device->guest interrupt events.
 */

#define VIRTIO_GPIO_F_IRQ                 0

#define VIRTIO_GPIO_MSG_GET_LINE_NAMES    0x01
#define VIRTIO_GPIO_MSG_GET_DIRECTION     0x02
#define VIRTIO_GPIO_MSG_SET_DIRECTION     0x03
#define VIRTIO_GPIO_MSG_GET_VALUE         0x04
#define VIRTIO_GPIO_MSG_SET_VALUE         0x05
#define VIRTIO_GPIO_MSG_SET_IRQ_TYPE      0x06

#define VIRTIO_GPIO_DIRECTION_NONE        0x00
#define VIRTIO_GPIO_DIRECTION_OUT         0x01
#define VIRTIO_GPIO_DIRECTION_IN          0x02

#define VIRTIO_GPIO_IRQ_TYPE_NONE         0x00
#define VIRTIO_GPIO_IRQ_TYPE_EDGE_RISING  0x01
#define VIRTIO_GPIO_IRQ_TYPE_EDGE_FALLING 0x02
#define VIRTIO_GPIO_IRQ_TYPE_EDGE_BOTH    0x03
#define VIRTIO_GPIO_IRQ_TYPE_LEVEL_HIGH   0x04
#define VIRTIO_GPIO_IRQ_TYPE_LEVEL_LOW    0x08

#define VIRTIO_GPIO_STATUS_OK             0x00
#define VIRTIO_GPIO_STATUS_ERR            0x01

#define VIRTIO_GPIO_IRQ_STATUS_INVALID    0x00
#define VIRTIO_GPIO_IRQ_STATUS_VALID      0x01

/****************************************************************************
 * Public Types
 ****************************************************************************/

begin_packed_struct struct virtio_gpio_config_s
{
  uint16_t ngpio;
  uint8_t  padding[2];
  uint32_t gpio_names_size;
} end_packed_struct;

/* Device-side config extension: the spec config plus the index of the
 * local ioexpander this device forwards to.
 */

begin_packed_struct struct vhost_gpio_config_s
{
  struct virtio_gpio_config_s config;
  uint8_t bus;
} end_packed_struct;

begin_packed_struct struct virtio_gpio_request_s
{
  uint16_t type;
  uint16_t gpio;
  uint32_t value;
} end_packed_struct;

begin_packed_struct struct virtio_gpio_response_s
{
  uint8_t status;
  uint8_t value;
} end_packed_struct;

/* Eventq buffers are queued as a per-line pair, completed by the device
 * when the line fires (VALID) or gets masked (INVALID).
 */

begin_packed_struct struct virtio_gpio_irq_request_s
{
  uint16_t gpio;
} end_packed_struct;

begin_packed_struct struct virtio_gpio_irq_response_s
{
  uint8_t status;
} end_packed_struct;

#endif /* __INCLUDE_NUTTX_VIRTIO_VIRTIO_GPIO_H */
