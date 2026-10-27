/****************************************************************************
 * include/nuttx/virtio/virtio-serial.h
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

#ifndef __INCLUDE_NUTTX_VIRTIO_VIRTIO_SERIAL_H
#define __INCLUDE_NUTTX_VIRTIO_VIRTIO_SERIAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <limits.h>
#include <stdint.h>

#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VIRTIO_SERIAL_RX           0
#define VIRTIO_SERIAL_TX           1
#define VIRTIO_SERIAL_NUM          2

#define VIRTIO_SERIAL_PATH_SIZE    64

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

begin_packed_struct struct virtio_serial_config_s
{
  uint16_t cols;
  uint16_t rows;
  uint32_t max_nr_ports;
  uint32_t emerg_wr;
} end_packed_struct;

begin_packed_struct struct vhost_serial_config_s
{
  struct virtio_serial_config_s config;
  char path[VIRTIO_SERIAL_PATH_SIZE];
} end_packed_struct;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_VIRTIO_VIRTIO_SERIAL_H */
