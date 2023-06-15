/****************************************************************************
 * drivers/virtio/virtio-input.h
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

#ifndef __DRIVERS_VIRTIO_VIRTIO_INPUT_H
#define __DRIVERS_VIRTIO_VIRTIO_INPUT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#ifdef CONFIG_DRIVERS_VIRTIO_INPUT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VIRTIO_INPUT_CFG_UNSET       0x00
#define VIRTIO_INPUT_CFG_ID_NAME     0x01
#define VIRTIO_INPUT_CFG_ID_SERIAL   0x02
#define VIRTIO_INPUT_CFG_ID_DEVIDS   0x03
#define VIRTIO_INPUT_CFG_PROP_BITS   0x10
#define VIRTIO_INPUT_CFG_EV_BITS     0x11
#define VIRTIO_INPUT_CFG_ABS_INFO    0x12

#define VIRTIO_INPUT_CHUNK_SIZE      128  /* 128 bytes per chunk */

/****************************************************************************
 * Public Types
 ****************************************************************************/

begin_packed_struct struct virtio_input_absinfo
{
  uint32_t min;
  uint32_t max;
  uint32_t fuzz;
  uint32_t flat;
  uint32_t res;
} end_packed_struct;

begin_packed_struct struct virtio_input_devids
{
  uint16_t bustype;
  uint16_t vendor;
  uint16_t product;
  uint16_t version;
} end_packed_struct;

begin_packed_struct struct virtio_input_config
{
  uint8_t select;
  uint8_t subsel;
  uint8_t size;
  uint8_t reserved[5];
  union
  {
    char                        string[VIRTIO_INPUT_CHUNK_SIZE];
    uint8_t                     bitmap[VIRTIO_INPUT_CHUNK_SIZE];
    struct virtio_input_absinfo abs;
    struct virtio_input_devids  ids;
  } u;
} end_packed_struct;

begin_packed_struct struct virtio_input_event
{
  uint16_t type;
  uint16_t code;
  uint32_t value;
} end_packed_struct;

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int virtio_register_input_driver(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_DRIVERS_VIRTIO_INPUT */

#endif /* __DRIVERS_VIRTIO_VIRTIO_INPUT_H */
