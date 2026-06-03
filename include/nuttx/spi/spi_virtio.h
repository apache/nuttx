/****************************************************************************
 * include/nuttx/spi/spi_virtio.h
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

#ifndef __INCLUDE_NUTTX_SPI_SPI_VIRTIO_H
#define __INCLUDE_NUTTX_SPI_SPI_VIRTIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/spi/spi.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: virtio_spi_get_master
 *
 * Description:
 *   Return the SPI lower-half probed by the VirtIO SPI frontend for bus
 *   index, or NULL if no such bus has been probed.  Board/in-kernel code
 *   uses this to obtain the instance and may then pass it to device drivers
 *   or register it with spi_register() as /dev/spiN.
 *
 * Input Parameters:
 *   index - The VirtIO SPI bus index assigned at probe time.
 *
 * Returned Value:
 *   The matching SPI lower-half, or NULL if not found.
 *
 ****************************************************************************/

FAR struct spi_dev_s *virtio_spi_get_master(int index);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_SPI_SPI_VIRTIO_H */
