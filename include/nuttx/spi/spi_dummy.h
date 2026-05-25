/****************************************************************************
 * include/nuttx/spi/spi_dummy.h
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

#ifndef __INCLUDE_NUTTX_SPI_SPI_DUMMY_H
#define __INCLUDE_NUTTX_SPI_SPI_DUMMY_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>

#include <nuttx/spi/spi.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct spi_dummy_device_s
{
  CODE void (*select)(FAR struct spi_dummy_device_s *dev, bool selected);
  CODE uint8_t (*status)(FAR struct spi_dummy_device_s *dev);
  CODE uint32_t (*send)(FAR struct spi_dummy_device_s *dev, uint32_t wd,
                        int nbits);
#ifdef CONFIG_SPI_EXCHANGE
  CODE int (*exchange)(FAR struct spi_dummy_device_s *dev,
                       FAR const void *txbuffer, FAR void *rxbuffer,
                       size_t nwords, int nbits);
#endif
  uint32_t devid;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: spi_dummy_initialize
 *
 * Description:
 *   Initialize a dummy SPI bus for transport and command-line validation.
 *   The device pointer table must remain valid for the lifetime of the bus.
 *
 * Returned Value:
 *   A valid SPI lower-half instance on success; NULL on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_DUMMY
FAR struct spi_dev_s *
spi_dummy_initialize(FAR struct spi_dummy_device_s * const *devices,
                     int ndevices);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_SPI_SPI_DUMMY_H */
