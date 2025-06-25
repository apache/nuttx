/****************************************************************************
 * include/nuttx/net/ncv7410.h
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

#ifndef __INCLUDE_NUTTX_NET_NCV7410_H
#define __INCLUDE_NUTTX_NET_NCV7410_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the NCV7410
 * driver when the driver is instantiated. This structure provides
 * information about the configuration of the NCV7410.
 *
 * Memory for this structure is provided by the caller. It is not copied by
 * the driver and is presumed to persist while the driver is active.
 */

struct ncv7410_config_s
{
  uint32_t id;

  /* TODO: include hooks for interrupt logic */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ncv7410_initialize
 *
 * Description:
 *   Initialize the Ethernet driver.
 *
 * Input Parameters:
 *   spi - reference to the SPI driver state data
 *   irq - irq number of the pin connected to MAC-PHY's interrupt signal
 *
 * Returned Value:
 *   On success OK is returned, otherwise negated errno is returned.
 *
 ****************************************************************************/

struct spi_dev_s; /* forward declaration, see nuttx/spi/spi.h */
int ncv7410_initialize(FAR struct spi_dev_s *spi, int irq,
                       struct ncv7410_config_s *config);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_NET_NCV7410_H */
