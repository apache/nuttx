/****************************************************************************
 * include/nuttx/net/oa_tc6.h
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

#ifndef __INCLUDE_NUTTX_NET_OA_TC6_H
#define __INCLUDE_NUTTX_NET_OA_TC6_H

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

/* A reference to a structure of this type must be passed to the OA-TC6
 * driver when the driver is instantiated. This structure provides
 * information about the configuration of the MAC-PHY.
 *
 * Memory for this structure is provided by the caller. It is not copied by
 * the driver and is presumed to persist while the driver is active.
 */

struct oa_tc6_config_s
{
  uint32_t id;                /* Field used to guide SPI chip select
                               * and interrupt pin selection           */

  uint32_t frequency;         /* SPI frequency                         */
  uint8_t chunk_payload_size; /* 64, 32, 16, or 8                      */
  bool rx_cut_through;        /* Enable / disable RX cut through mode  */

  /* Attach handler to the falling edge of the interrupt pin. */

  CODE int  (*attach)(FAR const struct oa_tc6_config_s *config,
                      xcpt_t handler, FAR void *arg);

  /* Enable / disable the interrupt. */

  CODE int (*enable)(FAR const struct oa_tc6_config_s *config, bool enable);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: oa_tc6_initialize
 *
 * Description:
 *   Read the PHYID of the MAC-PHY device and initialize the matching
 *   driver.
 *
 * Input Parameters:
 *   spi    - pointer to the initialized SPI interface
 *   config - pointer to the initialized MAC-PHY configuration
 *
 * Returned Value:
 *   On success OK is returned, otherwise negated errno is returned.
 *
 ****************************************************************************/

struct spi_dev_s; /* forward declaration, see nuttx/spi/spi.h */

int oa_tc6_initialize(FAR struct spi_dev_s *spi,
                      FAR const struct oa_tc6_config_s *config);

/****************************************************************************
 * Name: ncv7410_initialize
 *
 * Description:
 *   Initialize and register the OA-TC6 and the NCV7410 (NCN26010) drivers.
 *   This function is called by the oa_tc6_initialize upon detecting
 *   the NCV7410 MAC-PHY on the SPI, but it also may be called directly from
 *   the board level code.
 *
 * Input Parameters:
 *   spi    - pointer to the initialized SPI interface
 *   config - pointer to the initialized MAC-PHY configuration
 *
 * Returned Value:
 *   On success OK is returned, otherwise negated errno is returned.
 *
 ****************************************************************************/

int ncv7410_initialize(FAR struct spi_dev_s *spi,
                       FAR const struct oa_tc6_config_s *config);

/****************************************************************************
 * Name: lan865x_initialize
 *
 * Description:
 *   Initialize and register the OA-TC6 and the LAN865x drivers.
 *   This function is called by the oa_tc6_initialize upon detecting
 *   the LAN865x MAC-PHY on the SPI, but it also may be called directly from
 *   the board level code.
 *
 * Input Parameters:
 *   spi    - pointer to the initialized SPI interface
 *   config - pointer to the initialized MAC-PHY configuration
 *
 * Returned Value:
 *   On success OK is returned, otherwise negated errno is returned.
 *
 ****************************************************************************/

int lan865x_initialize(FAR struct spi_dev_s *spi,
                       FAR const struct oa_tc6_config_s *config);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_NET_OA_TC6_H */
