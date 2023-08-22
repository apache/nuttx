/****************************************************************************
 * include/nuttx/ioexpander/iso1h812g.h
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

#ifndef __INCLUDE_NUTTX_IOEXPANDER_ISO1H812G_H
#define __INCLUDE_NUTTX_IOEXPANDER_ISO1H812G_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the ISO1H812Gxx
 * driver when the driver is instantiated. This structure provides
 * information about the configuration of the ISO1H812Gxx and provides some
 * board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied by
 * the driver and is presumed to persist while the driver is active. The
 * memory must be writeable because, under certain circumstances, the driver
 * may modify the frequency.
 */

struct iso1h812g_config_s
{
  /* Device characterization */

  uint8_t id;          /* ID if multiple devices used to select correct CS */
  uint8_t mode;        /* SPI mode */
  uint32_t frequency;  /* SPI frequency */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: iso1h812g_initialize
 *
 * Description:
 *   Instantiate and configure the ISO1H812Gxx device driver to use the
 *   provided SPI device instance.
 *
 * Input Parameters:
 *   spi     - A SPI driver instance
 *   config  - Persistent board configuration data
 *
 * Returned Value:
 *   an ioexpander_dev_s instance on success, NULL on failure.
 *
 ****************************************************************************/

struct spi_dev_s;
FAR struct ioexpander_dev_s *iso1h812g_initialize(FAR struct spi_dev_s *spi,
                                      FAR struct iso1h812g_config_s *config);

#endif /* __INCLUDE_NUTTX_IOEXPANDER_ISO1H812G_H */
