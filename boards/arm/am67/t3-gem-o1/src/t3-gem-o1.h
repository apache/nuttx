/****************************************************************************
 * boards/arm/am67/t3-gem-o1/src/t3-gem-o1.h
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

#ifndef __BOARDS_ARM_T3_GEM_O1_SRC_T3_GEM_O1_H
#define __BOARDS_ARM_T3_GEM_O1_SRC_T3_GEM_O1_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__

struct spi_dev_s;

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

int am67_bringup(void);

#ifdef CONFIG_AM67_MCSPI0
void am67_spi0select(FAR struct spi_dev_s *dev, uint32_t devid,
                     bool selected);
uint8_t am67_spi0status(FAR struct spi_dev_s *dev, uint32_t devid);
void am67_spidev_initialize(void);
#endif

#if defined(CONFIG_AM67_I2C0) || defined(CONFIG_AM67_WKUP_I2C0)
void am67_i2cdev_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_T3_GEM_O1_SRC_T3_GEM_O1_H */
