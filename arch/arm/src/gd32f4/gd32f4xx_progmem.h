/****************************************************************************
 * arch/arm/src/gd32f4/gd32f4xx_progmem.h
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

#ifndef __ARCH_ARM_SRC_GD32F4_GD32F4XX_PROGMEM_H
#define __ARCH_ARM_SRC_GD32F4_GD32F4XX_PROGMEM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/progmem.h>

/****************************************************************************
 * Refer to GD32F4xx_User_Manual Chapter 2: Flash memory controller to know
 * about how program with the flash memory on GD32F450Z.
 *
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FMC_STAT_PERR  (FMC_STAT_END | FMC_STAT_OPERR | FMC_STAT_WPERR | FMC_STAT_PGMERR | FMC_STAT_PGSERR)

#endif /* __ARCH_ARM_SRC_GD32F4_GD32F4XX_PROGMEM_H */
