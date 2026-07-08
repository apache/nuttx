/****************************************************************************
 * arch/arm/src/common/stm32/hardware/stm32_spi.h
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

#ifndef __ARCH_ARM_SRC_COMMON_STM32_HARDWARE_STM32_SPI_H
#define __ARCH_ARM_SRC_COMMON_STM32_HARDWARE_STM32_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/* The SPI IP version (V1..V4) is independent of the CPU core.  The register
 * layout is the same, but the M0 and M3/M4 ports currently keep separate
 * register headers, so the file is chosen by the standard NuttX core symbol
 * (CONFIG_ARCH_CORTEXM0) while the version is core-agnostic.
 */

#if (defined(CONFIG_STM32_HAVE_IP_SPI_V1) + \
     defined(CONFIG_STM32_HAVE_IP_SPI_V2) + \
     defined(CONFIG_STM32_HAVE_IP_SPI_V3) + \
     defined(CONFIG_STM32_HAVE_IP_SPI_V4)) > 1
#  error Only one STM32 SPI IP version must be selected
#endif

#if !(defined(CONFIG_STM32_HAVE_IP_SPI_V1) || \
      defined(CONFIG_STM32_HAVE_IP_SPI_V2) || \
      defined(CONFIG_STM32_HAVE_IP_SPI_V3) || \
      defined(CONFIG_STM32_HAVE_IP_SPI_V4))
#  error "Unsupported STM32 SPI"
#elif defined(CONFIG_ARCH_CORTEXM0)
#  include "hardware/stm32_spi_v1v2_m0.h"
#else
#  include "hardware/stm32_spi_v2v3v4.h"
#endif

#endif /* __ARCH_ARM_SRC_COMMON_STM32_HARDWARE_STM32_SPI_H */
