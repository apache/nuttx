/****************************************************************************
 * arch/arm/src/max326xx/max32660/max32660_periphclks.h
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

#ifndef __ARCH_ARM_SRC_MAX326XX_MAX32660_MAX32660_PERIPHCLKS_H
#define __ARCH_ARM_SRC_MAX326XX_MAX32660_MAX32660_PERIPHCLKS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "arm_internal.h"
#include "hardware/max326_gcr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX32660_PERIPH_ENABLE_0(n)  modifyreg32(MAX326_GCR_PCLKDIS0, (n), 0)
#define MAX32660_PERIPH_ENABLE_1(n)  modifyreg32(MAX326_GCR_PCLKDIS1, (n), 0)
#define MAX32660_PERIPH_DISABLE_0(n) modifyreg32(MAX326_GCR_PCLKDIS0, 0, (n))
#define MAX32660_PERIPH_DISABLE_1(n) modifyreg32(MAX326_GCR_PCLKDIS1, 0, (n))

/* Enable peripheral clocks */

#define max326_gpio0_enableclk()     MAX32660_PERIPH_ENABLE_0(GCR_PCLKDIS0_GPIO0D)
#define max326_dma_enableclk()       MAX32660_PERIPH_ENABLE_0(GCR_PCLKDIS0_DMAD)
#define max326_spi0_enableclk()      MAX32660_PERIPH_ENABLE_0(GCR_PCLKDIS0_SPI0D)
#define max326_spi1_enableclk()      MAX32660_PERIPH_ENABLE_0(GCR_PCLKDIS0_SPI1D)
#define max326_uart0_enableclk()     MAX32660_PERIPH_ENABLE_0(GCR_PCLKDIS0_UART0D)
#define max326_uart1_enableclk()     MAX32660_PERIPH_ENABLE_0(GCR_PCLKDIS0_UART1D)
#define max326_i2c0_enableclk()      MAX32660_PERIPH_ENABLE_0(GCR_PCLKDIS0_I2C0D)
#define max326_tmr0_enableclk()      MAX32660_PERIPH_ENABLE_0(GCR_PCLKDIS0_TMR0D)
#define max326_tmr1_enableclk()      MAX32660_PERIPH_ENABLE_0(GCR_PCLKDIS0_TMR1D)
#define max326_tmr2_enableclk()      MAX32660_PERIPH_ENABLE_0(GCR_PCLKDIS0_TMR2D)
#define max326_i2c1_enableclk()      MAX32660_PERIPH_ENABLE_0(GCR_PCLKDIS0_I2C1D)
#define max326_flc_enableclk()       MAX32660_PERIPH_ENABLE_1(GCR_PCLKDIS1_FLCD)
#define max326_icc_enableclk()       MAX32660_PERIPH_ENABLE_1(GCR_PCLKDIS1_ICCD)

/* Disable peripheral clocks */

#define max326_gpio0_disableclk()    MAX32660_PERIPH_DISABLE_0(GCR_PCLKDIS0_GPIO0D)
#define max326_dma_disableclk()      MAX32660_PERIPH_DISABLE_0(GCR_PCLKDIS0_DMAD)
#define max326_spi0_disableclk()     MAX32660_PERIPH_DISABLE_0(GCR_PCLKDIS0_SPI0D)
#define max326_spi1_disableclk()     MAX32660_PERIPH_DISABLE_0(GCR_PCLKDIS0_SPI1D)
#define max326_uart0_disableclk()    MAX32660_PERIPH_DISABLE_0(GCR_PCLKDIS0_UART0D)
#define max326_uart1_disableclk()    MAX32660_PERIPH_DISABLE_0(GCR_PCLKDIS0_UART1D)
#define max326_i2c0_disableclk()     MAX32660_PERIPH_DISABLE_0(GCR_PCLKDIS0_I2C0D)
#define max326_tmr0_disableclk()     MAX32660_PERIPH_DISABLE_0(GCR_PCLKDIS0_TMR0D)
#define max326_tmr1_disableclk()     MAX32660_PERIPH_DISABLE_0(GCR_PCLKDIS0_TMR1D)
#define max326_tmr2_disableclk()     MAX32660_PERIPH_DISABLE_0(GCR_PCLKDIS0_TMR2D)
#define max326_i2c1_disableclk()     MAX32660_PERIPH_DISABLE_0(GCR_PCLKDIS0_I2C1D)
#define max326_flc_disableclk()      MAX32660_PERIPH_DISABLE_1(GCR_PCLKDIS1_FLCD)
#define max326_icc_disableclk()      MAX32660_PERIPH_DISABLE_1(GCR_PCLKDIS1_ICCD)

#endif /* __ARCH_ARM_SRC_MAX326XX_MAX32660_MAX32660_PERIPHCLKS_H */
