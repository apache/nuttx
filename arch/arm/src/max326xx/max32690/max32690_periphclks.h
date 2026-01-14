/****************************************************************************
 * arch/arm/src/max326xx/max32690/max32690_periphclks.h
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

#ifndef __ARCH_ARM_SRC_MAX326XX_MAX32690_MAX32690_PERIPHCLKS_H
#define __ARCH_ARM_SRC_MAX326XX_MAX32690_MAX32690_PERIPHCLKS_H

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

#define MAX32690_PERIPH_ENABLE_0(n)  modifyreg32(MAX326_GCR_PCLKDIS0, (n), 0)
#define MAX32690_PERIPH_ENABLE_1(n)  modifyreg32(MAX326_GCR_PCLKDIS1, (n), 0)
#define MAX32690_PERIPH_DISABLE_0(n) modifyreg32(MAX326_GCR_PCLKDIS0, 0, (n))
#define MAX32690_PERIPH_DISABLE_1(n) modifyreg32(MAX326_GCR_PCLKDIS1, 0, (n))

/* Enable peripheral clocks ( register 0 ) */

#define max326_pt_enableclk()        MAX32690_PERIPH_ENABLE_0(GCR_PCLKDIS0_PTD)
#define max326_adc_enableclk()       MAX32690_PERIPH_ENABLE_0(GCR_PCLKDIS0_ADCD)
#define max326_crypto_enableclk()    MAX32690_PERIPH_ENABLE_0(GCR_PCLKDIS0_CRYPTOD)
#define max326_dma_enableclk()       MAX32690_PERIPH_ENABLE_0(GCR_PCLKDIS0_DMAD)
#define max326_usb_enableclk()       MAX32690_PERIPH_ENABLE_0(GCR_PCLKDIS0_USBD)

/* Enable peripheral clocks ( register 1 ) */

#define max326_cpu1_enableclk()      MAX32690_PERIPH_ENABLE_1(GCR_PCLKDIS1_CPU1)
#define max326_wdt0_enableclk()      MAX32690_PERIPH_ENABLE_1(GCR_PCLKDIS1_WDT0)
#define max326_i2s_enableclk()       MAX32690_PERIPH_ENABLE_1(GCR_PCLKDIS1_I2S)
#define max326_owm_enableclk()       MAX32690_PERIPH_ENABLE_1(GCR_PCLKDIS1_OW)
#define max326_sem_enableclk()       MAX32690_PERIPH_ENABLE_1(GCR_PCLKDIS1_SEM)
#define max326_syscache_enableclk()  MAX32690_PERIPH_ENABLE_1(GCR_PCLKDIS1_SYSCACHE)
#define max326_hpb_enableclk()       MAX32690_PERIPH_ENABLE_1(GCR_PCLKDIS1_HPB)
#define max326_puf_enableclk()       MAX32690_PERIPH_ENABLE_1(GCR_PCLKDIS1_PUF)
#define max326_trng_enableclk()      MAX32690_PERIPH_ENABLE_1(GCR_PCLKDIS1_TRNG)
#define max326_btle_enableclk()      MAX32690_PERIPH_ENABLE_1(GCR_PCLKDIS1_BTLE)

/* Enable peripheral clocks ( spi ) */

#define max326_spixr_enableclk()     MAX32690_PERIPH_ENABLE_1(GCR_PCLKDIS1_SPIXiRAM)
#define max326_spixipc_enableclk()   MAX32690_PERIPH_ENABLE_0(GCR_PCLKDIS0_SPIXiPMCD)
#define max326_spixip_enableclk()    MAX32690_PERIPH_ENABLE_0(GCR_PCLKDIS0_SPIXiPD)

#define max326_spi0_enableclk()      MAX32690_PERIPH_ENABLE_0(GCR_PCLKDIS0_SPI0)
#define max326_spi1_enableclk()      MAX32690_PERIPH_ENABLE_0(GCR_PCLKDIS0_SPI1)
#define max326_spi2_enableclk()      MAX32690_PERIPH_ENABLE_0(GCR_PCLKDIS0_SPI2)
#define max326_spi3_enableclk()      MAX32690_PERIPH_ENABLE_1(GCR_PCLKDIS1_SPI3)
#define max326_spi4_enableclk()      MAX32690_PERIPH_ENABLE_1(GCR_PCLKDIS1_SPI4)

/* Enable peripheral clocks ( can ) */

#define max326_can0_enableclk()      MAX32690_PERIPH_ENABLE_1(GCR_PCLKDIS1_CAN0)
#define max326_can1_enableclk()      MAX32690_PERIPH_ENABLE_1(GCR_PCLKDIS1_CAN1)

/* Enable peripheral clocks ( gpio ) */

#define max326_gpio0_enableclk()     MAX32690_PERIPH_ENABLE_0(GCR_PCLKDIS0_GPIO0D)
#define max326_gpio1_enableclk()     MAX32690_PERIPH_ENABLE_0(GCR_PCLKDIS0_GPIO1D)
#define max326_gpio2_enableclk()     MAX32690_PERIPH_ENABLE_0(GCR_PCLKDIS0_GPIO2D)

/* Enable peripheral clocks ( uart ) */

#define max326_uart0_enableclk()     MAX32690_PERIPH_ENABLE_0(GCR_PCLKDIS0_UART0D)
#define max326_uart1_enableclk()     MAX32690_PERIPH_ENABLE_0(GCR_PCLKDIS0_UART1D)
#define max326_uart2_enableclk()     MAX32690_PERIPH_ENABLE_1(GCR_PCLKDIS1_UART2D)

/* Enable peripheral clocks ( i2c ) */

#define max326_i2c0_enableclk()      MAX32690_PERIPH_ENABLE_0(GCR_PCLKDIS0_I2C0D)
#define max326_i2c1_enableclk()      MAX32690_PERIPH_ENABLE_0(GCR_PCLKDIS0_I2C1D)
#define max326_i2c2_enableclk()      MAX32690_PERIPH_ENABLE_1(GCR_PCLKDIS1_I2C2)

/* Enable peripheral clocks ( timer ) */

#define max326_tmr0_enableclk()      MAX32690_PERIPH_ENABLE_0(GCR_PCLKDIS0_TMR0D)
#define max326_tmr1_enableclk()      MAX32690_PERIPH_ENABLE_0(GCR_PCLKDIS0_TMR1D)
#define max326_tmr2_enableclk()      MAX32690_PERIPH_ENABLE_0(GCR_PCLKDIS0_TMR2D)
#define max326_tmr3_enableclk()      MAX32690_PERIPH_ENABLE_0(GCR_PCLKDIS0_TMR3D)

/* Disable peripheral clocks ( register 0 ) */

#define max326_pt_disableclk()        MAX32690_PERIPH_DISABLE_0(GCR_PCLKDIS0_PTD)
#define max326_adc_disableclk()       MAX32690_PERIPH_DISABLE_0(GCR_PCLKDIS0_ADCD)
#define max326_crypto_disableclk()    MAX32690_PERIPH_DISABLE_0(GCR_PCLKDIS0_CRYPTOD)
#define max326_dma_disableclk()       MAX32690_PERIPH_DISABLE_0(GCR_PCLKDIS0_DMAD)
#define max326_usb_disableclk()       MAX32690_PERIPH_DISABLE_0(GCR_PCLKDIS0_USBD)

/* Disable peripheral clocks ( register 1 ) */

#define max326_cpu1_disableclk()      MAX32690_PERIPH_DISABLE_1(GCR_PCLKDIS1_CPU1)
#define max326_wdt0_disableclk()      MAX32690_PERIPH_DISABLE_1(GCR_PCLKDIS1_WDT0)
#define max326_i2s_disableclk()       MAX32690_PERIPH_DISABLE_1(GCR_PCLKDIS1_I2S)
#define max326_owm_disableclk()       MAX32690_PERIPH_DISABLE_1(GCR_PCLKDIS1_OW)
#define max326_sem_disableclk()       MAX32690_PERIPH_DISABLE_1(GCR_PCLKDIS1_SEM)
#define max326_syscache_disableclk()  MAX32690_PERIPH_DISABLE_1(GCR_PCLKDIS1_SYSCACHE)
#define max326_hpb_disableclk()       MAX32690_PERIPH_DISABLE_1(GCR_PCLKDIS1_HPB)
#define max326_puf_disableclk()       MAX32690_PERIPH_DISABLE_1(GCR_PCLKDIS1_PUF)
#define max326_trng_disableclk()      MAX32690_PERIPH_DISABLE_1(GCR_PCLKDIS1_TRNG)
#define max326_btle_disableclk()      MAX32690_PERIPH_DISABLE_1(GCR_PCLKDIS1_BTLE)

/* Disable peripheral clocks ( spi ) */

#define max326_spixr_disableclk()     MAX32690_PERIPH_DISABLE_1(GCR_PCLKDIS1_SPIXiRAM)
#define max326_spixipc_disableclk()   MAX32690_PERIPH_DISABLE_0(GCR_PCLKDIS0_SPIXiPMCD)
#define max326_spixip_disableclk()    MAX32690_PERIPH_DISABLE_0(GCR_PCLKDIS0_SPIXiPD)

#define max326_spi0_disableclk()      MAX32690_PERIPH_DISABLE_0(GCR_PCLKDIS0_SPI0)
#define max326_spi1_disableclk()      MAX32690_PERIPH_DISABLE_0(GCR_PCLKDIS0_SPI1)
#define max326_spi2_disableclk()      MAX32690_PERIPH_DISABLE_0(GCR_PCLKDIS0_SPI2)
#define max326_spi3_disableclk()      MAX32690_PERIPH_DISABLE_1(GCR_PCLKDIS1_SPI3)
#define max326_spi4_disableclk()      MAX32690_PERIPH_DISABLE_1(GCR_PCLKDIS1_SPI4)

/* Disable peripheral clocks ( can ) */

#define max326_can0_disableclk()      MAX32690_PERIPH_DISABLE_1(GCR_PCLKDIS1_CAN0)
#define max326_can1_disableclk()      MAX32690_PERIPH_DISABLE_1(GCR_PCLKDIS1_CAN1)

/* Disable peripheral clocks ( gpio ) */

#define max326_gpio0_disableclk()     MAX32690_PERIPH_DISABLE_0(GCR_PCLKDIS0_GPIO0D)
#define max326_gpio1_disableclk()     MAX32690_PERIPH_DISABLE_0(GCR_PCLKDIS0_GPIO1D)
#define max326_gpio2_disableclk()     MAX32690_PERIPH_DISABLE_0(GCR_PCLKDIS0_GPIO2D)

/* Disable peripheral clocks ( uart ) */

#define max326_uart0_disableclk()     MAX32690_PERIPH_DISABLE_0(GCR_PCLKDIS0_UART0D)
#define max326_uart1_disableclk()     MAX32690_PERIPH_DISABLE_0(GCR_PCLKDIS0_UART1D)
#define max326_uart2_disableclk()     MAX32690_PERIPH_DISABLE_1(GCR_PCLKDIS1_UART2D)

/* Disable peripheral clocks ( i2c ) */

#define max326_i2c0_disableclk()      MAX32690_PERIPH_DISABLE_0(GCR_PCLKDIS0_I2C0D)
#define max326_i2c1_disableclk()      MAX32690_PERIPH_DISABLE_0(GCR_PCLKDIS0_I2C1D)
#define max326_i2c2_disableclk()      MAX32690_PERIPH_DISABLE_1(GCR_PCLKDIS1_I2C2)

/* Disable peripheral clocks ( timer ) */

#define max326_tmr0_disableclk()      MAX32690_PERIPH_DISABLE_0(GCR_PCLKDIS0_TMR0D)
#define max326_tmr1_disableclk()      MAX32690_PERIPH_DISABLE_0(GCR_PCLKDIS0_TMR1D)
#define max326_tmr2_disableclk()      MAX32690_PERIPH_DISABLE_0(GCR_PCLKDIS0_TMR2D)
#define max326_tmr3_disableclk()      MAX32690_PERIPH_DISABLE_0(GCR_PCLKDIS0_TMR3D)

#endif /* __ARCH_ARM_SRC_MAX326XX_MAX32690_MAX32690_PERIPHCLKS_H */
