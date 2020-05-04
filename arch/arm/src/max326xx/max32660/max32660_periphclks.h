/************************************************************************************
 * arch/arm/src/max326xx/max32600/max326_periphclks.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_MAX326XX_MAX32660_MAX32660_PERIPHCLKS_H
#define __ARCH_ARM_SRC_MAX326XX_MAX32660_MAX32660_PERIPHCLKS_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "arm_arch.h"
#include "hardware/max326_gcr.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

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
