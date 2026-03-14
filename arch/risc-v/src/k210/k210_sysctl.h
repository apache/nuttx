/****************************************************************************
 * arch/risc-v/src/k210/k210_sysctl.h
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

#ifndef __ARCH_RISCV_SRC_K210_K210_SYSCTL_H
#define __ARCH_RISCV_SRC_K210_K210_SYSCTL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/**
 * @brief K210 peripheral clock ID enumeration
 *
 * This enumeration defines all clock IDs for K210 peripherals.
 * These IDs are used to enable/disable clocks and get clock frequencies.
 */

typedef enum
{
  K210_CLOCK_PLL0 = 0,  /* PLL0 clock */
  K210_CLOCK_PLL1,      /* PLL1 clock */
  K210_CLOCK_PLL2,      /* PLL2 clock */
  K210_CLOCK_CPU,       /* CPU clock */
  K210_CLOCK_SRAM0,     /* SRAM0 clock */
  K210_CLOCK_SRAM1,     /* SRAM1 clock */
  K210_CLOCK_APB0,      /* APB0 bus clock */
  K210_CLOCK_APB1,      /* APB1 bus clock */
  K210_CLOCK_APB2,      /* APB2 bus clock */
  K210_CLOCK_ROM,       /* ROM clock */
  K210_CLOCK_DMA,       /* DMA clock */
  K210_CLOCK_AI,        /* AI accelerator clock */
  K210_CLOCK_DVP,       /* DVP camera interface clock */
  K210_CLOCK_FFT,       /* FFT accelerator clock */
  K210_CLOCK_GPIO,      /* GPIO clock */
  K210_CLOCK_SPI0,      /* SPI0 clock */
  K210_CLOCK_SPI1,      /* SPI1 clock */
  K210_CLOCK_SPI2,      /* SPI2 clock */
  K210_CLOCK_SPI3,      /* SPI3 clock */
  K210_CLOCK_I2S0,      /* I2S0 clock */
  K210_CLOCK_I2S1,      /* I2S1 clock */
  K210_CLOCK_I2S2,      /* I2S2 clock */
  K210_CLOCK_I2C0,      /* I2C0 clock */
  K210_CLOCK_I2C1,      /* I2C1 clock */
  K210_CLOCK_I2C2,      /* I2C2 clock */
  K210_CLOCK_UART1,     /* UART1 clock */
  K210_CLOCK_UART2,     /* UART2 clock */
  K210_CLOCK_UART3,     /* UART3 clock */
  K210_CLOCK_AES,       /* AES accelerator clock */
  K210_CLOCK_FPIOA,     /* FPIOA (GPIO mux) clock */
  K210_CLOCK_TIMER0,    /* TIMER0 clock */
  K210_CLOCK_TIMER1,    /* TIMER1 clock */
  K210_CLOCK_TIMER2,    /* TIMER2 clock */
  K210_CLOCK_WDT0,      /* Watchdog timer 0 clock */
  K210_CLOCK_WDT1,      /* Watchdog timer 1 clock */
  K210_CLOCK_SHA,       /* SHA accelerator clock */
  K210_CLOCK_OTP,       /* OTP (one-time programmable) clock */
  K210_CLOCK_RTC,       /* RTC clock */
  K210_CLOCK_MAX        /* Boundary check value */
} k210_clockid_t;

/**
 * @brief K210 peripheral reset ID enumeration
 *
 * This enumeration defines all reset IDs for K210 peripherals.
 * These IDs are used to reset individual peripherals.
 */

typedef enum
{
  K210_RESET_SOC = 0,   /* SOC reset */
  K210_RESET_ROM,       /* ROM reset */
  K210_RESET_DMA,       /* DMA reset */
  K210_RESET_AI,        /* AI accelerator reset */
  K210_RESET_DVP,       /* DVP camera interface reset */
  K210_RESET_FFT,       /* FFT accelerator reset */
  K210_RESET_GPIO,      /* GPIO reset */
  K210_RESET_SPI0,      /* SPI0 reset */
  K210_RESET_SPI1,      /* SPI1 reset */
  K210_RESET_SPI2,      /* SPI2 reset */
  K210_RESET_SPI3,      /* SPI3 reset */
  K210_RESET_I2S0,      /* I2S0 reset */
  K210_RESET_I2S1,      /* I2S1 reset */
  K210_RESET_I2S2,      /* I2S2 reset */
  K210_RESET_I2C0,      /* I2C0 reset */
  K210_RESET_I2C1,      /* I2C1 reset */
  K210_RESET_I2C2,      /* I2C2 reset */
  K210_RESET_UART1,     /* UART1 reset */
  K210_RESET_UART2,     /* UART2 reset */
  K210_RESET_UART3,     /* UART3 reset */
  K210_RESET_AES,       /* AES accelerator reset */
  K210_RESET_FPIOA,     /* FPIOA reset */
  K210_RESET_TIMER0,    /* TIMER0 reset */
  K210_RESET_TIMER1,    /* TIMER1 reset */
  K210_RESET_TIMER2,    /* TIMER2 reset */
  K210_RESET_WDT0,      /* Watchdog timer 0 reset */
  K210_RESET_WDT1,      /* Watchdog timer 1 reset */
  K210_RESET_SHA,       /* SHA accelerator reset */
  K210_RESET_RTC,       /* RTC reset */
  K210_RESET_MAX        /* Boundary check value */
} k210_rstidx_t;

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/**
 * @brief Initialize the system controller
 *
 * This function initializes the K210 system controller, including
 * PLL configuration and clock tree setup.
 */

EXTERN void k210_sysctl_init(void);

/**
 * @brief Enable a peripheral clock
 *
 * @param clkid Clock ID of the peripheral
 * @return 0 on success, negative error code on failure
 */

EXTERN int k210_sysctl_clock_enable(k210_clockid_t clkid);

/**
 * @brief Disable a peripheral clock
 *
 * @param clkid Clock ID of the peripheral
 * @return 0 on success, negative error code on failure
 */

EXTERN int k210_sysctl_clock_disable(k210_clockid_t clkid);

/**
 * @brief Get the frequency of a peripheral clock
 *
 * @param clkid Clock ID of the peripheral
 * @return Clock frequency in Hz, 0 on error
 */

EXTERN uint32_t k210_sysctl_clock_get_freq(k210_clockid_t clkid);

/**
 * @brief Get current CPU clock frequency
 *
 * @return CPU clock frequency in Hz
 */

EXTERN uint32_t k210_sysctl_cpu_get_freq(void);

/**
 * @brief Set CPU clock frequency (SDK-like)
 *
 * This function adjusts PLL0 and keeps current ACLK divider setting.
 * The final frequency may be near the target due to integer PLL factors.
 *
 * @param freq Target CPU frequency in Hz
 * @return Actual CPU clock frequency in Hz, 0 on failure
 */

EXTERN uint32_t k210_sysctl_cpu_set_freq(uint32_t freq);

/**
 * @brief Reset a peripheral
 *
 * @param rstidx Reset ID of the peripheral
 * @return 0 on success, negative error code on failure
 */

EXTERN int k210_sysctl_reset(k210_rstidx_t rstidx);

/**
 * @brief Initialize a peripheral (enable clock and deassert reset)
 *
 * This is a convenience function that enables the clock and releases
 * the peripheral from reset state.
 *
 * @param clkid Clock ID of the peripheral
 * @param rstidx Reset ID of the peripheral
 * @return 0 on success, negative error code on failure
 */

EXTERN int k210_sysctl_init_peripheral(k210_clockid_t clkid,
                                        k210_rstidx_t rstidx);

/**
 * @brief Check if a PLL is locked
 *
 * @param pll_offset PLL register offset (K210_SYSCTL_PLL0/PLL1/PLL2)
 * @return true if PLL is locked, false otherwise
 */

EXTERN bool k210_sysctl_pll_is_locked(uint32_t pll_offset);

#if defined(__cplusplus)
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_K210_K210_SYSCTL_H */
