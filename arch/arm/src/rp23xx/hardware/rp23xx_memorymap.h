/****************************************************************************
 * arch/arm/src/rp23xx/hardware/rp23xx_memorymap.h
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
#ifndef __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_MEMORYMAP_H
#define __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp23xx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "arm_internal.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RP23XX_FLASH_BASE                0x10000000 /* -0x001fffff: FLASH memory space (2048KB) */
#define RP23XX_SRAM_BASE                 0x20000000 /* -0x20041fff: SRAM memory space (520KB) */

#define RP23XX_SYSINFO_BASE              0x40000000
#define RP23XX_SYSCFG_BASE               0x40008000  /* Register block for various chip control signals */
#define RP23XX_CLOCKS_BASE               0x40010000
#define RP23XX_RESETS_BASE               0x40020000
#define RP23XX_PSM_BASE                  0x40018000
#define RP23XX_IO_BANK0_BASE             0x40028000
#define RP23XX_IO_QSPI_BASE              0x40030000
#define RP23XX_PADS_BANK0_BASE           0x40038000
#define RP23XX_PADS_QSPI_BASE            0x40040000
#define RP23XX_XOSC_BASE                 0x40048000  /* Controls the crystal oscillator */
#define RP23XX_PLL_SYS_BASE              0x40050000
#define RP23XX_PLL_USB_BASE              0x40058000
#define RP23XX_BUSCTRL_BASE              0x40068000  /* Register block for busfabric control signals and performance counters */
#define RP23XX_UART0_BASE                0x40070000
#define RP23XX_UART1_BASE                0x40078000
#define RP23XX_UART_BASE(n)              (0x40070000 + (n) * 0x8000)
#define RP23XX_SPI0_BASE                 0x40080000
#define RP23XX_SPI1_BASE                 0x40088000
#define RP23XX_SPI_BASE(n)               (0x40080000 + (n) * 0x8000)
#define RP23XX_I2C0_BASE                 0x40090000  /* DW_apb_i2c address block */
#define RP23XX_I2C1_BASE                 0x40098000  /* DW_apb_i2c address block */
#define RP23XX_I2C_BASE(n)               (0x40090000 + (n) * 0x8000)
#define RP23XX_ADC_BASE                  0x400a0000  /* Control and data interface to SAR ADC */
#define RP23XX_PWM_BASE                  0x400a8000  /* Simple PWM */
#define RP23XX_TIMER0_BASE               0x400b0000
#define RP23XX_TIMER1_BASE               0x400b8000
#define RP23XX_TIMER_BASE(n)             (0x400b0000 + (n) * 0x8000)
#define RP23XX_HSTX_CTRL_BASE            0x400c0000
#define RP23XX_XIP_CTRL_BASE             0x400c8000  /* QSPI flash execute-in-place block */
#define RP23XX_XIP_QMI_BASE              0x400d0000
#define RP23XX_WATCHDOG_BASE             0x400d8000
#define RP23XX_BOOTRAM_BASE              0x400e0000
#define RP23XX_ROSC_BASE                 0x400e8000
#define RP23XX_TRNG_BASE                 0x400f0000
#define RP23XX_SHA256_BASE               0x400f8000
#define RP23XX_POWMAN_BASE               0x40100000  /* Controls vreg, bor, lposc, chip resets & xosc startup, powman and provides scratch register for general use and for bootcode use */
#define RP23XX_TICKS_BASE                0x40108000
#define RP23XX_OTP_BASE                  0x40120000
#define RP23XX_OTP_DATA_BASE             0x40130000
#define RP23XX_OTP_DATA_RAW_BASE         0x40134000
#define RP23XX_OTP_DATA_GUARDED_BASE     0x40138000
#define RP23XX_OTP_DATA_RAW_GUARDED_BASE 0x4013c000
#define RP23XX_DFT_BASE                  0x40150000
#define RP23XX_GLITCH_DETECTOR_BASE      0x40158000
#define RP23XX_OTP_BASE                  0x40120000
#define RP23XX_TBMAN_BASE                0x40160000  /* Testbench manager. Allows the programmer to know what platform their software is running on. */
#define RP23XX_DMA_BASE                  0x50000000  /* DMA with separate read and write masters */
#define RP23XX_USBCTRL_DPSRAM_BASE       0x50100000  /* USB Dual Port SRAM */
#define RP23XX_USBCTRL_REGS_BASE         0x50110000  /* USB FS/LS controller device registers */
#define RP23XX_PIO0_BASE                 0x50200000  /* Programmable IO block */
#define RP23XX_PIO1_BASE                 0x50300000  /* Programmable IO block */
#define RP23XX_PIO2_BASE                 0x50400000  /* Programmable IO block */
#define RP23XX_PIO_BASE(n)               (0x50200000 + (n) * 0x100000)
#define RP23XX_XIP_AUX_BASE              0x50500000
#define RP23XX_HSTX_FIFO_BASE            0x50600000
#define RP23XX_CORESIGHT_TRACE_BASE      0x50700000
#define RP23XX_SIO_BASE                  0xd0000000  /* Single-cycle IO block Provides core-local and inter-core hardware for the two processors, with single-cycle access. */
#define RP23XX_SIO_NONSEC_BASE           0xd0020000
#define RP23XX_PPB_BASE                  0xe0000000
#define RP23XX_PPB_NONSEC_BASE           0xe0020000
#define RP23XX_EPPB_BASE                 0xe0080000

#define RP23XX_ATOMIC_XOR_REG_OFFSET         0x1000
#define RP23XX_ATOMIC_SET_REG_OFFSET         0x2000
#define RP23XX_ATOMIC_CLR_REG_OFFSET         0x3000

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/
#ifndef __ASSEMBLY__

#  define xorbits_reg32(v,a)   putreg32(v, (a) | RP23XX_ATOMIC_XOR_REG_OFFSET)
#  define setbits_reg32(v,a)   putreg32(v, (a) | RP23XX_ATOMIC_SET_REG_OFFSET)
#  define clrbits_reg32(v,a)   putreg32(v, (a) | RP23XX_ATOMIC_CLR_REG_OFFSET)
#  define modbits_reg32(v,m,a) xorbits_reg32((getreg32(a) ^ (v)) & (m), a)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_MEMORYMAP_H */
