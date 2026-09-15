/****************************************************************************
 * arch/arm/src/n32h7/hardware/n32h76x_rcc.h
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

#ifndef __ARCH_ARM_SRC_N32H7_HARDWARE_N32H76X_RCC_H
#define __ARCH_ARM_SRC_N32H7_HARDWARE_N32H76X_RCC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"
#include "hardware/n32h7_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define  DWT_CR      (*(volatile uint32_t *)0xE0001000U)
#define  DWT_CYCCNT  (*(volatile uint32_t *)0xE0001004U)
#define  DEM_CR      (*(volatile uint32_t *)0xE000EDFCU)
#define  DEM_CR_TRCENA        ((uint32_t)0x01000000U)
#define  DWT_CR_CYCCNTENA     ((uint32_t)0x00000001U)

#define CPU_DELAY_INTI()    do{                             \
                                /* Enable DWT*/             \
                                DEM_CR |= DEM_CR_TRCENA;    \
                                /* Clear DWT CYCCNT*/       \
                                DWT_CYCCNT = 0U;            \
                                /* Enable DWT CYCCNT*/      \
                                DWT_CR |= DWT_CR_CYCCNTENA; \
                            }while(0)

#define CPU_DELAY_DISABLE() do{                             \
                                /* Disable DWT*/            \
                                DEM_CR &= (uint32_t)(~(uint32_t)DEM_CR_TRCENA); \
                            }while(0)

#define __RCC_DELAY_US(usec)do{                                \
                              uint32_t delay_end;                   \
                              CPU_DELAY_INTI();                     \
                              /* Delay*/                            \
                              delay_end = DWT_CYCCNT + (usec * (600000000/1000000)); \
                              while(DWT_CYCCNT < delay_end){};      \
                              CPU_DELAY_DISABLE();                  \
                            }while(0)

#define __RCC_DELAY_MS(msec)do{                                \
                              uint32_t delay_end;                   \
                              CPU_DELAY_INTI();                     \
                              /* Delay*/                            \
                              delay_end = DWT_CYCCNT + (msec * (600000000/1000)); \
                              while(DWT_CYCCNT < delay_end){};      \
                              CPU_DELAY_DISABLE();                  \
                            }while(0)

/* Register Offsets *********************************************************/

#define N32_RCC_PLL1CTRL1_OFFSET     0x0000  /* PLL1 Control Register 1 */
#define N32_RCC_PLL1CTRL2_OFFSET     0x0004  /* PLL1 Control Register 2 */
#define N32_RCC_PLL2CTRL1_OFFSET     0x0010  /* PLL2 Control Register 1 */
#define N32_RCC_PLL2CTRL2_OFFSET     0x0014  /* PLL2 Control Register 2 */
#define N32_RCC_PLL3CTRL1_OFFSET     0x0020  /* PLL3 Control Register 1 */
#define N32_RCC_PLL3CTRL2_OFFSET     0x0024  /* PLL3 Control Register 2 */
#define N32_RCC_SRCCTRL1_OFFSET      0x0030  /* Source Control Register 1 */
#define N32_RCC_PLL1DIV_OFFSET       0x0034  /* PLL1 Divider Register */
#define N32_RCC_PLL2DIV_OFFSET       0x0038  /* PLL2 Divider Register */
#define N32_RCC_PLL3DIV_OFFSET       0x003C  /* PLL3 Divider Register */
#define N32_RCC_SYSBUSDIV1_OFFSET    0x0040  /* System Bus Divider 1 */
#define N32_RCC_SYSBUSDIV2_OFFSET    0x0044  /* System Bus Divider 2 */
#define N32_RCC_BOOTMODE_OFFSET      0x0048  /* Boot Mode Register */
#define N32_RCC_AHB1DIV1_OFFSET      0x004C  /* AHB1 Divider 1 */
#define N32_RCC_AHB1SEL1_OFFSET      0x0050  /* AHB1 Select 1 */
#define N32_RCC_AHB1EN1_OFFSET       0x0054  /* AHB1 Enable 1 */
#define N32_RCC_AHB1EN2_OFFSET       0x0058  /* AHB1 Enable 2 */
#define N32_RCC_AHB1EN3_OFFSET       0x005C  /* AHB1 Enable 3 */
#define N32_RCC_AHB1EN4_OFFSET       0x0060  /* AHB1 Enable 4 */
#define N32_RCC_AHB1RST1_OFFSET      0x0064  /* AHB1 Reset 1 */
#define N32_RCC_AHB1RST2_OFFSET      0x0068  /* AHB1 Reset 2 */
#define N32_RCC_AHB1RST3_OFFSET      0x006C  /* AHB1 Reset 3 */
#define N32_RCC_AHB1RST4_OFFSET      0x0070  /* AHB1 Reset 4 */
#define N32_RCC_APB1DIV1_OFFSET      0x0074  /* APB1 Divider 1 */
#define N32_RCC_APB1SEL1_OFFSET      0x0078  /* APB1 Select 1 */
#define N32_RCC_APB1SEL2_OFFSET      0x007C  /* APB1 Select 2 */
#define N32_RCC_APB1EN1_OFFSET       0x0080  /* APB1 Enable 1 */
#define N32_RCC_APB1EN2_OFFSET       0x0084  /* APB1 Enable 2 */
#define N32_RCC_APB1EN3_OFFSET       0x0088  /* APB1 Enable 3 */
#define N32_RCC_APB1EN4_OFFSET       0x008C  /* APB1 Enable 4 */
#define N32_RCC_APB1EN5_OFFSET       0x0090  /* APB1 Enable 5 */
#define N32_RCC_APB1RST1_OFFSET      0x0094  /* APB1 Reset 1 */
#define N32_RCC_APB1RST2_OFFSET      0x0098  /* APB1 Reset 2 */
#define N32_RCC_APB1RST3_OFFSET      0x009C  /* APB1 Reset 3 */
#define N32_RCC_APB1RST4_OFFSET      0x00A0  /* APB1 Reset 4 */
#define N32_RCC_APB1RST5_OFFSET      0x00A4  /* APB1 Reset 5 */
#define N32_RCC_AHB2DIV1_OFFSET      0x00A8  /* AHB2 Divider 1 */
#define N32_RCC_AHB2SEL1_OFFSET      0x00AC  /* AHB2 Select 1 */
#define N32_RCC_AHB2EN1_OFFSET       0x00B0  /* AHB2 Enable 1 */
#define N32_RCC_AHB2RST1_OFFSET      0x00B4  /* AHB2 Reset 1 */
#define N32_RCC_APB2DIV1_OFFSET      0x00B8  /* APB2 Divider 1 */
#define N32_RCC_APB2SEL1_OFFSET      0x00BC  /* APB2 Select 1 */
#define N32_RCC_APB2SEL2_OFFSET      0x00C0  /* APB2 Select 2 */
#define N32_RCC_APB2EN1_OFFSET       0x00C4  /* APB2 Enable 1 */
#define N32_RCC_APB2EN2_OFFSET       0x00C8  /* APB2 Enable 2 */
#define N32_RCC_APB2EN3_OFFSET       0x00CC  /* APB2 Enable 3 */
#define N32_RCC_APB2EN4_OFFSET       0x00D0  /* APB2 Enable 4 */
#define N32_RCC_APB2RST1_OFFSET      0x00D4  /* APB2 Reset 1 */
#define N32_RCC_APB2RST2_OFFSET      0x00D8  /* APB2 Reset 2 */
#define N32_RCC_APB2RST3_OFFSET      0x00DC  /* APB2 Reset 3 */
#define N32_RCC_APB2RST4_OFFSET      0x00E0  /* APB2 Reset 4 */
#define N32_RCC_AHB5EN1_OFFSET       0x00E4  /* AHB5 Enable 1 */
#define N32_RCC_AHB5EN2_OFFSET       0x00E8  /* AHB5 Enable 2 */
#define N32_RCC_AHB5RST1_OFFSET      0x00EC  /* AHB5 Reset 1 */
#define N32_RCC_AHB5RST2_OFFSET      0x00F0  /* AHB5 Reset 2 */
#define N32_RCC_APB5DIV1_OFFSET      0x00F4  /* APB5 Divider 1 */
#define N32_RCC_APB5SEL1_OFFSET      0x00F8  /* APB5 Select 1 */
#define N32_RCC_APB5EN1_OFFSET       0x00FC  /* APB5 Enable 1 */
#define N32_RCC_APB5EN2_OFFSET       0x0100  /* APB5 Enable 2 */
#define N32_RCC_APB5RST1_OFFSET      0x0104  /* APB5 Reset 1 */
#define N32_RCC_APB5RST2_OFFSET      0x0108  /* APB5 Reset 2 */
#define N32_RCC_RDDIV1_OFFSET        0x010C  /* RD Divider 1 */
#define N32_RCC_RDSEL1_OFFSET        0x0110  /* RD Select 1 */
#define N32_RCC_RDEN1_OFFSET         0x0114  /* RD Enable 1 */
#define N32_RCC_RDEN2_OFFSET         0x0118  /* RD Enable 2 */
#define N32_RCC_RDRST1_OFFSET        0x011C  /* RD Reset 1 */
#define N32_RCC_RDRST2_OFFSET        0x0120  /* RD Reset 2 */
#define N32_RCC_BDCTRL_OFFSET        0x0124  /* Backup Domain Control */
#define N32_RCC_CTRLSTS_OFFSET       0x0128  /* Control and Status */
#define N32_RCC_CLKINT1_OFFSET       0x012C  /* Clock Interrupt 1 */
#define N32_RCC_CLKINT2_OFFSET       0x0130  /* Clock Interrupt 2 */
#define N32_RCC_CFG1_OFFSET          0x0134  /* Configuration 1 */
#define N32_RCC_AXIDIV1_OFFSET       0x0138  /* AXI Divider 1 */
#define N32_RCC_AXISEL1_OFFSET       0x013C  /* AXI Select 1 */
#define N32_RCC_AXIEN1_OFFSET        0x0140  /* AXI Enable 1 */
#define N32_RCC_AXIEN2_OFFSET        0x0144  /* AXI Enable 2 */
#define N32_RCC_AXIEN3_OFFSET        0x0148  /* AXI Enable 3 */
#define N32_RCC_AXIEN4_OFFSET        0x014C  /* AXI Enable 4 */
#define N32_RCC_AXIRST1_OFFSET       0x0150  /* AXI Reset 1 */
#define N32_RCC_AXIRST2_OFFSET       0x0154  /* AXI Reset 2 */
#define N32_RCC_AXIRST3_OFFSET       0x0158  /* AXI Reset 3 */
#define N32_RCC_AXIRST4_OFFSET       0x015C  /* AXI Reset 4 */
#define N32_RCC_CFG2_OFFSET          0x0160  /* Configuration 2 */
#define N32_RCC_CFG3_OFFSET          0x0164  /* Configuration 3 */
#define N32_RCC_CFG4_OFFSET          0x0168  /* Configuration 4 */
#define N32_RCC_SRCCTRL2_OFFSET      0x016C  /* Source Control 2 */
#define N32_RCC_CFG5_OFFSET          0x0170  /* Configuration 5 */
#define N32_RCC_M4RSTREL_OFFSET      0x0174  /* M4 Reset Release */
#define N32_RCC_AXIDIV2_OFFSET       0x0178  /* AXI Divider 2 */
#define N32_RCC_AXISEL2_OFFSET       0x017C  /* AXI Select 2 */
#define N32_RCC_SHRPLLCTRL1_OFFSET   0x018C  /* Shared PLL Control 1 */
#define N32_RCC_SHRPLLCTRL2_OFFSET   0x0190  /* Shared PLL Control 2 */
#define N32_RCC_AHB1DIV2_OFFSET      0x0194  /* AHB1 Divider 2 */
#define N32_RCC_LSERDDL_OFFSET       0x0198  /* LSE Ready Delay */
#define N32_RCC_MSIRDDL_OFFSET       0x019C  /* MSI Ready Delay */
#define N32_RCC_HSERDDL_OFFSET       0x01A0  /* HSE Ready Delay */
#define N32_RCC_PLLSFTLK_OFFSET      0x01A4  /* PLL Soft Lock */
#define N32_RCC_RDCTRL1_OFFSET       0x01A8  /* RD Control 1 */
#define N32_RCC_RDCTRL2_OFFSET       0x01AC  /* RD Control 2 */
#define N32_RCC_RDCTRL3_OFFSET       0x01B0  /* RD Control 3 */
#define N32_RCC_AHB2EN2_OFFSET       0x01B4  /* AHB2 Enable 2 */
#define N32_RCC_AHB9DIV1_OFFSET      0x01B8  /* AHB9 Divider 1 */
#define N32_RCC_AHB9SEL1_OFFSET      0x01BC  /* AHB9 Select 1 */
#define N32_RCC_AHB9EN1_OFFSET       0x01C0  /* AHB9 Enable 1 */
#define N32_RCC_AHB9RST1_OFFSET      0x01C4  /* AHB9 Reset 1 */
#define N32_RCC_HSEOS_OFFSET         0x01C8  /* HSE Oscillator Status */
#define N32_RCC_LSEOS_OFFSET         0x01CC  /* LSE Oscillator Status */
#define N32_RCC_HSECAL_OFFSET        0x01D0  /* HSE Calibration */
#define N32_RCC_CLKINT3_OFFSET       0x01D4  /* Clock Interrupt 3 */
#define N32_RCC_PLLFD_OFFSET         0x01D8  /* PLL Frequency Detect */
#define N32_RCC_SRCCTRL3_OFFSET      0x01DC  /* Source Control 3 */
#define N32_RCC_LSICSSDL_OFFSET      0x01E0  /* LSI CSS Delay */

/* Register Addresses *******************************************************/

#define N32_RCC_PLL1CTRL1           (N32_RCC_BASE + N32_RCC_PLL1CTRL1_OFFSET)  /* PLL1 Control Register 1 */
#define N32_RCC_PLL1CTRL2           (N32_RCC_BASE + N32_RCC_PLL1CTRL2_OFFSET)  /* PLL1 Control Register 2 */
#define N32_RCC_PLL2CTRL1           (N32_RCC_BASE + N32_RCC_PLL2CTRL1_OFFSET)  /* PLL2 Control Register 1 */
#define N32_RCC_PLL2CTRL2           (N32_RCC_BASE + N32_RCC_PLL2CTRL2_OFFSET)  /* PLL2 Control Register 2 */
#define N32_RCC_PLL3CTRL1           (N32_RCC_BASE + N32_RCC_PLL3CTRL1_OFFSET)  /* PLL3 Control Register 1 */
#define N32_RCC_PLL3CTRL2           (N32_RCC_BASE + N32_RCC_PLL3CTRL2_OFFSET)  /* PLL3 Control Register 2 */
#define N32_RCC_SRCCTRL1            (N32_RCC_BASE + N32_RCC_SRCCTRL1_OFFSET)   /* Source Control Register 1 */
#define N32_RCC_PLL1DIV             (N32_RCC_BASE + N32_RCC_PLL1DIV_OFFSET)    /* PLL1 Divider Register */
#define N32_RCC_PLL2DIV             (N32_RCC_BASE + N32_RCC_PLL2DIV_OFFSET)    /* PLL2 Divider Register */
#define N32_RCC_PLL3DIV             (N32_RCC_BASE + N32_RCC_PLL3DIV_OFFSET)    /* PLL3 Divider Register */
#define N32_RCC_SYSBUSDIV1          (N32_RCC_BASE + N32_RCC_SYSBUSDIV1_OFFSET) /* System Bus Divider 1 */
#define N32_RCC_SYSBUSDIV2          (N32_RCC_BASE + N32_RCC_SYSBUSDIV2_OFFSET) /* System Bus Divider 2 */
#define N32_RCC_BOOTMODE            (N32_RCC_BASE + N32_RCC_BOOTMODE_OFFSET)   /* Boot Mode Register */
#define N32_RCC_AHB1DIV1            (N32_RCC_BASE + N32_RCC_AHB1DIV1_OFFSET)   /* AHB1 Divider 1 */
#define N32_RCC_AHB1SEL1            (N32_RCC_BASE + N32_RCC_AHB1SEL1_OFFSET)   /* AHB1 Select 1 */
#define N32_RCC_AHB1EN1             (N32_RCC_BASE + N32_RCC_AHB1EN1_OFFSET)    /* AHB1 Enable 1 */
#define N32_RCC_AHB1EN2             (N32_RCC_BASE + N32_RCC_AHB1EN2_OFFSET)    /* AHB1 Enable 2 */
#define N32_RCC_AHB1EN3             (N32_RCC_BASE + N32_RCC_AHB1EN3_OFFSET)    /* AHB1 Enable 3 */
#define N32_RCC_AHB1EN4             (N32_RCC_BASE + N32_RCC_AHB1EN4_OFFSET)    /* AHB1 Enable 4 */
#define N32_RCC_AHB1RST1            (N32_RCC_BASE + N32_RCC_AHB1RST1_OFFSET)   /* AHB1 Reset 1 */
#define N32_RCC_AHB1RST2            (N32_RCC_BASE + N32_RCC_AHB1RST2_OFFSET)   /* AHB1 Reset 2 */
#define N32_RCC_AHB1RST3            (N32_RCC_BASE + N32_RCC_AHB1RST3_OFFSET)   /* AHB1 Reset 3 */
#define N32_RCC_AHB1RST4            (N32_RCC_BASE + N32_RCC_AHB1RST4_OFFSET)   /* AHB1 Reset 4 */
#define N32_RCC_APB1DIV1            (N32_RCC_BASE + N32_RCC_APB1DIV1_OFFSET)   /* APB1 Divider 1 */
#define N32_RCC_APB1SEL1            (N32_RCC_BASE + N32_RCC_APB1SEL1_OFFSET)   /* APB1 Select 1 */
#define N32_RCC_APB1SEL2            (N32_RCC_BASE + N32_RCC_APB1SEL2_OFFSET)   /* APB1 Select 2 */
#define N32_RCC_APB1EN1             (N32_RCC_BASE + N32_RCC_APB1EN1_OFFSET)    /* APB1 Enable 1 */
#define N32_RCC_APB1EN2             (N32_RCC_BASE + N32_RCC_APB1EN2_OFFSET)    /* APB1 Enable 2 */
#define N32_RCC_APB1EN3             (N32_RCC_BASE + N32_RCC_APB1EN3_OFFSET)    /* APB1 Enable 3 */
#define N32_RCC_APB1EN4             (N32_RCC_BASE + N32_RCC_APB1EN4_OFFSET)    /* APB1 Enable 4 */
#define N32_RCC_APB1EN5             (N32_RCC_BASE + N32_RCC_APB1EN5_OFFSET)    /* APB1 Enable 5 */
#define N32_RCC_APB1RST1            (N32_RCC_BASE + N32_RCC_APB1RST1_OFFSET)   /* APB1 Reset 1 */
#define N32_RCC_APB1RST2            (N32_RCC_BASE + N32_RCC_APB1RST2_OFFSET)   /* APB1 Reset 2 */
#define N32_RCC_APB1RST3            (N32_RCC_BASE + N32_RCC_APB1RST3_OFFSET)   /* APB1 Reset 3 */
#define N32_RCC_APB1RST4            (N32_RCC_BASE + N32_RCC_APB1RST4_OFFSET)   /* APB1 Reset 4 */
#define N32_RCC_APB1RST5            (N32_RCC_BASE + N32_RCC_APB1RST5_OFFSET)   /* APB1 Reset 5 */
#define N32_RCC_AHB2DIV1            (N32_RCC_BASE + N32_RCC_AHB2DIV1_OFFSET)   /* AHB2 Divider 1 */
#define N32_RCC_AHB2SEL1            (N32_RCC_BASE + N32_RCC_AHB2SEL1_OFFSET)   /* AHB2 Select 1 */
#define N32_RCC_AHB2EN1             (N32_RCC_BASE + N32_RCC_AHB2EN1_OFFSET)    /* AHB2 Enable 1 */
#define N32_RCC_AHB2RST1            (N32_RCC_BASE + N32_RCC_AHB2RST1_OFFSET)   /* AHB2 Reset 1 */
#define N32_RCC_APB2DIV1            (N32_RCC_BASE + N32_RCC_APB2DIV1_OFFSET)   /* APB2 Divider 1 */
#define N32_RCC_APB2SEL1            (N32_RCC_BASE + N32_RCC_APB2SEL1_OFFSET)   /* APB2 Select 1 */
#define N32_RCC_APB2SEL2            (N32_RCC_BASE + N32_RCC_APB2SEL2_OFFSET)   /* APB2 Select 2 */
#define N32_RCC_APB2EN1             (N32_RCC_BASE + N32_RCC_APB2EN1_OFFSET)    /* APB2 Enable 1 */
#define N32_RCC_APB2EN2             (N32_RCC_BASE + N32_RCC_APB2EN2_OFFSET)    /* APB2 Enable 2 */
#define N32_RCC_APB2EN3             (N32_RCC_BASE + N32_RCC_APB2EN3_OFFSET)    /* APB2 Enable 3 */
#define N32_RCC_APB2EN4             (N32_RCC_BASE + N32_RCC_APB2EN4_OFFSET)    /* APB2 Enable 4 */
#define N32_RCC_APB2RST1            (N32_RCC_BASE + N32_RCC_APB2RST1_OFFSET)   /* APB2 Reset 1 */
#define N32_RCC_APB2RST2            (N32_RCC_BASE + N32_RCC_APB2RST2_OFFSET)   /* APB2 Reset 2 */
#define N32_RCC_APB2RST3            (N32_RCC_BASE + N32_RCC_APB2RST3_OFFSET)   /* APB2 Reset 3 */
#define N32_RCC_APB2RST4            (N32_RCC_BASE + N32_RCC_APB2RST4_OFFSET)   /* APB2 Reset 4 */
#define N32_RCC_AHB5EN1             (N32_RCC_BASE + N32_RCC_AHB5EN1_OFFSET)    /* AHB5 Enable 1 */
#define N32_RCC_AHB5EN2             (N32_RCC_BASE + N32_RCC_AHB5EN2_OFFSET)    /* AHB5 Enable 2 */
#define N32_RCC_AHB5RST1            (N32_RCC_BASE + N32_RCC_AHB5RST1_OFFSET)   /* AHB5 Reset 1 */
#define N32_RCC_AHB5RST2            (N32_RCC_BASE + N32_RCC_AHB5RST2_OFFSET)   /* AHB5 Reset 2 */
#define N32_RCC_APB5DIV1            (N32_RCC_BASE + N32_RCC_APB5DIV1_OFFSET)   /* APB5 Divider 1 */
#define N32_RCC_APB5SEL1            (N32_RCC_BASE + N32_RCC_APB5SEL1_OFFSET)   /* APB5 Select 1 */
#define N32_RCC_APB5EN1             (N32_RCC_BASE + N32_RCC_APB5EN1_OFFSET)    /* APB5 Enable 1 */
#define N32_RCC_APB5EN2             (N32_RCC_BASE + N32_RCC_APB5EN2_OFFSET)    /* APB5 Enable 2 */
#define N32_RCC_APB5RST1            (N32_RCC_BASE + N32_RCC_APB5RST1_OFFSET)   /* APB5 Reset 1 */
#define N32_RCC_APB5RST2            (N32_RCC_BASE + N32_RCC_APB5RST2_OFFSET)   /* APB5 Reset 2 */
#define N32_RCC_RDDIV1              (N32_RCC_BASE + N32_RCC_RDDIV1_OFFSET)     /* RD Divider 1 */
#define N32_RCC_RDSEL1              (N32_RCC_BASE + N32_RCC_RDSEL1_OFFSET)     /* RD Select 1 */
#define N32_RCC_RDEN1               (N32_RCC_BASE + N32_RCC_RDEN1_OFFSET)      /* RD Enable 1 */
#define N32_RCC_RDEN2               (N32_RCC_BASE + N32_RCC_RDEN2_OFFSET)      /* RD Enable 2 */
#define N32_RCC_RDRST1              (N32_RCC_BASE + N32_RCC_RDRST1_OFFSET)     /* RD Reset 1 */
#define N32_RCC_RDRST2              (N32_RCC_BASE + N32_RCC_RDRST2_OFFSET)     /* RD Reset 2 */
#define N32_RCC_BDCTRL              (N32_RCC_BASE + N32_RCC_BDCTRL_OFFSET)     /* Backup Domain Control */
#define N32_RCC_CTRLSTS             (N32_RCC_BASE + N32_RCC_CTRLSTS_OFFSET)    /* Control and Status */
#define N32_RCC_CLKINT1             (N32_RCC_BASE + N32_RCC_CLKINT1_OFFSET)    /* Clock Interrupt 1 */
#define N32_RCC_CLKINT2             (N32_RCC_BASE + N32_RCC_CLKINT2_OFFSET)    /* Clock Interrupt 2 */
#define N32_RCC_CFG1                (N32_RCC_BASE + N32_RCC_CFG1_OFFSET)       /* Configuration 1 */
#define N32_RCC_AXIDIV1             (N32_RCC_BASE + N32_RCC_AXIDIV1_OFFSET)    /* AXI Divider 1 */
#define N32_RCC_AXISEL1             (N32_RCC_BASE + N32_RCC_AXISEL1_OFFSET)    /* AXI Select 1 */
#define N32_RCC_AXIEN1              (N32_RCC_BASE + N32_RCC_AXIEN1_OFFSET)     /* AXI Enable 1 */
#define N32_RCC_AXIEN2              (N32_RCC_BASE + N32_RCC_AXIEN2_OFFSET)     /* AXI Enable 2 */
#define N32_RCC_AXIEN3              (N32_RCC_BASE + N32_RCC_AXIEN3_OFFSET)     /* AXI Enable 3 */
#define N32_RCC_AXIEN4              (N32_RCC_BASE + N32_RCC_AXIEN4_OFFSET)     /* AXI Enable 4 */
#define N32_RCC_AXIRST1             (N32_RCC_BASE + N32_RCC_AXIRST1_OFFSET)    /* AXI Reset 1 */
#define N32_RCC_AXIRST2             (N32_RCC_BASE + N32_RCC_AXIRST2_OFFSET)    /* AXI Reset 2 */
#define N32_RCC_AXIRST3             (N32_RCC_BASE + N32_RCC_AXIRST3_OFFSET)    /* AXI Reset 3 */
#define N32_RCC_AXIRST4             (N32_RCC_BASE + N32_RCC_AXIRST4_OFFSET)    /* AXI Reset 4 */
#define N32_RCC_CFG2                (N32_RCC_BASE + N32_RCC_CFG2_OFFSET)       /* Configuration 2 */
#define N32_RCC_CFG3                (N32_RCC_BASE + N32_RCC_CFG3_OFFSET)       /* Configuration 3 */
#define N32_RCC_CFG4                (N32_RCC_BASE + N32_RCC_CFG4_OFFSET)       /* Configuration 4 */
#define N32_RCC_SRCCTRL2            (N32_RCC_BASE + N32_RCC_SRCCTRL2_OFFSET)   /* Source Control 2 */
#define N32_RCC_CFG5                (N32_RCC_BASE + N32_RCC_CFG5_OFFSET)       /* Configuration 5 */
#define N32_RCC_M4RSTREL            (N32_RCC_BASE + N32_RCC_M4RSTREL_OFFSET)   /* M4 Reset Release */
#define N32_RCC_AXIDIV2             (N32_RCC_BASE + N32_RCC_AXIDIV2_OFFSET)    /* AXI Divider 2 */
#define N32_RCC_AXISEL2             (N32_RCC_BASE + N32_RCC_AXISEL2_OFFSET)    /* AXI Select 2 */
#define N32_RCC_SHRPLLCTRL1         (N32_RCC_BASE + N32_RCC_SHRPLLCTRL1_OFFSET)/* Shared PLL Control 1 */
#define N32_RCC_SHRPLLCTRL2         (N32_RCC_BASE + N32_RCC_SHRPLLCTRL2_OFFSET)/* Shared PLL Control 2 */
#define N32_RCC_AHB1DIV2            (N32_RCC_BASE + N32_RCC_AHB1DIV2_OFFSET)   /* AHB1 Divider 2 */
#define N32_RCC_LSERDDL             (N32_RCC_BASE + N32_RCC_LSERDDL_OFFSET)    /* LSE Ready Delay */
#define N32_RCC_MSIRDDL             (N32_RCC_BASE + N32_RCC_MSIRDDL_OFFSET)    /* MSI Ready Delay */
#define N32_RCC_HSERDDL             (N32_RCC_BASE + N32_RCC_HSERDDL_OFFSET)    /* HSE Ready Delay */
#define N32_RCC_PLLSFTLK            (N32_RCC_BASE + N32_RCC_PLLSFTLK_OFFSET)   /* PLL Soft Lock */
#define N32_RCC_RDCTRL1             (N32_RCC_BASE + N32_RCC_RDCTRL1_OFFSET)    /* RD Control 1 */
#define N32_RCC_RDCTRL2             (N32_RCC_BASE + N32_RCC_RDCTRL2_OFFSET)    /* RD Control 2 */
#define N32_RCC_RDCTRL3             (N32_RCC_BASE + N32_RCC_RDCTRL3_OFFSET)    /* RD Control 3 */
#define N32_RCC_AHB2EN2             (N32_RCC_BASE + N32_RCC_AHB2EN2_OFFSET)    /* AHB2 Enable 2 */
#define N32_RCC_AHB9DIV1            (N32_RCC_BASE + N32_RCC_AHB9DIV1_OFFSET)   /* AHB9 Divider 1 */
#define N32_RCC_AHB9SEL1            (N32_RCC_BASE + N32_RCC_AHB9SEL1_OFFSET)   /* AHB9 Select 1 */
#define N32_RCC_AHB9EN1             (N32_RCC_BASE + N32_RCC_AHB9EN1_OFFSET)    /* AHB9 Enable 1 */
#define N32_RCC_AHB9RST1            (N32_RCC_BASE + N32_RCC_AHB9RST1_OFFSET)   /* AHB9 Reset 1 */
#define N32_RCC_HSEOS               (N32_RCC_BASE + N32_RCC_HSEOS_OFFSET)      /* HSE Oscillator Status */
#define N32_RCC_LSEOS               (N32_RCC_BASE + N32_RCC_LSEOS_OFFSET)      /* LSE Oscillator Status */
#define N32_RCC_HSECAL              (N32_RCC_BASE + N32_RCC_HSECAL_OFFSET)     /* HSE Calibration */
#define N32_RCC_CLKINT3             (N32_RCC_BASE + N32_RCC_CLKINT3_OFFSET)    /* Clock Interrupt 3 */
#define N32_RCC_PLLFD               (N32_RCC_BASE + N32_RCC_PLLFD_OFFSET)      /* PLL Frequency Detect */
#define N32_RCC_SRCCTRL3            (N32_RCC_BASE + N32_RCC_SRCCTRL3_OFFSET)   /* Source Control 3 */
#define N32_RCC_LSICSSDL            (N32_RCC_BASE + N32_RCC_LSICSSDL_OFFSET)   /* LSI CSS Delay */

/* Register Bitfield Definitions ********************************************/

/* Bit definition for RCC_PLL1CTRL1 register ********************************/
#define RCC_PLL1CTRL1_PLL1SRC_SHIFT             (28)       /* Bits 28-29: PLL1 clock source selection */
#define RCC_PLL1CTRL1_PLL1SRC_MASK              (0x3 << RCC_PLL1CTRL1_PLL1SRC_SHIFT)
#define RCC_PLL1CTRL1_PLL1SRC_0                 (1 << 28) /* Bit 28 */
#define RCC_PLL1CTRL1_PLL1SRC_1                 (1 << 29) /* Bit 29 */

#define RCC_PLL1CTRL1_PLL1SRC_HSI               (0x0 << RCC_PLL1CTRL1_PLL1SRC_SHIFT) /* HSI Clock is the PLL source clock */
#define RCC_PLL1CTRL1_PLL1SRC_NONE              (0x1 << RCC_PLL1CTRL1_PLL1SRC_SHIFT) /* No clock */
#define RCC_PLL1CTRL1_PLL1SRC_MSI               (0x2 << RCC_PLL1CTRL1_PLL1SRC_SHIFT) /* MSI Clock is the PLL source clock */
#define RCC_PLL1CTRL1_PLL1SRC_HSE               (0x3 << RCC_PLL1CTRL1_PLL1SRC_SHIFT) /* HSE Clock is the PLL source clock */

#define RCC_PLL1CTRL1_PLL1PHLK                  (1 << 20) /* Bit 20: PLL1 PH lock */
#define RCC_PLL1CTRL1_PLL1LDOEN                 (1 << 19) /* Bit 19: PLL1 LDO enable */
#define RCC_PLL1CTRL1_PLL1EN                    (1 << 18) /* Bit 18: PLL1 enable */
#define RCC_PLL1CTRL1_PLL1RST                   (1 << 17) /* Bit 17: PLL1 reset */
#define RCC_PLL1CTRL1_PLL1PD                    (1 << 16) /* Bit 16: PLL1 power down */

#define RCC_PLL1CTRL1_PLL1BWAJ_SHIFT            (0)       /* Bits 0-11: PLL1 BWAJ value */
#define RCC_PLL1CTRL1_PLL1BWAJ_MASK             (0xFFF << RCC_PLL1CTRL1_PLL1BWAJ_SHIFT)
#define RCC_PLL1CTRL1_PLL1BWAJ_0                (1 << 0)  /* Bit 0 */
#define RCC_PLL1CTRL1_PLL1BWAJ_1                (1 << 1)  /* Bit 1 */
#define RCC_PLL1CTRL1_PLL1BWAJ_2                (1 << 2)  /* Bit 2 */
#define RCC_PLL1CTRL1_PLL1BWAJ_3                (1 << 3)  /* Bit 3 */
#define RCC_PLL1CTRL1_PLL1BWAJ_4                (1 << 4)  /* Bit 4 */
#define RCC_PLL1CTRL1_PLL1BWAJ_5                (1 << 5)  /* Bit 5 */
#define RCC_PLL1CTRL1_PLL1BWAJ_6                (1 << 6)  /* Bit 6 */
#define RCC_PLL1CTRL1_PLL1BWAJ_7                (1 << 7)  /* Bit 7 */
#define RCC_PLL1CTRL1_PLL1BWAJ_8                (1 << 8)  /* Bit 8 */
#define RCC_PLL1CTRL1_PLL1BWAJ_9                (1 << 9)  /* Bit 9 */
#define RCC_PLL1CTRL1_PLL1BWAJ_10               (1 << 10) /* Bit 10 */
#define RCC_PLL1CTRL1_PLL1BWAJ_11               (1 << 11) /* Bit 11 */

#define RCC_PLL1CTRL1_PLL1BWAJ(n)               ((n) << RCC_PLL1CTRL1_PLL1BWAJ_SHIFT)

/* Bit definition for RCC_PLL1CTRL2 register ********************************/
#define RCC_PLL1CTRL2_PLL1CLKR_SHIFT            (26)      /* Bits 26-31: PLL1 clock R divider */
#define RCC_PLL1CTRL2_PLL1CLKR_MASK             (0x3F << RCC_PLL1CTRL2_PLL1CLKR_SHIFT)
#define RCC_PLL1CTRL2_PLL1CLKR_0                (1 << 26) /* Bit 26 */
#define RCC_PLL1CTRL2_PLL1CLKR_1                (1 << 27) /* Bit 27 */
#define RCC_PLL1CTRL2_PLL1CLKR_2                (1 << 28) /* Bit 28 */
#define RCC_PLL1CTRL2_PLL1CLKR_3                (1 << 29) /* Bit 29 */
#define RCC_PLL1CTRL2_PLL1CLKR_4                (1 << 30) /* Bit 30 */
#define RCC_PLL1CTRL2_PLL1CLKR_5                (1 << 31) /* Bit 31 */

#define RCC_PLL1CTRL2_PLL1CLKR(n)               ((n) << RCC_PLL1CTRL2_PLL1CLKR_SHIFT)

#define RCC_PLL1CTRL2_PLL1CLKF_SHIFT            (0)        /* Bits 0-25: PLL1 clock F multiplier */
#define RCC_PLL1CTRL2_PLL1CLKF_MASK             (0x3FFFFFF << RCC_PLL1CTRL2_PLL1CLKF_SHIFT)
#define RCC_PLL1CTRL2_PLL1CLKF_0                (1 << 0)   /* Bit 0 */
#define RCC_PLL1CTRL2_PLL1CLKF_1                (1 << 1)   /* Bit 1 */
#define RCC_PLL1CTRL2_PLL1CLKF_2                (1 << 2)   /* Bit 2 */
#define RCC_PLL1CTRL2_PLL1CLKF_3                (1 << 3)   /* Bit 3 */
#define RCC_PLL1CTRL2_PLL1CLKF_4                (1 << 4)   /* Bit 4 */
#define RCC_PLL1CTRL2_PLL1CLKF_5                (1 << 5)   /* Bit 5 */
#define RCC_PLL1CTRL2_PLL1CLKF_6                (1 << 6)   /* Bit 6 */
#define RCC_PLL1CTRL2_PLL1CLKF_7                (1 << 7)   /* Bit 7 */
#define RCC_PLL1CTRL2_PLL1CLKF_8                (1 << 8)   /* Bit 8 */
#define RCC_PLL1CTRL2_PLL1CLKF_9                (1 << 9)   /* Bit 9 */
#define RCC_PLL1CTRL2_PLL1CLKF_10               (1 << 10)  /* Bit 10 */
#define RCC_PLL1CTRL2_PLL1CLKF_11               (1 << 11)  /* Bit 11 */
#define RCC_PLL1CTRL2_PLL1CLKF_12               (1 << 12)  /* Bit 12 */
#define RCC_PLL1CTRL2_PLL1CLKF_13               (1 << 13)  /* Bit 13 */
#define RCC_PLL1CTRL2_PLL1CLKF_14               (1 << 14)  /* Bit 14 */
#define RCC_PLL1CTRL2_PLL1CLKF_15               (1 << 15)  /* Bit 15 */
#define RCC_PLL1CTRL2_PLL1CLKF_16               (1 << 16)  /* Bit 16 */
#define RCC_PLL1CTRL2_PLL1CLKF_17               (1 << 17)  /* Bit 17 */
#define RCC_PLL1CTRL2_PLL1CLKF_18               (1 << 18)  /* Bit 18 */
#define RCC_PLL1CTRL2_PLL1CLKF_19               (1 << 19)  /* Bit 19 */
#define RCC_PLL1CTRL2_PLL1CLKF_20               (1 << 20)  /* Bit 20 */
#define RCC_PLL1CTRL2_PLL1CLKF_21               (1 << 21)  /* Bit 21 */
#define RCC_PLL1CTRL2_PLL1CLKF_22               (1 << 22)  /* Bit 22 */
#define RCC_PLL1CTRL2_PLL1CLKF_23               (1 << 23)  /* Bit 23 */
#define RCC_PLL1CTRL2_PLL1CLKF_24               (1 << 24)  /* Bit 24 */
#define RCC_PLL1CTRL2_PLL1CLKF_25               (1 << 25)  /* Bit 25 */

#define RCC_PLL1CTRL2_PLL1CLKF(n)               ((n) << RCC_PLL1CTRL2_PLL1CLKF_SHIFT)

/* Bit definition for RCC_PLL2CTRL1 register ********************************/
#define RCC_PLL2CTRL1_PLL2SRC_SHIFT             (28)       /* Bits 28-29: PLL2 clock source selection */
#define RCC_PLL2CTRL1_PLL2SRC_MASK              (0x3 << RCC_PLL2CTRL1_PLL2SRC_SHIFT)
#define RCC_PLL2CTRL1_PLL2SRC_0                 (1 << 28) /* Bit 28 */
#define RCC_PLL2CTRL1_PLL2SRC_1                 (1 << 29) /* Bit 29 */

#define RCC_PLL2CTRL1_PLL2SRC_HSI               (0x0 << RCC_PLL2CTRL1_PLL2SRC_SHIFT) /* HSI Clock is the PLL source clock */
#define RCC_PLL2CTRL1_PLL2SRC_NONE              (0x1 << RCC_PLL2CTRL1_PLL2SRC_SHIFT) /* No clock */
#define RCC_PLL2CTRL1_PLL2SRC_MSI               (0x2 << RCC_PLL2CTRL1_PLL2SRC_SHIFT) /* MSI Clock is the PLL source clock */
#define RCC_PLL2CTRL1_PLL2SRC_HSE               (0x3 << RCC_PLL2CTRL1_PLL2SRC_SHIFT) /* HSE Clock is the PLL source clock */

#define RCC_PLL2CTRL1_PLL2PHLK                  (1 << 20) /* Bit 20: PLL2 PH lock */
#define RCC_PLL2CTRL1_PLL2LDOEN                 (1 << 19) /* Bit 19: PLL2 LDO enable */
#define RCC_PLL2CTRL1_PLL2EN                    (1 << 18) /* Bit 18: PLL2 enable */
#define RCC_PLL2CTRL1_PLL2RST                   (1 << 17) /* Bit 17: PLL2 reset */
#define RCC_PLL2CTRL1_PLL2PD                    (1 << 16) /* Bit 16: PLL2 power down */

#define RCC_PLL2CTRL1_PLL2BWAJ_SHIFT            (0)       /* Bits 0-11: PLL2 BWAJ value */
#define RCC_PLL2CTRL1_PLL2BWAJ_MASK             (0xFFF << RCC_PLL2CTRL1_PLL2BWAJ_SHIFT)
#define RCC_PLL2CTRL1_PLL2BWAJ_0                (1 << 0)  /* Bit 0 */
#define RCC_PLL2CTRL1_PLL2BWAJ_1                (1 << 1)  /* Bit 1 */
#define RCC_PLL2CTRL1_PLL2BWAJ_2                (1 << 2)  /* Bit 2 */
#define RCC_PLL2CTRL1_PLL2BWAJ_3                (1 << 3)  /* Bit 3 */
#define RCC_PLL2CTRL1_PLL2BWAJ_4                (1 << 4)  /* Bit 4 */
#define RCC_PLL2CTRL1_PLL2BWAJ_5                (1 << 5)  /* Bit 5 */
#define RCC_PLL2CTRL1_PLL2BWAJ_6                (1 << 6)  /* Bit 6 */
#define RCC_PLL2CTRL1_PLL2BWAJ_7                (1 << 7)  /* Bit 7 */
#define RCC_PLL2CTRL1_PLL2BWAJ_8                (1 << 8)  /* Bit 8 */
#define RCC_PLL2CTRL1_PLL2BWAJ_9                (1 << 9)  /* Bit 9 */
#define RCC_PLL2CTRL1_PLL2BWAJ_10               (1 << 10) /* Bit 10 */
#define RCC_PLL2CTRL1_PLL2BWAJ_11               (1 << 11) /* Bit 11 */

#define RCC_PLL2CTRL1_PLL2BWAJ(n)               ((n) << RCC_PLL2CTRL1_PLL2BWAJ_SHIFT)

/* Bit definition for RCC_PLL2CTRL2 register ********************************/
#define RCC_PLL2CTRL2_PLL2CLKR_SHIFT            (26)       /* Bits 26-31: PLL2 clock R divider */
#define RCC_PLL2CTRL2_PLL2CLKR_MASK             (0x3F << RCC_PLL2CTRL2_PLL2CLKR_SHIFT)
#define RCC_PLL2CTRL2_PLL2CLKR_0                (1 << 26) /* Bit 26 */
#define RCC_PLL2CTRL2_PLL2CLKR_1                (1 << 27) /* Bit 27 */
#define RCC_PLL2CTRL2_PLL2CLKR_2                (1 << 28) /* Bit 28 */
#define RCC_PLL2CTRL2_PLL2CLKR_3                (1 << 29) /* Bit 29 */
#define RCC_PLL2CTRL2_PLL2CLKR_4                (1 << 30) /* Bit 30 */
#define RCC_PLL2CTRL2_PLL2CLKR_5                (1 << 31) /* Bit 31 */

#define RCC_PLL2CTRL2_PLL2CLKR(n)               ((n) << RCC_PLL2CTRL2_PLL2CLKR_SHIFT)

#define RCC_PLL2CTRL2_PLL2CLKF_SHIFT            (0)        /* Bits 0-25: PLL2 clock F multiplier */
#define RCC_PLL2CTRL2_PLL2CLKF_MASK             (0x3FFFFFF << RCC_PLL2CTRL2_PLL2CLKF_SHIFT)
#define RCC_PLL2CTRL2_PLL2CLKF_0                (1 << 0)   /* Bit 0 */
#define RCC_PLL2CTRL2_PLL2CLKF_1                (1 << 1)   /* Bit 1 */
#define RCC_PLL2CTRL2_PLL2CLKF_2                (1 << 2)   /* Bit 2 */
#define RCC_PLL2CTRL2_PLL2CLKF_3                (1 << 3)   /* Bit 3 */
#define RCC_PLL2CTRL2_PLL2CLKF_4                (1 << 4)   /* Bit 4 */
#define RCC_PLL2CTRL2_PLL2CLKF_5                (1 << 5)   /* Bit 5 */
#define RCC_PLL2CTRL2_PLL2CLKF_6                (1 << 6)   /* Bit 6 */
#define RCC_PLL2CTRL2_PLL2CLKF_7                (1 << 7)   /* Bit 7 */
#define RCC_PLL2CTRL2_PLL2CLKF_8                (1 << 8)   /* Bit 8 */
#define RCC_PLL2CTRL2_PLL2CLKF_9                (1 << 9)   /* Bit 9 */
#define RCC_PLL2CTRL2_PLL2CLKF_10               (1 << 10)  /* Bit 10 */
#define RCC_PLL2CTRL2_PLL2CLKF_11               (1 << 11)  /* Bit 11 */
#define RCC_PLL2CTRL2_PLL2CLKF_12               (1 << 12)  /* Bit 12 */
#define RCC_PLL2CTRL2_PLL2CLKF_13               (1 << 13)  /* Bit 13 */
#define RCC_PLL2CTRL2_PLL2CLKF_14               (1 << 14)  /* Bit 14 */
#define RCC_PLL2CTRL2_PLL2CLKF_15               (1 << 15)  /* Bit 15 */
#define RCC_PLL2CTRL2_PLL2CLKF_16               (1 << 16)  /* Bit 16 */
#define RCC_PLL2CTRL2_PLL2CLKF_17               (1 << 17)  /* Bit 17 */
#define RCC_PLL2CTRL2_PLL2CLKF_18               (1 << 18)  /* Bit 18 */
#define RCC_PLL2CTRL2_PLL2CLKF_19               (1 << 19)  /* Bit 19 */
#define RCC_PLL2CTRL2_PLL2CLKF_20               (1 << 20)  /* Bit 20 */
#define RCC_PLL2CTRL2_PLL2CLKF_21               (1 << 21)  /* Bit 21 */
#define RCC_PLL2CTRL2_PLL2CLKF_22               (1 << 22)  /* Bit 22 */
#define RCC_PLL2CTRL2_PLL2CLKF_23               (1 << 23)  /* Bit 23 */
#define RCC_PLL2CTRL2_PLL2CLKF_24               (1 << 24)  /* Bit 24 */
#define RCC_PLL2CTRL2_PLL2CLKF_25               (1 << 25)  /* Bit 25 */

#define RCC_PLL2CTRL2_PLL2CLKF(n)               ((n) << RCC_PLL2CTRL2_PLL2CLKF_SHIFT)

/* Bit definition for RCC_PLL3CTRL1 register ********************************/
#define RCC_PLL3CTRL1_PLL3SRC_SHIFT             (28)       /* Bits 28-29: PLL3 clock source selection */
#define RCC_PLL3CTRL1_PLL3SRC_MASK              (0x3 << RCC_PLL3CTRL1_PLL3SRC_SHIFT)
#define RCC_PLL3CTRL1_PLL3SRC_0                 (1 << 28) /* Bit 28 */
#define RCC_PLL3CTRL1_PLL3SRC_1                 (1 << 29) /* Bit 29 */

#define RCC_PLL3CTRL1_PLL3SRC_HSI               (0x0 << RCC_PLL3CTRL1_PLL3SRC_SHIFT) /* HSI Clock is the PLL source clock */
#define RCC_PLL3CTRL1_PLL3SRC_NONE              (0x1 << RCC_PLL3CTRL1_PLL3SRC_SHIFT) /* No clock */
#define RCC_PLL3CTRL1_PLL3SRC_MSI               (0x2 << RCC_PLL3CTRL1_PLL3SRC_SHIFT) /* MSI Clock is the PLL source clock */
#define RCC_PLL3CTRL1_PLL3SRC_HSE               (0x3 << RCC_PLL3CTRL1_PLL3SRC_SHIFT) /* HSE Clock is the PLL source clock */

#define RCC_PLL3CTRL1_PLL3PHLK                  (1 << 20) /* Bit 20: PLL3 PH lock */
#define RCC_PLL3CTRL1_PLL3LDOEN                 (1 << 19) /* Bit 19: PLL3 LDO enable */
#define RCC_PLL3CTRL1_PLL3EN                    (1 << 18) /* Bit 18: PLL3 enable */
#define RCC_PLL3CTRL1_PLL3RST                   (1 << 17) /* Bit 17: PLL3 reset */
#define RCC_PLL3CTRL1_PLL3PD                    (1 << 16) /* Bit 16: PLL3 power down */

#define RCC_PLL3CTRL1_PLL3BWAJ_SHIFT            (0)       /* Bits 0-11: PLL3 BWAJ value */
#define RCC_PLL3CTRL1_PLL3BWAJ_MASK             (0xFFF << RCC_PLL3CTRL1_PLL3BWAJ_SHIFT)
#define RCC_PLL3CTRL1_PLL3BWAJ_0                (1 << 0)  /* Bit 0 */
#define RCC_PLL3CTRL1_PLL3BWAJ_1                (1 << 1)  /* Bit 1 */
#define RCC_PLL3CTRL1_PLL3BWAJ_2                (1 << 2)  /* Bit 2 */
#define RCC_PLL3CTRL1_PLL3BWAJ_3                (1 << 3)  /* Bit 3 */
#define RCC_PLL3CTRL1_PLL3BWAJ_4                (1 << 4)  /* Bit 4 */
#define RCC_PLL3CTRL1_PLL3BWAJ_5                (1 << 5)  /* Bit 5 */
#define RCC_PLL3CTRL1_PLL3BWAJ_6                (1 << 6)  /* Bit 6 */
#define RCC_PLL3CTRL1_PLL3BWAJ_7                (1 << 7)  /* Bit 7 */
#define RCC_PLL3CTRL1_PLL3BWAJ_8                (1 << 8)  /* Bit 8 */
#define RCC_PLL3CTRL1_PLL3BWAJ_9                (1 << 9)  /* Bit 9 */
#define RCC_PLL3CTRL1_PLL3BWAJ_10               (1 << 10) /* Bit 10 */
#define RCC_PLL3CTRL1_PLL3BWAJ_11               (1 << 11) /* Bit 11 */

#define RCC_PLL3CTRL1_PLL3BWAJ(n)               ((n) << RCC_PLL3CTRL1_PLL3BWAJ_SHIFT)

/* Bit definition for RCC_PLL3CTRL2 register ********************************/
#define RCC_PLL3CTRL2_PLL3CLKR_SHIFT            (26)       /* Bits 26-31: PLL3 clock R divider */
#define RCC_PLL3CTRL2_PLL3CLKR_MASK             (0x3F << RCC_PLL3CTRL2_PLL3CLKR_SHIFT)
#define RCC_PLL3CTRL2_PLL3CLKR_0                (1 << 26) /* Bit 26 */
#define RCC_PLL3CTRL2_PLL3CLKR_1                (1 << 27) /* Bit 27 */
#define RCC_PLL3CTRL2_PLL3CLKR_2                (1 << 28) /* Bit 28 */
#define RCC_PLL3CTRL2_PLL3CLKR_3                (1 << 29) /* Bit 29 */
#define RCC_PLL3CTRL2_PLL3CLKR_4                (1 << 30) /* Bit 30 */
#define RCC_PLL3CTRL2_PLL3CLKR_5                (1 << 31) /* Bit 31 */

#define RCC_PLL3CTRL2_PLL3CLKR(n)               ((n) << RCC_PLL3CTRL2_PLL3CLKR_SHIFT)

#define RCC_PLL3CTRL2_PLL3CLKF_SHIFT            (0)        /* Bits 0-25: PLL3 clock F multiplier */
#define RCC_PLL3CTRL2_PLL3CLKF_MASK             (0x3FFFFFF << RCC_PLL3CTRL2_PLL3CLKF_SHIFT)
#define RCC_PLL3CTRL2_PLL3CLKF_0                (1 << 0)   /* Bit 0 */
#define RCC_PLL3CTRL2_PLL3CLKF_1                (1 << 1)   /* Bit 1 */
#define RCC_PLL3CTRL2_PLL3CLKF_2                (1 << 2)   /* Bit 2 */
#define RCC_PLL3CTRL2_PLL3CLKF_3                (1 << 3)   /* Bit 3 */
#define RCC_PLL3CTRL2_PLL3CLKF_4                (1 << 4)   /* Bit 4 */
#define RCC_PLL3CTRL2_PLL3CLKF_5                (1 << 5)   /* Bit 5 */
#define RCC_PLL3CTRL2_PLL3CLKF_6                (1 << 6)   /* Bit 6 */
#define RCC_PLL3CTRL2_PLL3CLKF_7                (1 << 7)   /* Bit 7 */
#define RCC_PLL3CTRL2_PLL3CLKF_8                (1 << 8)   /* Bit 8 */
#define RCC_PLL3CTRL2_PLL3CLKF_9                (1 << 9)   /* Bit 9 */
#define RCC_PLL3CTRL2_PLL3CLKF_10               (1 << 10)  /* Bit 10 */
#define RCC_PLL3CTRL2_PLL3CLKF_11               (1 << 11)  /* Bit 11 */
#define RCC_PLL3CTRL2_PLL3CLKF_12               (1 << 12)  /* Bit 12 */
#define RCC_PLL3CTRL2_PLL3CLKF_13               (1 << 13)  /* Bit 13 */
#define RCC_PLL3CTRL2_PLL3CLKF_14               (1 << 14)  /* Bit 14 */
#define RCC_PLL3CTRL2_PLL3CLKF_15               (1 << 15)  /* Bit 15 */
#define RCC_PLL3CTRL2_PLL3CLKF_16               (1 << 16)  /* Bit 16 */
#define RCC_PLL3CTRL2_PLL3CLKF_17               (1 << 17)  /* Bit 17 */
#define RCC_PLL3CTRL2_PLL3CLKF_18               (1 << 18)  /* Bit 18 */
#define RCC_PLL3CTRL2_PLL3CLKF_19               (1 << 19)  /* Bit 19 */
#define RCC_PLL3CTRL2_PLL3CLKF_20               (1 << 20)  /* Bit 20 */
#define RCC_PLL3CTRL2_PLL3CLKF_21               (1 << 21)  /* Bit 21 */
#define RCC_PLL3CTRL2_PLL3CLKF_22               (1 << 22)  /* Bit 22 */
#define RCC_PLL3CTRL2_PLL3CLKF_23               (1 << 23)  /* Bit 23 */
#define RCC_PLL3CTRL2_PLL3CLKF_24               (1 << 24)  /* Bit 24 */
#define RCC_PLL3CTRL2_PLL3CLKF_25               (1 << 25)  /* Bit 25 */

#define RCC_PLL3CTRL2_PLL3CLKF(n)               ((n) << RCC_PLL3CTRL2_PLL3CLKF_SHIFT)

/* Bit definition for RCC_SHRPLLCTRL1 register ******************************/

/* Bits 29:28 SHRPLL clock source selection */
#define RCC_SHRPLLCTRL1_SHRPLLSRC_SHIFT        (28)
#define RCC_SHRPLLCTRL1_SHRPLLSRC_MASK         (0x3 << RCC_SHRPLLCTRL1_SHRPLLSRC_SHIFT)
#define RCC_SHRPLLCTRL1_SHRPLLSRC_0            (0x1 << 28) /* Bit 28 */
#define RCC_SHRPLLCTRL1_SHRPLLSRC_1            (0x1 << 29) /* Bit 29 */

#define RCC_SHRPLLCTRL1_SHRPLLSRC_HSI               (0x0 << RCC_SHRPLLCTRL1_SHRPLLSRC_SHIFT) /* HSI Clock is the PLL source clock */
#define RCC_SHRPLLCTRL1_SHRPLLSRC_NONE              (0x1 << RCC_SHRPLLCTRL1_SHRPLLSRC_SHIFT) /* No clock */
#define RCC_SHRPLLCTRL1_SHRPLLSRC_MSI               (0x2 << RCC_SHRPLLCTRL1_SHRPLLSRC_SHIFT) /* MSI Clock is the PLL source clock */
#define RCC_SHRPLLCTRL1_SHRPLLSRC_HSE               (0x3 << RCC_SHRPLLCTRL1_SHRPLLSRC_SHIFT) /* HSE Clock is the PLL source clock */

/* Bit 20 SHRPLL PH lock */
#define RCC_SHRPLLCTRL1_SHRPLLPHLK             (0x1 << 20)

/* Bit 19 SHRPLL LDO enable */
#define RCC_SHRPLLCTRL1_SHRPLLLDOEN            (0x1 << 19)

/* Bit 18 SHRPLL enable */
#define RCC_SHRPLLCTRL1_SHRPLLEN               (0x1 << 18)

/* Bit 17 SHRPLL reset */
#define RCC_SHRPLLCTRL1_SHRPLLRST              (0x1 << 17)

/* Bit 16 SHRPLL power down */
#define RCC_SHRPLLCTRL1_SHRPLLPD               (0x1 << 16)

/* Bits 11:0 SHRPLL bandwidth adjustment */
#define RCC_SHRPLLCTRL1_SHRPLLBWAJ_SHIFT       (0)
#define RCC_SHRPLLCTRL1_SHRPLLBWAJ_MASK        (0xFFF << RCC_SHRPLLCTRL1_SHRPLLBWAJ_SHIFT)
#define RCC_SHRPLLCTRL1_SHRPLLBWAJ_0           (0x1 << 0)
#define RCC_SHRPLLCTRL1_SHRPLLBWAJ_1           (0x1 << 1)
#define RCC_SHRPLLCTRL1_SHRPLLBWAJ_2           (0x1 << 2)
#define RCC_SHRPLLCTRL1_SHRPLLBWAJ_3           (0x1 << 3)
#define RCC_SHRPLLCTRL1_SHRPLLBWAJ_4           (0x1 << 4)
#define RCC_SHRPLLCTRL1_SHRPLLBWAJ_5           (0x1 << 5)
#define RCC_SHRPLLCTRL1_SHRPLLBWAJ_6           (0x1 << 6)
#define RCC_SHRPLLCTRL1_SHRPLLBWAJ_7           (0x1 << 7)
#define RCC_SHRPLLCTRL1_SHRPLLBWAJ_8           (0x1 << 8)
#define RCC_SHRPLLCTRL1_SHRPLLBWAJ_9           (0x1 << 9)
#define RCC_SHRPLLCTRL1_SHRPLLBWAJ_10          (0x1 << 10)
#define RCC_SHRPLLCTRL1_SHRPLLBWAJ_11          (0x1 << 11)

#define RCC_SHRPLLCTRL1_SHRPLLBWAJ(n)          ((n) << RCC_SHRPLLCTRL1_SHRPLLBWAJ_SHIFT)

/* Bit definition for RCC_SHRPLLCTRL2 register ******************************/

/* Bits 31:26 SHRPLL clock R divider */
#define RCC_SHRPLLCTRL2_SHRPLLCLKR_SHIFT       (26)
#define RCC_SHRPLLCTRL2_SHRPLLCLKR_MASK        (0x3F << RCC_SHRPLLCTRL2_SHRPLLCLKR_SHIFT)
#define RCC_SHRPLLCTRL2_SHRPLLCLKR_0           (0x1 << 26)
#define RCC_SHRPLLCTRL2_SHRPLLCLKR_1           (0x1 << 27)
#define RCC_SHRPLLCTRL2_SHRPLLCLKR_2           (0x1 << 28)
#define RCC_SHRPLLCTRL2_SHRPLLCLKR_3           (0x1 << 29)
#define RCC_SHRPLLCTRL2_SHRPLLCLKR_4           (0x1 << 30)
#define RCC_SHRPLLCTRL2_SHRPLLCLKR_5           (0x1 << 31)

#define RCC_SHRPLLCTRL2_SHRPLLCLKR(n)          ((n) << RCC_SHRPLLCTRL2_SHRPLLCLKR_SHIFT)

/* Bits 25:0 SHRPLL clock F multiplier */
#define RCC_SHRPLLCTRL2_SHRPLLCLKF_SHIFT       (0)
#define RCC_SHRPLLCTRL2_SHRPLLCLKF_MASK        (0x3FFFFFF << RCC_SHRPLLCTRL2_SHRPLLCLKF_SHIFT)
#define RCC_SHRPLLCTRL2_SHRPLLCLKF_0           (0x1 << 0)
#define RCC_SHRPLLCTRL2_SHRPLLCLKF_1           (0x1 << 1)
#define RCC_SHRPLLCTRL2_SHRPLLCLKF_2           (0x1 << 2)
#define RCC_SHRPLLCTRL2_SHRPLLCLKF_3           (0x1 << 3)
#define RCC_SHRPLLCTRL2_SHRPLLCLKF_4           (0x1 << 4)
#define RCC_SHRPLLCTRL2_SHRPLLCLKF_5           (0x1 << 5)
#define RCC_SHRPLLCTRL2_SHRPLLCLKF_6           (0x1 << 6)
#define RCC_SHRPLLCTRL2_SHRPLLCLKF_7           (0x1 << 7)
#define RCC_SHRPLLCTRL2_SHRPLLCLKF_8           (0x1 << 8)
#define RCC_SHRPLLCTRL2_SHRPLLCLKF_9           (0x1 << 9)
#define RCC_SHRPLLCTRL2_SHRPLLCLKF_10          (0x1 << 10)
#define RCC_SHRPLLCTRL2_SHRPLLCLKF_11          (0x1 << 11)
#define RCC_SHRPLLCTRL2_SHRPLLCLKF_12          (0x1 << 12)
#define RCC_SHRPLLCTRL2_SHRPLLCLKF_13          (0x1 << 13)
#define RCC_SHRPLLCTRL2_SHRPLLCLKF_14          (0x1 << 14)
#define RCC_SHRPLLCTRL2_SHRPLLCLKF_15          (0x1 << 15)
#define RCC_SHRPLLCTRL2_SHRPLLCLKF_16          (0x1 << 16)
#define RCC_SHRPLLCTRL2_SHRPLLCLKF_17          (0x1 << 17)
#define RCC_SHRPLLCTRL2_SHRPLLCLKF_18          (0x1 << 18)
#define RCC_SHRPLLCTRL2_SHRPLLCLKF_19          (0x1 << 19)
#define RCC_SHRPLLCTRL2_SHRPLLCLKF_20          (0x1 << 20)
#define RCC_SHRPLLCTRL2_SHRPLLCLKF_21          (0x1 << 21)
#define RCC_SHRPLLCTRL2_SHRPLLCLKF_22          (0x1 << 22)
#define RCC_SHRPLLCTRL2_SHRPLLCLKF_23          (0x1 << 23)
#define RCC_SHRPLLCTRL2_SHRPLLCLKF_24          (0x1 << 24)
#define RCC_SHRPLLCTRL2_SHRPLLCLKF_25          (0x1 << 25)

#define RCC_SHRPLLCTRL2_SHRPLLCLKF(n)          ((n) << RCC_SHRPLLCTRL2_SHRPLLCLKF_SHIFT)

/* Bit definition for RCC_SRCCTRL1 register *********************************/
#define RCC_SRCCTRL1_AFEMSIRDF                  (1 << 31) /* Bit 31: AFE MSI ready flag */
#define RCC_SRCCTRL1_AFEHSIRDF                  (1 << 30) /* Bit 30: AFE HSI ready flag */

#define RCC_SRCCTRL1_SCLKSTS_SHIFT              (26)      /* Bits 26-27: System clock status */
#define RCC_SRCCTRL1_SCLKSTS_MASK               (0x3 << RCC_SRCCTRL1_SCLKSTS_SHIFT)
#define RCC_SRCCTRL1_SCLKSTS_HSI                (0x0 << RCC_SRCCTRL1_SCLKSTS_SHIFT)
#define RCC_SRCCTRL1_SCLKSTS_MSI                (0x1 << RCC_SRCCTRL1_SCLKSTS_SHIFT)
#define RCC_SRCCTRL1_SCLKSTS_HSE                (0x2 << RCC_SRCCTRL1_SCLKSTS_SHIFT)
#define RCC_SRCCTRL1_SCLKSTS_PLL1A              (0x3 << RCC_SRCCTRL1_SCLKSTS_SHIFT)

#define RCC_SRCCTRL1_SCLKSW_SHIFT               (24)      /* Bits 24-25: System clock switch */
#define RCC_SRCCTRL1_SCLKSW_MASK                (0x3 << RCC_SRCCTRL1_SCLKSW_SHIFT)
#define RCC_SRCCTRL1_SCLKSW_HSI                 (0x0 << RCC_SRCCTRL1_SCLKSW_SHIFT)
#define RCC_SRCCTRL1_SCLKSW_MSI                 (0x1 << RCC_SRCCTRL1_SCLKSW_SHIFT)
#define RCC_SRCCTRL1_SCLKSW_HSE                 (0x2 << RCC_SRCCTRL1_SCLKSW_SHIFT)
#define RCC_SRCCTRL1_SCLKSW_PLL1A               (0x3 << RCC_SRCCTRL1_SCLKSW_SHIFT)

#define RCC_SRCCTRL1_HSERDCNTEN                 (1 << 8)  /* Bit 8: HSE ready counter enable */
#define RCC_SRCCTRL1_MSIRDF                     (1 << 7)  /* Bit 7: MSI ready flag */
#define RCC_SRCCTRL1_MSIEN                      (1 << 6)  /* Bit 6: MSI enable */
#define RCC_SRCCTRL1_HSECSSEN                   (1 << 5)  /* Bit 5: HSE clock security system enable */
#define RCC_SRCCTRL1_HSEBP                      (1 << 4)  /* Bit 4: HSE bypass */
#define RCC_SRCCTRL1_HSERDF                     (1 << 3)  /* Bit 3: HSE ready flag */
#define RCC_SRCCTRL1_HSEEN                      (1 << 2)  /* Bit 2: HSE enable */
#define RCC_SRCCTRL1_HSIRDF                     (1 << 1)  /* Bit 1: HSI ready flag */
#define RCC_SRCCTRL1_HSIEN                      (1 << 0)  /* Bit 0: HSI enable */

/* Bit definition for RCC_SRCCTRL2 register *********************************/
#define RCC_SRCCTRL2_BORF                  (0x1 << 30) /* Bit 30: Brown-out reset flag */
#define RCC_SRCCTRL2_HSICALEF              (0x1 << 29) /* Bit 29: HSI calibration error flag */
#define RCC_SRCCTRL2_MSICALEF              (0x1 << 28) /* Bit 28: MSI calibration error flag */

#define RCC_SRCCTRL2_M7HYPSEL_SHIFT        (17)
#define RCC_SRCCTRL2_M7HYPSEL_MASK         (0x1 << RCC_SRCCTRL2_M7HYPSEL_SHIFT) /* Bit 17: M7 hypervisor clock source selection */
#define RCC_SRCCTRL2_M7HYPSEL_PLL1A        (0x0 << RCC_SRCCTRL2_M7HYPSEL_SHIFT) /* Use PLL1A as M7 hypervisor clock source */
#define RCC_SRCCTRL2_M7HYPSEL_PLL2A        (0x1 << RCC_SRCCTRL2_M7HYPSEL_SHIFT) /* Use PLL2A as M7 hypervisor clock source */

#define RCC_SRCCTRL2_AXIHYPSEL_SHIFT       (16)
#define RCC_SRCCTRL2_AXIHYPSEL_MASK        (0x1 << RCC_SRCCTRL2_AXIHYPSEL_SHIFT) /* Bit 16: AXI hypervisor clock source selection */
#define RCC_SRCCTRL2_AXIHYPSEL_PLL1A       (0x0 << RCC_SRCCTRL2_AXIHYPSEL_SHIFT) /* Use PLL1A as AXI hypervisor clock source */
#define RCC_SRCCTRL2_AXIHYPSEL_PLL2A       (0x1 << RCC_SRCCTRL2_AXIHYPSEL_SHIFT) /* Use PLL2A as AXI hypervisor clock source */

/* Bits 12:8 MSI calibration value */
#define RCC_SRCCTRL2_MSICAL_SHIFT          (8)
#define RCC_SRCCTRL2_MSICAL_MASK           (0x1F << RCC_SRCCTRL2_MSICAL_SHIFT)
#define RCC_SRCCTRL2_MSICAL_0              (0x1 << (RCC_SRCCTRL2_MSICAL_SHIFT + 0)) /* Bit 8 */
#define RCC_SRCCTRL2_MSICAL_1              (0x1 << (RCC_SRCCTRL2_MSICAL_SHIFT + 1)) /* Bit 9 */
#define RCC_SRCCTRL2_MSICAL_2              (0x1 << (RCC_SRCCTRL2_MSICAL_SHIFT + 2)) /* Bit 10 */
#define RCC_SRCCTRL2_MSICAL_3              (0x1 << (RCC_SRCCTRL2_MSICAL_SHIFT + 3)) /* Bit 11 */
#define RCC_SRCCTRL2_MSICAL_4              (0x1 << (RCC_SRCCTRL2_MSICAL_SHIFT + 4)) /* Bit 12 */

/* Bits 4:0 MSI trim value */
#define RCC_SRCCTRL2_MSITRIM_SHIFT         (0)
#define RCC_SRCCTRL2_MSITRIM_MASK          (0x1F << RCC_SRCCTRL2_MSITRIM_SHIFT)
#define RCC_SRCCTRL2_MSITRIM_0             (0x1 << (RCC_SRCCTRL2_MSITRIM_SHIFT + 0)) /* Bit 0 */
#define RCC_SRCCTRL2_MSITRIM_1             (0x1 << (RCC_SRCCTRL2_MSITRIM_SHIFT + 1)) /* Bit 1 */
#define RCC_SRCCTRL2_MSITRIM_2             (0x1 << (RCC_SRCCTRL2_MSITRIM_SHIFT + 2)) /* Bit 2 */
#define RCC_SRCCTRL2_MSITRIM_3             (0x1 << (RCC_SRCCTRL2_MSITRIM_SHIFT + 3)) /* Bit 3 */
#define RCC_SRCCTRL2_MSITRIM_4             (0x1 << (RCC_SRCCTRL2_MSITRIM_SHIFT + 4)) /* Bit 4 */

/* Bit definition for RCC_SRCCTRL3 register *********************************/

/* Bits 24:16 HSI calibration value */
#define RCC_SRCCTRL3_HSICAL_SHIFT          (16)
#define RCC_SRCCTRL3_HSICAL_MASK           (0x1FF << RCC_SRCCTRL3_HSICAL_SHIFT)
#define RCC_SRCCTRL3_HSICAL_0              (0x1 << (RCC_SRCCTRL3_HSICAL_SHIFT + 0)) /* Bit 16 */
#define RCC_SRCCTRL3_HSICAL_1              (0x1 << (RCC_SRCCTRL3_HSICAL_SHIFT + 1)) /* Bit 17 */
#define RCC_SRCCTRL3_HSICAL_2              (0x1 << (RCC_SRCCTRL3_HSICAL_SHIFT + 2)) /* Bit 18 */
#define RCC_SRCCTRL3_HSICAL_3              (0x1 << (RCC_SRCCTRL3_HSICAL_SHIFT + 3)) /* Bit 19 */
#define RCC_SRCCTRL3_HSICAL_4              (0x1 << (RCC_SRCCTRL3_HSICAL_SHIFT + 4)) /* Bit 20 */
#define RCC_SRCCTRL3_HSICAL_5              (0x1 << (RCC_SRCCTRL3_HSICAL_SHIFT + 5)) /* Bit 21 */
#define RCC_SRCCTRL3_HSICAL_6              (0x1 << (RCC_SRCCTRL3_HSICAL_SHIFT + 6)) /* Bit 22 */
#define RCC_SRCCTRL3_HSICAL_7              (0x1 << (RCC_SRCCTRL3_HSICAL_SHIFT + 7)) /* Bit 23 */
#define RCC_SRCCTRL3_HSICAL_8              (0x1 << (RCC_SRCCTRL3_HSICAL_SHIFT + 8)) /* Bit 24 */

/* Bits 8:0 HSI trim value */
#define RCC_SRCCTRL3_HSITRIM_SHIFT         (0)
#define RCC_SRCCTRL3_HSITRIM_MASK          (0x1FF << RCC_SRCCTRL3_HSITRIM_SHIFT)
#define RCC_SRCCTRL3_HSITRIM_0             (0x1 << (RCC_SRCCTRL3_HSITRIM_SHIFT + 0)) /* Bit 0 */
#define RCC_SRCCTRL3_HSITRIM_1             (0x1 << (RCC_SRCCTRL3_HSITRIM_SHIFT + 1)) /* Bit 1 */
#define RCC_SRCCTRL3_HSITRIM_2             (0x1 << (RCC_SRCCTRL3_HSITRIM_SHIFT + 2)) /* Bit 2 */
#define RCC_SRCCTRL3_HSITRIM_3             (0x1 << (RCC_SRCCTRL3_HSITRIM_SHIFT + 3)) /* Bit 3 */
#define RCC_SRCCTRL3_HSITRIM_4             (0x1 << (RCC_SRCCTRL3_HSITRIM_SHIFT + 4)) /* Bit 4 */
#define RCC_SRCCTRL3_HSITRIM_5             (0x1 << (RCC_SRCCTRL3_HSITRIM_SHIFT + 5)) /* Bit 5 */
#define RCC_SRCCTRL3_HSITRIM_6             (0x1 << (RCC_SRCCTRL3_HSITRIM_SHIFT + 6)) /* Bit 6 */
#define RCC_SRCCTRL3_HSITRIM_7             (0x1 << (RCC_SRCCTRL3_HSITRIM_SHIFT + 7)) /* Bit 7 */
#define RCC_SRCCTRL3_HSITRIM_8             (0x1 << (RCC_SRCCTRL3_HSITRIM_SHIFT + 8)) /* Bit 8 */

/* Bit definition for RCC_PLL1DIV register **********************************/

/* Bit[21:16] PLL1CDIV */
#define RCC_PLL1DIV_PLL1CDIV_SHIFT          (16)
#define RCC_PLL1DIV_PLL1CDIV_MASK           (0x3F << RCC_PLL1DIV_PLL1CDIV_SHIFT)
#define RCC_PLL1DIV_PLL1CDIV_0              (0x1 << (RCC_PLL1DIV_PLL1CDIV_SHIFT + 0)) /* Bit16 */

/* Bit definition for RCC_HSECAL register ***********************************/

/* [31:18] Reserved */

/* [17] HSE Calibration Count Enable */
#define RCC_PLL1DIV_PLL1CDIV_1              (0x1 << (RCC_PLL1DIV_PLL1CDIV_SHIFT + 1)) /* Bit17 */
#define RCC_PLL1DIV_PLL1CDIV_2              (0x1 << (RCC_PLL1DIV_PLL1CDIV_SHIFT + 2)) /* Bit18 */
#define RCC_PLL1DIV_PLL1CDIV_3              (0x1 << (RCC_PLL1DIV_PLL1CDIV_SHIFT + 3)) /* Bit19 */
#define RCC_PLL1DIV_PLL1CDIV_4              (0x1 << (RCC_PLL1DIV_PLL1CDIV_SHIFT + 4)) /* Bit20 */
#define RCC_PLL1DIV_PLL1CDIV_5              (0x1 << (RCC_PLL1DIV_PLL1CDIV_SHIFT + 5)) /* Bit21 */

#define RCC_PLL1DIV_PLL1CDIV_DIV(n)         ((n) << RCC_PLL1DIV_PLL1CDIV_SHIFT)

/* Bit[13:8] PLL1BDIV */
#define RCC_PLL1DIV_PLL1BDIV_SHIFT          (8)
#define RCC_PLL1DIV_PLL1BDIV_MASK           (0x3F << RCC_PLL1DIV_PLL1BDIV_SHIFT)
#define RCC_PLL1DIV_PLL1BDIV_0              (0x1 << (RCC_PLL1DIV_PLL1BDIV_SHIFT + 0)) /* Bit8 */
#define RCC_PLL1DIV_PLL1BDIV_1              (0x1 << (RCC_PLL1DIV_PLL1BDIV_SHIFT + 1)) /* Bit9 */
#define RCC_PLL1DIV_PLL1BDIV_2              (0x1 << (RCC_PLL1DIV_PLL1BDIV_SHIFT + 2)) /* Bit10 */
#define RCC_PLL1DIV_PLL1BDIV_3              (0x1 << (RCC_PLL1DIV_PLL1BDIV_SHIFT + 3)) /* Bit11 */
#define RCC_PLL1DIV_PLL1BDIV_4              (0x1 << (RCC_PLL1DIV_PLL1BDIV_SHIFT + 4)) /* Bit12 */
#define RCC_PLL1DIV_PLL1BDIV_5              (0x1 << (RCC_PLL1DIV_PLL1BDIV_SHIFT + 5)) /* Bit13 */

#define RCC_PLL1DIV_PLL1BDIV_DIV(n)         ((n) << RCC_PLL1DIV_PLL1BDIV_SHIFT)

/* Bit definition for RCC_PLL1DIV register **********************************/

/* Bit[5:0] PLL1ADIV */
#define RCC_PLL1DIV_PLL1ADIV_SHIFT          (0)
#define RCC_PLL1DIV_PLL1ADIV_MASK           (0x3F << RCC_PLL1DIV_PLL1ADIV_SHIFT)
#define RCC_PLL1DIV_PLL1ADIV_0              (0x1 << (RCC_PLL1DIV_PLL1ADIV_SHIFT + 0)) /* Bit0 */
#define RCC_PLL1DIV_PLL1ADIV_1              (0x1 << (RCC_PLL1DIV_PLL1ADIV_SHIFT + 1)) /* Bit1 */
#define RCC_PLL1DIV_PLL1ADIV_2              (0x1 << (RCC_PLL1DIV_PLL1ADIV_SHIFT + 2)) /* Bit2 */
#define RCC_PLL1DIV_PLL1ADIV_3              (0x1 << (RCC_PLL1DIV_PLL1ADIV_SHIFT + 3)) /* Bit3 */
#define RCC_PLL1DIV_PLL1ADIV_4              (0x1 << (RCC_PLL1DIV_PLL1ADIV_SHIFT + 4)) /* Bit4 */
#define RCC_PLL1DIV_PLL1ADIV_5              (0x1 << (RCC_PLL1DIV_PLL1ADIV_SHIFT + 5)) /* Bit5 */

#define RCC_PLL1DIV_PLL1ADIV_DIV(n)         ((n) << RCC_PLL1DIV_PLL1ADIV_SHIFT)

/* Bit definition for RCC_PLL2DIV register **********************************/

/* Bit[21:16] PLL2CDIV */
#define RCC_PLL2DIV_PLL2CDIV_SHIFT          (16)
#define RCC_PLL2DIV_PLL2CDIV_MASK           (0x3F << RCC_PLL2DIV_PLL2CDIV_SHIFT)
#define RCC_PLL2DIV_PLL2CDIV_0              (0x1 << (RCC_PLL2DIV_PLL2CDIV_SHIFT + 0)) /* Bit16 */
#define RCC_PLL2DIV_PLL2CDIV_1              (0x1 << (RCC_PLL2DIV_PLL2CDIV_SHIFT + 1)) /* Bit17 */
#define RCC_PLL2DIV_PLL2CDIV_2              (0x1 << (RCC_PLL2DIV_PLL2CDIV_SHIFT + 2)) /* Bit18 */
#define RCC_PLL2DIV_PLL2CDIV_3              (0x1 << (RCC_PLL2DIV_PLL2CDIV_SHIFT + 3)) /* Bit19 */
#define RCC_PLL2DIV_PLL2CDIV_4              (0x1 << (RCC_PLL2DIV_PLL2CDIV_SHIFT + 4)) /* Bit20 */
#define RCC_PLL2DIV_PLL2CDIV_5              (0x1 << (RCC_PLL2DIV_PLL2CDIV_SHIFT + 5)) /* Bit21 */

#define RCC_PLL2DIV_PLL2CDIV_DIV(n)         ((n) << RCC_PLL2DIV_PLL2CDIV_SHIFT)

/* Bit[13:8] PLL2BDIV */
#define RCC_PLL2DIV_PLL2BDIV_SHIFT          (8)
#define RCC_PLL2DIV_PLL2BDIV_MASK           (0x3F << RCC_PLL2DIV_PLL2BDIV_SHIFT)
#define RCC_PLL2DIV_PLL2BDIV_0              (0x1 << (RCC_PLL2DIV_PLL2BDIV_SHIFT + 0)) /* Bit8 */
#define RCC_PLL2DIV_PLL2BDIV_1              (0x1 << (RCC_PLL2DIV_PLL2BDIV_SHIFT + 1)) /* Bit9 */
#define RCC_PLL2DIV_PLL2BDIV_2              (0x1 << (RCC_PLL2DIV_PLL2BDIV_SHIFT + 2)) /* Bit10 */
#define RCC_PLL2DIV_PLL2BDIV_3              (0x1 << (RCC_PLL2DIV_PLL2BDIV_SHIFT + 3)) /* Bit11 */
#define RCC_PLL2DIV_PLL2BDIV_4              (0x1 << (RCC_PLL2DIV_PLL2BDIV_SHIFT + 4)) /* Bit12 */
#define RCC_PLL2DIV_PLL2BDIV_5              (0x1 << (RCC_PLL2DIV_PLL2BDIV_SHIFT + 5)) /* Bit13 */

#define RCC_PLL2DIV_PLL2BDIV_DIV(n)         ((n) << RCC_PLL2DIV_PLL2BDIV_SHIFT)

/* Bit definition for RCC_PLL2DIV register **********************************/
#define RCC_PLL2DIV_PLL2ADIV_SHIFT          (0)
#define RCC_PLL2DIV_PLL2ADIV_MASK           (0x3F << RCC_PLL2DIV_PLL2ADIV_SHIFT)
#define RCC_PLL2DIV_PLL2ADIV_0              (0x1 << (RCC_PLL2DIV_PLL2ADIV_SHIFT + 0)) /* Bit0 */
#define RCC_PLL2DIV_PLL2ADIV_1              (0x1 << (RCC_PLL2DIV_PLL2ADIV_SHIFT + 1)) /* Bit1 */
#define RCC_PLL2DIV_PLL2ADIV_2              (0x1 << (RCC_PLL2DIV_PLL2ADIV_SHIFT + 2)) /* Bit2 */
#define RCC_PLL2DIV_PLL2ADIV_3              (0x1 << (RCC_PLL2DIV_PLL2ADIV_SHIFT + 3)) /* Bit3 */
#define RCC_PLL2DIV_PLL2ADIV_4              (0x1 << (RCC_PLL2DIV_PLL2ADIV_SHIFT + 4)) /* Bit4 */
#define RCC_PLL2DIV_PLL2ADIV_5              (0x1 << (RCC_PLL2DIV_PLL2ADIV_SHIFT + 5)) /* Bit5 */

#define RCC_PLL2DIV_PLL2ADIV_DIV(n)         ((n) << RCC_PLL2DIV_PLL2ADIV_SHIFT)

/* Bit definition for RCC_PLL3DIV register **********************************/

/* Bit[21:16] PLL3CDIV */
#define RCC_PLL3DIV_PLL3CDIV_SHIFT          (16)
#define RCC_PLL3DIV_PLL3CDIV_MASK           (0x3F << RCC_PLL3DIV_PLL3CDIV_SHIFT)
#define RCC_PLL3DIV_PLL3CDIV_0              (0x1 << (RCC_PLL3DIV_PLL3CDIV_SHIFT + 0)) /* Bit16 */
#define RCC_PLL3DIV_PLL3CDIV_1              (0x1 << (RCC_PLL3DIV_PLL3CDIV_SHIFT + 1)) /* Bit17 */
#define RCC_PLL3DIV_PLL3CDIV_2              (0x1 << (RCC_PLL3DIV_PLL3CDIV_SHIFT + 2)) /* Bit18 */
#define RCC_PLL3DIV_PLL3CDIV_3              (0x1 << (RCC_PLL3DIV_PLL3CDIV_SHIFT + 3)) /* Bit19 */
#define RCC_PLL3DIV_PLL3CDIV_4              (0x1 << (RCC_PLL3DIV_PLL3CDIV_SHIFT + 4)) /* Bit20 */
#define RCC_PLL3DIV_PLL3CDIV_5              (0x1 << (RCC_PLL3DIV_PLL3CDIV_SHIFT + 5)) /* Bit21 */

#define RCC_PLL3DIV_PLL3CDIV_DIV(n)         ((n) << RCC_PLL3DIV_PLL3CDIV_SHIFT)

/* Bit[13:8] PLL3BDIV */
#define RCC_PLL3DIV_PLL3BDIV_SHIFT          (8)
#define RCC_PLL3DIV_PLL3BDIV_MASK           (0x3F << RCC_PLL3DIV_PLL3BDIV_SHIFT)
#define RCC_PLL3DIV_PLL3BDIV_0              (0x1 << (RCC_PLL3DIV_PLL3BDIV_SHIFT + 0)) /* Bit8 */
#define RCC_PLL3DIV_PLL3BDIV_1              (0x1 << (RCC_PLL3DIV_PLL3BDIV_SHIFT + 1)) /* Bit9 */
#define RCC_PLL3DIV_PLL3BDIV_2              (0x1 << (RCC_PLL3DIV_PLL3BDIV_SHIFT + 2)) /* Bit10 */
#define RCC_PLL3DIV_PLL3BDIV_3              (0x1 << (RCC_PLL3DIV_PLL3BDIV_SHIFT + 3)) /* Bit11 */
#define RCC_PLL3DIV_PLL3BDIV_4              (0x1 << (RCC_PLL3DIV_PLL3BDIV_SHIFT + 4)) /* Bit12 */
#define RCC_PLL3DIV_PLL3BDIV_5              (0x1 << (RCC_PLL3DIV_PLL3BDIV_SHIFT + 5)) /* Bit13 */

#define RCC_PLL3DIV_PLL3BDIV_DIV(n)         ((n) << RCC_PLL3DIV_PLL3BDIV_SHIFT)

/* Bit definition for RCC_PLL3DIV register **********************************/
#define RCC_PLL3DIV_PLL3ADIV_SHIFT          (0)
#define RCC_PLL3DIV_PLL3ADIV_MASK           (0x3F << RCC_PLL3DIV_PLL3ADIV_SHIFT)
#define RCC_PLL3DIV_PLL3ADIV_0              (0x1 << (RCC_PLL3DIV_PLL3ADIV_SHIFT + 0)) /* Bit0 */
#define RCC_PLL3DIV_PLL3ADIV_1              (0x1 << (RCC_PLL3DIV_PLL3ADIV_SHIFT + 1)) /* Bit1 */
#define RCC_PLL3DIV_PLL3ADIV_2              (0x1 << (RCC_PLL3DIV_PLL3ADIV_SHIFT + 2)) /* Bit2 */
#define RCC_PLL3DIV_PLL3ADIV_3              (0x1 << (RCC_PLL3DIV_PLL3ADIV_SHIFT + 3)) /* Bit3 */
#define RCC_PLL3DIV_PLL3ADIV_4              (0x1 << (RCC_PLL3DIV_PLL3ADIV_SHIFT + 4)) /* Bit4 */
#define RCC_PLL3DIV_PLL3ADIV_5              (0x1 << (RCC_PLL3DIV_PLL3ADIV_SHIFT + 5)) /* Bit5 */

#define RCC_PLL3DIV_PLL3ADIV_DIV(n)         ((n) << RCC_PLL3DIV_PLL3ADIV_SHIFT)

/* Bit definition for RCC_SYSBUSDIV1 register *******************************/

/* Bit[27:24] AXIHYPDIV */
#define RCC_SYSBUSDIV1_AXIHYPDIV_SHIFT      (24)
#define RCC_SYSBUSDIV1_AXIHYPDIV_MASK       (0x0F << RCC_SYSBUSDIV1_AXIHYPDIV_SHIFT)
#define RCC_SYSBUSDIV1_AXIHYPDIV_0          (0x01 << RCC_SYSBUSDIV1_AXIHYPDIV_SHIFT) /* Bit24 */
#define RCC_SYSBUSDIV1_AXIHYPDIV_1          (0x02 << RCC_SYSBUSDIV1_AXIHYPDIV_SHIFT) /* Bit25 */
#define RCC_SYSBUSDIV1_AXIHYPDIV_2          (0x04 << RCC_SYSBUSDIV1_AXIHYPDIV_SHIFT) /* Bit26 */
#define RCC_SYSBUSDIV1_AXIHYPDIV_3          (0x08 << RCC_SYSBUSDIV1_AXIHYPDIV_SHIFT) /* Bit27 */
#define RCC_SYSBUSDIV1_AXIHYPDIV_DIV(n) \
    ( (n) == 1   ? 0x00000000U << 24 : \
      (n) == 2   ? 0x00000001U << 24 : \
      (n) == 4   ? 0x00000002U << 24 : \
      (n) == 8   ? 0x00000004U << 24 : \
      (n) == 16  ? 0x00000007U << 24 : \
      (n) == 32  ? 0x00000008U << 24 : \
      (n) == 64  ? 0x00000009U << 24 : \
      (n) == 128 ? 0x0000000AU << 24 : \
      (n) == 256 ? 0x0000000BU << 24 : \
      (n) == 512 ? 0x0000000CU << 24 : \
      0x00000000U << 24 )

/* Bit[23:20] HSIDIV */
#define RCC_SYSBUSDIV1_HSIDIV_SHIFT         (20)
#define RCC_SYSBUSDIV1_HSIDIV_MASK          (0x0F << RCC_SYSBUSDIV1_HSIDIV_SHIFT)
#define RCC_SYSBUSDIV1_HSIDIV_0             (0x01 << RCC_SYSBUSDIV1_HSIDIV_SHIFT) /* Bit20 */
#define RCC_SYSBUSDIV1_HSIDIV_1             (0x02 << RCC_SYSBUSDIV1_HSIDIV_SHIFT) /* Bit21 */
#define RCC_SYSBUSDIV1_HSIDIV_2             (0x04 << RCC_SYSBUSDIV1_HSIDIV_SHIFT) /* Bit22 */
#define RCC_SYSBUSDIV1_HSIDIV_3             (0x08 << RCC_SYSBUSDIV1_HSIDIV_SHIFT) /* Bit23 */
#define RCC_SYSBUSDIV1_HSIDIV_DIV(n) \
    ( (n) == 1   ? 0x00000000U << 20 : \
      (n) == 2   ? 0x00000001U << 20 : \
      (n) == 4   ? 0x00000002U << 20 : \
      (n) == 8   ? 0x00000004U << 20 : \
      (n) == 16  ? 0x00000007U << 20 : \
      (n) == 32  ? 0x00000008U << 20 : \
      (n) == 64  ? 0x00000009U << 20 : \
      (n) == 128 ? 0x0000000AU << 20 : \
      (n) == 256 ? 0x0000000BU << 20 : \
      (n) == 512 ? 0x0000000CU << 20 : \
      0x00000000U << 20 )

/* Bit[19:16] M7HYPDIV */
#define RCC_SYSBUSDIV1_M7HYPDIV_SHIFT       (16)
#define RCC_SYSBUSDIV1_M7HYPDIV_MASK        (0x0F << RCC_SYSBUSDIV1_M7HYPDIV_SHIFT)
#define RCC_SYSBUSDIV1_M7HYPDIV_0           (0x01 << RCC_SYSBUSDIV1_M7HYPDIV_SHIFT) /* Bit16 */
#define RCC_SYSBUSDIV1_M7HYPDIV_1           (0x02 << RCC_SYSBUSDIV1_M7HYPDIV_SHIFT) /* Bit17 */
#define RCC_SYSBUSDIV1_M7HYPDIV_2           (0x04 << RCC_SYSBUSDIV1_M7HYPDIV_SHIFT) /* Bit18 */
#define RCC_SYSBUSDIV1_M7HYPDIV_3           (0x08 << RCC_SYSBUSDIV1_M7HYPDIV_SHIFT) /* Bit19 */
#define RCC_SYSBUSDIV1_M7HYPDIV_DIV(n) \
    ( (n) == 1   ? 0x00000000U << 16 : \
      (n) == 2   ? 0x00000001U << 16 : \
      (n) == 4   ? 0x00000002U << 16 : \
      (n) == 8   ? 0x00000004U << 16 : \
      (n) == 16  ? 0x00000007U << 16 : \
      (n) == 32  ? 0x00000008U << 16 : \
      (n) == 64  ? 0x00000009U << 16 : \
      (n) == 128 ? 0x0000000AU << 16 : \
      (n) == 256 ? 0x0000000BU << 16 : \
      (n) == 512 ? 0x0000000CU << 16 : \
      0x00000000U << 16 )

/* Bit[15:12] AXIDIV */
#define RCC_SYSBUSDIV1_AXIDIV_SHIFT         (12)
#define RCC_SYSBUSDIV1_AXIDIV_MASK          (0x0F << RCC_SYSBUSDIV1_AXIDIV_SHIFT)
#define RCC_SYSBUSDIV1_AXIDIV_0             (0x01 << RCC_SYSBUSDIV1_AXIDIV_SHIFT) /* Bit12 */
#define RCC_SYSBUSDIV1_AXIDIV_1             (0x02 << RCC_SYSBUSDIV1_AXIDIV_SHIFT) /* Bit13 */
#define RCC_SYSBUSDIV1_AXIDIV_2             (0x04 << RCC_SYSBUSDIV1_AXIDIV_SHIFT) /* Bit14 */
#define RCC_SYSBUSDIV1_AXIDIV_3             (0x08 << RCC_SYSBUSDIV1_AXIDIV_SHIFT) /* Bit15 */
#define RCC_SYSBUSDIV1_AXIDIV_DIV(n) \
    ( (n) == 1   ? 0x00000000U << 12 : \
      (n) == 2   ? 0x00000001U << 12 : \
      (n) == 4   ? 0x00000002U << 12 : \
      (n) == 8   ? 0x00000004U << 12 : \
      (n) == 16  ? 0x00000007U << 12 : \
      (n) == 32  ? 0x00000008U << 12 : \
      (n) == 64  ? 0x00000009U << 12 : \
      (n) == 128 ? 0x0000000AU << 12 : \
      (n) == 256 ? 0x0000000BU << 12 : \
      (n) == 512 ? 0x0000000CU << 12 : \
      0x00000000U << 12 )

/* Bit[11:8] BUSDIV */
#define RCC_SYSBUSDIV1_BUSDIV_SHIFT         (8)
#define RCC_SYSBUSDIV1_BUSDIV_MASK          (0x0F << RCC_SYSBUSDIV1_BUSDIV_SHIFT)
#define RCC_SYSBUSDIV1_BUSDIV_0             (0x01 << RCC_SYSBUSDIV1_BUSDIV_SHIFT) /* Bit8 */
#define RCC_SYSBUSDIV1_BUSDIV_1             (0x02 << RCC_SYSBUSDIV1_BUSDIV_SHIFT) /* Bit9 */
#define RCC_SYSBUSDIV1_BUSDIV_2             (0x04 << RCC_SYSBUSDIV1_BUSDIV_SHIFT) /* Bit10 */
#define RCC_SYSBUSDIV1_BUSDIV_3             (0x08 << RCC_SYSBUSDIV1_BUSDIV_SHIFT) /* Bit11 */
#define RCC_SYSBUSDIV1_BUSDIV_DIV(n) \
    ( (n) == 1   ? 0x00000000U << 8 : \
      (n) == 2   ? 0x00000001U << 8 : \
      (n) == 4   ? 0x00000002U << 8 : \
      (n) == 8   ? 0x00000004U << 8 : \
      (n) == 16  ? 0x00000007U << 8 : \
      (n) == 32  ? 0x00000008U << 8 : \
      (n) == 64  ? 0x00000009U << 8 : \
      (n) == 128 ? 0x0000000AU << 8 : \
      (n) == 256 ? 0x0000000BU << 8 : \
      (n) == 512 ? 0x0000000CU << 8 : \
      0x00000000U << 8 )

/* Bit[7:4] MSIDIV */
#define RCC_SYSBUSDIV1_MSIDIV_SHIFT         (4)
#define RCC_SYSBUSDIV1_MSIDIV_MASK          (0x0F << RCC_SYSBUSDIV1_MSIDIV_SHIFT)
#define RCC_SYSBUSDIV1_MSIDIV_0             (0x01 << RCC_SYSBUSDIV1_MSIDIV_SHIFT) /* Bit4 */
#define RCC_SYSBUSDIV1_MSIDIV_1             (0x02 << RCC_SYSBUSDIV1_MSIDIV_SHIFT) /* Bit5 */
#define RCC_SYSBUSDIV1_MSIDIV_2             (0x04 << RCC_SYSBUSDIV1_MSIDIV_SHIFT) /* Bit6 */
#define RCC_SYSBUSDIV1_MSIDIV_3             (0x08 << RCC_SYSBUSDIV1_MSIDIV_SHIFT) /* Bit7 */
#define RCC_SYSBUSDIV1_MSIDIV_DIV(n) \
    ( (n) == 1   ? 0x00000000U << 4 : \
      (n) == 2   ? 0x00000001U << 4 : \
      (n) == 4   ? 0x00000002U << 4 : \
      (n) == 8   ? 0x00000004U << 4 : \
      (n) == 16  ? 0x00000007U << 4 : \
      (n) == 32  ? 0x00000008U << 4 : \
      (n) == 64  ? 0x00000009U << 4 : \
      (n) == 128 ? 0x0000000AU << 4 : \
      (n) == 256 ? 0x0000000BU << 4 : \
      (n) == 512 ? 0x0000000CU << 4 : \
      0x00000000U << 4 )

/* Bit[3:0] SYSCLKDIV */
#define RCC_SYSBUSDIV1_SYSCLKDIV_SHIFT        (0)
#define RCC_SYSBUSDIV1_SYSCLKDIV_MASK         (0x0F << RCC_SYSBUSDIV1_SYSCLKDIV_SHIFT)
#define RCC_SYSBUSDIV1_SYSCLKDIV_0            (0x01 << RCC_SYSBUSDIV1_SYSCLKDIV_SHIFT) /* Bit0 */
#define RCC_SYSBUSDIV1_SYSCLKDIV_1            (0x02 << RCC_SYSBUSDIV1_SYSCLKDIV_SHIFT) /* Bit1 */
#define RCC_SYSBUSDIV1_SYSCLKDIV_2            (0x04 << RCC_SYSBUSDIV1_SYSCLKDIV_SHIFT) /* Bit2 */
#define RCC_SYSBUSDIV1_SYSCLKDIV_3            (0x08 << RCC_SYSBUSDIV1_SYSCLKDIV_SHIFT) /* Bit3 */
#define RCC_SYSBUSDIV1_SYSCLKDIV_DIV(n) \
    ( (n) == 1   ? 0x00000000U : \
      (n) == 2   ? 0x00000001U : \
      (n) == 4   ? 0x00000002U : \
      (n) == 8   ? 0x00000004U : \
      (n) == 16  ? 0x00000007U : \
      (n) == 32  ? 0x00000008U : \
      (n) == 64  ? 0x00000009U : \
      (n) == 128 ? 0x0000000AU : \
      (n) == 256 ? 0x0000000BU : \
      (n) == 512 ? 0x0000000CU : \
      0x00000000U )

/* Bit definition for RCC_SYSBUSDIV2 register *******************************/

/* Bit[26:24] APB6DIV */
#define RCC_SYSBUSDIV2_APB6DIV_SHIFT          (24)
#define RCC_SYSBUSDIV2_APB6DIV_MASK           (0x07 << RCC_SYSBUSDIV2_APB6DIV_SHIFT)
#define RCC_SYSBUSDIV2_APB6DIV_0              (0x01 << RCC_SYSBUSDIV2_APB6DIV_SHIFT) /* Bit24 */
#define RCC_SYSBUSDIV2_APB6DIV_1              (0x02 << RCC_SYSBUSDIV2_APB6DIV_SHIFT) /* Bit25 */
#define RCC_SYSBUSDIV2_APB6DIV_2              (0x04 << RCC_SYSBUSDIV2_APB6DIV_SHIFT) /* Bit26 */
#define RCC_SYSBUSDIV2_APB6DIV_DIV(n) \
    ( (n) == 1   ? 0x00000000U << RCC_SYSBUSDIV2_APB6DIV_SHIFT : \
      (n) == 2   ? 0x00000004U << RCC_SYSBUSDIV2_APB6DIV_SHIFT : \
      (n) == 4   ? 0x00000005U << RCC_SYSBUSDIV2_APB6DIV_SHIFT : \
      (n) == 8   ? 0x00000006U << RCC_SYSBUSDIV2_APB6DIV_SHIFT : \
      (n) == 16  ? 0x00000007U << RCC_SYSBUSDIV2_APB6DIV_SHIFT : \
      0x00000000U << RCC_SYSBUSDIV2_APB6DIV_SHIFT )

/* Bit[18:16] APB5DIV */
#define RCC_SYSBUSDIV2_APB5DIV_SHIFT          (16)
#define RCC_SYSBUSDIV2_APB5DIV_MASK           (0x07 << RCC_SYSBUSDIV2_APB5DIV_SHIFT)
#define RCC_SYSBUSDIV2_APB5DIV_0              (0x01 << RCC_SYSBUSDIV2_APB5DIV_SHIFT) /* Bit16 */
#define RCC_SYSBUSDIV2_APB5DIV_1              (0x02 << RCC_SYSBUSDIV2_APB5DIV_SHIFT) /* Bit17 */
#define RCC_SYSBUSDIV2_APB5DIV_2              (0x04 << RCC_SYSBUSDIV2_APB5DIV_SHIFT) /* Bit18 */
#define RCC_SYSBUSDIV2_APB5DIV_DIV(n) \
    ( (n) == 1   ? 0x00000000U << RCC_SYSBUSDIV2_APB5DIV_SHIFT : \
      (n) == 2   ? 0x00000004U << RCC_SYSBUSDIV2_APB5DIV_SHIFT : \
      (n) == 4   ? 0x00000005U << RCC_SYSBUSDIV2_APB5DIV_SHIFT : \
      (n) == 8   ? 0x00000006U << RCC_SYSBUSDIV2_APB5DIV_SHIFT : \
      (n) == 16  ? 0x00000007U << RCC_SYSBUSDIV2_APB5DIV_SHIFT : \
      0x00000000U << RCC_SYSBUSDIV2_APB5DIV_SHIFT )

/* Bit[10:8] APB2DIV */
#define RCC_SYSBUSDIV2_APB2DIV_SHIFT          (8)
#define RCC_SYSBUSDIV2_APB2DIV_MASK           (0x07 << RCC_SYSBUSDIV2_APB2DIV_SHIFT)
#define RCC_SYSBUSDIV2_APB2DIV_0              (0x01 << RCC_SYSBUSDIV2_APB2DIV_SHIFT) /* Bit8 */
#define RCC_SYSBUSDIV2_APB2DIV_1              (0x02 << RCC_SYSBUSDIV2_APB2DIV_SHIFT) /* Bit9 */
#define RCC_SYSBUSDIV2_APB2DIV_2              (0x04 << RCC_SYSBUSDIV2_APB2DIV_SHIFT) /* Bit10 */
#define RCC_SYSBUSDIV2_APB2DIV_DIV(n) \
    ( (n) == 1   ? 0x00000000U << RCC_SYSBUSDIV2_APB2DIV_SHIFT : \
      (n) == 2   ? 0x00000004U << RCC_SYSBUSDIV2_APB2DIV_SHIFT : \
      (n) == 4   ? 0x00000005U << RCC_SYSBUSDIV2_APB2DIV_SHIFT : \
      (n) == 8   ? 0x00000006U << RCC_SYSBUSDIV2_APB2DIV_SHIFT : \
      (n) == 16  ? 0x00000007U << RCC_SYSBUSDIV2_APB2DIV_SHIFT : \
      0x00000000U << RCC_SYSBUSDIV2_APB2DIV_SHIFT )

/* Bit[2:0] APB1DIV */
#define RCC_SYSBUSDIV2_APB1DIV_SHIFT          (0)
#define RCC_SYSBUSDIV2_APB1DIV_MASK           (0x07 << RCC_SYSBUSDIV2_APB1DIV_SHIFT)
#define RCC_SYSBUSDIV2_APB1DIV_0              (0x01 << RCC_SYSBUSDIV2_APB1DIV_SHIFT) /* Bit0 */
#define RCC_SYSBUSDIV2_APB1DIV_1              (0x02 << RCC_SYSBUSDIV2_APB1DIV_SHIFT) /* Bit1 */
#define RCC_SYSBUSDIV2_APB1DIV_2              (0x04 << RCC_SYSBUSDIV2_APB1DIV_SHIFT) /* Bit2 */
#define RCC_SYSBUSDIV2_APB1DIV_DIV(n) \
    ( (n) == 1   ? 0x00000000U << RCC_SYSBUSDIV2_APB1DIV_SHIFT : \
      (n) == 2   ? 0x00000004U << RCC_SYSBUSDIV2_APB1DIV_SHIFT : \
      (n) == 4   ? 0x00000005U << RCC_SYSBUSDIV2_APB1DIV_SHIFT : \
      (n) == 8   ? 0x00000006U << RCC_SYSBUSDIV2_APB1DIV_SHIFT : \
      (n) == 16  ? 0x00000007U << RCC_SYSBUSDIV2_APB1DIV_SHIFT : \
      0x00000000U << RCC_SYSBUSDIV2_APB1DIV_SHIFT )

/* Bit definition for RCC_BOOTMODE register *********************************/

#define RCC_BOOTMODE_BOOTMODE_SHIFT          (0)
#define RCC_BOOTMODE_BOOTMODE_MASK           (0xFFFFFFFF << RCC_BOOTMODE_BOOTMODE_SHIFT)
#define RCC_BOOTMODE_BOOTMODE(n)             ((n) << RCC_BOOTMODE_BOOTMODE_SHIFT)

/* Bit definition for RCC_AHB1DIV1 register *********************************/

/* Bit[31:28] ETH2SYSDIV */
#define RCC_AHB1DIV1_ETH2SYSDIV_SHIFT          (28)
#define RCC_AHB1DIV1_ETH2SYSDIV_MASK           (0x0F << RCC_AHB1DIV1_ETH2SYSDIV_SHIFT)
#define RCC_AHB1DIV1_ETH2SYSDIV_DIV1           (0x00000000U << RCC_AHB1DIV1_ETH2SYSDIV_SHIFT)
#define RCC_AHB1DIV1_ETH2SYSDIV_DIV2           (0x00000001U << RCC_AHB1DIV1_ETH2SYSDIV_SHIFT)
#define RCC_AHB1DIV1_ETH2SYSDIV_DIV4           (0x00000002U << RCC_AHB1DIV1_ETH2SYSDIV_SHIFT)
#define RCC_AHB1DIV1_ETH2SYSDIV_DIV8           (0x00000004U << RCC_AHB1DIV1_ETH2SYSDIV_SHIFT)
#define RCC_AHB1DIV1_ETH2SYSDIV_DIV16          (0x00000007U << RCC_AHB1DIV1_ETH2SYSDIV_SHIFT)
#define RCC_AHB1DIV1_ETH2SYSDIV_DIV32          (0x00000008U << RCC_AHB1DIV1_ETH2SYSDIV_SHIFT)
#define RCC_AHB1DIV1_ETH2SYSDIV_DIV64          (0x00000009U << RCC_AHB1DIV1_ETH2SYSDIV_SHIFT)
#define RCC_AHB1DIV1_ETH2SYSDIV_DIV128         (0x0000000AU << RCC_AHB1DIV1_ETH2SYSDIV_SHIFT)
#define RCC_AHB1DIV1_ETH2SYSDIV_DIV256         (0x0000000BU << RCC_AHB1DIV1_ETH2SYSDIV_SHIFT)
#define RCC_AHB1DIV1_ETH2SYSDIV_DIV512         (0x0000000CU << RCC_AHB1DIV1_ETH2SYSDIV_SHIFT)

/* Bit[27:24] SDMMC2SYSDIV */
#define RCC_AHB1DIV1_SDMMC2SYSDIV_SHIFT        (24)
#define RCC_AHB1DIV1_SDMMC2SYSDIV_MASK         (0x0F << RCC_AHB1DIV1_SDMMC2SYSDIV_SHIFT)
#define RCC_AHB1DIV1_SDMMC2SYSDIV_DIV1         (0x00000000U << RCC_AHB1DIV1_SDMMC2SYSDIV_SHIFT)
#define RCC_AHB1DIV1_SDMMC2SYSDIV_DIV2         (0x00000001U << RCC_AHB1DIV1_SDMMC2SYSDIV_SHIFT)
#define RCC_AHB1DIV1_SDMMC2SYSDIV_DIV4         (0x00000002U << RCC_AHB1DIV1_SDMMC2SYSDIV_SHIFT)
#define RCC_AHB1DIV1_SDMMC2SYSDIV_DIV8         (0x00000004U << RCC_AHB1DIV1_SDMMC2SYSDIV_SHIFT)
#define RCC_AHB1DIV1_SDMMC2SYSDIV_DIV16        (0x00000007U << RCC_AHB1DIV1_SDMMC2SYSDIV_SHIFT)
#define RCC_AHB1DIV1_SDMMC2SYSDIV_DIV32        (0x00000008U << RCC_AHB1DIV1_SDMMC2SYSDIV_SHIFT)
#define RCC_AHB1DIV1_SDMMC2SYSDIV_DIV64        (0x00000009U << RCC_AHB1DIV1_SDMMC2SYSDIV_SHIFT)
#define RCC_AHB1DIV1_SDMMC2SYSDIV_DIV128       (0x0000000AU << RCC_AHB1DIV1_SDMMC2SYSDIV_SHIFT)
#define RCC_AHB1DIV1_SDMMC2SYSDIV_DIV256       (0x0000000BU << RCC_AHB1DIV1_SDMMC2SYSDIV_SHIFT)
#define RCC_AHB1DIV1_SDMMC2SYSDIV_DIV512       (0x0000000CU << RCC_AHB1DIV1_SDMMC2SYSDIV_SHIFT)

/* Bit definition for RCC_AHB1DIV2 register *********************************/

/* Bit[21:16] ADC3SYSDIV */
#define RCC_AHB1DIV2_ADC3SYSDIV_SHIFT          (16)
#define RCC_AHB1DIV2_ADC3SYSDIV_MASK           (0x3F << RCC_AHB1DIV2_ADC3SYSDIV_SHIFT)
#define RCC_AHB1DIV2_ADC3SYSDIV_DIV(n)         ((n >= 1 && n <= 63) ? (n << RCC_AHB1DIV2_ADC3SYSDIV_SHIFT) : 0)

/* Bit[13:8] ADC2SYSDIV */
#define RCC_AHB1DIV2_ADC2SYSDIV_SHIFT          (8)
#define RCC_AHB1DIV2_ADC2SYSDIV_MASK           (0x3F << RCC_AHB1DIV2_ADC2SYSDIV_SHIFT)
#define RCC_AHB1DIV2_ADC2SYSDIV_DIV(n)         ((n >= 1 && n <= 63) ? (n << RCC_AHB1DIV2_ADC2SYSDIV_SHIFT) : 0)

/* Bit[5:0] ADC1SYSDIV */
#define RCC_AHB1DIV2_ADC1SYSDIV_SHIFT          (0)
#define RCC_AHB1DIV2_ADC1SYSDIV_MASK           (0x3F << RCC_AHB1DIV2_ADC1SYSDIV_SHIFT)
#define RCC_AHB1DIV2_ADC1SYSDIV_DIV(n)         ((n >= 1 && n <= 63) ? (n << RCC_AHB1DIV2_ADC1SYSDIV_SHIFT) : 0)

/* Bit definition for RCC_AHB1SEL1 register *********************************/

/* Bit[21:20] ETH2PTPSEL */
#define RCC_AHB1SEL1_ETH2PTPSEL_SHIFT          (20)
#define RCC_AHB1SEL1_ETH2PTPSEL_MASK           (0x03 << RCC_AHB1SEL1_ETH2PTPSEL_SHIFT)
#define RCC_AHB1SEL1_ETH2PTPSEL_SYSCLK         (0x00000000U << RCC_AHB1SEL1_ETH2PTPSEL_SHIFT)

#define RCC_AHB1SEL1_ETH2PTPSEL_PERIPH         (0x00000001U << RCC_AHB1SEL1_ETH2PTPSEL_SHIFT)
#define RCC_AHB1SEL1_ETH2PTPSEL_PLL2C          (0x00000002U << RCC_AHB1SEL1_ETH2PTPSEL_SHIFT)
#define RCC_AHB1SEL1_ETH2PTPSEL_PLL3A          (0x00000003U << RCC_AHB1SEL1_ETH2PTPSEL_SHIFT)

/* Bit[14:12] SDMMC2KERSEL */
#define RCC_AHB1SEL1_SDMMC2KERSEL_SHIFT        (12)
#define RCC_AHB1SEL1_SDMMC2KERSEL_MASK         (0x07 << RCC_AHB1SEL1_SDMMC2KERSEL_SHIFT)
#define RCC_AHB1SEL1_SDMMC2KERSEL_SYSCLK       (0x00000000U << RCC_AHB1SEL1_SDMMC2KERSEL_SHIFT)
#define RCC_AHB1SEL1_SDMMC2KERSEL_PERIPH       (0x00000001U << RCC_AHB1SEL1_SDMMC2KERSEL_SHIFT)
#define RCC_AHB1SEL1_SDMMC2KERSEL_PLL2A        (0x00000002U << RCC_AHB1SEL1_SDMMC2KERSEL_SHIFT)
#define RCC_AHB1SEL1_SDMMC2KERSEL_PLL3A        (0x00000003U << RCC_AHB1SEL1_SDMMC2KERSEL_SHIFT)
#define RCC_AHB1SEL1_SDMMC2KERSEL_PLL1B        (0x00000004U << RCC_AHB1SEL1_SDMMC2KERSEL_SHIFT)

/* Bit[9:8] ADC3PLLSEL */
#define RCC_AHB1SEL1_ADC3PLLSEL_SHIFT          (8)
#define RCC_AHB1SEL1_ADC3PLLSEL_MASK           (0x03 << RCC_AHB1SEL1_ADC3PLLSEL_SHIFT)
#define RCC_AHB1SEL1_ADC3PLLSEL_PLL2B          (0x00000000U << RCC_AHB1SEL1_ADC3PLLSEL_SHIFT)
#define RCC_AHB1SEL1_ADC3PLLSEL_PLL1B          (0x00000001U << RCC_AHB1SEL1_ADC3PLLSEL_SHIFT)
#define RCC_AHB1SEL1_ADC3PLLSEL_PLL3B          (0x00000002U << RCC_AHB1SEL1_ADC3PLLSEL_SHIFT)
#define RCC_AHB1SEL1_ADC3PLLSEL_PLL3C          (0x00000003U << RCC_AHB1SEL1_ADC3PLLSEL_SHIFT)

/* Bit[5:4] ADC2PLLSEL */
#define RCC_AHB1SEL1_ADC2PLLSEL_SHIFT          (4)
#define RCC_AHB1SEL1_ADC2PLLSEL_MASK           (0x03 << RCC_AHB1SEL1_ADC2PLLSEL_SHIFT)
#define RCC_AHB1SEL1_ADC2PLLSEL_PLL2B          (0x00000000U << RCC_AHB1SEL1_ADC2PLLSEL_SHIFT)
#define RCC_AHB1SEL1_ADC2PLLSEL_PLL1B          (0x00000001U << RCC_AHB1SEL1_ADC2PLLSEL_SHIFT)
#define RCC_AHB1SEL1_ADC2PLLSEL_PLL3B          (0x00000002U << RCC_AHB1SEL1_ADC2PLLSEL_SHIFT)
#define RCC_AHB1SEL1_ADC2PLLSEL_PLL3C          (0x00000003U << RCC_AHB1SEL1_ADC2PLLSEL_SHIFT)

/* Bit[1:0] ADC1PLLSEL */
#define RCC_AHB1SEL1_ADC1PLLSEL_SHIFT          (0)
#define RCC_AHB1SEL1_ADC1PLLSEL_MASK           (0x03 << RCC_AHB1SEL1_ADC1PLLSEL_SHIFT)
#define RCC_AHB1SEL1_ADC1PLLSEL_PLL2B          (0x00000000U << RCC_AHB1SEL1_ADC1PLLSEL_SHIFT)
#define RCC_AHB1SEL1_ADC1PLLSEL_PLL1B          (0x00000001U << RCC_AHB1SEL1_ADC1PLLSEL_SHIFT)
#define RCC_AHB1SEL1_ADC1PLLSEL_PLL3B          (0x00000002U << RCC_AHB1SEL1_ADC1PLLSEL_SHIFT)
#define RCC_AHB1SEL1_ADC1PLLSEL_PLL3C          (0x00000003U << RCC_AHB1SEL1_ADC1PLLSEL_SHIFT)

/* Bit definition for RCC_AHB1EN1 register **********************************/

#define RCC_AHB1EN1_M7SDMMC2EN                 (1U << 31)  /* Bit[31] */
#define RCC_AHB1EN1_M4SDMMC2EN                 (1U << 30)  /* Bit[30] */
#define RCC_AHB1EN1_M7SDMMC2LPEN               (1U << 29)  /* Bit[29] */
#define RCC_AHB1EN1_M4SDMMC2LPEN               (1U << 28)  /* Bit[28] */
#define RCC_AHB1EN1_M7USB2EN                   (1U << 23)  /* Bit[23] */
#define RCC_AHB1EN1_M4USB2EN                   (1U << 22)  /* Bit[22] */
#define RCC_AHB1EN1_M7USB2LPEN                 (1U << 21)  /* Bit[21] */
#define RCC_AHB1EN1_M4USB2LPEN                 (1U << 20)  /* Bit[20] */
#define RCC_AHB1EN1_M7DMAMUX1EN                (1U << 19)  /* Bit[19] */
#define RCC_AHB1EN1_M4DMAMUX1EN                (1U << 18)  /* Bit[18] */
#define RCC_AHB1EN1_M7DMAMUX1LPEN              (1U << 17)  /* Bit[17] */
#define RCC_AHB1EN1_M4DMAMUX1LPEN              (1U << 16)  /* Bit[16] */
#define RCC_AHB1EN1_M7ADC1PLLEN                (1U << 15)  /* Bit[15] */
#define RCC_AHB1EN1_M4ADC1PLLEN                (1U << 14)  /* Bit[14] */
#define RCC_AHB1EN1_M7ADC1PLLLPEN              (1U << 13)  /* Bit[13] */
#define RCC_AHB1EN1_M4ADC1PLLLPEN              (1U << 12)  /* Bit[12] */
#define RCC_AHB1EN1_M7ADC1SYSEN                (1U << 11)  /* Bit[11] */
#define RCC_AHB1EN1_M4ADC1SYSEN                (1U << 10)  /* Bit[10] */
#define RCC_AHB1EN1_M7ADC1SYSLPEN              (1U << 9)   /* Bit[9] */
#define RCC_AHB1EN1_M4ADC1SYSLPEN              (1U << 8)   /* Bit[8] */
#define RCC_AHB1EN1_M7ADC1BUSEN                (1U << 3)   /* Bit[3] */
#define RCC_AHB1EN1_M4ADC1BUSEN                (1U << 2)   /* Bit[2] */
#define RCC_AHB1EN1_M7ADC1BUSLPEN              (1U << 1)   /* Bit[1] */
#define RCC_AHB1EN1_M4ADC1BUSLPEN              (1U << 0)   /* Bit[0] */

/* Bit definition for RCC_AHB1EN2 register **********************************/

#define RCC_AHB1EN2_M7ETH2TXEN                 (1U << 11)  /* Bit[11] */
#define RCC_AHB1EN2_M4ETH2TXEN                 (1U << 10)  /* Bit[10] */
#define RCC_AHB1EN2_M7ETH2TXLPEN               (1U << 9)   /* Bit[9] */
#define RCC_AHB1EN2_M4ETH2TXLPEN               (1U << 8)   /* Bit[8] */
#define RCC_AHB1EN2_M7ETH2RXEN                 (1U << 7)   /* Bit[7] */
#define RCC_AHB1EN2_M4ETH2RXEN                 (1U << 6)   /* Bit[6] */
#define RCC_AHB1EN2_M7ETH2RXLPEN               (1U << 5)   /* Bit[5] */
#define RCC_AHB1EN2_M4ETH2RXLPEN               (1U << 4)   /* Bit[4] */
#define RCC_AHB1EN2_M7ETH2MACEN                (1U << 3)   /* Bit[3] */
#define RCC_AHB1EN2_M4ETH2MACEN                (1U << 2)   /* Bit[2] */
#define RCC_AHB1EN2_M7ETH2MACLPEN              (1U << 1)   /* Bit[1] */
#define RCC_AHB1EN2_M4ETH2MACLPEN              (1U << 0)   /* Bit[0] */

/* Bit definition for RCC_AHB1EN3 register **********************************/

#define RCC_AHB1EN3_M7ECCMACEN                 (1U << 27)  /* Bit[27] */
#define RCC_AHB1EN3_M4ECCMACEN                 (1U << 26)  /* Bit[26] */
#define RCC_AHB1EN3_M7ECCMACLPEN               (1U << 25)  /* Bit[25] */
#define RCC_AHB1EN3_M4ECCMACLPEN               (1U << 24)  /* Bit[24] */
#define RCC_AHB1EN3_M7DMA1EN                   (1U << 19)  /* Bit[19] */
#define RCC_AHB1EN3_M4DMA1EN                   (1U << 18)  /* Bit[18] */
#define RCC_AHB1EN3_M7DMA1LPEN                 (1U << 17)  /* Bit[17] */
#define RCC_AHB1EN3_M4DMA1LPEN                 (1U << 16)  /* Bit[16] */
#define RCC_AHB1EN3_M7DMA2EN                   (1U << 11)  /* Bit[11] */
#define RCC_AHB1EN3_M4DMA2EN                   (1U << 10)  /* Bit[10] */
#define RCC_AHB1EN3_M7DMA2LPEN                 (1U << 9)   /* Bit[9] */
#define RCC_AHB1EN3_M4DMA2LPEN                 (1U << 8)   /* Bit[8] */
#define RCC_AHB1EN3_M7DMA3EN                   (1U << 3)   /* Bit[3] */
#define RCC_AHB1EN3_M4DMA3EN                   (1U << 2)   /* Bit[2] */
#define RCC_AHB1EN3_M7DMA3LPEN                 (1U << 1)   /* Bit[1] */
#define RCC_AHB1EN3_M4DMA3LPEN                 (1U << 0)   /* Bit[0] */

/* Bit definition for RCC_AHB1EN4 register **********************************/

#define RCC_AHB1EN4_M7ADC2PLLEN                (1U << 31)  /* Bit[31] */
#define RCC_AHB1EN4_M4ADC2PLLEN                (1U << 30)  /* Bit[30] */
#define RCC_AHB1EN4_M7ADC2PLLLPEN              (1U << 29)  /* Bit[29] */
#define RCC_AHB1EN4_M4ADC2PLLLPEN              (1U << 28)  /* Bit[28] */
#define RCC_AHB1EN4_M7ADC2SYSEN                (1U << 27)  /* Bit[27] */
#define RCC_AHB1EN4_M4ADC2SYSEN                (1U << 26)  /* Bit[26] */
#define RCC_AHB1EN4_M7ADC2SYSLPEN              (1U << 25)  /* Bit[25] */
#define RCC_AHB1EN4_M4ADC2SYSLPEN              (1U << 24)  /* Bit[24] */
#define RCC_AHB1EN4_M7ADC2BUSEN                (1U << 19)  /* Bit[19] */
#define RCC_AHB1EN4_M4ADC2BUSEN                (1U << 18)  /* Bit[18] */
#define RCC_AHB1EN4_M7ADC2BUSLPEN              (1U << 17)  /* Bit[17] */
#define RCC_AHB1EN4_M4ADC2BUSLPEN              (1U << 16)  /* Bit[16] */
#define RCC_AHB1EN4_M7ADC3PLLEN                (1U << 15)  /* Bit[15] */
#define RCC_AHB1EN4_M4ADC3PLLEN                (1U << 14)  /* Bit[14] */
#define RCC_AHB1EN4_M7ADC3PLLLPEN              (1U << 13)  /* Bit[13] */
#define RCC_AHB1EN4_M4ADC3PLLLPEN              (1U << 12)  /* Bit[12] */
#define RCC_AHB1EN4_M7ADC3SYSEN                (1U << 11)  /* Bit[11] */
#define RCC_AHB1EN4_M4ADC3SYSEN                (1U << 10)  /* Bit[10] */
#define RCC_AHB1EN4_M7ADC3SYSLPEN              (1U << 9)   /* Bit[9] */
#define RCC_AHB1EN4_M4ADC3SYSLPEN              (1U << 8)   /* Bit[8] */
#define RCC_AHB1EN4_M7ADC3BUSEN                (1U << 3)   /* Bit[3] */
#define RCC_AHB1EN4_M4ADC3BUSEN                (1U << 2)   /* Bit[2] */
#define RCC_AHB1EN4_M7ADC3BUSLPEN              (1U << 1)   /* Bit[1] */
#define RCC_AHB1EN4_M4ADC3BUSLPEN              (1U << 0)   /* Bit[0] */

/* Bit definition for RCC_AHB1RST1 register *********************************/

#define RCC_AHB1RST1_SDMMC2RST                 (1U << 29)  /* Bit[29] */
#define RCC_AHB1RST1_SDHOST2RST                (1U << 28)  /* Bit[28] */
#define RCC_AHB1RST1_USB2WRAPRST               (1U << 22)  /* Bit[22] */
#define RCC_AHB1RST1_USB2PORRST                (1U << 21)  /* Bit[21] */
#define RCC_AHB1RST1_USB2RST                   (1U << 20)  /* Bit[20] */
#define RCC_AHB1RST1_DMAMUX1RST                (1U << 16)  /* Bit[16] */
#define RCC_AHB1RST1_ADC1RST                   (1U << 0)   /* Bit[0] */

/* Bit definition for RCC_AHB1RST2 register *********************************/

#define RCC_AHB1RST2_ETH2RST                   (1U << 0)   /* Bit[0] */

/* Bit definition for RCC_AHB1RST3 register *********************************/

#define RCC_AHB1RST3_ECCMACRST                 (1U << 24)  /* Bit[24] */
#define RCC_AHB1RST3_DMA1RST                   (1U << 16)  /* Bit[16] */
#define RCC_AHB1RST3_DMA2RST                   (1U << 8)   /* Bit[8] */
#define RCC_AHB1RST3_DMA3RST                   (1U << 0)   /* Bit[0] */

/* Bit definition for RCC_AHB1RST4 register *********************************/

#define RCC_AHB1RST4_ADC2RST                   (1U << 16)  /* Bit[16] */
#define RCC_AHB1RST4_ADC3RST                   (1U << 0)   /* Bit[0] */

/* Bit definition for RCC_APB1DIV1 register *********************************/

/* Bit[30:28] APB1USARTDIV */
#define RCC_APB1DIV1_APB1USARTDIV_SHIFT        (28)
#define RCC_APB1DIV1_APB1USARTDIV_MASK         (0x07 << RCC_APB1DIV1_APB1USARTDIV_SHIFT)
#define RCC_APB1DIV1_APB1USARTDIV_DIV1         (0x00000000U << RCC_APB1DIV1_APB1USARTDIV_SHIFT)
#define RCC_APB1DIV1_APB1USARTDIV_DIV2         (0x00000004U << RCC_APB1DIV1_APB1USARTDIV_SHIFT)
#define RCC_APB1DIV1_APB1USARTDIV_DIV4         (0x00000005U << RCC_APB1DIV1_APB1USARTDIV_SHIFT)
#define RCC_APB1DIV1_APB1USARTDIV_DIV8         (0x00000006U << RCC_APB1DIV1_APB1USARTDIV_SHIFT)
#define RCC_APB1DIV1_APB1USARTDIV_DIV16        (0x00000007U << RCC_APB1DIV1_APB1USARTDIV_SHIFT)

/* Bit[26:24] APB1BTIMDIV */
#define RCC_APB1DIV1_APB1BTIMDIV_SHIFT         (24)
#define RCC_APB1DIV1_APB1BTIMDIV_MASK          (0x07 << RCC_APB1DIV1_APB1BTIMDIV_SHIFT)
#define RCC_APB1DIV1_APB1BTIMDIV_DIV1          (0x00000000U << RCC_APB1DIV1_APB1BTIMDIV_SHIFT)
#define RCC_APB1DIV1_APB1BTIMDIV_DIV2          (0x00000004U << RCC_APB1DIV1_APB1BTIMDIV_SHIFT)
#define RCC_APB1DIV1_APB1BTIMDIV_DIV4          (0x00000005U << RCC_APB1DIV1_APB1BTIMDIV_SHIFT)
#define RCC_APB1DIV1_APB1BTIMDIV_DIV8          (0x00000006U << RCC_APB1DIV1_APB1BTIMDIV_SHIFT)
#define RCC_APB1DIV1_APB1BTIMDIV_DIV16         (0x00000007U << RCC_APB1DIV1_APB1BTIMDIV_SHIFT)

/* Bit[18:16] APB1GTIMDIV */
#define RCC_APB1DIV1_APB1GTIMDIV_SHIFT         (16)
#define RCC_APB1DIV1_APB1GTIMDIV_MASK          (0x07 << RCC_APB1DIV1_APB1GTIMDIV_SHIFT)
#define RCC_APB1DIV1_APB1GTIMDIV_DIV1          (0x00000000U << RCC_APB1DIV1_APB1GTIMDIV_SHIFT)
#define RCC_APB1DIV1_APB1GTIMDIV_DIV2          (0x00000004U << RCC_APB1DIV1_APB1GTIMDIV_SHIFT)
#define RCC_APB1DIV1_APB1GTIMDIV_DIV4          (0x00000005U << RCC_APB1DIV1_APB1GTIMDIV_SHIFT)
#define RCC_APB1DIV1_APB1GTIMDIV_DIV8          (0x00000006U << RCC_APB1DIV1_APB1GTIMDIV_SHIFT)
#define RCC_APB1DIV1_APB1GTIMDIV_DIV16         (0x00000007U << RCC_APB1DIV1_APB1GTIMDIV_SHIFT)

/* Bit[10:8] APB1I2SDIV */
#define RCC_APB1DIV1_APB1I2SDIV_SHIFT          (8)
#define RCC_APB1DIV1_APB1I2SDIV_MASK           (0x07 << RCC_APB1DIV1_APB1I2SDIV_SHIFT)
#define RCC_APB1DIV1_APB1I2SDIV_DIV1           (0x00000000U << RCC_APB1DIV1_APB1I2SDIV_SHIFT)
#define RCC_APB1DIV1_APB1I2SDIV_DIV2           (0x00000004U << RCC_APB1DIV1_APB1I2SDIV_SHIFT)
#define RCC_APB1DIV1_APB1I2SDIV_DIV4           (0x00000005U << RCC_APB1DIV1_APB1I2SDIV_SHIFT)
#define RCC_APB1DIV1_APB1I2SDIV_DIV8           (0x00000006U << RCC_APB1DIV1_APB1I2SDIV_SHIFT)
#define RCC_APB1DIV1_APB1I2SDIV_DIV16          (0x00000007U << RCC_APB1DIV1_APB1I2SDIV_SHIFT)

/* Bit[6:4] APB1FDCANDIV */
#define RCC_APB1DIV1_APB1FDCANDIV_SHIFT        (4)
#define RCC_APB1DIV1_APB1FDCANDIV_MASK         (0x07 << RCC_APB1DIV1_APB1FDCANDIV_SHIFT)
#define RCC_APB1DIV1_APB1FDCANDIV_DIV1         (0x00000000U << RCC_APB1DIV1_APB1FDCANDIV_SHIFT)
#define RCC_APB1DIV1_APB1FDCANDIV_DIV2         (0x00000004U << RCC_APB1DIV1_APB1FDCANDIV_SHIFT)
#define RCC_APB1DIV1_APB1FDCANDIV_DIV4         (0x00000005U << RCC_APB1DIV1_APB1FDCANDIV_SHIFT)
#define RCC_APB1DIV1_APB1FDCANDIV_DIV8         (0x00000006U << RCC_APB1DIV1_APB1FDCANDIV_SHIFT)
#define RCC_APB1DIV1_APB1FDCANDIV_DIV16        (0x00000007U << RCC_APB1DIV1_APB1FDCANDIV_SHIFT)

/* Bit[2:0] APB1I2CDIV */
#define RCC_APB1DIV1_APB1I2CDIV_SHIFT          (0)
#define RCC_APB1DIV1_APB1I2CDIV_MASK           (0x07 << RCC_APB1DIV1_APB1I2CDIV_SHIFT)
#define RCC_APB1DIV1_APB1I2CDIV_DIV1           (0x00000000U << RCC_APB1DIV1_APB1I2CDIV_SHIFT)
#define RCC_APB1DIV1_APB1I2CDIV_DIV2           (0x00000004U << RCC_APB1DIV1_APB1I2CDIV_SHIFT)
#define RCC_APB1DIV1_APB1I2CDIV_DIV4           (0x00000005U << RCC_APB1DIV1_APB1I2CDIV_SHIFT)
#define RCC_APB1DIV1_APB1I2CDIV_DIV8           (0x00000006U << RCC_APB1DIV1_APB1I2CDIV_SHIFT)
#define RCC_APB1DIV1_APB1I2CDIV_DIV16          (0x00000007U << RCC_APB1DIV1_APB1I2CDIV_SHIFT)

/* Bit definition for RCC_APB1SEL1 register *********************************/

/* Bit[30:28] I2C1KERSEL */
#define RCC_APB1SEL1_I2C1KERSEL_SHIFT          (28)
#define RCC_APB1SEL1_I2C1KERSEL_MASK           (0x07 << RCC_APB1SEL1_I2C1KERSEL_SHIFT)
#define RCC_APB1SEL1_I2C1KERSEL_SYSDIV         (0x00000000U << RCC_APB1SEL1_I2C1KERSEL_SHIFT)
#define RCC_APB1SEL1_I2C1KERSEL_PLL3C          (0x00000001U << RCC_APB1SEL1_I2C1KERSEL_SHIFT)
#define RCC_APB1SEL1_I2C1KERSEL_HSI            (0x00000002U << RCC_APB1SEL1_I2C1KERSEL_SHIFT)
#define RCC_APB1SEL1_I2C1KERSEL_MSI            (0x00000003U << RCC_APB1SEL1_I2C1KERSEL_SHIFT)

/* Bit[26:24] I2C2KERSEL */
#define RCC_APB1SEL1_I2C2KERSEL_SHIFT          (24)
#define RCC_APB1SEL1_I2C2KERSEL_MASK           (0x07 << RCC_APB1SEL1_I2C2KERSEL_SHIFT)
#define RCC_APB1SEL1_I2C2KERSEL_SYSDIV         (0x00000000U << RCC_APB1SEL1_I2C2KERSEL_SHIFT)
#define RCC_APB1SEL1_I2C2KERSEL_PLL3C          (0x00000001U << RCC_APB1SEL1_I2C2KERSEL_SHIFT)
#define RCC_APB1SEL1_I2C2KERSEL_HSI            (0x00000002U << RCC_APB1SEL1_I2C2KERSEL_SHIFT)
#define RCC_APB1SEL1_I2C2KERSEL_MSI            (0x00000003U << RCC_APB1SEL1_I2C2KERSEL_SHIFT)

/* Bit[22:20] I2C3KERSEL */
#define RCC_APB1SEL1_I2C3KERSEL_SHIFT          (20)
#define RCC_APB1SEL1_I2C3KERSEL_MASK           (0x07 << RCC_APB1SEL1_I2C3KERSEL_SHIFT)
#define RCC_APB1SEL1_I2C3KERSEL_SYSDIV         (0x00000000U << RCC_APB1SEL1_I2C3KERSEL_SHIFT)
#define RCC_APB1SEL1_I2C3KERSEL_PLL3C          (0x00000001U << RCC_APB1SEL1_I2C3KERSEL_SHIFT)
#define RCC_APB1SEL1_I2C3KERSEL_HSI            (0x00000002U << RCC_APB1SEL1_I2C3KERSEL_SHIFT)
#define RCC_APB1SEL1_I2C3KERSEL_MSI            (0x00000003U << RCC_APB1SEL1_I2C3KERSEL_SHIFT)

/* Bit[18:16] FDCAN1KERSEL */
#define RCC_APB1SEL1_FDCAN1KERSEL_SHIFT        (16)
#define RCC_APB1SEL1_FDCAN1KERSEL_MASK         (0x07 << RCC_APB1SEL1_FDCAN1KERSEL_SHIFT)
#define RCC_APB1SEL1_FDCAN1KERSEL_SYSDIV       (0x00000000U << RCC_APB1SEL1_FDCAN1KERSEL_SHIFT)
#define RCC_APB1SEL1_FDCAN1KERSEL_PLL1C        (0x00000001U << RCC_APB1SEL1_FDCAN1KERSEL_SHIFT)
#define RCC_APB1SEL1_FDCAN1KERSEL_PLL2C        (0x00000002U << RCC_APB1SEL1_FDCAN1KERSEL_SHIFT)
#define RCC_APB1SEL1_FDCAN1KERSEL_PLL3B        (0x00000003U << RCC_APB1SEL1_FDCAN1KERSEL_SHIFT)
#define RCC_APB1SEL1_FDCAN1KERSEL_PERIPH       (0x00000004U << RCC_APB1SEL1_FDCAN1KERSEL_SHIFT)

/* Bit[10:8] FDCAN2KERSEL */
#define RCC_APB1SEL1_FDCAN2KERSEL_SHIFT        (8)
#define RCC_APB1SEL1_FDCAN2KERSEL_MASK         (0x07 << RCC_APB1SEL1_FDCAN2KERSEL_SHIFT)
#define RCC_APB1SEL1_FDCAN2KERSEL_SYSDIV       (0x00000000U << RCC_APB1SEL1_FDCAN2KERSEL_SHIFT)
#define RCC_APB1SEL1_FDCAN2KERSEL_PLL1C        (0x00000001U << RCC_APB1SEL1_FDCAN2KERSEL_SHIFT)
#define RCC_APB1SEL1_FDCAN2KERSEL_PLL2C        (0x00000002U << RCC_APB1SEL1_FDCAN2KERSEL_SHIFT)
#define RCC_APB1SEL1_FDCAN2KERSEL_PLL3B        (0x00000003U << RCC_APB1SEL1_FDCAN2KERSEL_SHIFT)
#define RCC_APB1SEL1_FDCAN2KERSEL_PERIPH       (0x00000004U << RCC_APB1SEL1_FDCAN2KERSEL_SHIFT)

/* Bit[3:2] I2S4KERSEL */
#define RCC_APB1SEL1_I2S4KERSEL_SHIFT          (2)
#define RCC_APB1SEL1_I2S4KERSEL_MASK           (0x03 << RCC_APB1SEL1_I2S4KERSEL_SHIFT)
#define RCC_APB1SEL1_I2S4KERSEL_SYSDIV         (0x00000000U << RCC_APB1SEL1_I2S4KERSEL_SHIFT)
#define RCC_APB1SEL1_I2S4KERSEL_PLL3B          (0x00000001U << RCC_APB1SEL1_I2S4KERSEL_SHIFT)
#define RCC_APB1SEL1_I2S4KERSEL_HSI            (0x00000002U << RCC_APB1SEL1_I2S4KERSEL_SHIFT)
#define RCC_APB1SEL1_I2S4KERSEL_I2S4CKIN       (0x00000003U << RCC_APB1SEL1_I2S4KERSEL_SHIFT)

/* Bit[1:0] I2S3KERSEL */
#define RCC_APB1SEL1_I2S3KERSEL_SHIFT          (0)
#define RCC_APB1SEL1_I2S3KERSEL_MASK           (0x03 << RCC_APB1SEL1_I2S3KERSEL_SHIFT)
#define RCC_APB1SEL1_I2S3KERSEL_SYSDIV         (0x00000000U << RCC_APB1SEL1_I2S3KERSEL_SHIFT)
#define RCC_APB1SEL1_I2S3KERSEL_PLL3B          (0x00000001U << RCC_APB1SEL1_I2S3KERSEL_SHIFT)
#define RCC_APB1SEL1_I2S3KERSEL_HSI            (0x00000002U << RCC_APB1SEL1_I2S3KERSEL_SHIFT)
#define RCC_APB1SEL1_I2S3KERSEL_I2S4CKIN       (0x00000003U << RCC_APB1SEL1_I2S3KERSEL_SHIFT)

/* Bit definition for RCC_APB1SEL2 register *********************************/

/* Bit[30:28] FDCAN5KERSEL */
#define RCC_APB1SEL2_FDCAN5KERSEL_SHIFT        (28)
#define RCC_APB1SEL2_FDCAN5KERSEL_MASK         (0x07 << RCC_APB1SEL2_FDCAN5KERSEL_SHIFT)
#define RCC_APB1SEL2_FDCAN5KERSEL_SYSDIV       (0x00000000U << RCC_APB1SEL2_FDCAN5KERSEL_SHIFT)
#define RCC_APB1SEL2_FDCAN5KERSEL_PLL1C        (0x00000001U << RCC_APB1SEL2_FDCAN5KERSEL_SHIFT)
#define RCC_APB1SEL2_FDCAN5KERSEL_PLL2C        (0x00000002U << RCC_APB1SEL2_FDCAN5KERSEL_SHIFT)
#define RCC_APB1SEL2_FDCAN5KERSEL_PLL3B        (0x00000003U << RCC_APB1SEL2_FDCAN5KERSEL_SHIFT)
#define RCC_APB1SEL2_FDCAN5KERSEL_PERIPH       (0x00000004U << RCC_APB1SEL2_FDCAN5KERSEL_SHIFT)

/* Bit[22:20] FDCAN6KERSEL */
#define RCC_APB1SEL2_FDCAN6KERSEL_SHIFT        (20)
#define RCC_APB1SEL2_FDCAN6KERSEL_MASK         (0x07 << RCC_APB1SEL2_FDCAN6KERSEL_SHIFT)
#define RCC_APB1SEL2_FDCAN6KERSEL_SYSDIV       (0x00000000U << RCC_APB1SEL2_FDCAN6KERSEL_SHIFT)
#define RCC_APB1SEL2_FDCAN6KERSEL_PLL1C        (0x00000001U << RCC_APB1SEL2_FDCAN6KERSEL_SHIFT)
#define RCC_APB1SEL2_FDCAN6KERSEL_PLL2C        (0x00000002U << RCC_APB1SEL2_FDCAN6KERSEL_SHIFT)
#define RCC_APB1SEL2_FDCAN6KERSEL_PLL3B        (0x00000003U << RCC_APB1SEL2_FDCAN6KERSEL_SHIFT)
#define RCC_APB1SEL2_FDCAN6KERSEL_PERIPH       (0x00000004U << RCC_APB1SEL2_FDCAN6KERSEL_SHIFT)

/* Bit definition for RCC_APB1EN1 register **********************************/

#define RCC_APB1EN1_M7BTIM1EN                  (1U << 31)  /* Bit[31] */
#define RCC_APB1EN1_M4BTIM1EN                  (1U << 30)  /* Bit[30] */
#define RCC_APB1EN1_M7BTIM1LPEN                (1U << 29)  /* Bit[29] */
#define RCC_APB1EN1_M4BTIM1LPEN                (1U << 28)  /* Bit[28] */
#define RCC_APB1EN1_M7BTIM2EN                  (1U << 27)  /* Bit[27] */
#define RCC_APB1EN1_M4BTIM2EN                  (1U << 26)  /* Bit[26] */
#define RCC_APB1EN1_M7BTIM2LPEN                (1U << 25)  /* Bit[25] */
#define RCC_APB1EN1_M4BTIM2LPEN                (1U << 24)  /* Bit[24] */
#define RCC_APB1EN1_M7BTIM3EN                  (1U << 23)  /* Bit[23] */
#define RCC_APB1EN1_M4BTIM3EN                  (1U << 22)  /* Bit[22] */
#define RCC_APB1EN1_M7BTIM3LPEN                (1U << 21)  /* Bit[21] */
#define RCC_APB1EN1_M4BTIM3LPEN                (1U << 20)  /* Bit[20] */
#define RCC_APB1EN1_M7BTIM4EN                  (1U << 19)  /* Bit[19] */
#define RCC_APB1EN1_M4BTIM4EN                  (1U << 18)  /* Bit[18] */
#define RCC_APB1EN1_M7BTIM4LPEN                (1U << 17)  /* Bit[17] */
#define RCC_APB1EN1_M4BTIM4LPEN                (1U << 16)  /* Bit[16] */
#define RCC_APB1EN1_M7GTIMB1EN                 (1U << 15)  /* Bit[15] */
#define RCC_APB1EN1_M4GTIMB1EN                 (1U << 14)  /* Bit[14] */
#define RCC_APB1EN1_M7GTIMB1LPEN               (1U << 13)  /* Bit[13] */
#define RCC_APB1EN1_M4GTIMB1LPEN               (1U << 12)  /* Bit[12] */
#define RCC_APB1EN1_M7GTIMB2EN                 (1U << 11)  /* Bit[11] */
#define RCC_APB1EN1_M4GTIMB2EN                 (1U << 10)  /* Bit[10] */
#define RCC_APB1EN1_M7GTIMB2LPEN               (1U << 9)   /* Bit[9] */
#define RCC_APB1EN1_M4GTIMB2LPEN               (1U << 8)   /* Bit[8] */
#define RCC_APB1EN1_M7GTIMB3EN                 (1U << 7)   /* Bit[7] */
#define RCC_APB1EN1_M4GTIMB3EN                 (1U << 6)   /* Bit[6] */
#define RCC_APB1EN1_M7GTIMB3LPEN               (1U << 5)   /* Bit[5] */
#define RCC_APB1EN1_M4GTIMB3LPEN               (1U << 4)   /* Bit[4] */
#define RCC_APB1EN1_M7GTIMA4EN                 (1U << 3)   /* Bit[3] */
#define RCC_APB1EN1_M4GTIMA4EN                 (1U << 2)   /* Bit[2] */
#define RCC_APB1EN1_M7GTIMA4LPEN               (1U << 1)   /* Bit[1] */
#define RCC_APB1EN1_M4GTIMA4LPEN               (1U << 0)   /* Bit[0] */

/* Bit definition for RCC_APB1EN2 register **********************************/

#define RCC_APB1EN2_M7GTIMA5EN                 (1U << 31)  /* Bit[31] */
#define RCC_APB1EN2_M4GTIMA5EN                 (1U << 30)  /* Bit[30] */
#define RCC_APB1EN2_M7GTIMA5LPEN               (1U << 29)  /* Bit[29] */
#define RCC_APB1EN2_M4GTIMA5LPEN               (1U << 28)  /* Bit[28] */
#define RCC_APB1EN2_M7GTIMA6EN                 (1U << 27)  /* Bit[27] */
#define RCC_APB1EN2_M4GTIMA6EN                 (1U << 26)  /* Bit[26] */
#define RCC_APB1EN2_M7GTIMA6LPEN               (1U << 25)  /* Bit[25] */
#define RCC_APB1EN2_M4GTIMA6LPEN               (1U << 24)  /* Bit[24] */
#define RCC_APB1EN2_M7GTIMA7EN                 (1U << 23)  /* Bit[23] */
#define RCC_APB1EN2_M4GTIMA7EN                 (1U << 22)  /* Bit[22] */
#define RCC_APB1EN2_M7GTIMA7LPEN               (1U << 21)  /* Bit[21] */
#define RCC_APB1EN2_M4GTIMA7LPEN               (1U << 20)  /* Bit[20] */
#define RCC_APB1EN2_M7SPI3EN                   (1U << 19)  /* Bit[19] */
#define RCC_APB1EN2_M4SPI3EN                   (1U << 18)  /* Bit[18] */
#define RCC_APB1EN2_M7SPI3LPEN                 (1U << 17)  /* Bit[17] */
#define RCC_APB1EN2_M4SPI3LPEN                 (1U << 16)  /* Bit[16] */
#define RCC_APB1EN2_M7DAC12EN                  (1U << 15)  /* Bit[15] */
#define RCC_APB1EN2_M4DAC12EN                  (1U << 14)  /* Bit[14] */
#define RCC_APB1EN2_M7DAC12LPEN                (1U << 13)  /* Bit[13] */
#define RCC_APB1EN2_M4DAC12LPEN                (1U << 12)  /* Bit[12] */
#define RCC_APB1EN2_M7WWDG2EN                  (1U << 7)   /* Bit[7] */
#define RCC_APB1EN2_M4WWDG2EN                  (1U << 6)   /* Bit[6] */
#define RCC_APB1EN2_M7WWDG2LPEN                (1U << 5)   /* Bit[5] */
#define RCC_APB1EN2_M4WWDG2LPEN                (1U << 4)   /* Bit[4] */

/* Bit definition for RCC_APB1EN3 register **********************************/

#define RCC_APB1EN3_M7USART1EN                 (1U << 31)  /* Bit[31] */
#define RCC_APB1EN3_M4USART1EN                 (1U << 30)  /* Bit[30] */
#define RCC_APB1EN3_M7USART1LPEN               (1U << 29)  /* Bit[29] */
#define RCC_APB1EN3_M4USART1LPEN               (1U << 28)  /* Bit[28] */
#define RCC_APB1EN3_M7USART2EN                 (1U << 27)  /* Bit[27] */
#define RCC_APB1EN3_M4USART2EN                 (1U << 26)  /* Bit[26] */
#define RCC_APB1EN3_M7USART2LPEN               (1U << 25)  /* Bit[25] */
#define RCC_APB1EN3_M4USART2LPEN               (1U << 24)  /* Bit[24] */
#define RCC_APB1EN3_M7USART3EN                 (1U << 23)  /* Bit[23] */
#define RCC_APB1EN3_M4USART3EN                 (1U << 22)  /* Bit[22] */
#define RCC_APB1EN3_M7USART3LPEN               (1U << 21)  /* Bit[21] */
#define RCC_APB1EN3_M4USART3LPEN               (1U << 20)  /* Bit[20] */
#define RCC_APB1EN3_M7USART4EN                 (1U << 19)  /* Bit[19] */
#define RCC_APB1EN3_M4USART4EN                 (1U << 18)  /* Bit[18] */
#define RCC_APB1EN3_M7USART4LPEN               (1U << 17)  /* Bit[17] */
#define RCC_APB1EN3_M4USART4LPEN               (1U << 16)  /* Bit[16] */
#define RCC_APB1EN3_M7UART9EN                  (1U << 15)  /* Bit[15] */
#define RCC_APB1EN3_M4UART9EN                  (1U << 14)  /* Bit[14] */
#define RCC_APB1EN3_M7UART9LPEN                (1U << 13)  /* Bit[13] */
#define RCC_APB1EN3_M4UART9LPEN                (1U << 12)  /* Bit[12] */
#define RCC_APB1EN3_M7UART10EN                 (1U << 11)  /* Bit[11] */
#define RCC_APB1EN3_M4UART10EN                 (1U << 10)  /* Bit[10] */
#define RCC_APB1EN3_M7UART10LPEN               (1U << 9)   /* Bit[9] */
#define RCC_APB1EN3_M4UART10LPEN               (1U << 8)   /* Bit[8] */
#define RCC_APB1EN3_M7UART11EN                 (1U << 7)   /* Bit[7] */
#define RCC_APB1EN3_M4UART11EN                 (1U << 6)   /* Bit[6] */
#define RCC_APB1EN3_M7UART11LPEN               (1U << 5)   /* Bit[5] */
#define RCC_APB1EN3_M4UART11LPEN               (1U << 4)   /* Bit[4] */
#define RCC_APB1EN3_M7UART12EN                 (1U << 3)   /* Bit[3] */
#define RCC_APB1EN3_M4UART12EN                 (1U << 2)   /* Bit[2] */
#define RCC_APB1EN3_M7UART12LPEN               (1U << 1)   /* Bit[1] */
#define RCC_APB1EN3_M4UART12LPEN               (1U << 0)   /* Bit[0] */

/* Bit definition for RCC_APB1EN4 register **********************************/

#define RCC_APB1EN4_M7I2S3EN                   (1U << 31)  /* Bit[31] */
#define RCC_APB1EN4_M4I2S3EN                   (1U << 30)  /* Bit[30] */
#define RCC_APB1EN4_M7I2S3LPEN                 (1U << 29)  /* Bit[29] */
#define RCC_APB1EN4_M4I2S3LPEN                 (1U << 28)  /* Bit[28] */
#define RCC_APB1EN4_M7I2S4EN                   (1U << 27)  /* Bit[27] */
#define RCC_APB1EN4_M4I2S4EN                   (1U << 26)  /* Bit[26] */
#define RCC_APB1EN4_M7I2S4LPEN                 (1U << 25)  /* Bit[25] */
#define RCC_APB1EN4_M4I2S4LPEN                 (1U << 24)  /* Bit[24] */
#define RCC_APB1EN4_M7I2C1EN                   (1U << 23)  /* Bit[23] */
#define RCC_APB1EN4_M4I2C1EN                   (1U << 22)  /* Bit[22] */
#define RCC_APB1EN4_M7I2C1LPEN                 (1U << 21)  /* Bit[21] */
#define RCC_APB1EN4_M4I2C1LPEN                 (1U << 20)  /* Bit[20] */
#define RCC_APB1EN4_M7I2C2EN                   (1U << 19)  /* Bit[19] */
#define RCC_APB1EN4_M4I2C2EN                   (1U << 18)  /* Bit[18] */
#define RCC_APB1EN4_M7I2C2LPEN                 (1U << 17)  /* Bit[17] */
#define RCC_APB1EN4_M4I2C2LPEN                 (1U << 16)  /* Bit[16] */
#define RCC_APB1EN4_M7I2C3EN                   (1U << 15)  /* Bit[15] */
#define RCC_APB1EN4_M4I2C3EN                   (1U << 14)  /* Bit[14] */
#define RCC_APB1EN4_M7I2C3LPEN                 (1U << 13)  /* Bit[13] */
#define RCC_APB1EN4_M4I2C3LPEN                 (1U << 12)  /* Bit[12] */

/* Bit definition for RCC_APB1EN5 register **********************************/

#define RCC_APB1EN5_M7FDCAN1EN                 (1U << 31)  /* Bit[31] */
#define RCC_APB1EN5_M4FDCAN1EN                 (1U << 30)  /* Bit[30] */
#define RCC_APB1EN5_M7FDCAN1LPEN               (1U << 29)  /* Bit[29] */
#define RCC_APB1EN5_M4FDCAN1LPEN               (1U << 28)  /* Bit[28] */
#define RCC_APB1EN5_M7FDCAN2EN                 (1U << 27)  /* Bit[27] */
#define RCC_APB1EN5_M4FDCAN2EN                 (1U << 26)  /* Bit[26] */
#define RCC_APB1EN5_M7FDCAN2LPEN               (1U << 25)  /* Bit[25] */
#define RCC_APB1EN5_M4FDCAN2LPEN               (1U << 24)  /* Bit[24] */
#define RCC_APB1EN5_M7FDCAN5EN                 (1U << 23)  /* Bit[23] */
#define RCC_APB1EN5_M4FDCAN5EN                 (1U << 22)  /* Bit[22] */
#define RCC_APB1EN5_M7FDCAN5LPEN               (1U << 21)  /* Bit[21] */
#define RCC_APB1EN5_M4FDCAN5LPEN               (1U << 20)  /* Bit[20] */
#define RCC_APB1EN5_M7FDCAN6EN                 (1U << 19)  /* Bit[19] */
#define RCC_APB1EN5_M4FDCAN6EN                 (1U << 18)  /* Bit[18] */
#define RCC_APB1EN5_M7FDCAN6LPEN               (1U << 17)  /* Bit[17] */
#define RCC_APB1EN5_M4FDCAN6LPEN               (1U << 16)  /* Bit[16] */
#define RCC_APB1EN5_FDCAN1STPREQ               (1U << 15)  /* Bit[15] */
#define RCC_APB1EN5_FDCAN1STPACK               (1U << 14)  /* Bit[14] */
#define RCC_APB1EN5_FDCAN2STPREQ               (1U << 11)  /* Bit[11] */
#define RCC_APB1EN5_FDCAN2STPACK               (1U << 10)  /* Bit[10] */
#define RCC_APB1EN5_FDCAN5STPREQ               (1U << 7)   /* Bit[7] */
#define RCC_APB1EN5_FDCAN5STPACK               (1U << 6)   /* Bit[6] */
#define RCC_APB1EN5_FDCAN6STPREQ               (1U << 3)   /* Bit[3] */
#define RCC_APB1EN5_FDCAN6STPACK               (1U << 2)   /* Bit[2] */

/* Bit definition for RCC_APB1RST1 register *********************************/

#define RCC_APB1RST1_BTIM1RST                  (1U << 28)  /* Bit[28] */
#define RCC_APB1RST1_BTIM2RST                  (1U << 24)  /* Bit[24] */
#define RCC_APB1RST1_BTIM3RST                  (1U << 20)  /* Bit[20] */
#define RCC_APB1RST1_BTIM4RST                  (1U << 16)  /* Bit[16] */
#define RCC_APB1RST1_GTIMB1RST                 (1U << 12)  /* Bit[12] */
#define RCC_APB1RST1_GTIMB2RST                 (1U << 8)   /* Bit[8] */
#define RCC_APB1RST1_GTIMB3RST                 (1U << 4)   /* Bit[4] */
#define RCC_APB1RST1_GTIMA4RST                 (1U << 0)   /* Bit[0] */

/* Bit definition for RCC_APB1RST2 register *********************************/

#define RCC_APB1RST2_GTIMA5RST                 (1U << 28)  /* Bit[28] */
#define RCC_APB1RST2_GTIMA6RST                 (1U << 24)  /* Bit[24] */
#define RCC_APB1RST2_GTIMA7RST                 (1U << 20)  /* Bit[20] */
#define RCC_APB1RST2_SPI3RST                   (1U << 16)  /* Bit[16] */
#define RCC_APB1RST2_DAC12RST                  (1U << 12)  /* Bit[12] */
#define RCC_APB1RST2_WWDG2RST                  (1U << 4)   /* Bit[4] */

/* Bit definition for RCC_APB1RST3 register *********************************/

#define RCC_APB1RST3_USART1RST                 (1U << 28)  /* Bit[28] */
#define RCC_APB1RST3_USART2RST                 (1U << 24)  /* Bit[24] */
#define RCC_APB1RST3_USART3RST                 (1U << 20)  /* Bit[20] */
#define RCC_APB1RST3_USART4RST                 (1U << 16)  /* Bit[16] */
#define RCC_APB1RST3_UART9RST                  (1U << 12)  /* Bit[12] */
#define RCC_APB1RST3_UART10RST                 (1U << 8)   /* Bit[8] */
#define RCC_APB1RST3_UART11RST                 (1U << 4)   /* Bit[4] */
#define RCC_APB1RST3_UART12RST                 (1U << 0)   /* Bit[0] */

/* Bit definition for RCC_APB1RST4 register *********************************/

#define RCC_APB1RST4_I2S3RST                   (1U << 28)  /* Bit[28] */
#define RCC_APB1RST4_I2S4RST                   (1U << 24)  /* Bit[24] */
#define RCC_APB1RST4_I2C1RST                   (1U << 20)  /* Bit[20] */
#define RCC_APB1RST4_I2C2RST                   (1U << 16)  /* Bit[16] */
#define RCC_APB1RST4_I2C3RST                   (1U << 12)  /* Bit[12] */

/* Bit definition for RCC_APB1RST5 register *********************************/

#define RCC_APB1RST5_FDCAN1RST                 (1U << 28)  /* Bit[28] */
#define RCC_APB1RST5_FDCAN2RST                 (1U << 24)  /* Bit[24] */
#define RCC_APB1RST5_FDCAN5RST                 (1U << 20)  /* Bit[20] */
#define RCC_APB1RST5_FDCAN6RST                 (1U << 16)  /* Bit[16] */
#define RCC_APB1RST5_CAHIRST                   (1U << 4)   /* Bit[4] */
#define RCC_APB1RST5_CAHDRST                   (1U << 0)   /* Bit[0] */

/* Bit definition for RCC_AHB2DIV1 register *********************************/

/* Bit[31:28] ETH1SYSDIV */
#define RCC_AHB2DIV1_ETH1SYSDIV_SHIFT          (28)
#define RCC_AHB2DIV1_ETH1SYSDIV_MASK           (0x0F << RCC_AHB2DIV1_ETH1SYSDIV_SHIFT)
#define RCC_AHB2DIV1_ETH1SYSDIV_DIV1           (0x00000000U << RCC_AHB2DIV1_ETH1SYSDIV_SHIFT)
#define RCC_AHB2DIV1_ETH1SYSDIV_DIV2           (0x00000001U << RCC_AHB2DIV1_ETH1SYSDIV_SHIFT)
#define RCC_AHB2DIV1_ETH1SYSDIV_DIV4           (0x00000002U << RCC_AHB2DIV1_ETH1SYSDIV_SHIFT)
#define RCC_AHB2DIV1_ETH1SYSDIV_DIV8           (0x00000004U << RCC_AHB2DIV1_ETH1SYSDIV_SHIFT)
#define RCC_AHB2DIV1_ETH1SYSDIV_DIV16          (0x00000007U << RCC_AHB2DIV1_ETH1SYSDIV_SHIFT)
#define RCC_AHB2DIV1_ETH1SYSDIV_DIV32          (0x00000008U << RCC_AHB2DIV1_ETH1SYSDIV_SHIFT)
#define RCC_AHB2DIV1_ETH1SYSDIV_DIV64          (0x00000009U << RCC_AHB2DIV1_ETH1SYSDIV_SHIFT)
#define RCC_AHB2DIV1_ETH1SYSDIV_DIV128         (0x0000000AU << RCC_AHB2DIV1_ETH1SYSDIV_SHIFT)
#define RCC_AHB2DIV1_ETH1SYSDIV_DIV256         (0x0000000BU << RCC_AHB2DIV1_ETH1SYSDIV_SHIFT)
#define RCC_AHB2DIV1_ETH1SYSDIV_DIV512         (0x0000000CU << RCC_AHB2DIV1_ETH1SYSDIV_SHIFT)

/* Bit[27:24] USBHSEDIV */
#define RCC_AHB2DIV1_USBHSEDIV_SHIFT           (24)
#define RCC_AHB2DIV1_USBHSEDIV_MASK            (0x0F << RCC_AHB2DIV1_USBHSEDIV_SHIFT)
#define RCC_AHB2DIV1_USBHSEDIV_DIV1            (0x00000000U << RCC_AHB2DIV1_USBHSEDIV_SHIFT)
#define RCC_AHB2DIV1_USBHSEDIV_DIV2            (0x00000001U << RCC_AHB2DIV1_USBHSEDIV_SHIFT)
#define RCC_AHB2DIV1_USBHSEDIV_DIV4            (0x00000002U << RCC_AHB2DIV1_USBHSEDIV_SHIFT)
#define RCC_AHB2DIV1_USBHSEDIV_DIV8            (0x00000004U << RCC_AHB2DIV1_USBHSEDIV_SHIFT)
#define RCC_AHB2DIV1_USBHSEDIV_DIV16           (0x00000007U << RCC_AHB2DIV1_USBHSEDIV_SHIFT)
#define RCC_AHB2DIV1_USBHSEDIV_DIV32           (0x00000008U << RCC_AHB2DIV1_USBHSEDIV_SHIFT)
#define RCC_AHB2DIV1_USBHSEDIV_DIV64           (0x00000009U << RCC_AHB2DIV1_USBHSEDIV_SHIFT)
#define RCC_AHB2DIV1_USBHSEDIV_DIV128          (0x0000000AU << RCC_AHB2DIV1_USBHSEDIV_SHIFT)
#define RCC_AHB2DIV1_USBHSEDIV_DIV256          (0x0000000BU << RCC_AHB2DIV1_USBHSEDIV_SHIFT)
#define RCC_AHB2DIV1_USBHSEDIV_DIV512          (0x0000000CU << RCC_AHB2DIV1_USBHSEDIV_SHIFT)

/* Bit definition for RCC_AHB2SEL1 register *********************************/

/* Bit[21:20] ETH1PTPSEL */
#define RCC_AHB2SEL1_ETH1PTPSEL_SHIFT          (20)
#define RCC_AHB2SEL1_ETH1PTPSEL_MASK           (0x03 << RCC_AHB2SEL1_ETH1PTPSEL_SHIFT)
#define RCC_AHB2SEL1_ETH1PTPSEL_SYSCLK         (0x00000000U << RCC_AHB2SEL1_ETH1PTPSEL_SHIFT)
#define RCC_AHB2SEL1_ETH1PTPSEL_PERIPH         (0x00000001U << RCC_AHB2SEL1_ETH1PTPSEL_SHIFT)
#define RCC_AHB2SEL1_ETH1PTPSEL_PLL2C          (0x00000002U << RCC_AHB2SEL1_ETH1PTPSEL_SHIFT)
#define RCC_AHB2SEL1_ETH1PTPSEL_PLL3A          (0x00000003U << RCC_AHB2SEL1_ETH1PTPSEL_SHIFT)

/* Bit[18:16] ETH1GMIITXSEL */
#define RCC_AHB2SEL1_ETH1GMIITXSEL_SHIFT       (16)
#define RCC_AHB2SEL1_ETH1GMIITXSEL_MASK        (0x07 << RCC_AHB2SEL1_ETH1GMIITXSEL_SHIFT)
#define RCC_AHB2SEL1_ETH1GMIITXSEL_PLL3A       (0x00000000U << RCC_AHB2SEL1_ETH1GMIITXSEL_SHIFT)
#define RCC_AHB2SEL1_ETH1GMIITXSEL_PLL2B       (0x00000001U << RCC_AHB2SEL1_ETH1GMIITXSEL_SHIFT)
#define RCC_AHB2SEL1_ETH1GMIITXSEL_PAD         (0x00000002U << RCC_AHB2SEL1_ETH1GMIITXSEL_SHIFT)
#define RCC_AHB2SEL1_ETH1GMIITXSEL_PLL1C       (0x00000003U << RCC_AHB2SEL1_ETH1GMIITXSEL_SHIFT)

/* Bit definition for RCC_AHB2EN1 register **********************************/

#define RCC_AHB2EN1_M7USB1EN                   (1U << 23)  /* Bit[23] */
#define RCC_AHB2EN1_M4USB1EN                   (1U << 22)  /* Bit[22] */
#define RCC_AHB2EN1_M7USB1LPEN                 (1U << 21)  /* Bit[21] */
#define RCC_AHB2EN1_M4USB1LPEN                 (1U << 20)  /* Bit[20] */
#define RCC_AHB2EN1_M7ECCM2EN                  (1U << 15)  /* Bit[15] */
#define RCC_AHB2EN1_M4ECCM2EN                  (1U << 14)  /* Bit[14] */
#define RCC_AHB2EN1_M7ECCM2LPEN                (1U << 13)  /* Bit[13] */
#define RCC_AHB2EN1_M4ECCM2LPEN                (1U << 12)  /* Bit[12] */
#define RCC_AHB2EN1_M7CORDICEN                 (1U << 11)  /* Bit[11] */
#define RCC_AHB2EN1_M4CORDICEN                 (1U << 10)  /* Bit[10] */
#define RCC_AHB2EN1_M7CORDICLPEN               (1U << 9)   /* Bit[9] */
#define RCC_AHB2EN1_M4CORDICLPEN               (1U << 8)   /* Bit[8] */
#define RCC_AHB2EN1_M7SDPUEN                   (1U << 7)   /* Bit[7] */
#define RCC_AHB2EN1_M4SDPUEN                   (1U << 6)   /* Bit[6] */
#define RCC_AHB2EN1_M7SDPULPEN                 (1U << 5)   /* Bit[5] */
#define RCC_AHB2EN1_M4SDPULPEN                 (1U << 4)   /* Bit[4] */
#define RCC_AHB2EN1_M7FMACEN                   (1U << 3)   /* Bit[3] */
#define RCC_AHB2EN1_M4FMACEN                   (1U << 2)   /* Bit[2] */
#define RCC_AHB2EN1_M7FMACLPEN                 (1U << 1)   /* Bit[1] */
#define RCC_AHB2EN1_M4FMACLPEN                 (1U << 0)   /* Bit[0] */

/* Bit definition for RCC_AHB2EN2 register **********************************/

#define RCC_AHB2EN2_M7DAC56EN                  (1U << 23)  /* Bit[23] */
#define RCC_AHB2EN2_M4DAC56EN                  (1U << 22)  /* Bit[22] */
#define RCC_AHB2EN2_M7DAC56LPEN                (1U << 21)  /* Bit[21] */
#define RCC_AHB2EN2_M4DAC56LPEN                (1U << 20)  /* Bit[20] */
#define RCC_AHB2EN2_M7DAC34EN                  (1U << 19)  /* Bit[19] */
#define RCC_AHB2EN2_M4DAC34EN                  (1U << 18)  /* Bit[18] */
#define RCC_AHB2EN2_M7DAC34LPEN                (1U << 17)  /* Bit[17] */
#define RCC_AHB2EN2_M4DAC34LPEN                (1U << 16)  /* Bit[16] */
#define RCC_AHB2EN2_M7ETH1TEN                  (1U << 11)  /* Bit[11] */
#define RCC_AHB2EN2_M4ETH1TEN                  (1U << 10)  /* Bit[10] */
#define RCC_AHB2EN2_M7ETH1TLPEN                (1U << 9)   /* Bit[9] */
#define RCC_AHB2EN2_M4ETH1TLPEN                (1U << 8)   /* Bit[8] */
#define RCC_AHB2EN2_M7ETH1REN                  (1U << 7)   /* Bit[7] */
#define RCC_AHB2EN2_M4ETH1REN                  (1U << 6)   /* Bit[6] */
#define RCC_AHB2EN2_M7ETH1RLPEN                (1U << 5)   /* Bit[5] */
#define RCC_AHB2EN2_M4ETH1RLPEN                (1U << 4)   /* Bit[4] */
#define RCC_AHB2EN2_M7ETH1MEN                  (1U << 3)   /* Bit[3] */
#define RCC_AHB2EN2_M4ETH1MEN                  (1U << 2)   /* Bit[2] */
#define RCC_AHB2EN2_M7ETH1MLPEN                (1U << 1)   /* Bit[1] */
#define RCC_AHB2EN2_M4ETH1MLPEN                (1U << 0)   /* Bit[0] */

/* Bit definition for RCC_AHB2RST1 register *********************************/

#define RCC_AHB2RST1_DAC56RST                  (1U << 25)  /* Bit[25] */
#define RCC_AHB2RST1_DAC34RST                  (1U << 24)  /* Bit[24] */
#define RCC_AHB2RST1_USB1WRAPRST               (1U << 22)  /* Bit[22] */
#define RCC_AHB2RST1_USB1PORRST                (1U << 21)  /* Bit[21] */
#define RCC_AHB2RST1_USB1RST                   (1U << 20)  /* Bit[20] */
#define RCC_AHB2RST1_ECCM2RST                  (1U << 12)  /* Bit[12] */
#define RCC_AHB2RST1_CORDICRST                 (1U << 8)   /* Bit[8] */
#define RCC_AHB2RST1_SDPURST                   (1U << 4)   /* Bit[4] */
#define RCC_AHB2RST1_FMACRST                   (1U << 0)   /* Bit[0] */

/* Bit definition for RCC_APB2DIV1 register *********************************/

/* Bit[31:28] APB2ATIMDIV */
#define RCC_APB2DIV1_APB2ATIMDIV_SHIFT         (28)
#define RCC_APB2DIV1_APB2ATIMDIV_MASK          (0x0F << RCC_APB2DIV1_APB2ATIMDIV_SHIFT)
#define RCC_APB2DIV1_APB2ATIMDIV_DIV1          (0x00000000U << RCC_APB2DIV1_APB2ATIMDIV_SHIFT)
#define RCC_APB2DIV1_APB2ATIMDIV_DIV2          (0x00000004U << RCC_APB2DIV1_APB2ATIMDIV_SHIFT)
#define RCC_APB2DIV1_APB2ATIMDIV_DIV4          (0x00000005U << RCC_APB2DIV1_APB2ATIMDIV_SHIFT)
#define RCC_APB2DIV1_APB2ATIMDIV_DIV8          (0x00000006U << RCC_APB2DIV1_APB2ATIMDIV_SHIFT)
#define RCC_APB2DIV1_APB2ATIMDIV_DIV16         (0x00000007U << RCC_APB2DIV1_APB2ATIMDIV_SHIFT)

/* Bit[26:24] APB2GTIMDIV */
#define RCC_APB2DIV1_APB2GTIMDIV_SHIFT         (24)
#define RCC_APB2DIV1_APB2GTIMDIV_MASK          (0x07 << RCC_APB2DIV1_APB2GTIMDIV_SHIFT)
#define RCC_APB2DIV1_APB2GTIMDIV_DIV1          (0x00000000U << RCC_APB2DIV1_APB2GTIMDIV_SHIFT)
#define RCC_APB2DIV1_APB2GTIMDIV_DIV2          (0x00000004U << RCC_APB2DIV1_APB2GTIMDIV_SHIFT)
#define RCC_APB2DIV1_APB2GTIMDIV_DIV4          (0x00000005U << RCC_APB2DIV1_APB2GTIMDIV_SHIFT)
#define RCC_APB2DIV1_APB2GTIMDIV_DIV8          (0x00000006U << RCC_APB2DIV1_APB2GTIMDIV_SHIFT)
#define RCC_APB2DIV1_APB2GTIMDIV_DIV16         (0x00000007U << RCC_APB2DIV1_APB2GTIMDIV_SHIFT)

/* Bit[18:16] APB2I2SDIV */
#define RCC_APB2DIV1_APB2I2SDIV_SHIFT          (16)
#define RCC_APB2DIV1_APB2I2SDIV_MASK           (0x07 << RCC_APB2DIV1_APB2I2SDIV_SHIFT)
#define RCC_APB2DIV1_APB2I2SDIV_DIV1           (0x00000000U << RCC_APB2DIV1_APB2I2SDIV_SHIFT)
#define RCC_APB2DIV1_APB2I2SDIV_DIV2           (0x00000004U << RCC_APB2DIV1_APB2I2SDIV_SHIFT)
#define RCC_APB2DIV1_APB2I2SDIV_DIV4           (0x00000005U << RCC_APB2DIV1_APB2I2SDIV_SHIFT)
#define RCC_APB2DIV1_APB2I2SDIV_DIV8           (0x00000006U << RCC_APB2DIV1_APB2I2SDIV_SHIFT)
#define RCC_APB2DIV1_APB2I2SDIV_DIV16          (0x00000007U << RCC_APB2DIV1_APB2I2SDIV_SHIFT)

/* Bit[14:12] APB2DSMUDIV */
#define RCC_APB2DIV1_APB2DSMUDIV_SHIFT         (12)
#define RCC_APB2DIV1_APB2DSMUDIV_MASK          (0x07 << RCC_APB2DIV1_APB2DSMUDIV_SHIFT)
#define RCC_APB2DIV1_APB2DSMUDIV_DIV1          (0x00000000U << RCC_APB2DIV1_APB2DSMUDIV_SHIFT)
#define RCC_APB2DIV1_APB2DSMUDIV_DIV2          (0x00000004U << RCC_APB2DIV1_APB2DSMUDIV_SHIFT)
#define RCC_APB2DIV1_APB2DSMUDIV_DIV4          (0x00000005U << RCC_APB2DIV1_APB2DSMUDIV_SHIFT)
#define RCC_APB2DIV1_APB2DSMUDIV_DIV8          (0x00000006U << RCC_APB2DIV1_APB2DSMUDIV_SHIFT)
#define RCC_APB2DIV1_APB2DSMUDIV_DIV16         (0x00000007U << RCC_APB2DIV1_APB2DSMUDIV_SHIFT)

/* Bit[10:8] APB2I2CDIV */
#define RCC_APB2DIV1_APB2I2CDIV_SHIFT          (8)
#define RCC_APB2DIV1_APB2I2CDIV_MASK           (0x07 << RCC_APB2DIV1_APB2I2CDIV_SHIFT)
#define RCC_APB2DIV1_APB2I2CDIV_DIV1           (0x00000000U << RCC_APB2DIV1_APB2I2CDIV_SHIFT)
#define RCC_APB2DIV1_APB2I2CDIV_DIV2           (0x00000004U << RCC_APB2DIV1_APB2I2CDIV_SHIFT)
#define RCC_APB2DIV1_APB2I2CDIV_DIV4           (0x00000005U << RCC_APB2DIV1_APB2I2CDIV_SHIFT)
#define RCC_APB2DIV1_APB2I2CDIV_DIV8           (0x00000006U << RCC_APB2DIV1_APB2I2CDIV_SHIFT)
#define RCC_APB2DIV1_APB2I2CDIV_DIV16          (0x00000007U << RCC_APB2DIV1_APB2I2CDIV_SHIFT)

/* Bit[6:4] APB2FDCANDIV */
#define RCC_APB2DIV1_APB2FDCANDIV_SHIFT        (4)
#define RCC_APB2DIV1_APB2FDCANDIV_MASK         (0x07 << RCC_APB2DIV1_APB2FDCANDIV_SHIFT)
#define RCC_APB2DIV1_APB2FDCANDIV_DIV1         (0x00000000U << RCC_APB2DIV1_APB2FDCANDIV_SHIFT)
#define RCC_APB2DIV1_APB2FDCANDIV_DIV2         (0x00000004U << RCC_APB2DIV1_APB2FDCANDIV_SHIFT)
#define RCC_APB2DIV1_APB2FDCANDIV_DIV4         (0x00000005U << RCC_APB2DIV1_APB2FDCANDIV_SHIFT)
#define RCC_APB2DIV1_APB2FDCANDIV_DIV8         (0x00000006U << RCC_APB2DIV1_APB2FDCANDIV_SHIFT)
#define RCC_APB2DIV1_APB2FDCANDIV_DIV16        (0x00000007U << RCC_APB2DIV1_APB2FDCANDIV_SHIFT)

/* Bit definition for RCC_APB2SEL1 register *********************************/

/* Bit[20:18] DSMUKERASEL */
#define RCC_APB2SEL1_DSMUKERASEL_SHIFT         (18)
#define RCC_APB2SEL1_DSMUKERASEL_MASK          (0x07 << RCC_APB2SEL1_DSMUKERASEL_SHIFT)
#define RCC_APB2SEL1_DSMUKERASEL_SYSCLK        (0x00000000U << RCC_APB2SEL1_DSMUKERASEL_SHIFT)
#define RCC_APB2SEL1_DSMUKERASEL_PLL1B         (0x00000001U << RCC_APB2SEL1_DSMUKERASEL_SHIFT)
#define RCC_APB2SEL1_DSMUKERASEL_PLL2B         (0x00000002U << RCC_APB2SEL1_DSMUKERASEL_SHIFT)
#define RCC_APB2SEL1_DSMUKERASEL_PLL3A         (0x00000003U << RCC_APB2SEL1_DSMUKERASEL_SHIFT)
#define RCC_APB2SEL1_DSMUKERASEL_I2SCKIN       (0x00000004U << RCC_APB2SEL1_DSMUKERASEL_SHIFT)
#define RCC_APB2SEL1_DSMUKERASEL_PERIPH        (0x00000005U << RCC_APB2SEL1_DSMUKERASEL_SHIFT)

/* Bit[14:12] I2C4KERSEL */
#define RCC_APB2SEL1_I2C4KERSEL_SHIFT          (12)
#define RCC_APB2SEL1_I2C4KERSEL_MASK           (0x07 << RCC_APB2SEL1_I2C4KERSEL_SHIFT)
#define RCC_APB2SEL1_I2C4KERSEL_SYSCLK         (0x00000000U << RCC_APB2SEL1_I2C4KERSEL_SHIFT)
#define RCC_APB2SEL1_I2C4KERSEL_PLL3C          (0x00000001U << RCC_APB2SEL1_I2C4KERSEL_SHIFT)
#define RCC_APB2SEL1_I2C4KERSEL_HSI            (0x00000002U << RCC_APB2SEL1_I2C4KERSEL_SHIFT)
#define RCC_APB2SEL1_I2C4KERSEL_MSI            (0x00000003U << RCC_APB2SEL1_I2C4KERSEL_SHIFT)

/* Bit[10:8] I2C5KERSEL */
#define RCC_APB2SEL1_I2C5KERSEL_SHIFT          (8)
#define RCC_APB2SEL1_I2C5KERSEL_MASK           (0x07 << RCC_APB2SEL1_I2C5KERSEL_SHIFT)
#define RCC_APB2SEL1_I2C5KERSEL_SYSCLK         (0x00000000U << RCC_APB2SEL1_I2C5KERSEL_SHIFT)
#define RCC_APB2SEL1_I2C5KERSEL_PLL3C          (0x00000001U << RCC_APB2SEL1_I2C5KERSEL_SHIFT)
#define RCC_APB2SEL1_I2C5KERSEL_HSI            (0x00000002U << RCC_APB2SEL1_I2C5KERSEL_SHIFT)
#define RCC_APB2SEL1_I2C5KERSEL_MSI            (0x00000003U << RCC_APB2SEL1_I2C5KERSEL_SHIFT)

/* Bit[6:4] I2C6KERSEL */
#define RCC_APB2SEL1_I2C6KERSEL_SHIFT          (4)
#define RCC_APB2SEL1_I2C6KERSEL_MASK           (0x07 << RCC_APB2SEL1_I2C6KERSEL_SHIFT)
#define RCC_APB2SEL1_I2C6KERSEL_SYSCLK         (0x00000000U << RCC_APB2SEL1_I2C6KERSEL_SHIFT)
#define RCC_APB2SEL1_I2C6KERSEL_PLL3C          (0x00000001U << RCC_APB2SEL1_I2C6KERSEL_SHIFT)
#define RCC_APB2SEL1_I2C6KERSEL_HSI            (0x00000002U << RCC_APB2SEL1_I2C6KERSEL_SHIFT)
#define RCC_APB2SEL1_I2C6KERSEL_MSI            (0x00000003U << RCC_APB2SEL1_I2C6KERSEL_SHIFT)

/* Bit[2:0] I2S2KERSEL */
#define RCC_APB2SEL1_I2S2KERSEL_SHIFT          (0)
#define RCC_APB2SEL1_I2S2KERSEL_MASK           (0x03 << RCC_APB2SEL1_I2S2KERSEL_SHIFT)
#define RCC_APB2SEL1_I2S2KERSEL_SYSCLK         (0x00000000U << RCC_APB2SEL1_I2S2KERSEL_SHIFT)
#define RCC_APB2SEL1_I2S2KERSEL_PLL3B          (0x00000001U << RCC_APB2SEL1_I2S2KERSEL_SHIFT)
#define RCC_APB2SEL1_I2S2KERSEL_HSI            (0x00000002U << RCC_APB2SEL1_I2S2KERSEL_SHIFT)
#define RCC_APB2SEL1_I2S2KERSEL_I2S2CKIN       (0x00000003U << RCC_APB2SEL1_I2S2KERSEL_SHIFT)

/* Bit definition for RCC_APB2SEL2 register *********************************/

/* Bit[28:26] FDCAN3KERSEL */
#define RCC_APB2SEL2_FDCAN3KERSEL_SHIFT        (26)
#define RCC_APB2SEL2_FDCAN3KERSEL_MASK         (0x07 << RCC_APB2SEL2_FDCAN3KERSEL_SHIFT)
#define RCC_APB2SEL2_FDCAN3KERSEL_SYSCLK       (0x00000000U << RCC_APB2SEL2_FDCAN3KERSEL_SHIFT)
#define RCC_APB2SEL2_FDCAN3KERSEL_PLL1C        (0x00000001U << RCC_APB2SEL2_FDCAN3KERSEL_SHIFT)
#define RCC_APB2SEL2_FDCAN3KERSEL_PLL2C        (0x00000002U << RCC_APB2SEL2_FDCAN3KERSEL_SHIFT)
#define RCC_APB2SEL2_FDCAN3KERSEL_PLL3B        (0x00000003U << RCC_APB2SEL2_FDCAN3KERSEL_SHIFT)
#define RCC_APB2SEL2_FDCAN3KERSEL_PERIPH       (0x00000004U << RCC_APB2SEL2_FDCAN3KERSEL_SHIFT)

/* Bit[22:20] FDCAN4KERSEL */
#define RCC_APB2SEL2_FDCAN4KERSEL_SHIFT        (20)
#define RCC_APB2SEL2_FDCAN4KERSEL_MASK         (0x07 << RCC_APB2SEL2_FDCAN4KERSEL_SHIFT)
#define RCC_APB2SEL2_FDCAN4KERSEL_SYSCLK       (0x00000000U << RCC_APB2SEL2_FDCAN4KERSEL_SHIFT)
#define RCC_APB2SEL2_FDCAN4KERSEL_PLL1C        (0x00000001U << RCC_APB2SEL2_FDCAN4KERSEL_SHIFT)
#define RCC_APB2SEL2_FDCAN4KERSEL_PLL2C        (0x00000002U << RCC_APB2SEL2_FDCAN4KERSEL_SHIFT)
#define RCC_APB2SEL2_FDCAN4KERSEL_PLL3B        (0x00000003U << RCC_APB2SEL2_FDCAN4KERSEL_SHIFT)
#define RCC_APB2SEL2_FDCAN4KERSEL_PERIPH       (0x00000004U << RCC_APB2SEL2_FDCAN4KERSEL_SHIFT)

/* Bit[14:12] FDCAN7KERSEL */
#define RCC_APB2SEL2_FDCAN7KERSEL_SHIFT        (12)
#define RCC_APB2SEL2_FDCAN7KERSEL_MASK         (0x07 << RCC_APB2SEL2_FDCAN7KERSEL_SHIFT)
#define RCC_APB2SEL2_FDCAN7KERSEL_SYSCLK       (0x00000000U << RCC_APB2SEL2_FDCAN7KERSEL_SHIFT)
#define RCC_APB2SEL2_FDCAN7KERSEL_PLL1C        (0x00000001U << RCC_APB2SEL2_FDCAN7KERSEL_SHIFT)
#define RCC_APB2SEL2_FDCAN7KERSEL_PLL2C        (0x00000002U << RCC_APB2SEL2_FDCAN7KERSEL_SHIFT)
#define RCC_APB2SEL2_FDCAN7KERSEL_PLL3B        (0x00000003U << RCC_APB2SEL2_FDCAN7KERSEL_SHIFT)
#define RCC_APB2SEL2_FDCAN7KERSEL_PERIPH       (0x00000004U << RCC_APB2SEL2_FDCAN7KERSEL_SHIFT)

/* Bit[6:4] FDCAN8KERSEL */
#define RCC_APB2SEL2_FDCAN8KERSEL_SHIFT        (4)
#define RCC_APB2SEL2_FDCAN8KERSEL_MASK         (0x07 << RCC_APB2SEL2_FDCAN8KERSEL_SHIFT)
#define RCC_APB2SEL2_FDCAN8KERSEL_SYSCLK       (0x00000000U << RCC_APB2SEL2_FDCAN8KERSEL_SHIFT)
#define RCC_APB2SEL2_FDCAN8KERSEL_PLL1C        (0x00000001U << RCC_APB2SEL2_FDCAN8KERSEL_SHIFT)
#define RCC_APB2SEL2_FDCAN8KERSEL_PLL2C        (0x00000002U << RCC_APB2SEL2_FDCAN8KERSEL_SHIFT)
#define RCC_APB2SEL2_FDCAN8KERSEL_PLL3B        (0x00000003U << RCC_APB2SEL2_FDCAN8KERSEL_SHIFT)
#define RCC_APB2SEL2_FDCAN8KERSEL_PERIPH       (0x00000004U << RCC_APB2SEL2_FDCAN8KERSEL_SHIFT)

/* Bit definition for RCC_APB2EN1 register **********************************/

#define RCC_APB2EN1_M7ATIM1EN                  (1U << 31)  /* Bit[31] */
#define RCC_APB2EN1_M4ATIM1EN                  (1U << 30)  /* Bit[30] */
#define RCC_APB2EN1_M7ATIM1LPEN                (1U << 29)  /* Bit[29] */
#define RCC_APB2EN1_M4ATIM1LPEN                (1U << 28)  /* Bit[28] */
#define RCC_APB2EN1_M7ATIM2EN                  (1U << 27)  /* Bit[27] */
#define RCC_APB2EN1_M4ATIM2EN                  (1U << 26)  /* Bit[26] */
#define RCC_APB2EN1_M7ATIM2LPEN                (1U << 25)  /* Bit[25] */
#define RCC_APB2EN1_M4ATIM2LPEN                (1U << 24)  /* Bit[24] */
#define RCC_APB2EN1_M7GTIMA1EN                 (1U << 23)  /* Bit[23] */
#define RCC_APB2EN1_M4GTIMA1EN                 (1U << 22)  /* Bit[22] */
#define RCC_APB2EN1_M7GTIMA1LPEN               (1U << 21)  /* Bit[21] */
#define RCC_APB2EN1_M4GTIMA1LPEN               (1U << 20)  /* Bit[20] */
#define RCC_APB2EN1_M7GTIMA2EN                 (1U << 19)  /* Bit[19] */
#define RCC_APB2EN1_M4GTIMA2EN                 (1U << 18)  /* Bit[18] */
#define RCC_APB2EN1_M7GTIMA2LPEN               (1U << 17)  /* Bit[17] */
#define RCC_APB2EN1_M4GTIMA2LPEN               (1U << 16)  /* Bit[16] */
#define RCC_APB2EN1_M7GTIMA3EN                 (1U << 15)  /* Bit[15] */
#define RCC_APB2EN1_M4GTIMA3EN                 (1U << 14)  /* Bit[14] */
#define RCC_APB2EN1_M7GTIMA3LPEN               (1U << 13)  /* Bit[13] */
#define RCC_APB2EN1_M4GTIMA3LPEN               (1U << 12)  /* Bit[12] */
#define RCC_APB2EN1_M7SHRTIM1EN                (1U << 11)  /* Bit[11] */
#define RCC_APB2EN1_M4SHRTIM1EN                (1U << 10)  /* Bit[10] */
#define RCC_APB2EN1_M7SHRTIM1LPEN              (1U << 9)   /* Bit[9] */
#define RCC_APB2EN1_M4SHRTIM1LPEN              (1U << 8)   /* Bit[8] */
#define RCC_APB2EN1_M7SHRTIM2EN                (1U << 7)   /* Bit[7] */
#define RCC_APB2EN1_M4SHRTIM2EN                (1U << 6)   /* Bit[6] */
#define RCC_APB2EN1_M7SHRTIM2LPEN              (1U << 5)   /* Bit[5] */
#define RCC_APB2EN1_M4SHRTIM2LPEN              (1U << 4)   /* Bit[4] */

/* Bit definition for RCC_APB2EN2 register **********************************/

#define RCC_APB2EN2_M7I2S1EN                   (1U << 31)  /* Bit[31] */
#define RCC_APB2EN2_M4I2S1EN                   (1U << 30)  /* Bit[30] */
#define RCC_APB2EN2_M7I2S1LPEN                 (1U << 29)  /* Bit[29] */
#define RCC_APB2EN2_M4I2S1LPEN                 (1U << 28)  /* Bit[28] */
#define RCC_APB2EN2_M7I2S2EN                   (1U << 27)  /* Bit[27] */
#define RCC_APB2EN2_M4I2S2EN                   (1U << 26)  /* Bit[26] */
#define RCC_APB2EN2_M7I2S2LPEN                 (1U << 25)  /* Bit[25] */
#define RCC_APB2EN2_M4I2S2LPEN                 (1U << 24)  /* Bit[24] */
#define RCC_APB2EN2_M7SPI1EN                   (1U << 23)  /* Bit[23] */
#define RCC_APB2EN2_M4SPI1EN                   (1U << 22)  /* Bit[22] */
#define RCC_APB2EN2_M7SPI1LPEN                 (1U << 21)  /* Bit[21] */
#define RCC_APB2EN2_M4SPI1LPEN                 (1U << 20)  /* Bit[20] */
#define RCC_APB2EN2_M7SPI2EN                   (1U << 19)  /* Bit[19] */
#define RCC_APB2EN2_M4SPI2EN                   (1U << 18)  /* Bit[18] */
#define RCC_APB2EN2_M7SPI2LPEN                 (1U << 17)  /* Bit[17] */
#define RCC_APB2EN2_M4SPI2LPEN                 (1U << 16)  /* Bit[16] */
#define RCC_APB2EN2_M7DSMUEN                   (1U << 15)  /* Bit[15] */
#define RCC_APB2EN2_M4DSMUEN                   (1U << 14)  /* Bit[14] */
#define RCC_APB2EN2_M7DSMULPEN                 (1U << 13)  /* Bit[13] */
#define RCC_APB2EN2_M4DSMULPEN                 (1U << 12)  /* Bit[12] */
#define RCC_APB2EN2_M7I2C4EN                   (1U << 11)  /* Bit[11] */
#define RCC_APB2EN2_M4I2C4EN                   (1U << 10)  /* Bit[10] */
#define RCC_APB2EN2_M7I2C4LPEN                 (1U << 9)   /* Bit[9] */
#define RCC_APB2EN2_M4I2C4LPEN                 (1U << 8)   /* Bit[8] */
#define RCC_APB2EN2_M7I2C5EN                   (1U << 7)   /* Bit[7] */
#define RCC_APB2EN2_M4I2C5EN                   (1U << 6)   /* Bit[6] */
#define RCC_APB2EN2_M7I2C5LPEN                 (1U << 5)   /* Bit[5] */
#define RCC_APB2EN2_M4I2C5LPEN                 (1U << 4)   /* Bit[4] */
#define RCC_APB2EN2_M7I2C6EN                   (1U << 3)   /* Bit[3] */
#define RCC_APB2EN2_M4I2C6EN                   (1U << 2)   /* Bit[2] */
#define RCC_APB2EN2_M7I2C6LPEN                 (1U << 1)   /* Bit[1] */
#define RCC_APB2EN2_M4I2C6LPEN                 (1U << 0)   /* Bit[0] */

/* Bit definition for RCC_APB2EN3 register **********************************/

#define RCC_APB2EN3_M7USART5EN                 (1U << 31)  /* Bit[31] */
#define RCC_APB2EN3_M4USART5EN                 (1U << 30)  /* Bit[30] */
#define RCC_APB2EN3_M7USART5LPEN               (1U << 29)  /* Bit[29] */
#define RCC_APB2EN3_M4USART5LPEN               (1U << 28)  /* Bit[28] */
#define RCC_APB2EN3_M7USART6EN                 (1U << 27)  /* Bit[27] */
#define RCC_APB2EN3_M4USART6EN                 (1U << 26)  /* Bit[26] */
#define RCC_APB2EN3_M7USART6LPEN               (1U << 25)  /* Bit[25] */
#define RCC_APB2EN3_M4USART6LPEN               (1U << 24)  /* Bit[24] */
#define RCC_APB2EN3_M7USART7EN                 (1U << 23)  /* Bit[23] */
#define RCC_APB2EN3_M4USART7EN                 (1U << 22)  /* Bit[22] */
#define RCC_APB2EN3_M7USART7LPEN               (1U << 21)  /* Bit[21] */
#define RCC_APB2EN3_M4USART7LPEN               (1U << 20)  /* Bit[20] */
#define RCC_APB2EN3_M7USART8EN                 (1U << 19)  /* Bit[19] */
#define RCC_APB2EN3_M4USART8EN                 (1U << 18)  /* Bit[18] */
#define RCC_APB2EN3_M7USART8LPEN               (1U << 17)  /* Bit[17] */
#define RCC_APB2EN3_M4USART8LPEN               (1U << 16)  /* Bit[16] */
#define RCC_APB2EN3_M7UART13EN                 (1U << 15)  /* Bit[15] */
#define RCC_APB2EN3_M4UART13EN                 (1U << 14)  /* Bit[14] */
#define RCC_APB2EN3_M7UART13LPEN               (1U << 13)  /* Bit[13] */
#define RCC_APB2EN3_M4UART13LPEN               (1U << 12)  /* Bit[12] */
#define RCC_APB2EN3_M7UART14EN                 (1U << 11)  /* Bit[11] */
#define RCC_APB2EN3_M4UART14EN                 (1U << 10)  /* Bit[10] */
#define RCC_APB2EN3_M7UART14LPEN               (1U << 9)   /* Bit[9] */
#define RCC_APB2EN3_M4UART14LPEN               (1U << 8)   /* Bit[8] */
#define RCC_APB2EN3_M7UART15EN                 (1U << 7)   /* Bit[7] */
#define RCC_APB2EN3_M4UART15EN                 (1U << 6)   /* Bit[6] */
#define RCC_APB2EN3_M7UART15LPEN               (1U << 5)   /* Bit[5] */
#define RCC_APB2EN3_M4UART15LPEN               (1U << 4)   /* Bit[4] */

/* Bit definition for RCC_APB2EN4 register **********************************/

#define RCC_APB2EN4_M7FDCAN3EN                 (1U << 31)  /* Bit[31] */
#define RCC_APB2EN4_M4FDCAN3EN                 (1U << 30)  /* Bit[30] */
#define RCC_APB2EN4_M7FDCAN3LPEN               (1U << 29)  /* Bit[29] */
#define RCC_APB2EN4_M4FDCAN3LPEN               (1U << 28)  /* Bit[28] */
#define RCC_APB2EN4_M7FDCAN4EN                 (1U << 27)  /* Bit[27] */
#define RCC_APB2EN4_M4FDCAN4EN                 (1U << 26)  /* Bit[26] */
#define RCC_APB2EN4_M7FDCAN4LPEN               (1U << 25)  /* Bit[25] */
#define RCC_APB2EN4_M4FDCAN4LPEN               (1U << 24)  /* Bit[24] */
#define RCC_APB2EN4_M7FDCAN7EN                 (1U << 23)  /* Bit[23] */
#define RCC_APB2EN4_M4FDCAN7EN                 (1U << 22)  /* Bit[22] */
#define RCC_APB2EN4_M7FDCAN7LPEN               (1U << 21)  /* Bit[21] */
#define RCC_APB2EN4_M4FDCAN7LPEN               (1U << 20)  /* Bit[20] */
#define RCC_APB2EN4_M7FDCAN8EN                 (1U << 19)  /* Bit[19] */
#define RCC_APB2EN4_M4FDCAN8EN                 (1U << 18)  /* Bit[18] */
#define RCC_APB2EN4_M7FDCAN8LPEN               (1U << 17)  /* Bit[17] */
#define RCC_APB2EN4_M4FDCAN8LPEN               (1U << 16)  /* Bit[16] */
#define RCC_APB2EN4_FDCAN3STPREQ               (1U << 15)  /* Bit[15] */
#define RCC_APB2EN4_FDCAN3STPACK               (1U << 14)  /* Bit[14] */
#define RCC_APB2EN4_FDCAN4STPREQ               (1U << 11)  /* Bit[11] */
#define RCC_APB2EN4_FDCAN4STPACK               (1U << 10)  /* Bit[10] */
#define RCC_APB2EN4_FDCAN7STPREQ               (1U << 7)   /* Bit[7] */
#define RCC_APB2EN4_FDCAN7STPACK               (1U << 6)   /* Bit[6] */
#define RCC_APB2EN4_FDCAN8STPREQ               (1U << 3)   /* Bit[3] */
#define RCC_APB2EN4_FDCAN8STPACK               (1U << 2)   /* Bit[2] */

/* Bit definition for RCC_APB2RST1 register *********************************/

#define RCC_APB2RST1_ATIM1RST                  (1U << 28)  /* Bit[28] */
#define RCC_APB2RST1_ATIM2RST                  (1U << 24)  /* Bit[24] */
#define RCC_APB2RST1_GTIMA1RST                 (1U << 20)  /* Bit[20] */
#define RCC_APB2RST1_GTIMA2RST                 (1U << 16)  /* Bit[16] */
#define RCC_APB2RST1_GTIMA3RST                 (1U << 12)  /* Bit[12] */
#define RCC_APB2RST1_SHRTIM1RST                (1U << 8)   /* Bit[8] */
#define RCC_APB2RST1_SHRTIM2RST                (1U << 4)   /* Bit[4] */

/* Bit definition for RCC_APB2RST2 register *********************************/

#define RCC_APB2RST2_I2S1RST                   (1U << 28)  /* Bit[28] */
#define RCC_APB2RST2_I2S2RST                   (1U << 24)  /* Bit[24] */
#define RCC_APB2RST2_SPI1RST                   (1U << 20)  /* Bit[20] */
#define RCC_APB2RST2_SPI2RST                   (1U << 16)  /* Bit[16] */
#define RCC_APB2RST2_DSMURST                   (1U << 12)  /* Bit[12] */
#define RCC_APB2RST2_I2C4RST                   (1U << 8)   /* Bit[8] */
#define RCC_APB2RST2_I2C5RST                   (1U << 4)   /* Bit[4] */
#define RCC_APB2RST2_I2C6RST                   (1U << 0)   /* Bit[0] */

/* Bit definition for RCC_APB2RST3 register *********************************/

#define RCC_APB2RST3_USART5RST                 (1U << 28)  /* Bit[28] */
#define RCC_APB2RST3_USART6RST                 (1U << 24)  /* Bit[24] */
#define RCC_APB2RST3_USART7RST                 (1U << 20)  /* Bit[20] */
#define RCC_APB2RST3_USART8RST                 (1U << 16)  /* Bit[16] */
#define RCC_APB2RST3_UART13RST                 (1U << 12)  /* Bit[12] */
#define RCC_APB2RST3_UART14RST                 (1U << 8)   /* Bit[8] */
#define RCC_APB2RST3_UART15RST                 (1U << 4)   /* Bit[4] */

/* Bit definition for RCC_APB2RST4 register *********************************/

#define RCC_APB2RST4_FDCAN3RST                 (1U << 28)  /* Bit[28] */
#define RCC_APB2RST4_FDCAN4RST                 (1U << 24)  /* Bit[24] */
#define RCC_APB2RST4_FDCAN7RST                 (1U << 20)  /* Bit[20] */
#define RCC_APB2RST4_FDCAN8RST                 (1U << 16)  /* Bit[16] */

/* Bit definition for RCC_AHB5EN1 register **********************************/

#define RCC_AHB5EN1_M7GPIOAEN                  (1U << 31)  /* Bit[31] */
#define RCC_AHB5EN1_M4GPIOAEN                  (1U << 30)  /* Bit[30] */
#define RCC_AHB5EN1_M7GPIOALPEN                (1U << 29)  /* Bit[29] */
#define RCC_AHB5EN1_M4GPIOALPEN                (1U << 28)  /* Bit[28] */
#define RCC_AHB5EN1_M7GPIOBEN                  (1U << 27)  /* Bit[27] */
#define RCC_AHB5EN1_M4GPIOBEN                  (1U << 26)  /* Bit[26] */
#define RCC_AHB5EN1_M7GPIOBLPEN                (1U << 25)  /* Bit[25] */
#define RCC_AHB5EN1_M4GPIOBLPEN                (1U << 24)  /* Bit[24] */
#define RCC_AHB5EN1_M7GPIOCEN                  (1U << 23)  /* Bit[23] */
#define RCC_AHB5EN1_M4GPIOCEN                  (1U << 22)  /* Bit[22] */
#define RCC_AHB5EN1_M7GPIOCLPEN                (1U << 21)  /* Bit[21] */
#define RCC_AHB5EN1_M4GPIOCLPEN                (1U << 20)  /* Bit[20] */
#define RCC_AHB5EN1_M7GPIODEN                  (1U << 19)  /* Bit[19] */
#define RCC_AHB5EN1_M4GPIODEN                  (1U << 18)  /* Bit[18] */
#define RCC_AHB5EN1_M7GPIODLPEN                (1U << 17)  /* Bit[17] */
#define RCC_AHB5EN1_M4GPIODLPEN                (1U << 16)  /* Bit[16] */
#define RCC_AHB5EN1_M7GPIOEEN                  (1U << 15)  /* Bit[15] */
#define RCC_AHB5EN1_M4GPIOEEN                  (1U << 14)  /* Bit[14] */
#define RCC_AHB5EN1_M7GPIOELPEN                (1U << 13)  /* Bit[13] */
#define RCC_AHB5EN1_M4GPIOELPEN                (1U << 12)  /* Bit[12] */
#define RCC_AHB5EN1_M7GPIOFEN                  (1U << 11)  /* Bit[11] */
#define RCC_AHB5EN1_M4GPIOFEN                  (1U << 10)  /* Bit[10] */
#define RCC_AHB5EN1_M7GPIOFLPEN                (1U << 9)   /* Bit[9] */
#define RCC_AHB5EN1_M4GPIOFLPEN                (1U << 8)   /* Bit[8] */
#define RCC_AHB5EN1_M7GPIOGEN                  (1U << 7)   /* Bit[7] */
#define RCC_AHB5EN1_M4GPIOGEN                  (1U << 6)   /* Bit[6] */
#define RCC_AHB5EN1_M7GPIOGLPEN                (1U << 5)   /* Bit[5] */
#define RCC_AHB5EN1_M4GPIOGLPEN                (1U << 4)   /* Bit[4] */
#define RCC_AHB5EN1_M7GPIOHEN                  (1U << 3)   /* Bit[3] */
#define RCC_AHB5EN1_M4GPIOHEN                  (1U << 2)   /* Bit[2] */
#define RCC_AHB5EN1_M7GPIOHLPEN                (1U << 1)   /* Bit[1] */
#define RCC_AHB5EN1_M4GPIOHLPEN                (1U << 0)   /* Bit[0] */

/* Bit definition for RCC_AHB5EN2 register **********************************/

#define RCC_AHB5EN2_M7GPIOIEN                  (1U << 31)  /* Bit[31] */
#define RCC_AHB5EN2_M4GPIOIEN                  (1U << 30)  /* Bit[30] */
#define RCC_AHB5EN2_M7GPIOILPEN                (1U << 29)  /* Bit[29] */
#define RCC_AHB5EN2_M4GPIOILPEN                (1U << 28)  /* Bit[28] */
#define RCC_AHB5EN2_M7GPIOJEN                  (1U << 27)  /* Bit[27] */
#define RCC_AHB5EN2_M4GPIOJEN                  (1U << 26)  /* Bit[26] */
#define RCC_AHB5EN2_M7GPIOJLPEN                (1U << 25)  /* Bit[25] */
#define RCC_AHB5EN2_M4GPIOJLPEN                (1U << 24)  /* Bit[24] */
#define RCC_AHB5EN2_M7GPIOKEN                  (1U << 23)  /* Bit[23] */
#define RCC_AHB5EN2_M4GPIOKEN                  (1U << 22)  /* Bit[22] */
#define RCC_AHB5EN2_M7GPIOKLPEN                (1U << 21)  /* Bit[21] */
#define RCC_AHB5EN2_M4GPIOKLPEN                (1U << 20)  /* Bit[20] */
#define RCC_AHB5EN2_M7ECCM3EN                  (1U << 19)  /* Bit[19] */
#define RCC_AHB5EN2_M4ECCM3EN                  (1U << 18)  /* Bit[18] */
#define RCC_AHB5EN2_M7ECCM3LPEN                (1U << 17)  /* Bit[17] */
#define RCC_AHB5EN2_M4ECCM3LPEN                (1U << 16)  /* Bit[16] */
#define RCC_AHB5EN2_PWREN                      (1U << 15)  /* Bit[15] */
#define RCC_AHB5EN2_M7CRCEN                    (1U << 11)  /* Bit[11] */
#define RCC_AHB5EN2_M4CRCEN                    (1U << 10)  /* Bit[10] */
#define RCC_AHB5EN2_M7CRCLPEN                  (1U << 9)   /* Bit[9] */
#define RCC_AHB5EN2_M4CRCLPEN                  (1U << 8)   /* Bit[8] */
#define RCC_AHB5EN2_M7SEMA4EN                  (1U << 7)   /* Bit[7] */
#define RCC_AHB5EN2_M4SEMA4EN                  (1U << 6)   /* Bit[6] */
#define RCC_AHB5EN2_M7SEMA4LPEN                (1U << 5)   /* Bit[5] */
#define RCC_AHB5EN2_M4SEMA4LPEN                (1U << 4)   /* Bit[4] */
#define RCC_AHB5EN2_M7AFIOEN                   (1U << 3)   /* Bit[3] */
#define RCC_AHB5EN2_M4AFIOEN                   (1U << 2)   /* Bit[2] */
#define RCC_AHB5EN2_M7AFIOLPEN                 (1U << 1)   /* Bit[1] */
#define RCC_AHB5EN2_M4AFIOLPEN                 (1U << 0)   /* Bit[0] */

/* Bit definition for RCC_AHB5RST1 register *********************************/

#define RCC_AHB5RST1_GPIOARST                  (1U << 28)  /* Bit[28] */
#define RCC_AHB5RST1_GPIOBRST                  (1U << 24)  /* Bit[24] */
#define RCC_AHB5RST1_GPIOCRST                  (1U << 20)  /* Bit[20] */
#define RCC_AHB5RST1_GPIODRST                  (1U << 16)  /* Bit[16] */
#define RCC_AHB5RST1_GPIOERST                  (1U << 12)  /* Bit[12] */
#define RCC_AHB5RST1_GPIOFRST                  (1U << 8)   /* Bit[8] */
#define RCC_AHB5RST1_GPIOGRST                  (1U << 4)   /* Bit[4] */
#define RCC_AHB5RST1_GPIOHRST                  (1U << 0)   /* Bit[0] */

/* Bit definition for RCC_AHB5RST2 register *********************************/

#define RCC_AHB5RST2_GPIOIRST                  (1U << 28)  /* Bit[28] */
#define RCC_AHB5RST2_GPIOJRST                  (1U << 24)  /* Bit[24] */
#define RCC_AHB5RST2_GPIOKRST                  (1U << 20)  /* Bit[20] */
#define RCC_AHB5RST2_ECCM3RST                  (1U << 16)  /* Bit[16] */
#define RCC_AHB5RST2_PWRRST                    (1U << 12)  /* Bit[12] */
#define RCC_AHB5RST2_CRCRST                    (1U << 8)   /* Bit[8] */
#define RCC_AHB5RST2_SEMA4RST                  (1U << 4)   /* Bit[4] */
#define RCC_AHB5RST2_AFIORST                   (1U << 0)   /* Bit[0] */

/* Bit definition for RCC_APB5DIV1 register *********************************/

/* Bit[30:28] APB5ATIMDIV */
#define RCC_APB5DIV1_APB5ATIMDIV_SHIFT         (28)
#define RCC_APB5DIV1_APB5ATIMDIV_MASK          (0x07 << RCC_APB5DIV1_APB5ATIMDIV_SHIFT)
#define RCC_APB5DIV1_APB5ATIMDIV_DIV1          (0x00000000U << RCC_APB5DIV1_APB5ATIMDIV_SHIFT)
#define RCC_APB5DIV1_APB5ATIMDIV_DIV2          (0x00000004U << RCC_APB5DIV1_APB5ATIMDIV_SHIFT)
#define RCC_APB5DIV1_APB5ATIMDIV_DIV4          (0x00000005U << RCC_APB5DIV1_APB5ATIMDIV_SHIFT)
#define RCC_APB5DIV1_APB5ATIMDIV_DIV8          (0x00000006U << RCC_APB5DIV1_APB5ATIMDIV_SHIFT)
#define RCC_APB5DIV1_APB5ATIMDIV_DIV16         (0x00000007U << RCC_APB5DIV1_APB5ATIMDIV_SHIFT)

/* Bit[26:24] APB5I2CDIV */
#define RCC_APB5DIV1_APB5I2CDIV_SHIFT          (24)
#define RCC_APB5DIV1_APB5I2CDIV_MASK           (0x07 << RCC_APB5DIV1_APB5I2CDIV_SHIFT)
#define RCC_APB5DIV1_APB5I2CDIV_DIV1           (0x00000000U << RCC_APB5DIV1_APB5I2CDIV_SHIFT)
#define RCC_APB5DIV1_APB5I2CDIV_DIV2           (0x00000004U << RCC_APB5DIV1_APB5I2CDIV_SHIFT)
#define RCC_APB5DIV1_APB5I2CDIV_DIV4           (0x00000005U << RCC_APB5DIV1_APB5I2CDIV_SHIFT)
#define RCC_APB5DIV1_APB5I2CDIV_DIV8           (0x00000006U << RCC_APB5DIV1_APB5I2CDIV_SHIFT)
#define RCC_APB5DIV1_APB5I2CDIV_DIV16          (0x00000007U << RCC_APB5DIV1_APB5I2CDIV_SHIFT)

/* Bit[22:20] APB5EXTIDIV */
#define RCC_APB5DIV1_APB5EXTIDIV_SHIFT         (20)
#define RCC_APB5DIV1_APB5EXTIDIV_MASK          (0x07 << RCC_APB5DIV1_APB5EXTIDIV_SHIFT)
#define RCC_APB5DIV1_APB5EXTIDIV_DIV1          (0x00000000U << RCC_APB5DIV1_APB5EXTIDIV_SHIFT)
#define RCC_APB5DIV1_APB5EXTIDIV_DIV2          (0x00000004U << RCC_APB5DIV1_APB5EXTIDIV_SHIFT)
#define RCC_APB5DIV1_APB5EXTIDIV_DIV4          (0x00000005U << RCC_APB5DIV1_APB5EXTIDIV_SHIFT)
#define RCC_APB5DIV1_APB5EXTIDIV_DIV8          (0x00000006U << RCC_APB5DIV1_APB5EXTIDIV_SHIFT)
#define RCC_APB5DIV1_APB5EXTIDIV_DIV16         (0x00000007U << RCC_APB5DIV1_APB5EXTIDIV_SHIFT)

/* Bit definition for RCC_APB5SEL1 register *********************************/

/* Bit[30:28] I2C7KERSEL */
#define RCC_APB5SEL1_I2C7KERSEL_SHIFT          (28)
#define RCC_APB5SEL1_I2C7KERSEL_MASK           (0x07 << RCC_APB5SEL1_I2C7KERSEL_SHIFT)
#define RCC_APB5SEL1_I2C7KERSEL_SYSCLK         (0x00000000U << RCC_APB5SEL1_I2C7KERSEL_SHIFT)
#define RCC_APB5SEL1_I2C7KERSEL_PLL3C          (0x00000001U << RCC_APB5SEL1_I2C7KERSEL_SHIFT)
#define RCC_APB5SEL1_I2C7KERSEL_HSI            (0x00000002U << RCC_APB5SEL1_I2C7KERSEL_SHIFT)
#define RCC_APB5SEL1_I2C7KERSEL_MSI            (0x00000003U << RCC_APB5SEL1_I2C7KERSEL_SHIFT)

/* Bit[26:24] I2C8KERSEL */
#define RCC_APB5SEL1_I2C8KERSEL_SHIFT          (24)
#define RCC_APB5SEL1_I2C8KERSEL_MASK           (0x07 << RCC_APB5SEL1_I2C8KERSEL_SHIFT)
#define RCC_APB5SEL1_I2C8KERSEL_SYSCLK         (0x00000000U << RCC_APB5SEL1_I2C8KERSEL_SHIFT)
#define RCC_APB5SEL1_I2C8KERSEL_PLL3C          (0x00000001U << RCC_APB5SEL1_I2C8KERSEL_SHIFT)
#define RCC_APB5SEL1_I2C8KERSEL_HSI            (0x00000002U << RCC_APB5SEL1_I2C8KERSEL_SHIFT)
#define RCC_APB5SEL1_I2C8KERSEL_MSI            (0x00000003U << RCC_APB5SEL1_I2C8KERSEL_SHIFT)

/* Bit[22:20] I2C9KERSEL */
#define RCC_APB5SEL1_I2C9KERSEL_SHIFT          (20)
#define RCC_APB5SEL1_I2C9KERSEL_MASK           (0x07 << RCC_APB5SEL1_I2C9KERSEL_SHIFT)
#define RCC_APB5SEL1_I2C9KERSEL_SYSCLK         (0x00000000U << RCC_APB5SEL1_I2C9KERSEL_SHIFT)
#define RCC_APB5SEL1_I2C9KERSEL_PLL3C          (0x00000001U << RCC_APB5SEL1_I2C9KERSEL_SHIFT)
#define RCC_APB5SEL1_I2C9KERSEL_HSI            (0x00000002U << RCC_APB5SEL1_I2C9KERSEL_SHIFT)
#define RCC_APB5SEL1_I2C9KERSEL_MSI            (0x00000003U << RCC_APB5SEL1_I2C9KERSEL_SHIFT)

/* Bit[18:16] I2C10KERSEL */
#define RCC_APB5SEL1_I2C10KERSEL_SHIFT         (16)
#define RCC_APB5SEL1_I2C10KERSEL_MASK          (0x07 << RCC_APB5SEL1_I2C10KERSEL_SHIFT)
#define RCC_APB5SEL1_I2C10KERSEL_SYSCLK        (0x00000000U << RCC_APB5SEL1_I2C10KERSEL_SHIFT)
#define RCC_APB5SEL1_I2C10KERSEL_PLL3C         (0x00000001U << RCC_APB5SEL1_I2C10KERSEL_SHIFT)
#define RCC_APB5SEL1_I2C10KERSEL_HSI           (0x00000002U << RCC_APB5SEL1_I2C10KERSEL_SHIFT)
#define RCC_APB5SEL1_I2C10KERSEL_MSI           (0x00000003U << RCC_APB5SEL1_I2C10KERSEL_SHIFT)

/* Bit definition for RCC_APB5EN1 register **********************************/

#define RCC_APB5EN1_M7ATIM3EN                  (1U << 31)  /* Bit[31] */
#define RCC_APB5EN1_M4ATIM3EN                  (1U << 30)  /* Bit[30] */
#define RCC_APB5EN1_M7ATIM3LPEN                (1U << 29)  /* Bit[29] */
#define RCC_APB5EN1_M4ATIM3LPEN                (1U << 28)  /* Bit[28] */
#define RCC_APB5EN1_M7ATIM4EN                  (1U << 27)  /* Bit[27] */
#define RCC_APB5EN1_M4ATIM4EN                  (1U << 26)  /* Bit[26] */
#define RCC_APB5EN1_M7ATIM4LPEN                (1U << 25)  /* Bit[25] */
#define RCC_APB5EN1_M4ATIM4LPEN                (1U << 24)  /* Bit[24] */
#define RCC_APB5EN1_M7AFECEN                   (1U << 23)  /* Bit[23] */
#define RCC_APB5EN1_M4AFECEN                   (1U << 22)  /* Bit[22] */
#define RCC_APB5EN1_M7AFECLPEN                 (1U << 21)  /* Bit[21] */
#define RCC_APB5EN1_M4AFECLPEN                 (1U << 20)  /* Bit[20] */
#define RCC_APB5EN1_M7SPI4EN                   (1U << 15)  /* Bit[15] */
#define RCC_APB5EN1_M4SPI4EN                   (1U << 14)  /* Bit[14] */
#define RCC_APB5EN1_M7SPI4LPEN                 (1U << 13)  /* Bit[13] */
#define RCC_APB5EN1_M4SPI4LPEN                 (1U << 12)  /* Bit[12] */
#define RCC_APB5EN1_M7SPI5EN                   (1U << 11)  /* Bit[11] */
#define RCC_APB5EN1_M4SPI5EN                   (1U << 10)  /* Bit[10] */
#define RCC_APB5EN1_M7SPI5LPEN                 (1U << 9)   /* Bit[9] */
#define RCC_APB5EN1_M4SPI5LPEN                 (1U << 8)   /* Bit[8] */
#define RCC_APB5EN1_M7SPI6EN                   (1U << 7)   /* Bit[7] */
#define RCC_APB5EN1_M4SPI6EN                   (1U << 6)   /* Bit[6] */
#define RCC_APB5EN1_M7SPI6LPEN                 (1U << 5)   /* Bit[5] */
#define RCC_APB5EN1_M4SPI6LPEN                 (1U << 4)   /* Bit[4] */
#define RCC_APB5EN1_M7SPI7EN                   (1U << 3)   /* Bit[3] */
#define RCC_APB5EN1_M4SPI7EN                   (1U << 2)   /* Bit[2] */
#define RCC_APB5EN1_M7SPI7LPEN                 (1U << 1)   /* Bit[1] */
#define RCC_APB5EN1_M4SPI7LPEN                 (1U << 0)   /* Bit[0] */

/* Bit definition for RCC_APB5EN2 register **********************************/

#define RCC_APB5EN2_M7I2C7EN                   (1U << 31)  /* Bit[31] */
#define RCC_APB5EN2_M4I2C7EN                   (1U << 30)  /* Bit[30] */
#define RCC_APB5EN2_M7I2C7LPEN                 (1U << 29)  /* Bit[29] */
#define RCC_APB5EN2_M4I2C7LPEN                 (1U << 28)  /* Bit[28] */
#define RCC_APB5EN2_M7I2C8EN                   (1U << 27)  /* Bit[27] */
#define RCC_APB5EN2_M4I2C8EN                   (1U << 26)  /* Bit[26] */
#define RCC_APB5EN2_M7I2C8LPEN                 (1U << 25)  /* Bit[25] */
#define RCC_APB5EN2_M4I2C8LPEN                 (1U << 24)  /* Bit[24] */
#define RCC_APB5EN2_M7I2C9EN                   (1U << 23)  /* Bit[23] */
#define RCC_APB5EN2_M4I2C9EN                   (1U << 22)  /* Bit[22] */
#define RCC_APB5EN2_M7I2C9LPEN                 (1U << 21)  /* Bit[21] */
#define RCC_APB5EN2_M4I2C9LPEN                 (1U << 20)  /* Bit[20] */
#define RCC_APB5EN2_M7I2C10EN                  (1U << 19)  /* Bit[19] */
#define RCC_APB5EN2_M4I2C10EN                  (1U << 18)  /* Bit[18] */
#define RCC_APB5EN2_M7I2C10LPEN                (1U << 17)  /* Bit[17] */
#define RCC_APB5EN2_M4I2C10LPEN                (1U << 16)  /* Bit[16] */
#define RCC_APB5EN2_EXTIEN                     (1U << 15)  /* Bit[15] */
#define RCC_APB5EN2_M7RTCPCLKEN                (1U << 11)  /* Bit[11] */
#define RCC_APB5EN2_M4RTCPCLKEN                (1U << 10)  /* Bit[10] */
#define RCC_APB5EN2_M7RTCPCLKLPEN              (1U << 9)   /* Bit[9] */
#define RCC_APB5EN2_M4RTCPCLKLPEN              (1U << 8)   /* Bit[8] */
#define RCC_APB5EN2_IWDG1PCLKEN                (1U << 7)   /* Bit[7] */
#define RCC_APB5EN2_IWDG1PCLKLPEN              (1U << 5)   /* Bit[5] */
#define RCC_APB5EN2_IWDG2PCLKEN                (1U << 3)   /* Bit[3] */
#define RCC_APB5EN2_IWDG2PCLKLPEN              (1U << 1)   /* Bit[1] */

/* Bit definition for RCC_APB5RST1 register *********************************/

#define RCC_APB5RST1_ATIM3RST                  (1U << 28)  /* Bit[28] */
#define RCC_APB5RST1_ATIM4RST                  (1U << 24)  /* Bit[24] */
#define RCC_APB5RST1_SPI4RST                   (1U << 12)  /* Bit[12] */
#define RCC_APB5RST1_SPI5RST                   (1U << 8)   /* Bit[8] */
#define RCC_APB5RST1_SPI6RST                   (1U << 4)   /* Bit[4] */
#define RCC_APB5RST1_SPI7RST                   (1U << 0)   /* Bit[0] */

/* Bit definition for RCC_APB5RST2 register *********************************/

#define RCC_APB5RST2_I2C7RST                   (1U << 28)  /* Bit[28] */
#define RCC_APB5RST2_I2C8RST                   (1U << 24)  /* Bit[24] */
#define RCC_APB5RST2_I2C9RST                   (1U << 20)  /* Bit[20] */
#define RCC_APB5RST2_I2C10RST                  (1U << 16)  /* Bit[16] */

/* Bit definition for RCC_AHB9DIV1 register *********************************/

/* Bit[3:0] ESCSYSDIV */
#define RCC_AHB9DIV1_ESCSYSDIV_SHIFT           (0)
#define RCC_AHB9DIV1_ESCSYSDIV_MASK            (0x0F << RCC_AHB9DIV1_ESCSYSDIV_SHIFT)
#define RCC_AHB9DIV1_ESCSYSDIV_DIV1            (0x00000000U << RCC_AHB9DIV1_ESCSYSDIV_SHIFT)
#define RCC_AHB9DIV1_ESCSYSDIV_DIV2            (0x00000001U << RCC_AHB9DIV1_ESCSYSDIV_SHIFT)
#define RCC_AHB9DIV1_ESCSYSDIV_DIV4            (0x00000002U << RCC_AHB9DIV1_ESCSYSDIV_SHIFT)
#define RCC_AHB9DIV1_ESCSYSDIV_DIV8            (0x00000004U << RCC_AHB9DIV1_ESCSYSDIV_SHIFT)
#define RCC_AHB9DIV1_ESCSYSDIV_DIV16           (0x00000007U << RCC_AHB9DIV1_ESCSYSDIV_SHIFT)
#define RCC_AHB9DIV1_ESCSYSDIV_DIV32           (0x00000008U << RCC_AHB9DIV1_ESCSYSDIV_SHIFT)
#define RCC_AHB9DIV1_ESCSYSDIV_DIV64           (0x00000009U << RCC_AHB9DIV1_ESCSYSDIV_SHIFT)
#define RCC_AHB9DIV1_ESCSYSDIV_DIV128          (0x0000000AU << RCC_AHB9DIV1_ESCSYSDIV_SHIFT)
#define RCC_AHB9DIV1_ESCSYSDIV_DIV256          (0x0000000BU << RCC_AHB9DIV1_ESCSYSDIV_SHIFT)
#define RCC_AHB9DIV1_ESCSYSDIV_DIV512          (0x0000000CU << RCC_AHB9DIV1_ESCSYSDIV_SHIFT)

/* Bit definition for RCC_AHB9SEL1 register *********************************/

/* Bit[2:0] ESCKERSEL */
#define RCC_AHB9SEL1_ESCKERSEL_SHIFT           (0)
#define RCC_AHB9SEL1_ESCKERSEL_MASK            (0x07 << RCC_AHB9SEL1_ESCKERSEL_SHIFT)
#define RCC_AHB9SEL1_ESCKERSEL_SYSCLK          (0x00000000U << RCC_AHB9SEL1_ESCKERSEL_SHIFT)
#define RCC_AHB9SEL1_ESCKERSEL_PLL2B           (0x00000001U << RCC_AHB9SEL1_ESCKERSEL_SHIFT)
#define RCC_AHB9SEL1_ESCKERSEL_PLL3A           (0x00000002U << RCC_AHB9SEL1_ESCKERSEL_SHIFT)
#define RCC_AHB9SEL1_ESCKERSEL_PLL3C           (0x00000003U << RCC_AHB9SEL1_ESCKERSEL_SHIFT)
#define RCC_AHB9SEL1_ESCKERSEL_PLL1B           (0x00000004U << RCC_AHB9SEL1_ESCKERSEL_SHIFT)

/* Bit definition for RCC_AHB9EN1 register **********************************/

#define RCC_AHB9EN1_M7ESCEN                    (1U << 3)   /* Bit[3] */
#define RCC_AHB9EN1_M4ESCEN                    (1U << 2)   /* Bit[2] */
#define RCC_AHB9EN1_M7ESCLPEN                  (1U << 1)   /* Bit[1] */
#define RCC_AHB9EN1_M4ESCLPEN                  (1U << 0)   /* Bit[0] */

/* Bit definition for RCC_AHB9RST1 register *********************************/

#define RCC_AHB9RST1_ESCRST                    (1U << 0)   /* Bit[0] */

/* Bit definition for RCC_RDDIV1 register ***********************************/

/* Bit[30:28] COMPDIV */
#define RCC_RDDIV1_COMPDIV_SHIFT               (28)
#define RCC_RDDIV1_COMPDIV_MASK                (0x07 << RCC_RDDIV1_COMPDIV_SHIFT)
#define RCC_RDDIV1_COMPDIV_DIV1                (0x00000000U << RCC_RDDIV1_COMPDIV_SHIFT)
#define RCC_RDDIV1_COMPDIV_DIV2                (0x00000001U << RCC_RDDIV1_COMPDIV_SHIFT)
#define RCC_RDDIV1_COMPDIV_DIV4                (0x00000002U << RCC_RDDIV1_COMPDIV_SHIFT)
#define RCC_RDDIV1_COMPDIV_DIV8                (0x00000004U << RCC_RDDIV1_COMPDIV_SHIFT)
#define RCC_RDDIV1_COMPDIV_DIV16               (0x00000007U << RCC_RDDIV1_COMPDIV_SHIFT)

/* Bit[26:24] LPUARTDIV */
#define RCC_RDDIV1_LPUARTDIV_SHIFT             (24)
#define RCC_RDDIV1_LPUARTDIV_MASK              (0x07 << RCC_RDDIV1_LPUARTDIV_SHIFT)
#define RCC_RDDIV1_LPUARTDIV_DIV1              (0x00000000U << RCC_RDDIV1_LPUARTDIV_SHIFT)
#define RCC_RDDIV1_LPUARTDIV_DIV2              (0x00000001U << RCC_RDDIV1_LPUARTDIV_SHIFT)
#define RCC_RDDIV1_LPUARTDIV_DIV4              (0x00000002U << RCC_RDDIV1_LPUARTDIV_SHIFT)
#define RCC_RDDIV1_LPUARTDIV_DIV8              (0x00000004U << RCC_RDDIV1_LPUARTDIV_SHIFT)
#define RCC_RDDIV1_LPUARTDIV_DIV16             (0x00000007U << RCC_RDDIV1_LPUARTDIV_SHIFT)

/* Bit definition for RCC_RDSEL1 register ***********************************/

/* Bit[31:28] LPTIM1SEL */
#define RCC_RDSEL1_LPTIM1SEL_SHIFT             (28)
#define RCC_RDSEL1_LPTIM1SEL_MASK              (0x0F << RCC_RDSEL1_LPTIM1SEL_SHIFT)
#define RCC_RDSEL1_LPTIM1SEL_APB5              (0x00000000U << RCC_RDSEL1_LPTIM1SEL_SHIFT)
#define RCC_RDSEL1_LPTIM1SEL_LSI               (0x00000001U << RCC_RDSEL1_LPTIM1SEL_SHIFT)
#define RCC_RDSEL1_LPTIM1SEL_LSE               (0x00000002U << RCC_RDSEL1_LPTIM1SEL_SHIFT)
#define RCC_RDSEL1_LPTIM1SEL_HSE               (0x00000003U << RCC_RDSEL1_LPTIM1SEL_SHIFT)
#define RCC_RDSEL1_LPTIM1SEL_HIS               (0x00000004U << RCC_RDSEL1_LPTIM1SEL_SHIFT)
#define RCC_RDSEL1_LPTIM1SEL_MSI               (0x00000005U << RCC_RDSEL1_LPTIM1SEL_SHIFT)
#define RCC_RDSEL1_LPTIM1SEL_COMP1             (0x00000008U << RCC_RDSEL1_LPTIM1SEL_SHIFT)
#define RCC_RDSEL1_LPTIM1SEL_COMP2             (0x00000009U << RCC_RDSEL1_LPTIM1SEL_SHIFT)
#define RCC_RDSEL1_LPTIM1SEL_COMP3             (0x0000000AU << RCC_RDSEL1_LPTIM1SEL_SHIFT)
#define RCC_RDSEL1_LPTIM1SEL_COMP4             (0x0000000BU << RCC_RDSEL1_LPTIM1SEL_SHIFT)

/* Bit[27:24] LPTIM2SEL */
#define RCC_RDSEL1_LPTIM2SEL_SHIFT             (24)
#define RCC_RDSEL1_LPTIM2SEL_MASK              (0x0F << RCC_RDSEL1_LPTIM2SEL_SHIFT)
#define RCC_RDSEL1_LPTIM2SEL_APB5              (0x00000000U << RCC_RDSEL1_LPTIM2SEL_SHIFT)
#define RCC_RDSEL1_LPTIM2SEL_LSI               (0x00000001U << RCC_RDSEL1_LPTIM2SEL_SHIFT)
#define RCC_RDSEL1_LPTIM2SEL_LSE               (0x00000002U << RCC_RDSEL1_LPTIM2SEL_SHIFT)
#define RCC_RDSEL1_LPTIM2SEL_HSE               (0x00000003U << RCC_RDSEL1_LPTIM2SEL_SHIFT)
#define RCC_RDSEL1_LPTIM2SEL_HIS               (0x00000004U << RCC_RDSEL1_LPTIM2SEL_SHIFT)
#define RCC_RDSEL1_LPTIM2SEL_MSI               (0x00000005U << RCC_RDSEL1_LPTIM2SEL_SHIFT)
#define RCC_RDSEL1_LPTIM2SEL_COMP1             (0x00000008U << RCC_RDSEL1_LPTIM2SEL_SHIFT)
#define RCC_RDSEL1_LPTIM2SEL_COMP2             (0x00000009U << RCC_RDSEL1_LPTIM2SEL_SHIFT)
#define RCC_RDSEL1_LPTIM2SEL_COMP3             (0x0000000AU << RCC_RDSEL1_LPTIM2SEL_SHIFT)
#define RCC_RDSEL1_LPTIM2SEL_COMP4             (0x0000000BU << RCC_RDSEL1_LPTIM2SEL_SHIFT)

/* Bit[23:20] LPTIM3SEL */
#define RCC_RDSEL1_LPTIM3SEL_SHIFT             (20)
#define RCC_RDSEL1_LPTIM3SEL_MASK              (0x0F << RCC_RDSEL1_LPTIM3SEL_SHIFT)
#define RCC_RDSEL1_LPTIM3SEL_APB5              (0x00000000U << RCC_RDSEL1_LPTIM3SEL_SHIFT)
#define RCC_RDSEL1_LPTIM3SEL_LSI               (0x00000001U << RCC_RDSEL1_LPTIM3SEL_SHIFT)
#define RCC_RDSEL1_LPTIM3SEL_LSE               (0x00000002U << RCC_RDSEL1_LPTIM3SEL_SHIFT)
#define RCC_RDSEL1_LPTIM3SEL_HSE               (0x00000003U << RCC_RDSEL1_LPTIM3SEL_SHIFT)
#define RCC_RDSEL1_LPTIM3SEL_HIS               (0x00000004U << RCC_RDSEL1_LPTIM3SEL_SHIFT)
#define RCC_RDSEL1_LPTIM3SEL_MSI               (0x00000005U << RCC_RDSEL1_LPTIM3SEL_SHIFT)
#define RCC_RDSEL1_LPTIM3SEL_COMP1             (0x00000008U << RCC_RDSEL1_LPTIM3SEL_SHIFT)
#define RCC_RDSEL1_LPTIM3SEL_COMP2             (0x00000009U << RCC_RDSEL1_LPTIM3SEL_SHIFT)
#define RCC_RDSEL1_LPTIM3SEL_COMP3             (0x0000000AU << RCC_RDSEL1_LPTIM3SEL_SHIFT)
#define RCC_RDSEL1_LPTIM3SEL_COMP4             (0x0000000BU << RCC_RDSEL1_LPTIM3SEL_SHIFT)

/* Bit[19:16] LPTIM4SEL */
#define RCC_RDSEL1_LPTIM4SEL_SHIFT             (16)
#define RCC_RDSEL1_LPTIM4SEL_MASK              (0x0F << RCC_RDSEL1_LPTIM4SEL_SHIFT)
#define RCC_RDSEL1_LPTIM4SEL_APB5              (0x00000000U << RCC_RDSEL1_LPTIM4SEL_SHIFT)
#define RCC_RDSEL1_LPTIM4SEL_LSI               (0x00000001U << RCC_RDSEL1_LPTIM4SEL_SHIFT)
#define RCC_RDSEL1_LPTIM4SEL_LSE               (0x00000002U << RCC_RDSEL1_LPTIM4SEL_SHIFT)
#define RCC_RDSEL1_LPTIM4SEL_HSE               (0x00000003U << RCC_RDSEL1_LPTIM4SEL_SHIFT)
#define RCC_RDSEL1_LPTIM4SEL_HIS               (0x00000004U << RCC_RDSEL1_LPTIM4SEL_SHIFT)
#define RCC_RDSEL1_LPTIM4SEL_MSI               (0x00000005U << RCC_RDSEL1_LPTIM4SEL_SHIFT)
#define RCC_RDSEL1_LPTIM4SEL_COMP1             (0x00000008U << RCC_RDSEL1_LPTIM4SEL_SHIFT)
#define RCC_RDSEL1_LPTIM4SEL_COMP2             (0x00000009U << RCC_RDSEL1_LPTIM4SEL_SHIFT)
#define RCC_RDSEL1_LPTIM4SEL_COMP3             (0x0000000AU << RCC_RDSEL1_LPTIM4SEL_SHIFT)
#define RCC_RDSEL1_LPTIM4SEL_COMP4             (0x0000000BU << RCC_RDSEL1_LPTIM4SEL_SHIFT)

/* Bit[15:12] LPTIM5SEL */
#define RCC_RDSEL1_LPTIM5SEL_SHIFT             (12)
#define RCC_RDSEL1_LPTIM5SEL_MASK              (0x0F << RCC_RDSEL1_LPTIM5SEL_SHIFT)
#define RCC_RDSEL1_LPTIM5SEL_APB5              (0x00000000U << RCC_RDSEL1_LPTIM5SEL_SHIFT)
#define RCC_RDSEL1_LPTIM5SEL_LSI               (0x00000001U << RCC_RDSEL1_LPTIM5SEL_SHIFT)
#define RCC_RDSEL1_LPTIM5SEL_LSE               (0x00000002U << RCC_RDSEL1_LPTIM5SEL_SHIFT)
#define RCC_RDSEL1_LPTIM5SEL_HSE               (0x00000003U << RCC_RDSEL1_LPTIM5SEL_SHIFT)
#define RCC_RDSEL1_LPTIM5SEL_HIS               (0x00000004U << RCC_RDSEL1_LPTIM5SEL_SHIFT)
#define RCC_RDSEL1_LPTIM5SEL_MSI               (0x00000005U << RCC_RDSEL1_LPTIM5SEL_SHIFT)
#define RCC_RDSEL1_LPTIM5SEL_COMP1             (0x00000008U << RCC_RDSEL1_LPTIM5SEL_SHIFT)
#define RCC_RDSEL1_LPTIM5SEL_COMP2             (0x00000009U << RCC_RDSEL1_LPTIM5SEL_SHIFT)
#define RCC_RDSEL1_LPTIM5SEL_COMP3             (0x0000000AU << RCC_RDSEL1_LPTIM5SEL_SHIFT)
#define RCC_RDSEL1_LPTIM5SEL_COMP4             (0x0000000BU << RCC_RDSEL1_LPTIM5SEL_SHIFT)

/* Bit[10:8] LPUART1SEL */
#define RCC_RDSEL1_LPUART1SEL_SHIFT            (8)
#define RCC_RDSEL1_LPUART1SEL_MASK             (0x07 << RCC_RDSEL1_LPUART1SEL_SHIFT)
#define RCC_RDSEL1_LPUART1SEL_SYSCLK           (0x00000000U << RCC_RDSEL1_LPUART1SEL_SHIFT)
#define RCC_RDSEL1_LPUART1SEL_HSI              (0x00000001U << RCC_RDSEL1_LPUART1SEL_SHIFT)
#define RCC_RDSEL1_LPUART1SEL_LSE              (0x00000002U << RCC_RDSEL1_LPUART1SEL_SHIFT)
#define RCC_RDSEL1_LPUART1SEL_HSE              (0x00000003U << RCC_RDSEL1_LPUART1SEL_SHIFT)
#define RCC_RDSEL1_LPUART1SEL_MSI              (0x00000004U << RCC_RDSEL1_LPUART1SEL_SHIFT)

/* Bit[6:4] LPUART2SEL */
#define RCC_RDSEL1_LPUART2SEL_SHIFT            (4)
#define RCC_RDSEL1_LPUART2SEL_MASK             (0x07 << RCC_RDSEL1_LPUART2SEL_SHIFT)
#define RCC_RDSEL1_LPUART2SEL_SYSCLK           (0x00000000U << RCC_RDSEL1_LPUART2SEL_SHIFT)
#define RCC_RDSEL1_LPUART2SEL_HSI              (0x00000001U << RCC_RDSEL1_LPUART2SEL_SHIFT)
#define RCC_RDSEL1_LPUART2SEL_LSE              (0x00000002U << RCC_RDSEL1_LPUART2SEL_SHIFT)
#define RCC_RDSEL1_LPUART2SEL_HSE              (0x00000003U << RCC_RDSEL1_LPUART2SEL_SHIFT)
#define RCC_RDSEL1_LPUART2SEL_MSI              (0x00000004U << RCC_RDSEL1_LPUART2SEL_SHIFT)

/* Bit[0] COMPSEL */
#define RCC_RDSEL1_COMPSEL                     (1U << 0)  /* Bit[0] */

/* Bit definition for RCC_RDCTRL1 register **********************************/

#define RCC_RDCTRL1_LPTIM2FLTEN                (1U << 29)
#define RCC_RDCTRL1_LPTIM2FLTSEL               (1U << 28)
#define RCC_RDCTRL1_LPTIM2COMP4EN              (1U << 27)
#define RCC_RDCTRL1_LPTIM2COMP3EN              (1U << 26)
#define RCC_RDCTRL1_LPTIM2COMP2EN              (1U << 25)
#define RCC_RDCTRL1_LPTIM2COMP1EN              (1U << 24)
#define RCC_RDCTRL1_LPTIM2FLTDFC_SHIFT         (16)
#define RCC_RDCTRL1_LPTIM2FLTDFC_MASK          (0x1F << RCC_RDCTRL1_LPTIM2FLTDFC_SHIFT)
#define RCC_RDCTRL1_LPTIM1FLTEN                (1U << 13)
#define RCC_RDCTRL1_LPTIM1FLTSEL               (1U << 12)
#define RCC_RDCTRL1_LPTIM1COMP4EN              (1U << 11)
#define RCC_RDCTRL1_LPTIM1COMP3EN              (1U << 10)
#define RCC_RDCTRL1_LPTIM1COMP2EN              (1U << 9)
#define RCC_RDCTRL1_LPTIM1COMP1EN              (1U << 8)
#define RCC_RDCTRL1_LPTIM1FLTDFC_SHIFT         (0)
#define RCC_RDCTRL1_LPTIM1FLTDFC_MASK          (0x1F << RCC_RDCTRL1_LPTIM1FLTDFC_SHIFT)

/* Bit definition for RCC_RDCTRL2 register **********************************/

#define RCC_RDCTRL2_LPTIM4FLTEN                (1U << 29)
#define RCC_RDCTRL2_LPTIM4FLTSEL               (1U << 28)
#define RCC_RDCTRL2_LPTIM4COMP4EN              (1U << 27)
#define RCC_RDCTRL2_LPTIM4COMP3EN              (1U << 26)
#define RCC_RDCTRL2_LPTIM4COMP2EN              (1U << 25)
#define RCC_RDCTRL2_LPTIM4COMP1EN              (1U << 24)
#define RCC_RDCTRL2_LPTIM4FLTDFC_SHIFT         (16)
#define RCC_RDCTRL2_LPTIM4FLTDFC_MASK          (0x1F << RCC_RDCTRL2_LPTIM4FLTDFC_SHIFT)
#define RCC_RDCTRL2_LPTIM3FLTEN                (1U << 13)
#define RCC_RDCTRL2_LPTIM3FLTSEL               (1U << 12)
#define RCC_RDCTRL2_LPTIM3COMP4EN              (1U << 11)
#define RCC_RDCTRL2_LPTIM3COMP3EN              (1U << 10)
#define RCC_RDCTRL2_LPTIM3COMP2EN              (1U << 9)
#define RCC_RDCTRL2_LPTIM3COMP1EN              (1U << 8)
#define RCC_RDCTRL2_LPTIM3FLTDFC_SHIFT         (0)
#define RCC_RDCTRL2_LPTIM3FLTDFC_MASK          (0x1F << RCC_RDCTRL2_LPTIM3FLTDFC_SHIFT)

/* Bit definition for RCC_RDCTRL3 register **********************************/

#define RCC_RDCTRL3_LPTIM5FLTEN                (1U << 13)
#define RCC_RDCTRL3_LPTIM5FLTSEL               (1U << 12)
#define RCC_RDCTRL3_LPTIM5COMP4EN              (1U << 11)
#define RCC_RDCTRL3_LPTIM5COMP3EN              (1U << 10)
#define RCC_RDCTRL3_LPTIM5COMP2EN              (1U << 9)
#define RCC_RDCTRL3_LPTIM5COMP1EN              (1U << 8)
#define RCC_RDCTRL3_LPTIM5FLTDFC_SHIFT         (0)
#define RCC_RDCTRL3_LPTIM5FLTDFC_MASK          (0x1F << RCC_RDCTRL3_LPTIM5FLTDFC_SHIFT)

/* Bit definition for RCC_RDEN1 register ************************************/

#define RCC_RDEN1_M7LPTIM1EN                   (1U << 31)
#define RCC_RDEN1_M4LPTIM1EN                   (1U << 30)
#define RCC_RDEN1_M7LPTIM1LPEN                 (1U << 29)
#define RCC_RDEN1_M4LPTIM1LPEN                 (1U << 28)
#define RCC_RDEN1_M7LPTIM2EN                   (1U << 27)
#define RCC_RDEN1_M4LPTIM2EN                   (1U << 26)
#define RCC_RDEN1_M7LPTIM2LPEN                 (1U << 25)
#define RCC_RDEN1_M4LPTIM2LPEN                 (1U << 24)
#define RCC_RDEN1_M7LPTIM3EN                   (1U << 23)
#define RCC_RDEN1_M4LPTIM3EN                   (1U << 22)
#define RCC_RDEN1_M7LPTIM3LPEN                 (1U << 21)
#define RCC_RDEN1_M4LPTIM3LPEN                 (1U << 20)
#define RCC_RDEN1_M7LPTIM4EN                   (1U << 19)
#define RCC_RDEN1_M4LPTIM4EN                   (1U << 18)
#define RCC_RDEN1_M7LPTIM4LPEN                 (1U << 17)
#define RCC_RDEN1_M4LPTIM4LPEN                 (1U << 16)
#define RCC_RDEN1_M7LPTIM5EN                   (1U << 15)
#define RCC_RDEN1_M4LPTIM5EN                   (1U << 14)
#define RCC_RDEN1_M7LPTIM5LPEN                 (1U << 13)
#define RCC_RDEN1_M4LPTIM5LPEN                 (1U << 12)
#define RCC_RDEN1_M7LPUART1EN                  (1U << 11)
#define RCC_RDEN1_M4LPUART1EN                  (1U << 10)
#define RCC_RDEN1_M7LPUART1LPEN                (1U << 9)
#define RCC_RDEN1_M4LPUART1LPEN                (1U << 8)
#define RCC_RDEN1_M7LPUART2EN                  (1U << 7)
#define RCC_RDEN1_M4LPUART2EN                  (1U << 6)
#define RCC_RDEN1_M7LPUART2LPEN                (1U << 5)
#define RCC_RDEN1_M4LPUART2LPEN                (1U << 4)

/* Bit definition for RCC_RDEN2 register ************************************/

#define RCC_RDEN2_M7COMPEN                     (1U << 31)
#define RCC_RDEN2_M4COMPEN                     (1U << 30)
#define RCC_RDEN2_M7COMPLPEN                   (1U << 29)
#define RCC_RDEN2_M4COMPLPEN                   (1U << 28)

/* Bit definition for RCC_RDRST1 register ***********************************/

#define RCC_RDRST1_LPTIM1RST                   (1U << 28)
#define RCC_RDRST1_LPTIM2RST                   (1U << 24)
#define RCC_RDRST1_LPTIM3RST                   (1U << 20)
#define RCC_RDRST1_LPTIM4RST                   (1U << 16)
#define RCC_RDRST1_LPTIM5RST                   (1U << 12)
#define RCC_RDRST1_LPUART1RST                  (1U << 8)
#define RCC_RDRST1_LPUART2RST                  (1U << 4)

/* Bit definition for RCC_RDRST2 register ***********************************/

#define RCC_RDRST2_COMPRST                     (1U << 28)

/* Bit definition for RCC_BDCTRL register ***********************************/

#define RCC_BDCTRL_AFELSERDF                   (1U << 31)
#define RCC_BDCTRL_AFELSIRDF                   (1U << 30)
#define RCC_BDCTRL_LSELDOEN                    (1U << 28)
#define RCC_BDCTRL_LSIOVREN                    (1U << 27)
#define RCC_BDCTRL_LSIPFACK                    (1U << 26)
#define RCC_BDCTRL_LSIPFF                      (1U << 25)
#define RCC_BDCTRL_LSICSSEN                    (1U << 24)
#define RCC_BDCTRL_LSERDCNTEN                  (1U << 22)
#define RCC_BDCTRL_RTCLSFSW                    (1U << 21)
#define RCC_BDCTRL_RTCHSFSW                    (1U << 20)
#define RCC_BDCTRL_LSISECRDF                   (1U << 19)
#define RCC_BDCTRL_RTCEN                       (1U << 18)

/* RTCSEL[1:0] */
#define RCC_BDCTRL_RTCSEL_SHIFT                (16)
#define RCC_BDCTRL_RTCSEL_MASK                 (0x03 << RCC_BDCTRL_RTCSEL_SHIFT)
#define RCC_BDCTRL_RTCSEL_NONE                 (0x00 << RCC_BDCTRL_RTCSEL_SHIFT)
#define RCC_BDCTRL_RTCSEL_LSE                  (0x01 << RCC_BDCTRL_RTCSEL_SHIFT)
#define RCC_BDCTRL_RTCSEL_LSI                  (0x02 << RCC_BDCTRL_RTCSEL_SHIFT)
#define RCC_BDCTRL_RTCSEL_HSE_DIV              (0x03 << RCC_BDCTRL_RTCSEL_SHIFT)

#define RCC_BDCTRL_BORRSTEN                    (1U << 15)
#define RCC_BDCTRL_C1LPRSTEN                   (1U << 14)
#define RCC_BDCTRL_C2LPRSTEN                   (1U << 13)
#define RCC_BDCTRL_BDRST                       (1U << 12)
#define RCC_BDCTRL_BKPEMC_RSTEN                (1U << 11)
#define RCC_BDCTRL_RETEMC_RSTEN                (1U << 10)
#define RCC_BDCTRL_LSECSSF                     (1U << 9)
#define RCC_BDCTRL_LSECSSEN                    (1U << 8)
#define RCC_BDCTRL_LSERDEN                     (1U << 7)
#define RCC_BDCTRL_LSEBP                       (1U << 6)
#define RCC_BDCTRL_LSERDF                      (1U << 5)
#define RCC_BDCTRL_LSEEN                       (1U << 4)
#define RCC_BDCTRL_LSIRDEN                     (1U << 3)
#define RCC_BDCTRL_LSISECEN                    (1U << 2)
#define RCC_BDCTRL_LSIRDF                      (1U << 1)
#define RCC_BDCTRL_LSIEN                       (1U << 0)

/* Bit definition for RCC_LSICSSDL register *********************************/

/* Bit[31:0] DELAY */
#define RCC_LSICSSDL_DELAY_SHIFT               (0)
#define RCC_LSICSSDL_DELAY_MASK                (0xFFFFFFFFU << RCC_LSICSSDL_DELAY_SHIFT)

/* Bit definition for RCC_CTRLSTS register **********************************/

#define RCC_CTRLSTS_RMRSTF                     (1U << 31)
#define RCC_CTRLSTS_C1LPRSTF                   (1U << 15)
#define RCC_CTRLSTS_C2LPRSTF                   (1U << 14)
#define RCC_CTRLSTS_RETEMCRSTF                 (1U << 13)
#define RCC_CTRLSTS_BKPEMCRSTF                 (1U << 12)
#define RCC_CTRLSTS_BORRSTF                    (1U << 11)
#define RCC_CTRLSTS_MMURSTF                    (1U << 9)
#define RCC_CTRLSTS_WWDG1RSTF                  (1U << 7)
#define RCC_CTRLSTS_WWDG2RSTF                  (1U << 6)
#define RCC_CTRLSTS_IWDG1RSTF                  (1U << 5)
#define RCC_CTRLSTS_IWDG2RSTF                  (1U << 4)
#define RCC_CTRLSTS_CM4SFTRSTF                 (1U << 3)
#define RCC_CTRLSTS_CM7SFTRSTF                 (1U << 2)
#define RCC_CTRLSTS_PORRSTF                    (1U << 1)
#define RCC_CTRLSTS_PINRSTF                    (1U << 0)

/* Bit definition for RCC_CLKINT1 register **********************************/

#define RCC_CLKINT1_LSECSSIE                   (1U << 30)
#define RCC_CLKINT1_LSECSSIF                   (1U << 29)
#define RCC_CLKINT1_LSECSSIC                   (1U << 28)
#define RCC_CLKINT1_HSECSSIF                   (1U << 25)
#define RCC_CLKINT1_HSECSSIC                   (1U << 24)
#define RCC_CLKINT1_BORIE                      (1U << 22)
#define RCC_CLKINT1_BORIF                      (1U << 21)
#define RCC_CLKINT1_BORIC                      (1U << 20)
#define RCC_CLKINT1_PLL1RDIE                   (1U << 18)
#define RCC_CLKINT1_PLL1RDIF                   (1U << 17)
#define RCC_CLKINT1_PLL1RDIC                   (1U << 16)
#define RCC_CLKINT1_PLL2RDIE                   (1U << 14)
#define RCC_CLKINT1_PLL2RDIF                   (1U << 13)
#define RCC_CLKINT1_PLL2RDIC                   (1U << 12)
#define RCC_CLKINT1_PLL3RDIE                   (1U << 10)
#define RCC_CLKINT1_PLL3RDIF                   (1U << 9)
#define RCC_CLKINT1_PLL3RDIC                   (1U << 8)
#define RCC_CLKINT1_SHRPLLRDIE                 (1U << 6)
#define RCC_CLKINT1_SHRPLLRDIF                 (1U << 5)
#define RCC_CLKINT1_SHRPLLRDIC                 (1U << 4)

/* Bit definition for RCC_CLKINT2 register **********************************/

#define RCC_CLKINT2_HSERDIE                    (1U << 30)
#define RCC_CLKINT2_HSERDIF                    (1U << 29)
#define RCC_CLKINT2_HSERDIC                    (1U << 28)
#define RCC_CLKINT2_HSIRDIE                    (1U << 26)
#define RCC_CLKINT2_HSIRDIF                    (1U << 25)
#define RCC_CLKINT2_HSIRDIC                    (1U << 24)
#define RCC_CLKINT2_MSIRDIE                    (1U << 22)
#define RCC_CLKINT2_MSIRDIF                    (1U << 21)
#define RCC_CLKINT2_MSIRDIC                    (1U << 20)
#define RCC_CLKINT2_LSERDIE                    (1U << 18)
#define RCC_CLKINT2_LSERDIF                    (1U << 17)
#define RCC_CLKINT2_LSERDIC                    (1U << 16)
#define RCC_CLKINT2_LSIRDIE                    (1U << 14)
#define RCC_CLKINT2_LSIRDIF                    (1U << 13)
#define RCC_CLKINT2_LSIRDIC                    (1U << 12)
#define RCC_CLKINT2_HSICALEIE                  (1U << 10)
#define RCC_CLKINT2_HSICALEIF                  (1U << 9)
#define RCC_CLKINT2_HSICALEIC                  (1U << 8)
#define RCC_CLKINT2_MSICALEIE                  (1U << 6)
#define RCC_CLKINT2_MSICALEIF                  (1U << 5)
#define RCC_CLKINT2_MSICALEIC                  (1U << 4)

/* Bit definition for RCC_CLKINT3 register **********************************/

#define RCC_CLKINT3_PLL1LKFIEN                 (1U << 18)
#define RCC_CLKINT3_PLL1LKFIF                  (1U << 17)
#define RCC_CLKINT3_PLL1LKFIC                  (1U << 16)
#define RCC_CLKINT3_PLL2LKFIEN                 (1U << 14)
#define RCC_CLKINT3_PLL2LKFIF                  (1U << 13)
#define RCC_CLKINT3_PLL2LKFIC                  (1U << 12)
#define RCC_CLKINT3_PLL3LKFIEN                 (1U << 10)
#define RCC_CLKINT3_PLL3LKFIF                  (1U << 9)
#define RCC_CLKINT3_PLL3LKFIC                  (1U << 8)
#define RCC_CLKINT3_SHRPLLLKFIE                (1U << 6)
#define RCC_CLKINT3_SHRPLLLKFIF                (1U << 5)
#define RCC_CLKINT3_SHRPLLLKFIC                (1U << 4)
#define RCC_CLKINT3_LSIFIE                     (1U << 2)
#define RCC_CLKINT3_LSIFIF                     (1U << 1)
#define RCC_CLKINT3_LSIFIC                     (1U << 0)

/* Bit definition for RCC_CFG1 register *************************************/

#define RCC_CFG1_WWDG2RSTDLCNT_SHIFT           (24)       /* WWDG2 reset delay counter shift */
#define RCC_CFG1_WWDG2RSTDLCNT_MASK            (0x0F << RCC_CFG1_WWDG2RSTDLCNT_SHIFT)

#define RCC_CFG1_WWDG1RSTDLCNT_SHIFT           (20)       /* WWDG1 reset delay counter shift */
#define RCC_CFG1_WWDG1RSTDLCNT_MASK            (0x0F << RCC_CFG1_WWDG1RSTDLCNT_SHIFT)

#define RCC_CFG1_WWDG2RSTEN                    (1U << 19) /* WWDG2 reset enable */
#define RCC_CFG1_WWDG1RSTEN                    (1U << 18) /* WWDG1 reset enable */

#define RCC_CFG1_M7TRACEDIV_SHIFT              (12)       /* M7 trace clock divider shift */
#define RCC_CFG1_M7TRACEDIV_MASK               (0x0F << RCC_CFG1_M7TRACEDIV_SHIFT)
#define RCC_CFG1_M7TRACEDIV1                   (0U << RCC_CFG1_M7TRACEDIV_SHIFT)
#define RCC_CFG1_M7TRACEDIV2                   (1U << RCC_CFG1_M7TRACEDIV_SHIFT)
#define RCC_CFG1_M7TRACEDIV4                   (2U << RCC_CFG1_M7TRACEDIV_SHIFT)
#define RCC_CFG1_M7TRACEDIV8                   (4U << RCC_CFG1_M7TRACEDIV_SHIFT)
/* Bit definition for RCC_HSECAL register ***********************************/

/* [31:18] Reserved */

/* [17] HSE Calibration Count Enable */
#define RCC_CFG1_M7TRACEDIV16                  (7U << RCC_CFG1_M7TRACEDIV_SHIFT)
#define RCC_CFG1_M7TRACEDIV32                  (8U << RCC_CFG1_M7TRACEDIV_SHIFT)
#define RCC_CFG1_M7TRACEDIV64                  (9U << RCC_CFG1_M7TRACEDIV_SHIFT)
#define RCC_CFG1_M7TRACEDIV128                 (10U << RCC_CFG1_M7TRACEDIV_SHIFT)
#define RCC_CFG1_M7TRACEDIV256                 (11U << RCC_CFG1_M7TRACEDIV_SHIFT)
#define RCC_CFG1_M7TRACEDIV512                 (12U << RCC_CFG1_M7TRACEDIV_SHIFT)

#define RCC_CFG1_M4TRACEDIV_SHIFT              (8)        /* M4 trace clock divider shift */
#define RCC_CFG1_M4TRACEDIV_MASK               (0x0F << RCC_CFG1_M4TRACEDIV_SHIFT)
#define RCC_CFG1_M4TRACEDIV1                   (0U << RCC_CFG1_M4TRACEDIV_SHIFT)
#define RCC_CFG1_M4TRACEDIV2                   (1U << RCC_CFG1_M4TRACEDIV_SHIFT)
#define RCC_CFG1_M4TRACEDIV4                   (2U << RCC_CFG1_M4TRACEDIV_SHIFT)
#define RCC_CFG1_M4TRACEDIV8                   (4U << RCC_CFG1_M4TRACEDIV_SHIFT)
#define RCC_CFG1_M4TRACEDIV16                  (7U << RCC_CFG1_M4TRACEDIV_SHIFT)
#define RCC_CFG1_M4TRACEDIV32                  (8U << RCC_CFG1_M4TRACEDIV_SHIFT)
#define RCC_CFG1_M4TRACEDIV64                  (9U << RCC_CFG1_M4TRACEDIV_SHIFT)
#define RCC_CFG1_M4TRACEDIV128                 (10U << RCC_CFG1_M4TRACEDIV_SHIFT)
#define RCC_CFG1_M4TRACEDIV256                 (11U << RCC_CFG1_M4TRACEDIV_SHIFT)
#define RCC_CFG1_M4TRACEDIV512                 (12U << RCC_CFG1_M4TRACEDIV_SHIFT)

/* Bit definition for RCC_AXIDIV1 register **********************************/

#define RCC_AXIDIV1_SDMMC1AXIDIV_SHIFT         (24)  /* SDMMC1 AXI clock divider shift position */
#define RCC_AXIDIV1_SDMMC1AXIDIV_MASK          (0x0F << RCC_AXIDIV1_SDMMC1AXIDIV_SHIFT)
#define RCC_AXIDIV1_SDMMC1AXIDIV1              (0U << RCC_AXIDIV1_SDMMC1AXIDIV_SHIFT)
#define RCC_AXIDIV1_SDMMC1AXIDIV2              (1U << RCC_AXIDIV1_SDMMC1AXIDIV_SHIFT)
#define RCC_AXIDIV1_SDMMC1AXIDIV4              (2U << RCC_AXIDIV1_SDMMC1AXIDIV_SHIFT)
#define RCC_AXIDIV1_SDMMC1AXIDIV8              (4U << RCC_AXIDIV1_SDMMC1AXIDIV_SHIFT)
#define RCC_AXIDIV1_SDMMC1AXIDIV16             (7U << RCC_AXIDIV1_SDMMC1AXIDIV_SHIFT)
#define RCC_AXIDIV1_SDMMC1AXIDIV32             (8U << RCC_AXIDIV1_SDMMC1AXIDIV_SHIFT)
#define RCC_AXIDIV1_SDMMC1AXIDIV64             (9U << RCC_AXIDIV1_SDMMC1AXIDIV_SHIFT)
#define RCC_AXIDIV1_SDMMC1AXIDIV128            (10U << RCC_AXIDIV1_SDMMC1AXIDIV_SHIFT)
#define RCC_AXIDIV1_SDMMC1AXIDIV256            (11U << RCC_AXIDIV1_SDMMC1AXIDIV_SHIFT)
#define RCC_AXIDIV1_SDMMC1AXIDIV512            (12U << RCC_AXIDIV1_SDMMC1AXIDIV_SHIFT)

#define RCC_AXIDIV1_DSIREFDIV_SHIFT            (16)  /* DSI reference clock divider shift position */
#define RCC_AXIDIV1_DSIREFDIV_MASK             (0x0F << RCC_AXIDIV1_DSIREFDIV_SHIFT)
#define RCC_AXIDIV1_DSIREFDIV1                 (0U << RCC_AXIDIV1_DSIREFDIV_SHIFT)
#define RCC_AXIDIV1_DSIREFDIV2                 (1U << RCC_AXIDIV1_DSIREFDIV_SHIFT)
#define RCC_AXIDIV1_DSIREFDIV4                 (2U << RCC_AXIDIV1_DSIREFDIV_SHIFT)
#define RCC_AXIDIV1_DSIREFDIV8                 (4U << RCC_AXIDIV1_DSIREFDIV_SHIFT)
#define RCC_AXIDIV1_DSIREFDIV16                (7U << RCC_AXIDIV1_DSIREFDIV_SHIFT)
#define RCC_AXIDIV1_DSIREFDIV32                (8U << RCC_AXIDIV1_DSIREFDIV_SHIFT)
#define RCC_AXIDIV1_DSIREFDIV64                (9U << RCC_AXIDIV1_DSIREFDIV_SHIFT)
#define RCC_AXIDIV1_DSIREFDIV128               (10U << RCC_AXIDIV1_DSIREFDIV_SHIFT)
#define RCC_AXIDIV1_DSIREFDIV256               (11U << RCC_AXIDIV1_DSIREFDIV_SHIFT)
#define RCC_AXIDIV1_DSIREFDIV512               (12U << RCC_AXIDIV1_DSIREFDIV_SHIFT)

#define RCC_AXIDIV1_LCDAXIDIV_SHIFT            (12)  /* LCD AXI clock divider shift position */
#define RCC_AXIDIV1_LCDAXIDIV_MASK             (0x0F << RCC_AXIDIV1_LCDAXIDIV_SHIFT)
#define RCC_AXIDIV1_LCDAXIDIV1                 (0U << RCC_AXIDIV1_LCDAXIDIV_SHIFT)
#define RCC_AXIDIV1_LCDAXIDIV2                 (1U << RCC_AXIDIV1_LCDAXIDIV_SHIFT)
#define RCC_AXIDIV1_LCDAXIDIV4                 (2U << RCC_AXIDIV1_LCDAXIDIV_SHIFT)
#define RCC_AXIDIV1_LCDAXIDIV8                 (4U << RCC_AXIDIV1_LCDAXIDIV_SHIFT)
#define RCC_AXIDIV1_LCDAXIDIV16                (7U << RCC_AXIDIV1_LCDAXIDIV_SHIFT)
#define RCC_AXIDIV1_LCDAXIDIV32                (8U << RCC_AXIDIV1_LCDAXIDIV_SHIFT)
#define RCC_AXIDIV1_LCDAXIDIV64                (9U << RCC_AXIDIV1_LCDAXIDIV_SHIFT)
#define RCC_AXIDIV1_LCDAXIDIV128               (10U << RCC_AXIDIV1_LCDAXIDIV_SHIFT)
#define RCC_AXIDIV1_LCDAXIDIV256               (11U << RCC_AXIDIV1_LCDAXIDIV_SHIFT)
#define RCC_AXIDIV1_LCDAXIDIV512               (12U << RCC_AXIDIV1_LCDAXIDIV_SHIFT)

#define RCC_AXIDIV1_DVPMAXIDIV_SHIFT           (8)   /* DVP AXI clock divider shift position */
#define RCC_AXIDIV1_DVPMAXIDIV_MASK            (0x0F << RCC_AXIDIV1_DVPMAXIDIV_SHIFT)
#define RCC_AXIDIV1_DVPMAXIDIV1                (0U << RCC_AXIDIV1_DVPMAXIDIV_SHIFT)
#define RCC_AXIDIV1_DVPMAXIDIV2                (1U << RCC_AXIDIV1_DVPMAXIDIV_SHIFT)
#define RCC_AXIDIV1_DVPMAXIDIV4                (2U << RCC_AXIDIV1_DVPMAXIDIV_SHIFT)
#define RCC_AXIDIV1_DVPMAXIDIV8                (4U << RCC_AXIDIV1_DVPMAXIDIV_SHIFT)
#define RCC_AXIDIV1_DVPMAXIDIV16               (7U << RCC_AXIDIV1_DVPMAXIDIV_SHIFT)
#define RCC_AXIDIV1_DVPMAXIDIV32               (8U << RCC_AXIDIV1_DVPMAXIDIV_SHIFT)
#define RCC_AXIDIV1_DVPMAXIDIV64               (9U << RCC_AXIDIV1_DVPMAXIDIV_SHIFT)
#define RCC_AXIDIV1_DVPMAXIDIV128              (10U << RCC_AXIDIV1_DVPMAXIDIV_SHIFT)
#define RCC_AXIDIV1_DVPMAXIDIV256              (11U << RCC_AXIDIV1_DVPMAXIDIV_SHIFT)
#define RCC_AXIDIV1_DVPMAXIDIV512              (12U << RCC_AXIDIV1_DVPMAXIDIV_SHIFT)

/* Bit definition for RCC_AXIDIV2 register **********************************/

#define RCC_AXIDIV2_DSIAXIPPIDIV_SHIFT         (24)  /* DSI AXI PP clock divider position */
#define RCC_AXIDIV2_DSIAXIPPIDIV_MASK          (0x0F << RCC_AXIDIV2_DSIAXIPPIDIV_SHIFT)
#define RCC_AXIDIV2_DSIAXIPPIDIV1              (0U << RCC_AXIDIV2_DSIAXIPPIDIV_SHIFT)
#define RCC_AXIDIV2_DSIAXIPPIDIV2              (1U << RCC_AXIDIV2_DSIAXIPPIDIV_SHIFT)
#define RCC_AXIDIV2_DSIAXIPPIDIV4              (2U << RCC_AXIDIV2_DSIAXIPPIDIV_SHIFT)
#define RCC_AXIDIV2_DSIAXIPPIDIV8              (4U << RCC_AXIDIV2_DSIAXIPPIDIV_SHIFT)
#define RCC_AXIDIV2_DSIAXIPPIDIV16             (7U << RCC_AXIDIV2_DSIAXIPPIDIV_SHIFT)
#define RCC_AXIDIV2_DSIAXIPPIDIV32             (8U << RCC_AXIDIV2_DSIAXIPPIDIV_SHIFT)
#define RCC_AXIDIV2_DSIAXIPPIDIV64             (9U << RCC_AXIDIV2_DSIAXIPPIDIV_SHIFT)
#define RCC_AXIDIV2_DSIAXIPPIDIV128            (10U << RCC_AXIDIV2_DSIAXIPPIDIV_SHIFT)
#define RCC_AXIDIV2_DSIAXIPPIDIV256            (11U << RCC_AXIDIV2_DSIAXIPPIDIV_SHIFT)
#define RCC_AXIDIV2_DSIAXIPPIDIV512            (12U << RCC_AXIDIV2_DSIAXIPPIDIV_SHIFT)

#define RCC_AXIDIV2_DSIREFULPSDIV_SHIFT        (16)  /* DSI reference ULPS clock divider position */
#define RCC_AXIDIV2_DSIREFULPSDIV_MASK         (0x0F << RCC_AXIDIV2_DSIREFULPSDIV_SHIFT)
#define RCC_AXIDIV2_DSIREFULPSDIV1             (0U << RCC_AXIDIV2_DSIREFULPSDIV_SHIFT)
#define RCC_AXIDIV2_DSIREFULPSDIV2             (1U << RCC_AXIDIV2_DSIREFULPSDIV_SHIFT)
#define RCC_AXIDIV2_DSIREFULPSDIV4             (2U << RCC_AXIDIV2_DSIREFULPSDIV_SHIFT)
#define RCC_AXIDIV2_DSIREFULPSDIV8             (4U << RCC_AXIDIV2_DSIREFULPSDIV_SHIFT)
#define RCC_AXIDIV2_DSIREFULPSDIV16            (7U << RCC_AXIDIV2_DSIREFULPSDIV_SHIFT)
#define RCC_AXIDIV2_DSIREFULPSDIV32            (8U << RCC_AXIDIV2_DSIREFULPSDIV_SHIFT)
#define RCC_AXIDIV2_DSIREFULPSDIV64            (9U << RCC_AXIDIV2_DSIREFULPSDIV_SHIFT)
#define RCC_AXIDIV2_DSIREFULPSDIV128           (10U << RCC_AXIDIV2_DSIREFULPSDIV_SHIFT)
#define RCC_AXIDIV2_DSIREFULPSDIV256           (11U << RCC_AXIDIV2_DSIREFULPSDIV_SHIFT)
#define RCC_AXIDIV2_DSIREFULPSDIV512           (12U << RCC_AXIDIV2_DSIREFULPSDIV_SHIFT)

#define RCC_AXIDIV2_SDRAMMEMDIV_SHIFT          (8)   /* SDRAM memory clock divider position */
#define RCC_AXIDIV2_SDRAMMEMDIV_MASK           (0x0F << RCC_AXIDIV2_SDRAMMEMDIV_SHIFT)
#define RCC_AXIDIV2_SDRAMMEMDIV1               (0U << RCC_AXIDIV2_SDRAMMEMDIV_SHIFT)
#define RCC_AXIDIV2_SDRAMMEMDIV2               (1U << RCC_AXIDIV2_SDRAMMEMDIV_SHIFT)
#define RCC_AXIDIV2_SDRAMMEMDIV4               (2U << RCC_AXIDIV2_SDRAMMEMDIV_SHIFT)
#define RCC_AXIDIV2_SDRAMMEMDIV8               (4U << RCC_AXIDIV2_SDRAMMEMDIV_SHIFT)
#define RCC_AXIDIV2_SDRAMMEMDIV16              (7U << RCC_AXIDIV2_SDRAMMEMDIV_SHIFT)
#define RCC_AXIDIV2_SDRAMMEMDIV32              (8U << RCC_AXIDIV2_SDRAMMEMDIV_SHIFT)
#define RCC_AXIDIV2_SDRAMMEMDIV64              (9U << RCC_AXIDIV2_SDRAMMEMDIV_SHIFT)
#define RCC_AXIDIV2_SDRAMMEMDIV128             (10U << RCC_AXIDIV2_SDRAMMEMDIV_SHIFT)
#define RCC_AXIDIV2_SDRAMMEMDIV256             (11U << RCC_AXIDIV2_SDRAMMEMDIV_SHIFT)
#define RCC_AXIDIV2_SDRAMMEMDIV512             (12U << RCC_AXIDIV2_SDRAMMEMDIV_SHIFT)

#define RCC_AXIDIV2_FEMCM1AXIDIV_SHIFT         (4)   /* FEMC M1 AXI clock divider position */
#define RCC_AXIDIV2_FEMCM1AXIDIV_MASK          (0x0F << RCC_AXIDIV2_FEMCM1AXIDIV_SHIFT)

#define RCC_AXIDIV2_FEMCM0AXIDIV_SHIFT         (0)   /* FEMC M0 AXI clock divider position */
#define RCC_AXIDIV2_FEMCM0AXIDIV_MASK          (0x0F << RCC_AXIDIV2_FEMCM0AXIDIV_SHIFT)

/* Bit definition for RCC_AXISEL1 register **********************************/

/* DSI ULPS Clock Source Selection */
#define RCC_AXISEL1_DSIULPSSEL_SHIFT           (26)
#define RCC_AXISEL1_DSIULPSSEL_MASK            (0x03 << RCC_AXISEL1_DSIULPSSEL_SHIFT)
#define RCC_AXISEL1_DSIULPSSEL_DSIREF          (0x00 << RCC_AXISEL1_DSIULPSSEL_SHIFT)
#define RCC_AXISEL1_DSIULPSSEL_PLL3C           (0x01 << RCC_AXISEL1_DSIULPSSEL_SHIFT)

/* DSI Kernel Clock Source Selection */
#define RCC_AXISEL1_DSIKERSEL_SHIFT            (24)
#define RCC_AXISEL1_DSIKERSEL_MASK             (0x03 << RCC_AXISEL1_DSIKERSEL_SHIFT)
#define RCC_AXISEL1_DSIKERSEL_DSIREF           (0x00 << RCC_AXISEL1_DSIKERSEL_SHIFT)
#define RCC_AXISEL1_DSIKERSEL_PLL3C            (0x01 << RCC_AXISEL1_DSIKERSEL_SHIFT)

/* SDMMC1 Kernel Clock Source Selection */
#define RCC_AXISEL1_SDMMC1KERSEL_SHIFT         (20)
#define RCC_AXISEL1_SDMMC1KERSEL_MASK          (0x07 << RCC_AXISEL1_SDMMC1KERSEL_SHIFT)
#define RCC_AXISEL1_SDMMC1KERSEL_AXIDIV        (0x00 << RCC_AXISEL1_SDMMC1KERSEL_SHIFT)
#define RCC_AXISEL1_SDMMC1KERSEL_PERIPH        (0x01 << RCC_AXISEL1_SDMMC1KERSEL_SHIFT)
#define RCC_AXISEL1_SDMMC1KERSEL_PLL2A         (0x02 << RCC_AXISEL1_SDMMC1KERSEL_SHIFT)
#define RCC_AXISEL1_SDMMC1KERSEL_PLL3A         (0x03 << RCC_AXISEL1_SDMMC1KERSEL_SHIFT)
#define RCC_AXISEL1_SDMMC1KERSEL_PLL1B         (0x04 << RCC_AXISEL1_SDMMC1KERSEL_SHIFT)

/* DSI PPI TX Clock Source Selection */
#define RCC_AXISEL1_DSIPPITXSEL_SHIFT          (16)
#define RCC_AXISEL1_DSIPPITXSEL_MASK           (0x03 << RCC_AXISEL1_DSIPPITXSEL_SHIFT)
#define RCC_AXISEL1_DSIPPITXSEL_DSIDIV         (0x00 << RCC_AXISEL1_DSIPPITXSEL_SHIFT)
#define RCC_AXISEL1_DSIPPITXSEL_PLL2B          (0x01 << RCC_AXISEL1_DSIPPITXSEL_SHIFT)
#define RCC_AXISEL1_DSIPPITXSEL_PERIPH         (0x02 << RCC_AXISEL1_DSIPPITXSEL_SHIFT)
#define RCC_AXISEL1_DSIPPITXSEL_AXIDIV         (0x03 << RCC_AXISEL1_DSIPPITXSEL_SHIFT)

/* LCD Kernel Clock Source Selection */
#define RCC_AXISEL1_LCDKERSEL_SHIFT            (12)
#define RCC_AXISEL1_LCDKERSEL_MASK             (0x03 << RCC_AXISEL1_LCDKERSEL_SHIFT)
#define RCC_AXISEL1_LCDKERSEL_AXIDIV           (0x00 << RCC_AXISEL1_LCDKERSEL_SHIFT)
#define RCC_AXISEL1_LCDKERSEL_PERIPH           (0x01 << RCC_AXISEL1_LCDKERSEL_SHIFT)
#define RCC_AXISEL1_LCDKERSEL_PLL2C            (0x02 << RCC_AXISEL1_LCDKERSEL_SHIFT)
#define RCC_AXISEL1_LCDKERSEL_PLL3B            (0x03 << RCC_AXISEL1_LCDKERSEL_SHIFT)

/* DVP1 Memory Clock Source Selection */
#define RCC_AXISEL1_DVP1MSEL_SHIFT             (10)
#define RCC_AXISEL1_DVP1MSEL_MASK              (0x03 << RCC_AXISEL1_DVP1MSEL_SHIFT)
#define RCC_AXISEL1_DVP1MSEL_AXIDIV            (0x00 << RCC_AXISEL1_DVP1MSEL_SHIFT)
#define RCC_AXISEL1_DVP1MSEL_PERIPH            (0x01 << RCC_AXISEL1_DVP1MSEL_SHIFT)
#define RCC_AXISEL1_DVP1MSEL_PLL2C             (0x02 << RCC_AXISEL1_DVP1MSEL_SHIFT)
#define RCC_AXISEL1_DVP1MSEL_PLL3A             (0x03 << RCC_AXISEL1_DVP1MSEL_SHIFT)

/* DVP2 Memory Clock Source Selection */
#define RCC_AXISEL1_DVP2MSEL_SHIFT             (8)
#define RCC_AXISEL1_DVP2MSEL_MASK              (0x03 << RCC_AXISEL1_DVP2MSEL_SHIFT)
#define RCC_AXISEL1_DVP2MSEL_AXIDIV            (0x00 << RCC_AXISEL1_DVP2MSEL_SHIFT)
#define RCC_AXISEL1_DVP2MSEL_PERIPH            (0x01 << RCC_AXISEL1_DVP2MSEL_SHIFT)
#define RCC_AXISEL1_DVP2MSEL_PLL2C             (0x02 << RCC_AXISEL1_DVP2MSEL_SHIFT)
#define RCC_AXISEL1_DVP2MSEL_PLL3A             (0x03 << RCC_AXISEL1_DVP2MSEL_SHIFT)

/* XSPI1 SSI Clock Source Selection */
#define RCC_AXISEL1_XSPI1SSISEL_SHIFT          (4)
#define RCC_AXISEL1_XSPI1SSISEL_MASK           (0x07 << RCC_AXISEL1_XSPI1SSISEL_SHIFT)
#define RCC_AXISEL1_XSPI1SSISEL_AXI            (0x00 << RCC_AXISEL1_XSPI1SSISEL_SHIFT)
#define RCC_AXISEL1_XSPI1SSISEL_PLL3C          (0x01 << RCC_AXISEL1_XSPI1SSISEL_SHIFT)
#define RCC_AXISEL1_XSPI1SSISEL_PLL1B          (0x02 << RCC_AXISEL1_XSPI1SSISEL_SHIFT)
#define RCC_AXISEL1_XSPI1SSISEL_PLL2A          (0x03 << RCC_AXISEL1_XSPI1SSISEL_SHIFT)
#define RCC_AXISEL1_XSPI1SSISEL_PLL2C          (0x04 << RCC_AXISEL1_XSPI1SSISEL_SHIFT)

/* XSPI2 SSI Clock Source Selection */
#define RCC_AXISEL1_XSPI2SSISEL_SHIFT          (0)
#define RCC_AXISEL1_XSPI2SSISEL_MASK           (0x07 << RCC_AXISEL1_XSPI2SSISEL_SHIFT)
#define RCC_AXISEL1_XSPI2SSISEL_AXI            (0x00 << RCC_AXISEL1_XSPI2SSISEL_SHIFT)
#define RCC_AXISEL1_XSPI2SSISEL_PLL3C          (0x01 << RCC_AXISEL1_XSPI2SSISEL_SHIFT)
#define RCC_AXISEL1_XSPI2SSISEL_PLL1B          (0x02 << RCC_AXISEL1_XSPI2SSISEL_SHIFT)
#define RCC_AXISEL1_XSPI2SSISEL_PLL2A          (0x03 << RCC_AXISEL1_XSPI2SSISEL_SHIFT)
#define RCC_AXISEL1_XSPI2SSISEL_PLL2C          (0x04 << RCC_AXISEL1_XSPI2SSISEL_SHIFT)

/* Bit definition for RCC_AXISEL2 register **********************************/

/* Bit[10:8] SDRAMMEMSEL */
#define RCC_AXISEL2_SDRAMMEMSEL_SHIFT          (8)
#define RCC_AXISEL2_SDRAMMEMSEL_MASK           (0x07 << RCC_AXISEL2_SDRAMMEMSEL_SHIFT)
#define RCC_AXISEL2_SDRAMMEMSEL_AXIDIV         (0x00 << RCC_AXISEL2_SDRAMMEMSEL_SHIFT)
#define RCC_AXISEL2_SDRAMMEMSEL_PERIPH         (0x01 << RCC_AXISEL2_SDRAMMEMSEL_SHIFT)
#define RCC_AXISEL2_SDRAMMEMSEL_PLL2A          (0x02 << RCC_AXISEL2_SDRAMMEMSEL_SHIFT)
#define RCC_AXISEL2_SDRAMMEMSEL_PLL3A          (0x03 << RCC_AXISEL2_SDRAMMEMSEL_SHIFT)
#define RCC_AXISEL2_SDRAMMEMSEL_PLL1B          (0x04 << RCC_AXISEL2_SDRAMMEMSEL_SHIFT)

/* Bit[6:4] FEMCM1SEL */
#define RCC_AXISEL2_FEMCM1SEL_SHIFT            (4)
#define RCC_AXISEL2_FEMCM1SEL_MASK             (0x07 << RCC_AXISEL2_FEMCM1SEL_SHIFT)
#define RCC_AXISEL2_FEMCM1SEL_AXIDIV           (0x00 << RCC_AXISEL2_FEMCM1SEL_SHIFT)
#define RCC_AXISEL2_FEMCM1SEL_PERIPH           (0x01 << RCC_AXISEL2_FEMCM1SEL_SHIFT)
#define RCC_AXISEL2_FEMCM1SEL_PLL2C            (0x02 << RCC_AXISEL2_FEMCM1SEL_SHIFT)
#define RCC_AXISEL2_FEMCM1SEL_PLL3B            (0x03 << RCC_AXISEL2_FEMCM1SEL_SHIFT)
#define RCC_AXISEL2_FEMCM1SEL_PLL1B            (0x04 << RCC_AXISEL2_FEMCM1SEL_SHIFT)

/* Bit[2:0] FEMCM0SEL */
#define RCC_AXISEL2_FEMCM0SEL_SHIFT            (0)
#define RCC_AXISEL2_FEMCM0SEL_MASK             (0x07 << RCC_AXISEL2_FEMCM0SEL_SHIFT)
#define RCC_AXISEL2_FEMCM0SEL_AXIDIV           (0x00 << RCC_AXISEL2_FEMCM0SEL_SHIFT)
#define RCC_AXISEL2_FEMCM0SEL_PERIPH           (0x01 << RCC_AXISEL2_FEMCM0SEL_SHIFT)
#define RCC_AXISEL2_FEMCM0SEL_PLL2C            (0x02 << RCC_AXISEL2_FEMCM0SEL_SHIFT)
#define RCC_AXISEL2_FEMCM0SEL_PLL3B            (0x03 << RCC_AXISEL2_FEMCM0SEL_SHIFT)
#define RCC_AXISEL2_FEMCM0SEL_PLL1B            (0x04 << RCC_AXISEL2_FEMCM0SEL_SHIFT)

/* Bit definition for RCC_AXIEN1 register ***********************************/

#define RCC_AXIEN1_M7JPEGDEN                   (1U << 31)
#define RCC_AXIEN1_M4JPEGDEN                   (1U << 30)
#define RCC_AXIEN1_M7JPEGDLPEN                 (1U << 29)
#define RCC_AXIEN1_M4JPEGDLPEN                 (1U << 28)
#define RCC_AXIEN1_M7JPEGEEN                   (1U << 23)
#define RCC_AXIEN1_M4JPEGEEN                   (1U << 22)
#define RCC_AXIEN1_M7JPEGELPEN                 (1U << 21)
#define RCC_AXIEN1_M4JPEGELPEN                 (1U << 20)
#define RCC_AXIEN1_M7DMAMUX2EN                 (1U << 19)
#define RCC_AXIEN1_M4DMAMUX2EN                 (1U << 18)
#define RCC_AXIEN1_M7DMAMUX2LPEN               (1U << 17)
#define RCC_AXIEN1_M4DMAMUX2LPEN               (1U << 16)
#define RCC_AXIEN1_M7MDMAEN                    (1U << 15)
#define RCC_AXIEN1_M4MDMAEN                    (1U << 14)
#define RCC_AXIEN1_M7MDMALPEN                  (1U << 13)
#define RCC_AXIEN1_M4MDMALPEN                  (1U << 12)
#define RCC_AXIEN1_M7SDMMC1EN                  (1U << 11)
#define RCC_AXIEN1_M4SDMMC1EN                  (1U << 10)
#define RCC_AXIEN1_M7SDMMC1LPEN                (1U << 9)
#define RCC_AXIEN1_M4SDMMC1LPEN                (1U << 8)
#define RCC_AXIEN1_M7ECCM1EN                   (1U << 7)
#define RCC_AXIEN1_M4ECCM1EN                   (1U << 6)
#define RCC_AXIEN1_M7ECCM1LPEN                 (1U << 5)
#define RCC_AXIEN1_M4ECCM1LPEN                 (1U << 4)
#define RCC_AXIEN1_M7OTPCEN                    (1U << 3)
#define RCC_AXIEN1_M4OTPCEN                    (1U << 2)
#define RCC_AXIEN1_M7OTPCLPEN                  (1U << 1)
#define RCC_AXIEN1_M4OTPCLPEN                  (1U << 0)

/* Bit definition for RCC_AXIEN2 register ***********************************/

#define RCC_AXIEN2_M7DSIEN                     (1U << 31)
#define RCC_AXIEN2_M4DSIEN                     (1U << 30)
#define RCC_AXIEN2_M7DSILPEN                   (1U << 29)
#define RCC_AXIEN2_M4DSILPEN                   (1U << 28)
#define RCC_AXIEN2_M7LCDEN                     (1U << 27)
#define RCC_AXIEN2_M4LCDEN                     (1U << 26)
#define RCC_AXIEN2_M7LCDLPEN                   (1U << 25)
#define RCC_AXIEN2_M4LCDLPEN                   (1U << 24)
#define RCC_AXIEN2_M7LCDAPBEN                  (1U << 23)
#define RCC_AXIEN2_M4LCDAPBEN                  (1U << 22)
#define RCC_AXIEN2_M7LCDAPBLPEN                (1U << 21)
#define RCC_AXIEN2_M4LCDAPBLPEN                (1U << 20)
#define RCC_AXIEN2_M7DVP1EN                    (1U << 19)
#define RCC_AXIEN2_M4DVP1EN                    (1U << 18)
#define RCC_AXIEN2_M7DVP1LPEN                  (1U << 17)
#define RCC_AXIEN2_M4DVP1LPEN                  (1U << 16)
#define RCC_AXIEN2_M7DVP1APBEN                 (1U << 15)
#define RCC_AXIEN2_M4DVP1APBEN                 (1U << 14)
#define RCC_AXIEN2_M7DVP1APBLPEN               (1U << 13)
#define RCC_AXIEN2_M4DVP1APBLPEN               (1U << 12)
#define RCC_AXIEN2_M7DVP2EN                    (1U << 11)
#define RCC_AXIEN2_M4DVP2EN                    (1U << 10)
#define RCC_AXIEN2_M7DVP2LPEN                  (1U << 9)
#define RCC_AXIEN2_M4DVP2LPEN                  (1U << 8)
#define RCC_AXIEN2_M7DVP2APBEN                 (1U << 7)
#define RCC_AXIEN2_M4DVP2APBEN                 (1U << 6)
#define RCC_AXIEN2_M7DVP2APBLPEN               (1U << 5)
#define RCC_AXIEN2_M4DVP2APBLPEN               (1U << 4)
#define RCC_AXIEN2_M7WWDG1EN                   (1U << 3)
#define RCC_AXIEN2_M4WWDG1EN                   (1U << 2)
#define RCC_AXIEN2_M7WWDG1LPEN                 (1U << 1)
#define RCC_AXIEN2_M4WWDG1LPEN                 (1U << 0)

/* Bit definition for RCC_AXIEN3 register ***********************************/

#define RCC_AXIEN3_M7TASRAM2EN                 (1U << 31)
#define RCC_AXIEN3_M4TASRAM2EN                 (1U << 30)
#define RCC_AXIEN3_M7TASRAM2LPEN               (1U << 29)
#define RCC_AXIEN3_M4TASRAM2LPEN               (1U << 28)
#define RCC_AXIEN3_M7TASRAM3EN                 (1U << 27)
#define RCC_AXIEN3_M4TASRAM3EN                 (1U << 26)
#define RCC_AXIEN3_M7TASRAM3LPEN               (1U << 25)
#define RCC_AXIEN3_M4TASRAM3LPEN               (1U << 24)
#define RCC_AXIEN3_M7TCMEN                     (1U << 23)
#define RCC_AXIEN3_M4TCMEN                     (1U << 22)
#define RCC_AXIEN3_M7TCMLPEN                   (1U << 21)
#define RCC_AXIEN3_M4TCMLPEN                   (1U << 20)
#define RCC_AXIEN3_M7TCMAXIEN                  (1U << 19)
#define RCC_AXIEN3_M4TCMAXIEN                  (1U << 18)
#define RCC_AXIEN3_M7TCMAXILPEN                (1U << 17)
#define RCC_AXIEN3_M4TCMAXILPEN                (1U << 16)
#define RCC_AXIEN3_M7TCMAPBEN                  (1U << 15)
#define RCC_AXIEN3_M4TCMAPBEN                  (1U << 14)
#define RCC_AXIEN3_M7TCMAPBLPEN                (1U << 13)
#define RCC_AXIEN3_M4TCMAPBLPEN                (1U << 12)
#define RCC_AXIEN3_M7ASRAM1EN                  (1U << 11)
#define RCC_AXIEN3_M4ASRAM1EN                  (1U << 10)
#define RCC_AXIEN3_M7ASRAM1LPEN                (1U << 9)
#define RCC_AXIEN3_M4ASRAM1LPEN                (1U << 8)
#define RCC_AXIEN3_M7AXIROMEN                  (1U << 7)
#define RCC_AXIEN3_M4AXIROMEN                  (1U << 6)
#define RCC_AXIEN3_M7AXIROMLPEN                (1U << 5)
#define RCC_AXIEN3_M4AXIROMLPEN                (1U << 4)
#define RCC_AXIEN3_M7GPUEN                     (1U << 3)
#define RCC_AXIEN3_M4GPUEN                     (1U << 2)
#define RCC_AXIEN3_M7GPULPEN                   (1U << 1)
#define RCC_AXIEN3_M4GPULPEN                   (1U << 0)

/* Bit definition for RCC_AXIEN4 register ***********************************/

#define RCC_AXIEN4_M7XSPI1EN                   (1U << 31)
#define RCC_AXIEN4_M4XSPI1EN                   (1U << 30)
#define RCC_AXIEN4_M7XSPI1LPEN                 (1U << 29)
#define RCC_AXIEN4_M4XSPI1LPEN                 (1U << 28)
#define RCC_AXIEN4_M7XSPI2EN                   (1U << 27)
#define RCC_AXIEN4_M4XSPI2EN                   (1U << 26)
#define RCC_AXIEN4_M7XSPI2LPEN                 (1U << 25)
#define RCC_AXIEN4_M4XSPI2LPEN                 (1U << 24)
#define RCC_AXIEN4_M7FEMCEN                    (1U << 23)
#define RCC_AXIEN4_M4FEMCEN                    (1U << 22)
#define RCC_AXIEN4_M7FEMCLPEN                  (1U << 21)
#define RCC_AXIEN4_M4FEMCLPEN                  (1U << 20)
#define RCC_AXIEN4_M7SDRAMEN                   (1U << 19)
#define RCC_AXIEN4_M4SDRAMEN                   (1U << 18)
#define RCC_AXIEN4_M7SDRAMLPEN                 (1U << 17)
#define RCC_AXIEN4_M4SDRAMLPEN                 (1U << 16)
#define RCC_AXIEN4_M7DSIULPSEN                 (1U << 3)
#define RCC_AXIEN4_M4DSIULPSEN                 (1U << 2)
#define RCC_AXIEN4_M7DSIULPSLPEN               (1U << 1)
#define RCC_AXIEN4_M4DSIULPSLPEN               (1U << 0)

/* Bit definition for RCC_AXIRST1 register **********************************/

#define RCC_AXIRST1_JPEGDRST                   (1U << 28)
#define RCC_AXIRST1_JPEGERST                   (1U << 20)
#define RCC_AXIRST1_DMAMUX2RST                 (1U << 16)
#define RCC_AXIRST1_MDMARST                    (1U << 12)
#define RCC_AXIRST1_SDMMC1RST                  (1U << 9)
#define RCC_AXIRST1_SDHOST1RST                 (1U << 8)
#define RCC_AXIRST1_ECCM1RST                   (1U << 4)
#define RCC_AXIRST1_OTPCRST                    (1U << 0)

/* Bit definition for RCC_AXIRST2 register **********************************/

#define RCC_AXIRST2_DSICFGRST                  (1U << 29)
#define RCC_AXIRST2_DSIRST                     (1U << 28)
#define RCC_AXIRST2_LCDRST                     (1U << 24)
#define RCC_AXIRST2_DVP1RST                    (1U << 16)
#define RCC_AXIRST2_DVP2RST                    (1U << 8)
#define RCC_AXIRST2_WWDG1RST                   (1U << 0)

/* Bit definition for RCC_AXIRST3 register **********************************/

#define RCC_AXIRST3_GPURST                     (1U << 0)

/* Bit definition for RCC_AXIRST4 register **********************************/

#define RCC_AXIRST4_XSPI1RST                   (1U << 28)
#define RCC_AXIRST4_XSPI2RST                   (1U << 24)
#define RCC_AXIRST4_FEMCCFGRST                 (1U << 21)
#define RCC_AXIRST4_FEMCRST                    (1U << 20)
#define RCC_AXIRST4_SDRAMRST                   (1U << 16)

/* Bit definition for RCC_CFG2 register *************************************/

/* Cache & MMU */
#define RCC_CFG2_M4CAHIEN                      (1U << 30)
#define RCC_CFG2_M4CAHIPCLKEN                  (1U << 28)
#define RCC_CFG2_M4CAHDEN                      (1U << 26)
#define RCC_CFG2_M4CAHDPCLKEN                  (1U << 24)

#define RCC_CFG2_M7MMUEN                       (1U << 23)
#define RCC_CFG2_M7MMULPEN                     (1U << 22)
#define RCC_CFG2_M4MMUEN                       (1U << 21)
#define RCC_CFG2_M4MMULPEN                     (1U << 20)

/* BKP SRAM */
#define RCC_CFG2_M7SRAMBKPEN                   (1U << 19)
#define RCC_CFG2_M4SRAMBKPEN                   (1U << 18)
#define RCC_CFG2_M7SRAMBKPLPEN                 (1U << 17)
#define RCC_CFG2_M4SRAMBKPLPEN                 (1U << 16)

/* SRAM1~4 */
#define RCC_CFG2_M7SRAM1EN                     (1U << 15)
#define RCC_CFG2_M4SRAM1EN                     (1U << 14)
#define RCC_CFG2_M7SRAM1LPEN                   (1U << 13)
#define RCC_CFG2_M4SRAM1LPEN                   (1U << 12)

#define RCC_CFG2_M7SRAM2EN                     (1U << 11)
#define RCC_CFG2_M4SRAM2EN                     (1U << 10)
#define RCC_CFG2_M7SRAM2LPEN                   (1U << 9)
#define RCC_CFG2_M4SRAM2LPEN                   (1U << 8)

#define RCC_CFG2_M7SRAM3EN                     (1U << 7)
#define RCC_CFG2_M4SRAM3EN                     (1U << 6)
#define RCC_CFG2_M7SRAM3LPEN                   (1U << 5)
#define RCC_CFG2_M4SRAM3LPEN                   (1U << 4)

#define RCC_CFG2_M7SRAM4EN                     (1U << 3)
#define RCC_CFG2_M4SRAM4EN                     (1U << 2)
#define RCC_CFG2_M7SRAM4LPEN                   (1U << 1)
#define RCC_CFG2_M4SRAM4LPEN                   (1U << 0)

/* Bit definition for RCC_CFG3 register *************************************/

/* MCO1 */
#define RCC_CFG3_MCO1SEL_SHIFT                 (28)
#define RCC_CFG3_MCO1SEL_MASK                  (0x0F << RCC_CFG3_MCO1SEL_SHIFT)
#define RCC_CFG3_MCO1SEL_LSI                   (0x08 << RCC_CFG3_MCO1SEL_SHIFT)
#define RCC_CFG3_MCO1SEL_HSI                   (0x09 << RCC_CFG3_MCO1SEL_SHIFT)
#define RCC_CFG3_MCO1SEL_MSI                   (0x0A << RCC_CFG3_MCO1SEL_SHIFT)
#define RCC_CFG3_MCO1SEL_LSE                   (0x0B << RCC_CFG3_MCO1SEL_SHIFT)
#define RCC_CFG3_MCO1SEL_HSE                   (0x0C << RCC_CFG3_MCO1SEL_SHIFT)
#define RCC_CFG3_MCO1SEL_PLL3B                 (0x0D << RCC_CFG3_MCO1SEL_SHIFT)

#define RCC_CFG3_MCO1DIV_SHIFT                 (24)
#define RCC_CFG3_MCO1DIV_MASK                  (0x0F << RCC_CFG3_MCO1DIV_SHIFT)
#define RCC_CFG3_MCO1DIV_DIV1                  (0x00 << RCC_CFG3_MCO1DIV_SHIFT)
#define RCC_CFG3_MCO1DIV_DIV2                  (0x01 << RCC_CFG3_MCO1DIV_SHIFT)
#define RCC_CFG3_MCO1DIV_DIV4                  (0x02 << RCC_CFG3_MCO1DIV_SHIFT)
#define RCC_CFG3_MCO1DIV_DIV8                  (0x04 << RCC_CFG3_MCO1DIV_SHIFT)
#define RCC_CFG3_MCO1DIV_DIV16                 (0x07 << RCC_CFG3_MCO1DIV_SHIFT)
#define RCC_CFG3_MCO1DIV_DIV32                 (0x08 << RCC_CFG3_MCO1DIV_SHIFT)
#define RCC_CFG3_MCO1DIV_DIV64                 (0x09 << RCC_CFG3_MCO1DIV_SHIFT)
#define RCC_CFG3_MCO1DIV_DIV128                (0x0A << RCC_CFG3_MCO1DIV_SHIFT)

/* MCO2 */
#define RCC_CFG3_MCO2SEL_SHIFT                 (20)
#define RCC_CFG3_MCO2SEL_MASK                  (0x0F << RCC_CFG3_MCO2SEL_SHIFT)
#define RCC_CFG3_MCO2SEL_SYSCLK                (0x08 << RCC_CFG3_MCO2SEL_SHIFT)
#define RCC_CFG3_MCO2SEL_PLL1A                 (0x09 << RCC_CFG3_MCO2SEL_SHIFT)
#define RCC_CFG3_MCO2SEL_PLL2A                 (0x0A << RCC_CFG3_MCO2SEL_SHIFT)
#define RCC_CFG3_MCO2SEL_PLL3A                 (0x0B << RCC_CFG3_MCO2SEL_SHIFT)
#define RCC_CFG3_MCO2SEL_SHRPLL                (0x0C << RCC_CFG3_MCO2SEL_SHIFT)
#define RCC_CFG3_MCO2SEL_LSE                   (0x0D << RCC_CFG3_MCO2SEL_SHIFT)

#define RCC_CFG3_MCO2DIV_SHIFT                 (16)
#define RCC_CFG3_MCO2DIV_MASK                  (0x0F << RCC_CFG3_MCO2DIV_SHIFT)
#define RCC_CFG3_MCO2DIV_DIV1                  (0x00 << RCC_CFG3_MCO2DIV_SHIFT)
#define RCC_CFG3_MCO2DIV_DIV2                  (0x01 << RCC_CFG3_MCO2DIV_SHIFT)
#define RCC_CFG3_MCO2DIV_DIV4                  (0x02 << RCC_CFG3_MCO2DIV_SHIFT)
#define RCC_CFG3_MCO2DIV_DIV8                  (0x04 << RCC_CFG3_MCO2DIV_SHIFT)
#define RCC_CFG3_MCO2DIV_DIV16                 (0x07 << RCC_CFG3_MCO2DIV_SHIFT)
#define RCC_CFG3_MCO2DIV_DIV32                 (0x08 << RCC_CFG3_MCO2DIV_SHIFT)
#define RCC_CFG3_MCO2DIV_DIV64                 (0x09 << RCC_CFG3_MCO2DIV_SHIFT)
#define RCC_CFG3_MCO2DIV_DIV128                (0x0A << RCC_CFG3_MCO2DIV_SHIFT)

/* I2S CKIN 选择 */
#define RCC_CFG3_I2SSEL_SHIFT               (14)
#define RCC_CFG3_I2SSEL_MASK                (0x03 << RCC_CFG3_I2SSEL_SHIFT)
#define RCC_CFG3_I2SSEL_I2S1                (0x00 << RCC_CFG3_I2SSEL_SHIFT)
#define RCC_CFG3_I2SSEL_I2S2                (0x01 << RCC_CFG3_I2SSEL_SHIFT)
#define RCC_CFG3_I2SSEL_I2S3                (0x02 << RCC_CFG3_I2SSEL_SHIFT)
#define RCC_CFG3_I2SSEL_I2S4                (0x03 << RCC_CFG3_I2SSEL_SHIFT)

/* 外设时钟选择 */
#define RCC_CFG3_PERSW_SHIFT               (12)
#define RCC_CFG3_PERSW_MASK                (0x03 << RCC_CFG3_PERSW_SHIFT)
#define RCC_CFG3_PERSW_HSI                 (0x00 << RCC_CFG3_PERSW_SHIFT)
#define RCC_CFG3_PERSW_MSI                 (0x02 << RCC_CFG3_PERSW_SHIFT)
#define RCC_CFG3_PERSW_HSE                 (0x03 << RCC_CFG3_PERSW_SHIFT)

/* M7/M4 SysTick 分频器 */
#define RCC_CFG3_M7STCLKDIV_SHIFT          (4)
#define RCC_CFG3_M7STCLKDIV_MASK           (0x0F << RCC_CFG3_M7STCLKDIV_SHIFT)
#define RCC_CFG3_M7STCLKDIV_DIV1           (0x00 << RCC_CFG3_M7STCLKDIV_SHIFT)
#define RCC_CFG3_M7STCLKDIV_DIV2           (0x01 << RCC_CFG3_M7STCLKDIV_SHIFT)
#define RCC_CFG3_M7STCLKDIV_DIV4           (0x02 << RCC_CFG3_M7STCLKDIV_SHIFT)
#define RCC_CFG3_M7STCLKDIV_DIV8           (0x04 << RCC_CFG3_M7STCLKDIV_SHIFT)
#define RCC_CFG3_M7STCLKDIV_DIV16          (0x07 << RCC_CFG3_M7STCLKDIV_SHIFT)
#define RCC_CFG3_M7STCLKDIV_DIV32          (0x08 << RCC_CFG3_M7STCLKDIV_SHIFT)
#define RCC_CFG3_M7STCLKDIV_DIV64          (0x09 << RCC_CFG3_M7STCLKDIV_SHIFT)
#define RCC_CFG3_M7STCLKDIV_DIV128         (0x0A << RCC_CFG3_M7STCLKDIV_SHIFT)
#define RCC_CFG3_M7STCLKDIV_DIV256         (0x0B << RCC_CFG3_M7STCLKDIV_SHIFT)
#define RCC_CFG3_M7STCLKDIV_DIV512         (0x0C << RCC_CFG3_M7STCLKDIV_SHIFT)

#define RCC_CFG3_M4STCLKDIV_SHIFT          (0)
#define RCC_CFG3_M4STCLKDIV_MASK           (0x0F << RCC_CFG3_M4STCLKDIV_SHIFT)
#define RCC_CFG3_M4STCLKDIV_DIV1           (0x00 << RCC_CFG3_M4STCLKDIV_SHIFT)
#define RCC_CFG3_M4STCLKDIV_DIV2           (0x01 << RCC_CFG3_M4STCLKDIV_SHIFT)
#define RCC_CFG3_M4STCLKDIV_DIV4           (0x02 << RCC_CFG3_M4STCLKDIV_SHIFT)
#define RCC_CFG3_M4STCLKDIV_DIV8           (0x04 << RCC_CFG3_M4STCLKDIV_SHIFT)
#define RCC_CFG3_M4STCLKDIV_DIV16          (0x07 << RCC_CFG3_M4STCLKDIV_SHIFT)
#define RCC_CFG3_M4STCLKDIV_DIV32          (0x08 << RCC_CFG3_M4STCLKDIV_SHIFT)
#define RCC_CFG3_M4STCLKDIV_DIV64          (0x09 << RCC_CFG3_M4STCLKDIV_SHIFT)
#define RCC_CFG3_M4STCLKDIV_DIV128         (0x0A << RCC_CFG3_M4STCLKDIV_SHIFT)
#define RCC_CFG3_M4STCLKDIV_DIV256         (0x0B << RCC_CFG3_M4STCLKDIV_SHIFT)
#define RCC_CFG3_M4STCLKDIV_DIV512         (0x0C << RCC_CFG3_M4STCLKDIV_SHIFT)

/* Bit definition for RCC_CFG4 register *************************************/

/* 总线时钟使能位 */
#define RCC_CFG4_AHB1CLKEN          (1U << 31)
#define RCC_CFG4_AHB2CLKEN          (1U << 30)
#define RCC_CFG4_AHB3CLKEN          (1U << 29)
#define RCC_CFG4_AHB5CLKEN          (1U << 27)
#define RCC_CFG4_AHB6CLKEN          (1U << 26)
#define RCC_CFG4_AXICLKEN           (1U << 23)
#define RCC_CFG4_APB1CLKEN          (1U << 22)
#define RCC_CFG4_APB2CLKEN          (1U << 21)
#define RCC_CFG4_APB5CLKEN          (1U << 20)
#define RCC_CFG4_APB6CLKEN          (1U << 19)
#define RCC_CFG4_AHB9CLKEN          (1U << 18)

/* AXI 总线矩阵 */
#define RCC_CFG4_AXIMM7GCLKEN       (1U << 17)
#define RCC_CFG4_AXIMM4GCLKEN       (1U << 16)

/* 时钟源使能位 */
#define RCC_CFG4_HSICGEN            (1U << 15)
#define RCC_CFG4_HSIKERCGEN         (1U << 14)
#define RCC_CFG4_HSECGEN            (1U << 13)
#define RCC_CFG4_HSEKERCGEN         (1U << 12)
#define RCC_CFG4_MSICGEN            (1U << 11)
#define RCC_CFG4_MSIKERCGEN         (1U << 10)

#define RCC_CFG4_RSVD               (1U << 9)

/* 子系统时钟 */
#define RCC_CFG4_AXIMM7CLKEN        (1U << 8)
#define RCC_CFG4_AXIGCLKEN          (1U << 7)
#define RCC_CFG4_AXIMM4CLKEN        (1U << 6)
#define RCC_CFG4_DCMUM7CLKEN        (1U << 5)
#define RCC_CFG4_DCMUM4CLKEN        (1U << 4)
#define RCC_CFG4_AHBM1CLKEN         (1U << 3)
#define RCC_CFG4_AHBM2CLKEN         (1U << 2)
#define RCC_CFG4_AHBM3CLKEN         (1U << 1)
#define RCC_CFG4_DCMURST            (1U << 0)

/* Bit definition for RCC_CFG5 register *************************************/

/* RTC HSE 分频 */
#define RCC_CFG5_RTCHSEDIV_SHIFT    (24)
#define RCC_CFG5_RTCHSEDIV_MASK     (0x3F << RCC_CFG5_RTCHSEDIV_SHIFT)
#define RCC_CFG5_RTCHSEDIV(n)       ((n)  << RCC_CFG5_RTCHSEDIV_SHIFT)

/* SRAM5 */
#define RCC_CFG5_M7SRAM5EN          (1U << 23)
#define RCC_CFG5_M4SRAM5EN          (1U << 22)
#define RCC_CFG5_M7SRAM5LPEN        (1U << 21)
#define RCC_CFG5_M4SRAM5LPEN        (1U << 20)

/* DCD 时钟 */
#define RCC_CFG5_DCDCLKEN           (1U << 16)

/* TRNG 配置 */
#define RCC_CFG5_TRNGEN             (1U << 13)
#define RCC_CFG5_TRNGSEL            (1U << 12)

#define RCC_CFG5_TRNGDIV_SHIFT      (8)
#define RCC_CFG5_TRNGDIV_MASK       (0x0F << RCC_CFG5_TRNGDIV_SHIFT)
#define RCC_CFG5_TRNGDIV_DIV1       (0x00 << RCC_CFG5_TRNGDIV_SHIFT)
#define RCC_CFG5_TRNGDIV_DIV2       (0x01 << RCC_CFG5_TRNGDIV_SHIFT)
#define RCC_CFG5_TRNGDIV_DIV4       (0x02 << RCC_CFG5_TRNGDIV_SHIFT)
#define RCC_CFG5_TRNGDIV_DIV8       (0x04 << RCC_CFG5_TRNGDIV_SHIFT)
#define RCC_CFG5_TRNGDIV_DIV16      (0x07 << RCC_CFG5_TRNGDIV_SHIFT)
#define RCC_CFG5_TRNGDIV_DIV32      (0x08 << RCC_CFG5_TRNGDIV_SHIFT)
#define RCC_CFG5_TRNGDIV_DIV64      (0x09 << RCC_CFG5_TRNGDIV_SHIFT)
#define RCC_CFG5_TRNGDIV_DIV128     (0x0A << RCC_CFG5_TRNGDIV_SHIFT)
#define RCC_CFG5_TRNGDIV_DIV256     (0x0B << RCC_CFG5_TRNGDIV_SHIFT)
#define RCC_CFG5_TRNGDIV_DIV512     (0x0C << RCC_CFG5_TRNGDIV_SHIFT)

/* DSI HSE 分频 */
#define RCC_CFG5_DSIHSEDIV_SHIFT    (4)
#define RCC_CFG5_DSIHSEDIV_MASK     (0x0F << RCC_CFG5_DSIHSEDIV_SHIFT)
#define RCC_CFG5_DSIHSEDIV1         (0x00 << RCC_CFG5_DSIHSEDIV_SHIFT)
#define RCC_CFG5_DSIHSEDIV2         (0x01 << RCC_CFG5_DSIHSEDIV_SHIFT)

/* RTC HSI 分频 */
#define RCC_CFG5_RTCHSIDIV_SHIFT    (0)
#define RCC_CFG5_RTCHSIDIV_MASK     (0x0F << RCC_CFG5_RTCHSIDIV_SHIFT)
#define RCC_CFG5_RTCHSIDIV_DIV1     (0x00 << RCC_CFG5_RTCHSIDIV_SHIFT)
#define RCC_CFG5_RTCHSIDIV_DIV2     (0x01 << RCC_CFG5_RTCHSIDIV_SHIFT)
#define RCC_CFG5_RTCHSIDIV_DIV4     (0x02 << RCC_CFG5_RTCHSIDIV_SHIFT)
#define RCC_CFG5_RTCHSIDIV_DIV8     (0x04 << RCC_CFG5_RTCHSIDIV_SHIFT)
#define RCC_CFG5_RTCHSIDIV_DIV16    (0x07 << RCC_CFG5_RTCHSIDIV_SHIFT)
#define RCC_CFG5_RTCHSIDIV_DIV32    (0x08 << RCC_CFG5_RTCHSIDIV_SHIFT)
#define RCC_CFG5_RTCHSIDIV_DIV64    (0x09 << RCC_CFG5_RTCHSIDIV_SHIFT)
#define RCC_CFG5_RTCHSIDIV_DIV128   (0x0A << RCC_CFG5_RTCHSIDIV_SHIFT)
#define RCC_CFG5_RTCHSIDIV_DIV256   (0x0B << RCC_CFG5_RTCHSIDIV_SHIFT)
#define RCC_CFG5_RTCHSIDIV_DIV512   (0x0C << RCC_CFG5_RTCHSIDIV_SHIFT)

/* Bit definition for RCC_M4RSTREL register *********************************/

/* 0: CM4 in reset state；1: Release CM4 reset */
#define RCC_M4RSTREL_EN         (1U << 0)

/* Bit definition for RCC_LSERDDL register **********************************/

/* 32-bit delay counter */
#define RCC_LSERDDL_DELAY_SHIFT     (0)
#define RCC_LSERDDL_DELAY_MASK      (0xFFFFFFFFU << RCC_LSERDDL_DELAY_SHIFT)

/* Bit definition for RCC_MSIRDDL register **********************************/

/* 32-bit delay counter */
#define RCC_MSIRDDL_DELAY_SHIFT     (0)
#define RCC_MSIRDDL_DELAY_MASK      (0xFFFFFFFFU << RCC_MSIRDDL_DELAY_SHIFT)

/* Bit definition for RCC_HSERDDL register **********************************/

/* 32-bit delay counter */
#define RCC_HSERDDL_DELAY_SHIFT     (0)
#define RCC_HSERDDL_DELAY_MASK      (0xFFFFFFFFU << RCC_HSERDDL_DELAY_SHIFT)

/* Bit definition for RCC_PLLSFTLK register *********************************/

/* [31:29] Reserved */

/* [28:25] SDRAM Delay Chain Select */
#define RCC_PLLSFTLK_SDRAMDLSEL_SHIFT   (25)
#define RCC_PLLSFTLK_SDRAMDLSEL_MASK    (0x0F << RCC_PLLSFTLK_SDRAMDLSEL_SHIFT)

#define RCC_PLLSFTLK_SDRAMDLSEL_0P2NS   (0x0 << RCC_PLLSFTLK_SDRAMDLSEL_SHIFT)
#define RCC_PLLSFTLK_SDRAMDLSEL_0P4NS   (0x1 << RCC_PLLSFTLK_SDRAMDLSEL_SHIFT)
#define RCC_PLLSFTLK_SDRAMDLSEL_0P6NS   (0x2 << RCC_PLLSFTLK_SDRAMDLSEL_SHIFT)
#define RCC_PLLSFTLK_SDRAMDLSEL_0P8NS   (0x3 << RCC_PLLSFTLK_SDRAMDLSEL_SHIFT)
#define RCC_PLLSFTLK_SDRAMDLSEL_1P0NS   (0x4 << RCC_PLLSFTLK_SDRAMDLSEL_SHIFT)
#define RCC_PLLSFTLK_SDRAMDLSEL_1P2NS   (0x5 << RCC_PLLSFTLK_SDRAMDLSEL_SHIFT)
#define RCC_PLLSFTLK_SDRAMDLSEL_1P4NS   (0x6 << RCC_PLLSFTLK_SDRAMDLSEL_SHIFT)
#define RCC_PLLSFTLK_SDRAMDLSEL_1P6NS   (0x7 << RCC_PLLSFTLK_SDRAMDLSEL_SHIFT)
#define RCC_PLLSFTLK_SDRAMDLSEL_1P8NS   (0x8 << RCC_PLLSFTLK_SDRAMDLSEL_SHIFT)
#define RCC_PLLSFTLK_SDRAMDLSEL_2P0NS   (0x9 << RCC_PLLSFTLK_SDRAMDLSEL_SHIFT)
#define RCC_PLLSFTLK_SDRAMDLSEL_2P2NS   (0xA << RCC_PLLSFTLK_SDRAMDLSEL_SHIFT)
#define RCC_PLLSFTLK_SDRAMDLSEL_2P4NS   (0xB << RCC_PLLSFTLK_SDRAMDLSEL_SHIFT)
#define RCC_PLLSFTLK_SDRAMDLSEL_2P6NS   (0xC << RCC_PLLSFTLK_SDRAMDLSEL_SHIFT)
#define RCC_PLLSFTLK_SDRAMDLSEL_2P8NS   (0xD << RCC_PLLSFTLK_SDRAMDLSEL_SHIFT)
#define RCC_PLLSFTLK_SDRAMDLSEL_3P0NS   (0xE << RCC_PLLSFTLK_SDRAMDLSEL_SHIFT)
#define RCC_PLLSFTLK_SDRAMDLSEL_3P2NS   (0xF << RCC_PLLSFTLK_SDRAMDLSEL_SHIFT)

/* [24] SDRAM Delay Chain Enable */
#define RCC_PLLSFTLK_SDRAMDLEN      (1U << 24)
/* [23:6] Reserved */

/* [5] SHRTIM AFE Soft Reset */
#define RCC_PLLSFTLK_SHRTIMAFERST   (1U << 5)
/* [4] Reserved */

/* [3] SHRPLL Software Lock */
#define RCC_PLLSFTLK_SHRPLLSFTLK    (1U << 3)
/* [2] PLL3 Software Lock */
#define RCC_PLLSFTLK_PLL3SFTLK      (1U << 2)
/* [1] PLL2 Software Lock */
#define RCC_PLLSFTLK_PLL2SFTLK      (1U << 1)
/* [0] PLL1 Software Lock */
#define RCC_PLLSFTLK_PLL1SFTLK      (1U << 0)

/* Bit definition for RCC_HSEOS register ************************************/

/* [31:24] HSE Max Positive Deviation Threshold */
#define RCC_HSEOS_HSEMAXPDTHR_SHIFT     (24)
#define RCC_HSEOS_HSEMAXPDTHR_MASK      (0xFFUL << RCC_HSEOS_HSEMAXPDTHR_SHIFT)

/* [23:16] HSE Min Negative Deviation Threshold */
#define RCC_HSEOS_HSEMINNDTHR_SHIFT     (16)
#define RCC_HSEOS_HSEMINNDTHR_MASK      (0xFFUL << RCC_HSEOS_HSEMINNDTHR_SHIFT)

/* [15:8] HSE Offset Threshold for 5% Detection */
#define RCC_HSEOS_HSEOSTHR_SHIFT        (8)
#define RCC_HSEOS_HSEOSTHR_MASK         (0xFFUL << RCC_HSEOS_HSEOSTHR_SHIFT)

/* Single Frequency */
#define RCC_HSEOS_HSEOSTHR_4MHZ         (0x20UL << RCC_HSEOS_HSEOSTHR_SHIFT)
#define RCC_HSEOS_HSEOSTHR_5MHZ         (0x1AUL << RCC_HSEOS_HSEOSTHR_SHIFT)
#define RCC_HSEOS_HSEOSTHR_6MHZ         (0x16UL << RCC_HSEOS_HSEOSTHR_SHIFT)
#define RCC_HSEOS_HSEOSTHR_7MHZ         (0x12UL << RCC_HSEOS_HSEOSTHR_SHIFT)
#define RCC_HSEOS_HSEOSTHR_8MHZ         (0x10UL << RCC_HSEOS_HSEOSTHR_SHIFT)
#define RCC_HSEOS_HSEOSTHR_9MHZ         (0x0EUL << RCC_HSEOS_HSEOSTHR_SHIFT)
#define RCC_HSEOS_HSEOSTHR_10MHZ        (0x0DUL << RCC_HSEOS_HSEOSTHR_SHIFT)
#define RCC_HSEOS_HSEOSTHR_11MHZ        (0x0CUL << RCC_HSEOS_HSEOSTHR_SHIFT)
#define RCC_HSEOS_HSEOSTHR_12MHZ        (0x0BUL << RCC_HSEOS_HSEOSTHR_SHIFT)
#define RCC_HSEOS_HSEOSTHR_13MHZ        (0x0AUL << RCC_HSEOS_HSEOSTHR_SHIFT)

/* Range Frequency */
#define RCC_HSEOS_HSEOSTHR_14_15MHZ     (0x09UL << RCC_HSEOS_HSEOSTHR_SHIFT)
#define RCC_HSEOS_HSEOSTHR_16_17MHZ     (0x08UL << RCC_HSEOS_HSEOSTHR_SHIFT)
#define RCC_HSEOS_HSEOSTHR_18_19MHZ     (0x07UL << RCC_HSEOS_HSEOSTHR_SHIFT)
#define RCC_HSEOS_HSEOSTHR_20_24MHZ     (0x06UL << RCC_HSEOS_HSEOSTHR_SHIFT)
#define RCC_HSEOS_HSEOSTHR_25_30MHZ     (0x05UL << RCC_HSEOS_HSEOSTHR_SHIFT)
#define RCC_HSEOS_HSEOSTHR_31_39MHZ     (0x04UL << RCC_HSEOS_HSEOSTHR_SHIFT)
#define RCC_HSEOS_HSEOSTHR_40_50MHZ     (0x03UL << RCC_HSEOS_HSEOSTHR_SHIFT)

/* [7] Reserved */

/* [6] HSE Max Positive Deviation Flag (rc_w1) */
#define RCC_HSEOS_HSEMAXPDF             (1U << 6)
/* [5] HSE Min Negative Deviation Flag (rc_w1) */
#define RCC_HSEOS_HSEMINNDF             (1U << 5)
/* [4] HSE Offset Flag ≥5% (rc_w1) */
#define RCC_HSEOS_HSEOSF                (1U << 4)
/* [3] Reserved */

/* [2] HSE Max Positive Deviation Detect Enable */
#define RCC_HSEOS_HSEMAXPDEN            (1U << 2)
/* [1] HSE Min Negative Deviation Detect Enable */
#define RCC_HSEOS_HSEMINNDEN            (1U << 1)
/* [0] HSE Offset Detect Enable */
#define RCC_HSEOS_HSEOSEN               (1U << 0)

/* Bit definition for RCC_HSECAL register ***********************************/

/* [31:18] Reserved */

/* [17] HSE Calibration Count Enable */
#define RCC_HSECAL_HSECALCNTEN      (1U << 17)

/* [16] HSE Calibration Count Ready Flag (read-only) */
#define RCC_HSECAL_HSECALCNTF       (1U << 16)

/* [15:0] HSE Calibration Count Value (read-only) */
#define RCC_HSECAL_HSECALCNT_SHIFT  (0)

#define RCC_HSECAL_HSECALCNT_MASK   (0xFFFFU << RCC_HSECAL_HSECALCNT_SHIFT)

/* Bit definition for RCC_LSEOS register ************************************/

/* [31:26] Reserved */

/* [25] LSE Calibration Count Enable */
#define RCC_LSEOS_LSECALCNTEN       (1U << 25)
/* [24] LSE Calibration Count Ready Flag (read-only) */
#define RCC_LSEOS_LSECALCNTF        (1U << 24)

/* [23:16] LSE Calibration Count Value (read-only) */
#define RCC_LSEOS_LSECALCNT_SHIFT   (16)
#define RCC_LSEOS_LSECALCNT_MASK    (0xFFUL << RCC_LSEOS_LSECALCNT_SHIFT)

/* [15:10] Reserved */

/* [9] LSE Offset Flag ≥10% (rc_w1) */
#define RCC_LSEOS_LSEOSF            (1U << 9)

/* [8] LSE Offset Detect Enable */
#define RCC_LSEOS_LSEOSEN           (1U << 8)

/* [7:0] LSE Offset Threshold for 10% Detection */
#define RCC_LSEOS_LSEOSTHR_SHIFT    (0)
#define RCC_LSEOS_LSEOSTHR_MASK     (0xFFUL << RCC_LSEOS_LSEOSTHR_SHIFT)

/* Bit definition for RCC_PLLFD register ************************************/

/* [31:12] Reserved */

/* [11] SHRPLL Clock Gate Status (read-only) */
#define RCC_PLLFD_SHRPLLGF      (1U << 11)

/* [10] PLL3 Clock Gate Status (read-only) */
#define RCC_PLLFD_PLL3GF        (1U << 10)

/* [9] PLL2 Clock Gate Status (read-only) */
#define RCC_PLLFD_PLL2GF        (1U << 9)

/* [8] PLL1 Clock Gate Status (read-only) */
#define RCC_PLLFD_PLL1GF        (1U << 8)

/* [7] SHRPLL Fail Flag (read-only) */
#define RCC_PLLFD_SHRPLLFF      (1U << 7)

/* [6] PLL3 Fail Flag (read-only) */
#define RCC_PLLFD_PLL3FF        (1U << 6)

/* [5] PLL2 Fail Flag (read-only) */
#define RCC_PLLFD_PLL2FF        (1U << 5)

/* [4] PLL1 Fail Flag (read-only) */
#define RCC_PLLFD_PLL1FF        (1U << 4)

/* [3] SHRPLL Fail Detect Enable */
#define RCC_PLLFD_SHRPLLFEN     (1U << 3)

/* [2] PLL3 Fail Detect Enable */
#define RCC_PLLFD_PLL3FEN       (1U << 2)

/* [1] PLL2 Fail Detect Enable */
#define RCC_PLLFD_PLL2FEN       (1U << 1)

/* [0] PLL1 Fail Detect Enable */
#define RCC_PLLFD_PLL1FEN       (1U << 0)

#endif /* __ARCH_ARM_SRC_N32H7_HARDWARE_N32H76X_RCC_H */
