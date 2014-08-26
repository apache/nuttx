/********************************************************************************************
 * arch/arm/src/tiva/chip/cc3200_syscontrol.h
 *
 *   Copyright (C) 2014 Droidifi LLC. All rights reserved.
 *   Author: Jim Ewing <jim@droidifi.com>
 *
 *   Adapted for the cc3200 from code:
 *
 *   Copyright (C) Gregory Nutt.
 *   Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name Droidifi nor the names of its contributors may be
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_TIVA_CHIP_CC3200_SYSCONTROL_H
#define __ARCH_ARM_SRC_TIVA_CHIP_CC3200_SYSCONTROL_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/* System Control Register Offsets **********************************************************/

#define TIVA_SYSCON_DID0_OFFSET       0x000 /* Device Identification 0 */
#define TIVA_SYSCON_DID1_OFFSET       0x004 /* Device Identification 1 */
#define TIVA_SYSCON_PBORCTL_OFFSET    0x030 /* Brown-Out Reset Control */
#define TIVA_SYSCON_RIS_OFFSET        0x050 /* Raw Interrupt Status */
#define TIVA_SYSCON_IMC_OFFSET        0x054 /* Interrupt Mask Control */
#define TIVA_SYSCON_MISC_OFFSET       0x058 /* Masked Interrupt Status and Clear */
#define TIVA_SYSCON_RESC_OFFSET       0x05c /* Reset Cause */
#define TIVA_SYSCON_RCC_OFFSET        0x060 /* Run-Mode Clock Configuration */
#define TIVA_SYSCON_GPIOHBCTL_OFFSET  0x06c /* GPIO High-Performance Bus Control */
#define TIVA_SYSCON_RCC2_OFFSET       0x070 /* Run-Mode Clock Configuration 2 */
#define TIVA_SYSCON_MOSCCTL_OFFSET    0x07c /* Main Oscillator Control */
#define TIVA_SYSCON_DSLPCLKCFG_OFFSET 0x144 /* Deep Sleep Clock Configuration */
#define TIVA_SYSCON_SYSPROP_OFFSET    0x14c /* System Properties */
#define TIVA_SYSCON_PIOSCCAL_OFFSET   0x150 /* Precision Internal Oscillator Calibration */
#define TIVA_SYSCON_PIOSCSTAT_OFFSET  0x154 /* Precision Internal Oscillator Statistics */
#define TIVA_SYSCON_PLLFREQ0_OFFSET   0x160 /* PLL 0 Frequency */
#define TIVA_SYSCON_PLLFREQ1_OFFSET   0x164 /* PLL 1 Frequency */
#define TIVA_SYSCON_PLLSTAT_OFFSET    0x168 /* PLL Status */
#define TIVA_SYSCON_SLPPWRCFG_OFFSET  0x188 /* Sleep Power Configuration */
#define TIVA_SYSCON_DSLPPWRCFG_OFFSET 0x18c /* Deep-Sleep Power Configuration */
#define TIVA_SYSCON_LDOSPCTL_OFFSET   0x1b4 /* LDO Sleep Power Control */
#define TIVA_SYSCON_LDOSPCAL_OFFSET   0x1b8 /* LDO Sleep Power Calibration */
#define TIVA_SYSCON_LDODPCTL_OFFSET   0x1bc /* LDO Deep-Sleep Power Control */
#define TIVA_SYSCON_LDODPCAL_OFFSET   0x1c0 /* LDO Deep-Sleep Power Calibration */
#define TIVA_SYSCON_SDPMST_OFFSET     0x1cc /* Sleep / Deep-Sleep Power Mode Status */

#define TIVA_SYSCON_PPWD_OFFSET       0x300 /* Watchdog Timer Peripheral Present */
#define TIVA_SYSCON_PPTIMER_OFFSET    0x304 /* 16/32-Bit Timer Peripheral Present */
#define TIVA_SYSCON_PPGPIO_OFFSET     0x308 /* GPIO Peripheral Present */
#define TIVA_SYSCON_PPDMA_OFFSET      0x30c /* uDMA Peripheral Present */
#define TIVA_SYSCON_PPHIB_OFFSET      0x314 /* Hibernation Peripheral Present */
#define TIVA_SYSCON_PPUART_OFFSET     0x318 /* UART Present */
#define TIVA_SYSCON_PPSSI_OFFSET      0x31c /* SSI Peripheral Present */
#define TIVA_SYSCON_PPI2C_OFFSET      0x320 /* I2C Peripheral Present */
#define TIVA_SYSCON_PPUSB_OFFSET      0x328 /* USB Peripheral Present */
#define TIVA_SYSCON_PPCAN_OFFSET      0x334 /* CAN Peripheral Present */
#define TIVA_SYSCON_PPADC_OFFSET      0x338 /* ADC Peripheral Present */
#define TIVA_SYSCON_PPACMP_OFFSET     0x33c /* Analog Comparator Peripheral Present */
#define TIVA_SYSCON_PPPWM_OFFSET      0x340 /* Pulse Width Modulator Peripheral Present */
#define TIVA_SYSCON_PPQEI_OFFSET      0x344 /* Quadrature Encoder Peripheral Present */
#define TIVA_SYSCON_PPEEPROM_OFFSET   0x358 /* EEPROM Peripheral Present */
#define TIVA_SYSCON_PPWTIMER_OFFSET   0x35c /* 32/64-Bit Wide Timer Peripheral Present */

#define TIVA_SYSCON_SRWD_OFFSET       0x500 /* Watchdog Timer Software Reset */
#define TIVA_SYSCON_SRTIMER_OFFSET    0x504 /* 16/32-Bit Timer Software Reset */
#define TIVA_SYSCON_SRGPIO_OFFSET     0x508 /* GPIO Software Reset */
#define TIVA_SYSCON_SRDMA_OFFSET      0x50c /* uDMA Software Reset */
#define TIVA_SYSCON_SRHIB_OFFSET      0x514 /* Hibernation Software Reset */
#define TIVA_SYSCON_SRUART_OFFSET     0x518 /* UART Software Reset*/
#define TIVA_SYSCON_SRSSI_OFFSET      0x51c /* SSI Software Reset */
#define TIVA_SYSCON_SRI2C_OFFSET      0x520 /* I2C Software Reset */
#define TIVA_SYSCON_SRUSB_OFFSET      0x528 /* USB Software Reset */
#define TIVA_SYSCON_SRCAN_OFFSET      0x534 /* CAN Software Reset */
#define TIVA_SYSCON_SRADC_OFFSET      0x538 /* ADC Software Reset */
#define TIVA_SYSCON_SRACMP_OFFSET     0x53c /* Analog Comparator Software Reset */
#define TIVA_SYSCON_SRPWM_OFFSET      0x540 /* Pulse Width Modulator Software Reset */
#define TIVA_SYSCON_SRQEI_OFFSET      0x544 /* Quadrature Encoder Interface Software Reset */
#define TIVA_SYSCON_SREEPROM_OFFSET   0x558 /* EEPROM Software Reset */
#define TIVA_SYSCON_SRWTIMER_OFFSET   0x55c /* 32/64-Bit Wide Timer Software Reset */

#define TIVA_SYSCON_RCGCWD_OFFSET     0x600 /* Watchdog Timer Run Mode Clock Gating Control */
#define TIVA_SYSCON_RCGCTIMER_OFFSET  0x604 /* 16/32-Bit Timer Run Mode Clock Gating Control */
#define TIVA_SYSCON_RCGCGPIO_OFFSET   0x608 /* GPIO Run Mode Clock Gating Control*/
#define TIVA_SYSCON_RCGCDMA_OFFSET    0x60c /* uDMA Run Mode Clock Gating Control*/
#define TIVA_SYSCON_RCGCHIB_OFFSET    0x614 /* Hibernation Run Mode Clock Gating Control */
#define TIVA_SYSCON_RCGCUART_OFFSET   0x618 /* UART Run Mode Clock Gating Control*/
#define TIVA_SYSCON_RCGCSSI_OFFSET    0x61c /* SSI Run Mode Clock Gating Control*/
#define TIVA_SYSCON_RCGCI2C_OFFSET    0x620 /* I2C Run Mode Clock Gating Control */
#define TIVA_SYSCON_RCGCUSB_OFFSET    0x628 /* USB Run Mode Clock Gating Control */
#define TIVA_SYSCON_RCGCCAN_OFFSET    0x634 /* CAN Run Mode Clock Gating Control */
#define TIVA_SYSCON_RCGCADC_OFFSET    0x638 /* ADC Run Mode Clock Gating Control */
#define TIVA_SYSCON_RCGCACMP_OFFSET   0x63c /* Analog Comparator Run Mode Clock Gating Control */
#define TIVA_SYSCON_RCGCPWM_OFFSET    0x640 /* Pulse Width Modulator Run Mode Clock Gating Control */
#define TIVA_SYSCON_RCGCQEI_OFFSET    0x644 /* Quadrature Encoder Interface Run Mode Clock Gating Control */
#define TIVA_SYSCON_RCGCEEPROM_OFFSET 0x658 /* EEPROM Run Mode Clock Gating Control */
#define TIVA_SYSCON_RCGCWTIMER_OFFSET 0x65c /* 32/64-BitWide Timer Run Mode Clock Gating Control */

#define TIVA_SYSCON_SCGCWD_OFFSET     0x700 /* Watchdog Timer Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_SCGCTIMER_OFFSET  0x704 /* 16/32-Bit Timer Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_SCGCGPIO_OFFSET   0x708 /* GPIO Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_SCGCDMA_OFFSET    0x70c /* uDMA Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_SCGCHIB_OFFSET    0x714 /* Hibernation Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_SCGCUART_OFFSET   0x718 /* UART Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_SCGCSSI_OFFSET    0x71c /* SSI Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_SCGCI2C_OFFSET    0x720 /* I2C Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_SCGCUSB_OFFSET    0x728 /* USB Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_SCGCCAN_OFFSET    0x734 /* CAN Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_SCGCADC_OFFSET    0x738 /* ADC Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_SCGCACMP_OFFSET   0x73c /* Analog Comparator Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_SCGCPWM_OFFSET    0x740 /* PulseWidthModulator Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_SCGCQEI_OFFSET    0x744 /* Quadrature Encoder Interface Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_SCGCEEPROM_OFFSET 0x758 /* EEPROM Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_SCGCWTIMER_OFFSET 0x75c /* 32/64-BitWide Timer Sleep Mode Clock Gating Control */

#define TIVA_SYSCON_DCGCWD_OFFSET     0x800 /* Watchdog Timer Deep-Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_DCGCTIMER_OFFSET  0x804 /* Clock Gating Control */
#define TIVA_SYSCON_DCGCGPIO_OFFSET   0x808 /* GPIO Deep-Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_DCGCDMA_OFFSET    0x80c /* uDMA Deep-Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_DCGCHIB_OFFSET    0x814 /* Hibernation Deep-Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_DCGCUART_OFFSET   0x818 /* UART Deep-Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_DCGCSSI_OFFSET    0x81c /* SSI Deep-Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_DCGCI2C_OFFSET    0x820 /* I2C Deep-Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_DCGCUSB_OFFSET    0x828 /* USB Deep-Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_DCGCCAN_OFFSET    0x834 /* CAN Deep-Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_DCGCADC_OFFSET    0x838 /* ADC Deep-Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_DCGCACMP_OFFSET   0x83c /* Analog Comparator Deep-Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_DCGCPWM_OFFSET    0x840 /* Pulse Width Modulator Deep-Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_DCGCQEI_OFFSET    0x844 /* Quadrature Encoder Interface Deep-Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_DCGCEEPROM_OFFSET 0x858 /* EEPROM Deep-Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_DCGCWTIMER_OFFSET 0x85c /* 32/64-BitWide Timer Deep-Sleep Mode Clock Gating Control */

#define TIVA_SYSCON_PRWD_OFFSET       0xa00 /* Watchdog Timer Peripheral Ready */
#define TIVA_SYSCON_PRTIMER_OFFSET    0xa04 /* 16/32-Bit Timer Peripheral Ready */
#define TIVA_SYSCON_PRGPIO_OFFSET     0xa08 /* GPIO Peripheral Ready */
#define TIVA_SYSCON_PRDMA_OFFSET      0xa0c /* uDMA Peripheral Ready */
#define TIVA_SYSCON_PRHIB_OFFSET      0xa14 /* Hibernation Peripheral Ready */
#define TIVA_SYSCON_PRUART_OFFSET     0xa18 /* UART Peripheral Ready */
#define TIVA_SYSCON_PRSSI_OFFSET      0xa1c /* SSI Peripheral Ready */
#define TIVA_SYSCON_PRI2C_OFFSET      0xa20 /* I2C Peripheral Ready */
#define TIVA_SYSCON_PRUSB_OFFSET      0xa28 /* USB Peripheral Ready */
#define TIVA_SYSCON_PRCAN_OFFSET      0xa34 /* CAN Peripheral Ready */
#define TIVA_SYSCON_PRADC_OFFSET      0xa38 /* ADC Peripheral Ready */
#define TIVA_SYSCON_PRACMP_OFFSET     0xa3c /* Analog Comparator Peripheral Ready */
#define TIVA_SYSCON_PRPWM_OFFSET      0xa40 /* Pulse Width Modulator Peripheral Ready */
#define TIVA_SYSCON_PRQEI_OFFSET      0xa44 /* Quadrature Encoder Interface Peripheral Ready */
#define TIVA_SYSCON_PREEPROM_OFFSET   0xa58 /* EEPROM Peripheral Ready */
#define TIVA_SYSCON_PRWTIMER_OFFSET   0xa5c /* 2/64-BitWide Timer Peripheral Ready */

/* System Control Legacy Register Offsets ***************************************************/

#define TIVA_SYSCON_DC0_OFFSET        0x008 /* Device Capabilities 0 */
#define TIVA_SYSCON_DC1_OFFSET        0x010 /* Device Capabilities 1 */
#define TIVA_SYSCON_DC2_OFFSET        0x014 /* Device Capabilities 2 */
#define TIVA_SYSCON_DC3_OFFSET        0x018 /* Device Capabilities 3 */
#define TIVA_SYSCON_DC4_OFFSET        0x01c /* Device Capabilities 4 */
#define TIVA_SYSCON_DC5_OFFSET        0x020 /* Device Capabilities 5 */
#define TIVA_SYSCON_DC6_OFFSET        0x024 /* Device Capabilities 6 */
#define TIVA_SYSCON_DC7_OFFSET        0x028 /* Device Capabilities 7 */
#define TIVA_SYSCON_DC8_OFFSET        0x02c /* Device Capabilities 8 */

#define TIVA_SYSCON_SRCR0_OFFSET      0x040 /* Software Reset Control 0 */
#define TIVA_SYSCON_SRCR1_OFFSET      0x044 /* Software Reset Control 1 */
#define TIVA_SYSCON_SRCR2_OFFSET      0x048 /* Software Reset Control 2 */

#define TIVA_SYSCON_RCGC0_OFFSET      0x100 /* Run Mode Clock Gating Control Register 0 */
#define TIVA_SYSCON_RCGC1_OFFSET      0x104 /* Run Mode Clock Gating Control Register 1 */
#define TIVA_SYSCON_RCGC2_OFFSET      0x108 /* Run Mode Clock Gating Control Register 2 */

#define TIVA_SYSCON_SCGC0_OFFSET      0x110 /* Sleep Mode Clock Gating Control Register 0 */
#define TIVA_SYSCON_SCGC1_OFFSET      0x114 /* Sleep Mode Clock Gating Control Register 1 */
#define TIVA_SYSCON_SCGC2_OFFSET      0x118 /* Sleep Mode Clock Gating Control Register 2 */

#define TIVA_SYSCON_DCGC0_OFFSET      0x120 /* Deep Sleep Mode Clock Gating Control Register 0 */
#define TIVA_SYSCON_DCGC1_OFFSET      0x124 /* Deep Sleep Mode Clock Gating Control Register 1 */
#define TIVA_SYSCON_DCGC2_OFFSET      0x128 /* Deep Sleep Mode Clock Gating Control Register 2 */

#define TIVA_SYSCON_DC9_OFFSET        0x190 /* Device Capabilities */
#define TIVA_SYSCON_NVMSTAT_OFFSET    0x1a0 /* Non-Volatile Memory Information */

/* System Control Register Addresses ********************************************************/

#define TIVA_SYSCON_DID0              (TIVA_SYSCON_BASE + TIVA_SYSCON_DID0_OFFSET)
#define TIVA_SYSCON_DID1              (TIVA_SYSCON_BASE + TIVA_SYSCON_DID1_OFFSET)
#define TIVA_SYSCON_PBORCTL           (TIVA_SYSCON_BASE + TIVA_SYSCON_PBORCTL_OFFSET)
#define TIVA_SYSCON_RIS               (TIVA_SYSCON_BASE + TIVA_SYSCON_RIS_OFFSET)
#define TIVA_SYSCON_IMC               (TIVA_SYSCON_BASE + TIVA_SYSCON_IMC_OFFSET)
#define TIVA_SYSCON_MISC              (TIVA_SYSCON_BASE + TIVA_SYSCON_MISC_OFFSET)
#define TIVA_SYSCON_RESC              (TIVA_SYSCON_BASE + TIVA_SYSCON_RESC_OFFSET)
#define TIVA_SYSCON_RCC               (TIVA_SYSCON_BASE + TIVA_SYSCON_RCC_OFFSET)
#define TIVA_SYSCON_GPIOHBCTL         (TIVA_SYSCON_BASE + TIVA_SYSCON_GPIOHBCTL_OFFSET)
#define TIVA_SYSCON_RCC2              (TIVA_SYSCON_BASE + TIVA_SYSCON_RCC2_OFFSET)
#define TIVA_SYSCON_MOSCCTL           (TIVA_SYSCON_BASE + TIVA_SYSCON_MOSCCTL_OFFSET)
#define TIVA_SYSCON_DSLPCLKCFG        (TIVA_SYSCON_BASE + TIVA_SYSCON_DSLPCLKCFG_OFFSET)
#define TIVA_SYSCON_SYSPROP           (TIVA_SYSCON_BASE + TIVA_SYSCON_SYSPROP_OFFSET)
#define TIVA_SYSCON_PIOSCCAL          (TIVA_SYSCON_BASE + TIVA_SYSCON_PIOSCCAL_OFFSET)
#define TIVA_SYSCON_PIOSCSTAT         (TIVA_SYSCON_BASE + TIVA_SYSCON_PIOSCSTAT_OFFSET)
#define TIVA_SYSCON_PLLFREQ0          (TIVA_SYSCON_BASE + TIVA_SYSCON_PLLFREQ0_OFFSET)
#define TIVA_SYSCON_PLLFREQ1          (TIVA_SYSCON_BASE + TIVA_SYSCON_PLLFREQ1_OFFSET)
#define TIVA_SYSCON_PLLSTAT           (TIVA_SYSCON_BASE + TIVA_SYSCON_PLLSTAT_OFFSET)
#define TIVA_SYSCON_SLPPWRCFG         (TIVA_SYSCON_BASE + TIVA_SYSCON_SLPPWRCFG_OFFSET)
#define TIVA_SYSCON_DSLPPWRCFG        (TIVA_SYSCON_BASE + TIVA_SYSCON_DSLPPWRCFG_OFFSET)
#define TIVA_SYSCON_LDOSPCTL          (TIVA_SYSCON_BASE + TIVA_SYSCON_LDOSPCTL_OFFSET)
#define TIVA_SYSCON_LDOSPCAL          (TIVA_SYSCON_BASE + TIVA_SYSCON_LDOSPCAL_OFFSET)
#define TIVA_SYSCON_LDODPCTL          (TIVA_SYSCON_BASE + TIVA_SYSCON_LDODPCTL_OFFSET)
#define TIVA_SYSCON_LDODPCAL          (TIVA_SYSCON_BASE + TIVA_SYSCON_LDODPCAL_OFFSET)
#define TIVA_SYSCON_SDPMST            (TIVA_SYSCON_BASE + TIVA_SYSCON_SDPMST_OFFSET)

#define TIVA_SYSCON_PPWD              (TIVA_SYSCON_BASE + TIVA_SYSCON_PPWD_OFFSET)
#define TIVA_SYSCON_PPTIMER           (TIVA_SYSCON_BASE + TIVA_SYSCON_PPTIMER_OFFSET)
#define TIVA_SYSCON_PPGPIO            (TIVA_SYSCON_BASE + TIVA_SYSCON_PPGPIO_OFFSET)
#define TIVA_SYSCON_PPDMA             (TIVA_SYSCON_BASE + TIVA_SYSCON_PPDMA_OFFSET)
#define TIVA_SYSCON_PPHIB             (TIVA_SYSCON_BASE + TIVA_SYSCON_PPHIB_OFFSET)
#define TIVA_SYSCON_PPUART            (TIVA_SYSCON_BASE + TIVA_SYSCON_PPUART_OFFSET)
#define TIVA_SYSCON_PPSSI             (TIVA_SYSCON_BASE + TIVA_SYSCON_PPSSI_OFFSET)
#define TIVA_SYSCON_PPI2C             (TIVA_SYSCON_BASE + TIVA_SYSCON_PPI2C_OFFSET)
#define TIVA_SYSCON_PPUSB             (TIVA_SYSCON_BASE + TIVA_SYSCON_PPUSB_OFFSET)
#define TIVA_SYSCON_PPCAN             (TIVA_SYSCON_BASE + TIVA_SYSCON_PPCAN_OFFSET)
#define TIVA_SYSCON_PPADC             (TIVA_SYSCON_BASE + TIVA_SYSCON_PPADC_OFFSET)
#define TIVA_SYSCON_PPACMP            (TIVA_SYSCON_BASE + TIVA_SYSCON_PPACMP_OFFSET)
#define TIVA_SYSCON_PPPWM             (TIVA_SYSCON_BASE + TIVA_SYSCON_PPPWM_OFFSET)
#define TIVA_SYSCON_PPQEI             (TIVA_SYSCON_BASE + TIVA_SYSCON_PPQEI_OFFSET)
#define TIVA_SYSCON_PPEEPROM          (TIVA_SYSCON_BASE + TIVA_SYSCON_PPEEPROM_OFFSET)
#define TIVA_SYSCON_PPWTIMER          (TIVA_SYSCON_BASE + TIVA_SYSCON_PPWTIMER_OFFSET)

#define TIVA_SYSCON_SRWD              (TIVA_SYSCON_BASE + TIVA_SYSCON_SRWD_OFFSET)
#define TIVA_SYSCON_SRTIMER           (TIVA_SYSCON_BASE + TIVA_SYSCON_SRTIMER_OFFSET)
#define TIVA_SYSCON_SRGPIO            (TIVA_SYSCON_BASE + TIVA_SYSCON_SRGPIO_OFFSET)
#define TIVA_SYSCON_SRDMA             (TIVA_SYSCON_BASE + TIVA_SYSCON_SRDMA_OFFSET)
#define TIVA_SYSCON_SRHIB             (TIVA_SYSCON_BASE + TIVA_SYSCON_SRHIB_OFFSET)
#define TIVA_SYSCON_SRUART            (TIVA_SYSCON_BASE + TIVA_SYSCON_SRUART_OFFSET)
#define TIVA_SYSCON_SRSSI             (TIVA_SYSCON_BASE + TIVA_SYSCON_SRSSI_OFFSET)
#define TIVA_SYSCON_SRI2C             (TIVA_SYSCON_BASE + TIVA_SYSCON_SRI2C_OFFSET)
#define TIVA_SYSCON_SRUSB             (TIVA_SYSCON_BASE + TIVA_SYSCON_SRUSB_OFFSET)
#define TIVA_SYSCON_SRCAN             (TIVA_SYSCON_BASE + TIVA_SYSCON_SRCAN_OFFSET)
#define TIVA_SYSCON_SRADC             (TIVA_SYSCON_BASE + TIVA_SYSCON_SRADC_OFFSET)
#define TIVA_SYSCON_SRACMP            (TIVA_SYSCON_BASE + TIVA_SYSCON_SRACMP_OFFSET)
#define TIVA_SYSCON_SRPWM             (TIVA_SYSCON_BASE + TIVA_SYSCON_SRPWM_OFFSET)
#define TIVA_SYSCON_SRQEI             (TIVA_SYSCON_BASE + TIVA_SYSCON_SRQEI_OFFSET)
#define TIVA_SYSCON_SREEPROM          (TIVA_SYSCON_BASE + TIVA_SYSCON_SREEPROM_OFFSET)
#define TIVA_SYSCON_SRWTIMER          (TIVA_SYSCON_BASE + TIVA_SYSCON_SRWTIMER_OFFSET)

#define TIVA_SYSCON_RCGCWD            (TIVA_SYSCON_BASE + TIVA_SYSCON_RCGCWD_OFFSET)
#define TIVA_SYSCON_RCGCTIMER         (TIVA_SYSCON_BASE + TIVA_SYSCON_RCGCTIMER_OFFSET)
#define TIVA_SYSCON_RCGCGPIO          (TIVA_SYSCON_BASE + TIVA_SYSCON_RCGCGPIO_OFFSET)
#define TIVA_SYSCON_RCGCDMA           (TIVA_SYSCON_BASE + TIVA_SYSCON_RCGCDMA_OFFSET)
#define TIVA_SYSCON_RCGCHIB           (TIVA_SYSCON_BASE + TIVA_SYSCON_RCGCHIB_OFFSET)
#define TIVA_SYSCON_RCGCUART          (TIVA_SYSCON_BASE + TIVA_SYSCON_RCGCUART_OFFSET)
#define TIVA_SYSCON_RCGCSSI           (TIVA_SYSCON_BASE + TIVA_SYSCON_RCGCSSI_OFFSET)
#define TIVA_SYSCON_RCGCI2C           (TIVA_SYSCON_BASE + TIVA_SYSCON_RCGCI2C_OFFSET)
#define TIVA_SYSCON_RCGCUSB           (TIVA_SYSCON_BASE + TIVA_SYSCON_RCGCUSB_OFFSET)
#define TIVA_SYSCON_RCGCCAN           (TIVA_SYSCON_BASE + TIVA_SYSCON_RCGCCAN_OFFSET)
#define TIVA_SYSCON_RCGCADC           (TIVA_SYSCON_BASE + TIVA_SYSCON_RCGCADC_OFFSET)
#define TIVA_SYSCON_RCGCACMP          (TIVA_SYSCON_BASE + TIVA_SYSCON_RCGCACMP_OFFSET)
#define TIVA_SYSCON_RCGCPWM           (TIVA_SYSCON_BASE + TIVA_SYSCON_RCGCPWM_OFFSET)
#define TIVA_SYSCON_RCGCQEI           (TIVA_SYSCON_BASE + TIVA_SYSCON_RCGCQEI_OFFSET)
#define TIVA_SYSCON_RCGCEEPROM        (TIVA_SYSCON_BASE + TIVA_SYSCON_RCGCEEPROM_OFFSET)
#define TIVA_SYSCON_RCGCWTIMER        (TIVA_SYSCON_BASE + TIVA_SYSCON_RCGCWTIMER_OFFSET)

#define TIVA_SYSCON_SCGCWD            (TIVA_SYSCON_BASE + TIVA_SYSCON_SCGCWD_OFFSET)
#define TIVA_SYSCON_SCGCTIMER         (TIVA_SYSCON_BASE + TIVA_SYSCON_SCGCTIMER_OFFSET)
#define TIVA_SYSCON_SCGCGPIO          (TIVA_SYSCON_BASE + TIVA_SYSCON_SCGCGPIO_OFFSET)
#define TIVA_SYSCON_SCGCDMA           (TIVA_SYSCON_BASE + TIVA_SYSCON_SCGCDMA_OFFSET)
#define TIVA_SYSCON_SCGCHIB           (TIVA_SYSCON_BASE + TIVA_SYSCON_SCGCHIB_OFFSET)
#define TIVA_SYSCON_SCGCUART          (TIVA_SYSCON_BASE + TIVA_SYSCON_SCGCUART_OFFSET)
#define TIVA_SYSCON_SCGCSSI           (TIVA_SYSCON_BASE + TIVA_SYSCON_SCGCSSI_OFFSET)
#define TIVA_SYSCON_SCGCI2C           (TIVA_SYSCON_BASE + TIVA_SYSCON_SCGCI2C_OFFSET)
#define TIVA_SYSCON_SCGCUSB           (TIVA_SYSCON_BASE + TIVA_SYSCON_SCGCUSB_OFFSET)
#define TIVA_SYSCON_SCGCCAN           (TIVA_SYSCON_BASE + TIVA_SYSCON_SCGCCAN_OFFSET)
#define TIVA_SYSCON_SCGCADC           (TIVA_SYSCON_BASE + TIVA_SYSCON_SCGCADC_OFFSET)
#define TIVA_SYSCON_SCGCACMP          (TIVA_SYSCON_BASE + TIVA_SYSCON_SCGCACMP_OFFSET)
#define TIVA_SYSCON_SCGCPWM           (TIVA_SYSCON_BASE + TIVA_SYSCON_SCGCPWM_OFFSET
#define TIVA_SYSCON_SCGCQEI           (TIVA_SYSCON_BASE + TIVA_SYSCON_SCGCQEI_OFFSET
#define TIVA_SYSCON_SCGCEEPROM        (TIVA_SYSCON_BASE + TIVA_SYSCON_SCGCEEPROM_OFFSET)
#define TIVA_SYSCON_SCGCWTIMER        (TIVA_SYSCON_BASE + TIVA_SYSCON_SCGCWTIMER_OFFSET)

#define TIVA_SYSCON_DCGCWD            (TIVA_SYSCON_BASE + TIVA_SYSCON_DCGCWD_OFFSET)
#define TIVA_SYSCON_DCGCTIMER         (TIVA_SYSCON_BASE + TIVA_SYSCON_DCGCTIMER_OFFSET)
#define TIVA_SYSCON_DCGCGPIO          (TIVA_SYSCON_BASE + TIVA_SYSCON_DCGCGPIO_OFFSET)
#define TIVA_SYSCON_DCGCDMA           (TIVA_SYSCON_BASE + TIVA_SYSCON_DCGCDMA_OFFSET)
#define TIVA_SYSCON_DCGCHIB           (TIVA_SYSCON_BASE + TIVA_SYSCON_DCGCHIB_OFFSET)
#define TIVA_SYSCON_DCGCUART          (TIVA_SYSCON_BASE + TIVA_SYSCON_DCGCUART_OFFSET)
#define TIVA_SYSCON_DCGCSSI           (TIVA_SYSCON_BASE + TIVA_SYSCON_DCGCSSI_OFFSET)
#define TIVA_SYSCON_DCGCI2C           (TIVA_SYSCON_BASE + TIVA_SYSCON_DCGCI2C_OFFSET)
#define TIVA_SYSCON_DCGCUSB           (TIVA_SYSCON_BASE + TIVA_SYSCON_DCGCUSB_OFFSET)
#define TIVA_SYSCON_DCGCCAN           (TIVA_SYSCON_BASE + TIVA_SYSCON_DCGCCAN_OFFSET)
#define TIVA_SYSCON_DCGCADC           (TIVA_SYSCON_BASE + TIVA_SYSCON_DCGCADC_OFFSET)
#define TIVA_SYSCON_DCGCACMP          (TIVA_SYSCON_BASE + TIVA_SYSCON_DCGCACMP_OFFSET)
#define TIVA_SYSCON_DCGCPWM           (TIVA_SYSCON_BASE + TIVA_SYSCON_DCGCPWM_OFFSET)
#define TIVA_SYSCON_DCGCQEI           (TIVA_SYSCON_BASE + TIVA_SYSCON_DCGCQEI_OFFSET)
#define TIVA_SYSCON_DCGCEEPROM        (TIVA_SYSCON_BASE + TIVA_SYSCON_DCGCEEPROM_OFFSET)
#define TIVA_SYSCON_DCGCWTIMER        (TIVA_SYSCON_BASE + TIVA_SYSCON_DCGCWTIMER_OFFSET)

#define TIVA_SYSCON_PRWD              (TIVA_SYSCON_BASE + TIVA_SYSCON_PRWD_OFFSET)
#define TIVA_SYSCON_PRTIMER           (TIVA_SYSCON_BASE + TIVA_SYSCON_PRTIMER_OFFSET)
#define TIVA_SYSCON_PRGPIO            (TIVA_SYSCON_BASE + TIVA_SYSCON_PRGPIO_OFFSET)
#define TIVA_SYSCON_PRDMA             (TIVA_SYSCON_BASE + TIVA_SYSCON_PRDMA_OFFSET)
#define TIVA_SYSCON_PRHIB             (TIVA_SYSCON_BASE + TIVA_SYSCON_PRHIB_OFFSET)
#define TIVA_SYSCON_PRUART            (TIVA_SYSCON_BASE + TIVA_SYSCON_PRUART_OFFSET)
#define TIVA_SYSCON_PRSSI             (TIVA_SYSCON_BASE + TIVA_SYSCON_PRSSI_OFFSET)
#define TIVA_SYSCON_PRI2C             (TIVA_SYSCON_BASE + TIVA_SYSCON_PRI2C_OFFSET)
#define TIVA_SYSCON_PRUSB             (TIVA_SYSCON_BASE + TIVA_SYSCON_PRUSB_OFFSET)
#define TIVA_SYSCON_PRCAN             (TIVA_SYSCON_BASE + TIVA_SYSCON_PRCAN_OFFSET)
#define TIVA_SYSCON_PRADC             (TIVA_SYSCON_BASE + TIVA_SYSCON_PRADC_OFFSET)
#define TIVA_SYSCON_PRACMP            (TIVA_SYSCON_BASE + TIVA_SYSCON_PRACMP_OFFSET)
#define TIVA_SYSCON_PRPWM             (TIVA_SYSCON_BASE + TIVA_SYSCON_PRPWM_OFFSET)
#define TIVA_SYSCON_PRQEI             (TIVA_SYSCON_BASE + TIVA_SYSCON_PRQEI_OFFSET)
#define TIVA_SYSCON_PREEPROM          (TIVA_SYSCON_BASE + TIVA_SYSCON_PREEPROM_OFFSET)
#define TIVA_SYSCON_PRWTIMER          (TIVA_SYSCON_BASE + TIVA_SYSCON_PRWTIMER_OFFSET)

/* System Control Legacy Register Addresses *************************************************/

#define TIVA_SYSCON_DC0               (TIVA_SYSCON_BASE + TIVA_SYSCON_DC0_OFFSET)
#define TIVA_SYSCON_DC1               (TIVA_SYSCON_BASE + TIVA_SYSCON_DC1_OFFSET)
#define TIVA_SYSCON_DC2               (TIVA_SYSCON_BASE + TIVA_SYSCON_DC2_OFFSET)
#define TIVA_SYSCON_DC3               (TIVA_SYSCON_BASE + TIVA_SYSCON_DC3_OFFSET)
#define TIVA_SYSCON_DC4               (TIVA_SYSCON_BASE + TIVA_SYSCON_DC4_OFFSET)
#define TIVA_SYSCON_DC5               (TIVA_SYSCON_BASE + TIVA_SYSCON_DC5_OFFSET)
#define TIVA_SYSCON_DC6               (TIVA_SYSCON_BASE + TIVA_SYSCON_DC6_OFFSET)
#define TIVA_SYSCON_DC7               (TIVA_SYSCON_BASE + TIVA_SYSCON_DC7_OFFSET)
#define TIVA_SYSCON_DC8               (TIVA_SYSCON_BASE + TIVA_SYSCON_DC8_OFFSET)

#define TIVA_SYSCON_SRCR0             (TIVA_SYSCON_BASE + TIVA_SYSCON_SRCR0_OFFSET)
#define TIVA_SYSCON_SRCR1             (TIVA_SYSCON_BASE + TIVA_SYSCON_SRCR1_OFFSET)
#define TIVA_SYSCON_SRCR2             (TIVA_SYSCON_BASE + TIVA_SYSCON_SRCR2_OFFSET)

#define TIVA_SYSCON_RCGC0             (TIVA_SYSCON_BASE + TIVA_SYSCON_RCGC0_OFFSET)
#define TIVA_SYSCON_RCGC1             (TIVA_SYSCON_BASE + TIVA_SYSCON_RCGC1_OFFSET)
#define TIVA_SYSCON_RCGC2             (TIVA_SYSCON_BASE + TIVA_SYSCON_RCGC2_OFFSET)

#define TIVA_SYSCON_SCGC0             (TIVA_SYSCON_BASE + TIVA_SYSCON_SCGC0_OFFSET)
#define TIVA_SYSCON_SCGC1             (TIVA_SYSCON_BASE + TIVA_SYSCON_SCGC1_OFFSET)
#define TIVA_SYSCON_SCGC2             (TIVA_SYSCON_BASE + TIVA_SYSCON_SCGC2_OFFSET)

#define TIVA_SYSCON_DCGC0             (TIVA_SYSCON_BASE + TIVA_SYSCON_DCGC0_OFFSET)
#define TIVA_SYSCON_DCGC1             (TIVA_SYSCON_BASE + TIVA_SYSCON_DCGC1_OFFSET)
#define TIVA_SYSCON_DCGC2             (TIVA_SYSCON_BASE + TIVA_SYSCON_DCGC2_OFFSET)

#define TIVA_SYSCON_DC9               (TIVA_SYSCON_BASE + TIVA_SYSCON_DC9_OFFSET)
#define TIVA_SYSCON_NVMSTAT           (TIVA_SYSCON_BASE + TIVA_SYSCON_NVMSTAT_OFFSET)

/* System Control Register Bit Definitions **************************************************/

/* Device Identification 0 */

#define SYSCON_DID0_MINOR_SHIFT       0         /* Bits 7-0: Minor Revision of the device */
#define SYSCON_DID0_MINOR_MASK        (0xff << SYSCON_DID0_MINOR_SHIFT)
#define SYSCON_DID0_MAJOR_SHIFT       8         /* Bits 15-8: Major Revision of the device */
#define SYSCON_DID0_MAJOR_MASK        (0xff << SYSCON_DID0_MAJOR_SHIFT)
#define SYSCON_DID0_CLASS_SHIFT       16        /* Bits 23-16: Device Class */
#define SYSCON_DID0_CLASS_MASK        (0xff << SYSCON_DID0_CLASS_SHIFT)
#define SYSCON_DID0_VER_SHIFT         28        /* Bits 30-28: DID0 Version */
#define SYSCON_DID0_VER_MASK          (7 << SYSCON_DID0_VER_SHIFT)

/* Device Identification 1 */

#define SYSCON_DID1_QUAL_SHIFT        0         /* Bits 1-0: Qualification Status */
#define SYSCON_DID1_QUAL_MASK         (0x03 << SYSCON_DID1_QUAL_SHIFT)
#define SYSCON_DID1_ROHS              (1 << 2)  /* Bit 2: RoHS-Compliance */
#define SYSCON_DID1_PKG_SHIFT         3 /* Bits 4-3: Package Type */
#define SYSCON_DID1_PKG_MASK          (0x03 << SYSCON_DID1_PKG_SHIFT)
#define SYSCON_DID1_TEMP_SHIFT        5         /* Bits 7-5: Temperature Range */
#define SYSCON_DID1_TEMP_MASK         (0x07 << SYSCON_DID1_TEMP_SHIFT)
#define SYSCON_DID1_PINCOUNT_SHIFT    13        /* Bits 15-13: Package Pin Count */
#define SYSCON_DID1_PINCOUNT_MASK     (0x07 << SYSCON_DID1_PINCOUNT_SHIFT)
#define SYSCON_DID1_PARTNO_SHIFT      16        /* Bits 23-16: Part Number */
#define SYSCON_DID1_PARTNO_MASK       (0xff << SYSCON_DID1_PARTNO_SHIFT)
#define SYSCON_DID1_FAM_SHIFT         24        /* Bits 27-24: Family */
#define SYSCON_DID1_FAM_MASK          (0x0f << SYSCON_DID1_FAM_SHIFT)
#define SYSCON_DID1_VER_SHIFT         28        /* Bits 31-28:  DID1 Version */
#define SYSCON_DID1_VER_MASK          (0x0f << SYSCON_DID1_VER_SHIFT)

/* Brown-Out Reset Control */

#define SYSCON_PBORCTL_BORI1          (1 << 1)  /* Bit 1: VDD under BOR1 Event Action */
#define SYSCON_PBORCTL_BORI0          (1 << 2)  /* Bit 2: VDD under BOR0 Event Action */

/* Raw Interrupt Status */

#define SYSCON_RIS_BORR1RIS           (1 << 1)  /* Bit 1:  VDD under BOR1 Raw Interrupt Status */
#define SYSCON_RIS_MOFRIS             (1 << 3)  /* Bit 3:  Main Oscillator Failure Raw Interrupt Status */
#define SYSCON_RIS_PLLLRIS            (1 << 6)  /* Bit 6:  PLL Lock Raw Interrupt Status */
#define SYSCON_RIS_USBPLLLRIS         (1 << 7)  /* Bit 7:  USB PLL Lock Raw Interrupt Status */
#define SYSCON_RIS_MOSCPUPRIS         (1 << 8)  /* Bit 8:  MOSC Power Up Raw Interrupt Status */
#define SYSCON_RIS_VDDARIS            (1 << 10) /* Bit 10: VDDA Power OK Event Raw Interrupt Status */
#define SYSCON_RIS_BOR0RIS            (1 << 11) /* Bit 11: VDD under BOR0 Raw Interrupt Status */

/* Interrupt Mask Control */

#define SYSCON_IMC_BORR1RIM           (1 << 1)  /* Bit 1:  VDD under BOR1 Raw Interrupt Mask */
#define SYSCON_IMC_MOFRIM             (1 << 3)  /* Bit 3:  Main Oscillator Failure Raw Interrupt Mask */
#define SYSCON_IMC_PLLLRIM            (1 << 6)  /* Bit 6:  PLL Lock Raw Interrupt Mask */
#define SYSCON_IMC_USBPLLLRIM         (1 << 7)  /* Bit 7:  USB PLL Lock Raw Interrupt Mask */
#define SYSCON_IMC_MOSCPUPRIM         (1 << 8)  /* Bit 8:  MOSC Power Up Raw Interrupt Mask */
#define SYSCON_IMC_VDDARIM            (1 << 10) /* Bit 10: VDDA Power OK Event Raw Interrupt Mask */
#define SYSCON_IMC_BOR0RIM            (1 << 11) /* Bit 11: VDD under BOR0 Raw Interrupt Mask */

/* Masked Interrupt Status and Clear */

#define SYSCON_MISC_BORR1MIS          (1 << 1)  /* Bit 1:  VDD under BOR1 Masked Interrupt Status */
#define SYSCON_MISC_MOFMIS            (1 << 3)  /* Bit 3:  Main Oscillator Failure Masked Interrupt Status */
#define SYSCON_MISC_PLLLMIS           (1 << 6)  /* Bit 6:  PLL Lock Masked Interrupt Status */
#define SYSCON_MISC_USBPLLLMIS        (1 << 7)  /* Bit 7:  USB PLL Lock Masked Interrupt Status */
#define SYSCON_MISC_MOSCPUPMIS        (1 << 8)  /* Bit 8:  MOSC Power Up Masked Interrupt Status */
#define SYSCON_MISC_VDDAMIS           (1 << 10) /* Bit 10: VDDA Power OK Event Masked Interrupt Status */
#define SYSCON_MISC_BOR0MIS           (1 << 11) /* Bit 11: VDD under BOR0 Masked Interrupt Status */

/* Reset Cause */

#define SYSCON_RESC_EXT               (1 << 0)  /* Bit 0:  External Reset */
#define SYSCON_RESC_POR               (1 << 1)  /* Bit 1:  Power-On Reset */
#define SYSCON_RESC_BOR               (1 << 2)  /* Bit 2:  Brown-Out Reset */
#define SYSCON_RESC_WDT0              (1 << 3)  /* Bit 3:  Watchdog Timer 0 Reset */
#define SYSCON_RESC_SW                (1 << 4)  /* Bit 4:  Software Reset */
#define SYSCON_RESC_WDT1              (1 << 5)  /* Bit 5:  Watchdog Timer 1 Reset */
#define SYSCON_RESC_MOSCFAIL          (1 << 16) /* Bit 16: MOSC Failure Reset */

/* Run-Mode Clock Configuration */

#define SYSCON_RCC_MOSCDIS            (1 << 0)  /* Bit 0: Main Oscillator Disable */
#define SYSCON_RCC_OSCSRC_SHIFT       4         /* Bits 5-4: Oscillator Source */
#define SYSCON_RCC_OSCSRC_MASK        (0x03 << SYSCON_RCC_OSCSRC_SHIFT)
#  define SYSCON_RCC_OSCSRC_MOSC      (0 << SYSCON_RCC_OSCSRC_SHIFT) /* Main oscillator */
#  define SYSCON_RCC_OSCSRC_PIOSC     (1 << SYSCON_RCC_OSCSRC_SHIFT) /* Precision internal oscillator (reset) */
#  define SYSCON_RCC_OSCSRC_PIOSC4    (2 << SYSCON_RCC_OSCSRC_SHIFT) /* Precision internal oscillator / 4 */
#  define SYSCON_RCC_OSCSRC_LFIOSC    (3 << SYSCON_RCC_OSCSRC_SHIFT) /* Low-frequency internal oscillator */
#define SYSCON_RCC_XTAL_SHIFT         6         /* Bits 10-6: Crystal Value */
#define SYSCON_RCC_XTAL_MASK          (31 << SYSCON_RCC_XTAL_SHIFT)
#  define SYSCON_RCC_XTAL4000KHZ      (6 << SYSCON_RCC_XTAL_SHIFT)  /* 4 MHz (NO PLL) */
#  define SYSCON_RCC_XTAL4096KHZ      (7 << SYSCON_RCC_XTAL_SHIFT)  /* 4.096 MHz (NO PLL) */
#  define SYSCON_RCC_XTAL4915p2KHZ    (8 << SYSCON_RCC_XTAL_SHIFT)  /* 4.9152 MHz (NO PLL) */
#  define SYSCON_RCC_XTAL5000KHZ      (9 << SYSCON_RCC_XTAL_SHIFT)  /* 5 MHz (USB) */
#  define SYSCON_RCC_XTAL5120KHZ      (10 << SYSCON_RCC_XTAL_SHIFT) /* 5.12 MHz */
#  define SYSCON_RCC_XTAL6000KHZ      (11 << SYSCON_RCC_XTAL_SHIFT) /* 6 MHz (USB) */
#  define SYSCON_RCC_XTAL6144KHZ      (12 << SYSCON_RCC_XTAL_SHIFT) /* 6.144 MHz */
#  define SYSCON_RCC_XTAL7372p8KHZ    (13 << SYSCON_RCC_XTAL_SHIFT) /* 7.3728 MHz */
#  define SYSCON_RCC_XTAL8000KHZ      (14 << SYSCON_RCC_XTAL_SHIFT) /* 8 MHz (USB) */
#  define SYSCON_RCC_XTAL8192KHZ      (15 << SYSCON_RCC_XTAL_SHIFT) /* 8.192 MHz */
#  define SYSCON_RCC_XTAL10000KHZ     (16 << SYSCON_RCC_XTAL_SHIFT) /* 10.0 MHz (USB) */
#  define SYSCON_RCC_XTAL12000KHZ     (17 << SYSCON_RCC_XTAL_SHIFT) /* 12.0 MHz (USB) */
#  define SYSCON_RCC_XTAL12288KHZ     (18 << SYSCON_RCC_XTAL_SHIFT) /* 12.288 MHz */
#  define SYSCON_RCC_XTAL13560KHZ     (19 << SYSCON_RCC_XTAL_SHIFT) /* 13.56 MHz */
#  define SYSCON_RCC_XTAL14318p18KHZ  (20 << SYSCON_RCC_XTAL_SHIFT) /* 14.31818 MHz */
#  define SYSCON_RCC_XTAL16000KHZ     (21 << SYSCON_RCC_XTAL_SHIFT) /* 16.0 MHz (USB) */
#  define SYSCON_RCC_XTAL16384KHZ     (22 << SYSCON_RCC_XTAL_SHIFT) /* 16.384 MHz */
#  define SYSCON_RCC_XTAL18000KHZ     (23 << SYSCON_RCC_XTAL_SHIFT) /* 18.0 MHz (USB) */
#  define SYSCON_RCC_XTAL20000KHZ     (24 << SYSCON_RCC_XTAL_SHIFT) /* 20.0 MHz (USB) */
#  define SYSCON_RCC_XTAL24000KHZ     (25 << SYSCON_RCC_XTAL_SHIFT) /* 24.0 MHz (USB) */
#  define SYSCON_RCC_XTAL25000KHZ     (26 << SYSCON_RCC_XTAL_SHIFT) /* 25.0 MHz (USB) */
#  define SYSCON_RCC_XTAL40000KHZ     (27 << SYSCON_RCC_XTAL_SHIFT) /* 40.0 MHz */
#define SYSCON_RCC_BYPASS             (1 << 11) /* Bit 11: PLL Bypass */
#define SYSCON_RCC_PWRDN              (1 << 13) /* Bit 13: PLL Power Down */
#define SYSCON_RCC_PWMDIV_SHIFT       17         /* Bits 19-17: PWM Unit Clock Divisor */
#define SYSCON_RCC_PWMDIV_MASK        (7 << SYSCON_RCC_PWMDIV_SHIFT)
#  define SYSCON_RCC_PWMDIV_2         (0 << SYSCON_RCC_PWMDIV_SHIFT) /* /2 */
#  define SYSCON_RCC_PWMDIV_4         (1 << SYSCON_RCC_PWMDIV_SHIFT) /* /4 */
#  define SYSCON_RCC_PWMDIV_8         (2 << SYSCON_RCC_PWMDIV_SHIFT) /* /8 */
#  define SYSCON_RCC_PWMDIV_16        (3 << SYSCON_RCC_PWMDIV_SHIFT) /* /16 */
#  define SYSCON_RCC_PWMDIV_32        (4 << SYSCON_RCC_PWMDIV_SHIFT) /* /32 */
#  define SYSCON_RCC_PWMDIV_64        (7 << SYSCON_RCC_PWMDIV_SHIFT) /* /64 (default) */
#define SYSCON_RCC_USEPWMDIV          (1 << 20) /* Bit 20: Enable PWM Clock Divisor */
#define SYSCON_RCC_USESYSDIV          (1 << 22) /* Bit 22: Enable System Clock Divider */
#define SYSCON_RCC_SYSDIV_SHIFT       23        /* Bits 26-23: System Clock Divisor */
#define SYSCON_RCC_SYSDIV_MASK        (0x0f << SYSCON_RCC_SYSDIV_SHIFT)
#  define SYSCON_RCC_SYSDIV(n)        (((n)-1) << SYSCON_RCC_SYSDIV_SHIFT)
#define SYSCON_RCC_ACG                (1 << 27) /* Bit 27: Auto Clock Gating */

/* GPIO High-Performance Bus Control */

#define SYSCON_GPIOHBCTL_PORT(n)      (1 << (n))
#  define SYSCON_GPIOHBCTL_PORTA      (1 << 0)  /* Bit 0:  Port A Advanced High-Performance Bus */
#  define SYSCON_GPIOHBCTL_PORTB      (1 << 1)  /* Bit 1:  Port B Advanced High-Performance Bus */
#  define SYSCON_GPIOHBCTL_PORTC      (1 << 2)  /* Bit 2:  Port C Advanced High-Performance Bus */
#  define SYSCON_GPIOHBCTL_PORTD      (1 << 3)  /* Bit 3:  Port D Advanced High-Performance Bus */
#  define SYSCON_GPIOHBCTL_PORTE      (1 << 4)  /* Bit 4:  Port E Advanced High-Performance Bus */
#  define SYSCON_GPIOHBCTL_PORTF      (1 << 5)  /* Bit 5:  Port F Advanced High-Performance Bus */
#  define SYSCON_GPIOHBCTL_PORTG      (1 << 6)  /* Bit 6:  Port G Advanced High-Performance Bus */
#  define SYSCON_GPIOHBCTL_PORTH      (1 << 7)  /* Bit 7:  Port H Advanced High-Performance Bus */
#  define SYSCON_GPIOHBCTL_PORTJ      (1 << 8)  /* Bit 8:  Port J Advanced High-Performance Bus */
#  define SYSCON_GPIOHBCTL_PORTK      (1 << 9)  /* Bit 9:  Port K Advanced High-Performance Bus */
#  define SYSCON_GPIOHBCTL_PORTL      (1 << 10) /* Bit 10: Port L Advanced High-Performance Bus */
#  define SYSCON_GPIOHBCTL_PORTM      (1 << 11) /* Bit 11: Port M Advanced High-Performance Bus */
#  define SYSCON_GPIOHBCTL_PORTN      (1 << 12) /* Bit 12: Port N Advanced High-Performance Bus */
#  define SYSCON_GPIOHBCTL_PORTP      (1 << 13) /* Bit 13: Port P Advanced High-Performance Bus */
#  define SYSCON_GPIOHBCTL_PORTQ      (1 << 14) /* Bit 14: Port Q Advanced High-Performance Bus */

/* Run-Mode Clock Configuration 2 */

#define SYSCON_RCC2_OSCSRC2_SHIFT      4         /* Bits 6-4: Oscillator Source */
#define SYSCON_RCC2_OSCSRC2_MASK       (7 << SYSCON_RCC2_OSCSRC2_SHIFT)
#  define SYSCON_RCC2_OSCSRC2_MOSC     (0 << SYSCON_RCC2_OSCSRC2_SHIFT) /* Main oscillator */
#  define SYSCON_RCC2_OSCSRC2_PIOSC    (1 << SYSCON_RCC2_OSCSRC2_SHIFT) /* Precision internal oscillator (reset) */
#  define SYSCON_RCC2_OSCSRC2_PIOSC4   (2 << SYSCON_RCC2_OSCSRC2_SHIFT) /* Precision internal oscillator / 4 */
#  define SYSCON_RCC2_OSCSRC2_LFIOSC   (4 << SYSCON_RCC2_OSCSRC2_SHIFT) /* Low-frequency internal oscillator */
#  define SYSCON_RCC2_OSCSRC2_32768HZ  (7 << SYSCON_RCC2_OSCSRC2_SHIFT) /* 32.768KHz external oscillator */
#define SYSCON_RCC2_BYPASS2            (1 << 11) /* Bit 11: Bypass PLL */
#define SYSCON_RCC2_PWRDN2             (1 << 13) /* Bit 13: Power-Down PLL */
#define SYSCON_RCC2_USBPWRDN           (1 << 14) /* Bit 14: Power-Down USB PLL */
#define SYSCON_RCC2_SYSDIV2LSB         (1 << 22) /* Bit 22: Additional LSB for SYSDIV2 */
#define SYSCON_RCC2_SYSDIV2_SHIFT      23        /* Bits 28-23: System Clock Divisor */
#define SYSCON_RCC2_SYSDIV2_MASK       (0x3f << SYSCON_RCC2_SYSDIV2_SHIFT)
#  define SYSCON_RCC2_SYSDIV(n)        ((n-1) << SYSCON_RCC2_SYSDIV2_SHIFT)
#  define SYSCON_RCC2_SYSDIV_DIV400(n) (((n-1) >> 1) << SYSCON_RCC2_SYSDIV2_SHIFT)
#define SYSCON_RCC2_DIV400             (1 << 30) /* Bit 30: Divide PLL as 400 MHz vs. 200 MHz */
#define SYSCON_RCC2_USERCC2            (1 << 31) /* Bit 31: Use RCC2 When set */

/* Main Oscillator Control */

#define SYSCON_MOSCCTL_CVAL           (1 << 0) /* Bit 0:  Clock Validation for MOSC */
#define SYSCON_MOSCCTL_MOSCIM         (1 << 1) /* Bit 1:  MOSC Failure Action */
#define SYSCON_MOSCCTL_NOXTAL         (1 << 2) /* Bit 2:  No Crystal Connected */

/* Deep Sleep Clock Configuration */

#define SYSCON_DSLPCLKCFG_PIOSCPD             (1 << 1) /* Bit 1:  PIOSC Power Down Request */
#define SYSCON_DSLPCLKCFG_DSOSCSRC_SHIFT      4 /* Bits 6-4: Clock Source */
#define SYSCON_DSLPCLKCFG_DSOSCSRC_MASK       (7 << SYSCON_DSLPCLKCFG_DSOSCSRC_SHIFT)
#  define SYSCON_DSLPCLKCFG_DSOSCSRC_MOSC     (0 << SYSCON_DSLPCLKCFG_DSOSCSRC_SHIFT) /* Main oscillator */
#  define SYSCON_DSLPCLKCFG_DSOSCSRC_PIOSC    (1 << SYSCON_DSLPCLKCFG_DSOSCSRC_SHIFT) /* Precision internal oscillator (reset) */
#  define SYSCON_DSLPCLKCFG_DSOSCSRC_PIOSC4   (2 << SYSCON_DSLPCLKCFG_DSOSCSRC_SHIFT) /* Precision internal oscillator / 4 */
#  define SYSCON_DSLPCLKCFG_DSOSCSRC_LFIOSC   (4 << SYSCON_DSLPCLKCFG_DSOSCSRC_SHIFT) /* Low-frequency internal oscillator */
#  define SYSCON_DSLPCLKCFG_DSOSCSRC_32768KHZ (7 << SYSCON_DSLPCLKCFG_DSOSCSRC_SHIFT) /* 32.768KHz external oscillator */
#define SYSCON_DSLPCLKCFG_DSDIVORIDE_SHIFT    23 /* Bits 28-23: Divider Field Override */
#define SYSCON_DSLPCLKCFG_DSDIVORIDE_MASK     (0x3f << SYSCON_DSLPCLKCFG_DSDIVORIDE_SHIFT)
#  define SYSCON_DSLPCLKCFG_DSDIVORIDE(b)     (((n)-1) << SYSCON_DSLPCLKCFG_DSDIVORIDE_SHIFT)

/* System Properties */

#define SYSCON_SYSPROP_FPU            (1 << 0)  /* Bit 0:  FPU Present */
#define SYSCON_SYSPROP_FLASHLPM       (1 << 8)  /* Bit 8:  Flash Memory Sleep/Deep-Sleep Low Power Mode Present */
#define SYSCON_SYSPROP_SRAMLPM        (1 << 10) /* Bit 10: SRAM Sleep/Deep-Sleep Low Power Mode Present */
#define SYSCON_SYSPROP_SRAMSM         (1 << 11) /* Bit 11: SRAM Sleep/Deep-Sleep Standby Mode Present */
#define SYSCON_SYSPROP_PIOSCPDE       (1 << 12) /* Bit 12: PIOSC Power Down Present */

/* Precision Internal Oscillator Calibration */

#define SYSCON_PIOSCCAL_UT_SHIFT      (0)      /* Bits 0-6: User Trim Value */
#define SYSCON_PIOSCCAL_UT_MASK       (0x7f << SYSCON_PIOSCCAL_UT_SHIFT)
#  define SYSCON_PIOSCCAL_UT(n)       ((uint32_t)(n) << SYSCON_PIOSCCAL_UT_SHIFT)
#define SYSCON_PIOSCCAL_UPDATE        (1 << 8)  /* Bit 8:  Update Trim */
#define SYSCON_PIOSCCAL_CAL           (1 << 9)  /* Bit 9:  Start Calibration */
#define SYSCON_PIOSCCAL_UTEN          (1 << 31) /* Bit 31: Use User Trim Value */

/* Precision Internal Oscillator Statistics */

#define SYSCON_PIOSCSTAT_CT_SHIFT     (0)       /* Bits 0-6: Calibration Trim Value */
#define SYSCON_PIOSCSTAT_CT_MASK      (0x7f << SYSCON_PIOSCSTAT_CT_SHIFT)
#  define SYSCON_PIOSCSTAT_CT(n)      ((uint32_t)(n) << SYSCON_PIOSCSTAT_CT_SHIFT)
#define SYSCON_PIOSCSTAT_RESULT_SHIFT (8)       /* Bits 8-9: Calibration Result */
#define SYSCON_PIOSCSTAT_RESULT_MASK  (3 << SYSCON_PIOSCSTAT_RESULT_SHIFT)
#  define SYSCON_PIOSCSTAT_RESULT(n)  ((uint32_t)(n) << SYSCON_PIOSCSTAT_RESULT_SHIFT)
#define SYSCON_PIOSCSTAT_DT_SHIFT     (16)      /* Bits 16-22: Default Trim Value */
#define SYSCON_PIOSCSTAT_DT_MASK      (0x7f << SYSCON_PIOSCSTAT_DT_SHIFT)
#  define SYSCON_PIOSCSTAT_DT(n)      ((uint32_t)(n) << SYSCON_PIOSCSTAT_DT_SHIFT)

/* PLL0 Frequency */

#define SYSCON_PLLFREQ0_MINT_SHIFT    (0)       /* Bits 0-9: PLL M Integer Value */
#define SYSCON_PLLFREQ0_MINT_MASK     (0x3ff << SYSCON_PLLFREQ0_MINT_SHIFT)
#  define SYSCON_PLLFREQ0_MINT(n)     ((uint32_t)(n) << SYSCON_PLLFREQ0_MINT_SHIFT)
#define SYSCON_PLLFREQ0_MFRAC_SHIFT   (10)      /* Bits 10-19:  PLL M Fractional Value */
#define SYSCON_PLLFREQ0_MFRAC_MASK    (0x3ff << SYSCON_PLLFREQ0_MFRAC_SHIFT)
#  define SYSCON_PLLFREQ0_MFRAC(n)    ((uint32_t)(n) << SYSCON_PLLFREQ0_MFRAC_SHIFT)

/* PLL1 Frequency */

#define SYSCON_PLLFREQ1_N_SHIFT       (0)       /* Bits 0-4: PLL N Value */
#define SYSCON_PLLFREQ1_N_MASK        (31 << SYSCON_PLLFREQ1_N_SHIFT)
#  define SYSCON_PLLFREQ1_N(n)        ((uint32_t)(n) << SYSCON_PLLFREQ1_N_SHIFT)
#define SYSCON_PLLFREQ1_Q_SHIFT       (8)       /* Bits 8-12: PLL Q Value */
#define SYSCON_PLLFREQ1_Q_MASK        (31 << SYSCON_PLLFREQ1_Q_SHIFT)
#  define SYSCON_PLLFREQ1_Q(n)        ((uint32_t)(n) << SYSCON_PLLFREQ1_Q_SHIFT)

/* PLL Status */

#define SYSCON_PLLSTAT_LOCK           (1 << 0)  /* Bit 0: PLL Lock */

/* Sleep Power Configuration */

#define SYSCON_SLPPWRCFG_SRAMPM_SHIFT      (0)  /* Bits 1-0: SRAM Power Modes */
#define SYSCON_SLPPWRCFG_SRAMPM_MASK       (3 << SYSCON_SLPPWRCFG_SRAMPM_SHIFT)
#  define SYSCON_SLPPWRCFG_SRAMPM_ACTIVE   (0 << SYSCON_SLPPWRCFG_SRAMPM_SHIFT) /* Active Mode */
#  define SYSCON_SLPPWRCFG_SRAMPM_STANDBY  (1 << SYSCON_SLPPWRCFG_SRAMPM_SHIFT) /* Standby Mode */
#  define SYSCON_SLPPWRCFG_SRAMPM_LOWPWR   (2 << SYSCON_SLPPWRCFG_SRAMPM_SHIFT) /* Low Power Mode */
#define SYSCON_SLPPWRCFG_FLASHPM_SHIFT     (4)  /* Bits 5-4: Flash Power Modes */
#define SYSCON_SLPPWRCFG_FLASHPM_MASK      (3 << SYSCON_SLPPWRCFG_FLASHPM_SHIFT)
#  define SYSCON_SLPPWRCFG_FLASHPM_ACTIVE  (0 << SYSCON_SLPPWRCFG_FLASHPM_SHIFT) /* Active Mode */
#  define SYSCON_SLPPWRCFG_FLASHPM_LOWPWRR (2 << SYSCON_SLPPWRCFG_FLASHPM_SHIFT) /* Low Power Mode */

/* Deep-Sleep Power Configuration */

#define SYSCON_DSLPPWRCFG_SRAMPM_SHIFT     (0)  /* Bits 1-0: SRAM Power Modes */
#define SYSCON_DSLPPWRCFG_SRAMPM_MASK      (3 << SYSCON_DSLPPWRCFG_SRAMPM_SHIFT)
#  define SYSCON_DSLPPWRCFG_SRAMPM_ACTIVE  (0 << SYSCON_DSLPPWRCFG_SRAMPM_SHIFT) /* Active Mode */
#  define SYSCON_DSLPPWRCFG_SRAMPM_STANDBY (1 << SYSCON_DSLPPWRCFG_SRAMPM_SHIFT) /* Standby Mode */
#  define SYSCON_DSLPPWRCFG_SRAMPM_LOWPWR  (2 << SYSCON_DSLPPWRCFG_SRAMPM_SHIFT) /* Low Power Mode */
#define SYSCON_DSLPPWRCFG_FLASHPM_SHIFT    (4)  /* Bits 5-4: Flash Power Modes */
#define SYSCON_DSLPPWRCFG_FLASHPM_MASK     (3 << SYSCON_DSLPPWRCFG_FLASHPM_SHIFT)
#  define SYSCON_DSLPPWRCFG_FLASHPM_ACTIVE (0 << SYSCON_DSLPPWRCFG_FLASHPM_SHIFT) /* Active Mode */
#  define SYSCON_DSLPPWRCFG_FLASHPM_LOWPWR (2 << SYSCON_DSLPPWRCFG_FLASHPM_SHIFT) /* Low Power Mode */

/* LDO Sleep Power Control */

#define SYSCON_LDOSPCTL_VLDO_SHIFT    (0)       /* Bits 7-0: LDO Output Voltage */
#define SYSCON_LDOSPCTL_VLDO_MASK     (0xff << SYSCON_LDOSPCTL_VLDO_SHIFT)
#  define SYSCON_LDOSPCTL_VLDO_0p90V  (0x12 << SYSCON_LDOSPCTL_VLDO_SHIFT) /* 0.90 V */
#  define SYSCON_LDOSPCTL_VLDO_0p95V  (0x13 << SYSCON_LDOSPCTL_VLDO_SHIFT) /* 0.95 V */
#  define SYSCON_LDOSPCTL_VLDO_1p00V  (0x14 << SYSCON_LDOSPCTL_VLDO_SHIFT) /* 1.00 V */
#  define SYSCON_LDOSPCTL_VLDO_1p05V  (0x15 << SYSCON_LDOSPCTL_VLDO_SHIFT) /* 1.05 V */
#  define SYSCON_LDOSPCTL_VLDO_1p10V  (0x16 << SYSCON_LDOSPCTL_VLDO_SHIFT) /* 1.10 V */
#  define SYSCON_LDOSPCTL_VLDO_1p15V  (0x17 << SYSCON_LDOSPCTL_VLDO_SHIFT) /* 1.15 V */
#  define SYSCON_LDOSPCTL_VLDO_1p20V  (0x18 << SYSCON_LDOSPCTL_VLDO_SHIFT) /* 1.20 V */
#define SYSCON_LDOSPCTL_VADJEN        (1 << 31) /* Bit 31: Voltage Adjust Enable */

/* LDO Sleep Power Calibration */

#define SYSCON_LDOSPCAL_NOPLL_SHIFT     (0)     /* Bits 7-0: Sleep without PLL */
#define SYSCON_LDOSPCAL_NOPLL_MASK      (0xff << SYSCON_LDOSPCAL_NOPLL_SHIFT)
#  define SYSCON_LDOSPCAL_NOPLL_0p90V   (0x12 << SYSCON_LDOSPCAL_NOPLL_SHIFT) /* 0.90 V */
#  define SYSCON_LDOSPCAL_NOPLL_0p95V   (0x13 << SYSCON_LDOSPCAL_NOPLL_SHIFT) /* 0.95 V */
#  define SYSCON_LDOSPCAL_NOPLL_1p00V   (0x14 << SYSCON_LDOSPCAL_NOPLL_SHIFT) /* 1.00 V */
#  define SYSCON_LDOSPCAL_NOPLL_1p05V   (0x15 << SYSCON_LDOSPCAL_NOPLL_SHIFT) /* 1.05 V */
#  define SYSCON_LDOSPCAL_NOPLL_1p10V   (0x16 << SYSCON_LDOSPCAL_NOPLL_SHIFT) /* 1.10 V */
#  define SYSCON_LDOSPCAL_NOPLL_1p15V   (0x17 << SYSCON_LDOSPCAL_NOPLL_SHIFT) /* 1.15 V */
#  define SYSCON_LDOSPCAL_NOPLL_1p20V   (0x18 << SYSCON_LDOSPCAL_NOPLL_SHIFT) /* 1.20 V */
#define SYSCON_LDOSPCAL_WITHPLL_SHIFT   (8)     /* Bits 15-8: Sleep with PLL */
#define SYSCON_LDOSPCAL_WITHPLL_MASK    (0xff << SYSCON_LDOSPCAL_WITHPLL_SHIFT)
#  define SYSCON_LDOSPCAL_WITHPLL_0p90V (0x12 << SYSCON_LDOSPCAL_WITHPLL_SHIFT) /* 0.90 V */
#  define SYSCON_LDOSPCAL_WITHPLL_0p95V (0x13 << SYSCON_LDOSPCAL_WITHPLL_SHIFT) /* 0.95 V */
#  define SYSCON_LDOSPCAL_WITHPLL_1p00V (0x14 << SYSCON_LDOSPCAL_WITHPLL_SHIFT) /* 1.00 V */
#  define SYSCON_LDOSPCAL_WITHPLL_1p05V (0x15 << SYSCON_LDOSPCAL_WITHPLL_SHIFT) /* 1.05 V */
#  define SYSCON_LDOSPCAL_WITHPLL_1p10V (0x16 << SYSCON_LDOSPCAL_WITHPLL_SHIFT) /* 1.10 V */
#  define SYSCON_LDOSPCAL_WITHPLL_1p15V (0x17 << SYSCON_LDOSPCAL_WITHPLL_SHIFT) /* 1.15 V */
#  define SYSCON_LDOSPCAL_WITHPLL_1p20V (0x18 << SYSCON_LDOSPCAL_WITHPLL_SHIFT) /* 1.20 V */

/* LDO Deep-Sleep Power Control */

#define SYSCON_LDODPCTL_VLDO_SHIFT    (0)       /* Bits 7-0: LDO Output Voltage */
#define SYSCON_LDODPCTL_VLDO_MASK     (0xff << SYSCON_LDODPCTL_VLDO_SHIFT)
#  define SYSCON_LDODPCTL_VLDO_0p90V  (0x12 << SYSCON_LDODPCTL_VLDO_SHIFT) /* 0.90 V */
#  define SYSCON_LDODPCTL_VLDO_0p95V  (0x13 << SYSCON_LDODPCTL_VLDO_SHIFT) /* 0.95 V */
#  define SYSCON_LDODPCTL_VLDO_1p00V  (0x14 << SYSCON_LDODPCTL_VLDO_SHIFT) /* 1.00 V */
#  define SYSCON_LDODPCTL_VLDO_1p05V  (0x15 << SYSCON_LDODPCTL_VLDO_SHIFT) /* 1.05 V */
#  define SYSCON_LDODPCTL_VLDO_1p10V  (0x16 << SYSCON_LDODPCTL_VLDO_SHIFT) /* 1.10 V */
#  define SYSCON_LDODPCTL_VLDO_1p15V  (0x17 << SYSCON_LDODPCTL_VLDO_SHIFT) /* 1.15 V */
#  define SYSCON_LDODPCTL_VLDO_1p20V  (0x18 << SYSCON_LDODPCTL_VLDO_SHIFT) /* 1.20 V */
#define SYSCON_LDODPCTL_VADJEN        (1 << 31) /* Bit 31: Voltage Adjust Enable */

/* LDO Deep-Sleep Power Calibration */

#define SYSCON_LDODPCAL_NOPLL_SHIFT   (0)       /* Bits 7-0: Deep-Sleep without PLL */
#define SYSCON_LDODPCAL_NOPLL_MASK    (0xff << SYSCON_LDODPCAL_NOPLL_SHIFT)
#  define SYSCON_LDODPCAL_NOPLL_0p90V (0x12 << SYSCON_LDODPCAL_NOPLL_SHIFT) /* 0.90 V */
#  define SYSCON_LDODPCAL_NOPLL_0p95V (0x13 << SYSCON_LDODPCAL_NOPLL_SHIFT) /* 0.95 V */
#  define SYSCON_LDODPCAL_NOPLL_1p00V (0x14 << SYSCON_LDODPCAL_NOPLL_SHIFT) /* 1.00 V */
#  define SYSCON_LDODPCAL_NOPLL_1p05V (0x15 << SYSCON_LDODPCAL_NOPLL_SHIFT) /* 1.05 V */
#  define SYSCON_LDODPCAL_NOPLL_1p10V (0x16 << SYSCON_LDODPCAL_NOPLL_SHIFT) /* 1.10 V */
#  define SYSCON_LDODPCAL_NOPLL_1p15V (0x17 << SYSCON_LDODPCAL_NOPLL_SHIFT) /* 1.15 V */
#  define SYSCON_LDODPCAL_NOPLL_1p20V (0x18 << SYSCON_LDODPCAL_NOPLL_SHIFT) /* 1.20 V */
#define SYSCON_LDODPCAL_30KHZ_SHIFT   (8)       /* Bits 15-8: Deep-Sleep with IOSC */
#define SYSCON_LDODPCAL_30KHZ_MASK    (0xff << SYSCON_LDODPCAL_30KHZ_SHIFT)
#  define SYSCON_LDODPCAL_30KHZ_0p90V (0x12 << SYSCON_LDODPCAL_30KHZ_SHIFT) /* 0.90 V */
#  define SYSCON_LDODPCAL_30KHZ_0p95V (0x13 << SYSCON_LDODPCAL_30KHZ_SHIFT) /* 0.95 V */
#  define SYSCON_LDODPCAL_30KHZ_1p00V (0x14 << SYSCON_LDODPCAL_30KHZ_SHIFT) /* 1.00 V */
#  define SYSCON_LDODPCAL_30KHZ_1p05V (0x15 << SYSCON_LDODPCAL_30KHZ_SHIFT) /* 1.05 V */
#  define SYSCON_LDODPCAL_30KHZ_1p10V (0x16 << SYSCON_LDODPCAL_30KHZ_SHIFT) /* 1.10 V */
#  define SYSCON_LDODPCAL_30KHZ_1p15V (0x17 << SYSCON_LDODPCAL_30KHZ_SHIFT) /* 1.15 V */
#  define SYSCON_LDODPCAL_30KHZ_1p20V (0x18 << SYSCON_LDODPCAL_30KHZ_SHIFT) /* 1.20 V */

/* Sleep / Deep-Sleep Power Mode Status */

#define SYSCON_SDPMST_SPDERR          (1 << 0)  /* Bit 0:  SRAM Power Down Request Error */
#define SYSCON_SDPMST_FPDERR          (1 << 1)  /* Bit 1:  Flash Memory Power Down Request Error */
#define SYSCON_SDPMST_PPDERR          (1 << 2)  /* Bit 2:  PIOSC Power Down Request Error */
#define SYSCON_SDPMST_LDMINERR        (1 << 3)  /* Bit 3:  VLDO Value Below Minimum Error in Deep-Sleep Mode */
#define SYSCON_SDPMST_LSMINERR        (1 << 4)  /* Bit 4:  VLDO Value Below Minimum Error in Sleep Mode */
#define SYSCON_SDPMST_LMAXERR         (1 << 6)  /* Bit 6:  VLDO Value Above Maximum Error */
#define SYSCON_SDPMST_PPDW            (1 << 7)  /* Bit 7:  PIOSC Power Down Request Warning */
#define SYSCON_SDPMST_PRACT           (1 << 16) /* Bit 16: Sleep or Deep-Sleep Power Request Active */
#define SYSCON_SDPMST_LOWPWR          (1 << 17) /* Bit 17: Sleep or Deep-Sleep Mode */
#define SYSCON_SDPMST_FLASHLP         (1 << 18) /* Bit 18: Flash Memory in Low Power State */
#define SYSCON_SDPMST_LDOUA           (1 << 19) /* Bit 19: LDO Update Active */

/* Watchdog Timer Peripheral Present */

#define SYSCON_PPWD(n)                (1 << (n)) /* Bit n:  WDTn present */
#  define SYSCON_PPWD_P0              (1 << 0)   /* Bit 0:  WDT0 present */
#  define SYSCON_PPWD_P1              (1 << 1)   /* Bit 1:  WDT1 present */

/* 16/32-Bit Timer Peripheral Present */

#define SYSCON_PPTIMER(n)             (1 << (n)) /* Bit n: 16/32-Bit General-Purpose Timer n Present */
#  define SYSCON_PPTIMER_P0           (1 << 0)   /* Bit 0: 16/32-Bit General-Purpose Timer 0 Present */
#  define SYSCON_PPTIMER_P1           (1 << 1)   /* Bit 1: 16/32-Bit General-Purpose Timer 0 Present */
#  define SYSCON_PPTIMER_P2           (1 << 2)   /* Bit 2: 16/32-Bit General-Purpose Timer 0 Present */
#  define SYSCON_PPTIMER_P3           (1 << 3)   /* Bit 3: 16/32-Bit General-Purpose Timer 0 Present */
#  define SYSCON_PPTIMER_P4           (1 << 4)   /* Bit 4: 16/32-Bit General-Purpose Timer 0 Present */
#  define SYSCON_PPTIMER_P5           (1 << 5)   /* Bit 5: 16/32-Bit General-Purpose Timer 0 Present */

/* GPIO Peripheral Present */

#define SYSCON_PPGPIO(n)              (1 << (n)) /* Bit n:  GPIO Port n Present */
#  define SYSCON_PPGPIO_P0            (1 << 0)   /* Bit 0:  GPIO Port A Present */
#  define SYSCON_PPGPIO_P1            (1 << 1)   /* Bit 1:  GPIO Port B Present */
#  define SYSCON_PPGPIO_P2            (1 << 2)   /* Bit 2:  GPIO Port C Present */
#  define SYSCON_PPGPIO_P3            (1 << 3)   /* Bit 3:  GPIO Port D Present */
#  define SYSCON_PPGPIO_P4            (1 << 4)   /* Bit 4:  GPIO Port E Present */
#  define SYSCON_PPGPIO_P5            (1 << 5)   /* Bit 5:  GPIO Port F Present */
#  define SYSCON_PPGPIO_P6            (1 << 6)   /* Bit 6:  GPIO Port G Present */
#  define SYSCON_PPGPIO_P7            (1 << 7)   /* Bit 7:  GPIO Port H Present */
#  define SYSCON_PPGPIO_P8            (1 << 8)   /* Bit 8:  GPIO Port J Present */
#  define SYSCON_PPGPIO_P9            (1 << 9)   /* Bit 9:  GPIO Port K Present */
#  define SYSCON_PPGPIO_P10           (1 << 10)  /* Bit 10: GPIO Port L Present */
#  define SYSCON_PPGPIO_P11           (1 << 11)  /* Bit 11: GPIO Port M Present */
#  define SYSCON_PPGPIO_P12           (1 << 12)  /* Bit 12: GPIO Port N Present */
#  define SYSCON_PPGPIO_P13           (1 << 13)  /* Bit 13: GPIO Port P Present */
#  define SYSCON_PPGPIO_P14           (1 << 14)  /* Bit 14: GPIO Port Q Present */

/* uDMA Peripheral Present */

#define SYSCON_PPDMA_P0               (1 << 0)   /* Bit 0:  μDMA Module Present */

/* Hibernation Peripheral Present */

#define SYSCON_PPHIB_P0               (1 << 0)   /* Bit 0: Hibernation Module Present */

/* UART Present */

#define SYSCON_PPUART(n)              (1 << (n)) /* Bit n:  UART Module n Present */
#  define SYSCON_PPUART_P0            (1 << 0)   /* Bit 0:  UART Module 0 Present */
#  define SYSCON_PPUART_P1            (1 << 1)   /* Bit 1:  UART Module 1 Present */
#  define SYSCON_PPUART_P2            (1 << 2)   /* Bit 2:  UART Module 2 Present */
#  define SYSCON_PPUART_P3            (1 << 3)   /* Bit 3:  UART Module 3 Present */
#  define SYSCON_PPUART_P4            (1 << 4)   /* Bit 4:  UART Module 4 Present */
#  define SYSCON_PPUART_P5            (1 << 5)   /* Bit 5:  UART Module 5 Present */
#  define SYSCON_PPUART_P6            (1 << 6)   /* Bit 6:  UART Module 6 Present */
#  define SYSCON_PPUART_P7            (1 << 7)   /* Bit 7:  UART Module 7 Present */

/* SSI Peripheral Present */

#define SYSCON_PPSSI(n)               (1 << (n)) /* Bit n:  SSI Module n Present */
#  define SYSCON_PPSSI_P0             (1 << 0)   /* Bit 0:  SSI Module 0 Present */
#  define SYSCON_PPSSI_P1             (1 << 1)   /* Bit 1:  SSI Module 1 Present */
#  define SYSCON_PPSSI_P2             (1 << 2)   /* Bit 2:  SSI Module 2 Present */
#  define SYSCON_PPSSI_P3             (1 << 3)   /* Bit 3:  SSI Module 3 Present */

/* I2C Peripheral Present */

#define SYSCON_PPI2C(n)               (1 << (n)) /* Bit n:  I2C Module n Present */
#  define SYSCON_PPI2C_P0             (1 << 0)   /* Bit 0:  I2C Module 0 Present */
#  define SYSCON_PPI2C_P1             (1 << 1)   /* Bit 1:  I2C Module 1 Present */
#  define SYSCON_PPI2C_P2             (1 << 2)   /* Bit 2:  I2C Module 2 Present */
#  define SYSCON_PPI2C_P3             (1 << 3)   /* Bit 3:  I2C Module 3 Present */
#  define SYSCON_PPI2C_P4             (1 << 4)   /* Bit 4:  I2C Module 4 Present */
#  define SYSCON_PPI2C_P5             (1 << 5)   /* Bit 5:  I2C Module 5 Present */

/* USB Peripheral Present */

#define SYSCON_PPUSB_P0               (1 << 0)   /* USB Module Present */

/* CAN Peripheral Present */

#define SYSCON_PPCAN(n)               (1 << (n)) /* Bit n:  CAN Module n Present */
#  define SYSCON_PPCAN_P0             (1 << 0)   /* Bit 0:  CAN Module 0 Present */
#  define SYSCON_PPCAN_P1             (1 << 1)   /* Bit 1:  CAN Module 1 Present */

/* ADC Peripheral Present */

#define SYSCON_PPADC(n)               (1 << (n)) /* Bit n:  ADC Module n Present */
#  define SYSCON_PPADC_P0             (1 << 0)   /* Bit 0:  ADC Module 0 Present */
#  define SYSCON_PPADC_P1             (1 << 1)   /* Bit 1:  ADC Module 1 Present */

/* Analog Comparator Peripheral Present */

#define SYSCON_PPACMP_P0              (1 << 0)   /* Bit 0:  Analog Comparator Module Present */

/* Pulse Width Modulator Peripheral Present */

#define SYSCON_PPWM(n)                (1 << (n)) /* Bit n:  PWM Module n Present */
#  define SYSCON_PPWM_P0              (1 << 0)   /* Bit 0:  PWM Module 0 Present */
#  define SYSCON_PPWM_P1              (1 << 1)   /* Bit 1:  PWM Module 1 Present */

/* Quadrature Encoder Peripheral Present */

#define SYSCON_PPQEI(n)               (1 << (n)) /* Bit n:  QEI Module n Present */
#  define SYSCON_PPQEI_P0             (1 << 0)   /* Bit 0:  QEI Module 0 Present */
#  define SYSCON_PPUART_P1            (1 << 1)   /* Bit 1:  QEI Module 1 Present */

/* EEPROM Peripheral Present */

#define SYSCON_PPEEPROM_P0            (1 << 0)   /* Bit 0:  EEPROM Module Present */

/* 32/64-Bit Wide Timer Peripheral Present */

#define SYSCON_PPWTIMER(n)            (1 << (n)) /* Bit n:  32/64-Bit Wide General-Purpose Timer n Present */
#  define SYSCON_PPWTIMER_P0          (1 << 0)   /* Bit 0:  32/64-Bit Wide General-Purpose Timer 0 Present */
#  define SYSCON_PPWTIMER_P1          (1 << 1)   /* Bit 1:  32/64-Bit Wide General-Purpose Timer 1 Present */
#  define SYSCON_PPWTIMER_P2          (1 << 2)   /* Bit 2:  32/64-Bit Wide General-Purpose Timer 2 Present */
#  define SYSCON_PPWTIMER_P3          (1 << 3)   /* Bit 3:  32/64-Bit Wide General-Purpose Timer 3 Present */
#  define SYSCON_PPWTIMER_P4          (1 << 4)   /* Bit 4:  32/64-Bit Wide General-Purpose Timer 4 Present */
#  define SYSCON_PPWTIMER_P5          (1 << 5)   /* Bit 5:  32/64-Bit Wide General-Purpose Timer 5 Present */

/* Watchdog Timer Software Reset */

#define SYSCON_SRWD(n)                (1 << (n)) /* Bit n:  Watchdog Timer n Software Reset */
#  define SYSCON_SRWD_R0              (1 << 0)   /* Bit 0:  Watchdog Timer 0 Software Reset */
#  define SYSCON_SRWD_R1              (1 << 1)   /* Bit 1:  Watchdog Timer 1 Software Reset */

/* 16/32-Bit Timer Software Reset */

#define SYSCON_SRTIMER(n)             (1 << (n)) /* Bit n:  16/32-Bit General-Purpose Timer n Software Reset */
#  define SYSCON_SRTIMER_R0           (1 << 0)   /* Bit 0:  16/32-Bit General-Purpose Timer 0 Software Reset */
#  define SYSCON_SRTIMER_R1           (1 << 1)   /* Bit 1:  16/32-Bit General-Purpose Timer 1 Software Reset */
#  define SYSCON_SRTIMER_R2           (1 << 2)   /* Bit 2:  16/32-Bit General-Purpose Timer 2 Software Reset */
#  define SYSCON_SRTIMER_R3           (1 << 3)   /* Bit 3:  16/32-Bit General-Purpose Timer 3 Software Reset */
#  define SYSCON_SRTIMER_R4           (1 << 4)   /* Bit 4:  16/32-Bit General-Purpose Timer 4 Software Reset */
#  define SYSCON_SRTIMER_R5           (1 << 5)   /* Bit 5:  16/32-Bit General-Purpose Timer 5 Software Reset */

/* GPIO Software Reset */

#define SYSCON_SRGPIO(n)              (1 << (n)) /* Bit n:  GPIO Port n Software Reset */
#  define SYSCON_SRGPIO_R0            (1 << 0)   /* Bit 0:  GPIO Port A Software Reset */
#  define SYSCON_SRGPIO_R1            (1 << 1)   /* Bit 1:  GPIO Port B Software Reset */
#  define SYSCON_SRGPIO_R2            (1 << 2)   /* Bit 2:  GPIO Port C Software Reset */
#  define SYSCON_SRGPIO_R3            (1 << 3)   /* Bit 3:  GPIO Port D Software Reset */
#  define SYSCON_SRGPIO_R4            (1 << 4)   /* Bit 4:  GPIO Port E Software Reset */
#  define SYSCON_SRPGIO_R5            (1 << 5)   /* Bit 5:  GPIO Port F Software Reset */
#  define SYSCON_SRPGIO_R6            (1 << 6)   /* Bit 6:  GPIO Port G Software Reset */
#  define SYSCON_SRPGIO_R7            (1 << 7)   /* Bit 7:  GPIO Port H Software Reset */
#  define SYSCON_SRPGIO_R8            (1 << 8)   /* Bit 8:  GPIO Port J Software Reset */
#  define SYSCON_SRPGIO_R9            (1 << 9)   /* Bit 9:  GPIO Port K Software Reset */
#  define SYSCON_SRPGIO_R10           (1 << 0)   /* Bit 0:  GPIO Port L Software Reset */
#  define SYSCON_SRPGIO_R11           (1 << 1)   /* Bit 1:  GPIO Port M Software Reset */
#  define SYSCON_SRPGIO_R12           (1 << 2)   /* Bit 2:  GPIO Port N Software Reset */
#  define SYSCON_SRPGIO_R13           (1 << 3)   /* Bit 3:  GPIO Port P Software Reset */
#  define SYSCON_SRPGIO_R14           (1 << 4)   /* Bit 4:  GPIO Port Q Software Reset */

/* uDMA Software Reset */

#define SYSCON_SRDMA_R0               (1 << 0)   /* Bit 0:  μDMA Module Software Reset */

/* Hibernation Software Reset */

#define SYSCON_SRHIB_R0               (1 << 0)   /* Bit 0:  Hibernation Module Software Reset */

/* UART Software Reset*/

#define SYSCON_SRUARTR(n)             (1 << (n)) /* Bit n:  UART Module n Software Reset */
#  define SYSCON_SRUARTR_R0           (1 << 0)   /* Bit 0:  UART Module 0 Software Reset */
#  define SYSCON_SRUARTR_R1           (1 << 1)   /* Bit 1:  UART Module 1 Software Reset */
#  define SYSCON_SRUARTR_R2           (1 << 2)   /* Bit 2:  UART Module 2 Software Reset */
#  define SYSCON_SRUARTR_R3           (1 << 3)   /* Bit 3:  UART Module 3 Software Reset */
#  define SYSCON_SRUARTR_R4           (1 << 4)   /* Bit 4:  UART Module 4 Software Reset */
#  define SYSCON_SRUARTR_R5           (1 << 5)   /* Bit 5:  UART Module 5 Software Reset */
#  define SYSCON_SRUARTR_R6           (1 << 6)   /* Bit 6:  UART Module 6 Software Reset */
#  define SYSCON_SRUARTR_R7           (1 << 7)   /* Bit 7:  UART Module 7 Software Reset */

/* SSI Software Reset */

#define SYSCON_SRSSI(n)               (1 << (n)) /* Bit n:  SSI Module n Software Reset */
#  define SYSCON_SRSSI_R0             (1 << 0)   /* Bit 0:  SSI Module 0 Software Reset */
#  define SYSCON_SRSSI_R1             (1 << 1)   /* Bit 1:  SSI Module 1 Software Reset */
#  define SYSCON_SRSSI_R2             (1 << 2)   /* Bit 2:  SSI Module 2 Software Reset */
#  define SYSCON_SRSSI_R3             (1 << 3)   /* Bit 3:  SSI Module 3 Software Reset */

/* I2C Software Reset */

#define SYSCON_SRI2C(n)               (1 << (n)) /* Bit n:  I2C Module n Software Reset */
#  define SYSCON_SRI2C_R0             (1 << 0)   /* Bit 0:  I2C Module 0 Software Reset */
#  define SYSCON_SRI2C_R1             (1 << 1)   /* Bit 1:  I2C Module 1 Software Reset */
#  define SYSCON_SRI2C_R2             (1 << 2)   /* Bit 2:  I2C Module 2 Software Reset */
#  define SYSCON_SRI2C_R3             (1 << 3)   /* Bit 3:  I2C Module 3 Software Reset */
#  define SYSCON_SRI2C_R4             (1 << 4)   /* Bit 4:  I2C Module 4 Software Reset */
#  define SYSCON_SRI2C_R5             (1 << 5)   /* Bit 5:  I2C Module 5 Software Reset */

/* USB Software Reset */

#define SYSCON_SRUSB_R0               (1 << 0)   /* Bit 0:  USB Module Software Reset */

/* CAN Software Reset */

#define SYSCON_SRCAN(n)               (1 << (n)) /* Bit n:  CAN Module n Software Reset */
#  define SYSCON_SRCAN_R0             (1 << 0)   /* Bit 0:  CAN Module 0 Software Reset */
#  define SYSCON_SRCAN_R1             (1 << 1)   /* Bit 1:  CAN Module 1 Software Reset*/

/* ADC Software Reset */

#define SYSCON_SRADC(n)               (1 << (n)) /* Bit n:  ADC Module n Software Reset */
#  define SYSCON_SRADC_R0             (1 << 0)   /* Bit 0:  ADC Module 0 Software Reset */
#  define SYSCON_SRADC_R1             (1 << 1)   /* Bit 1:  ADC Module 1 Software Reset */

/* Analog Comparator Software Reset */

#define SYSCON_SRACMP_R0              (1 << 0)   /* Bit 0:  Analog Comparator Module 0 Software Reset */

/* Pulse Width Modulator Software Reset */

#define SYSCON_SRPWM(n)               (1 << (n)) /* Bit n:  PWM Module n Software Reset */
#  define SYSCON_SRPWM_R0             (1 << 0)   /* Bit 0:  PWM Module 0 Software Reset */
#  define SYSCON_SRPWM_R1             (1 << 1)   /* Bit 1:  PWM Module 1 Software Reset */

/* Quadrature Encoder Interface Software Reset */
#define SYSCON_SRQEI_
#define SYSCON_SRQEI(n)               (1 << (n)) /* Bit n:  QEI Module n Software Reset */
#  define SYSCON_SRQEI_R0             (1 << 0)   /* Bit 0:  QEI Module 0 Software Reset */
#  define SYSCON_SRQEI_R1             (1 << 1)   /* Bit 1:  QEI Module 1 Software Reset */

/* EEPROM Software Reset */

#define SYSCON_SREEPROM_R0            (1 << 0)   /* Bit 0:  EEPROM Module Software Reset */

/* 32/64-Bit Wide Timer Software Reset */

#define SYSCON_SRWTIMER(n)            (1 << (n)) /* Bit n:  32/64-Bit Wide General-Purpose Timer n Software Reset */
#  define SYSCON_SRWTIMER_R0          (1 << 0)   /* Bit 0:  32/64-Bit Wide General-Purpose Timer 0 Software Reset */
#  define SYSCON_SRWTIMER_R1          (1 << 1)   /* Bit 1:  32/64-Bit Wide General-Purpose Timer 1 Software Reset */
#  define SYSCON_SRWTIMER_R2          (1 << 2)   /* Bit 2:  32/64-Bit Wide General-Purpose Timer 2 Software Reset */
#  define SYSCON_SRWTIMER_R3          (1 << 3)   /* Bit 3:  32/64-Bit Wide General-Purpose Timer 3 Software Reset */
#  define SYSCON_SRWTIMER_R4          (1 << 4)   /* Bit 4:  32/64-Bit Wide General-Purpose Timer 4 Software Reset */
#  define SYSCON_SRWTIMER_R5          (1 << 5)   /* Bit 5:  32/64-Bit Wide General-Purpose Timer 5 Software Reset */

/* Watchdog Timer Run Mode Clock Gating Control */

#define SYSCON_RCGCWD(n)              (1 << (n)) /* Bit n:  Watchdog Timer n Run Mode Clock Gating Control */
#  define SYSCON_RCGCWD_R0            (1 << 0)   /* Bit 0:  Watchdog Timer 0 Run Mode Clock Gating Control */
#  define SYSCON_RCGCWD_R1            (1 << 1)   /* Bit 1:  Watchdog Timer 1 Run Mode Clock Gating Control */

/* 16/32-Bit Timer Run Mode Clock Gating Control */

#define SYSCON_RCGCTIMER(n)           (1 << (n)) /* Bit n:  16/32-Bit General-Purpose Timer n Run Mode Clock Gating Control */
#  define SYSCON_RCGCTIMER_R0         (1 << 0)   /* Bit 0:  16/32-Bit General-Purpose Timer 0 Run Mode Clock Gating Control */
#  define SYSCON_RCGCTIMER_R1         (1 << 1)   /* Bit 1:  16/32-Bit General-Purpose Timer 1 Run Mode Clock Gating Control */
#  define SYSCON_RCGCTIMER_R2         (1 << 2)   /* Bit 2:  16/32-Bit General-Purpose Timer 2 Run Mode Clock Gating Control */
#  define SYSCON_RCGCTIMER_R3         (1 << 3)   /* Bit 3:  16/32-Bit General-Purpose Timer 3 Run Mode Clock Gating Control */
#  define SYSCON_RCGCTIMER_R4         (1 << 4)   /* Bit 4:  16/32-Bit General-Purpose Timer 4 Run Mode Clock Gating Control */
#  define SYSCON_RCGCTIMER_R5         (1 << 5)   /* Bit 5:  16/32-Bit General-Purpose Timer 5 Run Mode Clock Gating Control */

/* GPIO Run Mode Clock Gating Control*/

#define SYSCON_RCGCGPIO(n)            (1 << (n)) /* Bit n:  16/32-Bit GPIO Port n Run Mode Clock Gating Control */
#  define SYSCON_RCGCGPIO_R0          (1 << 0)   /* Bit 0:  16/32-Bit GPIO Port A Run Mode Clock Gating Control */
#  define SYSCON_RCGCGPIO_R1          (1 << 1)   /* Bit 1:  16/32-Bit GPIO Port B Run Mode Clock Gating Control */
#  define SYSCON_RCGCGPIO_R2          (1 << 2)   /* Bit 2:  16/32-Bit GPIO Port C Run Mode Clock Gating Control */
#  define SYSCON_RCGCGPIO_R3          (1 << 3)   /* Bit 3:  16/32-Bit GPIO Port D Run Mode Clock Gating Control */
#  define SYSCON_RCGCGPIO_R4          (1 << 4)   /* Bit 4:  16/32-Bit GPIO Port E Run Mode Clock Gating Control */
#  define SYSCON_RCGCGPIO_R5          (1 << 5)   /* Bit 5:  16/32-Bit GPIO Port F Run Mode Clock Gating Control */
#  define SYSCON_RCGCGPIO_R6          (1 << 6)   /* Bit 6:  16/32-Bit GPIO Port G Run Mode Clock Gating Control */
#  define SYSCON_RCGCGPIO_R7          (1 << 7)   /* Bit 7:  16/32-Bit GPIO Port H Run Mode Clock Gating Control */
#  define SYSCON_RCGCGPIO_R8          (1 << 8)   /* Bit 8:  16/32-Bit GPIO Port J Run Mode Clock Gating Control */
#  define SYSCON_RCGCGPIO_R9          (1 << 9)   /* Bit 9:  16/32-Bit GPIO Port K Run Mode Clock Gating Control */
#  define SYSCON_RCGCGPIO_R10         (1 << 10)  /* Bit 10: 16/32-Bit GPIO Port L Run Mode Clock Gating Control */
#  define SYSCON_RCGCGPIO_R11         (1 << 11)  /* Bit 11: 16/32-Bit GPIO Port M Run Mode Clock Gating Control */
#  define SYSCON_RCGCGPIO_R12         (1 << 12)  /* Bit 12: 16/32-Bit GPIO Port N Run Mode Clock Gating Control */
#  define SYSCON_RCGCGPIO_R13         (1 << 13)  /* Bit 13: 16/32-Bit GPIO Port P Run Mode Clock Gating Control */
#  define SYSCON_RCGCGPIO_R14         (1 << 14)  /* Bit 14: 16/32-Bit GPIO Port Q Run Mode Clock Gating Control */

/* uDMA Run Mode Clock Gating Control*/

#define SYSCON_RCGCDMA_R0             (1 << 0)   /* Bit 0:  μDMA Module Run Mode Clock Gating Control */

/* Hibernation Run Mode Clock Gating Control */

#define SYSCON_RCGCHIB_R0             (1 << 0)   /* Bit 0:  Hibernation Module Run Mode Clock Gating Control */

/* UART Run Mode Clock Gating Control*/

#define SYSCON_RCGCUART(n)            (1 << (n)) /* Bit n:  UART Module n Run Mode Clock Gating Control */
#  define SYSCON_RCGCUART_R0          (1 << 0)   /* Bit 0:  UART Module 0 Run Mode Clock Gating Control */
#  define SYSCON_RCGCUART_R1          (1 << 1)   /* Bit 1:  UART Module 1 Run Mode Clock Gating Control */
#  define SYSCON_RCGCUART_R2          (1 << 2)   /* Bit 2:  UART Module 2 Run Mode Clock Gating Control */
#  define SYSCON_RCGCUART_R3          (1 << 3)   /* Bit 3:  UART Module 3 Run Mode Clock Gating Control */
#  define SYSCON_RCGCUART_R4          (1 << 4)   /* Bit 4:  UART Module 4 Run Mode Clock Gating Control */
#  define SYSCON_RCGCUART_R5          (1 << 5)   /* Bit 5:  UART Module 5 Run Mode Clock Gating Control */
#  define SYSCON_RCGCUART_R6          (1 << 6)   /* Bit 6:  UART Module 6 Run Mode Clock Gating Control */
#  define SYSCON_RCGCUART_R7          (1 << 7)   /* Bit 7:  UART Module 7 Run Mode Clock Gating Control */

/* SSI Run Mode Clock Gating Control*/

#define SYSCON_RCGCSSI(n)             (1 << (n)) /* Bit n:  SSI Module n Run Mode Clock Gating Control */
#  define SYSCON_RCGCSSI_R0           (1 << 0)   /* Bit 0:  SSI Module 0 Run Mode Clock Gating Control */
#  define SYSCON_RCGCSSI_R1           (1 << 1)   /* Bit 1:  SSI Module 1 Run Mode Clock Gating Control */
#  define SYSCON_RCGCSSI_R2           (1 << 2)   /* Bit 2:  SSI Module 2 Run Mode Clock Gating Control */
#  define SYSCON_RCGCSSI_R3           (1 << 3)   /* Bit 3:  SSI Module 3 Run Mode Clock Gating Control */

/* I2C Run Mode Clock Gating Control */

#define SYSCON_RCGCI2C(n)             (1 << (n)) /* Bit n:  I2C Module n Run Mode Clock Gating Control */
#  define SYSCON_RCGCI2C_R0           (1 << 0)   /* Bit 0:  I2C Module 0 Run Mode Clock Gating Control */
#  define SYSCON_RCGCI2C_R1           (1 << 1)   /* Bit 1:  I2C Module 1 Run Mode Clock Gating Control */
#  define SYSCON_RCGCI2C_R2           (1 << 2)   /* Bit 2:  I2C Module 2 Run Mode Clock Gating Control */
#  define SYSCON_RCGCI2C_R3           (1 << 3)   /* Bit 3:  I2C Module 3 Run Mode Clock Gating Control */
#  define SYSCON_RCGCI2C_R4           (1 << 4)   /* Bit 4:  I2C Module 4 Run Mode Clock Gating Control */
#  define SYSCON_RCGCI2C_R5           (1 << 5)   /* Bit 5:  I2C Module 5 Run Mode Clock Gating Control */

/* USB Run Mode Clock Gating Control */

#define SYSCON_RCGCUSB_R0             (1 << 0)   /* Bit 0:  USB Module Run Mode Clock Gating Control */

/* CAN Run Mode Clock Gating Control */

#define SYSCON_RCGCCAN(n)             (1 << (n)) /* Bit n:  CAN Module n Run Mode Clock Gating Control */
#  define SYSCON_RCGCCAN_R0           (1 << 0)   /* Bit 0:  CAN Module 0 Run Mode Clock Gating Control */
#  define SYSCON_RCGCCAN_R1           (1 << 1)   /* Bit 1:  CAN Module 1 Run Mode Clock Gating Control */

/* ADC Run Mode Clock Gating Control */

#define SYSCON_RCGCADC(n)             (1 << (n)) /* Bit n:  ADC Module n Run Mode Clock Gating Control */
#  define SYSCON_RCGCADC_R0           (1 << 0)   /* Bit 0:  ADC Module 0 Run Mode Clock Gating Control */
#  define SYSCON_RCGCADC_R1           (1 << 1)   /* Bit 1:  ADC Module 1 Run Mode Clock Gating Control */

/* Analog Comparator Run Mode Clock Gating Control */

#define SYSCON_RCGCACMP_R0            (1 << 0)   /* Bit 0:  Analog Comparator Module 0 Run Mode Clock Gating Control */

/* Pulse Width Modulator Run Mode Clock Gating Control */

#define SYSCON_RCGCPWM(n)             (1 << (n)) /* Bit n:  PWM Module n Run Mode Clock Gating Control */
#  define SYSCON_RCGCPWM_R0           (1 << 0)   /* Bit 0:  PWM Module 0 Run Mode Clock Gating Control */
#  define SYSCON_RCGCPWM_R1           (1 << 1)   /* Bit 1:  PWM Module 1 Run Mode Clock Gating Control */

/* Quadrature Encoder Interface Run Mode Clock Gating Control */

#define SYSCON_RCGCQEI(n)             (1 << (n)) /* Bit n:  QEI Module n Run Mode Clock Gating Control */
#  define SYSCON_RCGCQEI_R0           (1 << 0)   /* Bit 0:  QEI Module 0 Run Mode Clock Gating Control */
#  define SYSCON_RCGCQEI_R1           (1 << 1)   /* Bit 1:  QEI Module 1 Run Mode Clock Gating Control */

/* EEPROM Run Mode Clock Gating Control */

#define SYSCON_RCGCEEPROM_R0          (1 << 0)   /* Bit 0:  EEPROM Module Run Mode Clock Gating Control */

/* 32/64-BitWide Timer Run Mode Clock Gating Control */

#define SYSCON_RCGCWTIMER(n)          (1 << (n)) /* Bit n:  32/64-Bit Wide General-Purpose Timer n Run Mode Clock Gating Control */
#  define SYSCON_RCGCWTIMER_R0        (1 << 0)   /* Bit 0:  32/64-Bit Wide General-Purpose Timer 0 Run Mode Clock Gating Control */
#  define SYSCON_RCGCWTIMER_R1        (1 << 1)   /* Bit 1:  32/64-Bit Wide General-Purpose Timer 1 Run Mode Clock Gating Control */
#  define SYSCON_RCGCWTIMER_R2        (1 << 2)   /* Bit 2:  32/64-Bit Wide General-Purpose Timer 2 Run Mode Clock Gating Control */
#  define SYSCON_RCGCWTIMER_R3        (1 << 3)   /* Bit 3:  32/64-Bit Wide General-Purpose Timer 3 Run Mode Clock Gating Control */
#  define SYSCON_RCGCWTIMER_R4        (1 << 4)   /* Bit 4:  32/64-Bit Wide General-Purpose Timer 4 Run Mode Clock Gating Control */
#  define SYSCON_RCGCWTIMER_R5        (1 << 5)   /* Bit 5:  32/64-Bit Wide General-Purpose Timer 5 Run Mode Clock Gating Control */

/* Watchdog Timer Sleep Mode Clock Gating Control */

#define SYSCON_SCGCWD(n)              (1 << (n)) /* Bit n:  Watchdog Timer n Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCWD_S0            (1 << 0)   /* Bit 0:  Watchdog Timer 0 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCWD_S1            (1 << 1)   /* Bit 1:  Watchdog Timer 1 Sleep Mode Clock Gating Control */

/* 16/32-Bit Timer Sleep Mode Clock Gating Control */

#define SYSCON_SCGCWD(n)              (1 << (n)) /* Bit n:  16/32-Bit General-Purpose Timer n Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCWD_S0            (1 << 0)   /* Bit 0:  16/32-Bit General-Purpose Timer 0 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCWD_S1            (1 << 1)   /* Bit 1:  16/32-Bit General-Purpose Timer 1 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCWD_S2            (1 << 2)   /* Bit 2:  16/32-Bit General-Purpose Timer 2 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCWD_S3            (1 << 3)   /* Bit 3:  16/32-Bit General-Purpose Timer 3 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCWD_S4            (1 << 4)   /* Bit 4:  16/32-Bit General-Purpose Timer 4 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCWD_S5            (1 << 5)   /* Bit 5:  16/32-Bit General-Purpose Timer 5 Sleep Mode Clock Gating Control */

/* GPIO Sleep Mode Clock Gating Control */

#define SYSCON_SCGCGPIO(n)            (1 << (n)) /* Bit n:  GPIO Port n Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCGPIO_S0          (1 << 0)   /* Bit 0:  GPIO Port A Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCGPIO_S1          (1 << 1)   /* Bit 1:  GPIO Port B Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCGPIO_S2          (1 << 2)   /* Bit 2:  GPIO Port C Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCGPIO_S3          (1 << 3)   /* Bit 3:  GPIO Port D Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCGPIO_S4          (1 << 4)   /* Bit 4:  GPIO Port E Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCGPIO_S5          (1 << 5)   /* Bit 5:  GPIO Port F Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCGPIO_S6          (1 << 6)   /* Bit 6:  GPIO Port G Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCGPIO_S7          (1 << 7)   /* Bit 7:  GPIO Port H Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCGPIO_S8          (1 << 8)   /* Bit 8:  GPIO Port J Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCGPIO_S9          (1 << 9)   /* Bit 9:  GPIO Port K Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCGPIO_S10         (1 << 10)  /* Bit 10:  GPIO Port L Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCGPIO_S11         (1 << 11)  /* Bit 11:  GPIO Port M Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCGPIO_S12         (1 << 12)  /* Bit 12:  GPIO Port N Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCGPIO_S13         (1 << 13)  /* Bit 13:  GPIO Port P Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCGPIO_S14         (1 << 14)  /* Bit 14:  GPIO Port Q Sleep Mode Clock Gating Control */

/* uDMA Sleep Mode Clock Gating Control */

#define SYSCON_SCGCDMA_S0             (1 << 0)   /* Bit 0:  μDMA Module Sleep Mode Clock Gating Control */

/* Hibernation Sleep Mode Clock Gating Control */

#define SYSCON_SCGCHIB_S0             (1 << 0)   /* Bit 0:  Hibernation Module Sleep Mode Clock Gating Control */

/* UART Sleep Mode Clock Gating Control */

#define SYSCON_SCGCUART(n)            (1 << (n)) /* Bit n:  UART Module n Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCUART_S0          (1 << 0)   /* Bit 0:  UART Module 0 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCUART_S1          (1 << 1)   /* Bit 1:  UART Module 1 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCUART_S2          (1 << 2)   /* Bit 2:  UART Module 2 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCUART_S3          (1 << 3)   /* Bit 3:  UART Module 3 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCUART_S4          (1 << 4)   /* Bit 4:  UART Module 4 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCUART_S5          (1 << 5)   /* Bit 5:  UART Module 5 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCUART_S6          (1 << 6)   /* Bit 6:  UART Module 6 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCUART_S7          (1 << 7)   /* Bit 7:  UART Module 7 Sleep Mode Clock Gating Control */

/* SSI Sleep Mode Clock Gating Control */

#define SYSCON_SCGCSSI(n)             (1 << (n)) /* Bit n:  SSI Module n Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCSSI_S0           (1 << 0)   /* Bit 0:  SSI Module 0 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCSSI_S1           (1 << 1)   /* Bit 1:  SSI Module 1 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCSSI_S2           (1 << 2)   /* Bit 2:  SSI Module 2 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCSSI_S3           (1 << 3)   /* Bit 3:  SSI Module 3 Sleep Mode Clock Gating Control */

/* I2C Sleep Mode Clock Gating Control */

#define SYSCON_SCGCI2C(n)             (1 << (n)) /* Bit n:  I2C Module n Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCI2C_S0           (1 << 0)   /* Bit 0:  I2C Module 0 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCI2C_S1           (1 << 1)   /* Bit 1:  I2C Module 1 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCI2C_S2           (1 << 2)   /* Bit 2:  I2C Module 2 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCI2C_S3           (1 << 3)   /* Bit 3:  I2C Module 3 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCI2C_S4           (1 << 4)   /* Bit 4:  I2C Module 4 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCI2C_S5           (1 << 5)   /* Bit 5:  I2C Module 5 Sleep Mode Clock Gating Control */

/* USB Sleep Mode Clock Gating Control */

#define SYSCON_SCGCUSB_S0             (1 << 0)   /* Bit 0:  USB Module Sleep Mode Clock Gating Control */

/* CAN Sleep Mode Clock Gating Control */

#define SYSCON_SCGCCAN(n)             (1 << (n)) /* Bit n:  CAN Module n Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCCAN_S0           (1 << 0)   /* Bit 0:  CAN Module 0 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCCAN_S1           (1 << 1)   /* Bit 1:  CAN Module 1 Sleep Mode Clock Gating Control */

/* ADC Sleep Mode Clock Gating Control */

#define SYSCON_SCGCADC(n)             (1 << (n)) /* Bit n:  ADC Module n Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCADC_S0           (1 << 0)   /* Bit 0:  ADC Module 0 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCADC_S1           (1 << 1)   /* Bit 1:  ADC Module 1 Sleep Mode Clock Gating Control */

/* Analog Comparator Sleep Mode Clock Gating Control */

#define SYSCON_SCGCACMP_S0            (1 << 0)   /* Bit 0:  Analog Comparator Module 0 Sleep Mode Clock Gating Control */

/* PulseWidthModulator Sleep Mode Clock Gating Control */

#define SYSCON_SCGCPWM(n)             (1 << (n)) /* Bit n:  PWM Module n Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCPWM_S0           (1 << 0)   /* Bit 0:  PWM Module 0 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCPWM_S1           (1 << 1)   /* Bit 1:  PWM Module 1 Sleep Mode Clock Gating Control */

/* Quadrature Encoder Interface Sleep Mode Clock Gating Control */

#define SYSCON_SCGCQEI(n)             (1 << (n)) /* Bit n:  QEI Module n Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCQEI_S0           (1 << 0)   /* Bit 0:  QEI Module 0 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCQEI_S1           (1 << 1)   /* Bit 1:  QEI Module 1 Sleep Mode Clock Gating Control */

/* EEPROM Sleep Mode Clock Gating Control */

#define SYSCON_SCGCEEPROM_S0          (1 << 0)   /* Bit 0:  EEPROM Module Sleep Mode Clock Gating Control */

/* 32/64-BitWide Timer Sleep Mode Clock Gating Control */

#define SYSCON_SCGCWTIMER(n)          (1 << (n)) /* Bit n:  32/64-Bit Wide General-Purpose Timer n Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCWTIMER_S0        (1 << 0)   /* Bit 0:  32/64-Bit Wide General-Purpose Timer 0 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCWTIMER_S1        (1 << 1)   /* Bit 1:  32/64-Bit Wide General-Purpose Timer 1 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCWTIMER_S2        (1 << 2)   /* Bit 2:  32/64-Bit Wide General-Purpose Timer 2 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCWTIMER_S3        (1 << 3)   /* Bit 3:  32/64-Bit Wide General-Purpose Timer 3 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCWTIMER_S4        (1 << 4)   /* Bit 4:  32/64-Bit Wide General-Purpose Timer 4 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCWTIMER_S5        (1 << 5)   /* Bit 5:  32/64-Bit Wide General-Purpose Timer 5 Sleep Mode Clock Gating Control */

/* Watchdog Timer Deep-Sleep Mode Clock Gating Control */

#define SYSCON_DCGCWD(n)              (1 << (n)) /* Bit n:  Watchdog Timer n Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCWD_D0            (1 << 0)   /* Bit 0:  Watchdog Timer 0 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCWD_D1            (1 << 1)   /* Bit 1:  Watchdog Timer 1 Deep-Sleep Mode Clock Gating Control */

/* Clock Gating Control */

#define SYSCON_DCGCTIMER(n)           (1 << (n)) /* Bit n:  16/32-Bit General-Purpose Timer n Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCTIMER_D0         (1 << 0)   /* Bit 0:  16/32-Bit General-Purpose Timer 0 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCTIMER_D1         (1 << 1)   /* Bit 1:  16/32-Bit General-Purpose Timer 1 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCTIMER_D2         (1 << 2)   /* Bit 2:  16/32-Bit General-Purpose Timer 2 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCTIMER_D3         (1 << 3)   /* Bit 3:  16/32-Bit General-Purpose Timer 3 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCTIMER_D4         (1 << 4)   /* Bit 4:  16/32-Bit General-Purpose Timer 4 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCTIMER_D5         (1 << 5)   /* Bit 5:  16/32-Bit General-Purpose Timer 5 Deep-Sleep Mode Clock Gating Control */

/* GPIO Deep-Sleep Mode Clock Gating Control */

#define SYSCON_DCGCGPIO(n)            (1 << (n)) /* Bit n:  GPIO Port F Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCGPIO_D0          (1 << 0)   /* Bit 0:  GPIO Port A Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCGPIO_D1          (1 << 1)   /* Bit 1:  GPIO Port B Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCGPIO_D2          (1 << 2)   /* Bit 2:  GPIO Port C Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCGPIO_D3          (1 << 3)   /* Bit 3:  GPIO Port D Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCGPIO_D4          (1 << 4)   /* Bit 4:  GPIO Port E Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCGPIO_D5          (1 << 5)   /* Bit 5:  GPIO Port F Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCGPIO_D6          (1 << 6)   /* Bit 6:  GPIO Port G Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCGPIO_D7          (1 << 7)   /* Bit 7:  GPIO Port H Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCGPIO_D8          (1 << 8)   /* Bit 8:  GPIO Port J Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCGPIO_D9          (1 << 9)   /* Bit 9:  GPIO Port K Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCGPIO_D10         (1 << 10)  /* Bit 10: GPIO Port L Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCGPIO_D11         (1 << 11)  /* Bit 11: GPIO Port M Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCGPIO_D12         (1 << 12)  /* Bit 12: GPIO Port N Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCGPIO_D13         (1 << 13)  /* Bit 13: GPIO Port P Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCGPIO_D14         (1 << 14)  /* Bit 14: GPIO Port Q Deep-Sleep Mode Clock Gating Control */

/* uDMA Deep-Sleep Mode Clock Gating Control */

#define SYSCON_DCGCDMA_D0             (1 << 0)   /* Bit 0:  μDMA Module Deep-Sleep Mode Clock Gating Control */

/* Hibernation Deep-Sleep Mode Clock Gating Control */

#define SYSCON_DCGCHIB_D0             (1 << 0)   /* Bit 0:  Hibernation Module Deep-Sleep Mode Clock Gating Control */

/* UART Deep-Sleep Mode Clock Gating Control */

#define SYSCON_DCGCUART(n)            (1 << (n)) /* Bit n:  UART Module n Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCUART_D0          (1 << 0)   /* Bit 0:  UART Module 0 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCUART_D1          (1 << 1)   /* Bit 1:  UART Module 1 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCUART_D2          (1 << 2)   /* Bit 2:  UART Module 2 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCUART_D3          (1 << 3)   /* Bit 3:  UART Module 3 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCUART_D4          (1 << 4)   /* Bit 4:  UART Module 4 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCUART_D5          (1 << 5)   /* Bit 5:  UART Module 5 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCUART_D6          (1 << 6)   /* Bit 6:  UART Module 6 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCUART_D7          (1 << 7)   /* Bit 7:  UART Module 7 Deep-Sleep Mode Clock Gating Control */

/* SSI Deep-Sleep Mode Clock Gating Control */

#define SYSCON_DCGCSSI(n)             (1 << (n)) /* Bit n:  SSI Module n Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCSSI_D0           (1 << 0)   /* Bit 0:  SSI Module 0 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCSSI_D1           (1 << 1)   /* Bit 1:  SSI Module 1 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCSSI_D2           (1 << 2)   /* Bit 2:  SSI Module 2 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCSSI_D3           (1 << 3)   /* Bit 3:  SSI Module 3 Deep-Sleep Mode Clock Gating Control */

/* I2C Deep-Sleep Mode Clock Gating Control */

#define SYSCON_DCGCI2C(n)             (1 << (n)) /* Bit n:  I2C Module n Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCI2C_D0           (1 << 0)   /* Bit 0:  I2C Module 0 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCI2C_D1           (1 << 1)   /* Bit 1:  I2C Module 1 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCI2C_D2           (1 << 2)   /* Bit 2:  I2C Module 2 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCI2C_D3           (1 << 3)   /* Bit 3:  I2C Module 3 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCI2C_D4           (1 << 4)   /* Bit 4:  I2C Module 4 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCI2C_D5           (1 << 5)   /* Bit 5:  I2C Module 5 Deep-Sleep Mode Clock Gating Control */

/* USB Deep-Sleep Mode Clock Gating Control */

#define SYSCON_DCGCUSB_D0             (1 << 0)   /* Bit 0:  USB Module Deep-Sleep Mode Clock Gating Control */

/* CAN Deep-Sleep Mode Clock Gating Control */

#define SYSCON_DCGCCAN(n)             (1 << (n)) /* Bit n:  CAN Module n Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCCAN_D0           (1 << 0)   /* Bit 0:  CAN Module 0 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCCAN_D1           (1 << 1)   /* Bit 1:  CAN Module 1 Deep-Sleep Mode Clock Gating Control */

/* ADC Deep-Sleep Mode Clock Gating Control */

#define SYSCON_DCGCADC(n)             (1 << (n)) /* Bit n:  ADC Module n Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCADC_D0           (1 << 0)   /* Bit 0:  ADC Module 0 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCADC_D1           (1 << 1)   /* Bit 1:  ADC Module 1 Deep-Sleep Mode Clock Gating Control */

/* Analog Comparator Deep-Sleep Mode Clock Gating Control */

#define SYSCON_DCGCACMP_D0            (1 << 0)   /* Bit 0:  Analog Comparator Module 0 Deep-Sleep Mode Clock Gating Control */

/* Pulse Width Modulator Deep-Sleep Mode Clock Gating Control */

#define SYSCON_DCGCPWM(n)             (1 << (n)) /* Bit n:  PWM Module n Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCPWM_D0           (1 << 0)   /* Bit 0:  PWM Module 0 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCPWM_D1           (1 << 1)   /* Bit 1:  PWM Module 1 Deep-Sleep Mode Clock Gating Control */

/* Quadrature Encoder Interface Deep-Sleep Mode Clock Gating Control */

#define SYSCON_DCGCQEI(n)             (1 << (n)) /* Bit n:  QEI Module n Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCQEI_D0           (1 << 0)   /* Bit 0:  QEI Module 0 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCQEI_D1           (1 << 1)   /* Bit 1:  QEI Module 1 Deep-Sleep Mode Clock Gating Control */

/* EEPROM Deep-Sleep Mode Clock Gating Control */

#define SYSCON_DCGCEEPROM_D0          (1 << 0)   /* Bit 0:  EEPROM Module Deep-Sleep Mode Clock Gating Control */

/* 32/64-BitWide Timer Deep-Sleep Mode Clock Gating Control */

#define SYSCON_DCGCWTIMER(n)          (1 << (n)) /* Bit n:  UART Module n Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCWTIMER_D0        (1 << 0)   /* Bit 0:  UART Module 0 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCWTIMER_D1        (1 << 1)   /* Bit 1:  UART Module 1 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCWTIMER_D2        (1 << 2)   /* Bit 2:  UART Module 2 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCWTIMER_D3        (1 << 3)   /* Bit 3:  UART Module 3 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCWTIMER_D4        (1 << 4)   /* Bit 4:  UART Module 4 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCWTIMER_D5        (1 << 5)   /* Bit 5:  UART Module 5 Deep-Sleep Mode Clock Gating Control */

/* Watchdog Timer Peripheral Ready */

#define SYSCON_PRWD(n)                (1 << (n)) /* Bit n:  Watchdog Timer n Peripheral Ready */
#  define SYSCON_PRWD_R0              (1 << 0)   /* Bit 0:  Watchdog Timer 0 Peripheral Ready */
#  define SYSCON_PRWD_R1              (1 << 1)   /* Bit 1:  Watchdog Timer 1 Peripheral Ready */

/* 16/32-Bit Timer Peripheral Ready */

#define SYSCON_PRTIMER(n)             (1 << (n)) /* Bit n:  16/32-Bit General-Purpose Timer n Peripheral Ready */
#  define SYSCON_PRTIMER_R0           (1 << 0)   /* Bit 0:  16/32-Bit General-Purpose Timer 0 Peripheral Ready */
#  define SYSCON_PRTIMER_R1           (1 << 1)   /* Bit 1:  16/32-Bit General-Purpose Timer 1 Peripheral Ready */
#  define SYSCON_PRTIMER_R2           (1 << 2)   /* Bit 2:  16/32-Bit General-Purpose Timer 2 Peripheral Ready */
#  define SYSCON_PRTIMER_R3           (1 << 3)   /* Bit 3:  16/32-Bit General-Purpose Timer 3 Peripheral Ready */
#  define SYSCON_PRTIMER_R4           (1 << 4)   /* Bit 4:  16/32-Bit General-Purpose Timer 4 Peripheral Ready */
#  define SYSCON_PRTIMER_R5           (1 << 5)   /* Bit 5:  16/32-Bit General-Purpose Timer 5 Peripheral Ready */

/* GPIO Peripheral Ready */

#define SYSCON_PRGPIO(n)              (1 << (n)) /* Bit n:  GPIO Port F Peripheral Ready */
#  define SYSCON_PRGPIO_R0            (1 << 0)   /* Bit 0:  GPIO Port A Peripheral Ready */
#  define SYSCON_PRGPIO_R1            (1 << 1)   /* Bit 1:  GPIO Port B Peripheral Ready */
#  define SYSCON_PRGPIO_R2            (1 << 2)   /* Bit 2:  GPIO Port C Peripheral Ready */
#  define SYSCON_PRGPIO_R3            (1 << 3)   /* Bit 3:  GPIO Port D Peripheral Ready */
#  define SYSCON_PRGPIO_R4            (1 << 4)   /* Bit 4:  GPIO Port E Peripheral Ready */
#  define SYSCON_PRGPIO_R5            (1 << 5)   /* Bit 5:  GPIO Port F Peripheral Ready */
#  define SYSCON_PRGPIO_R6            (1 << 6)   /* Bit 6:  GPIO Port G Peripheral Ready */
#  define SYSCON_PRGPIO_R7            (1 << 7)   /* Bit 7:  GPIO Port H Peripheral Ready */
#  define SYSCON_PRGPIO_R8            (1 << 8)   /* Bit 8:  GPIO Port J Peripheral Ready */
#  define SYSCON_PRGPIO_R9            (1 << 9)   /* Bit 9:  GPIO Port K Peripheral Ready */
#  define SYSCON_PRGPIO_R10           (1 << 10)  /* Bit 10: GPIO Port L Peripheral Ready */
#  define SYSCON_PRGPIO_R11           (1 << 11)  /* Bit 11: GPIO Port M Peripheral Ready */
#  define SYSCON_PRGPIO_R12           (1 << 12)  /* Bit 12: GPIO Port N Peripheral Ready */
#  define SYSCON_PRGPIO_R13           (1 << 13)  /* Bit 13: GPIO Port P Peripheral Ready */
#  define SYSCON_PRGPIO_R14           (1 << 14)  /* Bit 14: GPIO Port Q Peripheral Ready */

/* uDMA Peripheral Ready */

#define SYSCON_PRDMA_R0               (1 << 0)   /* Bit 0:  μDMA Module Peripheral Ready */

/* Hibernation Peripheral Ready */

#define SYSCON_PRHIB_R0               (1 << 0)   /* Bit 0:  Hibernation Module Peripheral Ready */

/* UART Peripheral Ready */

#define SYSCON_PRUART(n)              (1 << (n)) /* Bit n:  UART Module n Peripheral Ready */
#  define SYSCON_PRUART_R0            (1 << 0)   /* Bit 0:  UART Module 0 Peripheral Ready */
#  define SYSCON_PRUART_R1            (1 << 1)   /* Bit 1:  UART Module 1 Peripheral Ready */
#  define SYSCON_PRUART_R2            (1 << 2)   /* Bit 2:  UART Module 2 Peripheral Ready */
#  define SYSCON_PRUART_R3            (1 << 3)   /* Bit 3:  UART Module 3 Peripheral Ready */
#  define SYSCON_PRUART_R4            (1 << 4)   /* Bit 4:  UART Module 4 Peripheral Ready */
#  define SYSCON_PRUART_R5            (1 << 5)   /* Bit 5:  UART Module 5 Peripheral Ready */
#  define SYSCON_PRUART_R6            (1 << 6)   /* Bit 6:  UART Module 6 Peripheral Ready */
#  define SYSCON_PRUART_R7            (1 << 7)   /* Bit 7:  UART Module 7 Peripheral Ready */

/* SSI Peripheral Ready */

#define SYSCON_PRSSI(n)               (1 << (n)) /* Bit n:  SSI Module n Peripheral Ready */
#  define SYSCON_PRSSI_R0             (1 << 0)   /* Bit 0:  SSI Module 0 Peripheral Ready */
#  define SYSCON_PRSSI_R1             (1 << 1)   /* Bit 1:  SSI Module 1 Peripheral Ready */
#  define SYSCON_PRSSI_R2             (1 << 2)   /* Bit 2:  SSI Module 2 Peripheral Ready */
#  define SYSCON_PRSSI_R3             (1 << 3)   /* Bit 3:  SSI Module 3 Peripheral Ready */

/* I2C Peripheral Ready */

#define SYSCON_PRI2C(n)               (1 << (n)) /* Bit n:  I2C Module n Peripheral Ready */
#  define SYSCON_PRI2C_R0             (1 << 0)   /* Bit 0:  I2C Module 0 Peripheral Ready */
#  define SYSCON_PRI2C_R1             (1 << 1)   /* Bit 1:  I2C Module 1 Peripheral Ready */
#  define SYSCON_PRI2C_R2             (1 << 2)   /* Bit 2:  I2C Module 2 Peripheral Ready */
#  define SYSCON_PRI2C_R3             (1 << 3)   /* Bit 3:  I2C Module 3 Peripheral Ready */
#  define SYSCON_PRI2C_R4             (1 << 4)   /* Bit 4:  I2C Module 4 Peripheral Ready */
#  define SYSCON_PRI2C_R5             (1 << 5)   /* Bit 5:  I2C Module 5 Peripheral Ready */

/* USB Peripheral Ready */

#define SYSCON_PRUSB_R0               (1 << 0)   /* Bit 0:  USB Module Peripheral Ready */

/* CAN Peripheral Ready */

#define SYSCON_PRCAN(n)               (1 << (n)) /* Bit n:  CAN Module n Peripheral Ready */
#  define SYSCON_PRCAN_R0             (1 << 0)   /* Bit 0:  CAN Module 0 Peripheral Ready */
#  define SYSCON_PRCAN_R1             (1 << 1)   /* Bit 1:  CAN Module 1 Peripheral Ready */

/* ADC Peripheral Ready */

#define SYSCON_PRADC(n)               (1 << (n)) /* Bit n:  ADC Module n Peripheral Ready */
#  define SYSCON_PRADC_R0             (1 << 0)   /* Bit 0:  ADC Module 0 Peripheral Ready */
#  define SYSCON_PRADC_R1             (1 << 1)   /* Bit 1:  ADC Module 1 Peripheral Ready */

/* Analog Comparator Peripheral Ready */

#define SYSCON_PRACMP_R0              (1 << 0)   /* Bit 0:  Analog Comparator Module 0 Peripheral Ready */

/* Pulse Width Modulator Peripheral Ready */

#define SYSCON_PRPWM(n)               (1 << (n)) /* Bit n:  PWM Module n Peripheral Ready */
#  define SYSCON_PRPWM_R0             (1 << 0)   /* Bit 0:  PWM Module 0 Peripheral Ready */
#  define SYSCON_PRPWM_R1             (1 << 1)   /* Bit 1:  PWM Module 1 Peripheral Ready */

/* Quadrature Encoder Interface Peripheral Ready */

#define SYSCON_PRQEI(n)               (1 << (n)) /* Bit n:  QEI Module n Peripheral Ready */
#  define SYSCON_PRQEI_R0             (1 << 0)   /* Bit 0:  QEI Module 0 Peripheral Ready */
#  define SYSCON_PRQEI_R1             (1 << 1)   /* Bit 1:  QEI Module 1 Peripheral Ready */

/* EEPROM Peripheral Ready */

#define SYSCON_PREEPROM_0             (1 << 0)   /* Bit 0:  EEPROM Module Peripheral Ready */

/* 2/64-BitWide Timer Peripheral Ready */

#define SYSCON_PRWTIMER(n)            (1 << (n)) /* Bit n:  32/64-Bit Wide General-Purpose Timer n Peripheral Ready */
#  define SYSCON_PRWTIMER_R0          (1 << 0)   /* Bit 0:  32/64-Bit Wide General-Purpose Timer 0 Peripheral Ready */
#  define SYSCON_PRWTIMER_R1          (1 << 1)   /* Bit 1:  32/64-Bit Wide General-Purpose Timer 1 Peripheral Ready */
#  define SYSCON_PRWTIMER_R2          (1 << 2)   /* Bit 2:  32/64-Bit Wide General-Purpose Timer 2 Peripheral Ready */
#  define SYSCON_PRWTIMER_R3          (1 << 3)   /* Bit 3:  32/64-Bit Wide General-Purpose Timer 3 Peripheral Ready */
#  define SYSCON_PRWTIMER_R4          (1 << 4)   /* Bit 4:  32/64-Bit Wide General-Purpose Timer 4 Peripheral Ready */
#  define SYSCON_PRWTIMER_R5          (1 << 5)   /* Bit 5:  32/64-Bit Wide General-Purpose Timer 5 Peripheral Ready */

/* System Control Legacy Register Bit Definitions *******************************************/
/* Device Capabilities 0 */

#define SYSCON_DC0_FLASHSZ_SHIFT      0         /* Bits 15-0: FLASH Size */
#define SYSCON_DC0_FLASHSZ_MASK       (0xffff << SYSCON_DC0_FLASHSZ_SHIFT)
#define SYSCON_DC0_SRAMSZ_SHIFT       16        /* Bits 31-16: SRAM Size */
#define SYSCON_DC0_SRAMSZ_MASK        (0xffff << SYSCON_DC0_SRAMSZ_SHIFT)

/* Device Capabilities 1 */

#define SYSCON_DC1_JTAG               (1 << 0)  /* Bit 0: JTAG Present */
#define SYSCON_DC1_SWD                (1 << 1)  /* Bit 1: SWD Present */
#define SYSCON_DC1_SWO                (1 << 2)  /* Bit 2: SWO Trace Port Present */
#define SYSCON_DC1_WDT0               (1 << 3)  /* Bit 3: Watchdog Timer 0 Present */
#define SYSCON_DC1_PLL                (1 << 4)  /* Bit 4: PLL Present */
#define SYSCON_DC1_TEMPSNS            (1 << 5)  /* Bit 5: Temp Sensor Present */
#define SYSCON_DC1_HIB                (1 << 6)  /* Bit 6: Hibernation Module Present */
#define SYSCON_DC1_MPU                (1 << 7)  /* Bit 7: MPU Present */
#define SYSCON_DC1_MAXADC0SPD_SHIFT   (8)       /* Bits 9-8: Max ADC Speed */
#define SYSCON_DC1_MAXADC0SPD_MASK    (3 << SYSCON_DC1_MAXADC0SPD_SHIFT)
#define SYSCON_DC1_MAXADC1SPD_SHIFT   (10)      /* Bits 10-11: Max ADC Speed */
#define SYSCON_DC1_MAXADC1SPD_MASK    (3 << SYSCON_DC1_MAXADC1SPD_SHIFT)
#define SYSCON_DC1_MINSYSDIV_SHIFT    12        /* Bits 12-15: System Clock Divider Minimum */
#define SYSCON_DC1_MINSYSDIV_MASK     (15 << SYSCON_DC1_MINSYSDIV_SHIFT)
#define SYSCON_DC1_ADC0               (1 << 16) /* Bit 16: ADC0 Module Present */
#define SYSCON_DC1_ADC1               (1 << 17) /* Bit 17: ADC1 Module Present */
#define SYSCON_DC1_PWM0               (1 << 20) /* Bit 20: PWM0 Module Present */
#define SYSCON_DC1_PWM1               (1 << 21) /* Bit 21: PWM1 Module Present */
#define SYSCON_DC1_CAN0               (1 << 24) /* Bit 24: CAN0 Module Present */
#define SYSCON_DC1_CAN1               (1 << 25) /* Bit 25: CAN1 Module Present */
#define SYSCON_DC1_WDT1               (1 << 28) /* Bit 28: Watchdog Timer 1 Present */

/* Device Capabilities 2 */

#define SYSCON_DC2_UART0              (1 << 0)  /* Bit 0: UART0 Module Present */
#define SYSCON_DC2_UART1              (1 << 1)  /* Bit 1: UART1 Module Present */
#define SYSCON_DC2_UART2              (1 << 2)  /* Bit 2: UART2 Module Present */
#define SYSCON_DC2_SSI0               (1 << 4)  /* Bit 4: SSI0 Module Present */
#define SYSCON_DC2_SSI1               (1 << 5)  /* Bit 5: SSI1 Module Present */
#define SYSCON_DC2_QEI0               (1 << 8)  /* Bit 8: QEI0 Module Present */
#define SYSCON_DC2_QEI1               (1 << 9)  /* Bit 9: QEI1 Module Present */
#define SYSCON_DC2_I2C0               (1 << 12) /* Bit 12: I2C Module 0 Present */
#define SYSCON_DC2_I2C0HS             (1 << 13) /* Bit 13: I2C Module 0 Speed */
#define SYSCON_DC2_I2C1               (1 << 14) /* Bit 14: I2C Module 1 Present */
#define SYSCON_DC2_I2C1HS             (1 << 15) /* Bit 15: I2C Module 1 Speed */
#define SYSCON_DC2_TIMER0             (1 << 16) /* Bit 16: Timer 0 Present */
#define SYSCON_DC2_TIMER1             (1 << 17) /* Bit 17: Timer 1 Present */
#define SYSCON_DC2_TIMER2             (1 << 18) /* Bit 18: Timer 2 Present */
#define SYSCON_DC2_TIMER3             (1 << 19) /* Bit 19: Timer 3 Present */
#define SYSCON_DC2_COMP0              (1 << 24) /* Bit 24: Analog Comparator 0 Present */
#define SYSCON_DC2_COMP1              (1 << 25) /* Bit 25: Analog Comparator 1 Present */
#define SYSCON_DC2_COMP2              (1 << 26) /* Bit 26: Analog Comparator 2 Present */
#define SYSCON_DC2_I2S0               (1 << 28) /* Bit 28: I2S Module 0 Present */
#define SYSCON_DC2_EPI0               (1 << 30) /* Bit 30: EPI Module 0 Present */

/* Device Capabilities 3 */

#define SYSCON_DC3_PWM0               (1 << 0)  /* Bit 0: PWM0 Pin Present */
#define SYSCON_DC3_PWM1               (1 << 1)  /* Bit 1: PWM1 Pin Present */
#define SYSCON_DC3_PWM2               (1 << 2)  /* Bit 2: PWM2 Pin Present */
#define SYSCON_DC3_PWM3               (1 << 3)  /* Bit 3: PWM3 Pin Present */
#define SYSCON_DC3_PWM4               (1 << 4)  /* Bit 4: PWM4 Pin Present */
#define SYSCON_DC3_PWM5               (1 << 5)  /* Bit 5: PWM5 Pin Present */
#define SYSCON_DC3_C0MINUS            (1 << 6)  /* Bit 6: C0- Pin Present */
#define SYSCON_DC3_C0PLUS             (1 << 7)  /* Bit 7: C0+ Pin Present */
#define SYSCON_DC3_C0O                (1 << 8)  /* Bit 8: C0o Pin Present */
#define SYSCON_DC3_C1MINUS            (1 << 9)  /* Bit 9: C1- Pin Present */
#define SYSCON_DC3_C1PLUS             (1 << 10) /* Bit 10: C1+ Pin Present */
#define SYSCON_DC3_C1O                (1 << 11) /* Bit 11: C1o Pin Present */
#define SYSCON_DC3_C2MINUS            (1 << 12) /* Bit 12: C2- Pin Present */
#define SYSCON_DC3_C2PLUS             (1 << 13) /* Bit 13: C2+ Pin Present */
#define SYSCON_DC3_C2O                (1 << 14) /* Bit 14: C2o Pin Present */
#define SYSCON_DC3_PWMFAULT           (1 << 15) /* Bit 15: PWM Fault Pin Pre */
#define SYSCON_DC3_ADC0AIN0           (1 << 16) /* Bit 16: ADC Module 0 AIN0 Pin Present */
#define SYSCON_DC3_ADC0AIN1           (1 << 17) /* Bit 17: ADC Module 0 AIN1 Pin Present */
#define SYSCON_DC3_ADC0AIN2           (1 << 18) /* Bit 18: ADC Module 0 AIN2 Pin Present */
#define SYSCON_DC3_ADC0AIN3           (1 << 19) /* Bit 19: ADC Module 0 AIN3 Pin Present */
#define SYSCON_DC3_ADC0AIN4           (1 << 20) /* Bit 20: ADC Module 0 AIN4 Pin Present */
#define SYSCON_DC3_ADC0AIN5           (1 << 21) /* Bit 21: ADC Module 0 AIN5 Pin Present */
#define SYSCON_DC3_ADC0AIN6           (1 << 22) /* Bit 22: ADC Module 0 AIN6 Pin Present */
#define SYSCON_DC3_ADC0AIN7           (1 << 23) /* Bit 23: ADC Module 0 AIN7 Pin Present */
#define SYSCON_DC3_CCP0               (1 << 24) /* Bit 24: T0CCP0 Pin Present */
#define SYSCON_DC3_CCP1               (1 << 25) /* Bit 25: T0CCP1 Pin Present */
#define SYSCON_DC3_CCP2               (1 << 26) /* Bit 26: T1CCP0 Pin Present */
#define SYSCON_DC3_CCP3               (1 << 27) /* Bit 27: T1CCP1 Pin Present */
#define SYSCON_DC3_CCP4               (1 << 28) /* Bit 28: T2CCP0 Pin Present */
#define SYSCON_DC3_CCP5               (1 << 29) /* Bit 29: T2CCP1 Pin Present */
#define SYSCON_DC3_32KHZ              (1 << 31) /* Bit 31: 32KHz Input Clock Available */

/* Device Capabilities 4 */

#define SYSCON_DC4_GPIO(n)            (1 << (n))
#define SYSCON_DC4_GPIOA              (1 << 0)  /* Bit 0:  GPIO Port A Present */
#define SYSCON_DC4_GPIOB              (1 << 1)  /* Bit 1:  GPIO Port B Present */
#define SYSCON_DC4_GPIOC              (1 << 2)  /* Bit 2:  GPIO Port C Present */
#define SYSCON_DC4_GPIOD              (1 << 3)  /* Bit 3:  GPIO Port D Present */
#define SYSCON_DC4_GPIOE              (1 << 4)  /* Bit 4:  GPIO Port E Present */
#define SYSCON_DC4_GPIOF              (1 << 5)  /* Bit 5:  GPIO Port F Present */
#define SYSCON_DC4_GPIOG              (1 << 6)  /* Bit 6:  GPIO Port G Present */
#define SYSCON_DC4_GPIOH              (1 << 7)  /* Bit 7:  GPIO Port H Present */
#define SYSCON_DC4_GPIOJ              (1 << 8)  /* Bit 8:  GPIO Port J Present */

#define SYSCON_DC4_ROM                (1 << 12) /* Bit 12: Internal Code ROM Present */
#define SYSCON_DC4_UDMA               (1 << 13) /* Bit 13: Micro-DMA Module Present */
#define SYSCON_DC4_CCP6               (1 << 14) /* Bit 14: T3CCP0 Pin Present */
#define SYSCON_DC4_CCP7               (1 << 15) /* Bit 15: T3CCP1 Pin Present */
#define SYSCON_DC4_PICAL              (1 << 18) /* Bit 18: PIOSC Calibrate */
#define SYSCON_DC4_E1588              (1 << 24) /* Bit 24: 1588 Capable */
#define SYSCON_DC4_EMAC0              (1 << 28) /* Bit 28: Ethernet MAC0 Present */
#define SYSCON_DC4_EPHY0              (1 << 30) /* Bit 30: Ethernet PHY0 Present */

/* Device Capabilities 5 */

#define TIVA_SYSCON_DC5_PWM0            (1 << 0)  /* Bit 0:  PWM0 Pin Present */
#define TIVA_SYSCON_DC5_PWM1            (1 << 1)  /* Bit 1:  PWM1 Pin Present */
#define TIVA_SYSCON_DC5_PWM2            (1 << 2)  /* Bit 2:  PWM2 Pin Present */
#define TIVA_SYSCON_DC5_PWM3            (1 << 3)  /* Bit 3:  PWM3 Pin Present */
#define TIVA_SYSCON_DC5_PWM4            (1 << 4)  /* Bit 4:  PWM4 Pin Present */
#define TIVA_SYSCON_DC5_PWM5            (1 << 5)  /* Bit 5:  PWM5 Pin Present */
#define TIVA_SYSCON_DC5_PWM6            (1 << 6)  /* Bit 6:  PWM6 Pin Present */
#define TIVA_SYSCON_DC5_PWM7            (1 << 7)  /* Bit 7:  PWM7 Pin Present */
#define TIVA_SYSCON_DC5_PWMESYNC        (1 << 20) /* Bit 20: PWM Extended SYNC Active */
#define TIVA_SYSCON_DC5_PWMEFLT         (1 << 21) /* Bit 21: PWM Extended Fault Active */
#define TIVA_SYSCON_DC5_PWMFAULT0       (1 << 24) /* Bit 24: PWM Fault 0 Pin Present */
#define TIVA_SYSCON_DC5_PWMFAULT1       (1 << 25) /* Bit 25: PWM Fault 1 Pin Present */
#define TIVA_SYSCON_DC5_PWMFAULT2       (1 << 26) /* Bit 26: PWM Fault 2 Pin Present */
#define TIVA_SYSCON_DC5_PWMFAULT3       (1 << 27) /* Bit 27: PWM Fault 3 Pin Present */

/* Device Capabilities 6 */

#define TIVA_SYSCON_DC6_USB0_SHIFT      (0)       /* Bits 0-1: USB Module 0 Present */
#define TIVA_SYSCON_DC6_USB0_MASK       (3 << TIVA_SYSCON_DC6_USB0_SHIFT)
#  define TIVA_SYSCON_DC6_USB0_NONE     (1 << TIVA_SYSCON_DC6_USB0_SHIFT)
#  define TIVA_SYSCON_DC6_USB0_DEVICE   (2 << TIVA_SYSCON_DC6_USB0_SHIFT)
#  define TIVA_SYSCON_DC6_USB0_HOST     (3 << TIVA_SYSCON_DC6_USB0_SHIFT)
#  define TIVA_SYSCON_DC6_USB0_OTG      (3 << TIVA_SYSCON_DC6_USB0_SHIFT)
#define TIVA_SYSCON_DC6_USB0PHY         (1 << 4)  /* Bit 4: USB Module 0 PHY Present */

/* Device Capabilities 7 */

#define TIVA_SYSCON_DC7_DMACH0          (1 << 0)  /* Bit 0:  DMA Channel 0 */
#define TIVA_SYSCON_DC7_DMACH1          (1 << 1)  /* Bit 1:  DMA Channel 1 */
#define TIVA_SYSCON_DC7_DMACH2          (1 << 2)  /* Bit 2:  DMA Channel 2 */
#define TIVA_SYSCON_DC7_DMACH3          (1 << 3)  /* Bit 3:  DMA Channel 3 */
#define TIVA_SYSCON_DC7_DMACH4          (1 << 4)  /* Bit 4:  DMA Channel 4 */
#define TIVA_SYSCON_DC7_DMACH5          (1 << 5)  /* Bit 5:  DMA Channel 5 */
#define TIVA_SYSCON_DC7_DMACH6          (1 << 6)  /* Bit 6:  DMA Channel 6 */
#define TIVA_SYSCON_DC7_DMACH7          (1 << 7)  /* Bit 7:  DMA Channel 7 */
#define TIVA_SYSCON_DC7_DMACH8          (1 << 8)  /* Bit 8:  DMA Channel 8 */
#define TIVA_SYSCON_DC7_DMACH9          (1 << 9)  /* Bit 9:  DMA Channel 9 */
#define TIVA_SYSCON_DC7_DMACH10         (1 << 10) /* Bit 10: DMA Channel 10 */
#define TIVA_SYSCON_DC7_DMACH11         (1 << 11) /* Bit 11: DMA Channel 11 */
#define TIVA_SYSCON_DC7_DMACH12         (1 << 12) /* Bit 12: DMA Channel 12 */
#define TIVA_SYSCON_DC7_DMACH13         (1 << 13) /* Bit 13: DMA Channel 13 */
#define TIVA_SYSCON_DC7_DMACH14         (1 << 14) /* Bit 14: DMA Channel 14 */
#define TIVA_SYSCON_DC7_DMACH15         (1 << 15) /* Bit 15: DMA Channel 15 */
#define TIVA_SYSCON_DC7_DMACH16         (1 << 16) /* Bit 16: DMA Channel 16 */
#define TIVA_SYSCON_DC7_DMACH17         (1 << 17) /* Bit 17: DMA Channel 17 */
#define TIVA_SYSCON_DC7_DMACH18         (1 << 18) /* Bit 18: DMA Channel 18 */
#define TIVA_SYSCON_DC7_DMACH19         (1 << 19) /* Bit 19: DMA Channel 19 */
#define TIVA_SYSCON_DC7_DMACH20         (1 << 20) /* Bit 20: DMA Channel 20 */
#define TIVA_SYSCON_DC7_DMACH21         (1 << 21) /* Bit 21: DMA Channel 21 */
#define TIVA_SYSCON_DC7_DMACH22         (1 << 22) /* Bit 22: DMA Channel 22 */
#define TIVA_SYSCON_DC7_DMACH23         (1 << 23) /* Bit 23: DMA Channel 23 */
#define TIVA_SYSCON_DC7_DMACH24         (1 << 24) /* Bit 24: DMA Channel 24 */
#define TIVA_SYSCON_DC7_DMACH25         (1 << 25) /* Bit 25: DMA Channel 25 */
#define TIVA_SYSCON_DC7_DMACH26         (1 << 26) /* Bit 26: DMA Channel 26 */
#define TIVA_SYSCON_DC7_DMACH27         (1 << 27) /* Bit 27: DMA Channel 27 */
#define TIVA_SYSCON_DC7_DMACH28         (1 << 28) /* Bit 28: DMA Channel 28 */
#define TIVA_SYSCON_DC7_DMACH29         (1 << 29) /* Bit 29: DMA Channel 29 */
#define TIVA_SYSCON_DC7_DMACH30         (1 << 30) /* Bit 30: DMA Channel 30 */

/* Device Capabilities 8 */

#define TIVA_SYSCON_DC8_ADC0AIN0        (1 << 0)  /* Bit 0:  ADC Module 0 AIN0 Pin Present */
#define TIVA_SYSCON_DC8_ADC0AIN1        (1 << 1)  /* Bit 1:  ADC Module 0 AIN1 Pin Present */
#define TIVA_SYSCON_DC8_ADC0AIN2        (1 << 2)  /* Bit 2:  ADC Module 0 AIN2 Pin Present */
#define TIVA_SYSCON_DC8_ADC0AIN3        (1 << 3)  /* Bit 3:  ADC Module 0 AIN3 Pin Present */
#define TIVA_SYSCON_DC8_ADC0AIN4        (1 << 4)  /* Bit 4:  ADC Module 0 AIN4 Pin Present */
#define TIVA_SYSCON_DC8_ADC0AIN5        (1 << 5)  /* Bit 5:  ADC Module 0 AIN5 Pin Present */
#define TIVA_SYSCON_DC8_ADC0AIN6        (1 << 6)  /* Bit 6:  ADC Module 0 AIN6 Pin Present */
#define TIVA_SYSCON_DC8_ADC0AIN7        (1 << 7)  /* Bit 7:  ADC Module 0 AIN7 Pin Present */
#define TIVA_SYSCON_DC8_ADC0AIN8        (1 << 8)  /* Bit 8:  ADC Module 0 AIN8 Pin Present */
#define TIVA_SYSCON_DC8_ADC0AIN9        (1 << 9)  /* Bit 9:  ADC Module 0 AIN9 Pin Present */
#define TIVA_SYSCON_DC8_ADC0AIN10       (1 << 10) /* Bit 10: ADC Module 0 AIN10 Pin Present */
#define TIVA_SYSCON_DC8_ADC0AIN11       (1 << 11) /* Bit 11: ADC Module 0 AIN11 Pin Present */
#define TIVA_SYSCON_DC8_ADC0AIN12       (1 << 12) /* Bit 12: ADC Module 0 AIN12 Pin Present */
#define TIVA_SYSCON_DC8_ADC0AIN13       (1 << 13) /* Bit 13: ADC Module 0 AIN13 Pin Present */
#define TIVA_SYSCON_DC8_ADC0AIN14       (1 << 14) /* Bit 14: ADC Module 0 AIN14 Pin Present */
#define TIVA_SYSCON_DC8_ADC0AIN15       (1 << 15) /* Bit 15: ADC Module 0 AIN15 Pin Present */
#define TIVA_SYSCON_DC8_ADC1AIN0        (1 << 16) /* Bit 16: ADC Module 1 AIN0 Pin Present */
#define TIVA_SYSCON_DC8_ADC1AIN1        (1 << 17) /* Bit 17: ADC Module 1 AIN1 Pin Present */
#define TIVA_SYSCON_DC8_ADC1AIN2        (1 << 18) /* Bit 18: ADC Module 1 AIN2 Pin Present */
#define TIVA_SYSCON_DC8_ADC1AIN3        (1 << 19) /* Bit 19: ADC Module 1 AIN3 Pin Present */
#define TIVA_SYSCON_DC8_ADC1AIN4        (1 << 20) /* Bit 20: ADC Module 1 AIN4 Pin Present */
#define TIVA_SYSCON_DC8_ADC1AIN5        (1 << 21) /* Bit 21: ADC Module 1 AIN5 Pin Present */
#define TIVA_SYSCON_DC8_ADC1AIN6        (1 << 22) /* Bit 22: ADC Module 1 AIN6 Pin Present */
#define TIVA_SYSCON_DC8_ADC1AIN7        (1 << 23) /* Bit 23: ADC Module 1 AIN7 Pin Present */
#define TIVA_SYSCON_DC8_ADC1AIN8        (1 << 24) /* Bit 24: ADC Module 1 AIN8 Pin Present */
#define TIVA_SYSCON_DC8_ADC1AIN9        (1 << 25) /* Bit 25: ADC Module 1 AIN9 Pin Present */
#define TIVA_SYSCON_DC8_ADC1AIN10       (1 << 26) /* Bit 26: ADC Module 1 AIN10 Pin Present */
#define TIVA_SYSCON_DC8_ADC1AIN11       (1 << 27) /* Bit 27: ADC Module 1 AIN11 Pin Present */
#define TIVA_SYSCON_DC8_ADC1AIN12       (1 << 28) /* Bit 28: ADC Module 1 AIN12 Pin Present */
#define TIVA_SYSCON_DC8_ADC1AIN13       (1 << 29) /* Bit 29: ADC Module 1 AIN13 Pin Present */
#define TIVA_SYSCON_DC8_ADC1AIN14       (1 << 30) /* Bit 30: ADC Module 1 AIN14 Pin Present */
#define TIVA_SYSCON_DC8_ADC1AIN15       (1 << 31) /* Bit 31: ADC Module 1 AIN15 Pin Present */

/* Software Reset Control 0 */

#define SYSCON_SRCR0_WDT0             (1 << 3)  /* Bit 3:  Watchdog Timer 0 Reset Control */
#define SYSCON_SRCR0_HIB              (1 << 6)  /* Bit 6:  Hibernation Module Reset Control */
#define SYSCON_SRCR0_ADC0             (1 << 16) /* Bit 16: ADC0 Reset Control */
#define SYSCON_SRCR0_ADC1             (1 << 17) /* Bit 17: ADC1 Reset Control */
#define SYSCON_SRCR0_CAN0             (1 << 24) /* Bit 24: CAN0 Reset Control */
#define SYSCON_SRCR0_CAN1             (1 << 25) /* Bit 24: CAN1 Reset Control */
#define SYSCON_SRCR0_WDT1             (1 << 28) /* Bit 28: Watchdog Timer 1 Reset Control */

/* Software Reset Control 1 */

#define SYSCON_SRCR1_UART0            (1 << 0)  /* Bit 0:  UART0 Reset Control */
#define SYSCON_SRCR1_UART1            (1 << 1)  /* Bit 1:  UART1 Reset Control */
#define SYSCON_SRCR1_UART2            (1 << 2)  /* Bit 2:  UART2 Reset Control */
#define SYSCON_SRCR1_SSI0             (1 << 4)  /* Bit 4:  SSI0 Reset Control */
#define SYSCON_SRCR1_SSI1             (1 << 5)  /* Bit 5:  SSI1 Reset Control */
#define SYSCON_SRCR1_QEI0             (1 << 8)  /* Bit 8:  QEI0 Reset Control */
#define SYSCON_SRCR1_QEI1             (1 << 9)  /* Bit 9:  QEI1 Reset Control */
#define SYSCON_SRCR1_I2C0             (1 << 12) /* Bit 12: I2C 0 Reset Control */
#define SYSCON_SRCR1_I2C1             (1 << 14) /* Bit 14: I2C 1 Reset Control */
#define SYSCON_SRCR1_TIMER0           (1 << 16) /* Bit 16: Timer 0 Reset Control */
#define SYSCON_SRCR1_TIMER1           (1 << 17) /* Bit 17: Timer 1 Reset Control */
#define SYSCON_SRCR1_TIMER2           (1 << 18) /* Bit 18: Timer 2 Reset Control */
#define SYSCON_SRCR1_TIMER3           (1 << 19) /* Bit 19: Timer 3 Reset Control */
#define SYSCON_SRCR1_COMP0            (1 << 24) /* Bit 24: Analog Comparator 0 Reset Control */
#define SYSCON_SRCR1_COMP1            (1 << 25) /* Bit 25: Analog Comparator 1 Reset Control */
#define SYSCON_SRCR1_COMP2            (1 << 26) /* Bit 26: Analog Comparator 2 Reset Control */

/* Software Reset Control 2 */

#define SYSCON_SRCR2_GPIO(n)          (1 << (n))
#define SYSCON_SRCR2_GPIOA            (1 << 0)  /* Bit 0:  Port A Reset Control */
#define SYSCON_SRCR2_GPIOB            (1 << 1)  /* Bit 1:  Port B Reset Control */
#define SYSCON_SRCR2_GPIOC            (1 << 2)  /* Bit 2:  Port C Reset Control */
#define SYSCON_SRCR2_GPIOD            (1 << 3)  /* Bit 3:  Port D Reset Control */
#define SYSCON_SRCR2_GPIOE            (1 << 4)  /* Bit 4:  Port E Reset Control */
#define SYSCON_SRCR2_GPIOF            (1 << 5)  /* Bit 5:  Port F Reset Control */
#define SYSCON_SRCR2_GPIOG            (1 << 6)  /* Bit 6:  Port G Reset Control */
#define SYSCON_SRCR2_GPIOH            (1 << 7)  /* Bit 7:  Port H Reset Control */
#define SYSCON_SRCR2_GPIOJ            (1 << 8)  /* Bit 8:  Port J Reset Control */
#define SYSCON_SRCR2_UDMA             (1 << 13) /* Bit 13: Micro-DMA Reset Control */
#define SYSCON_SRCR2_USB0             (1 << 16) /* Bit 16: USB0 Reset Control */

/* Run Mode Clock Gating Control Register 0 */

#define SYSCON_RCGC0_WDT0             (1 << 3)  /* Bit 3: WDT0 Clock Gating Control */
#define SYSCON_RCGC0_HIB              (1 << 6)  /* Bit 6: HIB Clock Gating Control */
#define SYSCON_RCGC0_MAXADC0SPD_SHIFT (8)       /* Bits 8-9: ADC0 Sample Speed */
#define SYSCON_RCGC0_MAXADC0SPD_MASK  (3 << SYSCON_RCGC0_MAXADC0SPD_SHIFT)
#  define SYSCON_RCGC0_MAXADC0_125KSPS (0 << SYSCON_RCGC0_MAXADC0SPD_SHIFT)
#  define SYSCON_RCGC0_MAXADC0_250KSPS (1 << SYSCON_RCGC0_MAXADC0SPD_SHIFT)
#  define SYSCON_RCGC0_MAXADC0_500KSPS (2 << SYSCON_RCGC0_MAXADC0SPD_SHIFT)
#  define SYSCON_RCGC0_MAXADC0_1MSPS   (3 << SYSCON_RCGC0_MAXADC0SPD_SHIFT)
#define SYSCON_RCGC0_MAXADC1SPD_SHIFT (8)       /* Bits 10-11: ADC1 Sample Speed */
#define SYSCON_RCGC0_MAXADC1SPD_MASK  (3 << SYSCON_RCGC0_MAXADC1SPD_SHIFT)
#  define SYSCON_RCGC0_MAXADC1_125KSPS (0 << SYSCON_RCGC0_MAXADC1SPD_SHIFT)
#  define SYSCON_RCGC0_MAXADC1_250KSPS (1 << SYSCON_RCGC0_MAXADC1SPD_SHIFT)
#  define SYSCON_RCGC0_MAXADC1_500KSPS (2 << SYSCON_RCGC0_MAXADC1SPD_SHIFT)
#  define SYSCON_RCGC0_MAXADC1_1MSPS   (3 << SYSCON_RCGC0_MAXADC1SPD_SHIFT)
#define SYSCON_RCGC0_ADC0             (1 << 16) /* Bit 16: ADC0 Clock Gating Control */
#define SYSCON_RCGC0_ADC1             (1 << 17) /* Bit 17: ADC1 Clock Gating Control */
#define SYSCON_RCGC0_PWM0             (1 << 20) /* Bit 20: PWM0 Clock Gating Control */
#define SYSCON_RCGC0_CAN0             (1 << 24) /* Bit 24: CAN0 Clock Gating Control */
#define SYSCON_RCGC0_CAN1             (1 << 25) /* Bit 25: CAN1 Clock Gating Control */
#define SYSCON_RCGC0_WDT1             (1 << 28) /* Bit 28: WDT1 Clock Gating Control */

/* Run Mode Clock Gating Control Register 1 */

#define SYSCON_RCGC1_UART0            (1 << 0)  /* Bit 0:  UART0 Clock Gating Control */
#define SYSCON_RCGC1_UART1            (1 << 1)  /* Bit 1:  UART1 Clock Gating Control */
#define SYSCON_RCGC1_UART2            (1 << 2)  /* Bit 2:  UART2 Clock Gating Control */
#define SYSCON_RCGC1_SSI0             (1 << 4)  /* Bit 4:  SSI0 Clock Gating Control */
#define SYSCON_RCGC1_SSI1             (1 << 5)  /* Bit 5:  SSI1 Clock Gating Control */
#define SYSCON_RCGC1_QEI0             (1 << 8)  /* Bit 8:  QEI0 Clock Gating Control */
#define SYSCON_RCGC1_QEI1             (1 << 9)  /* Bit 9:  QEI1 Clock Gating Control */
#define SYSCON_RCGC1_I2C0             (1 << 12) /* Bit 12: I2C0 Clock Gating Control */
#define SYSCON_RCGC1_I2C1             (1 << 14) /* Bit 14: I2C1 Clock Gating Control */
#define SYSCON_RCGC1_TIMER0           (1 << 16) /* Bit 16: Timer 0 Clock Gating Control */
#define SYSCON_RCGC1_TIMER1           (1 << 17) /* Bit 17: Timer 1 Clock Gating Control */
#define SYSCON_RCGC1_TIMER2           (1 << 18) /* Bit 18: Timer 2 Clock Gating Control */
#define SYSCON_RCGC1_TIMER3           (1 << 19) /* Bit 19: Timer 3 Clock Gating Control */
#define SYSCON_RCGC1_COMP0            (1 << 24) /* Bit 24: Analog Comparator 0 Clock Gating */
#define SYSCON_RCGC1_COMP1            (1 << 25) /* Bit 25: Analog Comparator 1 Clock Gating */
#define SYSCON_RCGC1_COMP2            (1 << 26) /* Bit 26: Analog Comparator 2 Clock Gating */

/* Run Mode Clock Gating Control Register 2 */

#define SYSCON_RCGC2_GPIO(n)          (1 << (n))
#define SYSCON_RCGC2_GPIOA            (1 << 0)  /* Bit 0:  Port A Clock Gating Control */
#define SYSCON_RCGC2_GPIOB            (1 << 1)  /* Bit 1:  Port B Clock Gating Control */
#define SYSCON_RCGC2_GPIOC            (1 << 2)  /* Bit 2:  Port C Clock Gating Control */
#define SYSCON_RCGC2_GPIOD            (1 << 3)  /* Bit 3:  Port D Clock Gating Control */
#define SYSCON_RCGC2_GPIOE            (1 << 4)  /* Bit 4:  Port E Clock Gating Control */
#define SYSCON_RCGC2_GPIOF            (1 << 5)  /* Bit 5:  Port F Clock Gating Control */
#define SYSCON_RCGC2_GPIOG            (1 << 6)  /* Bit 6:  Port GClock Gating Control */
#define SYSCON_RCGC2_GPIOH            (1 << 7)  /* Bit 7:  Port H Clock Gating Control */
#define SYSCON_RCGC2_GPIOJ            (1 << 8)  /* Bit 8:  Port J Clock Gating Control */
#define SYSCON_RCGC2_UDMA             (1 << 13) /* Bit 13: Micro-DMA Clock Gating Control */
#define SYSCON_RCGC2_USB0             (1 << 16) /* Bit 16: USB0 Clock Gating Control */

/* Sleep Mode Clock Gating Control Register 0 */

#define SYSCON_SCGC0_WDT0             (1 << 3)  /* Bit 3:  WDT0 Clock Gating Control */
#define SYSCON_SCGC0_HIB              (1 << 6)  /* Bit 6:  HIB Clock Gating Control */
#define SYSCON_SCGC0_ADC0             (1 << 16) /* Bit 16: ADC0 Clock Gating Control */
#define SYSCON_SCGC0_ADC1             (1 << 17) /* Bit 17: ADC1 Clock Gating Control */
#define SYSCON_SCGC0_PWM0             (1 << 20) /* Bit 20: PWM0 Clock Gating Control */
#define SYSCON_SCGC0_CAN0             (1 << 24) /* Bit 24: CAN0 Clock Gating Control */
#define SYSCON_SCGC0_CAN1             (1 << 25) /* Bit 25: CAN1 Clock Gating Control */
#define SYSCON_SCGC0_WDT1             (1 << 28) /* Bit 28: WDT1 Clock Gating Control */

/* Sleep Mode Clock Gating Control Register 1 */

#define SYSCON_SCGC1_UART0            (1 << 0)  /* Bit 0:  UART0 Clock Gating Control */
#define SYSCON_SCGC1_UART1            (1 << 1)  /* Bit 1:  UART1 Clock Gating Control */
#define SYSCON_SCGC1_UART2            (1 << 2)  /* Bit 2:  UART2 Clock Gating Control */
#define SYSCON_SCGC1_SSI0             (1 << 4)  /* Bit 4:  SSI0 Clock Gating Control */
#define SYSCON_SCGC1_SSI1             (1 << 5)  /* Bit 5:  SSI1 Clock Gating Control */
#define SYSCON_SCGC1_QEI0             (1 << 8)  /* Bit 8:  QEI0 Clock Gating Control */
#define SYSCON_SCGC1_QEI1             (1 << 9)  /* Bit 9:  QEI1 Clock Gating Control */
#define SYSCON_SCGC1_I2C0             (1 << 12) /* Bit 12: I2C0 Clock Gating Control */
#define SYSCON_SCGC1_I2C1             (1 << 14) /* Bit 14: I2C1 Clock Gating Control */
#define SYSCON_SCGC1_TIMER0           (1 << 16) /* Bit 16: Timer 0 Clock Gating Control */
#define SYSCON_SCGC1_TIMER1           (1 << 17) /* Bit 17: Timer 1 Clock Gating Control */
#define SYSCON_SCGC1_TIMER2           (1 << 18) /* Bit 18: Timer 2 Clock Gating Control */
#define SYSCON_SCGC1_TIMER3           (1 << 19) /* Bit 19: Timer 3 Clock Gating Control */
#define SYSCON_SCGC1_COMP0            (1 << 24) /* Bit 24: Analog Comparator 0 Clock Gating */
#define SYSCON_SCGC1_COMP1            (1 << 25) /* Bit 25: Analog Comparator 1 Clock Gating */
#define SYSCON_SCGC1_COMP2            (1 << 26) /* Bit 26: Analog Comparator 2 Clock Gating */

/* Sleep Mode Clock Gating Control Register 2 */

#define SYSCON_SCGC2_GPIO(n)          (1 << (n))
#define SYSCON_SCGC2_GPIOA            (1 << 0)  /* Bit 0:  Port A Clock Gating Control */
#define SYSCON_SCGC2_GPIOB            (1 << 1)  /* Bit 1:  Port B Clock Gating Control */
#define SYSCON_SCGC2_GPIOC            (1 << 2)  /* Bit 2:  Port C Clock Gating Control */
#define SYSCON_SCGC2_GPIOD            (1 << 3)  /* Bit 3:  Port D Clock Gating Control */
#define SYSCON_SCGC2_GPIOE            (1 << 4)  /* Bit 4:  Port E Clock Gating Control */
#define SYSCON_SCGC2_GPIOF            (1 << 5)  /* Bit 5:  Port F Clock Gating Control */
#define SYSCON_SCGC2_GPIOG            (1 << 6)  /* Bit 6:  Port G Clock Gating Control */
#define SYSCON_SCGC2_GPIOH            (1 << 7)  /* Bit 7:  Port H Clock Gating Control */
#define SYSCON_SCGC2_GPIOI            (1 << 8)  /* Bit 8:  Port I Clock Gating Control */
#define SYSCON_SCGC2_UDMA             (1 << 13) /* Bit 13: Micro-DMA Clock Gating Control */
#define SYSCON_SCGC2_USB0             (1 << 16) /* Bit 16: PHY0 Clock Gating Control */

/* Deep Sleep Mode Clock Gating Control Register 0 */

#define SYSCON_DCGC0_WDT0             (1 << 3)  /* Bit 3:  WDT0 Clock Gating Control */
#define SYSCON_DCGC0_HIB              (1 << 6)  /* Bit 6:  HIB Clock Gating Control */
#define SYSCON_DCGC0_ADC0             (1 << 16) /* Bit 16: ADC0 Clock Gating Control */
#define SYSCON_DCGC0_ADC1             (1 << 17) /* Bit 17: ADC1 Clock Gating Control */
#define SYSCON_DCGC0_PWM0             (1 << 20) /* Bit 20: PWM0 Clock Gating Control */
#define SYSCON_DCGC0_CAN0             (1 << 24) /* Bit 24: CAN0 Clock Gating Control */
#define SYSCON_DCGC0_CAN1             (1 << 25) /* Bit 25: CAN1 Clock Gating Control */
#define SYSCON_DCGC0_WDT1             (1 << 28) /* Bit 28: WDT1 Clock Gating Control */

/* Deep Sleep Mode Clock Gating Control Register 1 */

#define SYSCON_DCGC1_UART0            (1 << 0)  /* Bit 0:  UART0 Clock Gating Control */
#define SYSCON_DCGC1_UART1            (1 << 1)  /* Bit 1:  UART1 Clock Gating Control */
#define SYSCON_DCGC1_UART2            (1 << 2)  /* Bit 2:  UART2 Clock Gating Control */
#define SYSCON_DCGC1_SSI0             (1 << 4)  /* Bit 4:  SSI0 Clock Gating Control */
#define SYSCON_DCGC1_SSI1             (1 << 5)  /* Bit 5:  SSI1 Clock Gating Control */
#define SYSCON_DCGC1_QEI0             (1 << 8)  /* Bit 8:  QEI0 Clock Gating Control */
#define SYSCON_DCGC1_QEI1             (1 << 9)  /* Bit 9:  QEI1 Clock Gating Control */
#define SYSCON_DCGC1_I2C0             (1 << 12) /* Bit 12: I2C0 Clock Gating Control */
#define SYSCON_DCGC1_I2C1             (1 << 14) /* Bit 14: I2C1 Clock Gating Control */
#define SYSCON_DCGC1_TIMER0           (1 << 16) /* Bit 16: Timer 0 Clock Gating Control */
#define SYSCON_DCGC1_TIMER1           (1 << 17) /* Bit 17: Timer 1 Clock Gating Control */
#define SYSCON_DCGC1_TIMER2           (1 << 18) /* Bit 18: Timer 2 Clock Gating Control */
#define SYSCON_DCGC1_TIMER3           (1 << 19) /* Bit 19: Timer 3 Clock Gating Control */
#define SYSCON_DCGC1_COMP0            (1 << 24) /* Bit 24: Analog Comparator 0 Clock Gating */
#define SYSCON_DCGC1_COMP1            (1 << 25) /* Bit 25: Analog Comparator 1 Clock Gating */
#define SYSCON_DCGC1_COMP2            (1 << 26) /* Bit 26: Analog Comparator 6 Clock Gating */

/* Deep Sleep Mode Clock Gating Control Register 2 */

#define SYSCON_DCGC2_GPIO(n)          (1 << (n))
#define SYSCON_DCGC2_GPIOA            (1 << 0)  /* Bit 0:  Port A Clock Gating Control */
#define SYSCON_DCGC2_GPIOB            (1 << 1)  /* Bit 1:  Port B Clock Gating Control */
#define SYSCON_DCGC2_GPIOC            (1 << 2)  /* Bit 2:  Port C Clock Gating Control */
#define SYSCON_DCGC2_GPIOD            (1 << 3)  /* Bit 3:  Port D Clock Gating Control */
#define SYSCON_DCGC2_GPIOE            (1 << 4)  /* Bit 4:  Port E Clock Gating Control */
#define SYSCON_DCGC2_GPIOF            (1 << 5)  /* Bit 5:  Port F Clock Gating Control */
#define SYSCON_DCGC2_GPIOG            (1 << 6)  /* Bit 6:  Port G Clock Gating Control */
#define SYSCON_DCGC2_GPIOH            (1 << 7)  /* Bit 7:  Port H Clock Gating Control */
#define SYSCON_DCGC2_GPIOI            (1 << 8)  /* Bit 8:  Port I Clock Gating Control */
#define SYSCON_DCGC2_UDMA             (1 << 13) /* Bit 13: Micro-DMA Clock Gating Control */
#define SYSCON_DCGC2_USB0             (1 << 16) /* Bit 16: PHY0 Clock Gating Control */

/* Device Capabilities */

#define TIVA_SYSCON_DC9_ADC0DC0         (1 << 0)  /* Bit 0:  ADC0 DC0 Present */
#define TIVA_SYSCON_DC9_ADC0DC1         (1 << 1)  /* Bit 1:  ADC0 DC1 Present */
#define TIVA_SYSCON_DC9_ADC0DC2         (1 << 2)  /* Bit 2:  ADC0 DC2 Present */
#define TIVA_SYSCON_DC9_ADC0DC3         (1 << 3)  /* Bit 3:  ADC0 DC3 Present */
#define TIVA_SYSCON_DC9_ADC0DC4         (1 << 4)  /* Bit 4:  ADC0 DC4 Present */
#define TIVA_SYSCON_DC9_ADC0DC5         (1 << 5)  /* Bit 5:  ADC0 DC5 Present */
#define TIVA_SYSCON_DC9_ADC0DC6         (1 << 6)  /* Bit 6:  ADC0 DC6 Present */
#define TIVA_SYSCON_DC9_ADC0DC7         (1 << 7)  /* Bit 7:  ADC0 DC7 Present */
#define TIVA_SYSCON_DC9_ADC1DC0         (1 << 16) /* Bit 16: ADC1 DC0 Present */
#define TIVA_SYSCON_DC9_ADC1DC1         (1 << 17) /* Bit 17: ADC1 DC1 Present */
#define TIVA_SYSCON_DC9_ADC1DC2         (1 << 18) /* Bit 18: ADC1 DC2 Present */
#define TIVA_SYSCON_DC9_ADC1DC3         (1 << 19) /* Bit 19: ADC1 DC3 Present */
#define TIVA_SYSCON_DC9_ADC1DC4         (1 << 20) /* Bit 20: ADC1 DC4 Present */
#define TIVA_SYSCON_DC9_ADC1DC5         (1 << 21) /* Bit 21: ADC1 DC5 Present */
#define TIVA_SYSCON_DC9_ADC1DC6         (1 << 22) /* Bit 22: ADC1 DC6 Present */
#define TIVA_SYSCON_DC9_ADC1DC7         (1 << 23) /* Bit 23: ADC1 DC7 Present */

/* Non-Volatile Memory Information */

#define TIVA_SYSCON_NVMSTAT_FWB         (1 << 0)  /* Bit 0: 32 Word Flash Write Buffer Available */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* __ARCH_ARM_SRC_TIVA_CHIP_CC3200_SYSCONTROL_H */
