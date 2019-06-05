/********************************************************************************************
 * arch/arm/src/samd5e5/hardware/samd5e5_memorymap.h
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMD5E5_HARDWARE_SAMD5E5_MEMORYMAP_H
#define __ARCH_ARM_SRC_SAMD5E5_HARDWARE_SAMD5E5_MEMORYMAP_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/* System Memory Map */

#define SAM_CODE_BASE       0x00000000 /* Code address space */
#  define SAM_FLASH_BASE    0x00000000 /* FLASH address space */
#  define SAM_CMCC_BASE     0x03000000 /* CMCC address space */
#  define SAM_QSPI_BASE     0x04000000 /* QSPI address space */
#define SAM_SRAM_BASE       0x20000000 /* SRAM address space */
#define SAM_PERIPH_BASE     0x40000000 /* Peripherals memory space */
#  define SAM_AHBA_BASE     0x40000000 /* AHB-APB Bridge A */
#  define SAM_AHBB_BASE     0x41000000 /* AHB-APB Bridge B */
#  define SAM_AHBC_BASE     0x42000000 /* AHB-APB Bridge C */
#  define SAM_AHBD_BASE     0x43000000 /* AHB-APB Bridge D */
#  define SAM_SEEPROM_BASE  0x44000000 /* SEEPROM */
#  define SAM_SDHC0_BASE    0x45000000 /* Memory card interface (SDHC0) */
#  define SAM_SDHC1_BASE    0x46000000 /* Memory card interface (SDHC1) */
#  define SAM_BKPRAM_BASE   0x47000000 /* Backup RAM */
#define SAM_SYSTEM_BASE     0xe0000000 /* System address space */
#  define SAM_SCS_BASE      0xe000e000 /* SCSS */
#  define SAM_ROMTAB_BASE   0xe000ff00 /* ROM table */

/* NVM area */

#define SAM_NVM_CALIBAREA   0x00800080 /* NVM software calibration area */
#define SAM_NVM_USERPAGE    0x00804000 /* NVM user page (512b) */
#define SAM_NVM_SNWORD0     0x008061fc /* Serial number word 0 */
#define SAM_NVM_SNWORD1     0x00806010 /* Serial number word 1 */
#define SAM_NVM_SNWORD2     0x00806014 /* Serial number word 2 */
#define SAM_NVM_SNWORD3     0x00806018 /* Serial number word 3 */

/* AHB-APB Bridge A */

#define SAM_PAC_BASE        0x40000000 /* Peripheral Access Controller 0 */
#define SAM_PM_BASE         0x40000400 /* Power Manager (PM) */
#define SAM_MCLK_BASE       0x40000800 /* Main clock (MCLK) */
#define SAM_RSTC_BASE       0x40000c00 /* Reset Controller (RSTC) */
#define SAM_OSCCTRL_BASE    0x40001000 /* OSCCTRL */
#define SAM_OSC32KCTRL_BASE 0x40001400 /* OSC32KCTRL */
#define SAM_SUPC_BASE       0x40001800 /* Supply Controller (SUPC) */
#define SAM_GCLK_BASE       0x40001c00 /* Generic Clock Controller */
#define SAM_WDT_BASE        0x40002000 /* Watchdog Timer */
#define SAM_RTC_BASE        0x40002400 /* Real-Time Counter */
#define SAM_EIC_BASE        0x40002800 /* External Interrupt Controller */
#define SAM_FREQM_BASE      0x40002c00 /* Frequency Meter (FREQM) */
#define SAM_SERCOM0_BASE    0x40003000 /* Serial Communication Interface 0 */
#define SAM_SERCOM1_BASE    0x40003400 /* Serial Communication Interface 1 */
#define SAM_TC0_BASE        0x40003800 /* Timer/Counter 2 */
#define SAM_TC1_BASE        0x40003c00 /* Timer/Counter 3 */
                                       /* Reserved */

/* AHB-APB Bridge B */

#define SAM_USB_BASE        0x41000000 /* USB */
#define SAM_DSU_BASE        0x41002000 /* Device Service Unit (DSU) */
#define SAM_NVMCTRL_BASE    0x41004000 /* Non-Volatile Memory Controller (NVMCTRL) */
#define SAM_CMCCC_BASE      0x41006000 /* Cortex-M Cache Controller (CMCC) */
#define SAM_PORT_BASE       0x41008000 /* Ports */
#define SAM_DMAC_BASE       0x4100a000 /* DMA Controller */
                                       /* Reserved */
#define SAM_EVSYS_BASE      0x4100e000 /* Event System */
                                       /* Reserved */
#define SAM_SERCOM2_BASE    0x41012000 /* Serial Communication Interface 2 */
#define SAM_SERCOM3_BASE    0x41014000 /* Serial Communication Interface 3 */
#define SAM_TCC0_BASE       0x41016000 /* Timer/Counter Control 0 */
#define SAM_TCC1_BASE       0x41018000 /* Timer/Counter Control 1 */
#define SAM_TC2_BASE        0x4101a000 /* Timer/Counter 2 */
#define SAM_TC3_BASE        0x4101c000 /* Timer/Counter 3 */
                                       /* Reserved */
#define SAM_RAMECC_BASE     0x41020000 /* RAM Error Correction Code (RAMECC) */
                                       /* Reserved */

/* AHB-APB Bridge C */

#define SAM_CAN0_BASE       0x42000000 /* CAN0 */
#define SAM_CAN1_BASE       0x42000400 /* CAN1 */
#define SAM_GMAC_BASE       0x42000800 /* GMAC */
#define SAM_TCC2_BASE       0x42000c00 /* Timer/Counter Control 2 */
#define SAM_TCC3_BASE       0x42001000 /* Timer/Counter Control 3 */
#define SAM_TC4_BASE        0x42001400 /* Timer/Counter 4 */
#define SAM_TC5_BASE        0x42001800 /* Timer/Counter 5 */
#define SAM_PDEC_BASE       0x42001c00 /* Position Decoder (PDEC) */
#define SAM_AC_BASE         0x42002000 /* Analog Comparator (AC) */
#define SAM_AES_BASE        0x42002400 /* Advanced Encryption Standard (AES) */
#define SAM_TRNG_BASE       0x42002800 /* True Random Number Generator (TRNG) */
#define SAM_ICM_BASE        0x42002c00 /* Integrity Check Monitor (ICM) */
#define SAM_PUKCC_BASE      0x42003000 /* Public-Key Cryptography Controller (PUKCC) */
#define SAM_QSPIC_BASE      0x42003400 /* QSPI controller */
#define SAM_CCL_BASE        0x42003800 /* Configurable Custom Logic (CCL) */
                                       /* Reserved */

/* AHB-APB Bridge D */

#define SAM_SERCOM4_BASE    0x43000000 /* Serial Communication Interface 4 */
#define SAM_SERCOM5_BASE    0x43000400 /* Serial Communication Interface 5 */
#define SAM_SERCOM6_BASE    0x43000800 /* Serial Communication Interface 6 */
#define SAM_SERCOM7_BASE    0x43000c00 /* Serial Communication Interface 7 */
#define SAM_TCC4_BASE       0x43001000 /* Timer/Counter Control 4 */
#define SAM_TC6_BASE        0x43001400 /* Timer/Counter 6 */
#define SAM_TC7_BASE        0x43001800 /* Timer/Counter 7 */
#define SAM_ADC0_BASE       0x43001c00 /* Analog-to-Digital Converter 0 (ADC0) */
#define SAM_ADC1_BASE       0x43002000 /* Analog-to-Digital Converter 1 (ADC1) */
#define SAM_DAC_BASE        0x43002400 /* Digital-to-Analog Converter (DAC) */
#define SAM_I2S_BASE        0x43002800 /* Inter IC Sound (I2S) */
#define SAM_PCC_BASE        0x44002c00 /* Parallel Capture Controller (PCC) */
                                       /* Reserved */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMD5E5_HARDWARE_SAMD5E5_MEMORYMAP_H */
