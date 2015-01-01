/************************************************************************************
 * arch/arm/src/tiva/chip/tm4c_flash.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Based on register definitions provided by:
 *
 *   Copyright (C) 2014 TRD2 Inc. All rights reserved.
 *   Author: Calvin Maguranis <calvin.maguranis@trd2inc.com>
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

#ifndef __ARCH_ARM_SRC_TIVA_CHIP_TM4C_FLASH_H
#define __ARCH_ARM_SRC_TIVA_CHIP_TM4C_FLASH_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* FLASH dimensions ****************************************************************/

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)

/* For the TM4C129X family, the Flash memory is configured in groups of four banks
 * four banks of 16K x 128 bits (4 * 256 KB total) which are two-way interleaved.
 * Because the memory is two-way interleaved and each bank individually is an 8-KB
 * sector, when the user erases a sector, using the ERASE bits in the Flash Memory
 * Control (FMC) register, it is a 16 KB erase.
 */

#  define TIVA_FLASH_NPAGES        64
#  define TIVA_FLASH_PAGESIZE      16384
#  define FLASH_PROTECT_SIZE       2048

#else
#  warning "No flash dimensions defined for selected chip."
#endif

#define TIVA_FLASH_SIZE            (TIVA_FLASH_NPAGES * TIVA_FLASH_PAGESIZE)

/* Flash register offsets ************************************************************/

/* Internal Memory Registers (Relative to Internal Memory Control Offset) */

#define TIVA_FLASH_FMA_OFFSET        0x0000 /* Flash Memory Address */
#define TIVA_FLASH_FMD_OFFSET        0x0004 /* Flash Memory Data */
#define TIVA_FLASH_FMC_OFFSET        0x0008 /* Flash Memory Control */
#define TIVA_FLASH_FCRIS_OFFSET      0x000c /* Flash Controller Raw Interrupt Status */
#define TIVA_FLASH_FCIM_OFFSET       0x0010 /* Flash Controller Interrupt Mask */
#define TIVA_FLASH_FCMISC_OFFSET     0x0014 /* Flash Controller Masked Interrupt Status and Clear */
#define TIVA_FLASH_FMC2_OFFSET       0x0020 /* Flash Memory Control 2 */
#define TIVA_FLASH_FWBVAL_OFFSET     0x0030 /* Flash Write Buffer Valid */
#define TIVA_FLASH_FLPEKEY_OFFSET    0x003c /* Flash Program/Erase Key */
#define TIVA_FLASH_FWBN_OFFSET       0x0100 /* Flash Write Buffer n */
#define TIVA_FLASH_PP_OFFSET         0x0fc0 /* Flash Peripheral Properties */
#define TIVA_FLASH_FSIZE_OFFSET      0x0fc0 /* Flash Size */
#define TIVA_FLASH_SSIZE_OFFSET      0x0fc4 /* SRAM Size */
#define TIVA_FLASH_CONF_OFFSET       0x0fc8 /* Flash Configuration Register */
#define TIVA_FLASH_ROMSWMAP_OFFSET   0x0fcc /* ROM Software Map */
#define TIVA_FLASH_DMASZ_OFFSET      0x0fd0 /* Flash DMA Address Size */
#define TIVA_FLASH_DMAST_OFFSET      0x0fd4 /* Flash DMA Starting Address */

/* Memory Registers (Relative to System Control Offset) */

#define TIVA_FLASH_RVP_OFFSET        0x00d4 /* Reset Vector Pointer */
#define TIVA_FLASH_RMCTL_OFFSET      0x00f0 /* ROM Control */
#define TIVA_FLASH_BOOTCFG_OFFSET    0x01d0 /* Boot Configuration */
#define TIVA_FLASH_USERREG0_OFFSET   0x01e0 /* User Register 0 */
#define TIVA_FLASH_USERREG1_OFFSET   0x01e4 /* User Register 1 */
#define TIVA_FLASH_USERREG2_OFFSET   0x01e8 /* User Register 2 */
#define TIVA_FLASH_USERREG3_OFFSET   0x01ec /* User Register 3 */
#define TIVA_FLASH_FMPRE0_OFFSET     0x0200 /* Flash Memory Protection Read Enable 0 */
#define TIVA_FLASH_FMPRE1_OFFSET     0x0204 /* Flash Memory Protection Read Enable 1 */
#define TIVA_FLASH_FMPRE2_OFFSET     0x0208 /* Flash Memory Protection Read Enable 2 */
#define TIVA_FLASH_FMPRE3_OFFSET     0x020c /* Flash Memory Protection Read Enable 3 */
#define TIVA_FLASH_FMPRE4_OFFSET     0x0210 /* Flash Memory Protection Read Enable 4 */
#define TIVA_FLASH_FMPRE5_OFFSET     0x0214 /* Flash Memory Protection Read Enable 5 */
#define TIVA_FLASH_FMPRE6_OFFSET     0x0218 /* Flash Memory Protection Read Enable 6 */
#define TIVA_FLASH_FMPRE7_OFFSET     0x021c /* Flash Memory Protection Read Enable 7 */
#define TIVA_FLASH_FMPRE8_OFFSET     0x0220 /* Flash Memory Protection Read Enable 8 */
#define TIVA_FLASH_FMPRE9_OFFSET     0x0224 /* Flash Memory Protection Read Enable 9 */
#define TIVA_FLASH_FMPRE10_OFFSET    0x0228 /* Flash Memory Protection Read Enable 10 */
#define TIVA_FLASH_FMPRE11_OFFSET    0x022c /* Flash Memory Protection Read Enable 11 */
#define TIVA_FLASH_FMPRE12_OFFSET    0x0230 /* Flash Memory Protection Read Enable 12 */
#define TIVA_FLASH_FMPRE13_OFFSET    0x0234 /* Flash Memory Protection Read Enable 13 */
#define TIVA_FLASH_FMPRE14_OFFSET    0x0238 /* Flash Memory Protection Read Enable 14 */
#define TIVA_FLASH_FMPRE15_OFFSET    0x023c /* Flash Memory Protection Read Enable 15 */
#define TIVA_FLASH_FMPPE0_OFFSET     0x0400 /* Flash Memory Protection Program Enable 0 */
#define TIVA_FLASH_FMPPE1_OFFSET     0x0404 /* Flash Memory Protection Program Enable 1 */
#define TIVA_FLASH_FMPPE2_OFFSET     0x0408 /* Flash Memory Protection Program Enable 2 */
#define TIVA_FLASH_FMPPE3_OFFSET     0x040c /* Flash Memory Protection Program Enable 3 */
#define TIVA_FLASH_FMPPE4_OFFSET     0x0410 /* Flash Memory Protection Program Enable 4 */
#define TIVA_FLASH_FMPPE5_OFFSET     0x0414 /* Flash Memory Protection Program Enable 5 */
#define TIVA_FLASH_FMPPE6_OFFSET     0x0418 /* Flash Memory Protection Program Enable 6 */
#define TIVA_FLASH_FMPPE7_OFFSET     0x041c /* Flash Memory Protection Program Enable 7 */
#define TIVA_FLASH_FMPPE8_OFFSET     0x0420 /* Flash Memory Protection Program Enable 8 */
#define TIVA_FLASH_FMPPE9_OFFSET     0x0424 /* Flash Memory Protection Program Enable 9 */
#define TIVA_FLASH_FMPPE10_OFFSET    0x0428 /* Flash Memory Protection Program Enable 10 */
#define TIVA_FLASH_FMPPE11_OFFSET    0x042c /* Flash Memory Protection Program Enable 11 */
#define TIVA_FLASH_FMPPE12_OFFSET    0x0430 /* Flash Memory Protection Program Enable 12 */
#define TIVA_FLASH_FMPPE13_OFFSET    0x0434 /* Flash Memory Protection Program Enable 13 */
#define TIVA_FLASH_FMPPE14_OFFSET    0x0438 /* Flash Memory Protection Program Enable 14 */
#define TIVA_FLASH_FMPPE15_OFFSET    0x043c /* Flash Memory Protection Program Enable 15 */

/* Flash register addresses **********************************************************/
/* Internal Memory Registers (Internal Memory Control Offset) */

#define TIVA_FLASH_FMA               (TIVA_FLASHCON_BASE + TIVA_FLASH_FMA_OFFSET)
#define TIVA_FLASH_FMD               (TIVA_FLASHCON_BASE + TIVA_FLASH_FMD_OFFSET)
#define TIVA_FLASH_FMC               (TIVA_FLASHCON_BASE + TIVA_FLASH_FMC_OFFSET)
#define TIVA_FLASH_FCRIS             (TIVA_FLASHCON_BASE + TIVA_FLASH_FCRIS_OFFSET)
#define TIVA_FLASH_FCIM              (TIVA_FLASHCON_BASE + TIVA_FLASH_FCIM_OFFSET)
#define TIVA_FLASH_FCMISC            (TIVA_FLASHCON_BASE + TIVA_FLASH_FCMISC_OFFSET)
#define TIVA_FLASH_FMC2              (TIVA_FLASHCON_BASE + TIVA_FLASH_FMC2_OFFSET)
#define TIVA_FLASH_FWBVAL            (TIVA_FLASHCON_BASE + TIVA_FLASH_FWBVAL_OFFSET)
#define TIVA_FLASH_FLPEKEY           (TIVA_FLASHCON_BASE + TIVA_FLASH_FLPEKEY_OFFSET)
#define TIVA_FLASH_FWBN              (TIVA_FLASHCON_BASE + TIVA_FLASH_FWBN_OFFSET)
#define TIVA_FLASH_PP                (TIVA_FLASHCON_BASE + TIVA_FLASH_PP_OFFSET)
#define TIVA_FLASH_FSIZE             (TIVA_FLASHCON_BASE + TIVA_FLASH_FSIZE_OFFSET)
#define TIVA_FLASH_SSIZE             (TIVA_FLASHCON_BASE + TIVA_FLASH_SSIZE_OFFSET)
#define TIVA_FLASH_CONF              (TIVA_FLASHCON_BASE + TIVA_FLASH_CONF_OFFSET)
#define TIVA_FLASH_ROMSWMAP          (TIVA_FLASHCON_BASE + TIVA_FLASH_ROMSWMAP_OFFSET)
#define TIVA_FLASH_DMASZ             (TIVA_FLASHCON_BASE + TIVA_FLASH_DMASZ_OFFSET)
#define TIVA_FLASH_DMAST             (TIVA_FLASHCON_BASE + TIVA_FLASH_DMAST_OFFSET)

/* Memory Registers (System Control Offset) */

#define TIVA_FLASH_RVP               (TIVA_SYSCON_BASE + TIVA_FLASH_RVP_OFFSET)
#define TIVA_FLASH_RMCTL             (TIVA_SYSCON_BASE + TIVA_FLASH_RMCTL_OFFSET)
#define TIVA_FLASH_BOOTCFG           (TIVA_SYSCON_BASE + TIVA_FLASH_BOOTCFG_OFFSET)
#define TIVA_FLASH_USERREG0          (TIVA_SYSCON_BASE + TIVA_FLASH_USERREG0_OFFSET)
#define TIVA_FLASH_USERREG1          (TIVA_SYSCON_BASE + TIVA_FLASH_USERREG1_OFFSET)
#define TIVA_FLASH_USERREG2          (TIVA_SYSCON_BASE + TIVA_FLASH_USERREG2_OFFSET)
#define TIVA_FLASH_USERREG3          (TIVA_SYSCON_BASE + TIVA_FLASH_USERREG3_OFFSET)
#define TIVA_FLASH_FMPRE0            (TIVA_SYSCON_BASE + TIVA_FLASH_FMPRE0_OFFSET)
#define TIVA_FLASH_FMPRE1            (TIVA_SYSCON_BASE + TIVA_FLASH_FMPRE1_OFFSET)
#define TIVA_FLASH_FMPRE2            (TIVA_SYSCON_BASE + TIVA_FLASH_FMPRE2_OFFSET)
#define TIVA_FLASH_FMPRE3            (TIVA_SYSCON_BASE + TIVA_FLASH_FMPRE3_OFFSET)
#define TIVA_FLASH_FMPRE4            (TIVA_SYSCON_BASE + TIVA_FLASH_FMPRE4_OFFSET)
#define TIVA_FLASH_FMPRE5            (TIVA_SYSCON_BASE + TIVA_FLASH_FMPRE5_OFFSET)
#define TIVA_FLASH_FMPRE6            (TIVA_SYSCON_BASE + TIVA_FLASH_FMPRE6_OFFSET)
#define TIVA_FLASH_FMPRE7            (TIVA_SYSCON_BASE + TIVA_FLASH_FMPRE7_OFFSET)
#define TIVA_FLASH_FMPRE8            (TIVA_SYSCON_BASE + TIVA_FLASH_FMPRE8_OFFSET)
#define TIVA_FLASH_FMPRE9            (TIVA_SYSCON_BASE + TIVA_FLASH_FMPRE9_OFFSET)
#define TIVA_FLASH_FMPRE10           (TIVA_SYSCON_BASE + TIVA_FLASH_FMPRE10_OFFSET)
#define TIVA_FLASH_FMPRE11           (TIVA_SYSCON_BASE + TIVA_FLASH_FMPRE11_OFFSET)
#define TIVA_FLASH_FMPRE12           (TIVA_SYSCON_BASE + TIVA_FLASH_FMPRE12_OFFSET)
#define TIVA_FLASH_FMPRE13           (TIVA_SYSCON_BASE + TIVA_FLASH_FMPRE13_OFFSET)
#define TIVA_FLASH_FMPRE14           (TIVA_SYSCON_BASE + TIVA_FLASH_FMPRE14_OFFSET)
#define TIVA_FLASH_FMPRE15           (TIVA_SYSCON_BASE + TIVA_FLASH_FMPRE15_OFFSET)
#define TIVA_FLASH_FMPPE0            (TIVA_SYSCON_BASE + TIVA_FLASH_FMPPE0_OFFSET)
#define TIVA_FLASH_FMPPE1            (TIVA_SYSCON_BASE + TIVA_FLASH_FMPPE1_OFFSET)
#define TIVA_FLASH_FMPPE2            (TIVA_SYSCON_BASE + TIVA_FLASH_FMPPE2_OFFSET)
#define TIVA_FLASH_FMPPE3            (TIVA_SYSCON_BASE + TIVA_FLASH_FMPPE3_OFFSET)
#define TIVA_FLASH_FMPPE4            (TIVA_SYSCON_BASE + TIVA_FLASH_FMPPE4_OFFSET)
#define TIVA_FLASH_FMPPE5            (TIVA_SYSCON_BASE + TIVA_FLASH_FMPPE5_OFFSET)
#define TIVA_FLASH_FMPPE6            (TIVA_SYSCON_BASE + TIVA_FLASH_FMPPE6_OFFSET)
#define TIVA_FLASH_FMPPE7            (TIVA_SYSCON_BASE + TIVA_FLASH_FMPPE7_OFFSET)
#define TIVA_FLASH_FMPPE8            (TIVA_SYSCON_BASE + TIVA_FLASH_FMPPE8_OFFSET)
#define TIVA_FLASH_FMPPE9            (TIVA_SYSCON_BASE + TIVA_FLASH_FMPPE9_OFFSET)
#define TIVA_FLASH_FMPPE10           (TIVA_SYSCON_BASE + TIVA_FLASH_FMPPE10_OFFSET)
#define TIVA_FLASH_FMPPE11           (TIVA_SYSCON_BASE + TIVA_FLASH_FMPPE11_OFFSET)
#define TIVA_FLASH_FMPPE12           (TIVA_SYSCON_BASE + TIVA_FLASH_FMPPE12_OFFSET)
#define TIVA_FLASH_FMPPE13           (TIVA_SYSCON_BASE + TIVA_FLASH_FMPPE13_OFFSET)
#define TIVA_FLASH_FMPPE14           (TIVA_SYSCON_BASE + TIVA_FLASH_FMPPE14_OFFSET)
#define TIVA_FLASH_FMPPE15           (TIVA_SYSCON_BASE + TIVA_FLASH_FMPPE15_OFFSET)

/* Flash register bit definitions ****************************************************/

/* Internal Memory Registers (Relative to Internal Memory Control Offset) */

/* Flash Memory Address */

#define FLASH_FMA_OFFSET_SHIFT       (0)/* Bits 0-19: Address Offset */
#define FLASH_FMA_OFFSET_MASK        (0xfffff << FLASH_FMA_OFFSET_SHIFT)

/* Flash Memory Data (32-bit value) */

/* Flash Memory Control */

#define FLASH_FMC_WRITE              (1 << 0)  /* Bit 0:  Write a Word into Flash Memory */
#define FLASH_FMC_ERASE              (1 << 1)  /* Bit 1:  Erase a Page of Flash Memory */
#define FLASH_FMC_MERASE             (1 << 2)  /* Bit 2:  Mass Erase Flash Memory */
#define FLASH_FMC_COMT               (1 << 3)  /* Bit 3:  Commit Register Value */
#define FLASH_FMC_WRKEY_SHIFT        (16)      /* Bits 16-31: FLASH write key */
#define FLASH_FMC_WRKEY_MASK         (0xffff << FLASH_FMC_WRKEY_SHIFT)
#  define FLASH_FMC_WRKEY            (0xa442 << FLASH_FMC_WRKEY_SHIFT)

/* Flash Controller Raw Interrupt Status */

#define FLASH_FCRIS_ARIS             (1 << 0)  /* Bit 0:  Access Raw Interrupt Status */
#define FLASH_FCRIS_PRIS             (1 << 1)  /* Bit 1:  Programming Raw Interrupt Status */
#define FLASH_FCRIS_ERIS             (1 << 2)  /* Bit 2:  EEPROM Raw Interrupt Status */
#define FLASH_FCRIS_VOLTRIS          (1 << 9)  /* Bit 9:  Pump Voltage Raw Interrupt Status */
#define FLASH_FCRIS_INVDRIS          (1 << 10) /* Bit 10: Invalid Data Raw Interrupt Status */
#define FLASH_FCRIS_ERRIS            (1 << 11) /* Bit 11: Erase Verify Error Raw Interrupt Status */
#define FLASH_FCRIS_PROGRIS          (1 << 13) /* Bit 13: Program Verify Error Raw Interrupt Status */

/* Flash Controller Interrupt Mask */

#define FLASH_FCIM_AMASK             (1 << 0)  /* Bit 0:  Access Interrupt Mask */
#define FLASH_FCIM_PMASK             (1 << 1)  /* Bit 1:  Programming Interrupt Mask */
#define FLASH_FCIM_EMASK             (1 << 2)  /* Bit 2:  EEPROM Interrupt Mask */
#define FLASH_FCIM_VOLTMASK          (1 << 9)  /* Bit 8:  VOLT Interrupt Mask */
#define FLASH_FCIM_INVDMASK          (1 << 10) /* Bit 10: Invalid Data Interrupt Mask */
#define FLASH_FCIM_ERMASK            (1 << 11) /* Bit 11: ERVER Interrupt Mask */
#define FLASH_FCIM_PROGMASK          (1 << 13) /* Bit 13: PROGVER Interrupt Mask */

/* Flash Controller Masked Interrupt Status and Clear */

#define FLASH_FCMISC_AMISC           (1 << 0)  /* Bit 0:  Access Masked Interrupt Status and Clear */
#define FLASH_FCMISC_PMISC           (1 << 1)  /* Bit 1:  Programming Masked Interrupt Status and Clear */
#define FLASH_FCMISC_EMISC           (1 << 2)  /* Bit 2:  EEPROM Masked Interrupt Status and Clear */
#define FLASH_FCMISC_VOLTMISC        (1 << 9)  /* Bit 9:  VOLT Masked Interrupt Status and Clear */
#define FLASH_FCMISC_INVDMISC        (1 << 10) /* Bit 10: Invalid Data Masked Interrupt Status and Clear */
#define FLASH_FCMISC_ERMISC          (1 << 11) /* Bit 11: ERVER Masked Interrupt Status and Clear */
#define FLASH_FCMISC_PROGMISC        (1 << 13) /* Bit 13: PROGVER Masked Interrupt Status and Clear */

/* Flash Memory Control 2 */

#define FLASH_FMC2_WRBUF             (1 << 0)  /* Bit 0:  Buffered Flash Memory Write */
#define FLASH_FMC_WRKEY_SHIFT        (16)      /* Bits 16-31: FLASH write key */
#define FLASH_FMC_WRKEY_MASK         (0xffff << FLASH_FMC_WRKEY_SHIFT)
#  define FLASH_FMC_WRKEY            (0xa442 << FLASH_FMC_WRKEY_SHIFT)

/* Flash Write Buffer Valid (32-bit value) */

/* Flash Program/Erase Key */

#define FLASH_FLPEKEY_PEKEY_SHIFT    (0)       /* Bits 0-16: Key Value */
#define FLASH_FLPEKEY_PEKEY_MASK     (0xffff << FLASH_FLPEKEY_PEKEY_SHIFT)

/* Flash Write Buffer n (32-bit value) */

/* Flash Peripheral Properties */

#define FLASH_PP_SIZE_SHIFT          (0)       /* Bits 0-15: Flash Size */
#define FLASH_PP_SIZE_MASK           (0xffff << FLASH_PP_SIZE_SHIFT)
#  define FLASH_PP_SIZE_512KB        (255 << FLASH_PP_SIZE_SHIFT)/* 512 KB of Flash */
#  define FLASH_PP_SIZE_1MB          (511 << FLASH_PP_SIZE_SHIFT)/* 1024 KB of Flash */
#define FLASH_PP_MAINSS_SHIFT        (16)      /* Bit1 16-18: Flash Sector Size of the physical bank */
#define FLASH_PP_MAINSS_MASK         (7 << FLASH_PP_MAINSS_SHIFT)
#  define FLASH_PP_MAINSS_1KB        (0 << FLASH_PP_MAINSS_SHIFT) /* 1 KB */
#  define FLASH_PP_MAINSS_2KB        (1 << FLASH_PP_MAINSS_SHIFT) /* 2 KB */
#  define FLASH_PP_MAINSS_4KB        (2 << FLASH_PP_MAINSS_SHIFT) /* 4 KB */
#  define FLASH_PP_MAINSS_8KB        (3 << FLASH_PP_MAINSS_SHIFT) /* 8 KB */
#  define FLASH_PP_MAINSS_16KB       (4 << FLASH_PP_MAINSS_SHIFT) /* 16 KB */
#define FLASH_PP_EESS_SHIFT          (19)      /* Bits 19-22: EEPROM Sector Size of the physical bank */
#define FLASH_PP_EESS_MASK           (15 << FLASH_PP_EESS_SHIFT)
#  define FLASH_PP_EESS_1KB          (0 << FLASH_PP_EESS_SHIFT) /* 1 KB */
#  define FLASH_PP_EESS_2KB          (1 << FLASH_PP_EESS_SHIFT) /* 2 KB */
#  define FLASH_PP_EESS_4KB          (2 << FLASH_PP_EESS_SHIFT) /* 4 KB */
#  define FLASH_PP_EESS_8KB          (3 << FLASH_PP_EESS_SHIFT) /* 8 KB */
#define FLASH_PP_DFA                 (1 << 28) /* Bit 28: DMA Flash Access */
#define FLASH_PP_FMM                 (1 << 29) /* Bit 29: Flash Mirror Mode */
#define FLASH_PP_PFC                 (1 << 30) /* Bit 30: Prefetch Buffer Mode */

/* Flash Size */

#define FLASH_FSIZE_SIZE_SHIFT       (0)  /* Bits 0-15: Flash Size */
#define FLASH_FSIZE_SIZE_MASK        (0xffff << FLASH_FSIZE_SIZE_SHIFT)
#define FLASH_FSIZE_SIZE_32KB        (15 << FLASH_FSIZE_SIZE_SHIFT)  /* 32 KB of Flash */
#define FLASH_FSIZE_SIZE_64KB        (31 << FLASH_FSIZE_SIZE_SHIFT)  /* 64 KB of Flash */
#define FLASH_FSIZE_SIZE_128KB       (63 << FLASH_FSIZE_SIZE_SHIFT)  /* 128 KB of Flash */
#define FLASH_FSIZE_SIZE_256KB       (127 << FLASH_FSIZE_SIZE_SHIFT) /* 256 KB of Flash */

/* SRAM Size */

#define FLASH_SSIZE_SIZE_SHIFT       (0)  /* Bits 0-15: SRAM Size */
#define FLASH_SSIZE_SIZE_MASK        (0xffff << FLASH_SSIZE_SIZE_SHIFT)
#  define FLASH_SSIZE_SIZE_12KB      (47 << FLASH_SSIZE_SIZE_SHIFT)   /* 12 KB of SRAM */
#  define FLASH_SSIZE_SIZE_24KB      (95 << FLASH_SSIZE_SIZE_SHIFT)   /* 24 KB of SRAM */
#  define FLASH_SSIZE_SIZE_32KB      (127 << FLASH_SSIZE_SIZE_SHIFT)  /* 32 KB of SRAM */
#  define FLASH_SSIZE_SIZE_256KB     (1023 << FLASH_SSIZE_SIZE_SHIFT) /* 256 KB of SRAM */

/* Flash Configuration Register */

#define FLASH_CONF_FPFOFF            (1 << 16) /* Bit 16: Force Prefetch Off */
#define FLASH_CONF_FPFON             (1 << 17) /* Bit 17: Force Prefetch On */
#define FLASH_CONF_CLRTV             (1 << 20) /* Bit 20: Clear Valid Tags */
#define FLASH_CONF_SPFE              (1 << 29) /* Bit 29: Single Prefetch Mode Enable */
#define FLASH_CONF_FMME              (1 << 30) /* Bit 30: Flash Mirror Mode Enable */

/* ROM Software Map */

#define FLASH_ROMSWMAP_SW0EN_SHIFT     (0)     /* Bits 0-1: ROM SW Region 0 Availability */
#define FLASH_ROMSWMAP_SW0EN_MASK      (3 << FLASH_ROMSWMAP_SW0EN_SHIFT)
#  define FLASH_ROMSWMAP_SW0EN_NOTVIS  (0 << FLASH_ROMSWMAP_SW0EN_SHIFT)  /* Software region not available to the core */
#  define FLASH_ROMSWMAP_SAFERTOS      (1 << FLASH_ROMSWMAP_SW0EN_SHIFT)  /* SafeRTOS Present */
#  define FLASH_ROMSWMAP_SW0EN_CORE    (1 << FLASH_ROMSWMAP_SW0EN_SHIFT)  /* Region available to core */
#define FLASH_ROMSWMAP_SW1EN_SHIFT     (2)     /* Bits 2-3: ROM SW Region 1 Availability */
#define FLASH_ROMSWMAP_SW1EN_MASK      (3 << FLASH_ROMSWMAP_SW1EN_SHIFT)
#  define FLASH_ROMSWMAP_SW1EN_NOTVIS  (0 << FLASH_ROMSWMAP_SW1EN_SHIFT)  /* Software region not available to the core */
#  define FLASH_ROMSWMAP_SW1EN_CORE    (1 << FLASH_ROMSWMAP_SW1EN_SHIFT)  /* Region available to core */
#define FLASH_ROMSWMAP_SW2EN_SHIFT     (4)     /* Bits 4-5: ROM SW Region 2 Availability */
#define FLASH_ROMSWMAP_SW2EN_MASK      (3 << FLASH_ROMSWMAP_SW2EN_SHIFT)
#  define FLASH_ROMSWMAP_SW2EN_NOTVIS  (0 << FLASH_ROMSWMAP_SW2EN_SHIFT)  /* Software region not available to the core */
#  define FLASH_ROMSWMAP_SW2EN_CORE    (1 << FLASH_ROMSWMAP_SW2EN_SHIFT)  /* Region available to core */
#define FLASH_ROMSWMAP_SW3EN_SHIFT     (6)     /* Bits 6-7: ROM SW Region 3 Availability */
#define FLASH_ROMSWMAP_SW3EN_MASK      (3 << FLASH_ROMSWMAP_SW3EN_SHIFT)
#  define FLASH_ROMSWMAP_SW3EN_NOTVIS  (0 << FLASH_ROMSWMAP_SW3EN_SHIFT)  /* Software region not available to the core */
#  define FLASH_ROMSWMAP_SW3EN_CORE    (1 << FLASH_ROMSWMAP_SW3EN_SHIFT)  /* Region available to core */
#define FLASH_ROMSWMAP_SW4EN_SHIFT     (8)     /* Bits 8-9: ROM SW Region 4 Availability */
#define FLASH_ROMSWMAP_SW4EN_MASK      (3 << FLASH_ROMSWMAP_SW4EN_SHIFT)
#  define FLASH_ROMSWMAP_SW4EN_NOTVIS  (0 << FLASH_ROMSWMAP_SW4EN_SHIFT)  /* Software region not available to the core */
#  define FLASH_ROMSWMAP_SW4EN_CORE    (1 << FLASH_ROMSWMAP_SW4EN_SHIFT)  /* Region available to core */
#define FLASH_ROMSWMAP_SW5EN_SHIFT     (10)    /* Bits 10-11: ROM SW Region 5 Availability */
#define FLASH_ROMSWMAP_SW5EN_MASK      (3 << FLASH_ROMSWMAP_SW5EN_SHIFT)
#  define FLASH_ROMSWMAP_SW5EN_NOTVIS  (0 << FLASH_ROMSWMAP_SW5EN_SHIFT)  /* Software region not available to the core */
#  define FLASH_ROMSWMAP_SW5EN_CORE    (1 << FLASH_ROMSWMAP_SW5EN_SHIFT) /* Region available to core */
#define FLASH_ROMSWMAP_SW6EN_SHIFT     (12)    /* Bits 12-13: ROM SW Region 6 Availability */
#define FLASH_ROMSWMAP_SW6EN_MASK      (3 << FLASH_ROMSWMAP_SW6EN_SHIFT)
#  define FLASH_ROMSWMAP_SW6EN_NOTVIS  (0 << FLASH_ROMSWMAP_SW6EN_SHIFT) /* Software region not available to the core */
#  define FLASH_ROMSWMAP_SW6EN_CORE    (1 << FLASH_ROMSWMAP_SW6EN_SHIFT) /* Region available to core */
#define FLASH_ROMSWMAP_SW7EN_SHIFT     (14)    /* Bits 14-15: ROM SW Region 7 Availability */
#define FLASH_ROMSWMAP_SW7EN_MASK      (3 << FLASH_ROMSWMAP_SW7EN_SHIFT)
#  define FLASH_ROMSWMAP_SW7EN_NOTVIS  (0 << FLASH_ROMSWMAP_SW7EN_SHIFT) /* Software region not available to the core */
#  define FLASH_ROMSWMAP_SW7EN_CORE    (1 << FLASH_ROMSWMAP_SW7EN_SHIFT) /* Region available to core */

/* Flash DMA Address Size */

#define FLASH_DMASZ_SIZE_SHIFT       (0)       /* Bits 0-17  uDMA-accessible Memory Size */
#define FLASH_DMASZ_SIZE_MASK        (0x3ffff << FLASH_DMASZ_SIZE_SHIFT)

/* Flash DMA Starting Address */

#define FLASH_DMAST_ADDR_MASK        (0x1ffff800) /* Bits 11-18:  Starting address of the
                                                   * flash region accessible by uDMA */

/* Memory Registers (System Control Offset) */

/* Reset Vector Pointer (32-bit value) */

/* ROM Control */

#define FLASH_RMCTL_BA               (1 << 0)  /* Bit 0:  Boot Alias */

/* Boot Configuration */

#define FLASH_BOOTCFG_DBG0           (1 << 0)  /* Bit 0:  Debug Control 0 */
#define FLASH_BOOTCFG_DBG1           (1 << 1)  /* Bit 1:  Debug Control 1 */
#define FLASH_BOOTCFG_KEY            (1 << 4)  /* Bit 4:  KEY Select */
#define FLASH_BOOTCFG_EN             (1 << 8)  /* Bit 8:  Boot GPIO Enable */
#define FLASH_BOOTCFG_POL            (1 << 9)  /* Bit 9:  Boot GPIO Polarity */
#define FLASH_BOOTCFG_PIN_SHIFT      (10)      /* Bits 10-12:  Boot GPIO Pin */
#define FLASH_BOOTCFG_PIN_MASK       (7 <<  FLASH_BOOTCFG_PIN_SHIFT)
#  define FLASH_BOOTCFG_PIN_0        (0 << FLASH_BOOTCFG_PIN_SHIFT) /* Pin 0 */
#  define FLASH_BOOTCFG_PIN_1        (1 << FLASH_BOOTCFG_PIN_SHIFT) /* Pin 1 */
#  define FLASH_BOOTCFG_PIN_2        (2 << FLASH_BOOTCFG_PIN_SHIFT) /* Pin 2 */
#  define FLASH_BOOTCFG_PIN_3        (3 << FLASH_BOOTCFG_PIN_SHIFT) /* Pin 3 */
#  define FLASH_BOOTCFG_PIN_4        (4 << FLASH_BOOTCFG_PIN_SHIFT) /* Pin 4 */
#  define FLASH_BOOTCFG_PIN_5        (5 << FLASH_BOOTCFG_PIN_SHIFT) /* Pin 5 */
#  define FLASH_BOOTCFG_PIN_6        (6 << FLASH_BOOTCFG_PIN_SHIFT) /* Pin 6 */
#  define FLASH_BOOTCFG_PIN_7        (7 << FLASH_BOOTCFG_PIN_SHIFT) /* Pin 7 */
#define FLASH_BOOTCFG_PORT_SHIFT     (13)      /* Bits 13-15:  Boot GPIO Port */
#define FLASH_BOOTCFG_PORT_MASK      (7 << FLASH_BOOTCFG_PORT_SHIFT)
#  define FLASH_BOOTCFG_PORT_A       (0 << FLASH_BOOTCFG_PORT_SHIFT) /* Port A */
#  define FLASH_BOOTCFG_PORT_B       (1 << FLASH_BOOTCFG_PORT_SHIFT) /* Port B */
#  define FLASH_BOOTCFG_PORT_C       (2 << FLASH_BOOTCFG_PORT_SHIFT) /* Port C */
#  define FLASH_BOOTCFG_PORT_D       (3 << FLASH_BOOTCFG_PORT_SHIFT) /* Port D */
#  define FLASH_BOOTCFG_PORT_E       (4 << FLASH_BOOTCFG_PORT_SHIFT) /* Port E */
#  define FLASH_BOOTCFG_PORT_F       (5 << FLASH_BOOTCFG_PORT_SHIFT) /* Port F */
#  define FLASH_BOOTCFG_PORT_G       (6 << FLASH_BOOTCFG_PORT_SHIFT) /* Port G */
#  define FLASH_BOOTCFG_PORT_H       (7 << FLASH_BOOTCFG_PORT_SHIFT) /* Port H */
#define FLASH_BOOTCFG_NW             (1 << 31) /* Bit 31:  Not Written */

/* User Register 0-3 (32-bit value) */
/* Flash Memory Protection Read Enable 0-15 (32-bit, bit-encoded) */
/* Flash Memory Protection Program Enable 0-15 (32-bit, bit-encoded) */

#endif // __ARCH_ARM_SRC_TIVA_CHIP_TM4C_FLASH_H
