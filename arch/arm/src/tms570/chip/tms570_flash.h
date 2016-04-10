/****************************************************************************************************
 * arch/arm/src/tms570/chip/tms570_flash.h
 * FLASH Module Controller Register Definitions
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *
 *   TMS570LS04x/03x 16/32-Bit RISC Flash Microcontroller, Technical Reference Manual, Texas
 *   Instruments, Literature Number: SPNU517A, September 2013
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_TMS570_CHIP_TMS570_FLASH_H
#define __ARCH_ARM_SRC_TMS570_CHIP_TMS570_FLASH_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include "chip/tms570_memorymap.h"

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* Register Offsets *********************************************************************************/

#define TMS570_FLASH_FRDCNTL_OFFSET       0x000 /* Flash Option Control Register */
#define TMS570_FLASH_FEDACTRL1_OFFSET     0x008 /* Flash Error Detection and Correction Control Register 1 */
#define TMS570_FLASH_FEDACTRL2_OFFSET     0x00c /* Flash Error Detection and Correction Control Register 2 */
#define TMS570_FLASH_FCORERRCNT_OFFSET    0x010 /* Flash Correctable Error Count Register */
#define TMS570_FLASH_FCORERRADD_OFFSET    0x014 /* Flash Correctable Error Address Register */
#define TMS570_FLASH_FCORERRPOS_OFFSET    0x018 /* Flash Correctable Error Position Register */
#define TMS570_FLASH_FEDACSTATUS_OFFSET   0x01c /* Flash Error Detection and Correction Status Register */
#define TMS570_FLASH_FUNCERRADD_OFFSET    0x020 /* Flash Un-Correctable Error Address Register */
#define TMS570_FLASH_FEDACSDIS_OFFSET     0x024 /* Flash Error Detection and Correction Sector Disable Register */
#define TMS570_FLASH_FPRIMADDTAG_OFFSET   0x028 /* Flash Primary Address Tag Register */
#define TMS570_FLASH_FDUPDDTAG_OFFSET     0x02c /* Flash Duplicate Address Tag Register */
#define TMS570_FLASH_FBPROT_OFFSET        0x030 /* Flash Bank Protection Register */
#define TMS570_FLASH_FBSE_OFFSET          0x034 /* Flash Bank Sector Enable Register */
#define TMS570_FLASH_FBBUSY_OFFSET        0x038 /* Flash Bank Busy Register */
#define TMS570_FLASH_FBAC_OFFSET          0x03c /* Flash Bank Access Control Register */
#define TMS570_FLASH_FBFALLBACK_OFFSET    0x040 /* Flash Bank Fallback Power Register */
#define TMS570_FLASH_FBPRDY_OFFSET        0x044 /* Flash Bank/Pump Ready Register */
#define TMS570_FLASH_FPAC1_OFFSET         0x048 /* Flash Pump Access Control Register 1 */
#define TMS570_FLASH_FPAC2_OFFSET         0x04c /* Flash Pump Access Control Register 2 */
#define TMS570_FLASH_FMAC_OFFSET          0x050 /* Flash Module Access Control Register */
#define TMS570_FLASH_FMSTAT_OFFSET        0x054 /* Flash Module Status Register */
#define TMS570_FLASH_FEMUDMSW_OFFSET      0x058 /* EEPROM Emulation Data MSW Register */
#define TMS570_FLASH_FEMUDLSW_OFFSET      0x05c /* EEPROM Emulation Data LSW Register */
#define TMS570_FLASH_FEMUECC_OFFSET       0x060 /* EEPROM Emulation ECC Register */
#define TMS570_FLASH_FEMUADDR_OFFSET      0x068 /* EEPROM Emulation Address Register */
#define TMS570_FLASH_FDIAGCTRL_OFFSET     0x06c /* Diagnostic Control Register */
#define TMS570_FLASH_FRAWDATAH_OFFSET     0x070 /* Uncorrected Raw Data High Register */
#define TMS570_FLASH_FRAWDATAL_OFFSET     0x074 /* Uncorrected Raw Data Low Register */
#define TMS570_FLASH_FRAWECC_OFFSET       0x078 /* Uncorrected Raw ECC Register */
#define TMS570_FLASH_FPAROVR_OFFSET       0x07c /* Parity Override Register */
#define TMS570_FLASH_FEDACSDIS2_OFFSET    0x0c0 /* Flash Error Detection and Correction Sector Disable Register 2 */
#define TMS570_FLASH_FSMWRENA_OFFSET      0x288 /* FSM Register Write Enable */
#define TMS570_FLASH_FSMSECTOR_OFFSET     0x2a4 /* FSM Sector Register */
#define TMS570_FLASH_EEPROMCFG_OFFSET     0x2b8 /* EEPROM Emulation Configuration Register */
#define TMS570_FLASH_EECTRL1_OFFSET       0x308 /* EEPROM Emulation Error Detection and Correction Control Register 1 */
#define TMS570_FLASH_EECTRL2_OFFSET       0x30c /* EEPROM Emulation Error Detection and Correction Control Register 2 */
#define TMS570_FLASH_EECORERRCNT_OFFSET   0x310 /* EEPROM Emulation Correctable Error Count Register */
#define TMS570_FLASH_EECORERRADD_OFFSET   0x314 /* EEPROM Emulation Correctable Error Address Register */
#define TMS570_FLASH_EECORERRPOS_OFFSET   0x318 /* EEPROM Emulation Correctable Error Bit Position Register */
#define TMS570_FLASH_EESTATUS_OFFSET      0x31c /* EEPROM Emulation Error Status Register */
#define TMS570_FLASH_EEUNCERRADD_OFFSET   0x320 /* EEPROM Emulation Un-Correctable Error Address Register */
#define TMS570_FLASH_FCFGBANK_OFFSET      0x400 /* Flash Bank Configuration Register */

/* Register Addresses *******************************************************************************/

#define TMS570_FLASH_FRDCNTL              (TMS570_FWRAP_BASE+TMS570_FLASH_FRDCNTL_OFFSET)
#define TMS570_FLASH_FEDACTRL1            (TMS570_FWRAP_BASE+TMS570_FLASH_FEDACTRL1_OFFSET)
#define TMS570_FLASH_FEDACTRL2            (TMS570_FWRAP_BASE+TMS570_FLASH_FEDACTRL2_OFFSET)
#define TMS570_FLASH_FCORERRCNT           (TMS570_FWRAP_BASE+TMS570_FLASH_FCORERRCNT_OFFSET)
#define TMS570_FLASH_FCORERRADD           (TMS570_FWRAP_BASE+TMS570_FLASH_FCORERRADD_OFFSET)
#define TMS570_FLASH_FCORERRPOS           (TMS570_FWRAP_BASE+TMS570_FLASH_FCORERRPOS_OFFSET)
#define TMS570_FLASH_FEDACSTATUS          (TMS570_FWRAP_BASE+TMS570_FLASH_FEDACSTATUS_OFFSET)
#define TMS570_FLASH_FUNCERRADD           (TMS570_FWRAP_BASE+TMS570_FLASH_FUNCERRADD_OFFSET)
#define TMS570_FLASH_FEDACSDIS            (TMS570_FWRAP_BASE+TMS570_FLASH_FEDACSDIS_OFFSET)
#define TMS570_FLASH_FPRIMADDTAG          (TMS570_FWRAP_BASE+TMS570_FLASH_FPRIMADDTAG_OFFSET)
#define TMS570_FLASH_FDUPDDTAG            (TMS570_FWRAP_BASE+TMS570_FLASH_FDUPDDTAG_OFFSET)
#define TMS570_FLASH_FBPROT               (TMS570_FWRAP_BASE+TMS570_FLASH_FBPROT_OFFSET)
#define TMS570_FLASH_FBSE                 (TMS570_FWRAP_BASE+TMS570_FLASH_FBSE_OFFSET)
#define TMS570_FLASH_FBBUSY               (TMS570_FWRAP_BASE+TMS570_FLASH_FBBUSY_OFFSET)
#define TMS570_FLASH_FBAC                 (TMS570_FWRAP_BASE+TMS570_FLASH_FBAC_OFFSET)
#define TMS570_FLASH_FBFALLBACK           (TMS570_FWRAP_BASE+TMS570_FLASH_FBFALLBACK_OFFSET)
#define TMS570_FLASH_FBPRDY               (TMS570_FWRAP_BASE+TMS570_FLASH_FBPRDY_OFFSET)
#define TMS570_FLASH_FPAC1                (TMS570_FWRAP_BASE+TMS570_FLASH_FPAC1_OFFSET)
#define TMS570_FLASH_FPAC2                (TMS570_FWRAP_BASE+TMS570_FLASH_FPAC2_OFFSET)
#define TMS570_FLASH_FMAC                 (TMS570_FWRAP_BASE+TMS570_FLASH_FMAC_OFFSET)
#define TMS570_FLASH_FMSTAT               (TMS570_FWRAP_BASE+TMS570_FLASH_FMSTAT_OFFSET)
#define TMS570_FLASH_FEMUDMSW             (TMS570_FWRAP_BASE+TMS570_FLASH_FEMUDMSW_OFFSET)
#define TMS570_FLASH_FEMUDLSW             (TMS570_FWRAP_BASE+TMS570_FLASH_FEMUDLSW_OFFSET)
#define TMS570_FLASH_FEMUECC              (TMS570_FWRAP_BASE+TMS570_FLASH_FEMUECC_OFFSET)
#define TMS570_FLASH_FEMUADDR             (TMS570_FWRAP_BASE+TMS570_FLASH_FEMUADDR_OFFSET)
#define TMS570_FLASH_FDIAGCTRL            (TMS570_FWRAP_BASE+TMS570_FLASH_FDIAGCTRL_OFFSET)
#define TMS570_FLASH_FRAWDATAH            (TMS570_FWRAP_BASE+TMS570_FLASH_FRAWDATAH_OFFSET)
#define TMS570_FLASH_FRAWDATAL            (TMS570_FWRAP_BASE+TMS570_FLASH_FRAWDATAL_OFFSET)
#define TMS570_FLASH_FRAWECC              (TMS570_FWRAP_BASE+TMS570_FLASH_FRAWECC_OFFSET)
#define TMS570_FLASH_FPAROVR              (TMS570_FWRAP_BASE+TMS570_FLASH_FPAROVR_OFFSET)
#define TMS570_FLASH_FEDACSDIS2           (TMS570_FWRAP_BASE+TMS570_FLASH_FEDACSDIS2_OFFSET)
#define TMS570_FLASH_FSMWRENA             (TMS570_FWRAP_BASE+TMS570_FLASH_FSMWRENA_OFFSET)
#define TMS570_FLASH_FSMSECTOR            (TMS570_FWRAP_BASE+TMS570_FLASH_FSMSECTOR_OFFSET)
#define TMS570_FLASH_EEPROMCFG            (TMS570_FWRAP_BASE+TMS570_FLASH_EEPROMCFG_OFFSET)
#define TMS570_FLASH_EECTRL1              (TMS570_FWRAP_BASE+TMS570_FLASH_EECTRL1_OFFSET)
#define TMS570_FLASH_EECTRL2              (TMS570_FWRAP_BASE+TMS570_FLASH_EECTRL2_OFFSET)
#define TMS570_FLASH_EECORERRCNT          (TMS570_FWRAP_BASE+TMS570_FLASH_EECORERRCNT_OFFSET)
#define TMS570_FLASH_EECORERRADD          (TMS570_FWRAP_BASE+TMS570_FLASH_EECORERRADD_OFFSET)
#define TMS570_FLASH_EECORERRPOS          (TMS570_FWRAP_BASE+TMS570_FLASH_EECORERRPOS_OFFSET)
#define TMS570_FLASH_EESTATUS             (TMS570_FWRAP_BASE+TMS570_FLASH_EESTATUS_OFFSET)
#define TMS570_FLASH_EEUNCERRADD          (TMS570_FWRAP_BASE+TMS570_FLASH_EEUNCERRADD_OFFSET)
#define TMS570_FLASH_FCFGBANK             (TMS570_FWRAP_BASE+TMS570_FLASH_FCFGBANK_OFFSET)

/* Register Bit-Field Definitions *******************************************************************/

/* Flash Option Control Register */

#define FLASH_FRDCNTL_ENPIPE              (1 << 0)  /* Bit 0:  Enable Pipeline Mode */
#define FLASH_FRDCNTL_ASWSTEN             (1 << 4)  /* Bit 4:  Address Setup Wait State Enable */
#define FLASH_FRDCNTL_RWAIT_SHIFT         (8)       /* Bits 8-11:  Random/data Read Wait State */
#define FLASH_FRDCNTL_RWAIT_MASK          (15 << FLASH_FRDCNTL_RWAIT_SHIFT)
#  define FLASH_FRDCNTL_RWAIT(n)          ((uint32_t)(n) << FLASH_FRDCNTL_RWAIT_SHIFT)

/* Flash Error Detection and Correction Control Register 1 */
#define FLASH_FEDACTRL1_
/* Flash Error Detection and Correction Control Register 2 */
#define FLASH_FEDACTRL2_
/* Flash Correctable Error Count Register */
#define FLASH_FCORERRCNT_
/* Flash Correctable Error Address Register */
#define FLASH_FCORERRADD_
/* Flash Correctable Error Position Register */
#define FLASH_FCORERRPOS_
/* Flash Error Detection and Correction Status Register */
#define FLASH_FEDACSTATUS_
/* Flash Un-Correctable Error Address Register */
#define FLASH_FUNCERRADD_
/* Flash Error Detection and Correction Sector Disable Register */
#define FLASH_FEDACSDIS_
/* Flash Primary Address Tag Register */
#define FLASH_FPRIMADDTAG_
/* Flash Duplicate Address Tag Register */
#define FLASH_FDUPDDTAG_
/* Flash Bank Protection Register */
#define FLASH_FBPROT_
/* Flash Bank Sector Enable Register */
#define FLASH_FBSE_
/* Flash Bank Busy Register */
#define FLASH_FBBUSY_
/* Flash Bank Access Control Register */
#define FLASH_FBAC_

/* Flash Bank Fallback Power Register */

#define FLASH_FBFALLBACK_BANKPWR0_SHIFT   (0)       /* Bit 0:  Bank 0 Fallback Power Mode */
#define FLASH_FBFALLBACK_BANKPWR0_MASK    (3 << FLASH_FBFALLBACK_BANKPWR0_SHIFT)
#  define FLASH_FBFALLBACK_BANKPWR0_SLEEP (0 << FLASH_FBFALLBACK_BANKPWR0_SHIFT)
#  define FLASH_FBFALLBACK_BANKPWR0_STDBY (1 << FLASH_FBFALLBACK_BANKPWR0_SHIFT)
#  define FLASH_FBFALLBACK_BANKPWR0_ACTIV (3 << FLASH_FBFALLBACK_BANKPWR0_SHIFT)
#define FLASH_FBFALLBACK_BANKPWR1_SHIFT   (2)       /* Bit 2:  Bank 1 Fallback Power Mode */
#define FLASH_FBFALLBACK_BANKPWR1_MASK    (3 << FLASH_FBFALLBACK_BANKPWR1_SHIFT)
#  define FLASH_FBFALLBACK_BANKPWR1_SLEEP (0 << FLASH_FBFALLBACK_BANKPWR1_SHIFT)
#  define FLASH_FBFALLBACK_BANKPWR1_STDBY (1 << FLASH_FBFALLBACK_BANKPWR1_SHIFT)
#  define FLASH_FBFALLBACK_BANKPWR1_ACTIV (3 << FLASH_FBFALLBACK_BANKPWR1_SHIFT)
#define FLASH_FBFALLBACK_BANKPWR7_SHIFT   (14)      /* Bit 14: Bank 7 Fallback Power Mode */
#define FLASH_FBFALLBACK_BANKPWR7_MASK    (3 << FLASH_FBFALLBACK_BANKPWR7_SHIFT)
#  define FLASH_FBFALLBACK_BANKPWR7_SLEEP (0 << FLASH_FBFALLBACK_BANKPWR7_SHIFT)
#  define FLASH_FBFALLBACK_BANKPWR7_STDBY (1 << FLASH_FBFALLBACK_BANKPWR7_SHIFT)
#  define FLASH_FBFALLBACK_BANKPWR7_ACTIV (3 << FLASH_FBFALLBACK_BANKPWR7_SHIFT)

/* Flash Bank/Pump Ready Register */
#define FLASH_FBPRDY_
/* Flash Pump Access Control Register 1 */
#define FLASH_FPAC1_
/* Flash Pump Access Control Register 2 */
#define FLASH_FPAC2_
/* Flash Module Access Control Register */
#define FLASH_FMAC_
/* Flash Module Status Register */
#define FLASH_FMSTAT_
/* EEPROM Emulation Data MSW Register */
#define FLASH_FEMUDMSW_
/* EEPROM Emulation Data LSW Register */
#define FLASH_FEMUDLSW_
/* EEPROM Emulation ECC Register */
#define FLASH_FEMUECC_
/* EEPROM Emulation Address Register */
#define FLASH_FEMUADDR_
/* Diagnostic Control Register */
#define FLASH_FDIAGCTRL_
/* Uncorrected Raw Data High Register */
#define FLASH_FRAWDATAH_
/* Uncorrected Raw Data Low Register */
#define FLASH_FRAWDATAL_
/* Uncorrected Raw ECC Register */
#define FLASH_FRAWECC_
/* Parity Override Register */
#define FLASH_FPAROVR_
/* Flash Error Detection and Correction Sector Disable Register 2 */
#define FLASH_FEDACSDIS2_

/* FSM Register Write Enable */

#define FLASH_FSMWRENA_ENABLE_SHIFT     (0)       /* Bits 0-2: FSM Write Enable */
#define FLASH_FSMWRENA_ENABLE_MASK      (7 << FLASH_FSMWRENA_ENABLE_SHIFT)
#  define FLASH_FSMWRENA_ENABLE         (5 << FLASH_FSMWRENA_ENABLE_SHIFT) /* Enable write to FSM registers */
#  define FLASH_FSMWRENA_DISABLE        (2 << FLASH_FSMWRENA_ENABLE_SHIFT) /* Any other value disables */

/* FSM Sector Register */
#define FLASH_FSMSECTOR_

/* EEPROM Emulation Configuration Register */

#define FLASH_EEPROMCFG_GRACE_SHIFT     (0)       /* Bits 0-7: Auto-suspend Startup Grace Period */
#define FLASH_EEPROMCFG_GRACE_MASK      (0xff << FLASH_EEPROMCFG_GRACE_SHIFT)
#  define FLASH_EEPROMCFG_GRACE(n)      ((uint32_t)(n) << FLASH_EEPROMCFG_GRACE_SHIFT)
#define FLASH_EEPROMCFG_AUTOSUSPEN      (1 << 8)  /* Bit 8: Auto suspend enable */
#define FLASH_EEPROMCFG_EWAIT_SHIFT     (16)      /* Bits 16-19: EEPROM Wait state Counter */
#define FLASH_EEPROMCFG_EWAIT_MASK      (15 << FLASH_EEPROMCFG_EWAIT_SHIFT)
#  define FLASH_EEPROMCFG_EWAIT(n)      ((uint32_t)(n) << FLASH_EEPROMCFG_EWAIT_SHIFT)

/* EEPROM Emulation Error Detection and Correction Control Register 1 */
#define FLASH_EECTRL1_
/* EEPROM Emulation Error Detection and Correction Control Register 2 */
#define FLASH_EECTRL2_
/* EEPROM Emulation Correctable Error Count Register */
#define FLASH_EECORERRCNT_
/* EEPROM Emulation Correctable Error Address Register */
#define FLASH_EECORERRADD_
/* EEPROM Emulation Correctable Error Bit Position Register */
#define FLASH_EECORERRPOS_
/* EEPROM Emulation Error Status Register */
#define FLASH_EESTATUS_
/* EEPROM Emulation Un-Correctable Error Address Register */
#define FLASH_EEUNCERRADD_
/* Flash Bank Configuration Register */
#define FLASH_FCFGBANK_

#endif /* __ARCH_ARM_SRC_TMS570_CHIP_TMS570_FLASH_H */
