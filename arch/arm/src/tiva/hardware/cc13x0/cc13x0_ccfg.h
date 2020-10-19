/************************************************************************************************************
 * arch/arm/src/tiva/hardware/cc13x0/cc13x0_ccfg.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *
 * Technical content derives from a TI header file that has a compatible
 * BSD license:
 *
 *   Copyright (c) 2015-2017, Texas Instruments Incorporated
 *   All rights reserved.
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
 ************************************************************************************************************/

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X0_CC13X0_CCFG_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X0_CC13X0_CCFG_H

/************************************************************************************************************
 * Included Files
 ************************************************************************************************************/

#include <nuttx/config.h>
#include "hardware/tiva_memorymap.h"

/************************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************************/

/* CCFG Register Offsets ************************************************************************************/

#define TIVA_CCFG_EXT_LF_CLK_OFFSET             0x0fa8  /* Extern LF clock configuration */
#define TIVA_CCFG_MODE_CONF_1_OFFSET            0x0fac  /* Mode Configuration 1 */
#define TIVA_CCFG_SIZE_AND_DIS_FLAGS_OFFSET     0x0fb0  /* CCFG Size and Disable Flags */
#define TIVA_CCFG_MODE_CONF_OFFSET              0x0fb4  /* Mode Configuration 0 */
#define TIVA_CCFG_VOLT_LOAD_0_OFFSET            0x0fb8  /* Voltage Load 0 */
#define TIVA_CCFG_VOLT_LOAD_1_OFFSET            0x0fbc  /* Voltage Load 1 */
#define TIVA_CCFG_RTC_OFFSET_OFFSET             0x0fc0  /* Real Time Clock Offset */
#define TIVA_CCFG_FREQ_OFFSET_OFFSET            0x0fc4  /* Frequency Offset */
#define TIVA_CCFG_IEEE_MAC_0_OFFSET             0x0fc8  /* IEEE MAC Address 0 */
#define TIVA_CCFG_IEEE_MAC_1_OFFSET             0x0fcc  /* IEEE MAC Address 1 */
#define TIVA_CCFG_IEEE_BLE_0_OFFSET             0x0fd0  /* IEEE BLE Address 0 */
#define TIVA_CCFG_IEEE_BLE_1_OFFSET             0x0fd4  /* IEEE BLE Address 1 */
#define TIVA_CCFG_BL_CONFIG_OFFSET              0x0fd8  /* Bootloader Configuration */
#define TIVA_CCFG_ERASE_CONF_OFFSET             0x0fdc  /* Erase Configuration */
#define TIVA_CCFG_CCFG_TI_OPTIONS_OFFSET        0x0fe0  /* TI Options */
#define TIVA_CCFG_CCFG_TAP_DAP_0_OFFSET         0x0fe4  /* Test Access Points Enable 0 */
#define TIVA_CCFG_CCFG_TAP_DAP_1_OFFSET         0x0fe8  /* Test Access Points Enable 1 */
#define TIVA_CCFG_IMAGE_VALID_CONF_OFFSET       0x0fec  /* Image Valid */
#define TIVA_CCFG_CCFG_PROT_31_0_OFFSET         0x0ff0  /* Protect Sectors 0-31 */
#define TIVA_CCFG_CCFG_PROT_63_32_OFFSET        0x0ff4  /* Protect Sectors 32-63 */
#define TIVA_CCFG_CCFG_PROT_95_64_OFFSET        0x0ff8  /* Protect Sectors 64-95 */
#define TIVA_CCFG_CCFG_PROT_127_96_OFFSET       0x0ffc  /* Protect Sectors 96-127 */

/* CCFG Register Addresses **********************************************************************************/

#define TIVA_CCFG_EXT_LF_CLK                    (TIVA_CCFG_BASE + TIVA_CCFG_EXT_LF_CLK_OFFSET)
#define TIVA_CCFG_MODE_CONF_1                   (TIVA_CCFG_BASE + TIVA_CCFG_MODE_CONF_1_OFFSET)
#define TIVA_CCFG_SIZE_AND_DIS_FLAGS            (TIVA_CCFG_BASE + TIVA_CCFG_SIZE_AND_DIS_FLAGS_OFFSET)
#define TIVA_CCFG_MODE_CONF                     (TIVA_CCFG_BASE + TIVA_CCFG_MODE_CONF_OFFSET)
#define TIVA_CCFG_VOLT_LOAD_0                   (TIVA_CCFG_BASE + TIVA_CCFG_VOLT_LOAD_0_OFFSET)
#define TIVA_CCFG_VOLT_LOAD_1                   (TIVA_CCFG_BASE + TIVA_CCFG_VOLT_LOAD_1_OFFSET)
#define TIVA_CCFG_RTC_OFFSET                    (TIVA_CCFG_BASE + TIVA_CCFG_RTC_OFFSET_OFFSET)
#define TIVA_CCFG_FREQ_OFFSET                   (TIVA_CCFG_BASE + TIVA_CCFG_FREQ_OFFSET_OFFSET)
#define TIVA_CCFG_IEEE_MAC_0                    (TIVA_CCFG_BASE + TIVA_CCFG_IEEE_MAC_0_OFFSET)
#define TIVA_CCFG_IEEE_MAC_1                    (TIVA_CCFG_BASE + TIVA_CCFG_IEEE_MAC_1_OFFSET)
#define TIVA_CCFG_IEEE_BLE_0                    (TIVA_CCFG_BASE + TIVA_CCFG_IEEE_BLE_0_OFFSET)
#define TIVA_CCFG_IEEE_BLE_1                    (TIVA_CCFG_BASE + TIVA_CCFG_IEEE_BLE_1_OFFSET)
#define TIVA_CCFG_BL_CONFIG                     (TIVA_CCFG_BASE + TIVA_CCFG_BL_CONFIG_OFFSET)
#define TIVA_CCFG_ERASE_CONF                    (TIVA_CCFG_BASE + TIVA_CCFG_ERASE_CONF_OFFSET)
#define TIVA_CCFG_CCFG_TI_OPTIONS               (TIVA_CCFG_BASE + TIVA_CCFG_CCFG_TI_OPTIONS_OFFSET)
#define TIVA_CCFG_CCFG_TAP_DAP_0                (TIVA_CCFG_BASE + TIVA_CCFG_CCFG_TAP_DAP_0_OFFSET)
#define TIVA_CCFG_CCFG_TAP_DAP_1                (TIVA_CCFG_BASE + TIVA_CCFG_CCFG_TAP_DAP_1_OFFSET)
#define TIVA_CCFG_IMAGE_VALID_CONF              (TIVA_CCFG_BASE + TIVA_CCFG_IMAGE_VALID_CONF_OFFSET)
#define TIVA_CCFG_CCFG_PROT_31_0                (TIVA_CCFG_BASE + TIVA_CCFG_CCFG_PROT_31_0_OFFSET)
#define TIVA_CCFG_CCFG_PROT_63_32               (TIVA_CCFG_BASE + TIVA_CCFG_CCFG_PROT_63_32_OFFSET)
#define TIVA_CCFG_CCFG_PROT_95_64               (TIVA_CCFG_BASE + TIVA_CCFG_CCFG_PROT_95_64_OFFSET)
#define TIVA_CCFG_CCFG_PROT_127_96              (TIVA_CCFG_BASE + TIVA_CCFG_CCFG_PROT_127_96_OFFSET)

/* CCFG Bifield Definitions *********************************************************************************/

/* TIVA_CCFG_EXT_LF_CLK */

#define CCFG_EXT_LF_CLK_RTC_INCREMENT_SHIFT     (0)       /* Bits 0-23: Input frequency of the external clock */
#define CCFG_EXT_LF_CLK_RTC_INCREMENT_MASK      (0xffffff << CCFG_EXT_LF_CLK_RTC_INCREMENT_SHIFT)
#define CCFG_EXT_LF_CLK_DIO_SHIFT               (24)      /* Bits 24-31: DIO to supply external 32kHz clock */
#define CCFG_EXT_LF_CLK_DIO_MASK                (0xff << CCFG_EXT_LF_CLK_DIO_SHIFT)

/* TIVA_CCFG_MODE_CONF_1 */

#define CCFG_MODE_CONF_1_XOSC_MAX_START_SHIFT   (0)       /* Bits 0-7: Maximum XOSC startup time (100us units). */
#define CCFG_MODE_CONF_1_XOSC_MAX_START_MASK    (0xff << CCFG_MODE_CONF_1_XOSC_MAX_START_SHIFT)
#define CCFG_MODE_CONF_1_DELTA_IBIAS_OFFSET_SHIFT  (8)    /* Bits 8-11: Signed delta value for IBIAS_OFFSET */
#define CCFG_MODE_CONF_1_DELTA_IBIAS_OFFSET_MASK   (15 << CCFG_MODE_CONF_1_DELTA_IBIAS_OFFSET_SHIFT)
#define CCFG_MODE_CONF_1_DELTA_IBIAS_OFFSET_MASK   (15 << CCFG_MODE_CONF_1_DELTA_IBIAS_OFFSET_SHIFT)
#  define CCFG_MODE_CONF_1_DELTA_IBIAS_OFFSET(n)   ((uint32_t)(n) << CCFG_MODE_CONF_1_DELTA_IBIAS_OFFSET_SHIFT)
#define CCFG_MODE_CONF_1_DELTA_IBIAS_INIT_SHIFT (12)      /* Bits 12-15: Signed delta value for IBIAS_INIT */
#define CCFG_MODE_CONF_1_DELTA_IBIAS_INIT_MASK  (15 << CCFG_MODE_CONF_1_DELTA_IBIAS_INIT_SHIFT)
#define CCFG_MODE_CONF_1_ALT_DCDC_IPEAK_SHIFT   (16)      /* Bits 16-18: Inductor peak current */
                                                          /* Peak current = 31 + (4 * ALT_DCDC_IPEAK) */
#define CCFG_MODE_CONF_1_ALT_DCDC_IPEAK_MASK    (7 << CCFG_MODE_CONF_1_ALT_DCDC_IPEAK_SHIFT)
#  define CCFG_MODE_CONF_1_ALT_DCDC_IPEAK(n)    ((uint32_t)(n) << CCFG_MODE_CONF_1_ALT_DCDC_IPEAK_SHIFT)
#define CCFG_MODE_CONF_1_ALT_DCDC_DITHER_EN     (1 << 19) /* Bit 19: Enable DC/DC dithering */
#define CCFG_MODE_CONF_1_ALT_DCDC_VMIN_SHIFT    (20)      /* Bits 20-23: Minimum voltage for when DC/DC should be used */
                                                          /* Voltage = (28 + ALT_DCDC_VMIN) / 16 */
#define CCFG_MODE_CONF_1_ALT_DCDC_VMIN_MASK     (15 << CCFG_MODE_CONF_1_ALT_DCDC_VMIN_SHIFT)
#  define CCFG_MODE_CONF_1_ALT_DCDC_VMIN(n)     ((uint32_t)(n) << CCFG_MODE_CONF_1_ALT_DCDC_VMIN_SHIFT)

/* TIVA_CCFG_SIZE_AND_DIS_FLAGS */

#define CCFG_SIZE_AND_DIS_FLAGS_DIS_XOSC_OVR         (1 << 0)  /* Bit 0: Disable XOSC override */
#define CCFG_SIZE_AND_DIS_FLAGS_DIS_ALT_DCDC_SETTING (1 << 1)  /* Bit 1: Disable alternate DC/DC settings */
#define CCFG_SIZE_AND_DIS_FLAGS_DIS_GPRAM            (1 << 2)  /* Bit 2:  Disable GPRAM */
#define CCFG_SIZE_AND_DIS_FLAGS_DIS_TCXO             (1 << 3)  /* Bit 3: Disable TCXO */
#define CCFG_SIZE_AND_DIS_FLAGS_DISABLE_FLAGS_SHIFT  (4)       /* Bits 4-15: Reserved for future use */
#define CCFG_SIZE_AND_DIS_FLAGS_DISABLE_FLAGS_MASK   (0xfff << CCFG_SIZE_AND_DIS_FLAGS_DISABLE_FLAGS_SHIFT)
#define CCFG_SIZE_AND_DIS_FLAGS_SIZE_OF_CCFG_SHIFT   (16)      /* Bits 16-31: otal size of CCFG in bytes */
#define CCFG_SIZE_AND_DIS_FLAGS_SIZE_OF_CCFG_MASK    (0xffff << CCFG_SIZE_AND_DIS_FLAGS_SIZE_OF_CCFG_SHIFT)

/* TIVA_CCFG_MODE_CONF */

#define CCFG_MODE_CONF_VDDR_CAP_SHIFT                (0)       /* Bits 0-7: Minimum decoupling capacitance
                                                                * on VDDR (100nF units) */
#define CCFG_MODE_CONF_VDDR_CAP_MASK                 (0xff << CCFG_MODE_CONF_VDDR_CAP_SHIFT)
#  define CCFG_MODE_CONF_VDDR_CAP(n)                 ((uint32_t)(n) << CCFG_MODE_CONF_VDDR_CAP_SHIFT)
#define CCFG_MODE_CONF_XOSC_CAPARRAY_DELTA_SHIFT     (8)       /* Bits 8-15: Trimmed XOSC cap-array step value modifier */
#define CCFG_MODE_CONF_XOSC_CAPARRAY_DELTA_MASK      (0xff << CCFG_MODE_CONF_XOSC_CAPARRAY_DELTA_SHIFT)
#define CCFG_MODE_CONF_XOSC_CAPARRAY_DELTA_WIDTH     (8)       /* (For sign extension) */
#  define CCFG_MODE_CONF_XOSC_CAPARRAY_DELTA(n)      ((uint32_t)(n) << CCFG_MODE_CONF_XOSC_CAPARRAY_DELTA_SHIFT)
#define CCFG_MODE_CONF_HF_COMP                       (1 << 16) /* Bit 16: Reserved for future use */
#define CCFG_MODE_CONF_XOSC_CAP_MOD                  (1 << 17) /* Bit 17: Enable modification to XOSC cap-array */
#define CCFG_MODE_CONF_XOSC_FREQ_SHIFT               (18)      /* Bits 18-19: Reserved for future use */
#define CCFG_MODE_CONF_XOSC_FREQ_MASK                (3 << CCFG_MODE_CONF_XOSC_FREQ_SHIFT)
#  define CCFG_MODE_CONF_XOSC_FREQ_HPOSC             (1 << CCFG_MODE_CONF_XOSC_FREQ_SHIFT)
#  define CCFG_MODE_CONF_XOSC_FREQ_48M               (2 << CCFG_MODE_CONF_XOSC_FREQ_SHIFT)
#  define CCFG_MODE_CONF_XOSC_FREQ_24M               (3 << CCFG_MODE_CONF_XOSC_FREQ_SHIFT)
#define CCFG_MODE_CONF_VDDR_TRIM_SLEEP_TC            (1 << 21) /* Bit 21: VDDR trim sleep temp compensation */
#  define CCFG_MODE_CONF_VDDR_TRIM_SLEEP_TC_NONE     (0)
#  define CCFG_MODE_CONF_VDDR_TRIM_SLEEP_TC_ENABLE   CCFG_MODE_CONF_VDDR_TRIM_SLEEP_TC
#define CCFG_MODE_CONF_SCLK_LF_OPTION_SHIFT          (22)      /* Bits 22-23: Select source for SCLK_LF */
#define CCFG_MODE_CONF_SCLK_LF_OPTION_MASK           (3 << CCFG_MODE_CONF_SCLK_LF_OPTION_SHIFT)
#  define CCFG_MODE_CONF_SCLK_LF_OPTION_XOSC_HF_DLF  (0 << CCFG_MODE_CONF_SCLK_LF_OPTION_SHIFT
#  define CCFG_MODE_CONF_SCLK_LF_OPTION_EXTERNAL_LF  (1 << CCFG_MODE_CONF_SCLK_LF_OPTION_SHIFT
#  define CCFG_MODE_CONF_SCLK_LF_OPTION_XOSC_LF      (2 << CCFG_MODE_CONF_SCLK_LF_OPTION_SHIFT
#  define CCFG_MODE_CONF_SCLK_LF_OPTION_RCOSC_LF     (3 << CCFG_MODE_CONF_SCLK_LF_OPTION_SHIFT
#define CCFG_MODE_CONF_VDDS_BOD_LEVEL                (1 << 24) /* Bit nn: VDDS BOD level */
#  define CCFG_MODE_CONF_VDDS_BOD_2p0V               (0)
#  define CCFG_MODE_CONF_VDDS_BOD_1p8V               CCFG_MODE_CONF_VDDS_BOD_LEVEL
#define CCFG_MODE_CONF_VDDR_EXT_LOAD                 (1 << 25) /* Bit 25: Reserved for future use */
#define CCFG_MODE_CONF_DCDC_ACTIVE                   (1 << 26) /* Bit 26: DC/DC in active mode */
#define CCFG_MODE_CONF_DCDC_RECHARGE                 (1 << 27) /* Bit27nn: DC/DC during recharge in powerdown */
#define CCFG_MODE_CONF_VDDR_TRIM_SLEEP_DELTA_SHIFT   (28)      /* Bits 28-31: Signed delta value to apply to the
                                                                * VDDR_TRIM_SLEEP target, minus one */
#define CCFG_MODE_CONF_VDDR_TRIM_SLEEP_DELTA_MASK    (15 << CCFG_MODE_CONF_VDDR_TRIM_SLEEP_DELTA_SHIFT)
#define CCFG_MODE_CONF_VDDR_TRIM_SLEEP_DELTA_WIDTH   4         /* (Supports sign extension) */
#  define CCFG_MODE_CONF_VDDR_TRIM_SLEEP_DELTA(n)    (((uint8_t)((n) - 1) & 15) << CCFG_MODE_CONF_VDDR_TRIM_SLEEP_DELTA_SHIFT)

/* TIVA_CCFG_VOLT_LOAD_0 */

#define CCFG_VOLT_LOAD_0_VDDR_EXT_TM15_SHIFT    (0)       /* Bits 0-7: Reserved for future use */
#define CCFG_VOLT_LOAD_0_VDDR_EXT_TM15_MASK     (0xff << CCFG_VOLT_LOAD_0_VDDR_EXT_TM15_SHIFT)
#define CCFG_VOLT_LOAD_0_VDDR_EXT_TP5_SHIFT     (8)       /* Bits 8-15: Reserved for future use */
#define CCFG_VOLT_LOAD_0_VDDR_EXT_TP5_MASK      (0xff << CCFG_VOLT_LOAD_0_VDDR_EXT_TP5_SHIFT)
#define CCFG_VOLT_LOAD_0_VDDR_EXT_TP25_SHIFT    (16)      /* Bits 16-23: Reserved for future use */
#define CCFG_VOLT_LOAD_0_VDDR_EXT_TP25_MASK     (0xff << CCFG_VOLT_LOAD_0_VDDR_EXT_TP25_SHIFT)
#define CCFG_VOLT_LOAD_0_VDDR_EXT_TP45_SHIFT    (24)      /* Bits 24-31: Reserved for future use */
#define CCFG_VOLT_LOAD_0_VDDR_EXT_TP45_MASK     (0xff << CCFG_VOLT_LOAD_0_VDDR_EXT_TP45_SHIFT)

/* TIVA_CCFG_VOLT_LOAD_1 */

#define CCFG_VOLT_LOAD_1_VDDR_EXT_TP65_SHIFT    (0)       /* Bits 0-7: Reserved for future use */
#define CCFG_VOLT_LOAD_1_VDDR_EXT_TP65_MASK     (0xff << CCFG_VOLT_LOAD_1_VDDR_EXT_TP65_SHIFT)
#define CCFG_VOLT_LOAD_1_VDDR_EXT_TP85_SHIFT    (8)       /* Bits 8-15: Reserved for future use */
#define CCFG_VOLT_LOAD_1_VDDR_EXT_TP85_MASK     (0xff << CCFG_VOLT_LOAD_1_VDDR_EXT_TP85_SHIFT)
#define CCFG_VOLT_LOAD_1_VDDR_EXT_TP105_SHIFT   (16)      /* Bits 16-23: Reserved for future use */
#define CCFG_VOLT_LOAD_1_VDDR_EXT_TP105_MASK    (0xff << CCFG_VOLT_LOAD_1_VDDR_EXT_TP105_SHIFT)
#define CCFG_VOLT_LOAD_1_VDDR_EXT_TP125_SHIFT   (24)      /* Bits 24-31: Reserved for future use */
#define CCFG_VOLT_LOAD_1_VDDR_EXT_TP125_MASK    (0xff << CCFG_VOLT_LOAD_1_VDDR_EXT_TP125_SHIFT)

/* TIVA_CCFG_RTC_OFFSET */

#define CCFG_RTC_OFFSET_RTC_COMP_P0_SHIFT       (16)      /* Bits 16-31: Reserved for future use */
#define CCFG_RTC_OFFSET_RTC_COMP_P0_MASK        (0xffff << CCFG_RTC_OFFSET_RTC_COMP_P0_SHIFT)
#define CCFG_RTC_OFFSET_RTC_COMP_P1_SHIFT       (8)       /* Bits 8-15: Reserved for future use */
#define CCFG_RTC_OFFSET_RTC_COMP_P1_MASK        (0xff << CCFG_RTC_OFFSET_RTC_COMP_P1_SHIFT)
#define CCFG_RTC_OFFSET_RTC_COMP_P2_SHIFT       (0)       /* Bits 0-7: Reserved for future use */
#define CCFG_RTC_OFFSET_RTC_COMP_P2_MASK        (0xff << CCFG_RTC_OFFSET_RTC_COMP_P2_SHIFT)

/* TIVA_CCFG_FREQ_OFFSET */

#define CCFG_FREQ_OFFSET_HF_COMP_P2_SHIFT       (0)       /* Bits 0-7: Reserved for future use */
#define CCFG_FREQ_OFFSET_HF_COMP_P2_MASK        (0xff << CCFG_FREQ_OFFSET_HF_COMP_P2_SHIFT)
#define CCFG_FREQ_OFFSET_HF_COMP_P1_SHIFT       (8)       /* Bits 8-15: Reserved for future use */
#define CCFG_FREQ_OFFSET_HF_COMP_P1_MASK        (0xff << CCFG_FREQ_OFFSET_HF_COMP_P1_SHIFT)
#define CCFG_FREQ_OFFSET_HF_COMP_P0_SHIFT       (16)      /* Bits 16-31: Reserved for future use */
#define CCFG_FREQ_OFFSET_HF_COMP_P0_MASK        (0xffff << CCFG_FREQ_OFFSET_HF_COMP_P0_SHIFT)

/* TIVA_CCFG_IEEE_MAC_0 (32-bit value) */

/* TIVA_CCFG_IEEE_MAC_1 (32-bit value) */

/* TIVA_CCFG_IEEE_BLE_0 (32-bit value) */

/* TIVA_CCFG_IEEE_BLE_1 (32-bit value) */

/* TIVA_CCFG_BL_CONFIG */

#define CCFG_BL_CONFIG_BL_ENABLE_SHIFT          (0)       /* Bits 0-7: Enables the boot loader backdoor */
#define CCFG_BL_CONFIG_BL_ENABLE_MASK           (0xff << CCFG_BL_CONFIG_BL_ENABLE_SHIFT)
#  define CCFG_BL_CONFIG_BL_ENABLE              (0xc5 << CCFG_BL_CONFIG_BL_ENABLE_SHIFT)
#define CCFG_BL_CONFIG_BL_PIN_NUMBER_SHIFT      (8)       /* Bits 8-15:  DIO number that is level checked
                                                           * if the boot loader backdoor is enabled */
#define CCFG_BL_CONFIG_BL_PIN_NUMBER_MASK       (0xff << CCFG_BL_CONFIG_BL_PIN_NUMBER_SHIFT)
#define CCFG_BL_CONFIG_BL_LEVEL                 (1 << 16) /* Bit 16: Active level of the selected DIO */
#  define CCFG_BL_CONFIG_BL_LOW                 (0)
#  define CCFG_BL_CONFIG_BL_HIGH                CCFG_BL_CONFIG_BL_LEVEL
#define CCFG_BL_CONFIG_BOOTLOADER_ENABLE_SHIFT  (24)      /* Bits 24-31: Bootloader enable */
#define CCFG_BL_CONFIG_BOOTLOADER_ENABLE_MASK   (0xff << CCFG_BL_CONFIG_BOOTLOADER_ENABLE_SHIFT)
#  define CCFG_BL_CONFIG_BOOTLOADER_ENABLE      (0xc5 << CCFG_BL_CONFIG_BOOTLOADER_ENABLE_SHIFT)

/* TIVA_CCFG_O_ERASE_CONF */

#define CCFG_ERASE_CONF_BANK_ERASE_DIS_N        (1 << 0)  /* Bit 0: Bank erase */
#define CCFG_ERASE_CONF_CHIP_ERASE_DIS_N        (1 << 8)  /* Bit 8: Chip erase */

/* TIVA_CCFG_CCFG_TI_OPTIONS */

#define CCFG_CCFG_TI_OPTIONS_TI_FA_ENABLE_SHIFT (0)       /* Bits 0-7: Enable TI Failure Analysis */
#define CCFG_CCFG_TI_OPTIONS_TI_FA_ENABLE_MASK  (0xff << CCFG_CCFG_TI_OPTIONS_TI_FA_ENABLE_SHIFT)
#  define CCFG_CCFG_TI_OPTIONS_TI_FA_ENABLE     (0xc5 << CCFG_CCFG_TI_OPTIONS_TI_FA_ENABLE_SHIFT)

/* TIVA_CCFG_CCFG_TAP_DAP_0 */

#define CCFG_CCFG_TAP_DAP_0_TEST_TAP_ENABLE_SHIFT    (0)       /* Bits 0-7: Enable Test TAP */
#define CCFG_CCFG_TAP_DAP_0_TEST_TAP_ENABLE_MASK     (0xff << CCFG_CCFG_TAP_DAP_0_TEST_TAP_ENABLE_SHIFT)
#  define CCFG_CCFG_TAP_DAP_0_TEST_TAP_ENABLE        (0xc5 << CCFG_CCFG_TAP_DAP_0_TEST_TAP_ENABLE_SHIFT)
#define CCFG_CCFG_TAP_DAP_0_PRCM_TAP_ENABLE_SHIFT    (8)       /* Bits 8-15: Enable PRCM TAP */
#define CCFG_CCFG_TAP_DAP_0_PRCM_TAP_ENABLE_MASK     (0xff << CCFG_CCFG_TAP_DAP_0_PWRPROF_TAP_ENABLE_SHIFT)
#  define CCFG_CCFG_TAP_DAP_0_PWRPROF_TAP_ENABLE     (0xc5 << CCFG_CCFG_TAP_DAP_0_PWRPROF_TAP_ENABLE_SHIFT)
#define CCFG_CCFG_TAP_DAP_0_CPU_DAP_ENABLE_SHIFT     (16)       /* Bits 16-23: Enable CPU DAP */
#define CCFG_CCFG_TAP_DAP_0_CPU_DAP_ENABLE_MASK      (0xff << CCFG_CCFG_TAP_DAP_0_CPU_DAP_ENABLE_SHIFT)
#  define CCFG_CCFG_TAP_DAP_0_CPU_DAP_ENABLE         (0xc5 << CCFG_CCFG_TAP_DAP_0_CPU_DAP_ENABLE_SHIFT)

/* TIVA_CCFG_CCFG_TAP_DAP_1 */

#define CCFG_CCFG_TAP_DAP_1_PBIST2_TAP_ENABLE_SHIFT  (16)       /* Bits 16-23: Enable PBIST2 TAP */
#define CCFG_CCFG_TAP_DAP_1_PBIST2_TAP_ENABLE_MASK   (0xff << CCFG_CCFG_TAP_DAP_1_PBIST2_TAP_ENABLE_SHIFT)
#  define CCFG_CCFG_TAP_DAP_1_PBIST2_TAP_ENABLE      (0xc5 << CCFG_CCFG_TAP_DAP_1_PBIST2_TAP_ENABLE_SHIFT)
#define CCFG_CCFG_TAP_DAP_1_PBIST1_TAP_ENABLE_SHIFT  (8)       /* Bits 8-15: Enable PBIST1 TAP */
#define CCFG_CCFG_TAP_DAP_1_PBIST1_TAP_ENABLE_MASK   (0xff << CCFG_CCFG_TAP_DAP_1_PBIST1_TAP_ENABLE_SHIFT)
#  define CCFG_CCFG_TAP_DAP_1_PBIST1_TAP_ENABLE      (0xc5 << CCFG_CCFG_TAP_DAP_1_PBIST1_TAP_ENABLE_SHIFT)
#define CCFG_CCFG_TAP_DAP_1_WUC_TAP_ENABLE_SHIFT     (0)       /* Bits 0-7: Enable WUC TAP */
#define CCFG_CCFG_TAP_DAP_1_WUC_TAP_ENABLE_MASK      (0xff << CCFG_CCFG_TAP_DAP_1_AON_TAP_ENABLE_SHIFT)
#  define CCFG_CCFG_TAP_DAP_1_WUC_TAP_ENABLE         (0xc5 << CCFG_CCFG_TAP_DAP_1_AON_TAP_ENABLE_SHIFT)

/* TIVA_CCFG_IMAGE_VALID_CONF (32-bit value) */

/* TIVA_CCFG_CCFG_PROT_31_0 */

#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC(n)      (1 << (n))
#  define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_0     (1 << 0)
#  define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_1     (1 << 1)
#  define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_2     (1 << 2)
#  define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_3     (1 << 3)
#  define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_4     (1 << 4)
#  define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_5     (1 << 5)
#  define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_6     (1 << 6)
#  define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_7     (1 << 7)
#  define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_8     (1 << 8)
#  define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_9     (1 << 9)
#  define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_10    (1 << 10)
#  define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_11    (1 << 11)
#  define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_12    (1 << 12)
#  define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_13    (1 << 13)
#  define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_14    (1 << 14)
#  define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_15    (1 << 15)
#  define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_16    (1 << 16)
#  define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_17    (1 << 17)
#  define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_18    (1 << 18)
#  define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_19    (1 << 19)
#  define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_20    (1 << 20)
#  define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_21    (1 << 21)
#  define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_22    (1 << 22)
#  define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_23    (1 << 23)
#  define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_24    (1 << 24)
#  define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_25    (1 << 25)
#  define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_26    (1 << 26)
#  define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_27    (1 << 27)
#  define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_28    (1 << 28)
#  define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_29    (1 << 29)
#  define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_30    (1 << 30)
#  define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_31    (1 << 31)

/* TIVA_CCFG_CCFG_PROT_63_32 */

#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC(n)     (1 << ((n) - 32))
#  define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_32   (1 << 0)
#  define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_33   (1 << 1)
#  define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_34   (1 << 2)
#  define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_35   (1 << 3)
#  define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_36   (1 << 4)
#  define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_37   (1 << 5)
#  define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_38   (1 << 6)
#  define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_39   (1 << 7)
#  define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_40   (1 << 8)
#  define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_41   (1 << 9)
#  define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_42   (1 << 10)
#  define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_43   (1 << 11)
#  define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_44   (1 << 12)
#  define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_45   (1 << 13)
#  define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_46   (1 << 14)
#  define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_47   (1 << 15)
#  define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_48   (1 << 16)
#  define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_49   (1 << 17)
#  define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_50   (1 << 18)
#  define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_51   (1 << 19)
#  define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_52   (1 << 20)
#  define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_53   (1 << 21)
#  define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_54   (1 << 22)
#  define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_55   (1 << 23)
#  define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_56   (1 << 24)
#  define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_57   (1 << 25)
#  define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_58   (1 << 26)
#  define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_59   (1 << 27)
#  define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_60   (1 << 28)
#  define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_61   (1 << 29)
#  define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_62   (1 << 30)
#  define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_63   (1 << 31)

/* TIVA_CCFG_CCFG_PROT_95_64 */

#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC(n)     (1 << ((n) - 64))
#  define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_64   (1 << 0)
#  define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_65   (1 << 1)
#  define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_66   (1 << 2)
#  define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_67   (1 << 3)
#  define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_68   (1 << 4)
#  define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_69   (1 << 5)
#  define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_70   (1 << 6)
#  define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_71   (1 << 7)
#  define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_72   (1 << 8)
#  define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_73   (1 << 9)
#  define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_74   (1 << 10)
#  define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_75   (1 << 11)
#  define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_76   (1 << 12)
#  define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_77   (1 << 13)
#  define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_78   (1 << 14)
#  define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_79   (1 << 15)
#  define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_80   (1 << 16)
#  define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_81   (1 << 17)
#  define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_82   (1 << 18)
#  define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_83   (1 << 19)
#  define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_84   (1 << 20)
#  define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_85   (1 << 21)
#  define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_86   (1 << 22)
#  define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_87   (1 << 23)
#  define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_88   (1 << 24)
#  define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_89   (1 << 25)
#  define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_90   (1 << 26)
#  define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_91   (1 << 27)
#  define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_92   (1 << 28)
#  define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_93   (1 << 29)
#  define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_94   (1 << 30)
#  define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_95   (1 << 31)

/* TIVA_CCFG_CCFG_PROT_127_96 */

#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC(n)    (1 << ((n) - 96))
#  define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_96  (1 << 0)
#  define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_97  (1 << 1)
#  define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_98  (1 << 2)
#  define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_99  (1 << 3)
#  define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_100 (1 << 4)
#  define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_101 (1 << 5)
#  define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_102 (1 << 6)
#  define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_103 (1 << 7)
#  define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_104 (1 << 8)
#  define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_105 (1 << 9)
#  define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_106 (1 << 10)
#  define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_107 (1 << 11)
#  define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_108 (1 << 12)
#  define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_109 (1 << 13)
#  define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_110 (1 << 14)
#  define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_111 (1 << 15)
#  define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_112 (1 << 16)
#  define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_113 (1 << 17)
#  define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_114 (1 << 18)
#  define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_115 (1 << 19)
#  define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_116 (1 << 20)
#  define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_117 (1 << 21)
#  define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_118 (1 << 22)
#  define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_119 (1 << 23)
#  define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_120 (1 << 24)
#  define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_121 (1 << 25)
#  define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_122 (1 << 26)
#  define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_123 (1 << 27)
#  define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_124 (1 << 28)
#  define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_125 (1 << 29)
#  define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_126 (1 << 30)
#  define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_127 (1 << 31)

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X0_CC13X0_CCFG_H */
