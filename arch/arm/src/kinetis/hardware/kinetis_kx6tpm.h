/****************************************************************************
 * arch/arm/src/kinetis/hardware/kinetis_kx6tpm.h
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

#ifndef __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_KX6TPM_H
#define __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_KX6TPM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Declarations
 ****************************************************************************/

#define KINETIS_TPM_SC_OFFSET       0x0000 /* Status and Control offset*/
#define KINETIS_TPM_CNT_OFFSET      0x0004 /* Counter offset */
#define KINETIS_TPM_MOD_OFFSET      0x0008 /* Modulo offset */
#define KINETIS_TPM_C0SC_OFFSET     0x000C /* Channel 0 Status and Control offset */
#define KINETIS_TPM_C0V_OFFSET      0x0010 /* Channel 0 Value offset */
#define KINETIS_TPM_C1SC_OFFSET     0x0014 /* Channel 1 Status and Control offset */
#define KINETIS_TPM_C1V_OFFSET      0x0018 /* Channel 1 Value offset */
#define KINETIS_TPM_STATUS_OFFSET   0x0050 /* Capture and Compare Status offset */
#define KINETIS_TPM_COMBINE_OFFSET  0x0064 /* Combine Channel Register offset */
#define KINETIS_TPM_POL_OFFSET      0x0070 /* Channel Polarity offset */
#define KINETIS_TPM_FILTER_OFFSET   0x0078 /* Filter Control offset */
#define KINETIS_TPM_QDCTRL_OFFSET   0x0080 /* Quadrature Decoder Control and Status offset */
#define KINETIS_TPM_CONF_OFFSET     0x0084 /* Configuration offset */

#define KINETIS_TPM1_SC             (KINETIS_TPM1_BASE + KINETIS_TPM_SC_OFFSET)      /* TPM1 Status and Control */
#define KINETIS_TPM1_CNT            (KINETIS_TPM1_BASE + KINETIS_TPM_CNT_OFFSET)     /* TPM1 Counter */
#define KINETIS_TPM1_MOD            (KINETIS_TPM1_BASE + KINETIS_TPM_MOD_OFFSET)     /* TPM1 Modulo */
#define KINETIS_TPM1_C0SC           (KINETIS_TPM1_BASE + KINETIS_TPM_C0SC_OFFSET)    /* TPM1 Channel 0 Status and Control */
#define KINETIS_TPM1_C0V            (KINETIS_TPM1_BASE + KINETIS_TPM_C0V_OFFSET)     /* TPM1 Channel 0 Value */
#define KINETIS_TPM1_C1SC           (KINETIS_TPM1_BASE + KINETIS_TPM_C1SC_OFFSET)    /* TPM1 Channel 1 Status and Control */
#define KINETIS_TPM1_C1V            (KINETIS_TPM1_BASE + KINETIS_TPM_C1V_OFFSET)     /* TPM1 Channel 1 Value */
#define KINETIS_TPM1_C2SC           (KINETIS_TPM1_BASE + KINETIS_TPM_C2SC_OFFSET)    /* TPM1 Channel 2 Status and Control */
#define KINETIS_TPM1_C2V            (KINETIS_TPM1_BASE + KINETIS_TPM_C2V_OFFSET)     /* TPM1 Channel 2 Value */
#define KINETIS_TPM1_STATUS         (KINETIS_TPM1_BASE + KINETIS_TPM_STATUS_OFFSET)  /* TPM1 Capture and Compare Status */
#define KINETIS_TPM1_COMBINE        (KINETIS_TPM1_BASE + KINETIS_TPM_COMBINE_OFFSET) /* TPM1 Combine Channel Register offset */
#define KINETIS_TPM1_POL            (KINETIS_TPM1_BASE + KINETIS_TPM_POL_OFFSET)     /* TPM1 Channel Polarity offset */
#define KINETIS_TPM1_FILTER         (KINETIS_TPM1_BASE + KINETIS_TPM_FILTER_OFFSET)  /* TPM1 Filter Control offset */
#define KINETIS_TPM1_QDCTRL         (KINETIS_TPM1_BASE + KINETIS_TPM_QDCTRL_OFFSET)  /* TPM1 Quadrature Decoder Control and Status offset */
#define KINETIS_TPM1_CONF           (KINETIS_TPM1_BASE + KINETIS_TPM_CONF_OFFSET)    /* TPM1 Configuration */

#define KINETIS_TPM2_SC             (KINETIS_TPM2_BASE + KINETIS_TPM_SC_OFFSET)      /* TPM2 Status and Control */
#define KINETIS_TPM2_CNT            (KINETIS_TPM2_BASE + KINETIS_TPM_CNT_OFFSET)     /* TPM2 Counter */
#define KINETIS_TPM2_MOD            (KINETIS_TPM2_BASE + KINETIS_TPM_MOD_OFFSET)     /* TPM2 Modulo */
#define KINETIS_TPM2_C0SC           (KINETIS_TPM2_BASE + KINETIS_TPM_C0SC_OFFSET)    /* TPM2 Channel 0 Status and Control */
#define KINETIS_TPM2_C0V            (KINETIS_TPM2_BASE + KINETIS_TPM_C0V_OFFSET)     /* TPM2 Channel 0 Value */
#define KINETIS_TPM2_C1SC           (KINETIS_TPM2_BASE + KINETIS_TPM_C1SC_OFFSET)    /* TPM2 Channel 1 Status and Control */
#define KINETIS_TPM2_C1V            (KINETIS_TPM2_BASE + KINETIS_TPM_C1V_OFFSET)     /* TPM2 Channel 1 Value */
#define KINETIS_TPM2_C2SC           (KINETIS_TPM2_BASE + KINETIS_TPM_C2SC_OFFSET)    /* TPM2 Channel 2 Status and Control */
#define KINETIS_TPM2_C2V            (KINETIS_TPM2_BASE + KINETIS_TPM_C2V_OFFSET)     /* TPM2 Channel 2 Value */
#define KINETIS_TPM2_STATUS         (KINETIS_TPM2_BASE + KINETIS_TPM_STATUS_OFFSET)  /* TPM2 Capture and Compare Status */
#define KINETIS_TPM2_COMBINE        (KINETIS_TPM2_BASE + KINETIS_TPM_COMBINE_OFFSET) /* TPM2 Combine Channel Register offset */
#define KINETIS_TPM2_POL            (KINETIS_TPM2_BASE + KINETIS_TPM_POL_OFFSET)     /* TPM2 Channel Polarity offset */
#define KINETIS_TPM2_FILTER         (KINETIS_TPM2_BASE + KINETIS_TPM_FILTER_OFFSET)  /* TPM2 Filter Control offset */
#define KINETIS_TPM2_QDCTRL         (KINETIS_TPM2_BASE + KINETIS_TPM_QDCTRL_OFFSET)  /* TPM2 Quadrature Decoder Control and Status offset */
#define KINETIS_TPM2_CONF           (KINETIS_TPM2_BASE + KINETIS_TPM_CONF_OFFSET)    /* TPM2 Configuration */

#define TPM_SC_PS_SHIFT             0 /* Bits 0-2: Prescale Factor Selection */
#define TPM_SC_PS_MASK              (7 << TPM_SC_PS_SHIFT)
#  define TPM_SC_PS_DIV1            (0 << TPM_SC_PS_SHIFT) /* Divide Clock by 1 */
#  define TPM_SC_PS_DIV2            (1 << TPM_SC_PS_SHIFT) /* Divide Clock by 2 */
#  define TPM_SC_PS_DIV4            (2 << TPM_SC_PS_SHIFT) /* Divide Clock by 4 */
#  define TPM_SC_PS_DIV8            (3 << TPM_SC_PS_SHIFT) /* Divide Clock by 8 */
#  define TPM_SC_PS_DIV16           (4 << TPM_SC_PS_SHIFT) /* Divide Clock by 16 */
#  define TPM_SC_PS_DIV32           (5 << TPM_SC_PS_SHIFT) /* Divide Clock by 32 */
#  define TPM_SC_PS_DIV64           (6 << TPM_SC_PS_SHIFT) /* Divide Clock by 64 */
#  define TPM_SC_PS_DIV128          (7 << TPM_SC_PS_SHIFT) /* Divide Clock by 128 */

#define TPM_SC_CMOD_SHIFT           3 /* Bits 3-4: Clock Mode Selection */
#define TPM_SC_CMOD_MASK            (3 << TPM_SC_CMOD_SHIFT)
#  define TPM_SC_CMOD_DIS           (0 << TPM_SC_CMOD_SHIFT) /* TPM counter is disabled */
#  define TPM_SC_CMOD_LPTPM_CLK     (1 << TPM_SC_CMOD_SHIFT) /* TPM increments on every counter clock */
#  define TPM_SC_CMOD_LPTPM_EXTCLK  (2 << TPM_SC_CMOD_SHIFT) /* TPM increments on rising edge of EXTCLK */

#define TPM_SC_CPWMS                (1 << 5) /* Bit 5: Center-aligned PWM Select */
#define TPM_SC_TOIE                 (1 << 6) /* Bit 6: Timer Overflow Interrupt Enable */
#define TPM_SC_TOF                  (1 << 7) /* Bit 7: Timer Overflow Flag*/
#define TPM_SC_DMA                  (1 << 8) /* Bit 8: DMA Enable*/
                                             /* Bits 9-31: Reserved */

#define TPM_CNT_SHIFT               0 /* Bits 0-15: Counter value */

#define TPM_CNT_MASK                (0xffff << TPM_COUNT_SHIFT) /* Any write clears Count */

                                             /* Bits 16-31: Reserved */

#define TPM_MOD_SHIFT               0 /* Bits 0-15: Mod value */

#define TPM_MOD_MASK                (0xffff << TPM_MOD_SHIFT) /* This field must be written with single 16 or 32-bit access */

                                             /* Bits 16-31: Reserved */

#define TPM_CnSC_DMA                (1 << 0) /* Bit 0: Enables DMA transfers for the channel */
                                             /* Bit 1: Reserved */
#define TPM_CnSC_ELSA               (1 << 2) /* Bit 2: Edge or Level Select */
#define TPM_CnSC_ELSB               (1 << 3) /* Bit 3: Edge or Level Select */
#define TPM_CnSC_MSA                (1 << 4) /* Bit 4: Channel Mode Select */
#define TPM_CnSC_MSB                (1 << 5) /* Bit 5: Channel Mode Select */
#define TPM_CnSC_CHIE               (1 << 6) /* Bit 6: Channel Interrupt Enable */
#define TPM_CnSC_CHF                (1 << 7) /* Bit 7: Channel Flag */
                                             /* Bits 8-31: Reserved */

#define TPM_VAL_SHIFT               0 /* Bits 0-15: Channel value */

#define TPM_VAL_MASK                (0xffff << TPM_VAL_SHIFT) /* Captured TPM counter value of the input modes or
                                                               * the match value for the output modes. This field
                                                               * must be written with single 16 or 32-bit access. */

                                             /* Bits 16-31: Reserved */

#define TPM_STATUS_CH0F             (1 << 0) /* Bit 0: Channel 0 Flag */
#define TPM_STATUS_CH1F             (1 << 1) /* Bit 1: Channel 1 Flag */
                                             /* Bits 2-7: Reserved */
#define TPM_STATUS_TOF              (1 << 8) /* Bit 8: Timer Overflow Flag */
                                             /* Bits 9-31: Reserved */

#define TPM_COMBINE_COMBINE0        (1 << 0) /* Bit 0: Combine Channels 0 and 1 */
#define TPM_COMBINE_COMSWAP0        (1 << 1) /* Bit 1: Combine Channel 0 and 1 Swap */
                                             /* Bits 2-7: Reserved */
                                             /* Bits 8-31: Reserved */

#define TPM_POL_POL0                (1 << 0) /* Bit 0: Channel 0 Polarity */
#define TPM_POL_POL1                (1 << 1) /* Bit 1: Channel 1 Polarity */
                                             /* Bits 2-31: Reserved */

#define TPM_FILTER_CH0FVAL_SHIFT    0 /* Bits 0-3: Channel 0 Filter Value */
#define TPM_FILTER_CH0FVAL_MASK     (0xf << TPM_FILTER_CH0FVAL_SHIFT)
#define TPM_FILTER_CH1FVAL_SHIFT    4 /* Bits 4-7: Channel 1 Filter Value */
#define TPM_FILTER_CH1FVAL_MASK     (0xf << TPM_FILTER_CH1FVAL_SHIFT)

#define TPM_QDCTRL_QDCTRL           (1 << 0) /* Bit 0: Enables the quadrature decoder mode */
#define TPM_QDCTRL_TOFDIR           (1 << 1) /* Bit 1: Indicates if the TOF bit was set (Read Only) */
#define TPM_QDCTRL_QUADIR           (1 << 2) /* Bit 2: Counter Direction in Quadrature Decode Mode (Read Only) */
#define TPM_QDCTRL_QUADMODE         (1 << 3) /* Bit 3: Quadrature Decoder Mode */
                                             /* Bits 4-31: Reserved */

#define TPM_CONF_DOZEEN             (1 << 5) /* Bit 5: Doze Enable */

#define TPM_CONF_DBGMODE_SHIFT      6 /* Bits 6-7: Debug Mode */
#define TPM_CONF_DBGMODE_MASK       (3 << TPM_CONF_DBGMODE_SHIFT)
#  define TPM_CONF_DBGMODE_PAUSE    (0 << TPM_CONF_DBGMODE_SHIFT) /* TPM counter will pause during DEBUG mode */
#  define TPM_CONF_DBGMODE_CONT     (3 << TPM_CONF_DBGMODE_SHIFT) /* TPM counter continue working in DEBUG mode */

#define TPM_CONF_GTBSYNC            (1 << 8)  /* Bit 8: Global Time Base Synchronization */
#define TPM_CONF_GTBEEN             (1 << 9)  /* Bit 9: Global Time Base Enable */
                                              /* Bits 10-15: Reserved */
#define TPM_CONF_CSOT               (1 << 16) /* Bit 16: Counter Start On Trigger */
#define TPM_CONF_CSOO               (1 << 17) /* Bit 17: Counter Stop On Overflow */
#define TPM_CONF_CROT               (1 << 18) /* Bit 18: Counter Reload On Trigger */
#define TPM_CONF_CPOT               (1 << 19) /* Bit 19: Counter Pause On Trigger */
                                              /* Bits 20-21: Reserved */
#define TPM_CONF_TRGPOL             (1 << 22) /* Bit 22: Trigger Polarity */
#define TPM_CONF_TRGSRC             (1 << 23) /* Bit 23: Trigger Source */

#define TPM_CONF_TRGSEL_SHIFT       24 /* Bits 24-27: Trigger Select */
#define TPM_CONF_TRGSEL_MASK        (0xf << TPM_CONF_TRGSEL_SHIFT)
                                                                  /* Internal TPM_CONF_TRGSRC set */
#  define TPM_CONF_TRGSEL_INTC0     (0 << TPM_CONF_TRGSEL_SHIFT)  /* Internal Channel 0 pin input capture */
#  define TPM_CONF_TRGSEL_INTC1     (2 << TPM_CONF_TRGSEL_SHIFT)  /* Internal Channel 1 pin input capture */
#  define TPM_CONF_TRGSEL_INTC01    (3 << TPM_CONF_TRGSEL_SHIFT)  /* Internal Channel 0 or 1 pin input capture */

#  define TPM_CONF_TRGSEL_EXTRG_IN  (0 << TPM_CONF_TRGSEL_SHIFT)  /* External trigger pin input */
#  define TPM_CONF_TRGSEL_CMP0      (1 << TPM_CONF_TRGSEL_SHIFT)  /* CPM0 output */
#  define TPM_CONF_TRGSEL_CMP1      (2 << TPM_CONF_TRGSEL_SHIFT)  /* CPM1 output */
#  define TPM_CONF_TRGSEL_CMP2      (3 << TPM_CONF_TRGSEL_SHIFT)  /* CPM2 output */
#  define TPM_CONF_TRGSEL_PIT0      (4 << TPM_CONF_TRGSEL_SHIFT)  /* PIT trigger 0 */
#  define TPM_CONF_TRGSEL_PIT1      (5 << TPM_CONF_TRGSEL_SHIFT)  /* PIT trigger 1 */
#  define TPM_CONF_TRGSEL_PIT2      (6 << TPM_CONF_TRGSEL_SHIFT)  /* PIT trigger 2 */
#  define TPM_CONF_TRGSEL_PIT3      (7 << TPM_CONF_TRGSEL_SHIFT)  /* PIT trigger 3 */
#  define TPM_CONF_TRGSEL_FTM0      (8 << TPM_CONF_TRGSEL_SHIFT)  /* FTM0 initialization trigger and channel triggers */
#  define TPM_CONF_TRGSEL_FTM1      (9 << TPM_CONF_TRGSEL_SHIFT)  /* FTM1 initialization trigger and channel triggers */
#  define TPM_CONF_TRGSEL_FTM2      (10 << TPM_CONF_TRGSEL_SHIFT) /* FTM2 initialization trigger and channel triggers */
#  define TPM_CONF_TRGSEL_FTM3      (11 << TPM_CONF_TRGSEL_SHIFT) /* FTM3 initialization trigger and channel triggers */
#  define TPM_CONF_TRGSEL_RTC_ALRM  (12 << TPM_CONF_TRGSEL_SHIFT) /* RTC Alarm */
#  define TPM_CONF_TRGSEL_RTC_SECS  (13 << TPM_CONF_TRGSEL_SHIFT) /* RTC Seconds */
#  define TPM_CONF_TRGSEL_LPTMR     (14 << TPM_CONF_TRGSEL_SHIFT) /* LPTMR trigger */
#  define TPM_CONF_TRGSEL_SW        (15 << TPM_CONF_TRGSEL_SHIFT) /* Software Trigger */

#endif /* __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_KX6TPM_H */
