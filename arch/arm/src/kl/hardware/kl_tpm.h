/****************************************************************************
 * arch/arm/src/kl/hardware/kl_tpm.h
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

#ifndef __ARCH_ARM_SRC_KL_HARDWARE_KL_TPM_H
#define __ARCH_ARM_SRC_KL_HARDWARE_KL_TPM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "kl_config.h"

/****************************************************************************
 * Pre-processor Declarations
 ****************************************************************************/

#define TPM_SC_OFFSET       0x0000 /* Status and Control offset*/
#define TPM_CNT_OFFSET      0x0004 /* Counter offset */
#define TPM_MOD_OFFSET      0x0008 /* Modulo offset */
#define TPM_C0SC_OFFSET     0x000C /* Channel 0 Status and Control offset */
#define TPM_C0V_OFFSET      0x0010 /* Channel 0 Value offset */
#define TPM_C1SC_OFFSET     0x0014 /* Channel 1 Status and Control offset */
#define TPM_C1V_OFFSET      0x0018 /* Channel 1 Value offset */
#define TPM_C2SC_OFFSET     0x001C /* Channel 2 Status and Control offset */
#define TPM_C2V_OFFSET      0x0020 /* Channel 2 Value offset */
#define TPM_C3SC_OFFSET     0x0024 /* Channel 3 Status and Control offset */
#define TPM_C3V_OFFSET      0x0028 /* Channel 3 Value offset */
#define TPM_C4SC_OFFSET     0x002C /* Channel 4 Status and Control offset */
#define TPM_C4V_OFFSET      0x0030 /* Channel 4 Value offset */
#define TPM_C5SC_OFFSET     0x0034 /* Channel 5 Status and Control offset */
#define TPM_C5V_OFFSET      0x0038 /* Channel 5 Value offset */
#define TPM_STATUS_OFFSET   0x0050 /* Capture and Compare Status offset */
#define TPM_CONF_OFFSET     0x0084 /* Configuration offset */

#define TPM0_SC             (KL_TPM0_BASE + TPM_SC_OFFSET)   /* TPM0 Status and Control */
#define TPM0_CNT            (KL_TPM0_BASE + TPM_CNT_OFFSET)  /* TPM0 Counter */
#define TPM0_MOD            (KL_TPM0_BASE + TPM_MOD_OFFSET)  /* TPM0 Modulo */
#define TPM0_C0SC           (KL_TPM0_BASE + TPM_C0SC_OFFSET) /* TPM0 Channel 0 Status and Control */
#define TPM0_C0V            (KL_TPM0_BASE + TPM_C0V_OFFSET)  /* TPM0 Channel 0 Value */
#define TPM0_C1SC           (KL_TPM0_BASE + TPM_C1SC_OFFSET) /* TPM0 Channel 1 Status and Control */
#define TPM0_C1V            (KL_TPM0_BASE + TPM_C1V_OFFSET)  /* TPM0 Channel 1 Value */
#define TPM0_C2SC           (KL_TPM0_BASE + TPM_C2SC_OFFSET) /* TPM0 Channel 2 Status and Control */
#define TPM0_C2V            (KL_TPM0_BASE + TPM_C2V_OFFSET)  /* TPM0 Channel 2 Value */
#define TPM0_C3SC           (KL_TPM0_BASE + TPM_C3SC_OFFSET) /* TPM0 Channel 3 Status and Control */
#define TPM0_C3V            (KL_TPM0_BASE + TPM_C3V_OFFSET)  /* TPM0 Channel 3 Value */
#define TPM0_C4SC           (KL_TPM0_BASE + TPM_C4SC_OFFSET) /* TPM0 Channel 4 Status and Control */
#define TPM0_C4V            (KL_TPM0_BASE + TPM_C4V_OFFSET)  /* TPM0 Channel 4 Value */
#define TPM0_C5SC           (KL_TPM0_BASE + TPM_C5SC_OFFSET) /* TPM0 Channel 5 Status and Control */
#define TPM0_C5V            (KL_TPM0_BASE + TPM_C5V_OFFSET)  /* TPM0 Channel 5 Value */

#define TPM0_STATUS         (KL_TPM0_BASE + TPM_STATUS_OFFSET)  /* TPM0 Capture and Compare Status */

#define TPM0_CONF           (KL_TPM0_BASE + TPM_CONF_OFFSET) /* TPM0 Configuration */

#define TPM1_SC             (KL_TPM1_BASE + TPM_SC_OFFSET)   /* TPM1 Status and Control */
#define TPM1_CNT            (KL_TPM1_BASE + TPM_CNT_OFFSET)  /* TPM1 Counter */
#define TPM1_MOD            (KL_TPM1_BASE + TPM_MOD_OFFSET)  /* TPM1 Modulo */
#define TPM1_C0SC           (KL_TPM1_BASE + TPM_C0SC_OFFSET) /* TPM1 Channel 0 Status and Control */
#define TPM1_C0V            (KL_TPM1_BASE + TPM_C0V_OFFSET)  /* TPM1 Channel 0 Value */
#define TPM1_C1SC           (KL_TPM1_BASE + TPM_C1SC_OFFSET) /* TPM1 Channel 1 Status and Control */
#define TPM1_C1V            (KL_TPM1_BASE + TPM_C1V_OFFSET)  /* TPM1 Channel 1 Value */
#define TPM1_C2SC           (KL_TPM1_BASE + TPM_C2SC_OFFSET) /* TPM1 Channel 2 Status and Control */
#define TPM1_C2V            (KL_TPM1_BASE + TPM_C2V_OFFSET)  /* TPM1 Channel 2 Value */
#define TPM1_C3SC           (KL_TPM1_BASE + TPM_C3SC_OFFSET) /* TPM1 Channel 3 Status and Control */
#define TPM1_C3V            (KL_TPM1_BASE + TPM_C3V_OFFSET)  /* TPM1 Channel 3 Value */
#define TPM1_C4SC           (KL_TPM1_BASE + TPM_C4SC_OFFSET) /* TPM1 Channel 4 Status and Control */
#define TPM1_C4V            (KL_TPM1_BASE + TPM_C4V_OFFSET)  /* TPM1 Channel 4 Value */
#define TPM1_C5SC           (KL_TPM1_BASE + TPM_C5SC_OFFSET) /* TPM1 Channel 5 Status and Control */
#define TPM1_C5V            (KL_TPM1_BASE + TPM_C5V_OFFSET)  /* TPM1 Channel 5 Value */

#define TPM1_STATUS         (KL_TPM1_BASE + TPM_STATUS_OFFSET)  /* TPM1 Capture and Compare Status */

#define TPM1_CONF           (KL_TPM1_BASE + TPM_CONF_OFFSET) /* TPM1 Configuration */

#define TPM2_SC             (KL_TPM2_BASE + TPM_SC_OFFSET)   /* TPM2 Status and Control */
#define TPM2_CNT            (KL_TPM2_BASE + TPM_CNT_OFFSET)  /* TPM2 Counter */
#define TPM2_MOD            (KL_TPM2_BASE + TPM_MOD_OFFSET)  /* TPM2 Modulo */
#define TPM2_C0SC           (KL_TPM2_BASE + TPM_C0SC_OFFSET) /* TPM2 Channel 0 Status and Control */
#define TPM2_C0V            (KL_TPM2_BASE + TPM_C0V_OFFSET)  /* TPM2 Channel 0 Value */
#define TPM2_C1SC           (KL_TPM2_BASE + TPM_C1SC_OFFSET) /* TPM2 Channel 1 Status and Control */
#define TPM2_C1V            (KL_TPM2_BASE + TPM_C1V_OFFSET)  /* TPM2 Channel 1 Value */
#define TPM2_C2SC           (KL_TPM2_BASE + TPM_C2SC_OFFSET) /* TPM2 Channel 2 Status and Control */
#define TPM2_C2V            (KL_TPM2_BASE + TPM_C2V_OFFSET)  /* TPM2 Channel 2 Value */
#define TPM2_C3SC           (KL_TPM2_BASE + TPM_C3SC_OFFSET) /* TPM2 Channel 3 Status and Control */
#define TPM2_C3V            (KL_TPM2_BASE + TPM_C3V_OFFSET)  /* TPM2 Channel 3 Value */
#define TPM2_C4SC           (KL_TPM2_BASE + TPM_C4SC_OFFSET) /* TPM2 Channel 4 Status and Control */
#define TPM2_C4V            (KL_TPM2_BASE + TPM_C4V_OFFSET)  /* TPM2 Channel 4 Value */
#define TPM2_C5SC           (KL_TPM2_BASE + TPM_C5SC_OFFSET) /* TPM2 Channel 5 Status and Control */
#define TPM2_C5V            (KL_TPM2_BASE + TPM_C5V_OFFSET)  /* TPM2 Channel 5 Value */

#define TPM2_STATUS         (KL_TPM2_BASE + TPM_STATUS_OFFSET)  /* TPM2 Capture and Compare Status */

#define TPM2_CONF           (KL_TPM2_BASE + TPM_CONF_OFFSET) /* TPM2 Configuration */

#define TPM_SC_PS_SHIFT     0 /* Bits 0-2: Prescale Factor Selection */

#define TPM_SC_PS_MASK      (7 << TPM_SC_PS_SHIFT)
# define TPM_SC_PS_DIV1     (0 << TPM_SC_PS_SHIFT) /* Divide Clock by 1 */
# define TPM_SC_PS_DIV2     (1 << TPM_SC_PS_SHIFT) /* Divide Clock by 2 */
# define TPM_SC_PS_DIV4     (2 << TPM_SC_PS_SHIFT) /* Divide Clock by 4 */
# define TPM_SC_PS_DIV8     (3 << TPM_SC_PS_SHIFT) /* Divide Clock by 8 */
# define TPM_SC_PS_DIV16    (4 << TPM_SC_PS_SHIFT) /* Divide Clock by 16 */
# define TPM_SC_PS_DIV32    (5 << TPM_SC_PS_SHIFT) /* Divide Clock by 32 */
# define TPM_SC_PS_DIV64    (6 << TPM_SC_PS_SHIFT) /* Divide Clock by 64 */
# define TPM_SC_PS_DIV128   (7 << TPM_SC_PS_SHIFT) /* Divide Clock by 128 */

#define TPM_SC_CMOD_SHIFT   3 /* Bits 3-4: Clock Mode Selection */

#define TPM_SC_CMOD_MASK          (3 << TPM_SC_CMOD_SHIFT)
# define TPM_SC_CMOD_DIS          (0 << TPM_SC_CMOD_SHIFT) /* TPM counter is disabled */
# define TPM_SC_CMOD_LPTPM_CLK    (1 << TPM_SC_CMOD_SHIFT) /* TPM increments on every counter clock */
# define TPM_SC_CMOD_LPTPM_EXTCLK (2 << TPM_SC_CMOD_SHIFT) /* TPM increments on rising edge of EXTCLK */
# define TPM_SC_CMOD_RESERV       (3 << TPM_SC_CMOD_SHIFT) /* Reserved */

#define TPM_SC_CPWMS              (1 << 5) /* Bit 5: Center-aligned PWM Select */
#define TPM_SC_TOIE               (1 << 6) /* Bit 6: Timer Overflow Interrupt Enable */
#define TPM_SC_TOF                (1 << 7) /* Bit 7: Timer Overflow Flag*/
#define TPM_SC_DMA                (1 << 8) /* Bit 8: DMA Enable*/

#define TPM_CNSC_DMA              (1 << 0) /* Bit 0: Enables DMA transfers for the channel */
                                           /* Bit 1: Reserved */
#define TPM_CNSC_ELSA             (1 << 2) /* Bit 2: Edge or Level Select */
#define TPM_CNSC_ELSB             (1 << 3) /* Bit 3: Edge or Level Select */
#define TPM_CNSC_MSA              (1 << 4) /* Bit 4: Channel Mode Select */
#define TPM_CNSC_MSB              (1 << 5) /* Bit 5: Channel Mode Select */
#define TPM_CNSC_CHIE             (1 << 6) /* Bit 6: Channel Interrupt Enable */
#define TPM_CNSC_CHF              (1 << 7) /* Bit 7: Channel Flag */
                                           /* Bits 8-31: Reserved */

#define TPM_STATUS_CH0F           (1 << 0) /* Bit 0: Channel 0 Flag */
#define TPM_STATUS_CH1F           (1 << 1) /* Bit 1: Channel 1 Flag */
#define TPM_STATUS_CH2F           (1 << 2) /* Bit 2: Channel 2 Flag */
#define TPM_STATUS_CH3F           (1 << 3) /* Bit 3: Channel 3 Flag */
#define TPM_STATUS_CH4F           (1 << 4) /* Bit 4: Channel 4 Flag */
#define TPM_STATUS_CH5F           (1 << 5) /* Bit 5: Channel 5 Flag */
                                           /* Bits 6-7: Reserved */
#define TPM_STATUS_TOF            (1 << 8) /* Bit 8: Timer Overflow Flag */

#define TPM_CONF_DOZEEN           5 /* Bit 5: Doze Enable */
#define TPM_CONF_DBGMODE_SHIFT    6 /* Bits 6-7: Debug Mode */
#define TPM_CONF_DBGMODE_MASK     (3 << TPM_DBGMODE_SHIFT)
# define TPM_CONF_DBGMODE_PAUSE   (0 << TPM_DBGMODE_SHIFT) /* TPM counter will pause during DEBUG mode */
# define TPM_CONF_DBGMODE_CONT    (3 << TPM_DBGMODE_SHIFT) /* TPM counter continue working in DEBUG mode */

                                            /* Bit 8: Reserved */
#define TPM_CONF_GTBEEN           (1 << 9)  /* Bit 9: Global Time Base Enable */
                                            /* Bits 10-15: Reserved */
#define TPM_CONF_CSOT             (1 << 16) /* Bit 16: Counter Start On Trigger */
#define TPM_CONF_CSOO             (1 << 17) /* Bit 17: Counter Stop On Overflow */
#define TPM_CONF_CROT             (1 << 18) /* Bit 18: Counter Reload On Trigger */
                                            /* Bits 19-23: Reserved */
#define TPM_CONF_TRGSEL_SHIFT     24
#define TPM_CONF_TRGSEL_MASK      (15 << TPM_CONF_TRGSEL_SHIFT)
# define TPM_CONF_TRGSEL_EXTRG_IN (0 << TPM_CONF_TRGSEL_SHIFT) /* External trigger pin input */
# define TPM_CONF_TRGSEL_CMP0     (1 << TPM_CONF_TRGSEL_SHIFT) /* CPM0 output */

                                /* (2 << TPM_CONF_TRGSEL_SHIFT) Reserved */

                                /* (3 << TPM_CONF_TRGSEL_SHIFT) Reserved */

# define TPM_CONF_TRGSEL_PIT0     (4 << TPM_CONF_TRGSEL_SHIFT) /* PIT trigger 0 */
# define TPM_CONF_TRGSEL_PIT1     (5 << TPM_CONF_TRGSEL_SHIFT) /* PIT trigger 1 */

                                /* (6 << TPM_CONF_TRGSEL_SHIFT) Reserved */

                                /* (7 << TPM_CONF_TRGSEL_SHIFT) Reserved */

# define TPM_CONF_TRGSEL_TPM0     (8 << TPM_CONF_TRGSEL_SHIFT)  /* TPM0 Overflow */
# define TPM_CONF_TRGSEL_TPM1     (9 << TPM_CONF_TRGSEL_SHIFT)  /* TPM1 Overflow */
# define TPM_CONF_TRGSEL_TPM2     (10 << TPM_CONF_TRGSEL_SHIFT) /* TPM1 Overflow */

                                /* (11 << TPM_CONF_TRGSEL_SHIFT) Reserved */

# define TPM_CONF_TRGSEL_RTC_ALRM (12 << TPM_CONF_TRGSEL_SHIFT) /* RTC Alarm */
# define TPM_CONF_TRGSEL_RTC_SECS (13 << TPM_CONF_TRGSEL_SHIFT) /* RTC Seconds */
# define TPM_CONF_TRGSEL_LPTMR    (14 << TPM_CONF_TRGSEL_SHIFT) /* LPTMR trigger */

                                /* (15 << TPM_CONF_TRGSEL_SHIFT) Reserved */

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_KL_HARDWARE_KL_TPM_H */
