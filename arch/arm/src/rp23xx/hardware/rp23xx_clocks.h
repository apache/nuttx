/****************************************************************************
 * arch/arm/src/rp23xx/hardware/rp23xx_clocks.h
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

#ifndef __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_CLOCKS_H
#define __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_CLOCKS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp23xx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clock index **************************************************************/

#define RP23XX_CLOCKS_NDX_GPOUT0    0   /* Clock output to GPIO 21 */
#define RP23XX_CLOCKS_NDX_GPOUT1    1   /* Clock output to GPIO 23 */
#define RP23XX_CLOCKS_NDX_GPOUT2    2   /* Clock output to GPIO 24 */
#define RP23XX_CLOCKS_NDX_GPOUT3    3   /* Clock output to GPIO 25 */
#define RP23XX_CLOCKS_NDX_REF       4   /* Reference clock */
#define RP23XX_CLOCKS_NDX_SYS       5   /* System clock */
#define RP23XX_CLOCKS_NDX_PERI      6   /* Peripheral clock */
#define RP23XX_CLOCKS_NDX_HSTX      7   /* HSTX clock */
#define RP23XX_CLOCKS_NDX_USB       8   /* USB clock */
#define RP23XX_CLOCKS_NDX_ADC       9   /* ADC clock */
#define RP23XX_CLOCKS_NDX_MAX       10

/* Register offsets *********************************************************/
#define RP23XX_CLOCKS_CLK_CTRL_OFFSET              0x000000  /* Clock control */
#define RP23XX_CLOCKS_CLK_DIV_OFFSET               0x000004  /* Clock divisor */
#define RP23XX_CLOCKS_CLK_SELECTED_OFFSET          0x000008  /* Indicates which src is currently selected */

#define RP23XX_CLOCKS_CLK_NDX_CTRL_OFFSET(n)        ((n) * 12 + RP23XX_CLOCKS_CLK_CTRL_OFFSET)
#define RP23XX_CLOCKS_CLK_NDX_DIV_OFFSET(n)         ((n) * 12 + RP23XX_CLOCKS_CLK_DIV_OFFSET)
#define RP23XX_CLOCKS_CLK_NDX_SELECTED_OFFSET(n)    ((n) * 12 + RP23XX_CLOCKS_CLK_SELECTED_OFFSET)

#define RP23XX_CLOCKS_DFTCLK_XOSC_CTRL_OFFSET      0x000078
#define RP23XX_CLOCKS_DFTCLK_ROSC_CTRL_OFFSET      0x00007c
#define RP23XX_CLOCKS_DFTCLK_LPOSC_CTRL_OFFSET     0x000080
#define RP23XX_CLOCKS_CLK_SYS_RESUS_CTRL_OFFSET    0x000084
#define RP23XX_CLOCKS_CLK_SYS_RESUS_STATUS_OFFSET  0x000088
#define RP23XX_CLOCKS_FC0_REF_KHZ_OFFSET           0x00008c  /* Reference clock frequency in kHz */
#define RP23XX_CLOCKS_FC0_MIN_KHZ_OFFSET           0x000090  /* Minimum pass frequency in kHz. This is optional. Set to 0 if you are not using the pass/fail flags */
#define RP23XX_CLOCKS_FC0_MAX_KHZ_OFFSET           0x000094  /* Maximum pass frequency in kHz. This is optional. Set to 0x1ffffff if you are not using the pass/fail flags */
#define RP23XX_CLOCKS_FC0_DELAY_OFFSET             0x000098  /* Delays the start of frequency counting to allow the mux to settle Delay is measured in multiples of the reference clock period */
#define RP23XX_CLOCKS_FC0_INTERVAL_OFFSET          0x00009c  /* The test interval is 0.98us * 2**interval, but let's call it 1us * 2**interval The default gives a test interval of 250us */
#define RP23XX_CLOCKS_FC0_SRC_OFFSET               0x0000a0  /* Clock sent to frequency counter, set to 0 when not required Writing to this register initiates the frequency count */
#define RP23XX_CLOCKS_FC0_STATUS_OFFSET            0x0000a4  /* Frequency counter status */
#define RP23XX_CLOCKS_FC0_RESULT_OFFSET            0x0000a8  /* Result of frequency measurement, only valid when status_done=1 */
#define RP23XX_CLOCKS_WAKE_EN0_OFFSET              0x0000ac  /* enable clock in wake mode */
#define RP23XX_CLOCKS_WAKE_EN1_OFFSET              0x0000b0  /* enable clock in wake mode */
#define RP23XX_CLOCKS_SLEEP_EN0_OFFSET             0x0000b4  /* enable clock in sleep mode */
#define RP23XX_CLOCKS_SLEEP_EN1_OFFSET             0x0000b8  /* enable clock in sleep mode */
#define RP23XX_CLOCKS_ENABLED0_OFFSET              0x0000bc  /* indicates the state of the clock enable */
#define RP23XX_CLOCKS_ENABLED1_OFFSET              0x0000c0  /* indicates the state of the clock enable */
#define RP23XX_CLOCKS_INTR_OFFSET                  0x0000c4  /* Raw Interrupts */
#define RP23XX_CLOCKS_INTE_OFFSET                  0x0000c8  /* Interrupt Enable */
#define RP23XX_CLOCKS_INTF_OFFSET                  0x0000cc  /* Interrupt Force */
#define RP23XX_CLOCKS_INTS_OFFSET                  0x0000d0  /* Interrupt status after masking & forcing */

/* Register definitions *****************************************************/
#define RP23XX_CLOCKS_CLK_NDX_CTRL(n)       (RP23XX_CLOCKS_BASE + RP23XX_CLOCKS_CLK_NDX_CTRL_OFFSET(n))
#define RP23XX_CLOCKS_CLK_NDX_DIV(n)        (RP23XX_CLOCKS_BASE + RP23XX_CLOCKS_CLK_NDX_DIV_OFFSET(n))
#define RP23XX_CLOCKS_CLK_NDX_SELECTED(n)   (RP23XX_CLOCKS_BASE + RP23XX_CLOCKS_CLK_NDX_SELECTED_OFFSET(n))

#define RP23XX_CLOCKS_CLK_GPOUT0_CTRL       (RP23XX_CLOCKS_CLK_NDX_CTRL(RP23XX_CLOCKS_NDX_GPOUT0))
#define RP23XX_CLOCKS_CLK_GPOUT0_DIV        (RP23XX_CLOCKS_CLK_NDX_DIV(RP23XX_CLOCKS_NDX_GPOUT0))
#define RP23XX_CLOCKS_CLK_GPOUT0_SELECTED   (RP23XX_CLOCKS_CLK_NDX_SELECTED(RP23XX_CLOCKS_NDX_GPOUT0))
#define RP23XX_CLOCKS_CLK_GPOUT1_CTRL       (RP23XX_CLOCKS_CLK_NDX_CTRL(RP23XX_CLOCKS_NDX_GPOUT1))
#define RP23XX_CLOCKS_CLK_GPOUT1_DIV        (RP23XX_CLOCKS_CLK_NDX_DIV(RP23XX_CLOCKS_NDX_GPOUT1))
#define RP23XX_CLOCKS_CLK_GPOUT1_SELECTED   (RP23XX_CLOCKS_CLK_NDX_SELECTED(RP23XX_CLOCKS_NDX_GPOUT1))
#define RP23XX_CLOCKS_CLK_GPOUT2_CTRL       (RP23XX_CLOCKS_CLK_NDX_CTRL(RP23XX_CLOCKS_NDX_GPOUT2))
#define RP23XX_CLOCKS_CLK_GPOUT2_DIV        (RP23XX_CLOCKS_CLK_NDX_DIV(RP23XX_CLOCKS_NDX_GPOUT2))
#define RP23XX_CLOCKS_CLK_GPOUT2_SELECTED   (RP23XX_CLOCKS_CLK_NDX_SELECTED(RP23XX_CLOCKS_NDX_GPOUT2))
#define RP23XX_CLOCKS_CLK_GPOUT3_CTRL       (RP23XX_CLOCKS_CLK_NDX_CTRL(RP23XX_CLOCKS_NDX_GPOUT3))
#define RP23XX_CLOCKS_CLK_GPOUT3_DIV        (RP23XX_CLOCKS_CLK_NDX_DIV(RP23XX_CLOCKS_NDX_GPOUT3))
#define RP23XX_CLOCKS_CLK_GPOUT3_SELECTED   (RP23XX_CLOCKS_CLK_NDX_SELECTED(RP23XX_CLOCKS_NDX_GPOUT3))
#define RP23XX_CLOCKS_CLK_REF_CTRL          (RP23XX_CLOCKS_CLK_NDX_CTRL(RP23XX_CLOCKS_NDX_REF))
#define RP23XX_CLOCKS_CLK_REF_DIV           (RP23XX_CLOCKS_CLK_NDX_DIV(RP23XX_CLOCKS_NDX_REF))
#define RP23XX_CLOCKS_CLK_REF_SELECTED      (RP23XX_CLOCKS_CLK_NDX_SELECTED(RP23XX_CLOCKS_NDX_REF))
#define RP23XX_CLOCKS_CLK_SYS_CTRL          (RP23XX_CLOCKS_CLK_NDX_CTRL(RP23XX_CLOCKS_NDX_SYS))
#define RP23XX_CLOCKS_CLK_SYS_DIV           (RP23XX_CLOCKS_CLK_NDX_DIV(RP23XX_CLOCKS_NDX_SYS))
#define RP23XX_CLOCKS_CLK_SYS_SELECTED      (RP23XX_CLOCKS_CLK_NDX_SELECTED(RP23XX_CLOCKS_NDX_SYS))
#define RP23XX_CLOCKS_CLK_PERI_CTRL         (RP23XX_CLOCKS_CLK_NDX_CTRL(RP23XX_CLOCKS_NDX_PERI))
#define RP23XX_CLOCKS_CLK_PERI_SELECTED     (RP23XX_CLOCKS_CLK_NDX_SELECTED(RP23XX_CLOCKS_NDX_PERI))
#define RP23XX_CLOCKS_CLK_HSTX_CTRL         (RP23XX_CLOCKS_CLK_NDX_CTRL(RP23XX_CLOCKS_NDX_HSTX))
#define RP23XX_CLOCKS_CLK_HSTX_DIV          (RP23XX_CLOCKS_CLK_NDX_DIV(RP23XX_CLOCKS_NDX_HSTX))
#define RP23XX_CLOCKS_CLK_HSTX_SELECTED     (RP23XX_CLOCKS_CLK_NDX_SELECTED(RP23XX_CLOCKS_NDX_HSTX))
#define RP23XX_CLOCKS_CLK_USB_CTRL          (RP23XX_CLOCKS_CLK_NDX_CTRL(RP23XX_CLOCKS_NDX_USB))
#define RP23XX_CLOCKS_CLK_USB_DIV           (RP23XX_CLOCKS_CLK_NDX_DIV(RP23XX_CLOCKS_NDX_USB))
#define RP23XX_CLOCKS_CLK_USB_SELECTED      (RP23XX_CLOCKS_CLK_NDX_SELECTED(RP23XX_CLOCKS_NDX_USB))
#define RP23XX_CLOCKS_CLK_ADC_CTRL          (RP23XX_CLOCKS_CLK_NDX_CTRL(RP23XX_CLOCKS_NDX_ADC))
#define RP23XX_CLOCKS_CLK_ADC_DIV           (RP23XX_CLOCKS_CLK_NDX_DIV(RP23XX_CLOCKS_NDX_ADC))
#define RP23XX_CLOCKS_CLK_ADC_SELECTED      (RP23XX_CLOCKS_CLK_NDX_SELECTED(RP23XX_CLOCKS_NDX_ADC))

#define RP23XX_CLOCKS_DFTCLK_XOSC_CTRL      (RP23XX_CLOCKS_BASE + RP23XX_CLOCKS_DFTCLK_XOSC_CTRL_OFFSET)
#define RP23XX_CLOCKS_DFTCLK_ROSC_CTRL      (RP23XX_CLOCKS_BASE + RP23XX_CLOCKS_DFTCLK_ROSC_CTRL_OFFSET)
#define RP23XX_CLOCKS_DFTCLK_LPOSC_CTRL     (RP23XX_CLOCKS_BASE + RP23XX_CLOCKS_DFTCLK_LPOSC_CTRL_OFFSET)
#define RP23XX_CLOCKS_CLK_SYS_RESUS_CTRL    (RP23XX_CLOCKS_BASE + RP23XX_CLOCKS_CLK_SYS_RESUS_CTRL_OFFSET)
#define RP23XX_CLOCKS_CLK_SYS_RESUS_STATUS  (RP23XX_CLOCKS_BASE + RP23XX_CLOCKS_CLK_SYS_RESUS_STATUS_OFFSET)
#define RP23XX_CLOCKS_FC0_REF_KHZ           (RP23XX_CLOCKS_BASE + RP23XX_CLOCKS_FC0_REF_KHZ_OFFSET)
#define RP23XX_CLOCKS_FC0_MIN_KHZ           (RP23XX_CLOCKS_BASE + RP23XX_CLOCKS_FC0_MIN_KHZ_OFFSET)
#define RP23XX_CLOCKS_FC0_MAX_KHZ           (RP23XX_CLOCKS_BASE + RP23XX_CLOCKS_FC0_MAX_KHZ_OFFSET)
#define RP23XX_CLOCKS_FC0_DELAY             (RP23XX_CLOCKS_BASE + RP23XX_CLOCKS_FC0_DELAY_OFFSET)
#define RP23XX_CLOCKS_FC0_INTERVAL          (RP23XX_CLOCKS_BASE + RP23XX_CLOCKS_FC0_INTERVAL_OFFSET)
#define RP23XX_CLOCKS_FC0_SRC               (RP23XX_CLOCKS_BASE + RP23XX_CLOCKS_FC0_SRC_OFFSET)
#define RP23XX_CLOCKS_FC0_STATUS            (RP23XX_CLOCKS_BASE + RP23XX_CLOCKS_FC0_STATUS_OFFSET)
#define RP23XX_CLOCKS_FC0_RESULT            (RP23XX_CLOCKS_BASE + RP23XX_CLOCKS_FC0_RESULT_OFFSET)
#define RP23XX_CLOCKS_WAKE_EN0              (RP23XX_CLOCKS_BASE + RP23XX_CLOCKS_WAKE_EN0_OFFSET)
#define RP23XX_CLOCKS_WAKE_EN1              (RP23XX_CLOCKS_BASE + RP23XX_CLOCKS_WAKE_EN1_OFFSET)
#define RP23XX_CLOCKS_SLEEP_EN0             (RP23XX_CLOCKS_BASE + RP23XX_CLOCKS_SLEEP_EN0_OFFSET)
#define RP23XX_CLOCKS_SLEEP_EN1             (RP23XX_CLOCKS_BASE + RP23XX_CLOCKS_SLEEP_EN1_OFFSET)
#define RP23XX_CLOCKS_ENABLED0              (RP23XX_CLOCKS_BASE + RP23XX_CLOCKS_ENABLED0_OFFSET)
#define RP23XX_CLOCKS_ENABLED1              (RP23XX_CLOCKS_BASE + RP23XX_CLOCKS_ENABLED1_OFFSET)
#define RP23XX_CLOCKS_INTR                  (RP23XX_CLOCKS_BASE + RP23XX_CLOCKS_INTR_OFFSET)
#define RP23XX_CLOCKS_INTE                  (RP23XX_CLOCKS_BASE + RP23XX_CLOCKS_INTE_OFFSET)
#define RP23XX_CLOCKS_INTF                  (RP23XX_CLOCKS_BASE + RP23XX_CLOCKS_INTF_OFFSET)
#define RP23XX_CLOCKS_INTS                  (RP23XX_CLOCKS_BASE + RP23XX_CLOCKS_INTS_OFFSET)

/* Register bit definitions *************************************************/

#define RP23XX_CLOCKS_CLK_GPOUT0_CTRL_NUDGE                                   (1 << 20)  /* An edge on this signal shifts the phase of the output by 1 cycle of the input clock This can be done at any time */
#define RP23XX_CLOCKS_CLK_GPOUT0_CTRL_PHASE_SHIFT                             (16)       /* This delays the enable signal by up to 3 cycles of the input clock This must be set before the clock is enabled to have any effect */
#define RP23XX_CLOCKS_CLK_GPOUT0_CTRL_PHASE_MASK                              (0x03 << RP23XX_CLOCKS_CLK_GPOUT0_CTRL_PHASE_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT0_CTRL_DC50                                    (1 << 12)  /* Enables duty cycle correction for odd divisors */
#define RP23XX_CLOCKS_CLK_GPOUT0_CTRL_ENABLE                                  (1 << 11)  /* Starts and stops the clock generator cleanly */
#define RP23XX_CLOCKS_CLK_GPOUT0_CTRL_KILL                                    (1 << 10)  /* Asynchronously kills the clock generator */
#define RP23XX_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_SHIFT                            (5)        /* Selects the auxiliary clock source, will glitch when switching */
#define RP23XX_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_MASK                             (0x0f << RP23XX_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_CLKSRC_PLL_SYS                   (0x0 << RP23XX_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_CLKSRC_GPIN0                     (0x1 << RP23XX_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_CLKSRC_GPIN1                     (0x2 << RP23XX_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_CLKSRC_PLL_USB                   (0x3 << RP23XX_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_CLKSRC_PLL_USB_PRIMARY_REF_OPCG  (0x4 << RP23XX_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_ROSC_CLKSRC                      (0x5 << RP23XX_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_XOSC_CLKSRC                      (0x6 << RP23XX_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_LPOSC_CLKSRC                     (0x7 << RP23XX_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_CLK_SYS                          (0x8 << RP23XX_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_CLK_USB                          (0x9 << RP23XX_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_CLK_ADC                          (0xa << RP23XX_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_CLK_REF                          (0xb << RP23XX_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_CLK_PERI                         (0xc << RP23XX_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_CLK_HSTX                         (0xd << RP23XX_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_CLK_CLK2FC                       (0xe << RP23XX_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_SHIFT)

#define RP23XX_CLOCKS_CLK_GPOUT0_DIV_INT_SHIFT                                (16)       /* Integer component of the divisor, 0 -> divide by 2^16 */
#define RP23XX_CLOCKS_CLK_GPOUT0_DIV_INT_MASK                                 (0xffff << RP23XX_CLOCKS_CLK_GPOUT0_DIV_INT_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT0_DIV_FRAC_MASK                                (0xffff)   /* Fractional component of the divisor */

#define RP23XX_CLOCKS_CLK_GPOUT1_CTRL_NUDGE                                   (1 << 20)  /* An edge on this signal shifts the phase of the output by 1 cycle of the input clock This can be done at any time */
#define RP23XX_CLOCKS_CLK_GPOUT1_CTRL_PHASE_SHIFT                             (16)       /* This delays the enable signal by up to 3 cycles of the input clock This must be set before the clock is enabled to have any effect */
#define RP23XX_CLOCKS_CLK_GPOUT1_CTRL_PHASE_MASK                              (0x03 << RP23XX_CLOCKS_CLK_GPOUT1_CTRL_PHASE_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT1_CTRL_DC50                                    (1 << 12)  /* Enables duty cycle correction for odd divisors */
#define RP23XX_CLOCKS_CLK_GPOUT1_CTRL_ENABLE                                  (1 << 11)  /* Starts and stops the clock generator cleanly */
#define RP23XX_CLOCKS_CLK_GPOUT1_CTRL_KILL                                    (1 << 10)  /* Asynchronously kills the clock generator */
#define RP23XX_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_SHIFT                            (5)        /* Selects the auxiliary clock source, will glitch when switching */
#define RP23XX_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_MASK                             (0x0f << RP23XX_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_CLKSRC_PLL_SYS                   (0x0 << RP23XX_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_CLKSRC_GPIN0                     (0x1 << RP23XX_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_CLKSRC_GPIN1                     (0x2 << RP23XX_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_CLKSRC_PLL_USB                   (0x3 << RP23XX_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_CLKSRC_PLL_USB_PRIMARY_REF_OPCG  (0x4 << RP23XX_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_ROSC_CLKSRC                      (0x5 << RP23XX_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_XOSC_CLKSRC                      (0x6 << RP23XX_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_LPOSC_CLKSRC                     (0x7 << RP23XX_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_CLK_SYS                          (0x8 << RP23XX_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_CLK_USB                          (0x9 << RP23XX_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_CLK_ADC                          (0xa << RP23XX_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_CLK_REF                          (0xb << RP23XX_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_CLK_PERI                         (0xc << RP23XX_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_CLK_HSTX                         (0xd << RP23XX_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_CLK_CLK2FC                       (0xe << RP23XX_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_SHIFT)

#define RP23XX_CLOCKS_CLK_GPOUT1_DIV_INT_SHIFT                                (16)       /* Integer component of the divisor, 0 -> divide by 2^16 */
#define RP23XX_CLOCKS_CLK_GPOUT1_DIV_INT_MASK                                 (0xffff << RP23XX_CLOCKS_CLK_GPOUT1_DIV_INT_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT1_DIV_FRAC_MASK                                (0xffff)   /* Fractional component of the divisor */

#define RP23XX_CLOCKS_CLK_GPOUT2_CTRL_NUDGE                                   (1 << 20)  /* An edge on this signal shifts the phase of the output by 1 cycle of the input clock This can be done at any time */
#define RP23XX_CLOCKS_CLK_GPOUT2_CTRL_PHASE_SHIFT                             (16)       /* This delays the enable signal by up to 3 cycles of the input clock This must be set before the clock is enabled to have any effect */
#define RP23XX_CLOCKS_CLK_GPOUT2_CTRL_PHASE_MASK                              (0x03 << RP23XX_CLOCKS_CLK_GPOUT2_CTRL_PHASE_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT2_CTRL_DC50                                    (1 << 12)  /* Enables duty cycle correction for odd divisors */
#define RP23XX_CLOCKS_CLK_GPOUT2_CTRL_ENABLE                                  (1 << 11)  /* Starts and stops the clock generator cleanly */
#define RP23XX_CLOCKS_CLK_GPOUT2_CTRL_KILL                                    (1 << 10)  /* Asynchronously kills the clock generator */
#define RP23XX_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_SHIFT                            (5)        /* Selects the auxiliary clock source, will glitch when switching */
#define RP23XX_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_MASK                             (0x0f << RP23XX_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_CLKSRC_PLL_SYS                   (0x0 << RP23XX_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_CLKSRC_GPIN0                     (0x1 << RP23XX_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_CLKSRC_GPIN1                     (0x2 << RP23XX_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_CLKSRC_PLL_USB                   (0x3 << RP23XX_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_CLKSRC_PLL_USB_PRIMARY_REF_OPCG  (0x4 << RP23XX_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_ROSC_CLKSRC_PH                   (0x5 << RP23XX_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_XOSC_CLKSRC                      (0x6 << RP23XX_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_LPOSC_CLKSRC                     (0x7 << RP23XX_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_CLK_SYS                          (0x8 << RP23XX_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_CLK_USB                          (0x9 << RP23XX_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_CLK_ADC                          (0xa << RP23XX_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_CLK_REF                          (0xb << RP23XX_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_CLK_PERI                         (0xc << RP23XX_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_CLK_HSTX                         (0xd << RP23XX_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_CLK_CLK2FC                       (0xe << RP23XX_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_SHIFT)

#define RP23XX_CLOCKS_CLK_GPOUT2_DIV_INT_SHIFT                                (16)       /* Integer component of the divisor, 0 -> divide by 2^16 */
#define RP23XX_CLOCKS_CLK_GPOUT2_DIV_INT_MASK                                 (0xffff << RP23XX_CLOCKS_CLK_GPOUT2_DIV_INT_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT2_DIV_FRAC_MASK                                (0xffff)   /* Fractional component of the divisor */

#define RP23XX_CLOCKS_CLK_GPOUT3_CTRL_NUDGE                                   (1 << 20)  /* An edge on this signal shifts the phase of the output by 1 cycle of the input clock This can be done at any time */
#define RP23XX_CLOCKS_CLK_GPOUT3_CTRL_PHASE_SHIFT                             (16)       /* This delays the enable signal by up to 3 cycles of the input clock This must be set before the clock is enabled to have any effect */
#define RP23XX_CLOCKS_CLK_GPOUT3_CTRL_PHASE_MASK                              (0x03 << RP23XX_CLOCKS_CLK_GPOUT3_CTRL_PHASE_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT3_CTRL_DC50                                    (1 << 12)  /* Enables duty cycle correction for odd divisors */
#define RP23XX_CLOCKS_CLK_GPOUT3_CTRL_ENABLE                                  (1 << 11)  /* Starts and stops the clock generator cleanly */
#define RP23XX_CLOCKS_CLK_GPOUT3_CTRL_KILL                                    (1 << 10)  /* Asynchronously kills the clock generator */
#define RP23XX_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_SHIFT                            (5)        /* Selects the auxiliary clock source, will glitch when switching */
#define RP23XX_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_MASK                             (0x0f << RP23XX_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_CLKSRC_PLL_SYS                   (0x0 << RP23XX_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_CLKSRC_GPIN0                     (0x1 << RP23XX_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_CLKSRC_GPIN1                     (0x2 << RP23XX_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_CLKSRC_PLL_USB                   (0x3 << RP23XX_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_CLKSRC_PLL_USB_PRIMARY_REF_OPCG  (0x4 << RP23XX_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_ROSC_CLKSRC_PH                   (0x5 << RP23XX_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_XOSC_CLKSRC                      (0x6 << RP23XX_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_LPOSC_CLKSRC                     (0x7 << RP23XX_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_CLK_SYS                          (0x8 << RP23XX_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_CLK_USB                          (0x9 << RP23XX_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_CLK_ADC                          (0xa << RP23XX_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_CLK_REF                          (0xb << RP23XX_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_CLK_PERI                         (0xc << RP23XX_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_CLK_HSTX                         (0xd << RP23XX_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_CLK_CLK2FC                       (0xe << RP23XX_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_SHIFT)

#define RP23XX_CLOCKS_CLK_GPOUT3_DIV_INT_SHIFT                                (16)       /* Integer component of the divisor, 0 -> divide by 2^16 */
#define RP23XX_CLOCKS_CLK_GPOUT3_DIV_INT_MASK                                 (0xffff << RP23XX_CLOCKS_CLK_GPOUT3_DIV_INT_SHIFT)
#define RP23XX_CLOCKS_CLK_GPOUT3_DIV_FRAC_MASK                                (0xffff)   /* Fractional component of the divisor */

#define RP23XX_CLOCKS_CLK_REF_CTRL_AUXSRC_SHIFT                               (5)        /* Selects the auxiliary clock source, will glitch when switching */
#define RP23XX_CLOCKS_CLK_REF_CTRL_AUXSRC_MASK                                (0x03 << RP23XX_CLOCKS_CLK_REF_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_REF_CTRL_AUXSRC_CLKSRC_PLL_USB                      (0x0 << RP23XX_CLOCKS_CLK_REF_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_REF_CTRL_AUXSRC_CLKSRC_GPIN0                        (0x1 << RP23XX_CLOCKS_CLK_REF_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_REF_CTRL_AUXSRC_CLKSRC_GPIN1                        (0x2 << RP23XX_CLOCKS_CLK_REF_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_REF_CTRL_AUXSRC_CLKSRC_PLL_USB_PRIMARY_REF_OPCG     (0x3 << RP23XX_CLOCKS_CLK_REF_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_REF_CTRL_SRC_MASK                                   (0x03)
#define RP23XX_CLOCKS_CLK_REF_CTRL_SRC_ROSC_CLKSRC_PH                         (0x0)
#define RP23XX_CLOCKS_CLK_REF_CTRL_SRC_CLKSRC_CLK_REF_AUX                     (0x1)
#define RP23XX_CLOCKS_CLK_REF_CTRL_SRC_XOSC_CLKSRC                            (0x2)
#define RP23XX_CLOCKS_CLK_REF_CTRL_SRC_LPOSC_CLKSRC                           (0x3)

#define RP23XX_CLOCKS_CLK_REF_DIV_INT_SHIFT                                   (8)        /* Integer component of the divisor, 0 -> divide by 2^2 */
#define RP23XX_CLOCKS_CLK_REF_DIV_INT_MASK                                    (0x03 << RP23XX_CLOCKS_CLK_REF_DIV_INT_SHIFT)

#define RP23XX_CLOCKS_CLK_SYS_CTRL_AUXSRC_SHIFT                               (5)        /* Selects the auxiliary clock source, will glitch when switching */
#define RP23XX_CLOCKS_CLK_SYS_CTRL_AUXSRC_MASK                                (0x07 << RP23XX_CLOCKS_CLK_SYS_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_SYS_CTRL_AUXSRC_CLKSRC_PLL_SYS                      (0x0 << RP23XX_CLOCKS_CLK_SYS_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_SYS_CTRL_AUXSRC_CLKSRC_PLL_USB                      (0x1 << RP23XX_CLOCKS_CLK_SYS_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_SYS_CTRL_AUXSRC_ROSC_CLKSRC                         (0x2 << RP23XX_CLOCKS_CLK_SYS_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_SYS_CTRL_AUXSRC_XOSC_CLKSRC                         (0x3 << RP23XX_CLOCKS_CLK_SYS_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_SYS_CTRL_AUXSRC_CLKSRC_GPIN0                        (0x4 << RP23XX_CLOCKS_CLK_SYS_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_SYS_CTRL_AUXSRC_CLKSRC_GPIN1                        (0x5 << RP23XX_CLOCKS_CLK_SYS_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_SYS_CTRL_SRC                                        (1 << 0)
#define RP23XX_CLOCKS_CLK_SYS_CTRL_SRC_CLKSRC_CLK_SYS_AUX                     (0x1)

#define RP23XX_CLOCKS_CLK_SYS_DIV_INT_SHIFT                                   (16)       /* Integer component of the divisor, 0 -> divide by 2^16 */
#define RP23XX_CLOCKS_CLK_SYS_DIV_INT_MASK                                    (0xffff << RP23XX_CLOCKS_CLK_SYS_DIV_INT_SHIFT)
#define RP23XX_CLOCKS_CLK_SYS_DIV_FRAC_MASK                                   (0xffff)   /* Fractional component of the divisor */

#define RP23XX_CLOCKS_CLK_PERI_CTRL_ENABLE                                    (1 << 11)  /* Starts and stops the clock generator cleanly */
#define RP23XX_CLOCKS_CLK_PERI_CTRL_KILL                                      (1 << 10)  /* Asynchronously kills the clock generator */
#define RP23XX_CLOCKS_CLK_PERI_CTRL_AUXSRC_SHIFT                              (5)        /* Selects the auxiliary clock source, will glitch when switching */
#define RP23XX_CLOCKS_CLK_PERI_CTRL_AUXSRC_MASK                               (0x07 << RP23XX_CLOCKS_CLK_PERI_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_PERI_CTRL_AUXSRC_CLK_SYS                            (0x0 << RP23XX_CLOCKS_CLK_PERI_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_PERI_CTRL_AUXSRC_CLKSRC_PLL_SYS                     (0x1 << RP23XX_CLOCKS_CLK_PERI_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_PERI_CTRL_AUXSRC_CLKSRC_PLL_USB                     (0x2 << RP23XX_CLOCKS_CLK_PERI_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_PERI_CTRL_AUXSRC_ROSC_CLKSRC_PH                     (0x3 << RP23XX_CLOCKS_CLK_PERI_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_PERI_CTRL_AUXSRC_XOSC_CLKSRC                        (0x4 << RP23XX_CLOCKS_CLK_PERI_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_PERI_CTRL_AUXSRC_CLKSRC_GPIN0                       (0x5 << RP23XX_CLOCKS_CLK_PERI_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_PERI_CTRL_AUXSRC_CLKSRC_GPIN1                       (0x6 << RP23XX_CLOCKS_CLK_PERI_CTRL_AUXSRC_SHIFT)

#define RP23XX_CLOCKS_CLK_PERI_DIV_INT_SHIFT                                  (16)       /* Integer component of the divisor, 0 -> divide by 2^2 */
#define RP23XX_CLOCKS_CLK_PERI_DIV_INT_MASK                                   (0x03 << RP23XX_CLOCKS_CLK_PERI_DIV_INT_SHIFT)

#define RP23XX_CLOCKS_CLK_HSTX_CTRL_NUDGE                                     (1 << 20)  /* An edge on this signal shifts the phase of the output by 1 cycle of the input clock This can be done at any time */
#define RP23XX_CLOCKS_CLK_HSTX_CTRL_PHASE_SHIFT                               (16)       /* This delays the enable signal by up to 3 cycles of the input clock This must be set before the clock is enabled to have any effect */
#define RP23XX_CLOCKS_CLK_HSTX_CTRL_PHASE_MASK                                (0x03 << RP23XX_CLOCKS_CLK_HSTX_CTRL_PHASE_SHIFT)
#define RP23XX_CLOCKS_CLK_HSTX_CTRL_ENABLE                                    (1 << 11)  /* Starts and stops the clock generator cleanly */
#define RP23XX_CLOCKS_CLK_HSTX_CTRL_KILL                                      (1 << 10)  /* Asynchronously kills the clock generator */
#define RP23XX_CLOCKS_CLK_HSTX_CTRL_AUXSRC_SHIFT                              (5)        /* Selects the auxiliary clock source, will glitch when switching */
#define RP23XX_CLOCKS_CLK_HSTX_CTRL_AUXSRC_MASK                               (0x07 << RP23XX_CLOCKS_CLK_HSTX_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_HSTX_CTRL_AUXSRC_CLK_SYS                            (0x0 << RP23XX_CLOCKS_CLK_HSTX_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_HSTX_CTRL_AUXSRC_CLKSRC_PLL_SYS                     (0x1 << RP23XX_CLOCKS_CLK_HSTX_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_HSTX_CTRL_AUXSRC_CLKSRC_PLL_USB                     (0x2 << RP23XX_CLOCKS_CLK_HSTX_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_HSTX_CTRL_AUXSRC_CLKSRC_GPIN0                       (0x3 << RP23XX_CLOCKS_CLK_HSTX_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_HSTX_CTRL_AUXSRC_CLKSRC_GPIN1                       (0x4 << RP23XX_CLOCKS_CLK_HSTX_CTRL_AUXSRC_SHIFT)

#define RP23XX_CLOCKS_CLK_HSTX_DIV_INT_SHIFT                                  (16)       /* Integer component of the divisor, 0 -> divide by 2^2 */
#define RP23XX_CLOCKS_CLK_HSTX_DIV_INT_MASK                                   (0x03 << RP23XX_CLOCKS_CLK_HSTX_DIV_INT_SHIFT)

#define RP23XX_CLOCKS_CLK_USB_CTRL_NUDGE                                      (1 << 20)  /* An edge on this signal shifts the phase of the output by 1 cycle of the input clock This can be done at any time */
#define RP23XX_CLOCKS_CLK_USB_CTRL_PHASE_SHIFT                                (16)       /* This delays the enable signal by up to 3 cycles of the input clock This must be set before the clock is enabled to have any effect */
#define RP23XX_CLOCKS_CLK_USB_CTRL_PHASE_MASK                                 (0x03 << RP23XX_CLOCKS_CLK_USB_CTRL_PHASE_SHIFT)
#define RP23XX_CLOCKS_CLK_USB_CTRL_ENABLE                                     (1 << 11)  /* Starts and stops the clock generator cleanly */
#define RP23XX_CLOCKS_CLK_USB_CTRL_KILL                                       (1 << 10)  /* Asynchronously kills the clock generator */
#define RP23XX_CLOCKS_CLK_USB_CTRL_AUXSRC_SHIFT                               (5)        /* Selects the auxiliary clock source, will glitch when switching */
#define RP23XX_CLOCKS_CLK_USB_CTRL_AUXSRC_MASK                                (0x07 << RP23XX_CLOCKS_CLK_USB_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_USB_CTRL_AUXSRC_CLKSRC_PLL_USB                      (0x0 << RP23XX_CLOCKS_CLK_USB_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_USB_CTRL_AUXSRC_CLKSRC_PLL_SYS                      (0x1 << RP23XX_CLOCKS_CLK_USB_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_USB_CTRL_AUXSRC_ROSC_CLKSRC_PH                      (0x2 << RP23XX_CLOCKS_CLK_USB_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_USB_CTRL_AUXSRC_XOSC_CLKSRC                         (0x3 << RP23XX_CLOCKS_CLK_USB_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_USB_CTRL_AUXSRC_CLKSRC_GPIN0                        (0x4 << RP23XX_CLOCKS_CLK_USB_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_USB_CTRL_AUXSRC_CLKSRC_GPIN1                        (0x5 << RP23XX_CLOCKS_CLK_USB_CTRL_AUXSRC_SHIFT)

#define RP23XX_CLOCKS_CLK_USB_DIV_INT_SHIFT                                   (16)       /* Integer component of the divisor, 0 -> divide by 2^4 */
#define RP23XX_CLOCKS_CLK_USB_DIV_INT_MASK                                    (0x0f << RP23XX_CLOCKS_CLK_USB_DIV_INT_SHIFT)

#define RP23XX_CLOCKS_CLK_ADC_CTRL_NUDGE                                      (1 << 20)  /* An edge on this signal shifts the phase of the output by 1 cycle of the input clock This can be done at any time */
#define RP23XX_CLOCKS_CLK_ADC_CTRL_PHASE_SHIFT                                (16)       /* This delays the enable signal by up to 3 cycles of the input clock This must be set before the clock is enabled to have any effect */
#define RP23XX_CLOCKS_CLK_ADC_CTRL_PHASE_MASK                                 (0x03 << RP23XX_CLOCKS_CLK_ADC_CTRL_PHASE_SHIFT)
#define RP23XX_CLOCKS_CLK_ADC_CTRL_ENABLE                                     (1 << 11)  /* Starts and stops the clock generator cleanly */
#define RP23XX_CLOCKS_CLK_ADC_CTRL_KILL                                       (1 << 10)  /* Asynchronously kills the clock generator */
#define RP23XX_CLOCKS_CLK_ADC_CTRL_AUXSRC_SHIFT                               (5)        /* Selects the auxiliary clock source, will glitch when switching */
#define RP23XX_CLOCKS_CLK_ADC_CTRL_AUXSRC_MASK                                (0x07 << RP23XX_CLOCKS_CLK_ADC_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_ADC_CTRL_AUXSRC_CLKSRC_PLL_USB                      (0x0 << RP23XX_CLOCKS_CLK_ADC_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_ADC_CTRL_AUXSRC_CLKSRC_PLL_SYS                      (0x1 << RP23XX_CLOCKS_CLK_ADC_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_ADC_CTRL_AUXSRC_ROSC_CLKSRC_PH                      (0x2 << RP23XX_CLOCKS_CLK_ADC_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_ADC_CTRL_AUXSRC_XOSC_CLKSRC                         (0x3 << RP23XX_CLOCKS_CLK_ADC_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_ADC_CTRL_AUXSRC_CLKSRC_GPIN0                        (0x4 << RP23XX_CLOCKS_CLK_ADC_CTRL_AUXSRC_SHIFT)
#define RP23XX_CLOCKS_CLK_ADC_CTRL_AUXSRC_CLKSRC_GPIN1                        (0x5 << RP23XX_CLOCKS_CLK_ADC_CTRL_AUXSRC_SHIFT)

#define RP23XX_CLOCKS_CLK_ADC_DIV_INT_SHIFT                                   (16)       /* Integer component of the divisor, 0 -> divide by 2^4 */
#define RP23XX_CLOCKS_CLK_ADC_DIV_INT_MASK                                    (0x0f << RP23XX_CLOCKS_CLK_ADC_DIV_INT_SHIFT)

#define RP23XX_CLOCKS_DFTCLK_XOSC_CTRL_MASK                                   (0x03)
#define RP23XX_CLOCKS_DFTCLK_XOSC_CTRL_SRC_NULL                               (0x0)
#define RP23XX_CLOCKS_DFTCLK_XOSC_CTRL_SRC_CLKSRC_PLL_USB_PRIMARY             (0x1)
#define RP23XX_CLOCKS_DFTCLK_XOSC_CTRL_SRC_CLKSRC_GPIN0                       (0x2)

#define RP23XX_CLOCKS_DFTCLK_ROSC_CTRL_MASK                                   (0x03)
#define RP23XX_CLOCKS_DFTCLK_ROSC_CTRL_SRC_NULL                               (0x0)
#define RP23XX_CLOCKS_DFTCLK_ROSC_CTRL_SRC_CLKSRC_PLL_SYS_PRIMARY_ROSC        (0x1)
#define RP23XX_CLOCKS_DFTCLK_ROSC_CTRL_SRC_CLKSRC_GPIN1                       (0x2)

#define RP23XX_CLOCKS_DFTCLK_LPOSC_CTRL_MASK                                  (0x03)
#define RP23XX_CLOCKS_DFTCLK_LPOSC_CTRL_SRC_NULL                              (0x0)
#define RP23XX_CLOCKS_DFTCLK_LPOSC_CTRL_SRC_CLKSRC_PLL_USB_PRIMARY_LPOSC      (0x1)
#define RP23XX_CLOCKS_DFTCLK_LPOSC_CTRL_SRC_CLKSRC_GPIN1                      (0x2)

#define RP23XX_CLOCKS_CLK_SYS_RESUS_CTRL_CLEAR                                (1 << 16)  /* For clearing the resus after the fault that triggered it has been corrected */
#define RP23XX_CLOCKS_CLK_SYS_RESUS_CTRL_FRCE                                 (1 << 12)  /* Force a resus, for test purposes only */
#define RP23XX_CLOCKS_CLK_SYS_RESUS_CTRL_ENABLE                               (1 << 8)   /* Enable resus */
#define RP23XX_CLOCKS_CLK_SYS_RESUS_CTRL_TIMEOUT_MASK                         (0xff)     /* This is expressed as a number of clk_ref cycles and must be >= 2x /min_ */

#define RP23XX_CLOCKS_CLK_SYS_RESUS_STATUS_RESUSSED                           (1 << 0)   /* Clock has been resuscitated, correct the error then send ctrl_clear=1 */

#define RP23XX_CLOCKS_FC0_REF_KHZ_MASK                                        (0xfffff)

#define RP23XX_CLOCKS_FC0_MIN_KHZ_MASK                                        (0x1ffffff)

#define RP23XX_CLOCKS_FC0_MAX_KHZ_MASK                                        (0x1ffffff)

#define RP23XX_CLOCKS_FC0_DELAY_MASK                                          (0x07)

#define RP23XX_CLOCKS_FC0_INTERVAL_MASK                                       (0x0f)

#define RP23XX_CLOCKS_FC0_SRC_MASK                                            (0xff)
#define RP23XX_CLOCKS_FC0_SRC_NULL                                            (0x0)
#define RP23XX_CLOCKS_FC0_SRC_PLL_SYS_CLKSRC_PRIMARY                          (0x1)
#define RP23XX_CLOCKS_FC0_SRC_PLL_USB_CLKSRC_PRIMARY                          (0x2)
#define RP23XX_CLOCKS_FC0_SRC_ROSC_CLKSRC                                     (0x3)
#define RP23XX_CLOCKS_FC0_SRC_ROSC_CLKSRC_PH                                  (0x4)
#define RP23XX_CLOCKS_FC0_SRC_XOSC_CLKSRC                                     (0x5)
#define RP23XX_CLOCKS_FC0_SRC_CLKSRC_GPIN0                                    (0x6)
#define RP23XX_CLOCKS_FC0_SRC_CLKSRC_GPIN1                                    (0x7)
#define RP23XX_CLOCKS_FC0_SRC_CLK_REF                                         (0x8)
#define RP23XX_CLOCKS_FC0_SRC_CLK_SYS                                         (0x9)
#define RP23XX_CLOCKS_FC0_SRC_CLK_PERI                                        (0xa)
#define RP23XX_CLOCKS_FC0_SRC_CLK_USB                                         (0xb)
#define RP23XX_CLOCKS_FC0_SRC_CLK_ADC                                         (0xc)
#define RP23XX_CLOCKS_FC0_SRC_CLK_HSTX                                        (0xd)
#define RP23XX_CLOCKS_FC0_SRC_CLK_LPOSC_CLKSRC                                (0xe)
#define RP23XX_CLOCKS_FC0_SRC_CLK_OTP_CLK2FC                                  (0xf)
#define RP23XX_CLOCKS_FC0_SRC_CLK_PLL_USB_CLKSRC_PRIMARY_DFT                  (0x10)

#define RP23XX_CLOCKS_FC0_STATUS_DIED                                         (1 << 28)  /* Test clock stopped during test */
#define RP23XX_CLOCKS_FC0_STATUS_FAST                                         (1 << 24)  /* Test clock faster than expected, only valid when status_done=1 */
#define RP23XX_CLOCKS_FC0_STATUS_SLOW                                         (1 << 20)  /* Test clock slower than expected, only valid when status_done=1 */
#define RP23XX_CLOCKS_FC0_STATUS_FAIL                                         (1 << 16)  /* Test failed */
#define RP23XX_CLOCKS_FC0_STATUS_WAITING                                      (1 << 12)  /* Waiting for test clock to start */
#define RP23XX_CLOCKS_FC0_STATUS_RUNNING                                      (1 << 8)   /* Test running */
#define RP23XX_CLOCKS_FC0_STATUS_DONE                                         (1 << 4)   /* Test complete */
#define RP23XX_CLOCKS_FC0_STATUS_PASS                                         (1 << 0)   /* Test passed */

#define RP23XX_CLOCKS_FC0_RESULT_KHZ_SHIFT                                    (5)
#define RP23XX_CLOCKS_FC0_RESULT_KHZ_MASK                                     (0x1ffffff << RP23XX_CLOCKS_FC0_RESULT_KHZ_SHIFT)
#define RP23XX_CLOCKS_FC0_RESULT_FRAC_MASK                                    (0x1f)

#define RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_SIO                                    (1 << 31)
#define RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_SHA256                                 (1 << 30)
#define RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_PSM                                    (1 << 29)
#define RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_ROSC                                   (1 << 28)
#define RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_ROM                                    (1 << 27)
#define RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_RESETS                                 (1 << 26)
#define RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_PWM                                    (1 << 25)
#define RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_POWMAN                                 (1 << 24)
#define RP23XX_CLOCKS_WAKE_EN0_CLK_REF_POWMAN                                 (1 << 23)
#define RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_PLL_USB                                (1 << 22)
#define RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_PLL_SYS                                (1 << 21)
#define RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_PIO2                                   (1 << 20)
#define RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_PIO1                                   (1 << 19)
#define RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_PIO0                                   (1 << 18)
#define RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_PADS                                   (1 << 17)
#define RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_OTP                                    (1 << 16)
#define RP23XX_CLOCKS_WAKE_EN0_CLK_REF_OTP                                    (1 << 15)
#define RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_JTAG                                   (1 << 14)
#define RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_IO                                     (1 << 13)
#define RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_I2C1                                   (1 << 12)
#define RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_I2C0                                   (1 << 11)
#define RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_HSTX                                   (1 << 10)
#define RP23XX_CLOCKS_WAKE_EN0_CLK_HSTX                                       (1 << 9)
#define RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_GLITCH_DETECTOR                        (1 << 8)
#define RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_DMA                                    (1 << 7)
#define RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_BUSFABRIC                              (1 << 6)
#define RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_BUSCTRL                                (1 << 5)
#define RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_BOOTRAM                                (1 << 4)
#define RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_ADC                                    (1 << 3)
#define RP23XX_CLOCKS_WAKE_EN0_CLK_ADC_ADC                                    (1 << 2)
#define RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_ACCESSCTRL                             (1 << 1)
#define RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_CLOCKS                                 (1 << 0)

#define RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_XOSC                                   (1 << 30)
#define RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_XIP                                    (1 << 29)
#define RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_WATCHDOG                               (1 << 28)
#define RP23XX_CLOCKS_WAKE_EN1_CLK_USB                                        (1 << 27)
#define RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_USBCTRL                                (1 << 26)
#define RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_UART1                                  (1 << 25)
#define RP23XX_CLOCKS_WAKE_EN1_CLK_PERI_UART1                                 (1 << 24)
#define RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_UART0                                  (1 << 23)
#define RP23XX_CLOCKS_WAKE_EN1_CLK_PERI_UART0                                 (1 << 22)
#define RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_TRNG                                   (1 << 21)
#define RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_TIMER1                                 (1 << 20)
#define RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_TIMER0                                 (1 << 19)
#define RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_TICKS                                  (1 << 18)
#define RP23XX_CLOCKS_WAKE_EN1_CLK_REF_TICKS                                  (1 << 17)
#define RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_TBMAN                                  (1 << 16)
#define RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_SYSINFO                                (1 << 15)
#define RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_SYSCFG                                 (1 << 14)
#define RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_SRAM9                                  (1 << 13)
#define RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_SRAM8                                  (1 << 12)
#define RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_SRAM7                                  (1 << 11)
#define RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_SRAM6                                  (1 << 10)
#define RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_SRAM5                                  (1 << 9)
#define RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_SRAM4                                  (1 << 8)
#define RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_SRAM3                                  (1 << 7)
#define RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_SRAM2                                  (1 << 6)
#define RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_SRAM1                                  (1 << 5)
#define RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_SRAM0                                  (1 << 4)
#define RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_SPI1                                   (1 << 3)
#define RP23XX_CLOCKS_WAKE_EN1_CLK_PERI_SPI1                                  (1 << 2)
#define RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_SPI0                                   (1 << 1)
#define RP23XX_CLOCKS_WAKE_EN1_CLK_PERI_SPI0                                  (1 << 0)

#define RP23XX_CLOCKS_SLEEP_EN0_CLK_SYS_SIO                                   (1 << 31)
#define RP23XX_CLOCKS_SLEEP_EN0_CLK_SYS_SHA256                                (1 << 30)
#define RP23XX_CLOCKS_SLEEP_EN0_CLK_SYS_PSM                                   (1 << 29)
#define RP23XX_CLOCKS_SLEEP_EN0_CLK_SYS_ROSC                                  (1 << 28)
#define RP23XX_CLOCKS_SLEEP_EN0_CLK_SYS_ROM                                   (1 << 27)
#define RP23XX_CLOCKS_SLEEP_EN0_CLK_SYS_RESETS                                (1 << 26)
#define RP23XX_CLOCKS_SLEEP_EN0_CLK_SYS_PWM                                   (1 << 25)
#define RP23XX_CLOCKS_SLEEP_EN0_CLK_SYS_POWMAN                                (1 << 24)
#define RP23XX_CLOCKS_SLEEP_EN0_CLK_REF_POWMAN                                (1 << 23)
#define RP23XX_CLOCKS_SLEEP_EN0_CLK_SYS_PLL_USB                               (1 << 22)
#define RP23XX_CLOCKS_SLEEP_EN0_CLK_SYS_PLL_SYS                               (1 << 21)
#define RP23XX_CLOCKS_SLEEP_EN0_CLK_SYS_PIO2                                  (1 << 20)
#define RP23XX_CLOCKS_SLEEP_EN0_CLK_SYS_PIO1                                  (1 << 19)
#define RP23XX_CLOCKS_SLEEP_EN0_CLK_SYS_PIO0                                  (1 << 18)
#define RP23XX_CLOCKS_SLEEP_EN0_CLK_SYS_PADS                                  (1 << 17)
#define RP23XX_CLOCKS_SLEEP_EN0_CLK_SYS_OTP                                   (1 << 16)
#define RP23XX_CLOCKS_SLEEP_EN0_CLK_REF_OTP                                   (1 << 15)
#define RP23XX_CLOCKS_SLEEP_EN0_CLK_SYS_JTAG                                  (1 << 14)
#define RP23XX_CLOCKS_SLEEP_EN0_CLK_SYS_IO                                    (1 << 13)
#define RP23XX_CLOCKS_SLEEP_EN0_CLK_SYS_I2C1                                  (1 << 12)
#define RP23XX_CLOCKS_SLEEP_EN0_CLK_SYS_I2C0                                  (1 << 11)
#define RP23XX_CLOCKS_SLEEP_EN0_CLK_SYS_HSTX                                  (1 << 10)
#define RP23XX_CLOCKS_SLEEP_EN0_CLK_HSTX                                      (1 << 9)
#define RP23XX_CLOCKS_SLEEP_EN0_CLK_SYS_GLITCH_DETECTOR                       (1 << 8)
#define RP23XX_CLOCKS_SLEEP_EN0_CLK_SYS_DMA                                   (1 << 7)
#define RP23XX_CLOCKS_SLEEP_EN0_CLK_SYS_BUSFABRIC                             (1 << 6)
#define RP23XX_CLOCKS_SLEEP_EN0_CLK_SYS_BUSCTRL                               (1 << 5)
#define RP23XX_CLOCKS_SLEEP_EN0_CLK_SYS_BOOTRAM                               (1 << 4)
#define RP23XX_CLOCKS_SLEEP_EN0_CLK_SYS_ADC                                   (1 << 3)
#define RP23XX_CLOCKS_SLEEP_EN0_CLK_ADC_ADC                                   (1 << 2)
#define RP23XX_CLOCKS_SLEEP_EN0_CLK_SYS_ACCESSCTRL                            (1 << 1)
#define RP23XX_CLOCKS_SLEEP_EN0_CLK_SYS_CLOCKS                                (1 << 0)

#define RP23XX_CLOCKS_SLEEP_EN1_CLK_SYS_XOSC                                  (1 << 30)
#define RP23XX_CLOCKS_SLEEP_EN1_CLK_SYS_XIP                                   (1 << 29)
#define RP23XX_CLOCKS_SLEEP_EN1_CLK_SYS_WATCHDOG                              (1 << 28)
#define RP23XX_CLOCKS_SLEEP_EN1_CLK_USB                                       (1 << 27)
#define RP23XX_CLOCKS_SLEEP_EN1_CLK_SYS_USBCTRL                               (1 << 26)
#define RP23XX_CLOCKS_SLEEP_EN1_CLK_SYS_UART1                                 (1 << 25)
#define RP23XX_CLOCKS_SLEEP_EN1_CLK_PERI_UART1                                (1 << 24)
#define RP23XX_CLOCKS_SLEEP_EN1_CLK_SYS_UART0                                 (1 << 23)
#define RP23XX_CLOCKS_SLEEP_EN1_CLK_PERI_UART0                                (1 << 22)
#define RP23XX_CLOCKS_SLEEP_EN1_CLK_SYS_TRNG                                  (1 << 21)
#define RP23XX_CLOCKS_SLEEP_EN1_CLK_SYS_TIMER1                                (1 << 20)
#define RP23XX_CLOCKS_SLEEP_EN1_CLK_SYS_TIMER0                                (1 << 19)
#define RP23XX_CLOCKS_SLEEP_EN1_CLK_SYS_TICKS                                 (1 << 18)
#define RP23XX_CLOCKS_SLEEP_EN1_CLK_REF_TICKS                                 (1 << 17)
#define RP23XX_CLOCKS_SLEEP_EN1_CLK_SYS_TBMAN                                 (1 << 16)
#define RP23XX_CLOCKS_SLEEP_EN1_CLK_SYS_SYSINFO                               (1 << 15)
#define RP23XX_CLOCKS_SLEEP_EN1_CLK_SYS_SYSCFG                                (1 << 14)
#define RP23XX_CLOCKS_SLEEP_EN1_CLK_SYS_SRAM9                                 (1 << 13)
#define RP23XX_CLOCKS_SLEEP_EN1_CLK_SYS_SRAM8                                 (1 << 12)
#define RP23XX_CLOCKS_SLEEP_EN1_CLK_SYS_SRAM7                                 (1 << 11)
#define RP23XX_CLOCKS_SLEEP_EN1_CLK_SYS_SRAM6                                 (1 << 10)
#define RP23XX_CLOCKS_SLEEP_EN1_CLK_SYS_SRAM5                                 (1 << 9)
#define RP23XX_CLOCKS_SLEEP_EN1_CLK_SYS_SRAM4                                 (1 << 8)
#define RP23XX_CLOCKS_SLEEP_EN1_CLK_SYS_SRAM3                                 (1 << 7)
#define RP23XX_CLOCKS_SLEEP_EN1_CLK_SYS_SRAM2                                 (1 << 6)
#define RP23XX_CLOCKS_SLEEP_EN1_CLK_SYS_SRAM1                                 (1 << 5)
#define RP23XX_CLOCKS_SLEEP_EN1_CLK_SYS_SRAM0                                 (1 << 4)
#define RP23XX_CLOCKS_SLEEP_EN1_CLK_SYS_SPI1                                  (1 << 3)
#define RP23XX_CLOCKS_SLEEP_EN1_CLK_PERI_SPI1                                 (1 << 2)
#define RP23XX_CLOCKS_SLEEP_EN1_CLK_SYS_SPI0                                  (1 << 1)
#define RP23XX_CLOCKS_SLEEP_EN1_CLK_PERI_SPI0                                 (1 << 0)

#define RP23XX_CLOCKS_ENABLED0_CLK_SYS_SIO                                    (1 << 31)
#define RP23XX_CLOCKS_ENABLED0_CLK_SYS_SHA256                                 (1 << 30)
#define RP23XX_CLOCKS_ENABLED0_CLK_SYS_PSM                                    (1 << 29)
#define RP23XX_CLOCKS_ENABLED0_CLK_SYS_ROSC                                   (1 << 28)
#define RP23XX_CLOCKS_ENABLED0_CLK_SYS_ROM                                    (1 << 27)
#define RP23XX_CLOCKS_ENABLED0_CLK_SYS_RESETS                                 (1 << 26)
#define RP23XX_CLOCKS_ENABLED0_CLK_SYS_PWM                                    (1 << 25)
#define RP23XX_CLOCKS_ENABLED0_CLK_SYS_POWMAN                                 (1 << 24)
#define RP23XX_CLOCKS_ENABLED0_CLK_REF_POWMAN                                 (1 << 23)
#define RP23XX_CLOCKS_ENABLED0_CLK_SYS_PLL_USB                                (1 << 22)
#define RP23XX_CLOCKS_ENABLED0_CLK_SYS_PLL_SYS                                (1 << 21)
#define RP23XX_CLOCKS_ENABLED0_CLK_SYS_PIO2                                   (1 << 20)
#define RP23XX_CLOCKS_ENABLED0_CLK_SYS_PIO1                                   (1 << 19)
#define RP23XX_CLOCKS_ENABLED0_CLK_SYS_PIO0                                   (1 << 18)
#define RP23XX_CLOCKS_ENABLED0_CLK_SYS_PADS                                   (1 << 17)
#define RP23XX_CLOCKS_ENABLED0_CLK_SYS_OTP                                    (1 << 16)
#define RP23XX_CLOCKS_ENABLED0_CLK_REF_OTP                                    (1 << 15)
#define RP23XX_CLOCKS_ENABLED0_CLK_SYS_JTAG                                   (1 << 14)
#define RP23XX_CLOCKS_ENABLED0_CLK_SYS_IO                                     (1 << 13)
#define RP23XX_CLOCKS_ENABLED0_CLK_SYS_I2C1                                   (1 << 12)
#define RP23XX_CLOCKS_ENABLED0_CLK_SYS_I2C0                                   (1 << 11)
#define RP23XX_CLOCKS_ENABLED0_CLK_SYS_HSTX                                   (1 << 10)
#define RP23XX_CLOCKS_ENABLED0_CLK_HSTX                                       (1 << 9)
#define RP23XX_CLOCKS_ENABLED0_CLK_SYS_GLITCH_DETECTOR                        (1 << 8)
#define RP23XX_CLOCKS_ENABLED0_CLK_SYS_DMA                                    (1 << 7)
#define RP23XX_CLOCKS_ENABLED0_CLK_SYS_BUSFABRIC                              (1 << 6)
#define RP23XX_CLOCKS_ENABLED0_CLK_SYS_BUSCTRL                                (1 << 5)
#define RP23XX_CLOCKS_ENABLED0_CLK_SYS_BOOTRAM                                (1 << 4)
#define RP23XX_CLOCKS_ENABLED0_CLK_SYS_ADC                                    (1 << 3)
#define RP23XX_CLOCKS_ENABLED0_CLK_ADC_ADC                                    (1 << 2)
#define RP23XX_CLOCKS_ENABLED0_CLK_SYS_ACCESSCTRL                             (1 << 1)
#define RP23XX_CLOCKS_ENABLED0_CLK_SYS_CLOCKS                                 (1 << 0)

#define RP23XX_CLOCKS_ENABLED1_CLK_SYS_XOSC                                   (1 << 30)
#define RP23XX_CLOCKS_ENABLED1_CLK_SYS_XIP                                    (1 << 29)
#define RP23XX_CLOCKS_ENABLED1_CLK_SYS_WATCHDOG                               (1 << 28)
#define RP23XX_CLOCKS_ENABLED1_CLK_USB                                        (1 << 27)
#define RP23XX_CLOCKS_ENABLED1_CLK_SYS_USBCTRL                                (1 << 26)
#define RP23XX_CLOCKS_ENABLED1_CLK_SYS_UART1                                  (1 << 25)
#define RP23XX_CLOCKS_ENABLED1_CLK_PERI_UART1                                 (1 << 24)
#define RP23XX_CLOCKS_ENABLED1_CLK_SYS_UART0                                  (1 << 23)
#define RP23XX_CLOCKS_ENABLED1_CLK_PERI_UART0                                 (1 << 22)
#define RP23XX_CLOCKS_ENABLED1_CLK_SYS_TRNG                                   (1 << 21)
#define RP23XX_CLOCKS_ENABLED1_CLK_SYS_TIMER1                                 (1 << 20)
#define RP23XX_CLOCKS_ENABLED1_CLK_SYS_TIMER0                                 (1 << 19)
#define RP23XX_CLOCKS_ENABLED1_CLK_SYS_TICKS                                  (1 << 18)
#define RP23XX_CLOCKS_ENABLED1_CLK_REF_TICKS                                  (1 << 17)
#define RP23XX_CLOCKS_ENABLED1_CLK_SYS_TBMAN                                  (1 << 16)
#define RP23XX_CLOCKS_ENABLED1_CLK_SYS_SYSINFO                                (1 << 15)
#define RP23XX_CLOCKS_ENABLED1_CLK_SYS_SYSCFG                                 (1 << 14)
#define RP23XX_CLOCKS_ENABLED1_CLK_SYS_SRAM9                                  (1 << 13)
#define RP23XX_CLOCKS_ENABLED1_CLK_SYS_SRAM8                                  (1 << 12)
#define RP23XX_CLOCKS_ENABLED1_CLK_SYS_SRAM7                                  (1 << 11)
#define RP23XX_CLOCKS_ENABLED1_CLK_SYS_SRAM6                                  (1 << 10)
#define RP23XX_CLOCKS_ENABLED1_CLK_SYS_SRAM5                                  (1 << 9)
#define RP23XX_CLOCKS_ENABLED1_CLK_SYS_SRAM4                                  (1 << 8)
#define RP23XX_CLOCKS_ENABLED1_CLK_SYS_SRAM3                                  (1 << 7)
#define RP23XX_CLOCKS_ENABLED1_CLK_SYS_SRAM2                                  (1 << 6)
#define RP23XX_CLOCKS_ENABLED1_CLK_SYS_SRAM1                                  (1 << 5)
#define RP23XX_CLOCKS_ENABLED1_CLK_SYS_SRAM0                                  (1 << 4)
#define RP23XX_CLOCKS_ENABLED1_CLK_SYS_SPI1                                   (1 << 3)
#define RP23XX_CLOCKS_ENABLED1_CLK_PERI_SPI1                                  (1 << 2)
#define RP23XX_CLOCKS_ENABLED1_CLK_SYS_SPI0                                   (1 << 1)
#define RP23XX_CLOCKS_ENABLED1_CLK_PERI_SPI0                                  (1 << 0)

#define RP23XX_CLOCKS_INTR_CLK_SYS_RESUS                                      (1 << 0)

#define RP23XX_CLOCKS_INTE_CLK_SYS_RESUS                                      (1 << 0)

#define RP23XX_CLOCKS_INTF_CLK_SYS_RESUS                                      (1 << 0)

#define RP23XX_CLOCKS_INTS_CLK_SYS_RESUS                                      (1 << 0)

#endif /* __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_CLOCKS_H */
