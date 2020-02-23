/********************************************************************************************************************
 * arch/arm/src/tiva/hardware/cc13x0/cc13x0_aux_wuc.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *
 * Technical content derives from a TI header file that has a compatible BSD license:
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
 ********************************************************************************************************************/

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X0_CC13X0_AUX_WUC_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X0_CC13X0_AUX_WUC_H

/********************************************************************************************************************
 * Included Files
 ********************************************************************************************************************/

#include <nuttx/config.h>
#include "hardware/tiva_memorymap.h"

/********************************************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************************************/

/* AUX WUC Register Offsets *****************************************************************************************/

#define TIVA_AUX_WUC_MODCLKEN0_OFFSET             0x0000  /* Module Clock Enable */
#define TIVA_AUX_WUC_PWROFFREQ_OFFSET             0x0004  /* Power Off Request */
#define TIVA_AUX_WUC_PWRDWNREQ_OFFSET             0x0008  /* Power Down Request */
#define TIVA_AUX_WUC_PWRDWNACK_OFFSET             0x000c  /* Power Down Acknowledgment */
#define TIVA_AUX_WUC_CLKLFREQ_OFFSET              0x0010  /* Low Frequency Clock Request */
#define TIVA_AUX_WUC_CLKLFACK_OFFSET              0x0014  /* Low Frequency Clock Acknowledgment */
#define TIVA_AUX_WUC_WUEVFLAGS_OFFSET             0x0028  /* Wake-up Event Flags */
#define TIVA_AUX_WUC_WUEVCLR_OFFSET               0x002c  /* Wake-up Event Clear */
#define TIVA_AUX_WUC_ADCCLKCTL_OFFSET             0x0030  /* ADC Clock Control */
#define TIVA_AUX_WUC_TDCCLKCTL_OFFSET             0x0034  /* ADC Clock Control */
#define TIVA_AUX_WUC_REFCLKCTL_OFFSET             0x0038  /* ADC Clock Control */
#define TIVA_AUX_WUC_RTCSUBSECINC0_OFFSET         0x003c  /* Real Time Counter Sub Second Increment 0 */
#define TIVA_AUX_WUC_RTCSUBSECINC1_OFFSET         0x0040  /* Real Time Counter Sub Second Increment 1 */
#define TIVA_AUX_WUC_RTCSUBSECINCCTL_OFFSET       0x0044  /* Real Time Counter Sub Second Increment Control */
#define TIVA_AUX_WUC_MCUBUSCTL_OFFSET             0x0048  /* MCU Bus Control */
#define TIVA_AUX_WUC_MCUBUSSTAT_OFFSET            0x004c  /* MCU Bus Status */
#define TIVA_AUX_WUC_AONCTLSTAT_OFFSET            0x0050  /* AON Domain Control Status */
#define TIVA_AUX_WUC_AUXIOLATCH_OFFSET            0x0054  /* AUX Input Output Latch */
#define TIVA_AUX_WUC_MODCLKEN1_OFFSET             0x005c  /* Module Clock Enable 1 */

/* AUX WUC Register Addresses ***************************************************************************************/

#define TIVA_AUX_WUC_MODCLKEN0                    (TIVA_AUX_WUC_BASE + TIVA_AUX_WUC_MODCLKEN0_OFFSET)
#define TIVA_AUX_WUC_PWROFFREQ                    (TIVA_AUX_WUC_BASE + TIVA_AUX_WUC_PWROFFREQ_OFFSET)
#define TIVA_AUX_WUC_PWRDWNREQ                    (TIVA_AUX_WUC_BASE + TIVA_AUX_WUC_PWRDWNREQ_OFFSET)
#define TIVA_AUX_WUC_PWRDWNACK                    (TIVA_AUX_WUC_BASE + TIVA_AUX_WUC_PWRDWNACK_OFFSET)
#define TIVA_AUX_WUC_CLKLFREQ                     (TIVA_AUX_WUC_BASE + TIVA_AUX_WUC_CLKLFREQ_OFFSET)
#define TIVA_AUX_WUC_CLKLFACK                     (TIVA_AUX_WUC_BASE + TIVA_AUX_WUC_CLKLFACK_OFFSET)
#define TIVA_AUX_WUC_WUEVFLAGS                    (TIVA_AUX_WUC_BASE + TIVA_AUX_WUC_WUEVFLAGS_OFFSET)
#define TIVA_AUX_WUC_WUEVCLR                      (TIVA_AUX_WUC_BASE + TIVA_AUX_WUC_WUEVCLR_OFFSET)
#define TIVA_AUX_WUC_ADCCLKCTL                    (TIVA_AUX_WUC_BASE + TIVA_AUX_WUC_ADCCLKCTL_OFFSET)
#define TIVA_AUX_WUC_TDCCLKCTL                    (TIVA_AUX_WUC_BASE + TIVA_AUX_WUC_TDCCLKCTL_OFFSET)
#define TIVA_AUX_WUC_REFCLKCTL                    (TIVA_AUX_WUC_BASE + TIVA_AUX_WUC_REFCLKCTL_OFFSET)
#define TIVA_AUX_WUC_RTCSUBSECINC0                (TIVA_AUX_WUC_BASE + TIVA_AUX_WUC_RTCSUBSECINC0_OFFSET)
#define TIVA_AUX_WUC_RTCSUBSECINC1                (TIVA_AUX_WUC_BASE + TIVA_AUX_WUC_RTCSUBSECINC1_OFFSET)
#define TIVA_AUX_WUC_RTCSUBSECINCCTL              (TIVA_AUX_WUC_BASE + TIVA_AUX_WUC_RTCSUBSECINCCTL_OFFSET)
#define TIVA_AUX_WUC_MCUBUSCTL                    (TIVA_AUX_WUC_BASE + TIVA_AUX_WUC_MCUBUSCTL_OFFSET)
#define TIVA_AUX_WUC_MCUBUSSTAT                   (TIVA_AUX_WUC_BASE + TIVA_AUX_WUC_MCUBUSSTAT_OFFSET)
#define TIVA_AUX_WUC_AONCTLSTAT                   (TIVA_AUX_WUC_BASE + TIVA_AUX_WUC_AONCTLSTAT_OFFSET)
#define TIVA_AUX_WUC_AUXIOLATCH                   (TIVA_AUX_WUC_BASE + TIVA_AUX_WUC_AUXIOLATCH_OFFSET)
#define TIVA_AUX_WUC_MODCLKEN1                    (TIVA_AUX_WUC_BASE + TIVA_AUX_WUC_MODCLKEN1_OFFSET)

/* AUX WUC Register Bitfield Definitions ****************************************************************************/

/* AUX_WUC_MODCLKEN0 */

#define AUX_WUC_MODCLKEN0_SMPH                    (1 << 0)  /* Bit 0:  Enable clock for AUX_SMPH */
#define AUX_WUC_MODCLKEN0_AIODIO0                 (1 << 1)  /* Bit 1:  Enable clock for AUX_AIODIO0 */
#define AUX_WUC_MODCLKEN0_AIODIO1                 (1 << 2)  /* Bit 2:  Enable clock for AUX_AIODIO1 */
#define AUX_WUC_MODCLKEN0_TIMER                   (1 << 3)  /* Bit 3:  Enable clock for AUX_TIMER */
#define AUX_WUC_MODCLKEN0_ANAIF                   (1 << 4)  /* Bit 4:  Enable clock for AUX_ANAIF */
#define AUX_WUC_MODCLKEN0_TDC                     (1 << 5)  /* Bit 5:  Enable clock for AUX_TDCIF */
#define AUX_WUC_MODCLKEN0_AUX_DDI0_OSC            (1 << 6)  /* Bit 6:  Enable clock for AUX_DDI0_OSC */
#define AUX_WUC_MODCLKEN0_AUX_ADI4                (1 << 7)  /* Bit 7:  Enable clock for AUX_ADI4 */

/* AUX_WUC_PWROFFREQ */

#define AUX_WUC_PWROFFREQ_REQ                     (1 << 0)  /* Bit 0:  Power off request */

/* AUX_WUC_PWRDWNREQ */

#define AUX_WUC_PWRDWNREQ_REQ                     (1 << 0)  /* Bit 0:  Power down request */

/* AUX_WUC_PWRDWNACK */

#define AUX_WUC_PWRDWNACK_ACK                     (1 << 0)  /* Bit 0: Power down acknowledgment */

/* AUX_WUC_CLKLFREQ */

#define AUX_WUC_CLKLFREQ_REQ                      (1 << 0)  /* Bit 0:  Low frequency request */

/* AUX_WUC_CLKLFACK */

#define AUX_WUC_CLKLFACK_ACK                      (1 << 0)  /* Bit 0:  Acknowledge low free frequency clock */

/* AUX_WUC_WUEVFLAGS */

#define AUX_WUC_WUEVFLAGS_AON_PROG_WU             (1 << 0)  /* Bit 0:  Pending event triggered by sources select in
                                                             *         AON_EVENT:AUXWUSEL.WU0_EV, WU1_EV, and WU2_EV */
#define AUX_WUC_WUEVFLAGS_AON_SW                  (1 << 1)  /* Bit 1:  Pending event triggered by write to
                                                             *         ON_WUC:AUXCTL.SWEV */
#define AUX_WUC_WUEVFLAGS_AON_RTC_CH2             (1 << 2)  /* Bit 2:  Pending event from AON_RTC_CH2 compare */

/* AUX_WUC_WUEVCLR */

#define AUX_WUC_WUEVCLR_AON_PROG_WU               (1 << 0)  /* Bit 0:  Clear WUEVFLAGS.AON_PROG_WU wake-up event */
#define AUX_WUC_WUEVCLR_AON_SW                    (1 << 1)  /* Bit 1:  Clear WUEVFLAGS.AON_SW wake-up event */
#define AUX_WUC_WUEVCLR_AON_RTC_CH2               (1 << 2)  /* Bit 2:  Clear WUEVFLAGS.AON_RTC_CH2 wake-up event */

/* AUX_WUC_ADCCLKCTL */

#define AUX_WUC_ADCCLKCTL_REQ                     (1 << 0)  /* Bit 0:  Enable ADC internal clock */
#define AUX_WUC_ADCCLKCTL_ACK                     (1 << 1)  /* Bit 1:  Acknowledge last value written to REQ */

/* AUX_WUC_TDCCLKCTL */

#define AUX_WUC_TDCCLKCTL_REQ                     (1 << 0)  /* Bit 0:  Enable TDC counter clock source */
#define AUX_WUC_TDCCLKCTL_ACK                     (1 << 1)  /* Bit 1:  Acknowledge last value written to REQ */

/* AUX_WUC_REFCLKCTL */

#define AUX_WUC_REFCLKCTL_REQ                     (1 << 0)  /* Bit 0:  Enable TDC reference clock source */
#define AUX_WUC_REFCLKCTL_ACK                     (1 << 1)  /* Bit 1:  Acknowledge last value written to REQ */

/* AUX_WUC_RTCSUBSECINC0 */

#define AUX_WUC_RTCSUBSECINC0_INC15_0_SHIFT       (0)        /* Bits 0-15: Bits 0-15 of RTC sub-second increment */
#define AUX_WUC_RTCSUBSECINC0_INC15_0_MASK        (0xffff << AUX_WUC_RTCSUBSECINC0_INC15_0_SHIFT)
#  define AUX_WUC_RTCSUBSECINC0_INC15_0(n)        ((uint32_t)(n) << AUX_WUC_RTCSUBSECINC0_INC15_0_SHIFT)

/* AUX_WUC_RTCSUBSECINC1 */

#define AUX_WUC_RTCSUBSECINC1_INC23_16_SHIFT      (0)        /* Bits 0-7: Bits 15-23 of RTC sub-second increment */
#define AUX_WUC_RTCSUBSECINC1_INC23_16_MASK       (0xff << AUX_WUC_RTCSUBSECINC1_INC23_16_SHIFT)
#  define AUX_WUC_RTCSUBSECINC1_INC23_16(n)       ((uint32_t)(n) << AUX_WUC_RTCSUBSECINC1_INC23_16_SHIFT)

/* AUX_WUC_RTCSUBSECINCCTL */

#define AUX_WUC_RTCSUBSECINCCTL_UPD_REQ           (1 << 0)  /* Bit 0:  New RTC sub-second increment available */
#define AUX_WUC_RTCSUBSECINCCTL_UPD_ACK           (1 << 1)  /* Bit 1:  Acknowledgment of UPD_REQ */

/* AUX_WUC_MCUBUSCTL */

#define AUX_WUC_MCUBUSCTL_DISCONNECT_REQ          (1 << 0)  /* Bit 0:  Request AUX domain disconnect from MCU domain */

/* AUX_WUC_MCUBUSSTAT */

#define AUX_WUC_MCUBUSSTAT_DISCONNECT_ACK         (1 << 0)  /* Bit 0:  Acknowledge bus disconnection request */
#define AUX_WUC_MCUBUSSTAT_DISCONNECTED           (1 << 1)  /* Bit 1:  AUX and MCU domain buses disconnected */

/* AUX_WUC_AONCTLSTAT */

#define AUX_WUC_AONCTLSTAT_SCE_RUN_EN             (1 << 0)  /* Bit 0:  Status of AON_WUC:AUX_CTL.SCE_RUN_EN */
#define AUX_WUC_AONCTLSTAT_AUX_FORCE_ON           (1 << 1)  /* Bit 1:  Status of AON_WUC:AUX_CTL.AUX_FORCE_ON */

/* AUX_WUC_AUXIOLATCH */

#define AUX_WUC_AUXIOLATCH_EN                     (1 << 0)  /* Bit 0:  Open AUX_AIODIO0/AUX_AIODIO1 signal latching */

/* AUX_WUC_MODCLKEN1 */

#define AUX_WUC_MODCLKEN1_SMPH                    (1 << 0)  /* Bit 0:  Enable clock for AUX_SMPH */
#define AUX_WUC_MODCLKEN1_AIODIO0                 (1 << 1)  /* Bit 1:  Enable clock for AUX_AIODIO0 */
#define AUX_WUC_MODCLKEN1_AIODIO1                 (1 << 2)  /* Bit 2:  Enable clock for AUX_AIODIO1 */
#define AUX_WUC_MODCLKEN1_TIMER                   (1 << 3)  /* Bit 3:  Enable clock for AUX_TIMER */
#define AUX_WUC_MODCLKEN1_ANAIF                   (1 << 4)  /* Bit 4:  Enable clock for AUX_ANAIF */
#define AUX_WUC_MODCLKEN1_AUX_DDI0_OSC            (1 << 6)  /* Bit 6:  Enable clock for AUX_DDI0_OSC */
#define AUX_WUC_MODCLKEN1_AUX_ADI4                (1 << 7)  /* Bit 7:  Enable clock for AUX_ADI4 */

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X0_CC13X0_AUX_WUC_H */
