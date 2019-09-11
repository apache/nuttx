/********************************************************************************************
 * arch/arm/src/samd2l2/hardware/saml_opamp.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   "Atmel SAM L21E / SAM L21G / SAM L21J Smart ARM-Based Microcontroller
 *   Datasheet", Atmel-42385C-SAML21_Datasheet_Preliminary-03/20/15
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

#ifndef __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAML_OPAMP_H
#define __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAML_OPAMP_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#ifdef CONFIG_ARCH_FAMILY_SAML21

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/
/* OPAMP register offsets ********************************************************************/

#define SAM_OPAMP_CTRLA_OFFSET     0x0000  /* Control A Register */
#define SAM_OPAMP_STATUS_OFFSET    0x0002  /* Status Register */
#define SAM_OPAMP_CTRL0_OFFSET     0x0004  /* OPAMP Control 0 Register */
#define SAM_OPAMP_CTRL1_OFFSET     0x0008  /* OPAMP Control 1 Register */
#define SAM_OPAMP_CTRL2_OFFSET     0x000c  /* OPAMP Control 2 Register */

/* OPAMP register addresses ******************************************************************/

#define SAM_OPAMP_CTRLA            (SAM_OPAMP_BASE+SAM_OPAMP_CTRLA_OFFSET)
#define SAM_OPAMP_STATUS           (SAM_OPAMP_BASE+SAM_OPAMP_STATUS_OFFSET)
#define SAM_OPAMP_CTRL0            (SAM_OPAMP_BASE+SAM_OPAMP_CTRL0_OFFSET)
#define SAM_OPAMP_CTRL1            (SAM_OPAMP_BASE+SAM_OPAMP_CTRL1_OFFSET)
#define SAM_OPAMP_CTRL2            (SAM_OPAMP_BASE+SAM_OPAMP_CTRL2_OFFSET)

/* OPAMP register bit definitions ************************************************************/

/* Control A Register */

#define OPAMP_CTRLA_SWRST            (1 << 0)  /* Bit 0:  Software reset */
#define OPAMP_CTRLA_ENABLE           (1 << 1)  /* Bit 1:  Enable */
#define OPAMP_CTRLA_LPMUX            (1 << 7)  /* Bit 7:  Low-power mux */

/* Status Register */

#define OPAMP_STATUS_READY0          (1 << 0)  /* Bit 0:  OPAMP 0 ready */
#define OPAMP_STATUS_READY1          (1 << 1)  /* Bit 1:  OPAMP 1 ready */
#define OPAMP_STATUS_READY2          (1 << 2)  /* Bit 2:  OPAMP 2 ready */

/* OPAMP Control 0-2 Register */

#define OPAMP_CTRL_ENABLE            (1 << 1)  /* Bit 1:  Operation amplifier enable */
#define OPAMP_CTRL_ANAOUT            (1 << 2)  /* Bit 2:  Analog output */
#define OPAMP_CTRL_BIAS_SHIFT        (3)  /* Bits 3-5: Bias selection */
#define OPAMP_CTRL_BIAS_MASK         (7 << OPAMP_CTRL_BIAS_SHIFT)
#  define OPAMP_CTRL_BIAS_MODE0      (0 << OPAMP_CTRL_BIAS_SHIFT) /* Minimum current, slowest mode */
#  define OPAMP_CTRL_BIAS_MODE1      (1 << OPAMP_CTRL_BIAS_SHIFT) /* Low current, slow */
#  define OPAMP_CTRL_BIAS_MODE2      (2 << OPAMP_CTRL_BIAS_SHIFT) /* High current, fast */
#  define OPAMP_CTRL_BIAS_MODE3      (3 << OPAMP_CTRL_BIAS_SHIFT) /* Maximum current, fastest mode */
#define OPAMP_CTRL_RUNSTDBY          (1 << 6)  /* Bit 6:  Run in standby */
#define OPAMP_CTRL_ONDEMAND          (1 << 7)  /* Bit 7:  On demand control */
#define OPAMP_CTRL_RES2OUT           (1 << 8)  /* Bit 8:  Resistor ladder to output */
#define OPAMP_CTRL_RES2VCC           (1 << 9)  /* Bit 9:  Resistor ladder to VCC */
#define OPAMP_CTRL_RES1EN            (1 << 10)  /* Bit 10:  Resistor 1 enable */
#define OPAMP_CTRL_RES1MUX_SHIFT     (11)  /* Bits 11-12: Resistor 1 mux */
#define OPAMP_CTRL_RES1MUX_MASK      (3 << OPAMP_CTRL_RES1MUX_MASK)
#  define OPAMP_CTRL_RES1MUX_OAxPOS   (0 << OPAMP_CTRL_RES1MUX_MASK) /* Positive input of OPAMPn, n=0,1,2 */
#  define OPAMP_CTRL_RES1MUX_OAxNEG   (1 << OPAMP_CTRL_RES1MUX_MASK) /* Negative input of OPAMPn, n=0,1,2 */
#  define OPAMP_CTRL_RES1MUX_DAC_0    (2 << OPAMP_CTRL_RES1MUX_MASK) /* DAC output, OPAMP0 */
#  define OPAMP_CTRL_RES1MUX_OA0OUT_1 (2 << OPAMP_CTRL_RES1MUX_MASK) /* OPAMP0 output, OPAMP1 */
#  define OPAMP_CTRL_RES1MUX_OA1OUT_2 (2 << OPAMP_CTRL_RES1MUX_MASK) /* OPAMP1 output, OPAMP2 */
#  define OPAMP_CTRL_RES1MUX_GND      (3 << OPAMP_CTRL_RES1MUX_MASK) /* Ground, OPAMPn, n=0,1,2 */
#define OPAMP_CTRL_POTMUX_SHIFT      (13)  /* Bits 13-15: Potentiometer selection */
#define OPAMP_CTRL_POTMUX_MASK       (7 << OPAMP_CTRL_POTMUX_SHIFT)
#  define OPAMP_CTRL_POTMUX_14R_2R   (0 << OPAMP_CTRL_POTMUX_SHIFT) /* Gain 1/7 */
#  define OPAMP_CTRL_POTMUX_12R_4R   (1 << OPAMP_CTRL_POTMUX_SHIFT) /* Gain 1/3 */
#  define OPAMP_CTRL_POTMUX_8R_8R    (2 << OPAMP_CTRL_POTMUX_SHIFT) /* Gain 1 */
#  define OPAMP_CTRL_POTMUX_6R_10R   (3 << OPAMP_CTRL_POTMUX_SHIFT) /* Gain 1+2/3 */
#  define OPAMP_CTRL_POTMUX_4R_12R   (4 << OPAMP_CTRL_POTMUX_SHIFT) /* Gain 3 */
#  define OPAMP_CTRL_POTMUX_3R_13R   (5 << OPAMP_CTRL_POTMUX_SHIFT) /* Gain 4+1/3 */
#  define OPAMP_CTRL_POTMUX_2R_14R   (6 << OPAMP_CTRL_POTMUX_SHIFT) /* Gain 7 */
#  define OPAMP_CTRL_POTMUX_R_15R    (7 << OPAMP_CTRL_POTMUX_SHIFT) /* Gain 15 */
#define OPAMP_CTRL_MUXPOS_SHIFT      (16)  /* Bits 16-18: Positive input mux selection */
#define OPAMP_CTRL_MUXPOS_MASK       (7 << OPAMP_CTRL_MUXPOS_SHIFT)
#  define OPAMP_CTRL_MUXPOS_OAxPOS   (0 << OPAMP_CTRL_MUXPOS_SHIFT) /* Positive I/O pin, OPAMPn, n=0,1,2 */
#  define OPAMP_CTRL_MUXPOS_OAxTAP   (1 << OPAMP_CTRL_MUXPOS_SHIFT) /* Resister ladder x taps, OPAMPn, n=0,1,2 */
#  define OPAMP_CTRL_MUXPOS_DAC_0    (2 << OPAMP_CTRL_MUXPOS_SHIFT) /* DAC output, OPAMP0 */
#  define OPAMP_CTRL_MUXPOS_OA0OUT_1 (2 << OPAMP_CTRL_MUXPOS_SHIFT) /* OPAMP0 output, OPAMP1 */
#  define OPAMP_CTRL_MUXPOS_OA1OUT_2 (2 << OPAMP_CTRL_MUXPOS_SHIFT) /* OPAMP1 output, OPAMP2 */
#  define OPAMP_CTRL_MUXPOS_GND      (3 << OPAMP_CTRL_MUXPOS_SHIFT) /* Ground, OPAMPn, n=0,1,2 */
#  define OPAMP_CTRL_MUXPOS_OA0POS_2 (4 << OPAMP_CTRL_MUXPOS_SHIFT) /* Positive I/O pin OPA0, OPAMP2 */
#  define OPAMP_CTRL_MUXPOS_OA1POS_2 (5 << OPAMP_CTRL_MUXPOS_SHIFT) /* Positive I/O pin OPA1, OPAMP2 */
#  define OPAMP_CTRL_MUXPOS_OA0TAP_2 (6 << OPAMP_CTRL_MUXPOS_SHIFT) /* Resistor ladder 0 taps, OPAMP2 */
#define OPAMP_CTRL_MUXNEG_SHIFT      (20)  /* Bits 20-22: Negative input mux selection */
#define OPAMP_CTRL_MUXNEG_MASK       (7 << OPAMP_CTRL_MUXNEG_SHIFT)
#  define OPAMP_CTRL_MUXNEG_OAxNEG   (0 << OPAMP_CTRL_MUXNEG_SHIFT) /* Negative I/O pin OPAMP n, n=0,1,2 */
#  define OPAMP_CTRL_MUXNEG_OAxTAP   (1 << OPAMP_CTRL_MUXNEG_SHIFT) /* Resister laddr x tap OPAMP n, n=0,1,2 */
#  define OPAMP_CTRL_MUXNEG_OAxOUT   (2 << OPAMP_CTRL_MUXNEG_SHIFT) /* OPAMPn output, n=0,1,2 */
#  define OPAMP_CTRL_MUXNEG_DAC_01   (3 << OPAMP_CTRL_MUXNEG_SHIFT) /* DAC output, OPAMP0,1 */
#  define OPAMP_CTRL_MUXNEG_OA0NEG_2 (3 << OPAMP_CTRL_MUXNEG_SHIFT) /* Negative I/O pin OPA0, OPAMP2 */
#  define OPAMP_CTRL_MUXNEG_OA1NEG_2 (4 << OPAMP_CTRL_MUXNEG_SHIFT) /* Negative I/O OPA1, OPAMP2 */
#  define OPAMP_CTRL_MUXNEG_DAC_2    (5 << OPAMP_CTRL_MUXNEG_SHIFT) /* DAC output, OPAMP2 */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* CONFIG_ARCH_FAMILY_SAML21 */
#endif /* __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAML_OPAMP_H */
