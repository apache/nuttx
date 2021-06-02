/****************************************************************************
 * arch/arm/src/samd2l2/hardware/samd_tcc.h
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

/* References:
 *   "Microchip SAMD21 datasheet"
 */

#ifndef __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD_TCC_H
#define __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD_TCC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#ifdef CONFIG_ARCH_FAMILY_SAMD21

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* TCC register offsets *****************************************************/

#define SAM_TCC_CTRLA_OFFSET          0x0000  /* Control A register */
#define SAM_TCC_CTRLBCLR_OFFSET       0x0004  /* Control B clear register */
#define SAM_TCC_CTRLBSET_OFFSET       0x0005  /* Control B clear register */
#define SAM_TCC_SYNCBUSY_OFFSET       0x0008  /* Sync Busy register */
#define SAM_TCC_FCTRLA_OFFSET         0x000C  /* Fault control A register */
#define SAM_TCC_FCTRLB_OFFSET         0x0010  /* Fault control B register */
#define SAM_TCC_WEXCTRL_OFFSET        0x0014  /* Waveform extension control register */
#define SAM_TCC_DRVCTRL_OFFSET        0x0018  /* Event control register */
#define SAM_TCC_DBGCTRL_OFFSET        0x001E  /* Debug control register */
#define SAM_TCC_EVCTRL_OFFSET         0x0020  /* Event control register */
#define SAM_TCC_INTENCLR_OFFSET       0x0024  /* Interrupt enable clear register */
#define SAM_TCC_INTENSET_OFFSET       0x0028  /* Interrupt enable set register */
#define SAM_TCC_INTFLAG_OFFSET        0x002C  /* Interrupt flag register */
#define SAM_TCC_STATUS_OFFSET         0x0030  /* Status register */
#define SAM_TCC_COUNT_OFFSET          0x0034  /* Count register */
#define SAM_TCC_PATT_OFFSET           0x0038  /* Pattern register */
#define SAM_TCC_WAVE_OFFSET           0x003C  /* Waveform register */
#define SAM_TCC_PER_OFFSET            0x0040  /* Period register */
#define SAM_TCC_CC0_OFFSET            0x0044  /* Capture Compare 0 register */
#define SAM_TCC_CC1_OFFSET            0x0048  /* Capture Compare 1 register */
#define SAM_TCC_CC2_OFFSET            0x004C  /* Capture Compare 2 register */
#define SAM_TCC_CC3_OFFSET            0x0050  /* Capture Compare 3 register */
#define SAM_TCC_PATTB_OFFSET          0x0064  /* Capture Compare 3 register */
#define SAM_TCC_WAVEB_OFFSET          0x0068  /* Capture Compare 3 register */
#define SAM_TCC_PERB_OFFSET           0x006C  /* Capture Compare 3 register */
#define SAM_TCC_CCB0_OFFSET           0x0070  /* Capture Compare B0 register */
#define SAM_TCC_CCB1_OFFSET           0x0074  /* Capture Compare B0 register */
#define SAM_TCC_CCB2_OFFSET           0x0078  /* Capture Compare B0 register */
#define SAM_TCC_CCB3_OFFSET           0x007C  /* Capture Compare B0 register */

/* TC register addresses ****************************************************/

#define SAM_TCC0_CTRLA                (SAM_TCC0_BASE+SAM_TCC_CTRLA_OFFSET)
#define SAM_TCC0_CTRLBCLR             (SAM_TCC0_BASE+SAM_TCC_CTRLBCLR_OFFSET)
#define SAM_TCC0_CTRLBSET             (SAM_TCC0_BASE+SAM_TCC_CTRLBSET_OFFSET)
#define SAM_TCC0_SYNCBUSY             (SAM_TCC0_BASE+SAM_TCC_SYNCBUSY_OFFSET)
#define SAM_TCC0_FCTRLA               (SAM_TCC0_BASE+SAM_TCC_FCTRLA_OFFSET)
#define SAM_TCC0_FCTRLB               (SAM_TCC0_BASE+SAM_TCC_FCTRLB_OFFSET)
#define SAM_TCC0_WEXCTRL              (SAM_TCC0_BASE+SAM_TCC_WEXCTRL_OFFSET)
#define SAM_TCC0_DRVCTRL              (SAM_TCC0_BASE+SAM_TCC_DRVCTRL_OFFSET)
#define SAM_TCC0_DBGCTRL              (SAM_TCC0_BASE+SAM_TCC_DBGCTRL_OFFSET)
#define SAM_TCC0_EVCTRL               (SAM_TCC0_BASE+SAM_TCC_EVCTRL_OFFSET)
#define SAM_TCC0_INTENCLR             (SAM_TCC0_BASE+SAM_TCC_INTENCLR_OFFSET)
#define SAM_TCC0_INTENSET             (SAM_TCC0_BASE+SAM_TCC_INTENSET_OFFSET)
#define SAM_TCC0_INTFLAG              (SAM_TCC0_BASE+SAM_TCC_INTFLAG_OFFSET)
#define SAM_TCC0_STATUS               (SAM_TCC0_BASE+SAM_TCC_STATUS_OFFSET)
#define SAM_TCC0_COUNT                (SAM_TCC0_BASE+SAM_TCC_COUNT_OFFSET)
#define SAM_TCC0_PATT                 (SAM_TCC0_BASE+SAM_TCC_PATT_OFFSET)
#define SAM_TCC0_WAVE                 (SAM_TCC0_BASE+SAM_TCC_WAVE_OFFSET)
#define SAM_TCC0_PER                  (SAM_TCC0_BASE+SAM_TCC_PER_OFFSET)
#define SAM_TCC0_CC0                  (SAM_TCC0_BASE+SAM_TCC_CC0_OFFSET)
#define SAM_TCC0_CC1                  (SAM_TCC0_BASE+SAM_TCC_CC1_OFFSET)
#define SAM_TCC0_CC2                  (SAM_TCC0_BASE+SAM_TCC_CC2_OFFSET)
#define SAM_TCC0_CC3                  (SAM_TCC0_BASE+SAM_TCC_CC3_OFFSET)
#define SAM_TCC0_PATTB                (SAM_TCC0_BASE+SAM_TCC_PATTB_OFFSET)
#define SAM_TCC0_WAVEB                (SAM_TCC0_BASE+SAM_TCC_WAVEB_OFFSET)
#define SAM_TCC0_PERB                 (SAM_TCC0_BASE+SAM_TCC_PERB_OFFSET)
#define SAM_TCC0_CCB0                 (SAM_TCC0_BASE+SAM_TCC_CCB0_OFFSET)
#define SAM_TCC0_CCB1                 (SAM_TCC0_BASE+SAM_TCC_CCB1_OFFSET)
#define SAM_TCC0_CCB2                 (SAM_TCC0_BASE+SAM_TCC_CCB2_OFFSET)
#define SAM_TCC0_CCB3                 (SAM_TCC0_BASE+SAM_TCC_CCB3_OFFSET)

#define SAM_TCC1_CTRLA                (SAM_TCC1_BASE+SAM_TCC_CTRLA_OFFSET)
#define SAM_TCC1_CTRLBCLR             (SAM_TCC1_BASE+SAM_TCC_CTRLBCLR_OFFSET)
#define SAM_TCC1_CTRLBSET             (SAM_TCC1_BASE+SAM_TCC_CTRLBSET_OFFSET)
#define SAM_TCC1_SYNCBUSY             (SAM_TCC1_BASE+SAM_TCC_SYNCBUSY_OFFSET)
#define SAM_TCC1_FCTRLA               (SAM_TCC1_BASE+SAM_TCC_FCTRLA_OFFSET)
#define SAM_TCC1_FCTRLB               (SAM_TCC1_BASE+SAM_TCC_FCTRLB_OFFSET)
#define SAM_TCC1_WEXCTRL              (SAM_TCC1_BASE+SAM_TCC_WEXCTRL_OFFSET)
#define SAM_TCC1_DRVCTRL              (SAM_TCC1_BASE+SAM_TCC_DRVCTRL_OFFSET)
#define SAM_TCC1_DBGCTRL              (SAM_TCC1_BASE+SAM_TCC_DBGCTRL_OFFSET)
#define SAM_TCC1_EVCTRL               (SAM_TCC1_BASE+SAM_TCC_EVCTRL_OFFSET)
#define SAM_TCC1_INTENCLR             (SAM_TCC1_BASE+SAM_TCC_INTENCLR_OFFSET)
#define SAM_TCC1_INTENSET             (SAM_TCC1_BASE+SAM_TCC_INTENSET_OFFSET)
#define SAM_TCC1_INTFLAG              (SAM_TCC1_BASE+SAM_TCC_INTFLAG_OFFSET)
#define SAM_TCC1_STATUS               (SAM_TCC1_BASE+SAM_TCC_STATUS_OFFSET)
#define SAM_TCC1_COUNT                (SAM_TCC1_BASE+SAM_TCC_COUNT_OFFSET)
#define SAM_TCC1_PATT                 (SAM_TCC1_BASE+SAM_TCC_PATT_OFFSET)
#define SAM_TCC1_WAVE                 (SAM_TCC1_BASE+SAM_TCC_WAVE_OFFSET)
#define SAM_TCC1_PER                  (SAM_TCC1_BASE+SAM_TCC_PER_OFFSET)
#define SAM_TCC1_CC0                  (SAM_TCC1_BASE+SAM_TCC_CC0_OFFSET)
#define SAM_TCC1_CC1                  (SAM_TCC1_BASE+SAM_TCC_CC1_OFFSET)
#define SAM_TCC1_CC2                  (SAM_TCC1_BASE+SAM_TCC_CC2_OFFSET)
#define SAM_TCC1_CC3                  (SAM_TCC1_BASE+SAM_TCC_CC3_OFFSET)
#define SAM_TCC1_PATTB                (SAM_TCC1_BASE+SAM_TCC_PATTB_OFFSET)
#define SAM_TCC1_WAVEB                (SAM_TCC1_BASE+SAM_TCC_WAVEB_OFFSET)
#define SAM_TCC1_PERB                 (SAM_TCC1_BASE+SAM_TCC_PERB_OFFSET)
#define SAM_TCC1_CCB0                 (SAM_TCC1_BASE+SAM_TCC_CCB0_OFFSET)
#define SAM_TCC1_CCB1                 (SAM_TCC1_BASE+SAM_TCC_CCB1_OFFSET)
#define SAM_TCC1_CCB2                 (SAM_TCC1_BASE+SAM_TCC_CCB2_OFFSET)
#define SAM_TCC1_CCB3                 (SAM_TCC1_BASE+SAM_TCC_CCB3_OFFSET)

#define SAM_TCC2_CTRLA                (SAM_TCC2_BASE+SAM_TCC_CTRLA_OFFSET)
#define SAM_TCC2_CTRLBCLR             (SAM_TCC2_BASE+SAM_TCC_CTRLBCLR_OFFSET)
#define SAM_TCC2_CTRLBSET             (SAM_TCC2_BASE+SAM_TCC_CTRLBSET_OFFSET)
#define SAM_TCC2_SYNCBUSY             (SAM_TCC2_BASE+SAM_TCC_SYNCBUSY_OFFSET)
#define SAM_TCC2_FCTRLA               (SAM_TCC2_BASE+SAM_TCC_FCTRLA_OFFSET)
#define SAM_TCC2_FCTRLB               (SAM_TCC2_BASE+SAM_TCC_FCTRLB_OFFSET)
#define SAM_TCC2_WEXCTRL              (SAM_TCC2_BASE+SAM_TCC_WEXCTRL_OFFSET)
#define SAM_TCC2_DRVCTRL              (SAM_TCC2_BASE+SAM_TCC_DRVCTRL_OFFSET)
#define SAM_TCC2_DBGCTRL              (SAM_TCC2_BASE+SAM_TCC_DBGCTRL_OFFSET)
#define SAM_TCC2_EVCTRL               (SAM_TCC2_BASE+SAM_TCC_EVCTRL_OFFSET)
#define SAM_TCC2_INTENCLR             (SAM_TCC2_BASE+SAM_TCC_INTENCLR_OFFSET)
#define SAM_TCC2_INTENSET             (SAM_TCC2_BASE+SAM_TCC_INTENSET_OFFSET)
#define SAM_TCC2_INTFLAG              (SAM_TCC2_BASE+SAM_TCC_INTFLAG_OFFSET)
#define SAM_TCC2_STATUS               (SAM_TCC2_BASE+SAM_TCC_STATUS_OFFSET)
#define SAM_TCC2_COUNT                (SAM_TCC2_BASE+SAM_TCC_COUNT_OFFSET)
#define SAM_TCC2_PATT                 (SAM_TCC2_BASE+SAM_TCC_PATT_OFFSET)
#define SAM_TCC2_WAVE                 (SAM_TCC2_BASE+SAM_TCC_WAVE_OFFSET)
#define SAM_TCC2_PER                  (SAM_TCC2_BASE+SAM_TCC_PER_OFFSET)
#define SAM_TCC2_CC0                  (SAM_TCC2_BASE+SAM_TCC_CC0_OFFSET)
#define SAM_TCC2_CC1                  (SAM_TCC2_BASE+SAM_TCC_CC1_OFFSET)
#define SAM_TCC2_CC2                  (SAM_TCC2_BASE+SAM_TCC_CC2_OFFSET)
#define SAM_TCC2_CC3                  (SAM_TCC2_BASE+SAM_TCC_CC3_OFFSET)
#define SAM_TCC2_PATTB                (SAM_TCC2_BASE+SAM_TCC_PATTB_OFFSET)
#define SAM_TCC2_WAVEB                (SAM_TCC2_BASE+SAM_TCC_WAVEB_OFFSET)
#define SAM_TCC2_PERB                 (SAM_TCC2_BASE+SAM_TCC_PERB_OFFSET)
#define SAM_TCC2_CCB0                 (SAM_TCC2_BASE+SAM_TCC_CCB0_OFFSET)
#define SAM_TCC2_CCB1                 (SAM_TCC2_BASE+SAM_TCC_CCB1_OFFSET)
#define SAM_TCC2_CCB2                 (SAM_TCC2_BASE+SAM_TCC_CCB2_OFFSET)
#define SAM_TCC2_CCB3                 (SAM_TCC2_BASE+SAM_TCC_CCB3_OFFSET)

/* TC register bit definitions **********************************************/

/* Control A register */

#define TCC_CTRLA_SWRST               (1 << 0)  /* Bit 0:  Software reset */
#define TCC_CTRLA_ENABLE              (1 << 1)  /* Bit 1:  Enable */
#define TCC_CTRLA_RES_SHIFT           (5)
#define TCC_CTRLA_RES_MASK            (3 << TCC_CTRLA_RES_SHIFT)
#  define TCC_CTRLA_RES_NONE          (0 << TCC_CTRLA_RES_SHIFT)
#  define TCC_CTRLA_RES_DITH4         (1 << TCC_CTRLA_RES_SHIFT)
#  define TCC_CTRLA_RES_DITH5         (2 << TCC_CTRLA_RES_SHIFT)
#  define TCC_CTRLA_RES_DITH6         (3 << TCC_CTRLA_RES_SHIFT)
#define TCC_CTRLA_PRESCALER_SHIFT     (8)
#define TCC_CTRLA_PRESCALER_MASK      (7 << TCC_CTRLA_PRESCALER_SHIFT)
#  define TCC_CTRLA_PRESCALER_DIV1    (0 << TCC_CTRLA_PRESCALER_SHIFT)
#  define TCC_CTRLA_PRESCALER_DIV2    (1 << TCC_CTRLA_PRESCALER_SHIFT)
#  define TCC_CTRLA_PRESCALER_DIV4    (2 << TCC_CTRLA_PRESCALER_SHIFT)
#  define TCC_CTRLA_PRESCALER_DIV8    (3 << TCC_CTRLA_PRESCALER_SHIFT)
#  define TCC_CTRLA_PRESCALER_DIV16   (4 << TCC_CTRLA_PRESCALER_SHIFT)
#  define TCC_CTRLA_PRESCALER_DIV64   (5 << TCC_CTRLA_PRESCALER_SHIFT)
#  define TCC_CTRLA_PRESCALER_DIV256  (6 << TCC_CTRLA_PRESCALER_SHIFT)
#  define TCC_CTRLA_PRESCALER_DIV1024 (7 << TCC_CTRLA_PRESCALER_SHIFT)
#define TCC_CTRLA_RUNSTDBY            (1 << 11)
#define TCC_CTRLA_PRESCSYNC_SHIFT     (12)
#define TCC_CTRLA_PRESCSYNC_MASK      (3 << TCC_CTRLA_PRESCSYNC_SHIFT)
#  define TCC_CTRLA_PRESCSYNC_GCLK    (0 << TCC_CTRLA_PRESCSYNC_SHIFT)
#  define TCC_CTRLA_PRESCSYNC_PRESC   (1 << TCC_CTRLA_PRESCSYNC_SHIFT)
#  define TCC_CTRLA_PRESCSYNC_RESYNC  (2 << TCC_CTRLA_PRESCSYNC_SHIFT)
#define TCC_CTRLA_ALOCK               (1 << 14)
#define TCC_CTRLA_CPTEN0              (1 << 24)
#define TCC_CTRLA_CPTEN1              (1 << 25)
#define TCC_CTRLA_CPTEN2              (1 << 26)
#define TCC_CTRLA_CPTEN3              (1 << 27)

/* Control B Set/Clear register */

#define TCC_CTRLB_DIR                 (1 << 0)
#define TCC_CTRLB_LUPD                (1 << 1)
#define TCC_CTRLB_ONESHOT             (1 << 2)
#define TCC_CTRLB_IDXCMD_SHIFT        (3)
#define TCC_CTRLB_IDXCMD_MASK         (3 << TCC_CTRLB_IDXCMD_SHIFT)
#  define TCC_CTRLB_IDXCMD_DISABLE    (0 << TCC_CTRLB_IDXCMD_SHIFT)
#  define TCC_CTRLB_IDXCMD_SET        (1 << TCC_CTRLB_IDXCMD_SHIFT)
#  define TCC_CTRLB_IDXCMD_CLEAR      (2 << TCC_CTRLB_IDXCMD_SHIFT)
#  define TCC_CTRLB_IDXCMD_HOLD       (3 << TCC_CTRLB_IDXCMD_SHIFT)
#define TCC_CTRLB_CMD_SHIFT           (6)
#define TCC_CTRLB_CMD_MASK            (7 << TCC_CTRLB_CMD_SHIFT)
#  define TCC_CTRLB_CMD_NONE          (0 << TCC_CTRLB_CMD_SHIFT)
#  define TCC_CTRLB_CMD_RETRIGGER     (1 << TCC_CTRLB_CMD_SHIFT)
#  define TCC_CTRLB_CMD_STOP          (2 << TCC_CTRLB_CMD_SHIFT)
#  define TCC_CTRLB_CMD_UPDATE        (3 << TCC_CTRLB_CMD_SHIFT)
#  define TCC_CTRLB_CMD_READSYNC      (4 << TCC_CTRLB_CMD_SHIFT)

/* Sync Busy register */

#define TCC_SYNCBUSY_SWRST            (1 << 0)
#define TCC_SYNCBUSY_ENABLE           (1 << 1)
#define TCC_SYNCBUSY_CTRLB            (1 << 2)
#define TCC_SYNCBUSY_STATUS           (1 << 3)
#define TCC_SYNCBUSY_COUNT            (1 << 4)
#define TCC_SYNCBUSY_PATT             (1 << 5)
#define TCC_SYNCBUSY_WAVE             (1 << 6)
#define TCC_SYNCBUSY_PER              (1 << 7)
#define TCC_SYNCBUSY_CC0              (1 << 8)
#define TCC_SYNCBUSY_CC1              (1 << 9)
#define TCC_SYNCBUSY_CC2              (1 << 10)
#define TCC_SYNCBUSY_CC3              (1 << 11)
#define TCC_SYNCBUSY_PATTB            (1 << 16)
#define TCC_SYNCBUSY_WAVEB            (1 << 17)
#define TCC_SYNCBUSY_PERB             (1 << 18)
#define TCC_SYNCBUSY_CCB0             (1 << 19)
#define TCC_SYNCBUSY_CCB1             (1 << 20)
#define TCC_SYNCBUSY_CCB2             (1 << 21)
#define TCC_SYNCBUSY_CCB3             (1 << 22)

/* Fault Control A and B */

#define TCC_FCTRL_SRC_SHIFT           (0)
#define TCC_FCTRL_SRC_MASK            (3 << TCC_FCTRL_SRC_SHIFT)
#  define TCC_FCTRL_SRC_DISABLE       (0 << TCC_FCTRL_SRC_SHIFT)
#  define TCC_FCTRL_SRC_ENABLE        (1 << TCC_FCTRL_SRC_SHIFT)
#  define TCC_FCTRL_SRC_INVERT        (2 << TCC_FCTRL_SRC_SHIFT)
#  define TCC_FCTRL_SRC_ALTFAULT      (3 << TCC_FCTRL_SRC_SHIFT)
#define TCC_FCTRL_KEEP                (1 << 3)
#define TCC_FCTRL_QUAL                (1 << 4)
#define TCC_FCTRL_BLANK_SHIFT         (5)
#define TCC_FCTRL_BLANK_MASK          (3 << TCC_FCTRL_BLANK_SHIFT)
#  define TCC_FCTRL_BLANK_START       (0 << TCC_FCTRL_BLANK_SHIFT)
#  define TCC_FCTRL_BLANK_RISE        (1 << TCC_FCTRL_BLANK_SHIFT)
#  define TCC_FCTRL_BLANK_FALL        (2 << TCC_FCTRL_BLANK_SHIFT)
#  define TCC_FCTRL_BLANK_BOTH        (3 << TCC_FCTRL_BLANK_SHIFT)
#define TCC_FCTRL_RESTART             (1 << 7)
#define TCC_FCTRL_HALT_SHIFT          (8)
#define TCC_FCTRL_HALT_MASK           (3 << TCC_FCTRL_HALT_SHIFT)
#  define TCC_FCTRL_HALT_DISABLE      (0 << TCC_FCTRL_HALT_SHIFT)
#  define TCC_FCTRL_HALT_HW           (1 << TCC_FCTRL_HALT_SHIFT)
#  define TCC_FCTRL_HALT_SW           (2 << TCC_FCTRL_HALT_SHIFT)
#  define TCC_FCTRL_HALT_NR           (3 << TCC_FCTRL_HALT_SHIFT)
#define TCC_FCTRL_CHSEL_SHIFT         (10)
#define TCC_FCTRL_CHSEL_MASK          (3 << TCC_FCTRL_CHSEL_SHIFT)
#  define TCC_FCTRL_CHSEL_CC0         (0 << TCC_FCTRL_CHSEL_SHIFT)
#  define TCC_FCTRL_CHSEL_CC1         (1 << TCC_FCTRL_CHSEL_SHIFT)
#  define TCC_FCTRL_CHSEL_CC2         (2 << TCC_FCTRL_CHSEL_SHIFT)
#  define TCC_FCTRL_CHSEL_CC3         (3 << TCC_FCTRL_CHSEL_SHIFT)
#define TCC_FCTRL_CAPTURE_SHIFT       (12)
#define TCC_FCTRL_CAPTURE_MASK        (7 << TCC_FCTRL_CAPTURE_SHIFT)
#  define TCC_FCTRL_CAPTURE_DISABLE   (0 << TCC_FCTRL_CAPTURE_SHIFT)
#  define TCC_FCTRL_CAPTURE_CAPT      (1 << TCC_FCTRL_CAPTURE_SHIFT)
#  define TCC_FCTRL_CAPTURE_CAPTMIN   (2 << TCC_FCTRL_CAPTURE_SHIFT)
#  define TCC_FCTRL_CAPTURE_CAPTMAX   (3 << TCC_FCTRL_CAPTURE_SHIFT)
#  define TCC_FCTRL_CAPTURE_LOCMIN    (4 << TCC_FCTRL_CAPTURE_SHIFT)
#  define TCC_FCTRL_CAPTURE_LOCMAX    (5 << TCC_FCTRL_CAPTURE_SHIFT)
#  define TCC_FCTRL_CAPTURE_DERIV0    (6 << TCC_FCTRL_CAPTURE_SHIFT)
#define TCC_FCTRL_BLANKVAL_SHIFT      (16)
#define TCC_FCTRL_BLANKVAL_MASK       (0xff << TCC_FCTRL_BLANKVAL_SHIFT)
#define TCC_FCTRL_FILTERVAL_SHIFT     (24)
#define TCC_FCTRL_FILTERVAL_MASK      (0xf << TCC_FCTRL_FILTERVAL_SHIFT)

/* Waveform Extension Control register */

#define TCC_WEXCTRL_OTMX_SHIFT        (0)
#define TCC_WEXCTRL_OTMX_MASK         (3 << TCC_WEXCTRL_OTMX_SHIFT)
#define TCC_WEXCTRL_DTIEN0            (1 << 8)
#define TCC_WEXCTRL_DTIEN1            (1 << 9)
#define TCC_WEXCTRL_DTIEN2            (1 << 10)
#define TCC_WEXCTRL_DTIEN3            (1 << 11)
#define TCC_WEXCTRL_DTLS_SHIFT        (16)
#define TCC_WEXCTRL_DTLS_MASK         (0xff << TCC_WEXCTRL_DTLS_SHIFT)
#define TCC_WEXCTRL_DTHS_SHIFT        (24)
#define TCC_WEXCTRL_DTHS_MASK         (0xff << TCC_WEXCTRL_DTHS_SHIFT)

/* Driver Control register */

#define TCC_DRVCTRL_NRE(n)            (1 << n)
#define TCC_DRVCTRL_NRV(n)            (1 << (8+n))
#define TCC_DRVCTRL_INVEN(n)          (1 << (16+n))
#define TCC_DRVCTRL_FILTERVAL0_SHIFT  (24)
#define TCC_DRVCTRL_FILTERVAL0_MASK   (0xf << TCC_DRVCTRL_FILTERVAL0_SHIFT)
#define TCC_DRVCTRL_FILTERVAL1_SHIFT  (28)
#define TCC_DRVCTRL_FILTERVAL1_MASK   (0xf << TCC_DRVCTRL_FILTERVAL1_SHIFT)

/* Debug control register */

#define TCC_DBGCTRL_DBGRUN            (1 << 0)
#define TCC_DBGCTRL_FDDBD             (1 << 2)

/* Event control register */

#define TCC_EVCTRL_EVACT0_SHIFT        (0)
#define TCC_EVCTRL_EVACT0_MASK         (7 << TCC_EVCTRL_EVACT0_SHIFT)
#  define TCC_EVCTRL_EVACT0_OFF        (0 << TCC_EVCTRL_EVACT0_SHIFT)
#  define TCC_EVCTRL_EVACT0_RETRIGGER  (1 << TCC_EVCTRL_EVACT0_SHIFT)
#  define TCC_EVCTRL_EVACT0_COUNTEV    (2 << TCC_EVCTRL_EVACT0_SHIFT)
#  define TCC_EVCTRL_EVACT0_START      (3 << TCC_EVCTRL_EVACT0_SHIFT)
#  define TCC_EVCTRL_EVACT0_INC        (4 << TCC_EVCTRL_EVACT0_SHIFT)
#  define TCC_EVCTRL_EVACT0_COUNT      (5 << TCC_EVCTRL_EVACT0_SHIFT)
#  define TCC_EVCTRL_EVACT0_FAULT      (7 << TCC_EVCTRL_EVACT0_SHIFT)
#define TCC_EVCTRL_EVACT1_SHIFT        (3)
#define TCC_EVCTRL_EVACT1_MASK         (7 << TCC_EVCTRL_EVACT1_SHIFT)
#  define TCC_EVCTRL_EVACT1_OFF        (0 << TCC_EVCTRL_EVACT1_SHIFT)
#  define TCC_EVCTRL_EVACT1_RETRIGGER  (1 << TCC_EVCTRL_EVACT1_SHIFT)
#  define TCC_EVCTRL_EVACT1_DIR        (2 << TCC_EVCTRL_EVACT1_SHIFT)
#  define TCC_EVCTRL_EVACT1_STOP       (3 << TCC_EVCTRL_EVACT1_SHIFT)
#  define TCC_EVCTRL_EVACT1_DEC        (4 << TCC_EVCTRL_EVACT1_SHIFT)
#  define TCC_EVCTRL_EVACT1_PPW        (5 << TCC_EVCTRL_EVACT1_SHIFT)
#  define TCC_EVCTRL_EVACT1_PWP        (6 << TCC_EVCTRL_EVACT1_SHIFT)
#  define TCC_EVCTRL_EVACT1_FAULT      (7 << TCC_EVCTRL_EVACT1_SHIFT)
#define TCC_EVCTRL_CNTSEL_SHIFT        (6)
#define TCC_EVCTRL_CNTSEL_MASK         (3 << TCC_EVCTRL_CNTSEL_SHIFT)
#  define TCC_EVCTRL_CNTSEL_BEGIN      (0 << TCC_EVCTRL_CNTSEL_SHIFT)
#  define TCC_EVCTRL_CNTSEL_END        (1 << TCC_EVCTRL_CNTSEL_SHIFT)
#  define TCC_EVCTRL_CNTSEL_BETWEEN    (2 << TCC_EVCTRL_CNTSEL_SHIFT)
#  define TCC_EVCTRL_CNTSEL_BOUNDARY   (3 << TCC_EVCTRL_CNTSEL_SHIFT)
#define TCC_EVCTRL_OVFEO               (1 << 8)
#define TCC_EVCTRL_TRGEO               (1 << 9)
#define TCC_EVCTRL_CNTEO               (1 << 10)
#define TCC_EVCTRL_TCINV0              (1 << 12)
#define TCC_EVCTRL_TCINV1              (1 << 13)
#define TCC_EVCTRL_TCEI0               (1 << 14)
#define TCC_EVCTRL_TCEI1               (1 << 15)
#define TCC_EVCTRL_MCEI0               (1 << 16)
#define TCC_EVCTRL_MCEI1               (1 << 17)
#define TCC_EVCTRL_MCEI2               (1 << 18)
#define TCC_EVCTRL_MCEI3               (1 << 19)
#define TCC_EVCTRL_MCEO0               (1 << 24)
#define TCC_EVCTRL_MCEO1               (1 << 25)
#define TCC_EVCTRL_MCEO2               (1 << 26)
#define TCC_EVCTRL_MCEO3               (1 << 27)

/* Interrupt register bits */

#define TCC_INT_OVF                   (1 << 0)
#define TCC_INT_TRG                   (1 << 1)
#define TCC_INT_CNT                   (1 << 2)
#define TCC_INT_ERR                   (1 << 3)
#define TCC_INT_DFS                   (1 << 11)
#define TCC_INT_FAULTA                (1 << 12)
#define TCC_INT_FAULTB                (1 << 13)
#define TCC_INT_FAULT0                (1 << 14)
#define TCC_INT_FAULT1                (1 << 15)
#define TCC_INT_MC0                   (1 << 16)
#define TCC_INT_MC1                   (1 << 17)
#define TCC_INT_MC2                   (1 << 18)
#define TCC_INT_MC3                   (1 << 19)

/* Status register */

#define TCC_STATUS_STOP               (1 << 0)
#define TCC_STATUS_IDX                (1 << 1)
#define TCC_STATUS_DFS                (1 << 3)
#define TCC_STATUS_PATTBV             (1 << 5)
#define TCC_STATUS_WAVEBV             (1 << 6)
#define TCC_STATUS_PERBV              (1 << 7)
#define TCC_STATUS_FAULTAIN           (1 << 8)
#define TCC_STATUS_FAULTBIN           (1 << 9)
#define TCC_STATUS_FAULT0IN           (1 << 10)
#define TCC_STATUS_FAULT1IN           (1 << 11)
#define TCC_STATUS_FAULTA             (1 << 12)
#define TCC_STATUS_FAULTB             (1 << 13)
#define TCC_STATUS_FAULT0             (1 << 14)
#define TCC_STATUS_FAULT1             (1 << 15)
#define TCC_STATUS_CCBV0              (1 << 16)
#define TCC_STATUS_CCBV1              (1 << 17)
#define TCC_STATUS_CCBV2              (1 << 18)
#define TCC_STATUS_CCBV3              (1 << 19)
#define TCC_STATUS_CMP0               (1 << 24)
#define TCC_STATUS_CMP1               (1 << 25)
#define TCC_STATUS_CMP2               (1 << 26)
#define TCC_STATUS_CMP3               (1 << 27)

/* Waveform register */

#define TCC_WAVE_WAVEGEN_SHIFT        (0)
#define TCC_WAVE_WAVEGEN_MASK         (7 << TCC_WAVE_WAVEGEN_SHIFT)
#  define TCC_WAVE_WAVEGEN_NFRQ       (0 << TCC_WAVE_WAVEGEN_SHIFT)
#  define TCC_WAVE_WAVEGEN_MFRQ       (1 << TCC_WAVE_WAVEGEN_SHIFT)
#  define TCC_WAVE_WAVEGEN_NPWM       (2 << TCC_WAVE_WAVEGEN_SHIFT)
#  define TCC_WAVE_WAVEGEN_DSCRITICAL (4 << TCC_WAVE_WAVEGEN_SHIFT)
#  define TCC_WAVE_WAVEGEN_DSBOTTOM   (5 << TCC_WAVE_WAVEGEN_SHIFT)
#  define TCC_WAVE_WAVEGEN_DSBOTH     (6 << TCC_WAVE_WAVEGEN_SHIFT)
#  define TCC_WAVE_WAVEGEN_DSTOP      (7 << TCC_WAVE_WAVEGEN_SHIFT)
#define TCC_WAVE_RAMP_SHIFT           (4)
#define TCC_WAVE_RAMP_MASK            (3 << TCC_WAVE_RAMP_SHIFT)
#  define TCC_WAVE_RAMP_RAMP1         (0 << TCC_WAVE_RAMP_SHIFT)
#  define TCC_WAVE_RAMP_RAMP2A        (1 << TCC_WAVE_RAMP_SHIFT)
#  define TCC_WAVE_RAMP_RAMP2         (2 << TCC_WAVE_RAMP_SHIFT)
#define TCC_WAVE_CIPEREN              (1 << 7)
#define TCC_WAVE_CICCEN0              (1 << 8)
#define TCC_WAVE_CICCEN1              (1 << 9)
#define TCC_WAVE_CICCEN2              (1 << 10)
#define TCC_WAVE_CICCEN3              (1 << 11)
#define TCC_WAVE_POL0                 (1 << 16)
#define TCC_WAVE_POL1                 (1 << 17)
#define TCC_WAVE_POL2                 (1 << 18)
#define TCC_WAVE_POL3                 (1 << 19)
#define TCC_WAVE_SWAP0                (1 << 24)
#define TCC_WAVE_SWAP1                (1 << 25)
#define TCC_WAVE_SWAP2                (1 << 26)
#define TCC_WAVE_SWAP3                (1 << 27)

/* Period, CCx, PERB, CCBx register */

#define TCC_DITHER_SHIFT              (0)
#define TCC_DITHER_MASK               (0x3f << TCC_PER_DITHER_SHIFT)
#  define TCC_DITHER_NONE             (0 << TCC_DITHER_SHIFT)
#  define TCC_DITHER_DITH4            (1 << TCC_DITHER_SHIFT)
#  define TCC_DITHER_DITH5            (2 << TCC_DITHER_SHIFT)
#  define TCC_DITHER_DITH6            (3 << TCC_DITHER_SHIFT)
#define TCC_VALUE_SHIFT               (6)
#define TCC_VALUE_MASK                (0x3ff << TCC_VALUE_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* CONFIG_ARCH_FAMILY_SAMD21 */
#endif /* __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD_TCC_H */
