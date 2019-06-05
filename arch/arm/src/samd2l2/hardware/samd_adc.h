/********************************************************************************************
 * arch/arm/src/samd2l2/hardware/samd_adc.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Alexander Vasiljev <alexvasiljev@gmail.com>
 *
 * References:
 *   "Microchip SAM D21 Family Datasheet", Rev D - 9/2018
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

#ifndef __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD_ADC_H
#define __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD_ADC_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#ifdef CONFIG_ARCH_FAMILY_SAMD21

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/
/* ADC register offsets ********************************************************************/

#define SAM_ADC_CTRLA_OFFSET       0x0000 /* Control A Register */
#define SAM_ADC_REFCTL_OFFSET      0x0001 /* Reference Control Register */
#define SAM_ADC_AVGCTRL_OFFSET     0x0002 /* Average Control Register */
#define SAM_ADC_SAMPCTRL_OFFSET    0x0003 /* Sampling Time Control Register */
#define SAM_ADC_CTRLB_OFFSET       0x0004 /* Control B Register */
#define SAM_ADC_WINCTRL_OFFSET     0x0008 /* Window Monitor Control Register */
#define SAM_ADC_SWTRIG_OFFSET      0x000C /* Software Trigger Register */
#define SAM_ADC_INPUTCTRL_OFFSET   0x0010 /* Input Control Register */
#define SAM_ADC_EVCTRL_OFFSET      0x0014 /* Event Control Register */
#define SAM_ADC_INTENCLR_OFFSET    0x0016 /* Interrupt Enable Clear Register */
#define SAM_ADC_INTENSET_OFFSET    0x0017 /* Interrupt Enable Set Register */
#define SAM_ADC_INTFLAG_OFFSET     0x0018 /* Interrupt Flag Status and Clear Register */
#define SAM_ADC_STATUS_OFFSET      0x0019 /* Status Register */
#define SAM_ADC_RESULT_OFFSET      0x001A /* Result Register */
#define SAM_ADC_WINLT_OFFSET       0x001C /* Window Monitor Lower Threshold Register */
#define SAM_ADC_WINUT_OFFSET       0x0020 /* Window Monitor Upper Threshold Register */
#define SAM_ADC_GAINCORR_OFFSET    0x0024 /* Gain Correction Register */
#define SAM_ADC_OFFSETCORR_OFFSET  0x0026 /* Offset Correction Register */
#define SAM_ADC_CALIB_OFFSET       0x0028 /* Calibration Register */
#define SAM_ADC_DBGCTRL_OFFSET     0x002A /* Debug Control Register */

/* ADC register addresses ******************************************************************/

#define SAM_ADC_CTRLA              (SAM_ADC_BASE + SAM_ADC_CTRLA_OFFSET)
#define SAM_ADC_REFCTL             (SAM_ADC_BASE + SAM_ADC_REFCTL_OFFSET)
#define SAM_ADC_AVGCTRL            (SAM_ADC_BASE + SAM_ADC_AVGCTRL_OFFSET)
#define SAM_ADC_SAMPCTRL           (SAM_ADC_BASE + SAM_ADC_SAMPCTRL_OFFSET)
#define SAM_ADC_CTRLB              (SAM_ADC_BASE + SAM_ADC_CTRLB_OFFSET)
#define SAM_ADC_WINCTRL            (SAM_ADC_BASE + SAM_ADC_WINCTRL_OFFSET)
#define SAM_ADC_SWTRIG             (SAM_ADC_BASE + SAM_ADC_SWTRIG_OFFSET)
#define SAM_ADC_INPUTCTRL          (SAM_ADC_BASE + SAM_ADC_INPUTCTRL_OFFSET)
#define SAM_ADC_EVCTRL             (SAM_ADC_BASE + SAM_ADC_EVCTRL_OFFSET)
#define SAM_ADC_INTENCLR           (SAM_ADC_BASE + SAM_ADC_INTENCLR_OFFSET)
#define SAM_ADC_INTENSET           (SAM_ADC_BASE + SAM_ADC_INTENSET_OFFSET)
#define SAM_ADC_INTFLAG            (SAM_ADC_BASE + SAM_ADC_INTFLAG_OFFSET)
#define SAM_ADC_STATUS             (SAM_ADC_BASE + SAM_ADC_STATUS_OFFSET)
#define SAM_ADC_RESULT             (SAM_ADC_BASE + SAM_ADC_RESULT_OFFSET)
#define SAM_ADC_WINLT              (SAM_ADC_BASE + SAM_ADC_WINLT_OFFSET)
#define SAM_ADC_WINUT              (SAM_ADC_BASE + SAM_ADC_WINUT_OFFSET)
#define SAM_ADC_GAINCORR           (SAM_ADC_BASE + SAM_ADC_GAINCORR_OFFSET)
#define SAM_ADC_OFFSETCORR         (SAM_ADC_BASE + SAM_ADC_OFFSETCORR_OFFSET)
#define SAM_ADC_CALIB              (SAM_ADC_BASE + SAM_ADC_CALIB_OFFSET)
#define SAM_ADC_ADC_DBGCTRL        (SAM_ADC_BASE + SAM_ADC_DBGCTRL_OFFSET)

/* ADC register bit definitions ************************************************************/

/* Control A Register */

#define ADC_CTRLA_SWRST              (1 << 0)  /* Bit 0:  Software reset */
#define ADC_CTRLA_ENABLE             (1 << 1)  /* Bit 1:  Enable ADC controller */
#define ADC_CTRLA_RUNSTDBY           (1 << 2)  /* Bit 2:  Run in standby */

/* Reference Control Register */

#define ADC_REFCTRL_REFSEL_OFFSET    (0) /* Bit 3:0: Reference selection */
#define ADC_REFCTRL_REFSEL_MASK      (0x0F << ADC_REFCTRL_REFSEL_OFFSET)
#  define ADC_REFCTRL_REFSEL_INT1V   (0 << ADC_REFCTRL_REFSEL_OFFSET) /* 1.0V voltage reference */
#  define ADC_REFCTRL_REFSEL_INTVCC0 (1 << ADC_REFCTRL_REFSEL_OFFSET) /* 1/1.48 VDDANA */
#  define ADC_REFCTRL_REFSEL_INTVCC1 (2 << ADC_REFCTRL_REFSEL_OFFSET) /* 1/2 VDDANA (only for VDDANA > 2.0V) */
#  define ADC_REFCTRL_REFSEL_VREFA   (3 << ADC_REFCTRL_REFSEL_OFFSET) /* External reference */
#  define ADC_REFCTRL_REFSEL_VREFB   (4 << ADC_REFCTRL_REFSEL_OFFSET) /* External reference */
#define ADC_REFCTRL_REFCOMP          (1 << 7) /* Bit 7: Reference buffer offset compensation enable */

/* Average Control Register */

#define ADC_AVGCTRL_SAMPLENUM_OFFSET (0) /* Bit 3:0: Number of samples to be collected */
#define ADC_AVGCTRL_SAMPLENUM_MASK   (0x0F << ADC_AVGCTRL_SAMPLENUM_OFFSET)
#  define ADC_AVGCTRL_SAMPLENUM_1    (0x00 << ADC_AVGCTRL_SAMPLENUM_OFFSET)
#  define ADC_AVGCTRL_SAMPLENUM_2    (0x01 << ADC_AVGCTRL_SAMPLENUM_OFFSET)
#  define ADC_AVGCTRL_SAMPLENUM_4    (0x02 << ADC_AVGCTRL_SAMPLENUM_OFFSET)
#  define ADC_AVGCTRL_SAMPLENUM_8    (0x03 << ADC_AVGCTRL_SAMPLENUM_OFFSET)
#  define ADC_AVGCTRL_SAMPLENUM_16   (0x04 << ADC_AVGCTRL_SAMPLENUM_OFFSET)
#  define ADC_AVGCTRL_SAMPLENUM_32   (0x05 << ADC_AVGCTRL_SAMPLENUM_OFFSET)
#  define ADC_AVGCTRL_SAMPLENUM_64   (0x06 << ADC_AVGCTRL_SAMPLENUM_OFFSET)
#  define ADC_AVGCTRL_SAMPLENUM_128  (0x07 << ADC_AVGCTRL_SAMPLENUM_OFFSET)
#  define ADC_AVGCTRL_SAMPLENUM_256  (0x08 << ADC_AVGCTRL_SAMPLENUM_OFFSET)
#  define ADC_AVGCTRL_SAMPLENUM_512  (0x09 << ADC_AVGCTRL_SAMPLENUM_OFFSET)
#  define ADC_AVGCTRL_SAMPLENUM_1024 (0x0A << ADC_AVGCTRL_SAMPLENUM_OFFSET)
#define ADC_AVGCTRL_ADJRES_OFFSET    (1 << 4) /* Bit 4:6: Adjusting result/division coefficient */
#define ADC_AVGCTRL_ADJRES_MASK      (7 << ADC_AVGCTRL_ADJRES_OFFSET)

/* Sampling Time Control Register */

#define ADC_SAMPCTRL_SAMPLEN_OFFSET  (0) /* Bit 5:0:Sampling time length */
#define ADC_SAMPCTRL_SAMPLEN_MASK    (0x3F << ADC_SAMPCTRL_SAMPLEN_OFFSET)

/* Control B Register */

#define ADC_CTRLB_DIFFMODE           (1 << 0) /* Bit 0: Differenstial mode */
#define ADC_CTRLB_LEFTADJ            (1 << 1) /* Bit 1: Left-adjusted result */
#define ADC_CTRLB_FREERUN            (1 << 2) /* Bit 2: Free running mode */
#define ADC_CTRLB_CORREN             (1 << 3) /* Bit 3: Digital correction logic enabled */
#define ADC_CTRLB_RESSEL_OFFSET      (4) /* Bit 5:4: Conversion result resolution */
#define ADC_CTRLB_RESSEL_MASK        (3 << ADC_CTRLB_RESSEL_OFFSET)
#  define ADC_CTRLB_RESSEL_12BIT     (0 << ADC_CTRLB_RESSEL_OFFSET) /* 12-bit result */
#  define ADC_CTRLB_RESSEL_16BIT     (1 << ADC_CTRLB_RESSEL_OFFSET) /* For averaging mode output */
#  define ADC_CTRLB_RESSEL_10BIT     (2 << ADC_CTRLB_RESSEL_OFFSET) /* 10-bit result */
#  define ADC_CTRLB_RESSEL_8BIT      (3 << ADC_CTRLB_RESSEL_OFFSET) /* 8-bit result */
#define ADC_CTRLB_PRESCALER_OFFSET   (8) /* Bit 10:8: Prescaler configuration */
#define ADC_CTRLB_PRESCALER_MASK     (7 << ADC_CTRLB_PRESCALER_OFFSET)
#  define ADC_CTRLB_PRESCALER_DIV4   (0 << ADC_CTRLB_PRESCALER_OFFSET)
#  define ADC_CTRLB_PRESCALER_DIV8   (1 << ADC_CTRLB_PRESCALER_OFFSET)
#  define ADC_CTRLB_PRESCALER_DIV16  (2 << ADC_CTRLB_PRESCALER_OFFSET)
#  define ADC_CTRLB_PRESCALER_DIV32  (3 << ADC_CTRLB_PRESCALER_OFFSET)
#  define ADC_CTRLB_PRESCALER_DIV64  (4 << ADC_CTRLB_PRESCALER_OFFSET)
#  define ADC_CTRLB_PRESCALER_DIV128 (5 << ADC_CTRLB_PRESCALER_OFFSET)
#  define ADC_CTRLB_PRESCALER_DIV256 (6 << ADC_CTRLB_PRESCALER_OFFSET)
#  define ADC_CTRLB_PRESCALER_DIV512 (7 << ADC_CTRLB_PRESCALER_OFFSET)

/* Window Monitor Control Register */

#define ADC_WINCTRL_WINMODE_OFFSET     (0) /* Bit 2:0: Window monitor mode */
#define ADC_WINCTRL_WINMODE_MASK       (7)
#  define ADC_WINCTRL_WINMODE_DISABLE  (0 << ADC_WINCTRL_WINMODE_OFFSET) /* No window mode */
#  define ADC_WINCTRL_WINMODE_MODE1    (1 << ADC_WINCTRL_WINMODE_OFFSET) /* Mode 1: result > winlt */
#  define ADC_WINCTRL_WINMODE_MODE2    (2 << ADC_WINCTRL_WINMODE_OFFSET) /* Mode 2: result < winut */
#  define ADC_WINCTRL_WINMODE_MODE3    (3 << ADC_WINCTRL_WINMODE_OFFSET) /* Mode 3: winlt < result < winut */
#  define ADC_WINCTRL_WINMODE_MODE4    (4 << ADC_WINCTRL_WINMODE_OFFSET) /* Mode 4: !(winlt < result < winut) */

/* Software Trigger Register */

#define ADC_SWTRIG_FLUSH            (1 << 0) /* Bit 0: Adc conversion flush */
#define ADC_SWTRIG_START            (1 << 1) /* Bit 1: Adc start conversion */

/* Input Control Register */

#define ADC_INPUTCTRL_MUXPOS_OFFSET           (0) /* Bit 4:0: Positive mux input selection */
#define ADC_INPUTCTRL_MUXPOS_MASK             (0x1F << ADC_INPUTCTRL_MUXPOS_OFFSET)
#  define ADC_INPUTCTRL_MUXPOS_AIN0           (0 << ADC_INPUTCTRL_MUXPOS_OFFSET)
#  define ADC_INPUTCTRL_MUXPOS_AIN1           (1 << ADC_INPUTCTRL_MUXPOS_OFFSET)
#  define ADC_INPUTCTRL_MUXPOS_AIN2           (2 << ADC_INPUTCTRL_MUXPOS_OFFSET)
#  define ADC_INPUTCTRL_MUXPOS_AIN3           (3 << ADC_INPUTCTRL_MUXPOS_OFFSET)
#  define ADC_INPUTCTRL_MUXPOS_AIN4           (4 << ADC_INPUTCTRL_MUXPOS_OFFSET)
#  define ADC_INPUTCTRL_MUXPOS_AIN5           (5 << ADC_INPUTCTRL_MUXPOS_OFFSET)
#  define ADC_INPUTCTRL_MUXPOS_AIN6           (6 << ADC_INPUTCTRL_MUXPOS_OFFSET)
#  define ADC_INPUTCTRL_MUXPOS_AIN7           (7 << ADC_INPUTCTRL_MUXPOS_OFFSET)
#  define ADC_INPUTCTRL_MUXPOS_AIN8           (8 << ADC_INPUTCTRL_MUXPOS_OFFSET)
#  define ADC_INPUTCTRL_MUXPOS_AIN9           (9 << ADC_INPUTCTRL_MUXPOS_OFFSET)
#  define ADC_INPUTCTRL_MUXPOS_AIN10          (10 << ADC_INPUTCTRL_MUXPOS_OFFSET)
#  define ADC_INPUTCTRL_MUXPOS_AIN11          (11 << ADC_INPUTCTRL_MUXPOS_OFFSET)
#  define ADC_INPUTCTRL_MUXPOS_AIN12          (12 << ADC_INPUTCTRL_MUXPOS_OFFSET)
#  define ADC_INPUTCTRL_MUXPOS_AIN13          (13 << ADC_INPUTCTRL_MUXPOS_OFFSET)
#  define ADC_INPUTCTRL_MUXPOS_AIN14          (14 << ADC_INPUTCTRL_MUXPOS_OFFSET)
#  define ADC_INPUTCTRL_MUXPOS_AIN15          (15 << ADC_INPUTCTRL_MUXPOS_OFFSET)
#  define ADC_INPUTCTRL_MUXPOS_AIN16          (16 << ADC_INPUTCTRL_MUXPOS_OFFSET)
#  define ADC_INPUTCTRL_MUXPOS_AIN17          (17 << ADC_INPUTCTRL_MUXPOS_OFFSET)
#  define ADC_INPUTCTRL_MUXPOS_AIN18          (18 << ADC_INPUTCTRL_MUXPOS_OFFSET)
#  define ADC_INPUTCTRL_MUXPOS_AIN19          (19 << ADC_INPUTCTRL_MUXPOS_OFFSET)
#  define ADC_INPUTCTRL_MUXPOS_BANDGAP        (0x19 << ADC_INPUTCTRL_MUXPOS_OFFSET) /* Bandgap voltage */
#  define ADC_INPUTCTRL_MUXPOS_SCALEDCOREVCC  (0x1A << ADC_INPUTCTRL_MUXPOS_OFFSET) /* 1/4 scaled core voltage */
#  define ADC_INPUTCTRL_MUXPOS_SCALEDIOVCC    (0x1B << ADC_INPUTCTRL_MUXPOS_OFFSET) /* 1/4 scaled I/) supplly */
#  define ADC_INPUTCTRL_MUXPOS_DAC            (0x1C << ADC_INPUTCTRL_MUXPOS_OFFSET) /* DAC output */
#define ADC_INPUTCTRL_MUXNEG_OFFSET           (8) /* Bit 12:8: Negative mux input selection */
#define ADC_INPUTCTRL_MUXNEG_MASK             (0x1F << ADC_INPUTCTRL_MUXNEG_OFFSET)
#  define ADC_INPUTCTRL_MUXNEG_AIN0             (0 << ADC_INPUTCTRL_MUXNEG_OFFSET)
#  define ADC_INPUTCTRL_MUXNEG_AIN1             (1 << ADC_INPUTCTRL_MUXNEG_OFFSET)
#  define ADC_INPUTCTRL_MUXNEG_AIN2             (2 << ADC_INPUTCTRL_MUXNEG_OFFSET)
#  define ADC_INPUTCTRL_MUXNEG_AIN3             (3 << ADC_INPUTCTRL_MUXNEG_OFFSET)
#  define ADC_INPUTCTRL_MUXNEG_AIN4             (4 << ADC_INPUTCTRL_MUXNEG_OFFSET)
#  define ADC_INPUTCTRL_MUXNEG_AIN5             (5 << ADC_INPUTCTRL_MUXNEG_OFFSET)
#  define ADC_INPUTCTRL_MUXNEG_AIN6             (6 << ADC_INPUTCTRL_MUXNEG_OFFSET)
#  define ADC_INPUTCTRL_MUXNEG_AIN7             (7 << ADC_INPUTCTRL_MUXNEG_OFFSET)
#  define ADC_INPUTCTRL_MUXNEG_GND              (0x18 << ADC_INPUTCTRL_MUXNEG_OFFSET) /* Internal ground */
#  define ADC_INPUTCTRL_MUXNEG_IOGND            (0x19 << ADC_INPUTCTRL_MUXNEG_OFFSET)  /* I/O ground */
#define ADC_INPUTCTRL_INPUTSCAN_OFFSET        (16) /* Bit 19:16: Number of input channels included in scan */
#define ADC_INPUTCTRL_INPUTSCAN_MASK          (0x0F << ADC_INPUTCTRL_INPUTSCAN_OFFSET)
#define ADC_INPUTCTRL_INPUTOFFSET_OFFSET      (20) /* Bit 23:20: Positive mux setting offset */
#define ADC_INPUTCTRL_INPUTOFFSET_MASK        (0x0F << ADC_INPUTCTRL_INPUTOFFSET_OFFSET)
#define ADC_INPUTCTRL_GAIN_OFFSET             (24) /* Bit 27:24: Gain factor selection */
#define ADC_INPUTCTRL_GAIN_MASK               (0x0F) << ADC_INPUTCTRL_GAIN_OFFSET
#  define ADC_INPUTCTRL_GAIN_1X               (0 << ADC_INPUTCTRL_GAIN_OFFSET)
#  define ADC_INPUTCTRL_GAIN_2X               (1 << ADC_INPUTCTRL_GAIN_OFFSET)
#  define ADC_INPUTCTRL_GAIN_4X               (2 << ADC_INPUTCTRL_GAIN_OFFSET)
#  define ADC_INPUTCTRL_GAIN_8X               (3 << ADC_INPUTCTRL_GAIN_OFFSET)
#  define ADC_INPUTCTRL_GAIN_16X              (4 << ADC_INPUTCTRL_GAIN_OFFSET)
#  define ADC_INPUTCTRL_GAIN_DIV2             (15 << ADC_INPUTCTRL_GAIN_OFFSET)

/* Event Control Register */

#define ADC_EVCTRL_STARTEI               (1 << 0) /* Bit 0: Start conversion event in */
#define ADC_EVCTRL_SYNCEI                (1 << 1) /* Bit 1: Synchronization event in */
#define ADC_EVCTRL_RESRDYEO              (1 << 4) /* Bit 4: Result ready event out */
#define ADC_EVCTRL_WINMONEO              (1 << 5) /* Bit 5: Window monitor event out */

/* Common bit definitions for Interrupt Enable Clear Register, Interrupt Enable Set
 * Register, and Interrupt Flag Status and Clear Register
 */

#define ADC_INT_RESRDY              (1 << 0) /* Bit 0: Result ready */
#define ADC_INT_OVERRUN             (1 << 1) /* Bit 1: Overrun */
#define ADC_INT_WINMON              (1 << 2) /* Bit 2: Window monitor */
#define ADC_INT_SYNCRDY             (1 << 3) /* Bit 3: Synchronization ready */

/* Status Register */

#define ADC_STATUS_SYNCBUSY         (1 << 7) /* Bit 7: Synchronization buzy */

/* Calibration Register */
#define ADC_CALIB_LINEARITY_OFFSET  (0) /* Bit 7:0: Linearity calibration value */
#define ADC_CALIB_LINEARITY_MASK    (0xFF)
#define ADC_CALIB_BIAS_OFFSET       (8) /* Bit 10:8: Bias calibration value */
#define ADC_CALIB_BIAS_MASK         (7 << ADC_CALIB_BIAS_OFFSET)

/* Debug Control Register */

#define ADC_DBGCTRL_DBGRUN          (1) /* Bit 0: Debug run */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* CONFIG_ARCH_FAMILY_SAMD21 */
#endif /* __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD_ADC_H */
