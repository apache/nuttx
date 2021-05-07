/****************************************************************************
 * arch/arm/src/rp2040/hardware/rp2040_rosc.h
 *
 * Generated from rp2040.svd originally provided by
 *   Raspberry Pi (Trading) Ltd.
 *
 * Copyright 2020 (c) 2020 Raspberry Pi (Trading) Ltd.
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
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_ROSC_H
#define __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_ROSC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp2040_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define RP2040_ROSC_CTRL_OFFSET       0x000000  /* Ring Oscillator control */
#define RP2040_ROSC_FREQA_OFFSET      0x000004  /* The FREQA & FREQB registers control the frequency by controlling the drive strength of each stage The drive strength has 4 levels determined by the number of bits set Increasing the number of bits set increases the drive strength and increases the oscillation frequency 0 bits set is the default drive strength 1 bit set doubles the drive strength 2 bits set triples drive strength 3 bits set quadruples drive strength */
#define RP2040_ROSC_FREQB_OFFSET      0x000008  /* For a detailed description see freqa register */
#define RP2040_ROSC_DORMANT_OFFSET    0x00000c  /* Ring Oscillator pause control This is used to save power by pausing the ROSC On power-up this field is initialised to WAKE An invalid write will also select WAKE Warning: setup the irq before selecting dormant mode */
#define RP2040_ROSC_DIV_OFFSET        0x000010  /* Controls the output divider */
#define RP2040_ROSC_PHASE_OFFSET      0x000014  /* Controls the phase shifted output */
#define RP2040_ROSC_STATUS_OFFSET     0x000018  /* Ring Oscillator Status */
#define RP2040_ROSC_RANDOMBIT_OFFSET  0x00001c  /* This just reads the state of the oscillator output so randomness is compromised if the ring oscillator is stopped or run at a harmonic of the bus frequency */
#define RP2040_ROSC_COUNT_OFFSET      0x000020  /* A down counter running at the ROSC frequency which counts to zero and stops. To start the counter write a non-zero value. Can be used for short software pauses when setting up time sensitive hardware. */

/* Register definitions *****************************************************/

#define RP2040_ROSC_CTRL       (RP2040_ROSC_BASE + RP2040_ROSC_CTRL_OFFSET)
#define RP2040_ROSC_FREQA      (RP2040_ROSC_BASE + RP2040_ROSC_FREQA_OFFSET)
#define RP2040_ROSC_FREQB      (RP2040_ROSC_BASE + RP2040_ROSC_FREQB_OFFSET)
#define RP2040_ROSC_DORMANT    (RP2040_ROSC_BASE + RP2040_ROSC_DORMANT_OFFSET)
#define RP2040_ROSC_DIV        (RP2040_ROSC_BASE + RP2040_ROSC_DIV_OFFSET)
#define RP2040_ROSC_PHASE      (RP2040_ROSC_BASE + RP2040_ROSC_PHASE_OFFSET)
#define RP2040_ROSC_STATUS     (RP2040_ROSC_BASE + RP2040_ROSC_STATUS_OFFSET)
#define RP2040_ROSC_RANDOMBIT  (RP2040_ROSC_BASE + RP2040_ROSC_RANDOMBIT_OFFSET)
#define RP2040_ROSC_COUNT      (RP2040_ROSC_BASE + RP2040_ROSC_COUNT_OFFSET)

/* Register bit definitions *************************************************/

#define RP2040_ROSC_CTRL_ENABLE_SHIFT        (12)  /* On power-up this field is initialised to ENABLE The system clock must be switched to another source before setting this field to DISABLE otherwise the chip will lock up The 12-bit code is intended to give some protection against accidental writes. An invalid setting will enable the oscillator. */
#define RP2040_ROSC_CTRL_ENABLE_MASK         (0xfff << RP2040_ROSC_CTRL_ENABLE_SHIFT)
#define RP2040_ROSC_CTRL_ENABLE_DISABLE      (0xd1e << RP2040_ROSC_CTRL_ENABLE_SHIFT)
#define RP2040_ROSC_CTRL_ENABLE_ENABLE       (0xfab << RP2040_ROSC_CTRL_ENABLE_SHIFT)
#define RP2040_ROSC_CTRL_FREQ_RANGE_MASK     (0xfff)
#define RP2040_ROSC_CTRL_FREQ_RANGE_LOW      (0xfa4)
#define RP2040_ROSC_CTRL_FREQ_RANGE_MEDIUM   (0xfa5)
#define RP2040_ROSC_CTRL_FREQ_RANGE_HIGH     (0xfa7)
#define RP2040_ROSC_CTRL_FREQ_RANGE_TOOHIGH  (0xfa6)

#define RP2040_ROSC_FREQA_PASSWD_SHIFT       (16)    /* Set to 0x9696 to apply the settings Any other value in this field will set all drive strengths to 0 */
#define RP2040_ROSC_FREQA_PASSWD_MASK        (0xffff << RP2040_ROSC_FREQA_PASSWD_SHIFT)
#define RP2040_ROSC_FREQA_PASSWD_PASS        (0x9696 << RP2040_ROSC_FREQA_PASSWD_SHIFT)
#define RP2040_ROSC_FREQA_DS3_SHIFT          (12)    /* Stage 3 drive strength */
#define RP2040_ROSC_FREQA_DS3_MASK           (0x07 << RP2040_ROSC_FREQA_DS3_SHIFT)
#define RP2040_ROSC_FREQA_DS2_SHIFT          (8)     /* Stage 2 drive strength */
#define RP2040_ROSC_FREQA_DS2_MASK           (0x07 << RP2040_ROSC_FREQA_DS2_SHIFT)
#define RP2040_ROSC_FREQA_DS1_SHIFT          (4)     /* Stage 1 drive strength */
#define RP2040_ROSC_FREQA_DS1_MASK           (0x07 << RP2040_ROSC_FREQA_DS1_SHIFT)
#define RP2040_ROSC_FREQA_DS0_MASK           (0x07)  /* Stage 0 drive strength */

#define RP2040_ROSC_FREQB_PASSWD_SHIFT       (16)    /* Set to 0x9696 to apply the settings Any other value in this field will set all drive strengths to 0 */
#define RP2040_ROSC_FREQB_PASSWD_MASK        (0xffff << RP2040_ROSC_FREQB_PASSWD_SHIFT)
#define RP2040_ROSC_FREQB_PASSWD_PASS        (0x9696 << RP2040_ROSC_FREQB_PASSWD_SHIFT)
#define RP2040_ROSC_FREQB_DS7_SHIFT          (12)    /* Stage 7 drive strength */
#define RP2040_ROSC_FREQB_DS7_MASK           (0x07 << RP2040_ROSC_FREQB_DS7_SHIFT)
#define RP2040_ROSC_FREQB_DS6_SHIFT          (8)     /* Stage 6 drive strength */
#define RP2040_ROSC_FREQB_DS6_MASK           (0x07 << RP2040_ROSC_FREQB_DS6_SHIFT)
#define RP2040_ROSC_FREQB_DS5_SHIFT          (4)     /* Stage 5 drive strength */
#define RP2040_ROSC_FREQB_DS5_MASK           (0x07 << RP2040_ROSC_FREQB_DS5_SHIFT)
#define RP2040_ROSC_FREQB_DS4_MASK           (0x07)  /* Stage 4 drive strength */

#define RP2040_ROSC_DIV_MASK                 (0xfff)
#define RP2040_ROSC_DIV_PASS                 (0xaa0)

#define RP2040_ROSC_PHASE_PASSWD_SHIFT       (4)       /* set to 0xaa0 any other value enables the output with shift=0 */
#define RP2040_ROSC_PHASE_PASSWD_MASK        (0xff << RP2040_ROSC_PHASE_PASSWD_SHIFT)
#define RP2040_ROSC_PHASE_ENABLE             (1 << 3)  /* enable the phase-shifted output this can be changed on-the-fly */
#define RP2040_ROSC_PHASE_FLIP               (1 << 2)  /* invert the phase-shifted output this is ignored when div=1 */
#define RP2040_ROSC_PHASE_SHIFT_MASK         (0x03)    /* phase shift the phase-shifted output by SHIFT input clocks this can be changed on-the-fly must be set to 0 before setting div=1 */

#define RP2040_ROSC_STATUS_STABLE            (1 << 31) /* Oscillator is running and stable */
#define RP2040_ROSC_STATUS_BADWRITE          (1 << 24) /* An invalid value has been written to CTRL_ENABLE or CTRL_FREQ_RANGE or FRFEQA or FREQB or DORMANT */
#define RP2040_ROSC_STATUS_DIV_RUNNING       (1 << 16) /* post-divider is running this resets to 0 but transitions to 1 during chip startup */
#define RP2040_ROSC_STATUS_ENABLED           (1 << 12) /* Oscillator is enabled but not necessarily running and stable this resets to 0 but transitions to 1 during chip startup */

#define RP2040_ROSC_RANDOMBIT                (1 << 0)

#define RP2040_ROSC_COUNT_MASK               (0xff)

#endif /* __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_ROSC_H */
