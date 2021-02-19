/****************************************************************************
 * arch/arm/src/rp2040/hardware/rp2040_clocks.h
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

#ifndef __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_CLOCKS_H
#define __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_CLOCKS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp2040_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clock index **************************************************************/

#define RP2040_CLOCKS_NDX_GPOUT0    0   /* Clock output to GPIO 21 */
#define RP2040_CLOCKS_NDX_GPOUT1    1   /* Clock output to GPIO 23 */
#define RP2040_CLOCKS_NDX_GPOUT2    2   /* Clock output to GPIO 24 */
#define RP2040_CLOCKS_NDX_GPOUT3    3   /* Clock output to GPIO 25 */
#define RP2040_CLOCKS_NDX_REF       4   /* Reference clock */
#define RP2040_CLOCKS_NDX_SYS       5   /* System clock */
#define RP2040_CLOCKS_NDX_PERI      6   /* Peripheral clock */
#define RP2040_CLOCKS_NDX_USB       7   /* USB clock */
#define RP2040_CLOCKS_NDX_ADC       8   /* ADC clock */
#define RP2040_CLOCKS_NDX_RTC       9   /* RTC clock */
#define RP2040_CLOCKS_NDX_MAX       10

/* Register offsets *********************************************************/

#define RP2040_CLOCKS_CLK_CTRL_OFFSET              0x000000  /* Clock control */
#define RP2040_CLOCKS_CLK_DIV_OFFSET               0x000004  /* Clock divisor */
#define RP2040_CLOCKS_CLK_SELECTED_OFFSET          0x000008  /* Indicates which src is currently selected */

#define RP2040_CLOCKS_CLK_NDX_CTRL_OFFSET(n)        ((n) * 12 + RP2040_CLOCKS_CLK_CTRL_OFFSET)
#define RP2040_CLOCKS_CLK_NDX_DIV_OFFSET(n)         ((n) * 12 + RP2040_CLOCKS_CLK_DIV_OFFSET)
#define RP2040_CLOCKS_CLK_NDX_SELECTED_OFFSET(n)    ((n) * 12 + RP2040_CLOCKS_CLK_SELECTED_OFFSET)

#define RP2040_CLOCKS_CLK_SYS_RESUS_CTRL_OFFSET    0x000078
#define RP2040_CLOCKS_CLK_SYS_RESUS_STATUS_OFFSET  0x00007c
#define RP2040_CLOCKS_FC0_REF_KHZ_OFFSET           0x000080  /* Reference clock frequency in kHz */
#define RP2040_CLOCKS_FC0_MIN_KHZ_OFFSET           0x000084  /* Minimum pass frequency in kHz. This is optional. Set to 0 if you are not using the pass/fail flags */
#define RP2040_CLOCKS_FC0_MAX_KHZ_OFFSET           0x000088  /* Maximum pass frequency in kHz. This is optional. Set to 0x1ffffff if you are not using the pass/fail flags */
#define RP2040_CLOCKS_FC0_DELAY_OFFSET             0x00008c  /* Delays the start of frequency counting to allow the mux to settle Delay is measured in multiples of the reference clock period */
#define RP2040_CLOCKS_FC0_INTERVAL_OFFSET          0x000090  /* The test interval is 0.98us * 2**interval, but let's call it 1us * 2**interval The default gives a test interval of 250us */
#define RP2040_CLOCKS_FC0_SRC_OFFSET               0x000094  /* Clock sent to frequency counter, set to 0 when not required Writing to this register initiates the frequency count */
#define RP2040_CLOCKS_FC0_STATUS_OFFSET            0x000098  /* Frequency counter status */
#define RP2040_CLOCKS_FC0_RESULT_OFFSET            0x00009c  /* Result of frequency measurement, only valid when status_done=1 */
#define RP2040_CLOCKS_WAKE_EN0_OFFSET              0x0000a0  /* enable clock in wake mode */
#define RP2040_CLOCKS_WAKE_EN1_OFFSET              0x0000a4  /* enable clock in wake mode */
#define RP2040_CLOCKS_SLEEP_EN0_OFFSET             0x0000a8  /* enable clock in sleep mode */
#define RP2040_CLOCKS_SLEEP_EN1_OFFSET             0x0000ac  /* enable clock in sleep mode */
#define RP2040_CLOCKS_ENABLED0_OFFSET              0x0000b0  /* indicates the state of the clock enable */
#define RP2040_CLOCKS_ENABLED1_OFFSET              0x0000b4  /* indicates the state of the clock enable */
#define RP2040_CLOCKS_INTR_OFFSET                  0x0000b8  /* Raw Interrupts */
#define RP2040_CLOCKS_INTE_OFFSET                  0x0000bc  /* Interrupt Enable */
#define RP2040_CLOCKS_INTF_OFFSET                  0x0000c0  /* Interrupt Force */
#define RP2040_CLOCKS_INTS_OFFSET                  0x0000c4  /* Interrupt status after masking & forcing */

/* Register definitions *****************************************************/

#define RP2040_CLOCKS_CLK_NDX_CTRL(n)       (RP2040_CLOCKS_BASE + RP2040_CLOCKS_CLK_NDX_CTRL_OFFSET(n))
#define RP2040_CLOCKS_CLK_NDX_DIV(n)        (RP2040_CLOCKS_BASE + RP2040_CLOCKS_CLK_NDX_DIV_OFFSET(n))
#define RP2040_CLOCKS_CLK_NDX_SELECTED(n)   (RP2040_CLOCKS_BASE + RP2040_CLOCKS_CLK_NDX_SELECTED_OFFSET(n))

#define RP2040_CLOCKS_CLK_GPOUT0_CTRL       (RP2040_CLOCKS_CLK_NDX_CTRL(RP2040_CLOCKS_NDX_GPOUT0))
#define RP2040_CLOCKS_CLK_GPOUT0_DIV        (RP2040_CLOCKS_CLK_NDX_DIV(RP2040_CLOCKS_NDX_GPOUT0))
#define RP2040_CLOCKS_CLK_GPOUT0_SELECTED   (RP2040_CLOCKS_CLK_NDX_SELECTED(RP2040_CLOCKS_NDX_GPOUT0))
#define RP2040_CLOCKS_CLK_GPOUT1_CTRL       (RP2040_CLOCKS_CLK_NDX_CTRL(RP2040_CLOCKS_NDX_GPOUT1))
#define RP2040_CLOCKS_CLK_GPOUT1_DIV        (RP2040_CLOCKS_CLK_NDX_DIV(RP2040_CLOCKS_NDX_GPOUT1))
#define RP2040_CLOCKS_CLK_GPOUT1_SELECTED   (RP2040_CLOCKS_CLK_NDX_SELECTED(RP2040_CLOCKS_NDX_GPOUT1))
#define RP2040_CLOCKS_CLK_GPOUT2_CTRL       (RP2040_CLOCKS_CLK_NDX_CTRL(RP2040_CLOCKS_NDX_GPOUT2))
#define RP2040_CLOCKS_CLK_GPOUT2_DIV        (RP2040_CLOCKS_CLK_NDX_DIV(RP2040_CLOCKS_NDX_GPOUT2))
#define RP2040_CLOCKS_CLK_GPOUT2_SELECTED   (RP2040_CLOCKS_CLK_NDX_SELECTED(RP2040_CLOCKS_NDX_GPOUT2))
#define RP2040_CLOCKS_CLK_GPOUT3_CTRL       (RP2040_CLOCKS_CLK_NDX_CTRL(RP2040_CLOCKS_NDX_GPOUT3))
#define RP2040_CLOCKS_CLK_GPOUT3_DIV        (RP2040_CLOCKS_CLK_NDX_DIV(RP2040_CLOCKS_NDX_GPOUT3))
#define RP2040_CLOCKS_CLK_GPOUT3_SELECTED   (RP2040_CLOCKS_CLK_NDX_SELECTED(RP2040_CLOCKS_NDX_GPOUT3))
#define RP2040_CLOCKS_CLK_REF_CTRL          (RP2040_CLOCKS_CLK_NDX_CTRL(RP2040_CLOCKS_NDX_REF))
#define RP2040_CLOCKS_CLK_REF_DIV           (RP2040_CLOCKS_CLK_NDX_DIV(RP2040_CLOCKS_NDX_REF))
#define RP2040_CLOCKS_CLK_REF_SELECTED      (RP2040_CLOCKS_CLK_NDX_SELECTED(RP2040_CLOCKS_NDX_REF))
#define RP2040_CLOCKS_CLK_SYS_CTRL          (RP2040_CLOCKS_CLK_NDX_CTRL(RP2040_CLOCKS_NDX_SYS))
#define RP2040_CLOCKS_CLK_SYS_DIV           (RP2040_CLOCKS_CLK_NDX_DIV(RP2040_CLOCKS_NDX_SYS))
#define RP2040_CLOCKS_CLK_SYS_SELECTED      (RP2040_CLOCKS_CLK_NDX_SELECTED(RP2040_CLOCKS_NDX_SYS))
#define RP2040_CLOCKS_CLK_PERI_CTRL         (RP2040_CLOCKS_CLK_NDX_CTRL(RP2040_CLOCKS_NDX_PERI))
#define RP2040_CLOCKS_CLK_PERI_SELECTED     (RP2040_CLOCKS_CLK_NDX_SELECTED(RP2040_CLOCKS_NDX_PERI))
#define RP2040_CLOCKS_CLK_USB_CTRL          (RP2040_CLOCKS_CLK_NDX_CTRL(RP2040_CLOCKS_NDX_USB))
#define RP2040_CLOCKS_CLK_USB_DIV           (RP2040_CLOCKS_CLK_NDX_DIV(RP2040_CLOCKS_NDX_USB))
#define RP2040_CLOCKS_CLK_USB_SELECTED      (RP2040_CLOCKS_CLK_NDX_SELECTED(RP2040_CLOCKS_NDX_USB))
#define RP2040_CLOCKS_CLK_ADC_CTRL          (RP2040_CLOCKS_CLK_NDX_CTRL(RP2040_CLOCKS_NDX_ADC))
#define RP2040_CLOCKS_CLK_ADC_DIV           (RP2040_CLOCKS_CLK_NDX_DIV(RP2040_CLOCKS_NDX_ADC))
#define RP2040_CLOCKS_CLK_ADC_SELECTED      (RP2040_CLOCKS_CLK_NDX_SELECTED(RP2040_CLOCKS_NDX_ADC))
#define RP2040_CLOCKS_CLK_RTC_CTRL          (RP2040_CLOCKS_CLK_NDX_CTRL(RP2040_CLOCKS_NDX_RTC))
#define RP2040_CLOCKS_CLK_RTC_DIV           (RP2040_CLOCKS_CLK_NDX_DIV(RP2040_CLOCKS_NDX_RTC))
#define RP2040_CLOCKS_CLK_RTC_SELECTED      (RP2040_CLOCKS_CLK_NDX_SELECTED(RP2040_CLOCKS_NDX_RTC))

#define RP2040_CLOCKS_CLK_SYS_RESUS_CTRL    (RP2040_CLOCKS_BASE + RP2040_CLOCKS_CLK_SYS_RESUS_CTRL_OFFSET)
#define RP2040_CLOCKS_CLK_SYS_RESUS_STATUS  (RP2040_CLOCKS_BASE + RP2040_CLOCKS_CLK_SYS_RESUS_STATUS_OFFSET)
#define RP2040_CLOCKS_FC0_REF_KHZ           (RP2040_CLOCKS_BASE + RP2040_CLOCKS_FC0_REF_KHZ_OFFSET)
#define RP2040_CLOCKS_FC0_MIN_KHZ           (RP2040_CLOCKS_BASE + RP2040_CLOCKS_FC0_MIN_KHZ_OFFSET)
#define RP2040_CLOCKS_FC0_MAX_KHZ           (RP2040_CLOCKS_BASE + RP2040_CLOCKS_FC0_MAX_KHZ_OFFSET)
#define RP2040_CLOCKS_FC0_DELAY             (RP2040_CLOCKS_BASE + RP2040_CLOCKS_FC0_DELAY_OFFSET)
#define RP2040_CLOCKS_FC0_INTERVAL          (RP2040_CLOCKS_BASE + RP2040_CLOCKS_FC0_INTERVAL_OFFSET)
#define RP2040_CLOCKS_FC0_SRC               (RP2040_CLOCKS_BASE + RP2040_CLOCKS_FC0_SRC_OFFSET)
#define RP2040_CLOCKS_FC0_STATUS            (RP2040_CLOCKS_BASE + RP2040_CLOCKS_FC0_STATUS_OFFSET)
#define RP2040_CLOCKS_FC0_RESULT            (RP2040_CLOCKS_BASE + RP2040_CLOCKS_FC0_RESULT_OFFSET)
#define RP2040_CLOCKS_WAKE_EN0              (RP2040_CLOCKS_BASE + RP2040_CLOCKS_WAKE_EN0_OFFSET)
#define RP2040_CLOCKS_WAKE_EN1              (RP2040_CLOCKS_BASE + RP2040_CLOCKS_WAKE_EN1_OFFSET)
#define RP2040_CLOCKS_SLEEP_EN0             (RP2040_CLOCKS_BASE + RP2040_CLOCKS_SLEEP_EN0_OFFSET)
#define RP2040_CLOCKS_SLEEP_EN1             (RP2040_CLOCKS_BASE + RP2040_CLOCKS_SLEEP_EN1_OFFSET)
#define RP2040_CLOCKS_ENABLED0              (RP2040_CLOCKS_BASE + RP2040_CLOCKS_ENABLED0_OFFSET)
#define RP2040_CLOCKS_ENABLED1              (RP2040_CLOCKS_BASE + RP2040_CLOCKS_ENABLED1_OFFSET)
#define RP2040_CLOCKS_INTR                  (RP2040_CLOCKS_BASE + RP2040_CLOCKS_INTR_OFFSET)
#define RP2040_CLOCKS_INTE                  (RP2040_CLOCKS_BASE + RP2040_CLOCKS_INTE_OFFSET)
#define RP2040_CLOCKS_INTF                  (RP2040_CLOCKS_BASE + RP2040_CLOCKS_INTF_OFFSET)
#define RP2040_CLOCKS_INTS                  (RP2040_CLOCKS_BASE + RP2040_CLOCKS_INTS_OFFSET)

/* Register bit definitions *************************************************/

#define RP2040_CLOCKS_CLK_GPOUT0_CTRL_NUDGE                  (1 << 20)  /* An edge on this signal shifts the phase of the output by 1 cycle of the input clock This can be done at any time */
#define RP2040_CLOCKS_CLK_GPOUT0_CTRL_PHASE_SHIFT            (16)       /* This delays the enable signal by up to 3 cycles of the input clock This must be set before the clock is enabled to have any effect */
#define RP2040_CLOCKS_CLK_GPOUT0_CTRL_PHASE_MASK             (0x03 << RP2040_CLOCKS_CLK_GPOUT0_CTRL_PHASE_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT0_CTRL_DC50                   (1 << 12)  /* Enables duty cycle correction for odd divisors */
#define RP2040_CLOCKS_CLK_GPOUT0_CTRL_ENABLE                 (1 << 11)  /* Starts and stops the clock generator cleanly */
#define RP2040_CLOCKS_CLK_GPOUT0_CTRL_KILL                   (1 << 10)  /* Asynchronously kills the clock generator */
#define RP2040_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_SHIFT           (5)        /* Selects the auxiliary clock source, will glitch when switching */
#define RP2040_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_MASK            (0x0f << RP2040_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_CLKSRC_PLL_SYS  (0x0 << RP2040_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_CLKSRC_GPIN0    (0x1 << RP2040_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_CLKSRC_GPIN1    (0x2 << RP2040_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_CLKSRC_PLL_USB  (0x3 << RP2040_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_ROSC_CLKSRC     (0x4 << RP2040_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_XOSC_CLKSRC     (0x5 << RP2040_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_CLK_SYS         (0x6 << RP2040_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_CLK_USB         (0x7 << RP2040_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_CLK_ADC         (0x8 << RP2040_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_CLK_RTC         (0x9 << RP2040_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_CLK_REF         (0xa << RP2040_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_SHIFT)

#define RP2040_CLOCKS_CLK_GPOUT0_DIV_INT_SHIFT               (8)        /* Integer component of the divisor, 0 -> divide by 2^16 */
#define RP2040_CLOCKS_CLK_GPOUT0_DIV_INT_MASK                (0xffffff << RP2040_CLOCKS_CLK_GPOUT0_DIV_INT_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT0_DIV_FRAC_MASK               (0xff)     /* Fractional component of the divisor */

#define RP2040_CLOCKS_CLK_GPOUT1_CTRL_NUDGE                  (1 << 20)  /* An edge on this signal shifts the phase of the output by 1 cycle of the input clock This can be done at any time */
#define RP2040_CLOCKS_CLK_GPOUT1_CTRL_PHASE_SHIFT            (16)       /* This delays the enable signal by up to 3 cycles of the input clock This must be set before the clock is enabled to have any effect */
#define RP2040_CLOCKS_CLK_GPOUT1_CTRL_PHASE_MASK             (0x03 << RP2040_CLOCKS_CLK_GPOUT1_CTRL_PHASE_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT1_CTRL_DC50                   (1 << 12)  /* Enables duty cycle correction for odd divisors */
#define RP2040_CLOCKS_CLK_GPOUT1_CTRL_ENABLE                 (1 << 11)  /* Starts and stops the clock generator cleanly */
#define RP2040_CLOCKS_CLK_GPOUT1_CTRL_KILL                   (1 << 10)  /* Asynchronously kills the clock generator */
#define RP2040_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_SHIFT           (5)        /* Selects the auxiliary clock source, will glitch when switching */
#define RP2040_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_MASK            (0x0f << RP2040_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_CLKSRC_PLL_SYS  (0x0 << RP2040_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_CLKSRC_GPIN0    (0x1 << RP2040_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_CLKSRC_GPIN1    (0x2 << RP2040_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_CLKSRC_PLL_USB  (0x3 << RP2040_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_ROSC_CLKSRC     (0x4 << RP2040_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_XOSC_CLKSRC     (0x5 << RP2040_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_CLK_SYS         (0x6 << RP2040_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_CLK_USB         (0x7 << RP2040_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_CLK_ADC         (0x8 << RP2040_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_CLK_RTC         (0x9 << RP2040_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_CLK_REF         (0xa << RP2040_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_SHIFT)

#define RP2040_CLOCKS_CLK_GPOUT1_DIV_INT_SHIFT               (8)        /* Integer component of the divisor, 0 -> divide by 2^16 */
#define RP2040_CLOCKS_CLK_GPOUT1_DIV_INT_MASK                (0xffffff << RP2040_CLOCKS_CLK_GPOUT1_DIV_INT_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT1_DIV_FRAC_MASK               (0xff)     /* Fractional component of the divisor */

#define RP2040_CLOCKS_CLK_GPOUT2_CTRL_NUDGE                  (1 << 20)  /* An edge on this signal shifts the phase of the output by 1 cycle of the input clock This can be done at any time */
#define RP2040_CLOCKS_CLK_GPOUT2_CTRL_PHASE_SHIFT            (16)       /* This delays the enable signal by up to 3 cycles of the input clock This must be set before the clock is enabled to have any effect */
#define RP2040_CLOCKS_CLK_GPOUT2_CTRL_PHASE_MASK             (0x03 << RP2040_CLOCKS_CLK_GPOUT2_CTRL_PHASE_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT2_CTRL_DC50                   (1 << 12)  /* Enables duty cycle correction for odd divisors */
#define RP2040_CLOCKS_CLK_GPOUT2_CTRL_ENABLE                 (1 << 11)  /* Starts and stops the clock generator cleanly */
#define RP2040_CLOCKS_CLK_GPOUT2_CTRL_KILL                   (1 << 10)  /* Asynchronously kills the clock generator */
#define RP2040_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_SHIFT           (5)        /* Selects the auxiliary clock source, will glitch when switching */
#define RP2040_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_MASK            (0x0f << RP2040_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_CLKSRC_PLL_SYS  (0x0 << RP2040_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_CLKSRC_GPIN0    (0x1 << RP2040_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_CLKSRC_GPIN1    (0x2 << RP2040_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_CLKSRC_PLL_USB  (0x3 << RP2040_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_ROSC_CLKSRC_PH  (0x4 << RP2040_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_XOSC_CLKSRC     (0x5 << RP2040_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_CLK_SYS         (0x6 << RP2040_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_CLK_USB         (0x7 << RP2040_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_CLK_ADC         (0x8 << RP2040_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_CLK_RTC         (0x9 << RP2040_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_CLK_REF         (0xa << RP2040_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_SHIFT)

#define RP2040_CLOCKS_CLK_GPOUT2_DIV_INT_SHIFT               (8)        /* Integer component of the divisor, 0 -> divide by 2^16 */
#define RP2040_CLOCKS_CLK_GPOUT2_DIV_INT_MASK                (0xffffff << RP2040_CLOCKS_CLK_GPOUT2_DIV_INT_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT2_DIV_FRAC_MASK               (0xff)     /* Fractional component of the divisor */

#define RP2040_CLOCKS_CLK_GPOUT3_CTRL_NUDGE                  (1 << 20)  /* An edge on this signal shifts the phase of the output by 1 cycle of the input clock This can be done at any time */
#define RP2040_CLOCKS_CLK_GPOUT3_CTRL_PHASE_SHIFT            (16)       /* This delays the enable signal by up to 3 cycles of the input clock This must be set before the clock is enabled to have any effect */
#define RP2040_CLOCKS_CLK_GPOUT3_CTRL_PHASE_MASK             (0x03 << RP2040_CLOCKS_CLK_GPOUT3_CTRL_PHASE_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT3_CTRL_DC50                   (1 << 12)  /* Enables duty cycle correction for odd divisors */
#define RP2040_CLOCKS_CLK_GPOUT3_CTRL_ENABLE                 (1 << 11)  /* Starts and stops the clock generator cleanly */
#define RP2040_CLOCKS_CLK_GPOUT3_CTRL_KILL                   (1 << 10)  /* Asynchronously kills the clock generator */
#define RP2040_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_SHIFT           (5)        /* Selects the auxiliary clock source, will glitch when switching */
#define RP2040_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_MASK            (0x0f << RP2040_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_CLKSRC_PLL_SYS  (0x0 << RP2040_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_CLKSRC_GPIN0    (0x1 << RP2040_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_CLKSRC_GPIN1    (0x2 << RP2040_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_CLKSRC_PLL_USB  (0x3 << RP2040_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_ROSC_CLKSRC_PH  (0x4 << RP2040_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_XOSC_CLKSRC     (0x5 << RP2040_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_CLK_SYS         (0x6 << RP2040_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_CLK_USB         (0x7 << RP2040_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_CLK_ADC         (0x8 << RP2040_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_CLK_RTC         (0x9 << RP2040_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_CLK_REF         (0xa << RP2040_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_SHIFT)

#define RP2040_CLOCKS_CLK_GPOUT3_DIV_INT_SHIFT               (8)        /* Integer component of the divisor, 0 -> divide by 2^16 */
#define RP2040_CLOCKS_CLK_GPOUT3_DIV_INT_MASK                (0xffffff << RP2040_CLOCKS_CLK_GPOUT3_DIV_INT_SHIFT)
#define RP2040_CLOCKS_CLK_GPOUT3_DIV_FRAC_MASK               (0xff)     /* Fractional component of the divisor */

#define RP2040_CLOCKS_CLK_REF_CTRL_AUXSRC_SHIFT              (5)        /* Selects the auxiliary clock source, will glitch when switching */
#define RP2040_CLOCKS_CLK_REF_CTRL_AUXSRC_MASK               (0x03 << RP2040_CLOCKS_CLK_REF_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_REF_CTRL_AUXSRC_CLKSRC_PLL_USB     (0x0 << RP2040_CLOCKS_CLK_REF_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_REF_CTRL_AUXSRC_CLKSRC_GPIN0       (0x1 << RP2040_CLOCKS_CLK_REF_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_REF_CTRL_AUXSRC_CLKSRC_GPIN1       (0x2 << RP2040_CLOCKS_CLK_REF_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_REF_CTRL_SRC_MASK                  (0x03)
#define RP2040_CLOCKS_CLK_REF_CTRL_SRC_ROSC_CLKSRC_PH        (0x0)
#define RP2040_CLOCKS_CLK_REF_CTRL_SRC_CLKSRC_CLK_REF_AUX    (0x1)
#define RP2040_CLOCKS_CLK_REF_CTRL_SRC_XOSC_CLKSRC           (0x2)

#define RP2040_CLOCKS_CLK_REF_DIV_INT_SHIFT                  (8)        /* Integer component of the divisor, 0 -> divide by 2^16 */
#define RP2040_CLOCKS_CLK_REF_DIV_INT_MASK                   (0x03 << RP2040_CLOCKS_CLK_REF_DIV_INT_SHIFT)

#define RP2040_CLOCKS_CLK_SYS_CTRL_AUXSRC_SHIFT              (5)        /* Selects the auxiliary clock source, will glitch when switching */
#define RP2040_CLOCKS_CLK_SYS_CTRL_AUXSRC_MASK               (0x07 << RP2040_CLOCKS_CLK_SYS_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_SYS_CTRL_AUXSRC_CLKSRC_PLL_SYS     (0x0 << RP2040_CLOCKS_CLK_SYS_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_SYS_CTRL_AUXSRC_CLKSRC_PLL_USB     (0x1 << RP2040_CLOCKS_CLK_SYS_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_SYS_CTRL_AUXSRC_ROSC_CLKSRC        (0x2 << RP2040_CLOCKS_CLK_SYS_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_SYS_CTRL_AUXSRC_XOSC_CLKSRC        (0x3 << RP2040_CLOCKS_CLK_SYS_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_SYS_CTRL_AUXSRC_CLKSRC_GPIN0       (0x4 << RP2040_CLOCKS_CLK_SYS_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_SYS_CTRL_AUXSRC_CLKSRC_GPIN1       (0x5 << RP2040_CLOCKS_CLK_SYS_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_SYS_CTRL_SRC                       (1 << 0)
#define RP2040_CLOCKS_CLK_SYS_CTRL_SRC_CLKSRC_CLK_SYS_AUX    (0x1)

#define RP2040_CLOCKS_CLK_SYS_DIV_INT_SHIFT                  (8)  /* Integer component of the divisor, 0 -> divide by 2^16 */
#define RP2040_CLOCKS_CLK_SYS_DIV_INT_MASK                   (0xffffff << RP2040_CLOCKS_CLK_SYS_DIV_INT_SHIFT)
#define RP2040_CLOCKS_CLK_SYS_DIV_FRAC_MASK                  (0xff)     /* Fractional component of the divisor */

#define RP2040_CLOCKS_CLK_PERI_CTRL_ENABLE                   (1 << 11)  /* Starts and stops the clock generator cleanly */
#define RP2040_CLOCKS_CLK_PERI_CTRL_KILL                     (1 << 10)  /* Asynchronously kills the clock generator */
#define RP2040_CLOCKS_CLK_PERI_CTRL_AUXSRC_SHIFT             (5)        /* Selects the auxiliary clock source, will glitch when switching */
#define RP2040_CLOCKS_CLK_PERI_CTRL_AUXSRC_MASK              (0x07 << RP2040_CLOCKS_CLK_PERI_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_PERI_CTRL_AUXSRC_CLK_SYS           (0x0 << RP2040_CLOCKS_CLK_PERI_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_PERI_CTRL_AUXSRC_CLKSRC_PLL_SYS    (0x1 << RP2040_CLOCKS_CLK_PERI_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_PERI_CTRL_AUXSRC_CLKSRC_PLL_USB    (0x2 << RP2040_CLOCKS_CLK_PERI_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_PERI_CTRL_AUXSRC_ROSC_CLKSRC_PH    (0x3 << RP2040_CLOCKS_CLK_PERI_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_PERI_CTRL_AUXSRC_XOSC_CLKSRC       (0x4 << RP2040_CLOCKS_CLK_PERI_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_PERI_CTRL_AUXSRC_CLKSRC_GPIN0      (0x5 << RP2040_CLOCKS_CLK_PERI_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_PERI_CTRL_AUXSRC_CLKSRC_GPIN1      (0x6 << RP2040_CLOCKS_CLK_PERI_CTRL_AUXSRC_SHIFT)

#define RP2040_CLOCKS_CLK_USB_CTRL_NUDGE                     (1 << 20)  /* An edge on this signal shifts the phase of the output by 1 cycle of the input clock This can be done at any time */
#define RP2040_CLOCKS_CLK_USB_CTRL_PHASE_SHIFT               (16)       /* This delays the enable signal by up to 3 cycles of the input clock This must be set before the clock is enabled to have any effect */
#define RP2040_CLOCKS_CLK_USB_CTRL_PHASE_MASK                (0x03 << RP2040_CLOCKS_CLK_USB_CTRL_PHASE_SHIFT)
#define RP2040_CLOCKS_CLK_USB_CTRL_ENABLE                    (1 << 11)  /* Starts and stops the clock generator cleanly */
#define RP2040_CLOCKS_CLK_USB_CTRL_KILL                      (1 << 10)  /* Asynchronously kills the clock generator */
#define RP2040_CLOCKS_CLK_USB_CTRL_AUXSRC_SHIFT              (5)        /* Selects the auxiliary clock source, will glitch when switching */
#define RP2040_CLOCKS_CLK_USB_CTRL_AUXSRC_MASK               (0x07 << RP2040_CLOCKS_CLK_USB_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_USB_CTRL_AUXSRC_CLKSRC_PLL_USB     (0x0 << RP2040_CLOCKS_CLK_USB_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_USB_CTRL_AUXSRC_CLKSRC_PLL_SYS     (0x1 << RP2040_CLOCKS_CLK_USB_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_USB_CTRL_AUXSRC_ROSC_CLKSRC_PH     (0x2 << RP2040_CLOCKS_CLK_USB_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_USB_CTRL_AUXSRC_XOSC_CLKSRC        (0x3 << RP2040_CLOCKS_CLK_USB_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_USB_CTRL_AUXSRC_CLKSRC_GPIN0       (0x4 << RP2040_CLOCKS_CLK_USB_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_USB_CTRL_AUXSRC_CLKSRC_GPIN1       (0x5 << RP2040_CLOCKS_CLK_USB_CTRL_AUXSRC_SHIFT)

#define RP2040_CLOCKS_CLK_USB_DIV_INT_SHIFT                  (8)        /* Integer component of the divisor, 0 -> divide by 2^16 */
#define RP2040_CLOCKS_CLK_USB_DIV_INT_MASK                   (0x03 << RP2040_CLOCKS_CLK_USB_DIV_INT_SHIFT)

#define RP2040_CLOCKS_CLK_ADC_CTRL_NUDGE                     (1 << 20)  /* An edge on this signal shifts the phase of the output by 1 cycle of the input clock This can be done at any time */
#define RP2040_CLOCKS_CLK_ADC_CTRL_PHASE_SHIFT               (16)       /* This delays the enable signal by up to 3 cycles of the input clock This must be set before the clock is enabled to have any effect */
#define RP2040_CLOCKS_CLK_ADC_CTRL_PHASE_MASK                (0x03 << RP2040_CLOCKS_CLK_ADC_CTRL_PHASE_SHIFT)
#define RP2040_CLOCKS_CLK_ADC_CTRL_ENABLE                    (1 << 11)  /* Starts and stops the clock generator cleanly */
#define RP2040_CLOCKS_CLK_ADC_CTRL_KILL                      (1 << 10)  /* Asynchronously kills the clock generator */
#define RP2040_CLOCKS_CLK_ADC_CTRL_AUXSRC_SHIFT              (5)        /* Selects the auxiliary clock source, will glitch when switching */
#define RP2040_CLOCKS_CLK_ADC_CTRL_AUXSRC_MASK               (0x07 << RP2040_CLOCKS_CLK_ADC_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_ADC_CTRL_AUXSRC_CLKSRC_PLL_USB     (0x0 << RP2040_CLOCKS_CLK_ADC_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_ADC_CTRL_AUXSRC_CLKSRC_PLL_SYS     (0x1 << RP2040_CLOCKS_CLK_ADC_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_ADC_CTRL_AUXSRC_ROSC_CLKSRC_PH     (0x2 << RP2040_CLOCKS_CLK_ADC_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_ADC_CTRL_AUXSRC_XOSC_CLKSRC        (0x3 << RP2040_CLOCKS_CLK_ADC_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_ADC_CTRL_AUXSRC_CLKSRC_GPIN0       (0x4 << RP2040_CLOCKS_CLK_ADC_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_ADC_CTRL_AUXSRC_CLKSRC_GPIN1       (0x5 << RP2040_CLOCKS_CLK_ADC_CTRL_AUXSRC_SHIFT)

#define RP2040_CLOCKS_CLK_ADC_DIV_INT_SHIFT                  (8)        /* Integer component of the divisor, 0 -> divide by 2^16 */
#define RP2040_CLOCKS_CLK_ADC_DIV_INT_MASK                   (0x03 << RP2040_CLOCKS_CLK_ADC_DIV_INT_SHIFT)

#define RP2040_CLOCKS_CLK_RTC_CTRL_NUDGE                     (1 << 20)  /* An edge on this signal shifts the phase of the output by 1 cycle of the input clock This can be done at any time */
#define RP2040_CLOCKS_CLK_RTC_CTRL_PHASE_SHIFT               (16)       /* This delays the enable signal by up to 3 cycles of the input clock This must be set before the clock is enabled to have any effect */
#define RP2040_CLOCKS_CLK_RTC_CTRL_PHASE_MASK                (0x03 << RP2040_CLOCKS_CLK_RTC_CTRL_PHASE_SHIFT)
#define RP2040_CLOCKS_CLK_RTC_CTRL_ENABLE                    (1 << 11)  /* Starts and stops the clock generator cleanly */
#define RP2040_CLOCKS_CLK_RTC_CTRL_KILL                      (1 << 10)  /* Asynchronously kills the clock generator */
#define RP2040_CLOCKS_CLK_RTC_CTRL_AUXSRC_SHIFT              (5)        /* Selects the auxiliary clock source, will glitch when switching */
#define RP2040_CLOCKS_CLK_RTC_CTRL_AUXSRC_MASK               (0x07 << RP2040_CLOCKS_CLK_RTC_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_RTC_CTRL_AUXSRC_CLKSRC_PLL_USB     (0x0 << RP2040_CLOCKS_CLK_RTC_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_RTC_CTRL_AUXSRC_CLKSRC_PLL_SYS     (0x1 << RP2040_CLOCKS_CLK_RTC_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_RTC_CTRL_AUXSRC_ROSC_CLKSRC_PH     (0x2 << RP2040_CLOCKS_CLK_RTC_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_RTC_CTRL_AUXSRC_XOSC_CLKSRC        (0x3 << RP2040_CLOCKS_CLK_RTC_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_RTC_CTRL_AUXSRC_CLKSRC_GPIN0       (0x4 << RP2040_CLOCKS_CLK_RTC_CTRL_AUXSRC_SHIFT)
#define RP2040_CLOCKS_CLK_RTC_CTRL_AUXSRC_CLKSRC_GPIN1       (0x5 << RP2040_CLOCKS_CLK_RTC_CTRL_AUXSRC_SHIFT)

#define RP2040_CLOCKS_CLK_RTC_DIV_INT_SHIFT                  (8)        /* Integer component of the divisor, 0 -> divide by 2^16 */
#define RP2040_CLOCKS_CLK_RTC_DIV_INT_MASK                   (0xffffff << RP2040_CLOCKS_CLK_RTC_DIV_INT_SHIFT)
#define RP2040_CLOCKS_CLK_RTC_DIV_FRAC_MASK                  (0xff)     /* Fractional component of the divisor */

#define RP2040_CLOCKS_CLK_SYS_RESUS_CTRL_CLEAR               (1 << 16)  /* For clearing the resus after the fault that triggered it has been corrected */
#define RP2040_CLOCKS_CLK_SYS_RESUS_CTRL_FRCE                (1 << 12)  /* Force a resus, for test purposes only */
#define RP2040_CLOCKS_CLK_SYS_RESUS_CTRL_ENABLE              (1 << 8)   /* Enable resus */
#define RP2040_CLOCKS_CLK_SYS_RESUS_CTRL_TIMEOUT_MASK        (0xff)     /* This is expressed as a number of clk_ref cycles and must be >= 2x clk_ref_freq/min_clk_tst_freq */

#define RP2040_CLOCKS_CLK_SYS_RESUS_STATUS_RESUSSED          (1 << 0)   /* Clock has been resuscitated, correct the error then send ctrl_clear=1 */

#define RP2040_CLOCKS_FC0_REF_KHZ_MASK                       (0xfffff)

#define RP2040_CLOCKS_FC0_MIN_KHZ_MASK                       (0x1ffffff)

#define RP2040_CLOCKS_FC0_MAX_KHZ_MASK                       (0x1ffffff)

#define RP2040_CLOCKS_FC0_DELAY_MASK                         (0x07)

#define RP2040_CLOCKS_FC0_INTERVAL_MASK                      (0x0f)

#define RP2040_CLOCKS_FC0_SRC_MASK                           (0xff)
#define RP2040_CLOCKS_FC0_SRC_NULL                           (0x0)
#define RP2040_CLOCKS_FC0_SRC_PLL_SYS_CLKSRC_PRIMARY         (0x1)
#define RP2040_CLOCKS_FC0_SRC_PLL_USB_CLKSRC_PRIMARY         (0x2)
#define RP2040_CLOCKS_FC0_SRC_ROSC_CLKSRC                    (0x3)
#define RP2040_CLOCKS_FC0_SRC_ROSC_CLKSRC_PH                 (0x4)
#define RP2040_CLOCKS_FC0_SRC_XOSC_CLKSRC                    (0x5)
#define RP2040_CLOCKS_FC0_SRC_CLKSRC_GPIN0                   (0x6)
#define RP2040_CLOCKS_FC0_SRC_CLKSRC_GPIN1                   (0x7)
#define RP2040_CLOCKS_FC0_SRC_CLK_REF                        (0x8)
#define RP2040_CLOCKS_FC0_SRC_CLK_SYS                        (0x9)
#define RP2040_CLOCKS_FC0_SRC_CLK_PERI                       (0xa)
#define RP2040_CLOCKS_FC0_SRC_CLK_USB                        (0xb)
#define RP2040_CLOCKS_FC0_SRC_CLK_ADC                        (0xc)
#define RP2040_CLOCKS_FC0_SRC_CLK_RTC                        (0xd)

#define RP2040_CLOCKS_FC0_STATUS_DIED                        (1 << 28)  /* Test clock stopped during test */
#define RP2040_CLOCKS_FC0_STATUS_FAST                        (1 << 24)  /* Test clock faster than expected, only valid when status_done=1 */
#define RP2040_CLOCKS_FC0_STATUS_SLOW                        (1 << 20)  /* Test clock slower than expected, only valid when status_done=1 */
#define RP2040_CLOCKS_FC0_STATUS_FAIL                        (1 << 16)  /* Test failed */
#define RP2040_CLOCKS_FC0_STATUS_WAITING                     (1 << 12)  /* Waiting for test clock to start */
#define RP2040_CLOCKS_FC0_STATUS_RUNNING                     (1 << 8)   /* Test running */
#define RP2040_CLOCKS_FC0_STATUS_DONE                        (1 << 4)   /* Test complete */
#define RP2040_CLOCKS_FC0_STATUS_PASS                        (1 << 0)   /* Test passed */

#define RP2040_CLOCKS_FC0_RESULT_KHZ_SHIFT                   (5)
#define RP2040_CLOCKS_FC0_RESULT_KHZ_MASK                    (0x1ffffff << RP2040_CLOCKS_FC0_RESULT_KHZ_SHIFT)
#define RP2040_CLOCKS_FC0_RESULT_FRAC_MASK                   (0x1f)

#define RP2040_CLOCKS_WAKE_EN0_clk_sys_sram3                 (1 << 31)
#define RP2040_CLOCKS_WAKE_EN0_clk_sys_sram2                 (1 << 30)
#define RP2040_CLOCKS_WAKE_EN0_clk_sys_sram1                 (1 << 29)
#define RP2040_CLOCKS_WAKE_EN0_clk_sys_sram0                 (1 << 28)
#define RP2040_CLOCKS_WAKE_EN0_clk_sys_spi1                  (1 << 27)
#define RP2040_CLOCKS_WAKE_EN0_clk_peri_spi1                 (1 << 26)
#define RP2040_CLOCKS_WAKE_EN0_clk_sys_spi0                  (1 << 25)
#define RP2040_CLOCKS_WAKE_EN0_clk_peri_spi0                 (1 << 24)
#define RP2040_CLOCKS_WAKE_EN0_clk_sys_sio                   (1 << 23)
#define RP2040_CLOCKS_WAKE_EN0_clk_sys_rtc                   (1 << 22)
#define RP2040_CLOCKS_WAKE_EN0_clk_rtc_rtc                   (1 << 21)
#define RP2040_CLOCKS_WAKE_EN0_clk_sys_rosc                  (1 << 20)
#define RP2040_CLOCKS_WAKE_EN0_clk_sys_rom                   (1 << 19)
#define RP2040_CLOCKS_WAKE_EN0_clk_sys_resets                (1 << 18)
#define RP2040_CLOCKS_WAKE_EN0_clk_sys_pwm                   (1 << 17)
#define RP2040_CLOCKS_WAKE_EN0_clk_sys_psm                   (1 << 16)
#define RP2040_CLOCKS_WAKE_EN0_clk_sys_pll_usb               (1 << 15)
#define RP2040_CLOCKS_WAKE_EN0_clk_sys_pll_sys               (1 << 14)
#define RP2040_CLOCKS_WAKE_EN0_clk_sys_pio1                  (1 << 13)
#define RP2040_CLOCKS_WAKE_EN0_clk_sys_pio0                  (1 << 12)
#define RP2040_CLOCKS_WAKE_EN0_clk_sys_pads                  (1 << 11)
#define RP2040_CLOCKS_WAKE_EN0_clk_sys_vreg_and_chip_reset   (1 << 10)
#define RP2040_CLOCKS_WAKE_EN0_clk_sys_jtag                  (1 << 9)
#define RP2040_CLOCKS_WAKE_EN0_clk_sys_io                    (1 << 8)
#define RP2040_CLOCKS_WAKE_EN0_clk_sys_i2c1                  (1 << 7)
#define RP2040_CLOCKS_WAKE_EN0_clk_sys_i2c0                  (1 << 6)
#define RP2040_CLOCKS_WAKE_EN0_clk_sys_dma                   (1 << 5)
#define RP2040_CLOCKS_WAKE_EN0_clk_sys_busfabric             (1 << 4)
#define RP2040_CLOCKS_WAKE_EN0_clk_sys_busctrl               (1 << 3)
#define RP2040_CLOCKS_WAKE_EN0_clk_sys_adc                   (1 << 2)
#define RP2040_CLOCKS_WAKE_EN0_clk_adc_adc                   (1 << 1)
#define RP2040_CLOCKS_WAKE_EN0_clk_sys_clocks                (1 << 0)

#define RP2040_CLOCKS_WAKE_EN1_clk_sys_xosc                  (1 << 14)
#define RP2040_CLOCKS_WAKE_EN1_clk_sys_xip                   (1 << 13)
#define RP2040_CLOCKS_WAKE_EN1_clk_sys_watchdog              (1 << 12)
#define RP2040_CLOCKS_WAKE_EN1_clk_usb_usbctrl               (1 << 11)
#define RP2040_CLOCKS_WAKE_EN1_clk_sys_usbctrl               (1 << 10)
#define RP2040_CLOCKS_WAKE_EN1_clk_sys_uart1                 (1 << 9)
#define RP2040_CLOCKS_WAKE_EN1_clk_peri_uart1                (1 << 8)
#define RP2040_CLOCKS_WAKE_EN1_clk_sys_uart0                 (1 << 7)
#define RP2040_CLOCKS_WAKE_EN1_clk_peri_uart0                (1 << 6)
#define RP2040_CLOCKS_WAKE_EN1_clk_sys_timer                 (1 << 5)
#define RP2040_CLOCKS_WAKE_EN1_clk_sys_tbman                 (1 << 4)
#define RP2040_CLOCKS_WAKE_EN1_clk_sys_sysinfo               (1 << 3)
#define RP2040_CLOCKS_WAKE_EN1_clk_sys_syscfg                (1 << 2)
#define RP2040_CLOCKS_WAKE_EN1_clk_sys_sram5                 (1 << 1)
#define RP2040_CLOCKS_WAKE_EN1_clk_sys_sram4                 (1 << 0)

#define RP2040_CLOCKS_SLEEP_EN0_clk_sys_sram3                (1 << 31)
#define RP2040_CLOCKS_SLEEP_EN0_clk_sys_sram2                (1 << 30)
#define RP2040_CLOCKS_SLEEP_EN0_clk_sys_sram1                (1 << 29)
#define RP2040_CLOCKS_SLEEP_EN0_clk_sys_sram0                (1 << 28)
#define RP2040_CLOCKS_SLEEP_EN0_clk_sys_spi1                 (1 << 27)
#define RP2040_CLOCKS_SLEEP_EN0_clk_peri_spi1                (1 << 26)
#define RP2040_CLOCKS_SLEEP_EN0_clk_sys_spi0                 (1 << 25)
#define RP2040_CLOCKS_SLEEP_EN0_clk_peri_spi0                (1 << 24)
#define RP2040_CLOCKS_SLEEP_EN0_clk_sys_sio                  (1 << 23)
#define RP2040_CLOCKS_SLEEP_EN0_clk_sys_rtc                  (1 << 22)
#define RP2040_CLOCKS_SLEEP_EN0_clk_rtc_rtc                  (1 << 21)
#define RP2040_CLOCKS_SLEEP_EN0_clk_sys_rosc                 (1 << 20)
#define RP2040_CLOCKS_SLEEP_EN0_clk_sys_rom                  (1 << 19)
#define RP2040_CLOCKS_SLEEP_EN0_clk_sys_resets               (1 << 18)
#define RP2040_CLOCKS_SLEEP_EN0_clk_sys_pwm                  (1 << 17)
#define RP2040_CLOCKS_SLEEP_EN0_clk_sys_psm                  (1 << 16)
#define RP2040_CLOCKS_SLEEP_EN0_clk_sys_pll_usb              (1 << 15)
#define RP2040_CLOCKS_SLEEP_EN0_clk_sys_pll_sys              (1 << 14)
#define RP2040_CLOCKS_SLEEP_EN0_clk_sys_pio1                 (1 << 13)
#define RP2040_CLOCKS_SLEEP_EN0_clk_sys_pio0                 (1 << 12)
#define RP2040_CLOCKS_SLEEP_EN0_clk_sys_pads                 (1 << 11)
#define RP2040_CLOCKS_SLEEP_EN0_clk_sys_vreg_and_chip_reset  (1 << 10)
#define RP2040_CLOCKS_SLEEP_EN0_clk_sys_jtag                 (1 << 9)
#define RP2040_CLOCKS_SLEEP_EN0_clk_sys_io                   (1 << 8)
#define RP2040_CLOCKS_SLEEP_EN0_clk_sys_i2c1                 (1 << 7)
#define RP2040_CLOCKS_SLEEP_EN0_clk_sys_i2c0                 (1 << 6)
#define RP2040_CLOCKS_SLEEP_EN0_clk_sys_dma                  (1 << 5)
#define RP2040_CLOCKS_SLEEP_EN0_clk_sys_busfabric            (1 << 4)
#define RP2040_CLOCKS_SLEEP_EN0_clk_sys_busctrl              (1 << 3)
#define RP2040_CLOCKS_SLEEP_EN0_clk_sys_adc                  (1 << 2)
#define RP2040_CLOCKS_SLEEP_EN0_clk_adc_adc                  (1 << 1)
#define RP2040_CLOCKS_SLEEP_EN0_clk_sys_clocks               (1 << 0)

#define RP2040_CLOCKS_SLEEP_EN1_clk_sys_xosc                 (1 << 14)
#define RP2040_CLOCKS_SLEEP_EN1_clk_sys_xip                  (1 << 13)
#define RP2040_CLOCKS_SLEEP_EN1_clk_sys_watchdog             (1 << 12)
#define RP2040_CLOCKS_SLEEP_EN1_clk_usb_usbctrl              (1 << 11)
#define RP2040_CLOCKS_SLEEP_EN1_clk_sys_usbctrl              (1 << 10)
#define RP2040_CLOCKS_SLEEP_EN1_clk_sys_uart1                (1 << 9)
#define RP2040_CLOCKS_SLEEP_EN1_clk_peri_uart1               (1 << 8)
#define RP2040_CLOCKS_SLEEP_EN1_clk_sys_uart0                (1 << 7)
#define RP2040_CLOCKS_SLEEP_EN1_clk_peri_uart0               (1 << 6)
#define RP2040_CLOCKS_SLEEP_EN1_clk_sys_timer                (1 << 5)
#define RP2040_CLOCKS_SLEEP_EN1_clk_sys_tbman                (1 << 4)
#define RP2040_CLOCKS_SLEEP_EN1_clk_sys_sysinfo              (1 << 3)
#define RP2040_CLOCKS_SLEEP_EN1_clk_sys_syscfg               (1 << 2)
#define RP2040_CLOCKS_SLEEP_EN1_clk_sys_sram5                (1 << 1)
#define RP2040_CLOCKS_SLEEP_EN1_clk_sys_sram4                (1 << 0)

#define RP2040_CLOCKS_ENABLED0_clk_sys_sram3                 (1 << 31)
#define RP2040_CLOCKS_ENABLED0_clk_sys_sram2                 (1 << 30)
#define RP2040_CLOCKS_ENABLED0_clk_sys_sram1                 (1 << 29)
#define RP2040_CLOCKS_ENABLED0_clk_sys_sram0                 (1 << 28)
#define RP2040_CLOCKS_ENABLED0_clk_sys_spi1                  (1 << 27)
#define RP2040_CLOCKS_ENABLED0_clk_peri_spi1                 (1 << 26)
#define RP2040_CLOCKS_ENABLED0_clk_sys_spi0                  (1 << 25)
#define RP2040_CLOCKS_ENABLED0_clk_peri_spi0                 (1 << 24)
#define RP2040_CLOCKS_ENABLED0_clk_sys_sio                   (1 << 23)
#define RP2040_CLOCKS_ENABLED0_clk_sys_rtc                   (1 << 22)
#define RP2040_CLOCKS_ENABLED0_clk_rtc_rtc                   (1 << 21)
#define RP2040_CLOCKS_ENABLED0_clk_sys_rosc                  (1 << 20)
#define RP2040_CLOCKS_ENABLED0_clk_sys_rom                   (1 << 19)
#define RP2040_CLOCKS_ENABLED0_clk_sys_resets                (1 << 18)
#define RP2040_CLOCKS_ENABLED0_clk_sys_pwm                   (1 << 17)
#define RP2040_CLOCKS_ENABLED0_clk_sys_psm                   (1 << 16)
#define RP2040_CLOCKS_ENABLED0_clk_sys_pll_usb               (1 << 15)
#define RP2040_CLOCKS_ENABLED0_clk_sys_pll_sys               (1 << 14)
#define RP2040_CLOCKS_ENABLED0_clk_sys_pio1                  (1 << 13)
#define RP2040_CLOCKS_ENABLED0_clk_sys_pio0                  (1 << 12)
#define RP2040_CLOCKS_ENABLED0_clk_sys_pads                  (1 << 11)
#define RP2040_CLOCKS_ENABLED0_clk_sys_vreg_and_chip_reset   (1 << 10)
#define RP2040_CLOCKS_ENABLED0_clk_sys_jtag                  (1 << 9)
#define RP2040_CLOCKS_ENABLED0_clk_sys_io                    (1 << 8)
#define RP2040_CLOCKS_ENABLED0_clk_sys_i2c1                  (1 << 7)
#define RP2040_CLOCKS_ENABLED0_clk_sys_i2c0                  (1 << 6)
#define RP2040_CLOCKS_ENABLED0_clk_sys_dma                   (1 << 5)
#define RP2040_CLOCKS_ENABLED0_clk_sys_busfabric             (1 << 4)
#define RP2040_CLOCKS_ENABLED0_clk_sys_busctrl               (1 << 3)
#define RP2040_CLOCKS_ENABLED0_clk_sys_adc                   (1 << 2)
#define RP2040_CLOCKS_ENABLED0_clk_adc_adc                   (1 << 1)
#define RP2040_CLOCKS_ENABLED0_clk_sys_clocks                (1 << 0)

#define RP2040_CLOCKS_ENABLED1_clk_sys_xosc                  (1 << 14)
#define RP2040_CLOCKS_ENABLED1_clk_sys_xip                   (1 << 13)
#define RP2040_CLOCKS_ENABLED1_clk_sys_watchdog              (1 << 12)
#define RP2040_CLOCKS_ENABLED1_clk_usb_usbctrl               (1 << 11)
#define RP2040_CLOCKS_ENABLED1_clk_sys_usbctrl               (1 << 10)
#define RP2040_CLOCKS_ENABLED1_clk_sys_uart1                 (1 << 9)
#define RP2040_CLOCKS_ENABLED1_clk_peri_uart1                (1 << 8)
#define RP2040_CLOCKS_ENABLED1_clk_sys_uart0                 (1 << 7)
#define RP2040_CLOCKS_ENABLED1_clk_peri_uart0                (1 << 6)
#define RP2040_CLOCKS_ENABLED1_clk_sys_timer                 (1 << 5)
#define RP2040_CLOCKS_ENABLED1_clk_sys_tbman                 (1 << 4)
#define RP2040_CLOCKS_ENABLED1_clk_sys_sysinfo               (1 << 3)
#define RP2040_CLOCKS_ENABLED1_clk_sys_syscfg                (1 << 2)
#define RP2040_CLOCKS_ENABLED1_clk_sys_sram5                 (1 << 1)
#define RP2040_CLOCKS_ENABLED1_clk_sys_sram4                 (1 << 0)

#define RP2040_CLOCKS_INTR_CLK_SYS_RESUS                     (1 << 0)

#define RP2040_CLOCKS_INTE_CLK_SYS_RESUS                     (1 << 0)

#define RP2040_CLOCKS_INTF_CLK_SYS_RESUS                     (1 << 0)

#define RP2040_CLOCKS_INTS_CLK_SYS_RESUS                     (1 << 0)

#endif /* __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_CLOCKS_H */
