/****************************************************************************
 * drivers/audio/wm8904.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
 *
 * Reference:
 *   "WM8904 Ultra Low Power CODEC for Portable Audio Applications, Pre-
 *    Production", September 2012, Rev 3.3, Wolfson Microelectronics
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
 ****************************************************************************/

#ifndef __DRIVERS_AUDIO_WM8904_H
#define __DRIVERS_AUDIO_WM8904_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <pthread.h>
#include <mqueue.h>

#include <nuttx/wqueue.h>
#include <nuttx/fs/ioctl.h>

#ifdef CONFIG_AUDIO

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* So far, I have not been able to get FLL lock interrupts. Worse, I have
 * been able to get the FLL to claim that it is locked at all even when
 * polling.  What am I doing wrong?
 *
 * Hmmm.. seems unnecessary anyway
 */

#undef WM8904_USE_FFLOCK_INT
#undef WM8904_USE_FFLOCK_POLL

/* Registers Addresses ******************************************************/

#define WM8904_SWRST                 0x00 /* SW Reset and ID */
#define WM8904_ID                    0x00 /* SW Reset and ID */
#define WM8904_BIAS_CTRL             0x04 /* Bias Control */
#define WM8904_VMID_CTRL             0x05 /* VMID Control */
#define WM8904_MIC_BIAS_CTRL0        0x06 /* Mic Bias Control 0 */
#define WM8904_MIC_BIAS_CTRL1        0x07 /* Mic Bias Control 1 */
#define WM8904_ANALOG_ADC            0x0a /* Analogue ADC */
#define WM8904_PM0                   0x0c /* Power Management 0 */
#define WM8904_PM2                   0x0e /* Power Management 2 */
#define WM8904_PM3                   0x0f /* Power Management 3 */
#define WM8904_PM6                   0x12 /* Power Management 6 */
#define WM8904_CLKRATE0              0x14 /* Clock Rates 0 */
#define WM8904_CLKRATE1              0x15 /* Clock Rates 1 */
#define WM8904_CLKRATE2              0x16 /* Clock Rates 2 */
#define WM8904_AIF0                  0x18 /* Audio Interface 0 */
#define WM8904_AIF1                  0x19 /* Audio Interface 1 */
#define WM8904_AIF2                  0x1a /* Audio Interface 2 */
#define WM8904_AIF3                  0x1b /* Audio Interface 3 */
#define WM8904_DAC_VOL_LEFT          0x1e /* DAC Digital Volume Left */
#define WM8904_DAC_VOL_RIGHT         0x1f /* DAC Digital Volume Right */
#define WM8904_DAC_DIGI0             0x20 /* DAC Digital 0 */
#define WM8904_DAC_DIGI1             0x21 /* DAC Digital 1 */
#define WM8904_ADC_VOL_LEFT          0x24 /* ADC Digital Volume Left */
#define WM8904_ADC_VOL_RIGHT         0x25 /* ADC Digital Volume Right */
#define WM8904_ADC_DIGI              0x26 /* ADC Digital */
#define WM8904_MIC_DIGI              0x27 /* Digital Microphone */
#define WM8904_DRC0                  0x28 /* DRC 0 */
#define WM8904_DRC1                  0x29 /* DRC 1 */
#define WM8904_DRC2                  0x2a /* DRC 2 */
#define WM8904_DRC3                  0x2b /* DRC 3 */
#define WM8904_ANA_LEFT_IN0          0x2c /* Analogue Left Input 0 */
#define WM8904_ANA_RIGHT_IN0         0x2d /* Analogue Right Input 0 */
#define WM8904_ANA_LEFT_IN1          0x2e /* Analogue Left Input 1 */
#define WM8904_ANA_RIGHT_IN1         0x2f /* Analogue Right Input 1 */
#define WM8904_ANA_LEFT_OUT1         0x39 /* Analogue OUT1 Left */
#define WM8904_ANA_RIGHT_OUT1        0x3a /* Analogue OUT1 Right */
#define WM8904_ANA_LEFT_OUT2         0x3b /* Analogue OUT2 Left */
#define WM8904_ANA_RIGHT_OUT2        0x3c /* Analogue OUT2 Right */
#define WM8904_ANA_OUT12_ZC          0x3d /* Analogue OUT12 ZC */
#define WM8904_DC_SERVO0             0x43 /* DC Servo 0 */
#define WM8904_DC_SERVO1             0x44 /* DC Servo 1 */
#define WM8904_DC_SERVO2             0x45 /* DC Servo 2 */
#define WM8904_DC_SERVO4             0x47 /* DC Servo 4 */
#define WM8904_DC_SERVO5             0x48 /* DC Servo 5 */
#define WM8904_DC_SERVO6             0x49 /* DC Servo 6 */
#define WM8904_DC_SERVO7             0x4a /* DC Servo 7 */
#define WM8904_DC_SERVO8             0x4b /* DC Servo 8 */
#define WM8904_DC_SERVO9             0x4c /* DC Servo 9 */
#define WM8904_DC_SERVO_RDBACK       0x4d /* DC Servo Readback 0 */
#define WM8904_ANA_HP0               0x5a /* Analogue HP 0 */
#define WM8904_ANA_LINEOUT0          0x5e /* Analogue Lineout 0 */
#define WM8904_CHG_PUMP0             0x62 /* Charge Pump 0 */
#define WM8904_CLASS_W0              0x68 /* Class W 0 */
#define WM8904_WR_SEQ0               0x6c /* Write Sequencer 0 */
#define WM8904_WR_SEQ1               0x6d /* Write Sequencer 1 */
#define WM8904_WR_SEQ2               0x6e /* Write Sequencer 2 */
#define WM8904_WR_SEQ3               0x6f /* Write Sequencer 3 */
#define WM8904_WR_SEQ4               0x70 /* Write Sequencer 4 */
#define WM8904_FLL_CTRL1             0x74 /* FLL Control 1 */
#define WM8904_FLL_CTRL2             0x75 /* FLL Control 2 */
#define WM8904_FLL_CTRL3             0x76 /* FLL Control 3 */
#define WM8904_FLL_CTRL4             0x77 /* FLL Control 4 */
#define WM8904_FLL_CTRL5             0x78 /* FLL Control 5 */
#define WM8904_GPIO_CTRL1            0x79 /* GPIO Control 1 */
#define WM8904_GPIO_CTRL2            0x7a /* GPIO Control 2 */
#define WM8904_GPIO_CTRL3            0x7b /* GPIO Control 3 */
#define WM8904_GPIO_CTRL4            0x7c /* GPIO Control 4 */
#define WM8904_DIGI_PULLS            0x7e /* Digital Pulls */
#define WM8904_INT_STATUS            0x7f /* Interrupt Status */
#define WM8904_INT_MASK              0x80 /* Interrupt Status Mask */
#define WM8904_INT_POL               0x81 /* Interrupt Polarity */
#define WM8904_INT_DEBOUNCE          0x82 /* Interrupt Debounce */
#define WM8904_EQ1                   0x86 /* EQ1 */
#define WM8904_EQ2                   0x87 /* EQ2 */
#define WM8904_EQ3                   0x88 /* EQ3 */
#define WM8904_EQ4                   0x89 /* EQ4 */
#define WM8904_EQ5                   0x8a /* EQ5 */
#define WM8904_EQ6                   0x8b /* EQ6 */
#define WM8904_EQ7                   0x8c /* EQ7 */
#define WM8904_EQ8                   0x8d /* EQ8 */
#define WM8904_EQ9                   0x8e /* EQ9 */
#define WM8904_EQ10                  0x8f /* EQ10 */
#define WM8904_EQ11                  0x90 /* EQ11 */
#define WM8904_EQ12                  0x91 /* EQ12 */
#define WM8904_EQ13                  0x92 /* EQ13 */
#define WM8904_EQ14                  0x93 /* EQ14 */
#define WM8904_EQ15                  0x94 /* EQ15 */
#define WM8904_EQ16                  0x95 /* EQ16 */
#define WM8904_EQ17                  0x96 /* EQ17 */
#define WM8904_EQ18                  0x97 /* EQ18 */
#define WM8904_EQ19                  0x98 /* EQ19 */
#define WM8904_EQ20                  0x99 /* EQ20 */
#define WM8904_EQ21                  0x9a /* EQ21 */
#define WM8904_EQ22                  0x9b /* EQ22 */
#define WM8904_EQ23                  0x9c /* EQ23 */
#define WM8904_EQ24                  0x9d /* EQ24 */
#define WM8904_CTRLIF_TEST_1         0xa1 /* Control Interface Test 1 */
#define WM8904_ADC_TEST              0xc6 /* ADC Test  */
#define WM8904_ANA_OUT_BIAS_0        0xCC /* Analogue Output Bias 0 */
#define WM8904_FLL_NCO_TEST0         0xf7 /* FLL NCO Test 0 */
#define WM8904_FLL_NCO_TEST1         0xf8 /* FLL NCO Test 1 */

#define WM8904_DUMMY                 0xff /* Dummy register address */

/* Register Default Values **************************************************/
/* Registers have some undocumented bits set on power up.  These probably
 * should be retained on writes (?).
 */

#define WM8904_SWRST_DEFAULT           0x8904
#define WM8904_ID_DEFAULT              0x0018
#define WM8904_BIAS_CTRL_DEFAULT       0x0000
#define WM8904_VMID_CTRL_DEFAULT       0x0000
#define WM8904_MIC_BIAS_CTRL0_DEFAULT  0x0000
#define WM8904_MIC_BIAS_CTRL1_DEFAULT  0x0000
#define WM8904_ANALOG_ADC_DEFAULT      0x0001
#define WM8904_PM0_DEFAULT             0x0000
#define WM8904_PM2_DEFAULT             0x0000
#define WM8904_PM3_DEFAULT             0x0000
#define WM8904_PM6_DEFAULT             0x0000
#define WM8904_CLKRATE0_DEFAULT        0x8c5e
#define WM8904_CLKRATE1_DEFAULT        0x0c05
#define WM8904_CLKRATE2_DEFAULT        0x0000
#define WM8904_AIF0_DEFAULT            0x0050
#define WM8904_AIF1_DEFAULT            0x000a
#define WM8904_AIF2_DEFAULT            0x00e4
#define WM8904_AIF3_DEFAULT            0x0040
#define WM8904_DAC_VOL_LEFT_DEFAULT    0x00c0
#define WM8904_DAC_VOL_RIGHT_DEFAULT   0x00c0
#define WM8904_DAC_DIGI0_DEFAULT       0x0000
#define WM8904_DAC_DIGI1_DEFAULT       0x0004
#define WM8904_ADC_VOL_LEFT_DEFAULT    0x00c0
#define WM8904_ADC_VOL_RIGHT_DEFAULT   0x00c0
#define WM8904_ADC_DIGI_DEFAULT        0x0010
#define WM8904_MIC_DIGI_DEFAULT        0x0000
#define WM8904_DRC0_DEFAULT            0x01af
#define WM8904_DRC1_DEFAULT            0x3248
#define WM8904_DRC2_DEFAULT            0x0000
#define WM8904_DRC3_DEFAULT            0x0000
#define WM8904_ANA_LEFT_IN0_DEFAULT    0x0085
#define WM8904_ANA_RIGHT_IN0_DEFAULT   0x0085
#define WM8904_ANA_LEFT_IN1_DEFAULT    0x0044
#define WM8904_ANA_RIGHT_IN1_DEFAULT   0x0044
#define WM8904_ANA_LEFT_OUT1_DEFAULT   0x002d
#define WM8904_ANA_RIGHT_OUT1_DEFAULT  0x002d
#define WM8904_ANA_LEFT_OUT2_DEFAULT   0x0039
#define WM8904_ANA_RIGHT_OUT2_DEFAULT  0x0039
#define WM8904_ANA_OUT12_ZC_DEFAULT    0x0000
#define WM8904_DC_SERVO0_DEFAULT       0x0000
#define WM8904_DC_SERVO2_DEFAULT       0xaaaa
#define WM8904_DC_SERVO4_DEFAULT       0xaaaa
#define WM8904_DC_SERVO5_DEFAULT       0xaaaa
#define WM8904_DC_SERVO6_DEFAULT       0x0000
#define WM8904_DC_SERVO7_DEFAULT       0x0000
#define WM8904_DC_SERVO8_DEFAULT       0x0000
#define WM8904_DC_SERVO9_DEFAULT       0x0000
#define WM8904_DC_SERVO_RDBACK_DEFAULT 0x0000
#define WM8904_ANA_HP0_DEFAULT         0x0000
#define WM8904_ANA_LINEOUT0_DEFAULT    0x0000
#define WM8904_CHG_PUMP0_DEFAULT       0x0000
#define WM8904_CLASS_W0_DEFAULT        0x0000
#define WM8904_WR_SEQ0_DEFAULT         0x0000
#define WM8904_WR_SEQ1_DEFAULT         0x0000
#define WM8904_WR_SEQ2_DEFAULT         0x0000
#define WM8904_WR_SEQ3_DEFAULT         0x0000
#define WM8904_WR_SEQ4_DEFAULT         0x0000
#define WM8904_FLL_CTRL1_DEFAULT       0x0000
#define WM8904_FLL_CTRL2_DEFAULT       0x0007
#define WM8904_FLL_CTRL3_DEFAULT       0x0000
#define WM8904_FLL_CTRL4_DEFAULT       0x2ee0
#define WM8904_FLL_CTRL5_DEFAULT       0x0004
#define WM8904_GPIO_CTRL1_DEFAULT      0x0014
#define WM8904_GPIO_CTRL2_DEFAULT      0x0010
#define WM8904_GPIO_CTRL3_DEFAULT      0x0010
#define WM8904_GPIO_CTRL4_DEFAULT      0x0000
#define WM8904_DIGI_PULLS_DEFAULT      0x0000
#define WM8904_INT_MASK_DEFAULT        0xffff
#define WM8904_INT_POL_DEFAULT         0x0000
#define WM8904_INT_DEBOUNCE_DEFAULT    0x0000
#define WM8904_EQ1_DEFAULT             0x0000
#define WM8904_EQ2_DEFAULT             0x000c
#define WM8904_EQ3_DEFAULT             0x000c
#define WM8904_EQ4_DEFAULT             0x000c
#define WM8904_EQ5_DEFAULT             0x000c
#define WM8904_EQ6_DEFAULT             0x000c
#define WM8904_EQ7_DEFAULT             0x0fca
#define WM8904_EQ8_DEFAULT             0x0400
#define WM8904_EQ9_DEFAULT             0x00d8
#define WM8904_EQ10_DEFAULT            0x1eb5
#define WM8904_EQ11_DEFAULT            0xf145
#define WM8904_EQ12_DEFAULT            0x0b75
#define WM8904_EQ13_DEFAULT            0x01c5
#define WM8904_EQ14_DEFAULT            0x1c54
#define WM8904_EQ15_DEFAULT            0xf373
#define WM8904_EQ16_DEFAULT            0x0a54
#define WM8904_EQ17_DEFAULT            0x0558
#define WM8904_EQ18_DEFAULT            0x168e
#define WM8904_EQ19_DEFAULT            0xf829
#define WM8904_EQ20_DEFAULT            0x07ad
#define WM8904_EQ21_DEFAULT            0x1103
#define WM8904_EQ22_DEFAULT            0x0564
#define WM8904_EQ23_DEFAULT            0x0559
#define WM8904_EQ24_DEFAULT            0x4000
#define WM8904_CTRLIF_TEST_1_DEFAULT   0x0000
#define WM8904_FLL_NCO_TEST0_DEFAULT   0x0000
#define WM8904_ANA_OUT_BIAS_0_DEFAULT  0x0000
#define WM8904_ADC_TEST_DEFAULT        0x0000
#define WM8904_FLL_NCO_TEST1_DEFAULT   0x0019

/* Register Bit Definitions *************************************************/

/* 0x00 SW Reset and ID */

#define WM8904_SW_RST_DEV_ID1        0x8904

/* 0x04 Bias Control */

#define WM8904_ISEL_SHIFT            (2)       /* Bits 2-3: Master Bias Control */
#define WM8904_ISEL_MASK             (3 << WM8904_ISEL_SHIFT)
#  define WM8904_ISEL_LOW            (0 << WM8904_ISEL_SHIFT) /* Low power bias */
#  define WM8904_ISEL_HIGH           (2 << WM8904_ISEL_SHIFT) /* High performance bias */
#define WM8904_BIAS_ENA              (1 << 0)  /* Bit 0:  Normal bias current generator */

/* 0x05 VMID Control */

#define WM8904_VMID_BUF_ENA          (1 << 6)  /* Bit 6:  Enable VMID buffer to unused I/O */
#define WM8904_VMID_RES_SHIFT        (1)       /* Bits 1-2: VMID divider enable and select */
#define WM8904_VMID_RES_MASK         (3 << WM8904_VMID_RES_SHIFT)
#  define WM8904_VMID_RES_OFF        (0 << WM8904_VMID_RES_SHIFT) /* VMID disabled */
#  define WM8904_VMID_RES_NORMAL     (1 << WM8904_VMID_RES_SHIFT) /* 2 x 50k divider */
#  define WM8904_VMID_RES_STANDBY    (2 << WM8904_VMID_RES_SHIFT) /* 2 x 250k divider */
#  define WM8904_VMID_RES_FASTSTART  (3 << WM8904_VMID_RES_SHIFT) /* 2 x 5k divider */
#define WM8904_VMID_ENA              (1 << 0)  /* Bit 0:  VMID buffer enable */

/* 0x06 Mic Bias Control 0 */

#define WM8904_MICDET_THR_SHIFT      (4)       /* Bits 4-6: MICBIAS current detect threshold */
#define WM8904_MICDET_THR_MASK       (7 << WM8904_MICDET_THR_SHIFT)
#  define WM8904_MICDET_THR(n)       ((uint16_t)(n) << WM8904_MICDET_THR_SHIFT)
#define WM8904_MICSHORT_THR_SHIFT    (2)       /* Bits 2-3: MICBIAS short circuit threshold */
#define WM8904_MICSHORT_THR_MASK     (3 << WM8904_MICSHORT_THR_SHIFT)
  #define WM8904_MICSHORT_THR(n)     ((uint16_t)(n) << WM8904_MICSHORT_THR_SHIFT)
#define WM8904_MICDET_ENA            (1 << 1)  /* Bit 1:  MICBIAS current/short circuit detect enable */
#define WM8904_MICBIAS_ENA           (1 << 0)  /* Bit 0:  MICBIAS enable */

/* 0x07 Mic Bias Control 1 */

#define WM8904_MICBIAS_SEL_MASK      (0x0007)  /* Bits 0-2: Selects MICBIAS voltage */

/* 0x0a Analogue ADC */

#define WM8904_ADC_OSR128            (1 << 0)  /* Bit 0:  ADC Oversampling Ratio */

/* 0x0c Power Management 0 */

#define WM8904_INL_ENA               (1 << 1)  /* Bit 1:  Left input PGA enable */
#define WM8904_INR_ENA               (1 << 0)  /* Bit 0:  Right input PGA enable */

/* 0x0e Power Management 2 */

#define WM8904_HPL_PGA_ENA           (1 << 1)  /* Bit 1:  Left headphone output enable */
#define WM8904_HPR_PGA_ENA           (1 << 0)  /* Bit 0:  Right headphone output enable */

/* 0x0f Power Management 3 */

#define WM8904_LINEOUTL_PGA_ENA      (1 << 1)  /* Bit 1:  Left line output enable */
#define WM8904_LINEOUTR_PGA_ENA      (1 << 0)  /* Bit 0:  Right line output enable */

/* 0x12 Power Management 6 */

#define WM8904_DACL_ENA              (1 << 3)  /* Bit 3:  Left DAC enable */
#define WM8904_DACR_ENA              (1 << 2)  /* Bit 2:  Right DAC enable */
#define WM8904_ADCL_ENA              (1 << 1)  /* Bit 1:  Left ADC enable */
#define WM8904_ADCR_ENA              (1 << 0)  /* Bit 0:  Right ADC enable */

/* 0x14 Clock Rates 0 */

#define WM8904_TOCLK_RATE_DIV16      (1 << 14) /* Bit 14: TOCLK Rate Divider (/16) */
#define WM8904_TOCLK_RATE_X4         (1 << 13) /* Bit 13: TOCLK Rate Multiplier */
#define WM8904_MCLK_DIV              (1 << 0)  /* Bit 0:  Enables divide by 2 on MCLK */
#  define WM8904_MCLK_DIV1           (0)       /*         0: SYSCLK = MCLK */
#  define WM8904_MCLK_DIV2           (1 << 0)  /*         1: SYSCLK = MCLK/2*/

/* 0x15 Clock Rates 1 */

#define WM8904_CLK_SYS_RATE_SHIFT    (10)     /* Bits 10-13: Selects the SYSCLK / fs ratio */
#define WM8904_CLK_SYS_RATE_MASK     (15 << WM8904_CLK_SYS_RATE_SHIFT)
#  define WM8904_CLK_SYS_RATE_DIV64    (0 << WM8904_CLK_SYS_RATE_SHIFT)
#  define WM8904_CLK_SYS_RATE_DIV128   (1 << WM8904_CLK_SYS_RATE_SHIFT)
#  define WM8904_CLK_SYS_RATE_DIV192   (2 << WM8904_CLK_SYS_RATE_SHIFT)
#  define WM8904_CLK_SYS_RATE_DIV256   (3 << WM8904_CLK_SYS_RATE_SHIFT)
#  define WM8904_CLK_SYS_RATE_DIV384   (4 << WM8904_CLK_SYS_RATE_SHIFT)
#  define WM8904_CLK_SYS_RATE_DIV512   (5 << WM8904_CLK_SYS_RATE_SHIFT)
#  define WM8904_CLK_SYS_RATE_DIV768   (6 << WM8904_CLK_SYS_RATE_SHIFT)
#  define WM8904_CLK_SYS_RATE_DIV1024  (7 << WM8904_CLK_SYS_RATE_SHIFT)
#  define WM8904_CLK_SYS_RATE_DIV1408  (8 << WM8904_CLK_SYS_RATE_SHIFT)
#  define WM8904_CLK_SYS_RATE_DIV1536  (9 << WM8904_CLK_SYS_RATE_SHIFT)
#define WM8904_SAMPLE_RATE_SHIFT     (0)      /* Bits 0-2: Selects the Sample Rate (fs) */
#define WM8904_SAMPLE_RATE_MASK      (3 << WM8904_SAMPLE_RATE_SHIFT)
#  define WM8904_SAMPLE_RATE_8KHZ    (0 << WM8904_SAMPLE_RATE_SHIFT) /* 8kHz */
#  define WM8904_SAMPLE_RATE_12KHZ   (1 << WM8904_SAMPLE_RATE_SHIFT) /* 11.025kHz, 12kHz */
#  define WM8904_SAMPLE_RATE_16KHZ   (2 << WM8904_SAMPLE_RATE_SHIFT) /* 16kHz */
#  define WM8904_SAMPLE_RATE_24KHZ   (3 << WM8904_SAMPLE_RATE_SHIFT) /* 22.05kHz, 24kHz */
#  define WM8904_SAMPLE_RATE_32KHZ   (4 << WM8904_SAMPLE_RATE_SHIFT) /* 32kHz */
#  define WM8904_SAMPLE_RATE_48KHZ   (5 << WM8904_SAMPLE_RATE_SHIFT) /* 44.1kHz, 48kHz */

/* 0x16 Clock Rates 2 */

#define WM8904_MCLK_INV              (1 << 15) /* Bit 15: MCLK invert */
#define WM8904_SYSCLK_SRC            (1 << 14) /* Bit 14: SYSCLK source select */
#  define WM8904_SYSCLK_SRCMCLK      (0)       /*         0: MCLK */
#  define WM8904_SYSCLK_SRCFLL       (1 << 14) /*         1: FLL Output */
#define WM8904_TOCLK_RATE            (1 << 12) /* Bit 12: TOCLK rate divider (/2) */
#define WM8904_OPCLK_ENA             (1 << 3)  /* Bit 3:  GPIO clock output enable */
#define WM8904_CLK_SYS_ENA           (1 << 2)  /* Bit 2:  System clock enable */
#define WM8904_CLK_DSP_ENA           (1 << 1)  /* Bit 1:  DSP clock enable */
#define WM8904_TOCLK_ENA             (1 << 0)  /* Bit 0:  Zero cross timeout enable */

/* 0x18 Audio Interface 0 */

#define WM8904_DACL_DATINV           (1 << 12) /* Bit 12: Left DAC invert */
#define WM8904_DACR_DATINV           (1 << 11) /* Bit 11: Right DAC invert */
#define WM8904_DAC_BOOST_SHIFT       (9)       /* Bits 9-10: DAC digital input volume boost */
#define WM8904_DAC_BOOST_MASK        (3 << WM8904_DAC_BOOST_SHIFT)
#  define WM8904_DAC_BOOST_0DB       (0 << WM8904_DAC_BOOST_SHIFT)
#  define WM8904_DAC_BOOST_6DB       (1 << WM8904_DAC_BOOST_SHIFT)
#  define WM8904_DAC_BOOST_12DB      (2 << WM8904_DAC_BOOST_SHIFT)
#define WM8904_LOOPBACK              (1 << 8)  /* Bit 8:  Digital loopback function */
#define WM8904_AIFADCL_SRC           (1 << 7)  /* Bit 7:  Left digital audio channel source */
#define WM8904_AIFADCR_SRC           (1 << 6)  /* Bit 6:  Right digital audio channel source */
#define WM8904_AIFDACL_SRC           (1 << 5)  /* Bit 5:  Left DAC data source select */
#define WM8904_AIFDACR_SRC           (1 << 4)  /* Bit 4:  Right DAC data source select */
#define WM8904_ADC_COMP              (1 << 3)  /* Bit 3:  ADC companding enable */
#define WM8904_ADC_COMPMODE          (1 << 2)  /* Bit 2:  ADC companding type */
#define WM8904_DAC_COMP              (1 << 1)  /* Bit 1:  DAC companding enable */
#define WM8904_DAC_COMPMODE          (1 << 0)  /* Bit 0:  DAC companding type */

/* 0x19 Audio Interface 1 */

#define WM8904_AIFDAC_TDM            (1 << 13) /* Bit 13: DAC TDM enable */
#define WM8904_AIFDAC_TDM_CHAN       (1 << 12) /* Bit 12: DACDAT TDM channel select */
#define WM8904_AIFADC_TDM            (1 << 11) /* Bit 11: ADC TDM enable */
#define WM8904_AIFADC_TDM_CHAN       (1 << 10) /* Bit 10: ADCDAT TDM channel select */
#define WM8904_AIF_TRIS              (1 << 8)  /* Bit 8:  Audio interface tristate */
#define WM8904_AIF_BCLK_INV          (1 << 7)  /* Bit 7:  BCLK invert */
#define WM8904_BCLK_DIR              (1 << 6)  /* Bit 6:  Audio interface BCLK direction */
#define WM8904_AIF_LRCLK_INV         (1 << 4)  /* Bit 4:  LRC polarity/DSP mode A-B select */
#define WM8904_AIF_WL_SHIFT          (2)       /* Bits 2-3: Digital audio interface word length */
#define WM8904_AIF_WL_MASK           (3 << WM8904_AIF_WL_SHIFT)
#  define WM8904_AIF_WL_16BITS       (0 << WM8904_AIF_WL_SHIFT)
#  define WM8904_AIF_WL_20BITS       (1 << WM8904_AIF_WL_SHIFT)
#  define WM8904_AIF_WL_24BITS       (2 << WM8904_AIF_WL_SHIFT)
#  define WM8904_AIF_WL_32BITS       (3 << WM8904_AIF_WL_SHIFT)
#define WM8904_AIF_FMT_SHIFT         (0)       /* Bits 0-1: Digital audio interface format */
#define WM8904_AIF_FMT_MASK          (3 << WM8904_AIF_FMT_SHIFT)
#  define WM8904_AIF_FMT_RJUST       (0 << WM8904_AIF_FMT_SHIFT) /* Right Justified */
#  define WM8904_AIF_FMT_LJUST       (1 << WM8904_AIF_FMT_SHIFT) /* Left Justified */
#  define WM8904_AIF_FMT_I2S         (2 << WM8904_AIF_FMT_SHIFT) /* I2S */
#  define WM8904_AIF_FMT_DSP         (3 << WM8904_AIF_FMT_SHIFT) /* DSP */

/* 0x1a Audio Interface 2 */

#define WM8904_OPCLK_DIV_SHIFT       (8)       /* Bits 8-11: GPIO Output Clock Divider */
#define WM8904_OPCLK_DIV_MASK        (15 << WM8904_OPCLK_DIV_SHIFT)
#  define WM8904_OPCLK_DIV1          (0 << WM8904_OPCLK_DIV_SHIFT) /* SYSCLK */
#  define WM8904_OPCLK_DIV2          (1 << WM8904_OPCLK_DIV_SHIFT) /* SYSCLK / 2 */
#  define WM8904_OPCLK_DIV3          (2 << WM8904_OPCLK_DIV_SHIFT) /* SYSCLK / 3 */
#  define WM8904_OPCLK_DIV4          (3 << WM8904_OPCLK_DIV_SHIFT) /* SYSCLK / 4 */
#  define WM8904_OPCLK_DIV5p5        (4 << WM8904_OPCLK_DIV_SHIFT) /* SYSCLK / 5.5 */
#  define WM8904_OPCLK_DIV6          (5 << WM8904_OPCLK_DIV_SHIFT) /* SYSCLK / 6 */
#  define WM8904_OPCLK_DIV8          (6 << WM8904_OPCLK_DIV_SHIFT) /* SYSCLK / 8 */
#  define WM8904_OPCLK_DIV12         (7 << WM8904_OPCLK_DIV_SHIFT) /* SYSCLK / 12 */
#  define WM8904_OPCLK_DIV16         (8 << WM8904_OPCLK_DIV_SHIFT) /* SYSCLK / 16 */
#define WM8904_BCLK_DIV_SHIFT        (0)       /* Bits 0-4: BCLK Frequency (Master Mode) */
#define WM8904_BCLK_DIV_MASK         (31 << WM8904_BCLK_DIV_SHIFT)
#  define WM8904_BCLK_DIV(n)         ((uint16_t)(n) << WM8904_BCLK_DIV_SHIFT)
#  define WM8904_BCLK_DIV1           (0 << WM8904_BCLK_DIV_SHIFT) /* SYSCLK */
#  define WM8904_BCLK_DIV1p5         (1 << WM8904_BCLK_DIV_SHIFT) /* SYSCLK / 1.5 */
#  define WM8904_BCLK_DIV2           (2 << WM8904_BCLK_DIV_SHIFT) /* SYSCLK / 2 */
#  define WM8904_BCLK_DIV3           (3 << WM8904_BCLK_DIV_SHIFT) /* SYSCLK / 3 */
#  define WM8904_BCLK_DIV4           (4 << WM8904_BCLK_DIV_SHIFT) /* SYSCLK / 4 */
#  define WM8904_BCLK_DIV5           (5 << WM8904_BCLK_DIV_SHIFT) /* SYSCLK / 5 */
#  define WM8904_BCLK_DIV5p5         (6 << WM8904_BCLK_DIV_SHIFT) /* SYSCLK / 5.5 */
#  define WM8904_BCLK_DIV6           (7 << WM8904_BCLK_DIV_SHIFT) /* SYSCLK / 6 */
#  define WM8904_BCLK_DIV8           (8 << WM8904_BCLK_DIV_SHIFT) /* SYSCLK / 8 */
#  define WM8904_BCLK_DIV10          (9 << WM8904_BCLK_DIV_SHIFT) /* SYSCLK / 10 */
#  define WM8904_BCLK_DIV11          (10 << WM8904_BCLK_DIV_SHIFT) /* SYSCLK / 11 */
#  define WM8904_BCLK_DIV12          (11 << WM8904_BCLK_DIV_SHIFT) /* SYSCLK / 12 */
#  define WM8904_BCLK_DIV16          (12 << WM8904_BCLK_DIV_SHIFT) /* SYSCLK / 16 */
#  define WM8904_BCLK_DIV20          (13 << WM8904_BCLK_DIV_SHIFT) /* SYSCLK / 20 */
#  define WM8904_BCLK_DIV22          (14 << WM8904_BCLK_DIV_SHIFT) /* SYSCLK / 22 */
#  define WM8904_BCLK_DIV24          (15 << WM8904_BCLK_DIV_SHIFT) /* SYSCLK / 24 */
#  define WM8904_BCLK_DIV25          (16 << WM8904_BCLK_DIV_SHIFT) /* SYSCLK / 25 */
#  define WM8904_BCLK_DIV30          (17 << WM8904_BCLK_DIV_SHIFT) /* SYSCLK / 30 */
#  define WM8904_BCLK_DIV32          (18 << WM8904_BCLK_DIV_SHIFT) /* SYSCLK / 32 */
#  define WM8904_BCLK_DIV44          (19 << WM8904_BCLK_DIV_SHIFT) /* SYSCLK / 44 */
#  define WM8904_BCLK_DIV48          (20 << WM8904_BCLK_DIV_SHIFT) /* SYSCLK / 48 */

/* 0x1b Audio Interface 3 */

#define WM8904_LRCLK_DIR             (1 << 11) /* Bit 11: Audio interface LRC direction */
#define WM8904_LRCLK_RATE_SHIFT      (0)       /* Bits 0-10: LRC rate (master mode) */
#define WM8904_LRCLK_RATE_MASK       (0x7ff << WM8904_LRCLK_RATE_SHIFT)
#  define WM8904_LRCLK_RATE(n)       ((uint16_t)(n) << WM8904_LRCLK_RATE_SHIFT)

/* 0x1e DAC Digital Volume Left */
/* 0x1f DAC Digital Volume Right */

#define WM8904_DAC_VU                (1 << 8)  /* Bit 8:  DAC volume update */
#define WM8904_DAC_VOL_SHIFT         (0)       /* Bits 0-7: DAC digital volume */
#define WM8904_DAC_VOL_MASK          (0xff << WM8904_DAC_VOL_SHIFT)
#  define WM8904_DAC_VOL(n)          ((uint16_t)(n) << WM8904_DAC_VOL_SHIFT)

/* 0x20 DAC Digital 0 */

#define WM8904_ADCL_DAC_SVOL_SHIFT   (8)       /* Bits 8-11: Left digital sidetone volume */
#define WM8904_ADCL_DAC_SVOL_MASK    (15 << WM8904_ADCL_DAC_SVOL_SHIFT)
#  define WM8904_ADCL_DAC_SVOL(n)    ((uint16_t)(n) << WM8904_ADCL_DAC_SVOL_SHIFT)
#define WM8904_ADCR_DAC_SVOL_SHIFT   (4)       /* Bits 4-7:  Right digital sidetone volume*/
#define WM8904_ADCR_DAC_SVOL_MASK    (15 << WM8904_ADCR_DAC_SVOL_SHIFT)
#  define WM8904_ADCR_DAC_SVOL(n)    ((uint16_t)(n) << WM8904_ADCR_DAC_SVOL_SHIFT)
#define WM8904_ADC_TO_DACL_SHIFT     (2)       /* Bits 2-3: Left DAC digital sidetone source */
#define WM8904_ADC_TO_DACL_MASK      (3 << WM8904_ADC_TO_DACL_SHIFT)
#  define WM8904_ADC_TO_DACL_NONE    (0 << WM8904_ADC_TO_DACL_SHIFT) /* No sidetone */
#  define WM8904_ADC_TO_DACL_LEFT    (1 << WM8904_ADC_TO_DACL_SHIFT) /* Left ADC */
#  define WM8904_ADC_TO_DACL_RIGHT   (2 << WM8904_ADC_TO_DACL_SHIFT) /* Right ADC */
#define WM8904_ADC_TO_DACR_SHIFT     (0)       /* Bits 0-1: Right DAC digital sidetone source */
#define WM8904_ADC_TO_DACR_MASK      (3 << WM8904_ADC_TO_DACR_SHIFT)
#  define WM8904_ADC_TO_DACR_NONE    (0 << WM8904_ADC_TO_DACR_SHIFT) /* No sidetone */
#  define WM8904_ADC_TO_DACR_LEFT    (1 << WM8904_ADC_TO_DACR_SHIFT) /* Left ADC */
#  define WM8904_ADC_TO_DACR_RIGHT   (2 << WM8904_ADC_TO_DACR_SHIFT) /* Right ADC */

/* 0x21 DAC Digital 1 */

#define WM8904_DAC_MONO              (1 << 12) /* Bit 12: DAC mono mix */
#define WM8904_DAC_SB_FILT           (1 << 11) /* Bit 11: Selects DAC filter characteristics */
#define WM8904_DAC_MUTERATE          (1 << 10) /* Bit 10: DAC soft mute ramp rate */
#define WM8904_DAC_UNMUTE_RAMP       (1 << 9)  /* Bit 9:  DAC soft mute mode */
#define WM8904_DAC_OSR128            (1 << 6)  /* Bit 6:  DAC oversample rate select */
#define WM8904_DAC_MUTE              (1 << 3)  /* Bit 3:  DAC soft mute control */
#define WM8904_DEEMPH_SHIFT          (1)       /* Bits 1-2: DAC de-emphasis control */
#define WM8904_DEEMPH_MASK           (3 << WM8904_DEEMPH_SHIFT)
#  define WM8904_DEEMPH_NONE         (0 << WM8904_DEEMPH_SHIFT) /* No de-emphasis */
#  define WM8904_DEEMPH_32KHZ        (1 << WM8904_DEEMPH_SHIFT) /* 32kHz sample rate */
#  define WM8904_DEEMPH_44p1KHZ      (2 << WM8904_DEEMPH_SHIFT) /* 44.1kHz sample rate */
#  define WM8904_DEEMPH_48KHZ        (3 << WM8904_DEEMPH_SHIFT) /* 48kHz sample rate */

/* 0x24 ADC Digital Volume Left */
/* 0x25 ADC Digital Volume Right */

#define WM8904_ADC_VU                (1 << 8)  /* Bit 8:  ADC Volume Update */
#define WM8904_ADC_VOL_SHIFT         (0)       /* Bits 0-7: ADC Digital Volume */
#define WM8904_ADC_VOL_MASK          (0xff << WM8904_ADC_VOL_SHIFT)
#  define WM8904_ADC_VOL(n)          ((uint16_t)(n) << WM8904_ADC_VOL_SHIFT)

/* 0x26 ADC Digital */

#define WM8904_ADC_HPF_CUT_SHIFT     (5)      /* Bits 5-6: ADC digital high pass filter cut-off */
#define WM8904_ADC_HPF_CUT_MASK      (3 << WM8904_ADC_HPF_CUT_SHIFT)
#  define WM8904_ADC_HPF_CUT_HIFI    (0 << WM8904_ADC_HPF_CUT_SHIFT) /* Hi-fi mode */
#  define WM8904_ADC_HPF_CUT_VOICE1  (1 << WM8904_ADC_HPF_CUT_SHIFT) /* Voice mode 1 */
#  define WM8904_ADC_HPF_CUT_VOICE2  (2 << WM8904_ADC_HPF_CUT_SHIFT) /* Voice mode 2 */
#  define WM8904_ADC_HPF_CUT_VOICE3  (3 << WM8904_ADC_HPF_CUT_SHIFT) /* Voice mode 3 */
#define WM8904_ADC_HPF               (1 << 4)  /* Bit 4:  ADC digital high pass filter enable */
#define WM8904_ADCL_DATINV           (1 << 1)  /* Bit 1:  Left ADC invert */
#define WM8904_ADCR_DATINV           (1 << 0)  /* Bit 0:  Right ADC invert */

/* 0x27 Digital Microphone */

#define WM8904_DMIC_ENA              (1 << 12) /* Bit 12: Enables digital microphone mode */
#define WM8904_DMIC_SRC              (1 << 11) /* Bit 11: Selects digital microphone data input */

/* 0x28 DRC 0 */

#define WM8904_DRC_ENA               (1 << 15) /* Bit 15: DRC enable */
#define WM8904_DRC_DAC_PATH          (1 << 14) /* Bit 14: DRC path select */
#define WM8904_DRC_GS_HYST_LVL_SHIFT (11)      /* Bits 11-12: Gain smoothing hysteresis threshold */
#define WM8904_DRC_GS_HYST_LVL_MASK  (3 << WM8904_DRC_GS_HYST_LVL_SHIFT)
#  define WM8904_DRC_GS_HYST_LVL(n)  ((uint16_t)(n) << WM8904_DRC_GS_HYST_LVL_SHIFT)
#  define WM8904_DRC_GS_HYST_LOW     (0 << WM8904_DRC_GS_HYST_LVL_SHIFT) /* Low */
#  define WM8904_DRC_GS_HYST_MEDIUM  (1 << WM8904_DRC_GS_HYST_LVL_SHIFT) /* Medium */
#  define WM8904_DRC_GS_HYST_HIGH    (2 << WM8904_DRC_GS_HYST_LVL_SHIFT) /* High */
#define WM8904_DRC_STARTUP_GAIN_SHIFT  (6)      /* Bits 6-10: Initial gain at DRC startup */
#define WM8904_DRC_STARTUP_GAIN_MASK   (31 << WM8904_DRC_STARTUP_GAIN_SHIFT)
#  define WM8904_DRC_STARTUP_GAIN(n)   ((uint16_t)(n) << WM8904_DRC_STARTUP_GAIN_SHIFT)
#define WM8904_DRC_FF_DELAY          (1 << 5)  /* Bit 5:  Feed-forward delay for anti-clip feature */
#define WM8904_DRC_GS_ENA            (1 << 3)  /* Bit 3:  Gain smoothing enable */
#define WM8904_DRC_QR                (1 << 2)  /* Bit 2:  Quick release enable */
#define WM8904_DRC_ANTICLIP          (1 << 1)  /* Bit 1:  Anti-clip enable */
#define WM8904_DRC_GS_HYST           (1 << 0)  /* Bit 0:  Gain smoothing hysteresis enable */

/* 0x29 DRC 1 */

#define WM8904_DRC_ATK_SHIFT         (12)      /* Bits 13-15: Gain attack rate (seconds/6dB) */
#define WM8904_DRC_ATK_MASK          (15 << WM8904_DRC_ATK_SHIFT)
#  define WM8904_DRC_ATK(n)          ((uint16_t)(n) << WM8904_DRC_ATK_SHIFT)
#define WM8904_DRC_DCY_SHIFT         (8)       /* Bits 8-11: Gain decay rate (seconds/6dB) */
#define WM8904_DRC_DCY_MASK          (15 << WM8904_DRC_DCY_SHIFT)
#  define WM8904_DRC_DCY(n)          ((uint16_t)(n) << WM8904_DRC_DCY_SHIFT)
#define WM8904_DRC_QR_THR_SHIFT      (6)       /* Bits 6-7: Quick release crest factor threshold */
#define WM8904_DRC_QR_THR_MASK       (3 << WM8904_DRC_QR_THR_SHIFT)
#  define WM8904_DRC_QR_THR(n)       ((uint16_t)(n) << WM8904_DRC_QR_THR_SHIFT)
#define WM8904_DRC_QR_DCY_SHIFT      (4)       /* Bits 4-5: Quick release decay rate (seconds/6dB)*/
#define WM8904_DRC_QR_DCY_MASK       (3 << WM8904_DRC_QR_DCY_SHIFT)
#  define WM8904_DRC_QR_DCY(n)       ((uint16_t)(n) << WM8904_DRC_QR_DCY_SHIFT)
#define WM8904_DRC_MINGAIN_SHIFT     (2)       /* Bits 2-3:  Minimum gain the DRC can use to attenuate audio*/
#define WM8904_DRC_MINGAIN_MASK      (3 << WM8904_DRC_MINGAIN_SHIFT)
#  define WM8904_DRC_MINGAIN(n)      ((uint16_t)(n) << WM8904_DRC_MINGAIN_SHIFT)
#define WM8904_DRC_MAXGAIN_SHIFT     (0)       /* Bits 0-1:  Maximum gain the DRC can use to boost audio*/
#define WM8904_DRC_MAXGAIN_MASK      (3 << WM8904_DRC_MAXGAIN_SHIFT)
#  define WM8904_DRC_MAXGAIN(n)      ((uint16_t)(n) << WM8904_DRC_MAXGAIN_SHIFT)

/* 0x2a DRC 2 */

#define WM8904_DRC_HI_COMP_SHIFT     (3)      /* Bits 3-5: Compressor slope (upper region) */
#define WM8904_DRC_HI_COMP_MASK      (7 << WM8904_DRC_HI_COMP_SHIFT)
#  define WM8904_DRC_HI_COMP_DIV1    (0 << WM8904_DRC_HI_COMP_SHIFT) /* 1 (no compression) */
#  define WM8904_DRC_HI_COMP_DIV2    (1 << WM8904_DRC_HI_COMP_SHIFT) /* 1/2 */
#  define WM8904_DRC_HI_COMP_DIV4    (2 << WM8904_DRC_HI_COMP_SHIFT) /* 1/4 */
#  define WM8904_DRC_HI_COMP_DIV8    (3 << WM8904_DRC_HI_COMP_SHIFT) /* 1/8 */
#  define WM8904_DRC_HI_COMP_DIV16   (4 << WM8904_DRC_HI_COMP_SHIFT) /* 1/16 */
#  define WM8904_DRC_HI_COMP_ZERO    (5 << WM8904_DRC_HI_COMP_SHIFT) /* 0 */
#define WM8904_DRC_LO_COMP_SHIFT     (0)      /* Bits 0-2:  Compressor slope (lower region)*/
#define WM8904_DRC_LO_COMP_MASK      (7 << WM8904_DRC_LO_COMP_SHIFT)
#  define WM8904_DRC_LO_COMP(n)      ((uint16_t)(n) << WM8904_DRC_LO_COMP_SHIFT)
#  define WM8904_DRC_LO_COMP_DIV1    (0 << WM8904_DRC_HI_COMP_SHIFT) /* 1 (no compression) */
#  define WM8904_DRC_LO_COMP_DIV2    (1 << WM8904_DRC_HI_COMP_SHIFT) /* 1/2 */
#  define WM8904_DRC_LO_COMP_DIV4    (2 << WM8904_DRC_HI_COMP_SHIFT) /* 1/4 */
#  define WM8904_DRC_LO_COMP_DIV8    (3 << WM8904_DRC_HI_COMP_SHIFT) /* 1/8 */
#  define WM8904_DRC_LO_COMP_DIV16   (4 << WM8904_DRC_HI_COMP_SHIFT) /* 1/16 */
#  define WM8904_DRC_LO_COMP_ZERO    (5 << WM8904_DRC_HI_COMP_SHIFT) /* 0 */

/* 0x2b DRC 3 */

#define WM8904_DRC_KNEE_IP_SHIFT     (5)       /* Bits 5-10:  Input signal at the compressor knee*/
#define WM8904_DRC_KNEE_IP_MASK      (0X3f << WM8904_DRC_KNEE_IP_SHIFT)
#  define WM8904_DRC_KNEE_IP(n)      ((uint16_t)(n) << WM8904_DRC_KNEE_IP_SHIFT)
#define WM8904_DRC_KNEE_OP_SHIFT     (0)       /* Bits 0-4: Output signal at the compressor knee */
#define WM8904_DRC_KNEE_OP_MASK      (0x1f << WM8904_DRC_KNEE_OP_SHIFT)
#  define WM8904_DRC_KNEE_OP(n)      ((uint16_t)(n) << WM8904_DRC_KNEE_OP_SHIFT)

/* 0x2c Analogue Left Input 0 */
/* 0x2d Analogue Right Input 0 */

#define WM8904_INMUTE                (1 << 7)  /* Bit 7:  Input PGA mute */
#define WM8904_IN_VOL_SHIFT          (0)       /* Bits 0-4: Input PGA volume */
#define WM8904_IN_VOL_MASK           (31 << WM8904_IN_VOL_SHIFT)
#  define WM8904_IN_VOL(n)           ((uint16_t)(n) << WM8904_IN_VOL_SHIFT)

/* 0x2e Analogue Left Input 1 */
/* 0x2f Analogue Right Input 1 */

#define WM8904_IN_CM_ENA             (1 << 6)  /* Bit 6: Input PGA common mode rejection enable  */
#define WM8904_IP_SEL_N_SHIFT        (4)       /* Bits 4-5:  Select pin for inverting input */
#define WM8904_IP_SEL_N_MASK         (3 << WM8904_IP_SEL_N_SHIFT)
#  define WM8904_IP_SEL_N_IN1L       (0 << WM8904_IP_SEL_N_SHIFT)
#  define WM8904_IP_SEL_N_IN2L       (1 << WM8904_IP_SEL_N_SHIFT)
#  define WM8904_IP_SEL_N_IN3L       (2 << WM8904_IP_SEL_N_SHIFT)
#define WM8904_IP_SEL_P_SHIFT        (2)       /* Bits 2-3: Select pin for non-inverting input */
#define WM8904_IP_SEL_P_MASK         (3 << WM8904_IP_SEL_P_SHIFT)
#  define WM8904_IP_SEL_P(n)         ((uint16_t)(n) << WM8904_IP_SEL_P_SHIFT)
#  define WM8904_IP_SEL_P_IN1L       (0 << WM8904_IP_SEL_P_SHIFT)
#  define WM8904_IP_SEL_P_IN2L       (1 << WM8904_IP_SEL_P_SHIFT)
#  define WM8904_IP_SEL_P_IN3L       (2 << WM8904_IP_SEL_P_SHIFT)
#define WM8904_MODE_SHIFT            (0)       /* Bits 0-1:  Sets mode for analog input */
#define WM8904_MODE_MASK             (23<< WM8904_MODE_SHIFT)
#  define WM8904_MODE(n)             ((uint16_t)(n) << WM8904_MODE_SHIFT)
#  define WM8904_MODE_SINGLE         (0 << WM8904_MODE_SHIFT) /* Single-Ended */
#  define WM8904_MODE_DIFFLINE       (1 << WM8904_MODE_SHIFT) /* Differential Line */
#  define WM8904_MODE_DIFFMIC        (2 << WM8904_MODE_SHIFT) /* Differential MIC */

/* 0x39 Analogue OUT1 Left */
/* 0x3a Analogue OUT1 Right */

#define WM8904_HPOUT_MUTE            (1 << 8)  /* Bit 8:  Headphone output mute */
#define WM8904_HPOUT_VU              (1 << 7)  /* Bit 7:  Headphone output volume update */
#define WM8904_HPOUTZC               (1 << 6)  /* Bit 6:  Headphone output zero cross enable */
#define WM8904_HPOUT_VOL_SHIFT       (0)       /* Bits 0-5: Headphone output volume */
#define WM8904_HPOUT_VOL_MASK        (0x3f << WM8904_HPOUT_VOL_SHIFT)
#  define WM8904_HPOUT_VOL(n)        ((uint16_t)(n) << WM8904_HPOUT_VOL_SHIFT)

/* 0x3b Analogue OUT2 Left */
/* 0x3c Analogue OUT2 Right */

#define WM8904_LINEOUT_MUTE          (1 << 8)  /* Bit 8:  Headphone output mute */
#define WM8904_LINEOUT_VU            (1 << 7)  /* Bit 7:  Headphone output volume update */
#define WM8904_LINEOUTZC             (1 << 6)  /* Bit 6:  Headphone output zero cross enable */
#define WM8904_LINEOUT_VOL_SHIFT     (0)       /* Bits 0-5: Headphone output volume */
#define WM8904_LINEOUT_VOL_MASK      (0x3f << WM8904_LINEOUT_VOL_SHIFT)
#  define WM8904_LINEOUT_VOL(n)      ((uint16_t)(n) << WM8904_LINEOUT_VOL_SHIFT)

/* 0x3d Analogue OUT12 ZC */

#define WM8904_HPL_BYP_ENA           (1 << 3)  /* Bit 3:  Selects input for left headphone output MUX */
#define WM8904_HPR_BYP_ENA           (1 << 2)  /* Bit 2:  Selects input for right headphone output MUX */
#define WM8904_LINEOUTL_BYP_ENA      (1 << 1)  /* Bit 1:  Selects input for left line output MUX */
#define WM8904_LINEOUTR_BYP_ENA      (1 << 0)  /* Bit 0:  Selects input for right line output MUX */

/* 0x43 DC Servo 0 */

#define WM8904_DCS_ENA_CHAN_3        (1 << 3)  /* Bit 3:  DC servo enable for LINEOUTR */
#define WM8904_DCS_ENA_CHAN_2        (1 << 2)  /* Bit 2:  DC servo enable for LINEOUTL */
#define WM8904_DCS_ENA_CHAN_1        (1 << 1)  /* Bit 1:  DC servo enable for HPOUTR */
#define WM8904_DCS_ENA_CHAN_0        (1 << 0)  /* Bit 0:  DC servo enable for HPOUTL */

/* 0x44 DC Servo 1 */

#define WM8904_DCS_TRIG_SINGLE_3     (1 << 15) /* Bit 15: Single DC offset correction for LINEOUTR */
#define WM8904_DCS_TRIG_SINGLE_2     (1 << 14) /* Bit 14: Single DC offset correction for LINEOUTL */
#define WM8904_DCS_TRIG_SINGLE_1     (1 << 13) /* Bit 13: Single DC offset correction for HPOUTR */
#define WM8904_DCS_TRIG_SINGLE_0     (1 << 12) /* Bit 12: Single DC offset correction for HPOUTL */
#define WM8904_DCS_TRIG_SERIES_3     (1 << 11) /* Bit 11: Series of DC offset corrections for LINEOUTR */
#define WM8904_DCS_TRIG_SERIES_2     (1 << 10) /* Bit 10: Series of DC offset corrections for LINEOUTL */
#define WM8904_DCS_TRIG_SERIES_1     (1 << 9)  /* Bit 9:  Series of DC offset corrections for HPOUTR */
#define WM8904_DCS_TRIG_SERIES_0     (1 << 8)  /* Bit 8:  Series of DC offset corrections for HPOUTL */
#define WM8904_DCS_TRIG_STARTUP_3    (1 << 7)  /* Bit 7:  Start-up DC Servo mode for LINEOUTR */
#define WM8904_DCS_TRIG_STARTUP_2    (1 << 6)  /* Bit 6:  Start-up DC Servo mode for LINEOUTL */
#define WM8904_DCS_TRIG_STARTUP_1    (1 << 5)  /* Bit 5:  Start-up DC Servo mode for HPOUTR */
#define WM8904_DCS_TRIG_STARTUP_0    (1 << 4)  /* Bit 4:  Start-up DC Servo mode for HPOUTL */
#define WM8904_DCS_TRIG_DAC_WR_3     (1 << 3)  /* Bit 3:  DAC write DC Servo mode for LINEOUTR */
#define WM8904_DCS_TRIG_DAC_WR_2     (1 << 2)  /* Bit 2:  DAC write DC Servo mode for LINEOUTL */
#define WM8904_DCS_TRIG_DAC_WR_1     (1 << 1)  /* Bit 1:  DAC write DC Servo mode for HPOUTR */
#define WM8904_DCS_TRIG_DAC_WR_0     (1 << 0)  /* Bit 0:  DAC write DC Servo mode for HPOUTL */

/* 0x45 DC Servo 2 */

#define WM8904_DCS_TIMER_PERIOD_23_SHIFT (8)      /* Bits 8-11: LINEOUTL/LINEOUTR update period */
#define WM8904_DCS_TIMER_PERIOD_23_MASK  (15 << WM8904_DCS_TIMER_PERIOD_23_SHIFT)
#  define WM8904_DCS_TIMER_PERIOD_23(n)  ((uint16_t)(n) << WM8904_DCS_TIMER_PERIOD_23_SHIFT)
#define WM8904_DCS_TIMER_PERIOD_01_SHIFT (0)      /* Bits 0-3: HPOUTL/HPOUTR  update period */
#define WM8904_DCS_TIMER_PERIOD_01_MASK  (15 << WM8904_DCS_TIMER_PERIOD_01_SHIFT)
#  define WM8904_DCS_TIMER_PERIOD_01(n)  ((uint16_t)(n) << WM8904_DCS_TIMER_PERIOD_01_SHIFT)

/* 0x47 DC Servo 4 */

#define WM8904_DCS_SERIES_NO_23_MASK (0x007f)      /* Bits 0-6: Number updates to perform in series for LINEOUTL/R */

/* 0x48 DC Servo 5 */

#define WM8904_DCS_SERIES_NO_01_MASK (0x007f)      /* Bits 0-6: Number updates to perform in series for HPOUTL/R */

/* 0x49 DC Servo 6 */

#define WM8904_DCS_DAC_WR_VAL_3_MASK (0x00ff)      /* Bits 0-7: DC Offset value for LINEOUTR */

/* 0x4a DC Servo 7 */

#define WM8904_DCS_DAC_WR_VAL_2_MASK (0x00ff)      /* Bits 0-7: DC Offset value for LINEOUTL */

/* 0x4b DC Servo 8 */

#define WM8904_DCS_DAC_WR_VAL_1_MASK (0x00ff)      /* Bits 0-7: DC Offset value for HPOUTR */

/* 0x4c DC Servo 9 */

#define WM8904_DCS_DAC_WR_VAL_0_MASK (0x00ff)      /* Bits 0-7: DC Offset value for HPOUTL */

/* 0x4d DC Servo Readback 0 */

#define WM8904_DCS_CAL_COMPLETE_SHIFT          (8)      /* Bits 8-11: DC servo complete status */
#define WM8904_DCS_CAL_COMPLETE_MASK           (15 << WM8904_DCS_CAL_COMPLETE_SHIFT)
#  define WM8904_DCS_CAL_COMPLETE_LINEOUTR     (8 << WM8904_DCS_CAL_COMPLETE_SHIFT)
#  define WM8904_DCS_CAL_COMPLETE_LINEOUTL     (4 << WM8904_DCS_CAL_COMPLETE_SHIFT)
#  define WM8904_DCS_CAL_COMPLETE_HPOUTR       (2 << WM8904_DCS_CAL_COMPLETE_SHIFT)
#  define WM8904_DCS_CAL_COMPLETE_HPOUTL       (1 << WM8904_DCS_CAL_COMPLETE_SHIFT)
#define WM8904_DCS_DAC_WR_COMPLETE_SHIFT       (4)      /* Bits 4-7: DC servo DAC write status */
#define WM8904_DCS_DAC_WR_COMPLETE_MASK        (15 << WM8904_DCS_DAC_WR_COMPLETE_SHIFT)
#  define WM8904_DCS_DAC_WR_COMPLETE_LINEOUTR  (8 << WM8904_DCS_DAC_WR_COMPLETE_SHIFT)
#  define WM8904_DCS_DAC_WR_COMPLETE_LINEOUTL  (4 << WM8904_DCS_DAC_WR_COMPLETE_SHIFT)
#  define WM8904_DCS_DAC_WR_COMPLETE_HPOUTR    (2 << WM8904_DCS_DAC_WR_COMPLETE_SHIFT)
#  define WM8904_DCS_DAC_WR_COMPLETE_HPOUTL    (1 << WM8904_DCS_DAC_WR_COMPLETE_SHIFT)
#define WM8904_DCS_STARTUP_COMPLETE_SHIFT      (0)      /* Bits 0-3: DC servo start-up status */
#define WM8904_DCS_STARTUP_COMPLETE_MASK       (15 << WM8904_DCS_STARTUP_COMPLETE_SHIFT)
#  define WM8904_DCS_STARTUP_COMPLETE(n)       ((uint16_t)(n) << WM8904_DCS_STARTUP_COMPLETE_SHIFT)
#  define WM8904_DCS_STARTUP_COMPLETE_LINEOUTR (8 << WM8904_DCS_DAC_WR_COMPLETE_SHIFT)
#  define WM8904_DCS_STARTUP_COMPLETE_LINEOUTL (4 << WM8904_DCS_DAC_WR_COMPLETE_SHIFT)
#  define WM8904_DCS_STARTUP_COMPLETE_HPOUTR   (2 << WM8904_DCS_DAC_WR_COMPLETE_SHIFT)
#  define WM8904_DCS_STARTUP_COMPLETE_HPOUTL   (1 << WM8904_DCS_DAC_WR_COMPLETE_SHIFT)

/* 0x5a Analogue HP 0 */

#define WM8904_HPL_RMV_SHORT         (1 << 7)  /* Bit 7:  Removes HPOUTL short */
#define WM8904_HPL_ENA_OUTP          (1 << 6)  /* Bit 6:  Enables HPOUTL output stage */
#define WM8904_HPL_ENA_DLY           (1 << 5)  /* Bit 5:  Enables HPOUTL intermediate stage  */
#define WM8904_HPL_ENA               (1 << 4)  /* Bit 4:  Enables HPOUTL input stage */
#define WM8904_HPR_RMV_SHORT         (1 << 3)  /* Bit 3:  Removes HPOUTR short */
#define WM8904_HPR_ENA_OUTP          (1 << 2)  /* Bit 2:  Enables HPOUTR output stage */
#define WM8904_HPR_ENA_DLY           (1 << 1)  /* Bit 1:  Enables HPOUTR intermediate stage */
#define WM8904_HPR_ENA               (1 << 0)  /* Bit 0:  Enables HPOUTR input stage */

/* 0x5e Analogue Lineout 0 */

#define WM8904_LINEOUTL_RMV_SHORT    (1 << 7)  /* Bit 7:  Removes LINEOUTL short */
#define WM8904_LINEOUTL_ENA_OUTP     (1 << 6)  /* Bit 6:  Enables LINEOUTL output stage */
#define WM8904_LINEOUTL_ENA_DLY      (1 << 5)  /* Bit 5:  Enables LINEOUTL intermediate stage */
#define WM8904_LINEOUTL_ENA          (1 << 4)  /* Bit 4:  Enables LINEOUTL input stage */
#define WM8904_LINEOUTR_RMV_SHORT    (1 << 3)  /* Bit 3:  Removes LINEOUTR short */
#define WM8904_LINEOUTR_ENA_OUTP     (1 << 2)  /* Bit 2:  Enables LINEOUTR output stage */
#define WM8904_LINEOUTR_ENA_DLY      (1 << 1)  /* Bit 1:  Enables LINEOUTR intermediate stage */
#define WM8904_LINEOUTR_ENA          (1 << 0)  /* Bit 0:  Enables LINEOUTR input stage */

/* 0x62 Charge Pump 0 */

#define WM8904_CP_ENA                (1 << 0)  /* Bit 0:  Enable charge-pump digits */

/* 0x68 Class W 0 */

#define WM8904_CP_DYN_PWR            (1 << 0)  /* Bit 0:  Enable dynamic charge pump power control */

/* 0x6c Write Sequencer 0 */

#define WM8904_WSEQ_ENA               (1 << 8)  /* Bit 8:  Write sequencer enable */
#define WM8904_WSEQ_WRITE_INDEX_SHIFT (0)       /* Bits 0-4: Sequence write index */
#define WM8904_WSEQ_WRITE_INDEX_MASK  (31 << WM8904_WSEQ_WRITE_INDEX_SHIFT)
#  define WM8904_WSEQ_WRITE_INDEX(n)  ((uint16_t)(n) << WM8904_WSEQ_WRITE_INDEX_SHIFT)

/* 0x6d Write Sequencer 1 */

#define WM8904_WSEQ_DATA_WIDTH_SHIFT (12)      /* Bits 12-14: Width of data block in sequence step */
#define WM8904_WSEQ_DATA_WIDTH_MASK  (7 << WM8904_WSEQ_DATA_WIDTH_SHIFT)
#  define WM8904_WSEQ_DATA_WIDTH(n)  ((uint16_t)((n)-1) << WM8904_WSEQ_DATA_WIDTH_SHIFT)
#define WM8904_WSEQ_DATA_START_SHIFT (8)      /* Bits 8-11: LSB bit of data block in sequence step */
#define WM8904_WSEQ_DATA_START_MASK  (15 << WM8904_WSEQ_DATA_START_SHIFT)
#  define WM8904_WSEQ_DATA_START(n)  ((uint16_t)(n) << WM8904_WSEQ_DATA_START_SHIFT)
#define WM8904_WSEQ_ADDR_SHIFT       (0)      /* Bits 0-7: Control Register be written in sequence step */
#define WM8904_WSEQ_ADDR_MASK        (0xff << WM8904_WSEQ_ADDR_SHIFT)
#  define WM8904_WSEQ_ADDR(n)        ((uint16_t)(n) << WM8904_WSEQ_ADDR_SHIFT)

/* 0x6e Write Sequencer 2 */

#define WM8904_WSEQ_EOS              (1 << 14) /* Bit 14: End of sequence flag */
#define WM8904_WSEQ_DELAY_SHIFT      (8)       /* Bits 8-11: Time delay after executing this step */
#define WM8904_WSEQ_DELAY_MASK       (15 << WM8904_WSEQ_DELAY_SHIFT)
#  define WM8904_WSEQ_DELAY(n)       ((uint16_t)(n) << WM8904_WSEQ_DELAY_SHIFT)
#define WM8904_WSEQ_DATA_SHIFT       (0)       /* Bits 0-7: Data to be written in this sequence step */
#define WM8904_WSEQ_DATA_MASK        (0xff << WM8904_WSEQ_DATA_SHIFT)
#  define WM8904_WSEQ_DATA(n)        ((uint16_t)(n) << WM8904_WSEQ_DATA_SHIFT)

/* 0x6f Write Sequencer 3 */

#define WM8904_WSEQ_ABORT             (1 << 9) /* Bit 9:  Aborts the current sequence */
#define WM8904_WSEQ_START             (1 << 8) /* Bit 8:  Starts the write sequencer */
#define WM8904_WSEQ_START_INDEX_SHIFT (0)      /* Bits 0-5: Sequence start index */
#define WM8904_WSEQ_START_INDEX_MASK  (63 << WM8904_WSEQ_START_INDEX_SHIFT)
#  define WM8904_WSEQ_START_INDEX(n)  ((uint16_t)(n) << WM8904_WSEQ_START_INDEX_SHIFT)

/* 0x70 Write Sequencer 4 */

#define WM8904_WSEQ_CURRENT_INDEX_SHIFT (4)      /* Bits 4-9: Sequence current index */
#define WM8904_WSEQ_CURRENT_INDEX_MASK  (63 << WM8904_WSEQ_CURRENT_INDEX_SHIFT)
#define WM8904_WSEQ_BUSY                (1 << 0) /* Bit 0:  Sequencer busy flag */

/* 0x74 FLL Control 1 */

#define WM8904_FLL_FRACN_ENA         (1 << 2)  /* Bit 2:  FLL fractional enable */
#define WM8904_FLL_OSC_ENA           (1 << 1)  /* Bit 1:  FLL oscillator enable */
#define WM8904_FLL_ENA               (1 << 0)  /* Bit 0:  FLL enable */

/* 0x75 FLL Control 2 */

#define WM8904_FLL_OUTDIV_SHIFT      (8)      /* Bits 8-13: FLL FOUT clock divider */
#define WM8904_FLL_OUTDIV_MASK       (63 << WM8904_FLL_OUTDIV_SHIFT)
#  define WM8904_FLL_OUTDIV(n)       ((uint16_t)((n)-1) << WM8904_FLL_OUTDIV_SHIFT)
#define WM8904_FLL_CTRL_RATE_SHIFT   (4)      /* Bits 4-6: Frequency of the FLL control block */
#define WM8904_FLL_CTRL_RATE_MASK    (7 << WM8904_FLL_CTRL_RATE_SHIFT)
#  define WM8904_FLL_CTRL_RATE(n)    ((uint16_t)((n)-1) << WM8904_FLL_CTRL_RATE_SHIFT)
#define WM8904_FLL_FRATIO_SHIFT      (0)      /* Bits 0-2: FVCO clock divider */
#define WM8904_FLL_FRATIO_MASK       (7 << WM8904_FLL_FRATIO_SHIFT)
#  define WM8904_FLL_FRATIO(n)       ((uint32_t)(n) << WM8904_FLL_FRATIO_SHIFT)
#  define WM8904_FLL_FRATIO_DIV1     (0 << WM8904_FLL_FRATIO_SHIFT) /* Divide by 1 */
#  define WM8904_FLL_FRATIO_DIV2     (1 << WM8904_FLL_FRATIO_SHIFT) /* Divide by 2 */
#  define WM8904_FLL_FRATIO_DIV4     (2 << WM8904_FLL_FRATIO_SHIFT) /* Divide by 4 */
#  define WM8904_FLL_FRATIO_DIV8     (3 << WM8904_FLL_FRATIO_SHIFT) /* Divide by 8 */
#  define WM8904_FLL_FRATIO_DIV16    (4 << WM8904_FLL_FRATIO_SHIFT) /* Divide by 16 */

/* 0x76 FLL Control 3, Bits 0-15=Fractional multiply for FREF */
/* 0x77 FLL Control 4 */

#define WM8904_FLL_N_SHIFT           (5)      /* Bits 5-14: Integer multiply for FREF */
#define WM8904_FLL_N_MASK            (0x3ff << WM8904_FLL_N_SHIFT)
#  define WM8904_FLL_N(n)            ((uint16_t)(n) << WM8904_FLL_N_SHIFT)
#define WM8904_FLL_GAIN_SHIFT        (0)      /* Bits 0-3: FLL Gain applied to error */
#define WM8904_FLL_GAIN_MASK         (15 << WM8904_FLL_GAIN_SHIFT)
#  define WM8904_FLL_GAIN_X1         (0 << WM8904_FLL_GAIN_SHIFT)
#  define WM8904_FLL_GAIN_X2         (1 << WM8904_FLL_GAIN_SHIFT)
#  define WM8904_FLL_GAIN_X4         (2 << WM8904_FLL_GAIN_SHIFT)
#  define WM8904_FLL_GAIN_X8         (3 << WM8904_FLL_GAIN_SHIFT)
#  define WM8904_FLL_GAIN_X16        (4 << WM8904_FLL_GAIN_SHIFT)
#  define WM8904_FLL_GAIN_X32        (5 << WM8904_FLL_GAIN_SHIFT)
#  define WM8904_FLL_GAIN_X64        (6 << WM8904_FLL_GAIN_SHIFT)
#  define WM8904_FLL_GAIN_X128       (7 << WM8904_FLL_GAIN_SHIFT)
#  define WM8904_FLL_GAIN_X256       (8 << WM8904_FLL_GAIN_SHIFT)

/* 0x78 FLL Control 5 */

#define WM8904_FLL_CLK_REF_DIV_SHIFT (3)      /* Bits 3-4: FLL clock reference divider */
#define WM8904_FLL_CLK_REF_DIV_MASK  (3 << WM8904_FLL_CLK_REF_DIV_SHIFT)
#  define WM8904_FLL_CLK_REF_DIV1    (0 << WM8904_FLL_CLK_REF_DIV_SHIFT) /* MCLK / 1 */
#  define WM8904_FLL_CLK_REF_DIV2    (1 << WM8904_FLL_CLK_REF_DIV_SHIFT) /* MCLK / 2 */
#  define WM8904_FLL_CLK_REF_DIV4    (2 << WM8904_FLL_CLK_REF_DIV_SHIFT) /* MCLK / 4 */
#  define WM8904_FLL_CLK_REF_DIV8    (3 << WM8904_FLL_CLK_REF_DIV_SHIFT) /* MCLK / 8 */
#define WM8904_FLL_CLK_REF_SRC_SHIFT   (0)      /* Bits 0-2: FLL clock source */
#define WM8904_FLL_CLK_REF_SRC_MASK    (3 << WM8904_FLL_CLK_REF_SRC_SHIFT)
#  define WM8904_FLL_CLK_REF_SRC_MCLK  (0 << WM8904_FLL_CLK_REF_SRC_SHIFT)
#  define WM8904_FLL_CLK_REF_SRC_BCLK  (1 << WM8904_FLL_CLK_REF_SRC_SHIFT)
#  define WM8904_FLL_CLK_REF_SRC_LRCLK (2 << WM8904_FLL_CLK_REF_SRC_SHIFT)

/* 0x79 GPIO Control 1 */

#define WM8904_GPIO1_PU              (1 << 5)  /* Bit 5:  GPIO1 pull-up resistor enable */
#define WM8904_GPIO1_PD              (1 << 4)  /* Bit 4:  GPIO1 pull-down resistor enable */
#define WM8904_GPIO1_SEL_SHIFT       (0)       /* Bits 0-3:  */
#define WM8904_GPIO1_SEL_MASK        (15 << WM8904_GPIO1_SEL_SHIFT)
#  define WM8904_GPIO1_SEL_INPUT     (0 << WM8904_GPIO1_SEL_SHIFT) /* Input pin */
#  define WM8904_GPIO1_SEL_CLKOUT    (1 << WM8904_GPIO1_SEL_SHIFT) /* Clock output (f=SYSCLK/OPCLKDIV) */
#  define WM8904_GPIO1_SEL_ZERO      (2 << WM8904_GPIO1_SEL_SHIFT) /* Logic '0' */
#  define WM8904_GPIO1_SEL_ONE       (3 << WM8904_GPIO1_SEL_SHIFT) /* Logic '1' */
#  define WM8904_GPIO1_SEL_IRQ       (4 << WM8904_GPIO1_SEL_SHIFT) /* IRQ */
#  define WM8904_GPIO1_SEL_FLLLOCK   (5 << WM8904_GPIO1_SEL_SHIFT) /* FLL Lock */
#  define WM8904_GPIO1_SEL_MICDET    (6 << WM8904_GPIO1_SEL_SHIFT) /* Mic Detect */
#  define WM8904_GPIO1_SEL_MICSHORT  (7 << WM8904_GPIO1_SEL_SHIFT) /* Mic Short */
#  define WM8904_GPIO1_SEL_DMICOUT   (8 << WM8904_GPIO1_SEL_SHIFT) /* DMIC clock out */
#  define WM8904_GPIO1_SEL_FLLOUT    (9 << WM8904_GPIO1_SEL_SHIFT) /* FLL Clock Output */

/* 0x7a GPIO Control 2 */

#define WM8904_GPIO2_PU              (1 << 5)  /* Bit 5:  GPIO2 pull-up resistor enable */
#define WM8904_GPIO2_PD              (1 << 4)  /* Bit 4:  GPIO2 pull-down resistor enable */
#define WM8904_GPIO2_SEL_SHIFT       (0)       /* Bits 0-3:  */
#define WM8904_GPIO2_SEL_MASK        (15 << WM8904_GPIO2_SEL_SHIFT)
#  define WM8904_GPIO2_SEL_INPUT     (0 << WM8904_GPIO2_SEL_SHIFT) /* Input pin */
#  define WM8904_GPIO2_SEL_CLKOUT    (1 << WM8904_GPIO2_SEL_SHIFT) /* Clock output (f=SYSCLK/OPCLKDIV) */
#  define WM8904_GPIO2_SEL_ZERO      (2 << WM8904_GPIO2_SEL_SHIFT) /* Logic '0' */
#  define WM8904_GPIO2_SEL_ONE       (3 << WM8904_GPIO2_SEL_SHIFT) /* Logic '1' */
#  define WM8904_GPIO2_SEL_IRQ       (4 << WM8904_GPIO2_SEL_SHIFT) /* IRQ */
#  define WM8904_GPIO2_SEL_FLLLOCK   (5 << WM8904_GPIO2_SEL_SHIFT) /* FLL Lock */
#  define WM8904_GPIO2_SEL_MICDET    (6 << WM8904_GPIO2_SEL_SHIFT) /* Mic Detect */
#  define WM8904_GPIO2_SEL_MICSHORT  (7 << WM8904_GPIO2_SEL_SHIFT) /* Mic Short */
#  define WM8904_GPIO2_SEL_DMICOUT   (8 << WM8904_GPIO2_SEL_SHIFT) /* DMIC clock out */
#  define WM8904_GPIO2_SEL_FLLOUT    (9 << WM8904_GPIO2_SEL_SHIFT) /* FLL Clock Output */

/* 0x7b GPIO Control 3 */

#define WM8904_GPIO3_PU              (1 << 5)  /* Bit 5:  GPIO3 pull-up resistor enable */
#define WM8904_GPIO3_PD              (1 << 4)  /* Bit 4:  GPIO3 pull-down resistor enable */
#define WM8904_GPIO3_SEL_SHIFT       (0)       /* Bits 0-3:  */
#define WM8904_GPIO3_SEL_MASK        (15 << WM8904_GPIO3_SEL_SHIFT)
#  define WM8904_GPIO3_SEL_INPUT     (0 << WM8904_GPIO3_SEL_SHIFT) /* Input pin */
#  define WM8904_GPIO3_SEL_CLKOUT    (1 << WM8904_GPIO3_SEL_SHIFT) /* Clock output (f=SYSCLK/OPCLKDIV) */
#  define WM8904_GPIO3_SEL_ZERO      (2 << WM8904_GPIO3_SEL_SHIFT) /* Logic '0' */
#  define WM8904_GPIO3_SEL_ONE       (3 << WM8904_GPIO3_SEL_SHIFT) /* Logic '1' */
#  define WM8904_GPIO3_SEL_IRQ       (4 << WM8904_GPIO3_SEL_SHIFT) /* IRQ */
#  define WM8904_GPIO3_SEL_FLLLOCK   (5 << WM8904_GPIO3_SEL_SHIFT) /* FLL Lock */
#  define WM8904_GPIO3_SEL_MICDET    (6 << WM8904_GPIO3_SEL_SHIFT) /* Mic Detect */
#  define WM8904_GPIO3_SEL_MICSHORT  (7 << WM8904_GPIO3_SEL_SHIFT) /* Mic Short */
#  define WM8904_GPIO3_SEL_DMICOUT   (8 << WM8904_GPIO3_SEL_SHIFT) /* DMIC clock out */
#  define WM8904_GPIO3_SEL_FLLOUT    (9 << WM8904_GPIO3_SEL_SHIFT) /* FLL Clock Output */

/* 0x7c GPIO Control 4 */

#define WM8904_GPI7_ENA              (1 << 9)  /* Bit 9:  GPI7 input enable */
#define WM8904_GPI8_ENA              (1 << 8)  /* Bit 8:  GPI8 input enable */
#define WM8904_GPIO_BCLK_MODE_ENA    (1 << 7)  /* Bit 7:  Selects BCLK/GPIO4 pin function */
#define WM8904_GPIO_BCLK_SEL_SHIFT   (0)       /* Bits 0-3: GPIO_BCLK function select */
#define WM8904_GPIO_BCLK_SEL_MASK    (15 << WM8904_GPIO_BCLK_SEL_SHIFT)
#  define WM8904_GPIO_BCLK_SEL_INPUT    (0 << WM8904_GPIO_BCLK_SEL_SHIFT) /* Input pin */
#  define WM8904_GPIO_BCLK_SEL_CLKOUT   (1 << WM8904_GPIO_BCLK_SEL_SHIFT) /* Clock output (f=SYSCLK/OPCLKDIV) */
#  define WM8904_GPIO_BCLK_SEL_ZERO     (2 << WM8904_GPIO_BCLK_SEL_SHIFT) /* Logic '0' */
#  define WM8904_GPIO_BCLK_SEL_ONE      (3 << WM8904_GPIO_BCLK_SEL_SHIFT) /* Logic '1' */
#  define WM8904_GPIO_BCLK_SEL_IRQ      (4 << WM8904_GPIO_BCLK_SEL_SHIFT) /* IRQ */
#  define WM8904_GPIO_BCLK_SEL_FLLLOCK  (5 << WM8904_GPIO_BCLK_SEL_SHIFT) /* FLL Lock */
#  define WM8904_GPIO_BCLK_SEL_MICDET   (6 << WM8904_GPIO_BCLK_SEL_SHIFT) /* Mic Detect */
#  define WM8904_GPIO_BCLK_SEL_MICSHORT (7 << WM8904_GPIO_BCLK_SEL_SHIFT) /* Mic Short */
#  define WM8904_GPIO_BCLK_SEL_DMICOUT  (8 << WM8904_GPIO_BCLK_SEL_SHIFT) /* DMIC clock out */
#  define WM8904_GPIO_BCLK_SEL_FLLOUT   (9 << WM8904_GPIO_BCLK_SEL_SHIFT) /* FLL Clock Output */

/* 0x7e Digital Pulls */

#define WM8904_MCLK_PU               (1 << 7)  /* Bit 7:  MCLK pull-up resistor enable */
#define WM8904_MCLK_PD               (1 << 6)  /* Bit 6:  MCLK pull-down resistor enable */
#define WM8904_DACDAT_PU             (1 << 5)  /* Bit 5:  DACDAT pull-up resistor enable */
#define WM8904_DACDAT_PD             (1 << 4)  /* Bit 4:  DACDAT pull-down resistor enable */
#define WM8904_LRCLK_PU              (1 << 3)  /* Bit 3:  LRCLK pull-up resistor enable */
#define WM8904_LRCLK_PD              (1 << 2)  /* Bit 2:  LRCLK pull-down resistor enable */
#define WM8904_BCLK_PU               (1 << 1)  /* Bit 1:  BCLK pull-up resistor enable */
#define WM8904_BCLK_PD               (1 << 0)  /* Bit 0:  BCLK pull-down resistor enable */

/* Common interrupt bits */
/* 0x7f Interrupt Status */
/* 0x80 Interrupt Status Mask */
/* 0x81 Interrupt Polarity */
/* 0x82 Interrupt Debounce */

#define WM8904_GPIO_BCLK_INT         (1 << 9)  /* Bit 9:  GPIO4 interrupt */
#define WM8904_WSEQ_INT              (1 << 8)  /* Bit 8:  Write Sequence interrupt */
#define WM8904_GPIO3_INT             (1 << 7)  /* Bit 7:  GPIO3 interrupt */
#define WM8904_GPIO2_INT             (1 << 6)  /* Bit 6:  GPIO2 interrupt */
#define WM8904_GPIO1_INT             (1 << 5)  /* Bit 5:  GPIO1 interrupt */
#define WM8904_GPI8_INT              (1 << 4)  /* Bit 4:  GPI8 interrupt */
#define WM8904_GPI7_INT              (1 << 3)  /* Bit 3:  GPI7 interrupt */
#define WM8904_FLL_LOCK_INT          (1 << 2)  /* Bit 2:  FLL Lock interrupt */
#define WM8904_MIC_SHRT_INT          (1 << 1)  /* Bit 1:  MICBIAS short circuit interrupt */
#define WM8904_MIC_DET_INT           (1 << 0)  /* Bit 0:  MICBIAS short circuit interrupt */

#define WM8904_ALL_INTS              (0x03ff)

/* 0x7f Interrupt Status (only) */

#define WM8904_IRQ                   (1 << 10) /* Bit 10: Logical OR of all other interrupt flags */

/* 0x86 EQ1 */

#define WM8904_EQ_ENA                (1 << 0)  /* Bit 0:  EQ enable */

/* 0x87-0x8b EQ2-EQ6:   5 bit equalizer value */
/* 0x8c-0x9d EQ7-EQ24: 16-bit equalizer value */

/* 0xa1 Control Interface Test 1 */

#define WM8904_USER_KEY              0x0002

/* 0xc6 ADC Test  */

#define WM8904_ADC_128_OSR_TST_MODE  (1 << 2)  /* Bit 2:  ADC bias control (1) */
#define WM8904_ADC_BIASX1P5          (1 << 0)  /* Bit 0:  ADC bias control (2) */

/* 0xcc Analogue Output Bias 0 */

#define WM8904_ANA_OUT_BIAS_SHIFT    (4)       /* Bits 4-6: */
#define WM8904_ANA_OUT_BIAS_MASK     (7 << WM8904_ANA_OUT_BIAS_SHIFT)
#  define WM8904_ANA_OUT_BIAS(n)     ((uint16_t)(n) << WM8904_ANA_OUT_BIAS_SHIFT)

/* 0xf7 FLL NCO Test 0 */

#define WM8904_FLL_FRC_NCO           (1 << 0)  /* Bit 0: FLL Forced control select  */

/* 0xf8 FLL NCO Test 1 */

#define WM8904FLL_FRC_NCO_SHIFT      (0)       /* Bits 0-5: FLL forced oscillator value */
#define WM8904FLL_FRC_NCO_MASK       (0x3f << WM8904FLL_FRC_NCO_SHIFT)
#  define WM8904FLL_FRC_NCO_VAL(n)   ((uint16_t)(n) << WM8904FLL_FRC_NCO_SHIFT)

/* FLL Configuration *********************************************************/
/* Default FLL configuration */

#define WM8904_DEFAULT_SAMPRATE      11025     /* Initial sample rate */
#define WM8904_DEFAULT_NCHANNELS     1         /* Initial number of channels */
#define WM8904_DEFAULT_BPSAMP        16        /* Initial bits per sample */

#define WM8904_NFLLRATIO_DIV1        0         /* Values of the FLL_RATIO field */
#define WM8904_NFLLRATIO_DIV2        1
#define WM8904_NFLLRATIO_DIV4        2
#define WM8904_NFLLRATIO_DIV8        3
#define WM8904_NFLLRATIO_DIV16       4
#define WM8904_NFLLRATIO             5          /* Number of FLL_RATIO values */

#define WM8904_MINOUTDIV             4          /* Minimum FLL_OUTDIV divider */
#define WM8904_MAXOUTDIV             64         /* Maximum FLL_OUTDIV divider */

#define WM8904_BCLK_MAXDIV           20         /* Maximum BCLK divider */

#define WM8904_FVCO_MIN              90000000   /* Minimum value of Fvco */
#define WM8904_FVCO_MAX              100000000  /* Maximum value of Fvco */

#define WM8904_FRAMELEN8              14        /* Bits per frame for 8-bit data */
#define WM8904_FRAMELEN16             32        /* Bits per frame for 16-bit data */

/* Commonly defined and redefined macros */

#ifndef MIN
#  define MIN(a,b)                   (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#  define MAX(a,b)                   (((a) > (b)) ? (a) : (b))
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct wm8904_dev_s
{
  /* We are an audio lower half driver (We are also the upper "half" of
   * the WM8904 driver with respect to the board lower half driver).
   *
   * Terminology: Our "lower" half audio instances will be called dev for the
   * publicly visible version and "priv" for the version that only this driver
   * knows.  From the point of view of this driver, it is the board lower
   * "half" that is referred to as "lower".
   */

  struct audio_lowerhalf_s dev;             /* WM8904 audio lower half (this device) */

  /* Our specific driver data goes here */

  const FAR struct wm8904_lower_s *lower;   /* Pointer to the board lower functions */
  FAR struct i2c_master_s *i2c;             /* I2C driver to use */
  FAR struct i2s_dev_s   *i2s;              /* I2S driver to use */
  struct dq_queue_s       pendq;            /* Queue of pending buffers to be sent */
  struct dq_queue_s       doneq;            /* Queue of sent buffers to be returned */
  mqd_t                   mq;               /* Message queue for receiving messages */
  char                    mqname[16];       /* Our message queue name */
  pthread_t               threadid;         /* ID of our thread */
  uint32_t                bitrate;          /* Actual programmed bit rate */
  sem_t                   pendsem;          /* Protect pendq */
#ifdef WM8904_USE_FFLOCK_INT
  struct work_s           work;             /* Interrupt work */
#endif
  uint16_t                samprate;         /* Configured samprate (samples/sec) */
#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
#ifndef CONFIG_AUDIO_EXCLUDE_BALANCE
  uint16_t                balance;          /* Current balance level (b16) */
#endif  /* CONFIG_AUDIO_EXCLUDE_BALANCE */
  uint8_t                 volume;           /* Current volume level {0..63} */
#endif  /* CONFIG_AUDIO_EXCLUDE_VOLUME */
  uint8_t                 nchannels;        /* Number of channels (1 or 2) */
  uint8_t                 bpsamp;           /* Bits per sample (8 or 16) */
  volatile uint8_t        inflight;         /* Number of audio buffers in-flight */
#ifdef WM8904_USE_FFLOCK_INT
  volatile bool           locked;           /* FLL is locked */
#endif
  bool                    running;          /* True: Worker thread is running */
  bool                    paused;           /* True: Playing is paused */
  bool                    mute;             /* True: Output is muted */
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  bool                    terminating;      /* True: Stop requested */
#endif
  bool                    reserved;         /* True: Device is reserved */
  volatile int            result;           /* The result of the last transfer */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_WM8904_CLKDEBUG
extern const uint8_t g_sysclk_scaleb1[WM8904_BCLK_MAXDIV+1];
extern const uint8_t g_fllratio[WM8904_NFLLRATIO];
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: wm8904_readreg
 *
 * Description:
 *    Read the specified 16-bit register from the WM8904 device.
 *
 ****************************************************************************/

#if defined(CONFIG_WM8904_REGDUMP) || defined(CONFIG_WM8904_CLKDEBUG)
struct wm8904_dev_s;
uint16_t wm8904_readreg(FAR struct wm8904_dev_s *priv, uint8_t regaddr);
#endif

#endif /* CONFIG_AUDIO */
#endif /* __DRIVERS_AUDIO_WM8904_H */
