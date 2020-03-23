/****************************************************************************************************
 * arch/arm/src/s32k1xx/hardware/s32k1xx_ftm.h
 *
 *   Copyright (C) 2020 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_FTM_H
#define __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_FTM_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include <hardware/s32k1xx_memorymap.h>

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

#define S32K1XX_FTM_SC_OFFSET       0x0000 /* Status and Control offset*/
#define S32K1XX_FTM_CNT_OFFSET      0x0004 /* Counter offset */
#define S32K1XX_FTM_MOD_OFFSET      0x0008 /* Modulo offset */
#define S32K1XX_FTM_C0SC_OFFSET     0x000C /* Channel 0 Status and Control offset */
#define S32K1XX_FTM_C0V_OFFSET      0x0010 /* Channel 0 Value offset */
#define S32K1XX_FTM_C1SC_OFFSET     0x0014 /* Channel 1 Status and Control offset */
#define S32K1XX_FTM_C1V_OFFSET      0x0018 /* Channel 1 Value offset */
#define S32K1XX_FTM_STATUS_OFFSET   0x0050 /* Capture and Compare Status offset */
#define S32K1XX_FTM_COMBINE_OFFSET  0x0064 /* Combine Channel Register offset */
#define S32K1XX_FTM_POL_OFFSET      0x0070 /* Channel Polarity offset */
#define S32K1XX_FTM_FILTER_OFFSET   0x0078 /* Filter Control offset */
#define S32K1XX_FTM_QDCTRL_OFFSET   0x0080 /* Quadrature Decoder Control and Status offset */
#define S32K1XX_FTM_CONF_OFFSET     0x0084 /* Configuration offset */

#define S32K1XX_FTM1_SC             (S32K1XX_FTM1_BASE + S32K1XX_FTM_SC_OFFSET)      /* FTM1 Status and Control */
#define S32K1XX_FTM1_CNT            (S32K1XX_FTM1_BASE + S32K1XX_FTM_CNT_OFFSET)     /* FTM1 Counter */
#define S32K1XX_FTM1_MOD            (S32K1XX_FTM1_BASE + S32K1XX_FTM_MOD_OFFSET)     /* FTM1 Modulo */
#define S32K1XX_FTM1_C0SC           (S32K1XX_FTM1_BASE + S32K1XX_FTM_C0SC_OFFSET)    /* FTM1 Channel 0 Status and Control */
#define S32K1XX_FTM1_C0V            (S32K1XX_FTM1_BASE + S32K1XX_FTM_C0V_OFFSET)     /* FTM1 Channel 0 Value */
#define S32K1XX_FTM1_C1SC           (S32K1XX_FTM1_BASE + S32K1XX_FTM_C1SC_OFFSET)    /* FTM1 Channel 1 Status and Control */
#define S32K1XX_FTM1_C1V            (S32K1XX_FTM1_BASE + S32K1XX_FTM_C1V_OFFSET)     /* FTM1 Channel 1 Value */
#define S32K1XX_FTM1_C2SC           (S32K1XX_FTM1_BASE + S32K1XX_FTM_C2SC_OFFSET)    /* FTM1 Channel 2 Status and Control */
#define S32K1XX_FTM1_C2V            (S32K1XX_FTM1_BASE + S32K1XX_FTM_C2V_OFFSET)     /* FTM1 Channel 2 Value */
#define S32K1XX_FTM1_STATUS         (S32K1XX_FTM1_BASE + S32K1XX_FTM_STATUS_OFFSET)  /* FTM1 Capture and Compare Status */
#define S32K1XX_FTM1_COMBINE        (S32K1XX_FTM1_BASE + S32K1XX_FTM_COMBINE_OFFSET) /* FTM1 Combine Channel Register offset */
#define S32K1XX_FTM1_POL            (S32K1XX_FTM1_BASE + S32K1XX_FTM_POL_OFFSET)     /* FTM1 Channel Polarity offset */
#define S32K1XX_FTM1_FILTER         (S32K1XX_FTM1_BASE + S32K1XX_FTM_FILTER_OFFSET)  /* FTM1 Filter Control offset */
#define S32K1XX_FTM1_QDCTRL         (S32K1XX_FTM1_BASE + S32K1XX_FTM_QDCTRL_OFFSET)  /* FTM1 Quadrature Decoder Control and Status offset */
#define S32K1XX_FTM1_CONF           (S32K1XX_FTM1_BASE + S32K1XX_FTM_CONF_OFFSET)    /* FTM1 Configuration */

#define S32K1XX_FTM2_SC             (S32K1XX_FTM2_BASE + S32K1XX_FTM_SC_OFFSET)      /* FTM2 Status and Control */
#define S32K1XX_FTM2_CNT            (S32K1XX_FTM2_BASE + S32K1XX_FTM_CNT_OFFSET)     /* FTM2 Counter */
#define S32K1XX_FTM2_MOD            (S32K1XX_FTM2_BASE + S32K1XX_FTM_MOD_OFFSET)     /* FTM2 Modulo */
#define S32K1XX_FTM2_C0SC           (S32K1XX_FTM2_BASE + S32K1XX_FTM_C0SC_OFFSET)    /* FTM2 Channel 0 Status and Control */
#define S32K1XX_FTM2_C0V            (S32K1XX_FTM2_BASE + S32K1XX_FTM_C0V_OFFSET)     /* FTM2 Channel 0 Value */
#define S32K1XX_FTM2_C1SC           (S32K1XX_FTM2_BASE + S32K1XX_FTM_C1SC_OFFSET)    /* FTM2 Channel 1 Status and Control */
#define S32K1XX_FTM2_C1V            (S32K1XX_FTM2_BASE + S32K1XX_FTM_C1V_OFFSET)     /* FTM2 Channel 1 Value */
#define S32K1XX_FTM2_C2SC           (S32K1XX_FTM2_BASE + S32K1XX_FTM_C2SC_OFFSET)    /* FTM2 Channel 2 Status and Control */
#define S32K1XX_FTM2_C2V            (S32K1XX_FTM2_BASE + S32K1XX_FTM_C2V_OFFSET)     /* FTM2 Channel 2 Value */
#define S32K1XX_FTM2_STATUS         (S32K1XX_FTM2_BASE + S32K1XX_FTM_STATUS_OFFSET)  /* FTM2 Capture and Compare Status */
#define S32K1XX_FTM2_COMBINE        (S32K1XX_FTM2_BASE + S32K1XX_FTM_COMBINE_OFFSET) /* FTM2 Combine Channel Register offset */
#define S32K1XX_FTM2_POL            (S32K1XX_FTM2_BASE + S32K1XX_FTM_POL_OFFSET)     /* FTM2 Channel Polarity offset */
#define S32K1XX_FTM2_FILTER         (S32K1XX_FTM2_BASE + S32K1XX_FTM_FILTER_OFFSET)  /* FTM2 Filter Control offset */
#define S32K1XX_FTM2_QDCTRL         (S32K1XX_FTM2_BASE + S32K1XX_FTM_QDCTRL_OFFSET)  /* FTM2 Quadrature Decoder Control and Status offset */
#define S32K1XX_FTM2_CONF           (S32K1XX_FTM2_BASE + S32K1XX_FTM_CONF_OFFSET)    /* FTM2 Configuration */

#define FTM_SC_PS_SHIFT             0 /* Bit 0-2: Prescaler */
#define FTM_SC_PS_MASK              (7 << FTM_SC_PS_SHIFT)
# define FTP_SC_PS_DIV1             (0 << FTM_SC_PS_SHIFT)
# define FTP_SC_PS_DIV2             (1 << FTM_SC_PS_SHIFT)
# define FTP_SC_PS_DIV4             (2 << FTM_SC_PS_SHIFT)
# define FTP_SC_PS_DIV8             (3 << FTM_SC_PS_SHIFT)
# define FTP_SC_PS_DIV16            (4 << FTM_SC_PS_SHIFT)
# define FTP_SC_PS_DIV32            (5 << FTM_SC_PS_SHIFT)
# define FTP_SC_PS_DIV64            (6 << FTM_SC_PS_SHIFT)
# define FTP_SC_PS_DIV128           (7 << FTM_SC_PS_SHIFT)
#define FTM_SC_CLKS_SHIFT           3 /* Bits 3-4: Clock Srouce Selection */
#define FTM_SC_CLKS_MASK            (3 << FTM_SC_CLKS_SHIFT)
# define FTM_SC_CLKS_DIS            (0 << FTM_SC_CLKS_SHIFT)
# define FTM_SC_CLKS_FTM            (1 << FTM_SC_CLKS_SHIFT)
# define FTM_SC_CLKS_FIXED          (2 << FTM_SC_CLKS_SHIFT)
# define FTM_SC_CLKS_EXTCLK         (3 << FTM_SC_CLKS_SHIFT)
#define FTM_SC_CPWMS                (1 << 5) /* Bit 5: Center-aligned PWM Select */
#define FTM_SC_RIE                  (1 << 6) /* Bit 6: Reload Point Interrupt Enable */
#define FTM_SC_RF                   (1 << 7) /* Bit 7: Reload flag */
#define FTM_SC_TOIE                 (1 << 8) /* Bit 8: Timer Overflow Interrupt Enable */
#define FTM_SC_TOF                  (1 << 9) /* Bit 9: Timer Overflow Flag*/
#define FTM_SC_FLTPS_SHIFT          24       /* Bits 24-27: Filter Prescaler */
#define FTM_SC_FLTPS_MASK           (7 << FTM_SC_FLTPS_SHIFT)
# define FTM_SC_FLTPS_DIV1          (0 << FTM_SC_FLTPS_SHIFT)  /* Divide Clock by 1 */
# define FTM_SC_FLTPS_DIV2          (1 << FTM_SC_FLTPS_SHIFT)  /* Divide Clock by 2 */
# define FTM_SC_FLTPS_DIV3          (2 << FTM_SC_FLTPS_SHIFT)  /* Divide Clock by 3 */
# define FTM_SC_FLTPS_DIV4          (3 << FTM_SC_FLTPS_SHIFT)  /* Divide Clock by 4 */
# define FTM_SC_FLTPS_DIV5          (4 << FTM_SC_FLTPS_SHIFT)  /* Divide Clock by 5 */
# define FTM_SC_FLTPS_DIV6          (5 << FTM_SC_FLTPS_SHIFT)  /* Divide Clock by 6 */
# define FTM_SC_FLTPS_DIV7          (6 << FTM_SC_FLTPS_SHIFT)  /* Divide Clock by 7 */
# define FTM_SC_FLTPS_DIV8          (7 << FTM_SC_FLTPS_SHIFT)  /* Divide Clock by 8 */
# define FTM_SC_FLTPS_DIV9          (8 << FTM_SC_FLTPS_SHIFT)  /* Divide Clock by 9 */
# define FTM_SC_FLTPS_DIV10         (9 << FTM_SC_FLTPS_SHIFT)  /* Divide Clock by 10 */
# define FTM_SC_FLTPS_DIV11         (10 << FTM_SC_FLTPS_SHIFT) /* Divide Clock by 11 */
# define FTM_SC_FLTPS_DIV12         (11 << FTM_SC_FLTPS_SHIFT) /* Divide Clock by 12 */
# define FTM_SC_FLTPS_DIV13         (12 << FTM_SC_FLTPS_SHIFT) /* Divide Clock by 13 */
# define FTM_SC_FLTPS_DIV14         (13 << FTM_SC_FLTPS_SHIFT) /* Divide Clock by 14 */
# define FTM_SC_FLTPS_DIV15         (14 << FTM_SC_FLTPS_SHIFT) /* Divide Clock by 15 */
# define FTM_SC_FLTPS_DIV16         (15 << FTM_SC_FLTPS_SHIFT) /* Divide Clock by 16 */

#define FTM_CNSC_DMA                (1 << 0)  /* Bit 0: ChnN Enable DMA */
#define FTM_CNSC_ICRST              (1 << 1)  /* Bit 1: ChnN Input capture reset */
#define FTM_CNSC_ELSA               (1 << 2)  /* Bit 2: ChnN Edge or Level */
#define FTM_CNSC_ELSB               (1 << 3)  /* Bit 3: ChnN Edge or Level */
#define FTM_CNSC_MSA                (1 << 4)  /* Bit 4: ChnN Mode select */
#define FTM_CNSC_MSB                (1 << 5)  /* Bit 5: ChnN Mode select */
#define FTM_CNSC_CHIE               (1 << 6)  /* Bit 6: ChnN interrupt enable */
#define FTM_CNSC_CHF                (1 << 7)  /* Bit 7: ChnN flag */
#define FTM_CNSC_TRIGMODE           (1 << 8)  /* Bit 8: ChnN tigger mode */
#define FTM_CNSC_CHIS               (1 << 9)  /* Bit 9: ChnN input state */
#define FTM_CNSC_CHOV               (1 << 10) /* Bit 10: ChnN output value */

#define FTM_STATUS_CH0F             (1 << 0) /* Bit 0: Chn0 event */
#define FTM_STATUS_CH1F             (1 << 1) /* Bit 1: Chn1 event */
#define FTM_STATUS_CH2F             (1 << 2) /* Bit 2: Chn2 event */
#define FTM_STATUS_CH3F             (1 << 3) /* Bit 3: Chn3 event */
#define FTM_STATUS_CH4F             (1 << 4) /* Bit 4: Chn4 event */
#define FTM_STATUS_CH5F             (1 << 5) /* Bit 5: Chn5 event */
#define FTM_STATUS_CH6F             (1 << 6) /* Bit 6: Chn6 event */
#define FTM_STATUS_CH7F             (1 << 7) /* Bit 7: Chn7 event */

#endif /* __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_FTM_H */
