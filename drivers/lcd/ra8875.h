/****************************************************************************
 * drivers/lcd/ra8875.h
 * Definitions for the RAiO Technologies RA8875 LCD controller
 *
 *   Copyright (C) 2015 Intuitive Aerial AB. All rights reserved.
 *   Author: Marten Svanfeldt <marten@intuitiveaerial.com>
 *
 * References: RA8875, Rev 1.6, Apr 2013, RAiO Technologies Inc
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

#ifndef __DRIVERS_LCD_RA8875_H
#define __DRIVERS_LCD_RA8875_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_LCD_RA8875

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RA8875 Register Addresses (All with RS=1) */

/* System and Configuration Addresses */

#define RA8875_PWRR       0x01  /* Power and display control */
#define RA8875_MRWC       0x02  /* Memory Read/Write Control/command */
#define RA8875_PCSR       0x04  /* Pixel Clock Setting Register */
#define RA8875_SROC       0x05  /* Serial Flash/ROM Configuration */
#define RA8875_SFCLR      0x06  /* Serial Flash/ROM CLK Setting */
#define RA8875_SYSR       0x10  /* System configuration Register */
#define RA8875_GPI        0x12  /* General Purpose Input */
#define RA8875_GPO        0x13  /* General Purpose Output */
#define RA8875_HDWR       0x14  /* LCD Horizontal Display Width Register */
#define RA8875_HNDFTR     0x15  /* LCD Horizontal Non-Display Period Fine Tuning Register */
#define RA8875_HNDR       0x16  /* LCD Horizontal Non-Display Period Register */
#define RA8875_HSTR       0x17  /* HSYNC Start Position Register */
#define RA8875_HPWR       0x18  /* HSYNC Pulse Width Register */
#define RA8875_VDHR0      0x19  /* LCD Vertical Display Height Register 0 */
#define RA8875_VDHR1      0x1A  /* LCD Vertical Display Height Register 1 */
#define RA8875_VNDR0      0x1B  /* LCD Vertical Non-Display Period Register 0 */
#define RA8875_VNDR1      0x1C  /* LCD Vertical Non-Display Period Register 1 */
#define RA8875_VSTR0      0x1D  /* VSYNC Start Position Register 0 */
#define RA8875_VSTR1      0x1E  /* VSYNC Start Position Register 1 */
#define RA8875_VPWR       0x1F  /* VSYNC Pulse Width Register */

/* LCD Display Control Registers */

#define RA8875_DPCR       0x20  /* Display Configuration Registers */

/* Active Window and Scroll Window Settings */

#define RA8875_HSAW0      0x30  /* Horizontal Start Point of Active Window 0 */
#define RA8875_HSAW1      0x31  /* Horizontal Start Point of Active Window 1 */
#define RA8875_VSAW0      0x32  /* Vertical Start Point of Active Window 0 */
#define RA8875_VSAW1      0x33  /* Vertical Start Point of Active Window 1 */
#define RA8875_HEAW0      0x34  /* Horizontal End Point of Active Window 0 */
#define RA8875_HEAW1      0x35  /* Horizontal End Point of Active Window 1 */
#define RA8875_VEAW0      0x36  /* Vertical End Point of Active Window 0 */
#define RA8875_VEAW1      0x37  /* Vertical End Point of Active Window 1 */

/* Cursor Setting Registers */

#define RA8875_MWCR0      0x40  /* Memory Write Control Register 0 */
#define RA8875_MWCR1      0x41  /* Memory Write Control Register 1 */
#define RA8875_BTCR       0x44  /* Blink Time Control Register */
#define RA8875_MRDC       0x45  /* Memory Read Cursor Direction */
#define RA8875_CURH0      0x46  /* Memory Write Cursor Horizontal Position Register 0 */
#define RA8875_CURH1      0x47  /* Memory Write Cursor Horizontal Position Register 1 */
#define RA8875_CURV0      0x48  /* Memory Write Cursor Vertical Position Register 0 */
#define RA8875_CURV1      0x49  /* Memory Write Cursor Vertical Position Register 1 */
#define RA8875_RCURH0     0x4A  /* Memory Read Cursor Horizontal Position Register 0 */
#define RA8875_RCURH1     0x4B  /* Memory Read Cursor Horizontal Position Register 1 */
#define RA8875_RCURV0     0x4C  /* Memory Read Cursor Vertical Position Register 0 */
#define RA8875_RCURV1     0x4D  /* Memory Read Cursor Vertical Position Register 1 */

/* BTE Control Registers */

#define RA8875_LTPR0      0x50  /* Layer Transparency Register 0 */
#define RA8875_BGCR0      0x60  /* Background Color Register 0 */
#define RA8875_BGCR1      0x61  /* Background Color Register 1 */
#define RA8875_BGCR2      0x62  /* Background Color Register 2 */
#define RA8875_FGCR0      0x63  /* Foreground Color Register 0 */
#define RA8875_FGCR1      0x64  /* Foreground Color Register 1 */
#define RA8875_FGCR2      0x65  /* Foreground Color Register 2 */

/* PWM Control Registers */

#define RA8875_P1CR       0x8A  /* PWM1 Control Register */
#define RA8875_P1DCR      0x8B  /* PWM1 Duty Cycle Register */

#define RA8875_MCLR       0x8E  /* Memory Clear Control Register */

/* PLL Setting Registers */

#define RA8875_PLLC1      0x88  /* PLL Control Register 1 */
#define RA8875_PLLC2      0x89  /* PLL Control Register 2 */

/* Drawing Control Registers */

#define RA8875_DCR        0x90  /* Draw Line/Circle/Square Control Register */
#define RA8875_DLHSR0     0x91  /* Draw Horizontal Start Address Register 0 */
#define RA8875_DLHSR1     0x92  /* Draw Horizontal Start Address Register 1 */
#define RA8875_DLVSR0     0x93  /* Draw Vertical Start Address Register 0 */
#define RA8875_DLVSR1     0x94  /* Draw Vertical Start Address Register 1 */
#define RA8875_DLHER0     0x95  /* Draw Horizontal End Address Register 0 */
#define RA8875_DLHER1     0x96  /* Draw Horizontal End Address Register 1 */
#define RA8875_DLVER0     0x97  /* Draw Vertical End Address Register 0 */
#define RA8875_DLVER1     0x98  /* Draw Vertical End Address Register 1 */
#define RA8875_DCHR0      0x99  /* Draw Circle Center Horizontal Address Register 0 */
#define RA8875_DCHR1      0x9A  /* Draw Circle Center Horizontal Address Register 1 */
#define RA8875_DCVR0      0x9B  /* Draw Circle Center Vertical Address Register 0 */
#define RA8875_DCVR1      0x9C  /* Draw Circle Center Vertical Address Register 1 */
#define RA8875_DCRR       0x9D  /* Draw Circle Radius Register */
#define RA8875_DTPH0      0xA9  /* Draw Triangle Point 2 Horizontal Address Register 0 */
#define RA8875_DTPH1      0xAA  /* Draw Triangle Point 2 Horizontal Address Register 1 */
#define RA8875_DTPV0      0xAB  /* Draw Triangle Point 2 Vertical Address Register 0 */
#define RA8875_DTPV1      0xAC  /* Draw Triangle Point 2 Vertical Address Register 1 */

/* Bit definitions */

/* Power and display control */

#define RA8875_PWRR_DISPLAY_OFF         (0)
#define RA8875_PWRR_DISPLAY_ON          (1<<7)
#define RA8875_PWRR_SLEEP               (1<<1)
#define RA8875_PWRR_SWRESET             (1<<0)

/* Pixel Clock Setting Register */

#define RA8875_PCSR_PCLK_INV            (1<<7)
#define RA8875_PCSR_PERIOD_MASK         (3)
#  define RA8875_PCSR_PERIOD_SYS        (0)
#  define RA8875_PCSR_PERIOD_2SYS       (1)
#  define RA8875_PCSR_PERIOD_4SYS       (2)
#  define RA8875_PCSR_PERIOD_8SYS       (3)

/* System configuration Register */

#define RA8875_SYSR_COLOR_SHIFT         (2)
#define RA8875_SYSR_COLOR_MASK          (3<<RA8875_SYSR_COLOR_SHIFT)
#  define RA8875_SYSR_COLOR_256         (0<<RA8875_SYSR_COLOR_SHIFT)
#  define RA8875_SYSR_COLOR_65K         (3<<RA8875_SYSR_COLOR_SHIFT)
#define RA8875_SYSR_MCUIF_MASK          (3)
#  define RA8875_SYSR_MCUIF_8BIT        (0)
#  define RA8875_SYSR_MCUIF_16BIT       (3)

/* LCD Horizontal Display Width Register */

#define RA8875_HDWR_WIDTH(p)            ((p)/8 - 1)

/* LCD Vertical Display Height Register 0 */

#define RA8875_VDHR0_HEIGHT(p)          (((p) - 1)&0xff)

/* LCD Vertical Display Height Register 1 */

#define RA8875_VDHR1_HEIGHT(p)          (((p) - 1)>>8)

/* Display Configuration Registers */

#define RA8875_DPCR_LAYERS_ONE          (0)
#define RA8875_DPCR_LAYERS_TWO          (1<<7)
#define RA8875_DPCR_HSCAN_INC           (0)
#define RA8875_DPCR_HSCAN_DEC           (1<<3)
#define RA8875_DPCR_VSCAN_INC           (0)
#define RA8875_DPCR_VSCAN_DEC           (1<<2)

/* Memory Write Control Register 0 */

#define RA8875_MWCR0_MODE_GRAPHICS      (0)
#define RA8875_MWCR0_MODE_TEXT          (1<<7)
#define RA8875_MWCR0_CURSOR_INVISIBLE   (0)
#define RA8875_MWCR0_CURSOR_VISIBLE     (1<<6)
#define RA8875_MWCR0_CURSOR_NORMAL      (0)
#define RA8875_MWCR0_CURSOR_BLINK       (1<<5)
#define RA8875_MWCR0_MEMDIR_SHIFT       (2)
#define RA8875_MWCR0_MEMDIR_MASK        (3<<RA8875_MWCR0_MEMDIR_SHIFT)
#  define RA8875_MWCR0_MEMDIR_LEFTRIGHT (0<<RA8875_MWCR0_MEMDIR_SHIFT)
#  define RA8875_MWCR0_MEMDIR_RIGHTLEFT (1<<RA8875_MWCR0_MEMDIR_SHIFT)
#  define RA8875_MWCR0_MEMDIR_TOPDOWN   (2<<RA8875_MWCR0_MEMDIR_SHIFT)
#  define RA8875_MWCR0_MEMDIR_DOWNTOP   (3<<RA8875_MWCR0_MEMDIR_SHIFT)
#define RA8875_MWCR0_WINC_ENABLE        (0)
#define RA8875_MWCR0_WINC_DISABLE       (1<<1)
#define RA8875_MWCR0_RINC_ENABLE        (0)
#define RA8875_MWCR0_RINC_DISABLE       (1<<0)

/* Memory Write Control Register 1 */

#define RA8875_MWCR1_LAYER_1            (0)
#define RA8875_MWCR1_LAYER_2            (1)

/* Memory Read Cursor Direction */

#define RA8875_MRCD_MEMDIR_SHIFT        (0)
#define RA8875_MRCD_MEMDIR_MASK         (3<<RA8875_MWCR0_MEMDIR_SHIFT)
#  define RA8875_MRCD_MEMDIR_LEFTRIGHT  (0<<RA8875_MWCR0_MEMDIR_SHIFT)
#  define RA8875_MRCD_MEMDIR_RIGHTLEFT  (1<<RA8875_MWCR0_MEMDIR_SHIFT)
#  define RA8875_MRCD_MEMDIR_TOPDOWN    (2<<RA8875_MWCR0_MEMDIR_SHIFT)
#  define RA8875_MRCD_MEMDIR_DOWNTOP    (3<<RA8875_MWCR0_MEMDIR_SHIFT)

/* Layer Transparency Register 0 */

#define RA8875_LTPR0_MODE_MASK          (7)
#  define RA8875_LTPR0_MODE_L1          (0)
#  define RA8875_LTPR0_MODE_L2          (1)

/* PWM1 Control Register */

#define RA8875_P1CR_PWM_DISABLE         (0)
#define RA8875_P1CR_PWM_ENABLE          (1<<7)
#define RA8875_P1CR_DISABLE_LOW         (0)
#define RA8875_P1CR_DISABLE_HIGH        (1<<6)
#define RA8875_P1CR_FUNC_PWM            (0)
#define RA8875_P1CR_FUNC_FIXED          (1<<4)
#define RA8875_P1CR_CSDIV_MASK          (0xf)
#  define RA8875_P1CR_CSDIV(p)          ((p) & 0xf)

/* Memory Clear Control Register */

#define RA8875_MCLR_CLEAR               (1<<7)
#define RA8875_MCLR_FULL                (0)
#define RA8875_MCLR_ACTIVE              (1<<6)

/* PLL Control Register 1 */

#define RA8875_PLLC1_PLLDIVM            (1<<7)
#define RA8875_PLLC1_PLLDIVN(p)         ((p)&0x1f)

/* PLL Control Register 2 */

#define RA8875_PLLC2_PLLDIVK(p)         ((p)&0x7)

/* Draw Line/Circle/Square Control Register */

#define RA8875_DCR_LINE_START           (1<<7)
#define RA8875_DCR_CIRCLE_START         (1<<6)
#define RA8875_DCR_NOFILL               (0)
#define RA8875_DCR_FILL                 (1<<5)
#define RA8875_DCR_LINE                 (0)
#define RA8875_DCR_SQUARE               (1<<4)
#define RA8875_DCR_TRIANGLE             (1<<0)

#endif /* CONFIG_LCD_RA8875 */
#endif /* __DRIVERS_LCD_RA8875_H */
