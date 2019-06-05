/************************************************************************************
 * arch/arm/src/kinetis/hardware/kinetis_sim.h
 *
 *   Copyright (C) 2011, 2016-2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_SIM_H
#define __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_SIM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define KINETIS_SIM_SOPT1_OFFSET              0x0000  /* System Options Register 1 */
#if defined (KINETIS_SIM_HAS_SOPT1CFG)
#  define KINETIS_SIM_SOPT1CFG_OFFSET         0x0004  /* SOPT1 Configuration Register */
#endif
#if defined(KINETIS_SIM_HAS_USBPHYCTL)
#  define KINETIS_SIM_USBPHYCTL_OFFSET        0x0008  /* USB PHY Control Register */
#endif
#if defined(KINETIS_SIM_HAS_SOPT2)
#  define KINETIS_SIM_SOPT2_OFFSET            0x0004  /* System Options Register 2 */
#endif
#define KINETIS_SIM_SOPT4_OFFSET              0x000c  /* System Options Register 4 */
#define KINETIS_SIM_SOPT5_OFFSET              0x0010  /* System Options Register 5 */
#define KINETIS_SIM_SOPT6_OFFSET              0x0014  /* System Options Register 6 */
#define KINETIS_SIM_SOPT7_OFFSET              0x0018  /* System Options Register 7 */
#if defined(KINETIS_SIM_HAS_SOPT8)
#  define KINETIS_SIM_SOPT8_OFFSET            0x001c  /* System Options Register 8 */
#endif
#if defined(KINETIS_SIM_HAS_SOPT9)
#  define KINETIS_SIM_SOPT9_OFFSET            0x0020  /* System Options Register 9 */
#endif
#define KINETIS_SIM_SDID_OFFSET               0x0024  /* System Device Identification Register */
#define KINETIS_SIM_SCGC1_OFFSET              0x0028  /* System Clock Gating Control Register 1 */
#define KINETIS_SIM_SCGC2_OFFSET              0x002c  /* System Clock Gating Control Register 2 */
#define KINETIS_SIM_SCGC3_OFFSET              0x0030  /* System Clock Gating Control Register 3 */
#define KINETIS_SIM_SCGC4_OFFSET              0x0034  /* System Clock Gating Control Register 4 */
#define KINETIS_SIM_SCGC5_OFFSET              0x0038  /* System Clock Gating Control Register 5 */
#define KINETIS_SIM_SCGC6_OFFSET              0x003c  /* System Clock Gating Control Register 6 */
#define KINETIS_SIM_SCGC7_OFFSET              0x0040  /* System Clock Gating Control Register 7 */
#define KINETIS_SIM_CLKDIV1_OFFSET            0x0044  /* System Clock Divider Register 1 */
#define KINETIS_SIM_CLKDIV2_OFFSET            0x0048  /* System Clock Divider Register 2 */
#define KINETIS_SIM_FCFG1_OFFSET              0x004c  /* Flash Configuration Register 1 */
#define KINETIS_SIM_FCFG2_OFFSET              0x0050  /* Flash Configuration Register 2 */
#define KINETIS_SIM_UIDH_OFFSET               0x0054  /* Unique Identification Register High */
#define KINETIS_SIM_UIDMH_OFFSET              0x0058  /* Unique Identification Register Mid-High */
#define KINETIS_SIM_UIDML_OFFSET              0x005c  /* Unique Identification Register Mid Low */
#define KINETIS_SIM_UIDL_OFFSET               0x0060  /* Unique Identification Register Low */
#if defined(KINETIS_SIM_HAS_CLKDIV3)
#  define KINETIS_SIM_CLKDIV3_OFFSET          0x0064  /* System Clock Divider Register 3 */
#endif
#if defined(KINETIS_SIM_HAS_CLKDIV4)
#  define KINETIS_SIM_CLKDIV4_OFFSET          0x0068  /* System Clock Divider Register 4 */
#endif

/* Register Addresses ***************************************************************/
/* NOTE: The SIM_SOPT1, SIM_SOPT1CFG and SIM_USBPHYCTL registers are located at a
 * different base address than the other SIM registers.
 */

#define KINETIS_SIM_SOPT1                     (KINETIS_SIMLP_BASE+KINETIS_SIM_SOPT1_OFFSET)
#if defined(KINETIS_SIM_HAS_SOPT1CFG)
#  define KINETIS_SIM_SOPT1CFG                (KINETIS_SIMLP_BASE+KINETIS_SIM_SOPT1CFG_OFFSET)
#endif
#if defined(KINETIS_SIM_HAS_USBPHYCTL)
#  define KINETIS_SIM_USBPHYCTL               (KINETIS_SIMLP_BASE+KINETIS_SIM_USBPHYCTL_OFFSET)
#endif
#if defined(KINETIS_SIM_HAS_SOPT2)
#  define KINETIS_SIM_SOPT2                   (KINETIS_SIM_BASE+KINETIS_SIM_SOPT2_OFFSET)
#endif
#define KINETIS_SIM_SOPT4                     (KINETIS_SIM_BASE+KINETIS_SIM_SOPT4_OFFSET)
#define KINETIS_SIM_SOPT5                     (KINETIS_SIM_BASE+KINETIS_SIM_SOPT5_OFFSET)
#define KINETIS_SIM_SOPT6                     (KINETIS_SIM_BASE+KINETIS_SIM_SOPT6_OFFSET)
#define KINETIS_SIM_SOPT7                     (KINETIS_SIM_BASE+KINETIS_SIM_SOPT7_OFFSET)
#if defined(KINETIS_SIM_HAS_SOPT8)
#  define KINETIS_SIM_SOPT8                   (KINETIS_SIM_BASE+KINETIS_SIM_SOPT8_OFFSET)
#endif
#if defined(KINETIS_SIM_HAS_SOPT9)
#  define KINETIS_SIM_SOPT9                   (KINETIS_SIM_BASE+KINETIS_SIM_SOPT8_OFFSET)
#endif
#define KINETIS_SIM_SDID                      (KINETIS_SIM_BASE+KINETIS_SIM_SDID_OFFSET)
#define KINETIS_SIM_SCGC1                     (KINETIS_SIM_BASE+KINETIS_SIM_SCGC1_OFFSET)
#define KINETIS_SIM_SCGC2                     (KINETIS_SIM_BASE+KINETIS_SIM_SCGC2_OFFSET)
#define KINETIS_SIM_SCGC3                     (KINETIS_SIM_BASE+KINETIS_SIM_SCGC3_OFFSET)
#define KINETIS_SIM_SCGC4                     (KINETIS_SIM_BASE+KINETIS_SIM_SCGC4_OFFSET)
#define KINETIS_SIM_SCGC5                     (KINETIS_SIM_BASE+KINETIS_SIM_SCGC5_OFFSET)
#define KINETIS_SIM_SCGC6                     (KINETIS_SIM_BASE+KINETIS_SIM_SCGC6_OFFSET)
#define KINETIS_SIM_SCGC7                     (KINETIS_SIM_BASE+KINETIS_SIM_SCGC7_OFFSET)
#define KINETIS_SIM_CLKDIV1                   (KINETIS_SIM_BASE+KINETIS_SIM_CLKDIV1_OFFSET)
#define KINETIS_SIM_CLKDIV2                   (KINETIS_SIM_BASE+KINETIS_SIM_CLKDIV2_OFFSET)
#define KINETIS_SIM_FCFG1                     (KINETIS_SIM_BASE+KINETIS_SIM_FCFG1_OFFSET)
#define KINETIS_SIM_FCFG2                     (KINETIS_SIM_BASE+KINETIS_SIM_FCFG2_OFFSET)
#define KINETIS_SIM_UIDH                      (KINETIS_SIM_BASE+KINETIS_SIM_UIDH_OFFSET)
#define KINETIS_SIM_UIDMH                     (KINETIS_SIM_BASE+KINETIS_SIM_UIDMH_OFFSET)
#define KINETIS_SIM_UIDML                     (KINETIS_SIM_BASE+KINETIS_SIM_UIDML_OFFSET)
#define KINETIS_SIM_UIDL                      (KINETIS_SIM_BASE+KINETIS_SIM_UIDL_OFFSET)
#if defined(KINETIS_SIM_HAS_CLKDIV3)
#  define KINETIS_SIM_CLKDIV3                 (KINETIS_SIM_BASE+KINETIS_SIM_CLKDIV3_OFFSET)
#endif
#if defined(KINETIS_SIM_HAS_CLKDIV4)
#  define KINETIS_SIM_CLKDIV4                 (KINETIS_SIM_BASE+KINETIS_SIM_CLKDIV4_OFFSET)
#endif

/* Register Bit Definitions *********************************************************/

/* System Options Register 1 */
                                                         /* Bits 0-11: Reserved */
#if defined(KINETIS_SIM_HAS_SOPT1_RAMSIZE)
#  define SIM_SOPT1_RAMSIZE_SHIFT             (12)       /* Bits 12-15: RAM size */
#  define SIM_SOPT1_RAMSIZE_MASK              (15 << SIM_SOPT1_RAMSIZE_SHIFT)
#    define SIM_SOPT1_RAMSIZE_32KB            (5 << SIM_SOPT1_RAMSIZE_SHIFT)  /* 32 KBytes */
#    define SIM_SOPT1_RAMSIZE_64KB            (7 << SIM_SOPT1_RAMSIZE_SHIFT)  /* 64 KBytes */
#    define SIM_SOPT1_RAMSIZE_96KB            (8 << SIM_SOPT1_RAMSIZE_SHIFT)  /* 96 KBytes */
#    define SIM_SOPT1_RAMSIZE_128KB           (9 << SIM_SOPT1_RAMSIZE_SHIFT)  /* 128 KBytes */
#    define SIM_SOPT1_RAMSIZE_256KB           (10 << SIM_SOPT1_RAMSIZE_SHIFT) /* 256 KBytes */
#endif
                                                         /* Bits 16-18: Reserved */
#if defined(KINETIS_SIM_HAS_SOPT1_OSC32KSEL)
#  define SIM_SOPT1_OSC32KSEL_SHIFT           (20-KINETIS_SIM_HAS_SOPT1_OSC32KSEL_BITS) /* Bit 19 or 18: 32K oscillator clock select */
#  define SIM_SOPT1_OSC32KSEL_MASK            (KINETIS_SIM_SOPT1_OSC32KSEL_MASK << SIM_SOPT1_OSC32KSEL_SHIFT)
#  define SIM_SOPT1_OSC32KSEL(n)              ((((n) & KINETIS_SIM_SOPT1_OSC32KSEL_MASK)) << SIM_SOPT1_OSC32KSEL_SHIFT)
#  if KINETIS_SIM_HAS_SOPT1_OSC32KSEL_BITS == 1
#    define SIM_SOPT1_OSC32KSEL_OSC32KCLK     (((0 & KINETIS_SIM_SOPT1_OSC32KSEL_MASK)) << SIM_SOPT1_OSC32KSEL_SHIFT)
#    define SIM_SOPT1_OSC32KSEL_RTC           (((1 & KINETIS_SIM_SOPT1_OSC32KSEL_MASK)) << SIM_SOPT1_OSC32KSEL_SHIFT)
#  endif
#  if KINETIS_SIM_HAS_SOPT1_OSC32KSEL_BITS == 2
#    define SIM_SOPT1_OSC32KSEL_OSC32KCLK      (((0 & KINETIS_SIM_SOPT1_OSC32KSEL_MASK)) << SIM_SOPT1_OSC32KSEL_SHIFT)
#    define SIM_SOPT1_OSC32KSEL_RTC            (((2 & KINETIS_SIM_SOPT1_OSC32KSEL_MASK)) << SIM_SOPT1_OSC32KSEL_SHIFT)
#    define SIM_SOPT1_OSC32KSEL_LPO1KZ         (((3 & KINETIS_SIM_SOPT1_OSC32KSEL_MASK)) << SIM_SOPT1_OSC32KSEL_SHIFT)
#  endif
#endif
                                                         /* Bits 20-28: Reserved */
#if defined(KINETIS_SIM_HAS_SOPT1_USBVSTBY)
                                                         /* Bits 24-28: Reserved */
#  define SIM_SOPT1_USBVSTBY                  (1 << 29)  /* Bit 29: USB voltage regulator in standby mode during VLPR and VLPW modes */
#endif
#if defined(KINETIS_SIM_HAS_SOPT1_USBSSTBY)
#  define SIM_SOPT1_USBSTBY                   (1 << 30)  /* Bit 30: USB voltage regulator in standby mode */
#endif
#if defined(KINETIS_SIM_HAS_SOPT1_USBREGEN)
#  define SIM_SOPT1_USBREGEN                  (1 << 31)  /* Bit 31: USB voltage regulator enable */
#endif

/* SOPT1 Configuration Register */

#if defined(KINETIS_SIM_HAS_SOPT1CFG)
                                                         /* Bits 0-22: Reserved */
#  if defined(KINETIS_SIM_HAS_SOPT1CFG_URWE)
#    define SIM_SOPT1CFG_URWE                 (1 << 24)  /* Bit 24: USB voltage regulator enable write enable */
#  endif
#  if defined(KINETIS_SIM_HAS_SOPT1CFG_USSWE)
#    define SIM_SOPT1CFG_USSWE                (1 << 25)  /* Bit 25: USB voltage regulator VLP standby write enable */
#  endif
#  if defined(KINETIS_SIM_HAS_SOPT1CFG_UVSWE)
#    define SIM_SOPT1CFG_UVSWE                (1 << 26)  /* Bit 26: USB voltage regulator stop standby write enable */
#  endif
                                                         /* Bits 27-31: Reserved */
#endif

/* USB PHY Control Register */

#if defined(KINETIS_SIM_HAS_USBPHYCTL)
                                                         /* Bits 0-7: Reserved */
#  if defined(KINETIS_SIM_HAS_USBPHYCTL_USBVREGSEL)
#    define SIM_USBPHYCTL_USBVREGSEL          (1 << 8)   /* Bit 8: Selects the default input voltage source */
#  endif
#  if defined(KINETIS_SIM_HAS_USBPHYCTL_USBVREGPD)
#    define SIM_USBPHYCTL_USBVREGPD           (1 << 9)   /* Bit 9: Enables the pulldown on the output of the USB Regulator */
#  endif
                                                         /* Bits 10-19: Reserved */
#  if defined(KINETIS_SIM_HAS_USBPHYCTL_USB3VOUTTRG)
#    define SIM_USBPHYCTL_USB3VOUTTRG_SHIFT   (20)       /* Bit 20-22: USB 3.3V Output Target */
#    define SIM_USBPHYCTL_USB3VOUTTRG_MASK    (7 << SIM_USBPHYCTL_USB3VOUTTRG_SHIFT)
#      define SIM_USBPHYCTL_USB3VOUTTRG_2V733 (0 << SIM_USBPHYCTL_USB3VOUTTRG_SHIFT) /* 2.733V */
#      define SIM_USBPHYCTL_USB3VOUTTRG_3V020 (1 << SIM_USBPHYCTL_USB3VOUTTRG_SHIFT) /* 3.020V */
#      define SIM_USBPHYCTL_USB3VOUTTRG_3V074 (2 << SIM_USBPHYCTL_USB3VOUTTRG_SHIFT) /* 3.074V */
#      define SIM_USBPHYCTL_USB3VOUTTRG_3V130 (3 << SIM_USBPHYCTL_USB3VOUTTRG_SHIFT) /* 3.130V */
#      define SIM_USBPHYCTL_USB3VOUTTRG_3V188 (4 << SIM_USBPHYCTL_USB3VOUTTRG_SHIFT) /* 3.188V */
#      define SIM_USBPHYCTL_USB3VOUTTRG_3V248 (5 << SIM_USBPHYCTL_USB3VOUTTRG_SHIFT) /* 3.248V */
#      define SIM_USBPHYCTL_USB3VOUTTRG_3V310 (6 << SIM_USBPHYCTL_USB3VOUTTRG_SHIFT) /* 3.310V (default) */
#      define SIM_USBPHYCTL_USB3VOUTTRG_3V662 (7 << SIM_USBPHYCTL_USB3VOUTTRG_SHIFT) /* 3.662V (For Freescale use only, not for customer use) */
#  endif
#  if defined(KINETIS_SIM_HAS_USBPHYCTL_USBDISILIM)
#    define SIM_USBPHYCTL_USBDISILIM           (1 << 23)  /* Bit 23: USB Disable Inrush Current Limit */
#  endif
                                                         /* Bits 24-31: Reserved */
#endif

/* System Options Register 2 */

#if defined(KINETIS_SIM_HAS_SOPT2)
#  if defined(KINETIS_SIM_HAS_SOPT2_MCGCLKSEL)
#    define SIM_SOPT2_MCGCLKSEL               (1 << 0)  /* Bit 0: MCG clock select */
#  endif
#  if defined(KINETIS_SIM_HAS_SOPT2_USBSLSRC)
#    define SIM_SOPT2_USBSLSRC                (1 << 0)  /* Bit 0: USB Slow Clock Source */
#  endif
#  if defined(KINETIS_SIM_HAS_SOPT2_USBREGEN)
#    define SIM_SOPT2_USBREGEN                (1 << 1)  /* Bit 1: USB PHY PLL Regulator Enable */
#  endif
                                                        /* Bits 2-3: Reserved */
#  if defined(KINETIS_SIM_HAS_SOPT2_USBHSRC)
#    define SIM_SOPT2_USBSHSRC_SHIFT          (2)       /* Bit 2-3: USB HS clock source select */
#    define SIM_SOPT2_USBSHSRC_MASK           (3 << SIM_SOPT2_USBSHSRC_SHIFT)
#      define SIM_SOPT2_USBSHSRC_BUSCLK       (0 << SIM_SOPT2_USBSHSRC_SHIFT)
#      define SIM_SOPT2_USBSHSRC_MCGPLL0CLK   (1 << SIM_SOPT2_USBSHSRC_SHIFT)
#      define SIM_SOPT2_USBSHSRC_MCGPLL1CLK   (2 << SIM_SOPT2_USBSHSRC_SHIFT)
#      define SIM_SOPT2_USBSHSRC_OSC0ERCLK    (3 << SIM_SOPT2_USBSHSRC_SHIFT)
#  endif
#  if defined(KINETIS_SIM_HAS_SOPT2_RTCCLKOUTSEL)
#    define SIM_SOPT2_RTCCLKOUTSEL            (1 << 4)  /* Bit 4: RTC clock out select */
#  endif
#  if defined(KINETIS_SIM_HAS_SOPT2_CLKOUTSEL)
#    define SIM_SOPT2_CLKOUTSEL_SHIFT         (5)       /* Bits 5-7: CLKOUT select */
#    define SIM_SOPT2_CLKOUTSEL_MASK          (7 << SIM_SOPT2_CLKOUTSEL_SHIFT)
#      define SIM_SOPT2_CLKOUTSEL_FBCLKOUT    (0 << SIM_SOPT2_CLKOUTSEL_SHIFT)
#      define SIM_SOPT2_CLKOUTSEL_FLSHCLK     (2 << SIM_SOPT2_CLKOUTSEL_SHIFT)
#      define SIM_SOPT2_CLKOUTSEL_LPO1KHZ     (3 << SIM_SOPT2_CLKOUTSEL_SHIFT)
#      define SIM_SOPT2_CLKOUTSEL_MCGIRCLK    (4 << SIM_SOPT2_CLKOUTSEL_SHIFT)
#      define SIM_SOPT2_CLKOUTSEL_RTC32768KHZ (5 << SIM_SOPT2_CLKOUTSEL_SHIFT)
#      define SIM_SOPT2_CLKOUTSEL_OSCERCLK0   (6 << SIM_SOPT2_CLKOUTSEL_SHIFT)
#      define SIM_SOPT2_CLKOUTSEL_IRC48MHZ    (7 << SIM_SOPT2_CLKOUTSEL_SHIFT)
#  endif
#  if defined(KINETIS_SIM_HAS_SOPT2_FBSL)
#    define SIM_SOPT2_FBSL_SHIFT              (8)       /* Bits 8-9: FlexBus security level */
#    define SIM_SOPT2_FBSL_MASK               (3 << SIM_SOPT2_FBSL_SHIFT)
#      define SIM_SOPT2_FBSL_NONE             (0 << SIM_SOPT2_FBSL_SHIFT) /* All off-chip accesses disallowed */
#      define SIM_SOPT2_FBSL_DATA             (2 << SIM_SOPT2_FBSL_SHIFT) /* Off-chip data accesses are allowed */
#      define SIM_SOPT2_FBSL_ALL              (3 << SIM_SOPT2_FBSL_SHIFT) /* All Off-chip accesses allowed */
#  endif
                                                        /* Bit 10: Reserved */
#  if defined(KINETIS_SIM_HAS_SOPT2_CMTUARTPAD)
#    define SIM_SOPT2_CMTUARTPAD              (1 << 11) /* Bit 11: CMT/UART pad drive strength */
#  endif
#  if defined(KINETIS_SIM_HAS_SOPT2_PTD7PAD)
#    define SIM_SOPT2_PTD7PAD                 (1 << 11) /* Bit 11: PTD7P pad drive strength */
#  endif
#  if defined(KINETIS_SIM_HAS_SOPT2_TRACECLKSEL)
#    define SIM_SOPT2_TRACECLKSEL             (1 << 12) /* Bit 12: Debug trace clock select */
#  endif
                                                        /* Bits 13-15: Reserved */
#  if defined(KINETIS_SIM_HAS_SOPT2_PLLFLLSEL)
#    define SIM_SOPT2_PLLFLLSEL_SHIFT         (16)      /* Bits 16-[17]: PLL/FLL clock select */
#    define SIM_SOPT2_PLLFLLSEL_MASK          (KINETIS_SIM_SOPT2_PLLFLLSEL_MASK << SIM_SOPT2_PLLFLLSEL_SHIFT)
#      define SIM_SOPT2_PLLFLLSEL(n)          (((n) &  KINETIS_SIM_SOPT2_PLLFLLSEL_MASK) << SIM_SOPT2_PLLFLLSEL_SHIFT)
#      define SIM_SOPT2_PLLFLLSEL_MCGFLLCLK   ((0 &  KINETIS_SIM_SOPT2_PLLFLLSEL_MASK) << SIM_SOPT2_PLLFLLSEL_SHIFT)
#      define SIM_SOPT2_PLLFLLSEL_MCGPLLCLK   ((1 &  KINETIS_SIM_SOPT2_PLLFLLSEL_MASK) << SIM_SOPT2_PLLFLLSEL_SHIFT)
#      if KINETIS_SIM_HAS_SOPT2_PLLFLLSEL_BITS > 1
#        define SIM_SOPT2_PLLFLLSEL_USB1PFD   ((2 &  KINETIS_SIM_SOPT2_PLLFLLSEL_MASK) << SIM_SOPT2_PLLFLLSEL_SHIFT)
#        define SIM_SOPT2_PLLFLLSEL_IRC48MHZ  ((3 &  KINETIS_SIM_SOPT2_PLLFLLSEL_MASK) << SIM_SOPT2_PLLFLLSEL_SHIFT)
#      endif
#  endif
                                                        /* Bit 17: Reserved */
#  if defined(KINETIS_SIM_HAS_SOPT2_USBSRC)
#    define SIM_SOPT2_USBSRC                  (1 << 18) /* Bit 18: USB clock source select */
#  endif
                                                        /* Bit 19: Reserved */
#  if defined(KINETIS_SIM_HAS_SOPT2_RMIISRC)
#    define SIM_SOPT2_RMIISRC_SHIFT           (19)      /* Bit 19: RMII clock source select */
#    define SIM_SOPT2_RMIISRC_EXTAL           (0 << SIM_SOPT2_RMIISRC_SHIFT) /* EXTAL clock */
#    define SIM_SOPT2_RMIISRC_EXTBYP          (1 << SIM_SOPT2_RMIISRC_SHIFT) /* External bypass clock (ENET_1588_CLKIN) */
#  endif
#  if defined(KINETIS_SIM_HAS_SOPT2_TIMESRC)
#    define SIM_SOPT2_TIMESRC_SHIFT           (20)      /* Bit 20-21: IEEE 1588 timestamp clock source select */
#    define SIM_SOPT2_TIMESRC_MASK            (3 << SIM_SOPT2_TIMESRC_SHIFT)
#      define SIM_SOPT2_TIMESRC_CORE          (0 << SIM_SOPT2_TIMESRC_SHIFT) /* Core/system clock */
#      define SIM_SOPT2_TIMESRC_PLLSEL        (1 << SIM_SOPT2_TIMESRC_SHIFT) /* MCGFLLCLK,MCGPLLCLK,IRC48M,USB1 PFD
                                                                                  clock as selected by SOPT2[PLLFLLSEL] */
#      define SIM_SOPT2_TIMESRC_OSCERCLK      (2 << SIM_SOPT2_TIMESRC_SHIFT) /* OSCERCLK clock */
#      define SIM_SOPT2_TIMESRC_EXTBYP        (3 << SIM_SOPT2_TIMESRC_SHIFT) /* External bypass clock (ENET_1588_CLKIN) */
#  endif
#  if defined(KINETIS_SIM_HAS_SOPT2_FLEXIOSRC)
#    define SIM_SOPT2_FLEXIOSRC_SHIFT         (22)    /* Bits 22-23: FlexIO Module Clock Source Select */
#    define SIM_SOPT2_FLEXIOSRC_MASK          (3 << SIM_SOPT2_FLEXIOSRC_SHIFT)
#      define SIM_SOPT2_FLEXIOSRC_CORE        (0 << SIM_SOPT2_FLEXIOSRC_SHIFT) /* Core/system clock */
#      define SIM_SOPT2_FLEXIOSRC_PLLSEL      (1 << SIM_SOPT2_FLEXIOSRC_SHIFT) /* MCGFLLCLK,MCGPLLCLK,IRC48M,USB1 PFD
                                                                                * clock as selected by SOPT2[PLLFLLSEL] */
#      define SIM_SOPT2_FLEXIOSRC_OSCERCLK    (2 << SIM_SOPT2_FLEXIOSRC_SHIFT) /* OSCERCLK clock */
#      define SIM_SOPT2_FLEXIOSRC_MCGIRCLK    (3 << SIM_SOPT2_FLEXIOSRC_SHIFT) /* MCGIRCLK clock */
#  endif
#  if defined(KINETIS_SIM_HAS_SOPT2_USBFSRC)
#    define SIM_SOPT2_USBFSRC_SHIFT           (22)     /* Bits 22-23: USB FS clock source select */
#    define SIM_SOPT2_USBFSRC_MASK            (3 << SIM_SOPT2_USBFSRC_SHIFT)
#      define SIM_SOPT2_USBFSRC_MCGCLK        (0 << SIM_SOPT2_USBFSRC_SHIFT) /* MCGFLLCLK,MCGPLLCLK clock as selected by SOPT2[PLLFLLSEL] */
#      define SIM_SOPT2_USBFSRC_MCGPLL0CLK    (1 << SIM_SOPT2_USBFSRC_SHIFT) /* MCGPLL0CLK clock */
#      define SIM_SOPT2_USBFSRC_MCGPLL1CLK    (2 << SIM_SOPT2_USBFSRC_SHIFT) /* MCGPLL1CLK clock */
#      define SIM_SOPT2_USBFSRC_OCS0ERCLK     (3 << SIM_SOPT2_USBFSRC_SHIFT) /* OSC0ERCLK clock */
#  endif
#  if defined(KINETIS_SIM_HAS_SOPT2_TPMSRC)
#    define SIM_SOPT2_TPMSRC_SHIFT            (24)      /* Bits 24-25: TPM clock source select */
#    define SIM_SOPT2_TPMSRC_MASK             (3 << SIM_SOPT2_TPMSRC_SHIFT)
#      define SIM_SOPT2_TPMSRC_CORE           (0 << SIM_SOPT2_TPMSRC_SHIFT) /* Clock disabled */
#      define SIM_SOPT2_TPMSRC_MCGCLK         (1 << SIM_SOPT2_TPMSRC_SHIFT) /* MCGFLLCLK,MCGPLLCLK,IRC48M,USB1 PFD
                                                                                     clock as selected by SOPT2[PLLFLLSEL] and then
                                                                                     divided by the PLLFLLCLK fractional divider
                                                                                     as configured by SIM_CLKDIV3[PLLFLLFRAC, PLLFLLDIV] */
#      define SIM_SOPT2_TPMSRC_OCSERCLK       (2 << SIM_SOPT2_TPMSRC_SHIFT) /* OSCERCLK clock */
#      define SIM_SOPT2_TPMSRC_MCGIRCLK       (3 << SIM_SOPT2_TPMSRC_SHIFT) /* MCGIRCLK clock */
#  endif
#  if defined(KINETIS_SIM_HAS_SOPT2_I2SSRC)
#      define SIM_SOPT2_I2SSRC_SHIFT          (24)      /* Bits 24-25: I2S master clock source select */
#      define SIM_SOPT2_I2SSRC_MASK           (3 << SIM_SOPT2_I2SSRC_SHIFT)
#        define SIM_SOPT2_I2SCSRC_CORE        (0 << SIM_SOPT2_I2SSRC_SHIFT) /* Core/system clock / I2S fractional divider */
#        define SIM_SOPT2_I2SCSRC_MCGCLK      (1 << SIM_SOPT2_I2SSRC_SHIFT) /* MCGPLLCLK/MCGFLLCLK clock/ I2S fractional divider */
#        define SIM_SOPT2_I2SCSRC_OCSERCLK    (2 << SIM_SOPT2_I2SSRC_SHIFT) /* OSCERCLK clock */
#        define SIM_SOPT2_I2SCSRC_EXTBYP      (3 << SIM_SOPT2_I2SSRC_SHIFT) /* External bypass clock (I2S0_CLKIN) */
#  endif
                                                        /* Bits 26-27: Reserved */
#  if defined(KINETIS_SIM_HAS_SOPT2_LPUARTSRC)
#    define SIM_SOPT2_LPUARTSRC_SHIFT         (26)      /* Bits 26-27: LPUART clock source select */
#    define SIM_SOPT2_LPUARTSRC_MASK          (3 << SIM_SOPT2_LPUARTSRC_SHIFT)
#      define SIM_SOPT2_LPUARTSRC_CORE        (0 << SIM_SOPT2_LPUARTSRC_SHIFT) /* Clock disabled */
#      define SIM_SOPT2_LPUARTSRC_MCGCLK      (1 << SIM_SOPT2_LPUARTSRC_SHIFT) /* MCGFLLCLK,MCGPLLCLK,IRC48M,USB1 PFD
                                                                                  clock as selected by SOPT2[PLLFLLSEL] and then
                                                                                  divided by the PLLFLLCLK fractional divider
                                                                                  as configured by SIM_CLKDIV3[PLLFLLFRAC, PLLFLLDIV] */
#      define SIM_SOPT2_LPUARTSRC_OCSERCLK    (2 << SIM_SOPT2_LPUARTSRC_SHIFT) /* OSCERCLK clock */
#      define SIM_SOPT2_LPUARTSRC_MCGIRCLK    (3 << SIM_SOPT2_LPUARTSRC_SHIFT) /* MCGIRCLK clock */
#  endif
#  if defined(KINETIS_SIM_HAS_SOPT2_SDHCSRC)
#    define SIM_SOPT2_SDHCSRC_SHIFT           (28)      /* Bits 28-29: SDHC clock source select */
#    define SIM_SOPT2_SDHCSRC_MASK            (3 << SIM_SOPT2_SDHCSRC_SHIFT)
#      define SIM_SOPT2_SDHCSRC_CORE          (0 << SIM_SOPT2_SDHCSRC_SHIFT) /* Core/system clock */
#      define SIM_SOPT2_SDHCSRC_MCGCLK        (1 << SIM_SOPT2_SDHCSRC_SHIFT) /* MCGPLLCLK/MCGFLLCLK clock */
#      define SIM_SOPT2_SDHCSRC_OCSERCLK      (2 << SIM_SOPT2_SDHCSRC_SHIFT) /* OSCERCLK clock */
#      define SIM_SOPT2_SDHCSRC_EXTBYP        (3 << SIM_SOPT2_SDHCSRC_SHIFT) /* External bypass clock (SDHC0_CLKIN) */
                                                        /* Bits 30-31: Reserved */
#  endif
                                                        /* Bits 30-31: Reserved */
#  if defined(KINETIS_SIM_HAS_SOPT2_NFCSRC)
#    define SIM_SOPT2_NFCSRC_SHIFT            (30)      /* Bits 30-31: NFC Flash clock source select */
#    define SIM_SOPT2_NFCSRC_MASK             (3 << SIM_SOPT2_NFCSRC_SHIFT)
#      define SIM_SOPT2_NFCSRC_BUS            (0 << SIM_SOPT2_NFCSRC_SHIFT) /* BUS clock */
#      define SIM_SOPT2_NFCSRC_MCGPLL0CLK     (1 << SIM_SOPT2_NFCSRC_SHIFT) /* MCGPLL0CLK clock */
#      define SIM_SOPT2_NFCSRC_MCGPLL1CLK     (2 << SIM_SOPT2_NFCSRC_SHIFT) /* MCGPLL1CLK clock */
#      define SIM_SOPT2_NFCSRC_OCS0ERCLK      (3 << SIM_SOPT2_NFCSRC_SHIFT) /* OSC0ERCLK clock */
#  endif
#endif

/* System Options Register 4 */

#define SIM_SOPT4_FTM0FLT0                    (1 << 0)  /* Bit 0:  FTM0 Fault 0 Select */
#define SIM_SOPT4_FTM0FLT1                    (1 << 1)  /* Bit 1:  FTM0 Fault 1 Select */
#if defined(KINETIS_SIM_HAS_SOPT4_FTM0FLT2)
#  define SIM_SOPT4_FTM0FLT2                  (1 << 2)  /* Bit 2:  FTM0 Fault 2 Select */
#endif
                                                        /* Bit 3: Reserved */
#if defined(KINETIS_SIM_HAS_SOPT4_FTM0FLT3)
#  define SIM_SOPT4_FTM0FLT3                  (1 << 3)  /* Bit 3:  FTM0 Fault 3 Select */
#endif
#define SIM_SOPT4_FTM1FLT0                    (1 << 4)  /* Bit 4:  FTM1 Fault 0 Select */
                                                        /* Bits 5-7: Reserved */
#define SIM_SOPT4_FTM2FLT0                    (1 << 8)  /* Bit 8:  FTM2 Fault 0 Select */
                                                        /* Bits 9-17: Reserved */
#if defined(KINETIS_SIM_HAS_SOPT4_FTM3FLT0)
                                                        /* Bits 9-11,13-17: Reserved */
#  define SIM_SOPT4_FTM3FLT0                  (1 << 12) /* Bit 12:  FTM3 Fault 0 Select */
#endif
#if defined(KINETIS_SIM_HAS_SOPT4_FTM1CH0SRC)
#  define SIM_SOPT4_FTM1CH0SRC_SHIFT          (18)      /* Bits 18-19: FTM1 channel 0 input capture source select */
#  define SIM_SOPT4_FTM1CH0SRC_MASK           (3 << SIM_SOPT4_FTM1CH0SRC_SHIFT)
#    define SIM_SOPT4_FTM1CH0SRC_CH0          (0 << SIM_SOPT4_FTM1CH0SRC_SHIFT) /* FTM1_CH0 signal */
#    define SIM_SOPT4_FTM1CH0SRC_CMP0         (1 << SIM_SOPT4_FTM1CH0SRC_SHIFT) /* CMP0 output */
#    define SIM_SOPT4_FTM1CH0SRC_CMP1         (2 << SIM_SOPT4_FTM1CH0SRC_SHIFT) /* CMP1 output */
#  if KINETIS_SIM_HAS_SOPT4_FTM1CH0SRC > 2
#      define SIM_SOPT4_FTM1CH0SRC_USBSOF     (3 << SIM_SOPT4_FTM1CH0SRC_SHIFT) /* USB start of frame pulse */
#  endif
#endif
#if defined(KINETIS_SIM_HAS_SOPT4_FTM2CH0SRC)
#  define SIM_SOPT4_FTM2CH0SRC_SHIFT           (20)     /* Bits 20-21: FTM2 channel 0 input capture source select */
#  define SIM_SOPT4_FTM2CH0SRC_MASK            (3 << SIM_SOPT4_FTM2CH0SRC_SHIFT)
#    define SIM_SOPT4_FTM2CH0SRC_CH0           (0 << SIM_SOPT4_FTM2CH0SRC_SHIFT) /* FTM2_CH0 signal */
#    define SIM_SOPT4_FTM2CH0SRC_CMP0          (1 << SIM_SOPT4_FTM2CH0SRC_SHIFT) /* CMP0 output */
#    define SIM_SOPT4_FTM2CH0SRC_CMP1          (2 << SIM_SOPT4_FTM2CH0SRC_SHIFT) /* CMP1 output */
#endif
                                                        /* Bits 22-23: Reserved */
#if defined(KINETIS_SIM_HAS_SOPT4_FTM2CH1SRC)
  #define SIM_SOPT4_FTM2CH1SRC                (1 << 22) /* Bit 22:  FTM2 channel 1 input capture source select */
                                                        /* Bit 23: Reserved */
#endif
#define SIM_SOPT4_FTM0CLKSEL                  (1 << 24) /* Bit 24:  FlexTimer 0 External Clock Pin Select */
#define SIM_SOPT4_FTM1CLKSEL                  (1 << 25) /* Bit 25:  FTM1 External Clock Pin Select */
#define SIM_SOPT4_FTM2CLKSEL                  (1 << 26) /* Bit 26:  FlexTimer 2 External Clock Pin Select */
                                                        /* Bits 27-31: Reserved */
#if defined(KINETIS_SIM_HAS_SOPT4_FTM3TRG1SRC) || defined(KINETIS_SIM_HAS_SOPT4_FTM3TRG0SRC)
#  define SIM_SOPT4_FTM3CLKSEL                (1 << 27) /* Bit 27: FlexTimer 3 External Clock Pin Select */
#endif
#if defined(KINETIS_SIM_HAS_SOPT4_FTM0TRG0SRC)
                                                        /* Bits 27,30-31: Reserved */
#  define SIM_SOPT4_FTM0TRG0SRC               (1 << 28) /* Bit 28:  FlexTimer 0 Hardware Trigger 0 Source Select */
#endif
#if defined(KINETIS_SIM_HAS_SOPT4_FTM0TRG1SRC)
                                                        /* Bits 27,30-31: Reserved */
#  define SIM_SOPT4_FTM0TRG1SRC               (1 << 29) /* Bit 29:  FlexTimer 0 Hardware Trigger 1 Source Select */
#endif
#if defined(KINETIS_SIM_HAS_SOPT4_FTM3TRG0SRC)
#  define SIM_SOPT4_FTM3TRG0SRC               (1 << 30) /* Bit 30: FlexTimer 3 Hardware Trigger 0 Source Select */
#endif
#if defined(KINETIS_SIM_HAS_SOPT4_FTM3TRG1SRC)
#  define SIM_SOPT4_FTM3TRG1SRC               (1 << 31) /* Bit 31: FlexTimer 3 Hardware Trigger 1 Source Select */
#endif

/* System Options Register 5 */

#if defined(KINETIS_SIM_HAS_SOPT5_UART0TXSRC)
#  define SIM_SOPT5_UART0TXSRC_SHIFT          (0)       /* Bits 0-1: UART 0 transmit data source select */
#  define SIM_SOPT5_UART0TXSRC_MASK           (3 << SIM_SOPT5_UART0TXSRC_SHIFT)
#    define SIM_SOPT5_UART0TXSRC_TX           (0 << SIM_SOPT5_UART0TXSRC_SHIFT) /* UART0_TX pin */
#    define SIM_SOPT5_UART0TXSRC_FTM1         (1 << SIM_SOPT5_UART0TXSRC_SHIFT) /* UART0_TX modulated with FTM1 ch0 output */
#    define SIM_SOPT5_UART0TXSRC_FTM2         (2 << SIM_SOPT5_UART0TXSRC_SHIFT) /* UART0_TX modulated with FTM2 ch0 output */
#endif
#if defined(KINETIS_SIM_HAS_SOPT5_UART0RXSRC)
#  define SIM_SOPT5_UART0RXSRC_SHIFT          (2)       /* Bits 2-3: UART 0 receive data source select */
#  define SIM_SOPT5_UART0RXSRC_MASK           (3 << SIM_SOPT5_UART0RXSRC_SHIFT)
#    define SIM_SOPT5_UART0RXSRC_RX           (0 << SIM_SOPT5_UART0RXSRC_SHIFT) /* UART0_RX pin */
#    define SIM_SOPT5_UART0RXSRC_CMP0         (1 << SIM_SOPT5_UART0RXSRC_SHIFT) /* CMP0 */
#    define SIM_SOPT5_UART0RXSRC_CMP1         (2 << SIM_SOPT5_UART0RXSRC_SHIFT) /* CMP1 */
#endif
#if defined(KINETIS_SIM_HAS_SOPT5_UART1TXSRC)
#  define SIM_SOPT5_UART1TXSRC_SHIFT          (4)       /* Bits 4-5: UART 1 transmit data source select */
#  define SIM_SOPT5_UART1TXSRC_MASK           (3 << SIM_SOPT5_UART1TXSRC_SHIFT)
#    define SIM_SOPT5_UART1TXSRC_TX           (0 << SIM_SOPT5_UART1TXSRC_SHIFT) /* UART1_TX pin */
#    define SIM_SOPT5_UART1TXSRC_FTM1         (1 << SIM_SOPT5_UART1TXSRC_SHIFT) /* UART1_TX modulated with FTM1 ch0 output */
#    define SIM_SOPT5_UART1TXSRC_FTM2         (2 << SIM_SOPT5_UART1TXSRC_SHIFT) /* UART1_TX modulated with FTM2 ch0 output */
#endif
#if defined(KINETIS_SIM_HAS_SOPT5_UART1RXSRC)
#  define SIM_SOPT5_UART1RXSRC_SHIFT          (6)       /* Bits 6-7: UART 1 receive data source select */
#  define SIM_SOPT5_UART1RXSRC_MASK           (3 << SIM_SOPT5_UART1RXSRC_SHIFT)
#    define SIM_SOPT5_UART1RXSRC_RX           (0 << SIM_SOPT5_UART1RXSRC_SHIFT) /* UART1_RX pin */
#    define SIM_SOPT5_UART1RXSRC_CMP0         (1 << SIM_SOPT5_UART1RXSRC_SHIFT) /* CMP0 */
#    define SIM_SOPT5_UART1RXSRC_CMP1         (2 << SIM_SOPT5_UART1RXSRC_SHIFT) /* CMP1 */
#endif
                                                        /* Bits 8-31: Reserved */
#if defined(KINETIS_SIM_HAS_SOPT5_LPUART0TXSRC)
                                                        /* Bits 8-15, 18-31: Reserved */
#  define SIM_SOPT5_LPUART0TXSRC_SHIFT        (16)      /* Bit 16:  LPUART0 transmit data source select */
#  define SIM_SOPT5_LPUART0TXSRC_MASK         (3 << SIM_SOPT5_LPUART0TXSRC_SHIFT)
#    define SIM_SOPT5_LPUART0TXSRC_TX         (0 << SIM_SOPT5_LPUART0TXSRC_SHIFT) /* LPUART0_TX pin */
#    define SIM_SOPT5_LPUART0TXSRC_TXTMP1CH0  (1 << SIM_SOPT5_LPUART0TXSRC_SHIFT) /* LPUART0_TX pin modulated with TPM1 channel 0 output */
#    define SIM_SOPT5_LPUART0TXSRC_TXTMP2CH0  (2 << SIM_SOPT5_LPUART0TXSRC_SHIFT) /* LPUART0_TX pin modulated with TPM2 channel 0 output */
#endif
                                                        /* Bits 8-15, 18-31: Reserved */
#if defined(KINETIS_SIM_HAS_SOPT5_LPUART0RXSRC)
                                                        /* Bits 8-15, 20-31: Reserved */
#  define SIM_SOPT5_LPUART0RXSRC_SHIFT        (18)      /* Bit 18:  LPUART0 receive data source select */
#  define SIM_SOPT5_LPUART0RXSRC_MASK         (3 << SIM_SOPT5_LPUART0RXSRC_SHIFT)
#    define SIM_SOPT5_LPUART0RXSRC_TX         (0 << SIM_SOPT5_LPUART0RXSRC_SHIFT) /* LPUART0_RX pin */
#    define SIM_SOPT5_LPUART0RXSRC_TXTMP1CH0  (1 << SIM_SOPT5_LPUART0RXSRC_SHIFT) /* CMP0 output */
#    define SIM_SOPT5_LPUART0RXSRC_TXTMP2CH0  (2 << SIM_SOPT5_LPUART0RXSRC_SHIFT) /* CMP1 output */
#endif

#if defined(KINETIS_SIM_HAS_SOPT5_LPUART1TXSRC)
                                                        /* Bits 8-15, 18-31: Reserved */
#  define SIM_SOPT5_LPUART1TXSRC_SHIFT        (16)      /* Bit 16:  LPUART1 transmit data source select */
#  define SIM_SOPT5_LPUART1TXSRC_MASK         (3 << SIM_SOPT5_LPUART1TXSRC_SHIFT)
#    define SIM_SOPT5_LPUART1TXSRC_TX         (0 << SIM_SOPT5_LPUART1TXSRC_SHIFT) /* LPUART1_TX pin */
#    define SIM_SOPT5_LPUART1TXSRC_TXTMP1CH0  (1 << SIM_SOPT5_LPUART1TXSRC_SHIFT) /* LPUART1_TX pin modulated with TPM1 channel 0 output */
#    define SIM_SOPT5_LPUART1TXSRC_TXTMP2CH0  (2 << SIM_SOPT5_LPUART1TXSRC_SHIFT) /* LPUART1_TX pin modulated with TPM2 channel 0 output */
#endif
                                                        /* Bits 8-15, 18-31: Reserved */
#if defined(KINETIS_SIM_HAS_SOPT5_LPUART1RXSRC)
                                                        /* Bits 8-15, 20-31: Reserved */
#  define SIM_SOPT5_LPUART1RXSRC_SHIFT        (18)      /* Bit 18:  LPUART1 receive data source select */
#  define SIM_SOPT5_LPUART1RXSRC_MASK         (3 << SIM_SOPT5_LPUART1RXSRC_SHIFT)
#    define SIM_SOPT5_LPUART1RXSRC_TX         (0 << SIM_SOPT5_LPUART1RXSRC_SHIFT) /* LPUART1_RX pin */
#    define SIM_SOPT5_LPUART1RXSRC_TXTMP1CH0  (1 << SIM_SOPT5_LPUART1RXSRC_SHIFT) /* CMP0 output */
#    define SIM_SOPT5_LPUART1RXSRC_TXTMP2CH0  (2 << SIM_SOPT5_LPUART1RXSRC_SHIFT) /* CMP1 output */
#endif

/* System Options Register 6 */

#if defined(KINETIS_SIM_HAS_SOPT6)
                                                        /* Bits 0-23: Reserved */
#  if defined(KINETIS_SIM_HAS_SOPT6_MCC)
                                                        /* Bits 16-23: Reserved */
#    define SIM_SOPT6_MCC_SHIFT               (0)       /* Bits 0-15: NFC hold cycle in case FlexBus request while NFC is granted */
#    define SIM_SOPT6_MCC_MASK                (0xffff << SIM_SOPT6_MCC_SHIFT)
#    define SIM_SOPT6_MCC(n)                  (((n) & 0xffff) << SIM_SOPT6_MCC_SHIFT)
#  endif
                                                        /* Bits 16-23: Reserved */
#  if defined(KINETIS_SIM_HAS_SOPT6_PCR)
                                                        /* Bits 20-23: Reserved */
#    define SIM_SOPT6_PCR_SHIFT               (16)      /* Bits 16-19: FlexBus hold cycles before FlexBus can release bus to NFC or to IDLE */
#    define SIM_SOPT6_PCR_MASK                (7 << SIM_SOPT6_PCR_SHIFT)
#    define SIM_SOPT6_PCR(n)                  (((n) & 7) << SIM_SOPT6_PCR_SHIFT)
#  endif
#  if defined(KINETIS_SIM_HAS_SOPT6_RSTFLTSEL)
#    define SIM_SOPT6_RSTFLTSEL_SHIFT         (24)      /* Bits 24-28: Reset pin filter select */
#    define SIM_SOPT6_RSTFLTSEL_MASK          (31 << SIM_SOPT6_RSTFLTSEL_SHIFT)
#       define SIM_SOPT6_RSTFLTSEL(n)         ((uint32_t)((n)-1) << SIM_SOPT6_RSTFLTSEL_SHIFT) /* n=1..32 */
#  endif
#  if defined(KINETIS_SIM_HAS_SOPT6_RSTFLTEN)
#    define SIM_SOPT6_RSTFLTEN_SHIFT          (29)      /* Bits 29-31: Reset pin filter enable */
#    define SIM_SOPT6_RSTFLTEN_MASK           (7 << SIM_SOPT6_RSTFLTEN_SHIFT)
#    define SIM_SOPT6_RSTFLTEN_DISABLED       (0 << SIM_SOPT6_RSTFLTEN_SHIFT) /* All filtering disabled */
#      define SIM_SOPT6_RSTFLTEN_BUSCLK1      (1 << SIM_SOPT6_RSTFLTEN_SHIFT) /* Bus clock filter enabled (normal); LPO clock filter enabled (stop) */
#      define SIM_SOPT6_RSTFLTEN_LPO1         (2 << SIM_SOPT6_RSTFLTEN_SHIFT) /* LPO clock filter enabled */
#      define SIM_SOPT6_RSTFLTEN_BUSCLK2      (3 << SIM_SOPT6_RSTFLTEN_SHIFT) /* Bus clock filter enabled (normal); All filtering disabled (stop) */
#      define SIM_SOPT6_RSTFLTEN_LPO2         (4 << SIM_SOPT6_RSTFLTEN_SHIFT) /* PO clock filter enabled (normal); All filtering disabled (stop) */
#  endif
#endif

/* System Options Register 7 */

#if defined(KINETIS_SIM_HAS_SOPT7_ADC0TRGSEL)
#  define SIM_SOPT7_ADC0TRGSEL_SHIFT          (0)       /* Bits 0-3: ADC0 trigger select */
#  define SIM_SOPT7_ADC0TRGSEL_MASK           (15 << SIM_SOPT7_ADC0TRGSEL_SHIFT)
#    define SIM_SOPT7_ADC0TRGSEL_PDB          (0 << SIM_SOPT7_ADC0TRGSEL_SHIFT)  /* PDB external trigger (PDB0_EXTRG) */
#    define SIM_SOPT7_ADC0TRGSEL_CMP0         (1 << SIM_SOPT7_ADC0TRGSEL_SHIFT)  /* High speed comparator 0 output */
#    define SIM_SOPT7_ADC0TRGSEL_CMP1         (2 << SIM_SOPT7_ADC0TRGSEL_SHIFT)  /* High speed comparator 1 output */
#    define SIM_SOPT7_ADC0TRGSEL_CMP2         (3 << SIM_SOPT7_ADC0TRGSEL_SHIFT)  /* High speed comparator 2 output */
#    define SIM_SOPT7_ADC0TRGSEL_PIT0         (4 << SIM_SOPT7_ADC0TRGSEL_SHIFT)  /* PIT trigger 0 */
#    define SIM_SOPT7_ADC0TRGSEL_PIT1         (5 << SIM_SOPT7_ADC0TRGSEL_SHIFT)  /* PIT trigger 1 */
#    define SIM_SOPT7_ADC0TRGSEL_PIT2         (6 << SIM_SOPT7_ADC0TRGSEL_SHIFT)  /* PIT trigger 2 */
#    define SIM_SOPT7_ADC0TRGSEL_PIT3         (7 << SIM_SOPT7_ADC0TRGSEL_SHIFT)  /* PIT trigger 3 */
#    define SIM_SOPT7_ADC0TRGSEL_FTM0         (8 << SIM_SOPT7_ADC0TRGSEL_SHIFT)  /* FTM0 trigger */
#    define SIM_SOPT7_ADC0TRGSEL_FTM1         (9 << SIM_SOPT7_ADC0TRGSEL_SHIFT)  /* FTM1 trigger */
#    define SIM_SOPT7_ADC0TRGSEL_FTM2         (10 << SIM_SOPT7_ADC0TRGSEL_SHIFT) /* FTM2 trigger */
#    if KINETIS_SIM_HAS_SOPT7_ADC0TRGSEL > 10 && defined(KINETIS_SIM_HAS_SOPT4_FTM3CH0SRC)
#        define SIM_SOPT7_ADC0TRGSEL_FTM3     (11 << SIM_SOPT7_ADC0TRGSEL_SHIFT) /* FTM3 trigger */
#    endif
#    if KINETIS_SIM_HAS_SOPT7_ADC0TRGSEL > 11
#      define SIM_SOPT7_ADC0TRGSEL_ALARM      (12 << SIM_SOPT7_ADC0TRGSEL_SHIFT) /* RTC alarm */
#    endif
#    if KINETIS_SIM_HAS_SOPT7_ADC0TRGSEL > 12
#      define SIM_SOPT7_ADC0TRGSEL_SECS       (13 << SIM_SOPT7_ADC0TRGSEL_SHIFT) /* RTC seconds */
#    endif
#    if KINETIS_SIM_HAS_SOPT7_ADC0TRGSEL > 13
#      define SIM_SOPT7_ADC0TRGSEL_LPTMR      (14 << SIM_SOPT7_ADC0TRGSEL_SHIFT) /* Low-power timer trigger */
#    endif
#    if KINETIS_SIM_HAS_SOPT7_ADC0TRGSEL > 14
#        define SIM_SOPT7_ADC0TRGSEL_TPM1CH0  (15 << SIM_SOPT7_ADC0TRGSEL_SHIFT) /* TPM1 channel 0 (A pretrigger) and channel 1 (B pretrigger) */
#    endif
#endif
#if defined(KINETIS_SIM_HAS_SOPT7_ADC0PRETRGSEL)
#  define SIM_SOPT7_ADC0PRETRGSEL             (1 << 4)  /* Bit 4:  ADC0 pretrigger select */
#endif
                                                        /* Bits 5-6: Reserved */
#if defined(KINETIS_SIM_SOPT7_ADC0ALTTRGEN)
#  define SIM_SOPT7_ADC0ALTTRGEN              (1 << 7)  /* Bit 7:  ADC0 alternate trigger enable */
#endif

#if defined(KINETIS_SIM_HAS_SOPT7_ADC1TRGSEL)
#  define SIM_SOPT7_ADC1TRGSEL_SHIFT          (8)       /* Bits 8-11: ADC1 trigger select */
#  define SIM_SOPT7_ADC1TRGSEL_MASK           (15 << SIM_SOPT7_ADC1TRGSEL_SHIFT)
#    define SIM_SOPT7_ADC1TRGSEL_PDB          (0 << SIM_SOPT7_ADC1TRGSEL_SHIFT)  /* PDB external trigger (PDB0_EXTRG) */
#    define SIM_SOPT7_ADC1TRGSEL_CMP0         (1 << SIM_SOPT7_ADC1TRGSEL_SHIFT)  /* High speed comparator 0 output */
#    define SIM_SOPT7_ADC1TRGSEL_CMP1         (2 << SIM_SOPT7_ADC1TRGSEL_SHIFT)  /* High speed comparator 1 output */
#    define SIM_SOPT7_ADC1TRGSEL_CMP2         (3 << SIM_SOPT7_ADC1TRGSEL_SHIFT)  /* High speed comparator 2 output */
#    define SIM_SOPT7_ADC1TRGSEL_PIT0         (4 << SIM_SOPT7_ADC1TRGSEL_SHIFT)  /* PIT trigger 0 */
#    define SIM_SOPT7_ADC1TRGSEL_PIT1         (5 << SIM_SOPT7_ADC1TRGSEL_SHIFT)  /* PIT trigger 1 */
#    define SIM_SOPT7_ADC1TRGSEL_PIT2         (6 << SIM_SOPT7_ADC1TRGSEL_SHIFT)  /* PIT trigger 2 */
#    define SIM_SOPT7_ADC1TRGSEL_PIT3         (7 << SIM_SOPT7_ADC1TRGSEL_SHIFT)  /* PIT trigger 3 */
#    define SIM_SOPT7_ADC1TRGSEL_FTM0         (8 << SIM_SOPT7_ADC1TRGSEL_SHIFT)  /* FTM0 trigger */
#    define SIM_SOPT7_ADC1TRGSEL_FTM1         (9 << SIM_SOPT7_ADC1TRGSEL_SHIFT)  /* FTM1 trigger */
#    define SIM_SOPT7_ADC1TRGSEL_FTM2         (10 << SIM_SOPT7_ADC1TRGSEL_SHIFT) /* FTM2 trigger */
#    define SIM_SOPT7_ADC1TRGSEL_ALARM        (12 << SIM_SOPT7_ADC1TRGSEL_SHIFT) /* RTC alarm */
#    if KINETIS_SIM_HAS_SOPT7_ADC1TRGSEL > 10 && defined(KINETIS_SIM_HAS_SOPT4_FTM3CH0SRC)
#        define SIM_SOPT7_ADC1TRGSEL_FTM3     (11 << SIM_SOPT7_ADC1TRGSEL_SHIFT) /* FTM3 trigger */
#    endif
#    if KINETIS_SIM_HAS_SOPT7_ADC1TRGSEL > 11
#      define SIM_SOPT7_ADC1TRGSEL_ALARM      (12 << SIM_SOPT7_ADC1TRGSEL_SHIFT) /* RTC alarm */
#    endif
#    if KINETIS_SIM_HAS_SOPT7_ADC1TRGSEL > 12
#      define SIM_SOPT7_ADC1TRGSEL_SECS       (13 << SIM_SOPT7_ADC1TRGSEL_SHIFT) /* RTC seconds */
#    endif
#    if KINETIS_SIM_HAS_SOPT7_ADC1TRGSEL > 13
#      define SIM_SOPT7_ADC1TRGSEL_LPTMR      (14 << SIM_SOPT7_ADC1TRGSEL_SHIFT) /* Low-power timer trigger */
#    endif
#    if KINETIS_SIM_HAS_SOPT7_ADC1TRGSEL > 14
#        define SIM_SOPT7_ADC1TRGSEL_TPM2CH0  (15 << SIM_SOPT7_ADC1TRGSEL_SHIFT) /* TPM2 channel 0 (A pretrigger) and channel 1 (B pretrigger) */
#    endif
#endif
#if defined(KINETIS_SIM_HAS_SOPT7_ADC1PRETRGSEL)
#  define SIM_SOPT7_ADC1PRETRGSEL             (1 << 12) /* Bit 12: ADC1 pre-trigger select */
#endif
                                                        /* Bits 13-14: Reserved */
#if defined(KINETIS_SIM_SOPT7_ADC1ALTTRGEN)
# define SIM_SOPT7_ADC1ALTTRGEN               (1 << 15) /* Bit 15: ADC1 alternate trigger enable */
#endif
                                                        /* Bits 16-31: Reserved */
#if defined(KINETIS_SIM_HAS_SOPT7_ADC2TRGSEL)
#  define SIM_SOPT7_ADC2TRGSEL_SHIFT          (16)      /* Bits 16-19: ADC2 trigger select */
#  define SIM_SOPT7_ADC2TRGSEL_MASK           (15 << SIM_SOPT7_ADC2TRGSEL_SHIFT)
#    define SIM_SOPT7_ADC2TRGSEL_PDB          (0 << SIM_SOPT7_ADC2TRGSEL_SHIFT)  /* PDB external trigger (PDB0_EXTRG) */
#    define SIM_SOPT7_ADC2TRGSEL_CMP0         (1 << SIM_SOPT7_ADC2TRGSEL_SHIFT)  /* High speed comparator 0 output */
#    define SIM_SOPT7_ADC2TRGSEL_CMP1         (2 << SIM_SOPT7_ADC2TRGSEL_SHIFT)  /* High speed comparator 1 output */
#    define SIM_SOPT7_ADC2TRGSEL_CMP2         (3 << SIM_SOPT7_ADC2TRGSEL_SHIFT)  /* High speed comparator 2 output */
#    define SIM_SOPT7_ADC2TRGSEL_PIT0         (4 << SIM_SOPT7_ADC2TRGSEL_SHIFT)  /* PIT trigger 0 */
#    define SIM_SOPT7_ADC2TRGSEL_PIT1         (5 << SIM_SOPT7_ADC2TRGSEL_SHIFT)  /* PIT trigger 1 */
#    define SIM_SOPT7_ADC2TRGSEL_PIT2         (6 << SIM_SOPT7_ADC2TRGSEL_SHIFT)  /* PIT trigger 2 */
#    define SIM_SOPT7_ADC2TRGSEL_PIT3         (7 << SIM_SOPT7_ADC2TRGSEL_SHIFT)  /* PIT trigger 3 */
#    define SIM_SOPT7_ADC2TRGSEL_FTM0         (8 << SIM_SOPT7_ADC2TRGSEL_SHIFT)  /* FTM0 trigger */
#    define SIM_SOPT7_ADC2TRGSEL_FTM1         (9 << SIM_SOPT7_ADC2TRGSEL_SHIFT)  /* FTM1 trigger */
#    define SIM_SOPT7_ADC2TRGSEL_FTM2         (10 << SIM_SOPT7_ADC2TRGSEL_SHIFT) /* FTM2 trigger */
#    if KINETIS_SIM_HAS_SOPT7_ADC2TRGSEL > 10 && defined(KINETIS_SIM_HAS_SOPT4_FTM3CH0SRC)
#        define SIM_SOPT7_ADC2TRGSEL_FTM3     (11 << SIM_SOPT7_ADC2TRGSEL_SHIFT) /* FTM3 trigger */
#    endif
#    if KINETIS_SIM_HAS_SOPT7_ADC2TRGSEL > 11
#      define SIM_SOPT7_ADC2TRGSEL_ALARM      (12 << SIM_SOPT7_ADC2TRGSEL_SHIFT) /* RTC alarm */
#    endif
#    if KINETIS_SIM_HAS_SOPT7_ADC2TRGSEL > 12
#      define SIM_SOPT7_ADC2TRGSEL_SECS       (13 << SIM_SOPT7_ADC2TRGSEL_SHIFT) /* RTC seconds */
#    endif
#    if KINETIS_SIM_HAS_SOPT7_ADC2TRGSEL > 13
#      define SIM_SOPT7_ADC2TRGSEL_LPTMR      (14 << SIM_SOPT7_ADC2TRGSEL_SHIFT) /* Low-power timer trigger */
#    endif
#    if KINETIS_SIM_HAS_SOPT7_ADC2TRGSEL > 14
#        define SIM_SOPT7_ADC2TRGSEL_CMP3    (15 << SIM_SOPT7_ADC2TRGSEL_SHIFT) /* High speed comparator 3 asynchronous interrupt */
#    endif
#endif
#if defined(KINETIS_SIM_HAS_SOPT7_ADC2PRETRGSEL)
#  define SIM_SOPT7_ADC2PRETRGSEL             (1 << 20) /* Bit 20:  ADC2 pretrigger select */
#endif
                                                        /* Bits 21-22: Reserved */
#if defined(KINETIS_SIM_SOPT7_ADC2ALTTRGEN)
#  define SIM_SOPT7_ADC2ALTTRGEN              (1 << 23) /* Bit 23:  ADC2 alternate trigger enable */
#endif
                                                        /* Bits 23-27: Reserved */
#if defined(KINETIS_SIM_HAS_SOPT7_ADC3TRGSEL)
#  define SIM_SOPT7_ADC3TRGSEL_SHIFT          (24)      /* Bits 24-27: ADC3 trigger select */
#  define SIM_SOPT7_ADC3TRGSEL_MASK           (15 << SIM_SOPT7_ADC3TRGSEL_SHIFT)
#    define SIM_SOPT7_ADC3TRGSEL_PDB          (0 << SIM_SOPT7_ADC3TRGSEL_SHIFT)  /* PDB external trigger (PDB0_EXTRG) */
#    define SIM_SOPT7_ADC3TRGSEL_CMP0         (1 << SIM_SOPT7_ADC3TRGSEL_SHIFT)  /* High speed comparator 0 output */
#    define SIM_SOPT7_ADC3TRGSEL_CMP1         (2 << SIM_SOPT7_ADC3TRGSEL_SHIFT)  /* High speed comparator 1 output */
#    define SIM_SOPT7_ADC3TRGSEL_CMP2         (3 << SIM_SOPT7_ADC3TRGSEL_SHIFT)  /* High speed comparator 2 output */
#    define SIM_SOPT7_ADC3TRGSEL_PIT0         (4 << SIM_SOPT7_ADC3TRGSEL_SHIFT)  /* PIT trigger 0 */
#    define SIM_SOPT7_ADC3TRGSEL_PIT1         (5 << SIM_SOPT7_ADC3TRGSEL_SHIFT)  /* PIT trigger 1 */
#    define SIM_SOPT7_ADC3TRGSEL_PIT2         (6 << SIM_SOPT7_ADC3TRGSEL_SHIFT)  /* PIT trigger 2 */
#    define SIM_SOPT7_ADC3TRGSEL_PIT3         (7 << SIM_SOPT7_ADC3TRGSEL_SHIFT)  /* PIT trigger 3 */
#    define SIM_SOPT7_ADC3TRGSEL_FTM0         (8 << SIM_SOPT7_ADC3TRGSEL_SHIFT)  /* FTM0 trigger */
#    define SIM_SOPT7_ADC3TRGSEL_FTM1         (9 << SIM_SOPT7_ADC3TRGSEL_SHIFT)  /* FTM1 trigger */
#    define SIM_SOPT7_ADC3TRGSEL_FTM2         (10 << SIM_SOPT7_ADC3TRGSEL_SHIFT) /* FTM2 trigger */
#    if KINETIS_SIM_HAS_SOPT7_ADC3TRGSEL > 10 && defined(KINETIS_SIM_HAS_SOPT4_FTM3CH0SRC)
#        define SIM_SOPT7_ADC3TRGSEL_FTM3     (11 << SIM_SOPT7_ADC3TRGSEL_SHIFT) /* FTM3 trigger */
#    endif
#    if KINETIS_SIM_HAS_SOPT7_ADC3TRGSEL > 11
#      define SIM_SOPT7_ADC3TRGSEL_ALARM      (12 << SIM_SOPT7_ADC3TRGSEL_SHIFT) /* RTC alarm */
#    endif
#    if KINETIS_SIM_HAS_SOPT7_ADC3TRGSEL > 12
#      define SIM_SOPT7_ADC3TRGSEL_SECS       (13 << SIM_SOPT7_ADC3TRGSEL_SHIFT) /* RTC seconds */
#    endif
#    if KINETIS_SIM_HAS_SOPT7_ADC3TRGSEL > 13
#      define SIM_SOPT7_ADC3TRGSEL_LPTMR      (14 << SIM_SOPT7_ADC3TRGSEL_SHIFT) /* Low-power timer trigger */
#    endif
#    if KINETIS_SIM_HAS_SOPT7_ADC3TRGSEL > 14
#        define SIM_SOPT7_ADC3TRGSEL_CMP3    (15 << SIM_SOPT7_ADC3TRGSEL_SHIFT) /* High speed comparator 3 asynchronous interrupt */
#    endif
#endif
#if defined(KINETIS_SIM_HAS_SOPT7_ADC3PRETRGSEL)
#  define SIM_SOPT7_ADC3PRETRGSEL             (1 << 28) /* Bit 28:  ADC3 pretrigger select */
#endif
                                                        /* Bits 29-30: Reserved */
#if defined(KINETIS_SIM_SOPT7_ADC3ALTTRGEN)
#  define SIM_SOPT7_ADC3ALTTRGEN              (1 << 31) /* Bit 31:  ADC3 alternate trigger enable */
#endif

/* System Options Register 8 */

#if defined(KINETIS_SIM_HAS_SOPT8)
#  if defined(KINETIS_SIM_HAS_SOPT8_FTM0SYNCBIT)
#    define SIM_SOPT8_FTM0SYNCBIT             (1 << 0)  /* Bit 0:  FTM0 Hardware Trigger 0 Software Synchronization */
#  endif
#  if defined(KINETIS_SIM_HAS_SOPT8_FTM1SYNCBIT)
#    define SIM_SOPT8_FTM1SYNCBIT             (1 << 1)  /* Bit 1:  FTM1 Hardware Trigger 0 Software Synchronization */
#  endif
#  if defined(KINETIS_SIM_HAS_SOPT8_FTM2SYNCBIT)
#    define SIM_SOPT8_FTM2SYNCBIT             (1 << 2)  /* Bit 2:  FTM2 Hardware Trigger 0 Software Synchronization */
#  endif
#  if defined(KINETIS_SIM_HAS_SOPT8_FTM3SYNCBIT)
#    define SIM_SOPT8_FTM3SYNCBIT             (1 << 3)  /* Bit 3:  FTM3 Hardware Trigger 0 Software Synchronization */
#  endif
                                                        /* Bits 4-15: Reserved */
#  if defined(KINETIS_SIM_HAS_SOPT8_FTM0OCH0SRC)
#    define SIM_SOPT8_FTM0OCH0SRC             (1 << 16) /* Bit 16:  FTM0 channel 0 output source */
#  endif
#  if defined(KINETIS_SIM_HAS_SOPT8_FTM0OCH1SRC)
#    define SIM_SOPT8_FTM0OCH1SRC             (1 << 17) /* Bit 17:  FTM0 channel 1 output source */
#  endif
#  if defined(KINETIS_SIM_HAS_SOPT8_FTM0OCH2SRC)
#    define SIM_SOPT8_FTM0OCH2SRC             (1 << 18) /* Bit 18:  FTM0 channel 2 output source */
#  endif
#  if defined(KINETIS_SIM_HAS_SOPT8_FTM0OCH3SRC)
#    define SIM_SOPT8_FTM0OCH3SRC             (1 << 19) /* Bit 19:  FTM0 channel 3 output source */
#  endif
#  if defined(KINETIS_SIM_HAS_SOPT8_FTM0OCH4SRC)
#    define SIM_SOPT8_FTM0OCH4SRC             (1 << 20) /* Bit 20:  FTM0 channel 4 output source */
#  endif
#  if defined(KINETIS_SIM_HAS_SOPT8_FTM0OCH5SRC)
#    define SIM_SOPT8_FTM0OCH5SRC             (1 << 21) /* Bit 21:  FTM0 channel 5 output source */
#  endif
#  if defined(KINETIS_SIM_HAS_SOPT8_FTM0OCH6SRC)
#    define SIM_SOPT8_FTM0OCH6SRC             (1 << 22) /* Bit 22:  FTM0 channel 6 output source */
#  endif
#  if defined(KINETIS_SIM_HAS_SOPT8_FTM0OCH7SRC)
#    define SIM_SOPT8_FTM0OCH7SRC             (1 << 23) /* Bit 23:  FTM0 channel 7 output source */
#  endif
#  if defined(KINETIS_SIM_HAS_SOPT8_FTM3OCH0SRC)
#    define SIM_SOPT8_FTM3OCH0SRC             (1 << 24) /* Bit 24:  FTM3 channel 0 output source */
#  endif
#  if defined(KINETIS_SIM_HAS_SOPT8_FTM3OCH1SRC)
#    define SIM_SOPT8_FTM3OCH1SRC             (1 << 25) /* Bit 25:  FTM3 channel 1 output source */
#  endif
#  if defined(KINETIS_SIM_HAS_SOPT8_FTM3OCH2SRC)
#    define SIM_SOPT8_FTM3OCH2SRC             (1 << 26) /* Bit 26:  FTM3 channel 2 output source */
#  endif
#  if defined(KINETIS_SIM_HAS_SOPT8_FTM3OCH3SRC)
#    define SIM_SOPT8_FTM3OCH3SRC             (1 << 27) /* Bit 27:  FTM3 channel 3 output source */
#  endif
#  if defined(KINETIS_SIM_HAS_SOPT8_FTM3OCH4SRC)
#    define SIM_SOPT8_FTM3OCH4SRC             (1 << 28) /* Bit 28:  FTM3 channel 4 output source */
#  endif
#  if defined(KINETIS_SIM_HAS_SOPT8_FTM3OCH5SRC)
#    define SIM_SOPT8_FTM3OCH5SRC             (1 << 29) /* Bit 29:  FTM3 channel 5 output source */
#  endif
#  if defined(KINETIS_SIM_HAS_SOPT8_FTM3OCH6SRC)
#    define SIM_SOPT8_FTM3OCH6SRC             (1 << 30) /* Bit 30:  FTM3 channel 6 output source */
#  endif
#  if defined(KINETIS_SIM_HAS_SOPT8_FTM3OCH7SRC)
#    define SIM_SOPT8_FTM3OCH7SRC             (1 << 31) /* Bit 31:  FTM3 channel 7 output source */
#  endif
#endif

/* System Options Register 9 */

#if defined(KINETIS_SIM_HAS_SOPT9)
                                                        /* Bits 0-17: Reserved */
#  if defined(KINETIS_SIM_HAS_SOPT9_TPM1CH0SRC)
#    define SIM_SOPT9_TPM1CH0SRC_SHIFT        (18)      /* Bits 18-19:  TPM1 channel 0 input capture source select */
#    define SIM_SOPT9_TPM1CH0SRC_MASK         (3 << SIM_SOPT9_TPM1CH0SRC_SHIFT)
#      define SIM_SOPT9_TPM1CH0SRC_TMP1CH0    (0 << SIM_SOPT9_TPM1CH0SRC_SHIFT)
#      define SIM_SOPT9_TPM1CH0SRC_CMP0       (1 << SIM_SOPT9_TPM1CH0SRC_SHIFT)
#      define SIM_SOPT9_TPM1CH0SRC_CMP1       (2 << SIM_SOPT9_TPM1CH0SRC_SHIFT)
#  endif
#  if defined(KINETIS_SIM_HAS_SOPT9_TPM2CH0SRC)
#    define SIM_SOPT9_TPM2CH0SRC_SHIFT        (20)      /* Bits 20-21  TPM2 channel 0 input capture source select */
#    define SIM_SOPT9_TPM2CH0SRC_MASK         (3 << SIM_SOPT9_TPM2CH0SRC_SHIFT)
#      define SIM_SOPT9_TPM2CH0SRC_TMP1CH0    (0 << SIM_SOPT9_TPM2CH0SRC_SHIFT)
#      define SIM_SOPT9_TPM2CH0SRC_CMP0       (1 << SIM_SOPT9_TPM2CH0SRC_SHIFT)
#      define SIM_SOPT9_TPM2CH0SRC_CMP1       (2 << SIM_SOPT9_TPM2CH0SRC_SHIFT)
#  endif
                                                        /* Bits 22-24: Reserved */
#  if defined(KINETIS_SIM_HAS_SOPT9_TPM1CLKSEL)
#    define SIM_SOPT9_TPM1CLKSEL              (1 << 25) /* Bit 25:  TPM1 External Clock Pin Select */
#  endif
#  if defined(KINETIS_SIM_HAS_SOPT9_TPM2CLKSEL)
#    define SIM_SOPT9_TPM2CLKSEL              (1 << 26) /* Bit 26:  TPM2 External Clock Pin Select */
#  endif
                                                        /* Bits 27-31: Reserved */
#endif

/* System Device Identification Register */

#define SIM_SDID_PINID_SHIFT                  (0)       /* Bits 0-3: Pincount identification */
#define SIM_SDID_PINID_MASK                   (15 << SIM_SDID_PINID_SHIFT)
#  define SIM_SDID_PINID_32PIN                (2 << SIM_SDID_PINID_SHIFT)  /* 32-pin */
#  define SIM_SDID_PINID_48PIN                (4 << SIM_SDID_PINID_SHIFT)  /* 48-pin */
#  define SIM_SDID_PINID_64PIN                (5 << SIM_SDID_PINID_SHIFT)  /* 64-pin */
#  define SIM_SDID_PINID_80PIN                (6 << SIM_SDID_PINID_SHIFT)  /* 80-pin */
#  define SIM_SDID_PINID_81PIN                (7 << SIM_SDID_PINID_SHIFT)  /* 81-pin */
#  define SIM_SDID_PINID_100PIN               (8 << SIM_SDID_PINID_SHIFT)  /* 100-pin */
#  define SIM_SDID_PINID_121PIN               (9 << SIM_SDID_PINID_SHIFT)  /* 121-pin */
#  define SIM_SDID_PINID_144PIN               (10 << SIM_SDID_PINID_SHIFT) /* 144-pin */
#  define SIM_SDID_PINID_196PIN               (12 << SIM_SDID_PINID_SHIFT) /* 196-pin */
#  define SIM_SDID_PINID_256PIN               (14 << SIM_SDID_PINID_SHIFT) /* 256-pin */
#if defined(KINETIS_SIM_HAS_SDID_FAMID)
#  if !defined(KINETIS_SIM_HAS_SDID_FAMILYID)
#    define SIM_SDID_FAMID_SHIFT              (4)       /* Bits 4-6: Kinetis family identification */
#    define SIM_SDID_FAMID_MASK               (7 << SIM_SDID_FAMID_SHIFT)
#      define SIM_SDID_FAMID_K10              (0 << SIM_SDID_FAMID_SHIFT) /* K10 */
#      define SIM_SDID_FAMID_K20              (1 << SIM_SDID_FAMID_SHIFT) /* K20 */
#      define SIM_SDID_FAMID_K30              (2 << SIM_SDID_FAMID_SHIFT) /* K30 */
#      define SIM_SDID_FAMID_K40              (3 << SIM_SDID_FAMID_SHIFT) /* K40 */
#      define SIM_SDID_FAMID_K60              (4 << SIM_SDID_FAMID_SHIFT) /* K60 */
#      define SIM_SDID_FAMID_K70              (5 << SIM_SDID_FAMID_SHIFT) /* K70 */
#      define SIM_SDID_FAMID_K50              (6 << SIM_SDID_FAMID_SHIFT) /* K50 and K52 */
#      define SIM_SDID_FAMID_K51              (7 << SIM_SDID_FAMID_SHIFT) /* K51 and K53 */
#  else
#      define SIM_SDID_FAMID_K1X              (0 << SIM_SDID_FAMID_SHIFT) /* K1X */
#      define SIM_SDID_FAMID_K2X              (1 << SIM_SDID_FAMID_SHIFT) /* K2X */
#      define SIM_SDID_FAMID_K3X              (2 << SIM_SDID_FAMID_SHIFT) /* K3X */
#      define SIM_SDID_FAMID_K4X              (3 << SIM_SDID_FAMID_SHIFT) /* K4X */
#      define SIM_SDID_FAMID_K6X              (4 << SIM_SDID_FAMID_SHIFT) /* K6X */
#      define SIM_SDID_FAMID_K7X              (5 << SIM_SDID_FAMID_SHIFT) /* K7X */
#  endif
#endif
                                                        /* Bits 7-11: Reserved */
#if defined(KINETIS_SIM_HAS_SDID_DIEID)
#  define SIM_SDID_DIEID_SHIFT                (7)       /* Bits 7-11:  Device Die ID */
#  define SIM_SDID_DIEID_MASK                 (31 < SIM_SDID_DIEID_SHIFT)
#endif

#define SIM_SDID_REVID_SHIFT                  (12)      /* Bits 12-15: Device revision number */
#define SIM_SDID_REVID_MASK                   (15 << SIM_SDID_REVID_SHIFT)
                                                        /* Bits 16-31: Reserved */
#if defined(KINETIS_SIM_HAS_SDID_SRAMSIZE)
#  define SIM_SDID_SRAMSIZE_SHIFT             (16)      /* Bits 16-19: SRAM Size */
#  define SIM_SDID_SRAMSIZE_MASK              (15 < SIM_SDID_SRAMSIZE_SHIFT)
#endif
#if defined(KINETIS_SIM_HAS_SDID_SERIESID)
#  define SIM_SDID_SERIESID_SHIFT             (20)      /* Bits 20-23: Kinetis Series ID */
#  define SIM_SDID_SERIESID_MASK              (15 << SIM_SDID_SERIESID_SHIFT)
#  define SIM_SDID_SERIESID_K                 (0 << SIM_SDID_SERIESID_SHIFT) /* Kinetis K series */
#  define SIM_SDID_SERIESID_L                 (1 << SIM_SDID_SERIESID_SHIFT) /* Kinetis L series */
#  define SIM_SDID_SERIESID_W                 (5 << SIM_SDID_SERIESID_SHIFT) /* Kinetis W series */
#  define SIM_SDID_SERIESID_V                 (6 << SIM_SDID_SERIESID_SHIFT) /* Kinetis V series */
#endif
#if defined(KINETIS_SIM_HAS_SDID_SUBFAMID)
#  define SIM_SDID_SUBFAMID_SHIFT             (24)      /* Bits 24-27: Kinetis Sub-Family ID */
#  define SIM_SDID_SUBFAMID_MASK              (15 << SIM_SDID_SUBFAMID_SHIFT)
#  define SIM_SDID_SUBFAMID_KX0               (0 << SIM_SDID_SUBFAMID_SHIFT) /* Kx0 Subfamily */
#  define SIM_SDID_SUBFAMID_KX1               (1 << SIM_SDID_SUBFAMID_SHIFT) /* Kx1 Subfamily (tamper detect) */
#  define SIM_SDID_SUBFAMID_KX2               (2 << SIM_SDID_SUBFAMID_SHIFT) /* Kx2 Subfamily */
#  define SIM_SDID_SUBFAMID_KX3               (3 << SIM_SDID_SUBFAMID_SHIFT) /* Kx3 Subfamily (tamper detect) */
#  define SIM_SDID_SUBFAMID_KX4               (4 << SIM_SDID_SUBFAMID_SHIFT) /* Kx4 Subfamily */
#  define SIM_SDID_SUBFAMID_KX5               (5 << SIM_SDID_SUBFAMID_SHIFT) /* Kx5 Subfamily (tamper detect) */
#  define SIM_SDID_SUBFAMID_KX6               (6 << SIM_SDID_SUBFAMID_SHIFT) /* Kx6 Subfamily */
#endif
#if defined(KINETIS_SIM_HAS_SDID_FAMILYID)
#  define SIM_SDID_FAMILYID_SHIFT             (28)      /* Bits 28-31: Kinetis Family ID */
#  define SIM_SDID_FAMILYID_MASK              (15 << SIM_SDID_FAMILYID_SHIFT)
#    define SIM_SDID_FAMILYID_K0X             (0 << SIM_SDID_FAMILYID_SHIFT) /* K0x Family */
#    define SIM_SDID_FAMILYID_K1X             (1 << SIM_SDID_FAMILYID_SHIFT) /* K1x Family */
#    define SIM_SDID_FAMILYID_K2X             (2 << SIM_SDID_FAMILYID_SHIFT) /* K2x Family */
#    define SIM_SDID_FAMILYID_K3X             (3 << SIM_SDID_FAMILYID_SHIFT) /* K3x Family */
#    define SIM_SDID_FAMILYID_K4X             (4 << SIM_SDID_FAMILYID_SHIFT) /* K4x Family */
#    define SIM_SDID_FAMILYID_K6X             (6 << SIM_SDID_FAMILYID_SHIFT) /* K6x Family */
#    define SIM_SDID_FAMILYID_K7X             (7 << SIM_SDID_FAMILYID_SHIFT) /* K7x Family */
#    define SIM_SDID_FAMILYID_K8X             (8 << SIM_SDID_FAMILYID_SHIFT) /* K8x Family */
#endif


/* System Clock Gating Control Register 1 */

#if defined(KINETIS_SIM_HAS_SCGC1)
                                                        /* Bits 0-9: Reserved */
#  if defined(KINETIS_SIM_HAS_SCGC1_OSC1)
                                                        /* Bits 0-4: Reserved */
#    define SIM_SCGC1_OSC1                    (1 << 5)  /* OSC1 clock gate control */
#  endif
                                                        /* Bits 6-9: Reserved */
#  if defined(KINETIS_SIM_HAS_SCGC1_I2C2)
#    define SIM_SCGC1_I2C2                    (1 << 6)  /* Bit 6: I2C2 Clock Gate Control */
#  endif
                                                        /* Bits 7-9: Reserved */
#  if defined(KINETIS_SIM_HAS_SCGC1_I2C3)
#    define SIM_SCGC1_I2C3                    (1 << 7)  /* Bit 7: I2C3 Clock Gate Control */
#  endif
#  if defined(KINETIS_SIM_HAS_SCGC1_UART4)
#    define SIM_SCGC1_UART4                   (1 << 10) /* Bit 10: UART4 Clock Gate Control */
#  endif
#  if defined(KINETIS_SIM_HAS_SCGC1_UART5)
#    define SIM_SCGC1_UART5                   (1 << 11) /* Bit 11: UART5 Clock Gate Control */
#  endif
                                                        /* Bits 12-31: Reserved */
#endif

/* System Clock Gating Control Register 2 */

#if defined(KINETIS_SIM_HAS_SCGC2)
#  if defined(KINETIS_SIM_HAS_SCGC2_ENET) && defined(KINETIS_NENET) && KINETIS_NENET > 0
#    define SIM_SCGC2_ENET                    (1 << 0)  /* Bit 0:  ENET Clock Gate Control */
#  endif
#  if defined(KINETIS_SIM_HAS_SCGC2_LPUART0)
#    define SIM_SCGC2_LPUART0                 (1 << 4)  /* Bit 4:  LPUART0 Clock Gate Control */
#  endif
#  if defined(KINETIS_SIM_HAS_SCGC2_LPUART1)
#    define SIM_SCGC2_LPUART1                 (1 << 5)  /* Bit 5:  LPUART1 Clock Gate Control */
#  endif
#  if defined(KINETIS_SIM_HAS_SCGC2_LPUART2)
#    define SIM_SCGC2_LPUART2                 (1 << 6)  /* Bit 6:  LPUART2 Clock Gate Control */
#  endif
#  if defined(KINETIS_SIM_HAS_SCGC2_LPUART3)
#    define SIM_SCGC2_LPUART3                 (1 << 7)  /* Bit 7:  LPUART3 Clock Gate Control */
#  endif
#  if defined(KINETIS_SIM_HAS_SCGC2_TPM1)
#    define SIM_SCGC2_TPM1                    (1 << 9)  /* Bit 9:  TPM1 Clock Gate Control */
#  endif
#  if defined(KINETIS_SIM_HAS_SCGC2_TPM2)
#    define SIM_SCGC2_TPM2                    (1 << 10) /* Bit 10:  TPM2 Clock Gate Control */
#  endif
#  define SIM_SCGC2_DAC0                      (1 << 12) /* Bit 12: DAC0 Clock Gate Control */
#  if defined(KINETIS_SIM_HAS_SCGC2_DAC1)
#    define SIM_SCGC2_DAC1                    (1 << 13) /* Bit 13: DAC1 Clock Gate Control */
#  endif
                                                        /* Bits 14-21: Reserved */
#  if defined(KINETIS_SIM_HAS_SCGC2_LPUART4)
#    define SIM_SCGC2_LPUART4                 (1 << 22) /* Bit 22: LPUART4 Clock Gate Control */
#  endif
                                                        /* Bits 23-25: Reserved */
#  if defined(KINETIS_SIM_HAS_SCGC2_QSPI)
#    define SIM_SCGC2_QSPI                    (1 << 26) /* Bit 26: QSPI Clock Gate Control */
#  endif
                                                        /* Bits 27-30: Reserved */
#  if defined(KINETIS_SIM_HAS_SCGC2_FLEXIO)
#    define SIM_SCGC2_FLEXIO                  (1 << 31) /* Bit 31: FlexIO Clock Gate Control */
#  endif
#endif

/* System Clock Gating Control Register 3 */

#if defined(KINETIS_SIM_HAS_SCGC3)
#  if defined(KINETIS_SIM_HAS_SCGC3_RNGA)  && defined(KINETIS_NRNG) && KINETIS_NRNG > 0
#    define SIM_SCGC3_RNGA                    (1 << 0)  /* Bit 0:  TRNG/RNGA Clock Gate Control */
#  endif
#  if defined(KINETIS_SIM_HAS_SCGC3_USBHS)
#    define SIM_SCGC3_USBHS                   (1 << 1)  /* Bit 1:  USBHS Clock Gate Control */
#  endif
#  if defined(KINETIS_SIM_HAS_SCGC3_USBHSPHY)
#    define SIM_SCGC3_USBHSPHY                (1 << 2)  /* Bit 2:  USBHS PHY Clock Gate Control */
#  endif
#  if defined(KINETIS_SIM_HAS_SCGC3_USBHSDCD)
#    define SIM_SCGC3_USBHSDCD                (1 << 3)  /* Bit 3:  USBHS DCD Clock Gate Control */
#  endif
                                                        /* Bits 5-11: Reserved */
#  if defined(KINETIS_SIM_HAS_SCGC3_FLEXCAN1)
#    define SIM_SCGC3_FLEXCAN1                (1 << 4)  /* Bit 4:  FlexCAN1 Clock Gate Control */
#  endif
#  if defined(KINETIS_SIM_HAS_SCGC3_NFC)
#    define SIM_SCGC3_FLEXCAN1                (1 << 8)  /* Bit 8:  NFC Clock Gate Control */
#  endif
#  if defined(KINETIS_SIM_HAS_SCGC3_SPI2)
#    define SIM_SCGC3_SPI2                    (1 << 12) /* Bit 12: SPI2 Clock Gate Control */
#  endif
#  if defined(KINETIS_SIM_HAS_SCGC3_SPI3)
#    define SIM_SCGC3_SPI3                    (1 << 13) /* Bit 13: SPI3 Clock Gate Control */
#  endif
                                                        /* Bit 14: Reserved */
#  if defined(KINETIS_SIM_HAS_SCGC3_SAI1)
#    define SIM_SCGC3_SAI1                    (1 << 15) /* Bit 15:  I2S1/SAI1 clock Gate control */
#  endif
                                                        /* Bit 16: Reserved */
#  if defined(KINETIS_SIM_HAS_SCGC3_SDHC)
#    define SIM_SCGC3_SDHC                    (1 << 17) /* Bit 17:  SDHC Clock Gate Control */
#  endif
                                                        /* Bits 18-23: Reserved */
#  if defined(KINETIS_SIM_HAS_SCGC3_FTM2)
#    define SIM_SCGC3_FTM2                    (1 << 24) /* Bit 24:  FTM2 Clock Gate Control */
#  endif
#  if defined(KINETIS_SIM_HAS_SCGC3_FTM3) && defined(KINETIS_SIM_HAS_SOPT4_FTM3CH0SRC)
#    define SIM_SCGC3_FTM3                    (1 << 25) /* Bit 25:  RFTM3 Clock Gate Control */
#  endif
                                                        /* Bit 26: Reserved */
#  if defined(KINETIS_SIM_HAS_SCGC3_ADC1)
#    define SIM_SCGC3_ADC1                    (1 << 27) /* Bit 27: ADC1 Clock Gate Control */
#  endif
#  if defined(KINETIS_SIM_HAS_SCGC3_ADC3)
#    define SIM_SCGC3_ADC3                    (1 << 28) /* Bit 28:  ADC3 Clock Gate Control */
#  endif
                                                        /* Bit 29: Reserved */
#  if defined(KINETIS_SIM_HAS_SCGC3_SLCD) && defined(KINETIS_NSLCD) && KINETIS_NSLCD > 0
#    define SIM_SCGC3_SLCD                    (1 << 30) /* Bit 30: Segment LCD Clock Gate Control */
#  endif
                                                        /* Bit 31: Reserved */
#endif

/* System Clock Gating Control Register 4 */

                                                        /* Bit 0:  Reserved */
#define SIM_SCGC4_EWM                         (1 << 1)  /* Bit 1:  EWM Clock Gate Control */
#define SIM_SCGC4_CMT                         (1 << 2)  /* Bit 2:  CMT Clock Gate Control */
                                                        /* Bits 3-5: Reserved */
#define SIM_SCGC4_I2C0                        (1 << 6)  /* Bit 6:  I2C0 Clock Gate Control */
#define SIM_SCGC4_I2C1                        (1 << 7)  /* Bit 7:  I2C1 Clock Gate Control */
                                                        /* Bits 8-9: Reserved */
#if defined(KINETIS_SIM_HAS_SCGC4_UART0)
#  define SIM_SCGC4_UART0                     (1 << 10) /* Bit 10: UART0 Clock Gate Control */
#endif
#if defined(KINETIS_SIM_HAS_SCGC4_UART1)
#  define SIM_SCGC4_UART1                     (1 << 11) /* Bit 11: UART1 Clock Gate Control */
#endif
#if defined(KINETIS_SIM_HAS_SCGC4_UART2)
#  define SIM_SCGC4_UART2                     (1 << 12) /* Bit 12: UART2 Clock Gate Control */
#endif
#if defined(KINETIS_SIM_HAS_SCGC4_UART3)
#  define SIM_SCGC4_UART3                     (1 << 13) /* Bit 13: UART3 Clock Gate Control */
#endif
                                                        /* Bits 14-17: Reserved */
#define SIM_SCGC4_USBOTG                      (1 << 18) /* Bit 18: USB Clock Gate Control */
#define SIM_SCGC4_CMP                         (1 << 19) /* Bit 19: Comparator Clock Gate Control */
#define SIM_SCGC4_VREF                        (1 << 20) /* Bit 20: VREF Clock Gate Control */
                                                        /* Bits 21-17: Reserved */
#if defined(KINETIS_SIM_HAS_SCGC4_LLWU)
#  define SIM_SCGC4_LLWU                      (1 << 28) /* Bit 28: LLWU Clock Gate Control */
#endif
                                                /* Bits 29-31: Reserved */

/* System Clock Gating Control Register 5 */

#define SIM_SCGC5_LPTMR0                      (1 << 0)  /* Bit 0:  Low Power Timer 0 Clock Gate Control */
#if defined(KINETIS_SIM_HAS_SCGC5_REGFILE)
#  define SIM_SCGC5_REGFILE                   (1 << 1)  /* Bit 1:  Register File Clock Gate Control */
#endif
                                                        /* Bits 2-3: Reserved */
#if defined(KINETIS_SIM_HAS_SCGC5_LPTMR1)
#  define SIM_SCGC5_LPTMR1                    (1 << 4)  /* Bit 4:  Low Power Timer 1 Clock Gate Control */
#endif
#if defined(KINETIS_SIM_HAS_SCGC5_TSI)
#  define SIM_SCGC5_TSI                       (1 << 5)  /* Bit 5:  TSI Clock Gate Control */
#endif
                                                        /* Bits 6-8: Reserved */
#define SIM_SCGC5_PORTA                       (1 << 9)  /* Bit 9:  Port A Clock Gate Control */
#define SIM_SCGC5_PORTB                       (1 << 10) /* Bit 10: Port B Clock Gate Control */
#define SIM_SCGC5_PORTC                       (1 << 11) /* Bit 11: Port C Clock Gate Control */
#define SIM_SCGC5_PORTD                       (1 << 12) /* Bit 12: Port D Clock Gate Control */
#define SIM_SCGC5_PORTE                       (1 << 13) /* Bit 13: Port E Clock Gate Control */
#if defined(KINETIS_SIM_HAS_SCGC5_PORTF)
#  define SIM_SCGC5_PORTF                     (1 << 14) /* Bit 14: Port F Clock Gate Control */
#endif
                                                /* Bits 14-31: Reserved */

/* System Clock Gating Control Register 6 */

#if defined(KINETIS_SIM_HAS_SCGC6_FTFL)
#  define SIM_SCGC6_FTFL                      (1 << 0)  /* Bit 0:  Flash Memory Clock Gate Control */
#endif
#define SIM_SCGC6_DMAMUX0                     (1 << 1)  /* Bit 1:  DMA Mux 0 Clock Gate Control */
                                                        /* Bits 2-3: Reserved */
#if defined(KINETIS_SIM_HAS_SCGC6_DMAMUX1)
#  define SIM_SCGC6_DMAMUX1                   (1 << 2)  /* Bit 2:  DMA Mux 1 Clock Gate Control */
#endif
#if defined(KINETIS_SIM_HAS_SCGC6_FLEXCAN0)
#  define SIM_SCGC6_FLEXCAN0                  (1 << 4)  /* Bit 4:  FlexCAN0 Clock Gate Control */
#endif
                                                        /* Bits 5-9: Reserved */

#if defined(KINETIS_SIM_HAS_SCGC6_RNGA)
#  define SIM_SCGC6_RNGA                      (1 << 9)  /* Bit 9: SPI0 Clock Gate Control */
#endif
                                                        /* Bits 10-11: Reserved */
#define SIM_SCGC6_SPI0                        (1 << 12) /* Bit 12: SPI0 Clock Gate Control */
#define SIM_SCGC6_SPI1                        (1 << 13) /* Bit 13: SPI1 Clock Gate Control */
                                                        /* Bit 14: Reserved */
#define SIM_SCGC6_I2S0                        (1 << 15) /* Bit 15: I2S0 Clock Gate Control */
                                                        /* Bits 16-17: Reserved */
#define SIM_SCGC6_CRC                         (1 << 18) /* Bit 18: CRC Clock Gate Control */
                                                        /* Bits 19-20: Reserved */
#if defined(KINETIS_SIM_HAS_SCGC6_USBHS)
#  define SIM_SCGC6_USBHS                     (1 << 20) /* Bit 20: USB HS Clock Gate Control */
#endif
#define SIM_SCGC6_USBDCD                      (1 << 21) /* Bit 21: USB DCD Clock Gate Control */
#define SIM_SCGC6_PDB                         (1 << 22) /* Bit 22: PDB Clock Gate Control */
#define SIM_SCGC6_PIT                         (1 << 23) /* Bit 23: PIT Clock Gate Control */
#define SIM_SCGC6_FTM0                        (1 << 24) /* Bit 24: FTM0 Clock Gate Control */
#define SIM_SCGC6_FTM1                        (1 << 25) /* Bit 25: FTM1 Clock Gate Control */
#if defined(KINETIS_SIM_HAS_SCGC6_FTM2)
#  define SIM_SCGC6_FTM2                      (1 << 26) /* Bit 26: FTM2 Clock Gate Control */
#endif
#define SIM_SCGC6_ADC0                        (1 << 27) /* Bit 27: ADC0 Clock Gate Control */
                                                        /* Bit 28: Reserved */
#if defined(KINETIS_SIM_HAS_SCGC6_ADC2)
#  define SIM_SCGC6_ADC2                      (1 << 28) /* Bit 28: ADC2 Clock Gate Control */
#endif
#define SIM_SCGC6_RTC                         (1 << 29) /* Bit 29: RTC Clock Gate Control */
                                                        /* Bits 30-31: Reserved */
#if defined(KINETIS_SIM_HAS_SCGC6_DAC0)
#  define SIM_SCGC6_DAC0                      (1 << 31) /* Bit 31: RTC Clock Gate Control */
#endif

/* System Clock Gating Control Register 7 */

#if defined(KINETIS_SIM_HAS_SCGC7)
#  if defined(KINETIS_SIM_HAS_SCGC7_FLEXBUS)
#    define SIM_SCGC7_FLEXBUS                 (1 << 0)  /* Bit 0:  FlexBus Clock Gate Control */
#  endif
#  if defined(KINETIS_SIM_HAS_SCGC7_DMA)
#    define SIM_SCGC7_DMA                     (1 << 1)  /* Bit 1:  DMA Clock Gate Control */
#  endif
#  if defined(KINETIS_SIM_HAS_SCGC7_MPU)
#    define SIM_SCGC7_MPU                     (1 << 2)  /* Bit 2:  MPU Clock Gate Control */
#  endif
#  if defined(KINETIS_SIM_HAS_SCGC7_SDRAMC)
#    define SIM_SCGC7_SDRAMC                  (1 << 3)  /* Bit 3:  SDRAMC Clock Gate Control */
#  endif
                                                        /* Bits 4-31: Reserved */
#  endif

/* System Clock Divider Register 1 */

#if defined(KINETIS_SIM_HAS_CLKDIV1_OUTDIV5)
                                                        /* Bits 0-15: Reserved */
#endif
#if defined(KINETIS_SIM_HAS_CLKDIV1_OUTDIV4)
#  define SIM_CLKDIV1_OUTDIV4_SHIFT           (16)      /* Bits 16-19: Clock 4 output divider value */
#  define SIM_CLKDIV1_OUTDIV4_MASK            (15 << SIM_CLKDIV1_OUTDIV4_SHIFT)
#    define SIM_CLKDIV1_OUTDIV4(n)            ((uint32_t)(((n)-1) & 0xf) << SIM_CLKDIV1_OUTDIV4_SHIFT) /* n=1..16 */
#    define SIM_CLKDIV1_OUTDIV4_1             (0 << SIM_CLKDIV1_OUTDIV4_SHIFT)  /* Divide by 1 */
#    define SIM_CLKDIV1_OUTDIV4_2             (1 << SIM_CLKDIV1_OUTDIV4_SHIFT)  /* Divide by 2 */
#    define SIM_CLKDIV1_OUTDIV4_3             (2 << SIM_CLKDIV1_OUTDIV4_SHIFT)  /* Divide by 3 */
#    define SIM_CLKDIV1_OUTDIV4_4             (3 << SIM_CLKDIV1_OUTDIV4_SHIFT)  /* Divide by 4 */
#    define SIM_CLKDIV1_OUTDIV4_5             (4 << SIM_CLKDIV1_OUTDIV4_SHIFT)  /* Divide by 5 */
#    define SIM_CLKDIV1_OUTDIV4_6             (5 << SIM_CLKDIV1_OUTDIV4_SHIFT)  /* Divide by 6 */
#    define SIM_CLKDIV1_OUTDIV4_7             (6 << SIM_CLKDIV1_OUTDIV4_SHIFT)  /* Divide by 7 */
#    define SIM_CLKDIV1_OUTDIV4_8             (7 << SIM_CLKDIV1_OUTDIV4_SHIFT)  /* Divide by 8 */
#    define SIM_CLKDIV1_OUTDIV4_9             (8 << SIM_CLKDIV1_OUTDIV4_SHIFT)  /* Divide by 9 */
#    define SIM_CLKDIV1_OUTDIV4_10            (9 << SIM_CLKDIV1_OUTDIV4_SHIFT)  /* Divide by 10 */
#    define SIM_CLKDIV1_OUTDIV4_11            (10 << SIM_CLKDIV1_OUTDIV4_SHIFT) /* Divide by 11 */
#    define SIM_CLKDIV1_OUTDIV4_12            (11 << SIM_CLKDIV1_OUTDIV4_SHIFT) /* Divide by 12 */
#    define SIM_CLKDIV1_OUTDIV4_13            (12 << SIM_CLKDIV1_OUTDIV4_SHIFT) /* Divide by 13 */
#    define SIM_CLKDIV1_OUTDIV4_14            (13 << SIM_CLKDIV1_OUTDIV4_SHIFT) /* Divide by 14 */
#    define SIM_CLKDIV1_OUTDIV4_15            (14 << SIM_CLKDIV1_OUTDIV4_SHIFT) /* Divide by 15 */
#    define SIM_CLKDIV1_OUTDIV4_16            (15 << SIM_CLKDIV1_OUTDIV4_SHIFT) /* Divide by 16 */
#endif
#if defined(KINETIS_SIM_HAS_CLKDIV1_OUTDIV3)
#  define SIM_CLKDIV1_OUTDIV3_SHIFT           (20)      /* Bits 20-23: Clock 3 output divider value */
#  define SIM_CLKDIV1_OUTDIV3_MASK            (15 << SIM_CLKDIV1_OUTDIV3_SHIFT)
#    define SIM_CLKDIV1_OUTDIV3(n)            ((uint32_t)(((n)-1) & 0xf) << SIM_CLKDIV1_OUTDIV3_SHIFT) /* n=1..16 */
#    define SIM_CLKDIV1_OUTDIV3_1             (0 << SIM_CLKDIV1_OUTDIV3_SHIFT)  /* Divide by 1 */
#    define SIM_CLKDIV1_OUTDIV3_2             (1 << SIM_CLKDIV1_OUTDIV3_SHIFT)  /* Divide by 2 */
#    define SIM_CLKDIV1_OUTDIV3_3             (2 << SIM_CLKDIV1_OUTDIV3_SHIFT)  /* Divide by 3 */
#    define SIM_CLKDIV1_OUTDIV3_4             (3 << SIM_CLKDIV1_OUTDIV3_SHIFT)  /* Divide by 4 */
#    define SIM_CLKDIV1_OUTDIV3_5             (4 << SIM_CLKDIV1_OUTDIV3_SHIFT)  /* Divide by 5 */
#    define SIM_CLKDIV1_OUTDIV3_6             (5 << SIM_CLKDIV1_OUTDIV3_SHIFT)  /* Divide by 6 */
#    define SIM_CLKDIV1_OUTDIV3_7             (6 << SIM_CLKDIV1_OUTDIV3_SHIFT)  /* Divide by 7 */
#    define SIM_CLKDIV1_OUTDIV3_8             (7 << SIM_CLKDIV1_OUTDIV3_SHIFT)  /* Divide by 8 */
#    define SIM_CLKDIV1_OUTDIV3_9             (8 << SIM_CLKDIV1_OUTDIV3_SHIFT)  /* Divide by 9 */
#    define SIM_CLKDIV1_OUTDIV3_10            (9 << SIM_CLKDIV1_OUTDIV3_SHIFT)  /* Divide by 10 */
#    define SIM_CLKDIV1_OUTDIV3_11            (10 << SIM_CLKDIV1_OUTDIV3_SHIFT) /* Divide by 11 */
#    define SIM_CLKDIV1_OUTDIV3_12            (11 << SIM_CLKDIV1_OUTDIV3_SHIFT) /* Divide by 12 */
#    define SIM_CLKDIV1_OUTDIV3_13            (12 << SIM_CLKDIV1_OUTDIV3_SHIFT) /* Divide by 13 */
#    define SIM_CLKDIV1_OUTDIV3_14            (13 << SIM_CLKDIV1_OUTDIV3_SHIFT) /* Divide by 14 */
#    define SIM_CLKDIV1_OUTDIV3_15            (14 << SIM_CLKDIV1_OUTDIV3_SHIFT) /* Divide by 15 */
#    define SIM_CLKDIV1_OUTDIV3_16            (15 << SIM_CLKDIV1_OUTDIV3_SHIFT) /* Divide by 16 */
#endif
#if defined(KINETIS_SIM_HAS_CLKDIV1_OUTDIV2)
#  define SIM_CLKDIV1_OUTDIV2_SHIFT           (24)      /* Bits 24-27: Clock 2 output divider value */
#  define SIM_CLKDIV1_OUTDIV2_MASK            (15 << SIM_CLKDIV1_OUTDIV2_SHIFT)
#    define SIM_CLKDIV1_OUTDIV2(n)            ((uint32_t)(((n)-1) & 0xf) << SIM_CLKDIV1_OUTDIV2_SHIFT) /* n=1..16 */
#    define SIM_CLKDIV1_OUTDIV2_1             (0 << SIM_CLKDIV1_OUTDIV2_SHIFT)  /* Divide by 1 */
#    define SIM_CLKDIV1_OUTDIV2_2             (1 << SIM_CLKDIV1_OUTDIV2_SHIFT)  /* Divide by 2 */
#    define SIM_CLKDIV1_OUTDIV2_3             (2 << SIM_CLKDIV1_OUTDIV2_SHIFT)  /* Divide by 3 */
#    define SIM_CLKDIV1_OUTDIV2_4             (3 << SIM_CLKDIV1_OUTDIV2_SHIFT)  /* Divide by 4 */
#    define SIM_CLKDIV1_OUTDIV2_5             (4 << SIM_CLKDIV1_OUTDIV2_SHIFT)  /* Divide by 5 */
#    define SIM_CLKDIV1_OUTDIV2_6             (5 << SIM_CLKDIV1_OUTDIV2_SHIFT)  /* Divide by 6 */
#    define SIM_CLKDIV1_OUTDIV2_7             (6 << SIM_CLKDIV1_OUTDIV2_SHIFT)  /* Divide by 7 */
#    define SIM_CLKDIV1_OUTDIV2_8             (7 << SIM_CLKDIV1_OUTDIV2_SHIFT)  /* Divide by 8 */
#    define SIM_CLKDIV1_OUTDIV2_9             (8 << SIM_CLKDIV1_OUTDIV2_SHIFT)  /* Divide by 9 */
#    define SIM_CLKDIV1_OUTDIV2_10            (9 << SIM_CLKDIV1_OUTDIV2_SHIFT)  /* Divide by 10 */
#    define SIM_CLKDIV1_OUTDIV2_11            (10 << SIM_CLKDIV1_OUTDIV2_SHIFT) /* Divide by 11 */
#    define SIM_CLKDIV1_OUTDIV2_12            (11 << SIM_CLKDIV1_OUTDIV2_SHIFT) /* Divide by 12 */
#    define SIM_CLKDIV1_OUTDIV2_13            (12 << SIM_CLKDIV1_OUTDIV2_SHIFT) /* Divide by 13 */
#    define SIM_CLKDIV1_OUTDIV2_14            (13 << SIM_CLKDIV1_OUTDIV2_SHIFT) /* Divide by 14 */
#    define SIM_CLKDIV1_OUTDIV2_15            (14 << SIM_CLKDIV1_OUTDIV2_SHIFT) /* Divide by 15 */
#    define SIM_CLKDIV1_OUTDIV2_16            (15 << SIM_CLKDIV1_OUTDIV2_SHIFT) /* Divide by 16 */
#endif
#define SIM_CLKDIV1_OUTDIV1_SHIFT             (28)      /* Bits 28-31: Clock 1 output divider value */
#define SIM_CLKDIV1_OUTDIV1_MASK              (15 << SIM_CLKDIV1_OUTDIV1_SHIFT)
#  define SIM_CLKDIV1_OUTDIV1(n)              ((uint32_t)(((n)-1) & 0xf) << SIM_CLKDIV1_OUTDIV1_SHIFT) /* n=1..16 */
#  define SIM_CLKDIV1_OUTDIV1_1               (0 << SIM_CLKDIV1_OUTDIV1_SHIFT)  /* Divide by 1 */
#  define SIM_CLKDIV1_OUTDIV1_2               (1 << SIM_CLKDIV1_OUTDIV1_SHIFT)  /* Divide by 2 */
#  define SIM_CLKDIV1_OUTDIV1_3               (2 << SIM_CLKDIV1_OUTDIV1_SHIFT)  /* Divide by 3 */
#  define SIM_CLKDIV1_OUTDIV1_4               (3 << SIM_CLKDIV1_OUTDIV1_SHIFT)  /* Divide by 4 */
#  define SIM_CLKDIV1_OUTDIV1_5               (4 << SIM_CLKDIV1_OUTDIV1_SHIFT)  /* Divide by 5 */
#  define SIM_CLKDIV1_OUTDIV1_6               (5 << SIM_CLKDIV1_OUTDIV1_SHIFT)  /* Divide by 6 */
#  define SIM_CLKDIV1_OUTDIV1_7               (6 << SIM_CLKDIV1_OUTDIV1_SHIFT)  /* Divide by 7 */
#  define SIM_CLKDIV1_OUTDIV1_8               (7 << SIM_CLKDIV1_OUTDIV1_SHIFT)  /* Divide by 8 */
#  define SIM_CLKDIV1_OUTDIV1_9               (8 << SIM_CLKDIV1_OUTDIV1_SHIFT)  /* Divide by 9 */
#  define SIM_CLKDIV1_OUTDIV1_10              (9 << SIM_CLKDIV1_OUTDIV1_SHIFT)  /* Divide by 10 */
#  define SIM_CLKDIV1_OUTDIV1_11              (10 << SIM_CLKDIV1_OUTDIV1_SHIFT) /* Divide by 11 */
#  define SIM_CLKDIV1_OUTDIV1_12              (11 << SIM_CLKDIV1_OUTDIV1_SHIFT) /* Divide by 12 */
#  define SIM_CLKDIV1_OUTDIV1_13              (12 << SIM_CLKDIV1_OUTDIV1_SHIFT) /* Divide by 13 */
#  define SIM_CLKDIV1_OUTDIV1_14              (13 << SIM_CLKDIV1_OUTDIV1_SHIFT) /* Divide by 14 */
#  define SIM_CLKDIV1_OUTDIV1_15              (14 << SIM_CLKDIV1_OUTDIV1_SHIFT) /* Divide by 15 */
#  define SIM_CLKDIV1_OUTDIV1_16              (15 << SIM_CLKDIV1_OUTDIV1_SHIFT) /* Divide by 16 */

/* System Clock Divider Register 2 */

#if defined(KINETIS_SIM_HAS_CLKDIV2_USBFRAC)
#  define SIM_CLKDIV2_USBFRAC_SHIFT           (0)  /* Bit 0:  USB clock divider fraction */
#  define SIM_CLKDIV2_USBFRAC_MASK            (1 << SIM_CLKDIV2_USBFRAC_SHIFT)
#    define SIM_CLKDIV2_USBFRAC(n)            ((((n)-1) & 1) << SIM_CLKDIV2_USBFRAC_SHIFT) /* n=1..2 */
#endif
#if defined(KINETIS_SIM_HAS_CLKDIV2_USBDIV)
#  define SIM_CLKDIV2_USBDIV_SHIFT            (1)       /* Bits 1-3: USB clock divider divisor */
#  define SIM_CLKDIV2_USBDIV_MASK             (7 << SIM_CLKDIV2_USBDIV_SHIFT)
#  define SIM_CLKDIV2_USBDIV(n)               ((((n)-1) & 7) << SIM_CLKDIV2_USBDIV_SHIFT) /* n=1..8 */
#endif
                                                        /* Bits 4-7: Reserved */
#if defined(KINETIS_SIM_HAS_CLKDIV2_USBHSFRAC)
#  define SIM_CLKDIV2_USBHSFRAC_SHIFT         (8)       /* Bit 8:  USB HS clock divider fraction */
#  define SIM_CLKDIV2_USBHSFRAC_MASK          (1 << SIM_CLKDIV2_USBHSFRAC_SHIFT)
#    define SIM_CLKDIV2_USBHSFRAC(n)          ((((n)-1) & 1) << SIM_CLKDIV2_USBHSFRAC_SHIFT) /* n=1..2 */
#endif
#if defined(KINETIS_SIM_HAS_CLKDIV2_USBHSDIV)
#  define SIM_CLKDIV2_USBHSDIV_SHIFT          (9)       /* Bits 1-3: USB HS clock divider divisor */
#  define SIM_CLKDIV2_USBHSDIV_MASK           (7 << SIM_CLKDIV2_USBHSDIV_SHIFT)
#  define SIM_CLKDIV2_USBHSDIV(n)             ((((n)-1) & 7) << SIM_CLKDIV2_USBHSDIV_SHIFT) /* n=1..8 */
#endif
#if defined(KINETIS_SIM_HAS_CLKDIV2_I2SFRAC)
#  define SIM_CLKDIV2_I2SFRAC_SHIFT           (8)       /* Bits 8-15: I2S clock divider fraction */
#  define SIM_CLKDIV2_I2SFRAC_MASK            (0xff << SIM_CLKDIV2_I2SFRAC_SHIFT)
#endif
                                                        /* Bits 16-19: Reserved */
#if defined(KINETIS_SIM_HAS_CLKDIV2_I2SDIV)
#  define SIM_CLKDIV2_I2SDIV_SHIFT            (20)      /* Bits 20-31: I2S clock divider value */
#  define SIM_CLKDIV2_I2SDIV_MASK             (0xfff << SIM_CLKDIV2_I2SDIV_SHIFT)
#endif

/* Flash Configuration Register 1 */

#if defined(KINETIS_SIM_HAS_FCFG1_FTFDIS)
#  define SIM_FCFG1_FTFDIS                    (1 << 0)  /* Bit 0:  Disable FTFE */
#endif
#if defined(KINETIS_SIM_HAS_FCFG1_FLASHDIS)
#  define SIM_FCFG1_FLASHDIS                  (1 << 0)  /* Bit 0:  Flash Disable */
#endif
#if defined(KINETIS_SIM_HAS_FCFG1_FLASHDOZE)
#  define SIM_FCFG1_FLASHDOZE                 (1 << 1)  /* Bit 1:  Flash Doze */
#endif
                                                        /* Bits 0-7: Reserved */
#if defined(KINETIS_SIM_HAS_FCFG1_DEPART)
#  define SIM_FCFG1_DEPART_SHIFT              (8)       /* Bits 8-11: FlexNVM partition */
#  define SIM_FCFG1_DEPART_MASK               (15 << SIM_FCFG1_DEPART_SHIFT)
#endif
                                                        /* Bits 12-15: Reserved */
#if defined(KINETIS_SIM_HAS_FCFG1_EESIZE)
#  define SIM_FCFG1_EESIZE_SHIFT              (16)      /* Bits 16-19: EEPROM size */
#  define SIM_FCFG1_EESIZE_MASK               (15 << SIM_FCFG1_EESIZE_SHIFT)
#    define SIM_FCFG1_EESIZE_16KB             (0 << SIM_FCFG1_EESIZE_SHIFT)  /* 16 KB */
#    define SIM_FCFG1_EESIZE_8KB              (1 << SIM_FCFG1_EESIZE_SHIFT)  /* 8 KB */
#    define SIM_FCFG1_EESIZE_4KB              (2 << SIM_FCFG1_EESIZE_SHIFT)  /* 4 KB */
#    define SIM_FCFG1_EESIZE_2KB              (3 << SIM_FCFG1_EESIZE_SHIFT)  /* 2 KB */
#    define SIM_FCFG1_EESIZE_1KB              (4 << SIM_FCFG1_EESIZE_SHIFT)  /* 1 KB */
#    define SIM_FCFG1_EESIZE_512B             (5 << SIM_FCFG1_EESIZE_SHIFT)  /* 512 Bytes */
#    define SIM_FCFG1_EESIZE_256B             (6 << SIM_FCFG1_EESIZE_SHIFT)  /* 256 Bytes */
#    define SIM_FCFG1_EESIZE_128B             (7 << SIM_FCFG1_EESIZE_SHIFT)  /* 128 Bytes */
#    define SIM_FCFG1_EESIZE_64B              (8 << SIM_FCFG1_EESIZE_SHIFT)  /* 64 Bytes */
#    define SIM_FCFG1_EESIZE_32B              (9 << SIM_FCFG1_EESIZE_SHIFT)  /* 32 Bytes */
#    define SIM_FCFG1_EESIZE_NONE             (15 << SIM_FCFG1_EESIZE_SHIFT) /* 0 Bytes */
#endif
                                                        /* Bits 20-23: Reserved */
#define SIM_FCFG1_PFSIZE_SHIFT                (24)      /* Bits 24-27: Program flash size */
#define SIM_FCFG1_PFSIZE_MASK                 (15 << SIM_FCFG1_PFSIZE_SHIFT)
#  if defined(KINETIS_K40)
#    define SIM_FCFG1_PFSIZE_128KB            (7 << SIM_FCFG1_PFSIZE_SHIFT)  /* 128KB program flash, 4KB protection region */
#    define SIM_FCFG1_PFSIZE_256KB            (9 << SIM_FCFG1_PFSIZE_SHIFT)  /* 256KB program flash, 8KB protection region */
#    define SIM_FCFG1_PFSIZE_512KB            (11 << SIM_FCFG1_PFSIZE_SHIFT) /* 512KB program flash, 16KB protection region */
#    define SIM_FCFG1_PFSIZE_512KB2           (15 << SIM_FCFG1_PFSIZE_SHIFT) /* 512KB program flash, 16KB protection region */
#  endif
#  if defined(KINETIS_K60)
#    define SIM_FCFG1_PFSIZE_512KB            (11 << SIM_FCFG1_PFSIZE_SHIFT)  /* 512 KB, 16 KB protection size */
#    define SIM_FCFG1_PFSIZE_1024KB           (13 << SIM_FCFG1_PFSIZE_SHIFT)  /* 1024 KB, 32 KB protection size */
#    define SIM_FCFG1_PFSIZE_2048KB           (15 << SIM_FCFG1_PFSIZE_SHIFT)  /* 1024 KB, 32 KB protection size */
#  endif
#  if defined(KINETIS_K28) || defined(KINETIS_K64) || defined(KINETIS_K66)
#    define SIM_FCFG1_PFSIZE_32KB             (3 << SIM_FCFG1_PFSIZE_SHIFT)  /* 32 KB of program flash memory */
#    define SIM_FCFG1_PFSIZE_64KB             (5 << SIM_FCFG1_PFSIZE_SHIFT)  /* 64 KB of program flash memory */
#    define SIM_FCFG1_PFSIZE_128KB            (7 << SIM_FCFG1_PFSIZE_SHIFT)  /* 128 KB of program flash memory */
#    define SIM_FCFG1_PFSIZE_256KB            (9 << SIM_FCFG1_PFSIZE_SHIFT)  /* 256 KB of program flash memory */
#    define SIM_FCFG1_PFSIZE_512KB            (11 << SIM_FCFG1_PFSIZE_SHIFT) /* 512 KB of program flash memory */
#    define SIM_FCFG1_PFSIZE_1024KB           (13 << SIM_FCFG1_PFSIZE_SHIFT) /* 1024 KB of program flash memory */
#    define SIM_FCFG1_PFSIZE_2048KB           (15 << SIM_FCFG1_PFSIZE_SHIFT) /* 2048 KB of program flash memory */
#  endif

#if defined(KINETIS_SIM_HAS_FCFG1_NVMSIZE)
#  define SIM_FCFG1_NVMSIZE_SHIFT             (28)      /* Bits 28-31: FlexNVM size */
#  define SIM_FCFG1_NVMSIZE_MASK              (15 << SIM_FCFG1_NVMSIZE_SHIFT)
#    define SIM_FCFG1_NVMSIZE_NONE            (0 << SIM_FCFG1_NVMSIZE_SHIFT)  /* 0KB FlexNVM */
#  if defined(KINETIS_K28)
#    define SIM_FCFG1_NVMSIZE_32KB            (3 << SIM_FCFG1_NVMSIZE_SHIFT)  /* 32KB FlexNVM */
#    define SIM_FCFG1_NVMSIZE_64KB            (5 << SIM_FCFG1_NVMSIZE_SHIFT)  /* 64KB FlexNVM */
#    define SIM_FCFG1_NVMSIZE_128KB           (7 << SIM_FCFG1_NVMSIZE_SHIFT)  /* 128KB FlexNVM */
#    define SIM_FCFG1_NVMSIZE_256KB           (9 << SIM_FCFG1_NVMSIZE_SHIFT)  /* 256KB FlexNVM */
#    define SIM_FCFG1_NVMSIZE_512KB           (11 << SIM_FCFG1_NVMSIZE_SHIFT) /* 512KB FlexNVM */
#  else
#    define SIM_FCFG1_NVMSIZE_128KB           (7 << SIM_FCFG1_NVMSIZE_SHIFT)  /* 128KB FlexNVM, 16KB protection region */
#    define SIM_FCFG1_NVMSIZE_256KB           (9 << SIM_FCFG1_NVMSIZE_SHIFT)  /* 256KB FlexNVM, 32KB protection region */
#    define SIM_FCFG1_NVMSIZE_256KB2          (15 << SIM_FCFG1_NVMSIZE_SHIFT) /* 256KB FlexNVM, 32KB protection region */
#  endif
#endif

/* Flash Configuration Register 2 */
                                                /* Bits 0-15: Reserved */
#if defined(KINETIS_SIM_HAS_FCFG2_MAXADDR1)
#  define SIM_FCFG2_MAXADDR1_SHIFT            (16)      /* Bits 16-[21|22]: Max address block 1 */
#  define SIM_FCFG2_MAXADDR1_MASK             (KINETIS_SIM_FCFG2_MAXADDR1_MASK << SIM_FCFG2_MAXADDR1_SHIFT)
#  define SIM_FCFG2_MAXADDR1(n)               (((n) & KINETIS_SIM_FCFG2_MAXADDR1_MASK) << SIM_FCFG2_MAXADDR1_SHIFT)
#endif
                                                        /* Bit 22: Reserved */
#if defined(KINETIS_SIM_HAS_FCFG2_PFLSH)
#  define SIM_FCFG2_PFLSH                     (1 << 23) /* Bit 23: Program flash */
#endif
#if defined(KINETIS_SIM_HAS_FCFG2_MAXADDR0)
#  define SIM_FCFG2_MAXADDR0_SHIFT            (24)      /* Bits 24-[29|30]: Max address block 0 */
#  define SIM_FCFG2_MAXADDR0_MASK             (KINETIS_SIM_FCFG2_MAXADDR0_MASK << SIM_FCFG2_MAXADDR0_SHIFT)
#  define SIM_FCFG2_MAXADDR0(n)               (((n) & KINETIS_SIM_FCFG2_MAXADDR0_MASK) << SIM_FCFG2_MAXADDR0_SHIFT)
                                                        /* Bit 30: Reserved */
#endif
#if defined(KINETIS_SIM_HAS_FCFG2_SWAPPFLSH)
#  define SIM_FCFG2_SWAPPFLSH                 (1 << 31) /* Bit 31: Swap program flash */
#endif

/* Unique Identification Register High. 32-bit Unique Identification. */
/* Unique Identification Register Mid-High. 32-bit Unique Identification. */
/* Unique Identification Register Mid Low. 32-bit Unique Identification. */
/* Unique Identification Register Low. 32-bit Unique Identification. */

#if defined(KINETIS_SIM_HAS_CLKDIV3)
/* System Clock Divider Register 3 */

#  if defined(KINETIS_SIM_HAS_CLKDIV3_PLLFLLFRAC)
#    define SIM_CLKDIV3_PLLFLLFRAC_SHIFT       (0) /* Bit 0: PLLFLL clock divider fraction */
#    define SIM_CLKDIV3_PLLFLLFRAC_MASK        (1 << SIM_CLKDIV3_PLLFLLFRAC_SHIFT)
#      define SIM_CLKDIV3_PLLFLLFRAC(n)         ((((n)-1) & 1) <<  SIM_CLKDIV3_PLLFLLFRAC_SHIFT) /* n=1..2 */
#  endif
#  if defined(KINETIS_SIM_HAS_CLKDIV3_PLLFLLDIV)
#    define SIM_CLKDIV3_PLLFLLDIV_SHIFT       (1)      /* Bits 1-3: PLLFLL clock divider divisor */
#    define SIM_CLKDIV3_PLLFLLDIV_MASK        (7 << SIM_CLKDIV3_PLLFLLDIV_SHIFT)
#      define SIM_CLKDIV3_PLLFLLDIV(n)        ((((n)-1) & 7)  << SIM_CLKDIV3_PLLFLLDIV_SHIFT) /* n=1..8 */
#  endif
#endif

/* System Clock Divider Register 4 */

#if defined(KINETIS_SIM_HAS_CLKDIV4)
#  if defined(KINETIS_SIM_HAS_CLKDIV4_TRACEFRAC)
#    define SIM_CLKDIV4_TRACEFRAC_SHIFTS      (0)      /* Bit 0: Trace clock divider fraction */
#    define SIM_CLKDIV4_TRACEFRAC_MASK        (1 << SIM_CLKDIV4_TRACEFRAC_SHIFTS)
#    define SIM_CLKDIV4_TRACEFRAC(n)          ((((n)-1) & 1) <<  SIM_CLKDIV4_TRACEFRAC_SHIFTS) /* n=1..2 */
#  endif
#  if defined(KINETIS_SIM_HAS_CLKDIV4_TRACEDIV)
#    define SIM_CLKDIV4_TRACEDIV_SHIFT        (1)      /* Bits 1-3: Trace clock divider divisor */
#    define SIM_CLKDIV4_TRACEDIV_MASK         (7 << SIM_CLKDIV3_TRACEDIV_SHIFT)
#      define SIM_CLKDIV4_TRACEDIV(n)         ((((n)-1) & 7)  << SIM_CLKDIV4_TRACEDIV_SHIFT) /* n=1..8 */
#  endif
#  if defined(KINETIS_SIM_HAS_CLKDIV4_NFCFRAC)
#    define SIM_CLKDIV4_NFCFRAC_SHIFT         (24) /* Bits 24-26: NFC clock divider fraction */
#    define SIM_CLKDIV4_NFCFRAC_MASK          (7 << SIM_CLKDIV4_NFCFRAC_SHIFT)
#    define SIM_CLKDIV4_NFCFRAC(n)            ((((n)-1) & 7) << SIM_CLKDIV4_NFCFRAC_SHIFT) /* n=1..8 */
#  endif
#  if defined(KINETIS_SIM_HAS_CLKDIV4_NFCDIV)
#    define SIM_CLKDIV4_NFCDIV_SHIFT          (27)      /* Bits 27-31: NFC clock divider divisor */
#    define SIM_CLKDIV4_NFCDIV_MASK           (31 << SIM_CLKDIV3_NFCDIV_SHIFT)
#      define SIM_CLKDIV4_NFCDIV(n)           ((((n)-1) & 31)  << SIM_CLKDIV4_NFCDIV_SHIFT) /* n=1..32 */
#  endif
#endif

/* Misc Control Register */

#if defined(KINETIS_SIM_HAS_MCR)
                                                            /* Bits 0-28: Reserved */
#  define SIM_MCR_PDBLOOP                     (1<< 29)      /* Bit 29: PDB Loop Mode */
                                                            /* Bit 30: Reserved */
#  define SIM_MCR_TRACECLKDIS                 (1<< 31)      /* Bit 31: Trace clock disable. */
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_SIM_H */
