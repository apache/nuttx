/****************************************************************************
 * arch/arm/src/samv7/hardware/sam_usbhs.h
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
 *   SAMV7D3 Series Data Sheet
 */

#ifndef __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_USBHS_H
#define __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_USBHS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/samv7/chip.h>

#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* General Definitions ******************************************************/

/* Number of endpoints and DMA channels */

#define SAM_USBHS_NENDPOINTS               10                /* EP0-9 */
#define SAM_USBHS_NDMACHANNELS             7                 /* For DMA1-7 */

/* Capabilities and characteristics of endpoints */

#define SAM_USBHS_MAXPACKETSIZE(ep)       (((unsigned)(ep) < 1) ? 64 : 1024)
#define SAM_USBHS_NBANKS(ep)              (((unsigned)(ep) < 1) ? 1 : (((unsigned)(ep) < 3) ? 3 : 2))
#define SAM_USBHS_DMA(ep)                 (((unsigned)(ep) < 1) ? false : (((unsigned)(ep) < 8) ? true : false))

/* Register offsets *********************************************************/

/* USBHS Device Controller Register Offsets */

#define SAM_USBHS_DEVCTRL_OFFSET           0x0000            /* Device General Control Register */
#define SAM_USBHS_DEVISR_OFFSET            0x0004            /* Device Global Interrupt Status Register */
#define SAM_USBHS_DEVICR_OFFSET            0x0008            /* Device Global Interrupt Clear Register */
#define SAM_USBHS_DEVIFR_OFFSET            0x000c            /* Device Global Interrupt Set Register */
#define SAM_USBHS_DEVIMR_OFFSET            0x0010            /* Device Global Interrupt Mask Register */
#define SAM_USBHS_DEVIDR_OFFSET            0x0014            /* Device Global Interrupt Disable Register */
#define SAM_USBHS_DEVIER_OFFSET            0x0018            /* Device Global Interrupt Enable Register */
#define SAM_USBHS_DEVEPT_OFFSET            0x001c            /* Device Endpoint Register */
#define SAM_USBHS_DEVFNUM_OFFSET           0x0020            /* Device Frame Number Register */

#define SAM_USBHS_DEVEPTCFG_OFFSET(n)      (0x0100+((n)<<2)) /* Device Endpoint Configuration Register */
#define SAM_USBHS_DEVEPTISR_OFFSET(n)      (0x0130+((n)<<2)) /* Device Endpoint Status Register */
#define SAM_USBHS_DEVEPTICR_OFFSET(n)      (0x0160+((n)<<2)) /* Device Endpoint Clear Register */
#define SAM_USBHS_DEVEPTIFR_OFFSET(n)      (0x0190+((n)<<2)) /* Device Endpoint Set Register */
#define SAM_USBHS_DEVEPTIMR_OFFSET(n)      (0x01c0+((n)<<2)) /* Device Endpoint Mask Register */
#define SAM_USBHS_DEVEPTIER_OFFSET(n)      (0x01f0+((n)<<2)) /* Device Endpoint Enable Register */
#define SAM_USBHS_DEVEPTIDR_OFFSET(n)      (0x0220+((n)<<2)) /* Device Endpoint Disable Register */

#define SAM_USBHS_DEVDMANXTDSC_OFFSET(n)   (0x0300+((n)<<4)) /* Device DMA Channel Next Descriptor Address Register */
#define SAM_USBHS_DEVDMAADDR_OFFSET(n)     (0x0304+((n)<<4)) /* Device DMA Channel Address Register */
#define SAM_USBHS_DEVDMACTRL_OFFSET(n)     (0x0308+((n)<<4)) /* Device DMA Channel Control Register */
#define SAM_USBHS_DEVDMASTA_OFFSET(n)      (0x030c+((n)<<4)) /* Device DMA Channel Status Register */

/* USBHS Mini-Host Controller Register Offsets */

#define SAM_USBHS_HSTCTRL_OFFSET           0x0400            /* Host General Control Register */
#define SAM_USBHS_HSTISR_OFFSET            0x0404            /* Host Global Interrupt Status Register */
#define SAM_USBHS_HSTICR_OFFSET            0x0408            /* Host Global Interrupt Clear Register */
#define SAM_USBHS_HSTIFR_OFFSET            0x040c            /* Host Global Interrupt Set Register */
#define SAM_USBHS_HSTIMR_OFFSET            0x0410            /* Host Global Interrupt Mask Register */
#define SAM_USBHS_HSTIDR_OFFSET            0x0414            /* Host Global Interrupt Disable Register */
#define SAM_USBHS_HSTIER_OFFSET            0x0418            /* Host Global Interrupt Enable Register */
#define SAM_USBHS_HSTPIP_OFFSET            0x041c            /* Host Pipe Register */
#define SAM_USBHS_HSTFNUM_OFFSET           0x0420            /* Host Frame Number Register */
#define SAM_USBHS_HSTADDR1_OFFSET          0x0424            /* Host Address 1 Register */
#define SAM_USBHS_HSTADDR2_OFFSET          0x0428            /* Host Address 2 Register */
#define SAM_USBHS_HSTADDR3_OFFSET          0x042c            /* Host Address 3 Register */

#define SAM_USBHS_HSTPIPCFG_OFFSET(n)      (0x0500+((n)<<2)) /* Host Pipe Configuration Register */
#define SAM_USBHS_HSTPIPISR_OFFSET(n)      (0x0530+((n)<<2)) /* Host Pipe Status Register */
#define SAM_USBHS_HSTPIPICR_OFFSET(n)      (0x0560+((n)<<2)) /* Host Pipe Clear Register */
#define SAM_USBHS_HSTPIPIFR_OFFSET(n)      (0x0590+((n)<<2)) /* Host Pipe Set Register */
#define SAM_USBHS_HSTPIPIMR_OFFSET(n)      (0x05c0+((n)<<2)) /* Host Pipe Mask Register */
#define SAM_USBHS_HSTPIPIER_OFFSET(n)      (0x05f0+((n)<<2)) /* Host Pipe Enable Register */
#define SAM_USBHS_HSTPIPIDR_OFFSET(n)      (0x0620+((n)<<2)) /* Host Pipe Disable Register */
#define SAM_USBHS_HSTPIPINRQ_OFFSET(n)     (0x0650+((n)<<2)) /* Host Pipe IN Request Register */
#define SAM_USBHS_HSTPIPERR_OFFSET(n)      (0x0680+((n)<<2)) /* Host Pipe Error Register */

#define SAM_USBHS_HSTDMANXTDSC_OFFSET(n)   (0x0700+((n)<<4)) /* Host DMA Channel Next Descriptor Address Register */
#define SAM_USBHS_HSTDMAADDR_OFFSET(n)     (0x0704+((n)<<4)) /* Host DMA Channel Address Register */
#define SAM_USBHS_HSTDMACTRL_OFFSET(n)     (0x0708+((n)<<4)) /* Host DMA Channel Control Register */
#define SAM_USBHS_HSTDMASTA_OFFSET(n)      (0x070c+((n)<<4)) /* Host DMA Channel Status Register */

/* USBHS General Register Offsets */

#define SAM_USBHS_CTRL_OFFSET              0x0800            /* General Control Register */
#define SAM_USBHS_SR_OFFSET                0x0804            /* General Status Register */
#define SAM_USBHS_SCR_OFFSET               0x0808            /* General Status Clear Register */
#define SAM_USBHS_SFR_OFFSET               0x080c            /* General Status Set Register */
                                                             /* 0x0810-0x082c: Reserved */

/* Register addresses *******************************************************/

/* USBHS Device Controller Register Addresses */

#define SAM_USBHS_DEVCTRL                  (SAM_USBHS_BASE+SAM_USBHS_DEVCTRL_OFFSET)
#define SAM_USBHS_DEVISR                   (SAM_USBHS_BASE+SAM_USBHS_DEVISR_OFFSET)
#define SAM_USBHS_DEVICR                   (SAM_USBHS_BASE+SAM_USBHS_DEVICR_OFFSET)
#define SAM_USBHS_DEVIFR                   (SAM_USBHS_BASE+SAM_USBHS_DEVIFR_OFFSET)
#define SAM_USBHS_DEVIMR                   (SAM_USBHS_BASE+SAM_USBHS_DEVIMR_OFFSET)
#define SAM_USBHS_DEVIDR                   (SAM_USBHS_BASE+SAM_USBHS_DEVIDR_OFFSET)
#define SAM_USBHS_DEVIER                   (SAM_USBHS_BASE+SAM_USBHS_DEVIER_OFFSET)
#define SAM_USBHS_DEVEPT                   (SAM_USBHS_BASE+SAM_USBHS_DEVEPT_OFFSET)
#define SAM_USBHS_DEVFNUM                  (SAM_USBHS_BASE+SAM_USBHS_DEVFNUM_OFFSET)

#define SAM_USBHS_DEVEPTCFG(n)             (SAM_USBHS_BASE+SAM_USBHS_DEVEPTCFG_OFFSET(n))
#define SAM_USBHS_DEVEPTISR(n)             (SAM_USBHS_BASE+SAM_USBHS_DEVEPTISR_OFFSET(n))
#define SAM_USBHS_DEVEPTICR(n)             (SAM_USBHS_BASE+SAM_USBHS_DEVEPTICR_OFFSET(n))
#define SAM_USBHS_DEVEPTIFR(n)             (SAM_USBHS_BASE+SAM_USBHS_DEVEPTIFR_OFFSET(n))
#define SAM_USBHS_DEVEPTIMR(n)             (SAM_USBHS_BASE+SAM_USBHS_DEVEPTIMR_OFFSET(n))
#define SAM_USBHS_DEVEPTIER(n)             (SAM_USBHS_BASE+SAM_USBHS_DEVEPTIER_OFFSET(n))
#define SAM_USBHS_DEVEPTIDR(n)             (SAM_USBHS_BASE+SAM_USBHS_DEVEPTIDR_OFFSET(n))

#define SAM_USBHS_DEVDMANXTDSC(n)          (SAM_USBHS_BASE+SAM_USBHS_DEVDMANXTDSC_OFFSET(n))
#define SAM_USBHS_DEVDMAADDR(n)            (SAM_USBHS_BASE+SAM_USBHS_DEVDMAADDR_OFFSET(n))
#define SAM_USBHS_DEVDMACTRL(n)            (SAM_USBHS_BASE+SAM_USBHS_DEVDMACTRL_OFFSET(n))
#define SAM_USBHS_DEVDMASTA(n)             (SAM_USBHS_BASE+SAM_USBHS_DEVDMASTA_OFFSET(n))

/* USBHS Mini-Host Controller Register Addresses */

#define SAM_USBHS_HSTCTRL                  (SAM_USBHS_BASE+SAM_USBHS_HSTCTRL_OFFSET)
#define SAM_USBHS_HSTISR                   (SAM_USBHS_BASE+SAM_USBHS_HSTISR_OFFSET)
#define SAM_USBHS_HSTICR                   (SAM_USBHS_BASE+SAM_USBHS_HSTICR_OFFSET)
#define SAM_USBHS_HSTIFR                   (SAM_USBHS_BASE+SAM_USBHS_HSTIFR_OFFSET)
#define SAM_USBHS_HSTIMR                   (SAM_USBHS_BASE+SAM_USBHS_HSTIMR_OFFSET)
#define SAM_USBHS_HSTIDR                   (SAM_USBHS_BASE+SAM_USBHS_HSTIDR_OFFSET)
#define SAM_USBHS_HSTIER                   (SAM_USBHS_BASE+SAM_USBHS_HSTIER_OFFSET)
#define SAM_USBHS_HSTPIP                   (SAM_USBHS_BASE+SAM_USBHS_HSTPIP_OFFSET)
#define SAM_USBHS_HSTFNUM                  (SAM_USBHS_BASE+SAM_USBHS_HSTFNUM_OFFSET)
#define SAM_USBHS_HSTADDR1                 (SAM_USBHS_BASE+SAM_USBHS_HSTADDR1_OFFSET)
#define SAM_USBHS_HSTADDR2                 (SAM_USBHS_BASE+SAM_USBHS_HSTADDR2_OFFSET)
#define SAM_USBHS_HSTADDR3                 (SAM_USBHS_BASE+SAM_USBHS_HSTADDR3_OFFSET)

#define SAM_USBHS_HSTPIPCFG(n)             (SAM_USBHS_BASE+SAM_USBHS_HSTPIPCFG_OFFSET(n))
#define SAM_USBHS_HSTPIPISR(n)             (SAM_USBHS_BASE+SAM_USBHS_HSTPIPISR_OFFSET(n))
#define SAM_USBHS_HSTPIPICR(n)             (SAM_USBHS_BASE+SAM_USBHS_HSTPIPICR_OFFSET(n))
#define SAM_USBHS_HSTPIPIFR(n)             (SAM_USBHS_BASE+SAM_USBHS_HSTPIPIFR_OFFSET(n))
#define SAM_USBHS_HSTPIPIMR(n)             (SAM_USBHS_BASE+SAM_USBHS_HSTPIPIMR_OFFSET(n))
#define SAM_USBHS_HSTPIPIER(n)             (SAM_USBHS_BASE+SAM_USBHS_HSTPIPIER_OFFSET(n))
#define SAM_USBHS_HSTPIPIDR(n)             (SAM_USBHS_BASE+SAM_USBHS_HSTPIPIDR_OFFSET(n))
#define SAM_USBHS_HSTPIPINRQ(n)            (SAM_USBHS_BASE+SAM_USBHS_HSTPIPINRQ_OFFSET(n))
#define SAM_USBHS_HSTPIPERR(n)             (SAM_USBHS_BASE+SAM_USBHS_HSTPIPERR_OFFSET(n))

#define SAM_USBHS_HSTDMANXTDSC(n)          (SAM_USBHS_BASE+SAM_USBHS_HSTDMANXTDSC_OFFSET(n))
#define SAM_USBHS_HSTDMAADDR(n)            (SAM_USBHS_BASE+SAM_USBHS_HSTDMAADDR_OFFSET(n))
#define SAM_USBHS_HSTDMACTRL(n)            (SAM_USBHS_BASE+SAM_USBHS_HSTDMACTRL_OFFSET(n))
#define SAM_USBHS_HSTDMASTA(n)             (SAM_USBHS_BASE+SAM_USBHS_HSTDMASTA_OFFSET(n)

/* USBHS General Register Addresses */

#define SAM_USBHS_CTRL                     (SAM_USBHS_BASE+SAM_USBHS_CTRL_OFFSET)
#define SAM_USBHS_SR                       (SAM_USBHS_BASE+SAM_USBHS_SR_OFFSET)
#define SAM_USBHS_SCR                      (SAM_USBHS_BASE+SAM_USBHS_SCR_OFFSET)
#define SAM_USBHS_SFR                      (SAM_USBHS_BASE+SAM_USBHS_SFR_OFFSET)

/* Register bit-field definitions *******************************************/

/* USBHS Device Controller Register Bit Field Definitions */

/* Device General Control Register */

#define USBHS_DEVCTRL_UADD_SHIFT           (0)               /* Bits 0-6: USBHS Address */
#define USBHS_DEVCTRL_UADD_MASK            (0x7f << USBHS_DEVCTRL_UADD_SHIFT)
#  define USBHS_DEVCTRL_UADD(n)            ((n) << USBHS_DEVCTRL_UADD_SHIFT)
#define USBHS_DEVCTRL_ADDEN                (1 << 7)          /* Bit 7:  Function Address Enable */
#define USBHS_DEVCTRL_DETACH               (1 << 8)          /* Bit 8:  Detach Command */
#define USBHS_DEVCTRL_RMWKUP               (1 << 9)          /* Bit 9:  Send Remote Wake Up */
#define USBHS_DEVCTRL_SPDCONF_SHIFT        (10)              /* Bits 10-11:  Mode Configuration */
#define USBHS_DEVCTRL_SPDCONF_MASK         (3 << USBHS_DEVCTRL_SPDCONF_SHIFT)
#  define USBHS_DEVCTRL_SPDCONF_NORMAL     (0 << USBHS_DEVCTRL_SPDCONF_SHIFT)
#  define USBHS_DEVCTRL_SPDCONF_LOWPOWER   (1 << USBHS_DEVCTRL_SPDCONF_SHIFT)
#define USBHS_DEVCTRL_LS                   (1 << 12)         /* Bit 12: Low-Speed Mode Force */
#define USBHS_DEVCTRL_TSTJ                 (1 << 13)         /* Bit 13: Test mode J */
#define USBHS_DEVCTRL_TSTK                 (1 << 14)         /* Bit 14: Test mode K */
#define USBHS_DEVCTRL_TSTPCKT              (1 << 15)         /* Bit 15: Test packet mode */
#define USBHS_DEVCTRL_OPMODE2              (1 << 16)         /* Bit 16: Specific Operational mode */

/* Device Global Interrupt Status Register
 * Device Global Interrupt Clear Register
 * Device Global Interrupt Set Register
 * Device Global Interrupt Mask Register
 * Device Global Interrupt Disable Register
 * Device Global Interrupt Enable Register
 *
 * (1) Not clear or set registers
 * (2) Not clear register
 */

#define USBHS_DEVINT_SUSPD                 (1 << 0)          /* Bit 0:  Suspend Interrupt */
#define USBHS_DEVINT_MSOF                  (1 << 1)          /* Bit 1:  Micro Start Of Frame Interrupt */
#define USBHS_DEVINT_SOF                   (1 << 2)          /* Bit 2:  Start Of Frame Interrupt */
#define USBHS_DEVINT_EORST                 (1 << 3)          /* Bit 3:  End Of Reset Interrupt */
#define USBHS_DEVINT_WAKEUP                (1 << 4)          /* Bit 4:  Wake Up CPU Interrupt */
#define USBHS_DEVINT_EORSM                 (1 << 5)          /* Bit 5:  End Of Resume Interrupt */
#define USBHS_DEVINT_UPRSM                 (1 << 6)          /* Bit 6:  Upstream Resume Interrupt */
#define USBHS_DEVINT_PEP_SHIFT             (12)              /* Bits 12-23: Endpoint interrupts (1) */
#define USBHS_DEVINT_PEP_MASK              (0xfff << USBHS_DEVINT_PEP_SHIFT)
#  define USBHS_DEVINT_PEP(n)              (1 << ((n)+12))   /* Endpoint n Interrupt, n=0-11 (1) */
#  define USBHS_DEVINT_PEP0                (1 << 12)         /* Bit 12: Endpoint 0 Interrupt (1) */
#  define USBHS_DEVINT_PEP1                (1 << 13)         /* Bit 13: Endpoint 1 Interrupt (1) */
#  define USBHS_DEVINT_PEP2                (1 << 14)         /* Bit 14: Endpoint 2 Interrupt (1) */
#  define USBHS_DEVINT_PEP3                (1 << 15)         /* Bit 15: Endpoint 3 Interrupt (1) */
#  define USBHS_DEVINT_PEP4                (1 << 16)         /* Bit 16: Endpoint 4 Interrupt (1) */
#  define USBHS_DEVINT_PEP5                (1 << 17)         /* Bit 17: Endpoint 5 Interrupt (1) */
#  define USBHS_DEVINT_PEP6                (1 << 18)         /* Bit 18: Endpoint 6 Interrupt (1) */
#  define USBHS_DEVINT_PEP7                (1 << 19)         /* Bit 19: Endpoint 7 Interrupt (1) */
#  define USBHS_DEVINT_PEP8                (1 << 20)         /* Bit 20: Endpoint 8 Interrupt (1) */
#  define USBHS_DEVINT_PEP9                (1 << 21)         /* Bit 21: Endpoint 9 Interrupt (1) */
#  define USBHS_DEVINT_PEP10               (1 << 22)         /* Bit 22: Endpoint 10 Interrupt (1) */
#  define USBHS_DEVINT_PEP11               (1 << 23)         /* Bit 23: Endpoint 11 Interrupt (1) */
#define USBHS_DEVINT_DMA_SHIFT             (25)              /* Bits 25-31: DMA channel interrupts (2) */
#define USBHS_DEVINT_DMA_MASK              (0x7f << USBHS_DEVINT_DMA_SHIFT)
#define USBHS_DEVINT_DMA(n)                (1 << ((n)+24))   /* DMA Channel n Interrupt, n=1-7 (2) */
#  define USBHS_DEVINT_DMA1                (1 << 25)         /* Bit 25: DMA Channel 1 Interrupt (2) */
#  define USBHS_DEVINT_DMA2                (1 << 26)         /* Bit 26: DMA Channel 2 Interrupt (2) */
#  define USBHS_DEVINT_DMA3                (1 << 27)         /* Bit 27: DMA Channel 3 Interrupt (2) */
#  define USBHS_DEVINT_DMA4                (1 << 28)         /* Bit 28: DMA Channel 4 Interrupt (2) */
#  define USBHS_DEVINT_DMA5                (1 << 29)         /* Bit 29: DMA Channel 5 Interrupt (2) */
#  define USBHS_DEVINT_DMA6                (1 << 30)         /* Bit 30: DMA Channel 6 Interrupt (2) */
#  define USBHS_DEVINT_DMA7                (1 << 31)         /* Bit 31: DMA Channel 7 Interrupt (2) */

#define USBHS_DEVINT_ALL                   0xfefff07f

/* Device Endpoint Register */

#define USBHS_DEVEPT_EPEN(n)               (1 << ((n)))      /* Endpoint n Enable */
#  define USBHS_DEVEPT_EPEN0               (1 << 0)          /* Bit 0:  Endpoint 0 Enable */
#  define USBHS_DEVEPT_EPEN1               (1 << 1)          /* Bit 1:  Endpoint 1 Enable */
#  define USBHS_DEVEPT_EPEN2               (1 << 2)          /* Bit 2:  Endpoint 2 Enable */
#  define USBHS_DEVEPT_EPEN3               (1 << 3)          /* Bit 3:  Endpoint 3 Enable */
#  define USBHS_DEVEPT_EPEN4               (1 << 4)          /* Bit 4:  Endpoint 4 Enable */
#  define USBHS_DEVEPT_EPEN5               (1 << 5)          /* Bit 5:  Endpoint 5 Enable */
#  define USBHS_DEVEPT_EPEN6               (1 << 6)          /* Bit 6:  Endpoint 6 Enable */
#  define USBHS_DEVEPT_EPEN7               (1 << 7)          /* Bit 7:  Endpoint 7 Enable */
#  define USBHS_DEVEPT_EPEN8               (1 << 8)          /* Bit 8:  Endpoint 8 Enable */
#  define USBHS_DEVEPT_EPEN9               (1 << 9)          /* Bit 9:  Endpoint 9 Enable */
#  define USBHS_DEVEPT_ALLEPEN             0x000003ff
#define USBHS_DEVEPT_EPRST(n)              (1 << ((n)+16))   /* Endpoint n Reset */
#  define USBHS_DEVEPT_EPRST0              (1 << 16)         /* Bit 16:  Endpoint 0 Reset */
#  define USBHS_DEVEPT_EPRST1              (1 << 17)         /* Bit 17:  Endpoint 1 Reset */
#  define USBHS_DEVEPT_EPRST2              (1 << 18)         /* Bit 18:  Endpoint 2 Reset */
#  define USBHS_DEVEPT_EPRST3              (1 << 19)         /* Bit 19:  Endpoint 3 Reset */
#  define USBHS_DEVEPT_EPRST4              (1 << 20)         /* Bit 20:  Endpoint 4 Reset */
#  define USBHS_DEVEPT_EPRST5              (1 << 21)         /* Bit 21:  Endpoint 5 Reset */
#  define USBHS_DEVEPT_EPRST6              (1 << 22)         /* Bit 22:  Endpoint 6 Reset */
#  define USBHS_DEVEPT_EPRST7              (1 << 23)         /* Bit 23:  Endpoint 7 Reset */
#  define USBHS_DEVEPT_EPRST8              (1 << 24)         /* Bit 24:  Endpoint 8 Reset */
#  define USBHS_DEVEPT_EPRST9              (1 << 25)         /* Bit 25:  Endpoint 9 Reset */
#  define USBHS_DEVEPT_ALLEPRST            0x03ff0000

/* Device Frame Number Register */

#define USBHS_DEVFNUM_MFNUM_SHIFT          (0)               /* Bits 0-2: Microframe Number */
#define USBHS_DEVFNUM_MFNUM_MASK           (7 << USBHS_DEVFNUM_MFNUM_SHIFT)
#define USBHS_DEVFNUM_FNUM_SHIFT           (3)               /* Bits 3-13: Frame Number */
#define USBHS_DEVFNUM_FNUM_MASK            (0x7ff << USBHS_DEVFNUM_FNUM_SHIFT)
#define USBHS_DEVFNUM_FNCERR               (1 << 15)         /* Bit 15: Frame Number CRC Error */

/* Device Endpoint Configuration Register */

#define USBHS_DEVEPTCFG_ALLOC              (1 << 1)          /* Bit 1: Endpoint Memory Allocate */
#define USBHS_DEVEPTCFG_EPBK_SHIFT         (2)               /* Bits 2-3: Endpoint Banks */
#define USBHS_DEVEPTCFG_EPBK_MASK          (3 << USBHS_DEVEPTCFG_EPBK_SHIFT)
#  define USBHS_DEVEPTCFG_EPBK(n)          ((uint32_t)((n)-1) << USBHS_DEVEPTCFG_EPBK_SHIFT)
#  define USBHS_DEVEPTCFG_EPBK_1BANK       (0 << USBHS_DEVEPTCFG_EPBK_SHIFT) /* Single-bank endpoint */
#  define USBHS_DEVEPTCFG_EPBK_2BANK       (1 << USBHS_DEVEPTCFG_EPBK_SHIFT) /* Double-bank endpoint */
#  define USBHS_DEVEPTCFG_EPBK_3BANK       (2 << USBHS_DEVEPTCFG_EPBK_SHIFT) /* Triple-bank endpoint */

#define USBHS_DEVEPTCFG_EPSIZE_SHIFT       (4)               /* Bits 4-6: Endpoint Size */
#define USBHS_DEVEPTCFG_EPSIZE_MASK        (7 << USBHS_DEVEPTCFG_EPSIZE_SHIFT)
#  define USBHS_DEVEPTCFG_EPSIZE_8         (0 << USBHS_DEVEPTCFG_EPSIZE_SHIFT) /* 8 bytes */
#  define USBHS_DEVEPTCFG_EPSIZE_16        (1 << USBHS_DEVEPTCFG_EPSIZE_SHIFT) /* 16 bytes */
#  define USBHS_DEVEPTCFG_EPSIZE_32        (2 << USBHS_DEVEPTCFG_EPSIZE_SHIFT) /* 32 bytes */
#  define USBHS_DEVEPTCFG_EPSIZE_64        (3 << USBHS_DEVEPTCFG_EPSIZE_SHIFT) /* 64 bytes  */
#  define USBHS_DEVEPTCFG_EPSIZE_128       (4 << USBHS_DEVEPTCFG_EPSIZE_SHIFT) /* 128 bytes */
#  define USBHS_DEVEPTCFG_EPSIZE_256       (5 << USBHS_DEVEPTCFG_EPSIZE_SHIFT) /* 256 bytes */
#  define USBHS_DEVEPTCFG_EPSIZE_512       (6 << USBHS_DEVEPTCFG_EPSIZE_SHIFT) /* 512 bytes */
#  define USBHS_DEVEPTCFG_EPSIZE_1024      (7 << USBHS_DEVEPTCFG_EPSIZE_SHIFT) /* 1024 bytes */

#define USBHS_DEVEPTCFG_EPDIR_SHIFT        (8)               /* Bit 8:  Endpoint Direction */
#define USBHS_DEVEPTCFG_EPDIR_MASK         (1 << 8)          /* Bit 8:  Endpoint Direction */
#  define USBHS_DEVEPTCFG_EPDIR(n)         ((uint32_t)(n) << 8)
#define USBHS_DEVEPTCFG_AUTOSW             (1 << 9)          /* Bit 9: Automatic Switch */
#define USBHS_DEVEPTCFG_EPTYPE_SHIFT       (11)              /* Bits 11-12: Endpoint Type */
#define USBHS_DEVEPTCFG_EPTYPE_MASK        (3 << USBHS_DEVEPTCFG_EPTYPE_SHIFT)
#  define USBHS_DEVEPTCFG_EPTYPE(n)        ((uint32_t)(n) << USBHS_DEVEPTCFG_EPTYPE_SHIFT)
#  define USBHS_DEVEPTCFG_EPTYPE_CTRL      (0 << USBHS_DEVEPTCFG_EPTYPE_SHIFT) /* Control endpoint */
#  define USBHS_DEVEPTCFG_EPTYPE_ISO       (1 << USBHS_DEVEPTCFG_EPTYPE_SHIFT) /* Isochronous endpoint */
#  define USBHS_DEVEPTCFG_EPTYPE_BLK       (2 << USBHS_DEVEPTCFG_EPTYPE_SHIFT) /* Bulk endpoint */
#  define USBHS_DEVEPTCFG_EPTYPE_INTRPT    (3 << USBHS_DEVEPTCFG_EPTYPE_SHIFT) /* Interrupt endpoint */

#define USBHS_DEVEPTCFG_NBTRANS_SHIFT      (13)              /* Bits 13-14: Number Transaction per uframe */
#define USBHS_DEVEPTCFG_NBTRANS_MASK       (3 << USBHS_DEVEPTCFG_NBTRANS_SHIFT)
#  define USBHS_DEVEPTCFG_NBTRANS(n)       ((uint32_t)(n) << USBHS_DEVEPTCFG_NBTRANS_SHIFT)

/* Common Endpoint Interrupt Bit Definitions
 *
 *   Device Endpoint Status Register
 *   Device Endpoint Clear Register
 *   Device Endpoint Set Register
 *   Device Endpoint Mask Register
 *   Device Endpoint Enable Register
 *
 * (1) Control, Bulk, Interrupt endpoints
 * (2) Isochronous endpoints only
 */

#define USBHS_DEVEPTINT_TXINI              (1 << 0)          /* Bit 0: Transmitted IN Data Interrupt */
#define USBHS_DEVEPTINT_RXOUTI             (1 << 1)          /* Bit 1: Received OUT Data Interrupt */
#define USBHS_DEVEPTINT_RXSTPI             (1 << 2)          /* Bit 2: Received SETUP Interrupt (2) */
#define USBHS_DEVEPTINT_UNDERFI            (1 << 2)          /* Bit 2: Underflow Interrupt (3) */
#define USBHS_DEVEPTINT_NAKOUTI            (1 << 3)          /* Bit 3: NAKed OUT Interrupt (2) */
#define USBHS_DEVEPTINT_HBISOINERRI        (1 << 3)          /* Bit 3: High Bandwidth Isochronous IN Underflow Error Interrupt (3) */
#define USBHS_DEVEPTINT_NAKINI             (1 << 4)          /* Bit 4: NAKed IN Interrupt (2) */
#define USBHS_DEVEPTINT_HBISOFLUSHI        (1 << 4)          /* Bit 4: High Bandwidth Isochronous IN Flush Interrupt (3) */
#define USBHS_DEVEPTINT_OVERFI             (1 << 5)          /* Bit 5: Overflow Interrupt */
#define USBHS_DEVEPTINT_STALLEDI           (1 << 6)          /* Bit 6: STALLed Interrupt (2) */
#define USBHS_DEVEPTINT_CRCERRI            (1 << 6)          /* Bit 6: CRC Error Interrupt (3) */
#define USBHS_DEVEPTINT_SHRTPCKTI          (1 << 7)          /* Bit 7: Short Packet Interrupt */

/* Device Endpoint Mask, Device Endpoint Disable,
 * and Device Endpoint Enable Registers only
 */

#define USBHS_DEVEPTINT_MDATAI             (1 << 8)          /* Bit 8:  MData Interrupt (2) */
#define USBHS_DEVEPTINT_DATAXI             (1 << 9)          /* Bit 9:  DataX Interrupt (2) */
#define USBHS_DEVEPTINT_ERRORTRANSI        (1 << 10)         /* Bit 10: Transaction Error Interrupt (2) */

/* Device Endpoint Set, Device Endpoint Mask, Device Endpoint Disable,
 * and Device Endpoint Enable Registers only
 */

#define USBHS_DEVEPTINT_NBUSYBKI           (1 << 12)         /* Bit 12: Number of Busy Banks Interrupt */

/* Device Endpoint Mask and Device Endpoint Enable Registers only */

#define USBHS_DEVEPTINT_KILLBKI            (1 << 13)         /* Bit 13: Kill IN Bank */

/* Device Endpoint Mask, Device Endpoint Disable,
 * and Device Endpoint Enable Registers only
 */

#define USBHS_DEVEPTINT_FIFOCONI           (1 << 14)         /* Bit 14: FIFO Control */
#define USBHS_DEVEPTINT_EPDISHDMAI         (1 << 16)         /* Bit 16: Endpoint Interrupts Disable HDMA Request */
#define USBHS_DEVEPTINT_NYETDISI           (1 << 17)         /* Bit 17: NYET Token Disable (1) */
#define USBHS_DEVEPTINT_RSTDTI             (1 << 18)         /* Bit 18: Reset Data Toggle */
#define USBHS_DEVEPTINT_STALLRQI           (1 << 19)         /* Bit 19: STALL Request (1) */

#define USBHS_DEVEPTICR_ALLINTS            0x000000ff
#define USBHS_DEVEPTIFR_ALLINTS            0x000010ff
#define USBHS_DEVEPTIDR_ALLINTS            0x000d57ff
#define USBHS_DEVEPTIER_ALLINTS            0x000f77ff

/* Device Endpoint Status Register only */

#define USBHS_DEVEPTISR_DTSEQ_SHIFT        (8)               /* Bits 8-9: Data Toggle Sequence */
#define USBHS_DEVEPTISR_DTSEQ_MASK         (3 << USBHS_DEVEPTISR_DTSEQ_SHIFT)
#  define USBHS_DEVEPTISR_DTSEQ_DATA0      (0 << USBHS_DEVEPTISR_DTSEQ_SHIFT) /* Data0 toggle sequence */
#  define USBHS_DEVEPTISR_DTSEQ_DATA1      (1 << USBHS_DEVEPTISR_DTSEQ_SHIFT) /* Data1 toggle sequence */
#  define USBHS_DEVEPTISR_DTSEQ_DATA2      (2 << USBHS_DEVEPTISR_DTSEQ_SHIFT) /* Data1 toggle sequence (2) */
#  define USBHS_DEVEPTISR_DTSEQ_MDATA      (3 << USBHS_DEVEPTISR_DTSEQ_SHIFT) /* MData toggle sequence (2) */

#define USBHS_DEVEPTISR_ERRORTRANS         (1 << 10)         /* Bit 10: High-bandwidth Isochronous OUT Endpoint Transaction Error Interrupt (2) */
#define USBHS_DEVEPTISR_NBUSYBK_SHIFT      (12)              /* Bits 12-13:  Number of Busy Banks */
#define USBHS_DEVEPTISR_NBUSYBK_MASK       (3 << USBHS_DEVEPTISR_NBUSYBK_SHIFT)
#  define USBHS_DEVEPTISR_NBUSYBK_0BUSY    (0 << USBHS_DEVEPTISR_NBUSYBK_SHIFT) /* 0 busy bank (all banks free) */
#  define USBHS_DEVEPTISR_NBUSYBK_1BUSY    (1 << USBHS_DEVEPTISR_NBUSYBK_SHIFT) /* 1 busy bank */
#  define USBHS_DEVEPTISR_NBUSYBK_2BUSY    (2 << USBHS_DEVEPTISR_NBUSYBK_SHIFT) /* 2 busy banks */
#  define USBHS_DEVEPTISR_NBUSYBK_3BUSY    (3 << USBHS_DEVEPTISR_NBUSYBK_SHIFT) /* 3 busy banks */

#define USBHS_DEVEPTISR_CURRBK_SHIFT       (14)              /* Bits 14-15: Current Bank */
#define USBHS_DEVEPTISR_CURRBK_MASK        (3 << USBHS_DEVEPTISR_CURRBK_SHIFT)
#  define USBHS_DEVEPTISR_CURRBK_BANK0     (0 << USBHS_DEVEPTISR_CURRBK_SHIFT) /* Current bank is bank0 */
#  define USBHS_DEVEPTISR_CURRBK_BANK1     (1 << USBHS_DEVEPTISR_CURRBK_SHIFT) /* Current bank is bank1 */
#  define USBHS_DEVEPTISR_CURRBK_BANK2     (2 << USBHS_DEVEPTISR_CURRBK_SHIFT) /* Current bank is bank2 */

#define USBHS_DEVEPTISR_RWALL              (1 << 16)         /* Bit 16:  Read/Write Allowed */
#define USBHS_DEVEPTISR_CTRLDIR            (1 << 17)         /* Bit 17:  Control Direction (1) */
#  define USBHS_DEVEPTISR_CTRLDIR_OUT      (0 << 17)         /*   0=Following packet is an OUT packet */
#  define USBHS_DEVEPTISR_CTRLDIR_IN       (1 << 17)         /*   1=Following packet is an IN packet */
#define USBHS_DEVEPTISR_CFGOK              (1 << 18)         /* Bit 18:  Configuration OK Status */
#define USBHS_DEVEPTISR_BYCT_SHIFT         (20)              /* Bits 20-30: USBHS Byte Count */
#define USBHS_DEVEPTISR_BYCT_MASK          (0x7ff << USBHS_DEVEPTISR_BYCT_SHIFT)

/* Device DMA Channel Next Descriptor Address Register
 * (32-bit, 16 byte aligned address)
 */

/* Device DMA Channel Address Register (32-bit address) */

/* Device DMA Channel Control Register */

#define USBHS_DEVDMACTRL_CMD_SHIFT         (0)               /* Bits 0-1: Command */
#define USBHS_DEVDMACTRL_CMD_MASK          (3 << USBHS_DEVDMACTRL_CMD_SHIFT)
#  define USBHS_DEVDMACTRL_CHANNENB        (1 << 0)          /* Bit 0:  Channel Enable Command */
#  define USBHS_DEVDMACTRL_LDNXTDSC        (1 << 1)          /* Bit 1:  Load Next Channel Transfer Descriptor Enable Command */

#  define USBHS_DEVDMACTRL_STOPNOW         (0 << USBHS_DEVDMACTRL_CMD_SHIFT) /* Stop now */
#  define USBHS_DEVDMACTRL_RUNSTOP         (1 << USBHS_DEVDMACTRL_CMD_SHIFT) /* Run and stop at end of buffer */
#  define USBHS_DEVDMACTRL_LOADNEXT        (2 << USBHS_DEVDMACTRL_CMD_SHIFT) /* Load next descriptor now */
#  define USBHS_DEVDMACTRL_RUNLINK         (3 << USBHS_DEVDMACTRL_CMD_SHIFT) /* Run and link at end of buffer */

#define USBHS_DEVDMACTRL_ENDTREN           (1 << 2)          /* Bit 2:  End of Transfer Enable Control */
#define USBHS_DEVDMACTRL_ENDBEN            (1 << 3)          /* Bit 3:  End of Buffer Enable Control */
#define USBHS_DEVDMACTRL_ENDTRIT           (1 << 4)          /* Bit 4:  End of Transfer Interrupt Enable */
#define USBHS_DEVDMACTRL_ENDBUFFIT         (1 << 5)          /* Bit 5:  End of Buffer Interrupt Enable */
#define USBHS_DEVDMACTRL_DESCLDIT          (1 << 6)          /* Bit 6:  Descriptor Loaded Interrupt Enable */
#define USBHS_DEVDMACTRL_BURSTLCK          (1 << 7)          /* Bit 7:  Burst Lock Enable */
#define USBHS_DEVDMACTRL_BUFLEN_SHIFT      (16)              /* Bits 16-31: Buffer Byte Length  */
#define USBHS_DEVDMACTRL_BUFLEN_MASK       (0xffff << USBHS_DEVDMACTRL_BUFLEN_SHIFT)
#  define USBHS_DEVDMACTRL_BUFLEN(n)       ((uint32_t)(n) << USBHS_DEVDMACTRL_BUFLEN_SHIFT)

/* Device DMA Channel Status Register */

#define USBHS_DEVDMASTA_CHANNENB           (1 << 0)          /* Bit 0:  Channel Enable Status */
#define USBHS_DEVDMASTA_CHANNACT           (1 << 1)          /* Bit 1:  Channel Active Status */
#define USBHS_DEVDMASTA_ENDTRST            (1 << 4)          /* Bit 4:  End of Transfer Status */
#define USBHS_DEVDMASTA_ENDBUFFST          (1 << 5)          /* Bit 5:  End of Buffer Status */
#define USBHS_DEVDMASTA_DESCLDST           (1 << 6)          /* Bit 6:  Descriptor Loaded Status */
#define USBHS_DEVDMASTA_BUFCNT_SHIFT       (16)              /* Bits 16-31: Buffer Byte Count */
#define USBHS_DEVDMASTA_BUFCNT_MASK        (0xffff << USBHS_DEVDMASTA_BUFCNT_SHIFT)
#  define USBHS_DEVDMASTA_BUFCNT(n)        ((uint32_t)(n) << USBHS_DEVDMASTA_BUFCNT_SHIFT)

/* USBHS Mini-Host Controller Register Bit Field Definitions */

/* Host General Control Register */

#define USBHS_HSTCTRL_SOFE                 (1 << 8)          /* Bit 8:  Start of Frame Generation Enable */
#define USBHS_HSTCTRL_RESET                (1 << 9)          /* Bit 9:  Send USB Reset */
#define USBHS_HSTCTRL_RESUME               (1 << 10)         /* Bit 10: Send USB Resume */
#define USBHS_HSTCTRL_SPDCONF              (12)              /* Bits 12-13: Mode Configuration */
#define USBHS_HSTCTRL_MASK                 (3 << USBHS_HSTCTRL_SPDCONF)
#  define USBHS_HSTCTRL_NORMAL             (0 << USBHS_HSTCTRL_SPDCONF)
#  define USBHS_HSTCTRL_LOWPOWER           (1 << USBHS_HSTCTRL_SPDCONF)

/* Host Global Interrupt Status Register
 * Host Global Interrupt Clear Register
 * Host Global Interrupt Set Register
 * Host Global Interrupt Mask Register
 * Host Global Interrupt Disable Register
 * Host Global Interrupt Enable Register
 *
 * (1) Not clear or set registers
 * (2) Not clear register
 */

#define USBHS_HSTINT_DCONNI                (1 << 0)          /* Bit 0:  Device Connection Interrupt */
#define USBHS_HSTINT_DDISCI                (1 << 1)          /* Bit 1:  Device Disconnection Interrupt */
#define USBHS_HSTINT_RSTI                  (1 << 2)          /* Bit 2:  USB Reset Sent Interrupt */
#define USBHS_HSTINT_RSMEDI                (1 << 3)          /* Bit 3:  Downstream Resume Sent Interrupt */
#define USBHS_HSTINT_RXRSMI                (1 << 4)          /* Bit 4:  Upstream Resume Received Interrupt */
#define USBHS_HSTINT_HSOFI                 (1 << 5)          /* Bit 5:  Host Start of Frame Interrupt */
#define USBHS_HSTINT_HWUPI                 (1 << 6)          /* Bit 6:  Host Wake-Up Interrupt */
#define USBHS_HSTINT_PEP_SHIFT             (12)              /* Bits 12-23: Pipe interrupts (1) */
#define USBHS_HSTINT_PEP_MASK              (0xfff << USBHS_HSTINT_PEP_SHIFT)
#  define USBHS_HSTINT_PEP(n)              (1 << +((n)+12))  /* Pipe n Interrupt, n=0-11 (1) */
#  define USBHS_HSTINT_PEP0                (1 << 12)         /* Bit 12: Pipe 0 Interrupt (1) */
#  define USBHS_HSTINT_PEP1                (1 << 13)         /* Bit 13: Pipe 1 Interrupt (1) */
#  define USBHS_HSTINT_PEP2                (1 << 14)         /* Bit 14: Pipe 2 Interrupt (1) */
#  define USBHS_HSTINT_PEP3                (1 << 15)         /* Bit 15: Pipe 3 Interrupt (1) */
#  define USBHS_HSTINT_PEP4                (1 << 16)         /* Bit 16: Pipe 4 Interrupt (1) */
#  define USBHS_HSTINT_PEP5                (1 << 17)         /* Bit 17: Pipe 5 Interrupt (1) */
#  define USBHS_HSTINT_PEP6                (1 << 18)         /* Bit 18: Pipe 6 Interrupt (1) */
#  define USBHS_HSTINT_PEP7                (1 << 19)         /* Bit 19: Pipe 7 Interrupt (1) */
#  define USBHS_HSTINT_PEP8                (1 << 20)         /* Bit 20: Pipe 8 Interrupt (1) */
#  define USBHS_HSTINT_PEP9                (1 << 21)         /* Bit 21: Pipe 9 Interrupt (1) */
#  define USBHS_HSTINT_PEP10               (1 << 22)         /* Bit 22: Pipe 10 Interrupt (1) */
#  define USBHS_HSTINT_PEP11               (1 << 23)         /* Bit 23: Pipe 11 Interrupt (1) */
#define USBHS_HSTINT_DMA_SHIFT             (25)              /* Bits 25-31: DMA channel interrupts (2) */
#define USBHS_HSTINT_DMA_MASK              (0x7f << USBHS_HSTINT_DMA_SHIFT)
#define USBHS_HSTINT_DMA(n)                (1 << ((n)+24))   /* DMA Channel n Interrupt, n=1-7 (2) */
#  define USBHS_HSTINT_DMA1                (1 << 25)         /* Bit 25: DMA Channel 1 Interrupt (2) */
#  define USBHS_HSTINT_DMA2                (1 << 26)         /* Bit 26: DMA Channel 2 Interrupt (2) */
#  define USBHS_HSTINT_DMA3                (1 << 27)         /* Bit 27: DMA Channel 3 Interrupt (2) */
#  define USBHS_HSTINT_DMA4                (1 << 28)         /* Bit 28: DMA Channel 4 Interrupt (2) */
#  define USBHS_HSTINT_DMA5                (1 << 29)         /* Bit 29: DMA Channel 5 Interrupt (2) */
#  define USBHS_HSTINT_DMA6                (1 << 30)         /* Bit 30: DMA Channel 6 Interrupt (2) */
#  define USBHS_HSTINT_DMA7                (1 << 31)         /* Bit 31: DMA Channel 7 Interrupt (2) */

/* Host Pipe Register */

#define USBHS_HSTPIP_PEN(n)                (1 << ((n)))      /* Pipe n Enable */
#  define USBHS_HSTPIP_PEN0                (1 << 0)          /* Bit 0:  Pipe 0 Enable */
#  define USBHS_HSTPIP_PEN1                (1 << 1)          /* Bit 1:  Pipe 1 Enable */
#  define USBHS_HSTPIP_PEN2                (1 << 2)          /* Bit 2:  Pipe 2 Enable */
#  define USBHS_HSTPIP_PEN3                (1 << 3)          /* Bit 3:  Pipe 3 Enable */
#  define USBHS_HSTPIP_PEN4                (1 << 4)          /* Bit 4:  Pipe 4 Enable */
#  define USBHS_HSTPIP_PEN5                (1 << 5)          /* Bit 5:  Pipe 5 Enable */
#  define USBHS_HSTPIP_PEN6                (1 << 6)          /* Bit 6:  Pipe 6 Enable */
#  define USBHS_HSTPIP_PEN7                (1 << 7)          /* Bit 7:  Pipe 7 Enable */
#  define USBHS_HSTPIP_PEN8                (1 << 8)          /* Bit 8:  Pipe 8 Enable */
#  define USBHS_HSTPIP_PEN9                (1 << 9)          /* Bit 9:  Pipe 9 Enable */
#define USBHS_HSTPIP_PRST(n)               (1 << ((n)+16))   /* Pipe n Reset */
#  define USBHS_HSTPIP_PRST0               (1 << 16)         /* Bit 16:  Pipe 0 Reset */
#  define USBHS_HSTPIP_PRST1               (1 << 17)         /* Bit 17:  Pipe 1 Reset */
#  define USBHS_HSTPIP_PRST2               (1 << 18)         /* Bit 18:  Pipe 2 Reset */
#  define USBHS_HSTPIP_PRST3               (1 << 19)         /* Bit 19:  Pipe 3 Reset */
#  define USBHS_HSTPIP_PRST4               (1 << 20)         /* Bit 20:  Pipe 4 Reset */
#  define USBHS_HSTPIP_PRST5               (1 << 21)         /* Bit 21:  Pipe 5 Reset */
#  define USBHS_HSTPIP_PRST6               (1 << 22)         /* Bit 22:  Pipe 6 Reset */
#  define USBHS_HSTPIP_PRST7               (1 << 23)         /* Bit 23:  Pipe 7 Reset */
#  define USBHS_HSTPIP_PRST8               (1 << 24)         /* Bit 24:  Pipe 8 Reset */
#  define USBHS_HSTPIP_PRST9               (1 << 25)         /* Bit 25:  Pipe 9 Reset */

/* Host Frame Number Register */

#define USBHS_HSTFNUM_MFNUM_SHIFT          (0)               /* Bits 0-2: Microframe Number */
#define USBHS_HSTFNUM_MFNUM_MASK           (7 << USBHS_HSTFNUM_MFNUM_SHIFT)
#define USBHS_HSTFNUM_FNUM_SHIFT           (3)               /* Bits 3-13: Frame Number */
#define USBHS_HSTFNUM_FNUM_MASK            (0x7ff << USBHS_HSTFNUM_FNUM_SHIFT)
#define USBHS_HSTFNUM_FLENHIGH_SHIFT       (16)              /* Bits 16-23: Frame Length */
#define USBHS_HSTFNUM_FLENHIGH_MASK        (0xff << USBHS_HSTFNUM_FLENHIGH_SHIFT)
#  define USBHS_HSTFNUM_FLENHIGH(n)        ((uint32_t)(n) << USBHS_HSTFNUM_FLENHIGH_SHIFT)

/* Host Address 1 Register */

#define USBHS_HSTADDR1_HSTADDRP_SHIFT(n)   ((n) << 3)         /* USB Host Address */
#define USBHS_HSTADDR1_HSTADDRP_MASK(n)    (0x7f << USBHS_HSTADDR1_HSTADDRP_SHIFT(n))
#  define USBHS_HSTADDR1_HSTADDRP(n,v)     ((uint32_t)(v) << USBHS_HSTADDR1_HSTADDRP_SHIFT(n))

#define USBHS_HSTADDR1_HSTADDRP0_SHIFT     (0)               /* Bits 0-6: USB Host Address */
#define USBHS_HSTADDR1_HSTADDRP0_MASK      (0x7f << USBHS_HSTADDR1_HSTADDRP0_SHIFT)
#  define USBHS_HSTADDR1_HSTADDRP0(n)      (0x7f << USBHS_HSTADDR1_HSTADDRP0_SHIFT)
#define USBHS_HSTADDR1_HSTADDRP1_SHIFT     (8)               /* Bits 8-14: USB Host Address */
#define USBHS_HSTADDR1_HSTADDRP1_MASK      (0x7f << USBHS_HSTADDR1_HSTADDRP1_SHIFT)
#  define USBHS_HSTADDR1_HSTADDRP1(n)      ((uint32_t)(n) << USBHS_HSTADDR1_HSTADDRP1_SHIFT)
#define USBHS_HSTADDR1_HSTADDRP2_SHIFT     (16)              /* Bits 16-22: USB Host Address */
#define USBHS_HSTADDR1_HSTADDRP2_MASK      (0x7f << USBHS_HSTADDR1_HSTADDRP2_SHIFT)
#  define USBHS_HSTADDR1_HSTADDRP2(n)      ((uint32_t)(n) << USBHS_HSTADDR1_HSTADDRP2_SHIFT)
#define USBHS_HSTADDR1_HSTADDRP3_SHIFT     (24)              /* Bits 24-30: USB Host Address */
#define USBHS_HSTADDR1_HSTADDRP3_MASK      (0x7f << USBHS_HSTADDR1_HSTADDRP3_SHIFT)
#  define USBHS_HSTADDR1_HSTADDRP3(n)      ((uint32_t)(n) << USBHS_HSTADDR1_HSTADDRP3_SHIFT)

/* Host Address 2 Register */

#define USBHS_HSTADDR2_HSTADDRP_SHIFT(n)   (((n)-4) << 3)         /* USB Host Address */
#define USBHS_HSTADDR2_HSTADDRP_MASK(n)    (0x7f << USBHS_HSTADDR2_HSTADDRP_SHIFT(n))
#  define USBHS_HSTADDR2_HSTADDRP(n,v)     ((uint32_t)(v) << USBHS_HSTADDR2_HSTADDRP_SHIFT(n))

#define USBHS_HSTADDR2_HSTADDRP4_SHIFT     (0)               /* Bits 0-6: USB Host Address */
#define USBHS_HSTADDR2_HSTADDRP4_MASK      (0x7f << USBHS_HSTADDR2_HSTADDRP4_SHIFT)
#  define USBHS_HSTADDR2_HSTADDRP4(n)      (0x7f << USBHS_HSTADDR2_HSTADDRP4_SHIFT)
#define USBHS_HSTADDR2_HSTADDRP5_SHIFT     (8)               /* Bits 8-14: USB Host Address */
#define USBHS_HSTADDR2_HSTADDRP5_MASK      (0x7f << USBHS_HSTADDR2_HSTADDRP5_SHIFT)
#  define USBHS_HSTADDR2_HSTADDRP5(n)      ((uint32_t)(n) << USBHS_HSTADDR2_HSTADDRP5_SHIFT)
#define USBHS_HSTADDR2_HSTADDRP6_SHIFT     (16)              /* Bits 16-22: USB Host Address */
#define USBHS_HSTADDR2_HSTADDRP6_MASK      (0x7f << USBHS_HSTADDR2_HSTADDRP6_SHIFT)
#  define USBHS_HSTADDR2_HSTADDRP6(n)      ((uint32_t)(n) << USBHS_HSTADDR2_HSTADDRP6_SHIFT)
#define USBHS_HSTADDR2_HSTADDRP7_SHIFT     (24)              /* Bits 24-30: USB Host Address */
#define USBHS_HSTADDR2_HSTADDRP7_MASK      (0x7f << USBHS_HSTADDR2_HSTADDRP7_SHIFT)
#  define USBHS_HSTADDR2_HSTADDRP7(n)      ((uint32_t)(n) << USBHS_HSTADDR2_HSTADDRP7_SHIFT)

/* Host Address 3 Register */

#define USBHS_HSTADDR3_HSTADDRP_SHIFT(n)   (((n)-8) << 3)         /* USB Host Address */
#define USBHS_HSTADDR3_HSTADDRP_MASK(n)    (0x7f << USBHS_HSTADDR3_HSTADDRP_SHIFT(n))
#  define USBHS_HSTADDR3_HSTADDRP(n,v)     ((uint32_t)(v) << USBHS_HSTADDR3_HSTADDRP_SHIFT(n))

#define USBHS_HSTADDR3_HSTADDRP8_SHIFT     (0)               /* Bits 0-6: USB Host Address */
#define USBHS_HSTADDR3_HSTADDRP8_MASK      (0x7f << USBHS_HSTADDR3_HSTADDRP8_SHIFT)
#  define USBHS_HSTADDR3_HSTADDRP8(n)      (0x7f << USBHS_HSTADDR3_HSTADDRP8_SHIFT)
#define USBHS_HSTADDR3_HSTADDRP9_SHIFT     (8)               /* Bits 8-14: USB Host Address */
#define USBHS_HSTADDR3_HSTADDRP9_MASK      (0x7f << USBHS_HSTADDR3_HSTADDRP9_SHIFT)
#  define USBHS_HSTADDR3_HSTADDRP9(n)      ((uint32_t)(n) << USBHS_HSTADDR3_HSTADDRP9_SHIFT)

/* Host Pipe Configuration Register
 *
 * (1) Not for High-Speed Bulk OUT
 * (2) For High-Speed Bulk Out
 */

#define USBHS_HSTPIPCFG_ALLOC              (1 << 0)          /* Bit 0: Pipe Memory Allocate */
#define USBHS_HSTPIPCFG_PBK_SHIFT          (2)               /* Bits 2-3: Pipe Banks */
#define USBHS_HSTPIPCFG_PBK_MASK           (3 << USBHS_HSTPIPCFG_PBK_SHIFT)
#  define USBHS_HSTPIPCFG_PBK_1BANK        (0 << USBHS_HSTPIPCFG_PBK_SHIFT) /* Single-bank pipe */
#  define USBHS_HSTPIPCFG_PBK_2BANK        (1 << USBHS_HSTPIPCFG_PBK_SHIFT) /* Double-bank pipe */
#  define USBHS_HSTPIPCFG_PBK_3BANK        (2 << USBHS_HSTPIPCFG_PBK_SHIFT) /* Triple-bank pipe */

#define USBHS_HSTPIPCFG_PSIZE_SHIFT        (4)               /* Bits 4-6: Pipe Size */
#define USBHS_HSTPIPCFG_PSIZE_MASK         (7 << USBHS_HSTPIPCFG_PSIZE_SHIFT)
#  define USBHS_HSTPIPCFG_PSIZE_8          (0 << USBHS_HSTPIPCFG_PSIZE_SHIFT) /* 8 bytes */
#  define USBHS_HSTPIPCFG_PSIZE_16         (1 << USBHS_HSTPIPCFG_PSIZE_SHIFT) /* 16 bytes */
#  define USBHS_HSTPIPCFG_PSIZE_32         (2 << USBHS_HSTPIPCFG_PSIZE_SHIFT) /* 32 bytes */
#  define USBHS_HSTPIPCFG_PSIZE_64         (3 << USBHS_HSTPIPCFG_PSIZE_SHIFT) /* 64 bytes  */
#  define USBHS_HSTPIPCFG_PSIZE_128        (4 << USBHS_HSTPIPCFG_PSIZE_SHIFT) /* 128 bytes */
#  define USBHS_HSTPIPCFG_PSIZE_256        (5 << USBHS_HSTPIPCFG_PSIZE_SHIFT) /* 256 bytes */
#  define USBHS_HSTPIPCFG_PSIZE_512        (6 << USBHS_HSTPIPCFG_PSIZE_SHIFT) /* 512 bytes */
#  define USBHS_HSTPIPCFG_PSIZE_1024       (7 << USBHS_HSTPIPCFG_PSIZE_SHIFT) /* 1024 bytes */

#define USBHS_HSTPIPCFG_PTOKEN_SHIFT       (8)               /* Bits 8-9: Pipe Token */
#define USBHS_HSTPIPCFG_PTOKEN_MASK        (3 << USBHS_HSTPIPCFG_PTOKEN_SHIFT)
#  define USBHS_HSTPIPCFG_PTOKEN_SETUP     (0 << USBHS_HSTPIPCFG_PTOKEN_SHIFT)
#  define USBHS_HSTPIPCFG_PTOKEN_IN        (1 << USBHS_HSTPIPCFG_PTOKEN_SHIFT)
#  define USBHS_HSTPIPCFG_PTOKEN_OUT       (2 << USBHS_HSTPIPCFG_PTOKEN_SHIFT)
#define USBHS_HSTPIPCFG_AUTOSW             (1 << 10)         /* Bit 10: Automatic Switch */
#define USBHS_HSTPIPCFG_PTYPE_SHIFT        (12)              /* Bits 12-13: Pipe Type */
#define USBHS_HSTPIPCFG_PTYPE_MASK         (3 << USBHS_HSTPIPCFG_PTYPE_SHIFT)
#  define USBHS_HSTPIPCFG_PTYPE_CTRL       (0 << USBHS_HSTPIPCFG_PTYPE_SHIFT) /* Control pipe */
#  define USBHS_HSTPIPCFG_PTYPE_ISO        (1 << USBHS_HSTPIPCFG_PTYPE_SHIFT) /* Isochronous pipe */
#  define USBHS_HSTPIPCFG_PTYPE_BLK        (2 << USBHS_HSTPIPCFG_PTYPE_SHIFT) /* Bulk pipe */
#  define USBHS_HSTPIPCFG_PTYPE_INTRPT     (3 << USBHS_HSTPIPCFG_PTYPE_SHIFT) /* Interrupt pipe */

#define USBHS_HSTPIPCFG_PEPNUM_SHIFT       (16)              /* Bits 16-19: Pipe Endpoint Number */
#define USBHS_HSTPIPCFG_PEPNUM_MASK        (15 << USBHS_HSTPIPCFG_PEPNUM_SHIFT)
#  define USBHS_HSTPIPCFG_PEPNUM(n)        ((uint32_t)(n) << USBHS_HSTPIPCFG_PEPNUM_SHIFT)
#define USBHS_HSTPIPCFG_PINGEN             (1 << 20)         /* Bit 20: Ping Enable (2) */
#define USBHS_HSTPIPCFG_INTFRQ_SHIFT       (24)              /* Bits 24-31: Pipe Interrupt Request Frequency (1) */
#define USBHS_HSTPIPCFG_INTFRQ_MASK        (0xff << USBHS_HSTPIPCFG_INTFRQ_SHIFT)
#  define USBHS_HSTPIPCFG_INTFRQ(n)        ((uint32_t)(n) << USBHS_HSTPIPCFG_INTFRQ_SHIFT)
#define USBHS_HSTPIPCFG_BINTERVAL_SHIFT    (24)              /* Bits 24-31: Binterval Parameter for the Bulk-Out/Ping Transaction (2) */
#define USBHS_HSTPIPCFG_BINTERVAL_MASK     (0xff << USBHS_HSTPIPCFG_BINTERVAL_SHIFT)
#  define USBHS_HSTPIPCFG_BINTERVAL(n)     ((uint32_t)(n) << USBHS_HSTPIPCFG_BINTERVAL_SHIFT)

/* Common Pipe Interrupt Bit Definitions:
 *
 *   Host Pipe Status Register
 *   Host Pipe Clear Register
 *   Host Pipe Set Register
 *   Host Pipe Mask Register
 *   Host Pipe Enable Register
 *   Host Pipe Disable Register
 *
 * (1) Control and bulk pipes
 * (2) Interrupt pipes
 * (3) Isochronous pipes
 */

/* All registers */

#define USBHS_HSTPIPINT_RXINI              (1 << 0)          /* Bit 0:  Received IN Data Interrupt */
#define USBHS_HSTPIPINT_TXOUTI             (1 << 1)          /* Bit 1:  Transmitted OUT Data Interrupt */
#define USBHS_HSTPIPINT_TXSTPI             (1 << 2)          /* Bit 2:  Transmitted SETUP Interrupt (1) */
#define USBHS_HSTPIPINT_UNDERFI            (1 << 2)          /* Bit 2:  Underflow Interrupt (2,3) */
#define USBHS_HSTPIPINT_PERRI              (1 << 3)          /* Bit 3:  Pipe Error Interrupt */
#define USBHS_HSTPIPINT_NAKEDI             (1 << 4)          /* Bit 4:  NAKed Interrupt */
#define USBHS_HSTPIPINT_OVERFI             (1 << 5)          /* Bit 5:  Overflow Interrupt */
#define USBHS_HSTPIPINT_RXSTALLDI          (1 << 6)          /* Bit 6:  Received STALLed Interrupt (1,2) */
#define USBHS_HSTPIPINT_CRCERRI            (1 << 6)          /* Bit 6:  CRC Error Interrupt (3) */
#define USBHS_HSTPIPINT_SHRTPCKTI          (1 << 7)          /* Bit 7:  Short Packet Interrupt */

/* Host Pipe Set, Host Pipe Mask and Host Pipe Disable Registers only */

#define USBHS_HSTPIPINT_NBUSYBKI           (1 << 12)         /* Bit 12: Number of Busy Banks Interrupt Enable */

/* Host Pipe Mask and Host Pipe Disable Registers only */

#define USBHS_HSTPIPINT_FIFOCONI           (1 << 14)         /* Bit 14: FIFO Control */
#define USBHS_HSTPIPINT_PDISHDMAI          (1 << 16)         /* Bit 16: Pipe Interrupts Disable HDMA Request Enable */
#define USBHS_HSTPIPINT_PFREEZEI           (1 << 17)         /* Bit 17: Pipe Freeze */
#define USBHS_HSTPIPINT_RSTDTI             (1 << 18)         /* Bit 18: Reset Data Toggle */

/* Host Pipe Status Register only */

#define USBHS_HSTPIPISR_DTSEQ_SHIFT        (8)               /* Bits 8-9: Data Toggle Sequence */
#define USBHS_HSTPIPISR_DTSEQ_MASK         (3 << USBHS_HSTPIPISR_DTSEQ_SHIFT)
#  define USBHS_HSTPIPISR_DTSEQ_DATA0      (0 << USBHS_HSTPIPISR_DTSEQ_SHIFT) /* Data0 toggle sequence */
#  define USBHS_HSTPIPISR_DTSEQ_DATA1      (1 << USBHS_HSTPIPISR_DTSEQ_SHIFT) /* Data1 toggle sequence */

#define USBHS_HSTPIPISR_NBUSYBK_SHIFT      (12)              /* Bits 12-13: Number of Busy Banks */
#define USBHS_HSTPIPISR_NBUSYBK_MASK       (3 << USBHS_HSTPIPISR_NBUSYBK_SHIFT)
#  define USBHS_HSTPIPISR_NBUSYBK_0BUSY    (0 << USBHS_HSTPIPISR_NBUSYBK_SHIFT) /* 0 busy bank (all banks free) */
#  define USBHS_HSTPIPISR_NBUSYBK_1BUSY    (1 << USBHS_HSTPIPISR_NBUSYBK_SHIFT) /* 1 busy bank */
#  define USBHS_HSTPIPISR_NBUSYBK_2BUSY    (2 << USBHS_HSTPIPISR_NBUSYBK_SHIFT) /* 2 busy banks */
#  define USBHS_HSTPIPISR_NBUSYBK_3BUSY    (3 << USBHS_HSTPIPISR_NBUSYBK_SHIFT) /* 3 busy banks */

#define USBHS_HSTPIPISR_CURRBK_SHIFT       (14)              /* Bits 14-15: Current Bank */
#define USBHS_HSTPIPISR_CURRBK_MASK        (3 << USBHS_HSTPIPISR_CURRBK_SHIFT)
# define USBHS_HSTPIPISR_CURRBK_BANK0      (0 << USBHS_HSTPIPISR_CURRBK_SHIFT) /* Current bank is bank0 */
# define USBHS_HSTPIPISR_CURRBK_BANK1      (1 << USBHS_HSTPIPISR_CURRBK_SHIFT) /* Current bank is bank1 */
# define USBHS_HSTPIPISR_CURRBK_BANK2      (2 << USBHS_HSTPIPISR_CURRBK_SHIFT) /* Current bank is bank2 */

#define USBHS_HSTPIPISR_RWALL              (1 << 16)         /* Bit 16: Read/Write Allowed */
#define USBHS_HSTPIPISR_CFGOK              (1 << 18)         /* Bit 18: Configuration OK Status */
#define USBHS_HSTPIPISR_PBYCT_SHIFT        (20)              /* Bits 20-30: Pipe Byte Count */
#define USBHS_HSTPIPISR_PBYCT_MASK         (0x7ff << USBHS_HSTPIPISR_PBYCT_SHIFT)

/* Host Pipe IN Request Register */

#define USBHS_HSTPIPINRQ_INRQ_SHIFT        (0)               /* Bits 0-7: IN Request Number before Freeze */
#define USBHS_HSTPIPINRQ_INRQ_MASK         (0xff << USBHS_HSTPIPINRQ_INRQ_SHIFT)
#  define USBHS_HSTPIPINRQ_INRQ(n)         ((uint32_t)(n) << USBHS_HSTPIPINRQ_INRQ_SHIFT)
#define USBHS_HSTPIPINRQ_INMODE            (1 << 8)          /* Bit 8:  IN Request Mode */

/* Host Pipe Error Register */

#define USBHS_HSTPIPERR_DATATGL            (1 << 0)          /* Bit 0:  Data Toggle Error */
#define USBHS_HSTPIPERR_DATAPID            (1 << 1)          /* Bit 1:  Data PID Error */
#define USBHS_HSTPIPERR_PID                (1 << 2)          /* Bit 2:  PID Error */
#define USBHS_HSTPIPERR_TIMEOUT            (1 << 3)          /* Bit 3:  Time-Out Error */
#define USBHS_HSTPIPERR_CRC16              (1 << 4)          /* Bit 4:  CRC16 Error */
#define USBHS_HSTPIPERR_COUNTER_SHIFT      (5)               /* Bits 5-6: Error Counter */
#define USBHS_HSTPIPERR_COUNTER_MASK       (3 << USBHS_HSTPIPERR_COUNTER_SHIFT)
#  define USBHS_HSTPIPERR_COUNTER(n)       ((uint32_t)(n) << USBHS_HSTPIPERR_COUNTER_SHIFT)

/* Host DMA Channel Next Descriptor Address Register (32-bit address) */

/* Host DMA Channel Address Register (32-bit address) */

/* Host DMA Channel Control Register */

#define USBHS_HSTDMACTRL_CMD_SHIFT         (0)               /* Bits 0-1: Command */
#define USBHS_HSTDMACTRL_CMD_MASK          (3 << USBHS_HSTDMACTRL_CMD_SHIFT)
#  define USBHS_HSTDMACTRL_CHANNENB        (1 << 0)          /* Bit 0:  Channel Enable Command */
#  define USBHS_HSTDMACTRL_LDNXTDSC        (1 << 1)          /* Bit 1:  Load Next Channel Transfer Descriptor Enable Command */

#  define USBHS_HSTDMACTRL_STOPNOW         (0 << USBHS_HSTDMACTRL_CMD_SHIFT) /* Stop now */
#  define USBHS_HSTDMACTRL_RUNSTOP         (1 << USBHS_HSTDMACTRL_CMD_SHIFT) /* Run and stop at end of buffer */
#  define USBHS_HSTDMACTRL_LOADNEXT        (2 << USBHS_HSTDMACTRL_CMD_SHIFT) /* Load next descriptor now */
#  define USBHS_HSTDMACTRL_RUNLINK         (3 << USBHS_HSTDMACTRL_CMD_SHIFT) /* Run and link at end of buffer */

#define USBHS_HSTDMACTRL_ENDTREN           (1 << 2)          /* Bit 2:  End of Transfer Enable Control */
#define USBHS_HSTDMACTRL_ENDBEN            (1 << 3)          /* Bit 3:  End of Buffer Enable Control */
#define USBHS_HSTDMACTRL_ENDTRIT           (1 << 4)          /* Bit 4:  End of Transfer Interrupt Enable */
#define USBHS_HSTDMACTRL_ENDBUFFIT         (1 << 5)          /* Bit 5:  End of Buffer Interrupt Enable */
#define USBHS_HSTDMACTRL_DESCLDIT          (1 << 6)          /* Bit 6:  Descriptor Loaded Interrupt Enable */
#define USBHS_HSTDMACTRL_BURSTLCK          (1 << 7)          /* Bit 7:  Burst Lock Enable */
#define USBHS_HSTDMACTRL_BUFLEN_SHIFT      (16)              /* Bits 16-31: Buffer Byte Length  */
#define USBHS_HSTDMACTRL_BUFLEN_MASK       (0xffff << USBHS_HSTDMACTRL_BUFLEN_SHIFT)
#  define USBHS_HSTDMACTRL_BUFLEN(n)       ((uint32_t)(n) << USBHS_HSTDMACTRL_BUFLEN_SHIFT)

/* Host DMA Channel Status Register */

#define USBHS_HSTDMASTA_CHANNENB           (1 << 0)          /* Bit 0:  Channel Enable Status */
#define USBHS_HSTDMASTA_CHANNACT           (1 << 1)          /* Bit 1:  Channel Active Status */
#define USBHS_HSTDMASTA_ENDTRST            (1 << 4)          /* Bit 4:  End of Transfer Status */
#define USBHS_HSTDMASTA_ENDBUFFST          (1 << 5)          /* Bit 5:  End of Buffer Status */
#define USBHS_HSTDMASTA_DESCLDST           (1 << 6)          /* Bit 6:  Descriptor Loaded Status */
#define USBHS_HSTDMASTA_BUFCNT_SHIFT       (16)              /* Bits 16-31: Buffer Byte Count */
#  define USBHS_HSTDMASTA_BUFCNT(n)        ((uint32_t)(n) << USBHS_HSTDMASTA_BUFCNT_SHIFT)

/* USBHS General Register Bit Field Definitions */

/* General Control Register */

#define USBHS_CTRL_RDERRE                  (1 << 4)          /* Bit 4:  Remote Device Connection Error Interrupt Enable */
#define USBHS_CTRL_FRZCLK                  (1 << 14)         /* Bit 14: Freeze USB Clock */
#define USBHS_CTRL_USBE                    (1 << 15)         /* Bit 15: USBHS Enable */
#define USBHS_CTRL_UIDE                    (1 << 24)         /* Bit 24: UOTGID Pin Enable */
#  define USBHS_CTRL_UIDE_UIMOD            (0 << 24)         /*   0=USB mode selected UIMOD bit. */
#  define USBHS_CTRL_UIDE_UOTGID           (1 << 24)         /*   1=USB mode selected by UOTGID */
#define USBHS_CTRL_UIMOD_MASK              (1 << 25)         /* Bit 25: USBHS Mode */
#  define USBHS_CTRL_UIMOD_HOST            (0 << 25)         /*   0=Host mode */
#  define USBHS_CTRL_UIMOD_DEVICE          (1 << 25)         /*   1=Device mode */

/* General Status Register */

#define USBHS_SR_RDERRI                    (1 << 4)          /* Bit 4:  Remote Device Connection Error Interrupt (host mode) */
#define USBHS_SR_VBUSRQ                    (1 << 9)          /* Bit 9:  VBus Request (host mode) */
#define USBHS_SR_SPEED_SHIFT               (12)              /* Bits 12-13: Speed Status (device mode) */
#define USBHS_SR_SPEED_MASK                (3 << USBHS_SR_SPEED_SHIFT)
#  define USBHS_SR_SPEED_FULL              (0 << USBHS_SR_SPEED_SHIFT) /* Full-Speed mode */
#  define USBHS_SR_SPEED_HIGH              (1 << USBHS_SR_SPEED_SHIFT) /* High-Speed mode */
#  define USBHS_SR_SPEED_LOW               (2 << USBHS_SR_SPEED_SHIFT) /* Low-Speed mode */

#define USBHS_SR_CLKUSABLE                 (1 << 14)         /* Bit 14: UTMI Clock Usable */

/* General Status Clear Register */

#define USBHS_SCR_RDERRIC                  (1 << 4)          /* Bit 4:  Remote Device Connection Error Interrupt Clear */
#define USBHS_SCR_VBUSRQC                  (1 << 9)          /* Bit 9:   VBus Request Clear */

/* General Status Set Register */

#define USBHS_SFR_RDERRIS                  (1 << 4)          /* Bit 4:  Remote Device Connection Error Interrupt Set */
#define USBHS_SFR_VBUSRQS                  (1 << 9)          /* Bit 9:  VBus Request Set */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure defines the USBHS DMA Transfer Descriptor.
 * Instances of DMA transfer descriptors must by aligned to 16-byte address
 * boundaries.
 *
 * Each value contains the next value of each of three USBHS DMA registers.
 * The first register value (USBHS_xxxDMANXTDSCx) is a link that can be
 * used to chain sequences of DMA transfers.
 */

struct usbhs_dtd_s
{
  uint32_t nxtd;  /* Next Descriptor Address Register: USBHS_xxxDMANXTDSCx */
  uint32_t addr;  /* DMA Channelx Address Register: USBHS_xxxDMAADDRESSx */
  uint32_t ctrl;  /* DMA Channelx Control Register: USBHS_xxxDMACONTROLx */
};
#define SIZEOF_USPHS_DTD_S 12

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_USBHS_H */
