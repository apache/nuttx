/************************************************************************************
 * arch/arm/src/sama5/sam_dmac.h
 *
 *   Copyright (C) 2013-2014 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMA5_SAM_DMAC_H
#define __ARCH_ARM_SRC_SAMA5_SAM_DMAC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Configuration ********************************************************************/

#ifndef CONFIG_DEBUG_FEATURES
#  undef CONFIG_DEBUG_DMA
#endif

/* DMA ******************************************************************************/

/* Flags used to characterize the DMA channel.  The naming convention is that one
 * side is the peripheral and the other is memory (however, the interface could still
 * be used if, for example, both sides were memory although the naming would be
 * awkward)
 */

#if defined(ATSAMA5D3)
/* MMMM MMMM MMMM MMMP PPPP PPPP PPPP PPFF
 * .... .... .... .... .... .... .... ..FF       Configurable properties of the channel
 * .... .... .... ...P PPPP PPPP PPPP PP..       Peripheral endpoint characteristics
 * MMMM MMMM MMMM MMM. .... .... .... ....       Memory endpoint characteristics
 */

/* Bits 0-1: Configurable properties of the channel
 *
 * .... .... .... .... .... .... .... ..FF       Configurable properties of the channel
 */

#  define DMACH_FLAG_BURST_LARGEST            0  /* Largest length AHB burst */
#  define DMACH_FLAG_BURST_HALF               1  /* Half FIFO size */
#  define DMACH_FLAG_BURST_SINGLE             2  /* Single AHB access */

#  define DMACH_FLAG_FIFOCFG_SHIFT            (0)       /* Bits 0-1: FIFO configuration */
#  define DMACH_FLAG_FIFOCFG_MASK             (3 << DMACH_FLAG_FIFOCFG_SHIFT)
#    define DMACH_FLAG_FIFOCFG_LARGEST        (DMACH_FLAG_BURST_LARGEST << DMACH_FLAG_FIFOCFG_SHIFT)
#    define DMACH_FLAG_FIFOCFG_HALF           (DMACH_FLAG_BURST_HALF << DMACH_FLAG_FIFOCFG_SHIFT)
#    define DMACH_FLAG_FIFOCFG_SINGLE         (DMACH_FLAG_BURST_SINGLE << DMACH_FLAG_FIFOCFG_SHIFT)

/* Bits 2-16: Peripheral endpoint characteristics
 *
 * .... .... .... ...P PPPP PPPP PPPP PP..       Peripheral endpoint characteristics
 * .... .... .... .... .... .... IIII II..       Peripheral ID, range 0-49
 * .... .... .... .... .... ...H .... ....       HW Handshaking
 * .... .... .... .... .... ..P. .... ....       0=memory; 1=peripheral
 * .... .... .... .... .... NN.. .... ....       Peripheral ABH layer number
 * .... .... .... .... ..WW .... .... ....       Peripheral width
 * .... .... .... .... .A.. .... .... ....       Auto-increment peripheral address
 * .... .... .... ...S S... .... .... ....       Peripheral chunk size
 */

#  define DMACH_FLAG_PERIPHPID_SHIFT          (2)       /* Bits 1-7: Peripheral PID */
#  define DMACH_FLAG_PERIPHPID_MASK           (0x3f << DMACH_FLAG_PERIPHPID_SHIFT)
#    define DMACH_FLAG_PERIPHPID(n)           ((uint32_t)(n) << DMACH_FLAG_PERIPHPID_SHIFT)
#    define DMACH_FLAG_PERIPHPID_MAX          DMACH_FLAG_PERIPHPID_MASK
#  define DMACH_FLAG_PERIPHH2SEL              (1 << 8)  /* Bits 8: HW handshaking */
#  define DMACH_FLAG_PERIPHISPERIPH           (1 << 9)  /* Bits 9: 0=memory; 1=peripheral */
#  define DMACH_FLAG_PERIPHAHB_SHIFT          (10)      /* Bits 10-11: Peripheral ABH layer number */
#  define DMACH_FLAG_PERIPHAHB_MASK           (3 << DMACH_FLAG_PERIPHAHB_SHIFT)
#    define DMACH_FLAG_PERIPHAHB_AHB_IF0      (0 << DMACH_FLAG_PERIPHAHB_SHIFT) /* AHB-Lite Interface 0 */
#    define DMACH_FLAG_PERIPHAHB_AHB_IF1      (1 << DMACH_FLAG_PERIPHAHB_SHIFT) /* AHB-Lite Interface 1 */
#    define DMACH_FLAG_PERIPHAHB_AHB_IF2      (2 << DMACH_FLAG_PERIPHAHB_SHIFT) /* AHB-Lite Interface 2 */
#  define DMACH_FLAG_PERIPHWIDTH_SHIFT        (12)      /* Bits 12-13: Peripheral width */
#  define DMACH_FLAG_PERIPHWIDTH_MASK         (3 << DMACH_FLAG_PERIPHWIDTH_SHIFT)
#    define DMACH_FLAG_PERIPHWIDTH_8BITS      (0 << DMACH_FLAG_PERIPHWIDTH_SHIFT) /* 8 bits */
#    define DMACH_FLAG_PERIPHWIDTH_16BITS     (1 << DMACH_FLAG_PERIPHWIDTH_SHIFT) /* 16 bits */
#    define DMACH_FLAG_PERIPHWIDTH_32BITS     (2 << DMACH_FLAG_PERIPHWIDTH_SHIFT) /* 32 bits */
#    define DMACH_FLAG_PERIPHWIDTH_64BITS     (3 << DMACH_FLAG_PERIPHWIDTH_SHIFT) /* 64 bits */
#  define DMACH_FLAG_PERIPHINCREMENT          (1 << 14) /* Bit 14: Auto-increment peripheral address */
#  define DMACH_FLAG_PERIPHCHUNKSIZE_SHIFT    (15)      /* Bits 15-16: Peripheral chunk size */
#  define DMACH_FLAG_PERIPHCHUNKSIZE_MASK     (3 << DMACH_FLAG_PERIPHCHUNKSIZE_SHIFT)
#    define DMACH_FLAG_PERIPHCHUNKSIZE_1      (0 << DMACH_FLAG_PERIPHCHUNKSIZE_SHIFT) /* Peripheral chunksize = 1 */
#    define DMACH_FLAG_PERIPHCHUNKSIZE_2      (0 << DMACH_FLAG_PERIPHCHUNKSIZE_SHIFT) /* No chunksize = 2 */
#    define DMACH_FLAG_PERIPHCHUNKSIZE_4      (1 << DMACH_FLAG_PERIPHCHUNKSIZE_SHIFT) /* Peripheral chunksize = 4 */
#    define DMACH_FLAG_PERIPHCHUNKSIZE_8      (2 << DMACH_FLAG_PERIPHCHUNKSIZE_SHIFT) /* Peripheral chunksize = 8 */
#    define DMACH_FLAG_PERIPHCHUNKSIZE_16     (3 << DMACH_FLAG_PERIPHCHUNKSIZE_SHIFT) /* Peripheral chunksize = 16 */

/* Bits 17-31: Memory endpoint characteristics
 *
 * MMMM MMMM MMMM MMM. .... .... .... ....      Memory endpoint characteristics
 * .... .... .III III. .... .... .... ....      Memory/peripheral ID, range 0-49
 * .... .... H... .... .... .... .... ....      HW Handshaking
 * .... ...P .... .... .... .... .... ....      0=memory; 1=peripheral
 * .... .NN. .... .... .... .... .... ....      Peripheral ABH layer number
 * ...W W... .... .... .... .... .... ....      Peripheral width
 * ..A. .... .... .... .... .... .... ....      Auto-increment peripheral address
 * SS.. .... .... .... .... .... .... ....      Peripheral chunk size
 */

#  define DMACH_FLAG_MEMPID_SHIFT             (17)      /* Bits 17-22: Memory PID */
#  define DMACH_FLAG_MEMPID_MASK              (0x3f << DMACH_FLAG_MEMPID_SHIFT)
#    define DMACH_FLAG_MEMPID(n)              ((uint32_t)(n) << DMACH_FLAG_MEMPID_SHIFT)
#    define DMACH_FLAG_MEMPID_MAX             DMACH_FLAG_MEMPID_MASK
#  define DMACH_FLAG_MEMH2SEL                 (1 << 23) /* Bits 23: HW handshaking */
#  define DMACH_FLAG_MEMISPERIPH              (1 << 24) /* Bits 24: 0=memory; 1=peripheral */
#  define DMACH_FLAG_MEMAHB_SHIFT             (25)      /* Bits 25-26: Peripheral ABH layer number */
#  define DMACH_FLAG_MEMAHB_MASK              (3 << DMACH_FLAG_MEMAHB_SHIFT)
#    define DMACH_FLAG_MEMAHB_AHB_IF0         (0 << DMACH_FLAG_MEMAHB_SHIFT) /* AHB-Lite Interface 0 */
#    define DMACH_FLAG_MEMAHB_AHB_IF1         (1 << DMACH_FLAG_MEMAHB_SHIFT) /* AHB-Lite Interface 1 */
#    define DMACH_FLAG_MEMAHB_AHB_IF2         (2 << DMACH_FLAG_MEMAHB_SHIFT) /* AHB-Lite Interface 2 */
#  define DMACH_FLAG_MEMWIDTH_SHIFT           (27)      /* Bits 27-28: Memory width */
#  define DMACH_FLAG_MEMWIDTH_MASK            (3 << DMACH_FLAG_MEMWIDTH_SHIFT)
#    define DMACH_FLAG_MEMWIDTH_8BITS         (0 << DMACH_FLAG_MEMWIDTH_SHIFT) /* 8 bits */
#    define DMACH_FLAG_MEMWIDTH_16BITS        (1 << DMACH_FLAG_MEMWIDTH_SHIFT) /* 16 bits */
#    define DMACH_FLAG_MEMWIDTH_32BITS        (2 << DMACH_FLAG_MEMWIDTH_SHIFT) /* 32 bits */
#    define DMACH_FLAG_MEMWIDTH_64BITS        (3 << DMACH_FLAG_MEMWIDTH_SHIFT) /* 64 bits */
#  define DMACH_FLAG_MEMINCREMENT             (1 << 29) /* Bit 29: Auto-increment memory address */
#  define DMACH_FLAG_MEMCHUNKSIZE_SHIFT       (30)      /* Bit 30-31: Memory chunk size */
#  define DMACH_FLAG_MEMCHUNKSIZE_MASK        (3 << DMACH_FLAG_MEMCHUNKSIZE_SHIFT)
#    define DMACH_FLAG_MEMCHUNKSIZE_1         (0 << DMACH_FLAG_MEMCHUNKSIZE_SHIFT) /* Peripheral chunksize = 1 */
#    define DMACH_FLAG_MEMCHUNKSIZE_2         (0 << DMACH_FLAG_MEMCHUNKSIZE_SHIFT) /* No chunksize = 2 */
#    define DMACH_FLAG_MEMCHUNKSIZE_4         (1 << DMACH_FLAG_MEMCHUNKSIZE_SHIFT) /* Peripheral chunksize = 4 */
#    define DMACH_FLAG_MEMCHUNKSIZE_8         (2 << DMACH_FLAG_MEMCHUNKSIZE_SHIFT) /* Peripheral chunksize = 8 */
#    define DMACH_FLAG_MEMCHUNKSIZE_16        (3 << DMACH_FLAG_MEMCHUNKSIZE_SHIFT) /* Peripheral chunksize = 16 */

#  define DMACH_FLAG_MEMBURST_1               (0) /* No memory burst size */
#  define DMACH_FLAG_MEMBURST_4               (0)
#  define DMACH_FLAG_MEMBURST_8               (0)
#  define DMACH_FLAG_MEMBURST_16              (0)

#elif defined(ATSAMA5D4)
/* .... .... .... MMMM .PPP PPPP PPPP PPPP
 * .... .... .... .... .... .... .... ....       Configurable properties of the channel
 * .... .... .... .... .PPP PPPP PPPP PPPP       Peripheral endpoint characteristics
 * .... .... .... MMMM .... .... .... ....       Memory endpoint characteristics
 */

/* Bits 0-1: Configurable properties of the channel
 *
 * .... .... .... .... .... .... .... ....       Configurable properties of the channel
 *
 * NOTE: Many "peripheral" attributes are really "channel" attributes for
 * the SAMA5D4's XDMAC since it does not support peripheral-to-peripheral
 * DMA.
 */

#  define DMACH_FLAG_FIFOCFG_LARGEST        (0) /* No FIFO controls */
#  define DMACH_FLAG_FIFOCFG_HALF           (0)
#  define DMACH_FLAG_FIFOCFG_SINGLE         (0)

/* Bits 0-15: Peripheral endpoint characteristics
 *
 * .... .... .... .... .PPP PPPP PPPP PPPP       Peripheral endpoint characteristics
 * .... .... .... .... .... .... .III IIII       Peripheral ID, range 0-67
 * .... .... .... .... .... .... .... ....       No HW Handshaking
 * .... .... .... .... .... .... P... ....       0=memory; 1=peripheral
 * .... .... .... .... .... ...N .... ....       Peripheral ABH layer number
 * .... .... .... .... .... .WW. .... ....       Peripheral width
 * .... .... .... .... .... A... .... ....       Auto-increment peripheral address
 * .... .... .... .... .SSS .... .... ....       Peripheral chunk size
 */

#  define DMACH_FLAG_PERIPHPID_SHIFT          (0)       /* Bits 0-7: Peripheral PID */
#  define DMACH_FLAG_PERIPHPID_MASK           (0x7f << DMACH_FLAG_PERIPHPID_SHIFT)
#    define DMACH_FLAG_PERIPHPID(n)           ((uint32_t)(n) << DMACH_FLAG_PERIPHPID_SHIFT)
#    define DMACH_FLAG_PERIPHPID_MAX          DMACH_FLAG_PERIPHPID_MASK
#  define DMACH_FLAG_PERIPHH2SEL              (0)       /* No HW handshaking */
#  define DMACH_FLAG_PERIPHISPERIPH           (1 << 7)  /* Bit 7:  0=memory; 1=peripheral */
#  define DMACH_FLAG_PERIPHAHB_MASK           (1 << 8)  /* Bit 8:  Peripheral ABH layer 1 */
#    define DMACH_FLAG_PERIPHAHB_AHB_IF0      (0)
#    define DMACH_FLAG_PERIPHAHB_AHB_IF1      DMACH_FLAG_PERIPHAHB_MASK
#  define DMACH_FLAG_PERIPHWIDTH_SHIFT        (9)       /* Bits 9-10: Peripheral width */
#  define DMACH_FLAG_PERIPHWIDTH_MASK         (3 << DMACH_FLAG_PERIPHWIDTH_SHIFT)
#    define DMACH_FLAG_PERIPHWIDTH_8BITS      (0 << DMACH_FLAG_PERIPHWIDTH_SHIFT) /* 8 bits */
#    define DMACH_FLAG_PERIPHWIDTH_16BITS     (1 << DMACH_FLAG_PERIPHWIDTH_SHIFT) /* 16 bits */
#    define DMACH_FLAG_PERIPHWIDTH_32BITS     (2 << DMACH_FLAG_PERIPHWIDTH_SHIFT) /* 32 bits */
#    define DMACH_FLAG_PERIPHWIDTH_64BITS     (3 << DMACH_FLAG_PERIPHWIDTH_SHIFT) /* 64 bits */
#  define DMACH_FLAG_PERIPHINCREMENT          (1 << 11) /* Bit 11: Auto-increment peripheral address */
#  define DMACH_FLAG_PERIPHCHUNKSIZE_SHIFT    (12)      /* Bits 12-14: Peripheral chunk size */
#  define DMACH_FLAG_PERIPHCHUNKSIZE_MASK     (7 << DMACH_FLAG_PERIPHCHUNKSIZE_SHIFT)
#    define DMACH_FLAG_PERIPHCHUNKSIZE_1      (0 << DMACH_FLAG_PERIPHCHUNKSIZE_SHIFT) /* Peripheral chunksize=1 */
#    define DMACH_FLAG_PERIPHCHUNKSIZE_2      (1 << DMACH_FLAG_PERIPHCHUNKSIZE_SHIFT) /* No chunksize=2 */
#    define DMACH_FLAG_PERIPHCHUNKSIZE_4      (2 << DMACH_FLAG_PERIPHCHUNKSIZE_SHIFT) /* Peripheral chunksize=4 */
#    define DMACH_FLAG_PERIPHCHUNKSIZE_8      (3 << DMACH_FLAG_PERIPHCHUNKSIZE_SHIFT) /* Peripheral chunksize=8 */
#    define DMACH_FLAG_PERIPHCHUNKSIZE_16     (4 << DMACH_FLAG_PERIPHCHUNKSIZE_SHIFT) /* Peripheral chunksize=16 */

/* Bits 16-19: Memory endpoint characteristics
 *
 * .... .... .... MMMM .... .... .... ....      Memory endpoint characteristics
 * .... .... .... .... .... .... .... ....      No memory peripheral ID, range 0-49
 * .... .... .... .... .... .... .... ....      No HW Handshaking
 * .... .... .... .... .... .... .... ....      No peripheral-to-peripheral
 * .... .... .... ...N .... .... .... ....      Memory ABH layer number
 * .... .... .... .... .... .... .... ....      No memory width
 * .... .... .... ..A. .... .... .... ....      Auto-increment memory address
 * .... .... .... .... .... .... .... ....      No memory chunk size
 * .... .... .... BB.. .... .... .... ....      Memory burst size
 */

#  define DMACH_FLAG_MEMPID(n)                (0)       /* No memory peripheral identifier */
#    define DMACH_FLAG_MEMPID_MAX             (0)
#  define DMACH_FLAG_MEMH2SEL                 (0)       /* No HW handshaking */
#  define DMACH_FLAG_MEMISPERIPH              (0)       /* No peripheral-to-peripheral */
#  define DMACH_FLAG_MEMAHB_MASK              (1 << 16) /* Bit 16: Memory ABH layer 1 */
#    define DMACH_FLAG_MEMAHB_AHB_IF0         (0)
#    define DMACH_FLAG_MEMAHB_AHB_IF1         DMACH_FLAG_MEMAHB_MASK

#  define DMACH_FLAG_MEMWIDTH_8BITS           (0) /* Only peripheral data width */
#  define DMACH_FLAG_MEMWIDTH_16BITS          (0)
#  define DMACH_FLAG_MEMWIDTH_32BITS          (0)
#  define DMACH_FLAG_MEMWIDTH_64BITS          (0)

#  define DMACH_FLAG_MEMINCREMENT             (1 << 17) /* Bit 17: Auto-increment memory address */

#  define DMACH_FLAG_MEMCHUNKSIZE_1           (0) /* Only peripheral chunk size */
#  define DMACH_FLAG_MEMCHUNKSIZE_2           (0)
#  define DMACH_FLAG_MEMCHUNKSIZE_4           (0)
#  define DMACH_FLAG_MEMCHUNKSIZE_8           (0)
#  define DMACH_FLAG_MEMCHUNKSIZE_16          (0)

#  define DMACH_FLAG_MEMBURST_SHIFT           (18)      /* Bits 18-19: Memory burst size */
#  define DMACH_FLAG_MEMBURST_MASK            (3 << DMACH_FLAG_MEMBURST_SHIFT)
#    define DMACH_FLAG_MEMBURST_1             (0 << DMACH_FLAG_MEMBURST_SHIFT)
#    define DMACH_FLAG_MEMBURST_4             (1 << DMACH_FLAG_MEMBURST_SHIFT)
#    define DMACH_FLAG_MEMBURST_8             (2 << DMACH_FLAG_MEMBURST_SHIFT)
#    define DMACH_FLAG_MEMBURST_16            (3 << DMACH_FLAG_MEMBURST_SHIFT)

#endif /* ATSAMA5D4 */

/************************************************************************************
 * Public Types
 ************************************************************************************/

typedef FAR void *DMA_HANDLE;
typedef void (*dma_callback_t)(DMA_HANDLE handle, void *arg, int result);

/* The following is used for sampling DMA registers when CONFIG DEBUG_DMA is selected */

#ifdef CONFIG_DEBUG_DMA
#if defined(CONFIG_SAMA5_DMAC0) || defined(CONFIG_SAMA5_DMAC1)

struct sam_dmaregs_s
{
  /* Global Registers */

  uint32_t gcfg;    /* DMAC Global Configuration Register */
  uint32_t en;      /* DMAC Enable Register */
  uint32_t sreq;    /* DMAC Software Single Request Register */
  uint32_t creq;    /* DMAC Software Chunk Transfer Request Register */
  uint32_t last;    /* DMAC Software Last Transfer Flag Register */
  uint32_t ebcimr;  /* DMAC Error Mask */
  uint32_t ebcisr;  /* DMAC Error Status */
  uint32_t chsr;    /* DMAC Channel Handler Status Register */
  uint32_t wpmr;    /* DMAC Write Protect Mode Register */
  uint32_t wpsr;    /* DMAC Write Protect Status Register */

  /* Channel Registers */

  uint32_t saddr;   /* DMAC Channel Source Address Register */
  uint32_t daddr;   /* DMAC Channel Destination Address Register */
  uint32_t dscr;    /* DMAC Channel Descriptor Address Register */
  uint32_t ctrla;   /* DMAC Channel Control A Register */
  uint32_t ctrlb;   /* DMAC Channel Control B Register */
  uint32_t cfg;     /* DMAC Channel Configuration Register */
  uint32_t spip;    /* DMAC Channel Source PinP Configuration Register */
  uint32_t dpip;    /* DMAC Channel Destination PinP Configuration Register */
};

#elif defined(CONFIG_SAMA5_XDMAC0) || defined(CONFIG_SAMA5_XDMAC1)

struct sam_dmaregs_s
{
  /* Global Registers */

  uint32_t gtype;   /* Global Type Register */
  uint32_t gcfg;    /* Global Configuration Register */
  uint32_t gwac;    /* Global Weighted Arbiter Configuration Register */
  uint32_t gim;     /* Global Interrupt Mask Register */
  uint32_t gis;     /* Global Interrupt Status Register */
  uint32_t gs;      /* Global Channel Status Register */
  uint32_t grs;     /* Global Channel Read Suspend Register */
  uint32_t gws;     /* Global Channel Write Suspend Register */
  uint32_t gsws;    /* Global Channel Software Request Status Register */

  /* Channel Registers */

  uint32_t cim;     /* Channel Interrupt Mask Register */
  uint32_t cis;     /* Channel Interrupt Status Register */
  uint32_t csa;     /* Channel Source Address Register */
  uint32_t cda;     /* Channel Destination Address Register */
  uint32_t cnda;    /* Channel Next Descriptor Address Register */
  uint32_t cndc;    /* Channel Next Descriptor Control Register */
  uint32_t cubc;    /* Channel Microblock Control Register */
  uint32_t cbc;     /* Channel Block Control Register */
  uint32_t cc;      /* Channel Configuration Register */
  uint32_t cdsmsp;  /* Channel Data Stride Memory Set Pattern */
  uint32_t csus;    /* Channel Source Microblock Stride */
  uint32_t cdus;    /* Channel Destination Microblock Stride */
};

#endif /* CONFIG_SAMA5_XDMAC0 || CONFIG_SAMA5_XDMAC1 */
#endif /* CONFIG_DEBUG_DMA */

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Data
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/************************************************************************************
 * Name: sam_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function sets aside a DMA channel then gives the
 *   caller exclusive access to the DMA channel.
 *
 *   The naming convention in all of the DMA interfaces is that one side is the
 *   'peripheral' and the other is 'memory'.  However, the interface could still
 *   be used if, for example, both sides were memory although the naming would be
 *   awkward.
 *
 * Returned Value:
 *   If a DMA channel is available, this function returns a non-NULL, void* DMA
 *   channel handle.  NULL is returned on any failure.
 *
 ************************************************************************************/

DMA_HANDLE sam_dmachannel(uint8_t dmacno, uint32_t chflags);

/************************************************************************************
 * Name: sam_dmaconfig
 *
 * Description:
 *   There are two channel usage models:  (1) The channel is allocated and configured
 *   in one step.  This is the typical case where a DMA channel performs a constant
 *   role.  The alternative is (2) where the DMA channel is reconfigured on the fly.
 *   In this case, the chflags provided to sam_dmachannel are not used and
 *   sam_dmaconfig() is called before each DMA to configure the DMA channel
 *   appropriately.
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

void sam_dmaconfig(DMA_HANDLE handle, uint32_t chflags);

/************************************************************************************
 * Name: sam_dmafree
 *
 * Description:
 *   Release a DMA channel.  NOTE:  The 'handle' used in this argument must NEVER be
 *   used again until sam_dmachannel() is called again to re-gain a valid handle.
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

void sam_dmafree(DMA_HANDLE handle);

/************************************************************************************
 * Name: sam_dmatxsetup
 *
 * Description:
 *   Configure DMA for transmit of one buffer (memory to peripheral).  This function
 *   may be called multiple times to handle large and/or discontinuous transfers.
 *   Calls to sam_dmatxsetup() and sam_dmarxsetup() must not be intermixed on the
 *   same transfer, however.
 *
 ************************************************************************************/

int sam_dmatxsetup(DMA_HANDLE handle, uint32_t paddr, uint32_t maddr, size_t nbytes);

/************************************************************************************
 * Name: sam_dmarxsetup
 *
 * Description:
 *   Configure DMA for receipt of one buffer (peripheral to memory).  This function
 *   may be called multiple times to handle large and/or discontinuous transfers.
 *   Calls to sam_dmatxsetup() and sam_dmarxsetup() must not be intermixed on the
 *   same transfer, however.
 *
 ************************************************************************************/

int sam_dmarxsetup(DMA_HANDLE handle, uint32_t paddr, uint32_t maddr, size_t nbytes);

/************************************************************************************
 * Name: sam_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 ************************************************************************************/

int sam_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg);

/************************************************************************************
 * Name: sam_dmastop
 *
 * Description:
 *   Cancel the DMA.  After sam_dmastop() is called, the DMA channel is reset and
 *   sam_dmarx/txsetup() must be called before sam_dmastart() can be called again
 *
 ************************************************************************************/

void sam_dmastop(DMA_HANDLE handle);

/************************************************************************************
 * Name: sam_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 ************************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void sam_dmasample(DMA_HANDLE handle, struct sam_dmaregs_s *regs);
#else
#  define sam_dmasample(handle,regs)
#endif

/************************************************************************************
 * Name: sam_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 ************************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void sam_dmadump(DMA_HANDLE handle, const struct sam_dmaregs_s *regs, const char *msg);
#else
#  define sam_dmadump(handle,regs,msg)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_SAMA5_SAM_DMAC_H */
