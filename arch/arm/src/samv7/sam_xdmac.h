/****************************************************************************
 * arch/arm/src/samv7/sam_xdmac.h
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

#ifndef __ARCH_ARM_SRC_SAMV7_SAM_XDMAC_H
#define __ARCH_ARM_SRC_SAMV7_SAM_XDMAC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "chip.h"

#include "hardware/sam_xdmac.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DMA **********************************************************************/

/* Flags used to characterize the DMA channel.
 * The naming convention is that one side is the peripheral and the other is
 * memory (however, the interface could still be used if, for example, both
 * sides were memory although the naming would be awkward)
 *
 * Encoding:
 *
 * .... .... .... MMMM .PPP PPPP PPPP PPPP
 * .... .... .... .... .... .... .... ....  Configurable properties of the
 *                                          channel
 * .... .... .... .... .PPP PPPP PPPP PPPP  Peripheral endpoint
 *                                          characteristics
 * .... .... .... MMMM .... .... .... ....  Memory endpoint characteristics
 */

/* Bits 0-1: Configurable properties of the channel
 *
 * .... .... .... .... .... .... .... ....  Configurable properties of the
 *                                          channel
 *
 * NOTE: Many "peripheral" attributes are really "channel" attributes for
 * the samv7D4's XDMAC since it does not support peripheral-to-peripheral
 * DMA.
 */

#  define DMACH_FLAG_FIFOCFG_LARGEST        (0) /* No FIFO controls */
#  define DMACH_FLAG_FIFOCFG_HALF           (0)
#  define DMACH_FLAG_FIFOCFG_SINGLE         (0)

/* Bits 0-15: Peripheral endpoint characteristics
 *
 * .... .... .... .... .PPP PPPP PPPP PPPP  Peripheral endpoint
 *                                          characteristics
 * .... .... .... .... .... .... .III IIII  Peripheral ID, range 0-67
 * .... .... .... .... .... .... .... ....  No HW Handshaking
 * .... .... .... .... .... .... P... ....  0=memory; 1=peripheral
 * .... .... .... .... .... ...N .... ....  Peripheral ABH layer number
 * .... .... .... .... .... .WW. .... ....  Peripheral width
 * .... .... .... .... .... A... .... ....  Auto-increment peripheral address
 * .... .... .... .... .SSS .... .... ....  Peripheral chunk size
 */

#define DMACH_FLAG_PERIPHPID_SHIFT          (0)       /* Bits 0-7: Peripheral PID */
#define DMACH_FLAG_PERIPHPID_MASK           (0x7f << DMACH_FLAG_PERIPHPID_SHIFT)
#  define DMACH_FLAG_PERIPHPID(n)           ((uint32_t)(n) << DMACH_FLAG_PERIPHPID_SHIFT)
#  define DMACH_FLAG_PERIPHPID_MAX          DMACH_FLAG_PERIPHPID_MASK
#define DMACH_FLAG_PERIPHH2SEL              (0)       /* No HW handshaking */
#define DMACH_FLAG_PERIPHIS_MASK            (1 << 7)  /* Bit 7:  Peripheral type */
#  define DMACH_FLAG_PERIPHISPERIPH         (1 << 7)  /* Bit 7:  1 = Peripheral */
#  define DMACH_FLAG_PERIPHISMEMORY         (0 << 7)  /* Bit 7:  0 = Memory */
#define DMACH_FLAG_PERIPHAHB_MASK           (1 << 8)  /* Bit 8:  Peripheral ABH layer 1 */
#  define DMACH_FLAG_PERIPHAHB_AHB_IF0      (0)
#  define DMACH_FLAG_PERIPHAHB_AHB_IF1      DMACH_FLAG_PERIPHAHB_MASK
#define DMACH_FLAG_PERIPHWIDTH_SHIFT        (9)       /* Bits 9-10: Peripheral width */
#define DMACH_FLAG_PERIPHWIDTH_MASK         (3 << DMACH_FLAG_PERIPHWIDTH_SHIFT)
#  define DMACH_FLAG_PERIPHWIDTH_8BITS      (0 << DMACH_FLAG_PERIPHWIDTH_SHIFT) /* 8 bits */
#  define DMACH_FLAG_PERIPHWIDTH_16BITS     (1 << DMACH_FLAG_PERIPHWIDTH_SHIFT) /* 16 bits */
#  define DMACH_FLAG_PERIPHWIDTH_32BITS     (2 << DMACH_FLAG_PERIPHWIDTH_SHIFT) /* 32 bits */
#  define DMACH_FLAG_PERIPHWIDTH_64BITS     (3 << DMACH_FLAG_PERIPHWIDTH_SHIFT) /* 64 bits */

#define DMACH_FLAG_PERIPHINCREMENT          (1 << 11) /* Bit 11: Auto-increment peripheral address */
#define DMACH_FLAG_PERIPHCHUNKSIZE_SHIFT    (12)      /* Bits 12-14: Peripheral chunk size */
#define DMACH_FLAG_PERIPHCHUNKSIZE_MASK     (7 << DMACH_FLAG_PERIPHCHUNKSIZE_SHIFT)
#  define DMACH_FLAG_PERIPHCHUNKSIZE_1      (0 << DMACH_FLAG_PERIPHCHUNKSIZE_SHIFT) /* Peripheral chunksize=1 */
#  define DMACH_FLAG_PERIPHCHUNKSIZE_2      (1 << DMACH_FLAG_PERIPHCHUNKSIZE_SHIFT) /* No chunksize=2 */
#  define DMACH_FLAG_PERIPHCHUNKSIZE_4      (2 << DMACH_FLAG_PERIPHCHUNKSIZE_SHIFT) /* Peripheral chunksize=4 */
#  define DMACH_FLAG_PERIPHCHUNKSIZE_8      (3 << DMACH_FLAG_PERIPHCHUNKSIZE_SHIFT) /* Peripheral chunksize=8 */
#  define DMACH_FLAG_PERIPHCHUNKSIZE_16     (4 << DMACH_FLAG_PERIPHCHUNKSIZE_SHIFT) /* Peripheral chunksize=16 */

/* Bits 16-19: Memory endpoint characteristics
 *
 * .... .... .... MMMM .... .... .... ....  Memory endpoint characteristics
 * .... .... .... .... .... .... .... ....  No memory peripheral ID,
 *                                           range 0-49
 * .... .... .... .... .... .... .... ....  No HW Handshaking
 * .... .... .... .... .... .... .... ....  No peripheral-to-peripheral
 * .... .... .... ...N .... .... .... ....  Memory ABH layer number
 * .... .... .... .... .... .... .... ....  No memory width
 * .... .... .... ..A. .... .... .... ....  Auto-increment memory address
 * .... .... .... .... .... .... .... ....  No memory chunk size
 * .... .... .... BB.. .... .... .... ....  Memory burst size
 */

#define DMACH_FLAG_MEMPID(n)                (0)       /* No memory peripheral identifier */
#  define DMACH_FLAG_MEMPID_MAX             (0)
#define DMACH_FLAG_MEMH2SEL                 (0)       /* No HW handshaking */
#define DMACH_FLAG_MEMISPERIPH              (0)       /* No peripheral-to-peripheral */
#define DMACH_FLAG_MEMAHB_MASK              (1 << 16) /* Bit 16: Memory ABH layer 1 */
#  define DMACH_FLAG_MEMAHB_AHB_IF0         (0)
#  define DMACH_FLAG_MEMAHB_AHB_IF1         DMACH_FLAG_MEMAHB_MASK

#define DMACH_FLAG_MEMWIDTH_8BITS           (0) /* Only peripheral data width */
#define DMACH_FLAG_MEMWIDTH_16BITS          (0)
#define DMACH_FLAG_MEMWIDTH_32BITS          (0)
#define DMACH_FLAG_MEMWIDTH_64BITS          (0)

#define DMACH_FLAG_MEMINCREMENT             (1 << 17) /* Bit 17: Auto-increment memory address */

#define DMACH_FLAG_MEMCHUNKSIZE_1           (0) /* Only peripheral chunk size */
#define DMACH_FLAG_MEMCHUNKSIZE_2           (0)
#define DMACH_FLAG_MEMCHUNKSIZE_4           (0)
#define DMACH_FLAG_MEMCHUNKSIZE_8           (0)
#define DMACH_FLAG_MEMCHUNKSIZE_16          (0)

#define DMACH_FLAG_MEMBURST_SHIFT           (18)      /* Bits 18-19: Memory burst size */
#define DMACH_FLAG_MEMBURST_MASK            (3 << DMACH_FLAG_MEMBURST_SHIFT)
#  define DMACH_FLAG_MEMBURST_1             (0 << DMACH_FLAG_MEMBURST_SHIFT)
#  define DMACH_FLAG_MEMBURST_4             (1 << DMACH_FLAG_MEMBURST_SHIFT)
#  define DMACH_FLAG_MEMBURST_8             (2 << DMACH_FLAG_MEMBURST_SHIFT)
#  define DMACH_FLAG_MEMBURST_16            (3 << DMACH_FLAG_MEMBURST_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef void *DMA_HANDLE;
typedef void (*dma_callback_t)(DMA_HANDLE handle, void *arg, int result);

/* The following is used for sampling DMA registers when CONFIG DEBUG_DMA is
 * selected
 */

#ifdef CONFIG_DEBUG_DMA_INFO
struct sam_dmaregs_s
{
  /* Global Registers.
   *
   * This includes all readable global XDMAC registers except for the global
   * interrupt status register (XDMAC_GIS).  Reading from the status
   * register could cause loss of interrupts.
   */

  uint32_t gtype;   /* Global Type Register */
  uint32_t gcfg;    /* Global Configuration Register */
  uint32_t gwac;    /* Global Weighted Arbiter Configuration Register */
  uint32_t gim;     /* Global Interrupt Mask Register */
  uint32_t gs;      /* Global Channel Status Register */
  uint32_t grs;     /* Global Channel Read Suspend Register */
  uint32_t gws;     /* Global Channel Write Suspend Register */
  uint32_t gsws;    /* Global Channel Software Request Status Register */

  /* Channel Registers
   *
   * This includes all readable XDMAC channel registers except for the
   * channel interrupt status register (XDMAC_CIS).  Reading from the status
   * register could cause loss of interrupts.
   */

  uint32_t cim;     /* Channel Interrupt Mask Register */
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
#endif /* CONFIG_DEBUG_DMA_INFO */

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: sam_destaddr
 *
 * Description:
 *   Returns the pointer to the destionation address, i.e the last address
 *   data were written by DMA.
 *
 * Assumptions:
 *   - DMA handle allocated by sam_dmachannel()
 *
 ****************************************************************************/

size_t sam_destaddr(DMA_HANDLE handle);

/****************************************************************************
 * Name: sam_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function sets aside a DMA channel then
 *   gives the caller exclusive access to the DMA channel.
 *
 *   The naming convention in all of the DMA interfaces is that one side is
 *   the 'peripheral' and the other is 'memory'.  However, the interface
 *   could still be used if, for example, both sides were memory although
 *   the naming would be awkward.
 *
 * Returned Value:
 *   If a DMA channel is available, this function returns a non-NULL,
 *   void* DMA channel handle.  NULL is returned on any failure.
 *
 ****************************************************************************/

DMA_HANDLE sam_dmachannel(uint8_t dmacno, uint32_t chflags);

/****************************************************************************
 * Name: sam_dmaconfig
 *
 * Description:
 *   There are two channel usage models:  (1) The channel is allocated and
 *   configured in one step.  This is the typical case where a DMA channel
 *   performs a constant role.  The alternative is (2) where the DMA channel
 *   is reconfigured on the fly. In this case, the chflags provided to
 *   sam_dmachannel are not used and sam_dmaconfig() is called before each
 *   DMA to configure the DMA channel appropriately.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_dmaconfig(DMA_HANDLE handle, uint32_t chflags);

/****************************************************************************
 * Name: sam_dmafree
 *
 * Description:
 *   Release a DMA channel.
 *   NOTE:  The 'handle' used in this argument must NEVER be used again until
 *   sam_dmachannel() is called again to re-gain a valid handle.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_dmafree(DMA_HANDLE handle);

/****************************************************************************
 * Name: sam_dmatxsetup
 *
 * Description:
 *   Configure DMA for transmit of one buffer (memory to peripheral).
 *   This function may be called multiple times to handle large and/or
 *   discontinuous transfers.
 *   Calls to sam_dmatxsetup() and sam_dmarxsetup() must not be intermixed
 *   on the same transfer, however.
 *
 ****************************************************************************/

int sam_dmatxsetup(DMA_HANDLE handle, uint32_t paddr,
                   uint32_t maddr, size_t nbytes);

/****************************************************************************
 * Name: sam_dmarxsetup
 *
 * Description:
 *   Configure DMA for receipt of one buffer (peripheral to memory).  This
 *   function may be called multiple times to handle large and/or
 *   discontinuous transfers. Calls to sam_dmatxsetup() and sam_dmarxsetup()
 *   must not be intermixed on the same transfer, however.
 *
 ****************************************************************************/

int sam_dmarxsetup(DMA_HANDLE handle, uint32_t paddr,
                   uint32_t maddr, size_t nbytes);

/****************************************************************************
 * Name: sam_dmarxsetup_circular
 *
 * Description:
 *   Configure DMA for receipt of two circular buffers for peripheral to
 *   memory transfer. Function sam_dmastart_circular() needs to be called
 *   to start the transfer. Only peripheral to memory transfer is currently
 *   supported.
 *
 * Input Parameters:
 *   handle - DMA handler
 *   descr - array with DMA descriptors
 *   maddr - array of memory addresses (i.e. destination addresses)
 *   paddr - peripheral address (i.e. source address)
 *   nbytes - number of bytes to transfer
 *   ndescrs - number of descriptors (i.e. the lenght of descr array)
 *
 ****************************************************************************/

int sam_dmarxsetup_circular(DMA_HANDLE handle,
                            struct chnext_view1_s *descr[],
                            uint32_t maddr[],
                            uint32_t paddr,
                            size_t nbytes,
                            uint8_t ndescrs);

/****************************************************************************
 * Name: sam_dmastart_circular
 *
 * Description:
 *   Start the DMA transfer with circular buffers.
 *
 ****************************************************************************/

int sam_dmastart_circular(DMA_HANDLE handle, dma_callback_t callback,
                          void *arg);

/****************************************************************************
 * Name: sam_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 ****************************************************************************/

int sam_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg);

/****************************************************************************
 * Name: sam_dmastop
 *
 * Description:
 *   Cancel the DMA.
 *   After sam_dmastop() is called, the DMA channel is reset and
 *   sam_dmarx/txsetup() must be called before sam_dmastart() can be
 *   called again
 *
 ****************************************************************************/

void sam_dmastop(DMA_HANDLE handle);

/****************************************************************************
 * Name: sam_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
void sam_dmasample(DMA_HANDLE handle, struct sam_dmaregs_s *regs);
#else
#  define sam_dmasample(handle,regs)
#endif

/****************************************************************************
 * Name: sam_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
void sam_dmadump(DMA_HANDLE handle,
                 const struct sam_dmaregs_s *regs, const char *msg);
#else
#  define sam_dmadump(handle,regs,msg)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_SAMV7_SAM_XDMAC_H */
