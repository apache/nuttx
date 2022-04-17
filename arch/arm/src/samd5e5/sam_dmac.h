/****************************************************************************
 * arch/arm/src/samd5e5/sam_dmac.h
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

#ifndef __ARCH_ARM_SRC_SAMD5E5_SAM_DMAC_H
#define __ARCH_ARM_SRC_SAMD5E5_SAM_DMAC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "hardware/sam_dmac.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DMA **********************************************************************/

/* Flags used to characterize the desired DMA channel.  The naming convention
 * is that one side is the peripheral and the other is memory (however, the
 * interface could still be used if, for example, both sides were memory
 * although the naming would be awkward)
 */

/* Common characteristics
 *
 *   BEATSIZE  - The size of one bus transfer or "beat".
 *                 8-, 16-, or 32-bits
 *   STEPSEL   - The STEPSIZE may be applied only to the memory to the
 *                 peripheral.
 *   STEPSIZE  - When the address is incremented, it is increments by
 *                 how many "beats"?
 *   PRIORITY  - DMA transfer priority
 *   RUNSTDBY  - DMA runs in standby mode
 *   BURSTLEN  - Burst length.
 *   THRESHOLD - Threshold at which DMA starts to write
 *                 (multi-BEAT transfers only)
 */

#define DMACH_FLAG_BEATSIZE_SHIFT         (0)       /* Bits 0-1: Beat size */
#define DMACH_FLAG_BEATSIZE_MASK          (3 << DMACH_FLAG_BEATSIZE_SHIFT)
#  define DMACH_FLAG_BEATSIZE_BYTE        (0 << DMACH_FLAG_BEATSIZE_SHIFT) /* 8-bit bus transfer */
#  define DMACH_FLAG_BEATSIZE_HWORD       (1 << DMACH_FLAG_BEATSIZE_SHIFT) /* 16-bit bus transfer */
#  define DMACH_FLAG_BEATSIZE_WORD        (2 << DMACH_FLAG_BEATSIZE_SHIFT) /* 32-bit bus transfer */

#define DMACH_FLAG_STEPSEL                (1 << 2) /* Bit 2: Step selection */
#  define DMACH_FLAG_STEPSEL_MEM          (0)      /*        0=Step size applies to memory */
#  define DMACH_FLAG_STEPSEL_PERIPH       (1 << 2) /*        1=Step size applies to peripheral */
#define DMACH_FLAG_STEPSIZE_SHIFT         (3)      /* Bits 3-5: Address increment step */
#define DMACH_FLAG_STEPSIZE_MASK          (7 << DMACH_FLAG_STEPSIZE_SHIFT)
#  define DMACH_FLAG_STEPSIZE_X1          (0 << DMACH_FLAG_STEPSIZE_SHIFT) /* Next ADDR = ADDR + (BEATSIZE+1) * 1 */
#  define DMACH_FLAG_STEPSIZE_X2          (1 << DMACH_FLAG_STEPSIZE_SHIFT) /* Next ADDR = ADDR + (BEATSIZE+1) * 2 */
#  define DMACH_FLAG_STEPSIZE_X4          (2 << DMACH_FLAG_STEPSIZE_SHIFT) /* Next ADDR = ADDR + (BEATSIZE+1) * 4 */
#  define DMACH_FLAG_STEPSIZE_X8          (3 << DMACH_FLAG_STEPSIZE_SHIFT) /* Next ADDR = ADDR + (BEATSIZE+1) * 8 */
#  define DMACH_FLAG_STEPSIZE_X16         (4 << DMACH_FLAG_STEPSIZE_SHIFT) /* Next ADDR = ADDR + (BEATSIZE+1) * 16 */
#  define DMACH_FLAG_STEPSIZE_X32         (5 << DMACH_FLAG_STEPSIZE_SHIFT) /* Next ADDR = ADDR + (BEATSIZE+1) * 32 */
#  define DMACH_FLAG_STEPSIZE_X64         (6 << DMACH_FLAG_STEPSIZE_SHIFT) /* Next ADDR = ADDR + (BEATSIZE+1) * 64 */
#  define DMACH_FLAG_STEPSIZE_X128        (7 << DMACH_FLAG_STEPSIZE_SHIFT) /* Next ADDR = ADDR + (BEATSIZE+1) * 128 */

#define DMACH_FLAG_PRIORITY_SHIFT         (6)       /* Bit 6-7: Arbitration priority */
#define DMACH_FLAG_PRIORITY_MASK          (3 << DMACH_FLAG_PRIORITY_SHIFT)
#  define DMACH_FLAG_PRIORITY(n)          ((uint32_t)(n) << DMACH_FLAG_PRIORITY_SHIFT)
#define DMACH_FLAG_RUNINSTDBY             (1 << 8)  /* Bit 8: Run in standby */
#define DMACH_FLAG_BURSTLEN_SHIFT         (9)       /* Bits 9-12: Burst length */
#define DMACH_FLAG_BURSTLEN_MASK         (15 << DMACH_FLAG_BURSTLEN_SHIFT)
#  define DMACH_FLAG_BURSTLEN(n)          ((uint32_t)(n) << DMACH_FLAG_BURSTLEN_SHIFT) /* n=1-16 */

#define DMACH_FLAG_THRESHOLD_SHIFT        (13)      /* Bits 13-14: Threshold */
#define DMACH_FLAG_THRESHOLD_MASK         (15 << DMACH_FLAG_THRESHOLD_SHIFT)
#  define DMACH_FLAG_THRESHOLD_1BEAT      (0 << DMACH_FLAG_THRESHOLD_SHIFT) /* Write after 1 beat */
#  define DMACH_FLAG_THRESHOLD_2BEATS     (1 << DMACH_FLAG_THRESHOLD_SHIFT) /* Write after 2 beats */
#  define DMACH_FLAG_THRESHOLD_4BEATS     (2 << DMACH_FLAG_THRESHOLD_SHIFT) /* Write after 3 beats */
#  define DMACH_FLAG_THRESHOLD_8BEATS     (3 << DMACH_FLAG_THRESHOLD_SHIFT) /* Write after 8 beats */

/* Peripheral endpoint characteristics.
 *
 * PERIPH_TRIGSRC   - The TX ID of the peripheral that provides the DMA
 *                    trigger.
 *                    This is one of the DMAC_CHCTRLA_TRIGSRC_*[_TX]
 *                    definitions.  This trigger source is selected when
 *                    sam_dmatxsetup() is called.
 * PERIPH_TRIGACT   - Trigger action
 * PERIPH_INCREMENT - Indicates that the peripheral address should be
 *                    incremented on each "beat"
 */

#define DMACH_FLAG_PERIPH_TRIGSRC_SHIFT   (15)       /* Bits 15-21: See DMAC_CHCTRLA_TRIGSRC_* */
#define DMACH_FLAG_PERIPH_TRIGSRC_MASK    (0x7f << DMACH_FLAG_PERIPH_TRIGSRC_SHIFT)
#  define DMACH_FLAG_PERIPH_TRIGSRC(n)    ((uint32_t)(n) << DMACH_FLAG_PERIPH_TRIGSRC_SHIFT)
#define DMACH_FLAG_PERIPH_TRIGACT_SHIFT   (22)      /* Bits 22-23:  Tx trigger action */
#define DMACH_FLAG_PERIPH_TRIGACT_MASK    (3 << DMACH_FLAG_PERIPH_TRIGACT_SHIFT)
#  define DMACH_FLAG_PERIPH_TRIGACT_BLOCK (0 << DMACH_FLAG_PERIPH_TRIGACT_SHIFT) /* Trigger per block transfer */
#  define DMACH_FLAG_PERIPH_TRIGACT_BURST (2 << DMACH_FLAG_PERIPH_TRIGACT_SHIFT) /* Trigger per burst transfer */
#  define DMACH_FLAG_PERIPH_TRIGACT_TRANS (3 << DMACH_FLAG_PERIPH_TRIGACT_SHIFT) /* Trigger for each transaction */

#define DMACH_FLAG_PERIPH_INCREMENT       (1 << 24) /* Bit 24: Autoincrement peripheral address */

/* Memory endpoint characteristics
 *
 * MEM_INCREMENT - Indicates that the memory address should be incremented
 *                 on each "beat"
 */

#define DMACH_FLAG_MEM_INCREMENT          (1 << 25) /* Bit 25: Autoincrement memory address */
                                                    /* Bits 26-31: Not used */

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef void *DMA_HANDLE;
typedef void (*dma_callback_t)(DMA_HANDLE handle, void *arg, int result);

/* The following is used for sampling DMA registers when CONFIG DEBUG_DMA
 * is selected
 */

#ifdef CONFIG_DEBUG_DMA_INFO
struct sam_dmaregs_s
{
  uint8_t  chan;             /* Channel index */

  /* DMAC Registers */

  uint8_t  crcstatus;        /* CRC Status Register */
  uint8_t  dbgctrl;          /* Debug Control Register */
  uint8_t  chid;             /* Channel ID Register */
  uint8_t  chctrlb;          /* Channel Control B Register */
  uint8_t  chpirlvl;         /* Channel Priority Level */
  uint8_t  chevctrl;         /* Channel Event Control Register */
  uint8_t  chintflag;        /* Channel Interrupt Flag Status and Clear Register */
  uint8_t  chstatus;         /* Channel Status Register */

  uint16_t ctrl;             /* Control Register */
  uint16_t crcctrl;          /* CRC Control Register */
  uint16_t intpend;          /* Interrupt Pending Register */

  uint32_t crcdatain;        /* CRC Data Input Register */
  uint32_t crcchksum;        /* CRC Checksum Register */
  uint32_t swtrigctrl;       /* Software Trigger Control Register */
  uint32_t prictrl0;         /* Priority Control 0 Register */
  uint32_t intstatus;        /* Interrupt Status Register */
  uint32_t busych;           /* Busy Channels Register */
  uint32_t pendch;           /* Pending Channels Register */
  uint32_t active;           /* Active Channels and Levels Register */
  uint32_t baseaddr;         /* Descriptor Memory Section Base Address Register */
  uint32_t wrbaddr;          /* Write-Back Memory Section Base Address Register */
  uint32_t chctrla;          /* Channel Control A Register */
};
#endif

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
 * Name: sam_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function sets aside a DMA channel and
 *   gives the caller exclusive access to the DMA channel.
 *
 *   The naming convention in all of the DMA interfaces is that one side is
 *   the 'peripheral' and the other is 'memory'.  However, the interface
 *   could still be used if, for example, both sides were memory although
 *   the naming would be awkward.
 *
 * Returned Value:
 *   If a DMA channel if the required FIFO size is available, this function
 *   returns a non-NULL, void* DMA channel handle.  NULL is returned on any
 *   failure.
 *
 ****************************************************************************/

DMA_HANDLE sam_dmachannel(uint32_t txflags, uint32_t rxflags);

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

void sam_dmaconfig(DMA_HANDLE handle, uint32_t txflags, uint32_t rxflags);

/****************************************************************************
 * Name: sam_dmafree
 *
 * Description:
 *   Release a DMA channel.
 *   NOTE:  The 'handle' used in this argument must NEVER be used again
 *  until sam_dmachannel() is called again to re-gain a valid handle.
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
 *   Configure DMA for transmit of one buffer (memory to peripheral).  This
 *   function may be called multiple times to handle large and/or
 *   non-contiguous transfers. Calls to sam_dmatxsetup() and sam_dmarxsetup()
 *   must not be intermixed on the same transfer, however.
 *
 ****************************************************************************/

int sam_dmatxsetup(DMA_HANDLE handle, uint32_t paddr, uint32_t maddr,
                   size_t nbytes);

/****************************************************************************
 * Name: sam_dmarxsetup
 *
 * Description:
 *   Configure DMA for receipt of one buffer (peripheral to memory).  This
 *   function may be called multiple times to handle large and/or
 *   non-contiguous transfers. Calls to sam_dmatxsetup() and sam_dmarxsetup()
 *   must not be intermixed on the same transfer, however.
 *
 ****************************************************************************/

int sam_dmarxsetup(DMA_HANDLE handle, uint32_t paddr, uint32_t maddr,
                   size_t nbytes);

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
 *   Cancel the DMA.  After sam_dmastop() is called, the DMA channel is
 *   reset and sam_dmarx/txsetup() must be called before sam_dmastart()
 *   can be called again
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
void sam_dmadump(DMA_HANDLE handle, const struct sam_dmaregs_s *regs,
                 const char *msg);
#else
#  define sam_dmadump(handle,regs,msg)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_SAMD5E5_SAM_DMAC_H */
