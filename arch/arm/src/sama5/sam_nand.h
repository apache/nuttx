/****************************************************************************
 * arch/arm/src/sama5/sam_nand.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMA5_SAM_NAND_H
#define __ARCH_ARM_SRC_SAMA5_SAM_NAND_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/mtd/nand_config.h>

#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <debug.h>

#include <nuttx/mtd/nand_raw.h>

#include "up_arch.h"
#include "chip.h"
#include "chip/sam_hsmc.h"

#include "sam_dmac.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
#ifndef CONFIG_SAMA5_DMAC1
#  warning CONFIG_SAMA5_DMAC1 should be enabled for DMA transfers
#endif

/* Hardware ECC types.  These are extensions to the NANDECC_HWECC value
 * defined in include/nuttx/mtd/nand_raw.h.
 *
 *   NANDECC_CHIPECC ECC is performed internal to chip
 *   NANDECC_PMECC   Programmable Multibit Error Correcting Code (PMECC)
 */

#define NANDECC_CHIPECC (NANDECC_HWECC + 0)
#define NANDECC_PMECC   (NANDECC_HWECC + 1)

/* Per NAND bank ECC selections */

#if defined(CONFIG_SAMA5_EBICS0_NAND)
#  if defined(CONFIG_SAMA5_EBICS0_ECCNONE)
#    define SAMA5_EBICS0_ECCTYPE NANDECC_NONE

#  elif defined(CONFIG_SAMA5_EBICS0_SWECC)
#    if !defined(CONFIG_MTD_NAND_BLOCKCHECK) || !defined(CONFIG_MTD_NAND_SWECC)
#      error CONFIG_SAMA5_EBICS0_SWECC is an invalid selection
#    endif
#    define SAMA5_EBICS0_ECCTYPE NANDECC_SWECC

#  elif defined(CONFIG_SAMA5_EBICS0_PMECC)
#    if !defined(CONFIG_MTD_NAND_BLOCKCHECK) || !defined(CONFIG_MTD_NAND_HWECC)
#      error CONFIG_SAMA5_EBICS0_PMECC is an invalid selection
#    endif
#    define SAMA5_EBICS0_ECCTYPE NANDECC_PMECC

#  elif defined(CONFIG_SAMA5_EBICS0_CHIPECC)
#    if !defined(CONFIG_MTD_NAND_BLOCKCHECK) || !defined(CONFIG_MTD_NAND_EMBEDDEDECC)
#      error CONFIG_SAMA5_EBICS0_CHIPECC is an invalid selection
#    endif
#    define SAMA5_EBICS0_ECCTYPE NANDECC_CHIPECC

#  else
#    error "No ECC type specified for CS0"
#  endif
#endif /* CONFIG_SAMA5_EBICS0_NAND */

#if defined(CONFIG_SAMA5_EBICS1_NAND)
#  if defined(CONFIG_SAMA5_EBICS1_ECCNONE)
#    define SAMA5_EBICS1_ECCTYPE NANDECC_NONE

#  elif defined(CONFIG_SAMA5_EBICS1_SWECC)
#    if !defined(CONFIG_MTD_NAND_BLOCKCHECK) || !defined(CONFIG_MTD_NAND_SWECC)
#      error CONFIG_SAMA5_EBICS1_SWECC is an invalid selection
#    endif
#    define SAMA5_EBICS1_ECCTYPE NANDECC_SWECC

#  elif defined(CONFIG_SAMA5_EBICS1_PMECC)
#    if !defined(CONFIG_MTD_NAND_BLOCKCHECK) || !defined(CONFIG_MTD_NAND_HWECC)
#      error CONFIG_SAMA5_EBICS1_PMECC is an invalid selection
#    endif
#    define SAMA5_EBICS1_ECCTYPE NANDECC_PMECC

#  elif defined(CONFIG_SAMA5_EBICS1_CHIPECC)
#    if !defined(CONFIG_MTD_NAND_BLOCKCHECK) || !defined(CONFIG_MTD_NAND_EMBEDDEDECC)
#      error CONFIG_SAMA5_EBICS1_CHIPECC is an invalid selection
#    endif
#    define SAMA5_EBICS1_ECCTYPE NANDECC_CHIPECC

#  else
#    error "No ECC type specified for CS1"
#  endif
#endif /* CONFIG_SAMA5_EBICS1_NAND */

#if defined(CONFIG_SAMA5_EBICS2_NAND)
#  if defined(CONFIG_SAMA5_EBICS2_ECCNONE)
#    define SAMA5_EBICS2_ECCTYPE NANDECC_NONE

#  elif defined(CONFIG_SAMA5_EBICS2_SWECC
#    if !defined(CONFIG_MTD_NAND_BLOCKCHECK) || !defined(CONFIG_MTD_NAND_SWECC)
#      error CONFIG_SAMA5_EBICS2_SWECC is an invalid selection
#    endif
#    define SAMA5_EBICS2_ECCTYPE NANDECC_SWECC

#  elif defined(CONFIG_SAMA5_EBICS2_PMECC)
#    if !defined(CONFIG_MTD_NAND_BLOCKCHECK) || !defined(CONFIG_MTD_NAND_HWECC)
#      error CONFIG_SAMA5_EBICS2_PMECC is an invalid selection
#    endif
#    define SAMA5_EBICS2_ECCTYPE NANDECC_PMECC

#  elif defined(CONFIG_SAMA5_EBICS2_CHIPECC)
#    if !defined(CONFIG_MTD_NAND_BLOCKCHECK) || !defined(CONFIG_MTD_NAND_EMBEDDEDECC)
#      error CONFIG_SAMA5_EBICS2_CHIPECC is an invalid selection
#    endif
#    define SAMA5_EBICS2_ECCTYPE NANDECC_CHIPECC

#  else
#    error "No ECC type specified for CS2"
#  endif
#endif /* CONFIG_SAMA5_EBICS2_NAND */

#if defined(CONFIG_SAMA5_EBICS3_NAND)
#  if defined(CONFIG_SAMA5_EBICS3_ECCNONE)
#    define SAMA5_EBICS3_ECCTYPE NANDECC_NONE

#  elif defined(CONFIG_SAMA5_EBICS3_SWECC)
#    if !defined(CONFIG_MTD_NAND_BLOCKCHECK) || !defined(CONFIG_MTD_NAND_SWECC)
#      error CONFIG_SAMA5_EBICS3_SWECC is an invalid selection
#    endif
#    define SAMA5_EBICS3_ECCTYPE NANDECC_SWECC

#  elif defined(CONFIG_SAMA5_EBICS3_PMECC)
#    if !defined(CONFIG_MTD_NAND_BLOCKCHECK) || !defined(CONFIG_MTD_NAND_HWECC)
#      error CONFIG_SAMA5_EBICS3_PMECC is an invalid selection
#    endif
#    define SAMA5_EBICS3_ECCTYPE NANDECC_PMECC

#  elif defined(CONFIG_SAMA5_EBICS3_CHIPECC)
#    if !defined(CONFIG_MTD_NAND_BLOCKCHECK) || !defined(CONFIG_MTD_NAND_EMBEDDEDECC)
#      error CONFIG_SAMA5_EBICS3_CHIPECC is an invalid selection
#    endif
#    define SAMA5_EBICS3_ECCTYPE NANDECC_CHIPECC

#  else
#    error "No ECC type specified for CS3"
#  endif
#endif /* CONFIG_SAMA5_EBICS3_NAND */

/****************************************************************************
 * Public Types
 ****************************************************************************/
/* This type represents the state of a raw NAND MTD device on a single chip
 * select.  The struct nand_raw_s must appear at the beginning of the
 * definition so that you can freely cast between pointers to struct
 * nand_raw_s and struct sam_nandcs_s.
 */

struct sam_nandcs_s
{
  struct nand_raw_s raw;     /* Externally visible part of the driver */

  /* Static configuration */

  uint8_t cs;                /* Chip select number (0..3) */

  /* Dynamic state */

  volatile bool cmddone;     /* True:  NFC commnad has completed */
  volatile bool xfrdone;     /* True:  Transfer has completed */
  volatile bool rbedge;      /* True:  Ready/busy edge detected */
  volatile bool dmadone;     /* True:  DMA has completed */
  sem_t waitsem;             /* Used to wait for one of the above states */

  DMA_HANDLE dma;            /* DMA channel assigned to this CS */
  int result;                /* The result of the DMA */
};

struct sam_nand_s
{
  bool initialized;          /* True:  One time initialization is complete */

#ifdef NAND_HAVE_PMECC
  uint8_t ecctab[CONFIG_MTD_NAND_MAX_PMECCSIZE];
#endif

#ifdef CONFIG_SAMA5_NAND_REGDEBUG
  /* Register debug state */

   bool wr;                 /* Last was a write */
   uint32_t regadddr;       /* Last address */
   uint32_t regval;         /* Last value */
   int ntimes;              /* Number of times */
#endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/* NAND global state */

EXTERN struct sam_nand_s g_nand;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_nand_initialize
 *
 * Description:
 *   Create and initialize an raw NAND device instance.  This driver
 *   implements the RAW NAND interface:  No software ECC or sparing is
 *   performed here.  Those necessary NAND features are provided by common,
 *   higher level NAND MTD layers found in drivers/mtd.
 *
 * Input parameters:
 *   cs - Chip select number (in the event that multiple NAND devices
 *        are connected on-board).
 *
 * Returned value.
 *   On success a non-NULL pointer to an MTD device structure is returned;
 *   NULL is returned on a failure.
 *
 ****************************************************************************/

struct mtd_dev_s;
struct mtd_dev_s *sam_nand_initialize(int cs);

/****************************************************************************
 * Name: board_nandflash_config
 *
 * Description:
 *   If CONFIG_SAMA5_BOOT_CS3FLASH is defined, then NAND FLASH support is
 *   enabled.  This function provides the board-specific implementation of
 *   the logic to reprogram the SMC to support NAND FLASH on the specified
 *   CS.
 *
 * Input Parameters:
 *   cs - Chip select number (in the event that multiple NAND devices
 *        are connected on-board).
 *
 * Returned Values:
 *   OK if the HSMC was successfully configured for this CS.  A negated
 *   errno value is returned on a failure.  This would fail with -ENODEV,
 *   for example, if the board does not support NAND FLASH on the requested
 *   CS.
 *
 ****************************************************************************/

int board_nandflash_config(int cs);

/****************************************************************************
 * Name: board_nand_busy
 *
 * Description:
 *   Must be provided if the board logic supports and interface to detect
 *   NAND Busy/Ready signal.
 *
 * Input Parameters:
 *   cs - Chip select number (in the event that multiple NAND devices
 *        are connected on-board).
 *
 * Returned Values:
 *   True:  NAND is busy, False: NAND is ready
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_NAND_READYBUSY
bool board_nand_busy(int cs);
#endif

/****************************************************************************
 * Name: board_nandflash_config
 *
 * Description:
 *   Must be provided if the board logic supports and interface to control
 *   the NAND Chip Enable signal.
 *
 * Input Parameters:
 *   cs - Chip select number (in the event that multiple NAND devices
 *        are connected on-board).
 *   enable - True: enable Chip Select, False: Disable Chip select
 *
 * Returned Values:
 *   OK if the HSMC was successfully configured for this CS.  A negated
 *   errno value is returned on a failure.  This would fail with -ENODEV,
 *   for example, if the board does not support NAND FLASH on the requested
 *   CS.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_NAND_CE
void board_nand_ce(int cs, bool enable);
#endif

/****************************************************************************
 * Name: nand_checkreg
 *
 * Description:
 *   Check if the current HSMC register access is a duplicate of the preceding.
 *
 * Input Parameters:
 *   regval   - The value to be written
 *   regaddr - The address of the register to write to
 *
 * Returned Value:
 *   true:  This is the first register access of this type.
 *   flase: This is the same as the preceding register access.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_NAND_REGDEBUG
bool nand_checkreg(bool wr, uintptr_t regaddr, uint32_t regval);
#endif

/****************************************************************************
 * Name: nand_getreg
 *
 * Description:
 *  Read an HSMC register
 *
 ****************************************************************************/

static inline uint32_t nand_getreg(uintptr_t regaddr)
{
  uint32_t regval = getreg32(regaddr);

#ifdef CONFIG_SAMA5_NAND_REGDEBUG
  if (nand_checkreg(false, regaddr, regval))
    {
      lldbg("%08x->%08x\n", regaddr, regval);
    }
#endif

  return regval;
}

/****************************************************************************
 * Name: nand_putreg
 *
 * Description:
 *  Write a value to an HSMC register
 *
 ****************************************************************************/

static inline void nand_putreg(uintptr_t regaddr, uint32_t regval)
{
#ifdef CONFIG_SAMA5_NAND_REGDEBUG
  if (nand_checkreg(true, regaddr, regval))
    {
      lldbg("%08x<-%08x\n", regaddr, regval);
    }
#endif

  putreg32(regval, regaddr);
}

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_SAMA5_SAM_NAND_H */
