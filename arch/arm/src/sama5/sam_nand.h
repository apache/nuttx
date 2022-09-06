/****************************************************************************
 * arch/arm/src/sama5/sam_nand.h
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

#ifndef __ARCH_ARM_SRC_SAMA5_SAM_NAND_H
#define __ARCH_ARM_SRC_SAMA5_SAM_NAND_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/mtd/nand_config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/mtd/nand_raw.h>
#include <nuttx/mutex.h>

#include "arm_internal.h"
#include "chip.h"
#include "hardware/sam_hsmc.h"

#include "sam_dmac.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* DMA.  DMA support requires that DMAC0 be enabled.  According to
 * "Table 15-2. SAMA5 Master to Slave Access", DMAC1 does not have access
 * to NFC SRAM.
 */

#ifdef CONFIG_SAMA5_NAND_DMA
#  ifdef CONFIG_SAMA5_DMAC0
#    define NAND_DMAC 0
#  else
#    error "DMA controller 0 (DMAC0) must be enabled to perform DMA transfers"
#    undef CONFIG_SAMA5_NAND_DMA
#  endif
#endif

/* If memory-to-memory DMAs are used, then two context switches will occur:
 * (1) when the NAND logic waits for the DMA to complete, and (2) again when
 * the DMA completes and the NAND logic is re-awakened.  Each context switch
 * will required saving and restoring a set of registers defining the task
 * state.  Those register include the PSR, 16 general purpose registers, and
 * 32 floating point registers or about 196 bytes per task state.  That is
 * then 392*2 bytes per context and 784 bytes for both.  Plus there is
 * processing overhead. So certainly, there is no reason to use a memory-to-
 * memory DMA transfer for much smaller blocks of data.
 */

#ifdef CONFIG_SAMA5_NAND_DMA
#  ifndef CONFIG_SAMA5_NAND_DMA_THRESHOLD
#    define CONFIG_SAMA5_NAND_DMA_THRESHOLD 784
#  endif
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

/* Only CS3 can support NAND.  The rest is a fantasy */

# undef CONFIG_SAMA5_EBICS0_NAND
# undef CONFIG_SAMA5_EBICS1_NAND
# undef CONFIG_SAMA5_EBICS2_NAND

#if defined(CONFIG_SAMA5_EBICS0_NAND)
#  if defined(CONFIG_SAMA5_EBICS0_ECCNONE)
#    define SAMA5_EBICS0_ECCTYPE NANDECC_NONE

#  elif defined(CONFIG_SAMA5_EBICS0_SWECC)
#    ifndef CONFIG_MTD_NAND_SWECC
#      error CONFIG_SAMA5_EBICS0_SWECC is an invalid selection
#    endif
#    define SAMA5_EBICS0_ECCTYPE NANDECC_SWECC

#  elif defined(CONFIG_SAMA5_EBICS0_PMECC)
#    ifndef CONFIG_MTD_NAND_HWECC
#      error CONFIG_SAMA5_EBICS0_PMECC is an invalid selection
#    endif
#    define SAMA5_EBICS0_ECCTYPE NANDECC_PMECC

#  elif defined(CONFIG_SAMA5_EBICS0_CHIPECC)
#    ifndef CONFIG_MTD_NAND_EMBEDDEDECC
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
#    ifndef CONFIG_MTD_NAND_SWECC
#      error CONFIG_SAMA5_EBICS1_SWECC is an invalid selection
#    endif
#    define SAMA5_EBICS1_ECCTYPE NANDECC_SWECC

#  elif defined(CONFIG_SAMA5_EBICS1_PMECC)
#    ifndef CONFIG_MTD_NAND_HWECC
#      error CONFIG_SAMA5_EBICS1_PMECC is an invalid selection
#    endif
#    define SAMA5_EBICS1_ECCTYPE NANDECC_PMECC

#  elif defined(CONFIG_SAMA5_EBICS1_CHIPECC)
#    ifndef CONFIG_MTD_NAND_EMBEDDEDECC
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
#    ifndef CONFIG_MTD_NAND_SWECC
#      error CONFIG_SAMA5_EBICS2_SWECC is an invalid selection
#    endif
#    define SAMA5_EBICS2_ECCTYPE NANDECC_SWECC

#  elif defined(CONFIG_SAMA5_EBICS2_PMECC)
#    ifndef CONFIG_MTD_NAND_HWECC
#      error CONFIG_SAMA5_EBICS2_PMECC is an invalid selection
#    endif
#    define SAMA5_EBICS2_ECCTYPE NANDECC_PMECC

#  elif defined(CONFIG_SAMA5_EBICS2_CHIPECC)
#    ifndef CONFIG_MTD_NAND_EMBEDDEDECC
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
#    ifndef CONFIG_MTD_NAND_SWECC
#      error CONFIG_SAMA5_EBICS3_SWECC is an invalid selection
#    endif
#    define SAMA5_EBICS3_ECCTYPE NANDECC_SWECC

#  elif defined(CONFIG_SAMA5_EBICS3_PMECC)
#    ifndef CONFIG_MTD_NAND_HWECC
#      error CONFIG_SAMA5_EBICS3_PMECC is an invalid selection
#    endif
#    define SAMA5_EBICS3_ECCTYPE NANDECC_PMECC

#  elif defined(CONFIG_SAMA5_EBICS3_CHIPECC)
#    ifndef CONFIG_MTD_NAND_EMBEDDEDECC
#      error CONFIG_SAMA5_EBICS3_CHIPECC is an invalid selection
#    endif
#    define SAMA5_EBICS3_ECCTYPE NANDECC_CHIPECC

#  else
#    error "No ECC type specified for CS3"
#  endif
#endif /* CONFIG_SAMA5_EBICS3_NAND */

/* Count the number of banks that configured for NAND with PMECC support
 * enabled.
 */

#undef CONFIG_SAMA5_HAVE_NAND
#ifdef CONFIG_SAMA5_EBICS0_NAND
#  define CONFIG_SAMA5_HAVE_NAND 1
#  define NAND_HAVE_EBICS0 1
#else
#  define NAND_HAVE_EBICS0 0
#endif

#ifdef CONFIG_SAMA5_EBICS1_NAND
#  define CONFIG_SAMA5_HAVE_NAND 1
#  define NAND_HAVE_EBICS1 1
#else
#  define NAND_HAVE_EBICS1 0
#endif

#ifdef CONFIG_SAMA5_EBICS2_NAND
#  define CONFIG_SAMA5_HAVE_NAND 1
#  define NAND_HAVE_EBICS2 1
#else
#  define NAND_HAVE_EBICS2 0
#endif

#ifdef CONFIG_SAMA5_EBICS3_NAND
#  define CONFIG_SAMA5_HAVE_NAND 1
#  define NAND_HAVE_EBICS3 1
#else
#  define NAND_HAVE_EBICS3 0
#endif

#ifdef CONFIG_SAMA5_HAVE_NAND

/* Count the number of banks configured for NAND */

#define NAND_NBANKS \
   (NAND_HAVE_EBICS0 + NAND_HAVE_EBICS1 + NAND_HAVE_EBICS2 + NAND_HAVE_EBICS3)

/* Debug */

#ifndef defined(CONFIG_DEBUG_FS_INFO
#  undef CONFIG_SAMA5_NAND_DMADEBUG
#  undef CONFIG_SAMA5_NAND_REGDEBUG
#  undef CONFIG_SAMA5_NAND_DUMP
#endif

#if !defined(CONFIG_SAMA5_NAND_DMA) || !defined(CONFIG_DEBUG_DMA_INFO)
#  undef CONFIG_SAMA5_NAND_DMADEBUG
#endif

/* An early version of this driver used SMC interrupts to determine when
 * NAND commands completed, transfers completed, and RB edges occurred.  It
 * turns out that those interrupts occurred so quickly that some really
 * nasty race conditions were created.  Rather than resolve those, I simply
 * disabled the interrupt logic with this setting.  The setting is retained
 * in case, for some reason, someone wants to restore the interrupt-driven
 * logic. Polling should be better solution in this case.
 */

#undef CONFIG_SAMA5_NAND_HSMCINTERRUPTS

/* DMA Debug */

#define DMA_INITIAL      0
#define DMA_AFTER_SETUP  1
#define DMA_AFTER_START  2
#define DMA_CALLBACK     3
#define DMA_TIMEOUT      3 /* No timeout */
#define DMA_END_TRANSFER 4
#define DMA_NSAMPLES     5

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This type represents the state of a raw NAND MTD device on a single chip
 * select.  The struct nand_raw_s must appear at the beginning of the
 * definition so that you can freely cast between pointers to struct
 * nand_raw_s and struct sam_nandcs_s.
 *
 * NOTE: Currently, only SAMA5D3x CS3 can support NAND.  The logic here would
 * support NAND on any CS, but that capability is not needed.
 */

struct sam_nandcs_s
{
  struct nand_raw_s raw;     /* Externally visible part of the driver */

  /* Static configuration */

  uint8_t cs;                /* Chip select number (0..3) */
#ifdef CONFIG_SAMA5_NAND_DMA
  volatile bool dmadone;     /* True:  DMA has completed */
#endif

#ifdef CONFIG_SAMA5_PMECC_TRIMPAGE
  bool dropjss;              /* Enable page trimming */
  uint16_t trimpage;         /* Trim page number boundary */
#endif

#ifdef CONFIG_SAMA5_NAND_DMA
  sem_t waitsem;             /* Used to wait for DMA done */
  DMA_HANDLE dma;            /* DMA channel assigned to this CS */
  int result;                /* The result of the DMA */

#ifdef CONFIG_SAMA5_NAND_DMADEBUG
  struct sam_dmaregs_s dmaregs[DMA_NSAMPLES];
#endif
#endif
};

struct sam_nand_s
{
  bool initialized;          /* True:  One time initialization is complete */
#if NAND_NBANKS > 1
  mutex_t lock;              /* Enforce exclusive access to the SMC hardware */
#endif

  /* Dynamic state */

#ifdef CONFIG_SAMA5_NAND_HSMCINTERRUPTS
  volatile bool cmddone;     /* True:  NFC command has completed (latching) */
  volatile bool xfrdone;     /* True:  Transfer has completed (latching)  */
  volatile bool rbedge;      /* True:  Ready/busy edge detected (latching) */
  sem_t waitsem;             /* Used to wait for one of the above states */

#else
  bool cmddone;              /* True:  NFC command has completed (latching)  */
  bool xfrdone;              /* True:  Transfer has completed (latching)  */
  bool rbedge;               /* True:  Ready/busy edge detected (latching) */

#endif

#ifdef CONFIG_SAMA5_HAVE_PMECC
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
extern "C"
{
#else
#define EXTERN extern
#endif

/* NAND global state */

EXTERN struct sam_nand_s g_nand;

/****************************************************************************
 * Public Functions Prototypes
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
 * Input Parameters:
 *   cs - Chip select number (in the event that multiple NAND devices
 *        are connected on-board).
 *
 * Returned Value:
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
 *   CS.  As a minimum, this board-specific initialization should do the
 *   following:
 *
 *     1. Enable clocking to the HSMC
 *     2. Configure timing for the HSMC CS
 *     3. Configure NAND PIO pins
 *
 * Input Parameters:
 *   cs - Chip select number (in the event that multiple NAND devices
 *        are connected on-board).
 *
 * Returned Value:
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
 * Returned Value:
 *   True:  NAND is busy, False: NAND is ready
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_NAND_READYBUSY
bool board_nand_busy(int cs);
#endif

/****************************************************************************
 * Name: board_nand_ce
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
 * Returned Value:
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
 *   Check if the current HSMC register access is a duplicate of the
 *   preceding.
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
      sinfo("%08x->%08x\n", regaddr, regval);
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
      sinfo("%08x<-%08x\n", regaddr, regval);
    }
#endif

  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: nand_trimffs_enable
 *
 * Description:
 *   Set current trimffs status.
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_PMECC_TRIMPAGE
static inline void nand_trimffs_enable(struct sam_nandcs_s *priv,
                                       bool enable)
{
  priv->dropjss = enable;
}
#else
#  define nand_trimffs_enable(p,e)
#endif

/****************************************************************************
 * Name: nand_trrimffs
 *
 * Description:
 *   Get current trimffs status.
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_PMECC_TRIMPAGE
static inline bool nand_trrimffs(struct sam_nandcs_s *priv)
{
  return priv->dropjss;
}
#else
#  define nand_trrimffs(p) (false)
#endif

/****************************************************************************
 * Name: nand_set_trimpage
 *
 * Description:
 *   Set current trimffs page.
 *
 * Input Parameters:
 *   page - Start trim page.
 *
 * Returned Value:
 *
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_PMECC_TRIMPAGE
static inline void nand_set_trimpage(struct sam_nandcs_s *priv,
                                     uint16_t page)
{
  priv->trimpage = page;
}
#else
#  define nand_set_trimpage(p,t)
#endif

/****************************************************************************
 * Name: nand_get_trimpage
 *
 * Description:
 *   Get current trimffs page.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *    Start trim page.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_PMECC_TRIMPAGE
uint16_t nand_get_trimpage(struct sam_nandcs_s *priv)
{
  return priv->trimpage;
}
#else
#  define nand_get_trimpage(p) (0)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_SAMA5_HAVE_NAND */
#endif /* __ARCH_ARM_SRC_SAMA5_SAM_NAND_H */
