/****************************************************************************
 * arch/arm/src/sama5/sam_pmecc.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This logic was based largely on Atmel sample code with modifications for
 * better integration with NuttX.  The Atmel sample code has a BSD
 * compatibile license that requires this copyright notice:
 *
 *   Copyright (c) 2010, Atmel Corporation
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
 * 3. Neither the names NuttX nor Atmel nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
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

#ifndef __ARCH_ARM_SRC_SAMA5_PMECC_H
#define __ARCH_ARM_SRC_SAMA5_PMECC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/mtd/nand_config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* Block checking and H/W ECC support must be enabled for PMECC */

#if !defined(CONFIG_MTD_NAND_BLOCKCHECK) || !defined(CONFIG_MTD_NAND_HWECC)
#  undef CONFIG_SAMA5_EBICS0_PMECC
#  undef CONFIG_SAMA5_EBICS1_PMECC
#  undef CONFIG_SAMA5_EBICS2_PMECC
#  undef CONFIG_SAMA5_EBICS3_PMECC
#endif

/* Disable PMECC support for any banks not enabled or configured for NAND */

#if !defined(CONFIG_SAMA5_EBICS0) || !defined(CONFIG_SAMA5_EBICS0_NAND)
#  undef CONFIG_SAMA5_EBICS0_PMECC
#endif

#if !defined(CONFIG_SAMA5_EBICS1) || !defined(CONFIG_SAMA5_EBICS1_NAND)
#  undef CONFIG_SAMA5_EBICS1_PMECC
#endif

#if !defined(CONFIG_SAMA5_EBICS2) || !defined(CONFIG_SAMA5_EBICS2_NAND)
#  undef CONFIG_SAMA5_EBICS2_PMECC
#endif

#if !defined(CONFIG_SAMA5_EBICS3) || !defined(CONFIG_SAMA5_EBICS3_NAND)
#  undef CONFIG_SAMA5_EBICS3_PMECC
#endif

/* Count the number of banks that confaigured for NAND with PMECC support
 * enabled.
 */

#undef NAND_HAVE_PMECC
#ifdef CONFIG_SAMA5_EBICS0_PMECC
#  undef  NAND_HAVE_PMECC
#  define NAND_HAVE_PMECC 1
#  define NAND_HAVE_EBICS0_PMECC 1
#else
#  define NAND_HAVE_EBICS0_PMECC 0
#endif

#ifdef CONFIG_SAMA5_EBICS1_PMECC
#  undef  NAND_HAVE_PMECC
#  define NAND_HAVE_PMECC 1
#  define NAND_HAVE_EBICS1_PMECC 1
#else
#  define NAND_HAVE_EBICS1_PMECC 0
#endif

#ifdef CONFIG_SAMA5_EBICS2_PMECC
#  undef  NAND_HAVE_PMECC
#  define NAND_HAVE_PMECC 1
#  define NAND_HAVE_EBICS2_PMECC 1
#else
#  define NAND_HAVE_EBICS2_PMECC 0
#endif

#ifdef CONFIG_SAMA5_EBICS3_PMECC
#  undef  NAND_HAVE_PMECC
#  define NAND_HAVE_PMECC 1
#  define NAND_HAVE_EBICS3_PMECC 1
#else
#  define NAND_HAVE_EBICS3_PMECC 0
#endif

/* Count the number of banks using PMECC */

#define NAND_NPMECC_BANKS \
   (NAND_HAVE_EBICS0_PMECC + NAND_HAVE_EBICS1_PMECC + \
    NAND_HAVE_EBICS2_PMECC + NAND_HAVE_EBICS3_PMECC)

/* Compile this logic only if there is at least one CS configure for NAND
 * and with PMECC support enabled.
 */

#ifdef NAND_HAVE_PMECC

/* The ROM code embeds the software used in the process of ECC
 * detection/correction
 */

#ifdef CONFIG_SAMA5_PMECC_EMBEDDEDALGO_ADDR
#  ifndef CONFIG_SAMA5_PMECC_EMBEDDEDALGO_ADDR
#    define CONFIG_SAMA5_PMECC_EMBEDDEDALGO_ADDR 0x00104510
#  endif
#endif

#ifdef CONFIG_SAMA5_PMECC_GALOIS_ROMTABLES
#  ifndef CONFIG_SAMA5_PMECC_GALOIS_TABLE512_ROMADDR
#    define CONFIG_SAMA5_PMECC_GALOIS_TABLE512_ROMADDR 0x00110000
#  endif

#  ifndef CONFIG_SAMA5_PMECC_GALOIS_TABLE1024_ROMADDR
#    define CONFIG_SAMA5_PMECC_GALOIS_TABLE1024_ROMADDR 0x00118000
#  endif
#endif

/* Other PMECC Definitions **************************************************/
/* Defines the maximum value of the error correcting capability */

#define PMECC_MAX_CORRECTABILITY 25

/* Start address of ECC cvalue in spare zone, this must not be 0 since bad
 * block tags are at address 0.
 */

#define PMECC_ECC_STARTOFFSET 2

/****************************************************************************
 * Public Types
 ****************************************************************************/
/* This is the form of the PMECC descriptor that is passed to the ECC
 * detection correction algorithm in ROM.  The binary for of this structure
 * cannot be altered!
 */

struct pmecc_desc_s
{
  uint32_t pagesize;     /*   0-3: See HSMC_PMECCFG_PAGESIZE_* definitions */
  uint32_t sparesize;    /*   4-7: The spare area size is equal to (SPARESIZE+1) bytes */
  uint32_t sectorsz;     /*  8-11: See HSMC_PMECCFG_SECTORSZ_* definitions*/
  uint32_t bcherr;       /* 12-15: See HSMC_PMECCFG_BCHERR_* definitions */
  uint32_t eccsize;      /* 16-19: Real size in bytes of ECC in spare */
  uint32_t eccstart;     /* 20-23: The first byte address of the ECC area */
  uint32_t eccend;       /* 24-27: The last byte address of the ECC area */
  uint32_t nandwr;       /* 28-31: NAND Write Access */
  uint32_t sparena;      /* 32-35: Spare Enable */
  uint32_t automode;     /* 36-39: Automatic Mode */
  uint32_t clkctrl;      /* 40-43: PMECC Module data path Setup Time is CLKCTRL+1. */
  uint32_t interrupt;    /* 44-47: */
  int32_t tt;            /* 48-51: Error correcting capability */
  int32_t mm;            /* 52-55: Degree of the remainders, GF(2**mm) */
  int32_t nn;            /* 56-59: Length of codeword =  nn=2**mm -1 */
  int16_t *alphato;      /* 60-63: Gallois field table */
  int16_t *indexof;      /* 64-67: Index of Gallois field table */
  int16_t partsyn[100];  /* 68-267: */
  int16_t si[100];       /* 268-467: Current syndrome value */

  /* 468-: Sigma table */

  int16_t smu[PMECC_MAX_CORRECTABILITY + 2][2 * PMECC_MAX_CORRECTABILITY + 1];

  /* Polynomial order */

  int16_t lmu[PMECC_MAX_CORRECTABILITY + 1];
};

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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pmecc_lock
 *
 * Description:
 *   Get exclusive access to PMECC hardware
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if NAND_NPMECC_BANKS > 1
void pmecc_lock(void);
#else
#  define pmecc_lock()
#endif

/****************************************************************************
 * Name: pmecc_unlock
 *
 * Description:
 *   Relinquish exclusive access to PMECC hardware
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if NAND_NPMECC_BANKS > 1
void pmecc_unlock(void);
#else
#  define pmecc_unlock()
#endif

/****************************************************************************
 * Name: pmecc_enable
 *
 * Description:
 *   Enable PMECC
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void pmecc_enable(void);

/****************************************************************************
 * Name: pmecc_disable
 *
 * Description:
 *   Enable PMECC
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void pmecc_disable(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#else /* NAND_HAVE_PMECC */
/****************************************************************************/
/* Stub definitions to minimize conditional compilation when PMECC is
 * disabled
 */

#  define pmecc_lock()
#  define pmecc_unlock()
#  define pmecc_enable()
#  define pmecc_disable()

#endif /* NAND_HAVE_PMECC */
#endif /* __ARCH_ARM_SRC_SAMA5_PMECC_H */
