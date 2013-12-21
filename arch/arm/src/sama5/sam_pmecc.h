/****************************************************************************
 * arch/arm/src/sama5/sam_pmecc.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This logic was based largely on Atmel sample code with modifications for
 * better integration with NuttX.  The Atmel sample code has a BSD
 * compatible license that requires this copyright notice:
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

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* Block checking and H/W ECC support must be enabled for PMECC */

#ifndef CONFIG_MTD_NAND_HWECC
#  undef CONFIG_SAMA5_EBICS0_PMECC
#  undef CONFIG_SAMA5_EBICS1_PMECC
#  undef CONFIG_SAMA5_EBICS2_PMECC
#  undef CONFIG_SAMA5_EBICS3_PMECC
#endif

/* Only CS3 can support NAND.  The rest of what follows is a fantasy */

# undef CONFIG_SAMA5_EBICS0_NAND
# undef CONFIG_SAMA5_EBICS1_NAND
# undef CONFIG_SAMA5_EBICS2_NAND

# undef CONFIG_SAMA5_EBICS0_PMECC
# undef CONFIG_SAMA5_EBICS1_PMECC
# undef CONFIG_SAMA5_EBICS2_PMECC

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

/* Count the number of banks that configured for NAND with PMECC support
 * enabled.
 */

#undef CONFIG_SAMA5_HAVE_PMECC
#ifdef CONFIG_SAMA5_EBICS0_PMECC
#  define CONFIG_SAMA5_HAVE_PMECC 1
#  define NAND_HAVE_EBICS0_PMECC 1
#else
#  define NAND_HAVE_EBICS0_PMECC 0
#endif

#ifdef CONFIG_SAMA5_EBICS1_PMECC
#  define CONFIG_SAMA5_HAVE_PMECC 1
#  define NAND_HAVE_EBICS1_PMECC 1
#else
#  define NAND_HAVE_EBICS1_PMECC 0
#endif

#ifdef CONFIG_SAMA5_EBICS2_PMECC
#  define CONFIG_SAMA5_HAVE_PMECC 1
#  define NAND_HAVE_EBICS2_PMECC 1
#else
#  define NAND_HAVE_EBICS2_PMECC 0
#endif

#ifdef CONFIG_SAMA5_EBICS3_PMECC
#  define CONFIG_SAMA5_HAVE_PMECC 1
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

#ifdef CONFIG_SAMA5_HAVE_PMECC

/* Maximum PMECC size */

#ifndef CONFIG_MTD_NAND_MAX_PMECCSIZE
#  define CONFIG_MTD_NAND_MAX_PMECCSIZE        200
#endif

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

/* Gallois Field Tables *****************************************************/

/* Indexes of tables in Gallois Field tables */

#define PMECC_GF_INDEX_OF     0
#define PMECC_GF_ALPHA_TO     1

/* Gallois Field tables for 512 and 1024 bytes sectors
 * First raw is "index_of" and second one is "alpha_to"
 */

#define PMECC_GF_SIZEOF_512   0x2000
#define PMECC_GF_SIZEOF_1024  0x4000

/****************************************************************************
 * Public Types
 ****************************************************************************/

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

/* Gallois Field tables for 512 bytes sectors. First raw is "index_of" and
 * second one is "alpha_to"
 */

#ifdef CONFIG_SAMA5_PMECC_GALOIS_TABLE512_ROMADDR
#  ifndef CONFIG_SAMA5_PMECC_GALOIS_TABLE512_ROMADDR
#     error CONFIG_SAMA5_PMECC_GALOIS_TABLE512_ROMADDR is not defined
#  endif
#  define pmecc_gf512  ((const int16_t *)CONFIG_SAMA5_PMECC_GALOIS_TABLE512_ROMADDR)

#  ifndef CONFIG_SAMA5_PMECC_GALOIS_TABLE1024_ROMADDR
#     error CONFIG_SAMA5_PMECC_GALOIS_TABLE1024_ROMADDR is not defined
#  endif
#  define pmecc_gf1024 ((const int16_t *)CONFIG_SAMA5_PMECC_GALOIS_TABLE1024_ROMADDR)

#else
  EXTERN const uint16_t pmecc_gf512[2][PMECC_GF_SIZEOF_512];
  EXTERN const uint16_t pmecc_gf1024[2][PMECC_GF_SIZEOF_1024];
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

/****************************************************************************
 * Name: pmecc_initialize
 *
 * Description:
 *   Perform one-time PMECC initialization.  This must be called before any
 *   other PMECC interfaces are used.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if NAND_NPMECC_BANKS > 1
void pmecc_initialize(void);
#else
#  define pmecc_initialize()
#endif

/****************************************************************************
 * Name: pmecc_configure
 *
 * Description:
 *   Configure and Initialize the PMECC peripheral for this CS.
 *
 * Input Parameters:
 *  priv      - Pointer to a struct sam_nandcs_s instance.
 *  protected - True:  The spare area is protected with the last sector of
 *                     data.
 *              False: The spare area is skipped in read or write mode.
 *
 * Returned Value:
 *  OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

struct sam_nandcs_s;
int pmecc_configure(struct sam_nandcs_s *priv, bool protected);

/****************************************************************************
 * Name: pmecc_correction
 *
 * Description:
 *   Perform the PMECC correction algorithm
 *
 * Input Parameters:
 *   isr  - Value of the PMECC ISR register
 *   data - Data to be corrected
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 * Assumptions:
 *  PMECC has been initialized for the CS and the caller holds the PMECC
 *  lock.
 *
 ****************************************************************************/

int pmecc_correction(uint32_t isr, uintptr_t data);

/****************************************************************************
 * Name: pmecc_get*
 *
 * Description:
 *   Various PMECC accessor functions
 *
 *   pmecc_get_eccsize()  - Returns the raw ECS size in bytes
 *   pmecc_get_pagesize() - Returns encoded HSMC_PMECCFG_PAGESIZE_* value
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *  The requested value
 *
 * Assumptions:
 *  PMECC has been initialized for the CS and the caller holds the PMECC
 *  lock.
 *
 ****************************************************************************/

uint32_t pmecc_get_eccsize(void);
uint32_t pmecc_get_pagesize(void);

/****************************************************************************
 * Name: pmecc_buildgf
 *
 * Description:
 *   This function is able to build Galois Field.
 *
 * Input Parameters:
 *   mm      - Degree of the remainders.
 *   indexof - Pointer to a buffer for indexof table.
 *   alphato - Pointer to a buffer for alphato table.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_PMECC_GALOIS_CUSTOM
void pmecc_buildgf(uint32_t mm, int16_t* indexof, int16_t* alphato);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#else /* CONFIG_SAMA5_HAVE_PMECC */
/****************************************************************************/
/* Stub definitions to minimize conditional compilation when PMECC is
 * disabled
 */

#  define pmecc_lock()
#  define pmecc_unlock()
#  define pmecc_enable()
#  define pmecc_disable()
#  define pmecc_initialize()
#  define pmecc_configure(a,b)   (0)
#  define pmecc_get_eccsize()    (0)
#  define pmecc_get_pagesize()   (0)

#endif /* CONFIG_SAMA5_HAVE_PMECC */
#endif /* __ARCH_ARM_SRC_SAMA5_PMECC_H */
