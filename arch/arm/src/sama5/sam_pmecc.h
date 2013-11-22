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
