/****************************************************************************
 * arch/arm/src/sama5/sam_pmecc.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   SAMA5D3 Series Data Sheet
 *   Atmel NoOS sample code.
 *
 * The Atmel sample code has a BSD compatibile license that requires this
 * copyright notice:
 *
 *   Copyright (c) 2012, Atmel Corporation
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/mtd/nand_config.h>

#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <assert.h>

#include "sam_pmecc.h"
#include "sam_nand.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Number of bits of correction.  These much match the (unshifted) values
 * in the SMC_PMECCFG register BCH_ERR field.
 */

#define BCH_ERR2  0 /* 2 bit errors */
#define BCH_ERR4  1 /* 4 bit errors */
#define BCH_ERR8  2 /* 8 bit errors */
#define BCH_ERR12 3 /* 12 bit errors */
#define BCH_ERR24 4 /* 24 bit errors */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* PMECC state data */

struct sam_pmecc_s
{
  bool configured;           /* True: Configured for some HSMC NAND bank */
#if NAND_NPMECC_BANKS > 1
  sem_t exclem;              /* For mutually exclusive access to the PMECC */
  uint8_t cs;                /* Currently configured for this bank */
#endif
  bool    sector1k;          /* True: 1024B sector size; False: 512B sector size */
  uint8_t nsectors;          /* Number of sectors per page */
  uint8_t correctability;    /* Number of correctable bits per sector */
  struct pmecc_desc_s desc;  /* Atmel PMECC descriptor */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

 /****************************************************************************
 * Private Data
 ****************************************************************************/
/* PMECC state data */

static struct sam_pmecc_s g_pmecc;

/* Maps BCH_ERR correctability register value to number of errors per sector */

static const uint8_t g_correctability[5] = {2, 4, 8, 12, 24};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pmecc_bcherr512
 *
 * Description:
 *   Get the correctabity that could be achieved using a 512 byte sector
 *
 ****************************************************************************/

static int pmecc_bcherr512(uint8_t nsectors, uint16_t sparesize)
{
  /* 39-bytes per 512 byte sector are required correctability of 24 errors */

  if (sparesize <= 39 * ((unsigned int)nsectors))
    {
      return BCH_ERR24;
    }

  /* 20-bytes per 512 byte sector are required correctability of 12 errors */

  else if (sparesize <= (20 * (unsigned int)nsectors))
    {
      return BCH_ERR12;
    }

  /* 13-bytes per 512 byte sector are required correctability of 8 errors */

  else if (sparesize <= (13 * (unsigned int)nsectors))
    {
      return BCH_ERR8;
    }
  
  /* 7-bytes per 512 byte sector are required correctability of 4 errors */

  else if (sparesize <= (7 *(unsigned int) nsectors))
    {
      return BCH_ERR4;
    }

  /* 4-bytes per 512 byte sector are required correctability of 2 errors */

  else if (sparesize <= (4 *(unsigned int) nsectors))
    {
      return BCH_ERR2;
    }
  
  return 0;
}

/****************************************************************************
 * Name: pmecc_bcherr512
 *
 * Description:
 *   Get the correctabity that could be achieved using a 512 byte sector
 *
 ****************************************************************************/

static int pmecc_bcherr1k(uint8_t nsectors, uint16_t sparesize)
{
  /* 42-bytes per 1024 byte sector are required correctability of 24 errors */

  if (sparesize <= 42 * ((unsigned int)nsectors))
    {
      return BCH_ERR24;
    }

  /* 21-bytes per 1024 byte sector are required correctability of 12 errors */

  else if (sparesize <= (20 * (unsigned int)nsectors))
    {
      return BCH_ERR12;
    }

  /* 14-bytes per 1024 byte sector are required correctability of 8 errors */

  else if (sparesize <= (13 * (unsigned int)nsectors))
    {
      return BCH_ERR8;
    }
  
  /* 7-bytes per 1024 byte sector are required correctability of 4 errors */

  else if (sparesize <= (7 *(unsigned int) nsectors))
    {
      return BCH_ERR4;
    }

  /* 4-bytes per 1024 byte sector are required correctability of 2 errors */

  else if (sparesize <= (4 *(unsigned int) nsectors))
    {
      return BCH_ERR2;
    }
  
  return 0;
}

/****************************************************************************
 * Name: pmecc_pagelayout
 *
 * Description:
 *   Given the data size and the spare size, determine the optimal sector
 *   size and correctability.
 *
 ****************************************************************************/

static void pmecc_pagelayout(uint16_t datasize, uint16_t sparesize,
                             uint16_t offset)
{
  uint16_t correctability512;
  uint16_t correctability1K;
  uint8_t nsectors512;
  uint8_t nsectors1k;
  uint8_t bcherr512;
  uint8_t bcherr1k;
  uint8_t bcherr;

  /* ECC must not start at address zero, since bad block tags are at offset
   * zero.
   */

  DEBUGASSERT(offset > 0);

  /* Decrease the spare size by the offset */

  sparesize -= offset;

  /* Try for 512 byte sectors */

  DEBUGASSERT((datasize & 0xfffffe00) == 0 && datasize >= 512);
  
  nsectors512 = (datasize >> 9);
  bcherr512   = pmecc_bcherr512(nsectors512, sparesize);

  /* Try for 1024 byte sectors */

  if ((datasize & 0xfffffc00) == 0)
    {
      nsectors1k = (datasize >> 9);
      bcherr1k   = pmecc_bcherr1k(nsectors1k, sparesize);
    }
  else
    {
      nsectors1k = 0;
      bcherr1k   = 0;
    }

  /* Now pick the best (most likely 1024) */

  DEBUGASSERT(bcherr512 > 0 || bcherr1k > 0);
  if (bcherr1k == 0)
    {
      g_pmecc.sector1k = false;
      g_pmecc.nsectors = nsectors512;
      bcherr           = bcherr512;
    }
  else
    {
      correctability512 = nsectors512 * g_correctability[bcherr512];
      correctability1K  = nsectors1k * g_correctability[bcherr1k];
      if (correctability512 >= correctability1K)
        {
          g_pmecc.sector1k = false;
          g_pmecc.nsectors = nsectors512;
          bcherr           = bcherr512;
        }
      else
        {
          g_pmecc.sector1k = true;
          g_pmecc.nsectors = nsectors1k;
          bcherr           = bcherr1k;
        }
    }

  /* Save the correctability value */

  g_pmecc.correctability = g_correctability[bcherr];

  /* And the correctly shifted BCH_ERR register value */

  g_pmecc.desc.bcherr = ((uint32_t)bcherr << HSMC_PMECCFG_BCHERR_SHIFT);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
void pmecc_initialize(void)
{
  sem_init(&g_pmecc.exclsem, 0, 1);
}
#endif

/****************************************************************************
 * Name: pmecc_configure
 *
 * Description:
 *   Configure the PMECC for use by this CS
 *
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
void pmecc_lock(void)
{
  int ret;

  do
    {
      ret = sem_wait(&g_pmecc.exclsem);
      DEBUGASSERT(ret == OK || errno == EINTR);
    }
  while (ret != OK);
}
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
void pmecc_unlock(void)
{
  sem_post(&g_pmecc.exclsem);
}
#endif

