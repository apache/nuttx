/****************************************************************************
 * arch/arm/src/sama5/sam_pmecc.c
 *
 *   Copyright (C) 2013, 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * All of the detailed PMECC operations are taken directly from the Atmel
 * NoOS sample code.  The Atmel sample code has a BSD compatible license
 * that requires this copyright notice:
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

/* References:
 *   SAMA5D3 Series Data Sheet
 *   Atmel NoOS sample code.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/mtd/nand_config.h>

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/mtd/nand_model.h>
#include <nuttx/mtd/nand_scheme.h>
#include <nuttx/semaphore.h>

#include "sam_pmecc.h"
#include "sam_nand.h"

/* Compile this logic only if there is at least one CS configure for NAND
 * and with PMECC support enabled.
 */

#ifdef CONFIG_SAMA5_HAVE_PMECC

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

/* Defines the maximum value of the error correcting capability */

#define PMECC_MAX_CORRECTABILITY 25

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This is the form of the PMECC descriptor that is passed to the ECC
 * detection correction algorithm in ROM.  The binary for of this structure
 * cannot be altered!
 */

struct pmecc_desc_s
{
  uint32_t pagesize;     /*   0-3: See HSMC_PMECCFG_PAGESIZE_* definitions */
  uint32_t sparesize;    /*   4-7: The spare area size is equal to (SPARESIZE+1) bytes */
  uint32_t sectorsz;     /*  8-11: See HSMC_PMECCFG_SECTORSZ_* definitions */
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

  int16_t smu[PMECC_MAX_CORRECTABILITY + 2]
             [2 * PMECC_MAX_CORRECTABILITY + 1];

  /* Polynomial order */

  int16_t lmu[PMECC_MAX_CORRECTABILITY + 1];
};

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

/* This is the type of the ROM detection/correction function
 *
 * REVISIT:  Where are the types pmecc and pmerrloc?
 */

#ifdef CONFIG_SAMA5_PMECC_EMBEDDEDALGO
typedef uint32_t (*pmecc_correctionalgo_t)(pmecc *, pmerrloc *,
                                           struct pmecc_desc_s *desc,
                                           uint32_t isr, uintptr_t data);
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_SAMA5_PMECC_EMBEDDEDALGO
#  define pmecc_correctionalgo \
    ((pmecc_correctionalgo_t)CONFIG_SAMA5_PMECC_EMBEDDEDALGO_ADDR)
#else
static uint32_t pmecc_correctionalgo(uint32_t isr, uintptr_t data);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* PMECC state data */

static struct sam_pmecc_s g_pmecc;

/* Maps BCH_ERR correctability register value to number of errors per
 * sector.
 */

static const uint8_t g_correctability[5] =
{
  2, 4, 8, 12, 24
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pmecc_gensyn
 *
 * Description:
 *   Build the pseudo syndromes table
 *
 * Input Parameters:
 *   sector - Targeted sector.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SAMA5_PMECC_EMBEDDEDALGO
static void pmecc_gensyn(uint32_t sector)
{
  int16_t *remainder;
  int i;

  remainder = (int16_t *)SAM_HSMC_REM_BASE(sector);

  for (i = 0; i < (uint32_t)g_pmecc.desc.tt; i++)
    {
      /* Fill odd syndromes */

      g_pmecc.desc.partsyn[1 + (2 * i)] = remainder[i];
    }
}
#endif /* CONFIG_SAMA5_PMECC_EMBEDDEDALGO */

/****************************************************************************
 * Name: pmecc_substitute
 *
 * Description:
 *   The pmecc_substitute function evaluates the polynomial remainder, with
 *   different values of the field primitive elements.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (always)
 *
 ****************************************************************************/

#ifndef CONFIG_SAMA5_PMECC_EMBEDDEDALGO
static uint32_t pmecc_substitute(void)
{
  int16_t *si      = g_pmecc.desc.si;
  int16_t *partsyn = g_pmecc.desc.partsyn;
  int16_t *alphato = g_pmecc.desc.alphato;
  int16_t *indexof = g_pmecc.desc.indexof;
  int i;
  int j;

  /* si[] is a table that holds the current syndrome value, an element of
   * that table belongs to the field.
   */

  for (i = 1; i < 2 * PMECC_MAX_CORRECTABILITY; i++)
    {
      si[i] = 0;
    }

  /* Computation 2t syndromes based on S(x) */

  /* Odd syndromes */

  for (i = 1; i <= 2 * g_pmecc.desc.tt - 1; i = i + 2)
    {
      si[i] = 0;
      for (j = 0; j < g_pmecc.desc.mm; j++)
        {
          if (partsyn[i] & ((uint16_t)0x1 << j))
            {
              si[i] = alphato[(i * j)] ^ si[i];
            }
        }
    }

  /* Even syndrome = (Odd syndrome) ** 2 */

  for (i = 2; i <= 2 * g_pmecc.desc.tt; i = i + 2)
    {
      j = i / 2;
      if (si[j] == 0)
        {
          si[i] = 0;
        }
      else
        {
          si[i] = alphato[(2 * indexof[si[j]]) % g_pmecc.desc.nn];
        }
    }

  return 0;
}
#endif /* CONFIG_SAMA5_PMECC_EMBEDDEDALGO */

/****************************************************************************
 * Name: pmecc_getsigma
 *
 * Description:
 *   The substitute function finding the value of the error location
 *   polynomial.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (always)
 *
 ****************************************************************************/

#ifndef CONFIG_SAMA5_PMECC_EMBEDDEDALGO
static uint32_t pmecc_getsigma(void)
{
  uint32_t dmu0count;
  int16_t *lmu = g_pmecc.desc.lmu;
  int16_t *si = g_pmecc.desc.si;
  int16_t tt = g_pmecc.desc.tt;
  int32_t mu[PMECC_MAX_CORRECTABILITY + 1];     /* Mu */
  int32_t dmu[PMECC_MAX_CORRECTABILITY + 1];    /* Discrepancy */
  int32_t delta[PMECC_MAX_CORRECTABILITY + 1];  /* Delta order   */
  int32_t largest;
  int32_t diff;
  int ro;                                       /* Index of largest delta */
  int i;
  int j;
  int k;

  dmu0count = 0;

  /* First Row */

  /* Mu */

  mu[0] = -1; /* Actually -1/2 */

  /* Sigma(x) set to 1 */

  for (i = 0; i < (2 * PMECC_MAX_CORRECTABILITY + 1); i++)
    {
      g_pmecc.desc.smu[0][i] = 0;
    }

  g_pmecc.desc.smu[0][0] = 1;

  /* Discrepancy set to 1 */

  dmu[0] = 1;

  /* Polynom order set to 0 */

  lmu[0] = 0;

  /* delta set to -1 */

  delta[0] = (mu[0] * 2 - lmu[0]) >> 1;

  /* Second row */

  /* Mu */

  mu[1]  = 0;

  /* Sigma(x) set to 1 */

  for (i = 0; i < (2 * PMECC_MAX_CORRECTABILITY + 1); i++)
    {
      g_pmecc.desc.smu[1][i] = 0;
    }

  g_pmecc.desc.smu[1][0] = 1;

  /* Discrepancy set to S1 */

  dmu[1] = si[1];

  /* Polynom order set to 0 */

  lmu[1] = 0;

  /* Delta set to 0 */

  delta[1]  = (mu[1] * 2 - lmu[1]) >> 1;

  /* Initialize the Sigma(x) last row */

  for (i = 0; i < (2 * PMECC_MAX_CORRECTABILITY + 1); i++)
    {
      g_pmecc.desc.smu[tt + 1][i] = 0;
    }

  for (i = 1; i <= tt; i++)
    {
      mu[i + 1] = i << 1;

      /* Compute Sigma (Mu+1) and L(mu). */

      /* Check if discrepancy is set to 0 */

      if (dmu[i] == 0)
        {
          dmu0count++;
          if ((tt - (lmu[i] >> 1) - 1) & 0x1)
            {
              if (dmu0count == (uint32_t)((tt - (lmu[i] >> 1) - 1) / 2) + 2)
                {
                  for (j = 0; j <= (lmu[i] >> 1) + 1; j++)
                    {
                      g_pmecc.desc.smu[tt + 1][j] = g_pmecc.desc.smu[i][j];
                    }

                  lmu[tt + 1] = lmu[i];
                  return 0;
                }
            }
          else
            {
              if (dmu0count == (uint32_t)((tt - (lmu[i] >> 1) - 1) / 2) + 1)
                {
                  for (j = 0; j <= (lmu[i] >> 1) + 1; j++)
                    {
                      g_pmecc.desc.smu[tt + 1][j] = g_pmecc.desc.smu[i][j];
                    }

                  lmu[tt + 1] = lmu[i];
                  return 0;
                }
            }

          /* Copy polynom */

          for (j = 0; j <= lmu[i] >> 1; j++)
            {
              g_pmecc.desc.smu[i + 1][j] = g_pmecc.desc.smu[i][j];
            }

          /* Copy previous polynom order to the next */

          lmu[i + 1] = lmu[i];
        }
      else
        {
          ro = 0;
          largest = -1;

          /* find largest delta with dmu != 0 */

          for (j = 0; j < i; j++)
            {
              if (dmu[j])
                {
                  if (delta[j] > largest)
                    {
                      largest = delta[j];
                      ro = j;
                    }
                }
            }

          /* Compute difference */

          diff = (mu[i] - mu[ro]);

          /* Compute degree of the new smu polynomial */

          if ((lmu[i] >> 1) > ((lmu[ro] >> 1) + diff))
            {
              lmu[i + 1] = lmu[i];
            }
          else
            {
              lmu[i + 1] = ((lmu[ro] >> 1) + diff) * 2;
            }

          /* Init smu[i+1] with 0 */

          for (k = 0; k < (2 * PMECC_MAX_CORRECTABILITY + 1); k ++)
            {
              g_pmecc.desc.smu[i + 1][k] = 0;
            }

          /* Compute smu[i+1] */

          for (k = 0; k <= lmu[ro] >> 1; k++)
            {
              if (g_pmecc.desc.smu[ro][k] && dmu[i])
                {
                   g_pmecc.desc.smu[i + 1][k + diff] =
                     g_pmecc.desc.alphato[(g_pmecc.desc.indexof[dmu[i]] +
                     (g_pmecc.desc.nn - g_pmecc.desc.indexof[dmu[ro]]) +
                     g_pmecc.desc.indexof[g_pmecc.desc.smu[ro][k]]) %
                                          g_pmecc.desc.nn];
                }
            }

          for (k = 0; k <= lmu[i] >> 1; k++)
            {
              g_pmecc.desc.smu[i + 1][k] ^= g_pmecc.desc.smu[i][k];
            }
        }

      /* End Compute Sigma (Mu+1) and L(mu) */

      /* In either case compute delta */

      delta[i + 1]  = (mu[i + 1] * 2 - lmu[i + 1]) >> 1;

      /* Do not compute discrepancy for the last iteration */

      if (i < tt)
        {
          for (k = 0 ; k <= (lmu[i + 1] >> 1); k++)
            {
              if (k == 0)
                {
                  dmu[i + 1] = si[2 * (i - 1) + 3];
                }

              /* Check if one operand of the multiplier is null, its index
               * is -1
               */

              else if (g_pmecc.desc.smu[i + 1][k] &&
                       si[2 * (i - 1) + 3 - k])
                {
                  dmu[i + 1] =
                    g_pmecc.desc.alphato[
                      (g_pmecc.desc.indexof[g_pmecc.desc.smu[i + 1][k]] +
                    g_pmecc.desc.indexof[si[2 * (i - 1) + 3 - k]]) %
                      g_pmecc.desc.nn] ^ dmu[i + 1];
                }
            }
        }
    }

  return 0;
}
#endif /* CONFIG_SAMA5_PMECC_EMBEDDEDALGO */

/****************************************************************************
 * Name: pmecc_errorlocation
 *
 * Description:
 *   Initialize the PMECC Error Location peripheral and start the error
 *   location processing
 *
 * Input Parameters:
 *   bitsize - Size of the sector in bits.
 *
 * Returned Value:
 *   Number of errors (or -1 on an error)
 *
 ****************************************************************************/

#ifndef CONFIG_SAMA5_PMECC_EMBEDDEDALGO
static int32_t pmecc_errorlocation(uint32_t bitsize)
{
  uint32_t *sigma;
  uint32_t errornumber;
  uint32_t nroots;
  uint32_t regval;
  uint32_t alphax;

  /* Disable PMECC Error Location IP */

  nand_putreg(SAM_HSMC_ELDIS, 0xffffffff);

  errornumber = 0;
  alphax = 0;

  sigma = (uint32_t *)SAM_HSMC_SIGMA0;
  for (alphax = 0;
       alphax <= (uint32_t)(g_pmecc.desc.lmu[g_pmecc.desc.tt + 1] >> 1);
       alphax++)
    {
      *sigma++ = g_pmecc.desc.smu[g_pmecc.desc.tt + 1][alphax];
      errornumber++;
    }

  regval  = nand_getreg(SAM_HSMC_ELCFG);
  regval |= HSMC_ELCFG_ERRNUM(errornumber - 1);
  nand_putreg(SAM_HSMC_ELCFG, regval);

  /* Enable error location process */

  nand_putreg(SAM_HSMC_ELEN, bitsize);
  while ((nand_getreg(SAM_HSMC_ELISR) & HSMC_ELIINT_DONE) == 0);

  nroots = (nand_getreg(SAM_HSMC_ELISR) & HSMC_ELISR_ERRCNT_MASK) >>
            HSMC_ELISR_ERRCNT_SHIFT;

  /* Number of roots == degree of smu hence <= tt */

  if (nroots == (uint32_t)(g_pmecc.desc.lmu[g_pmecc.desc.tt + 1] >> 1))
    {
      return (errornumber - 1);
    }

  /* Number of roots not match the degree of smu ==> unable to correct
   * error
   */

  return -1;
}
#endif /* CONFIG_SAMA5_PMECC_EMBEDDEDALGO */

/****************************************************************************
 * Name: pmecc_errorcorrection
 *
 * Description:
 *   Correct errors indicated in the PMECCEL error location registers.
 *
 * Input Parameters:
 *   sectorbase - Base address of the sector.
 *   extrabytes - Number of extra bytes of the sector (encoded spare area,
 *                only for the last sector)
 *   nerrors Number of error to correct
 *
 * Returned Value:
 *   Number of errors
 *
 ****************************************************************************/

#ifndef CONFIG_SAMA5_PMECC_EMBEDDEDALGO
static uint32_t pmecc_errorcorrection(uintptr_t sectorbase,
                                      uint32_t extrabytes, uint32_t nerrors)
{
  uint32_t *errpos;
  uint32_t bytepos;
  uint32_t bitpos;
  uint32_t sectorsz;
  uint32_t eccsize;
  uint32_t eccend;

  errpos = (uint32_t *)SAM_HSMC_ERRLOC_BASE(0);

  if ((nand_getreg(SAM_HSMC_PMECCFG) & HSMC_PMECCFG_SECTORSZ_MASK) ==
       HSMC_PMECCFG_SECTORSZ_512)
    {
      sectorsz = 512;
    }
  else
    {
      sectorsz = 1024;
    }

  /* Get number of ECC bytes */

  eccend = nand_getreg(SAM_HSMC_PMECCEADDR);
  eccsize = (eccend - nand_getreg(SAM_HSMC_PMECCSADDR)) + 1;

  while (nerrors)
    {
      bytepos = (*errpos - 1) / 8;
      bitpos  = (*errpos - 1) % 8;

      /* If error is located in the data area (not in ECC) */

      if (bytepos < (sectorsz + extrabytes))
        {
          /* If the error position is before ECC area */

          if (bytepos < sectorsz + nand_getreg(SAM_HSMC_PMECCSADDR))
            {
              fwarn("WARNING: Correct error bit @[Byte %d, Bit %d]\n",
                      (int)bytepos, (int)bitpos);

              if (*(uint8_t *)(sectorbase + bytepos) & (1 << bitpos))
                {
                  *(uint8_t *)(sectorbase + bytepos) &=
                    (0xff ^ (1 << bitpos));
                }
              else
                {
                  *(uint8_t *)(sectorbase + bytepos) |=
                    (1 << bitpos);
                }
            }
          else
            {
              if (*(uint8_t *)(sectorbase + bytepos + eccsize) &
                  (1 << bitpos))
                {
                  *(uint8_t *)(sectorbase + bytepos + eccsize) &=
                    (0xff ^ (1 << bitpos));
                }
              else
                {
                  *(uint8_t *)(sectorbase + bytepos + eccsize) |=
                    (1 << bitpos);
                }
            }
        }

      errpos++;
      nerrors--;
    }

  return 0;
}
#endif /* CONFIG_SAMA5_PMECC_EMBEDDEDALGO */

/****************************************************************************
 * Name: pmecc_correctionalgo
 *
 * Description:
 *   Launch error detection functions and correct corrupted bits.
 *
 * Input Parameters:
 *   isr  - Value of the PMECC status register.
 *   data - Base address of the buffer containing the page to be corrected.
 *
 * Returned Value:
 *    0 if all errors have been corrected, 1 if too many errors detected
 *
 ****************************************************************************/

#ifndef CONFIG_SAMA5_PMECC_EMBEDDEDALGO
static uint32_t pmecc_correctionalgo(uint32_t isr, uintptr_t data)
{
  uintptr_t sectorbase;
  uint32_t sector = 0;
  uint32_t sectorsz;
  int32_t nerrors;
  unsigned int mm;

  /* Set the sector size (512 or 1024 bytes) */

  if ((g_pmecc.desc.sectorsz & HSMC_PMECCFG_SECTORSZ_MASK) != 0)
    {
      sectorsz = 1024;
      mm       = 14;
      nand_putreg(SAM_HSMC_ELCFG, HSMC_ELCFG_SECTORSZ_1024);
    }
  else
    {
      sectorsz = 512;
      mm       = 13;
      nand_putreg(SAM_HSMC_ELCFG, HSMC_ELCFG_SECTORSZ_512);
    }

#define HSMC_PAGESIZE \
  (1 << ((nand_getreg(SAM_HSMC_PMECCFG) & HSMC_PMECCFG_PAGESIZE_MASK) >> \
   HSMC_PMECCFG_PAGESIZE_SHIFT))

  while (sector < (uint32_t)HSMC_PAGESIZE && isr != 0)
    {
      nerrors = 0;
      if ((isr & 1) != 0)
        {
          sectorbase = data + (sector * sectorsz);

          pmecc_gensyn(sector);
          pmecc_substitute();
          pmecc_getsigma();

          /* Number of bits of the sector + ecc */

          nerrors = pmecc_errorlocation((sectorsz * 8) +
                    (g_pmecc.desc.tt * mm));
          if (nerrors == -1)
            {
              return 1;
            }
          else
            {
              /* Extra byte is 0 */

              pmecc_errorcorrection(sectorbase, 0, nerrors);
            }
        }

      sector++;
      isr = isr >> 1;
    }

  return 0;
}
#endif /* CONFIG_SAMA5_PMECC_EMBEDDEDALGO */

/****************************************************************************
 * Name: pmecc_bcherr512
 *
 * Description:
 *   Get the correctabity that could be achieved using a 512 byte sector
 *
 ****************************************************************************/

static int pmecc_bcherr512(uint8_t nsectors, uint16_t eccsize)
{
  /* 39-bytes per 512 byte sector are required correctability of 24 errors */

  if (eccsize >= 39 * ((unsigned int)nsectors))
    {
      return BCH_ERR24;
    }

  /* 20-bytes per 512 byte sector are required correctability of 12 errors */

  else if (eccsize >= (20 * (unsigned int)nsectors))
    {
      return BCH_ERR12;
    }

  /* 13-bytes per 512 byte sector are required correctability of 8 errors */

  else if (eccsize >= (13 * (unsigned int)nsectors))
    {
      return BCH_ERR8;
    }

  /* 7-bytes per 512 byte sector are required correctability of 4 errors */

  else if (eccsize >= (7 * (unsigned int) nsectors))
    {
      return BCH_ERR4;
    }

  /* 4-bytes per 512 byte sector are required correctability of 2 errors */

  else if (eccsize >= (4 * (unsigned int) nsectors))
    {
      return BCH_ERR2;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: pmecc_bcherr512
 *
 * Description:
 *   Get the correctabity that could be achieved using a 512 byte sector
 *
 ****************************************************************************/

static int pmecc_bcherr1k(uint8_t nsectors, uint16_t eccsize)
{
  /* 42-bytes per 1024 byte sector are required correctability of 24 errors */

  if (eccsize >= 42 * ((unsigned int)nsectors))
    {
      return BCH_ERR24;
    }

  /* 21-bytes per 1024 byte sector are required correctability of 12 errors */

  else if (eccsize >= (21 * (unsigned int)nsectors))
    {
      return BCH_ERR12;
    }

  /* 14-bytes per 1024 byte sector are required correctability of 8 errors */

  else if (eccsize >= (14 * (unsigned int)nsectors))
    {
      return BCH_ERR8;
    }

  /* 7-bytes per 1024 byte sector are required correctability of 4 errors */

  else if (eccsize >= (7 * (unsigned int) nsectors))
    {
      return BCH_ERR4;
    }

  /* 4-bytes per 1024 byte sector are required correctability of 2 errors */

  else if (eccsize >= (4 * (unsigned int) nsectors))
    {
      return BCH_ERR2;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: pmecc_pagelayout
 *
 * Description:
 *   Given the size of the data region and the size of the ECC region,
 *   determine the optimal sector size and correctability.
 *
 ****************************************************************************/

static int pmecc_pagelayout(uint16_t datasize, uint16_t eccsize)
{
  uint16_t correctability512;
  uint16_t correctability1k;
  uint8_t nsectors512;
  uint8_t nsectors1k;
  uint8_t bcherr;
  int bcherr512;
  int bcherr1k;
  int selector;

  finfo("datasize=%d eccsize=%d\n", datasize, eccsize);
  DEBUGASSERT(datasize > 0 && eccsize > 0);

  /* Try for 512 byte sectors */

  DEBUGASSERT((datasize & 0x000001ff) == 0 && datasize >= 512);

  selector    = 0;
  nsectors512 = (datasize >> 9);
  bcherr512   = pmecc_bcherr512(nsectors512, eccsize);
  if (bcherr512 < 0)
    {
      fwarn("WARNING: Cannot realize 512B sectors\n");
    }
  else
    {
      selector = 1;
    }

  finfo("nsectors512=%d bcherr512=%d selector=%d\n",
        nsectors512, bcherr512, selector);

  /* Try for 1024 byte sectors */

  if ((datasize & 0x000003ff) == 0)
    {
      nsectors1k = (datasize >> 10);
      bcherr1k   = pmecc_bcherr1k(nsectors1k, eccsize);
    }
  else
    {
      nsectors1k = 0;
      bcherr1k   = -EINVAL;
    }

  if (bcherr1k < 0)
    {
      fwarn("WARNING: Cannot realize 1KB sectors\n");
    }
  else
    {
      selector |= 2;
    }

  finfo("nsectors1k=%d bcherr1k=%d selector=%d\n",
        nsectors1k, bcherr1k, selector);

  /* Now pick the best (most likely 1024) */

  DEBUGASSERT(bcherr512 >= 0 || bcherr1k >= 0);
  switch (selector)
    {
      case 1:  /* 512B sectors possible; 1KB sectors not possible */
        {
          g_pmecc.sector1k = false;
          g_pmecc.nsectors = nsectors512;
          bcherr           = bcherr512;
          DEBUGASSERT(bcherr512 >= 0);
        }
        break;

      case 3:  /* Both 512B and 1KB sectors possible */
        {
          correctability512 = nsectors512 * g_correctability[bcherr512];
          correctability1k  = nsectors1k * g_correctability[bcherr1k];

          /* Use 512B sectors unless we can do better with 1K sectors */

          if (correctability512 >= correctability1k)
            {
              g_pmecc.sector1k = false;
              g_pmecc.nsectors = nsectors512;
              bcherr           = bcherr512;
              DEBUGASSERT(bcherr512 >= 0);
              break;
            }
        }

      /* Otherwise, fall through for the 1KB sectors */

      case 2:  /* 512B sectors not possible; 1KB sectors possible */
        {
          g_pmecc.sector1k = true;
          g_pmecc.nsectors = nsectors1k;
          bcherr           = bcherr1k;
          DEBUGASSERT(bcherr1k >= 0);
        }
        break;

      case 0: /* Either 512B and 1KB sectors possible */
      default:
        return -ENOSYS;
    }

  /* Save the correctability value */

  g_pmecc.correctability = g_correctability[bcherr];

  /* And the correctly shifted BCH_ERR register value */

  g_pmecc.desc.bcherr = ((uint32_t)bcherr << HSMC_PMECCFG_BCHERR_SHIFT);

  finfo("sector1k=%d nsectors=%d bcherr=%d correctability=%d\n",
        g_pmecc.sector1k, g_pmecc.nsectors, bcherr, g_pmecc.correctability);

  return OK;
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
  nxsem_init(&g_pmecc.exclsem, 0, 1);
}
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

int pmecc_configure(struct sam_nandcs_s *priv, bool protected)
{
  struct nand_model_s *model;
  unsigned int sectorsperpage = 0;
  uint16_t eccoffset;
  uint16_t eccsize;
  uint32_t regval;
  int ret;

  finfo("protected=%d configured=%d\n", protected, g_pmecc.configured);

  /* Check if we need to re-configure */

#if NAND_NPMECC_BANKS > 1
  if (g_pmecc.configured && g_pmecc.cs == priv->cs)
#else
  if (g_pmecc.configured)
#endif
    {
      /* No, we are already configured */

      finfo("Already configured\n");
      return OK;
    }

  /* Get a convenience pointer to the NAND model */

  model = &priv->raw.model;

  /* Get the offset and size of the ECC information in the spare area from
   * the NAND scheme.
   */

  DEBUGASSERT(model->scheme);
  eccoffset = nandscheme_eccoffset(model->scheme);
  eccsize   = nandscheme_eccsize(model->scheme);

  /* Get the number of sectors and the error correction per sector.  This
   * function will set the following structure values in order to get the
   * best overall correctability:
   *
   * g_pmecc.sector1k   : True if we are using 1024B sectors
   * g_pmecc.nsectors   : The number of sectors per page
   * g_pmecc.correctability : Number of correctable bits per sector
   * g_pmecc.desc.bcherr : The BCH_ERR value for the PMECC CFG register
   */

  ret = pmecc_pagelayout(priv->raw.model.pagesize, eccsize);
  if (ret < 0)
    {
      ferr("ERROR: pmecc_pagelayout failed: %d\n", ret);
      return ret;
    }

  /* Number of Sectors in one Page */

  if (g_pmecc.sector1k)
    {
      /* 1024 bytes per sector */

      g_pmecc.desc.sectorsz = HSMC_PMECCFG_SECTORSZ_1024;
      sectorsperpage        = (priv->raw.model.pagesize >> 10);
      g_pmecc.desc.mm       = 14;

#if defined (CONFIG_SAMA5_PMECC_GALOIS_TABLE1024_ROMADDR) && \
    defined (CONFIG_SAMA5_PMECC_GALOIS_ROMTABLES)
      g_pmecc.desc.alphato  =
        (int16_t *)&(pmecc_gf1024[PMECC_GF_SIZEOF_1024]);
      g_pmecc.desc.indexof  =
        (int16_t *)&(pmecc_gf1024[0]);
#else
      g_pmecc.desc.alphato  =
        (int16_t *)&(pmecc_gf1024[PMECC_GF_ALPHA_TO]);
      g_pmecc.desc.indexof  =
        (int16_t *)&(pmecc_gf1024[PMECC_GF_INDEX_OF]);
#endif
    }
  else
    {
      /* 512 bytes per sector */

      g_pmecc.desc.sectorsz = HSMC_PMECCFG_SECTORSZ_512;
      sectorsperpage        = (priv->raw.model.pagesize >> 9);
      g_pmecc.desc.mm       = 13;

#if defined (CONFIG_SAMA5_PMECC_GALOIS_TABLE512_ROMADDR) && \
    defined (CONFIG_SAMA5_PMECC_GALOIS_ROMTABLES)
      g_pmecc.desc.alphato  =
        (int16_t *)&(pmecc_gf512[PMECC_GF_SIZEOF_512]);
      g_pmecc.desc.indexof  =
        (int16_t *)&(pmecc_gf512[0]);
#else
      g_pmecc.desc.alphato  =
        (int16_t *)&(pmecc_gf512[PMECC_GF_ALPHA_TO]);
      g_pmecc.desc.indexof  =
        (int16_t *)&(pmecc_gf512[PMECC_GF_INDEX_OF]);
#endif
    }

  finfo("sectorsz=%08x sectorsperpage=%d mm=%d\n",
        g_pmecc.desc.sectorsz, sectorsperpage, g_pmecc.desc.mm);

  switch (sectorsperpage)
    {
      case 1:
          g_pmecc.desc.pagesize = HSMC_PMECCFG_PAGESIZE_1SEC;
          break;
      case 2:
          g_pmecc.desc.pagesize = HSMC_PMECCFG_PAGESIZE_2SEC;
          break;
      case 4:
          g_pmecc.desc.pagesize = HSMC_PMECCFG_PAGESIZE_4SEC;
          break;
      case 8:
          g_pmecc.desc.pagesize = HSMC_PMECCFG_PAGESIZE_8SEC;
          break;
      default:
        ferr("ERROR: Unsupported sectors per page: %d\n", sectorsperpage);
        return -EINVAL;
    }

  g_pmecc.desc.nn = (1 << g_pmecc.desc.mm) - 1;

  finfo("pagesize=%08x nn=%d\n", g_pmecc.desc.pagesize, g_pmecc.desc.nn);

  /* Real value of ECC bit number correction (2, 4, 8, 12, 24) */

  g_pmecc.desc.tt = g_pmecc.correctability;
  if (((g_pmecc.desc.mm * g_pmecc.correctability) & 7) == 0)
    {
      g_pmecc.desc.eccsize =
        ((g_pmecc.desc.mm * g_pmecc.correctability) >> 3) * sectorsperpage;
    }
  else
    {
      g_pmecc.desc.eccsize =
        (((g_pmecc.desc.mm * g_pmecc.correctability) >> 3) + 1) *
        sectorsperpage;
    }

  finfo("mm=%d correctability=%d eccsize=%d\n",
        g_pmecc.desc.mm, g_pmecc.correctability, g_pmecc.desc.eccsize);

  g_pmecc.desc.eccstart = eccoffset;
  g_pmecc.desc.eccend   = eccoffset + g_pmecc.desc.eccsize;

  finfo("eccstart=%d eccend=%d sparesize=%d\n",
        g_pmecc.desc.eccstart, g_pmecc.desc.eccend,
        priv->raw.model.sparesize);

  if (g_pmecc.desc.eccend > priv->raw.model.sparesize)
    {
      ferr("ERROR: No room for ECC in spare bytes %d > %d\n",
           g_pmecc.desc.eccend, priv->raw.model.sparesize);

      return -ENOSPC;
    }

  /* Save the size of the spare area.
   *
   * REVISIT: Could we save a bit by setting this to eccend since there is
   * no need to read beyond that?
   */

  g_pmecc.desc.sparesize = priv->raw.model.sparesize;

#if 0
  g_pmecc.desc.nandwr = PMECC_CFG_NANDWR;  /* NAND write access */
#else
  g_pmecc.desc.nandwr = 0;                 /* NAND Read access */
#endif
  if (protected)
    {
      g_pmecc.desc.sparena = HSMC_PMECCFG_SPARE_ENABLE;
    }
  else
    {
      g_pmecc.desc.sparena = 0;
    }

  /* PMECC_CFG_AUTO indicates that the spare is error protected. In this
   * case, the ECC computation takes into account the whole spare area
   * minus the ECC area in the ECC computation operation
   *
   * NOTE:  At 133 MHz, the clkctrl field must be programmed with 2,
   * indicating that the setup time is 3 clock cycles.
   */

  g_pmecc.desc.automode  = 0;
  g_pmecc.desc.clkctrl   = 2;
  g_pmecc.desc.interrupt = 0;

  /* Disable ECC module */

  nand_putreg(SAM_HSMC_PMECCTRL, HSMC_PMECCTRL_DISABLE);

  /* Reset the ECC module */

  nand_putreg(SAM_HSMC_PMECCTRL, HSMC_PMECCTRL_RST);

  regval = g_pmecc.desc.bcherr | g_pmecc.desc.sectorsz |
           g_pmecc.desc.pagesize | g_pmecc.desc.nandwr |
           g_pmecc.desc.sparena | g_pmecc.desc.automode;
  nand_putreg(SAM_HSMC_PMECCFG, regval);

  nand_putreg(SAM_HSMC_PMECCSAREA, g_pmecc.desc.sparesize - 1);
  nand_putreg(SAM_HSMC_PMECCSADDR, g_pmecc.desc.eccstart);
  nand_putreg(SAM_HSMC_PMECCEADDR, g_pmecc.desc.eccend - 1);

  /* Disable all interrupts */

  nand_putreg(SAM_HSMC_PMECCIDR, 0xff);

  /* Enable ECC module */

  nand_putreg(SAM_HSMC_PMECCTRL, HSMC_PMECCTRL_ENABLE);

  /* Now we are configured */

  g_pmecc.configured = true;
#if NAND_NPMECC_BANKS > 1
  g_pmecc.cs = priv->cs;
#endif
  return OK;
}

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
 *  Normally success (OK) is returned, but the error -ECANCELED may be
 *  return in the event that task has been canceled.
 *
 ****************************************************************************/

#if NAND_NPMECC_BANKS > 1
int pmecc_lock(void)
{
  return nxsem_wait_uninterruptible(&g_pmecc.exclsem);
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
  nxsem_post(&g_pmecc.exclsem);
}
#endif

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

int pmecc_correction(uint32_t isr, uintptr_t data)
{
#ifdef CONFIG_SAMA5_PMECC_EMBEDDEDALGO
  /* REVISIT:  Whare are the types pmecc and pmerrloc?
   * REVISIT:  Check returned value
   */

  return pmecc_correctionalgo(??, ??, &g_pmecc, isr, data);
#else
  /* REVISIT:  Check returned value */

  return pmecc_correctionalgo(isr, data);
#endif
}

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

uint32_t pmecc_get_eccsize(void)
{
  return g_pmecc.desc.eccsize;
}

uint32_t pmecc_get_pagesize(void)
{
  return g_pmecc.desc.pagesize;
}

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
void pmecc_buildgf(uint32_t mm, int16_t *indexof, int16_t *alphato)
{
  uint32_t i;
  uint32_t mask;
  uint32_t nn;
  uint32_t p[15];

  nn = (1 << mm) - 1;

  /* Set default value */

  for (i = 1; i < mm; i++)
    {
      p[i] = 0;
    }

  /* 1 + X^mm */

  p[0]  = 1;
  p[mm] = 1;

  /* others */

  if (mm == 3)
    {
      p[1] = 1;
    }
  else if (mm == 4)
    {
      p[1] = 1;
    }
  else if (mm == 5)
    {
      p[2] = 1;
    }
  else if (mm == 6)
    {
      p[1] = 1;
    }
  else if (mm == 7)
    {
      p[3] = 1;
    }
  else if (mm == 8)
    {
      p[2] = p[3] = p[4] = 1;
    }
  else if (mm == 9)
    {
      p[4] = 1;
    }
  else if (mm == 10)
    {
      p[3] = 1;
    }
  else if (mm == 11)
    {
      p[2] = 1;
    }
  else if (mm == 12)
    {
      p[1] = p[4] = p[6] = 1;
    }
  else if (mm == 13)
    {
      p[1] = p[3] = p[4] = 1;
    }
  else if (mm == 14)
    {
      p[1] = p[6] = p[10] = 1;
    }
  else if (mm == 15)
    {
      p[1] = 1;
    }

  /* First
   *
   * build alpha ^ mm it will help to generate the field (primitive)
   */

  alphato[mm] = 0;
  for (i = 0; i < mm; i++)
    {
      if (p[i])
        {
          alphato[mm] |= 1 << i;
        }
    }

  /* Second
   *
   * Build elements from 0 to mm - 1.  Very easy because degree is less than
   * mm so it is just a logical shift ! (only the remainder)
   */

  mask = 1;
  for (i = 0; i < mm; i++)
    {
      alphato[i] = mask;
      indexof[alphato[i]] = i;
      mask <<= 1;
    }

  indexof[alphato[mm]] = mm ;

  /* Use a mask to select the MSB bit of the LFSR ! */

  mask >>= 1; /* Previous value moust be decremented */

  /* Then finish the building */

  for (i = mm + 1; i <= nn; i++)
    {
      /* Check if the msb bit of the lfsr is set */

      if (alphato[i - 1] & mask)
        {
          /* Feedback loop is set */

          alphato[i] = alphato[mm] ^ ((alphato[i - 1] ^ mask) << 1);
        }
      else
        {
          /* Only shift is enabled */

          alphato[i] = alphato[i - 1] << 1;
        }

      /* lookup table */

      indexof[alphato[i]] = i % nn;
    }

  /* Of course index of 0 is undefined in a multiplicative field */

  indexof[0] = -1;
}
#endif /* CONFIG_SAMA5_PMECC_GALOIS_CUSTOM */

#endif /* CONFIG_SAMA5_HAVE_PMECC */
