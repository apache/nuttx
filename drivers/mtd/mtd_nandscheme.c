/****************************************************************************
 * drivers/mtd/mtd_nandscheme.c
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

#include <sys/types.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/mtd/nand_scheme.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Spare area placement scheme for 256 byte pages */

const struct nand_scheme_s g_nand_sparescheme256 =
{
  5,             /* Bad block marker is at position #5 */
  3,             /* 3 ecc bytes */
  4,             /* 4 extra bytes */
  {0, 1, 2},     /* Ecc bytes positions */
  {3, 4, 6, 7},  /* Extra bytes positions */
};

/* Spare area placement scheme for 512 byte pages */

const struct nand_scheme_s g_nand_sparescheme512 =
{
  5,                               /* Bad block marker is at position #5 */
  6,                               /* 6 ecc bytes */
  8,                               /* 8 extra bytes */
  {0, 1, 2, 3, 6, 7},              /* Ecc bytes positions */
  {8, 9, 10, 11, 12, 13, 14, 15},  /* Extra bytes positions */
};

/* Spare area placement scheme for 2048 byte pages */

const struct nand_scheme_s g_nand_sparescheme2048 =
{
  0,                          /* Bad block marker is at position #0 */
  24,                         /* 24 ecc bytes */
  38,                         /* 38 extra bytes */
  {40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57,
  58, 59, 60, 61, 62, 63},   /* Ecc bytes positions */

  { 2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
  20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37,
  38, 39},                   /* Extra bytes positions */
};

/* Spare area placement scheme for 4096 byte pages. */

const struct nand_scheme_s g_nand_sparescheme4096 =
{
  0,                              /* Bad block marker is at position #0 */
  48,                             /* 48 ecc bytes */
  78,                             /* 78 extra bytes */
  {80,  81,  82,  83,  84,  85,  86,  87,  88,  89,  90,  91,  92,  93,
  94,  95,  96,  97,  98,  99, 100, 101, 102, 103, 104, 105, 106, 107,
  108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121,
  122, 123, 124, 125, 126, 127},  /* Ecc bytes positions */

  { 2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
  20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37,
  38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55,
  56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73,
  74, 75, 76, 77, 78, 79},        /* Extra bytes positions */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nandscheme_readbadblockmarker
 *
 * Description:
 *   Reads the bad block marker inside a spare area buffer using the provided
 *   scheme.
 *
 * Input Parameters:
 *   scheme  Pointer to a nand_scheme_s instance.
 *   spare   Spare area buffer.
 *   marker  Pointer to the variable to store the bad block marker.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nandscheme_readbadblockmarker(FAR const struct nand_scheme_s *scheme,
                                   FAR const uint8_t *spare,
                                   FAR uint8_t *marker)
{
  *marker = spare[scheme->bbpos];
}

/****************************************************************************
 * Name: nandscheme_writebadblockmarker
 *
 * Description:
 *   Modifies the bad block marker inside a spare area, using the given
 *   scheme.
 *
 * Input Parameters:
 *   scheme  Pointer to a nand_scheme_s instance.
 *   spare   Spare area buffer.
 *   marker  Bad block marker to write.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nandscheme_writebadblockmarker(FAR const struct nand_scheme_s *scheme,
                                    FAR uint8_t *spare, uint8_t marker)
{
  spare[scheme->bbpos] = marker;
}

/****************************************************************************
 * Name: nandscheme_readecc
 *
 * Description:
 *   Reads ECC information from a spare area using the provided scheme.
 *
 * Input Parameters:
 *   scheme  Pointer to a nand_scheme_s instance.
 *   spare   Spare area buffer.
 *   ecc     ECC buffer.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nandscheme_readecc(FAR const struct nand_scheme_s *scheme,
                        FAR const uint8_t *spare, FAR uint8_t *ecc)
{
  int i;

  for (i = 0; i < scheme->eccsize; i++)
    {
      ecc[i] = spare[scheme->eccbytepos[i]];
    }
}

/****************************************************************************
 * Name: nandscheme_writeecc
 *
 * Description:
 *   Writes ECC information in a spare area, using a particular scheme.
 *
 * Input Parameters:
 *   scheme  Pointer to a nand_scheme_s instance.
 *   spare   Spare area buffer.
 *   ecc     ECC buffer.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nandscheme_writeecc(FAR const struct nand_scheme_s *scheme,
                         FAR uint8_t *spare, FAR const uint8_t *ecc)
{
  int i;

  for (i = 0; i < scheme->eccsize; i++)
    {
      spare[scheme->eccbytepos[i]] = ecc[i];
    }
}

/****************************************************************************
 * Name: nandscheme_readextra
 *
 * Description:
 *   Reads extra bytes of information from a spare area, using the provided
 *   scheme.
 *
 * Input Parameters:
 *   scheme  Pointer to a nand_scheme_s instance.
 *   spare   Spare area buffer.
 *   extra   Extra bytes buffer.
 *   size    Number of extra bytes to read.
 *   offset  Index where to read the first extra byte.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nandscheme_readextra(FAR const struct nand_scheme_s *scheme,
                          FAR const uint8_t *spare, FAR void *extra,
                          unsigned int size, unsigned int offset)
{
  DEBUGASSERT((size + offset) <= scheme->nxbytes);

  int i;

  for (i = 0; i < size; i++)
    {
      ((FAR uint8_t *)extra)[i] = spare[scheme->xbytepos[i + offset]];
    }
}

/****************************************************************************
 * Name: nandscheme_writeextra
 *
 * Description:
 *   Write extra bytes of information inside a spare area, using the provided
 *   scheme.
 *
 * Input Parameters:
 *   scheme  Pointer to a nand_scheme_s instance.
 *   spare   Spare area buffer.
 *   extra   Extra bytes buffer.
 *   size    Number of extra bytes to write.
 *   offset  Index where to write the first extra byte.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nandscheme_writeextra(FAR const struct nand_scheme_s *scheme,
                           FAR uint8_t *spare, FAR const void *extra,
                           unsigned int size, unsigned int offset)
{
  DEBUGASSERT((size + offset) <= scheme->nxbytes);

  uint32_t i;
  for (i = 0; i < size; i++)
    {
      spare[scheme->xbytepos[i + offset]] = ((FAR uint8_t *) extra)[i];
    }
}

/****************************************************************************
 * Name: nandscheme_build4096
 *
 * Description:
 *   Build a scheme instance for 4096 page size nand flash
 *
 * Input Parameters:
 *   scheme  Pointer to a nand_scheme_s instance.
 *   sparesize Size of spare area.
 *   eccoffset Index where to write the first extra byte.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

int nandscheme_build4096(FAR struct nand_scheme_s *scheme,
                         unsigned int sparesize, unsigned int eccoffset)
{
  uint8_t eccsize = g_nand_sparescheme4096.eccsize;
  int i;

  if ((eccoffset + eccsize) > sparesize)
    {
      return -E2BIG;
    }

  scheme->bbpos   = g_nand_sparescheme4096.bbpos;
  scheme->eccsize = eccsize;

  for (i = 0; i < eccsize; i++)
    {
      scheme->eccbytepos[i] = eccoffset + i;
    }

  scheme->nxbytes = sparesize - eccsize - 2;

  for (i = 0; i < scheme->nxbytes; i++)
    {
      scheme->xbytepos[i] = 2 + i;
    }

  return OK;
};
