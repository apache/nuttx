/****************************************************************************
 * include/nuttx/mtd/nand_scheme.h
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

#ifndef __INCLUDE_NUTTX_MTD_NAND_SCHEME_H
#define __INCLUDE_NUTTX_MTD_NAND_SCHEME_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/mtd/nand_config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct nand_scheme_s
{
  uint8_t bbpos;        /* Bad block position marker */
  uint8_t eccsize;      /* Number of bytes of ECC correction */
  uint8_t nxbytes;      /* Number of extra bytes */

  /* ECC byte position offsets */

  uint8_t eccbytepos[CONFIG_MTD_NAND_MAXSPAREECCBYTES];

  /* Extra byte position offsets */

  uint8_t xbytepos[CONFIG_MTD_NAND_MAXSPAREEXTRABYTES];
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

EXTERN const struct nand_scheme_s g_nand_sparescheme256;
EXTERN const struct nand_scheme_s g_nand_sparescheme512;
EXTERN const struct nand_scheme_s g_nand_sparescheme2048;
EXTERN const struct nand_scheme_s g_nand_sparescheme4096;

/****************************************************************************
 * Public Function Prototypes
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
                                   FAR uint8_t *marker);

/****************************************************************************
 * Name: nandscheme_readbadblockmarker
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
                                    FAR uint8_t *spare, uint8_t marker);

/****************************************************************************
 * Name: nandscheme_eccoffset
 *
 * Description:
 *   Return the offset to the first byte of ECC information.  This define
 *   makes the assumption that the first byte of the eccbytepos[] array
 *   is an offset to beginning of the ECC area.  This might not necessarily
 *   be true!
 *
 * Input Parameters:
 *   scheme  Pointer to a nand_scheme_s instance.
 *
 * Returned Value:
 *   Offset in the spare area to the first ECC byte
 *
 ****************************************************************************/

#define nandscheme_eccoffset(s) ((s)->eccbytepos[0])

/****************************************************************************
 * Name: nandscheme_eccsize
 *
 * Description:
 *   Return the size of the ECC information in the spare area
 *
 * Input Parameters:
 *   scheme  Pointer to a nand_scheme_s instance.
 *
 * Returned Value:
 *   Size of the ECC information in the spare area.
 *
 ****************************************************************************/

#define nandscheme_eccsize(s) ((s)->eccsize)

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
                        FAR const uint8_t *spare, FAR uint8_t *ecc);

/****************************************************************************
 * Name: nandschem_writeecc
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
                         FAR uint8_t *spare, FAR const uint8_t *ecc);

/****************************************************************************
 * Name: nandscheme_xoffset
 *
 * Description:
 *   Return the offset to the first byte of extra information.  This define
 *   makes the assumption that the first byte of the xbytepos[] array
 *   is an offset to the beginning of the extra information area.  This
 *   might not necessarily be true!
 *
 * Input Parameters:
 *   scheme  Pointer to a nand_scheme_s instance.
 *
 * Returned Value:
 *   Offset in the spare area to the first extra byte
 *
 ****************************************************************************/

#define nandscheme_xoffset(s) ((s)->xbytepos[0])

/****************************************************************************
 * Name: nandscheme_xsize
 *
 * Description:
 *   Return the size of the extra information in the spare area
 *
 * Input Parameters:
 *   scheme  Pointer to a nand_scheme_s instance.
 *
 * Returned Value:
 *   Size of the extra information in the spare area.
 *
 ****************************************************************************/

#define nandscheme_xsize(s) ((s)->nxbytes)

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
                          unsigned int size, unsigned int offset);

/****************************************************************************
 * Name: nandscheme_readextra
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
                           unsigned int size, unsigned int offset);

/****************************************************************************
 * Name: nandscheme_readextra
 *
 * Description:
 *   Build a scheme instance for 4096 page size nand flash
 *
 * Input Parameters:
 *   scheme  Pointer to a nand_scheme_s instance.
 *   sparesize Size of spare area.
 *   offset  Index where to write the first extra byte.
 *   size    Number of extra bytes to write.
 *   offset  Index where to write the first extra byte.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

int nandscheme_build4086(FAR struct nand_scheme_s *scheme,
                         unsigned int sparesize, unsigned int eccoffset);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_MTD_NAND_SCHEME_H */
