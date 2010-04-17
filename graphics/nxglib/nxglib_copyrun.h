/****************************************************************************
 * graphics/nxglib/nxsglib_copyrun.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __GRAPHICS_NXGLIB_NXGLIB_COPYRUN_H
#define __GRAPHICS_NXGLIB_NXGLIB_COPYRUN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <string.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxgl_copyrun_*bpp
 *
 * Description:
 *   Copy a row from an image into run.
 *
 ****************************************************************************/

#if NXGLIB_BITSPERPIXEL == 1
static inline void
nxgl_copyrun_1bpp(FAR const uint8_t *src, FAR uint8_t *dest,
                  unsigned int inbit, size_t npixels)
{
  uint8_t indata;
  uint8_t outdata;
  unsigned int inpixels = 0;
  unsigned int outpixels = 0;
  unsigned int outbit;

  /* Set up the input */

  indata = *src++;

  /* Set up the output */

  outdata = 0;
  outbit  = 0;

  /* Loop until all pixels have been packed into the destination */

  while (outpixels < npixels && inpixels < npixels)
    {
      /* Pack pixels from the source into the destination */
      /* Check the input bit */

      if ((*src & (1 << inbit)) != 0)
        {
          /* If it is set, then set the corresponding bit
           * in the output (probably not the same bit.
           */

          outdata |= (1 << outbit);
        }
      inpixels++;

      /* Check if we have used all of the bits in the input */

      if (++inbit >= 8)
        {
          /* Yes.. Get the next byte from the source and reset
           * the source bit number.
           */

          indata = *src++;
          inbit  = 0;
        }

      /* Now check if we have filled the output byte */

      if (++outbit >= 8)
        {
          /* Yes.. Write the output and reset the output bit
           * number
           */

          *dest++    = outdata;
          outdata    = 0;
          outbit     = 0;
          outpixels += 8;
        }
    }

  /* Handle any bits still in outdata */

  if (outpixels < inpixels)
    {
      *dest = outdata;
    }
}

#elif NXGLIB_BITSPERPIXEL == 2
static inline void
nxgl_copyrun_2bpp(FAR const uint8_t *src, FAR uint8_t *dest,
                  unsigned int inbit, size_t npixels)
{
}

#elif NXGLIB_BITSPERPIXEL == 4
static inline void
nxgl_copyrun_4bpp(FAR const uint8_t *src, FAR uint8_t *dest,
                  unsigned int inbit, size_t npixels)
{
}
#endif
#endif /* __GRAPHICS_NXGLIB_NXGLIB_COPYRUN_H */


