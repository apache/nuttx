/****************************************************************************
 * drivers/mtd/hamming.c
 *
 *   Copyright (c) 2011, Atmel Corporation
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

#include <assert.h>
#include <debug.h>

#include <nuttx/mtd/hamming.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hamming_bitsinbyte
 *
 * Description:
 *   Counts the number of bits set to '1' in the given byte.
 *
 * Input Parameters:
 *   bytes - The byte to use.
 *
 * Returned Value:
 *   Returns the number of bits set to '1' in the given byte.
 *
 ****************************************************************************/

static unsigned int hamming_bitsinbyte(uint8_t byte)
{
  unsigned int count = 0;

  while (byte != 0)
    {
      if ((byte & 1) != 0)
        {
          count++;
        }

      byte >>= 1;
    }

  return count;
}

/****************************************************************************
 * Name: hamming_bitsincode256
 *
 * Description:
 *   Counts the number of bits set to '1' in the given hamming code.
 *
 * Input Parameters:
 *   code - Hamming code
 *
 * Returned Value:
 *   Returns the number of bits set to '1' in the given hamming code.
 *
 ****************************************************************************/

static uint8_t hamming_bitsincode256(FAR uint8_t *code)
{
  return hamming_bitsinbyte(code[0]) +
         hamming_bitsinbyte(code[1]) +
         hamming_bitsinbyte(code[2]);
}

/****************************************************************************
 * Name: hamming_compute256
 *
 * Description:
 *   Calculates the 22-bit hamming code for a 256-bytes block of data.
 *
 * Input Parameters:
 *   data - Data buffer to calculate code
 *   code - Pointer to a buffer where the code should be stored
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void hamming_compute256(FAR const uint8_t *data, FAR uint8_t *code)
{
  uint8_t colsum = 0;
  uint8_t evenline = 0;
  uint8_t oddline = 0;
  uint8_t evencol = 0;
  uint8_t oddcol = 0;
  int i;

  /* Xor all bytes together to get the column sum;
   * At the same time, calculate the even and odd line codes
   */

  for (i = 0; i < 256; i++)
    {
      colsum ^= data[i];

      /* If the xor sum of the byte is 0, then this byte has no incidence on
       * the computed code; so check if the sum is 1.
       */

      if ((hamming_bitsinbyte(data[i]) & 1) == 1)
        {
          /* Parity groups are formed by forcing a particular index bit to 0
           * (even) or 1 (odd).
           * Example on one byte:
           *
           * bits (dec)  7   6   5   4   3   2   1   0
           *      (bin) 111 110 101 100 011 010 001 000
           *                            '---'---'---'----------.
           *                                                   |
           * groups P4' ooooooooooooooo eeeeeeeeeeeeeee P4     |
           *        P2' ooooooo eeeeeee ooooooo eeeeeee P2     |
           *        P1' ooo eee ooo eee ooo eee ooo eee P1     |
           *                                                   |
           * We can see that:                                  |
           *  - P4  -> bit 2 of index is 0 --------------------'
           *  - P4' -> bit 2 of index is 1.
           *  - P2  -> bit 1 of index if 0.
           *  - etc...
           * We deduce that a bit position has an impact on all even Px if
           * the log2(x)nth bit of its index is 0
           *     ex: log2(4) = 2, bit2 of the index must be 0 (-> 0 1 2 3)
           * and on all odd Px' if the log2(x)nth bit of its index is 1
           *     ex: log2(2) = 1, bit1 of the index must be 1 (-> 0 1 4 5)
           *
           * As such, we calculate all the possible Px and Px' values at the
           * same time in two variables, evenline and oddline, such as
           *     evenline bits: P128  P64  P32  P16  P8  P4  P2  P1
           *     oddline  bits: P128' P64' P32' P16' P8' P4' P2' P1'
           */

          evenline ^= (255 - i);
          oddline ^= i;
        }
    }

  /* At this point, we have the line parities, and the column sum. First, We
   * must calculate the parity group values on the column sum.
   */

  for (i = 0; i < 8; i++)
    {
      if (colsum & 1)
        {
          evencol ^= (7 - i);
          oddcol ^= i;
        }

      colsum >>= 1;
    }

  /* Now, we must interleave the parity values,
   * to obtain the following layout:
   * Code[0] = Line1
   * Code[1] = Line2
   * Code[2] = Column
   * Line = Px' Px P(x-1)- P(x-1) ...
   * Column = P4' P4 P2' P2 P1' P1 PadBit PadBit
   */

  code[0] = 0;
  code[1] = 0;
  code[2] = 0;

  for (i = 0; i < 4; i++)
    {
      code[0] <<= 2;
      code[1] <<= 2;
      code[2] <<= 2;

      /* Line 1 */

      if ((oddline & 0x80) != 0)
        {
          code[0] |= 2;
        }

      if ((evenline & 0x80) != 0)
        {
          code[0] |= 1;
        }

      /* Line 2 */

      if ((oddline & 0x08) != 0)
        {
          code[1] |= 2;
        }

      if ((evenline & 0x08) != 0)
        {
          code[1] |= 1;
        }

      /* Column */

      if ((oddcol & 0x04) != 0)
        {
          code[2] |= 2;
        }

      if ((evencol & 0x04) != 0)
        {
          code[2] |= 1;
        }

      oddline <<= 1;
      evenline <<= 1;
      oddcol <<= 1;
      evencol <<= 1;
    }

  /* Invert codes (linux compatibility) */

  code[0] = (~(uint32_t)code[0]);
  code[1] = (~(uint32_t)code[1]);
  code[2] = (~(uint32_t)code[2]);
}

/****************************************************************************
 * Name: hamming_verify256
 *
 * Description:
 *   Verifies and corrects a 256-bytes block of data using the given 22-bits
 *   hamming code.
 *
 * Input Parameters:
 *   data     - Data buffer to check
 *   original - Hamming code to use for verifying the data
 *
 * Returned Value:
 *   Zero on success, otherwise returns a HAMMING_ERROR_ code.
 *
 ****************************************************************************/

static int hamming_verify256(FAR uint8_t *data, FAR const uint8_t *original)
{
  /* Calculate new code */

  uint8_t computed[3];
  uint8_t correction[3];

  hamming_compute256(data, computed);

  /* Xor both codes together */

  correction[0] = computed[0] ^ original[0];
  correction[1] = computed[1] ^ original[1];
  correction[2] = computed[2] ^ original[2];

  /* If all bytes are 0, there is no error */

  if ((correction[0] == 0) && (correction[1] == 0) && (correction[2] == 0))
    {
      return 0;
    }

  /* There are bit errors */

  finfo("Read:       %02x %02x %02x\n",
        original[0], original[1], original[2]);
  finfo("Computed:   %02x %02x %02x\n",
        computed[0], computed[1], computed[2]);
  finfo("Correction: %02x %02x %02x\n",
        correction[0], correction[1], correction[2]);

  /* If there is a single bit error, there are 11 bits set to 1 */

  if (hamming_bitsincode256(correction) == 11)
    {
      uint8_t byte;
      uint8_t bit;

      /* Get byte and bit indexes */

      byte  =  correction[0]       & 0x80;
      byte |= (correction[0] << 1) & 0x40;
      byte |= (correction[0] << 2) & 0x20;
      byte |= (correction[0] << 3) & 0x10;

      byte |= (correction[1] >> 4) & 0x08;
      byte |= (correction[1] >> 3) & 0x04;
      byte |= (correction[1] >> 2) & 0x02;
      byte |= (correction[1] >> 1) & 0x01;

      bit   = (correction[2] >> 5) & 0x04;
      bit  |= (correction[2] >> 4) & 0x02;
      bit  |= (correction[2] >> 3) & 0x01;

      /* Correct bit */

      finfo("Correcting byte %d at bit %d\n", byte, bit);
      data[byte] ^= (1 << bit);

      return HAMMING_ERROR_SINGLEBIT;
    }

  /* Check if ECC has been corrupted */

  if (hamming_bitsincode256(correction) == 1)
    {
      ferr("ERROR: ECC has been correupted\n");
      return HAMMING_ERROR_ECC;
    }

  /* Otherwise, there are multiple bit errors */

  else
    {
      ferr("ERROR: Multiple bit errors\n");
      return HAMMING_ERROR_MULTIPLEBITS;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hamming_compute256x
 *
 * Description:
 *   Computes 3-bytes hamming codes for a data block whose size is multiple
 *   of 256 bytes. Each 256 bytes block gets its own code.
 *
 * Input Parameters:
 *   data - Data to compute code for
 *   size - Data size in bytes
 *   code - Codes buffer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void hamming_compute256x(FAR const uint8_t *data, size_t size,
                         FAR uint8_t *code)
{
  ssize_t remaining = (ssize_t)size;
  DEBUGASSERT((size & 0xff) == 0);

  /* Loop, computing the Hamming code on each 256 byte chunk of data */

  while (remaining > 0)
    {
      hamming_compute256(data, code);

      /* Setup for the next 256 byte chunk */

      data      += 256;
      code      += 3;
      remaining -= 256;
    }
}

/****************************************************************************
 * Name: hamming_verify256x
 *
 * Description:
 *   Verifies 3-bytes hamming codes for a data block whose size is multiple
 *   of 256 bytes. Each 256-bytes block is verified with its own code.
 *
 * Input Parameters:
 *   data - Data buffer to verify
 *   size - Size of the data in bytes
 *   code - Original codes
 *
 * Returned Value:
 *   Return 0 if the data is correct, HAMMING_ERROR_SINGLEBIT if one or more
 *   block(s) have had a single bit corrected, or either HAMMING_ERROR_ECC
 *   or HAMMING_ERROR_MULTIPLEBITS.
 *
 ****************************************************************************/

int hamming_verify256x(FAR uint8_t *data,
                       size_t size,
                       FAR const uint8_t *code)
{
  ssize_t remaining = (ssize_t)size;
  int result = HAMMING_SUCCESS;
  int ret;

  DEBUGASSERT((size & 0xff) == 0);

  /* Loop, verifying each 256 byte chunk of data */

  while (remaining > 0)
    {
      result = hamming_verify256(data, code);
      if (result != HAMMING_SUCCESS)
        {
          /* Check for the case of a single bit error that was corrected */

          if (result == HAMMING_ERROR_SINGLEBIT)
            {
              /* Report the error, but continue verifying */

              ret = HAMMING_ERROR_SINGLEBIT;
            }
          else
            {
              /* A bad error occurred, abort the verification and return the
               * error code
               */

              return result;
            }
        }

      /* Setup for the next 256 byte chunk */

      data      += 256;
      code      += 3;
      remaining -= 256;
    }

  return ret;
}
