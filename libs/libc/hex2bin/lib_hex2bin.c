/****************************************************************************
 * libs/libc/hex2bin/lib_hex2bin.c
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

/****************************************************************************
 * References:
 *   - http://en.wikipedia.org/wiki/Intel_HEX
 *   - Hexadecimal Object File Format Specification, Revision A January 6,
 *     1988, Intel
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdint.h>
#include <debug.h>
#include <errno.h>
#include <hex2bin.h>

#include <nuttx/streams.h>

#include "libc.h"

#ifdef CONFIG_LIBC_HEX2BIN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ASCII record sizes */

#define BYTECOUNT_ASCSIZE      2
#define ADDRESS_ASCSIZE        4
#define RECTYPE_ASCSIZE        2

#define BYTECOUNT_LINENDX      (0)
#define ADDRESS_LINENDX        (BYTECOUNT_LINENDX + BYTECOUNT_ASCSIZE)
#define RECTYPE_LINENDX        (ADDRESS_LINENDX + ADDRESS_ASCSIZE)
#define DATA_LINENDX           (RECTYPE_LINENDX + RECTYPE_ASCSIZE)
#define HEADER_ASCSIZE          DATA_LINENDX

#define CHECKSUM_ASCSIZE       2
#define TRAILER_SIZE           (CHECKSUM_ASCSIZE)

#define MAXDATA_BINSIZE        255
#define RECORD_ASCSIZE(n)      (HEADER_ASCSIZE + TRAILER_SIZE + 2*(n))
#define MAXRECORD_ASCSIZE      RECORD_ASCSIZE(MAXDATA_BINSIZE)
#define MINRECORD_ASCSIZE      RECORD_ASCSIZE(0)
#define LINE_ALLOC             MAXRECORD_ASCSIZE

/* Binary record sizes */

#define BYTECOUNT_BINSIZE      1
#define ADDRESS_BINSIZE        2
#define RECTYPE_BINSIZE        1

#define BYTECOUNT_BINNDX       (0)
#define ADDRESS_BINNDX         (BYTECOUNT_BINNDX + BYTECOUNT_BINSIZE)
#define RECTYPE_BINNDX         (ADDRESS_BINNDX + ADDRESS_BINSIZE)
#define DATA_BINNDX            (RECTYPE_BINNDX + RECTYPE_BINSIZE)
#define HEADER_BINSIZE         DATA_BINNDX

#define CHECKSUM_BINSIZE       1
#define TRAILER_BINSIZE        CHECKSUM_BINSIZE

#define RECORD_BINSIZE(n)      (HEADER_BINSIZE + TRAILER_BINSIZE + (n))
#define MAXRECORD_BINSIZE      RECORD_BINSIZE(MAXDATA_BINSIZE)
#define MINRECORD_BKINSIZE     RECORD_BINSIZE(0)
#define BIN_ALLOC              MAXRECORD_BINSIZE

/* Record start code */

#define RECORD_STARTCODE       ':'

/* Record Types */

#define RECORD_DATA            0  /* Data */
#define RECORD_EOF             1  /* End of file */
#define RECORD_EXT_SEGADDR     2  /* Extended segment address record */
#define RECORD_START_SEGADDR   3  /* Start segment address record */
#define RECORD_EXT_LINADDR     4  /* Extended linear address record */
#define RECORD_START_LINADDR   5  /* Start linear address record */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nibble2bin
 ****************************************************************************/

static int nibble2bin(uint8_t ascii)
{
  if (ascii >= '0' && ascii <= '9')
    {
      return (ascii - 0x30);
    }
  else if (ascii >= 'a' && ascii <= 'f')
    {
      return (ascii - 'a' + 10);
    }
  else if (ascii >= 'A' && ascii <= 'F')
    {
      return (ascii - 'A' + 10);
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: byte2bin
 ****************************************************************************/

static int byte2bin(FAR const uint8_t *ascii)
{
  int nibble;
  int byte;

  /* Get the MS nibble (big endian order) */

  nibble = nibble2bin(*ascii++);
  if (nibble < 0)
    {
      return nibble;
    }

  byte = (nibble << 4);

  /* Get the MS nibble */

  nibble = nibble2bin(*ascii);
  if (nibble < 0)
    {
      return nibble;
    }

  byte |= nibble;
  return byte;
}

/****************************************************************************
 * Name: word2bin
 ****************************************************************************/

#if 0 /* Not used */
static int word2bin(FAR const char *ascii)
{
  int byte;
  int word;

  /* Get the MS byte (big endian order) */

  byte = word2bin(ascii);
  if (byte < 0)
    {
      return byte;
    }

  word = (byte << 8);

  /* Get the MS byte */

  byte = word2bin(ascii + 2);
  if (byte < 0)
    {
      return byte;
    }

  word |= byte;
  return word;
}
#endif

/****************************************************************************
 * Name: data2bin
 ****************************************************************************/

int data2bin(FAR uint8_t *dest, FAR const uint8_t *src, int nsrcbytes)
{
  int byte;

  /* An even number of source bytes is expected */

  if ((nsrcbytes & 1) != 0)
    {
      return -EINVAL;
    }

  /* Convert src bytes in groups of 2, writing one byte to the output on each
   * pass through the loop.
   */

  while (nsrcbytes > 0)
    {
      /* Get the MS nibble (big endian order) */

      byte = byte2bin(src);
      if (byte < 0)
        {
          return byte;
        }

      src += 2;

      /* And write the byte to the destination */

      *dest++ = byte;
      nsrcbytes -= 2;
    }

  return OK;
}

/****************************************************************************
 * Name: readstream
 ****************************************************************************/

static int readstream(FAR struct lib_instream_s *instream,
                      FAR uint8_t *line, unsigned int lineno)
{
  int nbytes = 0;
  int ch;

  /* Skip until the beginning of line start code is encountered */

  ch = lib_stream_getc(instream);
  while (ch != RECORD_STARTCODE && ch != EOF)
    {
      ch = lib_stream_getc(instream);
    }

  /* Skip over the startcode */

  if (ch != EOF)
    {
      ch = lib_stream_getc(instream);
    }

  /* Then read, verify, and buffer until the end of line is encountered */

  while (ch != EOF && nbytes < (MAXRECORD_ASCSIZE - 1))
    {
      if (ch == '\n' || ch == '\r')
        {
          *line = '\0';
          return nbytes;
        }

      /* Only hex data goes into the line buffer */

      else if (isxdigit(ch))
        {
          *line++ = ch;
          nbytes++;
        }
      else if (!isspace(ch)) /* Not expected */
        {
          lerr("Line %u ERROR: Unexpected character %c[%02x] in stream\n",
               lineno, isprint(ch) ? ch : '.', ch);
          break;
        }

      /* Read the next character from the input stream */

      ch = lib_stream_getc(instream);
    }

  /* Some error occurred: Unexpected EOF, line too long, or bad character in
   * stream
   */

  lerr("Line %u ERROR: Failed to read line. %d characters read\n",
       lineno, nbytes);
  return EOF;
}

/****************************************************************************
 * Name: hex2bin_swap16 and hex2bin_swap32
 ****************************************************************************/

static inline void hex2bin_swap16(FAR uint8_t *data, int bytecount)
{
  for (; bytecount > 0; bytecount -= 2)
    {
      uint8_t b0 = data[0];
      uint8_t b1 = data[1];

      *data++ = b1;
      *data++ = b0;
    }
}

static inline void hex2bin_swap32(FAR uint8_t *data, int bytecount)
{
  for (; bytecount > 0; bytecount -= 4)
    {
      uint8_t b0 = data[0];
      uint8_t b1 = data[1];
      uint8_t b2 = data[2];
      uint8_t b3 = data[3];

      *data++ = b3;
      *data++ = b2;
      *data++ = b1;
      *data++ = b0;
    }
}

/****************************************************************************
 * Name: writedata
 ****************************************************************************/

static inline void writedata(FAR struct lib_sostream_s *outstream,
                             FAR uint8_t *data, int bytecount)
{
  for (; bytecount > 0; bytecount--)
    {
      lib_stream_putc(outstream, *data++);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hex2bin
 *
 * Description:
 *   Read the Intel HEX ASCII data provided on the serial IN stream and write
 *   the binary to the seek-able serial OUT stream.
 *
 *   These streams may be files or, in another usage example, the IN stream
 *   could be a serial port and the OUT stream could be a memory stream.
 *   This would decode and write the serial input to memory.
 *
 * Input Parameters:
 *   instream  - The incoming stream from which Intel HEX data will be
 *               received.
 *   outstream - The outgoing stream in which binary data will be written.
 *   baseaddr  - The base address of the outgoing stream.  Seeking in the
 *               output stream will be relative to this address.
 *   endpaddr  - The end address (plus 1) of the outgoing stream.  This
 *               value is used only for range checking.  endpaddr must
 *               be larger than baseaddr.  A zero value for endpaddr
 *               disables range checking.
 *   swap      - Controls byte ordering.  See enum hex2bin_swap_e for
 *               description of the values.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int hex2bin(FAR struct lib_instream_s *instream,
            FAR struct lib_sostream_s *outstream, uint32_t baseaddr,
            uint32_t endpaddr, enum hex2bin_swap_e swap)
{
  FAR uint8_t *alloc;
  FAR uint8_t *line;
  FAR uint8_t *bin;
  int nbytes;
  int bytecount;
  uint32_t address;
  uint32_t endaddr;
  uint32_t expected;
  uint32_t extension;
  uint16_t address16;
  uint8_t checksum;
  unsigned int lineno;
  int i;
  int ret = OK;

  /* Allocate buffer memory */

  alloc = (FAR uint8_t *)lib_malloc(LINE_ALLOC + BIN_ALLOC);
  if (alloc == NULL)
    {
      lerr("ERROR: Failed to allocate memory\n");
      return -ENOMEM;
    }

  line = alloc;
  bin  = &alloc[LINE_ALLOC];

  extension = 0;
  expected = 0;
  lineno = 0;

  /* Read and process the HEX input stream stream until the end of file
   * record is received (or until an error occurs)
   */

  while ((nbytes = readstream(instream, line, lineno)) != EOF)
    {
      /* Increment the line number */

      lineno++;

      /* Did we read enough data to do anything? */

      if (nbytes < MINRECORD_ASCSIZE)
        {
          lerr("Line %u ERROR: Record too short: %d\n", lineno, nbytes);
          goto errout_with_einval;
        }

      /* We should always read an even number of bytes */

      if ((nbytes & 1) != 0)
        {
          lerr("Line %u ERROR: Record length is odd: %d\n", lineno, nbytes);
          goto errout_with_einval;
        }

      /* Get the data byte count */

      bytecount = byte2bin(&line[BYTECOUNT_LINENDX]);
      if (bytecount < 0)
        {
          lerr("Line %u ERROR: Failed to read bytecount: %d\n",
               lineno, bytecount);
          ret = bytecount;
          goto errout_with_buffers;
        }

      /* Verify that the bytecount matches the length of the record */

      if (RECORD_ASCSIZE(bytecount) != nbytes)
        {
          lerr("Line %u ERROR: Expected %d bytes, read %d\n",
               lineno, RECORD_ASCSIZE(bytecount), nbytes);
          goto errout_with_einval;
        }

      /* Convert the entire line to binary.  We need to do this for
       * checksum calculation which includes the entire line (minus
       * the start code and the checksum at the end of the line itself)
       */

      ret = data2bin(bin, line, nbytes);
      if (ret < 0)
        {
          lerr("Line %u ERROR: Failed to convert line to binary: %d\n",
               lineno, ret);
          goto errout_with_buffers;
        }

      /* Calculate and verify the checksum over all of the data */

      nbytes >>= 1;  /* Number of bytes in bin[] */
      checksum = 0;

      for (i = 0; i < nbytes; i++)
        {
          checksum += bin[i];
        }

      if (checksum != 0)
        {
          lerr("Line %u ERROR: Bad checksum %02x\n", lineno, checksum);
          goto errout_with_einval;
        }

      /* Get the 16-bit (unextended) address from the record */

      address16 = (uint16_t)bin[ADDRESS_BINNDX] << 8 |
                  (uint16_t)bin[ADDRESS_BINNDX + 1];

      /* Handle the record by its record type */

      switch (bin[RECTYPE_BINNDX])
        {
        case RECORD_DATA: /* Data */
          {
            /* Swap data in place in the binary buffer as required */

            switch (swap)
              {
              case HEX2BIN_NOSWAP: /* No swap, stream is the correct byte order */
                break;

              case HEX2BIN_SWAP16: /* Swap bytes in 16-bit values */
                {
                  if ((bytecount & 1) != 0)
                    {
                      lerr("Line %d ERROR: Byte count %d is not a multiple "
                           "of 2\n",
                           lineno, bytecount);
                       goto errout_with_einval;
                    }

                  /* Do the byte swap */

                  hex2bin_swap16(&bin[DATA_BINNDX], bytecount);
                }
                break;

              case HEX2BIN_SWAP32: /* Swap bytes in 32-bit values */
                {
                  if ((bytecount & 3) != 0)
                    {
                      lerr("Line %d ERROR: Byte count %d is not a multiple "
                           "of 4\n",
                           lineno, bytecount);
                       goto errout_with_einval;
                    }

                  /* Do the byte swap */

                  hex2bin_swap32(&bin[DATA_BINNDX], bytecount);
                }
                break;

              default:
                {
                  lerr("ERROR: Invalid swap argument: %d\n", swap);
                  goto errout_with_einval;
                }
              }

            /* Get and verify the full 32-bit address */

            address = extension + (uint32_t)address16;
            endaddr = address + bytecount;

            if (address < baseaddr || (endpaddr != 0 && endaddr >= endpaddr))
              {
                lerr("Line %d ERROR: Extended address %08lx is out of "
                     "range\n",
                     lineno, (unsigned long)address);
                goto errout_with_einval;
              }

            /* Seek to the correct position in the OUT stream if we have
             * made an unexpected jump in the data address.
             */

            if (address != expected)
              {
                off_t pos = lib_stream_seek(outstream,
                                            address - baseaddr, SEEK_SET);
                if (pos == (off_t)-1)
                  {
                    lerr("Line %u ERROR: Seek to address %08lu failed\n",
                         lineno, (unsigned long)address);
                    ret = -ESPIPE;
                    goto errout_with_buffers;
                  }
              }

            /* Transfer data to the OUT stream */

            writedata(outstream, &bin[DATA_BINNDX], bytecount);

            /* This is the next data address that we expect to see */

            expected = address + bytecount;
          }
          break;

        case RECORD_EOF: /* End of file */

          /*  End Of File record.  Must occur exactly once per file in the
           * last line of the file. The byte count is 00 and the data field
           * is empty. Usually the address field is also 0000.
           */

          if (bytecount == 0)
            {
              ret = OK;
              goto exit_with_buffers;
            }

          lerr("Line %u ERROR: Nonzero bytecount %d in EOF\n",
               lineno, bytecount);
          goto errout_with_einval;

        case RECORD_EXT_SEGADDR: /* Extended segment address record */

          /* The address specified by the data field is multiplied by 16
           * (shifted 4 bits left) and added to the subsequent data record
           * addresses. This allows addressing of up to a megabyte of
           * address space. The address field of this record has to be
           * 0000, the byte count is 02 (the segment is 16-bit). The
           * least significant hex digit of the segment address is always
           * 0.
           */

          if (bytecount != 2 || address16 != 0 || bin[DATA_BINNDX + 1] != 0)
            {
              lerr("Line %u ERROR: Invalid segment address\n", lineno);
              lerr("  bytecount=%d address=%04x segment=%02x%02x\n",
                   bytecount, address16, bin[DATA_BINNDX],
                   bin[DATA_BINNDX + 1]);
              goto errout_with_einval;
            }

          extension = (uint32_t)bin[DATA_BINNDX] << 12;
          break;

        case RECORD_START_SEGADDR: /* Start segment address record */

          /* For 80x86 processors, it specifies the initial content of
           * the CS:IP registers. The address field is 0000, the byte
           * count is 04, the first two bytes are the CS value, the
           * latter two are the IP value.
           */

          break;

        case RECORD_EXT_LINADDR: /* Extended linear address record */

          /* The address field is 0000, the byte count is 02. The two
           * data bytes (two hex digit pairs in big endian order)
           * represent the upper 16 bits of the 32 bit address for
           * all subsequent 00 type records until the next 04 type
           * record comes. If there is not a 04 type record, the
           * upper 16 bits default to 0000. To get the absolute
           * address for subsequent 00 type records, the address
           * specified by the data field of the most recent 04 record
           * is added to the 00 record addresses.
           */

          if (bytecount != 2 || address16 != 0)
            {
              lerr("Line %u ERROR: Invalid linear address\n", lineno);
              lerr("  bytecount=%d address=%04x\n", bytecount, address16);
              goto errout_with_einval;
            }

          extension = (uint32_t)bin[DATA_BINNDX] << 24 |
                      (uint32_t)bin[DATA_BINNDX + 1] << 16;
          break;

        case RECORD_START_LINADDR: /* Start linear address record */

          /* The address field is 0000, the byte count is 04. The 4
           * data bytes represent the 32-bit value loaded into the EIP
           * register of the 80386 and higher CPU.
           */

          break;

        default:
          break;
        }
    }

  lerr("ERROR: No EOF record found\n");

errout_with_einval:
  ret = -EINVAL;

errout_with_buffers:
exit_with_buffers:
  lib_free(alloc);
  return ret;
}

#endif /* CONFIG_LIBC_HEX2BIN */
