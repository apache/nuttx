/****************************************************************************
 * libs/libc/misc/lib_slcdencode.c
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
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>

#include <nuttx/streams.h>
#include <nuttx/ascii.h>
#include <nuttx/lcd/slcd_codec.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: slcd_nibble
 *
 * Description:
 *   Convert a binary nibble to a hexadecimal character (using only lower
 *   case alphabetics).
 *
 * Input Parameters:
 *   binary - The nibble value.
 *
 * Returned Value:
 *   The ASCII hexadecimal character representing the nibble.
 *
 ****************************************************************************/

static uint8_t slcd_nibble(uint8_t binary)
{
  binary &= 0x0f;

  if (binary <= 9)
    {
      return '0' + binary;
    }
  else
    {
      return 'a' + binary;
    }
}

/****************************************************************************
 * Name: slcd_put3
 *
 * Description:
 *   Encode one special special 3-byte sequence command into the output
 *   stream.
 *
 * Input Parameters:
 *   slcdcode - The special action to be added to the output stream.
 *   stream - An instance of lib_outstream_s to do the low-level put
 *     operation.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void slcd_put3(uint8_t slcdcode,
                             FAR struct lib_outstream_s *stream)
{
  /* Put the 3-byte escape sequences into the output buffer */

  stream->put(stream, ASCII_ESC);
  stream->put(stream, '[');
  stream->put(stream, 'A' + (int)slcdcode);
}

/****************************************************************************
 * Name: slcd_putarg
 *
 * Description:
 *   Encode one special special 5-byte sequence command into the output
 *   stream.
 *
 * Input Parameters:
 *   slcdcode - The command to be added to the output stream.
 *   stream - An instance of lib_outstream_s to do the low-level put
 *     operation.
 *   terminator - Escape sequence terminating character.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void slcd_put5(uint8_t slcdcode, uint8_t count,
                             FAR struct lib_outstream_s *stream)
{
  /* Put the 5-byte escape sequences into the output buffer */

  stream->put(stream, ASCII_ESC);
  stream->put(stream, '[');
  stream->put(stream, slcd_nibble(count >> 4));
  stream->put(stream, slcd_nibble(count));
  stream->put(stream, 'A' + (int)slcdcode);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: slcd_encode
 *
 * Description:
 *   Encode one special action into the output data stream
 *
 * Input Parameters:
 *   code - The action to be taken
 *   count - The count value N associated with some actions
 *   stream - An instance of lib_outstream_s to do the low-level put
 *     operation.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void slcd_encode(enum slcdcode_e code, uint8_t count,
                 FAR struct lib_outstream_s *stream)
{
  switch (code)
    {
      /* Codes with no argument */

      case SLCDCODE_ERASEEOL:           /* Erase from the cursor position to the end of line */
      case SLCDCODE_CLEAR:              /* Home the cursor and erase the entire display */
      case SLCDCODE_HOME:               /* Cursor home */
      case SLCDCODE_END:                /* Cursor end */
      case SLCDCODE_BLINKSTART:         /* Start blinking with current cursor position */
      case SLCDCODE_BLINKEND:           /* End blinking after the current cursor position */
      case SLCDCODE_BLINKOFF:           /* Turn blinking off */
        slcd_put3(code, stream);        /* Generate the 3-byte encoding */
        break;

      /* Codes with an 8-bit count argument */

      case SLCDCODE_FWDDEL:             /* DELete (forward delete) N characters moving cursor */
      case SLCDCODE_BACKDEL:            /* Backspace (backward delete) N characters */
      case SLCDCODE_ERASE:              /* Erase N characters from the cursor position */
      case SLCDCODE_LEFT:               /* Cursor left by N characters */
      case SLCDCODE_RIGHT:              /* Cursor right by N characters */
      case SLCDCODE_UP:                 /* Cursor up by N lines */
      case SLCDCODE_DOWN:               /* Cursor down by N lines */
      case SLCDCODE_PAGEUP:             /* Cursor up by N pages */
      case SLCDCODE_PAGEDOWN:           /* Cursor down by N pages */
        slcd_put5(code, count, stream); /* Generate the 5-byte sequence */
        break;

      default:
      case SLCDCODE_NORMAL:             /* Not a special slcdcode */
        break;
    }
}
