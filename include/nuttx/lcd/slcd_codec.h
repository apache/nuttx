/****************************************************************************
 * include/nuttx/lcd/slcd_codec.h
 * Serialize and marshaling data and events for character-based, segment LCDs
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

#ifndef __INCLUDE_NUTTX_LCD_SLCD_CODEC_H
#define __INCLUDE_NUTTX_LCD_SLCD_CODEC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/streams.h>

#ifdef CONFIG_LIBC_SLCDCODEC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* These are the special SLCD commands recognized by the CODEC.  NOTE: that
 * some require a a count argument, N.
 */

enum slcdcode_e
{
  SLCDCODE_NORMAL = 0,      /* Not a special keycode */

  /* Delete and Backspace keycodes (in case they may be different than the
   * ASCII BKSP and DEL values.
   */

  SLCDCODE_FWDDEL,          /* DELete (forward delete) N characters moving text */
  SLCDCODE_BACKDEL,         /* Backspace (backward delete) N characters  moving cursor */
  SLCDCODE_ERASE,           /* Erase N characters from the cursor position */
  SLCDCODE_ERASEEOL,        /* Erase from the cursor position to the end of line */
  SLCDCODE_CLEAR,           /* Home the cursor and erase the entire display */

  /* Cursor movement */

  SLCDCODE_HOME,            /* Cursor home */
  SLCDCODE_END,             /* Cursor end */
  SLCDCODE_LEFT,            /* Cursor left by N characters */
  SLCDCODE_RIGHT,           /* Cursor right by N characters */
  SLCDCODE_UP,              /* Cursor up by N lines */
  SLCDCODE_DOWN,            /* Cursor down by N lines */
  SLCDCODE_PAGEUP,          /* Cursor up by N pages */
  SLCDCODE_PAGEDOWN,        /* Cursor down by N pages */

  /* Blinking */

  SLCDCODE_BLINKSTART,      /* Start blinking with current cursor position */
  SLCDCODE_BLINKEND,        /* End blinking after the current cursor position */
  SLCDCODE_BLINKOFF         /* Turn blinking off */
};

#define FIRST_SLCDCODE SLCDCODE_FWDDEL
#define LAST_SLCDCODE  SLCDCODE_BLINKOFF

/* Values returned by slcd_decode() */

enum slcdret_e
{
  SLCDRET_CHAR = 0,         /* A normal character was returned */
  SLCDRET_SPEC,             /* A special SLCD action was returned */
  SLCDRET_EOF               /* An EOF (or possibly an error) occurred */
};

/* Working data needed by slcd_encode that must be provided and initialized
 * by the caller.
 */

struct slcdstate_s
{
  uint8_t nch;              /* Number of characters in the buffer */
  uint8_t ndx;              /* Index to next character in the buffer */
  uint8_t buf[5];           /* Buffer of ungotten data */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * The following functions are intended for use by "producer", application
 * code to encode information into driver I/O buffers.
 ****************************************************************************/

/****************************************************************************
 * Name: slcd_put
 *
 * Description:
 *   Put one byte of normal character data into the output data stream.
 *
 * Input Parameters:
 *   ch - The character to be added to the output stream.
 *   stream - An instance of lib_outstream_s to do the low-level put
 *     operation.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#define slcd_put(ch, stream) lib_stream_putc(stream, ch)

/****************************************************************************
 * Name: slcd_encode
 *
 * Description:
 *   Encode one special action into the output data stream
 *
 * Input Parameters:
 *   code - The action to be taken
 *   count - The 8-bit unsigned count value N associated with some actions
 *   stream - An instance of lib_outstream_s to do the low-level put
 *     operation.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void slcd_encode(enum slcdcode_e code, uint8_t count,
                 FAR struct lib_outstream_s *stream);

/****************************************************************************
 * The following functions are intended for use by "consumer" SLCD driver
 * code to remove and decode information from the application provided
 * buffer.
 ****************************************************************************/

/****************************************************************************
 * Name: slcd_decode
 *
 * Description:
 *   Get one byte of data or special command from the application provided
 *   input buffer.
 *
 * Input Parameters:
 *   stream - An instance of lib_instream_s to do the low-level get
 *     operation.
 *   state - A user provided buffer to support parsing.  This structure
 *     should be cleared the first time that slcd_decode is called.
 *   pch - The location to save the returned value.  This may be
 *     either a normal, character code or a special command from enum
 *     slcdcode_e, depending on the return value from slcd_decode()
 *   parg - The location to save the count argument that accompanies some
 *     special actions
 *
 * Returned Value:
 *
 *   See enum slcdret_e
 *
 ****************************************************************************/

enum slcdret_e slcd_decode(FAR struct lib_instream_s *stream,
                           FAR struct slcdstate_s *state, FAR uint8_t *pch,
                           FAR uint8_t *parg);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_LIBC_SLCDCODEC */
#endif /* __INCLUDE_NUTTX_LCD_SLCD_CODEC_H */
