/****************************************************************************
 * libs/libc/misc/lib_kbddecode.c
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
#include <debug.h>

#include <nuttx/streams.h>
#include <nuttx/ascii.h>
#include <nuttx/input/kbd_codec.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NDX_ESC        0
#define NDX_BRACKET    1
#define NDX_CODE       2
#define NDX_TERMINATOR 3

#define NCH_ESC        1
#define NCH_BRACKET    2
#define NCH_CODE       3
#define NCH_TERMINATOR 4

#define TERM_MIN       ('a' + KBD_RELEASE)
#define TERM_MAX       ('a' + KBD_SPECREL)
#define TERM_RETURN(a) ((a) - 'a')

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kbd_reget
 *
 * Description:
 *   We have unused characters from the last, unsuccessful decode attempt.
 *   Return one of these instead of the new character from the stream.
 *
 * Input Parameters:
 *   stream - An instance of lib_instream_s to do the low-level get
 *     operation.
 *   pch - The location character to save the returned value.  This may be
 *     either a normal, character code or a special command from enum
 *     kbd_keycode_e
 *
 * Returned Value:
 *   KBD_PRESS - Indicates the successful receipt of norma keyboard data.
 *
 ****************************************************************************/

static int kbd_reget(FAR struct kbd_getstate_s *state, FAR uint8_t *pch)
{
  /* Return the next character */

  *pch = state->buf[state->ndx];
  state->ndx++;
  state->nch--;
  return KBD_PRESS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kbd_decode
 *
 * Description:
 *   Get one byte of data or special command from the driver provided input
 *   buffer.
 *
 * Input Parameters:
 *   stream - An instance of lib_instream_s to do the low-level get
 *     operation.
 *   pch - The location to save the returned value.  This may be
 *     either a normal, character code or a special command from enum
 *     kbd_keycode_e
 *   state - A user provided buffer to support parsing.  This structure
 *     should be cleared the first time that kbd_decode is called.
 *
 * Returned Value:
 *
 *  KBD_PRESS  - Indicates the successful receipt of normal, keyboard data.
 *    This corresponds to a keypress event.  The returned value in pch is a
 *    simple byte of text or control data corresponding to the pressed key.
 *  KBD_RELEASE - Indicates a key release event.  The returned value in pch
 *    is the byte of text or control data corresponding to the released key.
 *  KBD_SPECPRESS - Indicates the successful receipt of a special keyboard
 *    command. The returned value in pch is a value from enum kbd_getstate_s.
 *  KBD_SPECREL - Indicates a special key release event.  The returned value
 *    in pch is a value from enum kbd_getstate_s.
 *  EOF - An error has getting the next character (reported by the stream).
 *    Normally indicates the end of file.
 *
 ****************************************************************************/

int kbd_decode(FAR struct lib_instream_s *stream,
               FAR struct kbd_getstate_s *state, FAR uint8_t *pch)
{
  int ch;

  DEBUGASSERT(stream && state && pch);

  /* Are their ungotten characters from the last, failed parse? */

  if (state->nch > 0)
    {
      /* Yes, return the next ungotten character */

      return kbd_reget(state, pch);
    }

  state->ndx = 0;

  /* No, ungotten characters.  Check for the beginning of an ESC sequence. */

  ch = stream->get(stream);
  if (ch == EOF)
    {
      /* End of file/stream */

      return KBD_ERROR;
    }
  else
    {
      state->buf[NDX_ESC] = (uint8_t)ch;
      state->nch = NCH_ESC;

      if (ch != ASCII_ESC)
        {
          /* Not the beginning of an escape sequence.
           *  Return the character.
           */

          return kbd_reget(state, pch);
        }
    }

  /* Check for ESC-[ */

  ch = stream->get(stream);
  if (ch == EOF)
    {
      /* End of file/stream.  Return the escape character now.  We will
       * return the EOF indication next time.
       */

      return kbd_reget(state, pch);
    }
  else
    {
      state->buf[NDX_BRACKET] = ch;
      state->nch = NCH_BRACKET;

      if (ch != '[')
        {
          /* Not the beginning of an escape sequence.  Return the ESC now,
           * return the following character later.
           */

          return kbd_reget(state, pch);
        }
    }

  /* Get and verify the special keyboard data to decode */

  ch = stream->get(stream);
  if (ch == EOF)
    {
      /* End of file/stream.  Unget everything and return the ESC character.
       */

      return kbd_reget(state, pch);
    }
  else
    {
      state->buf[NDX_CODE] = (uint8_t)ch;
      state->nch = NCH_CODE;

      /* Check for a valid special command code */

      if (ch < FIRST_KEYCODE || ch > LAST_KEYCODE)
        {
          /* Not a special command code, return the ESC now and the next two
           * characters later.
           */

          return kbd_reget(state, pch);
        }
    }

  /* Check for the final semicolon */

  ch = stream->get(stream);
  if (ch == EOF)
    {
      /* End of file/stream.  Unget everything and return the ESC character.
       */

      return kbd_reget(state, pch);
    }
  else
    {
      state->buf[NDX_TERMINATOR] = (uint8_t)ch;
      state->nch = NCH_TERMINATOR;

      /* Check for a valid special command code */

      if (ch < TERM_MIN || ch > TERM_MAX)
        {
          /* Not a special command code, return the ESC now and the next two
           * characters later.
           */

          return kbd_reget(state, pch);
        }
    }

  /* We have successfully parsed the entire escape sequence.  Return the
   * keyboard value in pch and the value an indication determined by the
   * terminating character.
   */

  *pch = state->buf[NDX_CODE];
  state->nch = 0;
  return TERM_RETURN(state->buf[NDX_TERMINATOR]);
}
