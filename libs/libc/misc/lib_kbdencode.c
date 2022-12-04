/****************************************************************************
 * libs/libc/misc/lib_kbdencode.c
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
#include <nuttx/input/kbd_codec.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: kbd_encode
 *
 * Description:
 *   Encode one special special sequence command into the output stream.
 *
 * Input Parameters:
 *   keycode - The command to be added to the output stream.
 *   stream - An instance of lib_outstream_s to do the low-level put
 *     operation.
 *   terminator - Escape sequence terminating character.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void kbd_encode(uint8_t keycode, FAR struct lib_outstream_s *stream,
                       uint8_t terminator)
{
  lib_stream_putc(stream, ASCII_ESC);
  lib_stream_putc(stream, '[');
  lib_stream_putc(stream, (int)keycode);
  lib_stream_putc(stream, terminator);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kbd_release
 *
 * Description:
 *   Encode the release of a normal key.
 *
 * Input Parameters:
 *   ch - The character associated with the key that was releared.
 *   stream - An instance of lib_outstream_s to do the low-level put
 *     operation.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void kbd_release(uint8_t ch, FAR struct lib_outstream_s *stream)
{
  kbd_encode(ch, stream, ('a' + KBD_RELEASE));
}

/****************************************************************************
 * Name: kbd_specpress
 *
 * Description:
 *   Denotes a special key press event.  Put one special keyboard command
 *   into the output stream.
 *
 * Input Parameters:
 *   keycode - The command to be added to the output stream.
 *   stream - An instance of lib_outstream_s to do the low-level put
 *     operation.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void kbd_specpress(enum kbd_keycode_e keycode,
                   FAR struct lib_outstream_s *stream)
{
  DEBUGASSERT(stream &&
              keycode >= KEYCODE_FWDDEL &&
              keycode <= LAST_KEYCODE);
  kbd_encode((uint8_t)keycode, stream, ('a' + KBD_SPECPRESS));
}

/****************************************************************************
 * Name: kbd_specrel
 *
 * Description:
 *   Denotes a special key release event.  Put one special keyboard
 *   command into the output stream.
 *
 * Input Parameters:
 *   keycode - The command to be added to the output stream.
 *   stream - An instance of lib_outstream_s to do the low-level put
 *     operation.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void kbd_specrel(enum kbd_keycode_e keycode,
                 FAR struct lib_outstream_s *stream)
{
  DEBUGASSERT(stream &&
              keycode >= KEYCODE_FWDDEL &&
              keycode <= LAST_KEYCODE);
  kbd_encode((uint8_t)keycode, stream, ('a' + KBD_SPECREL));
}
