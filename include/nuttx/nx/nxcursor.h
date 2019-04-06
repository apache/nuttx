/****************************************************************************
 * include/nuttx/nx/cursor.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights
 *     reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __INCLUDE_NUTTX_NX_NXCURSOR_H
#define __INCLUDE_NUTTX_NX_NXCURSOR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/video/cursor.h>

#ifndef defined(CONFIG_NX_SWCURSOR) || defined(CONFIG_NX_HWCURSOR)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxcursor_enable
 *
 * Description:
 *   Enable/disable presentation of the cursor
 *
 * Input Parameters:
 *   hnd    - The server handle returned by nx_connect()
 *   enable - True: show the cursor, false: hide the cursor.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxcursor_enable(NXHANDLE hnd, bool enable);

/****************************************************************************
 * Name: nxcursor_set_image
 *
 * Description:
 *   Set the cursor image
 *
 * Input Parameters:
 *   hnd   - The server handle returned by nx_connect()
 *   image - Describes the cursor image in the expected format.  For a
 *           software cursor, this is the format used with the display.  The
 *           format may be different if a hardware cursor is used.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_NX_HWCURSORIMAGE) || defined(CONFIG_NX_SWCURSOR)
void nxcursor_set_image(NXHANDLE hnd, FAR struct cursor_image_s *image);
#endif

/****************************************************************************
 * Name: nxcursor_set_position
 *
 * Description:
 *   Move the cursor to the specified position
 *
 * Input Parameters:
 *   hnd - The server handle returned by nx_connect()
 *   pos -
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxcursor_set_position(NXHANDLE hnd, FAR const struct cursor_pos_s *pos);

/****************************************************************************
 * Name: nxcursor_get_position
 *
 * Description:
 *   Return the current cursor position.
 *
 *   CAUTION:  The current cursor position is not updated until the display
 *   is actually changed.  Due to asynchronies caused by queue, the new
 *   current cursor position may not match the cursor position set until
 *   the client and server are syncrhonized.
 *
 * Input Parameters:
 *   hnd - The server handle returned by nx_connect()
 *   pos - The location to return the cursor position
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxcursor_get_position(NXHANDLE hnd, FAR struct cursor_pos_s *pos);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_NX_SWCURSOR || CONFIG_NX_HWCURSOR */
#endif /* __INCLUDE_NUTTX_NX_NXCURSOR_H */

