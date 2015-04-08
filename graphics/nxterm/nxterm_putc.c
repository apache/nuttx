/****************************************************************************
 * nuttx/graphics/nxterm/nxterm_putc.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/ascii.h>

#include "nxterm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
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
 * Name: nxterm_putc
 *
 * Description:
 *   Render the specified character at the current display position.
 *
 ****************************************************************************/

void nxterm_putc(FAR struct nxterm_state_s *priv, uint8_t ch)
{
  FAR const struct nxterm_bitmap_s *bm;
  int lineheight;

  /* Ignore carriage returns */

  if (ch == '\r')
    {
      return;
    }

  /* Handle backspace (treating both BS and DEL as backspace) */

  if (ch == ASCII_BS || ch == ASCII_DEL)
    {
      nxterm_backspace(priv);
      return;
    }

  /* Will another character fit on this line? */

  if (priv->fpos.x + priv->fwidth > priv->wndo.wsize.w)
    {
#ifndef CONFIG_NXTERM_NOWRAP
      /* No.. move to the next line */

      nxterm_newline(priv);

      /* If we were about to output a newline character, then don't */

      if (ch == '\n')
        {
          return;
        }
#else
      /* No.. Ignore all further characters until a newline is encountered */

      if (ch != '\n')
        {
          return;
        }
#endif
    }

  /* If it is a newline character, then just perform the logical newline
   * operation.
   */

  if (ch == '\n')
    {
      nxterm_newline(priv);
      return;
    }

  /* Check if we need to scroll up */

  lineheight = (priv->fheight + CONFIG_NXTERM_LINESEPARATION);
  while (priv->fpos.y >= priv->wndo.wsize.h - lineheight)
    {
      nxterm_scroll(priv, lineheight);
    }

  /* Find the glyph associated with the character and render it onto the
   * display.
   */

  bm = nxterm_addchar(priv->font, priv, ch);
  if (bm)
    {
      nxterm_fillchar(priv, NULL, bm);
    }
}

/****************************************************************************
 * Name: nxterm_showcursor
 *
 * Description:
 *   Render the cursor character at the current display position.
 *
 ****************************************************************************/

void nxterm_showcursor(FAR struct nxterm_state_s *priv)
{
  int lineheight;

  /* Will another character fit on this line? */

  if (priv->fpos.x + priv->fwidth > priv->wndo.wsize.w)
    {
#ifndef CONFIG_NXTERM_NOWRAP
      /* No.. move to the next line */

      nxterm_newline(priv);
#else
      return;
#endif
    }

  /* Check if we need to scroll up */

  lineheight = (priv->fheight + CONFIG_NXTERM_LINESEPARATION);
  while (priv->fpos.y >= priv->wndo.wsize.h - lineheight)
    {
      nxterm_scroll(priv, lineheight);
    }

  /* Render the cursor glyph onto the display. */

  priv->cursor.pos.x = priv->fpos.x;
  priv->cursor.pos.y = priv->fpos.y;
  nxterm_fillchar(priv, NULL, &priv->cursor);
}

/****************************************************************************
 * Name: nxterm_hidecursor
 *
 * Description:
 *   Render the cursor cursor character from the display.
 *
 ****************************************************************************/

void nxterm_hidecursor(FAR struct nxterm_state_s *priv)
{
  (void)nxterm_hidechar(priv, &priv->cursor);
}
