/****************************************************************************
 * graphics/nxterm/nxterm_putc.c
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

#include <nuttx/ascii.h>

#include "nxterm.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
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

  bm = nxterm_addchar(priv, ch);
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
  nxterm_hidechar(priv, &priv->cursor);
}
