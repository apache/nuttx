/****************************************************************************
 * graphics/nxterm/nxterm_vt100.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <string.h>
#include <assert.h>

#include <nuttx/vt100.h>

#include "nxterm.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef int (*seqhandler_t)(FAR struct nxterm_state_s *priv);

struct vt100_sequence_s
{
  FAR const char *seq;
  seqhandler_t handler;
  uint8_t size;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int nxterm_erasetoeol(FAR struct nxterm_state_s *priv);
static int nxterm_cursorleft(FAR struct nxterm_state_s *priv);
static int nxterm_cursorright(FAR struct nxterm_state_s *priv);
static enum nxterm_vt100state_e
  nxterm_sgr(FAR struct nxterm_state_s *priv, int seqsize);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* All recognized VT100 escape sequences.  Very little as present, this is
 * a placeholder for a future, more complete VT100 emulation.
 */

/* <esc>[K is the VT100 command erases to the end of the line. */

static const char g_erasetoeol[] = VT100_CLEAREOL;
static const char g_cursorleft[] =
{
  ASCII_ESC, '[', 'D'
};

static const char g_cursorright[] =
{
  ASCII_ESC, '[', 'C'
};

/* The list of all VT100 sequences supported by the emulation */

static const struct vt100_sequence_s g_vt100sequences[] =
{
  {g_erasetoeol, nxterm_erasetoeol, sizeof(g_erasetoeol)},
  {g_cursorleft, nxterm_cursorleft, sizeof(g_cursorleft)},
  {g_cursorright, nxterm_cursorright, sizeof(g_cursorright)},
  {NULL, NULL, 0}
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxterm_erasetoeol
 *
 * Description:
 *   Handle the erase-to-eol VT100 escapte sequence
 *
 * Input Parameters:
 *   priv - Driver data structure
 *
 * Returned Value:
 *   The index of the match in g_vt100sequences[]
 *
 ****************************************************************************/

static int nxterm_erasetoeol(FAR struct nxterm_state_s *priv)
{
  struct nxgl_rect_s bounds;
  uint16_t dst;
  uint16_t src;

  bounds.pt1.x = priv->fpos.x;
  bounds.pt1.y = priv->fpos.y;
  bounds.pt2.x = priv->wndo.wsize.w - 1;
  bounds.pt2.y = priv->fpos.y + priv->fheight - 1;

  priv->ops->fill(priv, &bounds, priv->wndo.wcolor);

  /* Discard every saved glyph covered by the erased portion of this line.
   * Keeping the bitmap list in sync is necessary for later redraw events.
   */

  for (src = 0, dst = 0; src < priv->nchars; src++)
    {
      if (priv->bm[src].pos.y == priv->fpos.y &&
          priv->bm[src].pos.x >= priv->fpos.x)
        {
          continue;
        }

      if (dst != src)
        {
          priv->bm[dst] = priv->bm[src];
        }

      dst++;
    }

  priv->nchars = dst;

  return OK;
}

/****************************************************************************
 * Name: nxterm_cursorleft
 ****************************************************************************/

static int nxterm_cursorleft(FAR struct nxterm_state_s *priv)
{
  nxgl_coord_t x = priv->spwidth;
  uint16_t i;

  /* Find the closest saved glyph to the left on the current line. */

  for (i = 0; i < priv->nchars; i++)
    {
      if (priv->bm[i].pos.y == priv->fpos.y &&
          priv->bm[i].pos.x < priv->fpos.x &&
          priv->bm[i].pos.x > x)
        {
          x = priv->bm[i].pos.x;
        }
    }

  priv->fpos.x = x;
  return OK;
}

/****************************************************************************
 * Name: nxterm_cursorright
 ****************************************************************************/

static int nxterm_cursorright(FAR struct nxterm_state_s *priv)
{
  FAR const struct nxfonts_glyph_s *glyph;
  int i;

  /* Prefer the most recently rendered glyph when editing has overdrawn a
   * character at the same location.
   */

  for (i = priv->nchars - 1; i >= 0; i--)
    {
      if (priv->bm[i].pos.y == priv->fpos.y &&
          priv->bm[i].pos.x == priv->fpos.x)
        {
          glyph = nxf_cache_getglyph(priv->fcache, priv->bm[i].code);
          priv->fpos.x += glyph == NULL ? priv->spwidth : glyph->width;
          break;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: nxterm_sgr
 *
 * Description:
 *   Consume an ANSI Select Graphic Rendition sequence.  NxTerm does not
 *   currently support changing text attributes, but consuming the sequence
 *   prevents unsupported color controls from being rendered as text.
 *
 ****************************************************************************/

static enum nxterm_vt100state_e
nxterm_sgr(FAR struct nxterm_state_s *priv, int seqsize)
{
  int i;

  if (seqsize < 2 || priv->seq[0] != ASCII_ESC || priv->seq[1] != '[')
    {
      return VT100_ABORT;
    }

  for (i = 2; i < seqsize; i++)
    {
      char ch = priv->seq[i];

      if (ch == 'm')
        {
          priv->nseq = 0;
          return VT100_PROCESSED;
        }

      if ((ch < '0' || ch > '9') && ch != ';' && ch != ':')
        {
          return VT100_ABORT;
        }
    }

  return seqsize < VT100_MAX_SEQUENCE ? VT100_CONSUMED : VT100_ABORT;
}

/****************************************************************************
 * Name: nxterm_vt100part
 *
 * Description:
 *   Return the next entry that is a partial match to the sequence.
 *
 * Input Parameters:
 *   priv - Driver data structure
 *   seqsize - The number of bytes in the sequence
 *   startndx - The index to start searching
 *
 * Returned Value:
 *   A pointer to the matching sequence in g_vt100sequences[]
 *
 ****************************************************************************/

FAR const struct vt100_sequence_s *
nxterm_vt100part(FAR struct nxterm_state_s *priv, int seqsize)
{
  FAR const struct vt100_sequence_s *seq;
  int ndx;

  /* Search from the beginning of the sequence table */

  for (ndx = 0; g_vt100sequences[ndx].seq; ndx++)
    {
      /* Is this sequence big enough? */

      seq = &g_vt100sequences[ndx];
      if (seq->size >= seqsize)
        {
          /* Yes... are the first 'seqsize' bytes the same */

          if (memcmp(seq->seq, priv->seq, seqsize) == 0)
            {
              /* Yes.. return the match */

              return seq;
            }
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: nxterm_vt100seq
 *
 * Description:
 *   Determine if the new sequence is a part of a supported VT100 escape
 *   sequence.
 *
 * Input Parameters:
 *   priv - Driver data structure
 *   seqsize - The number of bytes in the sequence
 *
 * Returned Value:
 *   state - See enum nxterm_vt100state_e;
 *
 ****************************************************************************/

static enum nxterm_vt100state_e nxterm_vt100seq(
                                             FAR struct nxterm_state_s *priv,
                                             int seqsize)
{
  FAR const struct vt100_sequence_s *seq;
  enum nxterm_vt100state_e ret;

  /* Is there any VT100 escape sequence that matches what we have
   * buffered so far?
   */

  seq = nxterm_vt100part(priv, seqsize);
  if (seq)
    {
      /* Yes.. if the size of that escape sequence is the same as what we
       * have buffered, then we have an exact match.
       */

      if (seq->size == seqsize)
        {
          /* Process the VT100 sequence */

          seq->handler(priv);
          priv->nseq = 0;
          return VT100_PROCESSED;
        }

      /* The 'seqsize' is still smaller than the potential match(es).  We
       * will need to collect more characters before we can make a decision.
       * Return an indication that we have consumed the character.
       */

      return VT100_CONSUMED;
    }

  /* SGR parameters are variable-length.  Consume these sequences even
   * though NxTerm does not yet implement their visual attributes.
   */

  ret = nxterm_sgr(priv, seqsize);
  if (ret != VT100_ABORT)
    {
      return ret;
    }

  /* We get here on a failure.  The buffer sequence is not part of any
   * supported VT100 escape sequence.  If seqsize > 1 then we need to
   * return a special value because we have to re-process the buffered
   * data.
   */

  ret = seqsize > 1 ? VT100_ABORT : VT100_NOT_CONSUMED;
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxterm_vt100
 *
 * Description:
 *   Test if the newly received byte is part of a VT100 escape sequence
 *
 * Input Parameters:
 *   priv - Driver data structure
 *   ch - The newly received character
 *
 * Returned Value:
 *   state - See enum nxterm_vt100state_e;
 *
 ****************************************************************************/

enum nxterm_vt100state_e nxterm_vt100(FAR struct nxterm_state_s *priv,
                                      char ch)
{
  enum nxterm_vt100state_e ret;
  int seqsize;

  DEBUGASSERT(priv && priv->nseq < VT100_MAX_SEQUENCE);

  /* If we have no buffered characters, then 'ch' must be the first character
   * of an escape sequence.
   */

  if (priv->nseq < 1)
    {
      /* The first character of an escape sequence must be an an escape
       * character (duh).
       */

      if (ch != ASCII_ESC)
        {
          return VT100_NOT_CONSUMED;
        }

      /* Add the escape character to the buffer but don't bother with any
       * further checking.
       */

      priv->seq[0] = ASCII_ESC;
      priv->nseq   = 1;
      return VT100_CONSUMED;
    }

  /* Temporarily add the next character to the buffer */

  seqsize = priv->nseq;
  priv->seq[seqsize] = ch;

  /* Then check if this sequence is part of an a valid escape sequence */

  seqsize++;
  ret = nxterm_vt100seq(priv, seqsize);
  if (ret == VT100_CONSUMED)
    {
      /* The newly added character is indeed part of a VT100 escape sequence
       * (which is still incomplete).  Keep it in the buffer.
       */

      priv->nseq = seqsize;
    }

  return ret;
}
