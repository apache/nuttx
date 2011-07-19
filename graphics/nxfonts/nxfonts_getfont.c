/****************************************************************************
 * graphics/nxfonts/nxfonts_getfont.c
 *
 *   Copyright (C) 2008-2009, 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT}
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING}
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

#include <stdint.h>
#include <stddef.h>
#include <debug.h>

#include <nuttx/nxfonts.h>

#include "nxfonts_internal.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
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
 * Name: nxf_getglyphset
 *
 * Description:
 *   Return information about the font set containtined he selected
 *   character encoding.
 *
 * Input Parameters:
 *   ch - character code
 *
 ****************************************************************************/

static inline FAR const struct nx_fontset_s *nxf_getglyphset(uint16_t ch)
{
  if (ch < 128)
    {
      if (ch >= g_7bitfonts.first && ch < g_7bitfonts.first + g_7bitfonts.nchars)
        {
          return &g_7bitfonts;
        }
      gdbg("No bitmap for 7-bit code %d\n", ch);
    }
  else if (ch < 256)
    {
#if CONFIG_NXFONTS_CHARBITS >= 8
      if (ch >= g_8bitfonts.first && ch < g_8bitfonts.first + g_8bitfonts.nchars)
        {
          return &g_8bitfonts;
        }
      gdbg("No bitmap for 8-bit code %d\n", ch);
#else
      gdbg("8-bit font support disabled: %d\n", ch);
#endif
    }
  else
    {
      gdbg("16-bit font not currently supported\n");
    }
  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxf_getfontset
 *
 * Description:
 *   Return information about the current font set
 *
 * Input Parameters:
 *   fontid:  Identifies the font set to get
 *
 ****************************************************************************/

FAR const struct nx_font_s *nxf_getfontset(enum nx_fontid_e fontid)
{
  return &g_fonts;
}

/****************************************************************************
 * Name: nxf_getbitmap
 *
 * Description:
 *   Return font bitmap information for the selected character encoding.
 *
 * Input Parameters:
 *   ch:      Character code
 *   fontid:  Identifies the font set to use
 *
 ****************************************************************************/

FAR const struct nx_fontbitmap_s *nxf_getbitmap(uint16_t ch,
                                                enum nx_fontid_e fontid)
{
  FAR const struct nx_fontset_s    *set = nxf_getglyphset(ch);
  FAR const struct nx_fontbitmap_s *bm  = NULL;

  if (set)
    {
      bm = &set->bitmap[ch - set->first];
    }
  return bm;
}
