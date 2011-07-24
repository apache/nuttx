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

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxfonts.h>

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

#ifdef CONFIG_NXFONT_SANS23X27
extern const struct nx_fontpackage_s g_sans23x27_package;
#endif

static FAR const struct nx_fontpackage_s *g_fontpackages[] =
{
#ifdef CONFIG_NXFONT_SANS23X27
  &g_sans23x27_package,
#endif
  NULL
};

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
 *   ch: character code
 *   package: The selected font package
 *
 ****************************************************************************/

static inline FAR const struct nx_fontset_s *
  nxf_getglyphset(uint16_t ch, FAR const struct nx_fontpackage_s *package)
{
  FAR const struct nx_fontset_s *fontset;

  /* Select the 7- or 8-bit font set */

  if (ch < 128)
    {
      /* Select the 7-bit font set */

      fontset = &package->font7;
    }
  else if (ch < 256)
    {
#if CONFIG_NXFONTS_CHARBITS >= 8
      /* Select the 8-bit font set */

      fontset = &package->font8;
#else
      gdbg("8-bit font support disabled: %d\n", ch);
      return NULL;
#endif
    }
  else
    {
      /* Someday, perhaps 16-bit fonts will go here */

      gdbg("16-bit font not currently supported\n");
      return NULL;
    }

  /* Then verify that the character actually resides in the font */

  if (ch >= fontset->first && ch < fontset->first +fontset->nchars)
    {
      return fontset;
    }

  gdbg("No bitmap for code %02x\n", ch);
  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxf_getfonthandle
 *
 * Description:
 *   Given a numeric font ID, return a handle that may be subsequently be
 *   used to access the font data sets.
 *
 * Input Parameters:
 *   fontid:  Identifies the font set to get
 *
 ****************************************************************************/

NXHANDLE nxf_getfonthandle(enum nx_fontid_e fontid)
{
  FAR const struct nx_fontpackage_s **pkglist;
  FAR const struct nx_fontpackage_s *package;

  /* Handle the default font package */

  if (fontid == FONTID_DEFAULT)
   {
     fontid = NXFONT_DEFAULT;
   }

  /* Then search for the font package with this ID */

  for (pkglist = g_fontpackages; *pkglist; pkglist++)
    {
      package = *pkglist;
      if (package->id == fontid)
        {
          return (NXHANDLE)package;
        }
    }

  return (NXHANDLE)NULL;
}

/****************************************************************************
 * Name: nxf_getfontset
 *
 * Description:
 *   Return information about the current font set
 *
 * Input Parameters:
 *   handle:  A font handle previously returned by nxf_getfonthandle
 *
 ****************************************************************************/

FAR const struct nx_font_s *nxf_getfontset(NXHANDLE handle)
{
  FAR const struct nx_fontpackage_s *package =
    (FAR const struct nx_fontpackage_s *)handle;

  /* Find the font package associated with this font ID */

  if (package)
    {
      /* Found... return the font set metrics for this font package */

      return &package->metrics;
    }

  return NULL;
}

/****************************************************************************
 * Name: nxf_getbitmap
 *
 * Description:
 *   Return font bitmap information for the selected character encoding.
 *
 * Input Parameters:
 *   handle:  A font handle previously returned by nxf_getfonthandle
 *   ch:      Character code whose bitmap is requested
 *
 * Returned Value:
 *   An instance of struct nx_fontbitmap_s describing the glyph.
 *
 ****************************************************************************/

FAR const struct nx_fontbitmap_s *nxf_getbitmap(NXHANDLE handle, uint16_t ch)
{
  FAR const struct nx_fontpackage_s *package =
    (FAR const struct nx_fontpackage_s *)handle;
  FAR const struct nx_fontset_s     *fontset;
  FAR const struct nx_fontbitmap_s  *bm  = NULL;

  /* Verify that the handle is a font package */

  if (package)
    {
      /* Now get the fontset from the package */

      fontset = nxf_getglyphset(ch, package);
      if (fontset)
        {
          /* Then get the bitmap from the font set */

          bm = &fontset->bitmap[ch - fontset->first];
        }
    }

  return bm;
}
