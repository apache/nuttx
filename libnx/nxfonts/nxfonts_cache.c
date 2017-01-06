/****************************************************************************
 * libnx/nxfonts/nxfonts_cache.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#include <string.h>
#include <semaphore.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/nx/nxfonts.h>

#include "nxcontext.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This describes a rendering function */

typedef CODE int (*nxf_renderer_t)(FAR nxgl_mxpixel_t *dest, uint16_t height,
                                   uint16_t width, uint16_t stride,
                                   FAR const struct nx_fontbitmap_s *bm,
                                   nxgl_mxpixel_t color);

/* This structure defines one font cache */

struct nxfonts_fcache_s
{
  FAR struct nxfonts_fcache_s *flink;  /* Supports a singly linked list */
  NXHANDLE font;                       /* Font handle associated with fontid */
  sem_t fsem;                          /* Serializes access to the font cache */
  uint16_t fontid;                     /* ID of font in this cache */
  int16_t fclients;                    /* Number of connected clients */
  uint8_t maxglyphs;                   /* Maximum size of glyph[] array */
  uint8_t nglyphs;                     /* Current size of glyph[] array */
  uint8_t bpp;                         /* Bits per pixel */
  nxgl_mxpixel_t fgcolor;              /* Foreground color */
  nxgl_mxpixel_t bgcolor;              /* Background color */
  nxf_renderer_t renderer;             /* Font renderer */

  /* Glyph cache data storage */

  FAR struct nxfonts_glyph_s *head;    /* Head of the list of glyphs */
  FAR struct nxfonts_glyph_s *tail;    /* Tail of the list of glyphs */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Head of a list of font caches */

static FAR struct nxfonts_fcache_s *g_fcaches;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxf_removeglyph
 *
 * Description:
 *   Removes the entry 'glyph' from the font cache.
 *
 ****************************************************************************/

static inline void nxf_removeglyph(FAR struct nxfonts_fcache_s *priv,
                                   FAR struct nxfonts_glyph_s *glyph,
                                   FAR struct nxfonts_glyph_s *prev)
{
  /* Remove the glyph for the list.  First check for removal from the head */

  if (prev == NULL)
    {
      /* Replace the head with the node following glyph */

      priv->head = glyph->flink;

      /* If there is no node following glyph, then the list is empty */

      if (priv->head == NULL)
        {
          priv->tail = NULL;
        }
    }

  /* Check for removal from the tail (we know that the list cannot become
   * empty in either of the next two cases).
   */

  else if (glyph->flink == NULL)
    {
      priv->tail = prev;
      prev->flink = NULL;
    }

  /* No.. removae from mid-list */

  else
    {
      prev->flink = glyph->flink;
    }

  glyph->flink = NULL;

  /* Decrement the count of glyphs in the font cache */

  DEBUGASSERT(priv->nglyphs > 0);
  priv->nglyphs--;
}

/****************************************************************************
 * Name: nxf_addglyph
 *
 * Description:
 *   Add the entry 'glyph' to the head font cache list.
 *
 ****************************************************************************/

static inline void nxf_addglyph(FAR struct nxfonts_fcache_s *priv,
                                FAR struct nxfonts_glyph_s *glyph)
{
  /* Add the glyph to the head of the list */

  glyph->flink = priv->head;

  if (priv->head == NULL)
    {
      priv->tail = glyph;
    }

  priv->head = glyph

  /* Increment the count of glyphs in the font cache. */

  DEBUGASSERT(priv->nglyphs < priv->maxglyphs);
  priv->nglyphs++;
}

/****************************************************************************
 * Name: nxf_findglyph
 *
 * Description:
 *   Find the glyph for the specific character 'ch' in the list of pre-
 *   rendered fonts in the font cache.
 *
 *   This is logically a part of nxf_cache_getglyph().  nxf_cache_getglyph()
 *   will attempt to find the cached glyph before rendering a new one.  So
 *   this function has two unexpected side-effects:  (1) If the font cache
 *   is full and the font is not found, then the least-recently-used glyph
 *   is deleted to make space for the new glyph that will be allocated.
 *
 *   If the glyph is found, then it is moved to the head of the list of
 *   glyphs since it is now the most recently used (leaving the least
 *   recently used glyph at the tail of the list).
 *
 ****************************************************************************/

static FAR struct nxfonts_glyph_s *
nxf_findglyph(FAR struct nxfonts_fcache_s *priv, uint8_t ch)
{
  FAR struct nxfonts_glyph_s *glyph;
  FAR struct nxfonts_glyph_s *prev;

  /* Try to find the glyph in the list of pre-rendered glyphs */

   for (prev = NULL, glyph = priv->head;
        glyph != NULL;
        prev = glyph, glyph = glyph->flink)
    {
      /* Check if we found the the glyph for this character */

      if (glyph->code == ch)
        {
          /* This is now the most recently used glyph.  Move it to the head
           * of the list.
           */

          nxf_removeglyph(priv, glyph, prev);
          nxf_addglyph(priv, glyph);

          /* And return the glyph that we found */

          return glyph;
        }

      /* Is this the last glyph in the list?  Has the cache reached its
       * limited for the number of cached fonts?
       */

      if (glyph->flink == NULL && priv->nglyphs >= priv->maxglyphs)
        {
          /* Yes.. then remove it from the list and free the glyph memory.
           * We do this because we have all of the information in hand now
           * and we will surely need to have this space later.
           */

          nxf_removeglyph(priv, glyph, prev);
          return NULL;
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: nxf_fillglyph
 *
 * Description:
 *   Fill the glyph memory with the background color
 *
 ****************************************************************************/

static inline void nxf_fillglyph(FAR struct nxfonts_fcache_s *priv,
                                 FAR struct nxfonts_glyph_s *glyph)
{
  int row;
  int col;

  /* Initialize the glyph memory to the background color. */

#if !defined(CONFIG_NX_DISABLE_1BPP) || !defined(CONFIG_NX_DISABLE_2BPP) || \
    !defined(CONFIG_NX_DISABLE_4BPP) || !defined(CONFIG_NX_DISABLE_8BPP)

  /* For pixel depths of 1, 2, 4, and 8, build up an 8-bit value containing
   * multiple background colored pixels.
   */

  if (priv->bpp <= 8)
    {
      uint8_t pixel = (uint8_t)priv->bgcolor;
      FAR uint8_t *ptr;

#ifndef CONFIG_NX_DISABLE_1BPP
      /* Pack a 1-bit pixel to 2 pixels */

      if (priv->bpp < 2)
        {
          /* Pack 1-bit pixels into a 2-bits */

          pixel &= 0x01;
          pixel  = (pixel) << 1 | pixel;
        }
#endif

#if !defined(CONFIG_NX_DISABLE_1BPP) || !defined(CONFIG_NX_DISABLE_2BPP)
      /* Pack a 2-bit pixel to a 4-bit nibble */

      if (priv->bpp < 4)
        {
          /* Pack 2-bit pixels into a nibble */

          pixel &= 0x03;
          pixel  = (pixel) << 2 | pixel;
        }
#endif

#if !defined(CONFIG_NX_DISABLE_1BPP) || !defined(CONFIG_NX_DISABLE_2BPP) || \
    !defined(CONFIG_NX_DISABLE_4BPP)
      /* Pack the 4-bit nibble into a byte */

      if (priv->bpp < 8)
        {
          pixel &= 0x0f;
          pixel  = (pixel) << 4 | pixel;
        }
#endif

      /* Then fill the glyph with the packed background color */

      ptr = (FAR uint8_t *)glyph->bitmap;
      for (row = 0; row < glyph->height; row++)
        {
          for (col = 0; col < glyph->stride; col++)
            {
              /* Transfer the packed bytes into the buffer */

              *ptr++ = pixel;
            }
        }
    }
  else
#endif

#if !defined(CONFIG_NX_DISABLE_16BPP)
  if (priv->bpp == 16)
    {
      FAR uint16_t *ptr = (FAR uint16_t *)glyph->bitmap;

      for (row = 0; row < glyph->height; row++)
        {
          /* Just copy the color value into the glyph memory */

          for (col = 0; col < glyph->width; col++)
            {
              *ptr++ = priv->bgcolor;
            }
        }
    }
  else
#endif

#ifndef CONFIG_NX_DISABLE_24BPP
  if (priv->bpp == 24)
    {
      gerr("ERROR: Additional logic is needed to support 24-bit color\n");
      goto errout_with_glyph;
    }
  else
#endif

#if !defined(CONFIG_NX_DISABLE_32BPP)
  if (priv->bpp == 32)
    {
      FAR uint32_t *ptr = (FAR uint32_t *)glyph->bitmap;

      for (row = 0; row < glyph->height; row++)
        {
          /* Just copy the color value into the glyph memory */

          for (col = 0; col < glyph->width; col++)
            {
              *ptr++ = priv->bgcolor;
            }
        }
    }
  else
#endif
    {
      PANIC();
    }
}

/****************************************************************************
 * Name: nxf_renderglyph
 ****************************************************************************/

static inline FAR struct nxfonts_glyph_s *
nxf_renderglyph(FAR struct nxfonts_fcache_s *priv,
                FAR const struct nx_fontbitmap_s *fbm, uint8_t ch)
{
  FAR struct nxfonts_glyph_s *glyph = NULL;
  size_t bmsize;
  unsigned int height;
  unsigned int width;
  unsigned int stride;
  int ret;

  /* Get the size of the glyph */

  width  = fbm->metric.width + fbm->metric.xoffset;
  height = fbm->metric.height + fbm->metric.yoffset;

  /* Get the physical width of the glyph in bytes */

  stride = (width * priv->bpp + 7) >> 3;

  /* Allocate the glyph (always succeeds) */

  bmsize = stride * height;
  glyph  = (FAR struct nxfonts_glyph_s *)lib_malloc(SIZEOF_NXFONTS_GLYPH_S(bmsize));

  if (glyph != NULL)
    {
      /* Save the character code, dimensions, and physcial width of the glyph */

      glyph->code   = ch;
      glyph->width  = width;
      glyph->height = height;
      glyph->stride = stride;

      /* Initialize the glyph memory to the background color. */

      nxf_fillglyph(priv, glyph);

      /* Then render the glyph into the allocated, initialized memory */

      ret = priv->renderer((FAR nxgl_mxpixel_t *)glyph->bitmap,
                           glyph->height, glyph->width, glyph->stride,
                           fbm, priv->fgcolor);
      if (ret < 0)
        {
          /* Actually, the renderer never returns a failure */

          gerr("ERROR: nxf_renderglyph: Renderer failed\n");
          lib_free(glyph);
          return NULL;
        }

      /* Add the new glyph to the font cache */

      nxf_addglyph(priv, glyph);
    }

  return glyph;
}

/****************************************************************************
 * Name: nxf_findcache
 ****************************************************************************/

static FAR struct nxfonts_fcache_s *
nxf_findcache(enum nx_fontid_e fontid, nxgl_mxpixel_t fgcolor,
              nxgl_mxpixel_t bgcolor, int bpp)
{
  FAR struct nxfonts_fcache_s *fcache;

  /* Get exclusive access to the font cache list */
#warning Missing logic

  /* Search for a cache for this font characteristics */

  for (fcache = g_fcaches; fcache != NULL; fcache = fcache->flink)
    {
      /* Does this font have the same characteristics? */

      if (fcache->fontid  == fontid &&
          fcache->fgcolor == fgcolor &&
          fcache->bgcolor == bgcolor &&
          fcache->bpp     == bpp)
        {
          /* Yes... return it */

          return fcache;
        }
    }

  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxf_cache_connect
 *
 * Description:
 *   Create a new font cache for the provided 'fontid'.  If the cache
 *   already, then just increment a reference count return the handle for
 *   the existing font cache.
 *
 * Input Parameters:
 *   fontid    - Identifies the font supported by this cache
 *   fgcolor   - Foreground color
 *   bgcolor   - Background color
 *   bpp       - Bits per pixel
 *   maxglyphs - Maximum number of glyphs permitted in the cache
 *
 * Returned value:
 *   On success a non-NULL handle is returned that then may sequently be
 *   used with nxf_getglyph() to extract fonts from the font cache.  NULL
 *   returned on any failure with the errno value set to indicate the nature
 *   of the error.
 *
 ****************************************************************************/

FCACHE nxf_cache_connect(enum nx_fontid_e fontid,
                         nxgl_mxpixel_t fgcolor, nxgl_mxpixel_t bgcolor,
                         int bpp, int maxglyphs)
{
  FAR struct nxfonts_fcache_s *priv;
  int errcode;

  /* Find a font cache with the matching font characteristics */

  priv = nxf_findcache(fontid, fgcolor, bgcolor, bpp);
  if (priv == NULL)
    {
      /* There isn't one... we will have to create a new font cache for this
       * client.
       */

      /* Allocate memory for the (empty) font cache */

      priv = (FAR struct nxfonts_fcache_s *)
        lib_zalloc(sizeof( struct nxfonts_fcache_s));

      if (priv == NULL)
        {
          errcode = ENOMEM;
          goto errout;
        }

      /* Initialize the font cache */

      priv->maxglyphs = maxglyphs;
      priv->fontid    = fontid;
      priv->fgcolor   = fgcolor;
      priv->bgcolor   = bgcolor;
      priv->bpp       = bpp;

      /* Select the rendering function */

      /* Select renderer -- Some additional logic would be required to
       * support pixel depths that are not directly addressable (1,2,4, and
       * 24).
       */

#ifndef CONFIG_NX_DISABLE_1BPP
      if (bpp == 1)
        {
          priv->renderer = (nxf_renderer_t)nxf_convert_1bpp;
        }
      else
#endif
#ifndef CONFIG_NX_DISABLE_2BPP
      if (bpp == 2)
        {
          priv->renderer = (nxf_renderer_t)nxf_convert_2bpp;
        }
      else
#endif
#ifndef CONFIG_NX_DISABLE_4BPP
      if (bpp == 4)
        {
          priv->renderer = (nxf_renderer_t)nxf_convert_4bpp;
        }
      else
#endif
#ifndef CONFIG_NX_DISABLE_8BPP
      if (bpp == 8)
        {
          priv->renderer = (nxf_renderer_t)nxf_convert_8bpp;
        }
      else
#endif
#ifndef CONFIG_NX_DISABLE_16BPP
      if (bpp == 16)
        {
          priv->renderer = (nxf_renderer_t)nxf_convert_16bpp;
        }
      else
#endif
#ifndef CONFIG_NX_DISABLE_24BPP
      if (bpp == 24)
        {
          priv->renderer = (nxf_renderer_t)nxf_convert_24bpp;
        }
      else
#endif
#ifndef CONFIG_NX_DISABLE_32BPP
      if (bpp == 32)
        {
          priv->renderer = (nxf_renderer_t)nxf_convert_32bpp;
        }
      else
#endif
        {
          gerr("ERROR: Unsupported pixel depth: %d\n", bpp);
          errcode = ENOSYS;
          goto errout_with_fcache;
        }

      /* Select the font */

      priv->font = nxf_getfonthandle(fontid);
      if (priv->font == NULL)
        {
          errcode = get_errno();
          gerr("ERROR: Failed to get font ID %d: %d\n",  fontid, errcode);
          goto errout_with_fcache;
        }
    }
  else
    {
      /* A font cache with these characteristics already exists.  Just make
       * sure that it is as least a big as the size requested.
       */

      if (priv->maxglyphs < maxglyphs)
        {
          priv->maxglyphs = maxglyphs;
        }
    }

  return (FCACHE)priv;

errout_with_fcache:
  lib_free(priv);
errout:
  set_errno(errcode);
  return NULL;
}

/****************************************************************************
 * Name: nxf_cache_disconnect
 *
 * Description:
 *   Decrement the reference count on the font cache and, if the reference
 *   count goes to zero, free all resources used by the font cache.  The
 *   font handle is invalid upon return in either case.
 *
 * Input Parameters:
 *   fcache - A font cache handle previously returned by nxf_cache_connect();
 *
 * Returned value:
 *   None
 *
 ****************************************************************************/

void nxf_cache_disconnect(FCACHE fcache)
{
  FAR struct nxfonts_fcache_s *priv = (FAR struct nxfonts_fcache_s *)fcache;
  FAR struct nxfonts_glyph_s *glyph;
  FAR struct nxfonts_glyph_s *next;
  int ret;

  DEBUGASSERT(priv != NULL && priv->fclients > 0);

  /* Get exclusive access to the font cache */

  while ((ret = sem_wait(&priv->fsem)) < 0)
    {
      int errorcode = errno;
      DEBUGASSERT(errorcode == EINTR || errorcode == ECANCELED);
      UNUSED(errorcode);
    }

  /* Is this the last client of the font cache? */

  if (priv->fclients <= 1)
    {
      /* Yes.. destroy the font cache */

      /* Free all allocated glyph memory */

      for (glyph = priv->head; glyph != NULL; glyph = next)
        {
          next = glyph->flink;
          lib_free(glyph);
        }

      /* Destroy the serializing semaphore... while we are holding it? */

      sem_destroy(&priv->fsem);

      /* Finally, free the font cache stucture itself */

      lib_free(priv);
    }
  else
    {
      /* No.. just decrement the number of clients connected to the font
       * cache.
       */

      priv->fclients--;
      sem_post(&priv->fsem);
    }
}

/****************************************************************************
 * Name: nxf_cache_getfonthandle
 *
 * Description:
 *   Return the handle to the font set used by this instance of the font
 *   cache.
 *
 * Input Parameters:
 *   fcache - A font cache handle previously returned by nxf_cache_connect();
 *
 * Returned value:
 *   Zero (OK) is returned if the metrics were
 *
 * Returned Value:
 *   One success, a non-NULL font handle is returned.
 *
 ****************************************************************************/

NXHANDLE nxf_cache_getfonthandle(FCACHE fcache)
{
  FAR struct nxfonts_fcache_s *priv = (FAR struct nxfonts_fcache_s *)fcache;

  DEBUGASSERT(priv != NULL && priv->font != NULL);
  return priv->font;
}

/****************************************************************************
 * Name: nxf_cache_getglyph
 *
 * Description:
 *   Get the font glyph for the character code 'ch' from the font cache.  If
 *   the glyph for that character code does not exist in the font cache, it
 *   be rendered.
 *
 * Returned Value:
 *   On success, a non-NULL pointer to the rendered glyph in the font cache
 *   is returned.  NULL is returned on any failure.
 *
 ****************************************************************************/

FAR const struct nxfonts_glyph_s *nxf_cache_getglyph(FCACHE fcache, uint8_t ch)
{
  FAR struct nxfonts_fcache_s *priv = (FAR struct nxfonts_fcache_s *)fcache;
  FAR struct nxfonts_glyph_s *glyph;
  FAR const struct nx_fontbitmap_s *fbm;

  /* First, try to find the glyph in the cache of pre-rendered glyphs */

  glyph = nxf_findglyph(priv, ch);
  if (glyph != NULL)
    {
      /* We found it in the cache .. return the cached glyph */

      return glyph;
    }

  /* No, it is not cached... Does the code map to a font? */

  fbm = nxf_getbitmap(priv->font, ch);
  if (fbm)
    {
      /* Yes.. render the glyph */

      glyph = nxf_renderglyph(priv, fbm, ch);
    }

  return glyph;
}
