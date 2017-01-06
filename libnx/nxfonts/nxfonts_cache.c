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
  uint8_t maxglyphs;                   /* Size of glyph[] array */
  uint8_t bpp;                         /* Bits per pixel */
  nxgl_mxpixel_t fgcolor;              /* Foreground color */
  nxgl_mxpixel_t bgcolor;              /* Background color */
  nxf_renderer_t renderer;             /* Font renderer */

  /* Glyph cache data storage */

  struct nxfonts_glyph_s glyph[CONFIG_NXTERM_CACHESIZE];
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
 * Name: nxf_freeglyph
 ****************************************************************************/

static void nxf_freeglyph(FAR struct nxfonts_glyph_s *glyph)
{
  if (glyph->bitmap)
    {
      lib_free(glyph->bitmap);
    }

  memset(glyph, 0, sizeof(struct nxfonts_glyph_s));
}

/****************************************************************************
 * Name: nxf_allocglyph
 ****************************************************************************/

static inline FAR struct nxfonts_glyph_s *
nxf_allocglyph(FAR struct nxfonts_fcache_s *priv)
{
  FAR struct nxfonts_glyph_s *glyph = NULL;
  FAR struct nxfonts_glyph_s *luglyph = NULL;
  uint8_t luusecnt;
  int i;

  /* Search through the glyph cache looking for an unused glyph.  Also, keep
   * track of the least used glyph as well.  We need that if we have to replace
   * a glyph in the cache.
   */

   for (i = 0; i < priv->maxglyphs; i++)
    {
      /* Is this glyph in use? */

      glyph = &priv->glyph[i];
      if (!glyph->usecnt)
        {
          /* No.. return this glyph with a use count of one */

          glyph->usecnt = 1;
          return glyph;
        }

      /* Yes.. check for the least recently used */

      if (!luglyph || glyph->usecnt < luglyph->usecnt)
        {
          luglyph = glyph;
        }
    }

  /* If we get here, the glyph cache is full.  We replace the least used
   * glyph with the one we need now. (luglyph can't be NULL).
   */

  luusecnt = luglyph->usecnt;
  nxf_freeglyph(luglyph);

  /* But lets decrement all of the usecnts so that the new one one be so
   * far behind in the counts as the older ones.
   */

  if (luusecnt > 1)
    {
       uint8_t decr = luusecnt - 1;

       for (i = 0; i < priv->maxglyphs; i++)
        {
          /* Is this glyph in use? */

          glyph = &priv->glyph[i];
          if (glyph->usecnt > decr)
            {
              glyph->usecnt -= decr;
            }
        }
    }

  /* Then return the least used glyph */

  luglyph->usecnt = 1;
  return luglyph;
}

/****************************************************************************
 * Name: nxf_findglyph
 ****************************************************************************/

static FAR struct nxfonts_glyph_s *
nxf_findglyph(FAR struct nxfonts_fcache_s *priv, uint8_t ch)
{
  int i;

  /* First, try to find the glyph in the cache of pre-rendered glyphs */

   for (i = 0; i < priv->maxglyphs; i++)
    {
      FAR struct nxfonts_glyph_s *glyph = &priv->glyph[i];
      if (glyph->usecnt > 0 && glyph->code == ch)
        {
          /* Increment the use count (unless it is already at the max) */

          if (glyph->usecnt < MAX_USECNT)
            {
               glyph->usecnt++;
            }

          /* And return the glyph that we found */

          return glyph;
        }
    }
  return NULL;
}

/****************************************************************************
 * Name: nxf_renderglyph
 ****************************************************************************/

static inline FAR struct nxfonts_glyph_s *
nxf_renderglyph(FAR struct nxfonts_fcache_s *priv,
                FAR const struct nx_fontbitmap_s *fbm, uint8_t ch)
{
  FAR struct nxfonts_glyph_s *glyph = NULL;
  FAR nxgl_mxpixel_t *ptr;
#if CONFIG_NXTERM_BPP < 8
  nxgl_mxpixel_t pixel;
#endif
  int bmsize;
  int row;
  int col;
  int ret;

  /* Allocate the glyph (always succeeds) */

  glyph         = nxf_allocglyph(priv);
  glyph->code   = ch;

  /* Get the dimensions of the glyph */

  glyph->width  = fbm->metric.width + fbm->metric.xoffset;
  glyph->height = fbm->metric.height + fbm->metric.yoffset;

  /* Get the physical width of the glyph in bytes */

  glyph->stride = (glyph->width * CONFIG_NXTERM_BPP + 7) / 8;

  /* Allocate memory to hold the glyph with its offsets */

  bmsize        =  glyph->stride * glyph->height;
  glyph->bitmap = (FAR uint8_t *)lib_malloc(bmsize);

  if (glyph->bitmap)
    {
      /* Initialize the glyph memory to the background color using the
       * hard-coded bits-per-pixel (BPP).
       *
       * TODO:  The rest of NX is configured to support multiple devices
       * with differing BPP.  They logic should be extended to support
       * differing BPP's as well.
       */

#if CONFIG_NXTERM_BPP < 8
      pixel  = priv->bgcolor;

#  if CONFIG_NXTERM_BPP == 1

      /* Pack 1-bit pixels into a 2-bits */

      pixel &= 0x01;
      pixel  = (pixel) << 1 | pixel;

#  endif
#  if CONFIG_NXTERM_BPP < 4

      /* Pack 2-bit pixels into a nibble */

      pixel &= 0x03;
      pixel  = (pixel) << 2 | pixel;

#  endif

      /* Pack 4-bit nibbles into a byte */

      pixel &= 0x0f;
      pixel  = (pixel) << 4 | pixel;

      ptr    = (FAR nxgl_mxpixel_t *)glyph->bitmap;
      for (row = 0; row < glyph->height; row++)
        {
          for (col = 0; col < glyph->stride; col++)
            {
              /* Transfer the packed bytes into the buffer */

              *ptr++ = pixel;
            }
        }

#elif CONFIG_NXTERM_BPP == 24
# error "Additional logic is needed here for 24bpp support"

#else /* CONFIG_NXTERM_BPP = {8,16,32} */

      ptr = (FAR nxgl_mxpixel_t *)glyph->bitmap;
      for (row = 0; row < glyph->height; row++)
        {
          /* Just copy the color value into the glyph memory */

          for (col = 0; col < glyph->width; col++)
            {
              *ptr++ = priv->bgcolor;
            }
        }
#endif

      /* Then render the glyph into the allocated memory */

      ret = priv->renderer((FAR nxgl_mxpixel_t *)glyph->bitmap,
                           glyph->height, glyph->width, glyph->stride,
                           fbm, priv->fgcolor);
      if (ret < 0)
        {
          /* Actually, the renderer never returns a failure */

          gerr("ERROR: nxf_renderglyph: Renderer failed\n");
          nxf_freeglyph(glyph);
          glyph = NULL;
        }
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
 *   fontid  - Identifies the font supported by this cache
 *   fgcolor - Foreground color
 *   bgcolor - Background color
 *   bpp     - Bits per pixel
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
                         int bpp)
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

      priv->maxglyphs = CONFIG_NXTERM_CACHESIZE;
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
  int ret;
  int i;

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

      /* Free all allocated glyph bitmap */

      for (i = 0; i < CONFIG_NXTERM_CACHESIZE; i++)
        {
          FAR struct nxfonts_glyph_s *glyph = &priv->glyph[i];
          if (glyph->bitmap)
            {
              lib_free(glyph->bitmap);
            }
        }

      /* Destroy the serializing semaphore... while we are holding it? */

      sem_destroy(&priv->fsem);

      /* Finally, free the font cache stucture itself */

      lib_free(fcache);
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

FAR struct nxfonts_glyph_s *nxf_cache_getglyph(FCACHE fcache, uint8_t ch)
{
  FAR struct nxfonts_fcache_s *priv = (FAR struct nxfonts_fcache_s *)fcache;
  FAR struct nxfonts_glyph_s *glyph;
  FAR const struct nx_fontbitmap_s *fbm;

  /* First, try to find the glyph in the cache of pre-rendered glyphs */

  glyph = nxf_findglyph(priv, ch);
  if (glyph)
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
