/****************************************************************************
 * arch/arm/include/stm32/ltdc.h
 *
 *   Copyright (C) 2014-2015 Marco Krahl. All rights reserved.
 *   Author: Marco Krahl <ocram.lhark@gmail.com>
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

#ifndef __ARCH_ARM_INCLUDE_STM32F7_LTDC_H
#define __ARCH_ARM_INCLUDE_STM32F7_LTDC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>
#include <nuttx/video/fb.h>

#ifdef CONFIG_STM32F7_LTDC
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct dma2d_layer_s; /* see arch/chip/dma2d.h */

/* Blend mode definitions */

enum ltdc_blend_e
{
  LTDC_BLEND_NONE           = 0,   /* Disable all blend operation */
  LTDC_BLEND_ALPHA          = 0x1, /* Enable alpha blending */
  LTDC_BLEND_PIXELALPHA     = 0x2, /* Enable alpha blending from pixel color */
  LTDC_BLEND_COLORKEY       = 0x4, /* Enable colorkey */
  LTDC_BLEND_ALPHAINV       = 0x8, /* Inverse alpha blending of source */
  LTDC_BLEND_PIXELALPHAINV  = 0x10 /* Invers pixel alpha blending of source */
};

/* layer control definitions */

enum ltdc_layer_e
{
  LTDC_LAYER_OWN            = 0,   /* The given layer */
  LTDC_LAYER_TOP            = 0x1, /* The initialized top layer */
  LTDC_LAYER_BOTTOM         = 0x2, /* the initialized bottom layer */
  LTDC_LAYER_ACTIVE         = 0x4, /* The current visible flip layer */
  LTDC_LAYER_INACTIVE       = 0x8  /* The current invisible flip layer */
#ifdef CONFIG_STM32F7_DMA2D
 ,LTDC_LAYER_DMA2D          = 0x10  /* The dma2d interface layer id */
#endif
};

/* Update operation flag */

enum ltdc_update_e
{
  LTDC_UPDATE_NONE          = 0,   /* Update given layer only */
  LTDC_UPDATE_SIM           = 0x1, /* Update both layer simultaneous */
  LTDC_UPDATE_FLIP          = 0x2, /* Perform flip operation */
  LTDC_UPDATE_ACTIVATE      = 0x4  /* Set the given layer to the active layer */
};

/* sync mode definitions */

enum ltdc_sync_e
{
  LTDC_SYNC_NONE            = 0,     /* Immediately */
  LTDC_SYNC_VBLANK          = 0x100, /* Upon vertical sync */
  LTDC_SYNC_WAIT            = 0x200  /* Waits upon vertical sync */
};

/* Definition of the visible layer position and size */

struct ltdc_area_s
{
  fb_coord_t xpos; /* X position in pixel */
  fb_coord_t ypos; /* Y position in pixel */
  fb_coord_t xres; /* X resolution in pixel */
  fb_coord_t yres; /* Y resolution in pixel */
};

/* The layer is controlled through the following structure */

struct ltdc_layer_s
{
  /* Name: getvideoinfo
   *
   * Description:
   *   Get video information about the layer
   *
   * Parameter:
   *   layer  - Reference to the layer control structure
   *   vinfo  - Reference to the video info structure
   *
   * Return:
   *   On success - OK
   *   On error   - -EINVAL
   */

  int (*getvideoinfo)(FAR struct ltdc_layer_s *layer,
                    FAR struct fb_videoinfo_s *vinfo);

  /* Name: getplaneinfo
   *
   * Description:
   *   Get plane information about the layer
   *
   * Parameter:
   *   layer   - Reference to the layer control structure
   *   planeno - Number of the plane
   *   pinfo   - Reference to the plane info structure
   *
   * Return:
   *   On success - OK
   *   On error   - -EINVAL
   */

  int (*getplaneinfo)(FAR struct ltdc_layer_s *layer, int planeno,
                      FAR struct fb_planeinfo_s *pinfo);

  /* Name: getlid
   *
   * Description:
   *   Get a specific layer identifier.
   *
   * Parameter:
   *   layer - Reference to the layer structure
   *   lid   - Reference to store the layer id
   *   flag  - Operation flag describe the layer identifier
   *           e.g. get the current active or inactive layer.
   *           See LTDC_LAYER_* for possible values
   *
   * Return:
   *   On success - OK
   *   On error   - -EINVAL
   */

  int (*getlid)(FAR struct ltdc_layer_s *layer, int *lid, uint32_t flag);

#ifdef CONFIG_FB_CMAP
  /* Name: setclut
   *
   * Description:
   *   Configure layer clut (color lookup table).
   *   Non clut is defined during initializing.
   *   Clut is active during next vertical blank period. Do not need an update.
   *
   * Parameter:
   *   layer  - Reference to the layer structure
   *   cmap   - color lookup table with up the 256 entries
   *   enable - Enable or disable clut support (if false cmap is ignored and can
   *            be NULL)
   *
   * Return:
   *   On success - OK
   *   On error   - -EINVAL
   */

  int (*setclut)(FAR struct ltdc_layer_s *layer,
                    const FAR struct fb_cmap_s *cmap);

  /* Name: getclut
   *
   * Description:
   *   Get configured layer clut (color lookup table).
   *
   * Parameter:
   *   layer - Reference to the layer structure
   *   cmap  - Reference to valid color lookup table accept up the 256 color
   *           entries
   *
   * Return:
   *   On success - OK
   *   On error   - -EINVAL
   */

  int (*getclut)(FAR struct ltdc_layer_s *layer, FAR struct fb_cmap_s *cmap);
#endif

  /* Name: setcolor
   *
   * Description:
   *   Configure layer color for the non active layer area.
   *   Default value during initializing: 0x00000000
   *   Color is active after next update.
   *
   * Parameter:
   *   layer - Reference to the layer structure
   *   argb  - ARGB8888 color value
   *
   * Return:
   *   On success - OK
   *   On error   - -EINVAL
   */

  int (*setcolor)(FAR struct ltdc_layer_s *layer, uint32_t argb);

  /* Name: getcolor
   *
   * Description:
   *   Get configured layer color for the non active layer area.
   *
   * Parameter:
   *   layer - Reference to the layer structure
   *   argb  - Reference to store the ARGB8888 color value
   *
   * Return:
   *   On success - OK
   *   On error   - -EINVAL
   */

  int (*getcolor)(FAR struct ltdc_layer_s *layer, uint32_t *argb);

  /* Name: setcolorkey
   *
   * Description:
   *   Configure the layer color key (chromakey) for transparence.
   *   Default value during initializing: 0x00000000
   *   Colorkey is active after next update.
   *
   * Parameter:
   *   layer  - Reference to the layer structure
   *   rgb    - RGB888 color key
   *
   * Return:
   *   On success - OK
   *   On error   - -EINVAL
   */

  int (*setcolorkey)(FAR struct ltdc_layer_s *layer, uint32_t rgb);

  /* Name: getcolorkey
   *
   * Description:
   *   Get the configured layer color key (chromakey) for transparence.
   *
   * Parameter:
   *   layer  - Reference to the layer structure
   *   rgb    - Reference to store the RGB888 color key
   *
   * Return:
   *   On success - OK
   *   On error   - -EINVAL
   */

  int (*getcolorkey)(FAR struct ltdc_layer_s *layer, uint32_t *rgb);

  /* Name: setalpha
   *
   * Description:
   *   Configure layer alpha value factor into blend operation.
   *   During the layer blend operation the source alpha value is multiplied
   *   with this alpha value. If the source color format doesn't support alpha
   *   channel (e.g. non ARGB8888) this alpha value will be used as constant
   *   alpha value for blend operation.
   *   Default value during initializing: 0xff
   *   Alpha is active after next update.
   *
   * Parameter:
   *   layer - Reference to the layer structure
   *   alpha - Alpha value
   *
   * Return:
   *   On success - OK
   *   On error - -EINVAL
   */

  int (*setalpha)(FAR struct ltdc_layer_s *layer, uint8_t alpha);

  /* Name: getalpha
   *
   * Description:
   *   Get configured layer alpha value factor for blend operation.
   *
   * Parameter:
   *   layer - Reference to the layer structure
   *   alpha - Reference to store the alpha value
   *
   * Return:
   *   On success - OK
   *   On error - -EINVAL
   */

  int (*getalpha)(FAR struct ltdc_layer_s *layer, uint8_t *alpha);

  /* Name: setblendmode
   *
   * Description:
   *   Configure blend mode of the layer.
   *   Default mode during initializing: LTDC_BLEND_NONE
   *   Blendmode is active after next update.
   *
   * Parameter:
   *   layer - Reference to the layer structure
   *   mode  - Blend mode (see LTDC_BLEND_*)
   *
   * Return:
   *   On success - OK
   *   On error - -EINVAL
   *
   * Procedure information:
   *   LTDC_BLEND_NONE:
   *     Informs the driver to disable all blend operation for the given layer.
   *     That means the layer is opaque. Note this has no effect on the
   *     colorkey settings.
   *
   *   LTDC_BLEND_ALPHA:
   *     Informs the driver to enable alpha blending for the given layer.
   *
   *   LTDC_BLEND_COLORKEY:
   *     Informs the driver to enable colorkeying for the given layer.
   *
   *   LTDC_BLEND_SRCPIXELALPHA:
   *     Informs the driver to use the pixel alpha value of the layer instead
   *     the constant alpha value. This is only useful for ARGB8888
   *     color format.
   *
   *   LTDC_BLEND_DESTPIXELALPHA:
   *     Informs the driver to use the pixel alpha value of the subjacent layer
   *     instead the constant alpha value. This is only useful for ARGB8888
   *     color format.
   */

  int (*setblendmode)(FAR struct ltdc_layer_s *layer, uint32_t mode);

  /* Name: getblendmode
   *
   * Description:
   *   Get configured blend mode of the layer.
   *
   * Parameter:
   *   layer - Reference to the layer structure
   *   mode  - Reference to store the blend mode
   *
   * Return:
   *   On success - OK
   *   On error - -EINVAL
   */

  int (*getblendmode)(FAR struct ltdc_layer_s *layer, uint32_t *mode);

  /* Name: setarea
   *
   * Description:
   *    Configure visible layer area and the reference position of the first
   *    pixel of the whole layer which is the first visible top left pixel in
   *    the active area.
   *    Default value during initializing:
   *      xpos = 0
   *      ypos = 0
   *      xres = display x resolution
   *      yres = display y resolution
   *
   *   Area is active after next update.
   *
   * Parameter:
   *   layer   - Reference to the layer control structure
   *   area    - Reference to the valid area structure for the new active area
   *   srcxpos - x position of the visible pixel of the whole layer
   *   srcypos - y position of the visible pixel of the whole layer
   *
   * Return:
   *   On success - OK
   *   On error - -EINVAL
   *
   * Procedure Information:
   *   If the srcxpos and srcypos unequal the xpos and ypos of the coord
   *   structure this acts like moving the visible area to another position on
   *   the screen during the next update operation.
   */

  int (*setarea)(FAR struct ltdc_layer_s *layer,
                  FAR const struct ltdc_area_s *area,
                  fb_coord_t srcxpos,
                  fb_coord_t srcypos);

  /* Name: getarea
   *
   * Description:
   *    Get configured visible layer area.
   *
   * Parameter:
   *   layer   - Reference to the layer control structure
   *   area    - Reference to the area structure to store the active area
   *   srcxpos - Reference to store the referenced x position of the whole layer
   *   srcypos - Reference to store the reterenced y position of the whole layer
   *
   * Return:
   *   On success - OK
   *   On error - -EINVAL
   */

  int (*getarea)(FAR struct ltdc_layer_s *layer,
                  FAR struct ltdc_area_s *area,
                  fb_coord_t *srcxpos,
                  fb_coord_t *srcypos);

  /* Name: update
   *
   * Description:
   *   Update current layer settings and make changes visible.
   *
   * Parameter:
   *   layer   - Reference to the layer structure
   *   mode    - operation mode (see LTDC_UPDATE_*)
   *
   * Return:
   *    OK        - On success
   *   -EINVAL    - If one of the parameter invalid
   *   -ECANCELED - Operation cancelled, something goes wrong
   *
   * Procedure information:
   *   LTDC_UPDATE_SIM:
   *     Informs the driver to update both ltdc layers simultaneously. Otherwise
   *     update the given layer only.
   *
   *   LTDC_UPDATE_FLIP:
   *     Informs the driver to perform a flip operation.
   *     This only effects the ltdc layer 1 and 2 and can be useful for double
   *     buffering. Each flip operation changed the active layer ot the inactive
   *     and vice versa. In the context of the ltdc that means, the inactive layer
   *     is complete disabled. So the subjacent layer is the background layer
   *     (background color). To reactivate both layer and their settings perform
   *     an update without LTDC_UPDATE_FLIP flag.
   *
   *   LTDC_UPDATE_ACTIVATE:
   *     Informs the driver that the given layer should be the active layer when
   *     the operation is complete.
   *
   *   LTDC_SYNC_VBLANK:
   *     Informs the driver to update the layer upon vertical blank. Otherwise
   *     immediately.
   */

  int (*update)(FAR struct ltdc_layer_s *layer, uint32_t mode);

#ifdef CONFIG_STM32F7_DMA2D
  /* Name: blit
   *
   * Description:
   *   Copy selected area from a source layer to selected position of the
   *   destination layer.
   *
   * Parameter:
   *   dest     - Reference to the destination layer
   *   destxpos - Selected x position of the destination layer
   *   destypos - Selected y position of the destination layer
   *   src      - Reference to the source layer
   *   srcarea  - Reference to the selected area of the source layer
   *
   * Return:
   *    OK      - On success
   *   -EINVAL  - If one of the parameter invalid or if the size of the selected
   *              source area outside the visible area of the destination layer.
   *              (The visible area usually represents the display size)
   */

   int (*blit)(FAR struct ltdc_layer_s *dest,
                fb_coord_t destxpos, fb_coord_t destypos,
                FAR const struct dma2d_layer_s *src,
                FAR const struct ltdc_area_s *srcarea);

  /* Name: blend
   *
   * Description:
   *   Blends the selected area from a foreground layer with selected position
   *   of the background layer. Copy the result to the destination layer. Note!
   *   The content of the foreground and background layer is not changed.
   *
   * Parameter:
   *   dest     - Reference to the destination layer
   *   destxpos - Selected x position of the destination layer
   *   destypos - Selected y position of the destination layer
   *   fore     - Reference to the foreground layer
   *   forexpos - Selected x position of the foreground layer
   *   foreypos - Selected y position of the foreground layer
   *   back     - Reference to the background layer
   *   backarea - Reference to the selected area of the background layer
   *
   * Return:
   *    OK      - On success
   *   -EINVAL  - If one of the parameter invalid or if the size of the selected
   *              source area outside the visible area of the destination layer.
   *              (The visible area usually represents the display size)
   */

   int (*blend)(FAR struct ltdc_layer_s *dest,
                fb_coord_t destxpos, fb_coord_t destypos,
                FAR const struct dma2d_layer_s *fore,
                fb_coord_t forexpos, fb_coord_t foreypos,
                FAR const struct dma2d_layer_s *back,
                FAR const struct ltdc_area_s *backarea);

  /* Name: fillarea
   *
   * Description:
   *   Fill the selected area of the whole layer with a specific color.
   *
   * Parameter:
   *   layer    - Reference to the layer structure
   *   area     - Reference to the valid area structure select the area
   *   color    - Color to fill the selected area. Color must be formatted
   *              according to the layer pixel format.
   *
   * Return:
   *    OK      - On success
   *   -EINVAL  - If one of the parameter invalid or if the size of the selected
   *              area outside the visible area of the layer.
   */

   int (*fillarea)(FAR struct ltdc_layer_s *layer,
                    FAR const struct ltdc_area_s *area,
                    uint32_t color);
#endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_ltdcgetlayer
 *
 * Description:
 *   Get the ltdc layer structure to perform hardware layer operation
 *
 * Parameter:
 *   lid - Layer identifier
 *
 * Return:
 *   Reference to the layer control structure on success or Null if parameter
 *   invalid.
 *
 ****************************************************************************/

FAR struct ltdc_layer_s *up_ltdcgetlayer(int lid);

#endif /* CONFIG_STM32F7_LTDC */
#endif /* __ARCH_ARM_INCLUDE_STM32F7_LTDC_H */

