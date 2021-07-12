/****************************************************************************
 * video/videomode/edid_parse.c
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
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/video/videomode.h>
#include <nuttx/video/vesagtf.h>
#include <nuttx/video/edid.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DIVIDE(x,y)     (((x) + ((y) / 2)) / (y))

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* These are reversed established timing order */

static FAR const char *g_edid_modes[] =
{
  "1280x1024x75",
  "1024x768x75",
  "1024x768x70",
  "1024x768x60",
  "1024x768x87i",
  "832x624x74",                 /* Rounding error, 74.55 Hz aka "832x624x75" */
  "800x600x75",
  "800x600x72",
  "800x600x60",
  "800x600x56",
  "640x480x75",
  "640x480x72",
  "640x480x67",
  "640x480x60",
  "720x400x87",                 /* Rounding error, 87.85 Hz aka "720x400x88" */
  "720x400x70",
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  edid_valid
 *
 * Description:
 *   Return true if the EDID is valid
 *
 ****************************************************************************/

static bool edid_valid(FAR const uint8_t *data)
{
  static const uint8_t magic[8] = EDID_MAGIC;
  int sum = 0;
  int i;

  /* Verify the EDID magic number */

  if (memcmp(data, magic, 8) != 0)
    {
      return false;
    }

  /* Verify the EDID checksum */

  for (i = 0; i < 128; i++)
    {
      sum += data[i];
    }

  if ((sum & 0xff) != 0)
    {
      return false;
    }

  return true;
}

/****************************************************************************
 * Name:  edid_std_timing
 *
 * Description:
 *   Parse STD timing entry
 *
 ****************************************************************************/

static bool edid_std_timing(FAR const uint8_t *stdtim,
                            FAR struct videomode_s *mode)
{
  FAR const struct videomode_s *lookup;
  char name[80];
  unsigned x;
  unsigned y;
  unsigned f;

  if ((stdtim[0] == 1 && stdtim[1] == 1) ||
      (stdtim[0] == 0 && stdtim[1] == 0) ||
      (stdtim[0] == 0x20 && stdtim[1] == 0x20))
    {
      return false;
    }

  x = stdtim[EDID_STDTIMING_XRES_OFFSET];
  switch (x & EDID_STDTIMING_ASPECT_MASK)
    {
    case EDID_STDTIMING_ASPECT_16_10:
      y = x * 10 / 16;
      break;

    case EDID_STDTIMING_ASPECT_4_3:
      y = x * 3 / 4;
      break;

    case EDID_STDTIMING_ASPECT_5_4:
      y = x * 4 / 5;
      break;

    case EDID_STDTIMING_ASPECT_16_9:
    default:
      y = x * 9 / 16;
      break;
    }

  f = stdtim[EDID_STDTIMING_INFO_OFFSET];

  /* First try to lookup the mode as a DMT timing */

  snprintf(name, sizeof(name), "%dx%dx%d", x, y, f);
  if ((lookup = videomode_lookup_by_name(name)) != NULL)
    {
      *mode = *lookup;
    }
  else
    {
      /* Failing that, calculate it using gtf
       *
       * Hmm. I'm not using alternate GTF timings, which
       * could, in theory, be present.
       */

      vesagtf_mode(x, y, f, mode);
    }

  return true;
}

/****************************************************************************
 * Name:  edid_search_mode
 *
 * Description:
 *   Check if for duplicate video modes.
 *
 ****************************************************************************/

static struct videomode_s *
  edid_search_mode(FAR struct edid_info_s *edid,
                   FAR const struct videomode_s *mode)
{
  int refresh;
  int i;

  refresh = DIVIDE(DIVIDE(mode->dotclock * 1000, mode->htotal),
                   mode->vtotal);

  for (i = 0; i < edid->edid_nmodes; i++)
    {
      if (mode->hdisplay == edid->edid_modes[i].hdisplay &&
          mode->vdisplay == edid->edid_modes[i].vdisplay &&
          refresh == DIVIDE(DIVIDE(edid->edid_modes[i].dotclock * 1000,
                                   edid->edid_modes[i].htotal),
                            edid->edid_modes[i].vtotal))
        {
          return &edid->edid_modes[i];
        }
    }

  return NULL;
}

/****************************************************************************
 * Name:  edid_desc_timing
 *
 * Description:
 *
 ****************************************************************************/

static bool edid_desc_timing(FAR const uint8_t *desc,
                             FAR struct videomode_s *mode)
{
  uint16_t hactive;
  unsigned int hblank;
  unsigned int hsyncwid;
  unsigned int hsyncoff;
  unsigned int vactive;
  unsigned int vblank;
  unsigned int vsyncwid;
  unsigned int vsyncoff;
  uint8_t flags;

  flags = desc[EDID_DESC_FEATURES_OFFSET];

  /* We don't support stereo modes (for now) */

  if (flags & (EDID_DESC_STEREO_MASK | EDID_DESC_STEREO_INTERLEAVE))
    {
      return false;
    }

  mode->dotclock    =  (uint16_t)desc[EDID_DESC_PIXCLOCK_OFFSET] |
                      ((uint16_t)desc[EDID_DESC_PIXCLOCK_OFFSET + 1] << 8);

  hactive           = EDID_DESC_HACTIVE(desc);
  hblank            = EDID_DESC_HBLANK(desc);
  hsyncwid          = EDID_DESC_HSYNC_WIDTH(desc);
  hsyncoff          = EDID_DESC_HSYNC_OFFSET(desc);

  vactive           = EDID_DESC_VACTIVE(desc);
  vblank            = EDID_DESC_VBLANK(desc);
  vsyncwid          = EDID_DESC_VSYNC_WIDTH(desc);
  vsyncoff          = EDID_DESC_VSYNC_OFFSET(desc);

  /* Borders are contained within the blank areas. */

  mode->hdisplay    = hactive;
  mode->htotal      = hactive + hblank;
  mode->hsync_start = hactive + hsyncoff;
  mode->hsync_end   = mode->hsync_start + hsyncwid;

  mode->vdisplay    = vactive;
  mode->vtotal      = vactive + vblank;
  mode->vsync_start = vactive + vsyncoff;
  mode->vsync_end   = mode->vsync_start + vsyncwid;

  mode->hskew       = 0;
  mode->flags       = 0;
  mode->name        = NULL;

  if ((flags & EDID_DESC_INTERLACED) != 0)
    {
      mode->flags |= VID_INTERLACE;
    }

  if ((flags & EDID_DESC_DIGITAL_HPOLARITY) != 0)
    {
      mode->flags |= VID_PHSYNC;
    }
  else
    {
      mode->flags |= VID_NHSYNC;
    }

  if ((flags & EDID_DESC_DIGITAL_VSERRATION) != 0)
    {
      mode->flags |= VID_PVSYNC;
    }
  else
    {
      mode->flags |= VID_NVSYNC;
    }

  return true;
}

/****************************************************************************
 * Name:  edid_block
 *
 * Description:
 *   Parse an EDID descriptor block.
 *
 ****************************************************************************/

static void edid_block(FAR struct edid_info_s *edid, FAR const uint8_t *desc)
{
  struct videomode_s mode;
  FAR struct videomode_s *exist_mode;
  uint16_t pixclk;
  int i;

  /* A detailed timing descriptor with have a nonzero pixel clock */

  pixclk = ((uint16_t)desc[EDID_DESC_PIXCLOCK_OFFSET] << 8) |
            (uint16_t)desc[EDID_DESC_PIXCLOCK_OFFSET + 1];

  if (pixclk > 0)
    {
      if (!edid_desc_timing(desc, &mode))
        {
          return;
        }

      /* Does this mode already exist? */

      exist_mode = edid_search_mode(edid, &mode);
      if (exist_mode != NULL)
        {
          *exist_mode = mode;
          if (edid->edid_preferred_mode == NULL)
            {
              edid->edid_preferred_mode = exist_mode;
            }
        }
      else
        {
          edid->edid_modes[edid->edid_nmodes] = mode;
          if (edid->edid_preferred_mode == NULL)
            {
              edid->edid_preferred_mode =
                          &edid->edid_modes[edid->edid_nmodes];
            }

          edid->edid_nmodes++;
        }

      return;
    }

  /* Not a detailed timing descriptor */

  switch (desc[EDID_DESC_DESCTYPE])
    {
    case EDID_DESCTYPE_SERIALNO:
#if 0 /* Not implemented */
      memcpy(edid->edid_serstr, desc + EDID_DESC_ASCII_DATA_OFFSET,
             EDID_DESC_ASCII_DATA_LEN);
      edid->edid_serstr[sizeof(edid->edid_serial) - 1] = 0;
#endif
      break;

    case EDID_DESCTYPE_TEXT:
#if 0 /* Not implemented */
      memcpy(edid->edid_comment, desc + EDID_DESC_ASCII_DATA_OFFSET,
             EDID_DESC_ASCII_DATA_LEN);
      edid->edid_comment[sizeof(edid->edid_comment) - 1] = 0;
#endif
      break;

    case EDID_DESCTYPE_LIMITS:
      edid->edid_have_range = true;
      edid->edid_range.er_min_vfreq = EDID_DESC_RANGE_MIN_VFREQ(desc);
      edid->edid_range.er_max_vfreq = EDID_DESC_RANGE_MAX_VFREQ(desc);
      edid->edid_range.er_min_hfreq = EDID_DESC_RANGE_MIN_HFREQ(desc);
      edid->edid_range.er_max_hfreq = EDID_DESC_RANGE_MAX_HFREQ(desc);
      edid->edid_range.er_max_clock = EDID_DESC_RANGE_MAX_CLOCK(desc);

      if (!EDID_DESC_RANGE_HAVE_GTF2(desc))
        {
          break;
        }

      edid->edid_range.er_have_gtf2 = true;
      edid->edid_range.er_gtf2_hfreq = EDID_DESC_RANGE_GTF2_HFREQ(desc);
      edid->edid_range.er_gtf2_c = EDID_DESC_RANGE_GTF2_C(desc);
      edid->edid_range.er_gtf2_m = EDID_DESC_RANGE_GTF2_M(desc);
      edid->edid_range.er_gtf2_j = EDID_DESC_RANGE_GTF2_J(desc);
      edid->edid_range.er_gtf2_k = EDID_DESC_RANGE_GTF2_K(desc);
      break;

    case EDID_DESCTYPE_NAME:
#if 0 /* Not implemented */
      /* Copy the product name into place */

      memcpy(edid->edid_productname,
             desc + EDID_DESC_ASCII_DATA_OFFSET, EDID_DESC_ASCII_DATA_LEN);
#endif
      break;

    case EDID_DESCTYPE_STDTIMING_ID:
      desc += EDID_DESC_STD_TIMING_START_OFFSET;
      for (i = 0; i < EDID_DESC_STD_TIMING_COUNT_OFFSET; i++)
        {
          if (edid_std_timing(desc, &mode))
            {
              /* Does this mode already exist? */

              exist_mode = edid_search_mode(edid, &mode);
              if (exist_mode == NULL)
                {
                  edid->edid_modes[edid->edid_nmodes] = mode;
                  edid->edid_nmodes++;
                }
            }

          desc += 2;
        }
      break;

    case EDID_DESCTYPE_WHITEPOINT:

      /* Not implemented yet */

      break;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  edid_parse
 *
 * Description:
 *   Given a block of raw EDID data, parse the data and convert it to the
 *   'digested' form of struct edid_info_s.
 *
 * Input Parameters:
 *   data - A reference to the raw EDID data
 *   edid - The location to return the digested EDID data.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; otherwise a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int edid_parse(FAR const uint8_t *data, FAR struct edid_info_s *edid)
{
  FAR const struct videomode_s *mode;
  uint16_t manufacturer;
  uint16_t estmodes;
  uint8_t gamma;
  int i;
  int max_dotclock = 0;
  int mhz;

  if (!edid_valid(&data[EDID_HEADER_MAGIC_OFFSET]))
    {
      return -EINVAL;
    }

  /* Get product identification */

  manufacturer =
            (uint16_t)data[EDID_VENDOR_MANUFACTURER_OFFSET] |
            ((uint16_t)data[EDID_VENDOR_MANUFACTURER_OFFSET + 1] << 8);

  edid->edid_manufacturer[0]  = EDID_VENDOR_MANUFACTURER_1(manufacturer);
  edid->edid_manufacturer[1]  = EDID_VENDOR_MANUFACTURER_2(manufacturer);
  edid->edid_manufacturer[2]  = EDID_VENDOR_MANUFACTURER_3(manufacturer);
  edid->edid_manufacturer[3]  = 0;     /* NUL terminate for convenience */

  edid->edid_product =
            (uint16_t)data[EDID_VENDOR_PRODUCTCODE_OFFSET] |
            ((uint16_t)data[EDID_VENDOR_PRODUCTCODE_OFFSET + 1] << 8);

  edid->edid_serial =
            ((uint32_t)data[EDID_VENDOR_SERIALNO_OFFSET] << 24) |
            ((uint32_t)data[EDID_VENDOR_SERIALNO_OFFSET + 1] << 16) |
            ((uint32_t)data[EDID_VENDOR_SERIALNO_OFFSET + 2] << 8) |
            (uint32_t)data[EDID_VENDOR_SERIALNO_OFFSET + 3];

  edid->edid_week             = data[EDID_VENDOR_WEEK_OFFSET];
  edid->edid_year             = data[EDID_VENDOR_YEAR_OFFSET] + 1990;

  /* Get EDID revision */

  edid->edid_version          = data[EDID_VERSION_MAJOR_OFFSET];
  edid->edid_revision         = data[EDID_VERSION_MINOR_OFFSET];

  edid->edid_video_input      = data[EDID_DISPLAY_INPUT_OFFSET];
  edid->edid_max_hsize        = data[EDID_DISPLAY_HSIZE_OFFSET];
  edid->edid_max_vsize        = data[EDID_DISPLAY_VSIZE_OFFSET];

  gamma                       = data[EDID_DISPLAY_GAMMA_OFFSET];
  edid->edid_gamma            = gamma == 0xff ? 100 : gamma + 100;
  edid->edid_features         = data[EDID_DISPLAY_FEATURES_OFFSET];

  edid->edid_chroma.ec_redx   = EDID_CHROMA_RED_X(data);
  edid->edid_chroma.ec_redy   = EDID_CHROMA_RED_X(data);
  edid->edid_chroma.ec_greenx = EDID_CHROMA_GREEN_X(data);
  edid->edid_chroma.ec_greeny = EDID_CHROMA_GREEN_Y(data);
  edid->edid_chroma.ec_bluex  = EDID_CHROMA_BLUE_X(data);
  edid->edid_chroma.ec_bluey  = EDID_CHROMA_BLUE_Y(data);
  edid->edid_chroma.ec_whitex = EDID_CHROMA_WHITE_X(data);
  edid->edid_chroma.ec_whitey = EDID_CHROMA_WHITE_Y(data);

  edid->edid_ext_block_count  = data[EDID_TRAILER_NEXTENSIONS_OFFSET];

  /* Lookup established modes */

  edid->edid_nmodes           = 0;
  edid->edid_preferred_mode   = NULL;
  estmodes                    = ((uint16_t)data[EDID_TIMING_OFFSET_1] << 8) |
                                 (uint16_t)data[EDID_TIMING_OFFSET_2];

  /* Iterate in established timing order */

  for (i = 15; i >= 0; i--)
    {
      if (estmodes & (1 << i))
        {
          mode = videomode_lookup_by_name(g_edid_modes[i]);
          if (mode != NULL)
            {
              edid->edid_modes[edid->edid_nmodes] = *mode;
              edid->edid_nmodes++;
            }
          else
            {
              lcdwarn("WARNING: No data for est. mode %s\n",
                      g_edid_modes[i]);
            }
        }
    }

  /* Do standard timing section */

  for (i = 0; i < EDID_STDTIMING_NUMBER; i++)
    {
      struct videomode_s stdmode;
      FAR struct videomode_s *exist_mode;

      if (edid_std_timing(data + EDID_STDTIMING_OFFSET + i * 2, &stdmode))
        {
          /* Does this mode already exist? */

          exist_mode = edid_search_mode(edid, &stdmode);
          if (exist_mode == NULL)
            {
              edid->edid_modes[edid->edid_nmodes] = stdmode;
              edid->edid_nmodes++;
            }
        }
    }

  /* Do detailed timings and descriptors */

  for (i = 0; i < EDID_DESCRIPTOR_NUMBER; i++)
    {
      edid_block(edid,
                 data + EDID_DESCRIPTOR_OFFSET + i * EDID_DESCRIPTOR_SIZE);
    }

  /* Some monitors lie about their maximum supported dot clock
   * by claiming to support modes which need a higher dot clock
   * than the stated maximum.
   *
   * For sanity's sake we bump it to the highest dot clock we find
   * in the list of supported modes
   */

  for (i = 0; i < edid->edid_nmodes; i++)
    {
      if (edid->edid_modes[i].dotclock > max_dotclock)
        {
          max_dotclock = edid->edid_modes[i].dotclock;
        }
    }

  lcdinfo("max_dotclock according to supported modes: %d\n",
          max_dotclock);

  mhz = (max_dotclock + 999) / 1000;

  if (edid->edid_have_range)
    {
      if (mhz > edid->edid_range.er_max_clock)
        {
          edid->edid_range.er_max_clock = mhz;
        }
    }
  else
    {
      edid->edid_range.er_max_clock = mhz;
    }

  return OK;
}
