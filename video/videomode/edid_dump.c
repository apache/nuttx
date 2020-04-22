/****************************************************************************
 * video/videomode/edid_dump.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derives from logic in FreeBSD which has an compatible 2-clause BSD
 * license:
 *
 *   Copyright (c) 2006 Itronix Inc. All rights reserved.
 *   Written by Garrett D'Amore for Itronix Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE NETBSD FOUNDATION BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <syslog.h>

#include <nuttx/video/videomode.h>
#include <nuttx/video/edid.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  edid_dump
 *
 * Description:
 *   Dump the full content of the EDID
 *
 * Input Parameters:
 *   edid - The edid to be dumped
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void edid_dump(FAR const struct edid_info_s *edid)
{
  FAR const char *prefix;
  int i;

  if (edid == NULL)
    {
      return;
    }

  syslog(LOG_INFO, "%-16s[%s]\n",
         "Manufacturer:", edid->edid_manufacturer);
  syslog(LOG_INFO, "%-16s%u.%u\n",
         "EDID Version:", edid->edid_version, edid->edid_revision);

  syslog(LOG_INFO, "%-16s%02x\n",
         "Video Input:", edid->edid_video_input);
  if ((edid->edid_video_input & EDID_DISPLAY_INPUT_DIGITAL) != 0)
    {
      if ((edid->edid_video_input & EDID_DISPLAY_INPUT_DFP1_COMPAT) != 0)
        {
          syslog(LOG_INFO, "%-16s%s\n",
                 "", "Digital (DFP 1.x compatible)\n");
        }
      else
        {
          syslog(LOG_INFO, "%-16s%s\n",
                 "", "Digital");
        }
    }
  else
    {
      switch (edid->edid_video_input & EDID_DISPLAY_INPUT_LEVELS_MASK)
        {
        case EDID_DISPLAY_INPUT_LEVEL_1:
          syslog(LOG_INFO, "%-16s-0.7, 0.3V\n", "", "Analog:");
          break;

        case EDID_DISPLAY_INPUT_LEVEL_2:
          syslog(LOG_INFO, "%-16s-0.714, 0.286V\n", "", "Analog:");
          break;

        case EDID_DISPLAY_INPUT_LEVEL_3:
          syslog(LOG_INFO, "%-16s-1.0, 0.4V\n", "", "Analog:");
          break;

        case EDID_DISPLAY_INPUT_LEVEL_4:
          syslog(LOG_INFO, "%-16s-0.7, 0.0V\n", "", "Analog:");
          break;
        }

      if ((edid->edid_video_input & EDID_DISPLAY_INPUT_BLANK2BLACK) != 0)
        {
          syslog(LOG_INFO, "%-16sBlank-to-black setup\n", "");
        }

      if ((edid->edid_video_input & EDID_DISPLAY_INPUT_SYNC) != 0)
        {
          syslog(LOG_INFO, " %-16sSeperate syncs\n", "");
        }

      if ((edid->edid_video_input & EDID_DISPLAY_INPUT_COMPOSITE) != 0)
        {
          syslog(LOG_INFO, "%-16sComposite sync\n", "");
        }

      if ((edid->edid_video_input & EDID_DISPLAY_INPUT_GREEN) != 0)
        {
          syslog(LOG_INFO, "%-16sSync on green\n", "");
        }

      if ((edid->edid_video_input & EDID_DISPLAY_INPUT_VSERRATED) != 0)
        {
          syslog(LOG_INFO, "%-16sSerrated vsync\n", "");
        }
    }

  syslog(LOG_INFO, "Max Size:      %d cm x %d cm\n",
         edid->edid_max_hsize, edid->edid_max_vsize);
  syslog(LOG_INFO, "Gamma:         %u.%02\n",
         edid->edid_gamma / 100, edid->edid_gamma % 100);

  syslog(LOG_INFO, "%-16s%02x\n",
         "Features:", edid->edid_features);
  if (edid->edid_features & EDID_DISPLAY_FEATURE_DPMSSTDBY)
    {
      syslog(LOG_INFO, "%-16sDPMS standby\n", "");
    }

  if (edid->edid_features & EDID_DISPLAY_FEATURE_DPMSSUSP)
    {
      syslog(LOG_INFO, "%-16sDPMS suspend\n", "");
    }

  if (edid->edid_features & EDID_DISPLAY_FEATURE_DPMSOFF)
    {
      syslog(LOG_INFO, "%-16sDPMS active-off\n", "");
    }

  switch (edid->edid_features & EDID_DISPLAY_FEATURE_DTYPE_MASK)
    {
    case EDID_ISPLAY_FEATURES_DTYPE_MONO:
      syslog(LOG_INFO, "%-16sMonochrome\n", "");
      break;

    case EDID_ISPLAY_FEATURES_DTYPE_RGB:
      syslog(LOG_INFO, "%-16sRGB\n", "");
      break;

    case EDID_ISPLAY_FEATURES_DTYPE_NON_RGB:
      syslog(LOG_INFO, "%-16sMulticolor\n", "");
      break;

    case EDID_ISPLAY_FEATURES_DTYPE_UNDEFINED:
      syslog(LOG_INFO, "%-16sUndefined monitor type\n", "");
      break;
    }

  if (edid->edid_features & EDID_DISPLAY_FEATURE_STDRGB)
    {
      syslog(LOG_INFO, "%-16sStandard color space\n", "");
    }

  if (edid->edid_features & EDID_DISPLAY_FEATURE_MODE)
    {
      syslog(LOG_INFO, "%-16sPreferred timing\n", "");
    }

  if (edid->edid_features & EDID_DISPLAY_FEATURE_CONTINUOUS)
    {
      syslog(LOG_INFO, "%-16sDefault GTF supported\n", "");
    }

  syslog(LOG_INFO, "%-16s%d\n",
         "Ext Blocks:", edid->edid_ext_block_count);
  syslog(LOG_INFO, "%-16s%u\n",
         "Product:", edid->edid_product);
  syslog(LOG_INFO, "%-16s%lu\n",
         "Serial No:", (unsigned long)edid->edid_serial);
  syslog(LOG_INFO, "%-16sYear %d, Week %d\n",
         "Manufactured:", edid->edid_year, edid->edid_week);

  if (edid->edid_have_range)
    {
      syslog(LOG_INFO, "%-16sHorizontal: %d - %d kHz\n",
             "Range:", edid->edid_range.er_min_hfreq,
              edid->edid_range.er_max_hfreq);
      syslog(LOG_INFO, "%-16sVertical: %d - %d Hz\n",
             "", edid->edid_range.er_min_vfreq,
             edid->edid_range.er_max_vfreq);
      syslog(LOG_INFO, "%-16sMax Dot Clock: %d MHz\n",
             "", edid->edid_range.er_max_clock);
      if (edid->edid_range.er_have_gtf2)
        {
          syslog(LOG_INFO, "%-16sGTF2 hfreq: %d\n",
                 "", edid->edid_range.er_gtf2_hfreq);
          syslog(LOG_INFO, "%-16sGTF2 C: %d\n",
                 "", edid->edid_range.er_gtf2_c);
          syslog(LOG_INFO, "%-16sGTF2 M: %d\n",
                 "", edid->edid_range.er_gtf2_m);
          syslog(LOG_INFO, "%-16sGTF2 J: %d\n",
                 "", edid->edid_range.er_gtf2_j);
          syslog(LOG_INFO, "%-16sGTF2 K: %d\n",
                 "", edid->edid_range.er_gtf2_k);
        }
    }

  syslog(LOG_INFO, "%-16sRed X: 0.%03d\n",
         "Chroma Info:", edid->edid_chroma.ec_redx);
  syslog(LOG_INFO, "%-16sRed Y: 0.%03d\n",
         "", edid->edid_chroma.ec_redy);
  syslog(LOG_INFO, "%-16sGrn X: 0.%03d\n",
         "", edid->edid_chroma.ec_greenx);
  syslog(LOG_INFO, "%-16sGrn Y: 0.%03d\n",
         "", edid->edid_chroma.ec_greeny);
  syslog(LOG_INFO, "%-16sBlu X: 0.%03d\n",
         "", edid->edid_chroma.ec_bluex);
  syslog(LOG_INFO, "%-16sBlu Y: 0.%03d\n",
         "", edid->edid_chroma.ec_bluey);
  syslog(LOG_INFO, "%-16sWht X: 0.%03d\n",
         "", edid->edid_chroma.ec_whitex);
  syslog(LOG_INFO, "%-16sWht Y: 0.%03d\n",
         "", edid->edid_chroma.ec_whitey);

  prefix = "Video modes:    ";
  for (i = 0; i < edid->edid_nmodes; i++)
    {
      videomode_dump(prefix, &edid->edid_modes[i], false);
      prefix = "                ";
    }

  if (edid->edid_preferred_mode)
    {
      prefix = "Preferred Mode: ";
      videomode_dump(prefix, edid->edid_preferred_mode, true);
    }
}
