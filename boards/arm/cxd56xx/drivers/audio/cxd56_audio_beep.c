/****************************************************************************
 * boards/arm/cxd56xx/drivers/audio/cxd56_audio_beep.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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

#include "cxd56_audio_config.h"
#include "cxd56_audio_ac_reg.h"
#include "cxd56_audio_beep.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Maximum frequency. */

#define FREQ_MAX        4085

/* Minimum frequency. */

#define FREQ_MIN        94

/* Maximum volume. */

#define VOL_MAX         0

/* Minimum volume. */

#define VOL_MIN         -90

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_beep = false;

static const uint16_t g_beepfreqtable[] =
{
  120,  127,  134,  142,  151,  160,  169,  180,  190,  201,  214,  226,
  240,  254,  269,  285,  302,  320,  339,  360,  381,  403,  428,  453,
  480,  509,  539,  571,  606,  642,  681,  719,  762,  810,  857,  910,
  965,  1021, 1079, 1143, 1215, 1289, 1362, 1444, 1536, 1627, 1714, 1829,
  1939, 2043, 2182, 2313, 2400, 2560, 2704, 2866, 3048, 3200, 3429, 3623,
  3840, 4085,   94
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

uint32_t convert_freq(uint32_t freq)
{
  uint32_t prev;
  uint32_t i;

  for (i = 0; i < sizeof(g_beepfreqtable) / sizeof(uint16_t); i++)
    {
      prev = (i + 62) % 63;
      if (freq < g_beepfreqtable[i])
        {
          if (prev == 62)
            {
              break;
            }
          else if(g_beepfreqtable[prev] <= freq)
            {
              break;
            }
        }
    }

  return prev;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

CXD56_AUDIO_ECODE cxd56_audio_beep_set_freq(uint16_t freq)
{
  uint32_t conv_freq = 0;

  if (freq > FREQ_MAX)
    {
      return CXD56_AUDIO_ECODE_BEP_FREQ_MAX;
    }

  if (freq < FREQ_MIN)
    {
      return CXD56_AUDIO_ECODE_BEP_FREQ_MIN;
    }

  /* Mute beep. */

  cxd56_audio_ac_reg_disable_beep();

  /* Convert Frequency. */

  conv_freq = convert_freq(freq);

  /* Set beep parameter. */

  cxd56_audio_ac_reg_set_beep_freq(conv_freq);

  if (g_beep)
    {
      cxd56_audio_ac_reg_enable_beep();
    }

  return CXD56_AUDIO_ECODE_OK;
}

CXD56_AUDIO_ECODE cxd56_audio_beep_set_vol(int16_t vol)
{
  uint32_t conv_vol  = 0;

  if (vol > VOL_MAX)
    {
      return CXD56_AUDIO_ECODE_BEP_VOL_MAX;
    }

  if (vol < VOL_MIN)
    {
      return CXD56_AUDIO_ECODE_BEP_VOL_MIN;
    }

  /* Mute off. */

  cxd56_audio_ac_reg_disable_beep();

  /* Convert Volume. */

  if (vol != 0)
    {
      conv_vol = -vol / 3;
    }
  else
    {
      conv_vol = 0;
    }

  /* Set beep parameter. */

  cxd56_audio_ac_reg_set_beep_vol(conv_vol);

  if (g_beep)
    {
      cxd56_audio_ac_reg_enable_beep();
    }

  return CXD56_AUDIO_ECODE_OK;
}

void cxd56_audio_beep_play(void)
{
  g_beep = true;

  cxd56_audio_ac_reg_enable_beep();
}

void cxd56_audio_beep_stop(void)
{
  g_beep = false;

  cxd56_audio_ac_reg_disable_beep();
}
