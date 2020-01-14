/****************************************************************************
 * boards/arm/cxd56xx/drivers/audio/cxd56_audio_volume.c
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

#include <nuttx/arch.h>
#include <nuttx/signal.h>

#include "cxd56_audio_config.h"
#include "cxd56_audio_ac_reg.h"
#include "cxd56_audio_analog.h"
#include "cxd56_audio_volume.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Number of volume device. */

#define VOLUME_NUM      3

/* Maximum value. */

#define VOLUME_MAX      120

/* Minimum value. */

#define VOLUME_MIN      -1020

/* Mute volume. */

#define VOLUME_MUTE     -1025

#define MUTE_BIT_API  0x01
#define MUTE_BIT_FADE 0x02
#define MUTE_VOL_REG  0x33
#define VOL_WAIT_TIME 20
#define VOL_TO_REG(vol) (((vol) / 5) & 0xff)
#define VOL_MUTE_TIME(vol, n_cycle) \
  (((VOL_TO_REG(vol) - MUTE_VOL_REG) & 0xff) * (n_cycle + 1) * 4 / 48 * 1000)

/****************************************************************************
 * Private Type
 ****************************************************************************/

struct set_vol_prm_s
{
  int16_t hold_vol;
  uint8_t mute_bit;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct set_vol_prm_s g_volparam[VOLUME_NUM] =
{
  {VOLUME_MUTE, MUTE_BIT_API},
  {VOLUME_MUTE, MUTE_BIT_API},
  {VOLUME_MUTE, MUTE_BIT_API}
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static CXD56_AUDIO_ECODE set_mute(cxd56_audio_volid_t id,
                                  bool wait,
                                  uint8_t type)
{
  uint32_t waittime = 0;

  waittime = VOL_MUTE_TIME(g_volparam[id].hold_vol, 1);

  if (g_volparam[id].mute_bit == 0)
    {
      switch (id)
        {
          case CXD56_AUDIO_VOLID_MIXER_IN1:
            cxd56_audio_ac_reg_set_vol_sdin1(MUTE_VOL_REG);
            break;

          case CXD56_AUDIO_VOLID_MIXER_IN2:
            cxd56_audio_ac_reg_set_vol_sdin2(MUTE_VOL_REG);
            break;

          default:
            cxd56_audio_ac_reg_set_vol_dac(MUTE_VOL_REG);
            break;
        }
    }

  if (wait)
    {
      nxsig_usleep(waittime);
    }

  g_volparam[id].mute_bit |= type;

  if (g_volparam[CXD56_AUDIO_VOLID_MIXER_OUT].mute_bit != 0)
    {
      CXD56_AUDIO_ECODE ret = cxd56_audio_analog_disable_output();
      if (ret != CXD56_AUDIO_ECODE_OK)
        {
          return ret;
        }
    }

  return CXD56_AUDIO_ECODE_OK;
}

static CXD56_AUDIO_ECODE set_unmute(cxd56_audio_volid_t id,
                                    bool wait,
                                    uint8_t type)
{
  uint32_t waittime = 0;

  g_volparam[id].mute_bit &= ~type;

  if (g_volparam[CXD56_AUDIO_VOLID_MIXER_OUT].mute_bit == 0)
    {
      CXD56_AUDIO_ECODE ret = cxd56_audio_analog_enable_output();
      if (ret != CXD56_AUDIO_ECODE_OK)
        {
          return ret;
        }
    }

  if (g_volparam[id].mute_bit == 0)
    {
      if (type == MUTE_BIT_API)
        {
          waittime = VOL_WAIT_TIME;
        }
      else
        {
          /* fade */

          waittime = VOL_MUTE_TIME(g_volparam[id].hold_vol, 1);
        }

      uint32_t vol = VOL_TO_REG(g_volparam[id].hold_vol);

      switch (id)
        {
          case CXD56_AUDIO_VOLID_MIXER_IN1:
            cxd56_audio_ac_reg_set_vol_sdin1(vol);
            break;

          case CXD56_AUDIO_VOLID_MIXER_IN2:
            cxd56_audio_ac_reg_set_vol_sdin2(vol);
            break;

          default:
            cxd56_audio_ac_reg_set_vol_dac(vol);
            break;
        }

      if (wait)
        {
          nxsig_usleep(waittime);
        }
    }

  return CXD56_AUDIO_ECODE_OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

CXD56_AUDIO_ECODE cxd56_audio_volume_set(cxd56_audio_volid_t id,
                                         int16_t vol)
{
  if (id >= VOLUME_NUM)
    {
      return CXD56_AUDIO_ECODE_VOL_ID;
    }

  if (VOLUME_MIN > vol)
    {
      if (VOLUME_MUTE != vol)
        {
          return CXD56_AUDIO_ECODE_VOL_MIN;
        }
    }

  if (VOLUME_MAX < vol)
    {
      return CXD56_AUDIO_ECODE_VOL_MAX;
    }

  g_volparam[id].hold_vol = vol;
  set_unmute(id, true, MUTE_BIT_API);

  return CXD56_AUDIO_ECODE_OK;
}

CXD56_AUDIO_ECODE cxd56_audio_volume_mute(cxd56_audio_volid_t id)
{
  if (id >= VOLUME_NUM)
    {
      return CXD56_AUDIO_ECODE_VOL_ID;
    }

  set_mute(id, true, MUTE_BIT_API);

  return CXD56_AUDIO_ECODE_OK;
}

CXD56_AUDIO_ECODE cxd56_audio_volume_unmute(cxd56_audio_volid_t id)
{
  if (id >= VOLUME_NUM)
    {
      return CXD56_AUDIO_ECODE_VOL_ID;
    }

  set_unmute(id, true, MUTE_BIT_API);

  return CXD56_AUDIO_ECODE_OK;
}

CXD56_AUDIO_ECODE cxd56_audio_volume_mute_fade(cxd56_audio_volid_t id,
                                               bool wait)
{
  if (id >= VOLUME_NUM)
    {
      return CXD56_AUDIO_ECODE_VOL_ID;
    }

  set_mute(id, wait, MUTE_BIT_FADE);

  return CXD56_AUDIO_ECODE_OK;
}

CXD56_AUDIO_ECODE cxd56_audio_volume_unmute_fade(cxd56_audio_volid_t id,
                                                 bool wait)
{
  if (id >= VOLUME_NUM)
    {
      return CXD56_AUDIO_ECODE_VOL_ID;
    }

  set_unmute(id, wait, MUTE_BIT_FADE);

  return CXD56_AUDIO_ECODE_OK;
}
