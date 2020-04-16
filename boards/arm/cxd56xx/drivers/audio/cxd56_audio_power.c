/****************************************************************************
 * boards/arm/cxd56xx/drivers/audio/cxd56_audio_power.c
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
#include <nuttx/irq.h>
#include <debug.h>

#include <arch/board/board.h>
#include <arch/chip/audio.h>

#include "cxd56_audio_config.h"
#include "cxd56_audio_power.h"
#include "cxd56_audio_digital.h"
#include "cxd56_audio_ac_reg.h"
#include "cxd56_audio_bca_reg.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static CXD56_AUDIO_ECODE power_on_codec(void)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;
  cxd56_audio_clkmode_t clk_mode = cxd56_audio_config_get_clkmode();

  ret = cxd56_audio_ac_reg_checkid();
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  cxd56_audio_ac_reg_initdsp();
  cxd56_audio_ac_reg_poweron_sdes();

  uint8_t mic_mode = cxd56_audio_config_get_micmode();
  ret = cxd56_audio_ac_reg_set_micmode(mic_mode);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  cxd56_audio_ac_reg_poweron_codec();
  cxd56_audio_digital_poweron();
  cxd56_audio_ac_reg_resetdsp();
  cxd56_audio_digital_enable();
  cxd56_audio_ac_reg_enable_serialif();
  cxd56_audio_ac_reg_init_selector();

  ret = cxd56_audio_ac_reg_set_alcspc();
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  cxd56_audio_bca_reg_set_datarate(clk_mode);

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

CXD56_AUDIO_ECODE cxd56_audio_power_on(void)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Power on audio codec block. */

  ret = power_on_codec();
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_power_off(void)
{
  /* Power off audio codec block. */

  cxd56_audio_ac_reg_poweroff_codec();

  return CXD56_AUDIO_ECODE_OK;
}
