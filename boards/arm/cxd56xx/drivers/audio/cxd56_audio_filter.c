/****************************************************************************
 * boards/arm/cxd56xx/drivers/audio/cxd56_audio_filter.c
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
#include "cxd56_audio_filter.h"
#include "cxd56_audio_ac_reg.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

CXD56_AUDIO_ECODE cxd56_audio_filter_set_cstereo(bool en,
                                                 bool sign_inv,
                                                 int16_t vol)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;
  if (en)
    {
      ret = cxd56_audio_ac_reg_enable_cstereo(sign_inv, vol);
    }
  else
    {
      cxd56_audio_ac_reg_disable_cstereo();
    }

  return ret;
}

void cxd56_audio_filter_poweron_dnc(void)
{
  cxd56_audio_ac_reg_poweron_dnc();
}

void cxd56_audio_filter_poweroff_dnc(void)
{
  cxd56_audio_ac_reg_poweroff_dnc();
}

void cxd56_audio_filter_set_dnc(cxd56_audio_dnc_id_t id,
                                bool en,
                                FAR cxd56_audio_dnc_bin_t *bin)
{
  /* Desable DNC. */

  cxd56_audio_ac_reg_disable_dnc(id);

  /* Set binary data to SRAM. */

  if (bin != NULL)
    {
      cxd56_audio_ac_reg_set_dncram(id, bin);
    }

  /* Enable DNC. */

  if (en)
    {
      cxd56_audio_ac_reg_enable_dnc(id);
    }
}

void cxd56_audio_filter_set_deq(bool en,
                                FAR cxd56_audio_deq_coef_t *deq)
{
  /* Disable DEQ. */

  cxd56_audio_ac_reg_disable_deq();

  /* Set DEQ coef data to register. */

  if (deq != NULL)
    {
      cxd56_audio_ac_reg_set_deq_param(deq);
    }

  /* Enable DEQ. */

  if (en)
    {
      cxd56_audio_ac_reg_enable_deq();
    }
}
