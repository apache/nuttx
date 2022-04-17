/****************************************************************************
 * boards/arm/cxd56xx/drivers/audio/cxd56_audio_filter.c
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
                                cxd56_audio_dnc_bin_t *bin)
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
                                cxd56_audio_deq_coef_t *deq)
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
