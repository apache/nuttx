/****************************************************************************
 * boards/arm/cxd56xx/drivers/audio/cxd56_audio_power.c
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
