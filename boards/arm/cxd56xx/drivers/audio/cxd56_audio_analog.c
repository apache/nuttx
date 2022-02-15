/****************************************************************************
 * boards/arm/cxd56xx/drivers/audio/cxd56_audio_analog.c
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

#include <time.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/signal.h>

#include <arch/board/board.h>
#include <arch/chip/audio.h>

#include "cxd56_audio_config.h"
#include "cxd56_audio_analog.h"
#include "cxd56_audio_aca.h"
#include "cxd56_clock.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AUD_MCLK_EXT     (0u<<16) /* External XTAL */

/****************************************************************************
 * Private Data
 ****************************************************************************/

uint64_t g_mic_boot_start_time = 0x0ull;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void clear_mic_boot_time(void)
{
  g_mic_boot_start_time = 0x0ull;
}

static void set_mic_boot_time(void)
{
  struct timespec start;
  if (clock_systime_timespec(&start) < 0)
    {
      g_mic_boot_start_time = 0x0ull;
      return;
    }

  g_mic_boot_start_time = (uint64_t)start.tv_sec * 1000 +
                          (uint64_t)start.tv_nsec / 1000000;
}

static void wait_mic_boot_finish(void)
{
  if (g_mic_boot_start_time != 0x0ull)
    {
      struct timespec end;
      if (clock_systime_timespec(&end) < 0)
        {
          return;
        }

      uint64_t time = (uint64_t)end.tv_sec * 1000 +
                      (uint64_t)end.tv_nsec / 1000000 -
                       g_mic_boot_start_time;

      if (time < CXD56_AUDIO_MIC_BOOT_WAIT)
        {
          nxsig_usleep((CXD56_AUDIO_MIC_BOOT_WAIT - time) * 1000);
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

CXD56_AUDIO_ECODE cxd56_audio_analog_poweron(void)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

#ifdef CONFIG_CXD56_AUDIO_ANALOG_CXD5247
  if (board_aca_power_control(CXD5247_AVDD | CXD5247_DVDD, true) != 0)
    {
      return CXD56_AUDIO_ECODE_ANA_PWON;
    }

  if (!board_aca_power_monitor(CXD5247_AVDD | CXD5247_DVDD))
    {
      return CXD56_AUDIO_ECODE_ANA_PWON;
    }

  ret = cxd56_audio_aca_poweron();
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  cxd56_audio_clock_enable(AUD_MCLK_EXT, 0);
  if (!cxd56_audio_clock_is_enabled())
    {
      return CXD56_AUDIO_ECODE_ANA_CLK_EN;
    }
#endif

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_analog_poweroff(void)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

#ifdef CONFIG_CXD56_AUDIO_ANALOG_CXD5247
  cxd56_audio_clock_disable();

  ret = cxd56_audio_aca_poweroff();
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  board_aca_power_control(CXD5247_AVDD | CXD5247_DVDD, false);
#endif

  return ret;
}

CXD56_AUDIO_ECODE
cxd56_audio_analog_poweron_input(FAR cxd56_audio_mic_gain_t *gain)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

#ifdef CONFIG_CXD56_AUDIO_ANALOG_CXD5247
  uint8_t mic_dev = cxd56_audio_config_get_micdev();

  if ((mic_dev == CXD56_AUDIO_CFG_MIC_DEV_ANALOG) ||
      (mic_dev == CXD56_AUDIO_CFG_MIC_DEV_ANADIG))
    {
      ret = cxd56_audio_aca_poweron_micbias();
      if (CXD56_AUDIO_ECODE_OK != ret)
        {
          return ret;
        }

      set_mic_boot_time();
    }

  ret = cxd56_audio_aca_poweron_input(gain);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }
#endif

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_analog_poweron_output(void)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

#ifdef CONFIG_CXD56_AUDIO_ANALOG_CXD5247

  ret = cxd56_audio_aca_set_smaster();
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  ret = cxd56_audio_aca_poweron_output();
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }
#endif

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_analog_poweroff_input(void)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

#ifdef CONFIG_CXD56_AUDIO_ANALOG_CXD5247
  ret = cxd56_audio_aca_poweroff_input();
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  clear_mic_boot_time();
#endif

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_analog_poweroff_output(void)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

#ifdef CONFIG_CXD56_AUDIO_ANALOG_CXD5247
  ret = cxd56_audio_aca_poweroff_output();
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }
#endif

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_analog_enable_output(void)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

#ifdef CONFIG_CXD56_AUDIO_ANALOG_CXD5247
  ret = cxd56_audio_aca_enable_output();
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }
#endif

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_analog_disable_output(void)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

#ifdef CONFIG_CXD56_AUDIO_ANALOG_CXD5247
  ret = cxd56_audio_aca_disable_output();
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }
#endif

  return ret;
}

CXD56_AUDIO_ECODE
cxd56_audio_analog_set_micgain(FAR cxd56_audio_mic_gain_t *gain)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

#ifdef CONFIG_CXD56_AUDIO_ANALOG_CXD5247

  ret = cxd56_audio_aca_set_micgain(gain);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }
#endif

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_analog_wait_input_standby()
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

#ifdef CONFIG_CXD56_AUDIO_ANALOG_CXD5247
  wait_mic_boot_finish();
  ret = cxd56_audio_aca_notify_micbootdone();
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }
#endif

  return ret;
}
