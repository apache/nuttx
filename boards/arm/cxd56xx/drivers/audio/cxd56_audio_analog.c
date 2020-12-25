/****************************************************************************
 * boards/arm/cxd56xx/drivers/audio/cxd56_audio_analog.c
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
  if (clock_gettime(CLOCK_REALTIME, &start) < 0)
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
      if (clock_gettime(CLOCK_REALTIME, &end) < 0)
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
