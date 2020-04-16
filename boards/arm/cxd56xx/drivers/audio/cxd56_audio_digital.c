/****************************************************************************
 * boards/arm/cxd56xx/drivers/audio/cxd56_audio_digital.c
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
#include <arch/chip/audio.h>

#include "cxd56_audio_config.h"
#include "cxd56_audio_ac_reg.h"
#include "cxd56_audio_bca_reg.h"
#include "cxd56_audio_digital.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void cxd56_audio_digital_poweron(void)
{
#if defined(CONFIG_CXD56_I2S0) || defined(CONFIG_CXD56_I2S1)
  cxd56_audio_clkmode_t clk_mode = cxd56_audio_config_get_clkmode();

  /* Clear interrupt status of bck_err. */

  cxd56_audio_bca_reg_clear_bck_err_int();

  /* PowerON i2s. */

  cxd56_audio_ac_reg_poweron_i2s(clk_mode);
#endif /* defined(CONFIG_CXD56_I2S0) || defined(CONFIG_CXD56_I2S1) */
}

void cxd56_audio_digital_enable(void)
{
#if defined(CONFIG_CXD56_I2S0) || defined(CONFIG_CXD56_I2S1)

#ifdef CONFIG_CXD56_I2S0
  /* Enable I2S data input and output of SRC1 */

  cxd56_audio_ac_reg_enable_i2s_src1();
#endif /* CONFIG_CXD56_I2S0 */

#ifdef CONFIG_CXD56_I2S1
  /* Enable I2S data input and output of SRC2 */

  cxd56_audio_ac_reg_enable_i2s_src2();
#endif /* CONFIG_CXD56_I2S1 */

  if ((CXD56_AUDIO_CFG_I2S1_MODE == CXD56_AUDIO_CFG_I2S_MODE_MASTER) ||
      (CXD56_AUDIO_CFG_I2S2_MODE == CXD56_AUDIO_CFG_I2S_MODE_MASTER))
    {
      /* Enable BCK, LRCK output. */

      cxd56_audio_ac_reg_enable_i2s_bcklrckout();
    }
  else
    {
      cxd56_audio_ac_reg_disable_i2s_bcklrckout();
    }
#endif /* defined(CONFIG_CXD56_I2S0) || defined(CONFIG_CXD56_I2S1) */
}
