/****************************************************************************
 * boards/arm/cxd56xx/drivers/audio/cxd56_audio_digital.c
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
