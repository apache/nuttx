/****************************************************************************
 * arch/xtensa/src/esp32/esp32_clockconfig.C
 *
 * Mofidifed by use in NuttX by:
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derives from software originally provided by Espressif Systems:
 *
 * Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include "xtensa.h"

#include "hardware/esp32_dport.h"
#include "hardware/esp32_soc.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum xtal_freq_e
{
  XTAL_40M = 40,
  XTAL_26M = 26,
  XTAL_24M = 24,
  XTAL_AUTO = 0
};

enum cpu_freq_e
{
  CPU_80M = 0,
  CPU_160M = 1,
  CPU_240M = 2,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

extern void ets_delay_us(int delay_us);

/****************************************************************************
 * Name: esp32_set_cpu_freq
 *
 * Description:
 *   Switch to one of PLL-based frequencies.
 *   Current frequency can be XTAL or PLL.
 *
 * Input Parameters:
 *   cpu_freq_mhz      - new CPU frequency
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32_set_cpu_freq(int cpu_freq_mhz)
{
  int dbias = DIG_DBIAS_80M_160M;
  int per_conf;
  uint32_t  value;

  switch (cpu_freq_mhz)
    {
      case 160:
        per_conf = CPU_160M;
        break;

      case 240:
        dbias = DIG_DBIAS_240M;
        per_conf = CPU_240M;
        break;

      case 80:
        per_conf = CPU_80M;

      default:
        break;
    }

  value = (((80 * MHZ) >> 12) & UINT16_MAX) |
          ((((80 * MHZ) >> 12) & UINT16_MAX) << 16);
  putreg32(value, RTC_APB_FREQ_REG);
  putreg32(per_conf, DPORT_CPU_PER_CONF_REG);
  REG_SET_FIELD(RTC_CNTL_REG, RTC_CNTL_DIG_DBIAS_WAK, dbias);
  REG_SET_FIELD(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_SOC_CLK_SEL,
                RTC_CNTL_SOC_CLK_SEL_PLL);
}

/****************************************************************************
 * Name: esp32_bbpll_configure
 *
 * Description:
 *   Configure main XTAL frequency values according to pll_freq.
 *
 * Input Parameters:
 *   xtal_freq -    XTAL frequency values
 *   pll_freq  -    PLL frequency values
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32_bbpll_configure(enum xtal_freq_e xtal_freq, int pll_freq)
{
  uint8_t div_ref;
  uint8_t div7_0;
  uint8_t div10_8;
  uint8_t lref;
  uint8_t dcur;
  uint8_t bw;
  uint8_t i2c_bbpll_lref;
  uint8_t i2c_bbpll_div_7_0;
  uint8_t i2c_bbpll_dcur;

  if (pll_freq == RTC_PLL_FREQ_320M)
    {
      /* Raise the voltage, if needed */

      REG_SET_FIELD(RTC_CNTL_REG, RTC_CNTL_DIG_DBIAS_WAK,
                    DIG_DBIAS_80M_160M);

      /* Configure 320M PLL */

      switch (xtal_freq)
        {
          case XTAL_40M:
            div_ref = 0;
            div7_0 = 32;
            div10_8 = 0;
            lref = 0;
            dcur = 6;
            bw = 3;
            break;

          case XTAL_26M:
            div_ref = 12;
            div7_0 = 224;
            div10_8 = 4;
            lref = 1;
            dcur = 0;
            bw = 1;
            break;

          case XTAL_24M:
            div_ref = 11;
            div7_0 = 224;
            div10_8 = 4;
            lref = 1;
            dcur = 0;
            bw = 1;
            break;

          default:
            div_ref = 12;
            div7_0 = 224;
            div10_8 = 4;
            lref = 0;
            dcur = 0;
            bw = 0;
            break;
        }

      I2C_WRITEREG_RTC(I2C_BBPLL, I2C_BBPLL_ENDIV5, BBPLL_ENDIV5_VAL_320M);
      I2C_WRITEREG_RTC(I2C_BBPLL, I2C_BBPLL_BBADC_DSMP,
                       BBPLL_BBADC_DSMP_VAL_320M);
    }
  else
    {
      /* Raise the voltage */

      REG_SET_FIELD(RTC_CNTL_REG, RTC_CNTL_DIG_DBIAS_WAK, DIG_DBIAS_240M);
      ets_delay_us(DELAY_PLL_DBIAS_RAISE);

      /* Configure 480M PLL */

      switch (xtal_freq)
        {
          case XTAL_40M:
            div_ref = 0;
            div7_0 = 28;
            div10_8 = 0;
            lref = 0;
            dcur = 6;
            bw = 3;
            break;

          case XTAL_26M:
            div_ref = 12;
            div7_0 = 144;
            div10_8 = 4;
            lref = 1;
            dcur = 0;
            bw = 1;
            break;

          case XTAL_24M:
            div_ref = 11;
            div7_0 = 144;
            div10_8 = 4;
            lref = 1;
            dcur = 0;
            bw = 1;
            break;

          default:
            div_ref = 12;
            div7_0 = 224;
            div10_8 = 4;
            lref = 0;
            dcur = 0;
            bw = 0;
            break;
        }

      I2C_WRITEREG_RTC(I2C_BBPLL, I2C_BBPLL_ENDIV5, BBPLL_ENDIV5_VAL_480M);
      I2C_WRITEREG_RTC(I2C_BBPLL,
                       I2C_BBPLL_BBADC_DSMP, BBPLL_BBADC_DSMP_VAL_480M);
    }

  i2c_bbpll_lref  = (lref << 7) | (div10_8 << 4) | (div_ref);
  i2c_bbpll_div_7_0 = div7_0;
  i2c_bbpll_dcur = (bw << 6) | dcur;
  I2C_WRITEREG_RTC(I2C_BBPLL, I2C_BBPLL_OC_LREF, i2c_bbpll_lref);
  I2C_WRITEREG_RTC(I2C_BBPLL, I2C_BBPLL_OC_DIV_7_0, i2c_bbpll_div_7_0);
  I2C_WRITEREG_RTC(I2C_BBPLL, I2C_BBPLL_OC_DCUR, i2c_bbpll_dcur);
}

/****************************************************************************
 * Name: esp32_bbpll_enable
 *
 * Description:
 *   Reset BBPLL configuration.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32_bbpll_enable(void)
{
  modifyreg32(RTC_CNTL_OPTIONS0_REG,
              RTC_CNTL_BIAS_I2C_FORCE_PD | RTC_CNTL_BB_I2C_FORCE_PD |
              RTC_CNTL_BBPLL_FORCE_PD | RTC_CNTL_BBPLL_I2C_FORCE_PD, 0);

  /* reset BBPLL configuration */

  I2C_WRITEREG_RTC(I2C_BBPLL, I2C_BBPLL_IR_CAL_DELAY,
                   BBPLL_IR_CAL_DELAY_VAL);
  I2C_WRITEREG_RTC(I2C_BBPLL, I2C_BBPLL_IR_CAL_EXT_CAP,
                   BBPLL_IR_CAL_EXT_CAP_VAL);
  I2C_WRITEREG_RTC(I2C_BBPLL, I2C_BBPLL_OC_ENB_FCAL,
                   BBPLL_OC_ENB_FCAL_VAL);
  I2C_WRITEREG_RTC(I2C_BBPLL, I2C_BBPLL_OC_ENB_VCON,
                   BBPLL_OC_ENB_VCON_VAL);
  I2C_WRITEREG_RTC(I2C_BBPLL, I2C_BBPLL_BBADC_CAL_7_0,
                   BBPLL_BBADC_CAL_7_0_VAL);
}

/****************************************************************************
 * Name: esp32_update_to_xtal
 *
 * Description:
 *   Switch to XTAL frequency, does not disable the PLL
 *
 * Input Parameters:
 *   freq -  XTAL frequency
 *   div  -  REF_TICK divider
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void esp32_update_to_xtal(int freq, int div)
{
  uint32_t value = (((freq * MHZ) >> 12) & UINT16_MAX)
                   | ((((freq * MHZ) >> 12) & UINT16_MAX) << 16);
  putreg32(value, RTC_APB_FREQ_REG);

  /* set divider from XTAL to APB clock */

  REG_SET_FIELD(APB_CTRL_SYSCLK_CONF_REG, APB_CTRL_PRE_DIV_CNT, div - 1);

  /* switch clock source */

  REG_SET_FIELD(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_SOC_CLK_SEL,
                RTC_CNTL_SOC_CLK_SEL_XTL);

  /* adjust ref_tick */

  modifyreg32(APB_CTRL_XTAL_TICK_CONF_REG, 0,
             (freq * MHZ) / REF_CLK_FREQ - 1);

  /* lower the voltage */

  if (freq <= 2)
    {
      REG_SET_FIELD(RTC_CNTL_REG, RTC_CNTL_DIG_DBIAS_WAK, DIG_DBIAS_2M);
    }
  else
    {
      REG_SET_FIELD(RTC_CNTL_REG, RTC_CNTL_DIG_DBIAS_WAK, DIG_DBIAS_XTAL);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_clockconfig
 *
 * Description:
 *   Called to initialize the ESP32.  This does whatever setup is needed to
 *   put the  SoC in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.
 *
 ****************************************************************************/

void esp32_clockconfig(void)
{
  uint32_t freq_mhz = CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ;
  uint32_t source_freq_mhz;
  enum xtal_freq_e xtal_freq = XTAL_40M;

  switch (freq_mhz)
    {
      case 240:
        source_freq_mhz = RTC_PLL_FREQ_480M;
        break;

      case 160:
        source_freq_mhz = RTC_PLL_FREQ_320M;
        break;

      case 80:
      default:
        return;
    }

  esp32_update_to_xtal(xtal_freq, 1);
  esp32_bbpll_enable();
  esp32_bbpll_configure(xtal_freq, source_freq_mhz);
  esp32_set_cpu_freq(freq_mhz);
}
