/****************************************************************************
 * drivers/audio/wm8994_debug.c
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

#include <stdint.h>
#include <assert.h>
#include <fixedmath.h>
#include <syslog.h>

#include <nuttx/audio/audio.h>
#include <nuttx/audio/wm8904.h>

#include "wm8994.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_WM8994_REGDUMP
struct wb8994_regdump_s
{
  FAR const char *regname;
  uint16_t regaddr;
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_WM8994_REGDUMP
static const struct wb8994_regdump_s g_wm8994_debug[] =
{
  {"ID",                            WM8994_ID},
  {"PM1",                           WM8994_PM1},
  {"PM2",                           WM8994_PM2},
  {"PM3",                           WM8994_PM3},
  {"PM4",                           WM8994_PM4},
  {"PM5",                           WM8994_PM5},
  {"PM6",                           WM8994_PM6},
  {"INPUT_MIXER1",                  WM8994_INPUT_MIXER1},
  {"LEFTLINE_12_VOL",               WM8994_LEFTLINE_12_VOL},
  {"LEFTLINE_34_VOL",               WM8994_LEFTLINE_34_VOL},
  {"RIGHTLINE_12_VOL",              WM8994_RIGHTLINE_12_VOL},
  {"RIGHTLINE_34_VOL",              WM8994_RIGHTLINE_34_VOL},
  {"LEFT_OUTPUT_VOL",               WM8994_LEFT_OUTPUT_VOL},
  {"RIGHT_OUTPUT_VOL",              WM8994_RIGHT_OUTPUT_VOL},
  {"LINE_OUTPUTS_VOL",              WM8994_LINE_OUTPUTS_VOL},
  {"HPOUT2_VOL",                    WM8994_HPOUT2_VOL},
  {"LEFT_OPGA_VOL",                 WM8994_LEFT_OPGA_VOL},
  {"RIGHT_OPGA_VOL",                WM8994_RIGHT_OPGA_VOL},
  {"SPKMIXL_ATT",                   WM8994_SPKMIXL_ATT},
  {"SPKMIXR_ATT",                   WM8994_SPKMIXR_ATT},
  {"SPKOUT_MIXERS",                 WM8994_SPKOUT_MIXERS},
  {"CLASS_D",                       WM8994_CLASS_D},
  {"SPEAKER_VOL_LEFT",              WM8994_SPEAKER_VOL_LEFT},
  {"SPEAKER_VOL_RIGHT",             WM8994_SPEAKER_VOL_RIGHT},
  {"INPUT_MIXER2",                  WM8994_INPUT_MIXER2},
  {"INPUT_MIXER3",                  WM8994_INPUT_MIXER3},
  {"INPUT_MIXER4",                  WM8994_INPUT_MIXER4},
  {"INPUT_MIXER5",                  WM8994_INPUT_MIXER5},
  {"INPUT_MIXER6",                  WM8994_INPUT_MIXER6},
  {"OUTPUT_MIXER1",                 WM8994_OUTPUT_MIXER1},
  {"OUTPUT_MIXER2",                 WM8994_OUTPUT_MIXER2},
  {"OUTPUT_MIXER3",                 WM8994_OUTPUT_MIXER3},
  {"OUTPUT_MIXER4",                 WM8994_OUTPUT_MIXER4},
  {"OUTPUT_MIXER5",                 WM8994_OUTPUT_MIXER5},
  {"OUTPUT_MIXER6",                 WM8994_OUTPUT_MIXER6},
  {"HPOUT2_MIXER",                  WM8994_HPOUT2_MIXER},
  {"LINE_MIXER1",                   WM8994_LINE_MIXER1},
  {"LINE_MIXER2",                   WM8994_LINE_MIXER2},
  {"SPEAKER_MIXER",                 WM8994_SPEAKER_MIXER},
  {"ADDITIONAL_CTL",                WM8994_ADDITIONAL_CTL},
  {"ANTI_POP1",                     WM8994_ANTI_POP1},
  {"ANTI_POP2",                     WM8994_ANTI_POP2},
  {"MIC_BIAS",                      WM8994_MIC_BIAS},
  {"LDO_1",                         WM8994_LDO_1},
  {"LDO_2",                         WM8994_LDO_2},
  {"CHARGE_PUMP1",                  WM8994_CHARGE_PUMP1},
  {"CHARGE_PUMP2",                  WM8994_CHARGE_PUMP2},
  {"CLASS_W_1",                     WM8994_CLASS_W_1},
  {"DC_SERVO1",                     WM8994_DC_SERVO1},
  {"DC_SERVO2",                     WM8994_DC_SERVO2},
  {"DC_SERVO_BB",                   WM8994_DC_SERVO_RB},
  {"DC_SERVO4",                     WM8994_DC_SERVO4},
  {"ANA_HP1",                       WM8994_ANA_HP1},
  {"CHIP_REV",                      WM8994_CHIP_REV},
  {"CTL_IF",                        WM8994_CTL_IF},
  {"WR_CTL_SEQ1",                   WM8994_WR_CTL_SEQ1},
  {"WR_CTL_SEQ2",                   WM8994_WR_CTL_SEQ2},
  {"AIF1_CLK1",                     WM8994_AIF1_CLK1},
  {"AIF1_CLK2",                     WM8994_AIF1_CLK2},
  {"AIF2_CLK1",                     WM8994_AIF2_CLK1},
  {"AIF2_CLK2",                     WM8994_AIF2_CLK2},
  {"CLK1",                          WM8994_CLK1},
  {"CLK2",                          WM8994_CLK2},
  {"AIF1_RATE",                     WM8994_AIF1_RATE},
  {"AIF2_RATE",                     WM8994_AIF2_RATE},
  {"RATE_STATUS",                   WM8994_RATE_STATUS},
  {"PLL1_CTL1",                     WM8994_PLL1_CTL1},
  {"PLL1_CTL2",                     WM8994_PLL1_CTL2},
  {"PLL1_CTL3",                     WM8994_PLL1_CTL3},
  {"PLL1_CTL4",                     WM8994_PLL1_CTL4},
  {"PLL1_CTL5",                     WM8994_PLL1_CTL5},
  {"PLL2_CTL1",                     WM8994_PLL2_CTL1},
  {"PLL2_CTL2",                     WM8994_PLL2_CTL2},
  {"PLL2_CTL3",                     WM8994_PLL2_CTL3},
  {"PLL2_CTL4",                     WM8994_PLL2_CTL4},
  {"PLL2_CTL5",                     WM8994_PLL2_CTL5},
  {"AIF1_CTL1",                     WM8994_AIF1_CTL1},
  {"AIF1_CTL2",                     WM8994_AIF1_CTL2},
  {"AIF1_MASTER_SLAVE",             WM8994_AIF1_MASTER_SLAVE},
  {"AIF1_BCLK",                     WM8994_AIF1_BCLK},
  {"AIF1_ADC_LRCLK",                WM8994_AIF1_ADC_LRCLK},
  {"AIF1_DAC_LRCLK",                WM8994_AIF1_DAC_LRCLK},
  {"AIF1_DAC_DATA",                 WM8994_AIF1_DAC_DATA},
  {"AIF1_ADC_DATA",                 WM8994_AIF1_ADC_DATA},
  {"AIF2_CTL1",                     WM8994_AIF2_CTL1},
  {"AIF2_CTL2",                     WM8994_AIF2_CTL2},
  {"AIF2_MASTER_SLAVE",             WM8994_AIF2_MASTER_SLAVE},
  {"AIF2_BCLKK",                    WM8994_AIF2_BCLK},
  {"AIF2_ADC_LRCLK",                WM8994_AIF2_ADC_LRCLK},
  {"AIF2_DAC_LRCLK",                WM8994_AIF2_DAC_LRCLK},
  {"AIF2_DAC_DATA",                 WM8994_AIF2_DAC_DATA},
  {"AIF2_ADC_DATA",                 WM8994_AIF2_ADC_DATA},
  {"AIF1_ADC1_LEFT_VOL",            WM8994_AIF1_ADC1_LEFT_VOL},
  {"AIF1_ADC1_RIGHT_VOL",           WM8994_AIF1_ADC1_RIGHT_VOL},
  {"AIF1_DAC1_LEFT_VOL",            WM8994_AIF1_DAC1_LEFT_VOL},
  {"AIF1_DAC1_RIGHT_VOL",           WM8994_AIF1_DAC1_RIGHT_VOL},
  {"AIF1_ADC2_LEFT_VOL",            WM8994_AIF1_ADC2_LEFT_VOL},
  {"AIF1_ADC2_RIGHT_VOL",           WM8994_AIF1_ADC2_RIGHT_VOL},
  {"AIF1_DAC2_LEFT_VOL",            WM8994_AIF1_DAC2_LEFT_VOL},
  {"AIF1_DAC2_RIGHT_VOL",           WM8994_AIF1_DAC2_RIGHT_VOL},
  {"AIF1_ADC1_FILTERS",             WM8994_AIF1_ADC1_FILTERS},
  {"AIF1_ADC2_FILTERS",             WM8994_AIF1_ADC2_FILTERS},
  {"AIF1_DAC1_FILTERS1",            WM8994_AIF1_DAC1_FILTERS1},
  {"AIF1_DAC1_FILTERS2",            WM8994_AIF1_DAC1_FILTERS2},
  {"AIF1_DAC2_FILTERS1",            WM8994_AIF1_DAC2_FILTERS1},
  {"AIF1_DAC2_FILTERS2",            WM8994_AIF1_DAC2_FILTERS2},
  {"AIF1_DRC1_1",                   WM8994_AIF1_DRC1_1},
  {"AIF1_DRC1_2",                   WM8994_AIF1_DRC1_2},
  {"AIF1_DRC1_3",                   WM8994_AIF1_DRC1_3},
  {"AIF1_DRC1_4",                   WM8994_AIF1_DRC1_4},
  {"AIF1_DRC1_5",                   WM8994_AIF1_DRC1_5},
  {"AIF1_DRC2_1",                   WM8994_AIF1_DRC2_1},
  {"AIF1_DRC2_2",                   WM8994_AIF1_DRC2_2},
  {"AIF1_DRC2_3",                   WM8994_AIF1_DRC2_3},
  {"AIF1_DRC2_4",                   WM8994_AIF1_DRC2_4},
  {"AIF1_DRC2_5",                   WM8994_AIF1_DRC2_5},
  {"AIF1_DAC1_EQ_GAINS_1",          WM8994_AIF1_DAC1_EQ_GAINS_1},
  {"AIF1_DAC1_EQ_GAINS_2",          WM8994_AIF1_DAC1_EQ_GAINS_2},
  {"AIF1_DAC1_EQ_BAND_1A",          WM8994_AIF1_DAC1_EQ_BAND_1A},
  {"AIF1_DAC1_EQ_BAND_1B",          WM8994_AIF1_DAC1_EQ_BAND_1B},
  {"AIF1_DAC1_EQ_BAND_1PG",         WM8994_AIF1_DAC1_EQ_BAND_1PG},
  {"AIF1_DAC1_EQ_BAND_2A",          WM8994_AIF1_DAC1_EQ_BAND_2A},
  {"AIF1_DAC1_EQ_BAND_2B",          WM8994_AIF1_DAC1_EQ_BAND_2B},
  {"AIF1_DAC1_EQ_BAND_2C",          WM8994_AIF1_DAC1_EQ_BAND_2C},
  {"AIF1_DAC1_EQ_BAND_2PG",         WM8994_AIF1_DAC1_EQ_BAND_2PG},
  {"AIF1_DAC1_EQ_BAND_3A",          WM8994_AIF1_DAC1_EQ_BAND_3A},
  {"AIF1_DAC1_EQ_BAND_3B",          WM8994_AIF1_DAC1_EQ_BAND_3B},
  {"AIF1_DAC1_EQ_BAND_3C",          WM8994_AIF1_DAC1_EQ_BAND_3C},
  {"AIF1_DAC1_EQ_BAND_3PG",         WM8994_AIF1_DAC1_EQ_BAND_3PG},
  {"AIF1_DAC1_EQ_BAND_4A",          WM8994_AIF1_DAC1_EQ_BAND_4A},
  {"AIF1_DAC1_EQ_BAND_4B",          WM8994_AIF1_DAC1_EQ_BAND_4B},
  {"AIF1_DAC1_EQ_BAND_4C",          WM8994_AIF1_DAC1_EQ_BAND_4C},
  {"AIF1_DAC1_EQ_BAND_4PG",         WM8994_AIF1_DAC1_EQ_BAND_4PG},
  {"AIF1_DAC1_EQ_BAND_5A",          WM8994_AIF1_DAC1_EQ_BAND_5A},
  {"AIF1_DAC1_EQ_BAND_5B",          WM8994_AIF1_DAC1_EQ_BAND_5B},
  {"AIF1_DAC1_EQ_BAND_5PG",         WM8994_AIF1_DAC1_EQ_BAND_5PG},
  {"AIF1_DAC2_EQ_GAINS_1",          WM8994_AIF1_DAC2_EQ_GAINS_1},
  {"AIF1_DAC2_EQ_GAINS_2",          WM8994_AIF1_DAC2_EQ_GAINS_2},
  {"AIF1_DAC2_EQ_BAND_1A",          WM8994_AIF1_DAC2_EQ_BAND_1A},
  {"AIF1_DAC2_EQ_BAND_1B",          WM8994_AIF1_DAC2_EQ_BAND_1B},
  {"AIF1_DAC2_EQ_BAND_1PG",         WM8994_AIF1_DAC2_EQ_BAND_1PG},
  {"AIF1_DAC2_EQ_BAND_2A",          WM8994_AIF1_DAC2_EQ_BAND_2A},
  {"AIF1_DAC2_EQ_BAND_2B",          WM8994_AIF1_DAC2_EQ_BAND_2B},
  {"AIF1_DAC2_EQ_BAND_2C",          WM8994_AIF1_DAC2_EQ_BAND_2C},
  {"AIF1_DAC2_EQ_BAND_2PG",         WM8994_AIF1_DAC2_EQ_BAND_2PG},
  {"AIF1_DAC2_EQ_BAND_3A",          WM8994_AIF1_DAC2_EQ_BAND_3A},
  {"AIF1_DAC2_EQ_BAND_3B",          WM8994_AIF1_DAC2_EQ_BAND_3B},
  {"AIF1_DAC2_EQ_BAND_3C",          WM8994_AIF1_DAC2_EQ_BAND_3C},
  {"AIF1_DAC2_EQ_BAND_3PG",         WM8994_AIF1_DAC2_EQ_BAND_3PG},
  {"AIF1_DAC2_EQ_BAND_4A",          WM8994_AIF1_DAC2_EQ_BAND_4A},
  {"AIF1_DAC2_EQ_BAND_4B",          WM8994_AIF1_DAC2_EQ_BAND_4B},
  {"AIF1_DAC2_EQ_BAND_4C",          WM8994_AIF1_DAC2_EQ_BAND_4C},
  {"AIF1_DAC2_EQ_BAND_4PG",         WM8994_AIF1_DAC2_EQ_BAND_4PG},
  {"AIF1_DAC2_EQ_BAND_5A",          WM8994_AIF1_DAC2_EQ_BAND_5A},
  {"AIF1_DAC2_EQ_BAND_5B",          WM8994_AIF1_DAC2_EQ_BAND_5B},
  {"AIF1_DAC2_EQ_BAND_5PG",         WM8994_AIF1_DAC2_EQ_BAND_5PG},
  {"AIF2_ADC1_LEFT_VOL",            WM8994_AIF2_ADC1_LEFT_VOL},
  {"AIF2_ADC1_RIGHT_VOL",           WM8994_AIF2_ADC1_RIGHT_VOL},
  {"AIF2_DAC1_LEFT_VOL",            WM8994_AIF2_DAC1_LEFT_VOL},
  {"AIF2_DAC1_RIGHT_VOL",           WM8994_AIF2_DAC1_RIGHT_VOL},
  {"AIF2_ADC_FILTERS",              WM8994_AIF2_ADC_FILTERS},
  {"AIF2_DAC_FILTERS1",             WM8994_AIF2_DAC_FILTERS1},
  {"AIF2_DAC_FILTERS2",             WM8994_AIF2_DAC_FILTERS2},
  {"AIF2_DRC_1",                    WM8994_AIF2_DRC_1},
  {"AIF2_DRC_2",                    WM8994_AIF2_DRC_2},
  {"AIF2_DRC_3",                    WM8994_AIF2_DRC_3},
  {"AIF2_DRC_4",                    WM8994_AIF2_DRC_4},
  {"AIF2_DRC_5",                    WM8994_AIF2_DRC_5},
  {"AIF2_EQ_GAINS_1",               WM8994_AIF2_EQ_GAINS_1},
  {"AIF2_EQ_GAINS_2",               WM8994_AIF2_EQ_GAINS_2},
  {"AIF2_EQ_BAND_1A",               WM8994_AIF2_EQ_BAND_1A},
  {"AIF2_EQ_BAND_1B",               WM8994_AIF2_EQ_BAND_1B},
  {"AIF2_EQ_BAND_1PG",              WM8994_AIF2_EQ_BAND_1PG},
  {"AIF2_EQ_BAND_2A",               WM8994_AIF2_EQ_BAND_2A},
  {"AIF2_EQ_BAND_2B",               WM8994_AIF2_EQ_BAND_2B},
  {"AIF2_EQ_BAND_2C",               WM8994_AIF2_EQ_BAND_2C},
  {"AIF2_EQ_BAND_2PG",              WM8994_AIF2_EQ_BAND_2PG},
  {"AIF2_EQ_BAND_3A",               WM8994_AIF2_EQ_BAND_3A},
  {"AIF2_EQ_BAND_3B",               WM8994_AIF2_EQ_BAND_3B},
  {"AIF2_EQ_BAND_3C",               WM8994_AIF2_EQ_BAND_3C},
  {"AIF2_EQ_BAND_3PG",              WM8994_AIF2_EQ_BAND_3PG},
  {"AIF2_EQ_BAND_4A",               WM8994_AIF2_EQ_BAND_4A},
  {"AIF2_EQ_BAND_4B",               WM8994_AIF2_EQ_BAND_4B},
  {"AIF2_EQ_BAND_4C",               WM8994_AIF2_EQ_BAND_4C},
  {"AIF2_EQ_BAND_4PG",              WM8994_AIF2_EQ_BAND_4PG},
  {"AIF2_EQ_BAND_5A",               WM8994_AIF2_EQ_BAND_5A},
  {"AIF2_EQ_BAND_5B",               WM8994_AIF2_EQ_BAND_5B},
  {"AIF2_EQ_BAND_5PG",              WM8994_AIF2_EQ_BAND_5PG},
  {"DAC1_MIXER_VOLS",               WM8994_DAC1_MIXER_VOLS},
  {"DAC1_LEFT_MIXER_ROUTING",       WM8994_DAC1_LEFT_MIXER_ROUTING},
  {"DAC1_RIGHT_MIXER_ROUTING",      WM8994_DAC1_RIGHT_MIXER_ROUTING},
  {"DAC2_MIXER_VOLS",               WM8994_DAC2_MIXER_VOLS},
  {"DAC2_LEFT_MIXER_ROUTING",       WM8994_DAC2_LEFT_MIXER_ROUTING},
  {"DAC2_RIGHT_MIXER_ROUTING",      WM8994_DAC2_RIGHT_MIXER_ROUTING},
  {"ADC1_LEFT_MIXER_ROUTING",       WM8994_ADC1_LEFT_MIXER_ROUTING},
  {"ADC1_RIGHT_MIXER_ROUTING",      WM8994_ADC1_RIGHT_MIXER_ROUTING},
  {"ADC2_LEFT_MIXER_ROUTING",       WM8994_ADC2_LEFT_MIXER_ROUTING},
  {"ADC2_RIGHT_MIXER_ROUTING",      WM8994_ADC2_RIGHT_MIXER_ROUTING},
  {"DAC1_LEFT_VOL",                 WM8994_DAC1_LEFT_VOL},
  {"DAC1_RIGHT_VOL",                WM8994_DAC1_RIGHT_VOL},
  {"DAC2_LEFT_VOL",                 WM8994_DAC2_LEFT_VOL},
  {"DAC2_RIGHT_VOL",                WM8994_DAC2_RIGHT_VOL},
  {"DAC_SOFT_MUTE",                 WM8994_DAC_SOFT_MUTE},
  {"OVER_SAMPLING",                 WM8994_OVER_SAMPLING},
  {"SIDE_TONE",                     WM8994_SIDE_TONE},
  {"GPIO1",                         WM8994_GPIO1},
  {"GPIO2",                         WM8994_GPIO2},
  {"GPIO3",                         WM8994_GPIO3},
  {"GPIO4",                         WM8994_GPIO4},
  {"GPIO5",                         WM8994_GPIO5},
  {"GPIO6",                         WM8994_GPIO6},
  {"GPIO7",                         WM8994_GPIO7},
  {"GPIO8",                         WM8994_GPIO8},
  {"GPIO9",                         WM8994_GPIO9},
  {"GPIO10",                        WM8994_GPIO10},
  {"GPIO11",                        WM8994_GPIO11},
  {"PULL_CTL1",                     WM8994_PULL_CTL1},
  {"PULL_CTL2",                     WM8994_PULL_CTL2},
  {"INT_STATUS1",                   WM8994_INT_STATUS1},
  {"INT_STATUS2",                   WM8994_INT_STATUS2},
  {"INT_RAW_STATUS2",               WM8994_INT_RAW_STATUS2},
  {"INT_STATUS1_MASK",              WM8994_INT_STATUS1_MASK},
  {"INT_STATUS2_MASK",              WM8994_INT_STATUS2_MASK},
  {"INT_CTL",                       WM8994_INT_CTL},
  {"INT_DEBOUNCE",                  WM8994_INT_DEBOUNCE},
};

#  define WM8994_NREGISTERS (sizeof(g_wm8994_debug)/sizeof(struct wb8994_regdump_s))
#endif /* CONFIG_WM8994_REGDUMP */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wm8994_dump_registers
 *
 * Description:
 *   Dump the contents of all WM8994 registers to the syslog device
 *
 * Input Parameters:
 *   dev - The device instance returned by wm8994_initialize
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_WM8994_REGDUMP
void wm8994_dump_registers(FAR struct audio_lowerhalf_s *dev,
                           FAR const char *msg)
{
  int i;

  syslog(LOG_INFO, "WM8994 Registers: %s\n", msg);
  for (i = 0; i < WM8994_NREGISTERS; i++)
    {
      syslog(LOG_INFO, "%24s[%04x]: %04x\n",
             g_wm8994_debug[i].regname, g_wm8994_debug[i].regaddr,
             wm8994_readreg((FAR struct wm8994_dev_s *)dev,
                            g_wm8994_debug[i].regaddr));
    }
}
#endif /* CONFIG_WM8994_REGDUMP */

/****************************************************************************
 * Name: wm8994_clock_analysis
 *
 * Description:
 *   Analyze the settings in the clock chain and dump to syslog.
 *
 * Input Parameters:
 *   dev - The device instance returned by wm8994_initialize
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_WM8994_CLKDEBUG
void wm8994_clock_analysis(FAR struct audio_lowerhalf_s *dev,
                           FAR const char *msg)
{
  FAR struct wm8994_dev_s *priv = (FAR struct wm8994_dev_s *)dev;
  uint32_t sysclk;
  uint32_t bclk;
  uint32_t lrclk;
  uint16_t regval;
  unsigned int tmp;
  double ftmp;

  syslog(LOG_INFO, "WM8994 Clock Analysis: %s\n", msg);
  DEBUGASSERT(priv && priv->lower);
  syslog(LOG_INFO, "  MCLK:            %lu Hz\n",
         (unsigned long)priv->lower->mclk);

  /* Is the SYSCLK source the FLL?  Or MCK? */

  regval = wm8994_readreg(priv, WM8994_CLKRATE2);
  if ((regval & WM8994_SYSCLK_SRC) == WM8994_SYSCLK_SRCMCLK)
    {
      /* The SYSCLK divider bypasses the FLL and takes its input
       * directly from MCLK.
       */

      sysclk = priv->lower->mclk;
      syslog(LOG_INFO, "  SYSCLK Source:   MCLK (%s)\n",
             (regval & WM8994_MCLK_INV) != 0 ? "inverted" : "not inverted");
    }
  else
    {
      uint32_t fref;
      uint32_t fvco;
      uint32_t fout;
      unsigned int outdiv;
      unsigned int frndx;
      unsigned int flln;
      unsigned int fllk;
      unsigned int fratio;
      double nk;

      /* Assume that the Fref input to the FLL is MCLK */

      fref = priv->lower->mclk;

      /* Now get the real FLL input source */

      regval = wm8994_readreg(priv, WM8994_FLL_CTRL5);
      switch (regval & WM8994_FLL_CLK_REF_SRC_MASK)
        {
          case WM8994_FLL_CLK_REF_SRC_MCLK:
            syslog(LOG_INFO, "  FLL Source:      MCLK\n");
            break;

          case WM8994_FLL_CLK_REF_SRC_BCLK:
            syslog(LOG_INFO, "  ERROR: FLL source is BCLK: %04x\n",
                   regval);
            break;

          case WM8994_FLL_CLK_REF_SRC_LRCLK:
            syslog(LOG_INFO, "  ERROR: FLL source is LRCLK: %04x\n",
                   regval);
            break;

          default:
            syslog(LOG_INFO, "  ERROR: Unrecognized FLL source: %04x\n",
                   regval);
        }

      syslog(LOG_INFO, "  Fref:            %lu Hz (before divider)\n",
             fref);
      switch (regval & WM8994_FLL_CLK_REF_DIV_MASK)
        {
          case WM8994_FLL_CLK_REF_DIV1:
            syslog(LOG_INFO, "  FLL_CLK_REF_DIV: 1\n");
            break;

          case WM8994_FLL_CLK_REF_DIV2:
            syslog(LOG_INFO, "  FLL_CLK_REF_DIV: 2\n");
            fref >>= 1;
            break;

          case WM8994_FLL_CLK_REF_DIV4:
            syslog(LOG_INFO, "  FLL_CLK_REF_DIV: 4\n");
            fref >>= 2;
            break;

          case WM8994_FLL_CLK_REF_DIV8:
            syslog(LOG_INFO, "  FLL_CLK_REF_DIV: 8\n");
            fref >>= 3;
            break;
        }

      syslog(LOG_INFO, "  Fref:            %lu Hz (after divider)\n", fref);

      regval = wm8994_readreg(priv, WM8994_FLL_CTRL2);
      frndx  = (regval & WM8994_FLL_FRATIO_MASK) >> WM8994_FLL_FRATIO_SHIFT;
      tmp    = (regval & WM8994_FLL_CTRL_RATE_MASK) >>
                WM8994_FLL_CTRL_RATE_SHIFT;
      outdiv = ((regval & WM8994_FLL_OUTDIV_MASK) >>
                 WM8994_FLL_OUTDIV_SHIFT) + 1;

      syslog(LOG_INFO, "  FLL_CTRL_RATE:   Fvco / %u\n", tmp + 1);

      regval = wm8904_readreg(priv, WM8994_FLL_CTRL4);
      flln   = (regval & WM8994_FLL_N_MASK) >> WM8994_FLL_N_SHIFT;
      tmp    = (regval & WM8994_FLL_GAIN_MASK) >> WM8994_FLL_GAIN_SHIFT;

      syslog(LOG_INFO, "  FLL_GAIN:        %u\n", (1 << tmp));

      fllk   = wm8994_readreg(priv, WM8994_FLL_CTRL3);
      nk     = (double)flln + ((double)fllk / 65536.0);
      fratio = g_fllratio[frndx];

      syslog(LOG_INFO, "  FLL_FRATIO:      %u\n", fratio);
      syslog(LOG_INFO, "  FLL_OUTDIV:      %u\n", outdiv);
      syslog(LOG_INFO, "  FLL_N.K:         %u.%05u\n", flln, fllk);

      ftmp = nk * (double)fref * (double)fratio;
      fvco = (uint32_t)ftmp;

      syslog(LOG_INFO, "  Fvco:            %lu Hz\n", (unsigned long)fvco);

      fout = fvco / outdiv;
      syslog(LOG_INFO, "  Fout:            %lu Hz\n", (unsigned long)fout);

      regval = wm8994_readreg(priv, WM8994_FLL_CTRL1);

      syslog(LOG_INFO, "  FLL_FRACN_ENA:   %s\n",
             (regval & WM8994_FLL_FRACN_ENA) != 0 ? "Enabled" : "Disabled");
      syslog(LOG_INFO, "  FLL_OSC_ENA:     %s\n",
             (regval & WM8994_FLL_OSC_ENA) != 0 ? "Enabled" : "Disabled");
      syslog(LOG_INFO, "  FLL_ENA:         %s\n",
             (regval & WM8994_FLL_ENA) != 0 ? "Enabled" : "Disabled");

      if ((regval & WM8994_FLL_ENA) == 0)
        {
          syslog(LOG_INFO, "  No SYSCLK\n");
          return;
        }

      sysclk = fout;
    }

  syslog(LOG_INFO, "  SYSCLK:          %lu Hz (before divider)\n",
         (unsigned long)sysclk);

  regval = wm8994_readreg(priv, WM8994_CLKRATE0);
  if ((regval & WM8994_MCLK_DIV) == WM8994_MCLK_DIV1)
    {
      syslog(LOG_INFO, "  MCLK_DIV:        1\n");
    }
  else
    {
      syslog(LOG_INFO, "  MCLK_DIV:        2\n");
      sysclk >>= 1;
    }

  syslog(LOG_INFO, "  SYSCLK:          %lu (after divider)\n",
         (unsigned long)sysclk);

  regval = wm8994_readreg(priv, WM8994_CLKRATE2);

  syslog(LOG_INFO, "  CLK_SYS_ENA:     %s\n",
         (regval & WM8994_CLK_SYS_ENA) != 0 ? "Enabled" : "Disabled");

  if ((regval & WM8994_CLK_SYS_ENA) == 0)
    {
      syslog(LOG_INFO, "  No SYSCLK\n");
      return;
    }

  regval = wm8994_readreg(priv, WM8994_AIF2);
  tmp    = (regval & WM8994_BCLK_DIV_MASK) >> WM8994_BCLK_DIV_SHIFT;
  tmp    = g_sysclk_scaleb1[tmp];
  ftmp   = (double)tmp / 2.0;

  syslog(LOG_INFO, "  BCLK_DIV:        SYSCLK / %u.%01u\n",
         (unsigned int)(tmp >> 1), (unsigned int)(5 * (tmp & 1)));

  bclk = (uint32_t)(sysclk / ftmp);

  syslog(LOG_INFO, "  BCLK:            %lu Hz\n", (unsigned long)bclk);

  regval = wm8994_readreg(priv, WM8994_AIF1);
  syslog(LOG_INFO, "  BCLK_DIR:        %s\n",
         (regval & WM8994_BCLK_DIR) != 0 ? "Output" : "Input");

  regval = wm8994_readreg(priv, WM8994_AIF3);
  tmp = (regval & WM8994_LRCLK_RATE_MASK) >> WM8994_LRCLK_RATE_SHIFT;

  lrclk = bclk / tmp;

  syslog(LOG_INFO, "  LRCLK_RATE:      BCLK / %lu\n", (unsigned long)tmp);
  syslog(LOG_INFO, "  LRCLK:           %lu Hz\n", (unsigned long)lrclk);
  syslog(LOG_INFO, "  LRCLK_DIR:       %s\n",
         (regval & WM8994_LRCLK_DIR) != 0 ? "Output" : "Input");
}
#endif /* CONFIG_WM8994_CLKDEBUG */
