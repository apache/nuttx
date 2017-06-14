/****************************************************************************
 * drivers/audio/cs43l22_debug.c
 * Audio device driver for Cirrus Logic CS43L22 Audio codec.
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author:  Taras Drozdovsky <t.drozdovskiy@gmail.comnuttx.org>
 *
 * References:
 * - "CS43L22 Ultra Low Power CODEC for Portable Audio Applications, Pre-
 *    Production", September 2012, Rev b1, Cirrus Logic
 * -  The framework for this driver is based on Ken Pettit's VS1053 driver.
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

#include <stdint.h>
#include <assert.h>
#include <fixedmath.h>
#include <syslog.h>

#include <nuttx/audio/audio.h>
#include <nuttx/audio/cs43l22.h>

#include "cs43l22.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_CS43L22_REGDUMP
struct cs43l22_regdump_s
{
  FAR const char *regname;
  uint8_t regaddr;
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_CS43L22_REGDUMP
static const struct cs43l22_regdump_s g_cs43l22_debug[] =
{
  {"CHIP_ID_REV",     CS43L22_ID_REV          },
  {"POWER_CTRL1",     CS43L22_POWER_CTRL1     },
  {"POWER_CTRL2",     CS43L22_POWER_CTRL2     },
  {"CLOCK_CTRL",      CS43L22_CLOCK_CTRL      },
  {"INTERFACE_CTRL1", CS43L22_INTERFACE_CTRL1 },
  {"INTERFACE_CTRL2", CS43L22_INTERFACE_CTRL2 },
  {"PASS_SEL_A",      CS43L22_PASS_SEL_A      },
  {"PASS_SEL_B",      CS43L22_PASS_SEL_B      },
  {"ANLG_ZC_SR_SEL",  CS43L22_ANLG_ZC_SR_SEL  },
  {"PASS_GANG_CTRL",  CS43L22_PASS_GANG_CTRL  },
  {"PLAYBACK_CTRL1",  CS43L22_PLAYBACK_CTRL1  },
  {"MISCLLNS_CTRL",   CS43L22_MISCLLNS_CTRL   },
  {"PLAYBACK_CTRL2",  CS43L22_PLAYBACK_CTRL2  },
  {"PASS_VOL_A",      CS43L22_PASS_VOL_A      },
  {"PASS_VOL_B",      CS43L22_PASS_VOL_B      },
  {"PCM_VOL_A",       CS43L22_PCM_VOL_A       },
  {"PCM_VOL_B",       CS43L22_PCM_VOL_B       },
  {"BP_FREQ_ON_T",    CS43L22_BP_FREQ_ON_TIME },
  {"BP_VOL_OFF_T",    CS43L22_BP_VOL_OFF_TIME },
  {"BP_TONE_CFG",     CS43L22_BP_TONE_CFG     },
  {"TONE_CTRL",       CS43L22_TONE_CTRL       },
  {"MS_VOL_CTRL_A",   CS43L22_MS_VOL_CTRL_A   },
  {"MS_VOL_CTRL_B",   CS43L22_MS_VOL_CTRL_B   },
  {"HP_VOL_CTRL_A",   CS43L22_HP_VOL_CTRL_A   },
  {"HP_VOL_CTRL_B",   CS43L22_HP_VOL_CTRL_B   },
  {"SPK_VOL_CTRL_A",  CS43L22_SPK_VOL_CTRL_A  },
  {"SPK_VOL_CTRL_B",  CS43L22_SPK_VOL_CTRL_B  },
  {"PCM_CH_SWAP",     CS43L22_PCM_CH_SWAP     },
  {"LIM_CTRL1",       CS43L22_LIM_CTRL1       },
  {"LIM_CTRL2",       CS43L22_LIM_CTRL2       },
  {"LIM_ATTACK_RATE", CS43L22_LIM_ATTACK_RATE },
  {"STATUS",          CS43L22_STATUS          },
  {"BAT_COMP",        CS43L22_BAT_COMP        },
  {"VP_BAT_LEVEL",    CS43L22_VP_BAT_LEVEL    },
  {"SPK_STATUS",      CS43L22_SPK_STATUS      },
  {"TEMP_MON_CTRL",   CS43L22_TEMP_MON_CTRL   },
  {"THERMAL_FOLDBACK",CS43L22_THERMAL_FOLDBACK},
  {"CHRG_PUMP_FREQ",  CS43L22_CHRG_PUMP_FREQ  }
};

#  define CS43L22_NREGISTERS (sizeof(g_cs43l22_debug)/sizeof(struct cs43l22_regdump_s))
#endif /* CONFIG_CS43L22_REGDUMP */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cs43l22_dump_registers
 *
 * Description:
 *   Dump the contents of all CS43L22 registers to the syslog device
 *
 * Input Parameters:
 *   dev - The device instance returned by cs43l22_initialize
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_CS43L22_REGDUMP
void cs43l22_dump_registers(FAR struct audio_lowerhalf_s *dev,
                           FAR const char *msg)
{
  int i;

  syslog(LOG_INFO, "CS43L22 Registers: %s\n", msg);
  for (i = 0; i < CS43L22_NREGISTERS; i++)
    {
      syslog(LOG_INFO, "%16s[%02x]: %02x\n",
             g_cs43l22_debug[i].regname, g_cs43l22_debug[i].regaddr,
             cs43l22_readreg((FAR struct cs43l22_dev_s *)dev,
                            g_cs43l22_debug[i].regaddr));
    }
}
#endif /* CONFIG_CS43L22_REGDUMP */

/****************************************************************************
 * Name: cs43l22_clock_analysis
 *
 * Description:
 *   Analyze the settings in the clock chain and dump to syslog.
 *
 * Input Parameters:
 *   dev - The device instance returned by cs43l22_initialize
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_CS43L22_CLKDEBUG
void cs43l22_clock_analysis(FAR struct audio_lowerhalf_s *dev,
                           FAR const char *msg)
{
  #warning Missing logic
  /* TODO */
}
#endif /* CONFIG_CS43L22_CLKDEBUG */
