/****************************************************************************
 * drivers/audio/es8388_debug.c
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

/* References:
 * -  "ES8388 Low Power Stereo Audio CODEC With Headphone Amplifier",
 *    July 2018, Rev 5.0, Everest Semiconductor
 * -  The framework for this driver is based on Taras Drozdovsky's CS43L22
 *    driver.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <fixedmath.h>
#include <syslog.h>

#include <nuttx/audio/audio.h>
#include <nuttx/audio/es8388.h>

#include "es8388.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_ES8388_REGDUMP
struct es8388_regdump_s
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

#ifdef CONFIG_ES8388_REGDUMP
static const struct es8388_regdump_s g_es8388_debug[] =
{
  {"Chip Control 1",            ES8388_CONTROL1      },
  {"Chip Control 2",            ES8388_CONTROL2      },
  {"Chip Power Management",     ES8388_CHIPPOWER     },
  {"ADC Power Management",      ES8388_ADCPOWER      },
  {"DAC Power Management",      ES8388_DACPOWER      },
  {"Chip Low Power 1",          ES8388_CHIPLOPOW1    },
  {"Chip Low Power 2",          ES8388_CHIPLOPOW2    },
  {"Analog Volume Management",  ES8388_ANAVOLMANAG   },
  {"Master Mode Control",       ES8388_MASTERMODE    },
  {"ADC Control 1",             ES8388_ADCCONTROL1   },
  {"ADC Control 2",             ES8388_ADCCONTROL2   },
  {"ADC Control 3",             ES8388_ADCCONTROL3   },
  {"ADC Control 4",             ES8388_ADCCONTROL4   },
  {"ADC Control 5",             ES8388_ADCCONTROL5   },
  {"ADC Control 6",             ES8388_ADCCONTROL6   },
  {"ADC Control 7",             ES8388_ADCCONTROL7   },
  {"ADC Control 8",             ES8388_ADCCONTROL8   },
  {"ADC Control 9",             ES8388_ADCCONTROL9   },
  {"ADC Control 10",            ES8388_ADCCONTROL10  },
  {"ADC Control 11",            ES8388_ADCCONTROL11  },
  {"ADC Control 12",            ES8388_ADCCONTROL12  },
  {"ADC Control 13",            ES8388_ADCCONTROL13  },
  {"ADC Control 14",            ES8388_ADCCONTROL14  },
  {"DAC Control 1",             ES8388_DACCONTROL1   },
  {"DAC Control 2",             ES8388_DACCONTROL2   },
  {"DAC Control 3",             ES8388_DACCONTROL3   },
  {"DAC Control 4",             ES8388_DACCONTROL4   },
  {"DAC Control 5",             ES8388_DACCONTROL5   },
  {"DAC Control 6",             ES8388_DACCONTROL6   },
  {"DAC Control 7",             ES8388_DACCONTROL7   },
  {"DAC Control 8",             ES8388_DACCONTROL8   },
  {"DAC Control 9",             ES8388_DACCONTROL9   },
  {"DAC Control 10",            ES8388_DACCONTROL10  },
  {"DAC Control 11",            ES8388_DACCONTROL11  },
  {"DAC Control 12",            ES8388_DACCONTROL12  },
  {"DAC Control 13",            ES8388_DACCONTROL13  },
  {"DAC Control 14",            ES8388_DACCONTROL14  },
  {"DAC Control 15",            ES8388_DACCONTROL15  },
  {"DAC Control 16",            ES8388_DACCONTROL16  },
  {"DAC Control 17",            ES8388_DACCONTROL17  },
  {"DAC Control 18",            ES8388_DACCONTROL18  },
  {"DAC Control 19",            ES8388_DACCONTROL19  },
  {"DAC Control 20",            ES8388_DACCONTROL20  },
  {"DAC Control 21",            ES8388_DACCONTROL21  },
  {"DAC Control 22",            ES8388_DACCONTROL22  },
  {"DAC Control 23",            ES8388_DACCONTROL23  },
  {"DAC Control 24",            ES8388_DACCONTROL24  },
  {"DAC Control 25",            ES8388_DACCONTROL25  },
  {"DAC Control 26",            ES8388_DACCONTROL26  },
  {"DAC Control 27",            ES8388_DACCONTROL27  },
  {"DAC Control 28",            ES8388_DACCONTROL28  },
  {"DAC Control 29",            ES8388_DACCONTROL29  },
  {"DAC Control 30",            ES8388_DACCONTROL30  },
};

#  define ES8388_NREGISTERS (sizeof(g_es8388_debug) / sizeof(struct es8388_regdump_s))
#endif /* CONFIG_ES8388_REGDUMP */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: es8388_dump_registers
 *
 * Description:
 *   Dump the contents of all ES8388 registers to the syslog device
 *
 * Input Parameters:
 *   dev - The device instance returned by es8388_initialize
 *   msg - Message to appear before registers
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ES8388_REGDUMP
void es8388_dump_registers(FAR struct audio_lowerhalf_s *dev,
                           FAR const char *msg)
{
  int i;

  syslog(LOG_INFO, "ES8388 Registers: %s\n", msg);
  for (i = 0; i < ES8388_NREGISTERS; i++)
    {
      syslog(LOG_INFO, "    %s[%02x]: %02x\n",
             g_es8388_debug[i].regname, g_es8388_debug[i].regaddr,
             es8388_readreg((FAR struct es8388_dev_s *)dev,
                            g_es8388_debug[i].regaddr));
    }
}
#endif /* CONFIG_ES8388_REGDUMP */
