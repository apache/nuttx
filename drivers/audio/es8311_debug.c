/****************************************************************************
 * drivers/audio/es8311_debug.c
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

  /* The framework for this driver is based on ESP-ADF's ES8311 driver. */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <syslog.h>

#include <nuttx/audio/audio.h>
#include <nuttx/audio/es8311.h>

#include "es8311.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_ES8311_REGDUMP
struct es8311_regdump_s
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

#ifdef CONFIG_ES8311_REGDUMP
static const struct es8311_regdump_s g_es8311_debug[] =
{
  {"ES8311_RESET_REG00",        ES8311_RESET_REG00        },
  {"ES8311_CLK_MANAGER_REG01",  ES8311_CLK_MANAGER_REG01  },
  {"ES8311_CLK_MANAGER_REG02",  ES8311_CLK_MANAGER_REG02  },
  {"ES8311_CLK_MANAGER_REG03",  ES8311_CLK_MANAGER_REG03  },
  {"ES8311_CLK_MANAGER_REG04",  ES8311_CLK_MANAGER_REG04  },
  {"ES8311_CLK_MANAGER_REG05",  ES8311_CLK_MANAGER_REG05  },
  {"ES8311_CLK_MANAGER_REG06",  ES8311_CLK_MANAGER_REG06  },
  {"ES8311_CLK_MANAGER_REG07",  ES8311_CLK_MANAGER_REG07  },
  {"ES8311_CLK_MANAGER_REG08",  ES8311_CLK_MANAGER_REG08  },
  {"ES8311_SDPIN_REG09",        ES8311_SDPIN_REG09        },
  {"ES8311_SDPOUT_REG0A",       ES8311_SDPOUT_REG0A       },
  {"ES8311_SYSTEM_REG0B",       ES8311_SYSTEM_REG0B       },
  {"ES8311_SYSTEM_REG0B",       ES8311_SYSTEM_REG0B       },
  {"ES8311_SYSTEM_REG0C",       ES8311_SYSTEM_REG0C       },
  {"ES8311_SYSTEM_REG0D",       ES8311_SYSTEM_REG0D       },
  {"ES8311_SYSTEM_REG0E",       ES8311_SYSTEM_REG0E       },
  {"ES8311_SYSTEM_REG0F",       ES8311_SYSTEM_REG0F       },
  {"ES8311_SYSTEM_REG10",       ES8311_SYSTEM_REG10       },
  {"ES8311_SYSTEM_REG11",       ES8311_SYSTEM_REG11       },
  {"ES8311_SYSTEM_REG12",       ES8311_SYSTEM_REG12       },
  {"ES8311_SYSTEM_REG13",       ES8311_SYSTEM_REG13       },
  {"ES8311_SYSTEM_REG14",       ES8311_SYSTEM_REG14       },
  {"ES8311_ADC_REG15",          ES8311_ADC_REG15          },
  {"ES8311_ADC_REG16",          ES8311_ADC_REG16          },
  {"ES8311_ADC_REG17",          ES8311_ADC_REG17          },
  {"ES8311_ADC_REG18",          ES8311_ADC_REG18          },
  {"ES8311_ADC_REG19",          ES8311_ADC_REG19          },
  {"ES8311_ADC_REG1A",          ES8311_ADC_REG1A          },
  {"ES8311_ADC_REG1B",          ES8311_ADC_REG1B          },
  {"ES8311_ADC_REG1C",          ES8311_ADC_REG1C          },
  {"ES8311_DAC_REG31",          ES8311_DAC_REG31          },
  {"ES8311_DAC_REG32",          ES8311_DAC_REG32          },
  {"ES8311_DAC_REG33",          ES8311_DAC_REG33          },
  {"ES8311_DAC_REG34",          ES8311_DAC_REG34          },
  {"ES8311_DAC_REG35",          ES8311_DAC_REG35          },
  {"ES8311_DAC_REG37",          ES8311_DAC_REG37          },
  {"ES8311_GPIO_REG44",         ES8311_GPIO_REG44         },
  {"ES8311_GP_REG45",           ES8311_GP_REG45           },
  {"ES8311_CHD1_REGFD",         ES8311_CHD1_REGFD         },
  {"ES8311_CHD2_REGFE",         ES8311_CHD2_REGFE         },
  {"ES8311_CHVER_REGFF",        ES8311_CHVER_REGFF        }
};

#  define ES8311_NREGISTERS (sizeof(g_es8311_debug) / sizeof(struct es8311_regdump_s))
#endif /* CONFIG_ES8311_REGDUMP */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: es8311_dump_registers
 *
 * Description:
 *   Dump the contents of all ES8311 registers to the syslog device
 *
 * Input Parameters:
 *   dev - The device instance returned by es8311_initialize
 *   msg - Message to appear before registers
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ES8311_REGDUMP
void es8311_dump_registers(FAR struct audio_lowerhalf_s *dev,
                           FAR const char *msg)
{
  int i;

  syslog(LOG_INFO, "ES8311 Registers: %s\n", msg);
  for (i = 0; i < ES8311_NREGISTERS; i++)
    {
      syslog(LOG_INFO, "    %s[%02x]: %02x\n",
             g_es8311_debug[i].regname, g_es8311_debug[i].regaddr,
             es8311_readreg((FAR struct es8311_dev_s *)dev,
                            g_es8311_debug[i].regaddr));
    }
}
#endif /* CONFIG_ES8311_REGDUMP */
