/****************************************************************************
 * drivers/audio/wm8904_debug.c
 *
 * Audio device driver for Wolfson Microelectronics WM8904 Audio codec.
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 * - "WM8904 Ultra Low Power CODEC for Portable Audio Applications, Pre-
 *    Production", September 2012, Rev 3.3, Wolfson Microelectronics
 *
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
#include <syslog.h>

#include <nuttx/audio/audio.h>

#include "wm8904.h"

#ifdef CONFIG_WM8904_REGDUMP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct wm8904_debug_s
{
  FAR const char *regname;
  uint8_t regaddr;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct wm8904_debug_s g_wm8904_debug[] =
{
  {"ID",              WM8904_ID              },
  {"BIAS_CTRL",       WM8904_BIAS_CTRL       },
  {"VMID_CTRL",       WM8904_VMID_CTRL       },
  {"MIC_BIAS_CTRL0",  WM8904_MIC_BIAS_CTRL0  },
  {"MIC_BIAS_CTRL1",  WM8904_MIC_BIAS_CTRL1  },
  {"ANALOG_ADC",      WM8904_ANALOG_ADC      },
  {"PM0",             WM8904_PM0             },
  {"PM2",             WM8904_PM2             },
  {"PM3",             WM8904_PM3             },
  {"PM6",             WM8904_PM6             },
  {"CLKRATE0",        WM8904_CLKRATE0        },
  {"CLKRATE1",        WM8904_CLKRATE1        },
  {"CLKRATE2",        WM8904_CLKRATE2        },
  {"AIF0",            WM8904_AIF0            },
  {"AIF1",            WM8904_AIF1            },
  {"AIF2",            WM8904_AIF2            },
  {"AIF3",            WM8904_AIF3            },
  {"DAC_VOL_LEFT",    WM8904_DAC_VOL_LEFT    },
  {"DAC_VOL_RIGHT",   WM8904_DAC_VOL_RIGHT   },
  {"DAC_DIGI0",       WM8904_DAC_DIGI0       },
  {"DAC_DIGI1",       WM8904_DAC_DIGI1       },
  {"ADC_VOL_LEFT",    WM8904_ADC_VOL_LEFT    },
  {"ADC_VOL_RIGHT",   WM8904_ADC_VOL_RIGHT   },
  {"ADC_DIGI",        WM8904_ADC_DIGI        },
  {"MIC_DIGI",        WM8904_MIC_DIGI        },
  {"DRC0",            WM8904_DRC0            },
  {"DRC1",            WM8904_DRC1            },
  {"DRC2",            WM8904_DRC2            },
  {"DRC3",            WM8904_DRC3            },
  {"ANA_LEFT_IN0",    WM8904_ANA_LEFT_IN0    },
  {"ANA_RIGHT_IN0",   WM8904_ANA_RIGHT_IN0   },
  {"ANA_LEFT_IN1",    WM8904_ANA_LEFT_IN1    },
  {"ANA_RIGHT_IN1",   WM8904_ANA_RIGHT_IN1   },
  {"ANA_LEFT_OUT1",   WM8904_ANA_LEFT_OUT1   },
  {"ANA_RIGHT_OUT1",  WM8904_ANA_RIGHT_OUT1  },
  {"ANA_LEFT_OUT2",   WM8904_ANA_LEFT_OUT2   },
  {"ANA_RIGHT_OUT2",  WM8904_ANA_RIGHT_OUT2  },
  {"ANA_OUT12_ZC",    WM8904_ANA_OUT12_ZC    },
  {"DC_SERVO0",       WM8904_DC_SERVO0       },
  {"DC_SERVO1",       WM8904_DC_SERVO1       },
  {"DC_SERVO2",       WM8904_DC_SERVO2       },
  {"DC_SERVO4",       WM8904_DC_SERVO4       },
  {"DC_SERVO5",       WM8904_DC_SERVO5       },
  {"DC_SERVO6",       WM8904_DC_SERVO6       },
  {"DC_SERVO7",       WM8904_DC_SERVO7       },
  {"DC_SERVO8",       WM8904_DC_SERVO8       },
  {"DC_SERVO9",       WM8904_DC_SERVO9       },
  {"DC_SERVO_RDBACK", WM8904_DC_SERVO_RDBACK },
  {"ANA_HP0",         WM8904_ANA_HP0         },
  {"ANA_LINEOUT0",    WM8904_ANA_LINEOUT0    },
  {"CHG_PUMP0",       WM8904_CHG_PUMP0       },
  {"CLASS_W0",        WM8904_CLASS_W0        },
  {"WR_SEQ0",         WM8904_WR_SEQ0         },
  {"WR_SEQ1",         WM8904_WR_SEQ1         },
  {"WR_SEQ2",         WM8904_WR_SEQ2         },
  {"WR_SEQ3",         WM8904_WR_SEQ3         },
  {"WR_SEQ4",         WM8904_WR_SEQ4         },
  {"FLL_CTRL1",       WM8904_FLL_CTRL1       },
  {"FLL_CTRL2",       WM8904_FLL_CTRL2       },
  {"FLL_CTRL3",       WM8904_FLL_CTRL3       },
  {"FLL_CTRL4",       WM8904_FLL_CTRL4       },
  {"FLL_CTRL5",       WM8904_FLL_CTRL5       },
  {"GPIO_CTRL1",      WM8904_GPIO_CTRL1      },
  {"GPIO_CTRL2",      WM8904_GPIO_CTRL2      },
  {"GPIO_CTRL3",      WM8904_GPIO_CTRL3      },
  {"GPIO_CTRL4",      WM8904_GPIO_CTRL4      },
  {"DIGI_PULLS",      WM8904_DIGI_PULLS      },
  {"INT_STATUS",      WM8904_INT_STATUS      },
  {"INT_MASK",        WM8904_INT_MASK        },
  {"INT_POL",         WM8904_INT_POL         },
  {"INT_DEBOUNCE",    WM8904_INT_DEBOUNCE    },
  {"EQ1",             WM8904_EQ1             },
  {"EQ2",             WM8904_EQ2             },
  {"EQ3",             WM8904_EQ3             },
  {"EQ4",             WM8904_EQ4             },
  {"EQ5",             WM8904_EQ5             },
  {"EQ6",             WM8904_EQ6             },
  {"EQ7",             WM8904_EQ7             },
  {"EQ8",             WM8904_EQ8             },
  {"EQ9",             WM8904_EQ9             },
  {"EQ10",            WM8904_EQ10            },
  {"EQ11",            WM8904_EQ11            },
  {"EQ12",            WM8904_EQ12            },
  {"EQ13",            WM8904_EQ13            },
  {"EQ14",            WM8904_EQ14            },
  {"EQ15",            WM8904_EQ15            },
  {"EQ16",            WM8904_EQ16            },
  {"EQ17",            WM8904_EQ17            },
  {"EQ18",            WM8904_EQ18            },
  {"EQ19",            WM8904_EQ19            },
  {"EQ20",            WM8904_EQ20            },
  {"EQ21",            WM8904_EQ21            },
  {"EQ22",            WM8904_EQ22            },
  {"EQ23",            WM8904_EQ23            },
  {"EQ24",            WM8904_EQ24            },
  {"ADC_TEST",        WM8904_ADC_TEST        },
  {"FLL_NCO_TEST0",   WM8904_FLL_NCO_TEST0   },
  {"FLL_NCO_TEST1",   WM8904_FLL_NCO_TEST1   }
};

#define WM8904_NREGISTERS (sizeof(g_wm8904_debug)/sizeof(struct wm8904_debug_s))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wm8904_dump_registers
 *
 * Description:
 *   Dump the contents of all WM8904 registers to the syslog device
 *
 * Input Parameters:
 *   dev - The device instance returned by wm8904_initialize
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void wm8904_dump_registers(FAR struct audio_lowerhalf_s *dev,
                           FAR const char *msg)
{
  int i;

  syslog("WM8904 Registers: %s\n", msg);
  for (i = 0; i < WM8904_NREGISTERS; i++)
    {
      syslog("%16s[%02x]: %04x\n",
             g_wm8904_debug[i].regname, g_wm8904_debug[i].regaddr,
             wm8904_readreg((FAR struct wm8904_dev_s *)dev,
                            g_wm8904_debug[i].regaddr));
    }
}

#endif /* CONFIG_WM8904_REGDUMP */
