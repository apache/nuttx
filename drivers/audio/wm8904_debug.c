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
#include <assert.h>
#include <fixedmath.h>
#include <syslog.h>

#include <nuttx/audio/audio.h>
#include <nuttx/audio/wm8904.h>

#include "wm8904.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_WM8904_REGDUMP
struct wb8904_regdump_s
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

#ifdef CONFIG_WM8904_REGDUMP
static const struct wb8904_regdump_s g_wm8904_debug[] =
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

#  define WM8904_NREGISTERS (sizeof(g_wm8904_debug)/sizeof(struct wb8904_regdump_s))
#endif /* CONFIG_WM8904_REGDUMP */

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

#ifdef CONFIG_WM8904_REGDUMP
void wm8904_dump_registers(FAR struct audio_lowerhalf_s *dev,
                           FAR const char *msg)
{
  int i;

  syslog(LOG_INFO, "WM8904 Registers: %s\n", msg);
  for (i = 0; i < WM8904_NREGISTERS; i++)
    {
      syslog(LOG_INFO, "%16s[%02x]: %04x\n",
             g_wm8904_debug[i].regname, g_wm8904_debug[i].regaddr,
             wm8904_readreg((FAR struct wm8904_dev_s *)dev,
                            g_wm8904_debug[i].regaddr));
    }
}
#endif /* CONFIG_WM8904_REGDUMP */

/****************************************************************************
 * Name: wm8904_clock_analysis
 *
 * Description:
 *   Analyze the settings in the clock chain and dump to syslog.
 *
 * Input Parameters:
 *   dev - The device instance returned by wm8904_initialize
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_WM8904_CLKDEBUG
void wm8904_clock_analysis(FAR struct audio_lowerhalf_s *dev,
                           FAR const char *msg)
{
  FAR struct wm8904_dev_s *priv = (FAR struct wm8904_dev_s *)dev;
  uint32_t sysclk;
  uint32_t bclk;
  uint32_t lrclk;
  uint16_t regval;
  unsigned int tmp;
  double ftmp;

  syslog(LOG_INFO, "WM8904 Clock Analysis: %s\n", msg);
  DEBUGASSERT(priv && priv->lower);
  syslog(LOG_INFO, "  MCLK:            %lu Hz\n",
         (unsigned long)priv->lower->mclk);

  /* Is the SYSCLK source the FLL?  Or MCK? */

  regval = wm8904_readreg(priv, WM8904_CLKRATE2);
  if ((regval & WM8904_SYSCLK_SRC) == WM8904_SYSCLK_SRCMCLK)
    {
      /* The SYSCLK divider bypasses the FLL and takes its input
       * directly from MCLK.
       */

      sysclk = priv->lower->mclk;
      syslog(LOG_INFO, "  SYSCLK Source:   MCLK (%s)\n",
             (regval & WM8904_MCLK_INV) != 0 ? "inverted" : "not inverted");
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

      regval = wm8904_readreg(priv, WM8904_FLL_CTRL5);
      switch (regval & WM8904_FLL_CLK_REF_SRC_MASK)
        {
          case WM8904_FLL_CLK_REF_SRC_MCLK:
            syslog(LOG_INFO, "  FLL Source:      MCLK\n");
            break;

          case WM8904_FLL_CLK_REF_SRC_BCLK:
            syslog(LOG_INFO, "  ERROR: FLL source is BCLK: %04x\n",
                   regval);
            break;

          case WM8904_FLL_CLK_REF_SRC_LRCLK:
            syslog(LOG_INFO, "  ERROR: FLL source is LRCLK: %04x\n",
                   regval);
            break;

          default:
            syslog(LOG_INFO, "  ERROR: Unrecognized FLL source: %04x\n",
                   regval);
        }

      syslog(LOG_INFO, "  Fref:            %lu Hz (before divider)\n",
             fref);
      switch (regval & WM8904_FLL_CLK_REF_DIV_MASK)
        {
          case WM8904_FLL_CLK_REF_DIV1:
            syslog(LOG_INFO, "  FLL_CLK_REF_DIV: 1\n");
            break;

          case WM8904_FLL_CLK_REF_DIV2:
            syslog(LOG_INFO, "  FLL_CLK_REF_DIV: 2\n");
            fref >>= 1;
            break;

          case WM8904_FLL_CLK_REF_DIV4:
            syslog(LOG_INFO, "  FLL_CLK_REF_DIV: 4\n");
            fref >>= 2;
            break;

          case WM8904_FLL_CLK_REF_DIV8:
            syslog(LOG_INFO, "  FLL_CLK_REF_DIV: 8\n");
            fref >>= 3;
            break;
        }

      syslog(LOG_INFO, "  Fref:            %lu Hz (after divider)\n", fref);

      regval = wm8904_readreg(priv, WM8904_FLL_CTRL2);
      frndx = (regval & WM8904_FLL_FRATIO_MASK) >>
                        WM8904_FLL_FRATIO_SHIFT;
      tmp = (regval & WM8904_FLL_CTRL_RATE_MASK) >>
                      WM8904_FLL_CTRL_RATE_SHIFT;
      outdiv = ((regval & WM8904_FLL_OUTDIV_MASK) >>
                          WM8904_FLL_OUTDIV_SHIFT) + 1;

      syslog(LOG_INFO, "  FLL_CTRL_RATE:   Fvco / %u\n", tmp + 1);

      regval = wm8904_readreg(priv, WM8904_FLL_CTRL4);
      flln   = (regval & WM8904_FLL_N_MASK) >> WM8904_FLL_N_SHIFT;
      tmp    = (regval & WM8904_FLL_GAIN_MASK) >> WM8904_FLL_GAIN_SHIFT;

      syslog(LOG_INFO, "  FLL_GAIN:        %u\n", (1 << tmp));

      fllk   = wm8904_readreg(priv, WM8904_FLL_CTRL3);
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

      regval = wm8904_readreg(priv, WM8904_FLL_CTRL1);

      syslog(LOG_INFO, "  FLL_FRACN_ENA:   %s\n",
             (regval & WM8904_FLL_FRACN_ENA) != 0 ? "Enabled" : "Disabled");
      syslog(LOG_INFO, "  FLL_OSC_ENA:     %s\n",
             (regval & WM8904_FLL_OSC_ENA) != 0 ? "Enabled" : "Disabled");
      syslog(LOG_INFO, "  FLL_ENA:         %s\n",
             (regval & WM8904_FLL_ENA) != 0 ? "Enabled" : "Disabled");

      if ((regval & WM8904_FLL_ENA) == 0)
        {
          syslog(LOG_INFO, "  No SYSCLK\n");
          return;
        }

      sysclk = fout;
    }

  syslog(LOG_INFO, "  SYSCLK:          %lu Hz (before divider)\n",
         (unsigned long)sysclk);

  regval = wm8904_readreg(priv, WM8904_CLKRATE0);
  if ((regval & WM8904_MCLK_DIV) == WM8904_MCLK_DIV1)
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

  regval = wm8904_readreg(priv, WM8904_CLKRATE2);

  syslog(LOG_INFO, "  CLK_SYS_ENA:     %s\n",
         (regval & WM8904_CLK_SYS_ENA) != 0 ? "Enabled" : "Disabled");

  if ((regval & WM8904_CLK_SYS_ENA) == 0)
    {
      syslog(LOG_INFO, "  No SYSCLK\n");
      return;
    }

  regval = wm8904_readreg(priv, WM8904_AIF2);
  tmp    = (regval & WM8904_BCLK_DIV_MASK) >> WM8904_BCLK_DIV_SHIFT;
  tmp    = g_sysclk_scaleb1[tmp];
  ftmp   = (double)tmp / 2.0;

  syslog(LOG_INFO, "  BCLK_DIV:        SYSCLK / %u.%01u\n",
         (unsigned int)(tmp >> 1), (unsigned int)(5 * (tmp & 1)));

  bclk = (uint32_t)(sysclk / ftmp);

  syslog(LOG_INFO, "  BCLK:            %lu Hz\n", (unsigned long)bclk);

  regval = wm8904_readreg(priv, WM8904_AIF1);
  syslog(LOG_INFO, "  BCLK_DIR:        %s\n",
         (regval & WM8904_BCLK_DIR) != 0 ? "Output" : "Input");

  regval = wm8904_readreg(priv, WM8904_AIF3);
  tmp = (regval & WM8904_LRCLK_RATE_MASK) >> WM8904_LRCLK_RATE_SHIFT;

  lrclk = bclk / tmp;

  syslog(LOG_INFO, "  LRCLK_RATE:      BCLK / %lu\n", (unsigned long)tmp);
  syslog(LOG_INFO, "  LRCLK:           %lu Hz\n", (unsigned long)lrclk);
  syslog(LOG_INFO, "  LRCLK_DIR:       %s\n",
         (regval & WM8904_LRCLK_DIR) != 0 ? "Output" : "Input");
}
#endif /* CONFIG_WM8904_CLKDEBUG */
