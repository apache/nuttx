/****************************************************************************
 * arch/arm/src/rp2040/rp2040_i2s_pio.c
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
#include <stdbool.h>

#include <arch/board/board.h>

#include "rp2040_i2s_pio.h"
#include "rp2040_pio.h"
#include "rp2040_pio_instructions.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_RP2040_I2S_PIO
  #define CONFIG_RP2040_I2S_PIO     0
#endif

#ifndef  CONFIG_RP2040_I2S_PIO_SM
  #define CONFIG_RP2040_I2S_PIO_SM  0
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rp2040_i2s_pio_config
{
  const rp2040_pio_program_t program;
  uint32_t entry;
  uint32_t wrap_target;
  uint32_t wrap;
  bool autopull;
  uint32_t clocks;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* PIO program for 16bit stereo I2S transfer */

static const uint16_t pio_program_i2s_16s[] =
  {
            /*     .wrap_target                       */

    0x6870, /*  0: out    null, 16        side 1      */
    0x6001, /*  1: out    pins, 1         side 0      */
    0xe82d, /*  2: set    x, 13           side 1      */
    0x6001, /*  3: out    pins, 1         side 0      */
    0x0843, /*  4: jmp    x--, 3          side 1      */
    0x7001, /*  5: out    pins, 1         side 2      */
    0x7870, /*  6: out    null, 16        side 3      */
    0x7001, /*  7: out    pins, 1         side 2      */
    0xf82d, /*  8: set    x, 13           side 3      */
    0x7001, /*  9: out    pins, 1         side 2      */
    0x1849, /* 10: jmp    x--, 9          side 3      */
    0x6001, /* 11: out    pins, 1         side 0      */

            /*     .wrap                              */
  };

/* PIO program for 16bit mono I2S transfer */

static const uint16_t pio_program_i2s_16m[] =
  {
            /*     .wrap_target                       */

    0x80a0, /*  0: pull   block           side 0      */
    0x6870, /*  1: out    null, 16        side 1      */
    0xa847, /*  2: mov    y, osr          side 1      */
    0x6101, /*  3: out    pins, 1         side 0 [1]  */
    0xe92d, /*  4: set    x, 13           side 1 [1]  */
    0x6101, /*  5: out    pins, 1         side 0 [1]  */
    0x0945, /*  6: jmp    x--, 5          side 1 [1]  */
    0x7101, /*  7: out    pins, 1         side 2 [1]  */
    0xb9e2, /*  8: mov    osr, y          side 3 [1]  */
    0x7101, /*  9: out    pins, 1         side 2 [1]  */
    0xf92d, /* 10: set    x, 13           side 3 [1]  */
    0x7101, /* 11: out    pins, 1         side 2 [1]  */
    0x194b, /* 12: jmp    x--, 11         side 3 [1]  */
    0x6001, /* 13: out    pins, 1         side 0      */

            /*     .wrap                              */
  };

/* PIO program for 8bit stereo I2S transfer */

static const uint16_t pio_program_i2s_8s[] =
  {
            /*     .wrap_target                       */

    0x80a0, /*  0: pull   block           side 0      */
    0x6078, /*  1: out    null, 24        side 0      */
    0xa9ef, /*  2: mov    osr, !osr       side 1 [1]  */
    0x6101, /*  3: out    pins, 1         side 0 [1]  */
    0xa8ef, /*  4: mov    osr, !osr       side 1      */
    0xe826, /*  5: set    x, 6            side 1      */
    0x6101, /*  6: out    pins, 1         side 0 [1]  */
    0x0946, /*  7: jmp    x--, 6          side 1 [1]  */
    0xe100, /*  8: set    pins, 0         side 0 [1]  */
    0xe925, /*  9: set    x, 5            side 1 [1]  */
    0xa142, /* 10: nop                    side 0 [1]  */
    0x094a, /* 11: jmp    x--, 10         side 1 [1]  */
    0x90a0, /* 12: pull   block           side 2      */
    0x7078, /* 13: out    null, 24        side 2      */
    0xb9ef, /* 14: mov    osr, !osr       side 3 [1]  */
    0x7101, /* 15: out    pins, 1         side 2 [1]  */
    0xb8ef, /* 16: mov    osr, !osr       side 3      */
    0xf826, /* 17: set    x, 6            side 3      */
    0x7101, /* 18: out    pins, 1         side 2 [1]  */
    0x1952, /* 19: jmp    x--, 18         side 3 [1]  */
    0xf100, /* 20: set    pins, 0         side 2 [1]  */
    0xf925, /* 21: set    x, 5            side 3 [1]  */
    0xb142, /* 22: nop                    side 2 [1]  */
    0x1956, /* 23: jmp    x--, 22         side 3 [1]  */

            /*     .wrap                              */
  };

/* PIO program for 8bit mono I2S transfer */

static const uint16_t pio_program_i2s_8m[] =
  {
            /*     .wrap_target                       */

    0x80a0, /*  0: pull   block           side 0      */
    0x6078, /*  1: out    null, 24        side 0      */
    0xa8ef, /*  2: mov    osr, !osr       side 1      */
    0xa847, /*  3: mov    y, osr          side 1      */
    0x6101, /*  4: out    pins, 1         side 0 [1]  */
    0xa8ef, /*  5: mov    osr, !osr       side 1      */
    0xe826, /*  6: set    x, 6            side 1      */
    0x6101, /*  7: out    pins, 1         side 0 [1]  */
    0x0947, /*  8: jmp    x--, 7          side 1 [1]  */
    0xe100, /*  9: set    pins, 0         side 0 [1]  */
    0xe925, /* 10: set    x, 5            side 1 [1]  */
    0xa142, /* 11: nop                    side 0 [1]  */
    0x094b, /* 12: jmp    x--, 11         side 1 [1]  */
    0xb142, /* 13: nop                    side 2 [1]  */
    0xb9e2, /* 14: mov    osr, y          side 3 [1]  */
    0x7101, /* 15: out    pins, 1         side 2 [1]  */
    0xb8ef, /* 16: mov    osr, !osr       side 3      */
    0xf826, /* 17: set    x, 6            side 3      */
    0x7101, /* 18: out    pins, 1         side 2 [1]  */
    0x1952, /* 19: jmp    x--, 18         side 3 [1]  */
    0xf100, /* 20: set    pins, 0         side 2 [1]  */
    0xf925, /* 21: set    x, 5            side 3 [1]  */
    0xb142, /* 22: nop                    side 2 [1]  */
    0x1956, /* 23: jmp    x--, 22         side 3 [1]  */

            /*     .wrap                              */
  };

/* PIO configuration table */

static const struct rp2040_i2s_pio_config g_pio_i2s_configs[] =
  {
    [RP2040_I2S_PIO_16BIT_STEREO] =
      {
        {
          pio_program_i2s_16s,
          sizeof(pio_program_i2s_16s) / sizeof(uint16_t),
          -1
        },
        0, 0, 11,
        true, 16 * 2 * 2
      },

    [RP2040_I2S_PIO_16BIT_MONO] =
      {
        {
          pio_program_i2s_16m,
          sizeof(pio_program_i2s_16m) / sizeof(uint16_t),
          -1
        },
        0, 0, 13,
        false, 16 * 2 * 4
      },

    [RP2040_I2S_PIO_8BIT_STEREO] =
      {
        {
          pio_program_i2s_8s,
          sizeof(pio_program_i2s_8s) / sizeof(uint16_t),
          -1
        },
        0, 0, 23,
        false, 16 * 2 * 4
      },

    [RP2040_I2S_PIO_8BIT_MONO] =
      {
        {
          pio_program_i2s_8m,
          sizeof(pio_program_i2s_8m) / sizeof(uint16_t),
          -1
        },
        0, 0, 23,
        false, 16 * 2 * 4
      }
  };

static const uint32_t g_i2s_pio = CONFIG_RP2040_I2S_PIO;
static const uint32_t g_i2s_pio_sm = CONFIG_RP2040_I2S_PIO_SM;

/* PIO I2S status */

static int g_pio_current_mode = -1;
static uint32_t g_pio_current_samplerate;
static uint32_t g_pio_current_offset;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static float get_clkdiv(int mode, uint32_t samplerate)
{
  float div = (float)BOARD_SYS_FREQ /
              (samplerate * g_pio_i2s_configs[mode].clocks);

  return div;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp2040_i2s_pio_configure
 *
 * Description:
 *   Configure RP2040 PIO for I2S
 *
 ****************************************************************************/

int rp2040_i2s_pio_configure(int mode, uint32_t samplerate)
{
  const struct rp2040_i2s_pio_config *conf;
  rp2040_pio_sm_config sm_config;

  uint32_t data_pin = CONFIG_RP2040_I2S_DATA;
  uint32_t clock_pin_base = CONFIG_RP2040_I2S_CLOCK;
  uint32_t pin_mask = (1u << data_pin) | (3u << clock_pin_base);

  /* Check parameters */

  if (mode < 0 || mode >= RP2040_I2S_PIO_MAX_MODE ||
      samplerate == 0)
    {
      return -1;
    }

  if (mode == g_pio_current_mode)
    {
      if (samplerate == g_pio_current_samplerate)
        {
          return 0;
        }
      else
        {
          /* Only changing the sampling rate */

          rp2040_pio_sm_set_clkdiv(g_i2s_pio, g_i2s_pio_sm,
                                   get_clkdiv(mode, samplerate));
          rp2040_pio_sm_clkdiv_restart(g_i2s_pio, g_i2s_pio_sm);
          return 0;
        }
    }

  if (g_pio_current_mode < 0)
    {
      /* Claim to use PIO state machine for I2S */

      rp2040_pio_sm_claim(g_i2s_pio, g_i2s_pio_sm);
    }
  else
    {
      /* Remove existing PIO program to change the I2S mode */

      rp2040_pio_remove_program(CONFIG_RP2040_I2S_PIO,
                          &g_pio_i2s_configs[g_pio_current_mode].program,
                          g_pio_current_offset);
    }

  /* Program the PIO */

  conf = &g_pio_i2s_configs[mode];
  g_pio_current_offset = rp2040_pio_add_program(CONFIG_RP2040_I2S_PIO,
                          &conf->program);
  g_pio_current_mode = mode;

  /* Configure the state machine */

  sm_config = rp2040_pio_get_default_sm_config();
  rp2040_sm_config_set_wrap(&sm_config,
                            g_pio_current_offset + conf->wrap_target,
                            g_pio_current_offset + conf->wrap);
  rp2040_sm_config_set_sideset(&sm_config, 2, false, false);

  rp2040_sm_config_set_out_pins(&sm_config, data_pin, 1);
  rp2040_sm_config_set_sideset_pins(&sm_config, clock_pin_base);
  rp2040_sm_config_set_out_shift(&sm_config, false, conf->autopull, 32);
  rp2040_sm_config_set_set_pins(&sm_config, data_pin, 1);
  rp2040_sm_config_set_clkdiv(&sm_config, get_clkdiv(mode, samplerate));
  rp2040_pio_sm_init(g_i2s_pio, g_i2s_pio_sm,
                     g_pio_current_offset, &sm_config);

  rp2040_pio_sm_set_pindirs_with_mask(g_i2s_pio, g_i2s_pio_sm,
                                      pin_mask, pin_mask);
  rp2040_pio_sm_set_pins(g_i2s_pio, g_i2s_pio_sm, 1); /* clear pins */
  rp2040_pio_sm_exec(g_i2s_pio, g_i2s_pio_sm,
                     pio_encode_jmp(g_pio_current_offset + conf->entry));

  return 0;
}

/****************************************************************************
 * Name: rp2040_i2s_pio_enable
 *
 * Description:
 *   Set enable I2S transfer
 *
 ****************************************************************************/

void rp2040_i2s_pio_enable(bool enable)
{
  rp2040_pio_sm_set_enabled(g_i2s_pio, g_i2s_pio_sm, enable);
}

/****************************************************************************
 * Name: rp2040_i2s_pio_getdmaaddr
 *
 * Description:
 *   Get DMA peripheral address for I2S transfer
 *
 ****************************************************************************/

uintptr_t rp2040_i2s_pio_getdmaaddr(void)
{
  return RP2040_PIO_TXF(g_i2s_pio, g_i2s_pio_sm);
}

/****************************************************************************
 * Name: rp2040_i2s_pio_getdmaaddr
 *
 * Description:
 *   Get DMA peripheral address for I2S transfer
 *
 ****************************************************************************/

uint8_t rp2040_i2s_pio_getdreq(void)
{
  return RP2040_DMA_DREQ_PIO0_TX0 + g_i2s_pio_sm + g_i2s_pio * 8;
}
