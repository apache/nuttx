/****************************************************************************
 * arch/arm/src/samd2l2/sam_usart.c
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

#include "sam_pinmap.h"
#include "sam_gclk.h"
#include "sam_usart.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef SAMD2L2_HAVE_USART0
const struct sam_usart_config_s g_usart0config =
{
  .sercom    = 0,
  .parity    = CONFIG_USART0_PARITY,
  .bits      = CONFIG_USART0_BITS,
  .irq       = SAM_IRQ_SERCOM0,
  .gclkgen   = BOARD_SERCOM0_GCLKGEN,
  .slowgen   = BOARD_SERCOM0_SLOW_GCLKGEN,
  .stopbits2 = CONFIG_USART0_2STOP,
  .baud      = CONFIG_USART0_BAUD,
  .pad0      = BOARD_SERCOM0_PINMAP_PAD0,
  .pad1      = BOARD_SERCOM0_PINMAP_PAD1,
  .pad2      = BOARD_SERCOM0_PINMAP_PAD2,
  .pad3      = BOARD_SERCOM0_PINMAP_PAD3,
  .muxconfig = BOARD_SERCOM0_MUXCONFIG,
  .frequency = BOARD_SERCOM0_FREQUENCY,
  .base      = SAM_SERCOM0_BASE,
};
#endif

#ifdef SAMD2L2_HAVE_USART1
const struct sam_usart_config_s g_usart1config =
{
  .sercom    = 1,
  .parity    = CONFIG_USART1_PARITY,
  .bits      = CONFIG_USART1_BITS,
  .irq       = SAM_IRQ_SERCOM1,
  .gclkgen   = BOARD_SERCOM1_GCLKGEN,
  .slowgen   = BOARD_SERCOM1_SLOW_GCLKGEN,
  .stopbits2 = CONFIG_USART1_2STOP,
  .baud      = CONFIG_USART1_BAUD,
  .pad0      = BOARD_SERCOM1_PINMAP_PAD0,
  .pad1      = BOARD_SERCOM1_PINMAP_PAD1,
  .pad2      = BOARD_SERCOM1_PINMAP_PAD2,
  .pad3      = BOARD_SERCOM1_PINMAP_PAD3,
  .muxconfig = BOARD_SERCOM1_MUXCONFIG,
  .frequency = BOARD_SERCOM1_FREQUENCY,
  .base      = SAM_SERCOM1_BASE,
};
#endif

#ifdef SAMD2L2_HAVE_USART2
const struct sam_usart_config_s g_usart2config =
{
  .sercom    = 2,
  .parity    = CONFIG_USART2_PARITY,
  .bits      = CONFIG_USART2_BITS,
  .irq       = SAM_IRQ_SERCOM2,
  .gclkgen   = BOARD_SERCOM2_GCLKGEN,
  .slowgen   = BOARD_SERCOM2_SLOW_GCLKGEN,
  .stopbits2 = CONFIG_USART2_2STOP,
  .baud      = CONFIG_USART2_BAUD,
  .pad0      = BOARD_SERCOM2_PINMAP_PAD0,
  .pad1      = BOARD_SERCOM2_PINMAP_PAD1,
  .pad2      = BOARD_SERCOM2_PINMAP_PAD2,
  .pad3      = BOARD_SERCOM2_PINMAP_PAD3,
  .muxconfig = BOARD_SERCOM2_MUXCONFIG,
  .frequency = BOARD_SERCOM2_FREQUENCY,
  .base      = SAM_SERCOM2_BASE,
};
#endif

#ifdef SAMD2L2_HAVE_USART3
const struct sam_usart_config_s g_usart3config =
{
  .sercom    = 3,
  .parity    = CONFIG_USART3_PARITY,
  .bits      = CONFIG_USART3_BITS,
  .irq       = SAM_IRQ_SERCOM3,
  .gclkgen   = BOARD_SERCOM3_GCLKGEN,
  .slowgen   = BOARD_SERCOM3_SLOW_GCLKGEN,
  .stopbits2 = CONFIG_USART3_2STOP,
  .baud      = CONFIG_USART3_BAUD,
  .pad0      = BOARD_SERCOM3_PINMAP_PAD0,
  .pad1      = BOARD_SERCOM3_PINMAP_PAD1,
  .pad2      = BOARD_SERCOM3_PINMAP_PAD2,
  .pad3      = BOARD_SERCOM3_PINMAP_PAD3,
  .muxconfig = BOARD_SERCOM3_MUXCONFIG,
  .frequency = BOARD_SERCOM3_FREQUENCY,
  .base      = SAM_SERCOM3_BASE,
};
#endif

#ifdef SAMD2L2_HAVE_USART4
const struct sam_usart_config_s g_usart4config =
{
  .sercom    = 4,
  .parity    = CONFIG_USART4_PARITY,
  .bits      = CONFIG_USART4_BITS,
  .irq       = SAM_IRQ_SERCOM4,
  .gclkgen   = BOARD_SERCOM4_GCLKGEN,
  .slowgen   = BOARD_SERCOM4_SLOW_GCLKGEN,
  .stopbits2 = CONFIG_USART4_2STOP,
  .baud      = CONFIG_USART4_BAUD,
  .pad0      = BOARD_SERCOM4_PINMAP_PAD0,
  .pad1      = BOARD_SERCOM4_PINMAP_PAD1,
  .pad2      = BOARD_SERCOM4_PINMAP_PAD2,
  .pad3      = BOARD_SERCOM4_PINMAP_PAD3,
  .muxconfig = BOARD_SERCOM4_MUXCONFIG,
  .frequency = BOARD_SERCOM4_FREQUENCY,
  .base      = SAM_SERCOM4_BASE,
};
#endif

#ifdef SAMD2L2_HAVE_USART5
const struct sam_usart_config_s g_usart5config =
{
  .sercom    = 5,
  .parity    = CONFIG_USART5_PARITY,
  .bits      = CONFIG_USART5_BITS,
  .irq       = SAM_IRQ_SERCOM5,
  .gclkgen   = BOARD_SERCOM5_GCLKGEN,
  .slowgen   = BOARD_SERCOM5_SLOW_GCLKGEN,
  .stopbits2 = CONFIG_USART5_2STOP,
  .baud      = CONFIG_USART5_BAUD,
  .pad0      = BOARD_SERCOM5_PINMAP_PAD0,
  .pad1      = BOARD_SERCOM5_PINMAP_PAD1,
  .pad2      = BOARD_SERCOM5_PINMAP_PAD2,
  .pad3      = BOARD_SERCOM5_PINMAP_PAD3,
  .muxconfig = BOARD_SERCOM5_MUXCONFIG,
  .frequency = BOARD_SERCOM5_FREQUENCY,
  .base      = SAM_SERCOM5_BASE,
};
#endif

const struct sam_usart_config_s *g_usartconfig[SAMD2L2_NSERCOM] =
{
#if SAMD2L2_NSERCOM > 0
#ifdef SAMD2L2_HAVE_USART0
  &g_usart0config,
#else
  (const struct sam_usart_config_s *)0,
#endif
#endif

#if SAMD2L2_NSERCOM > 1
#ifdef SAMD2L2_HAVE_USART1
  &g_usart1config,
#else
  (const struct sam_usart_config_s *)0,
#endif
#endif

#if SAMD2L2_NSERCOM > 2
#ifdef SAMD2L2_HAVE_USART2
  &g_usart2config,
#else
  (const struct sam_usart_config_s *)0,
#endif
#endif

#if SAMD2L2_NSERCOM > 3
#ifdef SAMD2L2_HAVE_USART3
  &g_usart3config,
#else
  (const struct sam_usart_config_s *)0,
#endif
#endif

#if SAMD2L2_NSERCOM > 4
#ifdef SAMD2L2_HAVE_USART4
  &g_usart4config,
#else
  (const struct sam_usart_config_s *)0,
#endif
#endif

#if SAMD2L2_NSERCOM > 5
#ifdef SAMD2L2_HAVE_USART5
  &g_usart5config,
#else
  (const struct sam_usart_config_s *)0,
#endif
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/
