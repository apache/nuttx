/****************************************************************************
 * arch/arm/src/samd5e5/sam_usart.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <arch/irq.h>
#include <arch/samd5e5/chip.h>

#include "chip/sam_memorymap.h"
#include "chip/sam_pinmap.h"
#include "sam_gclk.h"
#include "sam_sercom.h"
#include "sam_usart.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef SAMD5E5_HAVE_USART0
const struct sam_usart_config_s g_usart0config =
{
  .sercom    = 0,
  .parity    = CONFIG_USART0_PARITY,
  .bits      = CONFIG_USART0_BITS,
  .txirq     = BOARD_TXIRQ_SERCOM0,
  .rxirq     = BOARD_RXIRQ_SERCOM0,
  .coregen   = BOARD_SERCOM0_COREGEN,
  .slowgen   = BOARD_SERCOM_SLOWGEN,
  .stopbits2 = CONFIG_USART0_2STOP,
  .corelock  = BOARD_SERCOM0_CORELOCK,
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

#ifdef SAMD5E5_HAVE_USART1
const struct sam_usart_config_s g_usart1config =
{
  .sercom    = 1,
  .parity    = CONFIG_USART1_PARITY,
  .bits      = CONFIG_USART1_BITS,
  .txirq     = BOARD_TXIRQ_SERCOM1,
  .rxirq     = BOARD_RXIRQ_SERCOM1,
  .coregen   = BOARD_SERCOM1_COREGEN,
  .slowgen   = BOARD_SERCOM_SLOWGEN,
  .stopbits2 = CONFIG_USART1_2STOP,
  .corelock  = BOARD_SERCOM1_CORELOCK,
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

#ifdef SAMD5E5_HAVE_USART2
const struct sam_usart_config_s g_usart2config =
{
  .sercom    = 2,
  .parity    = CONFIG_USART2_PARITY,
  .bits      = CONFIG_USART2_BITS,
  .txirq     = BOARD_TXIRQ_SERCOM2,
  .rxirq     = BOARD_RXIRQ_SERCOM2,
  .coregen   = BOARD_SERCOM2_COREGEN,
  .slowgen   = BOARD_SERCOM_SLOWGEN,
  .stopbits2 = CONFIG_USART2_2STOP,
  .corelock  = BOARD_SERCOM2_CORELOCK,
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

#ifdef SAMD5E5_HAVE_USART3
const struct sam_usart_config_s g_usart3config =
{
  .sercom    = 3,
  .parity    = CONFIG_USART3_PARITY,
  .bits      = CONFIG_USART3_BITS,
  .txirq     = BOARD_TXIRQ_SERCOM3,
  .rxirq     = BOARD_RXIRQ_SERCOM3,
  .coregen   = BOARD_SERCOM3_COREGEN,
  .slowgen   = BOARD_SERCOM_SLOWGEN,
  .stopbits2 = CONFIG_USART3_2STOP,
  .corelock  = BOARD_SERCOM3_CORELOCK,
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

#ifdef SAMD5E5_HAVE_USART4
const struct sam_usart_config_s g_usart4config =
{
  .sercom    = 4,
  .parity    = CONFIG_USART4_PARITY,
  .bits      = CONFIG_USART4_BITS,
  .txirq     = BOARD_TXIRQ_SERCOM4,
  .rxirq     = BOARD_RXIRQ_SERCOM4,
  .coregen   = BOARD_SERCOM4_COREGEN,
  .slowgen   = BOARD_SERCOM_SLOWGEN,
  .stopbits2 = CONFIG_USART4_2STOP,
  .corelock  = BOARD_SERCOM4_CORELOCK,
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

#ifdef SAMD5E5_HAVE_USART5
const struct sam_usart_config_s g_usart5config =
{
  .sercom    = 5,
  .parity    = CONFIG_USART5_PARITY,
  .bits      = CONFIG_USART5_BITS,
  .txirq     = BOARD_TXIRQ_SERCOM5,
  .rxirq     = BOARD_RXIRQ_SERCOM5,
  .coregen   = BOARD_SERCOM5_COREGEN,
  .slowgen   = BOARD_SERCOM_SLOWGEN,
  .stopbits2 = CONFIG_USART5_2STOP,
  .corelock  = BOARD_SERCOM5_CORELOCK,
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

#ifdef SAMD5E5_HAVE_USART6
const struct sam_usart_config_s g_usart6config =
{
  .sercom    = 6,
  .parity    = CONFIG_USART6_PARITY,
  .bits      = CONFIG_USART6_BITS,
  .txirq     = BOARD_TXIRQ_SERCOM6,
  .rxirq     = BOARD_RXIRQ_SERCOM6,
  .coregen   = BOARD_SERCOM6_COREGEN,
  .slowgen   = BOARD_SERCOM_SLOWGEN,
  .stopbits2 = CONFIG_USART6_2STOP,
  .corelock  = BOARD_SERCOM6_CORELOCK,
  .baud      = CONFIG_USART6_BAUD,
  .pad0      = BOARD_SERCOM6_PINMAP_PAD0,
  .pad1      = BOARD_SERCOM6_PINMAP_PAD1,
  .pad2      = BOARD_SERCOM6_PINMAP_PAD2,
  .pad3      = BOARD_SERCOM6_PINMAP_PAD3,
  .muxconfig = BOARD_SERCOM6_MUXCONFIG,
  .frequency = BOARD_SERCOM6_FREQUENCY,
  .base      = SAM_SERCOM6_BASE,
};
#endif

#ifdef SAMD5E5_HAVE_USART7
const struct sam_usart_config_s g_usart7config =
{
  .sercom    = 7,
  .parity    = CONFIG_USART7_PARITY,
  .bits      = CONFIG_USART7_BITS,
  .txirq     = BOARD_TXIRQ_SERCOM7,
  .rxirq     = BOARD_RXIRQ_SERCOM7,
  .coregen   = BOARD_SERCOM7_COREGEN,
  .slowgen   = BOARD_SERCOM_SLOWGEN,
  .stopbits2 = CONFIG_USART7_2STOP,
  .corelock  = BOARD_SERCOM7_CORELOCK,
  .baud      = CONFIG_USART7_BAUD,
  .pad0      = BOARD_SERCOM7_PINMAP_PAD0,
  .pad1      = BOARD_SERCOM7_PINMAP_PAD1,
  .pad2      = BOARD_SERCOM7_PINMAP_PAD2,
  .pad3      = BOARD_SERCOM7_PINMAP_PAD3,
  .muxconfig = BOARD_SERCOM7_MUXCONFIG,
  .frequency = BOARD_SERCOM7_FREQUENCY,
  .base      = SAM_SERCOM7_BASE,
};
#endif

const struct sam_usart_config_s *g_usartconfig[SAMD5E5_NSERCOM] =
{
#if SAMD5E5_NSERCOM > 0
#ifdef SAMD5E5_HAVE_USART0
  &g_usart0config,
#else
  (const struct sam_usart_config_s *)0,
#endif
#endif

#if SAMD5E5_NSERCOM > 1
#ifdef SAMD5E5_HAVE_USART1
  &g_usart1config,
#else
  (const struct sam_usart_config_s *)0,
#endif
#endif

#if SAMD5E5_NSERCOM > 2
#ifdef SAMD5E5_HAVE_USART2
  &g_usart2config,
#else
  (const struct sam_usart_config_s *)0,
#endif
#endif

#if SAMD5E5_NSERCOM > 3
#ifdef SAMD5E5_HAVE_USART3
  &g_usart3config,
#else
  (const struct sam_usart_config_s *)0,
#endif
#endif

#if SAMD5E5_NSERCOM > 4
#ifdef SAMD5E5_HAVE_USART4
  &g_usart4config,
#else
  (const struct sam_usart_config_s *)0,
#endif
#endif

#if SAMD5E5_NSERCOM > 5
#ifdef SAMD5E5_HAVE_USART5
  &g_usart5config,
#else
  (const struct sam_usart_config_s *)0,
#endif
#endif

#if SAMD5E5_NSERCOM > 6
#ifdef SAMD5E5_HAVE_USART6
  &g_usart6config,
#else
  (const struct sam_usart_config_s *)0,
#endif
#endif

#if SAMD5E5_NSERCOM > 7
#ifdef SAMD5E5_HAVE_USART6
  &g_usart7config,
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
