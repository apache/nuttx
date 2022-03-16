/****************************************************************************
 * arch/arm/src/samd2l2/sam_usart.h
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

#ifndef __ARCH_ARM_SRC_SAMD2L2_SAM_USART_H
#define __ARCH_ARM_SRC_SAMD2L2_SAM_USART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <arch/chip/chip.h>

#include "arm_internal.h"

#if defined(CONFIG_ARCH_FAMILY_SAMD20) || defined(CONFIG_ARCH_FAMILY_SAMD21)
#  include "hardware/samd_usart.h"
#elif defined(CONFIG_ARCH_FAMILY_SAML21)
#  include "hardware/saml_usart.h"
#endif

#include "sam_config.h"
#include "sam_port.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Pick the console USART configuration */

#if defined(CONFIG_USART0_SERIAL_CONSOLE)
#  define g_consoleconfig (g_usart0config)
#elif defined(CONFIG_USART1_SERIAL_CONSOLE)
#  define g_consoleconfig (g_usart1config)
#elif defined(CONFIG_USART2_SERIAL_CONSOLE)
#  define g_consoleconfig (g_usart2config)
#elif defined(CONFIG_USART3_SERIAL_CONSOLE)
#  define g_consoleconfig (g_usart3config)
#elif defined(CONFIG_USART4_SERIAL_CONSOLE)
#  define g_consoleconfig (g_usart4config)
#elif defined(CONFIG_USART5_SERIAL_CONSOLE)
#  define g_consoleconfig (g_usart5config)
#else
#  undef  g_consoleconfig
#endif

/* Is RS-485 used? */

#if defined(CONFIG_USART0_RS485MODE) || defined(CONFIG_USART1_RS485MODE) || \
    defined(CONFIG_USART2_RS485MODE) || defined(CONFIG_USART3_RS485MODE) || \
    defined(CONFIG_USART4_RS485MODE) || defined(CONFIG_USART5_RS485MODE)
#  define HAVE_RS485 1
#endif

#ifdef HAVE_RS485
#  define USART_TX_INTS    (USART_INT_DRE | USART_INT_TXC)
#else
#  define USART_TX_INTS    (USART_INT_DRE)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure describes the static configuration of a USART */

struct sam_usart_config_s
{
  uint8_t sercom;     /* Identifies the SERCOM peripheral */
  uint8_t parity;     /* 0=none, 1=odd, 2=even */
  uint8_t bits;       /* Number of bits (5-9) */
  uint8_t irq;        /* SERCOM IRQ number */
  uint8_t gclkgen;    /* Source GCLK generator */
  uint8_t slowgen;    /* Slow GCLK generator */
  bool stopbits2;     /* True: Configure with 2 stop bits instead of 1 */
  uint32_t baud;      /* Configured baud */
  port_pinset_t pad0; /* Pin configuration for PAD0 */
  port_pinset_t pad1; /* Pin configuration for PAD1 */
  port_pinset_t pad2; /* Pin configuration for PAD2 */
  port_pinset_t pad3; /* Pin configuration for PAD3 */
  uint32_t muxconfig; /* Pad multiplexing configuration */
  uint32_t frequency; /* Source clock frequency */
  uintptr_t base;     /* SERCOM base address */
};

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_wait_synchronization
 *
 * Description:
 *   Return true is the SERCOM USART reports that it is synchronizing.
 *   This inline function hides register differences between the SAMD20
 *   and SAML21.
 *
 ****************************************************************************/

#ifdef SAMD2L2_HAVE_USART
static inline
bool usart_syncbusy(const struct sam_usart_config_s * const config)
{
#if defined(CONFIG_ARCH_FAMILY_SAMD20)
  return ((getreg16(config->base + SAM_USART_STATUS_OFFSET) &
           USART_STATUS_SYNCBUSY) != 0);
#elif defined(CONFIG_ARCH_FAMILY_SAMD21) || defined(CONFIG_ARCH_FAMILY_SAML21)
  return ((getreg16(config->base + SAM_USART_SYNCBUSY_OFFSET) &
           USART_SYNCBUSY_ALL) != 0);
#else
#  error Unrecognized SAMD/L family
  return false;
#endif
}
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#ifdef SAMD2L2_HAVE_USART0
EXTERN const struct sam_usart_config_s g_usart0config;
#endif

#ifdef SAMD2L2_HAVE_USART1
EXTERN const struct sam_usart_config_s g_usart1config;
#endif

#ifdef SAMD2L2_HAVE_USART2
EXTERN const struct sam_usart_config_s g_usart2config;
#endif

#ifdef SAMD2L2_HAVE_USART3
EXTERN const struct sam_usart_config_s g_usart3config;
#endif

#ifdef SAMD2L2_HAVE_USART4
EXTERN const struct sam_usart_config_s g_usart4config;
#endif

#ifdef SAMD2L2_HAVE_USART5
EXTERN const struct sam_usart_config_s g_usart5config;
#endif

EXTERN const struct sam_usart_config_s *g_usartconfig[SAMD2L2_NSERCOM];

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_SAMD2L2_SAM_USART_H */
