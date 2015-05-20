/************************************************************************************
 * arch/arm/src/samdl/sam_usart.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMDL_SAM_USART_H
#define __ARCH_ARM_SRC_SAMDL_SAM_USART_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <arch/chip/chip.h>

#include "up_arch.h"

#if defined(CONFIG_ARCH_FAMILY_SAMD20)
#  include "chip/samd_usart.h"
#elif defined(CONFIG_ARCH_FAMILY_SAML21)
#  include "chip/saml_usart.h"
#endif

#include "sam_config.h"
#include "sam_port.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
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

/************************************************************************************
 * Public Types
 ************************************************************************************/
/* This structure describes the static configuration of a USART */

struct sam_usart_config_s
{
  uint8_t sercom;     /* Identifies the SERCOM peripheral */
  uint8_t parity;     /* 0=none, 1=odd, 2=even */
  uint8_t bits;       /* Number of bits (5-9) */
  uint8_t irq;        /* SERCOM IRQ number */
  uint8_t gclkgen;    /* Source GCLK generator */
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

/************************************************************************************
 * Inline Functions
 ************************************************************************************/
/************************************************************************************
 * Name: sam_wait_synchronization
 *
 * Description:
 *   Return true is the SERCOM USART reports that it is synchronizing.  This inline
 *   function hides register differences between the SAMD20 and SAML21.
 *
 ***********************************************************************************/

#ifdef SAMDL_HAVE_USART
static inline bool usart_syncbusy(const struct sam_usart_config_s * const config)
{
#if defined(CONFIG_ARCH_FAMILY_SAMD20)
  return ((getreg16(config->base + SAM_USART_STATUS_OFFSET) & USART_STATUS_SYNCBUSY) != 0);
#elif defined(CONFIG_ARCH_FAMILY_SAML21)
  return ((getreg16(config->base + SAM_USART_SYNCBUSY_OFFSET) & USART_SYNCBUSY_ALL) != 0);
#else
#  error Unrecognized SAMD/L family
  return false;
#endif
}
#endif

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#ifdef SAMDL_HAVE_USART0
EXTERN const struct sam_usart_config_s g_usart0config;
#endif

#ifdef SAMDL_HAVE_USART1
EXTERN const struct sam_usart_config_s g_usart1config;
#endif

#ifdef SAMDL_HAVE_USART2
EXTERN const struct sam_usart_config_s g_usart2config;
#endif

#ifdef SAMDL_HAVE_USART3
EXTERN const struct sam_usart_config_s g_usart3config;
#endif

#ifdef SAMDL_HAVE_USART4
EXTERN const struct sam_usart_config_s g_usart4config;
#endif

#ifdef SAMDL_HAVE_USART5
EXTERN const struct sam_usart_config_s g_usart5config;
#endif

EXTERN const struct sam_usart_config_s *g_usartconfig[SAMDL_NSERCOM];

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_SAMDL_SAM_USART_H */
