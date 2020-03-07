/****************************************************************************
 * arch/arm/src/sama5/sam_serial.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_SAMA5_SAM_SERIAL_H
#define __ARCH_ARM_SRC_SAMA5_SAM_SERIAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "up_internal.h"
#include "sam_config.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: sam_earlyserialinit
 *
 * Description:
 *   Performs the low level USART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before up_serialinit.
 *
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT
void sam_earlyserialinit(void);
#endif

/****************************************************************************
 * Name: uart_earlyserialinit
 *
 * Description:
 *   Performs the low level USART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before up_serialinit.
 *
 ****************************************************************************/

#if defined(USE_EARLYSERIALINIT) && (defined(SAMA5_HAVE_UART) || defined(SAMA5_HAVE_USART))
void uart_earlyserialinit(void);
#endif

/****************************************************************************
 * Name: flexus_earlyserialinit
 *
 * Description:
 *   Performs the low level Flexcom USART initialization early so that the
 *   Flexcom serial console will be available during bootup.  This must be
 *   called before flexus_serialinit.
 *
 ****************************************************************************/

#if defined(USE_EARLYSERIALINIT) && defined(SAMA5_HAVE_FLEXCOM_USART)
void flexus_earlyserialinit(void);
#endif

/****************************************************************************
 * Name: uart_serialinit
 *
 * Description:
 *   Register UART/USART serial console and serial ports.  This assumes that
 *   uart_earlyserialinit was called previously.
 *
 ****************************************************************************/

#if defined(SAMA5_HAVE_UART) || defined(SAMA5_HAVE_USART)
void uart_serialinit(void);
#endif

/****************************************************************************
 * Name: flexus_serialinit
 *
 * Description:
 *   Register Flexcom serial console and serial ports.  This assumes that
 *   flexus_earlyserialinit was called previously.
 *
 ****************************************************************************/

#if defined(USE_EARLYSERIALINIT) && defined(SAMA5_HAVE_FLEXCOM_USART)
void flexus_serialinit(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_SAMA5_SAM_SERIAL_H */
