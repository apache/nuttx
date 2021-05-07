/****************************************************************************
 * arch/arm/src/sama5/sam_serial.h
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

#ifndef __ARCH_ARM_SRC_SAMA5_SAM_SERIAL_H
#define __ARCH_ARM_SRC_SAMA5_SAM_SERIAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "arm_internal.h"
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
 *   before arm_serialinit.
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
 *   before arm_serialinit.
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

#if defined(USE_SERIALDRIVER) && (defined(SAMA5_HAVE_UART) || defined(SAMA5_HAVE_USART))
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

#if defined(USE_SERIALDRIVER) && defined(SAMA5_HAVE_FLEXCOM_USART)
void flexus_serialinit(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_SAMA5_SAM_SERIAL_H */
