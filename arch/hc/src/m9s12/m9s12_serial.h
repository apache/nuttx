/****************************************************************************
 * arch/hc/src/m9s12/m9s12_serial.h
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

#ifndef __ARCH_HC_SRC_M9S12_M9S12_SERIAL_H
#define __ARCH_HC_SRC_M9S12_M9S12_SERIAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Is there a SCI enabled? */

#if defined(CONFIG_SCI0_DISABLE) && defined(CONFIG_SCI1_DISABLE)
#  undef HAVE_SERIAL_DEVICE
#  warning "No SCIs enabled"
#else
#  define HAVE_SERIAL_DEVICE 1
#endif

/* Is there a serial console? */

#if defined(CONFIG_SCI0_SERIAL_CONSOLE) && !defined(CONFIG_SCI0_DISABLE)
#  undef CONFIG_SCI1_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_SCI1_SERIAL_CONSOLE) && !defined(CONFIG_SCI1_DISABLE)
#  undef CONFIG_SCI0_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#else
#  warning "No valid CONFIG_SCIn_SERIAL_CONSOLE Setting"
#  undef CONFIG_SCI0_SERIAL_CONSOLE
#  undef CONFIG_SCI1_SERIAL_CONSOLE
#  undef HAVE_SERIAL_CONSOLE
#endif

/* Sanity checking */

#ifndef CONFIG_SCI0_DISABLE
#  ifndef CONFIG_SCI0_PARITY
#    warning "CONFIG_SCI0_PARITY not defined -- Assuming none"
#    define CONFIG_SCI0_PARITY 0
#  elif CONFIG_SCI0_PARITY != 0 && CONFIG_SCI0_PARITY != 2 && CONFIG_SCI0_PARITY != 2
#    error "CONFIG_SCI0_PARITY value not recognized"
#  endif
#  ifndef CONFIG_SCI0_BITS
#    warning "CONFIG_SCI0_BITS not defined -- Assuming 8"
#    define CONFIG_SCI0_BITS 8
#  elif CONFIG_SCI0_BITS != 8 && CONFIG_SCI0_BITS != 9
#    error "CONFIG_SCI0_BITS value not supported"
#  endif
#  if defined(CONFIG_SCI0_2STOP) && CONFIG_SCI0_2STOP != 0
#    error "Only a single stop bit is supported"
#  endif
#endif

#ifndef CONFIG_SCI1_DISABLE
#  ifndef CONFIG_SCI1_PARITY
#    warning "CONFIG_SCI1_PARITY not defined -- Assuming none"
#    define CONFIG_SCI1_PARITY 0
#  elif CONFIG_SCI1_PARITY != 0 && CONFIG_SCI1_PARITY != 2 && CONFIG_SCI1_PARITY != 2
#    error "CONFIG_SCI1_PARITY value not recognized"
#  endif
#  ifndef CONFIG_SCI1_BITS
#    warning "CONFIG_SCI1_BITS not defined -- Assuming 8"
#    define CONFIG_SCI1_BITS 8
#  elif CONFIG_SCI1_BITS != 8 && CONFIG_SCI1_BITS != 9
#    error "CONFIG_SCI1_BITS value not supported"
#  endif
#  if defined(CONFIG_SCI1_2STOP) && CONFIG_SCI1_2STOP != 0
#    error "Only a single stop bit is supported"
#  endif
#endif

/* BAUD *********************************************************************/

/* Baud calculations.  The SCI module is driven by the BUSCLK.  The SCIBR
 * register value divides down the BUSCLK to accomplish the required BAUD.
 *
 *   BAUD  = HCS12_BUSCLK / (16 * SCIBR)
 *   SCIBR = HCS12_BUSCLK / (16 * BAUD)
 */

#define SCIBR_VALUE(b) ((HCS12_BUSCLK * (b) + ((b) << 3))/((b) << 4))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_HC_SRC_M9S12_M9S12_SERIAL_H */
