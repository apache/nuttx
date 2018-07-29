/************************************************************************************
 * arch/arm/src/samd5e5/sam_config.h
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMD5E5_SAM_CONFIG_H
#define __ARCH_ARM_SRC_SAMD5E5_SAM_CONFIG_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* How many SERCOM peripherals are configured as USART peripherals? */

#define SAMD5E5_HAVE_USART0 1
#define SAMD5E5_HAVE_USART1 1
#define SAMD5E5_HAVE_USART2 1
#define SAMD5E5_HAVE_USART3 1
#define SAMD5E5_HAVE_USART4 1
#define SAMD5E5_HAVE_USART5 1
#define SAMD5E5_HAVE_USART6 1
#define SAMD5E5_HAVE_USART7 1

#if !defined(CONFIG_SAMD5E5_SERCOM0) || !defined(CONFIG_SAMD5E5_SERCOM0_ISUSART) || \
    !defined(CONFIG_USART0_SERIALDRIVER)
#  undef SAMD5E5_HAVE_USART0
#  undef CONFIG_SAMD5E5_SERCOM0_ISUSART
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART0_FLOWCONTROL
#  undef CONFIG_USART0_IRDAMODE
#  undef CONFIG_USART0_RS485MODE
#endif

#if !defined(CONFIG_SAMD5E5_SERCOM1) || !defined(CONFIG_SAMD5E5_SERCOM1_ISUSART) || \
    !defined(CONFIG_USART1_SERIALDRIVER)
#  undef SAMD5E5_HAVE_USART1
#  undef CONFIG_SAMD5E5_SERCOM1_ISUSART
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART1_FLOWCONTROL
#  undef CONFIG_USART1_IRDAMODE
#  undef CONFIG_USART1_RS485MODE
#endif

#if !defined(CONFIG_SAMD5E5_SERCOM2) || !defined(CONFIG_SAMD5E5_SERCOM2_ISUSART) || \
    !defined(CONFIG_USART2_SERIALDRIVER)
#  undef SAMD5E5_HAVE_USART2
#  undef CONFIG_SAMD5E5_SERCOM2_ISUSART
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART2_FLOWCONTROL
#  undef CONFIG_USART2_IRDAMODE
#  undef CONFIG_USART2_RS485MODE
#endif

#if !defined(CONFIG_SAMD5E5_SERCOM3) || !defined(CONFIG_SAMD5E5_SERCOM3_ISUSART) || \
    !defined(CONFIG_USART3_SERIALDRIVER)
#  undef SAMD5E5_HAVE_USART3
#  undef CONFIG_SAMD5E5_SERCOM3_ISUSART
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef CONFIG_USART3_FLOWCONTROL
#  undef CONFIG_USART3_IRDAMODE
#  undef CONFIG_USART3_RS485MODE
#endif

#if !defined(CONFIG_SAMD5E5_SERCOM4) || !defined(CONFIG_SAMD5E5_SERCOM4_ISUSART) || \
    !defined(CONFIG_USART4_SERIALDRIVER)
#  undef SAMD5E5_HAVE_USART4
#  undef CONFIG_SAMD5E5_SERCOM4_ISUSART
#  undef CONFIG_USART4_SERIAL_CONSOLE
#  undef CONFIG_USART4_FLOWCONTROL
#  undef CONFIG_USART4_IRDAMODE
#  undef CONFIG_USART4_RS485MODE
#endif

#if !defined(CONFIG_SAMD5E5_SERCOM5) || !defined(CONFIG_SAMD5E5_SERCOM5_ISUSART) || \
    !defined(CONFIG_USART5_SERIALDRIVER)
#  undef SAMD5E5_HAVE_USART5
#  undef CONFIG_SAMD5E5_SERCOM5_ISUSART
#  undef CONFIG_USART5_SERIAL_CONSOLE
#  undef CONFIG_USART5_FLOWCONTROL
#  undef CONFIG_USART5_IRDAMODE
#  undef CONFIG_USART5_RS485MODE
#endif

#if !defined(CONFIG_SAMD5E5_SERCOM6) || !defined(CONFIG_SAMD5E5_SERCOM6_ISUSART) || \
    !defined(CONFIG_USART6_SERIALDRIVER)
#  undef SAMD5E5_HAVE_USART6
#  undef CONFIG_SAMD5E5_SERCOM6_ISUSART
#  undef CONFIG_USART6_SERIAL_CONSOLE
#  undef CONFIG_USART6_FLOWCONTROL
#  undef CONFIG_USART6_IRDAMODE
#  undef CONFIG_USART6_RS485MODE
#endif

#if !defined(CONFIG_SAMD5E5_SERCOM7) || !defined(CONFIG_SAMD5E5_SERCOM7_ISUSART) || \
    !defined(CONFIG_USART7_SERIALDRIVER)
#  undef SAMD5E5_HAVE_USART7
#  undef CONFIG_SAMD5E5_SERCOM7_ISUSART
#  undef CONFIG_USART7_SERIAL_CONSOLE
#  undef CONFIG_USART7_FLOWCONTROL
#  undef CONFIG_USART7_IRDAMODE
#  undef CONFIG_USART7_RS485MODE
#endif

/* Are any USARTs enabled? */

#undef SAMD5E5_HAVE_USART
#if defined(SAMD5E5_HAVE_USART0) || defined(SAMD5E5_HAVE_USART1) || \
    defined(SAMD5E5_HAVE_USART2) || defined(SAMD5E5_HAVE_USART3) || \
    defined(SAMD5E5_HAVE_USART4) || defined(SAMD5E5_HAVE_USART5) || \
    defined(SAMD5E5_HAVE_USART6) || defined(SAMD5E5_HAVE_USART7)
#  define SAMD5E5_HAVE_USART 1
#endif

/* Is there a serial console? There should be at most one defined.  It could be on
 * any USARTn, n=0-5 - OR - there might not be any serial console at all.
 */

#if defined(CONFIG_USART0_SERIAL_CONSOLE)
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef CONFIG_USART4_SERIAL_CONSOLE
#  undef CONFIG_USART5_SERIAL_CONSOLE
#  undef CONFIG_USART6_SERIAL_CONSOLE
#  undef CONFIG_USART7_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_USART1_SERIAL_CONSOLE)
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef CONFIG_USART4_SERIAL_CONSOLE
#  undef CONFIG_USART5_SERIAL_CONSOLE
#  undef CONFIG_USART6_SERIAL_CONSOLE
#  undef CONFIG_USART7_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_USART2_SERIAL_CONSOLE)
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef CONFIG_USART4_SERIAL_CONSOLE
#  undef CONFIG_USART5_SERIAL_CONSOLE
#  undef CONFIG_USART6_SERIAL_CONSOLE
#  undef CONFIG_USART7_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_USART3_SERIAL_CONSOLE)
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART4_SERIAL_CONSOLE
#  undef CONFIG_USART5_SERIAL_CONSOLE
#  undef CONFIG_USART6_SERIAL_CONSOLE
#  undef CONFIG_USART7_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_USART4_SERIAL_CONSOLE)
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef CONFIG_USART5_SERIAL_CONSOLE
#  undef CONFIG_USART6_SERIAL_CONSOLE
#  undef CONFIG_USART7_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_USART5_SERIAL_CONSOLE)
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef CONFIG_USART4_SERIAL_CONSOLE
#  undef CONFIG_USART6_SERIAL_CONSOLE
#  undef CONFIG_USART7_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_USART6_SERIAL_CONSOLE)
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef CONFIG_USART4_SERIAL_CONSOLE
#  undef CONFIG_USART5_SERIAL_CONSOLE
#  undef CONFIG_USART7_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_USART7_SERIAL_CONSOLE)
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef CONFIG_USART4_SERIAL_CONSOLE
#  undef CONFIG_USART5_SERIAL_CONSOLE
#  undef CONFIG_USART6_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#else
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef CONFIG_USART4_SERIAL_CONSOLE
#  undef CONFIG_USART5_SERIAL_CONSOLE
#  undef CONFIG_USART6_SERIAL_CONSOLE
#  undef CONFIG_USART7_SERIAL_CONSOLE
#  undef HAVE_SERIAL_CONSOLE
#endif

/* Are any SERCOM peripherals are configured as SPI peripherals? */

#define SAMD5E5_HAVE_SPI0 1
#define SAMD5E5_HAVE_SPI1 1
#define SAMD5E5_HAVE_SPI2 1
#define SAMD5E5_HAVE_SPI3 1
#define SAMD5E5_HAVE_SPI4 1
#define SAMD5E5_HAVE_SPI5 1
#define SAMD5E5_HAVE_SPI6 1
#define SAMD5E5_HAVE_SPI7 1

#if !defined(CONFIG_SAMD5E5_SERCOM0) || !defined(CONFIG_SAMD5E5_SERCOM0_ISSPI)
#  undef SAMD5E5_HAVE_SPI0
#  undef CONFIG_SAMD5E5_SERCOM0_ISSPI
#endif

#if !defined(CONFIG_SAMD5E5_SERCOM1) || !defined(CONFIG_SAMD5E5_SERCOM1_ISSPI)
#  undef SAMD5E5_HAVE_SPI1
#  undef CONFIG_SAMD5E5_SERCOM1_ISSPI
#endif

#if !defined(CONFIG_SAMD5E5_SERCOM2) || !defined(CONFIG_SAMD5E5_SERCOM2_ISSPI)
#  undef SAMD5E5_HAVE_SPI2
#  undef CONFIG_SAMD5E5_SERCOM2_ISSPI
#endif

#if !defined(CONFIG_SAMD5E5_SERCOM3) || !defined(CONFIG_SAMD5E5_SERCOM3_ISSPI)
#  undef SAMD5E5_HAVE_SPI3
#  undef CONFIG_SAMD5E5_SERCOM3_ISSPI
#endif

#if !defined(CONFIG_SAMD5E5_SERCOM4) || !defined(CONFIG_SAMD5E5_SERCOM4_ISSPI)
#  undef SAMD5E5_HAVE_SPI4
#  undef CONFIG_SAMD5E5_SERCOM4_ISSPI
#endif

#if !defined(CONFIG_SAMD5E5_SERCOM5) || !defined(CONFIG_SAMD5E5_SERCOM5_ISSPI)
#  undef SAMD5E5_HAVE_SPI5
#  undef CONFIG_SAMD5E5_SERCOM5_ISSPI
#endif

#if !defined(CONFIG_SAMD5E5_SERCOM6) || !defined(CONFIG_SAMD5E5_SERCOM6_ISSPI)
#  undef SAMD5E5_HAVE_SPI6
#  undef CONFIG_SAMD5E5_SERCOM6_ISSPI
#endif

#if !defined(CONFIG_SAMD5E5_SERCOM7) || !defined(CONFIG_SAMD5E5_SERCOM7_ISSPI)
#  undef SAMD5E5_HAVE_SPI7
#  undef CONFIG_SAMD5E5_SERCOM7_ISSPI
#endif

/* Are any SERCOMs configured for SPI? */

#undef SAMD5E5_HAVE_SPI
#if defined(SAMD5E5_HAVE_SPI0) || defined(SAMD5E5_HAVE_SPI1) || \
    defined(SAMD5E5_HAVE_SPI2) || defined(SAMD5E5_HAVE_SPI3) || \
    defined(SAMD5E5_HAVE_SPI4) || defined(SAMD5E5_HAVE_SPI5) || \
    defined(SAMD5E5_HAVE_SPI6) || defined(SAMD5E5_HAVE_SPI7)
#  define SAMD5E5_HAVE_SPI 1
#endif

/* Are any SERCOM peripherals are configured as I2C peripherals? */

#define SAMD5E5_HAVE_I2C0_MASTER 1
#define SAMD5E5_HAVE_I2C1_MASTER 1
#define SAMD5E5_HAVE_I2C2_MASTER 1
#define SAMD5E5_HAVE_I2C3_MASTER 1
#define SAMD5E5_HAVE_I2C4_MASTER 1
#define SAMD5E5_HAVE_I2C5_MASTER 1
#define SAMD5E5_HAVE_I2C6_MASTER 1
#define SAMD5E5_HAVE_I2C7_MASTER 1

#if !defined(CONFIG_SAMD5E5_SERCOM0) || !defined(CONFIG_SAMD5E5_SERCOM0_ISI2C)
#  undef SAMD5E5_HAVE_I2C0_MASTER
#  undef CONFIG_SAMD5E5_SERCOM0_ISI2C
#endif

#if !defined(CONFIG_SAMD5E5_SERCOM1) || !defined(CONFIG_SAMD5E5_SERCOM1_ISI2C)
#  undef SAMD5E5_HAVE_I2C1_MASTER
#  undef CONFIG_SAMD5E5_SERCOM1_ISI2C
#endif

#if !defined(CONFIG_SAMD5E5_SERCOM2) || !defined(CONFIG_SAMD5E5_SERCOM2_ISI2C)
#  undef SAMD5E5_HAVE_I2C2_MASTER
#  undef CONFIG_SAMD5E5_SERCOM2_ISI2C
#endif

#if !defined(CONFIG_SAMD5E5_SERCOM3) || !defined(CONFIG_SAMD5E5_SERCOM3_ISI2C)
#  undef SAMD5E5_HAVE_I2C3_MASTER
#  undef CONFIG_SAMD5E5_SERCOM3_ISI2C
#endif

#if !defined(CONFIG_SAMD5E5_SERCOM4) || !defined(CONFIG_SAMD5E5_SERCOM4_ISI2C)
#  undef SAMD5E5_HAVE_I2C4_MASTER
#  undef CONFIG_SAMD5E5_SERCOM4_ISI2C
#endif

#if !defined(CONFIG_SAMD5E5_SERCOM5) || !defined(CONFIG_SAMD5E5_SERCOM5_ISI2C)
#  undef SAMD5E5_HAVE_I2C5_MASTER
#  undef CONFIG_SAMD5E5_SERCOM5_ISI2C
#endif

#if !defined(CONFIG_SAMD5E5_SERCOM6) || !defined(CONFIG_SAMD5E5_SERCOM6_ISI2C)
#  undef SAMD5E5_HAVE_I2C6_MASTER
#  undef CONFIG_SAMD5E5_SERCOM6_ISI2C
#endif

#if !defined(CONFIG_SAMD5E5_SERCOM7) || !defined(CONFIG_SAMD5E5_SERCOM7_ISI2C)
#  undef SAMD5E5_HAVE_I2C7_MASTER
#  undef CONFIG_SAMD5E5_SERCOM7_ISI2C
#endif

/* Are any SERCOMs configured for I2C? */

#undef SAMD5E5_HAVE_I2C_MASTER
#if defined(SAMD5E5_HAVE_I2C0_MASTER) || defined(SAMD5E5_HAVE_I2C1_MASTER) || \
    defined(SAMD5E5_HAVE_I2C2_MASTER) || defined(SAMD5E5_HAVE_I2C3_MASTER) || \
    defined(SAMD5E5_HAVE_I2C4_MASTER) || defined(SAMD5E5_HAVE_I2C5_MASTER) || \
    defined(SAMD5E5_HAVE_I2C6_MASTER) || defined(SAMD5E5_HAVE_I2C7_MASTER)
#  define SAMD5E5_HAVE_I2C_MASTER 1
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMD5E5_SAM_CONFIG_H */
