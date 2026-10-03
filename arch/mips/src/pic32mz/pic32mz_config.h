/****************************************************************************
 * arch/mips/src/pic32mz/pic32mz_config.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_MIPS_SRC_PIC32MZ_PIC32MZ_CONFIG_H
#define __ARCH_MIPS_SRC_PIC32MZ_PIC32MZ_CONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/chip/chip.h>
#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Cache line sizes (in bytes) for the PIC32MZ */

#define PIC32MZ_DCACHE_LINESIZE 16  /* 16 bytes (4 words) */
#define PIC32MZ_ICACHE_LINESIZE 16  /* 16 bytes (4 words) */

/* GPIO IRQs ****************************************************************/

#ifndef CONFIG_PIC32MZ_GPIOIRQ
#  undef CONFIG_PIC32MZ_GPIOIRQ_PORTA
#  undef CONFIG_PIC32MZ_GPIOIRQ_PORTB
#  undef CONFIG_PIC32MZ_GPIOIRQ_PORTC
#  undef CONFIG_PIC32MZ_GPIOIRQ_PORTD
#  undef CONFIG_PIC32MZ_GPIOIRQ_PORTE
#  undef CONFIG_PIC32MZ_GPIOIRQ_PORTF
#  undef CONFIG_PIC32MZ_GPIOIRQ_PORTG
#  undef CONFIG_PIC32MZ_GPIOIRQ_PORTH
#  undef CONFIG_PIC32MZ_GPIOIRQ_PORTJ
#  undef CONFIG_PIC32MZ_GPIOIRQ_PORTK
#endif

#if CHIP_NPORTS < 1
#  undef CONFIG_PIC32MZ_GPIOIRQ_PORTA
#endif
#if CHIP_NPORTS < 2
#  undef CONFIG_PIC32MZ_GPIOIRQ_PORTB
#endif
#if CHIP_NPORTS < 3
#  undef CONFIG_PIC32MZ_GPIOIRQ_PORTC
#endif
#if CHIP_NPORTS < 4
#  undef CONFIG_PIC32MZ_GPIOIRQ_PORTD
#endif
#if CHIP_NPORTS < 5
#  undef CONFIG_PIC32MZ_GPIOIRQ_PORTE
#endif
#if CHIP_NPORTS < 6
#  undef CONFIG_PIC32MZ_GPIOIRQ_PORTF
#endif
#if CHIP_NPORTS < 7
#  undef CONFIG_PIC32MZ_GPIOIRQ_PORTG
#endif
#if CHIP_NPORTS < 8
#  undef CONFIG_PIC32MZ_GPIOIRQ_PORTH
#endif
#if CHIP_NPORTS < 9
#  undef CONFIG_PIC32MZ_GPIOIRQ_PORTJ
#endif
#if CHIP_NPORTS < 10
#  undef CONFIG_PIC32MZ_GPIOIRQ_PORTK
#endif

/* UARTs ********************************************************************/

/* Don't enable UARTs not supported by the chip. */

#if CHIP_NUARTS < 1
#  undef CONFIG_PIC32MZ_UART1
#  undef CONFIG_PIC32MZ_UART2
#  undef CONFIG_PIC32MZ_UART3
#  undef CONFIG_PIC32MZ_UART4
#  undef CONFIG_PIC32MZ_UART5
#  undef CONFIG_PIC32MZ_UART6
#elif CHIP_NUARTS < 2
#  undef CONFIG_PIC32MZ_UART2
#  undef CONFIG_PIC32MZ_UART3
#  undef CONFIG_PIC32MZ_UART4
#  undef CONFIG_PIC32MZ_UART5
#  undef CONFIG_PIC32MZ_UART6
#elif CHIP_NUARTS < 3
#  undef CONFIG_PIC32MZ_UART3
#  undef CONFIG_PIC32MZ_UART4
#  undef CONFIG_PIC32MZ_UART5
#  undef CONFIG_PIC32MZ_UART6
#elif CHIP_NUARTS < 4
#  undef CONFIG_PIC32MZ_UART4
#  undef CONFIG_PIC32MZ_UART5
#  undef CONFIG_PIC32MZ_UART6
#elif CHIP_NUARTS < 5
#  undef CONFIG_PIC32MZ_UART5
#  undef CONFIG_PIC32MZ_UART6
#elif CHIP_NUARTS < 6
#  undef CONFIG_PIC32MZ_UART6
#endif

/* Are any UARTs enabled? */

#undef HAVE_UART_DEVICE
#if defined(CONFIG_PIC32MZ_UART1) || defined(CONFIG_PIC32MZ_UART2) || \
    defined(CONFIG_PIC32MZ_UART3) || defined(CONFIG_PIC32MZ_UART4) || \
    defined(CONFIG_PIC32MZ_UART5) || defined(CONFIG_PIC32MZ_UART6)
#  define HAVE_UART_DEVICE 1
#endif

/* Is there a serial console?  There should be no more than one defined.  It
 * could be on any UARTn, n=1,.. CHIP_NUARTS
 */

#if defined(CONFIG_UART1_SERIAL_CONSOLE) && defined(CONFIG_PIC32MZ_UART1)
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  undef CONFIG_UART6_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART2_SERIAL_CONSOLE) && defined(CONFIG_PIC32MZ_UART2)
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  undef CONFIG_UART6_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART3_SERIAL_CONSOLE) && defined(CONFIG_PIC32MZ_UART3)
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  undef CONFIG_UART6_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART4_SERIAL_CONSOLE) && defined(CONFIG_PIC32MZ_UART4)
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  undef CONFIG_UART6_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART5_SERIAL_CONSOLE) && defined(CONFIG_PIC32MZ_UART5)
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART6_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART6_SERIAL_CONSOLE) && defined(CONFIG_PIC32MZ_UART6)
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#else
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  undef CONFIG_UART6_SERIAL_CONSOLE
#  undef HAVE_SERIAL_CONSOLE
#endif

/* SPI **********************************************************************/

/* Don't enable SPI peripherals not supported by the chip. */

#if CHIP_NSPI < 1
#  undef CONFIG_PIC32MZ_SPI1
#  undef CONFIG_PIC32MZ_SPI2
#  undef CONFIG_PIC32MZ_SPI3
#  undef CONFIG_PIC32MZ_SPI4
#  undef CONFIG_PIC32MZ_SPI5
#  undef CONFIG_PIC32MZ_SPI6
#elif CHIP_NSPI < 2
#  undef CONFIG_PIC32MZ_SPI2
#  undef CONFIG_PIC32MZ_SPI3
#  undef CONFIG_PIC32MZ_SPI4
#  undef CONFIG_PIC32MZ_SPI5
#  undef CONFIG_PIC32MZ_SPI6
#elif CHIP_NSPI < 3
#  undef CONFIG_PIC32MZ_SPI3
#  undef CONFIG_PIC32MZ_SPI4
#  undef CONFIG_PIC32MZ_SPI5
#  undef CONFIG_PIC32MZ_SPI6
#elif CHIP_NSPI < 4
#  undef CONFIG_PIC32MZ_SPI4
#  undef CONFIG_PIC32MZ_SPI5
#  undef CONFIG_PIC32MZ_SPI6
#elif CHIP_NSPI < 5
#  undef CONFIG_PIC32MZ_SPI5
#  undef CONFIG_PIC32MZ_SPI6
#elif CHIP_NSPI < 6
#  undef CONFIG_PIC32MZ_SPI6
#endif

/* Are any SPI peripherals enabled? */

#undef CONFIG_PIC32MZ_SPI
#if defined(CONFIG_PIC32MZ_SPI1) || defined(CONFIG_PIC32MZ_SPI2) || \
    defined(CONFIG_PIC32MZ_SPI4) || defined(CONFIG_PIC32MZ_SPI4) || \
    defined(CONFIG_PIC32MZ_SPI5) || defined(CONFIG_PIC32MZ_SPI6)
#  define CONFIG_PIC32MZ_SPI 1
#endif

/* I2C **********************************************************************/

/* Don't enable I2C peripherals not supported by the chip. */

#if CHIP_NI2C < 1
#  undef CONFIG_PIC32MZ_I2C1
#  undef CONFIG_PIC32MZ_I2C2
#  undef CONFIG_PIC32MZ_I2C3
#  undef CONFIG_PIC32MZ_I2C4
#  undef CONFIG_PIC32MZ_I2C5
#elif CHIP_NI2C < 2
#  undef CONFIG_PIC32MZ_I2C2
#  undef CONFIG_PIC32MZ_I2C3
#  undef CONFIG_PIC32MZ_I2C4
#  undef CONFIG_PIC32MZ_I2C5
#elif CHIP_NI2C < 3
#  undef CONFIG_PIC32MZ_I2C3
#  undef CONFIG_PIC32MZ_I2C4
#  undef CONFIG_PIC32MZ_I2C5
#elif CHIP_NI2C < 4
#  undef CONFIG_PIC32MZ_I2C4
#  undef CONFIG_PIC32MZ_I2C5
#elif CHIP_NI2C < 5
#  undef CONFIG_PIC32MZ_I2C5
#endif

/* Are any I2C peripherals enabled? */

#undef CONFIG_PIC32MZ_I2C
#if defined(CONFIG_PIC32MZ_I2C1) || defined(CONFIG_PIC32MZ_I2C2) || \
    defined(CONFIG_PIC32MZ_I2C4) || defined(CONFIG_PIC32MZ_I2C4) || \
    defined(CONFIG_PIC32MZ_I2C5)
#  define CONFIG_PIC32MZ_I2C 1
#endif

/* Device Configuration *****************************************************/

/* PIC32MZ-W1 does not use the classic EC/EF DEVCFG0-3 layout (see
 * hardware/pic32mzw1_features.h); its words are composed in the #else
 * branch at the end of this block.
 */

#ifndef CONFIG_ARCH_CHIP_PIC32MZW1

/* DEVCFG3 */

/* Configurable settings */

#ifndef CONFIG_PIC32MZ_USERID               /* User ID */
#  define CONFIG_PIC32MZ_USERID   0x584e    /* "NX" */
#endif
#define ADEVCFG3_USERID           0x1234

#ifndef CONFIG_PIC32MZ_FMIIEN               /* Ethernet MII enable: 0=RMII 1=MII */
#  define CONFIG_PIC32MZ_FMIIEN   1         /* MII enabled */
#endif

#ifndef CONFIG_PIC32MZ_PGL1WAY              /* Permission group lock one way configuration */
#  define CONFIG_PIC32MZ_PGL1WAY  0         /* Allow multiple configurations */
#endif

#ifndef CONFIG_PIC32MZ_PMDL1WAY             /* Peripheral module disable configuration */
#  define CONFIG_PIC32MZ_PMDL1WAY 0         /* Allow multiple reconfigurations */
#endif

#ifndef CONFIG_PIC32MZ_IOL1WAY              /* Peripheral pin select configuration */
#  define CONFIG_PIC32MZ_IOL1WAY  0         /* Allow multiple reconfigurations */
#endif

#ifndef CONFIG_PIC32MZ_FETHIO               /* Ethernet I/O Pins 0=alternate 1=default */
#  define CONFIG_PIC32MZ_FETHIO   1         /* Default Ethernet I/O Pins */
#endif

#ifndef CONFIG_PIC32MZ_FUSBIDIO              /* USB USBID selection: 0=GPIO 1=USB */
#  ifdef CONFIG_PIC32MZ_USB
#    define CONFIG_PIC32MZ_FUSBIDIO 1        /* USBID pin is controlled by the USB module */
#  else
#    define CONFIG_PIC32MZ_FUSBIDIO 0        /* USBID pin is controlled by the IOPORT configuration */
#  endif
#endif

/* DEVCFG2 */

/* PLL Input Divider bits */

#undef CONFIG_PIC32MZ_PLLIDIV
#if BOARD_PLL_IDIV == 1
#  define CONFIG_PIC32MZ_PLLIDIV  DEVCFG2_FPLLIDIV_1
#elif BOARD_PLL_IDIV == 2
#  define CONFIG_PIC32MZ_PLLIDIV  DEVCFG2_FPLLIDIV_2
#elif BOARD_PLL_IDIV == 3
#  define CONFIG_PIC32MZ_PLLIDIV  DEVCFG2_FPLLIDIV_3
#elif BOARD_PLL_IDIV == 4
#  define CONFIG_PIC32MZ_PLLIDIV  DEVCFG2_FPLLIDIV_4
#elif BOARD_PLL_IDIV == 5
#  define CONFIG_PIC32MZ_PLLIDIV  DEVCFG2_FPLLIDIV_5
#elif BOARD_PLL_IDIV == 6
#  define CONFIG_PIC32MZ_PLLIDIV  DEVCFG2_FPLLIDIV_6
#elif BOARD_PLL_IDIV == 7
#  define CONFIG_PIC32MZ_PLLIDIV  DEVCFG2_FPLLIDIV_7
#elif BOARD_PLL_IDIV == 8
#  define CONFIG_PIC32MZ_PLLIDIV  DEVCFG2_FPLLIDIV_8
#else
#  error "Unsupported BOARD_PLL_IDIV"
#endif

/* System PLL Divided Input Clock Frequency Range bits.
 * REVISIT: Based on the name of this configuration value, the following
 * comparisons do not seem correct (the input clock is not divided).
 * These comparisons are used because this results in settings that match
 * Microchip sample code.
 */

#if BOARD_PLL_INPUT < 5000000
#  error BOARD_PLL_INPUT / BOARD_PLL_IDIV too low
#  define CONFIG_PIC32MZ_FPLLRNG  DEVCFG2_FPLLRNG_BYPASS   /* < 5 MHz */
#elif BOARD_PLL_INPUT < 10000000
#  define CONFIG_PIC32MZ_FPLLRNG  DEVCFG2_FPLLRNG_5_10MHZ  /* 5-10 MHz */
#elif BOARD_PLL_INPUT < 16000000
#  define CONFIG_PIC32MZ_FPLLRNG  DEVCFG2_FPLLRNG_8_16MHZ  /* 8-16 MHz */
#elif BOARD_PLL_INPUT < 26000000
#  define CONFIG_PIC32MZ_FPLLRNG  DEVCFG2_FPLLRNG_13_26MHZ /* 13-26 MHz */
#elif BOARD_PLL_INPUT < 42000000
#  define CONFIG_PIC32MZ_FPLLRNG  DEVCFG2_FPLLRNG_21_42MHZ /* 21-42 MHz */
#elif BOARD_PLL_INPUT <= 64000000
#  define CONFIG_PIC32MZ_FPLLRNG  DEVCFG2_FPLLRNG_34_64MHZ /* 36-64 MHz */
#else
#  error BOARD_PLL_INPUT too high
#  define CONFIG_PIC32MZ_FPLLRNG  DEVCFG2_FPLLRNG_34_64MHZ /* > 64 MHz */
#endif

/* PLL multiplier */

#undef CONFIG_PIC32MZ_PLLMULT
#if BOARD_PLL_MULT >= 1 && BOARD_PLL_MULT <= 128
#  define CONFIG_PIC32MZ_PLLMULT  ((BOARD_PLL_MULT-1) << DEVCFG2_FPLLMULT_SHIFT)
#else
#  error "Unsupported BOARD_PLL_MULT"
#endif

/* PLL output divider */

#undef CONFIG_PIC32MZ_PLLODIV
#if BOARD_PLL_ODIV == 2
#  define CONFIG_PIC32MZ_PLLODIV  DEVCFG2_FPLLODIV_2
#elif BOARD_PLL_ODIV == 4
#  define CONFIG_PIC32MZ_PLLODIV  DEVCFG2_FPLLODIV_4
#elif BOARD_PLL_ODIV == 8
#  define CONFIG_PIC32MZ_PLLODIV  DEVCFG2_FPLLODIV_8
#elif BOARD_PLL_ODIV == 16
#  define CONFIG_PIC32MZ_PLLODIV  DEVCFG2_FPLLODIV_16
#elif BOARD_PLL_ODIV == 32
#  define CONFIG_PIC32MZ_PLLODIV  DEVCFG2_FPLLODIV_32
#else
#  error "Unsupported BOARD_PLL_ODIV"
#endif

#if BOARD_POSC_FREQ == 12000000
#  define CONFIG_PIC32MZ_UPLLFSEL DEVCFG2_UPLLFSEL_12MHZ
#else
#  define CONFIG_PIC32MZ_UPLLFSEL DEVCFG2_UPLLFSEL_24MHZ
#endif

/* System PLL Input Clock Select bit */

#undef CONFIG_PIC32MZ_FPLLICLK
#if defined(BOARD_FPLLICLK_FRC)
#  define CONFIG_PIC32MZ_FPLLICLK DEVCFG2_FPLLICLK
#else
#  define CONFIG_PIC32MZ_FPLLICLK 0        /* POSC is selected as input to the System PLL */
#endif

/* USB PLL Input Frequency Select bit */

/* DEVCFG1 */

/* Configurable settings */

#undef CONFIG_PIC32MZ_FNOSC
#if defined(BOARD_FNOSC_FRC)
#  define CONFIG_PIC32MZ_FNOSC    DEVCFG1_FNOSC_FRC
#elif defined(BOARD_FNOSC_SPLL)
#  define CONFIG_PIC32MZ_FNOSC    DEVCFG1_FNOSC_SPLL
#elif defined(BOARD_FNOSC_POSC)
#  define CONFIG_PIC32MZ_FNOSC    DEVCFG1_FNOSC_POSC
#elif defined(BOARD_FNOSC_SOSC)
#  define CONFIG_PIC32MZ_FNOSC    DEVCFG1_FNOSC_SOSC
#elif defined(BOARD_FNOSC_LPRC)
#  define CONFIG_PIC32MZ_FNOSC    DEVCFG1_FNOSC_LPRC
#elif defined(BOARD_FNOSC_FRCDIV)
#  define CONFIG_PIC32MZ_FNOSC    DEVCFG1_FNOSC_FRCDIV
#else
#  error "Unknown board FNOSC selection"
#endif

#undef CONFIG_PIC32MZ_FSOSCEN
#ifdef BOARD_SOSC_ENABLE
#  define CONFIG_PIC32MZ_FSOSCEN DEVCFG1_FSOSCEN
#else
#  define CONFIG_PIC32MZ_FSOSCEN 0
#endif

#undef CONFIG_PIC32MZ_IESO
#ifdef BOARD_SOSC_IESO
#  define CONFIG_PIC32MZ_IESO    DEVCFG1_IESO
#else
#  define CONFIG_PIC32MZ_IESO    0
#endif

#undef CONFIG_PIC32MZ_POSCMOD
#if defined(BOARD_POSC_ECMODE)
#  define CONFIG_PIC32MZ_POSCMOD DEVCFG1_POSCMOD_EC
#elif defined(BOARD_POSC_HSMODE)
#  define CONFIG_PIC32MZ_POSCMOD DEVCFG1_POSCMOD_HS
#elif defined(BOARD_POSC_DISABLED)
#  define CONFIG_PIC32MZ_POSCMOD DEVCFG1_POSCMOD_DIS
#else
#  error "Unknown board POSC mode"
#endif

#ifdef CONFIG_PIC32MZ_OSCIOFNC
#  undef CONFIG_PIC32MZ_OSCIOFNC
#  define CONFIG_PIC32MZ_OSCIOFNC DEVCFG1_OSCIOFNC
#else
#  undef CONFIG_PIC32MZ_OSCIOFNC
#  define CONFIG_PIC32MZ_OSCIOFNC 0
#endif

#undef CONFIG_PIC32MZ_FCKSM
#if defined(BOARD_POSC_SWITCH)
#  if defined(BOARD_POSC_FSCM)
#    define CONFIG_PIC32MZ_FCKSM DEVCFG1_FCKSM_BOTH
#  else
#    define CONFIG_PIC32MZ_FCKSM DEVCFG1_FCKSM_SWITCH
#  endif
#else
#  if defined(BOARD_POSC_FSCM)
#    define CONFIG_PIC32MZ_FCKSM DEVCFG1_FCKSM_MONITOR
#  else
#    define CONFIG_PIC32MZ_FCKSM DEVCFG1_FCKSM_NONE
#  endif
#endif

#undef CONFIG_PIC32MZ_WDTPS
#if BOARD_WD_PRESCALER == 1
#  define CONFIG_PIC32MZ_WDTPS   DEVCFG1_WDTPS_1
#elif BOARD_WD_PRESCALER == 2
#  define CONFIG_PIC32MZ_WDTPS   DEVCFG1_WDTPS_2
#elif BOARD_WD_PRESCALER == 4
#  define CONFIG_PIC32MZ_WDTPS   DEVCFG1_WDTPS_4
#elif BOARD_WD_PRESCALER == 8
#  define CONFIG_PIC32MZ_WDTPS   DEVCFG1_WDTPS_8
#elif BOARD_WD_PRESCALER == 16
#  define CONFIG_PIC32MZ_WDTPS   DEVCFG1_WDTPS_16
#elif BOARD_WD_PRESCALER == 32
#  define CONFIG_PIC32MZ_WDTPS   DEVCFG1_WDTPS_32
#elif BOARD_WD_PRESCALER == 64
#  define CONFIG_PIC32MZ_WDTPS   DEVCFG1_WDTPS_64
#elif BOARD_WD_PRESCALER == 128
#  define CONFIG_PIC32MZ_WDTPS   DEVCFG1_WDTPS_128
#elif BOARD_WD_PRESCALER == 256
#  define CONFIG_PIC32MZ_WDTPS   DEVCFG1_WDTPS_256
#elif BOARD_WD_PRESCALER == 512
#  define CONFIG_PIC32MZ_WDTPS   DEVCFG1_WDTPS_512
#elif BOARD_WD_PRESCALER == 1024
#  define CONFIG_PIC32MZ_WDTPS   DEVCFG1_WDTPS_1024
#elif BOARD_WD_PRESCALER == 2048
#  define CONFIG_PIC32MZ_WDTPS   DEVCFG1_WDTPS_2048
#elif BOARD_WD_PRESCALER == 4096
#  define CONFIG_PIC32MZ_WDTPS   DEVCFG1_WDTPS_4096
#elif BOARD_WD_PRESCALER == 8192
#  define CONFIG_PIC32MZ_WDTPS   DEVCFG1_WDTPS_8192
#elif BOARD_WD_PRESCALER == 16384
#  define CONFIG_PIC32MZ_WDTPS   DEVCFG1_WDTPS_16384
#elif BOARD_WD_PRESCALER == 32768
#  define CONFIG_PIC32MZ_WDTPS   DEVCFG1_WDTPS_32768
#elif BOARD_WD_PRESCALER == 65536
#  define CONFIG_PIC32MZ_WDTPS   DEVCFG1_WDTPS_65536
#elif BOARD_WD_PRESCALER == 131072
#  define CONFIG_PIC32MZ_WDTPS   DEVCFG1_WDTPS_131072
#elif BOARD_WD_PRESCALER == 262144
#  define CONFIG_PIC32MZ_WDTPS   DEVCFG1_WDTPS_262144
#elif BOARD_WD_PRESCALER == 524288
#  define CONFIG_PIC32MZ_WDTPS   DEVCFG1_WDTPS_524288
#elif BOARD_WD_PRESCALER == 1048576
#  define CONFIG_PIC32MZ_WDTPS   DEVCFG1_WDTPS_1048576
#else
#  error "Unsupported BOARD_WD_PRESCALER"
#endif

#undef CONFIG_PIC32MZ_FWDTEN
#ifdef CONFIG_PIC32MZ_WDTENABLE
#  define CONFIG_PIC32MZ_FWDTEN  DEVCFG1_FWDT_ENABLED
#else
#  define CONFIG_PIC32MZ_FWDTEN  DEVCFG1_FWDT_DISABLED
#endif
#define ADEVCFG1_FWDTEN          DEVCFG1_FWDT_DISABLED

/* Not yet configurable settings */

#define CONFIG_PIC32MZ_DMTINV    DEVCFG1_DMTINV_127_128
#define CONFIG_PIC32MZ_WDTSPGM   DEVCFG1_WDTSPGM_STOP
#define CONFIG_PIC32MZ_WINDIS    DEVCFG1_WDT_NORMAL
#define CONFIG_PIC32MZ_FWDTWINSZ DEVCFG1_FWDTWINSZ_25
#define CONFIG_PIC32MZ_DMTCNT    DEVCFG1_DMTCNT_MAX
#define CONFIG_PIC32MZ_FDMTEN    0

/* DEVCFG0 */

/* Configurable settings */

#undef CONFIG_PIC32MZ_DEBUGGER
#ifdef CONFIG_PIC32MZ_DEBUGGER_ENABLE
#  define CONFIG_PIC32MZ_DEBUGGER DEVCFG0_DEBUG_ENABLED
#else
#  define CONFIG_PIC32MZ_DEBUGGER DEVCFG0_DEBUG_DISABLED
#endif

#undef CONFIG_PIC32MZ_JTAGEN
#ifdef CONFIG_PIC32MZ_JTAG_ENABLE
#  define CONFIG_PIC32MZ_JTAGEN DEVCFG0_JTAGEN
#else
#  define CONFIG_PIC32MZ_JTAGEN 0
#endif

#undef CONFIG_PIC32MZ_ICESEL
#ifdef CONFIG_PIC32MZ_ICESEL_CH2
#  define CONFIG_PIC32MZ_ICESEL DEVCFG0_ICESEL_2
#else
#  define CONFIG_PIC32MZ_ICESEL DEVCFG0_ICESEL_1
#endif

#undef CONFIG_PIC32MZ_TRCEN
#ifdef CONFIG_PIC32MZ_TRACE_ENABLE
#  define CONFIG_PIC32MZ_TRCEN DEVCFG0_TRCEN
#else
#  define CONFIG_PIC32MZ_TRCEN 0
#endif

#ifdef CONFIG_MIPS_MICROMIPS
#  define CONFIG_PIC32MZ_BOOTISA DEVCFG0_BOOT_MICROMIPS
#else
#  define CONFIG_PIC32MZ_BOOTISA DEVCFG0_BOOT_MIPS32
#endif

#ifndef CONFIG_PIC32MZ_ECC_OPTION
#  define CONFIG_PIC32MZ_ECC_OPTION 3
#endif
#if CONFIG_PIC32MZ_ECC_OPTION < 0 || CONFIG_PIC32MZ_ECC_OPTION > 3
#  error Invalid CONFIG_PIC32MZ_ECC_OPTION Invalid
#  undef CONFIG_PIC32MZ_ECC_OPTION
#  define CONFIG_PIC32MZ_ECC_OPTION 3
#endif
#define CONFIG_PIC32MZ_FECCCON  (CONFIG_PIC32MZ_ECC_OPTION << DEVCFG0_FECCCON_SHIFT)

/* Not yet configurable settings */

#if defined(CONFIG_ARCH_CHIP_PIC32MZEC)
#  define CONFIG_PIC32MX_SMCLR      0
#  define CONFIG_PIC32MX_SOSCGAIN   0
#  define CONFIG_PIC32MX_SOSCBOOST  0
#  define CONFIG_PIC32MX_POSCGAIN   0
#  define CONFIG_PIC32MX_POSCBOOST  0
#elif defined(CONFIG_ARCH_CHIP_PIC32MZEF)
#  define CONFIG_PIC32MX_SMCLR      DEVCFG0_SMCLR
#  define CONFIG_PIC32MX_SOSCGAIN   DEVCFG0_SOSCGAIN_HIGH
#  define CONFIG_PIC32MX_SOSCBOOST  DEVCFG0_SOSCBOOST
#  define CONFIG_PIC32MX_POSCGAIN   DEVCFG0_POSCGAIN_HIGH
#  define CONFIG_PIC32MX_POSCBOOST  DEVCFG0_POSCBOOST
#endif

#define CONFIG_PIC32MZ_FSLEEP   DEVCFG0_FSLEEP_OFF
#define CONFIG_PIC32MZ_DBGPER   DEVCFG0_DBGPER_ALL
#define CONFIG_PIC32MZ_EJTAGBEN DEVCFG0_EJTAG_NORMAL

#else /* CONFIG_ARCH_CHIP_PIC32MZW1 */

/* PIC32MZ-W1 boot-flash configuration words, emitted by pic32mz_head.S.
 * Field layouts: hardware/pic32mzw1_features.h.  Values that NuttX relies
 * on are fixed; the rest follow the existing PIC32MZ Kconfig options.
 * Everything else matches factory-programmed parts.
 *
 * Only '|' and '<<' are used: the result must be usable by the assembler.
 */

/* FUSERID */

#define W1CFG_FUSERID           (CONFIG_PIC32MZ_USERID & 0xffff)

/* DEVCFG4: VBAT/Deep Sleep zero-power BOR on, DSWDT clocked by LPRC */

#define W1CFG_DEVCFG4           (DEVCFG4_VBZPBOREN | DEVCFG4_DSZPBOREN | \
                                 DEVCFG4_DSWDTOSC)

/* DEVCFG2.  CKSWEN is required: pic32mz_wfi32_pwrclk.c switches SYSCLK in
 * software.  DMT stays off; POSC in HS mode (40 MHz crystal).
 */

#ifdef CONFIG_PIC32MZ_WDTENABLE
#if BOARD_WD_PRESCALER == 1
#  define W1CFG_WDTPS            0
#elif BOARD_WD_PRESCALER == 2
#  define W1CFG_WDTPS            1
#elif BOARD_WD_PRESCALER == 4
#  define W1CFG_WDTPS            2
#elif BOARD_WD_PRESCALER == 8
#  define W1CFG_WDTPS            3
#elif BOARD_WD_PRESCALER == 16
#  define W1CFG_WDTPS            4
#elif BOARD_WD_PRESCALER == 32
#  define W1CFG_WDTPS            5
#elif BOARD_WD_PRESCALER == 64
#  define W1CFG_WDTPS            6
#elif BOARD_WD_PRESCALER == 128
#  define W1CFG_WDTPS            7
#elif BOARD_WD_PRESCALER == 256
#  define W1CFG_WDTPS            8
#elif BOARD_WD_PRESCALER == 512
#  define W1CFG_WDTPS            9
#elif BOARD_WD_PRESCALER == 1024
#  define W1CFG_WDTPS            10
#elif BOARD_WD_PRESCALER == 2048
#  define W1CFG_WDTPS            11
#elif BOARD_WD_PRESCALER == 4096
#  define W1CFG_WDTPS            12
#elif BOARD_WD_PRESCALER == 8192
#  define W1CFG_WDTPS            13
#elif BOARD_WD_PRESCALER == 16384
#  define W1CFG_WDTPS            14
#elif BOARD_WD_PRESCALER == 32768
#  define W1CFG_WDTPS            15
#elif BOARD_WD_PRESCALER == 65536
#  define W1CFG_WDTPS            16
#elif BOARD_WD_PRESCALER == 131072
#  define W1CFG_WDTPS            17
#elif BOARD_WD_PRESCALER == 262144
#  define W1CFG_WDTPS            18
#elif BOARD_WD_PRESCALER == 524288
#  define W1CFG_WDTPS            19
#elif BOARD_WD_PRESCALER == 1048576
#  define W1CFG_WDTPS            20
#else
#  error "Unsupported BOARD_WD_PRESCALER"
#endif
#  define W1CFG_WDTEN           DEVCFG2_WDTEN
#else
#  define W1CFG_WDTPS           0
#  define W1CFG_WDTEN           0
#endif

#define W1CFG_DEVCFG2           ((6 << DEVCFG2_DMTINTV_SHIFT) | \
                                 DEVCFG2_POSCMOD_HS | \
                                 (3 << DEVCFG2_WDTRMCS_SHIFT) | \
                                 DEVCFG2_SOSCSEL | DEVCFG2_WAKE2SPD | \
                                 DEVCFG2_CKSWEN | DEVCFG2_FSCMEN | \
                                 (W1CFG_WDTPS << DEVCFG2_WDTPS_SHIFT) | \
                                 DEVCFG2_WDTSPGM | DEVCFG2_WINDIS | \
                                 W1CFG_WDTEN | \
                                 (3 << DEVCFG2_WDTWINSZ_SHIFT) | \
                                 (0x17 << DEVCFG2_DMTCNT_SHIFT))

/* DEVCFG1.  UART1 and SPI1 use their dedicated (non-PPS) pins unless the
 * board routes them through PPS (BOARD_U1RX_PPS/BOARD_SDI1_PPS defined).
 * Ethernet in RMII mode (FMIIEN = 0) with the reference clock on
 * ETH_CLK_OUT (ETHEXEREF = 0).  PIC32MZ-W1 only has the RMII interface
 * [DS Tables 33-1, 33-2]; the Ethernet driver checks CONFIG_PIC32MZ_FMIIEN.
 */

#define CONFIG_PIC32MZ_FMIIEN   0

#ifdef CONFIG_PIC32MZ_DEBUGGER_ENABLE
#  define W1CFG_DEBUG           DEVCFG1_DEBUG_ENABLED
#else
#  define W1CFG_DEBUG           DEVCFG1_DEBUG_DISABLED
#endif

#ifdef CONFIG_PIC32MZ_ICESEL_CH2
#  define W1CFG_ICESEL          DEVCFG1_ICESEL_PGX2
#else
#  define W1CFG_ICESEL          DEVCFG1_ICESEL_PGX1
#endif

#ifdef CONFIG_PIC32MZ_TRACE_ENABLE
#  define W1CFG_TRCEN           DEVCFG1_TRCEN
#else
#  define W1CFG_TRCEN           0
#endif

#ifdef BOARD_U1RX_PPS
#  define W1CFG_HSUARTEN        0
#else
#  define W1CFG_HSUARTEN        DEVCFG1_HSUARTEN
#endif

#ifdef BOARD_SDI1_PPS
#  define W1CFG_HSSPIEN         0
#else
#  define W1CFG_HSSPIEN         DEVCFG1_HSSPIEN
#endif

#define W1CFG_DEVCFG1           (W1CFG_DEBUG | W1CFG_ICESEL | W1CFG_TRCEN | \
                                 DEVCFG1_CLASSBDIS | DEVCFG1_USBIDIO | \
                                 DEVCFG1_VBUSIO | W1CFG_HSSPIEN | \
                                 DEVCFG1_SMCLR | W1CFG_HSUARTEN)

/* DEVCFG0.  PMU/PMD/peripheral-pin/configuration locks off and all PLLs
 * under software control (pic32mz_wfi32_pwrclk.c programs them).
 */

#ifdef CONFIG_PIC32MZ_JTAG_ENABLE
#  define W1CFG_JTAGEN          DEVCFG0_JTAGEN
#else
#  define W1CFG_JTAGEN          0
#endif

#define W1CFG_DEVCFG0           (DEVCFG0_TDOEN | W1CFG_JTAGEN | \
                                 DEVCFG0_PCM | \
                                 (CONFIG_PIC32MZ_ECC_OPTION << \
                                  DEVCFG0_FECCCON_SHIFT))

/* FBCFG0: boot and exception code are MIPS32 (pic32mz_head.S);
 * BINFOVALID, PCSCMODE and BUHSWEN cleared.
 */

#define W1CFG_FBCFG0            FBCFG0_BOOTISA

#endif /* CONFIG_ARCH_CHIP_PIC32MZW1 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_MIPS_SRC_PIC32MZ_PIC32MZ_CONFIG_H */
