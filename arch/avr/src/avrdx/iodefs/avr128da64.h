/****************************************************************************
 * arch/avr/src/avrdx/iodefs/avr128da64.h
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

#ifndef __ARCH_AVR_SRC_AVRDX_IODEFS_AVR128DA64_H
#define __ARCH_AVR_SRC_AVRDX_IODEFS_AVR128DA64_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <avr/io.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef __ARCH_AVR_SRC_AVRDX_AVRDX_IODEFS_H
#  error "Do not include this file directly, use avrdx_iodefs.h instead"
#endif

/* PORT.PINCONFIG */

#define PORT_ISC_GM ( PORT_ISC_0_bm | PORT_ISC_1_bm | PORT_ISC_2_bm )

#define PORT_ISC_BOTHEDGES_GC ( PORT_ISC_0_bm )

/* CLKCTRL.MCLKCTRLB */

#define CLKCTRL_PDIV_GM ( CLKCTRL_PDIV_0_bm | CLKCTRL_PDIV_1_bm | \
                          CLKCTRL_PDIV_2_bm | CLKCTRL_PDIV_3_bm )
#define CLKCTRL_PDIV_GP (1)

/* CLKCTRL.OSCHFCTRLA */

#define CLKCTRL_FRQSEL_GM ( CLKCTRL_FRQSEL_0_bm | CLKCTRL_FRQSEL_1_bm | \
                            CLKCTRL_FRQSEL_2_bm | CLKCTRL_FRQSEL_3_bm )
#define CLKCTRL_FRQSEL_GP (2)

#define CLKCTRL_FRQSEL_1M_GC (0)
#define CLKCTRL_FRQSEL_2M_GC (CLKCTRL_FRQSEL_0_bm)
#define CLKCTRL_FRQSEL_3M_GC (CLKCTRL_FRQSEL_1_bm)
#define CLKCTRL_FRQSEL_4M_GC (CLKCTRL_FRQSEL_1_bm | CLKCTRL_FRQSEL_0_bm)
#define CLKCTRL_FRQSEL_8M_GC (CLKCTRL_FRQSEL_2_bm | CLKCTRL_FRQSEL_0_bm)
#define CLKCTRL_FRQSEL_12M_GC (CLKCTRL_FRQSEL_2_bm | CLKCTRL_FRQSEL_1_bm)
#define CLKCTRL_FRQSEL_16M_GC (CLKCTRL_FRQSEL_2_bm | CLKCTRL_FRQSEL_1_bm | \
                               CLKCTRL_FRQSEL_0_bm )
#define CLKCTRL_FRQSEL_20M_GC (CLKCTRL_FRQSEL_3_bm)
#define CLKCTRL_FRQSEL_24M_GC (CLKCTRL_FRQSEL_3_bm | CLKCTRL_FRQSEL_0_bm)

/* RTC.CTRLA */

#define RTC_PRESCALER_GM ( RTC_PRESCALER_0_bm | RTC_PRESCALER_1_bm | \
                           RTC_PRESCALER_2_bm | RTC_PRESCALER_3_bm )

#define RTC_PRESCALER_DIV2_GC (RTC_PRESCALER_0_bm)

/* USART.CTRLB */

#define USART_RXMODE_NORMAL_GC (0)
#define USART_RXMODE_CLK2X_GC (USART_RXMODE_0_bm)

/* USART.CTRLC */

#define USART_PMODE_DISABLED_GC (0)
#define USART_PMODE_EVEN_GC = (USART_PMODE_1_bm)
#define USART_PMODE_ODD_GC = (USART_PMODE_1_bm | USART_PMODE_0_bm)

#define USART_SBMODE_1BIT_GC (0)
#define USART_SBMODE_2BIT_GC (USART_SBMODE_bm)

#define USART_CHSIZE_7BIT_GC (USART_CHSIZE_1_bm)
#define USART_CHSIZE_8BIT_GC (USART_CHSIZE_1_bm | USART_CHSIZE_0_bm)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

typedef struct avr_usart_struct
{
  register8_t RXDATAL;  /* Receive Data Low Byte */
  register8_t RXDATAH;  /* Receive Data High Byte */
  register8_t TXDATAL;  /* Transmit Data Low Byte */
  register8_t TXDATAH;  /* Transmit Data High Byte */
  register8_t STATUS;   /* Status */
  register8_t CTRLA;    /* Control A */
  register8_t CTRLB;    /* Control B */
  register8_t CTRLC;    /* Control C */
  _WORDREGISTER(BAUD);  /* Baud Rate */
  register8_t CTRLD;    /* Control D */
  register8_t DBGCTRL;  /* Debug Control */
  register8_t EVCTRL;   /* Event Control */
  register8_t TXPLCTRL; /* IRCOM Transmitter Pulse Length Control */
  register8_t RXPLCTRL; /* IRCOM Receiver Pulse Length Control */
  register8_t reserved_1[1];
} avr_usart_t;

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
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_AVR_SRC_AVRDX_AVRDX_H */
