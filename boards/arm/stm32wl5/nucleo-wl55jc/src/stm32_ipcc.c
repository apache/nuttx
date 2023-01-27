/****************************************************************************
 * boards/arm/stm32wl5/nucleo-wl55jc/src/stm32_ipcc.c
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
#include <nuttx/ipcc.h>
#include <debug.h>

#include <stm32wl5_ipcc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Define default values for macros if they are not define in config */

#ifndef CONFIG_ARCH_BOARD_IPCC_CHAN1_RXBUF
#  define CONFIG_ARCH_BOARD_IPCC_CHAN1_RXBUF 0
#endif

#ifndef CONFIG_ARCH_BOARD_IPCC_CHAN1_TXBUF
#  define CONFIG_ARCH_BOARD_IPCC_CHAN1_TXBUF 0
#endif

#ifndef CONFIG_ARCH_BOARD_IPCC_CHAN2_RXBUF
#  define CONFIG_ARCH_BOARD_IPCC_CHAN2_RXBUF 0
#endif

#ifndef CONFIG_ARCH_BOARD_IPCC_CHAN2_TXBUF
#  define CONFIG_ARCH_BOARD_IPCC_CHAN2_TXBUF 0
#endif

#ifndef CONFIG_ARCH_BOARD_IPCC_CHAN3_RXBUF
#  define CONFIG_ARCH_BOARD_IPCC_CHAN3_RXBUF 0
#endif

#ifndef CONFIG_ARCH_BOARD_IPCC_CHAN3_TXBUF
#  define CONFIG_ARCH_BOARD_IPCC_CHAN3_TXBUF 0
#endif

#ifndef CONFIG_ARCH_BOARD_IPCC_CHAN4_RXBUF
#  define CONFIG_ARCH_BOARD_IPCC_CHAN4_RXBUF 0
#endif

#ifndef CONFIG_ARCH_BOARD_IPCC_CHAN4_TXBUF
#  define CONFIG_ARCH_BOARD_IPCC_CHAN4_TXBUF 0
#endif

#ifndef CONFIG_ARCH_BOARD_IPCC_CHAN5_RXBUF
#  define CONFIG_ARCH_BOARD_IPCC_CHAN5_RXBUF 0
#endif

#ifndef CONFIG_ARCH_BOARD_IPCC_CHAN5_TXBUF
#  define CONFIG_ARCH_BOARD_IPCC_CHAN5_TXBUF 0
#endif

#ifndef CONFIG_ARCH_BOARD_IPCC_CHAN6_RXBUF
#  define CONFIG_ARCH_BOARD_IPCC_CHAN6_RXBUF 0
#endif

#ifndef CONFIG_ARCH_BOARD_IPCC_CHAN6_TXBUF
#  define CONFIG_ARCH_BOARD_IPCC_CHAN6_TXBUF 0
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int init_ipcc(int chan, size_t rxbuflen, size_t txbuflen);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: init_ipcc
 *
 * Description:
 *   Initializes IPCC channel with tx and rx buffer sizes. If ipcc is
 *   unbuffered, rxbuflen and txbuflen are ignored.
 *
 * Input Parameters:
 *   chan - channel number, indexed from 0
 *   rxbuflen - size of rxbuffer for buffered transactions
 *   txbuflen - size of txbuffer for buffered transactions
 *
 * Returned Value:
 *   0 on success or -1 on errors.
 *
 ****************************************************************************/

static int init_ipcc(int chan, size_t rxbuflen, size_t txbuflen)
{
  struct ipcc_lower_s *ipcc;
  int ret;

  if ((ipcc = stm32wl5_ipcc_init(chan)) == NULL)
    {
      syslog(LOG_ERR, "ERROR: stm32wl5_ipcc_init(%d) failed\n", chan);
      return -1;
    }

#ifdef CONFIG_IPCC_BUFFERED
  ret = ipcc_register(ipcc, rxbuflen, txbuflen);
#else
  UNUSED(rxbuflen);
  UNUSED(txbuflen);
  ret = ipcc_register(ipcc);
#endif

  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: ipcc_register() failed: %d, channel: %d\n",
             ret, chan);
      return -1;
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int ipcc_init(void)
{
  int ret = 0;

  /* First channel is always enabled in IPCC is enabled */

  ret |= init_ipcc(0, CONFIG_ARCH_BOARD_IPCC_CHAN1_RXBUF,
                   CONFIG_ARCH_BOARD_IPCC_CHAN1_TXBUF);

#ifdef CONFIG_ARCH_BOARD_IPCC_CHAN2
  ret |= init_ipcc(1, CONFIG_ARCH_BOARD_IPCC_CHAN2_RXBUF,
                   CONFIG_ARCH_BOARD_IPCC_CHAN2_TXBUF);
#endif

#ifdef CONFIG_ARCH_BOARD_IPCC_CHAN3
  ret |= init_ipcc(2, CONFIG_ARCH_BOARD_IPCC_CHAN3_RXBUF,
                   CONFIG_ARCH_BOARD_IPCC_CHAN3_TXBUF);
#endif

#ifdef CONFIG_ARCH_BOARD_IPCC_CHAN4
  ret |= init_ipcc(3, CONFIG_ARCH_BOARD_IPCC_CHAN4_RXBUF,
                   CONFIG_ARCH_BOARD_IPCC_CHAN4_TXBUF);
#endif

#ifdef CONFIG_ARCH_BOARD_IPCC_CHAN5
  ret |= init_ipcc(4, CONFIG_ARCH_BOARD_IPCC_CHAN5_RXBUF,
                   CONFIG_ARCH_BOARD_IPCC_CHAN5_TXBUF);
#endif

#ifdef CONFIG_ARCH_BOARD_IPCC_CHAN6
  ret |= init_ipcc(5, CONFIG_ARCH_BOARD_IPCC_CHAN6_RXBUF,
                   CONFIG_ARCH_BOARD_IPCC_CHAN6_TXBUF);
#endif

  return ret;
}
