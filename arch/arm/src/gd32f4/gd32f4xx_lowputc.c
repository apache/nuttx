/****************************************************************************
 * arch/arm/src/gd32f4/gd32f4xx_lowputc.c
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

#include <stdint.h>

#include <arch/board/board.h>

#include "arm_internal.h"

#include "chip.h"

#include "gd32f4xx.h"
#include "gd32f4xx_rcu.h"
#include "gd32f4xx_gpio.h"
#include "gd32f4xx_usart.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Select USART parameters for the selected console */

#ifdef HAVE_CONSOLE
#  if defined(CONFIG_USART0_SERIAL_CONSOLE)
#    define GD32_CONSOLE_BASE          GD32_USART0
#    define GD32_CONSOLE_APBEN_REG     GD32_RCU_APB2EN
#    define GD32_CONSOLE_APBEN         RCU_APB2EN_USART0EN
#    define GD32_CONSOLE_CLOCK         GD32_PCLK2_FREQUENCY
#    define GD32_CONSOLE_BAUD          CONFIG_USART0_BAUD
#    define GD32_CONSOLE_PARITY        CONFIG_USART0_PARITY
#    define GD32_CONSOLE_NBITS         CONFIG_USART0_BITS
#    define GD32_CONSOLE_2STOP         CONFIG_USART0_2STOP
#    define GD32_CONSOLE_TX            GPIO_USART0_TX
#    define GD32_CONSOLE_RX            GPIO_USART0_RX
#    ifdef CONFIG_USART0_RS485
#      define GD32_CONSOLE_RS485_DIR   GPIO_USART0_RS485_DIR
#      if (CONFIG_USART0_RS485_DIR_POLARITY == 0)
#        define GD32_CONSOLE_RS485_DIR_POLARITY false
#      else
#        define GDM32_CONSOLE_RS485_DIR_POLARITY true
#      endif
#    endif
#  elif defined(CONFIG_USART1_SERIAL_CONSOLE)
#    define GD32_CONSOLE_BASE          GD32_USART1
#    define GD32_CONSOLE_APBEN_REG     GD32_RCU_APB1EN
#    define GD32_CONSOLE_APBEN         RCU_APB1EN_USART1EN
#    define GD32_CONSOLE_CLOCK         GD32_PCLK1_FREQUENCY
#    define GD32_CONSOLE_BAUD          CONFIG_USART1_BAUD
#    define GD32_CONSOLE_PARITY        CONFIG_USART1_PARITY
#    define GD32_CONSOLE_NBITS          CONFIG_USART1_BITS
#    define GD32_CONSOLE_2STOP         CONFIG_USART1_2STOP
#    define GD32_CONSOLE_TX            GPIO_USART1_TX
#    define GD32_CONSOLE_RX            GPIO_USART1_RX
#    ifdef CONFIG_USART1_RS485
#      define GD32_CONSOLE_RS485_DIR   GPIO_USART1_RS485_DIR
#      if (CONFIG_USART1_RS485_DIR_POLARITY == 0)
#        define GD32_CONSOLE_RS485_DIR_POLARITY false
#      else
#        define GD32_CONSOLE_RS485_DIR_POLARITY true
#      endif
#    endif
#  elif defined(CONFIG_USART2_SERIAL_CONSOLE)
#    define GD32_CONSOLE_BASE          GD32_USART2
#    define GD32_CONSOLE_APBEN_REG     GD32_RCU_APB1EN
#    define GD32_CONSOLE_APBEN         RCU_APB1ENR_USART3EN
#    define GD32_CONSOLE_CLOCK         GD32_PCLK1_FREQUENCY
#    define GD32_CONSOLE_BAUD          CONFIG_USART2_BAUD
#    define GD32_CONSOLE_PARITY        CONFIG_USART2_PARITY
#    define GD32_CONSOLE_NBITS         CONFIG_USART2_BITS
#    define GD32_CONSOLE_2STOP         CONFIG_USART2_2STOP
#    define GD32_CONSOLE_TX            GPIO_USART2_TX
#    define GD32_CONSOLE_RX            GPIO_USART2_RX
#    ifdef CONFIG_USART2_RS485
#      define GD32_CONSOLE_RS485_DIR   GPIO_USART2_RS485_DIR
#      if (CONFIG_USART2_RS485_DIR_POLARITY == 0)
#        define GD32_CONSOLE_RS485_DIR_POLARITY false
#      else
#        define GD32_CONSOLE_RS485_DIR_POLARITY true
#      endif
#    endif
#  elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#    define GD32_CONSOLE_BASE          GD32_UART3
#    define GD32_CONSOLE_APBEN_REG     GD32_RCU_APB1EN
#    define GD32_CONSOLE_APBEN         RCU_APB1EN_UART3EN
#    define GD32_CONSOLE_CLOCK         GD32_PCLK1_FREQUENCY
#    define GD32_CONSOLE_BAUD          CONFIG_UART3_BAUD
#    define GD32_CONSOLE_PARITY        CONFIG_UART3_PARITY
#    define GD32_CONSOLE_NBITS         CONFIG_UART3_BITS
#    define GD32_CONSOLE_2STOP         CONFIG_UART3_2STOP
#    define GD32_CONSOLE_TX            GPIO_UART3_TX
#    define GD32_CONSOLE_RX            GPIO_UART3_RX
#    ifdef CONFIG_UART3_RS485
#      define GD32_CONSOLE_RS485_DIR   GPIO_UART3_RS485_DIR
#      if (CONFIG_UART3_RS485_DIR_POLARITY == 0)
#        define GD32_CONSOLE_RS485_DIR_POLARITY false
#      else
#        define GD32_CONSOLE_RS485_DIR_POLARITY true
#      endif
#    endif
#  elif defined(CONFIG_UART4_SERIAL_CONSOLE)
#    define GD32_CONSOLE_BASE          GD32_UART4
#    define GD32_CONSOLE_APBEN_REG     GD32_RCU_APB1EN
#    define GD32_CONSOLE_APBEN         RCU_APB1EN_UART4EN
#    define GD32_CONSOLE_CLOCK         GD32_PCLK1_FREQUENCY
#    define GD32_CONSOLE_BAUD          CONFIG_UART4_BAUD
#    define GD32_CONSOLE_PARITY        CONFIG_UART4_PARITY
#    define GD32_CONSOLE_NBITS         CONFIG_UART4_BITS
#    define GD32_CONSOLE_2STOP         CONFIG_UART4_2STOP
#    define GD32_CONSOLE_TX            GPIO_UART4_TX
#    define GD32_CONSOLE_RX            GPIO_UART4_RX
#    ifdef CONFIG_UART4_RS485
#      define GD32_CONSOLE_RS485_DIR   GPIO_UART4_RS485_DIR
#      if (CONFIG_UART4_RS485_DIR_POLARITY == 0)
#        define GD32_CONSOLE_RS485_DIR_POLARITY false
#      else
#        define GD32_CONSOLE_RS485_DIR_POLARITY true
#      endif
#    endif
#  elif defined(CONFIG_USART5_SERIAL_CONSOLE)
#    define GD32_CONSOLE_BASE          GD32_USART5
#    define GD32_CONSOLE_APBEN_REG     GD32_RCU_APB2EN
#    define GD32_CONSOLE_APBEN         RCU_APB2EN_USART5EN
#    define GD32_CONSOLE_CLOCK         GD32_PCLK2_FREQUENCY
#    define GD32_CONSOLE_BAUD          CONFIG_USART5_BAUD
#    define GD32_CONSOLE_PARITY        CONFIG_USART5_PARITY
#    define GD32_CONSOLE_NBITS         CONFIG_USART5_BITS
#    define GD32_CONSOLE_2STOP         CONFIG_USART5_2STOP
#    define GD32_CONSOLE_TX            GPIO_USART5_TX
#    define GD32_CONSOLE_RX            GPIO_USART5_RX
#    ifdef CONFIG_USART5_RS485
#      define GD32_CONSOLE_RS485_DIR   GPIO_USART5_RS485_DIR
#      if (CONFIG_USART5_RS485_DIR_POLARITY == 0)
#        define GD32_CONSOLE_RS485_DIR_POLARITY false
#      else
#        define GD32_CONSOLE_RS485_DIR_POLARITY true
#      endif
#    endif
#  elif defined(CONFIG_UART6_SERIAL_CONSOLE)
#    define GD32_CONSOLE_BASE          GD32_UART6
#    define GD32_CONSOLE_APBEN_REG     GD32_RCU_APB1EN
#    define GD32_CONSOLE_APBEN         RCU_APB1EN_UART6EN
#    define GD32_CONSOLE_CLOCK         GD32_PCLK1_FREQUENCY
#    define GD32_CONSOLE_BAUD          CONFIG_UART6_BAUD
#    define GD32_CONSOLE_PARITY        CONFIG_UART6_PARITY
#    define GD32_CONSOLE_NBITS         CONFIG_UART6_BITS
#    define GD32_CONSOLE_2STOP         CONFIG_UART6_2STOP
#    define GD32_CONSOLE_TX            GPIO_UART6_TX
#    define GD32_CONSOLE_RX            GPIO_UART6_RX
#    ifdef CONFIG_UART6_RS485
#      define GD32_CONSOLE_RS485_DIR   GPIO_UART6_RS485_DIR
#      if (CONFIG_UART6_RS485_DIR_POLARITY == 0)
#        define GD32_CONSOLE_RS485_DIR_POLARITY false
#      else
#        define GD32_CONSOLE_RS485_DIR_POLARITY true
#      endif
#    endif
#  elif defined(CONFIG_UART7_SERIAL_CONSOLE)
#    define GD32_CONSOLE_BASE          GD32_UART7
#    define GD32_CONSOLE_APBEN_REG     GD32_RCU_APB1EN
#    define GD32_CONSOLE_APBEN         RCU_APB1ENR_UART7EN
#    define GD32_CONSOLE_CLOCK         GD32_PCLK1_FREQUENCY
#    define GD32_CONSOLE_BAUD          CONFIG_UART7_BAUD
#    define GD32_CONSOLE_PARITY        CONFIG_UART7_PARITY
#    define GD32_CONSOLE_NBITS         CONFIG_UART7_BITS
#    define GD32_CONSOLE_2STOP         CONFIG_UART7_2STOP
#    define GD32_CONSOLE_TX            GPIO_UART7_TX
#    define GD32_CONSOLE_RX            GPIO_UART7_RX
#    ifdef CONFIG_UART7_RS485
#      define GD32_CONSOLE_RS485_DIR   GPIO_UART7_RS485_DIR
#      if (CONFIG_UART7_RS485_DIR_POLARITY == 0)
#        define GD32_CONSOLE_RS485_DIR_POLARITY false
#      else
#        define GD32_CONSOLE_RS485_DIR_POLARITY true
#      endif
#    endif
#  endif

/* CTL0 settings */

#  if GD32_CONSOLE_NBITS == 9
#    define USART_CTL0_WL_VALUE      USART_WL_9BIT
#  else
#    define USART_CTL0_WL_VALUE      USART_WL_8BIT
#  endif

#  if GD32_CONSOLE_PARITY == 1
#    define USART_CTL0_PARITY_VALUE  USART_CTL0_PM_ODD
#  elif GD32_CONSOLE_PARITY == 2
#    define USART_CTL0_PARITY_VALUE  USART_CTL0_PM_EVEN
#  else
#    define USART_CTL0_PARITY_VALUE  USART_CTL0_PM_NONE
#  endif

/* CTL1 settings */

#  if GD32_CONSOLE_2STOP != 0
#    define USART_USART_CTL1_STB2BIT_VALUE  USART_CTL1_STB2BIT
#  else
#    define USART_USART_CTL1_STB2BIT_VALUE  USART_CTL1_STB1BIT
#  endif

#endif /* HAVE_CONSOLE */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ****************************************************************************/

void arm_lowputc(char ch)
{
#if defined (HAVE_CONSOLE)
  /* Wait until the transmit data register is empty */

  while ((getreg32(GD32_CONSOLE_BASE + GD32_USART_STAT0_OFFSET) &
         USART_STAT0_TBE) == 0);
#ifdef GD32_CONSOLE_RS485_DIR
  gd32_gpio_write(GD32_CONSOLE_RS485_DIR,
                  GD32_CONSOLE_RS485_DIR_POLARITY);
#endif

  /* Then send the character */

  putreg32((uint32_t)(ch & USART_DATA_MASK), GD32_CONSOLE_BASE +
          GD32_USART_DATA_OFFSET);

#ifdef GD32_CONSOLE_RS485_DIR
  while ((getreg32(GD32_CONSOLE_BASE + GD32_USART_STAT0_OFFSET) &
        USART_STAT0_TC) == 0);
  gd32_gpio_write(GD32_CONSOLE_RS485_DIR,
                !GD32_CONSOLE_RS485_DIR_POLARITY);
#endif

#endif /* HAVE_CONSOLE */
}

/****************************************************************************
 * Name: gd32_lowsetup
 *
 * Description:
 *   This performs basic initialization of the USART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 ****************************************************************************/

void gd32_lowsetup(void)
{
#if defined(HAVE_SERIALDRIVER)
#if defined(HAVE_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)
  uint32_t regval;
  uint32_t udiv;
  uint32_t intdiv;
  uint32_t fradiv;
#endif

#if defined(HAVE_CONSOLE)
  /* Enable USART APB1/2 clock */

  modifyreg32(GD32_CONSOLE_APBEN_REG, 0, GD32_CONSOLE_APBEN);
#endif

  /* Enable the console USART and configure TX, RX pins. */

#ifdef GD32_CONSOLE_TX
  gd32_gpio_config(GD32_CONSOLE_TX);
#endif
#ifdef GD32_CONSOLE_RX
  gd32_gpio_config(GD32_CONSOLE_RX);
#endif

#ifdef GD32_CONSOLE_RS485_DIR
  gd32_gpio_config(GD32_CONSOLE_RS485_DIR);
  gd32_gpio_write(GD32_CONSOLE_RS485_DIR,
                  !GD32_CONSOLE_RS485_DIR_POLARITY);
#endif

  /* Enable and configure the selected console device */

#if defined(HAVE_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)

  /* Reset USART */

  gd32_usart_reset(GD32_CONSOLE_BASE);

  /* Enable USART clock */

  gd32_usart_clock_enable(GD32_CONSOLE_BASE);

  /* Disabled the USART before to configure it. */

  regval  = getreg32(GD32_CONSOLE_BASE + GD32_USART_CTL0_OFFSET);
  regval &= ~USART_CTL0_UEN;
  putreg32(regval, GD32_CONSOLE_BASE + GD32_USART_CTL0_OFFSET);

  /* Configure CTL0 */

  regval  = getreg32(GD32_CONSOLE_BASE + GD32_USART_CTL0_OFFSET);
  regval |= (USART_CTL0_WL_VALUE | USART_CTL0_PARITY_VALUE);
  putreg32(regval, GD32_CONSOLE_BASE + GD32_USART_CTL0_OFFSET);

  /* Configure CTL1 */

  regval  = getreg32(GD32_CONSOLE_BASE + GD32_USART_CTL1_OFFSET);
  regval |= USART_USART_CTL1_STB2BIT_VALUE;
  putreg32(regval, GD32_CONSOLE_BASE + GD32_USART_CTL1_OFFSET);

  /* Configure USART baud rate value */

  regval = getreg32(GD32_CONSOLE_BASE + GD32_USART_CTL0_OFFSET);

  if ((regval & USART_CTL0_OVSMOD) == USART_OVSMOD_8)
    {
      /* When oversampling by 8, configure the value of USART_BAUD */

      udiv = ((GD32_CONSOLE_CLOCK * 2) + GD32_CONSOLE_BAUD / 2) /
              GD32_CONSOLE_BAUD;
      intdiv = udiv & 0xfff0;
      fradiv = (udiv >> 1) & 0x7u;
    }
  else
    {
      /* When oversampling by 16, configure the value of USART_BAUD */

      udiv = (GD32_CONSOLE_CLOCK + GD32_CONSOLE_BAUD / 2) /
              GD32_CONSOLE_BAUD;
      intdiv = udiv & 0xfff0;
      fradiv = udiv & 0xf;
    }

  regval = ((USART_BAUD_FRADIV_MASK | USART_BAUD_INTDIV_MASK) &
            (intdiv | fradiv));
  putreg32(regval, GD32_CONSOLE_BASE + GD32_USART_BAUD_OFFSET);

  /* Enable Rx, Tx, and the USART */

  regval = getreg32(GD32_CONSOLE_BASE + GD32_USART_CTL0_OFFSET);
  regval |= (USART_CTL0_UEN | USART_CTL0_TEN | USART_CTL0_REN);
  putreg32(regval, GD32_CONSOLE_BASE + GD32_USART_CTL0_OFFSET);

#endif /* HAVE_CONSOLE && !CONFIG_SUPPRESS_UART_CONFIG */
#endif /* HAVE_SERIALDRIVER */
}

/****************************************************************************
 * Name: gd32_usart_reset
 *
 * Description:
 *   Reset the USART.
 *
 ****************************************************************************/

void gd32_usart_reset(uint32_t usartbase)
{
  uint32_t rcu_rst;
  uint32_t regaddr;

  /* Determine which USART to configure */

  switch (usartbase)
    {
    default:
      return;
#ifdef CONFIG_GD32F4_USART0_SERIALDRIVER
    case GD32_USART0:
      rcu_rst = RCU_APB2RST_USART0RST;
      regaddr = GD32_RCU_APB2RST;
      break;
#endif
#ifdef CONFIG_GD32F4_USART1_SERIALDRIVER
    case GD32_USART1:
      rcu_rst = RCU_APB1RST_USART1RST;
      regaddr = GD32_RCU_APB1RST;
      break;
#endif
#ifdef CONFIG_GD32F4_USART2_SERIALDRIVER
    case GD32_USART2:
      rcu_rst = RCU_APB1RST_USART2RST;
      regaddr = GD32_RCU_APB1RST;
      break;
#endif
#ifdef CONFIG_GD32F4_UART3_SERIALDRIVER
    case GD32_UART3:
      rcu_rst = RCU_APB1RST_UART3RST;
      regaddr = GD32_RCU_APB1RST;
      break;
#endif
#ifdef CONFIG_GD32F4_UART4_SERIALDRIVER
    case GD32_UART4:
      rcu_rst = RCU_APB1RST_UART4RST;
      regaddr = GD32_RCU_APB1RST;
      break;
#endif
#ifdef CONFIG_GD32F4_USART5_SERIALDRIVER
    case GD32_USART5:
      rcu_rst = RCU_APB2RST_USART5RST;
      regaddr = GD32_RCU_APB2RST;
      break;
#endif
#ifdef CONFIG_GD32F4_UART6_SERIALDRIVER
    case GD32_UART6:
      rcu_rst = RCU_APB1RST_UART6RST;
      regaddr = GD32_RCU_APB1RST;
      break;
#endif
#ifdef CONFIG_GD32F4_UART7_SERIALDRIVER
    case GD32_UART7:
      rcu_rst = RCU_APB1RST_UART7RST;
      regaddr = GD32_RCU_APB1RST;
      break;
#endif
    }

  /* Enable APB 1/2 reset for USART */

  modifyreg32(regaddr, 0, rcu_rst);

  /* Disable APB 1/2 reset for USART */

  modifyreg32(regaddr, rcu_rst, 0);
}

/****************************************************************************
 * Name: gd32_usart_clock_enable
 *
 * Description:
 *   Enable USART clock
 ****************************************************************************/

void gd32_usart_clock_enable(uint32_t usartbase)
{
  uint32_t rcu_en;
  uint32_t regaddr;

  /* Determine which USART to configure */

  switch (usartbase)
    {
    default:
      return;
#ifdef CONFIG_GD32F4_USART0_SERIALDRIVER
    case GD32_USART0:
      rcu_en = RCU_APB2EN_USART0EN;
      regaddr = GD32_RCU_APB2EN;
      break;
#endif
#ifdef CONFIG_GD32F4_USART1_SERIALDRIVER
    case GD32_USART1:
      rcu_en = RCU_APB1EN_USART1EN;
      regaddr = GD32_RCU_APB1EN;
      break;
#endif
#ifdef CONFIG_GD32F4_USART2_SERIALDRIVER
    case GD32_USART2:
      rcu_en = RCU_APB1EN_USART2EN;
      regaddr = GD32_RCU_APB1EN;
      break;
#endif
#ifdef CONFIG_GD32F4_UART3_SERIALDRIVER
    case GD32_UART3:
      rcu_en = RCU_APB1EN_UART3EN;
      regaddr = GD32_RCU_APB1EN;
      break;
#endif
#ifdef CONFIG_GD32F4_UART4_SERIALDRIVER
    case GD32_UART4:
      rcu_en = RCU_APB1EN_UART4EN;
      regaddr = GD32_RCU_APB1EN;
      break;
#endif
#ifdef CONFIG_GD32F4_USART5_SERIALDRIVER
    case GD32_USART5:
      rcu_en = RCU_APB2EN_USART5EN;
      regaddr = GD32_RCU_APB2EN;
      break;
#endif
#ifdef CONFIG_GD32F4_UART6_SERIALDRIVER
    case GD32_UART6:
      rcu_en = RCU_APB1EN_UART6EN;
      regaddr = GD32_RCU_APB1EN;
      break;
#endif
#ifdef CONFIG_GD32F4_UART7_SERIALDRIVER
    case GD32_UART7:
      rcu_en = RCU_APB1EN_UART7EN;
      regaddr = GD32_RCU_APB1EN;
      break;
#endif
    }

  /* Enable APB 1/2 clock for USART */

  modifyreg32(regaddr, 0, rcu_en);
}

/****************************************************************************
 * Name: gd32_usart_clock_disable
 *
 * Description:
 *   Dinable USART clock
 ****************************************************************************/

void gd32_usart_clock_disable(uint32_t usartbase)
{
  uint32_t rcu_en;
  uint32_t regaddr;

  /* Determine which USART to configure */

  switch (usartbase)
    {
    default:
      return;
#ifdef CONFIG_GD32F4_USART0_SERIALDRIVER
    case GD32_USART0:
      rcu_en = RCU_APB2EN_USART0EN;
      regaddr = GD32_RCU_APB2EN;
      break;
#endif
#ifdef CONFIG_GD32F4_USART1_SERIALDRIVER
    case GD32_USART1:
      rcu_en = RCU_APB1EN_USART1EN;
      regaddr = GD32_RCU_APB1EN;
      break;
#endif
#ifdef CONFIG_GD32F4_USART2_SERIALDRIVER
    case GD32_USART2:
      rcu_en = RCU_APB1EN_USART2EN;
      regaddr = GD32_RCU_APB1EN;
      break;
#endif
#ifdef CONFIG_GD32F4_UART3_SERIALDRIVER
    case GD32_UART3:
      rcu_en = RCU_APB1EN_UART3EN;
      regaddr = GD32_RCU_APB1EN;
      break;
#endif
#ifdef CONFIG_GD32F4_UART4_SERIALDRIVER
    case GD32_UART4:
      rcu_en = RCU_APB1EN_UART4EN;
      regaddr = GD32_RCU_APB1EN;
      break;
#endif
#ifdef CONFIG_GD32F4_USART5_SERIALDRIVER
    case GD32_USART5:
      rcu_en = RCU_APB2EN_USART5EN;
      regaddr = GD32_RCU_APB2EN;
      break;
#endif
#ifdef CONFIG_GD32F4_UART6_SERIALDRIVER
    case GD32_UART6:
      rcu_en = RCU_APB1EN_UART6EN;
      regaddr = GD32_RCU_APB1EN;
      break;
#endif
#ifdef CONFIG_GD32F4_UART7_SERIALDRIVER
    case GD32_UART7:
      rcu_en = RCU_APB1EN_UART7EN;
      regaddr = GD32_RCU_APB1EN;
      break;
#endif
    }

  /* Disable APB 1/2 clock for USART */

  modifyreg32(regaddr, rcu_en, 0);
}
