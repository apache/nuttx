/****************************************************************************
 * arch/arm/src/stm32/stm32_1wire.c
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

/* Links:
 * https://www.maximintegrated.com/en/app-notes/index.mvp/id/214
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/clock.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/1wire/1wire.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "stm32_rcc.h"
#include "stm32_1wire.h"

#ifdef HAVE_1WIREDRIVER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BUS_TIMEOUT     5      /* tv_sec */

#define RESET_BAUD      9600
#define RESET_TX        0xF0
#define TIMESLOT_BAUD   115200
#define READ_TX         0xFF
#define READ_RX1        0xFF
#define WRITE_TX0       0x00
#define WRITE_TX1       0xFF

#if defined(CONFIG_STM32_STM32F10XX)
#  define PIN_OPENDRAIN(GPIO) ((GPIO) | GPIO_CNF_OUTOD)
#else
#  define PIN_OPENDRAIN(GPIO) ((GPIO) | GPIO_OPENDRAIN)
#endif

#if defined(CONFIG_STM32_STM32F10XX)
#  define USART_CR3_ONEBIT (0)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* 1-Wire bus task */

enum stm32_1wire_msg_e
{
  ONEWIRETASK_NONE = 0,
  ONEWIRETASK_RESET,
  ONEWIRETASK_WRITE,
  ONEWIRETASK_READ,
  ONEWIRETASK_WRITEBIT,
  ONEWIRETASK_READBIT
};

struct stm32_1wire_msg_s
{
  enum stm32_1wire_msg_e task;    /* Task */
  uint8_t *buffer;                /* Task buffer */
  int      buflen;                /* Buffer length */
};

/* 1-Wire device hardware configuration */

struct stm32_1wire_config_s
{
  const uint32_t usartbase; /* Base address of USART registers */
  const uint32_t apbclock;  /* PCLK 1 or 2 frequency */
  const uint32_t data_pin;  /* GPIO configuration for DATA */
  const uint8_t  irq;       /* IRQ associated with this USART */
};

/* 1-Wire device Private Data */

struct stm32_1wire_priv_s
{
  const struct stm32_1wire_config_s *config; /* Port configuration */
  volatile int refs;                         /* Reference count */
  mutex_t  lock;                             /* Mutual exclusion mutex */
  sem_t    sem_isr;                          /* Interrupt wait semaphore */
  int      baud;                             /* Baud rate */
  const struct stm32_1wire_msg_s *msgs;      /* Messages data */
  uint8_t *byte;                             /* Current byte */
  uint8_t  bit;                              /* Current bit */
  volatile int result;                       /* Exchange result */
};

/* 1-Wire device, Instance */

struct stm32_1wire_inst_s
{
  const struct onewire_ops_s *ops;  /* Standard 1-Wire operations */
  struct stm32_1wire_priv_s  *priv; /* Common driver private data structure */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline uint32_t stm32_1wire_in(struct stm32_1wire_priv_s *priv,
                                      int offset);
static inline void stm32_1wire_out(struct stm32_1wire_priv_s *priv,
                                   int offset, uint32_t value);
static int stm32_1wire_recv(struct stm32_1wire_priv_s *priv);
static void stm32_1wire_send(struct stm32_1wire_priv_s *priv, int ch);
static void stm32_1wire_set_baud(struct stm32_1wire_priv_s *priv);
static void stm32_1wire_set_apb_clock(struct stm32_1wire_priv_s *priv,
                                      bool on);
static int stm32_1wire_init(struct stm32_1wire_priv_s *priv);
static int stm32_1wire_deinit(struct stm32_1wire_priv_s *priv);
static int stm32_1wire_process(struct stm32_1wire_priv_s *priv,
                               const struct stm32_1wire_msg_s *msgs,
                               int count);
static int stm32_1wire_isr(int irq, void *context, void *arg);
static int stm32_1wire_reset(struct onewire_dev_s *dev);
static int stm32_1wire_write(struct onewire_dev_s *dev,
                             const uint8_t *buffer, int buflen);
static int stm32_1wire_read(struct onewire_dev_s *dev, uint8_t *buffer,
                            int buflen);
static int stm32_1wire_exchange(struct onewire_dev_s *dev, bool reset,
                                const uint8_t *txbuffer, int txbuflen,
                                uint8_t *rxbuffer, int rxbuflen);
static int stm32_1wire_writebit(struct onewire_dev_s *dev,
                                const uint8_t *bit);
static int stm32_1wire_readbit(struct onewire_dev_s *dev, uint8_t *bit);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* 1-Wire device structures */

#ifdef CONFIG_STM32_USART1_1WIREDRIVER

static const struct stm32_1wire_config_s stm32_1wire1_config =
{
  .usartbase  = STM32_USART1_BASE,
  .apbclock   = STM32_PCLK2_FREQUENCY,
  .data_pin   = PIN_OPENDRAIN(GPIO_USART1_TX),
  .irq        = STM32_IRQ_USART1,
};

static struct stm32_1wire_priv_s stm32_1wire1_priv =
{
  .config     = &stm32_1wire1_config,
  .refs       = 0,
  .lock       = NXMUTEX_INITIALIZER,
  .sem_isr    = SEM_INITIALIZER(0),
  .msgs       = NULL
};

#endif

#ifdef CONFIG_STM32_USART2_1WIREDRIVER

static const struct stm32_1wire_config_s stm32_1wire2_config =
{
  .usartbase  = STM32_USART2_BASE,
  .apbclock   = STM32_PCLK1_FREQUENCY,
  .data_pin   = PIN_OPENDRAIN(GPIO_USART2_TX),
  .irq        = STM32_IRQ_USART2,
};

static struct stm32_1wire_priv_s stm32_1wire2_priv =
{
  .config   = &stm32_1wire2_config,
  .refs     = 0,
  .lock     = NXMUTEX_INITIALIZER,
  .sem_isr  = SEM_INITIALIZER(0),
  .msgs     = NULL
};

#endif

#ifdef CONFIG_STM32_USART3_1WIREDRIVER

static const struct stm32_1wire_config_s stm32_1wire3_config =
{
  .usartbase  = STM32_USART3_BASE,
  .apbclock   = STM32_PCLK1_FREQUENCY,
  .data_pin   = PIN_OPENDRAIN(GPIO_USART3_TX),
  .irq        = STM32_IRQ_USART3,
};

static struct stm32_1wire_priv_s stm32_1wire3_priv =
{
  .config   = &stm32_1wire3_config,
  .refs     = 0,
  .lock     = NXMUTEX_INITIALIZER,
  .sem_isr  = SEM_INITIALIZER(0),
  .msgs     = NULL
};

#endif

#ifdef CONFIG_STM32_UART4_1WIREDRIVER

static const struct stm32_1wire_config_s stm32_1wire4_config =
{
  .usartbase  = STM32_UART4_BASE,
  .apbclock   = STM32_PCLK1_FREQUENCY,
  .data_pin   = PIN_OPENDRAIN(GPIO_UART4_TX),
  .irq        = STM32_IRQ_UART4,
};

static struct stm32_1wire_priv_s stm32_1wire4_priv =
{
  .config   = &stm32_1wire4_config,
  .refs     = 0,
  .lock     = NXMUTEX_INITIALIZER,
  .sem_isr  = SEM_INITIALIZER(0),
  .msgs     = NULL
};

#endif

#ifdef CONFIG_STM32_UART5_1WIREDRIVER

static const struct stm32_1wire_config_s stm32_1wire5_config =
{
  .usartbase  = STM32_UART5_BASE,
  .apbclock   = STM32_PCLK1_FREQUENCY,
  .data_pin   = PIN_OPENDRAIN(GPIO_UART5_TX),
  .irq        = STM32_IRQ_UART5,
};

static struct stm32_1wire_priv_s stm32_1wire5_priv =
{
  .config   = &stm32_1wire5_config,
  .refs     = 0,
  .lock     = NXMUTEX_INITIALIZER,
  .sem_isr  = SEM_INITIALIZER(0),
  .msgs     = NULL
};

#endif

#ifdef CONFIG_STM32_USART6_1WIREDRIVER

static const struct stm32_1wire_config_s stm32_1wire6_config =
{
  .usartbase  = STM32_USART6_BASE,
  .apbclock   = STM32_PCLK2_FREQUENCY,
  .data_pin   = PIN_OPENDRAIN(GPIO_USART6_TX),
  .irq        = STM32_IRQ_USART6,
};

static struct stm32_1wire_priv_s stm32_1wire6_priv =
{
  .config   = &stm32_1wire6_config,
  .refs     = 0,
  .lock     = NXMUTEX_INITIALIZER,
  .sem_isr  = SEM_INITIALIZER(0),
  .msgs     = NULL
};

#endif

#ifdef CONFIG_STM32_UART7_1WIREDRIVER

static const struct stm32_1wire_config_s stm32_1wire7_config =
{
  .usartbase  = STM32_UART7_BASE,
  .apbclock   = STM32_PCLK1_FREQUENCY,
  .data_pin   = PIN_OPENDRAIN(GPIO_UART7_TX),
  .irq        = STM32_IRQ_UART7,
};

static struct stm32_1wire_priv_s stm32_1wire7_priv =
{
  .config   = &stm32_1wire7_config,
  .refs     = 0,
  .lock     = NXMUTEX_INITIALIZER,
  .sem_isr  = SEM_INITIALIZER(0),
  .msgs     = NULL
};

#endif

#ifdef CONFIG_STM32_UART8_1WIREDRIVER

static const struct stm32_1wire_config_s stm32_1wire8_config =
{
  .usartbase  = STM32_UART8_BASE,
  .apbclock   = STM32_PCLK1_FREQUENCY,
  .data_pin   = PIN_OPENDRAIN(GPIO_UART8_TX),
  .irq        = STM32_IRQ_UART8,
};

static struct stm32_1wire_priv_s stm32_1wire8_priv =
{
  .config   = &stm32_1wire8_config,
  .refs     = 0,
  .lock     = NXMUTEX_INITIALIZER,
  .sem_isr  = SEM_INITIALIZER(0),
  .msgs     = NULL
};

#endif

/* Device Structures, Instantiation */

static const struct onewire_ops_s stm32_1wire_ops =
{
  .reset    = stm32_1wire_reset,
  .write    = stm32_1wire_write,
  .read     = stm32_1wire_read,
  .exchange = stm32_1wire_exchange,
  .writebit = stm32_1wire_writebit,
  .readbit  = stm32_1wire_readbit
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_1wire_in
 ****************************************************************************/

static inline uint32_t stm32_1wire_in(struct stm32_1wire_priv_s *priv,
                                      int offset)
{
  return getreg32(priv->config->usartbase + offset);
}

/****************************************************************************
 * Name: stm32_1wire_out
 ****************************************************************************/

static inline void stm32_1wire_out(struct stm32_1wire_priv_s *priv,
                                   int offset, uint32_t value)
{
  putreg32(value, priv->config->usartbase + offset);
}

/****************************************************************************
 * Name: stm32_1wire_recv
 *
 * Description:
 *   This method will recv one byte on the USART
 *
 ****************************************************************************/

static int stm32_1wire_recv(struct stm32_1wire_priv_s *priv)
{
  return stm32_1wire_in(priv, STM32_USART_RDR_OFFSET) & 0xff;
}

/****************************************************************************
 * Name: stm32_1wire_send
 *
 * Description:
 *   This method will send one byte on the USART
 *
 ****************************************************************************/

static void stm32_1wire_send(struct stm32_1wire_priv_s *priv, int ch)
{
  stm32_1wire_out(priv, STM32_USART_TDR_OFFSET, (uint32_t)(ch & 0xff));
}

/****************************************************************************
 * Name: stm32_1wire_set_baud
 *
 * Description:
 *   Set the serial line baud.
 *
 ****************************************************************************/

static void stm32_1wire_set_baud(struct stm32_1wire_priv_s *priv)
{
#if defined(CONFIG_STM32_STM32F30XX) || defined(CONFIG_STM32_STM32F37XX)
  /* This first implementation is for U[S]ARTs that support oversampling
   * by 8 in additional to the standard oversampling by 16.
   */

  uint32_t usartdiv8;
  uint32_t cr1;
  uint32_t brr;

  /* In case of oversampling by 8, the equation is:
   *
   *   baud      = 2 * fCK / usartdiv8
   *   usartdiv8 = 2 * fCK / baud
   */

  usartdiv8 = ((priv->config->apbclock << 1) + (priv->baud >> 1)) /
                priv->baud;

  /* Baud rate for standard USART (SPI mode included):
   *
   * In case of oversampling by 16, the equation is:
   *   baud       = fCK / usartdiv16
   *   usartdiv16 = fCK / baud
   *              = 2 * usartdiv8
   */

  /* Use oversamply by 8 only if the divisor is small.  But what is small? */

  cr1 = stm32_1wire_in(priv, STM32_USART_CR1_OFFSET);
  if (usartdiv8 > 100)
    {
      /* Use usartdiv16 */

      brr  = (usartdiv8 + 1) >> 1;

      /* Clear oversampling by 8 to enable oversampling by 16 */

      cr1 &= ~USART_CR1_OVER8;
    }
  else
    {
      DEBUGASSERT(usartdiv8 >= 8);

      /* Perform mysterious operations on bits 0-3 */

      brr  = ((usartdiv8 & 0xfff0) | ((usartdiv8 & 0x000f) >> 1));

      /* Set oversampling by 8 */

      cr1 |= USART_CR1_OVER8;
    }

  stm32_1wire_out(priv, STM32_USART_CR1_OFFSET, cr1);
  stm32_1wire_out(priv, STM32_USART_BRR_OFFSET, brr);

#else

  /* This second implementation is for U[S]ARTs that support fractional
   * dividers.
   */

  uint32_t usartdiv32;
  uint32_t mantissa;
  uint32_t fraction;
  uint32_t brr;

  /* Configure the USART Baud Rate.  The baud rate for the receiver and
   * transmitter (Rx and Tx) are both set to the same value as programmed
   * in the Mantissa and Fraction values of USARTDIV.
   *
   *   baud     = fCK / (16 * usartdiv)
   *   usartdiv = fCK / (16 * baud)
   *
   * Where fCK is the input clock to the peripheral (PCLK1 for USART2, 3, 4,
   * 5 or PCLK2 for USART1)
   *
   * First calculate (NOTE: all stand baud values are even so dividing by two
   * does not lose precision):
   *
   *   usartdiv32 = 32 * usartdiv = fCK / (baud/2)
   */

  usartdiv32 = priv->config->apbclock / (priv->baud >> 1);

  /* The mantissa part is then */

  mantissa   = usartdiv32 >> 5;
  brr        = mantissa << USART_BRR_MANT_SHIFT;

  /* The fractional remainder (with rounding) */

  fraction   = (usartdiv32 - (mantissa << 5) + 1) >> 1;
  brr       |= fraction << USART_BRR_FRAC_SHIFT;
  stm32_1wire_out(priv, STM32_USART_BRR_OFFSET, brr);
#endif
}

/****************************************************************************
 * Name: stm32_1wire_set_apb_clock
 *
 * Description:
 *   Enable or disable APB clock for the USART peripheral
 *
 * Input Parameters:
 *   priv - A reference to the 1-Wire driver state structure
 *   on  - Enable clock if 'on' is 'true' and disable if 'false'
 *
 ****************************************************************************/

static void stm32_1wire_set_apb_clock(struct stm32_1wire_priv_s *priv,
                                      bool on)
{
  const struct stm32_1wire_config_s *config = priv->config;
  uint32_t rcc_en;
  uint32_t regaddr;

  /* Determine which USART to configure */

  switch (config->usartbase)
    {
    default:
      return;

#ifdef CONFIG_STM32_USART1_1WIREDRIVER
    case STM32_USART1_BASE:
      rcc_en = RCC_APB2ENR_USART1EN;
      regaddr = STM32_RCC_APB2ENR;
      break;
#endif
#ifdef CONFIG_STM32_USART2_1WIREDRIVER
    case STM32_USART2_BASE:
      rcc_en = RCC_APB1ENR_USART2EN;
      regaddr = STM32_RCC_APB1ENR;
      break;
#endif
#ifdef CONFIG_STM32_USART3_1WIREDRIVER
    case STM32_USART3_BASE:
      rcc_en = RCC_APB1ENR_USART3EN;
      regaddr = STM32_RCC_APB1ENR;
      break;
#endif
#ifdef CONFIG_STM32_UART4_1WIREDRIVER
    case STM32_UART4_BASE:
      rcc_en = RCC_APB1ENR_UART4EN;
      regaddr = STM32_RCC_APB1ENR;
      break;
#endif
#ifdef CONFIG_STM32_UART5_1WIREDRIVER
    case STM32_UART5_BASE:
      rcc_en = RCC_APB1ENR_UART5EN;
      regaddr = STM32_RCC_APB1ENR;
      break;
#endif
#ifdef CONFIG_STM32_USART6_1WIREDRIVER
    case STM32_USART6_BASE:
      rcc_en = RCC_APB2ENR_USART6EN;
      regaddr = STM32_RCC_APB2ENR;
      break;
#endif
#ifdef CONFIG_STM32_UART7_1WIREDRIVER
    case STM32_UART7_BASE:
      rcc_en = RCC_APB1ENR_UART7EN;
      regaddr = STM32_RCC_APB1ENR;
      break;
#endif
#ifdef CONFIG_STM32_UART8_1WIREDRIVER
    case STM32_UART8_BASE:
      rcc_en = RCC_APB1ENR_UART8EN;
      regaddr = STM32_RCC_APB1ENR;
      break;
#endif
    }

  /* Enable/disable APB 1/2 clock for USART */

  if (on)
    {
      modifyreg32(regaddr, 0, rcc_en);
    }
  else
    {
      modifyreg32(regaddr, rcc_en, 0);
    }
}

/****************************************************************************
 * Name: stm32_1wire_init
 *
 * Description:
 *   Setup the 1-Wire hardware, ready for operation with defaults
 *
 ****************************************************************************/

static int stm32_1wire_init(struct stm32_1wire_priv_s *priv)
{
  const struct stm32_1wire_config_s *config = priv->config;
  uint32_t regval;
  int ret;

  /* Enable USART APB1/2 clock */

  stm32_1wire_set_apb_clock(priv, true);

  /* Configure CR2
   * Clear STOP, CLKEN, CPOL, CPHA, LBCL, and interrupt enable bits
   * Set LBDIE
   */

  regval  = stm32_1wire_in(priv, STM32_USART_CR2_OFFSET);
  regval &= ~(USART_CR2_STOP_MASK | USART_CR2_CLKEN | USART_CR2_CPOL |
              USART_CR2_CPHA | USART_CR2_LBCL | USART_CR2_LBDIE);
  regval |= USART_CR2_LBDIE;
  stm32_1wire_out(priv, STM32_USART_CR2_OFFSET, regval);

  /* Configure CR1
   * Clear TE, REm, all interrupt enable bits, PCE, PS and M
   * Set RXNEIE
   */

  regval  = stm32_1wire_in(priv, STM32_USART_CR1_OFFSET);
  regval &= ~(USART_CR1_TE | USART_CR1_RE | USART_CR1_ALLINTS |
              USART_CR1_PCE | USART_CR1_PS | USART_CR1_M);
  regval |= USART_CR1_RXNEIE;
  stm32_1wire_out(priv, STM32_USART_CR1_OFFSET, regval);

  /* Configure CR3
   * Clear CTSE, RTSE, and all interrupt enable bits
   * Set ONEBIT, HDSEL and EIE
   */

  regval  = stm32_1wire_in(priv, STM32_USART_CR3_OFFSET);
  regval &= ~(USART_CR3_CTSIE | USART_CR3_CTSE | USART_CR3_RTSE |
              USART_CR3_EIE);
  regval |= (USART_CR3_ONEBIT | USART_CR3_HDSEL | USART_CR3_EIE);
  stm32_1wire_out(priv, STM32_USART_CR3_OFFSET, regval);

  /* Set baud rate */

  priv->baud = RESET_BAUD;
  stm32_1wire_set_baud(priv);

  /* Enable Rx, Tx, and the USART */

  regval  = stm32_1wire_in(priv, STM32_USART_CR1_OFFSET);
  regval |= (USART_CR1_UE | USART_CR1_TE | USART_CR1_RE);
  stm32_1wire_out(priv, STM32_USART_CR1_OFFSET, regval);

  /* Configure pins for USART use */

  stm32_configgpio(config->data_pin);

  ret = irq_attach(config->irq, stm32_1wire_isr, priv);
  if (ret == OK)
    {
      up_enable_irq(config->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: stm32_1wire_deinit
 *
 * Description:
 *   Shutdown the 1-Wire hardware
 *
 ****************************************************************************/

static int stm32_1wire_deinit(struct stm32_1wire_priv_s *priv)
{
  const struct stm32_1wire_config_s *config = priv->config;
  uint32_t regval;

  up_disable_irq(config->irq);
  irq_detach(config->irq);

  /* Unconfigure GPIO pins */

  stm32_unconfiggpio(config->data_pin);

  /* Disable RXNEIE, Rx, Tx, and the USART */

  regval  = stm32_1wire_in(priv, STM32_USART_CR1_OFFSET);
  regval &= ~(USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE);
  stm32_1wire_out(priv, STM32_USART_CR1_OFFSET, regval);

  /* Clear LBDIE */

  regval  = stm32_1wire_in(priv, STM32_USART_CR2_OFFSET);
  regval &= ~USART_CR2_LBDIE;
  stm32_1wire_out(priv, STM32_USART_CR2_OFFSET, regval);

  /* Clear ONEBIT, HDSEL and EIE */

  regval  = stm32_1wire_in(priv, STM32_USART_CR3_OFFSET);
  regval &= ~(USART_CR3_ONEBIT | USART_CR3_HDSEL | USART_CR3_EIE);
  stm32_1wire_out(priv, STM32_USART_CR3_OFFSET, regval);

  /* Disable USART APB1/2 clock */

  stm32_1wire_set_apb_clock(priv, false);

  return OK;
}

/****************************************************************************
 * Name: stm32_1wire_exec
 *
 * Description:
 *  Execute 1-Wire task
 ****************************************************************************/

static int stm32_1wire_process(struct stm32_1wire_priv_s *priv,
                               const struct stm32_1wire_msg_s *msgs,
                               int count)
{
  irqstate_t irqs;
  int indx;
  int ret;

  /* Lock out other clients */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  priv->result = ERROR;

  for (indx = 0; indx < count; indx++)
    {
      switch (msgs[indx].task)
        {
        case ONEWIRETASK_NONE:
          priv->result = OK;
          break;

        case ONEWIRETASK_RESET:

          /* Set baud rate */

          priv->baud = RESET_BAUD;
          stm32_1wire_set_baud(priv);

          /* Atomic */

          irqs = enter_critical_section();
          priv->msgs = &msgs[indx];
          stm32_1wire_send(priv, RESET_TX);
          leave_critical_section(irqs);

          /* Wait.  Break on timeout if TX line closed to GND */

          nxsem_tickwait(&priv->sem_isr, SEC2TICK(BUS_TIMEOUT));
          break;

        case ONEWIRETASK_WRITE:
        case ONEWIRETASK_WRITEBIT:

          /* Set baud rate */

          priv->baud = TIMESLOT_BAUD;
          stm32_1wire_set_baud(priv);

          /* Atomic */

          irqs = enter_critical_section();
          priv->msgs = &msgs[indx];
          priv->byte = priv->msgs->buffer;
          priv->bit = 0;
          stm32_1wire_send(priv, (*priv->byte & (1 << priv->bit)) ?
                           WRITE_TX1 : WRITE_TX0);
          leave_critical_section(irqs);

          /* Wait.  Break on timeout if TX line closed to GND */

          nxsem_tickwait(&priv->sem_isr, SEC2TICK(BUS_TIMEOUT));
          break;

        case ONEWIRETASK_READ:
        case ONEWIRETASK_READBIT:

          /* Set baud rate */

          priv->baud = TIMESLOT_BAUD;
          stm32_1wire_set_baud(priv);

          /* Atomic */

          irqs = enter_critical_section();
          priv->msgs = &msgs[indx];
          priv->byte = priv->msgs->buffer;
          priv->bit = 0;
          stm32_1wire_send(priv, READ_TX);
          leave_critical_section(irqs);

          /* Wait.  Break on timeout if TX line closed to GND */

          nxsem_tickwait(&priv->sem_isr, SEC2TICK(BUS_TIMEOUT));
          break;
        }

      if (priv->result != OK) /* break if error */
        {
          break;
        }
    }

  /* Atomic */

  irqs = enter_critical_section();
  priv->msgs = NULL;
  ret = priv->result;
  leave_critical_section(irqs);

  /* Release the port for re-use by other clients */

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: stm32_1wire_isr
 *
 * Description:
 *  Common Interrupt Service Routine
 ****************************************************************************/

static int stm32_1wire_isr(int irq, void *context, void *arg)
{
  struct stm32_1wire_priv_s *priv = (struct stm32_1wire_priv_s *)arg;
  uint32_t sr;
  uint32_t dr;

  DEBUGASSERT(priv != NULL);

  /* Get the masked USART status word. */

  sr = stm32_1wire_in(priv, STM32_USART_SR_OFFSET);

  /* Receive loop */

  if ((sr & USART_SR_RXNE) != 0)
    {
      dr = stm32_1wire_recv(priv);

      if (priv->msgs != NULL)
        {
          switch (priv->msgs->task)
            {
            case ONEWIRETASK_NONE:
              break;

            case ONEWIRETASK_RESET:
              priv->msgs = NULL;
              priv->result = (dr != RESET_TX) ? OK : -ENODEV; /* if read RESET_TX then no slave */
              nxsem_post(&priv->sem_isr);
              break;

            case ONEWIRETASK_WRITE:
              if (++priv->bit >= 8)
                {
                  priv->bit = 0;
                  if (++priv->byte >= (priv->msgs->buffer + priv->msgs->buflen)) /* Done? */
                    {
                      priv->msgs = NULL;
                      priv->result = OK;
                      nxsem_post(&priv->sem_isr);
                      break;
                    }
                }

              /* Send next bit */

              stm32_1wire_send(priv, (*priv->byte & (1 << priv->bit)) ?
                               WRITE_TX1 : WRITE_TX0);
              break;

            case ONEWIRETASK_READ:
              if (dr == READ_RX1)
                {
                  *priv->byte |= (1 << priv->bit);
                }
              else
                {
                  *priv->byte &= ~(1 << priv->bit);
                }

              if (++priv->bit >= 8)
                {
                  priv->bit = 0;
                  if (++priv->byte >= (priv->msgs->buffer + priv->msgs->buflen)) /* Done? */
                    {
                      priv->msgs = NULL;
                      priv->result = OK;
                      nxsem_post(&priv->sem_isr);
                      break;
                    }
                }

              /* Recv next bit */

              stm32_1wire_send(priv, READ_TX);
              break;

            case ONEWIRETASK_READBIT:
              *priv->byte = (dr == READ_RX1) ? 1 : 0;

              /* Fall through */

            case ONEWIRETASK_WRITEBIT:
              priv->msgs = NULL;
              priv->result = OK;
              nxsem_post(&priv->sem_isr);
              break;
            }
        }
    }

  /* Bounce check.  */

  if ((sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE)) != 0)
    {
#if defined(CONFIG_STM32_STM32F30XX) || defined(CONFIG_STM32_STM32F37XX)
      /* These errors are cleared by writing the corresponding bit to the
       * interrupt clear register (ICR).
       */

      stm32_1wire_out(priv, STM32_USART_ICR_OFFSET,
                      (USART_ICR_NCF | USART_ICR_ORECF | USART_ICR_FECF));
#else
      /* If an error occurs, read from DR to clear the error (data has
       * been lost).  If ORE is set along with RXNE then it tells you
       * that the byte *after* the one in the data register has been
       * lost, but the data register value is correct.  That case will
       * be handled above if interrupts are enabled. Otherwise, that
       * good byte will be lost.
       */

      stm32_1wire_recv(priv);
#endif

      if (priv->msgs != NULL)
        {
          priv->msgs = NULL;
          priv->result = ERROR;
          nxsem_post(&priv->sem_isr);
        }
    }

  /* Bounce check. LIN break detection  */

  if ((sr & USART_SR_LBD) != 0)
    {
      sr &= ~USART_SR_LBD;
      stm32_1wire_out(priv, STM32_USART_SR_OFFSET, sr);

      if (priv->msgs != NULL)
        {
          priv->msgs = NULL;
          priv->result = ERROR;
          nxsem_post(&priv->sem_isr);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_1wire_reset
 *
 * Description:
 *   1-Wire reset pulse and presence detect.
 *
 ****************************************************************************/

static int stm32_1wire_reset(struct onewire_dev_s *dev)
{
  struct stm32_1wire_priv_s *priv = ((struct stm32_1wire_inst_s *)dev)->priv;
  const struct stm32_1wire_msg_s msgs[1] =
  {
    [0].task = ONEWIRETASK_RESET
  };

  return stm32_1wire_process(priv, msgs, 1);
}

/****************************************************************************
 * Name: stm32_1wire_write
 *
 * Description:
 *   Write 1-Wire data
 *
 ****************************************************************************/

static int stm32_1wire_write(struct onewire_dev_s *dev,
                             const uint8_t *buffer, int buflen)
{
  struct stm32_1wire_priv_s *priv = ((struct stm32_1wire_inst_s *)dev)->priv;
  const struct stm32_1wire_msg_s msgs[1] =
  {
    [0].task = ONEWIRETASK_WRITE,
    [0].buffer = (uint8_t *)buffer,
    [0].buflen = buflen
  };

  return stm32_1wire_process(priv, msgs, 1);
}

/****************************************************************************
 * Name: stm32_1wire_read
 *
 * Description:
 *   Read 1-Wire data
 *
 ****************************************************************************/

static int stm32_1wire_read(struct onewire_dev_s *dev, uint8_t *buffer,
                            int buflen)
{
  struct stm32_1wire_priv_s *priv = ((struct stm32_1wire_inst_s *)dev)->priv;
  const struct stm32_1wire_msg_s msgs[1] =
  {
    [0].task = ONEWIRETASK_READ,
    [0].buffer = buffer,
    [0].buflen = buflen
  };

  return stm32_1wire_process(priv, msgs, 1);
}

/****************************************************************************
 * Name: stm32_1wire_exchange
 *
 * Description:
 *   1-Wire reset pulse and presence detect,
 *   Write 1-Wire data,
 *   Read 1-Wire data
 *
 ****************************************************************************/

static int stm32_1wire_exchange(struct onewire_dev_s *dev, bool reset,
                                const uint8_t *txbuffer, int txbuflen,
                                uint8_t *rxbuffer, int rxbuflen)
{
  int result = ERROR;
  struct stm32_1wire_priv_s *priv = ((struct stm32_1wire_inst_s *)dev)->priv;

  if (reset)
    {
      const struct stm32_1wire_msg_s msgs[3] =
      {
        [0].task = ONEWIRETASK_RESET,

        [1].task = ONEWIRETASK_WRITE,
        [1].buffer = (uint8_t *)txbuffer,
        [1].buflen = txbuflen,

        [2].task = ONEWIRETASK_READ,
        [2].buffer = rxbuffer,
        [2].buflen = rxbuflen
      };

      result = stm32_1wire_process(priv, msgs, 3);
    }
  else
    {
      const struct stm32_1wire_msg_s msgs[2] =
      {
        [0].task = ONEWIRETASK_WRITE,
        [0].buffer = (uint8_t *)txbuffer,
        [0].buflen = txbuflen,

        [1].task = ONEWIRETASK_READ,
        [1].buffer = rxbuffer,
        [1].buflen = rxbuflen
      };

      result = stm32_1wire_process(priv, msgs, 2);
    }

  return result;
}

/****************************************************************************
 * Name: stm32_1wire_writebit
 *
 * Description:
 *   Write one bit of 1-Wire data
 *
 ****************************************************************************/

static int stm32_1wire_writebit(struct onewire_dev_s *dev,
                                const uint8_t *bit)
{
  struct stm32_1wire_priv_s *priv = ((struct stm32_1wire_inst_s *)dev)->priv;
  const struct stm32_1wire_msg_s msgs[1] =
  {
    [0].task = ONEWIRETASK_WRITEBIT,
    [0].buffer = (uint8_t *)bit,
    [0].buflen = 1
  };

  DEBUGASSERT(*bit == 0 || *bit == 1);

  return stm32_1wire_process(priv, msgs, 1);
}

/****************************************************************************
 * Name: stm32_1wire_readbit
 *
 * Description:
 *   Sample one bit of 1-Wire data
 *
 ****************************************************************************/

static int stm32_1wire_readbit(struct onewire_dev_s *dev, uint8_t *bit)
{
  struct stm32_1wire_priv_s *priv = ((struct stm32_1wire_inst_s *)dev)->priv;
  const struct stm32_1wire_msg_s msgs[1] =
  {
    [0].task = ONEWIRETASK_READBIT,
    [0].buffer = bit,
    [0].buflen = 1
  };

  return stm32_1wire_process(priv, msgs, 1);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_1wireinitialize
 *
 * Description:
 *   Initialize the selected 1-Wire port. And return a unique instance of
 *   struct onewire_dev_s.  This function may be called to obtain multiple
 *   instances of the interface, each of which may be set up with a
 *   different frequency and slave address.
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple 1-Wire interfaces)
 *
 * Returned Value:
 *   Valid 1-Wire device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct onewire_dev_s *stm32_1wireinitialize(int port)
{
  struct stm32_1wire_priv_s *priv = NULL;  /* Private data of device with multiple instances */
  struct stm32_1wire_inst_s *inst = NULL;  /* Device, single instance */

  /* Get 1-Wire private structure */

  switch (port)
    {
#ifdef CONFIG_STM32_USART1_1WIREDRIVER
    case 1:
      priv = &stm32_1wire1_priv;
      break;
#endif
#ifdef CONFIG_STM32_USART2_1WIREDRIVER
    case 2:
      priv = &stm32_1wire2_priv;
      break;
#endif
#ifdef CONFIG_STM32_USART3_1WIREDRIVER
    case 3:
      priv = &stm32_1wire3_priv;
      break;
#endif
#ifdef CONFIG_STM32_UART4_1WIREDRIVER
    case 4:
      priv = &stm32_1wire4_priv;
      break;
#endif
#ifdef CONFIG_STM32_UART5_1WIREDRIVER
    case 5:
      priv = &stm32_1wire5_priv;
      break;
#endif
#ifdef CONFIG_STM32_USART6_1WIREDRIVER
    case 6:
      priv = &stm32_1wire6_priv;
      break;
#endif
#ifdef CONFIG_STM32_UART7_1WIREDRIVER
    case 7:
      priv = &stm32_1wire7_priv;
      break;
#endif
#ifdef CONFIG_STM32_UART8_1WIREDRIVER
    case 8:
      priv = &stm32_1wire8_priv;
      break;
#endif
    default:
      return NULL;
    }

  /* Allocate instance */

  inst = kmm_malloc(sizeof(*inst));
  if (inst == NULL)
    {
      return NULL;
    }

  /* Initialize instance */

  inst->ops  = &stm32_1wire_ops;
  inst->priv = priv;

  /* Initialize private data for the first time, increment reference count,
   * power-up hardware and configure GPIOs.
   */

  nxmutex_lock(&priv->lock);
  if (priv->refs++ == 0)
    {
      stm32_1wire_init(priv);
    }

  nxmutex_unlock(&priv->lock);
  return (struct onewire_dev_s *)inst;
}

/****************************************************************************
 * Name: stm32_1wireuninitialize
 *
 * Description:
 *   De-initialize the selected 1-Wire port, and power down the device.
 *
 * Input Parameters:
 *   Device structure as returned by the stm32_1wireinitialize()
 *
 * Returned Value:
 *   OK on success, ERROR when internal reference count mismatch or dev
 *   points to invalid hardware device.
 *
 ****************************************************************************/

int stm32_1wireuninitialize(struct onewire_dev_s *dev)
{
  struct stm32_1wire_priv_s *priv = ((struct stm32_1wire_inst_s *)dev)->priv;

  DEBUGASSERT(priv);

  /* Decrement reference count and check for underflow */

  if (priv->refs == 0)
    {
      return ERROR;
    }

  nxmutex_lock(&priv->lock);
  if (--priv->refs)
    {
      nxmutex_unlock(&priv->lock);
      return OK;
    }

  /* Disable power and other HW resource (GPIO's) */

  stm32_1wire_deinit(priv);
  nxmutex_unlock(&priv->lock);

  /* Free instance */

  kmm_free(dev);
  return OK;
}

#endif /* HAVE_1WIREDRIVER */
