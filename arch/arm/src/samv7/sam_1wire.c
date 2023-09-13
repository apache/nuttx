/****************************************************************************
 * arch/arm/src/samv7/sam_1wire.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/serial.h>
#include <nuttx/1wire/1wire.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "sam_config.h"

#include "hardware/sam_pinmap.h"
#include "hardware/sam_uart.h"
#include "sam_gpio.h"
#include "sam_serial.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SAMV7_1WIREDRIVER

#define BUS_TIMEOUT     5      /* tv_sec */

#define RESET_BAUD      9600
#define RESET_TX        0xF0
#define TIMESLOT_BAUD   115200
#define READ_TX         0xFF
#define READ_RX1        0xFF
#define WRITE_TX0       0x00
#define WRITE_TX1       0xFF

#define FAST_USART_CLOCK   BOARD_MCK_FREQUENCY
#define SLOW_USART_CLOCK   (BOARD_MCK_FREQUENCY >> 3)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* 1-Wire bus task */

enum sam_1wire_msg_e
{
  ONEWIRETASK_NONE = 0,
  ONEWIRETASK_RESET,
  ONEWIRETASK_WRITE,
  ONEWIRETASK_READ,
  ONEWIRETASK_WRITEBIT,
  ONEWIRETASK_READBIT
};

struct sam_1wire_msg_s
{
  enum sam_1wire_msg_e task;      /* Task */
  uint8_t *buffer;                /* Task buffer */
  int      buflen;                /* Buffer length */
};

/* 1-Wire device Private Data */

struct sam_1wire_s
{
  const uint32_t base;                 /* Base address of registers */
  const uint32_t rx;                   /* RX pin */
  const uint32_t tx;                   /* TX pin */
  volatile int refs;                   /* Reference count */
  volatile int result;                 /* Exchange result */
  mutex_t  lock;                       /* Mutual exclusion mutex */
  sem_t    sem_isr;                    /* Interrupt wait semaphore */
  int      baud;                       /* Baud rate */
  const struct sam_1wire_msg_s *msgs;  /* Messages data */
  const uint8_t  irq;                  /* IRQ associated with this USART */
  uint8_t *byte;                       /* Current byte */
  uint8_t  bit;                        /* Current bit */
};

/* 1-Wire device, Instance */

struct sam_1wire_inst_s
{
  const struct onewire_ops_s *ops;  /* Standard 1-Wire operations */
  struct sam_1wire_s  *priv;        /* Common driver private data structure */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int sam_interrupt(int irq, void *context, void *arg);
static int sam_writebit(struct onewire_dev_s *dev, const uint8_t *bit);
static int sam_reset(struct onewire_dev_s *dev);
static int sam_write(struct onewire_dev_s *dev, const uint8_t *buffer,
                     int buflen);
static int sam_read(struct onewire_dev_s *dev, uint8_t *buffer,
                     int buflen);
static int sam_exchange(struct onewire_dev_s *dev, bool reset,
                        const uint8_t *txbuffer, int txbuflen,
                        uint8_t *rxbuffer, int rxbuflen);
static int sam_readbit(struct onewire_dev_s *dev, uint8_t *bit);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct onewire_ops_s g_1wire_ops =
{
  .reset    = sam_reset,
  .write    = sam_write,
  .read     = sam_read,
  .exchange = sam_exchange,
  .writebit = sam_writebit,
  .readbit  = sam_readbit
};

/* 1-Wire device structures */

#ifdef CONFIG_SAMV7_UART0_1WIREDRIVER

static struct sam_1wire_s sam_1wire0_priv =
{
  .base  = SAM_UART0_BASE,
  .rx    = GPIO_UART0_RXD,
  .tx    = GPIO_UART0_TXD,
  .irq   = SAM_IRQ_UART0,
  .refs  = 0,
  .lock  = NXMUTEX_INITIALIZER,
  .sem_isr = SEM_INITIALIZER(0),
  .msgs  = NULL,
};

#endif

#ifdef CONFIG_SAMV7_UART1_1WIREDRIVER

static struct sam_1wire_s sam_1wire1_priv =
{
  .base  = SAM_UART1_BASE,
  .rx    = GPIO_UART1_RXD,
  .tx    = GPIO_UART1_TXD,
  .irq   = SAM_IRQ_UART1,
  .refs  = 0,
  .lock  = NXMUTEX_INITIALIZER,
  .sem_isr = SEM_INITIALIZER(0),
  .msgs  = NULL,
};

#endif

#ifdef CONFIG_SAMV7_UART2_1WIREDRIVER

static struct sam_1wire_s sam_1wire2_priv =
{
  .base  = SAM_UART2_BASE,
  .rx    = GPIO_UART2_RXD,
  .tx    = GPIO_UART2_TXD,
  .irq   = SAM_IRQ_UART2,
  .refs  = 0,
  .lock  = NXMUTEX_INITIALIZER,
  .sem_isr = SEM_INITIALIZER(0),
  .msgs  = NULL,
};

#endif

#ifdef CONFIG_SAMV7_UART3_1WIREDRIVER

static struct sam_1wire_s sam_1wire3_priv =
{
  .base  = SAM_UART3_BASE,
  .rx    = GPIO_UART3_RXD,
  .tx    = GPIO_UART3_TXD,
  .irq   = SAM_IRQ_UART3,
  .refs  = 0,
  .lock  = NXMUTEX_INITIALIZER,
  .sem_isr = SEM_INITIALIZER(0),
  .msgs  = NULL,
};

#endif

#ifdef CONFIG_SAMV7_UART4_1WIREDRIVER

static struct sam_1wire_s sam_1wire4_priv =
{
  .base  = SAM_UART4_BASE,
  .rx    = GPIO_UART4_RXD,
  .tx    = GPIO_UART4_TXD,
  .irq   = SAM_IRQ_UART4,
  .refs  = 0,
  .lock  = NXMUTEX_INITIALIZER,
  .sem_isr = SEM_INITIALIZER(0),
  .msgs  = NULL,
};

#endif

#ifdef CONFIG_SAMV7_USART0_1WIREDRIVER

static struct sam_1wire_s sam_1wire5_priv =
{
  .base  = SAM_USART0_BASE,
  .rx    = GPIO_USART0_RXD,
  .tx    = GPIO_USART0_TXD,
  .irq   = SAM_IRQ_USART0,
  .refs  = 0,
  .lock  = NXMUTEX_INITIALIZER,
  .sem_isr = SEM_INITIALIZER(0),
  .msgs  = NULL,
};

#endif

#ifdef CONFIG_SAMV7_USART1_1WIREDRIVER

static struct sam_1wire_s sam_1wire6_priv =
{
  .base  = SAM_USART1_BASE,
  .rx    = GPIO_USART1_RXD,
  .tx    = GPIO_USART1_TXD,
  .irq   = SAM_IRQ_USART1,
  .refs  = 0,
  .lock  = NXMUTEX_INITIALIZER,
  .sem_isr = SEM_INITIALIZER(0),
  .msgs  = NULL,
};

#endif

#ifdef CONFIG_SAMV7_USART2_1WIREDRIVER

static struct sam_1wire_s sam_1wire7_priv =
{
  .base  = SAM_USART2_BASE,
  .rx    = GPIO_USART2_RXD,
  .tx    = GPIO_USART2_TXD,
  .irq   = SAM_IRQ_USART2,
  .refs  = 0,
  .lock  = NXMUTEX_INITIALIZER,
  .sem_isr = SEM_INITIALIZER(0),
  .msgs  = NULL,
};

#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_serialin
 ****************************************************************************/

static inline uint32_t sam_serialin(struct sam_1wire_s *priv, int offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: sam_serialout
 ****************************************************************************/

static inline void sam_serialout(struct sam_1wire_s *priv, int offset,
                                 uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: sam_receive
 *
 * Description:
 *   Receive one byte from UART/USART
 *
 ****************************************************************************/

static int sam_receive(struct sam_1wire_s *priv)
{
  return (int)(sam_serialin(priv, SAM_UART_RHR_OFFSET) & 0xff);
}

/****************************************************************************
 * Name: sam_send
 *
 * Description:
 *   This method will send one byte on the UART/USART
 *
 ****************************************************************************/

static void sam_send(struct sam_1wire_s *priv, int ch)
{
  sam_serialout(priv, SAM_UART_THR_OFFSET, (uint32_t)ch);
}

/****************************************************************************
 * Name: sam_disableallints
 ****************************************************************************/

static void sam_disableallints(struct sam_1wire_s *priv, uint32_t *imr)
{
  irqstate_t flags;

  /* The following must be atomic */

  flags = enter_critical_section();
  if (imr)
    {
      /* Return the current interrupt mask */

      *imr = sam_serialin(priv, SAM_UART_IMR_OFFSET);
    }

  /* Disable all interrupts */

  sam_serialout(priv, SAM_UART_IDR_OFFSET, UART_INT_ALLINTS);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: sam_set_baud
 ****************************************************************************/

static int sam_set_baud(struct sam_1wire_s *priv, uint32_t baud)
{
  uint32_t divb3;
  uint32_t intpart;
  uint32_t fracpart;
  uint32_t regval;

  /* Configure the console baud:
   *
   *   Fbaud   = USART_CLOCK / (16 * divisor)
   *   divisor = USART_CLOCK / (16 * Fbaud)
   *
   * NOTE: Oversampling by 8 is not supported. This may limit BAUD rates
   * for lower USART clocks.
   */

  divb3    = ((FAST_USART_CLOCK + (baud << 3)) << 3) / (baud << 4);
  intpart  = divb3 >> 3;
  fracpart = divb3 & 7;

  /* Retain the fast MR peripheral clock UNLESS unless using that clock
   * would result in an excessively large divider.
   *
   * REVISIT: The fractional divider is not used.
   */

  if ((intpart & ~UART_BRGR_CD_MASK) != 0)
    {
      /* Use the divided USART clock */

      divb3    = ((SLOW_USART_CLOCK + (baud << 3)) << 3) /
                 (baud << 4);
      intpart  = divb3 >> 3;
      fracpart = divb3 & 7;

      /* Re-select the clock source */

      regval  = sam_serialin(priv, SAM_UART_MR_OFFSET);
      regval &= ~UART_MR_USCLKS_MASK;
      regval |= UART_MR_USCLKS_MCKDIV;
      sam_serialout(priv, SAM_UART_MR_OFFSET, regval);
    }

  /* Save the BAUD divider (the fractional part is not used for UARTs) */

  regval = UART_BRGR_CD(intpart) | UART_BRGR_FP(fracpart);
  sam_serialout(priv, SAM_UART_BRGR_OFFSET, regval);

  return OK;
}

/****************************************************************************
 * Name: sam_shutdown
 ****************************************************************************/

static void sam_shutdown(struct sam_1wire_s *priv)
{
  uint32_t regval;

  /* Reset and disable receiver and transmitter */

  sam_serialout(priv, SAM_UART_CR_OFFSET, (UART_CR_RSTRX | UART_CR_RSTTX |
                UART_CR_RXDIS | UART_CR_TXDIS));

  /* Set mode back to normal */

  regval = sam_serialin(priv, SAM_UART_MR_OFFSET);
  regval &= ~UART_MR_MODE_MASK;
  sam_serialout(priv, SAM_UART_MR_OFFSET, regval);

  /* Disable all interrupts */

  sam_disableallints(priv, NULL);
}

/****************************************************************************
 * Name: sam_init
 *
 * Description:
 *   Setup the 1-Wire hardware, ready for operation with defaults
 *
 ****************************************************************************/

static int sam_init(struct sam_1wire_s *priv)
{
  uint32_t regval;
  int ret;

  /* The shutdown method will put the UART in a known, disabled state */

  sam_shutdown(priv);

  regval = UART_MR_MODE_NORMAL | UART_MR_USCLKS_MCK | UART_MR_CHRL_8BITS | \
           UART_MR_PAR_NONE | UART_MR_NBSTOP_1;

  /* And save the new mode register value */

  sam_serialout(priv, SAM_UART_MR_OFFSET, regval);

  sam_set_baud(priv, RESET_BAUD);

  sam_serialout(priv, SAM_UART_IER_OFFSET, UART_INT_RXRDY);

  sam_configgpio(priv->rx);
  sam_configgpio(priv->tx | GPIO_CFG_OPENDRAIN);

  /* Enable receiver & transmitter */

  sam_serialout(priv, SAM_UART_CR_OFFSET, (UART_CR_RXEN | UART_CR_TXEN));

  ret = irq_attach(priv->irq, sam_interrupt, priv);
  if (ret == OK)
    {
      up_enable_irq(priv->irq);
    }

  return OK;
}

/****************************************************************************
 * Name: sam_interrupt
 *
 * Description:
 *  Common Interrupt Service Routine
 ****************************************************************************/

static int sam_interrupt(int irq, void *context, void *arg)
{
  struct sam_1wire_s *priv = (struct sam_1wire_s *)arg;
  uint32_t pending;
  uint32_t imr;
  uint32_t sr;
  uint32_t dr;

  DEBUGASSERT(priv != NULL);

  /* Get the masked USART status word. */

  sr       = sam_serialin(priv, SAM_UART_SR_OFFSET);
  imr      = sam_serialin(priv, SAM_UART_IMR_OFFSET);
  pending  = sr & imr;

  if ((pending & UART_INT_RXRDY) != 0)
    {
      /* Received data ready... process incoming bytes */

      dr = sam_receive(priv);

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

              sam_send(priv, (*priv->byte & (1 << priv->bit)) ?
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

              sam_send(priv, READ_TX);
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

  return OK;
}

/****************************************************************************
 * Name: sam_process
 *
 * Description:
 *  Execute 1-Wire task
 *
 ****************************************************************************/

static int sam_process(struct sam_1wire_s *priv,
                       const struct sam_1wire_msg_s *msgs, int count)
{
  irqstate_t irqs;
  uint8_t indx;
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

          sam_set_baud(priv, RESET_BAUD);

          /* Atomic */

          irqs = enter_critical_section();
          priv->msgs = &msgs[indx];
          sam_send(priv, RESET_TX);
          leave_critical_section(irqs);

          /* Wait.  Break on timeout if TX line closed to GND */

          nxsem_tickwait(&priv->sem_isr, SEC2TICK(BUS_TIMEOUT));
          break;

        case ONEWIRETASK_WRITE:
        case ONEWIRETASK_WRITEBIT:

          /* Set baud rate */

          sam_set_baud(priv, TIMESLOT_BAUD);

          /* Atomic */

          irqs = enter_critical_section();
          priv->msgs = &msgs[indx];
          priv->byte = priv->msgs->buffer;
          priv->bit = 0;
          sam_send(priv, (*priv->byte & (1 << priv->bit)) ?
                           WRITE_TX1 : WRITE_TX0);
          leave_critical_section(irqs);

          /* Wait.  Break on timeout if TX line closed to GND */

          nxsem_tickwait(&priv->sem_isr, SEC2TICK(BUS_TIMEOUT));
          break;

        case ONEWIRETASK_READ:
        case ONEWIRETASK_READBIT:

          /* Set baud rate */

          sam_set_baud(priv, TIMESLOT_BAUD);

          /* Atomic */

          irqs = enter_critical_section();
          priv->msgs = &msgs[indx];
          priv->byte = priv->msgs->buffer;
          priv->bit = 0;
          sam_send(priv, READ_TX);
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
 * Name: sam_writebit
 *
 * Description:
 *   Write one bit of 1-Wire data
 *
 ****************************************************************************/

static int sam_writebit(struct onewire_dev_s *dev, const uint8_t *bit)
{
  struct sam_1wire_s *priv = ((struct sam_1wire_inst_s *)dev)->priv;
  const struct sam_1wire_msg_s msgs[1] =
  {
    [0].task = ONEWIRETASK_WRITEBIT,
    [0].buffer = (uint8_t *)bit,
    [0].buflen = 1
  };

  DEBUGASSERT(*bit == 0 || *bit == 1);

  return sam_process(priv, msgs, 1);
}

/****************************************************************************
 * Name: stm32_1wire_reset
 *
 * Description:
 *   1-Wire reset pulse and presence detect.
 *
 ****************************************************************************/

static int sam_reset(struct onewire_dev_s *dev)
{
  struct sam_1wire_s *priv = ((struct sam_1wire_inst_s *)dev)->priv;
  const struct sam_1wire_msg_s msgs[1] =
  {
    [0].task = ONEWIRETASK_RESET
  };

  return sam_process(priv, msgs, 1);
}

/****************************************************************************
 * Name: sam_write
 *
 * Description:
 *   Write 1-Wire data
 *
 ****************************************************************************/

static int sam_write(struct onewire_dev_s *dev, const uint8_t *buffer,
                     int buflen)
{
  struct sam_1wire_s *priv = ((struct sam_1wire_inst_s *)dev)->priv;
  const struct sam_1wire_msg_s msgs[1] =
  {
    [0].task = ONEWIRETASK_WRITE,
    [0].buffer = (uint8_t *)buffer,
    [0].buflen = buflen
  };

  return sam_process(priv, msgs, 1);
}

/****************************************************************************
 * Name: sam_read
 *
 * Description:
 *   Read 1-Wire data
 *
 ****************************************************************************/

static int sam_read(struct onewire_dev_s *dev, uint8_t *buffer,
                     int buflen)
{
  struct sam_1wire_s *priv = ((struct sam_1wire_inst_s *)dev)->priv;
  const struct sam_1wire_msg_s msgs[1] =
  {
    [0].task = ONEWIRETASK_READ,
    [0].buffer = buffer,
    [0].buflen = buflen
  };

  return sam_process(priv, msgs, 1);
}

/****************************************************************************
 * Name: sam_exchange
 *
 * Description:
 *   1-Wire reset pulse and presence detect,
 *   Write 1-Wire data,
 *   Read 1-Wire data
 *
 ****************************************************************************/

static int sam_exchange(struct onewire_dev_s *dev, bool reset,
                        const uint8_t *txbuffer, int txbuflen,
                        uint8_t *rxbuffer, int rxbuflen)
{
  int result = ERROR;
  struct sam_1wire_s *priv = ((struct sam_1wire_inst_s *)dev)->priv;

  if (reset)
    {
      const struct sam_1wire_msg_s msgs[3] =
      {
        [0].task = ONEWIRETASK_RESET,

        [1].task = ONEWIRETASK_WRITE,
        [1].buffer = (uint8_t *)txbuffer,
        [1].buflen = txbuflen,

        [2].task = ONEWIRETASK_READ,
        [2].buffer = rxbuffer,
        [2].buflen = rxbuflen
      };

      result = sam_process(priv, msgs, 3);
    }
  else
    {
      const struct sam_1wire_msg_s msgs[2] =
      {
        [0].task = ONEWIRETASK_WRITE,
        [0].buffer = (uint8_t *)txbuffer,
        [0].buflen = txbuflen,

        [1].task = ONEWIRETASK_READ,
        [1].buffer = rxbuffer,
        [1].buflen = rxbuflen
      };

      result = sam_process(priv, msgs, 2);
    }

  return result;
}

/****************************************************************************
 * Name: sam_readbit
 *
 * Description:
 *   Sample one bit of 1-Wire data
 *
 ****************************************************************************/

static int sam_readbit(struct onewire_dev_s *dev, uint8_t *bit)
{
  struct sam_1wire_s *priv = ((struct sam_1wire_inst_s *)dev)->priv;
  const struct sam_1wire_msg_s msgs[1] =
  {
    [0].task = ONEWIRETASK_READBIT,
    [0].buffer = bit,
    [0].buflen = 1
  };

  return sam_process(priv, msgs, 1);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_1wireinitialize
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

struct onewire_dev_s *sam_1wireinitialize(int port)
{
  struct sam_1wire_s *priv = NULL;
  struct sam_1wire_inst_s *inst = NULL;

  /* Get 1-Wire private structure */

  switch (port)
    {
#ifdef CONFIG_SAMV7_UART0_1WIREDRIVER
    case 0:
      priv = &sam_1wire0_priv;
      break;
#endif
#ifdef CONFIG_SAMV7_UART1_1WIREDRIVER
    case 1:
      priv = &sam_1wire1_priv;
      break;
#endif
#ifdef CONFIG_SAMV7_UART2_1WIREDRIVER
    case 2:
      priv = &sam_1wire2_priv;
      break;
#endif
#ifdef CONFIG_SAMV7_UART3_1WIREDRIVER
    case 3:
      priv = &sam_1wire3_priv;
      break;
#endif
#ifdef CONFIG_SAMV7_UART4_1WIREDRIVER
    case 4:
      priv = &sam_1wire4_priv;
      break;
#endif
#ifdef CONFIG_SAMV7_USART0_1WIREDRIVER
    case 5:
      priv = &sam_1wire5_priv;
      break;
#endif
#ifdef CONFIG_SAMV7_USART1_1WIREDRIVER
    case 6:
      priv = &sam_1wire6_priv;
      break;
#endif
#ifdef CONFIG_SAMV7_USART2_1WIREDRIVER
    case 7:
      priv = &sam_1wire7_priv;
      break;
#endif
    default:
      return NULL;
    }

  inst = kmm_malloc(sizeof(*inst));
  if (inst == NULL)
    {
      return NULL;
    }

  inst->ops  = &g_1wire_ops;
  inst->priv = priv;

  nxmutex_lock(&priv->lock);
  if (priv->refs++ == 0)
    {
      sam_init(priv);
    }

  nxmutex_unlock(&priv->lock);
  return (struct onewire_dev_s *)inst;
}

#endif /* CONFIG_SAMV7_1WIREDRIVER */
