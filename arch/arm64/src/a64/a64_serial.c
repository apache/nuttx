/***************************************************************************
 * arch/arm64/src/a64/a64_serial.c
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
 ***************************************************************************/

/* Reference:
 *
 * "NuttX RTOS for PinePhone: UART Driver"
 * https://lupyuen.github.io/articles/serial
 *
 * "A64 Page" refers to Allwinner A64 User Manual
 * https://lupyuen.github.io/images/Allwinner_A64_User_Manual_V1.1.pdf
 */

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spinlock.h>
#include <nuttx/init.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/semaphore.h>
#include <nuttx/serial/serial.h>

#include "arm64_arch.h"
#include "arm64_internal.h"
#include "a64_pio.h"
#include "a64_serial.h"
#include "arm64_arch_timer.h"
#include "a64_boot.h"
#include "arm64_gic.h"

#ifdef USE_SERIALDRIVER

/***************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

/* UART0 Settings should be same as U-Boot Bootloader */

#ifndef CONFIG_UART0_BAUD
#  define CONFIG_UART0_BAUD 115200
#endif

#ifndef CONFIG_UART0_BITS
#  define CONFIG_UART0_BITS 8
#endif

#ifndef CONFIG_UART0_PARITY
#  define CONFIG_UART0_PARITY 0
#endif

#ifndef CONFIG_UART0_2STOP
#  define CONFIG_UART0_2STOP 0
#endif

#ifndef CONFIG_UART0_RXBUFSIZE
#  define CONFIG_UART0_RXBUFSIZE 256
#endif

#ifndef CONFIG_UART0_TXBUFSIZE
#  define CONFIG_UART0_TXBUFSIZE 256
#endif

/* UART0 is console and ttys0, follows U-Boot Bootloader */

#define CONSOLE_DEV     g_uart0port         /* UART0 is console */
#define TTYS0_DEV       g_uart0port         /* UART0 is ttyS0 */
#define UART0_ASSIGNED  1

/* A64 UART SCLK is the UART Input Clock.  Through experimentation, it has
 * been found that the serial clock is OSC24M
 */

#define UART_SCLK 24000000

/* Timeout for UART Busy Wait, in milliseconds */

#define UART_TIMEOUT_MS 100

/* A64 UART I/O Pins *******************************************************/

/* UART1: PG6 and PG7 (Peripheral 2) */

#ifdef CONFIG_A64_UART1
#  define PIO_UART1_TX (PIO_PERIPH2 | PIO_PORT_PIOG | PIO_PIN6)
#  define PIO_UART1_RX (PIO_PERIPH2 | PIO_PORT_PIOG | PIO_PIN7)
#endif

/* UART2: PB0 and PB1 (Peripheral 2) */

#ifdef CONFIG_A64_UART2
#  define PIO_UART2_TX (PIO_PERIPH2 | PIO_PORT_PIOB | PIO_PIN0)
#  define PIO_UART2_RX (PIO_PERIPH2 | PIO_PORT_PIOB | PIO_PIN1)
#endif

/* UART3: PD0 and PD1 (Peripheral 3) */

#ifdef CONFIG_A64_UART3
#  define PIO_UART3_TX (PIO_PERIPH3 | PIO_PORT_PIOD | PIO_PIN0)
#  define PIO_UART3_RX (PIO_PERIPH3 | PIO_PORT_PIOD | PIO_PIN1)
#endif

/* UART4: PD2 and PD3 (Peripheral 3) */

#ifdef CONFIG_A64_UART4
#  define PIO_UART4_TX (PIO_PERIPH3 | PIO_PORT_PIOD | PIO_PIN2)
#  define PIO_UART4_RX (PIO_PERIPH3 | PIO_PORT_PIOD | PIO_PIN3)
#endif

/* A64 UART Registers and Bit Definitions **********************************/

/* A64 UART Registers (A64 Page 562) */

#define UART_THR(uart_addr) (uart_addr + 0x00)  /* Tx Holding */
#define UART_RBR(uart_addr) (uart_addr + 0x00)  /* Rx Buffer */
#define UART_DLL(uart_addr) (uart_addr + 0x00)  /* Divisor Latch Low */
#define UART_DLH(uart_addr) (uart_addr + 0x04)  /* Divisor Latch High */
#define UART_IER(uart_addr) (uart_addr + 0x04)  /* Interrupt Enable */
#define UART_IIR(uart_addr) (uart_addr + 0x08)  /* Interrupt Identity */
#define UART_FCR(uart_addr) (uart_addr + 0x08)  /* FIFO Control */
#define UART_LCR(uart_addr) (uart_addr + 0x0c)  /* Line Control */
#define UART_LSR(uart_addr) (uart_addr + 0x14)  /* Line Status */
#define UART_MSR(uart_addr) (uart_addr + 0x18)  /* Modem Status */
#define UART_USR(uart_addr) (uart_addr + 0x7c)  /* UART Status */

/* A64 UART Register Bit Definitions (A64 Page 565) */

#define UART_IER_ERBFI (1 << 0)  /* Enable Rx Data Interrupt */
#define UART_IER_ETBEI (1 << 1)  /* Enable Tx Empty Interrupt */
#define UART_LSR_DR    (1 << 0)  /* Rx Data Ready */
#define UART_LSR_THRE  (1 << 5)  /* Tx Empty */
#define UART_USR_BUSY  (1 << 0)  /* UART Busy */

/* A64 UART Interrupt Identity Register (A64 Page 565) */

#define UART_IIR_IID_SHIFT        (0) /* Bits: 0-3: Interrupt ID */
#define UART_IIR_IID_MASK         (15 << UART_IIR_IID_SHIFT)
#  define UART_IIR_IID_MODEM      (0 << UART_IIR_IID_SHIFT)  /* Modem status */
#  define UART_IIR_IID_NONE       (1 << UART_IIR_IID_SHIFT)  /* No interrupt pending */
#  define UART_IIR_IID_TXEMPTY    (2 << UART_IIR_IID_SHIFT)  /* THR empty */
#  define UART_IIR_IID_RECV       (4 << UART_IIR_IID_SHIFT)  /* Received data available */
#  define UART_IIR_IID_LINESTATUS (6 << UART_IIR_IID_SHIFT)  /* Receiver line status */
#  define UART_IIR_IID_BUSY       (7 << UART_IIR_IID_SHIFT)  /* Busy detect */
#  define UART_IIR_IID_TIMEOUT    (12 << UART_IIR_IID_SHIFT) /* Character timeout */

#define UART_IIR_FEFLAG_SHIFT     (6) /* Bits 6-7: FIFOs Enable Flag */
#define UART_IIR_FEFLAG_MASK      (3 << UART_IIR_FEFLAG_SHIFT)
#  define UART_IIR_FEFLAG_DISABLE (0 << UART_IIR_FEFLAG_SHIFT)
#  define UART_IIR_FEFLAG_ENABLE  (3 << UART_IIR_FEFLAG_SHIFT)

/* A64 UART FIFO Control Register (A64 Page 567) */

#define UART_FCR_FIFOE            (1 << 0)  /* Bit 0:  Enable FIFOs */
#define UART_FCR_RFIFOR           (1 << 1)  /* Bit 1:  RCVR FIFO Reset */
#define UART_FCR_XFIFOR           (1 << 2)  /* Bit 2:  XMIT FIFO reset */
#define UART_FCR_DMAM             (1 << 3)  /* Bit 3:  DMA mode */
#define UART_FCR_TFT_SHIFT        (4)       /* Bits 4-5: TX Empty Trigger */
#define UART_FCR_TFT_MASK         (3 << UART_FCR_TFT_SHIFT)
#  define UART_FCR_TFT_EMPTY      (0 << UART_FCR_TFT_SHIFT) /* FIFO empty */
#  define UART_FCR_TFT_TWO        (1 << UART_FCR_TFT_SHIFT) /* 2 characters in the FIFO */
#  define UART_FCR_TFT_QUARTER    (2 << UART_FCR_TFT_SHIFT) /* FIFO 1/4 full */
#  define UART_FCR_TFT_HALF       (3 << UART_FCR_TFT_SHIFT) /* FIFO 1/2 full */

#define UART_FCR_RT_SHIFT         (6)       /* Bits 6-7: RCVR Trigger */
#define UART_FCR_RT_MASK          (3 << UART_FCR_RT_SHIFT)
#  define UART_FCR_RT_ONE         (0 << UART_FCR_RT_SHIFT) /* 1 character in the FIFO */
#  define UART_FCR_RT_QUARTER     (1 << UART_FCR_RT_SHIFT) /* FIFO 1/4 full */
#  define UART_FCR_RT_HALF        (2 << UART_FCR_RT_SHIFT) /* FIFO 1/2 full */
#  define UART_FCR_RT_MINUS2      (3 << UART_FCR_RT_SHIFT) /* FIFO-2 less than full */

/* A64 UART Line Control Register (A64 Page 568) */

#define UART_LCR_DLS_SHIFT        (0)       /* Bits 0-1: Data Length Select */
#define UART_LCR_DLS_MASK         (3 << UART_LCR_DLS_SHIFT)
#  define UART_LCR_DLS_5BITS      (0 << UART_LCR_DLS_SHIFT) /* 5 bits */
#  define UART_LCR_DLS_6BITS      (1 << UART_LCR_DLS_SHIFT) /* 6 bits */
#  define UART_LCR_DLS_7BITS      (2 << UART_LCR_DLS_SHIFT) /* 7 bits */
#  define UART_LCR_DLS_8BITS      (3 << UART_LCR_DLS_SHIFT) /* 8 bits */

#define UART_LCR_STOP             (1 << 2)  /* Bit 2:  Number of stop bits */
#define UART_LCR_PEN              (1 << 3)  /* Bit 3:  Parity Enable */
#define UART_LCR_EPS              (1 << 4)  /* Bit 4:  Even Parity Select */
#define UART_LCR_BC               (1 << 6)  /* Bit 6:  Break Control Bit */
#define UART_LCR_DLAB             (1 << 7)  /* Bit 7:  Divisor Latch Access Bit */

/* A64 CCU Registers and Bit Definitions ***********************************/

/* Bus Clock Gating Register 3 (A64 Page 104) */

#define BUS_CLK_GATING_REG3 (A64_CCU_ADDR + 0x006C)
#define UART0_GATING        (1 << 16)
#define UART1_GATING        (1 << 17)
#define UART2_GATING        (1 << 18)
#define UART3_GATING        (1 << 19)
#define UART4_GATING        (1 << 20)

/* Bus Software Reset Register 4 (A64 Page 142) */

#define BUS_SOFT_RST_REG4 (A64_CCU_ADDR + 0x02D8)
#define UART0_RST         (1 << 16)
#define UART1_RST         (1 << 17)
#define UART2_RST         (1 << 18)
#define UART3_RST         (1 << 19)
#define UART4_RST         (1 << 20)

/***************************************************************************
 * Private Types
 ***************************************************************************/

/* A64 UART Configuration */

struct a64_uart_config
{
  unsigned long uart;  /* UART Base Address */
};

/* A64 UART Device Data */

struct a64_uart_data
{
  uint32_t baud_rate;  /* UART Baud Rate */
  uint32_t ier;        /* Saved IER value */
  uint8_t  parity;     /* 0=none, 1=odd, 2=even */
  uint8_t  bits;       /* Number of bits (7 or 8) */
  bool     stopbits2;  /* true: Configure with 2 stop bits instead of 1 */
};

/* A64 UART Port */

struct a64_uart_port_s
{
  struct a64_uart_data data;     /* UART Device Data */
  struct a64_uart_config config; /* UART Configuration */
  unsigned int irq_num;          /* UART IRQ Number */
  bool is_console;               /* 1 if this UART is console */
};

/***************************************************************************
 * Private Function Prototypes
 ***************************************************************************/

static void a64_uart_rxint(struct uart_dev_s *dev, bool enable);
static void a64_uart_txint(struct uart_dev_s *dev, bool enable);

/***************************************************************************
 * Private Functions
 ***************************************************************************/

/***************************************************************************
 * Name: a64_uart_divisor
 *
 * Description:
 *   Select a divisor to produce the BAUD from the UART SCLK.
 *
 *     BAUD = SCLK / (16 * DL), or
 *     DL   = SCLK / BAUD / 16
 *
 * Returned Value:
 *   UART Divisor
 *
 ***************************************************************************/

static uint32_t a64_uart_divisor(uint32_t baud)
{
  DEBUGASSERT(baud != 0);
  return UART_SCLK / (baud << 4);
}

/***************************************************************************
 * Name: a64_uart_irq_handler
 *
 * Description:
 *   This is the common UART interrupt handler.  It should call
 *   uart_xmitchars or uart_recvchars to perform the appropriate data
 *   transfers.
 *
 * Input Parameters:
 *   irq     - IRQ Number
 *   context - Interrupt Context
 *   arg     - UART Device
 *
 * Returned Value:
 *   OK is always returned at present.
 *
 ***************************************************************************/

static int a64_uart_irq_handler(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  const struct a64_uart_port_s *port = (struct a64_uart_port_s *)dev->priv;
  const struct a64_uart_config *config = &port->config;
  uint32_t status;
  int passes;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  for (passes = 0; passes < 256; passes++)
    {
      /* Get the current UART status */

      status = getreg32(UART_IIR(config->uart));

      /* Handle the interrupt by its interrupt ID field */

      switch (status & UART_IIR_IID_MASK)
        {
          /* Handle incoming, receive bytes (with or without timeout) */

          case UART_IIR_IID_RECV:
          case UART_IIR_IID_TIMEOUT:
            {
              uart_recvchars(dev);
              break;
            }

          /* Handle outgoing, transmit bytes */

          case UART_IIR_IID_TXEMPTY:
            {
              uart_xmitchars(dev);
              break;
            }

          /* Just clear modem status interrupts (UART1 only) */

          case UART_IIR_IID_MODEM:
            {
              /* Read the modem status register (MSR) to clear */

              status = getreg32(UART_MSR(config->uart));
              break;
            }

          /* Just clear any line status interrupts */

          case UART_IIR_IID_LINESTATUS:
            {
              /* Read the line status register (LSR) to clear */

              status = getreg32(UART_LSR(config->uart));
              break;
            }

          /* Busy detect.
           * Just ignore.
           * Cleared by reading the status register
           */

          case UART_IIR_IID_BUSY:
            {
              /* Read from the UART status register
               * to clear the BUSY condition
               */

              status = getreg32(UART_USR(config->uart));
              break;
            }

          /* No further interrupts pending... return now */

          case UART_IIR_IID_NONE:
            {
              return OK;
            }

            /* Otherwise we have received an interrupt
             * that we cannot handle
             */

          default:
            {
              _err("ERROR: Unexpected IIR: %02" PRIx32 "\n", status);
              break;
            }
        }
    }

  return OK;
}

/***************************************************************************
 * Name: a64_uart_wait
 *
 * Description:
 *   Wait for UART to be non-busy.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; ERROR if timeout.
 *
 ***************************************************************************/

static int a64_uart_wait(struct uart_dev_s *dev)
{
  struct a64_uart_port_s *port = (struct a64_uart_port_s *)dev->priv;
  const struct a64_uart_config *config = &port->config;
  int i;

  for (i = 0; i < UART_TIMEOUT_MS; i++)
    {
      uint32_t status = getreg32(UART_USR(config->uart));

      if ((status & UART_USR_BUSY) == 0)
        {
          return OK;
        }

      up_mdelay(1);
    }

  _err("UART timeout\n");
  return ERROR;
}

/***************************************************************************
 * Name: a64_uart_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This method is
 *   called the first time that the serial port is opened.
 *
 * Input Parameters:
 *   dev - UART Device
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ***************************************************************************/

static int a64_uart_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  struct a64_uart_port_s *port = (struct a64_uart_port_s *)dev->priv;
  const struct a64_uart_config *config = &port->config;
  struct a64_uart_data *data = &port->data;
  uint16_t dl;
  uint32_t lcr;
  int ret;

  DEBUGASSERT(data != NULL);

  /* Clear fifos */

  putreg32(UART_FCR_RFIFOR | UART_FCR_XFIFOR, UART_FCR(config->uart));

  /* Set trigger */

  putreg32(UART_FCR_FIFOE | UART_FCR_RT_HALF, UART_FCR(config->uart));

  /* Set up the IER */

  data->ier = getreg32(UART_IER(config->uart));

  /* Set up the LCR */

  lcr = 0;

  switch (data->bits)
    {
    case 5:
      lcr |= UART_LCR_DLS_5BITS;
      break;

    case 6:
      lcr |= UART_LCR_DLS_6BITS;
      break;

    case 7:
      lcr |= UART_LCR_DLS_7BITS;
      break;

    case 8:
    default:
      lcr |= UART_LCR_DLS_8BITS;
      break;
    }

  if (data->stopbits2)
    {
      lcr |= UART_LCR_STOP;
    }

  if (data->parity == 1)
    {
      lcr |= UART_LCR_PEN;
    }
  else if (data->parity == 2)
    {
      lcr |= (UART_LCR_PEN | UART_LCR_EPS);
    }

  /* Set DLAB when UART is not busy */

  ret = a64_uart_wait(dev);

  if (ret < 0)
    {
      _err("UART wait failed, ret=%d\n", ret);
      return ret;
    }

  putreg32(lcr | UART_LCR_DLAB, UART_LCR(config->uart));

  ret = a64_uart_wait(dev);

  if (ret < 0)
    {
      _err("UART wait failed, ret=%d\n", ret);
      return ret;
    }

  /* Set the BAUD divisor */

  dl = a64_uart_divisor(data->baud_rate);
  putreg8(dl >> 8,   UART_DLH(config->uart));
  putreg8(dl & 0xff, UART_DLL(config->uart));

  /* Check the BAUD divisor */

  if (getreg8(UART_DLH(config->uart)) != (dl >> 8) ||
      getreg8(UART_DLL(config->uart)) != (dl & 0xff))
    {
      _err("UART BAUD divisor failed\n");
      return ERROR;
    }

  /* Clear DLAB */

  putreg32(lcr, UART_LCR(config->uart));

  /* Configure the FIFOs */

  putreg32(UART_FCR_RT_HALF | UART_FCR_XFIFOR | UART_FCR_RFIFOR |
           UART_FCR_FIFOE, UART_FCR(config->uart));

  /* Enable Auto-Flow Control in the Modem Control Register */

#if defined(CONFIG_SERIAL_IFLOWCONTROL) || defined(CONFIG_SERIAL_OFLOWCONTROL)
#  warning Missing logic
#endif

#endif /* CONFIG_SUPPRESS_UART_CONFIG */
  return OK;
}

/***************************************************************************
 * Name: a64_uart_shutdown
 *
 * Description:
 *   Disable the UART Port.  This method is called when the serial
 *   port is closed.
 *
 * Input Parameters:
 *   dev - UART Device
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

static void a64_uart_shutdown(struct uart_dev_s *dev)
{
  /* Disable the Receive and Transmit Interrupts */

  a64_uart_rxint(dev, false);
  a64_uart_txint(dev, false);
}

/***************************************************************************
 * Name: a64_uart_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.
 *   This method is called when the serial port is opened.
 *   Normally, this is just after the setup() method is called,
 *   however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method
 *   (unless the hardware supports multiple levels of interrupt
 *   enabling).  The RX and TX interrupts are not enabled until
 *   the txint() and rxint() methods are called.
 *
 * Input Parameters:
 *   dev - UART Device
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ***************************************************************************/

static int a64_uart_attach(struct uart_dev_s *dev)
{
  int ret;
  const struct a64_uart_port_s *port = (struct a64_uart_port_s *)dev->priv;

  DEBUGASSERT(port != NULL);

  /* Attach UART Interrupt Handler */

  ret = irq_attach(port->irq_num, a64_uart_irq_handler, dev);

  /* Set Interrupt Priority in Generic Interrupt Controller v2 */

  arm64_gic_irq_set_priority(port->irq_num, 0, IRQ_TYPE_LEVEL);

  /* Enable UART Interrupt */

  if (ret == OK)
    {
      up_enable_irq(port->irq_num);
    }
  else
    {
      _err("IRQ attach failed, ret=%d\n", ret);
    }

  return ret;
}

/***************************************************************************
 * Name: a64_uart_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 * Input Parameters:
 *   dev - UART Device
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

static void a64_uart_detach(struct uart_dev_s *dev)
{
  const struct a64_uart_port_s *port = (struct a64_uart_port_s *)dev->priv;

  DEBUGASSERT(port != NULL);

  /* Disable UART Interrupt */

  up_disable_irq(port->irq_num);

  /* Detach UART Interrupt Handler */

  irq_detach(port->irq_num);
}

/***************************************************************************
 * Name: a64_uart_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method.
 *
 * Input Parameters:
 *   filep - File Struct
 *   cmd   - ioctl Command
 *   arg   - ioctl Argument
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ***************************************************************************/

static int a64_uart_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  int ret = OK;

  UNUSED(filep);
  UNUSED(arg);

  switch (cmd)
    {
      case TIOCSBRK:  /* BSD compatibility: Turn break on, unconditionally */
      case TIOCCBRK:  /* BSD compatibility: Turn break off, unconditionally */
      default:
        {
          ret = -ENOTTY;
          break;
        }
    }

  return ret;
}

/***************************************************************************
 * Name: a64_uart_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 * Input Parameters:
 *   dev    - UART Device
 *   status - Return status, zero on success
 *
 * Returned Value:
 *   Received character
 *
 ***************************************************************************/

static int a64_uart_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct a64_uart_port_s *port = (struct a64_uart_port_s *)dev->priv;
  const struct a64_uart_config *config = &port->config;
  uint32_t rbr;

  *status = getreg8(UART_LSR(config->uart));
  rbr     = getreg8(UART_RBR(config->uart));
  return rbr;
}

/***************************************************************************
 * Name: a64_uart_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 * Input Parameters:
 *   dev    - UART Device
 *   enable - True to enable RX interrupts; false to disable
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

static void a64_uart_rxint(struct uart_dev_s *dev, bool enable)
{
  const struct a64_uart_port_s *port = (struct a64_uart_port_s *)dev->priv;
  const struct a64_uart_config *config = &port->config;

  /* Write to Interrupt Enable Register (UART_IER) */

  if (enable)
    {
      /* Set ERBFI bit (Enable Rx Data Available Interrupt) */

      modreg8(UART_IER_ERBFI, UART_IER_ERBFI, UART_IER(config->uart));
    }
  else
    {
      /* Clear ERBFI bit (Disable Rx Data Available Interrupt) */

      modreg8(0, UART_IER_ERBFI, UART_IER(config->uart));
    }
}

/***************************************************************************
 * Name: a64_uart_rxavailable
 *
 * Description:
 *   Return true if the Receive FIFO is not empty
 *
 * Input Parameters:
 *   dev - UART Device
 *
 * Returned Value:
 *   True if the Receive FIFO is not empty; false otherwise
 *
 ***************************************************************************/

static bool a64_uart_rxavailable(struct uart_dev_s *dev)
{
  const struct a64_uart_port_s *port = (struct a64_uart_port_s *)dev->priv;
  const struct a64_uart_config *config = &port->config;

  /* Data Ready Bit (Line Status Register) is 1 if Rx Data is ready */

  return getreg8(UART_LSR(config->uart)) & UART_LSR_DR;
}

/***************************************************************************
 * Name: a64_uart_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 * Input Parameters:
 *   dev - UART Device
 *   ch  - Character to be sent
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

static void a64_uart_send(struct uart_dev_s *dev, int ch)
{
  const struct a64_uart_port_s *port = (struct a64_uart_port_s *)dev->priv;
  const struct a64_uart_config *config = &port->config;

  /* Write char to Transmit Holding Register (UART_THR) */

  putreg8(ch, UART_THR(config->uart));
}

/***************************************************************************
 * Name: a64_uart_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 * Input Parameters:
 *   dev    - UART Device
 *   enable - True to enable TX interrupts; false to disable
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

static void a64_uart_txint(struct uart_dev_s *dev, bool enable)
{
  const struct a64_uart_port_s *port = (struct a64_uart_port_s *)dev->priv;
  const struct a64_uart_config *config = &port->config;

  /* Write to Interrupt Enable Register (UART_IER) */

  if (enable)
    {
      /* Set ETBEI bit (Enable Tx Holding Register Empty Interrupt) */

      modreg8(UART_IER_ETBEI, UART_IER_ETBEI, UART_IER(config->uart));
    }
  else
    {
      /* Clear ETBEI bit (Disable Tx Holding Register Empty Interrupt) */

      modreg8(0, UART_IER_ETBEI, UART_IER(config->uart));
    }
}

/***************************************************************************
 * Name: a64_uart_txready
 *
 * Description:
 *   Return true if the Transmit FIFO is not full
 *
 * Input Parameters:
 *   dev - UART Device
 *
 * Returned Value:
 *   True if the Transmit FIFO is not full; false otherwise
 *
 ***************************************************************************/

static bool a64_uart_txready(struct uart_dev_s *dev)
{
  const struct a64_uart_port_s *port = (struct a64_uart_port_s *)dev->priv;
  const struct a64_uart_config *config = &port->config;

  /* Tx FIFO is ready if THRE Bit is 1 (Tx Holding Register Empty) */

  return (getreg8(UART_LSR(config->uart)) & UART_LSR_THRE) != 0;
}

/***************************************************************************
 * Name: a64_uart_txempty
 *
 * Description:
 *   Return true if the Transmit FIFO is empty
 *
 * Input Parameters:
 *   dev - UART Device
 *
 * Returned Value:
 *   True if the Transmit FIFO is empty; false otherwise
 *
 ***************************************************************************/

static bool a64_uart_txempty(struct uart_dev_s *dev)
{
  /* Tx FIFO is empty if Tx FIFO is not full (for now) */

  return a64_uart_txready(dev);
}

/***************************************************************************
 * Name: a64_uart_wait_send
 *
 * Description:
 *   Wait for Transmit FIFO until it is not full, then transmit the
 *   character over UART.
 *
 * Input Parameters:
 *   dev - UART Device
 *   ch  - Character to be sent
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

static void a64_uart_wait_send(struct uart_dev_s *dev, int ch)
{
  DEBUGASSERT(dev != NULL);
  while (!a64_uart_txready(dev));
  a64_uart_send(dev, ch);
}

/***************************************************************************
 * Name: a64_uart_init
 *
 * Description:
 *   Configure the UART.  Enable the clocking, deassert the reset and
 *   configure the I/O pins.
 *
 * Input Parameters:
 *   gating - UART Gating Bit (e.g. UART1_GATING)
 *   rst    - UART Reset Bit (e.g. UART1_RST)
 *   tx     - UART Transmit Pin (e.g. PIO_UART1_TX)
 *   rx     - UART Receive Pin (e.g. PIO_UART1_RX)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ***************************************************************************/

static int a64_uart_init(uint32_t gating, uint32_t rst, pio_pinset_t tx,
                         pio_pinset_t rx)
{
  irqstate_t flags;
  int ret = OK;

  flags = enter_critical_section();

  /* Enable clocking to UART */

  modreg32(gating, gating, BUS_CLK_GATING_REG3);

  /* Deassert reset for UART */

  modreg32(rst, rst, BUS_SOFT_RST_REG4);

  /* Configure I/O pins for UART */

  ret = a64_pio_config(tx);
  if (ret < 0)
    {
      _err("UART TX Pin config failed, ret=%d\n", ret);
    }
  else
    {
      ret = a64_pio_config(rx);
      if (ret < 0)
        {
          _err("UART RX Pin config failed, ret=%d\n", ret);
        }
    }

  leave_critical_section(flags);
  return ret;
};

/***************************************************************************
 * Private Data
 ***************************************************************************/

/* UART Operations for Serial Driver */

static const struct uart_ops_s g_uart_ops =
{
  .setup    = a64_uart_setup,
  .shutdown = a64_uart_shutdown,
  .attach   = a64_uart_attach,
  .detach   = a64_uart_detach,
  .ioctl    = a64_uart_ioctl,
  .receive  = a64_uart_receive,
  .rxint    = a64_uart_rxint,
  .rxavailable = a64_uart_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol    = NULL,
#endif
  .send     = a64_uart_send,
  .txint    = a64_uart_txint,
  .txready  = a64_uart_txready,
  .txempty  = a64_uart_txempty,
};

/* UART0 Port State (Console) */

static struct a64_uart_port_s g_uart0priv =
{
  .data   =
    {
      .baud_rate  = CONFIG_UART0_BAUD,
      .parity     = CONFIG_UART0_PARITY,
      .bits       = CONFIG_UART0_BITS,
      .stopbits2  = CONFIG_UART0_2STOP
    },

  .config =
    {
      .uart       = A64_UART0_ADDR
    },

    .irq_num      = A64_UART0_IRQ,
    .is_console   = 1
};

#ifdef CONFIG_A64_UART

/* UART0 I/O Buffers (Console) */

static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];

/* UART0 Port Definition (Console) */

static struct uart_dev_s g_uart0port =
{
  .recv  =
    {
      .size   = CONFIG_UART0_RXBUFSIZE,
      .buffer = g_uart0rxbuffer,
    },

  .xmit  =
    {
      .size   = CONFIG_UART0_TXBUFSIZE,
      .buffer = g_uart0txbuffer,
    },

  .ops   = &g_uart_ops,
  .priv  = &g_uart0priv,
};

#endif /* CONFIG_A64_UART */

#ifdef CONFIG_A64_UART1

/* UART1 Port State */

static struct a64_uart_port_s g_uart1priv =
{
  .data   =
    {
      .baud_rate  = CONFIG_UART1_BAUD,
      .parity     = CONFIG_UART1_PARITY,
      .bits       = CONFIG_UART1_BITS,
      .stopbits2  = CONFIG_UART1_2STOP
    },

  .config =
    {
      .uart       = A64_UART1_ADDR
    },

    .irq_num      = A64_UART1_IRQ,
    .is_console   = 0
};

/* UART1 I/O Buffers */

static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];

/* UART1 Port Definition */

static struct uart_dev_s g_uart1port =
{
  .recv  =
    {
      .size   = CONFIG_UART1_RXBUFSIZE,
      .buffer = g_uart1rxbuffer,
    },

  .xmit  =
    {
      .size   = CONFIG_UART1_TXBUFSIZE,
      .buffer = g_uart1txbuffer,
    },

  .ops   = &g_uart_ops,
  .priv  = &g_uart1priv,
};

#endif /* CONFIG_A64_UART1 */

#ifdef CONFIG_A64_UART2

/* UART2 Port State */

static struct a64_uart_port_s g_uart2priv =
{
  .data   =
    {
      .baud_rate  = CONFIG_UART2_BAUD,
      .parity     = CONFIG_UART2_PARITY,
      .bits       = CONFIG_UART2_BITS,
      .stopbits2  = CONFIG_UART2_2STOP
    },

  .config =
    {
      .uart       = A64_UART2_ADDR
    },

    .irq_num      = A64_UART2_IRQ,
    .is_console   = 0
};

/* UART2 I/O Buffers */

static char g_uart2rxbuffer[CONFIG_UART2_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_UART2_TXBUFSIZE];

/* UART2 Port Definition */

static struct uart_dev_s g_uart2port =
{
  .recv  =
    {
      .size   = CONFIG_UART2_RXBUFSIZE,
      .buffer = g_uart2rxbuffer,
    },

  .xmit  =
    {
      .size   = CONFIG_UART2_TXBUFSIZE,
      .buffer = g_uart2txbuffer,
    },

  .ops   = &g_uart_ops,
  .priv  = &g_uart2priv,
};

#endif /* CONFIG_A64_UART2 */

#ifdef CONFIG_A64_UART3

/* UART3 Port State */

static struct a64_uart_port_s g_uart3priv =
{
  .data   =
    {
      .baud_rate  = CONFIG_UART3_BAUD,
      .parity     = CONFIG_UART3_PARITY,
      .bits       = CONFIG_UART3_BITS,
      .stopbits2  = CONFIG_UART3_2STOP
    },

  .config =
    {
      .uart       = A64_UART3_ADDR
    },

    .irq_num      = A64_UART3_IRQ,
    .is_console   = 0
};

/* UART3 I/O Buffers */

static char g_uart3rxbuffer[CONFIG_UART3_RXBUFSIZE];
static char g_uart3txbuffer[CONFIG_UART3_TXBUFSIZE];

/* UART3 Port Definition */

static struct uart_dev_s g_uart3port =
{
  .recv  =
    {
      .size   = CONFIG_UART3_RXBUFSIZE,
      .buffer = g_uart3rxbuffer,
    },

  .xmit  =
    {
      .size   = CONFIG_UART3_TXBUFSIZE,
      .buffer = g_uart3txbuffer,
    },

  .ops   = &g_uart_ops,
  .priv  = &g_uart3priv,
};

#endif /* CONFIG_A64_UART3 */

#ifdef CONFIG_A64_UART4

/* UART4 Port State */

static struct a64_uart_port_s g_uart4priv =
{
  .data   =
    {
      .baud_rate  = CONFIG_UART4_BAUD,
      .parity     = CONFIG_UART4_PARITY,
      .bits       = CONFIG_UART4_BITS,
      .stopbits2  = CONFIG_UART4_2STOP
    },

  .config =
    {
      .uart       = A64_UART4_ADDR
    },

    .irq_num      = A64_UART4_IRQ,
    .is_console   = 0
};

/* UART4 I/O Buffers */

static char g_uart4rxbuffer[CONFIG_UART4_RXBUFSIZE];
static char g_uart4txbuffer[CONFIG_UART4_TXBUFSIZE];

/* UART4 Port Definition */

static struct uart_dev_s g_uart4port =
{
  .recv  =
    {
      .size   = CONFIG_UART4_RXBUFSIZE,
      .buffer = g_uart4rxbuffer,
    },

  .xmit  =
    {
      .size   = CONFIG_UART4_TXBUFSIZE,
      .buffer = g_uart4txbuffer,
    },

  .ops   = &g_uart_ops,
  .priv  = &g_uart4priv,
};

#endif /* CONFIG_A64_UART4 */

/* Pick ttys1.  This could be any of UART1-4. */

#if defined(CONFIG_A64_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS1_DEV           g_uart1port /* UART1 is ttyS1 */
#  define UART1_ASSIGNED      1
#elif defined(CONFIG_A64_UART2) && !defined(UART2_ASSIGNED)
#  define TTYS1_DEV           g_uart2port /* UART2 is ttyS1 */
#  define UART2_ASSIGNED      1
#elif defined(CONFIG_A64_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS1_DEV           g_uart3port /* UART3 is ttyS1 */
#  define UART3_ASSIGNED      1
#elif defined(CONFIG_A64_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS1_DEV           g_uart4port /* UART4 is ttyS1 */
#  define UART4_ASSIGNED      1
#endif

/* Pick ttys2.  This could be one of UART2-4. */

#if defined(CONFIG_A64_UART2) && !defined(UART2_ASSIGNED)
#  define TTYS2_DEV           g_uart2port /* UART2 is ttyS2 */
#  define UART2_ASSIGNED      1
#elif defined(CONFIG_A64_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS2_DEV           g_uart3port /* UART3 is ttyS2 */
#  define UART3_ASSIGNED      1
#elif defined(CONFIG_A64_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS2_DEV           g_uart4port /* UART4 is ttyS2 */
#  define UART4_ASSIGNED      1
#endif

/* Pick ttys3.  This could be one of UART3-4. */

#if defined(CONFIG_A64_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS3_DEV           g_uart3port /* UART3 is ttyS3 */
#  define UART3_ASSIGNED      1
#elif defined(CONFIG_A64_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS3_DEV           g_uart4port /* UART4 is ttyS3 */
#  define UART4_ASSIGNED      1
#endif

/* Pick ttys4.  This could only be UART4. */

#if defined(CONFIG_A64_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS4_DEV           g_uart4port /* UART4 is ttyS4 */
#  define UART4_ASSIGNED      1
#endif

/***************************************************************************
 * Public Functions
 ***************************************************************************/

/***************************************************************************
 * Name: arm64_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in
 *   debug so that the serial console will be available
 *   during bootup.  This must be called before arm64_serialinit.
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

void arm64_earlyserialinit(void)
{
  int ret;

  /* NOTE: This function assumes that UART0 low level hardware configuration
   * -- including all clocking and pin configuration -- was performed
   * earlier by U-Boot Bootloader.
   */

#ifdef CONFIG_A64_UART1
  /* Configure UART1 */

  ret = a64_uart_init(UART1_GATING, UART1_RST, PIO_UART1_TX, PIO_UART1_RX);

  if (ret < 0)
    {
      _err("UART1 config failed, ret=%d\n", ret);
    }
#endif /* CONFIG_A64_UART1 */

#ifdef CONFIG_A64_UART2
  /* Configure UART2 */

  ret = a64_uart_init(UART2_GATING, UART2_RST, PIO_UART2_TX, PIO_UART2_RX);

  if (ret < 0)
    {
      _err("UART2 config failed, ret=%d\n", ret);
    }
#endif /* CONFIG_A64_UART2 */

#ifdef CONFIG_A64_UART3
  /* Configure UART3 */

  ret = a64_uart_init(UART3_GATING, UART3_RST, PIO_UART3_TX, PIO_UART3_RX);

  if (ret < 0)
    {
      _err("UART3 config failed, ret=%d\n", ret);
    }
#endif /* CONFIG_A64_UART3 */

#ifdef CONFIG_A64_UART4
  /* Configure UART4 */

  ret = a64_uart_init(UART4_GATING, UART4_RST, PIO_UART4_TX, PIO_UART4_RX);

  if (ret < 0)
    {
      _err("UART4 config failed, ret=%d\n", ret);
    }
#endif /* CONFIG_A64_UART4 */

#ifdef CONSOLE_DEV
  /* Enable the console at UART0 */

  CONSOLE_DEV.isconsole = true;
  a64_uart_setup(&CONSOLE_DEV);
#endif

  UNUSED(a64_uart_init);
  UNUSED(ret);
}

/***************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug
 *   writes
 *
 * Input Parameters:
 *   ch - Character to be transmitted over UART
 *
 * Returned Value:
 *   Character that was transmitted
 *
 ***************************************************************************/

int up_putc(int ch)
{
#ifdef CONSOLE_DEV
  struct uart_dev_s *dev = &CONSOLE_DEV;

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      a64_uart_wait_send(dev, '\r');
    }

  a64_uart_wait_send(dev, ch);
#endif
  return ch;
}

/***************************************************************************
 * Name: arm64_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that a64_earlyserialinit was called previously.
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

void arm64_serialinit(void)
{
#ifdef CONSOLE_DEV
  int ret;

  ret = uart_register("/dev/console", &CONSOLE_DEV);
  if (ret < 0)
    {
      _err("Register /dev/console failed, ret=%d\n", ret);
    }

  ret = uart_register("/dev/ttyS0", &TTYS0_DEV);

  if (ret < 0)
    {
      _err("Register /dev/ttyS0 failed, ret=%d\n", ret);
    }

#ifdef TTYS1_DEV
  ret = uart_register("/dev/ttyS1", &TTYS1_DEV);

  if (ret < 0)
    {
      _err("Register /dev/ttyS1 failed, ret=%d\n", ret);
    }
#endif /* TTYS1_DEV */

#ifdef TTYS2_DEV
  ret = uart_register("/dev/ttyS2", &TTYS2_DEV);

  if (ret < 0)
    {
      _err("Register /dev/ttyS2 failed, ret=%d\n", ret);
    }
#endif /* TTYS2_DEV */

#ifdef TTYS3_DEV
  ret = uart_register("/dev/ttyS3", &TTYS3_DEV);

  if (ret < 0)
    {
      _err("Register /dev/ttyS3 failed, ret=%d\n", ret);
    }
#endif /* TTYS3_DEV */

#ifdef TTYS4_DEV
  ret = uart_register("/dev/ttyS4", &TTYS4_DEV);

  if (ret < 0)
    {
      _err("Register /dev/ttyS4 failed, ret=%d\n", ret);
    }
#endif /* TTYS4_DEV */

#endif
}

#endif /* USE_SERIALDRIVER */
