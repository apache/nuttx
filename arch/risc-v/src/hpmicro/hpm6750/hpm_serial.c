/****************************************************************************
 * arch/risc-v/src/hpm/hpm_serial.c
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
#include <nuttx/arch.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include "board.h"
#include "hpm_clock_drv.h"
#include "hpm_uart_drv.h"
#include "pinmux.h"

#include <nuttx/serial/serial.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/tioctl.h>

#include "riscv_internal.h"
#include "hpm_config.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If we are not using the serial driver for the console, then we still must
 * provide some minimal implementation of up_putc.
 */

#ifdef USE_SERIALDRIVER

/* Which UART with be tty0/console and which tty1?  The console will always
 * be ttyS0.  If there is no console then will use the lowest numbered UART.
 */

#ifdef HAVE_SERIAL_CONSOLE
#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#define CONSOLE_DEV g_uart0port /* UART0 is console */
#define TTYS0_DEV   g_uart0port /* UART0 is ttyS0 */
#undef TTYS1_DEV                /* No ttyS1 */
#define SERIAL_CONSOLE 1
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#define CONSOLE_DEV g_uart1port /* UART0 is console */
#error "I'm confused... Do we have a serial console or not?"
#endif
#else
#undef CONSOLE_DEV /* No console */
#undef CONFIG_UART0_SERIAL_CONSOLE
#if defined(CONFIG_HPM_UART0)
#define TTYS0_DEV g_uart0port /* UART0 is ttyS0 */
#undef TTYS1_DEV              /* No ttyS1 */
#define SERIAL_CONSOLE 1
#else
#undef TTYS0_DEV
#undef TTYS1_DEV
#endif
#endif

/* Common initialization logic will not not know that the all of the UARTs
 * have been disabled.  So, as a result, we may still have to provide
 * stub implementations of riscv_earlyserialinit(), riscv_serialinit(), and
 * up_putc().
 */

#ifdef HAVE_UART_DEVICE

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct hpm_uart_s
{
  uint32_t             base;
  uint32_t             irq_num;
  clock_name_t         clock_name;
  uart_config_t        config;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Serial driver methods */

static int  hpm_setup(struct uart_dev_s *dev);
static void hpm_shutdown(struct uart_dev_s *dev);
static int  hpm_attach(struct uart_dev_s *dev);
static void hpm_detach(struct uart_dev_s *dev);
static int  hpm_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  hpm_receive(struct uart_dev_s *dev, unsigned int *status);
static void hpm_rxint(struct uart_dev_s *dev, bool enable);
static bool hpm_rxavailable(struct uart_dev_s *dev);
static void hpm_send(struct uart_dev_s *dev, int ch);
static void hpm_txint(struct uart_dev_s *dev, bool enable);
static bool hpm_txready(struct uart_dev_s *dev);
static bool hpm_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_ops =
{
  .setup       = hpm_setup,
  .shutdown    = hpm_shutdown,
  .attach      = hpm_attach,
  .detach      = hpm_detach,
  .ioctl       = hpm_ioctl,
  .receive     = hpm_receive,
  .rxint       = hpm_rxint,
  .rxavailable = hpm_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol = NULL,
#endif
  .send    = hpm_send,
  .txint   = hpm_txint,
  .txready = hpm_txready,
  .txempty = hpm_txempty,
};

/* I/O buffers */

#ifdef CONFIG_HPM_UART0
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];

static struct hpm_uart_s g_uart0priv =
{
  .base     = HPM_UART0_BASE,
  .irq_num  = HPM_IRQ_PERI_START + IRQn_UART0,
  .clock_name = clock_uart0,
  .config =
    {
      .src_freq_in_hz = 24000000,
      .baudrate = 115200,
      .num_of_stop_bits = stop_bits_1,
      .word_length = word_length_8_bits,
      .parity = parity_none,
      .rx_fifo_level = uart_rx_fifo_trg_not_empty,
      .tx_fifo_level = uart_tx_fifo_trg_not_full,
      .fifo_enable = true,
      .dma_enable = false,
      .modem_config = 
        {
          .auto_flow_ctrl_en = false,
          .loop_back_en = false,
          .set_rts_high = false,
        },
    },
};

static uart_dev_t g_uart0port =
{
#ifdef CONFIG_UART0_SERIAL_CONSOLE
  .isconsole = 1,
#endif
  .recv =
    {
      .size   = CONFIG_UART0_RXBUFSIZE,
      .buffer = g_uart0rxbuffer,
    },
  .xmit =
    {
      .size   = CONFIG_UART0_TXBUFSIZE,
      .buffer = g_uart0txbuffer,
    },
  .ops  = &g_uart_ops,
  .priv = (void *)&g_uart0priv,
};
#endif

#ifdef CONFIG_HPM_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];

static struct hpm_uart_s g_uart1priv =
{
  .base     = HPM_UART1_BASE,
  .irq_num  = HPM_IRQ_PERI_START + IRQn_UART1,
  .clock_name = clock_uart1,
  .config =
    {
      .src_freq_in_hz = 24000000,
      .baudrate = 115200,
      .num_of_stop_bits = stop_bits_1,
      .word_length = word_length_8_bits,
      .parity = parity_none,
      .rx_fifo_level = uart_rx_fifo_trg_not_empty,
      .tx_fifo_level = uart_tx_fifo_trg_not_full,
      .fifo_enable = true,
      .dma_enable = false,
      .modem_config = 
        {
          .auto_flow_ctrl_en = false,
          .loop_back_en = false,
          .set_rts_high = false,
        },
    },
};

static uart_dev_t g_uart1port =
{
#ifdef CONFIG_UART1_SERIAL_CONSOLE
  .isconsole = 1,
#endif
  .recv =
    {
      .size   = CONFIG_UART1_RXBUFSIZE,
      .buffer = g_uart1rxbuffer,
    },
  .xmit =
    {
      .size   = CONFIG_UART1_TXBUFSIZE,
      .buffer = g_uart1txbuffer,
    },
  .ops  = &g_uart_ops,
  .priv = (void *)&g_uart1priv,
};
#endif

static struct uart_dev_s *const g_uart_devs[] =
{
#ifdef CONFIG_HPM_UART0
  [0] = &g_uart0port,
#endif
#ifdef CONFIG_HPM_UART1
  [1] = &g_uart1port
#endif
};

/****************************************************************************
 * Name: uart_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked when an
 *   interrupt received on the 'irq'  It should call uart_transmitchars or
 *   uart_receivechar to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'irq' number into the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int __uart_interrupt(int irq, void *context, void *arg)
{
  uart_dev_t *dev         = (uart_dev_t *)arg;
  struct hpm_uart_s *priv = dev->priv;
  UART_Type *uart_ptr     = (UART_Type *)priv->base;
  uint32_t irq_id;

  irq_id = uart_get_irq_id(uart_ptr);
  /* Length of uart rx data transfer arrived interrupt */

  if ((irq_id & uart_intr_id_rx_data_avail))
    {
      /* Receive Data ready */

      uart_recvchars(dev);
    }

  /* Tx fifo ready interrupt,auto-cleared when data is pushed */

  if ((irq_id & uart_intr_id_tx_slot_avail))
    {
      /* Transmit data request interrupt */

      uart_xmitchars(dev);
    }

  return OK;
}

/****************************************************************************
 * Name: hpm_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int hpm_setup(struct uart_dev_s *dev)
{
  struct hpm_uart_s *priv = (struct hpm_uart_s *)dev->priv;

  clock_set_source_divider(priv->clock_name, clk_src_osc24m, 1U);
  priv->config.src_freq_in_hz = clock_get_frequency(priv->clock_name);

  init_uart_pins((UART_Type *)priv->base);

  uart_init((UART_Type *)priv->base, &priv->config);

  return OK;
}

/****************************************************************************
 * Name: hpm_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void hpm_shutdown(struct uart_dev_s *dev)
{
  ;  /* Do Nothing */
}

/****************************************************************************
 * Name: hpm_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.  This method
 *   is called when the serial port is opened.  Normally, this is just after
 *   the the setup() method is called, however, the serial console may
 *   operate in a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled by the attach method (unless the
 *   hardware supports multiple levels of interrupt enabling).  The RX and TX
 *   interrupts are not enabled until the txint() and rxint() are called.
 *
 ****************************************************************************/

static int hpm_attach(struct uart_dev_s *dev)
{
  int                  ret;
  struct hpm_uart_s *priv = (struct hpm_uart_s *)dev->priv;

  ret = irq_attach(priv->irq_num, __uart_interrupt, (void *)dev);
  if (ret == OK)
    {
      up_enable_irq(priv->irq_num);
    }

  return ret;
}

/****************************************************************************
 * Name: hpm_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void hpm_detach(struct uart_dev_s *dev)
{
  struct hpm_uart_s *priv = (struct hpm_uart_s *)dev->priv;

  /* Disable interrupts */

  up_disable_irq(priv->irq_num);

  /* Detach from the interrupt */

  irq_detach(priv->irq_num);
}

/****************************************************************************
 * Name: hpm_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int hpm_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: hpm_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int hpm_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct hpm_uart_s *priv = (struct hpm_uart_s *)dev->priv;
  UART_Type *uart_ptr     = (UART_Type *)priv->base;
  uint8_t data;
  int rxdata;
  hpm_stat_t stat;

  stat = uart_receive_byte(uart_ptr, &data);
  if(stat == status_success)
    {
      rxdata = (int)data;
      *status = OK;
    }
  else
    {
      rxdata = ERROR;
      *status = ERROR;
    }

  return rxdata;
}

/****************************************************************************
 * Name: hpm_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void hpm_rxint(struct uart_dev_s *dev, bool enable)
{
  struct hpm_uart_s *priv = (struct hpm_uart_s *)dev->priv;
  UART_Type *uart_ptr     = (UART_Type *)priv->base;
  irqstate_t       flags  = enter_critical_section();

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      uart_enable_irq(uart_ptr, uart_intr_rx_data_avail_or_timeout);
#endif
    }
  else
    {
      uart_disable_irq(uart_ptr, uart_intr_rx_data_avail_or_timeout);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: hpm_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

static bool hpm_rxavailable(struct uart_dev_s *dev)
{
  struct hpm_uart_s *priv = (struct hpm_uart_s *)dev->priv;
  UART_Type *uart_ptr     = (UART_Type *)priv->base;

  /* Return true is data is available in the receive data buffer */

  return (uart_check_status(uart_ptr, uart_stat_data_ready));
}

/****************************************************************************
 * Name: hpm_send
 *
 * Description:
 *   This method will send one byte on the UART.
 *
 ****************************************************************************/

static void hpm_send(struct uart_dev_s *dev, int ch)
{
  struct hpm_uart_s *priv = (struct hpm_uart_s *)dev->priv;
  UART_Type *uart_ptr     = (UART_Type *)priv->base;

  while (status_success != uart_send_byte(uart_ptr, ch)) {
  }
}

/****************************************************************************
 * Name: hpm_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void hpm_txint(struct uart_dev_s *dev, bool enable)
{
  struct hpm_uart_s *priv = (struct hpm_uart_s *)dev->priv;
  UART_Type *uart_ptr     = (UART_Type *)priv->base;
  irqstate_t       flags;

  flags = enter_critical_section();

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      /* Enable the TX interrupt */

      uart_enable_irq(uart_ptr, uart_intr_tx_slot_avail);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
#endif
    }
  else
    {
      /* Disable the TX interrupt */

      uart_disable_irq(uart_ptr, uart_intr_tx_slot_avail);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: hpm_txready
 *
 * Description:
 *   Return true if the transmit data register is not full
 *
 ****************************************************************************/

static bool hpm_txready(struct uart_dev_s *dev)
{
  struct hpm_uart_s *priv = (struct hpm_uart_s *)dev->priv;
  UART_Type *uart_ptr     = (UART_Type *)priv->base;

  /* Return TRUE if the TX FIFO is not full */

  return (uart_check_status(uart_ptr, uart_stat_tx_slot_avail));
}

/****************************************************************************
 * Name: hpm_txempty
 *
 * Description:
 *   Return true if the transmit data register is empty
 *
 ****************************************************************************/

static bool hpm_txempty(struct uart_dev_s *dev)
{
  struct hpm_uart_s *priv = (struct hpm_uart_s *)dev->priv;
  UART_Type *uart_ptr     = (UART_Type *)priv->base;

  return (uart_check_status(uart_ptr, uart_stat_transmitter_empty));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT

/****************************************************************************
 * Name: riscv_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before riscv_serialinit.  NOTE:  This function depends on GPIO pin
 *   configuration performed in up_consoleinit() and main clock
 *   initialization performed in up_clkinitialize().
 *
 ****************************************************************************/

void riscv_earlyserialinit(void)
{
#ifdef HAVE_SERIAL_CONSOLE
  /* Configuration whichever one is the console */

  CONSOLE_DEV.isconsole = true;
  hpm_setup(&CONSOLE_DEV);
#endif
}
#endif

/****************************************************************************
 * Name: riscv_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that riscv_earlyserialinit was called previously.
 *
 ****************************************************************************/

void riscv_serialinit(void)
{
  int  i;
  char devname[16];

#ifdef HAVE_SERIAL_CONSOLE
  /* Register the console */

  uart_register("/dev/console", &CONSOLE_DEV);
#endif

  /* Register all UARTs */

  strcpy(devname, "/dev/ttySx");
  for (i = 0; i < sizeof(g_uart_devs) / sizeof(g_uart_devs[0]); i++)
    {
      if (g_uart_devs[i] == 0)
        {
          continue;
        }

      /* Don't create a device for the console - we did that above */

      if (g_uart_devs[i]->isconsole)
        {
          continue;
        }

      /* Register USARTs as devices in increasing order */

      devname[9] = '0' + i;
      uart_register(devname, g_uart_devs[i]);
    }
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug  writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  irqstate_t flags = enter_critical_section();

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      riscv_lowputc('\r');
    }

  riscv_lowputc(ch);
  leave_critical_section(flags);
#endif
  return ch;
}

/****************************************************************************
 * Name: riscv_earlyserialinit, riscv_serialinit, and up_putc
 *
 * Description:
 *   stubs that may be needed.  These stubs would be used if all UARTs are
 *   disabled.  In that case, the logic in common/up_initialize() is not
 *   smart enough to know that there are not UARTs and will still expect
 *   these interfaces to be provided.
 *
 ****************************************************************************/

#else /* HAVE_UART_DEVICE */
void riscv_earlyserialinit(void)
{
}

void riscv_serialinit(void)
{
}

int up_putc(int ch)
{
  return ch;
}

#endif /* HAVE_UART_DEVICE */
#else  /* USE_SERIALDRIVER */

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      riscv_lowputc('\r');
    }

  riscv_lowputc(ch);
#endif
  return ch;
}

#endif /* USE_SERIALDRIVER */
