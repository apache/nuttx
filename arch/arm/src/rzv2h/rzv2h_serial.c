/****************************************************************************
 * arch/arm/src/rzv2h/rzv2h_serial.c
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

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/serial.h>

#include <arch/board/board.h>

#include "arm_internal.h"

#include "hardware/rzv2h_pinmap.h"
#include "rzv2h_fsp_err.h"
#include "rzv2h_gpio.h"
#include "rzv2h_irq.h"
#include "rzv2h_lowputc.h"
#include "rzv2h_serial.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RZV2H_UART_BAUD_ERROR 3000
#define RZV2H_NUARTS          10

/* FSP SPI-relative SCI-B interrupt layout. */

#define RZV2H_SCI_IRQ_BASE              SCI0_ERI_IRQn
#define RZV2H_SCI_IRQ_CHANNEL_OFFSET    6
#define RZV2H_SCI_ERI_OFFSET            0
#define RZV2H_SCI_RXI_OFFSET            1
#define RZV2H_SCI_TXI_OFFSET            2
#define RZV2H_SCI_TEI_OFFSET            3

#define RZV2H_SCI_IRQ(n, offset) \
  (RZV2H_SCI_IRQ_BASE + (RZV2H_SCI_IRQ_CHANNEL_OFFSET * (n)) + (offset))

#define RZV2H_SCI_ERI_IRQ(n) RZV2H_SCI_IRQ(n, RZV2H_SCI_ERI_OFFSET)
#define RZV2H_SCI_RXI_IRQ(n) RZV2H_SCI_IRQ(n, RZV2H_SCI_RXI_OFFSET)
#define RZV2H_SCI_TXI_IRQ(n) RZV2H_SCI_IRQ(n, RZV2H_SCI_TXI_OFFSET)
#define RZV2H_SCI_TEI_IRQ(n) RZV2H_SCI_IRQ(n, RZV2H_SCI_TEI_OFFSET)

#define RZV2H_UART_ERROR_EVENTS \
  (UART_EVENT_ERR_OVERFLOW | UART_EVENT_ERR_FRAMING | \
   UART_EVENT_ERR_PARITY | UART_EVENT_BREAK_DETECT)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  const uart_instance_t *hal_inst;       /* FSP UART instance */
  struct uart_dev_s *dev;                /* NuttX UART device */
  volatile bool rx_enabled;              /* Upper-half RX state */
  volatile bool tx_enabled;              /* Upper-half TX state */
  volatile bool tx_idle;                 /* FSP TX is idle */
  bool opened;                           /* FSP channel is open */
  volatile bool rx_pending;              /* RX byte is pending */
  uint8_t rx_char;                       /* Persistent RX storage */
  uint8_t tx_char;                       /* Persistent TX storage */
  volatile uint32_t last_error_flags;    /* Pending RX error flags */
  uint32_t configured_baud;              /* Current baud rate */

#ifdef CONFIG_SERIAL_TIOCGICOUNT
  struct serial_icounter_s icount;       /* Serial event counters */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

void sci_b_uart_rxi_isr(void);
void sci_b_uart_txi_isr(void);
void sci_b_uart_tei_isr(void);
void sci_b_uart_eri_isr(void);

static int rzv2h_uart_interrupt(int irq, void *context, void *arg);
static int up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
static int up_ioctl(struct file *filep, int cmd, unsigned long arg);
static int up_receive(struct uart_dev_s *dev, unsigned int *status);
static void up_rxint(struct uart_dev_s *dev, bool is_enabled);
static bool up_rxavailable(struct uart_dev_s *dev);
static void up_send(struct uart_dev_s *dev, int ch);
static void up_txint(struct uart_dev_s *dev, bool is_enabled);
static bool up_txready(struct uart_dev_s *dev);
static bool up_txempty(struct uart_dev_s *dev);

/* Private Function Helpers */

static int rzv2h_fsp_uart_baud_calculate(
  uint32_t baud, sci_b_baud_setting_t *setting);
static int rzv2h_fsp_uart_open(struct up_dev_s *ctx);
static int rzv2h_fsp_uart_close(struct up_dev_s *ctx);
#ifdef CONFIG_SERIAL_TERMIOS
static int rzv2h_fsp_uart_set_baud(struct up_dev_s *ctx, uint32_t baud);
#endif
static void rzv2h_fsp_uart_rxint(struct up_dev_s *ctx, bool is_enabled);
static void rzv2h_fsp_uart_rx_rearm(struct up_dev_s *ctx);
static void rzv2h_fsp_uart_txint(struct up_dev_s *ctx, bool is_enabled);
static int rzv2h_fsp_uart_send_byte(struct up_dev_s *ctx, uint8_t ch);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_ops =
{
  .setup        = up_setup,
  .shutdown     = up_shutdown,
  .attach       = up_attach,
  .detach       = up_detach,
  .ioctl        = up_ioctl,
  .receive      = up_receive,
  .rxint        = up_rxint,
  .rxavailable  = up_rxavailable,
  .send         = up_send,
  .txint        = up_txint,
  .txready      = up_txready,
  .txempty      = up_txempty,
};

#ifdef CONFIG_RZV2H_SCI_B_UART0
static char g_uart0rxbuffer[CONFIG_SCI0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_SCI0_TXBUFSIZE];
static struct up_dev_s g_uart0priv;
static sci_b_uart_instance_ctrl_t g_fsp_uart0_ctrl;
static sci_b_baud_setting_t g_fsp_uart0_baud_setting;

static sci_b_uart_extended_cfg_t g_fsp_uart0_cfg_extend =
{
  .clock = SCI_B_UART_CLOCK_INT,
  .rx_edge_start = SCI_B_UART_START_BIT_FALLING_EDGE,
  .noise_cancel = SCI_B_UART_NOISE_CANCELLATION_ENABLE,
  .rx_fifo_trigger = SCI_B_UART_RX_FIFO_TRIGGER_MAX,
  .p_baud_setting = &g_fsp_uart0_baud_setting,
  .flow_control = SCI_B_UART_FLOW_CONTROL_RTS,
  .flow_control_pin = 0xffff,
  .rs485_setting =
    {
      .enable = SCI_B_UART_RS485_DISABLE,
      .polarity = SCI_B_UART_RS485_DE_POLARITY_HIGH,
      .assertion_time = 1,
      .negation_time = 1,
    },
  .half_data_setting =
    {
      .enable = 0,
    },
  .p_reg = (void *)R_SCI0,
};

static uart_cfg_t g_fsp_uart0_cfg =
{
  .channel = 0,
  .data_bits = CONFIG_SCI0_BITS == 7 ?
               UART_DATA_BITS_7 : UART_DATA_BITS_8,
  .parity = CONFIG_SCI0_PARITY == 1 ? UART_PARITY_ODD :
            CONFIG_SCI0_PARITY == 2 ? UART_PARITY_EVEN : UART_PARITY_OFF,
  .stop_bits = CONFIG_SCI0_2STOP ? UART_STOP_BITS_2 : UART_STOP_BITS_1,
  .p_extend = &g_fsp_uart0_cfg_extend,
  .p_transfer_tx = NULL,
  .p_transfer_rx = NULL,
  .rxi_ipl = CONFIG_RZV2H_SCI_B_IRQ_PRIORITY,
  .rxi_irq = RZV2H_SCI_RXI_IRQ(0),
  .txi_ipl = CONFIG_RZV2H_SCI_B_IRQ_PRIORITY,
  .txi_irq = RZV2H_SCI_TXI_IRQ(0),
  .tei_ipl = CONFIG_RZV2H_SCI_B_IRQ_PRIORITY,
  .tei_irq = RZV2H_SCI_TEI_IRQ(0),
  .eri_ipl = CONFIG_RZV2H_SCI_B_IRQ_PRIORITY,
  .eri_irq = RZV2H_SCI_ERI_IRQ(0),
  .p_callback = rzv2h_fsp_uart_cb,
  .p_context = &g_uart0priv,
};

static const uart_instance_t g_fsp_uart0_instance =
{
  .p_ctrl = &g_fsp_uart0_ctrl,
  .p_cfg = &g_fsp_uart0_cfg,
  .p_api = &g_uart_on_sci_b,
};

static struct up_dev_s g_uart0priv =
{
  .hal_inst = &g_fsp_uart0_instance,
  .configured_baud = CONFIG_SCI0_BAUD,
};

static struct uart_dev_s g_uart0port =
{
  .recv =
    {
      .size = CONFIG_SCI0_RXBUFSIZE,
      .buffer = g_uart0rxbuffer,
    },
  .xmit =
    {
      .size = CONFIG_SCI0_TXBUFSIZE,
      .buffer = g_uart0txbuffer,
    },
  .ops = &g_uart_ops,
  .priv = &g_uart0priv,
};
#endif

#ifdef CONFIG_RZV2H_SCI_B_UART1
static char g_uart1rxbuffer[CONFIG_SCI1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_SCI1_TXBUFSIZE];
static struct up_dev_s g_uart1priv;
static sci_b_uart_instance_ctrl_t g_fsp_uart1_ctrl;
static sci_b_baud_setting_t g_fsp_uart1_baud_setting;

static sci_b_uart_extended_cfg_t g_fsp_uart1_cfg_extend =
{
  .clock = SCI_B_UART_CLOCK_INT,
  .rx_edge_start = SCI_B_UART_START_BIT_FALLING_EDGE,
  .noise_cancel = SCI_B_UART_NOISE_CANCELLATION_ENABLE,
  .rx_fifo_trigger = SCI_B_UART_RX_FIFO_TRIGGER_MAX,
  .p_baud_setting = &g_fsp_uart1_baud_setting,
  .flow_control = SCI_B_UART_FLOW_CONTROL_RTS,
  .flow_control_pin = 0xffff,
  .rs485_setting =
    {
      .enable = SCI_B_UART_RS485_DISABLE,
      .polarity = SCI_B_UART_RS485_DE_POLARITY_HIGH,
      .assertion_time = 1,
      .negation_time = 1,
    },
  .half_data_setting =
    {
      .enable = 0,
    },
  .p_reg = (void *)R_SCI1,
};

static uart_cfg_t g_fsp_uart1_cfg =
{
  .channel = 1,
  .data_bits = CONFIG_SCI1_BITS == 7 ?
               UART_DATA_BITS_7 : UART_DATA_BITS_8,
  .parity = CONFIG_SCI1_PARITY == 1 ? UART_PARITY_ODD :
            CONFIG_SCI1_PARITY == 2 ? UART_PARITY_EVEN : UART_PARITY_OFF,
  .stop_bits = CONFIG_SCI1_2STOP ? UART_STOP_BITS_2 : UART_STOP_BITS_1,
  .p_extend = &g_fsp_uart1_cfg_extend,
  .p_transfer_tx = NULL,
  .p_transfer_rx = NULL,
  .rxi_ipl = CONFIG_RZV2H_SCI_B_IRQ_PRIORITY,
  .rxi_irq = RZV2H_SCI_RXI_IRQ(1),
  .txi_ipl = CONFIG_RZV2H_SCI_B_IRQ_PRIORITY,
  .txi_irq = RZV2H_SCI_TXI_IRQ(1),
  .tei_ipl = CONFIG_RZV2H_SCI_B_IRQ_PRIORITY,
  .tei_irq = RZV2H_SCI_TEI_IRQ(1),
  .eri_ipl = CONFIG_RZV2H_SCI_B_IRQ_PRIORITY,
  .eri_irq = RZV2H_SCI_ERI_IRQ(1),
  .p_callback = rzv2h_fsp_uart_cb,
  .p_context = &g_uart1priv,
};

static const uart_instance_t g_fsp_uart1_instance =
{
  .p_ctrl = &g_fsp_uart1_ctrl,
  .p_cfg = &g_fsp_uart1_cfg,
  .p_api = &g_uart_on_sci_b,
};

static struct up_dev_s g_uart1priv =
{
  .hal_inst = &g_fsp_uart1_instance,
  .configured_baud = CONFIG_SCI1_BAUD,
};

static struct uart_dev_s g_uart1port =
{
  .recv =
    {
      .size = CONFIG_SCI1_RXBUFSIZE,
      .buffer = g_uart1rxbuffer,
    },
  .xmit =
    {
      .size = CONFIG_SCI1_TXBUFSIZE,
      .buffer = g_uart1txbuffer,
    },
  .ops = &g_uart_ops,
  .priv = &g_uart1priv,
};
#endif

#ifdef CONFIG_RZV2H_SCI_B_UART2
static char g_uart2rxbuffer[CONFIG_SCI2_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_SCI2_TXBUFSIZE];
static struct up_dev_s g_uart2priv;
static sci_b_uart_instance_ctrl_t g_fsp_uart2_ctrl;
static sci_b_baud_setting_t g_fsp_uart2_baud_setting;

static sci_b_uart_extended_cfg_t g_fsp_uart2_cfg_extend =
{
  .clock = SCI_B_UART_CLOCK_INT,
  .rx_edge_start = SCI_B_UART_START_BIT_FALLING_EDGE,
  .noise_cancel = SCI_B_UART_NOISE_CANCELLATION_ENABLE,
  .rx_fifo_trigger = SCI_B_UART_RX_FIFO_TRIGGER_MAX,
  .p_baud_setting = &g_fsp_uart2_baud_setting,
  .flow_control = SCI_B_UART_FLOW_CONTROL_RTS,
  .flow_control_pin = 0xffff,
  .rs485_setting =
    {
      .enable = SCI_B_UART_RS485_DISABLE,
      .polarity = SCI_B_UART_RS485_DE_POLARITY_HIGH,
      .assertion_time = 1,
      .negation_time = 1,
    },
  .half_data_setting =
    {
      .enable = 0,
    },
  .p_reg = (void *)R_SCI2,
};

static uart_cfg_t g_fsp_uart2_cfg =
{
  .channel = 2,
  .data_bits = CONFIG_SCI2_BITS == 7 ?
               UART_DATA_BITS_7 : UART_DATA_BITS_8,
  .parity = CONFIG_SCI2_PARITY == 1 ? UART_PARITY_ODD :
            CONFIG_SCI2_PARITY == 2 ? UART_PARITY_EVEN : UART_PARITY_OFF,
  .stop_bits = CONFIG_SCI2_2STOP ? UART_STOP_BITS_2 : UART_STOP_BITS_1,
  .p_extend = &g_fsp_uart2_cfg_extend,
  .p_transfer_tx = NULL,
  .p_transfer_rx = NULL,
  .rxi_ipl = CONFIG_RZV2H_SCI_B_IRQ_PRIORITY,
  .rxi_irq = RZV2H_SCI_RXI_IRQ(2),
  .txi_ipl = CONFIG_RZV2H_SCI_B_IRQ_PRIORITY,
  .txi_irq = RZV2H_SCI_TXI_IRQ(2),
  .tei_ipl = CONFIG_RZV2H_SCI_B_IRQ_PRIORITY,
  .tei_irq = RZV2H_SCI_TEI_IRQ(2),
  .eri_ipl = CONFIG_RZV2H_SCI_B_IRQ_PRIORITY,
  .eri_irq = RZV2H_SCI_ERI_IRQ(2),
  .p_callback = rzv2h_fsp_uart_cb,
  .p_context = &g_uart2priv,
};

static const uart_instance_t g_fsp_uart2_instance =
{
  .p_ctrl = &g_fsp_uart2_ctrl,
  .p_cfg = &g_fsp_uart2_cfg,
  .p_api = &g_uart_on_sci_b,
};

static struct up_dev_s g_uart2priv =
{
  .hal_inst = &g_fsp_uart2_instance,
  .configured_baud = CONFIG_SCI2_BAUD,
};

static struct uart_dev_s g_uart2port =
{
  .recv =
    {
      .size = CONFIG_SCI2_RXBUFSIZE,
      .buffer = g_uart2rxbuffer,
    },
  .xmit =
    {
      .size = CONFIG_SCI2_TXBUFSIZE,
      .buffer = g_uart2txbuffer,
    },
  .ops = &g_uart_ops,
  .priv = &g_uart2priv,
};
#endif

#ifdef CONFIG_RZV2H_SCI_B_UART3
static char g_uart3rxbuffer[CONFIG_SCI3_RXBUFSIZE];
static char g_uart3txbuffer[CONFIG_SCI3_TXBUFSIZE];
static struct up_dev_s g_uart3priv;
static sci_b_uart_instance_ctrl_t g_fsp_uart3_ctrl;
static sci_b_baud_setting_t g_fsp_uart3_baud_setting;

static sci_b_uart_extended_cfg_t g_fsp_uart3_cfg_extend =
{
  .clock = SCI_B_UART_CLOCK_INT,
  .rx_edge_start = SCI_B_UART_START_BIT_FALLING_EDGE,
  .noise_cancel = SCI_B_UART_NOISE_CANCELLATION_ENABLE,
  .rx_fifo_trigger = SCI_B_UART_RX_FIFO_TRIGGER_MAX,
  .p_baud_setting = &g_fsp_uart3_baud_setting,
  .flow_control = SCI_B_UART_FLOW_CONTROL_RTS,
  .flow_control_pin = 0xffff,
  .rs485_setting =
    {
      .enable = SCI_B_UART_RS485_DISABLE,
      .polarity = SCI_B_UART_RS485_DE_POLARITY_HIGH,
      .assertion_time = 1,
      .negation_time = 1,
    },
  .half_data_setting =
    {
      .enable = 0,
    },
  .p_reg = (void *)R_SCI3,
};

static uart_cfg_t g_fsp_uart3_cfg =
{
  .channel = 3,
  .data_bits = CONFIG_SCI3_BITS == 7 ?
               UART_DATA_BITS_7 : UART_DATA_BITS_8,
  .parity = CONFIG_SCI3_PARITY == 1 ? UART_PARITY_ODD :
            CONFIG_SCI3_PARITY == 2 ? UART_PARITY_EVEN : UART_PARITY_OFF,
  .stop_bits = CONFIG_SCI3_2STOP ? UART_STOP_BITS_2 : UART_STOP_BITS_1,
  .p_extend = &g_fsp_uart3_cfg_extend,
  .p_transfer_tx = NULL,
  .p_transfer_rx = NULL,
  .rxi_ipl = CONFIG_RZV2H_SCI_B_IRQ_PRIORITY,
  .rxi_irq = RZV2H_SCI_RXI_IRQ(3),
  .txi_ipl = CONFIG_RZV2H_SCI_B_IRQ_PRIORITY,
  .txi_irq = RZV2H_SCI_TXI_IRQ(3),
  .tei_ipl = CONFIG_RZV2H_SCI_B_IRQ_PRIORITY,
  .tei_irq = RZV2H_SCI_TEI_IRQ(3),
  .eri_ipl = CONFIG_RZV2H_SCI_B_IRQ_PRIORITY,
  .eri_irq = RZV2H_SCI_ERI_IRQ(3),
  .p_callback = rzv2h_fsp_uart_cb,
  .p_context = &g_uart3priv,
};

static const uart_instance_t g_fsp_uart3_instance =
{
  .p_ctrl = &g_fsp_uart3_ctrl,
  .p_cfg = &g_fsp_uart3_cfg,
  .p_api = &g_uart_on_sci_b,
};

static struct up_dev_s g_uart3priv =
{
  .hal_inst = &g_fsp_uart3_instance,
  .configured_baud = CONFIG_SCI3_BAUD,
};

static struct uart_dev_s g_uart3port =
{
  .recv =
    {
      .size = CONFIG_SCI3_RXBUFSIZE,
      .buffer = g_uart3rxbuffer,
    },
  .xmit =
    {
      .size = CONFIG_SCI3_TXBUFSIZE,
      .buffer = g_uart3txbuffer,
    },
  .ops = &g_uart_ops,
  .priv = &g_uart3priv,
};
#endif

#ifdef CONFIG_RZV2H_SCI_B_UART4
static char g_uart4rxbuffer[CONFIG_SCI4_RXBUFSIZE];
static char g_uart4txbuffer[CONFIG_SCI4_TXBUFSIZE];
static struct up_dev_s g_uart4priv;
static sci_b_uart_instance_ctrl_t g_fsp_uart4_ctrl;
static sci_b_baud_setting_t g_fsp_uart4_baud_setting;

static sci_b_uart_extended_cfg_t g_fsp_uart4_cfg_extend =
{
  .clock = SCI_B_UART_CLOCK_INT,
  .rx_edge_start = SCI_B_UART_START_BIT_FALLING_EDGE,
  .noise_cancel = SCI_B_UART_NOISE_CANCELLATION_ENABLE,
  .rx_fifo_trigger = SCI_B_UART_RX_FIFO_TRIGGER_MAX,
  .p_baud_setting = &g_fsp_uart4_baud_setting,
  .flow_control = SCI_B_UART_FLOW_CONTROL_RTS,
  .flow_control_pin = 0xffff,
  .rs485_setting =
    {
      .enable = SCI_B_UART_RS485_DISABLE,
      .polarity = SCI_B_UART_RS485_DE_POLARITY_HIGH,
      .assertion_time = 1,
      .negation_time = 1,
    },
  .half_data_setting =
    {
      .enable = 0,
    },
  .p_reg = (void *)R_SCI4,
};

static uart_cfg_t g_fsp_uart4_cfg =
{
  .channel = 4,
  .data_bits = CONFIG_SCI4_BITS == 7 ?
               UART_DATA_BITS_7 : UART_DATA_BITS_8,
  .parity = CONFIG_SCI4_PARITY == 1 ? UART_PARITY_ODD :
            CONFIG_SCI4_PARITY == 2 ? UART_PARITY_EVEN : UART_PARITY_OFF,
  .stop_bits = CONFIG_SCI4_2STOP ? UART_STOP_BITS_2 : UART_STOP_BITS_1,
  .p_extend = &g_fsp_uart4_cfg_extend,
  .p_transfer_tx = NULL,
  .p_transfer_rx = NULL,
  .rxi_ipl = CONFIG_RZV2H_SCI_B_IRQ_PRIORITY,
  .rxi_irq = RZV2H_SCI_RXI_IRQ(4),
  .txi_ipl = CONFIG_RZV2H_SCI_B_IRQ_PRIORITY,
  .txi_irq = RZV2H_SCI_TXI_IRQ(4),
  .tei_ipl = CONFIG_RZV2H_SCI_B_IRQ_PRIORITY,
  .tei_irq = RZV2H_SCI_TEI_IRQ(4),
  .eri_ipl = CONFIG_RZV2H_SCI_B_IRQ_PRIORITY,
  .eri_irq = RZV2H_SCI_ERI_IRQ(4),
  .p_callback = rzv2h_fsp_uart_cb,
  .p_context = &g_uart4priv,
};

static const uart_instance_t g_fsp_uart4_instance =
{
  .p_ctrl = &g_fsp_uart4_ctrl,
  .p_cfg = &g_fsp_uart4_cfg,
  .p_api = &g_uart_on_sci_b,
};

static struct up_dev_s g_uart4priv =
{
  .hal_inst = &g_fsp_uart4_instance,
  .configured_baud = CONFIG_SCI4_BAUD,
};

static struct uart_dev_s g_uart4port =
{
  .recv =
    {
      .size = CONFIG_SCI4_RXBUFSIZE,
      .buffer = g_uart4rxbuffer,
    },
  .xmit =
    {
      .size = CONFIG_SCI4_TXBUFSIZE,
      .buffer = g_uart4txbuffer,
    },
  .ops = &g_uart_ops,
  .priv = &g_uart4priv,
};
#endif

#ifdef CONFIG_RZV2H_SCI_B_UART5
static char g_uart5rxbuffer[CONFIG_SCI5_RXBUFSIZE];
static char g_uart5txbuffer[CONFIG_SCI5_TXBUFSIZE];
static struct up_dev_s g_uart5priv;
static sci_b_uart_instance_ctrl_t g_fsp_uart5_ctrl;
static sci_b_baud_setting_t g_fsp_uart5_baud_setting;

static sci_b_uart_extended_cfg_t g_fsp_uart5_cfg_extend =
{
  .clock = SCI_B_UART_CLOCK_INT,
  .rx_edge_start = SCI_B_UART_START_BIT_FALLING_EDGE,
  .noise_cancel = SCI_B_UART_NOISE_CANCELLATION_ENABLE,
  .rx_fifo_trigger = SCI_B_UART_RX_FIFO_TRIGGER_MAX,
  .p_baud_setting = &g_fsp_uart5_baud_setting,
  .flow_control = SCI_B_UART_FLOW_CONTROL_RTS,
  .flow_control_pin = 0xffff,
  .rs485_setting =
    {
      .enable = SCI_B_UART_RS485_DISABLE,
      .polarity = SCI_B_UART_RS485_DE_POLARITY_HIGH,
      .assertion_time = 1,
      .negation_time = 1,
    },
  .half_data_setting =
    {
      .enable = 0,
    },
  .p_reg = (void *)R_SCI5,
};

static uart_cfg_t g_fsp_uart5_cfg =
{
  .channel = 5,
  .data_bits = CONFIG_SCI5_BITS == 7 ?
               UART_DATA_BITS_7 : UART_DATA_BITS_8,
  .parity = CONFIG_SCI5_PARITY == 1 ? UART_PARITY_ODD :
            CONFIG_SCI5_PARITY == 2 ? UART_PARITY_EVEN : UART_PARITY_OFF,
  .stop_bits = CONFIG_SCI5_2STOP ? UART_STOP_BITS_2 : UART_STOP_BITS_1,
  .p_extend = &g_fsp_uart5_cfg_extend,
  .p_transfer_tx = NULL,
  .p_transfer_rx = NULL,
  .rxi_ipl = CONFIG_RZV2H_SCI_B_IRQ_PRIORITY,
  .rxi_irq = RZV2H_SCI_RXI_IRQ(5),
  .txi_ipl = CONFIG_RZV2H_SCI_B_IRQ_PRIORITY,
  .txi_irq = RZV2H_SCI_TXI_IRQ(5),
  .tei_ipl = CONFIG_RZV2H_SCI_B_IRQ_PRIORITY,
  .tei_irq = RZV2H_SCI_TEI_IRQ(5),
  .eri_ipl = CONFIG_RZV2H_SCI_B_IRQ_PRIORITY,
  .eri_irq = RZV2H_SCI_ERI_IRQ(5),
  .p_callback = rzv2h_fsp_uart_cb,
  .p_context = &g_uart5priv,
};

static const uart_instance_t g_fsp_uart5_instance =
{
  .p_ctrl = &g_fsp_uart5_ctrl,
  .p_cfg = &g_fsp_uart5_cfg,
  .p_api = &g_uart_on_sci_b,
};

static struct up_dev_s g_uart5priv =
{
  .hal_inst = &g_fsp_uart5_instance,
  .configured_baud = CONFIG_SCI5_BAUD,
};

static struct uart_dev_s g_uart5port =
{
  .recv =
    {
      .size = CONFIG_SCI5_RXBUFSIZE,
      .buffer = g_uart5rxbuffer,
    },
  .xmit =
    {
      .size = CONFIG_SCI5_TXBUFSIZE,
      .buffer = g_uart5txbuffer,
    },
  .ops = &g_uart_ops,
  .priv = &g_uart5priv,
};
#endif

#ifdef CONFIG_RZV2H_SCI_B_UART6
static char g_uart6rxbuffer[CONFIG_SCI6_RXBUFSIZE];
static char g_uart6txbuffer[CONFIG_SCI6_TXBUFSIZE];
static struct up_dev_s g_uart6priv;
static sci_b_uart_instance_ctrl_t g_fsp_uart6_ctrl;
static sci_b_baud_setting_t g_fsp_uart6_baud_setting;

static sci_b_uart_extended_cfg_t g_fsp_uart6_cfg_extend =
{
  .clock = SCI_B_UART_CLOCK_INT,
  .rx_edge_start = SCI_B_UART_START_BIT_FALLING_EDGE,
  .noise_cancel = SCI_B_UART_NOISE_CANCELLATION_ENABLE,
  .rx_fifo_trigger = SCI_B_UART_RX_FIFO_TRIGGER_MAX,
  .p_baud_setting = &g_fsp_uart6_baud_setting,
  .flow_control = SCI_B_UART_FLOW_CONTROL_RTS,
  .flow_control_pin = 0xffff,
  .rs485_setting =
    {
      .enable = SCI_B_UART_RS485_DISABLE,
      .polarity = SCI_B_UART_RS485_DE_POLARITY_HIGH,
      .assertion_time = 1,
      .negation_time = 1,
    },
  .half_data_setting =
    {
      .enable = 0,
    },
  .p_reg = (void *)R_SCI6,
};

static uart_cfg_t g_fsp_uart6_cfg =
{
  .channel = 6,
  .data_bits = CONFIG_SCI6_BITS == 7 ?
               UART_DATA_BITS_7 : UART_DATA_BITS_8,
  .parity = CONFIG_SCI6_PARITY == 1 ? UART_PARITY_ODD :
            CONFIG_SCI6_PARITY == 2 ? UART_PARITY_EVEN : UART_PARITY_OFF,
  .stop_bits = CONFIG_SCI6_2STOP ? UART_STOP_BITS_2 : UART_STOP_BITS_1,
  .p_extend = &g_fsp_uart6_cfg_extend,
  .p_transfer_tx = NULL,
  .p_transfer_rx = NULL,
  .rxi_ipl = CONFIG_RZV2H_SCI_B_IRQ_PRIORITY,
  .rxi_irq = RZV2H_SCI_RXI_IRQ(6),
  .txi_ipl = CONFIG_RZV2H_SCI_B_IRQ_PRIORITY,
  .txi_irq = RZV2H_SCI_TXI_IRQ(6),
  .tei_ipl = CONFIG_RZV2H_SCI_B_IRQ_PRIORITY,
  .tei_irq = RZV2H_SCI_TEI_IRQ(6),
  .eri_ipl = CONFIG_RZV2H_SCI_B_IRQ_PRIORITY,
  .eri_irq = RZV2H_SCI_ERI_IRQ(6),
  .p_callback = rzv2h_fsp_uart_cb,
  .p_context = &g_uart6priv,
};

static const uart_instance_t g_fsp_uart6_instance =
{
  .p_ctrl = &g_fsp_uart6_ctrl,
  .p_cfg = &g_fsp_uart6_cfg,
  .p_api = &g_uart_on_sci_b,
};

static struct up_dev_s g_uart6priv =
{
  .hal_inst = &g_fsp_uart6_instance,
  .configured_baud = CONFIG_SCI6_BAUD,
};

static struct uart_dev_s g_uart6port =
{
  .recv =
    {
      .size = CONFIG_SCI6_RXBUFSIZE,
      .buffer = g_uart6rxbuffer,
    },
  .xmit =
    {
      .size = CONFIG_SCI6_TXBUFSIZE,
      .buffer = g_uart6txbuffer,
    },
  .ops = &g_uart_ops,
  .priv = &g_uart6priv,
};
#endif

#ifdef CONFIG_RZV2H_SCI_B_UART7
static char g_uart7rxbuffer[CONFIG_SCI7_RXBUFSIZE];
static char g_uart7txbuffer[CONFIG_SCI7_TXBUFSIZE];
static struct up_dev_s g_uart7priv;
static sci_b_uart_instance_ctrl_t g_fsp_uart7_ctrl;
static sci_b_baud_setting_t g_fsp_uart7_baud_setting;

static sci_b_uart_extended_cfg_t g_fsp_uart7_cfg_extend =
{
  .clock = SCI_B_UART_CLOCK_INT,
  .rx_edge_start = SCI_B_UART_START_BIT_FALLING_EDGE,
  .noise_cancel = SCI_B_UART_NOISE_CANCELLATION_ENABLE,
  .rx_fifo_trigger = SCI_B_UART_RX_FIFO_TRIGGER_MAX,
  .p_baud_setting = &g_fsp_uart7_baud_setting,
  .flow_control = SCI_B_UART_FLOW_CONTROL_RTS,
  .flow_control_pin = 0xffff,
  .rs485_setting =
    {
      .enable = SCI_B_UART_RS485_DISABLE,
      .polarity = SCI_B_UART_RS485_DE_POLARITY_HIGH,
      .assertion_time = 1,
      .negation_time = 1,
    },
  .half_data_setting =
    {
      .enable = 0,
    },
  .p_reg = (void *)R_SCI7,
};

static uart_cfg_t g_fsp_uart7_cfg =
{
  .channel = 7,
  .data_bits = CONFIG_SCI7_BITS == 7 ?
               UART_DATA_BITS_7 : UART_DATA_BITS_8,
  .parity = CONFIG_SCI7_PARITY == 1 ? UART_PARITY_ODD :
            CONFIG_SCI7_PARITY == 2 ? UART_PARITY_EVEN : UART_PARITY_OFF,
  .stop_bits = CONFIG_SCI7_2STOP ? UART_STOP_BITS_2 : UART_STOP_BITS_1,
  .p_extend = &g_fsp_uart7_cfg_extend,
  .p_transfer_tx = NULL,
  .p_transfer_rx = NULL,
  .rxi_ipl = CONFIG_RZV2H_SCI_B_IRQ_PRIORITY,
  .rxi_irq = RZV2H_SCI_RXI_IRQ(7),
  .txi_ipl = CONFIG_RZV2H_SCI_B_IRQ_PRIORITY,
  .txi_irq = RZV2H_SCI_TXI_IRQ(7),
  .tei_ipl = CONFIG_RZV2H_SCI_B_IRQ_PRIORITY,
  .tei_irq = RZV2H_SCI_TEI_IRQ(7),
  .eri_ipl = CONFIG_RZV2H_SCI_B_IRQ_PRIORITY,
  .eri_irq = RZV2H_SCI_ERI_IRQ(7),
  .p_callback = rzv2h_fsp_uart_cb,
  .p_context = &g_uart7priv,
};

static const uart_instance_t g_fsp_uart7_instance =
{
  .p_ctrl = &g_fsp_uart7_ctrl,
  .p_cfg = &g_fsp_uart7_cfg,
  .p_api = &g_uart_on_sci_b,
};

static struct up_dev_s g_uart7priv =
{
  .hal_inst = &g_fsp_uart7_instance,
  .configured_baud = CONFIG_SCI7_BAUD,
};

static struct uart_dev_s g_uart7port =
{
  .recv =
    {
      .size = CONFIG_SCI7_RXBUFSIZE,
      .buffer = g_uart7rxbuffer,
    },
  .xmit =
    {
      .size = CONFIG_SCI7_TXBUFSIZE,
      .buffer = g_uart7txbuffer,
    },
  .ops = &g_uart_ops,
  .priv = &g_uart7priv,
};
#endif

#ifdef CONFIG_RZV2H_SCI_B_UART8
static char g_uart8rxbuffer[CONFIG_SCI8_RXBUFSIZE];
static char g_uart8txbuffer[CONFIG_SCI8_TXBUFSIZE];
static struct up_dev_s g_uart8priv;
static sci_b_uart_instance_ctrl_t g_fsp_uart8_ctrl;
static sci_b_baud_setting_t g_fsp_uart8_baud_setting;

static sci_b_uart_extended_cfg_t g_fsp_uart8_cfg_extend =
{
  .clock = SCI_B_UART_CLOCK_INT,
  .rx_edge_start = SCI_B_UART_START_BIT_FALLING_EDGE,
  .noise_cancel = SCI_B_UART_NOISE_CANCELLATION_ENABLE,
  .rx_fifo_trigger = SCI_B_UART_RX_FIFO_TRIGGER_MAX,
  .p_baud_setting = &g_fsp_uart8_baud_setting,
  .flow_control = SCI_B_UART_FLOW_CONTROL_RTS,
  .flow_control_pin = 0xffff,
  .rs485_setting =
    {
      .enable = SCI_B_UART_RS485_DISABLE,
      .polarity = SCI_B_UART_RS485_DE_POLARITY_HIGH,
      .assertion_time = 1,
      .negation_time = 1,
    },
  .half_data_setting =
    {
      .enable = 0,
    },
  .p_reg = (void *)R_SCI8,
};

static uart_cfg_t g_fsp_uart8_cfg =
{
  .channel = 8,
  .data_bits = CONFIG_SCI8_BITS == 7 ?
               UART_DATA_BITS_7 : UART_DATA_BITS_8,
  .parity = CONFIG_SCI8_PARITY == 1 ? UART_PARITY_ODD :
            CONFIG_SCI8_PARITY == 2 ? UART_PARITY_EVEN : UART_PARITY_OFF,
  .stop_bits = CONFIG_SCI8_2STOP ? UART_STOP_BITS_2 : UART_STOP_BITS_1,
  .p_extend = &g_fsp_uart8_cfg_extend,
  .p_transfer_tx = NULL,
  .p_transfer_rx = NULL,
  .rxi_ipl = CONFIG_RZV2H_SCI_B_IRQ_PRIORITY,
  .rxi_irq = RZV2H_SCI_RXI_IRQ(8),
  .txi_ipl = CONFIG_RZV2H_SCI_B_IRQ_PRIORITY,
  .txi_irq = RZV2H_SCI_TXI_IRQ(8),
  .tei_ipl = CONFIG_RZV2H_SCI_B_IRQ_PRIORITY,
  .tei_irq = RZV2H_SCI_TEI_IRQ(8),
  .eri_ipl = CONFIG_RZV2H_SCI_B_IRQ_PRIORITY,
  .eri_irq = RZV2H_SCI_ERI_IRQ(8),
  .p_callback = rzv2h_fsp_uart_cb,
  .p_context = &g_uart8priv,
};

static const uart_instance_t g_fsp_uart8_instance =
{
  .p_ctrl = &g_fsp_uart8_ctrl,
  .p_cfg = &g_fsp_uart8_cfg,
  .p_api = &g_uart_on_sci_b,
};

static struct up_dev_s g_uart8priv =
{
  .hal_inst = &g_fsp_uart8_instance,
  .configured_baud = CONFIG_SCI8_BAUD,
};

static struct uart_dev_s g_uart8port =
{
  .recv =
    {
      .size = CONFIG_SCI8_RXBUFSIZE,
      .buffer = g_uart8rxbuffer,
    },
  .xmit =
    {
      .size = CONFIG_SCI8_TXBUFSIZE,
      .buffer = g_uart8txbuffer,
    },
  .ops = &g_uart_ops,
  .priv = &g_uart8priv,
};
#endif

#ifdef CONFIG_RZV2H_SCI_B_UART9
static char g_uart9rxbuffer[CONFIG_SCI9_RXBUFSIZE];
static char g_uart9txbuffer[CONFIG_SCI9_TXBUFSIZE];
static struct up_dev_s g_uart9priv;
static sci_b_uart_instance_ctrl_t g_fsp_uart9_ctrl;
static sci_b_baud_setting_t g_fsp_uart9_baud_setting;

static sci_b_uart_extended_cfg_t g_fsp_uart9_cfg_extend =
{
  .clock = SCI_B_UART_CLOCK_INT,
  .rx_edge_start = SCI_B_UART_START_BIT_FALLING_EDGE,
  .noise_cancel = SCI_B_UART_NOISE_CANCELLATION_ENABLE,
  .rx_fifo_trigger = SCI_B_UART_RX_FIFO_TRIGGER_MAX,
  .p_baud_setting = &g_fsp_uart9_baud_setting,
  .flow_control = SCI_B_UART_FLOW_CONTROL_RTS,
  .flow_control_pin = 0xffff,
  .rs485_setting =
    {
      .enable = SCI_B_UART_RS485_DISABLE,
      .polarity = SCI_B_UART_RS485_DE_POLARITY_HIGH,
      .assertion_time = 1,
      .negation_time = 1,
    },
  .half_data_setting =
    {
      .enable = 0,
    },
  .p_reg = (void *)R_SCI9,
};

static uart_cfg_t g_fsp_uart9_cfg =
{
  .channel = 9,
  .data_bits = CONFIG_SCI9_BITS == 7 ?
               UART_DATA_BITS_7 : UART_DATA_BITS_8,
  .parity = CONFIG_SCI9_PARITY == 1 ? UART_PARITY_ODD :
            CONFIG_SCI9_PARITY == 2 ? UART_PARITY_EVEN : UART_PARITY_OFF,
  .stop_bits = CONFIG_SCI9_2STOP ? UART_STOP_BITS_2 : UART_STOP_BITS_1,
  .p_extend = &g_fsp_uart9_cfg_extend,
  .p_transfer_tx = NULL,
  .p_transfer_rx = NULL,
  .rxi_ipl = CONFIG_RZV2H_SCI_B_IRQ_PRIORITY,
  .rxi_irq = RZV2H_SCI_RXI_IRQ(9),
  .txi_ipl = CONFIG_RZV2H_SCI_B_IRQ_PRIORITY,
  .txi_irq = RZV2H_SCI_TXI_IRQ(9),
  .tei_ipl = CONFIG_RZV2H_SCI_B_IRQ_PRIORITY,
  .tei_irq = RZV2H_SCI_TEI_IRQ(9),
  .eri_ipl = CONFIG_RZV2H_SCI_B_IRQ_PRIORITY,
  .eri_irq = RZV2H_SCI_ERI_IRQ(9),
  .p_callback = rzv2h_fsp_uart_cb,
  .p_context = &g_uart9priv,
};

static const uart_instance_t g_fsp_uart9_instance =
{
  .p_ctrl = &g_fsp_uart9_ctrl,
  .p_cfg = &g_fsp_uart9_cfg,
  .p_api = &g_uart_on_sci_b,
};

static struct up_dev_s g_uart9priv =
{
  .hal_inst = &g_fsp_uart9_instance,
  .configured_baud = CONFIG_SCI9_BAUD,
};

static struct uart_dev_s g_uart9port =
{
  .recv =
    {
      .size = CONFIG_SCI9_RXBUFSIZE,
      .buffer = g_uart9rxbuffer,
    },
  .xmit =
    {
      .size = CONFIG_SCI9_TXBUFSIZE,
      .buffer = g_uart9txbuffer,
    },
  .ops = &g_uart_ops,
  .priv = &g_uart9priv,
};
#endif

static struct uart_dev_s *const g_uart_devs[] =
{
#ifdef CONFIG_RZV2H_SCI_B_UART0
  &g_uart0port,
#endif
#ifdef CONFIG_RZV2H_SCI_B_UART1
  &g_uart1port,
#endif
#ifdef CONFIG_RZV2H_SCI_B_UART2
  &g_uart2port,
#endif
#ifdef CONFIG_RZV2H_SCI_B_UART3
  &g_uart3port,
#endif
#ifdef CONFIG_RZV2H_SCI_B_UART4
  &g_uart4port,
#endif
#ifdef CONFIG_RZV2H_SCI_B_UART5
  &g_uart5port,
#endif
#ifdef CONFIG_RZV2H_SCI_B_UART6
  &g_uart6port,
#endif
#ifdef CONFIG_RZV2H_SCI_B_UART7
  &g_uart7port,
#endif
#ifdef CONFIG_RZV2H_SCI_B_UART8
  &g_uart8port,
#endif
#ifdef CONFIG_RZV2H_SCI_B_UART9
  &g_uart9port,
#endif
};

#if defined(CONFIG_SCI0_SERIAL_CONSOLE) && \
      defined(CONFIG_RZV2H_SCI_B_UART0)
#  define CONSOLE_DEV (&g_uart0port)
#elif defined(CONFIG_SCI1_SERIAL_CONSOLE) && \
      defined(CONFIG_RZV2H_SCI_B_UART1)
#  define CONSOLE_DEV (&g_uart1port)
#elif defined(CONFIG_SCI2_SERIAL_CONSOLE) && \
      defined(CONFIG_RZV2H_SCI_B_UART2)
#  define CONSOLE_DEV (&g_uart2port)
#elif defined(CONFIG_SCI3_SERIAL_CONSOLE) && \
      defined(CONFIG_RZV2H_SCI_B_UART3)
#  define CONSOLE_DEV (&g_uart3port)
#elif defined(CONFIG_SCI4_SERIAL_CONSOLE) && \
      defined(CONFIG_RZV2H_SCI_B_UART4)
#  define CONSOLE_DEV (&g_uart4port)
#elif defined(CONFIG_SCI5_SERIAL_CONSOLE) && \
      defined(CONFIG_RZV2H_SCI_B_UART5)
#  define CONSOLE_DEV (&g_uart5port)
#elif defined(CONFIG_SCI6_SERIAL_CONSOLE) && \
      defined(CONFIG_RZV2H_SCI_B_UART6)
#  define CONSOLE_DEV (&g_uart6port)
#elif defined(CONFIG_SCI7_SERIAL_CONSOLE) && \
      defined(CONFIG_RZV2H_SCI_B_UART7)
#  define CONSOLE_DEV (&g_uart7port)
#elif defined(CONFIG_SCI8_SERIAL_CONSOLE) && \
      defined(CONFIG_RZV2H_SCI_B_UART8)
#  define CONSOLE_DEV (&g_uart8port)
#elif defined(CONFIG_SCI9_SERIAL_CONSOLE) && \
      defined(CONFIG_RZV2H_SCI_B_UART9)
#  define CONSOLE_DEV (&g_uart9port)
#endif

static const char *const g_tty_paths[RZV2H_NUARTS] =
{
  "/dev/ttyS0", "/dev/ttyS1", "/dev/ttyS2", "/dev/ttyS3",
  "/dev/ttyS4", "/dev/ttyS5", "/dev/ttyS6", "/dev/ttyS7",
  "/dev/ttyS8", "/dev/ttyS9"
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rzv2h_fsp_uart_baud_calculate
 *
 * Description:
 *   Calculate the FSP baud-rate settings for an SCI-B UART.
 *
 ****************************************************************************/

static int rzv2h_fsp_uart_baud_calculate(
  uint32_t baud, sci_b_baud_setting_t *setting)
{
  fsp_err_t err;

  err = R_SCI_B_UART_BaudCalculate(baud, false,
                                   RZV2H_UART_BAUD_ERROR, setting);
  return rzv2h_fsp_err_to_errno(err);
}

/****************************************************************************
 * Name: rzv2h_fsp_uart_open
 *
 * Description:
 *   Configure and open an SCI-B UART instance through the FSP API.
 *
 ****************************************************************************/

static int rzv2h_fsp_uart_open(struct up_dev_s *ctx)
{
  const uart_cfg_t *cfg;
  sci_b_uart_extended_cfg_t *extend;
  fsp_err_t err;

  if (ctx == NULL || ctx->hal_inst == NULL || ctx->dev == NULL)
    {
      return -EINVAL;
    }

  cfg = ctx->hal_inst->p_cfg;
  extend = (sci_b_uart_extended_cfg_t *)cfg->p_extend;

  if (rzv2h_fsp_uart_baud_calculate(ctx->configured_baud,
                                    extend->p_baud_setting) < 0)
    {
      return -EINVAL;
    }

  err = R_SCI_B_UART_Open(ctx->hal_inst->p_ctrl, cfg);
  if (FSP_SUCCESS != err)
    {
      return rzv2h_fsp_err_to_errno(err);
    }

  ctx->rx_enabled        = false;
  ctx->tx_enabled        = false;
  ctx->tx_idle           = true;
  ctx->last_error_flags  = 0;
  ctx->rx_pending        = false;
  ctx->opened            = true;

  /* FSP stores received data directly in rx_char. */

  err = R_SCI_B_UART_Read(ctx->hal_inst->p_ctrl, &ctx->rx_char, 1);
  if (FSP_SUCCESS != err)
    {
      (void)R_SCI_B_UART_Close(ctx->hal_inst->p_ctrl);
      ctx->opened = false;
      return rzv2h_fsp_err_to_errno(err);
    }

  return OK;
}

/****************************************************************************
 * Name: rzv2h_fsp_uart_close
 *
 * Description:
 *   Close an SCI-B UART instance through the FSP API.
 *
 ****************************************************************************/

static int rzv2h_fsp_uart_close(struct up_dev_s *ctx)
{
  fsp_err_t err;

  if (ctx == NULL || ctx->hal_inst == NULL)
    {
      return -EINVAL;
    }

  if (!ctx->opened)
    {
      return OK;
    }

  err = R_SCI_B_UART_Close(ctx->hal_inst->p_ctrl);
  if (FSP_SUCCESS == err)
    {
      ctx->opened = false;
    }

  return rzv2h_fsp_err_to_errno(err);
}

#ifdef CONFIG_SERIAL_TERMIOS
/****************************************************************************
 * Name: rzv2h_fsp_uart_set_baud
 *
 * Description:
 *   Change the baud rate of an open SCI-B UART instance.
 *
 ****************************************************************************/

static int rzv2h_fsp_uart_set_baud(struct up_dev_s *ctx, uint32_t baud)
{
  sci_b_baud_setting_t setting;
  irqstate_t flags;
  fsp_err_t err;
  int ret;

  if (ctx == NULL || ctx->hal_inst == NULL || !ctx->opened || baud == 0)
    {
      return -EINVAL;
    }

  ret = rzv2h_fsp_uart_baud_calculate(baud, &setting);
  if (ret < 0)
    {
      return ret;
    }

  flags = enter_critical_section();

  /* FSP baudSet() terminates an active transmission.  Refuse the change
   * instead of losing the byte whose storage is owned by this driver.
   */

  if (!ctx->tx_idle)
    {
      leave_critical_section(flags);
      return -EBUSY;
    }

  err = R_SCI_B_UART_BaudSet(ctx->hal_inst->p_ctrl, &setting);
  if (err == FSP_SUCCESS)
    {
      ctx->configured_baud = baud;
    }

  leave_critical_section(flags);
  return rzv2h_fsp_err_to_errno(err);
}
#endif

/****************************************************************************
 * Name: rzv2h_fsp_uart_rxint
 *
 * Description:
 *   Enable or disable receive interrupt delivery for an SCI-B UART.
 *
 ****************************************************************************/

static void rzv2h_fsp_uart_rxint(struct up_dev_s *ctx, bool is_enabled)
{
  int rxi_irq;

  if (ctx == NULL || ctx->hal_inst == NULL)
    {
      return;
    }

  ctx->rx_enabled = is_enabled;

  /* The FSP API cannot disable reception. Gate RXI at the GIC to avoid
   * changing registers behind the FSP driver's state.
   */

  rxi_irq = RZV2H_FSP_TO_GIC_IRQ(ctx->hal_inst->p_cfg->rxi_irq);
  if (is_enabled)
    {
      up_enable_irq(rxi_irq);
    }
  else
    {
      up_disable_irq(rxi_irq);
    }
}

/****************************************************************************
 * Name: rzv2h_fsp_uart_rx_rearm
 *
 * Description:
 *   Arm the FSP driver to receive the next byte.
 *
 ****************************************************************************/

static void rzv2h_fsp_uart_rx_rearm(struct up_dev_s *ctx)
{
  fsp_err_t err;

  if (ctx == NULL || !ctx->opened)
    {
      return;
    }

  err = R_SCI_B_UART_Read(ctx->hal_inst->p_ctrl, &ctx->rx_char, 1);
  if (FSP_SUCCESS != err && FSP_ERR_IN_USE != err)
    {
      ctx->last_error_flags |= RZV2H_FSP_STATUS_OVERRUN_ERR;
    }
}

/****************************************************************************
 * Name: rzv2h_fsp_uart_txint
 *
 * Description:
 *   Enable or disable transmit processing for an SCI-B UART.
 *
 ****************************************************************************/

static void rzv2h_fsp_uart_txint(struct up_dev_s *ctx, bool is_enabled)
{
  if (ctx == NULL || ctx->hal_inst == NULL || ctx->dev == NULL)
    {
      return;
    }

  /* Keep TXI enabled while an asynchronous FSP write is active.
   */

  ctx->tx_enabled = is_enabled;

  if (is_enabled)
    {
      uart_xmitchars(ctx->dev);
    }
}

/****************************************************************************
 * Name: rzv2h_fsp_uart_send_byte
 *
 * Description:
 *   Start an asynchronous one-byte transmission through the FSP API.
 *
 ****************************************************************************/

static int rzv2h_fsp_uart_send_byte(struct up_dev_s *ctx, uint8_t ch)
{
  fsp_err_t err;

  if (ctx == NULL || ctx->hal_inst == NULL)
    {
      return -EINVAL;
    }

  ctx->tx_char = ch;
  ctx->tx_idle = false;
  err = R_SCI_B_UART_Write(ctx->hal_inst->p_ctrl, &ctx->tx_char, 1);
  if (FSP_SUCCESS != err)
    {
      ctx->tx_idle = true;
      return -EAGAIN;
    }

  return 1;
}

/****************************************************************************
 * Name: rzv2h_uart_interrupt
 *
 * Description:
 *   Translate a NuttX GIC interrupt into an FSP interrupt context and call
 *   the corresponding SCI-B interrupt handler.
 *
 ****************************************************************************/

static int rzv2h_uart_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct up_dev_s *priv;
  const uart_cfg_t *cfg;
  IRQn_Type fsp_irq;
  void (*fsp_isr)(void);

  DEBUGASSERT(dev != NULL && dev->priv != NULL);

  priv = (struct up_dev_s *)dev->priv;
  cfg = priv->hal_inst->p_cfg;
  fsp_irq = (IRQn_Type)(irq - BSP_CORTEX_VECTOR_TABLE_ENTRIES);

  if (fsp_irq == cfg->rxi_irq)
    {
      fsp_isr = sci_b_uart_rxi_isr;
    }
  else if (fsp_irq == cfg->txi_irq)
    {
      fsp_isr = sci_b_uart_txi_isr;
    }
  else if (fsp_irq == cfg->tei_irq)
    {
      fsp_isr = sci_b_uart_tei_isr;
    }
  else if (fsp_irq == cfg->eri_irq)
    {
      fsp_isr = sci_b_uart_eri_isr;
    }
  else
    {
      return -EINVAL;
    }

  rzv2h_interrupt_common_handler((rzv2h_irqn_t)fsp_irq, fsp_isr);

  /* Continue the NuttX TX ring only after the FSP ISR has cleared
   * its pending status and restored its interrupt context.
   */

  if (fsp_irq == cfg->tei_irq && priv->tx_enabled && priv->tx_idle)
    {
      uart_xmitchars(dev);
    }

  return OK;
}

/****************************************************************************
 * Name: rzv2h_fsp_uart_cb
 *
 * Description:
 *   Forward FSP receive, transmit, and error events to the NuttX serial
 *   framework.
 *
 ****************************************************************************/

void rzv2h_fsp_uart_cb(uart_callback_args_t *args)
{
  struct up_dev_s *ctx;
  uart_event_t event;

  if (args == NULL || args->p_context == NULL)
    {
      return;
    }

  ctx = (struct up_dev_s *)args->p_context;
  event = args->event;

  /* The FSP ERI handler can report multiple error bits in one callback. */

  if ((event & UART_EVENT_ERR_OVERFLOW) != 0)
    {
      ctx->last_error_flags |= RZV2H_FSP_STATUS_OVERRUN_ERR;

#ifdef CONFIG_SERIAL_TIOCGICOUNT
      ctx->icount.overrun++;
#endif
    }

  if ((event & UART_EVENT_ERR_FRAMING) != 0)
    {
      ctx->last_error_flags |= RZV2H_FSP_STATUS_FRAMING_ERR;

#ifdef CONFIG_SERIAL_TIOCGICOUNT
      ctx->icount.frame++;
#endif
    }

  if ((event & UART_EVENT_ERR_PARITY) != 0)
    {
      ctx->last_error_flags |= RZV2H_FSP_STATUS_PARITY_ERR;

#ifdef CONFIG_SERIAL_TIOCGICOUNT
      ctx->icount.parity++;
#endif
    }

#ifdef CONFIG_SERIAL_TIOCGICOUNT
  if ((event & UART_EVENT_BREAK_DETECT) != 0)
    {
      ctx->icount.brk++;
    }
#endif

  if ((event & RZV2H_UART_ERROR_EVENTS) != 0)
    {
      return;
    }

  switch (event)
    {
      case UART_EVENT_RX_COMPLETE:
        {
          if (ctx->rx_enabled)
            {
              ctx->rx_pending = true;
              uart_recvchars(ctx->dev);
            }

          rzv2h_fsp_uart_rx_rearm(ctx);
          break;
        }

      case UART_EVENT_RX_CHAR:
        {
          /* Preserve a byte received between one-byte read operations. */

          ctx->rx_char = (uint8_t)args->data;
          if (ctx->rx_enabled)
            {
              ctx->rx_pending = true;
              uart_recvchars(ctx->dev);
            }

          rzv2h_fsp_uart_rx_rearm(ctx);
          break;
        }

      case UART_EVENT_TX_COMPLETE:
        {
          ctx->tx_idle = true;
          break;
        }

      case UART_EVENT_TX_DATA_EMPTY:
        break;

      default:
        break;
    }
}

/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the pins and initialize the SCI-B UART instance.
 *
 ****************************************************************************/

static int up_setup(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  const uart_cfg_t *cfg;
  int err;

  rzv2h_serial_setup();

  priv->dev = dev;
  up_shutdown(dev);

  err = rzv2h_fsp_uart_open(priv);
  if (err)
    {
      return err;
    }

  /* FSP enables the channel IRQs during open.  Leave them disabled until
   * up_attach() configures them for NuttX.
   */

  cfg = priv->hal_inst->p_cfg;
  up_disable_irq(RZV2H_FSP_TO_GIC_IRQ(cfg->eri_irq));
  up_disable_irq(RZV2H_FSP_TO_GIC_IRQ(cfg->rxi_irq));
  up_disable_irq(RZV2H_FSP_TO_GIC_IRQ(cfg->txi_irq));
  up_disable_irq(RZV2H_FSP_TO_GIC_IRQ(cfg->tei_irq));

  return OK;
}

/****************************************************************************
 * Name: up_shutdown
 *
 * Description:
 *   Disable the SCI.
 *
 ****************************************************************************/

static void up_shutdown(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  (void)rzv2h_fsp_uart_close(priv);
}

/****************************************************************************
 * Name: up_attach
 *
 * Description:
 *   Configure the SCI to operation in interrupt driven mode.  This method
 *   is called when the serial port is opened.  Normally, this is just after
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless
 *   the hardware supports multiple levels of interrupt enabling).  The RX
 *   and TX interrupts are not enabled until the txint() and rxint() methods
 *   are called.
 *
 ****************************************************************************/

static int up_attach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv;
  const uart_cfg_t *cfg;
  int eri_irq;
  int rxi_irq;
  int txi_irq;
  int tei_irq;
  int ret;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);

  priv = (struct up_dev_s *)dev->priv;
  cfg = priv->hal_inst->p_cfg;

  eri_irq = RZV2H_FSP_TO_GIC_IRQ(cfg->eri_irq);
  rxi_irq = RZV2H_FSP_TO_GIC_IRQ(cfg->rxi_irq);
  txi_irq = RZV2H_FSP_TO_GIC_IRQ(cfg->txi_irq);
  tei_irq = RZV2H_FSP_TO_GIC_IRQ(cfg->tei_irq);

  /* arm_gic0_initialize() resets every SPI to level-sensitive.  Restore
   * the trigger modes required by SCI-B after NuttX initializes the GIC:
   * ERI/TEI are level-sensitive and RXI/TXI are edge-sensitive.
   */

  ret = up_set_irq_type(eri_irq, IRQ_HIGH_LEVEL);
  if (ret < 0)
    {
      return ret;
    }

  ret = up_set_irq_type(rxi_irq, IRQ_RISING_EDGE);
  if (ret < 0)
    {
      return ret;
    }

  ret = up_set_irq_type(txi_irq, IRQ_RISING_EDGE);
  if (ret < 0)
    {
      return ret;
    }

  ret = up_set_irq_type(tei_irq, IRQ_HIGH_LEVEL);
  if (ret < 0)
    {
      return ret;
    }

  up_prioritize_irq(eri_irq, CONFIG_RZV2H_SCI_B_IRQ_PRIORITY << 4);
  up_prioritize_irq(rxi_irq, CONFIG_RZV2H_SCI_B_IRQ_PRIORITY << 4);
  up_prioritize_irq(txi_irq, CONFIG_RZV2H_SCI_B_IRQ_PRIORITY << 4);
  up_prioritize_irq(tei_irq, CONFIG_RZV2H_SCI_B_IRQ_PRIORITY << 4);

  ret = irq_attach(eri_irq, rzv2h_uart_interrupt, dev);
  if (ret < 0)
    {
      return ret;
    }

  ret = irq_attach(rxi_irq, rzv2h_uart_interrupt, dev);
  if (ret < 0)
    {
      irq_detach(eri_irq);
      return ret;
    }

  ret = irq_attach(txi_irq, rzv2h_uart_interrupt, dev);
  if (ret < 0)
    {
      irq_detach(rxi_irq);
      irq_detach(eri_irq);
      return ret;
    }

  ret = irq_attach(tei_irq, rzv2h_uart_interrupt, dev);
  if (ret < 0)
    {
      irq_detach(txi_irq);
      irq_detach(rxi_irq);
      irq_detach(eri_irq);
      return ret;
    }

  up_enable_irq(eri_irq);
  up_enable_irq(rxi_irq);
  up_enable_irq(txi_irq);
  up_enable_irq(tei_irq);

  return OK;
}

/****************************************************************************
 * Name: up_detach
 *
 * Description:
 *   Disable and detach all interrupts for an SCI-B UART instance.
 *
 ****************************************************************************/

static void up_detach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  const uart_cfg_t *cfg = priv->hal_inst->p_cfg;
  int eri_irq = RZV2H_FSP_TO_GIC_IRQ(cfg->eri_irq);
  int rxi_irq = RZV2H_FSP_TO_GIC_IRQ(cfg->rxi_irq);
  int txi_irq = RZV2H_FSP_TO_GIC_IRQ(cfg->txi_irq);
  int tei_irq = RZV2H_FSP_TO_GIC_IRQ(cfg->tei_irq);

  rzv2h_fsp_uart_rxint(priv, false);
  rzv2h_fsp_uart_txint(priv, false);

  up_disable_irq(eri_irq);
  up_disable_irq(rxi_irq);
  up_disable_irq(txi_irq);
  up_disable_irq(tei_irq);

  irq_detach(eri_irq);
  irq_detach(rxi_irq);
  irq_detach(txi_irq);
  irq_detach(tei_irq);
}

/****************************************************************************
 * Name: up_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int up_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#if defined(CONFIG_SERIAL_TERMIOS) || \
    defined(CONFIG_SERIAL_TIOCSERGSTRUCT) || \
    defined(CONFIG_SERIAL_TIOCGICOUNT)
  struct inode *inode = filep->f_inode;
  struct uart_dev_s *dev = inode->i_private;
  struct up_dev_s *priv = dev->priv;
#endif
  int ret = -ENOTTY;

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TIOCSERGSTRUCT
      case TIOCSERGSTRUCT:
        {
          struct up_dev_s *user =
            (struct up_dev_s *)(uintptr_t)arg;

          if (user == NULL)
            {
              ret = -EINVAL;
            }
          else
            {
              memcpy(user, priv, sizeof(*user));
              ret = OK;
            }
        }
        break;
#endif

#ifdef CONFIG_SERIAL_TIOCGICOUNT
      case TIOCGICOUNT:
        {
          struct serial_icounter_s *user =
            (struct serial_icounter_s *)(uintptr_t)arg;
          struct serial_icounter_s snapshot;
          irqstate_t flags;

          if (user == NULL)
            {
              ret = -EINVAL;
              break;
            }

          flags = enter_critical_section();
          memcpy(&snapshot, &priv->icount, sizeof(snapshot));
          leave_critical_section(flags);

          memcpy(user, &snapshot, sizeof(*user));
          ret = OK;
        }
        break;
#endif

#ifdef CONFIG_SERIAL_TERMIOS
      case TCGETS:
        {
          struct termios *termiosp =
            (struct termios *)(uintptr_t)arg;
          const uart_cfg_t *cfg = priv->hal_inst->p_cfg;

          if (termiosp == NULL)
            {
              ret = -EINVAL;
              break;
            }

          termiosp->c_cflag =
            cfg->data_bits == UART_DATA_BITS_7 ? CS7 : CS8;

          if (cfg->parity != UART_PARITY_OFF)
            {
              termiosp->c_cflag |= PARENB;
              if (cfg->parity == UART_PARITY_ODD)
                {
                  termiosp->c_cflag |= PARODD;
                }
            }

          if (cfg->stop_bits == UART_STOP_BITS_2)
            {
              termiosp->c_cflag |= CSTOPB;
            }

          ret = cfsetspeed(termiosp, priv->configured_baud) == 0 ?
                OK : -EINVAL;
        }
        break;

      case TCSETS:
        {
          const struct termios *termiosp =
            (const struct termios *)(uintptr_t)arg;
          const uart_cfg_t *cfg = priv->hal_inst->p_cfg;
          uart_data_bits_t data_bits;
          uart_parity_t parity;
          uart_stop_bits_t stop_bits;
          speed_t baud;

          if (termiosp == NULL)
            {
              ret = -EINVAL;
              break;
            }

          switch (termiosp->c_cflag & CSIZE)
            {
              case CS7:
                data_bits = UART_DATA_BITS_7;
                break;

              case CS8:
                data_bits = UART_DATA_BITS_8;
                break;

              default:
                ret = -EINVAL;
                goto termios_out;
            }

          if ((termiosp->c_cflag & PARENB) == 0)
            {
              parity = UART_PARITY_OFF;
            }
          else if ((termiosp->c_cflag & PARODD) != 0)
            {
              parity = UART_PARITY_ODD;
            }
          else
            {
              parity = UART_PARITY_EVEN;
            }

          stop_bits = (termiosp->c_cflag & CSTOPB) != 0 ?
                      UART_STOP_BITS_2 : UART_STOP_BITS_1;

          /* FSP baudSet() changes only CCR2.  Reject frame-format changes
           * until the driver has a safe close/reopen reconfiguration path.
           */

          if (data_bits != cfg->data_bits || parity != cfg->parity ||
              stop_bits != cfg->stop_bits)
            {
              ret = -EINVAL;
              goto termios_out;
            }

#ifdef CCTS_OFLOW
          if ((termiosp->c_cflag & CCTS_OFLOW) != 0)
            {
              ret = -EINVAL;
              goto termios_out;
            }
#endif

#ifdef CRTS_IFLOW
          if ((termiosp->c_cflag & CRTS_IFLOW) != 0)
            {
              ret = -EINVAL;
              goto termios_out;
            }
#endif

          baud = cfgetspeed(termiosp);
          if (baud == 0)
            {
              ret = -EINVAL;
              goto termios_out;
            }

          if ((uint32_t)baud == priv->configured_baud)
            {
              ret = OK;
            }
          else
            {
              ret = rzv2h_fsp_uart_set_baud(priv, (uint32_t)baud);
            }

termios_out:
          ;
        }
        break;
#endif

      default:
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: up_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the SCI.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int up_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  int ch = priv->rx_char;

#ifdef CONFIG_SERIAL_TIOCGICOUNT
  sbuf_size_t nexthead = dev->recv.head + 1 < dev->recv.size ?
                         dev->recv.head + 1 : 0;

  /* uart_recvchars() still consumes a hardware byte when its software
   * receive ring is full.  Count that discarded byte separately from an
   * SCI-B hardware overrun.
   */

  if (nexthead == dev->recv.tail)
    {
      priv->icount.buf_overrun++;
    }
#endif

  priv->rx_pending = false;

  if (status)
    {
      *status = priv->last_error_flags;
      priv->last_error_flags = 0;
    }

  return ch;
}

/****************************************************************************
 * Name: up_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void up_rxint(struct uart_dev_s *dev, bool is_enabled)
{
  struct up_dev_s *priv = dev->priv;

  rzv2h_fsp_uart_rxint(priv, is_enabled);
}

/****************************************************************************
 * Name: up_rxavailable
 *
 * Description:
 *   Return true if the receive holding register is not empty
 *
 ****************************************************************************/

static bool up_rxavailable(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  return priv->rx_pending;
}

/****************************************************************************
 * Name: up_send
 *
 * Description:
 *   This method will send one byte on the SCI
 *
 ****************************************************************************/

static void up_send(struct uart_dev_s *dev, int ch)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  (void)rzv2h_fsp_uart_send_byte(priv, (uint8_t)ch);
}

/****************************************************************************
 * Name: up_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void up_txint(struct uart_dev_s *dev, bool is_enabled)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  rzv2h_fsp_uart_txint(priv, is_enabled);
}

/****************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Return true when no asynchronous transmission is active.
 *
 ****************************************************************************/

static bool up_txready(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  return priv->tx_idle;
}

/****************************************************************************
 * Name: up_txempty
 *
 * Description:
 *   Return true when no asynchronous transmission is active.
 *
 ****************************************************************************/

static bool up_txempty(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  return priv->tx_idle;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rzv2h_serial_setup
 *
 * Description:
 *   Configure the board-selected pins for each enabled SCI-B UART channel.
 *
 ****************************************************************************/

void rzv2h_serial_setup(void)
{
#ifdef CONFIG_RZV2H_SCI_B_UART0
  rzv2h_configgpio(BOARD_SCI0_TXD_GPIO);
  rzv2h_configgpio(BOARD_SCI0_RXD_GPIO);
#endif

#ifdef CONFIG_RZV2H_SCI_B_UART1
  rzv2h_configgpio(BOARD_SCI1_TXD_GPIO);
  rzv2h_configgpio(BOARD_SCI1_RXD_GPIO);
#endif

#ifdef CONFIG_RZV2H_SCI_B_UART2
  rzv2h_configgpio(BOARD_SCI2_TXD_GPIO);
  rzv2h_configgpio(BOARD_SCI2_RXD_GPIO);
#endif

#ifdef CONFIG_RZV2H_SCI_B_UART3
  rzv2h_configgpio(BOARD_SCI3_TXD_GPIO);
  rzv2h_configgpio(BOARD_SCI3_RXD_GPIO);
#endif

#ifdef CONFIG_RZV2H_SCI_B_UART4
  rzv2h_configgpio(BOARD_SCI4_TXD_GPIO);
  rzv2h_configgpio(BOARD_SCI4_RXD_GPIO);
#endif

#ifdef CONFIG_RZV2H_SCI_B_UART5
  rzv2h_configgpio(BOARD_SCI5_TXD_GPIO);
  rzv2h_configgpio(BOARD_SCI5_RXD_GPIO);
#endif

#ifdef CONFIG_RZV2H_SCI_B_UART6
  rzv2h_configgpio(BOARD_SCI6_TXD_GPIO);
  rzv2h_configgpio(BOARD_SCI6_RXD_GPIO);
#endif

#ifdef CONFIG_RZV2H_SCI_B_UART7
  rzv2h_configgpio(BOARD_SCI7_TXD_GPIO);
  rzv2h_configgpio(BOARD_SCI7_RXD_GPIO);
#endif

#ifdef CONFIG_RZV2H_SCI_B_UART8
  rzv2h_configgpio(BOARD_SCI8_TXD_GPIO);
  rzv2h_configgpio(BOARD_SCI8_RXD_GPIO);
#endif

#ifdef CONFIG_RZV2H_SCI_B_UART9
  rzv2h_configgpio(BOARD_SCI9_TXD_GPIO);
  rzv2h_configgpio(BOARD_SCI9_RXD_GPIO);
#endif
}

/****************************************************************************
 * Name: rzv2h_earlyserialinit
 *
 * Description:
 *   Initialize the selected SCI-B serial console during early boot.
 *
 ****************************************************************************/

void rzv2h_earlyserialinit(void)
{
#ifdef CONSOLE_DEV
  CONSOLE_DEV->isconsole = true;

  (void)up_setup(CONSOLE_DEV);
#endif
}

/****************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   Register the enabled SCI-B UART devices with the serial framework.
 *
 ****************************************************************************/

void arm_serialinit(void)
{
  unsigned int tty = 0;
  unsigned int i;

  /*  The selected console is /dev/console and /dev/ttyS0.
   *  Remaining enabled SCI channels are assigned in ascending
   *  hardware-channel order.
   */

#ifdef CONSOLE_DEV
  uart_register("/dev/console", CONSOLE_DEV);
  uart_register(g_tty_paths[tty++], CONSOLE_DEV);
#endif

  for (i = 0; i < sizeof(g_uart_devs) / sizeof(g_uart_devs[0]); i++)
    {
#ifdef CONSOLE_DEV
      if (g_uart_devs[i] == CONSOLE_DEV)
        {
          continue;
        }
#endif

      DEBUGASSERT(tty < RZV2H_NUARTS);
      uart_register(g_tty_paths[tty++], g_uart_devs[i]);
    }
}
