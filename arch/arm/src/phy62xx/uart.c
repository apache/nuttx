/****************************************************************************
 * arch/arm/src/phy62xx/uart.c
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

/****************************************************************************
 *   @file     uart.c
 *   @brief    Contains all functions support for uart driver
 *   @version  0.0
 *   @date     19. Oct. 2017
 *   @author   qing.han
 *
 ****************************************************************************/

#include "rom_sym_def.h"
#include <string.h>
#include "bus_dev.h"
#include "mcu.h"
#include "gpio.h"
#include "clock.h"
#include "uart.h"
#include "pwrmgr.h"
#include "error.h"
#include "jump_function.h"

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/serial.h>
#include <nuttx/mm/circbuf.h>

#define UART_TX_BUFFER_SIZE   64
#define UART_RX_BUFFER_SIZE   64
uint8_t fifo_data_store[2][UART_RX_FIFO_SIZE];
uint8_t fifo_len_store[2] =
{
  0, 0
};

typedef struct _uart_Context
{
  bool          enable;
  bool          rx_available; /* rx byte available */

  UART_INDEX_e  ID;
  AP_UART_TypeDef *reg;

  uint8_t irq;

  uint8_t       tx_state;
  uart_Tx_Buf_t tx_buf;
  uart_Cfg_t    cfg;
  uint8_t fifo_buf_store[UART_RX_BUFFER_SIZE];
  int buf_head;
  int buf_tail;
} uart_Ctx_t;

static uart_Ctx_t m_uartCtx[2] =
{
  {
    .ID = UART0,
    .reg = (AP_UART_TypeDef *) AP_UART0_BASE,
    .irq = PHY62XX_IRQ_UART0_IRQn,
    .enable = FALSE,
  },
  {
    .ID = UART1,
    .reg = (AP_UART_TypeDef *) AP_UART1_BASE,
    .irq = PHY62XX_IRQ_UART1_IRQn,
    .enable = FALSE,
  },
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

static int txmit_buf_use_tx_buf(UART_INDEX_e uart_index, uint8_t *buf,
    uint16_t len)
{
  uart_Tx_Buf_t *p_txbuf = &(m_uartCtx[uart_index].tx_buf);
  uint8_t *p_data;
  AP_UART_TypeDef *cur_uart = (AP_UART_TypeDef *) AP_UART0_BASE;

  if (len == 0 || buf == NULL)
      return PPlus_ERR_INVALID_PARAM;

  if (p_txbuf->tx_state == TX_STATE_UNINIT)
      return PPlus_ERR_NO_MEM;

  if (p_txbuf->tx_buf_size < len)
      return PPlus_ERR_NO_MEM;

  if (p_txbuf->tx_state != TX_STATE_IDLE)
    {
      if (p_txbuf->tx_data_size + len > p_txbuf->tx_buf_size)
          return PPlus_ERR_NO_MEM;

      _HAL_CS_ALLOC_();
      HAL_ENTER_CRITICAL_SECTION();
      memcpy(p_txbuf->tx_buf + p_txbuf->tx_data_size, buf, len);
      p_txbuf->tx_data_size += len;
      HAL_EXIT_CRITICAL_SECTION();
      return PPlus_SUCCESS;
    }

  memcpy(p_txbuf->tx_buf, buf, len);
  p_txbuf->tx_data_size = len;
  p_txbuf->tx_data_offset = 0;
  p_txbuf->tx_state = TX_STATE_TX;
  p_data = p_txbuf->tx_buf;

  /* len = p_txbuf->tx_data_size; */

  len = len > UART_TX_FIFO_SIZE ? UART_TX_FIFO_SIZE : len;

  if (uart_index == UART1)
      cur_uart = (AP_UART_TypeDef *) AP_UART1_BASE;

  cur_uart->IER &= ~(IER_ETBEI);

  while (len--)
    {
      cur_uart->THR = p_data[p_txbuf->tx_data_offset++];
    }

  if (uart_index == UART0)
      hal_pwrmgr_lock(MOD_UART0);
  else
      hal_pwrmgr_lock(MOD_UART1);

  cur_uart->IER |= IER_ETBEI;
  return PPlus_SUCCESS;
}

static int txmit_buf_polling(UART_INDEX_e uart_index,
    uint8_t *buf, uint16_t len)
{
  /* volatile int timeout = 0; */

  AP_UART_TypeDef *cur_uart = (AP_UART_TypeDef *) AP_UART0_BASE;

  if (uart_index == UART1)
      cur_uart = (AP_UART_TypeDef *) AP_UART1_BASE;

  HAL_WAIT_CONDITION_TIMEOUT(!(cur_uart->USR & USR_BUSY), 100000);

  while (len--)
    {
      HAL_WAIT_CONDITION_TIMEOUT((cur_uart->LSR & LSR_THRE), 100000);
      cur_uart->THR = *buf++;

      /* timeout = 0; */
    }

  /* wait shift register empty */

  HAL_WAIT_CONDITION_TIMEOUT((cur_uart->LSR & LSR_TEMT), 100000);
  return PPlus_SUCCESS;
}

static void irq_rx_handler(UART_INDEX_e uart_index, uint8_t flg)
{
  int i;
  uint8_t data[UART_RX_FIFO_SIZE];
  uint8_t len;
  AP_UART_TypeDef *cur_uart = (AP_UART_TypeDef *)AP_UART0_BASE;

  if (uart_index == UART1)
    {
      cur_uart = (AP_UART_TypeDef *) AP_UART1_BASE;
    }

  if (m_uartCtx[uart_index].cfg.use_fifo)
    {
      len = cur_uart->RFL;
      fifo_len_store[uart_index] = len;
      for (i = 0; i < len; i++)
        {
          data[i] = (uint8_t)(cur_uart->RBR & 0xff);
          fifo_data_store[uart_index][i] = data[i];
        }
    }
  else
    {
      len = 1;
      cur_uart->LSR;  /* clear interrupt */
      data[0] = (uint8_t)(cur_uart->RBR & 0xff);
    }

  if (m_uartCtx[uart_index].cfg.evt_handler)
    {
      uart_Evt_t evt;
      evt.type = flg;
      evt.data = data;
      evt.len = len;
      m_uartCtx[uart_index].cfg.evt_handler(&evt);
    }
}

static void irq_tx_empty_handler(UART_INDEX_e uart_index)
{
  uart_Tx_Buf_t *p_txbuf = &(m_uartCtx[uart_index].tx_buf);
  uint8_t *p_data;
  uint16_t len;
  AP_UART_TypeDef *cur_uart = (AP_UART_TypeDef *)AP_UART0_BASE;

  if (m_uartCtx[uart_index].enable == FALSE)
      return;

  if (m_uartCtx[uart_index].cfg.use_fifo == FALSE)
      return;

  if (m_uartCtx[uart_index].cfg.use_tx_buf == FALSE)
      return;

  if (p_txbuf->tx_state != TX_STATE_TX)
      return;

  p_data = p_txbuf->tx_buf;
  len = p_txbuf->tx_data_size - p_txbuf->tx_data_offset;
  len = len > UART_TX_FIFO_SIZE ? UART_TX_FIFO_SIZE : len;

  if (len == 0)
    {
      p_txbuf->tx_state = TX_STATE_IDLE;
      p_txbuf->tx_data_offset = 0;
      p_txbuf->tx_data_size = 0;

      if (m_uartCtx[uart_index].cfg.evt_handler)
        {
          uart_Evt_t evt =
            {
              .type = UART_EVT_TYPE_TX_COMPLETED,
              .data = NULL,
              .len = 0,
            };

          m_uartCtx[uart_index].cfg.evt_handler(&evt);
        }

      if (UART0 == uart_index)
          hal_pwrmgr_unlock(MOD_UART0);
      else
          hal_pwrmgr_unlock(MOD_UART1);

      return;
    }

  if (uart_index == UART1)
      cur_uart = (AP_UART_TypeDef *) AP_UART1_BASE;

  while (len--)
    {
      cur_uart->THR = p_data[p_txbuf->tx_data_offset++];
    }
}

static int uart_hw_deinit(UART_INDEX_e uart_index)
{
  MODULE_e mod = MOD_UART0;
  IRQn_Type irq_type = PHY62XX_IRQ_UART0_IRQn;
  AP_UART_TypeDef *cur_uart = AP_UART0;

  if (uart_index == UART1)
    {
      mod = MOD_UART1;
      irq_type = PHY62XX_IRQ_UART1_IRQn;
      cur_uart = AP_UART1;
    }

  NVIC_DisableIRQ(irq_type);
  hal_gpio_fmux(m_uartCtx[uart_index].cfg.tx_pin, Bit_DISABLE);
  hal_gpio_fmux(m_uartCtx[uart_index].cfg.rx_pin, Bit_DISABLE);
  cur_uart->LCR = 0x80;
  cur_uart->DLM = 0;
  cur_uart->DLL = 0;
  cur_uart->LCR = 0;
  cur_uart->FCR = 0;
  cur_uart->IER = 0;

  /* hal_clk_gate_enable(mod); */

  hal_clk_reset(mod);
  hal_clk_gate_disable(mod);

  return PPlus_SUCCESS;
}

int hal_uart_rxint_en(UART_INDEX_e uart_index, bool en)
{
  AP_UART_TypeDef *cur_uart = AP_UART0;
  if (uart_index == UART1)
    {
      cur_uart = AP_UART1;
    }

  if (en)
    {
      cur_uart->IER |= IER_ERBFI;
    }
  else
    {
      cur_uart->IER &= ~IER_ERBFI;
    }

  return 0;
}

int hal_uart_txint_en(UART_INDEX_e uart_index, bool en)
{
  AP_UART_TypeDef *cur_uart = AP_UART0;
  if (uart_index == UART1)
    {
      cur_uart = AP_UART1;
    }

  if (en)
    {
      cur_uart->IER |= IER_ETBEI;
    }
  else
    {
      cur_uart->IER &= ~IER_ETBEI;
    }

  return 0;
}

extern uint32_t timer_sysclk_get_clk(void);

int uart_hw_init(UART_INDEX_e uart_index)
{
  uart_Cfg_t *pcfg;
  int pclk = timer_sysclk_get_clk();
  uint32_t dll;
  AP_UART_TypeDef *cur_uart = AP_UART0;
  MODULE_e mod = MOD_UART0;
  IRQn_Type irq_type = PHY62XX_IRQ_UART0_IRQn;
  gpio_fmux_e fmux_tx = FMUX_UART0_TX;
  gpio_fmux_e fmux_rx = FMUX_UART0_RX;
  uart_hw_deinit(uart_index);

  if (uart_index == UART1)
    {
      cur_uart = AP_UART1;
      mod = MOD_UART1;
      irq_type = PHY62XX_IRQ_UART1_IRQn;
      fmux_tx = FMUX_UART1_TX;
      fmux_rx = FMUX_UART1_RX;
    }

  if ((m_uartCtx[uart_index].cfg.tx_pin == GPIO_DUMMY) &&
      (m_uartCtx[uart_index].cfg.rx_pin == GPIO_DUMMY))
    {
      return PPlus_ERR_INVALID_PARAM;
    }

  pcfg = &(m_uartCtx[uart_index].cfg);
  hal_clk_gate_enable(mod);
  hal_clk_reset(mod);

  /*  if (m_uartCtx[uart_index].enable == FALSE){
   *      hal_gpio_fmux(P9, Bit_DISABLE);
   *      hal_gpio_fmux(P10, Bit_DISABLE);
   *  }
   */

  hal_gpio_pull_set(pcfg->tx_pin, GPIO_PULL_UP);
  hal_gpio_pull_set(pcfg->rx_pin, GPIO_PULL_UP);
  hal_gpio_fmux_set(pcfg->tx_pin, fmux_tx);
  hal_gpio_fmux_set(pcfg->rx_pin, fmux_rx);
  cur_uart->LCR = 0;
  dll = ((pclk >> 4) + (pcfg->baudrate >> 1)) / pcfg->baudrate;
  cur_uart->MCR = 0x0;
  cur_uart->LCR = 0x80;
  cur_uart->DLM = (dll & 0xff00) >> 8;
  cur_uart->DLL = (dll & 0xff);

  if (pcfg->parity)
      cur_uart->LCR = 0x1b; /* 8bit, 1 stop even parity */
  else
      cur_uart->LCR = 0x3;  /* 8bit, 1 stop no parity */

  if (pcfg->use_fifo)       /* set fifo, enable tx FIFO mode(empty trigger), rx FIFO mode(1/2 trigger) */
    {
      cur_uart->FCR = FCR_TX_FIFO_RESET | FCR_RX_FIFO_RESET | FCR_FIFO_ENABLE
          | UART_FIFO_RX_TRIGGER | UART_FIFO_TX_TRIGGER;
    }
  else
    {
      cur_uart->FCR = 0;
    }

  /* enable Received Data Available Interrupt */

  cur_uart->IER = IER_ERBFI;

  if (pcfg->use_fifo)
    {
      cur_uart->IER |= IER_PTIME;
    }

  if (pcfg->use_tx_buf)
      cur_uart->IER |= IER_ETBEI;

  NVIC_SetPriority(irq_type, IRQ_PRIO_HAL);
  NVIC_EnableIRQ(irq_type);
  return PPlus_SUCCESS;
}

/****************************************************************************
 *   @fn          hal_UART0_IRQHandler
 *
 *   @brief       This function process for uart interrupt
 *
 *   input parameters
 *
 *   @param       None.
 *
 *   output parameters
 *
 *   @param       None.
 *
 *   @return      None.
 ****************************************************************************/

void __ATTR_SECTION_SRAM__  hal_UART0_IRQHandler(void)
{
  uint8_t IRQ_ID = (AP_UART0->IIR & 0x0f);

  /* if (m_uartCtx[UART0].enable == FALSE)
   *    return;
   */

  switch (IRQ_ID)
    {
      case TIMEOUT_IRQ:
          irq_rx_handler(UART0, UART_EVT_TYPE_RX_DATA_TO);
          break;

      case RDA_IRQ:
          irq_rx_handler(UART0, UART_EVT_TYPE_RX_DATA);
          break;

      case THR_EMPTY:
          irq_tx_empty_handler(UART0);
          break;

      case RLS_IRQ:
          break;

      case BUSY_IRQ:
          (void)AP_UART0->USR;
          break;

      default:
          break;
    }
}

void __attribute__((used)) hal_UART1_IRQHandler(void)
{
  uint8_t IRQ_ID = (AP_UART1->IIR & 0x0f);

  /* if(m_uartCtx[UART1].enable == FALSE)
   *   return;
   */

  switch (IRQ_ID)
    {
      case TIMEOUT_IRQ:
          irq_rx_handler(UART1, UART_EVT_TYPE_RX_DATA_TO);
          break;

      case RDA_IRQ:
          irq_rx_handler(UART1, UART_EVT_TYPE_RX_DATA);
          break;

      case THR_EMPTY:
          irq_tx_empty_handler(UART1);
          break;

      case RLS_IRQ:
          break;

      case BUSY_IRQ:
          (void)AP_UART1->USR;
          break;

      default:
          break;
    }
}

static void uart_wakeup_process_0(void)
{
  uart_hw_init(UART0);
}

static void uart_wakeup_process_1(void)
{
  uart_hw_init(UART1);
}

int hal_uart_init(uart_Cfg_t cfg, UART_INDEX_e uart_index)
{
  if (m_uartCtx[uart_index].enable)
      return PPlus_ERR_BUSY;

  /* if(cfg.hw_fwctrl || cfg.parity)
   * return PPlus_ERR_NOT_SUPPORTED;
   */

  if (cfg.hw_fwctrl)
      return PPlus_ERR_NOT_SUPPORTED;

  /* memset(&(m_uartCtx[uart_index]), 0, sizeof(uart_Ctx_t)); */

  memcpy(&(m_uartCtx[uart_index].cfg), &cfg, sizeof(uart_Cfg_t));
  uart_hw_init(uart_index);
  m_uartCtx[uart_index].enable = TRUE;

  if (uart_index == UART0)
      hal_pwrmgr_register(MOD_UART0, NULL, uart_wakeup_process_0);
  else
      hal_pwrmgr_register(MOD_UART1, NULL, uart_wakeup_process_1);

  return PPlus_SUCCESS;
}

int hal_uart_deinit(UART_INDEX_e uart_index)
{
  uart_hw_deinit(uart_index);
  m_uartCtx[uart_index].enable = FALSE;

  if (uart_index == UART0)
      hal_pwrmgr_unregister(MOD_UART0);
  else
      hal_pwrmgr_unregister(MOD_UART1);

  return PPlus_SUCCESS;
}

int hal_uart_set_tx_buf(UART_INDEX_e uart_index, uint8_t *buf, uint16_t size)
{
  uart_Tx_Buf_t *p_txbuf = &(m_uartCtx[uart_index].tx_buf);

  if (m_uartCtx[uart_index].enable == FALSE)
      return PPlus_ERR_INVALID_STATE;

  if (m_uartCtx[uart_index].cfg.use_tx_buf == FALSE)
      return PPlus_ERR_NOT_SUPPORTED;

  if (p_txbuf->tx_state != TX_STATE_UNINIT)
      return PPlus_ERR_INVALID_STATE;

  _HAL_CS_ALLOC_();
  HAL_ENTER_CRITICAL_SECTION();
  p_txbuf->tx_buf = buf;
  p_txbuf->tx_buf_size = size;
  p_txbuf->tx_data_offset = 0;
  p_txbuf->tx_data_size = 0;
  p_txbuf->tx_state = TX_STATE_IDLE;
  HAL_EXIT_CRITICAL_SECTION();
  return PPlus_SUCCESS;
}

int hal_uart_get_tx_ready(UART_INDEX_e uart_index)
{
  if (m_uartCtx[uart_index].cfg.use_tx_buf == FALSE)
      return PPlus_SUCCESS;

  if (m_uartCtx[uart_index].tx_buf.tx_state == TX_STATE_IDLE)
      return PPlus_SUCCESS;

  return PPlus_ERR_BUSY;
}

int hal_uart_send_buff(UART_INDEX_e uart_index, uint8_t *buff, uint16_t len)
{
  if (m_uartCtx[uart_index].cfg.use_tx_buf)
    {
      return txmit_buf_use_tx_buf(uart_index, buff, len);
    }

  return txmit_buf_polling(uart_index, buff, len);
}

int hal_uart_send_byte(UART_INDEX_e uart_index, unsigned char data)
{
  AP_UART_TypeDef *cur_uart = (AP_UART_TypeDef *) AP_UART0_BASE;

  if (uart_index == UART1)
      cur_uart = (AP_UART_TypeDef *) AP_UART1_BASE;

  HAL_WAIT_CONDITION_TIMEOUT((cur_uart->LSR & LSR_THRE), 10000);
  cur_uart->THR = data;
  HAL_WAIT_CONDITION_TIMEOUT((cur_uart->LSR & LSR_TEMT), 10000);
  return PPlus_SUCCESS;
}

static int pplus_uart_interrupt(int irq, void *context, void *arg);

static int pplus_uart_setup(struct uart_dev_s *dev)
{
  uart_Ctx_t *priv = (uart_Ctx_t *)dev->priv;
  uart_Cfg_t cfg =
    {
      .tx_pin = P9,
      .rx_pin = P10,
      .rts_pin = GPIO_DUMMY,
      .cts_pin = GPIO_DUMMY,
      .baudrate = 115200,
      .use_fifo = TRUE,
      .hw_fwctrl = FALSE,
      .use_tx_buf = FALSE,
      .parity     = FALSE,
      .evt_handler = NULL,
    };

  hal_uart_init(cfg, priv->ID);

  /* TODO: configure UART if not selected as console */

  return OK;
}

/****************************************************************************
 * Name: pplus_uart_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void pplus_uart_shutdown(struct uart_dev_s *dev)
{
  uart_Ctx_t *priv = (uart_Ctx_t *)dev->priv;

  /* Disable interrupts */

  /* Reset hardware and disable Rx and Tx */

  hal_uart_deinit(priv->ID);
}

/****************************************************************************
 * Name: pplus_uart_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.  This method
 *   is called when the serial port is opened.  Normally, this is just after
 *   the the setup() method is called, however, the serial console may
 *   operate in a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless
 *   the hardware supports multiple levels of interrupt enabling).
 *   The RX and TX interrupts are not enabled until the txint() and rxint()
 *   methods are called.
 *
 ****************************************************************************/

static int pplus_uart_attach(struct uart_dev_s *dev)
{
  uart_Ctx_t *priv = (uart_Ctx_t *)dev->priv;
  int ret;

  /* Attach and enable the IRQ(s).  The interrupts are (probably) still
   * disabled in the C2 register.
   */

  ret = irq_attach(priv->irq, pplus_uart_interrupt, dev);
  if (ret == OK)
    {
      up_enable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: pplus_uart_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.
 *   The exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void pplus_uart_detach(struct uart_dev_s *dev)
{
  uart_Ctx_t *priv = (uart_Ctx_t *)dev->priv;

  /* Disable interrupts */

  priv->reg->IER = 0;
  up_disable_irq(priv->irq);

  /* Detach from the interrupt(s) */

  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: pplus_uart_interrupt
 *
 * Description:
 *   This is the UART status interrupt handler.  It will be invoked when an
 *   interrupt received on the 'irq'  It should call uart_transmitchars or
 *   uart_receivechar to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'irq' number into the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int pplus_uart_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  uart_Ctx_t *priv = (uart_Ctx_t *)(dev->priv);

  uint8_t IRQ_ID = (priv->reg->IIR & 0x0f);

  switch (IRQ_ID)
    {
      case TIMEOUT_IRQ:
      case RDA_IRQ:
          priv->rx_available = true;
          priv->reg->LSR;  /* clear interrupt */
          uart_recvchars(dev);
          break;

      case THR_EMPTY:
      case RLS_IRQ:
          break;

      case BUSY_IRQ:
         (void)priv->reg->USR;
         break;

      default:
         break;
  }

  return OK;
}

static int pplus_uart_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#ifdef CONFIG_SERIAL_TERMIOS
  struct inode *inode = filep->f_inode;
  struct uart_dev_s *dev = inode->i_private;
  uart_Ctx_t *priv = (uart_Ctx_t *)dev->priv;
  struct uart_config_s *config = &priv->config;
#endif
  int ret = OK;

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TERMIOS
      case TCGETS:
        {
          struct termios *termiosp = (struct termios *)arg;

          if (!termiosp)
            {
              ret = -EINVAL;
              break;
            }

          termiosp->c_cflag = ((config->parity != 0) ? PARENB : 0)
                              | ((config->parity == 1) ? PARODD : 0)
                              | ((config->stopbits2) ? CSTOPB : 0) |
#ifdef CONFIG_SERIAL_OFLOWCONTROL
                              ((config->oflow) ? CCTS_OFLOW : 0) |
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
                              ((config->iflow) ? CRTS_IFLOW : 0) |
#endif
                              CS8;

          cfsetispeed(termiosp, config->baud);

          break;
        }

      case TCSETS:
        {
          struct termios *termiosp = (struct termios *)arg;

          if (!termiosp)
            {
              ret = -EINVAL;
              break;
            }

          /* Perform some sanity checks before accepting any changes */

          if ((termiosp->c_cflag & CSIZE) != CS8)
            {
              ret = -EINVAL;
              break;
            }

#ifndef HAVE_UART_STOPBITS
          if ((termiosp->c_cflag & CSTOPB) != 0)
            {
              ret = -EINVAL;
              break;
            }
#endif

          if (termiosp->c_cflag & PARODD)
            {
              ret = -EINVAL;
              break;
            }

          /* TODO: CCTS_OFLOW and CRTS_IFLOW */

          /* Parity */

          if (termiosp->c_cflag & PARENB)
            {
              config->parity = (termiosp->c_cflag & PARODD) ? 1 : 2;
            }
          else
            {
              config->parity = 0;
            }

#ifdef HAVE_UART_STOPBITS
          /* Stop bits */

          config->stopbits2 = (termiosp->c_cflag & CSTOPB) != 0;
#endif

          /* Note that only cfgetispeed is used because we have knowledge
           * that only one speed is supported.
           */

          config->baud = cfgetispeed(termiosp);

          /* Effect the changes */

          pplus_uart_set_format(dev);

          break;
        }
#endif

      default:
        {
          ret = -ENOTTY;
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: pplus_uart_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int pplus_uart_receive(struct uart_dev_s *dev, unsigned int *status)
{
  uart_Ctx_t *priv = (uart_Ctx_t *)dev->priv;

  /* uint32_t data;
   * static uint8_t fifo_buf_store[UART_RX_BUFFER_SIZE];
   * static int buf_head;
   * static int buf_tail;
   */

  /* Put fifo data into loopback buffer */

  for (int i = 0; i < fifo_len_store[priv->ID] ; i++)
    {
      priv->fifo_buf_store[priv->buf_head] = fifo_data_store[priv->ID][i];
      priv->buf_head = priv->buf_head + 1;
      if (priv->buf_head == UART_RX_BUFFER_SIZE)
        {
          priv->buf_head = 0;
        }
    }

  fifo_len_store[priv->ID] = 0;
  if (priv->buf_tail == UART_RX_BUFFER_SIZE)
    {
      priv->buf_tail = 0;
    }

  priv->rx_available = false;

  /* Return receiver control information */

  if (status)
    {
      *status = 0x00;
    }

  /* Then return the fifo data byte by byte. */

  return (char)priv->fifo_buf_store[priv->buf_tail++];
}

/****************************************************************************
 * Name: pplus_uart_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void pplus_uart_rxint(struct uart_dev_s *dev, bool enable)
{
  uart_Ctx_t *priv = (uart_Ctx_t *)dev->priv;

  hal_uart_rxint_en(priv->ID, enable);
}

/****************************************************************************
 * Name: pplus_uart_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

static bool pplus_uart_rxavailable(struct uart_dev_s *dev)
{
  static int len_fifo;
  uart_Ctx_t *priv = (uart_Ctx_t *)dev->priv;

  /* Detect the length of received data from fifo */

  if (len_fifo == 0)
    {
      hal_UART0_IRQHandler();
      len_fifo = fifo_len_store[priv->ID];
    }

  if (len_fifo <= 0)
    {
      len_fifo = 0;
      return len_fifo;
    }
  else
    {
      return len_fifo--;
    }
}

/****************************************************************************
 * Name: pplus_uart_send
 *
 * Description:
 *   This method will send one byte on the UART.
 *
 ****************************************************************************/

static void pplus_uart_send(struct uart_dev_s *dev, int ch)
{
  uart_Ctx_t *priv = (uart_Ctx_t *)dev->priv;

  hal_uart_send_byte(priv->ID, (uint8_t)ch);
}

/****************************************************************************
 * Name: pplus_uart_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void pplus_uart_txint(struct uart_dev_s *dev, bool enable)
{
  /* uart_Ctx_t *priv = (uart_Ctx_t *)dev->priv; */

  if (enable)
    {
      irqstate_t flags = enter_critical_section();
      uart_xmitchars(dev);
      leave_critical_section(flags);
    }
}

/****************************************************************************
 * Name: pplus_uart_txready
 *
 * Description:
 *   Return true if the tranmsit data register is empty
 *
 ****************************************************************************/

static bool pplus_uart_txready(struct uart_dev_s *dev)
{
  /* uart_Ctx_t *priv = (uart_Ctx_t *)dev->priv; */

  /* Return true if the transmit FIFO is "not full." */

  return true;
}

/****************************************************************************
 * Name: pplus_uart_txempty
 *
 * Description:
 *   Return true if the transmit data register is empty
 *
 ****************************************************************************/

static bool pplus_uart_txempty(struct uart_dev_s *dev)
{
  /* uart_Ctx_t *priv = (uart_Ctx_t *)dev->priv; */

  /* Return true if the transmit FIFO is "empty." */

  return true;
}
#define CONFIG_UART0_RXBUFSIZE 256
#define CONFIG_UART0_TXBUFSIZE 256

static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];

static const struct uart_ops_s g_pplus_uart_ops =
{
  .setup          = pplus_uart_setup,
  .shutdown       = pplus_uart_shutdown,
  .attach         = pplus_uart_attach,
  .detach         = pplus_uart_detach,
  .ioctl          = pplus_uart_ioctl,
  .receive        = pplus_uart_receive,
  .rxint          = pplus_uart_rxint,
  .rxavailable    = pplus_uart_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = pplus_uart_send,
  .txint          = pplus_uart_txint,
  .txready        = pplus_uart_txready,
  .txempty        = pplus_uart_txempty,
};

static uart_dev_t g_uart0port =
{
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

  .ops      = &g_pplus_uart_ops,
  .priv     = &m_uartCtx[0],
};

void arm_earlyserialinit(void)
{
}

/****************************************************************************
 * Name: stm32serial_getregit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that arm_earlyserialinit was called previously.
 *
 ****************************************************************************/

#  define CONSOLE_DEV         g_uart0port /* UART0 is console */
#  define TTYS0_DEV           g_uart0port /* UART0 is ttyS0 */

void arm_serialinit(void)
{
  /* #ifdef HAVE_UART_CONSOLE */

  /* Register the serial console */

  uart_register("/dev/console", &CONSOLE_DEV);

  /* #endif */

  uart_register("/dev/ttyS0", &TTYS0_DEV);
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
  hal_uart_send_byte(UART0, (char)ch);
  return ch;
}

struct h4uart_param_s
{
  struct circbuf_s *pcirc_h2c;
  sem_t            *psem_h2c ;
  struct circbuf_s *pcirc_c2h;
  sem_t            *psem_c2h ;
};

void h4uart_rx_irq(void *arg)
{
  AP_UART_TypeDef *preg = AP_UART1;
  uint8_t buf[UART_RX_FIFO_SIZE];
  int i;
  int len = preg->RFL;
  struct h4uart_param_s *param = (struct h4uart_param_s *)arg;
  if (len)
    {
      for (i = 0; i < len; i++)
        {
          buf[i] = (uint8_t)(preg->RBR & 0xff);
        }

      circbuf_write(param->pcirc_h2c, buf, len);
      nxsem_post(param->psem_h2c);
    }
}

void h4uart_tx_irq(void *arg)
{
  AP_UART_TypeDef *preg = AP_UART1;
  uint8_t buf[UART_TX_FIFO_SIZE];
  int i;
  int len;
  struct h4uart_param_s *param = (struct h4uart_param_s *)arg;

  len = circbuf_read(param->pcirc_c2h, buf, UART_TX_FIFO_SIZE);
  if (circbuf_used(param->pcirc_c2h) == 0)
      hal_uart_txint_en(UART1, false);

  if (len)
    {
      for (i = 0; i < len; i++)
        {
          while (preg->TFL >= UART_TX_FIFO_SIZE)
            {
            }

          preg->THR = buf[i];
        }
    }
}

static int h4uart_interrupt(int irq, void *context, void *arg)
{
  AP_UART_TypeDef *preg = AP_UART1;

  uint8_t IRQ_ID = preg->IIR & 0x0f;

  switch (IRQ_ID)
    {
      case TIMEOUT_IRQ:
      case RDA_IRQ:
          h4uart_rx_irq(arg);
          break;
      case THR_EMPTY:
          h4uart_tx_irq(arg);
          break;
      case RLS_IRQ:
      case BUSY_IRQ:
          preg->USR;
          break;
      default:
          break;
    }

  return 0;
}

int h4uart_init(void *param)
{
  uart_Cfg_t pcfg1 =
    {
      .tx_pin = P32,
      .rx_pin = P31,
      .rts_pin = GPIO_DUMMY,
      .cts_pin = GPIO_DUMMY,
      .baudrate = 115200,
      .use_fifo = TRUE,
      .hw_fwctrl = FALSE,
      .use_tx_buf = FALSE,
      .parity     = FALSE,
      .evt_handler = NULL,
    };

  irq_attach(PHY62XX_IRQ_UART1_IRQn, h4uart_interrupt, param);
  hal_uart_init(pcfg1, UART1);
  hal_uart_txint_en(UART1, true);
  hal_uart_rxint_en(UART1, true);
  return 0;
}
