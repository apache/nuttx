/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_lowputc.h
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

#ifndef __ARCH_RISCV_SRC_ESP32C3_ESP32C3_LOWPUTC_H
#define __ARCH_RISCV_SRC_ESP32C3_ESP32C3_LOWPUTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include "chip.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum uart_sclk
{
  APB_CLK = 1, /* 80 MHz */
  CLK_8,       /* 8 MHz */
  XTAL_CLK
};

enum uart_parity
{
  UART_PARITY_DISABLE,
  UART_PARITY_ODD,
  UART_PARITY_EVEN
};

enum uart_data_length
{
  UART_DATA_5_BITS,
  UART_DATA_6_BITS,
  UART_DATA_7_BITS,
  UART_DATA_8_BITS
};

enum uart_stop_length
{
    UART_STOP_BITS_1   = 0x1,  /* Stop bit: 1 bit */
    UART_STOP_BITS_2   = 0x3,  /* Stop bit: 2 bits */
};

/* Default FIFOs size */

#define UART_TX_FIFO_SIZE 128
#define UART_RX_FIFO_SIZE 128

/* Maximum serial clock divisor for integer part */

#define MAX_UART_CLKDIV (BIT(12) - 1)
#define DIV_UP(a, b)    (((a) + (b) - 1) / (b))

/* Struct used to store uart driver information and to
 * manipulate uart driver
 */

struct esp32c3_uart_s
{
  uint8_t   periph;         /* UART peripheral ID */
  int       cpuint;         /* CPU interrupt assigned to this UART */
  uint8_t   id;             /* UART ID */
  uint8_t   irq;            /* IRQ associated with this UART */
  uint32_t  baud;           /* Configured baud rate */
  uint8_t   bits;           /* Data length (5 to 8 bits). */
  uint8_t   parity;         /* 0=no parity, 1=odd parity, 2=even parity */
  uint8_t   stop_b2;        /* Use 2 stop bits? 0 = no (use 1) 1 = yes (use 2) */
  uint8_t   int_pri;        /* UART Interrupt Priority */
  uint8_t   txpin;          /* TX pin */
  uint8_t   txsig;          /* TX signal */
  uint8_t   rxpin;          /* RX pin */
  uint8_t   rxsig;          /* RX signal */
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  uint8_t  rtspin;          /* RTS pin number */
  uint8_t  rtssig;          /* RTS signal */
  bool     iflow;           /* Input flow control (RTS) enabled */
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  uint8_t  ctspin;          /* CTS pin number */
  uint8_t  ctssig;          /* CTS signal */
  bool     oflow;           /* Output flow control (CTS) enabled */
#endif
};

extern struct esp32c3_uart_s g_uart0_config;
extern struct esp32c3_uart_s g_uart1_config;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_lowputc_set_iflow
 *
 * Description:
 *   Configure the input hardware flow control.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *   threshold      - RX FIFO value from which RST will automatically be
 *                    asserted.
 *   enable         - true = enable, false = disable
 *
 ****************************************************************************/

void esp32c3_lowputc_set_iflow(const struct esp32c3_uart_s *priv,
                               uint8_t threshold, bool enable);

/****************************************************************************
 * Name: esp32c3_lowputc_set_oflow
 *
 * Description:
 *   Configure the output hardware flow control.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *   enable         - true = enable, false = disable
 *
 ****************************************************************************/

void esp32c3_lowputc_set_oflow(const struct esp32c3_uart_s *priv,
                               bool enable);

/****************************************************************************
 * Name: esp32c3_lowputc_reset_core
 *
 * Description:
 *   Reset both TX and RX cores.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp32c3_lowputc_reset_cores(const struct esp32c3_uart_s *priv);

/****************************************************************************
 * Name: esp32c3_lowputc_rst_tx
 *
 * Description:
 *   Reset TX core.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp32c3_lowputc_rst_tx(const struct esp32c3_uart_s *priv);

/****************************************************************************
 * Name: esp32c3_lowputc_rst_rx
 *
 * Description:
 *   Reset RX core.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp32c3_lowputc_rst_rx(const struct esp32c3_uart_s *priv);

/****************************************************************************
 * Name: esp32c3_lowputc_enable_sclk
 *
 * Description:
 *   Enable clock for whole core.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp32c3_lowputc_enable_sclk(const struct esp32c3_uart_s *priv);

/****************************************************************************
 * Name: esp32c3_lowputc_disable_sclk
 *
 * Description:
 *   Disable clock for whole core.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp32c3_lowputc_disable_sclk(const struct esp32c3_uart_s *priv);

/****************************************************************************
 * Name: esp32c3_lowputc_set_sclk
 *
 * Description:
 *   Set a source clock for UART.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *   source         - APB_CLK  = 1  80 MHz
 *                    CLK_8    = 2  8 MHz
 *                    XTAL_CLK = 3
 *
 ****************************************************************************/

void esp32c3_lowputc_set_sclk(const struct esp32c3_uart_s *priv,
                              enum uart_sclk source);

/****************************************************************************
 * Name: esp32c3_lowputc_get_sclk
 *
 * Description:
 *   Get the source clock for UART.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 * Returned Value:
 *   The frequency of the clock in Hz.
 *
 ****************************************************************************/

uint32_t esp32c3_lowputc_get_sclk(const struct esp32c3_uart_s *priv);

/****************************************************************************
 * Name: esp32c3_lowputc_baud
 *
 * Description:
 *   Set the baud rate according to the value in the private driver
 *   struct.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp32c3_lowputc_baud(const struct esp32c3_uart_s *priv);

/****************************************************************************
 * Name: esp32c3_lowputc_normal_mode
 *
 * Description:
 *   Set the UART to operate in normal mode, i.e., disable the RS485 mode and
 *   IRDA mode.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp32c3_lowputc_normal_mode(const struct esp32c3_uart_s *priv);

/****************************************************************************
 * Name: esp32c3_lowputc_parity
 *
 * Description:
 *   Set the parity, according to the value in the private driver
 *   struct.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp32c3_lowputc_parity(const struct esp32c3_uart_s *priv);

/****************************************************************************
 * Name: esp32c3_lowputc_data_length
 *
 * Description:
 *   Set the data bits length, according to the value in the private driver
 *   struct.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 ****************************************************************************/

int esp32c3_lowputc_data_length(const struct esp32c3_uart_s *priv);

/****************************************************************************
 * Name: esp32c3_lowputc_stop_length
 *
 * Description:
 *   Set the stop bits length, according to the value in the private driver
 *   struct.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp32c3_lowputc_stop_length(const struct esp32c3_uart_s *priv);

/****************************************************************************
 * Name: esp32c3_lowputc_set_tx_idle_time
 *
 * Description:
 *   Set the idle time between transfers.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *   time           - Desired time interval between the transfers.
 *
 ****************************************************************************/

void esp32c3_lowputc_set_tx_idle_time(const struct esp32c3_uart_s *priv,
                                      uint32_t time);

/****************************************************************************
 * Name: esp32c3_lowputc_send_byte
 *
 * Description:
 *   Send one byte.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *   byte           - Byte to be sent.
 *
 ****************************************************************************/

void esp32c3_lowputc_send_byte(const struct esp32c3_uart_s *priv,
                               char byte);

/****************************************************************************
 * Name: esp32c3_lowputc_is_tx_fifo_full
 *
 * Description:
 *   Verify if TX FIFO is full.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 * Returned Value:
 *   True if it is full, otherwise false.
 *
 ****************************************************************************/

bool esp32c3_lowputc_is_tx_fifo_full(const struct esp32c3_uart_s *priv);

/****************************************************************************
 * Name: esp32c3_lowputc_rst_peripheral
 *
 * Description:
 *   Reset the UART peripheral by using System reg.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp32c3_lowputc_rst_peripheral(const struct esp32c3_uart_s *priv);

/****************************************************************************
 * Name: esp32c3_lowputc_rst_txfifo
 *
 * Description:
 *   Reset TX FIFO.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp32c3_lowputc_rst_txfifo(const struct esp32c3_uart_s *priv);

/****************************************************************************
 * Name: esp32c3_lowputc_rst_rxfifo
 *
 * Description:
 *   Reset RX FIFO.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp32c3_lowputc_rst_rxfifo(const struct esp32c3_uart_s *priv);

/****************************************************************************
 * Name: esp32c3_lowputc_enable_sysclk
 *
 * Description:
 *   Enable clock for the UART using the System register.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp32c3_lowputc_enable_sysclk(const struct esp32c3_uart_s *priv);

/****************************************************************************
 * Name: esp32c3_lowputc_disable_all_uart_int
 *
 * Description:
 *   Disable all UART interrupts.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *   current_status - Pointer to a variable to store the current status of
 *                    the interrupt enable register before disabling
 *                    UART interrupts.
 *
 ****************************************************************************/

void esp32c3_lowputc_disable_all_uart_int(const struct esp32c3_uart_s *priv,
                                          uint32_t *current_status);

/****************************************************************************
 * Name: esp32c3_lowputc_restore_all_uart_int
 *
 * Description:
 *   Restore all UART interrupts.
 *
 * Parameters:
 *   priv        - Pointer to the private driver struct.
 *   last_status - Pointer to a variable that stored the last state of the
 *                 interrupt enable register.
 *
 ****************************************************************************/

void esp32c3_lowputc_restore_all_uart_int(const struct esp32c3_uart_s *priv,
                                          uint32_t * last_status);

/****************************************************************************
 * Name: esp32c3_lowputc_config_pins
 *
 * Description:
 *   Configure TX and RX UART pins.
 *
 * Parameters:
 *   priv        - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp32c3_lowputc_config_pins(const struct esp32c3_uart_s *priv);

/****************************************************************************
 * Name: esp32c3_lowputc_restore_pins
 *
 * Description:
 *   Configure both pins back to INPUT mode and detach the TX pin from the
 *   output signal and the RX pin from the input signal.
 *
 * Parameters:
 *   priv        - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp32c3_lowputc_restore_pins(const struct esp32c3_uart_s *priv);

/****************************************************************************
 * Name: esp32c3_lowsetup
 *
 * Description:
 *   This performs basic initialization of the UART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 ****************************************************************************/

void esp32c3_lowsetup(void);

#endif /* __ARCH_RISCV_SRC_ESP32C3_ESP32C3_LOWPUTC_H */
