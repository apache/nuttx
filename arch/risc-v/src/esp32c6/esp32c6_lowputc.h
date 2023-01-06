/****************************************************************************
 * arch/risc-v/src/esp32c6/esp32c6_lowputc.h
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

#ifndef __ARCH_RISCV_SRC_ESP32C6_ESP32C6_LOWPUTC_H
#define __ARCH_RISCV_SRC_ESP32C6_ESP32C6_LOWPUTC_H

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

#include "hardware/esp32c6_uart.h"
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
    UART_STOP_BITS_1   = 0x1,  /* stop bit: 1 bit */
    UART_STOP_BITS_2   = 0x3,  /* stop bit: 2bits */
};

/* Default FIFOs size */

#define UART_TX_FIFO_SIZE 128
#define UART_RX_FIFO_SIZE 128

/* Struct used to store uart driver information and to
 * manipulate uart driver
 */

struct esp32c6_uart_s
{
  uint32_t  base;           /* Base address of UART registers */
  uint8_t   periph;         /* UART peripheral ID */
  int       cpuint;         /* CPU interrupt assigned to this UART */
  uint8_t   id;             /* UART ID */
  uint8_t   irq;            /* IRQ associated with this UART */
  uint32_t  baud;           /* Configured baud rate */
  uint8_t   bits;
  uint8_t   parity;         /* 0=no parity, 1=odd parity, 2=even parity */
  uint8_t   stop_b2;        /* Use 2 stop bits? 0 no, others yes */
  uint8_t   int_pri;        /* UART Interrupt Priority */
};

#ifdef CONFIG_ESP32C6_UART0
extern struct esp32c6_uart_s g_uart0_config;
#endif

#ifdef CONFIG_ESP32C6_UART1
extern struct esp32c6_uart_s g_uart1_config;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c6_lowputc_reset_core
 *  Reset both TX and RX core
 ****************************************************************************/

void esp32c6_lowputc_reset_core(const struct esp32c6_uart_s *conf);

/****************************************************************************
 * Name: esp32c6_lowputc_enable_sclk
 *    Enable clock for whole core
 ****************************************************************************/

void esp32c6_lowputc_enable_sclk(const struct esp32c6_uart_s *conf);

/****************************************************************************
 * Name: esp32c6_lowputc_disable_sclk
 *    Disable clock for whole core
 ****************************************************************************/

void esp32c6_lowputc_disable_sclk(const struct esp32c6_uart_s *conf);

/****************************************************************************
 * Name: esp32c6_lowputc_set_sclk
 *    Set a source clock for UART
 *    APB_CLK  = 1  80 MHz
 *    CLK_8    = 2        8 MHz
 *    XTAL_CLK = 3
 ****************************************************************************/

void esp32c6_lowputc_set_sclk(const struct esp32c6_uart_s *conf, enum
                              uart_sclk source);

/****************************************************************************
 * Name: esp32c6_lowputc_get_sclk
 *    Get the source clock for UART
 ****************************************************************************/

uint32_t esp32c6_lowputc_get_sclk(const struct esp32c6_uart_s *conf);

/****************************************************************************
 * Name: esp32c6_lowputc_baud
 *    Set the baud rate
 ****************************************************************************/

void esp32c6_lowputc_baud(const struct esp32c6_uart_s * conf);

/****************************************************************************
 * Name: esp32c6_lowputc_normal_mode
 *    Set the UART to operate in normal mode
 ****************************************************************************/

void esp32c6_lowputc_normal_mode(const struct esp32c6_uart_s * conf);

/****************************************************************************
 * Name: esp32c6_lowputc_parity
 *    Set the parity
 ****************************************************************************/

void esp32c6_lowputc_parity(const struct esp32c6_uart_s * conf);

/****************************************************************************
 * Name: esp32c6_lowputc_data_length
 *    Set the data length
 ****************************************************************************/

int esp32c6_lowputc_data_length(const struct esp32c6_uart_s * conf);

/****************************************************************************
 * Name: esp32c6_lowputc_stop_length
 *    Set the stop length
 ****************************************************************************/

void esp32c6_lowputc_stop_length(const struct esp32c6_uart_s * conf);

/****************************************************************************
 * Name: esp32c6_lowputc_set_tx_idle_time
 *    Set the idle time between transfers
 ****************************************************************************/

void esp32c6_lowputc_set_tx_idle_time(const struct esp32c6_uart_s *
                                      conf, uint32_t time);

/****************************************************************************
 * Name: esp32c6_lowputc_send_byte
 *    Send one byte
 ****************************************************************************/

void esp32c6_lowputc_send_byte(const struct esp32c6_uart_s * conf,
                               char byte);

/****************************************************************************
 * Name: esp32c6_lowputc_is_tx_fifo_full
 *    Send one byte
 ****************************************************************************/

bool esp32c6_lowputc_is_tx_fifo_full(const struct esp32c6_uart_s *conf);

/****************************************************************************
 * Name: esp32c6_lowputc_disable_all_uart_int
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

void esp32c6_lowputc_disable_all_uart_int(const struct esp32c6_uart_s *priv,
                                          uint32_t *current_status);

/****************************************************************************
 * Name: esp32c6_lowputc_restore_all_uart_int
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

void esp32c6_lowputc_restore_all_uart_int(const struct esp32c6_uart_s *priv,
                                          uint32_t *last_status);

/****************************************************************************
 * Name: esp32c6_lowsetup
 *
 * Description:
 *   This performs basic initialization of the UART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 ****************************************************************************/

void esp32c6_lowsetup(void);

#endif /* __ARCH_RISCV_SRC_ESP32C6_ESP32C6_LOWPUTC_H */