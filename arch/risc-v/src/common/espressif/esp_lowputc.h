/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_lowputc.h
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

#ifndef __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_LOWPUTC_H
#define __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_LOWPUTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#include <sys/lock.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/spinlock.h>

#include "chip.h"
#include "esp_irq.h"
#include "hal/uart_hal.h"
#include "esp_private/critical_section.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Store information used for interfacing with the UART driver */

struct esp_uart_s
{
  int                 cpuint;    /* CPU interrupt assigned to this UART */
  irq_priority_t      int_pri;   /* UART Interrupt Priority */
  int                 id;        /* UART ID */
  uint32_t            baud;      /* Configured baud rate */
  bool                stop_b2;   /* Flag for using 2 stop bits */
  uint8_t             bits;      /* Data length (5 to 8 bits) */
  uint8_t             parity;    /* 0=no parity, 1=odd, 2=even */
  uint8_t             txpin;     /* TX pin */
  uint8_t             rxpin;     /* RX pin */
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  uint8_t             rtspin;    /* RTS pin number */
  bool                iflow;     /* Input flow control (RTS) enabled */
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  uint8_t             ctspin;    /* CTS pin number */
  bool                oflow;     /* Output flow control (CTS) enabled */
#endif
#ifdef HAVE_RS485
  uint8_t  rs485_dir_gpio;     /* UART RS-485 DIR GPIO pin cfg */
  bool     rs485_dir_polarity; /* UART RS-485 DIR TXEN polarity */
#endif
  soc_module_clk_t    clk_src;   /* Clock source */
  uart_hal_context_t *hal;       /* HAL context */
  spinlock_t          lock;      /* Spinlock */
};

typedef struct
{
  _lock_t mutex;                 /* Protect uart_module_enable, uart_module_disable, retention, etc. */
  uart_port_t port_id;
  uart_hal_context_t hal;        /* UART hal context */
  DECLARE_CRIT_SECTION_LOCK_IN_STRUCT(spinlock)
  bool hw_enabled;
} uart_context_t;

extern struct esp_uart_s g_uart0_config;
extern struct esp_uart_s g_uart1_config;
extern struct esp_uart_s g_lp_uart0_config;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp_lowputc_send_byte
 *
 * Description:
 *   Send one byte.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *   byte           - Byte to be sent.
 *
 ****************************************************************************/

void esp_lowputc_send_byte(const struct esp_uart_s *priv,
                           char byte);

/****************************************************************************
 * Name: esp_lowputc_disable_all_uart_int
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

void esp_lowputc_disable_all_uart_int(struct esp_uart_s *priv,
                                      uint32_t *current_status);

/****************************************************************************
 * Name: esp_lowputc_restore_all_uart_int
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

void esp_lowputc_restore_all_uart_int(const struct esp_uart_s *priv,
                                      uint32_t *last_status);

/****************************************************************************
 * Name: esp_lowputc_config_pins
 *
 * Description:
 *   Configure TX and RX UART pins.
 *
 * Parameters:
 *   priv        - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp_lowputc_config_pins(const struct esp_uart_s *priv);

/****************************************************************************
 * Name: esp_lowputc_restore_pins
 *
 * Description:
 *   Configure both pins back to INPUT mode and detach the TX pin from the
 *   output signal and the RX pin from the input signal.
 *
 * Parameters:
 *   priv        - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp_lowputc_restore_pins(const struct esp_uart_s *priv);

/****************************************************************************
 * Name: esp_lowsetup
 *
 * Description:
 *   This performs basic initialization of the UART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 ****************************************************************************/

void esp_lowsetup(void);

bool esp_lowputc_uart_module_enable(const struct esp_uart_s *priv);

#endif /* __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_LOWPUTC_H */
