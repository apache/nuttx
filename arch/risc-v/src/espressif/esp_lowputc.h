/****************************************************************************
 * arch/risc-v/src/espressif/esp_lowputc.h
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

#ifndef __ARCH_RISCV_SRC_ESPRESSIF_ESP_LOWPUTC_H
#define __ARCH_RISCV_SRC_ESPRESSIF_ESP_LOWPUTC_H

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
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "chip.h"
#include "esp_irq.h"
#include "hal/uart_hal.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Store information used for interfacing with the UART driver */

struct esp_uart_s
{
  int                 source;    /* UART interrupt source */
  int                 cpuint;    /* CPU interrupt assigned to this UART */
  irq_priority_t      int_pri;   /* UART Interrupt Priority */
  int                 id;        /* UART ID */
  int                 irq;       /* IRQ associated with this UART */
  uint32_t            baud;      /* Configured baud rate */
  bool                stop_b2;   /* Flag for using 2 stop bits */
  uint8_t             bits;      /* Data length (5 to 8 bits) */
  uint8_t             parity;    /* 0=no parity, 1=odd, 2=even */
  uint8_t             txpin;     /* TX pin */
  uint8_t             txsig;     /* TX signal */
  uint8_t             rxpin;     /* RX pin */
  uint8_t             rxsig;     /* RX signal */
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  uint8_t             rtspin;    /* RTS pin number */
  uint8_t             rtssig;    /* RTS signal */
  bool                iflow;     /* Input flow control (RTS) enabled */
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  uint8_t             ctspin;    /* CTS pin number */
  uint8_t             ctssig;    /* CTS signal */
  bool                oflow;     /* Output flow control (CTS) enabled */
#endif
  uart_hal_context_t *hal;       /* HAL context */
};

extern struct esp_uart_s g_uart0_config;
extern struct esp_uart_s g_uart1_config;

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
 * Name: esp_lowputc_enable_sysclk
 *
 * Description:
 *   Enable clock for the UART using the System register.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp_lowputc_enable_sysclk(const struct esp_uart_s *priv);

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

void esp_lowputc_disable_all_uart_int(const struct esp_uart_s *priv,
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

#endif /* __ARCH_RISCV_SRC_ESPRESSIF_ESP_LOWPUTC_H */
