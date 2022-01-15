/****************************************************************************
 * arch/arm/src/phy62xx/uart.h
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
 *   @file     uart.h
 *   @brief    Contains all functions support for uart driver
 *   @version  0.0
 *   @date     19. Oct. 2017
 *   @author   qing.han
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_PHY62XX_UART_H
#define __ARCH_ARM_SRC_PHY62XX_UART_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "types.h"
#include "gpio.h"

#define UART_TX_FIFO_SIZE    16
#define UART_RX_FIFO_SIZE    16

#define TX_FIFO_MODE    1
#define RX_FIFO_MODE    2
#define TX_RX_FIFO_MODE 3

#define FIFO_MODE 0  /* TX_RX_FIFO_MODE  #0 */

#define FCR_RX_TRIGGER_00 0x00
#define FCR_RX_TRIGGER_01 0x40
#define FCR_RX_TRIGGER_10 0x80
#define FCR_RX_TRIGGER_11 0xc0
#define FCR_TX_TRIGGER_00 0x00
#define FCR_TX_TRIGGER_01 0x10
#define FCR_TX_TRIGGER_10 0x20
#define FCR_TX_TRIGGER_11 0x30
#define FCR_TX_FIFO_RESET 0x04
#define FCR_RX_FIFO_RESET 0x02
#define FCR_FIFO_ENABLE   0x01

#define IER_PTIME   0x80
#define IER_EDSSI   0x08
#define IER_ELSI    0x04
#define IER_ETBEI   0x02
#define IER_ERBFI   0x01

/* LSR 0x14 */
#define LSR_RFE     0x80
#define LSR_TEMT    0x40
#define LSR_THRE    0x20
#define LSR_BI      0x10
#define LSR_FE      0x08
#define LSR_PE      0x04
#define LSR_OE      0x02
#define LSR_DR      0x01

/* USR 0x7c */
#define USR_RFF     0x10
#define USR_RFNE    0x08
#define USR_TFE     0x04
#define USR_TFNF    0x02
#define USR_BUSY    0x01

#define UART_FIFO_RX_TRIGGER    FCR_RX_TRIGGER_10  /* FCR_RX_TRIGGER_10 #FCR_RX_TRIGGER_11 */
#define UART_FIFO_TX_TRIGGER    FCR_TX_TRIGGER_00  /* FCR_TX_TRIGGER_00 #FCR_TX_TRIGGER_01 */

typedef enum
{
  UART0 = 0,  /* use uart 0 */
  UART1 = 1,  /* use uart 1 */
} UART_INDEX_e;

enum UARTIRQID
{
  NONE_IRQ = 0,
  NO_IRQ_PENDING_IRQ = 1,
  THR_EMPTY = 2,
  RDA_IRQ = 4,
  RLS_IRQ = 6,
  BUSY_IRQ = 7,
  TIMEOUT_IRQ = 12,
};

enum
{
  TX_STATE_UNINIT = 0,
  TX_STATE_IDLE,
  TX_STATE_TX,
  TX_STATE_ERR
};

typedef struct _uart_Evt_t
{
  uint8_t   type;
  uint8_t  *data;
  uint8_t   len;
} uart_Evt_t;

typedef enum
{
  UART_EVT_TYPE_RX_DATA = 1,
  UART_EVT_TYPE_RX_DATA_TO,   /* case rx data of uart RX timeout */
  UART_EVT_TYPE_TX_COMPLETED,
} uart_Evt_Type_t;

typedef void (*uart_Hdl_t)(uart_Evt_t *pev);

typedef struct _uart_Cfg_t
{
  gpio_pin_e  tx_pin;
  gpio_pin_e  rx_pin;
  gpio_pin_e  rts_pin;
  gpio_pin_e  cts_pin;
  uint32_t    baudrate;
  bool        use_fifo;
  bool        hw_fwctrl;
  bool        use_tx_buf;
  bool        parity;
  uart_Hdl_t  evt_handler;
} uart_Cfg_t;

typedef struct _uart_spi_t
{
    UART_INDEX_e uart_index;
} uart_t;

typedef struct _uart_Tx_Buf_t
{
  uint8_t   tx_state;
  uint16_t  tx_data_offset;
  uint16_t  tx_data_size;
  uint16_t  tx_buf_size;
  uint8_t  *tx_buf;
} uart_Tx_Buf_t;

int hal_uart_txint_en(UART_INDEX_e uart_index, bool en);
int hal_uart_rxint_en(UART_INDEX_e uart_index, bool en);
int hal_uart_init(uart_Cfg_t cfg, UART_INDEX_e uart_index);
int hal_uart_deinit(UART_INDEX_e uart_index);
int hal_uart_set_tx_buf(UART_INDEX_e uart_index, uint8_t *buf,
    uint16_t size);
int hal_uart_get_tx_ready(UART_INDEX_e uart_index);
int hal_uart_send_buff(UART_INDEX_e uart_index, uint8_t *buff, uint16_t len);
#define logx(...) {char tmp_str[128]; sprintf(tmp_str, __VA_ARGS__); hal_uart_send_buff(0, &tmp_str, strlen(tmp_str) + 1);}
int hal_uart_send_byte(UART_INDEX_e uart_index, unsigned char data);
void __attribute__((weak)) hal_UART0_IRQHandler(void);
void __attribute__((weak)) hal_UART1_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_PHY62XX_UART_H */
