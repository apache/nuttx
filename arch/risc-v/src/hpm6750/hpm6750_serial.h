/****************************************************************************
 * arch/risc-v/src/hpm6750/hpm6750_serial.h
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

#ifndef __ARCH_RISCV_SRC_HPM6750_HPM6750_SERIAL_H
#define __ARCH_RISCV_SRC_HPM6750_HPM6750_SERIAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* @brief Parity */

typedef enum parity
{
  parity_none = 0,
  parity_odd,
  parity_even,
  parity_always_1,
  parity_always_0,
} parity_setting_t;

/* @brief Stop bits */

typedef enum num_of_stop_bits
{
  stop_bits_1 = 0,
  stop_bits_1_5,
  stop_bits_2,
} num_of_stop_bits_t;

/* @brief Word length */

typedef enum word_length
{
  word_length_5_bits = 0,
  word_length_6_bits,
  word_length_7_bits,
  word_length_8_bits,
} word_length_t;

/* @brief UART fifo trigger levels */

typedef enum uart_fifo_trg_lvl
{
  uart_rx_fifo_trg_not_empty = 0,
  uart_rx_fifo_trg_gt_one_quarter = 1,
  uart_rx_fifo_trg_gt_half = 2,
  uart_rx_fifo_trg_gt_three_quarters = 3,

  uart_tx_fifo_trg_not_full = 0,
  uart_tx_fifo_trg_lt_three_quarters = 1,
  uart_tx_fifo_trg_lt_half = 2,
  uart_tx_fifo_trg_lt_one_quarter = 3,
} uart_fifo_trg_lvl_t;

/* @brief UART signals */

typedef enum uart_signal
{
  uart_signal_rts = UART_MCR_RTS_MASK,
} uart_signal_t;

/* @brief UART signal levels */

typedef enum uart_signal_level
{
  uart_signal_level_high,
  uart_signal_level_low,
} uart_signal_level_t;

/* @brief UART modem status */

typedef enum uart_modem_stat
{
  uart_modem_stat_cts = UART_MSR_CTS_MASK,
  uart_modem_stat_dcts_changed = UART_MSR_DCTS_MASK,
} uart_modem_stat_t;

/* @brief UART interrupt enable masks */

typedef enum uart_intr_enable
{
  uart_intr_rx_data_avail_or_timeout = UART_IER_ERBI_MASK,
  uart_intr_tx_slot_avail = UART_IER_ETHEI_MASK,
  uart_intr_rx_line_stat = UART_IER_ELSI_MASK,
  uart_intr_modem_stat = UART_IER_EMSI_MASK,
#if defined(UART_SOC_HAS_RXLINE_IDLE_DETECTION) && (UART_SOC_HAS_RXLINE_IDLE_DETECTION == 1)
  uart_intr_rx_line_idle = UART_IER_ERXIDLE_MASK,
#endif
} uart_intr_enable_t;

/* @brief UART interrupt IDs */

typedef enum uart_intr_id
{
  uart_intr_id_modem_stat = 0x0,
  uart_intr_id_tx_slot_avail = 0x2,
  uart_intr_id_rx_data_avail = 0x4,
  uart_intr_id_rx_line_stat = 0x6,
  uart_intr_id_rx_timeout = 0xc,
} uart_intr_id_t;

/* @brief UART status */

typedef enum uart_stat
{
  uart_stat_data_ready = UART_LSR_DR_MASK,
  uart_stat_overrun_error = UART_LSR_OE_MASK,
  uart_stat_parity_error = UART_LSR_PE_MASK,
  uart_stat_framing_error = UART_LSR_FE_MASK,
  uart_stat_line_break = UART_LSR_LBREAK_MASK,
  uart_stat_tx_slot_avail = UART_LSR_THRE_MASK,
  uart_stat_transmitter_empty = UART_LSR_TEMT_MASK,
  uart_stat_rx_fifo_error = UART_LSR_ERRF_MASK,
} uart_stat_t;

/**
 * @brief UART modem config
 */

typedef struct uart_modem_config
{
  bool auto_flow_ctrl_en;     /**< Auto flow control enable flag */
  bool loop_back_en;          /**< Loop back enable flag */
  bool set_rts_high;          /**< Set signal RTS level high flag */
} uart_modem_config_t;

#if defined(UART_SOC_HAS_RXLINE_IDLE_DETECTION) && (UART_SOC_HAS_RXLINE_IDLE_DETECTION == 1)
/**
 * @brief UART RX Line Idle detection conditions
 */

typedef enum hpm_uart_rxline_idle_cond
{
  uart_rxline_idle_cond_rxline_logic_one = 0,         /**< Treat as idle if the RX Line high duration exceeds threshold */
  uart_rxline_idle_cond_state_machine_idle = 1        /**< Treat as idle if the RX state machine idle state duration exceeds threshold */
} uart_rxline_idle_cond_t;

typedef struct hpm_uart_rxline_idle_detect_config
{
  bool detect_enable;                 /**< RX Line Idle detection flag */
  bool detect_irq_enable;             /**< Enable RX Line Idle detection interrupt */
  uart_rxline_idle_cond_t idle_cond;  /**< RX Line Idle detection condition */
  uint8_t threshold;                  /**< UART RX Line Idle detection threshold, in terms of bits */
} uart_rxline_idle_config_t;
#endif

/**
 * @brief UART config
 */

typedef struct hpm_uart_config
{
  uint32_t src_freq_in_hz;                    /**< Source clock frequency in Hz */
  uint32_t baudrate;                          /**< Baudrate */
  uint8_t num_of_stop_bits;                   /**< Number of stop bits */
  uint8_t word_length;                        /**< Word length */
  uint8_t parity;                             /**< Parity */
  uint8_t tx_fifo_level;                      /**< TX Fifo level */
  uint8_t rx_fifo_level;                      /**< RX Fifo level */
  bool dma_enable;                            /**< DMA Enable flag */
  bool fifo_enable;                           /**< Fifo Enable flag */
  uart_modem_config_t modem_config;           /**< Modem config */
#if defined(UART_SOC_HAS_RXLINE_IDLE_DETECTION) && (UART_SOC_HAS_RXLINE_IDLE_DETECTION == 1)
  uart_rxline_idle_config_t  rxidle_config;   /**< RX Idle configuration */
#endif
} uart_config_t;

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_HPM6750_HPM6750_SERIAL_H */
