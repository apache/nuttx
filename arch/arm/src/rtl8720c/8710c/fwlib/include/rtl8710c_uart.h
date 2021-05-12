/**************************************************************************//**
 * @file     rtl8710c_uart.h
 * @brief    The HAL related definition and macros for the UART device.
 *           Includes Registers and data type definition.
 * @version  V1.00
 * @date     2016-07-20
 *
 * @note
 *
 ******************************************************************************
 *
 * Copyright(c) 2007 - 2017 Realtek Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/

#ifndef _RTL8710C_UART_H_
#define _RTL8710C_UART_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup hs_hal_uart UART
 * @ingroup 8710c_hal
 * @{
 * @brief The UART HAL module of the AmebaZ2 platform.
 */


#include "rtl8710c_uart_type.h"

/// The number of UART port on this platform
#define MAX_UART_PORT                   4

/// the minimum OVSR value for baud rate setting calculation
#define UART_OVSR_POOL_MIN      1000    // 10.0
/// the maximum OVSR value for baud rate setting calculation
#define UART_OVSR_POOL_MAX      2099    //20.9
/// the setp size of the divisor increasing for the baud rate setting calculation
#define DIVISOR_RESOLUTION      10
#define JITTER_LIMIT            100
/// the setp size of the divisor increasing for the baud rate setting calculation
#define UART_SCLK               PLATFORM_SCLK

/**
  \brief  Define the UART port ID.
*/
enum  uart_id_e {
    Uart0             = 0,
    Uart1             = 1,
    Uart2             = 2,
    Uart3             = 3,

    MaxUartNum        = 4
};
typedef uint8_t uart_id_t;

/**
  \brief  Defines the FIFO size.
*/
enum uart_fifo_size_e {
    Uart_Tx_FIFO_Size   = 16,
    Uart_Rx_FIFO_Size   = 32
};

/**
  \brief  Define the level for RX FIFO empty event trigger.
*/
enum uart_rxfifo_trigger_level_e {
    OneByte       = 0x00,
    FourBytes     = 0x01,
    EightBytes    = 0x10,
    FourteenBytes = 0x11
};

/**
  \brief  Defines the hardware auto folw control setting, same as register's definition.
*/
enum uart_flow_ctrl_e {
    AutoFlowCtrlDisable = 0,
    AutoFlowCtrlEnable = 1
};

/**
  \brief  Defines the selection for the bit length of a word, same as register's definition.
*/
enum uart_word_len_sel_e {
    UartWordLen7b = 0,
    UartWordLen8b = 1
};

/**
  \brief  Defines the number of stop bits of a frame, same as register's definition.
*/
enum uart_stop_bits_e {
    UartStopBits_1 = 0,
    UartStopBits_2 = 1,
};

/**
  \brief  Defines the parity checking control, same as register's definition.
*/
enum uart_parity_control_e {
    UartParityDisable = 0,
    UartParityEnable = 1
};

/**
  \brief  Defines the parity checking type.
*/
enum uart_parity_type_e {
    UartParityNone = 0,
    UartParityOdd = 1,
    UartParityEven = 2,
    UartParityForced1 = 3,
    UartParityForced0 = 4
};

/**
  \brief  Defines the stick parity checking control, same as register's definition.
*/
enum uart_stick_parity_control_e {
    UartStickParityDisable = 0,
    UartStickParityEnable = 1
};

/**
  \brief  Defines the interrupt ID, same as register's definition.
*/
enum uart_interrupt_id_e {
    ModemStatus           = 0,
    TxFifoEmpty           = 1,
    ReceiverDataAvailable = 2,
    ReceivLineStatus      = 3,
    TimeoutIndication     = 6
};

/**
  \brief  Defines the RX filter match pattern length, 1 byte or 2 bytes.
*/
enum uart_rx_filter_len_e {
    UartRxFilter1Byte     = 0,
    UartRxFilter2Bytes    = 1
};

/**
  \brief  Defines the RX filter pattern matching mode.
*/
enum uart_rx_filter_op_e {
    UartRxFilter_OpAnd    = 0,
    UartRxFilter_OpOr     = 1,
    UartRxFilter_OpXor    = 2
};

/**
  \brief  Defines the trigger levels of the TX FIFO empty interrupt.
*/
enum _uart_tx_fifo_level_e {
    UartTxFifoLow_4bytes    = 0,
    UartTxFifoLow_8bytes    = 1
};

/**
  \brief  Define the trigger levels of the RX FIFO full interrupt.
*/
enum _uart_rx_fifo_level_e {
    UartRxFifoLev_1byte      = 0,
    UartRxFifoLev_8bytes     = 1,
    UartRxFifoLev_16bytes    = 2,
    UartRxFifoLev_28bytes    = 3
};

/**
  \brief  Defines the UART HAL states.
*/
enum HAL_UART_State_e {
  HAL_UART_STATE_NULL              = 0x00,    ///< UART hardware not been initial yet
  HAL_UART_STATE_TX_BUSY           = 0x01,    ///< UART is buzy on TX
  HAL_UART_STATE_RX_BUSY           = 0x02,    ///< UART is busy on RX
  HAL_UART_STATE_TIMEOUT           = 0x04,    ///< Transfer timeout
  HAL_UART_STATE_DMATX_BUSY        = 0x10,    ///< UART is buzy on DMA TX
  HAL_UART_STATE_DMARX_BUSY        = 0x20,    ///< UART is busy on DMA RX
  HAL_UART_STATE_ERROR             = 0x80     ///< UART Error
};

/**
  \brief  Defines the UART HAL error status.
*/
enum _HAL_UART_Status_ {
  HAL_UART_STATUS_OK               = 0x00,    ///< Transfer OK
  HAL_UART_STATUS_TIMEOUT          = 0x01,    ///< Transfer Timeout
  HAL_UART_STATUS_ERR_OVERRUN      = 0x02,    ///< RX Over run
  HAL_UART_STATUS_ERR_PARITY       = 0x04,    ///< Parity error
  HAL_UART_STATUS_ERR_FRAM         = 0x08,    ///< Framing Error
  HAL_UART_STATUS_ERR_BREAK        = 0x10,    ///< Break Interrupt
  HAL_UART_STATUS_ERR_PARA         = 0x20,    ///< Parameter error
  HAL_UART_STATUS_ERR_RXFIFO       = 0x80,    ///< RX FIFO error
};

/// the value to check the line status error
#define UART_LSR_ERR                (HAL_UART_STATUS_ERR_OVERRUN | HAL_UART_STATUS_ERR_PARITY | \
                                     HAL_UART_STATUS_ERR_FRAM | HAL_UART_STATUS_ERR_BREAK |\
                                     HAL_UART_STATUS_ERR_RXFIFO)

/**
  \brief  Defines the hardware flow control types.
*/
enum uart_flow_ctrl_setting_e {
    UartFlowCtlNone     = 0,
    UartFlowCtlRTS      = 1,
    UartFlowCtlCTS      = 2,
    UartFlowCtlRTSCTS   = 3
};

enum uart_pin_func_type_e {
    UART_Pin_TX         = 0,
    UART_Pin_RX         = 1,
    UART_Pin_RTS        = 2,
    UART_Pin_CTS        = 3
};
typedef uint8_t uart_pin_func_t;

/// The special timeout value as the wait forever.
#define UART_WAIT_FOREVER       0xffffffff

#define UART_DMA_MBLK_NUM       16      // maximum block number for each DMA transfer, it must <= 16
#define UART_DMA_BLOCK_SIZE     4092    // the block size of multiple block DMA, it cannot over 4095

/**
  \brief  UART call back function for TX/RX done, RX filter and RX idle events.
*/
typedef void (*uart_callback_t) (void *arg);
/**
  \brief  UART call back function for line status error events.
*/
typedef void (*uart_lsr_callback_t) (uint8_t lsr, void *arg);
/**
  \brief  UART call back function for TX done or RX data ready interrupr.
*/
typedef void (*uart_irq_callback_t)(uint32_t id, uint32_t event);

/**
  * @brief The structure to be used to pass the parameters for
  *        the calculation of OVSR and divisor  for a given baud rate.
  */
typedef struct uart_speed_setting_s {
    uint32_t baudrate;       /*!< baud rate setting */
    uint32_t ovsr;           /*!< generated over sampling value */
    uint32_t div;            /*!< generated divisor value */
    uint32_t ovsr_adj;       /*!< generated over sampling adjustment value */
    uint8_t ovsr_adj_max_bits;  /*!< the maximum bits number of a frame, 9: No parity, 10: with Parity */
    uint8_t ovsr_adj_bits;   /*!< the adjustment bits number in a frame */
    uint8_t reserv1[2];
    const uint16_t *ovsr_adj_map;  /*!< the table for adjustment bits */
    uint32_t max_err;        /*!< maximum baud rate error percentage, 10 ~ 100: 30 */
    uint32_t ovsr_min;       /*!< minum value of OVSR, 10 ~ 20: 1000 */
    uint32_t ovsr_max;       /*!< maximum value of OVSR, 10 ~ 20: 2000 */
    uint32_t divisor_resolution; /*!< the step resolution for availabe divisor value seaching, 1 ~ 20: 10 */
    uint32_t jitter_lim;     /*!< limition for jitter of bits, 50 ~ 100: 100 */
    uint32_t sclk;           /*!< the input system clock */
} uart_speed_setting_t, *puart_speed_setting_t;

/**
  \brief  The data structure for an UART port HAL operations.
*/
typedef struct hal_uart_adapter_s {
    UART0_Type *base_addr;      /*!< The UART register base address */

    volatile uint32_t state;    /*!< UART state: TX/RX busy, error, timeout */
    uint32_t baudrate;          /*!< The baud rate for configuration  */
    uint32_t flow_ctrl;         /*!< HW flow control setting: enable/disable */
    uint32_t tx_count;          /*!< The number of bytes data for current TX transfer */
    uint32_t rx_count;          /*!< The number of bytes data to be received for current RX transfer */
    uint8_t *ptx_buf;           /*!< the data buffer for TX */
    uint8_t *prx_buf;           /*!< the data buffer for RX */
    uint8_t *ptx_buf_sar;       /*!< the buffer source address for TX */
    uint8_t *prx_buf_dar;       /*!< the buffer destination address for RX */
    uint8_t tx_status;          /*!< TX transfer status (result), includes line error status */
    uint8_t rx_status;          /*!< RX transfer status (result), includes line error status */
    uint8_t uart_idx;           /*!< The UART index number */
    uint8_t word_len;           /*!< frame length select: 0 -> 7 bits, 1 -> 8 bits */
    uint8_t stop_bit;           /*!< stop bit length selection: 0 -> 1 stop bit, 1 -> 2 stop bit */
    uint8_t frame_bits;         /*!< total bits of a frame: start bit + word len + parity bit, it's baud rate calculation related */
    uint8_t parity_type;        /*!< parity check type: 0:Odd, 1: Even */
    uint8_t modem_status;       /*!< the modem status, it's updated in modem status ISR */
    uint8_t tx_dma_burst_size;  /*!< the burst size setting of a TX GDMA transfer */
    uint8_t rx_dma_burst_size;  /*!< the burst size setting of a RX GDMA transfer */
    uint8_t is_inited;          /*!< is UART initialed */
    uint8_t lsr;                /*!< LSR record */
    uint8_t tx_dma_width_1byte; /*!< is force TX DAM 1 byte for each transfer */
    uint8_t rx_dma_width_1byte; /*!< is force RX DAM 1 byte for each transfer */
    uint8_t tx_pin;             /*!< TX pin name */
    uint8_t rx_pin;             /*!< RX pin name */
    uint8_t rts_pin;            /*!< RTS pin name */
    uint8_t cts_pin;            /*!< CTS pin name */
    uint8_t reserv1[2];         ///< reserved

    const uint32_t *pdef_baudrate_tbl;      /*!< the pre-defined baud rate table */
    const uint8_t *pdef_ovsr_tbl;           /*!< OVSR table for pre-defined baud rate */
    const uint16_t *pdef_div_tbl;           /*!< the table of DIV for pre-defined baud rate */
    const uint8_t  *pdef_ovsradjbit_tbl10;  /*!< the table of OVSR-Adj bits for 10 bits frame */
    const uint8_t  *pdef_ovsradjbit_tbl9;   /*!< the table of OVSR-Adj bits for 9 bits frame */
    const uint8_t  *pdef_ovsradjbit_tbl8;   /*!< the table of OVSR-Adj bits for 8 bits frame */
    const uint16_t *pdef_ovsradj_tbl10;     /*!< the table of OVSR-Adj for for 10 bits frame */
    const uint16_t *pdef_ovsradj_tbl9;      /*!< the table of OVSR-Adj for for 9 bits frame */
    const uint16_t *pdef_ovsradj_tbl8;      /*!< the table of OVSR-Adj for for 8 bits frame */

    uart_callback_t modem_status_ind;       /*!< callback function for modem status indication  */
    uart_irq_callback_t tx_td_callback;     /*!< callback function for THR empty  */
    uart_irq_callback_t rx_dr_callback;     /*!< callback function for RX data ready  */
    uint32_t tx_td_cb_id;                   /*!< the argument for TX THR empty callback function */
    uint32_t rx_dr_cb_id;                   /*!< the argument for RX data ready callback function */
    uint32_t tx_td_cb_ev;                   /*!< the argument for TX THR empty callback function */
    uint32_t rx_dr_cb_ev;                   /*!< the argument for RX data ready callback function */
    uart_callback_t tx_done_callback;           /*!< User callback function for Interrupt / DMA mode Tx complete  */
    uart_callback_t rx_done_callback;           /*!< User callback function for Interrupt / DMA mode Rx complete */
    void *tx_done_cb_para;                      /*!< the argument for TX complete callback function */
    void *rx_done_cb_para;                      /*!< the argument for RX complete callback function */
    uart_lsr_callback_t lsr_callback;           /*!< User callback function for line status error */
    void *lsr_cb_para;                          /*!< the argument for line status error callback function */
    uart_callback_t rx_flt_timeout_callback;    /*!< callback function for RX filter timeout indication  */
    uart_callback_t rx_flt_matched_callback;    /*!< callback function for RX filter matched indication  */
    void *rx_flt_timeout_cb_arg;                /*!< the argument for RX filter timeout callback function */
    void *rx_flt_matched_cb_arg;                /*!< the argument for RX filter matched callback function */
    uart_callback_t rx_idle_timeout_callback;   /*!< callback function for RX idle timeout indication  */
    void *rx_idle_timeout_cb_arg;               /*!< the argument for RX idle timeout callback function */

    hal_gdma_adaptor_t *ptx_gdma;           /*!< The GDMA channel handler for the UART TX */
    hal_gdma_adaptor_t *prx_gdma;           /*!< The GDMA channel handler for the UART RX */

    void (*dcache_invalidate_by_addr)(uint32_t *addr, int32_t dsize);   /*!< callback function to do the D-cache invalidate  */
    void (*dcache_clean_by_addr) (uint32_t *addr, int32_t dsize);       /*!< callback function to do the D-cache clean  */

    // following members are added for B-cut ROM
    uart_callback_t tx_fifo_low_callback;       /*!< User callback function for Interrupt mode Tx event: TX FIFO low  */
    void *tx_fifo_low_cb_para;                  /*!< the argument for TX FIFO low callback function */
}hal_uart_adapter_t, *phal_uart_adapter_t;

/**
  \brief  The data structure to handle the common resource and setting for all UART adapters.
*/
typedef struct hal_uart_group_adapter_s {
    volatile uint32_t critical_lv;                  /*!< to record UART HAL enter critical section level */
    hal_uart_adapter_t *uart_adapter[MaxUartNum];   /*!< All the UART adapters of this platform */
    irq_handler_t   irq_fun[MaxUartNum];            /*!< the IRQ handler for different UART adapters */
} hal_uart_group_adapter_t, *phal_uart_group_adapter_t;

/**
  \brief  The data structure to be used to initial a new UART adapter.
*/
typedef struct hal_uart_defconfig_s {
    uint32_t baudrate;              /*!< The baud rate for configuration  */
    uint8_t flow_ctrl;              /*!< flow control setting */
    uint8_t word_len;               /*!< frame length select: 0 -> 7 bits, 1 -> 8 bits */
    uint8_t stop_bit;               /*!< stop bit length selection: 0 -> 1 stop bit, 1 -> 2 stop bit */
    uint8_t parity_type;            /*!< parity check type: 0:Odd, 1: Even */

    const uint32_t *pdef_baudrate_tbl;      /*!< the pre-defined baud rate table */
    const uint8_t *pdef_ovsr_tbl;           /*!< OVSR table for pre-defined baud rate */
    const uint16_t *pdef_div_tbl;           /*!< the table of DIV for pre-defined baud rate */
    const uint8_t  *pdef_ovsradjbit_tbl10;  /*!< point to the table of OVSR-Adj bits for 10 bits frame */
    const uint8_t  *pdef_ovsradjbit_tbl9;   /*!< point to the table of OVSR-Adj bits for 9 bits frame */
    const uint8_t  *pdef_ovsradjbit_tbl8;   /*!< point to the table of OVSR-Adj bits for 8 bits frame */
    const uint16_t *pdef_ovsradj_tbl10;     /*!< point to the table of OVSR-Adj for for 10 bits frame */
    const uint16_t *pdef_ovsradj_tbl9;      /*!< point to the table of OVSR-Adj for for 9 bits frame */
    const uint16_t *pdef_ovsradj_tbl8;      /*!< point to the table of OVSR-Adj for for 8 bits frame */
}hal_uart_defconfig_t, *phal_uart_defconfig_t;

/**
  \brief  The data type of a element of the pre-defined baud rate table.
          It's used as the hash for bard rate index searching in the baud rate table.
*/
typedef struct hal_uart_baudrate_hash_s {
    uint32_t baud_rate;  /*!< the baud rate */
    uint32_t idx;        /*!< the index of this baud rate in the baud rate table */
} hal_uart_baudrate_hash_t, *phal_uart_baudrate_hash_t;

/// @cond DOXYGEN_ROM_HAL_API

/**
 * @addtogroup hs_hal_uart_rom_func UART HAL ROM APIs.
 * @{
 */

/**
 *  @brief To clear a flag from the UART state.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  state  The flags(bits) going to be cleared.
 *
 *  @returns    void.
 */
__STATIC_INLINE void uart_clear_state_rtl8710c(phal_uart_adapter_t puart_adapter, uint32_t state)
{
    puart_adapter->state &= ~state;
    __DSB();
}

void hal_uart_enter_critical_rtl8710c (void);
void hal_uart_exit_critical_rtl8710c (void);
void hal_uart_en_ctrl_rtl8710c (uint8_t uart_idx, BOOL en);
hal_status_t hal_uart_reset_rx_fifo_rtl8710c (phal_uart_adapter_t puart_adapter);
hal_status_t hal_uart_gen_baudrate_rtl8710c (uart_speed_setting_t *pbaud_setting);
hal_status_t hal_uart_set_baudrate_rtl8710c (phal_uart_adapter_t puart_adapter, uint32_t baudrate);
hal_status_t hal_uart_set_format_rtl8710c (phal_uart_adapter_t puart_adapter, uint32_t data_bits,
                                            uint32_t parity, uint32_t stop_bits);
hal_status_t hal_uart_set_flow_control_rtl8710c (phal_uart_adapter_t puart_adapter, uint32_t flow_ctrl);
void hal_uart_comm_init_rtl8710c (hal_uart_group_adapter_t *puart_comm_adapter);
uint8_t hal_uart_pin_to_idx_rtl8710c (uint8_t pin_name, uart_pin_func_t pin_type);
uint8_t hal_uart_rx_pin_to_idx_rtl8710c (uint8_t pin_name);
uint8_t hal_uart_tx_pin_to_idx_rtl8710c (uint8_t pin_name);
uint8_t hal_uart_rts_pin_to_idx_rtl8710c (uint8_t pin_name);
uint8_t hal_uart_cts_pin_to_idx_rtl8710c (uint8_t pin_name);
hal_status_t hal_uart_init_rtl8710c (phal_uart_adapter_t puart_adapter, uint8_t tx_pin, uint8_t rx_pin,
                                        phal_uart_defconfig_t pconfig);
void hal_uart_deinit_rtl8710c (phal_uart_adapter_t puart_adapter);
hal_status_t hal_uart_tx_gdma_init_rtl8710c(phal_uart_adapter_t puart_adapter, phal_gdma_adaptor_t pgdma_chnl);
hal_status_t hal_uart_tx_gdma_deinit_rtl8710c(phal_uart_adapter_t puart_adapter);
hal_status_t hal_uart_rx_gdma_init_rtl8710c(phal_uart_adapter_t puart_adapter, phal_gdma_adaptor_t pgdma_chnl);
hal_status_t hal_uart_rx_gdma_deinit_rtl8710c(phal_uart_adapter_t puart_adapter);
BOOL hal_uart_writeable_rtl8710c (phal_uart_adapter_t puart_adapter);
void hal_uart_putc_rtl8710c (phal_uart_adapter_t puart_adapter, uint8_t tx_data);
void hal_uart_wputc_rtl8710c (phal_uart_adapter_t puart_adapter, uint8_t tx_data);
void hal_uart_wait_tx_done_rtl8710c (phal_uart_adapter_t puart_adapter, uint32_t timeout_ms);
uint32_t hal_uart_send_rtl8710c (phal_uart_adapter_t puart_adapter, uint8_t *ptx_buf, uint32_t len, uint32_t timeout_ms);
hal_status_t hal_uart_int_send_rtl8710c (phal_uart_adapter_t puart_adapter, uint8_t *ptx_buf, uint32_t len);
hal_status_t hal_uart_dma_send_rtl8710c (phal_uart_adapter_t puart_adapter, uint8_t *ptx_buf, uint32_t len);
hal_status_t hal_uart_send_abort_rtl8710c (phal_uart_adapter_t puart_adapter);
BOOL hal_uart_readable_rtl8710c (phal_uart_adapter_t puart_adapter);
char hal_uart_getc_rtl8710c (phal_uart_adapter_t puart_adapter);
int hal_uart_rgetc_rtl8710c (phal_uart_adapter_t puart_adapter, char *data);
uint32_t hal_uart_recv_rtl8710c (phal_uart_adapter_t puart_adapter, uint8_t *prx_buf, uint32_t len, uint32_t timeout_ms);
hal_status_t hal_uart_int_recv_rtl8710c (phal_uart_adapter_t puart_adapter, uint8_t *prx_buf, uint32_t len);
hal_status_t hal_uart_dma_recv_rtl8710c (phal_uart_adapter_t puart_adapter, uint8_t *prx_buf, uint32_t len);
uint32_t hal_uart_recv_abort_rtl8710c (phal_uart_adapter_t puart_adapter);
uint8_t hal_uart_get_imr_rtl8710c (phal_uart_adapter_t puart_adapter);
void hal_uart_set_imr_rtl8710c (phal_uart_adapter_t puart_adapter);
void hal_uart_set_rts_rtl8710c (phal_uart_adapter_t puart_adapter, BOOLEAN rts_ctrl);
void hal_uart_tx_pause_rtl8710c (phal_uart_adapter_t puart_adapter, BOOLEAN cts_ctrl);
void hal_uart_reg_comm_irq_rtl8710c (irq_handler_t handler);
void hal_uart_reg_irq_rtl8710c (phal_uart_adapter_t puart_adapter, irq_handler_t handler);
void hal_uart_unreg_irq_rtl8710c (phal_uart_adapter_t puart_adapter);
void hal_uart_adapter_init_rtl8710c (phal_uart_adapter_t puart_adapter, uint8_t uart_idx, phal_uart_defconfig_t pconfig);
void hal_uart_line_sts_hook_rtl8710c (phal_uart_adapter_t puart_adapter, uart_lsr_callback_t pcallback, void *pdata);
void hal_uart_rxind_hook_rtl8710c (phal_uart_adapter_t puart_adapter, uart_irq_callback_t pcallback,
                                     uint32_t id, uint32_t event);
void hal_uart_txtd_hook_rtl8710c (phal_uart_adapter_t puart_adapter, uart_irq_callback_t pcallback,
                                    uint32_t id, uint32_t event);
void hal_uart_txdone_hook_rtl8710c (phal_uart_adapter_t puart_adapter, uart_callback_t pcallback, void *parg);
void hal_uart_rxdone_hook_rtl8710c (phal_uart_adapter_t puart_adapter, uart_callback_t pcallback, void *parg);
void hal_uart_set_rx_filter_pattern_rtl8710c (phal_uart_adapter_t puart_adapter, uint32_t pattern,
                                                uint32_t mask, uint32_t mask_en);
void hal_uart_set_rx_filter_op_rtl8710c (phal_uart_adapter_t puart_adapter, uint32_t pattern_len, uint32_t match_op);
void hal_uart_set_rx_filter_timeout_rtl8710c (phal_uart_adapter_t puart_adapter, uint32_t timeout_ms,
                                                uart_callback_t callback, void *cb_arg);
void hal_uart_rx_filter_en_rtl8710c (phal_uart_adapter_t puart_adapter, uart_callback_t callback, void *cb_arg);
void hal_uart_rx_filter_dis_rtl8710c (phal_uart_adapter_t puart_adapter);
void hal_uart_reset_receiver_rtl8710c (phal_uart_adapter_t puart_adapter);
void hal_uart_set_tx_fifo_level_rtl8710c (phal_uart_adapter_t puart_adapter, uint32_t low_level);
void hal_uart_set_rx_fifo_level_rtl8710c (phal_uart_adapter_t puart_adapter, uint32_t level);
void hal_uart_rx_idle_timeout_en_rtl8710c (phal_uart_adapter_t puart_adapter, uint32_t timeout_us,
                                            uart_callback_t pcallback, void *parg);
void hal_uart_rx_idle_timeout_dis_rtl8710c (phal_uart_adapter_t puart_adapter);
void hal_uart_tx_fifo_low_hook_rtl8710c (phal_uart_adapter_t puart_adapter, uart_callback_t pcallback,
                                                        void *parg);

/** @} */ /* End of group hs_hal_uart_rom_func */
/// @endcond /* End of condition DOXYGEN_ROM_HAL_API */

/**
  \brief  The data structure of the stubs functions of the UART HAL functions in ROM.
*/
typedef struct hal_uart_func_stubs_s {
    phal_uart_group_adapter_t *ppuart_gadapter;
    hal_status_t (*hal_uart_reset_rx_fifo) (phal_uart_adapter_t puart_adapter);
    hal_status_t (*hal_uart_gen_baudrate) (uart_speed_setting_t *pbaud_setting);
    hal_status_t (*hal_uart_set_baudrate) (phal_uart_adapter_t puart_adapter, uint32_t baudrate);
    hal_status_t (*hal_uart_set_format) (phal_uart_adapter_t puart_adapter, uint32_t data_bits,
                                         uint32_t parity, uint32_t stop_bits);
    hal_status_t (*hal_uart_set_flow_control) (phal_uart_adapter_t puart_adapter, uint32_t flow_ctrl);
    void (*hal_uart_comm_init) (hal_uart_group_adapter_t *puart_comm_adapter);
    hal_status_t (*hal_uart_init) (phal_uart_adapter_t puart_adapter, uint8_t tx_pin, uint8_t rx_pin,
                                   phal_uart_defconfig_t pconfig);
    void (*hal_uart_deinit) (phal_uart_adapter_t puart_adapter);
    void (*uart_irq_handler) (phal_uart_adapter_t puart_adapter);
    hal_status_t (*hal_uart_tx_gdma_init)(phal_uart_adapter_t puart_adapter, phal_gdma_adaptor_t pgdma_chnl);
    hal_status_t (*hal_uart_tx_gdma_deinit)(phal_uart_adapter_t puart_adapter);
    hal_status_t (*hal_uart_rx_gdma_init)(phal_uart_adapter_t puart_adapter, phal_gdma_adaptor_t pgdma_chnl);
    hal_status_t (*hal_uart_rx_gdma_deinit)(phal_uart_adapter_t puart_adapter);
    void (*uart_tx_dma_irq_handler)( phal_uart_adapter_t puart_adapter);
    void (*uart_rx_dma_irq_handler)( phal_uart_adapter_t puart_adapter);
    BOOL (*hal_uart_writeable) (phal_uart_adapter_t puart_adapter);
    void (*hal_uart_putc) (phal_uart_adapter_t puart_adapter, uint8_t tx_data);
    void (*hal_uart_wputc) (phal_uart_adapter_t puart_adapter, uint8_t tx_data);
    void (*hal_uart_wait_tx_done) (phal_uart_adapter_t puart_adapter, uint32_t timeout_ms);
    uint32_t (*hal_uart_send) (phal_uart_adapter_t puart_adapter, uint8_t *ptx_buf, uint32_t len, uint32_t timeout_ms);
    hal_status_t (*hal_uart_int_send) (phal_uart_adapter_t puart_adapter, uint8_t *ptx_buf, uint32_t len);
    hal_status_t (*hal_uart_dma_send) (phal_uart_adapter_t puart_adapter, uint8_t *ptx_buf, uint32_t len);
    uint32_t (*hal_uart_send_abort) (phal_uart_adapter_t puart_adapter);
    BOOL (*hal_uart_readable) (phal_uart_adapter_t puart_adapter);
    char (*hal_uart_getc) (phal_uart_adapter_t puart_adapter);
    int (*hal_uart_rgetc) (phal_uart_adapter_t puart_adapter, char *data);
    uint32_t (*hal_uart_recv) (phal_uart_adapter_t puart_adapter, uint8_t *prx_buf, uint32_t len, uint32_t timeout_ms);
    hal_status_t (*hal_uart_int_recv) (phal_uart_adapter_t puart_adapter, uint8_t *prx_buf, uint32_t len);
    hal_status_t (*hal_uart_dma_recv) (phal_uart_adapter_t puart_adapter, uint8_t *prx_buf, uint32_t len);
    uint32_t (*hal_uart_recv_abort) (phal_uart_adapter_t puart_adapter);
    uint8_t (*hal_uart_get_imr) (phal_uart_adapter_t puart_adapter);
    void (*hal_uart_set_imr) (phal_uart_adapter_t puart_adapter);
    void (*hal_uart_set_rts) (phal_uart_adapter_t puart_adapter, BOOLEAN rts_ctrl);
    void (*hal_uart_tx_pause) (phal_uart_adapter_t puart_adapter, BOOLEAN cts_ctrl);
    void (*hal_uart_reg_comm_irq) (irq_handler_t handler);
    void (*hal_uart_reg_irq) (phal_uart_adapter_t puart_adapter, irq_handler_t handler);
    void (*hal_uart_unreg_irq) (phal_uart_adapter_t puart_adapter);
    void (*hal_uart_adapter_init) (phal_uart_adapter_t puart_adapter, uint8_t uart_idx, phal_uart_defconfig_t pconfig);
    void (*hal_uart_line_sts_hook) (phal_uart_adapter_t puart_adapter, uart_lsr_callback_t pcallback, void *pdata);
    void (*hal_uart_txtd_hook) (phal_uart_adapter_t puart_adapter, uart_irq_callback_t pcallback, uint32_t id, uint32_t event);
    void (*hal_uart_rxind_hook) (phal_uart_adapter_t puart_adapter, uart_irq_callback_t pcallback, uint32_t id, uint32_t event);
    void (*hal_uart_txdone_hook) (phal_uart_adapter_t puart_adapter, uart_callback_t pcallback, void *parg);
    void (*hal_uart_rxdone_hook) (phal_uart_adapter_t puart_adapter, uart_callback_t pcallback, void *parg);
    void (*hal_uart_set_rx_filter_pattern) (phal_uart_adapter_t puart_adapter, uint32_t pattern,
                                            uint32_t mask, uint32_t mask_en);
    void (*hal_uart_set_rx_filter_op) (phal_uart_adapter_t puart_adapter, uint32_t pattern_len, uint32_t match_op);
    void (*hal_uart_set_rx_filter_timeout) (phal_uart_adapter_t puart_adapter, uint32_t timeout_ms,
                                            uart_callback_t callback, void *cb_arg);
    void (*hal_uart_rx_filter_en) (phal_uart_adapter_t puart_adapter, uart_callback_t callback, void *cb_arg);
    void (*hal_uart_rx_filter_dis) (phal_uart_adapter_t puart_adapter);
    void (*hal_uart_reset_receiver) (phal_uart_adapter_t puart_adapter);
    void (*hal_uart_set_tx_fifo_level) (phal_uart_adapter_t puart_adapter, uint32_t low_level);
    void (*hal_uart_set_rx_fifo_level) (phal_uart_adapter_t puart_adapter, uint32_t level);
    void (*hal_uart_rx_idle_timeout_en) (phal_uart_adapter_t puart_adapter, uint32_t timeout_us,
                                         uart_callback_t pcallback, void *parg);
    void (*hal_uart_rx_idle_timeout_dis) (phal_uart_adapter_t puart_adapter);
    void (*hal_uart_enter_critical) (void);
    void (*hal_uart_exit_critical) (void);
    void (*hal_uart_en_ctrl) (uint8_t uart_idx, BOOL en);
    void (*hal_uart_tx_fifo_low_hook) (phal_uart_adapter_t puart_adapter, uart_callback_t pcallback, void *parg);
    uint8_t (*hal_uart_pin_to_idx) (uint8_t pin_name, uart_pin_func_t pin_type);
    void (*hal_uart_tx_isr) (phal_uart_adapter_t puart_adapter);
    void (*hal_uart_rx_isr) (phal_uart_adapter_t puart_adapter);
    void (*hal_uart_iir_isr) (phal_uart_adapter_t puart_adapter);

    uint32_t reserved[16];  // reserved space for next ROM code version function table extending.
} hal_uart_func_stubs_t;

/** @} */ /* End of group hs_hal_uart */

#ifdef __cplusplus
}
#endif

#endif  // #ifndef _RTL8710C_UART_H_

