/**************************************************************************//**
 * @file     hal_uart.h
 * @brief    The HAL API implementation for the UART device.
 * @version  V1.00
 * @date     2016-09-30
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

#ifndef _HAL_UART_H_
#define _HAL_UART_H_

#include "cmsis.h"
#include "hal_irq.h"

#ifdef  __cplusplus
extern "C"
{
#endif

/**
 * @addtogroup hs_hal_uart UART
 * @{
 */

extern const hal_uart_func_stubs_t hal_uart_stubs;

typedef enum uart_tx_fifo_level {
    UartTxFifoLv_4    = 4,
    UartTxFifoLv_8    = 8
} uart_tx_fifo_level_t;

typedef enum uart_rx_fifo_level {
    UartRxFifoLv_1    = 1,
    UartRxFifoLv_8    = 8,
    UartRxFifoLv_16   = 16,
    UartRxFifoLv_28   = 28
} uart_rx_fifo_level_t;

#define NONESET_UART_IDX            (0xFF)

/**
 *  @brief Resets the RX FIFO and the receiver.
 *         The reset steps are:
 *           1. Assert receiver reset hardware bit.
 *           2. Assert clear_rxfifo hardware bit.
 *           3. De-assert receiver reset hardware bit.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *
 *  @returns    Always return HAL_OK.
 */
__STATIC_INLINE hal_status_t hal_uart_reset_rx_fifo (phal_uart_adapter_t puart_adapter)
{
    return hal_uart_stubs.hal_uart_reset_rx_fifo (puart_adapter);
}

/**
 *  @brief Calculates the divisor, over sampling for a given baud rate.
 *
 *  @param[in] pbaud_setting The needed parameters, baud rate and system clock
 *                                for the calculation. The result also will be passed by
 *                                this structure.
 *
 *  @returns        Always return HAL_OK.
 */
__STATIC_INLINE hal_status_t hal_uart_gen_baudrate (uart_speed_setting_t *pbaud_setting)
{
    return hal_uart_stubs.hal_uart_gen_baudrate (pbaud_setting);
}

/**
 *  @brief Configures the baud rate setting.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  baudrate  The baud rate setting. If the baud rate value is invalid then
 *                        baud rate 9600 will be the used for the configuration.
 *
 *  @returns    Always return HAL_OK
 */
__STATIC_INLINE hal_status_t hal_uart_set_baudrate (phal_uart_adapter_t puart_adapter, uint32_t baudrate)
{
    return hal_uart_stubs.hal_uart_set_baudrate (puart_adapter, baudrate);
}

/**
 *  @brief Configures the UART frame format.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  data_bis  The number of bits in a UART word. This value can be 7 or 8.
 *  @param[in]  parity  The parity type.
 *                      - 0: No parity bit.
 *                      - 1: Odd parity.
 *                      - 2: Even parity.
 *                      - 3: Parity bit value always is 1.
 *                      - 4: Parity bit value always is 0.
 *  @param[in]  stop_bits  The number of stop bits. This value can be 1 or 2.
 *
 *  @returns    Always return HAL_OK
 */
__STATIC_INLINE hal_status_t hal_uart_set_format (phal_uart_adapter_t puart_adapter, uint32_t data_bits,
                                            uint32_t parity, uint32_t stop_bits)
{
    return hal_uart_stubs.hal_uart_set_format (puart_adapter, data_bits, parity, stop_bits);
}

/**
 *  @brief To get the UART port TX FIFO writable status.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *
 *  @return     value 1   TX FIFO is writable (not full).
 *  @return     value 0   TX FIFO is full.
 */
__STATIC_INLINE BOOL hal_uart_writeable (phal_uart_adapter_t puart_adapter)
{
    return hal_uart_stubs.hal_uart_writeable (puart_adapter);
}

/**
 *  @brief To send a char.
 *
 *  @param[in]  puart_adapter The UART adapter.
 *  @param[in]  tx_data The data(1 byte) to be send.
 *
 *  @returns    void.
 */
__STATIC_INLINE void hal_uart_putc (phal_uart_adapter_t puart_adapter, u8 tx_data)
{
    puart_adapter->base_addr->thr = tx_data;
}

/**
 *  @brief To wait TX FIFO is writable and then send a char.
 *
 *  @param[in]  puart_adapter The UART adapter.
 *  @param[in]  tx_data The data(1 byte) to be send.
 *
 *  @returns    void.
 */
__STATIC_INLINE
void hal_uart_wputc (phal_uart_adapter_t puart_adapter, u8 tx_data)
{
    hal_uart_stubs.hal_uart_wputc(puart_adapter, tx_data);
}

#define phal_uart_wputc         (hal_uart_stubs.hal_uart_wputc)

/**
 *  @brief Waits all data in the TX FIFO are transfered.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  timeout_ms  The maximum period(in mini-second) to wait the transfer done.
 *
 *  @returns    void.
 */
__STATIC_INLINE void hal_uart_wait_tx_done (phal_uart_adapter_t puart_adapter, uint32_t timeout_ms)
{
    hal_uart_stubs.hal_uart_wait_tx_done (puart_adapter, timeout_ms);
}

/**
 *  @brief To send a block of data.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  ptx_buf  The buffer of data to be transmitted.
 *  @param[in]  len  The length of data in bytes to be transmitted.
 *  @param[in]  timeout_ms  The maximum period(in mini-secand) to wait the data transmission is finished.
 *
 *  @returns    The length of data in byte has been sent.
 */
__STATIC_INLINE uint32_t hal_uart_send (phal_uart_adapter_t puart_adapter, u8 *ptx_buf, u32 len, u32 timeout_ms)
{
    return hal_uart_stubs.hal_uart_send (puart_adapter, ptx_buf, len, timeout_ms);
}

/**
 *  @brief To send a block of data by interrupt mode.
 *         The TX FIFO will be refilled by the TX FIFO level low interrupt handler.
 *         This function returns with no waiting of the transmission done.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  ptx_buf  The buffer of data to be transmitted.
 *  @param[in]  len  The length of data in bytes to be send.
 *
 *  @return     HAL_OK: function execution OK.
 *  @return     HAL_BUSY: UART TX is in busy state, previous transmission is not finished yet.
 *  @return     HAL_ERR_PARA: Input arguments are invalid.
 */
__STATIC_INLINE hal_status_t hal_uart_int_send (phal_uart_adapter_t puart_adapter, u8 *ptx_buf, u32 len)
{
    return hal_uart_stubs.hal_uart_int_send (puart_adapter, ptx_buf, len);
}

/**
 *  @brief To stop and skip currently on going interrupt mode or
 *         DMA mode data transmission.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *
 *  @returns The length of data, in byte, has been transmitted.
 *           Return 0 for the case of there is no on going data transmittion.
 */
__STATIC_INLINE uint32_t hal_uart_send_abort (phal_uart_adapter_t puart_adapter)
{
    return hal_uart_stubs.hal_uart_send_abort (puart_adapter);
}

/**
 *  @brief To check if any data is ready in the RX FIFO.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *
 *  @return     TRUE    at least 1 byte of data is ready in the RX FIFO.
 *  @return     FALSE   no data in the RX FIFO.
 */
__STATIC_INLINE BOOL hal_uart_readable (phal_uart_adapter_t puart_adapter)
{
    return hal_uart_stubs.hal_uart_readable (puart_adapter);
}

/**
 *  @brief To read a byte of data from the RX FIFO.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *
 *  @returns    The the received data.
 */
__STATIC_INLINE char hal_uart_getc (phal_uart_adapter_t puart_adapter)
{
    return (puart_adapter->base_addr->rbr_b.rxdata);
}

/**
 *  @brief To check if any data is ready in RX FIFO and read 1
 *         byte of data if data is available.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[out] data  The gotten character.
 *
 *  @return     TRUE    read RX FIFO OK.
 *  @return     FALSE   no data in the RX FIFO.
 */
__STATIC_INLINE int hal_uart_rgetc (phal_uart_adapter_t puart_adapter, char *data)
{
    return hal_uart_stubs.hal_uart_rgetc (puart_adapter, data);
}

#define phal_uart_rgetc         (hal_uart_stubs.hal_uart_rgetc)

/**
 *  @brief To receives a block of data by the polling mode.
 *
 *  @param[in]   puart_adapter  The UART adapter.
 *  @param[out]  prx_buf  The buffer for the data receiving.
 *  @param[in]   len  The length of data in byte to be received.
 *  @param[in]   timeout_ms  The maximum period(in ms) for the waiting of data receiving.
 *
 *  @returns     The length, in byte, of data has been received. Return 0 for the case of
 *               UART RX state is busy (previous receiving not didn't finished yet)
 */
__STATIC_INLINE uint32_t hal_uart_recv (phal_uart_adapter_t puart_adapter, u8 *prx_buf, u32 len, u32 timeout_ms)
{
    return hal_uart_stubs.hal_uart_recv (puart_adapter, prx_buf, len, timeout_ms);
}

/**
 *  @brief To receive a block of data by the interrupt mode.
 *         It read the RX FIFO in the RX FIFO level interrupr handler.
 *         This function returns without waiting of data receiving done.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[out] prx_buf The buffer for the data receiving.
 *  @param[in]  len  The length of data, in byte, are going to receive.
 *
 *  @return     HAL_OK: function execution OK.
 *  @return     HAL_BUSY: UART RX is in busy state, previous receiving is not finished yet.
 *  @return     HAL_ERR_PARA: Input arguments are invalid.
 */
__STATIC_INLINE hal_status_t hal_uart_int_recv (phal_uart_adapter_t puart_adapter, u8 *prx_buf, u32 len)
{
    return hal_uart_stubs.hal_uart_int_recv (puart_adapter, prx_buf, len);
}

/**
 *  @brief To abort an on going UART RX transfer.
 *
 *  @param[in]  puart_adapter The UART adapter.
 *
 *  @return     value 0  There is no on going UART data receiving.
 *  @return     value > 0  The data length, in byte, has been received.
 */
__STATIC_INLINE uint32_t hal_uart_recv_abort (phal_uart_adapter_t puart_adapter)
{
    return hal_uart_stubs.hal_uart_recv_abort (puart_adapter);
}

//u8 hal_uart_get_imr (phal_uart_adapter_t puart_adapter);
//void hal_uart_set_imr (phal_uart_adapter_t puart_adapter);

/**
 *  @brief Controls the RTS signal.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  rts_ctrl  The RTS signal control value.
 *                        - 1: the RTS signal will go low(notify peer can start to send data).
 *                        - 0: the RTS signal will go high(notify peer stop data sending).
 *
 *  @returns    void.
 */
__STATIC_INLINE
void hal_uart_set_rts (phal_uart_adapter_t puart_adapter, BOOLEAN rts_ctrl)
{
    puart_adapter->base_addr->mcr_b.rts = (rts_ctrl==0)?0:1;
}

/**
 *  @brief To pause/resume the TX by control the internal CTS.
 *         It can be used to implement the software flow control(XOn/XOff).
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  cts_ctrl The internal CTS signal control value.
 *                         - 1: the TX will be paused just like the external CTS goes high.
 *                         - 0: the TX will be resumed just like the externel CTS goes low.
 *
 *  @returns    void.
 */
__STATIC_INLINE
void hal_uart_tx_pause (phal_uart_adapter_t puart_adapter, BOOLEAN cts_ctrl)
{
    puart_adapter->base_addr->mcr_b.sw_cts = (cts_ctrl==0)?0:1;
}

/**
 *  @brief To register a common interrupt handler for all UART ports.
 *
 *  @param[in] handler  The interrupt handle function.
 *
 *  @returns    void.
 */
__STATIC_INLINE
void hal_uart_reg_comm_irq (irq_handler_t handler)
{
    hal_uart_stubs.hal_uart_reg_comm_irq (handler);
}

/**
 *  @brief To register a interrupt handler for the given UART port.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  handler  The interrupt handle function.
 *
 *  @returns void
 */
__STATIC_INLINE
void hal_uart_reg_irq (phal_uart_adapter_t puart_adapter, irq_handler_t handler)
{
    hal_uart_stubs.hal_uart_reg_irq (puart_adapter, handler);
}

/**
 *  @brief To un-register the interrupt handler of the given UART port.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *
 *  @returns    void
 */
__STATIC_INLINE
void hal_uart_unreg_irq (phal_uart_adapter_t puart_adapter)
{
    hal_uart_stubs.hal_uart_unreg_irq (puart_adapter);
}

/**
 *  @brief Initials a UART adapter contents by a given UART port configuration structure.
 *
 *  @param[in]  puart_adapter The UART adapter to be initialed.
 *  @param[in]  uart_idx  The UART index. The value can be 0 ~ 2.
 *  @param[in]  pconfig  The UART adapter configuration to be used to initial this UART adapter.
 *                       If this value is NULL, a default configuration will be applied for
 *                       the initialization.
 *
 *  @returns    void
 */
__STATIC_INLINE
void hal_uart_adapter_init (phal_uart_adapter_t puart_adapter, uint8_t uart_idx, phal_uart_defconfig_t pconfig)
{
    hal_uart_stubs.hal_uart_adapter_init (puart_adapter, uart_idx, pconfig);
}

/**
 *  @brief Hooks a callback function for the UART line status error interrupt.
 *         This function will enable the line status error interrupt.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  pcallback  The callback function.
 *  @param[in]  pdata  The argument of the callback function. It is an application
 *                     priviate data to be passed by this callback function.
 *
 *  @returns    void
 */
__STATIC_INLINE
void hal_uart_line_sts_hook (phal_uart_adapter_t puart_adapter, uart_lsr_callback_t pcallback, void *pdata)
{
    hal_uart_stubs.hal_uart_line_sts_hook (puart_adapter, pcallback, pdata);
}

/**
 *  @brief Hooks a callback function for data TX done interrupt.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  pcallback  The callback function.
 *  @param[in]  id  The argument of the callback function. It is an application
 *                    priviate data to be passed by this callback function.
 *  @param[in]  event  The argument of the callback function. It is an application
 *                    priviate data to be passed by this callback function.
 *
 *  @returns void
 */
__STATIC_INLINE
void hal_uart_txtd_hook (phal_uart_adapter_t puart_adapter, uart_irq_callback_t pcallback, uint32_t id, uint32_t event)
{
    hal_uart_stubs.hal_uart_txtd_hook(puart_adapter, pcallback, id, event);
}

/**
 *  @brief Hooks a callback function for RX data ready interrupt.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  pcallback  The callback function.
 *  @param[in]  id  The argument of the callback function. It is an application
 *                    priviate data to be passed by this callback function.
 *  @param[in]  event  The argument of the callback function. It is an application
 *                    priviate data to be passed by this callback function.
 *
 *
 *  @returns void
 */
__STATIC_INLINE
void hal_uart_rxind_hook (phal_uart_adapter_t puart_adapter, uart_irq_callback_t pcallback, uint32_t id, uint32_t event)
{
    hal_uart_stubs.hal_uart_rxind_hook (puart_adapter, pcallback, id, event);
}

/**
 *  @brief Hooks a callback function for interrupt mode or DMA mode
 *         data transmission finished event.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  pcallback  The callback function.
 *  @param[in]  parg  The argument of the callback function. It is an application
 *                    priviate data to be passed by this callback function.
 *
 *  @returns    void
 */
__STATIC_INLINE
void hal_uart_txdone_hook (phal_uart_adapter_t puart_adapter, uart_callback_t pcallback, void *parg)
{
    hal_uart_stubs.hal_uart_txdone_hook (puart_adapter, pcallback, parg);
}

/**
 *  @brief Hooks a callback function for interrupt mode or DMA mode
 *         data receiving finished event.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  pcallback  The callback function.
 *  @param[in]  parg  The argument of the callback function. It is an application
 *                    priviate data to be passed by this callback function.
 *
 *  @returns    void
 */
__STATIC_INLINE
void hal_uart_rxdone_hook (phal_uart_adapter_t puart_adapter, uart_callback_t pcallback, void *parg)
{
    hal_uart_stubs.hal_uart_rxdone_hook (puart_adapter, pcallback, parg);
}

/**
 *  @brief Setups and enable the UART RX match filter.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  pattern  The match pattern.
 *                       - [7:0]: byte0.
 *                       - [15:8]: byte1.
 *  @param[in]  mask  The mask of the match pattern.
 *  @param[in]  mask_en The mask enable control.
 *                      value = 0   mask is disabled.
 *                      value > 0   mask is enabled.
 *
 *  @returns void
 */
__STATIC_INLINE
void hal_uart_set_rx_filter_pattern (phal_uart_adapter_t puart_adapter, uint32_t pattern,
                                     uint32_t mask, uint32_t mask_en)
{
    hal_uart_stubs.hal_uart_set_rx_filter_pattern (puart_adapter, pattern, mask, mask_en);
}

/**
 *  @brief Configures the UART RX match filter option.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  pattern_len  The match pattern length:
 *                             - 1: 1 byte.
 *                             - 2: 2 bytes.
 *  @param[in]  match_op  The match filter operation mode
 *                          - 0: AND.
 *                          - 1: OR.
 *                          - 2: XOR.
 *
 *  @returns    void
 */
__STATIC_INLINE
void hal_uart_set_rx_filter_op (phal_uart_adapter_t puart_adapter, uint32_t pattern_len, uint32_t match_op)
{
    hal_uart_stubs.hal_uart_set_rx_filter_op (puart_adapter, pattern_len, match_op);
}

/**
 *  @brief Configures the UART RX match filter timeout value.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  timeout_ms  The period to wait next incoming RX byte, unit is mini-second.
 *  @param[in]  callback  The callback function to handle the timeout event of RX filter wait next byte.
 *  @param[in]  cb_arg  The argument of the RX filter timeout callback function. It is an application
 *                      priviate data to be passed by this callback function.
 *
 *  @returns void
 */
__STATIC_INLINE
void hal_uart_set_rx_filter_timeout (phal_uart_adapter_t puart_adapter, uint32_t timeout_ms,
                                     uart_callback_t callback, void *cb_arg)
{
    hal_uart_stubs.hal_uart_set_rx_filter_timeout (puart_adapter, timeout_ms, callback, cb_arg);
}

/**
 *  @brief Enables the UART RX match filter. The RX filter should be configured
 *         before enable it. The software need to re-enable the RX filter for every
 *         RX data matching. The hardware will disable this RX filter on a matched event hit.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  callback  The callback function to handle the RX filter matched event.
 *  @param[in]  cb_arg  The argument of the RX filter matched callback function.
 *                      It is an application priviate data to be passed by this callback function.
 *
 *  @returns    void
 */
__STATIC_INLINE
void hal_uart_rx_filter_en (phal_uart_adapter_t puart_adapter, uart_callback_t callback, void *cb_arg)
{
    hal_uart_stubs.hal_uart_rx_filter_en (puart_adapter, callback, cb_arg);
}

/**
 *  @brief Disables the UART RX match filter.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *
 *  @returns    void
 */
__STATIC_INLINE
void hal_uart_rx_filter_dis (phal_uart_adapter_t puart_adapter)
{
    hal_uart_stubs.hal_uart_rx_filter_dis (puart_adapter);
}

/**
 *  @brief Resets the UART receiver hardware state machine.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *
 *  @returns    void
 */
__STATIC_INLINE
void hal_uart_reset_receiver (phal_uart_adapter_t puart_adapter)
{
    hal_uart_stubs.hal_uart_reset_receiver (puart_adapter);
}

/**
 *  @brief Setups the TX FIFO low level. The TX FIFO low interrupt will be triggered
 *         when the the data count in the TX FIFO is lower than the low level.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  low_level  The low level selection:
 *                           - value 0: 4 bytes.
 *                           - value 1: 8 bytes.
 *  @returns    void
 */
__STATIC_INLINE void hal_uart_set_tx_fifo_level (phal_uart_adapter_t puart_adapter, uart_tx_fifo_level_t level)
{
    hal_uart_stubs.hal_uart_set_tx_fifo_level (puart_adapter, level);
}

/**
 *  @brief Configures the RX FIFO high level. The RX FIFO high interrupt will be triggered
 *         when the the data count in the RX FIFO is higher than the configured level.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  level  The high level selection:
 *                     - 0: 1 byte.
 *                     - 1: 8 bytes.
 *                     - 2: 16 bytes.
 *                     - 3: 28 bytes.
 *
 *  @returns    void
 */
__STATIC_INLINE void hal_uart_set_rx_fifo_level (phal_uart_adapter_t puart_adapter, uint32_t level)
{
    hal_uart_stubs.hal_uart_set_rx_fifo_level (puart_adapter, level);
}

/**
 *  @brief To read the number of bytes in the RX FIFO.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *
 *  @returns    The number of data bytes in the RX FIFO.
 */
__STATIC_INLINE uint32_t hal_uart_read_rx_fifo_level (phal_uart_adapter_t puart_adapter)
{
	return (puart_adapter->base_addr->rflvr);
}

/**
 *  @brief Enables the RX Idle timeout function. The RX idle timer will be start/re-start on
 *         every new byte receiving. If there is no new data be received in the given period
 *         the callback function will be called.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  timeout_us  The timeout period, in micro-second.
 *  @param[in]  pcallback  The call back function.
 *  @param[in]  parg  The argument for the callback function. It is an application priviate
 *                    data to be passed by this callback function.
 *
 *  @returns    void
 */
__STATIC_INLINE void hal_uart_rx_idle_timeout_en (phal_uart_adapter_t puart_adapter, uint32_t timeout_us,
                                   uart_callback_t pcallback, void *parg)
{
    hal_uart_stubs.hal_uart_rx_idle_timeout_en (puart_adapter, timeout_us, pcallback, parg);
}

/**
 *  @brief Disables the RX idle timeout function.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *
 *  @returns    void
 */
__STATIC_INLINE void hal_uart_rx_idle_timeout_dis (phal_uart_adapter_t puart_adapter)
{
    hal_uart_stubs.hal_uart_rx_idle_timeout_dis (puart_adapter);
}

/**
 *  @brief Initials the UART group common adapter.
 *         This function must be called before any UART port function call.
 *
 *  @param[in]  puart_comm_adapter  The UART group common adapter.
 *
 *  @returns    void.
 */
__STATIC_INLINE
void hal_uart_comm_init (hal_uart_group_adapter_t *puart_comm_adapter)
{
    hal_uart_stubs.hal_uart_comm_init (puart_comm_adapter);
    hal_irq_enable (UART_IRQn);
}
/**
 *  @brief To enter a critical code section, mainly it
 *         disable the UART interrupt to prevent race condition.
 *
 *  @returns void
 */
__STATIC_INLINE
void hal_uart_enter_critical (void)
{
    hal_uart_stubs.hal_uart_enter_critical ();
}

/**
 *  @brief To exit a critical code section, it will re-enable the UART interrupt
 *         only when the exiting critical section is the top level.
 *
 *  @returns void
 */
__STATIC_INLINE
void hal_uart_exit_critical (void)
{
    hal_uart_stubs.hal_uart_exit_critical ();
}

/**
 *  @brief To enable or disable an UART device. It controls the
 *         hardware function, bus domain, PCLK and SCLK of the
 *         specified UART.
 *
 *  @param[in]  uart_idx The UART index. The value can be 0 .. 2.
 *  @param[in]  en  Enable control:
 *                    - 0: Fully disable the UART hardware(function and clock).
 *                    - 1: Fully enable the UART hardware(function and clock).
 *
 *  @returns    void
 */
__STATIC_INLINE
void hal_uart_en_ctrl (uint8_t uart_idx, BOOL en)
{
    hal_uart_stubs.hal_uart_en_ctrl (uart_idx, en);
}

/** 
 *  @brief Query the UART index by a given pin name and its pin function.
 *
 *  @param[in]  pin_name  The pin name be used to query the corresponding UART index.
 *  @param[in]  pin_type  The function type of the pin.
 *
 *  @returns    The UART index. If the given pin name didn't map to a valid UART, the return value is 0xFF.
 */
__STATIC_INLINE
uint8_t hal_uart_pin_to_idx (uint8_t pin_name, uart_pin_func_t pin_type)
{
    return hal_uart_stubs.hal_uart_pin_to_idx (pin_name, pin_type);
}

hal_status_t hal_uart_init (phal_uart_adapter_t puart_adapter, uint8_t tx_pin, uint8_t rx_pin,
                            phal_uart_defconfig_t pconfig);
void hal_uart_deinit (phal_uart_adapter_t puart_adapter);
hal_status_t hal_uart_set_flow_control (phal_uart_adapter_t puart_adapter, uint32_t flow_ctrl);
hal_status_t hal_uart_rx_gdma_init(phal_uart_adapter_t puart_adapter, phal_gdma_adaptor_t pgdma_chnl);
hal_status_t hal_uart_rx_gdma_deinit(phal_uart_adapter_t puart_adapter);
hal_status_t hal_uart_tx_gdma_init(phal_uart_adapter_t puart_adapter, phal_gdma_adaptor_t pgdma_chnl);
hal_status_t hal_uart_tx_gdma_deinit(phal_uart_adapter_t puart_adapter);
hal_status_t hal_uart_dma_recv (phal_uart_adapter_t puart_adapter, uint8_t *prx_buf, uint32_t len);
hal_status_t hal_uart_dma_send (phal_uart_adapter_t puart_adapter, uint8_t *ptx_buf, uint32_t len);

/** @} */ /* End of group hs_hal_uart */

#ifdef  __cplusplus
}
#endif


#endif  // end of "#define _HAL_UART_H_"

