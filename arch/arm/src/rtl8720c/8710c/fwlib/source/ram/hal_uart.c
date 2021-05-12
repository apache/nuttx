/**************************************************************************//**
 * @file     hal_uart.c
 * @brief    This UART HAL API functions.
 *
 * @version  V1.00
 * @date     2016-07-15
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
#include "cmsis.h"
#include "hal_uart.h"
#include "hal_gdma.h"
#include "hal_pinmux.h"
#include "hal_cache.h"
#include "hal_gpio.h"
#include "hal_timer.h"
#include "memory.h"
#include "hal_sys_ctrl.h"

#if CONFIG_UART_EN

extern hal_status_t hal_gpio_pull_ctrl (uint32_t pin_name, pin_pull_type_t pull_type);

uint8_t hal_uart_check_uart_id (uint8_t tx_pin, uint8_t rx_pin)
{
    uint8_t uart_idx = NONESET_UART_IDX;

    if (tx_pin != PIN_NC) {
        uart_idx = hal_uart_stubs.hal_uart_pin_to_idx (tx_pin, UART_Pin_TX);
        if (uart_idx >= MAX_UART_PORT) {
            DBG_UART_ERR("%s: pin(0x%x) is not for UART TX\r\n", __func__, tx_pin);
            return NONESET_UART_IDX;
        }
    }

    if (rx_pin != PIN_NC) {
        if (uart_idx != 0xFF) {
            if (uart_idx != hal_uart_stubs.hal_uart_pin_to_idx (rx_pin, UART_Pin_RX)) {
                DBG_UART_ERR("%s:tx_pin(0x%x) & rx_pin(0x%x) is not on the same UART\r\n", __func__, tx_pin, rx_pin);
                return NONESET_UART_IDX;
            }
        } else {
            uart_idx = hal_uart_stubs.hal_uart_pin_to_idx (rx_pin, UART_Pin_RX);
            if (uart_idx >= MAX_UART_PORT) {
                DBG_UART_ERR("%s: pin(0x%x) is not for UART RX\r\n", __func__, rx_pin);
                return NONESET_UART_IDX;
            }
        }
    }

    if (uart_idx >= MaxUartNum) {
        return NONESET_UART_IDX;
    }
    return uart_idx;
}

/**
 * @addtogroup hs_hal_uart UART
 * @{
 * @brief The UART HAL APIs.
 */

/**
 *  @brief To initial a UART port adapter. This function must be called before any UART port
 *         operation. This function will do:
 *           - enable the UART hardware.
 *           - register the interrupt handler.
 *           - configures the pin mux.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  tx_pin   The UART TX pin name.
 *  @param[in]  rx_pin   The UART RX pin name.
 *  @param[in]  pin_sel  The pin mux selection.
 *  @param[in]  pconfig  The extra UART port setting for the initial configuration.
 *                       This is an UART adapter initial value. If this value is not NULL,
 *                       the initialization function will initial the new UART adapter by
 *                       this initial value. And also will do further configure, configures
 *                       the bard rate, hardware flow control and the frame format.
 *
 *  @return     HAL_ERR_PARA:  Input arguments are invalid.
 *  @return     HAL_OK:  UART port initialization OK.
 */
hal_status_t hal_uart_init (phal_uart_adapter_t puart_adapter, uint8_t tx_pin, uint8_t rx_pin,
                            phal_uart_defconfig_t pconfig)
{
    hal_status_t ret;
    uint8_t uart_idx;
    
    uart_idx = hal_uart_check_uart_id(tx_pin,rx_pin);

    if ((uart_idx <= Uart2) && (rx_pin != PIN_NC)) {
        // RX Pin pull-high to prevent this folating on this pin
        hal_gpio_pull_ctrl (rx_pin, Pin_PullUp);
        hal_delay_us(4);
    }

    if ((uart_idx < MaxUartNum) && (Uart0 == uart_idx)) {
        hal_syson_wakeup_uart_func_reset();
    }

    ret = hal_uart_stubs.hal_uart_init (puart_adapter, tx_pin, rx_pin, pconfig);
    if (ret == HAL_OK) {
        uart_idx = puart_adapter->uart_idx;
        if (uart_idx <= Uart2) {
            // only UART0,1,2 has real IO pin
            if (tx_pin != PIN_NC) {
                ret = hal_pinmux_register (tx_pin, (PID_UART0+uart_idx));
            }

            if (rx_pin != PIN_NC) {
                ret |= hal_pinmux_register (rx_pin, (PID_UART0+uart_idx)); 
            }
        }
    } else {
        if ((uart_idx <= Uart2) && (rx_pin != PIN_NC)) {
            hal_gpio_pull_ctrl (rx_pin, Pin_PullNone);
        }
    }
    return ret;
}

/**
 *  @brief Disable the given UART port. It will do:
 *           - disable UART hardware function.
 *           - disable UART GDMA channel.
 *           - disable UART pins.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *
 *  @returns void
 */
void hal_uart_deinit (phal_uart_adapter_t puart_adapter)
{
    uint32_t uart_idx = puart_adapter->uart_idx;

    hal_uart_stubs.hal_uart_deinit(puart_adapter);
    if (uart_idx <= Uart2) {
        if (puart_adapter->tx_pin != PIN_NC) {
            hal_pinmux_unregister (puart_adapter->tx_pin, (PID_UART0+uart_idx));
        }
        
        if (puart_adapter->rx_pin != PIN_NC) {
            hal_pinmux_unregister (puart_adapter->rx_pin, (PID_UART0+uart_idx));
            hal_gpio_pull_ctrl (puart_adapter->rx_pin, Pin_PullNone);
        }
        
        if (puart_adapter->rts_pin != PIN_NC) {
            hal_pinmux_unregister (puart_adapter->rts_pin, (PID_UART0+uart_idx));
        }
        
        if (puart_adapter->cts_pin != PIN_NC) {
            hal_pinmux_unregister (puart_adapter->cts_pin, (PID_UART0+uart_idx));
        }
    }
}

/**
 *  @brief Configures the UART hardware auto flow-control setting.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  flow_ctrl  The flow control setting.
 *                           - 0: no hardware flow control.
 *                           - 1: enable RX (RTS) flow control.
 *                           - 2: enable TX (CTS) flow control.
 *                           - 3: enable RTS and CTS hardware flow control.
 *
 *  @returns    Always return HAL_OK
 */
hal_status_t hal_uart_set_flow_control (phal_uart_adapter_t puart_adapter, uint32_t flow_ctrl)
{
    uint8_t uart_idx = puart_adapter->uart_idx;
    hal_status_t ret;

    ret = hal_uart_stubs.hal_uart_set_flow_control (puart_adapter, flow_ctrl);

    if (uart_idx <= Uart2) {
        switch (flow_ctrl) {
            case UartFlowCtlNone:
                if (puart_adapter->rts_pin != PIN_NC) {
                    ret |= hal_pinmux_unregister (puart_adapter->rts_pin, (PID_UART0+uart_idx));
                    puart_adapter->rts_pin = PIN_NC;
                }
                
                if (puart_adapter->cts_pin != PIN_NC) {
                    ret |= hal_pinmux_unregister (puart_adapter->cts_pin, (PID_UART0+uart_idx));
                    puart_adapter->cts_pin = PIN_NC;
                }
                break;
                
            case UartFlowCtlRTSCTS:
                ret |= hal_pinmux_register (puart_adapter->rts_pin, (PID_UART0+uart_idx));
                ret |= hal_pinmux_register (puart_adapter->cts_pin, (PID_UART0+uart_idx));
                break;
                
            case UartFlowCtlRTS:
                ret |= hal_pinmux_register (puart_adapter->rts_pin, (PID_UART0+uart_idx));
                if (puart_adapter->cts_pin != PIN_NC) {
                    ret |= hal_pinmux_unregister (puart_adapter->cts_pin, (PID_UART0+uart_idx));
                    puart_adapter->cts_pin = PIN_NC;
                }
                break;
                
            case UartFlowCtlCTS:
                ret |= hal_pinmux_register (puart_adapter->cts_pin, (PID_UART0+uart_idx));
                if (puart_adapter->rts_pin != PIN_NC) {
                    ret |= hal_pinmux_unregister (puart_adapter->rts_pin, (PID_UART0+uart_idx));
                    puart_adapter->rts_pin = PIN_NC;
                }
                break;
                
            default:
                break;
        }
    }
    
    return ret;
}

/**
 *  @brief To initial a GDMA channel for the UART TX DMA mode transfer.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  pgdma_chnl The GDMA channel adapter. It is use to control
 *              the GDMA channel transfer.
 *
 *  @return     HAL_ERR_PARA:  Input arguments are invalid.
 *  @return     HAL_OK:  GDMA channel initialization OK.
 */
hal_status_t hal_uart_tx_gdma_init(phal_uart_adapter_t puart_adapter, phal_gdma_adaptor_t pgdma_chnl)
{
    hal_status_t ret;

	memset((void *)pgdma_chnl, 0, sizeof(hal_gdma_adaptor_t));
    ret = hal_gdma_chnl_alloc (pgdma_chnl);    // default no-multiple block support

    if (ret == HAL_OK) {
        ret = hal_uart_stubs.hal_uart_tx_gdma_init(puart_adapter, pgdma_chnl);
        if (ret == HAL_OK) {
            puart_adapter->dcache_clean_by_addr = hal_cache_stubs.dcache_clean_by_addr;
            hal_gdma_chnl_init (pgdma_chnl);
        } else {
            DBG_UART_ERR("hal_uart_tx_gdma_init: GDMA init failed(%d)\r\n", ret);
            hal_gdma_chnl_free (pgdma_chnl);
        }
    } else {
        DBG_UART_ERR("hal_uart_tx_gdma_init: GDMA channel allocate failed(%d)\r\n", ret);
    }

    return ret;
}

/**
 *  @brief To de-initial the UART TX GDMA channel.
 *         Also will disable the UART TX DMA transfer mode.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *
 *  @return     HAL_ERR_PARA:  Input arguments are invalid.
 *  @return     HAL_OK:  GDMA channel de-initialization OK.
 */
hal_status_t hal_uart_tx_gdma_deinit(phal_uart_adapter_t puart_adapter)
{
    hal_gdma_chnl_free (puart_adapter->ptx_gdma);
    return hal_uart_stubs.hal_uart_tx_gdma_deinit(puart_adapter);
}

/**
 *  @brief To initial a GDMA channel for the UART RX DMA mode transfer.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  pgdma_chnl The GDMA channel adapter. It is use to control
 *              the GDMA channel transfer.
 *
 *  @return     HAL_ERR_PARA:  Input arguments are invalid.
 *  @return     HAL_OK:  GDMA channel initialization OK.
 */
hal_status_t hal_uart_rx_gdma_init(phal_uart_adapter_t puart_adapter, phal_gdma_adaptor_t pgdma_chnl)
{
    hal_status_t ret;

	memset((void *)pgdma_chnl, 0, sizeof(hal_gdma_adaptor_t));
    ret = hal_gdma_chnl_alloc (pgdma_chnl);    // default no-multiple block support

    if (ret == HAL_OK) {
        ret = hal_uart_stubs.hal_uart_rx_gdma_init(puart_adapter, pgdma_chnl);
        if (ret == HAL_OK) {
            puart_adapter->dcache_invalidate_by_addr = hal_cache_stubs.dcache_invalidate_by_addr;
            hal_gdma_chnl_init (pgdma_chnl);
        } else {
            DBG_UART_ERR("hal_uart_rx_gdma_init: GDMA init failed(%d)\r\n", ret);
            hal_gdma_chnl_free (pgdma_chnl);
        }
    } else {
        DBG_UART_ERR("hal_uart_rx_gdma_init: GDMA channel allocate failed(%d)\r\n", ret);
    }

    return ret;
}

/**
 *  @brief To de-initial the UART RX GDMA channel.
 *         Also will disable the UART RX DMA transfer mode.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *
 *  @return     HAL_ERR_PARA:  Input arguments are invalid.
 *  @return     HAL_OK:  GDMA channel de-initialization OK.
 */
hal_status_t hal_uart_rx_gdma_deinit(phal_uart_adapter_t puart_adapter)
{
    hal_gdma_chnl_free (puart_adapter->prx_gdma);
    return hal_uart_stubs.hal_uart_rx_gdma_deinit(puart_adapter);
}

/**
 *  @brief To receive a block of data by the DMA mode.
 *         This function returns without waiting of data receiving done.
 *
 *  @param[in]  puart_adapter The UART adapter.
 *  @param[out] prx_buf The buffer for the data receiving.
 *  @param[in]  len  The length of data, in byte, are going to receive.
 *
 *  @return     HAL_OK: function execution OK.
 *  @return     HAL_BUSY: UART RX is in busy state, previous receiving is not finished yet.
 *  @return     HAL_ERR_PARA: Input arguments are invalid.
 *  @return     HAL_NO_RESOURCE: Multiple-block DMA channel allocation failed.
 */
hal_status_t hal_uart_dma_recv (phal_uart_adapter_t puart_adapter, uint8_t *prx_buf, uint32_t len)
{
    hal_gdma_adaptor_t *pgdma_chnl = puart_adapter->prx_gdma;
//    hal_status_t ret;
//    volatile uint32_t gdma_idx;

    if (pgdma_chnl == NULL) {
        DBG_UART_ERR("hal_uart_dma_recv: No GDMA Chnl\r\n");
        return HAL_NO_RESOURCE;
    }

    if (len > MAX_DMA_BLOCK_SIZE) {
        #if 0   // 8710C GDMA has no multi-block
        if (len <= MAX_DMA_BLOCK_SIZE*32) {
            // Need to use multiple block DMA
            if (pgdma_chnl->ch_num < 4) {
                // Current GDMA Channel didn't support multiple block DMA, re-allocate another one
                gdma_idx = pgdma_chnl->gdma_index;  // backup old GDMA index
                hal_gdma_chnl_free (pgdma_chnl);
                ret = hal_gdma_chnl_alloc (pgdma_chnl, MultiBlkEn);
                if (ret != HAL_OK) {
                    puart_adapter->prx_gdma = NULL;
                    DBG_UART_ERR("hal_uart_dma_recv: Err: re-allocate multiple block DMA channel failed(%d)\r\n", ret);
                    return ret;
                } else {
                    DBG_UART_INFO("hal_uart_dma_recv: re-allocate GDMA %u chnl %u\r\n", pgdma_chnl->gdma_index, pgdma_chnl->ch_num );
                    pgdma_chnl->pgdma_ch_lli = &uart_rx_gdma_ch_lli[puart_adapter->uart_idx][0];
                    hal_gdma_chnl_init (pgdma_chnl);
                }

                // Update GDMA handshake bit and IRQ handler(since may use different GDMA HW)
                if (gdma_idx != pgdma_chnl->gdma_index) {
                    // allocated to different GDMA HW, update the handshake bit
                    hal_gdma_handshake_init (pgdma_chnl, pgdma_chnl->gdma_cfg.src_per);
                }
                hal_gdma_irq_reg (pgdma_chnl, (irq_handler_t)hal_uart_stubs.uart_rx_dma_irq_handler, puart_adapter);
            }

        }else
        #endif
        {
            DBG_UART_ERR("hal_uart_dma_recv: Err: RX Len(%lu) too big\n", len);
            return HAL_ERR_PARA;
        }
    }
    return hal_uart_stubs.hal_uart_dma_recv (puart_adapter, prx_buf, len);
}

/**
 *  @brief To send a block of data by the DMA transmission mode.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  ptx_buf  The buffer of data to be transmitted.
 *  @param[in]  len  The length of data in bytes to be transmitted.
 *
 *  @return     HAL_OK: function execution OK.
 *  @return     HAL_BUSY: UART TX is in busy state, previous transmission is not finished yet.
 *  @return     HAL_ERR_PARA: Input arguments are invalid.
 *  @return     HAL_NO_RESOURCE: Multiple-block DMA channel allocation failed.
 */
hal_status_t hal_uart_dma_send (phal_uart_adapter_t puart_adapter, uint8_t *ptx_buf, uint32_t len)
{
    hal_gdma_adaptor_t *pgdma_chnl = puart_adapter->ptx_gdma;
    uint32_t block_size;
//    hal_status_t ret;
//    volatile uint32_t gdma_idx;

    if (pgdma_chnl == NULL) {
        DBG_UART_ERR("hal_uart_dma_send: No GDMA Chnl\r\n");
        return HAL_NO_RESOURCE;
    }

    if (((len & 0x03)==0) && (((uint32_t)(ptx_buf) & 0x03)==0)) {
        // 4-bytes aligned, move 4 bytes each transfer
        block_size = len >> 2;
    } else {
        block_size = len;
    }

    if (block_size > MAX_DMA_BLOCK_SIZE) {
        #if 0   // 8710C GDMA has no multi-block
        if (block_size <= MAX_DMA_BLOCK_SIZE*32){
            // Need to use multiple block DMA
            if (pgdma_chnl->ch_num < 4) {
                // Current GDMA Channel didn't support multiple block DMA, re-allocate another one
                DBG_UART_INFO("hal_uart_dma_send: re-allocate GDMA chnl to support Multi-Blk transfer\r\n");
                gdma_idx = pgdma_chnl->gdma_index;  // backup old GDMA index
                hal_gdma_chnl_free (pgdma_chnl);
                ret = hal_gdma_chnl_alloc (pgdma_chnl, MultiBlkEn);

                if (ret != HAL_OK) {
                    puart_adapter->ptx_gdma = NULL;
                    DBG_UART_ERR("hal_uart_dma_send: Err: re-allocate multiple block DMA channel failed(%d)\r\n", ret);
                    return ret;
                } else {
                    DBG_UART_INFO("hal_uart_dma_send: re-allocated GDMA %u chnl %u\r\n", pgdma_chnl->gdma_index, pgdma_chnl->ch_num );
                    pgdma_chnl->pgdma_ch_lli = &uart_tx_gdma_ch_lli[puart_adapter->uart_idx][0];
                    hal_gdma_chnl_init (pgdma_chnl);
                }

                // Update GDMA handshake bit and IRQ handler(since may use different GDMA HW)
                if (gdma_idx != pgdma_chnl->gdma_index) {
                    // allocated to different GDMA HW, update the handshake bit
                    hal_gdma_handshake_init (pgdma_chnl, pgdma_chnl->gdma_cfg.dest_per);
                }
                hal_gdma_irq_reg (pgdma_chnl, (irq_handler_t)hal_uart_stubs.uart_tx_dma_irq_handler, puart_adapter);
            }
        }else 
        #endif
        {
            DBG_UART_ERR("hal_uart_dma_send: Err: TX length too big(%lu)\r\n", len);
            return HAL_ERR_PARA;
        }
    }

    return hal_uart_stubs.hal_uart_dma_send (puart_adapter, ptx_buf, len);
}

/** @} */ /* End of group hs_hal_uart */

#endif  // end of "#if CONFIG_UART_EN"

