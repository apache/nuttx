/**************************************************************************//**
 * @file     hal_sdio_dev.h
 * @brief    This file define SDIO device HAL macro and lower layer
 *           HAL functions wrapper.
 *
 * @version  V1.00
 * @date     2017-09-20
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

#ifndef _HAL_SDIO_DEV_H_
#define _HAL_SDIO_DEV_H_

#ifdef  __cplusplus
extern "C"
{
#endif

#include "cmsis.h"
#include "rtl8710c_sdio_dev.h"

/**
 * @addtogroup hs_hal_sdio_dev
 * @{
 */

#ifndef CONFIG_INIC_EN
/// Enables the SDIO device HAL code works as an SDIO INIC. mode.
#define CONFIG_INIC_EN              0
#endif
#if CONFIG_INIC_EN
/// Enables the SDIO device HAL to use the WLan driver SKB as the TX buffer directly to improve the throughput.
#define CONFIG_INIC_SKB_TX          1
/// Enables the SDIO device HAL to use the WLan driver SKB as the RX buffer directly to improve the throughput.
#define CONFIG_INIC_SKB_RX          1
#endif

#ifndef SDIO_API_DEFINED
/// Enables the SDIO device HAL API for upper layer application to TX/RX data over the SDIO bus.
#define SDIO_API_DEFINED            0
#endif

#ifndef SDIO_NO_RTOS
/// Enables the SDIO device HAL driver running without RTOS.
#define SDIO_NO_RTOS                0
#endif

/// Defines the mode of the SDIO device works as an iNIC solo or works a SDIO iNIC with other peripheral devices.
#define PURE_SDIO_INIC              0

/// Defines the stack size of the SDIO Device TX task
#define SDIO_TX_TASK_STACK_SIZE     (2048)
/// Defines the stack size of the SDIO Device RX task
#define SDIO_RX_TASK_STACK_SIZE     (1024)

/// Defines the SDIO task priority, it can be 0(lowest) ~ configMAX_PRIORITIES-1(highest)
#define SDIO_TX_TASK_PRIORITY          2
#define SDIO_RX_TASK_PRIORITY          1

/// Defines the queue size of the SDIO device message queue.
#define SDIO_MSG_QUEUE_SIZE         10

/// Enables the support of a SDIO RX (SDIO device to SDIO host) packet size over 16K bytes.
/// If true, a packet transmission will use multiple RX_BD.
#define SDIO_RX_PKT_SIZE_OVER_16K   0

#if CONFIG_INIC_EN
//TX BD setting
/// Number of TX BD
#define SDIO_TX_BD_NUM                  20
/// TX buffser size, 1514 + 24(size of header)
#define SDIO_TX_BD_BUF_SIZE             1540

//RX BD setting
/// the threshold to trigger the interrupt when free RX BD over this value.
#define RX_BD_FREE_TH                   4
/// RX buffer size, 1514 + 24(size of header header)
#define SDIO_RX_BD_BUF_SIZE             1540
/// Number of RX BD, to make 32K of bus aggregation will needs 22 RX_BDs at least
#define SDIO_RX_BD_NUM                  32
/// Number of RX packet handler
#define SDIO_RX_PKT_NUM                 128

#else
//TX BD setting
/// Number of TX BD
#define SDIO_TX_BD_NUM                  24
/// TX buffser size
#define SDIO_TX_BD_BUF_SIZE             (2048+32)
/// Number of TX packet handler
#define SDIO_TX_PKT_NUM                 128

//RX BD setting
/// the threshold to trigger the interrupt when free RX BD over this value.
#define RX_BD_FREE_TH                   4
/// RX buffer size
#define SDIO_RX_BD_BUF_SIZE             2048
/// Number of RX BD, to make 32K of bus aggregation will needs 22 RX_BDs at least
#define SDIO_RX_BD_NUM                  24
/// Number of RX packet handler
#define SDIO_RX_PKT_NUM                 128
#endif

extern const hal_sdiod_func_stubs_t hal_sdiod_stubs;


/**
 *  @brief Enters a critical section of code, mainly it
 *         disable the SDIO device interrupt to prevent race
 *         conditions.
 *
 *  @return     void
 */
__STATIC_INLINE
void hal_sdio_dev_enter_critical (hal_sdio_dev_adapter_t *psdio_adp)
{
    hal_sdiod_stubs.enter_critical(psdio_adp);
}

/**
 *  @brief To exit SDIO device critical section, it will re-enable
 *         SDIO device interrupt.
 *
 *  @return     void
 */
__STATIC_INLINE
void hal_sdio_dev_exit_critical (hal_sdio_dev_adapter_t *psdio_adp)
{
    hal_sdiod_stubs.exit_critical(psdio_adp);
}


/**
 *  @brief Sets one or multiple events of the SDIO device HAL.
 *  @param[in]  psdio_adp  the SDIO device adapter.
 *  @param[in]  event  the event(s)  to be set.
 *
 *  @return     void
 */
__STATIC_INLINE
void hal_sdio_dev_set_event (hal_sdio_dev_adapter_t *psdio_adp, uint32_t event)
{
    hal_sdiod_stubs.set_event(psdio_adp, event);
}

/**
 *  @brief Clears one or multiple events of the SDIO device HAL.
 *  @param[in]  psdio_adp  the SDIO device adapter.
 *  @param[in]  event  the event(s) to be cleared.
 *
 *  @return     void
 */
__STATIC_INLINE
void hal_sdio_dev_clear_event (hal_sdio_dev_adapter_t *psdio_adp, uint32_t event)
{
    hal_sdiod_stubs.clear_event (psdio_adp, event);
}

/**
 *  @brief Checks if any one of given events is pending.
 *  @param[in]  psdio_adp  the SDIO device adapter.
 *  @param[in]  event  the event(s) to be checked.
 *
 *  @return     0  no any given events is pending.
 *  @return     1  at least one of given events is pending.
 */
__STATIC_INLINE
BOOL hal_sdio_dev_event_pending (hal_sdio_dev_adapter_t *psdio_adp, uint32_t event)
{
    return hal_sdiod_stubs.event_pending (psdio_adp, event);
}


/**
 *  @brief Registers a IRQ handler for the SDIO device HAL.
 *  @param[in]  handler  the IRQ handler.
 *  @return     void.
 */
__STATIC_INLINE
void hal_sdio_dev_reg_irq (irq_handler_t handler)
{
    hal_sdiod_stubs.reg_irq (handler);
}

/**
 *  @brief SDIO device enable control.
 *  @details Enable or disable the SDIO device function, clock and pins.
 *  @param[in]   en  The enable control.
 *                 - 1 = Enable.
 *                 - 0 = Disable.
 *  @return      void
 *
 */
__STATIC_INLINE
void hal_sdio_dev_syson_ctrl(BOOL en)
{
    hal_sdiod_stubs.syson_ctrl(en);
}

/**
 *  @brief For no RTOS case, this function is called to process SDIO device HAL
 *         pending events. Basically it is used to replace the functions of TX and RX task.
 *  @param[in]  psdio_adp  the SDIO device HAL adapter.
 *  @return     void.
 */
__STATIC_INLINE
void hal_sdio_dev_task_up (hal_sdio_dev_adapter_t *psdio_adp)
{
    hal_sdiod_stubs.task_up (psdio_adp);
}

/**
 *  @brief The main routine of the TX thread. This thread mainly process pending
 *         events those related to data TX, data packets from SDIO host to SDIO device,
 *         and interrupts.
 *  @param[in]  psdio_adp  the SDIO device HAL adapter.
 *  @return     void.
 */
__STATIC_INLINE
void hal_sdio_dev_tx_task (hal_sdio_dev_adapter_t *psdio_adp)
{
    hal_sdiod_stubs.tx_task (psdio_adp);
}

/**
 *  @brief The main routine of the RX thread. This thread will be waked up to
 *          process the RX DMA done event(interrupt). Pending RX packets will
 *          be send to the SDIO host by fill RX descriptors.
 *  @param[in]  psdio_adp  the SDIO device HAL adapter.
 *  @return     void.
 */
__STATIC_INLINE
void hal_sdio_dev_rx_task (hal_sdio_dev_adapter_t *psdio_adp)
{
    hal_sdiod_stubs.rx_task (psdio_adp);
}

/**
 *  @brief This function is the button half of the SDIO device HAL interrupt handler.
 *         The top half of the interrupt handler only set the pending events
 *         by the interrupt status, those events will be processed in this
 *         function. Following events will be processed:
 *           - SDIO TX data packet (SDIO host to SDIO device) is ready.
 *           - A H2C (SDIO host to SDIO device) command is received.
 *           - The power management state of the driver on the SDIO host
 *             side is changed.
 *           - SDIO hardware operation error or bus error occurred.
 *  @param[in]  psdio_adp  the SDIO device HAL adapter.
 *  @return     void.
 */
__STATIC_INLINE
void hal_sdio_dev_irq_handler_bh (hal_sdio_dev_adapter_t *psdio_adp)
{
    hal_sdiod_stubs.irq_handler_bh (psdio_adp);
}

/**
 *  @brief It's the button half of the SDIO device RX DMA interrupt handler.
 *         This function is called by the RX task to process the SDIO RX DMA
 *         finish event.
 *  @param[in]  psdio_adp  the SDIO device HAL adapter.
 *  @return     void.
 */
__STATIC_INLINE
void hal_sdio_dev_rx_irq_handler_bh (hal_sdio_dev_adapter_t *psdio_adp)
{
    hal_sdiod_stubs.rx_irq_handler_bh (psdio_adp);
}

/**
 *  @brief Re-fills TX buffer descripters. This function is called by the TX task
 *         to submitts new TX descripters to the SDIO hardware. The TX descripter
 *         is used to control the SDIO device hardware to receive packets those
 *         from the SDIO host side.
 *  @param[in]  psdio_adp  the SDIO device HAL adapter.
 *  @return     void.
 */
__STATIC_INLINE
void hal_sdio_dev_txbd_buf_refill (hal_sdio_dev_adapter_t *psdio_adp)
{
    hal_sdiod_stubs.txbd_buf_refill (psdio_adp);
}

/**
 *  @brief Process the event of packets those from the SDIO host side are ready.
 *         Those packets will be forward to the application layer, ex. WLan driver.
 *         And then new TX descripters and buffers will be re-filled for the
 *         hardware can receive following packets.
 *  @param[in]  psdio_adp  the SDIO device HAL adapter.
 *  @return     void.
 */
__STATIC_INLINE
void hal_sdio_dev_tx_fifo_data_ready (hal_sdio_dev_adapter_t *psdio_adp)
{
    hal_sdiod_stubs.tx_fifo_data_ready (psdio_adp);
}

/**
 *  @brief Recycles RX BDs(buffer descripter). A RX BD is used to control
 *         the hardware to send a RX packet to the SDIO host. This function
 *         will be called to recycle one or more RX BDs when a packet RX DMA
 *         is done(triggered by interrupt).
 *         the RX free packet queue.
 *  @param[in]  psdio_adp  the SDIO device HAL adapter.
 *  @return     void.
 */
__STATIC_INLINE
void hal_sdio_dev_recycle_rxbd (hal_sdio_dev_adapter_t *psdio_adp)
{
    hal_sdiod_stubs.recycle_rxbd (psdio_adp);
}

/**
 *  @brief Sends packets in the RX packet pending queue to the SDIO host.
 *         The SDIO device HAL implements a RX packet pending queue to store
 *         RX packets those to be send to the SDIO host. The RX task will
 *         call this function when the RX packe pending queue is not empty.
 *  @param[in]  psdio_adp  the SDIO device HAL adapter.
 *  @return     void.
 */
__STATIC_INLINE
void hal_sdio_dev_return_rx_data (hal_sdio_dev_adapter_t *psdio_adp)
{
    hal_sdiod_stubs.return_rx_data (psdio_adp);
}

/**
 *  @brief Puts a RX packet into the queue for the RX task can transfer it to the SDIO host.
 *  @param[in]  psdio_adp  the SDIO device HAL adapter.
 *  @param[in]  ppkt  the RX packet to be sent to the SDIO host.
 *
 *  @return     HAL_OK  the packet push into the queue successfuly.
 */
__STATIC_INLINE
hal_status_t hal_sdio_dev_rx_pkt_enqueue (hal_sdio_dev_adapter_t *psdio_adp, sdiod_rx_packet_t *ppkt)
{
    return hal_sdiod_stubs.rx_pkt_enqueue (psdio_adp, ppkt);
}

hal_status_t hal_sdio_dev_init (void);
void hal_sdio_dev_deinit (void);
void hal_sdio_dev_send_c2h_iomsg (uint32_t c2h_msg);
void hal_sdio_dev_register_rx_done_callback (sdiod_rx_done_callback_t callback, void *para);
void hal_sdio_dev_register_tx_callback (sdiod_tx_callback_t callback, void *para);
void hal_sdio_dev_register_h2c_msg_callback (sdiod_h2c_msg_callback_t callback, void *para);
psdiod_rx_packet_t hal_sdio_dev_alloc_rx_pkt (hal_sdio_dev_adapter_t *psdio_adp);
void hal_sdio_dev_free_rx_pkt(hal_sdio_dev_adapter_t *psdio_adp, sdiod_rx_packet_t *ppkt);
int8_t hal_sdio_dev_rx_pkt_queue_push (void *pdata, uint16_t offset, uint16_t pkt_size, uint8_t cmd_type);
int8_t hal_sdio_dev_msg_handler (hal_sdio_dev_adapter_t *psdio_adp, sdiod_msg_blk_t *pmblk);
int8_t hal_sdio_dev_send_msg (hal_sdio_dev_adapter_t *psdio_adp, sdiod_msg_blk_t *pmblk, uint32_t timeout_ms);

/** @} */ /* End of group hs_hal_sdio_dev */

#ifdef  __cplusplus
}
#endif


#endif  // end of "#define _HAL_SDIO_DEV_H_"

