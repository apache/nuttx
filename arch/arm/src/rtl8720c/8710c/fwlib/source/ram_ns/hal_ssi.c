/**************************************************************************//**
 * @file     hal_ssi.c
 * @brief    Implement SSI HAL RAM code function.
 * @version  1.00
 * @date     2017-08-22
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

#include "hal_ssi.h"
#include "hal_gdma.h"
#include "hal_pinmux.h"
#include "hal_cache.h"
#include "hal_irq.h"
#include "hal_gpio.h"

extern hal_status_t hal_xip_get_phy_addr (uint32_t vaddr, uint32_t *ppaddr, uint32_t *pis_enc);

// TODO: modification for 8710c: pin-mux register & control , 
#if CONFIG_SPI_EN
/**

        \addtogroup hs_hal_ssi SPI
        \brief The Serial Peripheral Interface bus(SPI) HAL APIs.
        @{
*/


/**

        \addtogroup hs_hal_ssi_ram_func SPI HAL RAM APIs
        \ingroup hs_hal_ssi
        \brief The SPI HAL APIs
        @{
*/

/** \brief Description of hal_ssi_pin_ctl
 *
 *    hal_ssi_pin_ctl is used to enable and select pins for the target SPI device.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *   \param u8 ctl:      A control bit to decide to enable or disable the pinmux of the device.
 *
 *   \return hal_status_t.
 */
hal_status_t hal_ssi_pin_ctl(phal_ssi_adaptor_t phal_ssi_adaptor, u8 ctl)
{
    u8 index;
    hal_status_t ret;
    pspi_pin_sel_t pspi_pin = &(phal_ssi_adaptor->spi_pin);

    if (ctl == ENABLE) {       
        for (index = 0; index < 4; index++) {
            ret = hal_pinmux_register(*(((u8 *)pspi_pin) + index), PID_SPI0);
            if (ret != HAL_OK) {
                DBG_SSI_ERR("PIN %x cannot be registered.\r\n", *(((u8 *)pspi_pin) + index));
                return ret;
            }
        }
    } else {
        for (index = 0; index < 4; index++) {
            ret = hal_pinmux_unregister(*(((u8 *)pspi_pin) + index), PID_SPI0);
            if (ret != HAL_OK) {
                DBG_SSI_ERR("PIN %x cannot be unregistered.\r\n", *(((u8 *)pspi_pin) + index));
                return ret;
            }
        }
    }

    hal_gpio_schmitt_ctrl(pspi_pin->spi_clk_pin, ctl, 1);    
    return HAL_OK;
}


/** \brief Description of hal_ssi_init
 *
 *    hal_ssi_init is used to initialize the SPI device through ROM code. The pinmux selection is also registered here.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *   \param u8 index:      The index of the SPI device.
 *   \param spi_pin_sel_t pin_sel:      The pinmux selection of the SPI device.
 *
 *   \return hal_status_t.
 */
hal_status_t hal_ssi_init(phal_ssi_adaptor_t phal_ssi_adaptor)
{
    hal_status_t ret;

    phal_ssi_adaptor->index = 0;

    ret = hal_ssi_pin_ctl(phal_ssi_adaptor, ENABLE);

    if (ret == HAL_OK) {
        return hal_ssi_init_setting(phal_ssi_adaptor);
    } else {
        return ret;
    }
}

void hal_ssi_toggle_between_frame(phal_ssi_adaptor_t phal_ssi_adaptor, u8 ctl)
{
    u8 sclk_phase = phal_ssi_adaptor->sclk_phase;
    SSI0_Type *spi_dev = (SSI0_Type *) phal_ssi_adaptor->spi_dev;

    if (sclk_phase == ScphToggleInMiddle) {
        spi_dev->ssienr = DISABLE;
        spi_dev->ctrlr0_b.ss_t = ctl;
        spi_dev->ssienr = ENABLE;
    }
}

/** \brief Description of hal_spi_format
 *
 *    hal_spi_format is used to setup SPI frame format, size and SPI mode.
 *    The transfer mode is always transmit and receive mode.
 *    The SPI mode number is transformed  into clock phase and clock polarity before setting these values to registers through ROM code functions.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *   \param u8 bits:      The size of frame, only 8 bits and 16 bits are supported.
 *   \param u8 mode:      The SPI mode, mode 0~3 are supported.
 *
 *   \return void.
 */
void hal_spi_format(phal_ssi_adaptor_t phal_ssi_adaptor, u8 bits, u8 mode)
{
    phal_ssi_adaptor->data_frame_size = bits;
    phal_ssi_adaptor->data_frame_format = FrfMotorolaSpi;
    phal_ssi_adaptor->transfer_mode = TmodTr;

    switch (mode) {
        case 0:
            phal_ssi_adaptor->sclk_polarity = ScpolInactiveIsLow;
            phal_ssi_adaptor->sclk_phase    = ScphToggleInMiddle;
            break;

        case 1:
            phal_ssi_adaptor->sclk_polarity = ScpolInactiveIsLow;
            phal_ssi_adaptor->sclk_phase    = ScphToggleAtStart;
            break;

        case 2:
            phal_ssi_adaptor->sclk_polarity = ScpolInactiveIsHigh;
            phal_ssi_adaptor->sclk_phase    = ScphToggleInMiddle;
            break;

        case 3:
            phal_ssi_adaptor->sclk_polarity = ScpolInactiveIsHigh;
            phal_ssi_adaptor->sclk_phase    = ScphToggleAtStart;
            break;

        default:  // same as 0
            phal_ssi_adaptor->sclk_polarity = ScpolInactiveIsLow;
            phal_ssi_adaptor->sclk_phase    = ScphToggleInMiddle;
            break;
    }

    hal_ssi_set_format(phal_ssi_adaptor);
}

/** \brief Description of hal_ssi_callback_hook
 *
 *    hal_ssi_callback_hook is used to set callback functions when the transfer is complete.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *   \param void *tx_handler:      The callback function when data is transmitted.
 *   \param void *rx_handler:      The callback function when data is received.
 *
 *   \return void.
 */
void hal_ssi_callback_hook(phal_ssi_adaptor_t phal_ssi_adaptor, void *tx_handler, void *rx_handler)
{
    if (tx_handler != NULL) {
        phal_ssi_adaptor->tx_done_callback = (void(*)(void *))tx_handler;
        phal_ssi_adaptor->tx_done_cb_para = phal_ssi_adaptor;
    }

    if (rx_handler != NULL) {
        phal_ssi_adaptor->rx_done_callback = (void(*)(void *))rx_handler;
        phal_ssi_adaptor->rx_done_cb_para = phal_ssi_adaptor;
    }
}

/** \brief Description of hal_ssi_deinit
 *
 *    hal_ssi_deinit is used to deinit the SPI device when it is no longer to be used.
 *    If the transfer involves with GDMA engine, the channel is released by gdma_deinit functions.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *
 *   \return hal_status_t.
 */
hal_status_t hal_ssi_deinit(phal_ssi_adaptor_t phal_ssi_adaptor)
{
    hal_status_t ret;

    ret = hal_ssi_deinit_setting(phal_ssi_adaptor);
    if (ret != HAL_OK) {
        DBG_SSI_ERR("Deinit fails.\r\n");
    }

    ret = hal_ssi_pin_ctl(phal_ssi_adaptor, DISABLE);
    if (ret != HAL_OK) {
        DBG_SSI_ERR("Deinit pins fails.\r\n");
    }

    hal_ssi_tx_gdma_deinit(phal_ssi_adaptor);
    hal_ssi_rx_gdma_deinit(phal_ssi_adaptor);

    return ret;
}

/** \brief Description of hal_ssi_tx_gdma_init
 *
 *    hal_ssi_tx_gdma_init is used to initialize GDMA settings if the SPI device will handshake with GDMA to transmit data.
 *    GDMA channel management functions are called in GDMA RAM code.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *   \param phal_gdma_adaptor_t phal_gdma_adaptor:      The pointer of GDMA adaptor.
 *
 *   \return hal_status_t.
 */
hal_status_t hal_ssi_tx_gdma_init(phal_ssi_adaptor_t phal_ssi_adaptor, phal_gdma_adaptor_t phal_gdma_adaptor)
{
    hal_status_t ret;

    phal_gdma_adaptor->have_chnl = 0;

    ret = hal_gdma_chnl_alloc (phal_gdma_adaptor);    // default no-multiple block support

    if (ret == HAL_OK) {
        ret = hal_ssi_tx_gdma_init_setting(phal_ssi_adaptor, phal_gdma_adaptor);
        
#if !defined (CONFIG_BUILD_NONSECURE)
        hal_irq_disable (phal_gdma_adaptor->gdma_irq_num);
        __ISB();

        phal_gdma_adaptor->gdma_irq_num = SGDMA0_IRQn;

        hal_irq_enable (phal_gdma_adaptor->gdma_irq_num);
#endif

        if (ret == HAL_OK) {
            hal_gdma_chnl_init (phal_gdma_adaptor);
        } else {
            DBG_SSI_ERR("hal_ssi_tx_gdma_init: GDMA init failed(%d)\r\n", ret);
            hal_gdma_chnl_free (phal_gdma_adaptor);
        }
    } else {
        DBG_SSI_ERR("hal_ssi_tx_gdma_init: GDMA channel allocate failed(%d)\r\n", ret);
    }

    phal_ssi_adaptor->dcache_clean_by_addr = hal_cache_stubs.dcache_clean_by_addr;

    if (phal_ssi_adaptor->data_frame_size == DfsSixteenBits) {
        // 16 bits mode
        phal_ssi_adaptor->dma_tx_data_level = 8;  // when tx fifo entity number <=48 then dma request asserted
    } else {
        // 8 bits mode
        phal_ssi_adaptor->dma_tx_data_level = 16;  // when tx fifo entity number <=56 then dma request asserted
    }
    return ret;
}

/** \brief Description of hal_ssi_rx_gdma_init
 *
 *    hal_ssi_rx_gdma_init is used to initialize GDMA settings if the SPI device will handshake with GDMA to receive data.
 *    GDMA channel management functions are called in GDMA RAM code.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *   \param phal_gdma_adaptor_t phal_gdma_adaptor:      The pointer of GDMA adaptor.
 *
 *   \return hal_status_t.
 */
hal_status_t hal_ssi_rx_gdma_init(phal_ssi_adaptor_t phal_ssi_adaptor, phal_gdma_adaptor_t phal_gdma_adaptor)
{
    hal_status_t ret;

    phal_gdma_adaptor->have_chnl = 0;

    ret = hal_gdma_chnl_alloc (phal_gdma_adaptor);    // default no-multiple block support
    if (ret == HAL_OK) {
        ret = hal_ssi_rx_gdma_init_setting(phal_ssi_adaptor, phal_gdma_adaptor);

#if !defined (CONFIG_BUILD_NONSECURE)
                hal_irq_disable (phal_gdma_adaptor->gdma_irq_num);
                __ISB();
        
                phal_gdma_adaptor->gdma_irq_num = SGDMA0_IRQn;
        
                hal_irq_enable (phal_gdma_adaptor->gdma_irq_num);
#endif

        if (ret == HAL_OK) {
            hal_gdma_chnl_init (phal_gdma_adaptor);
        } else {
            DBG_SSI_ERR("hal_ssi_rx_gdma_init: GDMA init failed(%d)\r\n", ret);
            hal_gdma_chnl_free (phal_gdma_adaptor);
        }
    } else {
        DBG_SSI_ERR("hal_ssi_rx_gdma_init: GDMA channel allocate failed(%d)\r\n", ret);
    }

    phal_ssi_adaptor->dcache_invalidate_by_addr = hal_cache_stubs.dcache_invalidate_by_addr;

    if (phal_ssi_adaptor->data_frame_size == DfsSixteenBits) {
        // 16~9 bits mode
        phal_ssi_adaptor->dma_rx_data_level = 7;  // when rx fifo entity number >=8 then dma request asserted
    } else {
        // 8~4 bits mode
        phal_ssi_adaptor->dma_rx_data_level = 3;  // when rx fifo entity number >=4 then dma request asserted
    }

    return ret;
}


/** \brief Description of hal_ssi_tx_gdma_deinit
 *
 *    hal_ssi_tx_gdma_deinit is used to de-initialize GDMA settings if the SPI device will not handshake with GDMA to transmit data anymore.
 *    hal_gdma_chnl_free function is called in GDMA RAM code.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *
 *   \return hal_status_t.
 */
hal_status_t hal_ssi_tx_gdma_deinit(phal_ssi_adaptor_t phal_ssi_adaptor)
{
    phal_gdma_adaptor_t phal_gdma_adaptor;

    if (phal_ssi_adaptor == NULL) {
        DBG_SSI_ERR("hal_ssi_tx_gdma_deinit : Null Adaptor!\n");
        return HAL_ERR_PARA;
    }

    phal_gdma_adaptor = (phal_gdma_adaptor_t) phal_ssi_adaptor->ptx_gdma_adaptor;

    if (phal_gdma_adaptor->gdma_dev != NULL) {
        hal_gdma_chnl_free(phal_gdma_adaptor);
    }

    return HAL_OK;
}

/** \brief Description of hal_ssi_rx_gdma_deinit
 *
 *    hal_ssi_rx_gdma_deinit is used to de-initialize GDMA settings if the SPI device will not handshake with GDMA to receive data anymore.
 *    hal_gdma_chnl_free function is called in GDMA RAM code.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *
 *   \return hal_status_t.
 */
hal_status_t hal_ssi_rx_gdma_deinit(phal_ssi_adaptor_t phal_ssi_adaptor)
{
    phal_gdma_adaptor_t phal_gdma_adaptor;

    if (phal_ssi_adaptor == NULL) {
        DBG_SSI_ERR("hal_ssi_rx_gdma_deinit : Null Adaptor!\n");
        return HAL_ERR_PARA;
    }

    phal_gdma_adaptor = (phal_gdma_adaptor_t) phal_ssi_adaptor->prx_gdma_adaptor;

    if (phal_gdma_adaptor->gdma_dev != NULL) {
        hal_gdma_chnl_free(phal_gdma_adaptor);
    }

    return HAL_OK;
}

/** \brief Description of hal_ssi_dma_send
 *
 *    hal_ssi_dma_send is used to setup the transfer when the SPI device will transmit data with GDMA.
 *    GDMA moves data from memory (ptx_data) to the SPI FIFO, then the SPI device pops data out of its FIFO.
 *    If data length exceeds MAX_DMA_BLOCK_SIZE, this function will re-allocate a channel supporting multiblock mode.
 *    At the end of this function, the transfer begins.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *   \param u8 *ptx_data:      The source address.
 *   \param u32 length:      The total data length.
 *
 *   \return hal_status_t.
 */
hal_status_t hal_ssi_dma_send(phal_ssi_adaptor_t phal_ssi_adaptor, u8  *ptx_data, u32 length)
{
    phal_gdma_adaptor_t phal_gdma_adaptor;
    u32 is_encry;
    u32 *phy_src;
    
    phal_gdma_adaptor = (phal_gdma_adaptor_t) phal_ssi_adaptor->ptx_gdma_adaptor;

    if ((((u32)(ptx_data)) >> 24) == 0x9B) {
        hal_xip_get_phy_addr((u32) ptx_data, (u32 *)&phy_src, &is_encry);
        
        if (is_encry) {
            DBG_GDMA_ERR("Source address should not be on the encryted remapping region!\r\n");
            return HAL_ERR_MEM;
        } else {
            ptx_data = (u8 *)phy_src;
        }
    }
    
    hal_ssi_dma_send_init(phal_ssi_adaptor,ptx_data,length);
    hal_gdma_transfer_start(phal_gdma_adaptor);
    
    return HAL_OK;
}

/** \brief Description of hal_ssi_dma_recv
 *
 *    hal_ssi_dma_recv is used to setup the transfer when the SPI device will receive data with GDMA.
 *    Data is received to the SPI device's FIFO, then GDMA moves data from SPI device's FIFO to memory (prx_data).
 *    If data length exceeds MAX_DMA_BLOCK_SIZE, this function will re-allocate a channel supporting multiblock mode.
 *    At the end of this function, the transfer begins.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *   \param u8 *ptx_data:      The source address.
 *   \param u32 length:      The total data length.
 *
 *   \return hal_status_t.
 */
hal_status_t hal_ssi_dma_recv(phal_ssi_adaptor_t phal_ssi_adaptor, u8  *prx_data, u32 length)
{
    phal_gdma_adaptor_t phal_gdma_adaptor;
    phal_gdma_adaptor = (phal_gdma_adaptor_t) phal_ssi_adaptor->prx_gdma_adaptor;

    hal_ssi_dma_recv_init(phal_ssi_adaptor,prx_data,length);
    hal_gdma_transfer_start(phal_gdma_adaptor);
    
    return HAL_OK;
}

/** *@} */ /* End of group hs_hal_ssi_ram_func */

/** *@} */ /* End of group hs_hal_ssi */
#endif      // end of "#if CONFIG_SPI_EN"
