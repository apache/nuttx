/**************************************************************************//**
 * @file     hal_ssi.h
 * @brief    The header file of hal_ssi.c.
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

#ifndef _HAL_SSI_H_
#define _HAL_SSI_H_
#include "cmsis.h"

#ifdef  __cplusplus
extern "C"
{
#endif

/**

        \addtogroup hs_hal_ssi SPI
        @{
*/


extern const hal_ssi_func_stubs_t hal_ssi_stubs;

#define SPI_CS_PIN_SEL0 PA_2                //!< Define SPI Chip select pin selection 0
#define SPI_CS_PIN_SEL1 PA_7               //!< Define SPI Chip select pin selection 1
#define SPI_CS_PIN_SEL2 PA_15              //!< Define SPI Chip select pin selection 1
#define SPI_SCL_PIN_SEL0 PA_3              //!< Define SPI clock pin selection 0
#define SPI_SCL_PIN_SEL1 PA_8              //!< Define SPI clock pin selection 0
#define SPI_SCL_PIN_SEL2 PA_16             //!< Define SPI clock pin selection 0
#define SPI_MOSI_PIN_SEL0 PA_4             //!< Define SPI mosi data pin selection 0
#define SPI_MOSI_PIN_SEL1 PA_9             //!< Define SPI mosi data pin selection 0
#define SPI_MOSI_PIN_SEL2 PA_19            //!< Define SPI mosi data pin selection 0
#define SPI_MISO_PIN_SEL0 PA_5             //!< Define SPI miso pin selection 0
#define SPI_MISO_PIN_SEL1 PA_10            //!< Define SPI miso pin selection 0
#define SPI_MISO_PIN_SEL2 PA_20            //!< Define SPI miso pin selection 0

/**

        \addtogroup hs_hal_ssi_ram_func SPI HAL RAM APIs
        \ingroup hs_hal_ssi
        \brief The SPI HAL APIs
        @{
*/

#if (CHIP_VER == CHIP_A_CUT) && (defined(CONFIG_BUILD_RAM))
hal_status_t hal_ssi_init_setting_rtl8710c_patch(phal_ssi_adaptor_t phal_ssi_adaptor);
void SPI_IRQHandler_patch(void);
void hal_ssi_irq_handle_rtl8710c_patch(phal_ssi_adaptor_t phal_ssi_adaptor);
void hal_ssi_read_interrupt_rtl8710c_unfix_size_patch(phal_ssi_adaptor_t phal_ssi_adaptor);
#endif

#if ((CHIP_VER == CHIP_A_CUT) || (CHIP_VER == CHIP_B_CUT)) && (defined(CONFIG_BUILD_RAM))
hal_status_t hal_ssi_set_sclk_rtl8710c_patch(phal_ssi_adaptor_t phal_ssi_adaptor, u32 clk_rate);
#endif
/** \brief Description of hal_ssi_clock_ctl
 *
 *    hal_ssi_clock_ctl is used to enable IP and turn on the clock for the target SPI device.
 *
 *   \param u8 ctl:      A control bit to decide to enable or disable the device.
 *
 *   \return hal_status_t.
 */
__STATIC_INLINE hal_status_t hal_ssi_clock_ctl (u8 ctl)
{
    return hal_ssi_stubs.hal_ssi_clock_ctl (ctl);
}

/** \brief Description of hal_ssi_enable
 *
 *    hal_ssi_enable is used to enable the target SPI device.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *
 *   \return hal_status_t.
 */
__STATIC_INLINE hal_status_t hal_ssi_enable (phal_ssi_adaptor_t phal_ssi_adaptor)
{
    return hal_ssi_stubs.hal_ssi_enable (phal_ssi_adaptor);
}

/** \brief Description of hal_ssi_disable
 *
 *    hal_ssi_disable is used to disable the target SPI device.
 *    Data in SPI FIFO will be cleared when the device is disabled.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *
 *   \return hal_status_t.
 */
__STATIC_INLINE hal_status_t hal_ssi_disable (phal_ssi_adaptor_t phal_ssi_adaptor)
{
    return hal_ssi_stubs.hal_ssi_disable (phal_ssi_adaptor);
}

/** \brief Description of hal_ssi_init_setting
 *
 *    hal_ssi_init_setting is used to initialize SPI device.
 *    It includes register base, clock, enable control and IRQ parameter initialization.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *
 *   \return hal_status_t.
 */
__STATIC_INLINE hal_status_t hal_ssi_init_setting (phal_ssi_adaptor_t phal_ssi_adaptor)
{
#if (CHIP_VER == CHIP_A_CUT) && (defined(CONFIG_BUILD_RAM))
    return hal_ssi_init_setting_rtl8710c_patch(phal_ssi_adaptor);
#else
    return hal_ssi_stubs.hal_ssi_init_setting (phal_ssi_adaptor);
#endif
}

/** \brief Description of hal_ssi_deinit_setting
 *
 *    hal_ssi_deinit_setting is used to deinitialize the target SPI device after this SPI is no longer needed.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *
 *   \return hal_status_t.
 */
__STATIC_INLINE hal_status_t hal_ssi_deinit_setting (phal_ssi_adaptor_t phal_ssi_adaptor)
{
    return hal_ssi_stubs.hal_ssi_deinit_setting (phal_ssi_adaptor);
}

/** \brief Description of hal_ssi_read_interrupt
 *
 *    hal_ssi_read_interrupt is used to handle a interrupt when the receive FIFO threshold has reached in receive mode.
 *    As soon as CPU enters this handler function, it identifies the number of data frames in the receive FIFO.
 *    These data then are moved from the receive FIFO to memory.
 *    A SPI master still pushes new data into the receive FIFO at the same time, but these new data won't be handled in this interrupt.
 *    After all data have been received and handled properly by this interrupt handler, the interrupt handler jumps to a callback function assigned by users.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *
 *   \return void.
 */
__STATIC_INLINE void hal_ssi_read_interrupt (phal_ssi_adaptor_t phal_ssi_adaptor)
{
    hal_ssi_stubs.hal_ssi_read_interrupt (phal_ssi_adaptor);
}

/** \brief Description of hal_ssi_write_interrupt
 *
 *    hal_ssi_write_interrupt is used to handle a interrupt when the transmit FIFO threshold has reached in transmit mode.
 *    CPU will move data to the transmit FIFO, then the SPI device pops data out of its FIFO.
 *    After all data have been transmitted and handled properly by this interrupt handler, the interrupt handler jumps to a callback function assigned by users.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *
 *   \return hal_status_t.
 */
__STATIC_INLINE void hal_ssi_write_interrupt (phal_ssi_adaptor_t phal_ssi_adaptor)
{
    hal_ssi_stubs.hal_ssi_write_interrupt (phal_ssi_adaptor);
}

/** \brief Description of hal_ssi_irq_handle
 *
 *    hal_ssi_irq_handle is used to handle interrupt from the SPI side after identifying the SPI device triggering the interrupt.
 *    Each type of interrupt will be handled according to the circuit design.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *
 *   \return void.
 */
__STATIC_INLINE void hal_ssi_irq_handle (phal_ssi_adaptor_t phal_ssi_adaptor)
{
    hal_ssi_stubs.hal_ssi_irq_handle (phal_ssi_adaptor);
}

/** \brief Description of hal_ssi_interrupt_enable
 *
 *    hal_ssi_interrupt_enable is used to enable interrupt for all SPI devices.
 *    Note that all SPI devices share the same interrupt signal.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *
 *   \return hal_status_t.
 */
__STATIC_INLINE hal_status_t hal_ssi_interrupt_enable (phal_ssi_adaptor_t phal_ssi_adaptor)
{
    return hal_ssi_stubs.hal_ssi_interrupt_enable (phal_ssi_adaptor);
}

/** \brief Description of hal_ssi_interrupt_disable
 *
 *    hal_ssi_interrupt_disable is used to disable interrupt for all SPI devices.
 *    Note that all SPI devices share the same interrupt signal. Once it is disabled, interrupts from every SPI devices will not be handled.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *
 *   \return hal_status_t.
 */
__STATIC_INLINE hal_status_t hal_ssi_interrupt_disable (phal_ssi_adaptor_t phal_ssi_adaptor)
{
    return hal_ssi_stubs.hal_ssi_interrupt_disable (phal_ssi_adaptor);
}

/** \brief Description of hal_ssi_interrupt_init_read
 *
 *    hal_ssi_interrupt_init_read is used to configure relevant interrupt settings for receive mode.
 *    After the SPI device receives data from a SPI master, it triggers an interrupt once the receive FIFO threshold is reached.
 *    CPU will move data from the receive FIFO to the memory address(*rx_data) in the IRQ handler function.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *   \param void *rx_data:      The destination address of data. Data will be moved from the the SPI FIFO to the destination address by CPU.
 *   \param u32 length:      Total transfer length.
 *
 *   \return hal_status_t.
 */
__STATIC_INLINE hal_status_t hal_ssi_interrupt_init_read (phal_ssi_adaptor_t phal_ssi_adaptor, void *rx_data, u32 length)
{
    return hal_ssi_stubs.hal_ssi_interrupt_init_read (phal_ssi_adaptor, rx_data, length);
}

/** \brief Description of hal_ssi_interrupt_init_write
 *
 *    hal_ssi_interrupt_init_write is used to configure relevant interrupt settings for transmit mode.
 *    When the SPI device transmits data out of its transmit FIFO,
 *    it will triggers an interrupt to ask for more data as long as an transmit FIFO threshold is reached.
 *    CPU then moves data from the memory address(tx_data) to the transmit FIFO in the IRQ handler function.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *   \param void *tx_data:      The source address of data. Data will be moved from the source address to the SPI FIFO.
 *   \param u32 length:      Total transfer length.
 *
 *   \return hal_status_t.
 */
__STATIC_INLINE hal_status_t hal_ssi_interrupt_init_write (phal_ssi_adaptor_t phal_ssi_adaptor, void *tx_data, u32 length)
{
    return hal_ssi_stubs.hal_ssi_interrupt_init_write (phal_ssi_adaptor, tx_data, length);
}

/** \brief Description of hal_ssi_set_sclk
 *
 *    hal_ssi_set_sclk is used to set operting frequency of the SPI device.
 *    The operating frequency is determined by the SPI master device, but the SPI slave device still has a constraint for its speed.
 *    We list the maximum speed supported by different SPI devices below.
 *    SPI 0 Master : 25MHz, Slave : 5MHz
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *   \param u32 clk_rate:      The target speed of the SPI device. For the slave device, this parameter can be used to check if it is valid.
 *
 *   \return hal_status_t.
 */
__STATIC_INLINE hal_status_t hal_ssi_set_sclk(phal_ssi_adaptor_t phal_ssi_adaptor, u32 clk_rate)
{
#if ((CHIP_VER == CHIP_A_CUT) || (CHIP_VER == CHIP_B_CUT)) && (defined(CONFIG_BUILD_RAM))
    return hal_ssi_set_sclk_rtl8710c_patch(phal_ssi_adaptor, clk_rate);
#else
    return hal_ssi_stubs.hal_ssi_set_sclk(phal_ssi_adaptor, clk_rate);
#endif
}

/** \brief Description of hal_ssi_set_format
 *
 *    hal_ssi_set_format is used to set SPI mode, transfer mode, frame format and frame size.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *
 *   \return hal_status_t.
 */
__STATIC_INLINE hal_status_t hal_ssi_set_format (phal_ssi_adaptor_t phal_ssi_adaptor)
{
    return hal_ssi_stubs.hal_ssi_set_format (phal_ssi_adaptor);
}

/** \brief Description of hal_ssi_set_microwire
 *
 *    hal_ssi_set_microwire is used to set parameters for Microwire protocol.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *
 *   \return hal_status_t.
 */
__STATIC_INLINE hal_status_t hal_ssi_set_microwire (phal_ssi_adaptor_t phal_ssi_adaptor)
{
    return hal_ssi_stubs.hal_ssi_set_microwire (phal_ssi_adaptor);
}

/** \brief Description of hal_ssi_set_sclk_polarity
 *
 *    hal_ssi_set_sclk_polarity is used to set clock polarity of SPI mode.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *   \param u8 sclk_polarity:      The clock polarity.
 *
 *   \return hal_status_t.
 */
__STATIC_INLINE hal_status_t hal_ssi_set_sclk_polarity (phal_ssi_adaptor_t phal_ssi_adaptor, u8 sclk_polarity)
{
    return hal_ssi_stubs.hal_ssi_set_sclk_polarity (phal_ssi_adaptor, sclk_polarity);
}

/** \brief Description of hal_ssi_set_sclk_phase
 *
 *    hal_ssi_set_sclk_phase is used to set clock phase of SPI mode.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *   \param u8 sclk_phase:      The clock phase.
 *
 *   \return hal_status_t.
 */
__STATIC_INLINE hal_status_t hal_ssi_set_sclk_phase (phal_ssi_adaptor_t phal_ssi_adaptor, u8 sclk_phase)
{
    return hal_ssi_stubs.hal_ssi_set_sclk_phase (phal_ssi_adaptor, sclk_phase);
}

/** \brief Description of hal_ssi_set_data_frame_number
 *
 *    hal_ssi_set_data_frame_number is used to set the number of data frame in receive only mode so that the SPI master knows when to deassert CS line.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *   \param u32 length:      Data length expected to receive.
 *
 *   \return hal_status_t.
 */
__STATIC_INLINE hal_status_t hal_ssi_set_data_frame_number (phal_ssi_adaptor_t phal_ssi_adaptor, u32 length)
{
    return hal_ssi_stubs.hal_ssi_set_data_frame_number (phal_ssi_adaptor, length);
}

/** \brief Description of hal_ssi_set_interrupt_mask
 *
 *    hal_ssi_set_interrupt_mask is used to set the interrupt mask value to the register.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *   \param u8 imr_value:      The interrupt mask setting.
 *
 *   \return hal_status_t.
 */
__STATIC_INLINE hal_status_t hal_ssi_set_interrupt_mask (phal_ssi_adaptor_t phal_ssi_adaptor, u8 imr_value)
{
    return hal_ssi_stubs.hal_ssi_set_interrupt_mask (phal_ssi_adaptor, imr_value);
}

/** \brief Description of hal_ssi_set_device_role
 *
 *    hal_ssi_set_device_role is used to set the role of SPI device.
 *    SPI 0 & 1 can be either master or slave device.
 *    SPI 2 can only be a master while SPI 3 can only be a slave.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *   \param u32 role:      The role of the SPI device.
 *
 *   \return hal_status_t.
 */
__STATIC_INLINE hal_status_t hal_ssi_set_device_role (phal_ssi_adaptor_t phal_ssi_adaptor, u32 role)
{
    return hal_ssi_stubs.hal_ssi_set_device_role (phal_ssi_adaptor, role);
}

/** \brief Description of hal_ssi_set_txfifo_threshold
 *
 *    hal_ssi_set_txfifo_threshold is used to set the threshold value to trigger an interrupt in transmit mode.
 *    When transmit FIFO entries <= the threshold, the transmit FIFO empty interrupt is triggered.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *   \param u32 txftlr_value:      The threshold value of transmit FIFO.
 *
 *   \return hal_status_t.
 */
__STATIC_INLINE hal_status_t hal_ssi_set_txfifo_threshold (phal_ssi_adaptor_t phal_ssi_adaptor, u32 txftlr_value)
{
    return hal_ssi_stubs.hal_ssi_set_txfifo_threshold (phal_ssi_adaptor, txftlr_value);
}

/** \brief Description of hal_ssi_set_rxfifo_threshold
 *
 *    hal_ssi_set_rxfifo_threshold is used to set the threshold value to trigger an interrupt in receive mode.
 *    When receive FIFO entries >= the threshold+1, the receive FIFO full interrupt is triggered.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *   \param u32 rxftlr_value:      The threshold value of receive FIFO.
 *
 *   \return hal_status_t.
 */
__STATIC_INLINE hal_status_t hal_ssi_set_rxfifo_threshold (phal_ssi_adaptor_t phal_ssi_adaptor,u32 rxftlr_value)
{
    return hal_ssi_stubs.hal_ssi_set_rxfifo_threshold (phal_ssi_adaptor, rxftlr_value);
}

/** \brief Description of hal_ssi_set_slave_enable
 *
 *    hal_ssi_set_slave_enable is used to select the target SPI slave device.
 *    Only one slave (value = 0) can be selected for SPI master.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *   \param u32 slave_index:      The target SPI slave index, should be 0.
 *
 *   \return hal_status_t.
 */
__STATIC_INLINE hal_status_t hal_ssi_set_slave_enable (phal_ssi_adaptor_t phal_ssi_adaptor, u32 slave_index)
{
    return hal_ssi_stubs.hal_ssi_set_slave_enable (phal_ssi_adaptor, slave_index);
}

/** \brief Description of hal_ssi_get_rxfifo_level
 *
 *    hal_ssi_get_rxfifo_level is used to know the number of data frames in receive FIFO.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *
 *   \return u32: the number of data frames in receive FIFO.
 */
__STATIC_INLINE u32 hal_ssi_get_rxfifo_level (phal_ssi_adaptor_t phal_ssi_adaptor)
{
    return hal_ssi_stubs.hal_ssi_get_rxfifo_level (phal_ssi_adaptor);
}

/** \brief Description of hal_ssi_get_txfifo_level
 *
 *    hal_ssi_get_txfifo_level is used to know the number of data frames in transmit FIFO.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *
 *   \return u32: the number of data frames in transmit FIFO.
 */
__STATIC_INLINE u32 hal_ssi_get_txfifo_level (phal_ssi_adaptor_t phal_ssi_adaptor)
{
    return hal_ssi_stubs.hal_ssi_get_txfifo_level (phal_ssi_adaptor);
}

/** \brief Description of hal_ssi_get_status
 *
 *    hal_ssi_get_status is used to know the current status of the SPI devices.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *
 *   \return u32: the value of the status register.
 */
__STATIC_INLINE u32 hal_ssi_get_status (phal_ssi_adaptor_t phal_ssi_adaptor)
{
    return hal_ssi_stubs.hal_ssi_get_status (phal_ssi_adaptor);
}

/** \brief Description of hal_ssi_get_busy
 *
 *    hal_ssi_get_busy is used to know if the SPI device is under busy state.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *
 *   \return u32: 1: busy, 0: idle or disable
 */
__STATIC_INLINE u32 hal_ssi_get_busy (phal_ssi_adaptor_t phal_ssi_adaptor)
{
    return hal_ssi_stubs.hal_ssi_get_busy (phal_ssi_adaptor);
}

/** \brief Description of hal_ssi_get_interrupt_mask
 *
 *    hal_ssi_get_interrupt_mask is used to get current interrupt mask value from the register.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *
 *   \return u32: interrupt mask value.
 */
__STATIC_INLINE u32 hal_ssi_get_interrupt_mask (phal_ssi_adaptor_t phal_ssi_adaptor)
{
    return hal_ssi_stubs.hal_ssi_get_interrupt_mask (phal_ssi_adaptor);
}

/** \brief Description of hal_ssi_get_interrupt_status
 *
 *    hal_ssi_get_interrupt_status is used to know current interrupt status of the SPI device.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *
 *   \return u32: interrupt status.
 */
__STATIC_INLINE u32 hal_ssi_get_interrupt_status (phal_ssi_adaptor_t phal_ssi_adaptor)
{
    return hal_ssi_stubs.hal_ssi_get_interrupt_status (phal_ssi_adaptor);
}

/** \brief Description of hal_ssi_get_raw_interrupt_status
 *
 *    hal_ssi_get_raw_interrupt_status is used to know current raw interrupt status of the SPI device.
 *    The raw interrupt status indicates whether or not the interrupt is masked, the interrupt will be set once the requirment is met.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *
 *   \return u32: raw interrupt status.
 */
__STATIC_INLINE u32 hal_ssi_get_raw_interrupt_status (phal_ssi_adaptor_t phal_ssi_adaptor)
{
    return hal_ssi_stubs.hal_ssi_get_raw_interrupt_status (phal_ssi_adaptor);
}

/** \brief Description of hal_ssi_get_slave_enable_register
 *
 *    hal_ssi_get_slave_enable_register is used to know which slave device is selected. Should always be 0.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *
 *   \return u32: 0.
 */
__STATIC_INLINE u32 hal_ssi_get_slave_enable_register (phal_ssi_adaptor_t phal_ssi_adaptor)
{
    return hal_ssi_stubs.hal_ssi_get_slave_enable_register (phal_ssi_adaptor);
}

/** \brief Description of hal_ssi_writable
 *
 *    hal_ssi_writable is used to know if SPI transmit FIFO can be written.
 *    SPI transmit FIFO cannot be written if it is full.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *
 *   \return u32: 1: transmit FIFO is available, 0: transmit FIFO is full.
 */
__STATIC_INLINE u32 hal_ssi_writable (phal_ssi_adaptor_t phal_ssi_adaptor)
{
    return hal_ssi_stubs.hal_ssi_writable (phal_ssi_adaptor);
}

/** \brief Description of hal_ssi_readable
 *
 *    hal_ssi_readable is used to know if SPI receive FIFO can be read.
 *    SPI receive FIFO cannot be read if it is empty.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *
 *   \return u32: 1: receive FIFO can be read, 0: receive FIFO is empty.
 */
__STATIC_INLINE u32 hal_ssi_readable (phal_ssi_adaptor_t phal_ssi_adaptor)
{
    return hal_ssi_stubs.hal_ssi_readable (phal_ssi_adaptor);
}

/** \brief Description of hal_ssi_write
 *
 *    hal_ssi_write is used to push one data into transmit FIFO.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *   \param u32 value:      The data.
 *
 *   \return hal_status_t.
 */
__STATIC_INLINE hal_status_t hal_ssi_write (phal_ssi_adaptor_t phal_ssi_adaptor, u32 value)
{
    return hal_ssi_stubs.hal_ssi_write (phal_ssi_adaptor, value);
}

/** \brief Description of hal_ssi_read
 *
 *    hal_ssi_read is used to read one data from receive FIFO.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *
 *   \return u32: data read from receive FIFO.
 */
__STATIC_INLINE u32 hal_ssi_read (phal_ssi_adaptor_t phal_ssi_adaptor)
{
    return hal_ssi_stubs.hal_ssi_read (phal_ssi_adaptor);
}

/** \brief Description of hal_ssi_stop_recv
 *
 *    hal_ssi_stop_recv is used to terminate data receiving for the SPI device.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *
 *   \return hal_status_t.
 */
__STATIC_INLINE hal_status_t hal_ssi_stop_recv (phal_ssi_adaptor_t phal_ssi_adaptor)
{
    return hal_ssi_stubs.hal_ssi_stop_recv (phal_ssi_adaptor);
}

/** \brief Description of hal_ssi_enter_critical
 *
 *    hal_ssi_enter_critical is used to enter critical section for SPI device.
 *    It is accomplished by disabling relevant interrupts.
 *    This function can prevent from inadvert interferences from unexpected interrupts when users initialize the transfer.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *
 *   \return hal_status_t.
 */
__STATIC_INLINE hal_status_t hal_ssi_enter_critical (phal_ssi_adaptor_t phal_ssi_adaptor)
{
    return hal_ssi_stubs.hal_ssi_enter_critical (phal_ssi_adaptor);
}

/** \brief Description of hal_ssi_exit_critical
 *
 *    hal_ssi_exit_critical is used to exit critical section when the transfer finished initializing procedure.
 *    The interrupts being disabled before would be re-enabled again.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *
 *   \return hal_status_t.
 */
__STATIC_INLINE hal_status_t hal_ssi_exit_critical (phal_ssi_adaptor_t phal_ssi_adaptor)
{
    return hal_ssi_stubs.hal_ssi_exit_critical (phal_ssi_adaptor);
}

/** \brief Description of hal_ssi_tx_gdma_irq_handle
 *
 *    hal_ssi_tx_gdma_irq_handle is used to handle an interrupt in transmit mode when the tranfer is accomplished by GDMA.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *
 *   \return void.
 */
__STATIC_INLINE void hal_ssi_tx_gdma_irq_handle (phal_ssi_adaptor_t phal_ssi_adaptor)
{
    hal_ssi_stubs.hal_ssi_tx_gdma_irq_handle (phal_ssi_adaptor);
}

/** \brief Description of hal_ssi_rx_gdma_irq_handle
 *
 *    hal_ssi_rx_gdma_irq_handle is used to handle an interrupt in receive mode when the tranfer is accomplished by GDMA.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *
 *   \return void.
 */
__STATIC_INLINE void hal_ssi_rx_gdma_irq_handle (phal_ssi_adaptor_t phal_ssi_adaptor)
{
    hal_ssi_stubs.hal_ssi_rx_gdma_irq_handle (phal_ssi_adaptor);
}

/** \brief Description of hal_ssi_tx_gdma_init_setting
 *
 *    hal_ssi_tx_gdma_init_setting is used to initialize GDMA when data is moved from memory to SPI FIFO by GDMA.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *   \param phal_gdma_adaptor_t phal_gdma_adaptor:      The pointer of GDMA adaptor.
 *
 *   \return hal_status_t.
 */
__STATIC_INLINE hal_status_t hal_ssi_tx_gdma_init_setting (phal_ssi_adaptor_t phal_ssi_adaptor,phal_gdma_adaptor_t phal_gdma_adaptor)
{
    return hal_ssi_stubs.hal_ssi_tx_gdma_init_setting (phal_ssi_adaptor, phal_gdma_adaptor);
}

/** \brief Description of hal_ssi_rx_gdma_init_setting
 *
 *    hal_ssi_rx_gdma_init_setting is used to initialize GDMA when data is moved from SPI FIFO to memory by GDMA.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *   \param phal_gdma_adaptor_t phal_gdma_adaptor:      The pointer of GDMA adaptor.
 *
 *   \return hal_status_t.
 */
__STATIC_INLINE hal_status_t hal_ssi_rx_gdma_init_setting (phal_ssi_adaptor_t phal_ssi_adaptor,phal_gdma_adaptor_t phal_gdma_adaptor)
{
    return hal_ssi_stubs.hal_ssi_rx_gdma_init_setting (phal_ssi_adaptor, phal_gdma_adaptor);
}

/** \brief Description of hal_ssi_dma_send_init
 *
 *    hal_ssi_dma_send_init is used to configure a transfer to send data from memory to SPI FIFO by GDMA.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *   \param u8 *ptx_data:      The source address of data(on memory).
 *   \param u32 length:     Total transfer length.
 *
 *   \return hal_status_t.
 */
__STATIC_INLINE hal_status_t hal_ssi_dma_send_init (phal_ssi_adaptor_t phal_ssi_adaptor, u8 *ptx_data, u32 length)
{
    return hal_ssi_stubs.hal_ssi_dma_send_init (phal_ssi_adaptor, ptx_data, length);
}

/** \brief Description of hal_ssi_dma_recv_init
 *
 *    hal_ssi_dma_recv_init is used to configure a transfer to move data from SPI FIFO to memory by GDMA.
 *
 *   \param phal_ssi_adaptor_t phal_ssi_adaptor:      The pointer of SPI adaptor.
 *   \param u8 *prx_data:      The destination address of data(on memory).
 *   \param u32 length:     Total transfer length.
 *
 *   \return hal_status_t.
 */
__STATIC_INLINE hal_status_t hal_ssi_dma_recv_init (phal_ssi_adaptor_t phal_ssi_adaptor, u8 *prx_data, u32 length)
{
    return hal_ssi_stubs.hal_ssi_dma_recv_init (phal_ssi_adaptor, prx_data, length);
}

hal_status_t hal_ssi_pin_ctl(phal_ssi_adaptor_t phal_ssi_adaptor, u8 ctl);
hal_status_t hal_ssi_init(phal_ssi_adaptor_t phal_ssi_adaptor);
void hal_spi_format(phal_ssi_adaptor_t phal_ssi_adaptor, u8 bits, u8 mode);
void hal_ssi_irq_hook(phal_ssi_adaptor_t phal_ssi_adaptor, u32 tx_handler, u32 rx_handler);
hal_status_t hal_ssi_deinit(phal_ssi_adaptor_t phal_ssi_adaptor);
hal_status_t hal_ssi_tx_gdma_init(phal_ssi_adaptor_t phal_ssi_adaptor, phal_gdma_adaptor_t phal_gdma_adaptor);
hal_status_t hal_ssi_rx_gdma_init(phal_ssi_adaptor_t phal_ssi_adaptor, phal_gdma_adaptor_t phal_gdma_adaptor);
hal_status_t hal_ssi_tx_gdma_deinit(phal_ssi_adaptor_t phal_ssi_adaptor);
hal_status_t hal_ssi_rx_gdma_deinit(phal_ssi_adaptor_t phal_ssi_adaptor);
hal_status_t hal_ssi_dma_send(phal_ssi_adaptor_t phal_ssi_adaptor, u8  *ptx_data, u32 length);
hal_status_t hal_ssi_dma_recv(phal_ssi_adaptor_t phal_ssi_adaptor, u8  *prx_data, u32 length);
void hal_ssi_callback_hook(phal_ssi_adaptor_t phal_ssi_adaptor, void *tx_handler, void *rx_handler);

/** *@} */ /* End of group hs_hal_ssi_ram_func */

/** *@} */ /* End of group hs_hal_ssi */

#ifdef  __cplusplus
}
#endif


#endif  // end of "#define _HAL_SSI_H_"


