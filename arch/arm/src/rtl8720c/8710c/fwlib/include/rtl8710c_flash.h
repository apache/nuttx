/**************************************************************************//**
 * @file     rtl8710c_flash.h
 * @brief    The header file of rtl8710c_flash.c.
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

#ifndef _RTL8710C_FLASH_H_
#define _RTL8710C_FLASH_H_

/**

        \addtogroup hs_hal_flash FLASH
        \ingroup 8710c_hal
        \brief The Flash commands and information of the AmebaZ2 platform.
        @{
*/

#define FLASH_TYPE_OTHERS 0                     //!< Define unidentified flash type
#define FLASH_TYPE_MXIC 1                       //!< Define Macronix flash type
#define FLASH_TYPE_WINBOND 2                    //!< Define Winbond flash type
#define FLASH_TYPE_MICRON 3                     //!< Define Micron flash type

#define SPIC_FIFO_DEPTH 64                      //!< Define flash controller FIFO depth

#define FLASH_CMD_RDP 0xAB                      //!< Define Release from deep power down command
#define FLASH_CMD_EN_SPI1 0xFF                  //!< Define Enter SPI Mode flash command
#define FLASH_CMD_EN_SPI2 0xF5                  //!< Define the other Enter SPI Mode flash command

/**
  \brief  The data struct that lists dummy cycles of various flash commands
*/
typedef struct _flash_dummy_cycle_s {
    volatile uint8_t dc_read;                   //!< Dummy cycles of read command
    volatile uint8_t dc_dread;                  //!< Dummy cycles of dual output read command (1-1-2)
    volatile uint8_t dc_2read;                  //!< Dummy cycles of dual IO read command (1-2-2)
    volatile uint8_t dc_qread;                  //!< Dummy cycles of quad output read command (1-1-4)
    volatile uint8_t dc_4read;                  //!< Dummy cycles of quad IO read command (1-4-4)
    volatile uint8_t dc_qpi_read;               //!< Dummy cycles of QPI read command (4-4-4)
}flash_dummy_cycle_t, *pflash_dummy_cycle_t;

/**
  \brief  The data struct that lists all flash commands
*/
typedef struct _flash_cmd_s {
    volatile uint8_t wren;                      //!< Write Enable Command
    volatile uint8_t wrdi;                      //!< Write Enable Command
    volatile uint8_t rdsr;                      //!< Read Status Register Command
    volatile uint8_t wrsr;                      //!< Write Status Register Command
    volatile uint8_t rdsr2;                     //!< Read Status Register 2 Command
    volatile uint8_t wrsr2;                     //!< Write Status Register 2 Command
    volatile uint8_t rdsr3;                     //!< Read Status Register 3 Command
    volatile uint8_t wrsr3;                     //!< Write Status Register 3 Command
    volatile uint8_t rear;                      //!< Read Extended Address Register Command
    volatile uint8_t wear;                      //!< Write Extended Address Register Command
    volatile uint8_t se;                        //!< Sector Erase Command
    volatile uint8_t be_32k;                    //!< 32K-Byte Block Erase Command
    volatile uint8_t be_64k;                    //!< 64K-Byte Block Erase Command
    volatile uint8_t ce;                        //!< Chip Erase Command
    volatile uint8_t rdid;                      //!< Read Flash ID Command
    volatile uint8_t read;                      //!< Read Command
    volatile uint8_t fread;                     //!< Fast Read Command
    volatile uint8_t dread;                     //!< Dual Output Read Command (1-1-2)
    volatile uint8_t str_2read;                 //!< Dual IO Read Command (1-2-2)
    volatile uint8_t dtr_2read;                 //!< DTR Daul IO Read Command (2-2D-2D)
    volatile uint8_t qread;                     //!< Quad Output Read Command (1-1-4)
    volatile uint8_t str_4read;                 //!< Quad IO Read Command (1-4-4), also for QPI(4-4-4)
    volatile uint8_t dtr_4read;                 //!< DTR Quad IO Read Command (1-4D-4D), also for QPI DTR(4-4D-4D/4D-4D-4D)
    volatile uint8_t pp;                        //!< Page Program Command
    volatile uint8_t dp;                        //!< Enter Deep Power Down Command
    volatile uint8_t udp;                       //!< Enter Ultra Deep Power Down Command
    volatile uint8_t rdp;                       //!< Release From Deep Power Down Command
    volatile uint8_t en_spi;                    //!< Enter SPI Mode Command
    volatile uint8_t en_qpi;                    //!< Enter QPI Mode Command
    volatile uint8_t block_lock;                //!< Individual Block Lock Command
    volatile uint8_t block_unlock;              //!< Individual Block Unlock Command
    volatile uint8_t global_lock;               //!< Global Block Lock Command
    volatile uint8_t global_unlock;             //!< Global Block Unlock Command
    volatile uint8_t rd_block_lock;             //!< Read Block Lock State Command
    volatile uint8_t en_reset;                  //!< Enable Reset Command
    volatile uint8_t reset;                     //!< Reset Command
    volatile uint8_t read_sfdp;                 //!< Read SFDP Command
}flash_cmd_t, *pflash_cmd_t;


/// @cond DOXYGEN_ROM_HAL_API

/**

        \addtogroup hs_hal_flash_rom_func FLASH HAL ROM APIs
        @{
*/

void flash_read_unique_id_rtl8710c(void *adaptor, uint8_t *buf, uint8_t len);
hal_status_t flash_read_id_rtl8710c(void *adaptor);
void flash_set_write_enable_rtl8710c(void *adaptor);
void flash_set_status_rtl8710c(void *adaptor, u8 cmd, u8 data);
void flash_set_status_no_check_rtl8710c(void *adaptor, u8 cmd, u8 data);
void flash_set_status_with_addr_rtl8710c(void *adaptor, u8 cmd, u8 addr, u8 data);
void flash_set_extended_addr_rtl8710c(void *adaptor, u8 data);
void flash_set_write_protect_mode_rtl8710c(void *adaptor, u8 mode);
u8 flash_get_status_rtl8710c(void *adaptor, u8 cmd);
u8 flash_get_status_with_addr_rtl8710c(void *adaptor, u8 cmd, u8 addr);
u8 flash_get_extended_addr_rtl8710c(void *adaptor);
void flash_wait_ready_rtl8710c(void *adaptor);
void flash_chip_erase_rtl8710c(void *adaptor);
void flash_64k_block_erase_rtl8710c(void *adaptor, u32 address);
void flash_32k_block_erase_rtl8710c(void *adaptor, u32 address);
void flash_sector_erase_rtl8710c(void *adaptor, u32 address);
u8 flash_query_sector_protect_state_rtl8710c(void *adaptor, u32 address);
void flash_protect_sector_rtl8710c(void *adaptor, u32 address);
void flash_unprotect_sector_rtl8710c(void *adaptor, u32 address);
void flash_global_lock_rtl8710c(void *adaptor);
void flash_global_unlock_rtl8710c(void *adaptor);
void flash_set_dummy_cycle_rtl8710c(void *adaptor);
void flash_set_quad_enable_rtl8710c(void *adaptor);
void flash_unset_quad_enable_rtl8710c(void *adaptor);
void flash_enable_qpi_rtl8710c(void *adaptor);
void flash_return_spi_rtl8710c(void *adaptor);
hal_status_t flash_enter_power_down_rtl8710c(void *adaptor);
hal_status_t flash_release_from_power_down_rtl8710c(void *adaptor);
void flash_stream_read_rtl8710c(void *adaptor, u32 length, u32 addr, u8 *data);
void flash_stream_write_rtl8710c(void *adaptor, u32 length, u32 addr, u8 *data);
void flash_burst_read_rtl8710c(void *adaptor, u32 length, u32 addr, u8 *data);
void flash_burst_write_rtl8710c(void *adaptor, u32 length, u32 addr, u8 *data);
void flash_page_program_rtl8710c(void *adaptor, u32 length, u32 addr, u8 *data);
void flash_reset_to_spi_rtl8710c(void *adaptor);

/** *@} */ /* End of group hs_hal_flash_rom_func */

/// @endcond /* End of condition DOXYGEN_ROM_HAL_API */

/**
  \brief  The data struct of flash stub functions. ROM code functions are accessed in RAM code through stub functions.
*/
typedef struct hal_flash_func_stubs_s {
    void (*hal_flash_read_unique_id) (void *adaptor, uint8_t *buf, uint8_t len);
    hal_status_t (*hal_flash_read_id) (void *adaptor);
    void (*hal_flash_set_write_enable) (void *adaptor);
    void (*hal_flash_set_status) (void *adaptor, u8 cmd, u8 data);
    void (*hal_flash_set_status_no_check) (void *adaptor, u8 cmd, u8 data);
    void (*hal_flash_set_status_with_addr) (void *adaptor, u8 cmd, u8 addr, u8 data);
    void (*hal_flash_set_extended_addr) (void *adaptor, u8 data);
    void (*hal_flash_set_write_protect_mode) (void *adaptor, u8 mode);
    u8 (*hal_flash_get_status) (void *adaptor, u8 cmd);
    u8 (*hal_flash_get_status_with_addr) (void *adaptor, u8 cmd, u8 addr);
    u8 (*hal_flash_get_extended_addr) (void *adaptor);
    void (*hal_flash_wait_ready) (void *adaptor);
    void (*hal_flash_chip_erase) (void *adaptor);
    void (*hal_flash_64k_block_erase) (void *adaptor, u32 address);
    void (*hal_flash_32k_block_erase) (void *adaptor, u32 address);
    void (*hal_flash_sector_erase) (void *adaptor, u32 address);
    u8 (*hal_flash_query_sector_protect_state) (void *adaptor, u32 address);
    void (*hal_flash_protect_sector) (void *adaptor, u32 address);
    void (*hal_flash_unprotect_sector) (void *adaptor, u32 address);
    void (*hal_flash_global_lock) (void *adaptor);
    void (*hal_flash_global_unlock) (void *adaptor);
    void (*hal_flash_set_dummy_cycle) (void *adaptor);
    void (*hal_flash_set_quad_enable) (void *adaptor);
    void (*hal_flash_unset_quad_enable) (void *adaptor);
    void (*hal_flash_enable_qpi) (void *adaptor);
    void (*hal_flash_return_spi) (void *adaptor);
    void (*hal_flash_return_str) (void *adaptor);
    hal_status_t (*hal_flash_enter_power_down) (void *adaptor);
    hal_status_t (*hal_flash_release_from_power_down) (void *adaptor);
    void (*hal_flash_stream_read) (void *adaptor, u32 length, u32 addr, u8 *data);
    void (*hal_flash_stream_write) (void *adaptor, u32 length, u32 addr, u8 *data);
    void (*hal_flash_burst_read) (void *adaptor, u32 length, u32 addr, u8 *data);
    void (*hal_flash_burst_write) (void *adaptor, u32 length, u32 addr, u8 *data);
    void (*hal_flash_page_program) (void *adaptor, u32 length, u32 addr, u8 *data);
    void (*hal_flash_reset_to_spi) (void *adaptor);
    uint32_t reserved[10];  // reserved space for next ROM code version function table extending.
} hal_flash_func_stubs_t;


/** *@} */ /* End of group hs_hal_flash */


#endif

