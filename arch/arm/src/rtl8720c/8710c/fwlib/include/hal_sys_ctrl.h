/**************************************************************************//**
 * @file     hal_sys_ctrl.h
 * @brief    The system control functions declaration.
 * @version  v1.00
 * @date     2020/11/24
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
#ifndef HAL_SYS_CTRL_H
#define HAL_SYS_CTRL_H

#include "cmsis.h"

#define SYS_DATA_FLASH_BASE                 (0x1000)
#define SYS_DATA_OFFSET_FORCE_OLD_IMG       (SYS_DATA_FLASH_BASE + 0x08)
#define SYS_DATA_OFFSET_SPI_CFG             (SYS_DATA_FLASH_BASE + 0x20)
#define SYS_DATA_OFFSET_FLASH_INFO          (SYS_DATA_FLASH_BASE + 0x24)
#define SYS_DATA_OFFSET_ULOG_BAUD           (SYS_DATA_FLASH_BASE + 0x30)

#define CONFIG_EFUSE_SW_CHECK               (1)

#if (CONFIG_EFUSE_SW_CHECK==1) && (CONFIG_EFUSE_EN==1)

typedef enum chk_core_pwr_cali_status{
    CORPWR_CALI_LOAD_PASS_DONE_PASS                 =  0,
    CORPWR_CALI_LOAD_PASS_DONE_NONE                 = -1,
    CORPWR_CALI_AUTOLOAD_PIN_DISABLE                = -2,
    CORPWR_CALI_LOAD_PASS_PARSE_WEIRD               = -3,
    CORPWR_CALI_CHECK_EFUSE_ID_FAIL                 = -4,
    CORPWR_CALI_EFUSE_OTHER_FAIL                    = -5

} chk_core_pwr_cali_status_t;

chk_core_pwr_cali_status_t hal_efuse_check_core_pwr_cali_status (void);
#endif


/**
 * @brief  define debug port mode.
 */
enum dbg_port_mode_e {
    DBG_PORT_OFF    = 0,    ///< debug port off
    SWD_MODE        = 1,    ///< debugger use SWD mode
    JTAG_MODE       = 2,    ///< debugger use JTAG mode
};
typedef uint8_t     dbg_port_mode_t;

typedef union {
  uint32_t word;
  
  struct {
    uint32_t pin      : 5;       /*!< [4..0] GPIO trap pin selection: pin number */
    uint32_t port     : 1;       /*!< [5..5] GPIO trap pin selection: port number */
    uint32_t          : 1;
    uint32_t active   : 1;       /*!< [7..7] GPIO trap enable */
    uint32_t          : 24;
  } bit;
} sys_dat_force_old_img_t, *psys_dat_force_old_img_t;

enum sys_dat_spi_io_mode_e {
    SYS_DATA_SPI_QUAD_IO        = 0xFFFF,
    SYS_DATA_SPI_QUAD_OUTPUT    = 0x7FFF,
    SYS_DATA_SPI_DUAL_IO        = 0x3FFF,
    SYS_DATA_SPI_DUAL_OUTPUT    = 0x1FFF,
    SYS_DATA_SPI_ONE_IO         = 0x0FFF
};
typedef uint16_t    sys_dat_spi_io_mode_t;

enum sys_dat_spi_speed_e {
    SYS_DATA_SPI_100M           = 0xFFFF,
    SYS_DATA_SPI_50M            = 0x7FFF,
    SYS_DATA_SPI_25M            = 0x3FFF,
};
typedef uint16_t    sys_dat_spi_speed_t;

typedef union {
  uint32_t word;
  
  struct {
    uint32_t io_mode    : 16;       /*!< [15..0] SPI flash IO mode selection */
    uint32_t io_speed   : 16;       /*!< [31..16] SPI flash clock rate */
  } bit;
} sys_dat_spi_cfg_t, *psys_dat_spi_cfg_t;

enum sys_dat_flash_size_e {
    SYS_DATA_FLASH_SIZE_2M      = 0xFFFF,
    SYS_DATA_FLASH_SIZE_32M     = 0x7FFF,
    SYS_DATA_FLASH_SIZE_16M     = 0x3FFF,
    SYS_DATA_FLASH_SIZE_8M      = 0x1FFF,
    SYS_DATA_FLASH_SIZE_4M      = 0x0FFF,
    SYS_DATA_FLASH_SIZE_1M      = 0x03FF
};
typedef uint16_t    sys_dat_flash_size_t;

typedef union {
  uint32_t word;
  
  struct {
    uint32_t flash_id   : 16;       /*!< [15..0] flash chip ID, use this field only when the Get Flash ID command is failed */
    uint32_t flash_size : 16;       /*!< [31..16] flash size */
  } bit;
} sys_dat_flash_info_t, *psys_dat_flash_info_t;

typedef struct system_data_s {
    uint32_t                    reserved0;      // 0x00
    uint32_t                    reserved1;      // 0x04
    sys_dat_force_old_img_t     old_img_trap;   // 0x08: GPIO trap of force boot from older image2
    uint32_t                    reserved2;      // 0x0C
    uint32_t                    rdp_reserved[4];    // 0x10
    sys_dat_spi_cfg_t           spi_cfg;        // 0x20: SPI flash controller configuration
    sys_dat_flash_info_t        flash_info;     // 0x24: SPI flash chip information
    uint32_t                    reserved3[2];   // 0x28
    uint32_t                    ulog_baud;      // 0x30: Log UART baud rate setting
    uint32_t                    reserved4[3];   // 0x34
} system_data_t, *psystem_data_t;

/**
  \brief  Define the pin selection for flash port selection.
*/
enum flash_port_sel_e {
    FLASH_PORTAL    = 0,    /*!< flash pin on A[7, 8, 11, 10, (9, 12)] */
    FLASH_PORTB     = 1,    /*!< flash pin on B[8, 11, 12, 7, (6, 10)], Flash MCM */
    FLASH_PORTAH    = 2     /*!< flash pin on A[15,16, 19, 20, (17,18)] */
};
typedef uint8_t    flash_port_sel_t;

/**
 * @brief  define debug port mode.
 */
enum hal_reset_reason_e {
    HAL_RESET_REASON_POWER_ON         = 0,    ///< power on
    HAL_RESET_REASON_SOFTWARE        = BIT0,    ///< software reset
    HAL_RESET_REASON_WATCHDOG        = BIT1,    ///< watchdog reset
    HAL_RESET_REASON_JTAG        = BIT2,    ///< jtag reset
};
typedef uint32_t     hal_reset_reason_t;

/// The data structure to record a NS region configuration
typedef struct ns_region_s {
    uint32_t start_addr;   /*!< the start address of NS region */  
    uint32_t end_addr;   /*!< the end address of NS region */  
    uint32_t is_valid;   /*!< indicates the valid state of this record */
} ns_region_t, *pns_region_t;

/**
 *  @brief SDIO device enable control.
 *  @details Enable or disable the SDIO device function, clock and pins.
 *  @param[in]   en  The enable control.
 *                 - 1 = Enable.
 *                 - 0 = Disable.
 *  @return      void
 *
 */
void hal_syson_sdio_dev_ctrl(BOOL en);

/**
 *  @brief Audio codec device enable control.
 *  @details Enable or disable the audio codec device function, clock and pins.
 *  @param[in]   en  The enable control.
 *                 - 1 = Enable.
 *                 - 0 = Disable.
 *  @return      void
 *
 */
void hal_syson_audio_dev_ctrl(BOOL en);

void hal_syson_spic_dev_ctrl(BOOL en);

/**
 *  @brief I2C filter enable control.
 *  @param[in]   en  The enable control.
 *                 - 1 = Enable.
 *                 - 0 = Disable.
 *  @return      void
 *
 */
void hal_syson_i2c_filter_ctrl(BOOL en);

/**
 *  @brief I2C calibration input enable control.
 *  @param[in]   en  The enable control.
 *                 - 1 = Enable.
 *                 - 0 = Disable.
 *  @return      void
 *
 */
void hal_syson_i2c_cali_in_ctrl(BOOL en);

/** 
 *  @brief Wakeup UART function reset.
 *  @return      void
 *
 */
void hal_syson_wakeup_uart_func_reset(void);

/** 
 *  @brief To get chip version.
 *
 *  @returns The chip version:
 *           0: A-Cut; 1: B-Cut
 */
uint8_t hal_get_chip_ver (void);

/** 
 *  @brief To get the eFuse configuration for the flash port selection.
 *
 *  @returns The eFuse setting for the flash port selection
 *           0: flash port on A[7, 8, 11, 10, (9, 12)]
 *           1: flash port on B[8, 11, 12, 7, (6, 10)], Flash MCM
 *           2: flash port on A[15,16, 19, 20, (17,18)]
 */
flash_port_sel_t hal_get_flash_port_cfg (void);

#if !defined(CONFIG_BUILD_NONSECURE)
// Build for Secure/Ignore-Secure

void hal_sys_life_cycle_state_read (uint8_t *pstate);

hal_status_t hal_sys_life_cycle_state_write (const uint8_t w_state);

/** 
 *  @brief Configures debuger port setting.
 *  @param[in]   dbg_mode  The platform debug port mode selection, JTAG or SWD.
 *                 - DBG_PORT_OFF  disable debugger port.
 *                 - SWD_MODE  selectes the SWD mode.
 *                 - JTAG_MODE  selects the JTAG mode.
 *
 *  @return      HAL_ERR_PARA  One or more input arguments is/are invalid.
 *  @return      HAL_OK  Debugger port configures OK.
 *
 */
hal_status_t hal_dbg_port_cfg(dbg_port_mode_t dbg_mode);

#else   // else of "#if !defined(CONFIG_BUILD_NONSECURE)"

#define hal_sys_life_cycle_state_read   hal_sys_life_cycle_state_read_nsc
#define hal_sys_life_cycle_state_write  hal_sys_life_cycle_state_write_nsc

// Build for Non-Secure
hal_status_t hal_dbg_port_cfg_nsc (dbg_port_mode_t dbg_mode);

#define hal_dbg_port_cfg             hal_dbg_port_cfg_nsc

#endif  // end of else of "#if !defined(CONFIG_BUILD_NONSECURE)"

#if defined(CONFIG_BUILD_SECURE)
void NS_ENTRY hal_sys_life_cycle_state_read_nsc(uint8_t *pstate);

hal_status_t NS_ENTRY hal_sys_life_cycle_state_write_nsc(const uint8_t w_state);
#endif

/** 
 *  @brief Get device reset reason.
 *  @details To get the reset reason.
 *  @param[in]   reason  The reason code.
 *  @return      HAL_OK  reset reason read OK.
 */
hal_status_t rtl8710c_reset_reason_get (hal_reset_reason_t *reason);

/** 
 *  @brief Set device reset reason.
 *  @details To set reason before reset, only the last reason code will be stored.
 *  @param[in]   reason  The reason code.
 *  @return      HAL_OK  reset reason set OK.
 */
hal_status_t rtl8710c_reset_reason_set (hal_reset_reason_t reason);

/** 
 *  @brief Clear device boot reason.
 *  @details To clear reset reason.
 *  @param[in]   reason  The reason code.
 *  @return      HAL_OK  reset reason clear OK.
 */
hal_status_t rtl8710c_reset_reason_clear (hal_reset_reason_t reason);

#endif  // end of "#ifndef HAL_SYS_CTRL_H"

