/**************************************************************************//**
 * @file     rtl8710c_gpio.h
 * @brief    This file defines GPIO HAL macros and data types.
 *
 * @version  V1.00
 * @date     2016-11-18
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

/**
 * @addtogroup hs_hal_gpio GPIO
 * @ingroup 8710c_hal
 * @{
 * @brief The GPIO HAL module of the AmebaZ2 platform.
 */

#ifndef RTL8710C_GPIO_H
#define RTL8710C_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rtl8710c_gpio_type.h"

/**
  \brief  Defines the data type for GPIO pin name, a GPIO pin name is
          from combination of GPIO port and GPIO pin index.
 */
typedef struct gpio_pin_s {
    union {
        uint8_t pin_name;
        struct {
            uint8_t pin:5;              ///< bit:  4.. 0  the pin index in a port
            uint8_t port:3;             ///< bit:  7.. 5 the port index
        } pin_name_b;
    };
} gpio_pin_t, *pgpio_pin_t;

/**
  \brief  Defines the data type of GPIO port. The GPIO port data type is from the
          combination of port index, pin mask and the offset of the 1st pin.
 */
typedef struct gpio_port_s {
    uint32_t mask;                ///< the pin mask of the port
    uint8_t offset;               ///< the 1st pin offset
    uint8_t port;                 ///< the GPIO IP port index
    uint8_t reserved[2];
} gpio_port_t, *pgpio_port_t;

/// Defines the maximum GPIO pin in a port
#define GPIO_MAX_PIN_IDX            24     // the maximum pin number in a GPIO IP port
/// Defines the maximum GPIO interrupt pins. It is the number of the GPIO IRQ pins this platform can support.
#define GPIO_MAX_INT_PIN            16     // 0 ~ 15, one GPIO IP only support 16 configurable interrupt pin
/// Defines the maximum GPIO debounce pins. It is the number of GPIO pin can enable the debounce function simultaneously.
#define GPIO_MAX_DEBOUNCE_PIN       16     // 0 ~ 15, one GPIO IP only support 16 configurable de-bounce pin
/// define the GPIO port number. It's the number of the GPIO port this platform have.
#define GPIO_MAX_PORT_NUM           1      // 0 ~ 3, the port number of the GPIO IP

/**
  \brief  Defines the GPIO port name
 */
enum {
    GPIO_IP_PORT0         = 0,
    GPIO_IP_PORT1         = 1,
    GPIO_IP_PORT2         = 2,
    GPIO_IP_PORT3         = 3,

    GPIO_IP_MAX_PORT_ID   = 4
};

/**
  \brief  Defines GPIO interrupt polarity status
 */
enum {
    GPIO_LevelHigh      = 0,
    GPIO_LevelLow       = 1,
    GPIO_EdgeRising     = 0,
    GPIO_EdgeFalling    = 1,
    GPIO_EdgeDual       = 2
};

/**
  \brief  Defines GPIO interrupt trigger event type.
 */
enum  gpio_int_trigger_type_e {
    GPIO_IntType_None           = 0,
    GPIO_IntType_EdgeRising     = 1,
    GPIO_IntType_EdgeFalling    = 2,
    GPIO_IntType_LevelLow       = 3,
    GPIO_IntType_LevelHigh      = 4,
    GPIO_IntType_EdgeDual       = 5,

    GPIO_IntType_Invalid        = 0x0F
};
typedef uint8_t gpio_int_trig_type_t;

/**
  \brief  Defines GPIO pin direction.
 */
enum  gpio_dir_e {
    GPIO_IN                  = 0,
    GPIO_OUT                 = 1
};
typedef uint8_t gpio_dir_t;

/*! define GPIO Debounce clock source */
enum  gpio_deb_clk_e {
    GPIO_DebClk_LS32K      = 0,
    GPIO_DebClk_HS32K      = 0
};
typedef uint8_t gpio_deb_clk_t;

/**
  \brief  Defines GPIO Interrupt clock source.
 */
enum  gpio_int_clk_e {
    GPIO_IntClk_APB      = 0,
    GPIO_IntClk_32K      = 1    // Clock source same as debounce clock
};
typedef uint8_t gpio_int_clk_t;

/**
  \brief  Defines IO pad pull control type.
 */
enum pin_pull_ctrl_e {
    PullCtrl_HighZ = 0,
    PullCtrl_PullLow = 1,
    PullCtrl_PullHigh = 2
};

/**
  \brief  Defines IO pad pull type hardware supported.
 */
enum pin_pull_type_e {
    Pin_PullNone  = 0,
    Pin_PullDown  = 1,
    Pin_PullUp    = 2,
    Pin_PullDefault = Pin_PullNone
};
typedef uint8_t pin_pull_type_t;

/**
  \brief  Defines IO pad voltage.
 */
enum io_pad_vol_e {
    IO_1p8V  = 0,        // IO pad voltage = 1.8V
    IO_3p3V  = 1         // IO pad voltage = 3.3V
};
typedef uint8_t io_pad_vol_t;

/**
  \brief  Defines GPIO interrupt call back function type.
 */
typedef void (*gpio_irq_callback_t) (uint32_t int_id, gpio_int_trig_type_t int_type);

/**
  \brief  The data structure for a GPIO pin HAL operation.
*/
typedef struct hal_gpio_pin_adapter_s {
    uint8_t pin_name;               ///< chip pin name to locate the GPIO pin by port & pin index
    uint8_t port_idx;               ///< the GPIO IP port index of the GPIO pin
    uint8_t pin_idx;                ///< the GPIO IP pin index of the GPIO port
    uint8_t debounce_idx;           ///< the index of GPIO debounce function allocated to this GPIO pin, 0 ~ 15
    uint32_t bit_mask;              ///< the bit mask to read/write register

    volatile uint32_t *in_port;     ///< the IN port address
    volatile uint32_t *out0_port;   ///< the OUT port address for write 0
    volatile uint32_t *out1_port;   ///< the OUT port address for write 1
    volatile uint32_t *outt_port;   ///< the OUT port address for toggling
} hal_gpio_adapter_t, *phal_gpio_adapter_t;

/**
  \brief  The data structure for a GPIO IRQ pin HAL operation.
*/
typedef struct hal_gpio_irq_pin_adapter_s {
    uint8_t     pin_name;           ///< chip pin name to locate the GPIO pin by port & pin index
    uint8_t     ip_pin_name;        ///< IP pin name to locate the GPIO pin by port & pin index
    uint8_t     int_idx;            ///< the GPIO interrupt function index, 0 ~ 15
    uint8_t     debounce_idx;       ///< the GPIO debounce function index, 0 ~ 15
    uint8_t     int_type;           ///< the GPIO interrupt trigger type
    uint8_t     resv[3];
    uint32_t    bit_mask;           ///< the bit mask to read register
    volatile uint32_t    *in_port;  ///< the IN port address
    gpio_irq_callback_t irq_callback;   ///< the call-back function of the GPIO IRQ
    uint32_t irq_callback_arg;      ///< the argument of the IRQ call-back function, it's a priviate data will be pass to the application with the call back function.
    void *pnext;                    ///< point to the next gpio_irq_pin_adapter
} hal_gpio_irq_adapter_t, *phal_gpio_irq_adapter_t;

/**
  \brief  The data structure for a GPIO port HAL operation.
*/
typedef struct hal_gpio_port_adapter_s {
    uint8_t port_idx;               ///< the IP port index
    uint8_t pin_offset;             ///< the pin0 of this port which is located at the offset of the IP port
    uint8_t reserv0;
    uint8_t reserv1;
    uint32_t pin_mask;              ///< the mask of pin to indicates what pin is included
//    uint32_t bit_mask;            ///< the bit mask to read/write register
    volatile uint32_t *in_port;     ///< the IN port address
    volatile uint32_t *out0_port;   ///< the OUT port address for write 0
    volatile uint32_t *out1_port;   ///< the OUT port address for write 1
    volatile uint32_t *outt_port;   ///< the OUT port address for toggling
} hal_gpio_port_adapter_t, *phal_gpio_port_adapter_t;

/**
  \brief  The data structure for the GPIO Common resource menagement.
*/
typedef struct hal_gpio_comm_adapter_s {
    volatile phal_gpio_irq_adapter_t gpio_irq_list_head; ///< the head of the gpio_irq_pin_adapter list
    volatile phal_gpio_irq_adapter_t gpio_irq_list_tail; ///< the tail of the gpio_irq_pin_adapter list
    volatile uint32_t gpio_irq_using;        ///< the bit map to indicate using status of IRQ functions
    volatile uint32_t gpio_deb_using;        ///< the bit map to indicate using status of debounce functions
    volatile uint32_t critical_lv;           ///< to record GPIO HAL enter critical section level
    union {
        volatile uint32_t errs;
        struct {
            volatile uint32_t irq_err:1;     ///< IRQ handler error
            volatile uint32_t init_err:1;    ///< initial error
        } err_flag;
    };
} hal_gpio_comm_adapter_t, *phal_gpio_comm_adapter_t;

/**
  \brief Union type a gpio_ctrl
*/
typedef union {
        uint16_t w;                 /*!< GPIO Control Register                                             */

        struct {
            uint16_t pinmux_sel : 4;    /*!< [3..0] 000:SPIC/SDIO
                                                        001: JTAG/Test-SPIC
                                                        010: UART
                                                        011: SPI/WL-LED/EXT-32K
                                                        100: I2C/SIC
                                                        101: PWM
                                                        110: Wake/RFE-Ctrl
                                                        111: GPIO                                                  */
            uint16_t            : 2;
            uint16_t pull_ctrl : 2;     /*!< [7..6] 2b'00: high impedence; 2b'01: pull low; 2b'10: pull high;
                                                     2b'11: reserved                                               */
            uint16_t shdn_n : 1;        /*!< [8..8] PAD enable, 1: enable PAD, 0: shutdown                       */
            uint16_t smt_en : 1;        /*!< [9..9] Enable GPIOA0 Schmitt trigger; 1: enable                     */
            uint16_t driving : 2;       /*!< [11..10] (E3,E2)=(10:9) 1.8V: 00: 2mA; 01: 4mA; 10: 6mA; 11:
                                                          8mA 3.3V: 00: 4mA; 01: 6mA; 10: 12mA; 11: 16mA            */
            uint16_t            : 4;

      } b;                 /*!< bit fields for syson_gpio_ctrl */
} gpio_ctrl_t, *pgpio_ctrl_t;

/// @cond DOXYGEN_ROM_HAL_API

/**
 * @addtogroup hs_hal_gpio_rom_func GPIO HAL ROM APIs.
 * @ingroup hs_hal_gpio
 * @{
 */
#if !defined(CONFIG_BUILD_SECURE)

void hal_gpio_reg_irq_rtl8710c (irq_handler_t irq_handler);
void hal_gpio_set_irq_priority_rtl8710c (uint32_t priority);
void hal_gpio_comm_init_rtl8710c (phal_gpio_comm_adapter_t pgpio_comm_adp);
void hal_gpio_comm_deinit_rtl8710c (void);
void hal_gpio_enter_critical_rtl8710c (void);
void hal_gpio_exit_critical_rtl8710c (void);
hal_status_t hal_gpio_init_rtl8710c (phal_gpio_adapter_t pgpio_adapter, uint32_t pin_name);
void hal_gpio_deinit_rtl8710c (phal_gpio_adapter_t pgpio_adapter);
void hal_gpio_set_dir_rtl8710c (phal_gpio_adapter_t pgpio_adapter, gpio_dir_t dir);
gpio_dir_t hal_gpio_get_dir_rtl8710c (phal_gpio_adapter_t pgpio_adapter);
void hal_gpio_write_rtl8710c (phal_gpio_adapter_t pgpio_adapter, uint32_t io_data);
void hal_gpio_toggle_rtl8710c (phal_gpio_adapter_t pgpio_adapter);
uint32_t hal_gpio_read_rtl8710c (phal_gpio_adapter_t pgpio_adapter);
hal_status_t hal_gpio_debounce_enable_rtl8710c (phal_gpio_adapter_t pgpio_adapter, uint32_t debounce_us);
uint32_t hal_gpio_read_debounce_rtl8710c (phal_gpio_adapter_t pgpio_adapter);
void hal_gpio_debounce_disable_rtl8710c (phal_gpio_adapter_t pgpio_adapter);
hal_status_t hal_gpio_irq_init_rtl8710c (phal_gpio_irq_adapter_t pgpio_irq_adapter, uint32_t pin_name,
                gpio_irq_callback_t callback, uint32_t arg);
void hal_gpio_irq_deinit_rtl8710c (phal_gpio_irq_adapter_t pgpio_irq_adapter);
void hal_gpio_irq_set_trig_type_rtl8710c (phal_gpio_irq_adapter_t pgpio_adapter, gpio_int_trig_type_t int_type);
gpio_int_trig_type_t hal_gpio_irq_get_trig_type_rtl8710c (phal_gpio_irq_adapter_t pgpio_adapter);
void hal_gpio_irq_enable_rtl8710c (phal_gpio_irq_adapter_t pgpio_irq_adapter);
void hal_gpio_irq_disable_rtl8710c (phal_gpio_irq_adapter_t pgpio_irq_adapter);
hal_status_t hal_gpio_irq_debounce_enable_rtl8710c (phal_gpio_irq_adapter_t pgpio_irq_adapter, uint32_t debounce_us);
void hal_gpio_irq_debounce_disable_rtl8710c (phal_gpio_irq_adapter_t pgpio_irq_adapter);
uint32_t hal_gpio_irq_read_rtl8710c (phal_gpio_irq_adapter_t pgpio_irq_adapter);
hal_status_t hal_gpio_port_init_rtl8710c (phal_gpio_port_adapter_t pgpio_port_adapter, uint32_t port_idx,
                                    uint32_t mask, gpio_dir_t dir);
void hal_gpio_port_deinit_rtl8710c (phal_gpio_port_adapter_t pgpio_port_adapter);
void hal_gpio_port_write_rtl8710c (phal_gpio_port_adapter_t pgpio_port_adapter, uint32_t io_data);
uint32_t hal_gpio_port_read_rtl8710c (phal_gpio_port_adapter_t pgpio_port_adapter);
void hal_gpio_port_dir_rtl8710c (phal_gpio_port_adapter_t pgpio_port_adapter, gpio_dir_t dir);
#endif  // end of "#if !defined(CONFIG_BUILD_SECURE)"
hal_status_t hal_gpio_pull_ctrl_rtl8710c (uint32_t pin_name, pin_pull_type_t pull_type);
hal_status_t hal_gpio_schmitt_ctrl_rtl8710c (uint32_t pin_name, BOOLEAN ctrl, uint8_t v_h3l1);
hal_status_t hal_gpio_drive_ctrl_rtl8710c (uint32_t pin_name, uint8_t drv);
hal_status_t hal_gpio_get_ctrl_rtl8710c (uint32_t pin_name, gpio_ctrl_t *pctrl);


/** @} */ /* End of group hs_hal_gpio_rom_func */
/// @endcond /* End of condition DOXYGEN_ROM_HAL_API */


/**
  \brief  The data structure of the stubs functions of the GPIO HAL functions in ROM
*/
typedef struct hal_gpio_func_stubs_s {
    const psyson_gpio_ctrl_t pgpioa_ctrl_reg_tbl;
    const psyson_gpio_ctrl_t pgpiob_ctrl_reg_tbl;
    phal_gpio_comm_adapter_t *ppgpio_comm_adp;
    void (*hal_gpio_reg_irq) (irq_handler_t irq_handler);
    void (*hal_gpio_comm_init) (phal_gpio_comm_adapter_t pgpio_comm_adp);
    void (*hal_gpio_comm_deinit) (void);
    void (*hal_gpio_enter_critical) (void);
    void (*hal_gpio_exit_critical) (void);
    hal_status_t (*hal_gpio_init) (phal_gpio_adapter_t pgpio_adapter, uint32_t pin_name);
    void (*hal_gpio_deinit) (phal_gpio_adapter_t pgpio_adapter);
    void (*hal_gpio_set_dir) (phal_gpio_adapter_t pgpio_adapter, gpio_dir_t dir);
    gpio_dir_t (*hal_gpio_get_dir) (phal_gpio_adapter_t pgpio_adapter);
    void (*hal_gpio_write) (phal_gpio_adapter_t pgpio_adapter, uint32_t io_data);
    void (*hal_gpio_toggle) (phal_gpio_adapter_t pgpio_adapter);
    uint32_t (*hal_gpio_read) (phal_gpio_adapter_t pgpio_adapter);
    hal_status_t (*hal_gpio_debounce_enable) (phal_gpio_adapter_t pgpio_adapter, uint32_t debounce_us);
    uint32_t (*hal_gpio_read_debounce) (phal_gpio_adapter_t pgpio_adapter);
    void (*hal_gpio_debounce_disable) (phal_gpio_adapter_t pgpio_adapter);
    hal_status_t (*hal_gpio_irq_init) (phal_gpio_irq_adapter_t pgpio_irq_adapter,uint32_t pin_name,
                                        gpio_irq_callback_t callback, uint32_t arg);
    void (*hal_gpio_irq_deinit) (phal_gpio_irq_adapter_t pgpio_irq_adapter);
    void (*hal_gpio_irq_set_trig_type) (phal_gpio_irq_adapter_t pgpio_adapter, gpio_int_trig_type_t int_type);
    gpio_int_trig_type_t (*hal_gpio_irq_get_trig_type) (phal_gpio_irq_adapter_t pgpio_adapter);
    void (*hal_gpio_irq_enable) (phal_gpio_irq_adapter_t pgpio_irq_adapter);
    void (*hal_gpio_irq_disable) (phal_gpio_irq_adapter_t pgpio_irq_adapter);
    hal_status_t (*hal_gpio_irq_debounce_enable) (phal_gpio_irq_adapter_t pgpio_irq_adapter, uint32_t debounce_us);
    void (*hal_gpio_irq_debounce_disable) (phal_gpio_irq_adapter_t pgpio_irq_adapter);
    uint32_t (*hal_gpio_irq_read) (phal_gpio_irq_adapter_t pgpio_irq_adapter);
    hal_status_t (*hal_gpio_port_init) (phal_gpio_port_adapter_t pgpio_port_adapter, uint32_t port_idx,
                                        uint32_t mask, gpio_dir_t dir);
    void (*hal_gpio_port_deinit) (phal_gpio_port_adapter_t pgpio_port_adapter);
    void (*hal_gpio_port_write) (phal_gpio_port_adapter_t pgpio_port_adapter, uint32_t io_data);
    uint32_t (*hal_gpio_port_read) (phal_gpio_port_adapter_t pgpio_port_adapter);
    void (*hal_gpio_port_dir) (phal_gpio_port_adapter_t pgpio_port_adapter, gpio_dir_t dir);
    hal_status_t (*hal_gpio_pull_ctrl) (uint32_t pin_name, pin_pull_type_t pull_type);
    hal_status_t (*hal_gpio_schmitt_ctrl) (uint32_t pin_name, BOOLEAN ctrl, uint8_t v_h3l1);
    hal_status_t (*hal_gpio_drive_ctrl) (uint32_t pin_name, uint8_t drv);
    hal_status_t (*hal_gpio_get_ctrl) (uint32_t pin_name, gpio_ctrl_t *pctrl);
    uint32_t reserved[8];  // reserved space for next ROM code version function table extending.
} hal_gpio_func_stubs_t;

#if defined(CONFIG_BUILD_SECURE)
/**
  \brief  The data structure of the stubs functions of the GPIO HAL functions in Secure-domain ROM
*/
typedef struct hal_gpio_sfunc_stubs_s {
    const psyson_gpio_ctrl_t pgpioa_ctrl_reg_tbl;
    const psyson_gpio_ctrl_t pgpiob_ctrl_reg_tbl;
    hal_status_t (*hal_gpio_pull_ctrl) (uint32_t pin_name, pin_pull_type_t pull_type);
    hal_status_t (*hal_gpio_schmitt_ctrl) (uint32_t pin_name, BOOLEAN ctrl, uint8_t v_h3l1);
    hal_status_t (*hal_gpio_drive_ctrl) (uint32_t pin_name, uint8_t drv);
    hal_status_t (*hal_gpio_get_ctrl) (uint32_t pin_name, gpio_ctrl_t *pctrl);
    uint32_t reserved[8];  // reserved space for next ROM code version function table extending.
} hal_gpio_sfunc_stubs_t;
#endif

#ifdef __cplusplus
}
#endif

#endif /* RTL8710C_GPIO_H */


/** @} */ /* End of group hs_hal_gpio */

