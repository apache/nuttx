/**************************************************************************//**
 * @file     rtl8710c_pin_name.h
 * @brief    Define the IC pin name and IO port name
 * @version  V1.00
 * @date     2016-07-20
 *
 * @note
 *
 ******************************************************************************
 *
 * Copyright(c) 2007 - 2016 Realtek Corporation. All rights reserved.
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

#ifndef RTL8710C_PIN_NAME_H
#define RTL8710C_PIN_NAME_H

#ifdef __cplusplus
extern "C" {
#endif

/// Defines the macro to convert port index and pin index to a pin name
#define PIN_NAME(port_id, pin_id)           (((port_id) << 5) | (pin_id))
/// Defines the macro to get the port index by the given pin name
#define PIN_NAME_2_PORT(pin_name)           (((pin_name) >> 5) & 0x07)
/// Defines the macro to get the pin index by the given pin name
#define PIN_NAME_2_PIN(pin_name)            ((pin_name) & 0x1F)

/// Defines maximum number of pin in a GPIO port
#define MAX_PIN_IN_PORT     24

/**
  \brief  Defines Chip's IO port name.
 */
enum {
    PORT_A         = 0,
    PORT_B         = 1,

    PORT_MAX_NUM   = 2
};

// virtual internal port, no pin out
#define PORT_INTERNAL       (4)

/**
  \brief  Defines Chip's IO pin name.
 */
enum {
    PIN_A0         = PIN_NAME(PORT_A, 0),
    PIN_A1         = PIN_NAME(PORT_A, 1),
    PIN_A2         = PIN_NAME(PORT_A, 2),
    PIN_A3         = PIN_NAME(PORT_A, 3),
    PIN_A4         = PIN_NAME(PORT_A, 4),
    PIN_A5         = PIN_NAME(PORT_A, 5),
    PIN_A6         = PIN_NAME(PORT_A, 6),
    PIN_A7         = PIN_NAME(PORT_A, 7),
    PIN_A8         = PIN_NAME(PORT_A, 8),
    PIN_A9         = PIN_NAME(PORT_A, 9),
    PIN_A10        = PIN_NAME(PORT_A, 10),
    PIN_A11        = PIN_NAME(PORT_A, 11),
    PIN_A12        = PIN_NAME(PORT_A, 12),
    PIN_A13        = PIN_NAME(PORT_A, 13),
    PIN_A14        = PIN_NAME(PORT_A, 14),
    PIN_A15        = PIN_NAME(PORT_A, 15),
    PIN_A16        = PIN_NAME(PORT_A, 16),
    PIN_A17        = PIN_NAME(PORT_A, 17),
    PIN_A18        = PIN_NAME(PORT_A, 18),
    PIN_A19        = PIN_NAME(PORT_A, 19),
    PIN_A20        = PIN_NAME(PORT_A, 20),
    PIN_A21        = PIN_NAME(PORT_A, 21),
    PIN_A22        = PIN_NAME(PORT_A, 22),
    PIN_A23        = PIN_NAME(PORT_A, 23),

    PIN_B0         = PIN_NAME(PORT_B, 0),
    PIN_B1         = PIN_NAME(PORT_B, 1),
    PIN_B2         = PIN_NAME(PORT_B, 2),
    PIN_B3         = PIN_NAME(PORT_B, 3),
    PIN_B4         = PIN_NAME(PORT_B, 4),
    PIN_B5         = PIN_NAME(PORT_B, 5),
    PIN_B6         = PIN_NAME(PORT_B, 6),
    PIN_B7         = PIN_NAME(PORT_B, 7),
    PIN_B8         = PIN_NAME(PORT_B, 8),
    PIN_B9         = PIN_NAME(PORT_B, 9),
    PIN_B10        = PIN_NAME(PORT_B, 10),
    PIN_B11        = PIN_NAME(PORT_B, 11),
    PIN_B12        = PIN_NAME(PORT_B, 12),

// Virtual pin (internal pin, no pin out)
    PIN_UART3_TX   = PIN_NAME(PORT_INTERNAL, 0),
    PIN_UART3_RX   = PIN_NAME(PORT_INTERNAL, 1),
    PIN_UART3_RTS  = PIN_NAME(PORT_INTERNAL, 2),
    PIN_UART3_CTS  = PIN_NAME(PORT_INTERNAL, 3),
    
    PIN_NC         = 0xFF,
    PIN_LIST_END   = 0xFF
};
typedef uint8_t pin_name_t;

/**
  \brief  Defines the data type for IO pin.
 */
typedef struct io_pin_s {
    union {
        uint8_t pin_name;
        struct {
            uint8_t pin:5;              ///< bit:  4.. 0  the pin index in a port
            uint8_t port:3;             ///< bit:  7.. 5 the IO port index
        } pin_name_b;
    };
} io_pin_t, *pio_pin_t;

/**
  \brief  Defines the pin mux selection.
 */
enum  _pin_sel_e {
    PinSel0       = 0,
    PinSel1       = 1,
    PinSel2       = 2,
    PinSel3       = 3,
    PinSel4       = 4
};
typedef uint8_t pin_sel_t;

/**
  \brief  Defines the pin of SDIO device.
 */
enum  sdio_pins_e {
    PIN_SDIO_D2     = PIN_A15,
    PIN_SDIO_D3     = PIN_A16,
    PIN_SDIO_CMD    = PIN_A17,
    PIN_SDIO_CLK    = PIN_A18,
    PIN_SDIO_D0     = PIN_A19,
    PIN_SDIO_D1     = PIN_A20,
    PIN_SDIO_INT    = PIN_A14
};

#ifdef __cplusplus
}
#endif

#endif /* RTL8710C_PIN_NAME_H */


