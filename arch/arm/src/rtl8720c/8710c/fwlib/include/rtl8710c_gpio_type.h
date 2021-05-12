/**************************************************************************//**
 * @file      rtl8710c_gpio_type.h
 * @brief
 * @version   V1.00
 * @date      2018-1-4 15:41:31
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

#ifndef _RTL8710C_GPIO_TYPE_H_
#define _RTL8710C_GPIO_TYPE_H_

#ifdef  __cplusplus
extern "C"
{
#endif

/// @cond DOXYGEN_GENERAL_REG_TYPE || DOXYGEN_GPIO_REG_TYPE

/**
 * @addtogroup hs_hal_gpio_reg GPIO Registers.
 * @ingroup hs_hal_gpio
 * @{
 */

/**
  \brief Union type to access gpio_it_sts (@ 0x00000000).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000000) GPIO interrupt type status Register                        */
  
  struct {
    __IM  uint32_t gpio_it_sts : 16;          /*!< [15..0] For each bit: 0: the specific GPIO pin is configured
                                                   to edge sensitive interrupt mode 1: the specific GPIO pin
                                                   is configured to levle sensitive interrupt mode.                          */
  } b;                                        /*!< bit fields for gpio_it_sts */
} gpio_it_sts_t, *pgpio_it_sts_t;

/**
  \brief Union type to access gpio_ei_en (@ 0x00000004).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000004) GPIO edge sensitive interrupt mode enable                  */
  
  struct {
    __IOM uint32_t ei_en      : 16;           /*!< [15..0] For each bit write: 0: No operation 1: the conrtolled
                                                   GPIO pin is configured as edge sensitive interrupt mode
                                                   and also cause the corresponging bit of REG_GPIO_IT_STS
                                                   to be read as zero.                                                       */
  } b;                                        /*!< bit fields for gpio_ei_en */
} gpio_ei_en_t, *pgpio_ei_en_t;

/**
  \brief Union type to access gpio_li_en (@ 0x00000008).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000008) GPIO level sensitive interrupt mode enable                 */
  
  struct {
    __IOM uint32_t li_en      : 16;           /*!< [15..0] For each bit write: 0: No operation 1: the conrtolled
                                                   GPIO pin is configured as level sensitive interrupt mode
                                                   and also cause the corresponging bit of REG_GPIO_IT_STS
                                                   to be read as 1.                                                          */
  } b;                                        /*!< bit fields for gpio_li_en */
} gpio_li_en_t, *pgpio_li_en_t;

/**
  \brief Union type to access gpio_ip_sts (@ 0x0000000C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000000C) GPIO interrupt polarity status                             */
  
  struct {
    __IM  uint32_t ip_sts0    : 2;            /*!< [1..0] 00: the specific GPIO pin is configured as rising-edge
                                                   (active-high sensitive) 01: the specific GPIO pin is configured
                                                   as falling-edge (active-low sensitive) 10: the specific
                                                   GPIO pin is configured as dual-edge 11: reserved                          */
    __IM  uint32_t ip_sts1    : 2;            /*!< [3..2] 00: the specific GPIO pin is configured as rising-edge
                                                   (active-high sensitive) 01: the specific GPIO pin is configured
                                                   as falling-edge (active-low sensitive) 10: the specific
                                                   GPIO pin is configured as dual-edge 11: reserved                          */
    __IM  uint32_t ip_sts2    : 2;            /*!< [5..4] 00: the specific GPIO pin is configured as rising-edge
                                                   (active-high sensitive) 01: the specific GPIO pin is configured
                                                   as falling-edge (active-low sensitive) 10: the specific
                                                   GPIO pin is configured as dual-edge 11: reserved                          */
    __IM  uint32_t ip_sts3    : 2;            /*!< [7..6] 00: the specific GPIO pin is configured as rising-edge
                                                   (active-high sensitive) 01: the specific GPIO pin is configured
                                                   as falling-edge (active-low sensitive) 10: the specific
                                                   GPIO pin is configured as dual-edge 11: reserved                          */
    __IM  uint32_t ip_sts4    : 2;            /*!< [9..8] 00: the specific GPIO pin is configured as rising-edge
                                                   (active-high sensitive) 01: the specific GPIO pin is configured
                                                   as falling-edge (active-low sensitive) 10: the specific
                                                   GPIO pin is configured as dual-edge 11: reserved                          */
    __IM  uint32_t ip_sts5    : 2;            /*!< [11..10] 00: the specific GPIO pin is configured as rising-edge
                                                   (active-high sensitive) 01: the specific GPIO pin is configured
                                                   as falling-edge (active-low sensitive) 10: the specific
                                                   GPIO pin is configured as dual-edge 11: reserved                          */
    __IM  uint32_t ip_sts6    : 2;            /*!< [13..12] 00: the specific GPIO pin is configured as rising-edge
                                                   (active-high sensitive) 01: the specific GPIO pin is configured
                                                   as falling-edge (active-low sensitive) 10: the specific
                                                   GPIO pin is configured as dual-edge 11: reserved                          */
    __IM  uint32_t ip_sts7    : 2;            /*!< [15..14] 00: the specific GPIO pin is configured as rising-edge
                                                   (active-high sensitive) 01: the specific GPIO pin is configured
                                                   as falling-edge (active-low sensitive) 10: the specific
                                                   GPIO pin is configured as dual-edge 11: reserved                          */
    __IM  uint32_t ip_sts8    : 2;            /*!< [17..16] 00: the specific GPIO pin is configured as rising-edge
                                                   (active-high sensitive) 01: the specific GPIO pin is configured
                                                   as falling-edge (active-low sensitive) 10: the specific
                                                   GPIO pin is configured as dual-edge 11: reserved                          */
    __IM  uint32_t ip_sts9    : 2;            /*!< [19..18] 00: the specific GPIO pin is configured as rising-edge
                                                   (active-high sensitive) 01: the specific GPIO pin is configured
                                                   as falling-edge (active-low sensitive) 10: the specific
                                                   GPIO pin is configured as dual-edge 11: reserved                          */
    __IM  uint32_t ip_sts10   : 2;            /*!< [21..20] 00: the specific GPIO pin is configured as rising-edge
                                                   (active-high sensitive) 01: the specific GPIO pin is configured
                                                   as falling-edge (active-low sensitive) 10: the specific
                                                   GPIO pin is configured as dual-edge 11: reserved                          */
    __IM  uint32_t ip_sts11   : 2;            /*!< [23..22] 00: the specific GPIO pin is configured as rising-edge
                                                   (active-high sensitive) 01: the specific GPIO pin is configured
                                                   as falling-edge (active-low sensitive) 10: the specific
                                                   GPIO pin is configured as dual-edge 11: reserved                          */
    __IM  uint32_t ip_sts12   : 2;            /*!< [25..24] 00: the specific GPIO pin is configured as rising-edge
                                                   (active-high sensitive) 01: the specific GPIO pin is configured
                                                   as falling-edge (active-low sensitive) 10: the specific
                                                   GPIO pin is configured as dual-edge 11: reserved                          */
    __IM  uint32_t ip_sts13   : 2;            /*!< [27..26] 00: the specific GPIO pin is configured as rising-edge
                                                   (active-high sensitive) 01: the specific GPIO pin is configured
                                                   as falling-edge (active-low sensitive) 10: the specific
                                                   GPIO pin is configured as dual-edge 11: reserved                          */
    __IM  uint32_t ip_sts14   : 2;            /*!< [29..28] 00: the specific GPIO pin is configured as rising-edge
                                                   (active-high sensitive) 01: the specific GPIO pin is configured
                                                   as falling-edge (active-low sensitive) 10: the specific
                                                   GPIO pin is configured as dual-edge 11: reserved                          */
    __IM  uint32_t ip_sts15   : 2;            /*!< [31..30] The interrupt polarity status of GPIO15                          */
  } b;                                        /*!< bit fields for gpio_ip_sts */
} gpio_ip_sts_t, *pgpio_ip_sts_t;

/**
  \brief Union type to access gpio_ir_en (@ 0x00000014).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000014) GPIO interrupt rising-edge enable                          */
  
  struct {
    __IOM uint32_t ir_en      : 16;           /*!< [15..0] For each bit write: 0: No operation; 1: the conrtolled
                                                   GPIO pin is configured to rising-edge interrupt mode or
                                                   high-level mode (depends on REG_GPIO_IT_STS) and also cause
                                                   the corresponding bits of REG_GPIO_IP_STS to be read as
                                                   00b.                                                                      */
  } b;                                        /*!< bit fields for gpio_ir_en */
} gpio_ir_en_t, *pgpio_ir_en_t;

/**
  \brief Union type to access gpio_if_en (@ 0x00000018).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000018) GPIO interrupt falling-edge enable                         */
  
  struct {
    __IOM uint32_t if_en      : 16;           /*!< [15..0] For each bit write: 0: No operation; 1: the conrtolled
                                                   GPIO pin is configured to falling-edge interrupt mode or
                                                   low-level interrupt mode (depends on REG_GPIO_IT_STS) and
                                                   cause the corresponding bits of REG_GPIO_IP_STS to be read
                                                   as 01b.                                                                   */
  } b;                                        /*!< bit fields for gpio_if_en */
} gpio_if_en_t, *pgpio_if_en_t;

/**
  \brief Union type to access gpio_id_en (@ 0x0000001C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000001C) GPIO interrupt dual-edge enable                            */
  
  struct {
    __IOM uint32_t id_en      : 16;           /*!< [15..0] For each bit write: 0: No operation; 1: the conrtolled
                                                   GPIO pin is configured to dual-edge interrupt mode and
                                                   also cause the corresponding bits of REG_GPIO_IP_STS to
                                                   be read as 10b.                                                           */
  } b;                                        /*!< bit fields for gpio_id_en */
} gpio_id_en_t, *pgpio_id_en_t;

/**
  \brief Union type to access gpio_ie_sts (@ 0x00000020).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000020) GPIO interrupt enable status                               */
  
  struct {
    __IM  uint32_t ie_sts     : 16;           /*!< [15..0] For each bit: 0: the interrupt of the specific GPIO
                                                   pin is disabled 1: the interrupt of the specific GPIO pin
                                                   is enabled                                                                */
  } b;                                        /*!< bit fields for gpio_ie_sts */
} gpio_ie_sts_t, *pgpio_ie_sts_t;

/**
  \brief Union type to access gpio_int_en (@ 0x00000024).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000024) GPIO interrupt enable                                      */
  
  struct {
    __IOM uint32_t int_en     : 16;           /*!< [15..0] For each bit write: 0: No operation 1: to enable the
                                                   specific GPIO INT and also cause the specified bit of REG_GPIO_IE_STS
                                                   to be read as ONE.                                                        */
  } b;                                        /*!< bit fields for gpio_int_en */
} gpio_int_en_t, *pgpio_int_en_t;

/**
  \brief Union type to access gpio_int_dis (@ 0x00000028).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000028) GPIO interrupt disable                                     */
  
  struct {
    __IOM uint32_t int_dis    : 16;           /*!< [15..0] For each bit write: 0: No operation 1: to disable the
                                                   specified GPIO INT and also cause the specified bit of
                                                   REG_GPIO_IE_STS to be read as ZERO.                                       */
  } b;                                        /*!< bit fields for gpio_int_dis */
} gpio_int_dis_t, *pgpio_int_dis_t;

/**
  \brief Union type to access gpio_int_raw_sts (@ 0x0000002C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000002C) GPIO RAW interrupt status                                  */
  
  struct {
    __IM  uint32_t int_raw_sts : 16;          /*!< [15..0] For each bit: 0: the specified GPIO pin has no pending
                                                   interrupt 1: the specified GPIO pin's interrupt is pending
                                                   (permasking)                                                              */
  } b;                                        /*!< bit fields for gpio_int_raw_sts */
} gpio_int_raw_sts_t, *pgpio_int_raw_sts_t;

/**
  \brief Union type to access gpio_int_sts (@ 0x00000030).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000030) GPIO interrupt status                                      */
  
  struct {
    __IM  uint32_t int_sts    : 16;           /*!< [15..0] For each bit: 0: the specified GPIO pin has no pending
                                                   interrupt 1: the specified GPIO pin's interrupt is pending                */
  } b;                                        /*!< bit fields for gpio_int_sts */
} gpio_int_sts_t, *pgpio_int_sts_t;

/**
  \brief Union type to access gpio_int_clr (@ 0x00000034).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000034) GPIO interrupt status clear                                */
  
  struct {
    __IOM uint32_t int_clr    : 16;           /*!< [15..0] For each bit writting: 0: No operation; 1: Clear edge
                                                   type pending interrupt of the conrtolled GPIO pin and clear
                                                   the specified bits of REG_GPIO_INT_STS as ZERO                            */
  } b;                                        /*!< bit fields for gpio_int_clr */
} gpio_int_clr_t, *pgpio_int_clr_t;

/**
  \brief Union type to access gpio_int_func_en_sts (@ 0x00000038).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000038) GPIO interrupt function enable status                      */
  
  struct {
    __IM  uint32_t int_en_sts : 16;           /*!< [15..0] For each bit: 0: the specified GPIO INT is Disable 1:
                                                   the specified GPIO INT is Enable                                          */
  } b;                                        /*!< bit fields for gpio_int_func_en_sts */
} gpio_int_func_en_sts_t, *pgpio_int_func_en_sts_t;

/**
  \brief Union type to access gpio_int_func_en (@ 0x0000003C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000003C) GPIO interrupt function enable                             */
  
  struct {
    __IOM uint32_t int_en     : 16;           /*!< [15..0] For each bit write: 0: No operation 1: the specified
                                                   GPIO INT is enabled and also cause the specified bit of
                                                   REG_GPIO_INT_FUNC_EN_STS to be read as ONE.                               */
  } b;                                        /*!< bit fields for gpio_int_func_en */
} gpio_int_func_en_t, *pgpio_int_func_en_t;

/**
  \brief Union type to access gpio_int_func_dis (@ 0x00000040).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000040) GPIO interrupt function disable                            */
  
  struct {
    __IOM uint32_t int_dis    : 16;           /*!< [15..0] For each bit write: 0: No operation 1: the specified
                                                   GPIO INT is disabled and also cause the specified bit of
                                                   REG_GPIO_INT_FUNC_EN_STS to be read as ZERO.                              */
  } b;                                        /*!< bit fields for gpio_int_func_dis */
} gpio_int_func_dis_t, *pgpio_int_func_dis_t;

/**
  \brief Union type to access gpio_int0_sel (@ 0x00000050).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000050) GPIO INT0 selection                                        */
  
  struct {
    __IOM uint32_t int_mer_sel : 5;           /*!< [4..0] Pin selection of specified port for GPIO INT0                      */
    __IOM uint32_t int_gp_sel : 2;            /*!< [6..5] GPIO port selection for GPIO INT0                                  */
    __IM  uint32_t            : 1;
    __IOM uint32_t int_deb_sel : 4;           /*!< [11..8] Debounce Output signal selection for GPIO INT0                    */
    __IM  uint32_t            : 3;
    __IOM uint32_t int_sur_sel : 1;           /*!< [15..15] 0: Useing pin input signal as interrupt signal source
                                                   1: Using debounce output signal as interrupt signal source                */
  } b;                                        /*!< bit fields for gpio_int0_sel */
} gpio_int0_sel_t, *pgpio_int0_sel_t;

/**
  \brief Union type to access gpio_int1_sel (@ 0x00000054).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000054) GPIO INT1 selection                                        */
  
  struct {
    __IOM uint32_t int_mer_sel : 5;           /*!< [4..0] Pin selection of specified port for GPIO INT1                      */
    __IOM uint32_t int_gp_sel : 2;            /*!< [6..5] GPIO port selection for GPIO INT1                                  */
    __IM  uint32_t            : 1;
    __IOM uint32_t int_deb_sel : 4;           /*!< [11..8] Debounce Output signal selection for GPIO INT1                    */
    __IM  uint32_t            : 3;
    __IOM uint32_t int_sur_sel : 1;           /*!< [15..15] 0: Useing pin input signal as interrupt signal source
                                                   1: Using debounce output signal as interrupt signal source                */
  } b;                                        /*!< bit fields for gpio_int1_sel */
} gpio_int1_sel_t, *pgpio_int1_sel_t;

/**
  \brief Union type to access gpio_int2_sel (@ 0x00000058).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000058) GPIO INT2 selection                                        */
  
  struct {
    __IOM uint32_t int_mer_sel : 5;           /*!< [4..0] Pin selection of specified port for GPIO INT2                      */
    __IOM uint32_t int_gp_sel : 2;            /*!< [6..5] GPIO port selection for GPIO INT2                                  */
    __IM  uint32_t            : 1;
    __IOM uint32_t int_deb_sel : 4;           /*!< [11..8] Debounce Output signal selection for GPIO INT2                    */
    __IM  uint32_t            : 3;
    __IOM uint32_t int_sur_sel : 1;           /*!< [15..15] 0: Useing pin input signal as interrupt signal source
                                                   1: Using debounce output signal as interrupt signal source                */
  } b;                                        /*!< bit fields for gpio_int2_sel */
} gpio_int2_sel_t, *pgpio_int2_sel_t;

/**
  \brief Union type to access gpio_int3_sel (@ 0x0000005C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000005C) GPIO INT3 selection                                        */
  
  struct {
    __IOM uint32_t int_mer_sel : 5;           /*!< [4..0] Pin selection of specified port for GPIO INT3                      */
    __IOM uint32_t int_gp_sel : 2;            /*!< [6..5] GPIO port selection for GPIO INT3                                  */
    __IM  uint32_t            : 1;
    __IOM uint32_t int_deb_sel : 4;           /*!< [11..8] Debounce Output signal selection for GPIO INT3                    */
    __IM  uint32_t            : 3;
    __IOM uint32_t int_sur_sel : 1;           /*!< [15..15] 0: Useing pin input signal as interrupt signal source
                                                   1: Using debounce output signal as interrupt signal source                */
  } b;                                        /*!< bit fields for gpio_int3_sel */
} gpio_int3_sel_t, *pgpio_int3_sel_t;

/**
  \brief Union type to access gpio_int4_sel (@ 0x00000060).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000060) GPIO INT4 selection                                        */
  
  struct {
    __IOM uint32_t int_mer_sel : 5;           /*!< [4..0] Pin selection of specified port for GPIO INT4                      */
    __IOM uint32_t int_gp_sel : 2;            /*!< [6..5] GPIO port selection for GPIO INT4                                  */
    __IM  uint32_t            : 1;
    __IOM uint32_t int_deb_sel : 4;           /*!< [11..8] Debounce Output signal selection for GPIO INT4                    */
    __IM  uint32_t            : 3;
    __IOM uint32_t int_sur_sel : 1;           /*!< [15..15] 0: Useing pin input signal as interrupt signal source
                                                   1: Using debounce output signal as interrupt signal source                */
  } b;                                        /*!< bit fields for gpio_int4_sel */
} gpio_int4_sel_t, *pgpio_int4_sel_t;

/**
  \brief Union type to access gpio_int5_sel (@ 0x00000064).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000064) GPIO INT5 selection                                        */
  
  struct {
    __IOM uint32_t int_mer_sel : 5;           /*!< [4..0] Pin selection of specified port for GPIO INT5                      */
    __IOM uint32_t int_gp_sel : 2;            /*!< [6..5] GPIO port selection for GPIO INT5                                  */
    __IM  uint32_t            : 1;
    __IOM uint32_t int_deb_sel : 4;           /*!< [11..8] Debounce Output signal selection for GPIO INT5                    */
    __IM  uint32_t            : 3;
    __IOM uint32_t int_sur_sel : 1;           /*!< [15..15] 0: Useing pin input signal as interrupt signal source
                                                   1: Using debounce output signal as interrupt signal source                */
  } b;                                        /*!< bit fields for gpio_int5_sel */
} gpio_int5_sel_t, *pgpio_int5_sel_t;

/**
  \brief Union type to access gpio_int6_sel (@ 0x00000068).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000068) GPIO INT6 selection                                        */
  
  struct {
    __IOM uint32_t int_mer_sel : 5;           /*!< [4..0] Pin selection of specified port for GPIO INT6                      */
    __IOM uint32_t int_gp_sel : 2;            /*!< [6..5] GPIO port selection for GPIO INT6                                  */
    __IM  uint32_t            : 1;
    __IOM uint32_t int_deb_sel : 4;           /*!< [11..8] Debounce Output signal selection for GPIO INT6                    */
    __IM  uint32_t            : 3;
    __IOM uint32_t int_sur_sel : 1;           /*!< [15..15] 0: Useing pin input signal as interrupt signal source
                                                   1: Using debounce output signal as interrupt signal source                */
  } b;                                        /*!< bit fields for gpio_int6_sel */
} gpio_int6_sel_t, *pgpio_int6_sel_t;

/**
  \brief Union type to access gpio_int7_sel (@ 0x0000006C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000006C) GPIO INT7 selection                                        */
  
  struct {
    __IOM uint32_t int_mer_sel : 5;           /*!< [4..0] Pin selection of specified port for GPIO INT7                      */
    __IOM uint32_t int_gp_sel : 2;            /*!< [6..5] GPIO port selection for GPIO INT7                                  */
    __IM  uint32_t            : 1;
    __IOM uint32_t int_deb_sel : 4;           /*!< [11..8] Debounce Output signal selection for GPIO INT7                    */
    __IM  uint32_t            : 3;
    __IOM uint32_t int_sur_sel : 1;           /*!< [15..15] 0: Useing pin input signal as interrupt signal source
                                                   1: Using debounce output signal as interrupt signal source                */
  } b;                                        /*!< bit fields for gpio_int7_sel */
} gpio_int7_sel_t, *pgpio_int7_sel_t;

/**
  \brief Union type to access gpio_int8_sel (@ 0x00000070).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000070) GPIO INT8 selection                                        */
  
  struct {
    __IOM uint32_t int_mer_sel : 5;           /*!< [4..0] Pin selection of specified port for GPIO INT8                      */
    __IOM uint32_t int_gp_sel : 2;            /*!< [6..5] GPIO port selection for GPIO INT8                                  */
    __IM  uint32_t            : 1;
    __IOM uint32_t int_deb_sel : 4;           /*!< [11..8] Debounce Output signal selection for GPIO INT8                    */
    __IM  uint32_t            : 3;
    __IOM uint32_t int_sur_sel : 1;           /*!< [15..15] 0: Useing pin input signal as interrupt signal source
                                                   1: Using debounce output signal as interrupt signal source                */
  } b;                                        /*!< bit fields for gpio_int8_sel */
} gpio_int8_sel_t, *pgpio_int8_sel_t;

/**
  \brief Union type to access gpio_int9_sel (@ 0x00000074).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000074) GPIO INT9 selection                                        */
  
  struct {
    __IOM uint32_t int_mer_sel : 5;           /*!< [4..0] Pin selection of specified port for GPIO INT9                      */
    __IOM uint32_t int_gp_sel : 2;            /*!< [6..5] GPIO port selection for GPIO INT9                                  */
    __IM  uint32_t            : 1;
    __IOM uint32_t int_deb_sel : 4;           /*!< [11..8] Debounce Output signal selection for GPIO INT9                    */
    __IM  uint32_t            : 3;
    __IOM uint32_t int_sur_sel : 1;           /*!< [15..15] 0: Useing pin input signal as interrupt signal source
                                                   1: Using debounce output signal as interrupt signal source                */
  } b;                                        /*!< bit fields for gpio_int9_sel */
} gpio_int9_sel_t, *pgpio_int9_sel_t;

/**
  \brief Union type to access gpio_int10_sel (@ 0x00000078).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000078) GPIO INT10 selection                                       */
  
  struct {
    __IOM uint32_t int_mer_sel : 5;           /*!< [4..0] Pin selection of specified port for GPIO INT10                     */
    __IOM uint32_t int_gp_sel : 2;            /*!< [6..5] GPIO port selection for GPIO INT10                                 */
    __IM  uint32_t            : 1;
    __IOM uint32_t int_deb_sel : 4;           /*!< [11..8] Debounce Output signal selection for GPIO INT10                   */
    __IM  uint32_t            : 3;
    __IOM uint32_t int_sur_sel : 1;           /*!< [15..15] 0: Useing pin input signal as interrupt signal source
                                                   1: Using debounce output signal as interrupt signal source                */
  } b;                                        /*!< bit fields for gpio_int10_sel */
} gpio_int10_sel_t, *pgpio_int10_sel_t;

/**
  \brief Union type to access gpio_int11_sel (@ 0x0000007C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000007C) GPIO INT11 selection                                       */
  
  struct {
    __IOM uint32_t int_mer_sel : 5;           /*!< [4..0] Pin selection of specified port for GPIO INT11                     */
    __IOM uint32_t int_gp_sel : 2;            /*!< [6..5] GPIO port selection for GPIO INT11                                 */
    __IM  uint32_t            : 1;
    __IOM uint32_t int_deb_sel : 4;           /*!< [11..8] Debounce Output signal selection for GPIO INT11                   */
    __IM  uint32_t            : 3;
    __IOM uint32_t int_sur_sel : 1;           /*!< [15..15] 0: Useing pin input signal as interrupt signal source
                                                   1: Using debounce output signal as interrupt signal source                */
  } b;                                        /*!< bit fields for gpio_int11_sel */
} gpio_int11_sel_t, *pgpio_int11_sel_t;

/**
  \brief Union type to access gpio_int12_sel (@ 0x00000080).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000080) GPIO INT12 selection                                       */
  
  struct {
    __IOM uint32_t int_mer_sel : 5;           /*!< [4..0] Pin selection of specified port for GPIO INT12                     */
    __IOM uint32_t int_gp_sel : 2;            /*!< [6..5] GPIO port selection for GPIO INT12                                 */
    __IM  uint32_t            : 1;
    __IOM uint32_t int_deb_sel : 4;           /*!< [11..8] Debounce Output signal selection for GPIO INT12                   */
    __IM  uint32_t            : 3;
    __IOM uint32_t int_sur_sel : 1;           /*!< [15..15] 0: Useing pin input signal as interrupt signal source
                                                   1: Using debounce output signal as interrupt signal source                */
  } b;                                        /*!< bit fields for gpio_int12_sel */
} gpio_int12_sel_t, *pgpio_int12_sel_t;

/**
  \brief Union type to access gpio_int13_sel (@ 0x00000084).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000084) GPIO INT13 selection                                       */
  
  struct {
    __IOM uint32_t int_mer_sel : 5;           /*!< [4..0] Pin selection of specified port for GPIO INT13                     */
    __IOM uint32_t int_gp_sel : 2;            /*!< [6..5] GPIO port selection for GPIO INT13                                 */
    __IM  uint32_t            : 1;
    __IOM uint32_t int_deb_sel : 4;           /*!< [11..8] Debounce Output signal selection for GPIO INT13                   */
    __IM  uint32_t            : 3;
    __IOM uint32_t int_sur_sel : 1;           /*!< [15..15] 0: Useing pin input signal as interrupt signal source
                                                   1: Using debounce output signal as interrupt signal source                */
  } b;                                        /*!< bit fields for gpio_int13_sel */
} gpio_int13_sel_t, *pgpio_int13_sel_t;

/**
  \brief Union type to access gpio_int14_sel (@ 0x00000088).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000088) GPIO INT14 selection                                       */
  
  struct {
    __IOM uint32_t int_mer_sel : 5;           /*!< [4..0] Pin selection of specified port for GPIO INT14                     */
    __IOM uint32_t int_gp_sel : 2;            /*!< [6..5] GPIO port selection for GPIO INT14                                 */
    __IM  uint32_t            : 1;
    __IOM uint32_t int_deb_sel : 4;           /*!< [11..8] Debounce Output signal selection for GPIO INT14                   */
    __IM  uint32_t            : 3;
    __IOM uint32_t int_sur_sel : 1;           /*!< [15..15] 0: Useing pin input signal as interrupt signal source
                                                   1: Using debounce output signal as interrupt signal source                */
  } b;                                        /*!< bit fields for gpio_int14_sel */
} gpio_int14_sel_t, *pgpio_int14_sel_t;

/**
  \brief Union type to access gpio_int15_sel (@ 0x0000008C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000008C) GPIO INT15 selection                                       */
  
  struct {
    __IOM uint32_t int_mer_sel : 5;           /*!< [4..0] Pin selection of specified port for GPIO INT15                     */
    __IOM uint32_t int_gp_sel : 2;            /*!< [6..5] GPIO port selection for GPIO INT15                                 */
    __IM  uint32_t            : 1;
    __IOM uint32_t int_deb_sel : 4;           /*!< [11..8] Debounce Output signal selection for GPIO INT15                   */
    __IM  uint32_t            : 3;
    __IOM uint32_t int_sur_sel : 1;           /*!< [15..15] 0: Useing pin input signal as interrupt signal source
                                                   1: Using debounce output signal as interrupt signal source                */
  } b;                                        /*!< bit fields for gpio_int15_sel */
} gpio_int15_sel_t, *pgpio_int15_sel_t;

/**
  \brief Union type to access gpio_deb_sts (@ 0x000000F0).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000F0) GPIO port debounce status                                  */
  
  struct {
    __IM  uint32_t deb_sts    : 16;           /*!< [15..0] For each bit: 0: the specific GPIO debounce pin is DISABLE
                                                   debounce 1: the specific GPIO debounce pin is ENABLE debounce             */
  } b;                                        /*!< bit fields for gpio_deb_sts */
} gpio_deb_sts_t, *pgpio_deb_sts_t;

/**
  \brief Union type to access gpio_deb_en (@ 0x000000F4).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000F4) GPIO debounce pin enable                                   */
  
  struct {
    __IOM uint32_t deb_en     : 16;           /*!< [15..0] For each bit write: 0: No operation 1: the conrtolled
                                                   GPIO de-bouncing pin is enabled and also cause the specified
                                                   bit of REG_GPIO_DEB_STS to be read as ONE.                                */
  } b;                                        /*!< bit fields for gpio_deb_en */
} gpio_deb_en_t, *pgpio_deb_en_t;

/**
  \brief Union type to access gpio_deb_dis (@ 0x000000F8).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000F8) GPIO debounce pin disable                                  */
  
  struct {
    __IOM uint32_t deb_dis    : 16;           /*!< [15..0] For each bit write: 0: No operation 1: the conrtolled
                                                   GPIO de-bouncing pin is disabled and also cause the specified
                                                   bit of REG_GPIO_DEB_STS to be read as ZERO.                               */
  } b;                                        /*!< bit fields for gpio_deb_dis */
} gpio_deb_dis_t, *pgpio_deb_dis_t;

/**
  \brief Union type to access gpio_deb_dp_sts (@ 0x000000FC).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000FC) GPIO debounce data pin status                              */
  
  struct {
    __IM  uint32_t deb_dp_sts : 16;           /*!< [15..0] For each bit reading: read the signal level of the specified
                                                   pin after de-bounce                                                       */
  } b;                                        /*!< bit fields for gpio_deb_dp_sts */
} gpio_deb_dp_sts_t, *pgpio_deb_dp_sts_t;

/**
  \brief Union type to access gpio_deb0_sel (@ 0x00000100).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000100) GPIO debounce0 selection                                   */
  
  struct {
    __IOM uint32_t deb_mer_sel : 5;           /*!< [4..0] Pin selection of specified port for debounce0                      */
    __IOM uint32_t deb_gp_sel : 2;            /*!< [6..5] GPIO port selection for debounce0                                  */
    __IM  uint32_t            : 1;
    __IOM uint32_t deb_cyc    : 14;           /*!< [21..8] The GPIO signal will be filtered by the number of debounce
                                                   cycles given in this field.                                               */
  } b;                                        /*!< bit fields for gpio_deb0_sel */
} gpio_deb0_sel_t, *pgpio_deb0_sel_t;

/**
  \brief Union type to access gpio_deb1_sel (@ 0x00000104).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000104) GPIO debounce1 selection                                   */
  
  struct {
    __IOM uint32_t deb_mer_sel : 5;           /*!< [4..0] Pin selection of specified port for debounce1                      */
    __IOM uint32_t deb_gp_sel : 2;            /*!< [6..5] GPIO port selection for debounce1                                  */
    __IM  uint32_t            : 1;
    __IOM uint32_t deb_cyc    : 14;           /*!< [21..8] The GPIO signal will be filtered by the number of debounce
                                                   cycles given in this field.                                               */
  } b;                                        /*!< bit fields for gpio_deb1_sel */
} gpio_deb1_sel_t, *pgpio_deb1_sel_t;

/**
  \brief Union type to access gpio_deb2_sel (@ 0x00000108).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000108) GPIO debounce2 selection                                   */
  
  struct {
    __IOM uint32_t deb_mer_sel : 5;           /*!< [4..0] Pin selection of specified port for debounce2                      */
    __IOM uint32_t deb_gp_sel : 2;            /*!< [6..5] GPIO port selection for debounce2                                  */
    __IM  uint32_t            : 1;
    __IOM uint32_t deb_cyc    : 14;           /*!< [21..8] The GPIO signal will be filtered by the number of debounce
                                                   cycles given in this field.                                               */
  } b;                                        /*!< bit fields for gpio_deb2_sel */
} gpio_deb2_sel_t, *pgpio_deb2_sel_t;

/**
  \brief Union type to access gpio_deb3_sel (@ 0x0000010C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000010C) GPIO debounce3 selection                                   */
  
  struct {
    __IOM uint32_t deb_mer_sel : 5;           /*!< [4..0] Pin selection of specified port for debounce3                      */
    __IOM uint32_t deb_gp_sel : 2;            /*!< [6..5] GPIO port selection for debounce3                                  */
    __IM  uint32_t            : 1;
    __IOM uint32_t deb_cyc    : 14;           /*!< [21..8] The GPIO signal will be filtered by the number of debounce
                                                   cycles given in this field.                                               */
  } b;                                        /*!< bit fields for gpio_deb3_sel */
} gpio_deb3_sel_t, *pgpio_deb3_sel_t;

/**
  \brief Union type to access gpio_deb4_sel (@ 0x00000110).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000110) GPIO debounce4 selection                                   */
  
  struct {
    __IOM uint32_t deb_mer_sel : 5;           /*!< [4..0] Pin selection of specified port for debounce4                      */
    __IOM uint32_t deb_gp_sel : 2;            /*!< [6..5] GPIO port selection for debounce4                                  */
    __IM  uint32_t            : 1;
    __IOM uint32_t deb_cyc    : 14;           /*!< [21..8] The GPIO signal will be filtered by the number of debounce
                                                   cycles given in this field.                                               */
  } b;                                        /*!< bit fields for gpio_deb4_sel */
} gpio_deb4_sel_t, *pgpio_deb4_sel_t;

/**
  \brief Union type to access gpio_deb5_sel (@ 0x00000114).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000114) GPIO debounce5 selection                                   */
  
  struct {
    __IOM uint32_t deb_mer_sel : 5;           /*!< [4..0] Pin selection of specified port for debounce5                      */
    __IOM uint32_t deb_gp_sel : 2;            /*!< [6..5] GPIO port selection for debounce5                                  */
    __IM  uint32_t            : 1;
    __IOM uint32_t deb_cyc    : 14;           /*!< [21..8] The GPIO signal will be filtered by the number of debounce
                                                   cycles given in this field.                                               */
  } b;                                        /*!< bit fields for gpio_deb5_sel */
} gpio_deb5_sel_t, *pgpio_deb5_sel_t;

/**
  \brief Union type to access gpio_deb6_sel (@ 0x00000118).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000118) GPIO debounce6 selection                                   */
  
  struct {
    __IOM uint32_t deb_mer_sel : 5;           /*!< [4..0] Pin selection of specified port for debounce6                      */
    __IOM uint32_t deb_gp_sel : 2;            /*!< [6..5] GPIO port selection for debounce6                                  */
    __IM  uint32_t            : 1;
    __IOM uint32_t deb_cyc    : 14;           /*!< [21..8] The GPIO signal will be filtered by the number of debounce
                                                   cycles given in this field.                                               */
  } b;                                        /*!< bit fields for gpio_deb6_sel */
} gpio_deb6_sel_t, *pgpio_deb6_sel_t;

/**
  \brief Union type to access gpio_deb7_sel (@ 0x0000011C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000011C) GPIO debounce7 selection                                   */
  
  struct {
    __IOM uint32_t deb_mer_sel : 5;           /*!< [4..0] Pin selection of specified port for debounce7                      */
    __IOM uint32_t deb_gp_sel : 2;            /*!< [6..5] GPIO port selection for debounce7                                  */
    __IM  uint32_t            : 1;
    __IOM uint32_t deb_cyc    : 14;           /*!< [21..8] The GPIO signal will be filtered by the number of debounce
                                                   cycles given in this field.                                               */
  } b;                                        /*!< bit fields for gpio_deb7_sel */
} gpio_deb7_sel_t, *pgpio_deb7_sel_t;

/**
  \brief Union type to access gpio_deb8_sel (@ 0x00000120).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000120) GPIO debounce8 selection                                   */
  
  struct {
    __IOM uint32_t deb_mer_sel : 5;           /*!< [4..0] Pin selection of specified port for debounce8                      */
    __IOM uint32_t deb_gp_sel : 2;            /*!< [6..5] GPIO port selection for debounce8                                  */
    __IM  uint32_t            : 1;
    __IOM uint32_t deb_cyc    : 14;           /*!< [21..8] The GPIO signal will be filtered by the number of debounce
                                                   cycles given in this field.                                               */
  } b;                                        /*!< bit fields for gpio_deb8_sel */
} gpio_deb8_sel_t, *pgpio_deb8_sel_t;

/**
  \brief Union type to access gpio_deb9_sel (@ 0x00000124).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000124) GPIO debounce9 selection                                   */
  
  struct {
    __IOM uint32_t deb_mer_sel : 5;           /*!< [4..0] Pin selection of specified port for debounce9                      */
    __IOM uint32_t deb_gp_sel : 2;            /*!< [6..5] GPIO port selection for debounce9                                  */
    __IM  uint32_t            : 1;
    __IOM uint32_t deb_cyc    : 14;           /*!< [21..8] The GPIO signal will be filtered by the number of debounce
                                                   cycles given in this field.                                               */
  } b;                                        /*!< bit fields for gpio_deb9_sel */
} gpio_deb9_sel_t, *pgpio_deb9_sel_t;

/**
  \brief Union type to access gpio_deb10_sel (@ 0x00000128).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000128) GPIO debounce10 selection                                  */
  
  struct {
    __IOM uint32_t deb_mer_sel : 5;           /*!< [4..0] Pin selection of specified port for debounce10                     */
    __IOM uint32_t deb_gp_sel : 2;            /*!< [6..5] GPIO port selection for debounce10                                 */
    __IM  uint32_t            : 1;
    __IOM uint32_t deb_cyc    : 14;           /*!< [21..8] The GPIO signal will be filtered by the number of debounce
                                                   cycles given in this field.                                               */
  } b;                                        /*!< bit fields for gpio_deb10_sel */
} gpio_deb10_sel_t, *pgpio_deb10_sel_t;

/**
  \brief Union type to access gpio_deb11_sel (@ 0x0000012C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000012C) GPIO debounce11 selection                                  */
  
  struct {
    __IOM uint32_t deb_mer_sel : 5;           /*!< [4..0] Pin selection of specified port for debounce11                     */
    __IOM uint32_t deb_gp_sel : 2;            /*!< [6..5] GPIO port selection for debounce11                                 */
    __IM  uint32_t            : 1;
    __IOM uint32_t deb_cyc    : 14;           /*!< [21..8] The GPIO signal will be filtered by the number of debounce
                                                   cycles given in this field.                                               */
  } b;                                        /*!< bit fields for gpio_deb11_sel */
} gpio_deb11_sel_t, *pgpio_deb11_sel_t;

/**
  \brief Union type to access gpio_deb12_sel (@ 0x00000130).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000130) GPIO debounce12 selection                                  */
  
  struct {
    __IOM uint32_t deb_mer_sel : 5;           /*!< [4..0] Pin selection of specified port for debounce12                     */
    __IOM uint32_t deb_gp_sel : 2;            /*!< [6..5] GPIO port selection for debounce12                                 */
    __IM  uint32_t            : 1;
    __IOM uint32_t deb_cyc    : 14;           /*!< [21..8] The GPIO signal will be filtered by the number of debounce
                                                   cycles given in this field.                                               */
  } b;                                        /*!< bit fields for gpio_deb12_sel */
} gpio_deb12_sel_t, *pgpio_deb12_sel_t;

/**
  \brief Union type to access gpio_deb13_sel (@ 0x00000134).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000134) GPIO debounce13 selection                                  */
  
  struct {
    __IOM uint32_t deb_mer_sel : 5;           /*!< [4..0] Pin selection of specified port for debounce13                     */
    __IOM uint32_t deb_gp_sel : 2;            /*!< [6..5] GPIO port selection for debounce13                                 */
    __IM  uint32_t            : 1;
    __IOM uint32_t deb_cyc    : 14;           /*!< [21..8] The GPIO signal will be filtered by the number of debounce
                                                   cycles given in this field.                                               */
  } b;                                        /*!< bit fields for gpio_deb13_sel */
} gpio_deb13_sel_t, *pgpio_deb13_sel_t;

/**
  \brief Union type to access gpio_deb14_sel (@ 0x00000138).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000138) GPIO debounce14 selection                                  */
  
  struct {
    __IOM uint32_t deb_mer_sel : 5;           /*!< [4..0] Pin selection of specified port for debounce14                     */
    __IOM uint32_t deb_gp_sel : 2;            /*!< [6..5] GPIO port selection for debounce14                                 */
    __IM  uint32_t            : 1;
    __IOM uint32_t deb_cyc    : 14;           /*!< [21..8] The GPIO signal will be filtered by the number of debounce
                                                   cycles given in this field.                                               */
  } b;                                        /*!< bit fields for gpio_deb14_sel */
} gpio_deb14_sel_t, *pgpio_deb14_sel_t;

/**
  \brief Union type to access gpio_deb15_sel (@ 0x0000013C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000013C) GPIO debounce15 selection                                  */
  
  struct {
    __IOM uint32_t deb_mer_sel : 5;           /*!< [4..0] Pin selection of specified port for debounce15                     */
    __IOM uint32_t deb_gp_sel : 2;            /*!< [6..5] GPIO port selection for debounce15                                 */
    __IM  uint32_t            : 1;
    __IOM uint32_t deb_cyc    : 14;           /*!< [21..8] The GPIO signal will be filtered by the number of debounce
                                                   cycles given in this field.                                               */
  } b;                                        /*!< bit fields for gpio_deb15_sel */
} gpio_deb15_sel_t, *pgpio_deb15_sel_t;

/**
  \brief Union type to access gpio_port_a_dmd_sts (@ 0x00000200).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000200) GPIO port A data mode direction status                     */
  
  struct {
    __IM  uint32_t dmd_sts    : 32;           /*!< [31..0] For each bit: 0: the specific GPIO pin is configured
                                                   to INPUT data mode 1: the specific GPIO pin is configured
                                                   to OUTPUT data mode                                                       */
  } b;                                        /*!< bit fields for gpio_port_a_dmd_sts */
} gpio_port_a_dmd_sts_t, *pgpio_port_a_dmd_sts_t;

/**
  \brief Union type to access gpio_port_a_idm_en (@ 0x00000204).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000204) GPIO port A input data mode enable                         */
  
  struct {
    __IOM uint32_t idm_en     : 32;           /*!< [31..0] For each bit write: 0: the configuration of the conrtolled
                                                   GPIO pin is unchanged; 1: the conrtolled GPIO pin is configured
                                                   to input data mode and also cause the specified bit of
                                                   REG_PORT_A_DMD_STS to be read as zero.                                    */
  } b;                                        /*!< bit fields for gpio_port_a_idm_en */
} gpio_port_a_idm_en_t, *pgpio_port_a_idm_en_t;

/**
  \brief Union type to access gpio_port_a_odm_en (@ 0x00000208).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000208) GPIO port A output data mode enable                        */
  
  struct {
    __IOM uint32_t odm_en     : 32;           /*!< [31..0] For each bit write: 0: the configuration of conrtolled
                                                   GPIO pin is unchanged; 1: the conrtolled GPIO pin is configured
                                                   as output data mode and also cause the specified bit of
                                                   REG_PORT_A_DMD_STS to be read as one                                      */
  } b;                                        /*!< bit fields for gpio_port_a_odm_en */
} gpio_port_a_odm_en_t, *pgpio_port_a_odm_en_t;

/**
  \brief Union type to access gpio_port_a_od_sts (@ 0x0000020C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000020C) GPIO port A output data status                             */
  
  struct {
    __IM  uint32_t od_sts     : 32;           /*!< [31..0] For each bit: 0: the specified GPIO pin is configured
                                                   as output low 1: the specified GPIO pin is configured as
                                                   output high , if the specific GPIO pin is output data mode.               */
  } b;                                        /*!< bit fields for gpio_port_a_od_sts */
} gpio_port_a_od_sts_t, *pgpio_port_a_od_sts_t;

/**
  \brief Union type to access gpio_port_a_odl_en (@ 0x00000210).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000210) GPIO port A output data low enable                         */
  
  struct {
    __IOM uint32_t odl_en     : 32;           /*!< [31..0] For each bit write: 0: No operation 1: the conrtolled
                                                   GPIO pin is configured as output low and also cause the
                                                   specified bit of REG_PORT_A_OD_STS is zero.                               */
  } b;                                        /*!< bit fields for gpio_port_a_odl_en */
} gpio_port_a_odl_en_t, *pgpio_port_a_odl_en_t;

/**
  \brief Union type to access gpio_port_a_odh_en (@ 0x00000214).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000214) GPIO port A output data high enable                        */
  
  struct {
    __IOM uint32_t odh_en     : 32;           /*!< [31..0] For each bit write: 0: No operation 1: the conrtolled
                                                   GPIO pin is configured as output high and also cause the
                                                   specified bit of REG_PORT_A_OD_STS is one.                                */
  } b;                                        /*!< bit fields for gpio_port_a_odh_en */
} gpio_port_a_odh_en_t, *pgpio_port_a_odh_en_t;

/**
  \brief Union type to access gpio_port_a_odt_en (@ 0x00000218).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000218) GPIO port A output data toggle enable                      */
  
  struct {
    __IOM uint32_t odt_en     : 32;           /*!< [31..0] For each bit write: 0: No operation 1: Toggle output
                                                   of the conrtolled GPIO pin and the specified bit of REG_PORT_A_OD_STS
                                                   also will be toggled.                                                     */
  } b;                                        /*!< bit fields for gpio_port_a_odt_en */
} gpio_port_a_odt_en_t, *pgpio_port_a_odt_en_t;

/**
  \brief Union type to access gpio_port_a_dp_sts (@ 0x0000021C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000021C) GPIO port A data pin status                                */
  
  struct {
    __IM  uint32_t dp_sts     : 32;           /*!< [31..0] For each bit reading: If the direction of the specified
                                                   bit is input then read the signal level of the input pin.
                                                   If the direction of the specified bit is output then reads
                                                   the output data register for port A.                                      */
  } b;                                        /*!< bit fields for gpio_port_a_dp_sts */
} gpio_port_a_dp_sts_t, *pgpio_port_a_dp_sts_t;

/**
  \brief Union type to access gpio_port_b_dmd_sts (@ 0x00000240).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000240) GPIO port B data mode direction status                     */
  
  struct {
    __IM  uint32_t dmd_sts    : 32;           /*!< [31..0] For each bit: 0: the specific GPIO pin is configured
                                                   to INPUT data mode 1: the specific GPIO pin is configured
                                                   to OUTPUT data mode                                                       */
  } b;                                        /*!< bit fields for gpio_port_b_dmd_sts */
} gpio_port_b_dmd_sts_t, *pgpio_port_b_dmd_sts_t;

/**
  \brief Union type to access gpio_port_b_idm_en (@ 0x00000244).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000244) GPIO port B input data mode enable                         */
  
  struct {
    __IOM uint32_t idm_en     : 32;           /*!< [31..0] For each bit write: 0: the configuration of the conrtolled
                                                   GPIO pin is unchanged; 1: the conrtolled GPIO pin is configured
                                                   to input data mode and also cause the specified bit of
                                                   REG_PORT_B_DMD_STS to be read as zero.                                    */
  } b;                                        /*!< bit fields for gpio_port_b_idm_en */
} gpio_port_b_idm_en_t, *pgpio_port_b_idm_en_t;

/**
  \brief Union type to access gpio_port_b_odm_en (@ 0x00000248).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000248) GPIO port B output data mode enable                        */
  
  struct {
    __IOM uint32_t odm_en     : 32;           /*!< [31..0] For each bit write: 0: the configuration of the conrtolled
                                                   GPIO pin is unchanged; 1: the conrtolled GPIO pin is configured
                                                   as output data mode and also cause the specified bit of
                                                   REG_PORT_B_DMD_STS to be read as one                                      */
  } b;                                        /*!< bit fields for gpio_port_b_odm_en */
} gpio_port_b_odm_en_t, *pgpio_port_b_odm_en_t;

/**
  \brief Union type to access gpio_port_b_od_sts (@ 0x0000024C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000024C) GPIO port B output data status                             */
  
  struct {
    __IM  uint32_t od_sts     : 32;           /*!< [31..0] For each bit: 0: the specified GPIO pin is configured
                                                   as output low 1: the specified GPIO pin is configured as
                                                   output high , if the specific GPIO pin is output data mode.               */
  } b;                                        /*!< bit fields for gpio_port_b_od_sts */
} gpio_port_b_od_sts_t, *pgpio_port_b_od_sts_t;

/**
  \brief Union type to access gpio_port_b_odl_en (@ 0x00000250).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000250) GPIO port B output data low enable                         */
  
  struct {
    __IOM uint32_t odl_en     : 32;           /*!< [31..0] For each bit write: 0: No operation 1: the conrtolled
                                                   GPIO pin is configured as output low and also cause the
                                                   specified bit of REG_PORT_B_OD_STS is zero.                               */
  } b;                                        /*!< bit fields for gpio_port_b_odl_en */
} gpio_port_b_odl_en_t, *pgpio_port_b_odl_en_t;

/**
  \brief Union type to access gpio_port_b_odh_en (@ 0x00000254).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000254) GPIO port B output data high enable                        */
  
  struct {
    __IOM uint32_t odh_en     : 32;           /*!< [31..0] For each bit write: 0: No operation 1: the conrtolled
                                                   GPIO pin is configured as output high and also cause the
                                                   specified bit of REG_PORT_B_OD_STS is one.                                */
  } b;                                        /*!< bit fields for gpio_port_b_odh_en */
} gpio_port_b_odh_en_t, *pgpio_port_b_odh_en_t;

/**
  \brief Union type to access gpio_port_b_odt_en (@ 0x00000258).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000258) GPIO port B output data toggle enable                      */
  
  struct {
    __IOM uint32_t odt_en     : 32;           /*!< [31..0] For each bit write: 0: No operation 1: Toggle output
                                                   of the conrtolled GPIO pin and the specified bit of REG_PORT_B_OD_STS
                                                   also will be toggled.                                                     */
  } b;                                        /*!< bit fields for gpio_port_b_odt_en */
} gpio_port_b_odt_en_t, *pgpio_port_b_odt_en_t;

/**
  \brief Union type to access gpio_port_b_dp_sts (@ 0x0000025C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000025C) GPIO port B data pin status                                */
  
  struct {
    __IM  uint32_t dp_sts     : 32;           /*!< [31..0] For each bit reading: If the direction of the specified
                                                   bit is input then read the signal level of the input pin.
                                                   If the direction of the specified bit is output then reads
                                                   the output data register for port B.                                      */
  } b;                                        /*!< bit fields for gpio_port_b_dp_sts */
} gpio_port_b_dp_sts_t, *pgpio_port_b_dp_sts_t;

/**
  \brief Union type to access gpio_port_c_dmd_sts (@ 0x00000280).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000280) GPIO port C data mode direction status                     */
  
  struct {
    __IM  uint32_t dmd_sts    : 32;           /*!< [31..0] For each bit: 0: the specific GPIO pin is configured
                                                   to INPUT data mode 1: the specific GPIO pin is configured
                                                   to OUTPUT data mode                                                       */
  } b;                                        /*!< bit fields for gpio_port_c_dmd_sts */
} gpio_port_c_dmd_sts_t, *pgpio_port_c_dmd_sts_t;

/**
  \brief Union type to access gpio_port_c_idm_en (@ 0x00000284).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000284) GPIO port C input data mode enable                         */
  
  struct {
    __IOM uint32_t idm_en     : 32;           /*!< [31..0] For each bit write: 0: the configuration of the conrtolled
                                                   GPIO pin is unchanged; 1: the conrtolled GPIO pin is configured
                                                   to input data mode and also cause the specified bit of
                                                   REG_PORT_C_DMD_STS to be read as zero.                                    */
  } b;                                        /*!< bit fields for gpio_port_c_idm_en */
} gpio_port_c_idm_en_t, *pgpio_port_c_idm_en_t;

/**
  \brief Union type to access gpio_port_c_odm_en (@ 0x00000288).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000288) GPIO port C output data mode enable                        */
  
  struct {
    __IOM uint32_t odm_en     : 32;           /*!< [31..0] For each bit write: 0: the configuration of the conrtolled
                                                   GPIO pin is unchanged; 1: the conrtolled GPIO pin is configured
                                                   as output data mode and also cause the specified bit of
                                                   REG_PORT_C_DMD_STS to be read as one                                      */
  } b;                                        /*!< bit fields for gpio_port_c_odm_en */
} gpio_port_c_odm_en_t, *pgpio_port_c_odm_en_t;

/**
  \brief Union type to access gpio_port_c_od_sts (@ 0x0000028C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000028C) GPIO port C output data status                             */
  
  struct {
    __IM  uint32_t od_sts     : 32;           /*!< [31..0] For each bit: 0: the specified GPIO pin is configured
                                                   as output low 1: the specified GPIO pin is configured as
                                                   output high , if the specific GPIO pin is output data mode.               */
  } b;                                        /*!< bit fields for gpio_port_c_od_sts */
} gpio_port_c_od_sts_t, *pgpio_port_c_od_sts_t;

/**
  \brief Union type to access gpio_port_c_odl_en (@ 0x00000290).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000290) GPIO port C output data low enable                         */
  
  struct {
    __IOM uint32_t odl_en     : 32;           /*!< [31..0] For each bit write: 0: No operation 1: the conrtolled
                                                   GPIO pin is configured as output low and also cause the
                                                   specified bit of REG_PORT_C_OD_STS is zero.                               */
  } b;                                        /*!< bit fields for gpio_port_c_odl_en */
} gpio_port_c_odl_en_t, *pgpio_port_c_odl_en_t;

/**
  \brief Union type to access gpio_port_c_odh_en (@ 0x00000294).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000294) GPIO port C output data high enable                        */
  
  struct {
    __IOM uint32_t odh_en     : 32;           /*!< [31..0] For each bit write: 0: No operation 1: the conrtolled
                                                   GPIO pin is configured as output high and also cause the
                                                   specified bit of REG_PORT_C_OD_STS is one.                                */
  } b;                                        /*!< bit fields for gpio_port_c_odh_en */
} gpio_port_c_odh_en_t, *pgpio_port_c_odh_en_t;

/**
  \brief Union type to access gpio_port_c_odt_en (@ 0x00000298).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000298) GPIO port C output data toggle enable                      */
  
  struct {
    __IOM uint32_t odt_en     : 32;           /*!< [31..0] For each bit write: 0: No operation 1: Toggle output
                                                   of the conrtolled GPIO pin and the specified bit of REG_PORT_C_OD_STS
                                                   also will be toggled.                                                     */
  } b;                                        /*!< bit fields for gpio_port_c_odt_en */
} gpio_port_c_odt_en_t, *pgpio_port_c_odt_en_t;

/**
  \brief Union type to access gpio_port_c_dp_sts (@ 0x0000029C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000029C) GPIO port C data pin status                                */
  
  struct {
    __IM  uint32_t dp_sts     : 32;           /*!< [31..0] For each bit reading: If the direction of the specified
                                                   bit is input then read the signal level of the input pin.
                                                   If the direction of the specified bit is output then reads
                                                   the output data register for port C.                                      */
  } b;                                        /*!< bit fields for gpio_port_c_dp_sts */
} gpio_port_c_dp_sts_t, *pgpio_port_c_dp_sts_t;

/**
  \brief Union type to access gpio_port_d_dmd_sts (@ 0x000002C0).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000002C0) GPIO port D data mode direction status                     */
  
  struct {
    __IM  uint32_t dmd_sts    : 32;           /*!< [31..0] For each bit: 0: the specific GPIO pin is configured
                                                   to INPUT data mode 1: the specific GPIO pin is configured
                                                   to OUTPUT data mode                                                       */
  } b;                                        /*!< bit fields for gpio_port_d_dmd_sts */
} gpio_port_d_dmd_sts_t, *pgpio_port_d_dmd_sts_t;

/**
  \brief Union type to access gpio_port_d_idm_en (@ 0x000002C4).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000002C4) GPIO port D input data mode enable                         */
  
  struct {
    __IOM uint32_t idm_en     : 32;           /*!< [31..0] For each bit write: 0: the configuration of the conrtolled
                                                   GPIO pin is unchanged; 1: the conrtolled GPIO pin is configured
                                                   to input data mode and also cause the specified bit of
                                                   REG_PORT_D_DMD_STS to be read as zero.                                    */
  } b;                                        /*!< bit fields for gpio_port_d_idm_en */
} gpio_port_d_idm_en_t, *pgpio_port_d_idm_en_t;

/**
  \brief Union type to access gpio_port_d_odm_en (@ 0x000002C8).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000002C8) GPIO port D output data mode enable                        */
  
  struct {
    __IOM uint32_t odm_en     : 32;           /*!< [31..0] For each bit write: 0: the configuration of the conrtolled
                                                   GPIO pin is unchanged; 1: the conrtolled GPIO pin is configured
                                                   as output data mode and also cause the specified bit of
                                                   REG_PORT_D_DMD_STS to be read as one                                      */
  } b;                                        /*!< bit fields for gpio_port_d_odm_en */
} gpio_port_d_odm_en_t, *pgpio_port_d_odm_en_t;

/**
  \brief Union type to access gpio_port_d_od_sts (@ 0x000002CC).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000002CC) GPIO port D output data status                             */
  
  struct {
    __IM  uint32_t od_sts     : 32;           /*!< [31..0] For each bit: 0: the specified GPIO pin is configured
                                                   as output low 1: the specified GPIO pin is configured as
                                                   output high , if the specific GPIO pin is output data mode.               */
  } b;                                        /*!< bit fields for gpio_port_d_od_sts */
} gpio_port_d_od_sts_t, *pgpio_port_d_od_sts_t;

/**
  \brief Union type to access gpio_port_d_odl_en (@ 0x000002D0).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000002D0) GPIO port D output data low enable                         */
  
  struct {
    __IOM uint32_t odl_en     : 32;           /*!< [31..0] For each bit write: 0: No operation 1: the conrtolled
                                                   GPIO pin is configured as output low and also cause the
                                                   specified bit of REG_PORT_D_OD_STS is zero.                               */
  } b;                                        /*!< bit fields for gpio_port_d_odl_en */
} gpio_port_d_odl_en_t, *pgpio_port_d_odl_en_t;

/**
  \brief Union type to access gpio_port_d_odh_en (@ 0x000002D4).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000002D4) GPIO port D output data high enable                        */
  
  struct {
    __IOM uint32_t odh_en     : 32;           /*!< [31..0] For each bit write: 0: No operation 1: the conrtolled
                                                   GPIO pin is configured as output high and also cause the
                                                   specified bit of REG_PORT_D_OD_STS is one.                                */
  } b;                                        /*!< bit fields for gpio_port_d_odh_en */
} gpio_port_d_odh_en_t, *pgpio_port_d_odh_en_t;

/**
  \brief Union type to access gpio_port_d_odt_en (@ 0x000002D8).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000002D8) GPIO port D output data toggle enable                      */
  
  struct {
    __IOM uint32_t odt_en     : 32;           /*!< [31..0] For each bit write: 0: No operation 1: Toggle output
                                                   of the conrtolled GPIO pin and the specified bit of REG_PORT_D_OD_STS
                                                   also will be toggled.                                                     */
  } b;                                        /*!< bit fields for gpio_port_d_odt_en */
} gpio_port_d_odt_en_t, *pgpio_port_d_odt_en_t;

/**
  \brief Union type to access gpio_port_d_dp_sts (@ 0x000002DC).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000002DC) GPIO port D data pin status                                */
  
  struct {
    __IM  uint32_t dp_sts     : 32;           /*!< [31..0] For each bit reading: If the direction of the specified
                                                   bit is input then read the signal level of the input pin.
                                                   If the direction of the specified bit is output then reads
                                                   the output data register for port D.                                      */
  } b;                                        /*!< bit fields for gpio_port_d_dp_sts */
} gpio_port_d_dp_sts_t, *pgpio_port_d_dp_sts_t;

/** @} */ /* End of group ls_hal_gpio_reg */
/// @endcond /* End of condition DOXYGEN_GENERAL_REG_TYPE || DOXYGEN_GPIO_REG_TYPE */


#ifdef  __cplusplus
}
#endif

#endif    // end of #ifndef _RTL8710C_GPIO_TYPE_H_

