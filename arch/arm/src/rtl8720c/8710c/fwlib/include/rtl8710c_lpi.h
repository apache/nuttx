/**************************************************************************//**
 * @file     rtl8710c_lpi.h
 * @brief    The rtl8710c platform low priority interrupt handler header file.
 * @version  V1.00
 * @date     2016-07-04
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

#ifndef _RTL8710C_LPI_H_
#define _RTL8710C_LPI_H_
#include "basic_types.h"
 
#ifdef  __cplusplus
extern "C"
{
#endif

/* define the Low Priority Interrupt trigger signal type */
enum  low_pri_int_mode_e
{
    LPInt_Level_Trigger       = 0,      ///< LP Interrupt signal = High Level
    LPInt_Edge_Trigger        = 1       ///< LP Interrupt signal = Rising Edge
};
typedef uint8_t low_pri_int_mode_t;

/* define the Low Priority Interrupt event ID */
enum  low_pri_int_event_e
{
    LPInt_RXI_Bus       = 0,    // RXI bus
    LPInt_SPIC_Flash    = 1,    // SPI Flash controller
    LPInt_PSRAM_Cal     = 2,    // PSRAM calibration fail
    LPInt_PSRAM_Timeout = 3     // PSRAM calibration timeout
};
typedef uint8_t lowpri_int_id_t;

typedef struct hal_lpi_int_s {
    irq_handler_t rxi_bus_handler;      /* the RXI bus error interrupt handler */
    void *rxi_bus_arg;                  /* the argument of RXI bus error interrupt handler */
    irq_handler_t spic_handler;         /* the SPI Flash controller interrupt handler */
    void *spic_arg;                     /* the argument of SPI Flash controller interrupt handler */
    irq_handler_t psram_cal_handler;    /* the PSRAM Calibration error interrupt handler */
    void *psram_cal_arg;                /* the argument of PSRAM Calibration error interrupt handler */
    irq_handler_t psram_timeout_handler;    /* the PSRAM Calibration timeout interrupt handler */
    void *psram_timeout_arg;            /* the argument of PSRAM Calibration timeout interrupt handler */
} hal_lpi_int_t, *phal_lpi_int_t;

void hal_lpi_init_rtl8710c (hal_lpi_int_t *plpi_hdl);
void hal_lpi_handler_reg_rtl8710c (lowpri_int_id_t int_id, low_pri_int_mode_t int_trig_type,
                                            irq_handler_t handler, void *arg);
void hal_lpi_en_rtl8710c (lowpri_int_id_t int_id);
void hal_lpi_dis_rtl8710c (lowpri_int_id_t int_id);
void hal_lpi_reg_irq_rtl8710c (uint32_t handler, uint8_t priority);

/**
  \brief  The data structure of the stubs function for the Low Priority Interrupt API functions in ROM
*/
typedef struct hal_lpi_func_stubs_s {
    /* Low Priority Interrupt API*/
    void (*hal_lpi_init) (hal_lpi_int_t *plpi_hdl);
    void (*hal_lpi_handler_reg) (lowpri_int_id_t int_id, low_pri_int_mode_t int_trig_type,
                                 irq_handler_t handler, void *arg);
    void (*hal_lpi_en) (lowpri_int_id_t int_id);
    void (*hal_lpi_dis) (lowpri_int_id_t int_id);
    void (*hal_lpi_reg_irq) (uint32_t handler, uint8_t priority);
    
    uint32_t reserved[8];  // reserved space for next ROM code version function table extending.
} hal_lpi_func_stubs_t;


#ifdef  __cplusplus
}
#endif

#endif  // end of "#define _RTL8710C_LPI_H_"

