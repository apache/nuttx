/**************************************************************************//**
 * @file     rtl819bhp_i2c.h
 * @brief    The fundamental definition for RTL8195B HP i2C module.
 * @version  V1.00
 * @date     2017-03-07
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



#ifndef _RTL8710C_I2C_H_
#define _RTL8710C_I2C_H_

#include "rtl8710c_i2c_type.h"

#ifdef __cplusplus
extern "C" {
#endif
/**
 * @addtogroup hs_hal_i2c I2C
 * @{
 */

//#define LOCAL_SVD_DECLARATION
#undef LOCAL_SVD_DECLARATION
//this flag should be checked again when system integration is done.

#define TX_ABRT_TEMP_PATCH  //This flag should be removed when tx_abrt bit15 is fixed to a correct hardware in DMA mode

/// i2c module number
#define I2C_NO               1
/// i2c IRQ Priority
#define I2C_IRQ_PR           3
/// i2c address retry time-out
#define I2C_ATRY_TO          3000
/// i2c master mode address retry function
#define I2C_ADD_RETRY        1
/// i2c master mode address retry max count
#define I2C_ADD_RTY_MAX      256
/// i2c default DMA TX src width
#define I2C_DMA_DEFAULT_TX_SRC_WIDTH     4
/// i2c default DMA RX des width
#define I2C_DMA_DEFAULT_RX_DES_WIDTH     4
/// i2c timeout check disabled
#define I2C_TIMEOUT_DISABLE  0x00
/// i2c timeout check endless
#define I2C_TIMEOUT_ENDLESS  0xFFFFFFFF
/// i2c TX FIFO entry number
#define I2C_TX_FIFO_DEPTH    32
/// i2c RX FIFO entry number
#define I2C_RX_FIFO_DEPTH    32
/// i2c DMA transfer max length
#define I2C_DMA_MAX_LEN      4095
/// i2c filter CAP. low length
#define I2C_FLTR_CAP_L_LEN   20
/// i2c filter CAP. main length
#define I2C_FLTR_CAP_M_LEN   5
/// i2c filter Resis. low length
#define I2C_FLTR_RSTS_L_LEN  20
/// i2c filter Resis. main length
#define I2C_FLTR_RSTS_M_LEN  20

/** @defgroup i2c_reg_bit_macro I2C REG BIT MACRO
 *  bit-operation macros for i2c registers
 *  @{
 */
/** @} */ // end of I2C REG BIT MACRO


/*  Bit shift macros for i2c_con_t  */
/** @ingroup i2c_reg_bit_macro
 *  bit-operation macros for IC_SLAVE_DISABLE_1
 *  @{
 */
#define BIT_IC_SLAVE_DISABLE_1                             BIT7
#define BIT_SHIFT_IC_SLAVE_DISABLE_1                       7
#define BIT_MASK_IC_SLAVE_DISABLE_1                        0x1
#define BIT_CTRL_IC_SLAVE_DISABLE_1(x)                     (((x) &  BIT_MASK_IC_SLAVE_DISABLE_1) << BIT_SHIFT_IC_SLAVE_DISABLE_1)
#define BIT_GET_IC_SLAVE_DISABLE_1(x)                      (((x) >>  BIT_SHIFT_IC_SLAVE_DISABLE_1) & BIT_MASK_IC_SLAVE_DISABLE_1)
/** @} */

/** @ingroup i2c_reg_bit_macro
 *  bit-operation macros for IC_SLAVE_DISABLE
 *  @{
 */
#define BIT_IC_SLAVE_DISABLE                               BIT6
#define BIT_SHIFT_IC_SLAVE_DISABLE                         6
#define BIT_MASK_IC_SLAVE_DISABLE                          0x1
#define BIT_CTRL_IC_SLAVE_DISABLE(x)                       (((x) &  BIT_MASK_IC_SLAVE_DISABLE) << BIT_SHIFT_IC_SLAVE_DISABLE)
#define BIT_GET_IC_SLAVE_DISABLE(x)                        (((x) >>  BIT_SHIFT_IC_SLAVE_DISABLE) & BIT_MASK_IC_SLAVE_DISABLE)
/** @} */

/** @ingroup i2c_reg_bit_macro
 *  bit-operation macros for IC_RESTART_EN
 *  @{
 */
#define BIT_IC_RESTART_EN                                  BIT5
#define BIT_SHIFT_IC_RESTART_EN                            5
#define BIT_MASK_IC_RESTART_EN                             0x1
#define BIT_CTRL_IC_RESTART_EN(x)                          (((x) &  BIT_MASK_IC_RESTART_EN) << BIT_SHIFT_IC_RESTART_EN)
#define BIT_GET_IC_RESTART_EN(x)                           (((x) >>  BIT_SHIFT_IC_RESTART_EN) & BIT_MASK_IC_RESTART_EN)
/** @} */

/**
 *  bit-operation macros for IC_10BITADDR_MASTER
 *  @ingroup i2c_reg_bit_macro
 *  @{
 */
#define BIT_IC_10BITADDR_MASTER                            BIT4
#define BIT_SHIFT_IC_10BITADDR_MASTER                      4
#define BIT_MASK_IC_10BITADDR_MASTER                       0x1
#define BIT_CTRL_IC_10BITADDR_MASTER(x)                    (((x) &  BIT_MASK_IC_10BITADDR_MASTER) << BIT_SHIFT_IC_10BITADDR_MASTER)
#define BIT_GET_IC_10BITADDR_MASTER(x)                     (((x) >>  BIT_SHIFT_IC_10BITADDR_MASTER) & BIT_MASK_IC_10BITADDR_MASTER)
/** @} */

/**
 *  bit-operation macros for IC_10BITADDR_SLAVE
 *  @ingroup i2c_reg_bit_macro
 *  @{
 */
#define BIT_IC_10BITADDR_SLAVE                             BIT3
#define BIT_SHIFT_IC_10BITADDR_SLAVE                       3
#define BIT_MASK_IC_10BITADDR_SLAVE                        0x1
#define BIT_CTRL_IC_10BITADDR_SLAVE(x)                     (((x) &  BIT_MASK_IC_10BITADDR_SLAVE) << BIT_SHIFT_IC_10BITADDR_SLAVE)
#define BIT_GET_IC_10BITADDR_SLAVE(x)                      (((x) >>  BIT_SHIFT_IC_10BITADDR_SLAVE) & BIT_MASK_IC_10BITADDR_SLAVE)
/** @} */

/**
 *  bit-operation macros for SPEED
 *  @ingroup i2c_reg_bit_macro
 *  @{
 */
#define BIT_SHIFT_SPEED                                    1
#define BIT_MASK_SPEED                                     0x3
#define BIT_CTRL_SPEED(x)                                  (((x) &  BIT_MASK_SPEED) << BIT_SHIFT_SPEED)
#define BIT_GET_SPEED(x)                                   (((x) >>  BIT_SHIFT_SPEED) & BIT_MASK_SPEED)
/** @} */

/**
 *  bit-operation macros for MASTER_MODE
 *  @ingroup i2c_reg_bit_macro
 *  @{
 */
#define BIT_MASTER_MODE                                    BIT0
#define BIT_SHIFT_MASTER_MODE                              0
#define BIT_MASK_MASTER_MODE                               0x1
#define BIT_CTRL_MASTER_MODE(x)                            (((x) &  BIT_MASK_MASTER_MODE) << BIT_SHIFT_MASTER_MODE)
#define BIT_GET_MASTER_MODE(x)                             (((x) >>  BIT_SHIFT_MASTER_MODE) & BIT_MASK_MASTER_MODE)
/** @} */


/*  Bit shift macros for i2c_dat_cmd_t  */
/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for NULL_DATA
 *  @{
 */
#define BIT_NULL_DATA                                      BIT11
#define BIT_SHIFT_NULL_DATA                                11
#define BIT_MASK_NULL_DATA                                 0x1
#define BIT_CTRL_NULL_DATA(x)                              (((x) &  BIT_MASK_NULL_DATA) << BIT_SHIFT_NULL_DATA)
#define BIT_GET_NULL_DATA(x)                               (((x) >>  BIT_SHIFT_NULL_DATA) & BIT_MASK_NULL_DATA)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for RESTART
 *  @{
 */
#define BIT_RESTART                                        BIT10
#define BIT_SHIFT_RESTART                                  10
#define BIT_MASK_RESTART                                   0x1
#define BIT_CTRL_RESTART(x)                                (((x) &  BIT_MASK_RESTART) << BIT_SHIFT_RESTART)
#define BIT_GET_RESTART(x)                                 (((x) >>  BIT_SHIFT_RESTART) & BIT_MASK_RESTART)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for STOP
 *  @{
 */
#define BIT_STOP                                           BIT9
#define BIT_SHIFT_STOP                                     9
#define BIT_MASK_STOP                                      0x1
#define BIT_CTRL_STOP(x)                                   (((x) &  BIT_MASK_STOP) << BIT_SHIFT_STOP)
#define BIT_GET_STOP(x)                                    (((x) >>  BIT_SHIFT_STOP) & BIT_MASK_STOP)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for CMD
 *  @{
 */
#define BIT_CMD                                            BIT8
#define BIT_SHIFT_CMD                                      8
#define BIT_MASK_CMD                                       0x1
#define BIT_CTRL_CMD(x)                                    (((x) &  BIT_MASK_CMD) << BIT_SHIFT_CMD)
#define BIT_GET_CMD(x)                                     (((x) >>  BIT_SHIFT_CMD) & BIT_MASK_CMD)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for DAT
 *  @{
 */
#define BIT_SHIFT_DAT                                      0
#define BIT_MASK_DAT                                       0xFF
#define BIT_CTRL_DAT(x)                                    (((x) &  BIT_MASK_DAT) << BIT_SHIFT_DAT)
#define BIT_GET_DAT(x)                                     (((x) >>  BIT_SHIFT_DAT) & BIT_MASK_DAT)
/** @} */

/*  Bit shift macros for i2c_intr_msk_t  */
/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for M_DMA_I2C_DONE
 *  @{
 */
#define BIT_M_DMA_I2C_DONE                                 BIT15
#define BIT_SHIFT_M_DMA_I2C_DONE                           15
#define BIT_MASK_M_DMA_I2C_DONE                            0x1
#define BIT_CTRL_M_DMA_I2C_DONE(x)                         (((x) &  BIT_MASK_M_DMA_I2C_DONE) << BIT_SHIFT_M_DMA_I2C_DONE)
#define BIT_GET_M_DMA_I2C_DONE(x)                          (((x) >>  BIT_SHIFT_M_DMA_I2C_DONE) & BIT_MASK_M_DMA_I2C_DONE)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for M_MS_CODE_DET
 *  @{
 */
#define BIT_M_MS_CODE_DET                                  BIT14
#define BIT_SHIFT_M_MS_CODE_DET                            14
#define BIT_MASK_M_MS_CODE_DET                             0x1
#define BIT_CTRL_M_MS_CODE_DET(x)                          (((x) &  BIT_MASK_M_MS_CODE_DET) << BIT_SHIFT_M_MS_CODE_DET)
#define BIT_GET_M_MS_CODE_DET(x)                           (((x) >>  BIT_SHIFT_M_MS_CODE_DET) & BIT_MASK_M_MS_CODE_DET)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for M_ADDR_1_MATCH
 *  @{
 */
#define BIT_M_ADDR_1_MATCH                                 BIT13
#define BIT_SHIFT_M_ADDR_1_MATCH                           13
#define BIT_MASK_M_ADDR_1_MATCH                            0x1
#define BIT_CTRL_M_ADDR_1_MATCH(x)                         (((x) &  BIT_MASK_M_ADDR_1_MATCH) << BIT_SHIFT_M_ADDR_1_MATCH)
#define BIT_GET_M_ADDR_1_MATCH(x)                          (((x) >>  BIT_SHIFT_M_ADDR_1_MATCH) & BIT_MASK_M_ADDR_1_MATCH)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for M_ADDR_0_MATCH
 *  @{
 */
#define BIT_M_ADDR_0_MATCH                                 BIT12
#define BIT_SHIFT_M_ADDR_0_MATCH                           12
#define BIT_MASK_M_ADDR_0_MATCH                            0x1
#define BIT_CTRL_M_ADDR_0_MATCH(x)                         (((x) &  BIT_MASK_M_ADDR_0_MATCH) << BIT_SHIFT_M_ADDR_0_MATCH)
#define BIT_GET_M_ADDR_0_MATCH(x)                          (((x) >>  BIT_SHIFT_M_ADDR_0_MATCH) & BIT_MASK_M_ADDR_0_MATCH)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for M_GEN_CALL
 *  @{
 */
#define BIT_M_GEN_CALL                                     BIT11
#define BIT_SHIFT_M_GEN_CALL                               11
#define BIT_MASK_M_GEN_CALL                                0x1
#define BIT_CTRL_M_GEN_CALL(x)                             (((x) &  BIT_MASK_M_GEN_CALL) << BIT_SHIFT_M_GEN_CALL)
#define BIT_GET_M_GEN_CALL(x)                              (((x) >>  BIT_SHIFT_M_GEN_CALL) & BIT_MASK_M_GEN_CALL)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for M_START_DET
 *  @{
 */
#define BIT_M_START_DET                                    BIT10
#define BIT_SHIFT_M_START_DET                              10
#define BIT_MASK_M_START_DET                               0x1
#define BIT_CTRL_M_START_DET(x)                            (((x) &  BIT_MASK_M_START_DET) << BIT_SHIFT_M_START_DET)
#define BIT_GET_M_START_DET(x)                             (((x) >>  BIT_SHIFT_M_START_DET) & BIT_MASK_M_START_DET)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for M_STOP_DET
 *  @{
 */
#define BIT_M_STOP_DET                                     BIT9
#define BIT_SHIFT_M_STOP_DET                               9
#define BIT_MASK_M_STOP_DET                                0x1
#define BIT_CTRL_M_STOP_DET(x)                             (((x) &  BIT_MASK_M_STOP_DET) << BIT_SHIFT_M_STOP_DET)
#define BIT_GET_M_STOP_DET(x)                              (((x) >>  BIT_SHIFT_M_STOP_DET) & BIT_MASK_M_STOP_DET)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for M_ACTIVITY
 *  @{
 */
#define BIT_M_ACTIVITY                                     BIT8
#define BIT_SHIFT_M_ACTIVITY                               8
#define BIT_MASK_M_ACTIVITY                                0x1
#define BIT_CTRL_M_ACTIVITY(x)                             (((x) &  BIT_MASK_M_ACTIVITY) << BIT_SHIFT_M_ACTIVITY)
#define BIT_GET_M_ACTIVITY(x)                              (((x) >>  BIT_SHIFT_M_ACTIVITY) & BIT_MASK_M_ACTIVITY)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for M_RX_DONE
 *  @{
 */
#define BIT_M_RX_DONE                                      BIT7
#define BIT_SHIFT_M_RX_DONE                                7
#define BIT_MASK_M_RX_DONE                                 0x1
#define BIT_CTRL_M_RX_DONE(x)                              (((x) &  BIT_MASK_M_RX_DONE) << BIT_SHIFT_M_RX_DONE)
#define BIT_GET_M_RX_DONE(x)                               (((x) >>  BIT_SHIFT_M_RX_DONE) & BIT_MASK_M_RX_DONE)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for M_TX_ABRT
 *  @{
 */
#define BIT_M_TX_ABRT                                      BIT6
#define BIT_SHIFT_M_TX_ABRT                                6
#define BIT_MASK_M_TX_ABRT                                 0x1
#define BIT_CTRL_M_TX_ABRT(x)                              (((x) &  BIT_MASK_M_TX_ABRT) << BIT_SHIFT_M_TX_ABRT)
#define BIT_GET_M_TX_ABRT(x)                               (((x) >>  BIT_SHIFT_M_TX_ABRT) & BIT_MASK_M_TX_ABRT)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for M_RD_REQ
 *  @{
 */
#define BIT_M_RD_REQ                                       BIT5
#define BIT_SHIFT_M_RD_REQ                                 5
#define BIT_MASK_M_RD_REQ                                  0x1
#define BIT_CTRL_M_RD_REQ(x)                               (((x) &  BIT_MASK_M_RD_REQ) << BIT_SHIFT_M_RD_REQ)
#define BIT_GET_M_RD_REQ(x)                                (((x) >>  BIT_SHIFT_M_RD_REQ) & BIT_MASK_M_RD_REQ)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for M_TX_EMPTY
 *  @{
 */
#define BIT_M_TX_EMPTY                                     BIT4
#define BIT_SHIFT_M_TX_EMPTY                               4
#define BIT_MASK_M_TX_EMPTY                                0x1
#define BIT_CTRL_M_TX_EMPTY(x)                             (((x) &  BIT_MASK_M_TX_EMPTY) << BIT_SHIFT_M_TX_EMPTY)
#define BIT_GET_M_TX_EMPTY(x)                              (((x) >>  BIT_SHIFT_M_TX_EMPTY) & BIT_MASK_M_TX_EMPTY)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for M_TX_OVER
 *  @{
 */
#define BIT_M_TX_OVER                                      BIT3
#define BIT_SHIFT_M_TX_OVER                                3
#define BIT_MASK_M_TX_OVER                                 0x1
#define BIT_CTRL_M_TX_OVER(x)                              (((x) &  BIT_MASK_M_TX_OVER) << BIT_SHIFT_M_TX_OVER)
#define BIT_GET_M_TX_OVER(x)                               (((x) >>  BIT_SHIFT_M_TX_OVER) & BIT_MASK_M_TX_OVER)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for M_RX_FULL
 *  @{
 */
#define BIT_M_RX_FULL                                      BIT2
#define BIT_SHIFT_M_RX_FULL                                2
#define BIT_MASK_M_RX_FULL                                 0x1
#define BIT_CTRL_M_RX_FULL(x)                              (((x) &  BIT_MASK_M_RX_FULL) << BIT_SHIFT_M_RX_FULL)
#define BIT_GET_M_RX_FULL(x)                               (((x) >>  BIT_SHIFT_M_RX_FULL) & BIT_MASK_M_RX_FULL)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for M_RX_OVER
 *  @{
 */
#define BIT_M_RX_OVER                                      BIT1
#define BIT_SHIFT_M_RX_OVER                                1
#define BIT_MASK_M_RX_OVER                                 0x1
#define BIT_CTRL_M_RX_OVER(x)                              (((x) &  BIT_MASK_M_RX_OVER) << BIT_SHIFT_M_RX_OVER)
#define BIT_GET_M_RX_OVER(x)                               (((x) >>  BIT_SHIFT_M_RX_OVER) & BIT_MASK_M_RX_OVER)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for M_RX_UNDER
 *  @{
 */
#define BIT_M_RX_UNDER                                     BIT0
#define BIT_SHIFT_M_RX_UNDER                               0
#define BIT_MASK_M_RX_UNDER                                0x1
#define BIT_CTRL_M_RX_UNDER(x)                             (((x) &  BIT_MASK_M_RX_UNDER) << BIT_SHIFT_M_RX_UNDER)
#define BIT_GET_M_RX_UNDER(x)                              (((x) >>  BIT_SHIFT_M_RX_UNDER) & BIT_MASK_M_RX_UNDER)
/** @} */

/*  Bit shift macros for i2c_raw_intr_stat_t  */
/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for RAW_DMA_I2C_DONE
 *  @{
 */
#define BIT_RAW_DMA_I2C_DONE                               BIT15
#define BIT_SHIFT_RAW_DMA_I2C_DONE                         15
#define BIT_MASK_RAW_DMA_I2C_DONE                          0x1
#define BIT_CTRL_RAW_DMA_I2C_DONE(x)                       (((x) &  BIT_MASK_RAW_DMA_I2C_DONE) << BIT_SHIFT_RAW_DMA_I2C_DONE)
#define BIT_GET_RAW_DMA_I2C_DONE(x)                        (((x) >>  BIT_SHIFT_RAW_DMA_I2C_DONE) & BIT_MASK_RAW_DMA_I2C_DONE)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for RAW_MS_CODE_DET
 *  @{
 */
#define BIT_RAW_MS_CODE_DET                                BIT14
#define BIT_SHIFT_RAW_MS_CODE_DET                          14
#define BIT_MASK_RAW_MS_CODE_DET                           0x1
#define BIT_CTRL_RAW_MS_CODE_DET(x)                        (((x) &  BIT_MASK_RAW_MS_CODE_DET) << BIT_SHIFT_RAW_MS_CODE_DET)
#define BIT_GET_RAW_MS_CODE_DET(x)                         (((x) >>  BIT_SHIFT_RAW_MS_CODE_DET) & BIT_MASK_RAW_MS_CODE_DET)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for RAW_ADDR_1_MATCH
 *  @{
 */
#define BIT_RAW_ADDR_1_MATCH                               BIT13
#define BIT_SHIFT_RAW_ADDR_1_MATCH                         13
#define BIT_MASK_RAW_ADDR_1_MATCH                          0x1
#define BIT_CTRL_RAW_ADDR_1_MATCH(x)                       (((x) &  BIT_MASK_RAW_ADDR_1_MATCH) << BIT_SHIFT_RAW_ADDR_1_MATCH)
#define BIT_GET_RAW_ADDR_1_MATCH(x)                        (((x) >>  BIT_SHIFT_RAW_ADDR_1_MATCH) & BIT_MASK_RAW_ADDR_1_MATCH)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for RAW_ADDR_0_MATCH
 *  @{
 */
#define BIT_RAW_ADDR_0_MATCH                               BIT12
#define BIT_SHIFT_RAW_ADDR_0_MATCH                         12
#define BIT_MASK_RAW_ADDR_0_MATCH                          0x1
#define BIT_CTRL_RAW_ADDR_0_MATCH(x)                       (((x) &  BIT_MASK_RAW_ADDR_0_MATCH) << BIT_SHIFT_RAW_ADDR_0_MATCH)
#define BIT_GET_RAW_ADDR_0_MATCH(x)                        (((x) >>  BIT_SHIFT_RAW_ADDR_0_MATCH) & BIT_MASK_RAW_ADDR_0_MATCH)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for RAW_GEN_CALL
 *  @{
 */
#define BIT_RAW_GEN_CALL                                   BIT11
#define BIT_SHIFT_RAW_GEN_CALL                             11
#define BIT_MASK_RAW_GEN_CALL                              0x1
#define BIT_CTRL_RAW_GEN_CALL(x)                           (((x) &  BIT_MASK_RAW_GEN_CALL) << BIT_SHIFT_RAW_GEN_CALL)
#define BIT_GET_RAW_GEN_CALL(x)                            (((x) >>  BIT_SHIFT_RAW_GEN_CALL) & BIT_MASK_RAW_GEN_CALL)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for RAW_START_DET
 *  @{
 */
#define BIT_RAW_START_DET                                  BIT10
#define BIT_SHIFT_RAW_START_DET                            10
#define BIT_MASK_RAW_START_DET                             0x1
#define BIT_CTRL_RAW_START_DET(x)                          (((x) &  BIT_MASK_RAW_START_DET) << BIT_SHIFT_RAW_START_DET)
#define BIT_GET_RAW_START_DET(x)                           (((x) >>  BIT_SHIFT_RAW_START_DET) & BIT_MASK_RAW_START_DET)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for RAW_STOP_DET
 *  @{
 */
#define BIT_RAW_STOP_DET                                   BIT9
#define BIT_SHIFT_RAW_STOP_DET                             9
#define BIT_MASK_RAW_STOP_DET                              0x1
#define BIT_CTRL_RAW_STOP_DET(x)                           (((x) &  BIT_MASK_RAW_STOP_DET) << BIT_SHIFT_RAW_STOP_DET)
#define BIT_GET_RAW_STOP_DET(x)                            (((x) >>  BIT_SHIFT_RAW_STOP_DET) & BIT_MASK_RAW_STOP_DET)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for RAW_ACTIVITY
 *  @{
 */
#define BIT_RAW_ACTIVITY                                   BIT8
#define BIT_SHIFT_RAW_ACTIVITY                             8
#define BIT_MASK_RAW_ACTIVITY                              0x1
#define BIT_CTRL_RAW_ACTIVITY(x)                           (((x) &  BIT_MASK_RAW_ACTIVITY) << BIT_SHIFT_RAW_ACTIVITY)
#define BIT_GET_RAW_ACTIVITY(x)                            (((x) >>  BIT_SHIFT_RAW_ACTIVITY) & BIT_MASK_RAW_ACTIVITY)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for RAW_RX_DONE
 *  @{
 */
#define BIT_RAW_RX_DONE                                    BIT7
#define BIT_SHIFT_RAW_RX_DONE                              7
#define BIT_MASK_RAW_RX_DONE                               0x1
#define BIT_CTRL_RAW_RX_DONE(x)                            (((x) &  BIT_MASK_RAW_RX_DONE) << BIT_SHIFT_RAW_RX_DONE)
#define BIT_GET_RAW_RX_DONE(x)                             (((x) >>  BIT_SHIFT_RAW_RX_DONE) & BIT_MASK_RAW_RX_DONE)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for RAW_TX_ABRT
 *  @{
 */
#define BIT_RAW_TX_ABRT                                    BIT6
#define BIT_SHIFT_RAW_TX_ABRT                              6
#define BIT_MASK_RAW_TX_ABRT                               0x1
#define BIT_CTRL_RAW_TX_ABRT(x)                            (((x) &  BIT_MASK_RAW_TX_ABRT) << BIT_SHIFT_RAW_TX_ABRT)
#define BIT_GET_RAW_TX_ABRT(x)                             (((x) >>  BIT_SHIFT_RAW_TX_ABRT) & BIT_MASK_RAW_TX_ABRT)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for RAW_RD_REQ
 *  @{
 */
#define BIT_RAW_RD_REQ                                     BIT5
#define BIT_SHIFT_RAW_RD_REQ                               5
#define BIT_MASK_RAW_RD_REQ                                0x1
#define BIT_CTRL_RAW_RD_REQ(x)                             (((x) &  BIT_MASK_RAW_RD_REQ) << BIT_SHIFT_RAW_RD_REQ)
#define BIT_GET_RAW_RD_REQ(x)                              (((x) >>  BIT_SHIFT_RAW_RD_REQ) & BIT_MASK_RAW_RD_REQ)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for RAW_TX_EMPTY
 *  @{
 */
#define BIT_RAW_TX_EMPTY                                   BIT4
#define BIT_SHIFT_RAW_TX_EMPTY                             4
#define BIT_MASK_RAW_TX_EMPTY                              0x1
#define BIT_CTRL_RAW_TX_EMPTY(x)                           (((x) &  BIT_MASK_RAW_TX_EMPTY) << BIT_SHIFT_RAW_TX_EMPTY)
#define BIT_GET_RAW_TX_EMPTY(x)                            (((x) >>  BIT_SHIFT_RAW_TX_EMPTY) & BIT_MASK_RAW_TX_EMPTY)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for RAW_TX_OVER
 *  @{
 */
#define BIT_RAW_TX_OVER                                    BIT3
#define BIT_SHIFT_RAW_TX_OVER                              3
#define BIT_MASK_RAW_TX_OVER                               0x1
#define BIT_CTRL_RAW_TX_OVER(x)                            (((x) &  BIT_MASK_RAW_TX_OVER) << BIT_SHIFT_RAW_TX_OVER)
#define BIT_GET_RAW_TX_OVER(x)                             (((x) >>  BIT_SHIFT_RAW_TX_OVER) & BIT_MASK_RAW_TX_OVER)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for RAW_RX_FULL
 *  @{
 */
#define BIT_RAW_RX_FULL                                    BIT2
#define BIT_SHIFT_RAW_RX_FULL                              2
#define BIT_MASK_RAW_RX_FULL                               0x1
#define BIT_CTRL_RAW_RX_FULL(x)                            (((x) &  BIT_MASK_RAW_RX_FULL) << BIT_SHIFT_RAW_RX_FULL)
#define BIT_GET_RAW_RX_FULL(x)                             (((x) >>  BIT_SHIFT_RAW_RX_FULL) & BIT_MASK_RAW_RX_FULL)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for RAW_RX_OVER
 *  @{
 */
#define BIT_RAW_RX_OVER                                    BIT1
#define BIT_SHIFT_RAW_RX_OVER                              1
#define BIT_MASK_RAW_RX_OVER                               0x1
#define BIT_CTRL_RAW_RX_OVER(x)                            (((x) &  BIT_MASK_RAW_RX_OVER) << BIT_SHIFT_RAW_RX_OVER)
#define BIT_GET_RAW_RX_OVER(x)                             (((x) >>  BIT_SHIFT_RAW_RX_OVER) & BIT_MASK_RAW_RX_OVER)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for RAW_RX_UNDER
 *  @{
 */
#define BIT_RAW_RX_UNDER                                   BIT0
#define BIT_SHIFT_RAW_RX_UNDER                             0
#define BIT_MASK_RAW_RX_UNDER                              0x1
#define BIT_CTRL_RAW_RX_UNDER(x)                           (((x) &  BIT_MASK_RAW_RX_UNDER) << BIT_SHIFT_RAW_RX_UNDER)
#define BIT_GET_RAW_RX_UNDER(x)                            (((x) >>  BIT_SHIFT_RAW_RX_UNDER) & BIT_MASK_RAW_RX_UNDER)
/** @} */

/*  Bit shift macros for i2c_sts_t  */
/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for BUS_STATUS
 *  @{
 */
#define BIT_SHIFT_BUS_STATUS                               BIT11
#define BIT_MASK_BUS_STATUS                                0x3
#define BIT_CTRL_BUS_STATUS(x)                             (((x) &  BIT_MASK_BUS_STATUS) << BIT_SHIFT_BUS_STATUS)
#define BIT_GET_BUS_STATUS(x)                              (((x) >>  BIT_SHIFT_BUS_STATUS) & BIT_MASK_BUS_STATUS)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for SLV_HOLD_RX_FIFO_FULL
 *  @{
 */
#define BIT_SLV_HOLD_RX_FIFO_FULL                          BIT10
#define BIT_SHIFT_SLV_HOLD_RX_FIFO_FULL                    10
#define BIT_MASK_SLV_HOLD_RX_FIFO_FULL                     0x1
#define BIT_CTRL_SLV_HOLD_RX_FIFO_FULL(x)                  (((x) &  BIT_MASK_SLV_HOLD_RX_FIFO_FULL) << BIT_SHIFT_SLV_HOLD_RX_FIFO_FULL)
#define BIT_GET_SLV_HOLD_RX_FIFO_FULL(x)                   (((x) >>  BIT_SHIFT_SLV_HOLD_RX_FIFO_FULL) & BIT_MASK_SLV_HOLD_RX_FIFO_FULL)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for SLV_HOLD_TX_FIFO_EMPTY
 *  @{
 */
#define BIT_SLV_HOLD_TX_FIFO_EMPTY                         BIT9
#define BIT_SHIFT_SLV_HOLD_TX_FIFO_EMPTY                   9
#define BIT_MASK_SLV_HOLD_TX_FIFO_EMPTY                    0x1
#define BIT_CTRL_SLV_HOLD_TX_FIFO_EMPTY(x)                 (((x) &  BIT_MASK_SLV_HOLD_TX_FIFO_EMPTY) << BIT_SHIFT_SLV_HOLD_TX_FIFO_EMPTY)
#define BIT_GET_SLV_HOLD_TX_FIFO_EMPTY(x)                  (((x) >>  BIT_SHIFT_SLV_HOLD_TX_FIFO_EMPTY) & BIT_MASK_SLV_HOLD_TX_FIFO_EMPTY)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for MST_HOLD_RX_FIFO_FULL
 *  @{
 */
#define BIT_MST_HOLD_RX_FIFO_FULL                          BIT8
#define BIT_SHIFT_MST_HOLD_RX_FIFO_FULL                    8
#define BIT_MASK_MST_HOLD_RX_FIFO_FULL                     0x1
#define BIT_CTRL_MST_HOLD_RX_FIFO_FULL(x)                  (((x) &  BIT_MASK_MST_HOLD_RX_FIFO_FULL) << BIT_SHIFT_MST_HOLD_RX_FIFO_FULL)
#define BIT_GET_MST_HOLD_RX_FIFO_FULL(x)                   (((x) >>  BIT_SHIFT_MST_HOLD_RX_FIFO_FULL) & BIT_MASK_MST_HOLD_RX_FIFO_FULL)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for MST_HOLD_TX_FIFO_EMPTY
 *  @{
 */
#define BIT_MST_HOLD_TX_FIFO_EMPTY                         BIT7
#define BIT_SHIFT_MST_HOLD_TX_FIFO_EMPTY                   7
#define BIT_MASK_MST_HOLD_TX_FIFO_EMPTY                    0x1
#define BIT_CTRL_MST_HOLD_TX_FIFO_EMPTY(x)                 (((x) &  BIT_MASK_MST_HOLD_TX_FIFO_EMPTY) << BIT_SHIFT_MST_HOLD_TX_FIFO_EMPTY)
#define BIT_GET_MST_HOLD_TX_FIFO_EMPTY(x)                  (((x) >>  BIT_SHIFT_MST_HOLD_TX_FIFO_EMPTY) & BIT_MASK_MST_HOLD_TX_FIFO_EMPTY)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for SLV_ACTIVITY
 *  @{
 */
#define BIT_SLV_ACTIVITY                                   BIT6
#define BIT_SHIFT_SLV_ACTIVITY                             6
#define BIT_MASK_SLV_ACTIVITY                              0x1
#define BIT_CTRL_SLV_ACTIVITY(x)                           (((x) &  BIT_MASK_SLV_ACTIVITY) << BIT_SHIFT_SLV_ACTIVITY)
#define BIT_GET_SLV_ACTIVITY(x)                            (((x) >>  BIT_SHIFT_SLV_ACTIVITY) & BIT_MASK_SLV_ACTIVITY)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for MST_ACTIVITY
 *  @{
 */
#define BIT_MST_ACTIVITY                                   BIT5
#define BIT_SHIFT_MST_ACTIVITY                             5
#define BIT_MASK_MST_ACTIVITY                              0x1
#define BIT_CTRL_MST_ACTIVITY(x)                           (((x) &  BIT_MASK_MST_ACTIVITY) << BIT_SHIFT_MST_ACTIVITY)
#define BIT_GET_MST_ACTIVITY(x)                            (((x) >>  BIT_SHIFT_MST_ACTIVITY) & BIT_MASK_MST_ACTIVITY)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for RFF
 *  @{
 */
#define BIT_IC_RFF                                         BIT4
#define BIT_SHIFT_IC_RFF                                   4
#define BIT_MASK_IC_RFF                                    0x1
#define BIT_CTRL_IC_RFF(x)                                 (((x) &  BIT_MASK_IC_RFF) << BIT_SHIFT_IC_RFF)
#define BIT_GET_IC_RFF(x)                                  (((x) >>  BIT_SHIFT_IC_RFF) & BIT_MASK_IC_RFF)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for RFNE
 *  @{
 */
#define BIT_IC_RFNE                                        BIT3
#define BIT_SHIFT_IC_RFNE                                  3
#define BIT_MASK_IC_RFNE                                   0x1
#define BIT_CTRL_IC_RFNE(x)                                (((x) &  BIT_MASK_IC_RFNE) << BIT_SHIFT_IC_RFNE)
#define BIT_GET_IC_RFNE(x)                                 (((x) >>  BIT_SHIFT_IC_RFNE) & BIT_MASK_IC_RFNE)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for TFE
 *  @{
 */
#define BIT_IC_TFE                                         BIT2
#define BIT_SHIFT_IC_TFE                                   2
#define BIT_MASK_IC_TFE                                    0x1
#define BIT_CTRL_IC_TFE(x)                                 (((x) &  BIT_MASK_IC_TFE) << BIT_SHIFT_IC_TFE)
#define BIT_GET_IC_TFE(x)                                  (((x) >>  BIT_SHIFT_IC_TFE) & BIT_MASK_IC_TFE)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for TFNF
 *  @{
 */
#define BIT_IC_TFNF                                        BIT1
#define BIT_SHIFT_IC_TFNF                                  1
#define BIT_MASK_IC_TFNF                                   0x1
#define BIT_CTRL_IC_TFNF(x)                                (((x) &  BIT_MASK_IC_TFNF) << BIT_SHIFT_IC_TFNF)
#define BIT_GET_IC_TFNF(x)                                 (((x) >>  BIT_SHIFT_IC_TFNF) & BIT_MASK_IC_TFNF)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for ACTIVITY
 *  @{
 */
#define BIT_ACTIVITY                                       BIT0
#define BIT_SHIFT_ACTIVITY                                 0
#define BIT_MASK_ACTIVITY                                  0x1
#define BIT_CTRL_ACTIVITY(x)                               (((x) &  BIT_MASK_ACTIVITY) << BIT_SHIFT_ACTIVITY)
#define BIT_GET_ACTIVITY(x)                                (((x) >>  BIT_SHIFT_ACTIVITY) & BIT_MASK_ACTIVITY)
/** @} */

/*  Bit shift macros for i2c_dma_cmd_t  */
/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for IC_DMA_RESTART
 *  @{
 */
#define BIT_IC_DMA_RESTART                                 BIT7
#define BIT_SHIFT_IC_DMA_RESTART                           7
#define BIT_MASK_IC_DMA_RESTART                            0x1
#define BIT_CTRL_IC_DMA_RESTART(x)                         (((x) &  BIT_MASK_IC_DMA_RESTART) << BIT_SHIFT_IC_DMA_RESTART)
#define BIT_GET_IC_DMA_RESTART(x)                          (((x) >>  BIT_SHIFT_IC_DMA_RESTART) & BIT_MASK_IC_DMA_RESTART)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for IC_DMA_STOP
 *  @{
 */
#define BIT_IC_DMA_STOP                                    BIT6
#define BIT_SHIFT_IC_DMA_STOP                              6
#define BIT_MASK_IC_DMA_STOP                               0x1
#define BIT_CTRL_IC_DMA_STOP(x)                            (((x) &  BIT_MASK_IC_DMA_STOP) << BIT_SHIFT_IC_DMA_STOP)
#define BIT_GET_IC_DMA_STOP(x)                             (((x) >>  BIT_SHIFT_IC_DMA_STOP) & BIT_MASK_IC_DMA_STOP)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for IC_DMA_RW_CMD
 *  @{
 */
#define BIT_IC_DMA_RW_CMD                                  BIT5
#define BIT_SHIFT_IC_DMA_RW_CMD                            5
#define BIT_MASK_IC_DMA_RW_CMD                             0x1
#define BIT_CTRL_IC_DMA_RW_CMD(x)                          (((x) &  BIT_MASK_IC_DMA_RW_CMD) << BIT_SHIFT_IC_DMA_RW_CMD)
#define BIT_GET_IC_DMA_RW_CMD(x)                           (((x) >>  BIT_SHIFT_IC_DMA_RW_CMD) & BIT_MASK_IC_DMA_RW_CMD)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for IC_DMA_MODE_EN
 *  @{
 */
#define BIT_IC_DMA_MODE_EN                                 BIT0
#define BIT_SHIFT_IC_DMA_MODE_EN                           0
#define BIT_MASK_IC_DMA_MODE_EN                            0x1
#define BIT_CTRL_IC_DMA_MODE_EN(x)                         (((x) &  BIT_MASK_IC_DMA_MODE_EN) << BIT_SHIFT_IC_DMA_MODE_EN)
#define BIT_GET_IC_DMA_MODE_EN(x)                          (((x) >>  BIT_SHIFT_IC_DMA_MODE_EN) & BIT_MASK_IC_DMA_MODE_EN)
/** @} */

/*  Bit shift macros for i2c_tx_abrt_src_t  */
/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for ABRT_SLVRD_INTX
 *  @{
 */
#define BIT_ABRT_SLVRD_INTX                                BIT15
#define BIT_SHIFT_ABRT_SLVRD_INTX                          15
#define BIT_MASK_ABRT_SLVRD_INTX                           0x1
#define BIT_CTRL_ABRT_SLVRD_INTX(x)                        (((x) &  BIT_MASK_ABRT_SLVRD_INTX) << BIT_SHIFT_ABRT_SLVRD_INTX)
#define BIT_GET_ABRT_SLVRD_INTX(x)                         (((x) >>  BIT_SHIFT_ABRT_SLVRD_INTX) & BIT_MASK_ABRT_SLVRD_INTX)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for ABRT_SLV_ARBLOST
 *  @{
 */
#define BIT_ABRT_SLV_ARBLOST                               BIT14
#define BIT_SHIFT_ABRT_SLV_ARBLOST                         14
#define BIT_MASK_ABRT_SLV_ARBLOST                          0x1
#define BIT_CTRL_ABRT_SLV_ARBLOST(x)                       (((x) &  BIT_MASK_ABRT_SLV_ARBLOST) << BIT_SHIFT_ABRT_SLV_ARBLOST)
#define BIT_GET_ABRT_SLV_ARBLOST(x)                        (((x) >>  BIT_SHIFT_ABRT_SLV_ARBLOST) & BIT_MASK_ABRT_SLV_ARBLOST)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for ABRT_SLVFLUSH_TXFIFO
 *  @{
 */
#define BIT_ABRT_SLVFLUSH_TXFIFO                           BIT13
#define BIT_SHIFT_ABRT_SLVFLUSH_TXFIFO                     13
#define BIT_MASK_ABRT_SLVFLUSH_TXFIFO                      0x1
#define BIT_CTRL_ABRT_SLVFLUSH_TXFIFO(x)                   (((x) &  BIT_MASK_ABRT_SLVFLUSH_TXFIFO) << BIT_SHIFT_ABRT_SLVFLUSH_TXFIFO)
#define BIT_GET_ABRT_SLVFLUSH_TXFIFO(x)                    (((x) >>  BIT_SHIFT_ABRT_SLVFLUSH_TXFIFO) & BIT_MASK_ABRT_SLVFLUSH_TXFIFO)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for ARB_LOST
 *  @{
 */
#define BIT_ARB_LOST                                       BIT12
#define BIT_SHIFT_ARB_LOST                                 12
#define BIT_MASK_ARB_LOST                                  0x1
#define BIT_CTRL_ARB_LOST(x)                               (((x) &  BIT_MASK_ARB_LOST) << BIT_SHIFT_ARB_LOST)
#define BIT_GET_ARB_LOST(x)                                (((x) >>  BIT_SHIFT_ARB_LOST) & BIT_MASK_ARB_LOST)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for ABRT_MST_DIS
 *  @{
 */
#define BIT_ABRT_MST_DIS                                    BIT11
#define BIT_SHIFT_ABRT_MST_DIS                              11
#define BIT_MASK_ABRT_MST_DIS                               0x1
#define BIT_CTRL_ABRT_MST_DIS(x)                            (((x) &  BIT_MASK_ABRT_MST_DIS) << BIT_SHIFT_ABRT_MST_DIS)
#define BIT_GET_ABRT_MST_DIS(x)                             (((x) >>  BIT_SHIFT_ABRT_MST_DIS) & BIT_MASK_ABRT_MST_DIS)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for ABRT_NORSTRT_10BIT_RD
 *  @{
 */
#define BIT_ABRT_NORSTRT_10BIT_RD                           BIT10
#define BIT_SHIFT_ABRT_NORSTRT_10BIT_RD                     10
#define BIT_MASK_ABRT_NORSTRT_10BIT_RD                      0x1
#define BIT_CTRL_ABRT_NORSTRT_10BIT_RD(x)                   (((x) &  BIT_MASK_ABRT_NORSTRT_10BIT_RD) << BIT_SHIFT_ABRT_NORSTRT_10BIT_RD)
#define BIT_GET_ABRT_NORSTRT_10BIT_RD(x)                    (((x) >>  BIT_SHIFT_ABRT_NORSTRT_10BIT_RD) & BIT_MASK_ABRT_NORSTRT_10BIT_RD)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for ABRT_NORSTRT_SBYTE
 *  @{
 */
#define BIT_ABRT_NORSTRT_SBYTE                              BIT9
#define BIT_SHIFT_ABRT_NORSTRT_SBYTE                        9
#define BIT_MASK_ABRT_NORSTRT_SBYTE                         0x1
#define BIT_CTRL_ABRT_NORSTRT_SBYTE(x)                      (((x) &  BIT_MASK_ABRT_NORSTRT_SBYTE) << BIT_SHIFT_ABRT_NORSTRT_SBYTE)
#define BIT_GET_ABRT_NORSTRT_SBYTE(x)                       (((x) >>  BIT_SHIFT_ABRT_NORSTRT_SBYTE) & BIT_MASK_ABRT_NORSTRT_SBYTE)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for ABRT_NORSTRT_HS
 *  @{
 */
#define BIT_ABRT_NORSTRT_HS                                 BIT8
#define BIT_SHIFT_ABRT_NORSTRT_HS                           8
#define BIT_MASK_ABRT_NORSTRT_HS                            0x1
#define BIT_CTRL_ABRT_NORSTRT_HS(x)                         (((x) &  BIT_MASK_ABRT_NORSTRT_HS) << BIT_SHIFT_ABRT_NORSTRT_HS)
#define BIT_GET_ABRT_NORSTRT_HS(x)                          (((x) >>  BIT_SHIFT_ABRT_NORSTRT_HS) & BIT_MASK_ABRT_NORSTRT_HS)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for ABRT_SBYTE_ACKDET
 *  @{
 */
#define BIT_ABRT_SBYTE_ACKDET                               BIT7
#define BIT_SHIFT_ABRT_SBYTE_ACKDET                         7
#define BIT_MASK_ABRT_SBYTE_ACKDET                          0x1
#define BIT_CTRL_ABRT_SBYTE_ACKDET(x)                       (((x) &  BIT_MASK_ABRT_SBYTE_ACKDET) << BIT_SHIFT_ABRT_SBYTE_ACKDET)
#define BIT_GET_ABRT_SBYTE_ACKDET(x)                        (((x) >>  BIT_SHIFT_ABRT_SBYTE_ACKDET) & BIT_MASK_ABRT_SBYTE_ACKDET)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for ABRT_HS_ACKDET
 *  @{
 */
#define BIT_ABRT_HS_ACKDET                                  BIT6
#define BIT_SHIFT_ABRT_HS_ACKDET                            6
#define BIT_MASK_ABRT_HS_ACKDET                             0x1
#define BIT_CTRL_ABRT_HS_ACKDET(x)                          (((x) &  BIT_MASK_ABRT_HS_ACKDET) << BIT_SHIFT_ABRT_HS_ACKDET)
#define BIT_GET_ABRT_HS_ACKDET(x)                           (((x) >>  BIT_SHIFT_ABRT_HS_ACKDET) & BIT_MASK_ABRT_HS_ACKDET)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for ABRT_GCALL_RD
 *  @{
 */
#define BIT_ABRT_GCALL_RD                                BIT5
#define BIT_SHIFT_ABRT_GCALL_RD                          5
#define BIT_MASK_ABRT_GCALL_RD                           0x1
#define BIT_CTRL_ABRT_GCALL_RD(x)                        (((x) &  BIT_MASK_ABRT_GCALL_RD) << BIT_SHIFT_ABRT_GCALL_RD)
#define BIT_GET_ABRT_GCALL_RD(x)                         (((x) >>  BIT_SHIFT_ABRT_GCALL_RD) & BIT_MASK_ABRT_GCALL_RD)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for ABRT_GCALL_NOACK
 *  @{
 */
#define BIT_ABRT_GCALL_NOACK                            BIT4
#define BIT_SHIFT_ABRT_GCALL_NACK                       4
#define BIT_MASK_ABRT_GCALL_NACK                        0x1
#define BIT_CTRL_ABRT_GCALL_NACK(x)                     (((x) &  BIT_MASK_ABRT_GCALL_NACK) << BIT_SHIFT_ABRT_GCALL_NACK)
#define BIT_GET_ABRT_GCALL_NACK(x)                      (((x) >>  BIT_SHIFT_ABRT_GCALL_NACK) & BIT_MASK_ABRT_GCALL_NACK)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for ABRT_TXDAT_NACK
 *  @{
 */
#define BIT_ABRT_TXDAT_NACK                             BIT3
#define BIT_SHIFT_ABRT_TXDAT_NACK                       3
#define BIT_MASK_ABRT_TXDAT_NACK                        0x1
#define BIT_CTRL_ABRT_TXDAT_NACK(x)                     (((x) &  BIT_MASK_ABRT_TXDAT_NACK) << BIT_SHIFT_ABRT_TXDAT_NACK)
#define BIT_GET_ABRT_TXDAT_NACK(x)                      (((x) >>  BIT_SHIFT_ABRT_TXDAT_NACK) & BIT_MASK_ABRT_TXDAT_NACK)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for ABRT_ADDR2_10BIT_NACK
 *  @{
 */
#define BIT_ABRT_ADDR2_10BIT_NACK                       BIT2
#define BIT_SHIFT_ABRT_ADDR2_10BIT_NACK                 2
#define BIT_MASK_ABRT_ADDR2_10BIT_NACK                  0x1
#define BIT_CTRL_ABRT_ADDR2_10BIT_NACK(x)               (((x) &  BIT_MASK_ABRT_ADDR2_10BIT_NACK) << BIT_SHIFT_ABRT_ADDR2_10BIT_NACK)
#define BIT_GET_ABRT_ADDR2_10BIT_NACK(x)                (((x) >>  BIT_SHIFT_ABRT_ADDR2_10BIT_NACK) & BIT_MASK_ABRT_ADDR2_10BIT_NACK)
/** @} */

/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for ABRT_ADDR2_10BIT_NACK
 *  @{
 */
#define BIT_ABRT_ADDR1_10BIT_NACK                       BIT2
#define BIT_SHIFT_ABRT_ADDR1_10BIT_NACK                 2
#define BIT_MASK_ABRT_ADDR1_10BIT_NACK                  0x1
#define BIT_CTRL_ABRT_ADDR1_10BIT_NACK(x)               (((x) &  BIT_MASK_ABRT_ADDR1_10BIT_NACK) << BIT_SHIFT_ABRT_ADDR1_10BIT_NACK)
#define BIT_GET_ABRT_ADDR1_10BIT_NACK(x)                (((x) >>  BIT_SHIFT_ABRT_ADDR1_10BIT_NACK) & BIT_MASK_ABRT_ADDR1_10BIT_NACK)
/** @} */


/**
 *  @ingroup i2c_reg_bit_macro
 *  bit-operation macros for ABRT_ADDR_7BIT_NACK
 *  @{
 */
#define BIT_ABRT_ADDR_7BIT_NACK                         BIT0
#define BIT_SHIFT_ABRT_ADDR_7BIT_NACK                   0
#define BIT_MASK_ABRT_ADDR_7BIT_NACK                    0x1
#define BIT_CTRL_ABRT_ADDR_7BIT_NACK(x)                 (((x) &  BIT_MASK_ABRT_ADDR_7BIT_NACK) << BIT_SHIFT_ABRT_ADDR_7BIT_NACK)
#define BIT_GET_ABRT_ADDR_7BIT_NACK(x)                  (((x) >>  BIT_SHIFT_ABRT_ADDR_7BIT_NACK) & BIT_MASK_ABRT_ADDR_7BIT_NACK)
/** @} */


/**
  \brief  Type to access RTL8195B HP I2C function control register fields.
 */
typedef union {
    uint8_t w;
    struct {
        uint32_t en:1;                /*!< bit:   0       i2c enable */
        uint32_t pclk_en:1;           /*!< bit:   1       i2c pclk enable */
        uint32_t sclk_en:1;           /*!< bit:   2       i2c sclk enable */
        uint32_t rsvd:1;              /*!< bit:   3       reserved */
        uint32_t mux_en:1;            /*!< bit:   4       i2c mux enable */
        uint32_t mux_sel:2;           /*!< bit:   5 to 6  i2c mux selection
                                                            b'00: GPIOB 0~1
                                                            b'01: GPIOG 4~5
                                                            b'10: GPIOH 2~3
                                                            b'11: rsvd*/
    } b;
} hp_i2c_func_ctrl_entry_t, *php_i2c_func_ctrl_entry_t;

/**
  \brief  Type to access RTL8195B HP I2C function control register.
 */
typedef union {
    uint32_t w;                         /*!< Type      used for word access */
    struct {
        uint32_t en_0:1;                /*!< bit:   0       i2c0 enable */
        uint32_t pclk_en_0:1;           /*!< bit:   1       i2c0 pclk enable */
        uint32_t sclk_en_0:1;           /*!< bit:   2       i2c0 sclk enable */
        uint32_t rsvd0:1;               /*!< bit:   3       reserved */
        uint32_t mux_en_0:1;            /*!< bit:   4       i2c0 mux enable */
        uint32_t mux_sel_0:2;           /*!< bit:   5 to 6  i2c0 mux selection
                                                            b'00: GPIOB 0~1
                                                            b'01: GPIOG 4~5
                                                            b'10: GPIOH 2~3
                                                            b'11: rsvd*/
        uint32_t rsvd1:9;               /*!< bit:   7 to 15 reserved */

        uint32_t en_2:1;                /*!< bit:   16      i2c2 enable */
        uint32_t pclk_en_2:1;           /*!< bit:   17      i2c2 pclk enable */
        uint32_t sclk_en_2:1;           /*!< bit:   18      i2c2 sclk enable */
        uint32_t rsvd2:2;               /*!< bit:   19      reserved */
        uint32_t mux_en_2:1;            /*!< bit:   20      i2c2 mux enable */
        uint32_t mux_sel_2:2;           /*!< bit:   21 to 22 i2c2 mux selection
                                                            b'00: GPIOB 2~3
                                                            b'01: GPIOG 6~7
                                                            b'10: GPIOE 14~15
                                                            b'11: rsvd*/
        uint32_t rsvd3:1;               /*!< bit:   23      reserved */

        uint32_t en_3:1;                /*!< bit:   24      i2c3 enable */
        uint32_t pclk_en_3:1;           /*!< bit:   25      i2c3 pclk enable */
        uint32_t sclk_en_3:1;           /*!< bit:   26      i2c3 sclk enable */
        uint32_t rsvd4:1;               /*!< bit:   27      reserved */
        uint32_t mux_en_3:1;            /*!< bit:   28      i2c3 mux enable */
        uint32_t mux_sel_3:2;           /*!< bit:   29 to 30 i2c3 mux selection
                                                            b'00: GPIOB 4~5
                                                            b'01: GPIOG 8~9
                                                            b'10: GPIOF 10, 4
                                                            b'11: rsvd*/
    } b;                                /*!< Structure used for bit  access */
} hp_i2c_func_ctrl_t, *php_i2c_func_ctrl_t;


/** \brief i2c mode
*/
enum i2c_mode_e {
    I2CSlaveMode    =   0,                  /*!< 0: for i2c slave mode    */
    I2CMasterMode   =   1,                  /*!< 1: for i2c master mode    */
};

/** \brief i2c read/write
*/
enum i2c_read_write_e {
    I2CWrite        =   0,                  /*!< 0: for i2c write command    */
    I2CRead         =   1,                  /*!< 1: for i2c read command    */
};

/** \brief i2c address mode
*/
enum i2c_addr_mode_e {
    I2CAddress7bit     =   0,               /*!< 0: for i2c 7-bit address mode    */
    I2CAddress10bit    =   1,               /*!< 1: for i2c 10-bit address mode    */
};

/** \brief i2c speed mode
*/
enum i2c_spd_mode_e {
    I2CStandardSpeed    =   1,              /*!< 1: for i2c standard speed mode    */
    I2CFastSpeed        =   2,              /*!< 2: for i2c fast speed mode    */
    I2CHighSpeed        =   3,              /*!< 3: for i2c high speed mode    */
};

/** \brief i2c feature status
*/
enum i2c_enable_status_e {
    I2CDisable      =   0,                  /*!< 0: for i2c disable state    */
    I2CEnable       =   1,                  /*!< 1: for i2c enable state    */
    I2CForceDisable =   2,                  /*!< 2: for i2c force disable enable state    */
};

/** \brief i2c dma type
*/
enum i2c_dma_mod_e {
    I2CDmaDwc   =   0,                      /*!< 0: for i2c dwc dma mode    */
    I2CDmaReg   =   1,                      /*!< 1: for i2c register-based dma mode    */
    I2CDmaDes   =   2,                      /*!< 2: for i2c descritptor mode    */
};

/** \brief i2c general call or start byte
*/
enum i2c_gc_sb_e {
    I2CGeneralCall      =   0,                      /*!< 0: for i2c master could general call    */
    I2CStartByte        =   1,                      /*!< 1: for i2c master could START byte    */
};

/** \brief i2c bus status
*/
enum i2c_bus_status_e {
    I2CBusIdle          =   0,                      /*!< 0: for i2c bus in idle state    */
    I2CBusAddressPhase  =   1,                      /*!< 1: for i2c bus in address phase    */
    I2CBusDataPhase     =   2,                      /*!< 2: for i2c bus in data phase    */
    I2CBusClockStretch  =   3,                      /*!< 2: for i2c bus in clock stretch phase    */
};

/** \brief i2c module state
*/
enum i2c_status_e {
    I2CStatusUninitial      =   0x00,               /*!< 0x00: i2c uninitial state   */
    I2CStatusInitialized    =   0x01,               /*!< 0x01: i2c initialized state   */
    I2CStatusIdle           =   0x02,               /*!< 0x02: i2c idle state   */

    I2CStatusTxReady        =   0x03,               /*!< 0x03: i2c TX ready state   */
    I2CStatusTxing          =   0x04,               /*!< 0x04: i2c TXing state   */

    I2CStatusRxReady        =   0x05,               /*!< 0x05: i2c RX ready state   */
    I2CStatusRxing          =   0x06,               /*!< 0x06: i2c RXing state   */

    I2CStatusDisable        =   0x07,               /*!< 0x07: i2c disabled state   */
    I2CStatusError          =   0x10,               /*!< 0x10: i2c error state   */
    I2CStatusTimeOut        =   0x11,               /*!< 0x11: i2c time-out state   */
};

/** \brief i2c module operation
*/
enum i2c_operation_mode_e {
    I2CModePoll         =   0x00,                   /*!< 0x00: i2c module in poll mode   */
    I2CModeInterrupt    =   0x01,                   /*!< 0x01: i2c module in interrupt mode   */
    I2CModeDMA          =   0x02,                   /*!< 0x02: i2c module in DMA mode   */
};

/** \brief i2c module master special functions
*/
enum i2c_master_speicial_func_e {
    I2CAddressRetry     =   0x01,                   /*!< 0x01: i2c master mode could do address retry   */
    I2CMasterRestart    =   0x02,                   /*!< 0x02: i2c master mode could send RESTART   */
};

/** \brief i2c module slave special functions
*/
enum i2c_slave_speicial_func_e {
    I2CSlaveRXBySTPSTR =   0x01,                    /*!< 0x01: i2c slave mode receiving would end by STOP or START bit*/
};

/** \brief i2c module error type
*/
enum i2c_err_type_e {
    I2CErrorNone            =   0x00,               /*!< 0x00: i2c has no errors    */
    I2CErrorRxUnder         =   0x01,               /*!< 0x01: i2c has RX underflow error    */
    I2CErrorRxOver          =   0x02,               /*!< 0x02: i2c has RX overflow error    */
    I2CErrorRxDone          =   0x03,               /*!< 0x03: i2c has RX Done error    */
    I2CErrorTxOver          =   0x04,               /*!< 0x04: i2c has TX overflow error    */
    I2CErrorTxAbort         =   0x08,               /*!< 0x08: i2c has TX abort error    */
    I2CErrorSlaveTxNack     =   0x10,               /*!< 0x10: i2c has slave TX NACK error    */

    I2CErrorMasterAddrNack  =   0x12,               /*!< 0x12: i2c master sends address and gets NACK    */
    I2CErrorMasterDataNack  =   0x13,               /*!< 0x13: i2c master sends data and gets NACK    */
    I2CErrorMasterLostArb   =   0x14,               /*!< 0x14: i2c master lost bus arbitration    */

    I2CErrorRxCmdTimeOut    =   0x21,               /*!< 0x21: i2c master sends RX command and time-out happens    */
    I2CErrorRxFIFOTimeOut   =   0x22,               /*!< 0x22: i2c read RX data and time-out happens    */
    I2CErrorTxCmdTimeOut    =   0x23,               /*!< 0x23: i2c master sends TX command and time-out happens    */
    I2CErrorTxFIFOTimeOut   =   0x24,               /*!< 0x24: i2c fill TX data and time-out happens    */

    I2CErrorTxAddrTimeOut   =   0x25,               /*!< 0x25: i2c master sends TX address retry and time-out happens    */
    I2CErrorRxAddrTimeOut   =   0x26,               /*!< 0x26: i2c master sends RX address retry and time-out happens    */
    I2CErrorTarTimeOut      =   0x27,               /*!< 0x27: i2c master sets target address and time-out happens    */

    I2CErrorSlaveLostArb    =   0x30,               /*!< 0x30: i2c slave lost bus arbitration    */
    I2CErrorSlaveFlushFIFO  =   0x31,               /*!< 0x31: i2c slave flushes FIFO when getting RD Req. and TX FIFO
                                                               is NOT empty. */
    I2CErrorSlaveRDCmdInTX  =   0x32,               /*!< 0x32: i2c slave acts as a transmitter and a Read Cmd written
                                                               into TX FIFO */
};

/** \brief i2c dma channel status
*/
enum i2c_dma_ch_status_e {
    I2CDmaChNone            =   0x00,               /*!< 0x00: i2c got no channel    */
    I2CDmaChGot             =   0x01,               /*!< 0x00: i2c got a channel    */
    I2CDmaChInitialed       =   0x02,               /*!< 0x00: i2c got a channel and initialized it    */
};

/*  Macros for i2c SCL timing constant  */
/** @defgroup i2c_reg_scl_clock_constant I2C REG SCL CLOCK CONSTANT
 *  i2c SCL timing constant
 *  @{
 */
#define I2C_SS_MIN_SCL_HTIME    4000        //the unit is ns.
#define I2C_SS_MIN_SCL_LTIME    4700        //the unit is ns.

#define I2C_FS_MODIFY
#ifdef I2C_FS_MODIFY
#define I2C_FS_MIN_SCL_HTIME    900         //the unit is ns.
#define I2C_FS_MIN_SCL_LTIME    1000        //the unit is ns.
#else
#define I2C_FS_MIN_SCL_HTIME    600         //the unit is ns.
#define I2C_FS_MIN_SCL_LTIME    1300        //the unit is ns.
#endif

#define I2C_HS_MIN_SCL_HTIME_100    60      //the unit is ns, with bus loading = 100pf
#define I2C_HS_MIN_SCL_LTIME_100    120     //the unit is ns., with bus loading = 100pf

#define I2C_HS_MIN_SCL_HTIME_400    160     //the unit is ns, with bus loading = 400pf
#define I2C_HS_MIN_SCL_LTIME_400    320     //the unit is ns., with bus loading = 400pf

#define I2C_SS_HTIME_MIN        3
#define I2C_SS_LTIME_MIN        5

#define I2C_FS_HTIME_MIN        3
#define I2C_FS_LTIME_MIN        5

#define I2C_HS_HTIME_MIN        3
#define I2C_HS_LTIME_MIN        5

#define I2C_MST_SDA_HOLD_MIN    1
#define I2C_SLV_SDA_HOLD_MIN    7
/** @} */ // end of I2C REG SCL CLOCK CONSTANT

/** \brief i2c initail data structure */
typedef struct i2c_init_dat_s {
    uint8_t     index;                          /*!< Offset: 0x000   i2c index   */
    uint8_t     enable;                         /*!< Offset: 0x001   i2c enable state, updated by HAL   */
    uint8_t     master;                         /*!< Offset: 0x002   i2c master/slave state   */
    uint8_t     addr_mod;                       /*!< Offset: 0x003   i2c address mode   */

    uint8_t     spd_mod;                        /*!< Offset: 0x004   i2c speed mode   */
    uint8_t     sda_setup;                      /*!< Offset: 0x005   i2c SDA setup time   */
    uint8_t     ff_rxtl;                        /*!< Offset: 0x006   i2c RX FIFO Threshold, for RX_FULL state  */
    uint8_t     ff_txtl;                        /*!< Offset: 0x007   i2c TX FIFO Threshold, for TX_EMPTY state   */

    uint32_t    clock;                          /*!< Offset: 0x008   i2c SCL clock   */

    uint16_t    ack_addr0;                      /*!< Offset: 0x00c   i2c default target address in master mode,
                                                                 acknowledge address 0 in slave mode    */
    uint16_t    ack_addr1;                      /*!< Offset: 0x00e   i2c acknowledge address 1 in slave mode(only for slave)
                                                                 */

    uint16_t    sda_hold;                       /*!< Offset: 0x010   i2c SDA hold time   */
    uint8_t     bus_ld;                         /*!< Offset: 0x012   i2c bus loading   */
    uint8_t     dig_fltr_en;	                /*!< Offset: 0x013   i2c digital filter control   */

    uint8_t     dma_mod;                        /*!< Offset: 0x014   i2c DMA operation mode   */
    uint8_t     tx_dma_rq_lv;                   /*!< Offset: 0x015   i2c DMA TX request FIFO level   */
    uint8_t     rx_dma_rq_lv;                   /*!< Offset: 0x016   i2c DMA RX request FIFO level   */
    uint8_t     rx_dma_rq_lv_s1;                /*!< Offset: 0x017   i2c DMA RX request FIFO level for slave address 1  */

    I2C0_Type   *reg_base;                      /*!< Offset: 0x018   i2c register base address  */
    uint8_t     dig_fltr_lvl;                   /*!< Offset: 0x01C   i2c digital filter level  */
    uint8_t     hs_maddr;                       /*!< Offset: 0x01D   i2c high speed master code, only lsb 3 bits valid  */
    uint16_t    rsvd1;                          /*!< Offset: 0x01E   reserved 1  */
}i2c_init_dat_t, *pi2c_init_dat_t;

/** \brief i2c user callback adaptor
*/
typedef struct i2c_user_callback_adpt_s {
    VOID (*cb)      (VOID *data);                   /*! i2c callback function */
    uint32_t    dat;                                /*! i2c callback function argument */
}i2c_user_callback_adpt_t, *pi2c_user_callback_adpt_t;

/** \brief i2c user callback
*/
typedef struct i2c_user_callback_s {
    i2c_user_callback_adpt_t  tx_empty;             /*! i2c transmit callback */
    i2c_user_callback_adpt_t  txc;                  /*! i2c transmit complete callback */
    i2c_user_callback_adpt_t  rx_full;              /*! i2c receive callback */
    i2c_user_callback_adpt_t  rxc;                  /*! i2c receive complete callback */
    i2c_user_callback_adpt_t  rd_req;               /*! i2c read request callback */
    i2c_user_callback_adpt_t  err;                  /*! i2c error callback */
    i2c_user_callback_adpt_t  dma_tx;               /*! i2c dma transmit callback */
    i2c_user_callback_adpt_t  dma_txc;              /*! i2c dma transmit complete callback */
    i2c_user_callback_adpt_t  dma_rx;               /*! i2c dma receive callback */
    i2c_user_callback_adpt_t  dma_rxc;              /*! i2c dma receive complete callback */
    i2c_user_callback_adpt_t  gen_call;             /*! i2c general call callback */
    i2c_user_callback_adpt_t  wake;                 /*! i2c wake up callback */
}i2c_user_callback_t, *pi2c_user_callback_t;

/** \brief i2c platform related data
*/
typedef struct i2c_platform_data_s {
    uint8_t         scl_pin;                        /*! i2c scl pin */
    uint8_t         sda_pin;                        /*! i2c scl pin */
    uint8_t         tx_dma_bound;                   /*! i2c low bound to do tx-dma transmission,
                                                        if tx len is less than this bound, tx API
                                                        would change to poll mode transmission.*/
    uint8_t         rx_dma_bound;                   /*! i2c low bound to do rx-dma transmission,
                                                        if tx len is less than this bound, tx API
                                                        would change to poll mode transmission.*/
    uint32_t        tr_time_out;                    /*! i2c transmission time-out count */
}i2c_platform_data_t,*pi2c_platform_data_t;

/** \brief i2c transmit information (master/slave)
*/
typedef struct i2c_tx_info_s {
    uint16_t        addr;                           /*! i2c target address for master mode    */
    uint8_t         mst_stop;                       /*! i2c stop control for master mode    */
    uint8_t         rsvd0;                          /*! reserved 0   */
    const uint8_t *buf;                            /*! i2c trassmit data buffer pointer    */
    uint32_t        len;                            /*! i2c transmit data length    */
}i2c_tx_info_t,*pi2c_tx_info_t;

/** \brief i2c receive information (master/slave)
*/
typedef struct i2c_rx_info_s {
    uint16_t        addr;                           /*! i2c target address for master mode    */
    uint8_t         mst_stop;                       /*! i2c stop control for master mode    */
    uint8_t         rsvd0;                          /*! reserved 0   */
    uint8_t *buf;                            /*! i2c receive data buffer pointer    */
    uint32_t        len;                            /*! i2c receive data length    */
}i2c_rx_info_t,*pi2c_rx_info_t;

/** \brief i2c dma information (master/slave)
*/
typedef struct i2c_dma_info_s {
    hal_gdma_adaptor_t  *padaptor;                  /*! i2c dma adaptor     */
    irq_config_t        irq_config;                 /*! i2c dma interrupt handler     */
    uint8_t             ch_sts;                     /*! i2c dma channel status (i2c local)     */
    uint32_t            dat_len;                    /*! i2c original dma data length (i2c local)     */
}i2c_dma_info_t,*pi2c_dma_info_t;

/** \brief i2c control adapter
*/
typedef struct hal_i2c_adapter_s {
    volatile uint8_t    status;                     /*! i2c module status    */
    uint8_t             op_mode;                    /*! i2c module operation mode    */
    uint8_t             mst_spe_func;               /*! i2c module master special function    */
    uint8_t             slv_spe_func;               /*! i2c module slave special function    */
    volatile uint32_t   err_type;                   /*! i2c module error type    */
    i2c_init_dat_t      init_dat;                   /*! i2c initial data     */
    i2c_platform_data_t pltf_dat;                   /*! i2c platform data     */
    i2c_user_callback_t usr_cb;                     /*! i2c user callback     */
    irq_config_t        irq_config;                 /*! i2c interrupt handler     */
    i2c_tx_info_t       tx_dat;                     /*! i2c transmit information     */
    i2c_rx_info_t       rx_dat;                     /*! i2c receive information     */
    uint32_t            rd_cmd_no;                  /*! i2c read command number (master only)     */
    i2c_dma_info_t      tx_dma_dat;                 /*! i2c transmit dma data   */
    i2c_dma_info_t      rx_dma_dat;                 /*! i2c receive dma data   */
    void (*dcache_invalidate_by_addr)(uint32_t *addr, int32_t dsize);   /*! callback function to do the D-cache invalidate  */
    void (*dcache_clean_by_addr) (uint32_t *addr, int32_t dsize);       /*! callback function to do the D-cache clean  */
    void (*dcache_clean_invalidate_by_addr) (uint32_t *addr, int32_t dsize);       /*! callback function to do the D-cache clean
                                                                                       and invalidate */
    uint32_t            rsvd0;                      /*! reserved 0    */
}hal_i2c_adapter_t, *phal_i2c_adapter_t;

/** \brief Description of hal_i2c_write32
 *
 *    hal_i2c_write32 is used for general register write operation.
 *
 *   \param[in] uint32_t index:     i2c index
 *   \param[in] uint32_t reg:       register offset
 *   \param[in] uint32_t val:       register value
 */
static inline void hal_i2c_write32(uint8_t index, uint32_t reg, uint32_t val)
{
    switch (index) {
        case 0:
            HAL_WRITE32(I2C0_BASE, reg, val);
            break;

        default:
            break;
    }
}

/** \brief Description of hal_i2c_read32
 *
 *    hal_i2c_read32 is used for general register read operation.
 *
 *   \param[in] uint32_t index:     i2c index
 *   \param[in] uint32_t reg:       register offset
 *   \return uint32_t:          register value in 32-bit
 */
static inline uint32_t hal_i2c_read32(uint8_t index, uint32_t reg)
{
    switch (index) {
        case 0:
            return (uint32_t)(HAL_READ32(I2C0_BASE, reg));

        default:
            break;
    }

    return (uint32_t)0xFFFFFFFF;
}

/** \brief Description of hal_i2c_idx_chk_rtl8710c
 *
 *    hal_i2c_idx_chk_rtl8710c is used for checking i2c module is available or not
 *
 *   \param[in] uint8_t idx:     I2C index
 *   \return HAL_Status.
 */
static inline hal_status_t hal_i2c_idx_chk_rtl8710c (uint8_t idx)
{
    if ((idx == 1) || (idx > 3)) {
        DBG_I2C_ERR("Index(%d) is out of range", idx);
        return HAL_ERR_PARA;
    }

    return HAL_OK;
}

/** \brief Description of hal_i2c_slv_set_for_mst_rd_cmd_rtl8710c
 *
 *    hal_i2c_slv_set_for_mst_rd_cmd_rtl8710c is to set interrupt mask for Read Request interrupt which is from
 *    other masters.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:   pointer to I2C control adapter.
 *   \return void
 */
static inline void hal_i2c_slv_set_for_mst_rd_cmd_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter)
{
    phal_i2c_adapter->init_dat.reg_base->intr_msk_b.rd_req = I2CEnable;
}

/** \brief Description of hal_i2c_slv_clear_for_mst_rd_cmd_rtl8710c
 *
 *    hal_i2c_slv_clear_for_mst_rd_cmd_rtl8710c is to clear interrupt mask for Read Request interrupt which is from
 *    other masters.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:   pointer to I2C control adapter.
 *   \return void
 */
static inline void hal_i2c_slv_clear_for_mst_rd_cmd_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter)
{
    phal_i2c_adapter->init_dat.reg_base->intr_msk_b.rd_req = I2CDisable;
}

typedef struct hal_i2c_group_adapter_s {
    hal_i2c_adapter_t *adapter[I2C_NO];     /*!< All the I2C adapters of this platform */
    irq_handler_t   irq_fun[I2C_NO];     /*!< the IRQ handler for different I2C adapters */
} hal_i2c_group_adapter_t, *phal_i2c_group_adapter_t;

/*  Macros for i2c essential operation  */
/** @defgroup i2c_reg_access_macro I2C REG ACCESS MACRO
 *  i2c register access macro
 *  @{
 */
#define HAL_I2C_WRITE32(idx, addr, value)    hal_i2c_write32(idx, (uint32_t)addr,value)
#define HAL_I2C_READ32(idx, addr)            hal_i2c_read32(idx, (uint32_t)addr)
#define READ_CLEAR_I2C_REG(base, reg_name)   base->reg_name
#define I2C_BASE_REG        ((I2C0_Type *) 0)
/** @} */ // end of I2C REG ACCESS MACRO

uint8_t hal_i2c_timeout_chk_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter, uint32_t start_cnt);
void hal_i2c_dma_irq_handler_rtl8710c (void *data);
void hal_i2c_mst_irq_handler_rtl8710c (void *data);
void hal_i2c_slv_irq_handler_rtl8710c (void *data);
uint8_t hal_i2c_chk_mod_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter);
uint8_t hal_i2c_pure_init_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter);
uint8_t hal_i2c_pure_deinit_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter);
uint8_t hal_i2c_en_ctrl_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter, uint8_t enable);
hal_status_t hal_i2c_set_clk_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter);
hal_status_t hal_i2c_intr_ctrl_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter, uint8_t set_clr, uint16_t intr_msk);
hal_status_t hal_i2c_wr_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter, const uint8_t *dat_buf, uint32_t dat_len, uint8_t ctrl);
void hal_i2c_mst_send_rdcmd_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter, uint32_t cmd_len, uint8_t ctrl);
uint32_t hal_i2c_dma_ctrl_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter, uint8_t enable);
hal_status_t hal_i2c_mst_restr_ctrl_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter, uint8_t restr_en);
hal_status_t hal_i2c_mst_gc_sb_ctrl_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter, uint8_t enable, uint8_t gc_sb);
hal_status_t hal_i2c_slv_no_ack_ctrl_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter, uint8_t no_ack_en);
uint8_t hal_i2c_slv_no_ack_sts_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter);
hal_status_t hal_i2c_slv_ack_gc_ctrl_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter, uint8_t slv_gc_en);
uint8_t hal_i2c_slv_ack_gc_sts_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter);
hal_status_t hal_i2c_slv_to_slp_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter);
uint8_t hal_i2c_slv_chk_mst_wr_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter);
uint8_t hal_i2c_slv_chk_rd_req_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter);
hal_status_t hal_i2c_power_init_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter);
hal_status_t hal_i2c_power_deinit_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter);
void hal_i2c_reg_comm_irq_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter, irq_handler_t handler);
void hal_i2c_reg_irq_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter, irq_handler_t handler);
uint8_t hal_i2c_unreg_irq_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter);
hal_status_t hal_i2c_init_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter, uint8_t scl_pin, uint8_t sda_pin);
hal_status_t hal_i2c_deinit_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter);
hal_status_t hal_i2c_load_default_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter, uint8_t index);
hal_status_t hal_i2c_set_tar_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter, uint8_t mst_rw);
hal_status_t hal_i2c_send_addr_retry_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter);
hal_status_t hal_i2c_send_poll_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter);
hal_status_t hal_i2c_send_intr_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter);
//hal_status_t hal_i2c_send_dma_init_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter);
//hal_status_t hal_i2c_send_dma_deinit_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter);
//hal_status_t hal_i2c_send_dma_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter);
//hal_status_t hal_i2c_send_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter);
hal_status_t hal_i2c_recv_addr_retry_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter);
hal_status_t hal_i2c_recv_poll_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter);
hal_status_t hal_i2c_recv_intr_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter);
//hal_status_t hal_i2c_recv_dma_init_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter);
//hal_status_t hal_i2c_recv_dma_deinit_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter);
//hal_status_t hal_i2c_recv_dma_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter);
//hal_status_t hal_i2c_receive_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter);
hal_status_t hal_i2c_set_sar_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter, uint8_t sar_idx, uint16_t slv_addr);
uint32_t hal_i2c_slv_recv_poll_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter);
uint32_t hal_i2c_slv_recv_intr_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter);
//uint32_t hal_i2c_slv_recv_dma_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter);
//hal_status_t hal_i2c_slv_recv_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter);
uint32_t hal_i2c_slv_send_poll_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter);
uint32_t hal_i2c_slv_send_intr_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter);
//uint32_t hal_i2c_slv_send_dma_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter);
//hal_status_t hal_i2c_slv_send_rtl8710c (hal_i2c_adapter_t *phal_i2c_adapter);

/**
  \brief  The data structure of the stubs function for the I2C HAL functions in ROM
*/
typedef struct hal_i2c_func_stubs_s {
        hal_pin_map *hal_i2c_scl_pin_map;
        hal_pin_map *hal_i2c_sda_pin_map;
        hal_i2c_group_adapter_t *hal_i2c_group_adpt;
        uint8_t (*hal_i2c_timeout_chk) (hal_i2c_adapter_t *phal_i2c_adapter, uint32_t start_cnt);
        uint8_t (*hal_i2c_chk_mod) (hal_i2c_adapter_t *phal_i2c_adapter);
        uint8_t (*hal_i2c_pure_init) (hal_i2c_adapter_t *phal_i2c_adapter);
        uint8_t (*hal_i2c_pure_deinit) (hal_i2c_adapter_t *phal_i2c_adapter);
        uint8_t (*hal_i2c_en_ctrl) (hal_i2c_adapter_t *phal_i2c_adapter, uint8_t enable);
        hal_status_t (*hal_i2c_set_clk) (hal_i2c_adapter_t *phal_i2c_adapter);
        hal_status_t (*hal_i2c_intr_ctrl) (hal_i2c_adapter_t *phal_i2c_adapter, uint8_t set_clr, uint16_t intr_msk);
        hal_status_t (*hal_i2c_wr) (hal_i2c_adapter_t *phal_i2c_adapter, const uint8_t *dat_buf, uint32_t dat_len, uint8_t ctrl);
        void (*hal_i2c_mst_send_rdcmd) (hal_i2c_adapter_t *phal_i2c_adapter, uint32_t cmd_len, uint8_t ctrl);
        uint32_t (*hal_i2c_dma_ctrl) (hal_i2c_adapter_t *phal_i2c_adapter, uint8_t enable);
        hal_status_t (*hal_i2c_mst_restr_ctrl) (hal_i2c_adapter_t *phal_i2c_adapter, uint8_t restr_en);
        hal_status_t (*hal_i2c_mst_gc_sb_ctrl)(hal_i2c_adapter_t *phal_i2c_adapter, uint8_t enable, uint8_t gc_sb);
        hal_status_t (*hal_i2c_slv_no_ack_ctrl) (hal_i2c_adapter_t *phal_i2c_adapter, uint8_t no_ack_en);
        uint8_t (*hal_i2c_slv_no_ack_sts) (hal_i2c_adapter_t *phal_i2c_adapter);
        hal_status_t (*hal_i2c_slv_ack_gc_ctrl) (hal_i2c_adapter_t *phal_i2c_adapter, uint8_t slv_gc_en);
        uint8_t (*hal_i2c_slv_ack_gc_sts) (hal_i2c_adapter_t *phal_i2c_adapter);
        hal_status_t (*hal_i2c_slv_to_slp) (hal_i2c_adapter_t *phal_i2c_adapter);
        uint8_t (*hal_i2c_slv_chk_mst_wr) (hal_i2c_adapter_t *phal_i2c_adapter);
        uint8_t (*hal_i2c_slv_chk_rd_req) (hal_i2c_adapter_t *phal_i2c_adapter);
        hal_status_t (*hal_i2c_power_init) (hal_i2c_adapter_t *phal_i2c_adapter);
        hal_status_t (*hal_i2c_power_deinit) (hal_i2c_adapter_t *phal_i2c_adapter);
        void (*hal_i2c_reg_comm_irq) (hal_i2c_adapter_t *phal_i2c_adapter, irq_handler_t handler);
        void (*hal_i2c_reg_irq) (hal_i2c_adapter_t *phal_i2c_adapter, irq_handler_t handler);
        uint8_t (*hal_i2c_unreg_irq) (hal_i2c_adapter_t *phal_i2c_adapter);

        hal_status_t (*hal_i2c_init) (hal_i2c_adapter_t *phal_i2c_adapter, uint8_t scl_pin, uint8_t sda_pin);
        hal_status_t (*hal_i2c_deinit) (hal_i2c_adapter_t *phal_i2c_adapter);
        hal_status_t (*hal_i2c_load_default) (hal_i2c_adapter_t *phal_i2c_adapter, uint8_t index);
        hal_status_t (*hal_i2c_set_tar) (hal_i2c_adapter_t *phal_i2c_adapter, uint8_t mst_rw);
        hal_status_t (*hal_i2c_send_addr_retry) (hal_i2c_adapter_t *phal_i2c_adapter);
        hal_status_t (*hal_i2c_send_poll) (hal_i2c_adapter_t *phal_i2c_adapter);
        hal_status_t (*hal_i2c_send_intr) (hal_i2c_adapter_t *phal_i2c_adapter);
        //hal_status_t (*hal_i2c_send_dma_init) (hal_i2c_adapter_t *phal_i2c_adapter);
        //hal_status_t (*hal_i2c_send_dma_deinit) (hal_i2c_adapter_t *phal_i2c_adapter);
        //hal_status_t (*hal_i2c_send_dma) (hal_i2c_adapter_t *phal_i2c_adapter);
        //hal_status_t (*hal_i2c_send) (hal_i2c_adapter_t *phal_i2c_adapter);
        hal_status_t (*hal_i2c_recv_addr_retry) (hal_i2c_adapter_t *phal_i2c_adapter);
        hal_status_t (*hal_i2c_recv_poll) (hal_i2c_adapter_t *phal_i2c_adapter);
        hal_status_t (*hal_i2c_recv_intr) (hal_i2c_adapter_t *phal_i2c_adapter);
        //hal_status_t (*hal_i2c_recv_dma_init) (hal_i2c_adapter_t *phal_i2c_adapter);
        //hal_status_t (*hal_i2c_recv_dma_deinit) (hal_i2c_adapter_t *phal_i2c_adapter);
        //hal_status_t (*hal_i2c_recv_dma) (hal_i2c_adapter_t *phal_i2c_adapter);
        //hal_status_t (*hal_i2c_receive) (hal_i2c_adapter_t *phal_i2c_adapter);
        hal_status_t (*hal_i2c_set_sar) (hal_i2c_adapter_t *phal_i2c_adapter, uint8_t sar_idx, uint16_t slv_addr);
        uint32_t (*hal_i2c_slv_recv_poll) (hal_i2c_adapter_t *phal_i2c_adapter);
        uint32_t (*hal_i2c_slv_recv_intr) (hal_i2c_adapter_t *phal_i2c_adapter);
        //uint32_t (*hal_i2c_slv_recv_dma) (hal_i2c_adapter_t *phal_i2c_adapter);
        //hal_status_t (*hal_i2c_slv_recv) (hal_i2c_adapter_t *phal_i2c_adapter);
        uint32_t (*hal_i2c_slv_send_poll) (hal_i2c_adapter_t *phal_i2c_adapter);
        uint32_t (*hal_i2c_slv_send_intr) (hal_i2c_adapter_t *phal_i2c_adapter);
        //uint32_t (*hal_i2c_slv_send_dma) (hal_i2c_adapter_t *phal_i2c_adapter);
        //hal_status_t (*hal_i2c_slv_send) (hal_i2c_adapter_t *phal_i2c_adapter);
        void (*hal_i2c_dma_irq_handler) (void *data);
        void (*hal_i2c_mst_irq_handler) (void *data);
        void (*hal_i2c_slv_irq_handler) (void *data);
        uint32_t reserved[16];  // reserved space for next ROM code version function table extending.
} hal_i2c_func_stubs_t;

/** @} */ /* End of group hs_hal_i2c */

#ifdef __cplusplus
}
#endif

#endif /* _RTL8710C_I2C_H_ */

