/****************************************************************************
 * arch/arm/src/s32k3xx/hardware/s32k3xx_lpuart.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/* Copyright 2022 NXP */

#ifndef __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_LPUART_H
#define __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_LPUART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/s32k3xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LPUART Register Offsets **************************************************/

#define S32K3XX_LPUART_VERID_OFFSET    (0x00) /* Version ID Register (VERID) */
#define S32K3XX_LPUART_PARAM_OFFSET    (0x04) /* Parameter Register (PARAM) */
#define S32K3XX_LPUART_GLOBAL_OFFSET   (0x08) /* LPUART Global Register (GLOBAL) */
#define S32K3XX_LPUART_PINCFG_OFFSET   (0x0c) /* LPUART Pin Configuration Register (PINCFG) */
#define S32K3XX_LPUART_BAUD_OFFSET     (0x10) /* LPUART Baud Rate Register (BAUD) */
#define S32K3XX_LPUART_STAT_OFFSET     (0x14) /* LPUART Status Register (STAT) */
#define S32K3XX_LPUART_CTRL_OFFSET     (0x18) /* LPUART Control Register (CTRL) */
#define S32K3XX_LPUART_DATA_OFFSET     (0x1c) /* LPUART Data Register (DATA) */
#define S32K3XX_LPUART_MATCH_OFFSET    (0x20) /* LPUART Match Address Register (MATCH) */
#define S32K3XX_LPUART_MODIR_OFFSET    (0x24) /* LPUART Modem IrDA Register (MODIR) */
#define S32K3XX_LPUART_FIFO_OFFSET     (0x28) /* LPUART FIFO Register (FIFO) */
#define S32K3XX_LPUART_WATER_OFFSET    (0x2c) /* LPUART Watermark Register (WATER) */
#define S32K3XX_LPUART_DATARO_OFFSET   (0x30) /* Data read-only Register (DATARO) */

/* LPUART Register Addresses ************************************************/

#define S32K3XX_LPUART0_VERID          (S32K3XX_LPUART0_BASE + S32K3XX_LPUART_VERID_OFFSET)
#define S32K3XX_LPUART0_PARAM          (S32K3XX_LPUART0_BASE + S32K3XX_LPUART_PARAM_OFFSET)
#define S32K3XX_LPUART0_GLOBAL         (S32K3XX_LPUART0_BASE + S32K3XX_LPUART_GLOBAL_OFFSET)
#define S32K3XX_LPUART0_PINCFG         (S32K3XX_LPUART0_BASE + S32K3XX_LPUART_PINCFG_OFFSET)
#define S32K3XX_LPUART0_BAUD           (S32K3XX_LPUART0_BASE + S32K3XX_LPUART_BAUD_OFFSET)
#define S32K3XX_LPUART0_STAT           (S32K3XX_LPUART0_BASE + S32K3XX_LPUART_STAT_OFFSET)
#define S32K3XX_LPUART0_CTRL           (S32K3XX_LPUART0_BASE + S32K3XX_LPUART_CTRL_OFFSET)
#define S32K3XX_LPUART0_DATA           (S32K3XX_LPUART0_BASE + S32K3XX_LPUART_DATA_OFFSET)
#define S32K3XX_LPUART0_MATCH          (S32K3XX_LPUART0_BASE + S32K3XX_LPUART_MATCH_OFFSET)
#define S32K3XX_LPUART0_MODIR          (S32K3XX_LPUART0_BASE + S32K3XX_LPUART_MODIR_OFFSET)
#define S32K3XX_LPUART0_FIFO           (S32K3XX_LPUART0_BASE + S32K3XX_LPUART_FIFO_OFFSET)
#define S32K3XX_LPUART0_WATER          (S32K3XX_LPUART0_BASE + S32K3XX_LPUART_WATER_OFFSET)
#define S32K3XX_LPUART0_DATARO         (S32K3XX_LPUART0_BASE + S32K3XX_LPUART_DATARO_OFFSET)

#define S32K3XX_LPUART1_VERID          (S32K3XX_LPUART1_BASE + S32K3XX_LPUART_VERID_OFFSET)
#define S32K3XX_LPUART1_PARAM          (S32K3XX_LPUART1_BASE + S32K3XX_LPUART_PARAM_OFFSET)
#define S32K3XX_LPUART1_GLOBAL         (S32K3XX_LPUART1_BASE + S32K3XX_LPUART_GLOBAL_OFFSET)
#define S32K3XX_LPUART1_PINCFG         (S32K3XX_LPUART1_BASE + S32K3XX_LPUART_PINCFG_OFFSET)
#define S32K3XX_LPUART1_BAUD           (S32K3XX_LPUART1_BASE + S32K3XX_LPUART_BAUD_OFFSET)
#define S32K3XX_LPUART1_STAT           (S32K3XX_LPUART1_BASE + S32K3XX_LPUART_STAT_OFFSET)
#define S32K3XX_LPUART1_CTRL           (S32K3XX_LPUART1_BASE + S32K3XX_LPUART_CTRL_OFFSET)
#define S32K3XX_LPUART1_DATA           (S32K3XX_LPUART1_BASE + S32K3XX_LPUART_DATA_OFFSET)
#define S32K3XX_LPUART1_MATCH          (S32K3XX_LPUART1_BASE + S32K3XX_LPUART_MATCH_OFFSET)
#define S32K3XX_LPUART1_MODIR          (S32K3XX_LPUART1_BASE + S32K3XX_LPUART_MODIR_OFFSET)
#define S32K3XX_LPUART1_FIFO           (S32K3XX_LPUART1_BASE + S32K3XX_LPUART_FIFO_OFFSET)
#define S32K3XX_LPUART1_WATER          (S32K3XX_LPUART1_BASE + S32K3XX_LPUART_WATER_OFFSET)
#define S32K3XX_LPUART1_DATARO         (S32K3XX_LPUART1_BASE + S32K3XX_LPUART_DATARO_OFFSET)

#define S32K3XX_LPUART2_VERID          (S32K3XX_LPUART2_BASE + S32K3XX_LPUART_VERID_OFFSET)
#define S32K3XX_LPUART2_PARAM          (S32K3XX_LPUART2_BASE + S32K3XX_LPUART_PARAM_OFFSET)
#define S32K3XX_LPUART2_GLOBAL         (S32K3XX_LPUART2_BASE + S32K3XX_LPUART_GLOBAL_OFFSET)
#define S32K3XX_LPUART2_PINCFG         (S32K3XX_LPUART2_BASE + S32K3XX_LPUART_PINCFG_OFFSET)
#define S32K3XX_LPUART2_BAUD           (S32K3XX_LPUART2_BASE + S32K3XX_LPUART_BAUD_OFFSET)
#define S32K3XX_LPUART2_STAT           (S32K3XX_LPUART2_BASE + S32K3XX_LPUART_STAT_OFFSET)
#define S32K3XX_LPUART2_CTRL           (S32K3XX_LPUART2_BASE + S32K3XX_LPUART_CTRL_OFFSET)
#define S32K3XX_LPUART2_DATA           (S32K3XX_LPUART2_BASE + S32K3XX_LPUART_DATA_OFFSET)
#define S32K3XX_LPUART2_MATCH          (S32K3XX_LPUART2_BASE + S32K3XX_LPUART_MATCH_OFFSET)
#define S32K3XX_LPUART2_MODIR          (S32K3XX_LPUART2_BASE + S32K3XX_LPUART_MODIR_OFFSET)
#define S32K3XX_LPUART2_FIFO           (S32K3XX_LPUART2_BASE + S32K3XX_LPUART_FIFO_OFFSET)
#define S32K3XX_LPUART2_WATER          (S32K3XX_LPUART2_BASE + S32K3XX_LPUART_WATER_OFFSET)
#define S32K3XX_LPUART2_DATARO         (S32K3XX_LPUART2_BASE + S32K3XX_LPUART_DATARO_OFFSET)

#define S32K3XX_LPUART3_VERID          (S32K3XX_LPUART3_BASE + S32K3XX_LPUART_VERID_OFFSET)
#define S32K3XX_LPUART3_PARAM          (S32K3XX_LPUART3_BASE + S32K3XX_LPUART_PARAM_OFFSET)
#define S32K3XX_LPUART3_GLOBAL         (S32K3XX_LPUART3_BASE + S32K3XX_LPUART_GLOBAL_OFFSET)
#define S32K3XX_LPUART3_PINCFG         (S32K3XX_LPUART3_BASE + S32K3XX_LPUART_PINCFG_OFFSET)
#define S32K3XX_LPUART3_BAUD           (S32K3XX_LPUART3_BASE + S32K3XX_LPUART_BAUD_OFFSET)
#define S32K3XX_LPUART3_STAT           (S32K3XX_LPUART3_BASE + S32K3XX_LPUART_STAT_OFFSET)
#define S32K3XX_LPUART3_CTRL           (S32K3XX_LPUART3_BASE + S32K3XX_LPUART_CTRL_OFFSET)
#define S32K3XX_LPUART3_DATA           (S32K3XX_LPUART3_BASE + S32K3XX_LPUART_DATA_OFFSET)
#define S32K3XX_LPUART3_MATCH          (S32K3XX_LPUART3_BASE + S32K3XX_LPUART_MATCH_OFFSET)
#define S32K3XX_LPUART3_MODIR          (S32K3XX_LPUART3_BASE + S32K3XX_LPUART_MODIR_OFFSET)
#define S32K3XX_LPUART3_FIFO           (S32K3XX_LPUART3_BASE + S32K3XX_LPUART_FIFO_OFFSET)
#define S32K3XX_LPUART3_WATER          (S32K3XX_LPUART3_BASE + S32K3XX_LPUART_WATER_OFFSET)
#define S32K3XX_LPUART3_DATARO         (S32K3XX_LPUART3_BASE + S32K3XX_LPUART_DATARO_OFFSET)

#define S32K3XX_LPUART4_VERID          (S32K3XX_LPUART4_BASE + S32K3XX_LPUART_VERID_OFFSET)
#define S32K3XX_LPUART4_PARAM          (S32K3XX_LPUART4_BASE + S32K3XX_LPUART_PARAM_OFFSET)
#define S32K3XX_LPUART4_GLOBAL         (S32K3XX_LPUART4_BASE + S32K3XX_LPUART_GLOBAL_OFFSET)
#define S32K3XX_LPUART4_PINCFG         (S32K3XX_LPUART4_BASE + S32K3XX_LPUART_PINCFG_OFFSET)
#define S32K3XX_LPUART4_BAUD           (S32K3XX_LPUART4_BASE + S32K3XX_LPUART_BAUD_OFFSET)
#define S32K3XX_LPUART4_STAT           (S32K3XX_LPUART4_BASE + S32K3XX_LPUART_STAT_OFFSET)
#define S32K3XX_LPUART4_CTRL           (S32K3XX_LPUART4_BASE + S32K3XX_LPUART_CTRL_OFFSET)
#define S32K3XX_LPUART4_DATA           (S32K3XX_LPUART4_BASE + S32K3XX_LPUART_DATA_OFFSET)
#define S32K3XX_LPUART4_MATCH          (S32K3XX_LPUART4_BASE + S32K3XX_LPUART_MATCH_OFFSET)
#define S32K3XX_LPUART4_MODIR          (S32K3XX_LPUART4_BASE + S32K3XX_LPUART_MODIR_OFFSET)
#define S32K3XX_LPUART4_FIFO           (S32K3XX_LPUART4_BASE + S32K3XX_LPUART_FIFO_OFFSET)
#define S32K3XX_LPUART4_WATER          (S32K3XX_LPUART4_BASE + S32K3XX_LPUART_WATER_OFFSET)
#define S32K3XX_LPUART4_DATARO         (S32K3XX_LPUART4_BASE + S32K3XX_LPUART_DATARO_OFFSET)

#define S32K3XX_LPUART5_VERID          (S32K3XX_LPUART5_BASE + S32K3XX_LPUART_VERID_OFFSET)
#define S32K3XX_LPUART5_PARAM          (S32K3XX_LPUART5_BASE + S32K3XX_LPUART_PARAM_OFFSET)
#define S32K3XX_LPUART5_GLOBAL         (S32K3XX_LPUART5_BASE + S32K3XX_LPUART_GLOBAL_OFFSET)
#define S32K3XX_LPUART5_PINCFG         (S32K3XX_LPUART5_BASE + S32K3XX_LPUART_PINCFG_OFFSET)
#define S32K3XX_LPUART5_BAUD           (S32K3XX_LPUART5_BASE + S32K3XX_LPUART_BAUD_OFFSET)
#define S32K3XX_LPUART5_STAT           (S32K3XX_LPUART5_BASE + S32K3XX_LPUART_STAT_OFFSET)
#define S32K3XX_LPUART5_CTRL           (S32K3XX_LPUART5_BASE + S32K3XX_LPUART_CTRL_OFFSET)
#define S32K3XX_LPUART5_DATA           (S32K3XX_LPUART5_BASE + S32K3XX_LPUART_DATA_OFFSET)
#define S32K3XX_LPUART5_MATCH          (S32K3XX_LPUART5_BASE + S32K3XX_LPUART_MATCH_OFFSET)
#define S32K3XX_LPUART5_MODIR          (S32K3XX_LPUART5_BASE + S32K3XX_LPUART_MODIR_OFFSET)
#define S32K3XX_LPUART5_FIFO           (S32K3XX_LPUART5_BASE + S32K3XX_LPUART_FIFO_OFFSET)
#define S32K3XX_LPUART5_WATER          (S32K3XX_LPUART5_BASE + S32K3XX_LPUART_WATER_OFFSET)
#define S32K3XX_LPUART5_DATARO         (S32K3XX_LPUART5_BASE + S32K3XX_LPUART_DATARO_OFFSET)

#define S32K3XX_LPUART6_VERID          (S32K3XX_LPUART6_BASE + S32K3XX_LPUART_VERID_OFFSET)
#define S32K3XX_LPUART6_PARAM          (S32K3XX_LPUART6_BASE + S32K3XX_LPUART_PARAM_OFFSET)
#define S32K3XX_LPUART6_GLOBAL         (S32K3XX_LPUART6_BASE + S32K3XX_LPUART_GLOBAL_OFFSET)
#define S32K3XX_LPUART6_PINCFG         (S32K3XX_LPUART6_BASE + S32K3XX_LPUART_PINCFG_OFFSET)
#define S32K3XX_LPUART6_BAUD           (S32K3XX_LPUART6_BASE + S32K3XX_LPUART_BAUD_OFFSET)
#define S32K3XX_LPUART6_STAT           (S32K3XX_LPUART6_BASE + S32K3XX_LPUART_STAT_OFFSET)
#define S32K3XX_LPUART6_CTRL           (S32K3XX_LPUART6_BASE + S32K3XX_LPUART_CTRL_OFFSET)
#define S32K3XX_LPUART6_DATA           (S32K3XX_LPUART6_BASE + S32K3XX_LPUART_DATA_OFFSET)
#define S32K3XX_LPUART6_MATCH          (S32K3XX_LPUART6_BASE + S32K3XX_LPUART_MATCH_OFFSET)
#define S32K3XX_LPUART6_MODIR          (S32K3XX_LPUART6_BASE + S32K3XX_LPUART_MODIR_OFFSET)
#define S32K3XX_LPUART6_FIFO           (S32K3XX_LPUART6_BASE + S32K3XX_LPUART_FIFO_OFFSET)
#define S32K3XX_LPUART6_WATER          (S32K3XX_LPUART6_BASE + S32K3XX_LPUART_WATER_OFFSET)
#define S32K3XX_LPUART6_DATARO         (S32K3XX_LPUART6_BASE + S32K3XX_LPUART_DATARO_OFFSET)

#define S32K3XX_LPUART7_VERID          (S32K3XX_LPUART7_BASE + S32K3XX_LPUART_VERID_OFFSET)
#define S32K3XX_LPUART7_PARAM          (S32K3XX_LPUART7_BASE + S32K3XX_LPUART_PARAM_OFFSET)
#define S32K3XX_LPUART7_GLOBAL         (S32K3XX_LPUART7_BASE + S32K3XX_LPUART_GLOBAL_OFFSET)
#define S32K3XX_LPUART7_PINCFG         (S32K3XX_LPUART7_BASE + S32K3XX_LPUART_PINCFG_OFFSET)
#define S32K3XX_LPUART7_BAUD           (S32K3XX_LPUART7_BASE + S32K3XX_LPUART_BAUD_OFFSET)
#define S32K3XX_LPUART7_STAT           (S32K3XX_LPUART7_BASE + S32K3XX_LPUART_STAT_OFFSET)
#define S32K3XX_LPUART7_CTRL           (S32K3XX_LPUART7_BASE + S32K3XX_LPUART_CTRL_OFFSET)
#define S32K3XX_LPUART7_DATA           (S32K3XX_LPUART7_BASE + S32K3XX_LPUART_DATA_OFFSET)
#define S32K3XX_LPUART7_MATCH          (S32K3XX_LPUART7_BASE + S32K3XX_LPUART_MATCH_OFFSET)
#define S32K3XX_LPUART7_MODIR          (S32K3XX_LPUART7_BASE + S32K3XX_LPUART_MODIR_OFFSET)
#define S32K3XX_LPUART7_FIFO           (S32K3XX_LPUART7_BASE + S32K3XX_LPUART_FIFO_OFFSET)
#define S32K3XX_LPUART7_WATER          (S32K3XX_LPUART7_BASE + S32K3XX_LPUART_WATER_OFFSET)
#define S32K3XX_LPUART7_DATARO         (S32K3XX_LPUART7_BASE + S32K3XX_LPUART_DATARO_OFFSET)

#define S32K3XX_LPUART8_VERID          (S32K3XX_LPUART8_BASE + S32K3XX_LPUART_VERID_OFFSET)
#define S32K3XX_LPUART8_PARAM          (S32K3XX_LPUART8_BASE + S32K3XX_LPUART_PARAM_OFFSET)
#define S32K3XX_LPUART8_GLOBAL         (S32K3XX_LPUART8_BASE + S32K3XX_LPUART_GLOBAL_OFFSET)
#define S32K3XX_LPUART8_PINCFG         (S32K3XX_LPUART8_BASE + S32K3XX_LPUART_PINCFG_OFFSET)
#define S32K3XX_LPUART8_BAUD           (S32K3XX_LPUART8_BASE + S32K3XX_LPUART_BAUD_OFFSET)
#define S32K3XX_LPUART8_STAT           (S32K3XX_LPUART8_BASE + S32K3XX_LPUART_STAT_OFFSET)
#define S32K3XX_LPUART8_CTRL           (S32K3XX_LPUART8_BASE + S32K3XX_LPUART_CTRL_OFFSET)
#define S32K3XX_LPUART8_DATA           (S32K3XX_LPUART8_BASE + S32K3XX_LPUART_DATA_OFFSET)
#define S32K3XX_LPUART8_MATCH          (S32K3XX_LPUART8_BASE + S32K3XX_LPUART_MATCH_OFFSET)
#define S32K3XX_LPUART8_MODIR          (S32K3XX_LPUART8_BASE + S32K3XX_LPUART_MODIR_OFFSET)
#define S32K3XX_LPUART8_FIFO           (S32K3XX_LPUART8_BASE + S32K3XX_LPUART_FIFO_OFFSET)
#define S32K3XX_LPUART8_WATER          (S32K3XX_LPUART8_BASE + S32K3XX_LPUART_WATER_OFFSET)
#define S32K3XX_LPUART8_DATARO         (S32K3XX_LPUART8_BASE + S32K3XX_LPUART_DATARO_OFFSET)

#define S32K3XX_LPUART9_VERID          (S32K3XX_LPUART9_BASE + S32K3XX_LPUART_VERID_OFFSET)
#define S32K3XX_LPUART9_PARAM          (S32K3XX_LPUART9_BASE + S32K3XX_LPUART_PARAM_OFFSET)
#define S32K3XX_LPUART9_GLOBAL         (S32K3XX_LPUART9_BASE + S32K3XX_LPUART_GLOBAL_OFFSET)
#define S32K3XX_LPUART9_PINCFG         (S32K3XX_LPUART9_BASE + S32K3XX_LPUART_PINCFG_OFFSET)
#define S32K3XX_LPUART9_BAUD           (S32K3XX_LPUART9_BASE + S32K3XX_LPUART_BAUD_OFFSET)
#define S32K3XX_LPUART9_STAT           (S32K3XX_LPUART9_BASE + S32K3XX_LPUART_STAT_OFFSET)
#define S32K3XX_LPUART9_CTRL           (S32K3XX_LPUART9_BASE + S32K3XX_LPUART_CTRL_OFFSET)
#define S32K3XX_LPUART9_DATA           (S32K3XX_LPUART9_BASE + S32K3XX_LPUART_DATA_OFFSET)
#define S32K3XX_LPUART9_MATCH          (S32K3XX_LPUART9_BASE + S32K3XX_LPUART_MATCH_OFFSET)
#define S32K3XX_LPUART9_MODIR          (S32K3XX_LPUART9_BASE + S32K3XX_LPUART_MODIR_OFFSET)
#define S32K3XX_LPUART9_FIFO           (S32K3XX_LPUART9_BASE + S32K3XX_LPUART_FIFO_OFFSET)
#define S32K3XX_LPUART9_WATER          (S32K3XX_LPUART9_BASE + S32K3XX_LPUART_WATER_OFFSET)
#define S32K3XX_LPUART9_DATARO         (S32K3XX_LPUART9_BASE + S32K3XX_LPUART_DATARO_OFFSET)

#define S32K3XX_LPUART10_VERID         (S32K3XX_LPUART10_BASE + S32K3XX_LPUART_VERID_OFFSET)
#define S32K3XX_LPUART10_PARAM         (S32K3XX_LPUART10_BASE + S32K3XX_LPUART_PARAM_OFFSET)
#define S32K3XX_LPUART10_GLOBAL        (S32K3XX_LPUART10_BASE + S32K3XX_LPUART_GLOBAL_OFFSET)
#define S32K3XX_LPUART10_PINCFG        (S32K3XX_LPUART10_BASE + S32K3XX_LPUART_PINCFG_OFFSET)
#define S32K3XX_LPUART10_BAUD          (S32K3XX_LPUART10_BASE + S32K3XX_LPUART_BAUD_OFFSET)
#define S32K3XX_LPUART10_STAT          (S32K3XX_LPUART10_BASE + S32K3XX_LPUART_STAT_OFFSET)
#define S32K3XX_LPUART10_CTRL          (S32K3XX_LPUART10_BASE + S32K3XX_LPUART_CTRL_OFFSET)
#define S32K3XX_LPUART10_DATA          (S32K3XX_LPUART10_BASE + S32K3XX_LPUART_DATA_OFFSET)
#define S32K3XX_LPUART10_MATCH         (S32K3XX_LPUART10_BASE + S32K3XX_LPUART_MATCH_OFFSET)
#define S32K3XX_LPUART10_MODIR         (S32K3XX_LPUART10_BASE + S32K3XX_LPUART_MODIR_OFFSET)
#define S32K3XX_LPUART10_FIFO          (S32K3XX_LPUART10_BASE + S32K3XX_LPUART_FIFO_OFFSET)
#define S32K3XX_LPUART10_WATER         (S32K3XX_LPUART10_BASE + S32K3XX_LPUART_WATER_OFFSET)
#define S32K3XX_LPUART10_DATARO        (S32K3XX_LPUART10_BASE + S32K3XX_LPUART_DATARO_OFFSET)

#define S32K3XX_LPUART11_VERID         (S32K3XX_LPUART11_BASE + S32K3XX_LPUART_VERID_OFFSET)
#define S32K3XX_LPUART11_PARAM         (S32K3XX_LPUART11_BASE + S32K3XX_LPUART_PARAM_OFFSET)
#define S32K3XX_LPUART11_GLOBAL        (S32K3XX_LPUART11_BASE + S32K3XX_LPUART_GLOBAL_OFFSET)
#define S32K3XX_LPUART11_PINCFG        (S32K3XX_LPUART11_BASE + S32K3XX_LPUART_PINCFG_OFFSET)
#define S32K3XX_LPUART11_BAUD          (S32K3XX_LPUART11_BASE + S32K3XX_LPUART_BAUD_OFFSET)
#define S32K3XX_LPUART11_STAT          (S32K3XX_LPUART11_BASE + S32K3XX_LPUART_STAT_OFFSET)
#define S32K3XX_LPUART11_CTRL          (S32K3XX_LPUART11_BASE + S32K3XX_LPUART_CTRL_OFFSET)
#define S32K3XX_LPUART11_DATA          (S32K3XX_LPUART11_BASE + S32K3XX_LPUART_DATA_OFFSET)
#define S32K3XX_LPUART11_MATCH         (S32K3XX_LPUART11_BASE + S32K3XX_LPUART_MATCH_OFFSET)
#define S32K3XX_LPUART11_MODIR         (S32K3XX_LPUART11_BASE + S32K3XX_LPUART_MODIR_OFFSET)
#define S32K3XX_LPUART11_FIFO          (S32K3XX_LPUART11_BASE + S32K3XX_LPUART_FIFO_OFFSET)
#define S32K3XX_LPUART11_WATER         (S32K3XX_LPUART11_BASE + S32K3XX_LPUART_WATER_OFFSET)
#define S32K3XX_LPUART11_DATARO        (S32K3XX_LPUART11_BASE + S32K3XX_LPUART_DATARO_OFFSET)

#define S32K3XX_LPUART12_VERID         (S32K3XX_LPUART12_BASE + S32K3XX_LPUART_VERID_OFFSET)
#define S32K3XX_LPUART12_PARAM         (S32K3XX_LPUART12_BASE + S32K3XX_LPUART_PARAM_OFFSET)
#define S32K3XX_LPUART12_GLOBAL        (S32K3XX_LPUART12_BASE + S32K3XX_LPUART_GLOBAL_OFFSET)
#define S32K3XX_LPUART12_PINCFG        (S32K3XX_LPUART12_BASE + S32K3XX_LPUART_PINCFG_OFFSET)
#define S32K3XX_LPUART12_BAUD          (S32K3XX_LPUART12_BASE + S32K3XX_LPUART_BAUD_OFFSET)
#define S32K3XX_LPUART12_STAT          (S32K3XX_LPUART12_BASE + S32K3XX_LPUART_STAT_OFFSET)
#define S32K3XX_LPUART12_CTRL          (S32K3XX_LPUART12_BASE + S32K3XX_LPUART_CTRL_OFFSET)
#define S32K3XX_LPUART12_DATA          (S32K3XX_LPUART12_BASE + S32K3XX_LPUART_DATA_OFFSET)
#define S32K3XX_LPUART12_MATCH         (S32K3XX_LPUART12_BASE + S32K3XX_LPUART_MATCH_OFFSET)
#define S32K3XX_LPUART12_MODIR         (S32K3XX_LPUART12_BASE + S32K3XX_LPUART_MODIR_OFFSET)
#define S32K3XX_LPUART12_FIFO          (S32K3XX_LPUART12_BASE + S32K3XX_LPUART_FIFO_OFFSET)
#define S32K3XX_LPUART12_WATER         (S32K3XX_LPUART12_BASE + S32K3XX_LPUART_WATER_OFFSET)
#define S32K3XX_LPUART12_DATARO        (S32K3XX_LPUART12_BASE + S32K3XX_LPUART_DATARO_OFFSET)

#define S32K3XX_LPUART13_VERID         (S32K3XX_LPUART13_BASE + S32K3XX_LPUART_VERID_OFFSET)
#define S32K3XX_LPUART13_PARAM         (S32K3XX_LPUART13_BASE + S32K3XX_LPUART_PARAM_OFFSET)
#define S32K3XX_LPUART13_GLOBAL        (S32K3XX_LPUART13_BASE + S32K3XX_LPUART_GLOBAL_OFFSET)
#define S32K3XX_LPUART13_PINCFG        (S32K3XX_LPUART13_BASE + S32K3XX_LPUART_PINCFG_OFFSET)
#define S32K3XX_LPUART13_BAUD          (S32K3XX_LPUART13_BASE + S32K3XX_LPUART_BAUD_OFFSET)
#define S32K3XX_LPUART13_STAT          (S32K3XX_LPUART13_BASE + S32K3XX_LPUART_STAT_OFFSET)
#define S32K3XX_LPUART13_CTRL          (S32K3XX_LPUART13_BASE + S32K3XX_LPUART_CTRL_OFFSET)
#define S32K3XX_LPUART13_DATA          (S32K3XX_LPUART13_BASE + S32K3XX_LPUART_DATA_OFFSET)
#define S32K3XX_LPUART13_MATCH         (S32K3XX_LPUART13_BASE + S32K3XX_LPUART_MATCH_OFFSET)
#define S32K3XX_LPUART13_MODIR         (S32K3XX_LPUART13_BASE + S32K3XX_LPUART_MODIR_OFFSET)
#define S32K3XX_LPUART13_FIFO          (S32K3XX_LPUART13_BASE + S32K3XX_LPUART_FIFO_OFFSET)
#define S32K3XX_LPUART13_WATER         (S32K3XX_LPUART13_BASE + S32K3XX_LPUART_WATER_OFFSET)
#define S32K3XX_LPUART13_DATARO        (S32K3XX_LPUART13_BASE + S32K3XX_LPUART_DATARO_OFFSET)

#define S32K3XX_LPUART14_VERID         (S32K3XX_LPUART14_BASE + S32K3XX_LPUART_VERID_OFFSET)
#define S32K3XX_LPUART14_PARAM         (S32K3XX_LPUART14_BASE + S32K3XX_LPUART_PARAM_OFFSET)
#define S32K3XX_LPUART14_GLOBAL        (S32K3XX_LPUART14_BASE + S32K3XX_LPUART_GLOBAL_OFFSET)
#define S32K3XX_LPUART14_PINCFG        (S32K3XX_LPUART14_BASE + S32K3XX_LPUART_PINCFG_OFFSET)
#define S32K3XX_LPUART14_BAUD          (S32K3XX_LPUART14_BASE + S32K3XX_LPUART_BAUD_OFFSET)
#define S32K3XX_LPUART14_STAT          (S32K3XX_LPUART14_BASE + S32K3XX_LPUART_STAT_OFFSET)
#define S32K3XX_LPUART14_CTRL          (S32K3XX_LPUART14_BASE + S32K3XX_LPUART_CTRL_OFFSET)
#define S32K3XX_LPUART14_DATA          (S32K3XX_LPUART14_BASE + S32K3XX_LPUART_DATA_OFFSET)
#define S32K3XX_LPUART14_MATCH         (S32K3XX_LPUART14_BASE + S32K3XX_LPUART_MATCH_OFFSET)
#define S32K3XX_LPUART14_MODIR         (S32K3XX_LPUART14_BASE + S32K3XX_LPUART_MODIR_OFFSET)
#define S32K3XX_LPUART14_FIFO          (S32K3XX_LPUART14_BASE + S32K3XX_LPUART_FIFO_OFFSET)
#define S32K3XX_LPUART14_WATER         (S32K3XX_LPUART14_BASE + S32K3XX_LPUART_WATER_OFFSET)
#define S32K3XX_LPUART14_DATARO        (S32K3XX_LPUART14_BASE + S32K3XX_LPUART_DATARO_OFFSET)

#define S32K3XX_LPUART15_VERID         (S32K3XX_LPUART15_BASE + S32K3XX_LPUART_VERID_OFFSET)
#define S32K3XX_LPUART15_PARAM         (S32K3XX_LPUART15_BASE + S32K3XX_LPUART_PARAM_OFFSET)
#define S32K3XX_LPUART15_GLOBAL        (S32K3XX_LPUART15_BASE + S32K3XX_LPUART_GLOBAL_OFFSET)
#define S32K3XX_LPUART15_PINCFG        (S32K3XX_LPUART15_BASE + S32K3XX_LPUART_PINCFG_OFFSET)
#define S32K3XX_LPUART15_BAUD          (S32K3XX_LPUART15_BASE + S32K3XX_LPUART_BAUD_OFFSET)
#define S32K3XX_LPUART15_STAT          (S32K3XX_LPUART15_BASE + S32K3XX_LPUART_STAT_OFFSET)
#define S32K3XX_LPUART15_CTRL          (S32K3XX_LPUART15_BASE + S32K3XX_LPUART_CTRL_OFFSET)
#define S32K3XX_LPUART15_DATA          (S32K3XX_LPUART15_BASE + S32K3XX_LPUART_DATA_OFFSET)
#define S32K3XX_LPUART15_MATCH         (S32K3XX_LPUART15_BASE + S32K3XX_LPUART_MATCH_OFFSET)
#define S32K3XX_LPUART15_MODIR         (S32K3XX_LPUART15_BASE + S32K3XX_LPUART_MODIR_OFFSET)
#define S32K3XX_LPUART15_FIFO          (S32K3XX_LPUART15_BASE + S32K3XX_LPUART_FIFO_OFFSET)
#define S32K3XX_LPUART15_WATER         (S32K3XX_LPUART15_BASE + S32K3XX_LPUART_WATER_OFFSET)
#define S32K3XX_LPUART15_DATARO        (S32K3XX_LPUART15_BASE + S32K3XX_LPUART_DATARO_OFFSET)

/* Register bit definitions *************************************************/

/* Version ID Register (VERID) */

#define LPUART_VERID_FEATURE_SHIFT     (0)       /* Bits 0-15: Feature Identification Number (FEATURE) */
#define LPUART_VERID_FEATURE_MASK      (0xffff << LPUART_VERID_FEATURE_SHIFT)
#  define LPUART_VERID_FEATURE_STD     (1 << LPUART_VERID_FEATURE_SHIFT) /* Standard feature set */
#  define LPUART_VERID_FEATURE_MODEM   (3 << LPUART_VERID_FEATURE_SHIFT) /* MODEM/IrDA support */

#define LPUART_VERID_MINOR_SHIFT       (16)      /* Bits 16-23: Minor Version Number (MINOR) */
#define LPUART_VERID_MINOR_MASK        (0xff << LPUART_VERID_MINOR_SHIFT)
#define LPUART_VERID_MAJOR_SHIFT       (24)      /* Bits 24-31: Major Version Number (MAJOR) */
#define LPUART_VERID_MAJOR_MASK        (0xff << LPUART_VERID_MAJOR_SHIFT)

/* Parameter Register (PARAM) */

#define LPUART_PARAM_TXFIFO_SHIFT      (0)       /* Bits 0-7: Transmit FIFO Size (TXFIFO) */
#define LPUART_PARAM_TXFIFO_MASK       (0xff << LPUART_PARAM_TXFIFO_SHIFT)
#define LPUART_PARAM_RXFIFO_SHIFT      (8)       /* Bits 8-15: Receive FIFO Size (RXFIFO) */
#define LPUART_PARAM_RXFIFO_MASK       (0xff << LPUART_PARAM_RXFIFO_SHIFT)
                                                 /* Bits 16-31: Reserved */

/* LPUART Global Register (GLOBAL) */

                                                 /* Bit 0: Reserved */
#define LPUART_GLOBAL_RST              (1 << 1)  /* Bit 1: Software Reset (RST) */
                                                 /* Bits 2-31: Reserved */

/* LPUART Pin Configuration Register (PINCFG) */

#define LPUART_PINCFG_TRGSEL_SHIFT     (0)       /* Bits 0-1: Trigger Select (TRGSEL) */
#define LPUART_PINCFG_TRGSEL_MASK      (0x03 << LPUART_PINCFG_TRGSEL_SHIFT)
#  define LPUART_PINCFG_TRGSEL_DISABLE (0 << LPUART_PINCFG_TRGSEL_SHIFT) /* Trigger disabled */
#  define LPUART_PINCFG_TRGSEL_RXD     (1 << LPUART_PINCFG_TRGSEL_SHIFT) /* Trigger used instead of RXD pin */
#  define LPUART_PINCFG_TRGSEL_CTSB    (2 << LPUART_PINCFG_TRGSEL_SHIFT) /* Trigger used instead of CTS_B pin */
#  define LPUART_PINCFG_TRGSEL_TXDMOD  (3 << LPUART_PINCFG_TRGSEL_SHIFT) /* Trigger used to modulate the TXD output */

                                                 /* Bits 2-31: Reserved */

/* LPUART Baud Rate Register (BAUD) */

#define LPUART_BAUD_SBR_SHIFT          (0)       /* Bits 0-12: Baud Rate Modulo Divisor (SBR) */
#define LPUART_BAUD_SBR_MASK           (0x1fff << LPUART_BAUD_SBR_SHIFT)
#  define LPUART_BAUD_SBR(n)           ((n) << LPUART_BAUD_SBR_SHIFT)
#define LPUART_BAUD_SBNS               (1 << 13) /* Bit 13: Stop Bit Number Select (SBNS) */
#define LPUART_BAUD_RXEDGIE            (1 << 14) /* Bit 14: RX Input Active Edge Interrupt Enable (RXEDGIE) */
#define LPUART_BAUD_LBKDIE             (1 << 15) /* Bit 15: LIN Break Detect Interrupt Enable (LBKDIE) */
#define LPUART_BAUD_RESYNCDIS          (1 << 16) /* Bit 16: Resynchronization Disable (RESYNCDIS) */
#define LPUART_BAUD_BOTHEDGE           (1 << 17) /* Bit 17: Both Edge Sampling (BOTHEDGE) */
#define LPUART_BAUD_MATCFG_SHIFT       (18)      /* Bits 18-19: Match Configuration (MATCFG) */
#define LPUART_BAUD_MATCFG_MASK        (0x03 << LPUART_BAUD_MATCFG_SHIFT)
#  define LPUART_BAUD_MATCFG_ADDR      (0 << LPUART_BAUD_MATCFG_SHIFT) /* Address Match Wakeup */
#  define LPUART_BAUD_MATCFG_IDLE      (1 << LPUART_BAUD_MATCFG_SHIFT) /* Idle Match Wakeup */
#  define LPUART_BAUD_MATCFG_ONOFF     (2 << LPUART_BAUD_MATCFG_SHIFT) /* Match On and Match Off */
#  define LPUART_BAUD_MATCFG_RWUENAB   (3 << LPUART_BAUD_MATCFG_SHIFT) /* Enables RWU on Data Match and Match On/Off for transmitter CTS input */

                                                 /* Bit 20: Reserved */
#define LPUART_BAUD_RDMAE              (1 << 21) /* Bit 21: Receiver Full DMA Enable (RDMAE) */
                                                 /* Bit 22: Reserved */
#define LPUART_BAUD_TDMAE              (1 << 23) /* Bit 23: Transmitter DMA Enable (TDMAE) */
#define LPUART_BAUD_OSR_SHIFT          (24)      /* Bits 24-28: Oversampling Ratio (OSR) */
#define LPUART_BAUD_OSR_MASK           (0x0f << LPUART_BAUD_OSR_SHIFT)
#  define LPUART_BAUD_OSR(n)           (((n) - 1) << LPUART_BAUD_OSR_SHIFT) /* n=4..32 */

#define LPUART_BAUD_M10                (1 << 29) /* Bit 29: 10-bit Mode Select (M10) */
#define LPUART_BAUD_MAEN2              (1 << 30) /* Bit 30: Match Address Mode Enable 2 (MAEN2) */
#define LPUART_BAUD_MAEN1              (1 << 31) /* Bit 31: Match Address Mode Enable 1 (MAEN1) */

/* LPUART Status Register (STAT) */

#define LPUART_STAT_LBKFE              (1 << 0)  /* Bit 0: LIN Break Flag Enable (LBKFE) */
#define LPUART_STAT_AME                (1 << 1)  /* Bit 1: Address Mark Enable (AME) */
                                                 /* Bits 2-13: Reserved */
#define LPUART_STAT_MA2F               (1 << 14) /* Bit 14: Match 2 Flag (MA2F) */
#define LPUART_STAT_MA1F               (1 << 15) /* Bit 15: Match 1 Flag (MA1F) */
#define LPUART_STAT_PF                 (1 << 16) /* Bit 16: Parity Error Flag (PF) */
#define LPUART_STAT_FE                 (1 << 17) /* Bit 17: Framing Error Flag (FE) */
#define LPUART_STAT_NF                 (1 << 18) /* Bit 18: Noise Flag (NF) */
#define LPUART_STAT_OR                 (1 << 19) /* Bit 19: Receiver Overrun Flag (OR) */
#define LPUART_STAT_IDLE               (1 << 20) /* Bit 20: Idle Line Flag (IDLE) */
#define LPUART_STAT_RDRF               (1 << 21) /* Bit 21: Receive Data Register Full Flag (RDRF) */
#define LPUART_STAT_TC                 (1 << 22) /* Bit 22: Transmission Complete Flag (TC) */
#define LPUART_STAT_TDRE               (1 << 23) /* Bit 23: Transmit Data Register Empty Flag (TDRE) */
#define LPUART_STAT_RAF                (1 << 24) /* Bit 24: Receiver Active Flag (RAF) */
#define LPUART_STAT_LBKDE              (1 << 25) /* Bit 25: LIN Break Detection Enable (LBKDE) */
#define LPUART_STAT_BRK13              (1 << 26) /* Bit 26: Break Character Generation Length (BRK13) */
#define LPUART_STAT_RWUID              (1 << 27) /* Bit 27: Receive Wake Up Idle Detect (RWUID) */
#define LPUART_STAT_RXINV              (1 << 28) /* Bit 28: Receive Data Inversion (RXINV) */
#define LPUART_STAT_MSBF               (1 << 29) /* Bit 29: MSB First (MSBF) */
#define LPUART_STAT_RXEDGIF            (1 << 30) /* Bit 30: RXD Pin Active Edge Interrupt Flag (RXEDGIF) */
#define LPUART_STAT_LBKDIF             (1 << 31) /* Bit 31: LIN Break Detect Interrupt Flag (LBKDIF) */

/* LPUART Control Register (CTRL) */

#define LPUART_CTRL_PT                 (1 << 0)  /* Bit 0: Parity Type */
#  define LPUART_CTRL_PT_EVEN          (0 << 0)  /*        Even parity */
#  define LPUART_CTRL_PT_ODD           (1 << 0)  /*        Odd parity */
#define LPUART_CTRL_PE                 (1 << 1)  /* Bit 1: Parity Enable */
#define LPUART_CTRL_ILT                (1 << 2)  /* Bit 2: Idle Line Type Select */
#define LPUART_CTRL_WAKE               (1 << 3)  /* Bit 3: Receiver Wakeup Method Select */
#define LPUART_CTRL_M                  (1 << 4)  /* Bit 4: 9-Bit or 8-Bit Mode Select */
#define LPUART_CTRL_RSRC               (1 << 5)  /* Bit 5: Receiver Source Select */
#define LPUART_CTRL_DOZEEN             (1 << 6)  /* Bit 6: Doze Enable */
#define LPUART_CTRL_LOOPS              (1 << 7)  /* Bit 7: Loop Mode Select */
#define LPUART_CTRL_IDLECFG_SHIFT      (8)       /* Bits 8-10: Idle Configuration */
#define LPUART_CTRL_IDLECFG_MASK       (0x07 << LPUART_CTRL_IDLECFG_SHIFT)
#  define LPUART_CTRL_IDLECFG_1        (0 << LPUART_CTRL_IDLECFG_SHIFT) /* 1 idle character */
#  define LPUART_CTRL_IDLECFG_2        (1 << LPUART_CTRL_IDLECFG_SHIFT) /* 2 idle characters */
#  define LPUART_CTRL_IDLECFG_4        (2 << LPUART_CTRL_IDLECFG_SHIFT) /* 4 idle characters */
#  define LPUART_CTRL_IDLECFG_8        (3 << LPUART_CTRL_IDLECFG_SHIFT) /* 8 idle characters */
#  define LPUART_CTRL_IDLECFG_16       (4 << LPUART_CTRL_IDLECFG_SHIFT) /* 6 idle characters */
#  define LPUART_CTRL_IDLECFG_32       (5 << LPUART_CTRL_IDLECFG_SHIFT) /* 32 idle characters */
#  define LPUART_CTRL_IDLECFG_64       (6 << LPUART_CTRL_IDLECFG_SHIFT) /* 64 idle characters */
#  define LPUART_CTRL_IDLECFG_128      (7 << LPUART_CTRL_IDLECFG_SHIFT) /* 128 idle characters */

#define LPUART_CTRL_M7                 (1 << 11) /* Bit 11: 7-Bit Mode Select (M7) */
                                                 /* Bits 12-13: Reserved */
#define LPUART_CTRL_MA2IE              (1 << 14) /* Bit 14: Match 2 Interrupt Enable (MA2IE) */
#define LPUART_CTRL_MA1IE              (1 << 15) /* Bit 15: Match 1 Interrupt Enable (MA1IE) */
#define LPUART_CTRL_SBK                (1 << 16) /* Bit 16: Send Break (SBK) */
#define LPUART_CTRL_RWU                (1 << 17) /* Bit 17: Receiver Wakeup Control (RWU) */
#define LPUART_CTRL_RE                 (1 << 18) /* Bit 18: Receiver Enable (RE) */
#define LPUART_CTRL_TE                 (1 << 19) /* Bit 19: Transmitter Enable (TE) */
#define LPUART_CTRL_ILIE               (1 << 20) /* Bit 20: Idle Line Interrupt Enable (ILIE) */
#define LPUART_CTRL_RIE                (1 << 21) /* Bit 21: Receiver Interrupt Enable (RIE) */
#define LPUART_CTRL_TCIE               (1 << 22) /* Bit 22: Transmission Complete Interrupt Enable (TCIE) */
#define LPUART_CTRL_TIE                (1 << 23) /* Bit 23: Transmit Interrupt Enable (TIE) */
#define LPUART_CTRL_PEIE               (1 << 24) /* Bit 24: Parity Error Interrupt Enable (PEIE) */
#define LPUART_CTRL_FEIE               (1 << 25) /* Bit 25: Framing Error Interrupt Enable (FEIE) */
#define LPUART_CTRL_NEIE               (1 << 26) /* Bit 26: Noise Error Interrupt Enable (NEIE) */
#define LPUART_CTRL_ORIE               (1 << 27) /* Bit 27: Overrun Interrupt Enable (ORIE) */
#define LPUART_CTRL_TXINV              (1 << 28) /* Bit 28: Transmit Data Inversion (TXINV) */
#define LPUART_CTRL_TXDIR              (1 << 29) /* Bit 29: TXD Pin Direction in Single-Wire Mode (TXDIR) */
#define LPUART_CTRL_R9T8               (1 << 30) /* Bit 30: Receive Bit 9 / Transmit Bit 8 (R9T8) */
#define LPUART_CTRL_R8T9               (1 << 31) /* Bit 31: Receive Bit 8 / Transmit Bit 9 (R8T9) */

#define LPUART_ALL_INTS (LPUART_CTRL_ORIE | LPUART_CTRL_NEIE | LPUART_CTRL_FEIE |  \
                         LPUART_CTRL_PEIE | LPUART_CTRL_TIE  | LPUART_CTRL_TCIE |  \
                         LPUART_CTRL_RIE  | LPUART_CTRL_ILIE | LPUART_CTRL_MA1IE | \
                         LPUART_CTRL_MA2IE)

/* LPUART Data Register (DATA) */

#define LPUART_DATA_SHIFT              (0)       /* Bits 0-9: Data bits 0-9 (DATA)*/
#define LPUART_DATA_MASK               (0x03ff << LPUART_DATA_SHIFT)
#define LPUART_DATA_LINBRK             (1 << 10) /* Bit 10: LIN Break (LINBRK) */
#define LPUART_DATA_STATUS_SHIFT       (11)      /* Bits 11-15: Status */
#define LPUART_DATA_IDLINE             (1 << 11) /* Bit 11: Idle Line (IDLINE) */
#define LPUART_DATA_RXEMPT             (1 << 12) /* Bit 12: Receive Buffer Empty (RXEMPT) */
#define LPUART_DATA_FRETSC             (1 << 13) /* Bit 13: Frame Error / Transmit Special Character (FRETSC) */
#define LPUART_DATA_PARITYE            (1 << 14) /* Bit 14: Parity Error (PARITYE) */
#define LPUART_DATA_NOISY              (1 << 15) /* Bit 15: Noisy Data Received (NOISY) */
                                                 /* Bits 16-31: Reserved */

/* LPUART Match Address Register (MATCH) */

#define LPUART_MATCH_MA1_SHIFT         (0)       /* Bits 0-9: Match Address 1 (MA1) */
#define LPUART_MATCH_MA1_MASK          (0x03ff << LPUART_MATCH_MA1_SHIFT)
#  define LPUART_MATCH_MA1(n)          ((n) << LPUART_MATCH_MA1_SHIFT)
                                                 /* Bits 10-15: Reserved */
#define LPUART_MATCH_MA2_SHIFT         (16)      /* Bits 16-25: Match Address 2 (MA2) */
#define LPUART_MATCH_MA2_MASK          (0x03ff << LPUART_MATCH_MA2_SHIFT)
#  define LPUART_MATCH_MA2(n)          ((n) << LPUART_MATCH_MA2_SHIFT)
                                                 /* Bits 26-31: Reserved */

/* LPUART Modem IrDA Register (MODIR) */

#define LPUART_MODIR_TXCTSE            (1 << 0)  /* Bit 0: Transmitter clear-to-send enable (TXCTSE) */
#define LPUART_MODIR_TXRTSE            (1 << 1)  /* Bit 1: Transmitter request-to-send enable (TXRTSE) */
#define LPUART_MODIR_TXRTSPOL          (1 << 2)  /* Bit 2: Transmitter request-to-send polarity (TXRTSPOL) */
#define LPUART_MODIR_RXRTSE            (1 << 3)  /* Bit 3: Receiver request-to-send enable (RXRTSE) */
#define LPUART_MODIR_TXCTSC            (1 << 4)  /* Bit 4: Transmit CTS Configuration (TXCTSC) */
#  define LPUART_MODIR_TXCTSC_START    (0 << 4)  /*        CTS sampled at start of character */
#  define LPUART_MODIR_TXCTSC_IDLE     (1 << 4)  /*        CTS sampled when transmitter idle */
#define LPUART_MODIR_TXCTSSRC          (1 << 5)  /* Bit 5: Transmit CTS Source (TXCTSSRC) */
#  define LPUART_MODIR_TXCTSSRC_CTSB   (0 << 5)  /*        CTS input is CTS_B pin */
#  define LPUART_MODIR_TXCTSSRC_RXMAT  (1 << 5)  /*        CTS input is receiver address match result */
                                                 /* Bits 6-7: Reserved */
#define LPUART_MODIR_RTSWATER_SHIFT    (8)       /* Bits 8-9: Receive RTS Configuration (RTSWATER) */
#define LPUART_MODIR_RTSWATER_MASK     (0x03 << LPUART_MODIR_RTSWATER_SHIFT)
#  define LPUART_MODIR_RTSWATER(n)     ((n) << LPUART_MODIR_RTSWATER_SHIFT)
                                                 /* Bits 10-15: Reserved */
#define LPUART_MODIR_TNP_SHIFT         (16)      /* Bits 16-17: Transmitter narrow pulse (TNP) */
#define LPUART_MODIR_TNP_MASK          (0x03 << LPUART_MODIR_TNP_SHIFT)
#  define LPUART_MODIR_TNP(n)          (((n) - 1) << LPUART_MODIR_TNP_SHIFT) /* n/OSR */

#define LPUART_MODIR_IREN              (1 << 18) /* Bit 18: Infrared enable (IREN) */
                                                 /* Bits 19-31: Reserved */

/* LPUART FIFO Register (FIFO) */

#define LPUART_FIFO_RXFIFOSIZE_SHIFT   (0)       /* Bits 0-2: Receive FIFO Buffer Depth (RXFIFOSIZE) */
#define LPUART_FIFO_RXFIFOSIZE_MASK    (0x07 << LPUART_FIFO_RXFIFOSIZE_SHIFT)
#  define LPUART_FIFO_RXFIFOSIZE_1     (0 << LPUART_FIFO_RXFIFOSIZE_SHIFT) /* 1 dataword */
#  define LPUART_FIFO_RXFIFOSIZE_4     (1 << LPUART_FIFO_RXFIFOSIZE_SHIFT) /* 4 datawords */
#  define LPUART_FIFO_RXFIFOSIZE_8     (2 << LPUART_FIFO_RXFIFOSIZE_SHIFT) /* 8 datawords */
#  define LPUART_FIFO_RXFIFOSIZE_16    (3 << LPUART_FIFO_RXFIFOSIZE_SHIFT) /* 16 datawords */
#  define LPUART_FIFO_RXFIFOSIZE_32    (4 << LPUART_FIFO_RXFIFOSIZE_SHIFT) /* 32 datawords */
#  define LPUART_FIFO_RXFIFOSIZE_64    (5 << LPUART_FIFO_RXFIFOSIZE_SHIFT) /* 64 datawords */
#  define LPUART_FIFO_RXFIFOSIZE_128   (6 << LPUART_FIFO_RXFIFOSIZE_SHIFT) /* 128 datawords */
#  define LPUART_FIFO_RXFIFOSIZE_256   (7 << LPUART_FIFO_RXFIFOSIZE_SHIFT) /* 256 datawords */

#define LPUART_FIFO_RXFE               (1 << 3)  /* Bit 3: Receive FIFO Enable (RXFE) */
#define LPUART_FIFO_TXFIFOSIZE_SHIFT   (4)       /* Bits 4-6: Transmit FIFO Buffer Depth (TXFIFOSIZE) */
#define LPUART_FIFO_TXFIFOSIZE_MASK    (0x07 << LPUART_FIFO_TXFIFOSIZE_SHIFT)
#  define LPUART_FIFO_TXFIFOSIZE_1     (0 << LPUART_FIFO_TXFIFOSIZE_SHIFT) /* 1 dataword */
#  define LPUART_FIFO_TXFIFOSIZE_4     (1 << LPUART_FIFO_TXFIFOSIZE_SHIFT) /* 4 datawords */
#  define LPUART_FIFO_TXFIFOSIZE_8     (2 << LPUART_FIFO_TXFIFOSIZE_SHIFT) /* 8 datawords */
#  define LPUART_FIFO_TXFIFOSIZE_16    (3 << LPUART_FIFO_TXFIFOSIZE_SHIFT) /* 16 datawords */
#  define LPUART_FIFO_TXFIFOSIZE_32    (4 << LPUART_FIFO_TXFIFOSIZE_SHIFT) /* 32 datawords */
#  define LPUART_FIFO_TXFIFOSIZE_64    (5 << LPUART_FIFO_TXFIFOSIZE_SHIFT) /* 64 datawords */
#  define LPUART_FIFO_TXFIFOSIZE_128   (6 << LPUART_FIFO_TXFIFOSIZE_SHIFT) /* 128 datawords */
#  define LPUART_FIFO_TXFIFOSIZE_256   (7 << LPUART_FIFO_TXFIFOSIZE_SHIFT) /* 256 datawords */

#define LPUART_FIFO_TXFE               (1 << 7)  /* Bit 7: Transmit FIFO Enable (TXFE) */
#define LPUART_FIFO_RXUFE              (1 << 8)  /* Bit 8: Receive FIFO Underflow Interrupt Enable (RXUFE) */
#define LPUART_FIFO_TXOFE              (1 << 9)  /* Bit 9: Transmit FIFO Overflow Interrupt Enable (TXOFE) */
#define LPUART_FIFO_RXIDEN_SHIFT       (10)      /* Bits 10-12: Receiver Idle Empty Enable (RXIDEN) */
#define LPUART_FIFO_RXIDEN_MASK        (0x07 << LPUART_FIFO_RXIDEN_SHIFT)
#  define LPUART_FIFO_RXIDEN_DISABLE   (0 << LPUART_FIFO_RXIDEN_SHIFT) /* Disable RDRF assertion when receiver is idle */
#  define LPUART_FIFO_RXIDEN_1         (1 << LPUART_FIFO_RXIDEN_SHIFT) /* Enable RDRF assertion when receiver idle for 1 character */
#  define LPUART_FIFO_RXIDEN_2         (2 << LPUART_FIFO_RXIDEN_SHIFT) /* Enable RDRF assertion when receiver idle for 2 characters */
#  define LPUART_FIFO_RXIDEN_4         (3 << LPUART_FIFO_RXIDEN_SHIFT) /* Enable RDRF assertion when receiver idle for 4 characters */
#  define LPUART_FIFO_RXIDEN_8         (4 << LPUART_FIFO_RXIDEN_SHIFT) /* Enable RDRF assertion when receiver idle for 8 characters */
#  define LPUART_FIFO_RXIDEN_16        (5 << LPUART_FIFO_RXIDEN_SHIFT) /* Enable RDRF assertion when receiver idle for 16 characters */
#  define LPUART_FIFO_RXIDEN_32        (6 << LPUART_FIFO_RXIDEN_SHIFT) /* Enable RDRF assertion when receiver idle for 32 characters */
#  define LPUART_FIFO_RXIDEN_64        (7 << LPUART_FIFO_RXIDEN_SHIFT) /* Enable RDRF assertion when receiver idle for 64 characters */

                                                 /* Bit 13: Reserved */
#define LPUART_FIFO_RXFLUSH            (1 << 14) /* Bit 14: Receive FIFO Flush (RXFLUSH) */
#define LPUART_FIFO_TXFLUSH            (1 << 15) /* Bit 15: Transmit FIFO Flush (TXFLUSH) */
#define LPUART_FIFO_RXUF               (1 << 16) /* Bit 16: Receiver FIFO Underflow Flag (RXUF) */
#define LPUART_FIFO_TXOF               (1 << 17) /* Bit 17: Transmitter FIFO Overflow Flag (TXOF) */
                                                 /* Bits 18-21: Reserved */
#define LPUART_FIFO_RXEMPT             (1 << 22) /* Bit 22: Receive Buffer/FIFO Empty (RXEMPT) */
#define LPUART_FIFO_TXEMPT             (1 << 23) /* Bit 23: Transmit Buffer/FIFO Empty (TXEMPT) */
                                                 /* Bits 24-31: Reserved */

/* LPUART Watermark Register (WATER) */

#define LPUART_WATER_TXWATER_SHIFT     (0)       /* Bits 0-1: Transmit Watermark (TXWATER) */
#define LPUART_WATER_TXWATER_MASK      (0x03 << LPUART_WATER_TXWATER_SHIFT)
#  define LPUART_WATER_TXWATER(n)      ((n) << LPUART_WATER_TXWATER_SHIFT)
                                                 /* Bits 2-7: Reserved */
#define LPUART_WATER_TXCOUNT_SHIFT     (8)       /* Bits 8-10: Transmit Counter (TXCOUNT) */
#define LPUART_WATER_TXCOUNT_MASK      (0x07 << LPUART_WATER_TXCOUNT_SHIFT)
#  define LPUART_WATER_TXCOUNT(n)      ((n) << LPUART_WATER_TXCOUNT_SHIFT)
                                                 /* Bits 11-15: Reserved */
#define LPUART_WATER_RXWATER_SHIFT     (16)      /* Bits 16-17: Receive Watermark (RXWATER) */
#define LPUART_WATER_RXWATER_MASK      (0x03 << LPUART_WATER_RXWATER_SHIFT)
#  define LPUART_WATER_RXWATER(n)      ((n) << LPUART_WATER_RXWATER_SHIFT)
                                                 /* Bits 18-23: Reserved */
#define LPUART_WATER_RXCOUNT_SHIFT     (24)      /* Bits 24-26: Receive Counter (RXCOUNT) */
#define LPUART_WATER_RXCOUNT_MASK      (0x07 << LPUART_WATER_RXCOUNT_SHIFT)
#  define LPUART_WATER_RXCOUNT(n)      ((n) << LPUART_WATER_RXCOUNT_SHIFT)
                                                 /* Bits 27-31: Reserved */

/* Data read-only Register (DATARO) */

#define LPUART_DATARO_DATA_SHIFT       (0)       /* Bits 0-15: Receive Data (DATA) */
#define LPUART_DATARO_DATA_MASK        (0xffff << LPUART_DATARO_DATA_SHIFT)
                                                 /* Bits 16-31: Reserved */

#endif /* __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_LPUART_H */
