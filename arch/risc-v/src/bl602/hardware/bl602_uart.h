/****************************************************************************
 * arch/risc-v/src/bl602/hardware/bl602_uart.h
 *
 * Copyright (C) 2012, 2015 Gregory Nutt. All rights reserved.
 * Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_UART_H
#define __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_UART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/bl602_common.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 0x0 : utx_config */

#define UART_UTX_CONFIG_OFFSET (0x0)
#define UART_CR_UTX_EN         UART_CR_UTX_EN
#define UART_CR_UTX_EN_POS     (0)
#define UART_CR_UTX_EN_LEN     (1)
#define UART_CR_UTX_EN_MSK \
  (((1 << UART_CR_UTX_EN_LEN) - 1) << UART_CR_UTX_EN_POS)
#define UART_CR_UTX_EN_UMSK \
  (~(((1 << UART_CR_UTX_EN_LEN) - 1) << UART_CR_UTX_EN_POS))
#define UART_CR_UTX_CTS_EN     UART_CR_UTX_CTS_EN
#define UART_CR_UTX_CTS_EN_POS (1)
#define UART_CR_UTX_CTS_EN_LEN (1)
#define UART_CR_UTX_CTS_EN_MSK \
  (((1 << UART_CR_UTX_CTS_EN_LEN) - 1) << UART_CR_UTX_CTS_EN_POS)
#define UART_CR_UTX_CTS_EN_UMSK \
  (~(((1 << UART_CR_UTX_CTS_EN_LEN) - 1) << UART_CR_UTX_CTS_EN_POS))
#define UART_CR_UTX_FRM_EN     UART_CR_UTX_FRM_EN
#define UART_CR_UTX_FRM_EN_POS (2)
#define UART_CR_UTX_FRM_EN_LEN (1)
#define UART_CR_UTX_FRM_EN_MSK \
  (((1 << UART_CR_UTX_FRM_EN_LEN) - 1) << UART_CR_UTX_FRM_EN_POS)
#define UART_CR_UTX_FRM_EN_UMSK \
  (~(((1 << UART_CR_UTX_FRM_EN_LEN) - 1) << UART_CR_UTX_FRM_EN_POS))
#define UART_CR_UTX_PRT_EN     UART_CR_UTX_PRT_EN
#define UART_CR_UTX_PRT_EN_POS (4)
#define UART_CR_UTX_PRT_EN_LEN (1)
#define UART_CR_UTX_PRT_EN_MSK \
  (((1 << UART_CR_UTX_PRT_EN_LEN) - 1) << UART_CR_UTX_PRT_EN_POS)
#define UART_CR_UTX_PRT_EN_UMSK \
  (~(((1 << UART_CR_UTX_PRT_EN_LEN) - 1) << UART_CR_UTX_PRT_EN_POS))
#define UART_CR_UTX_PRT_SEL     UART_CR_UTX_PRT_SEL
#define UART_CR_UTX_PRT_SEL_POS (5)
#define UART_CR_UTX_PRT_SEL_LEN (1)
#define UART_CR_UTX_PRT_SEL_MSK \
  (((1 << UART_CR_UTX_PRT_SEL_LEN) - 1) << UART_CR_UTX_PRT_SEL_POS)
#define UART_CR_UTX_PRT_SEL_UMSK \
  (~(((1 << UART_CR_UTX_PRT_SEL_LEN) - 1) << UART_CR_UTX_PRT_SEL_POS))
#define UART_CR_UTX_IR_EN     UART_CR_UTX_IR_EN
#define UART_CR_UTX_IR_EN_POS (6)
#define UART_CR_UTX_IR_EN_LEN (1)
#define UART_CR_UTX_IR_EN_MSK \
  (((1 << UART_CR_UTX_IR_EN_LEN) - 1) << UART_CR_UTX_IR_EN_POS)
#define UART_CR_UTX_IR_EN_UMSK \
  (~(((1 << UART_CR_UTX_IR_EN_LEN) - 1) << UART_CR_UTX_IR_EN_POS))
#define UART_CR_UTX_IR_INV     UART_CR_UTX_IR_INV
#define UART_CR_UTX_IR_INV_POS (7)
#define UART_CR_UTX_IR_INV_LEN (1)
#define UART_CR_UTX_IR_INV_MSK \
  (((1 << UART_CR_UTX_IR_INV_LEN) - 1) << UART_CR_UTX_IR_INV_POS)
#define UART_CR_UTX_IR_INV_UMSK \
  (~(((1 << UART_CR_UTX_IR_INV_LEN) - 1) << UART_CR_UTX_IR_INV_POS))
#define UART_CR_UTX_BIT_CNT_D     UART_CR_UTX_BIT_CNT_D
#define UART_CR_UTX_BIT_CNT_D_POS (8)
#define UART_CR_UTX_BIT_CNT_D_LEN (3)
#define UART_CR_UTX_BIT_CNT_D_MSK \
  (((1 << UART_CR_UTX_BIT_CNT_D_LEN) - 1) << UART_CR_UTX_BIT_CNT_D_POS)
#define UART_CR_UTX_BIT_CNT_D_UMSK \
  (~(((1 << UART_CR_UTX_BIT_CNT_D_LEN) - 1) << UART_CR_UTX_BIT_CNT_D_POS))
#define UART_CR_UTX_BIT_CNT_P     UART_CR_UTX_BIT_CNT_P
#define UART_CR_UTX_BIT_CNT_P_POS (12)
#define UART_CR_UTX_BIT_CNT_P_LEN (2)
#define UART_CR_UTX_BIT_CNT_P_MSK \
  (((1 << UART_CR_UTX_BIT_CNT_P_LEN) - 1) << UART_CR_UTX_BIT_CNT_P_POS)
#define UART_CR_UTX_BIT_CNT_P_UMSK \
  (~(((1 << UART_CR_UTX_BIT_CNT_P_LEN) - 1) << UART_CR_UTX_BIT_CNT_P_POS))
#define UART_CR_UTX_LEN     UART_CR_UTX_LEN
#define UART_CR_UTX_LEN_POS (16)
#define UART_CR_UTX_LEN_LEN (16)
#define UART_CR_UTX_LEN_MSK \
  (((1 << UART_CR_UTX_LEN_LEN) - 1) << UART_CR_UTX_LEN_POS)
#define UART_CR_UTX_LEN_UMSK \
  (~(((1 << UART_CR_UTX_LEN_LEN) - 1) << UART_CR_UTX_LEN_POS))

/* 0x4 : urx_config */

#define UART_URX_CONFIG_OFFSET (0x4)
#define UART_CR_URX_EN         UART_CR_URX_EN
#define UART_CR_URX_EN_POS     (0)
#define UART_CR_URX_EN_LEN     (1)
#define UART_CR_URX_EN_MSK \
  (((1 << UART_CR_URX_EN_LEN) - 1) << UART_CR_URX_EN_POS)
#define UART_CR_URX_EN_UMSK \
  (~(((1 << UART_CR_URX_EN_LEN) - 1) << UART_CR_URX_EN_POS))
#define UART_CR_URX_RTS_SW_MODE     UART_CR_URX_RTS_SW_MODE
#define UART_CR_URX_RTS_SW_MODE_POS (1)
#define UART_CR_URX_RTS_SW_MODE_LEN (1)
#define UART_CR_URX_RTS_SW_MODE_MSK \
  (((1 << UART_CR_URX_RTS_SW_MODE_LEN) - 1) << UART_CR_URX_RTS_SW_MODE_POS)
#define UART_CR_URX_RTS_SW_MODE_UMSK \
  (~(((1 << UART_CR_URX_RTS_SW_MODE_LEN) - 1) << UART_CR_URX_RTS_SW_MODE_POS))
#define UART_CR_URX_RTS_SW_VAL     UART_CR_URX_RTS_SW_VAL
#define UART_CR_URX_RTS_SW_VAL_POS (2)
#define UART_CR_URX_RTS_SW_VAL_LEN (1)
#define UART_CR_URX_RTS_SW_VAL_MSK \
  (((1 << UART_CR_URX_RTS_SW_VAL_LEN) - 1) << UART_CR_URX_RTS_SW_VAL_POS)
#define UART_CR_URX_RTS_SW_VAL_UMSK \
  (~(((1 << UART_CR_URX_RTS_SW_VAL_LEN) - 1) << UART_CR_URX_RTS_SW_VAL_POS))
#define UART_CR_URX_ABR_EN     UART_CR_URX_ABR_EN
#define UART_CR_URX_ABR_EN_POS (3)
#define UART_CR_URX_ABR_EN_LEN (1)
#define UART_CR_URX_ABR_EN_MSK \
  (((1 << UART_CR_URX_ABR_EN_LEN) - 1) << UART_CR_URX_ABR_EN_POS)
#define UART_CR_URX_ABR_EN_UMSK \
  (~(((1 << UART_CR_URX_ABR_EN_LEN) - 1) << UART_CR_URX_ABR_EN_POS))
#define UART_CR_URX_PRT_EN     UART_CR_URX_PRT_EN
#define UART_CR_URX_PRT_EN_POS (4)
#define UART_CR_URX_PRT_EN_LEN (1)
#define UART_CR_URX_PRT_EN_MSK \
  (((1 << UART_CR_URX_PRT_EN_LEN) - 1) << UART_CR_URX_PRT_EN_POS)
#define UART_CR_URX_PRT_EN_UMSK \
  (~(((1 << UART_CR_URX_PRT_EN_LEN) - 1) << UART_CR_URX_PRT_EN_POS))
#define UART_CR_URX_PRT_SEL     UART_CR_URX_PRT_SEL
#define UART_CR_URX_PRT_SEL_POS (5)
#define UART_CR_URX_PRT_SEL_LEN (1)
#define UART_CR_URX_PRT_SEL_MSK \
  (((1 << UART_CR_URX_PRT_SEL_LEN) - 1) << UART_CR_URX_PRT_SEL_POS)
#define UART_CR_URX_PRT_SEL_UMSK \
  (~(((1 << UART_CR_URX_PRT_SEL_LEN) - 1) << UART_CR_URX_PRT_SEL_POS))
#define UART_CR_URX_IR_EN     UART_CR_URX_IR_EN
#define UART_CR_URX_IR_EN_POS (6)
#define UART_CR_URX_IR_EN_LEN (1)
#define UART_CR_URX_IR_EN_MSK \
  (((1 << UART_CR_URX_IR_EN_LEN) - 1) << UART_CR_URX_IR_EN_POS)
#define UART_CR_URX_IR_EN_UMSK \
  (~(((1 << UART_CR_URX_IR_EN_LEN) - 1) << UART_CR_URX_IR_EN_POS))
#define UART_CR_URX_IR_INV     UART_CR_URX_IR_INV
#define UART_CR_URX_IR_INV_POS (7)
#define UART_CR_URX_IR_INV_LEN (1)
#define UART_CR_URX_IR_INV_MSK \
  (((1 << UART_CR_URX_IR_INV_LEN) - 1) << UART_CR_URX_IR_INV_POS)
#define UART_CR_URX_IR_INV_UMSK \
  (~(((1 << UART_CR_URX_IR_INV_LEN) - 1) << UART_CR_URX_IR_INV_POS))
#define UART_CR_URX_BIT_CNT_D     UART_CR_URX_BIT_CNT_D
#define UART_CR_URX_BIT_CNT_D_POS (8)
#define UART_CR_URX_BIT_CNT_D_LEN (3)
#define UART_CR_URX_BIT_CNT_D_MSK \
  (((1 << UART_CR_URX_BIT_CNT_D_LEN) - 1) << UART_CR_URX_BIT_CNT_D_POS)
#define UART_CR_URX_BIT_CNT_D_UMSK \
  (~(((1 << UART_CR_URX_BIT_CNT_D_LEN) - 1) << UART_CR_URX_BIT_CNT_D_POS))
#define UART_CR_URX_DEG_EN     UART_CR_URX_DEG_EN
#define UART_CR_URX_DEG_EN_POS (11)
#define UART_CR_URX_DEG_EN_LEN (1)
#define UART_CR_URX_DEG_EN_MSK \
  (((1 << UART_CR_URX_DEG_EN_LEN) - 1) << UART_CR_URX_DEG_EN_POS)
#define UART_CR_URX_DEG_EN_UMSK \
  (~(((1 << UART_CR_URX_DEG_EN_LEN) - 1) << UART_CR_URX_DEG_EN_POS))
#define UART_CR_URX_DEG_CNT     UART_CR_URX_DEG_CNT
#define UART_CR_URX_DEG_CNT_POS (12)
#define UART_CR_URX_DEG_CNT_LEN (4)
#define UART_CR_URX_DEG_CNT_MSK \
  (((1 << UART_CR_URX_DEG_CNT_LEN) - 1) << UART_CR_URX_DEG_CNT_POS)
#define UART_CR_URX_DEG_CNT_UMSK \
  (~(((1 << UART_CR_URX_DEG_CNT_LEN) - 1) << UART_CR_URX_DEG_CNT_POS))
#define UART_CR_URX_LEN     UART_CR_URX_LEN
#define UART_CR_URX_LEN_POS (16)
#define UART_CR_URX_LEN_LEN (16)
#define UART_CR_URX_LEN_MSK \
  (((1 << UART_CR_URX_LEN_LEN) - 1) << UART_CR_URX_LEN_POS)
#define UART_CR_URX_LEN_UMSK \
  (~(((1 << UART_CR_URX_LEN_LEN) - 1) << UART_CR_URX_LEN_POS))

/* 0x8 : uart_bit_prd */

#define UART_BIT_PRD_OFFSET     (0x8)
#define UART_CR_UTX_BIT_PRD     UART_CR_UTX_BIT_PRD
#define UART_CR_UTX_BIT_PRD_POS (0)
#define UART_CR_UTX_BIT_PRD_LEN (16)
#define UART_CR_UTX_BIT_PRD_MSK \
  (((1 << UART_CR_UTX_BIT_PRD_LEN) - 1) << UART_CR_UTX_BIT_PRD_POS)
#define UART_CR_UTX_BIT_PRD_UMSK \
  (~(((1 << UART_CR_UTX_BIT_PRD_LEN) - 1) << UART_CR_UTX_BIT_PRD_POS))
#define UART_CR_URX_BIT_PRD     UART_CR_URX_BIT_PRD
#define UART_CR_URX_BIT_PRD_POS (16)
#define UART_CR_URX_BIT_PRD_LEN (16)
#define UART_CR_URX_BIT_PRD_MSK \
  (((1 << UART_CR_URX_BIT_PRD_LEN) - 1) << UART_CR_URX_BIT_PRD_POS)
#define UART_CR_URX_BIT_PRD_UMSK \
  (~(((1 << UART_CR_URX_BIT_PRD_LEN) - 1) << UART_CR_URX_BIT_PRD_POS))

/* 0xC : data_config */

#define UART_DATA_CONFIG_OFFSET  (0xC)
#define UART_CR_UART_BIT_INV     UART_CR_UART_BIT_INV
#define UART_CR_UART_BIT_INV_POS (0)
#define UART_CR_UART_BIT_INV_LEN (1)
#define UART_CR_UART_BIT_INV_MSK \
  (((1 << UART_CR_UART_BIT_INV_LEN) - 1) << UART_CR_UART_BIT_INV_POS)
#define UART_CR_UART_BIT_INV_UMSK \
  (~(((1 << UART_CR_UART_BIT_INV_LEN) - 1) << UART_CR_UART_BIT_INV_POS))

/* 0x10 : utx_ir_position */

#define UART_UTX_IR_POSITION_OFFSET (0x10)
#define UART_CR_UTX_IR_POS_S        UART_CR_UTX_IR_POS_S
#define UART_CR_UTX_IR_POS_S_POS    (0)
#define UART_CR_UTX_IR_POS_S_LEN    (16)
#define UART_CR_UTX_IR_POS_S_MSK \
  (((1 << UART_CR_UTX_IR_POS_S_LEN) - 1) << UART_CR_UTX_IR_POS_S_POS)
#define UART_CR_UTX_IR_POS_S_UMSK \
  (~(((1 << UART_CR_UTX_IR_POS_S_LEN) - 1) << UART_CR_UTX_IR_POS_S_POS))
#define UART_CR_UTX_IR_POS_P     UART_CR_UTX_IR_POS_P
#define UART_CR_UTX_IR_POS_P_POS (16)
#define UART_CR_UTX_IR_POS_P_LEN (16)
#define UART_CR_UTX_IR_POS_P_MSK \
  (((1 << UART_CR_UTX_IR_POS_P_LEN) - 1) << UART_CR_UTX_IR_POS_P_POS)
#define UART_CR_UTX_IR_POS_P_UMSK \
  (~(((1 << UART_CR_UTX_IR_POS_P_LEN) - 1) << UART_CR_UTX_IR_POS_P_POS))

/* 0x14 : urx_ir_position */

#define UART_URX_IR_POSITION_OFFSET (0x14)
#define UART_CR_URX_IR_POS_S        UART_CR_URX_IR_POS_S
#define UART_CR_URX_IR_POS_S_POS    (0)
#define UART_CR_URX_IR_POS_S_LEN    (16)
#define UART_CR_URX_IR_POS_S_MSK \
  (((1 << UART_CR_URX_IR_POS_S_LEN) - 1) << UART_CR_URX_IR_POS_S_POS)
#define UART_CR_URX_IR_POS_S_UMSK \
  (~(((1 << UART_CR_URX_IR_POS_S_LEN) - 1) << UART_CR_URX_IR_POS_S_POS))

/* 0x18 : urx_rto_timer */

#define UART_URX_RTO_TIMER_OFFSET (0x18)
#define UART_CR_URX_RTO_VALUE     UART_CR_URX_RTO_VALUE
#define UART_CR_URX_RTO_VALUE_POS (0)
#define UART_CR_URX_RTO_VALUE_LEN (8)
#define UART_CR_URX_RTO_VALUE_MSK \
  (((1 << UART_CR_URX_RTO_VALUE_LEN) - 1) << UART_CR_URX_RTO_VALUE_POS)
#define UART_CR_URX_RTO_VALUE_UMSK \
  (~(((1 << UART_CR_URX_RTO_VALUE_LEN) - 1) << UART_CR_URX_RTO_VALUE_POS))

/* 0x20 : UART interrupt status */

#define UART_INT_STS_OFFSET  (0x20)
#define UART_UTX_END_INT     UART_UTX_END_INT
#define UART_UTX_END_INT_POS (0)
#define UART_UTX_END_INT_LEN (1)
#define UART_UTX_END_INT_MSK \
  (((1 << UART_UTX_END_INT_LEN) - 1) << UART_UTX_END_INT_POS)
#define UART_UTX_END_INT_UMSK \
  (~(((1 << UART_UTX_END_INT_LEN) - 1) << UART_UTX_END_INT_POS))
#define UART_URX_END_INT     UART_URX_END_INT
#define UART_URX_END_INT_POS (1)
#define UART_URX_END_INT_LEN (1)
#define UART_URX_END_INT_MSK \
  (((1 << UART_URX_END_INT_LEN) - 1) << UART_URX_END_INT_POS)
#define UART_URX_END_INT_UMSK \
  (~(((1 << UART_URX_END_INT_LEN) - 1) << UART_URX_END_INT_POS))
#define UART_UTX_FIFO_INT     UART_UTX_FIFO_INT
#define UART_UTX_FIFO_INT_POS (2)
#define UART_UTX_FIFO_INT_LEN (1)
#define UART_UTX_FIFO_INT_MSK \
  (((1 << UART_UTX_FIFO_INT_LEN) - 1) << UART_UTX_FIFO_INT_POS)
#define UART_UTX_FIFO_INT_UMSK \
  (~(((1 << UART_UTX_FIFO_INT_LEN) - 1) << UART_UTX_FIFO_INT_POS))
#define UART_URX_FIFO_INT     UART_URX_FIFO_INT
#define UART_URX_FIFO_INT_POS (3)
#define UART_URX_FIFO_INT_LEN (1)
#define UART_URX_FIFO_INT_MSK \
  (((1 << UART_URX_FIFO_INT_LEN) - 1) << UART_URX_FIFO_INT_POS)
#define UART_URX_FIFO_INT_UMSK \
  (~(((1 << UART_URX_FIFO_INT_LEN) - 1) << UART_URX_FIFO_INT_POS))
#define UART_URX_RTO_INT     UART_URX_RTO_INT
#define UART_URX_RTO_INT_POS (4)
#define UART_URX_RTO_INT_LEN (1)
#define UART_URX_RTO_INT_MSK \
  (((1 << UART_URX_RTO_INT_LEN) - 1) << UART_URX_RTO_INT_POS)
#define UART_URX_RTO_INT_UMSK \
  (~(((1 << UART_URX_RTO_INT_LEN) - 1) << UART_URX_RTO_INT_POS))
#define UART_URX_PCE_INT     UART_URX_PCE_INT
#define UART_URX_PCE_INT_POS (5)
#define UART_URX_PCE_INT_LEN (1)
#define UART_URX_PCE_INT_MSK \
  (((1 << UART_URX_PCE_INT_LEN) - 1) << UART_URX_PCE_INT_POS)
#define UART_URX_PCE_INT_UMSK \
  (~(((1 << UART_URX_PCE_INT_LEN) - 1) << UART_URX_PCE_INT_POS))
#define UART_UTX_FER_INT     UART_UTX_FER_INT
#define UART_UTX_FER_INT_POS (6)
#define UART_UTX_FER_INT_LEN (1)
#define UART_UTX_FER_INT_MSK \
  (((1 << UART_UTX_FER_INT_LEN) - 1) << UART_UTX_FER_INT_POS)
#define UART_UTX_FER_INT_UMSK \
  (~(((1 << UART_UTX_FER_INT_LEN) - 1) << UART_UTX_FER_INT_POS))
#define UART_URX_FER_INT     UART_URX_FER_INT
#define UART_URX_FER_INT_POS (7)
#define UART_URX_FER_INT_LEN (1)
#define UART_URX_FER_INT_MSK \
  (((1 << UART_URX_FER_INT_LEN) - 1) << UART_URX_FER_INT_POS)
#define UART_URX_FER_INT_UMSK \
  (~(((1 << UART_URX_FER_INT_LEN) - 1) << UART_URX_FER_INT_POS))

/* 0x24 : UART interrupt mask */

#define UART_INT_MASK_OFFSET     (0x24)
#define UART_CR_UTX_END_MASK     UART_CR_UTX_END_MASK
#define UART_CR_UTX_END_MASK_POS (0)
#define UART_CR_UTX_END_MASK_LEN (1)
#define UART_CR_UTX_END_MASK_MSK \
  (((1 << UART_CR_UTX_END_MASK_LEN) - 1) << UART_CR_UTX_END_MASK_POS)
#define UART_CR_UTX_END_MASK_UMSK \
  (~(((1 << UART_CR_UTX_END_MASK_LEN) - 1) << UART_CR_UTX_END_MASK_POS))
#define UART_CR_URX_END_MASK     UART_CR_URX_END_MASK
#define UART_CR_URX_END_MASK_POS (1)
#define UART_CR_URX_END_MASK_LEN (1)
#define UART_CR_URX_END_MASK_MSK \
  (((1 << UART_CR_URX_END_MASK_LEN) - 1) << UART_CR_URX_END_MASK_POS)
#define UART_CR_URX_END_MASK_UMSK \
  (~(((1 << UART_CR_URX_END_MASK_LEN) - 1) << UART_CR_URX_END_MASK_POS))
#define UART_CR_UTX_FIFO_MASK     UART_CR_UTX_FIFO_MASK
#define UART_CR_UTX_FIFO_MASK_POS (2)
#define UART_CR_UTX_FIFO_MASK_LEN (1)
#define UART_CR_UTX_FIFO_MASK_MSK \
  (((1 << UART_CR_UTX_FIFO_MASK_LEN) - 1) << UART_CR_UTX_FIFO_MASK_POS)
#define UART_CR_UTX_FIFO_MASK_UMSK \
  (~(((1 << UART_CR_UTX_FIFO_MASK_LEN) - 1) << UART_CR_UTX_FIFO_MASK_POS))
#define UART_CR_URX_FIFO_MASK     UART_CR_URX_FIFO_MASK
#define UART_CR_URX_FIFO_MASK_POS (3)
#define UART_CR_URX_FIFO_MASK_LEN (1)
#define UART_CR_URX_FIFO_MASK_MSK \
  (((1 << UART_CR_URX_FIFO_MASK_LEN) - 1) << UART_CR_URX_FIFO_MASK_POS)
#define UART_CR_URX_FIFO_MASK_UMSK \
  (~(((1 << UART_CR_URX_FIFO_MASK_LEN) - 1) << UART_CR_URX_FIFO_MASK_POS))
#define UART_CR_URX_RTO_MASK     UART_CR_URX_RTO_MASK
#define UART_CR_URX_RTO_MASK_POS (4)
#define UART_CR_URX_RTO_MASK_LEN (1)
#define UART_CR_URX_RTO_MASK_MSK \
  (((1 << UART_CR_URX_RTO_MASK_LEN) - 1) << UART_CR_URX_RTO_MASK_POS)
#define UART_CR_URX_RTO_MASK_UMSK \
  (~(((1 << UART_CR_URX_RTO_MASK_LEN) - 1) << UART_CR_URX_RTO_MASK_POS))
#define UART_CR_URX_PCE_MASK     UART_CR_URX_PCE_MASK
#define UART_CR_URX_PCE_MASK_POS (5)
#define UART_CR_URX_PCE_MASK_LEN (1)
#define UART_CR_URX_PCE_MASK_MSK \
  (((1 << UART_CR_URX_PCE_MASK_LEN) - 1) << UART_CR_URX_PCE_MASK_POS)
#define UART_CR_URX_PCE_MASK_UMSK \
  (~(((1 << UART_CR_URX_PCE_MASK_LEN) - 1) << UART_CR_URX_PCE_MASK_POS))
#define UART_CR_UTX_FER_MASK     UART_CR_UTX_FER_MASK
#define UART_CR_UTX_FER_MASK_POS (6)
#define UART_CR_UTX_FER_MASK_LEN (1)
#define UART_CR_UTX_FER_MASK_MSK \
  (((1 << UART_CR_UTX_FER_MASK_LEN) - 1) << UART_CR_UTX_FER_MASK_POS)
#define UART_CR_UTX_FER_MASK_UMSK \
  (~(((1 << UART_CR_UTX_FER_MASK_LEN) - 1) << UART_CR_UTX_FER_MASK_POS))
#define UART_CR_URX_FER_MASK     UART_CR_URX_FER_MASK
#define UART_CR_URX_FER_MASK_POS (7)
#define UART_CR_URX_FER_MASK_LEN (1)
#define UART_CR_URX_FER_MASK_MSK \
  (((1 << UART_CR_URX_FER_MASK_LEN) - 1) << UART_CR_URX_FER_MASK_POS)
#define UART_CR_URX_FER_MASK_UMSK \
  (~(((1 << UART_CR_URX_FER_MASK_LEN) - 1) << UART_CR_URX_FER_MASK_POS))

/* 0x28 : UART interrupt clear */

#define UART_INT_CLEAR_OFFSET   (0x28)
#define UART_CR_UTX_END_CLR     UART_CR_UTX_END_CLR
#define UART_CR_UTX_END_CLR_POS (0)
#define UART_CR_UTX_END_CLR_LEN (1)
#define UART_CR_UTX_END_CLR_MSK \
  (((1 << UART_CR_UTX_END_CLR_LEN) - 1) << UART_CR_UTX_END_CLR_POS)
#define UART_CR_UTX_END_CLR_UMSK \
  (~(((1 << UART_CR_UTX_END_CLR_LEN) - 1) << UART_CR_UTX_END_CLR_POS))
#define UART_CR_URX_END_CLR     UART_CR_URX_END_CLR
#define UART_CR_URX_END_CLR_POS (1)
#define UART_CR_URX_END_CLR_LEN (1)
#define UART_CR_URX_END_CLR_MSK \
  (((1 << UART_CR_URX_END_CLR_LEN) - 1) << UART_CR_URX_END_CLR_POS)
#define UART_CR_URX_END_CLR_UMSK \
  (~(((1 << UART_CR_URX_END_CLR_LEN) - 1) << UART_CR_URX_END_CLR_POS))
#define UART_CR_URX_RTO_CLR     UART_CR_URX_RTO_CLR
#define UART_CR_URX_RTO_CLR_POS (4)
#define UART_CR_URX_RTO_CLR_LEN (1)
#define UART_CR_URX_RTO_CLR_MSK \
  (((1 << UART_CR_URX_RTO_CLR_LEN) - 1) << UART_CR_URX_RTO_CLR_POS)
#define UART_CR_URX_RTO_CLR_UMSK \
  (~(((1 << UART_CR_URX_RTO_CLR_LEN) - 1) << UART_CR_URX_RTO_CLR_POS))
#define UART_CR_URX_PCE_CLR     UART_CR_URX_PCE_CLR
#define UART_CR_URX_PCE_CLR_POS (5)
#define UART_CR_URX_PCE_CLR_LEN (1)
#define UART_CR_URX_PCE_CLR_MSK \
  (((1 << UART_CR_URX_PCE_CLR_LEN) - 1) << UART_CR_URX_PCE_CLR_POS)
#define UART_CR_URX_PCE_CLR_UMSK \
  (~(((1 << UART_CR_URX_PCE_CLR_LEN) - 1) << UART_CR_URX_PCE_CLR_POS))

/* 0x2C : UART interrupt enable */

#define UART_INT_EN_OFFSET     (0x2C)
#define UART_CR_UTX_END_EN     UART_CR_UTX_END_EN
#define UART_CR_UTX_END_EN_POS (0)
#define UART_CR_UTX_END_EN_LEN (1)
#define UART_CR_UTX_END_EN_MSK \
  (((1 << UART_CR_UTX_END_EN_LEN) - 1) << UART_CR_UTX_END_EN_POS)
#define UART_CR_UTX_END_EN_UMSK \
  (~(((1 << UART_CR_UTX_END_EN_LEN) - 1) << UART_CR_UTX_END_EN_POS))
#define UART_CR_URX_END_EN     UART_CR_URX_END_EN
#define UART_CR_URX_END_EN_POS (1)
#define UART_CR_URX_END_EN_LEN (1)
#define UART_CR_URX_END_EN_MSK \
  (((1 << UART_CR_URX_END_EN_LEN) - 1) << UART_CR_URX_END_EN_POS)
#define UART_CR_URX_END_EN_UMSK \
  (~(((1 << UART_CR_URX_END_EN_LEN) - 1) << UART_CR_URX_END_EN_POS))
#define UART_CR_UTX_FIFO_EN     UART_CR_UTX_FIFO_EN
#define UART_CR_UTX_FIFO_EN_POS (2)
#define UART_CR_UTX_FIFO_EN_LEN (1)
#define UART_CR_UTX_FIFO_EN_MSK \
  (((1 << UART_CR_UTX_FIFO_EN_LEN) - 1) << UART_CR_UTX_FIFO_EN_POS)
#define UART_CR_UTX_FIFO_EN_UMSK \
  (~(((1 << UART_CR_UTX_FIFO_EN_LEN) - 1) << UART_CR_UTX_FIFO_EN_POS))
#define UART_CR_URX_FIFO_EN     UART_CR_URX_FIFO_EN
#define UART_CR_URX_FIFO_EN_POS (3)
#define UART_CR_URX_FIFO_EN_LEN (1)
#define UART_CR_URX_FIFO_EN_MSK \
  (((1 << UART_CR_URX_FIFO_EN_LEN) - 1) << UART_CR_URX_FIFO_EN_POS)
#define UART_CR_URX_FIFO_EN_UMSK \
  (~(((1 << UART_CR_URX_FIFO_EN_LEN) - 1) << UART_CR_URX_FIFO_EN_POS))
#define UART_CR_URX_RTO_EN     UART_CR_URX_RTO_EN
#define UART_CR_URX_RTO_EN_POS (4)
#define UART_CR_URX_RTO_EN_LEN (1)
#define UART_CR_URX_RTO_EN_MSK \
  (((1 << UART_CR_URX_RTO_EN_LEN) - 1) << UART_CR_URX_RTO_EN_POS)
#define UART_CR_URX_RTO_EN_UMSK \
  (~(((1 << UART_CR_URX_RTO_EN_LEN) - 1) << UART_CR_URX_RTO_EN_POS))
#define UART_CR_URX_PCE_EN     UART_CR_URX_PCE_EN
#define UART_CR_URX_PCE_EN_POS (5)
#define UART_CR_URX_PCE_EN_LEN (1)
#define UART_CR_URX_PCE_EN_MSK \
  (((1 << UART_CR_URX_PCE_EN_LEN) - 1) << UART_CR_URX_PCE_EN_POS)
#define UART_CR_URX_PCE_EN_UMSK \
  (~(((1 << UART_CR_URX_PCE_EN_LEN) - 1) << UART_CR_URX_PCE_EN_POS))
#define UART_CR_UTX_FER_EN     UART_CR_UTX_FER_EN
#define UART_CR_UTX_FER_EN_POS (6)
#define UART_CR_UTX_FER_EN_LEN (1)
#define UART_CR_UTX_FER_EN_MSK \
  (((1 << UART_CR_UTX_FER_EN_LEN) - 1) << UART_CR_UTX_FER_EN_POS)
#define UART_CR_UTX_FER_EN_UMSK \
  (~(((1 << UART_CR_UTX_FER_EN_LEN) - 1) << UART_CR_UTX_FER_EN_POS))
#define UART_CR_URX_FER_EN     UART_CR_URX_FER_EN
#define UART_CR_URX_FER_EN_POS (7)
#define UART_CR_URX_FER_EN_LEN (1)
#define UART_CR_URX_FER_EN_MSK \
  (((1 << UART_CR_URX_FER_EN_LEN) - 1) << UART_CR_URX_FER_EN_POS)
#define UART_CR_URX_FER_EN_UMSK \
  (~(((1 << UART_CR_URX_FER_EN_LEN) - 1) << UART_CR_URX_FER_EN_POS))

/* 0x30 : uart_status */

#define UART_STATUS_OFFSET        (0x30)
#define UART_STS_UTX_BUS_BUSY     UART_STS_UTX_BUS_BUSY
#define UART_STS_UTX_BUS_BUSY_POS (0)
#define UART_STS_UTX_BUS_BUSY_LEN (1)
#define UART_STS_UTX_BUS_BUSY_MSK \
  (((1 << UART_STS_UTX_BUS_BUSY_LEN) - 1) << UART_STS_UTX_BUS_BUSY_POS)
#define UART_STS_UTX_BUS_BUSY_UMSK \
  (~(((1 << UART_STS_UTX_BUS_BUSY_LEN) - 1) << UART_STS_UTX_BUS_BUSY_POS))
#define UART_STS_URX_BUS_BUSY     UART_STS_URX_BUS_BUSY
#define UART_STS_URX_BUS_BUSY_POS (1)
#define UART_STS_URX_BUS_BUSY_LEN (1)
#define UART_STS_URX_BUS_BUSY_MSK \
  (((1 << UART_STS_URX_BUS_BUSY_LEN) - 1) << UART_STS_URX_BUS_BUSY_POS)
#define UART_STS_URX_BUS_BUSY_UMSK \
  (~(((1 << UART_STS_URX_BUS_BUSY_LEN) - 1) << UART_STS_URX_BUS_BUSY_POS))

/* 0x34 : sts_urx_abr_prd */

#define UART_STS_URX_ABR_PRD_OFFSET    (0x34)
#define UART_STS_URX_ABR_PRD_START     UART_STS_URX_ABR_PRD_START
#define UART_STS_URX_ABR_PRD_START_POS (0)
#define UART_STS_URX_ABR_PRD_START_LEN (16)
#define UART_STS_URX_ABR_PRD_START_MSK \
  (((1 << UART_STS_URX_ABR_PRD_START_LEN) - 1) \
   << UART_STS_URX_ABR_PRD_START_POS)
#define UART_STS_URX_ABR_PRD_START_UMSK \
  (~(((1 << UART_STS_URX_ABR_PRD_START_LEN) - 1) \
     << UART_STS_URX_ABR_PRD_START_POS))
#define UART_STS_URX_ABR_PRD_0X55     UART_STS_URX_ABR_PRD_0X55
#define UART_STS_URX_ABR_PRD_0X55_POS (16)
#define UART_STS_URX_ABR_PRD_0X55_LEN (16)
#define UART_STS_URX_ABR_PRD_0X55_MSK \
  (((1 << UART_STS_URX_ABR_PRD_0X55_LEN) - 1) \
   << UART_STS_URX_ABR_PRD_0X55_POS)
#define UART_STS_URX_ABR_PRD_0X55_UMSK \
  (~(((1 << UART_STS_URX_ABR_PRD_0X55_LEN) - 1) \
     << UART_STS_URX_ABR_PRD_0X55_POS))

/* 0x80 : uart_fifo_config_0 */

#define UART_FIFO_CONFIG_0_OFFSET (0x80)
#define UART_DMA_TX_EN            UART_DMA_TX_EN
#define UART_DMA_TX_EN_POS        (0)
#define UART_DMA_TX_EN_LEN        (1)
#define UART_DMA_TX_EN_MSK \
  (((1 << UART_DMA_TX_EN_LEN) - 1) << UART_DMA_TX_EN_POS)
#define UART_DMA_TX_EN_UMSK \
  (~(((1 << UART_DMA_TX_EN_LEN) - 1) << UART_DMA_TX_EN_POS))
#define UART_DMA_RX_EN     UART_DMA_RX_EN
#define UART_DMA_RX_EN_POS (1)
#define UART_DMA_RX_EN_LEN (1)
#define UART_DMA_RX_EN_MSK \
  (((1 << UART_DMA_RX_EN_LEN) - 1) << UART_DMA_RX_EN_POS)
#define UART_DMA_RX_EN_UMSK \
  (~(((1 << UART_DMA_RX_EN_LEN) - 1) << UART_DMA_RX_EN_POS))
#define UART_TX_FIFO_CLR     UART_TX_FIFO_CLR
#define UART_TX_FIFO_CLR_POS (2)
#define UART_TX_FIFO_CLR_LEN (1)
#define UART_TX_FIFO_CLR_MSK \
  (((1 << UART_TX_FIFO_CLR_LEN) - 1) << UART_TX_FIFO_CLR_POS)
#define UART_TX_FIFO_CLR_UMSK \
  (~(((1 << UART_TX_FIFO_CLR_LEN) - 1) << UART_TX_FIFO_CLR_POS))
#define UART_RX_FIFO_CLR     UART_RX_FIFO_CLR
#define UART_RX_FIFO_CLR_POS (3)
#define UART_RX_FIFO_CLR_LEN (1)
#define UART_RX_FIFO_CLR_MSK \
  (((1 << UART_RX_FIFO_CLR_LEN) - 1) << UART_RX_FIFO_CLR_POS)
#define UART_RX_FIFO_CLR_UMSK \
  (~(((1 << UART_RX_FIFO_CLR_LEN) - 1) << UART_RX_FIFO_CLR_POS))
#define UART_TX_FIFO_OVERFLOW     UART_TX_FIFO_OVERFLOW
#define UART_TX_FIFO_OVERFLOW_POS (4)
#define UART_TX_FIFO_OVERFLOW_LEN (1)
#define UART_TX_FIFO_OVERFLOW_MSK \
  (((1 << UART_TX_FIFO_OVERFLOW_LEN) - 1) << UART_TX_FIFO_OVERFLOW_POS)
#define UART_TX_FIFO_OVERFLOW_UMSK \
  (~(((1 << UART_TX_FIFO_OVERFLOW_LEN) - 1) << UART_TX_FIFO_OVERFLOW_POS))
#define UART_TX_FIFO_UNDERFLOW     UART_TX_FIFO_UNDERFLOW
#define UART_TX_FIFO_UNDERFLOW_POS (5)
#define UART_TX_FIFO_UNDERFLOW_LEN (1)
#define UART_TX_FIFO_UNDERFLOW_MSK \
  (((1 << UART_TX_FIFO_UNDERFLOW_LEN) - 1) << UART_TX_FIFO_UNDERFLOW_POS)
#define UART_TX_FIFO_UNDERFLOW_UMSK \
  (~(((1 << UART_TX_FIFO_UNDERFLOW_LEN) - 1) << UART_TX_FIFO_UNDERFLOW_POS))
#define UART_RX_FIFO_OVERFLOW     UART_RX_FIFO_OVERFLOW
#define UART_RX_FIFO_OVERFLOW_POS (6)
#define UART_RX_FIFO_OVERFLOW_LEN (1)
#define UART_RX_FIFO_OVERFLOW_MSK \
  (((1 << UART_RX_FIFO_OVERFLOW_LEN) - 1) << UART_RX_FIFO_OVERFLOW_POS)
#define UART_RX_FIFO_OVERFLOW_UMSK \
  (~(((1 << UART_RX_FIFO_OVERFLOW_LEN) - 1) << UART_RX_FIFO_OVERFLOW_POS))
#define UART_RX_FIFO_UNDERFLOW     UART_RX_FIFO_UNDERFLOW
#define UART_RX_FIFO_UNDERFLOW_POS (7)
#define UART_RX_FIFO_UNDERFLOW_LEN (1)
#define UART_RX_FIFO_UNDERFLOW_MSK \
  (((1 << UART_RX_FIFO_UNDERFLOW_LEN) - 1) << UART_RX_FIFO_UNDERFLOW_POS)
#define UART_RX_FIFO_UNDERFLOW_UMSK \
  (~(((1 << UART_RX_FIFO_UNDERFLOW_LEN) - 1) << UART_RX_FIFO_UNDERFLOW_POS))

/* 0x84 : uart_fifo_config_1 */

#define UART_FIFO_CONFIG_1_OFFSET (0x84)
#define UART_TX_FIFO_CNT          UART_TX_FIFO_CNT
#define UART_TX_FIFO_CNT_POS      (0)
#define UART_TX_FIFO_CNT_LEN      (6)
#define UART_TX_FIFO_CNT_MSK \
  (((1 << UART_TX_FIFO_CNT_LEN) - 1) << UART_TX_FIFO_CNT_POS)
#define UART_TX_FIFO_CNT_UMSK \
  (~(((1 << UART_TX_FIFO_CNT_LEN) - 1) << UART_TX_FIFO_CNT_POS))
#define UART_RX_FIFO_CNT     UART_RX_FIFO_CNT
#define UART_RX_FIFO_CNT_POS (8)
#define UART_RX_FIFO_CNT_LEN (6)
#define UART_RX_FIFO_CNT_MSK \
  (((1 << UART_RX_FIFO_CNT_LEN) - 1) << UART_RX_FIFO_CNT_POS)
#define UART_RX_FIFO_CNT_UMSK \
  (~(((1 << UART_RX_FIFO_CNT_LEN) - 1) << UART_RX_FIFO_CNT_POS))
#define UART_TX_FIFO_TH     UART_TX_FIFO_TH
#define UART_TX_FIFO_TH_POS (16)
#define UART_TX_FIFO_TH_LEN (5)
#define UART_TX_FIFO_TH_MSK \
  (((1 << UART_TX_FIFO_TH_LEN) - 1) << UART_TX_FIFO_TH_POS)
#define UART_TX_FIFO_TH_UMSK \
  (~(((1 << UART_TX_FIFO_TH_LEN) - 1) << UART_TX_FIFO_TH_POS))
#define UART_RX_FIFO_TH     UART_RX_FIFO_TH
#define UART_RX_FIFO_TH_POS (24)
#define UART_RX_FIFO_TH_LEN (5)
#define UART_RX_FIFO_TH_MSK \
  (((1 << UART_RX_FIFO_TH_LEN) - 1) << UART_RX_FIFO_TH_POS)
#define UART_RX_FIFO_TH_UMSK \
  (~(((1 << UART_RX_FIFO_TH_LEN) - 1) << UART_RX_FIFO_TH_POS))

/* 0x88 : uart_fifo_wdata */

#define UART_FIFO_WDATA_OFFSET (0x88)
#define UART_FIFO_WDATA        UART_FIFO_WDATA
#define UART_FIFO_WDATA_POS    (0)
#define UART_FIFO_WDATA_LEN    (8)
#define UART_FIFO_WDATA_MSK \
  (((1 << UART_FIFO_WDATA_LEN) - 1) << UART_FIFO_WDATA_POS)
#define UART_FIFO_WDATA_UMSK \
  (~(((1 << UART_FIFO_WDATA_LEN) - 1) << UART_FIFO_WDATA_POS))

/* 0x8C : uart_fifo_rdata */

#define UART_FIFO_RDATA_OFFSET (0x8C)
#define UART_FIFO_RDATA        UART_FIFO_RDATA
#define UART_FIFO_RDATA_POS    (0)
#define UART_FIFO_RDATA_LEN    (8)
#define UART_FIFO_RDATA_MSK \
  (((1 << UART_FIFO_RDATA_LEN) - 1) << UART_FIFO_RDATA_POS)
#define UART_FIFO_RDATA_UMSK \
  (~(((1 << UART_FIFO_RDATA_LEN) - 1) << UART_FIFO_RDATA_POS))

/* UART port type definition */

#define UART0_ID    0 /* UART0 port define */
#define UART1_ID    1 /* UART1 port define */
#define UART_ID_MAX 2 /* UART MAX ID define */

/* UART parity type definition */

#define UART_PARITY_NONE 0 /* UART parity none define */
#define UART_PARITY_ODD  1 /* UART parity odd define */
#define UART_PARITY_EVEN 2 /* UART parity even define */

/* UART data bits type definiton */

#define UART_DATABITS_5 0 /* UART data bits length:5 bits */
#define UART_DATABITS_6 1 /* UART data bits length:6 bits */
#define UART_DATABITS_7 2 /* UART data bits length:7 bits */
#define UART_DATABITS_8 3 /* UART data bits length:8 bits */

/* UART stop bits type definiton */

#define UART_STOPBITS_1   0 /* UART data stop bits length:1 bits */
#define UART_STOPBITS_1_5 1 /* UART data stop bits length:1.5 bits */
#define UART_STOPBITS_2   2 /* UART data stop bits length:2 bits */

/* UART each data byte is send out LSB-first or MSB-first type definiton */

#define UART_LSB_FIRST 0 /* UART each byte is send out LSB-first */
#define UART_MSB_FIRST 1 /* UART each byte is send out MSB-first */

/* UART auto baudrate detection using codeword 0x55 */

#define UART_AUTOBAUD_0X55 0

/* UART auto baudrate detection using start bit */

#define UART_AUTOBAUD_STARTBIT 1

/* UART interrupt type definition */

#define UART_INT_TX_END 0 /* UART tx transfer end interrupt */
#define UART_INT_RX_END 1 /* UART rx transfer end interrupt */
#define UART_INT_TX_FIFO_REQ \
  2 /* UART tx fifo interrupt when tx fifo count \
     * reaches,auto clear */
#define UART_INT_RX_FIFO_REQ \
  3                    /* UART rx fifo interrupt when rx fifo count \
                        * reaches,auto clear */
#define UART_INT_RTO 4 /* UART rx time-out interrupt */
#define UART_INT_PCE 5 /* UART rx parity check error interrupt */
#define UART_INT_TX_FER \
  6 /* UART tx fifo overflow/underflow error interrupt */
#define UART_INT_RX_FER \
  7                    /* UART rx fifo overflow/underflow error interrupt */
#define UART_INT_ALL 8 /* All the interrupt */

/* UART overflow or underflow type definition */

#define UART_TX_OVERFLOW  0 /* UART tx fifo overflow */
#define UART_TX_UNDERFLOW 1 /* UART tx fifo underflow */
#define UART_RX_OVERFLOW  2 /* UART rx fifo overflow */
#define UART_RX_UNDERFLOW 3 /* UART rx fifo underflow */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_UART_H */
