/****************************************************************************
 * arch/misoc/src/common/hw/flags.h
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

#ifndef __ARCH_MISOC_SRC_COMMON_HW_FLAGS_H
#define __ARCH_MISOC_SRC_COMMON_HW_FLAGS_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define UART_EV_TX                      0x1
#define UART_EV_RX                      0x2

#define DFII_CONTROL_SEL                0x01
#define DFII_CONTROL_CKE                0x02
#define DFII_CONTROL_ODT                0x04
#define DFII_CONTROL_RESET_N            0x08

#define DFII_COMMAND_CS                 0x01
#define DFII_COMMAND_WE                 0x02
#define DFII_COMMAND_CAS                0x04
#define DFII_COMMAND_RAS                0x08
#define DFII_COMMAND_WRDATA             0x10
#define DFII_COMMAND_RDDATA             0x20

#define ETHMAC_EV_SRAM_WRITER           0x1
#define ETHMAC_EV_SRAM_READER           0x1

#define CLKGEN_STATUS_BUSY              0x1
#define CLKGEN_STATUS_PROGDONE          0x2
#define CLKGEN_STATUS_LOCKED            0x4

#define DVISAMPLER_TOO_LATE             0x1
#define DVISAMPLER_TOO_EARLY            0x2

#define DVISAMPLER_DELAY_MASTER_CAL     0x01
#define DVISAMPLER_DELAY_MASTER_RST     0x02
#define DVISAMPLER_DELAY_SLAVE_CAL      0x04
#define DVISAMPLER_DELAY_SLAVE_RST      0x08
#define DVISAMPLER_DELAY_INC            0x10
#define DVISAMPLER_DELAY_DEC            0x20

#define DVISAMPLER_SLOT_EMPTY           0
#define DVISAMPLER_SLOT_LOADED          1
#define DVISAMPLER_SLOT_PENDING         2

#endif /* __ARCH_MISOC_SRC_COMMON_HW_FLAGS_H */
