/****************************************************************************
 * arch/arm/src/rzv2h/rzv2h_qspi.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM_SRC_RZV2H_RZV2H_QSPI_H
#define __ARCH_ARM_SRC_RZV2H_RZV2H_QSPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/spi/qspi.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Wire protocol used for memory reads (command-address-data lines) */

enum rzv2h_qspi_protocol_e
{
  RZV2H_QSPI_PROTOCOL_1S1S1S = 0,
  RZV2H_QSPI_PROTOCOL_1S2S2S,
  RZV2H_QSPI_PROTOCOL_1S4S4S,
  RZV2H_QSPI_PROTOCOL_4S4S4S,   /* Needs qpi_enter_cmd/qpi_exit_cmd */
  RZV2H_QSPI_PROTOCOL_4S4D4D    /* Needs qpi_enter_cmd/qpi_exit_cmd */
};

/* Description of the SPI NOR flash attached to the xSPI controller.  The
 * flash type and wiring are board properties, so the board supplies this
 * to rzv2h_qspibus_initialize().
 */

struct rzv2h_qspi_flash_s
{
  uint8_t  chip_select;       /* xSPI0 chip select the flash is on (0/1) */
  uint8_t  memory_mb;         /* Flash capacity in MiB (1,2,4,...,64) */
  uint16_t page_size;         /* Page program size in bytes */
  uint8_t  page_program_cmd;  /* Page program opcode */
  uint8_t  write_enable_cmd;  /* Write enable opcode */
  uint8_t  status_cmd;        /* Read status register opcode */
  uint8_t  read_cmd;          /* Plain read opcode (initial config) */
  uint8_t  write_status_bit;  /* Write-in-progress bit position in status */
  uint8_t  write_enable_bit;  /* Write-enable-latch bit position in status */

  /* Optional read re-encoding.  When read_xlate_from is nonzero, memory
   * reads that the MTD driver issues with that opcode are instead sent
   * as read_xlate_to using read_protocol.  read_dummies < 0 keeps the
   * caller's dummy-cycle count.  For the 4S protocols the flash is put in
   * persistent Quad SPI mode with qpi_enter_cmd around each read and
   * taken out with qpi_exit_cmd.
   */

  uint8_t  read_xlate_from;
  uint8_t  read_xlate_to;
  enum rzv2h_qspi_protocol_e read_protocol;
  int8_t   read_dummies;
  uint8_t  qpi_enter_cmd;
  uint8_t  qpi_exit_cmd;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rzv2h_qspibus_initialize
 *
 * Description:
 *   Initialize the selected xSPI bus for QSPI operation and return a
 *   reference to the QSPI interface.
 *
 * Input Parameters:
 *   intf  - The xSPI controller number (only 0 is supported).
 *   flash - Board-specific description of the attached flash; must
 *           remain valid for the lifetime of the driver.
 *
 * Returned Value:
 *   Valid QSPI device structure reference on success; a NULL on failure.
 *
 ****************************************************************************/

struct qspi_dev_s *rzv2h_qspibus_initialize(int intf,
                                const struct rzv2h_qspi_flash_s *flash);

#endif /* __ARCH_ARM_SRC_RZV2H_RZV2H_QSPI_H */
