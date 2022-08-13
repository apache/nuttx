/****************************************************************************
 * arch/arm/src/imxrt/imxrt_flexspi.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT_FLEXSPI_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT_FLEXSPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "chip.h"
#include "imxrt_config.h"

#ifdef CONFIG_IMXRT_FLEXSPI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LUT - LUT 0..LUT 63 */

#define FLEXSPI_LUT_OPERAND0_MASK                (0xffU)
#define FLEXSPI_LUT_OPERAND0_SHIFT               (0U)

/* OPERAND0 */

#define FLEXSPI_LUT_OPERAND0(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXSPI_LUT_OPERAND0_SHIFT)) & FLEXSPI_LUT_OPERAND0_MASK)
#define FLEXSPI_LUT_NUM_PADS0_MASK               (0x300U)
#define FLEXSPI_LUT_NUM_PADS0_SHIFT              (8U)

/* NUM_PADS0 */

#define FLEXSPI_LUT_NUM_PADS0(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXSPI_LUT_NUM_PADS0_SHIFT)) & FLEXSPI_LUT_NUM_PADS0_MASK)
#define FLEXSPI_LUT_OPCODE0_MASK                 (0xfc00U)
#define FLEXSPI_LUT_OPCODE0_SHIFT                (10U)

/* OPCODE0 */

#define FLEXSPI_LUT_OPCODE0(x)                   (((uint32_t)(((uint32_t)(x)) << FLEXSPI_LUT_OPCODE0_SHIFT)) & FLEXSPI_LUT_OPCODE0_MASK)
#define FLEXSPI_LUT_OPERAND1_MASK                (0xff0000U)
#define FLEXSPI_LUT_OPERAND1_SHIFT               (16U)

/* OPERAND1 */

#define FLEXSPI_LUT_OPERAND1(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXSPI_LUT_OPERAND1_SHIFT)) & FLEXSPI_LUT_OPERAND1_MASK)
#define FLEXSPI_LUT_NUM_PADS1_MASK               (0x3000000U)
#define FLEXSPI_LUT_NUM_PADS1_SHIFT              (24U)

/* NUM_PADS1 - NUM_PADS1 */

#define FLEXSPI_LUT_NUM_PADS1(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXSPI_LUT_NUM_PADS1_SHIFT)) & FLEXSPI_LUT_NUM_PADS1_MASK)
#define FLEXSPI_LUT_OPCODE1_MASK                 (0xfc000000U)
#define FLEXSPI_LUT_OPCODE1_SHIFT                (26U)

/* OPCODE1 */

#define FLEXSPI_LUT_OPCODE1(x)                   (((uint32_t)(((uint32_t)(x)) << FLEXSPI_LUT_OPCODE1_SHIFT)) & FLEXSPI_LUT_OPCODE1_MASK)

/* Formula to form FLEXSPI instructions in LUT table */

#define FLEXSPI_LUT_SEQ(cmd0, pad0, op0, cmd1, pad1, op1) \
    (FLEXSPI_LUT_OPERAND0(op0) | FLEXSPI_LUT_NUM_PADS0(pad0) | \
     FLEXSPI_LUT_OPCODE0(cmd0) | FLEXSPI_LUT_OPERAND1(op1) | \
     FLEXSPI_LUT_NUM_PADS1(pad1) | FLEXSPI_LUT_OPCODE1(cmd1))

/* Access macros ************************************************************/

/****************************************************************************
 * Name: FLEXSPI_LOCK
 *
 * Description:
 *   On FlexSPI buses where there are multiple devices, it will be necessary
 *   to lock FlexSPI to have exclusive access to the buses for a sequence of
 *   transfers.  The bus should be locked before the chip is selected.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   lock - true: Lock FlexSPI bus, false: unlock FlexSPI bus
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#define FLEXSPI_LOCK(d,l) (d)->ops->lock(d,l)

/****************************************************************************
 * Name: FLEXSPI_TRANSFER
 *
 * Description:
 *   Perform one FlexSPI transfer
 *
 * Input Parameters:
 *   dev     - Device-specific state data
 *   xfer    - Describes the transfer to be performed.
 *
 * Returned Value:
 *   0 on SUCCESS, STATUS_FLEXSPI_SEQUENCE_EXECUTION_TIMEOUT,
 *   STATUS_FLEXSPI_IP_COMMAND_SEQUENCE_ERROR or
 *   STATUS_FLEXSPI_IP_COMMAND_GRANT_TIMEOUT otherwise
 *
 ****************************************************************************/

#define FLEXSPI_TRANSFER(d,x) (d)->ops->transfer_blocking(d,x)

/****************************************************************************
 * Name: FLEXSPI_SOFTWARE_RESET
 *
 * Description:
 *   Perform FlexSPI software reset
 *
 * Input Parameters:
 *   dev     - Device-specific state data
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

#define FLEXSPI_SOFTWARE_RESET(d) (d)->ops->software_reset(d)

/****************************************************************************
 * Name: FLEXSPI_UPDATE_LUT
 *
 * Description:
 *   Perform FlexSPI LUT table update
 *
 * Input Parameters:
 *   dev     - Device-specific state data
 *   index   - Index start to update
 *   cmd     - Command array
 *   count   - Size of the array
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

#define FLEXSPI_UPDATE_LUT(d,i,c,n) (d)->ops->update_lut(d,i,c,n)

/****************************************************************************
 * Name: FLEXSPI_SET_DEVICE_CONFIG
 *
 * Description:
 *   Perform FlexSPI device config
 *
 * Input Parameters:
 *   dev     - Device-specific state data
 *   config  - Config data for external device
 *   port    - Port
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

#define FLEXSPI_SET_DEVICE_CONFIG(d,c,p) (d)->ops->set_device_config(d,c,p)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* CMD definition of FLEXSPI, use to form LUT instruction, flexspi_command */

enum
{
  FLEXSPI_COMMAND_STOP           = 0x00,  /* Stop execution, deassert CS */
  FLEXSPI_COMMAND_SDR            = 0x01,  /* Transmit Command code to Flash,
                                           * using SDR mode.
                                           */

  FLEXSPI_COMMAND_RADDR_SDR      = 0x02,  /* Transmit Row Address to Flash,
                                           * using SDR mode.
                                           */

  FLEXSPI_COMMAND_CADDR_SDR      = 0x03,  /* Transmit Column Address to
                                           * Flash, using SDR mode.
                                           */

  FLEXSPI_COMMAND_MODE1_SDR      = 0x04,  /* Transmit 1-bit Mode bits to
                                           * Flash, using SDR mode.
                                           */

  FLEXSPI_COMMAND_MODE2_SDR      = 0x05,  /* Transmit 2-bit Mode bits to
                                           * Flash, using SDR mode.
                                           */

  FLEXSPI_COMMAND_MODE4_SDR      = 0x06,  /* Transmit 4-bit Mode bits to
                                           * Flash, using SDR mode.
                                           */

  FLEXSPI_COMMAND_MODE8_SDR      = 0x07,  /* Transmit 8-bit Mode bits to
                                           * Flash, using SDR mode.
                                           */

  FLEXSPI_COMMAND_WRITE_SDR      = 0x08,  /* Transmit Programming Data to
                                           * Flash, using SDR mode.
                                           */

  FLEXSPI_COMMAND_READ_SDR       = 0x09,  /* Receive Read Data from Flash,
                                           * using SDR mode.
                                           */

  FLEXSPI_COMMAND_LEARN_SDR      = 0x0a,  /* Receive Read Data or Preamble
                                           * bit from Flash, SDR mode.
                                           */

  FLEXSPI_COMMAND_DATSZ_SDR      = 0x0b,  /* Transmit Read/Program Data size
                                           * (byte) to Flash, SDR mode.
                                           */

  FLEXSPI_COMMAND_DUMMY_SDR      = 0x0c,  /* Leave data lines undriven by
                                           * FlexSPI controller.
                                           */

  FLEXSPI_COMMAND_DUMMY_RWDS_SDR = 0x0d,  /* Leave data lines undriven by
                                           * FlexSPI controller, dummy cycles
                                           * decided by RWDS.
                                           */

  FLEXSPI_COMMAND_DDR            = 0x21,  /* Transmit Command code to Flash,
                                           * using DDR mode.
                                           */

  FLEXSPI_COMMAND_RADDR_DDR      = 0x22,  /* Transmit Row Address to Flash,
                                           * using DDR mode.
                                           */

  FLEXSPI_COMMAND_CADDR_DDR      = 0x23,  /* Transmit Column Address to
                                           * Flash, using DDR mode.
                                           */

  FLEXSPI_COMMAND_MODE1_DDR      = 0x24,  /* Transmit 1-bit Mode bits to
                                           * Flash, using DDR mode.
                                           */

  FLEXSPI_COMMAND_MODE2_DDR      = 0x25,  /* Transmit 2-bit Mode bits to
                                           * Flash, using DDR mode.
                                           */

  FLEXSPI_COMMAND_MODE4_DDR      = 0x26,  /* Transmit 4-bit Mode bits to
                                           * Flash, using DDR mode.
                                           */

  FLEXSPI_COMMAND_MODE8_DDR      = 0x27,  /* Transmit 8-bit Mode bits to
                                           * Flash, using DDR mode.
                                           */

  FLEXSPI_COMMAND_WRITE_DDR      = 0x28,  /* Transmit Programming Data to
                                           * Flash, using DDR mode.
                                           */

  FLEXSPI_COMMAND_READ_DDR       = 0x29,  /* Receive Read Data from Flash,
                                           * using DDR mode.
                                           */

  FLEXSPI_COMMAND_LEARN_DDR      = 0x2a,  /* Receive Read Data or Preamble
                                           * bit from Flash, DDR mode.
                                           */

  FLEXSPI_COMMAND_DATSZ_DDR      = 0x2b,  /* Transmit Read/Program Data size
                                           * (byte) to Flash, DDR mode.
                                           */

  FLEXSPI_COMMAND_DUMMY_DDR      = 0x2c,  /* Leave data lines undriven by
                                           * FlexSPI controller.
                                           */

  FLEXSPI_COMMAND_DUMMY_RWDS_DDR = 0x2d,  /* Leave data lines undriven by
                                           * FlexSPI controller, dummy cycles
                                           * decided by RWDS.
                                           */

  FLEXSPI_COMMAND_JUMP_ON_CS     = 0x1f,  /* Stop execution, deassert CS and
                                           * save operand[7:0] as the
                                           * instruction start pointer for
                                           * next sequence
                                           */
};

/* Pad definition of FLEXSPI, use to form LUT instruction */

enum flexspi_pad_e
{
  FLEXSPI_1PAD = 0x00,  /* Transmit command/address and transmit/receive data
                         * only through DATA0/DATA1.
                         */

  FLEXSPI_2PAD = 0x01,  /* Transmit command/address and transmit/receive data
                         * only through DATA[1:0].
                         */

  FLEXSPI_4PAD = 0x02,  /* Transmit command/address and transmit/receive data
                         * only through DATA[3:0].
                         */

  FLEXSPI_8PAD = 0x03,  /* Transmit command/address and transmit/receive data
                         * only through DATA[7:0].
                         */
};

/* FLEXSPI operation port select */

enum flexspi_port_e
{
  FLEXSPI_PORT_A1 = 0x0,  /* Access flash on A1 port */
  FLEXSPI_PORT_A2,        /* Access flash on A2 port */
  FLEXSPI_PORT_B1,        /* Access flash on B1 port */
  FLEXSPI_PORT_B2,        /* Access flash on B2 port */
  FLEXSPI_PORT_COUNT
};

/* Command type */

enum flexspi_command_type_e
{
  FLEXSPI_COMMAND, /* FlexSPI operation: Only command, both TX and Rx buffer
                    * are ignored.
                    */

  FLEXSPI_CONFIG,  /* FlexSPI operation: Configure device mode, the TX fifo
                    * size is fixed in LUT.
                    */

  FLEXSPI_READ,    /* FlexSPI operation: Read, only Rx Buffer is
                    * effective.
                    */

  FLEXSPI_WRITE,   /* FlexSPI operation: Read, only Tx Buffer is
                    * effective.
                    */
};

/* Status structure of FLEXSPI */

enum
{
  STATUS_FLEXSPI_BUSY = 0,                        /* FLEXSPI is busy */

  STATUS_FLEXSPI_SEQUENCE_EXECUTION_TIMEOUT = 1,  /* Sequence execution
                                                   * timeout error occurred
                                                   * during FLEXSPI transfer.
                                                   */

  STATUS_FLEXSPI_IP_COMMAND_SEQUENCE_ERROR = 2,   /* IP command Sequence
                                                   * execution timeout error
                                                   * occurred during FLEXSPI
                                                   * transfer.
                                                   */

  STATUS_FLEXSPI_IP_COMMAND_GRANT_TIMEOUT = 3,    /* IP command grant timeout
                                                   * error occurred during
                                                   * FLEXSPI transfer.
                                                   */
};

/* Transfer structure for FLEXSPI */

struct flexspi_transfer_s
{
  uint32_t device_address;              /* Operation device address */
  enum flexspi_port_e port;             /* Operation port */
  enum flexspi_command_type_e cmd_type; /* Execution command type */
  uint8_t seq_index;                    /* Sequence ID for command */
  uint8_t seq_number;                   /* Sequence number for command */
  uint32_t *data;                       /* Data buffer */
  size_t data_size;                     /* Data size in bytes */
};

/* FLEXSPI interval unit for flash device select */

enum flexspi_cs_interval_cycle_unit_e
{
  FLEXSPI_CS_INTERVAL_UNIT1_SCK_CYCLE   = 0x0,  /* Chip selection interval:
                                                 * CSINTERVAL * 1 serial
                                                 * clock cycle.
                                                 */

  FLEXSPI_CS_INTERVAL_UNIT256_SCK_CYCLE = 0x1,  /* Chip selection interval:
                                                 * CSINTERVAL * 256 serial
                                                 * clock cycle.
                                                 */
};

/* FLEXSPI AHB wait interval unit for writing */

enum flexspi_ahb_write_wait_unit_e
{
  FLEXSPI_AHB_WRITE_WAIT_UNIT2_AHB_CYCLE     = 0x0,  /* AWRWAIT unit is 2
                                                      * ahb clock cycle.
                                                      */

  FLEXSPI_AHB_WRITE_WAIT_UNIT8_AHB_CYCLE     = 0x1,  /* AWRWAIT unit is 8
                                                      * ahb clock cycle.
                                                      */

  FLEXSPI_AHB_WRITE_WAIT_UNIT32_AHB_CYCLE    = 0x2,  /* AWRWAIT unit is 32
                                                      * ahb clock cycle.
                                                      */

  FLEXSPI_AHB_WRITE_WAIT_UNIT128_AHB_CYCLE   = 0x3,  /* AWRWAIT unit is 128
                                                      * ahb clock cycle.
                                                      */

  FLEXSPI_AHB_WRITE_WAIT_UNIT512_AHB_CYCLE   = 0x4,  /* AWRWAIT unit is 512
                                                      * ahb clock cycle.
                                                      */

  FLEXSPI_AHB_WRITE_WAIT_UNIT2048_AHB_CYCLE  = 0x5,  /* AWRWAIT unit is 2048
                                                      * ahb clock cycle.
                                                      */

  FLEXSPI_AHB_WRITE_WAIT_UNIT8192_AHB_CYCLE  = 0x6,  /* AWRWAIT unit is 8192
                                                      * ahb clock cycle.
                                                      */

  FLEXSPI_AHB_WRITE_WAIT_UNIT32768_AHB_CYCLE = 0x7,  /* AWRWAIT unit is 32768
                                                      * ahb clock cycle.
                                                      */
};

/* External device configuration items */

struct flexspi_device_config_s
{
  uint32_t flexspi_root_clk; /* FLEXSPI serial root clock */
  bool is_sck2_enabled;      /* FLEXSPI use SCK2 */
  uint32_t flash_size;       /* Flash size in KByte */

  enum flexspi_cs_interval_cycle_unit_e cs_interval_unit; /* CS interval unit, 1
                                                      * or 256 cycle.
                                                      */

  uint16_t cs_interval;     /* CS line assert interval, multiply CS
                             * interval unit to get the CS line assert
                             * interval cycles.
                             */

  uint8_t cs_hold_time;     /* CS line hold time */
  uint8_t cs_setup_time;    /* CS line setup time */
  uint8_t data_valid_time;  /* Data valid time for external device */
  uint8_t columnspace;      /* Column space size */
  bool enable_word_address; /* If enable word address */
  uint8_t awr_seq_index;    /* Sequence ID for AHB write command */
  uint8_t awr_seq_number;   /* Sequence number for AHB write command */
  uint8_t ard_seq_index;    /* Sequence ID for AHB read command */
  uint8_t ard_seq_number;   /* Sequence number for AHB read command */

  enum flexspi_ahb_write_wait_unit_e ahb_write_wait_unit;  /* AHB write wait unit */

  uint16_t ahb_write_wait_interval;  /* AHB write wait interval, multiply AHB
                                      * write interval unit to get the AHB
                                      * write wait cycles.
                                      */

  bool enable_write_mask;   /* Enable/Disable FLEXSPI drive DQS pin as write mask
                             * when writing to external device.
                             */
};

/* The FlexSPI vtable */

struct flexspi_dev_s;
struct flexspi_ops_s
{
  int (*lock)(struct flexspi_dev_s *dev, bool lock);
  int (*transfer_blocking)(struct flexspi_dev_s *dev,
                           struct flexspi_transfer_s *xfer);
  void (*software_reset)(struct flexspi_dev_s *dev);
  void (*update_lut)(struct flexspi_dev_s *dev,
                     uint32_t index, const uint32_t *cmd,
                     uint32_t count);
  void (*set_device_config)(struct flexspi_dev_s *dev,
                            struct flexspi_device_config_s *config,
                            enum flexspi_port_e port);
};

/* FlexSPI private data.  This structure only defines the initial fields of
 * the structure visible to the FlexSPI client.  The specific implementation
 * may add additional, device specific fields
 */

struct flexspi_dev_s
{
  const struct flexspi_ops_s *ops;
};

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_flexspi_initialize
 *
 * Description:
 *   Initialize the selected FlexSPI port in master mode
 *
 * Input Parameters:
 *   intf - Interface number(must be zero)
 *
 * Returned Value:
 *   Valid FlexSPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct flexspi_dev_s;
struct flexspi_dev_s *imxrt_flexspi_initialize(int intf);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_IMXRT_FLEXSPI */
#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT_FLEXSPI_H */
