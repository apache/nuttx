/****************************************************************************
 * drivers/wireless/spirit/include/spirit_spi.h
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

#ifndef __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_SPI_H
#define __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "spirit_types.h"

/****************************************************************************
 * Pre-processor Defintiions
 ****************************************************************************/

/* SPIRIT1 SPI Headers */

#define HEADER_WRITE_MASK     0x00  /* Write mask for header byte */
#define HEADER_READ_MASK      0x01  /* Read mask for header byte */
#define HEADER_ADDRESS_MASK   0x00  /* Address mask for header byte */
#define HEADER_COMMAND_MASK   0x80  /* Command mask for header byte */
#define LINEAR_FIFO_ADDRESS   0xff  /* Linear FIFO address */

/* SPIRIT macros to construct headers */

#define __MKHEADER(a,rw)      (a | rw)
#define WRITE_HEADER          __MKHEADER(HEADER_ADDRESS_MASK, HEADER_WRITE_MASK)
#define READ_HEADER           __MKHEADER(HEADER_ADDRESS_MASK, HEADER_READ_MASK)
#define COMMAND_HEADER        __MKHEADER(HEADER_COMMAND_MASK, HEADER_WRITE_MASK)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

struct spi_dev_s; /* Forward reference */

/****************************************************************************
 * Name: spirit_reg_read
 *
 * Description:
 *   Read single or multiple SPIRIT1 register
 *
 * Input Parameters:
 *   regaddr: Base register's address to be read
 *   buffer:  Pointer to the buffer of registers' values to be read
 *   buflen:  Number of register values to be read
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned on
 *   any failure.  On success, spirit->state is updated.
 *
 ****************************************************************************/

int spirit_reg_read(FAR struct spirit_library_s *spirit, uint8_t regaddr,
                    FAR uint8_t *buffer, unsigned int buflen);

/****************************************************************************
 * Name: spirit_reg_write
 *
 * Description:
 *   Read single or multiple SPIRIT1 register.
 *
 * Input Parameters:
 *   spirit  - Reference to an instance of the driver state structure.
 *   regaddr - Base register's address to write
 *   buffer  - Pointer to the buffer of register values to write
 *   buflen  - Number of registers values to be written.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned on
 *   any failure.  On success, spirit->state is updated.
 *
 ****************************************************************************/

int spirit_reg_write(FAR struct spirit_library_s *spirit, uint8_t regaddr,
                     FAR const uint8_t *buffer, unsigned int buflen);

/****************************************************************************
 * Name: spirit_reg_modify
 *
 * Description:
 *   Perform atomic read/modify/write on a single SPIRIT1 register.  This is
 *   atomic only in the sense that other accesses to the SPI bus are
 *   prohibited throughout the operation.
 *
 * Input Parameters:
 *   spirit  - Reference to an instance of the driver state structure.
 *   regaddr - Base register's address to write
 *   clrbits - Bits to clear in the register
 *   setbits - Bits to set in the register
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned on
 *   any failure.  On success, spirit->state is updated.
 *
 ****************************************************************************/

int spirit_reg_modify(FAR struct spirit_library_s *spirit, uint8_t regaddr,
                      uint8_t setbits, uint8_t clrbits);

/****************************************************************************
 * Name: spirit_command
 *
 * Description:
 *   Send a command
 *
 * Input Parameters:
 *   spirit - Reference to an instance of the driver state structure.
 *   cmd    - Command code to be sent
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned on
 *   any failure.  On success, spirit->state is updated.
 *
 ****************************************************************************/

int spirit_command(FAR struct spirit_library_s *spirit, uint8_t cmd);

/****************************************************************************
 * Name: spirit_fifo_read
 *
 * Description:
 *   Read data from RX FIFO
 *
 * Input Parameters:
 *   spirit - Reference to an instance of the driver state structure.
 *   buffer - Pointer to the buffer of data values to write
 *   buflen - Number of bytes to be written
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned on
 *   any failure.  On success, spirit->state is updated.
 *
 ****************************************************************************/

int spirit_fifo_read(FAR struct spirit_library_s *spirit,
                     FAR uint8_t *buffer,
                     unsigned int buflen);

/****************************************************************************
 * Name: spirit_fifo_write
 *
 * Description:
 *   Write data into TX FIFO.
 *
 * Input Parameters:
 *   spirit  - Reference to an instance of the driver state structure.
 *   buffer  - Pointer to the buffer of data values to write
 *   buflen  - Number of data values to be written.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned on
 *   any failure.  On success, spirit->state is updated.
 *
 ****************************************************************************/

int spirit_fifo_write(FAR struct spirit_library_s *spirit,
                      FAR const uint8_t *buffer, unsigned int buflen);

/****************************************************************************
 * Name: spirit_update_status
 *
 * Description:
 *   Updates the state field in the driver instance by reading the MC_STATE
 *   register of SPIRIT.
 *
 * Input Parameters:
 *   spirit - Reference to an instance of the driver state structure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned on
 *   any failure.  On success, spirit->state is updated.
 *
 ****************************************************************************/

int spirit_update_status(FAR struct spirit_library_s *spirit);

/****************************************************************************
 * Name: spirit_waitstatus
 *
 * Description:
 *   Poll until the Spirit status is the requested value or until a timeout
 *   occurs.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   state  - That that we are waiting for.
 *   msec   - Timeout in millisedonds
 *
 * Returned Value:
 *   OK on success; a negated errno on a timeout
 *
 * Assumptions:
 *   We have exclusive access to the driver state and to the spirit library.
 *
 ****************************************************************************/

int spirit_waitstatus(FAR struct spirit_library_s *spirit,
                      enum spirit_state_e state, unsigned int msec);

#ifdef __cplusplus
}
#endif

#endif /* __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_SPI_H */
