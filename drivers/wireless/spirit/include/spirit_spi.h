/******************************************************************************
 * include/nuttx/wireless/spirit/include/spirit_spi.h
 * Header file for NuttX SPIRIT SPI driver interface.
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derives loosely from similarly licensed SPI interface definitions from
 * STMicro:
 *
 *   Copyright(c) 2015 STMicroelectronics
 *   Author: VMA division - AMS
 *   Version 3.2.2 08-July-2015
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

#ifndef __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_SPI_H
#define __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_SPI_H

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include "spirit_types.h"

/******************************************************************************
 * Pre-processor Defintiions
 ******************************************************************************/

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

/******************************************************************************
 * Public Function Prototypes
 ******************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

struct spi_dev_s; /* Forward reference */

/******************************************************************************
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
 ******************************************************************************/

int spirit_reg_read(FAR struct spirit_library_s *spirit, uint8_t regaddr,
                    FAR uint8_t *buffer, unsigned int buflen);

/******************************************************************************
 * Name: spirit_reg_write
 *
 * Description:
 *   Read single or multiple SPIRIT1 register.
 *
 * Input Parameters:
 *   spirit  - Reference to an instance of the driver state stucture.
 *   regaddr - Base register's address to write
 *   buffer  - Pointer to the buffer of register values to write
 *   buflen  - Number of registers values to be written.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned on
 *   any failure.  On success, spirit->state is updated.
 *
 ******************************************************************************/

int spirit_reg_write(FAR struct spirit_library_s *spirit, uint8_t regaddr,
                     FAR const uint8_t *buffer, unsigned int buflen);

/******************************************************************************
 * Name: spirit_reg_modify
 *
 * Description:
 *   Perform atomic read/modify/write on a single SPIRIT1 register.  This is
 *   atomic only in the sense that other accesses to the SPI bus are
 *   prohibited throughout the operation.
 *
 * Input Parameters:
 *   spirit  - Reference to an instance of the driver state stucture.
 *   regaddr - Base register's address to write
 *   clrbits - Bits to clear in the register
 *   setbits - Bits to set in the regiser
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned on
 *   any failure.  On success, spirit->state is updated.
 *
 ******************************************************************************/

int spirit_reg_modify(FAR struct spirit_library_s *spirit, uint8_t regaddr,
                      uint8_t setbits, uint8_t clrbits);

/******************************************************************************
 * Name: spirit_command
 *
 * Description:
 *   Send a command
 *
 * Input Parameters:
 *   spirit - Reference to an instance of the driver state stucture.
 *   cmd    - Command code to be sent
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned on
 *   any failure.  On success, spirit->state is updated.
 *
 ******************************************************************************/

int spirit_command(FAR struct spirit_library_s *spirit, uint8_t cmd);

/******************************************************************************
 * Name: spirit_fifo_read
 *
 * Description:
 *   Read data from RX FIFO
 *
 * Input Parameters:
 *   spirit - Reference to an instance of the driver state stucture.
 *   buffer - Pointer to the buffer of data values to write
 *   buflen - Number of bytes to be written
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned on
 *   any failure.  On success, spirit->state is updated.
 *
 ******************************************************************************/

int spirit_fifo_read(FAR struct spirit_library_s *spirit, FAR uint8_t *buffer,
                     unsigned int buflen);

/******************************************************************************
 * Name: spirit_fifo_write
 *
 * Description:
 *   Write data into TX FIFO.
 *
 * Input Parameters:
 *   spirit  - Reference to an instance of the driver state stucture.
 *   buffer  - Pointer to the buffer of data values to write
 *   buflen  - Number of data values to be written.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned on
 *   any failure.  On success, spirit->state is updated.
 *
 ******************************************************************************/

int spirit_fifo_write(FAR struct spirit_library_s *spirit,
                      FAR const uint8_t *buffer, unsigned int buflen);

/******************************************************************************
 * Name: spirit_update_status
 *
 * Description:
 *   Updates the state field in the driver instance by reading the MC_STATE
 *   register of SPIRIT.
 *
 * Input Parameters:
 *   spirit - Reference to an instance of the driver state stucture.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned on
 *   any failure.  On success, spirit->state is updated.
 *
 ******************************************************************************/

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
