/****************************************************************************
 * include/nuttx/drivers/1wire.h
 *
 *   Copyright (C) 2016 Aleksandr Vyhovanec. All rights reserved.
 *   Author: Aleksandr Vyhovanec <www.desh@gmail.com>
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
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_DRIVERS_1WIRE_H
#define __INCLUDE_NUTTX_DRIVERS_1WIRE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Supported 1-Wire commands */

#define ONEWIRE_CMD_SEARCH           0xf0
#define ONEWIRE_CMD_ALARM_SEARCH     0xec
#define ONEWIRE_CMD_SKIP_ROM         0xcc
#define ONEWIRE_CMD_COPY_SCRATCHPAD  0x48
#define ONEWIRE_CMD_WRITE_SCRATCHPAD 0x4e
#define ONEWIRE_CMD_READ_SCRATCHPAD  0xbe
#define ONEWIRE_CMD_READ_ROM         0x33
#define ONEWIRE_CMD_MATCH_ROM        0x55
#define ONEWIRE_CMD_RESUME           0xa5

/****************************************************************************
 * Name: ONEWIRE_RESET
 *
 * Description:
 *   Reset pulse and presence detect. Each write operational will be an 'atomic'
 *   operation in the sense that any other 1-Wire actions will be serialized
 *   and pend until this write completes.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

#define ONEWIRE_RESET(d) ((d)->ops->reset(d))

/****************************************************************************
 * Name: ONEWIRE_WRITE
 *
 * Description:
 *   Send a block of data on 1-Wire. Each write operational will be an 'atomic'
 *   operation in the sense that any other 1-Wire actions will be serialized
 *   and pend until this write completes.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   buffer - A pointer to the read-only buffer of data to be written to device
 *   buflen - The number of bytes to send from the buffer
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

#define ONEWIRE_WRITE(d,b,l) ((d)->ops->write(d,b,l))

/****************************************************************************
 * Name: ONEWIRE_READ
 *
 * Description:
 *   Receive a block of data from 1-Wire. Each read operational will be an 'atomic'
 *   operation in the sense that any other 1-Wire actions will be serialized
 *   and pend until this read completes.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   buffer - A pointer to a buffer of data to receive the data from the device
 *   buflen - The requested number of bytes to be read
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

#define ONEWIRE_READ(d,b,l) ((d)->ops->read(d,b,l))

/****************************************************************************
 * Name: ONEWIRE_EXCHANGE
 *
 * Description:
 *   Reset pulse and presence detect, send a block of data and receive a block
 *   of data from 1-Wire. Each write operational will be an 'atomic'
 *   operation in the sense that any other 1-Wire actions will be serialized
 *   and pend until this write completes.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   reset    - Reset pulse and presence detect
 *   txbuffer - A pointer to the read-only buffer of data to be written to device
 *   txbuflen - The number of bytes to send from the buffer
 *   rxbuffer - A pointer to a buffer of data to receive the data from the device
 *   rxbuflen - The requested number of bytes to be read
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

#define ONEWIRE_EXCHANGE(d,r,tx,tl,rx,rl) ((d)->ops->exchange(d,r,tx,tl,rx,rl))

/****************************************************************************
 * Name: ONEWIRE_WRITEBIT
 *
 * Description:
 *   Send a single bit on 1-Wire. Each write operational will be an 'atomic'
 *   operation in the sense that any other 1-Wire actions will be serialized
 *   and pend until this write completes.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   buffer - A pointer to the read-only 1 byte buffer for the bit value
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

#define ONEWIRE_WRITEBIT(d,b) ((d)->ops->writebit(d,b))

/****************************************************************************
 * Name: ONEWIRE_READBIT
 *
 * Description:
 *   Sample a single bit from 1-Wire. Each read operational will be an 'atomic'
 *   operation in the sense that any other 1-Wire actions will be serialized
 *   and pend until this read completes.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   buffer - A pointer to a 1 byte buffer for the bit value
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

#define ONEWIRE_READBIT(d,b) ((d)->ops->readbit(d,b))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The 1-Wire vtable */

struct onewire_dev_s;
struct onewire_ops_s
{
  int    (*reset)(FAR struct onewire_dev_s *dev);
  int    (*write)(FAR struct onewire_dev_s *dev, FAR const uint8_t *buffer,
           int buflen);
  int    (*read)(FAR struct onewire_dev_s *dev, FAR uint8_t *buffer,
           int buflen);
  int    (*exchange)(FAR struct onewire_dev_s *dev, bool reset,
                     FAR const uint8_t *txbuffer, int txbuflen,
                     FAR uint8_t *rxbuffer, int rxbuflen);
  int    (*writebit)(FAR struct onewire_dev_s *dev, FAR const uint8_t *bit);
  int    (*readbit)(FAR struct onewire_dev_s *dev, FAR uint8_t *bit);
};

/* 1-Wire private data. This structure only defines the initial fields of the
 * structure visible to the 1-Wire client. The specific implementation may
 * add additional, device specific fields after the vtable.
 */

struct onewire_dev_s
{
  const struct onewire_ops_s *ops; /* 1-Wire vtable */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* __INCLUDE_NUTTX_DRIVERS_1WIRE_H */
