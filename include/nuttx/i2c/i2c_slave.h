/****************************************************************************
 * include/nuttx/i2c/i2c_slave.h
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

#ifndef __INCLUDE_NUTTX_I2C_I2C_SLAVE_H
#define __INCLUDE_NUTTX_I2C_I2C_SLAVE_H

/****************************************************************************
 * Using I2C slave mode:
 *
 * After I2C slave mode is initialized by calling an architecture defined
 * initialization function, the hardware will monitor the I2C bus waiting
 * for messages with this device's address.
 *
 * Before I2C data can be received, the I2CS_READ macro should be called
 * to register a buffer where the received data will be stored, and a
 * callback function should be registered with either (not both) the
 * I2CS_REGISTERCALLBACK macro.  When the data is received (via an I2C
 * write message) it will be written to the supplied buffer and the callback
 * function will be called.
 *
 * The I2C_WRITE macro is used to register a buffer with data to be
 * sent when the master next issues an I2C read message.  There is no
 * specific notification that the data has been read, but since usual
 * I2C operation is for the master to transmit a message indicating
 * the desired data before reading, the slave should register the
 * return data in the callback function and preserve it until the next
 * callback it received.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#ifdef CONFIG_I2C_SLAVE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C address calculation.  Convert 7- and 10-bit address to 8-bit and
 * 16-bit read/write address
 */

#define I2CS_READBIT          0x01

/* Convert 7- to 8-bit address */

#define I2CS_ADDR8(a)         ((a) << 1)
#define I2CS_WRITEADDR8(a)    I2CS_ADDR8(a)
#define I2CS_READADDR8(a)     (I2CS_ADDR8(a) | I2CS_READBIT)

/* Convert 10- to 16-bit address */

#define I2CS_ADDR10H(a)       (0xf0 | (((a) >> 7) & 0x06))
#define I2CS_ADDR10L(a)       ((a) & 0xff)

#define I2CS_WRITEADDR10H(a)  I2CS_ADDR10H(a)
#define I2CS_WRITEADDR10L(a)  I2CS_ADDR10L(a)

#define I2CS_READADDR10H(a)   (I2CS_ADDR10H(a) | I2CS_READBIT)
#define I2CS_READADDR10L(a)   I2CS_ADDR10L(a)

/* Access macros ************************************************************/

/****************************************************************************
 * Name: I2CS_SETOWNADDRESS
 *
 * Description:
 *   Set our own I2C address. Calling this function enables Slave mode and
 *   disables Master mode on the I2C bus (note that I2C is a bus, where
 *   multiple masters and slave may be handled by one device driver).
 *
 *   One may register a callback to be notified about reception. During the
 *   slave mode reception, the methods READ and WRITE must be used to
 *   to handle reads and writes from a master.
 *
 * Input Parameters:
 *   dev     - Device-specific state data
 *   address - Our own slave address; If it is 0x00, then the device driver
 *             listens to general call
 *   nbits   - The number of address bits provided (7 or 10)
 *
 * Returned Value:
 *   OK on valid address and if the same address has not been assigned
 *   to another instance sharing the same port. Otherwise ERROR is returned.
 *
 ****************************************************************************/

#define I2CS_SETOWNADDRESS(d,a,n)  ((d)->ops->setownaddress(d,a,n))

/****************************************************************************
 * Name: I2CS_WRITE
 *
 * Description:
 *   Send a block of data on I2C to the next master to issue an I2C read
 *   transaction to this slave. Required.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   buffer - A pointer to the read-only buffer of data to be written to
 *            device
 *   buflen - The number of bytes to send from the buffer
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

#define I2CS_WRITE(d,b,l) ((d)->ops->write(d,b,l))

/****************************************************************************
 * Name: I2CS_READ
 *
 * Description:
 *   Register a buffer to receive the data from the next I2C write
 *   transaction addressed to this slave.  The callback function supplied
 *   by the I2CS_REGISTERCALLBACK macro will be called once the buffer
 *   has been filled.  Required.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   buffer - A pointer to a buffer of data to receive the data from the
 *            device
 *   buflen - The requested number of bytes to be read
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

#define I2CS_READ(d,b,l) ((d)->ops->read(d,b,l))

/****************************************************************************
 * Name: I2CS_REGISTERCALLBACK
 *
 * Description:
 *   Register to receive a callback when something is received on I2C.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   callback - The function to be called when something has been received.
 *   arg      - User provided argument to be used with the callback
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

#define I2CS_REGISTERCALLBACK(d,c,a) ((d)->ops->registercallback(d,c,a))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The callback function */

typedef int (i2c_slave_callback_t)(void *arg, size_t rx_len);

/* The I2C vtable */

struct i2c_slave_s;
struct i2c_slaveops_s
{
  int (*setownaddress)(FAR struct i2c_slave_s *dev,
                       int                     addr,
                       int                     nbits);

  int (*write)(FAR struct i2c_slave_s *dev,
               FAR const uint8_t      *buffer,
               int                     buflen);

  int (*read)(FAR struct i2c_slave_s *dev,
              FAR uint8_t            *buffer,
              int                     buflen);

  int (*registercallback)(FAR struct i2c_slave_s *dev,
                          i2c_slave_callback_t   *callback,
                          FAR void               *arg);
};

/* I2C private data.  This structure only defines the initial fields of the
 * structure visible to the I2C client.  The specific implementation may
 * add additional, device specific fields after the vtable.
 */

struct i2c_slave_s
{
  const struct i2c_slaveops_s *ops; /* I2C vtable */
};

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_I2C_SLAVE */
#endif /* __INCLUDE_NUTTX_I2C_I2C_SLAVE_H */
