/****************************************************************************
 * include/nuttx/i2c/i2c_slave.h
 *
 *   Copyright(C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_I2C_I2C_SLAVE_H
#define __INCLUDE_NUTTX_I2C_I2C_SLAVE_H

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
 *   Send a block of data on I2C using the previously selected I2C
 *   frequency and slave address. Each write operational will be an 'atomic'
 *   operation in the sense that any other I2C actions will be serialized
 *   and pend until this write completes. Required.
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

#define I2CS_WRITE(d,b,l) ((d)->ops->write(d,b,l))

/****************************************************************************
 * Name: I2CS_READ
 *
 * Description:
 *   Receive a block of data from I2C using the previously selected I2C
 *   frequency and slave address. Each read operational will be an 'atomic'
 *   operation in the sense that any other I2C actions will be serialized
 *   and pend until this read completes. Required.
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

/* The I2C vtable */

struct i2c_slave_s;
struct i2c_slaveops_s
{
  int (*setownaddress)(FAR struct i2c_slave_s *dev, int addr, int nbits);
  int (*write)(FAR struct i2c_slave_s *dev, FAR const uint8_t *buffer,
        int buflen);
  int (*read)(FAR struct i2c_slave_s *dev, FAR uint8_t *buffer,
        int buflen);
  int (*registercallback)(FAR struct i2c_slave_s *dev,
        int (*callback)(FAR void *arg), FAR void *arg);
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
 * Public Functions
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
