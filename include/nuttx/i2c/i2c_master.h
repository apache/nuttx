/****************************************************************************
 * include/nuttx/i2c/i2c_master.h
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

#ifndef __INCLUDE_NUTTX_I2C_I2C_MASTER_H
#define __INCLUDE_NUTTX_I2C_I2C_MASTER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>

#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C address calculation.  Convert 7- and 10-bit address to 8-bit and
 * 16-bit read/write address
 */

#define I2C_READBIT          0x01

/* Convert 7- to 8-bit address */

#define I2C_ADDR8(a)         ((a) << 1)
#define I2C_WRITEADDR8(a)    I2C_ADDR8(a)
#define I2C_READADDR8(a)     (I2C_ADDR8(a) | I2C_READBIT)

/* Convert 10- to 16-bit address */

#define I2C_ADDR10H(a)       (0xf0 | (((a) >> 7) & 0x06))
#define I2C_ADDR10L(a)       ((a) & 0xff)

#define I2C_WRITEADDR10H(a)  I2C_ADDR10H(a)
#define I2C_WRITEADDR10L(a)  I2C_ADDR10L(a)

#define I2C_READADDR10H(a)   (I2C_ADDR10H(a) | I2C_READBIT)
#define I2C_READADDR10L(a)   I2C_ADDR10L(a)

/* Bit definitions for the flags field in struct i2c_msg_s
 *
 * START/STOP Rules:
 *
 * 1. The lower half I2C driver will always issue the START condition at the
 *    beginning of a message unless I2C_M_NOSTART flag is set in the
 *    message.
 *
 * 2. The lower half I2C driver will always issue the STOP condition at the
 *    end of the messages unless:
 *
 *    a. The I2C_M_NOSTOP flag is set in the message, OR
 *    b. The following message has the I2C_M_NOSTART flag set (meaning
 *       that following message is simply a continuation of the transfer).
 *
 * A proper I2C repeated start would then have I2C_M_NOSTOP set on msg[n]
 * and I2C_M_NOSTART *not* set on msg[n+1].  See the following table:
 *
 *   msg[n].flags  msg[n+1].flags Behavior
 *   ------------ --------------- -----------------------------------------
 *   0            0                Two normal, separate messages with STOP
 *                                 on msg[n] then START on msg[n+1]
 *   0*           I2C_M_NOSTART    Continuation of the same transfer (must
 *                                 be the same direction).  See NOTE below.
 *   NO_STOP      0                No STOP on msg[n]; repeated START on
 *                                 msg[n+1].
 *
 * * NOTE: NO_STOP is implied in this case and may or not be explicitly
 *   included in the msg[n] flags
 */

#define I2C_M_READ           0x0001 /* Read data, from slave to master */
#define I2C_M_TEN            0x0002 /* Ten bit address */
#define I2C_M_NOSTOP         0x0040 /* Message should not end with a STOP */
#define I2C_M_NOSTART        0x0080 /* Message should not begin with a START */

/* I2c bus speed */

#define I2C_SPEED_STANDARD   100000  /* Standard speed (100Khz) */
#define I2C_SPEED_FAST       400000  /* Fast speed     (400Khz) */
#define I2C_SPEED_FAST_PLUS  1000000 /* Fast+ speed    (  1Mhz) */
#define I2C_SPEED_HIGH       3400000 /* High speed     (3.4Mhz) */

/* I2C Character Driver IOCTL Commands **************************************/

/* The I2C driver is intended to support application testing of the I2C bus.
 * The I2C driver simply wraps an instance of struct i2c_dev_s and then
 * provides the following IOCTL commands to access each method of the I2c
 * interface.
 */

/* Command:      I2CIOC_TRANSFER
 * Description:  Perform an I2C transfer
 * Argument:     A reference to an instance of struct i2c_transfer_s.
 * Dependencies: CONFIG_I2C_DRIVER
 */

#define I2CIOC_TRANSFER      _I2CIOC(0x0001)

/* Command:      I2CIOC_RESET
 * Description:  Perform an I2C bus reset in an attempt to break loose stuck
 *               I2C devices.
 * Argument:     None
 * Dependencies: CONFIG_I2C_DRIVER && CONFIG_I2C_RESET
 */

#define I2CIOC_RESET         _I2CIOC(0x0002)

/* Access macros ************************************************************/

/****************************************************************************
 * Name: I2C_TRANSFER
 *
 * Description:
 *   Perform a sequence of I2C transfers, each transfer is started with a
 *   START and the final transfer is completed with a STOP. Each sequence
 *   will be an 'atomic' operation in the sense that any other I2C actions
 *   will be serialized and pend until this sequence of transfers completes.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   msgs  - A pointer to a set of message descriptors
 *   count - The number of transfers to perform
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 *   Note : some implementations of this interface return the number of
 *          transfers completed, but others return OK on success.
 *
 ****************************************************************************/

#define I2C_TRANSFER(d,m,c) ((d)->ops->transfer(d,m,c))

/****************************************************************************
 * Name: I2C_RESET
 *
 * Description:
 *   Perform an I2C bus reset in an attempt to break loose stuck I2C devices.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_RESET
#  define I2C_RESET(d) ((d)->ops->reset(d))
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The I2C lower half driver interface */

struct i2c_master_s;
struct i2c_msg_s;
struct i2c_ops_s
{
  CODE int (*transfer)(FAR struct i2c_master_s *dev,
                       FAR struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
  CODE int (*reset)(FAR struct i2c_master_s *dev);
#endif
};

/* This structure contains the full state of I2C as needed for a specific
 * transfer.  It is passed to I2C methods so that I2C transfer may be
 * performed in a thread safe manner.
 */

struct i2c_config_s
{
  uint32_t frequency;          /* I2C frequency */
  uint16_t address;            /* I2C address (7- or 10-bit) */
  uint8_t addrlen;             /* I2C address length (7 or 10 bits) */
};

/* I2C transaction segment beginning with a START. A number of these can
 * be transferred together to form an arbitrary sequence of write/read
 * transfer to an I2C slave device.
 */

struct i2c_msg_s
{
  uint32_t frequency;         /* I2C frequency */
  uint16_t addr;              /* Slave address (7- or 10-bit) */
  uint16_t flags;             /* See I2C_M_* definitions */
  FAR uint8_t *buffer;        /* Buffer to be transferred */
  ssize_t length;             /* Length of the buffer in bytes */
};

/* I2C private data.  This structure only defines the initial fields of the
 * structure visible to the I2C client.  The specific implementation may
 * add additional, device specific fields after the vtable.
 */

struct i2c_master_s
{
  FAR const struct i2c_ops_s *ops; /* I2C vtable */
};

/* This structure is used to communicate with the I2C character driver in
 * order to perform IOCTL transfers.
 */

struct i2c_transfer_s
{
  FAR struct i2c_msg_s *msgv; /* Array of I2C messages for the transfer */
  size_t msgc;                /* Number of messages in the array. */
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

/****************************************************************************
 * Name: i2c_register
 *
 * Description:
 *   Create and register the I2C character driver.
 *
 *   The I2C character driver is a simple character driver that supports I2C
 *   transfers.  The intent of this driver is to support I2C testing.  It is
 *   not suitable for use in any real driver application.
 *
 * Input Parameters:
 *   i2c - An instance of the lower half I2C driver
 *   bus - The I2C bus number.  This will be used as the I2C device minor
 *     number.  The I2C character device will be registered as /dev/i2cN
 *     where N is the minor number
 *
 * Returned Value:
 *   OK if the driver was successfully register; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_DRIVER
int i2c_register(FAR struct i2c_master_s *i2c, int bus);
#endif

/****************************************************************************
 * Name: i2c_writeread
 *
 * Description:
 *   Send a block of data on I2C followed by restarted read access.  This
 *   provides a convenient wrapper to the transfer function.
 *
 * Input Parameters:
 *   dev     - Device-specific state data
 *   config  - Described the I2C configuration
 *   wbuffer - A pointer to the read-only buffer of data to be written to
 *             device
 *   wbuflen - The number of bytes to send from the buffer
 *   rbuffer - A pointer to a buffer of data to receive the data from the
 *             device
 *   rbuflen - The requested number of bytes to be read
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

int i2c_writeread(FAR struct i2c_master_s *dev,
                  FAR const struct i2c_config_s *config,
                  FAR const uint8_t *wbuffer, int wbuflen,
                  FAR uint8_t *rbuffer, int rbuflen);

/****************************************************************************
 * Name: i2c_write
 *
 * Description:
 *   Send a block of data on I2C. Each write operation will be an 'atomic'
 *   operation in the sense that any other I2C actions will be serialized
 *   and pend until this write completes.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   config  - Described the I2C configuration
 *   buffer - A pointer to the read-only buffer of data to be written to
 *            device
 *   buflen - The number of bytes to send from the buffer
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

int i2c_write(FAR struct i2c_master_s *dev,
              FAR const struct i2c_config_s *config,
              FAR const uint8_t *buffer, int buflen);

/****************************************************************************
 * Name: i2c_read
 *
 * Description:
 *   Receive a block of data from I2C. Each read operation will be an
 *   'atomic' operation in the sense that any other I2C actions will be
 *   serialized and pend until this read completes.
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

int i2c_read(FAR struct i2c_master_s *dev,
             FAR const struct i2c_config_s *config,
             FAR uint8_t *buffer, int buflen);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __INCLUDE_NUTTX_I2C_I2C_MASTER_H */
