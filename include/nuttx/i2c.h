/****************************************************************************
 * include/nuttx/i2c.h
 *
 *   Copyright(C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __NUTTX_I2C_H
#define __NUTTX_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Access macros */

/****************************************************************************
 * Name: I2C_SETFREQUENCY
 *
 * Description:
 *   Set the I2C frequency. Required.
 *
 * Input Parameters:
 *   dev -       Device-specific state data
 *   frequency - The I2C frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

#define I2C_SETFREQUENCY(d,f) ((d)->ops->setfrequency(d,f))

/****************************************************************************
 * Name: I2C_WRITE
 *
 * Description:
 *   Send a block of data on I2C. Required.
 *
 * Input Parameters:
 *   dev -    Device-specific state data
 *   buffer - A pointer to the read-only buffer of data to be written to device
 *   buflen - The number of bytes to send from the buffer
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

#define I2C_WRITE(d,b,l) ((d)->ops->sndblock(d,b,l))

/****************************************************************************
 * Name: I2C_READ
 *
 * Description:
 *   Receive a block of data from I2C. Required.
 *
 * Input Parameters:
 *   dev -   Device-specific state data
 *   buffer - A pointer to a buffer of data to receive the data from the device
 *   buflen - The requested number of bytes to be read
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

#define I2C_READ(d,b,l) ((d)->ops->sndblock(d,b,l))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The I2C vtable */

struct i2c_dev_s;
struct i2c_ops_s
{
  uint32 (*setfrequency)(FAR struct i2c_dev_s *dev, uint32 frequency);
  int    (*write)(FAR struct i2c_dev_s *dev, int addr, const ubyte *buffer, int buflen);
  int    (*read)(FAR struct i2c_dev_s *dev, int addr, ubyte *buffer, int buflen);
};

/* I2C private data.  This structure only defines the initial fields of the
 * structure visible to the I2C client.  The specific implementation may 
 * add additional, device specific fields
 */

struct i2c_dev_s
{
  const struct i2c_ops_s *ops;
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: up_i2cinitialize
 *
 * Description:
 *   Initialize the selected I2C port.
 *
 * Input Parameter:
 *   Port number (for hardware that has mutiple I2C interfaces)
 *
 * Returned Value:
 *   Valid I2C device structre reference on succcess; a NULL on failure
 *
 ****************************************************************************/

EXTERN FAR struct i2c_dev_s *up_i2cinitialize(int port);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __NUTTX_I2C_H */
