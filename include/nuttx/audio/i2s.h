/****************************************************************************
 * include/nuttx/audio/i2s.h
 *
 *   Copyright(C) 2013 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_AUDIO_I2S_H
#define __INCLUDE_NUTTX_AUDIO_I2S_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef CONFIG_I2S

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

/* Access macros ************************************************************/

/****************************************************************************
 * Name: I2S_FREQUENCY
 *
 * Description:
 *   Set the I2S frequency. Required.
 *
 * Input Parameters:
 *   dev -       Device-specific state data
 *   frequency - The I2S frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

#define I2S_FREQUENCY(d,f) ((d)->ops->i2s_frequency(d,f))

/****************************************************************************
 * Name: I2S_SEND
 *
 * Description:
 *   Send a block of data on I2S. Required.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   buffer - A pointer to the buffer of data to be sent
 *   nbytes - the length of data to send from the buffer in number of bytes.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#define I2S_SEND(d,b,n) ((d)->ops->i2s_send(d,b,n))

/****************************************************************************
 * Name: I2S_RECEIVE
 *
 * Description:
 *   Receive a block of data from I2S. Required.
 *
 * Input Parameters:
 *   dev -    Device-specific state data
 *   buffer - A pointer to the buffer in which to recieve data
 *   nbytes - The length of data that can be received in the buffer in number
 *            of bytes.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#define I2S_RECEIVE(d,b,n) ((d)->ops->i2s_receive(d,b,n))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The I2S vtable */

struct i2s_dev_s;
struct i2s_ops_s
{
  uint32_t (*i2s_frequency)(FAR struct i2s_dev_s *dev, uint32_t frequency);
  void     (*i2s_send)(FAR struct i2s_dev_s *dev, FAR const void *buffer,
                       size_t nbytes);
  void     (*i2s_receive)(FAR struct i2s_dev_s *dev, FAR void *buffer,
                          size_t nbytes);
};

/* I2S private data.  This structure only defines the initial fields of the
 * structure visible to the I2S client.  The specific implementation may 
 * add additional, device specific fields
 */

struct i2s_dev_s
{
  FAR const struct i2s_ops_s *ops;
};

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
 * Public Functions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_I2S */
#endif /* __INCLUDE_NUTTX_AUDIO_I2S_H */
