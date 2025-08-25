/****************************************************************************
 * include/nuttx/audio/i2s.h
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

#ifndef __INCLUDE_NUTTX_AUDIO_I2S_H
#define __INCLUDE_NUTTX_AUDIO_I2S_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include <nuttx/audio/audio.h>

#ifdef CONFIG_I2S

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IOCTL Commands ***********************************************************/

/* The I2S module uses a standard character driver framework with device
 * control interface for configuring I2S parameters such as data width,
 * channel count, and sample rate for both RX and TX directions. The I2S
 * ioctl commands are listed below:
 *
 * I2SIOC_GRXDATAWIDTH - Get RX Data Width
 *   ioctl argument:  Pointer to uint32_t to receive the current RX data
 *                    width in bits
 *   Returned value:  uint32_t - The current RX data width in bits (typically
 *                    8, 16, or 32)
 *   Error codes:     -ENOTTY if not supported by driver
 *
 * I2SIOC_GTXDATAWIDTH - Get TX Data Width
 *   ioctl argument:  Pointer to uint32_t to receive the current TX data
 *                    width in bits
 *   Returned value:  uint32_t - The current TX data width in bits (typically
 *                    8, 16, or 32)
 *   Error codes:     -ENOTTY if not supported by driver
 *
 * I2SIOC_GRXCHANNELS - Get RX Channel Count
 *   ioctl argument:  Pointer to int to receive the current RX channel count
 *   Returned value:  int - The current number of RX channels (typically 1
 *                    or 2)
 *   Error codes:     -ENOTTY if not supported by driver
 *
 * I2SIOC_GTXCHANNELS - Get TX Channel Count
 *   ioctl argument:  Pointer to int to receive the current TX channel count
 *   Returned value:  int - The current number of TX channels (typically 1
 *                    or 2)
 *   Error codes:     -ENOTTY if not supported by driver
 *
 * I2SIOC_GRXSAMPLERATE - Get RX Sample Rate
 *   ioctl argument:  Pointer to uint32_t to receive the current RX sample
 *                    rate
 *   Returned value:  uint32_t - The current RX sample rate in samples per
 *                    second (e.g., 44100)
 *   Error codes:     -ENOTTY if not supported by driver
 *
 * I2SIOC_GTXSAMPLERATE - Get TX Sample Rate
 *   ioctl argument:  Pointer to uint32_t to receive the current TX sample
 *                    rate
 *   Returned value:  uint32_t - The current TX sample rate in samples per
 *                    second (e.g., 44100)
 *   Error codes:     -ENOTTY if not supported by driver
 *
 * I2SIOC_SRXDATAWIDTH - Set RX Data Width
 *   ioctl argument:  uint32_t value specifying the desired RX data width
 *                    in bits
 *   Returned value:  uint32_t - The resulting bitrate (sample_rate *
 *                    data_width) or 0 on success
 *   Error codes:     -EINVAL if invalid data width, -ENOSYS if unsupported
 *                    width, -ENOTTY if not supported by driver
 *
 * I2SIOC_STXDATAWIDTH - Set TX Data Width
 *   ioctl argument:  uint32_t value specifying the desired TX data width
 *                    in bits
 *   Returned value:  uint32_t - The resulting bitrate (sample_rate *
 *                    data_width) or 0 on success
 *   Error codes:     -EINVAL if invalid data width, -ENOSYS if unsupported
 *                    width, -ENOTTY if not supported by driver
 *
 * I2SIOC_SRXCHANNELS - Set RX Channel Count
 *   ioctl argument:  int value specifying the desired RX channel count
 *   Returned value:  int - The number of RX channels set or a negated
 *                    errno value
 *   Error codes:     -EINVAL if invalid channel count (not 1 or 2),
 *                    -ENOTTY if not supported by driver
 *
 * I2SIOC_STXCHANNELS - Set TX Channel Count
 *   ioctl argument:  int value specifying the desired TX channel count
 *   Returned value:  int - The number of TX channels set or a negated
 *                    errno value
 *   Error codes:     -EINVAL if invalid channel count (not 1 or 2),
 *                    -ENOTTY if not supported by driver
 *
 * I2SIOC_SRXSAMPLERATE - Set RX Sample Rate
 *   ioctl argument:  uint32_t value specifying the desired RX sample rate
 *                    in Hz
 *   Returned value:  uint32_t - The resulting bitrate or 0 on success
 *   Error codes:     -EINVAL if invalid sample rate (typically < 8000 Hz),
 *                    -ENOTTY if not supported by driver
 *
 * I2SIOC_STXSAMPLERATE - Set TX Sample Rate
 *   ioctl argument:  uint32_t value specifying the desired TX sample rate
 *                    in Hz
 *   Returned value:  uint32_t - The resulting bitrate or 0 on success
 *   Error codes:     -EINVAL if invalid sample rate (typically < 8000 Hz),
 *                    -ENOTTY if not supported by driver
 *
 * Note: The bitrate is calculated as sample_rate * data_width. Some drivers
 * may have coupling between RX and TX parameters, so changing one may
 * affect the other. Sample rates are typically constrained to a minimum of
 * 8000 Hz, and data widths are typically limited to 8, 16, or 32 bits
 * depending on the hardware capabilities.
 */

/* Access macros ************************************************************/

#define I2SIOC_GRXDATAWIDTH            _I2SOC(1)
#define I2SIOC_GTXDATAWIDTH            _I2SOC(2)
#define I2SIOC_GRXCHANNELS             _I2SOC(3)
#define I2SIOC_GTXCHANNELS             _I2SOC(4)
#define I2SIOC_GRXSAMPLERATE           _I2SOC(5)
#define I2SIOC_GTXSAMPLERATE           _I2SOC(6)
#define I2SIOC_SRXDATAWIDTH            _I2SOC(7)
#define I2SIOC_STXDATAWIDTH            _I2SOC(8)
#define I2SIOC_SRXCHANNELS             _I2SOC(9)
#define I2SIOC_STXCHANNELS             _I2SOC(10)
#define I2SIOC_SRXSAMPLERATE           _I2SOC(11)
#define I2SIOC_STXSAMPLERATE           _I2SOC(12)

/****************************************************************************
 * Name: I2S_RXCHANNELS
 *
 * Description:
 *   Set the I2S RX channel num. NOTE: This may also have unexpected side-
 *   effects of the RX channel num is coupled with the TX channel num.
 *
 * Input Parameters:
 *   dev     - Device-specific state data
 *   channel - The I2S channel num
 *
 * Returned Value:
 *   Returns the number of RX channels
 *
 ****************************************************************************/

#define I2S_RXCHANNELS(d,c) \
  ((d)->ops->i2s_rxchannels ? (d)->ops->i2s_rxchannels(d,c) : -ENOTTY)

/****************************************************************************
 * Name: I2S_RXSAMPLERATE
 *
 * Description:
 *   Set the I2S RX sample rate.  NOTE:  This will have no effect if (1) the
 *   driver does not support an I2S receiver or if (2) the sample rate is
 *   driven by the I2S frame clock.  This may also have unexpected side-
 *   effects of the RX sample is coupled with the TX sample rate.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   rate - The I2S sample rate in samples (not bits) per second
 *
 * Returned Value:
 *   Returns the resulting bitrate
 *
 ****************************************************************************/

#define I2S_RXSAMPLERATE(d,r) \
  ((d)->ops->i2s_rxsamplerate ? (d)->ops->i2s_rxsamplerate(d,r) : -ENOTTY)

/****************************************************************************
 * Name: I2S_RXDATAWIDTH
 *
 * Description:
 *   Set the I2S RX data width.  The RX bitrate is determined by
 *   sample_rate * data_width.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   width - The I2S data with in bits.
 *
 * Returned Value:
 *   Returns the resulting bitrate
 *
 ****************************************************************************/

#define I2S_RXDATAWIDTH(d,b) \
  ((d)->ops->i2s_rxdatawidth ? (d)->ops->i2s_rxdatawidth(d,b) : -ENOTTY)

/****************************************************************************
 * Name: I2S_RECEIVE
 *
 * Description:
 *   Receive a block of data from I2S.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   apb      - A pointer to the audio buffer in which to receive data
 *   callback - A user provided callback function that will be called at
 *              the completion of the transfer.  The callback will be
 *              performed in the context of the worker thread.
 *   arg      - An opaque argument that will be provided to the callback
 *              when the transfer complete.
 *   timeout  - The timeout value to use.  The transfer will be canceled
 *              and an ETIMEDOUT error will be reported if this timeout
 *              elapsed without completion of the DMA transfer.  Units
 *              are system clock ticks.  Zero means no timeout.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.  NOTE:  This function
 *   only enqueues the transfer and returns immediately.  Success here only
 *   means that the transfer was enqueued correctly.
 *
 *   When the transfer is complete, a 'result' value will be provided as
 *   an argument to the callback function that will indicate if the transfer
 *   failed.
 *
 ****************************************************************************/

#define I2S_RECEIVE(d,b,c,a,t) \
  ((d)->ops->i2s_receive ? (d)->ops->i2s_receive(d,b,c,a,t) : -ENOTTY)

/****************************************************************************
 * Name: I2S_RXCHANNELS
 *
 * Description:
 *   Set the I2S TX channel num. NOTE: This may also have unexpected side-
 *   effects of the TX channel num is coupled with the RX channel num.
 *
 * Input Parameters:
 *   dev     - Device-specific state data
 *   channel - The I2S channel num
 *
 * Returned Value:
 *   Returns the number of TX channels
 *
 ****************************************************************************/

#define I2S_TXCHANNELS(d,c) \
  ((d)->ops->i2s_txchannels ? (d)->ops->i2s_txchannels(d,c) : -ENOTTY)

/****************************************************************************
 * Name: I2S_TXSAMPLERATE
 *
 * Description:
 *   Set the I2S TX sample rate.  NOTE:  This will have no effect if (1) the
 *   driver does not support an I2S transmitter or if (2) the sample rate is
 *   driven by the I2S frame clock.  This may also have unexpected side-
 *   effects of the TX sample is coupled with the RX sample rate.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   rate - The I2S sample rate in samples (not bits) per second
 *
 * Returned Value:
 *   Returns the resulting bitrate
 *
 ****************************************************************************/

#define I2S_TXSAMPLERATE(d,r) \
  ((d)->ops->i2s_txsamplerate ? (d)->ops->i2s_txsamplerate(d,r) : -ENOTTY)

/****************************************************************************
 * Name: I2S_TXDATAWIDTH
 *
 * Description:
 *   Set the I2S TX data width.  The TX bitrate is determined by
 *   sample_rate * data_width.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   width - The I2S data with in bits.
 *
 * Returned Value:
 *   Returns the resulting bitrate
 *
 ****************************************************************************/

#define I2S_TXDATAWIDTH(d,b) \
  ((d)->ops->i2s_txdatawidth ? (d)->ops->i2s_txdatawidth(d,b) : -ENOTTY)

/****************************************************************************
 * Name: I2S_SEND
 *
 * Description:
 *   Send a block of data on I2S.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   apb      - A pointer to the audio buffer from which to send data
 *   callback - A user provided callback function that will be called at
 *              the completion of the transfer.  The callback will be
 *              performed in the context of the worker thread.
 *   arg      - An opaque argument that will be provided to the callback
 *              when the transfer completes.
 *   timeout  - The timeout value to use.  The transfer will be cancelled
 *              and an ETIMEDOUT error will be reported if this timeout
 *              elapsed without completion of the DMA transfer.  Units
 *              are system clock ticks.  Zero means no timeout.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.  NOTE:  This function
 *   only enqueues the transfer and returns immediately.  Success here only
 *   means that the transfer was enqueued correctly.
 *
 *   When the transfer is complete, a 'result' value will be provided as
 *   an argument to the callback function that will indicate if the transfer
 *   failed.
 *
 ****************************************************************************/

#define I2S_SEND(d,b,c,a,t) \
  ((d)->ops->i2s_send ? (d)->ops->i2s_send(d,b,c,a,t) : -ENOTTY)

/****************************************************************************
 * Name: I2S_GETMCLKFREQUENCY
 *
 * Description:
 *   Get the current master clock frequency. NOTE: this parameter may not
 *   be implemented on I2S driver. If not implemented, the I2S may set
 *   internally any value to the master clock (or even does not support it).
 *
 * Input Parameters:
 *   dev        - Device-specific state data
 *
 * Returned Value:
 *   Returns the current master clock.
 *
 ****************************************************************************/

#define I2S_GETMCLKFREQUENCY(d) \
  ((d)->ops->i2s_getmclkfrequency ? \
   (d)->ops->i2s_getmclkfrequency(d) : -ENOTTY)

/****************************************************************************
 * Name: I2S_SETMCLKFREQUENCY
 *
 * Description:
 *   Set the master clock frequency. Usually, the MCLK is a multiple of the
 *   sample rate. Most of the audio codecs require setting specific MCLK
 *   frequency according to the sample rate. NOTE: this parameter may not
 *   be implemented on I2S driver. If not implemented, the I2S may set
 *   internally any value to the master clock (or even does not support it).
 *
 * Input Parameters:
 *   dev        - Device-specific state data
 *   frequency  - The I2S master clock's frequency
 *
 * Returned Value:
 *   Returns the resulting master clock or a negated errno value on failure.
 *
 ****************************************************************************/

#define I2S_SETMCLKFREQUENCY(d,f) \
  ((d)->ops->i2s_setmclkfrequency ? \
   (d)->ops->i2s_setmclkfrequency(d,f) : -ENOTTY)

/****************************************************************************
 * Name: I2S_IOCTL
 *
 * Description:
 *   IOCTL of I2S.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   cmd      - A pointer to the audio buffer from which to send data
 *   arg      - An opaque argument that will be provided to the callback
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

#define I2S_IOCTL(d,c,a) \
  ((d)->ops->i2s_ioctl ? (d)->ops->i2s_ioctl(d,c,a) : -ENOTTY)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Transfer complete callbacks */

struct i2s_dev_s;
typedef CODE void (*i2s_callback_t)(FAR struct i2s_dev_s *dev,
                   FAR struct ap_buffer_s *apb, FAR void *arg, int result);

/* The I2S vtable */

struct i2s_ops_s
{
  /* Receiver methods */

  CODE int      (*i2s_rxchannels)(FAR struct i2s_dev_s *dev,
                                  uint8_t channels);
  CODE uint32_t (*i2s_rxsamplerate)(FAR struct i2s_dev_s *dev,
                                    uint32_t rate);
  CODE uint32_t (*i2s_rxdatawidth)(FAR struct i2s_dev_s *dev,
                                   int bits);
  CODE int      (*i2s_receive)(FAR struct i2s_dev_s *dev,
                               FAR struct ap_buffer_s *apb,
                               i2s_callback_t callback,
                               FAR void *arg,
                               uint32_t timeout);

  /* Transmitter methods */

  CODE int      (*i2s_txchannels)(FAR struct i2s_dev_s *dev,
                                  uint8_t channels);
  CODE uint32_t (*i2s_txsamplerate)(FAR struct i2s_dev_s *dev,
                                    uint32_t rate);
  CODE uint32_t (*i2s_txdatawidth)(FAR struct i2s_dev_s *dev,
                                   int bits);
  CODE int      (*i2s_send)(FAR struct i2s_dev_s *dev,
                            FAR struct ap_buffer_s *apb,
                            i2s_callback_t callback,
                            FAR void *arg,
                            uint32_t timeout);

  /* Master Clock methods */

  CODE uint32_t (*i2s_getmclkfrequency)(FAR struct i2s_dev_s *dev);
  CODE uint32_t (*i2s_setmclkfrequency)(FAR struct i2s_dev_s *dev,
                                        uint32_t frequency);

  /* Ioctl */

  CODE int      (*i2s_ioctl)(FAR struct i2s_dev_s *dev,
                             int cmd, unsigned long arg);
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
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: i2schar_register
 *
 * Description:
 *   Create and register the I2S character driver.
 *
 *   The I2S character driver is a simple character driver that supports I2S
 *   transfers via a read() and write().  The intent of this driver is to
 *   support I2S testing.  It is not an audio driver but does conform to some
 *   of the buffer management heuristics of an audio driver.  It is not
 *   suitable for use in any real driver application in its current form.
 *
 * Input Parameters:
 *   i2s - An instance of the lower half I2S driver
 *   minor - The device minor number.  The I2S character device will be
 *     registers as /dev/i2scharN where N is the minor number
 *
 * Returned Value:
 *   OK if the driver was successfully register; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int i2schar_register(FAR struct i2s_dev_s *i2s, int minor);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_I2S */
#endif /* __INCLUDE_NUTTX_AUDIO_I2S_H */
