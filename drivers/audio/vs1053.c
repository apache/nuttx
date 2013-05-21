/****************************************************************************
 * drivers/audio/vs1053.c
 *
 * Audio device driver for VLSI Solutions VS1053 Audio codec.
 *
 *   Copyright (C) 2013 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/audio/audio.h>
#include <nuttx/audio/vs1053.h>

#include "vs1053.h"

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

#ifndef CONFIG_VS1053_SPIMODE
#  define CONFIG_VS1053_SPIMODE SPIDEV_MODE0
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct vs1053_struct_s
{
  FAR struct audio_lowerhalf_s lower;     /* We derive the Audio lower half */

  /* Our specific driver data goes here */
  FAR struct spi_dev_s *spi;              /* Pointer to the SPI bus */

};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     vs1053_getcaps(FAR struct audio_lowerhalf_s *lower, int type,
                 FAR struct audio_caps_s *pCaps);
static int     vs1053_configure(FAR struct audio_lowerhalf_s *lower,
                 FAR const struct audio_caps_s *pCaps, audio_callback_t upper,
                 FAR void *priv);
static int     vs1053_shutdown(FAR struct audio_lowerhalf_s *lower);
static int     vs1053_start(FAR struct audio_lowerhalf_s *lower);
static int     vs1053_stop(FAR struct audio_lowerhalf_s *lower);
static int     vs1053_enqueuebuffer(FAR struct audio_lowerhalf_s *lower,
                 FAR struct ap_buffer_s *apb);
static int     vs1053_cancelbuffer(FAR struct audio_lowerhalf_s *lower,
                 FAR struct ap_buffer_s *apb);
static int     vs1053_ioctl(FAR struct audio_lowerhalf_s *lower, int cmd,
                 unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct audio_ops_s g_audioops =
{
  vs1053_getcaps,       /* getcaps        */
  vs1053_configure,     /* configure      */
  vs1053_shutdown,      /* shutdown       */
  vs1053_start,         /* start          */
  vs1053_stop,          /* stop           */
  vs1053_enqueuebuffer, /* enqueue_buffer */
  vs1053_cancelbuffer,  /* cancel_buffer  */
  vs1053_ioctl          /* ioctl          */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/************************************************************************************
 * Name: vs1053_spi_lock
 ************************************************************************************/

static void vs1053_spi_lock(FAR struct spi_dev_s *dev)
{
  /* On SPI busses where there are multiple devices, it will be necessary to
   * lock SPI to have exclusive access to the busses for a sequence of
   * transfers.  The bus should be locked before the chip is selected.
   *
   * This is a blocking call and will not return until we have exclusiv access to
   * the SPI buss.  We will retain that exclusive access until the bus is unlocked.
   */

  (void)SPI_LOCK(dev, true);

  /* After locking the SPI bus, the we also need call the setfrequency, setbits, and
   * setmode methods to make sure that the SPI is properly configured for the device.
   * If the SPI buss is being shared, then it may have been left in an incompatible
   * state.
   */

  SPI_SETMODE(dev, CONFIG_VS1053_SPIMODE);
  SPI_SETBITS(dev, 8);
  (void)SPI_SETFREQUENCY(dev, 20000000);
}

/************************************************************************************
 * Name: vs1053_spi_unlock
 ************************************************************************************/

static inline void vs1053_spi_unlock(FAR struct spi_dev_s *dev)
{
  (void)SPI_LOCK(dev, false);
}

/************************************************************************************
 * Name: vs1053_readreg - Read the specified 16-bit register from the
 *                        VS1053 device.  Caller must hold the SPI lock.
 ************************************************************************************/

static uint16_t vs1053_readreg(FAR struct vs1053_struct_s *dev, uint16_t reg)
{
  uint16_t ret;
  FAR struct spi_dev_s *spi = dev->spi;

  /* Select the AUDIO_CTRL device on the SPI bus */

  SPI_SELECT(spi, SPIDEV_AUDIO_CTRL, true);

  /* Send the WRITE command followed by the address */

  SPI_SEND(spi, VS1053_OPCODE_READ);
  SPI_SEND(spi, reg);

  /* Now read the 16-bit value */

  ret = SPI_SEND(spi, 0xFF) << 8;
  ret |= SPI_SEND(spi, 0xFF);

  /* Deselect the CODEC */

  SPI_SELECT(spi, SPIDEV_AUDIO_CTRL, false);

  return ret;
}

/****************************************************************************
 * Name: vs1053_getcaps
 *
 * Description: Get the audio device capabilities
 *
 ****************************************************************************/

static int vs1053_getcaps(FAR struct audio_lowerhalf_s *lower, int type,
            FAR struct audio_caps_s *pCaps)
{
  audvdbg("Entry\n");
  return OK;
}

/****************************************************************************
 * Name: vs1053_configure
 *
 * Description: Configure the audio device for the specified  mode of
 *              operation.
 *
 ****************************************************************************/

static int vs1053_configure(FAR struct audio_lowerhalf_s *lower,
            FAR const struct audio_caps_s *pCaps, audio_callback_t upper,
            FAR void *priv)
{
  audvdbg("Entry\n");

  /* Save the binding to the upper level operations and private data */

  lower->upper = upper;
  lower->priv = priv;

  /* Now process the configure operation */

  return OK;
}

/****************************************************************************
 * Name: vs1053_shutdown
 *
 * Description: Shutdown the VS1053 chip and put it in the lowest power
 *              state possible.
 *
 ****************************************************************************/

static int vs1053_shutdown(FAR struct audio_lowerhalf_s *lower)
{
  audvdbg("Entry\n");
  return OK;
}

/****************************************************************************
 * Name: vs1053_start
 *
 * Description: Start the configured operation (audio streaming, volume
 *              enabled, etc.).
 *
 ****************************************************************************/

static int vs1053_start(FAR struct audio_lowerhalf_s *lower)
{
  //struct vs1053_struct_s *dev = (struct vs1053_struct_s *) lower;

  /* Perform the start */

  return OK;
}

/****************************************************************************
 * Name: vs1053_stop
 *
 * Description: Stop the configured operation (audio streaming, volume
 *              disabled, etc.).
 *
 ****************************************************************************/

static int vs1053_stop(FAR struct audio_lowerhalf_s *lower)
{
  /* Stop all audio streaming */

  return OK;
}

/****************************************************************************
 * Name: vs1053_enqueuebuffer
 *
 * Description: Enqueue an Audio Pipeline Buffer for playback/ processing.
 *
 ****************************************************************************/

static int vs1053_enqueuebuffer(FAR struct audio_lowerhalf_s *lower,
                 FAR struct ap_buffer_s *apb )
{
  return OK;
}

/****************************************************************************
 * Name: vs1053_cancelbuffer
 *
 * Description: Called when an enqueued buffer is being cancelled.
 *
 ****************************************************************************/

static int vs1053_cancelbuffer(FAR struct audio_lowerhalf_s *lower,
                 FAR struct ap_buffer_s *apb )
{
  return OK;
}

/****************************************************************************
 * Name: vs1053_ioctl
 *
 * Description: Perform a device ioctl
 *
 ****************************************************************************/

static int vs1053_ioctl(FAR struct audio_lowerhalf_s *lower, int cmd,
                  unsigned long arg)
{
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vs1053_initialize
 *
 * Description:
 *   Initialize the VS1053 device
 *
 * Input Parameters:
 *   spidevice - This is a placeholder argument until the Audio interface
 *      has been flushed out a bit.
 *
 ****************************************************************************/

struct audio_lowerhalf_s *vs1053_initialize(FAR struct spi_dev_s *spi,
                            FAR const struct vs1053_lower_s *lower,
                            unsigned int devno)
{
  struct vs1053_struct_s *dev;
  uint16_t                status;
  uint8_t                 id;

  /* Sanity check */

  DEBUGASSERT(spi != NULL);
  DEBUGASSERT(lower != NULL);
  DEBUGASSERT(lower->reset != NULL);

  /* Allocate a VS1053 device structure */

  dev = (struct vs1053_struct_s *)kmalloc(sizeof(struct vs1053_struct_s));
  if (dev)
    {
      /* Initialize the SMART device structure */

      dev->lower.ops = &g_audioops;
      dev->lower.upper = NULL;
      dev->lower.priv = NULL;

      /* Save our specific device data */

      dev->spi = spi;

      /* Reset the VS1053 chip */

      lower->reset(lower, false);
      up_udelay(4000);
      lower->reset(lower, true);

      /* Do device detection to validate the chip is there.
       * We have to hold the SPI lock during reads / writes.
       */

      vs1053_spi_lock(spi);
      status = vs1053_readreg(dev, VS1053_SCI_STATUS);
      vs1053_spi_unlock(spi);

      id = (status & VS1053_SS_VER) >> VS1053_VER_SHIFT;
      if (id != VS1053_VER_VS1053)
        {
          auddbg("Unexpected VER bits: 0x%0X\n", id);
        }
    }

  return &dev->lower;
}
