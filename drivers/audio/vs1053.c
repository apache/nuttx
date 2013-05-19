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
#include <nuttx/audio.h>

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct vs1053_struct_s
{
  FAR struct audio_lowerhalf_s lower;     /* We derive the Audio lower half */

  /* Our specific driver data goes here */
  int spidevice;                          /* Placeholder device data */

};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     vs1053_getcaps(FAR struct audio_lowerhalf_s *lower, int type,
                 FAR struct audio_caps_s *pCaps);
static int     vs1053_configure(FAR struct audio_lowerhalf_s *lower,
                 FAR const struct audio_caps_s *pCaps);
static int     vs1053_shutdown(FAR struct audio_lowerhalf_s *lower);
static int     vs1053_start(FAR struct audio_lowerhalf_s *lower); 
static int     vs1053_stop(FAR struct audio_lowerhalf_s *lower);
static int     vs1053_enqueuebuffer(FAR struct audio_lowerhalf_s *lower, 
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
  vs1053_ioctl          /* ioctl          */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
            FAR const struct audio_caps_s *pCaps)
{
  audvdbg("Entry\n");
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

struct audio_lowerhalf_s *vs1053_initialize(int spidevice)
{
  struct vs1053_struct_s *dev;

  /* Sanity check */

#ifdef CONFIG_DEBUG
  if (spidevice < 0 || spidevice > 3) 
    {
      return NULL;
    }
#endif

  /* Allocate a VS1053 device structure */

  dev = (struct vs1053_struct_s *)kmalloc(sizeof(struct vs1053_struct_s));
  if (dev)
    {
      /* Initialize the SMART device structure */

      dev->lower.ops = &g_audioops;

      /* Save our specific device data */

      dev->spidevice = spidevice;
    }

  return &dev->lower;
}
