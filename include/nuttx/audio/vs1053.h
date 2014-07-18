/****************************************************************************
 * include/nuttx/audio/vs1053.h
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

#ifndef __INCLUDE_NUTTX_AUDIO_VS1053_H
#define __INCLUDE_NUTTX_AUDIO_VS1053_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* VS1053 Configuration Settings:
 *
 * CONFIG_VS1053 - Enabled VS1053 support
 * CONFIG_VS1053_SPIMODE - Controls the SPI mode
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The VS1053 provides Data Request (DREQ) interrupts to the MCU via a GPIO
 * pin and also has a chip reset GPIO.  The following structure provides an
 * MCU-independent mechanism for controlling the VS1053 GPIOs.
 */

struct vs1053_lower_s
{
  int  (*attach)(FAR const struct vs1053_lower_s *lower, xcpt_t handler);
  void (*enable)(FAR const struct vs1053_lower_s *lower);
  void (*disable)(FAR const struct vs1053_lower_s *lower);
  void (*reset)(FAR const struct vs1053_lower_s *lower, bool state);
  int  (*read_dreq)(FAR const struct vs1053_lower_s *lower);
  int  irq;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Function: vs1053_initialize
 *
 * Description:
 *   Initialize the VS1053 driver.  This will perform a chip reset of the
 *   device as part of initialization.
 *
 * Parameters:
 *   spi   - A reference to the platform's SPI driver for the VS1053
 *   lower - The MCU-specific interrupt used to control low-level MCU
 *           functions (i.e., VS1053 GPIO interrupts).
 *   devno - If more than one VS1053 is supported, then this is the
 *           zero based number that identifies the VS1053;
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

struct spi_dev_s;         /* See nuttx/spi/spi.h */
struct audio_lowerhalf_s; /* See nuttx/audio/audio.h */

FAR struct audio_lowerhalf_s *vs1053_initialize(FAR struct spi_dev_s *spi,
                          FAR const struct vs1053_lower_s *lower,
                          unsigned int devno);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_AUDIO_VS1053_H */
