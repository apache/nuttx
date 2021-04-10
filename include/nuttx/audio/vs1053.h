/****************************************************************************
 * include/nuttx/audio/vs1053.h
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
 * CONFIG_AUDIO_VS1053 - Enabled VS1053 support
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
  int  (*attach)(FAR const struct vs1053_lower_s *lower, xcpt_t handler,
                 FAR void *arg);
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
 * Name: vs1053_initialize
 *
 * Description:
 *   Initialize the VS1053 driver.  This will perform a chip reset of the
 *   device as part of initialization.
 *
 * Input Parameters:
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
