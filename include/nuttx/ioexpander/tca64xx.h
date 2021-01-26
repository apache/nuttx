/****************************************************************************
 * include/nuttx/ioexpander/tca64xx.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This header file derives, in part, from the Project Ara TCA64xx driver
 * which has this copyright:
 *
 *   Copyright (c) 2014-2015 Google Inc.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_IOEXPANDER_TCA64XX_H
#define __INCLUDE_NUTTX_IOEXPANDER_TCA64XX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Identifies supported TCA64xx parts (as well as the number of supported
 * parts).
 */

enum tca64xx_part_e
{
  TCA6408_PART = 0,
  TCA6416_PART,
  TCA6424_PART,
  TCA64_NPARTS
};

#ifdef CONFIG_TCA64XX_INT_ENABLE
/* This is the type of the TCA64xx interrupt handler */

typedef CODE void (*tca64_handler_t)(FAR void *arg);
#endif

/* A reference to a structure of this type must be passed to the TCA64xx
 * driver when the driver is instantiated. This structure provides
 * information about the configuration of the TCA64xx and provides some
 * board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied by
 * the driver and is presumed to persist while the driver is active. The
 * memory must be writeable because, under certain circumstances, the driver
 * may modify the frequency.
 */

struct tca64_config_s
{
  /* Device characterization */

  uint8_t address;     /* 7-bit I2C address (only bits 0-6 used) */
  uint8_t part;        /* See enum tca64xx_part_e */
  uint32_t frequency;  /* I2C or SPI frequency */

#ifdef CONFIG_TCA64XX_INT_ENABLE
  /* IRQ/GPIO access callbacks.  These operations all hidden behind
   * callbacks to isolate the TCA64xx driver from differences in GPIO
   * interrupt handling by varying boards and MCUs.
   *
   * attach  - Attach the TCA64xx interrupt handler to the GPIO interrupt
   * enable  - Enable or disable the GPIO interrupt
   */

  CODE int  (*attach)(FAR struct tca64_config_s *state,
                      tca64_handler_t handler, FAR void *arg);
  CODE void (*enable)(FAR struct tca64_config_s *state, bool enable);
#endif
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: tca64_initialize
 *
 * Description:
 *   Instantiate and configure the TCA64xx device driver to use the provided
 *   I2C device instance.
 *
 * Input Parameters:
 *   i2c     - An I2C driver instance
 *   minor   - The device i2c address
 *   config  - Persistent board configuration data
 *
 * Returned Value:
 *   an ioexpander_dev_s instance on success, NULL on failure.
 *
 ****************************************************************************/

struct i2c_master_s;
FAR struct ioexpander_dev_s *tca64_initialize(FAR struct i2c_master_s *i2c,
                                        FAR struct tca64_config_s *config);

#endif /* __INCLUDE_NUTTX_IOEXPANDER_TCA64XX_H */
