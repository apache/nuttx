/****************************************************************************
 * include/nuttx/lcd/tca19988.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
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
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_LCD_TDA19988_H
#define __INCLUDE_NUTTX_LCD_TDA19988_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/irq.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/lcd/lcd_ioctl.h>
#include <nuttx/video/videomode.h>

#ifdef CONFIG_LCD_TDA19988

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* TDA19988 IOCTL commands **************************************************
 *
 * TDA19988_IOC_VIDEOMODE:
 *   Description:  Select the video mode.  This must be done as part of the
 *                 initialization of the driver.  This is equivalent to
 *                 calling tda18899_videomode() within the OS.
 *   Argument:     A reference to a videomode_s structure instance.
 *                 See struct videomode_s below.
 *   Returns:      None
 */

#define TDA19988_IOC_VIDEOMODE _LCDIOC(TDA19988_NIOCTL_BASE + 0)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Opaque handle returned by tda19988_register() */

typedef FAR void *TDA19988_HANDLE;

/* This structure defines the I2C interface.
 * REVISIT:  This could be simplified because the CEC and HDMI reside on
 * the same I2C bus (pins CSCL and CSCA).
 */

struct tda19988_i2c_s
{
  struct i2c_config_s config;   /* Frequency, address, address length */
  FAR struct i2c_master_s *i2c; /* I2C bus interface */
};

/* This structure provides the TDA19988 lower half interface */

struct i2c_master_s;     /* Forward reference */

struct tda19988_lower_s
{
  /* I2C bus interfaces (CEC and HDMI lie on same I2C bus) */

  struct tda19988_i2c_s cec;
  struct tda19988_i2c_s hdmi;

  /* Interrupt controls
   *
   * The interrupt output (open-drain) is used as dual function pin
   * selectable through I2C-bus.  In calibration mode this pin is used as
   * input for 10 ms +/- 1% calibration pulse.  In operational mode this pin
   * is used to warn the external microprocessor that a special event has
   * occurred for HDMI or CEC
   *
   * REVISIT: The interrupt is not currently used by the upper half driver.
   */

  CODE int (*attach)(FAR const struct tda19988_lower_s *lower,
                     xcpt_t handler, FAR void *arg);
  CODE int (*enable)(FAR const struct tda19988_lower_s *lower,
                     bool enable);
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
 * Name: tda19988_register
 *
 * Description:
 *   Create and register the the TDA19988 driver at 'devpath'
 *
 * Input Parameters:
 *   devpath - The location to register the TDA19988 driver instance.  The
 *             standard location would be a path like /dev/hdmi0.
 *   lower   - The interface to the the TDA19988 lower half driver.
 *
 * Returned Value:
 *   On success, non-NULL handle is returned that may be subsequently used
 *   with tda19988_videomode().  NULL is returned on failure.
 *
 ****************************************************************************/

TDA19988_HANDLE tda19988_register(FAR const char *devpath,
                                  FAR const struct tda19988_lower_s *lower);

/****************************************************************************
 * Name: tda19988_videomode
 *
 * Description:
 *   Initialize the TDA19988 driver to a specified video mode.  This is a
 *   necessary part of the TDA19988 initialization:  A video mode  must be
 *   configured before the driver is usable.
 *
 *   NOTE:  This may be done in two ways:  (1) via a call to
 *   tda19988_videomode() from board-specific logic within the OS, or
 *   equivalently (2) using the TDA19988_IOC_VIDEOMODE from application
 *   logic outside of the OS.
 *
 * Input Parameters:
 *   handle - The handle previously returned by tda19988_register().
 *   mode   - The new video mode.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int tda19988_videomode(TDA19988_HANDLE handle,
                       FAR const struct videomode_s *mode);

/****************************************************************************
 * Name: tda19988_read_edid
 *
 * Description:
 *   Read the EDID (Extended Display Identification Data).
 *
 *   NOTE:  This may be done in two ways:  (1) via a call to
 *   tda19988_read_edid() from board-specific logic within the OS, or
 *   equivalently (2) using a standard read() to read the EDID from
 *   application logic outside of the OS.
 *
 * Input Parameters:
 *   handle - The handle previously returned by tda19988_register().
 *   offset - The offset into the EDID to begin reading (0..127)
 *   buffer - Location in which to return the EDID data
 *   buflen - Size of buffer in bytes
 *
 * Returned Value:
 *   On success, the number of bytes read is returned; a negated errno value
 *   is returned on any failure.
 *
 ****************************************************************************/

ssize_t tda19988_read_edid(TDA19988_HANDLE handle, off_t offset,
                           FAR uint8_t *buffer, size_t buflen);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_LCD_TDA19988 */
#endif /* __INCLUDE_NUTTX_LCD_TDA19988_H */
