/****************************************************************************
 * include/nuttx/analog/ioctl.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_ANALOG_IOCTL_H
#define __INCLUDE_NUTTX_ANALOG_IOCTL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The analog driver sub-system uses the standard character driver framework.
 * However, since the driver is a devices control interface rather than a
 * data transfer interface, the majority of the functionality is implemented
 * in driver ioctl calls.  Standard ioctl commands are listed below:
 */

/* DAC/ADC */

#define ANIOC_TRIGGER     _ANIOC(0x0001)  /* Trigger one conversion
                                           * IN: None
                                           * OUT: None */
#define ANIOC_WDOG_UPPER  _ANIOC(0x0002)  /* Set upper threshold for watchdog
                                           * IN: Threshold value
                                           * OUT: None */
#define ANIOC_WDOG_LOWER  _ANIOC(0x0003)  /* Set lower threshold for watchdog
                                           * IN: Threshold value
                                           * OUT: None */

#define AN_FIRST          0x0001          /* First common command */
#define AN_NCMDS          3               /* Number of common commands */

/* User defined ioctl commands are also supported. These will be forwarded
 * by the upper-half QE driver to the lower-half QE driver via the ioctl()
 * method fo the QE lower-half interface.  However, the lower-half driver
 * must reserve a block of commands as follows in order prevent IOCTL
 * command numbers from overlapping.
 */

/* See include/nuttx/analog/ads1242.h */

#define AN_ADS2142_FIRST  (AN_FIRST + AN_NCMDS)
#define AN_ADS2142_NCMDS  6

/* See include/nuttx/analog/lm92001.h */

#define AN_LMP92001_FIRST (AN_FIRST + AN_NCMDS + AN_ADS2142_NCMDS)
#define AN_LMP92001_NCMDS 7

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_ANALOG_IOCTL_H */
