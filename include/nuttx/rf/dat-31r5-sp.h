/****************************************************************************
 * include/nuttx/rf/dat-31r5-sp.h
 * Character driver for the Mini-Circuits DAT-31R5-SP+ digital step
 * attenuator.
 *
 *   Copyright (C) 2019, Augusto Fraga Giachero. All rights reserved.
 *   Author: Augusto Fraga Giachero <afg@augustofg.net>
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

#ifndef __INCLUDE_NUTTX_RF_DAT_31R5_SP_H_
#define __INCLUDE_NUTTX_RF_DAT_31R5_SP_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/irq.h>
#include <nuttx/config.h>
#include <nuttx/rf/ioctl.h>
#include <nuttx/spi/spi.h>

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

/****************************************************************************
 * Name: dat31r5sp_register
 *
 * Description:
 *   Register the dat31r5sp character device as 'devpath'. WARNING:
 *   the DAT-31R5-SP+ is not spi compatible because it hasn't a proper
 *   chip-select input, but it can coexist with other devices on the
 *   spi bus assuming that the LE (Latch Enable) is always 0 when the
 *   device isn't selected. With LE=0 the internal shift-register will
 *   store the last 6 bits sent through the bus, but it will only
 *   change the attenuation level when LE=1. This driver sends the
 *   attenuation bitstream and gives a small positive pulse to LE.
 *
 *   Remember when implementing the corresponding spi select function
 *   when selected == true LE should be 1, and when selected == false
 *   LE should be 0.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/att0"
 *   spi     - An instance of the SPI interface to use to communicate with
 *   spidev  - Number of the spi device (used to drive the Latch Enable pin).
 *
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int dat31r5sp_register(FAR const char *devpath,
                       FAR struct spi_dev_s *spi,
                       int spidev);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_RF_DAT_31R5_SP_H_ */
