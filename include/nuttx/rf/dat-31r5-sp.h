/****************************************************************************
 * include/nuttx/rf/dat-31r5-sp.h
 * Character driver for the Mini-Circuits DAT-31R5-SP+ digital step
 * attenuator.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_RF_DAT_31R5_SP_H
#define __INCLUDE_NUTTX_RF_DAT_31R5_SP_H

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

#endif /* __INCLUDE_NUTTX_RF_DAT_31R5_SP_H */
