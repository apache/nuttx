/****************************************************************************
 * arch/risc-v/src/esp32c3-legacy/esp32c3_ice40.h
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

#ifndef __ARCH_RISCV_SRC_ESP32C3_LEGACY_HARDWARE_ESP32C3_ICE40_H
#define __ARCH_RISCV_SRC_ESP32C3_LEGACY_HARDWARE_ESP32C3_ICE40_H

/****************************************************************************
 * Included Files
 * *************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>

#include <arch/board/board.h>
#include <arch/esp32c3-legacy/chip.h>
#include <nuttx/arch.h>

#include <nuttx/spi/ice40.h>
#include <nuttx/spi/spi.h>

#include "hardware/esp32c3_gpio.h"
#include "hardware/esp32c3_gpio_sigmap.h"

#include "esp32c3_gpio.h"

#ifndef __ASSEMBLY__
#undef EXTERN
#if defined(__cplusplus)
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
 * Name: esp32c3_ice40_initialize
 *
 * Description:
 *  Initialize ICE40 FPGA GPIOs and SPI interface.
 *
 * Input Parameters:
 *  cdone_gpio - GPIO pin connected to the CDONE pin of the ICE40 FPGA.
 *  crst_gpio - GPIO pin connected to the CRST pin of the ICE40 FPGA.
 *  cs_gpio - GPIO pin connected to the CS pin of the ICE40 FPGA.
 *  spi_port - SPI port number to use for communication with the ICE40 FPGA.
 *
 * Returned Value:
 *  A reference to the initialized ICE40 FPGA driver instance.
 *  NULL in case of failure.
 *
 ****************************************************************************/

  struct ice40_dev_s *esp32c3_ice40_initialize(const uint16_t cdone_gpio,
                                               const uint16_t crst_gpio,
                                               const uint16_t cs_gpio,
                                               const uint16_t spi_port);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_ESP32C3_LEGACY_HARDWARE_ESP32C3_ICE40_H */
