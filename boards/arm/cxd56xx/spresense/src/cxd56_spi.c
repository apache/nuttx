/****************************************************************************
 * boards/arm/cxd56xx/spresense/src/cxd56_spi.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "arm_arch.h"
#include "chip.h"
#include "hardware/cxd56_spi.h"
#include "cxd56_clock.h"
#include "cxd56_gpio.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  cxd56_spi0/3/4/5select and cxd56_spi0/3/4/5status
 *
 * Description:
 *   The external functions, cxd56_spi1/2/3select and cxd56_spi1/2/3 status
 *   must be provided by board-specific logic.
 *   They are implementations of the select and status methods of the SPI
 *   interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h).
 *   All other methods (including cxd56_spibus_initialize()) are provided by
 *   common CXD56 logic.  To use this common SPI logic on your board:
 *
 *   1. Provide logic in cxd56_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide cxd56_spi0/3/4/5select() and cxd56_spi0/3/4/5status()
 *      functions in your board-specific logic.
 *      These functions will perform chip selection and status operations
 *      using GPIOs in the way your board is configured.
 *   3. Add a calls to cxd56_spibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by cxd56_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_CXD56_SPI0
void cxd56_spi0select(FAR struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");
}

uint8_t cxd56_spi0status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_CXD56_SPI3
void cxd56_spi3select(FAR struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");

  /* Disable clock gating (clock enable) */

  cxd56_spi_clock_gate_disable(3);

  if (selected)
    {
      putreg32(0, CXD56_SPI3_CS);
    }
  else
    {
      putreg32(1, CXD56_SPI3_CS);
    }

  /* Enable clock gating (clock disable) */

  cxd56_spi_clock_gate_enable(3);
}

uint8_t cxd56_spi3status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_CXD56_SPI4
void cxd56_spi4select(FAR struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");
}

uint8_t cxd56_spi4status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  uint8_t ret = 0;

#  if defined(CONFIG_CXD56_SPISD) && (CONFIG_CXD56_SPISD_SPI_CH == 4)
  ret = board_spisd_status(dev, devid);
#  endif
  return ret;
}
#endif

#ifdef CONFIG_CXD56_SPI5
void cxd56_spi5select(FAR struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");
}

uint8_t cxd56_spi5status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  uint8_t ret = 0;

#  if defined(CONFIG_CXD56_SPISD) && (CONFIG_CXD56_SPISD_SPI_CH == 5)
  ret = board_spisd_status(dev, devid);
#  endif
  return ret;
}
#endif
