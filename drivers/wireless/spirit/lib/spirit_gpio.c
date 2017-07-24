/******************************************************************************
 * drivers/wireless/spirit/lib/spirit_gpio.c
 * This file provides all the low level API to manage SPIRIT GPIO.
 *
 *  Copyright(c) 2015 STMicroelectronics
 *  Author: VMA division - AMS
 *  Version 3.2.2 08-July-2015
 *
 *  Adapted for NuttX by:
 *  Author:  Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include <stdint.h>
#include <assert.h>

#include "spirit_gpio.h"
#include "spirit_spi.h"

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

/******************************************************************************
 * Private Functions
 ******************************************************************************/

/******************************************************************************
 * Name:
 *
 * Description:
 *
 * Parameters:
 *
 * Returned Value:
 *
 ******************************************************************************/

/******************************************************************************
 * Public Functions
 ******************************************************************************/

/******************************************************************************
 * Name:
 *
 * Description:
 *   Initializes the Spirit GPIOx according to the specified parameters in
 *   the gpioinit parameter.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   gpioinit - A pointer to a struct spirit_gpio_init_s structure that
 *              contains the configuration information for the specified
 *              SPIRIT GPIO.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_gpio_initialize(FAR struct spirit_library_s *spirit,
                           FAR const struct spirit_gpio_init_s *gpioinit)
{
  uint8_t regval = 0x00;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_GPIO(gpioinit->gpiopin));
  DEBUGASSERT(IS_SPIRIT_GPIO_MODE(gpioinit->gpiomode));
  DEBUGASSERT(IS_SPIRIT_GPIO_IO(gpioinit->gpioio));

  regval = ((uint8_t)(gpioinit->gpiomode) | (uint8_t)(gpioinit->gpioio));
  return spirit_reg_write(spirit, gpioinit->gpiopin, &regval, 1);
}
