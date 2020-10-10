/****************************************************************************
 * arch/risc-v/src/gap8/gap8_gpio.c
 * GAP8 FLL clock generator
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: hhuysqt <1020988872@qq.com>
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

/****************************************************************************
 *  GAP8 has only 1 port. Each pin could be configured to GPIO or alternative
 *  functions.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "gap8_gpio.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gap8_configpin
 *
 * Description:
 *   Configure a pin based on bit-encoded description of the pin.
 *
 * GPIO software abstraction: bitmap configuration of pins
 *
 * |31 18|   17  |     16     |  15 |14   10|9     8|7      0|
 * | --- | drive | pull-up/OD | I/O | GPIOn |  alt  | pinnum |
 * | --- | 1-bit |   1-bit    | 1-b | 5-bit | 2-bit |  8-bit |
 *
 * Returned Value:
 *   OK on success
 *   ERROR on invalid pin.
 *
 ****************************************************************************/

int gap8_configpin(uint32_t cfgset)
{
  uint32_t pinnum    = cfgset & 0xff;
  uint32_t altfunc   = (cfgset >> 8) & 0x3;
  uint32_t pin_dr_pu = (cfgset >> 16) & 0x3;
  uint32_t port_cfg_reg;
  uint32_t pin_alt_reg;
  int shiftcnt;

  if (pinnum > MAX_PIN_NUM)
    {
      return ERROR;
    }

  /* Set pin drive strength (or pin speed in other words) and pulling
   * If it's a GPIO, set input or output.
   *
   * Note that GPIO and non-GPIO uses different register sets...
   * And all the GPIO functions are mapped to ALT-1, and ALT-1 contains
   * only GPIO functions...
   */

  port_cfg_reg = PORTA->PADCFG[pinnum >> 2];
  shiftcnt = (pinnum & 0x3) << 3;
  port_cfg_reg &=     ~(0x3 << shiftcnt);
  port_cfg_reg |= pin_dr_pu << shiftcnt;
  PORTA->PADCFG[pinnum >> 2] = port_cfg_reg;

  if (altfunc == 1)
    {
      uint32_t gpio_n = (cfgset >> 10) & 0x1f;
      uint32_t gpio_dir = (cfgset >> 15) & 0x1;
      uint32_t tmp;

      /* It must be a GPIO */

      GPIOA->EN |= (1L << gpio_n);

      tmp = GPIOA->PADCFG[gpio_n >> 2];
      shiftcnt = (gpio_n & 0x3) << 3;
      tmp &=     ~(0x3 << shiftcnt);
      tmp |= pin_dr_pu << shiftcnt;
      GPIOA->PADCFG[gpio_n >> 2] = tmp;

      tmp = GPIOA->DIR;
      tmp &= ~(1L << gpio_n);
      tmp |= gpio_dir << gpio_n;
      GPIOA->DIR = tmp;
    }

  /* Set pin alternative function */

  pin_alt_reg = PORTA->PADFUN[pinnum >> 4];
  shiftcnt = (pinnum & 0xf) << 1;
  pin_alt_reg &=   ~(0x3 << shiftcnt);
  pin_alt_reg |= altfunc << shiftcnt;
  PORTA->PADFUN[pinnum >> 4] = pin_alt_reg;

  return OK;
}

/****************************************************************************
 * Name: gap8_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 * Bit encoded pinset:
 *
 * |31 15|14   10|9   0|
 * | --- | GPIOn | --- |
 * | --- | 5-bit | --- |
 *
 ****************************************************************************/

void gap8_gpiowrite(uint32_t pinset, bool value)
{
  uint32_t gpio_n = (pinset >> 10) & 0x1f;
  if (value)
    {
      GPIOA->OUT |= (1L << gpio_n);
    }
  else
    {
      GPIOA->OUT &= ~(1L << gpio_n);
    }
}

/****************************************************************************
 * Name: gap8_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 * Bit encoded pinset:
 *
 * |31 15|14   10|9   0|
 * | --- | GPIOn | --- |
 * | --- | 5-bit | --- |
 *
 ****************************************************************************/

bool gap8_gpioread(uint32_t pinset)
{
  uint32_t gpio_n = (pinset >> 10) & 0x1f;
  return (GPIOA->IN >> gpio_n) & 0x1;
}

/****************************************************************************
 * Name: gap8_gpioirqset
 *
 * Description:
 *   Enable or disable interrupt on GPIO
 *
 * Bit encoded pinset:
 *
 * |31 20|19     18|17 15|14   10|9   0|
 * | --- | int-typ | --- | GPIOn | --- |
 * | --- |  2-bit  | --- | 5-bit | --- |
 *
 ****************************************************************************/

void gap8_gpioirqset(uint32_t pinset, bool enable)
{
  uint32_t gpio_n = (pinset >> 10) & 0x1f;
  uint32_t int_type = (pinset >> 18) * 0x3;
  uint32_t tmp;
  uint32_t shitfcnt;

  if (enable)
    {
      shitfcnt = (gpio_n & 0xf) << 1;
      tmp  = GPIOA->INTCFG[gpio_n >> 4];
      tmp &= ~(0x3 << shitfcnt);
      tmp |= int_type << shitfcnt;
      GPIOA->INTCFG[gpio_n >> 4] = tmp;

      GPIOA->INTEN |= (1L << gpio_n);
    }
}
