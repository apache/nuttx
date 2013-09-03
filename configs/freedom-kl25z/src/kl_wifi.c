/****************************************************************************
 * configs/freedom-kl25z/src/kl_tsi.c
 *
 *   Copyright (C) 2013 Alan Carvalho de Assis
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
 *
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
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/board/kl_wifi.h>

#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>

#include <nuttx/cc3000/spi.h>

#include "up_arch.h"
#include "kl_gpio.h"
#include "chip/kl_pinmux.h"
#include "chip/kl_sim.h"
#include "freedom-kl25z.h"


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/


/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/


/****************************************************************************
 * Private Data
 ****************************************************************************/


/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/*
 * Used by CC3000 driver to read status of WIFI_IRQ
 */
inline long ReadWlanInterruptPin(void)
{
        // Return the status of WIFI_IRQ pin
        return kl_gpioread(GPIO_WIFI_IRQ);
}

/*
 * Enable/Disable WiFi
 */
void WriteWlanEnablePin(uint8_t val)
{
	kl_gpiowrite(GPIO_WIFI_EN, val);
}

/*
 * Assert CC3000 CS
 */
void AssertWlanCS(void)
{
	kl_gpiowrite(GPIO_WIFI_CS, false);
}

/*
 * Deassert CC3000 CS
 */
void DeassertWlanCS(void)
{
	kl_gpiowrite(GPIO_WIFI_CS, true);
}

/****************************************************************************
 * Name: Wlan_Setup
 *
 * Description:
 *   Initialize all pins needed to control CC3000 Module and attach to IRQ
 *
 ****************************************************************************/

void Wlan_Setup(void)
{
  int ret;
  uint32_t regval;

  printf("\nExecuting kl_irq_initialize!\n");

  /* Configure the PIN used to enable the chip */
  kl_configgpio(GPIO_WIFI_EN);

  /* Configure PIN to detect interrupts */
  kl_configgpio(GPIO_WIFI_IRQ);

  /* Configure PIN used as SPI CS */
  kl_configgpio(GPIO_WIFI_CS);

  /* Make sure the chip is OFF before we start */
  WriteWlanEnablePin(false);

  /* Make sure the SPI CS pin is deasserted */
  DeassertWlanCS();

  /* Configure pin to detect interrupt on falling edge */
  regval = getreg32(KL_PORTA_PCR16);
  regval |= PORT_PCR_IRQC_FALLING;
  putreg32(regval, KL_PORTA_PCR16);

  ret = irq_attach(KL_IRQ_PORTA, CC3000InterruptHandler);
  if (ret == OK)
    {
	up_enable_irq(KL_IRQ_PORTA);
    }

}

