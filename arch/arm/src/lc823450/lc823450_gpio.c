/****************************************************************************
 * arch/arm/src/lc823450/lc823450_gpio.c
 *
 *   Copyright 2014,2015,2016,2017 Sony Video & Sound Products Inc.
 *   Author: Masayuki Ishikawa <Masayuki.Ishikawa@jp.sony.com>
 *   Author: Nobutaka Toyoshima <Nobutaka.Toyoshima@jp.sony.com>
 *   Author: Masatoshi Tateishi <Masatoshi.Tateishi@jp.sony.com>
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

#include <arch/board/board.h>
#include <nuttx/config.h>

#include <nuttx/arch.h>
#include <errno.h>
#include <debug.h>

#include "up_arch.h"
#include "lc823450_gpio.h"
#include "lc823450_syscontrol.h"

#ifdef CONFIG_IOEX
#  include <nuttx/ioex.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define  PORTX_OFFSET      0x00001000
#define  PORTX_DRC_OFFSET  0x0
#define  PORTX_DAT_OFFSET  0x4

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_IOEX
static FAR struct ioex_dev_s *g_ioex_dev;
#endif

#ifdef CONFIG_LC823450_VGPIO
#define GPIO_VIRTUAL_NUM 32
static FAR struct vgpio_ops_s *vgpio_ops[GPIO_VIRTUAL_NUM];
#endif /* CONFIG_LC823450_VGPIO */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lc823450_get_gpio_dir
 ****************************************************************************/

static uintptr_t lc823450_get_gpio_dir(unsigned int port)
{
  uintptr_t regaddr =
    PORT0_BASE +
    (port * PORTX_OFFSET) +
    PORTX_DRC_OFFSET;

  return regaddr;
}

/****************************************************************************
 * Name: lc823450_get_gpio_data
 ****************************************************************************/

static uintptr_t lc823450_get_gpio_data(unsigned int port)
{
  uintptr_t regaddr =
    PORT0_BASE +
    (port * PORTX_OFFSET) +
    PORTX_DAT_OFFSET;

  return regaddr;
}

/****************************************************************************
 * Name: lc823450_get_gpio_pull
 ****************************************************************************/

static uintptr_t lc823450_get_gpio_pull(unsigned int port)
{
  uintptr_t regaddr =
    PUDCNT0 + (4 * port);

  return regaddr;
}

/****************************************************************************
 * Name: lc823450_configinput
 *
 * Description:
 *   Configure pull up/down for the GPIO pin
 *
 * Assumptions:
 *   Interrupts are disabled so that read-modify-write operations are safe.
 *
 ****************************************************************************/

static inline void lc823450_configpull(uint16_t gpiocfg,
                                     unsigned int port, unsigned int pin)
{
  uintptr_t regaddr;
  uint32_t  regval;
  uint32_t  pull;

  regaddr = lc823450_get_gpio_pull(port);
  pull    = gpiocfg & GPIO_PUPD_MASK;
  regval  = getreg32(regaddr);
  regval &= ~(3 << (2 * pin)); /* clear the current setting */

  switch (pull)
    {
      case GPIO_FLOAT:
        /* do nothing */
        break;
      case GPIO_PULLUP:
        regval |=  (1 << (2 * pin));
        break;
      case GPIO_PULLDOWN:
        regval |=  (2 << (2 * pin));
        break;
    }

  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: lc823450_configinput
 *
 * Description:
 *   Configure a GPIO pin as an input
 *
 * Assumptions:
 *   Interrupts are disabled so that read-modify-write operations are safe.
 *
 ****************************************************************************/

static inline void lc823450_configinput(uint32_t port, uint32_t pin)
{
  uintptr_t regaddr;
  uint32_t  regval;

  regaddr = lc823450_get_gpio_dir(port);

  /* Then configure the pin as a normal input by clearing the corresponding
   * bit in the rPxDRC register for the port.
   */

  regval  = getreg32(regaddr);
  regval &= ~(1 << pin);
  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: lc823450_configoutput
 *
 * Description:
 *   Configure a GPIO pin as an output.
 *
 * Assumptions:
 *   Interrupts are disabled so that read-modify-write operations are safe.
 *
 ****************************************************************************/

static inline void lc823450_configoutput(uint16_t gpiocfg, uint32_t port,
                                         uint32_t pin)
{
  uintptr_t regaddr;
  uint32_t  regval;

  regaddr = lc823450_get_gpio_dir(port);

  /* Then configure the pin as an output by setting the corresponding
   * bit in the rPxDRC register for the port.
   */

  regval  = getreg32(regaddr);
  regval |= (1 << pin);
  putreg32(regval, regaddr);

  /* Set the initial value of the output */

  lc823450_gpio_write(gpiocfg, GPIO_IS_ONE(gpiocfg));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lc823450_gpio_mux
 *
 * Description:
 *   Change a GPIO pin mux. (only PORT0-5 are supported)
 *
 ****************************************************************************/

int lc823450_gpio_mux(uint16_t gpiocfg)
{
  uint32_t port = ((gpiocfg & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT);
  uint32_t pin  = ((gpiocfg & GPIO_PIN_MASK)  >> GPIO_PIN_SHIFT);
  uint32_t mux  = ((gpiocfg & GPIO_MUX_MASK)  >> GPIO_MUX_SHIFT);
  uint32_t val;
  int      ret = 0;

  if (port <= (GPIO_PORT5 >> GPIO_PORT_SHIFT))
    {
      irqstate_t flags = spin_lock_irqsave();
      val = getreg32(PMDCNT0 + (port * 4));
      val &= ~(3 << (2 * pin));
      val |= (mux << (2 *pin));
      putreg32(val, PMDCNT0 + (port * 4));
      spin_unlock_irqrestore(flags);
    }
  else
    {
      ret = -1;
    }

  return ret;
}

/****************************************************************************
 * Name: lc823450_gpio_config
 *
 * Description:
 *   Configure a GPIO based on bit-encoded description of the pin.  NOTE:
 *   The pin *must* have first been configured for GPIO usage with a
 *   corresponding call to lc823450_pin_config().
 *
 ****************************************************************************/

int lc823450_gpio_config(uint16_t gpiocfg)
{
  uint32_t   port = ((gpiocfg & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT);
  uint32_t   pin  = ((gpiocfg & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT);
  irqstate_t flags;
  int        ret = OK;

#ifdef CONFIG_LC823450_VGPIO
  if (port == (GPIO_PORTV >> GPIO_PORT_SHIFT))
    {
      return OK;
    }
#endif /* CONFIG_LC823450_VGPIO */

  if (port <= (GPIO_PORT5 >> GPIO_PORT_SHIFT))
    {
      DEBUGASSERT(pin < NUM_GPIO_PINS);

      modifyreg32(MCLKCNTAPB, 0, MCLKCNTAPB_PORT0_CLKEN << port);
      modifyreg32(MRSTCNTAPB, 0, MRSTCNTAPB_PORT0_RSTB << port);

      /* Handle the GPIO configuration by the basic mode of the pin */

      flags = spin_lock_irqsave();

      /* pull up/down specified */

      if (gpiocfg & GPIO_PUPD_MASK)
        {
          lc823450_configpull(gpiocfg, port, pin);
        }

      switch (gpiocfg & GPIO_MODE_MASK)
        {
          case GPIO_MODE_INPUT:      /* GPIO input pin */
            lc823450_configinput(port, pin);
            break;

          case GPIO_MODE_OUTPUT:     /* GPIO output pin */
            lc823450_configoutput(gpiocfg, port, pin);
            break;

          default :
            gpioerr("ERROR: Unrecognized pin mode: %04x\n", gpiocfg);
            ret = -EINVAL;
            break;
        }

      spin_unlock_irqrestore(flags);
    }
#ifdef CONFIG_IOEX
  else if (port <= (GPIO_PORTEX >> GPIO_PORT_SHIFT))
    {
      uint32_t dir;
      uint32_t pupd;
      DEBUGASSERT(pin < NUM_GPIOEX_PINS);

      if (GPIO_IS_INPUT(gpiocfg))
        {
          dir = IOEX_DIR_INPUT;

          switch (gpiocfg & GPIO_PUPD_MASK)
            {
              case GPIO_FLOAT:
                pupd = IOEX_PUPD_FLOAT;
                break;
              case GPIO_PULLUP:
                pupd = IOEX_PUPD_PULLUP;
                break;
              case GPIO_PULLDOWN:
                pupd = IOEX_PUPD_PULLDOWN;
                break;
              default:
                DEBUGASSERT(0);
                return -EINVAL;
            }
        }
      else
        {
          dir = IOEX_DIR_OUTPUT;
          pupd = 0;
        }

      ret = g_ioex_dev->ops->config(g_ioex_dev, pin, dir, pupd);
      if (ret != 0)
        {
          gpioerr("ERROR: Failed to configure I/O expanded port\n");
        }

      g_ioex_dev->ops->write(g_ioex_dev, pin, GPIO_IS_ONE(gpiocfg));
    }
#endif /* CONFIG_IOEX */
  else
    {
      ret = -EINVAL;
    }

  return ret;
}

/****************************************************************************
 * Name: lc823450_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void lc823450_gpio_write(uint16_t gpiocfg, bool value)
{
  uint32_t   port = ((gpiocfg & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT);
  uint32_t   pin  = ((gpiocfg & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT);

  uintptr_t  regaddr;
  uint32_t   regval;
  irqstate_t flags;

#ifdef CONFIG_LC823450_VGPIO
  if (port == (GPIO_PORTV >> GPIO_PORT_SHIFT))
    {
      assert(pin < GPIO_VIRTUAL_NUM);
      if (vgpio_ops[pin] && vgpio_ops[pin]->write)
        {
          vgpio_ops[pin]->write(pin, value);
        }

      return;
    }
#endif /* CONFIG_LC823450_VGPIO */

  if (port <= (GPIO_PORT5 >> GPIO_PORT_SHIFT))
    {
      DEBUGASSERT(pin < NUM_GPIO_PINS);

      regaddr = lc823450_get_gpio_data(port);

      flags = spin_lock_irqsave();

      /* Write the value (0 or 1).  To the data register */

      regval  = getreg32(regaddr);

      if (value)
        {
          regval |= (1 << pin);
        }
      else
        {
          regval &= ~(1 << pin);
        }

      putreg32(regval, regaddr);

      spin_unlock_irqrestore(flags);
  }
#ifdef CONFIG_IOEX
  else if (port <= (GPIO_PORTEX >> GPIO_PORT_SHIFT))
    {
      DEBUGASSERT(pin < NUM_GPIOEX_PINS);

      g_ioex_dev->ops->write(g_ioex_dev, pin, value);
    }
#endif /* CONFIG_IOEX */
}

/****************************************************************************
 * Name: lc823450_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool lc823450_gpio_read(uint16_t gpiocfg)
{
  uint32_t   port = ((gpiocfg & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT);
  uint32_t   pin  = ((gpiocfg & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT);

  uintptr_t  regaddr;
  uint32_t   regval;
  bool       value = false;

#ifdef CONFIG_LC823450_VGPIO
  if (port == (GPIO_PORTV >> GPIO_PORT_SHIFT))
    {
      assert(pin < GPIO_VIRTUAL_NUM);
      if (vgpio_ops[pin] && vgpio_ops[pin]->read)
        {
          return vgpio_ops[pin]->read(pin);
        }

      return 0;
    }
#endif /* CONFIG_LC823450_VGPIO */

  if (port <= (GPIO_PORT5 >> GPIO_PORT_SHIFT))
    {
      DEBUGASSERT(pin < NUM_GPIO_PINS);

      /* Get the value of the pin from the pin data register */
      regaddr = lc823450_get_gpio_data(port);
      regval  = getreg32(regaddr);
      value = ((regval >> pin) & 0x01);
    }
#ifdef CONFIG_IOEX
  else if (port <= (GPIO_PORTEX >> GPIO_PORT_SHIFT))
    {
      DEBUGASSERT(pin < NUM_GPIOEX_PINS);

      value = g_ioex_dev->ops->read(g_ioex_dev, pin);
    }
#endif /* CONFIG_IOEX */
  else
    {
      DEBUGASSERT(0);
    }

  return value;
}

/****************************************************************************
 * Name: lc823450_gpio_initialize
 *
 * Description:
 *   Initialize GPIO expander driver
 *
 ****************************************************************************/

#ifdef CONFIG_IOEX
int lc823450_gpio_initialize(void)
{
  g_ioex_dev = up_ioexinitialize(1);
  if (!g_ioex_dev)
    {
      gpioerr("ERROR: Failed to initialize ioex driver\n");
      return -EIO;
    }

  return OK;
}
#endif /* CONFIG_IOEX */

/****************************************************************************
 * Name: lc823450_vgpio_register
 *
 * Description:
 *   Register Virtual GPIO driver
 *
 ****************************************************************************/

#ifdef CONFIG_LC823450_VGPIO
int lc823450_vgpio_register(unsigned int pin, FAR struct vgpio_ops_s *ops)
{
  assert(pin < GPIO_VIRTUAL_NUM);
  vgpio_ops[pin] = ops;
  return OK;
}
#endif /* CONFIG_LC823450_VGPIO */
