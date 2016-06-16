/****************************************************************************
 * arch/arm/src/tiva/tiva_gpio.c
 *
 *   Copyright (C) 2009-2010, 2014-2015 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>

#include "up_arch.h"
#include "tiva_enablepwr.h"
#include "tiva_enableclks.h"
#include "tiva_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* These definitions are part of the implementation of the  GPIO pad
 * configuration of Table 9-1 in the LM3S6918 data sheet.
 */

#define AMSEL_SHIFT            6
#define AMSEL_1                (1 << AMSEL_SHIFT) /* Set/clear bit in GPIO AMSEL register */
#define AMSEL_0                0
#define AMSEL_X                0

#define AFSEL_SHIFT            5
#define AFSEL_1                (1 << AFSEL_SHIFT) /* Set/clear bit in GPIO AFSEL register */
#define AFSEL_0                0
#define AFSEL_X                0

#define DIR_SHIFT              4
#define DIR_1                  (1 << DIR_SHIFT)   /* Set/clear bit in GPIO DIR register */
#define DIR_0                  0
#define DIR_X                  0

#define ODR_SHIFT              3
#define ODR_1                  (1 << ODR_SHIFT)   /* Set/clear bit in GPIO ODR register */
#define ODR_0                  0
#define ODR_X                  0

#define DEN_SHIFT              2
#define DEN_1                  (1 << DEN_SHIFT)   /* Set/clear bit in GPIO DEN register */
#define DEN_0                  0
#define DEN_X                  0

#define PUR_SHIFT              1
#define PUR_1                  (1 << PUR_SHIFT)   /* Set/clear bit in GPIO PUR register */
#define PUR_0                  0
#define PUR_X                  0

#define PDR_SHIFT              0
#define PDR_1                  (1 << PDR_SHIFT)   /* Set/clear bit in GPIO PDR register */
#define PDR_0                  0
#define PDR_X                  0

#define GPIO_INPUT_SETBITS     (AMSEL_0 | AFSEL_0 | DIR_0 | ODR_0 | DEN_1 | PUR_X | PDR_X)
#define GPIO_INPUT_CLRBITS     (AMSEL_1 | AFSEL_1 | DIR_1 | ODR_1 | DEN_0 | PUR_X | PDR_X)

#define GPIO_OUTPUT_SETBITS    (AMSEL_0 | AFSEL_0 | DIR_1 | ODR_0 | DEN_1 | PUR_X | PDR_X)
#define GPIO_OUTPUT_CLRBITS    (AMSEL_1 | AFSEL_1 | DIR_0 | ODR_1 | DEN_0 | PUR_X | PDR_X)

#define GPIO_ODINPUT_SETBITS   (AMSEL_0 | AFSEL_0 | DIR_0 | ODR_1 | DEN_1 | PUR_X | PDR_X)
#define GPIO_ODINPUT_CLRBITS   (AMSEL_1 | AFSEL_1 | DIR_1 | ODR_0 | DEN_0 | PUR_X | PDR_X)

#define GPIO_ODOUTPUT_SETBITS  (AMSEL_0 | AFSEL_0 | DIR_1 | ODR_1 | DEN_1 | PUR_X | PDR_X)
#define GPIO_ODOUTPUT_CLRBITS  (AMSEL_1 | AFSEL_1 | DIR_0 | ODR_0 | DEN_0 | PUR_X | PDR_X)

#define GPIO_PFODIO_SETBITS    (AMSEL_0 | AFSEL_1 | DIR_X | ODR_1 | DEN_1 | PUR_X | PDR_X)
#define GPIO_PFODIO_CLRBITS    (AMSEL_1 | AFSEL_0 | DIR_X | ODR_0 | DEN_0 | PUR_X | PDR_X)

#define GPIO_PFIO_SETBITS      (AMSEL_0 | AFSEL_1 | DIR_X | ODR_0 | DEN_1 | PUR_X | PDR_X)
#define GPIO_PFIO_CLRBITS      (AMSEL_1 | AFSEL_0 | DIR_X | ODR_1 | DEN_0 | PUR_X | PDR_X)

#define GPIO_ANINPUT_SETBITS   (AMSEL_1 | AFSEL_0 | DIR_0 | ODR_0 | DEN_0 | PUR_0 | PDR_0)
#define GPIO_ANINPUT_CLRBITS   (AMSEL_0 | AFSEL_1 | DIR_1 | ODR_1 | DEN_1 | PUR_1 | PDR_1)

#define GPIO_INTERRUPT_SETBITS (AMSEL_0 | AFSEL_0 | DIR_0 | ODR_0 | DEN_1 | PUR_X | PDR_X)
#define GPIO_INTERRUPT_CLRBITS (AMSEL_1 | AFSEL_1 | DIR_1 | ODR_1 | DEN_0 | PUR_X | PDR_X)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct gpio_func_s
{
  uint8_t setbits;  /* A set of GPIO register bits to set */
  uint8_t clrbits;  /* A set of GPIO register bits to clear */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct gpio_func_s g_funcbits[] =
{
  {GPIO_INPUT_SETBITS,     GPIO_INPUT_CLRBITS},     /* GPIO_FUNC_INPUT */
  {GPIO_OUTPUT_SETBITS,    GPIO_OUTPUT_CLRBITS},    /* GPIO_FUNC_OUTPUT */
  {GPIO_ODINPUT_SETBITS,   GPIO_ODINPUT_CLRBITS},   /* GPIO_FUNC_ODINPUT */
  {GPIO_ODOUTPUT_SETBITS,  GPIO_ODOUTPUT_CLRBITS},  /* GPIO_FUNC_ODOUTPUT */
  {GPIO_PFODIO_SETBITS,    GPIO_PFODIO_CLRBITS},    /* GPIO_FUNC_PFODIO */
  {GPIO_PFIO_SETBITS,      GPIO_PFIO_CLRBITS},      /* GPIO_FUNC_PFIO */
  {GPIO_ANINPUT_SETBITS,   GPIO_ANINPUT_CLRBITS},   /* GPIO_FUNC_ANINPUT */
  {GPIO_INTERRUPT_SETBITS, GPIO_INTERRUPT_CLRBITS}, /* GPIO_FUNC_INTERRUPT */
};

/* NOTE: this is duplicated in tiva_dumpgpio.c */

static const uintptr_t g_gpiobase[TIVA_NPORTS] =
{
#if TIVA_NPORTS > 0
    TIVA_GPIOA_BASE
#endif
#if TIVA_NPORTS > 1
  , TIVA_GPIOB_BASE
#endif
#if TIVA_NPORTS > 2
  , TIVA_GPIOC_BASE
#endif
#if TIVA_NPORTS > 3
  , TIVA_GPIOD_BASE
#endif
#if TIVA_NPORTS > 4
  , TIVA_GPIOE_BASE
#endif
#if TIVA_NPORTS > 5
  , TIVA_GPIOF_BASE
#endif
#if TIVA_NPORTS > 6
  , TIVA_GPIOG_BASE
#endif
#if TIVA_NPORTS > 7
  , TIVA_GPIOH_BASE
#endif
#if TIVA_NPORTS > 8
  , TIVA_GPIOJ_BASE
#endif
#if TIVA_NPORTS > 9
  , TIVA_GPIOK_BASE
#endif
#if TIVA_NPORTS > 10
  , TIVA_GPIOL_BASE
#endif
#if TIVA_NPORTS > 11
  , TIVA_GPIOM_BASE
#endif
#if TIVA_NPORTS > 12
  , TIVA_GPION_BASE
#endif
#if TIVA_NPORTS > 13
  , TIVA_GPIOP_BASE
#endif
#if TIVA_NPORTS > 14
  , TIVA_GPIOQ_BASE
#endif
#if TIVA_NPORTS > 15
  , TIVA_GPIOR_BASE
#endif
#if TIVA_NPORTS > 16
  , TIVA_GPIOS_BASE
#endif
#if TIVA_NPORTS > 17
  , TIVA_GPIOT_BASE
#endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tiva_gpiobaseaddress
 *
 * Description:
 *   Given a GPIO enumeration value, return the base address of the
 *   associated GPIO registers.
 *
 ****************************************************************************/

inline uintptr_t tiva_gpiobaseaddress(unsigned int port)
{
  uintptr_t gpiobase = 0;
  if (port < TIVA_NPORTS)
    {
      gpiobase = g_gpiobase[port];
    }

  return gpiobase;
}

/****************************************************************************
 * Name: tiva_gpiofunc
 *
 * Description:
 *   Configure GPIO registers for a specific function. Overwrites certain
 *   padtype configurations.
 *
 ****************************************************************************/

static void tiva_gpiofunc(uint32_t base, uint32_t pinno,
                          const struct gpio_func_s *func)
{
  uint32_t setbit;
  uint32_t clrbit;

  /* Set/clear/ignore the GPIO DIR bit. "The GPIODIR register is the data
   * direction register. Bits set to 1 in the GPIODIR register configure
   * the corresponding pin to be an output, while bits set to 0 configure
   * the pins to be inputs. All bits are cleared by a reset, meaning all
   * GPIO pins are inputs by default.
   */

  setbit = (((uint32_t)func->setbits >> DIR_SHIFT) & 1) << pinno;
  clrbit = (((uint32_t)func->clrbits >> DIR_SHIFT) & 1) << pinno;

  if (setbit || clrbit)
    {
      modifyreg32(base + TIVA_GPIO_DIR_OFFSET, clrbit, setbit);
    }

  /* Set/clear/ignore the GPIO AFSEL bit. "The GPIOAFSEL register is the
   * mode control select register. Writing a 1 to any bit in this register
   * selects the hardware control for the corresponding GPIO line. All bits
   * are cleared by a reset, therefore no GPIO line is set to hardware
   * control by default."
   *
   * NOTE: In order to set JTAG/SWD GPIOs, it is also necessary to lock,
   * commit and unlock the GPIO.  That is not implemented here.
   */

  setbit = (((uint32_t)func->setbits >> AFSEL_SHIFT) & 1) << pinno;
  clrbit = (((uint32_t)func->clrbits >> AFSEL_SHIFT) & 1) << pinno;

  if (setbit || clrbit)
    {
      modifyreg32(base + TIVA_GPIO_AFSEL_OFFSET, clrbit, setbit);
    }

  /* Set/clear/ignore the GPIO ODR bit. "The GPIO ODR register is the open
   * drain control register. Setting a bit in this register enables the open
   * drain configuration of the corresponding GPIO pad. When open drain mode
   * is enabled, the corresponding bit should also be set in the GPIO Digital
   * Input Enable (GPIO DEN) register ... Corresponding bits in the drive
   * strength registers (GPIO DR2R, GPIO DR4R, GPIO DR8R, and GPIO SLR ) can
   * be set to achieve the desired rise and fall times. The GPIO acts as an
   * open drain input if the corresponding bit in the GPIO DIR register is
   * set to 0; and as an open drain output when set to 1."
   */

  setbit = (((uint32_t)func->setbits >> ODR_SHIFT) & 1) << pinno;
  clrbit = (((uint32_t)func->clrbits >> ODR_SHIFT) & 1) << pinno;

  if (setbit || clrbit)
    {
      modifyreg32(base + TIVA_GPIO_ODR_OFFSET, clrbit, setbit);
    }

  /* Set/clear the GPIO PUR bit. "The GPIOPUR register is the pull-up
   * control register. When a bit is set to 1, it enables a weak pull-up
   * resistor on the corresponding GPIO signal. Setting a bit in GPIOPUR
   * automatically clears the corresponding bit in the GPIO Pull-Down
   * Select (GPIOPDR) register ..."
   */

  setbit = (((uint32_t)func->setbits >> PUR_SHIFT) & 1) << pinno;
  clrbit = (((uint32_t)func->clrbits >> PUR_SHIFT) & 1) << pinno;

  if (setbit || clrbit)
    {
      modifyreg32(base + TIVA_GPIO_PUR_OFFSET, clrbit, setbit);
    }

  /* Set/clear the GPIO PDR bit. "The GPIOPDR register is the pull-down
   * control register. When a bit is set to 1, it enables a weak pull-down
   * resistor on the corresponding GPIO signal. Setting a bit in GPIOPDR
   * automatically clears the corresponding bit in the GPIO Pull-Up Select
   * (GPIOPUR) register ..."
   */

  setbit = (((uint32_t)func->setbits >> PDR_SHIFT) & 1) << pinno;
  clrbit = (((uint32_t)func->clrbits >> PDR_SHIFT) & 1) << pinno;

  if (setbit || clrbit)
    {
      modifyreg32(base + TIVA_GPIO_PDR_OFFSET, clrbit, setbit);
    }

  /* Set/clear the GPIO DEN bit. "The GPIODEN register is the digital enable
   * register. By default, with the exception of the GPIO signals used for
   * JTAG/SWD function, all other GPIO signals are configured out of reset
   * to be undriven (tristate). Their digital function is disabled; they do
   * not drive a logic value on the pin and they do not allow the pin voltage
   * into the GPIO receiver.  To use the pin in a digital function (either
   * GPIO or alternate function), the corresponding GPIODEN bit must be set."
   */

  setbit = (((uint32_t)func->setbits >> DEN_SHIFT) & 1) << pinno;
  clrbit = (((uint32_t)func->clrbits >> DEN_SHIFT) & 1) << pinno;

  if (setbit || clrbit)
    {
      modifyreg32(base + TIVA_GPIO_DEN_OFFSET, clrbit, setbit);
    }

#if defined(LM4F) || defined(TM4C)
  /* Set/clear/ignore the GPIO AMSEL bit. "The GPIOAMSEL register controls
   * isolation circuits to the analog side of a unified I/O pad. Because
   * the GPIOs may be driven by a 5-V source and affect analog operation,
   * analog circuitry requires isolation from the pins when they are not
   * used in their analog function.  Each bit of this register controls the
   * isolation circuitry for the corresponding GPIO signal.
   */

  setbit = (((uint32_t)func->setbits >> AMSEL_SHIFT) & 1) << pinno;
  clrbit = (((uint32_t)func->clrbits >> AMSEL_SHIFT) & 1) << pinno;

  if (setbit || clrbit)
    {
      modifyreg32(base + TIVA_GPIO_AMSEL_OFFSET, clrbit, setbit);
    }
#endif
}

/****************************************************************************
 * Name: tiva_gpiopadstrength
 *
 * Description:
 *   Set up pad strength and pull-ups
 *
 ****************************************************************************/

static inline void tiva_gpiopadstrength(uint32_t base, uint32_t pin,
                                        uint32_t pinset)
{
  int strength = (pinset & GPIO_STRENGTH_MASK);
  uint32_t slrset  = 0;
  uint32_t slrclr  = 0;
  uint32_t dr2rset = 0;
  uint32_t dr2rclr = 0;
  uint32_t dr4rset = 0;
  uint32_t dr4rclr = 0;
  uint32_t dr8rset = 0;
  uint32_t dr8rclr = 0;

  /* Set the output drive strength. */

  switch (strength)
    {
      case GPIO_STRENGTH_2MA:
        {
          dr2rset = pin;
          dr4rclr = pin;
          dr8rclr = pin;
          slrclr = pin;
        }
        break;

      case GPIO_STRENGTH_4MA:
        {
          dr2rclr = pin;
          dr4rset = pin;
          dr8rclr = pin;
          slrclr = pin;
        }
        break;

      case GPIO_STRENGTH_8MA:
        {
          dr2rclr = pin;
          dr4rclr = pin;
          dr8rset = pin;
          slrclr = pin;
        }
        break;

      case GPIO_STRENGTH_8MASC:
        {
          dr2rclr = pin;
          dr4rclr = pin;
          dr8rset = pin;
          slrset = pin;
        }
        break;

#ifdef CONFIG_ARCH_CHIP_TM4C129
#  if 0
      case GPIO_STRENGTH_10MA:
        {
        }
        break;

      case GPIO_STRENGTH_10MASC:
        {
        }
        break;

      case GPIO_STRENGTH_12MA:
        {
        }
        break;

      case GPIO_STRENGTH_12MASC:
        {
        }
        break;

#  endif
#endif
      default:
        break;
    }

  modifyreg32(base + TIVA_GPIO_DR2R_OFFSET, dr2rclr, dr2rset);
  modifyreg32(base + TIVA_GPIO_DR4R_OFFSET, dr4rclr, dr4rset);
  modifyreg32(base + TIVA_GPIO_DR8R_OFFSET, dr8rclr, dr8rset);
  modifyreg32(base + TIVA_GPIO_SLR_OFFSET,  slrclr,  slrset);

#ifdef CONFIG_ARCH_CHIP_TM4C129
  /* TODO: Add TM4C129 registers (TIVA_GPIO_DR12R) */
#  if 0
  /* Set the 12-mA drive select register.  This register only appears in
   * TM4E111 and later device classes, but is a harmless write on older
   * devices.
   */

  /* Set the GPIO peripheral configuration register first as required.  This
   * register only appears in TM4E111 and later device classes, but is a
   * harmless write on older devices. Walk pins 0-7 and clear or set the
   * provided PC[EDMn] encoding.
   */
#  endif // 0
#endif
}

/****************************************************************************
 * Name: tiva_gpiopadtype
 *
 * Description:
 *   Set up pad strength and pull-ups.  Some of these values may be over-
 *   written by tiva_gpiofunc, depending on the function selection.  Others
 *   are optional for different function selections.
 *
 ****************************************************************************/

static inline void tiva_gpiopadtype(uint32_t base, uint32_t pin,
                                    uint32_t pinset)
{
  int padtype       = (pinset & GPIO_PADTYPE_MASK);
  uint32_t odrset   = 0;
  uint32_t odrclr   = 0;
  uint32_t purset   = 0;
  uint32_t purclr   = 0;
  uint32_t pdrset   = 0;
  uint32_t pdrclr   = 0;
  uint32_t denset   = 0;
  uint32_t denclr   = 0;
#if defined(LM4F) || defined(TM4C)
  uint32_t amselset = 0;
  uint32_t amselclr = 0;
#endif

  /* Set the pin type. */

  switch (padtype)
    {
      case GPIO_PADTYPE_STD:
        {
          odrclr = pin;
          purclr = pin;
          pdrclr = pin;
          denset = pin;
#if defined(LM4F) || defined(TM4C)
          amselclr = pin;
#endif
        }
        break;

      case GPIO_PADTYPE_STDWPU:
        {
          odrclr = pin;
          purset = pin;
          pdrclr = pin;
          denset = pin;
#if defined(LM4F) || defined(TM4C)
          amselclr = pin;
#endif
        }
        break;

      case GPIO_PADTYPE_STDWPD:
        {
          odrclr = pin;
          purclr = pin;
          pdrset = pin;
          denset = pin;
#if defined(LM4F) || defined(TM4C)
          amselclr = pin;
#endif
        }
        break;

      case GPIO_PADTYPE_OD:
        {
          odrset = pin;
          purclr = pin;
          pdrclr = pin;
          denset = pin;
#if defined(LM4F) || defined(TM4C)
          amselclr = pin;
#endif
        }
        break;

      case GPIO_PADTYPE_ODWPU:
        {
          odrset = pin;
          purset = pin;
          pdrclr = pin;
          denclr = pin;
#if defined(LM4F) || defined(TM4C)
          amselclr = pin;
#endif
        }
        break;

      case GPIO_PADTYPE_ODWPD:
        {
          odrset = pin;
          purclr = pin;
          pdrset = pin;
          denclr = pin;
#if defined(LM4F) || defined(TM4C)
          amselclr = pin;
#endif
        }
        break;

      case GPIO_PADTYPE_ANALOG:
        {
          odrclr = pin;
          purclr = pin;
          pdrclr = pin;
          denclr = pin;
#if defined(LM4F) || defined(TM4C)
          amselset = pin;
#endif
        }
        break;

      default:
        break;
    }

  modifyreg32(base + TIVA_GPIO_ODR_OFFSET,   odrclr,   odrset);
  modifyreg32(base + TIVA_GPIO_PUR_OFFSET,   purclr,   purset);
  modifyreg32(base + TIVA_GPIO_PDR_OFFSET,   pdrclr,   pdrset);
  modifyreg32(base + TIVA_GPIO_DEN_OFFSET,   denclr,   denset);

#if defined(LM4F) || defined(TM4C)
  modifyreg32(base + TIVA_GPIO_AMSEL_OFFSET, amselclr, amselset);
#endif

#ifdef CONFIG_ARCH_CHIP_TM4C129
  /* Set the wake pin enable register and the wake level register.  These
   * registers only appear in TM4E111 and later device classes, but are
   * harmless writes on older devices.
   */
#endif
}

/****************************************************************************
 * Name: tiva_initoutput
 *
 * Description:
 *   Set the GPIO output value
 *
 ****************************************************************************/

static inline void tiva_initoutput(uint32_t pinset)
{
  bool value = ((pinset & GPIO_VALUE_MASK) != GPIO_VALUE_ZERO);
  tiva_gpiowrite(pinset, value);
}

/****************************************************************************
 * Name: tiva_interrupt
 *
 * Description:
 *   Configure the interrupt pin.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_GPIO_IRQS
static inline void tiva_interrupt(uint32_t pinset)
{
  uint8_t   port    = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  uint8_t   pin     = 1 << (pinset & GPIO_PIN_MASK);
  uintptr_t base    = tiva_gpiobaseaddress(port);
  uint32_t  inttype = pinset & GPIO_INT_MASK;

  uint32_t  isset   = 0;
  uint32_t  ibeset  = 0;
  uint32_t  ievset  = 0;
  uint32_t  isclr   = 0;
  uint32_t  ibeclr  = 0;
  uint32_t  ievclr  = 0;

  /* Mask and clear the GPIO interrupt */

  tiva_gpioirqdisable(port, pin);
  tiva_gpioirqclear(port, pin);

  /* handle according to the selected interrupt type */

  switch (inttype)
    {
      case GPIO_INT_FALLINGEDGE:
        {
          isclr  = pin;
          ibeclr = pin;
          ievclr = pin;
        }
        break;

      case GPIO_INT_RISINGEDGE:
        {
          isclr  = pin;
          ibeclr = pin;
          ievset = pin;
        }
        break;

      case GPIO_INT_BOTHEDGES:
        {
          isclr  = pin;
          ibeset = pin;
          ievclr = pin;
        }
        break;

      case GPIO_INT_LOWLEVEL:
        {
          isset  = pin;
          ibeclr = pin;
          ievclr = pin;
        }
        break;

      case GPIO_INT_HIGHLEVEL:
        {
          isset  = pin;
          ibeclr = pin;
          ievset = pin;
        }
        break;

      default:
        break;
    }

  /* "The GPIO IBE register is the interrupt both-edges register. When the
   * corresponding bit in the GPIO Interrupt Sense (GPIO IS) register ... is
   * set to detect edges, bits set to High in GPIO IBE configure the
   * corresponding pin to detect both rising and falling edges, regardless
   * of the corresponding bit in the GPIO Interrupt Event (GPIO IEV) register ...
   * Clearing a bit configures the pin to be controlled by GPIOIEV. All bits
   * are cleared by a reset.
   */

  modifyreg32(base + TIVA_GPIO_IBE_OFFSET, ibeclr, ibeset);

  /* "The GPIO IS register is the interrupt sense register. Bits set to
   * 1 in GPIOIS configure the corresponding pins to detect levels, while
   * bits set to 0 configure the pins to detect edges. All bits are cleared
   * by a reset.
   */

  modifyreg32(base + TIVA_GPIO_IS_OFFSET, isclr, isset);

  /* "The GPIOIEV register is the interrupt event register. Bits set to
   * High in GPIO IEV configure the corresponding pin to detect rising edges
   * or high levels, depending on the corresponding bit value in the GPIO
   * Interrupt Sense (GPIO IS) register... Clearing a bit configures the pin to
   * detect falling edges or low levels, depending on the corresponding bit
   * value in GPIOIS. All bits are cleared by a reset.
   */

  modifyreg32(base + TIVA_GPIO_IEV_OFFSET, ievclr, ievset);

#ifdef CONFIG_DEBUG_GPIO_INFO
  uint32_t regval;

  gpioinfo("reg expected actual: [interrupt type=%d]\n", inttype);

  regval = (getreg32(base+TIVA_GPIO_IS_OFFSET) & pin) ? pin : 0;
  gpioinfo("IS  0x%08x 0x%08x\n", isset, regval);

  regval = (getreg32(base+TIVA_GPIO_IBE_OFFSET) & pin) ? pin : 0;
  gpioinfo("IBE 0x%08x 0x%08x\n", ibeset, regval);

  regval = (getreg32(base+TIVA_GPIO_IEV_OFFSET) & pin) ? pin : 0;
  gpioinfo("IEV 0x%08x 0x%08x\n", ievset, regval);
#endif
}
#endif

/****************************************************************************
 * Name: tiva_portcontrol
 *
 * Description:
 *   Set the pin alternate function in the port control register.
 *
 ****************************************************************************/

#if defined(LM4F) || defined(TM4C)
static inline void tiva_portcontrol(uint32_t base, uint32_t pinno,
                                    uint32_t pinset,
                                    const struct gpio_func_s *func)
{
  uint32_t alt = 0;
  uint32_t mask;
  uint32_t regval;

  /* Is this pin an alternate function pin? */

  if ((func->setbits & AFSEL_1) != 0)
    {
      /* Yes, extract the alternate function number from the pin
       * configuration.
       */

      alt = (pinset & GPIO_ALT_MASK) >> GPIO_ALT_SHIFT;
    }

  /* Set the alternate function in the port control register */

  regval  = getreg32(base + TIVA_GPIO_PCTL_OFFSET);
  mask    = GPIO_PCTL_PMC_MASK(pinno);
  regval &= ~mask;
  regval |= (alt << GPIO_PCTL_PMC_SHIFT(pinno)) & mask;
  putreg32(regval, base + TIVA_GPIO_PCTL_OFFSET);
}
#else
#  define tiva_portcontrol(b,p,c,f)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tiva_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int tiva_configgpio(uint32_t pinset)
{
  irqstate_t   flags;
  unsigned int func;
  unsigned int port;
  unsigned int pinno;
  uintptr_t    base;
  uint32_t     pin;

  /* Decode the basics */

  func  = (pinset & GPIO_FUNC_MASK) >> GPIO_FUNC_SHIFT;
  port  = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  pinno = (pinset & GPIO_PIN_MASK);
  pin   = (1 << pinno);

  DEBUGASSERT(func <= GPIO_FUNC_MAX);

  /* Get the base address associated with the GPIO port */

  base = tiva_gpiobaseaddress(port);
  DEBUGASSERT(base != 0);

  /* The following requires exclusive access to the GPIO registers */

  flags = enter_critical_section();

  /* Enable power and clocking for this GPIO peripheral.
   *
   * - Enable Power (TM4C129 family only):  Applies power (only) to the GPIO
   *   peripheral.  This is not an essential step since enabling clocking
   *   will also apply power.  The only significance is that the GPIO state
   *   will be retained if the GPIO clocking is subsequently disabled.
   * - Enable Clocking (All families):  Applies both power and clocking to
   *   the GPIO peripheral, bringing it a fully functional state.
   */

  tiva_gpio_enablepwr(port);
  tiva_gpio_enableclk(port);

  /* First, set the port to digital input.  This is the safest state in which
   * to perform reconfiguration.
   */

  tiva_gpiofunc(base, pinno, &g_funcbits[0]);
  tiva_portcontrol(base, pinno, pinset, &g_funcbits[0]);

  /* Then set up pad strengths and pull-ups.  These setups should be done before
   * setting up the function because some function settings will over-ride these
   * user options.
   */

  tiva_gpiopadstrength(base, pin, pinset);
  tiva_gpiopadtype(base, pin, pinset);

  /* Then set up the real pin function */

  tiva_gpiofunc(base, pinno, &g_funcbits[func]);
  tiva_portcontrol(base, pinno, pinset, &g_funcbits[func]);

  /* Special case GPIO digital output pins */

  if (func == 1 || func == 3 || func == 5)
    {
      tiva_initoutput(pinset);
    }

#ifdef CONFIG_TIVA_GPIO_IRQS
  /* Special setup for interrupt GPIO pins */

  else if (func == 7)
    {
      tiva_interrupt(pinset);
    }
#endif

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: tiva_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void tiva_gpiowrite(uint32_t pinset, bool value)
{
  unsigned int port;
  unsigned int pinno;
  uintptr_t    base;

  /* Decode the basics */

  port  = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  pinno = (pinset & GPIO_PIN_MASK);

  /* Get the base address associated with the GPIO port */

  base  = tiva_gpiobaseaddress(port);

  /* "The GPIO DATA register is the data register. In software control mode,
   *  values written in the GPIO DATA register are transferred onto the GPIO
   *  port pins if the respective pins have been configured as outputs through
   *  the GPIO Direction (GPIO DIR) register ...
   *
   * "In order to write to GPIO DATA, the corresponding bits in the mask,
   *  resulting from the address bus bits [9:2], must be High. Otherwise, the
   *  bit values remain unchanged by the write.
   *
   * "... All bits are cleared by a reset."
   */

  putreg32((uint32_t)value << pinno, base + TIVA_GPIO_DATA_OFFSET + (1 << (pinno + 2)));
}

/****************************************************************************
 * Name: tiva_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool tiva_gpioread(uint32_t pinset)
{
  unsigned int port;
  unsigned int pinno;
  uintptr_t    base;

  /* Decode the basics */

  port  = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  pinno = (pinset & GPIO_PIN_MASK);

  /* Get the base address associated with the GPIO port */

  base = tiva_gpiobaseaddress(port);

  /* "... the values read from this register are determined for each bit
   *  by the mask bit derived from the address used to access the data register,
   *  bits [9:2]. Bits that are 1 in the address mask cause the corresponding
   *  bits in GPIODATA to be read, and bits that are 0 in the address mask cause
   *  the corresponding bits in GPIO DATA to be read as 0, regardless of their
   *  value.
   *
   * "A read from GPIO DATA returns the last bit value written if the respective
   *  pins are configured as outputs, or it returns the value on the
   *  corresponding input pin when these are configured as inputs. All bits
   *  are cleared by a reset."
   */
  return (getreg32(base + TIVA_GPIO_DATA_OFFSET + (1 << (pinno + 2))) != 0);
}

/****************************************************************************
 * Name: tiva_gpio_lockport
 *
 * Description:
 *   Certain pins require to be unlocked from the NMI to use for normal GPIO
 *   use. See table 10-10 in datasheet for pins with special considerations.
 *
 ****************************************************************************/

void tiva_gpio_lockport(uint32_t pinset, bool lock)
{
  unsigned int port;
  unsigned int pinno;
  uintptr_t    base;
  uint32_t     pinmask;

  /* Decode the basics */

  port    = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  pinno   = (pinset & GPIO_PIN_MASK);
  pinmask = 1 << pinno;

  /* Get the base address associated with the GPIO port */

  base = tiva_gpiobaseaddress(port);

  /* allow access to the TIVA_GPIO_CR_OFFSET register */

  modifyreg32(base + TIVA_GPIO_LOCK_OFFSET,  0, GPIO_LOCK_UNLOCK);

  /* lock or unlock the pin */

  if (lock)
    {
      gpioinfo("  locking port=%d pin=%d\n", port, pinno);
      modifyreg32(base + TIVA_GPIO_CR_OFFSET, pinmask, 0);
    }
  else
    {
      gpioinfo("unlocking port=%d pin=%d\n", port, pinno);
      modifyreg32(base + TIVA_GPIO_CR_OFFSET, 0, pinmask);
    }

  /* Restrict access to the TIVA_GPIO_CR_OFFSET register */

  modifyreg32(base + TIVA_GPIO_LOCK_OFFSET,  GPIO_LOCK_UNLOCK, GPIO_LOCK_LOCKED);
}
