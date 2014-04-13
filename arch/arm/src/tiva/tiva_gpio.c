/****************************************************************************
 * arch/arm/src/tiva/tiva_gpio.c
 *
 *   Copyright (C) 2009-2010, 2014 Gregory Nutt. All rights reserved.
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

#include <arch/irq.h>

#include "up_arch.h"
#include "os_internal.h"
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

static uintptr_t tiva_gpiobaseaddress(unsigned int port)
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
 *   Configure GPIO registers for a specific function
 *
 ****************************************************************************/

static void tiva_gpiofunc(uint32_t base, uint32_t pinno,
                          const struct gpio_func_s *func)
{
  uint32_t setbit;
  uint32_t clrbit;
  uint32_t regval;

  /* Set/clear/ignore the GPIO ODR bit. "The GPIO ODR register is the open drain
   * control register. Setting a bit in this register enables the open drain
   * configuration of the corresponding GPIO pad. When open drain mode is enabled,
   * the corresponding bit should also be set in the GPIO Digital Input Enable
   * (GPIO DEN) register ... Corresponding bits in the drive strength registers
   * (GPIO DR2R, GPIO DR4R, GPIO DR8R, and GPIO SLR ) can be set to achieve the
   * desired rise and fall times. The GPIO acts as an open drain input if the
   * corresponding bit in the GPIO DIR register is set to 0; and as an open
   * drain output when set to 1."
   */

  setbit = (((uint32_t)func->setbits >> ODR_SHIFT) & 1) << pinno;
  clrbit = (((uint32_t)func->clrbits >> ODR_SHIFT) & 1) << pinno;

  regval = getreg32(base + TIVA_GPIO_ODR_OFFSET);
  regval &= ~clrbit;
  regval |= setbit;
  putreg32(regval, base + TIVA_GPIO_ODR_OFFSET);

  /* Set/clear the GPIO PUR bit. "The GPIOPUR register is the pull-up control
   * register. When a bit is set to 1, it enables a weak pull-up resistor on the
   * corresponding GPIO signal. Setting a bit in GPIOPUR automatically clears the
   * corresponding bit in the GPIO Pull-Down Select (GPIOPDR) register ..."
   */

  setbit = (((uint32_t)func->setbits >> PUR_SHIFT) & 1) << pinno;
  clrbit = (((uint32_t)func->clrbits >> PUR_SHIFT) & 1) << pinno;

  if (setbit || clrbit)
    {
      regval = getreg32(base + TIVA_GPIO_PUR_OFFSET);
      regval &= ~clrbit;
      regval |= setbit;
      putreg32(regval, base + TIVA_GPIO_PUR_OFFSET);
    }

  /* Set/clear the GPIO PDR bit. "The GPIOPDR register is the pull-down control
   * register. When a bit is set to 1, it enables a weak pull-down resistor on the
   * corresponding GPIO signal. Setting a bit in GPIOPDR automatically clears
   * the corresponding bit in the GPIO Pull-Up Select (GPIOPUR) register ..."
   */

  setbit = (((uint32_t)func->setbits >> PDR_SHIFT) & 1) << pinno;
  clrbit = (((uint32_t)func->clrbits >> PDR_SHIFT) & 1) << pinno;

  if (setbit || clrbit)
    {
      regval = getreg32(base + TIVA_GPIO_PDR_OFFSET);
      regval &= ~clrbit;
      regval |= setbit;
      putreg32(regval, base + TIVA_GPIO_PDR_OFFSET);
    }

  /* Set/clear the GPIO DEN bit. "The GPIODEN register is the digital enable
   * register. By default, with the exception of the GPIO signals used for JTAG/SWD
   * function, all other GPIO signals are configured out of reset to be undriven
   * (tristate). Their digital function is disabled; they do not drive a logic
   * value on the pin and they do not allow the pin voltage into the GPIO receiver.
   * To use the pin in a digital function (either GPIO or alternate function), the
   * corresponding GPIODEN bit must be set."
   */

  setbit = (((uint32_t)func->setbits >> DEN_SHIFT) & 1) << pinno;
  clrbit = (((uint32_t)func->clrbits >> DEN_SHIFT) & 1) << pinno;

  regval = getreg32(base + TIVA_GPIO_DEN_OFFSET);
  regval &= ~clrbit;
  regval |= setbit;
  putreg32(regval, base + TIVA_GPIO_DEN_OFFSET);

  /* Set/clear/ignore the GPIO DIR bit. "The GPIODIR register is the data
   * direction register. Bits set to 1 in the GPIODIR register configure
   * the corresponding pin to be an output, while bits set to 0 configure the
   * pins to be inputs. All bits are cleared by a reset, meaning all GPIO
   * pins are inputs by default.
   */

  setbit = (((uint32_t)func->setbits >> DIR_SHIFT) & 1) << pinno;
  clrbit = (((uint32_t)func->clrbits >> DIR_SHIFT) & 1) << pinno;

  regval = getreg32(base + TIVA_GPIO_DIR_OFFSET);
  regval &= ~clrbit;
  regval |= setbit;
  putreg32(regval, base + TIVA_GPIO_DIR_OFFSET);

  /* Set/clear/ignore the GPIO AFSEL bit. "The GPIOAFSEL register is the mode
   * control select register. Writing a 1 to any bit in this register selects
   * the hardware control for the corresponding GPIO line. All bits are cleared
   * by a reset, therefore no GPIO line is set to hardware control by default."
   *
   * NOTE: In order so set JTAG/SWD GPIOs, it is also necessary to lock, commit
   * and unlock the GPIO.  That is not implemented here.
   */

  setbit = (((uint32_t)func->setbits >> AFSEL_SHIFT) & 1) << pinno;
  clrbit = (((uint32_t)func->clrbits >> AFSEL_SHIFT) & 1) << pinno;

  regval = getreg32(base + TIVA_GPIO_AFSEL_OFFSET);
  regval &= ~clrbit;
  regval |= setbit;
  putreg32(regval, base + TIVA_GPIO_AFSEL_OFFSET);

  /* Set/clear/ignore the GPIO AMSEL bit. "The GPIOAMSEL register controls
   * isolation circuits to the analog side of a unified I/O pad. Because
   * the GPIOs may be driven by a 5-V source and affect analog operation,
   * analog circuitry requires isolation from the pins when they are not
   * used in their analog function.  Each bit of this register controls the
   * isolation circuitry for the corresponding GPIO signal.
   */

#if defined(LM4F) || defined(TM4C)
  setbit = (((uint32_t)func->setbits >> AMSEL_SHIFT) & 1) << pinno;
  clrbit = (((uint32_t)func->clrbits >> AMSEL_SHIFT) & 1) << pinno;

  regval = getreg32(base + TIVA_GPIO_AMSEL_OFFSET);
  regval &= ~clrbit;
  regval |= setbit;
  putreg32(regval, base + TIVA_GPIO_AMSEL_OFFSET);
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
                                        uint32_t cfgset)
{
  int strength = (cfgset & GPIO_STRENGTH_MASK) >> GPIO_STRENGTH_SHIFT;
  uint32_t regoffset;
  uint32_t regval;
  uint32_t slrset;
  uint32_t slrclr;

  /* Prepare bits to disable slew */

  slrset = 0;
  slrclr = pin;

  switch (strength)
    {
      case 0: /* 2mA pad drive strength */
        {
          /* "The GPIODR2R register is the 2-mA drive control register. It
           * allows for each GPIO signal in the port to be individually configured
           * without affecting the other pads. When writing a DRV2 bit for a GPIO
           * signal, the corresponding DRV4 bit in the GPIO DR4R register and the
           * DRV8 bit in the GPIODR8R register are automatically cleared by hardware."
           */

          regoffset = TIVA_GPIO_DR2R_OFFSET;
        }
        break;

      case 1: /* 4mA pad drive strength */
        {
          /* "The GPIODR4R register is the 4-mA drive control register. It allows
           * for each GPIO signal in the port to be individually configured without
           * affecting the other pads. When writing the DRV4 bit for a GPIO signal,
           * the corresponding DRV2 bit in the GPIO DR2R register and the DRV8 bit
           * in the GPIO DR8R register are automatically cleared by hardware."
           */

          regoffset = TIVA_GPIO_DR4R_OFFSET;
        }
        break;

      case 3: /* 8mA Pad drive with slew rate control */
        {
          /* "The GPIOSLR register is the slew rate control register. Slew rate
           * control is only available when using the 8-mA drive strength option
           * via the GPIO 8-mA Drive Select (GPIODR8R) register..."
           */

          slrset = pin;
          slrclr = 0;
        }
        /* Fall through */

      case 2: /* 8mA pad drive strength (without slew rate control) */
        {
          /* "The GPIODR8R register is the 8-mA drive control register. It
           * allows for each GPIO signal in the port to be individually configured
           * without affecting the other pads. When writing the DRV8 bit for a GPIO
           * signal, the corresponding DRV2 bit in the GPIO DR2R register and the
           * DRV4 bit in the GPIO DR4R register are automatically cleared by hardware."
           */

          regoffset = TIVA_GPIO_DR8R_OFFSET;
        }
        break;
    }

  /* Set the selected pad strength and set/clear optional slew rate control */

  regval = getreg32(base + regoffset);
  regval |= pin;
  putreg32(regval, base + regoffset);

  regval  = getreg32(base + TIVA_GPIO_SLR_OFFSET);
  regval &= slrclr;
  regval |= slrset;
  putreg32(regval, base + TIVA_GPIO_SLR_OFFSET);
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
                                    uint32_t cfgset)
{
  int padtype  = (cfgset & GPIO_PADTYPE_MASK) >> GPIO_PADTYPE_SHIFT;
#if 0 /* always overwritten by tiva_gpiofunc */
  uint32_t odrset;
  uint32_t odrclr;
#endif
  uint32_t purset;
  uint32_t purclr;
  uint32_t pdrset;
  uint32_t pdrclr;
#if 0 /* always overwritten by tiva_gpiofunc */
  uint32_t denset;
  uint32_t denclr;
#endif
  uint32_t regval;

  /* Assume digital GPIO function, push-pull with no pull-up or pull-down */

#if 0 /* always overwritten by tiva_gpiofunc */
  odrset = 0;
  odrclr = pin;
#endif
  purset = 0;
  purclr = pin;
  pdrset = 0;
  pdrclr = pin;
#if 0 /* always overwritten by tiva_gpiofunc */
  denset = pin;
  denclr = 0;
#endif

  switch (padtype)
    {
      case 0: /* Push-pull */
      default:
        {
        }
        break;

      case 1: /* Push-pull with weak pull-up */
        {
          purset = pin;
          purclr = 0;
        }
        break;
      case 2: /* Push-pull with weak pull-down */
        {
          pdrset = pin;
          pdrclr = 0;
        }
        break;
      case 3: /* Open-drain */
        {
#if 0 /* always overwritten by tiva_gpiofunc */
          odrset = pin;
          odrclr = 0;
#endif
        }
        break;
      case 4: /* Open-drain with weak pull-up */
        {
#if 0 /* always overwritten by tiva_gpiofunc */
          odrset = pin;
          odrclr = 0;
#endif
          purset = pin;
          purclr = 0;
        }
        break;
      case 5: /* Open-drain with weak pull-down */
        {
#if 0 /* always overwritten by tiva_gpiofunc */
          odrset = pin;
          odrclr = 0;
#endif
          pdrset = pin;
          pdrclr = 0;
        }
        break;
      case 6: /* Analog comparator */
        {
#if 0 /* always overwritten by tiva_gpiofunc */
          denset = 0;
          denclr = pin;
#endif
        }
        break;
    }

  /* Set/clear the GPIO ODR bit. "The GPIO ODR register is the open drain
   * control register. Setting a bit in this register enables the open drain
   * configuration of the corresponding GPIO pad. When open drain mode is enabled,
   * the corresponding bit should also be set in the GPIO Digital Input Enable
   * (GPIO DEN) register ... Corresponding bits in the drive strength registers
   * (GPIO DR2R, GPIO DR4R, GPIO DR8R, and GPIO SLR ) can be set to achieve the
   * desired rise and fall times. The GPIO acts as an open drain input if the
   * corresponding bit in the GPIO DIR register is set to 0; and as an open
   * drain output when set to 1."
   */

#if 0 /* always overwritten by tiva_gpiofunc */
  regval = getreg32(base + TIVA_GPIO_ODR_OFFSET);
  regval &= ~odrclr;
  regval |= odrset;
  putreg32(regval, base + TIVA_GPIO_ODR_OFFSET);
#endif

  /* Set/clear the GPIO PUR bit. "The GPIOPUR register is the pull-up control
   * register. When a bit is set to 1, it enables a weak pull-up resistor on the
   * corresponding GPIO signal. Setting a bit in GPIOPUR automatically clears the
   * corresponding bit in the GPIO Pull-Down Select (GPIOPDR) register ..."
   */

  regval = getreg32(base + TIVA_GPIO_PUR_OFFSET);
  regval &= ~purclr;
  regval |= purset;
  putreg32(regval, base + TIVA_GPIO_PUR_OFFSET);

  /* Set/clear the GPIO PDR bit. "The GPIOPDR register is the pull-down control
   * register. When a bit is set to 1, it enables a weak pull-down resistor on the
   * corresponding GPIO signal. Setting a bit in GPIOPDR automatically clears
   * the corresponding bit in the GPIO Pull-Up Select (GPIOPUR) register ..."
   */

  regval = getreg32(base + TIVA_GPIO_PDR_OFFSET);
  regval &= ~pdrclr;
  regval |= pdrset;
  putreg32(regval, base + TIVA_GPIO_PDR_OFFSET);

  /* Set/clear the GPIO DEN bit. "The GPIODEN register is the digital enable
   * register. By default, with the exception of the GPIO signals used for JTAG/SWD
   * function, all other GPIO signals are configured out of reset to be undriven
   * (tristate). Their digital function is disabled; they do not drive a logic
   * value on the pin and they do not allow the pin voltage into the GPIO receiver.
   * To use the pin in a digital function (either GPIO or alternate function), the
   * corresponding GPIODEN bit must be set."
   */

#if 0 /* always overwritten by tiva_gpiofunc */
  regval = getreg32(base + TIVA_GPIO_DEN_OFFSET);
  regval &= ~denclr;
  regval |= denset;
  putreg32(regval, base + TIVA_GPIO_DEN_OFFSET);
#endif
}

/****************************************************************************
 * Name: tiva_initoutput
 *
 * Description:
 *   Set the GPIO output value
 *
 ****************************************************************************/

static inline void tiva_initoutput(uint32_t cfgset)
{
  bool value = ((cfgset & GPIO_VALUE_MASK) != GPIO_VALUE_ZERO);
  tiva_gpiowrite(cfgset, value);
}

/****************************************************************************
 * Name: tiva_interrupt
 *
 * Description:
 *   Configure the interrupt pin.
 *
 ****************************************************************************/

static inline void tiva_interrupt(uint32_t base, uint32_t pin, uint32_t cfgset)
{
  int inttype = (cfgset & GPIO_INT_MASK) >> GPIO_INT_SHIFT;
  uint32_t regval;
  uint32_t isset;
  uint32_t isclr;
  uint32_t ibeset;
  uint32_t ibeclr;
  uint32_t iveset;
  uint32_t iveclr;

  /* Mask and clear the GPIO interrupt
   *
   * "The GPIOIM register is the interrupt mask register. Bits set to High in
   * GPIO IM allow the corresponding pins to trigger their individual interrupts
   * and the combined GPIO INTR line. Clearing a bit disables interrupt triggering
   * on that pin. All bits are cleared by a reset."
   */

  regval  = getreg32(base + TIVA_GPIO_IM_OFFSET);
  regval &= ~pin;
  putreg32(regval, base + TIVA_GPIO_IM_OFFSET);

  /* "The GPIOICR register is the interrupt clear register. Writing a 1 to a bit
   * in this register clears the corresponding interrupt edge detection logic
   * register. Writing a 0 has no effect."
   */

  regval  = getreg32(base + TIVA_GPIO_ICR_OFFSET);
  regval |= pin;
  putreg32(regval, base + TIVA_GPIO_ICR_OFFSET);

  /* Assume rising edge */

  isset  = 0;            /* Not level sensed */
  isclr  = pin;
  ibeset = 0;            /* Single edge */
  ibeclr = pin;
  iveset = pin;          /* Rising edge or high levels*/
  iveclr = 0;

  /* Then handle according to the selected interrupt type */

  switch (inttype)
    {
      case 0:            /* Interrupt on falling edge */
        {
          iveset = 0;    /* Falling edge or low levels*/
          iveclr = pin;
        }
        break;

      case 1:           /* Interrupt on rising edge */
      default:
        break;

      case 2:           /* Interrupt on both edges */
        {
          ibeset = pin; /* Both edges */
          ibeclr = 0;
        }
        break;

      case 3:           /* Interrupt on low level */
        {
          isset = pin;  /* Level sensed */
          isclr = 0;
          iveset = 0;   /* Falling edge or low levels*/
          iveclr = pin;
        }
        break;

      case 4:           /* Interrupt on high level */
        {
          isset = pin;  /* Level sensed */
          isclr = 0;
        }
        break;
    }

  /* "The GPIO IS register is the interrupt sense register. Bits set to
   * 1 in GPIOIS configure the corresponding pins to detect levels, while
   * bits set to 0 configure the pins to detect edges. All bits are cleared
   * by a reset.
   */

  regval  = getreg32(base + TIVA_GPIO_IS_OFFSET);
  regval &= isclr;
  regval |= isset;
  putreg32(regval, base + TIVA_GPIO_IS_OFFSET);

  /* "The GPIO IBE register is the interrupt both-edges register. When the
   * corresponding bit in the GPIO Interrupt Sense (GPIO IS) register ... is
   * set to detect edges, bits set to High in GPIO IBE configure the
   * corresponding pin to detect both rising and falling edges, regardless
   * of the corresponding bit in the GPIO Interrupt Event (GPIO IEV) register ...
   * Clearing a bit configures the pin to be controlled by GPIOIEV. All bits
   * are cleared by a reset.
   */

  regval  = getreg32(base + TIVA_GPIO_IBE_OFFSET);
  regval &= ibeclr;
  regval |= ibeset;
  putreg32(regval, base + TIVA_GPIO_IBE_OFFSET);

  /* "The GPIOIEV register is the interrupt event register. Bits set to
   * High in GPIO IEV configure the corresponding pin to detect rising edges
   * or high levels, depending on the corresponding bit value in the GPIO
   * Interrupt Sense (GPIO IS) register... Clearing a bit configures the pin to
   * detect falling edges or low levels, depending on the corresponding bit
   * value in GPIOIS. All bits are cleared by a reset.
   */

  regval  = getreg32(base + TIVA_GPIO_IEV_OFFSET);
  regval &= iveclr;
  regval |= iveset;
  putreg32(regval, base + TIVA_GPIO_IEV_OFFSET);
}

/****************************************************************************
 * Name: tiva_portcontrol
 *
 * Description:
 *   Set the pin alternate function in the port control register.
 *
 ****************************************************************************/

#if defined(LM4F) || defined(TM4C)
static inline void tiva_portcontrol(uint32_t base, uint32_t pinno,
                                    uint32_t cfgset,
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

      alt = (cfgset & GPIO_ALT_MASK) >> GPIO_ALT_SHIFT;
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

int tiva_configgpio(uint32_t cfgset)
{
  irqstate_t   flags;
  unsigned int func;
  unsigned int port;
  unsigned int pinno;
  uintptr_t    base;
  uint32_t     pin;
  uint32_t     regval;

  /* Decode the basics */

  func  = (cfgset & GPIO_FUNC_MASK) >> GPIO_FUNC_SHIFT;
  port  = (cfgset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  pinno = (cfgset & GPIO_PIN_MASK);
  pin   = (1 << pinno);

  DEBUGASSERT(func <= GPIO_FUNC_MAX);

  /* Get the base address associated with the GPIO port */

  base = tiva_gpiobaseaddress(port);
  DEBUGASSERT(base != 0);

  /* The following requires exclusive access to the GPIO registers */

  flags = irqsave();

  /* Enable clocking for this GPIO peripheral. "To use the GPIO, the peripheral
   * clock must be enabled by setting the appropriate GPIO Port bit field (GPIOn)
   * in the RCGC2 register."
   */

  regval = getreg32(TIVA_SYSCON_RCGC2);
  regval |= SYSCON_RCGC2_GPIO(port);
  putreg32(regval, TIVA_SYSCON_RCGC2);

  /* First, set the port to digital input.  This is the safest state in which
   * to perform reconfiguration.
   */

  tiva_gpiofunc(base, pinno, &g_funcbits[0]);
  tiva_portcontrol(base, pinno, cfgset, &g_funcbits[0]);

  /* Then set up pad strengths and pull-ups.  These setups should be done before
   * setting up the function because some function settings will over-ride these
   * user options.
   */

  tiva_gpiopadstrength(base, pin, cfgset);
  tiva_gpiopadtype(base, pin, cfgset);

  /* Then set up the real pin function */

  tiva_gpiofunc(base, pinno, &g_funcbits[func]);
  tiva_portcontrol(base, pinno, cfgset, &g_funcbits[func]);

  /* Special case GPIO digital output pins */

  if (func == 1 || func == 3)
    {
      tiva_initoutput(cfgset);
    }

  /* Special setup for interrupt GPIO pins */

  else if (func == 7)
    {
      tiva_interrupt(base, pin, cfgset);
    }

  irqrestore(flags);
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

bool tiva_gpioread(uint32_t pinset, bool value)
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
