/****************************************************************************
 * arch/arm/src/samv7/sam_gpio.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <time.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "sam_gpio.h"
#include "hardware/sam_pio.h"
#include "hardware/sam_matrix.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(CONFIG_SAMV7_ERASE_ENABLE) || \
    !defined(CONFIG_SAMV7_JTAG_FULL_ENABLE)
#  if defined(CONFIG_SAMV7_ERASE_DISABLE)
#    define SYSIO_ERASE_BIT MATRIX_CCFG_SYSIO_SYSIO12
#  else
#    define SYSIO_ERASE_BIT 0
#  endif
#  if defined(CONFIG_SAMV7_JTAG_DISABLE)
#    define SYSIO_BITS (MATRIX_CCFG_SYSIO_SYSIO4 | MATRIX_CCFG_SYSIO_SYSIO5 | \
               MATRIX_CCFG_SYSIO_SYSIO6 | MATRIX_CCFG_SYSIO_SYSIO7)
#  endif
#  if defined(CONFIG_SAMV7_JTAG_FULL_SW_ENABLE)
#    define SYSIO_BITS MATRIX_CCFG_SYSIO_SYSIO4
#  endif
#  if defined(CONFIG_SAMV7_JTAG_SW_ENABLE)
#    define SYSIO_BITS (MATRIX_CCFG_SYSIO_SYSIO4 | MATRIX_CCFG_SYSIO_SYSIO5)
#  endif
#endif
#if !defined(SYSIO_BITS)
#   define SYSIO_BITS 0
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_DEBUG_GPIO_INFO
static const char g_portchar[SAMV7_NPIO] =
{
    'A'
#if SAMV7_NPIO > 1
  , 'B'
#endif
#if SAMV7_NPIO > 2
  , 'C'
#endif
#if SAMV7_NPIO > 3
  , 'D'
#endif
#if SAMV7_NPIO > 4
  , 'E'
#endif
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

const uintptr_t g_portbase[SAMV7_NPIO] =
{
  SAM_PIOA_BASE
#if SAMV7_NPIO > 1
  , SAM_PIOB_BASE
#endif
#if SAMV7_NPIO > 2
  , SAM_PIOC_BASE
#endif
#if SAMV7_NPIO > 3
  , SAM_PIOD_BASE
#endif
#if SAMV7_NPIO > 4
  , SAM_PIOE_BASE
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: sam_configinput
 *
 * Description:
 *   Configure a GPIO input pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

static inline int sam_configinput(uintptr_t base, uint32_t pin,
                                  gpio_pinset_t cfgset)
{
#ifdef GPIO_HAVE_SCHMITT
  uint32_t regval;
#endif

  /* Disable interrupts on the pin */

  putreg32(pin, base + SAM_PIO_IDR_OFFSET);

  /* Enable/disable the pull-up as requested */

  if ((cfgset & GPIO_CFG_PULLUP) != 0)
    {
#ifdef GPIO_HAVE_PULLDOWN
      /* The pull-up on a pin can not be enabled if its pull-down is still
       * active. Therefore, we need to disable the pull-down first before
       * enabling the pull-up.
       */

      putreg32(pin, base + SAM_PIO_PPDDR_OFFSET);
#endif
      putreg32(pin, base + SAM_PIO_PUER_OFFSET);
    }
  else
    {
      putreg32(pin, base + SAM_PIO_PUDR_OFFSET);
    }

#ifdef GPIO_HAVE_PULLDOWN
  /* Enable/disable the pull-down as requested */

  if ((cfgset & GPIO_CFG_PULLDOWN) != 0)
    {
      /* The pull-down on a pin can not be enabled if its pull-up is still
       * active. Therefore, we need to disable the pull-up first before
       * enabling the pull-down.
       */

      putreg32(pin, base + SAM_PIO_PUDR_OFFSET);
      putreg32(pin, base + SAM_PIO_PPDER_OFFSET);
    }
  else
    {
      putreg32(pin, base + SAM_PIO_PPDDR_OFFSET);
    }
#endif

  /* Check if filtering should be enabled */

  if ((cfgset & GPIO_CFG_DEGLITCH) != 0)
    {
      putreg32(pin, base + SAM_PIO_IFER_OFFSET);
    }
  else
    {
      putreg32(pin, base + SAM_PIO_IFDR_OFFSET);
    }

#ifdef GPIO_HAVE_SCHMITT
  /* Enable/disable the Schmitt trigger */

  regval = getreg32(base + SAM_PIO_SCHMITT_OFFSET);
  if ((cfgset & GPIO_CFG_PULLDOWN) != 0)
    {
      regval |= pin;
    }
  else
    {
      regval &= ~pin;
    }

  putreg32(regval, base + SAM_PIO_SCHMITT_OFFSET);
#endif

#ifdef GPIO_HAVE_DRIVER
  /* Reset output drive strength (PIO outputs only) */

  regval  = getreg32(base + SAM_PIO_DRIVER_OFFSET);
  regval &= ~pin;
  putreg32(regval, base + SAM_PIO_DRIVER_OFFSET);
#endif

  /* Configure the pin as an input and enable the GPIO function */

  putreg32(pin, base + SAM_PIO_ODR_OFFSET);
  putreg32(pin, base + SAM_PIO_PER_OFFSET);

  /* To-Do:  If DEGLITCH is selected, need to configure DIFSR, SCIFSR, and
   *         IFDGSR registers.  This would probably best be done with
   *         another, new API... perhaps sam_configfilter()
   */

  return OK;
}

/****************************************************************************
 * Name: sam_configoutput
 *
 * Description:
 *   Configure a GPIO output pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

static inline int sam_configoutput(uintptr_t base, uint32_t pin,
                                   gpio_pinset_t cfgset)
{
#ifdef GPIO_HAVE_DRIVER
  uint32_t regval;
#endif

  /* Disable interrupts on the pin */

  putreg32(pin, base + SAM_PIO_IDR_OFFSET);

  /* Enable/disable the pull-up as requested */

  if ((cfgset & GPIO_CFG_PULLUP) != 0)
    {
#ifdef GPIO_HAVE_PULLDOWN
      /* The pull-up on a pin can not be enabled if its pull-down is still
       * active. Therefore, we need to disable the pull-down first before
       * enabling the pull-up.
       */

      putreg32(pin, base + SAM_PIO_PPDDR_OFFSET);
#endif
      putreg32(pin, base + SAM_PIO_PUER_OFFSET);
    }
  else
    {
      putreg32(pin, base + SAM_PIO_PUDR_OFFSET);
    }

#ifdef GPIO_HAVE_PULLDOWN
  /* Enable/disable the pull-down as requested */

  if ((cfgset & GPIO_CFG_PULLDOWN) != 0)
    {
      /* The pull-down on a pin can not be enabled if its pull-up is still
       * active. Therefore, we need to disable the pull-up first before
       * enabling the pull-down.
       */

      putreg32(pin, base + SAM_PIO_PUDR_OFFSET);
      putreg32(pin, base + SAM_PIO_PPDER_OFFSET);
    }
  else
    {
      putreg32(pin, base + SAM_PIO_PPDDR_OFFSET);
    }
#endif

  /* Enable the open drain driver if requested */

  if ((cfgset & GPIO_CFG_OPENDRAIN) != 0)
    {
      putreg32(pin, base + SAM_PIO_MDER_OFFSET);
    }
  else
    {
      putreg32(pin, base + SAM_PIO_MDDR_OFFSET);
    }

  /* Set default value */

  if ((cfgset & GPIO_OUTPUT_SET) != 0)
    {
      putreg32(pin, base + SAM_PIO_SODR_OFFSET);
    }
  else
    {
      putreg32(pin, base + SAM_PIO_CODR_OFFSET);
    }

#ifdef GPIO_HAVE_DRIVER
  /* Select the pin output drive strength */

  regval = getreg32(base + SAM_PIO_DRIVER_OFFSET);
  if ((cfgset & GPIO_OUTPUT_DRIVE) != GPIO_OUTPUT_LOW_DRIVE)
    {
      regval |= pin;
    }
  else
    {
      regval &= ~pin;
    }

  putreg32(regval, base + SAM_PIO_DRIVER_OFFSET);
#endif

  /* Configure the pin as an output and enable the GPIO function */

  putreg32(pin, base + SAM_PIO_OER_OFFSET);
  putreg32(pin, base + SAM_PIO_PER_OFFSET);
  return OK;
}

/****************************************************************************
 * Name: sam_configperiph
 *
 * Description:
 *   Configure a GPIO pin driven by a peripheral A or B signal based on
 *   bit-encoded description of the pin.
 *
 ****************************************************************************/

static inline int sam_configperiph(uintptr_t base, uint32_t pin,
                                   gpio_pinset_t cfgset)
{
  uint32_t regval;

  /* Disable interrupts on the pin */

  putreg32(pin, base + SAM_PIO_IDR_OFFSET);

  /* Enable/disable the pull-up as requested */

  if ((cfgset & GPIO_CFG_PULLUP) != 0)
    {
#ifdef GPIO_HAVE_PULLDOWN
      /* The pull-up on a pin can not be enabled if its pull-down is still
       * active. Therefore, we need to disable the pull-down first before
       * enabling the pull-up.
       */

      putreg32(pin, base + SAM_PIO_PPDDR_OFFSET);
#endif
      putreg32(pin, base + SAM_PIO_PUER_OFFSET);
    }
  else
    {
      putreg32(pin, base + SAM_PIO_PUDR_OFFSET);
    }

#ifdef GPIO_HAVE_PULLDOWN
  /* Enable/disable the pull-down as requested */

  if ((cfgset & GPIO_CFG_PULLDOWN) != 0)
    {
      /* The pull-down on a pin can not be enabled if its pull-up is still
       * active. Therefore, we need to disable the pull-up first before
       * enabling the pull-down.
       */

      putreg32(pin, base + SAM_PIO_PUDR_OFFSET);
      putreg32(pin, base + SAM_PIO_PPDER_OFFSET);
    }
  else
    {
      putreg32(pin, base + SAM_PIO_PPDDR_OFFSET);
    }
#endif

#ifdef GPIO_HAVE_DRIVER
  /* Reset output drive strength (PIO outputs only) */

  regval  = getreg32(base + SAM_PIO_DRIVER_OFFSET);
  regval &= ~pin;
  putreg32(regval, base + SAM_PIO_DRIVER_OFFSET);
#endif

#ifdef GPIO_HAVE_PERIPHCD
  /* Configure pin, depending upon the peripheral A, B, C or D
   *
   *   PERIPHA: ABCDSR1[n] = 0 ABCDSR2[n] = 0
   *   PERIPHB: ABCDSR1[n] = 1 ABCDSR2[n] = 0
   *   PERIPHC: ABCDSR1[n] = 0 ABCDSR2[n] = 1
   *   PERIPHD: ABCDSR1[n] = 1 ABCDSR2[n] = 1
   */

  regval = getreg32(base + SAM_PIO_ABCDSR1_OFFSET);
  if ((cfgset & GPIO_MODE_MASK) == GPIO_PERIPHA ||
      (cfgset & GPIO_MODE_MASK) == GPIO_PERIPHC)
    {
      regval &= ~pin;
    }
  else
    {
      regval |= pin;
    }
  putreg32(regval, base + SAM_PIO_ABCDSR1_OFFSET);

  regval = getreg32(base + SAM_PIO_ABCDSR2_OFFSET);
  if ((cfgset & GPIO_MODE_MASK) == GPIO_PERIPHA ||
      (cfgset & GPIO_MODE_MASK) == GPIO_PERIPHB)
    {
      regval &= ~pin;
    }
  else
    {
      regval |= pin;
    }
  putreg32(regval, base + SAM_PIO_ABCDSR2_OFFSET);

#else
  /* Configure pin, depending upon the peripheral A or B:
   *
   *   PERIPHA: ABSR[n] = 0
   *   PERIPHB: ABSR[n] = 1
   */

  regval = getreg32(base + SAM_PIO_ABSR_OFFSET);
  if ((cfgset & GPIO_MODE_MASK) == GPIO_PERIPHA)
    {
      regval &= ~pin;
    }
  else
    {
      regval |= pin;
    }

  putreg32(regval, base + SAM_PIO_ABSR_OFFSET);
#endif

  /* Disable PIO functionality */

  putreg32(pin, base + SAM_PIO_PDR_OFFSET);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  sam_gpioinit
 *
 * Description:
 *   Based on configuration within the .config file, it does:
 *    - Configures the CCFG_SYSIO bits.
 *
 *   Typically called from sam_start().
 *
 * Assumptions:
 *   This function is called early in the initialization sequence so that
 *   no mutual exclusion is necessary.
 *
 ****************************************************************************/

#if !defined(CONFIG_SAMV7_ERASE_ENABLE) || \
    !defined(CONFIG_SAMV7_JTAG_FULL_ENABLE)
void sam_gpioinit(void)
{
  uint32_t regval;

  regval  = getreg32(SAM_MATRIX_CCFG_SYSIO);
  regval |= (SYSIO_ERASE_BIT | SYSIO_BITS);
  putreg32(regval, SAM_MATRIX_CCFG_SYSIO);
}
#endif

/****************************************************************************
 * Name: sam_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int sam_configgpio(gpio_pinset_t cfgset)
{
  uintptr_t base = sam_gpio_base(cfgset);
  uint32_t pin   = sam_gpio_pinmask(cfgset);
  irqstate_t flags;
  int ret;

  /* Disable interrupts to prohibit re-entrance. */

  flags = enter_critical_section();

  /* Enable writing to GPIO registers */

  putreg32(PIO_WPMR_WPKEY, base + SAM_PIO_WPMR_OFFSET);

  /* Handle the pin configuration according to pin type */

  switch (cfgset & GPIO_MODE_MASK)
    {
      case GPIO_ALTERNATE:
      case GPIO_INPUT:
        ret = sam_configinput(base, pin, cfgset);
        break;

      case GPIO_OUTPUT:
        ret = sam_configoutput(base, pin, cfgset);
        break;

      case GPIO_PERIPHA:
      case GPIO_PERIPHB:
#ifdef GPIO_HAVE_PERIPHCD
      case GPIO_PERIPHC:
      case GPIO_PERIPHD:
#endif
        ret = sam_configperiph(base, pin, cfgset);
        break;

      default:
        ret = -EINVAL;
        break;
    }

  /* Disable writing to GPIO registers */

  putreg32(PIO_WPMR_WPEN | PIO_WPMR_WPKEY, base + SAM_PIO_WPMR_OFFSET);
  leave_critical_section(flags);

  return ret;
}

/****************************************************************************
 * Name: sam_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void sam_gpiowrite(gpio_pinset_t pinset, bool value)
{
  uintptr_t base = sam_gpio_base(pinset);
  uint32_t  pin  = sam_gpio_pinmask(pinset);

  if (value)
    {
      putreg32(pin, base + SAM_PIO_SODR_OFFSET);
    }
  else
    {
      putreg32(pin, base + SAM_PIO_CODR_OFFSET);
    }
}

/****************************************************************************
 * Name: sam_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool sam_gpioread(gpio_pinset_t pinset)
{
  uintptr_t base = sam_gpio_base(pinset);
  uint32_t  pin  = sam_gpio_pinmask(pinset);
  uint32_t  regval;

  /* Always read the Pin Data Status Register.  Otherwise an Open-Drain
   * Output pin will not be read back correctly.
   */

  regval = getreg32(base + SAM_PIO_PDSR_OFFSET);
  return (regval & pin) != 0;
}

/****************************************************************************
 * Function:  sam_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the base address of the provided
 *   pinset.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_GPIO_INFO
int sam_dumpgpio(uint32_t pinset, const char *msg)
{
  irqstate_t    flags;
  uintptr_t     base;
  unsigned int  port;

  /* Get the base address associated with the PIO port */

  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  base = SAM_PION_BASE(port);

  /* The following requires exclusive access to the GPIO registers */

  flags = enter_critical_section();

  gpioinfo("PIO%c pinset: %08x base: %08x -- %s\n",
           g_portchar[port], pinset, base, msg);
  gpioinfo("    PSR: %08x    OSR: %08x   IFSR: %08x   ODSR: %08x\n",
           getreg32(base + SAM_PIO_PSR_OFFSET),
           getreg32(base + SAM_PIO_OSR_OFFSET),
           getreg32(base + SAM_PIO_IFSR_OFFSET),
           getreg32(base + SAM_PIO_ODSR_OFFSET));
  gpioinfo("   PDSR: %08x    IMR: %08x    ISR: %08x   MDSR: %08x\n",
           getreg32(base + SAM_PIO_PDSR_OFFSET),
           getreg32(base + SAM_PIO_IMR_OFFSET),
           getreg32(base + SAM_PIO_ISR_OFFSET),
           getreg32(base + SAM_PIO_MDSR_OFFSET));
  gpioinfo(" ABCDSR: %08x %08x         IFSCSR: %08x  PPDSR: %08x\n",
           getreg32(base + SAM_PIO_ABCDSR1_OFFSET),
           getreg32(base + SAM_PIO_ABCDSR2_OFFSET),
           getreg32(base + SAM_PIO_IFSCSR_OFFSET),
           getreg32(base + SAM_PIO_PPDSR_OFFSET));
  gpioinfo("   PUSR: %08x   SCDR: %08x   OWSR: %08x  AIMMR: %08x\n",
           getreg32(base + SAM_PIO_PUSR_OFFSET),
           getreg32(base + SAM_PIO_SCDR_OFFSET),
           getreg32(base + SAM_PIO_OWSR_OFFSET),
           getreg32(base + SAM_PIO_AIMMR_OFFSET));
  gpioinfo("    ESR: %08x    LSR: %08x   ELSR: %08x FELLSR: %08x\n",
           getreg32(base + SAM_PIO_ESR_OFFSET),
           getreg32(base + SAM_PIO_LSR_OFFSET),
           getreg32(base + SAM_PIO_ELSR_OFFSET),
           getreg32(base + SAM_PIO_FELLSR_OFFSET));
  gpioinfo(" FRLHSR: %08x LOCKSR: %08x   WPMR: %08x   WPSR: %08x\n",
           getreg32(base + SAM_PIO_FRLHSR_OFFSET),
           getreg32(base + SAM_PIO_LOCKSR_OFFSET),
           getreg32(base + SAM_PIO_WPMR_OFFSET),
           getreg32(base + SAM_PIO_WPSR_OFFSET));
  gpioinfo("   PCMR: %08x  PCIMR: %08x  PCISR: %08x   PCRHR: %08x\n",
           getreg32(base + SAM_PIO_PCMR_OFFSET),
           getreg32(base + SAM_PIO_PCIMR_OFFSET),
           getreg32(base + SAM_PIO_PCISR_OFFSET),
           getreg32(base + SAM_PIO_PCRHR_OFFSET));
  gpioinfo("SCHMITT: %08x DRIVER:%08x\n",
           getreg32(base + SAM_PIO_SCHMITT_OFFSET),
           getreg32(base + SAM_PIO_DRIVER_OFFSET));
  gpioinfo("    KER: %08x   KRCR: %08x    KDR: %08x   KIMR: %08x\n",
           getreg32(base + SAM_PIO_KER_OFFSET),
           getreg32(base + SAM_PIO_KRCR_OFFSET),
           getreg32(base + SAM_PIO_KDR_OFFSET),
           getreg32(base + SAM_PIO_KIMR_OFFSET));
  gpioinfo("    KSR: %08x   KKPR: %08x    KKRR: %08x\n",
           getreg32(base + SAM_PIO_KSR_OFFSET),
           getreg32(base + SAM_PIO_KKPR_OFFSET),
           getreg32(base + SAM_PIO_KKRR_OFFSET));
  gpioinfo("   PCMR: %08x  PCIMR: %08x  PCISR: %08x  PCRHR: %08x\n",
           getreg32(base + SAM_PIO_PCMR_OFFSET),
           getreg32(base + SAM_PIO_PCIMR_OFFSET),
           getreg32(base + SAM_PIO_PCISR_OFFSET),
           getreg32(base + SAM_PIO_PCRHR_OFFSET));

  leave_critical_section(flags);
  return OK;
}
#endif
