/****************************************************************************
 * arch/avr/src/sam34/sam4l_periphclks.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This file is derived from nuttx/arch/avr/src/at32uc3/at32uc3_clkinit.c
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

#include <arch/irq.h>
#include <arch/board/board.h>

#include "up_arch.h"

#include "up_internal.h"
#include "chip/sam4l_pm.h"

#include "sam4l_periphclks.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* USBC source clock selection */

#ifdef CONFIG_SAM34_USBC
#  if defined(BOARD_USBC_SRC_OSC0)
#    define SAM_USBC_GCLK_SOURCE SCIF_GCCTRL_OSCSEL_OSC0
#  elif defined(BOARD_USBC_SRC_PLL0)
#    define SAM_USBC_GCLK_SOURCE SCIF_GCCTRL_OSCSEL_PLL0
#  elif defined(BOARD_USBC_SRC_DFLL)
#    define SAM_USBC_GCLK_SOURCE SCIF_GCCTRL_OSCSEL_DFLL0
#  elif defined(BOARD_USBC_SRC_GCLKIN0)
#    define SAM_USBC_GCLK_SOURCE SCIF_GCCTRL_OSCSEL_GCLKIN0
#  else
#    error No USBC GCLK7 source clock defined
#  endif
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_init_cpumask
 *
 * Description:
 *   Called during boot to enable clocking on selected peripherals in the
 *   CPU mask register.
 *
 ****************************************************************************/

static inline void sam_init_cpumask(void)
{
  uint32_t mask = 0;

  /* OR in the user selected peripherals */

#ifdef CONFIG_SAM32_RESET_PERIPHCLKS
#ifdef CONFIG_SAM34_OCD
  mask |= PM_CPUMASK_OCD;             /* On-Chip Debug */
#endif
#endif

  /* Save the new CPU mask */

  putreg32(PM_UNLOCK_KEY(0xaa) | PM_UNLOCK_ADDR(SAM_PM_CPUMASK_OFFSET),
           SAM_PM_UNLOCK);
  putreg32(mask, SAM_PM_CPUMASK);
}

/****************************************************************************
 * Name: sam_init_hsbmask
 *
 * Description:
 *   Called during boot to enable clocking on selected peripherals in the
 *   HSB mask register.
 *
 ****************************************************************************/

static inline void sam_init_hsbmask(void)
{
  /* Select the non-optional peripherals */

  uint32_t mask = (PM_HSBMASK_FLASHCALW | PM_HSBMASK_APBB |
                   PM_HSBMASK_APBC | PM_HSBMASK_APBD);

  /* OR in the user selected peripherals */

#ifdef CONFIG_SAM32_RESET_PERIPHCLKS
#ifdef CONFIG_SAM34_PDCA
  mask |= PM_HSBMASK_PDCA;            /* PDCA */
#endif
#ifdef CONFIG_SAM34_HRAMC1
  mask |= PM_HSBMASK_HRAMC1;          /* HRAMC1 (picoCache RAM) */
#endif
#ifdef CONFIG_SAM34_USBC
  mask |= PM_HSBMASK_USBC;            /* USBC */
#endif
#ifdef CONFIG_SAM34_CRCCU
  mask |= PM_HSBMASK_CRCCU;           /* CRCCU */
#endif
#ifdef CONFIG_SAM34_APBA
  mask |= PM_HSBMASK_APBA;            /* APBA bridge */
#endif
#ifdef CONFIG_SAM34_AESA
  mask |= PM_HSBMASK_AESA;            /* AESA */
#endif
#endif

  /* Save the new HSB mask */

  putreg32(PM_UNLOCK_KEY(0xaa) | PM_UNLOCK_ADDR(SAM_PM_HSBMASK_OFFSET),
           SAM_PM_UNLOCK);
  putreg32(mask, SAM_PM_HSBMASK);
}

/****************************************************************************
 * Name: sam_init_pbamask
 *
 * Description:
 *   Called during boot to enable clocking on selected peripherals in the
 *   PBA mask register.
 *
 ****************************************************************************/

static inline void sam_init_pbamask(void)
{
  /* Select the non-optional peripherals */

  uint32_t mask = 0;
  uint32_t divmask = 0;

  /* OR in the user selected peripherals */

#ifdef CONFIG_SAM32_RESET_PERIPHCLKS
#ifdef CONFIG_SAM34_IISC
  mask    |= PM_PBAMASK_IISC;         /* IISC */
#endif
#ifdef CONFIG_SAM34_SPI
  mask    |= PM_PBAMASK_SPI;          /* SPI */
#endif
#ifdef CONFIG_SAM34_TC0
  mask    |= PM_PBAMASK_TC0;          /* TC0 */
  divmask |= PM_PBADIVMASK_TIMER_CLOCKS;
#endif
#ifdef CONFIG_SAM34_TC1
  mask    |= PM_PBAMASK_TC1;          /* TC1 */
  divmask |= PM_PBADIVMASK_TIMER_CLOCKS;
#endif
#ifdef CONFIG_SAM34_TWIM0
  mask    |= PM_PBAMASK_TWIM0;        /* TWIM0 */
#endif
#ifdef CONFIG_SAM34_TWIS0
  mask    |= PM_PBAMASK_TWIS0;        /* TWIS0 */
#endif
#ifdef CONFIG_SAM34_TWIM1
  mask    |= PM_PBAMASK_TWIM1;        /* TWIM1 */
#endif
#ifdef CONFIG_SAM34_TWIS1
  mask    |= PM_PBAMASK_TWIS1;        /* TWIS1 */
#endif
#ifdef CONFIG_SAM34_USART0
  mask    |= PM_PBAMASK_USART0;       /* USART0 */
  divmask |= PM_PBADIVMASK_CLK_USART;
#endif
#ifdef CONFIG_SAM34_USART1
  mask    |= PM_PBAMASK_USART1;       /* USART1 */
  divmask |= PM_PBADIVMASK_CLK_USART;
#endif
#ifdef CONFIG_SAM34_USART2
  mask    |= PM_PBAMASK_USART2;       /* USART2 */
  divmask |= PM_PBADIVMASK_CLK_USART;
#endif
#ifdef CONFIG_SAM34_USART3
  mask    |= PM_PBAMASK_USART3;       /* USART3 */
  divmask |= PM_PBADIVMASK_CLK_USART;
#endif
#ifdef CONFIG_SAM34_ADCIFE
  mask    |= PM_PBAMASK_ADCIFE;       /* ADCIFE */
#endif
#ifdef CONFIG_SAM34_DACC
  mask    |= PM_PBAMASK_DACC;         /* DACC */
#endif
#ifdef CONFIG_SAM34_ACIFC
  mask    |= PM_PBAMASK_ACIFC;        /* ACIFC */
#endif
#ifdef CONFIG_SAM34_GLOC
  mask    |= PM_PBAMASK_GLOC;         /* GLOC */
#endif
#ifdef CONFIG_SAM34_ABDACB
  mask    |= PM_PBAMASK_ABDACB;       /* ABDACB */
#endif
#ifdef CONFIG_SAM34_TRNG
  mask    |= PM_PBAMASK_TRNG;         /* TRNG */
#endif
#ifdef CONFIG_SAM34_PARC
  mask    |= PM_PBAMASK_PARC;         /* PARC */
#endif
#ifdef CONFIG_SAM34_CATB
  mask    |= PM_PBAMASK_CATB;         /* CATB */
#endif
#ifdef CONFIG_SAM34_TWIM2
  mask    |= PM_PBAMASK_TWIM2;        /* TWIM2 */
#endif
#ifdef CONFIG_SAM34_TWIM3
  mask    |= PM_PBAMASK_TWIM3;        /* TWIM3 */
#endif
#ifdef CONFIG_SAM34_LCDCA
  mask    |= PM_PBAMASK_LCDCA;        /* LCDCA*/
#endif
#endif

  /* Save the new PBA mask */

  putreg32(PM_UNLOCK_KEY(0xaa) | PM_UNLOCK_ADDR(SAM_PM_PBAMASK_OFFSET),
           SAM_PM_UNLOCK);
  putreg32(mask, SAM_PM_PBAMASK);

  /* Set the peripheral divider mask as necessary */

  putreg32(PM_UNLOCK_KEY(0xaa) | PM_UNLOCK_ADDR(SAM_PM_PBADIVMASK_OFFSET),
           SAM_PM_UNLOCK);
  putreg32(divmask, SAM_PM_PBADIVMASK);
}

/****************************************************************************
 * Name: sam_init_pbbmask
 *
 * Description:
 *   Called during boot to enable clocking on selected peripherals in the
 *   PBB mask register.
 *
 ****************************************************************************/

static inline void sam_init_pbbmask(void)
{
  /* Select the non-optional peripherals */

  uint32_t mask = PM_PBBMASK_FLASHCALW;

  /* OR in the user selected peripherals */

#ifdef CONFIG_SAM32_RESET_PERIPHCLKS
#ifdef CONFIG_SAM34_HRAMC1
  mask |= PM_PBBMASK_HRAMC1;          /* HRAMC1 */
#endif
#ifdef CONFIG_SAM34_HMATRIX
  mask |= PM_PBBMASK_HMATRIX;         /* HMATRIX */
#endif
#ifdef CONFIG_SAM34_PDCA
  mask |= PM_PBBMASK_PDCA;            /* PDCA */
#endif
#ifdef CONFIG_SAM34_CRCCU
  mask |= PM_PBBMASK_CRCCU;           /* CRCCU */
#endif
#ifdef CONFIG_SAM34_USBC
  mask |= PM_PBBMASK_USBC;            /* USBC */
#endif
#ifdef CONFIG_SAM34_PEVC
  mask |= PM_PBBMASK_PEVC;            /* PEVC */
#endif
#endif

  /* Save the new PBB mask */

  putreg32(PM_UNLOCK_KEY(0xaa) | PM_UNLOCK_ADDR(SAM_PM_PBBMASK_OFFSET),
           SAM_PM_UNLOCK);
  putreg32(mask, SAM_PM_PBBMASK);
}

/****************************************************************************
 * Name: sam_init_pbcmask
 *
 * Description:
 *   Called during boot to enable clocking on selected peripherals in the
 *   PBC mask register.
 *
 ****************************************************************************/

static inline void sam_init_pbcmask(void)
{
  /* Select the non-optional peripherals */

  uint32_t mask = (PM_PBCMASK_PM | PM_PBCMASK_SCIF | PM_PBCMASK_GPIO);

  /* OR in the user selected peripherals */

#ifdef CONFIG_SAM32_RESET_PERIPHCLKS
#ifdef CONFIG_SAM34_CHIPID
  mask |= PM_PBCMASK_CHIPID;          /* CHIPID */
#endif
#ifdef CONFIG_SAM34_FREQM
  mask |= PM_PBCMASK_FREQM;           /* FREQM */
#endif
#endif

  /* Save the new PBC mask */

  putreg32(PM_UNLOCK_KEY(0xaa) | PM_UNLOCK_ADDR(SAM_PM_PBCMASK_OFFSET),
           SAM_PM_UNLOCK);
  putreg32(mask, SAM_PM_PBCMASK);
}

/****************************************************************************
 * Name: sam_init_pbdmask
 *
 * Description:
 *   Called during boot to enable clocking on selected peripherals in the
 *   PBD mask register.
 *
 ****************************************************************************/

static inline void sam_init_pbdmask(void)
{
  /* Select the non-optional peripherals */

  uint32_t mask = (PM_PBDMASK_BPM | PM_PBDMASK_BSCIF);

  /* OR in the user selected peripherals */

#ifdef CONFIG_SAM32_RESET_PERIPHCLKS
#ifdef CONFIG_SAM34_AST
  mask |= PM_PBDMASK_AST;             /* AST */
#endif
#ifdef CONFIG_SAM34_WDT
  mask |= PM_PBDMASK_WDT;             /* WDT */
#endif
#ifdef CONFIG_SAM34_EIC
  mask |= PM_PBDMASK_EIC;             /* EIC */
#endif
#ifdef CONFIG_SAM34_PICOUART
  mask |= PM_PBDMASK_PICOUART;        /* PICOUART */
#endif
#endif

  /* Save the new PBD mask */

  putreg32(PM_UNLOCK_KEY(0xaa) | PM_UNLOCK_ADDR(SAM_PM_PBDMASK_OFFSET),
           SAM_PM_UNLOCK);
  putreg32(mask, SAM_PM_PBDMASK);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_init_periphclks
 *
 * Description:
 *   Called during boot to enable clocking on all selected peripherals.
 *
 ****************************************************************************/

void sam_init_periphclks(void)
{
  sam_init_cpumask();
  sam_init_hsbmask();
  sam_init_pbamask();
  sam_init_pbbmask();
  sam_init_pbcmask();
  sam_init_pbdmask();
}

/****************************************************************************
 * Name: sam_modifyperipheral
 *
 * Description:
 *   This is a convenience function that is intended to be used to enable
 *   or disable peripheral module clocking.
 *
 ****************************************************************************/

void sam_modifyperipheral(uintptr_t regaddr, uint32_t clrbits,
                          uint32_t setbits)
{
  irqstate_t flags;
  uint32_t regval;

  /* Make sure that the following operations are atomic */

  flags = irqsave();

  /* Enable/disabling clocking */

  regval  = getreg32(regaddr);
  regval &= ~clrbits;
  regval |= setbits;
  putreg32(PM_UNLOCK_KEY(0xaa) | PM_UNLOCK_ADDR(regaddr - SAM_PM_BASE),
           SAM_PM_UNLOCK);
  putreg32(regval, regaddr);

  irqrestore(flags);
}

/****************************************************************************
 * Name: sam_pba_modifydivmask
 *
 * Description:
 *   This is a convenience function that is intended to be used to modify
 *   bits in the PBA divided clock (DIVMASK) register.
 *
 ****************************************************************************/

void sam_pba_modifydivmask(uint32_t clrbits, uint32_t setbits)
{
  irqstate_t flags;
  uint32_t regval;

  /* Make sure that the following operations are atomic */

  flags = irqsave();

  /* Modify the PBA DIVMASK */

  regval  = getreg32(SAM_PM_PBADIVMASK);
  regval &= ~clrbits;
  regval |= setbits;
  putreg32(PM_UNLOCK_KEY(0xaa) | PM_UNLOCK_ADDR(SAM_PM_PBADIVMASK_OFFSET),
           SAM_PM_UNLOCK);
  putreg32(regval, SAM_PM_PBADIVMASK);

  irqrestore(flags);
}

/****************************************************************************
 * Name: sam_pba_enableperipheral
 *
 * Description:
 *   This is a convenience function to enable a peripheral on the APBA
 *   bridge.
 *
 ****************************************************************************/

void sam_pba_enableperipheral(uint32_t bitset)
{
  irqstate_t flags;

  /* The following operations must be atomic */

  flags = irqsave();

  /* Enable the APBA bridge if necessary */

  if (getreg32(SAM_PM_PBAMASK) == 0)
    {
      sam_hsb_enableperipheral(PM_HSBMASK_APBA);
    }

  irqrestore(flags);

  /* Enable the module */

  sam_enableperipheral(SAM_PM_PBAMASK, bitset);
}

/****************************************************************************
 * Name: sam_pba_disableperipheral
 *
 * Description:
 *   This is a convenience function to disable a peripheral on the APBA
 *   bridge.
 *
 ****************************************************************************/

void sam_pba_disableperipheral(uint32_t bitset)
{
  irqstate_t flags;

  /* Disable clocking to the module */

  sam_disableperipheral(SAM_PM_PBAMASK, bitset);

  /* Disable the APBA bridge if possible */

  flags = irqsave();

  if (getreg32(SAM_PM_PBAMASK) == 0)
    {
      sam_hsb_disableperipheral(PM_HSBMASK_APBA);
    }

   /* Disable PBA UART divided clock if none of the UARTS are in use */

  if ((getreg32(SAM_PM_PBAMASK) & PM_PBAMASK_UARTS) == 0)
    {
      sam_pba_disabledivmask(PM_PBADIVMASK_CLK_USART);
    }

   /* Disable PBA TIMER divided clocks if none of the UARTS are in use */

  if ((getreg32(SAM_PM_PBAMASK) & PM_PBAMASK_TIMERS) == 0)
    {
      sam_pba_disabledivmask(PM_PBADIVMASK_TIMER_CLOCKS);
    }

  irqrestore(flags);
}

/****************************************************************************
 * Name: sam_pbb_enableperipheral
 *
 * Description:
 *   This is a convenience function to enable a peripheral on the APBB
 *   bridge.
 *
 ****************************************************************************/

void sam_pbb_enableperipheral(uint32_t bitset)
{
  irqstate_t flags;

  /* The following operations must be atomic */

  flags = irqsave();

  /* Enable the APBB bridge if necessary */

  if (getreg32(SAM_PM_PBBMASK) == 0)
    {
      sam_hsb_enableperipheral(PM_HSBMASK_APBB);
    }

  irqrestore(flags);

  /* Enable the module */

  sam_enableperipheral(SAM_PM_PBBMASK, bitset);
}

/****************************************************************************
 * Name: sam_pbb_disableperipheral
 *
 * Description:
 *   This is a convenience function to disable a peripheral on the APBA
 *   bridge.
 *
 ****************************************************************************/

void sam_pbb_disableperipheral(uint32_t bitset)
{
  irqstate_t flags;

  /* Disable clocking to the peripheral module */

  sam_disableperipheral(SAM_PM_PBBMASK, bitset);

  /* Disable the APBB bridge if possible */

  flags = irqsave();

  if (getreg32(SAM_PM_PBBMASK) == 0)
    {
      sam_hsb_disableperipheral(PM_HSBMASK_APBB);
    }

  irqrestore(flags);
}

/****************************************************************************
 * Name: sam_usbc_enableclk
 *
 * Description:
 *   Enable clocking for the USBC using settings from the board.h header files.
 *
 *  "The USBC has two bus clocks connected: One High Speed Bus clock
 *   (CLK_USBC_AHB) and one Peripheral Bus clock (CLK_USBC_APB). These clocks
 *   are generated by the Power Manager.  Both clocks are enabled at reset
 *   and can be disabled by the Power Manager. It is recommended to disable
 *   the USBC before disabling the clocks, to avoid freezing the USBC in
 *   an undefined state.
 *
 *  "To follow the usb data rate at 12Mbit/s in full-speed mode, the
 *   CLK_USBC_AHB clock should be at minimum 12MHz.
 *
 *  "The 48MHz USB clock is generated by a dedicated generic clock from
 *   the SCIF module. Before using the USB, the user must ensure that the
 *   USB generic clock (GCLK_USBC) is enabled at 48MHz in the SCIF module."
 *
 ****************************************************************************/

#ifdef CONFIG_SAM34_USBC
void sam_usbc_enableclk(void)
{
  irqstate_t flags;
  uint32_t regval;

  /* Enable USBC clocking (possibly along with the PBB peripheral bridge) */

  flags = irqsave();
  sam_hsb_enableperipheral(PM_HSBMASK_USBC);
  sam_pbb_enableperipheral(PM_PBBMASK_USBC);

  /* Reset generic clock 7 */

  putreg32(0, SAM_SCIF_GCCTRL7);

  /* Set the generic clock source */

  regval  = getreg32(SAM_SCIF_GCCTRL7);
  regval &= ~SCIF_GCCTRL_OSCSEL_MASK;
  regval |= SAM_USBC_GCLK_SOURCE;
  putreg32(regval, SAM_SCIF_GCCTRL7);

  /* Set the generic clock divider */

  regval  = getreg32(SAM_SCIF_GCCTRL7);
  regval &= ~(SCIF_GCCTRL_DIVEN | SCIF_GCCTRL_DIV_MASK);

#if BOARD_USBC_GCLK_DIV > 1
  regval |= SCIF_GCCTRL_DIVEN;
  regval |= SCIF_GCCTRL_DIV(((divider + 1) / 2) - 1);
#endif

  putreg32(regval, SAM_SCIF_GCCTRL7);

  /* Enable the generic clock */

  regval  = getreg32(SAM_SCIF_GCCTRL7);
  regval |= SCIF_GCCTRL_CEN;
  putreg32(regval, SAM_SCIF_GCCTRL7);
  irqrestore(flags);
}
#endif /* CONFIG_SAM34_USBC */

/****************************************************************************
 * Name: sam_usbc_disableclk
 *
 * Description:
 *   Disable clocking to the USBC.
 *
 ****************************************************************************/

#ifdef CONFIG_SAM34_USBC
void sam_usbc_disableclk(void)
{
  putreg32(0, SAM_SCIF_GCCTRL7);
  sam_pbb_enableperipheral(PM_PBBMASK_USBC);
  sam_hsb_enableperipheral(PM_HSBMASK_USBC);
}
#endif /* CONFIG_SAM34_USBC */
