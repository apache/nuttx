/****************************************************************************
 * arch/arm/src/tiva/tiva_adclib.c
 *
 *   Copyright (C) 2015 TRD2 Inc. All rights reserved.
 *   Author: Calvin Maguranis <calvin.maguranis@trd2inc.com>
 *
 * References:
 *
 *   TM4C123GH6PM Series Data Sheet
 *   TI Tivaware driverlib ADC sample code.
 *
 * The Tivaware sample code has a BSD compatible license that requires this
 * copyright notice:
 *
 * Copyright (c) 2005-2014 Texas Instruments Incorporated.  All rights reserved.
 * Software License Agreement
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * This is part of revision 2.1.0.12573 of the Tiva Peripheral Driver Library.
 *****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/arch.h>
#include <nuttx/analog/adc.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>
#include <assert.h>

#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"
#include "tiva_gpio.h"
#include "tiva_adc.h"
#include "chip/tiva_adc.h"
#include "chip/tiva_pinmap.h"

#if defined (CONFIG_TIVA_ADC0) || defined (CONFIG_TIVA_ADC1)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CLOCK_CONFIG(div, src) \
    ( ((((div) << ADC_CC_CLKDIV_SHIFT) & ADC_CC_CLKDIV_MASK) | \
    ((src) & ADC_CC_CS_MASK)) & (ADC_CC_CLKDIV_MASK + ADC_CC_CS_MASK) )

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t sse2irq[] =
{
#ifdef CONFIG_TIVA_ADC0
  TIVA_IRQ_ADC0, TIVA_IRQ_ADC1, TIVA_IRQ_ADC2, TIVA_IRQ_ADC3
#endif
#ifdef CONFIG_TIVA_ADC1
  , TIVA_IRQ_ADC1_0, TIVA_IRQ_ADC1_1, TIVA_IRQ_ADC1_2, TIVA_IRQ_ADC1_3
#endif
};

static uint32_t ain2gpio[] =
{
#ifdef GPIO_ADC_AIN0
  GPIO_ADC_AIN0
#endif
#ifdef GPIO_ADC_AIN1
  , GPIO_ADC_AIN1
#endif
#ifdef GPIO_ADC_AIN2
  , GPIO_ADC_AIN2
#endif
#ifdef GPIO_ADC_AIN3
  , GPIO_ADC_AIN3
#endif
#ifdef GPIO_ADC_AIN4
  , GPIO_ADC_AIN4
#endif
#ifdef GPIO_ADC_AIN5
  , GPIO_ADC_AIN5
#endif
#ifdef GPIO_ADC_AIN6
  , GPIO_ADC_AIN6
#endif
#ifdef GPIO_ADC_AIN7
  , GPIO_ADC_AIN7
#endif
#ifdef GPIO_ADC_AIN8
  , GPIO_ADC_AIN8
#endif
#ifdef GPIO_ADC_AIN9
  , GPIO_ADC_AIN9
#endif
#ifdef GPIO_ADC_AIN10
  , GPIO_ADC_AIN10
#endif
#ifdef GPIO_ADC_AIN11
  , GPIO_ADC_AIN11
#endif
#ifdef GPIO_ADC_AIN12
  , GPIO_ADC_AIN12
#endif
#ifdef GPIO_ADC_AIN13
  , GPIO_ADC_AIN13
#endif
#ifdef GPIO_ADC_AIN14
  , GPIO_ADC_AIN14
#endif
#ifdef GPIO_ADC_AIN15
  , GPIO_ADC_AIN15
#endif
#ifdef GPIO_ADC_AIN16
  , GPIO_ADC_AIN16
#endif
#ifdef GPIO_ADC_AIN17
  , GPIO_ADC_AIN17
#endif
#ifdef GPIO_ADC_AIN18
  , GPIO_ADC_AIN18
#endif
#ifdef GPIO_ADC_AIN19
  , GPIO_ADC_AIN19
#endif
#ifdef GPIO_ADC_AIN20
  , GPIO_ADC_AIN20
#endif
#ifdef GPIO_ADC_AIN21
  , GPIO_ADC_AIN21
#endif
#ifdef GPIO_ADC_AIN22
  , GPIO_ADC_AIN22
#endif
#ifdef GPIO_ADC_AIN23
  , GPIO_ADC_AIN23
#endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tiva_adc_one_time_init
 *
 * Description:
 *   Perform one-time initialization of global ADC settings; clock frequency
 *   and sampling rate.
 *
 * Assumptions/Limitations:
 *   Peripheral must be powered before one-time initialization.
 *
 ****************************************************************************/

void tiva_adc_one_time_init(uint32_t clock, uint8_t sample_rate)
{
  static bool one_time_init = false;

#ifdef CONFIG_DEBUG_ANALOG
  avdbg("setting clock=%d MHz sample rate=%d\n",
         clock, sample_rate);
#endif

  /* Have the common peripheral properties already been initialized? If yes,
   * continue.
   */

  if (one_time_init == false)
    {
      avdbg("performing ADC one-time initialization...\n");
      /* set clock */

      tiva_adc_clock(clock);

      /* set sampling rate */

      tiva_adc_sample_rate(sample_rate);

#ifdef CONFIG_ARCH_CHIP_TM4C129
      /* voltage reference */

      tiva_adc_vref();
#endif
      one_time_init = true;
    }
#ifdef CONFIG_DEBUG_ANALOG
  else
    {
      avdbg("one time initialization previously completed\n");
    }
#endif
}

/****************************************************************************
 * Name: tiva_adc_configure
 *
 * Description:
 *   Performs configuration of a single ADC, it's valid sample sequencers and
 *   available steps.
 *
 ****************************************************************************/

void tiva_adc_configure(struct tiva_adc_cfg_s *cfg)
{
    uint8_t s;
    uint8_t c;

    avdbg("configure ADC%d...\n", cfg->adc);

    /* Configure each SSE */

    for (s=0; s<4; ++s)
      {
        if (cfg->sse[s])
          {
            tiva_adc_sse_cfg(cfg->adc, s, &cfg->ssecfg[s]);
          }
#ifdef CONFIG_DEBUG_ANALOG
        else
          {
            avdbg("ADC%d SSE%d has no configuration\n", cfg->adc, s);
          }
#endif
      }

    /* Configure each step */

    for (c=0; c<cfg->steps; ++c)
      {
          tiva_adc_step_cfg(&cfg->stepcfg[c]);
      }

#ifdef CONFIG_DEBUG_ANALOG
    tiva_adc_dump_reg_cfg(cfg->adc, 0);
#endif
}

/****************************************************************************
 * Name: tiva_adc_sse_cfg
 *
 * Description:
 *   Configure the sample sequencer.
 *
 ****************************************************************************/

void tiva_adc_sse_cfg(uint8_t adc, uint8_t sse,
                      struct tiva_adc_sse_cfg_s *ssecfg)
{
  avdbg("configure ADC%d SSE%d...\n", adc, sse);
#ifdef CONFIG_DEBUG_ANALOG
  avdbg("priority=%d trigger=%d...\n", ssecfg->priority, ssecfg->trigger);
#endif

  uint8_t priority = ssecfg->priority;
  uint8_t trigger  = ssecfg->trigger;

  /* Ensure SSE is disabled before configuring */

  tiva_adc_sse_enable(adc, sse, false);

  /* Set conversion priority and trigger source for all steps in the SSE */

  tiva_adc_sse_priority(adc, sse, priority);
  tiva_adc_sse_trigger(adc, sse, trigger);
}

/****************************************************************************
 * Name: tiva_adc_step_cfg
 *
 * Description:
 *   Configures a sample sequencer step.
 *
 ****************************************************************************/

void tiva_adc_step_cfg(struct tiva_adc_step_cfg_s *stepcfg)
{
#ifdef CONFIG_DEBUG_ANALOG
  avdbg("  shold=0x%02x flags=0x%02x ain=%d...\n",
        stepcfg->shold, stepcfg->flags, stepcfg->ain);
#endif

  uint8_t  adc   = stepcfg->adc;
  uint8_t  sse   = stepcfg->sse;
  uint8_t  step  = stepcfg->step;
#ifdef CONFIG_EXPERIMENTAL
  uint8_t  shold = stepcfg->shold;
#endif
  uint8_t  flags = stepcfg->flags;
  uint8_t  ain   = stepcfg->ain;
  uint32_t gpio  = ain2gpio[stepcfg->ain];

  avdbg("configure ADC%d SSE%d STEP%d...\n", adc, sse, step);

  /* Configure the AIN GPIO for analog input if not flagged to be muxed to
   * the internal temperature sensor
   */

  if ((flags & TIVA_ADC_FLAG_TS) != TIVA_ADC_FLAG_TS)
    {
      tiva_configgpio(gpio);
    }

  /* Register, set sample/hold time and configure the step */

  tiva_adc_sse_register_chn(adc, sse, step, ain);
  tiva_adc_sse_differential(adc, sse, step, 0); /* TODO: update when differential
                                        * support is added. */
#ifdef CONFIG_EXPERIMENTAL
  tiva_adc_sse_sample_hold_time(adc, sse, step, shold);
#endif
  tiva_adc_sse_step_cfg(adc, sse, step, flags);
}

/****************************************************************************
 * Name: tiva_adc_get_trigger
 *
 * Description:
 *   For a given adc, sse and step, get the AIN (channel) associated.
 *
 ****************************************************************************/

uint8_t tiva_adc_get_trigger(uint8_t adc, uint8_t sse)
{
  uintptr_t emuxaddr = TIVA_ADC_EMUX(adc);
  uint32_t emux = getreg32(emuxaddr) & ADC_EMUX_MASK(sse);
  return (emux >> ADC_EMUX_SHIFT(sse));
}

/****************************************************************************
 * Name: tiva_adc_get_ain
 *
 * Description:
 *   For a given adc, sse and step, get the AIN (channel) associated.
 *
 ****************************************************************************/

uint8_t tiva_adc_get_ain(uint8_t adc, uint8_t sse, uint8_t step)
{
  uintptr_t ssmuxaddr = TIVA_ADC_BASE(adc)+TIVA_ADC_SSMUX(sse);
  uint32_t ssmux = getreg32(ssmuxaddr) & ADC_SSMUX_MUX_MASK(step);
  return (ssmux >> ADC_SSMUX_MUX_SHIFT(step));
}

/* IRQS *********************************************************************/

/****************************************************************************
 * Name: tiva_adc_irq_attach
 *
 * Description:
 *  Attach a custom interrupt handler.
 *
 ****************************************************************************/

void tiva_adc_irq_attach(uint8_t adc, uint8_t sse, xcpt_t isr)
{

  uint32_t ret = 0;
  int irq = sse2irq[SSE_IDX(adc, sse)];

#ifdef CONFIG_DEBUG_ANALOG
  avdbg("assigning ISR=0x%p to ADC%d SSE%d IRQ=0x%02x...\n",
        isr, adc, sse, irq);
#endif

  ret = irq_attach(irq, isr);
  if (ret < 0)
    {
      adbg("ERROR: Failed to attach IRQ %d: %d\n", irq, ret);
      return;
    }

  up_enable_irq(irq);
}

/****************************************************************************
 * Name: tiva_adc_irq_detach
 *
 * Description:
 *  detach an interrupt handler.
 *
 ****************************************************************************/

void tiva_adc_irq_detach(uint8_t adc, uint8_t sse)
{
  uint32_t ret = 0;
  int irq = sse2irq[SSE_IDX(adc, sse)];

  /* Disable ADC interrupts at the level of the AIC */

  up_disable_irq(irq);

  /* Then detach the ADC interrupt handler. */

  ret = irq_detach(irq);
  if (ret < 0)
    {
      adbg("ERROR: Failed to detach IRQ %d: %d\n", irq, ret);
      return;
    }
}

/****************************************************************************
 * Name: tiva_adc_getirq
 *
 * Description:
 *  Maps ADC and SSE value to the correct IRQ value.
 *
 ****************************************************************************/

int tiva_adc_getirq(uint8_t adc, uint8_t sse)
{
  return sse2irq[SSE_IDX(adc, sse)];
}

/* Peripheral (base) level **************************************************/

/****************************************************************************
 * Name: tiva_adc_enable
 *
 * Description:
 *  Toggles the operational state of the ADC peripheral
 *
 * Input Parameters:
 *  state - operation state
 *
 ****************************************************************************/

int tiva_adc_enable(uint8_t adc, bool state)
{
  if (state == true)
    {
      /* Enable clocking to the ADC peripheral */

#ifdef TIVA_SYSCON_RCGCADC
      modifyreg32(TIVA_SYSCON_RCGCADC, 0, 1 << adc);
#else
      modifyreg32(TIVA_SYSCON_RCGC0, 0, SYSCON_RCGC0_ADC0);
#endif
      return OK;
    }
  else if (state == false)
    {
      /* Disable clocking to the ADC peripheral */

#ifdef TIVA_SYSCON_RCGCADC
      modifyreg32(TIVA_SYSCON_RCGCADC, 1 << adc, 0);
#else
      modifyreg32(TIVA_SYSCON_RCGC0, SYSCON_RCGC0_ADC0, 0);
#endif
      return OK;
    }

  /* ERROR! */

  return -1;
}

/****************************************************************************
 * Name: tiva_adc_clock
 *
 * Description:
 *  Sets the ADC peripherals clock to the desired frequency.
 *
 * Input Parameters:
 *  freq - ADC clock value; dependent on platform:
 *
 *  TM4C123 - Select either MOSC or PIOSC. Both result in 16 MHz operation,
 *  however the PIOSC allows the ADC to operate in deep sleep mode.
 *
 *  TM4C129 - For the 129, there is still a selection between various internal
 *  clocks, however the output frequency is variable (16 MHz - 32 MHz); so it
 *  is much more intuitive to allow the clock variable be a frequency value.
 *
 ****************************************************************************/

void tiva_adc_clock(uint32_t freq)
{
#if defined(CONFIG_ARCH_CHIP_TM4C123)
  /* For the TM4C123, the ADC clock source does not affect the frequency, it
   * runs at 16 MHz regardless. You end up selecting between the MOSC (default)
   * or the PIOSC. The PIOSC allows the ADC to operate even in deep sleep mode.
   * Since this is the case, the clock value for the TM4C123 is always 16 MHz
   */

  uintptr_t ccreg = (TIVA_ADC0_BASE + TIVA_ADC_CC_OFFSET);
  modifyreg32(ccreg, ADC_CC_CS_MASK, (ADC_CC_CS_PIOSC & ADC_CC_CS_MASK));

#elif defined (CONFIG_ARCH_CHIP_TM4C129)
  /* check clock bounds and specific match cases */

  uint32_t clk_src = 0;
  uint32_t div = 0;
  if (freq > TIVA_ADC_CLOCK_MAX)
    {
      clk_src = ADC_CC_CS_SYSPLL;
      div = (BOARD_FVCO_FREQUENCY / TIVA_ADC_CLOCK_MAX);
    }
  else if (freq < TIVA_ADC_CLOCK_MIN)
    {
      clk_src = ADC_CC_CS_PIOSC;
      div = 1;
    }
  else if (freq == XTAL_FREQUENCY)
    {
      clk_src = ADC_CC_CS_MOSC;
      div = 1;
    }
  else
    {
      clk_src = ADC_CC_CS_SYSPLL;
      div = (BOARD_FVCO_FREQUENCY / freq);
    }

  uintptr_t ccreg = (TIVA_ADC0_BASE + TIVA_ADC_CC_OFFSET);
  modifyreg32(ccreg, ADC_CC_CLKDIV_MASK, CLOCK_CONFIG(div, clk_src));
#else
#  error Unsupported architecture reported
#endif
}

/****************************************************************************
 * Name: tiva_adc_vref
 *
 * Description:
 *  Sets the ADC peripherals clock to the desired frequency.
 *
 * Input Parameters:
 *  vref - ADC clock voltage reference source
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_CHIP_TM4C129
void tiva_adc_vref(uint8_t vref)
{
  uintptr_t ctlreg = (TIVA_ADC0_BASE + TIVA_ADC_CTL_OFFSET);
  modifyreg32(ctlreg, ADC_CTL_VREF_MASK, (vref & ADC_CTL_VREF_MASK));
}
#endif

/****************************************************************************
 * Name: tiva_adc_sample_rate
 *
 * Description:
 *  Sets the ADC sample rate as follows for each processor.
 *  TM4C123 - by maximum samples: 125 ksps, 250 ksps, 500 ksps or 1 Msps
 *  TM4C129 - by a divisor either being full, half, quarter or
 *  an eighth.
 *
 * Input Parameters:
 *  rate - ADC sample rate divisor
 *
 ****************************************************************************/

void tiva_adc_sample_rate(uint8_t rate)
{
  uintptr_t pcreg = (TIVA_ADC0_BASE + TIVA_ADC_PC_OFFSET);

  /* NOTE: ADC_PC_SR_MASK is intended for use with the TM4C123, the
   * alternative is ADC_PC_MCR_MASK for the TM4C129. However both masks
   * mask off the first 4 bits (0xF) so there is no need to distinguish
   * between the two.
   */

  modifyreg32(pcreg, ADC_PC_SR_MASK, (rate & ADC_PC_SR_MASK));
}

/****************************************************************************
 * Name: tiva_adc_proc_trig
 *
 * Description:
 *   Triggers the sample sequence to start it's conversion(s) and store them
 *   to the FIFO. This is only required when the trigger source is set to the
 *   processor.
 *
 * Input parameters:
 *   adc - which ADC peripherals' sample sequencers to trigger
 *   sse_mask - sample sequencer bitmask, each sse is 1 shifted by the sse
 *              number. e.g.
 *              SSE0 = 1 << 0
 *              SSE1 = 1 << 1
 *              SSE2 = 1 << 2
 *              SSE3 = 1 << 3
 *
 ****************************************************************************/

void tiva_adc_proc_trig(uint8_t adc, uint8_t sse_mask)
{
  uintptr_t pssireg = TIVA_ADC_PSSI(adc);
  putreg32((sse_mask & ADC_PSSI_TRIG_MASK), pssireg);
#ifdef CONFIG_TIVA_ADC_SYNC
#  warning CONFIG_TIVA_ADC_SYNC unsupported at this time.
#endif
}

/****************************************************************************
 * Name: tiva_adc_int_status
 *
 * Description:
 *   Returns raw interrupt status for the input ADC
 *
 * Input parameters:
 *   adc - which ADC peripherals' interrupt status to retrieve
 *
 ****************************************************************************/

uint32_t tiva_adc_int_status(uint8_t adc)
{
  uint32_t ris = getreg32(TIVA_ADC_RIS(adc));
  return ris;
}

/* Sample sequencer (SSE) functions *****************************************/

/****************************************************************************
 * Name: tiva_adc_sse_enable
 *
 * Description:
 *   Sets the operation state of an ADC's sample sequencer (SSE). SSEs must
 *   be configured before being enabled.
 *
 * Input parameters:
 *   adc - peripheral state
 *   sse - sample sequencer
 *   state - sample sequencer enable/disable state
 *
 * Return value:
 *   Actual state of the ACTSS register.
 *
 ****************************************************************************/

uint8_t tiva_adc_sse_enable(uint8_t adc, uint8_t sse, bool state)
{
  avdbg("ADC%d SSE%d=%01d\n", adc, sse, state);

  uintptr_t actssreg = TIVA_ADC_ACTSS(adc);
  if (state == true)
    {
      modifyreg32(actssreg, 0, (1 << sse));
    }
  else
    {
      modifyreg32(actssreg, (1 << sse), 0);
    }

  return (getreg32(actssreg) & 0xF);
}

/****************************************************************************
 * Name: tiva_adc_sse_trigger
 *
 * Description:
 *   Sets the trigger configuration for an ADC's sample sequencer (SSE).
 *   Possible triggers are the following:
 *      - Processor
 *      - PWMs, requires that one of the PWMnn_TRIG_CFG defines be OR'd
 *        into the trigger value.
 *      - Timers
 *      - GPIO (which GPIO is platform specific, consult the datasheet)
 *      - Always
 *      - !!UNSUPPORTED: Comparators
 *
 * Input parameters:
 *   adc - peripheral state
 *   sse - sample sequencer
 *   trigger - interrupt trigger
 *
 ****************************************************************************/

void tiva_adc_sse_trigger(uint8_t adc, uint8_t sse, uint32_t trigger)
{
  uintptr_t emuxreg = (TIVA_ADC_EMUX(adc));
  uint32_t trig = ((trigger << ADC_EMUX_SHIFT(sse)) & ADC_EMUX_MASK(sse));
  modifyreg32(emuxreg, ADC_EMUX_MASK(sse), trig);

  /* NOTE: PWM triggering needs an additional register to be set (ADC_TSSEL)
   * A platform specific IOCTL command is provided to configure the triggering.
   */
}

/****************************************************************************
 * Name: tiva_adc_sse_pwm_trig
 *
 * Description:
 *   Additional triggering configuration for PWM. Sets which PWM and which
 *   generator.
 *
 * Input parameters:
 *   adc - peripheral state
 *   sse - sample sequencer
 *   cfg - which PWM modulator and generator to use, use TIVA_ADC_PWM_TRIG
 *         to encode the value correctly
 *
 ****************************************************************************/

#ifdef CONFIG_EXPERIMENTAL
void tiva_adc_sse_pwm_trig(uint8_t adc, uint8_t sse, uint32_t cfg)
{
  /* PWM triggering needs an additional register to be set (ADC_TSSEL) */

  uintptr_t tsselreg = TIVA_ADC_TSSEL(adc);

  modifyreg32(tsselreg, ADC_TSSEL_PS_MASK(see), cfg);
}
#endif

/****************************************************************************
 * Name: tiva_adc_sse_int_enable
 *
 * Description:
 *   Sets the interrupt state of an ADC's sample sequencer (SSE). SSEs must
 *   be enabled before setting interrupt state.
 *
 * Input parameters:
 *   adc - peripheral state
 *   sse - sample sequencer
 *   state - sample sequencer enable/disable interrupt state
 *
 ****************************************************************************/

void tiva_adc_sse_int_enable(uint8_t adc, uint8_t sse, bool state)
{
  irqstate_t flags;
  uintptr_t imreg = TIVA_ADC_IM(adc);
  int irq = tiva_adc_getirq(adc, sse);

  flags = irqsave();
  up_disable_irq(irq);

  tiva_adc_sse_clear_int(adc, sse);

  if (state == true)
    {
      modifyreg32(imreg, 0, (1 << sse));
    }
  else
    {
      modifyreg32(imreg, (1 << sse), 0);
    }

  up_enable_irq(irq);
  irqrestore(flags);
}

/****************************************************************************
 * Name: tiva_adc_sse_int_status
 *
 * Description:
 *   Returns interrupt status for the specificed SSE
 *
 * Input parameters:
 *   adc - which ADC peripherals' interrupt status to retrieve
 *   sse - which SSE interrupt status to retrieve
 *
 ****************************************************************************/

bool tiva_adc_sse_int_status(uint8_t adc, uint8_t sse)
{
  uint32_t intstat = tiva_adc_int_status(adc);
  uint32_t sseintstat = intstat & (1 << sse);
  return sseintstat > 0 ? true : false;
}

/****************************************************************************
 * Name: tiva_adc_sse_clear_int
 *
 * Description:
 *   Clears the interrupt bit for the SSE.
 *
 * Input parameters:
 *   adc - peripheral state
 *   sse - sample sequencer
 *   state - sample sequencer
 *
 ****************************************************************************/

void tiva_adc_sse_clear_int(uint8_t adc, uint8_t sse)
{
  uintptr_t iscreg = TIVA_ADC_ISC(adc);
  modifyreg32(iscreg, 0, (1 << sse));
}

/****************************************************************************
 * Name: tiva_adc_sse_data
 *
 * Description:
 *   Retrieves data from the FIFOs for all steps in the given sample sequencer.
 *   The input data buffer MUST be as large or larger than the sample sequencer.
 *   otherwise
 *
 * Input parameters:
 *   adc - peripheral state
 *   sse - sample sequencer
 *
 * Return value:
 *   number of steps read from FIFO.
 *
 ****************************************************************************/

uint8_t tiva_adc_sse_data(uint8_t adc, uint8_t sse, int32_t *buf)
{
  uint32_t ssfstatreg = getreg32(TIVA_ADC_BASE(adc) + TIVA_ADC_SSFSTAT(sse));
  uint8_t fifo_count = 0;

  /* Read samples from the FIFO until it is empty */

  while (!(ssfstatreg & ADC_SSFSTAT_EMPTY) && fifo_count < 8)
    {
      /* Read the FIFO and copy it to the destination */

      buf[fifo_count] = getreg32(TIVA_ADC_BASE(adc) + TIVA_ADC_SSFIFO(sse));
      fifo_count++;

      /* refresh fifo status register state */

      ssfstatreg = getreg32(TIVA_ADC_BASE(adc) + TIVA_ADC_SSFSTAT(sse));
    }

  avdbg("fifo=%d\n", fifo_count);

  return fifo_count;
}

/****************************************************************************
 * Name: tiva_adc_sse_priority
 *
 * Description:
 *   Sets the priority configuration for an ADC's sample sequencer (SSE). The
 *   priority value ranges from 0 to 3, 0 being the highest priority, 3 being
 *   the lowest. There can be no duplicate values.
 *
 * Input parameters:
 *   adc - peripheral state
 *   sse - sample sequencer
 *   priority - conversion priority
 *
 ****************************************************************************/

void tiva_adc_sse_priority(uint8_t adc, uint8_t sse, uint8_t priority)
{
  uintptr_t ssprireg = TIVA_ADC_SSPRI(adc);
  uint32_t sspri = 0;

  sspri = (priority << ADC_SSPRI_SHIFT(sse)) & ADC_SSPRI_MASK(sse);
  modifyreg32(ssprireg, ADC_SSPRI_MASK(sse), sspri);
}

/****************************************************************************
 * Name: tiva_adc_sse_register_chn
 *
 * Description:
 *   Registers an input channel to an SSE. Channels are registered according
 *   to the step and channel values stored in the channel struct. If the SSE
 *   already has a channel registered, it is overwritten by the new channel.
 *
 *   *SSEMUX only supported on TM4C129 devices
 *
 * Input parameters:
 *   adc - peripheral state
 *   sse - sample sequencer
 *   chn - sample sequencer step
 *   ain - analog input pin
 *
 ****************************************************************************/

void tiva_adc_sse_register_chn(uint8_t adc, uint8_t sse, uint8_t chn,
                             uint32_t ain)
{
  /* Configure SSE mux (SSMUX) with step number */

  uintptr_t ssmuxreg = (TIVA_ADC_BASE(adc) + TIVA_ADC_SSMUX(sse));
  uint32_t step = 0;

  step = ((ain << ADC_SSMUX_MUX_SHIFT(chn)) & ADC_SSMUX_MUX_MASK(chn));
  modifyreg32(ssmuxreg, ADC_SSMUX_MUX_MASK(chn), step);

#ifdef CONFIG_ARCH_CHIP_TM4C129
  /* Configure SSE extended mux (SSEMUX) with step number and configuration */

  ssmuxreg = (TIVA_ADC_BASE(adc) + TIVA_ADC_SSEMUX(sse));
  step = ((1 << ADC_SSEMUX_MUX_SHIFT(chn)) & ADC_SSEMUX_MUX_MASK(chn));
  modifyreg32(ssmuxreg, ADC_SSEMUX_MUX_MASK(chn), step);
#endif
}

/****************************************************************************
 * Name: tiva_adc_sse_differential
 *
 * Description:
 *   Sets the differential capability for a SSE. !! UNSUPPORTED
 *
 * Input parameters:
 *   adc - peripheral state
 *   sse - sample sequencer
 *   chn - sample sequencer channel
 *   diff - differential configuration
 *
 ****************************************************************************/

void tiva_adc_sse_differential(uint8_t adc, uint8_t sse, uint8_t chn, uint32_t diff)
{
#ifdef CONFIG_TIVA_ADC_DIFFERENTIAL
#  error CONFIG_TIVA_ADC_DIFFERENTIAL unsupported!!
#else
  /* for now, ensure the FIFO is used and differential sampling is disabled */

  uintptr_t ssopreg = (TIVA_ADC_BASE(adc) + TIVA_ADC_SSOP(sse));
  uint32_t sdcopcfg = (1 << chn);
  modifyreg32(ssopreg, sdcopcfg, 0);
#endif
}

/****************************************************************************
 * Name: tiva_adc_sse_sample_hold_time
 *
 * Description:
 *  Set the sample and hold time for this step.
 *
 *  This is not available on all devices, however on devices that do not
 *  support this feature these reserved bits are ignored on write access.
 *
 * Input parameters:
 *   adc - peripheral state
 *   sse - sample sequencer
 *   chn - sample sequencer channel
 *   shold - sample and hold time
 *
 ****************************************************************************/

#ifdef CONFIG_EXPERIMENTAL
void tiva_adc_sse_sample_hold_time(uint8_t adc, uint8_t sse,
                                 uint8_t chn, uint32_t shold)
{
  uintptr_t sstshreg = (TIVA_ADC_BASE(adc) + TIVA_ADC_SSTSH(sse));
  modifyreg32(sstshreg, ADC_SSTSH_MASK(sse), (shold << ADC_SSTSH_SHIFT(sse)));
}
#endif

/****************************************************************************
 * Name: tiva_adc_sse_step_cfg
 *
 * Description:
 *   Configures the given SSE step to one of the following options:
 *      -Temperature sensor select: this step is muxed to the internal
 *       temperature sensor.
 *      -Interrupt enabled select: this step causes the interrupt bit to
 *       be set and, if the MASK0 bit in ADC_IM register is set, the
 *       interrupt is promoted to the interrupt controller.
 *      -Sequence end select: This step is the last sequence to be sampled.
 *       This MUST be set somewhere in the SSE.
 *      -*Comparator/Differential select: The analog input is differentially
 *       sampled. The corresponding ADCSSMUXn nibble must be set to the pair
 *       number "i", where the paired inputs are "2i and 2i+1". Because the
 *       temperature sensor does not have a differential option, this bit must
 *       not be set when the TS3 bit is set.
 *
 *  *Comparator/Differential functionality is unsupported and ignored.
 *
 * Input parameters:
 *   adc - peripheral state
 *   sse - sample sequencer
 *   chn - sample sequencer channel
 *   cfg - step configuration
 *
 ****************************************************************************/

void tiva_adc_sse_step_cfg(uint8_t adc, uint8_t sse, uint8_t chn, uint8_t cfg)
{
  uintptr_t ssctlreg = (TIVA_ADC_BASE(adc) + TIVA_ADC_SSCTL(sse));
  uint32_t ctlcfg = cfg << ADC_SSCTL_SHIFT(chn) & ADC_SSCTL_MASK(chn);
  modifyreg32(ssctlreg, ADC_SSCTL_MASK(chn), ctlcfg);
}

/****************************************************************************
 * Name: tiva_adc_dump_reg_cfg
 *
 * Description:
 *   Dump all configured registers for the given ADC and SSE. This should
 *   only be used to verify that configuration routines were accurate.
 *
 * Input parameters:
 *   adc - peripheral state
 *   sse - sample sequencer
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_ANALOG
void tiva_adc_dump_reg_cfg(uint8_t adc, uint8_t sse)
{
  /* one-time initialization */

  uintptr_t ccreg = (TIVA_ADC0_BASE + TIVA_ADC_CC_OFFSET); /* Clock */
  uintptr_t pcreg = (TIVA_ADC0_BASE + TIVA_ADC_PC_OFFSET); /* Sample rate */

  /* SSE cfg */

  uintptr_t actssreg = TIVA_ADC_ACTSS(adc);  /* SSE enable state */
  uintptr_t ssprireg = TIVA_ADC_SSPRI(adc);  /* SSE priority */
  uintptr_t emuxreg  = TIVA_ADC_EMUX(adc);   /* SSE trigger */

  /* step cfg */

  uintptr_t ssmuxreg  = (TIVA_ADC_BASE(adc) + TIVA_ADC_SSMUX(sse));  /* step registration */
#ifdef CONFIG_ARCH_CHIP_TM4C129
  uintptr_t ssemuxreg = (TIVA_ADC_BASE(adc) + TIVA_ADC_SSEMUX(sse)); /* extended mux registration */
#endif
  uintptr_t ssopreg   = (TIVA_ADC_BASE(adc) + TIVA_ADC_SSOP(sse));   /* differential status */
#ifdef CONFIG_EXPERIMENTAL
  uintptr_t sstshreg  = (TIVA_ADC_BASE(adc) + TIVA_ADC_SSTSH(sse));  /* sample and hold time */
#endif
  uintptr_t ssctlreg  = (TIVA_ADC_BASE(adc) + TIVA_ADC_SSCTL(sse));  /* step configuration */

  /* Get register contents */

  uint32_t cc = getreg32(ccreg);
  uint32_t pc = getreg32(pcreg);

  /* SSE cfg */

  uint32_t actss = getreg32(actssreg);
  uint32_t sspri = getreg32(ssprireg);
  uint32_t emux  = getreg32(emuxreg);

  /* step cfg */

  uint32_t ssmux = getreg32(ssmuxreg);
#ifdef CONFIG_ARCH_CHIP_TM4C129
  uint32_t ssemux = getreg32(ssemuxreg);
#endif
  uint32_t ssop   = getreg32(ssopreg);
#ifdef CONFIG_EXPERIMENTAL
  uint32_t sstsh  = getreg32(sstshreg);
#endif
  uint32_t ssctl  = getreg32(ssctlreg);

  /* Dump register contents */

  avdbg("CC     [0x%08x]=0x%08x\n", ccreg, cc);
  avdbg("PC     [0x%08x]=0x%08x\n", pcreg, pc);
  avdbg("ACTSS  [0x%08x]=0x%08x\n", actssreg, actss);
  avdbg("SSPRI  [0x%08x]=0x%08x\n", ssprireg, sspri);
  avdbg("EMUX   [0x%08x]=0x%08x\n", emuxreg, emux);
  avdbg("SSMUX  [0x%08x]=0x%08x\n", ssmuxreg, ssmux);
#ifdef CONFIG_ARCH_CHIP_TM4C129
  avdbg("SSEMUX [0x%08x]=0x%08x\n", ssemuxreg, ssemux);
#endif
  avdbg("SSOP   [0x%08x]=0x%08x\n", ssopreg, ssop);
#ifdef CONFIG_EXPERIMENTAL
  avdbg("SSTSH  [0x%08x]=0x%08x\n", sstshreg, sstsh);
#endif
  avdbg("SSCTL  [0x%08x]=0x%08x\n", ssctlreg, ssctl);

}
#endif /* CONFIG_DEBUG_ANALOG */
#endif /* CONFIG_TIVA_ADC0 || CONFIG_TIVA_ADC1 */
