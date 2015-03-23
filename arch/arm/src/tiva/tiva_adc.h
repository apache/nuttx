/****************************************************************************
 * arch/arm/src/tiva/tiva_adc.h
 *
 *   Copyright (C) 2015 TRD2 Inc. All rights reserved.
 *   Author: Calvin Maguranis <calvin.maguranis@trd2inc.com>
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

#ifndef __ARCH_ARM_SRC_TIVA_TIVA_ADC_H
#define __ARCH_ARM_SRC_TIVA_TIVA_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/irq.h>

#include "chip.h"

#ifdef CONFIG_TIVA_ADC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_DEBUG
#  undef CONFIG_TIVA_ADC_REGDEBUG
#endif

/* ADC Configuration values */

#ifdef CONFIG_ARCH_CHIP_TM4C123
#  define TIVA_ADC_CLOCK_MAX (16000000)
#  define TIVA_ADC_CLOCK_MIN (16000000)
#elif CONFIG_ARCH_CHIP_TM4C129
#  define TIVA_ADC_CLOCK_MAX (32000000)
#  define TIVA_ADC_CLOCK_MIN (16000000)
#else
#  error TIVA_tiva_adc_clock: unsupported architecture
#endif

/* Allow the same function call to be used for sample rate */

#ifdef CONFIG_ARCH_CHIP_TM4C123
#  define TIVA_ADC_SAMPLE_RATE_SLOWEST  (ADC_PC_SR_125K)
#  define TIVA_ADC_SAMPLE_RATE_SLOW     (ADC_PC_SR_250K)
#  define TIVA_ADC_SAMPLE_RATE_FAST     (ADC_PC_SR_500K)
#  define TIVA_ADC_SAMPLE_RATE_FASTEST  (ADC_PC_SR_1M)
#elif CONFIG_ARCH_CHIP_TM4C129
#  define TIVA_ADC_SAMPLE_RATE_SLOWEST  (ADC_PC_MCR_1_8)
#  define TIVA_ADC_SAMPLE_RATE_SLOW     (ADC_PC_MCR_1_4)
#  define TIVA_ADC_SAMPLE_RATE_FAST     (ADC_PC_MCR_1_2)
#  define TIVA_ADC_SAMPLE_RATE_FASTEST  (ADC_PC_MCR_FULL)
#else
#  error TIVA_ADC_SAMPLE_RATE: unsupported architecture
#endif

/* Sample Sequencer triggers */

#define TIVA_ADC_TRIG_SW       (ADC_EMUX_PROC)     /* Processor (default) */
#define TIVA_ADC_TRIG_EXTERNAL (ADC_EMUX_EXTERNAL) /* External (GPIO Pins) */
#define TIVA_ADC_TRIG_TIMER    (ADC_EMUX_TIMER)    /* Timer */
#define TIVA_ADC_TRIG_PWM0     (ADC_EMUX_PWM0)     /* PWM generator 0 */
#define TIVA_ADC_TRIG_PWM1     (ADC_EMUX_PWM1)     /* PWM generator 1 */
#define TIVA_ADC_TRIG_PWM2     (ADC_EMUX_PWM2)     /* PWM generator 2 */
#define TIVA_ADC_TRIG_PWM3     (ADC_EMUX_PWM3)     /* PWM generator 3 */
#define TIVA_ADC_TRIG_NEVER    (ADC_EMUX_NEVER)    /* Never Trigger */
#define TIVA_ADC_TRIG_ALWAYS   (ADC_EMUX_ALWAYS)   /* Always (continuously sample) */

/* Step configuration */

#define TIVA_ADC_FLAG_TS       (ADC_SSCTL_TS)      /* Sample Temp Sensor Select */
#define TIVA_ADC_FLAG_IE       (ADC_SSCTL_IE)      /* Sample Interrupt Enable */
#define TIVA_ADC_FLAG_END      (ADC_SSCTL_END)     /* Sample is End of Sequence */

#define TIVA_ADC_PWM_TRIG_IOCTL _ANIOC(0x00F0)

/* PWM trigger ioctl support  ***********************************************/

#define TIVA_ADC_PWM_TRIG(sse, pwm, mod) \
    ((((mod) << 4) << ((pwm)  * 8)) + (sse))

/* Misc utility *************************************************************/

#define ADC_PER_CHIP 2
#define SSE_PER_ADC  4
#define SSE_IDX(a,s) (((a)*SSE_PER_ADC) + (s))

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Step configuration options */

struct tiva_adc_step_cfg_s
{
    uint8_t  adc;   /* Parent peripheral */
    uint8_t  sse;   /* Parent sample sequencer (SSE) */
    uint8_t  step;  /* Which step in the sequencer */
    uint8_t  shold; /* Sample and hold time */
    uint8_t  flags; /* Last step? Interrupt enabled?
                     * Internal temperature sensor? */
    uint8_t  ain;   /* Which analog input */
};

/* Sample Sequencer configuration options */

struct tiva_adc_sse_cfg_s
{
    uint8_t priority; /* Conversion priority, 0-3 no duplicates */
    uint8_t trigger;  /* Trigger source */
};

/* ADC peripheral configuration options */

struct tiva_adc_cfg_s
{
    uint8_t                    adc;       /* ADC peripheral number */
    bool                       sse[4];    /* active SSEs in a bitmask */
    struct tiva_adc_sse_cfg_s  ssecfg[4]; /* SSE configuration */
    uint8_t                    steps;     /* Size of the stepcfg array */
    struct tiva_adc_step_cfg_s *stepcfg;  /* Step configuration array */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Only allow access to upper level ADC drivers if they are enabled */
#ifdef CONFIG_ADC

/****************************************************************************
 * Driver Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: tiva_adc_initialize
 *
 * Description:
 *   Configuration and bind the ADC to the ADC lower half instance and
 *   register the ADC driver at 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the ADC device.  This should be of the
 *     form /dev/adc0
 *   cfg - ADC configuration structure, configures the whole ADC.
 *   clock - clock speed for all ADC's. This is only set once for the first
 *     call to tiva_adc_initialize, otherwise the values are ignored.
 *   sample_rate - maximum sample rate of any ADC. This is only set once
 *     for the first call to tiva_adc_initialize, otherwise the values are
 *     ignored.
 *
 ****************************************************************************/

int tiva_adc_initialize(const char *devpath, struct tiva_adc_cfg_s *cfg,
                        uint32_t clock, uint8_t sample_rate);

/****************************************************************************
 * Interfaces exported from the ADC driver
 ****************************************************************************/

struct tiva_adc_s;

/****************************************************************************
 * Name: tiva_adc_lock
 *
 * Description:
 *   Get exclusive access to the ADC interface
 *
 ****************************************************************************/

void tiva_adc_lock(FAR struct tiva_adc_s *priv, int sse);

/****************************************************************************
 * Name: tiva_adc_unlock
 *
 * Description:
 *   Relinquish the lock on the ADC interface
 *
 ****************************************************************************/

void tiva_adc_unlock(FAR struct tiva_adc_s *priv, int sse);

#endif /* CONFIG_ADC */

/****************************************************************************
 * Library Function Prototypes
 ****************************************************************************/

/* Initialization routines **************************************************/

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

void tiva_adc_one_time_init(uint32_t clock, uint8_t sample_rate);

/****************************************************************************
 * Name: tiva_adc_configure
 *
 * Description:
 *   Performs configuration of a single ADC, it's valid sample sequencers and
 *   available steps.
 *
 ****************************************************************************/

void tiva_adc_configure(struct tiva_adc_cfg_s *cfg);

/****************************************************************************
 * Name: tiva_adc_sse_cfg
 *
 * Description:
 *   Configure the sample sequencer.
 *
 ****************************************************************************/

void tiva_adc_sse_cfg(uint8_t adc, uint8_t sse,
                      struct tiva_adc_sse_cfg_s *ssecfg);

/****************************************************************************
 * Name: tiva_adc_step_cfg
 *
 * Description:
 *   Configures a sample sequencer step.
 *
 ****************************************************************************/

void tiva_adc_step_cfg(struct tiva_adc_step_cfg_s *stepcfg);

/****************************************************************************
 * Name: tiva_adc_get_trigger
 *
 * Description:
 *   For a given adc, sse and step, get the AIN (channel) associated.
 *
 ****************************************************************************/

uint8_t tiva_adc_get_trigger(uint8_t adc, uint8_t sse);

/****************************************************************************
 * Name: tiva_adc_get_ain
 *
 * Description:
 *   For a given adc, sse and step, get the AIN (channel) associated.
 *
 ****************************************************************************/

uint8_t tiva_adc_get_ain(uint8_t adc, uint8_t sse, uint8_t step);

/* IRQS *********************************************************************/

/****************************************************************************
 * Name: tiva_adc_irq_attach
 *
 * Description:
 *  Attach a custom interrupt handler.
 *
 ****************************************************************************/

void tiva_adc_irq_attach(uint8_t adc, uint8_t sse, xcpt_t isr);

/****************************************************************************
 * Name: tiva_adc_irq_detach
 *
 * Description:
 *  detach an interrupt handler.
 *
 ****************************************************************************/

void tiva_adc_irq_detach(uint8_t adc, uint8_t sse);

/****************************************************************************
 * Name: tiva_adc_getirq
 *
 * Description:
 *  Maps ADC and SSE value to the correct IRQ value.
 *
 ****************************************************************************/

int tiva_adc_getirq(uint8_t adc, uint8_t sse);

/* Common peripheral level **************************************************/

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

int tiva_adc_enable(uint8_t adc, bool state);

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

void tiva_adc_clock(uint32_t freq);

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
void tiva_adc_vref(uint8_t vref);
#endif

/* Peripheral (base) level **************************************************/

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

void tiva_adc_sample_rate(uint8_t rate);

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

void tiva_adc_proc_trig(uint8_t adc, uint8_t sse_mask);

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

uint32_t tiva_adc_int_status(uint8_t adc);

/* Sample Sequencer (SSE) level *********************************************/

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

uint8_t tiva_adc_sse_enable(uint8_t adc, uint8_t sse, bool state);

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

void tiva_adc_sse_trigger(uint8_t adc, uint8_t sse, uint32_t trigger);

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
void tiva_adc_sse_pwm_trig(uint8_t adc, uint8_t sse, uint32_t cfg);
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

void tiva_adc_sse_int_enable(uint8_t adc, uint8_t sse, bool state);

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

bool tiva_adc_sse_int_status(uint8_t adc, uint8_t sse);

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

void tiva_adc_sse_clear_int(uint8_t adc, uint8_t sse);

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

uint8_t tiva_adc_sse_data(uint8_t adc, uint8_t sse, int32_t *buf);

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

void tiva_adc_sse_priority(uint8_t adc, uint8_t sse, uint8_t priority);

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

void tiva_adc_sse_register_chn(uint8_t adc, uint8_t sse, uint8_t chn, uint32_t ain);

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

void tiva_adc_sse_differential(uint8_t adc, uint8_t sse, uint8_t chn, uint32_t diff);

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
void tiva_adc_sse_sample_hold_time(uint8_t adc, uint8_t sse, uint8_t chn, uint32_t shold);
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

void tiva_adc_sse_step_cfg(uint8_t adc, uint8_t sse, uint8_t chn, uint8_t cfg);

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
void tiva_adc_dump_reg_cfg(uint8_t adc, uint8_t sse);
#endif

#  undef EXTERN
#  ifdef __cplusplus
}
#  endif
#endif /* __ASSEMBLY__ */
#endif /* CONFIG_TIVA_ADC */
#endif /* __ARCH_ARM_SRC_TIVA_TIVA_ADC_H */
