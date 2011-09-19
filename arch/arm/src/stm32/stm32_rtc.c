/************************************************************************************
 * arch/arm/src/stm32/stm32_rtc.c
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Author: Uros Platise <uros.platise@isotel.eu>
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
 ************************************************************************************/

/** \file
 *  \author Uros Platise
 *  \brief STM32 Real-Time Clock
 *  
 * \addtogroup STM32_RTC
 * \{
 * 
 * The STM32 RTC Driver offers standard precision of 1 Hz or High Resolution
 * operating at rate up to 16384 Hz. It provides UTC time and alarm interface
 * with external output pin (for wake-up).
 * 
 * RTC is based on hardware RTC module which is located in a separate power 
 * domain. The 32-bit counter is extended by 16-bit registers in BKP domain 
 * STM32_BKP_DR1 to provide system equiv. function to the: time_t time(time_t *).
 * 
 * Notation: 
 *  - clock refers to 32-bit hardware counter
 *  - time is a combination of clock and upper bits stored in backuped domain
 *    with unit of 1 [s]
 * 
 * \todo Error Handling in case LSE fails during start-up or during operation.
 */

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/rtc.h>
#include <arch/board/board.h>

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "up_arch.h"

#include "stm32_pwr.h"
#include "stm32_rcc.h"
#include "stm32_rtc.h"
#include "stm32_waste.h"


#if defined(CONFIG_STM32_BKP)

/************************************************************************************
 * Configuration of the RTC Backup Register (16-bit)
 ************************************************************************************/

#define RTC_TIMEMSB_REG      STM32_BKP_DR1

/************************************************************************************
 * Private Data
 ************************************************************************************/

/** Variable determines the state of the LSE oscilator. 
 *  Possible errors:
 *   - on start-up
 *   - during operation, reported by LSE interrupt
 */

volatile bool g_rtc_enabled = false;

/************************************************************************************
 * Private Functions
 ************************************************************************************/

static inline void stm32_rtc_beginwr(void)
{
    /* Previous write is done? */
    while( (getreg16(STM32_RTC_CRL) & RTC_CRL_RTOFF)==0 ) up_waste();

    /* Enter Config mode, Set Value and Exit */
    modifyreg16(STM32_RTC_CRL, 0, RTC_CRL_CNF);
}

static inline void stm32_rtc_endwr(void)
{
    modifyreg16(STM32_RTC_CRL, RTC_CRL_CNF, 0);
}

/** Wait for registerred to synchronise with RTC module, call after power-up only */
static inline void stm32_rtc_wait4rsf(void)
{
    modifyreg16(STM32_RTC_CRL, RTC_CRL_RSF, 0);
    while( !(getreg16(STM32_RTC_CRL) & RTC_CRL_RSF) ) up_waste();
}

/************************************************************************************
 * Interrupt Service Routines
 ************************************************************************************/

static int stm32_rtc_overflow_isr(int irq, void *context)
{
    uint16_t source = getreg16( STM32_RTC_CRL );
    
    if (source & RTC_CRL_OWF) {
        putreg16( getreg16(RTC_TIMEMSB_REG) + 1, RTC_TIMEMSB_REG );
    }
    
    if (source & RTC_CRL_ALRF) {
        /* Alarm */
    }
    
    /* Clear pending flags, leave RSF high */
    
    putreg16( RTC_CRL_RSF, STM32_RTC_CRL );    
    return 0;
}

/************************************************************************************
 * Public Function - Initialization
 ************************************************************************************/

/** Power-up RTC 
 * 
 * \param prescaler A 20-bit value determines the time base, and is defined as: 
 *      f = 32768 / (prescaler + 1)
 * 
 * \return State of the RTC unit
 * 
 * \retval OK If RTC has been successfully configured.
 * \retval ERROR On error, if LSE does not start.
 **/
int up_rtcinitialize(void)
{
    /* For this initial version we use predefined value */
    
    uint32_t prescaler = STM32_RTC_PRESCALER_MIN;
    
    /* Set access to the peripheral, enable power and LSE */
    
    stm32_pwr_enablebkp();
    stm32_rcc_enablelse();
    
    // \todo Get state from this function, if everything is 
    //   okay and whether it is already enabled (if it was disabled
    //   reset upper time register)
    g_rtc_enabled = true;

    // \todo Possible stall? should we set the timeout period? and return with -1
    stm32_rtc_wait4rsf();
        
    /* Configure prescaler, note that these are write-only registers */
    
    stm32_rtc_beginwr();
    putreg16(prescaler >> 16,    STM32_RTC_PRLH);
    putreg16(prescaler & 0xFFFF, STM32_RTC_PRLL);
    stm32_rtc_endwr();
    
    /* Configure Overflow Interrupt */
    
    irq_attach(STM32_IRQ_RTC, stm32_rtc_overflow_isr);
    up_enable_irq(STM32_IRQ_RTC);

    /* Previous write is done? This is required prior writing into CRH */
    
    while( (getreg16(STM32_RTC_CRL) & RTC_CRL_RTOFF)==0 ) up_waste();
    
    modifyreg16(STM32_RTC_CRH, 0, RTC_CRH_OWIE);
    
    /* Alarm Int via EXTI Line */
    
    // STM32_IRQ_RTCALR /* 41: RTC alarm through EXTI line interrupt */

    return OK;
}

/** Get time (counter) value 
 * 
 * \return time, where the unit depends on the prescaler value
 **/
 
clock_t up_rtc_getclock(void)
{
  irqstate_t flags;
  uint16_t cnth;
  uint16_t cntl;
  uint16_t tmp;

  /* The RTC counter is read from two 16-bit registers to form one 32-bit
   * value.  Because these are non-atomic operations, many things can happen
   * between the two reads:  This thread could get suspended or interrrupted
   * or the lower 16-bit counter could rollover between reads.  Disabling
   * interrupts will prevent suspensions and interruptions:
   */

  flags = irqsave();

  /* And the following loop will handle any clock rollover events that may
   * happen between samples.  Most of the time (like 99.9%), the following
   * loop will execute only once.  In the rare rollover case, it should
   * execute no more than 2 times.
   */

  do
    {
      tmp  = getreg16(STM32_RTC_CNTL);
      cnth = getreg16(STM32_RTC_CNTH);
      cntl = getreg16(STM32_RTC_CNTL);
    }

  /* The second sample of CNTL could be less than the first sample of CNTL
   * only if rollover occurred.  In that case, CNTH may or may not be out
   * of sync.  The best thing to do is try again until we know that no
   * rollover occurred.
   */

  while (cntl < tmp);
  irqrestore(flags);

  /* Then return the full 32-bit counter value */

  return ((uint32_t)cnth << 16) | (uint32_t)cntl;
}

/** Set time (counter) value
 * 
 * \param time The unit depends on the prescaler value 
 **/

void up_rtc_setclock(clock_t newclock)
{
    stm32_rtc_beginwr();
    putreg16(newclock >> 16,    STM32_RTC_CNTH);
    putreg16(newclock & 0xFFFF, STM32_RTC_CNTL);
    stm32_rtc_endwr();
}

time_t up_rtc_gettime(void)
{
    /* Fetch time from LSB (hardware counter) and MSB (backup domain)
     * Take care on overflow of the LSB:
     *   - it may overflow just after reading the up_rtc_getclock, transition
     *     from 0xFF...FF -> 0x000000
     *   - ISR would be generated to increment the RTC_TIMEMSB_REG
     *   - Wrong result would when: DR+1 and LSB is old, resulting in ~DR+2 
     *     instead of just DR+1
     */

    irqstate_t irqs = irqsave();
    
    uint32_t time_lsb = up_rtc_getclock();
    uint32_t time_msb = getreg16(RTC_TIMEMSB_REG);
    
    irqrestore( irqs );
    
    /* Use the upper bits of the LSB and lower bits of the MSB 
     * structured as:
     *   time = time[31:18] from MSB[13:0] | time[17:0] from time_lsb[31:14]
     */
    
    time_lsb >>= RTC_CLOCKS_SHIFT;
    
    time_msb <<= (32-RTC_CLOCKS_SHIFT);
    time_msb &= ~((1<<(32-RTC_CLOCKS_SHIFT))-1);
    
    return time_msb | time_lsb;
}

void up_rtc_settime(time_t newtime)
{
    /* Do reverse compared to gettime above */
    
    uint32_t time_lsb = newtime << RTC_CLOCKS_SHIFT | 
        (up_rtc_getclock() & ((1<<RTC_CLOCKS_SHIFT)-1));
        
    uint32_t time_msb = newtime >> (32-RTC_CLOCKS_SHIFT);
    
    irqstate_t irqs = irqsave();
    
    up_rtc_setclock(time_lsb);
    putreg16( time_msb, RTC_TIMEMSB_REG );
    
    irqrestore( irqs );
}

/** Set ALARM at which time ALARM callback is going to be generated
 * 
 * The function sets the alarm and return present time at the time
 * of setting the alarm. 
 * 
 * Note that If actual time has already passed callback will not be
 * generated and it is up to the higher level code to  compare the 
 * returned (actual) time and desired time of alarm.
 * 
 * \param attime The unit depends on the prescaler value 
 * \return presenttime, where the unit depends on the prescaler value
 **/
clock_t up_rtc_setalarm(clock_t atclock)
{
    stm32_rtc_beginwr();
    putreg16(atclock >> 16,    STM32_RTC_ALRH);
    putreg16(atclock & 0xFFFF, STM32_RTC_ALRL);
    stm32_rtc_endwr();
    
    return up_rtc_getclock();
}

/** Set alarm output pin */
void stm32_rtc_settalarmpin(bool activate)
{
}

#endif // defined(CONFIG_STM32_BKP)
/** \} */
