/****************************************************************************
 * arch/avr/src/avrdx/avrdx_delay.c
 * Custom implementation of up_udelay
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <nuttx/arch.h>
#include <sys/types.h>

#include "avrdx.h"
#include "avrdx_delay_loop.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Division takes some 350 clock cycles and the delay loop needs 6 cycles
 * per loop. Each division therefore takes as much time as this number
 * of loops.
 */

#define DIVISION_LOOP_INCREMENT 55

/* Multiplication is not that difficult */

#define MULTIPLICATION_LOOP_INCREMENT 5

/* Detect if divisions are optimized for size - that means the program
 * will actually call the division function even when dividing by constant
 * (which can be done without the actual division.)
 */

#if (defined(CONFIG_ARCH_TOOLCHAIN_GCC) && defined(__OPTIMIZE_SIZE__))
#  define DIVISION_OPTIMIZED_FOR_SIZE
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

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
 * Name: up_udelay
 *
 * Description:
 *   Delay execution for requested number of microseconds.
 *
 *   This is a replacement for core implementation (which is defined
 *   as weak) specific for AVR DA/DB (AVRxt) cores. The main difference
 *   is the means used to calculate the number of cycles. The core function
 *   uses CONFIG_BOARD_LOOPSPERMSEC, a value configured by either the user,
 *   or the author of board code.
 *
 *   AVRxt chips can run with various clock sources and those can be further
 *   configured and prescaled during runtime. As such, there is no single
 *   universal value of CONFIG_BOARD_LOOPSPERMSEC (unless the board uses
 *   external clock source.)
 *
 *   This implementation therefore attempts to determine current running
 *   frequency of the MCU and calculate needed loop count from that.
 *
 *   Note - same as core implementation of up_udelay, the function waits
 *   in a busy-wait loop. As such:
 *
 *   * It is *** NOT multi-tasking friendly ***
 *   * If interrupted (by interrupt handler or preempted by another task),
 *     the total wait time is increased by the duration of the outside code.
 *
 * Input Parameters:
 *   microseconds - requested wait time
 *
 * Returned Value: none
 *
 * Assumptions/Limitations:
 *   Conversion from microseconds is done in run-time and can take
 *   a lot of cycles itself.
 *
 ****************************************************************************/

void up_udelay(useconds_t microseconds)
{
  /* Determined frequency of the CPU. 16bit variable is used
   * to force math with smaller data types.
   */

  uint32_t f_cpu;
  uint16_t f_cpu_shifted;

  /* Variable for determined loop count. The counter accumulates
   * number of loops that were already "done" by computations
   * in this function.
   */

  uint32_t loops;
  uint16_t loop_like_counter = 0;

  /* This is added to loop_like_counter whenever this function
   * does something that divides clock frequency by main prescaler
   * if the main prescaler is enabled.
   */

  uint8_t main_clock_prescaler = 0;

  /* Other helper variables. */

#ifndef DIVISION_OPTIMIZED_FOR_SIZE
  uint8_t microseconds_u8;
#endif
  uint8_t mclkctrla;

  if (microseconds == 0)
    {
      return;
    }

  if (CLKCTRL.MCLKCTRLB & CLKCTRL_PEN_bm)
    {
      /* Main prescaler is enabled. Frequency is divided
       * by prescaling ratio, loop count calculation is slowed
       * by that.
       */

      main_clock_prescaler = DIVISION_LOOP_INCREMENT;
    }

  /* We need to convert duration in microseconds to loop count.
   *
   * - one clock cycle takes 1/f seconds (where f is CPU frequency.)
   * - one loop of avrdx_delay_loop takes 6 clock cycles. Duration
   *   of one loop pass is therefore 6/f seconds.
   * - we receive microseconds as a parameter, duration of one loop
   *   pass is 6e6/f microseconds
   * - total number of required loops is therefore:
   *   usec/(6e6/f) == (f * usec) / 6e6
   */

  f_cpu = avrdx_current_freq_cpu();

  /* Increment the counter if the main clock prescaler is enabled,
   * ie. if the frequency obtained from the hardware got divided.
   */

  loop_like_counter += main_clock_prescaler;

  mclkctrla = CLKCTRL.MCLKCTRLA & CLKCTRL_CLKSEL_GM;
  if (mclkctrla == CLKCTRL_CLKSEL_OSCHF_GC)
    {
      /* All of this works with 32 bit operands and the calculation
       * will involve some divisions and those are expensive. We can
       * attempt to alleviate that by forcing 16 bit operation instead.
       * That is possible but some conditions need to be met:
       *
       *   1. f_cpu is more than 1MHz (that's minimum for high frequency
       *      oscillator but it can be prescaled
       *   2. microseconds is less than 180
       *   3. compiler is not optimizing for size
       *
       * The first condition allows us to bitshift the frequency by 16
       * without losing too much accuracy.
       *
       * The second condition makes sure that the result fits into 16 bit
       * value even with maximum f_cpu (24MHz)
       *
       * The third condition accounts for the fact that the compiler
       * is able to avoid division when dividing by a compile-time
       * constant but will not do it if optimizing for size.
       */

#ifndef DIVISION_OPTIMIZED_FOR_SIZE
      if ((microseconds < 180) && (f_cpu >= 1000000))
        {
          /* Condition above allows us to bitshift the frequency 16 bits
           * to the right, we will bitshift the denominator as well
           * to compensate
           */

          f_cpu_shifted = f_cpu >> 16;
          microseconds_u8 = microseconds;
          loops = microseconds_u8 * f_cpu_shifted / (6000000 >> 16);
        }
      else
#endif
        {
          /* Cannot do 16 bit math above for some reason. We still need
           * to bitshift the inputs to the multiplication though,
           * otherwise any delay (value of microseconds) greater than 178
           * overflows even 32 bit math.
           */

          if (f_cpu >= 1000000)
            {
              f_cpu_shifted = f_cpu >> 16;
            }
          else
            {
              /* This is safe to do - f_cpu is at least 41666, which
               * is 1MHz / 24 from prescaler.
               */

              f_cpu_shifted = f_cpu >> 8;

              /* Still need to shift something by 8 to the right. Can't do
               * microseconds blindly though - the compiler may know
               * (from the test above) that the value is less than 180
               * and use that knowledge here to determine that bitshifted
               * value is 0. Calculated value of loops will be 0 too and
               * the function terminates because value of 0 will always
               * be less than loop_like_counter which is non-zero.
               *
               * However - since we know frequency is less than 1MHz,
               * we also know that the prescaler is applied. Reading f_cpu
               * therefore did a division for 350 clock cycles. Highest
               * value of f_cpu can therefore be 833333Hz (20MHz oscillator
               * divided by 24 prescaler.) That is 1.2us per cycle.
               *
               * The division therefore took 420us. If the requested delay
               * value is less than 256us, we already achieved that.
               *
               * So in the end - we CAN do the bitshift blindly.
               */

              microseconds = microseconds >> 8;
            }

          /* This compiles into 2x16bit to 32bit multiplication, followed
           * by division. Some 350 cycles for the division,
           * 40 for multiplication, this function takes some, let's say
           * 420 clock cycles total. This means that this function already
           * did 70 loops by itself, without entering the actual loop.
           */

          loops = microseconds * f_cpu_shifted / (6000000 >> 16);
          loop_like_counter += (DIVISION_LOOP_INCREMENT + \
                                MULTIPLICATION_LOOP_INCREMENT);
        }
    }

#ifndef CONFIG_AVRDX_ARCH_UDELAY_NO_OSC32K
  else if ((mclkctrla == CLKCTRL_CLKSEL_OSC32K_GC) || \
           (mclkctrla == CLKCTRL_CLKSEL_XOSC32K_GC))
    {
      /* Unlike with the high frequency oscillator, we can't bitshift
       * the CPU frequency by 16 bits, that would erase it to zero.
       * We can only do bitshift by 8 bits (and we have to because
       * otherwise the multiplication between f_cpu and microseconds
       * overflows for 2^17 (131072) microseconds or more.
       *
       * Also cannot assume it's actually 32.768k - main clock prescaler
       * may be in effect. We will check for that one though, that could
       * save us a lot of time.
       */

      if (main_clock_prescaler)
        {
          /* Main prescaler in use, we are out of luck and need
           * to do the computation. Considering that the CPU clock
           * is slow and we need to divide (and that obtaining
           * f_cpu also did a division), the accuracy of this
           * function will most likely be way off.)
           */

          f_cpu_shifted = f_cpu >> 8;
          loops = f_cpu_shifted * microseconds / (6000000 >> 8);
          loop_like_counter += DIVISION_LOOP_INCREMENT;
        }
      else
        {
          /* Main prescaler not in use, frequency is a known constant.
           *
           */

          loops = (32768 >> 8) * microseconds / (6000000 >> 8);

          /* Does no division unless optimizing for size */

#  ifdef DIVISION_OPTIMIZED_FOR_SIZE
          loop_like_counter += DIVISION_LOOP_INCREMENT;
#  endif
        }
    }
#endif

#ifndef CONFIG_AVRDX_ARCH_UDELAY_NO_EXTCLK
  else if (mclkctrla == CLKCTRL_CLKSEL_EXTCLK_GC)
    {
      loops = (USEC_PER_MSEC * microseconds) * CONFIG_BOARD_LOOPSPERMSEC;
      avrdx_current_freq_main_prescaler(loops);
      loop_like_counter += main_clock_prescaler;
    }
#endif

  else
    {
      /* Unsupported clock, wait as long as possible */

      loops = UINT32_MAX;
    }

  if (loops < loop_like_counter)
    {
      /* This function already took more time that it was supposed to.
       * Note - must not pass 0 to avrdx_delay_loop.
       */

      return;
    }

  avrdx_delay_loop(loops - loop_like_counter);
}
