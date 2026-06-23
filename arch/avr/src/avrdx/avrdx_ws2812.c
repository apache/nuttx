/****************************************************************************
 * arch/avr/src/avrdx/avrdx_ws2812.c
 * Driver for programmable RGB LED WS12812
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
 * You can read on design philosophy of this driver in the Documentation
 * directory (platforms/avr/avrdx/docs/ws2812)
 *
 * The gist is that the implementation strives to be as fast as possible
 * because with 800kHz LED bit frequency, the chip has 30 clock cycles
 * to process each bit.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <nuttx/leds/ws2812.h>
#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <assert.h>
#include <stdlib.h>

#include "avrdx_ws2812.h"
#include "avrdx_serial.h"
#include "avrdx_swint.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This code assumes that WS2812_RW_PIXEL_SIZE == 4 */

static_assert(WS2812_RW_PIXEL_SIZE == 4,
              "This code assumes that WS2812_RW_PIXEL_SIZE == 4");

#if (defined CONFIG_WS2812_HAS_WHITE) && \
    (defined CONFIG_AVR_AVRDX_WS2812_NO_WHITE_LED)
#  undef CONFIG_WS2812_HAS_WHITE
#endif

#ifdef CONFIG_WS2812_HAS_WHITE

/* Author of this code does not have any such LED on hand and did not
 * succeed in finding a datasheet for one. Color component ordering
 * is unknown and was simply copied from driver for another chip.
 * (arch/risc-v/src/rp23xx-rv/rp23xx_ws2812.c)
 *
 * Since the code cannot be tested, it is DISABLED. Calling register
 * method avrdx_ws2812_register with has_white set will cause
 * it to return ENOTSUP. To save program memory space, disable
 * WS2812_HAS_WHITE in Kconfig if you can.
 *
 * In case someone with such a LED tests this code, please submit
 * a pull request that removes this warning and re-enables has_white
 * in avrdx_ws2812_register.
 */

#  warning CONFIG_WS2812_HAS_WHITE is untested and is disabled.\
 (see comment in source)
#endif

/* Note - position of data in various arrays is static for each lane;
 * for unused lanes, the position is left empty. Size is therefore
 * determined solely from highest used number.)
 *
 * For future edits: the ordering of existing items MUST be preserved,
 * code all around uses these values as offsets for various calculations.
 */

#if defined(CONFIG_AVR_AVRDX_WS2812_USART5)
#  define AVRDX_WS2812_LANE_SIZE 6
#elif defined(CONFIG_AVR_AVRDX_WS2812_USART4)
#  define AVRDX_WS2812_LANE_SIZE 5
#elif defined(CONFIG_AVR_AVRDX_WS2812_USART3)
#  define AVRDX_WS2812_LANE_SIZE 4
#elif defined(CONFIG_AVR_AVRDX_WS2812_USART2)
#  define AVRDX_WS2812_LANE_SIZE 3
#elif defined(CONFIG_AVR_AVRDX_WS2812_USART1)
#  define AVRDX_WS2812_LANE_SIZE 2
#elif defined(CONFIG_AVR_AVRDX_WS2812_USART0)
#  define AVRDX_WS2812_LANE_SIZE 1
#else
#  error There is no enabled lane
#endif

/* This is specific for UART lanes - arrays that are not relevant
 * for other lane types do not need to allocate space for them
 */

#if defined(CONFIG_AVR_AVRDX_WS2812_USART5)
#  define AVRDX_WS2812_LANE_ANY_USART
#  define AVRDX_WS2812_LANE_USART_SIZE 6
#elif defined(CONFIG_AVR_AVRDX_WS2812_USART4)
#  define AVRDX_WS2812_LANE_ANY_USART
#  define AVRDX_WS2812_LANE_USART_SIZE 5
#elif defined(CONFIG_AVR_AVRDX_WS2812_USART3)
#  define AVRDX_WS2812_LANE_ANY_USART
#  define AVRDX_WS2812_LANE_USART_SIZE 4
#elif defined(CONFIG_AVR_AVRDX_WS2812_USART2)
#  define AVRDX_WS2812_LANE_ANY_USART
#  define AVRDX_WS2812_LANE_USART_SIZE 3
#elif defined(CONFIG_AVR_AVRDX_WS2812_USART1)
#  define AVRDX_WS2812_LANE_ANY_USART
#  define AVRDX_WS2812_LANE_USART_SIZE 2
#elif defined(CONFIG_AVR_AVRDX_WS2812_USART0)
#  define AVRDX_WS2812_LANE_ANY_USART
#  define AVRDX_WS2812_LANE_USART_SIZE 1
#else
#  define AVRDX_WS2812_LANE_USART_SIZE 0
#endif

/* Avoid magic numbers in the code */

#ifdef CONFIG_WS2812_HAS_WHITE
#  define WS2812_BYTES_PER_LED_WITH_WHITE 12
#endif

#define WS2812_BYTES_PER_LED_NO_WHITE 8

#define WS2812_MAX_LED_COUNT 255

/* This is 5 * 65536 - allows the compiler to simplify some comparisons.
 * The value of T_reset specified in the datasheet is 280us. (Value
 * given in nanoseconds to be suitable for tv_nsec in struct timespec.)
 */

#define WS2812_T_RESET (327680)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Size of index needed to address all data for transmission.
 * Use smallest data type if the user asked for it.
 */

#ifdef CONFIG_AVR_AVRDX_WS2812_SHORT_LANE
typedef uint8_t avrdx_ws2812_idx_t;
#else
/* Needs to be able to index transmit data - which is not the same
 * as pixel buffer - for hardcoded limit of 255 LEDs
 */

typedef uint16_t avrdx_ws2812_idx_t;
#endif

/* This is stored in inode and we get it to recognize which character
 * device file we are working with at the moment. There should be more
 * information in the struct but we need to store most data in flat
 * arrays so they are accessible from assembler.
 */

struct avrdx_ws2812_s
{
  enum avrdx_ws2812_lane_e lane;   /* Lane - used hardware */
#ifndef CONFIG_AVR_AVRDX_WS2812_NO_READS
  /* There is a room for improvement - if CONFIG_WS2812_HAS_WHITE
   * is not set, we only need 24 bits to store the pixel (uint24_t
   * is supported by GCC.)
   *
   * Not implemented though - better memory savings can be achieved
   * by not storing the data altogether.
   */

  uint32_t *pixel_buffer;
#endif
};

#if (AVRDX_WS2812_LANE_SIZE <= 8)
typedef uint8_t avrdx_ws2812_bitmap_t;
#elif (AVRDX_WS2812_LANE_SIZE <= 16)
typedef uint16_t avrdx_ws2812_bitmap_t;
#elif (AVRDX_WS2812_LANE_SIZE <= 32)
typedef uint32_t avrdx_ws2812_bitmap_t;
#else
#  error Having more than 32 lines is not supported
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void trigger_transmission_hardware(void);

/* Technically not private since we load it from different file
 * but it is only supposed to be used by store_uart_bytes()
 * (Ie. it would be in this file if not written in assembler.)
 */

void avrdx_ws2812_store_uart_bytes_rgb(enum avrdx_ws2812_lane_e lane,
                                       uint32_t *data,
                                       uint8_t offset,
                                       uint8_t len);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Values here correspond to the Documentation file describing this driver
 * (platforms/avr/avrdx/docs/ws2812.rst) Declared in private data section
 * despite not being static - used by a function written in assembler.
 */

const uint8_t IOBJ avrdx_ws2812_uart_white[4] =
{
  0xf7, /* 0b11110111 */
  0xc7, /* 0b11000111 */
  0xf4, /* 0b11110100 */
  0xc4  /* 0b11000100 */
};

static avrdx_ws2812_bitmap_t avrdx_ws2812_dirty_lanes;
static avrdx_ws2812_bitmap_t avrdx_ws2812_active_lane;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Array of pointers to uint8_t. Each lane allocates appropriate amount
 * of bytes and stores the pointer here. Used by assembler code, needs
 * to be outside of struct.
 */

uint8_t *g_avrdx_ws2812_xmit_bytes[AVRDX_WS2812_LANE_SIZE];

/* Next byte to be sent. Hardware for a lane may need additional data,
 * defined below.
 */

avrdx_ws2812_idx_t g_avrdx_ws2812_xmit_next[AVRDX_WS2812_LANE_SIZE];

/* Total count of bytes to be sent */

avrdx_ws2812_idx_t g_avrdx_ws2812_xmit_count[AVRDX_WS2812_LANE_SIZE];

/* Software interrupt vectors to signal completion */

avrdx_swint_size_t g_avrdx_ws2812_swint_vector[AVRDX_WS2812_LANE_SIZE];

/* Array of pointers to struct timespec which records the last time
 * the lane data was transmitted. Used to ensure that the T_reset
 * period elapses between retransmissions.
 */

struct timespec *g_avrdx_ws2812_xmit_ts[AVRDX_WS2812_LANE_SIZE];

#ifdef AVRDX_WS2812_LANE_ANY_USART

/* UART lanes use 9 bit transmissions. 9th bits are stored here.
 * For each group of 8 bytes, one byte is allocated here and
 * 8x 9th bits are stored in it. LSB holds 9th bit for byte
 * with lowest index.
 */

uint8_t *g_avrdx_ws2812_uart9_bytes[AVRDX_WS2812_LANE_USART_SIZE];

/* This is a working copy from g_avrdx_ws2812_uart9_bytes. Fetched
 * in ISR every time xmit_next % 8 == 0, shifted to the right
 * after each UART byte is transmitted.
 *
 * The highest bit cannot be set in this value except between
 * the fetch and first shift. This means that it cannot be set
 * when we enter the transmit ISR (because it does the shift
 * after fetch.) We are therefore using this highest bit as a signal.
 * If set, the LEDs have white component and 9 bit transmission
 * is actually not in use. This check is skipped
 * if CONFIG_WS2812_HAS_WHITE is unset.
 */

uint8_t g_avrdx_ws2812_uart9_current[AVRDX_WS2812_LANE_USART_SIZE];

#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: transmission_completed
 *
 * Description:
 *   IRQ callback called by software interrupt triggered by lane transmit
 *   handler (for example UART Data Register Empty ISR) when it transmits
 *   all bytes to LEDs
 *
 * Input Parameters:
 *   irq, context - unused
 *   arg - lane number (given as pointer, type cast is needed)
 *
 * Returned Value: OK
 *
 * Assumptions/Limitations:
 *   Running in interrupt context.
 *
 ****************************************************************************/

static int transmission_completed(int irq, void *context, FAR void *arg)
{
  enum avrdx_ws2812_lane_e lane;

  /* Record the time - lane must not retransmit its data until
   * T_reset period in the LEDs elapses.
   */

  lane = (enum avrdx_ws2812_lane_e) arg;
  up_timer_gettime(g_avrdx_ws2812_xmit_ts[lane]);

  /* Trigger next transmission from next lane. Or not if no lane is dirty,
   * the function takes care of both cases.
   */

  trigger_transmission_hardware();

  return OK;
}

/****************************************************************************
 * Name: trigger_transmission_hardware
 *
 * Description:
 *   Start transmission. If currently transmitting lane is not set, start
 *   transmission from lowest dirty lane. (Dirty means lane that has new
 *   data not transmitted to the LEDs.)
 *
 *   If it is set, transmission of that (currently transmitting) lane just
 *   finished, find next dirty lane to transmit.
 *
 * Input Parameters: none
 *
 * Returned Value: void
 *
 * Assumptions/Limitations:
 *   - may be running in interrupt context
 *   - interrupts are disabled by the caller or by the (interrupt) context
 *   - only called if there is no active transmission or if it
 *     just finished
 *
 ****************************************************************************/

static void trigger_transmission_hardware(void)
{
  uint8_t lane_idx;
  avrdx_ws2812_bitmap_t lane_calc;

  if (avrdx_ws2812_active_lane == 0)
    {
      /* No active lane - find first dirty */

      if (avrdx_ws2812_dirty_lanes == 0)
        {
          /* This function is not supposed to be called
           * under these conditions.
           */

          return;
        }

      /* Lowest lane in bitmap. (This expression finds lowest
       * bit that is set in the value.)
       */

      avrdx_ws2812_active_lane = \
          avrdx_ws2812_dirty_lanes & ((~avrdx_ws2812_dirty_lanes) + 1);
    }
  else
    {
      /* Active lane set means that some transmission finished.
       * Move on to the next one.
       */

      if (avrdx_ws2812_dirty_lanes == 0)
        {
          /* There is no next one, everything is up-to-date */

          avrdx_ws2812_active_lane = 0;
          return;
        }

      while (1)
        {
          avrdx_ws2812_active_lane <<= 1;
          if (avrdx_ws2812_active_lane == 0)
            {
              avrdx_ws2812_active_lane = 1; /* wrap */
            }

          if (avrdx_ws2812_active_lane & avrdx_ws2812_dirty_lanes)
            {
              /* Found dirty lane, transmit that */

              break;
            }
        }
    }

  /* At this point the function either returned or the lane
   * to be transmitted next is set in avrdx_ws2812_active_lane.
   *
   * Mark the lane as not dirty now. This will allow write calls to submit
   * new data. But only after T_reset elapses, which the write must check.
   */

  avrdx_ws2812_dirty_lanes &= ~avrdx_ws2812_active_lane;

  /* Convert active lane - bitmap - to index. Bitshifting to the right
   * will eventually make the value 0 and that's the index we want.
   * (Bitshift in the initialization compensates for the fact that
   * value of 1 means lane_idx == 0 and the for loop must not run.)
   */

  lane_idx = 0;
  for (lane_calc = avrdx_ws2812_active_lane >> 1; lane_calc; lane_calc >>= 1)
    {
      lane_idx++;
    }

  /* Reset byte counter */

  g_avrdx_ws2812_xmit_next[lane_idx] = 0;

  /* Hardware-specific action. Currently only UART */

  if (lane_idx < AVRDX_WS2812_LANE_USART_SIZE)
    {
      AVRDX_USART(lane_idx).CTRLA |= USART_DREIE_bm;
    }
}

/****************************************************************************
 * Name: trigger_transmission
 *
 * Description:
 *   Trigger transmission on a lane. Intended to be called after a write
 *   (or other operation that changed pixel data.) Only flags the lane
 *   data as dirty, then calls common method (if needed.)
 *
 * Input Parameters:
 *   - lane - lane with dirty data to be written out
 *
 * Returned Value: void
 *
 * Assumptions/Limitations:
 *   - it is assumed that the caller holds mutex but the implementation also
 *     disables interrupts.
 *   - this function may sleep - interrupts must not be disabled
 *     by the caller
 *
 ****************************************************************************/

static void trigger_transmission(enum avrdx_ws2812_lane_e lane)
{
  irqstate_t irqstate;
  struct timespec ts;              /* Temporary value for calculations */
  avrdx_ws2812_bitmap_t lane_bit;  /* Cache that avoids recomputing
                                    * a bitshift */
  uint32_t u32;                    /* Computation variables */
  uint16_t u16;

  /* Pointer to a lane timestamp. The compiler will re-read the value
   * fetched from the g_avrdx_ws2812_xmit_ts array even if not declared
   * volatile but let's make sure it does it.
   */

  volatile struct timespec *lane_xmit_ts;

  /* Pointer to active lane value made volatile to ensure the compiler
   * will re-read it on each loop pass (it did that in testing even
   * without the volatile but again, let's make sure.)
   */

  volatile avrdx_ws2812_bitmap_t *active_lane;

  irqstate = up_irq_save();

  /* If this lane is currently being transmitted, it must not be marked
   * dirty again - that would cause immediate retransmission. Instead, wait
   * until it is not.
   */

  lane_bit = (1 << lane);
  active_lane = &avrdx_ws2812_active_lane;
  for (; (*active_lane) & lane_bit; )
    {
      /* Sleep for how long? 1.25us per bit means 30us per 24bits.
       * For a millisecond wait, we need 34 LEDs at least. (Ignoring LEDs
       * with white component.) Just use 2ms and let the loop handle the
       * rest.
       */

      nxsched_usleep(2);
    }

  /* Now we know the lane is currently not being transmitted. But we must
   * block the trigger if the transmission happened recently and T_reset
   * period is in progress
   */

  lane_xmit_ts = g_avrdx_ws2812_xmit_ts[lane];
  for (; ; )
    {
      up_timer_gettime(&ts);
      clock_timespec_subtract(&ts, lane_xmit_ts, &ts);

      /* Reset time is a microsecond value, if a second elapsed,
       * there is no wait.
       */

      if (ts.tv_sec > 0)
        {
          break;
        }

      /* tv_nsec should be positive, current time minus past,
       * tell this to the compiler.
       */

      u32 = (ts.tv_nsec < 0) ? 0 : ts.tv_nsec;

      /* Is this value higher than WS2812_T_RESET? Value of WS2812_T_RESET
       * is chosen to have 2 least significant bytes set to 0, simplify
       * the comparison using this knowledge.
       */

#if (WS2812_T_RESET & 0xffff)
#  error Value of WS2812_T_RESET does not meet expectations here
#endif

      u16 = u32 >> 16;
      if (u16 > (WS2812_T_RESET >> 16))
        {
          break;
        }

      /* Alright, we will be waiting. For how long?
       *
       * tv_nsec (u32) is a nanosecond value but we will call usleep
       * with a microsecond parameter. We can obtain the latter
       * from the former by dividing it by 1000. We do not like such
       * solution though because it does a division.
       *
       * So, we just bitshift u16 6 bits to the left (u16 is value
       * of tv_nsec bitshifted by 16 to the right.) That gives us
       * tv_nsec >> 10 and if we accept that a microsecond is 1024
       * nanoseconds long, we are done. Or at least close to done,
       * lower 6 bits of the resulting value are truncated and we need
       * to compensate for that. Add 1 << 6 - waiting for more time
       * is fine.
       *
       * Also - make sure that WS2812_T_RESET is set to a 24 bit value,
       * all of this depends on that. (The next check is even more
       * strict than that.)
       */

#if WS2812_T_RESET > 0x7ffff
#  error This code assumes WS2812_T_RESET to be 5 * 65536
#endif

      nxsched_usleep((u16 << 6) + (1 << 6));
    }

  avrdx_ws2812_dirty_lanes |= lane_bit;

  if (avrdx_ws2812_active_lane == 0)
    {
      /* No outgoing transmission currently, start it for this lane. */

      trigger_transmission_hardware();
    }

  up_irq_restore(irqstate);
}

/****************************************************************************
 * Name: store_uart_bytes
 *
 * Description:
 *   Receives data from write syscall and encodes them into bytes
 *   to be transmitted through the UART.
 *
 * Input Parameters:
 *   - lane - lane which is being written to
 *   - data - pointer to uint32_t array of RGB data
 *   - offset - first LED to change with data, given in items
 *   - len - data length in items
 *   - has_white - set if the lane has LEDs with white component,
 *     ignored if CONFIG_WS2812_HAS_WHITE is not set
 *
 * Returned Value: void
 *
 * Assumptions/Limitations:
 *   - len is truncated so offset+len does not exceed the data buffer
 *
 ****************************************************************************/

static void store_uart_bytes(enum avrdx_ws2812_lane_e lane,
                             uint32_t *data,
                             uint8_t offset,
                             uint8_t len,
                             bool has_white)
{
#ifdef CONFIG_WS2812_HAS_WHITE

  uint8_t *uart_bytes = g_avrdx_ws2812_xmit_bytes[lane];
  uint16_t temp_u16;
  uint8_t temp_u8;
  uint8_t i;

  /* UART lane with support for white component. */

  if (has_white)
    {
      /* Data is received as WWRRGGBB, the LED wants GGRRBBWW,
       * need to keep that in mind.
       */

      uart_bytes += WS2812_BYTES_PER_LED_WITH_WHITE * offset;

      for (; len; len--)
        {
          /* Little endian architecture, green is stored in byte +1,
           * LED wants it first. The compiler should pick the byte we
           * want without bitshifting.
           *
           * (From disassembly: no bitshifting but all four bytes
           * are loaded from memory. Can probably be improved
           * using some pointers and type casts.)
           */

#  ifdef CONFIG_ENDIAN_BIG
#    error AVR is not big endian
#  endif
          temp_u8 = ((*data) & 0xff00) >> 8;
          temp_u16 = temp_u8;
          for (i = 4; i; i--)
            {
              /* Except this bitshift with temp_u16, the compiler
               * recognizes that the rest can be done
               * without shifting. Saves CPU time.
               */

              temp_u16 <<= 2;
              *uart_bytes = \
                  avrdx_ws2812_uart_white[(temp_u16 & 0x0300) >> 8];
              uart_bytes++;
            }

          /* Red now. That one is at byte +2. The compiler only reads
           * one byte from the memory... but gets the address from base
           * value - 4 + 2 (sbiw, adiw)
           */

          temp_u8 = ((*data) & 0xff0000) >> 16;
          temp_u16 = temp_u8;
          for (i = 4; i; i--)
            {
              temp_u16 <<= 2;
              *uart_bytes = \
                  avrdx_ws2812_uart_white[(temp_u16 & 0x0300) >> 8];
              uart_bytes++;
            }

          /* Blue - byte 0 */

          temp_u8 = (*data) & 0xff;
          temp_u16 = temp_u8;
          for (i = 4; i; i--)
            {
              temp_u16 <<= 2;
              *uart_bytes = \
                  avrdx_ws2812_uart_white[(temp_u16 & 0x0300) >> 8];
              uart_bytes++;
            }

          /* And white at byte + 3. Compiler still does not use bitshifts
           * but it loads all bytes and then resets 3 of them
           * (in registers) to zero.
           */

          temp_u8 = ((*data) & 0xff000000) >> 24;
          temp_u16 = temp_u8;
          for (i = 4; i; i--)
            {
              temp_u16 <<= 2;
              *uart_bytes = \
                  avrdx_ws2812_uart_white[(temp_u16 & 0x0300) >> 8];
              uart_bytes++;
            }

          /* Done with this pixel, move onto the next one */

          data++;
        }
    }
  else
#endif /* ifdef CONFIG_WS2812_HAS_WHITE */
    {
      /* No support for LEDs with white component or this lane
       * does not have those. Conversion from RGB data to UART
       * bytes is going to take a lot of bitshift and similar,
       * we do this in assembly to save CPU time.
       *
       * Note - this method expects len to be non-zero.
       */

      if (len > 0)
        {
          avrdx_ws2812_store_uart_bytes_rgb(lane, data, offset, len);
        }
    }
}

/****************************************************************************
 * Name: avrdx_ws2812_open
 *
 * Description:
 *   Character device open handler. We do not track open count so it does
 *   nothing. Must exist though, upper half does not check it for NULL.
 *
 * Input Parameters:
 *   filep - struct file pointer
 *
 * Returned Value: OK
 *
 ****************************************************************************/

static int avrdx_ws2812_open(struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: avrdx_ws2812_close
 *
 * Description:
 *   Character device close handler. Upper half does check for NULL
 *   pointer here but we just return OK so let's have it.
 *
 * Input Parameters:
 *   filep - struct file pointer
 *
 * Returned Value:
 *   OK
 *
 ****************************************************************************/

static int avrdx_ws2812_close(struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: avrdx_ws2812_write
 *
 * Description:
 *   Writes RGB data to pixel buffer (unless configured not to do so)
 *   and to the transmit buffer.
 *
 * Input Parameters:
 *   filep - struct file pointer
 *   data - pointer to buffer with data to be written
 *   len - size of the buffer
 *
 * Returned Value:
 *   Number of bytes written.
 *
 * Assumptions/Limitations:
 *   - value of len was checked by the caller to be multiple
 *     of WS2812_RW_PIXEL_SIZE.
 *   - value of data was checked by the call chain and is not NULL
 *     (fs/vfs/fs_read.c does that in file_readv())
 *   - same for len != 0 (done by fs/vfs/fs_read.c in file_readv_compat())
 *
 ****************************************************************************/

static ssize_t avrdx_ws2812_write(struct file *filep,
                                  const char *data, size_t len)
{
  struct ws2812_dev_s *dev_data;
  struct avrdx_ws2812_s *priv_data;
  uint32_t *data_u32;
  size_t pos;

  data_u32 = (uint32_t *) data;
  dev_data = filep->f_inode->i_private;
  priv_data = dev_data->private;
  nxmutex_lock(&dev_data->lock);

  pos = filep->f_pos;

  /* Restrict len to not go beyond the buffer. */

  if (len + pos > dev_data->nleds * WS2812_RW_PIXEL_SIZE)
    {
      len = dev_data->nleds * WS2812_RW_PIXEL_SIZE - pos;
    }

#ifndef CONFIG_AVR_AVRDX_WS2812_NO_READS
  /* Write a copy to the pixel buffer */

  memcpy(((uint8_t *) priv_data->pixel_buffer) + pos, data, len);

#endif

  store_uart_bytes(priv_data->lane, data_u32,
                   pos >> 2, len >> 2,
                   dev_data->has_white);

  pos += len;
  filep->f_pos = pos;

  trigger_transmission(priv_data->lane);

  nxmutex_unlock(&dev_data->lock);

  return len;
}

/****************************************************************************
 * Name: avrdx_ws2812_read
 *
 * Description:
 *   Reads data from pixel buffer to provided pointer and returns them.
 *   Not compiled if CONFIG_AVR_AVRDX_WS2812_NO_READS is set.
 *
 * Input Parameters:
 *   filep - struct file pointer
 *   data - pointer to buffer which receives read data
 *   len - size of the buffer
 *
 * Returned Value:
 *   Number of bytes read.
 *
 * Assumptions/Limitations:
 *   Identical to avrdx_ws2812_write
 *
 ****************************************************************************/

#ifndef CONFIG_AVR_AVRDX_WS2812_NO_READS

static ssize_t avrdx_ws2812_read(struct file *filep,
                                 char *data, size_t len)
{
  struct ws2812_dev_s   *dev_data;
  struct avrdx_ws2812_s *priv_data;
  size_t pos;

  dev_data = filep->f_inode->i_private;
  priv_data = dev_data->private;

  nxmutex_lock(&dev_data->lock);

  pos = filep->f_pos;

  /* Restrict len to not go beyond the buffer. */

  if (len + pos > dev_data->nleds * WS2812_RW_PIXEL_SIZE)
    {
      len = dev_data->nleds * WS2812_RW_PIXEL_SIZE - pos;
    }

  memcpy(data, ((uint8_t *) priv_data->pixel_buffer) + pos, len);
  pos += len;
  filep->f_pos = pos;

  nxmutex_unlock(&dev_data->lock);

  return len;
}

#endif /* ifndef CONFIG_AVR_AVRDX_WS2812_NO_READS */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: avrdx_ws2812_register
 *
 * Description:
 *   Initializer for one WS2812 lane. Lane is a set of WS2812 LEDs
 *   connected serially and driven from single hardware device
 *   on the MCU side.
 *
 *   Note that this function triggers transmission of its freshly
 *   allocated and zeroed buffer, thus resetting the LEDs
 *   and turning them all of. Since the function does not know
 *   if it runs after a power-on or after a reset (and, therefore,
 *   how much time elapsed from last transmission), it will simply
 *   wait.
 *
 * Input Parameters:
 *   path - device path for this LED lane
 *   count - LED count. Limited to 255 regardless of upper half setting
 *           (The device does not have memory for more.)
 *   lane - enum avrdx_ws2812_lane_e value. Selects peripheral to which
 *          are the LEDs connected on the board
 *   has_white - bool. True if the lane has LEDs with white component.
 *               Ignored if CONFIG_WS2812_HAS_WHITE is unset. (Or if
 *               CONFIG_AVR_AVRDX_WS2812_NO_WHITE_LED is set.)
 *
 * Returned Value:
 *   OK or negated errno.
 *   ENOMEM - if the LED count exceeds limit (in addition to memory
 *            allocation failure.)
 *   ENOTSUP - if support for white LED is requested and not disabled
 *             (see #warning at the beginning of the file or description
 *             of AVR_AVRDX_WS2812_NO_WHITE_LED configuration option)
 *   EINVAL - count requests more LEDs in chain than is supported
 *
 * Assumptions/Limitations:
 *   Running in a context which allows sleeping.
 *
 ****************************************************************************/

int avrdx_ws2812_register(const char *path, uint8_t count,
                          enum avrdx_ws2812_lane_e lane,
                          bool has_white)
{
  int ret = OK;
  struct ws2812_dev_s   *dev_data = NULL;
  struct avrdx_ws2812_s *priv_data = NULL;
  avr_usart_t *usart;
  uint16_t lane_bytes;
  volatile uint8_t *portmux_reg;
  struct timespec ts;

#ifdef CONFIG_AVR_AVRDX_WS2812_NO_READS
  uint32_t *temp_pixel_buffer = NULL;
#endif

  if (count > WS2812_MAX_LED_COUNT)
    {
      return -EINVAL;
    }

  /* Deal with reset time (T_reset) of the LEDs. We do not know if this
   * code runs during a power-up or after a reset. For the latter case,
   * we have no way of knowing what was happening before the reset
   * and if the T_reset period of the LEDs already expired (if any
   * transmission was going out just when the MCU restarted.)
   *
   * This function will trigger a transmission to LEDs (to turn them
   * off in case this is a reset and they have previous values)
   * so it needs to make sure T_reset did expire.
   *
   * Datasheet says T_reset = 280us, we use 5 * 65536ns to save few
   * program memory bytes. T_reset did expire if our uptime is greater.
   */

  up_timer_gettime(&ts);

  if ((ts.tv_sec == 0) && (ts.tv_nsec < WS2812_T_RESET))
    {
      nxsched_usleep(WS2812_T_RESET - ts.tv_nsec);
    }

  /* * * * * * * Start of hardware-specific block * * * * * * *
   * * * * * * * * * * * * * *  * * * * * * * * * * * * * * * */

  /* * * * * * * Start of UART-specific block     * * * * * * */

#ifdef AVRDX_WS2812_LANE_ANY_USART
  if (
#  ifdef CONFIG_AVR_AVRDX_WS2812_USART0
      (lane == AVRDX_WS2812_LANE_USART0) ||
#  endif
#  ifdef CONFIG_AVR_AVRDX_WS2812_USART1
      (lane == AVRDX_WS2812_LANE_USART1) ||
#  endif
#  ifdef CONFIG_AVR_AVRDX_WS2812_USART2
      (lane == AVRDX_WS2812_LANE_USART2) ||
#  endif
#  ifdef CONFIG_AVR_AVRDX_WS2812_USART3
      (lane == AVRDX_WS2812_LANE_USART3) ||
#  endif
#  ifdef CONFIG_AVR_AVRDX_WS2812_USART4
      (lane == AVRDX_WS2812_LANE_USART4) ||
#  endif
#  ifdef CONFIG_AVR_AVRDX_WS2812_USART5
      (lane == AVRDX_WS2812_LANE_USART5) ||
#  endif
      0)
    {
      /* This is an UART lane. Configure the hardware. This code
       * is mostly a copy of what serial driver does. (PORTMUX_USARTROUTEB
       * is a macro defined in io.h if the MCU has more than 4 UARTs.)
       */

#  ifdef PORTMUX_USARTROUTEB
      if (lane < 4)
#  endif
        {
          portmux_reg = &(PORTMUX.USARTROUTEA);
        }
#  ifdef PORTMUX_USARTROUTEB
      else
        {
          portmux_reg = &(PORTMUX.USARTROUTEB);
        }
#  endif

      /* In theory, only the kernel should manipulate this register
       * and the value written is static (chosen by configuration.)
       * That means there should be no need to reset it. Still doing it,
       * runaway application may do accidental writes.
       */

      *(portmux_reg) = \
        (\
          *(portmux_reg) & \
          ~(avrdx_usart_portmux_masks[lane]) \
        ) | \
        avrdx_usart_portmux_bits[lane];

      /* Difference from serial driver - we must enable the pin
       * inverter (and we must do it before the pin switches to output.)
       */

      AVRDX_USART_PORT(lane).PINCONFIG = PORT_INVEN_bm;
      AVRDX_USART_PORT(lane).PINCTRLUPD = \
        avrdx_usart_tx_pins[lane];

      /* AVR128DA28/32/48/64 datasheet rev C, section 25.3.1 lists
       * changing the pin direction before activating the device itself
       * but that leads to short zero pulse. Change output value to serial
       * line idle state to prevent that.
       */

      AVRDX_USART_PORT(lane).OUTSET = \
        avrdx_usart_tx_pins[lane];
      AVRDX_USART_PORT(lane).DIRSET = \
        avrdx_usart_tx_pins[lane];

      usart = &(AVRDX_USART(lane));
      usart->BAUD = 64;
      usart->CTRLB = (USART_RXMODE_CLK2X_GC | USART_TXEN_bm);

      /* Default configuration of CTRLC is acceptable for LEDs
       * with white component; the opposite case is handled
       * below - amends the configuration.
       */

#  ifdef CONFIG_WS2812_HAS_WHITE

      /* UART lane with support for white component */

      if (has_white)
        {
          return -ENOTSUP;

#    ifdef CONFIG_AVR_AVRDX_WS2812_SHORT_LANE

          /* Has white component and uint8_t index is enforced.
           * 255 (UINT8_MAX) / 12 (bytes per LED) = 21 LEDs
           */

          if (count > (UINT8_MAX / WS2812_BYTES_PER_LED_WITH_WHITE))
            {
              ret = -EINVAL;
              goto errout;
            }
#    endif

          lane_bytes = count * WS2812_BYTES_PER_LED_WITH_WHITE;

          /* Top bit in this variable serves as a signal for the ISR
           * that the LED has white component and 9th UART data bit
           * is not used.
           */

          g_avrdx_ws2812_uart9_current[lane] = 0xff;
        }
      else
#  endif /* ifdef CONFIG_WS2812_HAS_WHITE*/
        {
          /* Does not have white component or white components
           * not enabled at all.
           */

#  ifdef CONFIG_AVR_AVRDX_WS2812_SHORT_LANE

          /* 255 (UINT8_MAX) / 8 (bytes per LED ) = 31 LEDs */

          if (count > (UINT8_MAX / WS2812_BYTES_PER_LED_NO_WHITE))
            {
              ret = -EINVAL;
              goto errout;
            }
#  endif

          lane_bytes = count * WS2812_BYTES_PER_LED_NO_WHITE;

          /* Will be populated in the ISR but must not have top bit
           * set when the ISR is entered. That would be interpreted
           * as a signal that 9th UART data bit is not in use.
           * If CONFIG_WS2812_HAS_WHITE is unset, 9th UART bit is
           * always used, the check in the ISR is skipped and we
           * don't need to do this.
           */

#  ifdef CONFIG_WS2812_HAS_WHITE
          g_avrdx_ws2812_uart9_current[lane] = 0;
#  endif

          /* Allocate array for 9th bits. Each LED needs exactly
           * one byte (24 bits packed into groups of 3, each group
           * needs one 9th bit.)
           */

          g_avrdx_ws2812_uart9_bytes[lane] = (uint8_t *) kmm_zalloc(count);
          if (g_avrdx_ws2812_uart9_bytes[lane] == NULL)
            {
              ret = -ENOMEM;
              goto errout;
            }

          /* Initialize the hardware - only the 9th bit */

          usart->CTRLC = USART_PMODE_EVEN_GC \
              | USART_CHSIZE_9BITH_GC;
        }
    }
  else

  /* When support for other hardware is added, we want this to continue
   * with else if. Doing it like this gives us an unterminated else though,
   * needs to be taken care of at the end of the hardware-specific section.
   */

  /* * * * * * * End of UART-specific block       * * * * * * */

#else

  /* No UART lane - since to other lane type exists, that's an error.
   * Should not happen though, checked elsewhere.
   */

#  error No lane selected
#endif
    {
      /* Every hardware-specific block ends with else to allow
       * the next block to start with if, forming else if.
       * This empty block executes after the last else.
       *
       * The compiler is unable to recognize that the list of enums
       * in the condition above is exhaustive and complaints (not always!)
       * that lane_bytes may be used uninitialized. Silence it.
       */

      lane_bytes = 0;
    }

  g_avrdx_ws2812_xmit_bytes[lane] = (uint8_t *) kmm_zalloc(lane_bytes);
  if (g_avrdx_ws2812_xmit_bytes[lane] == NULL)
    {
      ret = -ENOMEM;
      goto errout;
    }

  g_avrdx_ws2812_xmit_count[lane] = lane_bytes;
  g_avrdx_ws2812_xmit_next[lane] = 0;

  g_avrdx_ws2812_xmit_ts[lane] = \
      (struct timespec *) kmm_zalloc(sizeof(struct timespec));
  if (g_avrdx_ws2812_xmit_ts[lane] == NULL)
    {
      ret = -ENOMEM;
      goto errout;
    }

  dev_data = (struct ws2812_dev_s *) kmm_zalloc(sizeof(struct ws2812_dev_s));
  if (dev_data == NULL)
    {
      ret = -ENOMEM;
      goto errout;
    }

  priv_data = \
      (struct avrdx_ws2812_s *) kmm_zalloc(sizeof(struct avrdx_ws2812_s));
  if (priv_data == NULL)
    {
      ret = -ENOMEM;
      goto errout;
    }

#ifndef CONFIG_AVR_AVRDX_WS2812_NO_READS
  priv_data->pixel_buffer = (uint32_t *) \
      kmm_zalloc(sizeof(uint32_t) * count);
  if (priv_data->pixel_buffer == NULL)
    {
      ret = -ENOMEM;
      goto errout;
    }

  /* #else - pixel_buffer member is not part of the struct,
   * and it would be set to zero by kmm_zalloc above anyway
   */
#endif

  priv_data->lane = lane;

  nxmutex_init(&dev_data->lock);

  /* File operation handlers */

  dev_data->open = avrdx_ws2812_open;
  dev_data->close = avrdx_ws2812_close;
  dev_data->write = avrdx_ws2812_write;

#ifndef CONFIG_AVR_AVRDX_WS2812_NO_READS
  dev_data->read = avrdx_ws2812_read;

  /* #else - initialized to NULL by kmm_zalloc */
#endif

  /* Other data */

  dev_data->private = priv_data;
  dev_data->clock = 0;            /* not explained
                                   * in include/nuttx/leds/ws2812.h,
                                   * not present in drivers/leds/ws2812.c,
                                   * set to zero and see what breaks.
                                   */
  dev_data->port = 0;             /* same here */
  dev_data->nleds = count;

#ifdef CONFIG_WS2812_HAS_WHITE
  dev_data->has_white = has_white;

  /* #else - left unset. The struct is allocated with kmm_zalloc
   * so FALSE is already set
   */
#endif

  g_avrdx_ws2812_swint_vector[lane] = \
    avrdx_swint_allocate(transmission_completed, (void *)lane);
  if (g_avrdx_ws2812_swint_vector[lane] == 0)
    {
      goto errout; /* No vector available */
    }

  /* xmit_bytes are initialized to zero but we can't just put zero bytes
   * into the buffer - that value is not interpreted as zero by the LED.
   * Instead, we populate the buffer as if it was written by userspace.
   */

#ifdef CONFIG_AVR_AVRDX_WS2812_NO_READS
  /* No reads - no pixel buffer. We need to allocate a temporary one */

  temp_pixel_buffer = (uint32_t *) kmm_zalloc(sizeof(uint32_t) * count);
  if (temp_pixel_buffer == NULL)
    {
      ret = -ENOMEM;
      goto errout;
    }

  store_uart_bytes(lane, temp_pixel_buffer, 0, count, has_white);
  kmm_free(temp_pixel_buffer);
#else
  store_uart_bytes(lane, priv_data->pixel_buffer, 0, count, has_white);
#endif

  ret = ws2812_register(path, dev_data);
  if (ret < 0)
    {
      goto errout;
    }

  trigger_transmission(lane);

  return ret;

errout:

  /* Free memory that was allocated, preferably in reverse order. */

#ifdef CONFIG_AVR_AVRDX_WS2812_NO_READS
  if (temp_pixel_buffer)
    {
      kmm_free(temp_pixel_buffer);
    }
#endif

  if (priv_data)
    {
#ifndef CONFIG_AVR_AVRDX_WS2812_NO_READS
      if (priv_data->pixel_buffer)
        {
          kmm_free(priv_data->pixel_buffer);
        }

#endif
      kmm_free(priv_data);
    }

  if (dev_data)
    {
      kmm_free(dev_data);
    }

#ifdef AVRDX_WS2812_LANE_ANY_USART
  if (g_avrdx_ws2812_uart9_bytes[lane])
    {
      kmm_free(g_avrdx_ws2812_uart9_bytes[lane]);
    }
#endif

  if (g_avrdx_ws2812_xmit_ts[lane])
    {
      kmm_free(g_avrdx_ws2812_xmit_ts[lane]);
    }

  if (g_avrdx_ws2812_xmit_bytes[lane])
    {
      kmm_free(g_avrdx_ws2812_xmit_bytes[lane]);
    }

  return ret;
}
