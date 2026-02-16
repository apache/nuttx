/****************************************************************************
 * arch/avr/src/avrdx/avrdx_twi.c
 * Two Wire Interface (I2C and SMBus compatible) driver
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

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/signal.h>
#include <debug.h>

#include "avrdx.h"
#include "avrdx_twi.h"
#include <avr/io.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if (defined(CONFIG_AVR_TWI0) || defined(CONFIG_AVR_TWI1))

#define TWI_CLEAR_ALL_MSTATUS_FLAGS(twi) \
  (twi)->MSTATUS = TWI_RIF_bm | TWI_WIF_bm | \
    TWI_CLKHOLD_bm | TWI_ARBLOST_bm | TWI_BUSERR_bm;

#define AVRDX_TWI_TRNSFER_TIMEOUT_SEC 0
#define AVRDX_TWI_TRNSFER_TIMEOUT_MSEC 250
#define AVRDX_TWI_TRNSFER_TIMEOUT_TICKS \
  (SEC2TICK(AVRDX_TWI_TRNSFER_TIMEOUT_SEC) + MSEC2TICK(AVRDX_TWI_TRNSFER_TIMEOUT_MSEC))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct avrdx_twi_priv_s
{
  /* Pointer to this struct is cast to a pointer to struct i2c_master_s,
   * the beginning must match
   *
   * The struct has some 35 bytes so it fits into 64 bytes limit
   * for AVR instructions that use pointer with offset. That includes
   * large data types, allowing to store them directly without additional
   * allocations.
   */

  const struct i2c_ops_s *ops;

  uint8_t twi_n;            /* Peripheral index */

  uint8_t msgidx;           /* Index of current message */
  uint8_t msg_stopped_idx;  /* Index of first message that was not sent
                             * up to and including STOP condition. (See
                             * arbitration loss handling for more.)
                             */
  ssize_t msg_bufidx;       /* Current message's buffer index. Contains
                             * index of _next_ byte to be transmitted
                             * (zero indicates address write state) */
  struct i2c_msg_s *msgs;   /* Messages */
  uint8_t msg_count;        /* Message count */
  struct timespec timeout;  /* Transmission timeout, hardcoded to 250ms */

  int rval;                 /* Return value. If not OK, transmission
                             * had an error and will be interrupted. */

  mutex_t lock;             /* Bus ownership mutex */
  sem_t sem_isr;            /* Wait for interrupt semaphore */
};
typedef struct avrdx_twi_priv_s avrdx_twi_priv_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int avrdx_twi_transfer(struct i2c_master_s *dev,
                              struct i2c_msg_s *msgs, int count);
static int avrdx_twi_set_frequency(avrdx_twi_priv_t *priv, uint32_t f_scl);
static int avrdx_twi_start_transfer(avrdx_twi_priv_t *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* I2C ops structure */

static const IOBJ struct i2c_ops_s g_twi_ops =
{
  .transfer = avrdx_twi_transfer
};

/* Interrupt vectors (master mode) */

static const IOBJ uint8_t avrdx_twi_master_interrupts[] =
{
  AVRDX_IRQ_TWI0_TWIM
#  if defined(CONFIG_AVR_HAVE_TWI1)
  , AVRDX_IRQ_TWI1_TWIM
#  endif
};

/* TWI device description structs. Pointers or pointer, depending
 * on chip peripherals. Allocated as needed.
 */

#  if defined(CONFIG_AVR_HAVE_TWI1)
static avrdx_twi_priv_t *g_twi_ports[2] =
{
  NULL, NULL
};
#  else
static avrdx_twi_priv_t *g_twi_ports[1] =
{
  NULL
};
#  endif

/* SCL frequency limit based on mode given by configuration */

static const IOBJ uint32_t avrdx_twi_fscl_limit[] =
{
#  ifdef CONFIG_AVR_TWI0
#    if defined(CONFIG_AVR_TWI0_MODE_STD)
  I2C_SPEED_STANDARD  /* 100kHz */
#    elif defined(CONFIG_AVR_TWI0_MODE_FAST)
  I2C_SPEED_FAST      /* 400kHz */
#    else
  I2C_SPEED_FAST_PLUS /* 1MHz */
#    endif
#  else
  0
#  endif

#  if defined(CONFIG_AVR_HAVE_TWI1) && defined(CONFIG_AVR_TWI1)
#    if defined(CONFIG_AVR_TWI1_MODE_STD)
  , I2C_SPEED_STANDARD
#    elif defined(CONFIG_AVR_TWI1_MODE_FAST)
  , I2C_SPEED_FAST
#    else
  , I2C_SPEED_FAST_PLUS
#    endif
#  else
  , 0
#  endif
};

/* f_per bitshift for multiplication by t_R (see avrdx_twi_set_baud
 * for an actual explanation.)
 */

static const IOBJ uint8_t avrdx_twi_tr_bitshift[] =
{
#  ifdef CONFIG_AVR_TWI0
#    if defined(CONFIG_AVR_TWI0_MODE_STD)
  5
#    elif defined(CONFIG_AVR_TWI0_MODE_FAST)
  7
#    else
  8
#    endif
#  else
  0
#  endif

#  if defined(CONFIG_AVR_HAVE_TWI1) && defined(CONFIG_AVR_TWI1)
#    if defined(CONFIG_AVR_TWI1_MODE_STD)
  , 5
#    elif defined(CONFIG_AVR_TWI1_MODE_FAST)
  , 7
#    else
  , 8
#    endif
#  else
  , 0
#  endif
};

/* Values loaded into TWI.CTRLA register during initialization */

static const IOBJ uint8_t avrdx_twi_init_ctrla[] =
{
#  ifdef CONFIG_AVR_TWI0
  (
    0

#    if defined(CONFIG_AVR_TWI0_MODE_FAST_PLUS)
    | TWI_FMPEN_ON_GC
#    endif

#    if defined(CONFIG_AVR_TWI0_SDAHOLD_50NS)
    | TWI_SDAHOLD_50NS_GC
#    elif defined(CONFIG_AVR_TWI0_SDAHOLD_300NS)
    | TWI_SDAHOLD_300NS_GC
#    elif defined(CONFIG_AVR_TWI0_SDAHOLD_500NS)
    | TWI_SDAHOLD_500NS_GC
#    endif
  )
#  else /* for ifdef CONFIG_AVR_TWI0 */
  0
#  endif

#  if defined(CONFIG_AVR_HAVE_TWI1) && defined(CONFIG_AVR_TWI1)
  ,
  (
    0

#    if defined(CONFIG_AVR_TWI1_MODE_FAST_PLUS)
    | TWI_FMPEN_ON_GC
#    endif

#    if defined(CONFIG_AVR_TWI1_SDAHOLD_50NS)
    | TWI_SDAHOLD_50NS_GC
#    elif defined(CONFIG_AVR_TWI1_SDAHOLD_300NS)
    | TWI_SDAHOLD_300NS_GC
#    elif defined(CONFIG_AVR_TWI1_SDAHOLD_500NS)
    | TWI_SDAHOLD_500NS_GC
#    endif

  )
#  else
  , 0
#  endif
};

/* Port multiplexer settings, divided to value to be written
 * and mask.
 */

const IOBJ uint8_t avrdx_twi_portmux_bits[] =
{
#  ifdef CONFIG_AVR_TWI0
#    if defined(CONFIG_AVR_TWI0_ALT0)
  PORTMUX_TWI0_DEFAULT_GC
#    elif defined(CONFIG_AVR_TWI0_ALT1)
  PORTMUX_TWI0_ALT1_GC
#    elif defined(CONFIG_AVR_TWI0_ALT2)
  PORTMUX_TWI0_ALT2_GC
#    else
#      error Kconfig error, no option is set
#    endif
#  else
  0 /* any value, peripheral not enabled */
#  endif

#  ifdef CONFIG_AVR_TWI1
#    if defined(CONFIG_AVR_TWI1_ALT0)
  , PORTMUX_TWI1_DEFAULT_GC
#    elif defined(CONFIG_AVR_TWI1_ALT1)
  , PORTMUX_TWI1_ALT1_GC
#    elif defined(CONFIG_AVR_TWI1_ALT2)
  , PORTMUX_TWI1_ALT2_GC
#    else
#      error Kconfig error, no option is set
#    endif
#  else
  , 0 /* any value, peripheral not enabled */
#  endif
};

const IOBJ uint8_t avrdx_twi_portmux_masks[] =
{
  PORTMUX_TWI0_GM
#  if defined(CONFIG_AVR_HAVE_TWI1)
  , PORTMUX_TWI1_GM
#  endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: avrdx_twi_is_retry_time_left
 *
 * Description:
 *   This function compares timeout set on transmission start with current
 *   time.
 *
 * Input Parameters:
 *   priv - instance of avrdx_twi_priv_t denoting the peripheral
 *
 * Returned Value:
 *   True if the timeout wasn't reached and transmission can be retried.
 *
 *
 ****************************************************************************/

static bool avrdx_twi_is_retry_time_left(avrdx_twi_priv_t *priv)
{
  struct timespec ts;

  clock_systime_timespec(&ts);

  if (clock_timespec_compare(&ts, &(priv->timeout)) < 0)
    {
      return true;
    }
  else
    {
      return false;
    }
}

/****************************************************************************
 * Name: avrdx_twi_prepare_next_msg
 *
 * Description:
 *   Prepares new message to be processed. Specifically it looks
 *   at the current and next message and issues REPEATED START
 *   (unless ordered not to) or STOP condition.
 *
 *   Then it starts transmitting it.
 *
 * Input Parameters:
 *   priv - instance of avrdx_twi_priv_t denoting the peripheral
 *   twi - pointer to struct used to access peripheral I/O registers
 *
 * Assumptions/Limitations:
 *   Only ever executed from avrdx_twi_interrupt or its components
 *   (avrdx_twi_interrupt_m_wif, avrdx_twi_interrupt_m_rif)
 *
 ****************************************************************************/

static void avrdx_twi_prepare_next_msg(avrdx_twi_priv_t *priv,
                                       avr_twi_t *twi)
{
  struct i2c_msg_s *msg;
  int ret;

#  ifndef CONFIG_AVR_TWI_FORBID_NOSTART
  struct i2c_msg_s *next_msg;
#  endif

  msg = &(priv->msgs[priv->msgidx]);
  priv->msgidx++;
  if (priv->msgidx == priv->msg_count)
    {
      /* This was last message. It does not have NOSTOP flag so we send
       * the STOP condition. Right? Well, at least AMG88xx driver
       * does a single message transfer with NOSTOP on it (the code
       * then proceeds to send more messages.)
       *
       * On the other hand, I2C drivers for multiple chips
       * and architectures release bus mutex when the transfer is over,
       * leaving the bus open for other users (who can do whatever
       * including sending STOP condition on the bus.)
       *
       * This code assumes that NOSTOP on last message is incorrect
       * (or, at least, cannot be actually enforced without external
       * locking) and therefore that it is not set.
       *
       * Also, we just read last byte so nACK it. (Ignored by hardware
       * if this was a write transmission.)
       *
       * Then post the ISR semaphore. We are done.
       */

      twi->MCTRLB |= (TWI_ACKACT_NACK_GC | TWI_MCMD_STOP_GC);
      nxsem_post(&(priv->sem_isr));

      return;
    }

  /* Not last message */

#  ifndef CONFIG_AVR_TWI_FORBID_NOSTART

  next_msg = &(priv->msgs[priv->msgidx]);
  if (next_msg->flags & I2C_M_NOSTART)
    {
      /* Next message requests to transmit no REPEATED START condition
       * and is to be treated as a continuation of this one. Direction
       * must match according to i2c/i2c_master.h (and one would expect
       * that the address should match too.)
       *
       * Not checking to save flash space.
       *
       * This function is expected to start transmission of the next
       * message. However, it cannot do it by calling
       * avrdx_twi_start_transfer because that would write into MADDR
       * and start new transfer (not a continuation.) We must manually
       * do what is otherwise done by the interrupt handler.
       */

      if (msg->flags & I2C_M_READ)
        {
          uint8_t temp;

          /* NACK was set for last byte (not transmitted yet, Smart mode
           * only transmits ACK.) Clear it, we are not done after all.
           * Also command the peripheral to do next read.
           */

          temp = twi->MCTRLB;
          temp &= ~TWI_ACKACT_bm;
          temp |= TWI_MCMD_RECVTRANS_GC;
          twi->MCTRLB = temp;

          priv->msg_bufidx = 0;
        }
      else
        {
          /* QCEN_ZERO_LENGTH_CHECKED mark. Message can not have zero length.
           * If CONFIG_AVR_TWI_QCEN is not set, zero-length messages are
           * forbidden altogether. If CONFIG_AVR_TWI_QCEN is set, zero-length
           * messages are still forbidden for messages with I2C_M_NOSTART
           * flag.
           *
           * Write first byte of the message right away then.
           */

          twi->MDATA = next_msg->buffer[0];
          priv->msg_bufidx = 1;
        }

      return;
    }
#  endif

  /* Not last message and no NOSTART stuff */

  if (!(msg->flags & I2C_M_NOSTOP))
    {
      /* This flag requests to issue REPEATED START on next message
       * and it is not present. We are asked to transmit STOP condition
       * and a START condition on next message.
       */

      priv->msg_stopped_idx = priv->msgidx;
      twi->MCTRLB |= (TWI_ACKACT_NACK_GC | TWI_MCMD_STOP_GC);

      /* The interesting question here is if we are allowed to write
       * MADDR when the stop condition is still being processed.
       * (That is what avrdx_twi_start_transfer will do. Datasheet
       * doesn't seem to say anything on that topic so assume "yes"
       * and hope the user will not ask for this.
       */
    }
  else
    {
      /* Repeated start is requested and we also may need to execute
       * acknowledge action (if reading.)
       */

      twi->MCTRLB |= (TWI_ACKACT_NACK_GC | TWI_MCMD_REPSTART_GC);
    }

  /* No else, no other option left. (REPEATED) START is sent
   * on address write.
   */

  ret = avrdx_twi_start_transfer(priv);
  if (ret != OK)
    {
      /* Caller will handle this */

      priv->rval = ret;
    }
}

/****************************************************************************
 * Name: avrdx_twi_interrupt_m_rif
 *
 * Description:
 *   Interrupt handler for TWI peripheral - read interrupt. Not attached
 *   as interrupt handler directly, called from avrdx_twi_interrupt
 *   when RIF flag in MSTATUS is set (read was finished.)
 *
 * Input Parameters:
 *   priv - instance of avrdx_twi_priv_t denoting the peripheral
 *   twi - pointer to struct used to access peripheral I/O registers
 *
 * Returned Value:
 *   Void. Any errors encountered during processing are saved
 *   into the state variable and the caller must check it.
 *
 * Assumptions/Limitations:
 *   Only ever executed from avrdx_twi_interrupt
 *
 ****************************************************************************/

static void avrdx_twi_interrupt_m_rif(avrdx_twi_priv_t *priv, avr_twi_t *twi)
{
  struct i2c_msg_s *msg;

  msg = &(priv->msgs[priv->msgidx]);

#  ifdef CONFIG_AVR_TWI_QCEN
  if (msg->length == 0)
    {
      /* If the message has zero length, it was a Quick Command.
       *
       * Address write usually triggers write interrupt but this case is
       * an exception. Address write for quick command with R/nW bit set
       * triggers read interrupt. We need to handle the possibility
       * that the target device did not respond.
       */

      if (twi->MSTATUS & TWI_RXACK_bm)
        {
          /* STOP condition sent in the overall R/W ISR */

          priv->rval = -ENXIO;
          return;
        }

      /* Quick command and target device did respond. Must not read
       * any data, issue stop condition. (This call will take care of that.)
       */

      avrdx_twi_prepare_next_msg(priv, twi);
      return;
    }
#  endif

  /* QCEN_ZERO_LENGTH_CHECKED mark. Zero-length messages are not allowed,
   * we can read. Note that reading MDATA will trigger acknowledge action,
   * via Smart Mode unless we are reading last byte, in which case
   * the SCTRLB register was previously set to NACK and Smart mode will
   * not trigger. (See longer explanation in avrdx_twi_start_transfer.)
   */

  msg->buffer[priv->msg_bufidx] = twi->MDATA;
  priv->msg_bufidx++;

  if (priv->msg_bufidx == msg->length)
    {
      /* This was the last byte to be read, next message.
       *
       * Note: acknowledge action was not done yet and cannot be done yet.
       * If the next message has I2C_M_NOSTART, we will need to ACK this
       * "last" byte after all. avrdx_twi_prepare_next_msg will take care
       * of both cases.
       */

      avrdx_twi_prepare_next_msg(priv, twi);
    }
  else if (priv->msg_bufidx + 1 == msg->length)
    {
      /* The byte that will be read next is the last one.
       * Must not ACK it automatically. That would trigger
       * another (unwanted) read.
       */

      twi->MCTRLB &= ~TWI_ACKACT_bm;
    }

  return;
}

/****************************************************************************
 * Name: avrdx_twi_interrupt_m_wif
 *
 * Description:
 *   Interrupt handler for TWI peripheral - write interrupt. Not attached
 *   as interrupt handler directly, called from avrdx_twi_interrupt
 *   if WIF flag in MSTATUS is set (write was finished.)
 *
 *   Sets transaction return value to an error state if the byte
 *   transmission had an error (eg. the target did not respond.)
 *
 * Input Parameters:
 *   priv - instance of avrdx_twi_priv_t denoting the peripheral
 *   twi - pointer to struct used to access peripheral I/O registers
 *
 * Returned Value:
 *   None. Any errors encountered during processing are saved
 *   into the state variable and the caller must check it.
 *
 * Assumptions/Limitations:
 *   Only ever executed from avrdx_twi_interrupt
 *
 ****************************************************************************/

static void avrdx_twi_interrupt_m_wif(avrdx_twi_priv_t *priv, avr_twi_t *twi)
{
  struct i2c_msg_s *msg;

  msg = &(priv->msgs[priv->msgidx]);

  if (twi->MSTATUS & TWI_RXACK_bm)
    {
      /* Target did not acknowledge (address or data) */

      if (!(priv->msg_bufidx))
        {
          /* nACK from target on address write, abort transmission,
           * return ENXIO. ISR semaphore will be posted in the caller,
           * STOP condition sent from there as well.
           */

          priv->rval = -ENXIO;
          return;
        }

      /* nACK from target on data write, where exactly are we? */

      if (priv->msg_bufidx < msg->length)
        {
          /* This was not last byte we wanted to transmit,
           * that is an error. EIO, semaphore etc.
           */

          priv->rval = -EIO;
          return;
        }

      /* nACK from target on data write and we transmitted all bytes
       * we wanted to transmit. Not an error. Send next message.
       * (Or not, if this one was the last.)
       */

      avrdx_twi_prepare_next_msg(priv, twi);

      return;
    }

  /* RXACK is not set, the write was accepted by the receiving device */

  if (!(priv->msg_bufidx))
    {
      /* This was an address write (zero msg_bufidx tells us that)
       * and we are writing data.
       *
       * (Why? Because if we were reading, the MCU did not trigger
       * an interrupt and proceeded directly to reading first data byte.
       * In case of address write in read direction for quick command,
       * that runs the read interrupt handler, meaning we are still
       * not here.)
       */

#  ifdef CONFIG_AVR_TWI_QCEN
      /* Non-zero length is only permitted if CONFIG_AVR_TWI_QCEN is set */

      if (msg->length)
#  endif
        {
          /* Send first byte */

          twi->MDATA = msg->buffer[0];
          priv->msg_bufidx = 1;
        }
#  ifdef CONFIG_AVR_TWI_QCEN
      else
        {
          /* Attempt to write message of size 0. Done. */

          avrdx_twi_prepare_next_msg(priv, twi);
        }
#  endif

      /* Address write handled */

      return;
    }

  /* RxACK is not set (write accepted), msg_bufidx not zero, we are
   * in WIF handler: write next (not first) byte, if any.
   */

  if (priv->msg_bufidx == msg->length)
    {
      /* This message is done, send next one. (Last byte was not NACKed
       * by the target device.)
       */

      avrdx_twi_prepare_next_msg(priv, twi);
    }
  else
    {
      twi->MDATA = msg->buffer[priv->msg_bufidx];
      priv->msg_bufidx++;
    }
}

/****************************************************************************
 * Name: avrdx_twi_interrupt
 *
 * Description:
 *   Interrupt handler for TWI peripheral
 *
 * Input Parameters:
 *   - IRQ number
 *   - context (unused)
 *   - argument given to irq_attach - pointer to avrdx_twi_priv_t
 *
 * Returned Value:
 *   OK
 *
 * Assumptions/Limitations:
 *   Runs in interrupt context.
 *
 ****************************************************************************/

static int avrdx_twi_interrupt(int irq, void *context, FAR void *arg)
{
  avrdx_twi_priv_t *priv;
  avr_twi_t *twi;

  priv = (avrdx_twi_priv_t *) arg;
  twi = &(AVRDX_TWI(priv->twi_n));

  if (twi->MSTATUS & TWI_BUSERR_bm)
    {
      /* Bus error. Protocol violating Start condition, Stop condition etc.
       * Abort transfer with EIO and clear the error. No attempt to retry.
       * Flush the state (will change the internal state to IDLE. No attempt
       * to send STOP condition.)
       */

      priv->rval = -EIO;
      TWI_CLEAR_ALL_MSTATUS_FLAGS(twi);
      twi->MCTRLB |= TWI_FLUSH_bm;

      nxsem_post(&(priv->sem_isr));

      return OK;
    }

  if (twi->MSTATUS & TWI_ARBLOST_bm)
    {
      /* Arbitration lost. The side that won the arbitration was sending
       * the same data as we were up until this point. The receiver
       * has no way of knowing we were trying too and that allows us
       * to attempt a retry when the bus is released.
       */

      if (avrdx_twi_is_retry_time_left(priv))
        {
          /* It is possible to send a complete message and still have
           * an arbitration loss in the follow-up message. We will therefore
           * reset overall message index to first message that has not yet
           * been transmitted up to and including a STOP condition.
           * Index of such message is stored in msg_stopped_idx variable
           * which is updated every time we issue STOP condition on the bus.
           *
           * New start is triggered by writing address
           * in avrdx_twi_start_transfer(). The hardware will wait until
           * the bus is released.
           *
           * All status flags are cleared when the address is written.
           * (See AVR128DA28 datasheet rev C, 27.5.6 Host Status)
           */

          priv->msgidx = priv->msg_stopped_idx;
          avrdx_twi_start_transfer(priv);

          /* No nxsem_post, we are not done */

          return OK;
        }
      else
        {
          /* We don't have the time. Not sending STOP condition, we do
           * not own the bus.
           */

          priv->rval = -EBUSY;
          TWI_CLEAR_ALL_MSTATUS_FLAGS(twi);

          nxsem_post(&(priv->sem_isr));

          return OK;
        }
    }

  /* No bus error nor arbitration loss (all branches in previous
   * "if" blocks return.)
   *
   * If / else if is correct because "RIF and WIF are mutually
   * exclusive and cannot be set simultaneously." (DS40002183C,
   * AVR128DA28/32/48/64 data sheet revision C, TWI - Two-wire
   * interface, Receiving Data Packets, section 27.3.2.2.5)
   */

  if (twi->MSTATUS & TWI_RIF_bm)
    {
      avrdx_twi_interrupt_m_rif(priv, twi);
    }
  else if (twi->MSTATUS & TWI_WIF_bm)
    {
      avrdx_twi_interrupt_m_wif(priv, twi);
    }

  if (priv->rval != OK)
    {
      /* RIF/WIF handler detected an error and returned, issue STOP
       * condition on the bus and post the ISR semaphore.
       */

      twi->MCTRLB |= TWI_MCMD_STOP_GC;
      nxsem_post(&(priv->sem_isr));
    }

  return OK;
}

/****************************************************************************
 * Name: avrdx_twi_start_transfer
 *
 * Description:
 *   Starts transfer of a single (current) message.
 *
 * Input Parameters:
 *   Pointer to avrdx_twi_priv_t
 *
 * Returned Value:
 *   OK or error code. Error code may only be returned if no transmission
 *   was started. If a transmission starts, the function will return OK
 *   (and note the failure in a state variable.)
 *
 * Assumptions/Limitations:
 *   Called by avrdx_twi_transfer with interrupts disabled or by interrupt
 *   handler, bus mutex is locked, bus is not in unknown state.
 *
 ****************************************************************************/

static int avrdx_twi_start_transfer(avrdx_twi_priv_t *priv)
{
  struct i2c_msg_s *msg;
  avr_twi_t *twi;
  int ret;

  twi = &(AVRDX_TWI(priv->twi_n));

  msg = &(priv->msgs[priv->msgidx]);
  priv->msg_bufidx = 0;

  /* Set frequency requested by this message */

  ret = avrdx_twi_set_frequency(priv, msg->frequency);
  if (ret != OK)
    {
      return ret;
    }

#  ifdef CONFIG_AVR_TWI_QCEN
  /* With Quick Command mode enabled, we need to check every
   * message for zero length. If disabled, such messages are
   * rejected when submitted.
   */

  if (msg->length == 0)
    {
      twi->MCTRLA |= TWI_QCEN_bm;
    }
  else
    {
      twi->MCTRLA &= ~TWI_QCEN_bm;
    }
#  endif

  /* Peripheral will issue START condition unless the bus
   * is busy, in which case it'll wait and then issue
   * the START condition
   */

  if (msg->flags & I2C_M_READ)
    {
      /* The hardware will start transmitting the address with R/Wn
       * bit set to "read". If the target acknowledges the address
       * write, the hardware will immediately start reading a data
       * byte. It will raise read interrupt after that.
       *
       * Therefore:
       * - everything needs to be configured now
       * - reads of size 0 are not acceptable unless Quick Command
       *   mode is enabled
       *
       * Zero-length message is handled above. We also need to handle
       * message with length of 1. We read received byte in the interrupt
       * handler and that triggers acknowledge action with Smart mode
       * enabled. That needs to be prevented, we would read additional
       * byte beyond what the message expects.
       *
       * See AVR128DA28/32/48/64 data sheet revision C, TWI - Two-wire
       * interface, Smart Mode, section 27.3.3.3: This feature is only
       * active when the Acknowledge Action (ACKACT) bit in the Host
       * Control B (TWIn.MCTRLB) register is set to ACK. If the ACKACT
       * bit is set to NACK, the TWI host will not generate a NACK
       * after the MDATA register is read.
       *
       * Set NACK now, Smart mode will not perform acknowledge action.
       */

      if (msg->length == 1)
        {
          /* Note - using TWI0.MCTRLB = \
           * (TWI0.MCTRLB & ~TWI_ACKACT_bm) | TWI_ACKACT_NACK_GC;
           * would be cleaner but the compiler does not figure out
           * that it can do single instruction (and/or) in both branches
           */

          twi->MCTRLB |= TWI_ACKACT_bm;
        }
      else
        {
          twi->MCTRLB &= ~TWI_ACKACT_bm;
        }

      twi->MADDR = I2C_READADDR8(msg->addr);
    }
  else
    {
      twi->MADDR = I2C_WRITEADDR8(msg->addr);
    }

  /* Interrupt handler will take over */

  return OK;
}

/****************************************************************************
 * Name: avrdx_twi_transfer
 *
 * Description:
 *   Initiate TWI transfer.
 *
 * Input Parameters:
 *   - dev - pointer to TWI device structure
 *   - msgs - array of messages to be transmitted
 *   - count - count of messages
 *
 * Returned Value:
 *   Zero on success, negative number on error.
 *
 ****************************************************************************/

static int avrdx_twi_transfer(struct i2c_master_s *dev,
                              struct i2c_msg_s *msgs, int count)
{
  avrdx_twi_priv_t *priv = (avrdx_twi_priv_t *)dev;
  struct timespec timeout_duration =
    {
      .tv_sec = AVRDX_TWI_TRNSFER_TIMEOUT_SEC,
      .tv_nsec = AVRDX_TWI_TRNSFER_TIMEOUT_MSEC * 1000000UL
    };

  avr_twi_t *twi;
  irqstate_t irqflags;
  int ret;
  int idx;
  int i;

  /* Input check - this driver does not support 10 bit addresses,
   * limited message count etc.
   */

  if (count > 127)
    {
      priv->rval = -ENOTSUP;
      goto errout;
    }

  if (!count)
    {
      /* Feels like an error too but on the other hand, we can
       * send no message easily.
       */

      priv->rval = OK;
      goto errout;
    }

  for (idx = 0; idx < count; idx++)
    {
      if (msgs[idx].flags & I2C_M_TEN)
        {
          priv->rval = -ENOTSUP; /* Not implemented */
          goto errout;
        }

#  ifdef CONFIG_AVR_TWI_FORBID_NOSTART
      if (msgs[idx].flags & I2C_M_NOSTART)
        {
          priv->rval = -EINVAL;
          goto errout;
        }
#  endif

      if (msgs[idx].length == 0)
        {
#  ifndef CONFIG_AVR_TWI_QCEN
          /* Quick commands not enabled, messages must have non-zero length */

          priv->rval = -EINVAL;
          goto errout;
#  else
          /* Quick commands are enabled, messages may have zero length but
           * such message must not have NOSTOP flag. (The documentation says
           * that after sending a quick command, STOP condition must
           * be transmitted. See for example section 27.5.4 Host Control A
           * in AVR128DA28 datasheet.)
           *
           * It must not have NOSTART flag either. In that case, it is not
           * a quick command.
           *
           * Behaviour of both #ifdef branches is assumed by other parts
           * of the code. Those are marked with comment
           * QCEN_ZERO_LENGTH_CHECKED
           */

          if (msgs[idx].flags & (I2C_M_NOSTOP | I2C_M_NOSTART))
            {
              priv->rval = -EINVAL;
              goto errout;
            }
#  endif
        }
    }

  /* Make sure the bus is not in unknown state. Wait for 1ms for that
   * to happen at most. (That's i = 5 times 200us.)
   */

  twi = &(AVRDX_TWI(priv->twi_n));
  i = 5;

  while ((twi->MSTATUS & TWI_BUSSTATE_GM) == TWI_BUSSTATE_UNKNOWN_GC)
    {
      /* Bus is in unknown state after activation until the peripheral
       * either detects a STOP condition or times out waiting for it
       * while the bus is inactive.
       *
       * Wait for change of state.
       *
       * No need to wait in other states:
       * - if the bus is owned (BUSSTATE is OWNER), that's what
       *   the mutex below is for (another task is using the bus)
       * - if the bus is idle, that's what we want, we can transmit
       * - if the bus is busy, the hardware will simply wait
       *   for it to become idle
       */

      nxsched_usleep(200);
      i--;
      if (!i)
        {
          return -EIO;
        }
    }

  /* Lock mutex. After we start a transfer, bus is busy,
   * no other device may access it until we are done here.
   */

  ret = nxmutex_lock(&(priv->lock));
  if (ret < 0)
    {
      return ret;
    }

  /* Store messages to be processed (need them available
   * in the interrupt handler.) Also set OK return value.
   */

  priv->msgs = msgs;
  priv->msg_count = count;
  priv->rval = OK;

  /* Set current message index instance variable to first msg,
   * calculate transmission timeout and start transmitting
   */

  priv->msgidx = 0;
  priv->msg_stopped_idx = 0;
  clock_systime_timespec(&(priv->timeout));
  clock_timespec_add(&(priv->timeout), \
      &timeout_duration, \
      &(priv->timeout));

  /* If the semaphore wait times out, we will be cancelling the operation
   * and that also means resetting the TWI device (because we don't know
   * its state nor want to handle it. In that period, we don't want TWI's
   * ISR to execute.)
   *
   * Interrupt are disabled here though because as soon as start_transfer
   * function executes, the interrupt may be triggered.
   *
   * (Note - waiting for the semaphore will re-enable interrupts.)
   */

  irqflags = enter_critical_section();

  /* Start transmission - standalone function, called from ISR too */

  ret = avrdx_twi_start_transfer(priv);
  if (ret != OK)
    {
      priv->rval = ret;
      goto errout_isrs;
    }

  /* Wait for the transmission to complete or timeout. The timeout
   * is extended by some amount because we prefer that it is detected
   * and handled in the ISR instead. (Current state of things is known
   * there and can be dealt with properly - sending STOP condition
   * for example.)
   */

  ret = nxsem_tickwait_uninterruptible(&(priv->sem_isr),
                                       AVRDX_TWI_TRNSFER_TIMEOUT_TICKS + \
                                         MSEC2TICK(50)
                                       );
  if (ret == -ETIMEDOUT)
    {
      /* Transmission timed out, disable the peripheral, clear
       * all status flags, re-enable it (and cross your fingers
       * things will work from now on.)
       *
       * There is a FLUSH bit i MCTRLB that does almost the same thing
       * but resets the peripheral into IDLE state. We want to start
       * it in an unknown state instead, giving outside world time
       * to fix things (the timeout should be long enough to only
       * expire if the bus is locked up somehow.)
       */

      /* Do we own the bus? Issue STOP condition. Or at least try,
       * we may be interfering with current operation (transmission
       * in progress) and the manual does not specify what happens.
       *
       * In other words - we need to do our best to not end up here
       * in the first place. (And use this code only as a handler
       * for some kind of "bus locked up" condition.)
       */

      if ((twi->MSTATUS & TWI_BUSSTATE_GM) == TWI_BUSSTATE_OWNER_GC)
        {
          twi->MCTRLB |= (TWI_ACKACT_NACK_GC | TWI_MCMD_STOP_GC);
        }

      /* Do the "power cycle" now. */

      twi->MCTRLA &= ~(TWI_ENABLE_bm);
      TWI_CLEAR_ALL_MSTATUS_FLAGS(twi);
      twi->MCTRLA |= TWI_ENABLE_bm;

      priv->rval = -EIO;

      goto errout_isrs;
    }

errout_isrs:
  leave_critical_section(irqflags);

  nxmutex_unlock(&(priv->lock));

errout:
  return priv->rval;
}

/****************************************************************************
 * Name: avrdx_twi_set_frequency
 *
 * Description:
 *   Set BAUD register for given SCL frequency
 *
 * Input Parameters:
 *   - TWI peripheral to configure, identified by pointer
 *     to avrdx_twi_priv_t
 *   - Requested frequency in Hertz. Values that cannot be achieved
 *     with current peripheral clock frequency are refused. So are values
 *     exceeding limit for I2C mode selected for the peripheral.
 *     On the other hand, the caller is responsible for ensuring
 *     that the t_LOW duration (minimal duration of logic 0 on SDA/SCL
 *     lines) meets specification.
 *
 * Returned Value:
 *   Zero or EINVAL for invalid frequency.
 *
 * Assumptions/Limitations:
 *   Interrupts are disabled.
 *
 ****************************************************************************/

static int avrdx_twi_set_frequency(avrdx_twi_priv_t *priv, uint32_t f_scl)
{
  avr_twi_t *twi;
  uint8_t twi_n;
  uint32_t baud;
  uint16_t temp16;
  uint32_t f_per;

#ifndef CONFIG_AVR_HAVE_TWI1
  twi_n = 0; /* Ignore parameter if the chip only has single TWI */
#else
  twi_n = priv->twi_n;
#endif

  if (f_scl > avrdx_twi_fscl_limit[twi_n])
    {
      return -EINVAL;
    }

  twi = &(AVRDX_TWI(twi_n));

  /* The BAUD formula is:
   * BAUD = f_per / ( 2 * f_scl ) - ( 5 + ( f_per * t_R ) / 2 )
   * with t_R being SDA and SCL rise time. (See Electrical Characteristics
   * in the AVR128DA28 datasheet.)
   */

  f_per = avrdx_current_freq_per();
  baud = f_per / 2 / f_scl; /* Only first half of the equation */

  /* As for the second half of the equation:
   *
   * t_R differs for different modes but at most it is 1000ns,
   * turning the multiplication to division. We will round
   * the value so the division can be replaced with bitshift.
   * For starters, we will lose lowest two bytes because the largest
   * t_R is 1000ns, meaning division by 1M. (Removing two lowest
   * bytes divides by 65536.)
   *
   * Then we do bitshift by the constant read from the array
   * to finish the job. The result will only be approximate but
   * should be sufficiently precise. Note that the bitshift
   * constant also includes the division by 2.
   *
   * Adding 5 is last.
   */

  temp16 = f_per >> 16;
  temp16 >>= avrdx_twi_tr_bitshift[twi_n];
  temp16 += 5;

  /* Does the BAUD actually remain positive? */

  if (baud <= temp16)
    {
      return -EINVAL ; /* Overflow about to happen */
    }

  baud -= temp16;

  /* The result is supposed to be within specs, there is a minimum
   * duration of logic 0 called t_LOW. According to the AVR128DA28
   * datasheet, it is calculated like this:
   *
   * t_LOW = ((BAUD + 5 ) / f_per) - t_OF
   *
   * Where t_OF is at most 250ns, 20 - 250ns or 20 - 120ns
   * for standard, fast and fast plus modes respectively.
   *
   * Length of t_LOW must be at least 4700ns, 1300ns or 500ns
   * (depending on the mode again.) If it is lower, BAUD needs
   * to be recalculated:
   *
   * BAUD = f_per * (t_LOW + t_OF) -  5
   *
   * All of this is left to the user.
   *
   * The only thing we check is if the result actually fits 8 bit
   * register MBAUD.
   */

  if (baud > 0xff)
    {
      return -EINVAL;
    }

  twi->MBAUD = baud;

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: avrdx_initialize_twi
 *
 * Description:
 *   Initializer for TWI device. Allocates data structures and configures
 *   the peripheral. May be called multiple times by multiple drivers using
 *   the bus.
 *
 * Input Parameters:
 *   Peripheral number, ignored if the chip only has single TWI
 *
 * Returned Value:
 *   Initialized structure cast to i2c_master_s
 *
 ****************************************************************************/

FAR struct i2c_master_s *avrdx_initialize_twi(uint8_t twi_n)
{
  avrdx_twi_priv_t *priv;
  avr_twi_t *twi;

#ifndef CONFIG_AVR_HAVE_TWI1
  twi_n = 0; /* Ignore parameter if the chip only has single TWI */
#endif

  if (g_twi_ports[twi_n])
    {
      /* This device is already initialized */

      return (struct i2c_master_s *) g_twi_ports[twi_n];
    }

  priv = (avrdx_twi_priv_t *) kmm_zalloc(sizeof(avrdx_twi_priv_t));
  if (!priv)
    {
      PANIC();
    }

  twi = &(AVRDX_TWI(twi_n));

  priv->ops = &g_twi_ops;
  priv->twi_n = twi_n;

  nxmutex_init(&(priv->lock));
  sem_init(&(priv->sem_isr), 0, 0);

  /* Configure port multiplexer before the peripheral is initialized
   * so it initializes with correct pins.
   */

  PORTMUX.TWIROUTEA = \
    (PORTMUX.TWIROUTEA & avrdx_twi_portmux_masks[twi_n]) | \
    avrdx_twi_portmux_bits[twi_n];

  /* Initialization - configure CTRLA first, then MBAUD, then MCTRLA.
   * Also clear status in case its needed. Note - I/O port is overridden
   * automatically when the device is enabled.
   */

  twi->CTRLA = avrdx_twi_init_ctrla[twi_n];

  /* Achievable frequency depends on current CPU clock. 50kHz should
   * work for whole range of 1-24MHz. This affects inactive bus
   * time-out supervisor which we will use to transition the internal
   * state machine from UNKNOWN bus state to IDLE.
   *
   * The timeout is configured to be 200us but that is affected
   * by baud rate, which is assumed to be 100kHz (borderline value
   * for 1MHz peripheral clock setting.) We use 50kHz instead,
   * timeout of 400us should still be enough.
   *
   * (Might actually even be better for I2C bus, we are not supposed
   * to have timeout there at all.)
   */

  avrdx_twi_set_frequency(priv, 50000);

  /* Now attach interrupt, clean status flags and activate
   * the peripheral.
   */

  irq_attach(avrdx_twi_master_interrupts[twi_n],
             avrdx_twi_interrupt,
             (void *) priv);

  TWI_CLEAR_ALL_MSTATUS_FLAGS(twi)

  twi->MCTRLA = TWI_RIEN_bm | TWI_WIEN_bm | \
    TWI_TIMEOUT_200US_GC | TWI_SMEN_bm | \
    TWI_ENABLE_bm;

  g_twi_ports[twi_n] = priv;

  return (struct i2c_master_s *) priv;
}

#endif /* defined(CONFIG_AVR_TWI0) || defined(CONFIG_AVR_TWI1) */
