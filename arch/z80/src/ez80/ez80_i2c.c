/****************************************************************************
 * arch/z80/src/ez80/ez80_i2c.c
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
#include <stdbool.h>
#include <stdlib.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/semaphore.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <arch/io.h>
#include <arch/board/board.h>

#include "ez80f91.h"
#include "ez80_i2c.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define EZ80_NOSTOP    (1 << 0)   /* Bit 0: No STOP on this transfer */
#define EZ80_NOSTART   (1 << 1)   /* Bit 1: No address or START on this transfers */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ez80_i2cdev_s
{
  FAR const struct i2c_ops_s *ops; /* I2C vtable */
  uint32_t frequency;              /* Currently selected I2C frequency */
  uint16_t addr;                   /* 7- or 10-bit address */
  uint8_t  addr10 : 1;             /* 1=Address is 10-bit */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Misc. Helpers */

static void     ez80_i2c_setccr(uint16_t ccr);
static uint16_t ez80_i2c_getccr(uint32_t frequency);
static uint8_t  ez80_i2c_waitiflg(void);
static void     ez80_i2c_clriflg(void);
static void     ez80_i2c_start(void);
static void     ez80_i2c_stop(void);
static int      ez80_i2c_sendaddr(struct ez80_i2cdev_s *priv,
                  uint8_t readbit);
static int      ez80_i2c_read_transfer(FAR struct ez80_i2cdev_s *priv,
                  FAR uint8_t *buffer, int buflen, uint8_t flags);
static int      ez80_i2c_write_transfer(FAR struct ez80_i2cdev_s *priv,
                  FAR const uint8_t *buffer, int buflen, uint8_t flags);
static void     ez80_i2c_setfrequency(FAR struct ez80_i2cdev_s *priv,
                  uint32_t frequency);

/* I2C methods */

static int      ez80_i2c_transfer(FAR struct i2c_master_s *dev,
                  FAR struct i2c_msg_s *msgs, int count);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool  g_initialized;  /* true:I2C has been initialized */
static sem_t g_i2csem;       /* Serialize I2C transfers */

const struct i2c_ops_s g_ops =
{
  ez80_i2c_transfer
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ez80_i2c_semtake/ez80_i2c_semgive
 *
 * Description:
 *   Take/Give the I2C semaphore.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int ez80_i2c_semtake(void)
{
  return nxsem_wait(&g_i2csem);
}

/****************************************************************************
 * Name: ez80_i2c_setccr
 *
 * Description:
 *   Set the current BRG value for this transaction
 *
 * Input Parameters:
 *   ccr - BRG to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void ez80_i2c_setccr(uint16_t ccr)
{
  outp(EZ80_I2C_CCR, ccr);
}

/****************************************************************************
 * Name: ez80_i2c_getccr
 *
 * Description:
 *   Calculate the BRG value
 *
 * Input Parameters:
 *   fscl - The I2C frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

static uint16_t ez80_i2c_getccr(uint32_t fscl)
{
  uint32_t fsamp;
  uint32_t ftmp;
  uint8_t  n;
  uint8_t  m;

  /* The sampling frequency is given by:
   *
   *   fsamp = sysclock / 2**N
   *
   * And the I2C clock is determined by:
   *
   *   fscl = sysclock / 10 / (M + 1) / 2**N
   *        = fsamp / 10 / (M + 1)
   *
   * The fsmp must be >= 10 * fscl.  The best solution is the smallest value
   * of N so that the sampling rate is the highest subject to:
   *
   * The minimum value of the fsamp is given by:
   */

  fsamp = 10 * fscl;

  /* Now, serarch for the smallest value of N that results in the actual
   * fsamp >= the ideal fsamp.  Fortunately, we only have to check at most
   * eight values.
   */

  if (fsamp >= EZ80_SYS_CLK_FREQ)
    {
      ftmp = EZ80_SYS_CLK_FREQ / 10;
      n    = 0;
    }
  else if (fsamp >= (EZ80_SYS_CLK_FREQ >> 1))
    {
      ftmp = (EZ80_SYS_CLK_FREQ >> 1) / 10;
      n    = 1;
    }
  else if (fsamp >= (EZ80_SYS_CLK_FREQ >> 2))
    {
      ftmp = (EZ80_SYS_CLK_FREQ >> 2) / 10;
      n    = 2;
    }
  else if (fsamp >= (EZ80_SYS_CLK_FREQ >> 3))
    {
      ftmp = (EZ80_SYS_CLK_FREQ >> 3) / 10;
      n    = 3;
    }
  else if (fsamp >= (EZ80_SYS_CLK_FREQ >> 4))
    {
      ftmp = (EZ80_SYS_CLK_FREQ >> 4) / 10;
      n     = 4;
    }
  else if (fsamp >= (EZ80_SYS_CLK_FREQ >> 5))
    {
      ftmp = (EZ80_SYS_CLK_FREQ >> 5) / 10;
      n    = 5;
    }
  else if (fsamp >= (EZ80_SYS_CLK_FREQ >> 6))
    {
      ftmp = (EZ80_SYS_CLK_FREQ >> 6) / 10;
      n     = 6;
    }
  else if (fsamp >= (EZ80_SYS_CLK_FREQ >> 7))
    {
      ftmp  = (EZ80_SYS_CLK_FREQ >> 7) / 10;
      n     = 7;
    }
  else
    {
      ftmp  = (EZ80_SYS_CLK_FREQ >> 7) / 10;
      fscl  = ftmp;
      n     = 7;
    }

  /* Finally, get M:
   *
   *   M = (fsamp / 10) / fscl - 1 = ftmp / fscl - 1
   */

  m = ftmp / fscl;
  if (m > 0)
    {
      if (--m > 15)
        {
          m = 15;
        }
    }

  /* Return the value for CCR */

  return (n << I2C_CCR_NSHIFT) | (m << I2C_CCR_MSHIFT);
}

/****************************************************************************
 * Name: ez80_i2c_waitiflg
 *
 * Description:
 *   In polled mode, we have to spin until the IFLG bit in the I2C_CTL
 *   register goes to 1, signaling that the last send or receive is
 *   complete.  This could be used to generate an interrupt for a non-
 *   polled driver.
 *
 * Input Parameters:
 *   priv -    Device-specific state data
 *   readbit - 0 or I2C_READBIT
 *
 * Returned Value:
 *   The contents of the I2C_SR register at the time that IFLG became 1.
 *
 ****************************************************************************/

static uint8_t ez80_i2c_waitiflg(void)
{
  while ((inp(EZ80_I2C_CTL) & I2C_CTL_IFLG) != 0);
  return inp(EZ80_I2C_SR);
}

/****************************************************************************
 * Name: ez80_i2c_clriflg
 *
 * Description:
 *   Clear the IFLAG bit in the I2C_CTL register, acknowledging the event.
 *   If interrupts are enabled, this would clear the interrupt status.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void ez80_i2c_clriflg(void)
{
  uint8_t regval = inp(EZ80_I2C_CTL);
  regval &= ~I2C_CTL_IFLG;
  outp(EZ80_I2C_CTL, regval);
}

/****************************************************************************
 * Name: ez80_i2c_start
 *
 * Description:
 *   Send the START bit.  IFLAG must be zero; it will go to 1 when it is
 *   time to send the address.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void ez80_i2c_start(void)
{
  uint8_t regval  = inp(EZ80_I2C_CTL);
  regval |= I2C_CTL_STA;
  outp(EZ80_I2C_CTL, regval);
}

/****************************************************************************
 * Name: ez80_i2c_stop
 *
 * Description:
 *   Send the STOP bit.  This terminates the I2C transfer and reverts back
 *   to IDLE mode.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void ez80_i2c_stop(void)
{
  uint8_t regval  = inp(EZ80_I2C_CTL);
  regval |= I2C_CTL_STP;
  outp(EZ80_I2C_CTL, regval);
}

/****************************************************************************
 * Name: ez80_i2c_sendaddr
 *
 * Description:
 *   Send the 8- or 11-bit address for either a read or a write transaction.
 *
 * Input Parameters:
 *   priv    - Device-specific state data
 *   readbit - 0 or I2C_READBIT
 *
 * Returned Value:
 *   0: Success, IFLG is set and DATA can be sent or received.

 *   Or <0: Negated error value.  IFLG is cleared.
 *
 *   -EIO: Irrecoverable (or unexpected) error occurred
 *   -EAGAIN: And
 *
 ****************************************************************************/

static int ez80_i2c_sendaddr(struct ez80_i2cdev_s *priv, uint8_t readbit)
{
  uint8_t sr;

  /* Wait for the IFLG bit to transition to 1.  At this point, we should
   * have status == 8 meaning that the start bit was sent successfully.
   */

  sr = ez80_i2c_waitiflg();
#ifdef CONFIG_DEBUG_FEATURES
  if (sr != I2C_SR_MSTART)
    {
      /* This error should never occur */

      i2cerr("ERROR: Bad START status: %02x\n", sr);
      ez80_i2c_clriflg();
      return -EIO;
    }
#endif

  /* Now send the address */

  if (!priv->addr10)
    {
      /* Load the I2C_DR with the 8-bit I2C slave address and clear the
       * IFLG.  Clearing the IFLAG will cause the address to be transferred.
       */

      outp(EZ80_I2C_DR, (uint8_t)I2C_ADDR8(priv->addr) | readbit);
      ez80_i2c_clriflg();

      /* And wait for the address transfer to complete */

      sr = ez80_i2c_waitiflg();
      if (sr != I2C_SR_MADDRWRACK && sr != I2C_SR_MADDRWR)
        {
          i2cerr("ERROR: Bad ADDR8 status: %02x\n", sr);
          goto failure;
        }
    }
  else
    {
      /* Load the I2C_DR with upper part of the 10->16-bit I2C slave address
       * and clear the IFLG.  Clearing the IFLAG will cause the address to
       * be transferred.
       */

      outp(EZ80_I2C_DR, (uint8_t)I2C_ADDR10H(priv->addr) | readbit);
      ez80_i2c_clriflg();

      /* And wait for the address transfer to complete */

      sr = ez80_i2c_waitiflg();
      if (sr != I2C_SR_MADDRWRACK && sr != I2C_SR_MADDRWR)
        {
          i2cerr("ERROR: Bad ADDR10H status: %02x\n", sr);
          goto failure;
        }

      /* Now send the lower 8 bits of the 10-bit address */

      outp(EZ80_I2C_DR, (uint8_t)I2C_ADDR10L(priv->addr));
      ez80_i2c_clriflg();

      /* And wait for the address transfer to complete */

      sr = ez80_i2c_waitiflg();
      if (sr != I2C_SR_MADDR2WRACK && sr != I2C_SR_MADDR2WR)
        {
          i2cerr("ERROR: Bad ADDR10L status: %02x\n", sr);
          goto failure;
        }
    }

  return OK;

  /* We don't attempt any fancy status-based error recovery */

failure:
#ifdef CONFIG_DEBUG_FEATURES
  switch (sr)
    {
      case I2C_SR_ARBLOST1: /* Arbitration lost in address or data byte */
      case I2C_SR_ARBLOST2: /* Arbitration lost in address as master, slave
                             * address and Write bit received, ACK
                             * transmitted */
      case I2C_SR_ARBLOST3: /* Arbitration lost in address as master, General
                             * Call address received, ACK transmitted */
      case I2C_SR_ARBLOST4: /* Arbitration lost in address as master, slave
                             * address and Read bit received, ACK transmitted */
        i2cerr("ERROR: Arbitration lost: %02x\n", sr);
        ez80_i2c_clriflg();
        return -EAGAIN;

      default:
        i2cerr("ERROR: Unexpected status: %02x\n", sr);
        ez80_i2c_clriflg();
        return -EIO;
    }
#else
  ez80_i2c_clriflg();
  return -EAGAIN;
#endif
}

/****************************************************************************
 * Name: ez80_i2c_read_transfer
 *
 * Description:
 *   Receive a block of data from I2C using the previously selected I2C
 *   frequency and slave address. Each read operational will be an 'atomic'
 *   operation in the sense that any other I2C actions will be serialized
 *   and pend until this read completes. Required.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   buffer - A pointer to a buffer of data to receive the data from the
 *            device
 *   buflen - The requested number of bytes to be read
 *   flags  - Determines is a START and/or STOP indication is needed.
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

static int ez80_i2c_read_transfer(FAR struct ez80_i2cdev_s *priv,
                                  FAR uint8_t *buffer, int buflen,
                                  uint8_t flags)
{
  FAR uint8_t *ptr;
  uint8_t regval;
  int retry;
  int count;
  int ret;

  /* Retry as necessary to receive the whole message */

  for (retry = 0; retry < 100; retry++)
    {
      if ((flags & EZ80_NOSTART) == 0)
        {
          /* Enter MASTER TRANSMIT mode by setting the STA bit in the
           * I2C_CTL register to 1. The I2C then tests the I2C bus and
           * transmits a START condition when the bus is free.
           */

          ez80_i2c_start();

          /* When a START condition is transmitted, the IFLG bit is 1.
           * Then we may send the I2C slave address.
           */

          ret = ez80_i2c_sendaddr(priv, 0);
          if (ret < 0)
            {
              if (ret == -EAGAIN)
                {
                  continue;
                }
              else
                {
                  return ret;
                }
            }
        }

      /* Now loop to receive each data byte */

      ptr = buffer;
      for (count = buflen; count; count--)
        {
          /* Is this the last byte? If so, we must NACK it */

          regval  = inp(EZ80_I2C_CTL);
          if (count <= 1)
            {
              /* If the AAK bit is cleared to 0 during a transfer, the I2C
               * transmits a NACK bit after the next byte is received.
               */

              regval &= ~I2C_CTL_AAK;
            }
          else
            {
              /* If the AAK bit in the I2C_CTL register is set to 1 then an
               * ACK bit is transmitted and the IFLG bit is set after each
               * byte is received.
               */

              regval |= I2C_CTL_AAK;
            }

          outp(EZ80_I2C_CTL, regval);

          /* Wait for IFLG to be set meaning that incoming data is
           * available in the I2C_DR registers.
           */

          regval = ez80_i2c_waitiflg();

          /* Data byte received in MASTER mode, ACK transmitted */

          if (regval == I2C_SR_MDATARDACK)
            {
              /* Since we just ACKed the incoming byte, it must NOT be the
               * last
               */

              DEBUGASSERT(count > 1);

              /* Receive the data and clear the IFLGS */

              *ptr++ = inp(EZ80_I2C_DR);
              ez80_i2c_clriflg();
            }

          /* Data byte received in MASTER mode, NACK transmitted */

          else if (regval == I2C_SR_MDATARDNAK)
            {
              /* Since we just NACKed the incoming byte, it must be the last */

              DEBUGASSERT(count <= 1);

              if ((flags & EZ80_NOSTOP) == 0)
                {
                  /* When all bytes are received and the NACK has been sent,
                   * then the microcontroller must write 1 to the STP bit in
                   * the I2C_CTL register. The I2C then transmits a STOP
                   * condition, clears the STP bit and returns to an idle
                   * state.
                   */

                  ez80_i2c_stop();
                }

              ez80_i2c_clriflg();
              return OK;
            }

          /* Arbitration lost in address or data byte */

          else if (regval == I2C_SR_ARBLOST1)
            {
              /* Clear the IFLG and break out of the inner loop.
               * this will cause the whole transfer to start over
               */

              i2cerr("ERROR: Arbitration lost: %02x\n", regval);
              ez80_i2c_clriflg();
              break;
            }

          /* Unexpected status response */

          else
            {
              i2cerr("ERROR: Unexpected status: %02x\n", regval);
              ez80_i2c_clriflg();
              return -EIO;
            }
        }
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: ez80_i2c_write_transfer
 *
 * Description:
 *   Send a block of data on I2C using the previously selected I2C
 *   frequency and slave address. Each write operational will be an 'atomic'
 *   operation in the sense that any other I2C actions will be serialized
 *   and pend until this write completes. Required.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   buffer - A pointer to the read-only buffer of data to be written to
 *            device
 *   buflen - The number of bytes to send from the buffer
 *   flags  - Determines is a START and/or STOP indication is needed.
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

static int ez80_i2c_write_transfer(FAR struct ez80_i2cdev_s *priv,
                                   FAR const uint8_t *buffer, int buflen,
                                   uint8_t flags)
{
  FAR const uint8_t *ptr;
  uint8_t sr;
  int retry;
  int count;
  int ret;

  /* Retry as necessary to send this whole message */

  for (retry = 0; retry < 100; retry++)
    {
      if ((flags & EZ80_NOSTART) == 0)
        {
          /* Enter MASTER TRANSMIT mode by setting the STA bit in the
           * I2C_CTL register to 1. The I2C then tests the I2C bus and
           * transmits a START condition when the bus is free.
           */

          ez80_i2c_start();

          /* When a START condition is transmitted, the IFLG bit is 1.  Then
           * we may send the I2C slave address.
           */

          ret = ez80_i2c_sendaddr(priv, 0);
          if (ret < 0)
            {
              if (ret == -EAGAIN)
                {
                  continue;
                }
              else
                {
                  return ret;
                }
            }
        }

      /* Then send all of the bytes in the buffer */

      ptr = buffer;
      for (count = buflen; count; count--)
        {
          /* Load the I2C_DR with next data byte and clear the IFLG.
           * Clearing the IFLAG will cause the data to be transferred.
           */

          outp(EZ80_I2C_DR, *ptr++);
          ez80_i2c_clriflg();

          /* And wait for the data transfer to complete */

          sr = ez80_i2c_waitiflg();
          if (sr != I2C_SR_MDATAWRACK && sr != I2C_SR_MDATAWR)
            {
              i2cerr("ERROR: Bad DATA status: %02x\n", sr);
              ez80_i2c_clriflg();
              if (sr == I2C_SR_ARBLOST1)
                {
                  /* Arbitration lost, break out of the inner loop and
                   * try sending the message again
                   */

                  break;
                }

              /* Otherwise, it is fatal (shouldn't happen) */

              return -EIO;
            }

          /* Data byte was sent successfully.  Was that the last byte? */

          else if (count <= 1)
            {
              if ((flags & EZ80_NOSTOP) == 0)
                {
                  /* When all bytes are transmitted, the microcontroller
                   * must write a 1 to the STP bit in the I2C_CTL register.
                   * The I2C then transmits a STOP condition, clears the
                   * STP bit and returns to an idle state.
                   */

                  ez80_i2c_stop();
                }

              return OK;
            }
        }
    }

  /* If we get here, we timed out without successfully sending the message */

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: ez80_i2c_setfrequency
 *
 * Description:
 *   Set the I2C frequency. This frequency will be retained in the struct
 *   i2c_master_s instance and will be used with all transfers.  Required.
 *
 * Input Parameters:
 *   dev -       Device-specific state data
 *   frequency - The I2C frequency requested
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void ez80_i2c_setfrequency(FAR struct ez80_i2cdev_s *priv,
                                  uint32_t frequency)
{
  uint16_t ccr;

  if (priv->frequency != frequency)
    {
      /* Calculate and save the BRG and set the CCR */

      ccr = ez80_i2c_getccr(frequency);
      ez80_i2c_setccr(ccr);

      /* Save the new I2C frequency */

      priv->frequency = frequency;
    }
}

/****************************************************************************
 * Name: ez80_i2c_transfer
 *
 * Description:
 *   Perform a sequence of I2C transfers, each transfer is started with a
 *   START and the final transfer is completed with a STOP. Each sequence
 *   will be an 'atomic'  operation in the sense that any other I2C actions
 *   will be serialized and pend until this read completes. Optional.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   msgs     - A pointer to a set of message descriptors
 *   msgcount - The number of transfers to perform
 *
 * Returned Value:
 *   The number of transfers completed
 *
 ****************************************************************************/

static int ez80_i2c_transfer(FAR struct i2c_master_s *dev,
                             FAR struct i2c_msg_s *msgs, int count)
{
  FAR struct ez80_i2cdev_s *priv = (FAR struct ez80_i2cdev_s *)dev;
  FAR struct i2c_msg_s *msg;
  bool nostop;
  uint8_t flags;
  int ret;
  int i;

  /* Perform each segment of the transfer, message at a time */

  flags = 0;

  /* Get exclusive access to the I2C bus */

  ret = nxsem_wait(&g_i2csem);
  if (ret < 0)
    {
      return ret;
    }

  /* The process each message segment */

  for (i = 0; i < count; i++)
    {
      msg = &msgs[i];

      /* Set the I2C frequency and address */

      ez80_i2c_setfrequency(priv, msg->frequency);

      priv->addr   = msg->addr;
      priv->addr10 = ((msg->flags & I2C_M_TEN) != 0);

      /* Is this the last message in the sequence? */

      nostop = false;
      if (i < (count - 1))
        {
          FAR struct i2c_msg_s *next;

          /* No... Check if the next message should have a repeated start or
           * not.  The conditions for NO repeated start are:
           *
           *   - I2C_M_NOSTART bit set
           *   - Same direction (I2C_M_READ)
           *   - Same address (and I2C_M_TEN)
           */

          next = &msgs[i + 1];
          if ((msg->flags & I2C_M_NOSTART) != 0 &&
              (msg->flags & (I2C_M_READ | I2C_M_TEN)) ==
              (next->flags & (I2C_M_READ | I2C_M_TEN)) &&
              msg->addr == next->addr)
            {
              nostop = true;
            }
        }

      /* Perform the read or write operation */

      flags |= (nostop) ? EZ80_NOSTOP : 0;
      if ((msg->flags & I2C_M_READ) != 0)
        {
          ret = ez80_i2c_read_transfer(priv, msg->buffer, msg->length,
                                       flags);
        }
      else
        {
          ret = ez80_i2c_write_transfer(priv, msg->buffer, msg->length,
                                        flags);
        }

      /* Check for I2C transfer errors */

      if (ret < 0)
        {
          break;
        }

      /* If there was no STOP bit on this segment, then there should be no
       * START on the next segment.
       */

      flags = (nostop) ? EZ80_NOSTART : 0;
    }

  nxsem_post(&g_i2csem);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ez80_i2cbus_initialize
 *
 * Description:
 *   Initialize the selected I2C port. And return a unique instance of struct
 *   struct i2c_master_s.  This function may be called to obtain multiple
 *   instances of the interface, each of which may be set up with a
 *   different frequency and slave address.
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple I2C interfaces)
 *
 * Returned Value:
 *   Valid I2C device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct i2c_master_s *ez80_i2cbus_initialize(int port)
{
  FAR struct ez80_i2cdev_s *i2c;
  uint16_t ccr;
  uint8_t  regval;

  if (!g_initialized)
    {
      /* Set up some initial BRG value */

      ccr = ez80_i2c_getccr(100 * 1000);
      ez80_i2c_setccr(ccr);

      /* No GPIO setup is required -- I2C pints, SCL/SDA are not multiplexed */

      /* This semaphore enforces serialized access for I2C transfers */

      nxsem_init(&g_i2csem, 0, 1);

      /* Enable I2C -- but not interrupts */

      regval  = inp(EZ80_I2C_CTL);
      regval |= I2C_CTL_ENAB;
      outp(EZ80_I2C_CTL, regval);
    }

  /* Now, allocate an I2C instance for this caller */

  i2c = (FAR struct ez80_i2cdev_s *)
    kmm_zalloc(sizeof(FAR struct ez80_i2cdev_s));
  if (i2c != NULL)
    {
      /* Initialize the allocated instance */

      i2c->ops = &g_ops;
    }

  return (FAR struct i2c_master_s *)i2c;
}
