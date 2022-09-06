/****************************************************************************
 * arch/z80/src/z8/z8_i2c.c
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

#include <nuttx/mutex.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <arch/board/board.h>

#include <eZ8.h>  /* eZ8 Register definitions */
#include "chip.h" /* Register bit definitions */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define Z8_NOSTOP    (1 << 0)  /* Bit 0: No STOP on this transfer */
#define Z8_NOSTART   (1 << 1)  /* Bit 1: No address or START on this transfers */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct z8_i2cdev_s
{
  const struct i2c_ops_s *ops; /* I2C vtable */
  uint32_t frequency;          /* Currently selected I2C frequency */
  uint16_t brg;                /* Baud rate generator value */
  uint8_t  addr;               /* 8-bit address */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Misc. Helpers */

static void     z8_i2c_waittxempty(void);
static void     z8_i2c_waitrxavail(void);
static void     z8_i2c_setbrg(uint16_t brg);
static uint16_t z8_i2c_getbrg(uint32_t frequency);
static int      z8_i2c_read_transfer(FAR struct z8_i2cdev_s *priv,
                  FAR uint8_t *buffer, int buflen, uint8_t flags);
static int      z8_i2c_write_transfer(FAR struct z8_i2cdev_s *priv,
                  FAR const uint8_t *buffer, int buflen, uint8_t flags);
static void     z8_i2c_setfrequency(FAR struct z8_i2cdev_s *priv,
                  uint32_t frequency);

/* I2C methods */

static int      z8_i2c_transfer(FAR struct i2c_master_s *dev,
                  FAR struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int      z8_i2c_reset(FAR struct i2c_master_s *dev);
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* This function is normally prototyped int the ZiLOG header file sio.h */

extern uint32_t get_freq(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool    g_initialized;  /* true:I2C has been initialized */
static mutex_t g_i2clock;      /* Serialize I2C transfers */

const struct i2c_ops_s g_ops =
{
  z8_i2c_transfer,
#ifdef CONFIG_I2C_RESET
  , z8_i2c_reset
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: z8_i2c_waittxempty
 *
 * Description:
 *   Wait for the transmit data register to become empty.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void z8_i2c_waittxempty(void)
{
  int i;
  for (i = 0; i < 10000 && (I2CSTAT & I2C_STAT_TDRE) == 0;  i++);
}

/****************************************************************************
 * Name: z8_i2c_waitrxavail
 *
 * Description:
 *   Wait until we have received a full byte of data.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void z8_i2c_waitrxavail(void)
{
  int i;

  for (i = 0;
       i <= 10000 && (I2CSTAT & (I2C_STAT_RDRF | I2C_STAT_NCKI)) == 0;
       i++)
    {
    }
}

/****************************************************************************
 * Name: z8_i2c_setbrg
 *
 * Description:
 *   Set the current BRG value for this transaction
 *
 * Input Parameters:
 *   brg - BRG to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void z8_i2c_setbrg(uint16_t brg)
{
  I2CBRH = (uint8_t)(brg >> 8);
  I2CBRL = (uint8_t)(brg & 0xff);
}

/****************************************************************************
 * Name: z8_i2c_getbrg
 *
 * Description:
 *   Calculate the BRG value
 *
 * Input Parameters:
 *   frequency - The I2C frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

static uint16_t z8_i2c_getbrg(uint32_t frequency)
{
  uint32_t sysclock = get_freq();

  /* Max is 400 Kb/sec */

  if (frequency > 400 * 1000)
    {
      _err("ERROR: Invalid inputs\n");
      frequency = 400 * 1000;
    }

  /* BRG = sysclock / (4 * frequency) */

  return ((sysclock >> 2) + (frequency >> 1)) / frequency;
}

/****************************************************************************
 * Name: z8_i2c_read_transfer
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

static int z8_i2c_read_transfer(FAR struct z8_i2cdev_s *priv,
                                FAR uint8_t *buffer, int buflen,
                                uint8_t flags)
{
  FAR uint8_t *ptr;
  int retry;
  int count;

  /* Retry as necessary to receive the whole message */

  for (retry = 0; retry < 100; retry++)
    {
      if ((flags & Z8_NOSTART) == 0)
        {
          /* Load the address into the transmit register.  It is not sent
           * until the START bit is set.
           */

          I2CD = I2C_READADDR8(priv->addr);

          /* If we want only a single byte of data, then set the NACK
           * bit now.
           */

          I2CCTL |= I2C_CTL_NAK;

          /* The START bit begins the transaction */

          I2CCTL |= I2C_CTL_START;
        }

      /* Now loop to receive each data byte */

      ptr = buffer;
      for (count = buflen; count; count--)
        {
          /* Wait for the receive buffer to fill */

          z8_i2c_waitrxavail();

          /* Did we get a byte?  Or did an error occur? */

          if (I2CSTAT & I2C_STAT_RDRF)
            {
              /* Save the data byte */

              *ptr++ = I2CD;

              /* If the next byte is the last byte, then set NAK now */

              if (count == 2 && (flags & Z8_NOSTOP) == 0)
                {
                  I2CCTL |= I2C_CTL_NAK;
                }

              /* If this was the last byte, then set STOP and return
               * success
               */

              else if (count == 1)
                {
                  if ((flags & Z8_NOSTOP) == 0)
                    {
                      I2CCTL |= I2C_CTL_STOP;
                    }

                  return OK;
                }
            }

          /* An error occurred.  Clear byte bus and break out of the loop
           * to retry now.
           */

          else
            {
              /* No, flush the buffer and toggle the I2C on and off */

              I2CCTL |= I2C_CTL_FLUSH;
              I2CCTL &= ~I2C_CTL_IEN;
              I2CCTL |= I2C_CTL_IEN;

              /* Break out of the loop early and try again */

              break;
            }
        }
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: z8_i2c_write_transfer
 *
 * Description:
 *   Send a block of data on I2C using the previously selected I2C
 *   frequency and slave address. Each write operational will be an 'atomic'
 *   operation in the sense that any other I2C actions will be serialized
 *   and pend until this write completes. Required.
 *
 * Input Parameters:
 *   dev -    Device-specific state data
 *   buffer - A pointer to the read-only buffer of data to be written to
 *            device
 *   buflen - The number of bytes to send from the buffer
 *   flags  - Determines is a START and/or STOP indication is needed.
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

static int z8_i2c_write_transfer(FAR struct z8_i2cdev_s *priv,
                                 FAR const uint8_t *buffer, int buflen,
                                 uint8_t flags)
{
  FAR const uint8_t *ptr;
  int retry;
  int count;

  /* Retry as necessary to send this whole message */

  for (retry = 0; retry < 100; retry++)
    {
      if ((flags & Z8_NOSTART) == 0)
        {
          /* Load the address into the transmit register.  It is not sent
           * until the START bit is set.
           */

          I2CD    = I2C_WRITEADDR8(priv->addr);
          I2CCTL |= I2C_CTL_START;

          /* Wait for the xmt buffer to become empty */

          z8_i2c_waittxempty();
        }

      /* Then send all of the bytes in the buffer */

      ptr = buffer;
      for (count = buflen; count; count--)
        {
          /* Send a byte of data and wait for it to be sent */

          I2CD = *ptr++;
          z8_i2c_waittxempty();

          /* If this was the last byte, then send STOP immediately.  This
           * is because the ACK will not be valid until the STOP clocks out
           * the last bit.. Hmmm.  If this true then we will never be
           * able to send more than one data byte???
           */

          if (count == 1)
            {
              if ((flags & Z8_NOSTOP) == 0)
                {
                  I2CCTL |= I2C_CTL_STOP;
                }

              /* If this last byte was ACKed, then the whole buffer
               * was successfully sent and we can return success.
               */

              if ((I2CSTAT & I2C_STAT_ACK) != 0)
                {
                  return OK;
                }

              /* If was was not ACKed, then this inner loop will
               * terminated (because count will decrement to zero
               * and the whole message will be resent
               */
            }

          /* Not the last byte... was this byte ACKed? */

          else if ((I2CSTAT & I2C_STAT_ACK) == 0)
            {
              /* No, flush the buffer and toggle the I2C on and off */

              I2CCTL |= I2C_CTL_FLUSH;
              I2CCTL &= ~I2C_CTL_IEN;
              I2CCTL |= I2C_CTL_IEN;

              /* Break out of the loop early and try again */

              break;
            }
        }
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: z8_i2c_setfrequency
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

static void z8_i2c_setfrequency(FAR struct z8_i2cdev_s *priv,
                                uint32_t frequency)
{
  uint16_t brg;

  /* Has the frequency changed? */

  if (priv->frequency != frequency)
    {
      /* Calculate and save the BRG (we won't apply it until the first
       * transfer)
       */

      brg = z8_i2c_getbrg(frequency);
      z8_i2c_setbrg(brg);

      /* Save the new I2C frequency */

      priv->frequency = frequency;
    }
}

/****************************************************************************
 * Name: z8_i2c_transfer
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

static int z8_i2c_transfer(FAR struct i2c_master_s *dev,
                             FAR struct i2c_msg_s *msgs, int count)
{
  FAR struct z8_i2cdev_s *priv = (FAR struct z8_i2cdev_s *)dev;
  FAR struct i2c_msg_s *msg;
  bool nostop;
  uint8_t flags;
  int ret;
  int i;

  /* Perform each segment of the transfer, message at a time */

  flags = 0;

  /* Get exclusive access to the I2C bus */

  ret = nxmutex_lock(&g_i2clock);
  if (ret < 0)
    {
      return ret;
    }

  /* The process each message segment */

  for (i = 0; i < count; i++)
    {
      msg = &msgs[i];

      /* Set the I2C frequency and address */

      z8_i2c_setfrequency(priv, msg->frequency);

      priv->addr = msg->addr;
      DEBUGASSERT((msg->flags & I2C_M_TEN) == 0);

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

      flags |= (nostop) ? Z8_NOSTOP : 0;
      if ((msg->flags & I2C_M_READ) != 0)
        {
          ret = z8_i2c_read_transfer(priv, msg->buffer, msg->length, flags);
        }
      else
        {
          ret = z8_i2c_write_transfer(priv, msg->buffer, msg->length, flags);
        }

      /* Check for I2C transfer errors */

      if (ret < 0)
        {
          break;
        }

      /* If there was no STOP bit on this segment, then there should be no
       * START on the next segment.
       */

      flags = (nostop) ? Z8_NOSTART : 0;
    }

  nxmutex_unlock(&g_i2clock);
  return ret;
}

/****************************************************************************
 * Name: z8_i2c_reset
 *
 * Description:
 *   Perform an I2C bus reset in an attempt to break loose stuck I2C devices.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_RESET
static int z8_i2c_reset(FAR struct i2c_master_s * dev)
{
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: z8_i2cbus_initialize
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

FAR struct i2c_master_s *z8_i2cbus_initialize(int port)
{
  FAR struct z8_i2cdev_s *i2c;

  if (!g_initialized)
    {
      /* Set up some initial BRG value */

      uint16_t brg = z8_i2c_getbrg(100 * 1000);
      z8_i2c_setbrg(brg);

      /* Make sure that GPIOs are configured for the alternate function (this
       * varies with silicon revisions).
       */

      PAADDR = 0x02;
      PACTL |= 0xc0;

      /* This mutex enforces serialized access for I2C transfers */

      nxmutex_init(&g_i2clock);

      /* Enable I2C -- no interrupts */

      I2CCTL = I2C_CTL_IEN;
    }

  /* Now, allocate an I2C instance for this caller */

  i2c = (FAR struct z8_i2cdev_s *)kmm_zalloc(sizeof(FAR struct z8_i2cdev_s));
  if (i2c)
    {
      /* Initialize the allocated instance */

      i2c->ops = &g_ops;
    }

  return (FAR struct i2c_master_s *)i2c;
}
