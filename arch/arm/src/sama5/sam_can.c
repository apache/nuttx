/****************************************************************************
 * arch/arm/src/sama5/sam_can.c
 *
 *   Copyright (C) 2013-2014, 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * The Atmel sample code has a BSD compatible license that requires this
 * copyright notice:
 *
 *   Copyright (c) 2012, Atmel Corporation
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
 * 3. Neither the name NuttX, Atmel, nor the names of its contributors may
 *    be used to endorse or promote products derived from this software
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

/* References:
 *
 *   SAMA5D3 Series Data Sheet
 *   Atmel NoOS sample code (for bit timing configuration).
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <nuttx/can/can.h>
#include <nuttx/mutex.h>

#include "arm_internal.h"
#include "hardware/sam_pinmap.h"
#include "sam_periphclks.h"
#include "sam_pio.h"
#include "sam_can.h"

#if defined(CONFIG_CAN) && (defined(CONFIG_SAMA5_CAN0) || \
    defined(CONFIG_SAMA5_CAN1))

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Common definitions *******************************************************/

#ifndef MIN
#  define MIN(a,b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#  define MAX(a,b) (((a) > (b)) ? (a) : (b))
#endif

/* Mailboxes ****************************************************************/

#define SAMA5_CAN_NRECVMB MAX(CONFIG_SAMA5_CAN0_NRECVMB, CONFIG_SAMA5_CAN1_NRECVMB)

/* The set of all mailboxes */

#if SAM_CAN_NMAILBOXES == 8
#  define CAN_ALL_MAILBOXES 0xff /* 8 mailboxes */
#else
#  error Unsupported/undefined number of mailboxes
#endif

/* Interrupts ***************************************************************/

/* If debug is enabled, then print some diagnostic info if any of these
 * events occur:
 *
 * CAN_INT_ERRA      YES    Bit 16: Error Active Mode
 * CAN_INT_WARN      YES    Bit 17: Warning Limit
 * CAN_INT_ERRP      NO     Bit 18: Error Passive Mode
 * CAN_INT_BOFF      NO     Bit 19: Bus Off Mode
 *
 * CAN_INT_SLEEP     NO     Bit 20: CAN Controller in Low-power Mode
 * CAN_INT_WAKEUP    NO     Bit 21: Wake-up Interrupt
 * CAN_INT_TOVF      NO     Bit 22: Timer Overflow
 * CAN_INT_TSTP      NO     Bit 23: Timestamp
 *
 * CAN_INT_CERR      YES    Bit 24: Mailbox CRC Error
 * CAN_INT_SERR      YES    Bit 25: Mailbox Stuffing Error
 * CAN_INT_AERR      NO     Bit 26: Acknowledgment Error (usally means no
 *                                  CAN bus)
 * CAN_INT_FERR      YES    Bit 27: Form Error
 *
 * CAN_INT_BERR      YES    Bit 28: Bit Error
 */

#define CAN_DEBUG_INTS (CAN_INT_ERRA | CAN_INT_WARN | CAN_INT_CERR | \
                        CAN_INT_SERR | CAN_INT_FERR | CAN_INT_BERR)

#ifndef CONFIG_DEBUG_CAN_INFO
#  undef CONFIG_SAMA5_CAN_REGDEBUG
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes receive mailbox filtering */

struct sam_filter_s
{
#ifdef CONFIG_CAN_EXTID
  uint32_t addr;            /* 29-bit address to match */
  uint32_t mask;            /* 29-bit address mask */
#else
  uint16_t addr;            /* 11-bit address to match */
  uint16_t mask;            /* 11-bit address mask */
#endif
};

/* This structure provides the constant configuration of a CAN peripheral */

struct sam_config_s
{
  uint8_t port;             /* CAN port number (1 or 2) */
  uint8_t pid;              /* CAN periperal ID/IRQ number */
  uint8_t nrecvmb;          /* Number of receive mailboxes */
  uintptr_t base;           /* Base address of the CAN control registers */
  uint32_t baud;            /* Configured baud */
  pio_pinset_t rxpinset;    /* RX pin configuration */
  pio_pinset_t txpinset;    /* TX pin configuration */

  /* Mailbox filters */

  struct sam_filter_s filter[SAMA5_CAN_NRECVMB];
};

/* This structure provides the current state of a CAN peripheral */

struct sam_can_s
{
  /* The constant configuration */

  const struct sam_config_s *config;

  bool initialized;         /* TRUE: Device has been initialized */
  uint8_t freemb;           /* Rhe set of unalloated mailboxes */
  uint8_t rxmbset;          /* The set of mailboxes configured for receive */
  volatile uint8_t txmbset; /* The set of mailboxes actively transmitting */
  bool  txdisabled;         /* TRUE:  Keep TX interrupts disabled */
  mutex_t lock;             /* Enforces mutually exclusive access */
  uint32_t frequency;       /* CAN clock frequency */

#ifdef CONFIG_SAMA5_CAN_REGDEBUG
  uintptr_t regaddr;        /* Last register address read */
  uint32_t regval;          /* Last value read from the register */
  unsigned int count;       /* Number of times that the value was read */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* CAN Register access */

static uint32_t can_getreg(struct sam_can_s *priv, int offset);
static void can_putreg(struct sam_can_s *priv, int offset,
              uint32_t regval);
#ifdef CONFIG_SAMA5_CAN_REGDEBUG
static void can_dumpctrlregs(struct sam_can_s *priv,
              const char *msg);
static void can_dumpmbregs(struct sam_can_s *priv, const char *msg);
#else
#  define can_dumpctrlregs(priv,msg)
#  define can_dumpmbregs(priv,msg)
#endif

/* Mailboxes */

static int  can_recvsetup(struct sam_can_s *priv);

/* CAN driver methods */

static void can_reset(struct can_dev_s *dev);
static int  can_setup(struct can_dev_s *dev);
static void can_shutdown(struct can_dev_s *dev);
static void can_rxint(struct can_dev_s *dev, bool enable);
static void can_txint(struct can_dev_s *dev, bool enable);
static int  can_ioctl(struct can_dev_s *dev, int cmd, unsigned long arg);
static int  can_remoterequest(struct can_dev_s *dev, uint16_t id);
static int  can_send(struct can_dev_s *dev, struct can_msg_s *msg);
static bool can_txready(struct can_dev_s *dev);
static bool can_txempty(struct can_dev_s *dev);

/* CAN interrupt handling */

static inline void can_rxinterrupt(struct can_dev_s *dev, int mbndx,
                                   uint32_t msr);
static inline void can_txinterrupt(struct can_dev_s *dev, int mbndx);
static inline void can_mbinterrupt(struct can_dev_s *dev, int mbndx);
static void can_interrupt(int irq, void *context, void *arg);

/* Hardware initialization */

static int  can_bittiming(struct sam_can_s *priv);
#ifdef CONFIG_SAMA5_CAN_AUTOBAUD
static int  can_autobaud(struct sam_can_s *priv);
#endif
static int  can_hwinitialize(struct sam_can_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct can_ops_s g_canops =
{
  .co_reset         = can_reset,
  .co_setup         = can_setup,
  .co_shutdown      = can_shutdown,
  .co_rxint         = can_rxint,
  .co_txint         = can_txint,
  .co_ioctl         = can_ioctl,
  .co_remoterequest = can_remoterequest,
  .co_send          = can_send,
  .co_txready       = can_txready,
  .co_txempty       = can_txempty,
};

#ifdef CONFIG_SAMA5_CAN0
static const struct sam_config_s g_can0const =
{
  .port             = 0,
  .pid              = SAM_PID_CAN0,
  .nrecvmb          = CONFIG_SAMA5_CAN0_NRECVMB,
  .base             = SAM_CAN0_VBASE,
  .baud             = CONFIG_SAMA5_CAN0_BAUD,
  .rxpinset         = PIO_CAN0_RX,
  .txpinset         = PIO_CAN0_TX,
  .filter           =
  {
    {
      .addr         = CONFIG_SAMA5_CAN0_ADDR0,
      .mask         = CONFIG_SAMA5_CAN0_MASK0,
    },
#if CONFIG_SAMA5_CAN0_NRECVMB > 1
    {
      .addr         = CONFIG_SAMA5_CAN0_ADDR1,
      .mask         = CONFIG_SAMA5_CAN0_MASK1,
    },
#if CONFIG_SAMA5_CAN0_NRECVMB > 1
    {
      .addr         = CONFIG_SAMA5_CAN0_ADDR2,
      .mask         = CONFIG_SAMA5_CAN0_MASK2,
    },
#endif
#endif
  },
};

static struct sam_can_s g_can0priv =
{
  .config           = &g_can0const,
  .freemb           = CAN_ALL_MAILBOXES,
  .lock             = NXMUTEX_INITIALIZER,
};
static struct can_dev_s g_can0dev =
{
  .cd_ops           = &g_canops,
  .cd_priv          = &g_can0priv,
};
#endif

#ifdef CONFIG_SAMA5_CAN1
static const struct sam_config_s g_can1const =
{
  .port             = 1,
  .pid              = SAM_PID_CAN1,
  .nrecvmb          = CONFIG_SAMA5_CAN1_NRECVMB,
  .base             = SAM_CAN1_VBASE,
  .baud             = CONFIG_SAMA5_CAN1_BAUD,
  .rxpinset         = PIO_CAN1_RX,
  .txpinset         = PIO_CAN1_TX,
  .filter           =
  {
    {
      .addr         = CONFIG_SAMA5_CAN1_ADDR0,
      .mask         = CONFIG_SAMA5_CAN1_MASK0,
    },
#if CONFIG_SAMA5_CAN1_NRECVMB > 1
    {
      .addr         = CONFIG_SAMA5_CAN1_ADDR1,
      .mask         = CONFIG_SAMA5_CAN1_MASK1,
    },
#if CONFIG_SAMA5_CAN1_NRECVMB > 1
    {
      .addr         = CONFIG_SAMA5_CAN1_ADDR2,
      .mask         = CONFIG_SAMA5_CAN1_MASK2,
    },
#endif
#endif
  },
};

static struct sam_can_s g_can1priv =
{
  .config           = &g_can1const,
  .freemb           = CAN_ALL_MAILBOXES,
  .lock             = NXMUTEX_INITIALIZER,
};
static struct can_dev_s g_can1dev =
{
  .cd_ops           = &g_canops,
  .cd_priv          = &g_can1priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: can_getreg
 *
 * Description:
 *   Read the value of a CAN register.
 *
 * Input Parameters:
 *   priv - A reference to the CAN peripheral state
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_CAN_REGDEBUG
static uint32_t can_getreg(struct sam_can_s *priv, int offset)
{
  const struct sam_config_s *config = priv->config;
  uintptr_t regaddr;
  uint32_t regval;

  /* Read the value from the register */

  regaddr = config->base + offset;
  regval  = getreg32(regaddr);

  /* Is this the same value that we read from the same register last time?
   * Are we polling the register?  If so, suppress some of the output.
   */

  if (regaddr == priv->regaddr && regval == priv->regval)
    {
      if (priv->count == 0xffffffff || ++priv->count > 3)
        {
          if (priv->count == 4)
            {
              caninfo("...\n");
            }

          return regval;
        }
    }

  /* No this is a new address or value */

  else
    {
      /* Did we print "..." for the previous value? */

      if (priv->count > 3)
        {
          /* Yes.. then show how many times the value repeated */

          caninfo("[repeats %d more times]\n", priv->count - 3);
        }

      /* Save the new address, value, and count */

      priv->regaddr = regaddr;
      priv->regval  = regval;
      priv->count   = 1;
    }

  /* Show the register value read */

  caninfo("%08x->%08x\n", regaddr, regval);
  return regval;
}

#else
static uint32_t can_getreg(struct sam_can_s *priv, int offset)
{
  const struct sam_config_s *config = priv->config;
  return getreg32(config->base + offset);
}

#endif

/****************************************************************************
 * Name: can_putreg
 *
 * Description:
 *   Set the value of a CAN register.
 *
 * Input Parameters:
 *   priv - A reference to the CAN peripheral state
 *   offset - The offset to the register to write
 *   regval - The value to write to the register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_CAN_REGDEBUG
static void can_putreg(struct sam_can_s *priv, int offset,
                       uint32_t regval)
{
  const struct sam_config_s *config = priv->config;
  uintptr_t regaddr = config->base + offset;

  /* Show the register value being written */

  caninfo("%08x<-%08x\n", regaddr, regval);

  /* Write the value */

  putreg32(regval, regaddr);
}

#else
static void can_putreg(struct sam_can_s *priv, int offset,
                       uint32_t regval)
{
  const struct sam_config_s *config = priv->config;
  putreg32(regval, config->base + offset);
}

#endif

/****************************************************************************
 * Name: can_dumpctrlregs
 *
 * Description:
 *   Dump the contents of all CAN control registers
 *
 * Input Parameters:
 *   priv - A reference to the CAN peripheral state
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_CAN_REGDEBUG
static void can_dumpctrlregs(struct sam_can_s *priv, const char *msg)
{
  const struct sam_config_s *config = priv->config;

  if (msg)
    {
      caninfo("Control Registers: %s\n", msg);
    }
  else
    {
      caninfo("Control Registers:\n");
    }

  /* CAN control and status registers */

  caninfo("   MR: %08x      IMR: %08x      SR: %08x\n",
          getreg32(config->base + SAM_CAN_MR_OFFSET),
          getreg32(config->base + SAM_CAN_IMR_OFFSET),
          getreg32(config->base + SAM_CAN_SR_OFFSET));

  caninfo("   BR: %08x      TIM: %08x TIMESTP: %08x\n",
          getreg32(config->base + SAM_CAN_BR_OFFSET),
          getreg32(config->base + SAM_CAN_TIM_OFFSET),
          getreg32(config->base + SAM_CAN_TIMESTP_OFFSET));

  caninfo("  ECR: %08x     WPMR: %08x    WPSR: %08x\n",
          getreg32(config->base + SAM_CAN_ECR_OFFSET),
          getreg32(config->base + SAM_CAN_TCR_OFFSET),
          getreg32(config->base + SAM_CAN_ACR_OFFSET));
}
#endif

/****************************************************************************
 * Name: can_dumpmbregs
 *
 * Description:
 *   Dump the contents of all CAN mailbox registers
 *
 * Input Parameters:
 *   priv - A reference to the CAN peripheral state
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_CAN_REGDEBUG
static void can_dumpmbregs(struct sam_can_s *priv, const char *msg)
{
  const struct sam_config_s *config = priv->config;
  uintptr_t mbbase;
  int i;

  if (msg)
    {
      caninfo("Mailbox Registers: %s\n", msg);
    }
  else
    {
      caninfo("Mailbox Registers:\n");
    }

  for (i = 0; i < SAM_CAN_NMAILBOXES; i++)
    {
      mbbase = config->base + SAM_CAN_MBN_OFFSET(i);
      caninfo("  MB%d:\n", i);

      /* CAN mailbox registers */

      caninfo("    MMR: %08x MAM: %08x MID: %08x MFID: %08x\n",
              getreg32(mbbase + SAM_CAN_MMR_OFFSET),
              getreg32(mbbase + SAM_CAN_MAM_OFFSET),
              getreg32(mbbase + SAM_CAN_MID_OFFSET),
              getreg32(mbbase + SAM_CAN_MFID_OFFSET));

      caninfo("    MSR: %08x MDL: %08x MDH: %08x\n",
              getreg32(mbbase + SAM_CAN_MSR_OFFSET),
              getreg32(mbbase + SAM_CAN_MDL_OFFSET),
              getreg32(mbbase + SAM_CAN_MDH_OFFSET));
    }
}
#endif

/****************************************************************************
 * Name: can_mballoc
 *
 * Description:
 *   Allocate one mailbox
 *
 * Input Parameters:
 *   priv - A pointer to the private data structure for this CAN peripheral
 *
 * Returned Value:
 *   A non-negative mailbox index; a negated errno value on failure.
 *
 * Assumptions:
 *   The caller has exclusive access to the can data structures
 *
 ****************************************************************************/

static int can_mballoc(struct sam_can_s *priv)
{
  int i;

  /* There any mailboxes free? */

  if (priv->freemb)
    {
      /* Yes.. There are free mailboxes... pick one */

      for (i = 0; i < SAM_CAN_NMAILBOXES; i++)
        {
          /* Is mailbox i available? */

          uint8_t bit = (1 << i);
          if ((priv->freemb & bit) != 0)
            {
              /* No any more.  Mark it allocated and return its index */

              priv->freemb &= ~bit;
              return i;
            }
        }
    }

  /* No available mailboxes */

  return -ENOMEM;
}

/****************************************************************************
 * Name: can_mbfree
 *
 * Description:
 *   Free one mailbox
 *
 * Input Parameters:
 *   priv - A pointer to the private data structure for this CAN peripheral
 *   mbndx - Index of the mailbox to be freed
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The caller has exclusive access to the can data structures
 *
 ****************************************************************************/

static void can_mbfree(struct sam_can_s *priv, int mbndx)
{
  uint8_t bit;

  DEBUGASSERT(priv && (unsigned)mbndx < SAM_CAN_NMAILBOXES);

  /* Disable mailbox interrupts */

  can_putreg(priv, SAM_CAN_IDR_OFFSET, CAN_INT_MB(mbndx));

  /* Disable the mailbox */

  can_putreg(priv, SAM_CAN_MNMR_OFFSET(mbndx), 0);

  /* Free the mailbox by clearing the corresponding bit in the freemb and
   * txmbset (only TX mailboxes are freed in this way.
   */

  bit = (1 << mbndx);
  DEBUGASSERT((priv->freemb & bit) != 0);
  DEBUGASSERT((priv->txmbset & bit) != 0);

  priv->freemb &= ~bit;
  priv->txmbset &= ~bit;
}

/****************************************************************************
 * Name: can_recvsetup
 *
 * Description:
 *   Configure and enable mailbox(es) for reception
 *
 * Input Parameters:
 *   priv - A pointer to the private data structure for this CAN peripheral
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 * Assumptions:
 *   Caller has exclusive access to the CAN data structures
 *   CAN interrupts are disabled at the AIC
 *
 ****************************************************************************/

static int can_recvsetup(struct sam_can_s *priv)
{
  const struct sam_config_s *config = priv->config;
  int mbndx;
  int mbno;

  /* Setup the configured number of receive mailboxes */

  priv->rxmbset = 0;
  for (mbno = 0; mbno < config->nrecvmb; mbno++)
    {
      /* Allocate a(nother) receive mailbox */

      mbndx = can_mballoc(priv);
      if (mbndx < 0)
        {
          canerr("ERROR: Failed to allocate mailbox %d: %d\n", mbno, mbndx);
          return mbndx;
        }

      /* Add the allocated mailbox to the set of receive mailboxes */

      priv->rxmbset |= (1 << mbndx);

      caninfo("CAN%d Mailbox %d: Index=%d rxmbset=%02x\n",
              config->port, mbno, mbndx, priv->rxmbset);

      /* Set up the message ID and filter mask
       * REVISIT: This logic should be capable of setting up standard
       * filters when CONFIG_CAN_EXTID is selected.
       */

#ifdef CONFIG_CAN_EXTID
      can_putreg(priv, SAM_CAN_MNID_OFFSET(mbndx),
                 CAN_MID_EXTID(config->filter[mbno].addr));
      can_putreg(priv, SAM_CAN_MNAM_OFFSET(mbndx),
                 CAN_MAM_EXTID(config->filter[mbno].mask));
#else
      can_putreg(priv, SAM_CAN_MNID_OFFSET(mbndx),
                 CAN_MID_STDID(config->filter[mbno].addr));
      can_putreg(priv, SAM_CAN_MNAM_OFFSET(mbndx),
                 CAN_MAM_STDID(config->filter[mbno].mask));
#endif

      /* Note:  Chaining is not supported.  All receive mailboxes are
       * configured in normal receive mode.
       *
       * REVISIT:  Chaining would be needed if you want to support
       * multipart messages.
       */

      can_putreg(priv, SAM_CAN_MNMR_OFFSET(mbndx), CAN_MMR_MOT_RX);

      /* Clear pending interrupts and start reception of the next message */

      can_putreg(priv, SAM_CAN_MNCR_OFFSET(mbndx), CAN_MCR_MTCR);

      /* Enable interrupts from this mailbox */

      can_putreg(priv, SAM_CAN_IER_OFFSET, CAN_INT_MB(mbndx));
    }

  return OK;
}

/****************************************************************************
 * Name: can_reset
 *
 * Description:
 *   Reset the CAN device.  Called early to initialize the hardware. This
 *   function is called, before can_setup() and on error conditions.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

static void can_reset(struct can_dev_s *dev)
{
  struct sam_can_s *priv;
  const struct sam_config_s *config;
  int ret;
  int i;

  DEBUGASSERT(dev);
  priv = dev->cd_priv;
  DEBUGASSERT(priv);
  config = priv->config;
  DEBUGASSERT(config);

  caninfo("CAN%d\n", config->port);
  UNUSED(config);

  /* Get exclusive access to the CAN peripheral */

  nxmutex_lock(&priv->lock);

  /* Disable all interrupts */

  can_putreg(priv, SAM_CAN_IDR_OFFSET, CAN_INT_ALL);

  /* Disable all mailboxes */

  for (i = 0; i < SAM_CAN_NMAILBOXES; i++)
    {
      can_putreg(priv, SAM_CAN_MNMR_OFFSET(i), 0);
    }

  /* All mailboxes are again available */

  priv->freemb = CAN_ALL_MAILBOXES;

  /* Disable the CAN controller */

  can_putreg(priv, SAM_CAN_MR_OFFSET, 0);
  nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Name: can_setup
 *
 * Description:
 *   Configure the CAN. This method is called the first time that the CAN
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching CAN interrupts.
 *   All CAN interrupts are disabled upon return.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int can_setup(struct can_dev_s *dev)
{
  struct sam_can_s *priv;
  const struct sam_config_s *config;
  int ret;

  DEBUGASSERT(dev);
  priv = dev->cd_priv;
  DEBUGASSERT(priv);
  config = priv->config;
  DEBUGASSERT(config);

  caninfo("CAN%d pid: %d\n", config->port, config->pid);

  /* Get exclusive access to the CAN peripheral */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* CAN hardware initialization */

  ret = can_hwinitialize(priv);
  if (ret < 0)
    {
      canerr("ERROR: CAN%d H/W initialization failed: %d\n",
             config->port, ret);
      return ret;
    }

  can_dumpctrlregs(priv, "After hardware initialization");
  can_dumpmbregs(priv, NULL);

  /* Attach the CAN interrupt handler */

  ret = irq_attach(config->pid, can_interrupt, dev);
  if (ret < 0)
    {
      canerr("ERROR: Failed to attach CAN%d IRQ (%d)",
             config->port, config->pid);
      return ret;
    }

  /* Setup receive mailbox(es) (enabling receive interrupts) */

  ret = can_recvsetup(priv);
  if (ret < 0)
    {
      canerr("ERROR: CAN%d H/W initialization failed: %d\n",
             config->port, ret);
      return ret;
    }

  /* Enable all error interrupts */

#ifdef CONFIG_DEBUG_FEATURES
  can_putreg(priv, SAM_CAN_IER_OFFSET, CAN_DEBUG_INTS);
#endif

  can_dumpctrlregs(priv, "After receive setup");
  can_dumpmbregs(priv, NULL);

  /* Enable the interrupts at the AIC. */

  up_enable_irq(config->pid);
  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: can_shutdown
 *
 * Description:
 *   Disable the CAN.  This method is called when the CAN device is closed.
 *   This method reverses the operation the setup method.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void can_shutdown(struct can_dev_s *dev)
{
  struct sam_can_s *priv;
  const struct sam_config_s *config;

  DEBUGASSERT(dev);
  priv = dev->cd_priv;
  DEBUGASSERT(priv);
  config = priv->config;
  DEBUGASSERT(config);

  caninfo("CAN%d\n", config->port);

  /* Get exclusive access to the CAN peripheral */

  nxmutex_lock(&priv->lock);

  /* Disable the CAN interrupts */

  up_disable_irq(config->pid);

  /* Detach the CAN interrupt handler */

  irq_detach(config->pid);

  /* And reset the hardware */

  can_reset(dev);
  nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Name: can_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void can_rxint(struct can_dev_s *dev, bool enable)
{
  struct sam_can_s *priv = dev->cd_priv;
  DEBUGASSERT(priv && priv->config);

  caninfo("CAN%d enable: %d\n", priv->config->port, enable);

  /* Enable/disable the mailbox interrupts from all receive mailboxes */

  if (enable)
    {
      can_putreg(priv, SAM_CAN_IER_OFFSET, priv->rxmbset);
    }
  else
    {
      can_putreg(priv, SAM_CAN_IDR_OFFSET, priv->rxmbset);
    }
}

/****************************************************************************
 * Name: can_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void can_txint(struct can_dev_s *dev, bool enable)
{
  struct sam_can_s *priv = dev->cd_priv;
  DEBUGASSERT(priv && priv->config);

  caninfo("CAN%d enable: %d\n", priv->config->port, enable);

  /* Get exclusive access to the CAN peripheral */

  nxmutex_lock(&priv->lock);

  /* Support disabling interrupts on any mailboxes that are actively
   * transmitting (txmbset); also suppress enabling new TX mailbox until
   * txdisabled is reset by this function.
   */

  if (enable)
    {
      can_putreg(priv, SAM_CAN_IER_OFFSET, priv->txmbset);
      priv->txdisabled = false;
    }
  else
    {
      can_putreg(priv, SAM_CAN_IDR_OFFSET, priv->txmbset);
      priv->txdisabled = true;
    }

  nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Name: can_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int can_ioctl(struct can_dev_s *dev, int cmd, unsigned long arg)
{
  /* No CAN ioctls are supported */

  return -ENOTTY;
}

/****************************************************************************
 * Name: can_remoterequest
 *
 * Description:
 *   Send a remote request
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int can_remoterequest(struct can_dev_s *dev, uint16_t id)
{
  /* REVISIT:  Remote request not implemented */

  return -ENOSYS;
}

/****************************************************************************
 * Name: can_send
 *
 * Description:
 *    Send one can message.
 *
 *    One CAN-message consists of a maximum of 10 bytes.  A message is
 *    composed of at least the first 2 bytes (when there are no data bytes).
 *
 *    Byte 0:      Bits 0-7: Bits 3-10 of the 11-bit CAN identifier
 *    Byte 1:      Bits 5-7: Bits 0-2 of the 11-bit CAN identifier
 *                 Bit 4:    Remote Transmission Request (RTR)
 *                 Bits 0-3: Data Length Code (DLC)
 *    Bytes 2-10: CAN data
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int can_send(struct can_dev_s *dev, struct can_msg_s *msg)
{
  struct sam_can_s *priv;
  uint8_t *ptr;
  uint32_t regval;
  int mbndx;

  DEBUGASSERT(dev);
  priv = dev->cd_priv;
  DEBUGASSERT(priv && priv->config);

  caninfo("CAN%d\n", priv->config->port);
  caninfo("CAN%d ID: %d DLC: %d\n",
          priv->config->port, msg->cm_hdr.ch_id, msg->cm_hdr.ch_dlc);

  /* Get exclusive access to the CAN peripheral */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Allocate a mailbox */

  mbndx = can_mballoc(priv);
  if (mbndx < 0)
    {
      canerr("ERROR: CAN%d failed to allocate a mailbox: %d\n",
             priv->config->port, mbndx);
      return mbndx;
    }

  priv->txmbset |= (1 << mbndx);

  caninfo("Mailbox Index=%d txmbset=%02x\n", mbndx, priv->txmbset);

  /* Set up the ID and mask, standard 11-bit or extended 29-bit.
   * REVISIT: This logic should be capable of sending standard messages
   * when CONFIG_CAN_EXTID is selected.
   */

#ifdef CONFIG_CAN_EXTID
  DEBUGASSERT(msg->cm_hdr.ch_extid);
  DEBUGASSERT(msg->cm_hdr.ch_id < (1 << 29));
  can_putreg(priv, SAM_CAN_MNID_OFFSET(mbndx),
             CAN_MID_EXTID(msg->cm_hdr.ch_id));
#else
  DEBUGASSERT(msg->cm_hdr.ch_id < (1 << 11));
  can_putreg(priv, SAM_CAN_MNID_OFFSET(mbndx),
             CAN_MID_STDID(msg->cm_hdr.ch_id));
#endif

  /* Enable transmit mode */

  can_putreg(priv, SAM_CAN_MNMR_OFFSET(mbndx), CAN_MMR_MOT_TX);

  /* After Transmit Mode is enabled, the MRDY flag in the CAN_MSR register
   * is automatically set until the first command is sent. When the MRDY
   * flag is set, the software application can prepare a message to be sent
   * by writing to the CAN_MDx registers. The message is sent once the
   * software asks for a transfer command setting the MTCR bit and the
   * message data length in the CAN_MCRx register.
   */

  DEBUGASSERT((can_getreg(priv, SAM_CAN_MNSR_OFFSET(mbndx)) &
               CAN_MSR_MRDY) != 0);

  /* Bytes are received/sent on the bus in the following order:
   *
   *   1. CAN_MDL[7:0]
   *   2. CAN_MDL[15:8]
   *   3. CAN_MDL[23:16]
   *   4. CAN_MDL[31:24]
   *   5. CAN_MDH[7:0]
   *   6. CAN_MDH[15:8]
   *   7. CAN_MDH[23:16]
   *   8. CAN_MDH[31:24]
   */

#ifdef CONFIG_ENDIAN_BIG
#  warning REVISIT
#endif

  /* The message buffer is probably not properaly aligned for 32-bit
   * accesses
   */

  ptr    = msg->cm_data;
  regval = CAN_MDL0(ptr[0]) | CAN_MDL1(ptr[1]) | CAN_MDL2(ptr[2]) |
           CAN_MDL3(ptr[3]);
  can_putreg(priv, SAM_CAN_MNDL_OFFSET(mbndx), regval);

  regval = CAN_MDH4(ptr[4]) | CAN_MDH5(ptr[5]) | CAN_MDH6(ptr[6]) |
           CAN_MDH7(ptr[7]);
  can_putreg(priv, SAM_CAN_MNDH_OFFSET(mbndx), regval);

  /* Set the DLC value in the CAN_MCRx register.  Set the MTCR register
   * clearing MRDY, and indicating that the message is ready to be sent.
   */

  regval = CAN_MCR_MDLC(msg->cm_hdr.ch_dlc) | CAN_MCR_MTCR;
  can_putreg(priv, SAM_CAN_MNCR_OFFSET(mbndx), regval);

  /* If we have not been asked to suppress TX interrupts, then enable
   * interrupts from this mailbox now.
   */

  if (!priv->txdisabled)
    {
      can_putreg(priv, SAM_CAN_IER_OFFSET, CAN_INT_MB(mbndx));
    }

  can_dumpmbregs(priv, "After send");
  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: can_txready
 *
 * Description:
 *   Return true if the CAN hardware can accept another TX message.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   True if the CAN hardware is ready to accept another TX message.
 *
 ****************************************************************************/

static bool can_txready(struct can_dev_s *dev)
{
  struct sam_can_s *priv = dev->cd_priv;
  bool txready;

  /* Get exclusive access to the CAN peripheral */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return false;
    }

  /* Return true not all mailboxes are in-use */

  txready = ((priv->rxmbset | priv->txmbset) != CAN_ALL_MAILBOXES);

  nxmutex_unlock(&priv->lock);
  return txready;
}

/****************************************************************************
 * Name: can_txempty
 *
 * Description:
 *   Return true if all message have been sent.  If for example, the CAN
 *   hardware implements FIFOs, then this would mean the transmit FIFO is
 *   empty.  This method is called when the driver needs to make sure that
 *   all characters are "drained" from the TX hardware before calling
 *   co_shutdown().
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   True if there are no pending TX transfers in the CAN hardware.
 *
 ****************************************************************************/

static bool can_txempty(struct can_dev_s *dev)
{
  struct sam_can_s *priv = dev->cd_priv;
  return (priv->txmbset == 0);
}

/****************************************************************************
 * Name: can_rxinterrupt
 *
 * Description:
 *   CAN RX mailbox interrupt handler
 *
 * Input Parameters:
 *   priv - CAN-specific private data
 *   mbndx - The index of the mailbox that generated the interrupt
 *   msr - Applicable value from the mailbox status register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void can_rxinterrupt(struct can_dev_s *dev, int mbndx,
                                   uint32_t msr)
{
  struct sam_can_s *priv = dev->cd_priv;
  struct can_hdr_s hdr;
  uint32_t md[2];
  uint32_t mid;
  int ret;

  /* REVISIT:  Check the MMI bit in CAN_MSRx to determine messages have been
   * lost.
   */

  /* Read the mailbox data.  Bytes are received/sent on the bus in the
   * following order:
   *
   *   1. CAN_MDL[7:0]
   *   2. CAN_MDL[15:8]
   *   3. CAN_MDL[23:16]
   *   4. CAN_MDL[31:24]
   *   5. CAN_MDH[7:0]
   *   6. CAN_MDH[15:8]
   *   7. CAN_MDH[23:16]
   *   8. CAN_MDH[31:24]
   */

#ifdef CONFIG_ENDIAN_BIG
#  warning REVISIT
#endif

  md[0] = can_getreg(priv, SAM_CAN_MNDH_OFFSET(mbndx));
  md[1] = can_getreg(priv, SAM_CAN_MNDL_OFFSET(mbndx));

  /* Get the ID associated with the newly received message: )nce a new
   * message is received, its ID is masked with the CAN_MAMx value and
   * compared with the CAN_MIDx value. If accepted, the message ID is
   * copied to the CAN_MIDx register.
   */

  mid = can_getreg(priv, SAM_CAN_MNID_OFFSET(mbndx));

  /* Format the CAN header.
   * REVISIT: This logic should be capable of receiving standard messages
   * when CONFIG_CAN_EXTID is selected.
   */

#ifdef CONFIG_CAN_EXTID
  /* Save the extended ID of the newly received message */

  hdr.ch_id     = (mid & CAN_MAM_EXTID_MASK) >> CAN_MAM_EXTID_SHIFT;
  hdr.ch_extid  = true;
#else
  /* Save the standard ID of the newly received message */

  hdr.ch_id     = (mid & CAN_MAM_STDID_MASK) >> CAN_MAM_STDID_SHIFT;
#endif

  hdr.ch_dlc    = (msr & CAN_MSR_MDLC_MASK) >> CAN_MSR_MDLC_SHIFT;
  hdr.ch_rtr    = 0;
#ifdef CONFIG_CAN_ERRORS
  hdr.ch_error  = 0; /* Error reporting not supported */
#endif
  hdr.ch_unused = 0;

  /* And provide the CAN message to the upper half logic */

  ret = can_receive(dev, &hdr, (uint8_t *)md);
  if (ret < 0)
    {
      canerr("ERROR: can_receive failed: %d\n", ret);
    }

  /* Set the MTCR flag in the CAN_MCRx register.  This clears the
   * MRDY bit, notifices the hardware that processing has ended, and
   * requests a new RX transfer.
   */

  can_putreg(priv, SAM_CAN_MNCR_OFFSET(mbndx), CAN_MCR_MTCR);
}

/****************************************************************************
 * Name: can_txinterrupt
 *
 * Description:
 *   CAN TX mailbox interrupt handler
 *
 * Input Parameters:
 *   priv - CAN-specific private data
 *   mbndx - The index of the mailbox that generated the interrupt
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void can_txinterrupt(struct can_dev_s *dev, int mbndx)
{
  struct sam_can_s *priv = dev->cd_priv;

  /* REVISIT:  Check the MABT bit in CAN_MSRx to determine if the transfer
   * was aborted.
   */

  /* Disable and free the mailbox */

  can_mbfree(priv, mbndx);

  /* Report that the TX transfer is complete to the upper half logic */

  can_txdone(dev);
}

/****************************************************************************
 * Name: can_mbinterrupt
 *
 * Description:
 *   CAN mailbox interrupt handler
 *
 * Input Parameters:
 *   priv - CAN-specific private data
 *   mbndx - The index of the mailbox that generated the interrupt
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void can_mbinterrupt(struct can_dev_s *dev, int mbndx)
{
  struct sam_can_s *priv = dev->cd_priv;
  uint32_t msr;
  uint32_t mmr;

  /* There are two causes of mailbox interrupts:
   *
   * - Data registers in the mailbox object are available to the
   *   application. In Receive Mode, a new message was received. In Transmit
   *   Mode, a message was transmitted successfully.
   * - A sent transmission was aborted.
   *
   * Both conditions are are reported by the MRDY bit in the CAN_MSR
   * register.
   */

  msr = can_getreg(priv, SAM_CAN_MNSR_OFFSET(mbndx));
  if ((msr & (CAN_MSR_MRDY | CAN_MSR_MABT)) != 0)
    {
      /* Handle the result based on how the mailbox was configured */

      mmr = can_getreg(priv, SAM_CAN_MNMR_OFFSET(mbndx));
      switch (mmr & CAN_MMR_MOT_MASK)
        {
          case CAN_MMR_MOT_RX:       /* Reception Mailbox */
            can_rxinterrupt(dev, mbndx, msr);
            break;

          case CAN_MMR_MOT_TX:       /* Transmit mailbox */
            can_txinterrupt(dev, mbndx);
            break;

          case CAN_MMR_MOT_RXOVRWR:  /* Reception mailbox with overwrite */
          case CAN_MMR_MOT_CONSUMER: /* Consumer Mailbox */
          case CAN_MMR_MOT_PRODUCER: /* Producer Mailbox */
          case CAN_MMR_MOT_DISABLED: /* Mailbox is disabled */
            canerr("ERROR: CAN%d MB%d: Unsupported or "
                    "invalid mailbox type\n",
                   priv->config->port, mbndx);
            canerr("       MSR: %08x MMR: %08x\n", msr, mmr);
            break;
        }
    }
}

/****************************************************************************
 * Name: can_interrupt
 *
 * Description:
 *   Common CAN interrupt handler
 *
 * Input Parameters:
 *   Standard interrupt handler inputs
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void can_interrupt(int irq, void *context, void *arg)
{
  struct can_dev_s *dev = (struct can_dev_s *)arg;
  struct sam_can_s *priv;
  uint32_t sr;
  uint32_t imr;
  uint32_t pending;

  DEBUGASSERT(dev != NULL);
  struct sam_can_s *priv = dev->cd_priv;
  DEBUGASSERT(priv != NULL && priv->config != NULL);

  /* Get the set of pending interrupts.
   *
   * All interrupts are cleared by clearing the interrupt source except for
   * the internal timer counter overflow interrupt and the timestamp
   * interrupt. * These interrupts are cleared by reading the CAN_SR
   * register.
   */

  sr      = can_getreg(priv, SAM_CAN_SR_OFFSET);
  imr     = can_getreg(priv, SAM_CAN_IMR_OFFSET);
  pending = sr & imr;

  /* There are two different types of interrupts. One type of interrupt is a
   * message-object related interrupt, the other is a system interrupt that
   * handles errors or system-related interrupt sources.
   */

  /* Check for message related interrupts
   *
   * - Data registers in the mailbox object are available to the
   *   application. In Receive Mode, a new message was received. In Transmit
   *   Mode, a message was transmitted successfully.
   * - A sent transmission was aborted.
   */

  if ((pending & CAN_INT_MBALL) != 0)
    {
      int mbndx;

      /* Check for pending interrupts from each mailbox */

      for (mbndx = 0; mbndx < SAM_CAN_NMAILBOXES; mbndx++)
        {
          /* Check for a pending interrupt for this mailbox */

          if ((pending & CAN_INT_MB(mbndx)) != 0)
            {
              can_mbinterrupt(dev, mbndx);
            }
        }
    }

  /* Check for system interrupts
   *
   * - Bus off interrupt: The CAN module enters the bus off state.
   * - Error passive interrupt: The CAN module enters Error Passive Mode.
   * - Error Active Mode: The CAN module is neither in Error Passive Mode
   *   nor in Bus Off mode.
   * - Warn Limit interrupt: The CAN module is in Error-active Mode, but at
   *   least one of its error counter value exceeds 96.
   * - Wake-up interrupt: This interrupt is generated after a wake-up and a
   *   bus synchronization.
   * - Sleep interrupt: This interrupt is generated after a Low-power Mode
   *   enable once all pending messages in transmission have been sent.
   * - Internal timer counter overflow interrupt: This interrupt is
   *   generated when the internal timer rolls over.
   * - Timestamp interrupt: This interrupt is generated after the reception
   *   or the transmission of a start of frame or an end of frame. The value
   *   of the internal counter is copied in the CAN_TIMESTP register.
   */

  if ((pending & ~CAN_INT_MBALL) != 0)
    {
      canerr("ERROR: CAN%d system interrupt, SR=%08x IMR=%08x\n",
             priv->config->port, sr, imr);
    }
}

/****************************************************************************
 * Name: can_bittiming
 *
 * Description:
 *   Set the CAN baudrate register (BR) based on the configured BAUD.
 *
 *   Definitions:
 *
 *      TIME QUANTUM.  The TIME QUANTUM (Tq) is a fixed unit of time derived
 *      from the MCK period. The total number of TIME QUANTA in a bit time is
 *      programmable from 8 to 25.
 *
 *      INFORMATION PROCESSING TIME.  The Information Processing Time (IPT)
 *      is the time required for the logic to determine the bit level of a
 *      sampled bit. The IPT begins at the sample point, is measured in Tq
 *      and is fixed at 2 Tq for the Atmel CAN.
 *
 *      SAMPLE POINT. The SAMPLE POINT is the point in time at which the
 *      bus level is read and interpreted as the value of that respective
 *      bit.  Its location is at the end of PHASE_SEG1.
 *
 *   The CAN protocol specification partitions the nominal bit time into
 *   four different segments:
 *
 *   1. Synchronization segment (SYNC_SEG): a bit change is expected to occur
 *      within this time segment. It has a fixed length of one time quantum
 *      (1 x tCAN).
 *   2. Propagation segment (PROP_SEG):  This part of the bit time is used
 *      to compensate for the physical delay times within the network. It is
 *      twice the sum of the signal�s propagation time on the bus line, the
 *      input comparator delay, and the output driver delay. It is
 *      programmable to be 1 to 8 Tq long.  This parameter is defined in the
 *      PROPAG field of the CAN Baudrate Register.
 *   3. Phase segment 1 (PHASE_SEG1): defines the location of the sample
 *      point.  Phase Segment 1 is programmable to be 1-8 Tq long.
 *   4. Phase segment 2 (PHASE_SEG2):  defines the location of the transmit
 *      point.Phase Segment 2 length has to be at least as long as the
 *      Information Processing Time (IPT) and may not be more than the
 *      length of Phase Segment 1 (since Phase Segment 2 also begins at the
 *      sample point and is the last segment in the bit time).
 *
 *   The Phase-Buffer-Segments are used to compensate for edge phase errors.
 *   These segments can be lengthened (PHASE SEG1) or shortened (PHASE SEG2)
 *   by resynchronization:
 *
 *      SJW: ReSynchronization Jump Width.  The ReSynchronization Jump Width
 *      defines the limit to the amount of lengthening or shortening of the
 *      Phase Segments.  SJW is programmable to be the minimum of PHASE SEG1
 *      and 4 Tq.
 *
 *   In the CAN controller, the length of a bit on the CAN bus is determined
 *   by the parameters (BRP, PROPAG, PHASE1 and PHASE2).
 *
 *      Tbit = Tcsc + Tprs + Tphs1 + Tphs2
 *
 *   Pictorially:
 *
 *    |<----------------------- NOMINAL BIT TIME ------------------------>|
 *    |<- SYNC_SEG ->|<- PROP_SEG ->|<-- PHASE_SEG1 -->|<-- PHASE_SEG2 -->|
 *    |<--- Tcsc --->|<--- Tprs --->|<---- Tphs1 ----->|<---- Tphs2 ----->|
 *    |<--- 1 Tq --->|<-- 1-8 Tq -->|<---- 1-8 Tq ---->|<--- <= Tphs1 --->|
 *
 *   Where
 *     Tcsc  is the duration of the SYNC_SEG segment
 *     Tprs  is the duration of the PROP_SEG segment
 *     Tphs1 is the duration of the PHASE_SEG1 segment
 *     Tphs2 is the duration of the PHASE_SEG2 segment
 *     Tq    is the "Time Quantum"
 *
 *   Relationships:
 *
 *     baud     = 1 / Tbit
 *     Tbit     = Tq + Tprs + Tphs1 + Tphs2
 *     Tq       = (BRP + 1) / MCK
 *     Tprs     = Tq * (PROPAG + 1)
 *     Tphs1    = Tq * (PHASE1 + 1)
 *     Tphs2    = Tq * (PHASE2 + 1)
 *
 * Input Parameters:
 *   config - A reference to the CAN constant configuration
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int can_bittiming(struct sam_can_s *priv)
{
  const struct sam_config_s *config = priv->config;
  uint32_t regval;
  uint32_t brp;
  uint32_t propag;
  uint32_t phase1;
  uint32_t phase2;
  uint32_t sjw;
  uint32_t t1t2;
  uint8_t  tq;

  /* Select the time quantum
   *
   * REVISIT:  We could probably do a better job than this.
   */

  if (config->baud >= 1000)
    {
      tq = 8;
    }
  else
    {
      tq = 16;
    }

  /* Calculate the baudrate prescaler (BRP).  This depends only on the
   * selected Tq value, the desired BAUD and the CAN peripheral clock
   * frequency.
   *
   *   Tq   = (BRP + 1) / CAN_FRQUENCY
   *   Tbit = Nquanta * (BRP + 1) / Fcan
   *   baud = Fcan / (Nquanta * (brp + 1))
   *   brp  = Fcan / (baud * nquanta) - 1
   */

  brp = (priv->frequency / (config->baud * 1000 * tq)) - 1;
  if (brp == 0)
    {
      /* The BRP field must be within the range 1 - 0x7f */

      canerr("CAN%d ERROR: baud %d too high\n", config->port, config->baud);
      return -EINVAL;
    }

  /* Propagation delay:
   *
   *   Delay Bus Driver     - 50ns
   *   Delay Receiver       - 30ns
   *   Delay Bus Line (20m) - 110ns
   */

  propag = tq * config->baud * 2 * (50 + 30 + 110) / 1000000;
  if (propag >= 1)
    {
      propag--;
    }
  else
    {
      propag = 0;
    }

  /* This the time of the first two segments */

  t1t2 = tq - 1 - (propag + 1);

  /* Calculate phase1 and phase2 */

  phase1 = (t1t2 >> 1) - 1;
  phase2 = phase1;

  if ((t1t2 & 1) != 0)
    {
      phase2++;
    }

  /* Calculate SJW */

  if (1 > (4 / (phase1 + 1)))
    {
      sjw = 3;
    }
  else
    {
      sjw = phase1;
    }

  if ((propag + phase1 + phase2) != (uint32_t)(tq - 4))
    {
      canerr("CAN%d ERROR: Could not realize baud %d\n",
             config->port, config->baud);
      return -EINVAL;
    }

  regval = CAN_BR_PHASE2(phase2) | CAN_BR_PHASE1(phase1) |
           CAN_BR_PROPAG(propag) | CAN_BR_SJW(sjw) | CAN_BR_BRP(brp) |
           CAN_BR_ONCE;
  can_putreg(priv, SAM_CAN_BR_OFFSET, regval);
  return OK;
}

/****************************************************************************
 * Name: can_autobaud
 *
 * Description:
 *   Use the SAMA5 auto-baud feature to correct the initial timing
 *
 * Input Parameters:
 *   priv - A pointer to the private data structure for this CAN block
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_CAN_AUTOBAUD
static int can_autobaud(struct sam_can_s *priv)
{
  volatile uint32_t timeout;
  uint32_t regval;
  int ret;

  caninfo("CAN%d\n", config->port);

  /* The CAN controller can start listening to the network in Autobaud Mode.
   * In this case, the error counters are locked and a mailbox may be
   * configured in Receive Mode. By scanning error flags, the CAN_BR
   * register values synchronized with the network.
   */

  /* Configure a Mailbox in Reception Mode */
#warning Missing Logic

  /* Loop, adjusting bit rate parameters until no errors are reported in
   * either CAR_SR or the CAN_MSRx registers.
   */

  do
    {
      /* Adjust baud rate setting */
#warning Missing Logic

      /* Autobaud Mode. The autobaud feature is enabled by setting the ABM
       * field in the CAN_MR register. In this mode, the CAN controller is
       * only listening to the line without acknowledging the received
       * messages. It can not send any message. The errors flags are
       * updated. The bit timing can be adjusted until no error occurs (good
       * configuration found).  In this mode, the error counters are frozen.
       */

      regval  = can_getreg(priv, SAM_CAN_MR_OFFSET);
      regval |= (CAN_MR_CANEN | CAN_MR_ABM);
      can_putreg(priv, SAM_CAN_MR_OFFSET, regval);

#warning Missing logic
    }
  while (no errors reported);

  /* Once no error has been detected, the application disables the Autobaud
   * Mode, clearing the ABM field in the CAN_MR register.  To go back to the
   * standard mode, the ABM bit must be cleared in the CAN_MR register.
   */

  regval  = can_getreg(priv, SAM_CAN_MR_OFFSET);
  regval &= ~(CAN_MR_CANEN | CAN_MR_ABM);
  can_putreg(priv, SAM_CAN_MR_OFFSET, regval);

  return OK;
}
#endif

/****************************************************************************
 * Name: can_hwinitialize
 *
 * Description:
 *   CAN cell initialization
 *
 * Input Parameters:
 *   priv - A pointer to the private data structure for this CAN peripheral
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int can_hwinitialize(struct sam_can_s *priv)
{
  const struct sam_config_s *config = priv->config;
  uint32_t regval;
  uint32_t mck;
  int ret;

  caninfo("CAN%d\n", config->port);

  /* Configure CAN pins */

  sam_configpio(config->rxpinset);
  sam_configpio(config->txpinset);

  /* Determine the maximum CAN peripheral clock frequency */

  mck = BOARD_MCK_FREQUENCY;
  if (mck <= SAM_CAN_MAXPERCLK)
    {
      priv->frequency = mck;
      regval          = PMC_PCR_DIV1;
    }
  else if ((mck >> 1) <= SAM_CAN_MAXPERCLK)
    {
      priv->frequency = (mck >> 1);
      regval          = PMC_PCR_DIV2;
    }
  else if ((mck >> 2) <= SAM_CAN_MAXPERCLK)
    {
      priv->frequency = (mck >> 2);
      regval          = PMC_PCR_DIV4;
    }
  else if ((mck >> 3) <= SAM_CAN_MAXPERCLK)
    {
      priv->frequency = (mck >> 3);
      regval          = PMC_PCR_DIV8;
    }
  else
    {
      canerr("ERROR: Cannot realize CAN input frequency\n");
      return -EINVAL;
    }

  /* Set the maximum CAN peripheral clock frequency */

  regval |= PMC_PCR_PID(config->pid) | PMC_PCR_CMD | PMC_PCR_EN;
  can_putreg(priv, SAM_PMC_PCR, regval);

  /* Enable peripheral clocking */

  sam_enableperiph1(config->pid);

  /* Disable all CAN interrupts */

  can_putreg(priv, SAM_CAN_IDR_OFFSET, CAN_INT_ALL);

  /* Configure bit timing. */

  ret = can_bittiming(priv);
  if (ret < 0)
    {
      canerr("ERROR: Failed to set bit timing: %d\n", ret);
      return ret;
    }

#ifdef CONFIG_SAMA5_CAN_AUTOBAUD
  /* Optimize/correct bit timing */

  ret = can_autobaud(priv);
  if (ret < 0)
    {
      canerr("ERROR: can_autobaud failed: %d\n", ret);
      return ret;
    }
#endif

  /* The CAN controller is enabled by setting the CANEN flag in the CAN_MR
   * register. At this stage, the internal CAN controller state machine is
   * reset, error counters are reset to 0, error flags are reset to 0.
   */

  regval  = can_getreg(priv, SAM_CAN_MR_OFFSET);
  regval |= CAN_MR_CANEN;
  can_putreg(priv, SAM_CAN_MR_OFFSET, regval);

  /* Once the CAN controller is enabled, bus synchronization is done
   * automatically by scanning eleven recessive bits. The WAKEUP bit in
   * the CAN_SR register is automatically set to 1 when the CAN controller
   * is synchronized (WAKEUP and SLEEP are stuck at 0 after a reset).
   */

  while ((can_getreg(priv, SAM_CAN_SR_OFFSET) & CAN_INT_WAKEUP) == 0);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_caninitialize
 *
 * Description:
 *   Initialize the selected CAN port
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple CAN interfaces)
 *
 * Returned Value:
 *   Valid CAN device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct can_dev_s *sam_caninitialize(int port)
{
  struct can_dev_s *dev;
  struct sam_can_s *priv;

  caninfo("CAN%d\n", port);

  /* NOTE:  Peripherical clocking for CAN0 and/or CAN1 was already provided
   * by sam_clockconfig() early in the reset sequence.
   */

#ifdef CONFIG_SAMA5_CAN0
  if (port == 0)
    {
      /* Select the CAN0 device structure */

      dev  = &g_can0dev;
      priv = &g_can0priv;
    }
  else
#endif
#ifdef CONFIG_SAMA5_CAN1
  if (port == 1)
    {
      /* Select the CAN1 device structure */

      dev  = &g_can1dev;
      priv = &g_can1priv;
    }
  else
#endif
    {
      canerr("ERROR: Unsupported port %d\n", port);
      return NULL;
    }

  /* Is this the first time that we have handed out this device? */

  if (!priv->initialized)
    {
      /* Yes, then perform one time data initialization */

      priv->initialized = true;

      /* And put the hardware in the initial state */

      can_reset(dev);
    }

  return dev;
}

#endif /* CONFIG_CAN && (CONFIG_SAMA5_CAN0 || CONFIG_SAMA5_CAN1) */
