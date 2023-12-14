/****************************************************************************
 * arch/arm/src/at32/at32_can_sock.c
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

#include <inttypes.h>
#include <stdio.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include <nuttx/wqueue.h>
#include <nuttx/can.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/can.h>
#include <netpacket/can.h>

#include "arm_internal.h"
#include "chip.h"
#include "at32.h"
#include "at32_rcc.h"
#include "at32_can.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Delays *******************************************************************/

/* Time out for INAK bit */

#define INAK_TIMEOUT 65535

/* Bit timing ***************************************************************/

#define CAN_BIT_QUANTA (CONFIG_AT32_CAN_TSEG1 + CONFIG_AT32_CAN_TSEG2 + 1)

#ifndef CONFIG_DEBUG_CAN_INFO
#  undef CONFIG_AT32_CAN_REGDEBUG
#endif

/* Pool configuration *******************************************************/

#define POOL_SIZE  (1)

/* Work queue support is required. */

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error Work queue support is required
#endif

/* The low priority work queue is preferred.  If it is not enabled, LPWORK
 * will be the same as HPWORK.
 *
 * NOTE:  However, the network should NEVER run on the high priority work
 * queue!  That queue is intended only to service short back end interrupt
 * processing that never suspends.  Suspending the high priority work queue
 * may bring the system to its knees!
 */

#define CANWORK LPWORK

/* CAN error interrupts */

#ifdef CONFIG_NET_CAN_ERRORS
#  define AT32_CAN_ERRINT (CAN_INTEN_ETRIEN | CAN_INTEN_EOIEN |             \
                            CAN_INTEN_BOIEN | CAN_INTEN_EPIEN |             \
                            CAN_INTEN_EAIEN)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct at32_can_s
{
  uint8_t  port;     /* CAN port number (1 or 2) */
  uint8_t  canrx[2]; /* CAN RX FIFO 0/1 IRQ number */
  uint8_t  cantx;    /* CAN TX IRQ number */
#ifdef CONFIG_NET_CAN_ERRORS
  uint8_t  cansce;   /* CAN SCE IRQ number */
#endif
  uint8_t  filter;   /* Filter number */
  uint32_t base;     /* Base address of the CAN control registers */
  uint32_t fbase;    /* Base address of the CAN filter registers */
  uint32_t baud;     /* Configured baud */

  bool                bifup;  /* true:ifup false:ifdown */
  struct net_driver_s dev;    /* Interface understood by the network */

  struct work_s irqwork;  /* For deferring interrupt work to the wq */
  struct work_s pollwork; /* For deferring poll work to the work wq */

  /* A pointers to the list of TX/RX descriptors */

  struct can_frame *txdesc;
  struct can_frame *rxdesc;

  /* TX/RX pool */

  uint8_t tx_pool[sizeof(struct can_frame)*POOL_SIZE];
  uint8_t rx_pool[sizeof(struct can_frame)*POOL_SIZE];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* CAN Register access */

static uint32_t at32can_getreg(struct at32_can_s *priv,
                                int offset);
static uint32_t at32can_getfreg(struct at32_can_s *priv,
                                 int offset);
static void at32can_putreg(struct at32_can_s *priv, int offset,
                            uint32_t value);
static void at32can_putfreg(struct at32_can_s *priv, int offset,
                             uint32_t value);
#ifdef CONFIG_AT32_CAN_REGDEBUG
static void at32can_dumpctrlregs(struct at32_can_s *priv,
                                  const char *msg);
static void at32can_dumpmbregs(struct at32_can_s *priv,
                                const char *msg);
static void at32can_dumpfiltregs(struct at32_can_s *priv,
                                  const char *msg);
#else
#  define at32can_dumpctrlregs(priv,msg)
#  define at32can_dumpmbregs(priv,msg)
#  define at32can_dumpfiltregs(priv,msg)
#endif

/* CAN interrupt enable functions */

static void at32can_rx0int(struct at32_can_s *priv, bool enable);
static void at32can_rx1int(struct at32_can_s *priv, bool enable);
static void at32can_txint(struct at32_can_s *priv, bool enable);
#ifdef CONFIG_NET_CAN_ERRORS
static void at32can_errint(struct at32_can_s *priv, bool enable);
#endif

/* Common TX logic */

static int at32can_transmit(struct at32_can_s *priv);
static bool at32can_txready(struct at32_can_s *priv);
static int  at32can_txpoll(struct net_driver_s *dev);

/* CAN RX interrupt handling */

static int  at32can_rxinterrupt_work(struct at32_can_s *priv,
                                      int rxmb);

static void at32can_rx0interrupt_work(void *arg);
static void at32can_rx1interrupt_work(void *arg);
static int at32can_rxinterrupt(struct at32_can_s *priv, int rxmb);

static int  at32can_rx0interrupt(int irq, void *context, void *arg);
static int  at32can_rx1interrupt(int irq, void *context, void *arg);

/* CAN TX interrupt handling */

static int  at32can_txinterrupt(int irq, void *context, void *arg);
static void at32can_txdone_work(void *arg);
static void at32can_txdone(struct at32_can_s *priv);

#ifdef CONFIG_NET_CAN_ERRORS
/* CAN errors interrupt handling */

static void at32can_sceinterrupt_work(void *arg);
static int  at32can_sceinterrupt(int irq, void *context, void *arg);
#endif

/* Initialization */

static int  at32can_setup(struct at32_can_s *priv);
static void  at32can_shutdown(struct at32_can_s *priv);
static void  at32can_reset(struct at32_can_s *priv);
static int  at32can_enterinitmode(struct at32_can_s *priv);
static int  at32can_exitinitmode(struct at32_can_s *priv);
static int  at32can_bittiming(struct at32_can_s *priv);
static int  at32can_cellinit(struct at32_can_s *priv);
static int  at32can_filterinit(struct at32_can_s *priv);

/* TX mailbox status */

static bool at32can_txmb0empty(uint32_t tsr_regval);
static bool at32can_txmb1empty(uint32_t tsr_regval);
static bool at32can_txmb2empty(uint32_t tsr_regval);

/* NuttX callback functions */

static int  at32can_ifup(struct net_driver_s *dev);
static int  at32can_ifdown(struct net_driver_s *dev);

static void at32can_txavail_work(void *arg);
static int  at32can_txavail(struct net_driver_s *dev);

#ifdef CONFIG_NETDEV_IOCTL
static int  at32can_netdev_ioctl(struct net_driver_s *dev, int cmd,
                                  unsigned long arg);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_AT32_CAN1

static struct at32_can_s g_can1priv =
{
  .port             = 1,
  .canrx            =
  {
                      AT32_IRQ_CAN1RX0,
                      AT32_IRQ_CAN1RX1,
  },
  .cantx            = AT32_IRQ_CAN1TX,
#ifdef CONFIG_NET_CAN_ERRORS
  .cansce           = AT32_IRQ_CAN1SCE,
#endif
  .filter           = 0,
  .base             = AT32_CAN1_BASE,
  .fbase            = AT32_CAN1_BASE,
  .baud             = CONFIG_AT32_CAN1_BAUD,
};

#endif

#ifdef CONFIG_AT32_CAN2

static struct at32_can_s g_can2priv =
{
  .port             = 2,
  .canrx            =
  {
                      AT32_IRQ_CAN2RX0,
                      AT32_IRQ_CAN2RX1,
  },
  .cantx            = AT32_IRQ_CAN2TX,
#ifdef CONFIG_NET_CAN_ERRORS
  .cansce           = AT32_IRQ_CAN2SCE,
#endif
  .filter           = CAN_NFILTERS / 2,
  .base             = AT32_CAN2_BASE,
  .fbase            = AT32_CAN1_BASE,
  .baud             = CONFIG_AT32_CAN2_BAUD,
};

#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: at32can_getreg
 * Name: at32can_getfreg
 *
 * Description:
 *   Read the value of a CAN register or filter block register.
 *
 ****************************************************************************/

#ifdef CONFIG_AT32_CAN_REGDEBUG
static uint32_t at32can_vgetreg(uint32_t addr)
{
  static uint32_t prevaddr = 0;
  static uint32_t preval   = 0;
  static uint32_t count    = 0;

  /* Read the value from the register */

  uint32_t val = getreg32(addr);

  /* Is this the same value that we read from the same register last time?
   * Are we polling the register?  If so, suppress some of the output.
   */

  if (addr == prevaddr && val == preval)
    {
      if (count == 0xffffffff || ++count > 3)
        {
          if (count == 4)
            {
              ninfo("...\n");
            }

          return val;
        }
    }

  /* No this is a new address or value */

  else
    {
      /* Did we print "..." for the previous value? */

      if (count > 3)
        {
          /* Yes.. then show how many times the value repeated */

          ninfo("[repeats %" PRIu32 " more times]\n", count - 3);
        }

      /* Save the new address, value, and count */

      prevaddr = addr;
      preval   = val;
      count    = 1;
    }

  /* Show the register value read */

  ninfo("%08" PRIx32 "->%08" PRIx32 "\n", addr, val);
  return val;
}

static uint32_t at32can_getreg(struct at32_can_s *priv, int offset)
{
  return at32can_vgetreg(priv->base + offset);
}

static uint32_t at32can_getfreg(struct at32_can_s *priv, int offset)
{
  return at32can_vgetreg(priv->fbase + offset);
}

#else
static uint32_t at32can_getreg(struct at32_can_s *priv, int offset)
{
  return getreg32(priv->base + offset);
}

static uint32_t at32can_getfreg(struct at32_can_s *priv, int offset)
{
  return getreg32(priv->fbase + offset);
}

#endif

/****************************************************************************
 * Name: at32can_putreg
 * Name: at32can_putfreg
 *
 * Description:
 *   Set the value of a CAN register or filter block register.
 *
 ****************************************************************************/

#ifdef CONFIG_AT32_CAN_REGDEBUG
static void at32can_vputreg(uint32_t addr, uint32_t value)
{
  /* Show the register value being written */

  ninfo("%08" PRIx32 "->%08" PRIx32 "\n", addr, val);

  /* Write the value */

  putreg32(value, addr);
}

static void at32can_putreg(struct at32_can_s *priv, int offset,
                            uint32_t value)
{
  at32can_vputreg(priv->base + offset, value);
}

static void at32can_putfreg(struct at32_can_s *priv, int offset,
                             uint32_t value)
{
  at32can_vputreg(priv->fbase + offset, value);
}

#else
static void at32can_putreg(struct at32_can_s *priv, int offset,
                            uint32_t value)
{
  putreg32(value, priv->base + offset);
}

static void at32can_putfreg(struct at32_can_s *priv, int offset,
                             uint32_t value)
{
  putreg32(value, priv->fbase + offset);
}
#endif

/****************************************************************************
 * Name: at32can_dumpctrlregs
 *
 * Description:
 *   Dump the contents of all CAN control registers
 *
 * Input Parameters:
 *   priv - reference to the private CAN driver state structure
 *   msg  - message
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_AT32_CAN_REGDEBUG
static void at32can_dumpctrlregs(struct at32_can_s *priv,
                                  const char *msg)
{
  if (msg)
    {
      ninfo("Control Registers: %s\n", msg);
    }
  else
    {
      ninfo("Control Registers:\n");
    }

  /* CAN control and status registers */

  caninfo("  MCTRL: %08" PRIx32 "   MSTS: %08" PRIx32 "   \
  TSTS: %08" PRIx32 "\n",
          getreg32(priv->base + AT32_CAN_MCTRL_OFFSET),
          getreg32(priv->base + AT32_CAN_MSTS_OFFSET),
          getreg32(priv->base + AT32_CAN_TSTS_OFFSET));

  caninfo(" RF0: %08" PRIx32 "  RF1: %08" PRIx32 "\n",
          getreg32(priv->base + AT32_CAN_RF0_OFFSET),
          getreg32(priv->base + AT32_CAN_RF1_OFFSET));

  caninfo("  INTEN: %08" PRIx32 "   ESTS: %08" PRIx32 "   \
  BTMG: %08" PRIx32 "\n",
          getreg32(priv->base + AT32_CAN_INTEN_OFFSET),
          getreg32(priv->base + AT32_CAN_ESTS_OFFSET),
          getreg32(priv->base + AT32_CAN_BTMG_OFFSET));
}
#endif

/****************************************************************************
 * Name: at32can_dumpmbregs
 *
 * Description:
 *   Dump the contents of all CAN mailbox registers
 *
 * Input Parameters:
 *   priv - reference to the private CAN driver state structure
 *   msg  - message
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_AT32_CAN_REGDEBUG
static void at32can_dumpmbregs(struct at32_can_s *priv,
                                const char *msg)
{
  if (msg)
    {
      ninfo("Mailbox Registers: %s\n", msg);
    }
  else
    {
      ninfo("Mailbox Registers:\n");
    }

  /* CAN mailbox registers (3 TX and 2 RX) */

  caninfo(" TMI0: %08" PRIx32 " TMC0: %08" PRIx32 " TMDTL0: %08"
          PRIx32 " TMDTH0: %08" PRIx32 "\n",
          getreg32(priv->base + AT32_CAN_TMI0_FFSET),
          getreg32(priv->base + AT32_CAN_TMC0_OFFSET),
          getreg32(priv->base + AT32_CAN_TMDTL0_OFFSET),
          getreg32(priv->base + AT32_CAN_TMDTH0_OFFSET));

  caninfo(" TMI1: %08" PRIx32 " TMC1: %08" PRIx32 " TMDTL1: %08"
          PRIx32 " TMDTH1: %08" PRIx32 "\n",
          getreg32(priv->base + AT32_CAN_TMI1_FFSET),
          getreg32(priv->base + AT32_CAN_TMC1_OFFSET),
          getreg32(priv->base + AT32_CAN_TMDTL1_OFFSET),
          getreg32(priv->base + AT32_CAN_TMDTH1_OFFSET));

  caninfo(" TMI2: %08" PRIx32 " TMC2: %08" PRIx32 " TMDTL2: %08"
          PRIx32 " TMDTH2: %08" PRIx32 "\n",
          getreg32(priv->base + AT32_CAN_TMI2_FFSET),
          getreg32(priv->base + AT32_CAN_TMC2_OFFSET),
          getreg32(priv->base + AT32_CAN_TMDTL2_OFFSET),
          getreg32(priv->base + AT32_CAN_TMDTH2_OFFSET));

  caninfo(" RFI0: %08" PRIx32 " RFC0: %08" PRIx32 " RFDTL0: %08"
          PRIx32 " RFDTH0: %08" PRIx32 "\n",
          getreg32(priv->base + AT32_CAN_RFI0_OFFSET),
          getreg32(priv->base + AT32_CAN_RFC0_OFFSET),
          getreg32(priv->base + AT32_CAN_RFDTL0_OFFSET),
          getreg32(priv->base + AT32_CAN_RFDTH0_OFFSET));

  caninfo(" RFI1: %08" PRIx32 " RFC1: %08" PRIx32 " RFDTL1: %08"
          PRIx32 " RFDTH1: %08" PRIx32 "\n",
          getreg32(priv->base + AT32_CAN_RFI1_OFFSET),
          getreg32(priv->base + AT32_CAN_RFC1_OFFSET),
          getreg32(priv->base + AT32_CAN_RFDTL1_OFFSET),
          getreg32(priv->base + AT32_CAN_RFDTH1_OFFSET));
}
#endif

/****************************************************************************
 * Name: at32can_dumpfiltregs
 *
 * Description:
 *   Dump the contents of all CAN filter registers
 *
 * Input Parameters:
 *   priv - reference to the private CAN driver state structure
 *   msg  - message
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_AT32_CAN_REGDEBUG
static void at32can_dumpfiltregs(struct at32_can_s *priv,
                                  const char *msg)
{
  int i;

  if (msg)
    {
      ninfo("Filter Registers: %s\n", msg);
    }
  else
    {
      ninfo("Filter Registers:\n");
    }

  caninfo(" FCTRL: %08" PRIx32 "   FMCFG: %08" PRIx32 "  FSCFG: %08"
          PRIx32 " FRF: %08" PRIx32 "  FACFG: %08" PRIx32 "\n",
          getreg32(priv->base + AT32_CAN_FCTRL_OFFSET),
          getreg32(priv->base + AT32_CAN_FMCFG_OFFSET),
          getreg32(priv->base + AT32_CAN_FSCFG_OFFSET),
          getreg32(priv->base + AT32_CAN_FRF_OFFSET),
          getreg32(priv->base + AT32_CAN_FACFG_OFFSET));

  for (i = 0; i < CAN_NFILTERS; i++)
    {
      caninfo(" F%dR1: %08" PRIx32 " F%dR2: %08" PRIx32 "\n",
              i, getreg32(priv->base + AT32_CAN_FBF_OFFSET(i, 1)),
              i, getreg32(priv->base + AT32_CAN_FBF_OFFSET(i, 2)));
    }
}
#endif

/****************************************************************************
 * Name: at32can_rx0int
 *
 * Description:
 *   Call to enable or disable RX0 interrupts.
 *
 * Input Parameters:
 *   priv - reference to the private CAN driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void at32can_rx0int(struct at32_can_s *priv, bool enable)
{
  uint32_t regval = 0;

  ninfo("CAN%" PRIu8 "RX0 enable: %d\n", priv->port, enable);

  /* Enable/disable the FIFO 0 message pending interrupt */

  regval = at32can_getreg(priv, AT32_CAN_INTEN_OFFSET);
  if (enable)
    {
      regval |= CAN_INTEN_RF0MIEN;
    }
  else
    {
      regval &= ~CAN_INTEN_RF0MIEN;
    }

  at32can_putreg(priv, AT32_CAN_INTEN_OFFSET, regval);
}

/****************************************************************************
 * Name: at32can_rx1int
 *
 * Description:
 *   Call to enable or disable RX1 interrupts.
 *
 * Input Parameters:
 *   priv - reference to the private CAN driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void at32can_rx1int(struct at32_can_s *priv, bool enable)
{
  uint32_t regval = 0;

  ninfo("CAN%" PRIu8 "RX1 enable: %d\n", priv->port, enable);

  /* Enable/disable the FIFO 1 message pending interrupt */

  regval = at32can_getreg(priv, AT32_CAN_INTEN_OFFSET);
  if (enable)
    {
      regval |= CAN_INTEN_RF1MIEN;
    }
  else
    {
      regval &= ~CAN_INTEN_RF1MIEN;
    }

  at32can_putreg(priv, AT32_CAN_INTEN_OFFSET, regval);
}

/****************************************************************************
 * Name: at32can_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts.
 *
 * Input Parameters:
 *   priv - reference to the private CAN driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void at32can_txint(struct at32_can_s *priv, bool enable)
{
  uint32_t regval = 0;

  ninfo("CAN%" PRIu8 " txint enable: %d\n", priv->port, enable);

  /* Enable/disable the transmit mailbox interrupt */

  regval  = at32can_getreg(priv, AT32_CAN_INTEN_OFFSET);
  if (enable)
    {
      regval |= CAN_INTEN_TCIEN;
    }
  else
    {
      regval &= ~CAN_INTEN_TCIEN;
    }

  at32can_putreg(priv, AT32_CAN_INTEN_OFFSET, regval);
}

#ifdef CONFIG_NET_CAN_ERRORS
/****************************************************************************
 * Name: at32can_txint
 *
 * Description:
 *   Call to enable or disable CAN SCE interrupts.
 *
 * Input Parameters:
 *   priv - reference to the private CAN driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void at32can_errint(struct at32_can_s *priv, bool enable)
{
  uint32_t regval = 0;

  /* Enable/disable the transmit mailbox interrupt */

  regval  = at32can_getreg(priv, AT32_CAN_INTEN_OFFSET);
  if (enable)
    {
      regval |= AT32_CAN_ERRINT;
    }
  else
    {
      regval &= ~AT32_CAN_ERRINT;
    }

  at32can_putreg(priv, AT32_CAN_INTEN_OFFSET, regval);
}
#endif

/****************************************************************************
 * Function: at32can_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Ethernet interface when an IP address is
 *   provided
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int at32can_ifup(struct net_driver_s *dev)
{
  struct at32_can_s *priv = (struct at32_can_s *)dev->d_private;

  /* Setup CAN */

  at32can_setup(priv);

  /* Enable interrupts */

  at32can_rx0int(priv, true);
  at32can_rx1int(priv, true);
  at32can_txint(priv, true);
#ifdef CONFIG_NET_CAN_ERRORS
  at32can_errint(priv, true);
#endif

  /* Enable the interrupts at the NVIC */

  up_enable_irq(priv->canrx[0]);
  up_enable_irq(priv->canrx[1]);
  up_enable_irq(priv->cantx);
#ifdef CONFIG_NET_CAN_ERRORS
  up_enable_irq(priv->cansce);
#endif

  priv->bifup = true;

  priv->txdesc = (struct can_frame *)priv->tx_pool;
  priv->rxdesc = (struct can_frame *)priv->rx_pool;

  priv->dev.d_buf = (uint8_t *)priv->txdesc;

  return OK;
}

/****************************************************************************
 * Function: at32can_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int at32can_ifdown(struct net_driver_s *dev)
{
  struct at32_can_s *priv = (struct at32_can_s *)dev->d_private;

  /* Disable CAN interrupts */

  at32can_shutdown(priv);

  /* Reset CAN */

  at32can_reset(priv);

  return OK;
}

/****************************************************************************
 * Name: at32can_txready
 *
 * Description:
 *   Return true if the CAN hardware can accept another TX message.
 *
 ****************************************************************************/

static bool at32can_txready(struct at32_can_s *priv)
{
  uint32_t regval;

  /* Return true if any mailbox is available */

  regval = at32can_getreg(priv, AT32_CAN_TSTS_OFFSET);
  ninfo("CAN%" PRIu8 " TSR: %08" PRIx32 "\n", priv->port, regval);

  return (at32can_txmb0empty(regval) || at32can_txmb1empty(regval) ||
          at32can_txmb2empty(regval));
}

/****************************************************************************
 * Name: at32can_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Input Parameters:
 *   priv - reference to the private CAN driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static int at32can_transmit(struct at32_can_s *priv)
{
  struct can_frame *frame = (struct can_frame *)priv->dev.d_buf;
  uint8_t          *ptr;
  uint32_t          regval;
  uint32_t          tmp;
  int               dlc;
  int               txmb;

  ninfo("CAN%" PRIu8 " ID: %" PRIu32 " DLC: %" PRIu8 "\n",
        priv->port, (uint32_t)frame->can_id, frame->can_dlc);

  /* Select one empty transmit mailbox */

  regval = at32can_getreg(priv, AT32_CAN_TSTS_OFFSET);
  if (at32can_txmb0empty(regval))
    {
      txmb = 0;
    }
  else if (at32can_txmb1empty(regval))
    {
      txmb = 1;
    }
  else if (at32can_txmb2empty(regval))
    {
      txmb = 2;
    }
  else
    {
      canerr("ERROR: No available mailbox\n");
      return -EBUSY;
    }

  /* Clear TXRQ, RTR, IDE, EXID, and STID fields */

  regval  = at32can_getreg(priv, AT32_CAN_TMI_OFFSET(txmb));
  regval &= ~(CAN_TMI_TMSR | CAN_TMI_TMFRSEL | CAN_TMI_TMIDSEL |
              CAN_TMI_TMEID_MASK | CAN_TMI_TMSID_TMEID_MASK);
  at32can_putreg(priv, AT32_CAN_TMI_OFFSET(txmb), regval);

  /* Set up the ID, standard 11-bit or extended 29-bit. */

#ifdef CONFIG_NET_CAN_EXTID
  regval &= ~CAN_TIR_EXID_MASK;
  if (frame->can_id & CAN_EFF_FLAG)
    {
      DEBUGASSERT(frame->can_id < (1 << 29));
      regval |= (frame->can_id << CAN_TMI_TMEID_SHIFT) | CAN_TMI_TMIDSEL;
    }
  else
    {
      DEBUGASSERT(frame->can_id < (1 << 11));
      regval |= frame->can_id << CAN_TMI_TMSID_TMEID_SHIFT;
    }

#else
  regval |= (((uint32_t) frame->can_id << CAN_TMI_TMSID_TMEID_SHIFT) &
               CAN_TMI_TMSID_TMEID_MASK);

#endif

#ifdef CONFIG_CAN_USE_RTR
  regval |= ((frame->can_id & CAN_RTR_FLAG) ? CAN_TMI_TMFRSEL : 0);
#endif

  at32can_putreg(priv, AT32_CAN_TMI_OFFSET(txmb), regval);

  /* Set up the DLC */

  dlc     = frame->can_dlc;
  regval  = at32can_getreg(priv, AT32_CAN_TMC_OFFSET(txmb));
  regval &= ~(CAN_TMC_TMDTBL_MASK | CAN_TMC_TMTSTEN);
  regval |= (uint32_t)dlc << CAN_TMC_TMDTBL_SHIFT;
  at32can_putreg(priv, AT32_CAN_TMC_OFFSET(txmb), regval);

  /* Set up the data fields */

  ptr    = frame->data;
  regval = 0;

  if (dlc > 0)
    {
      tmp    = (uint32_t)*ptr++;
      regval = tmp << CAN_TMDTL_TMDT0_SHIFT;

      if (dlc > 1)
        {
          tmp     = (uint32_t)*ptr++;
          regval |= tmp << CAN_TMDTL_TMDT1_SHIFT;

          if (dlc > 2)
            {
              tmp     = (uint32_t)*ptr++;
              regval |= tmp << CAN_TMDTL_TMDT2_SHIFT;

              if (dlc > 3)
                {
                  tmp     = (uint32_t)*ptr++;
                  regval |= tmp << CAN_TMDTL_TMDT3_SHIFT;
                }
            }
        }
    }

  at32can_putreg(priv, AT32_CAN_TMDTL_OFFSET(txmb), regval);

  regval = 0;
  if (dlc > 4)
    {
      tmp    = (uint32_t)*ptr++;
      regval = tmp << CAN_TMDTH_TMDT4_SHIFT;

      if (dlc > 5)
        {
          tmp     = (uint32_t)*ptr++;
          regval |= tmp << CAN_TMDTH_TMDT5_SHIFT;

          if (dlc > 6)
            {
              tmp     = (uint32_t)*ptr++;
              regval |= tmp << CAN_TMDTH_TMDT6_SHIFT;

              if (dlc > 7)
                {
                  tmp     = (uint32_t)*ptr++;
                  regval |= tmp << CAN_TMDTH_TMDT7_SHIFT;
                }
            }
        }
    }

  at32can_putreg(priv, AT32_CAN_TMDTH_OFFSET(txmb), regval);

  /* Enable the transmit mailbox empty interrupt (may already be enabled) */

  regval  = at32can_getreg(priv, AT32_CAN_INTEN_OFFSET);
  regval |= CAN_INTEN_TCIEN;
  at32can_putreg(priv, AT32_CAN_INTEN_OFFSET, regval);

  /* Request transmission */

  regval  = at32can_getreg(priv, AT32_CAN_TMI_OFFSET(txmb));
  regval |= CAN_TMI_TMSR;  /* Transmit Mailbox Request */
  at32can_putreg(priv, AT32_CAN_TMI_OFFSET(txmb), regval);

  at32can_dumpmbregs(priv, "After send");
  return OK;
}

/****************************************************************************
 * Function: at32can_txpoll
 *
 * Description:
 *   The transmitter is available, check if the network has any outgoing
 *   packets ready to send.  This is a callback from devif_poll().
 *   devif_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. When the preceding TX packet send timesout and the interface is reset
 *   3. During normal TX polling
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static int at32can_txpoll(struct net_driver_s *dev)
{
  struct at32_can_s *priv = (struct at32_can_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (priv->dev.d_len > 0)
    {
      at32can_txdone(priv);

      /* Send the packet */

      at32can_transmit(priv);

      /* Check if there is room in the device to hold another packet. If
       * not, return a non-zero value to terminate the poll.
       */

      if (at32can_txready(priv) == false)
        {
          return -EBUSY;
        }
    }

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return 0;
}

/****************************************************************************
 * Function: at32can_txavail_work
 *
 * Description:
 *   Perform an out-of-cycle poll on the worker thread.
 *
 * Input Parameters:
 *   arg - Reference to the NuttX driver state structure (cast to void*)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called on the higher priority worker thread.
 *
 ****************************************************************************/

static void at32can_txavail_work(void *arg)
{
  struct at32_can_s *priv = (struct at32_can_s *)arg;

  /* Ignore the notification if the interface is not yet up */

  net_lock();
  if (priv->bifup)
    {
      /* Check if there is room in the hardware to hold another outgoing
       * packet.
       */

      if (at32can_txready(priv))
        {
          /* No, there is space for another transfer.  Poll the network for
           * new XMIT data.
           */

          devif_poll(&priv->dev, at32can_txpoll);
        }
    }

  net_unlock();
}

/****************************************************************************
 * Function: at32can_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called in normal user mode
 *
 ****************************************************************************/

static int at32can_txavail(struct net_driver_s *dev)
{
  struct at32_can_s *priv = (struct at32_can_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      at32can_txavail_work(priv);
    }

  return OK;
}

/****************************************************************************
 * Function: at32can_ioctl
 *
 * Description:
 *   PHY ioctl command handler
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   cmd  - ioctl command
 *   arg  - Argument accompanying the command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IOCTL
static int at32can_netdev_ioctl(struct net_driver_s *dev, int cmd,
                                 unsigned long arg)
{
#if 0
  struct at32_can_s *priv = (struct at32_can_s *)dev->d_private;
#endif
  int ret  = OK;

  switch (cmd)
    {
      /* TODO */

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}
#endif /* CONFIG_NETDEV_IOCTL */

/****************************************************************************
 * Name: at32can_rxinterrupt_work
 *
 * Description:
 *   CAN RX FIFO 0/1 interrupt handler
 *
 * Input Parameters:
 *   irq - The IRQ number of the interrupt.
 *   context - The register state save array at the time of the interrupt.
 *   rxmb - The RX mailbox number.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int at32can_rxinterrupt_work(struct at32_can_s *priv, int rxmb)
{
  struct can_frame *frame = (struct can_frame *)priv->rxdesc;
  uint32_t          regval;
  int               ret   = OK;

  DEBUGASSERT(priv != NULL);

  if (rxmb == 0)
    {
      at32can_dumpmbregs(priv, "RX0 interrupt");
    }
  else
    {
      at32can_dumpmbregs(priv, "RX1 interrupt");
    }

  /* Get the CAN identifier. */

  regval = at32can_getreg(priv, AT32_CAN_RFI_OFFSET(rxmb));

#ifdef CONFIG_NET_CAN_EXTID
  if ((regval & CAN_RDI_RFIDI) != 0)
    {
      frame->can_id  = (regval & CAN_RDI_RFEID_MASK) >> CAN_RDI_RFEID_SHIFT;
      frame->can_id |= CAN_EFF_FLAG;
    }
  else
    {
      frame->can_id  = \
      (regval & CAN_RDI_RFSID_RFEID_MASK) >> CAN_RDI_RFSID_RFEID_SHIFT;
    }
#else
  if ((regval & CAN_RDI_RFIDI) != 0)
    {
      nerr("ERROR: Received message with extended identifier.  Dropped\n");
      ret = -ENOSYS;
      goto errout;
    }

  frame->can_id = \
  (regval & CAN_RDI_RFSID_RFEID_MASK) >> CAN_RDI_RFSID_RFEID_SHIFT;
#endif

  /* Extract the RTR bit */

  if ((regval & CAN_RDI_RFFRI) != 0)
    {
      frame->can_id |= CAN_RTR_FLAG;
    }

  /* Get the DLC */

  regval        = at32can_getreg(priv, AT32_CAN_RFC_OFFSET(rxmb));
  frame->can_dlc = (regval & CAN_RFC_RFDTL_MASK) >> CAN_RFC_RFDTL_SHIFT;

  /* Save the message data */

  regval  = at32can_getreg(priv, AT32_CAN_RFDTL_OFFSET(rxmb));
  frame->data[0] = (regval & CAN_RFDTL_RFDT0_MASK) >> CAN_RFDTL_RFDT0_SHIFT;
  frame->data[1] = (regval & CAN_RFDTL_RFDT1_MASK) >> CAN_RFDTL_RFDT1_SHIFT;
  frame->data[2] = (regval & CAN_RFDTL_RFDT2_MASK) >> CAN_RFDTL_RFDT2_SHIFT;
  frame->data[3] = (regval & CAN_RFDTL_RFDT3_MASK) >> CAN_RFDTL_RFDT3_SHIFT;

  regval  = at32can_getreg(priv, AT32_CAN_RFDTH_OFFSET(rxmb));
  frame->data[4] = (regval & CAN_RFDTH_RFDT4_MASK) >> CAN_RFDTH_RFDT4_SHIFT;
  frame->data[5] = (regval & CAN_RFDTH_RFDT5_MASK) >> CAN_RFDTH_RFDT5_SHIFT;
  frame->data[6] = (regval & CAN_RFDTH_RFDT6_MASK) >> CAN_RFDTH_RFDT6_SHIFT;
  frame->data[7] = (regval & CAN_RFDTH_RFDT7_MASK) >> CAN_RFDTH_RFDT7_SHIFT;

  /* Copy the buffer pointer to priv->dev..  Set amount of data
   * in priv->dev.d_len
   */

  priv->dev.d_len = sizeof(struct can_frame);
  priv->dev.d_buf = (uint8_t *)frame;

  /* Send to socket interface */

  NETDEV_RXPACKETS(&priv->dev);

  can_input(&priv->dev);

  /* Point the packet buffer back to the next Tx buffer that will be
   * used during the next write.  If the write queue is full, then
   * this will point at an active buffer, which must not be written
   * to.  This is OK because devif_poll won't be called unless the
   * queue is not full.
   */

  priv->dev.d_buf = (uint8_t *)priv->txdesc;

  /* Release the FIFO */

#ifndef CONFIG_NET_CAN_EXTID
errout:
#endif
  regval  = at32can_getreg(priv, AT32_CAN_RF_OFFSET(rxmb));
  regval |= CAN_RF_RFR;
  at32can_putreg(priv, AT32_CAN_RF_OFFSET(rxmb), regval);

  /* Re-enable CAN RX interrupts */

  if (rxmb == 0)
    {
      at32can_rx0int(priv, true);
    }
  else if (rxmb == 1)
    {
      at32can_rx1int(priv, true);
    }
  else
    {
      DEBUGPANIC();
    }

  return ret;
}

/****************************************************************************
 * Name: at32can_rx0interrupt_work
 ****************************************************************************/

static void at32can_rx0interrupt_work(void *arg)
{
  struct at32_can_s *priv = (struct at32_can_s *)arg;
  at32can_rxinterrupt_work(priv, 0);
}

/****************************************************************************
 * Name: at32can_rx1interrupt_work
 ****************************************************************************/

static void at32can_rx1interrupt_work(void *arg)
{
  struct at32_can_s *priv = (struct at32_can_s *)arg;
  at32can_rxinterrupt_work(priv, 1);
}

/****************************************************************************
 * Name: at32can_rxinterrupt
 *
 * Description:
 *   CAN RX FIFO common interrupt handler
 *
 ****************************************************************************/

static int at32can_rxinterrupt(struct at32_can_s *priv, int rxmb)
{
  uint32_t regval   = 0;
  int      npending = 0;

  /* Verify that a message is pending in the FIFO */

  regval   = at32can_getreg(priv, AT32_CAN_RF_OFFSET(rxmb));
  npending = (regval & CAN_RF_RFMN_MASK) >> CAN_RF_RFMN_SHIFT;
  if (npending < 1)
    {
      nwarn("WARNING: No messages pending\n");
      return OK;
    }

  /* Disable further CAN RX interrupts and schedule to perform the
   * interrupt processing on the worker thread
   */

  if (rxmb == 0)
    {
      at32can_rx0int(priv, false);
      work_queue(CANWORK, &priv->irqwork,
                 at32can_rx0interrupt_work, priv, 0);
    }
  else if (rxmb == 1)
    {
      at32can_rx1int(priv, false);
      work_queue(CANWORK, &priv->irqwork,
                 at32can_rx1interrupt_work, priv, 0);
    }
  else
    {
      DEBUGPANIC();
    }

  return OK;
}

/****************************************************************************
 * Name: at32can_rx0interrupt
 *
 * Description:
 *   CAN RX FIFO 0 interrupt handler
 *
 * Input Parameters:
 *   irq - The IRQ number of the interrupt.
 *   context - The register state save array at the time of the interrupt.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int at32can_rx0interrupt(int irq, void *context, void *arg)
{
  struct at32_can_s *priv = (struct at32_can_s *)arg;
  return at32can_rxinterrupt(priv, 0);
}

/****************************************************************************
 * Name: at32can_rx1interrupt
 *
 * Description:
 *   CAN RX FIFO 1 interrupt handler
 *
 * Input Parameters:
 *   irq - The IRQ number of the interrupt.
 *   context - The register state save array at the time of the interrupt.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int at32can_rx1interrupt(int irq, void *context, void *arg)
{
  struct at32_can_s *priv = (struct at32_can_s *)arg;
  return at32can_rxinterrupt(priv, 1);
}

/****************************************************************************
 * Name: at32can_txinterrupt
 *
 * Description:
 *   CAN TX mailbox complete interrupt handler
 *
 * Input Parameters:
 *   irq - The IRQ number of the interrupt.
 *   context - The register state save array at the time of the interrupt.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int at32can_txinterrupt(int irq, void *context, void *arg)
{
  struct at32_can_s *priv = (struct at32_can_s *)arg;
  uint32_t regval;

  DEBUGASSERT(priv != NULL);

  /* Get the transmit status */

  regval = at32can_getreg(priv, AT32_CAN_TSTS_OFFSET);

  /* Check for RQCP0: Request completed mailbox 0 */

  if ((regval & CAN_TSTS_TM0TCF) != 0)
    {
      /* Writing '1' to RCP0 clears RCP0 and all the status bits (TXOK0,
       * ALST0 and TERR0) for Mailbox 0.
       */

      at32can_putreg(priv, AT32_CAN_TSTS_OFFSET, CAN_TSTS_TM0TCF);

      /* Tell the upper half that the transfer is finished. */

      /* Disable further TX CAN interrupts. here can be no race
       * condition here.
       */

      at32can_txint(priv, false);
      work_queue(CANWORK, &priv->irqwork, at32can_txdone_work, priv, 0);
    }

  /* Check for RQCP1: Request completed mailbox 1 */

  if ((regval & CAN_TSTS_TM1TCF) != 0)
    {
      /* Writing '1' to RCP1 clears RCP1 and all the status bits (TXOK1,
       * ALST1 and TERR1) for Mailbox 1.
       */

      at32can_putreg(priv, AT32_CAN_TSTS_OFFSET, CAN_TSTS_TM1TCF);

      /* Tell the upper half that the transfer is finished. */

      /* Disable further TX CAN interrupts. here can be no race
       * condition here.
       */

      at32can_txint(priv, false);
      work_queue(CANWORK, &priv->irqwork, at32can_txdone_work, priv, 0);
    }

  /* Check for RQCP2: Request completed mailbox 2 */

  if ((regval & CAN_TSTS_TM2TCF) != 0)
    {
      /* Writing '1' to RCP2 clears RCP2 and all the status bits (TXOK2,
       * ALST2 and TERR2) for Mailbox 2.
       */

      at32can_putreg(priv, AT32_CAN_TSTS_OFFSET, CAN_TSTS_TM2TCF);

      /* Disable further TX CAN interrupts. here can be no race
       * condition here.
       */

      at32can_txint(priv, false);
      work_queue(CANWORK, &priv->irqwork, at32can_txdone_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: at32can_txdone_work
 ****************************************************************************/

static void at32can_txdone_work(void *arg)
{
  struct at32_can_s *priv = (struct at32_can_s *)arg;

  at32can_txdone(priv);

  /* There should be space for a new TX in any event.  Poll the network for
   * new XMIT data
   */

  net_lock();
  devif_poll(&priv->dev, at32can_txpoll);
  net_unlock();
}

/****************************************************************************
 * Name: at32can_txdone
 ****************************************************************************/

static void at32can_txdone(struct at32_can_s *priv)
{
  at32can_txint(priv, true);

  NETDEV_TXDONE(&priv->dev);
}

#ifdef CONFIG_NET_CAN_ERRORS

/****************************************************************************
 * Name: at32can_sceinterrupt_work
 *
 * Description:
 *   CAN status change interrupt work
 *
 * Input Parameters:
 *   arg  - reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void at32can_sceinterrupt_work(void *arg)
{
  struct at32_can_s *priv    = (struct at32_can_s *)arg;
  struct can_frame   *frame   = (struct can_frame *)priv->rxdesc;
  uint32_t            regval  = 0;
  uint16_t            errbits = 0;
  uint8_t             data[CAN_ERR_DLC];

  DEBUGASSERT(priv != NULL);

  /* Check Error Interrupt flag */

  regval = at32can_getreg(priv, AT32_CAN_MSTS_OFFSET);
  if (regval & CAN_MSTS_EOIF)
    {
      /* Encode error bits */

      errbits = 0;
      memset(data, 0, sizeof(data));

      /* Get Error statur register */

      regval = at32can_getreg(priv, AT32_CAN_ESTS_OFFSET);

      if (regval & CAN_ESTS_EAF)
        {
          /* Error warning flag */

          data[1] |= (CAN_ERR_CRTL_RX_WARNING | CAN_ERR_CRTL_TX_WARNING);
          errbits |= CAN_ERR_CRTL;
        }

      if (regval & CAN_ESTS_EPF)
        {
          /* Error passive flag */

          data[1] |= (CAN_ERR_CRTL_RX_PASSIVE | CAN_ERR_CRTL_TX_PASSIVE);
          errbits |= CAN_ERR_CRTL;
        }

      if (regval & CAN_ESTS_BOF)
        {
          /* Bus-off flag */

          errbits |= CAN_ERR_BUSOFF;
        }

      /* Last error code */

      if (regval & CAN_ESTS_ETR_MASK)
        {
          if (regval & CAN_ESTS_ETR_STUFF)
            {
              /* Stuff Error */

              errbits |= CAN_ERR_PROT;
              data[2] |= CAN_ERR_PROT_STUFF;
            }
          else if (regval & CAN_ESTS_ETR_FORM)
            {
              /* Format Error */

              errbits |= CAN_ERR_PROT;
              data[2] |= CAN_ERR_PROT_FORM;
            }
          else if (regval & CAN_ESTS_ETR_ACK)
            {
              /* Acknowledge Error */

              errbits |= CAN_ERR_ACK;
            }
          else if (regval & CAN_ESTS_ETR_BREC)
            {
              /* Bit recessive Error */

              errbits |= CAN_ERR_PROT;
              data[2] |= CAN_ERR_PROT_BIT1;
            }
          else if (regval & CAN_ESTS_ETR_BDOM)
            {
              /* Bit dominant Error */

              errbits |= CAN_ERR_PROT;
              data[2] |= CAN_ERR_PROT_BIT0;
            }
          else if (regval & CAN_ESTS_ETR_CRC)
            {
              /* Receive CRC Error */

              errbits |= CAN_ERR_PROT;
              data[3] |= CAN_ERR_PROT_LOC_CRC_SEQ;
            }
        }

      /* Get transmit status register */

      regval = at32can_getreg(priv, AT32_CAN_TSTS_OFFSET);

      if (regval & CAN_TSTS_TM0ALF || regval & CAN_TSTS_TM1ALF ||
          regval & CAN_TSTS_TM2ALF)
        {
          /* Lost arbitration Error */

          errbits |= CAN_ERR_LOSTARB;
        }

      /* Clear TSR register */

      at32can_putreg(priv, AT32_CAN_TSTS_OFFSET, regval);

      /* Clear ERRI flag */

      at32can_putreg(priv, AT32_CAN_MSTS_OFFSET, CAN_MSTS_EOIF);
    }

  /* Report a CAN error */

  if (errbits != 0)
    {
      canerr("ERROR: errbits = %08" PRIx16 "\n", errbits);

      /* Copy frame */

      frame->can_id  = errbits;
      frame->can_dlc = CAN_ERR_DLC;

      memcpy(frame->data, data, CAN_ERR_DLC);

      net_lock();

      /* Copy the buffer pointer to priv->dev..  Set amount of data
       * in priv->dev.d_len
       */

      priv->dev.d_len = sizeof(struct can_frame);
      priv->dev.d_buf = (uint8_t *)frame;

      /* Send to socket interface */

      NETDEV_ERRORS(&priv->dev);

      can_input(&priv->dev);

      /* Point the packet buffer back to the next Tx buffer that will be
       * used during the next write.  If the write queue is full, then
       * this will point at an active buffer, which must not be written
       * to.  This is OK because devif_poll won't be called unless the
       * queue is not full.
       */

      priv->dev.d_buf = (uint8_t *)priv->txdesc;
      net_unlock();
    }

  /* Re-enable CAN SCE interrupts */

  at32can_errint(priv, true);
}

/****************************************************************************
 * Name: at32can_sceinterrupt
 *
 * Description:
 *   CAN status change interrupt handler
 *
 * Input Parameters:
 *   irq - The IRQ number of the interrupt.
 *   context - The register state save array at the time of the interrupt.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int at32can_sceinterrupt(int irq, void *context, void *arg)
{
  struct at32_can_s *priv = (struct at32_can_s *)arg;

  /* Disable further CAN SCE interrupts and schedule to perform the
   * interrupt processing on the worker thread
   */

  at32can_errint(priv, false);
  work_queue(CANWORK, &priv->irqwork,
             at32can_sceinterrupt_work, priv, 0);

  return OK;
}
#endif

/****************************************************************************
 * Name: at32can_bittiming
 *
 * Description:
 *   Set the CAN bit timing register (BTR) based on the configured BAUD.
 *
 * "The bit timing logic monitors the serial bus-line and performs sampling
 *  and adjustment of the sample point by synchronizing on the start-bit edge
 *  and resynchronizing on the following edges.
 *
 * "Its operation may be explained simply by splitting nominal bit time into
 *  three segments as follows:
 *
 * 1. "Synchronization segment (SYNC_SEG): a bit change is expected to occur
 *     within this time segment. It has a fixed length of one time quantum
 *     (1 x tCAN).
 * 2. "Bit segment 1 (BS1): defines the location of the sample point. It
 *     includes the PROP_SEG and PHASE_SEG1 of the CAN standard. Its duration
 *     is programmable between 1 and 16 time quanta but may be automatically
 *     lengthened to compensate for positive phase drifts due to differences
 *     in the frequency of the various nodes of the network.
 * 3. "Bit segment 2 (BS2): defines the location of the transmit point. It
 *     represents the PHASE_SEG2 of the CAN standard. Its duration is
 *     programmable between 1 and 8 time quanta but may also be automatically
 *     shortened to compensate for negative phase drifts."
 *
 * Pictorially:
 *
 *  |<----------------- NOMINAL BIT TIME ----------------->|
 *  |<- SYNC_SEG ->|<------ BS1 ------>|<------ BS2 ------>|
 *  |<---- Tq ---->|<----- Tbs1 ------>|<----- Tbs2 ------>|
 *
 * Where
 *   Tbs1 is the duration of the BS1 segment
 *   Tbs2 is the duration of the BS2 segment
 *   Tq is the "Time Quantum"
 *
 * Relationships:
 *
 *   baud = 1 / bit_time
 *   bit_time = Tq + Tbs1 + Tbs2
 *   Tbs1 = Tq * ts1
 *   Tbs2 = Tq * ts2
 *   Tq = brp * Tpclk1
 *   baud = Fpclk1 / (brp  * (1 + ts1 + ts2))
 *
 * Where:
 *   Tpclk1 is the period of the APB1 clock (PCLK1).
 *
 * Input Parameters:
 *   priv - reference to the private CAN driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int at32can_bittiming(struct at32_can_s *priv)
{
  uint32_t tmp;
  uint32_t brp;
  uint32_t ts1;
  uint32_t ts2;

  ninfo("CAN%" PRIu8 " PCLK1: %lu baud: %" PRIu32 "\n",
          priv->port, (unsigned long) AT32_PCLK1_FREQUENCY, priv->baud);

  /* Try to get CAN_BIT_QUANTA quanta in one bit_time.
   *
   *   bit_time = Tq*(ts1 + ts2 + 1)
   *   nquanta  = bit_time / Tq
   *   nquanta  = (ts1 + ts2 + 1)
   *
   *   bit_time = brp * Tpclk1 * (ts1 + ts2 + 1)
   *   nquanta  = bit_time / brp / Tpclk1
   *            = PCLK1 / baud / brp
   *   brp      = PCLK1 / baud / nquanta;
   *
   * Example:
   *   PCLK1 = 144,000,000 baud = 500,000 nquanta = 13 : brp = 2
   */

  tmp = AT32_PCLK1_FREQUENCY / priv->baud;
  if (tmp < CAN_BIT_QUANTA)
    {
      /* At the smallest brp value (1), there are already too few bit times
       * (PCLCK1 / baud) to meet our goal.  brp must be one and we need
       * make some reasonable guesses about ts1 and ts2.
       */

      brp = 1;

      /* In this case, we have to guess a good value for ts1 and ts2 */

      ts1 = (tmp - 1) >> 1;
      ts2 = tmp - ts1 - 1;
      if (ts1 == ts2 && ts1 > 1 && ts2 < CAN_BTR_TSEG2_MAX)
        {
          ts1--;
          ts2++;
        }
    }

  /* Otherwise, nquanta is CAN_BIT_QUANTA, ts1 is CONFIG_AT32_CAN_TSEG1,
   * ts2 is CONFIG_AT32_CAN_TSEG2 and we calculate brp to achieve
   * CAN_BIT_QUANTA quanta in the bit time
   */

  else
    {
      ts1 = CONFIG_AT32_CAN_TSEG1;
      ts2 = CONFIG_AT32_CAN_TSEG2;
      brp = (tmp + (CAN_BIT_QUANTA / 2)) / CAN_BIT_QUANTA;
      DEBUGASSERT(brp >= 1 && brp <= CAN_BTR_BRP_MAX);
    }

  ninfo("TS1: %" PRIu32 " TS2: %" PRIu32 " BRP: %" PRIu32 "\n",
               ts1, ts2, brp);

  /* Configure bit timing.  This also does the following, less obvious
   * things.  Unless loopback mode is enabled, it:
   *
   * - Disables silent mode.
   * - Disables loopback mode.
   *
   * NOTE that for the time being, SJW is set to 1 just because I don't
   * know any better.
   */

  tmp = ((brp - 1) << CAN_BTMG_BRDIV_SHIFT) | \
  ((ts1 - 1) << CAN_BTMG_BTS1_SHIFT) | \
  ((ts2 - 1) << CAN_BTMG_BTS2_SHIFT) | \
  ((1 - 1) << CAN_BTMG_RSAW_SHIFT);

#ifdef CONFIG_CAN_LOOPBACK
  /* tmp |= (CAN_BTMG_LBEN | CAN_BTMG_LOEN); */

  tmp |= CAN_BTMG_LBEN;
#endif

  at32can_putreg(priv, AT32_CAN_BTMG_OFFSET, tmp);
  return OK;
}

/****************************************************************************
 * Name: at32can_setup
 ****************************************************************************/

static int  at32can_setup(struct at32_can_s *priv)
{
  int ret;

#ifdef CONFIG_NET_CAN_ERRORS
  ninfo("CAN%" PRIu8 " RX0 irq: %" PRIu8 " RX1 irq: %" PRIu8
        " TX irq: %" PRIu8 " SCE irq: %" PRIu8 "\n",
        priv->port, priv->canrx[0], priv->canrx[1], priv->cantx,
        priv->cansce);
#else
  ninfo("CAN%" PRIu8 " RX0 irq: %" PRIu8 " RX1 irq: %" PRIu8
        " TX irq: %" PRIu8 "\n",
        priv->port, priv->canrx[0], priv->canrx[1], priv->cantx);
#endif

  /* CAN cell initialization */

  ret = at32can_cellinit(priv);
  if (ret < 0)
    {
      nerr("ERROR: CAN%" PRId8 " cell initialization failed: %d\n",
             priv->port, ret);
      return ret;
    }

  at32can_dumpctrlregs(priv, "After cell initialization");
  at32can_dumpmbregs(priv, NULL);

  /* CAN filter initialization */

  ret = at32can_filterinit(priv);
  if (ret < 0)
    {
      nerr("ERROR: CAN%" PRIu8 " filter initialization failed: %d\n",
             priv->port, ret);
      return ret;
    }

  at32can_dumpfiltregs(priv, "After filter initialization");

  /* Attach the CAN RX FIFO 0/1 interrupts and TX interrupts.
   * The others are not used.
   */

  ret = irq_attach(priv->canrx[0], at32can_rx0interrupt, priv);
  if (ret < 0)
    {
      nerr("ERROR: Failed to attach CAN%" PRIu8 " RX0 IRQ (%" PRIu8 ")",
             priv->port, priv->canrx[0]);
      return ret;
    }

  ret = irq_attach(priv->canrx[1], at32can_rx1interrupt, priv);
  if (ret < 0)
    {
      nerr("ERROR: Failed to attach CAN%" PRIu8 " RX1 IRQ (%" PRIu8 ")",
             priv->port, priv->canrx[1]);
      return ret;
    }

  ret = irq_attach(priv->cantx, at32can_txinterrupt, priv);
  if (ret < 0)
    {
      nerr("ERROR: Failed to attach CAN%" PRIu8 " TX IRQ (%" PRIu8 ")",
             priv->port, priv->cantx);
      return ret;
    }

#ifdef CONFIG_NET_CAN_ERRORS
  ret = irq_attach(priv->cansce, at32can_sceinterrupt, priv);
  if (ret < 0)
    {
      nerr("ERROR: Failed to attach CAN%" PRIu8 " SCE IRQ (%" PRIu8 ")",
           priv->port, priv->cansce);
      return ret;
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: at32can_shutdown
 ****************************************************************************/

static void at32can_shutdown(struct at32_can_s *priv)
{
  ninfo("CAN%" PRIu8 "\n", priv->port);

  /* Disable the RX FIFO 0/1 and TX interrupts */

  up_disable_irq(priv->canrx[0]);
  up_disable_irq(priv->canrx[1]);
  up_disable_irq(priv->cantx);
#ifdef CONFIG_NET_CAN_ERRORS
  up_disable_irq(priv->cansce);
#endif

  /* Detach the RX FIFO 0/1 and TX interrupts */

  irq_detach(priv->canrx[0]);
  irq_detach(priv->canrx[1]);
  irq_detach(priv->cantx);
#ifdef CONFIG_NET_CAN_ERRORS
  irq_detach(priv->cansce);
#endif
}

/****************************************************************************
 * Name: at32can_reset
 *
 * Description:
 *   Put the CAN device in the non-operational, reset state
 *
 * Input Parameters:
 *   priv - reference to the private CAN driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void at32can_reset(struct at32_can_s *priv)
{
  uint32_t regval;
  uint32_t regbit = 0;
  irqstate_t flags;

  ninfo("CAN%" PRIu8 "\n", priv->port);

  /* Get the bits in the AHB1RSTR register needed to reset this CAN device */

#ifdef CONFIG_AT32_CAN1
  if (priv->port == 1)
    {
      regbit = CRM_APB1RST_CAN1RST;
    }
  else
#endif
#ifdef CONFIG_AT32_CAN2
  if (priv->port == 2)
    {
      regbit = CRM_APB1RST_CAN2RST;
    }
  else
#endif
    {
      nerr("ERROR: Unsupported port %d\n", priv->port);
      return;
    }

  /* Disable interrupts momentarily to stop any ongoing CAN event processing
   * and to prevent any concurrent access to the AHB1RSTR register.
   */

  flags = enter_critical_section();

  /* Reset the CAN */

  regval  = getreg32(AT32_CRM_APB1RST);
  regval |= regbit;
  putreg32(regval, AT32_CRM_APB1RST);

  regval &= ~regbit;
  putreg32(regval, AT32_CRM_APB1RST);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: at32can_enterinitmode
 *
 * Description:
 *   Put the CAN cell in Initialization mode. This only disconnects the CAN
 *   peripheral, no registers are changed. The initialization mode is
 *   required to change the baud rate.
 *
 * Input Parameters:
 *   priv - reference to the private CAN driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int at32can_enterinitmode(struct at32_can_s *priv)
{
  uint32_t regval;
  volatile uint32_t timeout;

  ninfo("CAN%" PRIu8 "\n", priv->port);

  /* Enter initialization mode */

  regval  = at32can_getreg(priv, AT32_CAN_MCTRL_OFFSET);
  regval |= CAN_MCTRL_FZEN;
  at32can_putreg(priv, AT32_CAN_MCTRL_OFFSET, regval);

  /* Wait until initialization mode is acknowledged */

  for (timeout = INAK_TIMEOUT; timeout > 0; timeout--)
    {
      regval = at32can_getreg(priv, AT32_CAN_MSTS_OFFSET);
      if ((regval & CAN_MSTS_FZC) != 0)
        {
          /* We are in initialization mode */

          break;
        }
    }

  /* Check for a timeout */

  if (timeout < 1)
    {
      nerr("ERROR: Timed out waiting to enter initialization mode\n");
      return -ETIMEDOUT;
    }

  return OK;
}

/****************************************************************************
 * Name: at32can_exitinitmode
 *
 * Description:
 *   Put the CAN cell out of the Initialization mode (to Normal mode)
 *
 * Input Parameters:
 *   priv - reference to the private CAN driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int at32can_exitinitmode(struct at32_can_s *priv)
{
  uint32_t regval;
  volatile uint32_t timeout;

  /* Exit Initialization mode, enter Normal mode */

  regval  = at32can_getreg(priv, AT32_CAN_MCTRL_OFFSET);
  regval &= ~CAN_MCTRL_FZEN;
  at32can_putreg(priv, AT32_CAN_MCTRL_OFFSET, regval);

  /* Wait until the initialization mode exit is acknowledged */

  for (timeout = INAK_TIMEOUT; timeout > 0; timeout--)
    {
      regval = at32can_getreg(priv, AT32_CAN_MSTS_OFFSET);
      if ((regval & CAN_MSTS_FZC) == 0)
        {
          /* We are out of initialization mode */

          break;
        }
    }

  /* Check for a timeout */

  if (timeout < 1)
    {
      nerr("ERROR: Timed out waiting to exit initialization mode: %08"
                  PRIx32 "\n", regval);
      return -ETIMEDOUT;
    }

  return OK;
}

/****************************************************************************
 * Name: at32can_cellinit
 *
 * Description:
 *   CAN cell initialization
 *
 * Input Parameters:
 *   priv - reference to the private CAN driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int at32can_cellinit(struct at32_can_s *priv)
{
  uint32_t regval;
  int ret;

  ninfo("CAN%" PRIu8 "\n", priv->port);

  /* Exit from sleep mode */

  regval  = at32can_getreg(priv, AT32_CAN_MCTRL_OFFSET);
  regval &= ~CAN_MCTRL_DZEN;
  at32can_putreg(priv, AT32_CAN_MCTRL_OFFSET, regval);

  ret = at32can_enterinitmode(priv);
  if (ret != 0)
    {
      return ret;
    }

  /* Disable the following modes:
   *
   *  - Time triggered communication mode
   *  - Automatic bus-off management
   *  - Automatic wake-up mode
   *  - No automatic retransmission
   *  - Receive FIFO locked mode
   *
   * Enable:
   *
   *  - Transmit FIFO priority
   */

  regval  = at32can_getreg(priv, AT32_CAN_MCTRL_OFFSET);
  regval &= ~(CAN_MCTRL_MDRSEL | CAN_MCTRL_PRSFEN | CAN_MCTRL_AEDEN |
              CAN_MCTRL_AEBOEN | CAN_MCTRL_TTCEN);
  regval |=  CAN_MCTRL_MMSSR;
  at32can_putreg(priv, AT32_CAN_MCTRL_OFFSET, regval);

  /* Configure bit timing. */

  ret = at32can_bittiming(priv);
  if (ret < 0)
    {
      nerr("ERROR: Failed to set bit timing: %d\n", ret);
      return ret;
    }

  return at32can_exitinitmode(priv);
}

/****************************************************************************
 * Name: at32can_filterinit
 *
 * Description:
 *   CAN filter initialization.  CAN filters are not currently used by this
 *   driver.  The CAN filters can be configured in a different way:
 *
 *   1. As a match of specific IDs in a list (IdList mode), or as
 *   2. And ID and a mask (IdMask mode).
 *
 *   Filters can also be configured as:
 *
 *   3. 16- or 32-bit.  The advantage of 16-bit filters is that you get
 *      more filters;  The advantage of 32-bit filters is that you get
 *      finer control of the filtering.
 *
 *   One filter is set up for each CAN.  The filter resources are shared
 *   between the two CAN modules:  CAN1 uses only filter 0 (but reserves
 *   0 through CAN_NFILTERS/2-1); CAN2 uses only filter CAN_NFILTERS/2
 *   (but reserves CAN_NFILTERS/2 through CAN_NFILTERS-1).
 *
 *   32-bit IdMask mode is configured.  However, both the ID and the MASK
 *   are set to zero thus suppressing all filtering because anything masked
 *   with zero matches zero.
 *
 * Input Parameters:
 *   priv - reference to the private CAN driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int at32can_filterinit(struct at32_can_s *priv)
{
  uint32_t regval;
  uint32_t bitmask;

  ninfo("CAN%" PRIu8 " filter: %" PRIu8 "\n", priv->port, priv->filter);

  /* Get the bitmask associated with the filter used by this CAN block */

  bitmask = (uint32_t)1 << priv->filter;

  /* Enter filter initialization mode */

  regval  = at32can_getfreg(priv, AT32_CAN_FCTRL_OFFSET);
  regval |= CAN_FCTRL_FCS;
  at32can_putfreg(priv, AT32_CAN_FCTRL_OFFSET, regval);

  /* Disable the filter */

  regval  = at32can_getfreg(priv, AT32_CAN_FACFG_OFFSET);
  regval &= ~bitmask;
  at32can_putfreg(priv, AT32_CAN_FACFG_OFFSET, regval);

  /* Select the 32-bit scale for the filter */

  regval  = at32can_getfreg(priv, AT32_CAN_FSCFG_OFFSET);
  regval |= bitmask;
  at32can_putfreg(priv, AT32_CAN_FSCFG_OFFSET, regval);

  /* There are 14 or 28 filter banks (depending) on the device.
   * Each filter bank is composed of two 32-bit registers, CAN_FiR:
   */

  at32can_putfreg(priv, AT32_CAN_FBF_OFFSET(priv->filter, 1), 0);
  at32can_putfreg(priv, AT32_CAN_FBF_OFFSET(priv->filter, 2), 0);

  /* Set Id/Mask mode for the filter */

  regval  = at32can_getfreg(priv, AT32_CAN_FMCFG_OFFSET);
  regval &= ~bitmask;
  at32can_putfreg(priv, AT32_CAN_FMCFG_OFFSET, regval);

  /* Assign FIFO 0 for the filter */

  regval  = at32can_getfreg(priv, AT32_CAN_FRF_OFFSET);
  regval &= ~bitmask;
  at32can_putfreg(priv, AT32_CAN_FRF_OFFSET, regval);

  /* Enable the filter */

  regval  = at32can_getfreg(priv, AT32_CAN_FACFG_OFFSET);
  regval |= bitmask;
  at32can_putfreg(priv, AT32_CAN_FACFG_OFFSET, regval);

  /* Exit filter initialization mode */

  regval  = at32can_getfreg(priv, AT32_CAN_FCTRL_OFFSET);
  regval &= ~CAN_FCTRL_FCS;
  at32can_putfreg(priv, AT32_CAN_FCTRL_OFFSET, regval);
  return OK;
}

/****************************************************************************
 * Name: at32can_txmb0empty
 *
 * Input Parameters:
 *   tsr_regval - value of CAN transmit status register
 *
 * Returned Value:
 *   Returns true if mailbox 0 is empty and can be used for sending.
 *
 ****************************************************************************/

static bool at32can_txmb0empty(uint32_t tsr_regval)
{
  return (tsr_regval & CAN_TSTS_TM0EF) != 0 &&
         (tsr_regval & CAN_TSTS_TM0TCF) == 0;
}

/****************************************************************************
 * Name: at32can_txmb1empty
 *
 * Input Parameters:
 *   tsr_regval - value of CAN transmit status register
 *
 * Returned Value:
 *   Returns true if mailbox 1 is empty and can be used for sending.
 *
 ****************************************************************************/

static bool at32can_txmb1empty(uint32_t tsr_regval)
{
  return (tsr_regval & CAN_TSTS_TM1EF) != 0 &&
         (tsr_regval & CAN_TSTS_TM1TCF) == 0;
}

/****************************************************************************
 * Name: at32can_txmb2empty
 *
 * Input Parameters:
 *   tsr_regval - value of CAN transmit status register
 *
 * Returned Value:
 *   Returns true if mailbox 2 is empty and can be used for sending.
 *
 ****************************************************************************/

static bool at32can_txmb2empty(uint32_t tsr_regval)
{
  return (tsr_regval & CAN_TSTS_TM2EF) != 0 &&
         (tsr_regval & CAN_TSTS_TM2TCF) == 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: at32_cansockinitialize
 *
 * Description:
 *   Initialize the selected CAN port as CAN socket interface
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple CAN interfaces)
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int at32_cansockinitialize(int port)
{
  struct at32_can_s *priv = NULL;
  int                 ret  = OK;

  ninfo("CAN%" PRIu8 "\n", port);

  /* NOTE:  Peripherical clocking for CAN1 and/or CAN2 was already provided
   * by at32_clockconfig() early in the reset sequence.
   */

#ifdef CONFIG_AT32_CAN1
  if (port == 1)
    {
      /* Select the CAN1 device structure */

      priv = &g_can1priv;

      /* Configure CAN1 pins.  The ambiguous settings in the at32*_pinmap.h
       * file must have been disambiguated in the board.h file.
       */

      at32_configgpio(GPIO_CAN1_RX);
      at32_configgpio(GPIO_CAN1_TX);
    }
  else
#endif
#ifdef CONFIG_AT32_CAN2
  if (port == 2)
    {
      /* Select the CAN2 device structure */

      priv = &g_can2priv;

      /* Configure CAN2 pins.  The ambiguous settings in the at32*_pinmap.h
       * file must have been disambiguated in the board.h file.
       */

      at32_configgpio(GPIO_CAN2_RX);
      at32_configgpio(GPIO_CAN2_TX);
    }
  else
#endif
    {
      nerr("ERROR: Unsupported port %d\n", port);
      ret = -EINVAL;
      goto errout;
    }

  /* Initialize the driver structure */

  priv->dev.d_ifup    = at32can_ifup;
  priv->dev.d_ifdown  = at32can_ifdown;
  priv->dev.d_txavail = at32can_txavail;
#ifdef CONFIG_NETDEV_IOCTL
  priv->dev.d_ioctl   = at32can_netdev_ioctl;
#endif
  priv->dev.d_private = priv;

  /* Put the interface in the down state.  This usually amounts to resetting
   * the device and/or calling at32can_ifdown().
   */

  ninfo("callbacks done\n");

  at32can_ifdown(&priv->dev);

  /* Register the device with the OS so that socket IOCTLs can be performed */

  ret = netdev_register(&priv->dev, NET_LL_CAN);

errout:
  return ret;
}

/****************************************************************************
 * Name: arm_netinitialize
 *
 * Description:
 *   Initialize the CAN device interfaces.  If there is more than one device
 *   interface in the chip, then board-specific logic will have to provide
 *   this function to determine which, if any, CAN interfaces should be
 *   initialized.
 *
 ****************************************************************************/

#if !defined(CONFIG_NETDEV_LATEINIT) && !defined(CONFIG_AT32_ETHMAC)
void arm_netinitialize(void)
{
#ifdef CONFIG_AT32_CAN1
  at32_cansockinitialize(1);
#endif

#ifdef CONFIG_AT32_CAN2
  at32_cansockinitialize(2);
#endif
}
#endif

