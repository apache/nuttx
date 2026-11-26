/****************************************************************************
 * arch/arm/src/common/stm32/stm32_can_bxcan_sock.c
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

#include <inttypes.h>
#include <stdio.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include <nuttx/can.h>
#include <nuttx/net/can.h>
#include <nuttx/net/netdev_lowerhalf.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32.h"
#include "stm32_rcc.h"
#include "stm32_can.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Delays *******************************************************************/

/* Time out for INAK bit */

#define INAK_TIMEOUT 65535

/* Bit timing ***************************************************************/

#define CAN_BIT_QUANTA (CONFIG_STM32_CAN_TSEG1 + CONFIG_STM32_CAN_TSEG2 + 1)

#ifndef CONFIG_DEBUG_CAN_INFO
#  undef CONFIG_STM32_CAN_REGDEBUG
#endif

/* Number of TX mailboxes */

#define CAN_NTXMB 3

/* CAN error interrupts */

#ifdef CONFIG_NET_CAN_ERRORS
#  define STM32_CAN_ERRINT (CAN_IER_LECIE | CAN_IER_ERRIE | \
                            CAN_IER_BOFIE | CAN_IER_EPVIE | \
                            CAN_IER_EWGIE)
#endif

/* RCC reset of the CAN instances */

#ifdef STM32_RCC_APB1RSTR1
#  define CAN_RCC_RSTR              STM32_RCC_APB1RSTR1
#  define CAN_RCC_CAN1RST           RCC_APB1RSTR1_CAN1RST
#  define CAN_RCC_CAN2RST           RCC_APB1RSTR1_CAN2RST
#else
#  define CAN_RCC_RSTR              STM32_RCC_APB1RSTR
#  define CAN_RCC_CAN1RST           RCC_APB1RSTR_CAN1RST
#  define CAN_RCC_CAN2RST           RCC_APB1RSTR_CAN2RST
#  define CAN_RCC_CAN3RST           RCC_APB1RSTR_CAN3RST
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_can_s
{
  struct netdev_lowerhalf_s dev; /* Lower half network device, first */

  uint8_t  port;     /* CAN port number (1 to 3) */
  uint8_t  canrx[2]; /* CAN RX FIFO 0/1 IRQ number */
  uint8_t  cantx;    /* CAN TX IRQ number */
#ifdef CONFIG_NET_CAN_ERRORS
  uint8_t  cansce;   /* CAN SCE IRQ number */
#endif
  uint8_t  filter;   /* Filter number */
  uint32_t base;     /* Base address of the CAN control registers */
  uint32_t fbase;    /* Base address of the CAN filter registers */
  uint32_t baud;     /* Configured baud */
  bool     active;   /* Between ifup and ifdown */
#ifdef CONFIG_NET_CAN_ERRORS
  bool     errpending; /* Error interrupt waits for receive() */
#endif
  netpkt_t *txpkt[CAN_NTXMB]; /* Packets held by the TX mailboxes */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* CAN Register access */

static uint32_t stm32can_getreg(struct stm32_can_s *priv,
                                int offset);
static uint32_t stm32can_getfreg(struct stm32_can_s *priv,
                                 int offset);
static void stm32can_putreg(struct stm32_can_s *priv, int offset,
                            uint32_t value);
static void stm32can_putfreg(struct stm32_can_s *priv, int offset,
                             uint32_t value);
#ifdef CONFIG_STM32_CAN_REGDEBUG
static void stm32can_dumpctrlregs(struct stm32_can_s *priv,
                                  const char *msg);
static void stm32can_dumpmbregs(struct stm32_can_s *priv,
                                const char *msg);
static void stm32can_dumpfiltregs(struct stm32_can_s *priv,
                                  const char *msg);
#else
#  define stm32can_dumpctrlregs(priv,msg)
#  define stm32can_dumpmbregs(priv,msg)
#  define stm32can_dumpfiltregs(priv,msg)
#endif

/* CAN interrupt enable functions */

static void stm32can_rx0int(struct stm32_can_s *priv, bool enable);
static void stm32can_rx1int(struct stm32_can_s *priv, bool enable);
static void stm32can_txint(struct stm32_can_s *priv, bool enable);
#ifdef CONFIG_NET_CAN_ERRORS
static void stm32can_errint(struct stm32_can_s *priv, bool enable);
#endif

/* TX and RX */

static int  stm32can_reclaim(struct stm32_can_s *priv);
static netpkt_t *stm32can_rxframe(struct stm32_can_s *priv, int rxmb);
#ifdef CONFIG_NET_CAN_ERRORS
static netpkt_t *stm32can_errframe(struct stm32_can_s *priv);
#endif

/* Interrupt handling */

static int  stm32can_rxinterrupt(struct stm32_can_s *priv, int rxmb);
static int  stm32can_rx0interrupt(int irq, void *context, void *arg);
static int  stm32can_rx1interrupt(int irq, void *context, void *arg);
static int  stm32can_txinterrupt(int irq, void *context, void *arg);
#ifdef CONFIG_NET_CAN_ERRORS
static int  stm32can_sceinterrupt(int irq, void *context, void *arg);
#endif

/* Initialization */

static int  stm32can_setup(struct stm32_can_s *priv);
static void  stm32can_shutdown(struct stm32_can_s *priv);
static void  stm32can_reset(struct stm32_can_s *priv);
static int  stm32can_enterinitmode(struct stm32_can_s *priv);
static int  stm32can_exitinitmode(struct stm32_can_s *priv);
static int  stm32can_bittiming(struct stm32_can_s *priv);
static int  stm32can_cellinit(struct stm32_can_s *priv);
static int  stm32can_filterinit(struct stm32_can_s *priv);

/* TX mailbox status */

static bool stm32can_txmb0empty(uint32_t tsr_regval);
static bool stm32can_txmb1empty(uint32_t tsr_regval);
static bool stm32can_txmb2empty(uint32_t tsr_regval);

/* NuttX callback functions */

static int  stm32can_ifup(struct netdev_lowerhalf_s *dev);
static int  stm32can_ifdown(struct netdev_lowerhalf_s *dev);
static int  stm32can_transmit(struct netdev_lowerhalf_s *dev,
                              netpkt_t *pkt);
static netpkt_t *stm32can_receive(struct netdev_lowerhalf_s *dev);
#ifdef CONFIG_NET_NETPOLL
static void stm32can_txpoll(struct netdev_lowerhalf_s *dev);
#endif
#ifdef CONFIG_NETDEV_IOCTL
static int  stm32can_netdev_ioctl(struct netdev_lowerhalf_s *dev, int cmd,
                                  unsigned long arg);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct netdev_ops_s g_can_ops =
{
  .ifup     = stm32can_ifup,
  .ifdown   = stm32can_ifdown,
  .transmit = stm32can_transmit,
  .receive  = stm32can_receive,
#ifdef CONFIG_NET_NETPOLL
  .txpoll   = stm32can_txpoll,
#endif
#ifdef CONFIG_NETDEV_IOCTL
  .ioctl    = stm32can_netdev_ioctl,
#endif
};

#ifdef CONFIG_STM32_CAN1

static struct stm32_can_s g_can1priv =
{
  .port             = 1,
  .canrx            =
  {
                      STM32_IRQ_CAN1RX0,
                      STM32_IRQ_CAN1RX1,
  },
  .cantx            = STM32_IRQ_CAN1TX,
#ifdef CONFIG_NET_CAN_ERRORS
  .cansce           = STM32_IRQ_CAN1SCE,
#endif
  .filter           = 0,
  .base             = STM32_CAN1_BASE,
  .fbase            = STM32_CAN1_BASE,
  .baud             = CONFIG_STM32_CAN1_BAUD,
};

#endif

#ifdef CONFIG_STM32_CAN2

static struct stm32_can_s g_can2priv =
{
  .port             = 2,
  .canrx            =
  {
                      STM32_IRQ_CAN2RX0,
                      STM32_IRQ_CAN2RX1,
  },
  .cantx            = STM32_IRQ_CAN2TX,
#ifdef CONFIG_NET_CAN_ERRORS
  .cansce           = STM32_IRQ_CAN2SCE,
#endif
  .filter           = CAN_NFILTERS / 2,
  .base             = STM32_CAN2_BASE,
  .fbase            = STM32_CAN1_BASE,
  .baud             = CONFIG_STM32_CAN2_BAUD,
};

#endif

#ifdef CONFIG_STM32_CAN3

static struct stm32_can_s g_can3priv =
{
  .port             = 3,
  .canrx            =
  {
    STM32_IRQ_CAN3RX0,
    STM32_IRQ_CAN3RX1,
  },
  .cantx            = STM32_IRQ_CAN3TX,
#ifdef CONFIG_NET_CAN_ERRORS
  .cansce           = STM32_IRQ_CAN3SCE,
#endif
  .filter           = 0,
  .base             = STM32_CAN3_BASE,
  .fbase            = STM32_CAN3_BASE,
  .baud             = CONFIG_STM32_CAN3_BAUD,
};

#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32can_getreg
 * Name: stm32can_getfreg
 *
 * Description:
 *   Read the value of a CAN register or filter block register.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_CAN_REGDEBUG
static uint32_t stm32can_vgetreg(uint32_t addr)
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

static uint32_t stm32can_getreg(struct stm32_can_s *priv, int offset)
{
  return stm32can_vgetreg(priv->base + offset);
}

static uint32_t stm32can_getfreg(struct stm32_can_s *priv, int offset)
{
  return stm32can_vgetreg(priv->fbase + offset);
}

#else
static uint32_t stm32can_getreg(struct stm32_can_s *priv, int offset)
{
  return getreg32(priv->base + offset);
}

static uint32_t stm32can_getfreg(struct stm32_can_s *priv, int offset)
{
  return getreg32(priv->fbase + offset);
}

#endif

/****************************************************************************
 * Name: stm32can_putreg
 * Name: stm32can_putfreg
 *
 * Description:
 *   Set the value of a CAN register or filter block register.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_CAN_REGDEBUG
static void stm32can_vputreg(uint32_t addr, uint32_t value)
{
  /* Show the register value being written */

  ninfo("%08" PRIx32 "->%08" PRIx32 "\n", addr, value);

  /* Write the value */

  putreg32(value, addr);
}

static void stm32can_putreg(struct stm32_can_s *priv, int offset,
                            uint32_t value)
{
  stm32can_vputreg(priv->base + offset, value);
}

static void stm32can_putfreg(struct stm32_can_s *priv, int offset,
                             uint32_t value)
{
  stm32can_vputreg(priv->fbase + offset, value);
}

#else
static void stm32can_putreg(struct stm32_can_s *priv, int offset,
                            uint32_t value)
{
  putreg32(value, priv->base + offset);
}

static void stm32can_putfreg(struct stm32_can_s *priv, int offset,
                             uint32_t value)
{
  putreg32(value, priv->fbase + offset);
}
#endif

/****************************************************************************
 * Name: stm32can_dumpctrlregs
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

#ifdef CONFIG_STM32_CAN_REGDEBUG
static void stm32can_dumpctrlregs(struct stm32_can_s *priv,
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

  ninfo("  MCR: %08" PRIx32 "   MSR: %08" PRIx32 "   TSR: %08" PRIx32 "\n",
          getreg32(priv->base + STM32_CAN_MCR_OFFSET),
          getreg32(priv->base + STM32_CAN_MSR_OFFSET),
          getreg32(priv->base + STM32_CAN_TSR_OFFSET));

  ninfo(" RF0R: %08" PRIx32 "  RF1R: %08" PRIx32 "\n",
          getreg32(priv->base + STM32_CAN_RF0R_OFFSET),
          getreg32(priv->base + STM32_CAN_RF1R_OFFSET));

  ninfo("  IER: %08" PRIx32 "   ESR: %08" PRIx32 "   BTR: %08" PRIx32 "\n",
          getreg32(priv->base + STM32_CAN_IER_OFFSET),
          getreg32(priv->base + STM32_CAN_ESR_OFFSET),
          getreg32(priv->base + STM32_CAN_BTR_OFFSET));
}
#endif

/****************************************************************************
 * Name: stm32can_dumpmbregs
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

#ifdef CONFIG_STM32_CAN_REGDEBUG
static void stm32can_dumpmbregs(struct stm32_can_s *priv,
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

  ninfo(" TI0R: %08" PRIx32 " TDT0R: %08" PRIx32 " TDL0R: %08"
          PRIx32 " TDH0R: %08" PRIx32 "\n",
          getreg32(priv->base + STM32_CAN_TI0R_OFFSET),
          getreg32(priv->base + STM32_CAN_TDT0R_OFFSET),
          getreg32(priv->base + STM32_CAN_TDL0R_OFFSET),
          getreg32(priv->base + STM32_CAN_TDH0R_OFFSET));

  ninfo(" TI1R: %08" PRIx32 " TDT1R: %08" PRIx32 " TDL1R: %08"
          PRIx32 " TDH1R: %08" PRIx32 "\n",
          getreg32(priv->base + STM32_CAN_TI1R_OFFSET),
          getreg32(priv->base + STM32_CAN_TDT1R_OFFSET),
          getreg32(priv->base + STM32_CAN_TDL1R_OFFSET),
          getreg32(priv->base + STM32_CAN_TDH1R_OFFSET));

  ninfo(" TI2R: %08" PRIx32 " TDT2R: %08" PRIx32 " TDL2R: %08"
          PRIx32 " TDH2R: %08" PRIx32 "\n",
          getreg32(priv->base + STM32_CAN_TI2R_OFFSET),
          getreg32(priv->base + STM32_CAN_TDT2R_OFFSET),
          getreg32(priv->base + STM32_CAN_TDL2R_OFFSET),
          getreg32(priv->base + STM32_CAN_TDH2R_OFFSET));

  ninfo(" RI0R: %08" PRIx32 " RDT0R: %08" PRIx32 " RDL0R: %08"
          PRIx32 " RDH0R: %08" PRIx32 "\n",
          getreg32(priv->base + STM32_CAN_RI0R_OFFSET),
          getreg32(priv->base + STM32_CAN_RDT0R_OFFSET),
          getreg32(priv->base + STM32_CAN_RDL0R_OFFSET),
          getreg32(priv->base + STM32_CAN_RDH0R_OFFSET));

  ninfo(" RI1R: %08" PRIx32 " RDT1R: %08" PRIx32 " RDL1R: %08"
          PRIx32 " RDH1R: %08" PRIx32 "\n",
          getreg32(priv->base + STM32_CAN_RI1R_OFFSET),
          getreg32(priv->base + STM32_CAN_RDT1R_OFFSET),
          getreg32(priv->base + STM32_CAN_RDL1R_OFFSET),
          getreg32(priv->base + STM32_CAN_RDH1R_OFFSET));
}
#endif

/****************************************************************************
 * Name: stm32can_dumpfiltregs
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

#ifdef CONFIG_STM32_CAN_REGDEBUG
static void stm32can_dumpfiltregs(struct stm32_can_s *priv,
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

  ninfo(" FMR: %08" PRIx32 "   FM1R: %08" PRIx32 "  FS1R: %08"
          PRIx32 " FFA1R: %08" PRIx32 "  FA1R: %08" PRIx32 "\n",
          getreg32(priv->base + STM32_CAN_FMR_OFFSET),
          getreg32(priv->base + STM32_CAN_FM1R_OFFSET),
          getreg32(priv->base + STM32_CAN_FS1R_OFFSET),
          getreg32(priv->base + STM32_CAN_FFA1R_OFFSET),
          getreg32(priv->base + STM32_CAN_FA1R_OFFSET));

  for (i = 0; i < CAN_NFILTERS; i++)
    {
      ninfo(" F%dR1: %08" PRIx32 " F%dR2: %08" PRIx32 "\n",
              i, getreg32(priv->base + STM32_CAN_FIR_OFFSET(i, 1)),
              i, getreg32(priv->base + STM32_CAN_FIR_OFFSET(i, 2)));
    }
}
#endif

/****************************************************************************
 * Name: stm32can_rx0int
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

static void stm32can_rx0int(struct stm32_can_s *priv, bool enable)
{
  uint32_t regval = 0;

  ninfo("CAN%" PRIu8 "RX0 enable: %d\n", priv->port, enable);

  /* Enable/disable the FIFO 0 message pending interrupt */

  regval = stm32can_getreg(priv, STM32_CAN_IER_OFFSET);
  if (enable)
    {
      regval |= CAN_IER_FMPIE0;
    }
  else
    {
      regval &= ~CAN_IER_FMPIE0;
    }

  stm32can_putreg(priv, STM32_CAN_IER_OFFSET, regval);
}

/****************************************************************************
 * Name: stm32can_rx1int
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

static void stm32can_rx1int(struct stm32_can_s *priv, bool enable)
{
  uint32_t regval = 0;

  ninfo("CAN%" PRIu8 "RX1 enable: %d\n", priv->port, enable);

  /* Enable/disable the FIFO 1 message pending interrupt */

  regval = stm32can_getreg(priv, STM32_CAN_IER_OFFSET);
  if (enable)
    {
      regval |= CAN_IER_FMPIE1;
    }
  else
    {
      regval &= ~CAN_IER_FMPIE1;
    }

  stm32can_putreg(priv, STM32_CAN_IER_OFFSET, regval);
}

/****************************************************************************
 * Name: stm32can_txint
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

static void stm32can_txint(struct stm32_can_s *priv, bool enable)
{
  uint32_t regval = 0;

  ninfo("CAN%" PRIu8 " txint enable: %d\n", priv->port, enable);

  /* Enable/disable the transmit mailbox interrupt */

  regval  = stm32can_getreg(priv, STM32_CAN_IER_OFFSET);
  if (enable)
    {
      regval |= CAN_IER_TMEIE;
    }
  else
    {
      regval &= ~CAN_IER_TMEIE;
    }

  stm32can_putreg(priv, STM32_CAN_IER_OFFSET, regval);
}

#ifdef CONFIG_NET_CAN_ERRORS
/****************************************************************************
 * Name: stm32can_txint
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

static void stm32can_errint(struct stm32_can_s *priv, bool enable)
{
  uint32_t regval = 0;

  /* Enable/disable the transmit mailbox interrupt */

  regval  = stm32can_getreg(priv, STM32_CAN_IER_OFFSET);
  if (enable)
    {
      regval |= STM32_CAN_ERRINT;
    }
  else
    {
      regval &= ~STM32_CAN_ERRINT;
    }

  stm32can_putreg(priv, STM32_CAN_IER_OFFSET, regval);
}
#endif

/****************************************************************************
 * Function: stm32can_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the CAN interface.
 *
 * Input Parameters:
 *   dev  - Reference to the lower half driver state structure
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int stm32can_ifup(struct netdev_lowerhalf_s *dev)
{
  struct stm32_can_s *priv = (struct stm32_can_s *)dev;
  int ret;

  /* Setup CAN */

  ret = stm32can_setup(priv);
  if (ret < 0)
    {
      stm32can_shutdown(priv);
      return ret;
    }

  priv->active = true;

  /* Enable interrupts */

  stm32can_rx0int(priv, true);
  stm32can_rx1int(priv, true);
  stm32can_txint(priv, true);
#ifdef CONFIG_NET_CAN_ERRORS
  stm32can_errint(priv, true);
#endif

  /* Enable the interrupts at the NVIC */

  up_enable_irq(priv->canrx[0]);
  up_enable_irq(priv->canrx[1]);
  up_enable_irq(priv->cantx);
#ifdef CONFIG_NET_CAN_ERRORS
  up_enable_irq(priv->cansce);
#endif

  netdev_lower_carrier_on(dev);
  return OK;
}

/****************************************************************************
 * Function: stm32can_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Input Parameters:
 *   dev  - Reference to the lower half driver state structure
 *
 * Returned Value:
 *   Zero (OK)
 *
 ****************************************************************************/

static int stm32can_ifdown(struct netdev_lowerhalf_s *dev)
{
  struct stm32_can_s *priv = (struct stm32_can_s *)dev;
  int i;

  netdev_lower_carrier_off(dev);

  /* Disable CAN interrupts and reset CAN */

  stm32can_shutdown(priv);
  priv->active = false;
  stm32can_reset(priv);

  /* Free the packets of the TX mailboxes */

  for (i = 0; i < CAN_NTXMB; i++)
    {
      if (priv->txpkt[i] != NULL)
        {
          netpkt_free(dev, priv->txpkt[i], NETPKT_TX);
          priv->txpkt[i] = NULL;
        }
    }

#ifdef CONFIG_NET_CAN_ERRORS
  priv->errpending = false;
#endif

  return OK;
}

/****************************************************************************
 * Name: stm32can_transmit
 *
 * Description:
 *   NuttX callback: put one frame into a free TX mailbox.
 *
 * Input Parameters:
 *   dev - Reference to the lower half driver state structure
 *   pkt - The packet with the CAN frame
 *
 * Returned Value:
 *   OK on success, -EBUSY if no mailbox is free, -EINVAL for a bad frame.
 *
 ****************************************************************************/

static int stm32can_transmit(struct netdev_lowerhalf_s *dev, netpkt_t *pkt)
{
  struct stm32_can_s *priv  = (struct stm32_can_s *)dev;
  struct can_frame   *frame = (struct can_frame *)netpkt_getdata(dev, pkt);
  uint8_t            *ptr;
  uint32_t            regval;
  uint32_t            tmp;
  irqstate_t          flags;
  int                 dlc;
  int                 txmb;

  /* The socket checks the frame size only, check the data length */

  if (frame->can_dlc > CAN_MAX_DLEN)
    {
      return -EINVAL;
    }

  ninfo("CAN%" PRIu8 " ID: %" PRIu32 " DLC: %" PRIu8 "\n",
        priv->port, (uint32_t)frame->can_id, frame->can_dlc);

  /* Select one empty transmit mailbox */

  flags  = enter_critical_section();
  regval = stm32can_getreg(priv, STM32_CAN_TSR_OFFSET);
  if (stm32can_txmb0empty(regval))
    {
      txmb = 0;
    }
  else if (stm32can_txmb1empty(regval))
    {
      txmb = 1;
    }
  else if (stm32can_txmb2empty(regval))
    {
      txmb = 2;
    }
  else
    {
      leave_critical_section(flags);
      return -EBUSY;
    }

  /* Clear TXRQ, RTR, IDE, EXID, and STID fields */

  regval  = stm32can_getreg(priv, STM32_CAN_TIR_OFFSET(txmb));
  regval &= ~(CAN_TIR_TXRQ | CAN_TIR_RTR | CAN_TIR_IDE |
              CAN_TIR_EXID_MASK | CAN_TIR_STID_MASK);
  stm32can_putreg(priv, STM32_CAN_TIR_OFFSET(txmb), regval);

  /* Set up the ID, standard 11-bit or extended 29-bit. */

#ifdef CONFIG_NET_CAN_EXTID
  regval &= ~CAN_TIR_EXID_MASK;
  if (frame->can_id & CAN_EFF_FLAG)
    {
      DEBUGASSERT((frame->can_id ^ CAN_EFF_FLAG) < (1 << 29));
      regval |= (frame->can_id << CAN_TIR_EXID_SHIFT) | CAN_TIR_IDE;
    }
  else
    {
      DEBUGASSERT(frame->can_id < (1 << 11));
      regval |= frame->can_id << CAN_TIR_STID_SHIFT;
    }

#else
  regval |= (((uint32_t) frame->can_id << CAN_TIR_STID_SHIFT) &
               CAN_TIR_STID_MASK);

#endif

#ifdef CONFIG_CAN_USE_RTR
  regval |= ((frame->can_id & CAN_RTR_FLAG) ? CAN_TIR_RTR : 0);
#endif

  stm32can_putreg(priv, STM32_CAN_TIR_OFFSET(txmb), regval);

  /* Set up the DLC */

  dlc     = frame->can_dlc;
  regval  = stm32can_getreg(priv, STM32_CAN_TDTR_OFFSET(txmb));
  regval &= ~(CAN_TDTR_DLC_MASK | CAN_TDTR_TGT);
  regval |= (uint32_t)dlc << CAN_TDTR_DLC_SHIFT;
  stm32can_putreg(priv, STM32_CAN_TDTR_OFFSET(txmb), regval);

  /* Set up the data fields */

  ptr    = frame->data;
  regval = 0;

  if (dlc > 0)
    {
      tmp    = (uint32_t)*ptr++;
      regval = tmp << CAN_TDLR_DATA0_SHIFT;

      if (dlc > 1)
        {
          tmp     = (uint32_t)*ptr++;
          regval |= tmp << CAN_TDLR_DATA1_SHIFT;

          if (dlc > 2)
            {
              tmp     = (uint32_t)*ptr++;
              regval |= tmp << CAN_TDLR_DATA2_SHIFT;

              if (dlc > 3)
                {
                  tmp     = (uint32_t)*ptr++;
                  regval |= tmp << CAN_TDLR_DATA3_SHIFT;
                }
            }
        }
    }

  stm32can_putreg(priv, STM32_CAN_TDLR_OFFSET(txmb), regval);

  regval = 0;
  if (dlc > 4)
    {
      tmp    = (uint32_t)*ptr++;
      regval = tmp << CAN_TDHR_DATA4_SHIFT;

      if (dlc > 5)
        {
          tmp     = (uint32_t)*ptr++;
          regval |= tmp << CAN_TDHR_DATA5_SHIFT;

          if (dlc > 6)
            {
              tmp     = (uint32_t)*ptr++;
              regval |= tmp << CAN_TDHR_DATA6_SHIFT;

              if (dlc > 7)
                {
                  tmp     = (uint32_t)*ptr++;
                  regval |= tmp << CAN_TDHR_DATA7_SHIFT;
                }
            }
        }
    }

  stm32can_putreg(priv, STM32_CAN_TDHR_OFFSET(txmb), regval);

  priv->txpkt[txmb] = pkt;

  /* Enable the transmit mailbox empty interrupt (may already be enabled) */

  regval  = stm32can_getreg(priv, STM32_CAN_IER_OFFSET);
  regval |= CAN_IER_TMEIE;
  stm32can_putreg(priv, STM32_CAN_IER_OFFSET, regval);

  /* Request transmission */

  regval  = stm32can_getreg(priv, STM32_CAN_TIR_OFFSET(txmb));
  regval |= CAN_TIR_TXRQ;  /* Transmit Mailbox Request */
  stm32can_putreg(priv, STM32_CAN_TIR_OFFSET(txmb), regval);

  leave_critical_section(flags);

  stm32can_dumpmbregs(priv, "After send");
  return OK;
}

/****************************************************************************
 * Name: stm32can_reclaim
 *
 * Description:
 *   Free the packets of the finished TX mailboxes.  Called with interrupts
 *   disabled: from the TX interrupt and stm32can_txpoll().
 *
 * Input Parameters:
 *   priv - reference to the private CAN driver state structure
 *
 * Returned Value:
 *   The number of finished mailboxes.
 *
 ****************************************************************************/

static int stm32can_reclaim(struct stm32_can_s *priv)
{
  uint32_t regval;
  uint32_t rqcp;
  int      ndone = 0;
  int      i;

  regval = stm32can_getreg(priv, STM32_CAN_TSR_OFFSET);

  for (i = 0; i < CAN_NTXMB; i++)
    {
      rqcp = CAN_TSR_RQCP0 << (8 * i);
      if ((regval & rqcp) == 0)
        {
          continue;
        }

      /* Writing '1' to RQCPx clears RQCPx and all the status bits (TXOKx,
       * ALSTx and TERRx) for mailbox x.
       */

      stm32can_putreg(priv, STM32_CAN_TSR_OFFSET, rqcp);

      if (priv->txpkt[i] != NULL)
        {
          if ((regval & (CAN_TSR_TXOK0 << (8 * i))) == 0)
            {
              NETDEV_TXERRORS(&priv->dev.netdev);
            }

          netpkt_free(&priv->dev, priv->txpkt[i], NETPKT_TX);
          priv->txpkt[i] = NULL;
        }

      ndone++;
    }

  return ndone;
}

/****************************************************************************
 * Name: stm32can_txpoll
 *
 * Description:
 *   NuttX callback: free the packets of the finished TX mailboxes for
 *   netpoll, which may run with or without interrupts disabled.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_NETPOLL
static void stm32can_txpoll(struct netdev_lowerhalf_s *dev)
{
  irqstate_t flags = enter_critical_section();

  stm32can_reclaim((struct stm32_can_s *)dev);
  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Function: stm32can_netdev_ioctl
 *
 * Description:
 *   PHY ioctl command handler
 *
 * Input Parameters:
 *   dev  - Reference to the lower half driver state structure
 *   cmd  - ioctl command
 *   arg  - Argument accompanying the command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IOCTL
static int stm32can_netdev_ioctl(struct netdev_lowerhalf_s *dev, int cmd,
                                 unsigned long arg)
{
  int ret = OK;

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
 * Name: stm32can_rxframe
 *
 * Description:
 *   Read the next message of an RX FIFO into a packet and release the FIFO
 *   element.  The message is dropped if no packet is free.
 *
 * Input Parameters:
 *   priv - reference to the private CAN driver state structure
 *   rxmb - The RX FIFO number
 *
 * Returned Value:
 *   The received packet, or NULL if the message was dropped.
 *
 ****************************************************************************/

static netpkt_t *stm32can_rxframe(struct stm32_can_s *priv, int rxmb)
{
  struct can_frame *frame;
  netpkt_t         *pkt;
  uint32_t          regval;

  if (rxmb == 0)
    {
      stm32can_dumpmbregs(priv, "RX0 interrupt");
    }
  else
    {
      stm32can_dumpmbregs(priv, "RX1 interrupt");
    }

  pkt = netpkt_alloc(&priv->dev, NETPKT_RX);
  if (pkt == NULL)
    {
      NETDEV_RXDROPPED(&priv->dev.netdev);
      goto release;
    }

  netpkt_setdatalen(&priv->dev, pkt, sizeof(struct can_frame));
  frame = (struct can_frame *)netpkt_getdata(&priv->dev, pkt);
  memset(frame, 0, sizeof(*frame));

  /* Get the CAN identifier. */

  regval = stm32can_getreg(priv, STM32_CAN_RIR_OFFSET(rxmb));

#ifdef CONFIG_NET_CAN_EXTID
  if ((regval & CAN_RIR_IDE) != 0)
    {
      frame->can_id  = (regval & CAN_RIR_EXID_MASK) >> CAN_RIR_EXID_SHIFT;
      frame->can_id |= CAN_EFF_FLAG;
    }
  else
    {
      frame->can_id  = (regval & CAN_RIR_STID_MASK) >> CAN_RIR_STID_SHIFT;
    }
#else
  if ((regval & CAN_RIR_IDE) != 0)
    {
      nerr("ERROR: Received message with extended identifier.  Dropped\n");
      netpkt_free(&priv->dev, pkt, NETPKT_RX);
      pkt = NULL;
      goto release;
    }

  frame->can_id = (regval & CAN_RIR_STID_MASK) >> CAN_RIR_STID_SHIFT;
#endif

  /* Extract the RTR bit */

  if ((regval & CAN_RIR_RTR) != 0)
    {
      frame->can_id |= CAN_RTR_FLAG;
    }

  /* Get the DLC */

  regval        = stm32can_getreg(priv, STM32_CAN_RDTR_OFFSET(rxmb));
  frame->can_dlc = (regval & CAN_RDTR_DLC_MASK) >> CAN_RDTR_DLC_SHIFT;

  /* Save the message data */

  regval  = stm32can_getreg(priv, STM32_CAN_RDLR_OFFSET(rxmb));
  frame->data[0] = (regval & CAN_RDLR_DATA0_MASK) >> CAN_RDLR_DATA0_SHIFT;
  frame->data[1] = (regval & CAN_RDLR_DATA1_MASK) >> CAN_RDLR_DATA1_SHIFT;
  frame->data[2] = (regval & CAN_RDLR_DATA2_MASK) >> CAN_RDLR_DATA2_SHIFT;
  frame->data[3] = (regval & CAN_RDLR_DATA3_MASK) >> CAN_RDLR_DATA3_SHIFT;

  regval  = stm32can_getreg(priv, STM32_CAN_RDHR_OFFSET(rxmb));
  frame->data[4] = (regval & CAN_RDHR_DATA4_MASK) >> CAN_RDHR_DATA4_SHIFT;
  frame->data[5] = (regval & CAN_RDHR_DATA5_MASK) >> CAN_RDHR_DATA5_SHIFT;
  frame->data[6] = (regval & CAN_RDHR_DATA6_MASK) >> CAN_RDHR_DATA6_SHIFT;
  frame->data[7] = (regval & CAN_RDHR_DATA7_MASK) >> CAN_RDHR_DATA7_SHIFT;

  /* Release the FIFO */

release:
  regval  = stm32can_getreg(priv, STM32_CAN_RFR_OFFSET(rxmb));
  regval |= CAN_RFR_RFOM;
  stm32can_putreg(priv, STM32_CAN_RFR_OFFSET(rxmb), regval);

  return pkt;
}

/****************************************************************************
 * Name: stm32can_receive
 *
 * Description:
 *   NuttX callback: return one received frame or error frame.  When both
 *   RX FIFOs are empty, enable the RX interrupts again and return NULL.
 *
 * Input Parameters:
 *   dev - Reference to the lower half driver state structure
 *
 * Returned Value:
 *   The received packet, or NULL if there is none.
 *
 ****************************************************************************/

static netpkt_t *stm32can_receive(struct netdev_lowerhalf_s *dev)
{
  struct stm32_can_s *priv = (struct stm32_can_s *)dev;
  netpkt_t           *pkt;
  irqstate_t          flags;
  int                 rxmb;

  flags = enter_critical_section();

#ifdef CONFIG_NET_CAN_ERRORS
  if (priv->errpending)
    {
      priv->errpending = false;
      pkt = stm32can_errframe(priv);
      stm32can_errint(priv, true);
      if (pkt != NULL)
        {
          leave_critical_section(flags);
          return pkt;
        }
    }
#endif

  for (rxmb = 0; rxmb < 2; rxmb++)
    {
      while ((stm32can_getreg(priv, STM32_CAN_RFR_OFFSET(rxmb)) &
              CAN_RFR_FMP_MASK) != 0)
        {
          pkt = stm32can_rxframe(priv, rxmb);
          if (pkt != NULL)
            {
              leave_critical_section(flags);
              return pkt;
            }
        }
    }

  /* Both FIFOs are empty */

  stm32can_rx0int(priv, true);
  stm32can_rx1int(priv, true);
  leave_critical_section(flags);
  return NULL;
}

/****************************************************************************
 * Name: stm32can_rxinterrupt
 *
 * Description:
 *   CAN RX FIFO common interrupt handler
 *
 ****************************************************************************/

static int stm32can_rxinterrupt(struct stm32_can_s *priv, int rxmb)
{
  /* receive() enables the interrupt again when the FIFOs are empty */

  if (rxmb == 0)
    {
      stm32can_rx0int(priv, false);
    }
  else
    {
      stm32can_rx1int(priv, false);
    }

  netdev_lower_rxready(&priv->dev);
  return OK;
}

/****************************************************************************
 * Name: stm32can_rx0interrupt
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

static int stm32can_rx0interrupt(int irq, void *context, void *arg)
{
  struct stm32_can_s *priv = (struct stm32_can_s *)arg;

  return stm32can_rxinterrupt(priv, 0);
}

/****************************************************************************
 * Name: stm32can_rx1interrupt
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

static int stm32can_rx1interrupt(int irq, void *context, void *arg)
{
  struct stm32_can_s *priv = (struct stm32_can_s *)arg;

  return stm32can_rxinterrupt(priv, 1);
}

/****************************************************************************
 * Name: stm32can_txinterrupt
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

static int stm32can_txinterrupt(int irq, void *context, void *arg)
{
  struct stm32_can_s *priv = (struct stm32_can_s *)arg;

  if (stm32can_reclaim(priv) > 0)
    {
      netdev_lower_txdone(&priv->dev);
    }

  return OK;
}

#ifdef CONFIG_NET_CAN_ERRORS

/****************************************************************************
 * Name: stm32can_errframe
 *
 * Description:
 *   Build an error frame from the CAN error status.  Called from receive().
 *
 * Input Parameters:
 *   priv - reference to the driver state structure
 *
 * Returned Value:
 *   The error frame packet, or NULL if there is no error or no free packet.
 *
 ****************************************************************************/

static netpkt_t *stm32can_errframe(struct stm32_can_s *priv)
{
  struct can_frame *frame;
  netpkt_t         *pkt;
  uint32_t          regval  = 0;
  uint16_t          errbits = 0;
  uint8_t           data[CAN_ERR_DLC];

  /* Check Error Interrupt flag */

  regval = stm32can_getreg(priv, STM32_CAN_MSR_OFFSET);
  if (regval & CAN_MSR_ERRI)
    {
      /* Encode error bits */

      errbits = 0;
      memset(data, 0, sizeof(data));

      /* Get Error statur register */

      regval = stm32can_getreg(priv, STM32_CAN_ESR_OFFSET);

      if (regval & CAN_ESR_EWGF)
        {
          /* Error warning flag */

          data[1] |= (CAN_ERR_CRTL_RX_WARNING | CAN_ERR_CRTL_TX_WARNING);
          errbits |= CAN_ERR_CRTL;
        }

      if (regval & CAN_ESR_EPVF)
        {
          /* Error passive flag */

          data[1] |= (CAN_ERR_CRTL_RX_PASSIVE | CAN_ERR_CRTL_TX_PASSIVE);
          errbits |= CAN_ERR_CRTL;
        }

      if (regval & CAN_ESR_BOFF)
        {
          /* Bus-off flag */

          errbits |= CAN_ERR_BUSOFF;
        }

      /* Last error code */

      if (regval & CAN_ESR_LEC_MASK)
        {
          if ((regval & CAN_ESR_LEC_MASK) == CAN_ESR_STUFFERROR)
            {
              /* Stuff Error */

              errbits |= CAN_ERR_PROT;
              data[2] |= CAN_ERR_PROT_STUFF;
            }
          else if ((regval & CAN_ESR_LEC_MASK) == CAN_ESR_FORMERROR)
            {
              /* Format Error */

              errbits |= CAN_ERR_PROT;
              data[2] |= CAN_ERR_PROT_FORM;
            }
          else if ((regval & CAN_ESR_LEC_MASK) == CAN_ESR_ACKERROR)
            {
              /* Acknowledge Error */

              errbits |= CAN_ERR_ACK;
            }
          else if ((regval & CAN_ESR_LEC_MASK) == CAN_ESR_BRECERROR)
            {
              /* Bit recessive Error */

              errbits |= CAN_ERR_PROT;
              data[2] |= CAN_ERR_PROT_BIT1;
            }
          else if ((regval & CAN_ESR_LEC_MASK) == CAN_ESR_BDOMERROR)
            {
              /* Bit dominant Error */

              errbits |= CAN_ERR_PROT;
              data[2] |= CAN_ERR_PROT_BIT0;
            }
          else if ((regval & CAN_ESR_LEC_MASK) == CAN_ESR_CRCERRPR)
            {
              /* Receive CRC Error */

              errbits |= CAN_ERR_PROT;
              data[3] |= CAN_ERR_PROT_LOC_CRC_SEQ;
            }
        }

      /* Get transmit status register */

      regval = stm32can_getreg(priv, STM32_CAN_TSR_OFFSET);

      if (regval & CAN_TSR_ALST0 || regval & CAN_TSR_ALST1 ||
          regval & CAN_TSR_ALST2)
        {
          /* Lost arbitration Error */

          errbits |= CAN_ERR_LOSTARB;
        }

      /* Clear ERRI flag */

      stm32can_putreg(priv, STM32_CAN_MSR_OFFSET, CAN_MSR_ERRI);
    }

  if (errbits == 0)
    {
      return NULL;
    }

  ninfo("errbits = %08" PRIx16 "\n", errbits);

  pkt = netpkt_alloc(&priv->dev, NETPKT_RX);
  if (pkt == NULL)
    {
      return NULL;
    }

  netpkt_setdatalen(&priv->dev, pkt, sizeof(struct can_frame));
  frame = (struct can_frame *)netpkt_getdata(&priv->dev, pkt);
  memset(frame, 0, sizeof(*frame));

  frame->can_id  = CAN_ERR_FLAG | errbits;
  frame->can_dlc = CAN_ERR_DLC;
  memcpy(frame->data, data, CAN_ERR_DLC);

  NETDEV_ERRORS(&priv->dev.netdev);
  return pkt;
}

/****************************************************************************
 * Name: stm32can_sceinterrupt
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

static int stm32can_sceinterrupt(int irq, void *context, void *arg)
{
  struct stm32_can_s *priv = (struct stm32_can_s *)arg;

  /* receive() reports the error and enables the interrupts again */

  stm32can_errint(priv, false);
  priv->errpending = true;
  netdev_lower_rxready(&priv->dev);

  return OK;
}
#endif

/****************************************************************************
 * Name: stm32can_bittiming
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

static int stm32can_bittiming(struct stm32_can_s *priv)
{
  uint32_t tmp;
  uint32_t brp;
  uint32_t ts1;
  uint32_t ts2;

  ninfo("CAN%" PRIu8 " PCLK1: %lu baud: %" PRIu32 "\n",
          priv->port, (unsigned long) STM32_PCLK1_FREQUENCY, priv->baud);

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
   *   PCLK1 = 42,000,000 baud = 1,000,000 nquanta = 14 : brp = 3
   *   PCLK1 = 42,000,000 baud =   700,000 nquanta = 14 : brp = 4
   */

  tmp = STM32_PCLK1_FREQUENCY / priv->baud;
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

  /* Otherwise, nquanta is CAN_BIT_QUANTA, ts1 is CONFIG_STM32_CAN_TSEG1,
   * ts2 is CONFIG_STM32_CAN_TSEG2 and we calculate brp to achieve
   * CAN_BIT_QUANTA quanta in the bit time
   */

  else
    {
      ts1 = CONFIG_STM32_CAN_TSEG1;
      ts2 = CONFIG_STM32_CAN_TSEG2;
      brp = (tmp + (CAN_BIT_QUANTA / 2)) / CAN_BIT_QUANTA;
    }

  if (brp > CAN_BTR_BRP_MAX || ts1 < 1 || ts1 > CAN_BTR_TSEG1_MAX ||
      ts2 < 1 || ts2 > CAN_BTR_TSEG2_MAX)
    {
      canerr("ERROR: Bit rate %" PRIu32 " not supported\n", priv->baud);
      return -EINVAL;
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

  tmp = ((brp - 1) << CAN_BTR_BRP_SHIFT) | ((ts1 - 1) << CAN_BTR_TS1_SHIFT) |
        ((ts2 - 1) << CAN_BTR_TS2_SHIFT) | ((1 - 1) << CAN_BTR_SJW_SHIFT);
#ifdef CONFIG_CAN_LOOPBACK
  /* tmp |= (CAN_BTR_LBKM | CAN_BTR_SILM); */

  tmp |= CAN_BTR_LBKM;
#endif

  stm32can_putreg(priv, STM32_CAN_BTR_OFFSET, tmp);
  return OK;
}

/****************************************************************************
 * Name: stm32can_setup
 ****************************************************************************/

static int  stm32can_setup(struct stm32_can_s *priv)
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

  ret = stm32can_cellinit(priv);
  if (ret < 0)
    {
      nerr("ERROR: CAN%" PRId8 " cell initialization failed: %d\n",
             priv->port, ret);
      return ret;
    }

  stm32can_dumpctrlregs(priv, "After cell initialization");
  stm32can_dumpmbregs(priv, NULL);

  /* CAN filter initialization */

  ret = stm32can_filterinit(priv);
  if (ret < 0)
    {
      nerr("ERROR: CAN%" PRIu8 " filter initialization failed: %d\n",
             priv->port, ret);
      return ret;
    }

  stm32can_dumpfiltregs(priv, "After filter initialization");

  /* Attach the CAN RX FIFO 0/1 interrupts and TX interrupts.
   * The others are not used.
   */

  ret = irq_attach(priv->canrx[0], stm32can_rx0interrupt, priv);
  if (ret < 0)
    {
      nerr("ERROR: Failed to attach CAN%" PRIu8 " RX0 IRQ (%" PRIu8 ")",
             priv->port, priv->canrx[0]);
      return ret;
    }

  ret = irq_attach(priv->canrx[1], stm32can_rx1interrupt, priv);
  if (ret < 0)
    {
      nerr("ERROR: Failed to attach CAN%" PRIu8 " RX1 IRQ (%" PRIu8 ")",
             priv->port, priv->canrx[1]);
      return ret;
    }

  ret = irq_attach(priv->cantx, stm32can_txinterrupt, priv);
  if (ret < 0)
    {
      nerr("ERROR: Failed to attach CAN%" PRIu8 " TX IRQ (%" PRIu8 ")",
             priv->port, priv->cantx);
      return ret;
    }

#ifdef CONFIG_NET_CAN_ERRORS
  ret = irq_attach(priv->cansce, stm32can_sceinterrupt, priv);
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
 * Name: stm32can_shutdown
 ****************************************************************************/

static void stm32can_shutdown(struct stm32_can_s *priv)
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
 * Name: stm32can_reset
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

static void stm32can_reset(struct stm32_can_s *priv)
{
  uint32_t regval;
  uint32_t regbit = 0;
  irqstate_t flags;

  ninfo("CAN%" PRIu8 "\n", priv->port);

  /* Get the bits in the AHB1RSTR register needed to reset this CAN device */

#ifdef CONFIG_STM32_CAN1
  if (priv->port == 1)
    {
#ifdef CONFIG_STM32_CAN2
      /* The CAN2 filter banks are in CAN1: only stop CAN1 */

      if (g_can2priv.active)
        {
          stm32can_putreg(priv, STM32_CAN_IER_OFFSET, 0);
          regval  = stm32can_getreg(priv, STM32_CAN_MCR_OFFSET);
          regval |= CAN_MCR_INRQ;
          stm32can_putreg(priv, STM32_CAN_MCR_OFFSET, regval);
          return;
        }
#endif

      regbit = CAN_RCC_CAN1RST;
    }
  else
#endif
#ifdef CONFIG_STM32_CAN2
  if (priv->port == 2)
    {
      regbit = CAN_RCC_CAN2RST;
    }
  else
#endif
#ifdef CONFIG_STM32_CAN3
  if (priv->port == 3)
    {
      regbit = CAN_RCC_CAN3RST;
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

  regval  = getreg32(CAN_RCC_RSTR);
  regval |= regbit;
  putreg32(regval, CAN_RCC_RSTR);

  regval &= ~regbit;
  putreg32(regval, CAN_RCC_RSTR);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: stm32can_enterinitmode
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

static int stm32can_enterinitmode(struct stm32_can_s *priv)
{
  uint32_t regval;
  volatile uint32_t timeout;

  ninfo("CAN%" PRIu8 "\n", priv->port);

  /* Enter initialization mode */

  regval  = stm32can_getreg(priv, STM32_CAN_MCR_OFFSET);
  regval |= CAN_MCR_INRQ;
  stm32can_putreg(priv, STM32_CAN_MCR_OFFSET, regval);

  /* Wait until initialization mode is acknowledged */

  for (timeout = INAK_TIMEOUT; timeout > 0; timeout--)
    {
      regval = stm32can_getreg(priv, STM32_CAN_MSR_OFFSET);
      if ((regval & CAN_MSR_INAK) != 0)
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
 * Name: stm32can_exitinitmode
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

static int stm32can_exitinitmode(struct stm32_can_s *priv)
{
  uint32_t regval;
  volatile uint32_t timeout;

  /* Exit Initialization mode, enter Normal mode */

  regval  = stm32can_getreg(priv, STM32_CAN_MCR_OFFSET);
  regval &= ~CAN_MCR_INRQ;
  stm32can_putreg(priv, STM32_CAN_MCR_OFFSET, regval);

  /* Wait until the initialization mode exit is acknowledged */

  for (timeout = INAK_TIMEOUT; timeout > 0; timeout--)
    {
      regval = stm32can_getreg(priv, STM32_CAN_MSR_OFFSET);
      if ((regval & CAN_MSR_INAK) == 0)
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
 * Name: stm32can_cellinit
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

static int stm32can_cellinit(struct stm32_can_s *priv)
{
  uint32_t regval;
  int ret;

  ninfo("CAN%" PRIu8 "\n", priv->port);

  /* Exit from sleep mode */

  regval  = stm32can_getreg(priv, STM32_CAN_MCR_OFFSET);
  regval &= ~CAN_MCR_SLEEP;
  stm32can_putreg(priv, STM32_CAN_MCR_OFFSET, regval);

  ret = stm32can_enterinitmode(priv);
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

  regval  = stm32can_getreg(priv, STM32_CAN_MCR_OFFSET);
  regval &= ~(CAN_MCR_RFLM | CAN_MCR_NART | CAN_MCR_AWUM |
              CAN_MCR_ABOM | CAN_MCR_TTCM);
  regval |=  CAN_MCR_TXFP;
  stm32can_putreg(priv, STM32_CAN_MCR_OFFSET, regval);

  /* Configure bit timing. */

  ret = stm32can_bittiming(priv);
  if (ret < 0)
    {
      nerr("ERROR: Failed to set bit timing: %d\n", ret);
      return ret;
    }

  return stm32can_exitinitmode(priv);
}

/****************************************************************************
 * Name: stm32can_filterinit
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
 *   (but reserves CAN_NFILTERS/2 through CAN_NFILTERS-1).  CAN3 has its
 *   own filter banks and uses only filter 0.
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

static int stm32can_filterinit(struct stm32_can_s *priv)
{
  uint32_t regval;
  uint32_t bitmask;

  ninfo("CAN%" PRIu8 " filter: %" PRIu8 "\n", priv->port, priv->filter);

  /* Get the bitmask associated with the filter used by this CAN block */

  bitmask = (uint32_t)1 << priv->filter;

  /* Enter filter initialization mode */

  regval  = stm32can_getfreg(priv, STM32_CAN_FMR_OFFSET);
  regval |= CAN_FMR_FINIT;
  stm32can_putfreg(priv, STM32_CAN_FMR_OFFSET, regval);

  /* Assign half the filters to CAN1, half to CAN2 */

#if defined(CONFIG_STM32_CONNECTIVITYLINE) || \
    defined(CONFIG_STM32_STM32F20XX) || \
    defined(CONFIG_STM32_STM32F4XXX)
  regval  = stm32can_getfreg(priv, STM32_CAN_FMR_OFFSET);
  regval &= ~CAN_FMR_CAN2SB_MASK;
  regval |= (CAN_NFILTERS / 2) << CAN_FMR_CAN2SB_SHIFT;
  stm32can_putfreg(priv, STM32_CAN_FMR_OFFSET, regval);
#endif

  /* Disable the filter */

  regval  = stm32can_getfreg(priv, STM32_CAN_FA1R_OFFSET);
  regval &= ~bitmask;
  stm32can_putfreg(priv, STM32_CAN_FA1R_OFFSET, regval);

  /* Select the 32-bit scale for the filter */

  regval  = stm32can_getfreg(priv, STM32_CAN_FS1R_OFFSET);
  regval |= bitmask;
  stm32can_putfreg(priv, STM32_CAN_FS1R_OFFSET, regval);

  /* There are 14 or 28 filter banks (depending) on the device.
   * Each filter bank is composed of two 32-bit registers, CAN_FiR:
   */

  stm32can_putfreg(priv, STM32_CAN_FIR_OFFSET(priv->filter, 1), 0);
  stm32can_putfreg(priv, STM32_CAN_FIR_OFFSET(priv->filter, 2), 0);

  /* Set Id/Mask mode for the filter */

  regval  = stm32can_getfreg(priv, STM32_CAN_FM1R_OFFSET);
  regval &= ~bitmask;
  stm32can_putfreg(priv, STM32_CAN_FM1R_OFFSET, regval);

  /* Assign FIFO 0 for the filter */

  regval  = stm32can_getfreg(priv, STM32_CAN_FFA1R_OFFSET);
  regval &= ~bitmask;
  stm32can_putfreg(priv, STM32_CAN_FFA1R_OFFSET, regval);

  /* Enable the filter */

  regval  = stm32can_getfreg(priv, STM32_CAN_FA1R_OFFSET);
  regval |= bitmask;
  stm32can_putfreg(priv, STM32_CAN_FA1R_OFFSET, regval);

  /* Exit filter initialization mode */

  regval  = stm32can_getfreg(priv, STM32_CAN_FMR_OFFSET);
  regval &= ~CAN_FMR_FINIT;
  stm32can_putfreg(priv, STM32_CAN_FMR_OFFSET, regval);
  return OK;
}

/****************************************************************************
 * Name: stm32can_txmb0empty
 *
 * Input Parameters:
 *   tsr_regval - value of CAN transmit status register
 *
 * Returned Value:
 *   Returns true if mailbox 0 is empty and can be used for sending.
 *
 ****************************************************************************/

static bool stm32can_txmb0empty(uint32_t tsr_regval)
{
  return (tsr_regval & CAN_TSR_TME0) != 0 &&
         (tsr_regval & CAN_TSR_RQCP0) == 0;
}

/****************************************************************************
 * Name: stm32can_txmb1empty
 *
 * Input Parameters:
 *   tsr_regval - value of CAN transmit status register
 *
 * Returned Value:
 *   Returns true if mailbox 1 is empty and can be used for sending.
 *
 ****************************************************************************/

static bool stm32can_txmb1empty(uint32_t tsr_regval)
{
  return (tsr_regval & CAN_TSR_TME1) != 0 &&
         (tsr_regval & CAN_TSR_RQCP1) == 0;
}

/****************************************************************************
 * Name: stm32can_txmb2empty
 *
 * Input Parameters:
 *   tsr_regval - value of CAN transmit status register
 *
 * Returned Value:
 *   Returns true if mailbox 2 is empty and can be used for sending.
 *
 ****************************************************************************/

static bool stm32can_txmb2empty(uint32_t tsr_regval)
{
  return (tsr_regval & CAN_TSR_TME2) != 0 &&
         (tsr_regval & CAN_TSR_RQCP2) == 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_cansockinitialize
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

int stm32_cansockinitialize(int port)
{
  struct stm32_can_s *priv = NULL;
  int                 ret  = OK;

  ninfo("CAN%" PRIu8 "\n", port);

  /* NOTE:  Peripherical clocking for CAN1 and/or CAN2 was already provided
   * by stm32_clockconfig() early in the reset sequence.
   */

#ifdef CONFIG_STM32_CAN1
  if (port == 1)
    {
      /* Select the CAN1 device structure */

      priv = &g_can1priv;

      /* Configure CAN1 pins.  The ambiguous settings in the stm32*_pinmap.h
       * file must have been disambiguated in the board.h file.
       */

      stm32_configgpio(GPIO_CAN1_RX);
      stm32_configgpio(GPIO_CAN1_TX);
    }
  else
#endif
#ifdef CONFIG_STM32_CAN2
  if (port == 2)
    {
      /* Select the CAN2 device structure */

      priv = &g_can2priv;

      /* Configure CAN2 pins.  The ambiguous settings in the stm32*_pinmap.h
       * file must have been disambiguated in the board.h file.
       */

      stm32_configgpio(GPIO_CAN2_RX);
      stm32_configgpio(GPIO_CAN2_TX);
    }
  else
#endif
#ifdef CONFIG_STM32_CAN3
  if (port == 3)
    {
      /* Select the CAN3 device structure */

      priv = &g_can3priv;

      /* Configure CAN3 pins.  The ambiguous settings in the stm32*_pinmap.h
       * file must have been disambiguated in the board.h file.
       */

      stm32_configgpio(GPIO_CAN3_RX);
      stm32_configgpio(GPIO_CAN3_TX);
    }
  else
#endif
    {
      nerr("ERROR: Unsupported port %d\n", port);
      ret = -EINVAL;
      goto errout;
    }

  /* Initialize the driver structure */

  priv->dev.ops = &g_can_ops;

  /* Put the controller in the reset state */

  stm32can_reset(priv);

  /* Register the device with the OS so that socket IOCTLs can be performed */

  ret = netdev_lower_register(&priv->dev, NET_LL_CAN);

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

#if !defined(CONFIG_NETDEV_LATEINIT)
void arm_netinitialize(void)
{
#ifdef CONFIG_STM32_CAN1
  stm32_cansockinitialize(1);
#endif

#ifdef CONFIG_STM32_CAN2
  stm32_cansockinitialize(2);
#endif

#ifdef CONFIG_STM32_CAN3
  stm32_cansockinitialize(3);
#endif
}
#endif
