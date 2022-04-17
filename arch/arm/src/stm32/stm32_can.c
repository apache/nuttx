/****************************************************************************
 * arch/arm/src/stm32/stm32_can.c
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
#include <nuttx/can/can.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32.h"
#include "stm32_rcc.h"
#include "stm32_can.h"

#if defined(CONFIG_CAN) && \
    (defined(CONFIG_STM32_CAN1) || defined(CONFIG_STM32_CAN2))

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

/* CAN error interrupts */

#ifdef CONFIG_CAN_ERRORS
#  define STM32_CAN_ERRINT (CAN_IER_LECIE | CAN_IER_ERRIE |             \
                            CAN_IER_BOFIE | CAN_IER_EPVIE |             \
                            CAN_IER_EWGIE)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_can_s
{
  uint8_t  port;     /* CAN port number (1 or 2) */
  uint8_t  canrx[2]; /* CAN RX FIFO 0/1 IRQ number */
  uint8_t  cantx;    /* CAN TX IRQ number */
#ifdef CONFIG_CAN_ERRORS
  uint8_t  cansce;   /* CAN SCE IRQ number */
#endif
  uint8_t  filter;   /* Filter number */
  uint32_t base;     /* Base address of the CAN control registers */
  uint32_t fbase;    /* Base address of the CAN filter registers */
  uint32_t baud;     /* Configured baud */
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

/* Filtering (todo) */

#ifdef CONFIG_CAN_EXTID
static int  stm32can_addextfilter(struct stm32_can_s *priv,
                                  struct canioc_extfilter_s *arg);
static int  stm32can_delextfilter(struct stm32_can_s *priv,
                                  int arg);
#endif
static int  stm32can_addstdfilter(struct stm32_can_s *priv,
                                  struct canioc_stdfilter_s *arg);
static int  stm32can_delstdfilter(struct stm32_can_s *priv,
                                  int arg);

/* CAN driver methods */

static void stm32can_reset(struct can_dev_s *dev);
static int  stm32can_setup(struct can_dev_s *dev);
static void stm32can_shutdown(struct can_dev_s *dev);
static void stm32can_rxint(struct can_dev_s *dev, bool enable);
static void stm32can_txint(struct can_dev_s *dev, bool enable);
static int  stm32can_ioctl(struct can_dev_s *dev, int cmd,
                           unsigned long arg);
static int  stm32can_remoterequest(struct can_dev_s *dev,
                                   uint16_t id);
static int  stm32can_send(struct can_dev_s *dev,
                          struct can_msg_s *msg);
static bool stm32can_txready(struct can_dev_s *dev);
static bool stm32can_txempty(struct can_dev_s *dev);

#ifdef CONFIG_CAN_ERRORS
static void stm32can_errint(struct can_dev_s *dev, bool enable);
#endif

/* CAN interrupt handling */

static int  stm32can_rxinterrupt(struct can_dev_s *dev, int rxmb);
static int  stm32can_rx0interrupt(int irq, void *context, void *arg);
static int  stm32can_rx1interrupt(int irq, void *context, void *arg);
static int  stm32can_txinterrupt(int irq, void *context, void *arg);
#ifdef CONFIG_CAN_ERRORS
static int  stm32can_sceinterrupt(int irq, void *context, void *arg);
#endif

/* Initialization */

static int  stm32can_enterinitmode(struct stm32_can_s *priv);
static int  stm32can_exitinitmode(struct stm32_can_s *priv);
static int  stm32can_bittiming(struct stm32_can_s *priv);
static int  stm32can_cellinit(struct stm32_can_s *priv);
static int  stm32can_filterinit(struct stm32_can_s *priv);

/* TX mailbox status */

static bool stm32can_txmb0empty(uint32_t tsr_regval);
static bool stm32can_txmb1empty(uint32_t tsr_regval);
static bool stm32can_txmb2empty(uint32_t tsr_regval);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct can_ops_s g_canops =
{
  .co_reset         = stm32can_reset,
  .co_setup         = stm32can_setup,
  .co_shutdown      = stm32can_shutdown,
  .co_rxint         = stm32can_rxint,
  .co_txint         = stm32can_txint,
  .co_ioctl         = stm32can_ioctl,
  .co_remoterequest = stm32can_remoterequest,
  .co_send          = stm32can_send,
  .co_txready       = stm32can_txready,
  .co_txempty       = stm32can_txempty,
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
#ifdef CONFIG_CAN_ERRORS
  .cansce           = STM32_IRQ_CAN1SCE,
#endif
  .filter           = 0,
  .base             = STM32_CAN1_BASE,
  .fbase            = STM32_CAN1_BASE,
  .baud             = CONFIG_STM32_CAN1_BAUD,
};

static struct can_dev_s g_can1dev =
{
  .cd_ops           = &g_canops,
  .cd_priv          = &g_can1priv,
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
#ifdef CONFIG_CAN_ERRORS
  .cansce           = STM32_IRQ_CAN2SCE,
#endif
  .filter           = CAN_NFILTERS / 2,
  .base             = STM32_CAN2_BASE,
  .fbase            = STM32_CAN1_BASE,
  .baud             = CONFIG_STM32_CAN2_BAUD,
};

static struct can_dev_s g_can2dev =
{
  .cd_ops           = &g_canops,
  .cd_priv          = &g_can2priv,
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
 * Input Parameters:
 *   priv - A reference to the CAN block status
 *   offset - The offset to the register to read
 *
 * Returned Value:
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
              caninfo("...\n");
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

          caninfo("[repeats %" PRIu32 " more times]\n", count - 3);
        }

      /* Save the new address, value, and count */

      prevaddr = addr;
      preval   = val;
      count    = 1;
    }

  /* Show the register value read */

  caninfo("%08" PRIx32 "->%08" PRIx32 "\n", addr, val);
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
 * Input Parameters:
 *   priv - A reference to the CAN block status
 *   offset - The offset to the register to write
 *   value - The value to write to the register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_CAN_REGDEBUG
static void stm32can_vputreg(uint32_t addr, uint32_t value)
{
  /* Show the register value being written */

  caninfo("%08" PRIx32 "->%08" PRIx32 "\n", addr, val);

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
 *   priv - A reference to the CAN block status
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
      caninfo("Control Registers: %s\n", msg);
    }
  else
    {
      caninfo("Control Registers:\n");
    }

  /* CAN control and status registers */

  caninfo("  MCR: %08" PRIx32 "   MSR: %08" PRIx32 "   TSR: %08" PRIx32 "\n",
          getreg32(priv->base + STM32_CAN_MCR_OFFSET),
          getreg32(priv->base + STM32_CAN_MSR_OFFSET),
          getreg32(priv->base + STM32_CAN_TSR_OFFSET));

  caninfo(" RF0R: %08" PRIx32 "  RF1R: %08" PRIx32 "\n",
          getreg32(priv->base + STM32_CAN_RF0R_OFFSET),
          getreg32(priv->base + STM32_CAN_RF1R_OFFSET));

  caninfo("  IER: %08" PRIx32 "   ESR: %08" PRIx32 "   BTR: %08" PRIx32 "\n",
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
 *   priv - A reference to the CAN block status
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
      caninfo("Mailbox Registers: %s\n", msg);
    }
  else
    {
      caninfo("Mailbox Registers:\n");
    }

  /* CAN mailbox registers (3 TX and 2 RX) */

  caninfo(" TI0R: %08" PRIx32 " TDT0R: %08" PRIx32 " TDL0R: %08"
          PRIx32 " TDH0R: %08" PRIx32 "\n",
          getreg32(priv->base + STM32_CAN_TI0R_OFFSET),
          getreg32(priv->base + STM32_CAN_TDT0R_OFFSET),
          getreg32(priv->base + STM32_CAN_TDL0R_OFFSET),
          getreg32(priv->base + STM32_CAN_TDH0R_OFFSET));

  caninfo(" TI1R: %08" PRIx32 " TDT1R: %08" PRIx32 " TDL1R: %08"
          PRIx32 " TDH1R: %08" PRIx32 "\n",
          getreg32(priv->base + STM32_CAN_TI1R_OFFSET),
          getreg32(priv->base + STM32_CAN_TDT1R_OFFSET),
          getreg32(priv->base + STM32_CAN_TDL1R_OFFSET),
          getreg32(priv->base + STM32_CAN_TDH1R_OFFSET));

  caninfo(" TI2R: %08" PRIx32 " TDT2R: %08" PRIx32 " TDL2R: %08"
          PRIx32 " TDH2R: %08" PRIx32 "\n",
          getreg32(priv->base + STM32_CAN_TI2R_OFFSET),
          getreg32(priv->base + STM32_CAN_TDT2R_OFFSET),
          getreg32(priv->base + STM32_CAN_TDL2R_OFFSET),
          getreg32(priv->base + STM32_CAN_TDH2R_OFFSET));

  caninfo(" RI0R: %08" PRIx32 " RDT0R: %08" PRIx32 " RDL0R: %08"
          PRIx32 " RDH0R: %08" PRIx32 "\n",
          getreg32(priv->base + STM32_CAN_RI0R_OFFSET),
          getreg32(priv->base + STM32_CAN_RDT0R_OFFSET),
          getreg32(priv->base + STM32_CAN_RDL0R_OFFSET),
          getreg32(priv->base + STM32_CAN_RDH0R_OFFSET));

  caninfo(" RI1R: %08" PRIx32 " RDT1R: %08" PRIx32 " RDL1R: %08"
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
 *   priv - A reference to the CAN block status
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
      caninfo("Filter Registers: %s\n", msg);
    }
  else
    {
      caninfo("Filter Registers:\n");
    }

  caninfo(" FMR: %08" PRIx32 "   FM1R: %08" PRIx32 "  FS1R: %08"
          PRIx32 " FFA1R: %08" PRIx32 "  FA1R: %08" PRIx32 "\n",
          getreg32(priv->base + STM32_CAN_FMR_OFFSET),
          getreg32(priv->base + STM32_CAN_FM1R_OFFSET),
          getreg32(priv->base + STM32_CAN_FS1R_OFFSET),
          getreg32(priv->base + STM32_CAN_FFA1R_OFFSET),
          getreg32(priv->base + STM32_CAN_FA1R_OFFSET));

  for (i = 0; i < CAN_NFILTERS; i++)
    {
      caninfo(" F%dR1: %08" PRIx32 " F%dR2: %08" PRIx32 "\n",
              i, getreg32(priv->base + STM32_CAN_FIR_OFFSET(i, 1)),
              i, getreg32(priv->base + STM32_CAN_FIR_OFFSET(i, 2)));
    }
}
#endif

/****************************************************************************
 * Name: stm32can_reset
 *
 * Description:
 *   Reset the CAN device.  Called early to initialize the hardware. This
 *   function is called, before stm32can_setup() and on error conditions.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

static void stm32can_reset(struct can_dev_s *dev)
{
  struct stm32_can_s *priv = dev->cd_priv;
  uint32_t regval;
  uint32_t regbit = 0;
  irqstate_t flags;

  caninfo("CAN%" PRIu8 "\n", priv->port);

  /* Get the bits in the AHB1RSTR register needed to reset this CAN device */

#ifdef CONFIG_STM32_CAN1
  if (priv->port == 1)
    {
      regbit = RCC_APB1RSTR_CAN1RST;
    }
  else
#endif
#ifdef CONFIG_STM32_CAN2
  if (priv->port == 2)
    {
      regbit = RCC_APB1RSTR_CAN2RST;
    }
  else
#endif
    {
      canerr("ERROR: Unsupported port %d\n", priv->port);
      return;
    }

  /* Disable interrupts momentarily to stop any ongoing CAN event processing
   * and to prevent any concurrent access to the AHB1RSTR register.
   */

  flags = enter_critical_section();

  /* Reset the CAN */

  regval  = getreg32(STM32_RCC_APB1RSTR);
  regval |= regbit;
  putreg32(regval, STM32_RCC_APB1RSTR);

  regval &= ~regbit;
  putreg32(regval, STM32_RCC_APB1RSTR);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: stm32can_setup
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

static int stm32can_setup(struct can_dev_s *dev)
{
  struct stm32_can_s *priv = dev->cd_priv;
  int ret;

#ifdef CONFIG_CAN_ERRORS
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
      canerr("ERROR: CAN%" PRId8 " cell initialization failed: %d\n",
             priv->port, ret);
      return ret;
    }

  stm32can_dumpctrlregs(priv, "After cell initialization");
  stm32can_dumpmbregs(priv, NULL);

  /* CAN filter initialization */

  ret = stm32can_filterinit(priv);
  if (ret < 0)
    {
      canerr("ERROR: CAN%" PRIu8 " filter initialization failed: %d\n",
             priv->port, ret);
      return ret;
    }

  stm32can_dumpfiltregs(priv, "After filter initialization");

  /* Attach the CAN RX FIFO 0/1 interrupts and TX interrupts.
   * The others are not used.
   */

  ret = irq_attach(priv->canrx[0], stm32can_rx0interrupt, dev);
  if (ret < 0)
    {
      canerr("ERROR: Failed to attach CAN%" PRIu8 " RX0 IRQ (%" PRIu8 ")",
             priv->port, priv->canrx[0]);
      return ret;
    }

  ret = irq_attach(priv->canrx[1], stm32can_rx1interrupt, dev);
  if (ret < 0)
    {
      canerr("ERROR: Failed to attach CAN%" PRIu8 " RX1 IRQ (%" PRIu8 ")",
             priv->port, priv->canrx[1]);
      return ret;
    }

  ret = irq_attach(priv->cantx, stm32can_txinterrupt, dev);
  if (ret < 0)
    {
      canerr("ERROR: Failed to attach CAN%" PRIu8 " TX IRQ (%" PRIu8 ")",
             priv->port, priv->cantx);
      return ret;
    }

#ifdef CONFIG_CAN_ERRORS
  ret = irq_attach(priv->cansce, stm32can_sceinterrupt, dev);
  if (ret < 0)
    {
      nerr("ERROR: Failed to attach CAN%" PRIu8 " SCE IRQ (%" PRIu8 ")",
           priv->port, priv->cansce);
      return ret;
    }

  /* Enable CAN error interrupts */

  stm32can_errint(dev, true);
#endif

  /* Enable the interrupts at the NVIC.  Interrupts are still disabled in
   * the CAN module.  Since we coming out of reset here, there should be
   * no pending interrupts.
   */

  up_enable_irq(priv->canrx[0]);
  up_enable_irq(priv->canrx[1]);
  up_enable_irq(priv->cantx);
#ifdef CONFIG_CAN_ERRORS
  up_enable_irq(priv->cansce);
#endif
  return OK;
}

/****************************************************************************
 * Name: stm32can_shutdown
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

static void stm32can_shutdown(struct can_dev_s *dev)
{
  struct stm32_can_s *priv = dev->cd_priv;

  caninfo("CAN%" PRIu8 "\n", priv->port);

  /* Disable the RX FIFO 0/1, TX and SCE interrupts */

  up_disable_irq(priv->canrx[0]);
  up_disable_irq(priv->canrx[1]);
  up_disable_irq(priv->cantx);
#ifdef CONFIG_CAN_ERRORS
  up_disable_irq(priv->cansce);
#endif

  /* Detach the RX FIFO 0/1, TX and SCE interrupts */

  irq_detach(priv->canrx[0]);
  irq_detach(priv->canrx[1]);
  irq_detach(priv->cantx);
#ifdef CONFIG_CAN_ERRORS
  irq_detach(priv->cansce);
#endif

  /* And reset the hardware */

  stm32can_reset(dev);
}

/****************************************************************************
 * Name: stm32can_rxint
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

static void stm32can_rxint(struct can_dev_s *dev, bool enable)
{
  struct stm32_can_s *priv = dev->cd_priv;
  uint32_t regval;

  caninfo("CAN%" PRIu8 " rxint enable: %d\n", priv->port, enable);

  /* Enable/disable the FIFO 0/1 message pending interrupt */

  regval = stm32can_getreg(priv, STM32_CAN_IER_OFFSET);
  if (enable)
    {
      regval |= CAN_IER_FMPIE0 | CAN_IER_FMPIE1;
    }
  else
    {
      regval &= ~(CAN_IER_FMPIE0 | CAN_IER_FMPIE1);
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
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stm32can_txint(struct can_dev_s *dev, bool enable)
{
  struct stm32_can_s *priv = dev->cd_priv;
  uint32_t regval;

  caninfo("CAN%" PRIu8 " txint enable: %d\n", priv->port, enable);

  /* Support only disabling the transmit mailbox interrupt */

  if (!enable)
    {
      regval  = stm32can_getreg(priv, STM32_CAN_IER_OFFSET);
      regval &= ~CAN_IER_TMEIE;
      stm32can_putreg(priv, STM32_CAN_IER_OFFSET, regval);
    }
}

#ifdef CONFIG_CAN_ERRORS
/****************************************************************************
 * Name: stm32can_errint
 *
 * Description:
 *   Call to enable or disable CAN error interrupts.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stm32can_errint(struct can_dev_s *dev, bool enable)
{
  struct stm32_can_s *priv = dev->cd_priv;
  uint32_t regval = 0;

  caninfo("CAN%" PRIu8 " errint enable: %d\n", priv->port, enable);

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
 * Name: stm32can_ioctl
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

static int stm32can_ioctl(struct can_dev_s *dev, int cmd,
                          unsigned long arg)
{
  struct stm32_can_s *priv;
  int ret = -ENOTTY;

  caninfo("cmd=%04x arg=%lu\n", cmd, arg);

  DEBUGASSERT(dev && dev->cd_priv);
  priv = dev->cd_priv;

  /* Handle the command */

  switch (cmd)
    {
      /* CANIOC_GET_BITTIMING:
       *   Description:    Return the current bit timing settings
       *   Argument:       A pointer to a write-able instance of struct
       *                   canioc_bittiming_s in which current bit timing
       *                   values will be returned.
       *   Returned Value: Zero (OK) is returned on success.  Otherwise -1
       *                   (ERROR) is returned with the errno variable set
       *                   to indicate the nature of the error.
       *   Dependencies:   None
       */

      case CANIOC_GET_BITTIMING:
        {
          struct canioc_bittiming_s *bt =
            (struct canioc_bittiming_s *)arg;
          uint32_t regval;
          uint32_t brp;

          DEBUGASSERT(bt != NULL);
          regval       = stm32can_getreg(priv, STM32_CAN_BTR_OFFSET);
          bt->bt_sjw   = ((regval & CAN_BTR_SJW_MASK) >>
                          CAN_BTR_SJW_SHIFT) + 1;
          bt->bt_tseg1 = ((regval & CAN_BTR_TS1_MASK) >>
                          CAN_BTR_TS1_SHIFT) + 1;
          bt->bt_tseg2 = ((regval & CAN_BTR_TS2_MASK) >>
                          CAN_BTR_TS2_SHIFT) + 1;

          brp          = ((regval & CAN_BTR_BRP_MASK) >>
                          CAN_BTR_BRP_SHIFT) + 1;
          bt->bt_baud  = STM32_PCLK1_FREQUENCY /
                         (brp * (bt->bt_tseg1 + bt->bt_tseg2 + 1));
          ret = OK;
        }
        break;

      /* CANIOC_SET_BITTIMING:
       *   Description:    Set new current bit timing values
       *   Argument:       A pointer to a read-able instance of struct
       *                   canioc_bittiming_s in which the new bit timing
       *                   values are provided.
       *   Returned Value: Zero (OK) is returned on success.  Otherwise -1
       *                   (ERROR)is returned with the errno variable set
       *                    to indicate thenature of the error.
       *   Dependencies:   None
       *
       * REVISIT: There is probably a limitation here:  If there are
       * multiple threads trying to send CAN packets, when one of these
       * threads reconfigures the bitrate, the MCAN hardware will be reset
       * and the context of operation will be lost.  Hence, this IOCTL can
       * only safely be executed in quiescent time periods.
       */

      case CANIOC_SET_BITTIMING:
        {
          const struct canioc_bittiming_s *bt =
            (const struct canioc_bittiming_s *)arg;
          uint32_t brp;
          uint32_t can_bit_quanta;
          uint32_t tmp;
          uint32_t regval;

          DEBUGASSERT(bt != NULL);
          DEBUGASSERT(bt->bt_baud < STM32_PCLK1_FREQUENCY);
          DEBUGASSERT(bt->bt_sjw > 0 && bt->bt_sjw <= 4);
          DEBUGASSERT(bt->bt_tseg1 > 0 && bt->bt_tseg1 <= 16);
          DEBUGASSERT(bt->bt_tseg2 > 0 && bt->bt_tseg2 <=  8);

          regval = stm32can_getreg(priv, STM32_CAN_BTR_OFFSET);

          /* Extract bit timing data
           * tmp is in clocks per bit time
           */

          tmp = STM32_PCLK1_FREQUENCY / bt->bt_baud;

          /* This value is dynamic as requested by user */

          can_bit_quanta = bt->bt_tseg1 + bt->bt_tseg2 + 1;

          if (tmp < can_bit_quanta)
            {
              /* This timing is not possible */

              ret = -EINVAL;
              break;
            }

          /* Otherwise, nquanta is can_bit_quanta, ts1 and ts2 are
           * provided by the user and we calculate brp to achieve
           * can_bit_quanta quanta in the bit times
           */

          else
            {
              brp = (tmp + (can_bit_quanta / 2)) / can_bit_quanta;
              DEBUGASSERT(brp >= 1 && brp <= CAN_BTR_BRP_MAX);
            }

          caninfo("TS1: %"PRIu8 " TS2: %" PRIu8 " BRP: %" PRIu32 "\n",
                  bt->bt_tseg1, bt->bt_tseg2, brp);

          /* Configure bit timing. */

          regval &= ~(CAN_BTR_BRP_MASK | CAN_BTR_TS1_MASK |
                      CAN_BTR_TS2_MASK | CAN_BTR_SJW_MASK);
          regval |= ((brp          - 1) << CAN_BTR_BRP_SHIFT) |
                    ((bt->bt_tseg1 - 1) << CAN_BTR_TS1_SHIFT) |
                    ((bt->bt_tseg2 - 1) << CAN_BTR_TS2_SHIFT) |
                    ((bt->bt_sjw   - 1) << CAN_BTR_SJW_SHIFT);

          /* Bit timing can only be configured in init mode. */

          ret = stm32can_enterinitmode(priv);
          if (ret < 0)
            {
              break;
            }

          stm32can_putreg(priv, STM32_CAN_BTR_OFFSET, regval);

          ret = stm32can_exitinitmode(priv);
          if (ret >= 0)
            {
              priv->baud  = STM32_PCLK1_FREQUENCY /
                (brp * (bt->bt_tseg1 + bt->bt_tseg2 + 1));
            }
        }
        break;

      /* CANIOC_GET_CONNMODES:
       *   Description:    Get the current bus connection modes
       *   Argument:       A pointer to a write-able instance of struct
       *                   canioc_connmodes_s in which the new bus modes will
       *                   be returned.
       *   Returned Value: Zero (OK) is returned on success.  Otherwise -1
       *                   (ERROR)is returned with the errno variable set
       *                   to indicate the nature of the error.
       *   Dependencies:   None
       */

      case CANIOC_GET_CONNMODES:
        {
          struct canioc_connmodes_s *bm =
            (struct canioc_connmodes_s *)arg;
          uint32_t regval;

          DEBUGASSERT(bm != NULL);

          regval          = stm32can_getreg(priv, STM32_CAN_BTR_OFFSET);

          bm->bm_loopback = ((regval & CAN_BTR_LBKM) == CAN_BTR_LBKM);
          bm->bm_silent   = ((regval & CAN_BTR_SILM) == CAN_BTR_SILM);
          ret = OK;
          break;
        }

      /* CANIOC_SET_CONNMODES:
       *   Description:    Set new bus connection modes values
       *   Argument:       A pointer to a read-able instance of struct
       *                   canioc_connmodes_s in which the new bus modes
       *                   are provided.
       *   Returned Value: Zero (OK) is returned on success.  Otherwise -1
       *                   (ERROR) is returned with the errno variable set
       *                   to indicate the nature of the error.
       *   Dependencies:   None
       */

      case CANIOC_SET_CONNMODES:
        {
          struct canioc_connmodes_s *bm =
            (struct canioc_connmodes_s *)arg;
          uint32_t regval;

          DEBUGASSERT(bm != NULL);

          regval = stm32can_getreg(priv, STM32_CAN_BTR_OFFSET);

          if (bm->bm_loopback)
            {
              regval |= CAN_BTR_LBKM;
            }
          else
            {
              regval &= ~CAN_BTR_LBKM;
            }

          if (bm->bm_silent)
            {
              regval |= CAN_BTR_SILM;
            }
          else
            {
              regval &= ~CAN_BTR_SILM;
            }

          /* This register can only be configured in init mode. */

          ret = stm32can_enterinitmode(priv);
          if (ret < 0)
            {
              break;
            }

          stm32can_putreg(priv, STM32_CAN_BTR_OFFSET, regval);

          ret = stm32can_exitinitmode(priv);
        }
        break;

#ifdef CONFIG_CAN_EXTID
      /* CANIOC_ADD_EXTFILTER:
       *   Description:    Add an address filter for a extended 29 bit
       *                   address.
       *   Argument:       A reference to struct canioc_extfilter_s
       *   Returned Value: A non-negative filter ID is returned on success.
       *                   Otherwise -1 (ERROR) is returned with the errno
       *                   variable set to indicate the nature of the error.
       */

      case CANIOC_ADD_EXTFILTER:
        {
          DEBUGASSERT(arg != 0);
          ret = stm32can_addextfilter(priv,
                                      (struct canioc_extfilter_s *)arg);
        }
        break;

      /* CANIOC_DEL_EXTFILTER:
       *   Description:    Remove an address filter for a standard 29 bit
       *                   address.
       *   Argument:       The filter index previously returned by the
       *                   CANIOC_ADD_EXTFILTER command
       *   Returned Value: Zero (OK) is returned on success.  Otherwise -1
       *                   (ERROR)is returned with the errno variable set
       *                   to indicate the nature of the error.
       */

      case CANIOC_DEL_EXTFILTER:
        {
#if 0 /* Unimplemented */
          DEBUGASSERT(arg <= priv->config->nextfilters);
#endif
          ret = stm32can_delextfilter(priv, (int)arg);
        }
        break;
#endif

      /* CANIOC_ADD_STDFILTER:
       *   Description:    Add an address filter for a standard 11 bit
       *                   address.
       *   Argument:       A reference to struct canioc_stdfilter_s
       *   Returned Value: A non-negative filter ID is returned on success.
       *                   Otherwise -1 (ERROR) is returned with the errno
       *                   variable set to indicate the nature of the error.
       */

      case CANIOC_ADD_STDFILTER:
        {
          DEBUGASSERT(arg != 0);
          ret = stm32can_addstdfilter(priv,
                                      (struct canioc_stdfilter_s *)arg);
        }
        break;

      /* CANIOC_DEL_STDFILTER:
       *   Description:    Remove an address filter for a standard 11 bit
       *                   address.
       *   Argument:       The filter index previously returned by the
       *                   CANIOC_ADD_STDFILTER command
       *   Returned Value: Zero (OK) is returned on success.  Otherwise -1
       *                   (ERROR) is returned with the errno variable set
       *                   to indicate the nature of the error.
       */

      case CANIOC_DEL_STDFILTER:
        {
#if 0 /* Unimplemented */
          DEBUGASSERT(arg <= priv->config->nstdfilters);
#endif
          ret = stm32can_delstdfilter(priv, (int)arg);
        }
        break;

      case CANIOC_SET_NART:
        {
          uint32_t regval;

          ret = stm32can_enterinitmode(priv);
          if (ret != 0)
            {
              return ret;
            }

          regval = stm32can_getreg(priv, STM32_CAN_MCR_OFFSET);
          if (arg == 1)
            {
              regval |= CAN_MCR_NART;
            }
          else
            {
              regval &= ~CAN_MCR_NART;
            }

          stm32can_putreg(priv, STM32_CAN_MCR_OFFSET, regval);
          return stm32can_exitinitmode(priv);
        }
        break;

      case CANIOC_SET_ABOM:
        {
          uint32_t regval;

          ret = stm32can_enterinitmode(priv);
          if (ret != 0)
            {
              return ret;
            }

          regval = stm32can_getreg(priv, STM32_CAN_MCR_OFFSET);
          if (arg == 1)
            {
              regval |= CAN_MCR_ABOM;
            }
          else
            {
              regval &= ~CAN_MCR_ABOM;
            }

          stm32can_putreg(priv, STM32_CAN_MCR_OFFSET, regval);
          return stm32can_exitinitmode(priv);
        }
        break;

      /* Unsupported/unrecognized command */

      default:
        canerr("ERROR: Unrecognized command: %04x\n", cmd);
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: stm32can_remoterequest
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

static int stm32can_remoterequest(struct can_dev_s *dev, uint16_t id)
{
#warning "Remote request not implemented"
  return -ENOSYS;
}

/****************************************************************************
 * Name: stm32can_send
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

static int stm32can_send(struct can_dev_s *dev,
                         struct can_msg_s *msg)
{
  struct stm32_can_s *priv = dev->cd_priv;
  uint8_t *ptr;
  uint32_t regval;
  uint32_t tmp;
  int dlc;
  int txmb;

  caninfo("CAN%" PRIu8 " ID: %" PRIu32 " DLC: %" PRIu8 "\n",
          priv->port, (uint32_t)msg->cm_hdr.ch_id, msg->cm_hdr.ch_dlc);

  /* Select one empty transmit mailbox */

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
      canerr("ERROR: No available mailbox\n");
      return -EBUSY;
    }

  /* Clear TXRQ, RTR, IDE, EXID, and STID fields */

  regval  = stm32can_getreg(priv, STM32_CAN_TIR_OFFSET(txmb));
  regval &= ~(CAN_TIR_TXRQ | CAN_TIR_RTR | CAN_TIR_IDE |
              CAN_TIR_EXID_MASK | CAN_TIR_STID_MASK);
  stm32can_putreg(priv, STM32_CAN_TIR_OFFSET(txmb), regval);

  /* Set up the ID, standard 11-bit or extended 29-bit. */

#ifdef CONFIG_CAN_EXTID
  regval &= ~CAN_TIR_EXID_MASK;
  if (msg->cm_hdr.ch_extid)
    {
      DEBUGASSERT(msg->cm_hdr.ch_id < (1 << 29));
      regval |= (msg->cm_hdr.ch_id << CAN_TIR_EXID_SHIFT) | CAN_TIR_IDE;
    }
  else
    {
      DEBUGASSERT(msg->cm_hdr.ch_id < (1 << 11));
      regval |= msg->cm_hdr.ch_id << CAN_TIR_STID_SHIFT;
    }

#else
  regval |= (((uint32_t) msg->cm_hdr.ch_id << CAN_TIR_STID_SHIFT) &
               CAN_TIR_STID_MASK);

#endif

#ifdef CONFIG_CAN_USE_RTR
  regval |= (msg->cm_hdr.ch_rtr ? CAN_TIR_RTR : 0);
#endif

  stm32can_putreg(priv, STM32_CAN_TIR_OFFSET(txmb), regval);

  /* Set up the DLC */

  dlc     = msg->cm_hdr.ch_dlc;
  regval  = stm32can_getreg(priv, STM32_CAN_TDTR_OFFSET(txmb));
  regval &= ~(CAN_TDTR_DLC_MASK | CAN_TDTR_TGT);
  regval |= (uint32_t)dlc << CAN_TDTR_DLC_SHIFT;
  stm32can_putreg(priv, STM32_CAN_TDTR_OFFSET(txmb), regval);

  /* Set up the data fields */

  ptr    = msg->cm_data;
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

  /* Enable the transmit mailbox empty interrupt (may already be enabled) */

  regval  = stm32can_getreg(priv, STM32_CAN_IER_OFFSET);
  regval |= CAN_IER_TMEIE;
  stm32can_putreg(priv, STM32_CAN_IER_OFFSET, regval);

  /* Request transmission */

  regval  = stm32can_getreg(priv, STM32_CAN_TIR_OFFSET(txmb));
  regval |= CAN_TIR_TXRQ;  /* Transmit Mailbox Request */
  stm32can_putreg(priv, STM32_CAN_TIR_OFFSET(txmb), regval);

  stm32can_dumpmbregs(priv, "After send");
  return OK;
}

/****************************************************************************
 * Name: stm32can_txready
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

static bool stm32can_txready(struct can_dev_s *dev)
{
  struct stm32_can_s *priv = dev->cd_priv;
  uint32_t regval;

  /* Return true if any mailbox is available */

  regval = stm32can_getreg(priv, STM32_CAN_TSR_OFFSET);
  caninfo("CAN%" PRIu8 " TSR: %08" PRIx32 "\n", priv->port, regval);

  return stm32can_txmb0empty(regval) || stm32can_txmb1empty(regval) ||
         stm32can_txmb2empty(regval);
}

/****************************************************************************
 * Name: stm32can_txempty
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

static bool stm32can_txempty(struct can_dev_s *dev)
{
  struct stm32_can_s *priv = dev->cd_priv;
  uint32_t regval;

  /* Return true if all mailboxes are available */

  regval = stm32can_getreg(priv, STM32_CAN_TSR_OFFSET);
  caninfo("CAN%" PRIu8 " TSR: %08" PRIx32 "\n", priv->port, regval);

  return stm32can_txmb0empty(regval) && stm32can_txmb1empty(regval) &&
         stm32can_txmb2empty(regval);
}

/****************************************************************************
 * Name: stm32can_rxinterrupt
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

static int stm32can_rxinterrupt(struct can_dev_s *dev, int rxmb)
{
  struct stm32_can_s *priv;
  struct can_hdr_s hdr;
  uint8_t data[CAN_MAXDATALEN];
  uint32_t regval;
  int npending;
  int ret;

  DEBUGASSERT(dev != NULL && dev->cd_priv != NULL);
  priv = dev->cd_priv;

  /* Verify that a message is pending in the FIFO */

  regval   = stm32can_getreg(priv, STM32_CAN_RFR_OFFSET(rxmb));
  npending = (regval & CAN_RFR_FMP_MASK) >> CAN_RFR_FMP_SHIFT;
  if (npending < 1)
    {
      canwarn("WARNING: No messages pending\n");
      return OK;
    }

  if (rxmb == 0)
    {
      stm32can_dumpmbregs(priv, "RX0 interrupt");
    }
  else
    {
      stm32can_dumpmbregs(priv, "RX1 interrupt");
    }

  /* Get the CAN identifier. */

  regval = stm32can_getreg(priv, STM32_CAN_RIR_OFFSET(rxmb));

#ifdef CONFIG_CAN_EXTID
  if ((regval & CAN_RIR_IDE) != 0)
    {
      hdr.ch_id    = (regval & CAN_RIR_EXID_MASK) >> CAN_RIR_EXID_SHIFT;
      hdr.ch_extid = true;
    }
  else
    {
      hdr.ch_id    = (regval & CAN_RIR_STID_MASK) >> CAN_RIR_STID_SHIFT;
      hdr.ch_extid = false;
    }
#else
  if ((regval & CAN_RIR_IDE) != 0)
    {
      canerr("ERROR: Received message with extended identifier.  Dropped\n");
      ret = -ENOSYS;
      goto errout;
    }

  hdr.ch_id = (regval & CAN_RIR_STID_MASK) >> CAN_RIR_STID_SHIFT;
#endif

  /* Clear the error indication and unused bits */

#ifdef CONFIG_CAN_ERRORS
  hdr.ch_error  = 0; /* Error reporting not supported */
#endif
  hdr.ch_unused = 0;

  /* Extract the RTR bit */

  hdr.ch_rtr = (regval & CAN_RIR_RTR) != 0;

  /* Get the DLC */

  regval     = stm32can_getreg(priv, STM32_CAN_RDTR_OFFSET(rxmb));
  hdr.ch_dlc = (regval & CAN_RDTR_DLC_MASK) >> CAN_RDTR_DLC_SHIFT;

  /* Save the message data */

  regval  = stm32can_getreg(priv, STM32_CAN_RDLR_OFFSET(rxmb));
  data[0] = (regval & CAN_RDLR_DATA0_MASK) >> CAN_RDLR_DATA0_SHIFT;
  data[1] = (regval & CAN_RDLR_DATA1_MASK) >> CAN_RDLR_DATA1_SHIFT;
  data[2] = (regval & CAN_RDLR_DATA2_MASK) >> CAN_RDLR_DATA2_SHIFT;
  data[3] = (regval & CAN_RDLR_DATA3_MASK) >> CAN_RDLR_DATA3_SHIFT;

  regval  = stm32can_getreg(priv, STM32_CAN_RDHR_OFFSET(rxmb));
  data[4] = (regval & CAN_RDHR_DATA4_MASK) >> CAN_RDHR_DATA4_SHIFT;
  data[5] = (regval & CAN_RDHR_DATA5_MASK) >> CAN_RDHR_DATA5_SHIFT;
  data[6] = (regval & CAN_RDHR_DATA6_MASK) >> CAN_RDHR_DATA6_SHIFT;
  data[7] = (regval & CAN_RDHR_DATA7_MASK) >> CAN_RDHR_DATA7_SHIFT;

  /* Provide the data to the upper half driver */

  ret = can_receive(dev, &hdr, data);

  /* Release the FIFO */

#ifndef CONFIG_CAN_EXTID
errout:
#endif
  regval  = stm32can_getreg(priv, STM32_CAN_RFR_OFFSET(rxmb));
  regval |= CAN_RFR_RFOM;
  stm32can_putreg(priv, STM32_CAN_RFR_OFFSET(rxmb), regval);
  return ret;
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
  struct can_dev_s *dev = (struct can_dev_s *)arg;
  return stm32can_rxinterrupt(dev, 0);
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
  struct can_dev_s *dev = (struct can_dev_s *)arg;
  return stm32can_rxinterrupt(dev, 1);
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
  struct can_dev_s *dev = (struct can_dev_s *)arg;
  struct stm32_can_s *priv;
  uint32_t regval;

  DEBUGASSERT(dev != NULL && dev->cd_priv != NULL);
  priv = dev->cd_priv;

  /* Get the transmit status */

  regval = stm32can_getreg(priv, STM32_CAN_TSR_OFFSET);

  /* Check for RQCP0: Request completed mailbox 0 */

  if ((regval & CAN_TSR_RQCP0) != 0)
    {
      /* Writing '1' to RCP0 clears RCP0 and all the status bits (TXOK0,
       * ALST0 and TERR0) for Mailbox 0.
       */

      stm32can_putreg(priv, STM32_CAN_TSR_OFFSET, CAN_TSR_RQCP0);

      /* Tell the upper half that the transfer is finished. */

      can_txdone(dev);
    }

  /* Check for RQCP1: Request completed mailbox 1 */

  if ((regval & CAN_TSR_RQCP1) != 0)
    {
      /* Writing '1' to RCP1 clears RCP1 and all the status bits (TXOK1,
       * ALST1 and TERR1) for Mailbox 1.
       */

      stm32can_putreg(priv, STM32_CAN_TSR_OFFSET, CAN_TSR_RQCP1);

      /* Tell the upper half that the transfer is finished. */

      can_txdone(dev);
    }

  /* Check for RQCP2: Request completed mailbox 2 */

  if ((regval & CAN_TSR_RQCP2) != 0)
    {
      /* Writing '1' to RCP2 clears RCP2 and all the status bits (TXOK2,
       * ALST2 and TERR2) for Mailbox 2.
       */

      stm32can_putreg(priv, STM32_CAN_TSR_OFFSET, CAN_TSR_RQCP2);

      /* Tell the upper half that the transfer is finished. */

      can_txdone(dev);
    }

  return OK;
}

#ifdef CONFIG_CAN_ERRORS
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
  struct can_dev_s   *dev     = (struct can_dev_s *)arg;
  struct stm32_can_s *priv    = NULL;
  struct can_hdr_s    hdr;
  uint32_t            regval  = 0;
  uint16_t            errbits = 0;
  uint8_t             data[CAN_ERROR_DLC];
  int                 ret     = OK;

  DEBUGASSERT(dev != NULL && dev->cd_priv != NULL);
  priv = dev->cd_priv;

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

          data[1] |= (CAN_ERROR1_RXWARNING | CAN_ERROR1_TXWARNING);
          errbits |= CAN_ERROR_CONTROLLER;
        }

      if (regval & CAN_ESR_EPVF)
        {
          /* Error passive flag */

          data[1] |= (CAN_ERROR1_RXPASSIVE | CAN_ERROR1_TXPASSIVE);
          errbits |= CAN_ERROR_CONTROLLER;
        }

      if (regval & CAN_ESR_BOFF)
        {
          /* Bus-off flag */

          errbits |= CAN_ERROR_BUSOFF;
        }

      /* Last error code */

      if (regval & CAN_ESR_LEC_MASK)
        {
          if (regval & CAN_ESR_STUFFERROR)
            {
              /* Stuff Error */

              errbits |= CAN_ERROR_PROTOCOL;
              data[2] |= CAN_ERROR2_STUFF;
            }
          else if (regval & CAN_ESR_FORMERROR)
            {
              /* Format Error */

              errbits |= CAN_ERROR_PROTOCOL;
              data[2] |= CAN_ERROR2_FORM;
            }
          else if (regval & CAN_ESR_ACKERROR)
            {
              /* Acknowledge Error */

              errbits |= CAN_ERROR_NOACK;
            }
          else if (regval & CAN_ESR_BRECERROR)
            {
              /* Bit recessive Error */

              errbits |= CAN_ERROR_PROTOCOL;
              data[2] |= CAN_ERROR2_BIT1;
            }
          else if (regval & CAN_ESR_BDOMERROR)
            {
              /* Bit dominant Error */

              errbits |= CAN_ERROR_PROTOCOL;
              data[2] |= CAN_ERROR2_BIT0;
            }
          else if (regval & CAN_ESR_CRCERRPR)
            {
              /* Receive CRC Error */

              errbits |= CAN_ERROR_PROTOCOL;
              data[3] |= CAN_ERROR3_CRCSEQ;
            }
        }

      /* Get transmit status register */

      regval = stm32can_getreg(priv, STM32_CAN_TSR_OFFSET);

      if (regval & CAN_TSR_ALST0 || regval & CAN_TSR_ALST1 ||
          regval & CAN_TSR_ALST2)
        {
          /* Lost arbitration Error */

          errbits |= CAN_ERROR_LOSTARB;
        }

      /* Clear TSR register */

      stm32can_putreg(priv, STM32_CAN_TSR_OFFSET, regval);

      /* Clear ERRI flag */

      stm32can_putreg(priv, STM32_CAN_MSR_OFFSET, CAN_MSR_ERRI);
    }

  /* TODO: RX overflow and TX overflow */

  /* Report a CAN error */

  if (errbits != 0)
    {
      canerr("ERROR: errbits = %08" PRIx16 "\n", errbits);

      /* Format the CAN header for the error report. */

      hdr.ch_id     = errbits;
      hdr.ch_dlc    = CAN_ERROR_DLC;
      hdr.ch_rtr    = 0;
      hdr.ch_error  = 1;
#ifdef CONFIG_CAN_EXTID
      hdr.ch_extid  = 0;
#endif
      hdr.ch_unused = 0;

      /* And provide the error report to the upper half logic */

      ret = can_receive(dev, &hdr, data);
      if (ret < 0)
        {
          canerr("ERROR: can_receive failed: %d\n", ret);
        }
    }

  return ret;
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
 *   priv - A reference to the CAN block status
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

  caninfo("CAN%" PRIu8 " PCLK1: %lu baud: %" PRIu32 "\n",
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
      DEBUGASSERT(brp >= 1 && brp <= CAN_BTR_BRP_MAX);
    }

  caninfo("TS1: %" PRIu32 " TS2: %" PRIu32 " BRP: %" PRIu32 "\n",
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
 * Name: stm32can_enterinitmode
 *
 * Description:
 *   Put the CAN cell in Initialization mode. This only disconnects the CAN
 *   peripheral, no registers are changed. The initialization mode is
 *   required to change the baud rate.
 *
 * Input Parameters:
 *   priv - A pointer to the private data structure for this CAN block
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int stm32can_enterinitmode(struct stm32_can_s *priv)
{
  uint32_t regval;
  volatile uint32_t timeout;

  caninfo("CAN%" PRIu8 "\n", priv->port);

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
      canerr("ERROR: Timed out waiting to enter initialization mode\n");
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
 *   priv - A pointer to the private data structure for this CAN block
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
      canerr("ERROR: Timed out waiting to exit initialization mode: %08"
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
 *   priv - A pointer to the private data structure for this CAN block
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int stm32can_cellinit(struct stm32_can_s *priv)
{
  uint32_t regval;
  int ret;

  caninfo("CAN%" PRIu8 "\n", priv->port);

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
      canerr("ERROR: Failed to set bit timing: %d\n", ret);
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
 *   (but reserves CAN_NFILTERS/2 through CAN_NFILTERS-1).
 *
 *   32-bit IdMask mode is configured.  However, both the ID and the MASK
 *   are set to zero thus suppressing all filtering because anything masked
 *   with zero matches zero.
 *
 * Input Parameters:
 *   priv - A pointer to the private data structure for this CAN block
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int stm32can_filterinit(struct stm32_can_s *priv)
{
  uint32_t regval;
  uint32_t bitmask;

  caninfo("CAN%" PRIu8 " filter: %" PRIu8 "\n", priv->port, priv->filter);

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
  regval &= CAN_FMR_CAN2SB_MASK;
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
 * Name: stm32can_addextfilter
 *
 * Description:
 *   Add a filter for extended CAN IDs
 *
 * Input Parameters:
 *   priv - A pointer to the private data structure for this CAN block
 *   arg  - A pointer to a structure describing the filter
 *
 * Returned Value:
 *   A non-negative filter ID is returned on success.
 *   Otherwise -1 (ERROR) is returned with the errno
 *   set to indicate the nature of the error.
 *
 ****************************************************************************/

#ifdef CONFIG_CAN_EXTID
static int stm32can_addextfilter(struct stm32_can_s *priv,
                                 struct canioc_extfilter_s *arg)
{
  return -ENOTTY;
}
#endif

/****************************************************************************
 * Name: stm32can_delextfilter
 *
 * Description:
 *   Remove a filter for extended CAN IDs
 *
 * Input Parameters:
 *   priv - A pointer to the private data structure for this CAN block
 *   arg  - The filter index previously returned by the
 *            CANIOC_ADD_EXTFILTER command
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise -1 (ERROR)
 *   returned with the errno variable set to indicate the
 *   of the error.
 *
 ****************************************************************************/

#ifdef CONFIG_CAN_EXTID
static int stm32can_delextfilter(struct stm32_can_s *priv, int arg)
{
  return -ENOTTY;
}
#endif

/****************************************************************************
 * Name: stm32can_addstdfilter
 *
 * Description:
 *   Add a filter for standard CAN IDs
 *
 * Input Parameters:
 *   priv - A pointer to the private data structure for this CAN block
 *   arg  - A pointer to a structure describing the filter
 *
 * Returned Value:
 *   A non-negative filter ID is returned on success.
 *   Otherwise -1 (ERROR) is returned with the errno
 *   set to indicate the nature of the error.
 *
 ****************************************************************************/

static int stm32can_addstdfilter(struct stm32_can_s *priv,
                                 struct canioc_stdfilter_s *arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: stm32can_delstdfilter
 *
 * Description:
 *   Remove a filter for standard CAN IDs
 *
 * Input Parameters:
 *   priv - A pointer to the private data structure for this CAN block
 *   arg  - The filter index previously returned by the
 *            CANIOC_ADD_STDFILTER command
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise -1 (ERROR)
 *   returned with the errno variable set to indicate the
 *   of the error.
 *
 ****************************************************************************/

static int stm32can_delstdfilter(struct stm32_can_s *priv, int arg)
{
  return -ENOTTY;
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
 * Name: stm32_caninitialize
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

struct can_dev_s *stm32_caninitialize(int port)
{
  struct can_dev_s *dev = NULL;

  caninfo("CAN%" PRIu8 "\n", port);

  /* NOTE:  Peripherical clocking for CAN1 and/or CAN2 was already provided
   * by stm32_clockconfig() early in the reset sequence.
   */

#ifdef CONFIG_STM32_CAN1
  if (port == 1)
    {
      /* Select the CAN1 device structure */

      dev = &g_can1dev;

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

      dev = &g_can2dev;

      /* Configure CAN2 pins.  The ambiguous settings in the stm32*_pinmap.h
       * file must have been disambiguated in the board.h file.
       */

      stm32_configgpio(GPIO_CAN2_RX);
      stm32_configgpio(GPIO_CAN2_TX);
    }
  else
#endif
    {
      canerr("ERROR: Unsupported port %d\n", port);
      return NULL;
    }

  return dev;
}

#endif /* CONFIG_CAN && (CONFIG_STM32_CAN1 || CONFIG_STM32_CAN2) */
