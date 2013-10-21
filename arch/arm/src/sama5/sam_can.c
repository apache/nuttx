/************************************************************************************
 * arch/arm/src/sama5/sam_can.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *
 *   SAMA5D3 Series Data Sheet
 *   Atmel NoOS sample code.
 *
 * The Atmel sample code has a BSD compatibile license that requires this
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
 ************************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <nuttx/can.h>

#include "up_internal.h"
#include "up_arch.h"

#include "os_internal.h"

#include "chip/sam_pinmap.h"
#include "sam_periphclks.h"
#include "sam_pio.h"
#include "sam_can.h"

#if defined(CONFIG_CAN) && (defined(CONFIG_SAMA5_CAN0) || defined(CONFIG_SAMA5_CAN1))

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Delays *******************************************************************/
/* Time out for INAK bit */

#define INAK_TIMEOUT 65535

/* Mailboxes ****************************************************************/

/* Bit timing ***************************************************************/

/* Clocking */

#if BOARD_MCK_FREQUENCY <= SAM_CAN_MAXPERCLK
#  define CAN_FREQUENCY BOARD_MCK_FREQUENCY
#  define CAN_PCR_DIV PMC_PCR_DIV1
#elif (BOARD_MCK_FREQUENCY >> 1) <= SAM_CAN_MAXPERCLK
#  define CAN_FREQUENCY (BOARD_MCK_FREQUENCY >> 1)
#  define CAN_PCR_DIV PMC_PCR_DIV2
#elif (BOARD_MCK_FREQUENCY >> 2) <= SAM_CAN_MAXPERCLK
#  define CAN_FREQUENCY (BOARD_MCK_FREQUENCY >> 2)
#  define CAN_PCR_DIV PMC_PCR_DIV4
#elif (BOARD_MCK_FREQUENCY >> 3) <= SAM_CAN_MAXPERCLK
#  define CAN_FREQUENCY (BOARD_MCK_FREQUENCY >> 3)
#  define CAN_PCR_DIV PMC_PCR_DIV8
#else
#  error Cannot realize ADC input frequency
#endif

#define CAN_BIT_QUANTA (CONFIG_CAN_TSEG1 + CONFIG_CAN_TSEG2 + 1)

/* Debug ********************************************************************/
/* Non-standard debug that may be enabled just for testing CAN */

#ifdef CONFIG_DEBUG_CAN
#  define candbg    dbg
#  define canvdbg   vdbg
#  define canlldbg  lldbg
#  define canllvdbg llvdbg
#else
#  define candbg(x...)
#  define canvdbg(x...)
#  define canlldbg(x...)
#  define canllvdbg(x...)
#endif

#if !defined(CONFIG_DEBUG) || !defined(CONFIG_DEBUG_CAN)
#  undef CONFIG_CAN_REGDEBUG
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sam_can_s
{
  uint8_t      port;    /* CAN port number (1 or 2) */
  uint8_t      pid;     /* CAN periperal ID/IRQ number */
  xcpt_t       handler; /* CAN interrupt handler */
  uintptr_t    base;    /* Base address of the CAN control registers */
  uint32_t     baud;    /* Configured baud */
  pio_pinset_t rxpinset; /* RX pin configuration */
  pio_pinset_t txpinset; /* TX pin configuration */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* CAN Register access */

static uint32_t can_getreg(struct sam_can_s *priv, int offset);
static void can_putreg(struct sam_can_s *priv, int offset, uint32_t regval);
#ifdef CONFIG_CAN_REGDEBUG
static void can_dumpctrlregs(struct sam_can_s *priv, FAR const char *msg);
static void can_dumpmbregs(struct sam_can_s *priv, FAR const char *msg);
#else
#  define can_dumpctrlregs(priv,msg)
#  define can_dumpmbregs(priv,msg)
#endif

/* CAN driver methods */

static void can_reset(FAR struct can_dev_s *dev);
static int  can_setup(FAR struct can_dev_s *dev);
static void can_shutdown(FAR struct can_dev_s *dev);
static void can_rxint(FAR struct can_dev_s *dev, bool enable);
static void can_txint(FAR struct can_dev_s *dev, bool enable);
static int  can_ioctl(FAR struct can_dev_s *dev, int cmd, unsigned long arg);
static int  can_remoterequest(FAR struct can_dev_s *dev, uint16_t id);
static int  can_send(FAR struct can_dev_s *dev, FAR struct can_msg_s *msg);
static bool can_txready(FAR struct can_dev_s *dev);
static bool can_txempty(FAR struct can_dev_s *dev);

/* CAN interrupt handling */

static void can_interrupt(struct sam_can_s *priv);
#ifdef CONFIG_SAMA5_CAN0
static int  can0_interrupt(int irq, void *context);
#endif
#ifdef CONFIG_SAMA5_CAN1
static int  can1_interrupt(int irq, void *context);
#endif
#if defined(CONFIG_CAN) && (defined() || defined())

/* Initialization */

static int  can_bittiming(struct sam_can_s *priv);
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
static const struct sam_can_s g_can0priv =
{
  .port             = 0,
  .pid              = SAM_PID_CAN0,
  .handler          = can0_interrupt,
  .base             = SAM_CAN0_VBASE,
  .baud             = CONFIG_CAN0_BAUD,
  .rxpinset         = PIO_CAN0_RX;
  .txpinset         = PIO_CAN0_TX;
};

static struct can_dev_s g_can0dev =
{
  .cd_ops           = &g_canops,
  .cd_priv          = (void *)&g_can0priv,
};
#endif

#ifdef CONFIG_SAMA5_CAN1
static const struct sam_can_s g_can1priv =
{
  .port             = 1,
  .pid              = SAM_PID_CAN1,
  .handler          = can1_interrupt,
  .base             = SAM_CAN1_VBASE,
  .baud             = CONFIG_CAN1_BAUD,
  .rxpinset         = PIO_CAN1_RX;
  .txpinset         = PIO_CAN1_TX;
};

static struct can_dev_s g_can1dev =
{
  .cd_ops           = &g_canops,
  .cd_priv          = (void *)&g_can1priv,
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
 *   priv - A reference to the CAN block status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *
 ****************************************************************************/

#ifdef CONFIG_CAN_REGDEBUG
static uint32_t can_getreg(struct sam_can_s *priv, int offset)
{
  static uintptr_t prevaddr = 0;
  static uint32_t preval = 0;
  static unsigned int count = 0;
  uintptr_t regaddr;
  uint32_t regval;

  /* Read the value from the register */

  regaddr = priv->base + offset;
  regval  = getreg32(regaddr);

  /* Is this the same value that we read from the same register last time?
   * Are we polling the register?  If so, suppress some of the output.
   */

  if (regaddr == prevaddr && regval == preval)
    {
      if (count == 0xffffffff || ++count > 3)
        {
           if (count == 4)
             {
               lldbg("...\n");
             }
          return regval;
        }
    }

  /* No this is a new address or value */

  else
    {
       /* Did we print "..." for the previous value? */

       if (count > 3)
         {
           /* Yes.. then show how many times the value repeated */

           lldbg("[repeats %d more times]\n", count-3);
         }

       /* Save the new address, value, and count */

       prevaddr = regaddr;
       preval   = regval;
       count    = 1;
    }

  /* Show the register value read */

  lldbg("%08x->%08x\n", addr, regval);
  return regval;
}

#else
static uint32_t can_getreg(struct sam_can_s *priv, int offset)
{
  return getreg32(priv->base + offset);
}

#endif

/****************************************************************************
 * Name: can_putreg
 *
 * Description:
 *   Set the value of a CAN register.
 *
 * Input Parameters:
 *   priv - A reference to the CAN block status
 *   offset - The offset to the register to write
 *   regval - The value to write to the register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_CAN_REGDEBUG
static void can_putreg(struct sam_can_s *priv, int offset, uint32_t regval)
{
  uintptr_t regaddr = priv->base + offset;

  /* Show the register value being written */

  lldbg("%08x<-%08x\n", regaddr, regval);

  /* Write the value */

  putreg32(regval, regaddr);
}

#else
static void can_putreg(struct sam_can_s *priv, int offset, uint32_t regval)
{
  putreg32(regval, priv->base + offset);
}

#endif

/****************************************************************************
 * Name: can_dumpctrlregs
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

#ifdef CONFIG_CAN_REGDEBUG
static void can_dumpctrlregs(struct sam_can_s *priv, FAR const char *msg)
{
  if (msg)
    {
      canlldbg("Control Registers: %s\n", msg);
    }
  else
    {
      canlldbg("Control Registers:\n");
    }

  /* CAN control and status registers */

  lldbg("   MR: %08x      IMR: %08x      SR: %08x\n",
        getreg32(priv->base + SAM_CAN_MR_OFFSET),
        getreg32(priv->base + SAM_CAN_IMR_OFFSET),
        getreg32(priv->base + SAM_CAN_SR_OFFSET));

  lldbg("   BR: %08x      TIM: %08x TIMESTP: %08x\n",
        getreg32(priv->base + SAM_CAN_BR_OFFSET),
        getreg32(priv->base + SAM_CAN_TIM_OFFSET),
        getreg32(priv->base + SAM_CAN_TIMESTP_OFFSET));

  lldbg("  ECR: %08x     WPMR: %08x    WPSR: %08x\n",
        getreg32(priv->base + SAM_CAN_ECR_OFFSET),
        getreg32(priv->base + SAM_CAN_TCR_OFFSET),
        getreg32(priv->base + SAM_CAN_ACR_OFFSET));
}
#endif

/****************************************************************************
 * Name: can_dumpmbregs
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

#ifdef CONFIG_CAN_REGDEBUG
static void can_dumpmbregs(struct sam_can_s *priv, FAR const char *msg)
{
  uintptr_t mbbase;
  int i;

  if (msg)
    {
      canlldbg("Mailbox Registers: %s\n", msg);
    }
  else
    {
      canlldbg("Mailbox Registers:\n");
    }

  for (i = 0; i < SAM_CAN_NMAILBOXES; i++)
    {
      mbbase = priv->base + SAM_CAN_MBn_OFFSET(i);
      lldbg("  MB%d:\n", i);

      /* CAN mailbox registers */

      lldbg("    MMR: %08x MAM: %08x MID: %08x MFID: %08x\n",
            getreg32(mbbase + SAM_CAN_MMR_OFFSET),
            getreg32(mbbase + SAM_CAN_MAM_OFFSET),
            getreg32(mbbase + SAM_CAN_MID_OFFSET),
            getreg32(mbbase + SAM_CAN_MFID_OFFSET));

      lldbg("    MSR: %08x MDL: %08x MDH: %08x\n",
            getreg32(mbbase + SAM_CAN_MSR_OFFSET),
            getreg32(mbbase + SAM_CAN_MDL_OFFSET),
            getreg32(mbbase + SAM_CAN_MDH_OFFSET));
    }
}
#endif

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

static void can_reset(FAR struct can_dev_s *dev)
{
  FAR struct sam_can_s *priv = dev->cd_priv;
  uint32_t regval;
  uint32_t regbit = 0;
  irqstate_t flags;

  canllvdbg("CAN%d\n", priv->port);

  /* Get the bits in the AHB1RSTR register needed to reset this CAN device */

#ifdef CONFIG_SAMA5_CAN0
  if (priv->port == 1)
    {
#warning Missing logic
    }
  else
#endif
#ifdef CONFIG_SAMA5_CAN1
  if (priv->port == 2)
    {
#warning Missing logic
    }
  else
#endif
    {
      canlldbg("Unsupported port %d\n", priv->port);
      return;
    }

  /* Disable interrupts momentary to stop any ongoing CAN event processing and
   * to prevent any concurrent access to the AHB1RSTR register.
  */

  flags = irqsave();

  /* Reset the CAN */
#warning Missing logic

  irqrestore(flags);
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

static int can_setup(FAR struct can_dev_s *dev)
{
  FAR struct sam_can_s *priv = dev->cd_priv;
  int ret;

  canllvdbg("CAN%d pid: %d\n", priv->port, priv->pid);

  /* CAN hardware initialization */

  ret = can_hwinitialize(priv);
  if (ret < 0)
    {
      canlldbg("CAN%d H/W initialization failed: %d\n", priv->port, ret);
      return ret;
    }

  can_dumpctrlregs(priv, "After cell initialization");
  can_dumpmbregs(priv, NULL);

  /* Attach the CAN interrupt handler */

  ret = irq_attach(priv->pid, priv->handler);
  if (ret < 0)
    {
      canlldbg("Failed to attach CAN%d IRQ (%d)", priv->port, priv->pid);
      return ret;
    }

  /* Enable the interrupts at the NVIC.  Interrupts arestill disabled in
   * the CAN module.  Since we coming out of reset here, there should be
   * no pending interrupts.
   */

  up_enable_irq(priv->pid);
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

static void can_shutdown(FAR struct can_dev_s *dev)
{
  FAR struct sam_can_s *priv = dev->cd_priv;

  canllvdbg("CAN%d\n", priv->port);

  /* Disable the CAN interrupts */

  up_disable_irq(priv->pid);

  /* Detach the CAN interrupt handler */

  irq_detach(priv->pid);

  /* And reset the hardware */

  can_reset(dev);
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

static void can_rxint(FAR struct can_dev_s *dev, bool enable)
{
  FAR struct sam_can_s *priv = dev->cd_priv;

  canllvdbg("CAN%d enable: %d\n", priv->port, enable);

  /* Enable/disable the message pending interrupt */

  if (enable)
    {
       sam_putreg(xxxx, SAM_CAN_IER_OFFSET);
#warning Missing logic
    }
  else
    {
       sam_putreg(xxxx, SAM_CAN_IDR_OFFSET);
#warning Missing logic
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

static void can_txint(FAR struct can_dev_s *dev, bool enable)
{
  FAR struct sam_can_s *priv = dev->cd_priv;

  canllvdbg("CAN%d enable: %d\n", priv->port, enable);

  /* Support only disabling the transmit mailbox interrupt */
#warning Missing logic

  if (enable)
    {
       sam_putreg(xxxx, SAM_CAN_IER_OFFSET);
#warning Missing logic
    }
  else
    {
       sam_putreg(xxxx, SAM_CAN_IDR_OFFSET);
#warning Missing logic
    }
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

static int can_ioctl(FAR struct can_dev_s *dev, int cmd, unsigned long arg)
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

static int can_remoterequest(FAR struct can_dev_s *dev, uint16_t id)
{
#warning "Remote request not implemented"
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
 *                 Bit 4:    Remote Tranmission Request (RTR)
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

static int can_send(FAR struct can_dev_s *dev, FAR struct can_msg_s *msg)
{
  FAR struct sam_can_s *priv = dev->cd_priv;
  FAR uint8_t *ptr;
  uint32_t regval;
  uint32_t tmp;
  int dlc;
  int txmb;

  canllvdbg("CAN%d ID: %d DLC: %d\n", priv->port, msg->cm_hdr.ch_id, msg->cm_hdr.ch_dlc);

  /* Select one empty transmit mailbox */

#warning Missing logic
    {
      canlldbg("ERROR: No available mailbox\n");
      return -EBUSY;
    }

  /* Set up the ID, standard 11-bit or extended 29-bit. */

#ifdef CONFIG_CAN_EXTID
  if (msg->cm_hdr.ch_extid)
    {
      DEBUGASSERT(msg->cm_hdr.ch_id < (1 << 29));
#warning Missing logic
    }
  else
    {
      DEBUGASSERT(msg->cm_hdr.ch_id < (1 << 11));
#warning Missing logic
    }
#else
#warning Missing logic
#endif

  /* Set up the DLC */

  dlc     = msg->cm_hdr.ch_dlc;
#warning Missing logic

  /* Set up the data fields */

  ptr    = msg->cm_data;
  regval = 0;

  if (dlc > 0)
    {
#warning Missing logic

      if (dlc > 1)
       {
#warning Missing logic

         if (dlc > 2)
           {
#warning Missing logic

             if (dlc > 3)
               {
#warning Missing logic
               }
           }
       }
    }

  regval = 0;
  if (dlc > 4)
    {
#warning Missing logic

      if (dlc > 5)
       {
#warning Missing logic

         if (dlc > 6)
           {
#warning Missing logic

             if (dlc > 7)
               {
#warning Missing logic
               }
           }
       }
    }

  /* Enable the transmit mailbox empty interrupt (may already be enabled) */
#warning Missing logic

  /* Request transmission */
#warning Missing logic

  can_dumpmbregs(priv, "After send");
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

static bool can_txready(FAR struct can_dev_s *dev)
{
  FAR struct sam_can_s *priv = dev->cd_priv;

  /* Return true if any mailbox is available */
#warning Missing logic

  return false;
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

static bool can_txempty(FAR struct can_dev_s *dev)
{
  FAR struct sam_can_s *priv = dev->cd_priv;

  /* Return true if all mailboxes are available */
#warning Missing logic

  return false;
}

/****************************************************************************
 * Name: can_interrupt
 *
 * Description:
 *   Common CAN interrupt handler
 *
 * Input Parameters:
 *   priv - CAN-specific private data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void can_interrupt(struct sam_can_s *priv)
{
  uint32_t sr;
  uint32_t imr;
  uint32_t pending;
  uint32_t regval;

  /* Get the set of pending interrupts */

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
#warning Missing logic
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
   *    generated when the internal timer rolls over.
   * - Timestamp interrupt: This interrupt is generated after the reception
   *   or the transmission of a start of frame or an end of frame. The value
   *   of the internal counter is copied in the CAN_TIMESTP register.
   */

  if ((pending & ~CAN_INT_MBALL) != 0)
    {
#warning Missing logic
    }
}

/****************************************************************************
 * Name: can0_interrupt
 *
 * Description:
 *   CAN0 interrupt handler
 *
 * Input Parameters:
 *   irq - The IRQ number of the interrupt.
 *   context - The register state save array at the time of the interrupt.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_CAN0
static int can0_interrupt(int irq, void *context)
{
  can_interrupt(&g_can0dev);
  return OK;
}
#endif

/****************************************************************************
 * Name: can0_interrupt
 *
 * Description:
 *   CAN0 interrupt handler
 *
 * Input Parameters:
 *   irq - The IRQ number of the interrupt.
 *   context - The register state save array at the time of the interrupt.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_CAN1
static int can1_interrupt(int irq, void *context)
{
  can_interrupt(&g_can1dev);
  return OK;
}
#endif

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
 *   2. Propogation segment (PROP_SEG):  This part of the bit time is used
 *      to compensate for the physical delay times within the network. It is
 *      twice the sum of the signal’s propagation time on the bus line, the
 *      input comparator delay, and the output driver delay. It is
 *      programmable to be 1 to 8 Tq long.  This parameter is defined in the
 *      PROPAG field of the CAN Baudrate Register.
 *   3. Phase segment 1 (PHASE_SEG1): defines the location of the sample
 *      point.  Phase Segment 1 is programmable to be 1-8 Tq long.
 *   4. Phase segement 2 (PHASE_SEG2):  defines the location of the transmit
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
 * Input Parameter:
 *   priv - A reference to the CAN block status
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int can_bittiming(struct sam_can_s *priv)
{
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

  if (priv->baud >= 1000)
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
   *   Tbit = Nquanta * (BRP + 1) / CAN_FREQUENCY
   *   baud = CAN_FREQUENCY / (Nquanta * (brp + 1))
   *   brp  = CAN_FREQUENCY / (baud * nquanta) - 1
   */

  brp = (CAN_FREQUENCY / (priv->baud * 1000 * tq)) - 1;
  if (brp == 0)
    {
      /* The BRP field must be within the range 1 - 0x7f */

      candbg("CAN%d: baud %d too high\n", priv->port, priv->baud);
      return -EINVAL;
    }

  /* Propagation delay:
   *
   *   Delay Bus Driver     - 50ns
   *   Delay Receiver       - 30ns
   *   Delay Bus Line (20m) - 110ns
   */

  propag = tq * priv->baud * 2 * (50 + 30 + 110) / 1000000
  if ((propag >= 1)
    {
      propag--;
    }
  else
    {
      propag = 0;
    }

  /* This the time of the first two segments */

  t1t2 = tq - 1 - (propag + 1);

  /* Calcuate phase1 and phase2 */

  phase1 = (t1t2 >> 1) - 1;
  phase2 = tphase1;

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
      candbg("CAN%d: Could not realize baud %d\n", priv->port, priv->baud);
      return -EINVAL;
    }

  regval = CAN_BR_PHASE2(phase2) | CAN_BR_PHASE1(phase1) |
           CAN_BR_PROPAG(propag) | CAN_BR_SJW(sjw) | CAN_BR_BRP(brp) |
           CAN_BR_ONCE;
  can_putreg(priv, SAM_CAN_BR_OFFSET);
}

/****************************************************************************
 * Name: can_hwinitialize
 *
 * Description:
 *   CAN cell initialization
 *
 * Input Parameter:
 *   priv - A pointer to the private data structure for this CAN block
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int can_hwinitialize(struct sam_can_s *priv)
{
  volatile uint32_t timeout;
  uint32_t regval;
  int ret;

  canllvdbg("CAN%d\n", priv->port);

  /* Configure CAN pins */

  sam_configpio(priv->rxpinset);
  sam_configpio(priv->txpinset);

  /* Set the maximum CAN peripheral clock frequency */

  regval = PMC_PCR_PID(priv->pid) | PMC_PCR_CMD | CAN_PCR_DIV | PMC_PCR_EN;
  sam_adc_putreg(priv, SAM_PMC_PCR, regval);

  /* Enable peripheral clocking */

  sam_enableperiph1(priv->pid);

    /* Exit from sleep mode */
#warning Missing Logic

  /* Configure CAN behavior.  Priority driven request order, not message ID. */
#warning Missing Logic

  /* Enter initialization mode */
#warning Missing Logic

  /* Wait until initialization mode is acknowledged */

  for (timeout = INAK_TIMEOUT; timeout > 0; timeout--)
    {
#warning Missing Logic
    }

  /* Check for a timeout */
#warning Missing Logic

  /* Disable the following modes:
   *
   *  - Time triggered communication mode
   *  - Automatic bus-off management
   *  - Automatic wake-up mode
   *  - No automatic retransmission
   *  - Receive FIFO locked mode
   *  - Transmit FIFO priority
   */
#warning Missing Logic

  /* Configure bit timing. */

  ret = can_bittiming(priv);
  if (ret < 0)
    {
      canlldbg("ERROR: Failed to set bit timing: %d\n", ret);
      return ret;
    }

  /* Exit initialization mode */
#warning Missing Logic

  /* Wait until the initialization mode exit is acknowledged */

  for (timeout = INAK_TIMEOUT; timeout > 0; timeout--)
    {
#warning Missing Logic
    }

  /* Check for a timeout */
#warning Missing Logic

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
 * Input Parameter:
 *   Port number (for hardware that has mutiple CAN interfaces)
 *
 * Returned Value:
 *   Valid CAN device structure reference on succcess; a NULL on failure
 *
 ****************************************************************************/

FAR struct can_dev_s *sam_caninitialize(int port)
{
  struct can_dev_s *dev = NULL;

  canvdbg("CAN%d\n", port);

  /* NOTE:  Peripherical clocking for CAN0 and/or CAN1 was already provided
   * by sam_clockconfig() early in the reset sequence.
   */

#ifdef CONFIG_SAMA5_CAN0
  if (port == 0)
    {
      /* Select the CAN0 device structure */

      dev = &g_can0dev;
    }
  else
#endif
#ifdef CONFIG_SAMA5_CAN1
  if (port == 1)
    {
      /* Select the CAN1 device structure */

      dev = &g_can1dev;
    }
  else
#endif
    {
      candbg("ERROR: Unsupported port %d\n", port);
      return NULL;
    }

  return dev;
}

#endif /* CONFIG_CAN && (CONFIG_SAMA5_CAN0 || CONFIG_SAMA5_CAN1) */

