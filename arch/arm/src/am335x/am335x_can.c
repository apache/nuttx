/****************************************************************************
 * arch/arm/src/am335x/am335x_can.c
 *
 *   Copyright (C) 2019 Petro Karashchenko. All rights reserved.
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Petro Karashchenko <petro.karashchenko@gmail.com>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/can/can.h>

#include "arm_arch.h"
#include "hardware/am335x_pinmux.h"
#include "hardware/am335x_prcm.h"
#include "hardware/am335x_dcan.h"
#include "am335x_can.h"
#include "am335x_gpio.h"
#include "am335x_pinmux.h"
#include "am335x_sysclk.h"

#if defined(CONFIG_AM335X_CAN0) || defined(CONFIG_AM335X_CAN1)

#define CAN_MSG_OBJECTS_NUM    0x40
#define CAN_RX_OBJ_NUM         0x20
#define CAN_TX_OBJ_NUM         0x20

#define CAN_RX_OBJ_FIRST       0x01
#define CAN_RX_OBJ_LAST        (CAN_RX_OBJ_FIRST + CAN_RX_OBJ_NUM - 0x01)

#define CAN_TX_OBJ_FIRST       (CAN_RX_OBJ_LAST + 0x01)
#define CAN_TX_OBJ_LAST        (CAN_TX_OBJ_FIRST + CAN_TX_OBJ_NUM - 0x01)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifdef CONFIG_AM335X_CAN0

/* A CAN bit rate must be provided */

#  ifndef CONFIG_AM335X_CAN0_BAUD
#    define CONFIG_AM335X_CAN0_BAUD 1000000
#   endif
#endif

#ifdef CONFIG_AM335X_CAN1

/* A CAN bit rate must be provided */

#  ifndef CONFIG_AM335X_CAN1_BAUD
#    define CONFIG_AM335X_CAN1_BAUD 1000000
#  endif
#endif

/* User-defined TSEG1 and TSEG2 settings may be used.
 *
 * CONFIG_AM335X_CAN_TSEG1 = the number of CAN time quanta in segment 1
 * CONFIG_AM335X_CAN_TSEG2 = the number of CAN time quanta in segment 2
 * CAN_BIT_QUANTA   = The number of CAN time quanta in on bit time
 */

#ifndef CONFIG_AM335X_CAN_TSEG1
#  define CONFIG_AM335X_CAN_TSEG1 6
#endif

#ifndef CONFIG_AM335X_CAN_TSEG2
#  define CONFIG_AM335X_CAN_TSEG2 1
#endif

#define CAN_BIT_QUANTA (CONFIG_AM335X_CAN_TSEG1 + CONFIG_AM335X_CAN_TSEG2 + 1)

/* Timing *******************************************************************/

/* CAN clock source */

#define CAN_CLOCK_FREQUENCY (am335x_get_sysclk())

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  uint8_t port; /* CAN port number */
  uint32_t baud; /* Configured baud */
  uint32_t base; /* CAN register base address */
  uint8_t irq; /* IRQ associated with this CAN */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* CAN Register access */

#ifdef CONFIG_AM335X_CAN_REGDEBUG
static void can_printreg(uint32_t addr, uint32_t value);
#endif

static uint32_t can_getreg(struct up_dev_s *priv, int offset);
static void can_putreg(struct up_dev_s *priv, int offset, uint32_t value);

/* CAN methods */

static void can_reset(FAR struct can_dev_s *dev);
static int can_setup(FAR struct can_dev_s *dev);
static void can_shutdown(FAR struct can_dev_s *dev);
static void can_rxint(FAR struct can_dev_s *dev, bool enable);
static void can_txint(FAR struct can_dev_s *dev, bool enable);
static int can_ioctl(FAR struct can_dev_s *dev, int cmd, unsigned long arg);
static int can_remoterequest(FAR struct can_dev_s *dev, uint16_t id);
static int can_send(FAR struct can_dev_s *dev, FAR struct can_msg_s *msg);
static bool candev_txready(FAR struct can_dev_s *dev);
static bool candev_txempty(FAR struct can_dev_s *dev);

/* CAN interrupts */

static int can_interrupt(int irq, FAR void *context, FAR void *arg);

/* Message Processing */

static void can_savemsg(struct up_dev_s *priv, struct can_hdr_s *hdr,
                        uint32_t *data);
static void can_readobj(struct up_dev_s *priv, uint32_t index);
static void can_invalobj(struct up_dev_s *priv, uint32_t index);
static void can_setuprxobj(struct up_dev_s *priv);

/* Initialization */

static int can_bittiming(struct up_dev_s *priv);

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
  .co_txready       = candev_txready,
  .co_txempty       = candev_txempty,
};

#ifdef CONFIG_AM335X_CAN0
static struct up_dev_s g_can0priv =
{
  .port   = 0,
  .baud   = CONFIG_AM335X_CAN0_BAUD,
  .base   = AM335X_DCAN0_VADDR,
  .irq    = AM335X_IRQ_DCAN0_LINE0,
};

static struct can_dev_s g_can0dev =
{
  .cd_ops  = &g_canops,
  .cd_priv = &g_can0priv,
};
#endif

#ifdef CONFIG_AM335X_CAN1
static struct up_dev_s g_can1priv =
{
  .port   = 1,
  .baud   = CONFIG_AM335X_CAN1_BAUD,
  .base   = AM335X_DCAN1_VADDR,
  .irq    = AM335X_IRQ_DCAN1_LINE0,
};

static struct can_dev_s g_can1dev =
{
  .cd_ops  = &g_canops,
  .cd_priv = &g_can1priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: can_printreg
 *
 * Description:
 *   Print the value read from a register.
 *
 * Input Parameters:
 *   addr - The register address
 *   value - The register value
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_AM335X_CAN_REGDEBUG
static void can_printreg(uint32_t addr, uint32_t value)
{
  static uint32_t prevaddr = 0;
  static uint32_t preval = 0;
  static uint32_t count = 0;

  /* Is this the same value that we read from the same register last time?
   * Are we polling the register?  If so, suppress some of the output.
   */

  if (addr == prevaddr && value == preval)
    {
      if (count == 0xffffffff || ++count > 3)
        {
          if (count == 4)
            {
              caninfo("...\n");
            }

          return;
        }
    }

  /* No this is a new address or value */

  else
    {
      /* Did we print "..." for the previous value? */

      if (count > 3)
        {
          /* Yes.. then show how many times the value repeated */

          caninfo("[repeats %d more times]\n", count - 3);
        }

      /* Save the new address, value, and count */

      prevaddr = addr;
      preval = value;
      count = 1;
    }

  /* Show the register value read */

  caninfo("%08x->%08x\n", addr, value);
}
#endif /* CONFIG_AM335X_CAN_REGDEBUG */

/****************************************************************************
 * Name: can_getreg
 *
 * Description:
 *   Read the value of an CAN1/2 register.
 *
 * Input Parameters:
 *   priv - A reference to the CAN block status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *
 ****************************************************************************/

#ifdef CONFIG_AM335X_CAN_REGDEBUG
static uint32_t can_getreg(struct up_dev_s *priv, int offset)
{
  uint32_t addr;
  uint32_t value;

  /* Read the value from the register */

  addr = priv->base + offset;
  value = getreg32(addr);
  can_printreg(addr, value);
  return value;
}
#else
static uint32_t can_getreg(struct up_dev_s *priv, int offset)
{
  return getreg32(priv->base + offset);
}
#endif /* CONFIG_AM335X_CAN_REGDEBUG */

/****************************************************************************
 * Name: can_putreg
 *
 * Description:
 *   Set the value of an CAN1/2 register.
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

#ifdef CONFIG_AM335X_CAN_REGDEBUG
static void can_putreg(struct up_dev_s *priv, int offset, uint32_t value)
{
  uint32_t addr = priv->base + offset;

  /* Show the register value being written */

  caninfo("%08x<-%08x\n", addr, value);

  /* Write the value */

  putreg32(value, addr);
}
#else
static void can_putreg(struct up_dev_s *priv, int offset, uint32_t value)
{
  putreg32(value, priv->base + offset);
}
#endif /* CONFIG_AM335X_CAN_REGDEBUG */

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
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->cd_priv;

  caninfo("CAN%d reset\n", priv->port);

  can_putreg(priv, AM335X_DCAN_CTL_OFFSET, DCAN_CTL_INIT | DCAN_CTL_SWR);

  while (can_getreg(priv, AM335X_DCAN_CTL_OFFSET) & DCAN_CTL_SWR)
    {
    }
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
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *) dev->cd_priv;
  uint32_t regval;
  uint8_t i;
  int ret = ERROR;

  caninfo("CAN%d setup\n", priv->port);

  can_bittiming(priv);

  /* Invalidate all message objects */

  for (i = 1; i <= CAN_MSG_OBJECTS_NUM; ++i)
    {
      can_invalobj(priv, i);
    }

  /* Initialization finished, normal operation now. */

  regval  = can_getreg(priv, AM335X_DCAN_CTL_OFFSET);
  regval &= ~DCAN_CTL_INIT;
  can_putreg(priv, AM335X_DCAN_CTL_OFFSET, regval);

  while (can_getreg(priv, AM335X_DCAN_CTL_OFFSET) & DCAN_CTL_INIT)
    {
    }

  ret = irq_attach(priv->irq, can_interrupt, (FAR void *)dev);
  if (ret == OK)
    {
      up_enable_irq(priv->irq);

      /* Enable CAN interrupts within CAN module */

      regval = can_getreg(priv, AM335X_DCAN_CTL_OFFSET);
      regval |= (DCAN_CTL_IE0 | DCAN_CTL_SIE | DCAN_CTL_EIE);
      can_putreg(priv, AM335X_DCAN_CTL_OFFSET, regval);
    }

  return ret;
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
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *) dev->cd_priv;
  uint32_t regval;

  caninfo("CAN%d\n", priv->port);

  /* Stop operation mode */

  regval = can_getreg(priv, AM335X_DCAN_CTL_OFFSET);
  regval |= DCAN_CTL_INIT;
  can_putreg(priv, AM335X_DCAN_CTL_OFFSET, regval);

  /* Disable CAN interrupts */

  regval = can_getreg(priv, AM335X_DCAN_CTL_OFFSET);
  regval &= ~(DCAN_CTL_IE0 | DCAN_CTL_IE1 | DCAN_CTL_SIE | DCAN_CTL_EIE);
  can_putreg(priv, AM335X_DCAN_CTL_OFFSET, regval);

  up_disable_irq(priv->irq);

  /* Then detach the CAN interrupt handler. */

  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: can_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts.
 *   This function is called two times: from can_open and can_close. Therefore
 *   this function enables and disables not only RX interrupts but all message
 *   objects.
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
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *) dev->cd_priv;

  if (enable == true)
    {
      can_setuprxobj(priv);
    }
  else
    {
      uint8_t i = 0;
      for (i = CAN_RX_OBJ_FIRST; i <= CAN_RX_OBJ_LAST; ++i)
        {
          can_invalobj(priv, i);
        }
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
  /* The TX interrupt is automatically enabled in can_send within a
   * message object.
   */
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
  caninfo("Fix me:Not Implemented\n");
  return 0;
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
  caninfo("Fix me:Not Implemented\n");
  return 0;
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

static int can_send(FAR struct can_dev_s *dev, FAR struct can_msg_s *msg)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *) dev->cd_priv;
  uint32_t regval;
  uint32_t num;
  uint32_t id;
  uint32_t dlc;
  uint8_t txobj;

  if (msg == NULL)
    {
      return ERROR;
    }

  regval = can_getreg(priv, AM335X_DCAN_TXRQ34_OFFSET);
  for (num = 0; num < 32; num++)
    {
      if ((regval & (1 << num)) == 0)
        {
          break;
        }
    }

  if (num == 32)
    {
      return ERROR;
    }

  txobj = CAN_TX_OBJ_LAST - num + CAN_TX_OBJ_FIRST;
  DEBUGASSERT((txobj >= CAN_TX_OBJ_FIRST) && (txobj <= CAN_TX_OBJ_LAST));

  id = (uint32_t) msg->cm_hdr.ch_id;
  dlc = (uint32_t) msg->cm_hdr.ch_dlc;

  caninfo("CAN%d ID: %d DLC: %d\n",
          priv->port, msg->cm_hdr.ch_id, msg->cm_hdr.ch_dlc);

  can_putreg(priv, AM335X_DCAN_IF1MSK_OFFSET, 0xffff);

  regval = ((dlc & DCAN_IFMCTL_DLC_MASK) | DCAN_IFMCTL_EOB | DCAN_IFMCTL_TX_RQST
                      | DCAN_IFMCTL_TX_IE);
  can_putreg(priv, AM335X_DCAN_IF1MCTL_OFFSET, regval);

  /* Write data to IF1 data registers */

  regval = msg->cm_data[0] + (msg->cm_data[1] << 8) +
           (msg->cm_data[2] << 16) + (msg->cm_data[3] << 24);
  can_putreg(priv, AM335X_DCAN_IF1DATA_OFFSET, regval);

  regval = msg->cm_data[4] + (msg->cm_data[5] << 8) +
           (msg->cm_data[6] << 16) + (msg->cm_data[7] << 24);
  can_putreg(priv, AM335X_DCAN_IF1DATB_OFFSET, regval);

#ifdef CONFIG_CAN_EXTID
  can_putreg(priv, AM335X_DCAN_IF1ARB_OFFSET, DCAN_IFARB_DIR | DCAN_IFARB_MSG_VAL
                   | DCAN_IFARB_XTD | (id << DCAN_IFARB_ID_SHIFT));
#else
  can_putreg(priv, AM335X_DCAN_IF1ARB_OFFSET, DCAN_IFARB_DIR | DCAN_IFARB_MSG_VAL
                   | (id << DCAN_IFARB_ID_SHIFT));
#endif

  /* Write to Message RAM */

  regval = (DCAN_IFCMD_WR_RD | DCAN_IFCMD_MASK | DCAN_IFCMD_ARB | DCAN_IFCMD_CTL
                   | DCAN_IFCMD_CLR_INTPND | DCAN_IFCMD_TX_RQST_NEWDAT
                   | DCAN_IFCMD_DATAA | DCAN_IFCMD_DATAB | DCAN_IFCMD_MSG_NUM(txobj));
  can_putreg(priv, AM335X_DCAN_IF1CMD_OFFSET, regval);

#ifdef CONFIG_CAN_TXREADY
  can_txdone(dev);
#endif

  return OK;
}

/****************************************************************************
 * Name: candev_txready
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

static bool candev_txready(FAR struct can_dev_s *dev)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *) dev->cd_priv;

  if (can_getreg(priv, AM335X_DCAN_TXRQ34_OFFSET) != 0xffffffff)
    {
      return false;
    }

  if (can_getreg(priv, AM335X_DCAN_IF1CMD_OFFSET) & DCAN_IFCMD_BUSY)
    {
      return false;
    }

  return true;
}

/****************************************************************************
 * Name: candev_txempty
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

static bool candev_txempty(FAR struct can_dev_s *dev)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *) dev->cd_priv;
  return (!((can_getreg(priv, AM335X_DCAN_TXRQ_X_OFFSET)) & 0xffff));
}

/****************************************************************************
 * Name: can_interrupt
 *
 * Description:
 *   CAN interrupt handler. There is a single interrupt for both CAN0 and
 *   CAN1.
 *
 * Input Parameters:
 *   irq - The IRQ number of the interrupt.
 *   context - The register state save array at the time of the interrupt.
 *   arg - pointer to up_dev_s
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int can_interrupt(int irq, FAR void *context, FAR void *arg)
{
  struct can_dev_s *dev = (FAR struct can_dev_s *)arg;
  FAR struct up_dev_s *priv;
  uint32_t regval = 0;

  DEBUGASSERT(dev != NULL && dev->cd_priv != NULL);
  priv = (struct up_dev_s *)dev->cd_priv;

  uint32_t msgindex = 0;

  /* Read CAN interrupt register */

  uint32_t interrupt = can_getreg(priv, AM335X_DCAN_INT_OFFSET) & DCAN_INT_LINE0_MASK;

  /* Read CAN status register */

  uint32_t stat = can_getreg(priv, AM335X_DCAN_ES_OFFSET);

  if (interrupt & DCAN_INT_LINE0_STATUS)
    {
      /* Clear all warning/error states except RXOK/TXOK */

      regval = can_getreg(priv, AM335X_DCAN_ES_OFFSET);
      regval &= DCAN_ES_RX_OK | DCAN_ES_TX_OK;
      can_putreg(priv, AM335X_DCAN_ES_OFFSET, regval);
    }
  else
    {
      msgindex = interrupt & 0x7fff;

      /* if no error detected */

      if (((stat & DCAN_ES_LEC_MASK) == 0) ||
          ((stat & DCAN_ES_LEC_MASK) == DCAN_ES_LEC_MASK))
        {
          if (msgindex <= CAN_RX_OBJ_LAST)
            {
              struct can_hdr_s hdr;
              uint32_t data[2];

              regval = can_getreg(priv, AM335X_DCAN_ES_OFFSET);
              regval &= ~DCAN_ES_RX_OK;
              can_putreg(priv, AM335X_DCAN_ES_OFFSET, regval);

              can_readobj(priv, msgindex);
              can_savemsg(priv, &hdr, data);
              can_invalobj(priv, msgindex);
              can_receive(dev, &hdr, (uint8_t *)data);
            }
          else
            {
              regval = can_getreg(priv, AM335X_DCAN_ES_OFFSET);
              regval &= ~DCAN_ES_TX_OK;
              can_putreg(priv, AM335X_DCAN_ES_OFFSET, regval);

              can_invalobj(priv, msgindex);
#ifdef CONFIG_CAN_TXREADY
              can_txready(dev);
#else
              can_txdone(dev);
#endif
            }
        }
      else
        {
          can_invalobj(priv, msgindex);
        }

      can_putreg(priv, AM335X_DCAN_ES_OFFSET, 0);

      if (msgindex == CAN_RX_OBJ_LAST)
        {
          can_setuprxobj(priv);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: can_savemsg
 *
 * Description:
 *   Save received message.
 *
 * Input Parameters:
 *   priv - A reference to the CAN block status
 *   hdr - A reference to the message header
 *   data - A reference to the data block
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void can_savemsg(struct up_dev_s *priv, struct can_hdr_s *hdr,
                        uint32_t *data)
{
  uint32_t regval;

  regval        = can_getreg(priv, AM335X_DCAN_IF2ARB_OFFSET);
  hdr->ch_id    = (regval >> DCAN_IFARB_ID_SHIFT) & DCAN_IFARB_ID_MASK;
  hdr->ch_rtr   = ((regval & DCAN_IFARB_DIR) != 0);
#ifdef CONFIG_CAN_EXTID
  hdr->ch_extid = ((regval & DCAN_IFARB_XTD) != 0);
#endif

  regval = can_getreg(priv, AM335X_DCAN_IF2MCTL_OFFSET);
  hdr->ch_dlc   = (regval >> DCAN_IFMCTL_DLC_SHIFT) & DCAN_IFMCTL_DLC_MASK;

  data[0]       = can_getreg(priv, AM335X_DCAN_IF2DATA_OFFSET);
  data[1]       = can_getreg(priv, AM335X_DCAN_IF2DATB_OFFSET);
}

/****************************************************************************
 * Name: can_readobj
 *
 * Description:
 *  Transfer Message Object into IF registers.
 *
 * Input Parameters:
 *  priv - A reference to the CAN block status
 *  index - Message Object number
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void can_readobj(struct up_dev_s *priv, uint32_t index)
{
  while (can_getreg(priv, AM335X_DCAN_IF2CMD_OFFSET) & DCAN_IFCMD_BUSY);

  can_putreg(priv, AM335X_DCAN_IF2CMD_OFFSET, DCAN_IFCMD_MASK | DCAN_IFCMD_ARB |
             DCAN_IFCMD_CTL | DCAN_IFCMD_CLR_INTPND | DCAN_IFCMD_DATAA |
             DCAN_IFCMD_DATAB | DCAN_IFCMD_MSG_NUM(index));
}

/****************************************************************************
 * Name: can_invalobj
 *
 * Description:
 *  Invalidate Message Object.
 *
 * Input Parameters:
 *  priv - A reference to the CAN block status
 *  index - Message Object number
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void can_invalobj(struct up_dev_s *priv, uint32_t index)
{
  while (can_getreg(priv, AM335X_DCAN_IF2CMD_OFFSET) & DCAN_IFCMD_BUSY);
  can_putreg(priv, AM335X_DCAN_IF2ARB_OFFSET, 0);

  /* Disable reception and transmission interrupts, clear transmit request */

  can_putreg(priv, AM335X_DCAN_IF2MCTL_OFFSET, DCAN_IFMCTL_EOB);
  can_putreg(priv, AM335X_DCAN_IF2CMD_OFFSET, DCAN_IFCMD_WR_RD |
             DCAN_IFCMD_CLR_INTPND | DCAN_IFCMD_CTL | DCAN_IFCMD_ARB |
             DCAN_IFCMD_MSG_NUM(index));
}

/****************************************************************************
 * Name: can_setuprxobj
 *
 * Description:
 *  Setup Message Object as buffer for received messages.
 *
 * Input Parameters:
 *  priv - A reference to the CAN block status
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void can_setuprxobj(struct up_dev_s *priv)
{
  uint8_t i;

  while (can_getreg(priv, AM335X_DCAN_IF2CMD_OFFSET) & DCAN_IFCMD_BUSY);

  can_putreg(priv, AM335X_DCAN_IF2MSK_OFFSET, DCAN_IFMSK_MXTD | DCAN_IFMSK_MDIR);
  can_putreg(priv, AM335X_DCAN_IF2MCTL_OFFSET, DCAN_IFMCTL_DLC_MASK |
             DCAN_IFMCTL_EOB | DCAN_IFMCTL_RX_IE | DCAN_IFMCTL_UMASK);

#ifdef CONFIG_CAN_EXTID
  can_putreg(priv, AM335X_DCAN_IF2ARB_OFFSET, DCAN_IFARB_MSG_VAL | DCAN_IFARB_XTD);
#else
  can_putreg(priv, AM335X_DCAN_IF2ARB_OFFSET, DCAN_IFARB_MSG_VAL);
#endif

  for (i = CAN_RX_OBJ_FIRST; i <= CAN_RX_OBJ_LAST; ++i)
    {
      while (can_getreg(priv, AM335X_DCAN_IF2CMD_OFFSET) & DCAN_IFCMD_BUSY);
      can_putreg(priv, AM335X_DCAN_IF2CMD_OFFSET, DCAN_IFCMD_WR_RD | DCAN_IFCMD_MASK |
                 DCAN_IFCMD_ARB | DCAN_IFCMD_CTL | DCAN_IFCMD_CLR_INTPND |
                 DCAN_IFCMD_DATAA | DCAN_IFCMD_DATAB | DCAN_IFCMD_MSG_NUM(i));
    }
}

/****************************************************************************
 * Name: can_bittiming
 *
 * Description:
 *   Set the CAN bit timing register (BTR) based on the configured BAUD.
 *
 * The bit timing logic monitors the serial bus-line and performs sampling
 * and adjustment of the sample point by synchronizing on the start-bit edge
 * and resynchronizing on the following edges.
 *
 * Its operation may be explained simply by splitting nominal bit time into
 * three segments as follows:
 *
 * 1. Synchronization segment (SYNC_SEG): a bit change is expected to occur
 *    within this time segment. It has a fixed length of one time quantum
 *    (1 x tCAN).
 * 2. Bit segment 1 (BS1): defines the location of the sample point. It
 *    includes the PROP_SEG and PHASE_SEG1 of the CAN standard. Its duration
 *    is programmable between 1 and 16 time quanta but may be automatically
 *    lengthened to compensate for positive phase drifts due to differences
 *    in the frequency of the various nodes of the network.
 * 3. Bit segment 2 (BS2): defines the location of the transmit point. It
 *    represents the PHASE_SEG2 of the CAN standard. Its duration is
 *    programmable between 1 and 8 time quanta but may also be automatically
 *    shortened to compensate for negative phase drifts.
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
 *   Tq = brp * Tcan
 *
 * Where:
 *   Tcan is the period of the APB clock.
 *
 * Input Parameters:
 *   priv - A reference to the CAN block status
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int can_bittiming(struct up_dev_s *priv)
{
  uint32_t ts1 = CONFIG_AM335X_CAN_TSEG1;
  uint32_t ts2 = CONFIG_AM335X_CAN_TSEG2;
  uint32_t sjw = 1;
  uint32_t brp = CAN_CLOCK_FREQUENCY / (priv->baud * CAN_BIT_QUANTA);

  uint32_t regval;

  syslog(LOG_DEBUG, "CAN%d PCLK: %d baud: %d\n",
         priv->port, CAN_CLOCK_FREQUENCY, priv->baud);
  syslog(LOG_DEBUG, "TS1: %d TS2: %d BRP: %d SJW= %d\n", ts1, ts2, brp, sjw);

  /* Start configuring bit timing */

  regval = can_getreg(priv, AM335X_DCAN_CTL_OFFSET);
  regval |= DCAN_CTL_CCE;
  can_putreg(priv, AM335X_DCAN_CTL_OFFSET, regval);

  regval = (((brp - 1) << DCAN_BTR_BRP_SHIFT)
         | ((ts1 - 1) << DCAN_BTR_TSEG1_SHIFT)
         | ((ts2 - 1) << DCAN_BTR_TSEG2_SHIFT)
         | ((sjw - 1) << DCAN_BTR_SJW_SHIFT));

  syslog(LOG_DEBUG, "Setting CANxBTR= 0x%08x\n", regval);

  /* Set bit timing */

  can_putreg(priv, AM335X_DCAN_BTR_OFFSET, regval);

  /* Stop configuring bit timing */

  regval = can_getreg(priv, AM335X_DCAN_CTL_OFFSET);
  regval &= ~DCAN_CTL_CCE;
  can_putreg(priv, AM335X_DCAN_CTL_OFFSET, regval);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am335x_can_initialize
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

FAR struct can_dev_s *am335x_can_initialize(int port)
{
  FAR struct can_dev_s *candev;
  irqstate_t flags;

  syslog(LOG_DEBUG, "CAN%d\n", port);

  flags = enter_critical_section();

#ifdef CONFIG_AM335X_CAN0
  if (port == 0)
    {
      /* Configure CAN IO pins */

      am335x_gpio_config(GPIO_DCAN0_RX);
      am335x_gpio_config(GPIO_DCAN0_TX);

      candev = &g_can0dev;
    }
  else
#endif /* CONFIG_AM335X_CAN0 */

#ifdef CONFIG_AM335X_CAN1
  if (port == 1)
    {
      /* Configure CAN IO pins */

      am335x_gpio_config(GPIO_DCAN1_RX);
      am335x_gpio_config(GPIO_DCAN1_TX);

      candev = &g_can1dev;
    }
  else
#endif /* CONFIG_AM335X_CAN1 */
    {
      canerr("Unsupported port: %d\n", port);

      leave_critical_section(flags);
      return NULL;
    }

  leave_critical_section(flags);

  return candev;
}

void am335x_can_uninitialize(FAR struct can_dev_s *dev)
{
  irqstate_t flags;

  DEBUGASSERT(dev);

  flags = enter_critical_section();

#ifdef CONFIG_AM335X_CAN0
  if (dev == &g_can0dev)
    {
      /* Configure CAN IO pins to GPIO */

      am335x_gpio_config(am335x_periph_gpio(GPIO_DCAN0_RX));
      am335x_gpio_config(am335x_periph_gpio(GPIO_DCAN0_TX));
    }
  else
#endif /* CONFIG_AM335X_CAN0 */

#ifdef CONFIG_AM335X_CAN1
  if (dev == &g_can1dev)
    {
      /* Configure CAN IO pins to GPIO */

      am335x_gpio_config(am335x_periph_gpio(GPIO_DCAN1_RX));
      am335x_gpio_config(am335x_periph_gpio(GPIO_DCAN1_TX));
    }
  else
#endif /* CONFIG_AM335X_CAN1 */
    {
      canerr("Not a CAN device: %p\n", dev);
    }

  leave_critical_section(flags);
}

#endif
