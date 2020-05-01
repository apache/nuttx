/****************************************************************************
 * arch/arm/src/lpc43xx/lpc43_can.c
 *
 *   Copyright(C) 2017 Gregory Nutt. All rights reserved.
 *
 *    Created on: 2 May 2017
 *        Author: katherine
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
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/can/can.h>

#include "arm_internal.h"
#include "arm_arch.h"

#include "chip.h"
#include "lpc43_gpio.h"
#include "hardware/lpc43_can.h"
#include "hardware/lpc43_rgu.h"
#include "lpc43_ccu.h"
#include "lpc43_cgu.h"

#if defined(CONFIG_LPC43_CAN0) || defined(CONFIG_LPC43_CAN1)

#define CAN_MSG_OBJECTS_NUM    0x20
#define CAN_RX_OBJ_NUM         0x10
#define CAN_TX_OBJ_NUM         0x10

#define CAN_RX_OBJ_FIRST       0x01
#define CAN_RX_OBJ_LAST         (CAN_RX_OBJ_FIRST + CAN_RX_OBJ_NUM - 0x01)

#define CAN_TX_OBJ_FIRST       (CAN_RX_OBJ_LAST + 0x01)
#define CAN_TX_OBJ_LAST         (CAN_TX_OBJ_FIRST + CAN_TX_OBJ_NUM - 0x01)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifdef CONFIG_LPC43_CAN0

/* A CAN bit rate must be provided */

#  ifndef CONFIG_LPC43_CAN0_BAUD
#    define CONFIG_LPC43_CAN0_BAUD 1000000
#   endif
#endif

#ifdef CONFIG_LPC43_CAN1

/* A CAN bit rate must be provided */

#  ifndef CONFIG_LPC43_CAN1_BAUD
#    define CONFIG_LPC43_CAN1_BAUD 1000000
#  endif
#endif

/* User-defined TSEG1 and TSEG2 settings may be used.
 *
 * CONFIG_LPC43_CAN_TSEG1 = the number of CAN time quanta in segment 1
 * CONFIG_LPC43_CAN_TSEG2 = the number of CAN time quanta in segment 2
 * CAN_BIT_QUANTA   = The number of CAN time quanta in on bit time
 */

#ifndef CONFIG_LPC43_CAN_TSEG1
#  define CONFIG_LPC43_CAN_TSEG1 12
#endif

#ifndef CONFIG_LPC43_CAN_TSEG2
#  define CONFIG_LPC43_CAN_TSEG2 4
#endif

#define CAN_BIT_QUANTA (CONFIG_LPC43_CAN_TSEG1 + CONFIG_LPC43_CAN_TSEG2 + 1)

/* Timing *******************************************************************/

/* CAN clock source is defined in board.h */

#define CAN_CLOCK_FREQUENCY(c) ((uint32_t)LPC43_CCLK / ((uint32_t)(c)))

/* CAN module clock must be less then 50 MHz */

#define CAN_CLKDIVVAL  0x05

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  uint8_t port; /* CAN port number */
  uint8_t clkdiv; /* CLKDIV register */
  uint32_t baud; /* Configured baud */
  uint32_t base; /* CAN register base address */
  uint8_t irq; /* IRQ associated with this CAN */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* CAN Register access */

#ifdef CONFIG_LPC43_CAN_REGDEBUG
static void can_printreg(uint32_t addr, uint32_t value);
#endif

static uint32_t can_getreg(struct up_dev_s *priv, int offset);
static void can_putreg(struct up_dev_s *priv, int offset, uint32_t value);

#ifdef CONFIG_LPC43_CAN_REGDEBUG
static uint32_t can_getcommon(uint32_t addr);
static void can_putcommon(uint32_t addr, uint32_t value);
#else
#  define can_getcommon(addr)        getreg32(addr)
#  define can_putcommon(addr, value) putreg32(value, addr)
#endif

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

#ifdef CONFIG_LPC43_CAN0
static int can0_interrupt(int irq, void *context, FAR void *arg);
#endif
#ifdef CONFIG_LPC43_CAN1
static int can1_interrupt(int irq, void *context, FAR void *arg);
#endif
static void can_interrupt(FAR struct can_dev_s *dev);

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
  .co_setup         =  can_setup,
  .co_shutdown      = can_shutdown,
  .co_rxint         = can_rxint,
  .co_txint         = can_txint,
  .co_ioctl         = can_ioctl,
  .co_remoterequest = can_remoterequest,
  .co_send          = can_send,
  .co_txready       = candev_txready,
  .co_txempty       = candev_txempty,
};

#ifdef CONFIG_LPC43_CAN0
static struct up_dev_s g_can0priv =
{
  .port   = 0,
  .clkdiv = CAN_CLKDIVVAL + 1,
  .baud   = CONFIG_LPC43_CAN0_BAUD,
  .base   = LPC43_CAN0_BASE,
  .irq    = LPC43M4_IRQ_CAN0,
};

static struct can_dev_s g_can0dev =
{
  .cd_ops  = &g_canops,
  .cd_priv = &g_can0priv,
};
#endif

#ifdef CONFIG_LPC43_CAN1
static struct up_dev_s g_can1priv =
{
  .port   = 1,
  .clkdiv = CAN_CLKDIVVAL + 1,
  .baud   = CONFIG_LPC43_CAN1_BAUD,
  .base   = LPC43_CAN1_BASE,
  .irq    = LPC43M4_IRQ_CAN1,
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

#ifdef CONFIG_LPC43_CAN_REGDEBUG
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

          caninfo("[repeats %d more times]\n", count-3);
        }

      /* Save the new address, value, and count */

      prevaddr = addr;
      preval = value;
      count = 1;
    }

  /* Show the register value read */

  caninfo("%08x->%08x\n", addr, value);
}
#endif /* CONFIG_LPC43_CAN_REGDEBUG */

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

#ifdef CONFIG_LPC43_CAN_REGDEBUG
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
#endif /* CONFIG_LPC43_CAN_REGDEBUG */

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

#ifdef CONFIG_LPC43_CAN_REGDEBUG
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
#endif /* CONFIG_LPC43_CAN_REGDEBUG */

/****************************************************************************
 * Name: can_getcommon
 *
 * Description:
 *   Get the value of common register.
 *
 * Input Parameters:
 *   addr - The address of the register to read
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_LPC43_CAN_REGDEBUG
static uint32_t can_getcommon(uint32_t addr)
{
  uint32_t value;

  /* Read the value from the register */

  value = getreg32(addr);
  can_printreg(addr, value);
  return value;
}
#endif /* CONFIG_LPC43_CAN_REGDEBUG */

/****************************************************************************
 * Name: can_putcommon
 *
 * Description:
 *   Set the value of common register.
 *
 * Input Parameters:
 *   addr - The address of the register to write
 *   value - The value to write to the register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_LPC43_CAN_REGDEBUG
static void can_putcommon(uint32_t addr, uint32_t value)
{
  /* Show the register value being written */

  caninfo("%08x<-%08x\n", addr, value);

  /* Write the value */

  putreg32(value, addr);
}
#endif /* CONFIG_LPC43_CAN_REGDEBUG */

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
  uint32_t regval;

  caninfo("CAN%d reset\n", priv->port);

#ifdef CONFIG_LPC43_CAN0
  if (priv->port == 0)
    {
      regval = ~(getreg32(LPC43_RGU_ACTIVE1));
      regval |= RGU_CTRL1_CAN0_RST;
      putreg32(regval, LPC43_RGU_CTRL1);
    }
#endif /* CONFIG_LPC43_CAN0 */

#ifdef CONFIG_LPC43_CAN1
  if (priv->port == 1)
    {
      regval = ~(getreg32(LPC43_RGU_ACTIVE1));
      regval |= RGU_CTRL1_CAN1_RST;
      putreg32(regval, LPC43_RGU_CTRL1);
    }
#endif /* CONFIG_LPC43_CAN1 */
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

  /* Clock must be divided by the CAN clock divider CLKDIV to be less than
   * 50 MHz.
   */

  can_putreg(priv, LPC43_CAN_CLKDIV_OFFSET, CAN_CLKDIVVAL);

  /* Invalidate all message objects */

  for (i = 1; i <= CAN_MSG_OBJECTS_NUM; ++i)
    {
      can_invalobj(priv, i);
    }

  /* Initialization finished, normal operation now. */

  regval  = can_getreg(priv, LPC43_CAN_CNTL_OFFSET);
  regval &= ~CAN_CNTL_INIT;
  can_putreg(priv, LPC43_CAN_CNTL_OFFSET, regval);

  while (can_getreg(priv, LPC43_CAN_CNTL_OFFSET) & CAN_CNTL_INIT);

#ifdef CONFIG_LPC43_CAN0
  if (priv->irq == LPC43M4_IRQ_CAN0)
    {
      ret = irq_attach(priv->irq, can0_interrupt, 0);
    }
#endif

#ifdef CONFIG_LPC43_CAN1
  if (priv->irq == LPC43M4_IRQ_CAN1)
    {
      ret = irq_attach(priv->irq, can1_interrupt, 0);
    }
#endif

  if (ret == OK)
    {
      up_enable_irq(priv->irq);

      /* Enable CAN interrupts within CAN module */

      regval = can_getreg(priv, LPC43_CAN_CNTL_OFFSET);
      regval |= (CAN_CNTL_IE | CAN_CNTL_SIE | CAN_CNTL_EIE);
      can_putreg(priv, LPC43_CAN_CNTL_OFFSET, regval);
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

  regval = can_getreg(priv, LPC43_CAN_CNTL_OFFSET);
  regval |= CAN_CNTL_INIT;
  can_putreg(priv, LPC43_CAN_CNTL_OFFSET, regval);

  /* Disable CAN interrupts */

  regval = can_getreg(priv, LPC43_CAN_CNTL_OFFSET);
  regval &= ~(CAN_CNTL_IE | CAN_CNTL_SIE | CAN_CNTL_EIE);
  can_putreg(priv, LPC43_CAN_CNTL_OFFSET, regval);

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
  uint32_t clz;
  uint32_t id;
  uint32_t dlc;
  uint8_t txobj;

  if (msg == NULL)
    {
      return ERROR;
    }

  regval = (can_getreg(priv, LPC43_CAN_MSGV2_OFFSET) & 0xFFFF);
  clz    = arm_clz(regval);

  if (clz == 0x10)
    {
      return ERROR;
    }

  txobj = CAN_TX_OBJ_LAST - clz + CAN_TX_OBJ_FIRST;
  DEBUGASSERT((txobj >= CAN_TX_OBJ_FIRST) && (txobj <= CAN_TX_OBJ_LAST));

  id = (uint32_t) msg->cm_hdr.ch_id;
  dlc = (uint32_t) msg->cm_hdr.ch_dlc;

  caninfo("CAN%d ID: %d DLC: %d\n",
          priv->port, msg->cm_hdr.ch_id, msg->cm_hdr.ch_dlc);

  can_putreg(priv, LPC43_CAN_IF1_MSK1_OFFSET, 0xFFFF);
  can_putreg(priv, LPC43_CAN_IF1_MSK2_OFFSET, 0xFFFF);

  regval = ((dlc & CAN_MCTRL_DLC_MASK) | CAN_MCTRL_EOB | CAN_MCTRL_TXRQST
                      | CAN_MCTRL_TXIE );
  can_putreg(priv, LPC43_CAN_IF1_MCTRL_OFFSET, regval);

  /* Write data to IF1 data registers */

  regval = msg->cm_data[0] + (msg->cm_data[1] << 8);
  can_putreg(priv, LPC43_CAN_IF1_DA1_OFFSET, regval);

  regval = msg->cm_data[2] + (msg->cm_data[3] << 8);
  can_putreg(priv, LPC43_CAN_IF1_DA2_OFFSET, regval);

  regval = msg->cm_data[4] + (msg->cm_data[5] << 8);
  can_putreg(priv, LPC43_CAN_IF1_DB1_OFFSET, regval);

  regval = msg->cm_data[6] + (msg->cm_data[7] << 8);
  can_putreg(priv, LPC43_CAN_IF1_DB2_OFFSET, regval);

#ifdef CONFIG_CAN_EXTID
  can_putreg(priv, LPC43_CAN_IF1_ARB1_OFFSET, id & 0x0000FFFF);
  can_putreg(priv, LPC43_CAN_IF1_ARB2_OFFSET, CAN_MSK2_DIR | CAN_MSK2_XTD
                   | CAN_MSK2_MSGVAL | id >> 16);
#else
  can_putreg(priv, LPC43_CAN_IF1_ARB1_OFFSET, 0x0000);
  can_putreg(priv, LPC43_CAN_IF1_ARB2_OFFSET, CAN_MSK2_DIR | CAN_MSK2_MSGVAL
                   | id << 2);
#endif

  regval = (CAN_CMDMSKW_WRRD | CAN_CMDMSKW_MASK | CAN_CMDMSKW_ARB
                       | CAN_CMDMSKW_CTRL | CAN_CMDMSKW_CLRINTPND | CAN_CMDMSKW_TXRQST
                       | CAN_CMDMSKW_DATAA | CAN_CMDMSKW_DATAB);
  can_putreg(priv, LPC43_CAN_IF1_CMDMSKW_OFFSET, regval);

  /* Write to Message RAM */

  can_putreg(priv, LPC43_CAN_IF1_CMDREQ_OFFSET, txobj);

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

  if (can_getreg(priv, LPC43_CAN_MSGV2_OFFSET) & 0x8000)
    {
      return false;
    }

  if (can_getreg(priv, LPC43_CAN_IF1_CMDREQ_OFFSET) & CAN_CMDREQ_BUSY)
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
  return (!((can_getreg(priv, LPC43_CAN_MSGV2_OFFSET)) & 0xFFFF));
}

/****************************************************************************
 * Name: can0/1_interrupt
 *
 * Description:
 *   CAN interrupt handler for CAN0 and CAN1.
 *
 * Input Parameters:
 *   irq - The IRQ number of the interrupt.
 *   context - The register state save array at the time of the interrupt.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/


#ifdef CONFIG_LPC43_CAN0
static int can0_interrupt(int irq, void *context, FAR void *arg)
{
  can_interrupt(&g_can0dev);
  return OK;
}
#endif

#ifdef CONFIG_LPC43_CAN1
static int can1_interrupt(int irq, void *context, FAR void *arg)
{
  can_interrupt(&g_can1dev);
  return OK;
}
#endif

/****************************************************************************
 * Name: can_interrupt
 *
 * Description:
 *   CAN interrupt handler.  There is a single interrupt for both CAN0 and
 *   CAN1.
 *
 ****************************************************************************/

static void can_interrupt(FAR struct can_dev_s *dev)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *) dev->cd_priv;
  uint32_t regval = 0;

  uint32_t msgindex = 0;

  /* Read CAN interrupt register */

  uint32_t interrupt = can_getreg(priv, LPC43_CAN_INT_OFFSET);

  /* Read CAN status register */

  uint32_t stat = can_getreg(priv, LPC43_CAN_STAT_OFFSET);

  if ( interrupt & CAN_INT_STAT )
    {
      /* Clear all warning/error states except RXOK/TXOK */

      regval = can_getreg(priv, LPC43_CAN_STAT_OFFSET);
      regval &= CAN_STAT_RXOK | CAN_STAT_TXOK;
      can_putreg(priv, LPC43_CAN_STAT_OFFSET, regval);
    }
  else
    {
      msgindex = interrupt & 0x7FFF;

      /* if no error detected */

      if (((stat & CAN_STAT_LEC_MASK) == 0) ||
          ((stat & CAN_STAT_LEC_MASK) == CAN_STAT_LEC_MASK))
        {
          if (msgindex <= CAN_RX_OBJ_LAST)
            {
              struct can_hdr_s hdr;
              uint32_t data[2];

              regval = can_getreg(priv, LPC43_CAN_STAT_OFFSET);
              regval &= ~CAN_STAT_RXOK;
              can_putreg(priv, LPC43_CAN_STAT_OFFSET, regval);

              can_readobj(priv, msgindex);
              can_savemsg(priv, &hdr, data);
              can_invalobj(priv, msgindex);
              can_receive(dev, &hdr, (uint8_t *)data);
            }
          else
            {
              regval = can_getreg(priv, LPC43_CAN_STAT_OFFSET);
              regval &= ~CAN_STAT_TXOK;
              can_putreg(priv, LPC43_CAN_STAT_OFFSET, regval);

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

      can_putreg(priv, LPC43_CAN_STAT_OFFSET, 0);

      if (msgindex == CAN_RX_OBJ_LAST)
        {
          can_setuprxobj(priv);
        }
    }
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
#ifdef CONFIG_CAN_EXTID
  hdr->ch_id    = (can_getreg(priv, LPC43_CAN_IF2_ARB1_OFFSET) & 0xFFFF) |
                  (can_getreg(priv, LPC43_CAN_IF2_ARB2_OFFSET) & 0x0FFF) << 16;
  hdr->ch_extid = 1;
#else
  hdr->ch_id    = (can_getreg(priv, LPC43_CAN_IF2_ARB2_OFFSET) & 0x1FFF) >> 2;
#endif
  hdr->ch_dlc   = can_getreg(priv, LPC43_CAN_IF2_MCTRL_OFFSET) & 0x000F;
  hdr->ch_rtr   = 0;

  data[0]       = can_getreg(priv, LPC43_CAN_IF2_DA2_OFFSET) << 16 |
                  can_getreg(priv, LPC43_CAN_IF2_DA1_OFFSET);
  data[1]       = can_getreg(priv, LPC43_CAN_IF2_DB2_OFFSET) << 16 |
                  can_getreg(priv, LPC43_CAN_IF2_DB1_OFFSET);
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
  uint32_t regval;

  while (can_getreg(priv, LPC43_CAN_IF2_CMDREQ_OFFSET) & CAN_CMDREQ_BUSY);

  regval = (CAN_CMDMSKR_MASK | CAN_CMDMSKR_ARB | CAN_CMDMSKR_CTRL |
            CAN_CMDMSKR_CLRINTPND | CAN_CMDMSKR_DATAA | CAN_CMDMSKR_DATAB);
  can_putreg(priv, LPC43_CAN_IF2_CMDMSKR_OFFSET, regval);
  can_putreg(priv, LPC43_CAN_IF2_CMDREQ_OFFSET, index);
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
  while (can_getreg(priv, LPC43_CAN_IF2_CMDREQ_OFFSET) & CAN_CMDREQ_BUSY);

  can_putreg(priv, LPC43_CAN_IF2_ARB1_OFFSET, 0x0000);
  can_putreg(priv, LPC43_CAN_IF2_ARB2_OFFSET, 0x0000);

  /* Disable reception and transmission interrupts, clear transmit request */

  can_putreg(priv, LPC43_CAN_IF2_MCTRL_OFFSET, CAN_MCTRL_EOB);
  can_putreg(priv, LPC43_CAN_IF2_CMDMSKW_OFFSET, CAN_CMDMSKW_WRRD
                       | CAN_CMDMSKW_CLRINTPND | CAN_CMDMSKW_CTRL | CAN_CMDMSKW_ARB);
  can_putreg(priv, LPC43_CAN_IF2_CMDREQ_OFFSET, index);
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
  uint32_t regval;
  uint8_t i;

  while (can_getreg(priv, LPC43_CAN_IF2_CMDREQ_OFFSET) & CAN_CMDREQ_BUSY);

  can_putreg(priv, LPC43_CAN_IF2_MSK1_OFFSET, 0x0000);
  regval = (CAN_MSK2_MXTD | CAN_MSK2_MDIR);
  can_putreg(priv, LPC43_CAN_IF2_MSK2_OFFSET, regval);

  regval = (CAN_MCTRL_DLC_MASK | CAN_MCTRL_EOB | CAN_MCTRL_RXIE |
            CAN_MCTRL_UMASK);
  can_putreg(priv, LPC43_CAN_IF2_MCTRL_OFFSET, regval);

  can_putreg(priv, LPC43_CAN_IF2_ARB1_OFFSET, 0x0000);
#ifdef CONFIG_CAN_EXTID
  can_putreg(priv, LPC43_CAN_IF2_ARB2_OFFSET, CAN_MSK2_MSGVAL | CAN_MSK2_XTD);
#else
  can_putreg(priv, LPC43_CAN_IF2_ARB2_OFFSET, CAN_MSK2_MSGVAL);
#endif

  regval = (CAN_CMDMSKR_WRRD | CAN_CMDMSKR_MASK | CAN_CMDMSKR_ARB |
            CAN_CMDMSKR_CTRL | CAN_CMDMSKR_CLRINTPND | CAN_CMDMSKR_DATAA |
            CAN_CMDMSKR_DATAB);
  can_putreg(priv, LPC43_CAN_IF2_CMDMSKR_OFFSET, regval);

  for (i = CAN_RX_OBJ_FIRST; i <= CAN_RX_OBJ_LAST; ++i)
    {
      while (can_getreg(priv, LPC43_CAN_IF2_CMDREQ_OFFSET) & CAN_CMDREQ_BUSY);
      can_putreg(priv, LPC43_CAN_IF2_CMDREQ_OFFSET, i);
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
  uint32_t ts1 = CONFIG_LPC43_CAN_TSEG1;
  uint32_t ts2 = CONFIG_LPC43_CAN_TSEG2;
  uint32_t sjw = 1;
  uint32_t brp = CAN_CLOCK_FREQUENCY(priv->clkdiv) /
                 (priv->baud * CAN_BIT_QUANTA);

  uint32_t regval;

  canllvdbg("CAN%d PCLK: %d baud: %d\n",
            priv->port, CAN_CLOCK_FREQUENCY(priv->clkdiv), priv->baud);
  canllvdbg("TS1: %d TS2: %d BRP: %d SJW= %d\n", ts1, ts2, brp, sjw);

  /* Start configuring bit timing */

  regval = can_getreg(priv, LPC43_CAN_CNTL_OFFSET);
  regval |= CAN_CNTL_CCE;
  can_putreg(priv, LPC43_CAN_CNTL_OFFSET, regval);

  regval = (((brp - 1) << CAN_BT_BRP_SHIFT)
                     | ((ts1 - 1) << CAN_BT_TSEG1_SHIFT)
                     | ((ts2 - 1) << CAN_BT_TSEG2_SHIFT)
                     | ((sjw - 1) << CAN_BT_SJW_SHIFT));

  canllvdbg("Setting CANxBTR= 0x%08x\n", regval);

  /* Set bit timing */

  can_putreg(priv, LPC43_CAN_BT_OFFSET, regval);

  /* Stop configuring bit timing */

  regval = can_getreg(priv, LPC43_CAN_CNTL_OFFSET);
  regval &= ~CAN_CNTL_CCE;
  can_putreg(priv, LPC43_CAN_CNTL_OFFSET, regval);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_caninitialize
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

FAR struct can_dev_s *lpc43_caninitialize(int port)
{
  FAR struct can_dev_s *candev;
  irqstate_t flags;
  uint32_t regval;

  canllvdbg("CAN%d\n", port);

  flags = enter_critical_section();

#ifdef CONFIG_LPC43_CAN0
  if (port == 0)
    {
      /* Enable clock */

      regval = getreg32(LPC43_CCU1_APB3_CAN0_CFG);
      regval |= CCU_CLK_CFG_RUN;
      putreg32(regval, LPC43_CCU1_APB3_CAN0_CFG);

      /* Configure CAN GPIO pins */

      lpc43_pin_config(PINCONF_CAN0_RD);
      lpc43_pin_config(PINCONF_CAN0_TD);

      candev = &g_can0dev;
    }
  else
#endif /* CONFIG_LPC43_CAN0 */

#ifdef CONFIG_LPC43_CAN1
  if (port == 1)
    {
      /* Enable clock */

      regval = getreg32(LPC43_CCU1_APB1_CAN1_CFG);
      regval |= CCU_CLK_CFG_RUN;
      putreg32(regval, LPC43_CCU1_APB1_CAN1_CFG);

      /* Configure CAN GPIO pins */

      lpc43_pin_config(PINCONF_CAN1_RD);
      lpc43_pin_config(PINCONF_CAN1_TD);

      candev = &g_can1dev;
    }
  else
#endif /* CONFIG_LPC43_CAN1 */
    {
      canerr("Unsupported port: %d\n", port);

      leave_critical_section(flags);
      return NULL;
    }

  leave_critical_section(flags);

  return candev;
}
#endif
