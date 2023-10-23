/****************************************************************************
 * arch/arm/src/samv7/sam_lin_sock.c
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
#include "hardware/sam_pinmap.h"
#include "hardware/sam_uart.h"
#include "sam_periphclks.h"
#include "sam_lin.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Delays *******************************************************************/

#ifndef CONFIG_DEBUG_CAN_INFO
#  undef CONFIG_SAMV7_LIN_REGDEBUG
#endif

/* Pool configuration *******************************************************/

#define POOL_SIZE (1)

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

#define LINWORK LPWORK

/* LIN error interrupts */

#define UART_INT_LINERR     (UART_INT_LINBE | UART_INT_LINISFE | \
                             UART_INT_LINIPE | UART_INT_LINCE | \
                             UART_INT_LINSNRE)

#define LIN_ID_MASK         0x0000003f

/* BAUD definitions
 *
 * The source clock is selectable and could be one of:
 *
 *   - The peripheral clock
 *   - A division of the peripheral clock, where the divider is product-
 *     dependent, but generally set to 8
 *   - A processor/peripheral independent clock source fully programmable
 *      provided by PMC (PCK)
 *   - The external clock, available on the SCK pin
 *
 * Only the first two options are supported by this driver.  The divided
 * peripheral clock is only used for very low BAUD selections.
 */

#define FAST_USART_CLOCK    BOARD_MCK_FREQUENCY
#define SLOW_USART_CLOCK    (BOARD_MCK_FREQUENCY >> 3)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sam_lin_s
{
  uint8_t  port;     /* LIN port number (1 or 2) */
  uint32_t base;     /* Base address of the USART registers */
  uint8_t  irq;      /* IRQ associated with this USART */
  uint32_t baud;     /* Configured baud */
  uint32_t sr;       /* Saved status bits */

  bool                bifup;  /* true:ifup false:ifdown */
  struct net_driver_s dev;    /* Interface understood by the network */

  struct work_s irqwork;  /* For deferring interrupt work to the wq */
  struct work_s pollwork; /* For deferring poll work to the work wq */

  /* A pointers to the list of TX/RX descriptors */

  struct can_frame *txdesc;
  struct can_frame *rxdesc;

  volatile uint8_t idx;     /* Data index */

  /* TX/RX pool */

  uint8_t tx_pool[sizeof(struct can_frame)*POOL_SIZE];
  uint8_t rx_pool[sizeof(struct can_frame)*POOL_SIZE];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* LIN Register access */

static uint32_t sam_lin_getreg(struct sam_lin_s *priv, int offset);
static void sam_lin_putreg(struct sam_lin_s *priv, int offset,
                           uint32_t value);

/* Common TX logic */

static int  sam_lin_transmit(struct sam_lin_s *priv);
static bool sam_lin_txready(struct sam_lin_s *priv);
static int  sam_lin_txpoll(struct net_driver_s *dev);

/* LIN RX interrupt handling */

static void sam_lin_rxinterrupt_work(void *arg);

/* LIN TX interrupt handling */

static void sam_lin_txdone_work(void *arg);
static void sam_lin_txdone(struct sam_lin_s *priv);

#ifdef CONFIG_NET_CAN_ERRORS
/* LIN errors interrupt handling */

static void sam_lin_errinterrupt_work(void *arg);
#endif

/* Initialization */

static int  sam_lin_setup(struct sam_lin_s *priv);
static void sam_lin_shutdown(struct sam_lin_s *priv);
static void sam_lin_reset(struct sam_lin_s *priv);

/* NuttX callback functions */

static int  sam_lin_ifup(struct net_driver_s *dev);
static int  sam_lin_ifdown(struct net_driver_s *dev);

static void sam_lin_txavail_work(void *arg);
static int  sam_lin_txavail(struct net_driver_s *dev);

#ifdef CONFIG_NETDEV_IOCTL
static int  sam_lin_netdev_ioctl(struct net_driver_s *dev, int cmd,
                                 unsigned long arg);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_SAMV7_USART0_LIN_SOCKET

static struct sam_lin_s g_lin0priv =
{
  .port          = 0,
  .base          = SAM_USART0_BASE,
  .irq           = SAM_IRQ_USART0,
  .baud          = CONFIG_SAMV7_USART0_LIN_BAUD,
  .dev           =
    {
      .d_ifname  = "lin%d",
      .d_ifup    = sam_lin_ifup,
      .d_ifdown  = sam_lin_ifdown,
      .d_txavail = sam_lin_txavail,
#  ifdef CONFIG_NETDEV_IOCTL
      .d_ioctl   = sam_lin_netdev_ioctl,
#  endif
    }
};

#endif

#ifdef CONFIG_SAMV7_USART1_LIN_SOCKET

static struct sam_lin_s g_lin1priv =
{
  .port          = 1,
  .base          = SAM_USART1_BASE,
  .irq           = SAM_IRQ_USART1,
  .baud          = CONFIG_SAMV7_USART1_LIN_BAUD,
  .dev           =
    {
      .d_ifname  = "lin%d",
      .d_ifup    = sam_lin_ifup,
      .d_ifdown  = sam_lin_ifdown,
      .d_txavail = sam_lin_txavail,
#  ifdef CONFIG_NETDEV_IOCTL
      .d_ioctl   = sam_lin_netdev_ioctl,
#  endif
    }
};

#endif

#ifdef CONFIG_SAMV7_USART2_LIN_SOCKET

static struct sam_lin_s g_lin2priv =
{
  .port          = 2,
  .base          = SAM_USART2_BASE,
  .irq           = SAM_IRQ_USART2,
  .baud          = CONFIG_SAMV7_USART2_LIN_BAUD,
  .dev           =
    {
      .d_ifname  = "lin%d",
      .d_ifup    = sam_lin_ifup,
      .d_ifdown  = sam_lin_ifdown,
      .d_txavail = sam_lin_txavail,
#  ifdef CONFIG_NETDEV_IOCTL
      .d_ioctl   = sam_lin_netdev_ioctl,
#  endif
    }
};

#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_lin_getreg
 *
 * Description:
 *   Read the value of a USART register register.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_LIN_REGDEBUG
static uint32_t sam_lin_vgetreg(uint32_t addr)
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

static uint32_t sam_lin_getreg(struct sam_lin_s *priv, int offset)
{
  return sam_lin_vgetreg(priv->base + offset);
}

#else
static uint32_t sam_lin_getreg(struct sam_lin_s *priv, int offset)
{
  return getreg32(priv->base + offset);
}

#endif

/****************************************************************************
 * Name: sam_lin_putreg
 *
 * Description:
 *   Set the value of a USART register register.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_LIN_REGDEBUG
static void sam_lin_vputreg(uint32_t addr, uint32_t value)
{
  /* Show the register value being written */

  ninfo("%08" PRIx32 "->%08" PRIx32 "\n", addr, val);

  /* Write the value */

  putreg32(value, addr);
}

static void sam_lin_putreg(struct sam_lin_s *priv, int offset,
                            uint32_t value)
{
  sam_lin_vputreg(priv->base + offset, value);
}

#else
static void sam_lin_putreg(struct sam_lin_s *priv, int offset,
                           uint32_t value)
{
  putreg32(value, priv->base + offset);
}

#endif

/****************************************************************************
 * Function: sam_lin_ifup
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

static int sam_lin_ifup(struct net_driver_s *dev)
{
  struct sam_lin_s *priv = (struct sam_lin_s *)dev->d_private;

  /* Setup LIN */

  sam_lin_setup(priv);

  /* Enable the interrupts at the NVIC */

  up_enable_irq(priv->irq);

  priv->bifup = true;

  priv->txdesc = (struct can_frame *)priv->tx_pool;
  priv->rxdesc = (struct can_frame *)priv->rx_pool;

  priv->dev.d_buf = (uint8_t *)priv->txdesc;

  return OK;
}

/****************************************************************************
 * Function: sam_lin_ifdown
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

static int sam_lin_ifdown(struct net_driver_s *dev)
{
  struct sam_lin_s *priv = (struct sam_lin_s *)dev->d_private;

  /* Disable LIN interrupts */

  sam_lin_shutdown(priv);

  /* Reset LIN */

  sam_lin_reset(priv);

  return OK;
}

/****************************************************************************
 * Name: sam_lin_txready
 *
 * Description:
 *   Return true if the LIN hardware can accept another TX message.
 *
 ****************************************************************************/

static bool sam_lin_txready(struct sam_lin_s *priv)
{
  uint32_t regval;

  /* Return true no interrupts are enabled */

  regval = sam_lin_getreg(priv, SAM_UART_IMR_OFFSET);
  ninfo("LIN%" PRIu8 " IMR: %08" PRIx32 "\n", priv->port, regval);

  return regval == 0;
}

/****************************************************************************
 * Name: sam_lin_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Input Parameters:
 *   priv - reference to the private LIN driver state structure
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

static int sam_lin_transmit(struct sam_lin_s *priv)
{
  struct can_frame *frame = (struct can_frame *)priv->dev.d_buf;
  uint32_t          regval;
  uint8_t           lin_dlc;

  ninfo("LIN%" PRIu8 " ID: %" PRIu32 " DLC: %" PRIu8 "\n",
        priv->port, (uint32_t)frame->can_id, frame->can_dlc);

  if (!sam_lin_txready(priv))
    {
      canerr("ERROR: transmitter is busy\n");
      return -EBUSY;
    }

  lin_dlc = frame->can_dlc;

  if (lin_dlc > CAN_MAX_DLEN)
    {
      return -EINVAL;
    }

  if (lin_dlc == 0)
    {
      regval = UART_LINMR_DLM;
      switch (((frame->can_id & LIN_ID_MASK) >> 4) & 0x03)
        {
          case 3:
            lin_dlc = 8;
            break;
          case 2:
            lin_dlc = 4;
            break;
          default:
            lin_dlc = 2;
            break;
        }
    }
  else
    {
      regval = UART_LINMR_DLC(lin_dlc - 1);
    }

  /* RTR flag means that we want to read data */

  if ((frame->can_id & CAN_RTR_FLAG) != 0)
    {
      /* Setup RX information and prepare to receive the data */

      priv->rxdesc->can_id = frame->can_id & LIN_ID_MASK;
      priv->rxdesc->can_dlc = lin_dlc;
      regval |= UART_LINMR_NACT_SUBSCRIBE;
    }

  /* Use Extended ID field as identifier to select classical vs
   * enhanced checksum */

  if ((frame->can_id & CAN_EFF_FLAG) == 0)
    {
      regval |= UART_LINMR_CHKTYP;
    }

  /* Abort previous transaction. Just in case */

  sam_lin_putreg(priv, SAM_UART_CR_OFFSET, UART_CR_LINABT | UART_CR_RSTSTA);

  /* Write LIN Mode Register value */

  sam_lin_putreg(priv, SAM_UART_LINMR_OFFSET, regval);

  /* Write LIN Identifier Register value */

  sam_lin_putreg(priv, SAM_UART_LINIR_OFFSET,
                 frame->can_id & UART_LINIR_MASK);

  /* Enable LIN Identifier Sent interrupt */

  sam_lin_putreg(priv, SAM_UART_IER_OFFSET, UART_INT_LINID | UART_INT_LINERR);

  return OK;
}

/****************************************************************************
 * Function: sam_lin_txpoll
 *
 * Description:
 *   The transmitter is available, check if the network has any outgoing
 *   packets ready to send.  This is a callback from devif_poll().
 *   devif_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. When the preceding TX packet send timeout and the interface is reset
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

static int sam_lin_txpoll(struct net_driver_s *dev)
{
  struct sam_lin_s *priv = (struct sam_lin_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (priv->dev.d_len > 0)
    {
      /* Send the packet */

      sam_lin_transmit(priv);

      /* There is no room in the device to hold another packet.  Return a
       * non-zero value to terminate the poll.
       */

      return -EBUSY;
    }

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return 0;
}

/****************************************************************************
 * Function: sam_lin_txavail_work
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

static void sam_lin_txavail_work(void *arg)
{
  struct sam_lin_s *priv = (struct sam_lin_s *)arg;

  /* Ignore the notification if the interface is not yet up */

  net_lock();
  if (priv->bifup)
    {
      /* Check if there the hardware is capable to process another outgoing
       * packet.
       */

      if (sam_lin_txready(priv))
        {
          /* Yes, there is space for another transfer.  Poll the network for
           * new XMIT data.
           */

          devif_poll(&priv->dev, sam_lin_txpoll);
        }
    }

  net_unlock();
}

/****************************************************************************
 * Function: sam_lin_txavail
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

static int sam_lin_txavail(struct net_driver_s *dev)
{
  struct sam_lin_s *priv = (struct sam_lin_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      sam_lin_txavail_work(priv);
    }

  return OK;
}

/****************************************************************************
 * Function: sam_lin_ioctl
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
static int sam_lin_netdev_ioctl(struct net_driver_s *dev, int cmd,
                                unsigned long arg)
{
  struct sam_lin_s *priv = (struct sam_lin_s *)dev->d_private;
  int               ret  = OK;

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

static int sam_interrupt(int irq, void *context, FAR void *arg)
{
  struct sam_lin_s *priv = (struct sam_lin_s *)arg;
  struct can_frame *frame = (struct can_frame *)priv->txdesc;
  uint32_t pending;
  uint32_t imr;
  int passes;
  bool handled;

  /* Loop until there are no characters to be transferred or, until we have
   * been looping for a long time.
   */

  handled = true;
  for (passes = 0; passes < 256 && handled; passes++)
    {
      handled = false;

      /* Get the USART status (we are only interested in the unmasked
       * interrupts).
       */

      priv->sr = sam_lin_getreg(priv, SAM_UART_SR_OFFSET);
      imr      = sam_lin_getreg(priv, SAM_UART_IMR_OFFSET);
      pending  = priv->sr & imr;

      /* Handle an incoming, receive byte.  RXRDY: At least one complete
       * character has been received and US_RHR has not yet been read.
       */

      if ((pending & UART_INT_RXRDY) != 0)
        {
          uint8_t byte;

          /* Received data ready... process incoming bytes */

          byte = sam_lin_getreg(priv, SAM_UART_RHR_OFFSET) & 0xff;

          /* Store byte to RX buffer */

          if (priv->idx < priv->rxdesc->can_dlc)
            {
              priv->rxdesc->data[priv->idx] = byte;
              priv->idx++;
            }
          else
            {
              /* Disable RX ready interrupt */

              sam_lin_putreg(priv, SAM_UART_IDR_OFFSET, UART_INT_RXRDY);
            }

          handled = true;
        }

      /* Handle outgoing, transmit bytes. TXRDY: There is no character in the
       * US_THR.
       */

      if ((pending & UART_INT_TXRDY) != 0)
        {
          /* Transmit data register empty ... process outgoing bytes */

          if (priv->idx < priv->txdesc->can_dlc)
            {
              sam_lin_putreg(priv, SAM_UART_THR_OFFSET,
                             priv->txdesc->data[priv->idx]);
              priv->idx++;
            }
          else
            {
              /* Disable TX ready interrupt */

              sam_lin_putreg(priv, SAM_UART_IDR_OFFSET, UART_INT_TXRDY);
            }

          handled = true;
        }

      /* Handle event when LIN identifier has been sent */

      if ((pending & UART_INT_LINID) != 0)
        {
          /* Disable LIN ready interrupt */

          sam_lin_putreg(priv, SAM_UART_IDR_OFFSET, UART_INT_LINID);

          /* Check LIN RX or TX */

          if ((frame->can_id & CAN_RTR_FLAG) != 0)
            {
              /* Enable RX ready interrupt */

              sam_lin_putreg(priv, SAM_UART_IER_OFFSET, UART_INT_RXRDY);
            }
          else
            {
              /* Enable TX ready interrupt */

              sam_lin_putreg(priv, SAM_UART_IER_OFFSET, UART_INT_TXRDY);
            }

          /* Enable LIN Transfer Completed interrupt */

          sam_lin_putreg(priv, SAM_UART_IER_OFFSET, UART_INT_LINTC);

          handled = true;
        }

      /* Handle LIN frame transfer complete */

      if ((pending & (UART_INT_LINTC | UART_INT_LINERR)) != 0)
        {
          /* Disable all interrupts */

          sam_lin_putreg(priv, SAM_UART_IDR_OFFSET, 0xffffffff);

          if ((pending & UART_INT_LINTC) != 0)
            {
              /* LIN transaction complete. Start RX/TX interrupt worker */

              if ((frame->can_id & CAN_RTR_FLAG) != 0)
                {
                  work_queue(LINWORK, &priv->irqwork,
                             sam_lin_rxinterrupt_work, priv, 0);
                }
              else
                {
                  work_queue(LINWORK, &priv->irqwork,
                             sam_lin_txdone_work, priv, 0);
                }
            }
#ifdef CONFIG_NET_CAN_ERRORS
          else
            {
              work_queue(LINWORK, &priv->irqwork,
                         sam_lin_errinterrupt_work, priv, 0);
            }
#endif

          /* Reset transaction status bit */

          sam_lin_putreg(priv, SAM_UART_CR_OFFSET, UART_CR_RSTSTA);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: sam_lin_rxinterrupt_work
 ****************************************************************************/

static void sam_lin_rxinterrupt_work(void *arg)
{
  struct sam_lin_s *priv = (struct sam_lin_s *)arg;

  DEBUGASSERT(priv != NULL);

  /* Copy the buffer pointer to priv->dev..  Set amount of data
   * in priv->dev.d_len
   */

  priv->dev.d_len = sizeof(struct can_frame);
  priv->dev.d_buf = (uint8_t *)priv->rxdesc;

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
}

/****************************************************************************
 * Name: sam_lin_txdone_work
 ****************************************************************************/

static void sam_lin_txdone_work(void *arg)
{
  struct sam_lin_s *priv = (struct sam_lin_s *)arg;

  sam_lin_txdone(priv);

  /* There should be space for a new TX in any event.  Poll the network for
   * new XMIT data
   */

  net_lock();
  devif_poll(&priv->dev, sam_lin_txpoll);
  net_unlock();
}

/****************************************************************************
 * Name: sam_lin_txdone
 ****************************************************************************/

static void sam_lin_txdone(struct sam_lin_s *priv)
{
  NETDEV_TXDONE(&priv->dev);
}

#ifdef CONFIG_NET_CAN_ERRORS

/****************************************************************************
 * Name: sam_lin_sceinterrupt_work
 *
 * Description:
 *   LIN error interrupt work
 *
 * Input Parameters:
 *   arg  - reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_lin_errinterrupt_work(void *arg)
{
  struct sam_lin_s *priv    = (struct sam_lin_s *)arg;
  struct can_frame *frame   = (struct can_frame *)priv->rxdesc;
  uint32_t          regval  = 0;
  uint16_t          errbits = 0;
  uint8_t           data[CAN_ERR_DLC];

  DEBUGASSERT(priv != NULL);

  /* Check Error Interrupt flag */

  if ((priv->sr & UART_INT_LINERR) != 0)
    {
      /* Encode error bits */

      errbits = 0;
      memset(data, 0, sizeof(data));

      if ((priv->sr & UART_INT_LINBE) != 0)
        {
          /* Buss error */

          errbits |= CAN_ERR_BUSERROR;
        }

      if ((priv->sr & UART_INT_LINISFE) != 0)
        {
          /* Inconsistent Synch Field Error */

          errbits |= CAN_ERR_LOSTARB;
        }

      if ((priv->sr & UART_INT_LINIPE) != 0)
        {
          /* Identifier Parity Error */

          errbits |= CAN_ERR_PROT;
          data[3] |= CAN_ERR_PROT_LOC_IDE;
        }

      if ((priv->sr & UART_INT_LINCE) != 0)
        {
          /* Checksum Error */

          errbits |= CAN_ERR_PROT;
          data[3] |= CAN_ERR_PROT_LOC_CRC_SEQ;
        }

      if ((priv->sr & UART_INT_LINSNRE) != 0)
        {
          /* Slave Not Responding Error */

          errbits |= CAN_ERR_ACK;
        }
    }

  /* Report a LIN error */

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
}
#endif

/****************************************************************************
 * Name: sam_lin_setup
 ****************************************************************************/

static int sam_lin_setup(struct sam_lin_s *priv)
{
  uint64_t divb3;
  uint32_t intpart;
  uint32_t fracpart;
  uint32_t regval;
  int ret;

  ninfo("LIN%" PRIu8 " USART irq: %" PRIu8 "\n", priv->port, priv->irq);

  /* Reset and disable receiver and transmitter */

  sam_lin_putreg(priv, SAM_UART_CR_OFFSET, UART_CR_RSTRX | UART_CR_RSTTX |
                                           UART_CR_RXDIS | UART_CR_TXDIS);

  /* Disable all interrupts */

  sam_lin_putreg(priv, SAM_UART_IDR_OFFSET, 0xffffffff);

  regval = UART_MR_MODE_LINMSTR | UART_MR_CHRL_8BITS | UART_MR_PAR_NONE |
           UART_MR_NBSTOP_1;

  /* Configure the console baud:
   *
   *   Fbaud   = USART_CLOCK / (16 * divisor)
   *   divisor = USART_CLOCK / (16 * Fbaud)
   *
   * NOTE: Oversampling by 8 is not supported. This may limit BAUD rates
   * for lower USART clocks.
   */

  divb3    = ((FAST_USART_CLOCK + (priv->baud << 3)) << 3) /
             (priv->baud << 4);
  intpart  = (divb3 >> 3);
  fracpart = (divb3 & 7);

  /* Retain the fast MR peripheral clock UNLESS unless using that clock
   * would result in an excessively large divider.
   *
   * REVISIT: The fractional divider is not used.
   */

  if ((intpart & ~UART_BRGR_CD_MASK) != 0)
    {
      /* Use the divided USART clock */

      divb3    = ((SLOW_USART_CLOCK + (priv->baud  << 3)) << 3) /
                 (priv->baud  << 4);
      intpart  = (divb3 >> 3);
      fracpart = (divb3 & 7);

      regval |= UART_MR_USCLKS_MCKDIV;
    }
  else
    {
      regval |= UART_MR_USCLKS_MCK;
    }

  /* Set mode register value */

  sam_lin_putreg(priv, SAM_UART_MR_OFFSET, regval);

  /* Save the BAUD divider (the fractional part is not used for UARTs) */

  regval = UART_BRGR_CD(intpart) | UART_BRGR_FP(fracpart);
  sam_lin_putreg(priv, SAM_UART_BRGR_OFFSET, regval);

  /* Enable receiver & transmitter */

  sam_lin_putreg(priv, SAM_UART_CR_OFFSET, UART_CR_RXEN | UART_CR_TXEN);

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, sam_interrupt, priv);
  if (ret == OK)
    {
      /* Enable the interrupt (RX and TX interrupts are still disabled
       * in the USART
       */

      up_enable_irq(priv->irq);
    }

  return OK;
}

/****************************************************************************
 * Name: sam_lin_shutdown
 ****************************************************************************/

static void sam_lin_shutdown(struct sam_lin_s *priv)
{
  ninfo("LIN%" PRIu8 "\n", priv->port);

  /* Disable USART interrupts */

  up_disable_irq(priv->irq);

  /* Detach USART interrupts */

  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: sam_lin_reset
 *
 * Description:
 *   Put the LIN device in the non-operational, reset state
 *
 * Input Parameters:
 *   priv - reference to the private LIN driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void sam_lin_reset(struct sam_lin_s *priv)
{
  ninfo("LIN%" PRIu8 "\n", priv->port);

  /* Reset and disable receiver and transmitter */

  sam_lin_putreg(priv, SAM_UART_CR_OFFSET, UART_CR_RSTRX | UART_CR_RSTTX |
                                           UART_CR_RXDIS | UART_CR_TXDIS);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_linsockinitialize
 *
 * Description:
 *   Initialize the selected LIN port as LIN socket interface
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple LIN interfaces)
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int sam_linsockinitialize(int port)
{
  struct sam_lin_s *priv = NULL;

  ninfo("LIN%" PRIu8 "\n", port);

  /* Select the LIN device structure */

  switch (port)
    {
#ifdef CONFIG_SAMV7_USART0_LIN_SOCKET
      case 0:
        priv = &g_lin0priv;

        sam_usart0_enableclk();

        sam_configgpio(GPIO_USART0_RXD);
        sam_configgpio(GPIO_USART0_TXD);
        break;
#endif

#ifdef CONFIG_SAMV7_USART1_LIN_SOCKET
      case 1:
        priv = &g_lin1priv;

        sam_usart1_enableclk();

        sam_configgpio(GPIO_USART1_RXD);
        sam_configgpio(GPIO_USART1_TXD);
        break;
#endif

#ifdef CONFIG_SAMV7_USART2_LIN_SOCKET
      case 2:
        priv = &g_lin2priv;

        sam_usart2_enableclk();

        sam_configgpio(GPIO_USART2_RXD);
        sam_configgpio(GPIO_USART2_TXD);
        break;
#endif

      default:
        nerr("ERROR: Unsupported port %d\n", port);
        return -ENODEV;
    }

  /* Setup device private information */

  priv->dev.d_private = priv;

  /* Put the interface in the down state.  This usually amounts to resetting
   * the device and/or calling sam_lin_ifdown().
   */

  ninfo("callbacks done\n");

  sam_lin_ifdown(&priv->dev);

  /* Register the device with the OS so that socket IOCTLs can be performed */

  return netdev_register(&priv->dev, NET_LL_CAN);
}

/****************************************************************************
 * Name: arm_netinitialize
 *
 * Description:
 *   Initialize the LIN device interfaces.  If there is more than one device
 *   interface in the chip, then board-specific logic will have to provide
 *   this function to determine which, if any, LIN interfaces should be
 *   initialized.
 *
 ****************************************************************************/

#if !defined(CONFIG_NETDEV_LATEINIT)
void arm_netinitialize(void)
{
#ifdef CONFIG_SAMV7_USART0_LIN_SOCKET
  sam_linsockinitialize(0);
#endif

#ifdef CONFIG_SAMV7_USART1_LIN_SOCKET
  sam_linsockinitialize(1);
#endif

#ifdef CONFIG_SAMV7_USART2_LIN_SOCKET
  sam_linsockinitialize(2);
#endif
}
#endif
