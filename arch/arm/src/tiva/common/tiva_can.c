/****************************************************************************
 * arch/arm/src/tiva/common/tiva_can.c
 * Classic (character-device) lower-half driver for the Tiva CAN modules.
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

#include <stdio.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/kthread.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>

#include <nuttx/can/can.h>

#ifdef CONFIG_CAN_ERRORS
/* A self-reenqueuing worker is used to report errors periodically when
 * status interrupts are disabled due to overload.
 */

#include <nuttx/wqueue.h>
#endif

#include "arm_internal.h"

#include "chip.h"
#include "tiva_can.h"
#include "hardware/tiva_can.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_TIVA_CAN) && (defined(CONFIG_TIVA_CAN0) || defined(CONFIG_TIVA_CAN1))

#if defined(CONFIG_CAN_ERRORS) && !defined(CONFIG_SCHED_LPWORK)
#  error "Error reporting requires the low-priority work queue to be enabled."
#endif

/* Combines two 16-bit registers into a single 32 bit value */
#define tivacan_readsplitreg32(base, r1, r2) ((getreg32((base) + (r1)) & 0xffff) | (getreg32((base) + (r2)) << 16))

#define tivacan_get_nwda32(base) tivacan_readsplitreg32((base), TIVA_CAN_OFFSET_NWDA1, TIVA_CAN_OFFSET_NWDA2)
#define tivacan_get_txrq32(base) tivacan_readsplitreg32((base), TIVA_CAN_OFFSET_TXRQ1, TIVA_CAN_OFFSET_TXRQ2)
#define tivacan_get_msgval32(base) tivacan_readsplitreg32((base), TIVA_CAN_OFFSET_MSG1VAL, TIVA_CAN_OFFSET_MSG2VAL)

/* Only the last few mailboxes are used for TX to help ensure that responses
 * to remote request frames we send are always collected by an RX FIFO and
 * not by the mailbox used to send the remote frame. (The Tiva hardware uses
 * the mailbox with the lowest number first.)
 *
 * This number is zero-indexed, althought the Command Request Register isn't.
 */
#define TIVA_CAN_TX_FIFO_START (TIVA_CAN_NUM_MBOXES - CONFIG_TIVA_CAN_TX_FIFO_DEPTH)

/* Frequency of async error message reporting when rate-limited */
#define TIVA_CAN_ERR_HANDLER_TICKS MSEC2TICK(CONFIG_TIVA_CAN_ERR_HANDLER_PER)

/* For the RX receive kthread */
#define TIVA_CAN_KTHREAD_STACKSIZE 1024

/* For tivacan_initfilter */
#define TIVA_CAN_FILTER_TYPE_STD 0
#define TIVA_CAN_FILTER_TYPE_EXT 1

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Bitfield representing the mailboxes belonging to this FIFO. The
 * high bit (mask: 1 << 31) corresponds to mailbox 31, the low bit
 * (mask: 1 << 0) corresponds to mailbox 0.
 *
 * NOTE: If a new chip came along with more mailboxes, this would
 * need to be reworked.
 */

typedef uint32_t tiva_can_fifo_t;

/* This structure represents a CAN module on the microcontroller */

struct tiva_canmod_s
{
  /* Module number, mostly for syscon register fields */

  int       modnum;

  /* Registers base address */

  uint32_t  base;

  /* Contents of the CANSTS reg the last time errors were processed. */

  uint32_t  status;

  /* Contents of the CANERR reg the last time errors were processed. */

  uint32_t  errors;

  /* kthread message handler waits on this; ISR posts */

  sem_t     rxsem;

  /* Mailbox number (zero-indexed) that the kthread message handler is
   * currently retrieving from the CAN module SRAM or in the process of
   * sending to the application. The kthread works through messages in
   * ascending order, so if the ISR will only post the semaphore for received
   * messages that the kthread has already "passed by."
   */

  int       kthd_cur_mbox;

  /* kthread message handler thread ID */

  pid_t     kthd_id;

#ifdef CONFIG_CAN_ERRORS
  /* Asynchronously report errors when status interrupts are disabled */

  struct work_s error_work;
#endif

  /* Whether to autorecover from bus-off conditions */

  bool      autorecover;

  /* Mutex for threads accessing the interface registers */

  mutex_t   thd_iface_lock;

  /* Interface registers base address for threads threads */

  uint32_t  thd_iface_base;

  /* Interface registers base address reserved exclusively for the ISR */

  uint32_t  isr_iface_base;

  /* Mutex for claiming, freeing, and potentially resizing RX FIFOs.
   * The TX FIFO should never be resized at runtime.
   */

  mutex_t   fifo_lock;

  /* All RX FIFOs + 1 TX FIFO */

  tiva_can_fifo_t fifos[CONFIG_TIVA_CAN_FILTERS_MAX + 1];

  /* Pointer to default catch-all RX FIFO initialized by the driver. */

  tiva_can_fifo_t *rxdefault_fifo;

  /* Pointer to the permanent, fixed-size TX FIFO. This is always located at
   * the end of the series of mailboxes to help ensure that responses to
   * remote request frames go to an RX (filter) FIFO and don't interfere.
   */

  tiva_can_fifo_t *tx_fifo;

  /* Index in the TX FIFO where new messages should be added. The Tiva CAN
   * module doesn't support a ring buffer, so new messages are only added
   * when the TX FIFO is empty (tx_fifo_tail == 0).
   */

  int tx_fifo_tail;
};

/* This structure represents the CAN bit timing settings as seen by the hw. */

struct tiva_can_timing_s
{
  int       prescaler;
  int       tseg1;
  int       tseg2;
  int       sjw;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Callbacks for upper-half CAN driver */

static void tivacan_reset(struct can_dev_s *dev);
static int  tivacan_setup(struct can_dev_s *dev);
static void tivacan_shutdown(struct can_dev_s *dev);
static void tivacan_rxintctl(struct can_dev_s *dev, bool enable);
static void tivacan_txintctl(struct can_dev_s *dev, bool enable);
static int  tivacan_ioctl(struct can_dev_s *dev,
                          int cmd, unsigned long arg);
static int  tivacan_send(struct can_dev_s *dev,
                         struct can_msg_s *msg);
static bool tivacan_txready(struct can_dev_s *dev);
static bool tivacan_txempty(struct can_dev_s *dev);

/* ISR */

static int  tivacan_isr(int irq, void *context, void *dev);

/* Internal utility functions */

static struct tiva_can_timing_s
tivacan_bittiming_get(struct can_dev_s *dev);

static void tivacan_bittiming_set(struct can_dev_s *dev,
                                  struct tiva_can_timing_s *timing);

int tivacan_alloc_fifo(struct can_dev_s *dev, int depth);
static void tivacan_free_fifo(struct can_dev_s *dev,
                              tiva_can_fifo_t *fifo);
static void tivacan_disable_mbox(struct can_dev_s *dev,
                                 uint32_t iface_base,
                                 int num);
static int  tivacan_initfilter(struct can_dev_s *dev,
                               tiva_can_fifo_t  *fifo,
                               void             *filter,
                               int               type);
static int  tivacan_rxhandler(int argc, char** argv);
int tivacan_handle_errors(struct can_dev_s *dev);

#ifdef CONFIG_CAN_ERRORS
void tivacan_handle_errors_wqueue(void *dev);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* List of callbacks provided to can_register via g_tivacanNdev */

static const struct can_ops_s g_tivacanops =
{
  .co_reset         = tivacan_reset,
  .co_setup         = tivacan_setup,
  .co_shutdown      = tivacan_shutdown,
  .co_rxint         = tivacan_rxintctl,
  .co_txint         = tivacan_txintctl,
  .co_ioctl         = tivacan_ioctl,
  .co_remoterequest = NULL,               /* Use CONFIG_CAN_USE_RTR instead */
  .co_send          = tivacan_send,
  .co_txready       = tivacan_txready,
  .co_txempty       = tivacan_txempty,
};

#ifdef CONFIG_TIVA_CAN0
static struct tiva_canmod_s g_tivacan0priv =
{
  .modnum           = 0,
  .base             = TIVA_CAN_BASE(0),
  .rxsem            = SEM_INITIALIZER(0),
  .thd_iface_lock   = NXMUTEX_INITIALIZER,
  .thd_iface_base   = TIVA_CAN_IFACE_BASE(0, 0),
  .isr_iface_base   = TIVA_CAN_IFACE_BASE(0, 1),
  .fifo_lock        = NXMUTEX_INITIALIZER,
  .rxdefault_fifo   = NULL,
};

static struct can_dev_s g_tivacan0dev =
{
  .cd_ops           = &g_tivacanops,
  .cd_priv          = &g_tivacan0priv,
};
#endif /* CONFIG_TIVA_CAN0 */

#ifdef CONFIG_TIVA_CAN1
static struct tiva_canmod_s g_tivacan1priv =
{
  .modnum           = 1,
  .base             = TIVA_CAN_BASE(1),
  .rxsem            = SEM_INITIALIZER(0),
  .thd_iface_lock   = NXMUTEX_INITIALIZER,
  .thd_iface_base   = TIVA_CAN_IFACE_BASE(1, 0),
  .isr_iface_base   = TIVA_CAN_IFACE_BASE(1, 1),
  .fifo_lock        = NXMUTEX_INITIALIZER,
  .rxdefault_fifo   = NULL,
};

static struct can_dev_s g_tivacan1dev =
{
  .cd_ops           = &g_tivacanops,
  .cd_priv          = &g_tivacan1priv,
};
#endif /* CONFIG_TIVA_CAN1 */

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tivacan_reset
 *
 * Description:
 *   Software-reset the CAN module using the SYSCON registers.
 *
 *   A pointer to this function is passed to can_register.
 *
 * Input parameters:
 *   dev - An instance of the "upper half" CAN driver structure
 *
 * Returned value: None
 ****************************************************************************/

static void tivacan_reset(struct can_dev_s *dev)
{
  struct tiva_canmod_s *canmod = dev->cd_priv;
  int modnum = canmod->modnum;
#ifndef CONFIG_TIVA_CAN0
  if (modnum == 0)
    {
      canerr("ERROR: tried to reset disabled module CAN0\n");
      return;
    }

#endif
#ifndef CONFIG_TIVA_CAN1
  if (modnum == 1)
    {
      canerr("ERROR: tried to reset disabled module CAN1\n");
    }

#endif
  if (modnum > 1)
    {
      canerr("ERROR: tried to reset nonexistant module CAN%d\n",
             canmod->modnum);
    }

  modifyreg32(TIVA_SYSCON_SRCAN, 0, SYSCON_SRCAN(modnum));

  /* Spin for a few clock cycles */

  for (volatile int i = 0; i < 16; ++i);

  modifyreg32(TIVA_SYSCON_SRCAN, SYSCON_SRCAN(modnum), 0);

  /* Wait for peripheral-ready signal */

  while (!(getreg32(TIVA_SYSCON_PRCAN) & (1 << modnum)));
}

/****************************************************************************
 * Name: tivacan_setup
 *
 * Description:
 *   Set up the CAN module. Register the ISR and enable the interrupt in the
 *   NVIC. Set default bit-timing and set filters to a consistent state. On
 *   return, the INIT bit is cleared in the CANCTL register, and interrupts
 *   are enabled on the side of the CAN module. (can_ops_s.txint and
 *   can_ops_s.rxint are unsuitable because the Tiva CAN modules do not allow
 *   disabling TX and RX interrupts separately).
 *
 *   A pointer to this function is passed to can_register.
 *
 * Input parameters:
 *   dev - An instance of the "upper half" CAN driver structure
 *
 * Returned value:
 *   Zero on success, or a negated errno on failure.
 ****************************************************************************/

static int tivacan_setup(struct can_dev_s *dev)
{
  uint32_t irq;
  int      ret;
  uint32_t reg;
  struct tiva_canmod_s *canmod = dev->cd_priv;
  char    *kthd_argv[2];
  kthd_argv[1] = NULL;

  switch (canmod->modnum)
    {
#ifdef CONFIG_TIVA_CAN0
      case 0:
        {
          kthd_argv[0] = "0";
          ret = kthread_create("Tiva CAN0",
                               CONFIG_TIVA_CAN0_PRIO,
                               TIVA_CAN_KTHREAD_STACKSIZE,
                               tivacan_rxhandler,
                               kthd_argv);
        }
      break;
#endif
#ifdef CONFIG_TIVA_CAN1
      case 1:
        {
          kthd_argv[0] = "1";
          ret = kthread_create("Tiva CAN1",
                               CONFIG_TIVA_CAN1_PRIO,
                               TIVA_CAN_KTHREAD_STACKSIZE,
                               tivacan_rxhandler,
                               kthd_argv);
        }
      break;
#endif
      default:
        return -EINVAL;
    }

  if (ret < 0)
    {
      canerr("tiva_can: failed to create kthread\n");
      return ret;
    }
  else
    {
      canmod->kthd_id = (pid_t)ret;
    }

#ifdef CONFIG_CAN_EXTID
  struct canioc_extfilter_s default_filter =
  {
    .xf_id1   = 0x123,
    .xf_id2   = 0,
    .xf_type  = CAN_FILTER_MASK,
    .xf_prio  = CAN_MSGPRIO_HIGH,
  };
#else
  struct canioc_stdfilter_s default_filter =
  {
    .sf_id1   = 0x123,
    .sf_id2   = 0,
    .sf_type  = CAN_FILTER_MASK,
    .sf_prio  = CAN_MSGPRIO_HIGH,
  };
#endif

  /* REVISIT: This will not necessarily come out nicely on boards with
   * different crystals.
   */

  struct canioc_bittiming_s default_timing =
  {
    .bt_baud  = 1000000,
    .bt_tseg1    = 6,
    .bt_tseg2    = 3,
    .bt_sjw      = 3,
  };

  /* Clear the status interrupt, just in case */

  getreg32(canmod->base + TIVA_CAN_OFFSET_STS);

  /* Set default bit timing */

  tivacan_ioctl(dev, CANIOC_SET_BITTIMING, (unsigned long)&default_timing);

  nxmutex_lock(&canmod->thd_iface_lock);

  /* Ensure a consistent state */

  for (int i = 0; i < TIVA_CAN_NUM_MBOXES; ++i)
    {
      tivacan_disable_mbox(dev, canmod->thd_iface_base, i);
    }

  /* Ensure the TX mailboxes have filtering enabled. Since higher mailbox
   * numbers are lower-priority for receiving messages, this is might not be
   * necessary - if the message matches an existing filter FIFO, the hardware
   * _shouldn't_ let it "fall through" to the next mailbox if the
   * End of Buffer bit is set (which tivacan_initfilter guarantees). However,
   * the datasheet does not explicitly guarantee this.
   *
   * The other case to be considered is if the incoming message doesn't match
   * any of the mailboxes in the existing filter FIFOs. In this case it could
   * fall through to the TX mailbox. The datasheet doesn't specify what
   * happens if an incoming message matches a mailbox with TXRQST set. It
   * is possible that the message would overwrite the contents of the TX
   * mailbox, which would be very bad.
   *
   * There are two parts to the filtering rules:
   *  - MDIR: filter-on-direction. This prevents received messages (of any
   *          type - RTR or normal data messages) from matching the TX
   *          mailbox whenever we're transmitting a data frame
   *          (i.e. ARB2.DIR = 1)
   *
   *  - ID filtering: Since MDIR does not protect the TX mailbox when a
   *                  remote request frame is being transmitted
   *                  (ARB2.DIR = 0), we require an exact match of the ID
   *                  and ID type (i.e. standard/extended) to limit the
   *                  likelihood of the remote request being corrupted before
   *                  transmission. Since remote requests contain no data,
   *                  the only thing that would be changed by a received
   *                  message would be the DLC.
   *
   * The MCTL.UMASK bit is set in tivacan_send.
   */

  for (int i = TIVA_CAN_TX_FIFO_START; i < TIVA_CAN_NUM_MBOXES; ++i)
    {
      reg = TIVA_CANIF_MSK1_IDMSK_EXT_MASK;
      putreg32(reg, canmod->thd_iface_base + TIVA_CANIF_OFFSET_MSK1);
      reg = TIVA_CANIF_MSK2_IDMSK_EXT_MASK
            | TIVA_CANIF_MSK2_MDIR
            | TIVA_CANIF_MSK2_MXTD;
      putreg32(reg, canmod->thd_iface_base + TIVA_CANIF_OFFSET_MSK2);

      putreg32(TIVA_CANIF_CMSK_MASK,
              canmod->thd_iface_base + TIVA_CANIF_OFFSET_CMSK);

      /* One-indexed */

      putreg32(i + 1, canmod->thd_iface_base + TIVA_CANIF_OFFSET_CRQ);

      /* Wait for request to process */

      while (getreg32(canmod->thd_iface_base + TIVA_CANIF_OFFSET_CRQ)
              & TIVA_CANIF_CRQ_BUSY);
    }

  nxmutex_unlock(&canmod->thd_iface_lock);

  /* Register the ISR */

  switch (canmod->modnum)
    {
      case 0:
        {
          irq = TIVA_IRQ_CAN0;
        }
        break;
      case 1:
        {
          irq = TIVA_IRQ_CAN1;
        }
        break;
      default:
        {
          canerr("ERROR: no such CAN module\n");
          return -ENODEV;
        }
    }

  ret = irq_attach(irq, tivacan_isr, dev);
  if (ret < 0)
    {
      canerr("ERROR: Failed to register CAN ISR.\n");
      return ret;
    }

  up_enable_irq(irq);

  /* Exit init mode and enable interrupts from the CAN module */

  modifyreg32(canmod->base + TIVA_CAN_OFFSET_CTL, TIVA_CAN_CTL_INIT,
              TIVA_CAN_CTL_EIE | TIVA_CAN_CTL_IE);

#ifdef CONFIG_CAN_ERRORS
  /* Status interrupts are used for reporting lost arbitration, etc. */

  modifyreg32(canmod->base + TIVA_CAN_OFFSET_CTL, 0, TIVA_CAN_CTL_SIE);
#endif

  /* The TX FIFO is manually allocated first because it must be located at
   * the end of the series of mailboxes to make sure responses to remote
   * request frames go to the correct place.
   */

  canmod->fifos[CONFIG_TIVA_CAN_FILTERS_MAX] =
                          0xffffffff << (32 - CONFIG_TIVA_CAN_TX_FIFO_DEPTH);
  canmod->tx_fifo = &canmod->fifos[CONFIG_TIVA_CAN_FILTERS_MAX];

  ret = tivacan_alloc_fifo(dev, CONFIG_TIVA_CAN_DEFAULT_FIFO_DEPTH);
  if (ret < 0)
    {
      canerr("ERROR: Failed to allocate default RX FIFO.\n");
      return ret;
    }
  else
    {
      canmod->rxdefault_fifo = &canmod->fifos[ret];
    }

#ifdef CONFIG_CAN_EXTID
  tivacan_initfilter(dev,
                     canmod->rxdefault_fifo,
                     &default_filter,
                     TIVA_CAN_FILTER_TYPE_EXT);
#else
  tivacan_initfilter(dev,
                     canmod->rxdefault_fifo,
                     &default_filter,
                     TIVA_CAN_FILTER_TYPE_STD);
#endif

  return OK;
}

/****************************************************************************
 * Name: tivacan_shutdown
 *
 * Description:
 *   Shut down the CAN module. This function does the reverse of
 *   tivacan_setup. The ISR is disabled in the NVIC, unregistered from the
 *   NuttX dispatcher, and the CAN module is reset.
 *
 *   A pointer to this function is passed to can_register.
 *
 * Input parameters:
 *   dev - An instance of the "upper half" CAN driver structure
 *
 * Returned value: None
 ****************************************************************************/

static void tivacan_shutdown(struct can_dev_s *dev)
{
  int irq;
  struct tiva_canmod_s    *canmod = dev->cd_priv;

  switch (canmod->modnum)
    {
      case 0:
        {
          irq = TIVA_IRQ_CAN0;
        }
        break;
      case 1:
        {
          irq = TIVA_IRQ_CAN1;
        }
        break;
      default:
        canerr("ERROR: No such CAN module to disable interrupt\n");
    }

  /* Free all allocated RX filter FIFOs */

  for (int i = 0; i < CONFIG_TIVA_CAN_FILTERS_MAX; ++i)
  {
    if (canmod->fifos[i])
    {
      tivacan_free_fifo(dev, &canmod->fifos[i]);
    }
  }

  /* Free the TX FIFO */

  tivacan_free_fifo(dev, canmod->tx_fifo);

  /* Disable interrupts from the CAN module */

  modifyreg32(canmod->base + TIVA_CAN_OFFSET_CTL,
              TIVA_CAN_CTL_EIE | TIVA_CAN_CTL_SIE | TIVA_CAN_CTL_IE, 0);

  up_disable_irq(irq);
  irq_detach(irq);

  /* Stop processing messages */

  kthread_delete(canmod->kthd_id);

  tivacan_reset(dev);
}

/****************************************************************************
 * Name: tivacan_rxhandler
 *
 * Description: Kthread RX message handler.
 ****************************************************************************/

int tivacan_rxhandler(int argc, char** argv)
{
  struct can_dev_s     *dev;
  struct tiva_canmod_s *canmod;
  struct can_msg_s msg;

#ifdef CONFIG_CAN_ERRORS
  int ret;
#endif

  /* argv[0] contains the thread name */

  switch (argv[1][0])
    {
#ifdef CONFIG_TIVA_CAN0
      case '0':
        {
          dev = &g_tivacan0dev;
        }
        break;
#endif
#ifdef CONFIG_TIVA_CAN1
      case '1':
        {
          dev = &g_tivacan1dev;
        }
        break;
#endif
      default:
        canerr("Incorrect Tiva CAN module passed to kthread.\n");
        return -EINVAL;
    }

  canmod = dev->cd_priv;
  int     *mbox = &canmod->kthd_cur_mbox;

  /* Must clear the garbage */

  memset(&msg, 0, sizeof(struct can_msg_s));

  while (true)
    {
      nxsem_wait(&canmod->rxsem);
      nxmutex_lock(&canmod->thd_iface_lock);

      /* Process the received message(s). Since hardware RX FIFOS are used
       * and new messages are received into the mailbox with the lowest
       * number, we can't use CANINT since a new message might arrive in a
       * mailbox we'd already read from before the higher-address mailboxes
       * in the FIFO were emptied. (Besides, the ISR clears CANINT).
       */

      for (*mbox = 0;
           *mbox < TIVA_CAN_NUM_MBOXES - CONFIG_TIVA_CAN_TX_FIFO_DEPTH;
          ++*mbox)
        {
          uint32_t reg;

          /* New Data registers */

          reg = tivacan_get_nwda32(canmod->base);

          if (!(reg & ~*(canmod->tx_fifo)))
            {
              /* No new messages to process in the RX FIFOs */

              break;
            }
          else if (! (reg & 1 << *mbox))
            {
              /* No new message in this mailbox */

              continue;
            }

          /* Command Mask register
           * Pull arbitration bits, control bits, and data bits from the
           * message SRAM. Clear the new data bit; the ISR cleared the IRQ.
           */

          reg =   TIVA_CANIF_CMSK_ARB       | TIVA_CANIF_CMSK_CONTROL
                | TIVA_CANIF_CMSK_NEWDAT    | TIVA_CANIF_CMSK_DATAA
                | TIVA_CANIF_CMSK_DATAB;
          putreg32(reg, canmod->thd_iface_base + TIVA_CANIF_OFFSET_CMSK);

          /* Submit request (1-indexed) */

          putreg32(*mbox + 1,
                   canmod->thd_iface_base + TIVA_CANIF_OFFSET_CRQ);

          /* Wait for response */

          while (getreg32(canmod->thd_iface_base + TIVA_CANIF_OFFSET_CRQ)
                                                    & TIVA_CANIF_CRQ_BUSY);

          /* ARB2 - upper chunk of the message ID, "direction," and extended
           * message indication. (NOTE: sadly, DIR is always 0 (receive) even
           * for RTR messages. There is no way to determine whether the RTR
           * bit was set on a received frame.
           */

          reg = getreg32(canmod->thd_iface_base + TIVA_CANIF_OFFSET_ARB2);

#ifdef CONFIG_CAN_EXTID
          msg.cm_hdr.ch_extid = (0 != (reg & TIVA_CANIF_ARB2_XTD));

          if (msg.cm_hdr.ch_extid)
            {
              msg.cm_hdr.ch_id = (reg & TIVA_CANIF_ARB2_ID_EXT_MASK)
                                      << TIVA_CANIF_ARB2_ID_EXT_PRESHIFT;

              /* ARB1 - lower chunk of the message ID for extended IDs */

              reg = getreg32(
                      canmod->thd_iface_base + TIVA_CANIF_OFFSET_ARB1);

              msg.cm_hdr.ch_id |= reg & TIVA_CANIF_ARB1_ID_EXT_MASK;
            }
          else
            {
              msg.cm_hdr.ch_id = (reg & TIVA_CANIF_ARB2_ID_STD_MASK)
                                        >> TIVA_CANIF_ARB2_ID_STD_SHIFT;
            }
#else
          msg.cm_hdr.ch_id = (reg & TIVA_CANIF_ARB2_ID_STD_MASK)
                                    >> TIVA_CANIF_ARB2_ID_STD_SHIFT;
#endif /* CONFIG_CAN_EXTID */

          /* Message Control - remote enable (RMTEN) & data length code */

          reg = getreg32(canmod->thd_iface_base + TIVA_CANIF_OFFSET_MCTL);
          msg.cm_hdr.ch_dlc = reg & TIVA_CANIF_MCTL_DLC_MASK;

          /* Data registers - these are little endian but the uint8_t
           * array in can_msg_s is big endian.
           */

          for (int j = 0; j < (msg.cm_hdr.ch_dlc + 1) / 2; ++j)
            {
              reg = getreg32(canmod->thd_iface_base
                              + TIVA_CANIF_OFFSET_DATA(j));

              msg.cm_data[j * 2]      = (reg & TIVA_CANIF_DATA_HBYTE_MASK)
                                        >> TIVA_CANIF_DATA_HBYTE_SHIFT;
              msg.cm_data[j * 2 + 1]  = (reg & TIVA_CANIF_DATA_LBYTE_MASK)
                                        >> TIVA_CANIF_DATA_LBYTE_SHIFT;
            }

#ifdef CONFIG_CAN_ERRORS
          ret = can_receive(dev, &msg.cm_hdr, msg.cm_data);
#else
          can_receive(dev, &msg.cm_hdr, msg.cm_data);
#endif
        }

        nxmutex_unlock(&canmod->thd_iface_lock);

#ifdef CONFIG_CAN_ERRORS
        if (ret == OK)
        {
          /* Switch to processing errors by interrupt when the application
           * is able to keep up with the message volume and messages are
           * being received successfully.
           */

          work_cancel(LPWORK, &canmod->error_work);
          modifyreg32(canmod->base + TIVA_CAN_OFFSET_CTL,
                      0, TIVA_CAN_CTL_SIE);
        }
#endif
    }

    return OK;
}

/****************************************************************************
 * Name: tivacan_rxintctl
 *
 * Description:
 *   The upper-half driver does not make meaningful use of this function,
 *   so there is no need to implement it. In theory it might be used so that
 *   messages are not received before the upper-half software FIFOs are
 *   initialized, but that code executes in a critical section so it
 *   doesn't make any difference anyway. And continuing to receive messages
 *   while the TX FIFO drains is not a problem.
 *
 * Input parameters:
 *   dev - An instance of the "upper half" CAN driver structure
 *   enable - True to enable the interrupt, or false to disable
 *
 * Returned value: None
 ****************************************************************************/

static void tivacan_rxintctl(struct can_dev_s *dev, bool enable)
{
}

/****************************************************************************
 * Name tivacan_txintctl
 *
 * Description:
 *   The upper-half driver does not make meaningful use of this function
 *   (at least when CONFIG_CAN_TXREADY is enabled), so there is no need to
 *   implement it.
 *
 * Input parameters:
 *   dev - An instance of the "upper half" CAN driver structure
 *   enable - True to enable the interrupt, or false to disable
 *
 * Returned value: None
 ****************************************************************************/

static void tivacan_txintctl(struct can_dev_s *dev, bool enable)
{
}

/****************************************************************************
 * Name: tivacan_ioctl
 *
 * Description:
 *   Perform the action requested by an ioctl syscall that was not handled by
 *   the "upper half" CAN driver.
 *
 *   A pointer to this function is passed to can_register.
 *
 * Input parameters:
 *   dev - An instance of the "upper half" CAN driver structure
 *   cmd - One of the CANIOC_* ioctls
 *   arg - Second argument to the ioctl call.
 *
 * Returned value:
 *   Zero on success; a negated errno on failure
 ****************************************************************************/

static int tivacan_ioctl(struct can_dev_s *dev, int cmd,
                         unsigned long arg)
{
  struct tiva_canmod_s *canmod = dev->cd_priv;
  int ret;

  switch (cmd)
    {
      case CANIOC_GET_BITTIMING:
      {
        struct canioc_bittiming_s *bt =
          (struct canioc_bittiming_s *)arg;
        struct tiva_can_timing_s timing;

        timing = tivacan_bittiming_get(dev);
        bt->bt_tseg1 = timing.tseg1;
        bt->bt_tseg2 = timing.tseg2;
        bt->bt_sjw   = timing.sjw;
        bt->bt_baud  = (SYSCLK_FREQUENCY / (unsigned long)timing.prescaler)
                        / (timing.tseg1 + timing.tseg2 + 1);
        ret = OK;
      }
      break;

      case CANIOC_SET_BITTIMING:
        {
          /* REVIST: This is VERY crude--there's no guarantees that rounding
           * won't cause problems with the prescaler selection.
           * However, NuttX's interface here is kinda sub-optimal anyway--
           * the choice of TSEG1, TSEG2, and SJW depend on the system clock
           * and the prescaler value, so there should be an ioctl for
           * the driver to take physical parameters like delay time and
           * crystal frequency tolerances into account and calculate the
           * timing parameters automagically... This could be implemented
           * in the upper half driver.
           */

          const struct canioc_bittiming_s *bt =
            (struct canioc_bittiming_s *)arg;
          struct tiva_can_timing_s timing;

          timing.tseg1 = bt->bt_tseg1;
          timing.tseg2 = bt->bt_tseg2;
          timing.sjw   = bt->bt_sjw;
          timing.prescaler = SYSCLK_FREQUENCY
            / (bt->bt_baud * (bt->bt_tseg1 + bt->bt_tseg2 + 1));
          DEBUGASSERT(timing.tseg1 <= 16 && timing.tseg1 >= 1);
          DEBUGASSERT(timing.tseg2 <= 8 && timing.tseg1 >= 1);
          DEBUGASSERT(timing.sjw <= 4 && timing.sjw >= 1);
          DEBUGASSERT(timing.prescaler <= 1024 && timing.prescaler >= 1);

          tivacan_bittiming_set(dev, &timing);
          ret = OK;
        }
        break;

      case CANIOC_GET_CONNMODES:
        {
          struct canioc_connmodes_s *modes =
                                        (struct canioc_connmodes_s *)arg;
          uint32_t reg = getreg32(canmod->base + TIVA_CAN_OFFSET_CTL);

          if (reg & TIVA_CAN_CTL_TEST)
            {
              reg = getreg32(canmod->base + TIVA_CAN_OFFSET_TST);
              modes->bm_loopback = reg & TIVA_CAN_TST_LBACK;
              modes->bm_silent   = reg & TIVA_CAN_TST_SILENT;
            }
          else
            {
              modes->bm_loopback = false;
              modes->bm_silent   = false;
            }

          ret = OK;
        }
        break;

      case CANIOC_SET_CONNMODES:
        {
          uint32_t reg;
          struct canioc_connmodes_s *modes =
                                        (struct canioc_connmodes_s *)arg;
          irqstate_t flags = enter_critical_section();

          if (modes->bm_loopback || modes->bm_silent)
            {
              modreg32(TIVA_CAN_CTL_TEST, 0,
                      canmod->base + TIVA_CAN_OFFSET_CTL);

              reg = getreg32(canmod->base + TIVA_CAN_OFFSET_TST);

              if (modes->bm_loopback)
                {
                  reg |= TIVA_CAN_TST_LBACK;
                }

              if (modes->bm_silent)
                {
                  reg |= TIVA_CAN_TST_SILENT;
                }

              putreg32(reg, canmod->base + TIVA_CAN_OFFSET_TST);
            }
          else
            {
              modreg32(0, TIVA_CAN_CTL_TEST,
                      canmod->base + TIVA_CAN_OFFSET_CTL);
            }

          leave_critical_section(flags);

          ret = OK;
        }
        break;

      case CANIOC_ADD_EXTFILTER:
      case CANIOC_ADD_STDFILTER:
        {
          int fifo_idx = tivacan_alloc_fifo(dev,
                                        CONFIG_TIVA_CAN_DEFAULT_FIFO_DEPTH);

          if (fifo_idx < 0)
            {
              /* Probably -ENOSPC or -ENOMEM */

              ret = fifo_idx;
              break;
            }

          if (cmd == CANIOC_ADD_EXTFILTER)
            {
              tivacan_initfilter(dev, &canmod->fifos[fifo_idx],
                                  (void *)arg, TIVA_CAN_FILTER_TYPE_EXT);
            }
          else
            {
              tivacan_initfilter(dev, &canmod->fifos[fifo_idx],
                                  (void *)arg, TIVA_CAN_FILTER_TYPE_STD);
            }

          if (canmod->rxdefault_fifo != NULL)
            {
              tivacan_free_fifo(dev, canmod->rxdefault_fifo);
              canmod->rxdefault_fifo = NULL;
            }

          ret = fifo_idx;
        }
        break;

      case CANIOC_DEL_EXTFILTER:
      case CANIOC_DEL_STDFILTER:
        {
          /* RX default fifo pointer removed when a filter is added. Do not
           * allow filters to be "removed" before they're added. Also do not
           * allow unusued filter FIFOs to be added.
           */

          if (arg < 0
                || arg >= CONFIG_TIVA_CAN_FILTERS_MAX
                || canmod->rxdefault_fifo != NULL
                || canmod->fifos[arg] == 0
                || arg == (unsigned long)canmod->tx_fifo
            )
            {
              ret = -EINVAL;
            }
          else
            {
              tivacan_free_fifo(dev, &canmod->fifos[arg]);
              ret = OK;
            }
        }
        break;

      case CANIOC_BUSOFF_RECOVERY:
        {
          uint32_t init = getreg32(canmod->base + TIVA_CAN_OFFSET_CTL)
                          & TIVA_CAN_CTL_INIT;
          uint32_t boff = canmod->status & TIVA_CAN_STS_BOFF;

          if (!boff && !init)
            {
              caninfo("The CAN module is not in bus-off mode.\n");
              ret = -ENMFILE;
            }
          else if (!boff && init)
            {
              canwarn("The CAN module is in INIT mode but not bus-off! "
                      "Exiting init mode.\n");
              modifyreg32(canmod->base + TIVA_CAN_OFFSET_CTL,
                          TIVA_CAN_CTL_INIT, 0);
              ret = -EIO;
            }
          else if (boff && !init)
            {
              caninfo("Already recovering from bus-off.\n");
              ret = -EINPROGRESS;
            }
          else
            {
              modifyreg32(canmod->base + TIVA_CAN_OFFSET_CTL,
                          TIVA_CAN_CTL_INIT, 0);
              ret = OK;
            }
        }
        break;

      case CANIOC_SET_NART:
        {
          modifyreg32(canmod->base + TIVA_CAN_OFFSET_CTL,
                      0, TIVA_CAN_CTL_DAR);
          ret = OK;
        }
        break;

      case CANIOC_SET_ABOM:
        {
          canmod->autorecover = true;
          ret = OK;
        }
        break;
      default:
        canerr("ERROR: Unrecognized ioctl\n");
        ret = -ENOSYS;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: tivacan_send
 *
 * Description:
 *   Enqueue a message to transmit into the hardware FIFO. Returns
 *   immediately if the FIFO is full. This function must not fail when
 *   tivacan_txready returns true, or the message will be lost.
 *
 *   A pointer to this function is passed to can_register.
 *
 * Input parameters:
 *   dev - An instance of the "upper half" CAN driver structure
 *   msg - The CAN message to enqueue
 *
 * Returned value:
 *   Zero on success; a negated errorno on failure.
 ****************************************************************************/

static int tivacan_send(struct can_dev_s *dev, struct can_msg_s *msg)
{
  struct tiva_canmod_s *canmod = dev->cd_priv;
  uint32_t reg = 0;

  /* REVISIT: Should can_txdone be called to advance the FIFO buffer
   * head when dev_send is called when it shouldn't be?
   */

  if (canmod->status & TIVA_CAN_STS_BOFF)
    {
      can_txdone(dev);
      return -ENETDOWN;
    }

  if (canmod->tx_fifo_tail >= CONFIG_TIVA_CAN_TX_FIFO_DEPTH)
    {
      can_txdone(dev);
      return -EBUSY;
    }

  nxmutex_lock(&canmod->thd_iface_lock);

  /* Protect the message object due the minute chance that the mailbox was
   * previously used for a remote frame and could receive messages, causing
   * an SRAM conflict.
   */

  tivacan_disable_mbox(dev, canmod->thd_iface_base,
                       TIVA_CAN_TX_FIFO_START + canmod->tx_fifo_tail);

  /* Command mask register */

  reg = TIVA_CANIF_CMSK_WRNRD | TIVA_CANIF_CMSK_ARB | TIVA_CANIF_CMSK_CONTROL
        | TIVA_CANIF_CMSK_CLRINTPND
        | TIVA_CANIF_CMSK_DATAA | TIVA_CANIF_CMSK_DATAB;
  putreg32(reg, canmod->thd_iface_base + TIVA_CANIF_OFFSET_CMSK);

  /* Arbitration registers (ARB1 and ARB2) */

#ifdef CONFIG_CAN_EXTID
  if (msg->cm_hdr.ch_extid)
    {
      reg = msg->cm_hdr.ch_id & TIVA_CANIF_ARB1_ID_EXT_MASK;
      putreg32(reg, canmod->thd_iface_base + TIVA_CANIF_OFFSET_ARB1);

      reg = (msg->cm_hdr.ch_id >> TIVA_CANIF_ARB2_ID_EXT_PRESHIFT)
            & TIVA_CANIF_ARB2_ID_EXT_MASK;
      reg |= TIVA_CANIF_ARB2_MSGVAL | TIVA_CANIF_ARB2_XTD;

    #ifdef CONFIG_CAN_USE_RTR
      reg |= (msg->cm_hdr.ch_rtr ? 0 : TIVA_CANIF_ARB2_DIR);
    #else
      reg |= TIVA_CANIF_ARB2_DIR;
    #endif

      putreg32(reg, canmod->thd_iface_base + TIVA_CANIF_OFFSET_ARB2);
    }
  else
    {
#endif /* CONFIG_CAN_EXTID */
      reg = (msg->cm_hdr.ch_id << TIVA_CANIF_ARB2_ID_STD_SHIFT)
            & TIVA_CANIF_ARB2_ID_STD_MASK;

      reg |= TIVA_CANIF_ARB2_MSGVAL;

    #ifdef CONFIG_CAN_USE_RTR
      reg |= (msg->cm_hdr.ch_rtr ? 0 : TIVA_CANIF_ARB2_DIR);
    #else
      reg |= TIVA_CANIF_ARB2_DIR;
    #endif

      putreg32(reg, canmod->thd_iface_base + TIVA_CANIF_OFFSET_ARB2);
#ifdef CONFIG_CAN_EXTID
    }
#endif

  /* Message Control (MCTL) register */

  reg = msg->cm_hdr.ch_dlc & TIVA_CANIF_MCTL_DLC_MASK;

  /* Protect the mailbox from rx messages; see comment in tivacan_setup */

  reg |= TIVA_CANIF_MCTL_UMASK;

  /* NOTE: Not sure whether the End Of Buffer bit means anything here */

  reg |= TIVA_CANIF_MCTL_NEWDAT | TIVA_CANIF_MCTL_TXIE
          | TIVA_CANIF_MCTL_TXRQST | TIVA_CANIF_MCTL_EOB;
  putreg32(reg, canmod->thd_iface_base + TIVA_CANIF_OFFSET_MCTL);

  /* Data registers, there are four of them. */

  for (int i = 0; i < (msg->cm_hdr.ch_dlc + 1) / 2; ++i)
    {
      reg = (uint32_t)msg->cm_data[i * 2]
              << TIVA_CANIF_DATA_HBYTE_SHIFT;
      reg |= (uint32_t)msg->cm_data[i * 2 + 1]
              << TIVA_CANIF_DATA_LBYTE_SHIFT;

      putreg32(reg, canmod->thd_iface_base + TIVA_CANIF_OFFSET_DATA(i));
    }

  /* Submit request; this register is one-indexed. */

  putreg32(TIVA_CAN_TX_FIFO_START + canmod->tx_fifo_tail + 1,
           canmod->thd_iface_base + TIVA_CANIF_OFFSET_CRQ);

  /* Wait for completion */

  while (getreg32(canmod->thd_iface_base + TIVA_CANIF_OFFSET_CRQ)
          & TIVA_CANIF_CRQ_BUSY);

  nxmutex_unlock(&canmod->thd_iface_lock);

  /* Move to the next message in the h/w TX FIFO.
   * Tell the upper-half the message has been submitted... this recurses
   * back on us until the software TX FIFO is empty.
   */

  ++canmod->tx_fifo_tail;
  can_txdone(dev);

  return OK;
}

/****************************************************************************
 * Name: tivacan_txready
 *
 * Description:
 *   Determine whether the hardware is ready to transmit another TX message.
 *   This checks for bus-off conditions as well as if the FIFO is full.
 *
 *   A pointer to this function is passed to can_register.
 *
 * Input parameters:
 *   dev - An instance of the "upper half" CAN driver structure
 *
 * Returned value:
 *   True if the hardware can accept another TX message, false otherwise.
 ****************************************************************************/

static bool tivacan_txready(struct can_dev_s *dev)
{
  struct tiva_canmod_s *canmod = dev->cd_priv;

  if (canmod->status & TIVA_CAN_STS_BOFF)
    {
      return false;
    }

  return (canmod->tx_fifo_tail < CONFIG_TIVA_CAN_TX_FIFO_DEPTH);
}

/****************************************************************************
 * Name: tivacan_txempty
 *
 * Description:
 *   Determines whether the TX FIFO is empty. Because the CAN_TXREADY
 *   interface is used, this function has a special connotation: it indicates
 *   whether can_txready is going to be called in the future to start the
 *   work queue that feeds the TX hardware FIFO.
 *
 *   A pointer to this function is passed to can_register.
 *
 * Input parameters:
 *   dev - An instance of the "upper half" CAN driver structure
 *
 * Returned value:
 *   True if the FIFO is empty, false otherwise.
 ****************************************************************************/

static bool tivacan_txempty(struct can_dev_s *dev)
{
  struct tiva_canmod_s *canmod = dev->cd_priv;

  return (canmod->tx_fifo_tail == 0);
}

/****************************************************************************
 * Name: tivacan_handle_errors_wqueue
 *
 * Description:
 *   Periodically handle errors when status interrupts are disabled due to
 *   overload. Only applicable when CAN error reporting is enabled.
 *
 *   Since error messages are processed directly in the ISR, it is easy to
 *   overwhelm the application with error messages. If tivacan_handle_errors
 *   fails to send an error message in the ISR, status interrupts are
 *   disabled and the ISR adds this function to the work queue.
 *
 *   On the other hand, when a message is successfully sent, or successfully
 *   passes through the filters and is received by the application, the ISR
 *   or the RX kthread will call work_cancel and re-enable status interrupts
 *   if they are disabled.
 *
 * Input parameters:
 *   dev - An instance of the "upper half" CAN driver structure (typed as
 *         void* for compatibility with worker_t.
 ****************************************************************************/

#ifdef CONFIG_CAN_ERRORS
void tivacan_handle_errors_wqueue(void *dev)
{
  irqstate_t flags;
  struct tiva_canmod_s *canmod = ((struct can_dev_s *)dev)->cd_priv;

  flags = enter_critical_section();
  tivacan_handle_errors((struct can_dev_s *)dev);
  work_queue(LPWORK,
             &canmod->error_work,
             tivacan_handle_errors_wqueue,
             dev,
             TIVA_CAN_ERR_HANDLER_TICKS);
  leave_critical_section(flags);
}
#endif /* CONFIG_CAN_ERRORS */

/****************************************************************************
 * Name: tivacan_handle_errors(struct can_dev_s *dev)
 *
 * Description:
 *   Process everything in the status register; i.e. initiate bus-off
 *   recovery and send error messages if desired.  This function may be
 *   called from either the main ISR or (when error messages are desired but
 *   the application is unable to handle the volume of errors) from the
 *   high-priority work queue. However, interrupts must be disabled when
 *   called from the work queue context to ensure that error messages are
 *   sent to the application in the correct order.
 *
 *   An IRQ is always raised for bus-off and error counter "warning" level,
 *   so this function may be executed from the ISR while still waiting in
 *   the work queue. In this case, the call to work_queue in the ISR
 *   will just reset the timer, which is fine.
 *
 * Input parameters:
 *   dev - An instance of the "upper half" CAN driver structure
 *
 * Returned value:
 *   1 if an error message was successfully sent to the application, the
 *   return value of can_receive() (a negated errno) if it failed, and
 *   zero if no error message was needed.
 ****************************************************************************/

int tivacan_handle_errors(struct can_dev_s *dev)
{
  uint32_t cansts;
  uint32_t canerr;
  struct tiva_canmod_s *canmod = dev->cd_priv;
  int      ret = 0;

#ifdef CONFIG_CAN_ERRORS
  int      errcnt;
  int      prev_errcnt;
  struct can_msg_s msg;

  memset(&msg, 0, sizeof(struct can_msg_s));
  msg.cm_hdr.ch_error = true;
  msg.cm_hdr.ch_dlc   = CAN_ERROR_DLC;
#endif

  /* Clear the status interrupt by reading CANSTS */

  cansts = getreg32(canmod->base + TIVA_CAN_OFFSET_STS);

  /* Get the contents of the error counter register */

  canerr = getreg32(canmod->base + TIVA_CAN_OFFSET_ERR);

  /* Bus-off handling *******************************************************/

  /* This is a new bus-off event. Proceed with error processing (if error
   * reporting is enabled) so that the cause of the "last straw" error is
   * included with the bus-off error message sent to the app.
   * Clear the INIT bit if autorecovery is enabled.
   */

  if (cansts & TIVA_CAN_STS_BOFF
      && ! (canmod->status & TIVA_CAN_STS_BOFF))
    {
      if (canmod->autorecover)
        {
          modreg32(0, TIVA_CAN_CTL_INIT,
                    canmod->base + TIVA_CAN_OFFSET_CTL);
        }

#ifdef CONFIG_CAN_ERRORS
      msg.cm_hdr.ch_id = CAN_ERROR_BUSOFF;
#endif
    }
#ifdef CONFIG_CAN_ERRORS

  /* If this an existing bus-off event and the module is recovering, don't do
   * anything. The interrupt is caused by the Last Error Code being updated
   * with a bit-0 error to indicate that the module is recovering.
   */

  else if (cansts & TIVA_CAN_STS_BOFF
            && canmod->status & TIVA_CAN_STS_BOFF)
    {
      goto save_regs_and_return;
    }

  /* The module has just finished recovering from bus-off; indicate this to
   * the app. If the LEC is a BIT0 error it must be ignored (see below).
   * We still check for other errors though because if we're running from the
   * work queue, the module could have recovered and then immediately
   * encountered another error without us being notified in between.
   */

  else if (! (cansts & TIVA_CAN_STS_BOFF)
              && canmod->status & TIVA_CAN_STS_BOFF)
    {
      msg.cm_hdr.ch_id = CAN_ERROR_RESTARTED;
    }

  /* Lost arbitration detection *********************************************/

  if (tivacan_get_txrq32(canmod->base) & ~tivacan_get_nwda32(canmod->base)
      && cansts & TIVA_CAN_STS_RXOK)
    {
      /* Received a message successfually but sending a message failed */

      msg.cm_hdr.ch_id |= CAN_ERROR_LOSTARB;
    }

  /* Electrical fault/framing errors ****************************************/

  /* This is not a bus-off event, or it is a brand-new bus-off event
   * that we need to indicate the cause of the error for.
   */

  switch (canmod->status & TIVA_CAN_STS_LEC_MASK)
    {
      case TIVA_CAN_STS_LEC_NONE:

        /* Skip unnecessary comparisons for the most common case */

        break;

      case TIVA_CAN_STS_LEC_STUFF:
        {
          msg.cm_hdr.ch_id  |= CAN_ERROR_PROTOCOL;
          msg.cm_data[2]    = CAN_ERROR2_STUFF;
        }
        break;
      case TIVA_CAN_STS_LEC_FORM:
        {
          msg.cm_hdr.ch_id  |= CAN_ERROR_PROTOCOL;
          msg.cm_data[2]    = CAN_ERROR2_FORM;
        }
        break;
      case TIVA_CAN_STS_LEC_ACK:
        {
          msg.cm_hdr.ch_id  |= CAN_ERROR_NOACK | CAN_ERROR_PROTOCOL;
          msg.cm_data[2]    = CAN_ERROR2_TX;
        }
        break;
      case TIVA_CAN_STS_LEC_BIT1:
        {
          msg.cm_hdr.ch_id  |=  CAN_ERROR_PROTOCOL;
          msg.cm_data[2]     =  CAN_ERROR2_BIT | CAN_ERROR2_BIT1
                              | CAN_ERROR2_TX;
        }
        break;
      case TIVA_CAN_STS_LEC_BIT0:
        {
          /* Note: this error code is set every time there is a sequence
           * of 11 recessive bits during bus-off recovery, the LEC is not
           * checked in that case.
           */

          msg.cm_hdr.ch_id |=  CAN_ERROR_PROTOCOL;
          msg.cm_data[2]    =  CAN_ERROR2_BIT | CAN_ERROR2_BIT0
                              | CAN_ERROR2_TX;
        }
        break;
      case TIVA_CAN_STS_LEC_CRC:
        {
          msg.cm_hdr.ch_id  |= CAN_ERROR_PROTOCOL;
          msg.cm_data[2]    = CAN_ERROR2_UNSPEC;
          msg.cm_data[3]    = CAN_ERROR3_CRCSEQ;
        }
        break;
      default:
        {
          /* Possible other LEC values:
          * TIVA_CAN_STS_LEC_NONE: normal conditions, including lost
          *                        arbitration; no action required.
          * TIVA_CAN_STS_NOEVENT:  possible if running in the work queue;
          *                        no action required.
          * Something else:        should never happen
          */
        }
    }

  /* Error counters
   * --------------
   * Rules for sending error messages to the application for error counters:
   * (1) Reaching the warning or passive level on their own don't generate
   *     error messages, but the information will be included in the error
   *     when an actual event has occurred.
   * (2) The application will be notified if the error goes away with an
   *     error message of type CAN_ERROR_RESTARTED.
   */

  errcnt = (canerr & (TIVA_CAN_ERR_RP | TIVA_CAN_ERR_REC_MASK))
            >> TIVA_CAN_ERR_REC_SHIFT;
  prev_errcnt = (canmod->errors
                & (TIVA_CAN_ERR_RP | TIVA_CAN_ERR_REC_MASK))
                >> TIVA_CAN_ERR_REC_SHIFT;

  if (errcnt >= 96)
    {
      if (errcnt >= 128)
        {
          msg.cm_data[1] |= CAN_ERROR1_RXPASSIVE;
        }
      else if (prev_errcnt >= 128)
        {
          msg.cm_hdr.ch_id |= CAN_ERROR_RESTARTED;
        }

      if (msg.cm_hdr.ch_id)
        {
          msg.cm_hdr.ch_id |= CAN_ERROR_CONTROLLER;
          msg.cm_data[1] |= CAN_ERROR1_RXWARNING;
        }
    }
  else if (prev_errcnt >= 96)
    {
      msg.cm_hdr.ch_id |= CAN_ERROR_RESTARTED;
    }

  errcnt = (canerr & TIVA_CAN_ERR_TEC_MASK) >> TIVA_CAN_ERR_TEC_SHIFT;
  prev_errcnt = (canmod->errors & TIVA_CAN_ERR_TEC_MASK)
                >> TIVA_CAN_ERR_TEC_SHIFT;

  if (errcnt >= 96)
    {
      if (errcnt >= 128)
        {
          msg.cm_data[1] |= CAN_ERROR1_TXPASSIVE;
        }
      else if (prev_errcnt >= 128)
        {
          msg.cm_hdr.ch_id |= CAN_ERROR_RESTARTED;
        }

      if (msg.cm_hdr.ch_id)
        {
          msg.cm_hdr.ch_id |= CAN_ERROR_CONTROLLER;
          msg.cm_data[1] |= CAN_ERROR1_TXWARNING;
        }
    }
  else if (prev_errcnt >= 96)
    {
      msg.cm_hdr.ch_id |= CAN_ERROR_RESTARTED;
    }

  /* Forwarding an error message to the app */

  if (msg.cm_hdr.ch_id)
    {
      /* If there was a good reason to send an error message, send it */

      ret = can_receive(dev, &msg.cm_hdr, msg.cm_data);

      /* Differentiate between "error message successfully sent" and
       * "no error message required" for the caller.
       */

      if (ret >= 0)
        {
          ret = 1;
        }
    }

  /* Reset the Last Error Code and TXOK/RXOK bits */

  modreg32(TIVA_CAN_STS_LEC_NOEVENT,
           TIVA_CAN_STS_LEC_MASK | TIVA_CAN_STS_TXOK | TIVA_CAN_STS_RXOK,
           canmod->base + TIVA_CAN_OFFSET_STS);

save_regs_and_return:
#endif /* CONFIG_CAN_ERRORS */

  /* Save contents of CANSTS and CANERR for other functions to use
   * and for comparison next time
   */

  canmod->status = cansts;
  canmod->errors = canerr;

  return ret;
}

/****************************************************************************
 * Name: tivacan_isr
 *
 * Description:
 *   ISR executed for all interrupts generated by the CAN module.
 *
 *   This ISR is registered in tivacan_setup.
 *
 * Returned value: Zero on success, negated errno if an error is encountered.
 ****************************************************************************/

static int  tivacan_isr(int irq, void *context, void *dev)
{
  struct tiva_canmod_s *canmod = ((struct can_dev_s *)dev)->cd_priv;
  uint32_t canint;
  int      ret = OK;

#ifdef CONFIG_CAN_ERRORS
  uint32_t reg;
#endif

  while ((canint = getreg32(canmod->base + TIVA_CAN_OFFSET_INT)
                                      & TIVA_CAN_INT_INTID_MASK))
    {
      if (canint == TIVA_CAN_INT_INTID_STATUS)
        {
          ret = tivacan_handle_errors((struct can_dev_s *)dev);

#ifdef CONFIG_CAN_ERRORS
          if (ret < 0)
            {
              /* We are spamming the application and it can't keep up...
               * disable status interrupts and send error messages
               * periodically.
               */

              modreg32(0, TIVA_CAN_CTL_SIE,
                       canmod->base + TIVA_CAN_OFFSET_CTL);
              work_queue(LPWORK,
                         &canmod->error_work,
                         tivacan_handle_errors_wqueue,
                         dev,
                         TIVA_CAN_ERR_HANDLER_TICKS);
            }

#endif
          continue;
        }
      else
        {
          /* Note: the datasheet incorrectly states "A message interrupt is
           * cleared by clearing the message object's INTPND bit in the
           * CANIFnMCTL register or by reading the CAN Status (CANSTS)
           * register."
           *
           * In fact, a message interrupt can only be cleared via the
           * interface registers. Here, we use the "shortcut" in the
           * command mask register to avoid a read-modify-write of MCTL.
           */

          putreg32(TIVA_CANIF_CMSK_CLRINTPND,
                   canmod->isr_iface_base + TIVA_CANIF_OFFSET_CMSK);

          /* Submit request and wait for completion */

          putreg32(canint, canmod->isr_iface_base + TIVA_CANIF_OFFSET_CRQ);
          while (getreg32(canmod->isr_iface_base + TIVA_CANIF_OFFSET_CRQ)
                  & TIVA_CANIF_CRQ_BUSY);

          if (canint >= TIVA_CAN_TX_FIFO_START + 1)
            {
              /* tx_fifo_tail contains the index of the next mailbox in the
               * hardware TX FIFO; i.e. where the next message will be put.
               * It is always one more than the last message to be added to
               * the FIFO, so even though canint is one-indexed and
               * TIVA_CAN_TX_FIFO_START is zero-indexed we do not add 1 here.
               */

              if (canint == TIVA_CAN_TX_FIFO_START + canmod->tx_fifo_tail)
                {
                  /* Only wake up the write thread when we've emptied the
                   * FIFO; the Tiva CAN module doesn't support a TX ring
                   * buffer.
                   */

                  can_txready(dev);
                  canmod->tx_fifo_tail = 0;
                }

#ifdef CONFIG_CAN_ERRORS
              reg = getreg32(canmod->base + TIVA_CAN_OFFSET_CTL);

              if (! (reg & TIVA_CAN_CTL_SIE))
                {
                  /* The application is not overwhelmed (it was able to call
                   * write()) and the message was successfully sent.
                   * Re-enable status interrupts to immediately notify of
                   * e.g. lost arbitration.
                   */

                  work_cancel(LPWORK, &canmod->error_work);
                  putreg32(reg | TIVA_CAN_CTL_SIE,
                          canmod->base + TIVA_CAN_OFFSET_CTL);
                }
#endif
            }
          else
            {
              int sval;
              nxsem_get_value(&canmod->rxsem, &sval);

              /* Unblock the message retrieval thread if blocked; signal it
               * if it needs to loop through all the mailboxes again once
               * it finishes its current iteration.
               */

              if (sval == -1
                  || (sval == 0 && canint - 1 < canmod->kthd_cur_mbox))
                {
                  nxsem_post(&canmod->rxsem);
                }
            }
        }
    }

  return ret;
}

/****************************************************************************
 * Name: tivacan_bittiming_get
 *
 * Description:
 *   Get the CAN bit timing parameters from the hardware (prescaler, TSEG1,
 *   TSEG2, and SJW). The values returned are the actual values in time
 *   quanta, not the values stored in the register (which are off by one).
 *
 *   This is a driver-internal utility function.
 *
 * Input parameters:
 *   dev - An instance of the "upper half" CAN driver structure
 *
 * Returned value:
 *   struct tiva_can_timing_s containing the four parameters listed above.
 ****************************************************************************/

static struct tiva_can_timing_s
tivacan_bittiming_get(struct can_dev_s *dev)
{
  struct tiva_can_timing_s timing;
  uint32_t canbase = ((struct tiva_canmod_s *)dev->cd_priv)->base;

  uint32_t canbit;
  uint32_t brpe;
  canbit = getreg32(canbase + TIVA_CAN_OFFSET_BIT);
  brpe   = getreg32(canbase + TIVA_CAN_OFFSET_BRPE);

  timing.prescaler = (canbit & TIVA_CAN_BIT_BRP_MASK)
                      >> TIVA_CAN_BIT_BRP_SHIFT;
  timing.prescaler |= ((brpe & TIVA_CAN_BRPE_BRPE_MASK)
                        >> TIVA_CAN_BRPE_BRPE_SHIFT)
                        << TIVA_CAN_BIT_BRP_LENGTH;

  timing.tseg1 = (canbit & TIVA_CAN_BIT_TSEG1_MASK)
                  >> TIVA_CAN_BIT_TSEG1_SHIFT;
  timing.tseg2 = (canbit & TIVA_CAN_BIT_TSEG2_MASK)
                  >> TIVA_CAN_BIT_TSEG2_SHIFT;
  timing.sjw   = (canbit & TIVA_CAN_BIT_SJW_MASK)
                  >> TIVA_CAN_BIT_SJW_SHIFT;

  /* Values stored in registers are 1 less than the values used */

  ++timing.prescaler;
  ++timing.tseg1;
  ++timing.tseg2;
  ++timing.sjw;

  return timing;
}

/****************************************************************************
 * Name: tivacan_bittiming_set
 *
 * Description:
 *   Set the CAN bit timing parameters to the hardware registers. The values
 *   used should be the actual values in time quanta, not the values stored
 *   in the registers (which are off by one).
 *
 *   This is a driver-internal utility function.
 *
 * Input parameters:
 *   dev - An instance of the "upper half" CAN driver structure
 *   timing - An instance of tiva_can_timing_s, with the prescaler, TSEG1,
 *            TSEG2, and SJW set to their real (not off-by-one) values.
 *
 * Returned value: None
 ****************************************************************************/

static void tivacan_bittiming_set(struct can_dev_s *dev,
                                  struct tiva_can_timing_s *timing)
{
  irqstate_t flags;
  uint32_t    canbit;
  uint32_t    brpe;
  uint32_t    ctl;
  bool        init_mode;
  uint32_t    canbase = ((struct tiva_canmod_s *)dev->cd_priv)->base;

  DEBUGASSERT(timing->prescaler > TIVA_CAN_PRESCALER_MIN &&
              timing->prescaler < TIVA_CAN_PRESCALER_MAX);
  DEBUGASSERT(timing->tseg1 > TIVA_CAN_TSEG1_MIN &&
              timing->tseg1 < TIVA_CAN_TSEG1_MAX);
  DEBUGASSERT(timing->tseg2 > TIVA_CAN_TSEG2_MIN &&
              timing->tseg2 < TIVA_CAN_TSEG2_MAX);
  DEBUGASSERT(timing->sjw > TIVA_CAN_SJW_MIN &&
              timing->sjw < TIVA_CAN_SJW_MAX);

  flags = enter_critical_section();

  ctl = getreg32(canbase + TIVA_CAN_OFFSET_CTL);

  /* Save the current state of the init bit */

  init_mode = ctl & TIVA_CAN_CTL_INIT;

  /* Enable writes to CANBIT and BRPE - set INIT and Configuration Change
   * Enable (CCE)
   */

  ctl |= TIVA_CAN_CTL_INIT | TIVA_CAN_CTL_CCE;
  putreg32(ctl, canbase + TIVA_CAN_OFFSET_CTL);

  canbit = getreg32(canbase + TIVA_CAN_OFFSET_BIT);
  brpe   = getreg32(canbase + TIVA_CAN_OFFSET_BRPE);

  canbit &= ~(TIVA_CAN_BIT_BRP_MASK | TIVA_CAN_BIT_TSEG1_MASK
              | TIVA_CAN_BIT_TSEG2_MASK | TIVA_CAN_BIT_SJW_MASK);

  /* Masking bits that belong in the baud rate prescaler extension register.
   * Values stored in registers are 1 less than the value used.
   */

  canbit |= ((timing->prescaler - 1) << TIVA_CAN_BIT_BRP_SHIFT)
              & TIVA_CAN_BIT_BRP_MASK;
  brpe   |= ((timing->prescaler - 1) >> TIVA_CAN_BIT_BRP_LENGTH)
                                      << TIVA_CAN_BRPE_BRPE_SHIFT;

  canbit |= (timing->tseg1 - 1) << TIVA_CAN_BIT_TSEG1_SHIFT;
  canbit |= (timing->tseg2 - 1) << TIVA_CAN_BIT_TSEG2_SHIFT;
  canbit |= (timing->sjw - 1) << TIVA_CAN_BIT_SJW_SHIFT;

  putreg32(canbit, canbase + TIVA_CAN_OFFSET_BIT);
  putreg32(brpe, canbase + TIVA_CAN_OFFSET_BRPE);

  /* Lock changes to CANBIT and BRPE (clear Configuration Change Enable) */

  ctl &= ~TIVA_CAN_CTL_CCE;

  /* Exit init mode if the baud rate was changed on-the-fly */

  if (!init_mode)
    {
      ctl &= ~TIVA_CAN_CTL_INIT;
    }

  putreg32(ctl, canbase + TIVA_CAN_OFFSET_CTL);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: tivacan_alloc_fifo
 *
 * Description:
 *   Try to allocate a FIFO of the specified depth in the CAN module SRAM.
 *   (FIFOs in the Tiva CAN modules don't need to be contiguous)
 *
 *   This is a driver-internal utility function.
 *
 * Input parameters:
 *   dev - An instance of the "upper half" CAN driver structure
 *   depth - The desired FIFO depth to allocate
 *
 * Return value: The index of a tiva_can_fifo_t in dev->cd_priv which
 *               contains a bit-field of the mailboxes that may be
 *               used, -ENOSPC if there is not enough room in the CAN module
 *               SRAM (i.e. too many FIFOs already allocated), or -ENOMEM if
 *               the limit CONFIG_TIVA_CAN_FILTERS_MAX has been reached.
 ****************************************************************************/

int tivacan_alloc_fifo(struct can_dev_s *dev, int depth)
{
  tiva_can_fifo_t claimed = 0;
  int i;
  int numclaimed = 0;
  int free_fifo_idx = -1;
  struct tiva_canmod_s *canmod = dev->cd_priv;

  nxmutex_lock(&canmod->fifo_lock);

  /* Mailboxes allocated other RX FIFOs or the TX FIFO */

  for (i = 0; i < CONFIG_TIVA_CAN_FILTERS_MAX + 1; ++i)
    {
      if (canmod->fifos[i] == 0)
        {
          free_fifo_idx = i;
        }

      claimed |= canmod->fifos[i];
    }

  if (free_fifo_idx < 0)
    {
      canwarn("Max number of filters allocated.\n");
      return -ENOMEM;
    }

  /* Mailboxes that are available to be allocated to this FIFO */

  claimed = ~claimed;

  for (i = 0; i < (TIVA_CAN_NUM_MBOXES - CONFIG_TIVA_CAN_TX_FIFO_DEPTH)
              && numclaimed < depth; ++i)
    {
      if (claimed & 1 << i)
        {
          ++numclaimed;
        }
    }

  if (numclaimed != depth)
    {
      nxmutex_unlock(&canmod->fifo_lock);
      return -ENOSPC;
    }
  else
    {
      /* Clear the bits for mailboxes we aren't claiming right now. */

      claimed &= 0xffffffff >> (32 - i);
      canmod->fifos[free_fifo_idx] = claimed;

      nxmutex_unlock(&canmod->fifo_lock);
      return free_fifo_idx;
    }
}

/****************************************************************************
 * Name: tivacan_free_fifo
 *
 * Description:
 *   Disable all the mailboxes in the specified FIFO and make them
 *   available for use by another filter. This function waits for pending
 *   transmissions to complete (for the TX FIFO) or new messages to be
 *   read (for RX filter FIFOs) before disabling a mailbox.
 *
 *   This is a driver-internal utility function.
 *
 * Input parameters:
 *   dev - An instance of the "upper half" CAN driver structure
 *   fifo - The FIFO to free
 *
 * Return value: None
 ****************************************************************************/

static void tivacan_free_fifo(struct can_dev_s *dev,
                              tiva_can_fifo_t *fifo)
{
  struct tiva_canmod_s *canmod = dev->cd_priv;
  nxmutex_lock(&canmod->thd_iface_lock);

  for (int i = 0; i < TIVA_CAN_NUM_MBOXES; ++i)
    {
      if (*fifo & 1 << i)
        {
          tivacan_disable_mbox(dev, canmod->thd_iface_base, i);

          *fifo &= ~(1 << i);
        }
    }

  nxmutex_unlock(&canmod->thd_iface_lock);
}

/****************************************************************************
 * Name: tivacan_disable_mbox
 *
 * Description:
 *   Disable a mailbox by clearing the MSGVAL bit. This function
 *   doesn't do any checks before disabling it. Don't call it without
 *   checking whether there's a pending message in the object! It takes the
 *   raw base interface register base address, so it can be called from
 *   thread or ISR context. Zeros out the entire ARB2 register.
 *
 *   This function guarantees that the mailbox is actually disabled by
 *   reading back the value of the register. As briefly mentioned in the
 *   datasheet and further explained on the TI forums [1], writes to the
 *   message RAM can be lost if the CAN peripheral is performing a
 *   read-modify-write operation (i.e. actively receiving a CAN message).
 *
 *   Yes, the thread is for the C2000 microcontrollers but the CAN peripheral
 *   appears to be very similar. However, this approach accomplishes the same
 *   thing and doesn't take the risk of missing a CAN message by setting the
 *   INIT bit as suggested in the forum.
 *
 *   This is a driver-internal utility function.
 *
 *   [1] https://e2e.ti.com/support/microcontrollers/c2000/f/171/t/916169
 *
 * Input parameters:
 *   dev - An instance of the "upper half" CAN driver structure
 *   iface_base - The interface registers base address to use
 *   num - The mailbox number to disable (zero-indexed)
 *
 * Return value: none
 ****************************************************************************/

static void tivacan_disable_mbox(struct can_dev_s *dev,
                                 uint32_t iface_base,
                                 int num)
{
  uint32_t reg;
  struct tiva_canmod_s *canmod = dev->cd_priv;

  while (true)
    {
      /* Read (back) the ARB2 register. Our write could have been lost if we
       * caught the peripheral in the middle of a read-modify-write operation
       * with a received message.
       */

      reg = tivacan_readsplitreg32(canmod->base, TIVA_CAN_OFFSET_MSG1VAL,
                                   TIVA_CAN_OFFSET_MSG2VAL);
      if (!(reg & 1 << num))
        {
          break;
        }

      /* Same as before, but select a write (WRNRD) */

      putreg32(TIVA_CANIF_CMSK_WRNRD | TIVA_CANIF_CMSK_ARB,
              iface_base + TIVA_CANIF_OFFSET_CMSK);

      /* Mark mailbox as invalid */

      reg &= ~TIVA_CANIF_ARB2_MSGVAL;
      putreg32(0, iface_base + TIVA_CANIF_OFFSET_ARB2);

      /* Submit request & wait for completion */

      putreg32(num + 1, iface_base + TIVA_CANIF_OFFSET_CRQ);
      while (getreg32(iface_base + TIVA_CANIF_OFFSET_CRQ)
              & TIVA_CANIF_CRQ_BUSY);
    }
}

/****************************************************************************
 * Name: tivacan_initfilter
 *
 * Description:
 *   Initialize the mailboxes in the specified FIFO to store messages
 *   matching the given filter. On return, the FIFO will generate interrupts
 *   on matching messages (if the CAN hardware is initialized).
 *
 *   Note that standard filters only match standard messages, but extended
 *   filters will match both standard and extended message IDs.
 *
 *   REVISIT: it might be nice to have an ioctl to control this behavior.
 *
 *   This is a driver-internal utility function.
 *
 * Input parameters:
 *   dev    - An instance of the "upper-half" CAN driver structure
 *   fifo   - A FIFO in the obtained by calling tivacan_alloc_fifo()
 *   filter - A canioc_stdfilter_s or canioc_extfilter_s
 *            of type CAN_FILTER_MASK
 *   type   - One of TIVA_CAN_FILTER_TYPE_STD or TIVA_CAN_FILTER_TYPE_EXT
 *
 * Return value: 0 on success, a negated errno on failure.
 ****************************************************************************/

static int  tivacan_initfilter(struct can_dev_s *dev,
                               tiva_can_fifo_t  *fifo,
                               void             *filter,
                               int               type)
{
  uint32_t  reg;

  /* Whether the End of Buffer bit has been set - the Tiva hardware
   * overwrites the mailbox with this bit even if there is new data in it
   * when the FIFO is full.
   */

  bool      eob_set     = false;

  struct tiva_canmod_s *canmod = dev->cd_priv;
  uint32_t  iface_base  = canmod->thd_iface_base;

  struct canioc_stdfilter_s *sfilter = NULL;
#ifdef CONFIG_CAN_EXTID
  struct canioc_extfilter_s *xfilter = NULL;
#endif

  if (type == TIVA_CAN_FILTER_TYPE_STD)
    {
      sfilter = filter;

      if (sfilter->sf_type != CAN_FILTER_MASK)
        {
          return -EINVAL;
        }
    }
#ifdef CONFIG_CAN_EXTID
  else if (type == TIVA_CAN_FILTER_TYPE_EXT)
    {
      xfilter = filter;

      if (xfilter->xf_type != CAN_FILTER_MASK)
        {
          return -EINVAL;
        }
    }
#endif
  else
    {
      canerr("Invalid filter type parameter.\n");
      return -EINVAL;
    }

  for (int i = TIVA_CAN_NUM_MBOXES - 1; i >= 0; --i)
    {
      if (*fifo & (1 << i))
        {
          /* Disable the message object to prevent message SRAM conflicts */

          tivacan_disable_mbox(dev, canmod->thd_iface_base, i);

          /* Command Mask Register
           *
           * Write to message SRAM and access mask, control, and arbitration
           * bits. Clear any interrupts (which might be spurious due to
           * the hardware being uninitialized
           */

          reg =   TIVA_CANIF_CMSK_WRNRD     | TIVA_CANIF_CMSK_MASK
                | TIVA_CANIF_CMSK_CONTROL   | TIVA_CANIF_CMSK_ARB
                | TIVA_CANIF_CMSK_CLRINTPND;
          putreg32(reg, iface_base + TIVA_CANIF_OFFSET_CMSK);

#ifdef CONFIG_CAN_EXTID
          /* Mask 1 Register - lower 16 bits of extended ID mask */

          if (type == TIVA_CAN_FILTER_TYPE_EXT)
            {
              reg = xfilter->xf_id2 & TIVA_CANIF_MSK1_IDMSK_EXT_MASK;
              putreg32(reg, iface_base + TIVA_CANIF_OFFSET_MSK1);
            }
#endif

          /* Mask 2 Register
           * Allow remote request frames through. This is done by disabling
           * filter-on-direction (MDIR = 0)
           */

#ifdef CONFIG_CAN_EXTID
          if (type == TIVA_CAN_FILTER_TYPE_EXT)
            {
              reg = (xfilter->xf_id2 >> TIVA_CANIF_MSK2_IDMSK_EXT_PRESHIFT)
                    & TIVA_CANIF_MSK2_IDMSK_EXT_MASK;
            }
          else
            {
#endif
              /* Filter out extended ID messages by default with standard
               * filters.
               */

              reg = TIVA_CANIF_MSK2_MXTD;
              reg |= sfilter->sf_id2 << TIVA_CANIF_MSK2_IDMSK_STD_SHIFT;
#ifdef CONFIG_CAN_EXTID
            }
#endif

          putreg32(reg, iface_base + TIVA_CANIF_OFFSET_MSK2);

#ifdef CONFIG_CAN_EXTID
          /* Arbitration 1 Register - lower 16 bits of extended ID */

          if (type == TIVA_CAN_FILTER_TYPE_EXT)
            {
              reg = xfilter->xf_id1 & TIVA_CANIF_ARB1_ID_EXT_MASK;
              putreg32(reg, iface_base + TIVA_CANIF_OFFSET_ARB1);
            }
#endif

          /* Arbitration 2 Register - upper extended & all standard ID bits
           * Also mark the mailbox as valid.
           */

          reg = TIVA_CANIF_ARB2_MSGVAL;

#ifdef CONFIG_CAN_EXTID
          if (type == TIVA_CAN_FILTER_TYPE_EXT)
            {
              reg |= xfilter->xf_id1 & TIVA_CANIF_ARB2_ID_EXT_MASK;
            }
          else
            {
#endif
              /* Leave the XTD bit clear to indicate that only standard
               * messages are allowed through the filter (MXTD is set).
               */

              reg |= sfilter->sf_id1 << TIVA_CANIF_ARB2_ID_STD_SHIFT;
#ifdef CONFIG_CAN_EXTID
            }
#endif

          putreg32(reg, iface_base + TIVA_CANIF_OFFSET_ARB2);

          /* Message Control Register
           * Use acceptance filter masks, enable RX interrupts for this
           * mailbox. DLC is initialized to zero but updated by received
           * frames.
           */

          reg = TIVA_CANIF_MCTL_UMASK | TIVA_CANIF_MCTL_RXIE;
          if (!eob_set)
            {
              reg |= TIVA_CANIF_MCTL_EOB;
              eob_set = true;
            }

          putreg32(reg, iface_base + TIVA_CANIF_OFFSET_MCTL);

          /* Request the registers be transferred to the message RAM and wait
           * for the transfer to complete. Mailboxes are 1-indexed in
           * hardware.
           */

          putreg32(i + 1, iface_base + TIVA_CANIF_OFFSET_CRQ);
          while (getreg32(iface_base + TIVA_CANIF_OFFSET_CRQ)
                                        & TIVA_CANIF_CRQ_BUSY);
        }
    }

  nxmutex_unlock(&canmod->thd_iface_lock);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tiva_can_initialize
 *
 * Description:
 *   Initialize the selected CAN module
 *
 * Input Parameters:
 *   Device path, a string of the form "/dev/can0" or "/dev/can1"
 *   Module number, for chips with multiple modules (typically 0 or 1)
 *
 * Returned Value:
 *   Pointer to can_dev_s (CAN device structure), or NULL on failure.
 *
 ****************************************************************************/

int tiva_can_initialize(char *devpath, int modnum)
{
  struct can_dev_s *dev;
  struct tiva_canmod_s *canmod;
  int ret;
  caninfo("tiva_can_initialize module %d\n", modnum);

#ifdef CONFIG_TIVA_CAN0
  if (modnum == 0)
    {
      dev = &g_tivacan0dev;
    }
#else
  if (modnum == 0)
    {
      return -ENODEV;
    }
#endif

#ifdef CONFIG_TIVA_CAN1
  if (modnum == 1)
    {
      dev = &g_tivacan1dev;
    }
#else
  if (modnum == 1)
    {
      return -ENODEV;
    }
#endif

  canmod = dev->cd_priv;

  /* Register the driver */

  ret = can_register(devpath, dev);
  if (ret < 0)
    {
      canerr("ERROR: can_register failed: %d\n", ret);
    }

  return ret;
}

#endif /* CONFIG_TIVA_CAN && (CONFIG_TIVA_CAN0 || CONFIG_TIVA_CAN1) */
