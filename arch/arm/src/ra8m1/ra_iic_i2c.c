/****************************************************************************
 * arch/arm/src/ra8m1/ra_iic_i2c.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/i2c/i2c_master.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"

#include "hardware/ra8m1_iic.h"
#include "hardware/ra8m1_mstp.h"
#include "hardware/ra8m1_system.h"
#include "ra_icu.h"
#include "ra_gpio.h"
#include "ra_clockconfig.h"
#include "ra_iic_i2c.h"

#ifdef CONFIG_RA_DTC
#  include <nuttx/cache.h>

#  include "hardware/ra8m1_dtc.h"
#  include "ra_dtc.h"
#endif

#if defined(CONFIG_RA_IIC0) || defined(CONFIG_RA_IIC1)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Master-only for now: slave addresses, general call, host address and
 * the ICWUR wakeup-from-standby block (fixed base, IIC0 only, unrelated to
 * the per-channel register map) are not implemented -- nothing in this
 * driver needs them yet.
 */

#define IIC_TIMEOUT_TICKS      SEC2TICK(1)  /* Per-transfer wait cap */
#define IIC_BBSY_SPIN_COUNT    1000         /* Bus-free poll iterations */

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum up_iic_phase_e
{
  IIC_PHASE_IDLE = 0,
  IIC_PHASE_ADDRESS,      /* Address+R/W byte written, waiting for TDRE/RDRF */
  IIC_PHASE_WRITE,        /* Clocking out message data bytes */
  IIC_PHASE_WRITE_DONE,   /* Last byte written, waiting for TEND */
  IIC_PHASE_READ_DUMMY,   /* Waiting for the post-address dummy RDRF */
  IIC_PHASE_READ,         /* Clocking in message data bytes */
  IIC_PHASE_STOP_PENDING, /* SP (or RS) issued, waiting for completion */
};

struct up_dev_s
{
  struct i2c_master_s dev;    /* Generic I2C interface */
  const uint32_t iicbase;     /* Base address of this channel's registers */
  uint32_t mstp;              /* Module Stop Control Register bit */
  uint8_t rxirq;              /* IICn_RXI */
  uint8_t txirq;              /* IICn_TXI */
  uint8_t teirq;              /* IICn_TEI */
  uint8_t erirq;              /* IICn_ERI */
  uint32_t frequency;         /* Last-configured bus frequency (0: unset) */

  mutex_t lock;               /* Serializes .transfer() calls */
  sem_t waitsem;              /* ISR -> transfer() completion signal */
  int result;                 /* 0, or -errno, set before waitsem is posted */

  FAR struct i2c_msg_s *msgs; /* Message array for this transfer */
  int nmsgs;                  /* Number of messages in the array */
  int msgndx;                 /* Index of the message in progress */
  FAR uint8_t *ptr;           /* Next byte position in the current message */
  int remaining;              /* Bytes left to move in the current message */
  enum up_iic_phase_e phase;  /* Current state within the message */

#ifdef CONFIG_RA_DTC
  bool dtc;                   /* true: move message data through the DTC */
  bool rxdtc_armed;           /* true: the next RXI is the DTC's own
                                * bulk-read completion, not a real byte */
  FAR uint8_t *rxdtc_buffer;  /* Start of the DTC's RX destination range */
  int rxdtc_length;           /* Its length, in bytes -- dtcrxinfo.dar/cra
                                * cannot be reused for this: the DTC writes
                                * its own incremented/decremented values
                                * back into them as it runs */
  struct dtc_transfer_info_s dtctxinfo;
  struct dtc_transfer_info_s dtcrxinfo;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int up_iic_transfer(FAR struct i2c_master_s *dev,
                            FAR struct i2c_msg_s *msgs, int count);

static int up_iic_interrupt(int irq, FAR void *context, FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct i2c_ops_s g_iic_ops =
{
  .transfer = up_iic_transfer,
};

#ifdef CONFIG_RA_IIC0
static struct up_dev_s g_iic0priv =
{
  .dev          =
  {
    .ops        = &g_iic_ops
  },

  .iicbase      = R_IIC0_BASE,
  .mstp         = R_MSTP_MSTPCRB_MSTPB9,
  .rxirq        = IIC0_RXI,
  .txirq        = IIC0_TXI,
  .teirq        = IIC0_TEI,
  .erirq        = IIC0_ERI,
#ifdef CONFIG_RA_IIC0_DTC
  .dtc          = true,
#endif
};
#endif

#ifdef CONFIG_RA_IIC1
static struct up_dev_s g_iic1priv =
{
  .dev          =
  {
    .ops        = &g_iic_ops
  },

  .iicbase      = R_IIC1_BASE,
  .mstp         = R_MSTP_MSTPCRB_MSTPB8,
  .rxirq        = IIC1_RXI,
  .txirq        = IIC1_TXI,
  .teirq        = IIC1_TEI,
  .erirq        = IIC1_ERI,
#ifdef CONFIG_RA_IIC1_DTC
  .dtc          = true,
#endif
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_iic_serialin / up_iic_serialout
 *
 * Description:
 *   All IIC registers are 8 bits wide (unlike SCI_B's mixed 8/16-bit
 *   register set), so these are thin wrappers over the 8-bit accessors.
 *
 ****************************************************************************/

static inline uint8_t up_iic_serialin(FAR struct up_dev_s *priv,
                                       uint32_t offset)
{
  return getreg8(priv->iicbase + offset);
}

static inline void up_iic_serialout(FAR struct up_dev_s *priv,
                                     uint32_t offset, uint8_t value)
{
  putreg8(value, priv->iicbase + offset);
}

/****************************************************************************
 * Name: up_iic_setfrequency
 *
 * Description:
 *   Derive ICMR1.CKS and ICBRH/ICBRL.BR[4:0] for the requested bus
 *   frequency, using formula (4) of RA8M1 User's Manual section 32.2.16
 *   (ICFER.SCLE=1, NFE=0, CKS != 000b): rate = IIC-phi / [(BRH+2)+(BRL+2)],
 *   ignoring bus rise/fall time (tr/tf are a function of external wiring,
 *   not something firmware can know; the manual's own example tables for
 *   this formula do the same).  A 50% duty split (BRH == BRL) is used.
 *
 *   Tries CKS = 1 upward (never 0: that is formula (2)/(3)'s domain, with
 *   a different "+3" offset) until (BRH+2) fits the 5-bit BR field, i.e.
 *   the smallest divider that does not overflow -- matching
 *   ra_sci_baud_ccr2()'s divider-search style for the same kind of
 *   problem.
 *
 ****************************************************************************/

static void up_iic_setfrequency(FAR struct up_dev_s *priv,
                                 uint32_t frequency)
{
  uint32_t iicphi;
  uint32_t half;
  uint8_t cks;
  uint8_t br;
  uint8_t regval;

  if (frequency == priv->frequency)
    {
      return;
    }

  for (cks = 1; cks < 8; cks++)
    {
      iicphi = RA_PCLKB_FREQUENCY >> cks;
      half   = iicphi / (2 * frequency);

      if (half >= 2 && half - 2 <= 31)
        {
          break;
        }
    }

  DEBUGASSERT(cks < 8);
  br = (uint8_t)(half - 2);

  regval  = up_iic_serialin(priv, R_IIC_ICMR1_OFFSET);
  regval &= ~(R_IIC_ICMR1_CKS_MASK << R_IIC_ICMR1_CKS_SHIFT);
  regval |= (cks << R_IIC_ICMR1_CKS_SHIFT);
  up_iic_serialout(priv, R_IIC_ICMR1_OFFSET, regval);

  up_iic_serialout(priv, R_IIC_ICBRH_OFFSET, 0xe0 | br);
  up_iic_serialout(priv, R_IIC_ICBRL_OFFSET, 0xe0 | br);

  priv->frequency = frequency;
}

/****************************************************************************
 * Name: up_iic_setup
 *
 * Description:
 *   One-time hardware bring-up: module-stop release, pin configuration,
 *   and the register sequence from RA8M1 User's Manual Figure 32.5
 *   (initial settings).  Slave address matching is left disabled
 *   (ICSER left at 0): this channel is master-only.
 *
 ****************************************************************************/

static void up_iic_setup(FAR struct up_dev_s *priv, uint32_t frequency)
{
  /* Dispatch on the channel's own base address, same reasoning as
   * ra_serial.c's up_setup(): more than one IIC channel may be enabled
   * at once, so pin selection must be chosen per-device, not per-build.
   */

#ifdef CONFIG_RA_IIC0
  if (priv->iicbase == R_IIC0_BASE)
    {
      ra_configgpio(GPIO_IIC0_SCL);
      ra_configgpio(GPIO_IIC0_SDA);
    }

#endif
#ifdef CONFIG_RA_IIC1
  if (priv->iicbase == R_IIC1_BASE)
    {
      ra_configgpio(GPIO_IIC1_SCL);
      ra_configgpio(GPIO_IIC1_SDA);
    }

#endif

  putreg16((R_SYSTEM_PRCR_S_PRKEY_V0XA5 | R_SYSTEM_PRCR_S_PRC1),
           R_SYSTEM_PRCR_S);
  modifyreg32(R_MSTP_MSTPCRB, priv->mstp, 0);
  putreg16(R_SYSTEM_PRCR_S_PRKEY_V0XA5, R_SYSTEM_PRCR_S);

  up_iic_serialout(priv, R_IIC_ICCR1_OFFSET, 0);
  up_iic_serialout(priv, R_IIC_ICCR1_OFFSET, R_IIC_ICCR1_IICRST);
  up_iic_serialout(priv, R_IIC_ICCR1_OFFSET,
                    R_IIC_ICCR1_IICRST | R_IIC_ICCR1_ICE);

  priv->frequency = 0;
  up_iic_setfrequency(priv, frequency);

  /* SCLE: SCL synchronization circuit on (needed for multi-master
   * arbitration and for formula (4)'s CKS != 0 bit-rate calculation to
   * apply).  MALE/NALE/SALE: arbitration-lost detection in every mode
   * this driver can enter.  NACKE: a NACK during data transmission
   * suspends the transfer automatically, instead of continuing to clock
   * out the rest of the message into a device that has already given up.
   */

  up_iic_serialout(priv, R_IIC_ICFER_OFFSET,
                    R_IIC_ICFER_SCLE | R_IIC_ICFER_MALE |
                    R_IIC_ICFER_NALE | R_IIC_ICFER_SALE |
                    R_IIC_ICFER_NACKE);

  /* RIE/TIE/TEIE for the normal per-byte flow; NAKIE/ALIE/SPIE so the
   * shared ERI vector actually fires for the three abort/completion
   * conditions this driver checks for (NACK, arbitration-lost, stop
   * detected).  TMOIE (bus-hang timeout) and STIE (start detected) are
   * not needed by anything this driver does.
   */

  up_iic_serialout(priv, R_IIC_ICIER_OFFSET,
                    R_IIC_ICIER_RIE | R_IIC_ICIER_TIE | R_IIC_ICIER_TEIE |
                    R_IIC_ICIER_NAKIE | R_IIC_ICIER_ALIE |
                    R_IIC_ICIER_SPIE);

  up_iic_serialout(priv, R_IIC_ICCR1_OFFSET, R_IIC_ICCR1_ICE);
}

/****************************************************************************
 * Name: up_iic_abort
 *
 * Description:
 *   Record the failure, request a stop condition (if the bus is still
 *   owned by this master), and move to IIC_PHASE_STOP_PENDING so the ERI
 *   handler's STOP detection posts waitsem once the bus is actually idle
 *   again, instead of leaving it mid-transaction for the next .transfer()
 *   call to trip over.
 *
 ****************************************************************************/

static void up_iic_abort(FAR struct up_dev_s *priv, int result)
{
  uint8_t regval = up_iic_serialin(priv, R_IIC_ICCR2_OFFSET);

  priv->result = result;

#ifdef CONFIG_RA_DTC
  if (priv->dtc)
    {
      /* Stop any DTC run this abort cuts short, or a stray activation
       * from leftover bus activity could still land in this message's
       * buffer after .transfer() has already returned.
       */

      ra_dtc_disable(priv->txirq - RA_IRQ_FIRST);
      ra_dtc_disable(priv->rxirq - RA_IRQ_FIRST);
      priv->rxdtc_armed = false;
    }
#endif

  if ((regval & R_IIC_ICCR2_MST) != 0)
    {
      up_iic_serialout(priv, R_IIC_ICCR2_OFFSET, R_IIC_ICCR2_SP);
    }

  priv->phase = IIC_PHASE_STOP_PENDING;
}

#ifdef CONFIG_RA_DTC
/****************************************************************************
 * Name: up_iic_dma_txarm
 *
 * Description:
 *   Arm the DTC to move priv->remaining data bytes from priv->ptr into
 *   ICDRT, one per TXI, with a single CPU interrupt once the count is
 *   exhausted (normal mode, MRB.DISEL=0) instead of one per byte.  Called
 *   right after the address byte is written, so the DTC is already armed
 *   before that byte's own TDRE-done edge requests the first data byte --
 *   see up_iic_write_address().
 *
 ****************************************************************************/

static void up_iic_dma_txarm(FAR struct up_dev_s *priv)
{
  struct dtc_transfer_info_s *info = &priv->dtctxinfo;
  uint8_t vector = priv->txirq - RA_IRQ_FIRST;
  uintptr_t buffer = (uintptr_t)priv->ptr;

  DEBUGASSERT(priv->remaining > 0 && priv->remaining <= 65536);

  /* The DTC reads SRAM directly: push out whatever the CPU has cached
   * for this range (i2ctool's register-address byte, in particular, is
   * a stack variable the CPU just wrote), or the DTC could send stale
   * data instead.
   */

  up_clean_dcache(buffer, buffer + priv->remaining);

  info->mra = R_DTC_MRA_MD_NORMAL | R_DTC_MRA_SZ_BYTE |
              R_DTC_MRA_SM_INCREMENT;
  info->mrb = R_DTC_MRB_DM_FIXED;
  info->sar = (uint32_t)buffer;
  info->dar = priv->iicbase + R_IIC_ICDRT_OFFSET;
  info->cra = (uint16_t)priv->remaining;
  info->crb = 0;  /* Unused in normal mode */

  ra_dtc_configure(vector, info);
  ra_dtc_enable(vector);
}

/****************************************************************************
 * Name: up_iic_dma_rxarm
 *
 * Description:
 *   Arm the DTC to move priv->remaining - 2 data bytes from ICDRR into
 *   priv->ptr, one per RXI, with a single CPU interrupt once exhausted.
 *   Stops two bytes short of the message end and leaves priv->remaining
 *   at 2 so the existing WAIT/ACKBT handling in up_iic_rxinterrupt() takes
 *   over for the last two bytes unchanged -- the DTC cannot touch ICMR3,
 *   which that sequencing needs.  Only called when priv->remaining > 2.
 *
 ****************************************************************************/

static void up_iic_dma_rxarm(FAR struct up_dev_s *priv)
{
  struct dtc_transfer_info_s *info = &priv->dtcrxinfo;
  uint8_t vector = priv->rxirq - RA_IRQ_FIRST;
  uintptr_t buffer = (uintptr_t)priv->ptr;
  int nbytes = priv->remaining - 2;

  DEBUGASSERT(nbytes > 0 && nbytes <= 65536);

  /* The DTC writes destination bytes directly to SRAM, bypassing the
   * CPU cache: invalidate up front so a stale cache line is not read
   * back later instead of what the DTC wrote (mirrors ra_serial.c's RX
   * DTC comment on priv->dtcrxbyte).  priv->ptr/nbytes are saved to
   * rxdtc_buffer/rxdtc_length now because dtcrxinfo.dar/cra get
   * overwritten with the DTC's own incremented/decremented values as it
   * runs -- they cannot be read back afterwards for the invalidate this
   * triggers in up_iic_rxinterrupt().
   */

  up_invalidate_dcache(buffer, buffer + nbytes);
  priv->rxdtc_buffer = priv->ptr;
  priv->rxdtc_length = nbytes;

  info->mra = R_DTC_MRA_MD_NORMAL | R_DTC_MRA_SZ_BYTE | R_DTC_MRA_SM_FIXED;
  info->mrb = R_DTC_MRB_DM_INCREMENT;
  info->sar = priv->iicbase + R_IIC_ICDRR_OFFSET;
  info->dar = (uint32_t)buffer;
  info->cra = (uint16_t)nbytes;
  info->crb = 0;  /* Unused in normal mode */

  priv->ptr       += nbytes;
  priv->remaining  = 2;
  priv->rxdtc_armed = true;

  ra_dtc_configure(vector, info);
  ra_dtc_enable(vector);
}
#endif

/****************************************************************************
 * Name: up_iic_write_address
 *
 * Description:
 *   Write the address+R/W byte to ICDRT and set the phase for what
 *   follows it (write data, or the post-address dummy read).
 *
 ****************************************************************************/

static void up_iic_write_address(FAR struct up_dev_s *priv,
                                  FAR struct i2c_msg_s *msg)
{
  bool tenbit = (msg->flags & I2C_M_TEN) != 0;
  bool read   = (msg->flags & I2C_M_READ) != 0;

  DEBUGASSERT(!tenbit);  /* 10-bit addressing: not implemented yet */

  priv->phase = read ? IIC_PHASE_READ_DUMMY : IIC_PHASE_WRITE;

#ifdef CONFIG_RA_DTC
  if (!read && priv->dtc && priv->remaining > 0)
    {
      up_iic_dma_txarm(priv);
    }
#endif

  up_iic_serialout(priv, R_IIC_ICDRT_OFFSET,
                    (uint8_t)((msg->addr << 1) | (read ? 1 : 0)));
}

/****************************************************************************
 * Name: up_iic_nextmsg
 *
 * Description:
 *   Point priv->ptr/remaining at message priv->msgndx and kick off its
 *   start condition (fresh START for the first message, or when the
 *   previous message did not set I2C_M_NOSTOP; RESTART otherwise).  For
 *   a fresh START, the address+R/W byte is written from
 *   up_iic_txinterrupt() once TDRE confirms the start condition
 *   completed (section 32.3.3/32.3.4 step 2-3).  For a RESTART, this
 *   function writes it directly after polling ICSR2.START instead (see
 *   the comment below) -- TDRE is not a usable signal for that case.
 *
 ****************************************************************************/

static void up_iic_nextmsg(FAR struct up_dev_s *priv, bool restart)
{
  FAR struct i2c_msg_s *msg = &priv->msgs[priv->msgndx];
  uint8_t regval;
  int i;

  priv->ptr       = msg->buffer;
  priv->remaining = msg->length;
  priv->phase     = IIC_PHASE_ADDRESS;

  if (restart)
    {
      /* RA8M1 User's Manual section 32.11.2 step 5 / Figure 32.46: clear
       * ICSR2.START (still 1 from the start/restart that began this
       * message) before requesting the next one, or the hardware's own
       * start-issuance consistency check misfires as a false
       * arbitration-lost on the restart that follows.
       */

      regval = up_iic_serialin(priv, R_IIC_ICSR2_OFFSET);
      up_iic_serialout(priv, R_IIC_ICSR2_OFFSET,
                        regval & ~R_IIC_ICSR2_START);

      up_iic_serialout(priv, R_IIC_ICCR2_OFFSET, R_IIC_ICCR2_RS);

      /* Manual step 7: write the address only after ICSR2.START is
       * re-set, confirming the restart condition itself has completed.
       * A TXI/TDRE edge fires sooner than that -- the address byte was
       * going out while the restart was still in progress on the bus,
       * corrupting it (reproduced at full speed; masked by anything that
       * delayed the write, such as a debugger breakpoint on TXI).
       */

      for (i = 0; i < IIC_BBSY_SPIN_COUNT; i++)
        {
          regval = up_iic_serialin(priv, R_IIC_ICSR2_OFFSET);
          if ((regval & R_IIC_ICSR2_START) != 0)
            {
              break;
            }
        }

      if ((regval & R_IIC_ICSR2_START) == 0)
        {
          up_iic_abort(priv, -ETIMEDOUT);
          return;
        }

      up_iic_write_address(priv, msg);
    }
  else
    {
      /* Bus-free poll: BBSY clears almost immediately once idle (section
       * 32.3.3 step 2); this is the one place in the driver that spins
       * rather than waits on an interrupt, since there is no "bus became
       * free" event to wait on cheaply.  No critical section needed: the
       * caller already holds priv->lock, and the ISRs this could race
       * with are not "live" for this message until the ST/RS write below.
       */

      for (i = 0; i < IIC_BBSY_SPIN_COUNT; i++)
        {
          regval = up_iic_serialin(priv, R_IIC_ICCR2_OFFSET);
          if ((regval & R_IIC_ICCR2_BBSY) == 0)
            {
              break;
            }
        }

      up_iic_serialout(priv, R_IIC_ICCR2_OFFSET, R_IIC_ICCR2_ST);
    }
}

/****************************************************************************
 * Name: up_iic_rxinterrupt
 *
 * Description:
 *   RDRF handling for master receive, implementing the WAIT/ACKBT timing
 *   from RA8M1 User's Manual section 32.3.4 steps 4-7 (Figure 32.10/32.11)
 *   uniformly for 1, 2, or 3+ byte reads, driven entirely by
 *   priv->remaining:
 *
 *   - The byte that is about to make remaining reach 1 (i.e. the
 *     second-to-last byte, or the only byte when the message is 1 byte
 *     long) gets WAIT armed before it is read (2+ byte case) or ACKBT
 *     armed right after the dummy read (1-byte case): either way, NACK
 *     is in effect before the final byte's own ack phase, not after.
 *   - The final byte (remaining == 1 going in) gets SP requested before
 *     it is read, so reading it both retrieves the data and releases the
 *     WAIT hold into the stop sequence.
 *
 ****************************************************************************/

static void up_iic_rxinterrupt(FAR struct up_dev_s *priv)
{
  uint8_t regval;

  if (priv->phase == IIC_PHASE_READ_DUMMY)
    {
#ifdef CONFIG_RA_DTC
      if (priv->dtc && priv->remaining > 2)
        {
          /* Right after the address handshake, before the dummy read
           * itself (same ordering as the TX arm in
           * up_iic_write_address()): hand the bulk middle of the
           * message to the DTC, leaving the last two bytes for the
           * WAIT/ACKBT handling below -- see up_iic_dma_rxarm().  This
           * also sets priv->remaining to 2.
           */

          up_iic_dma_rxarm(priv);
        }
#endif

      up_iic_serialin(priv, R_IIC_ICDRR_OFFSET);  /* Dummy read: starts SCL */
      priv->phase = IIC_PHASE_READ;

      if (priv->remaining == 1)
        {
          /* Single-byte message: no second-to-last byte exists to arm
           * NACK during -- arm it now, right after the dummy read,
           * before the one real byte's own ack phase.
           */

          regval  = up_iic_serialin(priv, R_IIC_ICMR3_OFFSET);
          regval |= R_IIC_ICMR3_ACKBT;
          up_iic_serialout(priv, R_IIC_ICMR3_OFFSET, regval);
        }

      return;
    }

#ifdef CONFIG_RA_DTC
  if (priv->rxdtc_armed)
    {
      /* The DTC moved the last byte of its bulk run already; this RXI
       * is its count-exhausted completion, not a new byte.  remaining
       * is already 2, set by up_iic_dma_rxarm().
       */

      priv->rxdtc_armed = false;

      up_invalidate_dcache((uintptr_t)priv->rxdtc_buffer,
                            (uintptr_t)priv->rxdtc_buffer +
                            priv->rxdtc_length);

      /* The real second-to-last byte's RDRF can already be pending: the
       * ICU drops a repeat occurrence of an event while its IR flag is
       * still 1 (RA8M1 User's Manual Table 13.5 note 2), so if it rose
       * while this completion's IR was still set, no further RXI will
       * ever come to report it.  Fall through to the normal remaining
       * == 2 handling below instead of waiting for an interrupt that
       * may never arrive.
       */

      if ((up_iic_serialin(priv, R_IIC_ICSR2_OFFSET) &
           R_IIC_ICSR2_RDRF) == 0)
        {
          return;
        }
    }
#endif

  if (priv->remaining == 1)
    {
      /* Final byte: request stop before reading it, so this read both
       * retrieves the data and releases the WAIT hold from below.
       */

      up_iic_serialout(priv, R_IIC_ICCR2_OFFSET, R_IIC_ICCR2_SP);
      *priv->ptr++ = up_iic_serialin(priv, R_IIC_ICDRR_OFFSET);
      priv->remaining = 0;

      regval  = up_iic_serialin(priv, R_IIC_ICMR3_OFFSET);
      regval &= ~R_IIC_ICMR3_WAIT;
      up_iic_serialout(priv, R_IIC_ICMR3_OFFSET, regval);

      priv->phase = IIC_PHASE_STOP_PENDING;
      return;
    }

  if (priv->remaining == 2)
    {
      /* About to read the second-to-last byte: hold the clock after its
       * ack phase so ACKBT below is in effect before the final byte's.
       */

      regval  = up_iic_serialin(priv, R_IIC_ICMR3_OFFSET);
      regval |= R_IIC_ICMR3_WAIT;
      up_iic_serialout(priv, R_IIC_ICMR3_OFFSET, regval);
    }

  *priv->ptr++ = up_iic_serialin(priv, R_IIC_ICDRR_OFFSET);
  priv->remaining--;

  if (priv->remaining == 1)
    {
      regval  = up_iic_serialin(priv, R_IIC_ICMR3_OFFSET);
      regval |= R_IIC_ICMR3_ACKBT;
      up_iic_serialout(priv, R_IIC_ICMR3_OFFSET, regval);
    }
}

/****************************************************************************
 * Name: up_iic_txinterrupt
 *
 * Description:
 *   TDRE handling: writes the address+R/W byte once (IIC_PHASE_ADDRESS),
 *   then message data bytes for a write (IIC_PHASE_WRITE), per RA8M1
 *   User's Manual section 32.3.3 steps 3-4 / 32.3.4 step 3.  For a read,
 *   nothing more happens here after the address byte -- the hardware
 *   flips to receive mode on its own and the next event is RDRF, not
 *   another TDRE.
 *
 ****************************************************************************/

static void up_iic_txinterrupt(FAR struct up_dev_s *priv)
{
  FAR struct i2c_msg_s *msg = &priv->msgs[priv->msgndx];

  if (priv->phase == IIC_PHASE_ADDRESS)
    {
      up_iic_write_address(priv, msg);
      return;
    }

  if (priv->phase == IIC_PHASE_WRITE)
    {
#ifdef CONFIG_RA_DTC
      if (priv->dtc && priv->remaining > 0 &&
          (getreg32(R_ICU_IELSR(priv->txirq - RA_IRQ_FIRST)) &
           R_ICU_IELSR_DTCE) == 0)
        {
          /* The DTC moved every data byte already and cleared its own
           * DTCE on exhaustion: this TXI is that completion, not a
           * request for another byte -- see up_iic_dma_txarm().  The
           * DTCE check also guards against the restart path's known
           * early TXI (see up_iic_nextmsg()), which this driver never
           * issues while remaining > 0, but would otherwise be
           * misread as a completion here too.
           */

          priv->remaining = 0;
          priv->phase = IIC_PHASE_WRITE_DONE;
          return;
        }
#endif

      if (priv->remaining > 0)
        {
          up_iic_serialout(priv, R_IIC_ICDRT_OFFSET, *priv->ptr++);
          priv->remaining--;
        }

      if (priv->remaining == 0)
        {
          priv->phase = IIC_PHASE_WRITE_DONE;
        }
    }
}

/****************************************************************************
 * Name: up_iic_teinterrupt
 *
 * Description:
 *   TEND handling: the last written byte has actually finished shifting
 *   out (not just left ICDRT), so it is now safe to request a stop or
 *   restart -- RA8M1 User's Manual section 32.3.3 step 5.
 *
 ****************************************************************************/

static void up_iic_teinterrupt(FAR struct up_dev_s *priv)
{
  FAR struct i2c_msg_s *msg;

  if (priv->phase != IIC_PHASE_WRITE_DONE)
    {
      return;
    }

  msg = &priv->msgs[priv->msgndx];

  if ((msg->flags & I2C_M_NOSTOP) != 0 && priv->msgndx + 1 < priv->nmsgs)
    {
      priv->msgndx++;
      up_iic_nextmsg(priv, true);
    }
  else
    {
      up_iic_serialout(priv, R_IIC_ICCR2_OFFSET, R_IIC_ICCR2_SP);
      priv->phase = IIC_PHASE_STOP_PENDING;
    }
}

/****************************************************************************
 * Name: up_iic_erinterrupt
 *
 * Description:
 *   Shared error/event vector: NACK and arbitration-lost both abort the
 *   transfer; stop-condition detection finalizes it (successfully, or
 *   following an abort -- either way the bus is idle again once this
 *   fires).  A read's own completion also funnels through here, since
 *   up_iic_rxinterrupt() only requests the stop, it does not wait for it.
 *
 ****************************************************************************/

static int up_iic_erinterrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)arg;
  uint8_t regval;

  /* IELSRn.IR must be cleared by software or the NVIC re-enters this
   * handler indefinitely (RA8M1 User's Manual section 13.5.1) -- missing
   * this call is what produced an endless stream of ERI entries reading
   * ICSR2 as 0x00 after the first real event.
   */

  ra_clear_ir(irq);

  regval = up_iic_serialin(priv, R_IIC_ICSR2_OFFSET);

  if ((regval & R_IIC_ICSR2_NACKF) != 0 &&
      priv->phase != IIC_PHASE_STOP_PENDING)
    {
      up_iic_abort(priv, -ENXIO);
    }
  else if ((regval & R_IIC_ICSR2_AL) != 0 &&
           priv->phase != IIC_PHASE_STOP_PENDING)
    {
      up_iic_abort(priv, -EAGAIN);
    }

  /* These flags are "write 0 to clear, write 1 is ignored" (section
   * 32.2.10 note 1): a plain read-AND-write leaves every other bit,
   * including the read-only TDRE, exactly as read.
   */

  if ((regval & R_IIC_ICSR2_STOP) != 0)
    {
      up_iic_serialout(priv, R_IIC_ICSR2_OFFSET,
                        regval & ~(R_IIC_ICSR2_STOP | R_IIC_ICSR2_NACKF |
                                   R_IIC_ICSR2_AL));
      priv->phase = IIC_PHASE_IDLE;
      nxsem_post(&priv->waitsem);
      return OK;
    }

  up_iic_serialout(priv, R_IIC_ICSR2_OFFSET,
                    regval & ~(R_IIC_ICSR2_NACKF | R_IIC_ICSR2_AL));
  return OK;
}

/****************************************************************************
 * Name: up_iic_interrupt
 *
 * Description:
 *   Common entry point for the RXI/TXI/TEI vectors (ERI has its own,
 *   above, since it is the only one that can complete or fail a transfer
 *   outright rather than just advance it).
 *
 ****************************************************************************/

static int up_iic_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)arg;

  ra_clear_ir(irq);

  if (irq == priv->rxirq)
    {
      up_iic_rxinterrupt(priv);
    }
  else if (irq == priv->txirq)
    {
      up_iic_txinterrupt(priv);
    }
  else
    {
      DEBUGASSERT(irq == priv->teirq);
      up_iic_teinterrupt(priv);
    }

  return OK;
}

/****************************************************************************
 * Name: up_iic_transfer
 *
 * Description:
 *   The .transfer() entry point: serializes callers, arms the first
 *   message, and blocks until the ERI handler reports completion (success
 *   or abort) via waitsem.  10-bit addressing and I2C_M_NOSTART are not
 *   implemented yet -- every message gets its own fresh address byte.
 *
 ****************************************************************************/

static int up_iic_transfer(FAR struct i2c_master_s *dev,
                            FAR struct i2c_msg_s *msgs, int count)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev;
  int ret;

  DEBUGASSERT(count > 0);

  nxmutex_lock(&priv->lock);

  up_iic_setfrequency(priv, msgs[0].frequency);

  priv->msgs   = msgs;
  priv->nmsgs  = count;
  priv->msgndx = 0;
  priv->result = OK;

  up_iic_nextmsg(priv, false);

  ret = nxsem_tickwait(&priv->waitsem, IIC_TIMEOUT_TICKS);
  if (ret < 0)
    {
      up_iic_abort(priv, -ETIMEDOUT);
      ret = -ETIMEDOUT;
    }
  else
    {
      ret = priv->result;
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_iic_i2cbus_initialize
 *
 * Description:
 *   Initialize an IIC channel as the generic NuttX I2C master interface.
 *
 ****************************************************************************/

FAR struct i2c_master_s *ra_iic_i2cbus_initialize(int bus)
{
  FAR struct up_dev_s *priv;

  switch (bus)
    {
#ifdef CONFIG_RA_IIC0
      case 0:
        priv = &g_iic0priv;
        break;
#endif
#ifdef CONFIG_RA_IIC1
      case 1:
        priv = &g_iic1priv;
        break;
#endif
      default:
        return NULL;
    }

  nxmutex_init(&priv->lock);
  nxsem_init(&priv->waitsem, 0, 0);

  up_iic_setup(priv, 100000);  /* Standard-mode default; .transfer() can
                                 * raise this per message's own frequency.
                                 */

  irq_attach(priv->rxirq, up_iic_interrupt, priv);
  irq_attach(priv->txirq, up_iic_interrupt, priv);
  irq_attach(priv->teirq, up_iic_interrupt, priv);
  irq_attach(priv->erirq, up_iic_erinterrupt, priv);

  up_enable_irq(priv->rxirq);
  up_enable_irq(priv->txirq);
  up_enable_irq(priv->teirq);
  up_enable_irq(priv->erirq);

  return &priv->dev;
}
#endif /* CONFIG_RA_IIC0 || CONFIG_RA_IIC1 */
