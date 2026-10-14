/****************************************************************************
 * arch/arm/src/ra8m1/ra_sci_i2c.c
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

/* I2C master over an SCI_B channel's Simple IIC mode (RA8M1 User's Manual
 * section 31.8), mutually exclusive on a given channel with that same
 * channel's UART mode (ra_serial.c) -- CONFIG_RA_SCIn_I2C vs
 * CONFIG_RA_SCIn_UART both drive the same physical peripheral instance and
 * the same RXDn/TXDn pins, just in a different CCR3.MOD.
 *
 * Unlike the dedicated IIC peripheral (ra_iic_i2c.c), Simple IIC mode has
 * no arbitration-lost detection and no separate NACK interrupt: ACK/NACK
 * (ISR.IICACKR) is polled inside the TXI handler, which is the only
 * per-byte vector this mode uses for BOTH directions -- in Simple IIC
 * mode SCIn_TXI fires once per complete frame, not just "TDR empty", so a
 * read's bytes are also collected there (RDR is read, then a dummy byte
 * is written back to TDR to keep generating SCL for the next frame).
 * RDR/RSR are double-buffered (section 31.2.2), so the byte is already
 * safely latched in RDR by TXI (9th-bit) time regardless of whether
 * CCR0.RIE is set -- RIE only gates whether the CPU is interrupted about
 * the 8th-bit RSR->RDR transfer, not whether that transfer (or the clock
 * generation driving it) happens at all.  Confirmed against the Renesas
 * FSP r_sci_b_i2c driver, which reads RDR inside its own TXI handler the
 * same way and never enables SCIn_RXI outside of DTC use.
 * The start/restart/stop-condition-complete interrupt (STIn) is aliased
 * onto the existing SCIn_TEI vector in this mode (manual page 1197,
 * CCR0.TEIE bit description) -- no new IRQ wiring is needed beyond what
 * UART mode already attaches.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
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

#include "hardware/ra8m1_sci.h"
#include "hardware/ra8m1_mstp.h"
#include "hardware/ra8m1_system.h"
#include "ra_icu.h"
#include "ra_gpio.h"
#include "ra_clockconfig.h"
#include "ra_sci_i2c.h"

#ifdef CONFIG_RA_DTC
#  include <nuttx/cache.h>

#  include "hardware/ra8m1_dtc.h"
#  include "ra_dtc.h"
#endif

#if defined(CONFIG_RA_SCI0_I2C) || defined(CONFIG_RA_SCI1_I2C) || \
    defined(CONFIG_RA_SCI2_I2C) || defined(CONFIG_RA_SCI3_I2C) || \
    defined(CONFIG_RA_SCI4_I2C) || defined(CONFIG_RA_SCI9_I2C)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Master-only, 7-bit addressing only: nothing in this driver implements
 * slave mode or 10-bit addressing yet.
 */

#define SCII2C_TIMEOUT_TICKS    SEC2TICK(1)  /* Per-transfer wait cap */
#define SCII2C_CESR_SPIN_COUNT  1000         /* CCR0 TE/RE sync-delay poll */

/* TDR bits 31:8 (TSYNC, MPBT, and the rest of the upper three bytes) are
 * documented "read as 1, write value should be 1" (section 31.2.3) for
 * the bits this driver does not use -- only TDAT[7:0] actually carries
 * data for Simple IIC mode's fixed 8-bit frames (bit 8 of TDAT is only
 * meaningful in 9-bit mode).  Every TDR write ORs this in, matching the
 * Renesas FSP r_sci_b_i2c driver's own SCI_B_I2C_PRV_DATA_REG_MASK
 * (0xFFFFFF00), applied unconditionally on every TDR write there too.
 */

#define SCII2C_TDR_RESERVED     0xffffff00

/* ICR.IICDL (SDA output delay): the manual (section 31.8.3) requires this
 * delay to exceed the SCLn pin's own fall time (300 ns for IIC normal/fast
 * mode) or SDAn may start changing before SCLn has actually gone low.
 * IICDL=0 ("no delay") would violate that; 1 ("0 to 1 cycle") is the
 * smallest setting that does not, and is used unconditionally here rather
 * than computed per frequency.
 */

#define SCII2C_IICDL_DEFAULT    1

/* ICR.NFCS (noise filter clock select): 000b is explicitly prohibited in
 * Simple IIC mode (section 31.2.6); 010b (on-chip baud rate generator
 * clock divided by 2) is a reasonable default in the 001b-100b range the
 * manual allows.
 */

#define SCII2C_NFCS_DEFAULT     2

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum up_scii2c_phase_e
{
  SCII2C_PHASE_IDLE = 0,
  SCII2C_PHASE_ADDRESS,  /* Start/restart requested, waiting for its STIn */
  SCII2C_PHASE_DATA,     /* Clocking message data, one byte per TXI */
};

struct up_dev_s
{
  struct i2c_master_s dev;    /* Generic I2C interface */
  const uint32_t scibase;     /* Base address of this channel's registers */
  uint32_t mstp;              /* Module Stop Control Register bit */
  uint8_t rxirq;              /* SCIn_RXI -- only ever fires when dtc is
                                * set (RIE is otherwise never enabled) */
  uint8_t txirq;              /* SCIn_TXI */
  uint8_t teirq;              /* SCIn_TEI (STIn in this mode) */
  uint32_t frequency;         /* Last-configured bus frequency (0: unset) */

  mutex_t lock;               /* Serializes .transfer() calls */
  sem_t waitsem;              /* ISR -> transfer() completion signal */
  int result;                 /* 0, or -errno, set before waitsem is posted */

  FAR struct i2c_msg_s *msgs; /* Message array for this transfer */
  int nmsgs;                  /* Number of messages in the array */
  int msgndx;                 /* Index of the message in progress */
  FAR uint8_t *ptr;           /* Next byte position in the current message */
  int remaining;              /* Bytes left to move in the current message */
  bool read;                  /* true: current message is a read */
  bool dummyread;             /* true: next TXI is the address-ack frame,
                                * not a real data byte */
  bool restart;               /* true: a restart (not a stop) is in
                                * flight to begin the next message */
  enum up_scii2c_phase_e phase;

#ifdef CONFIG_RA_DTC
  bool dtc;                   /* true: move message data through the DTC */
  bool rxdtc_armed;           /* true: the next RXI is the RX-DTC's own
                                * bulk-read completion, not a byte this
                                * driver needs to act on */
  bool txdtc_armed;           /* true: the next TXI is the TX-DTC's own
                                * dummy-byte-feed completion (read only --
                                * a write's own completion is instead
                                * disambiguated via remaining>0 && DTCE==0,
                                * see up_scii2c_txinterrupt()) */
  FAR uint8_t *rxdtc_buffer;  /* Start of the RX-DTC's destination range */
  int rxdtc_length;           /* Its length, in bytes -- dtcrxinfo.dar/cra
                                * cannot be reused for this: the DTC writes
                                * its own incremented/decremented values
                                * back into them as it runs */
  uint8_t dtc_dummy;          /* Fixed source byte for the TX-DTC's
                                * dummy-byte feed during a read */
  struct dtc_transfer_info_s dtctxinfo;
  struct dtc_transfer_info_s dtcrxinfo;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int up_scii2c_transfer(FAR struct i2c_master_s *dev,
                               FAR struct i2c_msg_s *msgs, int count);

static int up_scii2c_interrupt(int irq, FAR void *context, FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct i2c_ops_s g_scii2c_ops =
{
  .transfer = up_scii2c_transfer,
};

#ifdef CONFIG_RA_SCI0_I2C
static struct up_dev_s g_sci0priv =
{
  .dev          =
  {
    .ops        = &g_scii2c_ops
  },

  .scibase      = R_SCI_B0_BASE,
  .mstp         = R_MSTP_MSTPCRB_MSTPB31,
  .rxirq        = SCI0_RXI,
  .txirq        = SCI0_TXI,
  .teirq        = SCI0_TEI,
#ifdef CONFIG_RA_SCI0_DTC
  .dtc          = true,
#endif
};
#endif

#ifdef CONFIG_RA_SCI1_I2C
static struct up_dev_s g_sci1priv =
{
  .dev          =
  {
    .ops        = &g_scii2c_ops
  },

  .scibase      = R_SCI_B1_BASE,
  .mstp         = R_MSTP_MSTPCRB_MSTPB30,
  .rxirq        = SCI1_RXI,
  .txirq        = SCI1_TXI,
  .teirq        = SCI1_TEI,
#ifdef CONFIG_RA_SCI1_DTC
  .dtc          = true,
#endif
};
#endif

#ifdef CONFIG_RA_SCI2_I2C
static struct up_dev_s g_sci2priv =
{
  .dev          =
  {
    .ops        = &g_scii2c_ops
  },

  .scibase      = R_SCI_B2_BASE,
  .mstp         = R_MSTP_MSTPCRB_MSTPB29,
  .rxirq        = SCI2_RXI,
  .txirq        = SCI2_TXI,
  .teirq        = SCI2_TEI,
#ifdef CONFIG_RA_SCI2_DTC
  .dtc          = true,
#endif
};
#endif

#ifdef CONFIG_RA_SCI3_I2C
static struct up_dev_s g_sci3priv =
{
  .dev          =
  {
    .ops        = &g_scii2c_ops
  },

  .scibase      = R_SCI_B3_BASE,
  .mstp         = R_MSTP_MSTPCRB_MSTPB28,
  .rxirq        = SCI3_RXI,
  .txirq        = SCI3_TXI,
  .teirq        = SCI3_TEI,
#ifdef CONFIG_RA_SCI3_DTC
  .dtc          = true,
#endif
};
#endif

#ifdef CONFIG_RA_SCI4_I2C
static struct up_dev_s g_sci4priv =
{
  .dev          =
  {
    .ops        = &g_scii2c_ops
  },

  .scibase      = R_SCI_B4_BASE,
  .mstp         = R_MSTP_MSTPCRB_MSTPB27,
  .rxirq        = SCI4_RXI,
  .txirq        = SCI4_TXI,
  .teirq        = SCI4_TEI,
#ifdef CONFIG_RA_SCI4_DTC
  .dtc          = true,
#endif
};
#endif

#ifdef CONFIG_RA_SCI9_I2C
static struct up_dev_s g_sci9priv =
{
  .dev          =
  {
    .ops        = &g_scii2c_ops
  },

  .scibase      = R_SCI_B9_BASE,
  .mstp         = R_MSTP_MSTPCRB_MSTPB22,
  .rxirq        = SCI9_RXI,
  .txirq        = SCI9_TXI,
  .teirq        = SCI9_TEI,
#ifdef CONFIG_RA_SCI9_DTC
  .dtc          = true,
#endif
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_scii2c_serialin / up_scii2c_serialout
 ****************************************************************************/

static inline uint32_t up_scii2c_serialin(FAR struct up_dev_s *priv,
                                           uint32_t offset)
{
  return getreg32(priv->scibase + offset);
}

static inline void up_scii2c_serialout(FAR struct up_dev_s *priv,
                                        uint32_t offset, uint32_t value)
{
  putreg32(value, priv->scibase + offset);
}

/****************************************************************************
 * Name: up_scii2c_configgpio
 *
 * Description:
 *   ra_configgpio() for an SCI pin, with the N-channel open-drain bit
 *   added: Simple IIC mode needs the SCLn/SDAn pins driven open-drain
 *   with external pull-ups (RA8M1 User's Manual Table 31.41 step 7),
 *   unlike UART mode's normal push-pull use of the same pins -- so this
 *   cannot just be baked into the shared GPIO_SCIn_RX/TX macros.
 *
 ****************************************************************************/

static void up_scii2c_configgpio(gpio_pinset_t pinset)
{
  pinset.cfg |= R_PFS_NCODR;
  ra_configgpio(pinset);
}

/****************************************************************************
 * Name: up_scii2c_cesr_wait
 *
 * Description:
 *   Spin until CESR.TIST/RIST read back "expected" under "mask": RA8M1
 *   User's Manual's note on register access when TCLK is slower than the
 *   bus clock (PCLK) -- a CCR0 TE/RE write is not reflected in CESR
 *   instantly, and the FSP reference driver polls this after every such
 *   write rather than assume BPEN=1 (synchronizer bypass) is in effect.
 *
 ****************************************************************************/

static void up_scii2c_cesr_wait(FAR struct up_dev_s *priv, uint8_t mask,
                                 uint8_t expected)
{
  int i;

  for (i = 0; i < SCII2C_CESR_SPIN_COUNT; i++)
    {
      if ((up_scii2c_serialin(priv, R_SCI_B_CESR_OFFSET) & mask) ==
          expected)
        {
          break;
        }
    }
}

/****************************************************************************
 * Name: up_scii2c_setfrequency
 *
 * Description:
 *   Derive CCR2.CKS and CCR2.BRR for the requested bus frequency, using
 *   the Simple IIC row of RA8M1 User's Manual Table 31.7:
 *   N = TCLK / (64 x 2^(2n-1) x B) - 1, searching n (CKS) from 0 upward
 *   (divisor 32, 128, 512, 2048) for the first one that keeps N in BRR's
 *   8-bit range -- the same smallest-divider-first style as
 *   ra_iic_i2c.c's up_iic_setfrequency() for the dedicated IIC
 *   peripheral's own, different, formula.
 *
 ****************************************************************************/

static void up_scii2c_setfrequency(FAR struct up_dev_s *priv,
                                    uint32_t frequency)
{
  static const uint32_t divisor[4] =
  {
    32, 128, 512, 2048
  };

  uint32_t raw;
  uint32_t denom;
  uint8_t cks;
  uint8_t brr = 0;

  if (frequency == priv->frequency)
    {
      return;
    }

  for (cks = 0; cks < 4; cks++)
    {
      /* Round to nearest, not truncate: N+1 = TCLK/(divisor*B) is rarely
       * exact, and rounding here before the "- 1" below matters -- e.g.
       * TCLK=120MHz, B=100k, cks=0 gives 37.5 exactly, which must round
       * up to N=37 (confirmed against the manual's own worked example,
       * Table 31.20) or this comes out N=36, one low.
       */

      denom = divisor[cks] * frequency;
      raw   = (RA_SCICLK_FREQUENCY + denom / 2) / denom;
      if (raw >= 1 && raw <= 256)
        {
          brr = (uint8_t)(raw - 1);
          break;
        }
    }

  DEBUGASSERT(cks < 4);

  up_scii2c_serialout(priv, R_SCI_B_CCR2_OFFSET,
                       ((uint32_t)cks << R_SCI_B_CCR2_CKS_SHIFT) |
                       ((uint32_t)brr << R_SCI_B_CCR2_BRR_SHIFT));

  priv->frequency = frequency;
}

/****************************************************************************
 * Name: up_scii2c_setup
 *
 * Description:
 *   One-time hardware bring-up, following RA8M1 User's Manual Table 31.41
 *   ("Example flow of SCI initialization in simple IIC mode"), confirmed
 *   step-for-step against the Renesas FSP r_sci_b_i2c driver's
 *   sci_b_i2c_open_hw_master().  CCR0 (TE/RE/TIE/TEIE) is left at 0 here
 *   -- it is enabled per-transfer by up_scii2c_transfer(), not once at
 *   setup, matching that same reference driver.
 *
 ****************************************************************************/

static void up_scii2c_setup(FAR struct up_dev_s *priv, uint32_t frequency)
{
  uint32_t icr;

#ifdef CONFIG_RA_SCI0_I2C
  if (priv->scibase == R_SCI_B0_BASE)
    {
      up_scii2c_configgpio(GPIO_SCI0_SCL);
      up_scii2c_configgpio(GPIO_SCI0_SDA);
    }

#endif
#ifdef CONFIG_RA_SCI1_I2C
  if (priv->scibase == R_SCI_B1_BASE)
    {
      up_scii2c_configgpio(GPIO_SCI1_SCL);
      up_scii2c_configgpio(GPIO_SCI1_SDA);
    }

#endif
#ifdef CONFIG_RA_SCI2_I2C
  if (priv->scibase == R_SCI_B2_BASE)
    {
      up_scii2c_configgpio(GPIO_SCI2_SCL);
      up_scii2c_configgpio(GPIO_SCI2_SDA);
    }

#endif
#ifdef CONFIG_RA_SCI3_I2C
  if (priv->scibase == R_SCI_B3_BASE)
    {
      up_scii2c_configgpio(GPIO_SCI3_SCL);
      up_scii2c_configgpio(GPIO_SCI3_SDA);
    }

#endif
#ifdef CONFIG_RA_SCI4_I2C
  if (priv->scibase == R_SCI_B4_BASE)
    {
      up_scii2c_configgpio(GPIO_SCI4_SCL);
      up_scii2c_configgpio(GPIO_SCI4_SDA);
    }

#endif
#ifdef CONFIG_RA_SCI9_I2C
  if (priv->scibase == R_SCI_B9_BASE)
    {
      up_scii2c_configgpio(GPIO_SCI9_SCL);
      up_scii2c_configgpio(GPIO_SCI9_SDA);
    }

#endif

  putreg16((R_SYSTEM_PRCR_S_PRKEY_V0XA5 | R_SYSTEM_PRCR_S_PRC1),
           R_SYSTEM_PRCR_S);
  modifyreg32(R_MSTP_MSTPCRB, priv->mstp, 0);
  putreg16(R_SYSTEM_PRCR_S_PRKEY_V0XA5, R_SYSTEM_PRCR_S);

  up_scii2c_serialout(priv, R_SCI_B_CCR0_OFFSET, 0);
  up_scii2c_cesr_wait(priv, R_SCI_B_CESR_TIST | R_SCI_B_CESR_RIST, 0);

  /* ICR: SDA/SCL high-impedance (idle bus release), SDA output delay,
   * reception/transmission interrupt mode (not ACK/NACK interrupt mode --
   * NACK is polled via ISR.IICACKR from the TXI handler instead, see the
   * file header comment), clock synchronization, and default-ACK
   * transmission.
   */

  icr  = R_SCI_B_ICR_IICSDAS_V11 | R_SCI_B_ICR_IICSCLS_V11;
  icr |= (uint32_t)SCII2C_IICDL_DEFAULT << R_SCI_B_ICR_IICDL_SHIFT;
  icr |= R_SCI_B_ICR_IICINTM | R_SCI_B_ICR_IICCSC | R_SCI_B_ICR_IICACKT;
  up_scii2c_serialout(priv, R_SCI_B_ICR_OFFSET, icr);

  /* CCR3: 8-bit characters, MSB-first (LSBF/SINV left at their 0 reset
   * value, both required 0 in Simple IIC mode), Simple IIC mode.  BPEN
   * (synchronizer bypass) stays off, matching ra_serial.c's own choice
   * for this same peripheral in UART mode -- see its CCR2 comment.
   */

  up_scii2c_serialout(priv, R_SCI_B_CCR3_OFFSET,
                       R_SCI_B_CCR3_CHR_V10 | R_SCI_B_CCR3_MOD_SIMPLE_I2C);

  priv->frequency = 0;
  up_scii2c_setfrequency(priv, frequency);

  up_scii2c_serialout(priv, R_SCI_B_CCR1_OFFSET,
                       ((uint32_t)SCII2C_NFCS_DEFAULT <<
                        R_SCI_B_CCR1_NFCS_SHIFT) | R_SCI_B_CCR1_NFEN);

  up_scii2c_serialout(priv, R_SCI_B_CFCLR_OFFSET,
                       R_SCI_B_CFCLR_RDRFC | R_SCI_B_CFCLR_FERC |
                       R_SCI_B_CFCLR_PERC | R_SCI_B_CFCLR_MFFC |
                       R_SCI_B_CFCLR_ORERC | R_SCI_B_CFCLR_DPERC |
                       R_SCI_B_CFCLR_DFERC | R_SCI_B_CFCLR_DCMFC |
                       R_SCI_B_CFCLR_ERSC | R_SCI_B_CFCLR_TDREC);

  up_scii2c_serialout(priv, R_SCI_B_ICFCLR_OFFSET,
                       R_SCI_B_ICFCLR_IICSTIFC);
}

/****************************************************************************
 * Name: up_scii2c_release_condition
 *
 * Description:
 *   Return ICR's IICSDAS/IICSCLS fields to "serial data/clock output"
 *   (00b): the normal state while actually clocking address/data bytes,
 *   as opposed to the 01b ("generate a condition") value a start,
 *   restart, or stop request is issued with.
 *
 ****************************************************************************/

static void up_scii2c_release_condition(FAR struct up_dev_s *priv)
{
  uint32_t icr = up_scii2c_serialin(priv, R_SCI_B_ICR_OFFSET);

  icr &= ~((R_SCI_B_ICR_IICSDAS_MASK << R_SCI_B_ICR_IICSDAS_SHIFT) |
           (R_SCI_B_ICR_IICSCLS_MASK << R_SCI_B_ICR_IICSCLS_SHIFT));
  up_scii2c_serialout(priv, R_SCI_B_ICR_OFFSET, icr);
}

/****************************************************************************
 * Name: up_scii2c_request_condition
 *
 * Description:
 *   Request a start, restart, or stop condition: RA8M1 User's Manual
 *   section 31.8.1 and ICR register notes 3-4 require IICSTAREQ /
 *   IICRSTAREQ / IICSTPREQ to be set together with IICSDAS/IICSCLS = 01b
 *   in the same write, with no more than one of the three REQ bits set,
 *   and only once ISR.IICSTIF already reads 0 for whatever condition
 *   completed last -- guaranteed here since up_scii2c_interrupt() clears
 *   it before this is ever called again (see its own comment).
 *
 ****************************************************************************/

static void up_scii2c_request_condition(FAR struct up_dev_s *priv,
                                         uint32_t req)
{
  uint32_t icr = up_scii2c_serialin(priv, R_SCI_B_ICR_OFFSET);

  icr &= ~((R_SCI_B_ICR_IICSDAS_MASK << R_SCI_B_ICR_IICSDAS_SHIFT) |
           (R_SCI_B_ICR_IICSCLS_MASK << R_SCI_B_ICR_IICSCLS_SHIFT));
  icr |= R_SCI_B_ICR_IICSDAS_V01 | R_SCI_B_ICR_IICSCLS_V01 | req;
  up_scii2c_serialout(priv, R_SCI_B_ICR_OFFSET, icr);
}

#ifdef CONFIG_RA_DTC
/****************************************************************************
 * Name: up_scii2c_dma_txarm
 *
 * Description:
 *   Arm the DTC to move priv->remaining data bytes from priv->ptr into
 *   TDR, one per TXI, with a single CPU interrupt once the count is
 *   exhausted (normal mode, MRB.DISEL=0) instead of one per byte.  Called
 *   from up_scii2c_send_address() for a write, before the address byte is
 *   written, so the DTC is already armed before that byte's own 9th-bit
 *   edge requests the first data byte.  Per RA8M1 User's Manual Table
 *   31.46 note 1, ACK/NACK cannot be confirmed for any byte the DTC
 *   moves -- only the final one, at completion.
 *
 ****************************************************************************/

static void up_scii2c_dma_txarm(FAR struct up_dev_s *priv)
{
  struct dtc_transfer_info_s *info = &priv->dtctxinfo;
  uint8_t vector = priv->txirq - RA_IRQ_FIRST;
  uintptr_t buffer = (uintptr_t)priv->ptr;

  DEBUGASSERT(priv->remaining > 0 && priv->remaining <= 65536);

  up_clean_dcache(buffer, buffer + priv->remaining);

  info->mra = R_DTC_MRA_MD_NORMAL | R_DTC_MRA_SZ_BYTE |
              R_DTC_MRA_SM_INCREMENT;
  info->mrb = R_DTC_MRB_DM_FIXED;
  info->sar = (uint32_t)buffer;
  info->dar = priv->scibase + R_SCI_B_TDR_BY_LL_OFFSET;
  info->cra = (uint16_t)priv->remaining;
  info->crb = 0;  /* Unused in normal mode */

  ra_dtc_configure(vector, info);
  ra_dtc_enable(vector);
}

/****************************************************************************
 * Name: up_scii2c_dma_rxarm
 *
 * Description:
 *   Arm the DTC to move priv->remaining - 2 data bytes, stopping two
 *   bytes short of the message end so the existing ICR.IICACKT timing in
 *   up_scii2c_txinterrupt() still handles the last two unchanged -- the
 *   DTC cannot touch ICR itself.  Unlike the dedicated IIC peripheral's
 *   single-vector RX DTC (ra_iic_i2c.c), Simple IIC mode needs two
 *   vectors running together for this: one on RXI moves each received
 *   byte from RDR into the message buffer, a second on TXI keeps
 *   rewriting a fixed dummy byte into TDR, since nothing else sustains
 *   SCL between received bytes here (Table 31.46 has no "clock enable"
 *   separate from a TDR write).  Only called when priv->remaining > 2.
 *
 ****************************************************************************/

static void up_scii2c_dma_rxarm(FAR struct up_dev_s *priv)
{
  struct dtc_transfer_info_s *txinfo = &priv->dtctxinfo;
  struct dtc_transfer_info_s *rxinfo = &priv->dtcrxinfo;
  uint8_t txvector = priv->txirq - RA_IRQ_FIRST;
  uint8_t rxvector = priv->rxirq - RA_IRQ_FIRST;
  uintptr_t buffer = (uintptr_t)priv->ptr;
  uintptr_t dummy = (uintptr_t)&priv->dtc_dummy;
  int nbytes = priv->remaining - 2;

  DEBUGASSERT(nbytes > 0 && nbytes <= 65536);

  /* RXI (hence RX-DTC activation) is gated on CCR0.RIE (page 1197,
   * Table 31.46), which has stayed 0 since up_scii2c_transfer() -- turn
   * it on now, as a full CCR0 write, not a read-modify-write.  Section
   * 31.2.5 note 1 literally reads the other way (once TE or RE is 1,
   * only 0 may be written to them), which this write does not honor --
   * done this way regardless because the Renesas FSP r_sci_b_i2c driver
   * makes this exact same full CCR0 write, with TE/RE already 1, on its
   * own DTC-enable path.  If RIE does not actually take, this write is
   * the first thing to suspect.
   *
   * Dummy-read RDR first: the address frame already left RDRF set (the
   * auto RSR->RDR transfer happens regardless of RIE, see the file
   * header comment), and if the RXI condition this gates is level- not
   * edge-sensitive, turning RIE on here could otherwise fire the
   * RX-DTC immediately on that stale condition, shifting every real
   * byte into the buffer one position early.
   */

  up_scii2c_serialin(priv, R_SCI_B_RDR_OFFSET);
  up_scii2c_serialout(priv, R_SCI_B_CCR0_OFFSET,
                       R_SCI_B_CCR0_TE | R_SCI_B_CCR0_RE |
                       R_SCI_B_CCR0_TEIE | R_SCI_B_CCR0_TIE |
                       R_SCI_B_CCR0_RIE);

  /* The DTC writes destination bytes directly to SRAM, bypassing the CPU
   * cache: invalidate up front so a stale cache line is not read back
   * later instead of what the DTC wrote, and again in
   * up_scii2c_rxinterrupt() once the run completes, in case an unrelated
   * access re-cached a stale line for an overlapping address during the
   * window in between (mirrors ra_iic_i2c.c's RX DTC).
   */

  up_invalidate_dcache(buffer, buffer + nbytes);
  priv->rxdtc_buffer = priv->ptr;
  priv->rxdtc_length = nbytes;

  rxinfo->mra = R_DTC_MRA_MD_NORMAL | R_DTC_MRA_SZ_BYTE | R_DTC_MRA_SM_FIXED;
  rxinfo->mrb = R_DTC_MRB_DM_INCREMENT;
  rxinfo->sar = priv->scibase + R_SCI_B_RDR_OFFSET;
  rxinfo->dar = (uint32_t)buffer;
  rxinfo->cra = (uint16_t)nbytes;
  rxinfo->crb = 0;  /* Unused in normal mode */

  priv->dtc_dummy = 0xff;
  up_clean_dcache(dummy, dummy + sizeof(priv->dtc_dummy));

  txinfo->mra = R_DTC_MRA_MD_NORMAL | R_DTC_MRA_SZ_BYTE | R_DTC_MRA_SM_FIXED;
  txinfo->mrb = R_DTC_MRB_DM_FIXED;
  txinfo->sar = (uint32_t)dummy;
  txinfo->dar = priv->scibase + R_SCI_B_TDR_BY_LL_OFFSET;
  txinfo->cra = (uint16_t)nbytes;
  txinfo->crb = 0;  /* Unused in normal mode */

  priv->ptr         += nbytes;
  priv->remaining    = 2;
  priv->rxdtc_armed  = true;
  priv->txdtc_armed  = true;

  ra_dtc_configure(rxvector, rxinfo);
  ra_dtc_configure(txvector, txinfo);
  ra_dtc_enable(rxvector);
  ra_dtc_enable(txvector);
}
#endif

/****************************************************************************
 * Name: up_scii2c_send_address
 *
 * Description:
 *   Point priv->ptr/remaining/read at message priv->msgndx and write its
 *   address+R/W byte to TDR -- called once a start or restart condition
 *   for this message has completed (up_scii2c_teinterrupt()).
 *
 ****************************************************************************/

static void up_scii2c_send_address(FAR struct up_dev_s *priv)
{
  FAR struct i2c_msg_s *msg = &priv->msgs[priv->msgndx];
  bool tenbit = (msg->flags & I2C_M_TEN) != 0;

  DEBUGASSERT(!tenbit);  /* 10-bit addressing: not implemented yet */

  up_scii2c_release_condition(priv);

  priv->ptr       = msg->buffer;
  priv->remaining = msg->length;
  priv->read      = (msg->flags & I2C_M_READ) != 0;
  priv->dummyread = priv->read;
  priv->phase     = SCII2C_PHASE_DATA;

#ifdef CONFIG_RA_DTC
  if (!priv->read && priv->dtc && priv->remaining > 0)
    {
      up_scii2c_dma_txarm(priv);
    }
#endif

  up_scii2c_serialout(priv, R_SCI_B_TDR_OFFSET,
                       SCII2C_TDR_RESERVED |
                       (uint32_t)((msg->addr << 1) |
                                  (priv->read ? 1 : 0)));
}

/****************************************************************************
 * Name: up_scii2c_nack_abort
 *
 * Description:
 *   A NACK ended the transfer early: record the failure and request a
 *   stop condition, same as the normal end-of-message path
 *   (up_scii2c_issue_next_condition()) would, so the completion still
 *   comes through up_scii2c_teinterrupt() once the bus is idle again --
 *   its final-stop handling does not need to distinguish this from a
 *   successful completion; priv->result already carries the difference.
 *
 ****************************************************************************/

static void up_scii2c_nack_abort(FAR struct up_dev_s *priv, int result)
{
  priv->result    = result;
  priv->remaining = 0;
  priv->restart   = false;

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
      priv->txdtc_armed = false;
    }
#endif

  up_scii2c_request_condition(priv, R_SCI_B_ICR_IICSTPREQ);
}

/****************************************************************************
 * Name: up_scii2c_force_abort
 *
 * Description:
 *   Used only by up_scii2c_transfer()'s own timeout fallback: the normal
 *   completion interrupt may never come, so disable the channel directly
 *   instead of requesting a stop condition and waiting for its STIn.
 *
 ****************************************************************************/

static void up_scii2c_force_abort(FAR struct up_dev_s *priv, int result)
{
  priv->result = result;

#ifdef CONFIG_RA_DTC
  if (priv->dtc)
    {
      ra_dtc_disable(priv->txirq - RA_IRQ_FIRST);
      ra_dtc_disable(priv->rxirq - RA_IRQ_FIRST);
      priv->rxdtc_armed = false;
      priv->txdtc_armed = false;
    }
#endif

  up_scii2c_serialout(priv, R_SCI_B_CCR0_OFFSET, 0);
  up_scii2c_cesr_wait(priv, R_SCI_B_CESR_TIST | R_SCI_B_CESR_RIST, 0);

  up_scii2c_serialout(priv, R_SCI_B_ICR_OFFSET,
                       up_scii2c_serialin(priv, R_SCI_B_ICR_OFFSET) |
                       R_SCI_B_ICR_IICSDAS_V11 | R_SCI_B_ICR_IICSCLS_V11);

  priv->phase = SCII2C_PHASE_IDLE;
}

/****************************************************************************
 * Name: up_scii2c_issue_next_condition
 *
 * Description:
 *   The current message's data phase is done: request a restart (if
 *   I2C_M_NOSTOP and another message follows) or a stop (otherwise).
 *
 ****************************************************************************/

static void up_scii2c_issue_next_condition(FAR struct up_dev_s *priv)
{
  FAR struct i2c_msg_s *msg = &priv->msgs[priv->msgndx];

  if ((msg->flags & I2C_M_NOSTOP) != 0 && priv->msgndx + 1 < priv->nmsgs)
    {
      priv->restart = true;
      up_scii2c_request_condition(priv, R_SCI_B_ICR_IICRSTAREQ);
    }
  else
    {
      up_scii2c_request_condition(priv, R_SCI_B_ICR_IICSTPREQ);
    }
}

#ifdef CONFIG_RA_DTC
/****************************************************************************
 * Name: up_scii2c_rxinterrupt
 *
 * Description:
 *   RXI handling.  CCR0.RIE is only ever turned on for the duration of an
 *   RX-DTC bulk run (up_scii2c_dma_rxarm()), so every genuine call here
 *   while one is in flight is that run's own count-exhausted completion
 *   -- the data is already safely in the message buffer, moved there by
 *   the DTC itself.  RIE stays on afterward through the last two,
 *   CPU-driven bytes (read via TXI instead, see up_scii2c_txinterrupt()),
 *   so this also fires harmlessly for those; with rxdtc_armed already
 *   false by then, there is nothing to do.
 *
 ****************************************************************************/

static void up_scii2c_rxinterrupt(FAR struct up_dev_s *priv)
{
  if (!priv->rxdtc_armed)
    {
      return;
    }

  priv->rxdtc_armed = false;

  up_invalidate_dcache((uintptr_t)priv->rxdtc_buffer,
                        (uintptr_t)priv->rxdtc_buffer +
                        priv->rxdtc_length);
}
#endif

/****************************************************************************
 * Name: up_scii2c_txinterrupt
 *
 * Description:
 *   TXI handling: one call per completed frame (address or data, in
 *   either direction -- see the file header comment).  ISR.IICACKR polled
 *   here is the only ACK/NACK visibility this mode provides.
 *
 ****************************************************************************/

static void up_scii2c_txinterrupt(FAR struct up_dev_s *priv)
{
  uint32_t isr = up_scii2c_serialin(priv, R_SCI_B_ISR_OFFSET);
  uint32_t icr;

  if ((isr & R_SCI_B_ISR_IICACKR) != 0 && (!priv->read || priv->dummyread))
    {
      /* NACK on the address byte, or on a write's data byte: a real bus
       * error, not the master's own deliberate NACK on a read's last
       * byte (that case falls through below instead).
       */

      up_scii2c_nack_abort(priv, -ENXIO);
      return;
    }

  if (!priv->read)
    {
#ifdef CONFIG_RA_DTC
      if (priv->dtc && priv->remaining > 0 &&
          (getreg32(R_ICU_IELSR(priv->txirq - RA_IRQ_FIRST)) &
           R_ICU_IELSR_DTCE) == 0)
        {
          /* TX-DTC moved every data byte already and cleared its own
           * DTCE on exhaustion: this TXI is that completion, not a
           * request for another byte -- see up_scii2c_dma_txarm().  The
           * DTCE check guards against misreading a stray early TXI as a
           * completion, matching ra_iic_i2c.c's own TX disambiguation.
           *
           * In DTC normal mode the completion interrupt fires on the
           * same activation that performed the final transfer -- i.e.
           * the one that just put the last data byte into TDR, not the
           * one where it finishes shifting onto the bus (same timing
           * ra_serial.c's own TX DTC documents).  Requesting a stop or
           * restart condition right now would abandon that byte before
           * it is ever sent: just mark done and return without touching
           * ICR.  The next, genuine TXI -- that same last byte's own
           * completion -- then takes the remaining == 0 path below
           * exactly as a non-DTC write's last byte would, and its
           * ISR.IICACKR is still checked by the NACK test above this
           * block.
           */

          priv->remaining = 0;
          return;
        }
#endif

      if (priv->remaining > 0)
        {
          up_scii2c_serialout(priv, R_SCI_B_TDR_OFFSET,
                               SCII2C_TDR_RESERVED | *priv->ptr++);
          priv->remaining--;
        }
      else
        {
          up_scii2c_issue_next_condition(priv);
        }

      return;
    }

#ifdef CONFIG_RA_DTC
  if (priv->txdtc_armed)
    {
      /* The TX-DTC's dummy-byte-feed run just exhausted.  Same timing
       * as the write-completion case above: this activation already
       * wrote the dummy byte that triggers the next frame (the first
       * CPU-driven tail byte) into TDR -- that trigger has already
       * happened, so falling through to the common tail below would
       * write a second, extra dummy byte on top of a frame already
       * queued.  Just clear the flag and return: priv->remaining is
       * already 2 (set by up_scii2c_dma_rxarm()), dummyread is already
       * false, so the next genuine TXI -- the tail byte's own
       * completion -- correctly reaches the plain RDR-read branch
       * below instead.  (The RX-DTC completion for this same
       * underlying byte is handled separately, and harmlessly first,
       * in up_scii2c_rxinterrupt().)
       */

      priv->txdtc_armed = false;
      return;
    }
#endif

  if (priv->dummyread)
    {
      /* First TXI after the address ACK: no real data arrived with it,
       * this just confirms the address and arms ACK (not NACK) for what
       * follows, unless this is a 1-byte read -- RA8M1 User's Manual
       * Figure 31.96 steps 4-5.
       */

      priv->dummyread = false;

      if (priv->remaining > 1)
        {
          icr  = up_scii2c_serialin(priv, R_SCI_B_ICR_OFFSET);
          icr &= ~R_SCI_B_ICR_IICACKT;
          up_scii2c_serialout(priv, R_SCI_B_ICR_OFFSET, icr);
        }

#ifdef CONFIG_RA_DTC
      if (priv->dtc && priv->remaining > 2)
        {
          up_scii2c_dma_rxarm(priv);
        }
#endif
    }
  else
    {
      *priv->ptr++ = (uint8_t)up_scii2c_serialin(priv, R_SCI_B_RDR_OFFSET);
      priv->remaining--;

      if (priv->remaining == 0)
        {
          up_scii2c_issue_next_condition(priv);
          return;
        }
    }

  if (priv->remaining == 1)
    {
      /* Arm NACK now so it is in effect for the upcoming last byte's own
       * ack phase, not after.
       */

      icr  = up_scii2c_serialin(priv, R_SCI_B_ICR_OFFSET);
      icr |= R_SCI_B_ICR_IICACKT;
      up_scii2c_serialout(priv, R_SCI_B_ICR_OFFSET, icr);
    }

  /* A byte must be written to TDR to keep generating SCL for the next
   * frame, whether or not there is real data left to receive.
   */

  up_scii2c_serialout(priv, R_SCI_B_TDR_OFFSET, SCII2C_TDR_RESERVED | 0xff);
}

/****************************************************************************
 * Name: up_scii2c_teinterrupt
 *
 * Description:
 *   STIn handling (aliased onto the TEI vector in this mode): a start,
 *   restart, or stop condition has just completed.
 *
 ****************************************************************************/

static void up_scii2c_teinterrupt(FAR struct up_dev_s *priv)
{
  /* ICFCLR.IICSTIFC (ISR.IICSTIF) is cleared by up_scii2c_interrupt()
   * before this is called, not here -- see its comment for why the
   * order against ra_clear_ir() matters.
   */

  if (priv->phase == SCII2C_PHASE_ADDRESS)
    {
      up_scii2c_send_address(priv);
      return;
    }

  if (priv->restart)
    {
      priv->restart = false;
      priv->msgndx++;
      priv->phase = SCII2C_PHASE_ADDRESS;
      up_scii2c_send_address(priv);
      return;
    }

  /* Final stop -- either a successful transfer's own end, or the stop
   * up_scii2c_nack_abort() requested after a NACK; priv->result already
   * carries the difference, nothing else here needs to.  Disable the
   * transmitter and receiver (RA8M1 User's Manual Table 31.41 is the
   * mirror image of this, step 9's enable) and wait for CESR to catch up
   * before reporting completion.
   */

  up_scii2c_serialout(priv, R_SCI_B_CCR0_OFFSET, 0);
  up_scii2c_cesr_wait(priv, R_SCI_B_CESR_TIST | R_SCI_B_CESR_RIST, 0);

  priv->phase = SCII2C_PHASE_IDLE;
  nxsem_post(&priv->waitsem);
}

/****************************************************************************
 * Name: up_scii2c_interrupt
 *
 * Description:
 *   For teirq: clear the SCI-side source (ICFCLR.IICSTIFC, i.e.
 *   ISR.IICSTIF) before the ICU's latched IR flag, not after.  IICSTIF
 *   stays 1 for a brief window between the two if the order is reversed;
 *   if that is enough for the ICU to re-assert IR before ra_clear_ir()
 *   runs, the NVIC re-enters this vector the instant it returns -- with
 *   priv->phase already past SCII2C_PHASE_ADDRESS from the first call's
 *   up_scii2c_send_address(), that spurious second call falls through to
 *   up_scii2c_teinterrupt()'s final-stop cleanup and zeroes CCR0 moments
 *   after the transmitter was armed, killing SCL outright.  Reproduced
 *   and confirmed: a breakpoint placed right after the TDR write in
 *   up_scii2c_send_address() (delaying this ISR's return) made transfers
 *   work every time.
 *
 ****************************************************************************/

static int up_scii2c_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)arg;

#ifdef CONFIG_RA_DTC
  if (irq == priv->rxirq)
    {
      ra_clear_ir(irq);
      up_scii2c_rxinterrupt(priv);
    }
  else
#endif
  if (irq == priv->txirq)
    {
      ra_clear_ir(irq);
      up_scii2c_txinterrupt(priv);
    }
  else
    {
      DEBUGASSERT(irq == priv->teirq);
      up_scii2c_serialout(priv, R_SCI_B_ICFCLR_OFFSET,
                           R_SCI_B_ICFCLR_IICSTIFC);
      ra_clear_ir(irq);
      up_scii2c_teinterrupt(priv);
    }

  return OK;
}

/****************************************************************************
 * Name: up_scii2c_transfer
 *
 * Description:
 *   The .transfer() entry point: serializes callers, enables the
 *   channel, requests a start condition for the first message, and
 *   blocks until up_scii2c_teinterrupt() reports completion (success or
 *   abort) via waitsem.  10-bit addressing and I2C_M_NOSTART are not
 *   implemented yet -- every message gets its own fresh address byte.
 *
 ****************************************************************************/

static int up_scii2c_transfer(FAR struct i2c_master_s *dev,
                               FAR struct i2c_msg_s *msgs, int count)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev;
  int ret;

  DEBUGASSERT(count > 0);

  nxmutex_lock(&priv->lock);

  up_scii2c_setfrequency(priv, msgs[0].frequency);

  priv->msgs      = msgs;
  priv->nmsgs     = count;
  priv->msgndx    = 0;
  priv->result    = OK;
  priv->restart   = false;
  priv->phase     = SCII2C_PHASE_ADDRESS;

  up_scii2c_serialout(priv, R_SCI_B_CCR0_OFFSET,
                       R_SCI_B_CCR0_TE | R_SCI_B_CCR0_RE |
                       R_SCI_B_CCR0_TEIE | R_SCI_B_CCR0_TIE);
  up_scii2c_cesr_wait(priv, R_SCI_B_CESR_TIST | R_SCI_B_CESR_RIST,
                      R_SCI_B_CESR_TIST | R_SCI_B_CESR_RIST);

  up_scii2c_request_condition(priv, R_SCI_B_ICR_IICSTAREQ);

  ret = nxsem_tickwait(&priv->waitsem, SCII2C_TIMEOUT_TICKS);
  if (ret < 0)
    {
      up_scii2c_force_abort(priv, -ETIMEDOUT);
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
 * Name: ra_sci_i2cbus_initialize
 ****************************************************************************/

FAR struct i2c_master_s *ra_sci_i2cbus_initialize(int channel)
{
  FAR struct up_dev_s *priv;

  switch (channel)
    {
#ifdef CONFIG_RA_SCI0_I2C
      case 0:
        priv = &g_sci0priv;
        break;
#endif
#ifdef CONFIG_RA_SCI1_I2C
      case 1:
        priv = &g_sci1priv;
        break;
#endif
#ifdef CONFIG_RA_SCI2_I2C
      case 2:
        priv = &g_sci2priv;
        break;
#endif
#ifdef CONFIG_RA_SCI3_I2C
      case 3:
        priv = &g_sci3priv;
        break;
#endif
#ifdef CONFIG_RA_SCI4_I2C
      case 4:
        priv = &g_sci4priv;
        break;
#endif
#ifdef CONFIG_RA_SCI9_I2C
      case 9:
        priv = &g_sci9priv;
        break;
#endif
      default:
        return NULL;
    }

  nxmutex_init(&priv->lock);
  nxsem_init(&priv->waitsem, 0, 0);

  up_scii2c_setup(priv, 100000);  /* Standard-mode default; .transfer() can
                                    * raise this per message's own
                                    * frequency.
                                    */

#ifdef CONFIG_RA_DTC
  irq_attach(priv->rxirq, up_scii2c_interrupt, priv);
  up_enable_irq(priv->rxirq);
#endif

  irq_attach(priv->txirq, up_scii2c_interrupt, priv);
  irq_attach(priv->teirq, up_scii2c_interrupt, priv);

  up_enable_irq(priv->txirq);
  up_enable_irq(priv->teirq);

  return &priv->dev;
}
#endif /* CONFIG_RA_SCIn_I2C */
