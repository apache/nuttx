/****************************************************************************
 * arch/arm/src/common/ameba/ameba_i2c.c
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

/* NuttX I2C master lower half for the Realtek Ameba I2C controllers (I2C0
 * and I2C1).  Each controller is registered from board bring-up and appears
 * as /dev/i2cN through the stock I2C character driver (i2c_register()).
 *
 * The controller is a Synopsys DesignWare I2C block programmed through the
 * SDK fwlib I2C API in polling mode (no interrupts): I2C_Init() programs the
 * speed/target, and I2C_MasterWrite()/Read()/RepeatRead() drive the FIFO and
 * block until the transfer completes or the poll times out.
 *
 * Unlike the UART fwlib, which runs from on-chip ROM and resolves the secure
 * register alias itself via TrustZone_IsSecure(), the I2C fwlib routines are
 * compiled from ram_common/ameba_i2c.c (see AMEBA_FWLIB_SRCS) and use the
 * register pointer they are handed without any secure conversion.  NuttX
 * runs on the KM4 core in the SECURE state, yet the I2C block only responds
 * on its NON-secure alias (see the note at AMEBA_I2C_BASES in
 * ameba_i2c_chip.h), so this driver hands the fwlib those non-secure bases.
 *
 * The chip-specific wiring (controller count, register bases, clock masks,
 * pad-mux codes and the fwlib I2C_InitTypeDef layout) lives in the per-chip
 * ameba_i2c_chip.h.  To keep the vendor headers out of the NuttX include
 * world, the few fwlib symbols and that struct layout used here are declared
 * locally rather than pulled in from <ameba_i2c.h>.
 */

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <string.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/i2c/i2c_master.h>

#include "ameba_i2c.h"
#include "ameba_i2c_chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The register bases, peripheral-clock masks, crossbar pad-mux codes,
 * controller count (AMEBA_NI2C) and I2C_InitTypeDef layout
 * (AMEBA_I2C_HAS_DMA_FIELDS) all come from the per-chip ameba_i2c_chip.h.
 * Everything below is common to every current Ameba chip.
 */

#define AMEBA_GPIO_PUPD_UP      0x2   /* GPIO_PuPd_UP                       */

/* fwlib I2C_InitTypeDef field values (I2C_ADDR_*, I2C_*_MODE, ameba_i2c.h).
 * These are identical on every Ameba chip audited, so they stay here rather
 * than in the per-chip header.
 */

#define AMEBA_I2C_ADDR_7BIT     0x0   /* I2C_ADDR_7BIT                      */
#define AMEBA_I2C_ADDR_10BIT    0x1   /* I2C_ADDR_10BIT                     */

#define AMEBA_I2C_SS_MODE       0x1   /* I2C_SS_MODE  (<= 100 kHz)          */
#define AMEBA_I2C_FS_MODE       0x2   /* I2C_FS_MODE  (<= 400 kHz)          */
#define AMEBA_I2C_HS_MODE       0x3   /* I2C_HS_MODE  (I2C1 only)           */

#define AMEBA_I2C_MASTER_MODE   0x1   /* I2C_MASTER_MODE                    */

/* IC_RAW_INTR_STAT.TX_ABRT and the matching IC_CLR_TX_ABRT selector for
 * I2C_ClearINT().  A transmit abort latches whenever an address or data
 * byte goes unacknowledged (among other causes); the fwlib master helpers
 * can miss it and still return the full byte count, so this driver checks
 * it after each transfer to surface a NAK.  It sits at bit 6 of both
 * registers on every Ameba chip audited (amebadplus/smart/lite/green2/
 * RTL8720F), so it stays here rather than in the per-chip header.
 */

#define AMEBA_I2C_TX_ABRT       (1u << 6)  /* IC_RAW_INTR_STAT TX_ABRT     */
#define AMEBA_I2C_R_TX_ABRT     (1u << 6)  /* I2C_ClearINT() TX_ABRT sel.  */

/* IC_STATUS.TFNF (transmit FIFO not full) and the bounded spin used to wait
 * for a free FIFO slot while pushing the leading bytes of a chained write
 * (an I2C_M_NOSTOP segment).  TFNF sits at bit 1 on every Ameba chip.
 */

#define AMEBA_I2C_TFNF          (1u << 1)  /* IC_STATUS TFNF               */
#define AMEBA_I2C_FIFO_TIMEOUT  100000     /* ~loop guard, ample for 16 FIFO */

/* Second/third argument to fwlib "state" style APIs. */

#define AMEBA_DISABLE           0x0
#define AMEBA_ENABLE            0x1

/* Default bus frequency used until the first transfer requests one. */

#define AMEBA_I2C_DEFAULT_FREQ  I2C_SPEED_STANDARD

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Layout-compatible mirror of the fwlib I2C_InitTypeDef (all u32, same
 * order); passed by address to I2C_StructInit()/I2C_Init().  The leading
 * I2CIdx field is present on every Ameba chip audited (amebadplus/
 * amebasmart/amebalite/amebagreen2/RTL8720F), so it stays unconditional.  A
 * single per-chip layout switch keeps this struct byte-for-byte identical to
 * the fwlib one: AMEBA_I2C_HAS_DMA_FIELDS gates the three DMA request-level
 * fields between I2CFilter and I2CAckAddr1, which only some chips carry.
 * See ameba_i2c_chip.h for the per-chip value.
 */

struct ameba_i2c_init_s
{
  uint32_t idx;                /* I2CIdx        */
  uint32_t master;             /* I2CMaster     */
  uint32_t addrmod;            /* I2CAddrMod    */
  uint32_t spdmod;             /* I2CSpdMod     */
  uint32_t rxtl;               /* I2CRXTL       */
  uint32_t txtl;               /* I2CTXTL       */
  uint32_t mstrestr;           /* I2CMstReSTR   */
  uint32_t mstgc;              /* I2CMstGC      */
  uint32_t mststartb;          /* I2CMstStartB  */
  uint32_t slvnoack;           /* I2CSlvNoAck   */
  uint32_t slvackgc;           /* I2CSlvAckGC   */
  uint32_t ackaddr;            /* I2CAckAddr    */
  uint32_t slvsetup;           /* I2CSlvSetup   */
  uint32_t sdahd;              /* I2CSdaHd      */
  uint32_t clk;                /* I2CClk (kHz)  */
  uint32_t ipclk;              /* I2CIPClk (Hz) */
  uint32_t filter;             /* I2CFilter     */
#ifdef AMEBA_I2C_HAS_DMA_FIELDS
  uint32_t txdmarqlv;          /* I2CTxDMARqLv  */
  uint32_t rxdmarqlv;          /* I2CRxDMARqLv  */
  uint32_t dmamod;             /* I2CDMAMod     */
#endif
  uint32_t ackaddr1;           /* I2CAckAddr1   */
};

struct ameba_i2c_dev_s
{
  struct i2c_master_s dev;     /* I2C master lower half (must be first) */
  uintptr_t base;              /* Non-secure I2C register base address */
  uint32_t  periph;            /* APBPeriph function mask (RCC arg 1) */
  uint32_t  clk;               /* APBPeriph clock mask (RCC arg 2) */
  uint32_t  frequency;         /* Currently programmed bus frequency (Hz) */
  uint16_t  address;           /* Currently programmed target address */
  uint8_t   addrmod;           /* Currently programmed AMEBA_I2C_ADDR_*BIT */
  uint8_t   sclpin;            /* SCL pad (AMEBA_PA()/AMEBA_PB() encoding) */
  uint8_t   sdapin;            /* SDA pad (AMEBA_PA()/AMEBA_PB() encoding) */
  uint8_t   sclfid;            /* Pin mux function code for the SCL pad */
  uint8_t   sdafid;            /* Pin mux function code for the SDA pad */
  mutex_t   lock;              /* Serializes bus access */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SDK fwlib I2C/pin/clock API.  The pin/clock helpers resolve to the on-chip
 * ROM symbol table; the I2C helpers are compiled into libameba_fwlib.a from
 * ram_common/ameba_i2c.c (see AMEBA_FWLIB_SRCS).
 */

extern void RCC_PeriphClockCmd(uint32_t periph, uint32_t clock,
                               uint8_t newstate);
extern void Pinmux_Config(uint8_t pin, uint32_t func);
extern void PAD_PullCtrl(uint8_t pin, uint8_t pull);
extern void I2C_StructInit(struct ameba_i2c_init_s *init);
extern void I2C_Init(void *i2cx, struct ameba_i2c_init_s *init);
extern void I2C_Cmd(void *i2cx, uint8_t newstate);
extern uint32_t I2C_MasterWrite(void *i2cx, uint8_t *buf, uint32_t len);
extern uint32_t I2C_MasterRead(void *i2cx, uint8_t *buf, uint32_t len);
extern uint32_t I2C_MasterRepeatRead(void *i2cx, uint8_t *wbuf,
                                     uint32_t wlen, uint8_t *rbuf,
                                     uint32_t rlen);
extern uint32_t I2C_GetRawINT(void *i2cx);
extern uint32_t I2C_ClearINT(void *i2cx, uint32_t intrbit);
extern uint8_t I2C_CheckFlagState(void *i2cx, uint32_t flag);
extern void I2C_MasterSend(void *i2cx, uint8_t *buf, uint8_t cmd,
                           uint8_t stop, uint8_t restart);

/* I2C master lower-half operations. */

static int ameba_i2c_transfer(struct i2c_master_s *dev,
                              struct i2c_msg_s *msgs, int count);
static int ameba_i2c_setup(struct i2c_master_s *dev);
static int ameba_i2c_shutdown(struct i2c_master_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct i2c_ops_s g_ameba_i2c_ops =
{
  .transfer = ameba_i2c_transfer,
  .setup    = ameba_i2c_setup,
  .shutdown = ameba_i2c_shutdown,
};

/* Per-controller register base, peripheral function/clock masks and crossbar
 * pad-mux codes, indexed by controller number and supplied by the per-chip
 * ameba_i2c_chip.h.  g_i2c_periph feeds the "function" arg and g_i2c_clk
 * the "clock" argument of RCC_PeriphClockCmd() (equal on this chip, distinct
 * on others).
 */

static const uintptr_t g_i2c_base[AMEBA_NI2C]  = AMEBA_I2C_BASES;
static const uint32_t  g_i2c_periph[AMEBA_NI2C] = AMEBA_I2C_APBPERIPH;
static const uint32_t  g_i2c_clk[AMEBA_NI2C]   = AMEBA_I2C_APBPERIPH_CLK;
static const uint8_t   g_i2c_sclfid[AMEBA_NI2C] = AMEBA_I2C_SCLFID;
static const uint8_t   g_i2c_sdafid[AMEBA_NI2C] = AMEBA_I2C_SDAFID;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ameba_i2c_reconfigure
 *
 * Description:
 *   (Re)program the controller for a given target address, addressing mode
 *   and bus frequency, and enable it.  This is a comparatively heavy DW-IP
 *   reconfiguration (the block must be disabled to change IC_TAR/speed), so
 *   it runs only when one of those parameters changes between transfers.
 *
 ****************************************************************************/

static void ameba_i2c_reconfigure(struct ameba_i2c_dev_s *priv,
                                  uint16_t address, uint8_t addrmod,
                                  uint32_t frequency)
{
  struct ameba_i2c_init_s init;
  void *i2cx = (void *)priv->base;

  memset(&init, 0, sizeof(init));
  I2C_StructInit(&init);

  init.master   = AMEBA_I2C_MASTER_MODE;
  init.addrmod  = addrmod;
  init.ackaddr  = address;

  /* Enable RESTART so a combined write-then-read (I2C_MasterRepeatRead())
   * emits a true repeated START; the DW IP ignores the RESTART command bit
   * unless IC_CON.IC_RESTART_EN is set.
   */

  init.mstrestr = AMEBA_ENABLE;

  if (frequency <= I2C_SPEED_STANDARD)
    {
      init.spdmod = AMEBA_I2C_SS_MODE;
    }
  else if (frequency <= I2C_SPEED_FAST)
    {
      init.spdmod = AMEBA_I2C_FS_MODE;
    }
  else
    {
      init.spdmod = AMEBA_I2C_HS_MODE;
    }

  init.clk = frequency / 1000;    /* fwlib expects the bus clock in kHz */

  I2C_Cmd(i2cx, AMEBA_DISABLE);
  I2C_Init(i2cx, &init);
  I2C_Cmd(i2cx, AMEBA_ENABLE);

  priv->address   = address;
  priv->addrmod   = addrmod;
  priv->frequency = frequency;
}

/****************************************************************************
 * Name: ameba_i2c_aborted
 *
 * Description:
 *   Return true if the controller latched a transmit abort during the last
 *   transfer.  The fwlib master helpers poll IC_STATUS.TFE, which the DW IP
 *   also sets when it flushes the TX FIFO on an abort, so they can return
 *   the full byte count even though the address (or a data byte) was never
 *   acknowledged.  Checking IC_RAW_INTR_STAT.TX_ABRT directly is the only
 *   reliable way to detect that NAK.  The abort is cleared here (reading
 *   IC_CLR_TX_ABRT) so the block is left in a clean state.
 *
 ****************************************************************************/

static bool ameba_i2c_aborted(struct ameba_i2c_dev_s *priv)
{
  void *i2cx = (void *)priv->base;

  if ((I2C_GetRawINT(i2cx) & AMEBA_I2C_TX_ABRT) != 0)
    {
      I2C_ClearINT(i2cx, AMEBA_I2C_R_TX_ABRT);
      return true;
    }

  return false;
}

/****************************************************************************
 * Name: ameba_i2c_writeprefix
 *
 * Description:
 *   Push the bytes of a non-terminating write segment (one flagged
 *   I2C_M_NOSTOP) into the TX FIFO without asserting STOP.  Because the DW
 *   IP only issues a STOP when a byte carries the STOP bit, leaving it clear
 *   keeps the current transaction open so the following segment continues
 *   without a repeated START -- this is how I2C_M_NOSTOP/I2C_M_NOSTART are
 *   honoured for chained writes such as the "register address + data" pair
 *   emitted by "i2c set".  The terminating segment is handled by the tested
 *   I2C_MasterWrite(), which supplies the STOP and the final TFE wait.
 *
 ****************************************************************************/

static int ameba_i2c_writeprefix(struct ameba_i2c_dev_s *priv,
                                  const uint8_t *buf, uint32_t len)
{
  void *i2cx = (void *)priv->base;
  uint32_t i;
  uint32_t to;

  for (i = 0; i < len; i++)
    {
      uint8_t byte = buf[i];

      /* Wait for a free TX FIFO slot before pushing the next byte. */

      for (to = AMEBA_I2C_FIFO_TIMEOUT;
           to > 0 && I2C_CheckFlagState(i2cx, AMEBA_I2C_TFNF) == 0; to--);

      if (to == 0)
        {
          return -ETIMEDOUT;
        }

      /* Write, no STOP, no RESTART: leave the transaction open. */

      I2C_MasterSend(i2cx, &byte, 0, 0, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: ameba_i2c_transfer
 *
 * Description:
 *   Run a sequence of I2C messages on the bus.  A write message immediately
 *   followed by a read message to the same target is fused into a single
 *   combined transaction with a repeated START.  A write flagged
 *   I2C_M_NOSTOP is chained to the following write segment(s) without an
 *   intervening STOP (e.g. register address + data from "i2c set").  Every
 *   other message is a standalone START..STOP write or read.
 *
 ****************************************************************************/

static int ameba_i2c_transfer(struct i2c_master_s *dev,
                              struct i2c_msg_s *msgs, int count)
{
  struct ameba_i2c_dev_s *priv = (struct ameba_i2c_dev_s *)dev;
  void *i2cx = (void *)priv->base;
  int ret = OK;
  int i;

  if (msgs == NULL || count < 1)
    {
      return -EINVAL;
    }

  nxmutex_lock(&priv->lock);

  for (i = 0; i < count; )
    {
      struct i2c_msg_s *msg = &msgs[i];
      uint8_t addrmod = (msg->flags & I2C_M_TEN) ?
                        AMEBA_I2C_ADDR_10BIT : AMEBA_I2C_ADDR_7BIT;
      uint32_t freq = (msg->frequency != 0) ? msg->frequency :
                      AMEBA_I2C_DEFAULT_FREQ;
      uint32_t done;

      /* Reprogram the controller only when the target, addressing mode or
       * bus frequency differs from the running configuration.
       */

      if (msg->addr != priv->address || addrmod != priv->addrmod ||
          freq != priv->frequency)
        {
          ameba_i2c_reconfigure(priv, msg->addr, addrmod, freq);
        }

      if ((msg->flags & I2C_M_READ) == 0 && (i + 1) < count &&
          (msgs[i + 1].flags & I2C_M_READ) != 0 &&
          msgs[i + 1].addr == msg->addr)
        {
          /* Write followed by read to the same target: one transaction with
           * a repeated START between the two phases.
           */

          struct i2c_msg_s *rd = &msgs[i + 1];

          done = I2C_MasterRepeatRead(i2cx, msg->buffer, msg->length,
                                      rd->buffer, rd->length);
          if (ameba_i2c_aborted(priv))
            {
              ret = -ENXIO;
              break;
            }

          if (done != (uint32_t)rd->length)
            {
              ret = -EIO;
              break;
            }

          i += 2;
        }
      else if ((msg->flags & I2C_M_READ) != 0)
        {
          done = I2C_MasterRead(i2cx, msg->buffer, msg->length);

          if (ameba_i2c_aborted(priv))
            {
              ret = -ENXIO;
              break;
            }

          if (done != (uint32_t)msg->length)
            {
              ret = -EIO;
              break;
            }

          i += 1;
        }
      else if ((msg->flags & I2C_M_NOSTOP) != 0)
        {
          /* Leading segment of a chained write (e.g. the register address
           * before the data in "i2c set"): push its bytes but keep the
           * transaction open so the next segment continues without a STOP.
           */

          ret = ameba_i2c_writeprefix(priv, msg->buffer, msg->length);
          if (ret < 0)
            {
              break;
            }

          i += 1;
        }
      else
        {
          done = I2C_MasterWrite(i2cx, msg->buffer, msg->length);
          if (ameba_i2c_aborted(priv))
            {
              ret = -ENXIO;
              break;
            }

          if (done != (uint32_t)msg->length)
            {
              ret = -EIO;
              break;
            }

          i += 1;
        }
    }

  /* A failed transfer can leave the DW IP with a latched TX abort; force a
   * full reconfiguration before the next transfer to clear it.
   */

  if (ret < 0)
    {
      priv->frequency = 0;
      priv->address   = 0xffff;
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: ameba_i2c_setup
 *
 * Description:
 *   Called by the I2C character driver on the first open.  Gate the
 *   peripheral clock and route the SCL/SDA pads to this controller; the
 *   speed/target are programmed lazily on the first transfer.
 *
 ****************************************************************************/

static int ameba_i2c_setup(struct i2c_master_s *dev)
{
  struct ameba_i2c_dev_s *priv = (struct ameba_i2c_dev_s *)dev;

  RCC_PeriphClockCmd(priv->periph, priv->clk, AMEBA_ENABLE);

  Pinmux_Config(priv->sclpin, priv->sclfid);
  Pinmux_Config(priv->sdapin, priv->sdafid);
  PAD_PullCtrl(priv->sclpin, AMEBA_GPIO_PUPD_UP);
  PAD_PullCtrl(priv->sdapin, AMEBA_GPIO_PUPD_UP);

  /* Force the first transfer to program the controller. */

  priv->frequency = 0;
  priv->address   = 0xffff;
  return OK;
}

/****************************************************************************
 * Name: ameba_i2c_shutdown
 *
 * Description:
 *   Called by the I2C character driver on the last close.  Disable the
 *   controller and gate its peripheral clock off.
 *
 ****************************************************************************/

static int ameba_i2c_shutdown(struct i2c_master_s *dev)
{
  struct ameba_i2c_dev_s *priv = (struct ameba_i2c_dev_s *)dev;

  I2C_Cmd((void *)priv->base, AMEBA_DISABLE);
  RCC_PeriphClockCmd(priv->periph, priv->clk, AMEBA_DISABLE);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ameba_i2c_register
 *
 * Description:
 *   See ameba_i2c.h.
 *
 ****************************************************************************/

int ameba_i2c_register(int bus, uint8_t sclpin, uint8_t sdapin)
{
  struct ameba_i2c_dev_s *priv;
  int ret;

  if (bus < 0 || bus >= AMEBA_NI2C)
    {
      return -EINVAL;
    }

  priv = kmm_zalloc(sizeof(struct ameba_i2c_dev_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->dev.ops = &g_ameba_i2c_ops;
  priv->base    = g_i2c_base[bus];
  priv->periph  = g_i2c_periph[bus];
  priv->clk     = g_i2c_clk[bus];
  priv->sclpin  = sclpin;
  priv->sdapin  = sdapin;
  priv->sclfid  = g_i2c_sclfid[bus];
  priv->sdafid  = g_i2c_sdafid[bus];
  priv->address = 0xffff;
  nxmutex_init(&priv->lock);

  ret = i2c_register(&priv->dev, bus);
  if (ret < 0)
    {
      _err("ERROR: i2c_register(/dev/i2c%d) failed: %d\n", bus, ret);
      nxmutex_destroy(&priv->lock);
      kmm_free(priv);
    }

  return ret;
}
