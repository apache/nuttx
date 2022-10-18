/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_emmc.c
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
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "cxd56_clock.h"
#include "cxd56_emmc.h"
#include "hardware/cxd56_emmc.h"
#include "cxd56_pinconfig.h"

/****************************************************************************
 * Pre-processoro Definitions
 ****************************************************************************/

#define SECTOR_SIZE (512)

#define EMMC_DATA_WRITE 0
#define EMMC_DATA_READ  1
#define EMMC_NON_DATA   2

#define EMMC_NON_RESP 0
#define EMMC_RESP_R1  1
#define EMMC_RESP_R1B 2
#define EMMC_RESP_R2  3
#define EMMC_RESP_R3  4

#define EMMC_CLKDIV_UNDER_400KHZ  (32u)
#define EMMC_CLKDIV_NON_DIV       (0u)

#define EMMC_RCA                  (2)         /* greater than 1 */

#define EMMC_DATA_TIMEOUT         (0xFFFFFFu) /* max reg value */
#define EMMC_RESP_TIMEOUT         (0xFFu)     /* max reg value */

#define EMMC_MSIZE                (6)         /* Burst size is 512B */
#define EMMC_FIFO_DEPTH           (0x100)     /* FIFO size is 1KB */

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct emmc_dma_desc_s
{
  uint32_t ctrl;
  uint32_t size;
  uint32_t addr;
  uint32_t next;
};

struct cxd56_emmc_state_s
{
  mutex_t lock;
  int crefs;
  uint32_t total_sectors;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Block driver interfaces **************************************************/

static int       cxd56_emmc_open(struct inode *inode);
static int       cxd56_emmc_close(struct inode *inode);
static ssize_t   cxd56_emmc_read(struct inode *inode,
                                 unsigned char *buffer,
                                 blkcnt_t start_sector,
                                 unsigned int nsectors);
#if !defined(CONFIG_MMCSD_READONLY)
static ssize_t   cxd56_emmc_write(struct inode *inode,
                                  const unsigned char *buffer,
                                  blkcnt_t start_sector,
                                  unsigned int nsectors);
#endif
static int       cxd56_emmc_geometry(struct inode *inode,
                                     struct geometry *geometry);
static int       emmc_interrupt(int irq, void *context, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct block_operations g_bops =
{
  cxd56_emmc_open,     /* open     */
  cxd56_emmc_close,    /* close    */
  cxd56_emmc_read,     /* read     */
#if !defined(CONFIG_MMCSD_READONLY)
  cxd56_emmc_write,    /* write    */
#else
  NULL,                /* write    */
#endif
  cxd56_emmc_geometry, /* geometry */
  NULL                 /* ioctl    */
};

static sem_t g_waitsem;
struct cxd56_emmc_state_s g_emmcdev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void emmc_cmdstarted(void)
{
  uint32_t val;

  do
    {
      val = getreg32(EMMC_CMD);
    }
  while (val & EMMC_CMD_START_CMD);
}

static void emmc_reset(uint32_t reg, uint32_t bits)
{
  uint32_t val;

  val = getreg32(reg);
  putreg32((val | bits), reg);
  do
    {
      val = getreg32(reg);
    }
  while (val & bits);
}

#if !defined(CONFIG_MMCSD_READONLY)
static void emmc_flushwritefifo(void)
{
  /* eMMC host controller has a problem that invalid data is still remained
   * in the FIFO after data write done,
   * therefore, invalid data will be written to eMMC in the next data write.
   * So, we need to reset FIFO after data write.
   * And invalid data remain in FIFO when setting DDR_REG, too
   */

  emmc_reset(EMMC_CTRL, EMMC_CTRL_FIFO_RESET | EMMC_CTRL_CTRL_RESET);

  putreg32(0, EMMC_CMDARG);
  putreg32(EMMC_CMD_START_CMD | EMMC_CMD_USE_HOLDREG |
           EMMC_CMD_UPDATE_CLKREG | EMMC_CMD_WAIT_PRE_DATA,
           EMMC_CMD);

  emmc_cmdstarted();

  emmc_reset(EMMC_CTRL, EMMC_CTRL_FIFO_RESET);
}
#endif

static void emmc_initregister(void)
{
  uint32_t rxwmark;
  uint32_t txwmark;
  uint32_t blksize;
  uint32_t blksizeperhdatawidth;
  uint32_t rdthr;

  emmc_reset(EMMC_CTRL, EMMC_CTRL_CTRL_RESET);
  emmc_reset(EMMC_CTRL, EMMC_CTRL_FIFO_RESET);
  emmc_reset(EMMC_CTRL, EMMC_CTRL_DMA_RESET);
  emmc_reset(EMMC_BMOD, EMMC_BMOD_SW_RESET);

  putreg32(EMMC_CTRL_USE_IDMAC | EMMC_CTRL_INT_ENABLE, EMMC_CTRL);

  putreg32((EMMC_DATA_TIMEOUT << EMMC_TMOUT_DATA_SHIFT) |
           (EMMC_RESP_TIMEOUT << EMMC_TMOUT_RESP_SHIFT),
           EMMC_TMOUT);

  rxwmark = EMMC_FIFO_DEPTH / 2 - 1;
  txwmark = EMMC_FIFO_DEPTH / 2;
  putreg32((EMMC_MSIZE << EMMC_FIFOTH_MSIZE_SHIFT) |
           (rxwmark << EMMC_FIFOTH_RX_SHIFT) |
           (txwmark << EMMC_FIFOTH_TX_SHIFT),
           EMMC_FIFOTH);

  putreg32(EMMC_BMOD_IDMAC_EN | EMMC_BMOD_FIXED_BURST, EMMC_BMOD);

  blksize = SECTOR_SIZE;
  blksizeperhdatawidth = blksize / 4;
  if (blksize < EMMC_FIFO_DEPTH / 2)
    {
      rdthr = blksizeperhdatawidth;
    }
  else
    {
      rdthr = blksizeperhdatawidth / 2;
    }

  putreg32((rdthr << EMMC_CARD_RD_THR_SHIFT |
            EMMC_BSYCLR_INT_EN | EMMC_CARD_RD_THE_EN),
            EMMC_CARDTHRCTL);

  putreg32(0, EMMC_INTMASK);
}

static void emmc_changeclock(int clkdiv)
{
  uint32_t cmd;

  cmd = EMMC_CMD_START_CMD | EMMC_CMD_UPDATE_CLKREG |
    EMMC_CMD_USE_HOLDREG | EMMC_CMD_WAIT_PRE_DATA;

  /* disable clock */

  putreg32(EMMC_CLKENA_DIS, EMMC_CLKENA);
  putreg32(cmd, EMMC_CMD);
  emmc_cmdstarted();

  /* change clock */

  putreg32(clkdiv, EMMC_CLKDIV);
  putreg32(cmd, EMMC_CMD);
  emmc_cmdstarted();

  /* enable clock */

  putreg32(EMMC_CLKENA_ENA, EMMC_CLKENA);
  putreg32(cmd, EMMC_CMD);
  emmc_cmdstarted();
}

static struct emmc_dma_desc_s *emmc_setupdma(void *buf, unsigned int nbytes)
{
  int i;
  int ndescs;
  struct emmc_dma_desc_s *descs;
  struct emmc_dma_desc_s  *d;
  uint32_t addr;
  uint32_t size;
  unsigned int remain;

  if (nbytes == 0)
    {
      return NULL;
    }

  ndescs = nbytes / 4096;
  if ((nbytes & (4096 - 1)) != 0)
    {
      ndescs++;
    }

  descs = (struct emmc_dma_desc_s *)
    kmm_malloc(ndescs * sizeof(struct emmc_dma_desc_s));
  if (!descs)
    {
      return NULL;
    }

  remain = nbytes;
  addr = CXD56_PHYSADDR(buf);

  for (i = 0, d = descs; i < ndescs; i++, d++)
    {
      d->ctrl = EMMC_IDMAC_DES0_OWN | EMMC_IDMAC_DES0_CH |
                EMMC_IDMAC_DES0_DIC;

      size = MIN(remain, 4096);
      d->size = size;
      d->addr = addr;
      d->next = CXD56_PHYSADDR(d + 1);

      remain -= size;
      addr += size;
    }

  ASSERT(remain == 0);

  /* Adjust first and last descriptor members */

  descs[0].ctrl          |= EMMC_IDMAC_DES0_FD;
  descs[ndescs - 1].ctrl |= EMMC_IDMAC_DES0_LD;
  descs[ndescs - 1].next  = 0;

#ifdef CONFIG_DEBUG_FS_INFO
  for (i = 0, d = descs; i < ndescs; i++, d++)
    {
      finfo("desc %p = ctrl 0x%x, size 0x%x, addr 0x%x, next 0x%x\n",
            d, d->ctrl, d->size, d->addr, d->next);
    }
#endif

  putreg32(CXD56_PHYSADDR(descs), EMMC_DBADDR);

  return descs;
}

static int emmc_checkresponse(void)
{
  uint32_t resp = getreg32(EMMC_RESP0);
  uint32_t intsts = getreg32(EMMC_RINTSTS);

  if (intsts & EMMC_INTSTS_RTO)
    {
      ferr("Response timed out.\n");
      return -EIO;
    }

  if (resp & R1STATUS_ALL_ERR)
    {
      ferr("Response error %08" PRIx32 "\n", resp);
      return -EIO;
    }

  return OK;
}

static void emmc_send(int datatype, uint32_t opcode, uint32_t arg,
                      int resptype)
{
  uint32_t prev;
  uint32_t mask;
  uint32_t cmd;
  uint32_t status;
  int ret;

  /* Get current interrupt mask, leave SDIO relative bits. */

  prev = mask = getreg32(EMMC_INTMASK) & EMMC_INTSTS_SDIO;

  cmd = EMMC_CMD_START_CMD | EMMC_CMD_USE_HOLDREG |
    EMMC_CMD_CHK_RESP_CRC | EMMC_CMD_RESP_EXPECTED |
    EMMC_CMD_WAIT_PRE_DATA | opcode;

  switch (datatype)
    {
      case EMMC_DATA_WRITE:
        cmd |= EMMC_CMD_WRITE | EMMC_CMD_DATA_EXPECTED;
        mask |= EMMC_INTSTS_DTO;
        break;
      case EMMC_DATA_READ:
        cmd |= EMMC_CMD_READ | EMMC_CMD_DATA_EXPECTED;
        mask |= EMMC_INTSTS_DTO;
        break;
      case EMMC_NON_DATA:
        mask |= EMMC_INTSTS_CD;
        if (opcode == GO_IDLE_STATE)
          {
            cmd |= EMMC_CMD_SEND_INIT;
            cmd &= ~EMMC_CMD_CHK_RESP_CRC;
            cmd &= ~EMMC_CMD_RESP_EXPECTED;
          }
        else if (opcode == SEND_OP_COND)
          {
            cmd &= ~EMMC_CMD_CHK_RESP_CRC;
          }
        else if (opcode == ALL_SEND_CID)
          {
            cmd |= EMMC_CMD_RESP_LENGTH;
          }
        else if (opcode == SEND_CSD)
          {
            cmd |= EMMC_CMD_RESP_LENGTH;
          }
        else if (opcode == STOP_TRANS)
          {
            cmd |= EMMC_CMD_STOP_CMD;
          }
        break;
    }

  /* Enable error interrupts */

  mask |= EMMC_INTSTS_RCRC | EMMC_INTSTS_DCRC |
    EMMC_INTSTS_DRTO | EMMC_INTSTS_HTO | EMMC_INTSTS_FRUN |
    EMMC_INTSTS_HLE | EMMC_INTSTS_EBE;

  putreg32(0xffffu, EMMC_RINTSTS);
  putreg32(0x1ff37u, EMMC_IDSTS);
  putreg32(mask, EMMC_INTMASK);

  /* Send command */

  putreg32(arg, EMMC_CMDARG);
  putreg32(cmd, EMMC_CMD);

  /* Wait for command or data transfer done */

  ret = nxsem_wait_uninterruptible(&g_waitsem);
  if (ret < 0)
    {
      return;
    }

  /* Restore interrupt mask */

  putreg32(prev, EMMC_INTMASK);

  /* Waiting for device ready */

  do
    {
      status = getreg32(EMMC_STATUS);
    }
  while (status & EMMC_STATUS_DATA_BUSY);
}

static int emmc_is_powerup(void)
{
  int retry;

  /* 5ms * 1000 times */

  retry = 1000;
  do
    {
      uint32_t response;
      uint32_t intsts;

      emmc_send(EMMC_NON_DATA, SEND_OP_COND,
                OCR_SECTOR_MODE | OCR_DUAL_VOLT, EMMC_RESP_R3);
      intsts = getreg32(EMMC_RINTSTS);
      if (intsts & EMMC_INTSTS_RTO)
        {
          return -EIO;
        }

      response = getreg32(EMMC_RESP0);
      if (response == (OCR_SECTOR_MODE | OCR_DUAL_VOLT | OCR_POWER_UP))
        {
          return 0;
        }

      up_mdelay(5);
    }
  while (--retry);

  return -ETIMEDOUT;
}

static int emmc_switchcmd(uint8_t index, uint8_t val)
{
  emmc_send(EMMC_NON_DATA, SWITCH,
            (uint32_t)((3u << 24) | (index << 16) | (val << 8)),
            EMMC_RESP_R1B);
  if (emmc_checkresponse())
    {
      return -EIO;
    }

  emmc_send(EMMC_NON_DATA, SEND_STATUS, EMMC_RCA << 16, EMMC_RESP_R1);
  if (emmc_checkresponse())
    {
      return -EIO;
    }

  return OK;
}

/****************************************************************************
 * Name: emmc_interrupt
 *
 * Description:
 *   The eMMC Interrupt Handler
 *
 ****************************************************************************/

static int emmc_interrupt(int irq, void *context, void *arg)
{
  uint32_t intr;

  /* Get interrupt status and clear eMMC bits */

  intr = getreg32(EMMC_MINTSTS) & ~EMMC_INTSTS_SDIO;
  putreg32(intr, EMMC_RINTSTS);

  if (intr & EMMC_INTSTS_RE)
    {
      ferr("Response error.\n");
    }

  if (intr & EMMC_INTSTS_CD)
    {
      /* Command done */
    }

  if (intr & EMMC_INTSTS_DTO)
    {
      /* Data transfer over */
    }

  if (intr & EMMC_INTSTS_TXDR)
    {
      ferr("Transmit FIFO data request.\n");
    }

  if (intr & EMMC_INTSTS_RXDR)
    {
      ferr("Receive FIFO data request.\n");
    }

  if (intr & EMMC_INTSTS_RCRC)
    {
      ferr("Response CRC error.\n");
    }

  if (intr & EMMC_INTSTS_DCRC)
    {
      ferr("Data CRC error.\n");
    }

  if (intr & EMMC_INTSTS_RTO)
    {
      ferr("Response timeout/Boot Ack Received.\n");
    }

  if (intr & EMMC_INTSTS_DRTO)
    {
      ferr("Data read timeout/Boot Data Start.\n");
    }

  if (intr & EMMC_INTSTS_HTO)
    {
      ferr("Data stavation-by-host timeout/Volt_switch_int.\n");
    }

  if (intr & EMMC_INTSTS_FRUN)
    {
      ferr("FIFO underrun/overrun error.\n");
    }

  if (intr & EMMC_INTSTS_HLE)
    {
      ferr("Hardware locked write error.\n");
    }

  if (intr & EMMC_INTSTS_BCI)
    {
      /* Start-bit error/Busy clear interrupt. */
    }

  if (intr & EMMC_INTSTS_ACD)
    {
      /* Auto command done */
    }

  if (intr & EMMC_INTSTS_EBE)
    {
      ferr("End-bit error/write no CRC.\n");
    }

  nxsem_post(&g_waitsem);
  return OK;
}

static void emmc_pincontrol(bool on)
{
  if (on)
    {
      CXD56_PIN_CONFIGS(PINCONFS_EMMC);
    }
  else
    {
      CXD56_PIN_CONFIGS(PINCONFS_EMMC_GPIO);
    }
}

static int emmc_hwinitialize(void)
{
  int ret = OK;

  cxd56_emmc_clock_enable(1, 1, 0);

  /* Configure pin */

  emmc_pincontrol(true);

  /* Setup IRQ before command send */

  emmc_initregister();

  emmc_changeclock(EMMC_CLKDIV_UNDER_400KHZ);

  irq_attach(CXD56_IRQ_EMMC, emmc_interrupt, NULL);

  up_enable_irq(CXD56_IRQ_EMMC);

  emmc_send(EMMC_NON_DATA, GO_IDLE_STATE, 0, EMMC_NON_RESP);

  if ((ret = emmc_is_powerup()) != 0)
    {
      goto errout;
    }

  emmc_send(EMMC_NON_DATA, ALL_SEND_CID, 0, EMMC_RESP_R2);
  emmc_send(EMMC_NON_DATA, SET_RELATIVE_ADDR, EMMC_RCA << 16, EMMC_RESP_R1);
  if (emmc_checkresponse())
    {
      goto errout;
    }

#ifdef EMMC_USE_SEND_CSD
  emmc_send(EMMC_NON_DATA, SEND_CSD, (EMMC_RCA << 16), EMMC_RESP_R2);
#endif

  emmc_send(EMMC_NON_DATA, SELECT_DESELECT, (EMMC_RCA << 16), EMMC_RESP_R1);
  if (emmc_checkresponse())
    {
      goto errout;
    }

  ret = emmc_switchcmd(EXTCSD_HS_TIMING, EXTCSD_HS_TIMING_HIGH_SPEED);
  if (ret)
    {
      goto errout;
    }

  ret = emmc_switchcmd(EXTCSD_BUS_WIDTH, EXTCSD_BUS_WIDTH_4BIT_SDR);
  if (ret)
    {
      goto errout;
    }

  putreg32(EMMC_CTYPE_4BIT_MODE, EMMC_CTYPE);

  emmc_changeclock(EMMC_CLKDIV_NON_DIV);

#ifdef CONFIG_CXD56_EMMC_VENDOR_TOSHIBA
  /* Vendor-specific command */

  ret = emmc_switchcmd(EXTCSD_PON, EXTCSD_PON_POWERED_ON);
  if (ret)
    {
      goto errout;
    }
#endif

  return OK;

errout:
  up_disable_irq(CXD56_IRQ_EMMC);
  emmc_pincontrol(true);
  cxd56_emmc_clock_disable();

  return ret;
}

static int cxd56_emmc_readsectors(struct cxd56_emmc_state_s *priv,
                                  void *buf,
                                  size_t start_sector,
                                  unsigned int nsectors)
{
  struct emmc_dma_desc_s *descs;
  uint32_t idsts;
  int ret = OK;

  descs = emmc_setupdma(buf, nsectors * SECTOR_SIZE);
  if (!descs)
    {
      ferr("Building descriptor failed.\n");
      return -ENOMEM;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      kmm_free(descs);
      return ret;
    }

  putreg32(nsectors * SECTOR_SIZE, EMMC_BYTCNT);
  emmc_send(EMMC_NON_DATA, SET_BLOCK_COUNT, nsectors, EMMC_RESP_R1);

  /* Check command error */

  if (emmc_checkresponse())
    {
      ferr("SET_BLOCK_COUNT failed.\n");
      ret = -EIO;
      goto finish;
    }

  emmc_send(EMMC_DATA_READ, READ_MULTIPLE_BLOCK, start_sector, EMMC_RESP_R1);

  /* Check command error */

  if (emmc_checkresponse())
    {
      ferr("READ_MULTIPLE_BLOCK failed.\n");
      ret = -EIO;
      goto finish;
    }

  /* Check DMA status */

  idsts = getreg32(EMMC_IDSTS);
  if (idsts &
      (EMMC_IDSTS_FBE | EMMC_IDSTS_DU | EMMC_IDSTS_CES | EMMC_IDSTS_AIS))
    {
      ferr("DMA status failed. %08" PRIx32 "\n", idsts);
      ret = -EIO;
    }

finish:
  nxmutex_unlock(&priv->lock);
  kmm_free(descs);
  return ret;
}

#if !defined(CONFIG_MMCSD_READONLY)
static int cxd56_emmc_writesectors(struct cxd56_emmc_state_s *priv,
                                   const void *buf, blkcnt_t start_sector,
                                   unsigned int nsectors)
{
  struct emmc_dma_desc_s *descs;
  uint32_t idsts;
  int ret = OK;

  descs = emmc_setupdma((void *)buf, nsectors * SECTOR_SIZE);
  if (!descs)
    {
      return -ENOMEM;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      kmm_free(descs);
      return ret;
    }

  putreg32(nsectors * SECTOR_SIZE, EMMC_BYTCNT);
  emmc_send(EMMC_NON_DATA, SET_BLOCK_COUNT, nsectors, EMMC_RESP_R1);

  /* Check command error */

  if (emmc_checkresponse())
    {
      ferr("SET_BLOCK_COUNT failed.\n");
      ret = -EIO;
      goto finish;
    }

  emmc_send(EMMC_DATA_WRITE, WRITE_MULTIPLE_BLOCK,
            start_sector, EMMC_RESP_R1);

  /* Check command error */

  if (emmc_checkresponse())
    {
      ferr("WRITE_MULTIPLE_BLOCK failed.\n");
      ret = -EIO;
      goto finish;
    }

  /* Check DMA status */

  idsts = getreg32(EMMC_IDSTS);
  if (idsts &
      (EMMC_IDSTS_FBE | EMMC_IDSTS_DU | EMMC_IDSTS_CES | EMMC_IDSTS_AIS))
    {
      ferr("DMA status error. %08" PRIx32 "\n", idsts);
      ret = -EIO;
    }

  emmc_send(EMMC_NON_DATA, SEND_STATUS, EMMC_RCA << 16, EMMC_RESP_R1);

  /* Check command error */

  if (emmc_checkresponse())
    {
      ferr("SEND_STATUS failed.\n");
      ret = -EIO;
      goto finish;
    }

  emmc_flushwritefifo();

finish:
  nxmutex_unlock(&priv->lock);
  kmm_free(descs);
  return ret;
}
#endif

static int cxd56_emmc_open(struct inode *inode)
{
  struct cxd56_emmc_state_s *priv;
  int ret;

  DEBUGASSERT(inode && inode->i_private);
  priv = (struct cxd56_emmc_state_s *)inode->i_private;

  /* Just increment the reference count on the driver */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  priv->crefs++;
  nxmutex_unlock(&priv->lock);
  return OK;
}

static int cxd56_emmc_close(struct inode *inode)
{
  struct cxd56_emmc_state_s *priv;
  int ret;

  DEBUGASSERT(inode && inode->i_private);
  priv = (struct cxd56_emmc_state_s *)inode->i_private;

  /* Decrement the reference count on the block driver */

  DEBUGASSERT(priv->crefs > 0);
  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  priv->crefs--;
  nxmutex_unlock(&priv->lock);
  return OK;
}

static ssize_t cxd56_emmc_read(struct inode *inode,
                               unsigned char *buffer, blkcnt_t start_sector,
                               unsigned int nsectors)
{
  struct cxd56_emmc_state_s *priv;
  int ret;

  DEBUGASSERT(inode && inode->i_private);
  priv = (struct cxd56_emmc_state_s *)inode->i_private;

  finfo("Read sector %" PRIu32 " (%u sectors) to %p\n",
        start_sector, nsectors, buffer);

  ret = cxd56_emmc_readsectors(priv, buffer, start_sector, nsectors);
  if (ret)
    {
      ferr("Read sector failed. %d\n", ret);
      return 0;
    }

  return nsectors;
}

#if !defined(CONFIG_MMCSD_READONLY)
static ssize_t cxd56_emmc_write(struct inode *inode,
                                const unsigned char *buffer,
                                blkcnt_t start_sector,
                                unsigned int nsectors)
{
  struct cxd56_emmc_state_s *priv;
  int ret;

  DEBUGASSERT(inode && inode->i_private);
  priv = (struct cxd56_emmc_state_s *)inode->i_private;

  finfo("Write %p to sector %" PRIu32 " (%u sectors)\n", buffer,
        start_sector, nsectors);

  ret = cxd56_emmc_writesectors(priv, buffer, start_sector, nsectors);
  if (ret)
    {
      ferr("Write sector failed. %d\n", ret);
      return 0;
    }

  return nsectors;
}
#endif

static int cxd56_emmc_geometry(struct inode *inode,
                               struct geometry *geometry)
{
  struct cxd56_emmc_state_s *priv;

  DEBUGASSERT(inode && inode->i_private);
  priv = (struct cxd56_emmc_state_s *)inode->i_private;

  geometry->geo_available = true;
  geometry->geo_mediachanged = false;
#if !defined(CONFIG_MMCSD_READONLY)
  geometry->geo_writeenabled = true;
#else
  geometry->geo_writeenabled = false;
#endif
  geometry->geo_nsectors = priv->total_sectors;
  geometry->geo_sectorsize = SECTOR_SIZE;

  return OK;
}

int cxd56_emmcinitialize(void)
{
  struct cxd56_emmc_state_s *priv;
  uint8_t *buf;
  struct emmc_dma_desc_s *descs;
  int ret;

  priv = &g_emmcdev;

  memset(priv, 0, sizeof(struct cxd56_emmc_state_s));
  nxmutex_init(&priv->lock);
  nxsem_init(&g_waitsem, 0, 0);

  ret = emmc_hwinitialize();
  if (ret != OK)
    {
      return -EIO;
    }

  buf = (uint8_t *)kmm_malloc(SECTOR_SIZE);
  if (buf)
    {
      putreg32(SECTOR_SIZE, EMMC_BYTCNT);
      descs = emmc_setupdma(buf, SECTOR_SIZE);
      if (descs)
        {
          emmc_send(EMMC_DATA_READ, SEND_EXT_CSD, 0, EMMC_RESP_R1);
          if (emmc_checkresponse())
            {
              kmm_free(buf);
              return -EIO;
            }

          priv->total_sectors = *(uint32_t *)&buf[EXTCSD_SEC_COUNT];
          kmm_free(descs);
        }

      kmm_free(buf);
    }

  ret = register_blockdriver("/dev/emmc0", &g_bops, 0, priv);
  if (ret)
    {
      ferr("register_blockdriver failed: %d\n", -ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int emmc_uninitialize(void)
{
  /* Send power off command */

  emmc_switchcmd(EXTCSD_PON, EXTCSD_PON_POWERED_OFF_LONG);

  up_disable_irq(CXD56_IRQ_EMMC);

  /* Configure pin */

  emmc_pincontrol(false);

  cxd56_emmc_clock_disable();

  return 0;
}
