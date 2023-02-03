/****************************************************************************
 * arch/arm/src/s32k3xx/s32k3xx_qspi.c
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

/* Copyright 2022 NXP */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "s32k3xx_qspi.h"

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/param.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>

#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/cache.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/spi/qspi.h>

#include "arm_internal.h"
#include "barriers.h"

#ifdef CONFIG_S32K3XX_QSPI_DMA
#include "hardware/s32k3xx_dmamux.h"
#include "s32k3xx_edma.h"
#endif

#include "hardware/s32k3xx_qspi.h"
#include "hardware/s32k344_pinmux.h"

#ifdef CONFIG_S32K3XX_QSPI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* QSPI memory synchronization */

#define MEMORY_SYNC()     do { ARM_DSB(); ARM_ISB(); } while (0)

/* Ensure that the DMA buffers are word-aligned. */

#define ALIGN_SHIFT       2
#define ALIGN_MASK        3
#define ALIGN_UP(n)       (((n)+ALIGN_MASK) & ~ALIGN_MASK)
#define IS_ALIGNED(n)     (((uint32_t)(n) & ALIGN_MASK) == 0)

/* LUT entries used for various command sequences                 */
#define QSPI_LUT_READ        0U /* Quad Output read               */
#define QSPI_LUT_WRITE       1U /* Quad write                     */
#define QSPI_LUT_SHARE_TYPE1 2U /* Shared Lut                     */
#define QSPI_LUT_SHARE_TYPE2 3U /* Shared Lut                     */

#define QSPI_LUT_CMD_STOP      0U /* End of sequence */
#define QSPI_LUT_CMD_CMD       1U /* Command */
#define QSPI_LUT_CMD_ADDR      2U /* Address */
#define QSPI_LUT_CMD_DUMMY     3U /* Dummy cycles */
#define QSPI_LUT_CMD_MODE      4U /* 8-bit mode */
#define QSPI_LUT_CMD_MODE2     5U /* 2-bit mode */
#define QSPI_LUT_CMD_MODE4     6U /* 4-bit mode */
#define QSPI_LUT_CMD_READ      7U /* Read data */
#define QSPI_LUT_CMD_WRITE     8U /* Write data */
#define QSPI_LUT_CMD_JMP_ON_CS 9U /* Jump on chip select deassert */

#define QSPI_TRANSFER_TYPE_SYNC      0U /* Synchronous transfer using polling */
#define QSPI_TRANSFER_TYPE_ASYNC_INT 1U /* Interrupt-based asynchronous transfer */
#define QSPI_TRANSFER_TYPE_ASYNC_DMA 2U /* DMA-based asynchronous transfer */

#define QSPI_RX_BUF_SIZE             128U

/* Debug ********************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The state of the QSPI controller.
 *
 * NOTE: the S32K3XX supports only a single QSPI peripheral.  Logic here is
 * designed to support multiple QSPI peripherals.
 */

struct s32k3xx_qspidev_s
{
  struct qspi_dev_s qspi;       /* Externally visible part of the QSPI interface */
  uint32_t base;                /* QSPI controller register base address */
  uint32_t frequency;           /* Requested clock frequency */
  uint32_t actual;              /* Actual clock frequency */
  uint8_t mode;                 /* Mode 0,3 */
  uint8_t nbits;                /* Width of word in bits (8 to 32) */
  uint8_t intf;                 /* QSPI controller number (0) */
  bool initialized;             /* TRUE: Controller has been initialized */
  mutex_t lock;                 /* Assures mutually exclusive access to QSPI */
  bool memmap;                  /* TRUE: Controller is in memory mapped mode */
#ifdef CONFIG_S32K3XX_QSPI_INTERRUPTS
  xcpt_t handler;               /* Interrupt handler */
  uint8_t irq;                  /* Interrupt number */
  sem_t op_sem;                 /* Block until complete */
#endif
#ifdef CONFIG_S32K3XX_QSPI_DMA
  volatile uint32_t rxresult;   /* Result of the RX DMA */
  volatile uint32_t txresult;   /* Result of the TX DMA */
  const uint16_t    rxch;       /* The RX DMA channel number */
  const uint16_t    txch;       /* The TX DMA channel number */
  DMACH_HANDLE      rxdma;      /* DMA channel handle for RX transfers */
  DMACH_HANDLE      txdma;      /* DMA channel handle for TX transfers */
  sem_t             rxsem;      /* Wait for RX DMA to complete */
  sem_t             txsem;      /* Wait for TX DMA to complete */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

static void qspi_resetregisters(void);

#if defined(CONFIG_DEBUG_SPI_INFO) && defined(CONFIG_DEBUG_GPIO)
static void     qspi_dumpgpioconfig(const char *msg);
#else
# define        qspi_dumpgpioconfig(msg)
#endif

static inline uint32_t qspi_isbusy(void);
static inline uint32_t qspi_rxdataevent(void);
static inline uint32_t qspi_rxfill(void);

#ifdef CONFIG_S32K3XX_QSPI_INTERRUPTS
static int     qspi_interrupt(int irq, void *context, void *arg);
#endif

/* DMA support */

#ifdef CONFIG_S32K3XX_QSPI_DMA

#error DMA not supported

static int         qspi_dmarxwait(struct s32k3xx_qspidev_s *priv);
static int         qspi_dmatxwait(struct s32k3xx_qspidev_s *priv);
static inline void qspi_dmarxwakeup(struct s32k3xx_qspidev_s *priv);
static inline void qspi_dmatxwakeup(struct s32k3xx_qspidev_s *priv);
static void        qspi_dmarxcallback(DMACH_HANDLE handle, void *arg,
                                      bool done, int result);
static void        qspi_dmatxcallback(DMACH_HANDLE handle, void *arg,
                                      bool done, int result);
static inline void qspi_dmarxstart(struct s32k3xx_qspidev_s *priv);
static inline void qspi_dmatxstart(struct s32k3xx_qspidev_s *priv);
#endif

/* QSPI methods */

static int      qspi_lock(struct qspi_dev_s *dev, bool lock);
static uint32_t qspi_setfrequency(struct qspi_dev_s *dev,
                                  uint32_t frequency);
static void     qspi_setmode(struct qspi_dev_s *dev, enum qspi_mode_e mode);
static void     qspi_setbits(struct qspi_dev_s *dev, int nbits);
static int      qspi_command(struct qspi_dev_s *dev,
                  struct qspi_cmdinfo_s *cmdinfo);
static int      qspi_memory(struct qspi_dev_s *dev,
                  struct qspi_meminfo_s *meminfo);
static void    *qspi_alloc(struct qspi_dev_s *dev, size_t buflen);
static void     qspi_free(struct qspi_dev_s *dev, void *buffer);

/* Initialization */

static int      qspi_hw_initialize(struct s32k3xx_qspidev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* QSPI0 driver operations */

static const struct qspi_ops_s g_qspi0ops =
{
  .lock              = qspi_lock,
  .setfrequency      = qspi_setfrequency,
  .setmode           = qspi_setmode,
  .setbits           = qspi_setbits,
  .command           = qspi_command,
  .memory            = qspi_memory,
  .alloc             = qspi_alloc,
  .free              = qspi_free,
};

/* This is the overall state of the QSPI0 controller */

static struct s32k3xx_qspidev_s g_qspi0dev =
{
  .qspi              =
  {
    .ops             = &g_qspi0ops,
  },
  .base              = S32K3XX_QSPI_BASE,
  .lock              = NXMUTEX_INITIALIZER,
#ifdef CONFIG_S32K3XX_QSPI_INTERRUPTS
  .handler           = qspi_interrupt,
  .irq               = S32K3XX_IRQ_QSPI,
  .op_sem            = SEM_INITIALIZER(0),
#endif
  .intf              = 0,
#ifdef CONFIG_S32K3XX_QSPI_DMA
  .rxch              = DMA_REQ_QSPI_RX,
  .txch              = DMA_REQ_QSPI_TX,
  .rxsem             = SEM_INITIALIZER(0),
  .txsem             = SEM_INITIALIZER(0),
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: qspi_dumpregs
 *
 * Description:
 *   Dump the contents of all QSPI registers
 *
 * Input Parameters:
 *   priv - The QSPI controller to dump
 *   msg - Message to print before the register data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void qspi_resetregisters(void)
{
  uint8_t i;

  putreg32(0x000f404cul, S32K3XX_QSPI_MCR);
  putreg32(0x0, S32K3XX_QSPI_IPCR);
  putreg32(QSPI_FLSHCR_TCSS(3) | QSPI_FLSHCR_TCSH(3), S32K3XX_QSPI_FLSHCR);
  putreg32(QSPI_BUFCR_MSTRID(3), S32K3XX_QSPI_BUF0CR);
  putreg32(QSPI_BUFCR_MSTRID(2), S32K3XX_QSPI_BUF1CR);
  putreg32(QSPI_BUFCR_MSTRID(1), S32K3XX_QSPI_BUF2CR);
  putreg32(QSPI_BUFCR_MSTRID(0), S32K3XX_QSPI_BUF3CR);
  putreg32(0x0, S32K3XX_QSPI_BFGENCR);
  putreg32(0x0, S32K3XX_QSPI_SOCCR);
  putreg32(0x0, S32K3XX_QSPI_BUF0IND);
  putreg32(0x0, S32K3XX_QSPI_BUF1IND);
  putreg32(0x0, S32K3XX_QSPI_BUF2IND);
  putreg32(0x01200000ul, S32K3XX_QSPI_DLLCRA);
  putreg32(0x0, S32K3XX_QSPI_SFAR);
  putreg32(0xff000000ul, S32K3XX_QSPI_SMPR);
  putreg32(0x0, S32K3XX_QSPI_RBCT);
  putreg32(0x0, S32K3XX_QSPI_TBDR);
  putreg32(0x0, S32K3XX_QSPI_TBCT);
  putreg32(0x0c8378c1ul, S32K3XX_QSPI_FR);
  putreg32(0x0, S32K3XX_QSPI_RSER);
  putreg32(QSPI_SPTRCLR_BFPTRC | QSPI_SPTRCLR_IPPTRC, S32K3XX_QSPI_SPTRCLR);
  putreg32(QSPI_SFAD_TPAD(0x1c0000), S32K3XX_QSPI_SFA1AD);
  putreg32(QSPI_SFAD_TPAD(0x1c0000), S32K3XX_QSPI_SFA2AD);
  putreg32(QSPI_SFAD_TPAD(0x1c0000), S32K3XX_QSPI_SFB1AD);
  putreg32(QSPI_SFAD_TPAD(0x1c0000), S32K3XX_QSPI_SFB2AD);
  putreg32(QSPI_LUTKEY_KEY, S32K3XX_QSPI_LUTKEY);
  putreg32(QSPI_LKCR_UNLOCK, S32K3XX_QSPI_LCKCR);
  putreg32(QSPI_LUT_OPRND0(0x3) | QSPI_LUT_INSTR0(QSPI_LUT_CMD_CMD)
           | QSPI_LUT_OPRND1(0x18) | QSPI_LUT_INSTR1(0x2),
           S32K3XX_QSPI_LUT0);
  putreg32(QSPI_LUT_OPRND0(0x8) | QSPI_LUT_INSTR0(0x7)
           | QSPI_LUT_INSTR1(0x9), S32K3XX_QSPI_LUT1);

  putreg32(QSPI_LUT_OPRND0(0x38) | QSPI_LUT_INSTR0(QSPI_LUT_CMD_CMD)
           | QSPI_LUT_OPRND1(24) | QSPI_LUT_INSTR1(QSPI_LUT_CMD_ADDR),
           S32K3XX_QSPI_LUT5);
  putreg32(QSPI_LUT_OPRND0(0X10) | QSPI_LUT_INSTR0(QSPI_LUT_CMD_WRITE)
           | QSPI_LUT_INSTR1(QSPI_LUT_CMD_STOP), S32K3XX_QSPI_LUT6);
  for (i = 7; i < S32K3XX_QSPI_LUT_COUNT; i++)
    {
      putreg32(0x0, S32K3XX_QSPI_LUT(i));
    }
}

#if defined(CONFIG_DEBUG_SPI_INFO) && defined(CONFIG_DEBUG_GPIO)
static void qspi_dumpgpioconfig(const char *msg)
{
  uint32_t regval;
  spiinfo("%s:\n", msg);
}
#endif

static inline uint32_t qspi_isbusy(void)
{
  return (getreg32(S32K3XX_QSPI_SR) & QSPI_SR_BUSY);
}

static inline uint32_t qspi_rxdataevent(void)
{
  return (getreg32(S32K3XX_QSPI_SR) & QSPI_SR_RXWE);
}

static inline uint32_t qspi_rxfill(void)
{
  return (getreg32(S32K3XX_QSPI_RBSR) & QSPI_RBSR_RDBFL_MASK);
}

static void qspi_ipwrite(uint8_t seqid, uint32_t addr, uint8_t * data,
  uint32_t size)
{
  uint32_t regval;
  uint8_t bytecnt;

  /* Set write address */

  putreg32(QSPI_AMBA_BASE + addr, S32K3XX_QSPI_SFAR);

  /* Reset Tx queue */

  regval = getreg32(S32K3XX_QSPI_MCR);
  regval |= QSPI_MCR_CLR_TXF;
  putreg32(regval, S32K3XX_QSPI_MCR);

#ifdef CONFIG_S32K3XX_QSPI_INTERRUPTS
  /* clear leftover flags from previous transfers */

  putreg32(QSPI_FR_TFF, S32K3XX_QSPI_FR);

  /* enable end of transfer interrupt for asynchronous transfers */

  regval = getreg32(S32K3XX_QSPI_RSER);
  regval |= QSPI_RSER_TFIE;
  putreg32(regval, S32K3XX_QSPI_RSER);
#endif

  regval = 0;

  for (bytecnt = 0; bytecnt < size; bytecnt++)
    {
      regval += ((uint32_t)(data[bytecnt]) << (8 * bytecnt));
    }

  putreg32(regval, S32K3XX_QSPI_TBDR);

  /* Trigger IP command with specified sequence and size */

  putreg32((QSPICR_SEQID(seqid) | QSPICR_IDATSZ(size)), S32K3XX_QSPI_IPCR);

#ifndef CONFIG_S32K3XX_QSPI_INTERRUPTS
  /* Wait for command to be completed */

  while (qspi_isbusy())
    {
    }

  /* FIXME Check for errors reported by the QuadSPI */
#endif
}

static void qspi_ipread(uint8_t seqid, uint32_t addr, uint8_t * dataread,
  uint32_t size)
{
  uint32_t regval;

#ifdef CONFIG_S32K3XX_QSPI_INTERRUPTS
  /* clear leftover flags from previous transfers */

  putreg32(QSPI_FR_TFF, S32K3XX_QSPI_FR);

  /* enable end of transfer interrupt for asynchronous transfers */

  regval = getreg32(S32K3XX_QSPI_RSER);
  regval |= QSPI_RSER_TFIE;
  putreg32(regval, S32K3XX_QSPI_RSER);
#endif

  /* Reset Rx queue */

  regval = getreg32(S32K3XX_QSPI_MCR);
  regval |= QSPI_MCR_CLR_RXF;
  putreg32(regval, S32K3XX_QSPI_MCR);

  /* Set read address */

  putreg32(QSPI_AMBA_BASE + addr, S32K3XX_QSPI_SFAR);

  /* Trigger IP command with specified sequence and size */

  putreg32((QSPICR_SEQID(seqid) | QSPICR_IDATSZ(size)), S32K3XX_QSPI_IPCR);

#ifndef CONFIG_S32K3XX_QSPI_INTERRUPTS
  /* Wait for command to be completed */

  while (qspi_isbusy() || (qspi_rxfill() != 0U))
    {
      if (qspi_rxdataevent())
        {
          for (regval = 0; regval < size; regval += 4)
            {
              ((uint32_t *)dataread)[regval] =
              *(uint32_t *)S32K3XX_QSPI_RBDR(0);
              /* Perform a POP operation on the Rx buffer,
               * removing Rx_watermark entries
               */

              putreg32(QSPI_FR_RBDF, S32K3XX_QSPI_FR);
            }
        }
    }

  /* FIXME Check for errors reported by the QuadSPI */

#endif
}

static void qspi_setluttype1(uint8_t command, uint8_t instr1, uint8_t oprnd1)
{
  /* Unlock LUT */

  putreg32(QSPI_LUTKEY_KEY, S32K3XX_QSPI_LUTKEY);
  putreg32(QSPI_LKCR_UNLOCK, S32K3XX_QSPI_LCKCR);

  putreg32(QSPI_LUT_INSTR0(QSPI_LUT_CMD_CMD) |
          QSPI_LUT_PAD0_1   |
          QSPI_LUT_OPRND0(command) |
          QSPI_LUT_INSTR1(instr1) |
          QSPI_LUT_PAD1_1   |
          QSPI_LUT_OPRND1(oprnd1),
          S32K3XX_QSPI_LUT10);

  putreg32(QSPI_LUT_INSTR0(QSPI_LUT_CMD_STOP) |
          QSPI_LUT_PAD0_1   |
          QSPI_LUT_OPRND0(0) |
          QSPI_LUT_INSTR1(QSPI_LUT_CMD_STOP) |
          QSPI_LUT_PAD1_1   |
          QSPI_LUT_OPRND1(0),
          S32K3XX_QSPI_LUT11);

  /* Lock LUT */

  putreg32(QSPI_LUTKEY_KEY, S32K3XX_QSPI_LUTKEY);
  putreg32(QSPI_LKCR_LOCK, S32K3XX_QSPI_LCKCR);
}

static inline void qspi_setluttype2(uint8_t command)
{
  /* Unlock LUT */

  putreg32(QSPI_LUTKEY_KEY, S32K3XX_QSPI_LUTKEY);
  putreg32(QSPI_LKCR_UNLOCK, S32K3XX_QSPI_LCKCR);

  putreg32(QSPI_LUT_INSTR0(QSPI_LUT_CMD_CMD) |
           QSPI_LUT_PAD0_1   |
           QSPI_LUT_OPRND0(command) |
           QSPI_LUT_INSTR1(QSPI_LUT_CMD_STOP) |
           QSPI_LUT_PAD1_1   |
           QSPI_LUT_OPRND1(0),
           S32K3XX_QSPI_LUT15);

  /* Lock LUT */

  putreg32(QSPI_LUTKEY_KEY, S32K3XX_QSPI_LUTKEY);
  putreg32(QSPI_LKCR_LOCK, S32K3XX_QSPI_LCKCR);
}

/****************************************************************************
 * Name: qspi_setup_lut_cmd
 *
 * Description:
 *   Setup our transaction descriptor from a command info structure
 *
 * Input Parameters:
 *   xctn  - the transaction descriptor we setup
 *   cmdinfo  - the command info (originating from the MTD device)
 *
 * Returned Value:
 *   OK, or -errno if invalid
 *
 ****************************************************************************/

static int qspi_setup_lut_cmd(const struct qspi_cmdinfo_s *cmdinfo)
{
  int type = -1;

#ifdef CONFIG_DEBUG_SPI_INFO
  spiinfo("Transfer:\n");
  spiinfo("  flags: %02x\n", cmdinfo->flags);
  spiinfo("  cmd: %04x\n", cmdinfo->cmd);

  if (QSPICMD_ISADDRESS(cmdinfo->flags))
    {
      spiinfo("  address/length: %08lx/%d\n",
              (unsigned long)cmdinfo->addr, cmdinfo->addrlen);
    }

  if (QSPICMD_ISDATA(cmdinfo->flags))
    {
      spiinfo("  %s Data:\n",
              QSPICMD_ISWRITE(cmdinfo->flags) ? "Write" : "Read");
      spiinfo("    buffer/length: %p/%d\n",
              cmdinfo->buffer, cmdinfo->buflen);
    }
#endif

  DEBUGASSERT(cmdinfo->cmd < 256);

  /* Specify the instruction as per command info */

  if (QSPICMD_ISDATA(cmdinfo->flags))
    {
      if (QSPICMD_ISREAD(cmdinfo->flags))
        {
          qspi_setluttype1(cmdinfo->cmd,
                            QSPI_LUT_CMD_READ, cmdinfo->buflen);
        }
      else if (QSPICMD_ISWRITE(cmdinfo->flags))
        {
          qspi_setluttype1(cmdinfo->cmd,
                            QSPI_LUT_CMD_WRITE, cmdinfo->buflen);
        }

      type = QSPI_LUT_SHARE_TYPE1;
    }
  else if (QSPICMD_ISADDRESS(cmdinfo->flags))
    {
      qspi_setluttype1(cmdinfo->cmd,
                      QSPI_LUT_CMD_ADDR, cmdinfo->addrlen * 8);
      type = QSPI_LUT_SHARE_TYPE1;
    }
  else if (cmdinfo->flags == 0)
    {
      qspi_setluttype2(cmdinfo->cmd);
      type = QSPI_LUT_SHARE_TYPE2;
    }

  return type;
}

#if defined(CONFIG_S32K3XX_QSPI_INTERRUPTS)
/****************************************************************************
 * Name: qspi_interrupt
 *
 * Description:
 *   Interrupt handler; we handle all QSPI cases -- reads, writes,
 *   automatic status polling, etc.
 *
 * Input Parameters:
 *   irq  -
 *   context  -
 *
 * Returned Value:
 *   OK means we handled it
 *
 ****************************************************************************/

static int qspi_interrupt(int irq, void *context, void *arg)
{
  uint32_t regval;

  regval = getreg32(S32K3XX_QSPI_FR);

  if (regval & QSPI_FR_TFF)
    {
      nxsem_post(&g_qspi0dev.op_sem);

      /* Disable transfer interrupt */

      regval = getreg32(S32K3XX_QSPI_RSER);
      regval &= ~(QSPI_RSER_TFIE);
      putreg32(regval, S32K3XX_QSPI_RSER);
    }

  if (regval & QSPI_FR_RBDF)
    {
      nxsem_post(&g_qspi0dev.op_sem);

      /* Disable rx interrupt */

      regval = getreg32(S32K3XX_QSPI_RSER);
      regval &= ~(QSPI_RSER_RBDIE);
      putreg32(regval, S32K3XX_QSPI_RSER);
    }

  if (regval & QSPI_FR_TBFF)
    {
      nxsem_post(&g_qspi0dev.op_sem);

      /* Disable transfer interrupt */

      regval = getreg32(S32K3XX_QSPI_RSER);
      regval &= ~(QSPI_RSER_TBFIE);
      putreg32(regval, S32K3XX_QSPI_RSER);
    }

  return 0;
}
#endif

static inline void qspi_setlut_read(uint8_t command, uint8_t length)
{
  /* Unlock LUT */

  putreg32(QSPI_LUTKEY_KEY, S32K3XX_QSPI_LUTKEY);
  putreg32(QSPI_LKCR_UNLOCK, S32K3XX_QSPI_LCKCR);

  putreg32(QSPI_LUT_INSTR0(QSPI_LUT_CMD_CMD) |
           QSPI_LUT_PAD0_1   |
           QSPI_LUT_OPRND0(command) |
           QSPI_LUT_INSTR1(QSPI_LUT_CMD_ADDR) |
           QSPI_LUT_PAD1_4   |
           QSPI_LUT_OPRND1(24),
           S32K3XX_QSPI_LUT0);

  putreg32(QSPI_LUT_INSTR0(QSPI_LUT_CMD_MODE) |
           QSPI_LUT_PAD0_4   |
           QSPI_LUT_OPRND0(0) |
           QSPI_LUT_INSTR1(QSPI_LUT_CMD_DUMMY) |
           QSPI_LUT_PAD1_4   |
           QSPI_LUT_OPRND1(4),
           S32K3XX_QSPI_LUT1);

  putreg32(QSPI_LUT_INSTR0(QSPI_LUT_CMD_READ) |
           QSPI_LUT_PAD0_4   |
           QSPI_LUT_OPRND0(length) |
           QSPI_LUT_INSTR1(QSPI_LUT_CMD_STOP),
           S32K3XX_QSPI_LUT2);

  /* Lock LUT */

  putreg32(QSPI_LUTKEY_KEY, S32K3XX_QSPI_LUTKEY);
  putreg32(QSPI_LKCR_LOCK, S32K3XX_QSPI_LCKCR);
}

#if 1

/****************************************************************************
 * Name: qspi_receive_blocking
 *
 * Description:
 *   Do common data receive in a blocking (status polling) way
 *
 * Input Parameters:
 *   priv  - The QSPI controller to dump
 *   xctn  - the transaction descriptor
 *
 * Returned Value:
 *   OK, or -errno on error
 *
 ****************************************************************************/

static int qspi_receive_blocking(struct s32k3xx_qspidev_s *priv,
                                 struct qspi_meminfo_s *meminfo)
{
  uint32_t readlen;
  uint32_t remaining = meminfo->buflen;
  uint8_t *data = meminfo->buffer;
  uint32_t regval;
  int ret = 0;

  readlen = MIN(128, remaining);

  /* Copy sequence in LUT registers */

  qspi_setlut_read(meminfo->cmd, readlen);

#ifdef CONFIG_S32K3XX_QSPI_INTERRUPTS
  /* clear leftover flags from previous transfers */

  putreg32(QSPI_FR_RBDF, S32K3XX_QSPI_FR);
#endif

  while (remaining > 0)
    {
      /* Clear RX fifo */

      regval  = getreg32(S32K3XX_QSPI_MCR);
      regval |= QSPI_MCR_CLR_RXF;
      putreg32(regval, S32K3XX_QSPI_MCR);

      readlen = MIN(128, remaining);

#ifdef CONFIG_S32K3XX_QSPI_INTERRUPTS
      /* enable end of transfer interrupt for asynchronous transfers */

      regval = getreg32(S32K3XX_QSPI_RSER);
      regval |= QSPI_RSER_RBDIE;
      putreg32(regval, S32K3XX_QSPI_RSER);
#endif

      /* Set read address */

      putreg32(QSPI_AMBA_BASE +
               meminfo->addr + (meminfo->buflen - remaining),
               S32K3XX_QSPI_SFAR);

      /* Trigger IP command with specified sequence and size */

      putreg32((QSPICR_SEQID(QSPI_LUT_READ) | QSPICR_IDATSZ(readlen)),
               S32K3XX_QSPI_IPCR);

#ifdef CONFIG_S32K3XX_QSPI_INTERRUPTS
      nxsem_wait(&priv->op_sem);
#endif
      /* Wait until the command is sent */

      while (qspi_isbusy())
        {
        }

      /* Flush the contents of the modified RXBDR into physical
       * memory.
       */

      up_clean_dcache((uintptr_t)S32K3XX_QSPI_RBDR(0),
                      (uintptr_t)S32K3XX_QSPI_RBDR(0) + readlen);

      MEMORY_SYNC();

      memcpy(data, (void *)S32K3XX_QSPI_RBDR(0), readlen);
      data += readlen;
      remaining -= readlen;
    }

  return ret;
}

#endif

#ifdef CONFIG_S32K3XX_QSPI_DMA

/****************************************************************************
 * Name: qspi_receive
 *
 * Description:
 *   Do common data receive using DMA
 *
 * Input Parameters:
 *   priv     - The QSPI controller to dump
 *   meminfo  - the transaction descriptor
 *
 * Returned Value:
 *   OK, or -errno on error
 *
 ****************************************************************************/

static int qspi_receive(struct s32k3xx_qspidev_s *priv,
                                 struct qspi_meminfo_s *meminfo)
{
  uint32_t regval;
  int ret = 0;

  /* Copy sequence in LUT registers */

  qspi_setlut_read(meminfo->cmd, meminfo->buflen);

  putreg32(QSPI_RBCT_WMRK(3), S32K3XX_QSPI_RBCT);

  /* Clear RX fifo */

  regval  = getreg32(S32K3XX_QSPI_MCR);
  regval |= QSPI_MCR_CLR_RXF;
  putreg32(regval, S32K3XX_QSPI_MCR);

  /* Set read address */

  putreg32(QSPI_AMBA_BASE + meminfo->addr, S32K3XX_QSPI_SFAR);

  /* Set up the DMA */

  struct s32k3xx_edma_xfrconfig_s config;

  config.saddr  = S32K3XX_QSPI_RBDR(0);
  config.daddr  = (uint32_t) (meminfo->buffer);
  config.soff   = 0;
  config.doff   = 16;
  config.iter   = meminfo->buflen / 16;
  config.flags  = EDMA_CONFIG_LINKTYPE_LINKNONE;
  config.ssize  = EDMA_16BYTE;
  config.dsize  = EDMA_16BYTE;
  config.nbytes = 16;
#ifdef CONFIG_KINETIS_EDMA_ELINK
  config.linkch = NULL;
#endif
  config.bwc = 0;
  s32k3xx_dmach_xfrsetup(priv->rxdma, &config);

  /* Invoke DMA mode */

  putreg32(QSPI_RSER_RBDDE, S32K3XX_QSPI_RSER);

  /* Start the DMA */

  qspi_dmarxstart(priv);

  /* Trigger IP command with specified sequence and size */

  putreg32((QSPICR_SEQID(QSPI_LUT_READ) | QSPICR_IDATSZ(meminfo->buflen)),
           S32K3XX_QSPI_IPCR);

  ret = qspi_dmarxwait(priv);

  putreg32(0, S32K3XX_QSPI_RSER);
  putreg32(QSPI_RBCT_WMRK(0), S32K3XX_QSPI_RBCT);

  up_clean_dcache((uintptr_t)meminfo->addr,
                  (uintptr_t)meminfo->addr + meminfo->buflen);
  return ret;
}

#endif

static inline void qspi_setlut_write(uint8_t command)
{
  /* Unlock LUT */

  putreg32(QSPI_LUTKEY_KEY, S32K3XX_QSPI_LUTKEY);
  putreg32(QSPI_LKCR_UNLOCK, S32K3XX_QSPI_LCKCR);

  putreg32(QSPI_LUT_INSTR0(QSPI_LUT_CMD_CMD) |
           QSPI_LUT_PAD0_1   |
           QSPI_LUT_OPRND0(command) |
           QSPI_LUT_INSTR1(QSPI_LUT_CMD_ADDR) |
           QSPI_LUT_PAD1_4   |
           QSPI_LUT_OPRND1(24),
           S32K3XX_QSPI_LUT5);

  putreg32(QSPI_LUT_INSTR0(QSPI_LUT_CMD_WRITE) |
           QSPI_LUT_PAD0_4   |
           QSPI_LUT_OPRND0(16) |
           QSPI_LUT_INSTR1(QSPI_LUT_CMD_STOP),
           S32K3XX_QSPI_LUT6);

  /* Lock LUT */

  putreg32(QSPI_LUTKEY_KEY, S32K3XX_QSPI_LUTKEY);
  putreg32(QSPI_LKCR_LOCK, S32K3XX_QSPI_LCKCR);
}

#ifndef CONFIG_S32K3XX_QSPI_DMA

/****************************************************************************
 * Name: qspi_transmit_blocking
 *
 * Description:
 *   Do common data transmit in a blocking (status polling) way
 *
 * Input Parameters:
 *   priv  - The QSPI controller to dump
 *   xctn  - the transaction descriptor
 *
 * Returned Value:
 *   OK, or -errno on error
 *
 ****************************************************************************/

static int qspi_transmit_blocking(struct s32k3xx_qspidev_s *priv,
                                 struct qspi_meminfo_s *meminfo)
{
  int32_t remaining = meminfo->buflen;
  uint32_t regval;
  uint32_t count = UINT32_MAX;
  uint32_t *data = (uint32_t *)meminfo->buffer;
  uint32_t write_cycle = MIN(32, ((uint32_t)remaining) >> 2U);
  uint32_t timeout = 1000;
  int ret = 0;

  /* Copy sequence in LUT registers */

  qspi_setlut_write(meminfo->cmd);

  /* Reset serial flash and AHB */

  regval  = getreg32(S32K3XX_QSPI_MCR);
  regval |= QSPI_MCR_SWRSTSD | QSPI_MCR_SWRSTHD;
  putreg32(regval, S32K3XX_QSPI_MCR);

  regval  = getreg32(S32K3XX_QSPI_MCR);
  regval |= QSPI_MCR_MDIS;
  putreg32(regval, S32K3XX_QSPI_MCR);

  regval  = getreg32(S32K3XX_QSPI_MCR);
  regval &= ~(QSPI_MCR_SWRSTSD | QSPI_MCR_SWRSTHD);
  putreg32(regval, S32K3XX_QSPI_MCR);

  regval  = getreg32(S32K3XX_QSPI_MCR);
  regval &= ~QSPI_MCR_MDIS;
  putreg32(regval, S32K3XX_QSPI_MCR);

  /* Set read address */

  putreg32(QSPI_AMBA_BASE + meminfo->addr, S32K3XX_QSPI_SFAR);

  /* Clear TX fifo */

  regval  = getreg32(S32K3XX_QSPI_MCR);
  regval |= QSPI_MCR_CLR_TXF;
  putreg32(regval, S32K3XX_QSPI_MCR);

  do /* Wait for fifo clear otherwise timeout */
    {
      timeout--;
    }
  while ((getreg32(S32K3XX_QSPI_TBSR) & QSPI_TBSR_TRBLF_MASK) != 0
          && timeout > 0);

  if (timeout == 0)
    {
      return -ETIMEDOUT;
    }

  spiinfo("Transmit: %" PRIu32 " size: %" PRIu32 "\n",
          meminfo->addr, meminfo->buflen);

  for (count = 0U; count < write_cycle; count++)
    {
      putreg32(*data, S32K3XX_QSPI_TBDR);
      data++;
      remaining -= 4U;
    }

  /* Trigger IP command with specified sequence and size */

  putreg32((QSPICR_SEQID(QSPI_LUT_WRITE) | QSPICR_IDATSZ(meminfo->buflen)),
           S32K3XX_QSPI_IPCR);

  while (remaining > 0)
    {
      while ((getreg32(S32K3XX_QSPI_FR) & QSPI_FR_TBFF) != QSPI_FR_TBFF)
        {
        }

      putreg32(*data, S32K3XX_QSPI_TBDR);
      data++;
      remaining -= 4U;
    }

  /* Wait until the command is sent */

  while (qspi_isbusy())
    {
    }

  return ret;
}

#endif

#ifdef CONFIG_S32K3XX_QSPI_DMA

/****************************************************************************
 * Name: qspi_transmit
 *
 * Description:
 *   Do common data transmit in a blocking (status polling) way
 *
 * Input Parameters:
 *   priv  - The QSPI controller to dump
 *   xctn  - the transaction descriptor
 *
 * Returned Value:
 *   OK, or -errno on error
 *
 ****************************************************************************/

static int qspi_transmit(struct s32k3xx_qspidev_s *priv,
                                 struct qspi_meminfo_s *meminfo)
{
  int32_t remaining = meminfo->buflen;
  uint32_t regval;
  uint32_t count = UINT32_MAX;
  int ret = 0;

  /* Copy sequence in LUT registers */

  qspi_setlut_write(meminfo->cmd);

  /* Reset serial flash and AHB */

  regval  = getreg32(S32K3XX_QSPI_MCR);
  regval |= QSPI_MCR_SWRSTSD | QSPI_MCR_SWRSTHD;
  putreg32(regval, S32K3XX_QSPI_MCR);

  regval  = getreg32(S32K3XX_QSPI_MCR);
  regval |= QSPI_MCR_MDIS;
  putreg32(regval, S32K3XX_QSPI_MCR);

  regval  = getreg32(S32K3XX_QSPI_MCR);
  regval &= ~(QSPI_MCR_SWRSTSD | QSPI_MCR_SWRSTHD);
  putreg32(regval, S32K3XX_QSPI_MCR);

  regval  = getreg32(S32K3XX_QSPI_MCR);
  regval &= ~QSPI_MCR_MDIS;
  putreg32(regval, S32K3XX_QSPI_MCR);

  /* Clear TX fifo */

  regval  = getreg32(S32K3XX_QSPI_MCR);
  regval |= QSPI_MCR_CLR_TXF;
  putreg32(regval, S32K3XX_QSPI_MCR);

  /* Set read address */

  putreg32(QSPI_AMBA_BASE + meminfo->addr, S32K3XX_QSPI_SFAR);

  up_clean_dcache((uintptr_t)meminfo->buffer,
                  (uintptr_t)meminfo->buffer + meminfo->buflen);

  /* Set up the DMA */

  uint32_t adjust = 1;

  struct s32k3xx_edma_xfrconfig_s config;

  config.saddr  = (uint32_t) (meminfo->buffer);
  config.daddr  = S32K3XX_QSPI_TBDR;
  config.soff   = 4;
  config.doff   = 0;
  config.iter   = meminfo->buflen / 4;
  config.flags  = EDMA_CONFIG_LINKTYPE_LINKNONE;
  config.ssize  = EDMA_32BIT;
  config.dsize  = EDMA_32BIT;
  config.nbytes = 4;
  #ifdef CONFIG_KINETIS_EDMA_ELINK
  config.linkch = NULL;
  #endif
  config.bwc = 0x2;
  s32k3xx_dmach_xfrsetup(priv->txdma, &config);

  /* Invoke DMA mode */

  putreg32(QSPI_RSER_TBFDE, S32K3XX_QSPI_RSER);

  /* Start the DMA */

  qspi_dmatxstart(priv);

  /* Trigger IP command with specified sequence and size */

  putreg32((QSPICR_SEQID(QSPI_LUT_WRITE) | QSPICR_IDATSZ(meminfo->buflen)),
           S32K3XX_QSPI_IPCR);

  ret = qspi_dmatxwait(priv);

  putreg32(0, S32K3XX_QSPI_RSER);

  return ret;
}

#endif

/****************************************************************************
 * Name: qspi_lock
 *
 * Description:
 *   On QSPI buses where there are multiple devices, it will be necessary to
 *   lock QSPI to have exclusive access to the buses for a sequence of
 *   transfers.  The bus should be locked before the chip is selected. After
 *   locking the QSPI bus, the caller should then also call the setfrequency,
 *   setbits, and setmode methods to make sure that the QSPI is properly
 *   configured for the device.  If the QSPI bus is being shared, then it
 *   may have been left in an incompatible state.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   lock - true: Lock QSPI bus, false: unlock QSPI bus
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int qspi_lock(struct qspi_dev_s *dev, bool lock)
{
  struct s32k3xx_qspidev_s *priv = (struct s32k3xx_qspidev_s *)dev;
  int ret;

  spiinfo("lock=%d\n", lock);
  if (lock)
    {
      ret = nxmutex_lock(&priv->lock);
    }
  else
    {
      ret = nxmutex_unlock(&priv->lock);
    }

  return ret;
}

/****************************************************************************
 * Name: qspi_setfrequency
 *
 * Description:
 *   Set the QSPI frequency.
 *
 * Input Parameters:
 *   dev -       Device-specific state data
 *   frequency - The QSPI frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

static uint32_t qspi_setfrequency(struct qspi_dev_s *dev, uint32_t frequency)
{
  struct s32k3xx_qspidev_s *priv = (struct s32k3xx_qspidev_s *)dev;
  uint32_t actual = 0;
  uint32_t prescaler;

  (void)prescaler;
  (void)priv;

  /* FIXME add suport for frequency switching,
   * typically reads can be higher the nwrites
   */

  return actual;
}

/****************************************************************************
 * Name: qspi_setmode
 *
 * Description:
 *   Set the QSPI mode. Optional.  See enum qspi_mode_e for mode definitions.
 *   NOTE:  the S32K3XX QSPI supports only modes 0 and 3.
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   mode - The QSPI mode requested
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void qspi_setmode(struct qspi_dev_s *dev, enum qspi_mode_e mode)
{
  struct s32k3xx_qspidev_s *priv = (struct s32k3xx_qspidev_s *)dev;
  uint32_t regval;

  if (priv->memmap)
    {
      /* XXX we have no better return here, but the caller will find out
       * in their subsequent calls.
       */

      return;
    }

  spiinfo("mode=%d\n", mode);

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
      /* Yes... Set the mode appropriately:
       *
       * QSPI  CPOL CPHA
       * MODE
       *  0    0    0
       *  1    0    1
       *  2    1    0
       *  3    1    1
       */

      switch (mode)
        {
        case QSPIDEV_MODE0: /* CPOL=0; CPHA=0 */
          break;

        case QSPIDEV_MODE3: /* CPOL=1; CPHA=1 */
          break;

        case QSPIDEV_MODE1: /* CPOL=0; CPHA=1 */
        case QSPIDEV_MODE2: /* CPOL=1; CPHA=0 */
          spiinfo("unsupported mode=%d\n", mode);
        default:
          DEBUGASSERT(FALSE);
          return;
        }

      spiinfo("DCR=%08" PRIx32 "\n", regval);

      /* Save the mode so that subsequent re-configurations will be faster */

      priv->mode = mode;
    }
}

/****************************************************************************
 * Name: qspi_setbits
 *
 * Description:
 *   Set the number if bits per word.
 *   NOTE:  the S32K3XX QSPI only supports 8 bits, so this does nothing.
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   nbits - The number of bits requests
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void qspi_setbits(struct qspi_dev_s *dev, int nbits)
{
  /* Not meaningful for the S32K3XXx6 */

  if (8 != nbits)
    {
      spiinfo("unsupported nbits=%d\n", nbits);
      DEBUGASSERT(FALSE);
    }
}

/****************************************************************************
 * Name: qspi_command
 *
 * Description:
 *   Perform one QSPI data transfer
 *
 * Input Parameters:
 *   dev     - Device-specific state data
 *   cmdinfo - Describes the command transfer to be performed.
 *
 * Returned Value:
 *   Zero (OK) on SUCCESS, a negated errno on value of failure
 *
 ****************************************************************************/

static int qspi_command(struct qspi_dev_s *dev,
                        struct qspi_cmdinfo_s *cmdinfo)
{
  struct s32k3xx_qspidev_s *priv = (struct s32k3xx_qspidev_s *)dev;
  int ret;

  /* We assume cmd buflen should not be greater then 128 */

  DEBUGASSERT(cmdinfo->buflen < 128);

  /* Reject commands issued while in memory mapped mode, which will
   * automatically cancel the memory mapping.  You must exit the
   * memory mapped mode first.
   */

  if (priv->memmap)
    {
      return -EBUSY;
    }

  /* Setup LUT sequence */

  ret = qspi_setup_lut_cmd(cmdinfo);
  if (ret < 0)
    {
      return -1;
    }

  /* Prepare for transaction */

  if (QSPICMD_ISWRITE(cmdinfo->flags))
    {
      qspi_ipwrite(ret, 0, cmdinfo->buffer, cmdinfo->buflen);
    }
  else if(QSPICMD_ISADDRESS(cmdinfo->flags))
    {
      qspi_ipread(ret, cmdinfo->addr, cmdinfo->buffer,  1);
    }
  else
    {
      qspi_ipread(ret, 0, cmdinfo->buffer, cmdinfo->buflen);
    }

#ifdef CONFIG_S32K3XX_QSPI_INTERRUPTS
  /* Wait for the interrupt routine to finish it's magic */

  nxsem_wait(&priv->op_sem);
  MEMORY_SYNC();

  if (QSPICMD_ISREAD(cmdinfo->flags))
    {
      uint32_t regval;

      for (regval = 0; regval < cmdinfo->buflen; regval += 4)
        {
          ((uint32_t *)cmdinfo->buffer)[regval] =
          *(uint32_t *)S32K3XX_QSPI_RBDR(regval);
        }

      /* Reset Rx queue */

      regval = getreg32(S32K3XX_QSPI_MCR);
      regval |= QSPI_MCR_CLR_RXF;
      putreg32(regval, S32K3XX_QSPI_MCR);
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: qspi_memory
 *
 * Description:
 *   Perform one QSPI memory transfer
 *
 * Input Parameters:
 *   dev     - Device-specific state data
 *   meminfo - Describes the memory transfer to be performed.
 *
 * Returned Value:
 *   Zero (OK) on SUCCESS, a negated errno on value of failure
 *
 ****************************************************************************/

static int qspi_memory(struct qspi_dev_s *dev,
                       struct qspi_meminfo_s *meminfo)
{
  struct s32k3xx_qspidev_s *priv = (struct s32k3xx_qspidev_s *)dev;
  int ret;

  /* Reject commands issued while in memory mapped mode, which will
   * automatically cancel the memory mapping.  You must exit the
   * memory mapped mode first.
   */

  if (priv->memmap)
    {
      return -EBUSY;
    }

  if (QSPIMEM_ISWRITE(meminfo->flags))
    {
#ifdef CONFIG_S32K3XX_QSPI_DMA
      ret = qspi_transmit(priv, meminfo);
#else
      ret = qspi_transmit_blocking(priv, meminfo);
#endif
    }
  else
    {
#ifdef CONFIG_S32K3XX_QSPI_DMA
      ret = qspi_receive(priv, meminfo);
#else
      ret = qspi_receive_blocking(priv, meminfo);
#endif
    }

  MEMORY_SYNC();

  return ret;
}

/****************************************************************************
 * Name: qspi_alloc
 *
 * Description:
 *   Allocate a buffer suitable for DMA data transfer
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   buflen - Buffer length to allocate in bytes
 *
 * Returned Value:
 *   Address of the allocated memory on success; NULL is returned on any
 *   failure.
 *
 ****************************************************************************/

static void *qspi_alloc(struct qspi_dev_s *dev, size_t buflen)
{
  /* Here we exploit the carnal knowledge the kmm_malloc() will return memory
   * aligned to 64-bit addresses.  The buffer length must be large enough to
   * hold the rested buflen in units a 32-bits.
   */

  return kmm_malloc(ALIGN_UP(buflen));
}

/****************************************************************************
 * Name: QSPI_FREE
 *
 * Description:
 *   Free memory returned by QSPI_ALLOC
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   buffer - Buffer previously allocated via QSPI_ALLOC
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void qspi_free(struct qspi_dev_s *dev, void *buffer)
{
  if (buffer)
    {
      kmm_free(buffer);
    }
}

static void qspi_setmemmapsizea(uint32_t sizea1, uint32_t sizea2)
{
  putreg32(QSPI_AMBA_BASE + sizea1, S32K3XX_QSPI_SFA1AD);
  putreg32(QSPI_AMBA_BASE + sizea1 + sizea2, S32K3XX_QSPI_SFA2AD);
}

/****************************************************************************
 * Name: qspi_hw_initialize
 *
 * Description:
 *   Initialize the QSPI peripheral from hardware reset.
 *
 * Input Parameters:
 *   priv - Device state structure.
 *
 * Returned Value:
 *   Zero (OK) on SUCCESS, a negated errno on value of failure
 *
 ****************************************************************************/

static int qspi_hw_initialize(struct s32k3xx_qspidev_s *priv)
{
  uint32_t regval;

  /* Disable the QSPI */

  regval  = getreg32(S32K3XX_QSPI_MCR);
  regval |= QSPI_MCR_MDIS;
  putreg32(regval, S32K3XX_QSPI_MCR);

  /* Set all registers to reset value */

  qspi_resetregisters();

  /* Configure QSPI controller */

  qspi_setmemmapsizea(8388608U, 0);

  /* Sample register */

  putreg32(QSPI_SMPR_DLLFSMPFA(4), S32K3XX_QSPI_SMPR);

  /* CS Hold & Setup time & DLL tapselect */

  putreg32(QSPI_FLSHCR_TCSS(4) | QSPI_FLSHCR_TCSH(4),
           S32K3XX_QSPI_FLSHCR);

  /* Buffer configuration IP/AHB */

  regval = getreg32(S32K3XX_QSPI_RBCT);
  regval |= QSPI_RBCT_RXBRD_IP;
  putreg32(regval, S32K3XX_QSPI_RBCT);

  /* TX watermark */

  regval = getreg32(S32K3XX_QSPI_TBCT);
  regval |= QSPI_TBCT_WMRK(1);
  putreg32(regval, S32K3XX_QSPI_TBCT);

  /* RX watermark */

  regval = getreg32(S32K3XX_QSPI_RBCT);
  regval |= QSPI_RBCT_WMRK(0);
  putreg32(regval, S32K3XX_QSPI_RBCT);

  /* Set DQS Source */

  regval = getreg32(S32K3XX_QSPI_MCR);
  regval &= ~QSPI_MCR_DQS_FA_SEL_MASK;
  regval |= QSPI_MCR_DQS_FA_SEL_LOOPBACK;
  putreg32(regval, S32K3XX_QSPI_MCR);

  /* SCK pin S32K3XX pin options */

  putreg32(QSPI_SOCCR_IBE | QSPI_SOCCR_OBE | QSPI_SOCCR_DSE,
           S32K3XX_QSPI_SOCCR);

  /* AHB configuration */

  putreg32(QSPI_BUFCR_ADATSZ(64U >> 3)
                             | QSPI_BUFCR_MSTRID(0), S32K3XX_QSPI_BUF0CR);
  putreg32(QSPI_BUFCR_ADATSZ(64U >> 3)
                             | QSPI_BUFCR_MSTRID(1), S32K3XX_QSPI_BUF1CR);
  putreg32(QSPI_BUFCR_ADATSZ(64U >> 3)
                             | QSPI_BUFCR_MSTRID(2), S32K3XX_QSPI_BUF2CR);
  putreg32(QSPI_BUFCR_ADATSZ(64U >> 3) | QSPI_BUFCR_MSTRID(3)
                             | QSPI_BUF3CR_ALLMST, S32K3XX_QSPI_BUF3CR);
  putreg32(64U, S32K3XX_QSPI_BUF0IND);
  putreg32(128U, S32K3XX_QSPI_BUF1IND);
  putreg32(192U, S32K3XX_QSPI_BUF2IND);

  /* Enable QSPI */

  regval  = getreg32(S32K3XX_QSPI_MCR);
  regval &= ~QSPI_MCR_MDIS;
  putreg32(regval, S32K3XX_QSPI_MCR);

  /* Reset serial flash and AHB */

  regval  = getreg32(S32K3XX_QSPI_MCR);
  regval |= QSPI_MCR_SWRSTSD | QSPI_MCR_SWRSTHD;
  putreg32(regval, S32K3XX_QSPI_MCR);

  /* FIXME maybe add delay here */

  regval  = getreg32(S32K3XX_QSPI_MCR);
  regval |= QSPI_MCR_MDIS;
  putreg32(regval, S32K3XX_QSPI_MCR);

  regval  = getreg32(S32K3XX_QSPI_MCR);
  regval &= ~(QSPI_MCR_SWRSTSD | QSPI_MCR_SWRSTHD);
  putreg32(regval, S32K3XX_QSPI_MCR);

  regval  = getreg32(S32K3XX_QSPI_MCR);
  regval &= ~QSPI_MCR_MDIS;
  putreg32(regval, S32K3XX_QSPI_MCR);

  /* FIXME maybe add delay here */

  /* Configure DLL */

  regval  = getreg32(S32K3XX_QSPI_DLLCRA);
  regval |= (QSPI_DLLCRA_SLV_EN | QSPI_DLLCRA_SLV_DLL_BYPASS |
            QSPI_DLLCRA_SLV_DLY(0) | QSPI_DLLCRA_SLV_DLY_COARSE (4));
  putreg32(regval, S32K3XX_QSPI_DLLCRA);
  regval  = getreg32(S32K3XX_QSPI_DLLCRA);
  regval |= QSPI_DLLCRA_SLV_UPD;
  putreg32(regval, S32K3XX_QSPI_DLLCRA);

  return OK;
}

/****************************************************************************
 * Name: spi_dmarxwait
 *
 * Description:
 *   Wait for DMA to complete.
 *
 ****************************************************************************/

#ifdef CONFIG_S32K3XX_QSPI_DMA
static int qspi_dmarxwait(struct s32k3xx_qspidev_s *priv)
{
  int ret;

  /* Take the semaphore (perhaps waiting).  If the result is zero, then the
   *  DMA must not really have completed.
   */

  do
    {
      ret = nxsem_wait_uninterruptible(&priv->rxsem);

      /* The only expected error is ECANCELED which would occur if the
       * calling thread were canceled.
       */

      DEBUGASSERT(ret == OK || ret == -ECANCELED);
    }
  while (priv->rxresult == 0 && ret == OK);

  return ret;
}
#endif

/****************************************************************************
 * Name: spi_dmatxwait
 *
 * Description:
 *   Wait for DMA to complete.
 *
 ****************************************************************************/

#ifdef CONFIG_S32K3XX_QSPI_DMA
static int qspi_dmatxwait(struct s32k3xx_qspidev_s *priv)
{
  int ret;

  /* Take the semaphore (perhaps waiting).  If the result is zero, then the
   * DMA must not really have completed.
   */

  do
    {
      ret = nxsem_wait_uninterruptible(&priv->txsem);

      /* The only expected error is ECANCELED which would occur if the
       * calling thread were canceled.
       */

      DEBUGASSERT(ret == OK || ret == -ECANCELED);
    }
  while (priv->txresult == 0 && ret == OK);

  return ret;
}
#endif

/****************************************************************************
 * Name: spi_dmarxwakeup
 *
 * Description:
 *   Signal that DMA is complete
 *
 ****************************************************************************/

#ifdef CONFIG_S32K3XX_QSPI_DMA
static inline void qspi_dmarxwakeup(struct s32k3xx_qspidev_s *priv)
{
  nxsem_post(&priv->rxsem);
}
#endif

/****************************************************************************
 * Name: spi_dmatxwakeup
 *
 * Description:
 *   Signal that DMA is complete
 *
 ****************************************************************************/

#ifdef CONFIG_S32K3XX_QSPI_DMA
static inline void qspi_dmatxwakeup(struct s32k3xx_qspidev_s *priv)
{
  nxsem_post(&priv->txsem);
}
#endif

/****************************************************************************
 * Name: spi_dmarxcallback
 *
 * Description:
 *   Called when the RX DMA completes
 *
 ****************************************************************************/

#ifdef CONFIG_S32K3XX_QSPI_DMA
static void qspi_dmarxcallback(DMACH_HANDLE handle, void *arg, bool done,
                              int result)
{
  struct s32k3xx_qspidev_s *priv = (struct s32k3xx_qspidev_s *)arg;

  priv->rxresult = result | 0x80000000;  /* assure non-zero */
  qspi_dmarxwakeup(priv);
}
#endif

/****************************************************************************
 * Name: spi_dmatxcallback
 *
 * Description:
 *   Called when the RX DMA completes
 *
 ****************************************************************************/

#ifdef CONFIG_S32K3XX_QSPI_DMA
static void qspi_dmatxcallback(DMACH_HANDLE handle, void *arg, bool done,
                              int result)
{
  struct s32k3xx_qspidev_s *priv = (struct s32k3xx_qspidev_s *)arg;

  /* Wake-up the SPI driver */

  priv->txresult = result | 0x80000000;  /* assure non-zero */
  qspi_dmatxwakeup(priv);
}
#endif

/****************************************************************************
 * Name: spi_dmarxstart
 *
 * Description:
 *   Start RX DMA
 *
 ****************************************************************************/

#ifdef CONFIG_S32K3XX_QSPI_DMA
static inline void qspi_dmarxstart(struct s32k3xx_qspidev_s *priv)
{
  priv->rxresult = 0;
  s32k3xx_dmach_start(priv->rxdma, qspi_dmarxcallback, priv);
}
#endif

/****************************************************************************
 * Name: spi_dmatxstart
 *
 * Description:
 *   Start TX DMA
 *
 ****************************************************************************/

#ifdef CONFIG_S32K3XX_QSPI_DMA
static inline void qspi_dmatxstart(struct s32k3xx_qspidev_s *priv)
{
  priv->txresult = 0;
  s32k3xx_dmach_start(priv->txdma, qspi_dmatxcallback, priv);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k3xx_qspi_initialize
 *
 * Description:
 *   Initialize the selected QSPI port in master mode
 *
 * Input Parameters:
 *   intf - Interface number(must be zero)
 *
 * Returned Value:
 *   Valid QSPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct qspi_dev_s *s32k3xx_qspi_initialize(int intf)
{
  struct s32k3xx_qspidev_s *priv;
  uint32_t regval;
  int ret;

  /* The S32K3XX has only a single QSPI port */

  spiinfo("intf: %d\n", intf);
  DEBUGASSERT(intf == 0);

  /* Select the QSPI interface */

  if (intf == 0)
    {
      /* If this function is called multiple times, the following operations
       * will be performed multiple times.
       */

      /* Select QSPI0 */

      priv = &g_qspi0dev;

      /* Reset the QSPI peripheral */

      regval  = getreg32(S32K3XX_QSPI_MCR);
      regval |= QSPI_MCR_SWRSTSD | QSPI_MCR_SWRSTHD;
      putreg32(regval, S32K3XX_QSPI_MCR);

      /* Configure multiplexed pins as connected on the board. */

      s32k3xx_pinconfig(PIN_QSPI_PCSFA | PIN_OUTPUT_HIGHDRIVE);
      s32k3xx_pinconfig(PIN_QSPI_IOFA0 | PIN_OUTPUT_HIGHDRIVE);
      s32k3xx_pinconfig(PIN_QSPI_IOFA1 | PIN_OUTPUT_HIGHDRIVE);
      s32k3xx_pinconfig(PIN_QSPI_IOFA2 | PIN_OUTPUT_HIGHDRIVE);
      s32k3xx_pinconfig(PIN_QSPI_IOFA3 | PIN_OUTPUT_HIGHDRIVE);
      s32k3xx_pinconfig(PIN_QSPI_SCKFA | PIN_OUTPUT_HIGHDRIVE);
    }
  else
    {
      spierr("ERROR: QSPI%d not supported\n", intf);
      return NULL;
    }

  /* Has the QSPI hardware been initialized? */

  if (!priv->initialized)
    {
      /* Now perform one time initialization. */

#ifdef CONFIG_S32K3XX_QSPI_INTERRUPTS
      /* Attach the interrupt handler */

      ret = irq_attach(priv->irq, priv->handler, NULL);
      if (ret < 0)
        {
          spierr("ERROR: Failed to attach irq %d\n", priv->irq);
        }
#endif

      /* Perform hardware initialization.  Puts the QSPI into an active
       * state.
       */

      ret = qspi_hw_initialize(priv);
      if (ret < 0)
        {
          spierr("ERROR: Failed to initialize QSPI hardware\n");
        }

      /* Enable interrupts at the NVIC */

      priv->initialized = true;
      priv->memmap = false;
#ifdef CONFIG_S32K3XX_QSPI_INTERRUPTS
      up_enable_irq(priv->irq);
#endif

#ifdef CONFIG_S32K3XX_QSPI_DMA
      /* Initialize the QSPI dma channel. */

      if (priv->rxch && priv->txch)
        {
          if (priv->txdma == NULL && priv->rxdma == NULL)
            {
              priv->txdma = s32k3xx_dmach_alloc(priv->txch
                                                | DMAMUX_CHCFG_ENBL, 0);
              priv->rxdma = s32k3xx_dmach_alloc(priv->rxch
                                                | DMAMUX_CHCFG_ENBL, 0);
              DEBUGASSERT(priv->rxdma && priv->txdma);
            }
        }
      else
        {
          priv->rxdma = NULL;
          priv->txdma = NULL;
        }
#endif
    }

  return &priv->qspi;
}

#endif /* CONFIG_S32K3XX_QSPI */
