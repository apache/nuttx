/****************************************************************************
 * drivers/can/mcp2515.c
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
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <strings.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/signal.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spi/spi.h>
#include <nuttx/can/can.h>
#include <nuttx/can/mcp2515.h>

#include "mcp2515.h"

#if defined(CONFIG_CAN) && defined(CONFIG_CAN_MCP2515)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MCP2515 Configuration ****************************************************/

#define CAN_FRAME_MAX_DATA_LEN 8
#define SPI_TRANSFER_BUF_LEN   (6 + CAN_FRAME_MAX_DATA_LEN)
#define MCP2515_NUM_TX_BUFFERS 3

/* Bit timing */

#define MCP2515_PROPSEG  CONFIG_MCP2515_PROPSEG
#define MCP2515_PHSEG1   CONFIG_MCP2515_PHASESEG1
#define MCP2515_PHSEG2   CONFIG_MCP2515_PHASESEG2
#define MCP2515_TSEG1    (MCP2515_PROPSEG + MCP2515_PHSEG1)
#define MCP2515_TSEG2    MCP2515_PHSEG2
#define MCP2515_BRP      ((uint8_t)(((float)(MCP2515_CANCLK_FREQUENCY) / \
                         ((float)(MCP2515_TSEG1 + MCP2515_TSEG2 + 1) * \
                         (float)(2 * CONFIG_MCP2515_BITRATE))) - 1))
#define MCP2515_SJW      CONFIG_MCP2515_SJW

#if MCP2515_PROPSEG < 1
#  error Invalid PROPSEG. It cannot be lower than 1
#endif
#if MCP2515_PROPSEG > 8
#  error Invalid PROPSEG. It cannot be greater than 8
#endif
#if MCP2515_PHSEG1 < 1
#  error Invalid PHSEG1. It cannot be lower than 1
#endif
#if MCP2515_PHSEG1 > 8
#  error Invalid PHSEG1. It cannot be greater than 1
#endif
#if MCP2515_TSEG2 < 2
#  error Invalid TSEG2. It cannot be lower than 2
#endif
#if MCP2515_TSEG2 > 8
#  error Invalid TSEG2. It cannot be greater than 8
#endif
#if MCP2515_SJW > 4
#  error Invalid SJW. It cannot be greater than 4
#endif

/* MCP2515 RXB0 element size */

/* MCP2515 RXB1 element size */

/* MCP2515 Filters */

#ifndef CONFIG_MCP2515_NSTDFILTERS
#  define CONFIG_MCP2515_NSTDFILTERS 0
#endif

#if (CONFIG_MCP2515_NSTDFILTERS > 128)
#  error Invalid MCP25150 number of Standard Filters
#endif

#ifndef CONFIG_MCP2515_NEXTFILTERS
#  define CONFIG_MCP2515_NEXTFILTERS 0
#endif

#if (CONFIG_MCP2515_NEXTFILTERS > 64)
#  error Invalid MCP25150 number of Extended Filters
#endif

#define MCP2515_STDFILTER_BYTES \
   MCP2515_ALIGN_UP(CONFIG_MCP2515_NSTDFILTERS << 2)
#define MCP2515_STDFILTER_WORDS (MCP2515_STDFILTER_BYTES >> 2)

#define MCP2515_EXTFILTER_BYTES \
   MCP2515_ALIGN_UP(CONFIG_MCP2515_NEXTFILTERS << 3)
#define MCP2515_EXTFILTER_WORDS (MCP2515_EXTFILTER_BYTES >> 2)

/* MCP25150 TX buffer element size */

/* MCP25150 TX FIFOs */

/* Loopback mode */

#undef MCP2515_LOOPBACK
#if defined(CONFIG_MCP2515_LOOPBACK)
#  define MCP2515_LOOPBACK 1
#endif

/* Interrupts ***************************************************************/

/* Interrupts Errors
 *
 *   MCP2515_INT_MERR  - Message Error Interrupt Flag bit
 *   MCP2515_INT_ERR   - Error Interrupt Flag bit (mult src in EFLG register)
 */

#define MCP2515_ERROR_INTS    (MCP2515_INT_MERR | MCP2515_INT_ERR)

/* RXn buffer interrupts
 *
 *   MCP2515_INT_RX0  - Receive Buffer 0 New Message
 *   MCP2515_INT_RX1  - Receive Buffer 1 New Message
 */

#define MCP2515_RXBUFFER_INTS   (MCP2515_INT_RX0 | MCP2515_INT_RX1)

/* TXn buffer interrupts
 *
 *   MCP2515_INT_TX0  - Transmit Buffer 0 Empty Interrupt
 *   MCP2515_INT_TX1  - Transmit Buffer 1 Empty Interrupt
 *   MCP2515_INT_TX2  - Transmit Buffer 2 Empty Interrupt
 */

#define MCP2515_TXBUFFER_INTS  (MCP2515_INT_TX0 | MCP2515_INT_TX1 | MCP2515_INT_TX2)

/* Helpers ******************************************************************/

#define TXREGVAL(reg) priv->spi_txbuf[reg - MCP2515_TXB0CTRL]

/* Debug ********************************************************************/

/* Debug configurations that may be enabled just for testing MCP2515 */

#ifndef CONFIG_DEBUG_CAN_INFO
#  undef CONFIG_MCP2515_REGDEBUG
#endif

#ifdef CONFIG_MCP2515_REGDEBUG
#  define reginfo caninfo
#else
#  define reginfo(x...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* CAN driver state */

enum can_state_s
{
  MCP2515_STATE_UNINIT = 0,    /* Not yet initialized */
  MCP2515_STATE_RESET,         /* Initialized, reset state */
  MCP2515_STATE_SETUP,         /* can_setup() has been called */
};

/* This structure provides the current state of a CAN peripheral */

struct mcp2515_can_s
{
  FAR struct mcp2515_config_s *config; /* The constant configuration */

  uint8_t state;               /* See enum can_state_s */
  uint8_t nalloc;              /* Number of allocated filters */
  mutex_t lock;                /* Enforces mutually exclusive access */
  sem_t txfsem;                /* Used to wait for TX FIFO availability */
  uint32_t btp;                /* Current bit timing */
  uint8_t rxints;              /* Configured RX interrupts */
  uint8_t txints;              /* Configured TX interrupts */
#ifdef CONFIG_CAN_ERRORS
  uint32_t olderrors;          /* Used to detect the changes in error states */
#endif
  uint8_t filters;             /* Standard/Extende filter bit allocator. */
  uint8_t txbuffers;           /* TX Buffers bit allocator. */

  FAR uint8_t *spi_txbuf;
  FAR uint8_t *spi_rxbuf;
#ifdef CONFIG_MCP2515_REGDEBUG
  uintptr_t regaddr;           /* Last register address read */
  uint32_t regval;             /* Last value read from the register */
  unsigned int count;          /* Number of times that the value was read */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* MCP2515 Register access */

static void mcp2515_readregs(FAR struct mcp2515_can_s *priv, uint8_t regaddr,
              FAR uint8_t *buffer, uint8_t len);
static void mcp2515_writeregs(FAR struct mcp2515_can_s *priv,
              uint8_t regaddr,
              FAR const uint8_t *buffer, uint8_t len);
static void mcp2515_modifyreg(FAR struct mcp2515_can_s *priv,
              uint8_t regaddr, uint8_t mask, uint8_t value);
#ifdef CONFIG_MCP2515_REGDEBUG
static void mcp2515_dumpregs(FAR struct mcp2515_can_s *priv,
              FAR const char *msg);
#else
#  define mcp2515_dumpregs(priv,msg)
#endif

/* MCP2515 helpers */

#ifdef CONFIG_CAN_EXTID
static int mcp2515_add_extfilter(FAR struct mcp2515_can_s *priv,
              FAR struct canioc_extfilter_s *extconfig);
static int mcp2515_del_extfilter(FAR struct mcp2515_can_s *priv, int ndx);
#endif
static int mcp2515_add_stdfilter(FAR struct mcp2515_can_s *priv,
              FAR struct canioc_stdfilter_s *stdconfig);
static int mcp2515_del_stdfilter(FAR struct mcp2515_can_s *priv, int ndx);

/* CAN driver methods */

static void mcp2515_reset(FAR struct can_dev_s *dev);
static int  mcp2515_setup(FAR struct can_dev_s *dev);
static void mcp2515_shutdown(FAR struct can_dev_s *dev);
static void mcp2515_rxint(FAR struct can_dev_s *dev, bool enable);
static void mcp2515_txint(FAR struct can_dev_s *dev, bool enable);
static int  mcp2515_ioctl(FAR struct can_dev_s *dev, int cmd,
              unsigned long arg);
static int  mcp2515_remoterequest(FAR struct can_dev_s *dev, uint16_t id);
static int  mcp2515_send(FAR struct can_dev_s *dev,
              FAR struct can_msg_s *msg);
static bool mcp2515_txready(FAR struct can_dev_s *dev);
static bool mcp2515_txempty(FAR struct can_dev_s *dev);

/* MCP2515 interrupt handling */

#ifdef CONFIG_CAN_ERRORS
static void mcp2515_error(FAR struct can_dev_s *dev, uint8_t status,
              uint8_t oldstatus);
#endif
static void mcp2515_receive(FAR struct can_dev_s *dev, uint8_t offset);
static int  mcp2515_interrupt(FAR struct mcp2515_config_s *config,
             FAR void *arg);

/* Hardware initialization */

static int  mcp2515_hw_initialize(FAR struct mcp2515_can_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct can_ops_s g_mcp2515ops =
{
  mcp2515_reset,         /* co_reset */
  mcp2515_setup,         /* co_setup */
  mcp2515_shutdown,      /* co_shutdown */
  mcp2515_rxint,         /* co_rxint */
  mcp2515_txint,         /* co_txint */
  mcp2515_ioctl,         /* co_ioctl */
  mcp2515_remoterequest, /* co_remoterequest */
  mcp2515_send,          /* co_send */
  mcp2515_txready,       /* co_txready */
  mcp2515_txempty        /* co_txempty */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void mcp2515_read_2regs(FAR struct mcp2515_can_s *priv, uint8_t reg,
                               FAR uint8_t *v1, FAR uint8_t *v2)
{
  priv->spi_txbuf[0] = MCP2515_READ;
  priv->spi_txbuf[1] = reg;

  SPI_LOCK(priv->config->spi, true);
  SPI_SELECT(priv->config->spi, SPIDEV_CANBUS(0), true);
  SPI_EXCHANGE(priv->config->spi, priv->spi_txbuf, priv->spi_rxbuf, 4);
  SPI_SELECT(priv->config->spi, SPIDEV_CANBUS(0), false);
  SPI_LOCK(priv->config->spi, false);

  *v1 = priv->spi_rxbuf[2];
  *v2 = priv->spi_rxbuf[3];
}

/****************************************************************************
 * Name: mcp2515_readregs
 *
 * Description:
 *   Read value(s) of MCP2515 register(s).
 *
 * Input Parameters:
 *   priv - A reference to the MCP2515 peripheral state
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *
 ****************************************************************************/

static void mcp2515_readregs(FAR struct mcp2515_can_s *priv, uint8_t regaddr,
                             FAR uint8_t *buffer, uint8_t len)
{
  FAR struct mcp2515_config_s *config = priv->config;
#ifdef CONFIG_CANBUS_REGDEBUG
  int i;
#endif

  SPI_LOCK(config->spi, true);

  /* Select the MCP2515 */

  SPI_SELECT(config->spi, SPIDEV_CANBUS(0), true);

  /* Send the READ command */

  SPI_SEND(config->spi, MCP2515_READ);

  /* Send register to read and get the next bytes read back */

  SPI_SEND(config->spi, regaddr);
  SPI_RECVBLOCK(config->spi, buffer, len);

  /* Deselect the MCP2515 */

  SPI_SELECT(config->spi, SPIDEV_CANBUS(0), false);

  /* Unlock bus */

  SPI_LOCK(config->spi, false);

#ifdef CONFIG_CANBUS_REGDEBUG
  for (i = 0; i < len; i++)
    {
      caninfo("%02x->%02x\n", regaddr, buffer[i]);
    }
#endif
}

static void mcp2515_transfer(FAR struct mcp2515_can_s *priv, uint8_t len)
{
  FAR struct mcp2515_config_s *config = priv->config;

  SPI_LOCK(config->spi, true);

  /* Select the MCP2515 */

  SPI_SELECT(config->spi, SPIDEV_CANBUS(0), true);

  /* Send the READ command */

  SPI_EXCHANGE(config->spi, priv->spi_txbuf, priv->spi_rxbuf, len);

  /* Deselect the MCP2515 */

  SPI_SELECT(config->spi, SPIDEV_CANBUS(0), false);

  /* Unlock bus */

  SPI_LOCK(config->spi, false);
}

/****************************************************************************
 * Name: mcp2515_writeregs
 *
 * Description:
 *   Set the value of a MCP2515 register.
 *
 * Input Parameters:
 *   priv - A reference to the MCP2515 peripheral state
 *   offset - The offset to the register to write
 *   regval - The value to write to the register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mcp2515_writeregs(FAR struct mcp2515_can_s *priv,
                              uint8_t regaddr,
                              FAR const uint8_t *buffer, uint8_t len)
{
  FAR struct mcp2515_config_s *config = priv->config;
#ifdef CONFIG_CANBUS_REGDEBUG
  int i;

  for (i = 0; i < len; i++)
    {
      caninfo("%02x<-%02x\n", regaddr + i, buffer[i]);
    }
#endif

  SPI_LOCK(config->spi, true);

  /* Select the MCP2515 */

  SPI_SELECT(config->spi, SPIDEV_CANBUS(0), true);

  /* Send the READ command */

  SPI_SEND(config->spi, MCP2515_WRITE);

  /* Send initial register to be written */

  SPI_SEND(config->spi, regaddr);
  SPI_SNDBLOCK(config->spi, buffer, len);

  /* Deselect the MCP2515 */

  SPI_SELECT(config->spi, SPIDEV_CANBUS(0), false);

  /* Unlock bus */

  SPI_LOCK(config->spi, false);
}

/****************************************************************************
 * Name: mcp2515_modifyreg
 *
 * Description:
 *   Modify individuals bits of MCP2515 register
 *
 * Input Parameters:
 *   priv - A reference to the MCP2515 peripheral state
 *   offset - The offset to the register to write
 *   regval - The value to write to the register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mcp2515_modifyreg(FAR struct mcp2515_can_s *priv,
                              uint8_t regaddr, uint8_t mask, uint8_t value)
{
  FAR struct mcp2515_config_s *config = priv->config;
  uint8_t wr[4] =
  {
    MCP2515_BITMOD, regaddr, mask, value
  };

  SPI_LOCK(config->spi, true);

  /* Select the MCP2515 */

  SPI_SELECT(config->spi, SPIDEV_CANBUS(0), true);

  SPI_SNDBLOCK(config->spi, wr, 4);

  /* Deselect the MCP2515 */

  SPI_SELECT(config->spi, SPIDEV_CANBUS(0), false);

  /* Unlock bus */

  SPI_LOCK(config->spi, false);
}

/****************************************************************************
 * Name: mcp2515_dumpregs
 *
 * Description:
 *   Dump the contents of all MCP2515 control registers
 *
 * Input Parameters:
 *   priv - A reference to the MCP2515 peripheral state
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_MCP2515_REGDEBUG
static void mcp2515_dumpregs(FAR struct mcp2515_can_s *priv,
                             FAR const char *msg)
{
  FAR struct mcp2515_config_s *config = priv->config;
}
#endif

/****************************************************************************
 * Name: mcp2515_add_extfilter
 *
 * Description:
 *   Add an address filter for a extended 29 bit address.
 *
 * Input Parameters:
 *   priv      - An instance of the MCP2515 driver state structure.
 *   extconfig - The configuration of the extended filter
 *
 * Returned Value:
 *   A non-negative filter ID is returned on success.  Otherwise a negated
 *   errno value is returned to indicate the nature of the error.
 *
 ****************************************************************************/

#ifdef CONFIG_CAN_EXTID
static int mcp2515_add_extfilter(FAR struct mcp2515_can_s *priv,
                                 FAR struct canioc_extfilter_s *extconfig)
{
  FAR struct mcp2515_config_s *config;
  uint8_t regval;
  uint8_t offset;
  uint8_t mode = CAN_FILTER_MASK;
  int ndx;
  int ret;

  DEBUGASSERT(priv != NULL && priv->config != NULL);
  config = priv->config;

  /* Get exclusive excess to the MCP2515 hardware */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Find an unused standard filter */

  for (ndx = 0; ndx < config->nfilters; ndx++)
    {
      /* Is this filter assigned? */

      if ((priv->filters & (1 << ndx)) == 0)
        {
          /* No, assign the filter */

          DEBUGASSERT(priv->nalloc < priv->config->nfilters);
          priv->filters |= (1 << ndx);
          priv->nalloc++;

          /* Format and write filter */

          DEBUGASSERT(extconfig->xf_id1 <= CAN_MAX_STDMSGID);

          DEBUGASSERT(extconfig->xf_id2 <= CAN_MAX_STDMSGID);

          /* We can reach all RXFn registers (RXFnSIDH, RXFnSIDL,
           * RXFnEID8 and RXFnEID0) using this formula:
           *
           * filterN = RXF0reg + offset + ((priv->nalloc - 1) * 4) ;
           * maskN   = RXM0reg + offset
           */

          if (priv->nalloc <= 3)
            {
              offset = 0;
            }
          else
            {
              offset = 4;
            }

#if 0
          /* N.B. Buffer 0 is higher priority than Buffer 1
           * but to separate these messages we will make this
           * driver more complex. So let to consider that the
           * first 2 IDs inserted in the filter will have more
           * priority than the latest 4 IDs.
           */

          if (extconfig->sf_prio == CAN_MSGPRIO_LOW)
            {
              /* Use RXB1 filters */
            }
          else
            {
              /* Use RXB0 filters */
            }
#endif

          switch (extconfig->xf_type)
            {
              default:
              case CAN_FILTER_DUAL:
                mode = CAN_FILTER_DUAL;
                break;

              case CAN_FILTER_MASK:
                mode = CAN_FILTER_MASK;
                break;

              case CAN_FILTER_RANGE:

                /* Not supported */

                break;
            }

          /* Setup the CONFIG Mode */

          mcp2515_readregs(priv, MCP2515_CANCTRL, &regval, 1);
          regval = (regval & ~CANCTRL_REQOP_MASK) | (CANCTRL_REQOP_CONFIG);
          mcp2515_writeregs(priv, MCP2515_CANCTRL, &regval, 1);

          if (mode == CAN_FILTER_DUAL)
            {
              /* The MSD IDs will be filtered by separated Mask and Filter */

              /* Setup the Filter */

              /* EID0 - EID7 */

              regval = (uint8_t)(extconfig->xf_id1 & 0xff);
              mcp2515_writeregs(priv, MCP2515_RXF0EID0 + offset +
                                ((priv->nalloc - 1) * 4), &regval, 1);

              /* EID8 - EID15 */

              regval = (uint8_t)((extconfig->xf_id1 & 0xff00) >> 8);
              mcp2515_writeregs(priv, MCP2515_RXF0EID8 + offset +
                                ((priv->nalloc - 1) * 4), &regval, 1);

              /* EID16 - EID17 */

              regval = (uint8_t)((extconfig->xf_id1 & 0x30000) >> 16);

              /* STD0 - STD2 */

              regval = (regval) |
                       (uint8_t)(((extconfig->xf_id1 &
                                   0x1c0000) >> 16) << 3);
              mcp2515_writeregs(priv, MCP2515_RXF0SIDL + offset +
                                ((priv->nalloc - 1) * 4), &regval, 1);

              /* STD3 - STD10 */

              regval = (uint8_t)((extconfig->xf_id1 & 0x1fe00000) >> 21);
              regval |= RXFSIDL_EXIDE;
              mcp2515_writeregs(priv, MCP2515_RXF0SIDL + offset +
                                ((priv->nalloc - 1) * 4), &regval, 1);

              /* Setup the Mask */

              /* EID0 - EID7 */

              regval = (uint8_t)(extconfig->xf_id2 & 0xff);
              mcp2515_writeregs(priv, MCP2515_RXM0EID0 + offset, &regval, 1);

              /* EID8 - EID15 */

              regval = (uint8_t)((extconfig->xf_id2 & 0xff00) >> 8);
              mcp2515_writeregs(priv, MCP2515_RXM0EID8 + offset, &regval, 1);

              /* EID16 - EID17 */

              regval = (uint8_t)((extconfig->xf_id2 & 0x30000) >> 16);

              /* STD0 - STD2 */

              regval = (regval) |
                       (uint8_t)(((extconfig->xf_id2 &
                                   0x1c0000) >> 16) << 3);
              mcp2515_writeregs(priv, MCP2515_RXM0SIDL + offset, &regval, 1);

              /* STD3 - STD10 */

              regval = (uint8_t)((extconfig->xf_id2 & 0x1fe00000) >> 21);
              mcp2515_writeregs(priv, MCP2515_RXM0SIDL + offset, &regval, 1);
            }
          else
            {
              /* The IDs will be filtered only by the Filter register
               * (Mask == Filter)
               */

              /* Setup the Filter */

              /* EID0 - EID7 */

              regval = (uint8_t)(extconfig->xf_id1 & 0xff);
              mcp2515_writeregs(priv, MCP2515_RXF0EID0 + offset +
                                ((priv->nalloc - 1) * 4), &regval, 1);
              mcp2515_writeregs(priv, MCP2515_RXM0EID0 + offset, &regval, 1);

              /* EID8 - EID15 */

              regval = (uint8_t)((extconfig->xf_id1 & 0xff00) >> 8);
              mcp2515_writeregs(priv, MCP2515_RXF0EID8 + offset +
                                ((priv->nalloc - 1) * 4), &regval, 1);
              mcp2515_writeregs(priv, MCP2515_RXM0EID8 + offset, &regval, 1);

              /* EID16 - EID17 */

              regval = (uint8_t)((extconfig->xf_id1 & 0x30000) >> 16);

              /* STD0 - STD2 */

              regval = (regval) | (uint8_t)(((extconfig->xf_id1 &
                                0x1c0000) >> 16) << 3) | RXFSIDL_EXIDE;
              mcp2515_writeregs(priv, MCP2515_RXF0SIDL + offset +
                                ((priv->nalloc - 1) * 4), &regval, 1);
              mcp2515_writeregs(priv, MCP2515_RXM0SIDL + offset, &regval, 1);

              /* STD3 - STD10 */

              regval = (uint8_t)((extconfig->xf_id1 & 0x1fe00000) >> 21);
              mcp2515_writeregs(priv, MCP2515_RXF0SIDL + offset +
                                ((priv->nalloc - 1) * 4), &regval, 1);
              mcp2515_writeregs(priv, MCP2515_RXF0SIDL + offset, &regval, 1);
            }

          /* Leave the Configuration mode, Move to Normal mode */

          mcp2515_readregs(priv, MCP2515_CANCTRL, &regval, 1);
          regval = (regval & ~CANCTRL_REQOP_MASK) | (CANCTRL_REQOP_NORMAL);
          mcp2515_writeregs(priv, MCP2515_CANCTRL, &regval, 1);

          nxmutex_unlock(&priv->lock);
          return ndx;
        }
    }

  nxmutex_unlock(&priv->lock);
  return -EAGAIN;
}
#endif

/****************************************************************************
 * Name: mcp2515_del_extfilter
 *
 * Description:
 *   Remove an address filter for a standard 29 bit address.
 *
 * Input Parameters:
 *   priv - An instance of the MCP2515 driver state structure.
 *   ndx  - The filter index previously returned by the
 *          mcp2515_add_extfilter().
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the error.
 *
 ****************************************************************************/

#ifdef CONFIG_CAN_EXTID
static int mcp2515_del_extfilter(FAR struct mcp2515_can_s *priv, int ndx)
{
  FAR struct mcp2515_config_s *config;
  uint8_t regval;
  uint8_t offset;
  int ret;

  DEBUGASSERT(priv != NULL && priv->config != NULL);
  config = priv->config;

  /* Check Userspace Parameters */

  DEBUGASSERT(ndx >= 0 || ndx < config->nfilters);

  caninfo("ndx = %d\n", ndx);

  if (ndx < 0 || ndx >= config->nfilters)
    {
      return -EINVAL;
    }

  /* Get exclusive excess to the MCP2515 hardware */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Check if this filter is really assigned */

  if ((priv->filters & (1 << ndx)) == 0)
    {
      /* No, error out */

      nxmutex_unlock(&priv->lock);
      return -ENOENT;
    }

  /* Release the filter */

  priv->filters &= ~(1 << ndx);

  DEBUGASSERT(priv->nalloc > 0);
  priv->nalloc--;

  /* We can reach all RXFn registers (RXFnSIDH, RXFnSIDL,
   * RXFnEID8 and RXFnEID0) using this formula:
   *
   * filterN = RXF0reg + offset + ((priv->nalloc - 1) * 4) ;
   * maskN   = RXM0reg + offset
   */

  if (ndx < 3)
    {
      offset = 0;
    }
  else
    {
      offset = 4;
    }

  /* Setup the CONFIG Mode */

  mcp2515_readregs(priv, MCP2515_CANCTRL, &regval, 1);
  regval = (regval & ~CANCTRL_REQOP_MASK) | (CANCTRL_REQOP_CONFIG);
  mcp2515_writeregs(priv, MCP2515_CANCTRL, &regval, 1);

  /* Invalidate this filter, set its ID to 0 */

  regval = 0;
  mcp2515_writeregs(priv, MCP2515_RXF0SIDH + offset + (ndx * 4), &regval, 1);
  mcp2515_writeregs(priv, MCP2515_RXF0SIDL + offset + (ndx * 4), &regval, 1);
  mcp2515_writeregs(priv, MCP2515_RXF0EID8 + offset + (ndx * 4), &regval, 1);
  mcp2515_writeregs(priv, MCP2515_RXF0EID0 + offset + (ndx * 4), &regval, 1);

  /* Leave the Configuration mode, Move to Normal mode */

  mcp2515_readregs(priv, MCP2515_CANCTRL, &regval, 1);
  regval = (regval & ~CANCTRL_REQOP_MASK) | (CANCTRL_REQOP_NORMAL);
  mcp2515_writeregs(priv, MCP2515_CANCTRL, &regval, 1);

  nxmutex_unlock(&priv->lock);
  return OK;
}
#endif

/****************************************************************************
 * Name: mcp2515_add_stdfilter
 *
 * Description:
 *   Add an address filter for a standard 11 bit address.
 *
 * Input Parameters:
 *   priv      - An instance of the MCP2515 driver state structure.
 *   stdconfig - The configuration of the standard filter
 *
 * Returned Value:
 *   A non-negative filter ID is returned on success.  Otherwise a negated
 *   errno value is returned to indicate the nature of the error.
 *
 ****************************************************************************/

static int mcp2515_add_stdfilter(FAR struct mcp2515_can_s *priv,
                                 FAR struct canioc_stdfilter_s *stdconfig)
{
  FAR struct mcp2515_config_s *config;
  uint8_t regval;
  uint8_t offset;
  uint8_t mode = CAN_FILTER_MASK;
  int ndx;
  int ret;

  DEBUGASSERT(priv != NULL && priv->config != NULL);
  config = priv->config;

  /* Get exclusive excess to the MCP2515 hardware */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Find an unused standard filter */

  for (ndx = 0; ndx < config->nfilters; ndx++)
    {
      /* Is this filter assigned? */

      if ((priv->filters & (1 << ndx)) == 0)
        {
          /* No, assign the filter */

          DEBUGASSERT(priv->nalloc < priv->config->nfilters);
          priv->filters |= (1 << ndx);
          priv->nalloc++;

          /* Format and write filter */

          DEBUGASSERT(stdconfig->sf_id1 <= CAN_MAX_STDMSGID);

          DEBUGASSERT(stdconfig->sf_id2 <= CAN_MAX_STDMSGID);

          /* We can reach all RXFn registers (RXFnSIDH, RXFnSIDL,
           * RXFnEID8 and RXFnEID0) using this formula:
           *
           * filterN = RXF0reg + offset + ((priv->nalloc - 1) * 4) ;
           * maskN   = RXM0reg + offset
           */

          if (priv->nalloc <= 3)
            {
              offset = 0;
            }
          else
            {
              offset = 4;
            }

#if 0
          /* N.B. Buffer 0 is higher priority than Buffer 1
           * but to separate these messages we will make this
           * driver more complex. So let to consider that the
           * first 2 IDs inserted in the filter will have more
           * priority than the latest 4 IDs.
           */

          if (stdconfig->sf_prio == CAN_MSGPRIO_LOW)
            {
              /* Use RXB1 filters */
            }
          else
            {
              /* Use RXB0 filters */
            }
#endif

          switch (stdconfig->sf_type)
            {
              default:
              case CAN_FILTER_DUAL:
                mode = CAN_FILTER_DUAL;
                break;

              case CAN_FILTER_MASK:
                mode = CAN_FILTER_MASK;
                break;

              case CAN_FILTER_RANGE:

                /* Not supported */

                break;
            }

          /* Setup the CONFIG Mode */

          mcp2515_readregs(priv, MCP2515_CANCTRL, &regval, 1);
          regval = (regval & ~CANCTRL_REQOP_MASK) | (CANCTRL_REQOP_CONFIG);
          mcp2515_writeregs(priv, MCP2515_CANCTRL, &regval, 1);

          if (mode == CAN_FILTER_DUAL)
            {
              /* The MSD IDs will be filtered by separated Mask and Filter */

              /* Setup the Filter */

              regval = (uint8_t)(((stdconfig->sf_id1) & 0x7f8) >> 3);
              mcp2515_writeregs(priv, MCP2515_RXF0SIDH + offset +
                                ((priv->nalloc - 1) * 4), &regval, 1);

              regval = (uint8_t)((stdconfig->sf_id1 & 0x07) << 5);
              mcp2515_writeregs(priv, MCP2515_RXF0SIDL + offset +
                                ((priv->nalloc - 1) * 4), &regval, 1);

              /* Setup the Mask */

              regval = (uint8_t)(((stdconfig->sf_id2) & 0x7f8) >> 3);
              mcp2515_writeregs(priv, MCP2515_RXM0SIDH + offset, &regval, 1);

              regval = (uint8_t)((stdconfig->sf_id2 & 0x07) << 5);
              mcp2515_writeregs(priv, MCP2515_RXM0SIDL + offset, &regval, 1);
            }
          else
            {
              /* The IDs will be filtered only by the Filter register
               * (Mask == Filter)
               */

              /* Setup the Filter */

              regval = (uint8_t) (((stdconfig->sf_id1) & 0x7f8) >> 3);
              mcp2515_writeregs(priv, MCP2515_RXF0SIDH + offset +
                                ((priv->nalloc - 1) * 4), &regval, 1);
              mcp2515_writeregs(priv, MCP2515_RXM0SIDH + offset, &regval, 1);

              regval = (uint8_t)((stdconfig->sf_id1 & 0x07) << 5);
              mcp2515_writeregs(priv, MCP2515_RXF0SIDL + offset +
                                ((priv->nalloc - 1) * 4), &regval, 1);
              mcp2515_writeregs(priv, MCP2515_RXM0SIDL + offset, &regval, 1);
            }

          /* We need to clear the extended ID bits */

          regval = 0;
          mcp2515_writeregs(priv, MCP2515_RXF0EID0 + offset +
                            ((priv->nalloc - 1) * 4), &regval, 1);
          mcp2515_writeregs(priv, MCP2515_RXF0EID8 + offset +
                            ((priv->nalloc - 1) * 4), &regval, 1);
          mcp2515_writeregs(priv, MCP2515_RXM0EID0 + offset, &regval, 1);
          mcp2515_writeregs(priv, MCP2515_RXM0EID8 + offset, &regval, 1);

          /* Leave the Configuration mode, Move to Normal mode */

          mcp2515_readregs(priv, MCP2515_CANCTRL, &regval, 1);
          regval = (regval & ~CANCTRL_REQOP_MASK) | (CANCTRL_REQOP_NORMAL);
          mcp2515_writeregs(priv, MCP2515_CANCTRL, &regval, 1);

          nxmutex_unlock(&priv->lock);
          return ndx;
        }
    }

  nxmutex_unlock(&priv->lock);
  return -EAGAIN;
}

/****************************************************************************
 * Name: mcp2515_del_stdfilter
 *
 * Description:
 *   Remove an address filter for a standard 29 bit address.
 *
 * Input Parameters:
 *   priv - An instance of the MCP2515 driver state structure.
 *   ndx  - The filter index previously returned by the
 *          mcp2515_add_stdfilter().
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the error.
 *
 ****************************************************************************/

static int mcp2515_del_stdfilter(FAR struct mcp2515_can_s *priv, int ndx)
{
  FAR struct mcp2515_config_s *config;
  uint8_t regval;
  uint8_t offset;
  int ret;

  DEBUGASSERT(priv != NULL && priv->config != NULL);
  config = priv->config;

  /* Check Userspace Parameters */

  DEBUGASSERT(ndx >= 0 || ndx < config->nfilters);

  caninfo("ndx = %d\n", ndx);

  if (ndx < 0 || ndx >= config->nfilters)
    {
      return -EINVAL;
    }

  /* Get exclusive excess to the MCP2515 hardware */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Check if this filter is really assigned */

  if ((priv->filters & (1 << ndx)) == 0)
    {
      /* No, error out */

      nxmutex_unlock(&priv->lock);
      return -ENOENT;
    }

  /* Release the filter */

  priv->filters &= ~(1 << ndx);

  DEBUGASSERT(priv->nalloc > 0);
  priv->nalloc--;

  /* We can reach all RXFn registers (RXFnSIDH, RXFnSIDL,
   * RXFnEID8 and RXFnEID0) using this formula:
   *
   * filterN = RXF0reg + offset + ((priv->nalloc - 1) * 4) ;
   * maskN   = RXM0reg + offset
   */

  if (ndx < 3)
    {
      offset = 0;
    }
  else
    {
      offset = 4;
    }

  /* Setup the CONFIG Mode */

  mcp2515_readregs(priv, MCP2515_CANCTRL, &regval, 1);
  regval = (regval & ~CANCTRL_REQOP_MASK) | (CANCTRL_REQOP_CONFIG);
  mcp2515_writeregs(priv, MCP2515_CANCTRL, &regval, 1);

  /* Invalidade this filter, set its ID to 0 */

  regval = 0;
  mcp2515_writeregs(priv, MCP2515_RXF0SIDH + offset + (ndx * 4), &regval, 1);
  mcp2515_writeregs(priv, MCP2515_RXF0SIDL + offset + (ndx * 4), &regval, 1);

  /* Leave the Configuration mode, Move to Normal mode */

  mcp2515_readregs(priv, MCP2515_CANCTRL, &regval, 1);
  regval = (regval & ~CANCTRL_REQOP_MASK) | (CANCTRL_REQOP_NORMAL);
  mcp2515_writeregs(priv, MCP2515_CANCTRL, &regval, 1);

  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: mcp2515_reset_lowlevel
 *
 * Description:
 *   Reset the MCP2515 device.  Called early to initialize the hardware. This
 *   function is called, before mcp2515_setup() and on error conditions.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

static void mcp2515_reset_lowlevel(FAR struct mcp2515_can_s *priv)
{
  FAR struct mcp2515_config_s *config;
  int ret;

  DEBUGASSERT(priv);
  config = priv->config;
  DEBUGASSERT(config);

  UNUSED(config);

  /* Get exclusive access to the MCP2515 peripheral */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return;
    }

  /* Send SPI reset command to MCP2515 */

  SPI_LOCK(config->spi, true);
  SPI_SELECT(config->spi, SPIDEV_CANBUS(0), true);
  SPI_SEND(config->spi, MCP2515_RESET);
  SPI_SELECT(config->spi, SPIDEV_CANBUS(0), false);
  SPI_LOCK(config->spi, false);

  /* Wait 1ms to let MCP2515 restart */

  nxsig_usleep(1000);

  /* Make sure that all buffers are released. */

  nxsem_reset(&priv->txfsem, MCP2515_NUM_TX_BUFFERS);
  priv->txbuffers = 0b111;

  /* Define the current state and unlock */

  priv->state = MCP2515_STATE_RESET;
  nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Name: mcp2515_reset
 *
 * Description:
 *   Reset the MCP2515 device.  Called early to initialize the hardware. This
 *   function is called, before mcp2515_setup() and on error conditions.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

static void mcp2515_reset(FAR struct can_dev_s *dev)
{
  FAR struct mcp2515_can_s *priv;

  DEBUGASSERT(dev);
  priv = dev->cd_priv;
  DEBUGASSERT(priv);

  /* Execute the reset */

  mcp2515_reset_lowlevel(priv);
}

/****************************************************************************
 * Name: mcp2515_setup
 *
 * Description:
 *   Configure the MCP2515. This method is called the first time that the
 *   MCP2515 device is opened.  This will occur when the device file is
 *   first opened.  This setup includes configuring and attaching MCP2515
 *   interrupts.  All MCP2515 interrupts are disabled upon return.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int mcp2515_setup(FAR struct can_dev_s *dev)
{
  FAR struct mcp2515_can_s *priv;
  FAR struct mcp2515_config_s *config;
  int ret;

  DEBUGASSERT(dev);
  priv = dev->cd_priv;
  DEBUGASSERT(priv);
  config = priv->config;
  DEBUGASSERT(config);

  /* Get exclusive access to the MCP2515 peripheral */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* MCP2515 hardware initialization */

  ret = mcp2515_hw_initialize(priv);
  if (ret < 0)
    {
      canerr("ERROR: MCP2515%d H/W initialization failed: %d\n",
            config->devid, ret);
      return ret;
    }

  /* Attach the MCP2515 interrupt handler. */

  ret = config->attach(config, (mcp2515_handler_t)mcp2515_interrupt,
                       (FAR void *)dev);
  if (ret < 0)
    {
      canerr("ERROR: Failed to attach to IRQ Handler!\n");
      return ret;
    }

  /* Enable receive interrupts */

  priv->state = MCP2515_STATE_SETUP;
  mcp2515_rxint(dev, true);

  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: mcp2515_shutdown
 *
 * Description:
 *   Disable the MCP2515.  This method is called when the MCP2515 device is
 *   closed.  This method reverses the operation the setup method.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mcp2515_shutdown(FAR struct can_dev_s *dev)
{
  /* Nothing to do here! */
}

/****************************************************************************
 * Name: mcp2515_rxint
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

static void mcp2515_rxint(FAR struct can_dev_s *dev, bool enable)
{
  FAR struct mcp2515_can_s *priv;
  FAR struct mcp2515_config_s *config;
  irqstate_t flags;

  DEBUGASSERT(dev);
  priv = dev->cd_priv;
  DEBUGASSERT(priv);
  config = priv->config;
  DEBUGASSERT(config);

  caninfo("CAN%d enable: %d\n", config->devid, enable);
  UNUSED(config);

  /* Enable/disable the receive interrupts */

  flags = enter_critical_section();

  if (enable)
    {
      mcp2515_modifyreg(priv, MCP2515_CANINTE,
                        priv->rxints | MCP2515_ERROR_INTS,
                        priv->rxints | MCP2515_ERROR_INTS);
    }
  else
    {
      mcp2515_modifyreg(priv, MCP2515_CANINTE, priv->rxints, ~priv->rxints);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: mcp2515_txint
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

static void mcp2515_txint(FAR struct can_dev_s *dev, bool enable)
{
  FAR struct mcp2515_can_s *priv = dev->cd_priv;
  irqstate_t flags;

  DEBUGASSERT(priv && priv->config);

  caninfo("CAN%d enable: %d\n", priv->config->devid, enable);

  /* Enable/disable the receive interrupts */

  flags = enter_critical_section();

  if (enable)
    {
      mcp2515_modifyreg(priv, MCP2515_CANINTE,
                        priv->txints | MCP2515_ERROR_INTS,
                        priv->txints | MCP2515_ERROR_INTS);
    }
  else
    {
      mcp2515_modifyreg(priv, MCP2515_CANINTE, priv->txints, ~priv->txints);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: mcp2515_ioctl
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

static int mcp2515_ioctl(FAR struct can_dev_s *dev, int cmd,
                         unsigned long arg)
{
  FAR struct mcp2515_can_s *priv;
  int ret = -ENOTTY;

  caninfo("cmd=%04x arg=%lu\n", cmd, arg);

  DEBUGASSERT(dev && dev->cd_priv);
  priv = dev->cd_priv;

  /* Handle the command */

  switch (cmd)
    {
      /* CANIOC_GET_BITTIMING:
       *   Description:    Return the current bit timing settings
       *   Argument:       A pointer to a write-able instance of struct
       *                   canioc_bittiming_s in which current bit timing
       *                   values will be returned.
       *   Returned Value: Zero (OK) is returned on success.  Otherwise -1
       *                   (ERROR) is returned with the errno variable set
       *                   to indicate the nature of the error.
       *   Dependencies:   None
       */

      case CANIOC_GET_BITTIMING:
        {
          FAR struct canioc_bittiming_s *bt =
            (FAR struct canioc_bittiming_s *)arg;
          uint8_t regval;
          uint8_t brp;

          DEBUGASSERT(bt != NULL);

          mcp2515_readregs(priv, MCP2515_CNF1, &regval, 1);
          bt->bt_sjw    = ((regval & CNF1_SJW_MASK) >> CNF1_SJW_SHIFT) + 1;
          brp           = (((regval & CNF1_BRP_MASK) >>
                                      CNF1_BRP_SHIFT) + 1) * 2;

          mcp2515_readregs(priv, MCP2515_CNF2, &regval, 1);
          bt->bt_tseg1  = ((regval & CNF2_PRSEG_MASK) >>
                                     CNF2_PRSEG_SHIFT) + 1;
          bt->bt_tseg1 += ((regval & CNF2_PHSEG1_MASK) >>
                                     CNF2_PHSEG1_SHIFT) + 1;

          mcp2515_readregs(priv, MCP2515_CNF3, &regval, 1);
          bt->bt_tseg2  = ((regval & CNF3_PHSEG2_MASK) >>
                                     CNF3_PHSEG2_SHIFT) + 1;

          bt->bt_baud   = MCP2515_CANCLK_FREQUENCY / brp /
                          (bt->bt_tseg1 + bt->bt_tseg2 + 1);
          ret = OK;
        }
        break;

      /* CANIOC_SET_BITTIMING:
       *   Description:    Set new current bit timing values
       *   Argument:       A pointer to a read-able instance of struct
       *                   canioc_bittiming_s in which the new bit timing
       *                   values are provided.
       *   Returned Value: Zero (OK) is returned on success.  Otherwise -1
       *                   (ERROR) is returned with the errno variable set to
       *                   indicate the nature of the error.
       *   Dependencies:   None
       *
       * REVISIT: There is probably a limitation here:  If there are multiple
       * threads trying to send CAN packets, when one of these threads
       * reconfigures the bitrate, the MCP2515 hardware will be reset and
       * the context of operation will be lost.  Hence, this IOCTL can only
       * safely be executed in quiescent time periods.
       */

      case CANIOC_SET_BITTIMING:
        {
          FAR const struct canioc_bittiming_s *bt =
            (FAR const struct canioc_bittiming_s *)arg;
          irqstate_t flags;
          uint8_t brp;
          uint8_t sjw;
          uint8_t tseg1;
          uint8_t tseg2;
          uint8_t prseg;
          uint8_t phseg1;
          uint8_t regval;

          DEBUGASSERT(bt != NULL);
          DEBUGASSERT(bt->bt_baud < MCP2515_CANCLK_FREQUENCY);
          DEBUGASSERT(bt->bt_sjw > 0 && bt->bt_sjw <= 4);
          DEBUGASSERT(bt->bt_tseg1 > 1 && bt->bt_tseg1 <= 16);
          DEBUGASSERT(bt->bt_tseg2 > 1 && bt->bt_tseg2 <= 8);
          DEBUGASSERT(bt->bt_tseg1 > bt->bt_tseg2);
          DEBUGASSERT(bt->bt_tseg2 > bt->bt_sjw);

          /* Extract bit timing data */

          tseg1  = bt->bt_tseg1 - 1;
          tseg2  = bt->bt_tseg2 - 1;
          sjw   = bt->bt_sjw   - 1;

          /* PRSEG = TSEG1 - PHSEG1
           * Because we don't have PHSEG1 then let us to assume:
           * PHSEG1 == PHSEG2 (PHSEG2 = TSEG2)
           *
           * See more at:
           *  http://www.analog.com/en/analog-dialogue/articles/
           *          configure-can-bit-timing.html
           *
           */

          phseg1 = tseg2;
          prseg  = tseg1 - phseg1;

          brp = (uint32_t)(((float) MCP2515_CANCLK_FREQUENCY /
             ((float)(tseg1 + tseg2 + 1) * (float)(2 * bt->bt_baud))) - 1);

          /* Save the value of the new bit timing register */

          flags = enter_critical_section();

          /* Setup the CONFIG Mode */

          mcp2515_readregs(priv, MCP2515_CANCTRL, &regval, 1);
          regval = (regval & ~CANCTRL_REQOP_MASK) | (CANCTRL_REQOP_CONFIG);
          mcp2515_writeregs(priv, MCP2515_CANCTRL, &regval, 1);

          /* Setup CNF1 register */

          mcp2515_readregs(priv, MCP2515_CNF1, &regval, 1);
          regval = (regval & ~CNF1_BRP_MASK) |
                   (brp << CNF1_BRP_SHIFT);
          regval = (regval & ~CNF1_SJW_MASK) |
                   ((sjw) << CNF1_SJW_SHIFT);
          mcp2515_writeregs(priv, MCP2515_CNF1, &regval, 1);

          /* Setup CNF2 register */

          mcp2515_readregs(priv, MCP2515_CNF2, &regval, 1);
          regval = (regval & ~CNF2_PRSEG_MASK) |
                   ((prseg - 1) << CNF2_PRSEG_SHIFT);
          regval = (regval & ~CNF2_PHSEG1_MASK) |
                   (phseg1 << CNF2_PHSEG1_SHIFT);
          regval = (regval | CNF2_SAM | CNF2_BTLMODE);
          mcp2515_writeregs(priv, MCP2515_CNF2, &regval, 1);

          /* Setup CNF3 register */

          mcp2515_readregs(priv, MCP2515_CNF3, &regval, 1);
          regval = (regval & ~CNF3_PHSEG2_MASK) |
                   (tseg2 << CNF3_PHSEG2_SHIFT);
          regval = (regval | CNF3_SOF);
          mcp2515_writeregs(priv, MCP2515_CNF3, &regval, 1);

          /* Leave the Configuration mode, Move to Normal mode */

          mcp2515_readregs(priv, MCP2515_CANCTRL, &regval, 1);
          regval = (regval & ~CANCTRL_REQOP_MASK) | (CANCTRL_REQOP_NORMAL);
          mcp2515_writeregs(priv, MCP2515_CANCTRL, &regval, 1);

          leave_critical_section(flags);

          ret = OK;
        }
        break;

#ifdef CONFIG_CAN_EXTID
      /* CANIOC_ADD_EXTFILTER:
       *   Description:    Add an address filter for a extended 29 bit
       *                   address.
       *   Argument:       A reference to struct canioc_extfilter_s
       *   Returned Value: A non-negative filter ID is returned on success.
       *                   Otherwise -1 (ERROR) is returned with the errno
       *                   variable set to indicate the nature of the error.
       */

      case CANIOC_ADD_EXTFILTER:
        {
          DEBUGASSERT(arg != 0);
          ret = mcp2515_add_extfilter(priv,
                                      (FAR struct canioc_extfilter_s *)arg);
        }
        break;

      /* CANIOC_DEL_EXTFILTER:
       *   Description:    Remove an address filter for a standard 29 bit
       *                   address.
       *   Argument:       The filter index previously returned by the
       *                   CANIOC_ADD_EXTFILTER command
       *   Returned Value: Zero (OK) is returned on success.  Otherwise -1
       *                   (ERROR) is returned with the errno variable set to
       *                   indicate the nature of the error.
       */

      case CANIOC_DEL_EXTFILTER:
        {
          FAR int *ndx = (FAR int *)((uintptr_t)arg);

          DEBUGASSERT(*ndx <= priv->config->nfilters);
          ret = mcp2515_del_extfilter(priv, (int)*ndx);
        }
        break;
#endif

      /* CANIOC_ADD_STDFILTER:
       *   Description:    Add an address filter for a standard 11 bit
       *                   address.
       *   Argument:       A reference to struct canioc_stdfilter_s
       *   Returned Value: A non-negative filter ID is returned on success.
       *                   Otherwise -1 (ERROR) is returned with the errno
       *                   variable set to indicate the nature of the error.
       */

      case CANIOC_ADD_STDFILTER:
        {
          DEBUGASSERT(arg != 0);
          ret = mcp2515_add_stdfilter(priv,
                                      (FAR struct canioc_stdfilter_s *)arg);
        }
        break;

      /* CANIOC_DEL_STDFILTER:
       *   Description:    Remove an address filter for a standard 11 bit
       *                   address.
       *   Argument:       The filter index previously returned by the
       *                   CANIOC_ADD_STDFILTER command
       *   Returned Value: Zero (OK) is returned on success.  Otherwise -1
       *                   (ERROR) is returned with the errno variable set to
       *                   indicate the nature of the error.
       */

      case CANIOC_DEL_STDFILTER:
        {
          FAR int *ndx = (FAR int *)((uintptr_t)arg);

          DEBUGASSERT(*ndx <= priv->config->nfilters);
          ret = mcp2515_del_stdfilter(priv, (int)*ndx);
        }
        break;

      /* Unsupported/unrecognized command */

      default:
        canerr("ERROR: Unrecognized command: %04x\n", cmd);
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: mcp2515_remoterequest
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

static int mcp2515_remoterequest(FAR struct can_dev_s *dev, uint16_t id)
{
  /* REVISIT:  Remote request not implemented */

  return -ENOSYS;
}

/****************************************************************************
 * Name: mcp2515_send
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

static int mcp2515_send(FAR struct can_dev_s *dev, FAR struct can_msg_s *msg)
{
  FAR struct mcp2515_can_s *priv;
  FAR struct mcp2515_config_s *config;
  uint8_t regval;
  uint8_t txbuf;
  unsigned int nbytes;
  int ret;

  DEBUGASSERT(dev);
  priv = dev->cd_priv;
  DEBUGASSERT(priv && priv->config);
  config = priv->config;

  caninfo("CAN%d\n", config->devid);
  caninfo("CAN%d ID: %" PRId32 " DLC: %d\n",
          config->devid, (uint32_t)msg->cm_hdr.ch_id, msg->cm_hdr.ch_dlc);
  UNUSED(config);

  /* Get exclusive access to the MCP2515 peripheral */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Acquire buffer */

  ret = nxsem_wait(&priv->txfsem);
  if (ret < 0)
    {
      nxmutex_unlock(&priv->lock);
      return ret;
    }

  DEBUGASSERT(priv->txbuffers != 0);

  txbuf = ffs(priv->txbuffers) - 1;
  priv->txbuffers &= ~(1 << txbuf);

  /* Select one empty transmit buffer */

  /* Setup the MCP2515 TX Buffer with the message to send */

#ifdef CONFIG_CAN_EXTID
  if (msg->cm_hdr.ch_extid)
    {
      DEBUGASSERT(msg->cm_hdr.ch_id <= CAN_MAX_EXTMSGID);

      /* EID7 - EID0 */

      regval = (msg->cm_hdr.ch_id & 0xff);
      TXREGVAL(MCP2515_TXB0EID0) = regval;

      /* EID15 - EID8 */

      regval = (msg->cm_hdr.ch_id & 0xff00) >> 8;
      TXREGVAL(MCP2515_TXB0EID8) = regval;

      /* EID17 and EID16 */

      regval  = (msg->cm_hdr.ch_id & 0x30000) >> 16;
      regval |= TXBSIDL_EXIDE;

      /* STD2 - STD0 */

      regval |= (msg->cm_hdr.ch_id & 0x1c0000) >> 13;
      TXREGVAL(MCP2515_TXB0SIDL) = regval;

      /* STD10 - STD3 */

      regval  = (msg->cm_hdr.ch_id & 0x1fe00000) >> 21;
      TXREGVAL(MCP2515_TXB0SIDH) = regval;
    }
  else
#endif
    {
      DEBUGASSERT(msg->cm_hdr.ch_id <= CAN_MAX_STDMSGID);

      /* Setup the Standard ID of the message to send */

      /* STD10 - STD3 */

      regval  = (msg->cm_hdr.ch_id & 0x7f8) >> 3;
      TXREGVAL(MCP2515_TXB0SIDH) = regval;

      /* STD2 - STD0 */

      regval  = (msg->cm_hdr.ch_id & 0x007) << 5;
      TXREGVAL(MCP2515_TXB0SIDL) = regval;
    }

  /* Setup the DLC */

  regval = (msg->cm_hdr.ch_dlc & 0xf);

  if (msg->cm_hdr.ch_rtr)
    {
      regval |= TXBDLC_RTR;
    }

  TXREGVAL(MCP2515_TXB0DLC) = regval;

  /* Fill the data buffer */

  nbytes = msg->cm_hdr.ch_dlc;

  memcpy(&TXREGVAL(MCP2515_TXB0D0), msg->cm_data, nbytes);

  TXREGVAL(MCP2515_TXB0CTRL) = MCP2515_LOAD_TXB(txbuf);

  mcp2515_transfer(priv, SPI_TRANSFER_BUF_LEN);

  /* Request to send */

  priv->spi_txbuf[0] = MCP2515_RTS(txbuf);
  mcp2515_transfer(priv, 1);

  nxmutex_unlock(&priv->lock);

  /* Report that the TX transfer is complete to the upper half logic.  Of
   * course, the transfer is not complete, but this early notification
   * allows the upper half logic to free resources sooner.
   *
   * REVISIT:  Should we disable interrupts?  can_txdone() was designed to
   * be called from an interrupt handler and, hence, may be unsafe when
   * called from the tasking level.
   */

  can_txdone(dev);
  return OK;
}

/****************************************************************************
 * Name: mcp2515_txready
 *
 * Description:
 *   Return true if the MCP2515 hardware can accept another TX message.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   True if the MCP2515 hardware is ready to accept another TX message.
 *
 ****************************************************************************/

static bool mcp2515_txready(FAR struct can_dev_s *dev)
{
  FAR struct mcp2515_can_s *priv;

  DEBUGASSERT(dev);
  priv = dev->cd_priv;
  DEBUGASSERT(priv);

  return priv->txbuffers != 0;
}

/****************************************************************************
 * Name: mcp2515_txempty
 *
 * Description:
 *   Return true if all message have been sent.  If for example, the MCP2515
 *   hardware implements FIFOs, then this would mean the transmit FIFO is
 *   empty.  This method is called when the driver needs to make sure that
 *   all characters are "drained" from the TX hardware before calling
 *   co_shutdown().
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   True if there are no pending TX transfers in the MCP2515 hardware.
 *
 ****************************************************************************/

static bool mcp2515_txempty(FAR struct can_dev_s *dev)
{
  FAR struct mcp2515_can_s *priv;

  DEBUGASSERT(dev);
  priv = dev->cd_priv;
  DEBUGASSERT(priv);

  return priv->txbuffers == (1 << MCP2515_NUM_TX_BUFFERS) - 1;
}

/****************************************************************************
 * Name: mcp2515_error
 *
 * Description:
 *   Report a CAN error
 *
 * Input Parameters:
 *   dev        - CAN-common state data
 *   status     - Interrupt status with error bits set
 *   oldstatus  - Previous Interrupt status with error bits set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_CAN_ERRORS
static void mcp2515_error(FAR struct can_dev_s *dev, uint8_t status,
                       uint8_t oldstatus)
{
  FAR struct mcp2515_can_s *priv = dev->cd_priv;
  struct can_hdr_s hdr;
  uint8_t eflg;
  uint8_t txerr;
  uint8_t txb0err;
  uint8_t txb1err;
  uint8_t txb2err;
  uint16_t errbits;
  uint8_t data[CAN_ERROR_DLC];
  int ret;

  /* Encode error bits */

  errbits = 0;
  memset(data, 0, sizeof(data));

  /* Please note that MCP2515_CANINTF only reports if an error
   * happened. It doesn't report what error it is.
   * We need to check EFLG and TXBnCTRL to discover.
   */

  mcp2515_readregs(priv, MCP2515_EFLG, &eflg, 1);
  if (eflg & EFLG_TXBO)
    {
      errbits |= CAN_ERROR_BUSOFF;
    }

  if (eflg & EFLG_RXEP)
    {
      data[1] |= CAN_ERROR1_RXPASSIVE;
    }

  if (eflg & EFLG_TXEP)
    {
      data[1] |= CAN_ERROR1_TXPASSIVE;
    }

  if (eflg & EFLG_RXWAR)
    {
      data[1] |= CAN_ERROR1_RXWARNING;
    }

  if (eflg & EFLG_TXWAR)
    {
      data[1] |= CAN_ERROR1_TXWARNING;
    }

  if (eflg & (EFLG_RX0OVR | EFLG_RX1OVR))
    {
      data[1] |= CAN_ERROR1_RXOVERFLOW;
    }

  /* Verify Message Error */

  mcp2515_readregs(priv, MCP2515_TXB0CTRL, &txb0err, 1);
  mcp2515_readregs(priv, MCP2515_TXB1CTRL, &txb1err, 1);
  mcp2515_readregs(priv, MCP2515_TXB2CTRL, &txb2err, 1);

  txerr = txb0err | txb1err | txb2err;

  if (txerr & (TXBCTRL_MLOA))
    {
      errbits |= CAN_ERROR_LOSTARB;
    }

  if (txerr & (TXBCTRL_ABTF))
    {
      errbits |= CAN_ERROR_LOSTARB;
    }

  if (txerr & (TXBCTRL_MLOA))
    {
      data[0] |= CAN_ERROR0_UNSPEC;
    }

  if ((status & (MCP2515_INT_ERR | MCP2515_INT_MERR)) != 0)
    {
      /* If Message Error or Other error */

      errbits |= CAN_ERROR_CONTROLLER;
    }
  else if ((oldstatus & (MCP2515_INT_ERR | MCP2515_INT_MERR)) != 0)
    {
      errbits |= CAN_ERROR_CONTROLLER;
    }

  if (errbits != 0)
    {
      /* Format the CAN header for the error report. */

      hdr.ch_id     = errbits;
      hdr.ch_dlc    = CAN_ERROR_DLC;
      hdr.ch_rtr    = 0;
      hdr.ch_error  = 1;
#ifdef CONFIG_CAN_EXTID
      hdr.ch_extid  = 0;
#endif
      hdr.ch_unused = 0;

      /* And provide the error report to the upper half logic */

      ret = can_receive(dev, &hdr, data);
      if (ret < 0)
        {
          canerr("ERROR: can_receive failed: %d\n", ret);
        }
    }
}
#endif /* CONFIG_CAN_ERRORS */

/****************************************************************************
 * Name: mcp2515_receive
 *
 * Description:
 *   Receive an MCP2515 messages
 *
 * Input Parameters:
 *   dev      - CAN-common state data
 *   rxbuffer - The RX buffer containing the received messages
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#define RXREGVAL(reg) priv->spi_rxbuf[reg-MCP2515_RXB0CTRL]

static void mcp2515_receive(FAR struct can_dev_s *dev, uint8_t offset)
{
  FAR struct mcp2515_can_s *priv;
  struct can_hdr_s hdr;
  int ret;
  uint8_t regval;

  DEBUGASSERT(dev);
  priv = dev->cd_priv;
  DEBUGASSERT(priv);

  mcp2515_readregs(priv, MCP2515_RXB0CTRL + offset, priv->spi_rxbuf,
                   SPI_TRANSFER_BUF_LEN);

  regval = RXREGVAL(MCP2515_RXB0SIDL);

#ifdef CONFIG_CAN_EXTID
  if ((regval & RXBSIDL_IDE) != 0)
    {
      /* Save the extended ID of the newly received message */

      /* EID7 - EID0 */

      regval     = RXREGVAL(MCP2515_RXB0EID0);
      hdr.ch_id  = regval ;

      /* EID15 - EID8 */

      regval     = RXREGVAL(MCP2515_RXB0EID8);
      hdr.ch_id  = hdr.ch_id | (regval << 8);

      /* EID17 and EID16 */

      regval     = RXREGVAL(MCP2515_RXB0SIDL);
      hdr.ch_id  = hdr.ch_id | ((regval & RXBSIDL_EID_MASK) << 16);

      /* STD2 - STD0 */

      hdr.ch_id  = hdr.ch_id | ((regval >> 5) << 18);

      /* STD10 - STD3 */

      regval     = RXREGVAL(MCP2515_RXB0SIDH);
      hdr.ch_id  = hdr.ch_id | (regval << 21);

      hdr.ch_extid = true;
    }
  else
    {
      /* Save the standard ID of the newly received message */

      regval = RXREGVAL(MCP2515_RXB0SIDH);
      hdr.ch_id = regval;

      regval = RXREGVAL(MCP2515_RXB0SIDL);
      hdr.ch_id = (hdr.ch_id << 3) | (regval >> 5);
      hdr.ch_extid = false;
    }

#else
  if ((regval & RXBSIDL_IDE) != 0)
    {
      /* Drop any messages with extended IDs */

      canerr("ERROR: Extended MSG in Standard Mode\n");

      return;
    }

  /* Save the standard ID of the newly received message */

  regval    = RXREGVAL(MCP2515_RXB0SIDH);
  hdr.ch_id = regval;
  regval    = RXREGVAL(MCP2515_RXB0SIDL);
  hdr.ch_id = (hdr.ch_id << 3) | (regval >> 5);
#endif

#ifdef CONFIG_CAN_ERRORS
  hdr.ch_error  = 0; /* Error reporting not supported */
#endif
  hdr.ch_unused = 0;

  /* Extract the RTR bit */

  regval = RXREGVAL(MCP2515_RXB0CTRL);
  hdr.ch_rtr = (regval & RXBCTRL_RXRTR) != 0;

  /* Get the DLC */

  regval = RXREGVAL(MCP2515_RXB0DLC);
  hdr.ch_dlc = (regval & RXBDLC_DLC_MASK) >> RXBDLC_DLC_SHIFT;

  /* Save the message data */

  ret = can_receive(dev, &hdr, (FAR uint8_t *)&RXREGVAL(MCP2515_RXB0D0));

  if (ret < 0)
    {
      canerr("ERROR: can_receive failed: %d\n", ret);
    }
}

/****************************************************************************
 * Name: mcp2515_interrupt
 *
 * Description:
 *   Common MCP2515 interrupt handler
 *
 * Input Parameters:
 *   dev - CAN-common state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int mcp2515_interrupt(FAR struct mcp2515_config_s *config,
                             FAR void *arg)
{
  FAR struct can_dev_s *dev = (FAR struct can_dev_s *)arg;
  FAR struct mcp2515_can_s *priv;
  uint8_t ir;
  uint8_t ie;
  uint8_t pending;
  uint8_t clrmask;
  bool handled;

  DEBUGASSERT(dev);
  priv = dev->cd_priv;
  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(priv && priv->config);

  /* Loop while there are pending interrupt events */

  do
    {
      /* Get the set of pending interrupts. */

      mcp2515_read_2regs(priv, MCP2515_CANINTE, &ie, &ir);

      pending = (ir & ie);
      handled = false;
      clrmask = 0;

      if (pending == 0)
        {
          return OK;
        }

      /* Check for any errors */

      if ((pending & MCP2515_ERROR_INTS) != 0)
        {
          /* Clear interrupt errors */

          pending &= ~MCP2515_ERROR_INTS;
          clrmask |= MCP2515_ERROR_INTS;

#ifdef CONFIG_CAN_ERRORS
          /* Report errors */

          mcp2515_error(dev, pending & MCP2515_ERROR_INTS, priv->olderrors);

          priv->olderrors = (pending & MCP2515_ERROR_INTS);
#endif
          handled = true;
        }
#ifdef CONFIG_CAN_ERRORS
      else if (priv->olderrors != 0)
        {
          /* All (old) errors cleared  */

          canerr("ERROR: CLEARED\n");

          mcp2515_error(dev, 0, priv->olderrors);

          priv->olderrors = 0;
          handled = true;
        }
#endif

      /* Check for successful completion of a transmission */

      if ((pending & MCP2515_TXBUFFER_INTS) != 0)
        {
          /* Clear the pending TX completion interrupt (and all
           * other TX-related interrupts)
           */

          if (pending & MCP2515_INT_TX0)
            {
              caninfo("TX0 is empty to transmit new message!\n");

              /* Clear TX0 interrupt */

              priv->txbuffers |= (1 << 0);
              nxsem_post(&priv->txfsem);
              pending &= ~MCP2515_INT_TX0;
              clrmask |= MCP2515_INT_TX0;
            }

          if (pending & MCP2515_INT_TX1)
            {
              caninfo("TX1 is empty to transmit new message!\n");

              priv->txbuffers |= (1 << 1);
              nxsem_post(&priv->txfsem);

              /* Clear TX1 interrupt */

              pending &= ~MCP2515_INT_TX1;
              clrmask |= MCP2515_INT_TX1;
            }

          if (pending & MCP2515_INT_TX2)
            {
              caninfo("TX2 is empty to transmit new message!\n");

              priv->txbuffers |= (1 << 2);
              nxsem_post(&priv->txfsem);

              /* Clear TX2 interrupt */

              pending &= ~MCP2515_INT_TX2;
              clrmask |= MCP2515_INT_TX2;
            }

          handled = true;

#ifdef CONFIG_CAN_TXREADY
          /* Inform the upper half driver that we are again ready to accept
           * data in mcp2515_send().
           */

          can_txready(dev);
#endif
        }
      else if ((pending & priv->txints) != 0)
        {
          /* Clear unhandled TX events */

          handled = true;
        }

      /* Check if there is a new message to read */

      if ((pending & MCP2515_RXBUFFER_INTS) != 0)
        {
          /* RX Buffer 0 is the "high priority" buffer:  We will process
           * all messages in RXB0 before processing any message from RX
           * RXB1.
           */

          if ((pending & MCP2515_INT_RX0) != 0)
            {
              mcp2515_receive(dev, MCP2515_RX0_OFFSET);

              /* Clear RX0 interrupt */

              pending &= ~MCP2515_INT_RX0;
              clrmask |= MCP2515_INT_RX0;
            }
          else
            {
              if ((pending & MCP2515_INT_RX1) != 0)
                {
                  mcp2515_receive(dev, MCP2515_RX1_OFFSET);

                  /* Clear RX1 interrupt */

                  pending &= ~MCP2515_INT_RX1;
                  clrmask |= MCP2515_INT_RX1;
                }
            }

          /* Acknowledge reading the FIFO entry */

          handled = true;
        }

      mcp2515_modifyreg(priv, MCP2515_CANINTF, clrmask, pending);
    }
  while (handled);

  return OK;
}

/****************************************************************************
 * Name: mcp2515_hw_initialize
 *
 * Description:
 *   MCP2515 hardware initialization
 *
 * Input Parameters:
 *   priv - A pointer to the private data structure for this MCP2515
 *          peripheral
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int mcp2515_hw_initialize(struct mcp2515_can_s *priv)
{
  FAR struct mcp2515_config_s *config = priv->config;
  uint8_t regval;

  caninfo("CAN%d\n", config->devid);
  UNUSED(config);

  /* Setup CNF1 register */

  mcp2515_readregs(priv, MCP2515_CNF1, &regval, 1);
  regval = (regval & ~CNF1_BRP_MASK) |
           (MCP2515_BRP << CNF1_BRP_SHIFT);
  regval = (regval & ~CNF1_SJW_MASK) |
           ((MCP2515_SJW - 1) << CNF1_SJW_SHIFT);
  mcp2515_writeregs(priv, MCP2515_CNF1, &regval, 1);

  /* Setup CNF2 register */

  mcp2515_readregs(priv, MCP2515_CNF2, &regval, 1);
  regval = (regval & ~CNF2_PRSEG_MASK) |
           ((MCP2515_PROPSEG - 1) << CNF2_PRSEG_SHIFT);
  regval = (regval & ~CNF2_PHSEG1_MASK) |
           ((MCP2515_PHSEG1 - 1) << CNF2_PHSEG1_SHIFT);
  regval = (regval | CNF2_SAM | CNF2_BTLMODE);
  mcp2515_writeregs(priv, MCP2515_CNF2, &regval, 1);

  /* Setup CNF3 register */

  mcp2515_readregs(priv, MCP2515_CNF3, &regval, 1);
  regval = (regval & ~CNF3_PHSEG2_MASK) |
           ((MCP2515_PHSEG2 - 1) << CNF3_PHSEG2_SHIFT);
  regval = (regval | CNF3_SOF);
  mcp2515_writeregs(priv, MCP2515_CNF3, &regval, 1);

  /* Mask all messages to be received */

  mcp2515_readregs(priv, MCP2515_RXB0CTRL, &regval, 1);
  regval = (regval & ~RXBCTRL_RXM_MASK) |
           (RXBCTRL_RXM_ALLVALID << RXBCTRL_RXM_SHIFT);
  regval = (regval | RXB0CTRL_BUKT); /* Enable Rollover from RXB0 to RXB1 */
  mcp2515_writeregs(priv, MCP2515_RXB0CTRL, &regval, 1);

  mcp2515_readregs(priv, MCP2515_RXB1CTRL, &regval, 1);
  regval = (regval & ~RXBCTRL_RXM_MASK) |
           (RXBCTRL_RXM_ALLVALID << RXBCTRL_RXM_SHIFT);
  mcp2515_writeregs(priv, MCP2515_RXB1CTRL, &regval, 1);

  regval = 0x00;
  mcp2515_writeregs(priv, MCP2515_RXM0SIDH, &regval, 1);
  mcp2515_writeregs(priv, MCP2515_RXM0SIDL, &regval, 1);
#ifdef CONFIG_CAN_EXTID
  mcp2515_writeregs(priv, MCP2515_RXM0EID8, &regval, 1);
  mcp2515_writeregs(priv, MCP2515_RXM0EID0, &regval, 1);
#endif

  mcp2515_writeregs(priv, MCP2515_RXM1SIDH, &regval, 1);
  mcp2515_writeregs(priv, MCP2515_RXM1SIDL, &regval, 1);
#ifdef CONFIG_CAN_EXTID
  mcp2515_writeregs(priv, MCP2515_RXM1EID8, &regval, 1);
  mcp2515_writeregs(priv, MCP2515_RXM1EID0, &regval, 1);
#endif

#ifdef CONFIG_CAN_EXTID
  mcp2515_modifyreg(priv, MCP2515_RXM0SIDL, RXFSIDL_EXIDE, RXFSIDL_EXIDE);
  mcp2515_modifyreg(priv, MCP2515_RXM1SIDL, RXFSIDL_EXIDE, RXFSIDL_EXIDE);
#endif

  /* Leave the Configuration mode, Move to Normal mode */

  mcp2515_readregs(priv, MCP2515_CANCTRL, &regval, 1);
  regval = (regval & ~CANCTRL_REQOP_MASK) | (CANCTRL_REQOP_NORMAL);
  mcp2515_writeregs(priv, MCP2515_CANCTRL, &regval, 1);

  nxsig_usleep(100);

  /* Read the CANINTF */

  mcp2515_readregs(priv, MCP2515_CANINTF, &regval, 1);
  caninfo("CANINFT = 0x%02X\n", regval);

#ifdef MCP2515_LOOPBACK
  /* Is loopback mode selected for this peripheral? */

  if (config->loopback)
    {
      /* To Be Implemented */
    }
#endif

  /* Configure interrupt lines */

  /* Select RX-related interrupts */

  priv->rxints = MCP2515_RXBUFFER_INTS;

  /* Select TX-related interrupts */

  priv->txints = MCP2515_TXBUFFER_INTS;

  /* In this option we set a special receive mode in the
   * RXM[1:0] bits (RXBnCTRL[6:5]). In both registers:
   * RXB0CTRL and RXB1CTRL.
   * 11 = Turns mask/filters off; receives any message.
   *
   * In this mode it is tested that it receives both
   * extended and standard id messages.
   */

#ifdef CONFIG_CAN_EXTID
  mcp2515_readregs(priv, MCP2515_RXB0CTRL, &regval, 1);
  regval &= ~RXBCTRL_RXM_ALLMSG;
  regval |= RXBCTRL_RXM_ALLMSG;
  mcp2515_writeregs(priv, MCP2515_RXB0CTRL, &regval, 1);

  mcp2515_readregs(priv, MCP2515_RXB1CTRL, &regval, 1);
  regval &= ~RXBCTRL_RXM_ALLMSG;
  regval |= RXBCTRL_RXM_ALLMSG;
  mcp2515_writeregs(priv, MCP2515_RXB1CTRL, &regval, 1);
#endif

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mcp2515_instantiate
 *
 * Description:
 *   Initialize the selected MCP2515 CAN Bus Controller over SPI
 *
 * Input Parameters:
 *   config - The configuration structure passed by the board.
 *
 * Returned Value:
 *   Valid CAN device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct mcp2515_can_s *
  mcp2515_instantiate(FAR struct mcp2515_config_s *config)
{
  FAR struct mcp2515_can_s *priv;
  uint8_t  canctrl;

  caninfo("Starting mcp2515_instantiate()!\n");

  priv = (FAR struct mcp2515_can_s *)
    kmm_malloc(sizeof(struct mcp2515_can_s));
  if (priv == NULL)
    {
      canerr("ERROR: Failed to allocate instance of mcp2515_can_s!\n");
      return NULL;
    }

  /* Setup SPI frequency and mode */

  SPI_SETFREQUENCY(config->spi, CONFIG_MCP2515_SPI_SCK_FREQUENCY);
  SPI_SETMODE(config->spi, MCP2515_SPI_MODE);
  SPI_SETBITS(config->spi, 8);
  SPI_HWFEATURES(config->spi, 0);

  /* Perform one time data initialization */

  memset(priv, 0, sizeof(struct mcp2515_can_s));
  priv->config = config;

  priv->spi_txbuf = kmm_zalloc(SPI_TRANSFER_BUF_LEN);
  priv->spi_rxbuf = kmm_zalloc(SPI_TRANSFER_BUF_LEN);

  /* Set the initial bit timing.  This might change subsequently
   * due to IOCTL command processing.
   */

  /* Initialize mutex & semaphores */

  nxmutex_init(&priv->lock);
  nxsem_init(&priv->txfsem, 0, MCP2515_NUM_TX_BUFFERS);

  /* Initialize bitmask */

  priv->txbuffers = (1 << MCP2515_NUM_TX_BUFFERS)-1;

  /* And put the hardware in the initial state */

  mcp2515_reset_lowlevel(priv);

  /* Probe the MCP2515 to confirm it was detected */

  mcp2515_readregs(priv, MCP2515_CANCTRL, &canctrl, 1);

  if (canctrl != DEFAULT_CANCTRL_CONFMODE)
    {
      canerr("ERROR: CANCTRL = 0x%02X ! It should be 0x87\n", canctrl);
      nxmutex_destroy(&priv->lock);
      kmm_free(priv);
      return NULL;
    }

  /* Return our private data structure as an opaque handle */

  return priv;
}

/****************************************************************************
 * Name: mcp2515_initialize
 *
 * Description:
 *   Initialize the selected MCP2515 CAN Bus Controller over SPI
 *
 * Input Parameters:
 *   config - The configuration structure passed by the board.
 *
 * Returned Value:
 *   Valid CAN device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct can_dev_s *mcp2515_initialize(
    FAR struct mcp2515_can_s *mcp2515can)
{
  FAR struct can_dev_s *dev;

  caninfo("Starting mcp2515_initialize()!\n");

  /* Allocate a CAN Device structure */

  dev = (FAR struct can_dev_s *)kmm_zalloc(sizeof(struct can_dev_s));
  if (dev == NULL)
    {
      canerr("ERROR: Failed to allocate instance of can_dev_s!\n");
      return NULL;
    }

  dev->cd_ops  = &g_mcp2515ops;
  dev->cd_priv = (FAR void *)mcp2515can;

  return dev;
}

#endif /* CONFIG_CAN && CONFIG_MCP2515 */
