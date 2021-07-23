/****************************************************************************
 * arch/arm/src/stm32h7/stm32_spi_slave.c
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
#include <stddef.h>
#include <semaphore.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>
#include <nuttx/spi/slave.h>
#include <nuttx/power/pm.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "arm_arch.h"

#include "chip.h"
#include "stm32_rcc.h"
#include "stm32_gpio.h"
#include "stm32_spi.h"
#include "stm32_dma.h"

#if defined(CONFIG_STM32H7_SPI1_SLAVE) ||                               \
  defined(CONFIG_STM32H7_SPI2_SLAVE) ||                                 \
  defined(CONFIG_STM32H7_SPI3_SLAVE) ||                                 \
  defined(CONFIG_STM32H7_SPI4_SLAVE) ||                                 \
  defined(CONFIG_STM32H7_SPI5_SLAVE) ||                                 \
  defined(CONFIG_STM32H7_SPI6_SLAVE)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* SPI interrupts */

#ifdef CONFIG_STM32H7_SPI_INTERRUPTS
#  error "Interrupt driven SPI not yet supported"
#endif

/* Can't have both interrupt driven SPI and SPI DMA */

#if defined(CONFIG_STM32H7_SPI_INTERRUPTS) && defined(CONFIG_STM32H7_SPI_DMA)
#  error "Cannot enable both interrupt mode and DMA mode for SPI"
#endif

/* SPI DMA priority */

#ifdef CONFIG_STM32H7_SPI_DMA

#  if defined(CONFIG_SPI_DMAPRIO)
#    define SPI_DMA_PRIO  CONFIG_SPI_DMAPRIO
#  elif defined(DMA_SCR_PRIMED)
#    define SPI_DMA_PRIO  DMA_SCR_PRILO
#  else
#    error "Unknown STM32 DMA"
#  endif

#  if (SPI_DMA_PRIO & ~DMA_SCR_PL_MASK) != 0
#    error "Illegal value for CONFIG_SPI_DMAPRIO"
#  endif

/* DMA channel configuration */

#  define SPI_RXDMA16_CONFIG        (SPI_DMA_PRIO | DMA_SCR_MSIZE_16BITS | \
                                     DMA_SCR_PSIZE_16BITS | DMA_SCR_MINC | \
                                     DMA_SCR_DIR_P2M | DMA_SCR_CIRC)
#  define SPI_RXDMA8_CONFIG         (SPI_DMA_PRIO | DMA_SCR_MSIZE_8BITS | \
                                     DMA_SCR_PSIZE_8BITS | DMA_SCR_MINC| \
                                     DMA_SCR_DIR_P2M | DMA_SCR_CIRC)
#  define SPI_TXDMA16_CONFIG        (SPI_DMA_PRIO | DMA_SCR_MSIZE_16BITS| \
                                     DMA_SCR_PSIZE_16BITS | DMA_SCR_MINC| \
                                     DMA_SCR_DIR_M2P)
#  define SPI_TXDMA8_CONFIG         (SPI_DMA_PRIO | DMA_SCR_MSIZE_8BITS|  \
                                     DMA_SCR_PSIZE_8BITS | DMA_SCR_MINC|DMA_SCR_DIR_M2P)
#endif

/* Kernel clock configuration
 * TODO:
 *  - support for all kernel clock configuration
 */

#if defined(CONFIG_STM32H7_SPI1_SLAVE) || defined(CONFIG_STM32H7_SPI2_SLAVE) || \
  defined(CONFIG_STM32H7_SPI3_SLAVE)
#  if STM32_RCC_D2CCIP1R_SPI123SRC == RCC_D2CCIP1R_SPI123SEL_PLL1
#    define SPI123_KERNEL_CLOCK_FREQ STM32_PLL1Q_FREQUENCY
#  else
#    error Not supported yet
#  endif
#  if SPI123_KERNEL_CLOCK_FREQ > 200000000
#    error Not supported SPI123 frequency
#  endif
#endif  /* SPI123 */

#if defined(CONFIG_STM32H7_SPI4_SLAVE) || defined(CONFIG_STM32H7_SPI5_SLAVE)
#  if STM32_RCC_D2CCIP1R_SPI45SRC == RCC_D2CCIP1R_SPI45SEL_APB
#    define SPI45_KERNEL_CLOCK_FREQ STM32_PCLK2_FREQUENCY
#  else
#    error Not supported yet
#  endif
#  if SPI45_KERNEL_CLOCK_FREQ > 100000000
#    error Not supported SPI45 frequency
#  endif
#endif  /* SPI45 */

#if defined(CONFIG_STM32H7_SPI6_SLAVE)
#  if STM32_RCC_D3CCIPR_SPI6SRC == RCC_D3CCIPR_SPI6SEL_PCLK4
#    define SPI6_KERNEL_CLOCK_FREQ STM32_PCLK4_FREQUENCY
#  else
#    error Not supported yet
#  endif
#  if SPI6_KERNEL_CLOCK_FREQ > 100000000
#    error Not supported SPI6 frequency
#  endif
#endif  /* SPI6 */

#if defined (CONFIG_STM32H7_SPI_SLAVE_QSIZE)
#  if CONFIG_STM32H7_SPI_SLAVE_QSIZE > 65535
#    error CONFIG_STM32H7_SPI_SLAVE_QSIZE too large
#  endif
#endif

/* SPI6 is in D3 domain and is not yet supported. Remove this when the proper
 * support is in place
 */

#if defined (CONFIG_STM32H7_SPI6_SLAVE)
#    error SPI6 slave not supported yet
#endif

#define DMA_BUFFER_MASK    (ARMV7M_DCACHE_LINESIZE - 1)
#define DMA_ALIGN_UP(n)    (((n) + DMA_BUFFER_MASK) & ~DMA_BUFFER_MASK)
#define DMA_ALIGN_DOWN(n)  ((n) & ~DMA_BUFFER_MASK)

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum spi_config_e
{
  FULL_DUPLEX = 0,
  SIMPLEX_TX,
  SIMPLEX_RX,
  HALF_DUPLEX
};

struct stm32_spidev_s
{
  /* Externally visible part of the SPI slave interface */

  struct spi_slave_ctrlr_s ctrlr;
  struct spi_slave_dev_s *dev;   /* Bound SPI slave device interface */
  uint32_t         spibase;      /* SPIn base address */
  uint32_t         spiclock;     /* Clocking for the SPI module */
  uint8_t          irq;          /* SPI IRQ number */
  uint32_t         nss_pin;      /* Chip select pin configuration */
#ifdef CONFIG_STM32H7_SPI_DMA
  volatile uint8_t rxresult;     /* Result of the RX DMA */
  volatile uint8_t txresult;     /* Result of the TX DMA */
  uint32_t         rxch;         /* The RX DMA channel number */
  uint32_t         txch;         /* The TX DMA channel number */
  DMA_HANDLE       rxdma;        /* DMA channel handle for RX transfers */
  DMA_HANDLE       txdma;        /* DMA channel handle for TX transfers */
  sem_t            rxsem;        /* Wait for RX DMA to complete */
  sem_t            txsem;        /* Wait for TX DMA to complete */
  uint32_t         txccr;        /* DMA control register for TX transfers */
  uint32_t         rxccr;        /* DMA control register for RX transfers */
  bool             dmarunning;   /* DMA is started */
#endif
  bool             initialized;  /* Has SPI interface been initialized */
  sem_t            exclsem;      /* Held while chip is selected for mutual
                                  * exclusion */
  int8_t           nbits;        /* Width of word in bits */
  uint8_t          mode;         /* Mode 0,1,2,3 */
  uint8_t          bus;          /* SPI bus number */
  bool             nss;          /* True: Chip selected */
#ifdef CONFIG_PM
  struct pm_callback_s pm_cb;    /* PM callbacks */
#endif
  enum spi_config_e config;      /* full/half duplex, simplex rx/tx */

  /* Output queue */

  uint16_t ohead;                 /* Location of next value in out queue */
  uint16_t otail;                 /* Index of first value in out queue */
  uint8_t *outq;

  /* Input queue */

  uint16_t ihead;                 /* Location of next unread value */
#ifndef CONFIG_STM32H7_SPI_DMA
  uint16_t itail;                 /* Index of next free memory pointer */
#endif
  uint8_t *inq;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

static inline uint32_t spi_getreg(FAR struct stm32_spidev_s *priv,
                                  uint32_t offset);
static inline void spi_putreg(FAR struct stm32_spidev_s *priv,
                              uint32_t offset, uint32_t value);
static inline void spi_modifyreg(FAR struct stm32_spidev_s *priv,
                                 uint32_t offset, uint32_t clrbits,
                                 uint32_t setbits);
static inline uint32_t spi_readword(FAR struct stm32_spidev_s *priv);
static inline void spi_writeword(FAR struct stm32_spidev_s *priv,
                                 uint32_t byte);
static inline bool spi_9to16bitmode(FAR struct stm32_spidev_s *priv);
#ifdef CONFIG_DEBUG_SPI_INFO
static inline void spi_dumpregs(FAR struct stm32_spidev_s *priv);
#endif

/* DMA support */

#ifdef CONFIG_STM32H7_SPI_DMA
static void        spi_dmarxcallback(DMA_HANDLE handle, uint8_t isr,
                                     void *arg);
static void        spi_dmatxcallback(DMA_HANDLE handle, uint8_t isr,
                                     void *arg);
static void        spi_dmarxsetup(FAR struct stm32_spidev_s *priv,
                                  size_t nwords);
static void        spi_dmatxsetup(FAR struct stm32_spidev_s *priv,
                                  size_t nwords);
static inline void spi_dmarxstart(FAR struct stm32_spidev_s *priv);
static inline void spi_dmatxstart(FAR struct stm32_spidev_s *priv);
#endif

/* Interrupt handler detecting nss status changes */

static int  spi_nssinterrupt(int irq, void *context, void *arg);

/* SPI slave methods */

static void     spi_bind(struct spi_slave_ctrlr_s *ctrlr,
                         struct spi_slave_dev_s *dev,
                         enum spi_slave_mode_e mode,
                         int nbits);
static void     spi_unbind(struct spi_slave_ctrlr_s *ctrlr);
static int      spi_enqueue(struct spi_slave_ctrlr_s *ctrlr,
                            FAR const void *data,
                            size_t len);
static bool     spi_qfull(struct spi_slave_ctrlr_s *ctrlr);
static void     spi_qflush(struct spi_slave_ctrlr_s *ctrlr);
static size_t   spi_qpoll(struct spi_slave_ctrlr_s *ctrlr);

/* Initialization */

static void     spi_slave_initialize(FAR struct stm32_spidev_s *priv);

/* PM interface */

#ifdef CONFIG_PM
static int         spi_pm_prepare(FAR struct pm_callback_s *cb, int domain,
                                  enum pm_state_e pmstate);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* SPI slave controller driver operations */

static const struct spi_slave_ctrlrops_s g_ctrlr_ops =
{
  .bind              = spi_bind,
  .unbind            = spi_unbind,
  .enqueue           = spi_enqueue,
  .qfull             = spi_qfull,
  .qflush            = spi_qflush,
  .qpoll             = spi_qpoll,
};

#define SPI_SLAVE_OUTQ(x) spi##x##_outq
#define SPI_SLAVE_INQ(x) spi##x##_inq

#ifdef CONFIG_STM32H7_SPI_DMA
#define SPI_SLAVE_INIT_DMA(x)                           \
  .rxch          = DMAMAP_SPI##x##_RX,                  \
  .txch          = DMAMAP_SPI##x##_TX,                  \
  .outq	         = SPI_SLAVE_OUTQ(x),                   \
  .inq	         = SPI_SLAVE_INQ(x),
#else
#define SPI_SLAVE_INIT_DMA(x)
#endif

#ifdef CONFIG_PM
#define SPI_SLAVE_INIT_PM_PREPARE .pm_cb.prepare = spi_pm_prepare,
#else
#define SPI_SLAVE_INIT_PM_PREPARE
#endif

#define SPI_SLAVE_INIT(x)                               \
{                                                       \
  .ctrlr.ops     = &g_ctrlr_ops,                        \
  .dev           = NULL,                                \
  .spibase       = STM32_SPI##x##_BASE,                 \
  .spiclock      = SPI45_KERNEL_CLOCK_FREQ,             \
  .irq           = STM32_IRQ_SPI##x,                    \
  SPI_SLAVE_INIT_DMA(x)                                 \
  .initialized   = false,                               \
  SPI_SLAVE_INIT_PM_PREPARE                             \
  .config        = CONFIG_STM32H7_SPI##x##_COMMTYPE,    \
}

#ifdef CONFIG_STM32H7_SPI1_SLAVE

static
uint8_t SPI_SLAVE_OUTQ(1)[DMA_ALIGN_UP(CONFIG_STM32H7_SPI_SLAVE_QSIZE)]
__attribute__((aligned(ARMV7M_DCACHE_LINESIZE)));
static
uint8_t SPI_SLAVE_INQ(1)[DMA_ALIGN_UP(CONFIG_STM32H7_SPI_SLAVE_QSIZE)]
__attribute__((aligned(ARMV7M_DCACHE_LINESIZE)));
static struct stm32_spidev_s g_spi1ctrlr = SPI_SLAVE_INIT(1);

#endif

#ifdef CONFIG_STM32H7_SPI2_SLAVE

static
uint8_t SPI_SLAVE_OUTQ(2)[DMA_ALIGN_UP(CONFIG_STM32H7_SPI_SLAVE_QSIZE)]
__attribute__((aligned(ARMV7M_DCACHE_LINESIZE)));
static
uint8_t SPI_SLAVE_INQ(2)[DMA_ALIGN_UP(CONFIG_STM32H7_SPI_SLAVE_QSIZE)]
__attribute__((aligned(ARMV7M_DCACHE_LINESIZE)));
static struct stm32_spidev_s g_spi2ctrlr = SPI_SLAVE_INIT(2);

#endif

#ifdef CONFIG_STM32H7_SPI3_SLAVE

static
uint8_t SPI_SLAVE_OUTQ(3)[DMA_ALIGN_UP(CONFIG_STM32H7_SPI_SLAVE_QSIZE)]
__attribute__((aligned(ARMV7M_DCACHE_LINESIZE)));
static
uint8_t SPI_SLAVE_INQ(3)[DMA_ALIGN_UP(CONFIG_STM32H7_SPI_SLAVE_QSIZE)]
__attribute__((aligned(ARMV7M_DCACHE_LINESIZE)));
static struct stm32_spidev_s g_spi3ctrlr = SPI_SLAVE_INIT(3);

#endif

#ifdef CONFIG_STM32H7_SPI4_SLAVE

static
uint8_t SPI_SLAVE_OUTQ(4)[DMA_ALIGN_UP(CONFIG_STM32H7_SPI_SLAVE_QSIZE)]
__attribute__((aligned(ARMV7M_DCACHE_LINESIZE)));
static
uint8_t SPI_SLAVE_INQ(4)[DMA_ALIGN_UP(CONFIG_STM32H7_SPI_SLAVE_QSIZE)]
__attribute__((aligned(ARMV7M_DCACHE_LINESIZE)));
static struct stm32_spidev_s g_spi4ctrlr = SPI_SLAVE_INIT(4);

#endif

#ifdef CONFIG_STM32H7_SPI5_SLAVE

static
uint8_t SPI_SLAVE_OUTQ(5)[DMA_ALIGN_UP(CONFIG_STM32H7_SPI_SLAVE_QSIZE)]
__attribute__((aligned(ARMV7M_DCACHE_LINESIZE)));
static
uint8_t SPI_SLAVE_INQ(5)[DMA_ALIGN_UP(CONFIG_STM32H7_SPI_SLAVE_QSIZE)]
__attribute__((aligned(ARMV7M_DCACHE_LINESIZE)));
static struct stm32_spidev_s g_spi5ctrlr = SPI_SLAVE_INIT(5);

#endif

#ifdef CONFIG_STM32H7_SPI6_SLAVE

/* TODO: these needs to be located in SRAM3 for SPI6 */

static
uint8_t SPI_SLAVE_OUTQ(6)[DMA_ALIGN_UP(CONFIG_STM32H7_SPI_SLAVE_QSIZE)]
__attribute__((aligned(ARMV7M_DCACHE_LINESIZE)));
static
uint8_t SPI_SLAVE_INQ(6)[DMA_ALIGN_UP(CONFIG_STM32H7_SPI_SLAVE_QSIZE)]
__attribute__((aligned(ARMV7M_DCACHE_LINESIZE)));
static struct stm32_spidev_s g_spi6ctrlr = SPI_SLAVE_INIT(6);

#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_getreg8
 *
 * Description:
 *   Get the contents of the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *
 * Returned Value:
 *   The contents of the 8-bit register
 *
 ****************************************************************************/

static inline uint8_t spi_getreg8(FAR struct stm32_spidev_s *priv,
                                  uint32_t offset)
{
  return getreg8(priv->spibase + offset);
}

/****************************************************************************
 * Name: spi_putreg8
 *
 * Description:
 *   Write a 8-bit value to the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *   value  - the 8-bit value to be written
 *
 ****************************************************************************/

static inline void spi_putreg8(FAR struct stm32_spidev_s *priv,
                               uint32_t offset, uint8_t value)
{
  putreg8(value, priv->spibase + offset);
}

/****************************************************************************
 * Name: spi_getreg
 *
 * Description:
 *   Get the contents of the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *
 * Returned Value:
 *   The contents of the 16-bit register
 *
 ****************************************************************************/

static inline uint32_t spi_getreg(FAR struct stm32_spidev_s *priv,
                                  uint32_t offset)
{
  return getreg32(priv->spibase + offset);
}

/****************************************************************************
 * Name: spi_putreg
 *
 * Description:
 *   Write a 16-bit value to the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *   value  - the 16-bit value to be written
 *
 ****************************************************************************/

static inline void spi_putreg(FAR struct stm32_spidev_s *priv,
                              uint32_t offset, uint32_t value)
{
  putreg32(value, priv->spibase + offset);
}

/****************************************************************************
 * Name: spi_modifyreg
 *
 * Description:
 *   Write a 32-bit value to the SPI register at offset
 *
 * Input Parameters:
 *   priv    - private SPI device structure
 *   offset  - offset to the register of interest
 *   clrbits - the bits to clear
 *   setbits - the bits to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void spi_modifyreg(FAR struct stm32_spidev_s *priv,
                                 uint32_t offset, uint32_t clrbits,
                                 uint32_t setbits)
{
  modifyreg32(priv->spibase + offset, clrbits, setbits);
}

/****************************************************************************
 * Name: spi_readword
 *
 * Description:
 *   Read one byte from SPI
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *
 * Returned Value:
 *   Byte as read
 *
 ****************************************************************************/

static inline uint32_t spi_readword(FAR struct stm32_spidev_s *priv)
{
  /* Can't receive in tx only mode */

  if (priv->config == SIMPLEX_TX)
    {
      return 0;
    }

  /* Wait until the receive buffer is not empty */

  while ((spi_getreg(priv, STM32_SPI_SR_OFFSET) & SPI_SR_RXP) == 0);

  /* Then return the received byte */

  return spi_getreg(priv, STM32_SPI_RXDR_OFFSET);
}

/****************************************************************************
 * Name: spi_writeword
 *
 * Description:
 *   Write one byte to SPI
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *   byte - Byte to send
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void spi_writeword(FAR struct stm32_spidev_s *priv,
                                 uint32_t word)
{
  /* Can't transmit in rx only mode */

  if (priv->config == SIMPLEX_RX)
    {
      return;
    }

  /* Wait until the transmit buffer is empty */

  while ((spi_getreg(priv, STM32_SPI_SR_OFFSET) & SPI_SR_TXP) == 0);

  /* Then send the byte */

  spi_putreg(priv, STM32_SPI_TXDR_OFFSET, word);
}

/****************************************************************************
 * Name: spi_readbyte
 *
 * Description:
 *   Read one byte from SPI
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *
 * Returned Value:
 *   Byte as read
 *
 ****************************************************************************/

static inline uint8_t spi_readbyte(FAR struct stm32_spidev_s *priv)
{
  /* Can't receive in tx only mode */

  if (priv->config == SIMPLEX_TX)
    {
      return 0;
    }

  /* Wait until the receive buffer is not empty */

  while ((spi_getreg(priv, STM32_SPI_SR_OFFSET) & SPI_SR_RXP) == 0);

  /* Then return the received byte */

  return spi_getreg8(priv, STM32_SPI_RXDR_OFFSET);
}

/****************************************************************************
 * Name: spi_writebyte
 *
 * Description:
 *   Write one 8-bit frame to the SPI FIFO
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *   byte - Byte to send
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void spi_writebyte(FAR struct stm32_spidev_s *priv,
                                 uint8_t byte)
{
  /* Can't transmit in rx only mode */

  if (priv->config == SIMPLEX_RX)
    {
      return;
    }

  /* Wait until the transmit buffer is empty */

  while ((spi_getreg(priv, STM32_SPI_SR_OFFSET) & SPI_SR_TXP) == 0);

  /* Then send the byte */

  spi_putreg8(priv, STM32_SPI_TXDR_OFFSET, byte);
}

/****************************************************************************
 * Name: spi_dumpregs
 ****************************************************************************/

#ifdef CONFIG_DEBUG_SPI_INFO
static void spi_dumpregs(FAR struct stm32_spidev_s *priv)
{
  spiinfo("CR1: 0x%08x CFG1: 0x%08x CFG2: 0x%08x\n",
          spi_getreg(priv, STM32_SPI_CR1_OFFSET),
          spi_getreg(priv, STM32_SPI_CFG1_OFFSET),
          spi_getreg(priv, STM32_SPI_CFG2_OFFSET));
  spiinfo("IER: 0x%08x SR: 0x%08x I2SCFGR: 0x%08x\n",
          spi_getreg(priv, STM32_SPI_IER_OFFSET),
          spi_getreg(priv, STM32_SPI_SR_OFFSET),
          spi_getreg(priv, STM32_SPI_I2SCFGR_OFFSET));
}
#endif

/****************************************************************************
 * Name: spi_9to16bitmode
 *
 * Description:
 *   Check if the SPI is operating in more then 8 bit mode
 *
 * Input Parameters:
 *   priv     - Device-specific state data
 *
 * Returned Value:
 *   true: >8 bit mode-bit mode, false: <= 8-bit mode
 *
 ****************************************************************************/

static inline bool spi_9to16bitmode(FAR struct stm32_spidev_s *priv)
{
  uint32_t regval = spi_getreg(priv, STM32_SPI_CFG1_OFFSET);

  return ((regval & SPI_CFG1_CRCSIZE_9BIT) == SPI_CFG1_CRCSIZE_9BIT);
}

/****************************************************************************
 * Name: spi_dmarxcallback
 *
 * Description:
 *   Called when the RX DMA completes
 *
 ****************************************************************************/

#ifdef CONFIG_STM32H7_SPI_DMA
static void spi_dmarxcallback(DMA_HANDLE handle, uint8_t isr, void *arg)
{
  FAR struct stm32_spidev_s *priv = (FAR struct stm32_spidev_s *)arg;

  /* Wake-up the SPI driver */

  priv->rxresult = isr | 0x080;  /* OR'ed with 0x80 to assure non-zero */
}
#endif

/****************************************************************************
 * Name: spi_dmatxcallback
 *
 * Description:
 *   Called when the RX DMA completes
 *
 ****************************************************************************/

#ifdef CONFIG_STM32H7_SPI_DMA
static void spi_dmatxcallback(DMA_HANDLE handle, uint8_t isr, void *arg)
{
  FAR struct stm32_spidev_s *priv = (FAR struct stm32_spidev_s *)arg;

  /* Wake-up the SPI driver */

  priv->txresult = isr | 0x080;  /* OR'ed with 0x80 to assure non-zero */
}
#endif

/****************************************************************************
 * Name: spi_dmarxsetup
 *
 * Description:
 *   Setup to perform RX DMA
 *
 ****************************************************************************/

#ifdef CONFIG_STM32H7_SPI_DMA
static void spi_dmarxsetup(FAR struct stm32_spidev_s *priv, size_t nwords)
{
  stm32_dmacfg_t dmacfg;

  /* Can't receive in tx only mode */

  if (priv->config == SIMPLEX_TX)
    {
      return;
    }

  /* 8- or 16-bit mode? */

  if (spi_9to16bitmode(priv))
    {
      /* 16-bit mode */

      priv->rxccr = SPI_RXDMA16_CONFIG;
    }
  else
    {
      /* 8-bit mode -- is there a buffer to receive data in? */

      priv->rxccr = SPI_RXDMA8_CONFIG;
    }

  /* Configure the RX DMA */

  /* This just transfers continuously to circular buffer */

  dmacfg.paddr = priv->spibase + STM32_SPI_RXDR_OFFSET;
  dmacfg.maddr = (uint32_t)priv->inq;
  dmacfg.ndata = CONFIG_STM32H7_SPI_SLAVE_QSIZE;
  dmacfg.cfg1  = priv->rxccr;
  dmacfg.cfg2  = 0;

  stm32_dmasetup(priv->rxdma, &dmacfg);
}
#endif

/****************************************************************************
 * Name: spi_dmatxsetup
 *
 * Description:
 *   Setup to perform TX DMA
 *
 ****************************************************************************/

#ifdef CONFIG_STM32H7_SPI_DMA
static void spi_dmatxsetup(FAR struct stm32_spidev_s *priv, size_t nwords)
{
  /* TODO: set up dma to transfer out the new data from priv->outq,
   * which is set up in spi_enqueue
   * When the transfer is complete, update outq indexies accordingly
   */

#if 0
  stm32_dmacfg_t dmacfg;

  /* Can't transmit in rx only mode */

  if (priv->config == SIMPLEX_RX)
    {
      return;
    }

  /* 8- or 16-bit mode? */

  if (spi_9to16bitmode(priv))
    {
      /* 16-bit mode */

      priv->txccr = SPI_TXDMA16_CONFIG;
    }
  else
    {
      /* 8-bit mode */

      priv->txccr = SPI_TXDMA8_CONFIG;
    }

  dmacfg.paddr = priv->spibase + STM32_SPI_TXDR_OFFSET;
  dmacfg.maddr = (uint32_t)priv->outq[0];
  dmacfg.ndata = nwords;
  dmacfg.cfg1  = priv->txccr;
  dmacfg.cfg2  = 0;

  /* Setup the TX DMA */

  stm32_dmasetup(priv->txdma, &dmacfg);

#endif // 0
}
#endif

/****************************************************************************
 * Name: spi_dmarxstart
 *
 * Description:
 *   Start RX DMA. NB! This must be called before spi_dmatxstart
 *
 ****************************************************************************/

#ifdef CONFIG_STM32H7_SPI_DMA
static void spi_dmarxstart(FAR struct stm32_spidev_s *priv)
{
  /* Can't receive in tx only mode */

  if (priv->config == SIMPLEX_TX)
    {
      return;
    }

  priv->rxresult = 0;

  spi_modifyreg(priv, STM32_SPI_CFG1_OFFSET, 0, SPI_CFG1_RXDMAEN);
  stm32_dmastart(priv->rxdma, spi_dmarxcallback, priv, false);
}
#endif

/****************************************************************************
 * Name: spi_dmatxstart
 *
 * Description:
 *   Start TX DMA. NB! This must be called after spi_dmarxstart
 *
 ****************************************************************************/

#ifdef CONFIG_STM32H7_SPI_DMA
static void spi_dmatxstart(FAR struct stm32_spidev_s *priv)
{
  /* Can't transmit in rx only mode */

  if (priv->config == SIMPLEX_RX)
    {
      return;
    }

  priv->txresult = 0;
  stm32_dmastart(priv->txdma, spi_dmatxcallback, priv, false);
  spi_modifyreg(priv, STM32_SPI_CFG1_OFFSET, 0, SPI_CFG1_TXDMAEN);
}
#endif

/****************************************************************************
 * Name: spi_lock
 *
 * Description:
 *   Take or release the semaphore that enforces mutually exclusive access to
 *   SPI resources, handling any exceptional conditions
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   lock - true: Lock spi bus, false: unlock SPI bus
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int spi_lock(FAR struct spi_slave_ctrlr_s *ctrlr, bool lock)
{
  FAR struct stm32_spidev_s *priv = (FAR struct stm32_spidev_s *)ctrlr;
  int ret;

  if (lock)
    {
      /* Take the semaphore (perhaps waiting) */

      do
        {
          ret = nxsem_wait(&priv->exclsem);

          /* The only case that an error should occur here is if the wait
           * was awakened by a signal.
           */

          DEBUGASSERT(ret == OK || ret == -EINTR);
        }
      while (ret == -EINTR);
    }
  else
    {
      (void)nxsem_post(&priv->exclsem);
      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: spi_enable
 *
 * Description:
 *   Enable/disable the SPI peripheral
 *
 * Input Parameters:
 *   priv  - Device-specific state data
 *   state - true: enable, false: disable
 *
 ****************************************************************************/

static inline void spi_enable(FAR struct stm32_spidev_s *priv, bool state)
{
  if (state == true)
    {
      /* Enable SPI */

      spi_modifyreg(priv, STM32_SPI_CR1_OFFSET, 0, SPI_CR1_SPE);
    }
  else
    {
      /* Disable SPI */

      spi_modifyreg(priv, STM32_SPI_CR1_OFFSET, SPI_CR1_SPE, 0);
    }
}

/****************************************************************************
 * Name: spi_setmode
 *
 * Description:
 *   Set the SPI mode.  see enum spi_mode_e for mode definitions
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   mode - The SPI mode requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

static void spi_setmode(FAR struct spi_slave_ctrlr_s *ctrlr,
                        enum spi_mode_e mode)
{
  FAR struct stm32_spidev_s *priv = (FAR struct stm32_spidev_s *)ctrlr;
  uint32_t setbits = 0;
  uint32_t clrbits = 0;

  spiinfo("mode=%d\n", mode);

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
      /* Yes... Set CR1 appropriately */

      switch (mode)
        {
        case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
          setbits = 0;
          clrbits = SPI_CFG2_CPOL | SPI_CFG2_CPHA;
          break;

        case SPIDEV_MODE1: /* CPOL=0; CPHA=1 */
          setbits = SPI_CFG2_CPHA;
          clrbits = SPI_CFG2_CPOL;
          break;

        case SPIDEV_MODE2: /* CPOL=1; CPHA=0 */
          setbits = SPI_CFG2_CPOL;
          clrbits = SPI_CFG2_CPHA;
          break;

        case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
          setbits = SPI_CFG2_CPOL | SPI_CFG2_CPHA;
          clrbits = 0;
          break;

        default:
          return;
        }

      /* Change SPI mode */

      spi_modifyreg(priv, STM32_SPI_CFG2_OFFSET, clrbits, setbits);

      /* Save the mode so that subsequent re-configurations will be faster */

      priv->mode = mode;
    }
}

/****************************************************************************
 * Name: spi_setbits
 *
 * Description:
 *   Set the number of bits per word.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   nbits - The number of bits requested
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_setbits(FAR struct spi_slave_ctrlr_s *ctrlr, int nbits)
{
  FAR struct stm32_spidev_s *priv = (FAR struct stm32_spidev_s *)ctrlr;
  uint32_t setbits = 0;
  uint32_t clrbits = 0;

  spiinfo("nbits=%d\n", nbits);

  /* Has the number of bits changed? */

  if (nbits != priv->nbits)
    {
      /* Yes... Set CFG1 appropriately */

      /* Set the number of bits (valid range 4-32) */

      if (nbits < 4 || nbits > 32)
        {
          return;
        }

      clrbits = SPI_CFG1_DSIZE_MASK;
      setbits = SPI_CFG1_DSIZE_VAL(nbits);

      /* REVISIT: FIFO threshold level */

      /* If nbits is <=8, then we are in byte mode and FRXTH shall be set
       * (else, transaction will not complete).
       */

      if (nbits < 9)
        {
          setbits |= SPI_CFG1_FTHLV_1DATA; /* RX FIFO Threshold = 1 byte */
        }
      else
        {
          setbits |= SPI_CFG1_FTHLV_2DATA; /* RX FIFO Threshold = 2 byte */
        }

      spi_modifyreg(priv, STM32_SPI_CFG1_OFFSET, clrbits, setbits);

      priv->nbits = nbits;
    }
}

/****************************************************************************
 * Name: spi_bind
 *
 * Description:
 *   Bind the SPI slave device interface to the SPI slave controller
 *   interface and configure the SPI interface. Upon return, the SPI
 *   slave controller driver is fully operational and ready to perform
 *   transfers.
 *
 * Input Parameters:
 *   ctrlr - SPI Slave controller interface instance
 *   dev   - SPI Slave device interface instance
 *   mode  - The SPI Slave mode requested
 *   nbits - The number of bits requested.
 *           If value is greater than 0, then it implies MSB first
 *           If value is less than 0, then it implies LSB first with -nbits
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void spi_bind(struct spi_slave_ctrlr_s *ctrlr,
                     struct spi_slave_dev_s *dev, enum spi_slave_mode_e mode,
                     int nbits)
{
  struct stm32_spidev_s *priv = (struct stm32_spidev_s *)ctrlr;
  uint32_t nss_gpio;

  spiinfo("dev=%p mode=%d nbits=%d\n", sdv, mode, nbits);

  DEBUGASSERT(priv != NULL && priv->dev == NULL && dev != NULL);

  /* Get exclusive access to the SPI device */

  spi_lock(ctrlr, true);

  /* Make sure the spi is disabled */

  spi_enable(priv, false);

  /* Make sure the dma is disabled */

  spi_modifyreg(priv, STM32_SPI_CFG1_OFFSET,
                SPI_CFG1_RXDMAEN | SPI_CFG1_TXDMAEN, 0);

  /* invalidate the whole rx buffer */

  up_flush_dcache((uintptr_t)priv->inq,
                  (uintptr_t)priv->inq + CONFIG_STM32H7_SPI_SLAVE_QSIZE);

  /* Bind the SPI slave device interface instance to the SPI slave
   * controller interface.
   */

  priv->dev = dev;

  /* Initialize the circular buffer head */

  priv->ihead = 0;

  /* Setup to begin normal SPI operation */

  spi_setmode(ctrlr, mode);
  spi_setbits(ctrlr, nbits);

  /* First, configure NSS as GPIO EXTI input */

  nss_gpio = priv->nss_pin & (GPIO_PORT_MASK | GPIO_PIN_MASK);
  nss_gpio |= (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI);
  stm32_configgpio(nss_gpio);

  /* Bind to NSS interrupt */

  (void)stm32_gpiosetevent(priv->nss_pin, false, true, false,
                           spi_nssinterrupt, priv);

#ifdef CONFIG_PM
  /* Register to receive power management callbacks */

  ret = pm_register(&priv->pm_cb);
  DEBUGASSERT(ret == OK);
  UNUSED(ret);
#endif

  spi_lock(ctrlr, false);
}

/****************************************************************************
 * Name: spi_nssinterrupt
 *
 * Description:
 *   SPI slave NSS interrupt handler
 *
 * Input Parameters:
 *   irq     - not used
 *   context - not used
 *   arg     - pointer to controller
 *
 * Returned Value:
 *   OK
 *
 ****************************************************************************/

static int spi_nssinterrupt(int irq, void *context, void *arg)
{
  FAR struct stm32_spidev_s *priv = (FAR struct stm32_spidev_s *)arg;

  /* If the pin is low, just enable rising edge interrupt */

  if (!stm32_gpioread(priv->nss_pin))
    {
      /* Bind to NSS rising edge interrupt */

      (void)stm32_gpiosetevent(priv->nss_pin, true, false, false,
                               spi_nssinterrupt, priv);
      return OK;
    }

  /* Disable NSS interrupt */

  (void)stm32_gpiosetevent(priv->nss_pin, false, false, false,
                           NULL, priv);

  /* Re-configure nss pin */

  stm32_configgpio(priv->nss_pin);

  /* Enable spi peripheral */

  spi_enable(priv, true);

  /* Flush SPI read FIFO */

  while ((spi_getreg(priv, STM32_SPI_SR_OFFSET) & SPI_SR_RXPLVL_MASK) != 0)
    {
      spi_getreg(priv, STM32_SPI_RXDR_OFFSET);
    }

  /* Disable spi before dma setup */

  spi_enable(priv, false);

#ifdef CONFIG_STM32H7_SPI_DMA

  /* Setup DMAs */

  spi_dmarxsetup(priv, 0);
  spi_dmatxsetup(priv, 0);

  /* Start the DMAs */

  spi_dmarxstart(priv);
  spi_dmatxstart(priv);

  priv->dmarunning = true;
#endif

  /* Enable spi peripheral */

  spi_enable(priv, true);

  return OK;
}

/****************************************************************************
 * Name: spi_unbind
 *
 * Description:
 *   Un-bind the SPI slave device interface from the SPI slave controller
 *   interface. Reset the SPI interface and restore the SPI slave
 *   controller driver to its initial state.
 *
 * Input Parameters:
 *   ctrlr - SPI Slave controller interface instance
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void spi_unbind(struct spi_slave_ctrlr_s *ctrlr)
{
  struct stm32_spidev_s *priv = (struct stm32_spidev_s *)ctrlr;

  DEBUGASSERT(priv != NULL);
  spiinfo("Unbinding %p\n", priv->dev);

  DEBUGASSERT(priv->dev != NULL);

  /* Get exclusive access to the SPI device */

  spi_lock(ctrlr, true);

  /* Unbind the SPI slave interface */

  priv->dev = NULL;

  /* Disable DMA */

  spi_modifyreg(priv, STM32_SPI_CFG1_OFFSET,
                SPI_CFG1_RXDMAEN | SPI_CFG1_TXDMAEN, 0);

  /* Disable the SPI peripheral */

  spi_enable(priv, false);

  spi_lock(ctrlr, false);
}

/****************************************************************************
 * Name: spi_enqueue
 *
 * Description:
 *   Enqueue the next value to be shifted out from the interface. This adds
 *   the word the controller driver for a subsequent transfer but has no
 *   effect on any in-process or currently "committed" transfers.
 *
 * Input Parameters:
 *   ctrlr - SPI Slave controller interface instance
 *   data  - Pointer to the command/data mode data to be shifted out.
 *           The data width must be aligned to the nbits parameter which was
 *           previously provided to the bind() method.
 *   len   - Number of units of "nbits" wide to enqueue,
 *           "nbits" being the data width previously provided to the bind()
 *           method.
 *
 * Returned Value:
 *   Zero if the word was successfully queue; A negated errno valid is
 *   returned on any failure to enqueue the word (such as if the queue is
 *   full).
 *
 ****************************************************************************/

static int spi_enqueue(struct spi_slave_ctrlr_s *ctrlr,
                       FAR const void *data, size_t len)
{
  return 0;
}

/****************************************************************************
 * Name: spi_qfull
 *
 * Description:
 *   Return true if the queue is full or false if there is space to add an
 *   additional word to the queue.
 *
 * Input Parameters:
 *   ctrlr - SPI Slave controller interface instance
 *
 * Returned Value:
 *   true if the output queue is full, false otherwise.
 *
 ****************************************************************************/

static bool spi_qfull(struct spi_slave_ctrlr_s *ctrlr)
{
  return false;
}

/****************************************************************************
 * Name: spi_qflush
 *
 * Description:
 *   Discard all saved values in the output queue. On return from this
 *   function the output queue will be empty. Any in-progress or otherwise
 *   "committed" output values may not be flushed.
 *
 * Input Parameters:
 *   ctrlr - SPI Slave controller interface instance
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void spi_qflush(struct spi_slave_ctrlr_s *ctrlr)
{
  struct stm32_spidev_s *priv = (struct stm32_spidev_s *)ctrlr;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL && priv->dev != NULL);

#ifdef CONFIG_STM32H7_SPI_DMA
  if (!priv->dmarunning)
    {
      return;
    }
#endif

  /* Get exclusive access to the SPI device */

  spi_lock(ctrlr, true);
  flags = enter_critical_section();

  /* Flush the input buffers */

#ifdef CONFIG_STM32H7_SPI_DMA
  priv->ihead =
    CONFIG_STM32H7_SPI_SLAVE_QSIZE - stm32_dmaresidual(priv->rxdma);
#else
  priv->ihead = 0;
  priv->itail = 0;
#endif

  /* Flush the output buffers */

  priv->ohead = 0;
  priv->otail = 0;
  leave_critical_section(flags);
  spi_lock(ctrlr, false);
}

/****************************************************************************
 * Name: spi_rx_buffer_free
 *
 * Description:
 *   Handle buffer wrapping around & invalidate the previous buffer cache
 *
 * Input Parameters:
 *   ptr - pointer to the rx buffer
 *   start - start index of just received data
 *   end - end index of just received data
 *
 * Returned Value:
 *   new start index; 0 if buffer wrapped around, end otherwise
 *
 ****************************************************************************/

static inline int spi_rx_buffer_free(uint8_t *ptr, int start, int end)
{
  /* The priv->ihead can only be greater than qsize if the device driver
   * returns garbage
   */

  if (end >= CONFIG_STM32H7_SPI_SLAVE_QSIZE)
    {
      end = 0;
      up_invalidate_dcache((uintptr_t)&ptr[start],
                           (uintptr_t)&ptr[CONFIG_STM32H7_SPI_SLAVE_QSIZE]);
    }
  else
    {
      up_invalidate_dcache((uintptr_t)&ptr[start],
                           (uintptr_t)&ptr[end]);
    }

  return end;
}

/****************************************************************************
 * Name: spi_qpoll
 *
 * Description:
 *   Tell the controller to output all the receive queue data.
 *
 * Input Parameters:
 *   ctrlr - SPI Slave controller interface instance
 *
 * Returned Value:
 *   Number of bytes left in the rx queue. If the device accepted all the
 *
 ****************************************************************************/

static size_t spi_qpoll(struct spi_slave_ctrlr_s *ctrlr)
{
  struct stm32_spidev_s *priv = (struct stm32_spidev_s *)ctrlr;
  int itail;
  int ihead;
  uint16_t bytes_left;

  DEBUGASSERT(priv != NULL && priv->dev != NULL);
  DEBUGASSERT(priv->ihead < CONFIG_STM32H7_SPI_SLAVE_QSIZE);

#ifdef CONFIG_STM32H7_SPI_DMA
  if (!priv->dmarunning)
    {
      return 0;
    }
#endif

  /* Get exclusive access to the SPI device */

  spi_lock(ctrlr, true);

#ifdef CONFIG_STM32H7_SPI_DMA
  itail = CONFIG_STM32H7_SPI_SLAVE_QSIZE - stm32_dmaresidual(priv->rxdma);
#else
  #error Support only simplex mode rx with dma
#endif

  /* Receive all data between last read index and the current index */

  ihead = priv->ihead;
  if (ihead > itail)
    {
      /* Receive the end of receive buffer */

      priv->ihead += SPIS_DEV_RECEIVE(priv->dev,
                                      (const uint16_t *)&priv->inq[ihead],
                                      CONFIG_STM32H7_SPI_SLAVE_QSIZE -
                                        ihead);

      /* Invalidate dcache and wrap around the priv->ihead */

      priv->ihead = spi_rx_buffer_free(priv->inq, ihead, priv->ihead);
    }

  ihead = priv->ihead;
  if (ihead < itail)
    {
      /* Receive the data between ihead and itail */

      priv->ihead += SPIS_DEV_RECEIVE(priv->dev,
                                      (const uint16_t *)&priv->inq[ihead],
                                      itail - ihead);

      /* Invalidate dcache and wrap around the priv->ihead */

      priv->ihead = spi_rx_buffer_free(priv->inq, ihead, priv->ihead);
    }

  /* Calculate the number of bytes left in the buffer */

  bytes_left = itail < priv->ihead
    ? CONFIG_STM32H7_SPI_SLAVE_QSIZE - priv->ihead + itail
    : itail - priv->ihead;

  spi_lock(ctrlr, false);

  return bytes_left;
}

/****************************************************************************
 * Name: spi_pm_prepare
 *
 * Description:
 *   Request the driver to prepare for a new power state. This is a
 *   warning that the system is about to enter into a new power state.  The
 *   driver should begin whatever operations that may be required to enter
 *   power state.  The driver may abort the state change mode by returning
 *   a non-zero value from the callback function.
 *
 * Input Parameters:
 *   cb      - Returned to the driver.  The driver version of the callback
 *             structure may include additional, driver-specific state
 *             data at the end of the structure.
 *   domain  - Identifies the activity domain of the state change
 *   pmstate - Identifies the new PM state
 *
 * Returned Value:
 *   0 (OK) means the event was successfully processed and that the driver
 *   is prepared for the PM state change.  Non-zero means that the driver
 *   is not prepared to perform the tasks needed achieve this power setting
 *   and will cause the state change to be aborted.  NOTE:  The prepare
 *   method will also be recalled when reverting from lower back to higher
 *   power consumption modes (say because another driver refused a lower
 *   power state change).  Drivers are not permitted to return non-zero
 *   values when reverting back to higher power consumption modes!
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static int spi_pm_prepare(FAR struct pm_callback_s *cb, int domain,
                          enum pm_state_e pmstate)
{
  struct stm32_spidev_s *priv =
      (struct stm32_spidev_s *)((char *)cb -
                                    offsetof(struct stm32_spidev_s, pm_cb));
  int sval;

  /* Logic to prepare for a reduced power state goes here. */

  switch (pmstate)
    {
    case PM_NORMAL:
    case PM_IDLE:
      break;

    case PM_STANDBY:
    case PM_SLEEP:

      /* Check if exclusive lock for SPI bus is held. */

      if (nxsem_getvalue(&priv->exclsem, &sval) < 0)
        {
          DEBUGASSERT(false);
          return -EINVAL;
        }

      /* If exclusive lock is held, do not allow entry to deeper PM states */

      if (sval <= 0)
        {
          return -EBUSY;
        }

      break;

    default:

      /* Should not get here */

      break;
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: spi_slave_initialize
 *
 * Description:
 *   Initialize the selected SPI bus for slave operations
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_slave_initialize(struct stm32_spidev_s *priv)
{
  uint32_t setbits = 0;
  uint32_t clrbits = 0;
#ifdef CONFIG_PM
  int ret;
#endif

  /* Configure CR1, CFG1 and CFG2. Default configuration:
   *   Mode 0:                        CFG2.CPHA=0 and CFG2.CPOL=0
   *   Master:                        CFG2.MSTR=1
   *   8-bit:                         CFG1.DSIZE=7
   *   MSB transmitted first:         CFG2.LSBFRST=0
   *   Replace NSS with SSI & SSI=1:  CR1.SSI=1 CFG2.SSM=1 (prevent MODF err)
   *   Two lines full duplex:         CFG2.COMM=0
   */

  /* CR1 */

  clrbits = SPI_CR1_SPE;
  spi_modifyreg(priv, STM32_SPI_CR1_OFFSET, clrbits, setbits);

  /* CFG1 */

  clrbits = SPI_CFG1_DSIZE_MASK;
  setbits = SPI_CFG1_DSIZE_8BIT | SPI_CFG1_FTHLV_1DATA; /* REVISIT: FTHLV */
  spi_modifyreg(priv, STM32_SPI_CFG1_OFFSET, clrbits, setbits);

  /* CFG2 */

  clrbits = SPI_CFG2_CPHA | SPI_CFG2_CPOL | SPI_CFG2_LSBFRST |
    SPI_CFG2_COMM_MASK;
  setbits = SPI_CFG2_SSM;

  switch (priv->config)
    {
    default:
    case FULL_DUPLEX:
      setbits |= SPI_CFG2_COMM_FULL;
      break;
    case SIMPLEX_TX:
      setbits |= SPI_CFG2_COMM_STX;
      break;
    case SIMPLEX_RX:
      setbits |= SPI_CFG2_COMM_SRX;
      break;
    case HALF_DUPLEX:
      setbits |= SPI_CFG2_COMM_HALF;
      break;
    }

  spi_modifyreg(priv, STM32_SPI_CFG2_OFFSET, clrbits, setbits);

  priv->nbits     = 8;
  priv->mode      = SPIDEV_MODE0;

  /* CRCPOLY configuration */

  spi_putreg(priv, STM32_SPI_CRCPOLY_OFFSET, 7);

  /* Initialize the SPI semaphore that enforces mutually exclusive access. */

  nxsem_init(&priv->exclsem, 0, 1);

#ifdef CONFIG_STM32H7_SPI_DMA
  /* DMA will be started in the interrupt handler, synchronized to the master
   * nss
   */

  priv->dmarunning = false;

  /* Initialize the SPI semaphores that is used to wait for DMA completion.
   * This semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  nxsem_init(&priv->rxsem, 0, 0);
  nxsem_init(&priv->txsem, 0, 0);

  sem_setprotocol(&priv->rxsem, SEM_PRIO_NONE);
  sem_setprotocol(&priv->txsem, SEM_PRIO_NONE);

  if (priv->config != SIMPLEX_TX)
    {
      priv->rxdma = stm32_dmachannel(priv->rxch);
      DEBUGASSERT(priv->rxdma);
    }

  if (priv->config != SIMPLEX_RX)
    {
      priv->txdma = stm32_dmachannel(priv->txch);
      DEBUGASSERT(priv->txdma);
    }

#endif

#ifdef CONFIG_DEBUG_SPI_INFO
  /* Dump registers after initialization */

  spi_dumpregs(priv);
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_spi_slave_initialize
 *
 * Description:
 *   Initialize the selected SPI port(bus) to operate as spi slave
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

/* Helper macros to avoid duplicating all the code */

#define GPIO_SPI_SCK(x) GPIO_SPI##x##_SCK
#define GPIO_SPI_MISO(x) GPIO_SPI##x##_MISO
#define GPIO_SPI_MOSI(x) GPIO_SPI##x##_MOSI
#define GPIO_SPI_NSS(x) GPIO_SPI##x##_NSS

#define SPI_SLAVE_INIT_BUS(x) \
  priv = &g_spi##x##ctrlr;                                      \
                                                                \
  /* Only configure if the bus is not already configured */     \
                                                                \
  if (!priv->initialized)                                       \
    {                                                           \
      /* Configure SPI5 pins: SCK, MISO, MOSI and NSS */        \
                                                                \
      stm32_configgpio(GPIO_SPI_SCK(x));                        \
      stm32_configgpio(GPIO_SPI_MISO(x));                       \
      stm32_configgpio(GPIO_SPI_MOSI(x));                       \
      priv->nss_pin = GPIO_SPI_NSS(x);                          \
                                                                \
      /* Set up default configuration: 8-bit, etc. */           \
                                                                \
      spi_slave_initialize(priv);                               \
      priv->initialized = true;                                 \
    }

FAR struct spi_slave_ctrlr_s *stm32_spi_slave_initialize(int bus)
{
  FAR struct stm32_spidev_s *priv = NULL;
  irqstate_t flags = enter_critical_section();

#ifdef CONFIG_STM32H7_SPI1_SLAVE
  if (bus == 1)
    {
      SPI_SLAVE_INIT_BUS(1);
    }
  else
#endif

#ifdef CONFIG_STM32H7_SPI2_SLAVE
  if (bus == 2)
    {
      SPI_SLAVE_INIT_BUS(2);
    }
  else
#endif

#ifdef CONFIG_STM32H7_SPI3_SLAVE
  if (bus == 3)
    {
      SPI_SLAVE_INIT_BUS(3);
    }
  else
#endif

#ifdef CONFIG_STM32H7_SPI4_SLAVE
  if (bus == 4)
    {
      SPI_SLAVE_INIT_BUS(4);
    }
  else
#endif

#ifdef CONFIG_STM32H7_SPI5_SLAVE
  if (bus == 5)
    {
      SPI_SLAVE_INIT_BUS(5);
    }
  else
#endif

#ifdef CONFIG_STM32H7_SPI6_SLAVE
  if (bus == 6)
    {
      SPI_SLAVE_INIT_BUS(6);
    }
  else
#endif
    {
      spierr("ERROR: Unsupported SPI bus: %d\n", bus);
      return NULL;
    }

  /* Initialize the SPI operations */

  priv->ctrlr.ops = &g_ctrlr_ops;

  leave_critical_section(flags);
  return (FAR struct spi_slave_ctrlr_s *)priv;
}

#endif /* CONFIG_STM32H7_SPI1..6_SLAVE */
